"""The archive's HTTP origin, served only through a Cloudflare Tunnel (bound to loopback).

    GET  /o/<sha256>   the original bytes (Range: one byte range); x-content-sha256 on every answer
    HEAD /o/<sha256>   existence and size
    GET  /health       the last heartbeat (state, space, scrub)

Cloudflare Access in front of the tunnel lets only the coordinator's service token through. As defense in
depth the server checks the ``Cf-Access-Jwt-Assertion`` header itself (RS256 against the team's JWKS, the
application's AUD tag, expiry) when ``access`` is configured. Without ``access`` it answers only requests
that did not come through Cloudflare (no ``cf-ray``), i.e. local ones.
"""
from __future__ import annotations

import base64
import hashlib
import json
import re
import threading
import time
import urllib.request
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

from .store import CHUNK, Store

SHA_PATH = re.compile(r'^/o/([0-9a-f]{64})$')
RANGE = re.compile(r'^bytes=(\d*)-(\d*)$')

# DigestInfo prefix for SHA-256 (RFC 8017 9.2 note 1)
_SHA256_PREFIX = bytes.fromhex('3031300d060960864801650304020105000420')


def _b64url(s: str) -> bytes:
    return base64.urlsafe_b64decode(s + '=' * (-len(s) % 4))


def rs256_verify(n: int, e: int, signing_input: bytes, sig: bytes) -> bool:
    """RSASSA-PKCS1-v1_5 with SHA-256, stdlib only."""
    k = (n.bit_length() + 7) // 8
    if len(sig) != k:
        return False
    em = pow(int.from_bytes(sig, 'big'), e, n).to_bytes(k, 'big')
    t = _SHA256_PREFIX + hashlib.sha256(signing_input).digest()
    want = b'\x00\x01' + b'\xff' * (k - len(t) - 3) + b'\x00' + t
    return len(want) == k and hashlib.sha256(em).digest() == hashlib.sha256(want).digest()


class AccessVerifier:
    """Validates Cloudflare Access JWTs: ``team_domain`` (e.g. lunarz.cloudflareaccess.com) and ``aud``."""

    def __init__(self, team_domain: str, aud: str, *, fetch=None, ttl_s: float = 3600):
        self.team = team_domain.removeprefix('https://').rstrip('/')
        self.aud = aud
        self.ttl_s = ttl_s
        self._fetch = fetch or self._http_certs
        self._keys: dict = {}
        self._at = 0.0
        self._lock = threading.Lock()

    def _http_certs(self) -> dict:
        req = urllib.request.Request(f'https://{self.team}/cdn-cgi/access/certs', headers={'user-agent': 'crane-archive'})
        with urllib.request.urlopen(req, timeout=10) as r:
            return json.loads(r.read())

    def keys(self, refresh: bool = False) -> dict:
        with self._lock:
            if refresh or not self._keys or time.time() - self._at > self.ttl_s:
                certs = self._fetch()
                self._keys = {k['kid']: (int.from_bytes(_b64url(k['n']), 'big'), int.from_bytes(_b64url(k['e']), 'big'))
                              for k in certs.get('keys', []) if k.get('kty') == 'RSA'}
                self._at = time.time()
            return self._keys

    def check(self, token: str | None, now: float | None = None) -> str | None:
        """None when valid, else the reason."""
        if not token:
            return 'no access token'
        try:
            h64, p64, s64 = token.split('.')
            header, claims = json.loads(_b64url(h64)), json.loads(_b64url(p64))
            sig = _b64url(s64)
        except (ValueError, TypeError):
            return 'malformed token'
        if header.get('alg') != 'RS256':
            return 'bad alg'
        key = self.keys().get(header.get('kid')) or self.keys(refresh=True).get(header.get('kid'))
        if not key or not rs256_verify(key[0], key[1], f'{h64}.{p64}'.encode(), sig):
            return 'bad signature'
        aud = claims.get('aud')
        if self.aud not in (aud if isinstance(aud, list) else [aud]):
            return 'wrong audience'
        now = now or time.time()
        if not isinstance(claims.get('exp'), (int, float)) or claims['exp'] < now - 60:
            return 'expired'
        if claims.get('iss') not in (f'https://{self.team}', None):
            return 'wrong issuer'
        return None


def make_server(store: Store, listen: str, *, verifier: AccessVerifier | None = None, health=lambda: {}):
    host, _, port = listen.rpartition(':')
    local = threading.local()

    def db():
        if getattr(local, 'db', None) is None:
            local.db = store.connect()
        return local.db

    class Handler(BaseHTTPRequestHandler):
        server_version = 'crane-archive'
        protocol_version = 'HTTP/1.1'

        def log_message(self, fmt, *args):  # quiet; the journal gets errors only
            pass

        def _send_json(self, code: int, body: dict, extra: dict | None = None):
            data = json.dumps(body).encode()
            self.send_response(code)
            self.send_header('content-type', 'application/json')
            self.send_header('content-length', str(len(data)))
            for k, v in (extra or {}).items():
                self.send_header(k, v)
            self.end_headers()
            if self.command != 'HEAD':
                self.wfile.write(data)

        def _authorized(self) -> bool:
            if verifier is not None:
                why = verifier.check(self.headers.get('cf-access-jwt-assertion'))
                if why:
                    self._send_json(403, {'error': 'forbidden'})
                    return False
                return True
            if self.headers.get('cf-ray') or self.headers.get('cf-connecting-ip'):
                self._send_json(403, {'error': 'forbidden (no access configured: local requests only)'})
                return False
            return True

        def do_HEAD(self):
            self.do_GET()

        def do_GET(self):
            if not self._authorized():
                return
            if self.path == '/health':
                return self._send_json(200, health())
            m = SHA_PATH.match(self.path.split('?', 1)[0])
            if not m:
                return self._send_json(404, {'error': 'not found'})
            o = store.get(m.group(1), db())
            if not o:
                return self._send_json(404, {'error': 'not found'})
            start, end = 0, o.size - 1
            rng = self.headers.get('range')
            status = 200
            if rng:
                r = RANGE.match(rng.strip())
                if not r or (not r.group(1) and not r.group(2)):
                    return self._send_json(416, {'error': 'one byte range only'}, {'content-range': f'bytes */{o.size}'})
                if r.group(1):
                    start = int(r.group(1))
                    end = min(int(r.group(2)), o.size - 1) if r.group(2) else o.size - 1
                else:
                    start = max(0, o.size - int(r.group(2)))
                if start > end or start >= o.size:
                    return self._send_json(416, {'error': 'range not satisfiable'}, {'content-range': f'bytes */{o.size}'})
                status = 206
            self.send_response(status)
            self.send_header('content-type', 'application/octet-stream')
            self.send_header('content-length', str(end - start + 1))
            self.send_header('accept-ranges', 'bytes')
            self.send_header('x-content-sha256', o.sha256)
            self.send_header('cache-control', 'private, max-age=31536000, immutable')
            if status == 206:
                self.send_header('content-range', f'bytes {start}-{end}/{o.size}')
            self.end_headers()
            if self.command == 'HEAD':
                return
            try:
                with store.reader(o) as src:
                    _skip(src, start)
                    left = end - start + 1
                    while left > 0:
                        b = src.read(min(CHUNK, left))
                        if not b:
                            break
                        self.wfile.write(b)
                        left -= len(b)
            except (BrokenPipeError, ConnectionResetError):
                pass

    srv = ThreadingHTTPServer((host or '127.0.0.1', int(port)), Handler)
    srv.daemon_threads = True
    return srv


def _skip(src, n: int) -> None:
    """Advance a stream by n bytes (a seek on stored-as-is objects, decompress-and-drop on compressed ones)."""
    if n <= 0:
        return
    try:
        if src.seekable():
            src.seek(n, 1)
            return
    except (OSError, ValueError):
        pass
    while n > 0:
        b = src.read(min(CHUNK, n))
        if not b:
            return
        n -= len(b)
