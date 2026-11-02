"""crane-archive: the store (dedup, packs, codecs, scrub, move), the origin (Range, Access JWT) and the
daemon's pull, quota and scrub reports against a fake coordinator."""
import base64
import hashlib
import json
import os
import random
import threading
import time
import urllib.error
import urllib.request

import pytest

from crane_archive import daemon as D
from crane_archive.server import AccessVerifier, make_server
from crane_archive.store import PACK_MAX, Store, StoreError


def text_blob(n, seed=1):
    rng = random.Random(seed)
    words = [''.join(rng.choice('abcdefghij') for _ in range(rng.randint(2, 8))) for _ in range(200)]
    out = []
    while sum(map(len, out)) < n:
        out.append(json.dumps({'t': round(rng.random(), 3), 'w': rng.choice(words)}) + '\n')
    return ''.join(out).encode()[:n]


def noise(n, seed=2):
    return random.Random(seed).randbytes(n)


@pytest.fixture
def store(tmp_path):
    return Store(tmp_path / 'arch', node='t', create=True)


def test_dedup_packs_codecs_and_reads(store, tmp_path):
    small, big, video = text_blob(5_000), text_blob(400_000, 3) + noise(100_000, 4), noise(300_000)
    a = store.put_bytes(small, kind='summary')
    assert a.packed and store.put_bytes(small, kind='raw').sha256 == a.sha256
    assert store.stats()['objects'] == 1
    b = store.put_bytes(big, kind='raw', name='player.log')
    assert not b.packed and b.codec in ('zstd19', 'xz') and b.length < len(big) / 2
    assert b.path.startswith('objects/') and (store.root / b.path).is_file()
    v = store.put_bytes(video, kind='render_final', name='final.mp4')
    assert v.codec == 'none' and v.length == len(video)
    c = store.put_bytes(noise(1000, 9), kind='ctraj')
    assert c.codec == 'none' and c.packed
    for data, o in ((small, a), (big, b), (video, v)):
        assert store.read_bytes(o.sha256) == data and store.verify(o)
    with pytest.raises(StoreError):
        store.put_bytes(b'x' * 10, sha256='0' * 64)
    with pytest.raises(StoreError):
        Store(tmp_path / 'other')
    (tmp_path / 'full').mkdir()
    (tmp_path / 'full' / 'f').write_text('x')
    with pytest.raises(StoreError):
        Store(tmp_path / 'full', create=True)


def test_scrub_quarantines_a_corrupt_copy_and_move_verifies(store, tmp_path):
    objs = [store.put_bytes(noise(PACK_MAX * 2, i), kind='raw', name='x.bin') for i in range(3)]
    p = store.root / objs[1].path
    raw = bytearray(p.read_bytes())
    raw[100] ^= 0xFF
    p.write_bytes(bytes(raw))
    r = store.scrub(1 << 40)
    assert r['bad'] == [objs[1].sha256] and r['checked'] == 3
    assert store.get(objs[1].sha256) is None and any((store.root / 'quarantine').iterdir())
    new = store.move_to(tmp_path / 'moved')
    assert new.stats()['objects'] == 2 and new.read_bytes(objs[0].sha256) == noise(PACK_MAX * 2, 0)
    assert store.stats()['objects'] == 2   # the old root is untouched


# ---------------------------------------------------------------- origin

def _rsa_keypair(bits=1024):
    def prime(rng):
        while True:
            n = rng.getrandbits(bits // 2) | (1 << (bits // 2 - 1)) | 1
            if all(pow(a, n - 1, n) == 1 for a in (2, 3, 5, 7, 11, 13, 17, 19, 23, 29, 31, 37)):
                return n
    rng = random.Random(42)
    e = 65537
    while True:
        p, q = prime(rng), prime(rng)
        phi = (p - 1) * (q - 1)
        if p != q and phi % e:
            return p * q, e, pow(e, -1, phi)


def _b64(b):
    return base64.urlsafe_b64encode(b).rstrip(b'=').decode()


def _jwt(n, d, claims, kid='k1'):
    k = (n.bit_length() + 7) // 8
    h64 = _b64(json.dumps({'alg': 'RS256', 'kid': kid}).encode())
    p64 = _b64(json.dumps(claims).encode())
    t = bytes.fromhex('3031300d060960864801650304020105000420') + hashlib.sha256(f'{h64}.{p64}'.encode()).digest()
    em = b'\x00\x01' + b'\xff' * (k - len(t) - 3) + b'\x00' + t
    sig = pow(int.from_bytes(em, 'big'), d, n).to_bytes(k, 'big')
    return f'{h64}.{p64}.{_b64(sig)}'


def _get(url, headers=None, method='GET'):
    req = urllib.request.Request(url, headers=headers or {}, method=method)
    try:
        with urllib.request.urlopen(req, timeout=10) as r:
            return r.status, dict(r.headers), r.read()
    except urllib.error.HTTPError as e:
        return e.code, dict(e.headers), e.read()


@pytest.fixture
def origin(store):
    servers = []

    def start(verifier=None):
        srv = make_server(store, '127.0.0.1:0', verifier=verifier, health=lambda: {'state': 'ok'})
        threading.Thread(target=srv.serve_forever, daemon=True).start()
        servers.append(srv)
        return f'http://127.0.0.1:{srv.server_address[1]}'
    yield start
    for s in servers:
        s.shutdown()


def test_origin_serves_ranges_of_plain_and_compressed_objects(store, origin):
    big, vid = text_blob(300_000, 5), noise(200_000, 6)
    b = store.put_bytes(big, kind='raw', name='a.log')
    v = store.put_bytes(vid, kind='render_final', name='v.mp4')
    url = origin()
    st, h, body = _get(f'{url}/o/{v.sha256}', {'range': 'bytes=1000-1999'})
    assert st == 206 and body == vid[1000:2000] and h['content-range'] == f'bytes 1000-1999/{len(vid)}'
    st, h, body = _get(f'{url}/o/{b.sha256}', {'range': 'bytes=-500'})
    assert st == 206 and body == big[-500:]
    st, h, body = _get(f'{url}/o/{b.sha256}')
    assert st == 200 and body == big and h['x-content-sha256'] == b.sha256
    assert _get(f'{url}/o/{b.sha256}', method='HEAD')[1]['content-length'] == str(len(big))
    assert _get(f'{url}/o/{"0" * 64}')[0] == 404
    assert _get(f'{url}/o/{b.sha256}', {'range': 'bytes=999999999-'})[0] == 416
    assert _get(f'{url}/o/{b.sha256}', {'cf-ray': 'abc'})[0] == 403   # through Cloudflare without access config
    assert json.loads(_get(f'{url}/health')[2]) == {'state': 'ok'}


def test_origin_checks_the_access_jwt(store, origin):
    n, e, d = _rsa_keypair()
    certs = {'keys': [{'kty': 'RSA', 'kid': 'k1', 'n': _b64(n.to_bytes(128, 'big')), 'e': _b64(e.to_bytes(3, 'big'))}]}
    ver = AccessVerifier('team.cloudflareaccess.com', 'AUD1', fetch=lambda: certs)
    good = {'aud': ['AUD1'], 'exp': time.time() + 600, 'iss': 'https://team.cloudflareaccess.com'}
    assert ver.check(_jwt(n, d, good)) is None
    assert ver.check(_jwt(n, d, {**good, 'aud': ['other']})) == 'wrong audience'
    assert ver.check(_jwt(n, d, {**good, 'exp': time.time() - 3600})) == 'expired'
    tok = _jwt(n, d, good)
    assert ver.check(tok[:-4] + ('AAAA' if not tok.endswith('AAAA') else 'BBBB')) == 'bad signature'
    o = store.put_bytes(b'hello' * 100, kind='summary')
    url = origin(ver)
    assert _get(f'{url}/o/{o.sha256}')[0] == 403
    st, _, body = _get(f'{url}/o/{o.sha256}', {'cf-access-jwt-assertion': tok})
    assert st == 200 and body == b'hello' * 100


# ---------------------------------------------------------------- daemon

class FakeCoord:
    def __init__(self, objects):
        self.objects = objects      # key -> (bytes, kind)
        self.confirmed, self.missing_items, self.beats = [], [], []
        self.bad_key = None

    def pending(self, limit, min_age_h):
        done = {c['key'] for c in self.confirmed}
        return [{'key': k, 'sha256': hashlib.sha256(b).hexdigest(), 'size': len(b), 'kind': kind, 'protected': False,
                 'created': '2026-01-01T00:00:00Z'} for k, (b, kind) in self.objects.items() if k not in done][:limit]

    def download(self, key, dest):
        data = self.objects[key][0]
        if key == self.bad_key:
            data = data + b'!'
        dest.write_bytes(data)
        return hashlib.sha256(data).hexdigest(), len(data), None

    def confirm(self, node, items):
        self.confirmed += items
        return {'confirmed': len(items), 'refused': []}

    def missing(self, node, items):
        self.missing_items += items
        return {}

    def heartbeat(self, body):
        self.beats.append(body)
        return {'healthy': True}

    def protected(self):
        return set()


def cfg_for(store, **kw):
    cfg = json.loads(json.dumps(D.DEFAULTS))
    cfg.update({'root': str(store.root), 'node': 't', 'quota_bytes': 10 << 30, 'min_free_bytes': 0, **kw})
    return cfg


def test_pull_stores_dedups_refuses_bad_bytes_and_confirms(store):
    same = text_blob(80_000, 7)
    coord = FakeCoord({'artifacts/j/r1/a.log': (same, 'raw'), 'artifacts/j/r2/a.log': (same, 'raw'),
                       'artifacts/j/r1/run.ctraj': (noise(3000), 'ctraj'), 'artifacts/j/r1/bad.bin': (noise(10), 'raw')})
    coord.bad_key = 'artifacts/j/r1/bad.bin'
    d = D.Daemon(cfg_for(store), store, coord, log=lambda m: None)
    r = d.pull_once()
    assert r['stored'] == 2 and r['dedup'] == 1 and r['failed'] == 1
    assert {c['key'] for c in coord.confirmed} == {'artifacts/j/r1/a.log', 'artifacts/j/r2/a.log', 'artifacts/j/r1/run.ctraj'}
    traj = next(c for c in coord.confirmed if c['key'].endswith('ctraj'))
    assert traj['codec'] == 'pack'
    assert store.stats()['objects'] == 2
    assert d.pull_once()['skipped'] == 1     # the bad one backs off
    d.heartbeat()
    beat = coord.beats[-1]
    assert beat['objects'] == 2 and beat['state'] == 'ok' and beat['node'] == 't'


def test_quota_evicts_old_evictable_kinds_in_order_and_never_trajectories(store):
    old = time.time() - 400 * 86400
    keep_traj = store.put_bytes(noise(200_000, 1), kind='ctraj')
    master = store.put_bytes(noise(200_000, 2), kind='render_master', name='m.mp4')
    crane = store.put_bytes(noise(200_000, 3), kind='crane')
    prot = store.put_bytes(noise(200_000, 4), kind='render_master', name='p.mp4')
    store.db.execute('UPDATE objects SET first_seen = ?', (old,))
    coord = FakeCoord({})
    used = store.stats()['disk_bytes']
    d = D.Daemon(cfg_for(store, quota_bytes=int(used / 0.95)), store, coord, log=lambda m: None)
    r = d.evict(protected={prot.sha256})
    assert r['evicted'] >= 1
    assert store.get(master.sha256) is None and store.get(keep_traj.sha256) and store.get(prot.sha256)
    assert {'sha256': master.sha256, 'reason': 'evicted'} in coord.missing_items
    assert store.get(crane.sha256) is not None and r['evicted'] == 1   # under 75% after the first class


def test_scrub_reports_corrupt_objects_as_missing(store):
    o = store.put_bytes(noise(PACK_MAX * 2, 11), kind='raw', name='x.bin')
    p = store.root / o.path
    p.write_bytes(b'\0' + p.read_bytes()[1:])
    coord = FakeCoord({})
    D.Daemon(cfg_for(store), store, coord).scrub(1 << 40)
    assert coord.missing_items == [{'sha256': o.sha256, 'reason': 'corrupt'}]


def test_config_reads_sizes_and_merges_defaults(tmp_path):
    p = tmp_path / 'archive.yaml'
    p.write_text('root: ~/x\nquota_bytes: 1.5T\npull: {batch: 7}\n')
    c = D.load_config(p)
    assert c['quota_bytes'] == int(1.5 * (1 << 40)) and c['pull'] == {'interval_s': 300, 'batch': 7, 'min_age_h': 24}
    assert c['root'] == os.path.expanduser('~/x') and c['min_free_bytes'] == 25 << 30
