"""Coordinator HTTP client (stdlib only)."""
from __future__ import annotations

import json
import time
import urllib.error
import urllib.parse
import urllib.request

from . import __version__


class ApiError(Exception):
    def __init__(self, status: int, body):
        super().__init__(f"HTTP {status}: {body}")
        self.status = status
        self.body = body


class Client:
    def __init__(self, url: str, token: str | None = None, timeout: float = 60.0, retries: int = 3):
        self.url = url.rstrip("/")
        self.token = token
        self.timeout = timeout
        self.retries = retries

    def request(self, method: str, path: str, body=None, data: bytes | None = None, headers: dict | None = None):
        """JSON in, JSON out. Retries connection errors and 5xx with backoff; 4xx raise ApiError at once."""
        hdrs = {"x-client-version": __version__, "user-agent": f"crane-cluster/{__version__}", **(headers or {})}
        if self.token:
            hdrs["authorization"] = f"Bearer {self.token}"
        if body is not None:
            data = json.dumps(body).encode()
            hdrs["content-type"] = "application/json"
        delay = 1.0
        for attempt in range(self.retries + 1):
            req = urllib.request.Request(self.url + path, data=data, method=method, headers=hdrs)
            try:
                with urllib.request.urlopen(req, timeout=self.timeout) as r:
                    raw = r.read()
                    ctype = r.headers.get("content-type", "")
                    return json.loads(raw) if "json" in ctype else raw
            except urllib.error.HTTPError as e:
                raw = e.read()
                try:
                    payload = json.loads(raw)
                except ValueError:
                    payload = raw.decode(errors="replace")
                if e.code < 500 or attempt == self.retries:
                    raise ApiError(e.code, payload) from None
            except (urllib.error.URLError, TimeoutError, ConnectionError):
                if attempt == self.retries:
                    raise
            time.sleep(delay)
            delay = min(delay * 2, 30)

    def get(self, path, **q):
        qs = urllib.parse.urlencode({k: v for k, v in q.items() if v is not None})
        return self.request("GET", path + (f"?{qs}" if qs else ""))

    def post(self, path, body=None):
        return self.request("POST", path, body=body if body is not None else {})

    def put(self, path, body=None):
        return self.request("PUT", path, body=body if body is not None else {})

    def upload(self, path: str, data: bytes, sha256: str):
        return self.request("PUT", path, data=data, headers={"content-type": "application/octet-stream", "x-content-sha256": sha256})
