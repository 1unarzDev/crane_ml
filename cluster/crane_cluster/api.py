"""Coordinator HTTP client (stdlib only)."""
from __future__ import annotations

import json
import time
import urllib.error
import urllib.parse
import urllib.request

from . import __version__


class ApiError(Exception):
    def __init__(self, status: int, body, retry_after: float | None = None):
        super().__init__(f"HTTP {status}: {body}")
        self.status = status
        self.body = body
        self.retry_after = retry_after


BACKOFF_START_S = 5.0
BACKOFF_CAP_S = 300.0


def unavailable(e: BaseException) -> str | None:
    """Why the coordinator counts as unavailable (a 5xx, 503 out of quota, or no connection), else None."""
    if isinstance(e, ApiError):
        if e.status < 500:
            return None
        msg = e.body.get("error") if isinstance(e.body, dict) else str(e.body)[:120]
        return f"HTTP {e.status}: {msg}"
    if isinstance(e, (urllib.error.URLError, TimeoutError, ConnectionError, OSError)):
        return f"unreachable: {getattr(e, 'reason', None) or e}"
    return None


class Backoff:
    """Exponential waits for an unavailable coordinator: 5 s doubling to a 5-min cap; a Retry-After can
    lengthen one step but never past the cap. reset() after a success."""

    def __init__(self, start: float = BACKOFF_START_S, cap: float = BACKOFF_CAP_S):
        self.start, self.cap, self.next = start, cap, start

    def step(self, e: BaseException | None = None) -> float:
        wait = self.next
        ra = getattr(e, "retry_after", None)
        if ra:
            wait = max(wait, float(ra))
        wait = min(wait, self.cap)
        self.next = min(self.next * 2, self.cap)
        return wait

    def reset(self) -> None:
        self.next = self.start


def wait_until_up(client: "Client", log=print, sleep=time.sleep, backoff: Backoff | None = None) -> None:
    """Blocks until GET /v1/ping answers, printing one line per wait with the reason."""
    b = backoff or Backoff()
    while True:
        try:
            client.request("GET", "/v1/ping", retries=0)
            return
        except Exception as e:  # noqa: BLE001 - classified below
            why = unavailable(e)
            if why is None:
                raise
            w = b.step(e)
            log(f"coordinator unavailable ({why}); waiting {w:.0f} s")
            sleep(w)


class Client:
    def __init__(self, url: str, token: str | None = None, timeout: float = 60.0, retries: int = 3):
        self.url = url.rstrip("/")
        self.token = token
        self.timeout = timeout
        self.retries = retries

    def request(self, method: str, path: str, body=None, data: bytes | None = None, headers: dict | None = None,
                retries: int | None = None):
        """JSON in, JSON out. Retries connection errors and 5xx with backoff; 4xx raise ApiError at once."""
        retries = self.retries if retries is None else retries
        hdrs = {"x-client-version": __version__, "user-agent": f"crane-cluster/{__version__}", **(headers or {})}
        if self.token:
            hdrs["authorization"] = f"Bearer {self.token}"
        if body is not None:
            data = json.dumps(body).encode()
            hdrs["content-type"] = "application/json"
        delay = 1.0
        for attempt in range(retries + 1):
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
                if e.code < 500 or attempt == retries:
                    ra = e.headers.get("retry-after") if e.headers else None
                    raise ApiError(e.code, payload, float(ra) if ra and ra.isdigit() else None) from None
            except (urllib.error.URLError, TimeoutError, ConnectionError):
                if attempt == retries:
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
