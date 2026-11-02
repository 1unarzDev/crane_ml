"""crane-archive: the cold tier of the crane cluster (design-storage.md section 3).

Outbound only: it pulls artifacts from the coordinator (GET /v1/archive/pending, then GET /v1/artifact/<key>),
stores them content-addressed (store.py), confirms them (POST /v1/archive/confirm) only after fsync, posts a
heartbeat with space, scrub and disk health, rehashes ``scrub.gb_per_day`` of objects a day, and applies the
quota policy. It serves objects to the coordinator through a Cloudflare Tunnel (server.py).

Config (``~/.config/crane-cluster/archive.yaml``)::

    coordinator: https://crane.lunarz.dev
    token_file: ~/.config/crane-cluster/archive.token   # POST /v1/admin/archive_token
    root: ~/crane-archive            # /tank/crane-archive once the drives arrive (crane-archive move)
    node: innovation                 # free text
    quota_bytes: 20G                 # a hard cap (plain directory); the pool size on ZFS is the other cap
    min_free_bytes: 25G              # never fill the disk below this
    listen: 127.0.0.1:7420
    pull: {interval_s: 300, batch: 200, min_age_h: 24}
    scrub: {gb_per_day: 200}
    access: {team_domain: lunarz.cloudflareaccess.com, aud: <application AUD tag>}
    smart: [/dev/sda, /dev/sdb]      # smartctl -j -H per device in the heartbeat (optional)

Quota (capacity = min(quota_bytes, stored + free - min_free_bytes)): above 80% the daemon evicts unprotected
objects, oldest first, in this order until under 75%: render chunks and masters older than 30 days, ``.crane``
recordings older than 180 days, raw items and run bundles older than 365 days. Trajectories, LODs, summaries,
render finals, pinned objects and the coordinator's protected set are never evicted. Above 90% pulls of
evictable kinds pause and the heartbeat says ``full``.
"""
from __future__ import annotations

import hashlib
import json
import os
import shutil
import subprocess
import sys
import threading
import time
import urllib.error
import urllib.parse
import urllib.request
from pathlib import Path

from .offsite import OFFSITE_DEFAULTS, Offsite
from .store import Store, StoreError

UA = 'crane-archive/0.1'
EVICT_ORDER = (('render_master', 30), ('crane', 180), ('raw', 365), ('bundle', 365))
NEVER_EVICT = {'ctraj', 'lod', 'summary', 'render_final'}
HIGH, LOW, CRITICAL = 0.80, 0.75, 0.90
DEFAULTS = {
    'coordinator': 'https://crane.lunarz.dev', 'root': '~/crane-archive', 'node': 'archive', 'quota_bytes': '20G',
    'min_free_bytes': '25G', 'listen': '127.0.0.1:7420', 'token_file': '~/.config/crane-cluster/archive.token',
    'pull': {'interval_s': 300, 'batch': 200, 'min_age_h': 24}, 'scrub': {'gb_per_day': 200}, 'smart': [],
    'offsite': dict(OFFSITE_DEFAULTS),
}


def config_path() -> Path:
    return Path(os.environ.get('XDG_CONFIG_HOME', Path.home() / '.config')) / 'crane-cluster' / 'archive.yaml'


def parse_bytes(v) -> int:
    if isinstance(v, (int, float)):
        return int(v)
    s = str(v).strip().upper().removesuffix('B').removesuffix('I')
    mult = {'K': 1 << 10, 'M': 1 << 20, 'G': 1 << 30, 'T': 1 << 40}
    return int(float(s[:-1]) * mult[s[-1]]) if s and s[-1] in mult else int(float(s))


def load_config(path=None) -> dict:
    path = Path(path) if path else config_path()
    cfg = json.loads(json.dumps(DEFAULTS))
    if path.is_file():
        try:
            from crane_cluster import miniyaml
            data = miniyaml.load_file(path) or {}
        except ImportError:  # pragma: no cover
            data = json.loads(path.read_text())
        for k, v in data.items():
            cfg[k] = {**cfg[k], **v} if isinstance(cfg.get(k), dict) and isinstance(v, dict) else v
    cfg['quota_bytes'] = parse_bytes(cfg['quota_bytes'])
    cfg['min_free_bytes'] = parse_bytes(cfg['min_free_bytes'])
    cfg['root'] = str(Path(str(cfg['root'])).expanduser())
    cfg['_path'] = str(path)
    return cfg


class Coordinator:
    """The few coordinator calls the archive makes (archive token)."""

    def __init__(self, url: str, token: str, timeout: float = 120):
        self.url, self.token, self.timeout = url.rstrip('/'), token, timeout

    def _req(self, method, path, body=None):
        data = json.dumps(body).encode() if body is not None else None
        req = urllib.request.Request(self.url + path, data=data, method=method, headers={
            'authorization': f'Bearer {self.token}', 'user-agent': UA, **({'content-type': 'application/json'} if data else {})})
        with urllib.request.urlopen(req, timeout=self.timeout) as r:
            return json.loads(r.read())

    def pending(self, limit, min_age_h):
        return self._req('GET', f'/v1/archive/pending?limit={int(limit)}&min_age_h={int(min_age_h)}')['items']

    def confirm(self, node, items):
        return self._req('POST', '/v1/archive/confirm', {'node': node, 'items': items})

    def heartbeat(self, body):
        return self._req('POST', '/v1/archive/heartbeat', body)

    def missing(self, node, items):
        return self._req('POST', '/v1/archive/missing', {'node': node, 'items': items})

    def protected(self) -> set:
        return set(self._req('GET', '/v1/archive/protected')['sha256'])

    def download(self, key: str, dest: Path) -> tuple:
        """Stream /v1/artifact/<key> to ``dest``; returns (sha256, bytes, the x-content-sha256 header)."""
        req = urllib.request.Request(f'{self.url}/v1/artifact/{urllib.parse.quote(key)}',
                                     headers={'authorization': f'Bearer {self.token}', 'user-agent': UA})
        h, n = hashlib.sha256(), 0
        with urllib.request.urlopen(req, timeout=self.timeout) as r, open(dest, 'wb') as f:
            want = r.headers.get('x-content-sha256')
            while True:
                b = r.read(1 << 20)
                if not b:
                    break
                h.update(b)
                f.write(b)
                n += len(b)
            f.flush()
            os.fsync(f.fileno())
        return h.hexdigest(), n, want


class Daemon:
    def __init__(self, cfg: dict, store: Store, coord: Coordinator | None, *, log=print):
        self.cfg, self.store, self.coord, self.log = cfg, store, coord, log
        self.node = str(cfg['node'])
        self.failures: dict = {}          # key -> (count, next_try)
        self.last = {'pull': None, 'scrub': None, 'evict': None, 'offsite': None, 'error': None}
        self.protected: set = set()
        self.stop = threading.Event()
        self.offsite = Offsite(cfg, store, log=log)
        self.last_health: dict = {}

    # ------------------------------------------------------------------ space
    def space(self) -> dict:
        st = self.store.stats()
        disk = self.store.disk()
        cap = min(self.cfg['quota_bytes'], st['disk_bytes'] + max(0, disk['free'] - self.cfg['min_free_bytes']))
        used = st['disk_bytes']
        return {'capacity': cap, 'used': used, 'frac': used / cap if cap > 0 else 1.0, 'disk': disk, 'stats': st}

    # ------------------------------------------------------------------ pull
    def pull_once(self) -> dict:
        p = self.cfg['pull']
        out = {'stored': 0, 'dedup': 0, 'skipped': 0, 'failed': 0, 'bytes': 0}
        items = self.coord.pending(int(p['batch']) * 2, p['min_age_h'])
        now = time.time()
        confirms = []
        sp = self.space()
        for it in items:
            if len(confirms) + out['failed'] >= int(p['batch']):
                break
            key, sha, kind = it['key'], it['sha256'], it.get('kind') or 'raw'
            n, nxt = self.failures.get(key, (0, 0))
            if nxt > now:
                out['skipped'] += 1
                continue
            have = self.store.get(sha)
            if not have:
                evictable = kind not in NEVER_EVICT and not it.get('protected')
                room = sp['capacity'] * CRITICAL - sp['used']
                if (evictable and sp['frac'] >= CRITICAL) or int(it.get('size') or 0) > room:
                    out['skipped'] += 1
                    continue
                tmp = self.store.tmpfile('.dl')
                try:
                    got, size, header = self.coord.download(key, tmp)
                    if got != sha or (header and header != sha):
                        raise StoreError(f'{key}: downloaded sha256 {got} differs from the coordinator\'s {sha}')
                    have = self.store.put_file(tmp, sha256=sha, kind=kind, name=key)
                    out['stored'] += 1
                    out['bytes'] += have.length
                    sp['used'] += have.length
                    sp['frac'] = sp['used'] / sp['capacity'] if sp['capacity'] > 0 else 1.0
                except (OSError, urllib.error.URLError, StoreError, ValueError) as e:
                    self.failures[key] = (n + 1, now + min(86400, 300 * 2 ** n))
                    out['failed'] += 1
                    self.log(f'pull {key}: {e}')
                    continue
                finally:
                    tmp.unlink(missing_ok=True)
            else:
                out['dedup'] += 1
            self.failures.pop(key, None)
            self.store.remember_key(key, sha, kind, it.get('created'))
            codec = have.codec if not have.packed else 'pack' if have.codec == 'none' else f'pack+{have.codec}'
            confirms.append({'key': key, 'sha256': sha, 'archive_bytes': have.length, 'codec': codec})
        for i in range(0, len(confirms), 500):
            self.coord.confirm(self.node, confirms[i:i + 500])
        self.last['pull'] = {'t': time.time(), **out}
        return out

    # ------------------------------------------------------------------ quota
    def evict(self, protected: set | None = None, now: float | None = None) -> dict:
        """The quota policy; returns {evicted, bytes}. Reports evictions to the coordinator as missing."""
        now = now or time.time()
        protected = self.protected if protected is None else protected
        sp = self.space()
        out = {'evicted': 0, 'bytes': 0}
        if sp['frac'] < HIGH:
            return out
        target = sp['capacity'] * LOW
        used = sp['used']
        gone = []
        for kind, days in EVICT_ORDER:
            if used <= target:
                break
            rows = self.store.db.execute(
                "SELECT sha256 FROM objects WHERE kind = ? AND pinned = 0 AND first_seen < ? AND path LIKE 'objects/%' "
                'ORDER BY first_seen', (kind, now - days * 86400)).fetchall()
            for (sha,) in rows:
                if used <= target:
                    break
                if sha in protected:
                    continue
                o = self.store.get(sha)
                self.store.remove(o, f'quota {sp["frac"]:.0%}')
                used -= o.length
                out['evicted'] += 1
                out['bytes'] += o.length
                gone.append({'sha256': sha, 'reason': 'evicted'})
        if gone and self.coord:
            for i in range(0, len(gone), 500):
                self.coord.missing(self.node, gone[i:i + 500])
        self.last['evict'] = {'t': now, **out}
        return out

    # ------------------------------------------------------------------ scrub
    def scrub(self, budget_bytes: int) -> dict:
        r = self.store.scrub(budget_bytes)
        if r['bad'] and self.coord:
            self.coord.missing(self.node, [{'sha256': s, 'reason': 'corrupt'} for s in r['bad']])
        self.last['scrub'] = {'t': time.time(), 'checked': r['checked'], 'bytes': r['bytes'], 'bad': len(r['bad'])}
        return r

    # ------------------------------------------------------------------ health
    def health(self) -> dict:
        sp = self.space()
        smart = [smart_health(d) for d in self.cfg.get('smart') or []]
        pool = zpool_health() if sp['disk']['fs'] == 'zfs' else None
        bad_disk = any(s.get('passed') is False for s in smart) or (pool is not None and pool.get('state') not in ('ok', None))
        state = 'full' if sp['frac'] >= CRITICAL else 'degraded' if bad_disk else 'ok'
        st = sp['stats']
        return {'node': self.node, 'root_kind': sp['disk']['fs'], 'total': sp['capacity'], 'used': sp['used'],
                'free': max(0, sp['capacity'] - sp['used']), 'disk_free': sp['disk']['free'], 'objects': st['objects'],
                'original_bytes': st['original_bytes'], 'kinds': st['kinds'], 'state': state, 'smart': smart,
                'pool': pool, 'last': self.last, 'offsite': self.offsite.status()}

    def heartbeat(self) -> dict:
        h = self.health()
        self.last_health = h
        r = self.coord.heartbeat(h) if self.coord else {}
        return {**h, 'coordinator': r}

    # ------------------------------------------------------------------ loop
    def run(self) -> None:
        interval = float(self.cfg['pull']['interval_s'])
        scrub_per_s = float(self.cfg['scrub']['gb_per_day']) * 1e9 / 86400
        last_scrub = time.time()
        last_protected = 0.0
        while not self.stop.is_set():
            try:
                if time.time() - last_protected > 3600:
                    self.protected = self.coord.protected()
                    last_protected = time.time()
                self.pull_once()
                self.evict()
                now = time.time()
                self.scrub(int((now - last_scrub) * scrub_per_s))
                last_scrub = now
                self.store.clean_tmp()
                if self.offsite.enabled and self.offsite.due():     # the daily timer missed: hold the mirror policy
                    self.last['offsite'] = {'t': time.time(), **self.offsite.push()}
                self.heartbeat()
                self.last['error'] = None
            except Exception as e:  # noqa: BLE001 - the loop must survive any one failure
                self.last['error'] = {'t': time.time(), 'error': str(e)[:300]}
                self.log(f'loop: {e}')
            self.stop.wait(interval)


def smart_health(dev: str) -> dict:
    if not shutil.which('smartctl'):
        return {'device': dev, 'error': 'smartctl not installed'}
    try:
        p = subprocess.run(['smartctl', '-j', '-H', '-A', dev], capture_output=True, text=True, timeout=60)
        j = json.loads(p.stdout or '{}')
    except (OSError, ValueError, subprocess.TimeoutExpired) as e:
        return {'device': dev, 'error': str(e)[:200]}
    out = {'device': dev, 'passed': (j.get('smart_status') or {}).get('passed'),
           'temp_c': (j.get('temperature') or {}).get('current'),
           'power_on_h': (j.get('power_on_time') or {}).get('hours')}
    for a in (j.get('ata_smart_attributes') or {}).get('table', []):
        if a.get('id') in (5, 187, 197, 198):
            out[f'attr_{a["id"]}'] = (a.get('raw') or {}).get('value')
    scsi = j.get('scsi_grown_defect_list')
    if scsi is not None:
        out['grown_defects'] = scsi
    return out


def zpool_health() -> dict:
    if not shutil.which('zpool'):
        return {'state': None, 'error': 'zpool not installed'}
    try:
        p = subprocess.run(['zpool', 'status', '-x'], capture_output=True, text=True, timeout=30)
    except (OSError, subprocess.TimeoutExpired) as e:
        return {'state': None, 'error': str(e)[:200]}
    healthy = 'all pools are healthy' in p.stdout
    return {'state': 'ok' if healthy else 'degraded', 'detail': None if healthy else p.stdout[-800:]}


def read_token(cfg: dict) -> str:
    env = os.environ.get('CRANE_ARCHIVE_TOKEN')
    if env:
        return env
    p = Path(str(cfg['token_file'])).expanduser()
    if not p.is_file():
        sys.exit(f'crane-archive: no token at {p} (admin: POST /v1/admin/archive_token, then save it there, mode 600)')
    return p.read_text().strip()
