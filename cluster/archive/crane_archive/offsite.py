"""Off-site cold copy of the archive on an rclone remote (a Google Shared Drive today: ``drive:``).

Objects are packed into content-addressed bundles (tar, zstd; one member per object, named by its sha256, holding
the original bytes). A bundle is named ``YYYYMMDD-<sha256 of its sorted member hashes>.tar.zst``, uploaded with
``rclone copy``, verified (size and md5 of what the remote holds, else a download compare), and only then recorded
in the index (tables ``bundles`` and ``copies``). A JSON manifest beside each bundle lets the index be rebuilt
from the remote alone.

    <root>/bundles/<name>.tar.zst
    <root>/manifests/<name>.json

Config (``offsite:`` in archive.yaml; empty ``remote`` turns it off)::

    offsite: {remote: 'drive:', root: crane-archive, bundle_bytes: 1.5G, bandwidth: '', transfers: 4,
              chunk_size: 64M, retries: 5, mirror_within_h: 24}

rclone's own config holds the credentials; nothing here reads or prints it.
"""
from __future__ import annotations

import fcntl
import hashlib
import json
import lzma
import os
import shutil
import subprocess
import tarfile
import time
from pathlib import Path

from .store import CHUNK, ZSTD, Store, StoreError, _zstd_reader, zstd_writer

OFFSITE_DEFAULTS = {'remote': '', 'root': 'crane-archive', 'bundle_bytes': '1.5G', 'bandwidth': '', 'transfers': 4,
                    'chunk_size': '64M', 'retries': 5, 'mirror_within_h': 24, 'rclone': 'rclone'}


class OffsiteError(StoreError):
    pass


def _md5_file(path) -> tuple:
    m, s, n = hashlib.md5(usedforsecurity=False), hashlib.sha256(), 0
    with open(path, 'rb') as f:
        while True:
            b = f.read(CHUNK)
            if not b:
                break
            m.update(b)
            s.update(b)
            n += len(b)
    return m.hexdigest(), s.hexdigest(), n


class Offsite:
    def __init__(self, cfg: dict, store: Store, *, log=print):
        self.o = {**OFFSITE_DEFAULTS, **(cfg.get('offsite') or {})}
        self.store, self.log = store, log
        self.bundle_bytes = _parse_bytes(self.o['bundle_bytes'])

    @property
    def enabled(self) -> bool:
        return bool(str(self.o['remote']).strip())

    def path(self, *parts) -> str:
        """``remote:root/part/part`` (the remote name carries its colon, or is a bare name)."""
        remote = str(self.o['remote'])
        if ':' not in remote:
            remote += ':'
        tail = '/'.join([str(self.o['root']).rstrip('/'), *parts])
        return remote + tail

    # ------------------------------------------------------------------ rclone
    def rclone(self, *args, transfer=False, check=True) -> subprocess.CompletedProcess:
        cmd = [str(self.o['rclone']), *args, '--retries', str(self.o['retries']), '--low-level-retries', '10']
        if transfer:
            cmd += ['--transfers', str(self.o['transfers']), '--drive-chunk-size', str(self.o['chunk_size']),
                    '--drive-upload-cutoff', str(self.o['chunk_size'])]
            if str(self.o['bandwidth']).strip():
                cmd += ['--bwlimit', str(self.o['bandwidth'])]
        try:
            p = subprocess.run(cmd, capture_output=True, text=True)
        except OSError as e:
            raise OffsiteError(f'rclone: {e}') from e
        if check and p.returncode != 0:
            # the first args name the operation; stderr's tail is enough and carries no config
            raise OffsiteError(f'rclone {args[0]} exited {p.returncode}: {p.stderr.strip()[-300:]}')
        return p

    # ------------------------------------------------------------------ state
    def pending(self, db=None) -> list:
        """(sha256, size, kind, first_seen) of objects with no drive copy yet, oldest first."""
        return (db or self.store.db).execute(
            'SELECT sha256, size, kind, first_seen FROM objects WHERE sha256 NOT IN (SELECT sha256 FROM copies) '
            'ORDER BY first_seen, sha256').fetchall()

    def status(self) -> dict:
        if not self.enabled:
            return {'enabled': False}
        pend = self.pending()
        n, b, last = self.store.db.execute('SELECT COUNT(*), COALESCE(SUM(bytes), 0), MAX(verified_at) FROM bundles').fetchone()
        age_h = (time.time() - min(r[3] for r in pend)) / 3600 if pend else 0.0
        return {'enabled': True, 'remote': str(self.o['remote']), 'bundles': n, 'bundle_bytes': b, 'last_push': last,
                'pending_objects': len(pend), 'pending_bytes': sum(r[1] for r in pend), 'oldest_pending_h': round(age_h, 1),
                'behind': age_h > float(self.o['mirror_within_h'])}

    def due(self) -> bool:
        """True when the mirror policy is at risk: the oldest unmirrored object is within 2 h of the limit."""
        pend = self.pending()
        return bool(pend) and (time.time() - min(r[3] for r in pend)) / 3600 > float(self.o['mirror_within_h']) - 2

    # ------------------------------------------------------------------ push
    def push(self, *, limit_bundles: int | None = None) -> dict:
        """Bundle, upload and verify every object that has no drive copy. One push at a time (a file lock)."""
        out = {'bundles': 0, 'objects': 0, 'bytes': 0, 'failed': 0, 'error': None}
        if not self.enabled:
            out['error'] = 'offsite is not configured (offsite.remote)'
            return out
        lock = open(self.store.root / 'offsite.lock', 'a+')
        try:
            fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
        except OSError:
            out['error'] = 'another push is running'
            return out
        try:
            for members in self._plan(self.pending()):
                if limit_bundles is not None and out['bundles'] >= limit_bundles:
                    break
                try:
                    b = self._push_bundle(members)
                except (OSError, StoreError) as e:
                    out['failed'] += 1
                    out['error'] = str(e)[:300]
                    self.log(f'offsite push: {e}')
                    break      # the remote is likely down; the next run retries
                out['bundles'] += 1
                out['objects'] += b['objects']
                out['bytes'] += b['bytes']
        finally:
            lock.close()
        return out

    def _plan(self, pend) -> list:
        groups, cur, size = [], [], 0
        for sha, osize, kind, _ in pend:
            if cur and size + osize > self.bundle_bytes:
                groups.append(cur)
                cur, size = [], 0
            cur.append((sha, osize, kind))
            size += osize
        if cur:
            groups.append(cur)
        return groups

    def _push_bundle(self, members) -> dict:
        members = sorted(members)
        name = (f'{time.strftime("%Y%m%d", time.gmtime())}-'
                f'{hashlib.sha256(chr(10).join(m[0] for m in members).encode()).hexdigest()[:16]}.tar.{"zst" if ZSTD else "xz"}')
        stage = self.store.root / 'tmp' / f'offsite-{os.getpid()}'
        shutil.rmtree(stage, ignore_errors=True)
        stage.mkdir(parents=True)
        try:
            f = stage / name
            self._write_bundle(f, members)
            md5, sha, size = _md5_file(f)
            self.rclone('copy', str(stage), self.path('bundles'), '--include', name, transfer=True)
            self._verify_remote(stage, name, md5, size)
            manifest = stage / f'{name}.json'
            manifest.write_text(json.dumps({'name': name, 'md5': md5, 'sha256': sha, 'bytes': size, 'created': time.time(),
                                            'members': [{'sha256': s, 'size': z, 'kind': k} for s, z, k in members]}))
            self.rclone('copy', str(stage), self.path('manifests'), '--include', manifest.name, transfer=True)
            now = time.time()
            db = self.store.db
            db.execute('BEGIN')
            try:
                db.execute('INSERT OR REPLACE INTO bundles(name, remote, created, objects, original_bytes, bytes, md5, sha256, verified_at) '
                           'VALUES (?, ?, ?, ?, ?, ?, ?, ?, ?)', (name, self.path('bundles', name), now, len(members),
                                                                  sum(m[1] for m in members), size, md5, sha, now))
                db.executemany('INSERT OR REPLACE INTO copies(sha256, bundle, size, kind) VALUES (?, ?, ?, ?)',
                               [(s, name, z, k) for s, z, k in members])
                db.execute('COMMIT')
            except Exception:
                db.execute('ROLLBACK')
                raise
            self.store.log('offsite_push', None, {'bundle': name, 'objects': len(members), 'bytes': size})
            return {'name': name, 'objects': len(members), 'bytes': size}
        finally:
            shutil.rmtree(stage, ignore_errors=True)

    def _write_bundle(self, dest: Path, members) -> None:
        with open(dest, 'wb') as raw:
            comp = zstd_writer(raw) if ZSTD else lzma.LZMAFile(raw, 'wb', preset=6)
            with comp:
                with tarfile.open(fileobj=comp, mode='w|', format=tarfile.PAX_FORMAT) as tar:
                    for sha, size, _ in members:
                        o = self.store.get(sha)
                        if o is None:       # evicted since the plan; the bundle simply omits it
                            continue
                        ti = tarfile.TarInfo(sha)
                        ti.size, ti.mtime = o.size, int(o.first_seen)
                        with self.store.reader(o) as r:
                            tar.addfile(ti, r)
            raw.flush()
            os.fsync(raw.fileno())

    def _verify_remote(self, stage: Path, name: str, md5: str, size: int) -> None:
        p = self.rclone('lsjson', '--hash', '--files-only', self.path('bundles'), '--include', name)
        rows = json.loads(p.stdout or '[]')
        if len(rows) != 1:
            raise OffsiteError(f'verify {name}: the remote lists {len(rows)} files by that name')
        row = rows[0]
        if row.get('Size') != size:
            raise OffsiteError(f'verify {name}: remote has {row.get("Size")} bytes, sent {size}')
        got = (row.get('Hashes') or {}).get('md5')
        if got is not None:
            if got != md5:
                raise OffsiteError(f'verify {name}: remote md5 {got} differs from {md5}')
            return
        # a remote with no md5: compare the bytes themselves
        self.rclone('check', str(stage), self.path('bundles'), '--one-way', '--download', '--include', name)

    # ------------------------------------------------------------------ retrieve
    def locations(self, sha: str) -> list:
        return [r[0] for r in self.store.db.execute('SELECT bundle FROM copies WHERE sha256 = ?', (sha,))]

    def restore(self, sha: str):
        """Fetch the bundle holding ``sha`` from the remote, check it against the index, extract that object into
        the store and return its entry (None when no bundle has it). Raises OffsiteError when the remote fails."""
        if not self.enabled:
            return None
        got = self.store.get(sha)
        if got:
            return got
        for name in self.locations(sha):
            row = self.store.db.execute('SELECT md5, bytes FROM bundles WHERE name = ?', (name,)).fetchone()
            kind = self.store.db.execute('SELECT kind FROM copies WHERE sha256 = ? AND bundle = ?', (sha, name)).fetchone()[0]
            stage = self.store.root / 'tmp' / f'restore-{os.getpid()}-{time.monotonic_ns()}'
            stage.mkdir(parents=True)
            try:
                self.rclone('copyto', self.path('bundles', name), str(stage / name), transfer=True)
                md5, _, size = _md5_file(stage / name)
                if row and (md5, size) != (row[0], row[1]):
                    raise OffsiteError(f'{name}: fetched bundle differs from the index (md5 {md5}, {size} bytes)')
                tmp = self._extract(stage / name, sha)
                if tmp is None:
                    raise OffsiteError(f'{name} does not hold {sha}')
                try:
                    o = self.store.put_file(tmp, sha256=sha, kind=kind)
                finally:
                    Path(tmp).unlink(missing_ok=True)
                self.store.log('offsite_restore', sha, {'bundle': name})
                return o
            finally:
                shutil.rmtree(stage, ignore_errors=True)
        return None

    def _extract(self, bundle: Path, sha: str):
        with open(bundle, 'rb') as raw:
            comp = _zstd_reader(raw) if bundle.name.endswith('.zst') else lzma.LZMAFile(raw, 'rb')
            with comp, tarfile.open(fileobj=comp, mode='r|') as tar:
                for ti in tar:
                    if ti.name != sha:
                        continue
                    tmp = self.store.tmpfile('.x')
                    with open(tmp, 'wb') as out:
                        shutil.copyfileobj(tar.extractfile(ti), out, CHUNK)
                    return tmp
        return None

    # ------------------------------------------------------------------ rebuild
    def reindex(self) -> int:
        """Re-read the remote's manifests into the index (a lost index.sqlite). Returns bundles recorded."""
        p = self.rclone('lsjson', '--files-only', self.path('manifests'))
        n = 0
        for row in json.loads(p.stdout or '[]'):
            m = json.loads(self.rclone('cat', self.path('manifests', row['Path'])).stdout)
            now = time.time()
            self.store.db.execute('INSERT OR REPLACE INTO bundles(name, remote, created, objects, original_bytes, bytes, md5, sha256, verified_at) '
                                  'VALUES (?, ?, ?, ?, ?, ?, ?, ?, ?)', (m['name'], self.path('bundles', m['name']), m['created'],
                                                                         len(m['members']), sum(x['size'] for x in m['members']),
                                                                         m['bytes'], m['md5'], m['sha256'], now))
            self.store.db.executemany('INSERT OR REPLACE INTO copies(sha256, bundle, size, kind) VALUES (?, ?, ?, ?)',
                                      [(x['sha256'], m['name'], x['size'], x['kind']) for x in m['members']])
            n += 1
        return n


def _parse_bytes(v) -> int:
    from .daemon import parse_bytes
    return parse_bytes(v)
