"""The archive's store: content-addressed by the sha256 of the original bytes, so equal bytes are kept once.

Layout under ``root`` (any directory: a plain one on a system disk, a ZFS dataset, a USB disk)::

    .crane-archive-root               marker {node, created, layout}; the store refuses a root without it
    objects/ab/cd/<sha256>[.zst|.xz]  objects >= PACK_MAX, one file each
    packs/YYYY/MM/DD-NNNN.pack        smaller objects appended back to back (each its own frame or raw);
                                      a pack is sealed at PACK_SEAL bytes or at the day's end
    index.sqlite                      sha256 -> (path, offset, length, codec, size, kind, first_seen, verified_at)
    quarantine/                       objects that failed a scrub (kept for a look, never served)
    tmp/                              staging on the same filesystem (rename is atomic)

Writes: stage in tmp/, hash, compress, fsync, rename, fsync the directory, then the index row. Only after
that does the daemon confirm to the coordinator. Stdlib only; zstd through ``compression.zstd`` (Python
3.14) or the ``zstandard`` wheel, else xz (lzma), so it runs on any Python >= 3.9.
"""
from __future__ import annotations

import hashlib
import io
import json
import lzma
import os
import shutil
import sqlite3
import time
from dataclasses import dataclass
from pathlib import Path

LAYOUT = 1
MARKER = '.crane-archive-root'
PACK_MAX = 64 * 1024          # objects smaller than this (stored size) go into packs
PACK_SEAL = 256 * 1024 * 1024
CHUNK = 1024 * 1024

# kinds stored as they come (already compressed, or Range-served)
NO_COMPRESS = {'ctraj', 'lod', 'bundle', 'render_master', 'render_final'}
NEVER_COMPRESS_EXT = ('.zst', '.gz', '.xz', '.bz2', '.zip', '.7z', '.mp4', '.mkv', '.webm', '.gif', '.png', '.jpg',
                      '.jpeg', '.webp', '.ctraj', '.mcap', '.db3')


try:  # Python 3.14
    from compression import zstd as _zstd

    def _zstd_compress_file(src, dst):
        with open(src, 'rb') as fi, _zstd.open(dst, 'wb', level=19) as fo:
            shutil.copyfileobj(fi, fo, CHUNK)

    def _zstd_reader(f):
        return _zstd.ZstdFile(f, 'rb')
    ZSTD = True
except ImportError:  # pragma: no cover - depends on the Python
    try:
        import zstandard as _zs

        def _zstd_compress_file(src, dst):
            with open(src, 'rb') as fi, open(dst, 'wb') as fo:
                _zs.ZstdCompressor(level=19).copy_stream(fi, fo)

        def _zstd_reader(f):
            return _zs.ZstdDecompressor().stream_reader(f)
        ZSTD = True
    except ImportError:
        ZSTD = False


def compress_codec() -> str:
    return 'zstd19' if ZSTD else 'xz'


class StoreError(RuntimeError):
    pass


@dataclass
class Obj:
    sha256: str
    path: str          # relative to root
    offset: int
    length: int        # stored bytes
    codec: str         # none, zstd19, xz
    size: int          # original bytes
    kind: str
    first_seen: float
    verified_at: float | None
    pinned: int

    @property
    def packed(self) -> bool:
        return self.path.startswith('packs/')


SCHEMA = """
CREATE TABLE IF NOT EXISTS objects (
  sha256 TEXT PRIMARY KEY, path TEXT NOT NULL, offset INTEGER NOT NULL, length INTEGER NOT NULL,
  codec TEXT NOT NULL, size INTEGER NOT NULL, kind TEXT NOT NULL, first_seen REAL NOT NULL,
  verified_at REAL, pinned INTEGER NOT NULL DEFAULT 0);
CREATE INDEX IF NOT EXISTS objects_verified ON objects(verified_at);
CREATE INDEX IF NOT EXISTS objects_kind ON objects(kind, first_seen);
CREATE TABLE IF NOT EXISTS keys (key TEXT PRIMARY KEY, sha256 TEXT NOT NULL, kind TEXT, created TEXT);
CREATE INDEX IF NOT EXISTS keys_sha ON keys(sha256);
CREATE TABLE IF NOT EXISTS log (t REAL NOT NULL, event TEXT NOT NULL, sha256 TEXT, detail TEXT);
"""


def _fsync_dir(d: Path) -> None:
    try:
        fd = os.open(str(d), os.O_RDONLY)
    except OSError:
        return
    try:
        os.fsync(fd)
    except OSError:
        pass
    finally:
        os.close(fd)


def sha256_file(path, offset: int = 0, length: int | None = None) -> str:
    h = hashlib.sha256()
    with open(path, 'rb') as f:
        f.seek(offset)
        left = length
        while left is None or left > 0:
            b = f.read(CHUNK if left is None else min(CHUNK, left))
            if not b:
                break
            h.update(b)
            if left is not None:
                left -= len(b)
    return h.hexdigest()


def _worth_compressing(path: Path, kind: str, name: str = '') -> bool:
    if kind in NO_COMPRESS or name.lower().endswith(NEVER_COMPRESS_EXT):
        return False
    with open(path, 'rb') as f:
        head = f.read(256 * 1024)
    if len(head) < 512:
        return False
    return len(lzma.compress(head[:65536], preset=1)) < 0.85 * min(len(head), 65536)


class Store:
    def __init__(self, root, *, node: str = 'archive', create: bool = False):
        self.root = Path(root).expanduser()
        marker = self.root / MARKER
        if not marker.is_file():
            if not create:
                raise StoreError(f'{self.root} has no {MARKER}: not an archive root (crane-archive init creates one)')
            if self.root.exists() and any(p.name not in ('lost+found',) for p in self.root.iterdir()):
                raise StoreError(f'{self.root} is not empty and has no {MARKER}; refusing to adopt it')
            self.root.mkdir(parents=True, exist_ok=True)
            marker.write_text(json.dumps({'node': node, 'created': time.strftime('%Y-%m-%dT%H:%M:%SZ', time.gmtime()),
                                          'layout': LAYOUT}) + '\n')
        info = json.loads(marker.read_text())
        if info.get('layout') != LAYOUT:
            raise StoreError(f'archive layout {info.get("layout")} is not supported (this daemon: {LAYOUT})')
        for d in ('objects', 'packs', 'tmp', 'quarantine'):
            (self.root / d).mkdir(exist_ok=True)
        self.marker = info
        self._db = None

    # ------------------------------------------------------------------ index
    @property
    def db(self) -> sqlite3.Connection:
        if self._db is None:
            self._db = self.connect()
        return self._db

    def connect(self) -> sqlite3.Connection:
        """A connection of its own (one per thread: the HTTP server reads while the daemon writes)."""
        db = sqlite3.connect(str(self.root / 'index.sqlite'), timeout=30, isolation_level=None, check_same_thread=False)
        db.execute('PRAGMA journal_mode=WAL')
        db.execute('PRAGMA synchronous=FULL')
        db.executescript(SCHEMA)
        return db

    def get(self, sha: str, db: sqlite3.Connection | None = None) -> Obj | None:
        r = (db or self.db).execute('SELECT sha256, path, offset, length, codec, size, kind, first_seen, verified_at, pinned '
                                    'FROM objects WHERE sha256 = ?', (sha,)).fetchone()
        return Obj(*r) if r else None

    def has(self, sha: str) -> bool:
        return self.get(sha) is not None

    def log(self, event: str, sha: str | None = None, detail=None) -> None:
        self.db.execute('INSERT INTO log(t, event, sha256, detail) VALUES (?, ?, ?, ?)',
                        (time.time(), event, sha, json.dumps(detail) if detail is not None else None))

    def remember_key(self, key: str, sha: str, kind: str | None, created: str | None) -> None:
        self.db.execute('INSERT INTO keys(key, sha256, kind, created) VALUES (?, ?, ?, ?) ON CONFLICT(key) DO UPDATE '
                        'SET sha256 = excluded.sha256, kind = excluded.kind', (key, sha, kind, created))

    # ------------------------------------------------------------------ write
    def tmpfile(self, suffix: str = '') -> Path:
        return self.root / 'tmp' / f'{os.getpid()}-{time.monotonic_ns()}{suffix}'

    def put_file(self, src, *, sha256: str | None = None, kind: str = 'raw', name: str = '') -> Obj:
        """Store ``src`` (a file in tmp/ or anywhere; it is not modified). Returns the index entry; an object
        already present is returned as is (dedup). Raises StoreError when ``sha256`` does not match."""
        src = Path(src)
        actual = sha256_file(src)
        if sha256 and actual != sha256:
            raise StoreError(f'sha256 mismatch: expected {sha256}, file is {actual}')
        have = self.get(actual)
        if have:
            return have
        size = src.stat().st_size
        codec, payload = 'none', src
        staged = None
        if _worth_compressing(src, kind, name):
            codec = compress_codec()
            staged = self.tmpfile('.c')
            if codec == 'zstd19':
                _zstd_compress_file(src, staged)
            else:
                with open(src, 'rb') as fi, lzma.open(staged, 'wb', preset=9 | lzma.PRESET_EXTREME) as fo:
                    shutil.copyfileobj(fi, fo, CHUNK)
            if staged.stat().st_size >= 0.95 * size:   # not worth it after all
                staged.unlink()
                staged, codec = None, 'none'
            else:
                payload = staged
        try:
            length = payload.stat().st_size
            if length < PACK_MAX:
                rel, offset = self._append_pack(payload, length)
            else:
                rel, offset = self._place_object(payload, actual, codec), 0
            now = time.time()
            self.db.execute('INSERT INTO objects(sha256, path, offset, length, codec, size, kind, first_seen, verified_at) '
                            'VALUES (?, ?, ?, ?, ?, ?, ?, ?, ?)', (actual, rel, offset, length, codec, size, kind, now, now))
        finally:
            if staged and staged.exists():
                staged.unlink()
        return self.get(actual)

    def put_bytes(self, data: bytes, **kw) -> Obj:
        tmp = self.tmpfile()
        try:
            with open(tmp, 'wb') as f:
                f.write(data)
            return self.put_file(tmp, **kw)
        finally:
            tmp.unlink(missing_ok=True)

    def _place_object(self, payload: Path, sha: str, codec: str) -> str:
        ext = {'none': '', 'zstd19': '.zst', 'xz': '.xz'}[codec]
        rel = f'objects/{sha[:2]}/{sha[2:4]}/{sha}{ext}'
        dest = self.root / rel
        dest.parent.mkdir(parents=True, exist_ok=True)
        tmp = self.tmpfile('.o')
        shutil.copyfile(payload, tmp)
        with open(tmp, 'rb+') as f:
            os.fsync(f.fileno())
        os.replace(tmp, dest)
        _fsync_dir(dest.parent)
        return rel

    def _append_pack(self, payload: Path, length: int) -> tuple:
        day = time.strftime('%Y/%m/%d', time.gmtime())
        y, m, d = day.split('/')
        pdir = self.root / 'packs' / y / m
        pdir.mkdir(parents=True, exist_ok=True)
        n = 1
        while True:
            p = pdir / f'{d}-{n:04d}.pack'
            if not p.exists() or p.stat().st_size + length <= PACK_SEAL:
                break
            n += 1
        fresh = not p.exists()
        with open(p, 'ab') as f:
            offset = f.tell()
            with open(payload, 'rb') as fi:
                shutil.copyfileobj(fi, f, CHUNK)
            f.flush()
            os.fsync(f.fileno())
        if fresh:
            _fsync_dir(pdir)
        return str(p.relative_to(self.root)), offset

    # ------------------------------------------------------------------ read
    def open_raw(self, o: Obj):
        """A file object positioned at the stored bytes and the stored length."""
        f = open(self.root / o.path, 'rb')
        f.seek(o.offset)
        return f

    def reader(self, o: Obj):
        """A binary stream of the original bytes."""
        f = self.open_raw(o)
        if o.codec == 'none':
            return _Window(f, o.length)
        win = _Window(f, o.length)
        if o.codec == 'zstd19':
            if not ZSTD:
                raise StoreError('object is zstd but no zstd decoder is installed (Python 3.14 or pip install zstandard)')
            return _zstd_reader(win)
        if o.codec == 'xz':
            return lzma.open(win, 'rb')
        raise StoreError(f'unknown codec {o.codec}')

    def read_bytes(self, sha: str) -> bytes:
        o = self.get(sha)
        if not o:
            raise KeyError(sha)
        with self.reader(o) as r:
            return r.read()

    def verify(self, o: Obj) -> bool:
        h = hashlib.sha256()
        try:
            with self.reader(o) as r:
                while True:
                    b = r.read(CHUNK)
                    if not b:
                        break
                    h.update(b)
        except (OSError, EOFError, lzma.LZMAError, ValueError, StoreError):
            return False
        except Exception:  # zstd errors differ by library  # noqa: BLE001
            return False
        return h.hexdigest() == o.sha256

    # ------------------------------------------------------------------ maintenance
    def scrub(self, budget_bytes: int, *, now: float | None = None) -> dict:
        """Rehash objects, the least recently verified first, until ``budget_bytes`` of stored bytes.
        A mismatch quarantines the copy and drops its index row; returns {checked, bytes, bad: [sha]}."""
        now = now or time.time()
        done = {'checked': 0, 'bytes': 0, 'bad': []}
        rows = self.db.execute('SELECT sha256 FROM objects ORDER BY verified_at IS NOT NULL, verified_at, sha256').fetchall()
        for (sha,) in rows:
            if done['bytes'] >= budget_bytes:
                break
            o = self.get(sha)
            if not o:
                continue
            done['checked'] += 1
            done['bytes'] += o.length
            if self.verify(o):
                self.db.execute('UPDATE objects SET verified_at = ? WHERE sha256 = ?', (now, sha))
            else:
                self.quarantine(o, 'scrub mismatch')
                done['bad'].append(sha)
        return done

    def quarantine(self, o: Obj, why: str) -> None:
        if not o.packed and (self.root / o.path).exists():
            os.replace(self.root / o.path, self.root / 'quarantine' / Path(o.path).name)
        self.db.execute('DELETE FROM objects WHERE sha256 = ?', (o.sha256,))
        self.log('quarantine', o.sha256, {'why': why, 'path': o.path})

    def remove(self, o: Obj, why: str) -> None:
        """Delete an object (eviction). Packed members only lose their index row (the pack is not rewritten)."""
        if not o.packed:
            (self.root / o.path).unlink(missing_ok=True)
        self.db.execute('DELETE FROM objects WHERE sha256 = ?', (o.sha256,))
        self.log('evict', o.sha256, {'why': why, 'kind': o.kind, 'bytes': o.length})

    def pin(self, sha: str, pinned: bool = True) -> bool:
        return self.db.execute('UPDATE objects SET pinned = ? WHERE sha256 = ?', (int(pinned), sha)).rowcount == 1

    def stats(self) -> dict:
        n, stored, orig = self.db.execute('SELECT COUNT(*), COALESCE(SUM(length), 0), COALESCE(SUM(size), 0) FROM objects').fetchone()
        by_kind = {k: {'objects': c, 'stored': s, 'size': z} for k, c, s, z in self.db.execute(
            'SELECT kind, COUNT(*), SUM(length), SUM(size) FROM objects GROUP BY kind ORDER BY kind')}
        packs = sum(p.stat().st_size for p in (self.root / 'packs').rglob('*.pack'))
        loose = self.db.execute("SELECT COALESCE(SUM(length), 0) FROM objects WHERE path LIKE 'objects/%'").fetchone()[0]
        oldest = self.db.execute('SELECT MIN(COALESCE(verified_at, 0)) FROM objects').fetchone()[0]
        return {'objects': n, 'stored_bytes': stored, 'original_bytes': orig, 'disk_bytes': packs + loose,
                'kinds': by_kind, 'oldest_verified': oldest}

    def disk(self) -> dict:
        st = os.statvfs(self.root)
        return {'total': st.f_blocks * st.f_frsize, 'free': st.f_bavail * st.f_frsize, 'fs': fs_type(self.root)}

    def clean_tmp(self, older_s: float = 3600) -> None:
        cutoff = time.time() - older_s
        for p in (self.root / 'tmp').iterdir():
            try:
                if p.stat().st_mtime < cutoff:
                    p.unlink()
            except OSError:
                pass

    # ------------------------------------------------------------------ move (drives arrive, host changes)
    def move_to(self, dest, *, progress=print) -> 'Store':
        """Copy the archive to ``dest`` (an empty directory), verify every object there, and return the new store.
        The old root is left untouched; the caller switches the config and removes it when satisfied."""
        dest = Path(dest).expanduser()
        if dest.exists() and any(dest.iterdir()):
            raise StoreError(f'{dest} is not empty')
        dest.mkdir(parents=True, exist_ok=True)
        shutil.copy2(self.root / MARKER, dest / MARKER)
        for sub in ('objects', 'packs'):
            shutil.copytree(self.root / sub, dest / sub, dirs_exist_ok=True)
        with sqlite3.connect(str(dest / 'index.sqlite')) as out:
            self.db.backup(out)
        new = Store(dest)
        bad = [o for o in (new.get(s) for (s,) in new.db.execute('SELECT sha256 FROM objects')) if o and not new.verify(o)]
        if bad:
            raise StoreError(f'{len(bad)} objects failed verification at {dest}; the old root is untouched')
        progress(f'moved {new.stats()["objects"]} objects to {dest}; all verified')
        return new


class _Window(io.RawIOBase):
    """``length`` bytes of ``f`` from its current position."""

    def __init__(self, f, length: int):
        self.f, self.left = f, length

    def readable(self):
        return True

    def seekable(self):
        return True

    def seek(self, n, whence=1):
        """Forward only, relative (what a Range read needs)."""
        if whence != 1 or n < 0:
            raise io.UnsupportedOperation('forward relative seeks only')
        n = min(n, self.left)
        self.f.seek(n, 1)
        self.left -= n
        return n

    def readinto(self, b):
        if self.left <= 0:
            return 0
        n = self.f.readinto(memoryview(b)[:min(len(b), self.left)])
        self.left -= n or 0
        return n

    def close(self):
        self.f.close()
        super().close()


def fs_type(path) -> str:
    """The filesystem type holding ``path`` (from /proc/mounts; 'unknown' elsewhere)."""
    try:
        path = os.path.realpath(path)
        best, kind = '', 'unknown'
        with open('/proc/mounts') as f:
            for line in f:
                parts = line.split()
                if len(parts) >= 3:
                    mnt = parts[1].replace('\\040', ' ')
                    if (path == mnt or path.startswith(mnt.rstrip('/') + '/')) and len(mnt) >= len(best):
                        best, kind = mnt, parts[2]
        return kind
    except OSError:
        return 'unknown'
