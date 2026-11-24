"""A content-addressed unit cache: one ``records.jsonl`` per cache directory, safe for many processes.

    cache = UnitCache('~/.cache/crane/units/teamgame')
    unit = {'arm': 'a', 'cell': cell_sha, 'seed': 3, 'card': card_sha, 'commit': commit}
    if not cache.has(unit):
        cache.put(unit, {'success': 1, 'time_s': 41.2})
    cache.get(unit)                           # -> the record, or None
    sha = cache.put_blob(path_or_bytes, '.npz') ; cache.blob_path(sha)

A unit's key is ``crane.ids.digest(unit)``. Each line is ``{"key", "unit", "record"}``. A writer takes an
exclusive ``flock`` on the file, re-reads what others appended, and appends only when the key is new (the first
writer wins), then fsyncs. Readers skip a torn line a killed writer left; the next writer terminates it first.
Blobs live in ``blobs/<sha[:2]>/<sha><suffix>``, written by temp file + rename.
"""

import fcntl
import json
import os
from pathlib import Path
import tempfile

from crane.ids import digest, sha256

RECORDS = 'records.jsonl'


class UnitCache:
    def __init__(self, root):
        self.root = Path(os.path.expanduser(str(root)))
        self.path = self.root / RECORDS
        self._index, self._offset = {}, 0

    @staticmethod
    def key(unit):
        """The content key of a unit dict (or an already computed 64-hex key)."""
        if isinstance(unit, str) and len(unit) == 64:
            return unit
        return digest(unit)

    def _refresh(self):
        """Read lines appended since the last read (complete lines only)."""
        try:
            with open(self.path, 'rb') as f:
                f.seek(self._offset)
                data = f.read()
        except FileNotFoundError:
            return
        end = data.rfind(b'\n') + 1
        for raw in data[:end].splitlines():
            try:
                entry = json.loads(raw)
                self._index.setdefault(entry['key'], entry)
            except (ValueError, KeyError, TypeError):
                continue  # a torn line from a killed writer
        self._offset += end

    def _entry(self, unit):
        key = self.key(unit)
        if key not in self._index:
            self._refresh()
        return self._index.get(key)

    def has(self, unit):
        return self._entry(unit) is not None

    def get(self, unit, default=None):
        entry = self._entry(unit)
        return entry['record'] if entry else default

    def put(self, unit, record):
        """Store ``record`` for ``unit`` unless another writer got there first; returns the stored record."""
        key = self.key(unit)
        entry = {'key': key, 'unit': unit if isinstance(unit, dict) else None, 'record': record}
        line = (json.dumps(entry, sort_keys=True, separators=(',', ':'), default=str) + '\n').encode()
        self.root.mkdir(parents=True, exist_ok=True)
        fd = os.open(self.path, os.O_RDWR | os.O_CREAT | os.O_APPEND, 0o644)
        try:
            fcntl.flock(fd, fcntl.LOCK_EX)
            self._refresh()
            if key in self._index:
                return self._index[key]['record']
            size = os.fstat(fd).st_size
            if size and os.pread(fd, 1, size - 1) != b'\n':
                os.write(fd, b'\n')  # end a torn line so ours starts clean
            os.write(fd, line)
            os.fsync(fd)
            self._index[key] = json.loads(line)
            return self._index[key]['record']
        finally:
            os.close(fd)  # releases the lock

    def __iter__(self):
        """Every entry ``{key, unit, record}``, in the order written."""
        self._refresh()
        return iter(list(self._index.values()))

    def __len__(self):
        self._refresh()
        return len(self._index)

    def __contains__(self, unit):
        return self.has(unit)

    def put_blob(self, data, suffix=''):
        """Store bytes (or a file's bytes) by their sha256; returns the sha. An existing blob is kept."""
        data = Path(data).read_bytes() if isinstance(data, (str, Path)) else bytes(data)
        sha = sha256(data)
        path = self._blob(sha, suffix)
        if not path.exists():
            path.parent.mkdir(parents=True, exist_ok=True)
            fd, tmp = tempfile.mkstemp(dir=path.parent, prefix='.tmp-')
            try:
                with os.fdopen(fd, 'wb') as f:
                    f.write(data)
                    f.flush()
                    os.fsync(f.fileno())
                os.replace(tmp, path)
            except BaseException:
                Path(tmp).unlink(missing_ok=True)
                raise
        return sha

    def blob_path(self, sha, suffix=None):
        """The stored blob's path, or None. ``suffix`` None finds it whatever its suffix."""
        if suffix is not None:
            path = self._blob(sha, suffix)
            return path if path.exists() else None
        folder = self.root / 'blobs' / sha[:2]
        found = sorted(folder.glob(f'{sha}*')) if folder.is_dir() else []
        return found[0] if found else None

    def get_blob(self, sha, suffix=None):
        path = self.blob_path(sha, suffix)
        return path.read_bytes() if path else None

    def _blob(self, sha, suffix):
        return self.root / 'blobs' / sha[:2] / f'{sha}{suffix}'
