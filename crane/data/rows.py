"""Columnar rows: write them in parts, read them back with the stdlib (or numpy) alone, describe them in a manifest.

    w = RowWriter(out, {'seed': 'int', 'arm': 'str', 'success': 'float'})
    w.write({'seed': 3, 'arm': 'a', 'success': 1.0})
    files = w.close()                         # [{path, rows, sha256}]
    write_manifest(out, w.schema, files, {'study': 'boat01'})
    rows = list(read_rows(out))               # dicts, typed by the manifest when there is one

interim; w3-data owns this. This is the minimal compatible implementation the cluster side needs; the merge keeps
w3-data's version.
"""

import csv
import hashlib
import json
import math
import os
from pathlib import Path

MANIFEST = 'manifest.json'
SCHEMA = 'crane.data.manifest/1'
_CAST = {'int': int, 'float': float, 'bool': lambda v: str(v).lower() in ('1', 'true', 'yes'), 'str': str}


def _have_pyarrow():
    try:
        import pyarrow  # noqa: F401
        import pyarrow.parquet  # noqa: F401
        return True
    except ImportError:
        return False


def _sha(path):
    h = hashlib.sha256()
    with open(path, 'rb') as f:
        for block in iter(lambda: f.read(1 << 20), b''):
            h.update(block)
    return h.hexdigest()


class RowWriter:
    def __init__(self, dir, schema, fmt='auto', part=None):
        self.dir = Path(dir)
        self.schema = dict(schema)
        self.fmt = ('parquet' if _have_pyarrow() else 'csv') if fmt == 'auto' else fmt
        self.part = part
        self._rows = []

    def write(self, row):
        self._rows.append({k: row.get(k) for k in self.schema})

    def close(self):
        if not self._rows:
            return []
        self.dir.mkdir(parents=True, exist_ok=True)
        stem = 'rows' if self.part is None else f'rows-{self.part}'
        path = self.dir / f'{stem}.{self.fmt}'
        tmp = path.with_name(f'.{path.name}.tmp')
        cols = list(self.schema)
        if self.fmt == 'csv':
            with open(tmp, 'w', newline='') as f:
                w = csv.writer(f)
                w.writerow(cols)
                for r in self._rows:
                    w.writerow(['' if r[c] is None else r[c] for c in cols])
        elif self.fmt == 'npz':
            import numpy as np
            with open(tmp, 'wb') as f:
                np.savez(f, **{c: np.asarray([r[c] for r in self._rows]) for c in cols})
        elif self.fmt == 'parquet':
            import pyarrow as pa
            import pyarrow.parquet as pq
            pq.write_table(pa.table({c: [r[c] for r in self._rows] for c in cols}), tmp)
        else:
            raise ValueError(f'RowWriter: unknown format {self.fmt!r} (auto, parquet, npz, csv)')
        os.replace(tmp, path)
        n, self._rows = len(self._rows), []
        return [{'path': path.name, 'rows': n, 'sha256': _sha(path)}]


def write_manifest(dir, schema, files, extra=None):
    """dir/manifest.json: {schema, columns, files:[{path, rows, sha256}], rows, ...extra}; returns the document."""
    doc = {'schema': SCHEMA, 'columns': dict(schema), 'files': list(files),
           'rows': sum(int(f.get('rows', 0)) for f in files), **(extra or {})}
    path = Path(dir) / MANIFEST
    path.parent.mkdir(parents=True, exist_ok=True)
    tmp = path.with_name(f'.{MANIFEST}.tmp')
    tmp.write_text(json.dumps(doc, indent=1, sort_keys=True) + '\n')
    os.replace(tmp, path)
    return doc


def _typed(row, columns):
    out = {}
    for k, v in row.items():
        kind = columns.get(k)
        if v == '' or v is None:
            out[k] = None
        elif kind in _CAST:
            out[k] = _CAST[kind](v)
        else:
            out[k] = _guess(v)
    return out


def _guess(v):
    if not isinstance(v, str):
        return v
    for cast in (int, float):
        try:
            x = cast(v)
            return x if not (isinstance(x, float) and math.isnan(x)) else None
        except ValueError:
            continue
    return v


def _read_file(path, columns):
    path = Path(path)
    if path.suffix == '.csv':
        with open(path, newline='') as f:
            for row in csv.DictReader(f):
                yield _typed(row, columns)
    elif path.suffix == '.npz':
        import numpy as np
        with np.load(path, allow_pickle=False) as z:
            cols = {k: z[k].tolist() for k in z.files}
        for i in range(len(next(iter(cols.values()), []))):
            yield {k: v[i] for k, v in cols.items()}
    elif path.suffix == '.parquet':
        import pyarrow.parquet as pq
        yield from pq.read_table(path).to_pylist()
    elif path.suffix == '.jsonl':
        with open(path) as f:
            for line in f:
                if line.strip():
                    yield json.loads(line)


def read_rows(path_or_dir):
    """Every row of a file, or of a dir's manifest files (else its rows* files), as dicts."""
    p = Path(path_or_dir)
    if p.is_file():
        yield from _read_file(p, {})
        return
    man = p / MANIFEST
    if man.is_file():
        doc = json.loads(man.read_text())
        for f in doc.get('files', []):
            yield from _read_file(p / f['path'], doc.get('columns') or {})
        return
    for f in sorted(p.glob('rows*')):
        yield from _read_file(f, {})
