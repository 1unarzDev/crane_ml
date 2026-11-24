"""Columnar rows: one writer for every per-episode table crane writes, readable with the stdlib alone.

    w = RowWriter(out / 'rows', {'arm': 'str', 'seed': 'int', 'succeeded': 'float'})
    w.write({'arm': 'a', 'seed': 3, 'succeeded': 1.0})
    files = w.close()                       # [{path, rows, sha256}]
    write_manifest(out, schema, files, {'study': 'toy'})
    for row in read_rows(out): ...

fmt 'auto' is parquet when pyarrow imports, else CSV. A column type is str, int, float or bool; a missing value
is None (an empty CSV cell). read_rows takes a file, a dir holding manifest.json (types come from it), or a dir of
such dirs.

interim; w3-data owns this (crane/data/rows.py on that branch replaces this file at merge).
"""

import csv
import hashlib
import json
import math
from pathlib import Path

MANIFEST = 'manifest.json'
SCHEMA = 'crane.data.manifest/1'
TYPES = ('str', 'int', 'float', 'bool')


def _parquet():
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
        self.dir, self.schema = Path(dir), dict(schema)
        bad = {k: t for k, t in self.schema.items() if t not in TYPES}
        if bad:
            raise ValueError(f'column types {bad}: choose from {", ".join(TYPES)}')
        self.fmt = ('parquet' if _parquet() else 'csv') if fmt == 'auto' else fmt
        if self.fmt not in ('parquet', 'npz', 'csv'):
            raise ValueError(f'fmt {fmt!r}: choose auto, parquet, npz or csv')
        self.part = part or 'part-0'
        self.rows = []

    def write(self, row):
        self.rows.append({k: _cast(row.get(k), t) for k, t in self.schema.items()})

    def close(self):
        """Write the part file; returns [{path, rows, sha256}] (none when no row was written)."""
        if not self.rows:
            return []
        self.dir.mkdir(parents=True, exist_ok=True)
        path = self.dir / f'{self.part}.{self.fmt}'
        tmp = path.with_name(f'.{path.name}.tmp')
        if self.fmt == 'csv':
            with open(tmp, 'w', newline='') as f:
                w = csv.writer(f)
                w.writerow(list(self.schema))
                for r in self.rows:
                    w.writerow(['' if r[k] is None else _text(r[k]) for k in self.schema])
        elif self.fmt == 'npz':
            import numpy as np
            cols = {}
            for k, t in self.schema.items():
                vals = [r[k] for r in self.rows]
                if t == 'float':
                    cols[k] = np.array([math.nan if v is None else v for v in vals], dtype='f8')
                elif t in ('int', 'bool') and all(v is not None for v in vals):
                    cols[k] = np.array(vals, dtype='i8' if t == 'int' else bool)
                else:
                    cols[k] = np.array(['' if v is None else str(v) for v in vals])
            with open(tmp, 'wb') as f:
                np.savez(f, **cols)
        else:
            import pyarrow as pa
            import pyarrow.parquet as pq
            kinds = {'str': pa.string(), 'int': pa.int64(), 'float': pa.float64(), 'bool': pa.bool_()}
            table = pa.table({k: pa.array([r[k] for r in self.rows], type=kinds[t]) for k, t in self.schema.items()})
            pq.write_table(table, tmp)
        tmp.replace(path)
        n, self.rows = len(self.rows), []
        return [{'path': str(path), 'rows': n, 'sha256': _sha(path)}]


def _text(v):
    return ('true' if v else 'false') if isinstance(v, bool) else repr(v) if isinstance(v, float) else str(v)


def _cast(v, t):
    if v is None or v == '':
        return None
    try:
        if t == 'str':
            return v if isinstance(v, str) else json.dumps(v) if isinstance(v, (dict, list)) else str(v)
        if t == 'int':
            return int(v)
        if t == 'float':
            return float(v)
        if t == 'bool':
            return v if isinstance(v, bool) else str(v).lower() in ('1', 'true', 'yes')
    except (TypeError, ValueError):
        return None
    return v


def write_manifest(dir, schema, files, extra=None):
    """DIR/manifest.json: {schema, columns, files:[{path (relative to DIR), rows, sha256[, kind]}], rows, **extra}.
    A file's kind defaults to rows; other kinds (traj, events) are listed but not read back as rows."""
    dir = Path(dir)
    listed = []
    for f in files:
        p = Path(f['path'])
        try:
            rel = p.resolve().relative_to(dir.resolve()).as_posix()
        except ValueError:
            rel = str(p)
        listed.append({**f, 'path': rel})
    doc = {'schema': SCHEMA, 'columns': [{'name': k, 'type': t} for k, t in dict(schema).items()],
           'files': listed, 'rows': sum(f.get('rows', 0) for f in listed if f.get('kind', 'rows') == 'rows'),
           **(extra or {})}
    dir.mkdir(parents=True, exist_ok=True)
    tmp = dir / f'.{MANIFEST}.tmp'
    tmp.write_text(json.dumps(doc, indent=1, default=str) + '\n')
    tmp.replace(dir / MANIFEST)
    return doc


def read_rows(path_or_dir):
    """Every row (a dict) of a part file, a dir with manifest.json, or a dir of such dirs."""
    path = Path(path_or_dir)
    if path.is_file():
        yield from _read_file(path, None)
        return
    manifests = [path / MANIFEST] if (path / MANIFEST).is_file() else sorted(path.glob(f'*/{MANIFEST}'))
    for m in manifests:
        doc = json.loads(m.read_text())
        types = {c['name']: c['type'] for c in doc.get('columns', [])}
        for f in doc.get('files', []):
            p = m.parent / f['path']
            if p.suffix in ('.csv', '.parquet', '.npz') and f.get('kind', 'rows') == 'rows':
                yield from _read_file(p, types)


def _read_file(path, types):
    if path.suffix == '.csv':
        with open(path, newline='') as f:
            for r in csv.DictReader(f):
                yield {k: _cast(v, (types or {}).get(k, 'str')) for k, v in r.items()}
    elif path.suffix == '.npz':
        import numpy as np
        with np.load(path) as z:
            cols = {k: z[k].tolist() for k in z.files}
        n = len(next(iter(cols.values()), []))
        for i in range(n):
            row = {}
            for k, vals in cols.items():
                v = vals[i]
                t = (types or {}).get(k)
                row[k] = None if (v == '' or (isinstance(v, float) and math.isnan(v))) else _cast(v, t) if t else v
            yield row
    elif path.suffix == '.parquet':
        import pyarrow.parquet as pq
        yield from pq.read_table(path).to_pylist()
    else:
        raise ValueError(f'{path}: not a rows file (.csv, .npz, .parquet)')


__all__ = ['RowWriter', 'read_rows', 'write_manifest', 'MANIFEST']
