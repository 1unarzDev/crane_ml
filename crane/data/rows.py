"""Columnar rows: one schema, parquet / csv / npz / jsonl files, a manifest with a sha256 per file.

    w = RowWriter(out, {'seed': 'int', 'arm': 'str', 'score': 'float', 'obs': 'array:float16'})
    w.write({'seed': 3, 'arm': 'a', 'score': 0.5, 'obs': array_T_by_C})
    files = w.close()                       # [{path, rows, sha256}]
    write_manifest(out, w.schema, files, {'study': 'capxfer'})
    for row in read_rows(out): ...          # dicts; stdlib + numpy only for csv and npz

Types: ``int`` ``float`` ``str`` ``bool`` ``json`` (any JSON value) and ``array:DTYPE`` (a numpy array per row).
A missing column is null; a column the schema does not name is refused. ``fmt='auto'`` is parquet when pyarrow
imports, else csv. Array columns live in npz: the whole part with ``fmt='npz'``, else a ``PART.arrays.npz`` beside
the csv/parquet file, row for row. In npz an array column is stacked to (rows, *shape); rows of unequal first
dimension are padded (NaN, or 0 for ints) and ``COL@len`` holds each row's length. A file rolls over to the next
(``PART-001``...) at ``max_rows`` rows or about ``max_bytes`` of arrays, so no artifact passes the cluster's 95 MB.
npz files are written with fixed zip timestamps: equal rows give equal bytes.

Nulls and NaN: parquet, jsonl and npz keep a null and a float NaN apart (npz marks nulls in ``COL@null``; jsonl
writes ``NaN`` as Python's json does). csv has one empty cell for null: an empty string reads back as null, a NaN
reads back as NaN. Any ``.jsonl`` of one JSON object per line reads as rows (e.g. a UnitCache ``records.jsonl``).
"""

import csv
import hashlib
import io
import json
import os
from pathlib import Path
import tempfile
import zipfile

import numpy as np

MANIFEST = 'manifest.json'
SCHEMA = 'crane.data.manifest/1'
SCALARS = ('int', 'float', 'str', 'bool', 'json')
EXT = {'parquet': '.parquet', 'csv': '.csv', 'npz': '.npz', 'jsonl': '.jsonl'}
ARRAYS = '.arrays.npz'
LEN = '@len'
NULL = '@null'


class SchemaError(ValueError):
    """A row or a schema that does not fit; the message is one line naming the fix."""


def have_parquet():
    try:
        import pyarrow  # noqa: F401
        import pyarrow.parquet  # noqa: F401
    except ImportError:
        return False
    return True


def _need_parquet(what):
    if not have_parquet():
        raise SchemaError(f"{what} needs pyarrow; fix: pip install 'crane[data]' (or use --format npz|csv)")


def check_schema(schema):
    """{name: type} with every type known; returns it as a plain dict (order kept)."""
    schema = dict(schema)
    for name, kind in schema.items():
        if not isinstance(name, str) or not name or LEN in name or NULL in name:
            raise SchemaError(f'column name {name!r} is not allowed (non-empty, without {LEN!r} or {NULL!r})')
        if kind in SCALARS:
            continue
        if not (isinstance(kind, str) and kind.startswith('array:')):
            raise SchemaError(f"column {name!r}: unknown type {kind!r}; types: {', '.join(SCALARS)}, array:DTYPE")
        try:
            np.dtype(kind[6:])
        except TypeError:
            raise SchemaError(f'column {name!r}: {kind!r} names no numpy dtype (e.g. array:float16)') from None
    return schema


def is_array(kind):
    return kind.startswith('array:')


def _scalar(name, kind, value):
    """A row value as the column's Python type (None stays None)."""
    if value is None:
        return None
    try:
        if kind == 'int':
            if isinstance(value, float) and not value.is_integer():
                raise ValueError(value)
            return int(value)
        if kind == 'float':
            return float(value)
        if kind == 'bool':
            if isinstance(value, str):
                if value.lower() not in ('true', 'false', '1', '0'):
                    raise ValueError(value)
                return value.lower() in ('true', '1')
            return bool(value)
        if kind == 'str':
            return str(value)
        return json.dumps(value, sort_keys=True, separators=(',', ':'), default=_plain)
    except (TypeError, ValueError):
        raise SchemaError(f'column {name!r} ({kind}): cannot store {value!r}') from None


def _plain(o):
    if hasattr(o, 'tolist'):
        return o.tolist()
    raise TypeError(o)


class RowWriter:
    """Rows into DIR as one or more files of one format. ``part`` names the files (default the next free number)."""

    def __init__(self, dir, schema, fmt='auto', part=None, max_rows=1_000_000, max_bytes=90_000_000):
        self.dir = Path(dir)
        self.schema = check_schema(schema)
        if fmt == 'auto':
            fmt = 'parquet' if have_parquet() else 'csv'
        if fmt not in EXT:
            raise SchemaError(f"format {fmt!r}: use auto, parquet, npz, csv or jsonl")
        if fmt == 'parquet':
            _need_parquet('parquet')
        self.fmt, self.max_rows, self.max_bytes = fmt, int(max_rows), int(max_bytes)
        self.dir.mkdir(parents=True, exist_ok=True)
        self.part = str(part) if part is not None else self._free_part()
        self.arrays = [n for n, k in self.schema.items() if is_array(k)]
        self.files, self._k, self._closed = [], 0, False
        self._reset()

    def _free_part(self):
        n = 0
        while any(self.dir.glob(f'part-{n:04d}*')):
            n += 1
        return f'part-{n:04d}'

    def _reset(self):
        self._cols = {n: [] for n in self.schema}
        self._n, self._bytes = 0, 0

    def write(self, row):
        if self._closed:
            raise SchemaError('this RowWriter is closed')
        unknown = [k for k in row if k not in self.schema]
        if unknown:
            raise SchemaError(f"unknown column {unknown[0]!r}; schema columns: {', '.join(self.schema)} "
                              '(add it to the schema)')
        for name, kind in self.schema.items():
            value = row.get(name)
            if is_array(kind):
                if value is not None:
                    value = np.asarray(value, dtype=kind[6:])
                    if value.ndim == 0:
                        raise SchemaError(f'column {name!r} ({kind}): a scalar; give an array per row')
                    self._bytes += value.nbytes
            else:
                value = _scalar(name, kind, value)
            self._cols[name].append(value)
        self._n += 1
        if self._n >= self.max_rows or self._bytes >= self.max_bytes:
            self._flush()

    def close(self):
        """Write what is buffered; returns [{path (relative to DIR), rows, sha256}] of every file written."""
        if not self._closed:
            if self._n or not self.files:
                self._flush()
            self._closed = True
        return list(self.files)

    def __enter__(self):
        return self

    def __exit__(self, *exc):
        if exc[0] is None:
            self.close()

    def _flush(self):
        stem = self.part if self._k == 0 else f'{self.part}-{self._k:03d}'
        self._k += 1
        scalars = {n: c for n, c in self._cols.items() if not is_array(self.schema[n])}
        arrays = {n: self._cols[n] for n in self.arrays}
        if self.fmt == 'npz':
            self._emit(f'{stem}.npz', npz_bytes({**_np_scalars(scalars, self.schema), **_np_arrays(arrays, self.schema)}))
        else:
            data = {'csv': _csv_bytes, 'jsonl': _jsonl_bytes}.get(self.fmt, _parquet_bytes)(scalars, self.schema, self._n)
            self._emit(stem + EXT[self.fmt], data)
            if arrays:
                self._emit(stem + ARRAYS, npz_bytes(_np_arrays(arrays, self.schema)))
        self._reset()

    def _emit(self, name, data):
        _atomic(self.dir / name, data)
        self.files.append({'path': name, 'rows': self._n, 'sha256': hashlib.sha256(data).hexdigest()})


def _atomic(path, data):
    fd, tmp = tempfile.mkstemp(dir=path.parent, prefix=f'.{path.name}.')
    try:
        with os.fdopen(fd, 'wb') as f:
            f.write(data)
        os.replace(tmp, path)
    except BaseException:
        Path(tmp).unlink(missing_ok=True)
        raise


# ---- formats ---------------------------------------------------------------------------------------------------

def _cell(kind, value):
    if value is None:
        return ''
    if kind == 'bool':
        return 'true' if value else 'false'
    if kind == 'float':
        return repr(value)
    return str(value)


def _csv_bytes(cols, schema, n):
    buf = io.StringIO()
    w = csv.writer(buf, lineterminator='\n')
    names = list(cols)
    w.writerow(names)
    for i in range(n):
        w.writerow([_cell(schema[c], cols[c][i]) for c in names])
    return buf.getvalue().encode()


def _jsonl_bytes(cols, schema, n):
    names = list(cols)
    out = []
    for i in range(n):
        row = {}
        for c in names:
            v = cols[c][i]
            row[c] = json.loads(v) if schema[c] == 'json' and v is not None else v
        out.append(json.dumps(row, separators=(',', ':')) + '\n')
    return ''.join(out).encode()


def _parquet_bytes(cols, schema, n=None):
    import pyarrow as pa
    import pyarrow.parquet as pq
    types = {'int': pa.int64(), 'float': pa.float64(), 'str': pa.string(), 'bool': pa.bool_(), 'json': pa.string()}
    table = pa.table({n: pa.array(v, type=types[schema[n]]) for n, v in cols.items()})
    sink = pa.BufferOutputStream()
    pq.write_table(table, sink)
    return sink.getvalue().to_pybytes()


def _np_scalars(cols, schema):
    out = {}
    for name, values in cols.items():
        kind, nulls = schema[name], np.array([v is None for v in values], dtype=bool)
        if kind == 'float':
            out[name] = np.array([np.nan if v is None else v for v in values], dtype=np.float64)
        elif kind == 'int':
            out[name] = np.array([0 if v is None else v for v in values], dtype=np.int64)
        elif kind == 'bool':
            out[name] = np.array([bool(v) for v in values], dtype=bool)
        else:
            out[name] = np.array(['' if v is None else v for v in values], dtype=np.str_)
        if nulls is not None and nulls.any():
            out[name + NULL] = nulls
    return out


def _np_arrays(cols, schema):
    out = {}
    for name, values in cols.items():
        dtype = np.dtype(schema[name][6:])
        present = [v for v in values if v is not None]
        tail = present[0].shape[1:] if present else ()
        if any(v.shape[1:] != tail for v in present):
            raise SchemaError(f'column {name!r}: rows differ beyond the first dimension '
                              f'({present[0].shape} vs another); give equal channels per row')
        lengths = np.array([-1 if v is None else v.shape[0] for v in values], dtype=np.int64)
        width = int(lengths.max()) if len(lengths) else 0
        fill = np.nan if dtype.kind in 'fc' else 0
        stacked = np.full((len(values), max(width, 0), *tail), fill, dtype=dtype)
        for i, v in enumerate(values):
            if v is not None:
                stacked[i, :v.shape[0]] = v
        out[name] = stacked
        if (lengths != width).any():
            out[name + LEN] = lengths
    return out


def npz_bytes(arrays):
    """An .npz (np.load reads it, no pickle) with fixed zip timestamps, so equal arrays give equal bytes."""
    buf = io.BytesIO()
    with zipfile.ZipFile(buf, 'w', zipfile.ZIP_DEFLATED, compresslevel=1) as z:
        for name, arr in arrays.items():
            info = zipfile.ZipInfo(name + '.npy', date_time=(1980, 1, 1, 0, 0, 0))
            info.compress_type = zipfile.ZIP_DEFLATED
            with z.open(info, 'w', force_zip64=True) as f:
                arr = np.asarray(arr)
                np.lib.format.write_array(f, arr if arr.flags.c_contiguous else np.ascontiguousarray(arr),
                                          allow_pickle=False)
    return buf.getvalue()


# ---- the manifest ----------------------------------------------------------------------------------------------

def write_manifest(dir, schema, files, extra=None):
    """DIR/manifest.json: {schema: crane.data.manifest/1, columns, rows, files: [{path, rows, sha256[, kind]}],
    **extra} (written by atomic rename). Paths are relative to DIR (an absolute one inside DIR is made relative; one
    outside is refused, so no local path ships). A file's ``kind`` defaults to rows; other kinds (traj, events) are
    listed and hashed but not read as rows. Rows count each rows part once (an arrays sidecar is the same rows)."""
    dir = Path(dir)
    files = [{**f, 'path': _relative(dir, f['path'])} for f in files]
    doc = {'schema': SCHEMA, 'columns': check_schema(schema),
           'rows': sum(f['rows'] for f in files if _is_rows(f)), 'files': files, **(extra or {})}
    dir.mkdir(parents=True, exist_ok=True)
    _atomic(Path(dir) / MANIFEST, (json.dumps(doc, indent=1, default=_plain) + '\n').encode())
    return doc


def _relative(dir, path):
    p = Path(path)
    if not p.is_absolute():
        return p.as_posix()
    try:
        return p.resolve().relative_to(dir.resolve()).as_posix()
    except ValueError:
        raise SchemaError(f'{p}: outside {dir}; manifest paths are relative to the dataset dir') from None


def _is_rows(f):
    return f.get('kind', 'rows') == 'rows' and not f['path'].endswith(ARRAYS)


def read_manifest(dir):
    path = Path(dir) / MANIFEST
    try:
        doc = json.loads(path.read_text())
    except FileNotFoundError:
        return None
    except ValueError as error:
        raise SchemaError(f'{path}: not JSON ({error})') from None
    if doc.get('schema') != SCHEMA:
        raise SchemaError(f"{path}: schema {doc.get('schema')!r}, not {SCHEMA}")
    return doc


def verify(dir):
    """The files of DIR's manifest whose sha256 or presence differs: [(path, why)]; [] when whole."""
    doc = read_manifest(dir)
    if doc is None:
        return [(MANIFEST, 'missing')]
    bad = []
    for f in doc['files']:
        path = Path(dir) / f['path']
        if not path.is_file():
            bad.append((f['path'], 'missing'))
        elif hashlib.sha256(path.read_bytes()).hexdigest() != f['sha256']:
            bad.append((f['path'], 'sha256 differs'))
    return bad


# ---- reading ---------------------------------------------------------------------------------------------------

def data_files(path):
    """The row files of a file, a dataset dir (manifest order) or a dir of row files (sorted); sidecars excluded."""
    path = Path(path)
    if path.is_file():
        return [path]
    doc = read_manifest(path) if path.is_dir() else None
    if doc is not None:
        return [path / f['path'] for f in doc['files'] if _is_rows(f)]
    if not path.is_dir():
        raise SchemaError(f'{path}: no such file or dir')
    own = sorted(p for p in path.iterdir() if p.suffix in EXT.values() and not p.name.endswith(ARRAYS)
                 and not p.name.startswith('.'))
    if own:
        return own
    return [f for d in dataset_dirs(path) for f in data_files(d)]


def dataset_dirs(path):
    """The dirs under ``path`` holding a manifest.json (``path`` itself when it does), not looking below one: a
    fetched study (OUT/STUDY/stage/arm/cell/) reads as all its cells. Hidden dirs are skipped."""
    path = Path(path)
    if (path / MANIFEST).is_file():
        return [path]
    found = []
    for here, dirs, files in os.walk(path):
        if MANIFEST in files and Path(here) != path:
            found.append(Path(here))
            dirs[:] = []
        else:
            dirs[:] = sorted(d for d in dirs if not d.startswith('.'))
    return sorted(found)


def columns_of(path):
    """{name: type} of a dataset dir's manifest, else of its first file (csv: inferred from the values)."""
    doc = read_manifest(path) if Path(path).is_dir() else None
    if doc is not None:
        return dict(doc['columns'])
    files = data_files(path)
    return _infer_file(files[0]) if files else {}


def read_rows(path_or_dir):
    """Every row of a file or a dataset dir, as dicts (arrays as numpy arrays). csv, jsonl and npz need only the
    stdlib and numpy; parquet needs pyarrow. A csv without a manifest beside it reads ints, floats and strings by their look."""
    for f in data_files(path_or_dir):
        yield from _read_file(f)


def _types_for(f):
    """The columns of the manifest listing F (in F's dir or the one above, e.g. OUT/rows/part.csv)."""
    for d in (f.parent, f.parent.parent):
        if (d / MANIFEST).is_file():
            doc = read_manifest(d)
            if any((d / e['path']).resolve() == f.resolve() for e in doc['files']):
                return dict(doc['columns'])
    return None


def _read_file(f):
    f = Path(f)
    types = _types_for(f)
    if f.suffix == '.npz':
        yield from _read_npz(f)
        return
    if f.suffix == '.csv':
        rows = _read_csv(f, types)
    elif f.suffix == '.jsonl':
        rows = _read_jsonl(f)
    elif f.suffix == '.parquet':
        _need_parquet(f'reading {f.name}')
        import pyarrow.parquet as pq
        rows = pq.read_table(f).to_pylist()
        if types:
            rows = ({k: json.loads(v) if types.get(k) == 'json' and v is not None else v for k, v in r.items()}
                    for r in rows)
    else:
        raise SchemaError(f'{f}: not a row file (.parquet, .csv, .npz, .jsonl)')
    side = f.with_name(f.name[:-len(f.suffix)] + ARRAYS)
    arrays = _npz_columns(side) if side.is_file() else None
    for i, row in enumerate(rows):
        if arrays:
            row.update({k: v[i] for k, v in arrays.items()})
        yield row


def _read_csv(f, types):
    with open(f, newline='') as fh:
        reader = csv.reader(fh)
        header = next(reader, None) or []
        for values in reader:
            yield {k: _parse(v, (types or {}).get(k)) for k, v in zip(header, values)}


def _read_jsonl(f):
    with open(f) as fh:
        for i, line in enumerate(fh, 1):
            if not line.strip():
                continue
            try:
                row = json.loads(line)
            except ValueError:
                raise SchemaError(f'{f}:{i}: not JSON; a .jsonl holds one JSON object per line') from None
            if not isinstance(row, dict):
                raise SchemaError(f'{f}:{i}: a {type(row).__name__}, not an object; a .jsonl row is one JSON object')
            yield row


def _parse(text, kind):
    if text == '':
        return None
    if kind == 'int':
        return int(text)
    if kind == 'float':
        return float(text)
    if kind == 'bool':
        return text == 'true'
    if kind == 'json':
        return json.loads(text)
    if kind == 'str':
        return text
    for cast in (int, float):
        try:
            return cast(text)
        except ValueError:
            pass
    return {'true': True, 'false': False}.get(text, text)


def _npz_columns(f):
    """{name: list of per-row values} of an npz part (nulls, lengths and JSON strings undone)."""
    with np.load(f, allow_pickle=False) as z:
        raw = {k: z[k] for k in z.files}
    types = _types_for(f) or {}
    cols = {}
    for name, arr in raw.items():
        if LEN in name or NULL in name:
            continue
        kind = types.get(name)
        if name + LEN in raw:
            values = [None if n < 0 else arr[i, :n] for i, n in enumerate(raw[name + LEN])]
        elif kind and is_array(kind) or arr.ndim > 1:
            values = list(arr)
        else:
            values = arr.tolist()
            if kind == 'json':
                values = [None if v == '' else json.loads(v) for v in values]
        if name + NULL in raw:
            values = [None if null else v for v, null in zip(values, raw[name + NULL])]
        cols[name] = values
    return cols


def _read_npz(f):
    cols = _npz_columns(f)
    n = len(next(iter(cols.values()))) if cols else 0
    for i in range(n):
        yield {k: v[i] for k, v in cols.items()}


def infer_type(value):
    """The column type a Python value would need (None for a null)."""
    if value is None:
        return None
    if isinstance(value, (bool, np.bool_)):
        return 'bool'
    if isinstance(value, (int, np.integer)):
        return 'int'
    if isinstance(value, (float, np.floating)):
        return 'float'
    if isinstance(value, str):
        return 'str'
    if isinstance(value, np.ndarray) and value.ndim >= 1:
        return f'array:{value.dtype.name}'
    return 'json'


def widen(a, b):
    """The type that holds both (int and float give float; anything else mixed gives json)."""
    if a is None or a == b:
        return b if a is None else a
    if b is None:
        return a
    if {a, b} == {'int', 'float'}:
        return 'float'
    if is_array(a) and is_array(b):
        return f'array:{np.result_type(np.dtype(a[6:]), np.dtype(b[6:])).name}'
    return 'json'


def infer_schema(rows):
    """{name: type} over rows (first-seen column order); an all-null column is json."""
    schema = {}
    for row in rows:
        for k, v in row.items():
            schema[k] = widen(schema.get(k), infer_type(v))
    return {k: v or 'json' for k, v in schema.items()}


def _infer_file(f):
    if Path(f).suffix == '.npz':
        with np.load(f, allow_pickle=False) as z:
            out = {}
            for k in z.files:
                if LEN in k or NULL in k:
                    continue
                a = z[k]
                out[k] = f'array:{a.dtype.name}' if a.ndim > 1 else {'b': 'bool', 'i': 'int', 'u': 'int', 'f': 'float'}.get(
                    a.dtype.kind, 'str')
            return out
    first = []
    for row in _read_file(f):
        first.append(row)
        if len(first) >= 1000:
            break
    return infer_schema(first)
