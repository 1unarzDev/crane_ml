"""crane data export / crane data schema: run dirs, batch outputs and datasets to one columnar dataset.

    crane data export ~/.cache/crane/runs/toy/T0/base-1a2b3c4d --format npz --out /tmp/ds
    crane data export /workspace/out/capxfer --select seed,timeline,obs --where 'label.fault!=none' --out /tmp/ds
    crane data export /tmp/ds --split /tmp/ds/splits.json:train --out /tmp/train
    crane data schema /tmp/ds

A source is a dataset (a dir with manifest.json, or one row file: .parquet .csv .npz .jsonl), a run directory (run.json: one metrics-first row
per run, with its recorded arrays as ``obs`` / ``traj`` columns) or any dir holding those (walked; ``.stale-N`` and
partial dirs are skipped).

Rows that splits would call held-out (crane.study.guard's roles: role / block_role / tag held-out, held_out true)
are refused unless ``confirm`` (``--confirm``): pick the others with --where 'role!=held-out' or --split FILE:train.
With confirm they are written and counted (``held_count``). Non-finite floats are noted per column (``nonfinite``).
The manifest names its sources as given, with the home dir as ``~`` (no /home path ships).
"""

import json
import math
import operator
import os
from pathlib import Path

from crane import output
from crane.data import rows as R
from crane.output import Output

SKIP = ('.partial-', '.stale-')
FIRST = ('run', 'variant', 'seed', 'outcome', 'sim_s')


class SourceError(ValueError):
    """A source that holds no rows, or a --where / --select that cannot be read; one line naming the fix."""


# ---- sources ---------------------------------------------------------------------------------------------------

def find(src):
    """[(kind, path)] under SRC: ('dataset', dir or file) and ('run', run dir), in a stable order."""
    p = Path(src).expanduser()
    if p.is_file():
        if p.name == 'run.json':
            return [('run', p.parent)]
        if p.suffix in R.EXT.values() and not p.name.endswith(R.ARRAYS):
            return [('dataset', p)]
        raise SourceError(f'{p}: not a row file (.parquet, .csv, .npz, .jsonl) or a run.json')
    if not p.is_dir():
        raise SourceError(f'{p}: no such file or dir')
    if (p / R.MANIFEST).is_file():
        return [('dataset', p)]
    if (p / 'run.json').is_file():
        return [('run', p)]
    found = []
    for root, dirs, files in os.walk(p):
        dirs[:] = sorted(d for d in dirs if not d.startswith('.') and not any(s in d for s in SKIP))
        here = Path(root)
        if R.MANIFEST in files:
            found.append(('dataset', here))
            dirs[:] = []
        elif 'run.json' in files:
            found.append(('run', here))
            dirs[:] = []
    if not found:
        found = [('dataset', p)] if R.data_files(p) else []
    return found


def run_row(run_dir, root=None):
    """One metrics-first row of a fastsim run directory (its run.json, metrics.json and record/*.npz)."""
    run_dir = Path(run_dir)
    run = json.loads((run_dir / 'run.json').read_text())
    prov = run.get('provenance') or {}
    git = (prov.get('git') or {}).get('core') or {}
    ident = run.get('identity') or {}
    sc = run.get('scenario') or {}
    row = {'run': str(run_dir.relative_to(root)) if root and run_dir != Path(root) else run_dir.name,
           'variant': prov.get('variant'), 'seed': run.get('seed'), 'outcome': run.get('outcome'),
           'sim_s': run.get('sim_s'), 'time_limit_s': run.get('time_limit_s'),
           'min_clearance_m': run.get('min_clearance_m'), 'contacts': len(run.get('contacts') or ()),
           'flags': ','.join(run.get('flags') or ()), 'error': (run.get('error') or {}).get('error'),
           'domain': (run.get('adapter') or {}).get('domain'), 'tier': prov.get('tier'),
           'scenario': sc.get('name'), 'pair_key': sc.get('pair_key'), 'run_sha256': ident.get('run_sha256'),
           'commit': git.get('commit')}
    for k, v in (run.get('condition') or {}).items():
        row[f'condition.{k}'] = v if not isinstance(v, (dict, list)) else json.dumps(v, sort_keys=True)
    try:
        metrics = json.loads((run_dir / 'metrics.json').read_text())
    except (OSError, ValueError):
        metrics = {}
    from fastsim.pairs import flat_numbers
    for k, v in flat_numbers(metrics).items():
        row[k if k not in row else f'metric.{k}'] = v
    from crane.data.recorder import load_run
    for kind, arrays in load_run(run_dir).items():
        row[kind] = arrays[kind]
        if kind == 'obs':
            row.update({f'label.{k}': v for k, v in (arrays.get('labels') or {}).items()})
    return row


def iter_rows(sources):
    """Every row of every source, with ``meta`` (the arrays' channels and rates seen) filled as it goes."""
    meta = {}
    for src in sources:
        root = Path(src).expanduser()
        items = find(src)
        if not items:
            raise SourceError(f'{root}: no run dirs (run.json) and no datasets (manifest.json) under it')
        for kind, path in items:
            if kind == 'run':
                rec = (json.loads((path / 'run.json').read_text()).get('record') or {})
                for k in ('obs', 'traj'):
                    if k in rec:
                        meta.setdefault(k, rec[k])
                yield run_row(path, root if root.is_dir() else None), meta
            else:
                doc = R.read_manifest(path) if path.is_dir() else None
                if doc and doc.get('kind') in ('obs', 'traj') and doc.get('channels'):
                    meta.setdefault(doc['kind'], {k: doc.get(k) for k in ('channels', 'hz', 'dtype', 'window')})
                for row in R.read_rows(path):
                    yield row, meta


# ---- --select and --where --------------------------------------------------------------------------------------

OPS = [('!=', operator.ne), ('>=', operator.ge), ('<=', operator.le), ('=', operator.eq), ('>', operator.gt),
       ('<', operator.lt)]


def parse_where(exprs):
    """['seed>=10', 'arm=a,b'] -> [(column, op, values)]; '=' and '!=' take a comma list (any of / none of)."""
    out = []
    for e in exprs or ():
        for sym, fn in OPS:
            col, hit, val = e.partition(sym)
            if hit and col.strip():
                values = [_num(v.strip()) for v in val.split(',')] if sym in ('=', '!=') else [_num(val.strip())]
                out.append((col.strip(), sym, fn, values))
                break
        else:
            raise SourceError(f"--where {e!r}: give COLUMN OP VALUE with OP one of = != > >= < <= (e.g. 'seed>=10')")
    return out


def _num(text):
    for cast in (int, float):
        try:
            return cast(text)
        except ValueError:
            pass
    return {'true': True, 'false': False, 'null': None}.get(text, text)


def _match(row, where):
    for col, sym, fn, values in where:
        have = row.get(col)
        if sym == '=':
            ok = any(_eq(have, v) for v in values)
        elif sym == '!=':
            ok = not any(_eq(have, v) for v in values)
        else:
            try:
                ok = have is not None and fn(have, values[0])
            except TypeError:
                ok = fn(str(have), str(values[0]))
        if not ok:
            return False
    return True


def _eq(a, b):
    return a == b or (a is not None and str(a) == str(b))


def parse_select(text):
    cols = [c.strip() for part in (text or []) for c in str(part).split(',') if c.strip()]
    return cols or None


def _pick(row, select):
    return row if select is None else {c: row.get(c) for c in select}


# ---- export ----------------------------------------------------------------------------------------------------

def _order(schema):
    head = [c for c in FIRST if c in schema]
    return {c: schema[c] for c in head + [c for c in schema if c not in head]}


def export(sources, out, fmt='npz', select=None, where=None, part=None, split=None, confirm=False):
    """Write the rows of ``sources`` (filtered by --where and a split, selected) to ``out``; returns its manifest.
    Held-out rows raise SourceError unless ``confirm``."""
    where = parse_where(where)
    inside = None
    if split:
        from crane.data.splits import members
        inside = members(split)

    def keep(row):
        return _match(row, where) and (inside is None or inside(row))
    select = parse_select(select)
    from crane.data.splits import held_rule
    held = held_rule()[0]
    schema, n, seen, n_held, nonfinite = {}, 0, set(), 0, {}
    for row, _ in iter_rows(sources):        # pass 1: the columns and their types
        seen.update(row)
        if keep(row):
            n_held += bool(held(row))
            for k, v in _pick(row, select).items():
                schema[k] = R.widen(schema.get(k), R.infer_type(v))
                if isinstance(v, float) and not math.isfinite(v):
                    nonfinite[k] = nonfinite.get(k, 0) + 1
            n += 1
    unknown = [c for c in [*(select or ()), *(c for c, *_ in where)] if c not in seen]
    if unknown:
        raise SourceError(f"no column {unknown[0]!r} in the sources; list them with crane data schema {sources[0]}")
    if not n:
        raise SourceError('no rows match' + (' --where' if where else '') + '; check the columns with crane data schema')
    if n_held and not confirm:
        raise SourceError(f'{n_held} of {n} rows are held-out (role/block_role/tag held-out or held_out true) and carry '
                          "their outcomes; fix: --where 'role!=held-out' (or --split FILE:train), or --confirm to "
                          'export them')
    schema = _order({k: v or 'json' for k, v in schema.items()})
    meta = {}
    with R.RowWriter(out, schema, fmt=fmt, part=part) as w:
        for row, meta in iter_rows(sources):  # pass 2: the rows
            if keep(row):
                w.write(_pick(row, select))
    extra = {'source': [_shown(s) for s in sources], 'format': w.fmt,
             'select': select, 'split': split, 'where': [f'{c}{s}{",".join(map(str, v))}' for c, s, _, v in where],
             'arrays': {k: v for k, v in meta.items() if k in schema}, 'held_count': n_held, 'nonfinite': nonfinite}
    return R.write_manifest(out, schema, w.files, extra)


def _shown(path):
    """A source path as the manifest records it: relative stays relative, the home dir becomes ~."""
    p = Path(path).expanduser()
    if not p.is_absolute():
        return p.as_posix()
    try:
        return '~/' + p.resolve().relative_to(Path.home().resolve()).as_posix()
    except ValueError:
        return p.as_posix()


def schema_of(sources):
    """{columns: {name: type}, rows, files, nulls: {name: n}} of sources (a dataset's manifest when it has one)."""
    if len(sources) == 1 and find(sources[0]) == [('dataset', Path(sources[0]).expanduser())] \
            and Path(sources[0]).expanduser().is_dir():
        doc = R.read_manifest(Path(sources[0]).expanduser())
        if doc:
            return {'columns': doc['columns'], 'rows': doc['rows'], 'files': len(doc['files']),
                    'arrays': doc.get('arrays') or ({doc['kind']: {k: doc.get(k) for k in ('channels', 'hz', 'dtype')}}
                                                    if doc.get('kind') else {})}
    schema, n, nulls, meta = {}, 0, {}, {}
    for row, meta in iter_rows(sources):
        n += 1
        for k, v in row.items():
            schema[k] = R.widen(schema.get(k), R.infer_type(v))
            nulls[k] = nulls.get(k, 0) + (v is None)
    return {'columns': _order({k: v or 'json' for k, v in schema.items()}), 'rows': n, 'files': None,
            'arrays': dict(meta)}


# ---- the verbs -------------------------------------------------------------------------------------------------

def main(args):
    """crane data export SRC... --format F [--select COLS] [--where EXPR]... --out DIR"""
    out = getattr(args, 'output', None) or Output.of(args)
    dest = Path(args.out).expanduser()
    if dest.exists() and (not dest.is_dir() or any(dest.iterdir())):
        return out.usage('data export', f'{dest} exists and is not empty (a dataset is written once)',
                         f'pick another --out, or rm -r {dest}')
    from crane.data.splits import SplitError
    try:
        doc = export(args.sources, dest, args.format, args.select, args.where, split=args.split,
                     confirm=getattr(args, 'confirm', False))
    except (SourceError, R.SchemaError, SplitError) as error:
        return out.usage('data export', str(error))
    if out.doc({'schema': 'crane.data.export/1', 'out': str(dest), 'manifest': str(dest / R.MANIFEST),
                'rows': doc['rows'], 'columns': doc['columns'], 'files': doc['files'],
                'held_count': doc['held_count'], 'nonfinite': doc['nonfinite']}):
        return output.OK
    size = sum((dest / f['path']).stat().st_size for f in doc['files'])
    out.say(f"crane data export: {doc['rows']} rows, {len(doc['columns'])} columns, {len(doc['files'])} file(s), "
            f"{size / 1e6:.1f} MB, {doc['format']}")
    out.table([(f['path'], f"{f['rows']} rows", f['sha256'][:12]) for f in doc['files']])
    out.line(f'  manifest  {dest / R.MANIFEST}')
    if doc['held_count']:
        out.line(f"  note: {doc['held_count']} held-out row(s) exported (--confirm) with their outcomes; "
                 'keep this dataset out of training')
    if doc['nonfinite']:
        out.line('  note: non-finite floats: ' + ', '.join(f'{k} {v}' for k, v in doc['nonfinite'].items())
                 + ' (nan/inf; a cluster metrics.json must not carry them)')
    out.next(f'crane data schema {dest}')
    return output.OK


def schema_main(args):
    """crane data schema SRC..."""
    out = getattr(args, 'output', None) or Output.of(args)
    try:
        doc = schema_of(args.sources)
    except (SourceError, R.SchemaError) as error:
        return out.usage('data schema', str(error))
    if out.doc({'schema': 'crane.data.schema/1', 'sources': [str(s) for s in args.sources], **doc}):
        return output.OK
    files = f", {doc['files']} file(s)" if doc['files'] is not None else ''
    out.say(f"crane data schema: {doc['rows']} rows, {len(doc['columns'])} columns{files}")
    arrays = doc.get('arrays') or {}
    out.table([(name, kind, _array_note(arrays.get(name))) for name, kind in doc['columns'].items()])
    first = args.sources[0]
    out.next(f'crane data export {first} --format npz --out /tmp/ds' if not (Path(first).expanduser() / R.MANIFEST)
             .is_file() else f'crane data splits {first} --by seed')
    return output.OK


def _array_note(meta):
    if not meta:
        return ''
    chans = meta.get('channels') or []
    shown = ' '.join(chans[:6]) + (f' ... ({len(chans)})' if len(chans) > 6 else '')
    return f"{meta.get('hz')} Hz  {shown}".strip()
