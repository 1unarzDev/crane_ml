"""crane data splits: assign a dataset's units (seeds, conditions or timelines) to named splits; held-out stays out.

    crane data splits /tmp/ds --by seed --ratios train=0.7,calib=0.15,test=0.15
    crane data splits /tmp/ds --by timeline --held-out source=real --out /tmp/ds/splits.json

A unit goes to a split by a salted hash of its value, so the same unit always lands in the same split and adding
data never moves one. A unit with any held-out row is ``held-out`` and in no other split. A row is held-out when its
``role``, ``block_role`` or ``tag`` column says held-out, its ``held_out`` column is true, its ``block`` (or seed) is
a held-out block of ``--domain``'s seeds, or it matches a ``--held-out`` expression. Writing over an earlier split
file refuses to move any unit into ``train`` or ``calib`` from another split (or from held-out).
"""

import hashlib
import json
from pathlib import Path

from crane import output
from crane.data import export as X, rows as R
from crane.output import Output

SCHEMA = 'crane.data.splits/1'
HELD = 'held-out'
LEARN = ('train', 'calib')           # what a held-out or test unit must never enter
ROLE_COLUMNS = ('role', 'block_role', 'tag')
PRESETS = {'seed': ('seed',), 'timeline': ('timeline',)}
DEFAULT_RATIOS = 'train=0.7,calib=0.15,test=0.15'


class SplitError(ValueError):
    """A split request that cannot be met; one line naming the fix."""


def parse_ratios(text):
    out = {}
    for part in str(text or DEFAULT_RATIOS).split(','):
        name, eq, val = part.partition('=')
        name = name.strip()
        try:
            out[name] = float(val)
        except ValueError:
            raise SplitError(f"--ratios {text!r}: give NAME=FRACTION,... (e.g. {DEFAULT_RATIOS})") from None
        if not name or not eq or out[name] < 0:
            raise SplitError(f"--ratios {text!r}: give NAME=FRACTION,... (e.g. {DEFAULT_RATIOS})")
    if HELD in out:
        raise SplitError(f'--ratios: {HELD} is set by the held-out tags, not by a ratio; drop it from --ratios')
    total = sum(out.values())
    if abs(total - 1.0) > 1e-6:
        raise SplitError(f'--ratios add up to {total:g}, not 1')
    return out


def _is_held(value):
    return str(value).strip().lower().replace('_', '-') in ('held-out', 'heldout')


def held_rule(domain=None, exprs=()):
    """A predicate row -> bool, and the rule's description (recorded in the split file)."""
    blocks, seeds = set(), set()
    if domain:
        from crane import select
        for name, spec in select.seed_blocks(domain).items():
            if isinstance(spec, dict) and _is_held(spec.get('role', '')):
                blocks.add(name)
                seeds.update(select.parse_seeds(f'block:{name}', domain))
    where = X.parse_where(exprs)

    def held(row):
        if any(_is_held(row.get(c, '')) for c in ROLE_COLUMNS) or row.get('held_out') in (True, 1, 'true'):
            return True
        if blocks and (row.get('block') in blocks or row.get('seed') in seeds):
            return True
        return any(X._match(row, [w]) for w in where)
    rule = [f'{"|".join(ROLE_COLUMNS)} = held-out', 'held_out = true']
    rule += [f'block in {sorted(blocks)} (or their seeds) of domain {domain}'] if blocks else []
    rule += list(exprs or ())
    return held, rule


def unit_columns(by, columns):
    if by in PRESETS:
        cols = PRESETS[by]
    elif by == 'condition':
        cols = tuple(c for c in columns if c.startswith('condition.')) or tuple(c for c in ('cell', 'condition')
                                                                              if c in columns)
        if not cols:
            raise SplitError('--by condition: the rows have no condition.* (or cell / condition) columns; '
                             'use --by seed or --by COLUMN')
    else:
        cols = tuple(c.strip() for c in by.split(','))
    missing = [c for c in cols if c not in columns]
    if missing:
        raise SplitError(f"--by {by}: no column {missing[0]!r}; see crane data schema")
    return cols


def _unit(row, cols):
    return row[cols[0]] if len(cols) == 1 else {c: row.get(c) for c in cols}


def _key(unit):
    return json.dumps(unit, sort_keys=True, default=str)


def bucket(unit, salt, ratios):
    """The split of a unit: its salted sha256 as a fraction in [0, 1), cut at the cumulative ratios."""
    h = int(hashlib.sha256(f'{salt}\0{_key(unit)}'.encode()).hexdigest()[:16], 16) / 2.0 ** 64
    edge = 0.0
    for name, r in ratios.items():
        edge += r
        if h < edge:
            return name
    return list(ratios)[-1]


def splits(sources, by='seed', ratios=None, salt='crane', domain=None, held_out=(), previous=None):
    """The crane.data.splits/1 document of ``sources``."""
    ratios = parse_ratios(ratios) if not isinstance(ratios, dict) else ratios
    held, rule = held_rule(domain, held_out)
    rows = [row for row, _ in X.iter_rows(sources)]
    if not rows:
        raise SplitError('the sources hold no rows')
    columns = list(dict.fromkeys(k for r in rows for k in r))
    cols = unit_columns(by, columns)
    units, counts, held_units = {}, {}, set()
    for row in rows:
        k = _key(_unit(row, cols))
        units.setdefault(k, _unit(row, cols))
        counts[k] = counts.get(k, 0) + 1
        if held(row):
            held_units.add(k)
    assign = {k: HELD if k in held_units else bucket(u, salt, ratios) for k, u in units.items()}
    if previous:
        _guard(previous, assign, units)
    order = sorted(units, key=lambda k: (str(type(units[k])), units[k] if not isinstance(units[k], dict) else k))
    out = {name: {'units': [], 'rows': 0} for name in [*ratios, HELD]}
    for k in order:
        out[assign[k]]['units'].append(units[k])
        out[assign[k]]['rows'] += counts[k]
    n = len(units)
    return {'schema': SCHEMA, 'source': [str(Path(s).expanduser().resolve()) for s in sources], 'by': by,
            'columns': list(cols), 'ratios': ratios, 'salt': salt, 'held_out_rule': rule,
            'units': n, 'rows': len(rows),
            'splits': {name: {**s, 'n_units': len(s['units']), 'fraction': round(len(s['units']) / n, 4)}
                       for name, s in out.items()}}


def _guard(previous, assign, units):
    """Refuse when a unit an earlier split file put outside train/calib would now enter train/calib."""
    was = {_key(u): name for name, s in (previous.get('splits') or {}).items() for u in s.get('units', ())}
    for k, now in assign.items():
        before = was.get(k)
        if before is not None and before not in LEARN and now in LEARN:
            raise SplitError(f'unit {units[k]!r} was {before} in the earlier split file and would move to {now}; '
                             'keep that file\'s --salt and --ratios, or write a new --out')


def members(spec):
    """'FILE:NAME' -> a predicate row -> bool: the row's unit is in split NAME of split file FILE."""
    path, _, name = str(spec).rpartition(':')
    try:
        doc = json.loads(Path(path).expanduser().read_text())
    except (OSError, ValueError) as error:
        raise SplitError(f'--split {spec}: cannot read {path or "FILE"} ({error}); give FILE:NAME') from None
    if name not in doc.get('splits', {}):
        raise SplitError(f"--split {spec}: no split {name!r}; splits: {', '.join(doc.get('splits', {}))}")
    keys, cols = {_key(u) for u in doc['splits'][name]['units']}, tuple(doc['columns'])
    return lambda row: _key(_unit(row, cols)) in keys


def main(args):
    """crane data splits SRC... --by B --ratios R [--held-out EXPR]... [--domain D] [--salt S] [--out FILE]"""
    out = getattr(args, 'output', None) or Output.of(args)
    first = Path(args.sources[0]).expanduser()
    dest = Path(args.out).expanduser() if args.out else (first / 'splits.json' if (first / R.MANIFEST).is_file()
                                                         else None)
    if dest is None:
        return out.usage('data splits', '--out is required unless the source is one dataset dir',
                         f'crane data splits {args.sources[0]} --out splits.json')
    previous = None
    if dest.is_file():
        try:
            previous = json.loads(dest.read_text())
        except ValueError:
            return out.usage('data splits', f'{dest} exists and is not JSON', 'pick another --out')
    try:
        doc = splits(args.sources, args.by, args.ratios, args.salt, args.domain, args.held_out or (), previous)
    except (SplitError, X.SourceError, R.SchemaError) as error:
        return out.usage('data splits', str(error))
    except LookupError as error:
        return out.usage('data splits', str(error).strip('"\''), 'crane data splits --help')
    if not out.dry_run:
        dest.parent.mkdir(parents=True, exist_ok=True)
        R._atomic(dest, (json.dumps(doc, indent=1, default=str) + '\n').encode())
    if out.doc({**doc, 'out': str(dest), 'written': not out.dry_run}):
        return output.OK
    out.say(f"crane data splits: {doc['units']} units by {doc['by']} ({', '.join(doc['columns'])}), "
            f"{doc['rows']} rows" + (' (dry run: nothing written)' if out.dry_run else f' -> {dest}'))
    out.table([(name, f"{s['n_units']} units", f"{s['rows']} rows", f"{s['fraction']:.1%}")
               for name, s in doc['splits'].items()])
    out.line(f"  held-out rule: {'; '.join(doc['held_out_rule'])}")
    out.next(f'crane data export {args.sources[0]} --split {dest}:{next(iter(doc["ratios"]))} --out DIR')
    return output.OK
