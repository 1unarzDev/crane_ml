"""crane.study/1: the YAML study spec, read and checked. Every error is one line naming the key and the fix.

    study = load('examples/studies/toy_paired.yaml')
    study.arms['wide'].set          # {'adapter.options.reach_m': 2.5}
    study.blocks['test'].role       # 'held-out'

The spec's keys (Docs/Study.md has the full table):
    name, domain, tier | tiers, record, crn, targets, baseline, metrics, scorer, timeout_s, keep_runs,
    arms, axes, conditions, splits, seeds, stages, pack, campaign, purpose, expect, pin, kind
"""

from dataclasses import dataclass, field
import csv
import math
from pathlib import Path
import re

from crane import ids, select, tiers as tierlib

SCHEMA = 'crane.study/1'
ROLES = ('dev', 'check', 'train', 'calib', 'held-out')
RECORDS = ('metrics', 'events', 'traj_decimated', 'obs')
NAME = re.compile(r'^[A-Za-z0-9][A-Za-z0-9_-]{0,47}$')
TOP = {'schema', 'name', 'domain', 'tier', 'tiers', 'record', 'crn', 'targets', 'baseline', 'metrics', 'scorer',
       'timeout_s', 'keep_runs', 'arms', 'axes', 'conditions', 'splits', 'seeds', 'stages', 'pack', 'campaign',
       'purpose', 'expect', 'pin', 'kind', 'report', 'description'}
DEFAULT_STAGE = 'main'
RESERVED = {'study', 'stage', 'arm', 'cell', 'pair', 'block', 'split', 'role', 'tier', 'seed', 'run_seed', 'condition',
            'outcome', 'error', 'wall_s', 'commit', 'card_sha', 'touched_held_out', 'succeeded'}   # row columns


class StudyError(ValueError):
    """A spec that cannot be read; the message is one line naming the key and the fix."""


@dataclass
class Arm:
    name: str
    kind: str                      # sim | task
    set: dict = field(default_factory=dict)       # dotted key -> value (config overrides, or task args)
    card: str = None
    task: str = None               # 'module:function' or 'file.py:function'
    argv: list = None
    args: dict = field(default_factory=dict)
    timeout_s: float = None
    source: str = None             # the sample generator it came from
    raw: dict = field(default_factory=dict)       # its definition, hashed into the cell


@dataclass
class Axis:
    name: str
    levels: dict                   # level name -> {dotted key: value}
    card: bool = False             # level values are card refs


@dataclass
class Block:
    name: str
    role: str
    seeds: list


@dataclass
class Stage:
    name: str
    tiers: list = None
    blocks: list = None
    splits: list = None
    arms: list = None
    set: dict = field(default_factory=dict)
    after: list = field(default_factory=list)
    select: dict = None
    only: list = None              # [{arm: A, AXIS: LEVEL}]: the combinations a promoted stage runs
    record: dict = None
    pack: dict = None


@dataclass
class Study:
    name: str
    path: Path
    base: Path
    raw: dict
    domain: str = None
    arms: dict = field(default_factory=dict)
    axes: list = field(default_factory=list)
    conditions: list = None        # [{'id': ..., 'row': {col: value}, 'set': {key: value}, 'split': name}]
    condition_columns: list = field(default_factory=list)
    splits: dict = field(default_factory=dict)    # split name -> role
    blocks: dict = field(default_factory=dict)
    stages: dict = field(default_factory=dict)
    tiers: list = field(default_factory=lambda: ['T0'])
    record: dict = field(default_factory=lambda: {'level': 'metrics'})
    crn: str = 'plain'
    targets: Path = None
    baseline: str = None
    metrics: list = None
    scorer: dict = None
    timeout_s: float = None
    keep_runs: bool = False
    pack: dict = field(default_factory=dict)
    cluster: dict = field(default_factory=dict)  # kind, campaign, purpose, expect, pin
    report: dict = field(default_factory=dict)

    @property
    def ref(self):
        """The spec path as a command names it: relative to the current dir when under it."""
        import os
        rel = os.path.relpath(self.path)
        return rel if not rel.startswith('..') else str(self.path)

    # the plan side (crane/study/plan.py)
    def cells(self, stage=None, block=None, out=None):
        """Every Cell of the stage (all stages: None) and block; a stage that selects from another's results needs
        ``out`` (its rows) and is skipped without it."""
        from crane.study import plan
        return plan.cells(self, stage, block, out)

    def jobs(self, stage=None, block=None, out=None):
        from crane.study import plan
        return plan.jobs(self, stage, block, out)

    def order(self):
        """Stage names, each after the stages it depends on."""
        done, out = set(), []

        def visit(name):
            if name not in done:
                done.add(name)
                for dep in self.stages[name].after:
                    visit(dep)
                out.append(name)
        for name in self.stages:
            visit(name)
        return out


def _err(spec, where, what, fix=None):
    text = f'{spec}: {where}: {what}' + (f'; fix: {fix}' if fix else '')
    return StudyError(' '.join(text.split()))


def _yaml(text):
    try:
        import yaml
        return yaml.safe_load(text)
    except ImportError:
        from crane import miniyaml
        return miniyaml.safe_load(text)


def load(path):
    """The Study a crane.study/1 YAML describes; StudyError (one line) when it cannot."""
    path = Path(path).expanduser()
    label = path.name
    try:
        raw = _yaml(path.read_text())
    except OSError as error:
        raise StudyError(f'{path}: {error.strerror or error}; fix: crane study init > {label}') from None
    except Exception as error:  # noqa: BLE001  the YAML parser's own error, on one line
        raise _err(label, 'YAML', str(error).splitlines()[0]) from None
    if not isinstance(raw, dict):
        raise _err(label, 'top level', 'is not a mapping', 'crane study init (an example spec)')
    return parse(raw, path)


def parse(raw, path):
    path = Path(path).resolve()
    s = path.name
    unknown = sorted(set(raw) - TOP)
    if unknown:
        raise _err(s, unknown[0], 'unknown key', f'use one of {", ".join(sorted(TOP - {"description"}))}')
    if raw.get('schema', SCHEMA) != SCHEMA:
        raise _err(s, 'schema', f'{raw["schema"]!r} is not {SCHEMA}', f'schema: {SCHEMA}')
    name = str(raw.get('name') or path.stem)
    if not NAME.match(name):
        raise _err(s, 'name', f'{name!r} must be letters, digits, _ or - (48 at most)')
    st = Study(name=name, path=path, base=path.parent, raw=raw, domain=raw.get('domain'))
    if st.domain is not None:
        try:
            select.domain_config(st.domain)
        except select.SelectError as error:
            raise _err(s, 'domain', str(error)) from None
    st.tiers = _tiers(s, 'tiers', raw.get('tiers', raw.get('tier', 'T0')))
    st.record = _record(s, 'record', raw.get('record', 'metrics'))
    st.crn = raw.get('crn', 'plain')
    if st.crn not in ('plain', 'derive'):
        raise _err(s, 'crn', f'{st.crn!r}', 'crn: plain (the seed is the run seed) or derive (derive_seed of seed '
                   'and condition)')
    st.timeout_s = _num(s, 'timeout_s', raw.get('timeout_s'))
    st.keep_runs = bool(raw.get('keep_runs', False))
    if raw.get('targets') is not None:
        st.targets = (st.base / str(raw['targets'])).resolve()
    st.metrics = _names(s, 'metrics', raw.get('metrics'))
    st.scorer = _scorer(s, raw.get('scorer'))
    st.pack = _pack(s, 'pack', raw.get('pack'))
    st.report = dict(raw.get('report') or {})
    st.cluster = _cluster(s, raw)
    st.arms = _arms(s, st, raw.get('arms'))
    st.axes = _axes(s, st, raw.get('axes'))
    _conditions(s, st, raw.get('conditions'), raw.get('splits'))
    st.blocks = _blocks(s, st, raw.get('seeds'))
    st.stages = _stages(s, st, raw.get('stages'))
    if st.crn == 'derive' and any(a.kind == 'sim' for a in st.arms.values()):
        raise _err(s, 'crn', 'derive is for task arms: a sim episode draws its scenario from the seed itself',
                   'crn: plain (sim streams are already derived from the seed, the same in every arm)')
    st.baseline = raw.get('baseline', st.report.get('baseline')) or next(iter(st.arms))
    if st.baseline not in st.arms:
        raise _err(s, 'baseline', f'no arm {st.baseline!r}', f'one of {", ".join(st.arms)}')
    if any(b.role == 'held-out' for b in st.blocks.values()) or 'held-out' in st.splits.values():
        if st.targets is None:
            raise _err(s, 'targets', 'a held-out block or split needs a targets file',
                       'targets: PATH (the committed predictions/targets, relative to the spec)')
    return st


def _tiers(s, where, value):
    values = value if isinstance(value, list) else [value]
    try:
        out = [tierlib.normalise(v) for v in values]
    except ValueError as error:
        raise _err(s, where, str(error)) from None
    if not out:
        raise _err(s, where, 'is empty', 'tiers: [T0]')
    return out


def _record(s, where, value):
    rec = {'level': value} if isinstance(value, str) else dict(value or {})
    level = rec.get('level', 'metrics')
    if level not in RECORDS:
        raise _err(s, where, f'level {level!r}', f'one of {", ".join(RECORDS)}')
    bad = sorted(set(rec) - {'level', 'hz', 'until_s'})
    if bad:
        raise _err(s, f'{where}.{bad[0]}', 'unknown key', 'record: {level, hz, until_s}')
    rec['hz'] = _num(s, f'{where}.hz', rec.get('hz', 2.0 if level == 'traj_decimated' else None))
    rec['until_s'] = _num(s, f'{where}.until_s', rec.get('until_s'))
    rec['level'] = level
    return rec


def _num(s, where, value):
    if value is None:
        return None
    try:
        v = float(value)
    except (TypeError, ValueError):
        raise _err(s, where, f'{value!r} is not a number') from None
    if not math.isfinite(v) or v <= 0:
        raise _err(s, where, f'{value!r} must be a positive number')
    return v


def _names(s, where, value):
    if value is None:
        return None
    if not isinstance(value, list) or not all(isinstance(v, str) and v for v in value):
        raise _err(s, where, 'must be a list of names', f'{where}: [succeeded, run.sim_s]')
    return list(value)


def _ref(s, where, value):
    if not isinstance(value, str) or ':' not in value:
        raise _err(s, where, f'{value!r} is not a callable ref', f'{where}: module:function or file.py:function')
    return value


def _scorer(s, value):
    if value is None:
        return None
    spec = {'ref': value} if isinstance(value, str) else dict(value)
    bad = sorted(set(spec) - {'ref', 'args'})
    if bad:
        raise _err(s, f'scorer.{bad[0]}', 'unknown key', 'scorer: {ref: module:function, args: {...}}')
    return {'ref': _ref(s, 'scorer.ref', spec.get('ref')), 'args': dict(spec.get('args') or {})}


def _pack(s, where, value):
    pack = dict(value or {})
    bad = sorted(set(pack) - {'unit_s', 'target_s', 'local_chunk'})
    if bad:
        raise _err(s, f'{where}.{bad[0]}', 'unknown key', f'{where}: {{unit_s, target_s, local_chunk}}')
    out = {'unit_s': _num(s, f'{where}.unit_s', pack.get('unit_s')),
           'target_s': _num(s, f'{where}.target_s', pack.get('target_s'))}
    if pack.get('local_chunk') is not None:
        out['local_chunk'] = int(_num(s, f'{where}.local_chunk', pack['local_chunk']))
    return {k: v for k, v in out.items() if v is not None}


def _cluster(s, raw):
    pin = dict(raw.get('pin') or {})
    bad = sorted(set(pin) - {'hosts', 'tags', 'gpu', 'vram_gb'})
    if bad:
        raise _err(s, f'pin.{bad[0]}', 'unknown key', 'pin: {hosts: [...], tags: [...], gpu, vram_gb}')
    for key in ('hosts', 'tags'):
        if key in pin and not isinstance(pin[key], list):
            raise _err(s, f'pin.{key}', 'must be a list', f'pin: {{{key}: [a, b]}}')
    return {'kind': raw.get('kind', 'crane_batch'), 'campaign': raw.get('campaign'), 'purpose': raw.get('purpose'),
            'expect': raw.get('expect'), 'pin': pin}


def _flat(value, prefix='', leaf=None):
    """{'a': {'b': 1}} or {'a.b': 1} -> {'a.b': 1} (lists, and dicts ``leaf`` accepts, are leaves)."""
    out = {}
    for k, v in (value or {}).items():
        key = f'{prefix}.{k}' if prefix else str(k)
        if isinstance(v, dict) and v and not (leaf and leaf(v)):
            out.update(_flat(v, key, leaf))
        else:
            out[key] = v
    return out


ARM_KEYS = {'set', 'card', 'task', 'argv', 'args', 'timeout_s', 'sample'}
SAMPLERS = ('uniform', 'log_uniform', 'int', 'choice')


def _arms(s, st, value):
    if not value:
        if not st.domain:
            raise _err(s, 'arms', 'no arms and no domain', 'arms: {base: {set: {}}} with domain: D, or a task arm')
        value = {'base': {}}
    if isinstance(value, list):
        value = {str(a): {} for a in value}
    if not isinstance(value, dict):
        raise _err(s, 'arms', 'must be a mapping of name -> arm', 'arms: {base: {}, wide: {set: {key: value}}}')
    arms = {}
    for name, spec in value.items():
        where = f'arms.{name}'
        if not NAME.match(str(name)):
            raise _err(s, where, 'an arm name is letters, digits, _ or -')
        spec = dict(spec or {})
        bad = sorted(set(spec) - ARM_KEYS)
        if bad:
            raise _err(s, f'{where}.{bad[0]}', 'unknown key', f'an arm takes {", ".join(sorted(ARM_KEYS))}')
        sample = spec.pop('sample', None)
        if sample is None:
            arms[str(name)] = _arm(s, st, where, str(name), spec)
            continue
        sample = dict(sample)
        n = sample.get('n')
        if not isinstance(n, int) or n < 1:
            raise _err(s, f'{where}.sample.n', f'{n!r}', 'sample: {n: 8, seed: 0, set: {key: {uniform: [lo, hi]}}}')
        space = _flat(sample.get('set'), leaf=lambda v: len(v) == 1 and next(iter(v)) in SAMPLERS)
        if not space:
            raise _err(s, f'{where}.sample.set', 'is empty', 'sample: {set: {key: {uniform: [lo, hi]}}}')
        for key, dist in space.items():
            ok = isinstance(dist, dict) and len(dist) == 1 and next(iter(dist)) in SAMPLERS
            if not ok:
                raise _err(s, f'{where}.sample.set.{key}', f'{dist!r}', f'one of {{{", ".join(SAMPLERS)}: ...}}')
        seed = int(sample.get('seed', 0))
        width = len(str(n - 1))
        for i in range(n):
            drawn = {k: _draw(s, f'{where}.sample.set.{k}', d, ids.derive_seed(seed, st.name, str(name), i, k))
                     for k, d in space.items()}
            one = {**spec, 'set': {**_flat(spec.get('set')), **drawn}}
            arm = _arm(s, st, where, f'{name}-{i:0{width}d}', one)
            arm.source = str(name)
            arms[arm.name] = arm
    return arms


def _draw(s, where, dist, seed):
    import random
    (kind, a), = dist.items()
    r = random.Random(seed)
    if kind == 'choice':
        if not isinstance(a, list) or not a:
            raise _err(s, where, 'choice wants a non-empty list')
        return a[r.randrange(len(a))]
    if not isinstance(a, list) or len(a) != 2:
        raise _err(s, where, f'{kind} wants [lo, hi]')
    lo, hi = a
    if kind == 'int':
        return r.randint(int(lo), int(hi))
    if kind == 'log_uniform':
        if lo <= 0 or hi <= 0:
            raise _err(s, where, 'log_uniform wants lo, hi > 0')
        return round(math.exp(r.uniform(math.log(lo), math.log(hi))), 6)
    return round(r.uniform(float(lo), float(hi)), 6)


def _arm(s, st, where, name, spec):
    task, argv = spec.get('task'), spec.get('argv')
    if task is not None and argv is not None:
        raise _err(s, where, 'has both task and argv', 'give one: task: module:function, or argv: [cmd, ...]')
    if argv is not None and (not isinstance(argv, list) or not argv):
        raise _err(s, f'{where}.argv', 'must be a non-empty list', "argv: [python3, run.py, '{args}']")
    kind = 'task' if task is not None or argv is not None else 'sim'
    if kind == 'sim' and not st.domain:
        raise _err(s, where, 'a sim arm needs a domain', 'domain: D at the top, or task: module:function')
    if task is not None:
        _ref(s, f'{where}.task', task)
    args = spec.get('args') or {}
    if not isinstance(args, dict):
        raise _err(s, f'{where}.args', 'must be a mapping (JSON args)')
    sets = spec.get('set') or {}
    if not isinstance(sets, dict):
        raise _err(s, f'{where}.set', 'must be a mapping of key -> value', f'{where}: {{set: {{a.b: 1}}}}')
    card = spec.get('card')
    if card is not None and kind == 'sim':
        _card_path(s, st, f'{where}.card', card)
    return Arm(name=name, kind=kind, set=_flat(sets), card=None if card is None else str(card), task=task,
               argv=None if argv is None else [str(a) for a in argv], args=dict(args),
               timeout_s=_num(s, f'{where}.timeout_s', spec.get('timeout_s')),
               raw={k: v for k, v in spec.items()})


def card_keys(st):
    """The config keys a card path binds to: the domain's card.keys, else its view.card list."""
    cfg = select.domain_config(st.domain) if st.domain else {}
    keys = (cfg.get('card') or {}).get('keys') or (cfg.get('view') or {}).get('card') or []
    return [str(k) for k in keys]


def _card_path(s, st, where, ref):
    """A card ref to its file: a path (spec dir, then repo root), else DIR/REF.json from the domain's card.dir."""
    from crane import paths
    if not st.domain:
        raise _err(s, where, 'a card needs a domain', 'domain: D')
    if not card_keys(st):
        raise _err(s, where, f'domain {st.domain} names no card keys',
                   f'add card: {{keys: [config.key]}} to config/domains/{st.domain}.yaml')
    ref = str(ref)
    cfg = select.domain_config(st.domain)
    folder = (cfg.get('card') or {}).get('dir')
    candidates = [st.base / ref, paths.ROOT / ref]
    if folder:
        candidates += [paths.ROOT / folder / ref, paths.ROOT / folder / f'{ref}.json']
    for c in candidates:
        if c.is_file():
            return c.resolve()
    raise _err(s, where, f'no card {ref!r}', f'a profile JSON path, or a name in {folder or "the card dir"}')


def _axes(s, st, value):
    if not value:
        return []
    if not isinstance(value, dict):
        raise _err(s, 'axes', 'must be a mapping of name -> axis', 'axes: {cruise: {param: KEY, levels: [1, 2]}}')
    axes = []
    for name, spec in value.items():
        where = f'axes.{name}'
        if not NAME.match(str(name)) or name in RESERVED:
            raise _err(s, where, 'an axis name is letters, digits, _ or -, and not a row column',
                       f'rename it (not {", ".join(sorted(RESERVED))})')
        spec = dict(spec or {})
        bad = sorted(set(spec) - {'param', 'levels', 'card'})
        if bad:
            raise _err(s, f'{where}.{bad[0]}', 'unknown key', 'an axis takes param, levels, card')
        param, levels, card = spec.get('param'), spec.get('levels'), bool(spec.get('card'))
        if not levels:
            raise _err(s, f'{where}.levels', 'is empty', 'levels: [a, b] or {name: value}')
        if isinstance(levels, list):
            if not param and not card:
                raise _err(s, where, 'list levels need a param', f'{where}: {{param: KEY, levels: [...]}}')
            named = {}
            for i, v in enumerate(levels):
                named[str(v) if isinstance(v, (str, int, float, bool)) else f'l{i}'] = v
            levels = named
        if not isinstance(levels, dict):
            raise _err(s, f'{where}.levels', 'must be a list or a mapping')
        out = {}
        for lname, v in levels.items():
            lw = f'{where}.levels.{lname}'
            if card:
                _card_path(s, st, lw, v)
                out[str(lname)] = {'@card': str(v)}
            elif param:
                out[str(lname)] = {str(param): v}
            elif isinstance(v, dict) and v:
                out[str(lname)] = _flat(v)          # a named source: a set of values
            else:
                raise _err(s, lw, 'a level without param is a mapping of key -> value',
                           f'{where}: {{param: KEY, levels: {{{lname}: VALUE}}}}')
        if len(out) != len(levels):
            raise _err(s, f'{where}.levels', 'two levels share a name')
        axes.append(Axis(str(name), out, card))
    return axes


def _typed(text):
    for kind in (int, float):
        try:
            return kind(text)
        except ValueError:
            pass
    return text


def _conditions(s, st, value, splits):
    if value is None:
        if splits:
            raise _err(s, 'splits', 'needs a conditions table', 'conditions: {csv: FILE, id: COLUMN}')
        return
    spec = {'csv': value} if isinstance(value, str) else dict(value)
    bad = sorted(set(spec) - {'csv', 'id', 'set', 'where', 'limit'})
    if bad:
        raise _err(s, f'conditions.{bad[0]}', 'unknown key', 'conditions: {csv, id, set, where, limit}')
    if not spec.get('csv'):
        raise _err(s, 'conditions.csv', 'is missing', 'conditions: {csv: conditions.csv}')
    path = (st.base / str(spec['csv'])).resolve()
    try:
        with open(path, newline='') as f:
            reader = csv.DictReader(f)
            columns = list(reader.fieldnames or [])
            rows = [{k: _typed(v) for k, v in r.items()} for r in reader]
    except OSError as error:
        raise _err(s, 'conditions.csv', f'{path}: {error.strerror or error}') from None
    if not rows:
        raise _err(s, 'conditions.csv', f'{path.name} has no rows')
    id_col = spec.get('id') or columns[0]
    sets = spec.get('set') or {}
    for col in [id_col, *sets, *(spec.get('where') or {})]:
        if col not in columns:
            raise _err(s, 'conditions', f'no column {col!r} in {path.name}', f'columns: {", ".join(columns)}')
    where = spec.get('where') or {}
    rows = [r for r in rows if all(r[k] in (v if isinstance(v, list) else [v]) for k, v in where.items())]
    if spec.get('limit'):
        rows = rows[:int(spec['limit'])]
    seen = set()
    split_col, roles = None, {}
    if splits:
        sp = dict(splits)
        bad = sorted(set(sp) - {'column', 'roles'})
        if bad or not sp.get('column') or not sp.get('roles'):
            raise _err(s, 'splits', 'wants column and roles', 'splits: {column: split, roles: {calib: calib, '
                       'test: held-out}}')
        split_col, roles = str(sp['column']), {str(k): str(v) for k, v in sp['roles'].items()}
        if split_col not in columns:
            raise _err(s, 'splits.column', f'no column {split_col!r} in {path.name}',
                       f'columns: {", ".join(columns)}')
        for k, r in roles.items():
            if r not in ROLES:
                raise _err(s, f'splits.roles.{k}', f'role {r!r}', f'one of {", ".join(ROLES)}')
    out = []
    for r in rows:
        cid = str(r[id_col])
        if cid in seen:
            raise _err(s, 'conditions', f'id {cid!r} appears twice in column {id_col}', 'make the id column unique')
        seen.add(cid)
        split = None
        if split_col:
            split = str(r[split_col])
            if split not in roles:
                raise _err(s, 'splits.roles', f'condition {cid} has split {split!r} with no role',
                           f'add {split}: ROLE under splits.roles')
        out.append({'id': cid, 'row': r, 'set': {str(param): r[col] for col, param in sets.items()}, 'split': split})
    st.conditions, st.condition_columns, st.splits = out, columns, roles


def _blocks(s, st, value):
    if not value:
        raise _err(s, 'seeds', 'no seed blocks', "seeds: {dev: {role: dev, seeds: '0:8'}}")
    blocks = {}
    for name, spec in dict(value).items():
        where = f'seeds.{name}'
        if not NAME.match(str(name)):
            raise _err(s, where, 'a block name is letters, digits, _ or -')
        if not isinstance(spec, dict):
            spec = {'seeds': spec, 'role': name if name in ROLES else None}
        bad = sorted(set(spec) - {'role', 'seeds'})
        if bad:
            raise _err(s, f'{where}.{bad[0]}', 'unknown key', f"{where}: {{role: dev, seeds: '0:8'}}")
        role = spec.get('role')
        if role not in ROLES:
            raise _err(s, f'{where}.role', f'{role!r}', f'one of {", ".join(ROLES)}')
        text = spec.get('seeds')
        text = ','.join(str(x) for x in text) if isinstance(text, list) else text
        if text is None:
            raise _err(s, f'{where}.seeds', 'is missing', "seeds: '0:8' (N, A:B, a,b,c, @FILE, block:NAME)")
        try:
            seeds = select.parse_seeds(str(text), st.domain, base=st.base)
        except select.SelectError as error:
            raise _err(s, f'{where}.seeds', str(error)) from None
        blocks[str(name)] = Block(str(name), role, seeds)
    held = {x: b.name for b in blocks.values() if b.role == 'held-out' for x in b.seeds}
    for b in blocks.values():
        if b.role != 'held-out':
            both = [x for x in b.seeds if x in held]
            if both:
                raise _err(s, f'seeds.{b.name}', f'seed {both[0]} is also in held-out block {held[both[0]]}',
                           'held-out seeds never enter another block')
    return blocks


STAGE_KEYS = {'tier', 'tiers', 'blocks', 'splits', 'arms', 'only', 'set', 'after', 'select', 'record', 'pack'}


def _stages(s, st, value):
    if not value:
        return {DEFAULT_STAGE: Stage(DEFAULT_STAGE)}
    stages = {}
    for name, spec in dict(value).items():
        where = f'stages.{name}'
        if not NAME.match(str(name)):
            raise _err(s, where, 'a stage name is letters, digits, _ or -')
        spec = dict(spec or {})
        bad = sorted(set(spec) - STAGE_KEYS)
        if bad:
            raise _err(s, f'{where}.{bad[0]}', 'unknown key', f'a stage takes {", ".join(sorted(STAGE_KEYS))}')
        stage = Stage(str(name))
        if 'tier' in spec or 'tiers' in spec:
            stage.tiers = _tiers(s, f'{where}.tiers', spec.get('tiers', spec.get('tier')))
        for key, table in (('blocks', st.blocks), ('splits', st.splits)):
            if spec.get(key) is not None:
                names = [str(x) for x in (spec[key] if isinstance(spec[key], list) else [spec[key]])]
                for x in names:
                    if x not in table:
                        raise _err(s, f'{where}.{key}', f'no {key[:-1]} {x!r}', f'one of {", ".join(table) or "none"}')
                setattr(stage, key, names)
        if spec.get('arms') is not None:
            names = [str(x) for x in (spec['arms'] if isinstance(spec['arms'], list) else [spec['arms']])]
            expanded = []
            for x in names:
                found = [a for a in st.arms.values() if a.name == x or a.source == x]
                if not found:
                    raise _err(s, f'{where}.arms', f'no arm {x!r}', f'one of {", ".join(st.arms)}')
                expanded += [a.name for a in found if a.name not in expanded]
            stage.arms = expanded
        if spec.get('only') is not None:
            only = spec['only']
            keys = list(only[0]) if isinstance(only, list) and only and isinstance(only[0], dict) else None
            known = {'arm', *(a.name for a in st.axes)}
            if not keys or any(not isinstance(d, dict) or list(d) != keys for d in only) or set(keys) - known:
                raise _err(s, f'{where}.only', 'wants a list of {arm: A, AXIS: LEVEL} with the same keys',
                           f'keys from {", ".join(sorted(known))}')
            stage.only = [{k: str(v) for k, v in d.items()} for d in only]
        if not isinstance(spec.get('set') or {}, dict):
            raise _err(s, f'{where}.set', 'must be a mapping', f'{where}: {{set: {{dt: 0.01}}}}')
        stage.set = _flat(spec.get('set'))
        after = spec.get('after') or []
        stage.after = [str(x) for x in (after if isinstance(after, list) else [after])]
        if spec.get('select') is not None:
            sel = dict(spec['select'])
            bad = sorted(set(sel) - {'top_k', 'metric', 'order', 'by', 'from'})
            if bad:
                raise _err(s, f'{where}.select.{bad[0]}', 'unknown key', 'select: {top_k, metric, order, by, from}')
            k = sel.get('top_k')
            if not isinstance(k, int) or k < 1:
                raise _err(s, f'{where}.select.top_k', f'{k!r}', 'select: {top_k: 3, metric: NAME}')
            if not sel.get('metric'):
                raise _err(s, f'{where}.select.metric', 'is missing', 'select: {top_k: 3, metric: NAME}')
            if sel.get('order', 'max') not in ('max', 'min'):
                raise _err(s, f'{where}.select.order', f'{sel["order"]!r}', 'max or min')
            by = sel.get('by', 'arm')
            by = [str(x) for x in (by if isinstance(by, list) else [by])]
            axis_names = [a.name for a in st.axes]
            for x in by:
                if x != 'arm' and x not in axis_names:
                    raise _err(s, f'{where}.select.by', f'{x!r} is not arm or an axis',
                               f'arm or {", ".join(axis_names)}')
            src = sel.get('from') or (stage.after[0] if stage.after else None)
            if not src:
                raise _err(s, f'{where}.select', 'selects from no stage', f'{where}: {{after: STAGE}}')
            if src not in stage.after:
                stage.after.append(str(src))
            stage.select = {'top_k': k, 'metric': str(sel['metric']), 'order': sel.get('order', 'max'), 'by': by,
                            'from': str(src)}
        if spec.get('record') is not None:
            stage.record = _record(s, f'{where}.record', spec['record'])
        if spec.get('pack') is not None:
            stage.pack = _pack(s, f'{where}.pack', spec['pack'])
        stages[stage.name] = stage
    for stage in stages.values():
        for dep in stage.after:
            if dep not in stages:
                raise _err(s, f'stages.{stage.name}.after', f'no stage {dep!r}', f'one of {", ".join(stages)}')
    state = {}

    def visit(name, trail):
        if state.get(name) == 'open':
            raise _err(s, f'stages.{name}.after', f'a cycle: {" -> ".join(trail + [name])}')
        if state.get(name) != 'done':
            state[name] = 'open'
            for dep in stages[name].after:
                visit(dep, trail + [name])
            state[name] = 'done'
    for name in stages:
        visit(name, [])
    return stages
