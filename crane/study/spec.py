"""crane.study/1: the YAML study spec, read and checked. Every error is one line naming the key and the fix.

    study = load('examples/studies/toy_paired.yaml')
    study.arms['wide'].set          # {'adapter.options.reach_m': 2.5}
    study.blocks['test'].role       # 'held-out'

The spec's keys (Docs/Study.md has the full table):
    name, domain, tier | tiers, engine, backend, dtype, record, crn, targets, baseline, metrics, scorer, scorers,
    timeout_s, keep_runs, plugins, card, card_axes, arms, axes, conditions, splits, seeds, stages, pack, report,
    campaign, purpose, expect, pin, kind
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
ENGINES = ('scalar', 'batched')
BACKENDS = ('numpy', 'numba', 'jax')
DTYPES = ('float64', 'float32')
BATCHED_RECORDS = ('metrics', 'events', 'traj_decimated')    # what the batched engine serves (no full-rate obs)
NAME = re.compile(r'^[A-Za-z0-9][A-Za-z0-9_-]{0,47}$')
TOP = {'schema', 'name', 'domain', 'tier', 'tiers', 'record', 'crn', 'targets', 'baseline', 'metrics', 'scorer',
       'timeout_s', 'keep_runs', 'arms', 'axes', 'conditions', 'splits', 'seeds', 'stages', 'pack', 'campaign',
       'purpose', 'expect', 'pin', 'kind', 'report', 'description', 'engine', 'backend', 'dtype', 'plugins', 'card',
       'card_axes', 'scorers'}
DEFAULT_STAGE = 'main'
RESERVED = {'study', 'stage', 'arm', 'cell', 'pair', 'block', 'split', 'role', 'tier', 'seed', 'run_seed', 'condition',
            'outcome', 'error', 'wall_s', 'commit', 'card_sha', 'touched_held_out', 'succeeded', 'censored',
            'time_cap_s', 'engine'}   # row columns


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
    mission: str = None            # 'module:Class': a DomainAdapter (Python mission code) replacing the domain's
    engine: dict = field(default_factory=dict)    # {engine, backend, dtype}: what this arm overrides (per tier too)


@dataclass
class Axis:
    name: str
    levels: dict                   # level name -> {dotted key: value}
    card: bool = False             # level values are card refs


@dataclass
class CardAxis:
    """An axis whose levels edit fields of the cell's plant card (``card_axes``)."""
    name: str
    fields: list                   # field paths in the card JSON: dotted, '*' for every list item or key
    levels: dict                   # level name -> {kind: base|file|value|values|scale, ...}


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
    engine: dict = field(default_factory=dict)   # {engine, backend, dtype}: str, or {tier: str} each
    plugins: list = field(default_factory=list)  # Paths of the spec's plugin dirs (on sys.path for its runs only)
    card: str = None               # the base card the card axes edit (an arm's card overrides it)
    card_axes: list = field(default_factory=list)
    scorers: list = None           # population scorers: [{fn, over: [keys], args, name}]
    split_groups: dict = field(default_factory=dict)   # column value -> split name (splits.groups)

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

    def estimate(self, cell):
        """Seconds per (cell, seed): the stage's pack.unit_s, else the study's (None without one), for the cell's
        engine (pack.unit_s may be {scalar: S, batched: S})."""
        from crane.study import plan
        return plan.unit_s(self, cell.stage, cell.engine.get('mode', 'scalar'))

    def card_count(self):
        """How many derived cards the card axes cross to (1 without card axes), without deriving any."""
        n = 1
        for a in self.card_axes:
            n *= len(a.levels)
        return n

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
    st.engine = _engine(s, 'engine', raw)
    st.plugins = _plugins(s, st, raw.get('plugins'))
    if raw.get('targets') is not None:
        st.targets = (st.base / str(raw['targets'])).resolve()
    st.metrics = _names(s, 'metrics', raw.get('metrics'))
    st.scorer = _scorer(s, raw.get('scorer'))
    st.pack = _pack(s, 'pack', raw.get('pack'))
    st.report = dict(raw.get('report') or {})
    st.cluster = _cluster(s, raw)
    st.arms = _arms(s, st, raw.get('arms'))
    if raw.get('card') is not None:
        any_card(s, st, 'card', raw['card'])
        st.card = str(raw['card'])
    st.axes = _axes(s, st, raw.get('axes'))
    st.card_axes = _card_axes(s, st, raw.get('card_axes'))
    _conditions(s, st, raw.get('conditions'), raw.get('splits'))
    st.scorers = _scorers(s, st, raw.get('scorers'))
    st.blocks = _blocks(s, st, raw.get('seeds'))
    st.stages = _stages(s, st, raw.get('stages'))
    _check_engines(s, st)
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
    unit = pack.get('unit_s')
    if isinstance(unit, dict):
        bad = sorted(set(unit) - {'scalar', 'batched', 'task'})
        if bad:
            raise _err(s, f'{where}.unit_s.{bad[0]}', 'unknown key', 'unit_s: {scalar: S, batched: S, task: S} '
                       '(seconds per unit by engine)')
        unit = {k: _num(s, f'{where}.unit_s.{k}', v) for k, v in unit.items()} or None
    else:
        unit = _num(s, f'{where}.unit_s', unit)
    out = {'unit_s': unit, 'target_s': _num(s, f'{where}.target_s', pack.get('target_s'))}
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


ARM_KEYS = {'set', 'card', 'task', 'argv', 'args', 'timeout_s', 'sample', 'mission', 'engine', 'backend', 'dtype'}
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
    mission = spec.get('mission')
    sets = _flat(sets)
    if mission is not None:
        if kind != 'sim':
            raise _err(s, f'{where}.mission', 'is for sim arms (it replaces the domain adapter)',
                       'drop task/argv, or drop mission')
        sets['adapter.ref'] = _ref(s, f'{where}.mission', mission)
    engine = _engine(s, where, spec)
    if engine and kind != 'sim':
        raise _err(s, f'{where}.engine', 'is for sim arms', 'drop engine/backend/dtype from a task arm')
    return Arm(name=name, kind=kind, set=sets, card=None if card is None else str(card), task=task,
               argv=None if argv is None else [str(a) for a in argv], args=dict(args),
               timeout_s=_num(s, f'{where}.timeout_s', spec.get('timeout_s')),
               raw={k: v for k, v in spec.items()}, mission=mission, engine=engine)


def _engine(s, where, spec):
    """{engine, backend, dtype} of a spec or arm, each a str or {tier: str}; only the keys it gives."""
    out = {}
    for key, known in (('engine', ENGINES), ('backend', BACKENDS), ('dtype', DTYPES)):
        value = spec.get(key)
        if value is None:
            continue
        values = value if isinstance(value, dict) else {'*': value}
        for tier, v in values.items():
            if tier != '*':
                try:
                    tierlib.normalise(tier)
                except ValueError as error:
                    raise _err(s, f'{where}.{key}.{tier}', str(error)) from None
            if v not in known:
                raise _err(s, f'{where}.{key}' + (f'.{tier}' if tier != '*' else ''), f'{v!r}',
                           f'one of {", ".join(known)}')
        out[key] = {('*' if t == '*' else tierlib.normalise(t)): str(v) for t, v in values.items()}
    return out


def engine_of(st, arm, tier):
    """(engine, backend, dtype) the arm runs at a tier with: the arm's keys over the spec's, a tier's over '*'.
    Defaults: scalar; numba; float64 (float32 on jax)."""
    def pick(key, default):
        for src in (arm.engine if arm else {}, st.engine):
            table = src.get(key) or {}
            if tier in table:
                return table[tier]
            if '*' in table:
                return table['*']
        return default
    engine = pick('engine', 'scalar')
    backend = pick('backend', 'numba')
    return engine, backend, pick('dtype', 'float32' if backend == 'jax' else 'float64')


def _check_engines(s, st):
    for arm in st.arms.values():
        if arm.kind != 'sim':
            continue
        for tier in st.tiers + [t for g in st.stages.values() for t in (g.tiers or [])]:
            engine, backend, dtype = engine_of(st, arm, tier)
            if engine == 'batched' and backend == 'numpy' and dtype != 'float64':
                raise _err(s, f'arms.{arm.name}.dtype', 'the numpy backend is the float64 reference',
                           'dtype: float64, or backend: jax for float32')
            if engine != 'batched':
                continue
            for rec in [st.record] + [g.record for g in st.stages.values() if g.record]:
                if rec['level'] not in BATCHED_RECORDS:
                    raise _err(s, 'record', f'level {rec["level"]!r} is not served by the batched engine (it keeps '
                               'metrics, events and the decimated track)',
                               f'engine: {{{tier}: scalar}} for full-rate obs, or record: traj_decimated')


def _plugins(s, st, value):
    """The spec's plugin dirs (relative to the spec): their modules resolve 'module:attr' refs (mission, task,
    scorer, scorers) for this study's runs only."""
    if value is None:
        return []
    values = value if isinstance(value, list) else [value]
    out = []
    for i, v in enumerate(values):
        path = (st.base / str(v)).expanduser().resolve()
        if not path.is_dir():
            raise _err(s, f'plugins[{i}]', f'{v!r} is not a directory', 'plugins: [DIR] (relative to the spec)')
        out.append(path)
    return out


def card_doc(ref_path):
    import json
    return json.loads(Path(ref_path).read_text())


def card_matches(doc, path):
    """[(container, key)] of a field path in a card: dotted, '*' for every list item or dict key, an int for one
    list item. Empty when it names nothing."""
    nodes = [(None, None, doc)]
    for part in str(path).split('.'):
        nxt = []
        for _, _, node in nodes:
            if isinstance(node, list):
                keys = range(len(node)) if part == '*' else ([int(part)] if part.lstrip('-').isdigit()
                                                               and -len(node) <= int(part) < len(node) else [])
            elif isinstance(node, dict):
                keys = list(node) if part == '*' else ([part] if part in node else [])
            else:
                keys = []
            nxt += [(node, k, node[k]) for k in keys]
        nodes = nxt
    return [(c, k) for c, k, _ in nodes]


LEVEL_KINDS = ('base', 'file', 'value', 'values', 'scale')


def _card_level(s, st, where, value, transform):
    if value is None or value == 'base':
        return {'kind': 'base'}
    if isinstance(value, bool):
        raise _err(s, where, f'{value!r}', 'base, a card ref, a number, {values: {...}}, {scale: X}')
    if isinstance(value, (int, float)):
        return {'kind': transform, 'value': float(value)} if transform == 'scale' else {'kind': 'value',
                                                                                      'value': value}
    if isinstance(value, str):
        value = {'file': value}
    if not isinstance(value, dict) or len(value) != 1 or next(iter(value)) not in ('file', 'values', 'scale', 'value'):
        raise _err(s, where, f'{value!r}', 'base, a card ref (a named source), a number, {values: {PATH: V}}, '
                   '{scale: X}')
    (kind, v), = value.items()
    if kind == 'file':
        return {'kind': 'file', 'path': str(any_card(s, st, where, v)), 'ref': str(v)}
    if kind == 'values':
        if not isinstance(v, dict) or not v:
            raise _err(s, where, 'values wants a mapping of field path -> value', '{values: {mass: 31.0}}')
        return {'kind': 'values', 'values': {str(k): x for k, x in _flat(v).items()}}
    if isinstance(v, bool) or not isinstance(v, (int, float)):
        raise _err(s, where, f'{kind} wants a number', f'{{{kind}: 2.0}}')
    return {'kind': kind, 'value': float(v) if kind == 'scale' else v}


def _card_axes(s, st, value):
    """card_axes: {AXIS: {fields: [PATH], levels: {NAME: LEVEL}}}; a level is base (the card as is), a card ref
    (a named source: the fields' values from that card), a number (every field set to it), {values: {PATH: V}},
    or {scale: X} (every field multiplied). transform: scale makes plain-number levels multipliers."""
    if not value:
        return []
    if not isinstance(value, dict):
        raise _err(s, 'card_axes', 'must be a mapping of name -> {fields, levels}',
                   'card_axes: {thrust: {fields: [thrusters.*.kForward], levels: {pre: base, measured: m.json}}}')
    sim = [a for a in st.arms.values() if a.kind == 'sim']
    bases = {a.card or st.card for a in sim} if sim else {st.card}
    if None in bases:
        raise _err(s, 'card_axes', 'needs a base card', 'card: REF at the top (or a card on every sim arm)')
    docs = {b: card_doc(any_card(s, st, 'card', b)) for b in bases}
    taken = {a.name for a in st.axes}
    out = []
    for name, spec in value.items():
        where = f'card_axes.{name}'
        if not NAME.match(str(name)) or name in RESERVED or name in taken:
            raise _err(s, where, 'a card axis name is letters, digits, _ or -, not a row column or another axis')
        spec = dict(spec or {})
        bad = sorted(set(spec) - {'fields', 'levels', 'transform'})
        if bad:
            raise _err(s, f'{where}.{bad[0]}', 'unknown key', 'a card axis takes fields, levels, transform')
        fields = spec.get('fields')
        fields = [fields] if isinstance(fields, str) else fields
        if not fields or not all(isinstance(f, str) and f for f in fields):
            raise _err(s, f'{where}.fields', 'wants field paths', 'fields: [thrusters.*.kForward, izz]')
        transform = spec.get('transform', 'value')
        if transform not in ('value', 'scale'):
            raise _err(s, f'{where}.transform', f'{transform!r}', 'value (numbers set the fields) or scale')
        levels = spec.get('levels')
        if isinstance(levels, list):
            levels = {(f'x{v:g}' if transform == 'scale' and isinstance(v, (int, float)) else str(v)): v
                      for v in levels}
        if not isinstance(levels, dict) or not levels:
            raise _err(s, f'{where}.levels', 'is empty', 'levels: {pre: base, measured: CARD}')
        parsed = {str(k): _card_level(s, st, f'{where}.levels.{k}', v, transform) for k, v in levels.items()}
        for b, doc in docs.items():
            for f in fields:
                if not card_matches(doc, f):
                    raise _err(s, f'{where}.fields', f'card {b} has no field {f!r}',
                               'a dotted path into the card JSON (* for every thruster)')
            for lname, lv in parsed.items():
                paths = list(lv['values']) if lv['kind'] == 'values' else fields
                for f in paths:
                    if not card_matches(doc, f):
                        raise _err(s, f'{where}.levels.{lname}', f'card {b} has no field {f!r}')
                if lv['kind'] == 'file':
                    src = card_doc(lv['path'])
                    for f in fields:
                        if len(card_matches(src, f)) != len(card_matches(doc, f)):
                            raise _err(s, f'{where}.levels.{lname}', f'{lv["ref"]} and {b} differ at {f!r}',
                                       'a named source card with the same fields as the base')
        out.append(CardAxis(str(name), [str(f) for f in fields], parsed))
    return out


POPULATION_KEYS = ('card', 'block', 'split', 'role', 'arm', 'tier', 'stage')


def _scorers(s, st, value):
    """Population scorers: [{fn: module:function, over: 'card x block', args, name}]. Each gets the rows (condition
    columns + sim columns) of one group and returns named parts."""
    if value is None:
        return None
    if not isinstance(value, list) or not value:
        raise _err(s, 'scorers', 'wants a list', 'scorers: [{fn: module:function, over: card x block}]')
    known = set(POPULATION_KEYS) | {a.name for a in st.axes} | {f'cond.{c}' for c in st.condition_columns}
    out, names = [], set()
    for i, sc in enumerate(value):
        where = f'scorers[{i}]'
        sc = {'fn': sc} if isinstance(sc, str) else dict(sc or {})
        bad = sorted(set(sc) - {'fn', 'over', 'args', 'name'})
        if bad:
            raise _err(s, f'{where}.{bad[0]}', 'unknown key', 'a population scorer takes fn, over, args, name')
        fn = _ref(s, f'{where}.fn', sc.get('fn'))
        over = sc.get('over', 'card x block')
        keys = [k.strip() for k in (over if isinstance(over, list) else str(over).split(' x '))]
        for k in keys:
            if k not in known:
                raise _err(s, f'{where}.over', f'{k!r}', f'keys joined by " x " from {", ".join(sorted(known))}')
        name = str(sc.get('name') or fn.rpartition(':')[2])
        if name in names:
            raise _err(s, f'{where}.name', f'{name!r} is taken', 'name: OTHER')
        names.add(name)
        out.append({'fn': fn, 'over': keys, 'args': dict(sc.get('args') or {}), 'name': name})
    return out


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


def any_card(s, st, where, ref):
    """A card file: the domain's card lookup when it names card keys, else a path relative to the spec."""
    if st.domain and card_keys(st):
        return _card_path(s, st, where, ref)
    path = (st.base / str(ref)).expanduser()
    if path.is_file():
        return path.resolve()
    raise _err(s, where, f'no card file {ref!r}', 'a JSON path relative to the spec')


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
    split_col, roles, groups, rest = None, {}, {}, None
    if splits:
        sp = dict(splits)
        bad = sorted(set(sp) - {'column', 'roles', 'groups'})
        if bad or not sp.get('column') or not sp.get('roles'):
            raise _err(s, 'splits', 'wants column and roles (and optionally groups)',
                       'splits: {column: split, roles: {calib: calib, test: held-out}}')
        split_col, roles = str(sp['column']), {str(k): str(v) for k, v in sp['roles'].items()}
        if split_col not in columns:
            raise _err(s, 'splits.column', f'no column {split_col!r} in {path.name}',
                       f'columns: {", ".join(columns)}')
        for k, r in roles.items():
            if r not in ROLES:
                raise _err(s, f'splits.roles.{k}', f'role {r!r}', f'one of {", ".join(ROLES)}')
        for name, values in dict(sp.get('groups') or {}).items():
            where = f'splits.groups.{name}'
            if str(name) not in roles:
                raise _err(s, where, 'names no split of splits.roles', f'add {name}: ROLE under splits.roles')
            if values == '*':
                if rest is not None:
                    raise _err(s, where, f"'*' is already split {rest}", "one split takes '*' (every other value)")
                rest = str(name)
                continue
            if isinstance(values, str) and values.startswith('@'):
                try:
                    values = [ln.strip() for ln in (st.base / values[1:]).read_text().splitlines()
                              if ln.strip() and not ln.startswith('#')]
                except OSError as error:
                    raise _err(s, where, f'{values}: {error.strerror or error}') from None
            values = values if isinstance(values, list) else [values]
            for v in values:
                v = str(v)
                if v in groups and groups[v] != str(name):
                    raise _err(s, where, f'{split_col} {v!r} is also in split {groups[v]}',
                               'a column value sits in one split only')
                groups[v] = str(name)
    out = []
    for r in rows:
        cid = str(r[id_col])
        if cid in seen:
            raise _err(s, 'conditions', f'id {cid!r} appears twice in column {id_col}', 'make the id column unique')
        seen.add(cid)
        split = None
        if split_col:
            value = str(r[split_col])
            split = groups.get(value, rest) if (groups or rest) else value
            if split is None:
                raise _err(s, 'splits.groups', f'condition {cid} has {split_col} {value!r} in no group',
                           f"add it to a group, or give one group '*' (every other value)")
            if split not in roles:
                raise _err(s, 'splits.roles', f'condition {cid} has split {split!r} with no role',
                           f'add {split}: ROLE under splits.roles')
        out.append({'id': cid, 'row': r, 'set': {str(param): r[col] for col, param in sets.items()}, 'split': split})
    st.conditions, st.condition_columns, st.splits = out, columns, roles
    st.split_groups = dict(groups)


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
