"""A study's cells, units and jobs, and the adaptive selection between stages.

A cell is one (stage, arm, axis levels, condition, tier, seed block); its units are its seeds. A unit's cache key is
(study, stage, arm, cell hash, seed, card hash, code commit). The cell hash covers everything that changes a result
(the arm's definition, the levels' values, the condition row, the stage's set, tier, record level, scorer), so an
edited arm reruns and an unchanged one resumes. ``pair`` is the same hash without the arm: the cells whose rows pair
on the seed (common random numbers).

    for cell in study.cells(stage='R'):
        cell.id, cell.seeds, cell.run_seeds[cell.seeds[0]]
    jobs = study.jobs()            # cells x seed chunks of ~pack.target_s, for the cluster compiler
"""

from dataclasses import dataclass, field
import functools
import itertools
import math
from pathlib import Path

from crane import ids
from crane.study.spec import StudyError, _card_path, card_keys

OUT_ROOT = Path.home() / '.cache' / 'crane' / 'studies'
BOX_OUT = '/workspace/out'
TARGET_S = 900.0          # a job of ~15 min (compute: 5-30 min on interruptible boxes)
CHUNK = {'sim': 8, 'task': 1, 'batched': 256}   # units per local work item (a batched one is one engine call)
DERIVED = '@derived:'     # a card key's value until the executor writes the derived card (OUT/cards/SHA.json)


@dataclass
class Cell:
    study: str
    stage: str
    arm: str
    axes: dict                     # axis name -> level name
    condition: dict                # the condition row, or None
    tier: str
    seeds: list
    block: str
    key: dict                      # {study, stage, arm, cell, card}: the unit key without seed and commit
    split: str = None
    role: str = 'dev'
    id: str = ''
    pair: str = ''
    kind: str = 'sim'
    params: dict = field(default_factory=dict)    # dotted key -> value: overrides (sim) or args (task)
    card: str = None
    run_seeds: dict = field(default_factory=dict)  # seed -> the seed the episode runs with
    record: dict = None
    ctx: dict = field(default_factory=dict)       # what the executor needs (spec path, task ref, scorer, ...)
    engine: dict = field(default_factory=lambda: {'mode': 'scalar'})   # {mode, backend, dtype, why}
    card_doc: dict = None          # the derived card (card_axes) until the executor writes it

    @property
    def held_out(self):
        return self.role == 'held-out'

    @property
    def device(self):
        """gpu for a cell on the batched engine's jax backend (the cluster compiler asks for a GPU), else cpu."""
        return 'gpu' if self.engine.get('backend') == 'jax' else 'cpu'

    def unit(self, seed, commit):
        return {**self.key, 'seed': int(seed), 'commit': commit}

    def run(self, out_dir, progress=None, workers=1):
        """Run this cell's seeds that are not cached in OUT_DIR/cache; returns the rows of every seed done."""
        from crane.study import exec as ex
        return ex.run_cell(self, out_dir, progress, workers=workers)

    def info(self):
        """The JSON a task or scorer is handed about its cell."""
        return {'study': self.study, 'stage': self.stage, 'arm': self.arm, 'axes': dict(self.axes),
                'condition': self.condition, 'tier': self.tier, 'block': self.block, 'split': self.split,
                'role': self.role, 'id': self.id, 'card': self.card, 'card_sha': self.key.get('card'),
                'engine': dict(self.engine)}


class StageNotReady(StudyError):
    """A stage whose selection needs another stage's results first."""


@functools.lru_cache(maxsize=1)
def code_commit():
    """The crane checkout's commit; a dirty tree adds +<diff sha12> (the same edit shares it)."""
    from crane import paths
    state = ids.git_state(paths.ROOT)
    commit = state.get('commit') or 'unknown'
    return f"{commit}+{state['diff_sha'][:12]}" if state.get('dirty') and state.get('diff_sha') else commit


def default_out(study):
    return OUT_ROOT / study.name


def _nest(flat):
    out = {}
    for key, value in flat.items():
        *parents, leaf = key.split('.')
        node = out
        for part in parents:
            node = node.setdefault(part, {})
        node[leaf] = value
    return out


def label(block, split):
    """The name a cell's rows are counted and reported under: its seed block, plus its split when it has one."""
    return f'{block}/{split}' if split else block


def _role(block_role, split_role):
    if 'held-out' in (block_role, split_role):
        return 'held-out'
    return split_role or block_role


def _run_seed(study, seed, condition):
    if study.crn == 'derive':
        return ids.derive_seed(int(seed), study.name, (condition or {}).get('id', '')) & 0x7FFFFFFF
    return int(seed)


def _card_sha(path):
    return ids.sha256(Path(path).read_bytes()) if path else None


@functools.lru_cache(maxsize=64)
def _adapter_ref(domain, tier):
    from crane.run import tier_config
    from fastsim import config as fsconfig
    _, path, over = tier_config(domain, tier)
    cfg, _ = fsconfig.load(path, over)
    return str((cfg.get('adapter') or {}).get('ref', ''))


def engine(study, arm, tier, params):
    """{mode, backend, dtype, why} a sim cell runs on: the batched engine when the spec asks for it and the
    domain adapter has a batched controller; else the scalar fastsim path, ``why`` saying so (a plugin mission
    is Python mission code: scalar)."""
    from crane.study.spec import engine_of
    if arm.kind != 'sim':
        return {'mode': 'task'}
    mode, backend, dtype = engine_of(study, arm, tier)
    if mode != 'batched':
        return {'mode': 'scalar'}
    ref = str(params.get('adapter.ref') or _adapter_ref(study.domain, tier))
    if arm.mission:
        return {'mode': 'scalar', 'why': f'plugin mission {arm.mission} is Python mission code: scalar path'}
    from engine.episode import CONTROLLERS
    if ref.rpartition(':')[2] not in CONTROLLERS:
        return {'mode': 'scalar', 'why': f'no batched controller for {ref}: scalar path'}
    return {'mode': 'batched', 'backend': backend, 'dtype': dtype}


def plugin_state(study):
    """[{path, commit, dirty, diff_sha, sha256}] of the spec's plugin dirs: git state (recorded in the manifest)
    and the sha256 of their files (in every cell hash, so an edited plugin reruns)."""
    got = study.__dict__.get('_plugin_state')
    if got is None:
        import hashlib
        import os
        got = []
        for d in study.plugins:
            h = hashlib.sha256()
            for f in sorted(p for p in Path(d).rglob('*') if p.is_file() and '__pycache__' not in p.parts
                            and not p.name.startswith('.')):
                h.update(f.relative_to(d).as_posix().encode() + b'\0' + f.read_bytes() + b'\0')
            git = ids.git_state(d)
            dirty = bool(git.get('dirty'))
            if git.get('commit'):          # untracked files in the dir count too (its sha256 covers them)
                import subprocess
                try:
                    dirty = dirty or bool(subprocess.run(['git', '-C', str(d), 'status', '--porcelain', '--', '.'],
                                                         capture_output=True, text=True, timeout=30).stdout.strip())
                except (OSError, subprocess.TimeoutExpired):
                    pass
            rel = os.path.relpath(d, study.base)
            got.append({'path': Path(rel).as_posix() if not rel.startswith('..') else str(d),
                        'commit': git.get('commit'), 'dirty': dirty, 'diff_sha': git.get('diff_sha'),
                        'sha256': h.hexdigest()})
        study.__dict__['_plugin_state'] = got
    return got


def _stable_card_level(level):
    if level['kind'] == 'file':
        return {'kind': 'file', 'sha256': _card_sha(level['path'])}
    return level


def apply_card(doc, axis, level, sources):
    """Edit ``doc`` in place by one card axis level."""
    from crane.study.spec import card_doc, card_matches
    kind = level['kind']
    if kind == 'base':
        return
    if kind == 'values':
        for path, v in level['values'].items():
            for c, k in card_matches(doc, path):
                c[k] = v
        return
    for f in axis.fields:
        hits = card_matches(doc, f)
        if kind == 'file':
            if level['path'] not in sources:
                sources[level['path']] = card_doc(level['path'])
            src = card_matches(sources[level['path']], f)
            for (c, k), (sc, sk) in zip(hits, src):
                c[k] = sc[sk]
        elif kind == 'value':
            for c, k in hits:
                c[k] = level['value']
        else:                               # scale: a multiplicative transform
            for c, k in hits:
                if isinstance(c[k], bool) or not isinstance(c[k], (int, float)):
                    raise StudyError(f'card_axes.{axis.name}: field {f} is not a number; fix: scale numeric fields')
                c[k] = c[k] * level['value']


def derive_card(study, base_path, picks):
    """(doc, sha256, bytes) of the base card edited by ``picks`` [(CardAxis, level name)], made once per
    combination (the factorial is crossed lazily, cell by cell). The doc records its base and levels."""
    import copy
    import json
    memo = study.__dict__.setdefault('_derived', {})
    key = (str(base_path), tuple((a.name, n) for a, n in picks))
    if key not in memo:
        from crane.study.spec import card_doc
        bases = study.__dict__.setdefault('_card_bases', {})
        if str(base_path) not in bases:
            bases[str(base_path)] = card_doc(base_path)
        doc = copy.deepcopy(bases[str(base_path)])
        sources = study.__dict__.setdefault('_card_sources', {})
        for axis, name in picks:
            apply_card(doc, axis, axis.levels[name], sources)
        doc['crane_derived'] = {'base_sha256': _card_sha(base_path), 'levels': {a.name: n for a, n in picks}}
        data = (json.dumps(doc, indent=1, sort_keys=True) + '\n').encode()
        memo[key] = (doc, ids.sha256(data), data)
    return memo[key]


def stage_cells(study, stage, block=None, chosen=None):
    """The cells of one stage (``chosen``: the selected (arm, *levels) tuples of an adaptive stage)."""
    spec = study.stages[stage]
    arms = [study.arms[a] for a in (spec.arms or study.arms)]
    tiers = spec.tiers or study.tiers
    blocks = [study.blocks[b] for b in (spec.blocks or study.blocks)]
    conds = study.conditions or [None]
    if spec.splits:
        conds = [c for c in conds if c and c['split'] in spec.splits]
    record = spec.record or study.record
    levels = [[(a.name, n, v) for n, v in a.levels.items()] for a in study.axes]
    levels += [[(a.name, n, ('@card_axis', a, n)) for n in a.levels] for a in study.card_axes]
    plugins = [p['sha256'] for p in plugin_state(study)] if study.plugins else None
    by = (spec.select or {}).get('by', ['arm'])
    if chosen is None and spec.only:
        by = list(spec.only[0])
        chosen = {tuple(d[b] for b in by) for d in spec.only}
    out = []
    for blk, tier, cond, combo in itertools.product(blocks, tiers, conds, itertools.product(*levels)):
        split = cond['split'] if cond else None
        if block and block not in (blk.name, split):
            continue
        role = _role(blk.role, study.splits.get(split) if split else None)
        axes = {a: n for a, n, _ in combo}
        for arm in arms:
            if chosen is not None:
                pick = tuple(arm.name if b == 'arm' else axes[b] for b in by)
                if pick not in chosen:
                    continue
            params, card = dict(arm.set if arm.kind == 'sim' else {**_flat_args(arm.args), **arm.set}), arm.card
            picks = [v[1:] for _, _, v in combo if isinstance(v, tuple)]
            for _, _, values in combo:
                if isinstance(values, tuple):
                    continue
                for k, v in values.items():
                    if k == '@card':
                        card = v
                    else:
                        params[k] = v
            if cond:
                params.update(cond['set'])
            params.update(spec.set)
            card_path, card_doc, card_sha = None, None, None
            if picks:
                from crane.study.spec import any_card
                base = any_card(study.path.name, study, f'arms.{arm.name}.card', card or study.card)
                card_doc, card_sha, _ = derive_card(study, base, picks)
                for k in (card_keys(study) if arm.kind == 'sim' else ['card']):
                    params[k] = DERIVED + card_sha
            elif card is not None and arm.kind == 'sim':
                card_path = str(_card_path(study.path.name, study, f'arms.{arm.name}.card', card))
                for k in card_keys(study):
                    params[k] = card_path
            elif card is not None:
                params['card'] = card
            eng = engine(study, arm, tier, params)
            content = {'domain': study.domain, 'tier': tier, 'kind': arm.kind,
                       'axes': {a: (_stable_card_level(v[1].levels[v[2]]) if isinstance(v, tuple)
                                    else _stable_values(study, v)) for a, _, v in combo},
                       'condition': cond['row'] if cond else None, 'set': spec.set, 'record': record,
                       'scorer': _stable_scorer(study), 'crn': study.crn,
                       'metrics': study.metrics, 'block_role': role}
            if plugins:
                content['plugins'] = plugins
            pair = ids.digest(content)        # the card axes' levels are in it; the derived card's sha is in key
            stable = {k: v for k, v in params.items() if k not in card_keys(study) and k != 'card'}
            unit = {**content, 'arm': _stable_arm(study, arm), 'params': stable}
            if eng['mode'] == 'batched':      # in the unit key, not the pair: arms on either engine pair on the seed
                unit['engine'] = {k: eng[k] for k in ('mode', 'backend', 'dtype')}
            chash = ids.digest(unit)
            card_sha = card_sha or _card_sha(card_path)
            key = {'study': study.name, 'stage': stage, 'arm': arm.name, 'cell': chash, 'card': card_sha}
            cell = Cell(study=study.name, stage=stage, arm=arm.name, axes=axes,
                        condition=dict(cond['row']) if cond else None, tier=tier, seeds=list(blk.seeds),
                        block=blk.name, key=key, split=split, role=role, id=f'{stage}.{arm.name}.{chash[:10]}',
                        pair=pair[:16], kind=arm.kind, params=params, card=card_path,
                        run_seeds={s: _run_seed(study, s, cond) for s in blk.seeds}, record=record, engine=eng,
                        card_doc=card_doc)
            cell.ctx = {'spec': str(study.path), 'base': str(study.base), 'domain': study.domain,
                        'task': arm.task, 'argv': arm.argv, 'timeout_s': arm.timeout_s or study.timeout_s,
                        'scorer': study.scorer, 'metrics': study.metrics, 'keep_runs': study.keep_runs,
                        'condition_id': cond['id'] if cond else None,
                        'plugins': [str(p) for p in study.plugins]}
            out.append(cell)
    return out


def ref_id(ref, base):
    """A callable ref as hashed: a file ref by its file's sha256 (so moving the checkout keeps the hash), a module
    ref as written."""
    target, _, attr = str(ref).rpartition(':')
    if target.endswith('.py'):
        path = Path(target).expanduser()
        path = path if path.is_absolute() else Path(base) / path
        try:
            return f'sha256:{ids.sha256(path.read_bytes())}:{attr}'
        except OSError:
            return str(ref)
    return str(ref)


def _stable_scorer(study):
    return {'ref': ref_id(study.scorer['ref'], study.base), 'args': study.scorer['args']} if study.scorer else None


def _stable_values(study, values):
    """An axis level's values as hashed: a card ref by its file's sha256."""
    if '@card' in values:
        return {'@card': _card_sha(_card_path(study.path.name, study, 'axes', values['@card']))}
    return values


def _stable_arm(study, arm):
    raw = dict(arm.raw)
    if raw.get('task'):
        raw['task'] = ref_id(raw['task'], study.base)
    if raw.get('card') is not None and arm.kind == 'sim':
        raw['card'] = _card_sha(_card_path(study.path.name, study, f'arms.{arm.name}.card', raw['card']))
    return raw


def _flat_args(args):
    from crane.study.spec import _flat
    return _flat(args)


def cells(study, stage=None, block=None, out=None):
    """The cells of a stage (every stage when None). A stage that selects from another's results is resolved from
    OUT's cache; without ``out``, or before that stage is done, it is left out (see ``pending``)."""
    names = [stage] if stage else study.order()
    if stage and stage not in study.stages:
        raise StudyError(f'{study.path.name}: no stage {stage!r}; stages: {", ".join(study.stages)}')
    result = []
    for name in names:
        try:
            chosen = selection(study, name, out)['chosen'] if study.stages[name].select else None
        except StageNotReady:
            if stage:
                raise
            continue
        result += stage_cells(study, name, block, chosen)
    return result


def pending(study, out=None):
    """{stage: reason} for the stages whose cells wait on another stage's results."""
    reasons = {}
    for name in study.order():
        if study.stages[name].select:
            try:
                selection(study, name, out)
            except StageNotReady as error:
                reasons[name] = str(error).split(': ', 2)[-1]
    return reasons


def cached_rows(cells_, out, commit=None):
    """{cell id: {seed: record}} of the cells' units cached in OUT/cache (at the current commit)."""
    from crane.cache import UnitCache
    commit = commit or code_commit()
    cache = UnitCache(Path(out) / 'cache')
    got = {}
    for cell in cells_:
        for seed in cell.seeds:
            rec = cache.get(cell.unit(seed, commit))
            if rec is not None:
                got.setdefault(cell.id, {})[seed] = rec
    return got


def selection(study, stage, out):
    """{chosen: set of (arm, *levels), table: [(group, mean, n)], from} for an adaptive stage, from the rows of the
    stage it selects from (its non-held-out cells only). StageNotReady (one line) until those are all done."""
    spec = study.stages[stage]
    sel = spec.select
    src = sel['from']
    where = f'{study.path.name}: stages.{stage}'
    if out is None:
        raise StageNotReady(f'{where}: selects from stage {src} results: run it first; fix: crane study run '
                            f'{study.ref} --stage {src}')
    source = [c for c in cells(study, src, None, out) if not c.held_out]
    got = cached_rows(source, out)
    total = sum(len(c.seeds) for c in source)
    done = sum(len(v) for v in got.values())
    if done < total:
        raise StageNotReady(f'{where}: stage {src} has {done}/{total} units done; fix: crane study run '
                            f'{study.ref} --stage {src}')
    groups = {}
    for c in source:
        for rec in got.get(c.id, {}).values():
            v = (rec.get('row') or {}).get(sel['metric'])
            if isinstance(v, (int, float)) and not isinstance(v, bool) and math.isfinite(v):
                pick = tuple(c.arm if b == 'arm' else c.axes[b] for b in sel['by'])
                groups.setdefault(pick, []).append(float(v))
    if not groups:
        raise StageNotReady(f'{where}: stage {src} rows have no metric {sel["metric"]!r}; fix: name a row column '
                            f'in select.metric (crane study status {study.ref})')
    sign = -1.0 if sel['order'] == 'max' else 1.0
    table = sorted(((g, sum(v) / len(v), len(v)) for g, v in groups.items()), key=lambda t: (sign * t[1], t[0]))
    return {'chosen': {t[0] for t in table[:sel['top_k']]}, 'table': table, 'from': src, 'by': sel['by'],
            'metric': sel['metric'], 'order': sel['order'], 'top_k': sel['top_k']}


def unit_s(study, stage, mode='scalar'):
    """Seconds per unit of a stage: pack.unit_s, a number or {scalar: S, batched: S, task: S} by engine mode."""
    s = study.stages[stage]
    got = (s.pack or {}).get('unit_s') or study.pack.get('unit_s')
    if isinstance(got, dict):
        return got.get(mode) or got.get('scalar' if mode == 'task' else 'task')
    return got


def jobs(study, stage=None, block=None, out=None, box_out=BOX_OUT):
    """The cluster jobs (one cell x a seed chunk each) of the resolvable cells: what the rc-job/1 compiler reads.
    A job runs ``crane study run-cell SPEC --cell JOB_ID --out OUT`` and writes rows, manifest.json, metrics.json."""
    from crane import paths
    try:
        spec_ref = study.path.relative_to(paths.ROOT).as_posix()   # argv runs from the checkout's root
    except ValueError:
        spec_ref = str(study.path)
    from crane.study import guard
    result, pinned = [], None
    for cell in cells(study, stage, block, out):
        if cell.held_out and pinned is None:
            pinned = guard.targets_state(study)       # checked here, where git is; the box checks the sha256
        spec = study.stages[cell.stage]
        per_unit = unit_s(study, cell.stage, cell.engine.get('mode', 'scalar'))
        target = (spec.pack or {}).get('target_s') or study.pack.get('target_s') or TARGET_S
        size = max(1, int(target // per_unit)) if per_unit else len(cell.seeds)
        chunks = [cell.seeds[i:i + size] for i in range(0, len(cell.seeds), size)]
        for k, seeds in enumerate(chunks):
            job_id = cell.id if len(chunks) == 1 else f'{cell.id}.j{k}'
            where = f'{box_out}/{study.name}/{job_id}'
            argv = ['crane', 'study', 'run-cell', spec_ref, '--cell', job_id, '--out', where]
            targets, blocked = (pinned or (None, None)) if cell.held_out else (None, None)
            result.append({'job_id': job_id, 'cell': cell.id, 'stage': cell.stage, 'arm': cell.arm,
                           'block': cell.block, 'split': cell.split, 'role': cell.role, 'tier': cell.tier,
                           'held_out': cell.held_out, 'seeds': seeds, 'units': len(seeds),
                           'engine': cell.engine.get('mode'),
                           'est_seconds': round(len(seeds) * per_unit, 1) if per_unit else None,
                           'kind': study.cluster['kind'], 'campaign': study.cluster['campaign'],
                           'purpose': study.cluster['purpose'], 'expect': study.cluster['expect'],
                           'pin': study.cluster['pin'], 'out': where, 'spec': spec_ref,
                           'targets': {k: v for k, v in targets.items() if k != 'entries'} if targets else None,
                           'blocked': blocked, 'argv': argv})
    return result


def find_job(study, job_id, out=None):
    """(cell, seeds) of a job id (CELL_ID or CELL_ID.jK) or a cell id; StudyError names the fix."""
    import re
    m = re.match(r'^(.*)\.j(\d+)$', job_id)
    base, chunk = (m.group(1), m.group(2)) if m else (job_id, '')
    stage = job_id.split('.', 1)[0]
    if stage not in study.stages:
        form = 'is not of the form STAGE.ARM.HASH' if job_id.count('.') < 2 else f'names no stage of the spec ({stage})'
        raise StudyError(f'{study.path.name}: cell id {job_id!r} {form}; fix: crane study plan {study.ref} --json '
                         f'(its jobs[].job_id)')
    if study.stages[stage].select and out is None:
        raise StudyError(f'{study.path.name}: stage {stage} selects from stage results that a job does not have; '
                         f'fix: crane study promote {study.ref} --stage {stage}, and run the promoted spec')
    for cell in cells(study, stage, None, out):
        if cell.id == base:
            if not chunk:
                return cell, list(cell.seeds)
            for job in jobs(study, stage, None, out):
                if job['job_id'] == job_id:
                    return cell, job['seeds']
            break
    raise StudyError(f'{study.path.name}: no cell or job {job_id!r} (the spec or the code changed?); fix: crane '
                     f'study plan {study.ref} --json')


def nest(flat):
    return _nest(flat)
