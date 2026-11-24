"""A study to rc-job/1 jobs: one job per (cell, seed chunk), sized to 5-30 min from the study's per-unit estimate.

    jobs = compile(study)                                  # every cell of every stage
    jobs = compile(study, stage='A', block='dev', est_unit_s=12.0, tags=['cloud-ok', 'cpu'])
    jobs.warnings                                          # e.g. a cell whose whole work is under 5 min

A unit is one (cell, seed). Each job:
  - kind `crane_batch` (the crane-batch backend: one slot, every core) or `gpu_command` for a GPU cell;
  - seeds [0] (a batch job reports one run) and the cell's seed chunk in spec.seeds; est_seconds for the whole job;
  - repos: crane_ml (and the study spec's repo when it is another one) pinned at the exact commit, by local path
    (research submit bundles local paths);
  - spec.run (a shell line for py_command/gpu_command) and spec.argv (for crane-command), both
    `python3 -m crane_backends.batch --job JSON`, which runs `crane study run-cell SPEC --cell ID --seeds .. --out ..`;
  - job_id from the dedup key (the chunk's unit cache keys plus the commit): the same plan resubmits as `exists`.
Seeds already in `done` (a UnitCache or a predicate on unit keys) are left out. Field data never goes in: a cell or
spec that names a path under ~/field_corpus_* or a bags dir is refused, and so is a held-out block without confirm.
"""

import json
import math
import os
import re
import shlex
import subprocess
import time
from pathlib import Path

from crane import paths
from crane.cache import UnitCache
from crane.cluster import HELD_OUT, cell_dir, cell_id, role_of, slug, studies_root, study_name
from crane.ids import digest

KIND_CPU, KIND_GPU = 'crane_batch', 'gpu_command'
JOB_MIN_S, JOB_TARGET_S, JOB_MAX_S = 300.0, 900.0, 1800.0      # compute's contract: 5-30 min, aim for the middle
SEED_CAP_S = 240 * 60.0                                         # one unit may never take longer
OVERHEAD_S = 30.0                                               # process start, spec load, upload
PURPOSE_MIN = 20
CRANE_REPO = 'crane_ml'
FIELD = re.compile(r'(^|[\s/"\'=:])(~|/home/[^/\s]+)?/?field_corpus_[^\s"\']*|(^|/)[^/\s]*bags(/|$)|\.(mcap|db3|bag)\b')


class CompileError(ValueError):
    """The study cannot go to the cluster as it is; the message names the fix."""


class Jobs(list):
    """The rc-job/1 bodies, plus what the plan should say about them."""
    warnings: list


def unit_key(cell, seed):
    """The unit cache key of one (cell, seed): crane.cache.UnitCache.key({**cell.key, 'seed': seed})."""
    return UnitCache.key({**dict(cell.key), 'seed': int(seed)})


def _job_slug(text, n):
    return re.sub(r'[^A-Za-z0-9.-]+', '-', str(text)).strip('-.')[:n] or 'x'


# ---- field data -----------------------------------------------------------------------------------------------

def _strings(obj):
    if isinstance(obj, (str, os.PathLike)):
        yield str(obj)
    elif isinstance(obj, dict):
        for k, v in obj.items():
            yield str(k)
            yield from _strings(v)
    elif isinstance(obj, (list, tuple, set)):
        for v in obj:
            yield from _strings(v)


def field_paths(*objs):
    """The strings in objs that name field data (a ~/field_corpus_* path, a bags dir, an .mcap/.db3/.bag file)."""
    return [s for o in objs for s in _strings(o) if FIELD.search(s)]


# ---- repos and the spec file -------------------------------------------------------------------------------------

def _git(root, *args):
    r = subprocess.run(['git', '-C', str(root), *args], capture_output=True, text=True)
    if r.returncode:
        raise CompileError(f'git {" ".join(args)} in {root}: {r.stderr.strip() or r.returncode}')
    return r.stdout.strip()


def pin_repo(root, name, allow_dirty=False, only=None):
    """{name, url, ref}: the repo at its exact HEAD commit, by local path. Uncommitted tracked changes (in `only`
    when given) are refused: the job would run the commit, not the working tree."""
    top = Path(_git(root, 'rev-parse', '--show-toplevel'))
    sha = _git(top, 'rev-parse', 'HEAD')
    dirty = _git(top, 'status', '--porcelain', '--untracked-files=no', '--', *(only or ['.']))
    if dirty and not allow_dirty:
        first = dirty.splitlines()[0].split(maxsplit=1)[-1]
        raise CompileError(f'{name} has uncommitted changes ({first}); jobs run commit {sha[:12]}: commit them, '
                           'or pass --allow-dirty to run the commit as it is')
    tracked = _git(top, 'ls-tree', '-r', '--name-only', sha)
    bad = field_paths(tracked.splitlines())
    if bad:
        raise CompileError(f'{name}@{sha[:12]} tracks field data ({bad[0]}); a bundle would carry it: remove it from '
                           'the repo first')
    return {'name': name, 'url': str(top), 'ref': sha}


def spec_source(spec_path, crane_root, allow_dirty=False):
    """Where a job finds the study spec: ({'repo', 'path'}, extra repo pin or None), or ({'text'}, None) for a spec
    outside git (inlined; files it names relative to itself are then not available)."""
    spec_path = Path(spec_path).expanduser().resolve()
    try:
        top = Path(_git(spec_path.parent, 'rev-parse', '--show-toplevel')).resolve()
    except CompileError:
        text = spec_path.read_text()
        if len(text) > 48_000:
            raise CompileError(f'{spec_path} is outside git and too big to inline ({len(text)} bytes); commit it to '
                               'a repo') from None
        return {'text': text, 'name': spec_path.name}, None
    rel = spec_path.relative_to(top).as_posix()
    if not _git(top, 'ls-files', '--', rel):
        raise CompileError(f'{spec_path} is not committed; jobs read it from the commit: git add and commit it')
    if top == Path(crane_root).resolve():
        if _git(top, 'status', '--porcelain', '--', rel) and not allow_dirty:
            raise CompileError(f'{rel} has uncommitted changes; commit it (jobs read the committed spec)')
        return {'repo': CRANE_REPO, 'path': rel}, None
    name = slug(top.name, 40).replace('_', '-')
    return {'repo': name, 'path': rel}, pin_repo(top, name, allow_dirty)


# ---- estimates and chunks -----------------------------------------------------------------------------------------

def unit_estimate(study, cell, est_unit_s=None):
    """Seconds for one (cell, seed): the given estimate, else the cell's, study.estimate(cell), or the study's."""
    for value in (est_unit_s, getattr(cell, 'est_unit_s', None), getattr(cell, 'est_s', None)):
        if value:
            return float(value)
    fn = getattr(study, 'estimate', None)
    if callable(fn):
        value = fn(cell)
        if value:
            return float(value)
    value = getattr(study, 'est_unit_s', None)
    return float(value) if value else None


def bench_estimate(path):
    """Seconds per episode on one core from a bench.fast bench.json (episodes at W=1)."""
    doc = json.loads(Path(path).expanduser().read_text())
    rows = doc.get('episodes') or []
    rows = rows if isinstance(rows, list) else [rows]
    for row in rows:
        if int(row.get('workers', 1)) == 1 and row.get('episodes_per_s'):
            return 1.0 / float(row['episodes_per_s'])
    per_core = [float(r['per_core']) for r in rows if r.get('per_core')]
    if per_core:
        return 1.0 / max(per_core)
    raise CompileError(f'{path}: no episodes rate at 1 worker; rerun python3 -m bench.fast --workers 1')


def measure(spec_path, cell, seed, crane_root=None, timeout_s=1800):
    """A measured dry run: `crane study run-cell` for one seed into a temp dir, timed. Never for a T2 cell (that
    would start Unity)."""
    import tempfile
    if str(getattr(cell, 'tier', '')).upper() in ('T2', 'UNITY'):
        raise CompileError(f'{cell_id(cell)} is a T2 cell; measuring would start Unity: give --est-unit-s from '
                           'crane bench player instead')
    root = Path(crane_root or paths.ROOT)
    with tempfile.TemporaryDirectory(prefix='crane-measure-') as tmp:
        t0 = time.monotonic()
        r = subprocess.run([str(root / 'bin' / 'crane'), 'study', 'run-cell', str(spec_path), '--cell', cell_id(cell),
                            '--seeds', str(seed), '--out', tmp], capture_output=True, text=True, timeout=timeout_s,
                           env={**os.environ, 'CRANE_RUN_DIR': tmp})
        if r.returncode:
            tail = (r.stderr or r.stdout).strip().splitlines()[-1:] or [f'exit {r.returncode}']
            raise CompileError(f'measuring {cell_id(cell)} seed {seed} failed: {tail[0]}')
        return time.monotonic() - t0


def chunks(seeds, est_unit_s):
    """Seed chunks of about JOB_TARGET_S each, as even as possible (never more units than fit in JOB_MAX_S)."""
    seeds = list(seeds)
    if not seeds:
        return []
    per = max(1, int(JOB_TARGET_S // est_unit_s))
    per = min(per, max(1, int(JOB_MAX_S // est_unit_s)))
    n = math.ceil(len(seeds) / per)
    size = math.ceil(len(seeds) / n)
    return [seeds[i:i + size] for i in range(0, len(seeds), size)]


# ---- needs ----------------------------------------------------------------------------------------------------------

def _needs(study, cell, tags):
    own = {**(getattr(study, 'needs', None) or {}), **(getattr(cell, 'needs', None) or {})}
    gpu = bool(own.get('gpu') or str(getattr(cell, 'device', '')).lower() in ('gpu', 'cuda'))
    needs = {'tags': sorted({*tags, *(own.get('tags') or [])})}
    if gpu:
        needs['gpu'] = True
        needs['vram_gb'] = float(own.get('vram_gb') or own.get('vram') or 8)
    cpus = own.get('min_cpu') or own.get('cpus')
    if cpus:
        needs['min_cpu'] = int(cpus)
    for k in ('min_ram_gb', 'host', 'os', 'cuda'):
        if own.get(k):
            needs[k] = own[k]
    return needs, gpu


# ---- compile --------------------------------------------------------------------------------------------------------

def compile(study, stage=None, block=None, *, spec_path=None, est_unit_s=None, done=None, tags=(), project='crane',
            campaign=None, purpose=None, expect=None, submitter='crane-study', priority=100, confirm=False,
            allow_dirty=False, crane_root=None, out_root=None, kind=None, pins=None):
    """rc-job/1 bodies for the study's cells (one stage and/or block when given). Raises CompileError naming the fix.
    `pins` ({'crane_ml': {...}, ...}) skips the git lookups (tests, or a caller that pinned already)."""
    name = study_name(study)
    crane_root = Path(crane_root or paths.ROOT)
    cells = [c for c in study.cells() if (stage is None or c.stage == stage) and (block is None or c.block == block)]
    if not cells:
        raise CompileError(f'no cells in {name}' + (f' for stage {stage}' if stage else '')
                           + (f' block {block}' if block else '') + '; crane study plan SPEC lists them')
    purpose = ' '.join(str(purpose or getattr(study, 'purpose', '') or '').split())
    if len(purpose) < PURPOSE_MIN:
        raise CompileError('jobs need a purpose (1-2 sentences, public on the cluster feed): set purpose: in the spec '
                           'or pass --purpose')
    spec_path = spec_path or getattr(study, 'path', None)
    spec_text = Path(spec_path).expanduser().read_text() if spec_path and Path(spec_path).expanduser().is_file() else ''
    bad = field_paths(spec_text.splitlines())
    for c in cells:
        bad += field_paths(c.axes, c.condition, c.key, getattr(c, 'inputs', None))
    if bad:
        raise CompileError(f'{name} names field data ({bad[0]}); field data never goes to the cluster: run that cell '
                           'locally (crane study run), or point it at a derived table outside the corpus')
    held = sorted({cell_id(c) for c in cells if role_of(study, c) == HELD_OUT})
    if held and not confirm:
        raise CompileError(f'{len(held)} cell(s) are on a held-out block ({held[0]}); they run only with --confirm '
                           'and a committed targets file older than the run')
    if held:
        guard = getattr(study, 'check_held_out', None)
        if callable(guard):
            guard([c for c in cells if role_of(study, c) == HELD_OUT])   # raises with its own fix

    if pins is None:
        pins = {CRANE_REPO: pin_repo(crane_root, CRANE_REPO, allow_dirty,
                                     only=[l.split('#')[0].strip() for l in
                                           (crane_root / 'crane' / 'runtime.sparse').read_text().splitlines()
                                           if l.split('#')[0].strip()])}
        if spec_path:
            src, extra = spec_source(spec_path, crane_root, allow_dirty)
            if extra:
                pins[extra['name']] = extra
        else:
            raise CompileError('the study has no spec file path; pass spec_path (crane cluster submit SPEC)')
    else:
        pins = dict(pins)
        src = pins.pop('_spec', {'repo': CRANE_REPO, 'path': str(spec_path or 'study.yaml')})
    commit = pins[CRANE_REPO]['ref']
    repos = list(pins.values())
    campaign = slug(campaign or getattr(study, 'campaign', None) or name, 100)
    expect = expect or getattr(study, 'expect', None) or {
        'owner': submitter, 'metrics': ['rows', 'failed'],
        'stop': 'Stop when every cell has rows for all its seeds; a failed job is rerun from its cell, not in bulk.'}
    is_done = done.has if isinstance(done, UnitCache) else (done or (lambda key: False))
    study_root = studies_root(out_root) / name

    jobs, warnings, missing = Jobs(), [], []
    plans = []
    for c in cells:
        est = unit_estimate(study, c, est_unit_s)
        if est is None:
            missing.append(cell_id(c))
            continue
        if est > SEED_CAP_S:
            raise CompileError(f'{cell_id(c)}: one seed takes {est / 60:.0f} min, over the {SEED_CAP_S / 60:.0f} min '
                               'cap; split the cell (fewer episodes per seed) or checkpoint inside it')
        if est > JOB_MAX_S:
            warnings.append(f'{cell_id(c)}: one seed is {est / 60:.0f} min, over the 30 min job target; one seed per '
                            'job (interruptible boxes may lose it)')
        seeds = [s for s in c.seeds if not is_done(unit_key(c, s))]
        if not seeds:
            continue
        parts = chunks(seeds, est)
        if est * len(seeds) < JOB_MIN_S:
            warnings.append(f'{cell_id(c)}: the whole cell is {est * len(seeds) / 60:.1f} min, under the 5 min job '
                            'minimum; consider running it locally')
        plans.append((c, est, parts))
    if missing:
        raise CompileError(f'no time estimate for {len(missing)} cell(s) ({missing[0]}); pass --est-unit-s S, '
                           '--est-from bench.json, or --measure (one seed run locally)')

    for c, est, parts in plans:
        role = role_of(study, c)
        needs, gpu = _needs(study, c, tags)
        cid = cell_id(c)
        try:
            rel = cell_dir(out_root, study, c, name).relative_to(study_root).as_posix()
        except ValueError:
            rel = cell_dir(None, study, c, name).relative_to(studies_root(None) / name).as_posix()
        for i, seeds in enumerate(parts):
            keys = [unit_key(c, s) for s in seeds]
            dedup = digest({'units': keys, 'commit': commit, 'spec': src})
            job_id = '.'.join([_job_slug(name, 24), _job_slug(c.stage or 'main', 12), _job_slug(c.arm, 24),
                               f'c{i}', dedup[:10]])
            job = {'study': name, 'stage': c.stage, 'arm': c.arm, 'block': c.block, 'role': role, 'cell': cid,
                   'cell_dir': rel, 'tier': getattr(c, 'tier', None), 'seeds': list(seeds), 'spec_file': src,
                   'repos': {r['name']: r['name'] for r in repos}, 'est_unit_s': round(est, 3),
                   'stall_s': int(max(600.0, 3 * est + 120)), 'dedup': dedup}
            line = json.dumps(job, separators=(',', ':'), sort_keys=True)
            spec = {**job, 'schema': 'crane.cluster.job/1', 'repo': CRANE_REPO,
                    'run': f'PYTHONPATH=src/{CRANE_REPO} python3 -m crane_backends.batch --job {shlex.quote(line)}',
                    'argv': ['python3', '-m', 'crane_backends.batch', '--job', line]}
            jobs.append({
                'schema': 'rc-job/1', 'job_id': job_id, 'kind': kind or (KIND_GPU if gpu else KIND_CPU),
                'purpose': purpose, 'est_seconds': round(est * len(seeds) + OVERHEAD_S, 1), 'submitter': submitter,
                'campaign': campaign, 'priority': int(priority) + i, 'needs': needs, 'seeds': [0], 'repos': repos,
                'spec': spec, 'outputs': {'class': 'summary'}, 'expect': expect, 'max_attempts': 3})
    jobs.warnings = warnings
    return jobs


def summarize(jobs):
    """Rows for the plan table: stage, arm, block, role, jobs, units, est per job (min/max), total compute."""
    groups = {}
    for j in jobs:
        s = j['spec']
        g = groups.setdefault((s['stage'], s['arm'], s['block'], s['role'], j['kind']), [])
        g.append(j)
    out = []
    for (stage, arm, block, role, kind), js in sorted(groups.items(), key=lambda kv: tuple(map(str, kv[0]))):
        est = [j['est_seconds'] for j in js]
        out.append({'stage': stage, 'arm': arm, 'block': block, 'role': role, 'kind': kind, 'jobs': len(js),
                    'units': sum(len(j['spec']['seeds']) for j in js), 'est_min_s': min(est), 'est_max_s': max(est),
                    'compute_s': sum(est)})
    return out
