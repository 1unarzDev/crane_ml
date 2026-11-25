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
    `python3 -m crane_backends.batch --job JSON`, which runs `crane study run-cell SPEC --cell RUN_ID --out ..`
    (plus --confirm on a held-out block);
  - job_id from the dedup key (the chunk's unit cache keys plus the commit): the same plan resubmits as `exists`.
Chunks: a study with ``jobs(stage, block)`` (crane.study) decides them, and RUN_ID is its job id (CELL_ID or
CELL_ID.jK); any other study gets chunks here and RUN_ID is the cell id plus ``--seeds a,b,c``.
Seeds already in `done` (a UnitCache or a predicate on unit keys) are left out. Field data never goes in: a cell or
spec that names a path under ~/field_corpus_*, a bag dir (rosbag, *_bags/) or an .mcap/.db3/.bag file is refused.
Held-out cells are left out (``jobs.skipped``) unless ``confirm``; with it crane.study.guard checks here, where git is,
that the targets file is committed before now, and each held-out job carries the file's sha256, which crane-batch
passes to run-cell as ``--targets-sha256`` (the box has no git). A study without that guard fails closed.

This module is the one study -> rc-job/1 compiler: `crane study compile` and `crane cluster submit SPEC` both call
compile_spec().
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
KINDS = (KIND_CPU, KIND_GPU)
JOB_ID = re.compile(r'^[A-Za-z0-9][A-Za-z0-9._:-]{2,127}$')
LOCAL_PATH = re.compile(r'(^|\s)(/|~/)\S')
JOB_MIN_S, JOB_TARGET_S, JOB_MAX_S = 300.0, 900.0, 1800.0      # compute's contract: 5-30 min, aim for the middle
SEED_CAP_S = 240 * 60.0                                         # one unit may never take longer
OVERHEAD_S = 30.0                                               # process start, spec load, upload
PURPOSE_MIN, PURPOSE_MAX = 20, 600       # rc-job/1: 20-600 chars, published, no local path
CRANE_REPO = 'crane_ml'
FIELD_CORPUS = re.compile(r'(^|[\s/"\'=:~])field_corpus_[^\s"\'/]*')
FIELD_FILE = re.compile(r'\.(mcap|db3|bag)(?=$|[\s"\',;/])')
FIELD_BAGDIR = re.compile(r'(^|/)(ros)?bags?/|(^|/)[^/\s]*[_-](ros)?bags?(/|$)|(^|/)rosbags?(/|$)')


class CompileError(ValueError):
    """The study cannot go to the cluster as it is; the message names the fix."""


class Jobs(list):
    """The rc-job/1 bodies, plus what the plan should say about them: warnings, and the held-out cells left out
    (skipped: [cell id], with skip_reason the guard's line) because confirm was not given."""
    warnings: list
    skipped: list
    skip_reason: str = None


def unit_key(cell, seed, commit):
    """The unit cache key of one (cell, seed) at a code commit: UnitCache.key(cell.unit(seed, commit)) (crane.study),
    else of {**cell.key, seed, commit}. The study's cache keys units the same way, so done units line up."""
    unit = cell.unit(seed, commit) if callable(getattr(cell, 'unit', None)) else \
        {**dict(cell.key), 'seed': int(seed), 'commit': commit}
    return UnitCache.key(unit)


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


def is_field(s):
    """Whether a string names field data: a field_corpus_* path, an .mcap/.db3/.bag file, or a bag dir (bags/,
    rosbag/, lake_bags/ as a path component; a bare word is not a path)."""
    return bool(FIELD_CORPUS.search(s) or FIELD_FILE.search(s) or ('/' in s and FIELD_BAGDIR.search(s)))


def field_paths(*objs):
    """The strings in objs that name field data (see is_field). Derived tables committed elsewhere pass."""
    return [s for o in objs for s in _strings(o) if is_field(s)]


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
        first = rel.split('/', 1)[0]
        if '/' in rel and first not in sparse_dirs(crane_root):
            raise CompileError(f'{rel} is under {first}/, which rented boxes do not check out (crane/runtime.sparse); '
                               f'move the spec under config/ or add {first} to crane/runtime.sparse')
        return {'repo': CRANE_REPO, 'path': rel}, None
    name = slug(top.name, 40).replace('_', '-')
    return {'repo': name, 'path': rel}, pin_repo(top, name, allow_dirty)


def sparse_dirs(crane_root):
    """The top-level dirs a rented box checks out (crane/runtime.sparse)."""
    text = (Path(crane_root) / 'crane' / 'runtime.sparse').read_text()
    return [ln.split('#')[0].strip() for ln in text.splitlines() if ln.split('#')[0].strip()]


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
    pin = dict((getattr(study, 'cluster', None) or {}).get('pin') or {})
    hosts = pin.pop('hosts', None) or []
    if len(hosts) > 1:
        raise CompileError(f'pin.hosts names {len(hosts)} hosts; rc-job/1 needs only all-required tags: give those '
                           'hosts a shared worker tag and pin tags: [TAG] instead')
    study_needs, cell_needs = getattr(study, 'needs', None) or {}, getattr(cell, 'needs', None) or {}
    own = {**pin, **study_needs, **cell_needs}
    # a host is pinned by its worker tag (worker.yaml caps.tags: [HOST])
    own['tags'] = [*(pin.get('tags') or []), *hosts, *(study_needs.get('tags') or []), *(cell_needs.get('tags') or [])]
    gpu = bool(own.get('gpu') or str(getattr(cell, 'device', '')).lower() in ('gpu', 'cuda'))
    needs = {'os': own.get('os') or ['linux'], 'tags': sorted({*tags, *(own.get('tags') or [])})}
    if gpu:
        needs['gpu'] = True
        needs['vram_gb'] = float(own.get('vram_gb') or own.get('vram') or 8)
    cpus = own.get('min_cpu') or own.get('cpus')
    if cpus:
        needs['min_cpu'] = int(cpus)
    for k in ('min_ram_gb', 'host', 'cuda'):
        if own.get(k):
            needs[k] = own[k]
    return needs, gpu


# ---- compile --------------------------------------------------------------------------------------------------------

def _study_chunks(study, stage, block):
    """{cell id: [(run id, seeds, est_seconds or None)]} from a study that packs its own jobs (crane.study's
    Study.jobs), else None."""
    fn = getattr(study, 'jobs', None)
    if not callable(fn):
        return None
    try:
        jobs = fn(stage=stage, block=block)
    except ValueError as error:        # crane.study's StudyError (e.g. a stage that selects from unfetched results)
        raise CompileError(str(error)) from None
    out = {}
    for j in jobs:
        out.setdefault(j['cell'], []).append((j['job_id'], list(j['seeds']), j.get('est_seconds')))
    return out


def held_out_guard(study, cells, confirm):
    """The targets state recorded on every held-out job ({path, commit, committed_at, sha256}) when the held-out
    cells may go, else raises with the guard's line. The guard is crane.study.guard (the one implementation); a study
    it cannot read (no targets: in its spec) fails closed."""
    names = sorted({cell_id(c) for c in cells})
    from crane.study import guard
    if not hasattr(study, 'targets') or not hasattr(study, 'splits'):
        raise CompileError(f'held-out {names[0]}: nothing here can check its targets file (not a crane.study spec); '
                           'load the spec with crane.study (targets: PATH, committed before the run)')
    reason, state = guard.refuse(study, cells, confirm, fix=f'crane study compile {study.ref} --confirm'
                                 if hasattr(study, 'ref') else None)
    if reason:
        raise CompileError(reason)
    return state


def _expect(given, submitter):
    """rc-job/1 expect {owner, metrics, stop}: a dict from the spec as is; free text becomes the stop line."""
    default = {'owner': submitter, 'metrics': ['rows', 'failed'],
               'stop': 'Stop when every cell has rows for all its seeds; a failed job is rerun from its cell, not in bulk.'}
    if isinstance(given, dict):
        bad = sorted(set(given) - {'owner', 'metrics', 'artifacts', 'stop'})
        if bad:
            raise CompileError(f'expect.{bad[0]}: rc-job/1 expect has owner, metrics, artifacts and stop only')
        return {**default, **given}
    if given and len(str(given).strip()) >= 10:
        return {**default, 'stop': ' '.join(str(given).split())[:PURPOSE_MAX]}
    return default


def _purpose(text):
    purpose = ' '.join(str(text or '').split())
    if not PURPOSE_MIN <= len(purpose) <= PURPOSE_MAX or LOCAL_PATH.search(purpose):
        raise CompileError(f'jobs need a purpose of {PURPOSE_MIN}-{PURPOSE_MAX} chars with no local path (public on the '
                           'cluster feed): set purpose: in the spec (one sentence on what the study tests) or pass '
                           '--purpose')
    return purpose


def compile(study, stage=None, block=None, *, spec_path=None, est_unit_s=None, done=None, tags=(), project='crane',
            campaign=None, purpose=None, expect=None, submitter='crane-study', priority=100, confirm=False,
            allow_dirty=False, crane_root=None, out_root=None, kind=None, pins=None):
    """rc-job/1 bodies for the study's cells (one stage and/or block when given). Raises CompileError naming the fix.
    `pins` ({'crane_ml': {...}, ...}) skips the git lookups (tests, or a caller that pinned already)."""
    name = study_name(study)
    crane_root = Path(crane_root or paths.ROOT)
    own = getattr(study, 'cluster', None) or {}
    cells = [c for c in study.cells() if (stage is None or c.stage == stage)
             and (block is None or block in (c.block, getattr(c, 'split', None)))]
    if not cells:
        raise CompileError(f'no cells in {name}' + (f' for stage {stage}' if stage else '')
                           + (f' block {block}' if block else '') + '; crane study plan SPEC lists them')
    purpose = _purpose(purpose or own.get('purpose') or getattr(study, 'purpose', ''))
    for k in (kind, own.get('kind')):
        if k and k not in KINDS:
            raise CompileError(f'kind {k!r}: one of {", ".join(KINDS)} (crane_batch: one slot, every core; '
                               'gpu_command)')
    spec_path = spec_path or getattr(study, 'path', None)
    spec_text = Path(spec_path).expanduser().read_text() if spec_path and Path(spec_path).expanduser().is_file() else ''
    bad = field_paths(spec_text.splitlines())
    for c in cells:
        bad += field_paths(c.axes, c.condition, c.key, getattr(c, 'inputs', None))
    inputs = list(getattr(study, 'inputs', None) or [])     # scenario packs (crane env pack): pushed to /workspace/in/
    bad += field_paths(inputs)
    if bad:
        raise CompileError(f'{name} names field data ({bad[0]}); field data never goes to the cluster: run that cell '
                           'locally (crane study run), or point it at a derived table outside the corpus')
    for pack in inputs:
        if not Path(pack).expanduser().is_file():
            raise CompileError(f'{name} lists input {pack}, which does not exist; fix: crane env pack --domain D '
                               '--seeds A:B --out ' + pack)
    held = [c for c in cells if role_of(study, c) == HELD_OUT]
    skipped, skip_reason, targets = [], None, None
    if held and not confirm:
        skipped = sorted({cell_id(c) for c in held})
        skip_reason = f'{len(skipped)} held-out cell(s) left out ({skipped[0]}); they compile only with --confirm ' \
                      'and a committed targets file older than now'
        cells = [c for c in cells if role_of(study, c) != HELD_OUT]
    elif held:
        targets = held_out_guard(study, held, confirm)

    if pins is None:
        pins = {CRANE_REPO: pin_repo(crane_root, CRANE_REPO, allow_dirty, only=sparse_dirs(crane_root))}
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
    campaign = slug(campaign or own.get('campaign') or getattr(study, 'campaign', None) or name, 100)
    given_expect = expect or own.get('expect') or getattr(study, 'expect', None)
    expect = _expect(given_expect, submitter)
    is_done = done.has if isinstance(done, UnitCache) else (done or (lambda key: False))
    study_root = studies_root(out_root) / name
    packed = _study_chunks(study, stage, block)

    jobs, warnings, missing, small = Jobs(), [], [], []
    plans = []
    for c in cells:
        cid = cell_id(c)
        given = packed.get(cid) if packed is not None else None
        if packed is not None and not given:
            continue                    # the study packs no job for this cell (nothing left, or not resolvable)
        est = unit_estimate(study, c, est_unit_s)
        if est is None and given and given[0][2] is not None:
            est = float(given[0][2]) / max(1, len(given[0][1]))
        if est is None:
            missing.append(cid)
            continue
        if est > SEED_CAP_S:
            raise CompileError(f'{cid}: one seed takes {est / 60:.0f} min, over the {SEED_CAP_S / 60:.0f} min '
                               'cap; split the cell (fewer episodes per seed) or checkpoint inside it')
        if est > JOB_MAX_S:
            warnings.append(f'{cid}: one seed is {est / 60:.0f} min, over the 30 min job target; one seed per '
                            'job (interruptible boxes may lose it)')
        if given:
            parts = [(rid, seeds) for rid, seeds, _ in given
                     if not all(is_done(unit_key(c, s, commit)) for s in seeds)]
            for rid, seeds in parts:
                if est * len(seeds) > JOB_MAX_S and len(seeds) > 1:
                    warnings.append(f'{rid}: {len(seeds)} seeds are {est * len(seeds) / 60:.0f} min, over 30 min; '
                                    'lower pack.target_s in the spec')
        else:
            seeds = [s for s in c.seeds if not is_done(unit_key(c, s, commit))]
            parts = [(None, p) for p in chunks(seeds, est)]
        if not parts:
            continue
        n_units = sum(len(p) for _, p in parts)
        if est * n_units < JOB_MIN_S:
            small.append((cid, est * n_units))
        plans.append((c, est, parts))
    if small:
        warnings.append(f'{small[0][0]}' + (f' and {len(small) - 1} more cell(s)' if len(small) > 1 else '')
                        + f': the whole cell is {max(t for _, t in small) / 60:.1f} min or less, under the 5 min job '
                        'minimum; consider running it locally, or more seeds per cell')
    if missing:
        raise CompileError(f'no time estimate for {len(missing)} cell(s) ({missing[0]}); pass --est-unit-s S, '
                           '--est-from bench.json, --measure (one seed run locally), or set pack.unit_s in the spec')

    for c, est, parts in plans:
        role = role_of(study, c)
        needs, gpu = _needs(study, c, tags)
        if inputs:          # inputs reach only the boxes rented after the job is queued
            needs['tags'] = sorted({*needs['tags'], 'rented'})
        job_kind = kind or (KIND_GPU if gpu else own.get('kind') or KIND_CPU)
        if job_kind == KIND_GPU and not gpu:
            needs.update(gpu=True, vram_gb=float(needs.get('vram_gb') or 8))
        cid = cell_id(c)
        try:
            rel = cell_dir(out_root, study, c, name).relative_to(study_root).as_posix()
        except ValueError:
            rel = cell_dir(None, study, c, name).relative_to(studies_root(None) / name).as_posix()
        for i, (run_id, seeds) in enumerate(parts):
            keys = [unit_key(c, s, commit) for s in seeds]
            dedup = digest({'units': keys, 'commit': commit, 'spec': src,
                            **({'inputs': [Path(p).name for p in inputs]} if inputs else {})})
            job_id = '.'.join([_job_slug(name, 24), _job_slug(c.stage or 'main', 12), _job_slug(c.arm, 24),
                               f'c{i}', dedup[:10]])
            job = {'study': name, 'stage': c.stage, 'arm': c.arm, 'block': c.block, 'role': role, 'cell': cid,
                   'run_cell': run_id or cid, 'pass_seeds': run_id is None, 'confirm': role == HELD_OUT,
                   'touched_held_out': role == HELD_OUT, 'cell_dir': rel, 'tier': getattr(c, 'tier', None),
                   'seeds': list(seeds), 'spec_file': src, 'repos': {r['name']: r['name'] for r in repos},
                   'est_unit_s': round(est, 3), 'stall_s': int(max(600.0, 3 * est + 120)), 'dedup': dedup,
                   'commit': commit}
            if inputs:
                job['inputs'] = inputs
            if role == HELD_OUT and targets:
                job['targets'] = {k: targets.get(k) for k in ('path', 'commit', 'committed_at', 'sha256')
                                  if k in targets}
            if given_expect and not isinstance(given_expect, dict):
                job['expect'] = str(given_expect)
            if not JOB_ID.match(job_id):
                raise CompileError(f'job id {job_id!r} is not an rc-job/1 id; use a shorter study, stage or arm name')
            line = json.dumps(job, separators=(',', ':'), sort_keys=True)
            spec = {**job, 'schema': 'crane.cluster.job/1', 'repo': CRANE_REPO,
                    **({'inputs_rsync': list(inputs)} if inputs else {}),
                    'run': f'PYTHONPATH=src/{CRANE_REPO} python3 -m crane_backends.batch --job {shlex.quote(line)}',
                    'argv': ['python3', '-m', 'crane_backends.batch', '--job', line]}
            jobs.append({
                'schema': 'rc-job/1', 'job_id': job_id,
                'kind': job_kind,
                'purpose': purpose, 'est_seconds': round(est * len(seeds) + OVERHEAD_S, 1), 'submitter': submitter,
                'campaign': campaign, 'priority': int(priority) + i, 'needs': needs, 'seeds': [0], 'repos': repos,
                'spec': spec, 'outputs': {'class': 'summary'}, 'expect': expect, 'max_attempts': 3})
    jobs.warnings, jobs.skipped, jobs.skip_reason = warnings, skipped, skip_reason
    if skip_reason:
        warnings.append(skip_reason)
    return jobs


def load_study(path):
    """crane.study.load, with its StudyError as a CompileError."""
    from crane import study as study_mod
    try:
        return study_mod.load(path)
    except study_mod.StudyError as error:
        raise CompileError(str(error)) from None


def compile_spec(spec_path, stage=None, block=None, *, est_unit_s=None, est_from=None, measure_one=False,
                 skip_done=True, out=None, **kw):
    """The study spec at ``spec_path`` to rc-job/1 jobs: what `crane study compile` and `crane cluster submit SPEC`
    both run. The estimate is est_unit_s, else a bench.json's rate (est_from), else one seed measured here
    (measure_one), else the spec's pack.unit_s. Units already in the study's unit cache (OUT/cache, default
    ~/.cache/crane/studies/NAME) are left out unless skip_done is False. ``kw`` goes to compile(). Returns
    (study, jobs)."""
    src = Path(spec_path).expanduser()
    if not src.is_file():
        raise CompileError(f'no file {src}')
    study = load_study(src)
    if not est_unit_s and est_from:
        est_unit_s = bench_estimate(est_from)
    if not est_unit_s and measure_one:
        first = next(c for c in study.cells() if (not stage or c.stage == stage)
                     and (not block or block in (c.block, getattr(c, 'split', None))))
        est_unit_s = measure(src, first, first.seeds[0])
    done = None
    if skip_done:
        from crane.study.plan import default_out
        cache_dir = (Path(out).expanduser() if out else default_out(study)) / 'cache'
        if cache_dir.is_dir():
            done = UnitCache(cache_dir)
    return study, compile(study, stage=stage, block=block, spec_path=src, est_unit_s=est_unit_s, done=done, **kw)


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
