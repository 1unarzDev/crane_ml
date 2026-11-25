"""The local executor: cells to cached units, units to rows.

    summary = run_study(study, out, stage='R', max_workers=4)      # what `crane study run` does
    rows = cell.run(out)                                            # one cell (what a cluster job does)

Sim cells run through fastsim.batch.run_batch on the domain's tier config (an adapter that runs on the batched
engine only, as the sub: its run_batched), with the cell's overrides merged in
(the same reading `crane run --domain D --tier T --set k=v` makes); each run dir is read for its numbers and removed
(``keep_runs: true`` keeps them under OUT/runs/<cell>/). A cell on the batched engine (``engine: batched``) runs a
whole seed chunk in one engine call (E envs on numpy, numba or jax) and builds the same rows from the engine's
summaries, with no run directories. Task cells call ``module:function(args, seed, cell)`` or run
``argv`` with ``{args}`` (JSON), ``{seed}``, ``{out}``, ``{cell}``, ``{python}`` filled in; a task's numbers are its
return value, or its argv's OUT/metrics.json, or the last JSON line it prints.

Finished units go to OUT/cache/records.jsonl within FLUSH_S (crane.cache.UnitCache.put_many, one append and one
fsync for many; several processes may write), so a killed run loses only the units of its last second and the
same command resumes. A unit that raised is written to OUT/failures.jsonl (with its traceback) and not cached, so a
rerun retries it. After each stage the rows are
compiled from the cache: OUT/rows/<stage>.{parquet|csv}, OUT/manifest.json (sha256 per file), OUT/metrics.json.
"""

from concurrent.futures import ProcessPoolExecutor, as_completed
from dataclasses import replace
from datetime import datetime, timezone
import io
import json
import math
from multiprocessing import get_context
import os
from pathlib import Path
import shutil
import subprocess
import sys
import time
import traceback

from crane.cache import UnitCache
from crane.study import guard, plan
from crane.study.spec import StudyError

KEYS = {'study': 'str', 'stage': 'str', 'arm': 'str', 'cell': 'str', 'pair': 'str', 'block': 'str', 'split': 'str',
        'role': 'str', 'tier': 'str', 'seed': 'int', 'run_seed': 'int', 'condition': 'str'}
TAIL = {'outcome': 'str', 'error': 'str', 'censored': 'bool', 'time_cap_s': 'float'}
END = {'wall_s': 'float', 'commit': 'str', 'card_sha': 'str', 'engine': 'str', 'touched_held_out': 'bool'}
CENSORED = ('TIMEOUT', 'CENSORED', 'UNCAPTURED')   # outcomes that end at the time cap, not by the mission
METRICS_MAX = 1000
ARTIFACT_MAX = 95 * 2 ** 20      # bytes: the cluster's per-artifact cap


class _Heartbeat:
    """Ticks a Progress every ``every_s`` from a thread while long units run, so progress.json stays <= 60 s old
    and the stdout line keeps its minute even when one chunk takes longer."""

    def __init__(self, progress, every_s=20.0):
        import threading
        self.progress, self.every_s = progress, every_s
        self.stop = threading.Event()
        self.thread = threading.Thread(target=self._beat, daemon=True)

    def _beat(self):
        while not self.stop.wait(self.every_s):
            try:
                self.progress.tick()
            except OSError:
                pass

    def __enter__(self):
        self.thread.start()
        return self

    def __exit__(self, *exc):
        self.stop.set()
        self.thread.join(5)
        return False


def now():
    return datetime.now(timezone.utc).isoformat(timespec='seconds')


# ---- one unit ------------------------------------------------------------------------------------------------

def _numbers(tree, prefix=''):
    out = {}
    for key, value in (tree or {}).items():
        name = f'{prefix}.{key}' if prefix else str(key)
        if isinstance(value, dict):
            out.update(_numbers(value, name))
        elif isinstance(value, bool):
            out[name] = float(value)
        elif isinstance(value, (int, float)) and math.isfinite(float(value)):
            out[name] = float(value)
    return out


def plugin_path(cell):
    """Put the spec's plugin dirs on sys.path, for the enclosing plugin_scope (a run, a job, a report) or, in a
    worker process, for that worker's life (the study's own pool)."""
    for d in reversed(cell.ctx.get('plugins') or []):
        if d not in sys.path:
            sys.path.insert(0, d)


def plugin_scope(fn):
    """``fn`` with sys.path restored afterwards and the modules imported from the dirs it added dropped, so two
    studies run in one process never share a plugin module (e.g. both have a 'calib')."""
    import functools
    import os

    @functools.wraps(fn)
    def scoped(*args, **kw):
        path, mods = list(sys.path), set(sys.modules)
        try:
            return fn(*args, **kw)
        finally:
            added = [os.path.join(os.path.abspath(d), '') for d in set(sys.path) - set(path) if d]
            sys.path[:] = path
            for name in set(sys.modules) - mods:
                where = getattr(sys.modules[name], '__file__', None) or ''
                if any(os.path.abspath(where).startswith(d) for d in added):
                    del sys.modules[name]
    return scoped


def _load(ref, base, cell=None):
    from fastsim import plugin
    if cell is not None:
        plugin_path(cell)
    if base and base not in sys.path:
        sys.path.insert(0, base)
    return plugin.load(ref, base)


def materialize(cell, out):
    """A cell whose card is derived (card_axes) with that card written to OUT/cards/SHA.json (once) and bound to
    its card keys; any other cell as it is."""
    if cell.card_doc is None:
        return cell
    import copy
    from crane import progress
    sha = cell.key['card']
    path = Path(out).expanduser().resolve() / 'cards' / f'{sha}.json'
    if not path.is_file():
        path.parent.mkdir(parents=True, exist_ok=True)
        data = json.dumps(cell.card_doc, indent=1, sort_keys=True) + '\n'
        from crane.ids import sha256
        if sha256(data.encode()) != sha:
            raise RuntimeError(f'derived card {sha[:12]} does not hash to its key; fix: rerun crane study plan')
        progress.write_atomic(path, data)
    params = {k: (str(path) if isinstance(v, str) and v.startswith(plan.DERIVED) else v)
              for k, v in cell.params.items()}
    return replace(cell, params=params, card=str(path), card_doc=None, ctx=copy.copy(cell.ctx))


def engine_label(cell):
    e = cell.engine or {}
    return f"batched/{e['backend']}/{e['dtype']}" if e.get('mode') == 'batched' else e.get('mode', 'scalar')


def base_row(cell, seed, commit):
    row = {'study': cell.study, 'stage': cell.stage, 'arm': cell.arm, 'cell': cell.id, 'pair': cell.pair,
           'block': cell.block, 'split': cell.split, 'role': cell.role, 'tier': cell.tier, 'seed': int(seed),
           'run_seed': int(cell.run_seeds[seed]), 'condition': cell.ctx.get('condition_id'),
           'commit': commit, 'card_sha': cell.key.get('card'), 'engine': engine_label(cell),
           'touched_held_out': cell.held_out}
    row.update(cell.axes)
    for k, v in (cell.condition or {}).items():
        row[f'cond.{k}'] = v
    return row


def _score(cell, result):
    scorer = cell.ctx.get('scorer')
    if not scorer:
        return {}
    fn = _load(scorer['ref'], cell.ctx['base'], cell)
    parts = fn(result, cell.info(), **scorer.get('args', {}))
    if not isinstance(parts, dict):
        raise TypeError(f'scorer {scorer["ref"]} returned {type(parts).__name__}, not a dict of named parts')
    return {f'score.{k}': v for k, v in _numbers(parts).items()}


def _keep(cell, numbers):
    wanted = cell.ctx.get('metrics')
    return numbers if not wanted else {k: v for k, v in numbers.items() if k in wanted or k.startswith('score.')}


def _track_rows(track_path):
    with open(track_path) as f:
        for line in f:
            r = json.loads(line)
            yield r['t'], r['x'], r['y'], r['yaw']


def _decimate(track, hz, until_s):
    """An npz of (t, x, y, yaw) rows (a track.jsonl path, or the rows) at ``hz`` up to ``until_s``."""
    import numpy as np
    rows, next_t = [], -1.0
    for t, x, y, yaw in (_track_rows(track) if isinstance(track, (str, Path)) else track):
        if until_s is not None and t > until_s:
            break
        if hz is None or t >= next_t - 1e-9:
            rows.append((t, x, y, yaw))
            next_t = t + (1.0 / hz if hz else 0.0)
    arr = np.array(rows, dtype='f4').reshape(-1, 4)
    buf = io.BytesIO()
    np.savez(buf, t=arr[:, 0], x=arr[:, 1], y=arr[:, 2], yaw=arr[:, 3])
    return buf.getvalue()


def censoring(outcome, cap):
    """{censored, time_cap_s} of a unit: an episode that ran out of time (or was not captured) is censored at its
    time cap; its time metrics are the cap or absent, never NaN."""
    censored = str(outcome).upper() in CENSORED
    return {'censored': censored, 'time_cap_s': float(cap) if cap is not None and math.isfinite(float(cap)) else None}


class _Sim:
    """A sim cell's config, adapter and scenarios, made once per chunk."""

    def __init__(self, cell):
        plugin_path(cell)
        from crane.run import tier_config
        from fastsim import batch, config as fsconfig
        from fastsim.scenario import ScenarioSource
        _, path, over = tier_config(cell.ctx['domain'], cell.tier)
        self.cfg, self.base = fsconfig.load(path, fsconfig.merge(over, plan.nest(cell.params)))
        self.source = ScenarioSource(**self.cfg['scenarios'], base=self.base)
        self.adapter = batch.adapter_of(self.cfg, self.base)

    def cut_off(self, run):
        """Where the sim stops an episode that has not ended: its time limit plus the config's grace_s (a TIMEOUT's
        run.sim_s is this cap plus at most one tick; an episode that ends before it is observed, not censored)."""
        limit = run.get('time_limit_s')
        return None if limit is None else float(limit) + float(self.cfg.get('grace_s') or 0.0)

    def run(self, cell, seed, out, cache):
        from fastsim import batch
        from fastsim.pairs import flat_numbers
        run_seed = cell.run_seeds[seed]
        self.source.generate([run_seed])
        root = Path(out) / 'runs'
        engines = getattr(self.adapter, 'engines', None) or ()
        if 'batched' in engines and 'fastsim' not in engines:     # a domain with its own batched episode only
            run_dir = root / cell.id / f's{run_seed}'
            shutil.rmtree(run_dir, ignore_errors=True)            # the scalar path's force=True: always rerun
            ((_, _, _, run_dir),), _ = self.adapter.run_batched(
                self.cfg, self.base, [run_seed], root, cell.id, self.adapter.backends[0], None, 0,
                {'study': cell.study, 'cell': cell.id, 'engine': 'batched'})
            run_dir = Path(run_dir)
            run = {**json.loads((run_dir / 'run.json').read_text()),
                   **json.loads((run_dir / 'metrics.json').read_text())['summary']}
            head = {'sim_s': run['sim_s'], 'contacts': run['contacts'],
                    'min_clearance_m': self.adapter.metrics(run_dir).get('min_clearance_m')}
        else:
            result, = batch.run_batch(self.cfg, self.base, [run_seed], root, cell.id, 1,
                                      {'study': cell.study, 'cell': cell.id}, force=True, progress=False)
            run_dir = Path(result.out)
            run = json.loads((run_dir / 'run.json').read_text())
            head = {'sim_s': run['sim_s'], 'contacts': len(run['contacts']), 'min_clearance_m': run['min_clearance_m']}
        numbers = flat_numbers({'run': head})
        numbers.update(flat_numbers(self.adapter.metrics(run_dir)))
        numbers = {**_numbers(numbers), 'succeeded': float(run['outcome'] == 'SUCCEEDED')}
        res = {'metrics': numbers, 'outcome': run['outcome'], 'seed': seed, 'run_seed': run_seed,
               'run_dir': str(run_dir), 'run': run}
        numbers.update(_score(cell, res))
        blobs = {}
        rec = cell.record or {}
        if rec.get('level') in ('traj_decimated', 'obs') and (run_dir / 'track.jsonl').is_file():
            hz = rec.get('hz') if rec['level'] == 'traj_decimated' else None
            blobs['traj'] = cache.put_blob(_decimate(run_dir / 'track.jsonl', hz, rec.get('until_s')), '.npz')
        if rec.get('level') in ('events', 'traj_decimated', 'obs') and (run_dir / 'sim_events.jsonl').is_file():
            data = (run_dir / 'sim_events.jsonl').read_bytes()
            if data.strip():
                blobs['events'] = cache.put_blob(data, '.jsonl')
        if not cell.ctx.get('keep_runs'):
            shutil.rmtree(run_dir, ignore_errors=True)
        error = (run.get('error') or {}).get('error') if isinstance(run.get('error'), dict) else None
        return {'outcome': run['outcome'], 'error': error, **censoring(run['outcome'], self.cut_off(run)),
                **_keep(cell, numbers)}, blobs


class _Batch(_Sim):
    """A sim cell on the batched engine: one engine call for a chunk of seeds (E envs), rows as the scalar path's."""

    def run_chunk(self, cell, seeds, cache):
        """[(seed, numbers, blobs)] of every seed, from one engine call."""
        from engine.episode import Setup, run
        from fastsim.pairs import flat_numbers
        from fastsim.run import line
        run_seeds = [cell.run_seeds[s] for s in seeds]
        self.source.generate(run_seeds)
        rec = cell.record or {}
        level = rec.get('level', 'metrics')
        setup = Setup(self.cfg, self.base, self.adapter, run_seeds, workers=1)
        eng = cell.engine
        out = run(setup, eng['backend'], eng['dtype'], record=1 if level == 'traj_decimated' else 0)
        result = []
        for e, seed in enumerate(seeds):
            s = out.summary(e)
            got = out.metrics(e)
            numbers = flat_numbers({k: v for k, v in got.items() if k not in ('outcome', 'run_sha256', 'flags')})
            numbers = {**_numbers(numbers), 'succeeded': float(s['outcome'] == 'SUCCEEDED')}
            res = {'metrics': numbers, 'outcome': s['outcome'], 'seed': seed, 'run_seed': run_seeds[e],
                   'run_dir': None, 'run': s}
            numbers.update(_score(cell, res))
            blobs = {}
            if level == 'traj_decimated':
                blobs['traj'] = cache.put_blob(_decimate(out.track(e), rec.get('hz'), rec.get('until_s')), '.npz')
            if level in ('events', 'traj_decimated'):
                data = ''.join(line(ev) for ev in out.events(e)).encode()
                if data.strip():
                    blobs['events'] = cache.put_blob(data, '.jsonl')
            result.append((seed, {'outcome': s['outcome'], 'error': None,
                                  **censoring(s['outcome'], self.cut_off(s)), **_keep(cell, numbers)}, blobs))
        return result


def _task(cell, seed, out):
    import shlex
    run_seed = cell.run_seeds[seed]
    args = plan.nest(cell.params)
    ctx = cell.ctx
    if ctx.get('task'):
        fn = _load(ctx['task'], ctx['base'], cell)
        got = fn(args, run_seed, cell.info())
    else:
        unit_dir = Path(out) / 'units' / cell.id / f's{seed}'
        unit_dir.mkdir(parents=True, exist_ok=True)
        fill = {'args': json.dumps(args, sort_keys=True), 'seed': str(run_seed), 'out': str(unit_dir),
                'cell': json.dumps(cell.info(), sort_keys=True), 'python': sys.executable, 'spec_dir': ctx['base']}
        argv = [a.format(**fill) for a in ctx['argv']]
        try:
            r = subprocess.run(argv, cwd=ctx['base'], capture_output=True, text=True, timeout=ctx.get('timeout_s'))
        except subprocess.TimeoutExpired:
            raise TimeoutError(f'{shlex.join(argv)[:200]} ran past timeout_s {ctx.get("timeout_s")}') from None
        if r.returncode != 0:
            raise RuntimeError(f'exit {r.returncode}: {shlex.join(argv)[:200]}\n{r.stderr[-2000:]}')
        got = None
        if (unit_dir / 'metrics.json').is_file():
            got = json.loads((unit_dir / 'metrics.json').read_text())
        else:
            for line in reversed(r.stdout.strip().splitlines()):
                try:
                    got = json.loads(line)
                    break
                except ValueError:
                    continue
        if not isinstance(got, dict):
            raise ValueError(f'{shlex.join(argv)[:200]} wrote no metrics.json in {{out}} and printed no JSON object')
    if not isinstance(got, dict):
        raise TypeError(f'task {ctx["task"]} returned {type(got).__name__}, not a dict of numbers')
    outcome = str(got.get('outcome', 'ok'))
    cens = censoring(outcome, got.get('time_cap_s', args.get('time_limit_s') if isinstance(args, dict) else None))
    if got.get('censored'):
        cens['censored'] = True
    if cens['censored'] and cens['time_cap_s'] is None:
        raise ValueError(f'task {ctx.get("task") or ctx["argv"][0]} returned a censored unit without its time cap; '
                         'fix: return time_cap_s with censored (never NaN)')
    numbers = _numbers({k: v for k, v in got.items() if k not in ('outcome', 'censored', 'time_cap_s')})
    numbers.update(_score(cell, {'metrics': numbers, 'outcome': outcome, 'seed': seed, 'run_seed': run_seed,
                                 'run_dir': None, 'run': got}))
    return {'outcome': outcome, 'error': None, **cens, **_keep(cell, numbers)}, {}


def _failure(cell, seed, error):
    first = str(error).splitlines()[0] if str(error) else ''
    return {'cell': cell.id, 'stage': cell.stage, 'arm': cell.arm, 'seed': seed, 'role': cell.role, 'at': now(),
            'error': f'{type(error).__name__}: {first}', 'traceback': traceback.format_exc()[-4000:]}


def _work_batched(cell, seeds, out, commit, cache):
    """The batched engine's chunk: the missing seeds in one engine call; a raise fails every seed of it."""
    done, todo = [], []
    for seed in seeds:
        if cache.has(cell.unit(seed, commit)):
            done.append((seed, True, None))
        else:
            todo.append(seed)
    if not todo:
        return done
    started = time.perf_counter()
    try:
        got = _Batch(cell).run_chunk(cell, todo, cache)
    except Exception as error:  # noqa: BLE001  the chunk failed: every unit of it is a failure, retried on rerun
        return done + [(seed, False, _failure(cell, seed, error)) for seed in todo]
    wall = round((time.perf_counter() - started) / len(todo), 4)
    pending = []
    for seed, numbers, blobs in got:
        row = {**base_row(cell, seed, commit), **numbers, 'wall_s': wall}
        pending.append((cell.unit(seed, commit), {'row': row, 'blobs': blobs}))
        done.append((seed, numbers.get('outcome') != 'ERROR', None))
    cache.put_many(pending)
    return done


_CACHES = {}


def _unit_cache(out):
    """One UnitCache per OUT per process: it reads only what was appended since (a fresh one rereads it all)."""
    root = str(Path(out) / 'cache')
    try:
        st = os.stat(Path(root) / 'records.jsonl')
        stamp = (st.st_dev, st.st_ino)
    except FileNotFoundError:
        stamp = None
    held = _CACHES.get(root)
    if held is None or held[0] != stamp or (stamp and held[1]._offset > st.st_size):
        held = _CACHES[root] = (stamp, UnitCache(root))
    return held[1]


FLUSH_S = 1.0   # finished units wait at most this long (or the chunk's end) before one locked append + fsync


def work(cell, seeds, out, commit):
    """Run ``seeds`` of ``cell`` and cache the finished units at least every FLUSH_S (one append and one fsync
    for all of them, so many sub-millisecond units stay cheap). Returns [(seed, ok, failure|None)]."""
    cache = _unit_cache(out)
    cell = materialize(cell, out)
    if cell.engine.get('mode') == 'batched':
        return _work_batched(cell, seeds, out, commit, cache)
    done, sim, pending, flushed = [], None, [], time.perf_counter()
    for seed in seeds:
        if pending and time.perf_counter() - flushed >= FLUSH_S:
            cache.put_many(pending)
            pending, flushed = [], time.perf_counter()
        unit = cell.unit(seed, commit)
        if cache.has(unit):
            done.append((seed, True, None))
            continue
        started = time.perf_counter()
        try:
            if cell.kind == 'sim':
                sim = sim or _Sim(cell)
                numbers, blobs = sim.run(cell, seed, out, cache)
            else:
                numbers, blobs = _task(cell, seed, out)
        except Exception as error:  # noqa: BLE001  one unit failed: record it, keep going
            done.append((seed, False, _failure(cell, seed, error)))
            continue
        row = {**base_row(cell, seed, commit), **numbers, 'wall_s': round(time.perf_counter() - started, 4)}
        pending.append((unit, {'row': row, 'blobs': blobs}))
        done.append((seed, numbers.get('outcome') != 'ERROR', None))
    cache.put_many(pending)
    return done


def _note_failures(out, failures):
    if failures:
        with open(Path(out) / 'failures.jsonl', 'a') as f:
            for x in failures:
                f.write(json.dumps(x, default=str) + '\n')


def run_cell(cell, out, progress=None, workers=1, commit=None):
    """Run one cell's missing units into OUT; returns the rows of its seeds that are done."""
    commit = commit or plan.code_commit()
    out = Path(out)
    out.mkdir(parents=True, exist_ok=True)
    results = work(cell, cell.seeds, out, commit)
    _note_failures(out, [f for _, _, f in results if f])
    if progress is not None:
        for _, ok, _ in results:
            progress.step(failed=not ok)
    cache = UnitCache(out / 'cache')
    return [cache.get(cell.unit(s, commit))['row'] for s in cell.seeds if cache.has(cell.unit(s, commit))]


# ---- the study -----------------------------------------------------------------------------------------------

def _chunks(cell, seeds, study):
    size = (study.stages[cell.stage].pack or {}).get('local_chunk') or study.pack.get('local_chunk') \
        or plan.CHUNK['batched' if cell.engine.get('mode') == 'batched' else cell.kind]
    return [seeds[i:i + size] for i in range(0, len(seeds), size)]


def _spec_ref(path):
    """The spec path an output records: relative to the crane checkout, else with ~ for home (no /home ships)."""
    from crane import paths
    path = Path(path).resolve()
    for base, prefix in ((paths.ROOT.resolve(), ''), (Path.home().resolve(), '~/')):
        try:
            return prefix + path.relative_to(base).as_posix()
        except ValueError:
            pass
    return str(path)


def write_state(study, out):
    from crane import progress
    from crane.ids import sha256
    doc = {'schema': 'crane.study.out/1', 'study': study.name, 'spec': _spec_ref(study.path),
           'spec_sha256': sha256(study.path.read_bytes()), 'commit': plan.code_commit(), 'updated': now()}
    progress.write_atomic(Path(out) / 'study.json', json.dumps(doc, indent=1) + '\n')


@plugin_scope
def run_study(study, out, stage=None, block=None, confirm=False, max_workers=1, say=print, stream=None):
    """Run the study's cells (one stage, or every stage in order) into OUT. Returns a summary dict:
    {ran, reused, failed, skipped_held_out: [names], touched_held_out, stages: {name: {...}}, refused: str|None}."""
    from crane.progress import Progress
    out = Path(out).expanduser()
    out.mkdir(parents=True, exist_ok=True)
    commit = plan.code_commit()
    names = [stage] if stage else study.order()
    if stage and stage not in study.stages:
        raise StudyError(f'{study.path.name}: no stage {stage!r}; stages: {", ".join(study.stages)}')
    summary = {'ran': 0, 'reused': 0, 'failed': 0, 'skipped_held_out': [], 'touched_held_out': False,
               'stages': {}, 'refused': None, 'out': str(out), 'commit': commit}
    if block and block not in study.blocks and block not in study.splits:
        raise StudyError(f'{study.path.name}: no block or split {block!r}; blocks: '
                         f'{", ".join([*study.blocks, *study.splits])}')
    write_state(study, out)
    targets = None
    for name in names:
        try:
            cells = plan.cells(study, name, block, out)
        except plan.StageNotReady as error:
            summary['refused'] = str(error)
            return summary
        held = [c for c in cells if c.held_out]
        if held and not confirm and not block:
            skipped = sorted({c.split if c.split and study.splits.get(c.split) == 'held-out' else c.block
                              for c in held})
            summary['skipped_held_out'] += [f'{name}:{s}' for s in skipped]
            cells = [c for c in cells if not c.held_out]
        elif held:
            reason, targets = guard.refuse(study, held, confirm, out=out)
            if reason:
                summary['refused'] = reason
                return summary
        cache = UnitCache(out / 'cache')
        todo, reused = [], 0
        for cell in cells:
            missing = [s for s in cell.seeds if not cache.has(cell.unit(s, commit))]
            reused += len(cell.seeds) - len(missing)
            todo += [(cell, chunk) for chunk in _chunks(cell, missing, study)] if missing else []
        _pregenerate(todo)
        total = sum(len(c.seeds) for c in cells)
        stats = {'cells': len(cells), 'units': total, 'reused': reused, 'ran': 0, 'failed': 0}
        if any(c.held_out for c in cells):
            _log_held_out(out, name, cells, targets)
            summary['touched_held_out'] = True
        p = Progress(out, total, phase=f'stage {name}', label=study.name, stream=stream)
        p.skip(reused)
        failures = []
        beat = _Heartbeat(p).__enter__()
        try:
            if max_workers <= 1 or len(todo) <= 1:
                for cell, chunk in todo:
                    _account(work(cell, chunk, out, commit), p, stats, failures)
                    _note_failures(out, failures)
                    failures.clear()
            else:
                ctx = get_context('spawn')
                with ProcessPoolExecutor(max_workers, mp_context=ctx) as pool:
                    futures = [pool.submit(work, cell, chunk, str(out), commit) for cell, chunk in todo]
                    try:
                        for fut in as_completed(futures):
                            _account(fut.result(), p, stats, failures)
                            _note_failures(out, failures)
                            failures.clear()
                    except BaseException:
                        for fut in futures:
                            fut.cancel()
                        raise
        except BaseException:
            p.close('interrupted')
            compile_rows(study, out)
            raise
        finally:
            beat.__exit__()
        p.close('done' if not stats['failed'] else 'done-with-failures')
        summary['stages'][name] = stats
        summary['ran'] += stats['ran']
        summary['reused'] += reused
        summary['failed'] += stats['failed']
        _prune(out)
        compile_rows(study, out)
        if stats['failed'] and name != names[-1]:
            dependents = [n for n in names[names.index(name) + 1:] if name in study.stages[n].after]
            if dependents:
                summary['refused'] = (f'stage {name} has {stats["failed"]} failed units, so {", ".join(dependents)} '
                                      f'cannot select yet; fix: crane study status {out}, then rerun')
                return summary
    return summary


def _pregenerate(todo):
    """Write the missing scenarios of the sim units to run, once per scenario source, before any worker starts
    (two workers must never write the same scenario file)."""
    from crane.ids import digest
    from crane.run import tier_config
    from fastsim import config as fsconfig
    from fastsim.scenario import ScenarioSource
    groups = {}
    for cell, seeds in todo:
        if cell.kind != 'sim':
            continue
        plugin_path(cell)
        scen = {k: v for k, v in cell.params.items() if k.startswith('scenarios.')}
        key = digest([cell.ctx['domain'], cell.tier, scen])
        groups.setdefault(key, (cell, set()))[1].update(cell.run_seeds[s] for s in seeds)
    for cell, run_seeds in groups.values():
        _, path, over = tier_config(cell.ctx['domain'], cell.tier)
        cfg, base = fsconfig.load(path, fsconfig.merge(over, plan.nest(cell.params)))
        ScenarioSource(**cfg['scenarios'], base=base).generate(sorted(run_seeds))


def _prune(out):
    """Remove the empty run dirs a stage leaves (its run dirs are read and removed unless keep_runs)."""
    root = Path(out) / 'runs'
    if root.is_dir():
        for d in sorted(root.iterdir(), reverse=True):
            if d.is_dir() and not any(d.iterdir()):
                d.rmdir()
        if not any(root.iterdir()):
            root.rmdir()


def _account(results, p, stats, failures):
    for _, ok, failure in results:
        if failure:
            failures.append(failure)
            stats['failed'] += 1
        else:
            stats['ran'] += 1
        p.step(failed=not ok)


def _log_held_out(out, stage, cells, targets):
    blocks = sorted({c.split if c.split and c.role == 'held-out' and c.split else c.block for c in cells
                     if c.held_out})
    entry = {'at': now(), 'stage': stage, 'blocks': blocks, 'units': sum(len(c.seeds) for c in cells if c.held_out),
             'targets': targets}
    with open(Path(out) / 'held_out.jsonl', 'a') as f:
        f.write(json.dumps(entry) + '\n')


# ---- rows, manifest, metrics.json ----------------------------------------------------------------------------

def schema(study, rows):
    """The study's row schema: the key columns, its axes and condition columns, outcome, its metrics (the spec's
    ``metrics:`` when given, else every number the rows carry, sorted), its scorer parts, then wall_s and the
    identity columns."""
    cols = dict(KEYS)
    cols.update({a.name: 'str' for a in [*study.axes, *study.card_axes]})
    cols.update({f'cond.{c}': 'str' for c in study.condition_columns})
    for c in study.condition_columns:
        values = [r['row'][c] for r in study.conditions or []]
        if values and all(isinstance(v, (int, float)) for v in values):
            cols[f'cond.{c}'] = 'float'
    cols.update(TAIL)
    fixed = set(cols) | set(END)
    seen = sorted({k for r in rows for k, v in r.items() if k not in fixed and isinstance(v, (int, float))})
    names = study.metrics or [k for k in seen if not k.startswith('score.')]
    names = [n for n in names if not n.startswith('score.')]
    if study.metrics and 'succeeded' in seen and 'succeeded' not in names:
        names = ['succeeded', *names]
    cols.update({k: 'float' for k in names})
    cols.update({k: 'float' for k in seen if k.startswith('score.')})
    cols.update(END)
    return cols


def compile_rows(study, out, cells=None, part=None):
    """Rows of every cached unit of ``cells`` (every resolvable cell) into OUT/rows/, plus OUT/manifest.json and
    OUT/metrics.json. Returns the manifest."""
    from crane.data.rows import RowWriter, write_manifest
    out = Path(out)
    commit = plan.code_commit()
    cells = plan.cells(study, None, None, out) if cells is None else cells
    cache = UnitCache(out / 'cache')
    by_stage, blobs, units, cards = {}, [], {}, {}
    for cell in cells:
        if cell.card_doc is not None:
            cards[cell.key['card']] = {'levels': cell.card_doc['crane_derived']['levels'],
                                       'file': f"cards/{cell.key['card']}.json"}
        u = units.setdefault(cell.stage, {}).setdefault(_block_name(study, cell), {'role': cell.role, 'done': 0,
                                                                                     'total': 0})
        u['total'] += len(cell.seeds)
        for seed in cell.seeds:
            rec = cache.get(cell.unit(seed, commit))
            if rec is None:
                continue
            u['done'] += 1
            by_stage.setdefault(cell.stage, []).append(rec['row'])
            if rec.get('blobs'):
                blobs.append((cell.id, seed, rec['blobs']))
    all_rows = [r for rows in by_stage.values() for r in rows]
    cols = schema(study, all_rows)
    rows_dir = out / 'rows'
    files = []
    names = set()
    for stage, rows in by_stage.items():
        w = RowWriter(rows_dir, cols, part=part or stage)
        for r in rows:
            w.write(r)
        for f in w.close():
            names.add(f['path'])
            files.append({**f, 'path': f'rows/{f["path"]}', 'kind': 'rows'})
    if part is None and rows_dir.is_dir():
        for stale in rows_dir.iterdir():
            if stale.name not in names and not stale.name.startswith('.'):
                stale.unlink()
    files += _extras(out, cache, blobs)
    held = [json.loads(x) for x in (out / 'held_out.jsonl').read_text().splitlines()] \
        if (out / 'held_out.jsonl').is_file() else []
    failures = _failures(out, cache, commit)
    big = [f['path'] for f in files if (out / f['path']).stat().st_size > ARTIFACT_MAX]
    extra = {'study': study.name, 'spec': study.path.name, 'domain': study.domain, 'commit': commit, 'updated': now(),
             'baseline': study.baseline, 'axes': [a.name for a in [*study.axes, *study.card_axes]],
             'record': study.record['level'],
             'units': units, 'failed_units': len(failures),
             'touched_held_out': any(r.get('touched_held_out') for r in all_rows) or bool(held),
             'held_out': held[-1] if held else None,
             'oversize': big}
    if study.plugins:
        extra['plugins'] = plan.plugin_state(study)       # each dir's git commit + dirty diff sha, and its files' sha
    if cards:
        extra['cards'] = cards                            # derived card sha -> its card-axis levels
    manifest = write_manifest(out, cols, files, extra)
    _metrics_json(out, units, all_rows, len(failures))
    return manifest


def _block_name(study, cell):
    return plan.label(cell.block, cell.split)


def _extras(out, cache, blobs):
    import hashlib
    import numpy as np
    files = []
    traj = [(c, s, b['traj']) for c, s, b in blobs if b.get('traj')]
    if traj:
        cells, seeds, offsets, parts = [], [], [0], {k: [] for k in ('t', 'x', 'y', 'yaw')}
        for c, s, sha in traj:
            path = cache.blob_path(sha, '.npz')
            if path is None:
                continue
            with np.load(path) as z:
                for k in parts:
                    parts[k].append(z[k])
                n = len(z['t'])
            cells.append(c)
            seeds.append(s)
            offsets.append(offsets[-1] + n)
        path = out / 'traj.npz'
        with open(path.with_name('.traj.npz.tmp'), 'wb') as f:
            np.savez(f, cell=np.array(cells), seed=np.array(seeds, dtype='i8'), offset=np.array(offsets, dtype='i8'),
                     **{k: np.concatenate(v) if v else np.zeros(0, 'f4') for k, v in parts.items()})
        path.with_name('.traj.npz.tmp').replace(path)
        files.append({'path': path.name, 'rows': len(cells), 'sha256': hashlib.sha256(path.read_bytes()).hexdigest(),
                      'kind': 'traj'})
    events = [(c, s, b['events']) for c, s, b in blobs if b.get('events')]
    if events:
        path, n = out / 'events.jsonl', 0
        with open(path.with_name('.events.jsonl.tmp'), 'w') as f:
            for c, s, sha in events:
                data = cache.get_blob(sha, '.jsonl') or b''
                for line in data.decode().splitlines():
                    if line.strip():
                        f.write(json.dumps({'cell': c, 'seed': s, **json.loads(line)}) + '\n')
                        n += 1
        path.with_name('.events.jsonl.tmp').replace(path)
        files.append({'path': path.name, 'rows': n, 'sha256': hashlib.sha256(path.read_bytes()).hexdigest(),
                      'kind': 'events'})
    return files


def _failures(out, cache, commit):
    """The failures not since fixed (a unit later cached is no longer a failure), latest per unit."""
    path = Path(out) / 'failures.jsonl'
    if not path.is_file():
        return []
    done = {(e['unit'].get('stage'), e['record']['row']['cell'], e['unit'].get('seed')) for e in cache
            if e.get('unit') and e['unit'].get('commit') == commit}
    latest = {}
    for line in path.read_text().splitlines():
        try:
            f = json.loads(line)
        except ValueError:
            continue
        latest[(f.get('stage'), f.get('cell'), f.get('seed'))] = f
    return [f for k, f in latest.items() if k not in done]


def _metrics_json(out, units, rows, failed):
    """OUT/metrics.json: at most 1000 finite numbers: unit counts per stage and block, and per-arm means of the
    rows of dev blocks (guard.shows_means; held-out, check, train and calib blocks give counts only)."""
    numbers = {'units.done': sum(b['done'] for s in units.values() for b in s.values()),
               'units.total': sum(b['total'] for s in units.values() for b in s.values()), 'units.failed': failed}
    for stage, blocks in units.items():
        for name, b in blocks.items():
            numbers[f'{stage}.{name}.done'] = b['done']
            numbers[f'{stage}.{name}.total'] = b['total']
    sums = {}
    for r in rows:
        if not guard.shows_means(r.get('role')):
            continue
        for k, v in r.items():
            if k in KEYS or k in END or not isinstance(v, (int, float)) or isinstance(v, bool):
                continue
            key = f"{r['stage']}.{r['arm']}.{k}"
            s = sums.setdefault(key, [0.0, 0])
            s[0] += v
            s[1] += 1
    for key in sorted(sums):
        if len(numbers) >= METRICS_MAX:
            break
        total, n = sums[key]
        numbers[f'mean.{key}'] = round(total / n, 6)
    numbers = {k: v for k, v in numbers.items() if isinstance(v, (int, float)) and math.isfinite(v)}
    from crane import progress
    progress.write_atomic(Path(out) / 'metrics.json', json.dumps(numbers, indent=1) + '\n')


# ---- run-cell: one cluster job -------------------------------------------------------------------------------

@plugin_scope
def run_job(study, job_id, out, confirm=False, stream=None, targets_sha256=None, only=None):
    """One job (a cell, or one of its seed chunks) into OUT: what `crane study run-cell` does on a box.
    ``targets_sha256`` (the job argv's --targets-sha256) stands in for git on a box with no study checkout.
    ``only``: run just these of its seeds (crane cluster submit --measure times one). Returns (summary, refusal)."""
    from crane.progress import Progress
    out = Path(out).expanduser()
    cell, seeds = plan.find_job(study, job_id)
    if only is not None:
        extra = sorted(set(only) - set(seeds))
        if extra:
            raise StudyError(f'seed {extra[0]} is not in {job_id} (its seeds: {seeds[0]}..{seeds[-1]}); fix: crane '
                             f'study plan {study.ref} --json')
        seeds = [s for s in seeds if s in set(only)]
    targets = None
    if cell.held_out:
        fix = f'crane study run-cell {study.ref} --cell {job_id} --out {out} --confirm'
        reason, targets = guard.refuse(study, [cell], confirm, pinned=targets_sha256, out=out, fix=fix)
        if reason:
            return None, reason
    cell = replace(cell, seeds=list(seeds))
    out.mkdir(parents=True, exist_ok=True)
    write_state(study, out)
    if study.inputs:        # scenario packs the job pushed (or the local ones): their files join the scenario path
        from crane import pack
        try:
            pack.unpack_inputs(study.inputs, out)
        except pack.PackError as error:
            raise StudyError(str(error)) from None
    if cell.held_out:
        _log_held_out(out, cell.stage, [cell], targets)
    commit = plan.code_commit()
    cache = UnitCache(out / 'cache')
    missing = [s for s in cell.seeds if not cache.has(cell.unit(s, commit))]
    _pregenerate([(cell, missing)])      # the scenarios, or the one-line refusal before any work starts
    p = Progress(out, len(cell.seeds), phase='run', label=job_id, stream=stream)
    p.skip(len(cell.seeds) - len(missing))
    stats, failures = {'ran': 0, 'failed': 0}, []
    beat = _Heartbeat(p).__enter__()
    try:
        for chunk in _chunks(cell, missing, study):
            _account(work(cell, chunk, out, commit), p, stats, failures)
            _note_failures(out, failures)
            failures.clear()
    except BaseException:
        p.close('interrupted')
        compile_rows(study, out, [cell], part=_safe(job_id))
        raise
    finally:
        beat.__exit__()
    p.close('done' if not stats['failed'] else 'done-with-failures')
    _prune(out)
    manifest = compile_rows(study, out, [cell], part=_safe(job_id))
    return {'job_id': job_id, 'cell': cell.id, 'units': len(cell.seeds), 'reused': len(cell.seeds) - len(missing),
            **stats, 'out': str(out), 'rows': manifest['rows'], 'oversize': manifest.get('oversize') or []}, None


def _safe(name):
    return ''.join(c if c.isalnum() or c in '._-' else '_' for c in name)

