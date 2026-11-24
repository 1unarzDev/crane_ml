"""Batch runner: one domain on many seeds, across worker processes.

    python -m fastsim.batch --config DOMAIN.yaml --seeds 1:21 --out OUT [--variant NAME]
                            [--workers N] [--generate] [--calibration FILE] [--allow-dirty] [--force]

Each seed's scenario comes from the config's ``scenarios`` source; each run is
``OUT/<variant>/s<seed>`` (run.py). A dirty adapter tree is refused unless ``--allow-dirty``
(then every run records it). Workers are spawned processes, never above ``max_workers``.

Rerunning resumes: a seed whose run directory holds the same run identity is reused, not rerun; its
run.json keeps the provenance (git state, ``--allow-dirty``) of the run that wrote it, since provenance is
not part of the identity. A reused ERROR run is moved to ``<dir>.stale-N`` and run again. A directory
with another identity, or an incomplete one, is refused (``--force`` moves it to ``<dir>.stale-N``). Each run is written to a hidden ``.s<seed>.partial-<pid>`` beside it and renamed
into place, so a run directory is always whole. ``OUT/progress.json`` (crane.progress; under
``$CRANE_RUN_DIR`` when set) tracks the latest batch into OUT, its variant as ``label``; ``OUT/<variant>``
holds only run directories.
"""

import argparse
from dataclasses import dataclass
import json
from multiprocessing import TimeoutError as PoolTimeout, get_context
import os
from pathlib import Path
import shutil
import sys
import time

from fastsim import config as fsconfig, identity, plugin
from fastsim.config import merge
from fastsim.run import CRANE_RUN, UNHASHED, simulate
from fastsim.scenario import ScenarioSource


@dataclass(frozen=True)
class Result:
    out: str
    seed: int
    outcome: str
    sim_s: float
    wall_s: float
    run_sha256: str


def parse_seeds(text):
    """'N', 'FROM:TO' (TO exclusive) or a comma list; with crane beside fastsim also '@FILE' and the rest of
    crane.select.parse_seeds' forms. A bad spec raises ValueError (argparse's type= turns it into a usage error)."""
    try:
        from crane.select import parse_seeds as parse
    except ImportError:  # fastsim without crane beside it: the three plain forms
        if ',' in text:
            return [int(s) for s in text.split(',')]
        lo, _, hi = text.partition(':')
        return list(range(int(lo), int(hi))) if hi else [int(lo)]
    return parse(text)


def adapter_of(cfg, base):
    spec = cfg.get('adapter') or {}
    if 'ref' not in spec:
        raise ValueError('config: adapter.ref (MODULE:CLASS or FILE.py:CLASS) is required')
    return plugin.load(spec['ref'], base)(**(spec.get('options') or {}), base=base)


def adapter_source(cfg, base):
    """Where the adapter's code lives (for its git state), else None."""
    ref = (cfg.get('adapter') or {}).get('ref', '')
    target = ref.rpartition(':')[0]
    if target.endswith('.py'):
        return Path(base) / target
    try:
        import importlib.util
        found = importlib.util.find_spec(target.split('.')[0])
        return Path(found.origin) if found and found.origin else None
    except (ImportError, ValueError):
        return None


class RunDirConflict(FileExistsError):
    """A seed's run directory exists but is not this run (another identity, or incomplete)."""


_STATE = {}


def _init(cfg, base, provenance):
    # A failing initializer makes a Pool respawn workers forever: keep the error, raise it per job.
    try:
        _STATE.update(cfg=cfg, base=base, provenance=provenance, adapter=adapter_of(cfg, base),
                      source=ScenarioSource(**cfg['scenarios'], base=base))
    except Exception as error:  # noqa: BLE001
        _STATE['error'] = error


def expected_identity(cfg, adapter, scenario, seed):
    """The run identity simulate() would record for these inputs, without running."""
    merged = merge(cfg, adapter.overrides() or {})
    hashed = {k: v for k, v in merged.items() if k not in UNHASHED}
    return identity.run_identity(scenario.content_sha256, hashed, adapter.identity(), seed, identity.core_version())


def _existing(out):
    """run.json of a whole run directory, else None (missing files or unreadable)."""
    out = Path(out)
    try:
        run = json.loads((out / 'run.json').read_text())
    except (OSError, ValueError):
        return None
    return run if (out / CRANE_RUN).is_file() and isinstance(run.get('identity'), dict) else None


def _reused(out, run):
    return Result(str(out), int(run['seed']), run['outcome'], run['sim_s'], 0.0, run['identity']['run_sha256'])


def _pid_alive(pid):
    try:
        os.kill(pid, 0)
    except ProcessLookupError:
        return False
    except OSError:
        return True
    return True


def _clear_partials(out):
    """Remove hidden partial directories of ``out`` left by processes that are gone."""
    out = Path(out)
    for partial in out.parent.glob(f'.{out.name}.partial-*') if out.parent.is_dir() else ():
        pid = partial.name.rpartition('-')[2]
        if not pid.isdigit() or not _pid_alive(int(pid)):
            shutil.rmtree(partial, ignore_errors=True)


def _stale(out):
    n = 1
    while Path(f'{out}.stale-{n}').exists():
        n += 1
    return Path(f'{out}.stale-{n}')


def _check(out, want, force):
    """For an existing ``out``: its Result when it is this run; else refuse, or with ``force`` move it aside."""
    out = Path(out)
    if not out.exists() or (out.is_dir() and not any(out.iterdir())):
        return None
    run = _existing(out)
    have = (run or {}).get('identity', {}).get('run_sha256')
    if have == want['run_sha256'] and run.get('outcome') != 'ERROR':
        return _reused(out, run)
    if have == want['run_sha256']:  # this run, but it errored: keep it aside and run it again
        out.rename(_stale(out))
        return None
    if force:
        out.rename(_stale(out))
        return None
    why = 'is incomplete' if run is None else f'holds another run (identity {have[:12]}, this run {want["run_sha256"][:12]})'
    raise RunDirConflict(f'{out} {why}; rerun with --force to move it to {_stale(out).name}, or pick another --out')


def _one(job):
    index, seed, out = job
    s = _STATE
    if 'error' in s:
        raise RuntimeError(f"worker set-up failed: {type(s['error']).__name__}: {s['error']}") from s['error']
    started = time.perf_counter()
    out = Path(out)
    partial = out.parent / f'.{out.name}.partial-{os.getpid()}'
    shutil.rmtree(partial, ignore_errors=True)
    summary = simulate(s['source'].load(seed), s['adapter'], s['cfg'], seed, partial, s['base'],
                       s['provenance'])
    try:
        partial.rename(out)
    except OSError:  # another process wrote this run meanwhile: keep it when it is the same run
        run = _existing(out)
        shutil.rmtree(partial, ignore_errors=True)
        if not run or run['identity']['run_sha256'] != summary['identity']['run_sha256']:
            raise RunDirConflict(f'{out} appeared during the run with another identity; rerun with --force') from None
    return index, Result(str(out), seed, summary['outcome'], summary['sim_s'],
                         round(time.perf_counter() - started, 3), summary['identity']['run_sha256'])


def run_batch(cfg, base, seeds, out, variant='base', workers=None, provenance=None, *, force=False, progress=True):
    """Run every seed; returns [Result] in seed order. A seed whose run directory already holds this run is
    reused (its wall_s is 0.0); one holding another run or a partial one raises RunDirConflict, unless ``force``
    moves it aside. ``progress`` True writes ``OUT/progress.json`` (crane.progress), False nothing,
    or pass a crane.progress.Progress."""
    jobs = [(i, int(s), str(Path(out) / variant / f's{s}')) for i, s in enumerate(seeds)]
    workers = max(1, min(workers or cfg['workers'], cfg['max_workers'], len(jobs)))
    provenance = {'variant': variant, **(provenance or {})}
    results, todo = [None] * len(jobs), []
    if any(Path(o).exists() for _, _, o in jobs):
        adapter, source = adapter_of(cfg, base), ScenarioSource(**cfg['scenarios'], base=base)
        for i, seed, o in jobs:
            reused = Path(o).exists() and _check(o, expected_identity(cfg, adapter, source.load(seed), seed), force)
            if reused:
                results[i] = reused
    for job in jobs:
        _clear_partials(job[2])
        if results[job[0]] is None:
            todo.append(job)
    tracker = _progress(progress, out, variant, len(jobs))
    try:
        if tracker:
            tracker.skip(len(jobs) - len(todo))
        if workers <= 1 or len(todo) <= 1:
            if todo:
                _init(cfg, base, provenance)
            for job in todo:
                i, result = _one(job)
                results[i] = result
                tracker and tracker.step(failed=result.outcome == 'ERROR')
        else:
            with get_context('spawn').Pool(min(workers, len(todo)), _init, (cfg, base, provenance)) as pool:
                done = pool.imap_unordered(_one, todo, chunksize=1)
                for _ in todo:
                    while True:
                        try:
                            i, result = done.next(timeout=tracker.every_s if tracker else None)
                            break
                        except PoolTimeout:  # nothing finished: keep progress.json fresh
                            tracker.tick()
                    results[i] = result
                    tracker and tracker.step(failed=result.outcome == 'ERROR')
    except BaseException:
        if tracker:
            tracker.close('failed')
        raise
    if tracker:
        tracker.close()
    return results


def _progress(progress, out, variant, total):
    if progress is False or progress is None:
        return None
    if progress is True:
        try:
            from crane.progress import Progress
        except ImportError:  # fastsim without crane beside it
            return None
        return Progress(out, total, phase='run', label=variant)
    return progress


def main(argv=None):
    parser = argparse.ArgumentParser(prog='fastsim.batch', description=__doc__.split('\n')[0])
    parser.add_argument('--config', required=True, type=Path, help='the domain YAML')
    parser.add_argument('--seeds', required=True, type=parse_seeds, help='N, FROM:TO or a,b,c')
    parser.add_argument('--out', required=True, type=Path)
    parser.add_argument('--variant', default='base', help='name of this design/parameter set')
    parser.add_argument('--workers', type=int)
    parser.add_argument('--calibration', type=Path, help='a calibration epoch file (overrides the YAML)')
    parser.add_argument('--generate', action='store_true', help="write missing scenarios with the source's generator")
    parser.add_argument('--allow-dirty', action='store_true', help='run an uncommitted adapter tree')
    parser.add_argument('--force', action='store_true', help='move a conflicting run directory to <dir>.stale-N')
    args = parser.parse_args(argv)
    overrides = {'calibration': str(args.calibration.resolve())} if args.calibration else None
    cfg, base = fsconfig.load(args.config, overrides)
    if args.generate:
        ScenarioSource(**cfg['scenarios'], base=base).generate(args.seeds)
    where = adapter_source(cfg, base)
    adapter_git = identity.git_state(where) if where else None
    if adapter_git and adapter_git['dirty'] and not args.allow_dirty:
        print(f"fastsim.batch: {adapter_git['source']} has uncommitted changes; commit, or pass "
              '--allow-dirty', file=sys.stderr)
        return 2
    provenance = {'git': {'core': identity.git_state(Path(__file__).parent), 'adapter': adapter_git},
                  'config_file': str(args.config.resolve())}
    started = time.perf_counter()
    tracker = _progress(True, args.out, args.variant, len(args.seeds))
    try:
        results = run_batch(cfg, base, args.seeds, args.out, args.variant, args.workers, provenance,
                            force=args.force, progress=tracker or False)
    except RunDirConflict as error:
        print(f'fastsim.batch: {error}', file=sys.stderr)
        return 2
    wall = time.perf_counter() - started
    for r in results:
        print(f'{r.out}: {r.outcome} in {r.sim_s:.0f} s sim ({r.wall_s:.2f} s wall)')
    reused = tracker.skipped if tracker else 0
    ran = len(results) - reused
    print(f'{ran} runs, {wall:.1f} s wall: {ran / wall:.1f} runs/s.{f" {reused} reused." if reused else ""} '
          f'{cfg["fidelity"]}')
    return 0


if __name__ == '__main__':
    sys.exit(main())
