"""Batch runner: one domain on many seeds, across worker processes.

    python -m fastsim.batch --config DOMAIN.yaml --seeds 1:21 --out OUT [--variant NAME]
                            [--workers N] [--generate] [--calibration FILE] [--allow-dirty]

Each seed's scenario comes from the config's ``scenarios`` source; each run is
``OUT/<variant>/s<seed>`` (run.py). A dirty adapter tree is refused unless ``--allow-dirty``
(then every run records it). Workers are spawned processes, never above ``max_workers``.
"""

import argparse
from dataclasses import dataclass
from multiprocessing import get_context
from pathlib import Path
import sys
import time

from fastsim import config as fsconfig, identity, plugin
from fastsim.run import simulate
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
    """'N' or 'FROM:TO' (TO exclusive) or a comma list."""
    if ',' in text:
        return [int(s) for s in text.split(',')]
    lo, _, hi = text.partition(':')
    return list(range(int(lo), int(hi))) if hi else [int(lo)]


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


_STATE = {}


def _init(cfg, base, provenance):
    # A failing initializer makes a Pool respawn workers forever: keep the error, raise it per job.
    try:
        _STATE.update(cfg=cfg, base=base, provenance=provenance, adapter=adapter_of(cfg, base),
                      source=ScenarioSource(**cfg['scenarios'], base=base))
    except Exception as error:  # noqa: BLE001
        _STATE['error'] = error


def _one(job):
    seed, out = job
    s = _STATE
    if 'error' in s:
        raise RuntimeError(f"worker set-up failed: {type(s['error']).__name__}: {s['error']}") from s['error']
    started = time.perf_counter()
    summary = simulate(s['source'].load(seed), s['adapter'], s['cfg'], seed, out, s['base'],
                       s['provenance'])
    return Result(str(out), seed, summary['outcome'], summary['sim_s'],
                  round(time.perf_counter() - started, 3), summary['identity']['run_sha256'])


def run_batch(cfg, base, seeds, out, variant='base', workers=None, provenance=None):
    """Run every seed; returns [Result] in seed order."""
    jobs = [(int(s), str(Path(out) / variant / f's{s}')) for s in seeds]
    workers = max(1, min(workers or cfg['workers'], cfg['max_workers'], len(jobs)))
    provenance = {'variant': variant, **(provenance or {})}
    if workers <= 1:
        _init(cfg, base, provenance)
        return [_one(job) for job in jobs]
    with get_context('spawn').Pool(workers, _init, (cfg, base, provenance)) as pool:
        return pool.map(_one, jobs, chunksize=1)


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
    results = run_batch(cfg, base, args.seeds, args.out, args.variant, args.workers, provenance)
    wall = time.perf_counter() - started
    for r in results:
        print(f'{r.out}: {r.outcome} in {r.sim_s:.0f} s sim ({r.wall_s:.2f} s wall)')
    print(f'{len(results)} runs, {wall:.1f} s wall: {len(results) / wall:.1f} runs/s. {cfg["fidelity"]}')
    return 0


if __name__ == '__main__':
    sys.exit(main())
