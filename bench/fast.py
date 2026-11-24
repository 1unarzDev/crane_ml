"""Fast-tier baseline profiler: what one domain tier costs on the scalar fastsim path.

    python3 -m bench.fast --domain boat --tier T1 --seeds 0:50 --workers 1,8 --out DIR

Measures, for config/domains/<domain>.yaml's tier:
  plant      the plant alone, open loop: env-steps/s (one env-step = one core step of dt)
  episodes   fastsim.batch.run_batch per worker count: episodes/s in total and per core
  split      per-episode time in plant, tracker/navigator, decision tick, perception, contact
             geometry, recording and other, from timers patched in here only (the core is not
             modified); their own cost shows as the gap to the plain W=1 run
  cluster    crane_backends.fastsim.FastsimBackend: a run(seed) + metrics loop in one process,
             and one fresh process per job (prepare + run + metrics, as an isolated worker job)

Writes DIR/bench.json and DIR/bench.md. Run it through ``bin/heavy sim``.
"""

import argparse
from contextlib import contextmanager
import datetime
import json
import math
import os
from pathlib import Path
import platform
import subprocess
import sys
import time

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

import yaml  # noqa: E402

from fastsim import config as fsconfig, controller as fsctl, perception as fsperc, run as fsrun  # noqa: E402
from fastsim.batch import adapter_of, parse_seeds, run_batch  # noqa: E402
from fastsim.plant import Limits, vehicle  # noqa: E402
from fastsim.scenario import ScenarioSource  # noqa: E402

PARTS = ('plant', 'tracker', 'decision', 'perception', 'contacts', 'recording', 'other')


def domain_tier(domain, tier=None):
    """(tier, fastsim config path, overrides) of a tier in config/domains/<domain>.yaml; no tier = the first.

    Raises LookupError, naming what exists, when the domain or the tier is not there."""
    from crane import paths
    path = paths.DOMAINS / f'{domain}.yaml'
    if not path.is_file():
        known = sorted(p.stem for p in paths.DOMAINS.glob('*.yaml'))
        raise LookupError(f'no domain {domain!r} ({path} is missing); domain configs: '
                          f'{", ".join(known) if known else "none in " + str(paths.DOMAINS)}')
    tiers = (yaml.safe_load(path.read_text()) or {}).get('tiers') or {}
    tier = tier or next(iter(tiers), None)
    if tier not in tiers or not (tiers[tier] or {}).get('config'):
        raise LookupError(f'{path} has no tier {tier}; its tiers: {", ".join(tiers) or "none"}')
    return tier, ROOT / tiers[tier]['config'], tiers[tier].get('overrides') or {}


def load(domain, tier):
    try:
        _, path, overrides = domain_tier(domain, tier)
    except LookupError as error:
        raise SystemExit(f'bench: {error}')
    cfg, base = fsconfig.load(path, overrides)
    return path, overrides, cfg, base


# ---- plant alone -----------------------------------------------------------------------------

def plant_only(cfg, base, seconds=600.0, repeats=3):
    """Open-loop env-steps/s: the plant stepped on a fixed target program (built beforehand)."""
    lim = Limits.from_dict(cfg['limits'])
    dt = float(cfg['dt'])
    n = int(round(seconds / dt))
    v = lim.vmax
    targets = [(0.8 * v[0] * (1 if (k * dt) % 20 < 14 else -0.3), 0.5 * v[1] * math.sin(0.3 * k * dt),
                0.6 * v[2] * math.sin(0.2 * k * dt)) for k in range(n)]
    best = math.inf
    for _ in range(repeats):
        _, plant = vehicle(cfg['vehicle'], lim, base=base)
        t0 = time.perf_counter()
        for target in targets:
            plant.step(target, dt)
        best = min(best, time.perf_counter() - t0)
    return {'steps': n, 'dt': dt, 'wall_s': round(best, 4), 'env_steps_per_s': round(n / best, 1),
            'sim_s_per_wall_s': round(n * dt / best, 1), 'us_per_step': round(1e6 * best / n, 2)}


# ---- episodes ----------------------------------------------------------------------------------

def episodes(cfg, base, seeds, out, workers):
    t0 = time.perf_counter()
    results = run_batch(cfg, base, seeds, out, variant=f'w{workers}', workers=workers)
    wall = time.perf_counter() - t0
    eps = len(results) / wall
    return {'workers': workers, 'episodes': len(results), 'wall_s': round(wall, 3),
            'episodes_per_s': round(eps, 3), 'episodes_per_s_per_core': round(eps / workers, 3),
            'sim_s_mean': round(sum(r.sim_s for r in results) / len(results), 2),
            'succeeded': sum(r.outcome == 'SUCCEEDED' for r in results)}, [r.run_sha256 for r in results]


# ---- where the time goes -------------------------------------------------------------------------

class Clock:
    def __init__(self):
        self.t = dict.fromkeys(PARTS + ('total', 'setup'), 0.0)

    def wrap(self, part, fn):
        t = self.t
        clock = time.perf_counter

        def timed(*a, **k):
            s = clock()
            try:
                return fn(*a, **k)
            finally:
                t[part] += clock() - s
        return timed


@contextmanager
def instrumented(clock, adapter):
    """Timers round the core's calls, patched for the duration (bench only)."""
    saved = {(fsrun, 'vehicle'): fsrun.vehicle, (fsrun, 'sightings'): fsrun.sightings,
             (fsrun, 'footprint_clearance'): fsrun.footprint_clearance,
             (fsctl.Navigator, 'advance'): fsctl.Navigator.advance,
             (fsperc.HoldoverTracker, 'update'): fsperc.HoldoverTracker.update,
             (adapter, 'controller'): adapter.controller}
    mark = {}

    def timed_vehicle(*a, **k):
        tracker, plant = saved[(fsrun, 'vehicle')](*a, **k)
        tracker.command = clock.wrap('tracker', tracker.command)
        plant.step = clock.wrap('plant', plant.step)
        return tracker, plant

    def timed_controller(ctx):
        brain = saved[(adapter, 'controller')](ctx)
        brain.tick = clock.wrap('decision', brain.tick)
        close = brain.close

        def closing():
            mark['closed'] = time.perf_counter()
            return close()
        brain.close = closing
        return brain
    fsrun.vehicle = timed_vehicle
    fsrun.sightings = clock.wrap('perception', saved[(fsrun, 'sightings')])
    fsrun.footprint_clearance = clock.wrap('contacts', saved[(fsrun, 'footprint_clearance')])
    fsctl.Navigator.advance = clock.wrap('tracker', saved[(fsctl.Navigator, 'advance')])
    fsperc.HoldoverTracker.update = clock.wrap('perception', saved[(fsperc.HoldoverTracker, 'update')])
    adapter.controller = timed_controller
    try:
        yield mark
    finally:
        for (owner, name), fn in saved.items():
            if owner is adapter:
                del adapter.controller
            else:
                setattr(owner, name, fn)


def split(cfg, base, seeds, out):
    """Per-episode ms in each part over ``seeds``, in this process, one episode after another."""
    adapter = adapter_of(cfg, base)
    source = ScenarioSource(**cfg['scenarios'], base=base)
    clock = Clock()
    with instrumented(clock, adapter) as mark:
        for seed in seeds:
            t0 = time.perf_counter()
            scenario = source.load(seed)
            t1 = time.perf_counter()
            fsrun.simulate(scenario, adapter, cfg, seed, Path(out) / f's{seed}', base)
            t2 = time.perf_counter()
            clock.t['setup'] += t1 - t0
            clock.t['total'] += t2 - t1
            clock.t['recording'] += t2 - mark['closed']
    t = clock.t
    t['other'] = t['total'] - sum(t[p] for p in PARTS if p != 'other')
    n = len(seeds)
    return {'episodes': n, 'ms_per_episode': {k: round(1e3 * v / n, 3) for k, v in t.items()},
            'share': {p: round(t[p] / t['total'], 4) for p in PARTS}}


# ---- the cluster's per-job path ------------------------------------------------------------------

CHILD = '''
import json, sys, time
t0 = time.perf_counter()
sys.path.insert(0, sys.argv[1])
from crane_backends.fastsim import FastsimBackend
job = json.loads(sys.argv[2])
t1 = time.perf_counter()
backend = FastsimBackend().prepare(job)
t2 = time.perf_counter()
run_dir = backend.run(int(sys.argv[3]))
t3 = time.perf_counter()
backend.metrics(run_dir)
t4 = time.perf_counter()
print(json.dumps({'import_s': t1 - t0, 'prepare_s': t2 - t1, 'run_s': t3 - t2, 'metrics_s': t4 - t3}))
'''


def cluster(path, overrides, seeds, out, fresh_jobs):
    from crane_backends.fastsim import FastsimBackend
    job = {'config': str(path), 'workdir': str(ROOT), 'out': str(Path(out) / 'loop'), 'overrides': overrides}
    t0 = time.perf_counter()
    backend = FastsimBackend().prepare(job)
    t1 = time.perf_counter()
    for seed in seeds:
        backend.metrics(backend.run(seed))
    loop = (time.perf_counter() - t1) / len(seeds)
    fresh = []
    for seed in seeds[:fresh_jobs]:
        j = {**job, 'out': str(Path(out) / f'fresh-s{seed}')}
        s = time.perf_counter()
        done = subprocess.run([sys.executable, '-c', CHILD, str(ROOT), json.dumps(j), str(seed)],
                              capture_output=True, text=True, check=True)
        parts = json.loads(done.stdout.strip().splitlines()[-1])
        fresh.append({**parts, 'wall_s': time.perf_counter() - s})
    mean = {k: round(sum(f[k] for f in fresh) / len(fresh), 4) for k in fresh[0]} if fresh else {}
    return {'prepare_s': round(t1 - t0, 4), 'loop_s_per_episode': round(loop, 4), 'loop_episodes': len(seeds),
            'fresh_process_per_job': mean, 'fresh_jobs': len(fresh)}


# ---- report ------------------------------------------------------------------------------------

def host():
    def out(*cmd):
        try:
            return subprocess.run(cmd, capture_output=True, text=True, check=False).stdout.strip()
        except OSError:
            return ''
    gpu = out('nvidia-smi', '--query-gpu=name', '--format=csv,noheader')
    commit = out('git', '-C', str(ROOT), 'rev-parse', 'HEAD')
    dirty = bool(out('git', '-C', str(ROOT), 'status', '--porcelain', '--untracked-files=no'))
    return {'date': datetime.datetime.now().astimezone().isoformat(timespec='seconds'),
            'host': platform.node(), 'cores': os.cpu_count(), 'gpu': gpu or None,
            'cpu': next((line.split(':', 1)[1].strip() for line in Path('/proc/cpuinfo').read_text().splitlines()
                         if line.startswith('model name')), platform.processor()) if Path('/proc/cpuinfo').exists() else platform.processor(),
            'python': platform.python_version(), 'executable': sys.executable, 'numpy': np.__version__,
            'commit': commit, 'dirty': dirty, 'load_1m': round(os.getloadavg()[0], 2)}


def markdown(r):
    lines = [f"# bench.fast: {r['domain']} {r['tier']}", '',
             f"{r['host']['date']}, {r['host']['cpu']} ({r['host']['cores']} cores), {r['host']['gpu']}, "
             f"python {r['host']['python']}, numpy {r['host']['numpy']}, commit {r['host']['commit'][:12]}"
             f"{' (dirty)' if r['host']['dirty'] else ''}, load {r['host']['load_1m']}", '',
             f"config `{r['config']}`, dt {r['plant']['dt']} s, seeds {r['seeds']}", '',
             '| workers | episodes/s | episodes/s per core | sim s per episode | succeeded |',
             '|---:|---:|---:|---:|---:|']
    for e in r['episodes']:
        lines.append(f"| {e['workers']} | {e['episodes_per_s']} | {e['episodes_per_s_per_core']} | {e['sim_s_mean']} | "
                     f"{e['succeeded']}/{e['episodes']} |")
    p = r['plant']
    lines += ['', f"Plant alone: {p['env_steps_per_s']:,} env-steps/s ({p['us_per_step']} us/step, "
              f"{p['sim_s_per_wall_s']} sim s per wall s).", '',
              f"Same run_sha256 at every worker count: {r['batch_invariant']}.", '',
              '| part | ms per episode | share |', '|---|---:|---:|']
    s = r['split']
    for part in PARTS:
        lines.append(f"| {part} | {s['ms_per_episode'][part]} | {100 * s['share'][part]:.1f}% |")
    lines.append(f"| total (instrumented) | {s['ms_per_episode']['total']} | |")
    lines.append(f"| scenario load (not in total) | {s['ms_per_episode']['setup']} | |")
    c = r['cluster']
    f = c['fresh_process_per_job']
    lines += ['', f"Cluster path: in-process run(seed)+metrics {1e3 * c['loop_s_per_episode']:.1f} ms/episode "
              f"(run_batch W=1: {1e3 * r['episodes'][0]['wall_s'] / r['episodes'][0]['episodes']:.1f} ms)."]
    if f:
        lines.append(f"A fresh process per job: {1e3 * f['wall_s']:.0f} ms/job (imports {1e3 * f['import_s']:.0f}, "
                     f"prepare {1e3 * f['prepare_s']:.0f}, run {1e3 * f['run_s']:.0f}, metrics {1e3 * f['metrics_s']:.0f} ms).")
    return '\n'.join(lines) + '\n'


def main(argv=None):
    ap = argparse.ArgumentParser(prog='bench.fast', description=__doc__.split('\n')[0])
    ap.add_argument('--domain', required=True)
    ap.add_argument('--tier', required=True, choices=('T0', 'T1'))
    ap.add_argument('--seeds', required=True, type=parse_seeds, help='N, FROM:TO or a,b,c')
    ap.add_argument('--workers', default='1', help='comma list of worker counts (default 1)')
    ap.add_argument('--out', required=True, type=Path)
    ap.add_argument('--split-seeds', type=int, default=10, help='episodes timed part by part (default 10)')
    ap.add_argument('--fresh-jobs', type=int, default=5, help='cluster jobs run as fresh processes (default 5)')
    a = ap.parse_args(argv)
    path, overrides, cfg, base = load(a.domain, a.tier)
    if a.out.exists() and any(a.out.iterdir()):
        raise SystemExit(f'bench: {a.out} is not empty')
    a.out.mkdir(parents=True, exist_ok=True)
    seeds = list(a.seeds)
    ScenarioSource(**cfg['scenarios'], base=base).generate(seeds)
    cfg = {**cfg, 'max_workers': max(cfg['max_workers'], max(int(w) for w in a.workers.split(',')))}
    report = {'schema': 'crane.bench.fast/1', 'domain': a.domain, 'tier': a.tier,
              'config': str(path.relative_to(ROOT)), 'overrides': overrides,
              'seeds': f'{seeds[0]}:{seeds[-1] + 1}' if seeds == list(range(seeds[0], seeds[-1] + 1)) else seeds,
              'host': host()}
    report['plant'] = plant_only(cfg, base)
    report['episodes'], hashes = [], []
    for w in (int(w) for w in a.workers.split(',')):
        row, sha = episodes(cfg, base, seeds, a.out / 'runs', w)
        report['episodes'].append(row)
        hashes.append(sha)
        print(f"{a.domain} {a.tier} W={w}: {row['episodes_per_s']} episodes/s", file=sys.stderr)
    report['batch_invariant'] = all(h == hashes[0] for h in hashes)
    report['split'] = split(cfg, base, seeds[:a.split_seeds], a.out / 'split')
    report['cluster'] = cluster(path, overrides, seeds[:a.split_seeds], a.out / 'cluster', a.fresh_jobs)
    (a.out / 'bench.json').write_text(json.dumps(report, indent=2) + '\n')
    (a.out / 'bench.md').write_text(markdown(report))
    print(markdown(report))
    return 0


if __name__ == '__main__':
    sys.exit(main())
