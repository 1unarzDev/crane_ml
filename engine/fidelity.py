"""Gate 4: the fidelity loop scores the same against the batched T1 as against the scalar T1.

    python3 -m engine.fidelity --seeds 0:50

``fastsim/fidelity.py`` scores a fast vehicle (here the boat's T0 first-order spec) on a twin's goal
segments: each goal replayed from the twin's state, RMS position error against the twin's track. The
twin is the boat at T1, once from the scalar path and once from the batched engine, on the same seeds.
Per seed the segments' mean RMS is one unit; ``judge.stats.bootstrap_ci`` gives the CI over seeds.
T2 (Unity) pairs live in ~/.cache/crane-fastsim-pairs when they exist; this host has none, so the
T2 comparison needs the pairs regenerated (``crane run --tier T2``).
"""

import argparse
import json
import math
from pathlib import Path
import sys
import tempfile

from fastsim import fidelity
from judge import stats

PAIRS = Path('~/.cache/crane-fastsim-pairs').expanduser()


def segments(track, events, start, dt):
    """fidelity.windows of a twin run: track [(t, x, y, yaw)] in the world frame, velocities by
    forward difference in the body frame; goals from the run's 'goal' events (map frame -> world)."""
    x0, y0, a0 = start
    c0, s0 = math.cos(a0), math.sin(a0)
    rows = []
    for (t, x, y, yaw), (t1, x1, y1, yaw1) in zip(track, track[1:]):
        h = t1 - t
        vxw, vyw = (x1 - x) / h, (y1 - y) / h
        c, s = math.cos(yaw), math.sin(yaw)
        wz = math.atan2(math.sin(yaw1 - yaw), math.cos(yaw1 - yaw)) / h
        rows.append([t, x, y, yaw, c * vxw + s * vyw, -s * vxw + c * vyw, wz])
    goals = [{'t': e['t'], 'x': x0 + c0 * e['x'] - s0 * e['y'], 'y': y0 + s0 * e['x'] + c0 * e['y'],
              'yaw': e['yaw'] + a0, 'precise': e.get('why') == 'slip', 'task': e.get('why')}
             for e in events if e.get('event') == 'goal']
    return fidelity.windows(rows, goals, [])


def per_seed(tracks, events, starts, limits, spec, dt):
    out = {}
    for seed in tracks:
        sc = fidelity.score(segments(tracks[seed], events[seed], starts[seed], dt), limits, spec)
        out[seed] = sc['rms_mean'] if sc['n'] else float('nan')
    return out


def main(argv=None):
    from domains.boat.plant import ROOT
    from engine import equivalence as EQ
    from engine.episode import Setup, run
    from fastsim import config
    from fastsim.batch import adapter_of, parse_seeds
    from fastsim.plant import Limits
    ap = argparse.ArgumentParser(prog='engine.fidelity', description=__doc__.split('\n')[0])
    ap.add_argument('--seeds', default='0:50', type=parse_seeds)
    ap.add_argument('--backend', default='numba')
    ap.add_argument('--dtype', default='float64')
    ap.add_argument('--out', type=Path)
    a = ap.parse_args(argv)
    seeds = list(a.seeds)
    t1, base = config.load(ROOT / 'domains' / 'boat' / 'fastsim_t1.yaml')
    t0, _ = config.load(ROOT / 'domains' / 'boat' / 'fastsim_t0.yaml')
    spec, limits = t0['vehicle'], Limits.from_dict(t0['limits'])
    adapter = adapter_of(t1, base)
    ref = EQ.scalar(t1, base, seeds, tempfile.mkdtemp(prefix='engine-fid-'), workers=16, track=True)
    setup = Setup(t1, base, adapter, seeds)
    out = run(setup, a.backend, a.dtype, record=1)
    got = EQ.records(out, track=True)
    starts = {s: tuple(float(v) for v in setup.W.start[e]) for e, s in enumerate(seeds)}
    sides = {}
    for name, recs in (('scalar_T1', ref), (f'batched_T1_{a.backend}_{a.dtype}', got)):
        rms = per_seed({s: r['track'] for s, r in recs.items()}, {s: r['events'] for s, r in recs.items()},
                       starts, limits, spec, setup.dt)
        units = [rms[s] for s in seeds if not math.isnan(rms[s])]
        est, ci = stats.bootstrap_ci(units, lambda u: sum(u) / len(u), B=2000, seed=0)
        sides[name] = {'rms_mean': est, 'ci95': ci, 'seeds': len(units), 'per_seed': rms}
    a_, b_ = (sides[k]['per_seed'] for k in sides)
    diffs = [b_[s] - a_[s] for s in seeds if not (math.isnan(a_[s]) or math.isnan(b_[s]))]
    est, ci = stats.bootstrap_ci(diffs, lambda u: sum(u) / len(u), B=2000, seed=0)
    report = {'schema': 'crane.engine.fidelity/1', 'seeds': [seeds[0], seeds[-1] + 1],
              'vehicle': 'boat T0 (fastsim_t0.yaml) replayed on the T1 twin goal segments',
              'sides': {k: {kk: vv for kk, vv in v.items() if kk != 'per_seed'} for k, v in sides.items()},
              'paired_difference_m': {'mean': est, 'ci95': ci, 'identical_seeds': sum(d == 0 for d in diffs)},
              't2_pairs': sorted(str(p) for p in PAIRS.glob('*')) if PAIRS.is_dir() else
              f'none at {PAIRS}: regenerate T2 pairs with `crane run --tier T2` (Unity) to compare against T2'}
    text = json.dumps(report, indent=2, default=float)
    if a.out:
        a.out.write_text(text + '\n')
    print(text)
    return 0


if __name__ == '__main__':
    sys.exit(main())
