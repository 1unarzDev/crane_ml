"""Agreement between the engine and the scalar path, and between engine backends (gates 2 and 3).

    python3 -m engine.equivalence --tier T1 --seeds 0:500 --candidates numba,numba-fastmath,jax-f64,jax-f32

    ref = scalar(cfg, base, seeds, root)            # fastsim.run.simulate per seed: comparable records
    got = records(run(setup, 'numba'))              # the engine's records, same shape
    exact(ref, got)                                 # per seed: the fields that differ (gate 2)
    paired(ref, got)                                # outcome agreement and TOST (gate 3)

A record is one seed's outcome, sim_s, flags, contacts, idle, min_clearance_m, events, boat.json
numbers and (optionally) its track. Gate 3: per-seed outcome agreement >= 0.98, and TOST at 90%
(two one-sided 5% tests) on time-to-finish (relative, +-2%) and dock error (+-0.05 m) over the
seeds both sides docked.
"""

import argparse
import json
import math
from pathlib import Path
import sys
import tempfile
import time

from judge import stats

FIELDS = ('outcome', 'sim_s', 'flags', 'contacts', 'idle', 'min_clearance_m')
AGREEMENT = 0.98
TIME_MARGIN = 0.02     # relative
DOCK_MARGIN_M = 0.05
CANDIDATES = {'numpy': ('numpy', 'float64', False), 'numba': ('numba', 'float64', False),
              'numba-fastmath': ('numba', 'float64', True), 'jax-f64': ('jax', 'float64', False),
              'jax-f32': ('jax', 'float32', False)}


def _read(run_dir, track):
    d = Path(run_dir)
    run = json.loads((d / 'run.json').read_text())
    rec = {k: run[k] for k in FIELDS}
    rec['events'] = [json.loads(s) for s in (d / 'sim_events.jsonl').read_text().splitlines()]
    rec['boat'] = json.loads((d / 'boat.json').read_text())
    if track:
        rec['track'] = [(r['t'], r['x'], r['y'], r['yaw']) for r in
                        (json.loads(s) for s in (d / 'track.jsonl').read_text().splitlines())]
    return rec


def scalar(cfg, base, seeds, root=None, workers=8, track=False):
    """{seed: record} of the scalar fastsim path (fastsim.batch.run_batch)."""
    from fastsim.batch import run_batch
    root = Path(root or tempfile.mkdtemp(prefix='engine-scalar-'))
    results = run_batch(cfg, base, seeds, root, variant='scalar', workers=workers)
    return {r.seed: _read(r.out, track) for r in results}


def records(out, track=False):
    """{seed: record} of an engine Output."""
    recs = {}
    for e, seed in enumerate(out.setup.seeds):
        s = out.summary(e)
        rec = {k: s[k] for k in FIELDS}
        rec.update(events=out.events(e), boat=out.numbers(e))
        if track:
            rec['track'] = out.track(e)
        recs[int(seed)] = rec
    return recs


def exact(ref, got):
    """{seed: [differing fields]} (empty: run for run equal)."""
    bad = {}
    for seed, a in ref.items():
        b = got[seed]
        diff = [k for k in a if a[k] != b.get(k)]
        if diff:
            bad[seed] = diff
    return bad


def tost(diffs, margin, alpha=0.05):
    """(mean, 90% CI, equivalent): two one-sided tests at ``alpha`` against +-margin."""
    m, ci, _, n = stats.paired_t(diffs, alpha=2 * alpha)
    return {'mean': m, 'ci90': ci, 'n': n, 'margin': margin, 'equivalent': bool(n >= 2 and -margin < ci[0] and ci[1] < margin)}


def paired(ref, got):
    """Gate 3 numbers of ``got`` against ``ref`` over the same seeds."""
    seeds = sorted(ref)
    same = [ref[s]['outcome'] == got[s]['outcome'] and ref[s]['boat']['success'] == got[s]['boat']['success']
            for s in seeds]
    both = [s for s in seeds if ref[s]['boat']['success'] and got[s]['boat']['success']]
    time_rel = [(got[s]['boat']['time_s'] - ref[s]['boat']['time_s']) / ref[s]['boat']['time_s'] for s in both]
    dock = [got[s]['boat']['dock_error_m'] - ref[s]['boat']['dock_error_m'] for s in both]
    agree = sum(same) / len(seeds)
    t, d = tost(time_rel, TIME_MARGIN), tost(dock, DOCK_MARGIN_M)
    exactly = sum(not exact({s: ref[s]}, {s: got[s]}) for s in seeds)
    return {'seeds': len(seeds), 'outcome_agreement': agree, 'agreement_ok': agree >= AGREEMENT,
            'exact_records': exactly, 'both_docked': len(both), 'time_to_finish_rel': t, 'dock_error_m': d,
            'disagree': [s for s, ok in zip(seeds, same) if not ok][:20],
            'pass': agree >= AGREEMENT and t['equivalent'] and d['equivalent']}


def main(argv=None):
    from engine.episode import Setup, run
    from fastsim import config
    from fastsim.batch import adapter_of, parse_seeds
    from domains.boat.plant import ROOT
    ap = argparse.ArgumentParser(prog='engine.equivalence', description=__doc__.split('\n')[0])
    ap.add_argument('--tier', default='T1', choices=('T0', 'T1'))
    ap.add_argument('--seeds', default='0:500', type=parse_seeds)
    ap.add_argument('--candidates', default='numba,numba-fastmath,jax-f64,jax-f32')
    ap.add_argument('--reference', default='numpy', help='numpy (the float64 batch) or scalar (fastsim)')
    ap.add_argument('--workers', type=int, default=16, help='scalar reference workers')
    ap.add_argument('--track', action='store_true', help='also compare every track row (records them)')
    ap.add_argument('--out', type=Path, help='write the report JSON here')
    a = ap.parse_args(argv)
    cfg, base = config.load(ROOT / 'domains' / 'boat' / f'fastsim_{a.tier.lower()}.yaml')
    adapter = adapter_of(cfg, base)
    seeds = list(a.seeds)
    setup = Setup(cfg, base, adapter, seeds)
    report = {'schema': 'crane.engine.equivalence/1', 'tier': a.tier, 'seeds': [seeds[0], seeds[-1] + 1],
              'reference': a.reference, 'margins': {'agreement': AGREEMENT, 'time_rel': TIME_MARGIN,
                                                    'dock_m': DOCK_MARGIN_M}, 'candidates': {}}
    t0 = time.perf_counter()
    if a.reference == 'scalar':
        ref = scalar(cfg, base, seeds, workers=a.workers, track=a.track)
    else:
        ref = records(run(setup, 'numpy', record=1 if a.track else 0), a.track)
    report['reference_wall_s'] = round(time.perf_counter() - t0, 2)
    for name in a.candidates.split(','):
        backend, dtype, fastmath = CANDIDATES[name]
        out = run(setup, backend, dtype, record=1 if a.track else 0, fastmath=fastmath)
        got = records(out, a.track)
        r = paired(ref, got)
        r['exact_differences'] = {str(k): v for k, v in list(exact(ref, got).items())[:20]}
        r['wall_s'] = round(out.wall_s, 2)
        report['candidates'][name] = r
        print(f"{a.tier} {name}: agreement {r['outcome_agreement']:.4f}, exact {r['exact_records']}/{r['seeds']}, "
              f"time {r['time_to_finish_rel']['ci90']}, dock {r['dock_error_m']['ci90']} -> "
              f"{'PASS' if r['pass'] else 'FAIL'}", file=sys.stderr)
    if a.out:
        a.out.write_text(json.dumps(report, indent=2, default=float) + '\n')
    print(json.dumps(report, indent=2, default=float))
    return 0 if all(r['pass'] for r in report['candidates'].values()) else 1


if __name__ == '__main__':
    sys.exit(main())


__all__ = ['scalar', 'records', 'exact', 'paired', 'tost', 'math']
