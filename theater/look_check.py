"""Regression check for a theater look change: the simulation is untouched and the plain look is unchanged.

    heavy unity python3 -m theater.look_check --base BASE/CRANE.x86_64 --candidate CAND/CRANE.x86_64 \\
        --pack DIR --out OUT [--seed 1000] [--duration 20] [--frames 0:120]

1. Simulation: the same seeded benchmark (Roboboat Course, aquatic-full-sensor, ROS off) runs on the base player twice
   and on the candidate once; body trajectories are compared at equal simulated times (Tools/Performance/
   compare_benchmarks.py) and the candidate must differ from the base by no more than the base differs from itself
   (exactly zero when PhysX repeats, as it does here). Observation acquisition counters and signatures in the result
   JSON must match too.
2. Theater: the plain look renders frames A:B of the pack on both players; every frame's state digest and pixel
   sha256 must match (the plain look is the scene the base renders).

Writes OUT/look_check.json and exits 1 when a check fails. Both players run batchmode and never open a window.
"""

import argparse
import json
import os
import subprocess
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / 'Tools' / 'Performance'))
from compare_benchmarks import compare  # noqa: E402

from .render import player_argv  # noqa: E402


def env():
    e = {k: v for k, v in os.environ.items() if k not in ('DISPLAY', 'WAYLAND_DISPLAY')}
    e['SDL_VIDEODRIVER'] = 'dummy'
    return e


def bench(player, out, seed, duration):
    out.mkdir(parents=True, exist_ok=True)
    result = out / 'bench.json'
    argv = [str(player), '-batchmode', '-screen-fullscreen', '0', '-screen-width', '640', '-screen-height', '360',
            '-logFile', str(out / 'bench.log'), '--crane-worker', '--crane-benchmark', '--crane-disable-ros',
            '--crane-disable-sitl', '--crane-scene', 'Roboboat Course', '--crane-scenario', 'aquatic-full-sensor',
            '--crane-seed', str(seed), '--crane-time-scale', '1', '--crane-warmup', '2', '--crane-duration', str(duration),
            '--crane-output', str(result)]
    subprocess.run(argv, cwd=str(Path(player).parent), env=env(), stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL,
                   timeout=duration * 20 + 300)
    return json.loads(result.read_text())


def theater(player, pack, out, frames):
    out.mkdir(parents=True, exist_ok=True)
    m = json.loads((pack / 'manifest.json').read_text())
    argv = player_argv(Path(player).resolve(), pack.resolve(), m, out.resolve(), frames,
                       ['--crane-theater-ffmpeg', 'none', '--crane-theater-look', 'plain', '--crane-theater-preroll-s', '0'])
    subprocess.run(argv, cwd=str(Path(player).parent), env=env(), stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL, timeout=3600)
    rows = [json.loads(x) for x in (out / 'frames.jsonl').read_text().splitlines() if x.strip()]
    return {r['frame']: (r['state'], r['pixels']) for r in rows}


def observations(doc):
    """The result's observation counters and signatures (whatever the benchmark reports under those names)."""
    keep = {}

    def walk(o, path):
        if isinstance(o, dict):
            for k, v in o.items():
                p = f'{path}.{k}' if path else k
                low = k.lower()
                if any(w in low for w in ('signature', 'acquisition', 'hash')) and not isinstance(v, (dict, list)):
                    keep[p] = v
                elif k != 'validation':
                    walk(v, p)
    walk(doc, '')
    return keep


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--base', required=True); ap.add_argument('--candidate', required=True)
    ap.add_argument('--pack', required=True); ap.add_argument('--out', required=True)
    ap.add_argument('--seed', type=int, default=1000); ap.add_argument('--duration', type=float, default=20)
    ap.add_argument('--frames', default='0:120')
    a = ap.parse_args(argv)
    out = Path(a.out); out.mkdir(parents=True, exist_ok=True)
    b1 = bench(a.base, out / 'sim_base_1', a.seed, a.duration)
    b2 = bench(a.base, out / 'sim_base_2', a.seed, a.duration)
    c = bench(a.candidate, out / 'sim_candidate', a.seed, a.duration)
    floor, diff = compare(b1, b2, 0.1), compare(b1, c, 0.1)
    worst = {k: v['max'] for k, v in diff['metrics'].items()}
    allowed = {k: v['max'] for k, v in floor['metrics'].items()}
    sim_ok = diff['comparedTimes'] > 0 and all(worst[k] <= allowed.get(k, 0.0) + 1e-9 for k in worst)
    ob, oc = observations(b1), observations(c)
    obs_diff = sorted(k for k in set(ob) | set(oc) if ob.get(k) != oc.get(k) and observations(b2).get(k) == ob.get(k))
    tb = theater(a.base, Path(a.pack), out / 'theater_base', a.frames)
    tc = theater(a.candidate, Path(a.pack), out / 'theater_candidate', a.frames)
    state_diff = sorted(f for f in tb if tb[f][0] != tc.get(f, (None,))[0])
    pixel_diff = sorted(f for f in tb if tb[f][1] != tc.get(f, (None, None))[1])
    theater_ok = len(tb) > 0 and not state_diff and not pixel_diff
    doc = {'schema': 'crane-look-check/1', 'ok': sim_ok and not obs_diff and theater_ok,
           'simulation': {'ok': sim_ok, 'seed': a.seed, 'duration_s': a.duration, 'compared_times': diff['comparedTimes'],
                          'compared_bodies': diff['comparedBodies'], 'candidate_vs_base_max': worst, 'base_vs_base_max': allowed},
           'observations': {'ok': not obs_diff, 'compared': len(ob), 'differ': obs_diff},
           'theater_plain': {'ok': theater_ok, 'frames': len(tb), 'state_differs': state_diff[:20], 'pixels_differ': pixel_diff[:20]}}
    (out / 'look_check.json').write_text(json.dumps(doc, indent=1, sort_keys=True) + '\n')
    print(json.dumps(doc, indent=1, sort_keys=True))
    return 0 if doc['ok'] else 1


if __name__ == '__main__':
    sys.exit(main())
