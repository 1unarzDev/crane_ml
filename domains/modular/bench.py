"""Throughput of the modular kernel's replay: E envs, each its own 50 Hz timeline, per backend.

    python3 -m domains.modular.bench --E 256,1024,4096 --backend numba,jax:float32 --seconds 40 --robot mecanum-1
    python3 -m domains.modular.bench ... --json      # one JSON document

The first call of a shape (compile, or cache load) is timed apart. trials/s is envs over the timed call's wall.
"""

import argparse
import json
import os
import time

import numpy as np

from domains.modular import assembly as AS, profile as PR
from engine.kernels import mecanum as K, mecanum_run as R


def batch(E, T, robot, seed=0):
    b = AS.build(robot, PR.load('example#plated'))
    rng = np.random.default_rng(seed)
    p = K.stack([K.pack(b['wheels'], b['body'], {'mode': rng.integers(0, 5, len(b['wheels'])) * (rng.random() < 0.5),
                                                 'gain': rng.uniform(0.3, 0.9, len(b['wheels']))}) for _ in range(E)])
    t = np.arange(T) / 50
    f = rng.uniform(0.3, 2.0, (E, 3, 1))
    tl = np.stack([0.35 * np.sin(f[:, 0] * t), 0.25 * np.sin(f[:, 1] * t + 1), 0.8 * np.sin(f[:, 2] * t + 2)], -1)
    return p, tl


def main(argv=None):
    ap = argparse.ArgumentParser(prog='python3 -m domains.modular.bench')
    ap.add_argument('--E', default='256,1024')
    ap.add_argument('--backend', default='numba')
    ap.add_argument('--seconds', type=float, default=40.0)
    ap.add_argument('--robot', default='mecanum-1')
    ap.add_argument('--json', action='store_true')
    a = ap.parse_args(argv)
    T = int(round(a.seconds * 50))
    rows = []
    for spec in a.backend.split(','):
        be, _, dtype = spec.partition(':')
        for E in map(int, a.E.split(',')):
            p, tl = batch(E, T, a.robot)
            t0 = time.perf_counter()
            R.replay(p, tl[:, :max(2, T // 50)], backend=be, dtype=dtype or 'float64', delay=2)
            first = time.perf_counter() - t0
            t0 = time.perf_counter()
            out = R.replay(p, tl, backend=be, dtype=dtype or 'float64', delay=2)
            wall = time.perf_counter() - t0
            ok = bool(np.isfinite(out['pose']).all())
            rows.append({'backend': be, 'dtype': dtype or 'float64', 'robot': a.robot, 'wheels': K.wheels_of(p),
                         'E': E, 'sim_s': a.seconds, 'first_call_s': round(first, 2), 'wall_s': round(wall, 3),
                         'trials_per_s': round(E / wall, 1), 'plant_steps_per_s': round(E * T * 4 / wall),
                         'finite': ok, 'threads': os.cpu_count()})
            if not a.json:
                r = rows[-1]
                print(f"{be:6s} {r['dtype']:8s} {a.robot:13s} E={E:6d}  first {first:6.2f} s  {wall:7.2f} s  "
                      f"{r['trials_per_s']:8.1f} trials/s  {r['plant_steps_per_s'] / 1e6:6.2f} M steps/s  finite={ok}")
    if a.json:
        print(json.dumps({'schema': 'crane.modular.bench/1', 'rows': rows}, indent=1))


if __name__ == '__main__':
    main()
