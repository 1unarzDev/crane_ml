# SPDX-License-Identifier: GPL-3.0-or-later
"""Fly-to-waypoint scenarios from the seed: python3 domains/drones/waypoints.py A:B DIR [--min 20 --max 60].

A start pose anywhere in a 200 m square, and one waypoint ahead of it (distance uniform in [min, max] m,
bearing within 0.6 rad of the start heading), as fastsim's json format (no elements: open air).
"""

import argparse
import json
import math
from pathlib import Path

import numpy as np


def scenario(seed, near=20.0, far=60.0):
    rng = np.random.default_rng([int(seed), 0xD0])
    x, y, yaw = (float(v) for v in (rng.uniform(-100, 100), rng.uniform(-100, 100), rng.uniform(-math.pi, math.pi)))
    d, b = float(rng.uniform(near, far)), yaw + float(rng.uniform(-0.6, 0.6))
    wp = {'x': round(x + d * math.cos(b), 3), 'y': round(y + d * math.sin(b), 3), 'yaw': round(b, 5)}
    return {'name': f'drones-s{seed}', 'start': [round(x, 3), round(y, 3), round(yaw, 5)], 'elements': [],
            'waypoint': wp, 'identity': {'seed': int(seed), 'pair_key': f'drones-waypoint-{seed}'}}


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__.split('\n')[0])
    ap.add_argument('seeds', help='A:B (B exclusive)')
    ap.add_argument('dir')
    ap.add_argument('--min', type=float, default=20.0)
    ap.add_argument('--max', type=float, default=60.0)
    a = ap.parse_args(argv)
    lo, hi = (int(v) for v in a.seeds.split(':'))
    out = Path(a.dir)
    out.mkdir(parents=True, exist_ok=True)
    for s in range(lo, hi):
        (out / f'drones-s{s}.json').write_text(json.dumps(scenario(s, a.min, a.max), indent=1) + '\n')


if __name__ == '__main__':
    main()
