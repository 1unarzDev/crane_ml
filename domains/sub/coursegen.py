"""Write pool-gate courses for seeds: a generic rectangular pool and a few rectangular gates at depth.

    python3 domains/sub/coursegen.py FROM:TO DIR      # seeds FROM..TO-1; existing files are kept

NED with the origin at a pool corner: x along the pool (north), y across it (east), z down from the surface.
Each course is fastsim's ``json`` scenario (gate posts as elements, the pool outline as boundary, the planar
start) plus ``pool``, ``start_depth_m``, ``gates`` (centre, heading of the passage direction, opening) and
``finish`` (where the vehicle holds at the end). The numbers come from the seed alone.
"""

import json
import math
from pathlib import Path
import sys

import numpy as np

GENERATOR = 'pool_gates/1'
POOL = {'length_m': 25.0, 'width_m': 15.0, 'depth_m': 4.0}
GATES = 3


def course(seed):
    from fastsim.identity import derive_seed, digest
    g = np.random.default_rng(derive_seed(int(seed), 'coursegen', GENERATOR))
    L, W = POOL['length_m'], POOL['width_m']
    start = (2.0, float(g.uniform(5.0, W - 5.0)), 0.0)
    gates, prev = [], start[:2]
    for k in range(GATES):
        x = 6.0 + 5.0 * k + float(g.uniform(-0.5, 0.5))
        y = float(g.uniform(3.5, W - 3.5))
        bearing = math.atan2(y - prev[1], x - prev[0])
        yaw = max(-0.6, min(0.6, bearing + float(g.uniform(-0.25, 0.25))))
        gates.append({'id': f'gate{k + 1}', 'x': round(x, 3), 'y': round(y, 3),
                      'z': round(float(g.uniform(1.0, 2.6)), 3), 'yaw': round(yaw, 4),
                      'width_m': round(float(g.uniform(1.6, 2.2)), 3), 'height_m': 1.2})
        prev = (x, y)
    finish = {'x': round(min(L - 2.5, prev[0] + 4.0), 3), 'y': round(float(g.uniform(4.0, W - 4.0)), 3),
              'z': 1.0, 'yaw': 0.0}
    posts = []
    for gt in gates:
        c, s = math.cos(gt['yaw']), math.sin(gt['yaw'])
        half = gt['width_m'] / 2
        for side, sign in (('left', -1.0), ('right', 1.0)):
            posts.append({'id': f"{gt['id']}_{side}", 'x': round(gt['x'] - sign * half * s, 3),
                          'y': round(gt['y'] + sign * half * c, 3), 'radius': 0.05, 'label': 'gate_post'})
    body = {'name': f'pool_gates-s{seed}', 'start': list(start), 'start_depth_m': 0.5, 'pool': POOL,
            'elements': posts, 'boundary': [[0.0, 0.0], [L, 0.0], [L, W], [0.0, W]], 'gates': gates,
            'finish': finish}
    body['identity'] = {'generator': GENERATOR, 'seed': int(seed), 'pair_key': f'{GENERATOR}/s{seed}',
                        'truth_sha256': digest(body)}
    return body


def write(seeds, folder):
    folder = Path(folder)
    folder.mkdir(parents=True, exist_ok=True)
    for s in seeds:
        path = folder / f'pool_gates-s{s}.json'
        if not path.exists():
            tmp = path.with_name(path.name + '.tmp')
            tmp.write_text(json.dumps(course(s), indent=1) + '\n')
            tmp.replace(path)


if __name__ == '__main__':
    sys.path.insert(0, str(Path(__file__).resolve().parents[2]))
    lo, _, hi = sys.argv[1].partition(':')
    write(range(int(lo), int(hi) if hi else int(lo) + 1), sys.argv[2])
