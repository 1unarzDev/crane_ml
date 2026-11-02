"""The cross-language golden: a small plane-world manifest and pack that Python writes byte-for-byte the
same every time (test_pack checks it) and the Unity theater reads (Assets/Tests/Theater checks values and
replays it for determinism). Regenerate with ``python3 -m theater.tests.fixture`` after a format change."""

import json
import math
from pathlib import Path

from .. import pack

DIR = Path(__file__).resolve().parents[2] / 'Assets' / 'Tests' / 'Theater' / 'Fixtures'
HZ = 5


def runs():
    """9 runs over 3 iterations along +x (east), wandering in y; iteration k is faster, one failure each early."""
    out, meta = [], []
    for k in range(9):
        it = k // 3
        v = 1.0 + 0.5 * it + 0.1 * (k % 3)
        fail = (k % 3 == 2 and it < 2)
        n = 40 if fail else int(60 / v * HZ / 2) + 10
        samples = []
        for i in range(n):
            t = i / HZ
            x = v * t
            y = 1.5 * math.sin(0.3 * x + k) * (1 - 0.3 * it)
            yaw = math.degrees(math.atan2(1.5 * 0.3 * math.cos(0.3 * x + k) * (1 - 0.3 * it), 1.0))
            samples.append({'x': x, 'y': y, 'z': 0.0, 'yaw': yaw, 'pitch': 0.0, 'roll': 2.0 * math.sin(t),
                            'speed': v, 'progress': min(1.0, x / 30.0)})
        out.append(samples)
        last = samples[-1]
        meta.append({'id': f'g{k}', 'iteration': it, 'outcome': 'failed' if fail else 'finished',
                     'failure_reason': 'buoy_hit' if fail else None, 'score': round(last['progress'] + 0.1 * v, 4),
                     'progress': round(last['progress'], 4), 'valid': True, 'duration_s': (n - 1) / HZ,
                     'params': {'ctrl.speed': v, 'ctrl.k': 0.5 + it},
                     'events': [{'t': round((n - 1) / HZ, 2), 'kind': 'failure' if fail else 'finish',
                                 'x': round(last['x'], 2), 'y': round(last['y'], 2)}]})
    return out, meta


def write(dir=DIR):
    dir.mkdir(parents=True, exist_ok=True)
    tracks, meta = runs()
    pack.write_pack(dir / 'golden.pack', tracks, HZ)
    world = {'plugin': 'plane', 'bounds': [-5, -10, 40, 10], 'centreline': [[0, 0], [30, 0]]}
    pack.write_manifest(dir / 'golden_manifest.json', world=world, robot={'plugin': 'arrow'}, runs=meta,
                        pack='golden.pack', study='golden', iterations=[{'iteration': i, 'insight': f'iteration {i}'} for i in range(3)])
    # what C# must read back (quantized values: write then read)
    _, back = pack.read_pack(dir / 'golden.pack')
    expect = [{'run': r, 'k': k, **{c: round(v, 6) for c, v in back[r][k].items()}}
              for r in (0, 4, 8) for k in (0, 7, len(back[r]) - 1)]
    (dir / 'golden_expect.json').write_text(json.dumps(expect, indent=1, sort_keys=True) + '\n')


if __name__ == '__main__':
    write()
