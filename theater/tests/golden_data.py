"""The cross-language golden run: the Python writer makes golden/small.ctraj from this, and the
Unity EditMode test (Assets/Tests/Performance/CraneTrajTests.cs) decodes it and checks the
values listed in golden/small.json. Regenerate both with ``python3 -m theater.tests.golden_data``."""

import json
import math
from pathlib import Path

HERE = Path(__file__).parent / 'golden'


def data():
    hz, n = 20.0, 700   # 35 s: two blocks
    cols = {'boat.x': [], 'boat.y': [], 'boat.z': [], 'boat.yaw': [], 'boat.pitch': [], 'boat.roll': [],
            'boat.speed': []}
    for i in range(n):
        t = i / hz
        cols['boat.x'].append(round(-12.5 + 1.5 * t, 2))
        cols['boat.y'].append(round(40.0 + 8.0 * math.sin(t / 5.0), 2))
        cols['boat.z'].append(round(0.05 * math.sin(t * 2.0), 2))
        cols['boat.yaw'].append(round(math.atan2(math.cos(t / 5.0) * 1.6, 1.5), 4))
        cols['boat.pitch'].append(round(0.02 * math.sin(t), 4))
        cols['boat.roll'].append(round(-0.03 * math.cos(t), 4))
        cols['boat.speed'].append(round(1.5 + 0.1 * math.sin(t), 3))
    header = {'seed': 7, 'pose_dims': '6dof', 'robot': {'model': 'golden', 'agents': ['boat']},
              'frame': {'type': 'enu', 'origin': 'world'}, 'water': {'mode': 'still'},
              'build': {'recorder': 'golden'}}
    events = [{'t': 3.25, 'kind': 'gate', 'task': 'entry', 'index': 0},
              {'t': 31.0, 'kind': 'contact', 'id': 'buoy-3', 'cause': 'bump'}]
    return header, cols, events, hz


def main():
    from theater import traj
    header, cols, events, hz = data()
    sha = traj.write(HERE / 'small.ctraj', header, cols, events, hz=hz)
    picks = [0, 1, 599, 600, 699]
    expect = {'sha256': sha, 'ticks': 700, 'hz': hz, 'blocks': 2, 'columns': list(cols),
              'samples': {str(i): {k: v[i] for k, v in cols.items()} for i in picks},
              'events': events, 'header': {'seed': 7, 'pose_dims': '6dof'}}
    (HERE / 'small.json').write_text(json.dumps(expect, indent=1, sort_keys=True) + '\n')


if __name__ == '__main__':
    main()
