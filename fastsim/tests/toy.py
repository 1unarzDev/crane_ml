"""A toy domain for the core's tests: visit every marker it perceives, searching by turning.

Scenarios are the core's minimal json format, drawn from the seed (``write_scenario``).
"""

import json
import math
from pathlib import Path

import numpy as np

from fastsim.controller import Controller, DomainAdapter

HERE = Path(__file__).parent


def write_scenario(folder, seed, markers=4):
    rng = np.random.default_rng(seed)
    start = (float(rng.uniform(-50, 50)), float(rng.uniform(-50, 50)), float(rng.uniform(-math.pi, math.pi)))
    c, s = math.cos(start[2]), math.sin(start[2])
    elements = []
    for k in range(markers):  # ahead of the start pose, spread so none hides another for long
        fx, fy = float(rng.uniform(5, 25)), float(rng.uniform(-10, 10))
        elements.append({'id': f'm{k}', 'x': round(start[0] + c * fx - s * fy, 3),
                         'y': round(start[1] + s * fx + c * fy, 3), 'radius': 0.2, 'label': 'marker'})
    elements.append({'id': 'wall', 'x': start[0] - 30 * c, 'y': start[1] - 30 * s, 'radius': 0.0,
                     'length': 4.0, 'width': 1.0, 'yaw': start[2]})  # an unlabelled box, far behind
    path = Path(folder) / f'toy-s{seed}.json'
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps({'name': f'toy-s{seed}', 'start': list(start), 'elements': elements,
                                'identity': {'seed': seed, 'pair_key': f'toy-{seed}'}}))
    return path


class Visitor(Controller):
    def __init__(self, ctx, markers, reach_m):
        self.ctx, self.markers, self.reach = ctx, markers, reach_m
        self.visited = []  # (x, y) of visited markers

    def tick(self, obs):
        nav, (x, y, yaw) = self.ctx.navigator, obs.pose
        fresh = [t for t in obs.tracks if t.label == 'marker'
                 and all(math.dist((t.x, t.y), v) > 0.5 for v in self.visited)]
        for t in fresh:
            if math.dist((t.x, t.y), (x, y)) <= self.reach:
                self.visited.append((t.x, t.y))
                self.ctx.emit('visited', n=len(self.visited))
        if len(self.visited) >= self.markers:
            nav.hold()
            return 'SUCCEEDED'
        if nav.goal is not None and not nav.goal.done:
            return None
        fresh = [t for t in fresh if all(math.dist((t.x, t.y), v) > 0.5 for v in self.visited)]
        if fresh:
            t = min(fresh, key=lambda t: math.dist((t.x, t.y), (x, y)))
            heading = math.atan2(t.y - y, t.x - x)
            back = 0.6
            nav.send([(t.x - back * math.cos(heading), t.y - back * math.sin(heading), heading)],
                     why='visit')
        else:
            nav.send([(x, y, yaw + 1.0)], why='search')
        return None


class ToyAdapter(DomainAdapter):
    def identity(self):
        return {'domain': 'toy', 'options': self.options}

    def controller(self, ctx):
        markers = sum(bool(e.label) for e in ctx.scenario.elements)
        return Visitor(ctx, markers, float(self.options.get('reach_m', 1.5)))

    def time_limit(self, ctx):
        return self.options.get('time_limit_s')

    def finalize(self, run_dir, result):
        (Path(run_dir) / 'toy.json').write_text(json.dumps({'outcome': result['outcome'],
                                                             'visits': len(result['events'])}))

    def metrics(self, run_dir):
        run = json.loads((Path(run_dir) / 'run.json').read_text())
        return {'run': {'duration_s': run['sim_s'], 'contacts': len(run['contacts'])}}
