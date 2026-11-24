# SPDX-License-Identifier: GPL-3.0-or-later
"""The drones' public reference task at T1: fly to the scenario's waypoint, stop, hold.

    adapter: {ref: domains.drones.adapter:WaypointAdapter, options: {profile: x500, hold_s: 3.0}}

The controller sends one goal at the waypoint; when the tracker reports it reached (goal_tol), the vehicle is
held still for ``hold_s`` while the controller keeps measuring, then the run SUCCEEDS. A goal the progress
checker aborts FAILS. drones.json (and metrics) per run: arrive_s, overshoot_m (the peak travel past the
waypoint along the approach line), final_error_m (distance to the waypoint at the end of the hold), drift_m
(travel during the hold); and from the plant: alt_dev_m (peak altitude deviation), max_tilt_deg, saturated_s.
"""

import json
import math
from pathlib import Path

from domains.drones import card as cards
from fastsim.controller import Controller, DomainAdapter


class Waypoint(Controller):
    def __init__(self, ctx, hold_s):
        self.ctx, self.hold_s = ctx, float(hold_s)
        wp = ctx.scenario.raw['waypoint']
        (mx, my), = ctx.world.to_map([[wp['x'], wp['y']]])
        self.wp = (float(mx), float(my), float(wp.get('yaw', 0.0)) - ctx.world.start[2])
        n = math.hypot(mx, my) or 1.0
        self.dir = (mx / n, my / n)          # the approach line, from the start (the map origin)
        self.goal = self.arrived = self.held_at = None
        self.over = 0.0
        self.result = {}

    def tick(self, obs):
        nav, (x, y, _) = self.ctx.navigator, obs.pose
        past = (x - self.wp[0]) * self.dir[0] + (y - self.wp[1]) * self.dir[1]
        self.over = max(self.over, past)
        if self.goal is None:
            self.goal = nav.send([self.wp], why='waypoint')
            return None
        if self.arrived is None:
            if not self.goal.done:
                return None
            if self.goal.status != 'SUCCEEDED':
                self.result = {'status': self.goal.status}
                return 'FAILED'
            self.arrived, self.held_at = obs.t, (x, y)
            self.ctx.emit('arrived', error_m=round(math.hypot(x - self.wp[0], y - self.wp[1]), 3))
            nav.hold()
        nav.keep_still()
        if obs.t - self.arrived < self.hold_s:
            return None
        self.result = {'arrive_s': self.arrived, 'overshoot_m': self.over,
                       'final_error_m': math.hypot(x - self.wp[0], y - self.wp[1]),
                       'drift_m': math.hypot(x - self.held_at[0], y - self.held_at[1])}
        self.ctx.emit('stopped', **{k: round(v, 4) for k, v in self.result.items()})
        return 'SUCCEEDED'


class WaypointAdapter(DomainAdapter):
    def identity(self):
        doc = cards.read(self.options.get('profile'))
        return {'domain': 'drones', 'task': 'waypoint', 'card': doc.get('id'), 'card_sha256': cards.sha256(doc),
                'options': self.options}

    def controller(self, ctx):
        self.last = Waypoint(ctx, self.options.get('hold_s', 3.0))
        return self.last

    def time_limit(self, ctx):
        return self.options.get('time_limit_s')

    def finalize(self, run_dir, result):
        plant = result.get('plant') or {}
        doc = {'outcome': result['outcome'], **(self.last.result if hasattr(self, 'last') else {}),
               **{k: plant[k] for k in ('alt_dev_m', 'max_tilt_deg', 'saturated_s') if k in plant}}
        (Path(run_dir) / 'drones.json').write_text(json.dumps(doc, indent=1) + '\n')

    def metrics(self, run_dir):
        doc = json.loads((Path(run_dir) / 'drones.json').read_text())
        return {'stop': {k: v for k, v in doc.items() if isinstance(v, (int, float))}}
