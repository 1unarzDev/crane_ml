"""The boat domain for fastsim: a public reference mission over a crane course truth.

    python3 -m fastsim.batch --config domains/boat/fastsim_t1.yaml --seeds 0:5 --out runs/boat

One fastsim YAML per tier (``fastsim_t0.yaml``, ``fastsim_t1.yaml``; config/domains/boat.yaml
names them). The mission (``Mission``) is a small FSM on fastsim's navigator: the course leg
(entry gates, nav channel, the speed gate, a U-turn round its mark and back through the gate),
then down the fairway to the chosen slip, then a precise goal into it. ``finalize`` scores the
run into ``boat.json``; ``metrics`` reads it.
"""

import json
import math
from pathlib import Path

from fastsim.controller import Controller, DomainAdapter
from fastsim.identity import sha256

from domains.boat import mission
from domains.boat.plant import profile_path

TIERS = ('T0', 'T1')
MISSION = 'reference/1'
DOCK_TOL_M = 0.5  # a docked run ends this close to the slip's centre


class Mission(Controller):
    """course -> fairway -> slip, one goal per phase. A phase whose goal is aborted (no
    progress) is sent again once with what is left of its route; then the run fails."""

    PHASES = ('course', 'fairway', 'slip')

    def __init__(self, ctx, plan):
        self.ctx, self.plan = ctx, plan
        world = ctx.world

        def mapped(points):
            return [(float(x), float(y)) for x, y in world.to_map(points)]
        d = plan.dock
        self.course = mapped(plan.course)
        self.mouths = mapped(d['mouths'])
        self.approach = mapped([d['approach']])[0]
        self.berth = (*mapped([(d['x'], d['y'])])[0], d['yaw'] - world.start[2])
        self.obstacles = [(x, y, o[2]) for (x, y), o in zip(mapped([o[:2] for o in plan.obstacles]), plan.obstacles)]
        self.buoys = mapped([p for g in plan.gates for p in (g.port, g.starboard)])
        self.phase, self.retried, self.goal = 0, False, None
        self.seen = set()  # gate buoys the sensor has reported

    def route(self, here, phase):
        """(route, precise) of a phase from ``here``."""
        if phase == 'course':
            return mission.route([here] + self.course, self.obstacles, [False] + self.plan.centres)[1:], False
        if phase == 'fairway':
            mouth = min(self.mouths, key=lambda m: math.dist(m, here))
            legs = mission.route([here, mouth], self.obstacles)[1:]
            return legs + [(*self.approach, self.berth[2])], False
        return [self.berth], True

    def send(self, route, precise, why):
        self.goal = self.ctx.navigator.send(route, precise=precise, why=why)

    def tick(self, obs):
        for t in obs.tracks:
            for k, b in enumerate(self.buoys):
                if k not in self.seen and abs(t.x - b[0]) < 0.5 and abs(t.y - b[1]) < 0.5:
                    self.seen.add(k)
        name = self.PHASES[self.phase]
        goal = self.goal
        if goal is None:
            self.ctx.emit('phase', name=name)
            self.send(*self.route(obs.pose[:2], name), name)
            return None
        if not goal.done:
            return None
        if goal.status == 'SUCCEEDED':
            self.retried, self.goal = False, None
            if self.phase == len(self.PHASES) - 1:
                self.ctx.navigator.hold()
                self.ctx.emit('docked', slip=self.plan.dock['id'])
                return 'SUCCEEDED'
            self.phase += 1
            return self.tick(obs)
        if self.retried:
            self.ctx.emit('failed', phase=name, status=goal.status)
            return 'FAILED'
        self.retried = True
        self.ctx.emit('retry', phase=name, status=goal.status)
        self.send(list(goal.route), goal.precise, name)
        return None


class BoatAdapter(DomainAdapter):
    """Options: ``tier`` (T0|T1, recorded) and ``profile`` (the T1 plant's profile file, whose
    content is hashed into every run identity)."""

    def __init__(self, base=None, tier='T1', profile=None, **options):
        super().__init__(base, tier=tier, profile=profile, **options)
        if tier not in TIERS:
            raise ValueError(f'boat: tier {tier!r}; known: {TIERS}')
        self.tier, self.plans, self.current = tier, {}, None
        self.ident = {'domain': 'boat', 'tier': tier, 'mission': MISSION}
        if profile:
            self.ident['profile_sha256'] = sha256(profile_path(profile).read_bytes())

    def identity(self):
        return dict(self.ident)

    def plan(self, scenario):
        key = scenario.content_sha256
        if key not in self.plans:
            self.plans = {key: mission.Plan.from_truth(scenario.raw)}
        return self.plans[key]

    def controller(self, ctx):
        self.current = Mission(ctx, self.plan(ctx.scenario))
        return self.current

    def condition(self, ctx):
        return {'tier': self.tier, 'mission': MISSION}

    def finalize(self, run_dir, result):
        plan = self.plan(result['context'].scenario)
        track = [(t, x, y) for t, x, y, _ in result['track']]
        numbers = mission.score(plan, track, result['track'][-1][1:])
        numbers.update(contacts=len(result['contacts']), time_s=result['sim_s'], slip=plan.dock['id'],
                       gate_buoys_seen=len(self.current.seen), tier=self.tier)
        numbers['success'] = int(result['outcome'] == 'SUCCEEDED' and numbers['gates_passed'] == numbers['gates_total']
                                 and numbers['dock_error_m'] <= DOCK_TOL_M)
        (Path(run_dir) / 'boat.json').write_text(json.dumps(numbers, indent=2, sort_keys=True) + '\n')

    def metrics(self, run_dir):
        b = json.loads((Path(run_dir) / 'boat.json').read_text())
        return {'boat': {k: v for k, v in b.items() if isinstance(v, (int, float))}}
