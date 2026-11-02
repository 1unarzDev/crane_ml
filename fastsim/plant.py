"""Vehicle: limits, goals, a stand-in goal tracker and a pluggable plant, calibrated from YAML.

The tracker turns the live goal into a body-velocity target (straight at the next pose, turning
in place above ``rotate_above``, braking to stop at the last pose, then turning to its yaw);
the plant turns targets into motion. Plants:

  kinematic    rate limits (accel grows |v|, decel shrinks it), a per-axis command delay and a
               scale on the commanded translation (the fitted calibration's form)
  first_order  per-axis dead time then a first-order lag ``tau_s``, then the rate limits; its
               numbers may come from a plant card file through dotted paths (``card``/``card_map``)
  (a plugin)   any PlantModel subclass by reference

A vehicle spec is {model, gain, **plant options}; ``gain`` (1/s, heading error to yaw rate)
is the tracker's one tuned number, the rest the plant's.
"""

from abc import ABC, abstractmethod
from dataclasses import asdict, dataclass, field
import json
import math
from pathlib import Path

import yaml

from fastsim import plugin
from fastsim.world import wrap

AXES = 3  # body (vx, vy, wz)


@dataclass(frozen=True)
class Limits:
    """What the controlled vehicle allows; a domain adapter may read them from its own stack."""

    vmax: tuple                    # (vx, vy, wz)
    vmin: tuple
    accel: tuple                   # rate bound on a growing |v|
    decel: tuple                   # rate bound on a shrinking |v| (negative)
    goal_tol: tuple                # (xy m, yaw rad) a goal ends on
    precise_tol: tuple             # the same for a precise goal
    rotate_above: float            # heading error (rad) above which it turns in place
    rotate_wz: float               # its turn rate
    progress_radius: float         # a goal that moves less than this ...
    progress_s: float              # ... for this long is aborted
    pass_radius: float             # a via-pose closer than this is passed
    footprint: tuple               # vehicle polygon, body frame
    extra: dict = field(default_factory=dict)  # adapter-owned numbers, recorded, unused here

    @classmethod
    def from_dict(cls, d):
        d = dict(d)
        d['footprint'] = tuple(tuple(p) for p in d['footprint'])
        return cls(**{k: tuple(v) if isinstance(v, list) else v for k, v in d.items()})

    def record(self):
        return json.loads(json.dumps(asdict(self)))


class Goal:
    """One goal as the controller holds it: route, done, status, cancel. ``reverse`` is
    (distance m, speed m/s, seconds) for a straight reverse leg instead of a route."""

    def __init__(self, route, precise=False, why='', reverse=None):
        self.route, self.precise, self.why, self.reverse = [tuple(p) for p in route], precise, why, reverse
        self.done, self.status = False, None
        self.travelled = self.started = None
        self.anchor = None  # progress checker: (xy, since)

    def end(self, status):
        if not self.done:
            self.done, self.status = True, status

    def cancel(self):
        self.end('CANCELED')


class GoalTracker:
    def __init__(self, limits, gain):
        self.lim, self.gain = limits, float(gain)

    def command(self, goal, pose, now, cap=None):
        lim = self.lim
        if goal is None or goal.done:
            return (0.0, 0.0, 0.0)
        x, y, yaw = pose
        if goal.reverse is not None:
            distance, speed, seconds = goal.reverse
            if goal.started is None:
                goal.started, goal.travelled = now, 0.0
            if goal.travelled >= distance:
                goal.end('SUCCEEDED')
                return (0.0, 0.0, 0.0)
            if now - goal.started > seconds:
                goal.end('ABORTED')
                return (0.0, 0.0, 0.0)
            return (-min(abs(speed), -lim.vmin[0]), 0.0, 0.0)
        route = goal.route
        while len(route) > 1 and math.dist(route[0][:2], (x, y)) < lim.pass_radius:
            route.pop(0)
        tx, ty, tyaw = route[0]
        dx, dy = tx - x, ty - y
        d = math.hypot(dx, dy)
        final = len(route) == 1
        xy_tol, yaw_tol = lim.precise_tol if goal.precise else lim.goal_tol
        if final and d <= xy_tol:
            err = wrap(tyaw - yaw)
            if abs(err) <= yaw_tol:
                goal.end('SUCCEEDED')
                return (0.0, 0.0, 0.0)
            return (0.0, 0.0, math.copysign(lim.rotate_wz, err))
        err = wrap(math.atan2(dy, dx) - yaw)
        if abs(err) > lim.rotate_above:
            return (0.0, 0.0, math.copysign(lim.rotate_wz, err))
        top = lim.vmax[0] if cap is None else min(lim.vmax[0], cap)
        speed = min(top, math.sqrt(2 * -lim.decel[0] * d)) if final else top
        c, s = math.cos(yaw), math.sin(yaw)
        bx, by = (c * dx + s * dy) / d * speed, (-s * dx + c * dy) / d * speed
        k = 1.0  # scale both axes together so the direction holds
        for v, hi, lo in ((bx, lim.vmax[0], lim.vmin[0]), (by, lim.vmax[1], lim.vmin[1])):
            if v > hi:
                k = min(k, hi / v)
            elif v < lo:
                k = min(k, lo / v)
        wz = max(lim.vmin[2], min(lim.vmax[2], self.gain * err))
        return (bx * k, by * k, wz)


class PlantModel(ABC):
    def __init__(self, limits, pose=(0.0, 0.0, 0.0), vel=(0.0, 0.0, 0.0)):
        self.lim, self.pose, self.vel = limits, tuple(pose), tuple(vel)

    @abstractmethod
    def velocity(self, target, dt):
        """The body velocity after one step toward ``target``."""

    def params(self):
        return {}

    def rate_limit(self, target, dt):
        vel = []
        for v, want, up, down in zip(self.vel, target, self.lim.accel, self.lim.decel):
            grow = abs(want) > abs(v) and want * v >= 0
            rate = up if grow else -down
            vel.append(v + max(-rate * dt, min(rate * dt, want - v)))
        return tuple(vel)

    def step(self, target, dt):
        """Advance one step; returns the world displacement (dx, dy)."""
        self.vel = self.velocity(target, dt)
        x, y, yaw = self.pose
        vx, vy, wz = self.vel
        c, s = math.cos(yaw), math.sin(yaw)
        moved = (c * vx - s * vy) * dt, (s * vx + c * vy) * dt
        self.pose = (x + moved[0], y + moved[1], wrap(yaw + wz * dt))
        return moved


class DelayLine:
    """Per-axis command delay, quantised to steps; before a delayed sample exists the axis
    holds the plant's current velocity."""

    def __init__(self, delay_s):
        self.delay_s, self.sent = tuple(float(d) for d in delay_s), []

    def __call__(self, target, dt, current):
        if not any(self.delay_s):
            return target
        self.sent.append(target)
        steps = [round(d / dt) for d in self.delay_s]
        del self.sent[:max(0, len(self.sent) - 1 - max(steps))]
        return tuple(self.sent[-1 - n][axis] if n < len(self.sent) else current[axis]
                     for axis, n in enumerate(steps))


class KinematicPlant(PlantModel):
    def __init__(self, limits, pose=(0.0, 0.0, 0.0), vel=(0.0, 0.0, 0.0), delay_s=(0.0,) * AXES,
                 speed_scale=1.0):
        super().__init__(limits, pose, vel)
        self.speed_scale, self.delay = float(speed_scale), DelayLine(delay_s)

    def velocity(self, target, dt):
        k = self.speed_scale
        if k != 1.0 or any(self.delay.delay_s):
            target = self.delay((target[0] * k, target[1] * k, target[2]), dt, self.vel)
        return self.rate_limit(target, dt)

    def params(self):
        return {'delay_s': list(self.delay.delay_s), 'speed_scale': self.speed_scale}


def _dotted(tree, path):
    for part in path.split('.'):
        tree = tree[int(part)] if isinstance(tree, list) else tree[part]
    return tree


def _per_axis(value):
    return tuple(float(v) for v in value) if isinstance(value, (list, tuple)) else (float(value),) * AXES


class FirstOrderPlant(PlantModel):
    def __init__(self, limits, pose=(0.0, 0.0, 0.0), vel=(0.0, 0.0, 0.0), tau_s=(0.0,) * AXES,
                 dead_time_s=(0.0,) * AXES, speed_scale=1.0, card=None, card_map=None, base=None):
        super().__init__(limits, pose, vel)
        if card:  # numbers from a plant card: {option: dotted path} into its JSON/YAML
            path = Path(base or '.') / Path(card).expanduser()
            tree = yaml.safe_load(path.read_text())
            found = {k: _dotted(tree, v) for k, v in (card_map or {}).items()}
            tau_s, dead_time_s = found.get('tau_s', tau_s), found.get('dead_time_s', dead_time_s)
            speed_scale = found.get('speed_scale', speed_scale)
        self.tau_s, self.speed_scale = _per_axis(tau_s), float(speed_scale)
        self.delay = DelayLine(_per_axis(dead_time_s))

    def velocity(self, target, dt):
        k = self.speed_scale
        target = self.delay((target[0] * k, target[1] * k, target[2]), dt, self.vel)
        lagged = tuple(v + (w - v) * (1.0 - math.exp(-dt / tau)) if tau > 0 else w
                       for v, w, tau in zip(self.vel, target, self.tau_s))
        return self.rate_limit(lagged, dt)

    def params(self):
        return {'tau_s': list(self.tau_s), 'dead_time_s': list(self.delay.delay_s),
                'speed_scale': self.speed_scale}


PLANTS = {'kinematic': KinematicPlant, 'first_order': FirstOrderPlant}


def vehicle(spec, limits, pose=(0.0, 0.0, 0.0), vel=(0.0, 0.0, 0.0), base=None):
    """(GoalTracker, PlantModel) from a vehicle spec {model|ref, gain, **plant options}."""
    spec = dict(spec)
    gain = spec.pop('gain')
    model = spec.pop('model', None)
    plant_spec = {'name': model or 'kinematic', **spec} if 'ref' not in spec else spec
    if model == 'first_order':
        plant_spec['base'] = base
    return GoalTracker(limits, gain), plugin.build(plant_spec, PLANTS, base, limits=limits,
                                                   pose=pose, vel=vel)
