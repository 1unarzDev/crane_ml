"""Vehicles: a plant kernel plus whatever turns the tracker's body-velocity target into its input.

    v = vehicle(cfg, limits, base, dt)           # from a fastsim vehicle spec
    V = v.init(E)                                # {'x': [E, S], 'c': [E, 3]} plant state, loop state
    pose = v.pose(xp, V)                         # (x, y, yaw) [E] in the map frame, yaw wrapped
    V = v.drive(ops, V, target, p, q, dt)        # one core step toward target [E, 3]

  profile      the boat's T1 (domains.boat.plant:ProfileBoat): smoother, FF + P velocity loop,
               pseudo-inverse allocation to PWM, then ``substeps`` ProfilePlant ticks holding it
               (the substep count is fixed per card: dt / the profile's tick)
  first_order  fastsim's first_order plant (T0)

``nb_drive(kind, x, c, target, p, q, dt, substeps, pwm, blas)`` is the numba twin of ``drive``.
Each env carries its own parameter row (``p`` [E, P]); ``randomize`` draws per-env values from the
env's own RNG stream, so env i's parameters never depend on the batch.
"""

import json
import math

import numba
import numpy as np

from engine import exact, nbexact
from engine.kernels import first_order as FO, profile as PK
from fastsim.identity import derive_seed

PROFILE, FIRST_ORDER = 0, 1


class Vehicle:
    kind = None

    def pose(self, xp, V):
        x = V['x']
        if self.kind == PROFILE:
            return x[:, 0], x[:, 1], exact.wrap(x[:, 2], xp)
        return x[:, 0], x[:, 1], x[:, 2]

    def rows(self, seeds):
        """[E, P] parameters: the packed row, with any per-env draws from each env's own stream."""
        return np.repeat(self.p[None], len(seeds), axis=0)


class ProfileVehicle(Vehicle):
    kind = PROFILE
    name = 'profile_boat'

    def __init__(self, spec, limits, base, dt, randomize=None):
        from domains.boat.plant import profile_path
        path = profile_path(spec.get('profile', 'Tools/Characterization/profiles/characterized.json'))
        self.profile = json.loads(path.read_bytes())
        self.tick_s = float(spec.get('tick_s') or self.profile['fixedDeltaTime'])
        n = round(dt / self.tick_s)
        if n < 1 or abs(n * self.tick_s - dt) > 1e-9:
            raise ValueError(f'dt {dt} is not a multiple of the plant tick {self.tick_s}')
        self.substeps = n
        current = tuple(spec.get('current', (0.0, 0.0)))
        self.p = PK.pack(self.profile, self.tick_s, current)
        kp = spec.get('kp', (2.0, 2.0, 2.0))
        self.q = PK.pack_loop(self.profile, kp, limits.accel, limits.decel, spec.get('smooth', True))
        self.S = PK.size(self.p)
        self.randomize = dict(randomize or {})
        self.ident = {'kernel': self.name, 'substeps': {'policy': 'fixed_per_card', 'n': n, 'tick_s': self.tick_s},
                      'randomize': self.randomize}

    def rows(self, seeds):
        P = super().rows(seeds)
        sd = float(self.randomize.get('current_sd', 0.0))
        if sd > 0:
            for e, s in enumerate(seeds):
                g = np.random.default_rng(derive_seed(s, 'engine', 'current'))
                P[e, PK.IX['cx']], P[e, PK.IX['cy']] = sd * g.standard_normal(2)
        return P

    def init(self, E):
        return {'x': PK.init(np.repeat(self.p[None], E, axis=0), S=self.S), 'c': np.zeros((E, 3))}

    def drive(self, ops, V, target, p, q, dt):
        xp = ops.xp
        pwm, c = PK.loop_pwm(V['x'], V['c'], target, p, q, dt, xp, ops.blas)
        x = V['x']
        for _ in range(self.substeps):
            x = PK.step(x, pwm, p, self.tick_s, xp)
        return {'x': x, 'c': c}


class FirstOrderVehicle(Vehicle):
    kind = FIRST_ORDER
    name = 'first_order'

    def __init__(self, spec, limits, base, dt, randomize=None):
        if spec.get('card'):
            raise NotImplementedError('engine: a first_order plant from a card file; give tau_s inline')
        self.p = FO.pack(spec, limits, dt)
        self.q = np.zeros(1)
        self.S = FO.size(self.p)
        self.substeps = 1
        self.tick_s = dt
        self.randomize = dict(randomize or {})
        self.ident = {'kernel': self.name, 'substeps': {'policy': 'fixed', 'n': 1}, 'randomize': self.randomize}

    def init(self, E):
        return {'x': FO.init(np.repeat(self.p[None], E, axis=0), S=self.S), 'c': np.zeros((E, 3))}

    def drive(self, ops, V, target, p, q, dt):
        return {'x': FO.step(V['x'], target, p, dt, ops.xp), 'c': V['c']}


def vehicle(spec, limits, base, dt, randomize=None):
    """The engine's vehicle for a fastsim vehicle spec, or NotImplementedError naming what is missing."""
    spec = dict(spec)
    gain = float(spec.pop('gain'))
    ref, model = spec.pop('ref', None), spec.pop('model', None)
    if ref and ref.endswith(':ProfileBoat'):
        v = ProfileVehicle(spec, limits, base, dt, randomize)
    elif model == 'first_order':
        v = FirstOrderVehicle(spec, limits, base, dt, randomize)
    else:
        raise NotImplementedError(f'engine: no kernel for vehicle {ref or model!r} (P4 ports the other plants)')
    v.gain = gain
    return v


@numba.njit(cache=True, inline='always')
def nb_drive(kind, x, c, target, p, q, dt, tick_s, substeps, pwm, blas):
    if kind == PROFILE:
        cs, sn = PK.nb_loop_pwm(x, c, target, p, q, dt, pwm, blas)
        PK.nb_step_cs(x, pwm, p, tick_s, cs, sn)
        for _ in range(substeps - 1):
            PK.nb_step(x, pwm, p, tick_s)
    else:
        FO.nb_step(x, target, p, dt)


@numba.njit(cache=True, inline='always')
def nb_pose(kind, x):
    if kind == PROFILE:
        return x[0], x[1], nbexact.wrap(x[2])
    return x[0], x[1], x[2]


__all__ = ['vehicle', 'nb_drive', 'nb_pose', 'PROFILE', 'FIRST_ORDER', 'math']
