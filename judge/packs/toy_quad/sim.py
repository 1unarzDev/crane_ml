"""Planar quadrotor simulation (fixed step), its gate-following controller and the Source."""

from __future__ import annotations

from dataclasses import dataclass, replace
import math
import random

import numpy as np

from judge.record import (Channel, ChannelSpec, Clock, Event, Header, Robot, RunRecord, Scenario,
                          Validity)
from judge.source import FidelityProfile, Source

G = 9.81
GATES = ((4.0, 2.0), (8.0, 3.5), (12.0, 2.0), (16.0, 3.0))
GATE_HALF = 0.6


@dataclass(frozen=True)
class QuadParams:
    mass: float = 1.0
    arm: float = 0.2
    inertia: float = 0.02
    motor_tau: float = 0.0       # s, first-order motor lag
    drag: float = 0.0            # 1/s linear drag
    wind: float = 0.0            # m/s^2 gust std (OU)
    t_max: float = 12.0          # N per motor
    skip_gate: int = -1          # decision fault: never target this gate index
    cruise: float = 1.6          # m/s


PARAMS_LOW = QuadParams()
PARAMS_HIGH = QuadParams(motor_tau=0.12, drag=0.35, wind=0.6)


class Controller:
    def __init__(self, p: QuadParams, gate=0):
        self.p, self.gate = p, gate

    def target(self):
        return GATES[self.gate] if self.gate < len(GATES) else (GATES[-1][0] + 3.0, GATES[-1][1])

    def advance(self, x, z, events, t):
        """Gate logic: the decision layer. Returns True when the target changed."""
        if self.gate >= len(GATES):
            return False
        gx, gz = GATES[self.gate]
        if x >= gx:
            if abs(z - gz) <= GATE_HALF:
                events.append(Event(t, 'gate.passed', 'world', {'why': f'g{self.gate}'}))
            self.gate += 1
            if self.gate == self.p.skip_gate:
                self.gate += 1
            events.append(Event(t, 'gate.target', 'planner', {'why': f'g{self.gate}' if self.gate < len(GATES) else 'exit'}))
            return True
        return False

    def command(self, s):
        x, z, th, vx, vz, w = s
        tx, tz = self.target()
        ax = 1.2 * (min(self.p.cruise, max(-self.p.cruise, 0.8 * (tx - x))) - vx)
        az = 2.0 * (tz - z) - 1.8 * vz
        th_des = -math.atan2(ax, G + az)
        th_des = max(-0.5, min(0.5, th_des))
        total = self.p.mass * (G + az) / max(0.3, math.cos(th))
        tau = self.p.inertia * (40.0 * (th_des - th) - 10.0 * w)
        diff = tau / self.p.arm
        left, right = (total - diff) / 2, (total + diff) / 2
        return (max(0.0, min(self.p.t_max, left)), max(0.0, min(self.p.t_max, right)))


def step(s, thrust, p: QuadParams, dt, gust=(0.0, 0.0)):
    x, z, th, vx, vz, w = s
    T = thrust[0] + thrust[1]
    ax = -T * math.sin(th) / p.mass - p.drag * vx + gust[0]
    az = T * math.cos(th) / p.mass - G - p.drag * vz + gust[1]
    wd = (thrust[1] - thrust[0]) * p.arm / p.inertia
    vx, vz, w = vx + ax * dt, vz + az * dt, w + wd * dt
    return (x + vx * dt, z + vz * dt, th + w * dt, vx, vz, w)


def motor(actual, cmd, p, dt):
    if p.motor_tau <= 0:
        return cmd
    a = 1 - math.exp(-dt / p.motor_tau)
    return tuple(m + (c - m) * a for m, c in zip(actual, cmd))


def rollout(p: QuadParams, s0, t0, t1, dt=0.01, gate=0, cmds=None, seed=0):
    """Simulate from state s0 at t0 to t1. ``cmds`` (t, [N,2]) replays commands open loop;
    otherwise the controller flies from gate ``gate``. Returns (t, states, cmds, events)."""
    rng = random.Random(seed)
    ctl = Controller(p, gate)
    s, act, gust = tuple(s0), None, [0.0, 0.0]
    T, S, C, ev = [], [], [], []
    t = t0
    while t <= t1 + 1e-9:
        if cmds is None:
            ctl.advance(s[0], s[1], ev, t)
            c = ctl.command(s)
        else:
            c = tuple(np.interp(t, cmds[0], cmds[1][:, k]) for k in range(2))
        act = c if act is None else motor(act, c, p, dt)
        if p.wind > 0:
            gust = [g + (-g * 0.5 * dt + p.wind * math.sqrt(dt) * rng.gauss(0, 1)) for g in gust]
        T.append(t)
        S.append(s)
        C.append(c)
        s = step(s, act, p, dt, gust)
        if s[1] < 0:
            ev.append(Event(t, 'crash', 'world', {'why': 'ground'}))
            break
        t = round(t + dt, 10)
    return np.array(T), np.array(S), np.array(C), ev


def simulate(p: QuadParams = PARAMS_LOW, seed=0, tier='low', run_id=None, dt=0.01, log_hz=20):
    rng = random.Random(seed)
    s0 = (0.0, 1.0 + 0.2 * rng.uniform(-1, 1), 0.0, 0.0, 0.0, 0.0)
    T, S, C, ev = rollout(p, s0, 0.0, 20.0, dt, seed=seed + 1000)
    k = max(1, int(round(1 / (log_hz * dt))))
    T, S, C = T[::k], S[::k], C[::k]
    pose = Channel.of(ChannelSpec('pose.xz', ('x', 'z', 'pitch'), ('m', 'm', 'rad'), 'world.xz', 'exact',
                                  log_hz, ('pitch',)), T, S[:, :3])
    tw = Channel.of(ChannelSpec('twist.xz', ('vx', 'vz', 'wpitch'), ('m/s', 'm/s', 'rad/s'), 'world.xz',
                                'exact', log_hz), T, S[:, 3:])
    cmd = Channel.of(ChannelSpec('actuator.cmd', ('left', 'right'), ('N', 'N'), 'body', 'commanded', log_hz), T, C)
    end = 'crash' if any(e.kind == 'crash' for e in ev) else 'time'
    h = Header(run_id or f'quad-{tier}-s{seed}', tier, 'toy_quad', 'planar_quad',
               Robot('toyquad', 'planar-2motor'), 'gates4',
               Scenario('gates4', 'toy', seed, 'gates', {'course': 'gates4'}),
               {'params': p.__dict__}, Clock('sim'), (float(T[0]), float(T[-1]), end))
    return RunRecord(h, {'pose.xz': pose, 'twist.xz': tw, 'actuator.cmd': cmd}, tuple(ev),
                     {'truth': 'exact'}, Validity(), {'gates': [list(g) for g in GATES], 'half': GATE_HALF},
                     {'params': p})


class ToyQuadSource(Source):
    """In-memory source: refs are (tier, seed, params)."""
    name = 'toy_quad'
    profile = FidelityProfile('physics', 'truth', 'fixed_step', 'parametric', 0.01, 'planar toy')

    def __init__(self, tier, params, seeds):
        self.tier, self.params, self.seeds = tier, params, seeds

    def discover(self, root=None):
        yield from self.seeds

    def load(self, seed):
        return simulate(self.params, seed, self.tier)


def variant(p, **kw):
    return replace(p, **kw)
