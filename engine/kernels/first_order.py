"""The boat's T0 kernel: fastsim's ``FirstOrderPlant`` (per-axis dead time, first-order lag, the
``Limits`` rate limits, then planar kinematics) on a batch.

    p = pack(spec, limits, dt)       # [P]: lag gains, speed scale, delay steps, rate limits
    x = init(p[None])                # [E, S]: x y yaw vx vy wz | count | ring of delayed targets
    x = step(x, target, p, dt)       # one core step toward the body-velocity target [E, 3]

The dead time is ``DelayLine``'s: per axis ``round(d / dt)`` steps, from a ring of the last
max + 1 targets; before a delayed sample exists the axis holds the current velocity.
"""

import math

import numpy as np

from engine import exact
from engine.jit import nbexact, numba

NAMES = ['g0', 'g1', 'g2', 'lag0', 'lag1', 'lag2', 'k', 'n0', 'n1', 'n2', 'delayed',
         'up0', 'up1', 'up2', 'down0', 'down1', 'down2']
IX = {n: i for i, n in enumerate(NAMES)}
P = len(NAMES)
BODY, COUNT, RING = 6, 6, 7


def per_axis(value):
    return tuple(float(v) for v in value) if isinstance(value, (list, tuple)) else (float(value),) * 3


def pack(spec, limits, dt):
    """A parameter row from a fastsim ``first_order`` vehicle spec and its Limits (the constants
    computed as FirstOrderPlant computes them)."""
    tau = per_axis(spec.get('tau_s', 0.0))
    delay = per_axis(spec.get('dead_time_s', 0.0))
    gains = [1.0 - math.exp(-dt / t) if t > 0 else 0.0 for t in tau]
    steps = [float(round(d / dt)) for d in delay]
    return np.array(gains + [1.0 if t > 0 else 0.0 for t in tau] + [float(spec.get('speed_scale', 1.0))] + steps
                    + [1.0 if any(delay) else 0.0] + list(limits.accel) + list(limits.decel), dtype=np.float64)


def size(p):
    p = np.atleast_2d(p)
    return RING + 3 * (int(p[:, [IX['n0'], IX['n1'], IX['n2']]].max()) + 1)


def init(p, pose=(0.0, 0.0, 0.0), vel=(0.0, 0.0, 0.0), S=None):
    p = np.atleast_2d(p)
    x = np.zeros((p.shape[0], S or size(p)))
    x[:, :BODY] = (*pose, *vel)
    return x


def _rate_limit(xp, v, want, up, down, dt):
    rate = xp.where((xp.abs(want) > xp.abs(v)) & (want * v >= 0), up, -down)
    return v + xp.maximum(-rate * dt, xp.minimum(rate * dt, want - v))


def step(x, u, p, dt, xp=np):
    L = (x.shape[-1] - RING) // 3
    k = p[..., IX['k']]
    target = [u[..., 0] * k, u[..., 1] * k, u[..., 2]]
    vel = [x[..., 3 + j] for j in range(3)]
    count = x[..., COUNT]
    ring = x[..., RING:].reshape(x.shape[:-1] + (L, 3))
    delayed = p[..., IX['delayed']] > 0
    slot = xp.arange(L)
    write = slot == xp.remainder(count, L)[..., None]
    new = xp.stack(target, axis=-1)
    ring = xp.where(delayed[..., None, None] & write[..., None], new[..., None, :], ring)
    count = xp.where(delayed, count + 1.0, count)
    have = xp.minimum(count, float(L))
    out = []
    for j in range(3):
        n = p[..., IX[f'n{j}']]
        at = slot == xp.remainder(count - 1.0 - n, L)[..., None]
        sample = xp.sum(xp.where(at, ring[..., j], 0.0 * ring[..., j]), axis=-1)
        out.append(xp.where(delayed, xp.where(n < have, sample, vel[j]), target[j]))
    new_vel = []
    for j in range(3):
        v, w = vel[j], out[j]
        lagged = xp.where(p[..., IX[f'lag{j}']] > 0, v + (w - v) * p[..., IX[f'g{j}']], w)
        new_vel.append(_rate_limit(xp, v, lagged, p[..., IX[f'up{j}']], p[..., IX[f'down{j}']], dt))
    X, Y, yaw = x[..., 0], x[..., 1], x[..., 2]
    vx, vy, wz = new_vel
    c, s = xp.cos(yaw), xp.sin(yaw)
    m0, m1 = (c * vx - s * vy) * dt, (s * vx + c * vy) * dt
    body = xp.stack([X + m0, Y + m1, exact.wrap(yaw + wz * dt, xp), vx, vy, wz, count], axis=-1)
    return xp.concatenate([body, ring.reshape(x.shape[:-1] + (3 * L,))], axis=-1)


@numba.njit(cache=True, inline='always')
def nb_step(x, u, p, dt):
    L = (x.shape[0] - RING) // 3
    k = p[6]
    t0, t1, t2 = u[0] * k, u[1] * k, u[2]
    o0, o1, o2 = t0, t1, t2
    if p[10] > 0:
        c = int(x[COUNT])
        w = RING + 3 * (c % L)
        x[w], x[w + 1], x[w + 2] = t0, t1, t2
        c += 1
        x[COUNT] = float(c)
        have = min(c, L)
        o = np.empty(3)
        for j in range(3):
            n = int(p[7 + j])
            o[j] = x[RING + 3 * ((c - 1 - n) % L) + j] if n < have else x[3 + j]
        o0, o1, o2 = o[0], o[1], o[2]
    out = (o0, o1, o2)
    for j in range(3):
        v, w = x[3 + j], out[j]
        lagged = v + (w - v) * p[j] if p[3 + j] > 0 else w
        rate = p[11 + j] if abs(lagged) > abs(v) and lagged * v >= 0 else -p[14 + j]
        x[3 + j] = v + max(-rate * dt, min(rate * dt, lagged - v))
    vx, vy, wz = x[3], x[4], x[5]
    c, s = math.cos(x[2]), math.sin(x[2])
    x[0] = x[0] + (c * vx - s * vy) * dt
    x[1] = x[1] + (s * vx + c * vy) * dt
    x[2] = nbexact.wrap(x[2] + wz * dt)
