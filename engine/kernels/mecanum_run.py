"""Batched runs of the modular kernel (engine/kernels/mecanum.py): command-timeline replay and closed-loop ticks.

    out = replay(p, timelines, which=None, lengths=None, delay=None, dt=0.005, nsub=4, backend='numba')
    plant = Ticks(p, dt=0.005, nsub=4, delay=None, ticks=T, backend='numba'); x = plant.tick(u)

``timelines`` [M, T, 3] are body-twist commands (vx, vy, wz) at the tick rate (1 / (nsub dt): 50 Hz at 5 ms and
nsub 4); env e replays timeline ``which[e]`` (default e), each its own. A tick's command is held over its nsub plant
steps; ``delay`` [E] plant steps of command latency (plant step k uses the command of tick floor((k - delay) /
nsub), zero before the first tick and after the timeline's length). The channels are sampled at the end of
every tick:

    cmd [E, T, 3]   the logged command of the tick          duty [E, T, W]    the firmware's pre-fault duty
    counts [E, T, W] encoder counts                          speed [E, T, W]   wheel speed, rad/s
    gz ax ay [E, T]  IMU yaw rate and body acceleration      pose twist [E, T, 3]  the truth (labels, figures)

Backends: numpy (float64, the reference), numba (float64, envs across every core) and jax (float32 or float64,
replay only). An env reads only its own rows: env e at E = 1 is env e at any E (numpy, numba bit for bit).
"""

import math

import numpy as np

from engine.jit import numba
from engine.kernels import mecanum as K

I_CPR, I_IMX, I_IMY = K.IB['cpr'], K.IB['imu_x'], K.IB['imu_y']
CHANNELS = ('cmd', 'duty', 'counts', 'speed', 'gz', 'ax', 'ay', 'pose', 'twist')


def _prepare(p, timelines, which, lengths, delay, pose):
    p = np.ascontiguousarray(np.atleast_2d(p), dtype=np.float64)
    tl = np.ascontiguousarray(timelines, dtype=np.float64)
    if tl.ndim != 3 or tl.shape[-1] != 3:
        raise ValueError(f'timelines: shape [M, T, 3] (vx, vy, wz), not {tl.shape}')
    E = p.shape[0]
    which = np.arange(E) if which is None else np.asarray(which, dtype=np.int64)
    if which.shape != (E,) or which.min() < 0 or which.max() >= tl.shape[0]:
        raise ValueError(f'which: one timeline index per env ({E}) in 0..{tl.shape[0] - 1}')
    lengths = np.full(tl.shape[0], tl.shape[1], np.int64) if lengths is None else np.asarray(lengths, np.int64)
    delay = np.zeros(E, np.int64) if delay is None else np.broadcast_to(np.asarray(delay, np.int64), (E,)).copy()
    x0 = K.init(p, (0.0, 0.0, 0.0) if pose is None else pose)
    if pose is not None and np.ndim(pose) == 2:
        x0[:, 0:3] = pose
    return p, tl, which, lengths, delay, x0


def _command(tl, which, lengths, delay, k, nsub):
    idx = (k - delay) // nsub
    ok = (idx >= 0) & (idx < lengths[which])
    u = tl[which, np.clip(idx, 0, tl.shape[1] - 1)]
    return np.where(ok[:, None], u, 0.0)


def _alloc(E, T, W, dtype):
    out = {c: np.zeros((E, T, 3), dtype) for c in ('cmd', 'pose', 'twist')}
    out.update({c: np.zeros((E, T, W), dtype) for c in ('duty', 'counts', 'speed')})
    out.update({c: np.zeros((E, T), dtype) for c in ('gz', 'ax', 'ay')})
    return out


def _sample(out, j, x, p, u):
    E, W = x.shape[0], K.wheels_of(p)
    wx = x[:, K.S0:].reshape(E, W, K.PER)
    out['cmd'][:, j], out['pose'][:, j], out['twist'][:, j] = u, x[:, 0:3], x[:, 3:6]
    out['duty'][:, j], out['speed'][:, j], out['counts'][:, j] = wx[..., 2], wx[..., 1], K.counts(x, p)
    out['gz'][:, j], out['ax'][:, j], out['ay'][:, j] = K.imu(x, p)


def replay(p, timelines, which=None, lengths=None, delay=None, dt=0.005, nsub=4, backend='numba', dtype='float64',
           ticks=None, pose=None):
    """Every env replays its timeline for ``ticks`` ticks (default the longest of those replayed): the channels."""
    p, tl, which, lengths, delay, x0 = _prepare(p, timelines, which, lengths, delay, pose)
    T = int(ticks if ticks is not None else lengths[which].max())
    E, W = p.shape[0], K.wheels_of(p)
    if backend == 'numpy':
        out, x = _alloc(E, T, W, np.float64), x0
        for j in range(T):
            for s in range(nsub):
                x = K.step(x, _command(tl, which, lengths, delay, j * nsub + s, nsub), p, dt)
            _sample(out, j, x, p, _command(tl, which, lengths, np.zeros(E, np.int64), j * nsub, nsub))
    elif backend == 'numba':
        out = _alloc(E, T, W, np.float64)
        _nb_replay(x0, p, tl, which, lengths, delay, dt, nsub, T, out['cmd'], out['pose'], out['twist'],
                   out['duty'], out['speed'], out['counts'], out['gz'], out['ax'], out['ay'])
    elif backend == 'jax':
        out = _jax_replay(x0, p, tl, which, lengths, delay, dt, nsub, T, dtype)
    else:
        raise ValueError(f'backend {backend!r}: numpy, numba or jax')
    return {k: v.astype(dtype, copy=False) for k, v in out.items()} if backend != 'jax' else out


@numba.njit(cache=True, parallel=True)
def _nb_replay(x0, p, tl, which, lengths, delay, dt, nsub, T, o_cmd, o_pose, o_twist, o_duty, o_speed, o_counts,
               o_gz, o_ax, o_ay):
    E = x0.shape[0]
    W = (p.shape[1] - K.G) // K.Q
    for e in numba.prange(E):
        x = x0[e].copy()
        pe = p[e]
        cmd, om, scratch, Wr, A, r = np.zeros(W), np.zeros(W), np.zeros((4, W)), np.zeros((4, 3)), \
            np.zeros((3, 3)), np.zeros(3)
        u = np.zeros(3)
        m, L, d = which[e], lengths[which[e]], delay[e]
        for j in range(T):
            for s in range(nsub):
                idx = (j * nsub + s - d) // nsub
                for c in range(3):
                    u[c] = tl[m, idx, c] if (idx >= 0 and idx < L) else 0.0
                K.nb_step(x, u, pe, dt, cmd, om, scratch, Wr, A, r)
            for c in range(3):
                o_cmd[e, j, c] = tl[m, j, c] if j < L else 0.0
                o_pose[e, j, c] = x[c]
                o_twist[e, j, c] = x[3 + c]
            cpr = pe[I_CPR]
            for k in range(W):
                b = K.S0 + K.PER * k
                o_duty[e, j, k] = x[b + 2]
                o_speed[e, j, k] = x[b + 1]
                o_counts[e, j, k] = math.floor(x[b] / (2 * math.pi) * cpr)
            rx, ry = pe[I_IMX], pe[I_IMY]
            vx, vy, wz, dvx, dvy, dwz = x[3], x[4], x[5], x[6], x[7], x[8]
            o_gz[e, j] = wz
            o_ax[e, j] = dvx - wz * vy - dwz * ry - wz * wz * rx
            o_ay[e, j] = dvy + wz * vx + dwz * rx - wz * wz * ry


def _jax_replay(x0, p, tl, which, lengths, delay, dt, nsub, T, dtype):
    from engine.jaxconf import jax, jnp
    f = jnp.float32 if dtype == 'float32' else jnp.float64
    E, W = p.shape[0], K.wheels_of(p)
    pj, tlj = jnp.asarray(p, f), jnp.asarray(tl, f)
    wj, Lj, dj = jnp.asarray(which), jnp.asarray(lengths[which]), jnp.asarray(delay)

    def command(k):
        idx = (k - dj) // nsub
        ok = (idx >= 0) & (idx < Lj)
        u = tlj[wj, jnp.clip(idx, 0, tl.shape[1] - 1)]
        return jnp.where(ok[:, None], u, 0.0)

    def tick(x, j):
        x = jax.lax.fori_loop(0, nsub, lambda s, x: K.step(x, command(j * nsub + s), pj, dt, jnp), x)
        wx = x[:, K.S0:].reshape(E, W, K.PER)
        u = command(j * nsub + dj)
        gz, ax, ay = K.imu(x, pj, jnp)
        return x, (u, x[:, 0:3], x[:, 3:6], wx[..., 2], wx[..., 1], K.counts(x, pj, jnp), gz, ax, ay)
    run = jax.jit(lambda x: jax.lax.scan(tick, x, jnp.arange(T)))
    _, ys = run(jnp.asarray(x0, f))
    names = ('cmd', 'pose', 'twist', 'duty', 'speed', 'counts', 'gz', 'ax', 'ay')
    return {n: np.moveaxis(np.asarray(y), 0, 1) for n, y in zip(names, ys)}


class Ticks:
    """A closed-loop batch: ``tick(u)`` takes this tick's commands [E, 3], runs nsub plant steps (with each env's
    command latency) and returns the state [E, S]. numpy or numba."""

    def __init__(self, p, dt=0.005, nsub=4, delay=None, ticks=1, backend='numba', pose=None):
        self.p, _, _, _, self.delay, self.x = _prepare(p, np.zeros((1, 1, 3)), np.zeros(np.shape(np.atleast_2d(p))[0],
                                                                                         np.int64), None, delay, pose)
        if backend not in ('numpy', 'numba'):
            raise ValueError(f'backend {backend!r}: numpy or numba (closed loop)')
        self.dt, self.nsub, self.backend, self.j = dt, nsub, backend, 0
        self.hist = np.zeros((self.p.shape[0], int(ticks), 3))

    def tick(self, u):
        E = self.p.shape[0]
        self.hist[:, self.j] = u
        n = np.full(self.hist.shape[0], self.j + 1, np.int64)
        if self.backend == 'numpy':
            for s in range(self.nsub):
                k = self.j * self.nsub + s
                idx = (k - self.delay) // self.nsub
                ok = (idx >= 0) & (idx < n)
                uu = np.where(ok[:, None], self.hist[np.arange(E), np.clip(idx, 0, self.j)], 0.0)
                self.x = K.step(self.x, uu, self.p, self.dt)
        else:
            _nb_tick(self.x, self.p, self.hist, self.j, self.delay, self.dt, self.nsub)
        self.j += 1
        return self.x


@numba.njit(cache=True, parallel=True)
def _nb_tick(x, p, hist, j, delay, dt, nsub):
    E = x.shape[0]
    W = (p.shape[1] - K.G) // K.Q
    for e in numba.prange(E):
        cmd, om, scratch, Wr, A, r = np.zeros(W), np.zeros(W), np.zeros((4, W)), np.zeros((4, 3)), \
            np.zeros((3, 3)), np.zeros(3)
        u = np.zeros(3)
        xe = x[e]
        for s in range(nsub):
            idx = (j * nsub + s - delay[e]) // nsub
            for c in range(3):
                u[c] = hist[e, idx, c] if (idx >= 0 and idx <= j) else 0.0
            K.nb_step(xe, u, p[e], dt, cmd, om, scratch, Wr, A, r)
