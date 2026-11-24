# SPDX-License-Identifier: GPL-3.0-or-later
"""A cascaded multirotor controller for the aerial kernel (engine/kernels/aerial.py), on a batch.

    c = pack(p, gains)                         # [C]: gains (domain config) + the card's allocation (pinv of B)
    u, sat = control(x, sp, p, c)              # [E, R] rotor speed targets; sat: collective cut to keep attitude
    nb_control(x[e], sp[e], p[e], c[e], u[e])  # the same law as a numba scalar; returns sat

Setpoint ``sp`` [E, 7] = x y z (world, m), vx vy (world feed-forward, m/s), yaw (rad), hold_xy (1: horizontal
position loop on, 0: velocity only). The law: position P -> velocity P (the horizontal command limited to
vmax_xy, its acceleration to g tan(tilt_max)) -> desired force m (a + g z) -> desired attitude (z along the
force, x toward the yaw) -> the SO(3) attitude error of Lee, Leok & McClamroch 2010 -> body-rate P with the
gyroscopic feed-forward -> wrench [T, tau] -> per-rotor thrust by the pseudo-inverse of the card's effectiveness
matrix (collective cut first so the torques fit, then each rotor clipped to [0, kf omega_max^2]) -> sqrt(u / kf).

Gains are controller tuning: they come from the domain config (config/domains/drones.yaml ``control``), never
from the card; the allocation and the limits come from the card.
"""

import math

import numpy as np

from engine.kernels import aerial as A

try:
    import numba
    _njit = numba.njit(cache=True)
except ImportError:
    _njit = lambda f: f  # noqa: E731

GAINS = ['kp_xy', 'kp_z', 'kv_xy', 'kv_z', 'tilt_max_deg', 'a_z_max', 'vmax_xy', 'vmax_z', 'k_att', 'k_rate',
         'rate_max']
GLOBAL = ['kp_xy', 'kp_z', 'kv_xy', 'kv_z', 'a_xy_max', 'a_z_max', 'vmax_xy', 'vmax_z',
          'katt0', 'katt1', 'katt2', 'krate0', 'krate1', 'krate2', 'rmax0', 'rmax1', 'rmax2']
ROTOR = ['a0', 'a1', 'a2', 'a3', 'umax', 'kf']
CX = {n: i for i, n in enumerate(GLOBAL)}
CG, CK = len(GLOBAL), len(ROTOR)
SP = 7


def _three(v):
    return [float(a) for a in v] if isinstance(v, (list, tuple)) else [float(v)] * 3


def effectiveness(p):
    """B [4, R]: rotor thrust to (collective along body z, body torque), from one parameter row (padded rotors:
    zero columns)."""
    R = A.rotors_of(p)
    B = np.zeros((4, R))
    for i in range(R):
        b = A.G + A.K * i
        pos, ax = p[b:b + 3], p[b + 3:b + 6]
        spin, km = p[b + A.RX['spin']], p[b + A.RX['km']]
        if p[b + A.RX['kf']] > 0:
            B[:, i] = [ax[2], *(np.cross(pos, ax) - spin * km * ax)]
    return B


def pack(p, gains):
    """The controller row for one parameter row ``p`` and the gains (GAINS; k_att, k_rate, rate_max per axis)."""
    missing = [k for k in GAINS if k not in gains]
    if missing:
        raise KeyError(f'control gains miss {", ".join(missing)} (config/domains/drones.yaml control)')
    g = p[A.GX['g']]
    row = [gains['kp_xy'], gains['kp_z'], gains['kv_xy'], gains['kv_z'],
           g * math.tan(math.radians(gains['tilt_max_deg'])), gains['a_z_max'], gains['vmax_xy'], gains['vmax_z'],
           *_three(gains['k_att']), *_three(gains['k_rate']), *_three(gains['rate_max'])]
    pinv = np.linalg.pinv(effectiveness(p))
    for i in range(A.rotors_of(p)):
        b = A.G + A.K * i
        kf = p[b + A.RX['kf']]
        row += [*pinv[i], kf * p[b + A.RX['omega_max']] ** 2, kf]
    return np.array(row, dtype=np.float64)


def _c(c, k):
    return c[..., CX[k]]


def _rc(c, i, k):
    return c[..., CG + CK * i + ROTOR.index(k)]


def control(x, sp, p, c, xp=np):
    """(u [E, R] rotor speed targets, sat [E] 1 where the collective was cut) for states x and setpoints sp."""
    R = x.shape[-1] - A.BODY
    pos = [x[..., A.POS + j] for j in range(3)]
    vel = [x[..., A.VEL + j] for j in range(3)]
    q = [x[..., A.QUAT + j] for j in range(4)]
    om = [x[..., A.RATE + j] for j in range(3)]
    hold = sp[..., 6]
    vs = [hold * _c(c, 'kp_xy') * (sp[..., j] - pos[j]) + sp[..., 3 + j] for j in range(2)]
    n = xp.sqrt(vs[0] * vs[0] + vs[1] * vs[1])
    k = xp.where(n > _c(c, 'vmax_xy'), _c(c, 'vmax_xy') / xp.where(n > 0, n, 1.0), 1.0)
    vs = [vs[0] * k, vs[1] * k]
    vz = xp.clip(_c(c, 'kp_z') * (sp[..., 2] - pos[2]), -_c(c, 'vmax_z'), _c(c, 'vmax_z'))
    a = [_c(c, 'kv_xy') * (vs[j] - vel[j]) for j in range(2)]
    n = xp.sqrt(a[0] * a[0] + a[1] * a[1])
    k = xp.where(n > _c(c, 'a_xy_max'), _c(c, 'a_xy_max') / xp.where(n > 0, n, 1.0), 1.0)
    az = xp.clip(_c(c, 'kv_z') * (vz - vel[2]), -_c(c, 'a_z_max'), _c(c, 'a_z_max'))
    m, g = p[..., A.GX['m']], p[..., A.GX['g']]
    f = [m * a[0] * k, m * a[1] * k, m * (az + g)]
    Rm = A.matrix(xp, q)
    T = xp.maximum(f[0] * Rm[0][2] + f[1] * Rm[1][2] + f[2] * Rm[2][2], 0.0)
    nf = xp.sqrt(f[0] * f[0] + f[1] * f[1] + f[2] * f[2])
    zd = [f[j] / nf for j in range(3)]
    yaw = sp[..., 5]
    xc = [xp.cos(yaw), xp.sin(yaw), 0.0 * yaw]
    yd = A._cross(zd, xc)
    ny = xp.sqrt(yd[0] * yd[0] + yd[1] * yd[1] + yd[2] * yd[2])
    yd = [v / ny for v in yd]
    xd = A._cross(yd, zd)
    Rd = [[xd[i], yd[i], zd[i]] for i in range(3)]

    def E(i, j):  # (Rd^T R - R^T Rd)[i][j]
        return sum(Rd[k][i] * Rm[k][j] for k in range(3)) - sum(Rm[k][i] * Rd[k][j] for k in range(3))
    eR = [0.5 * E(2, 1), 0.5 * E(0, 2), 0.5 * E(1, 0)]
    I = [[p[..., A.GX[f'I{i}{j}']] for j in range(3)] for i in range(3)]
    alpha = []
    for j in range(3):
        rmax = _c(c, f'rmax{j}')
        ws = xp.clip(-_c(c, f'katt{j}') * eR[j], -rmax, rmax)
        alpha.append(_c(c, f'krate{j}') * (ws - om[j]))
    Iw = [I[i][0] * om[0] + I[i][1] * om[1] + I[i][2] * om[2] for i in range(3)]
    gy = A._cross(om, Iw)
    tau = [I[i][0] * alpha[0] + I[i][1] * alpha[1] + I[i][2] * alpha[2] + gy[i] for i in range(3)]
    att = [_rc(c, i, 'a1') * tau[0] + _rc(c, i, 'a2') * tau[1] + _rc(c, i, 'a3') * tau[2] for i in range(R)]
    cap = T
    for i in range(R):
        a0 = _rc(c, i, 'a0')
        room = xp.maximum(_rc(c, i, 'umax') - att[i], 0.0) / xp.where(a0 > 0, a0, 1.0)
        cap = xp.where(a0 > 0, xp.minimum(cap, room), cap)
    sat = (cap < T) * 1.0
    u = []
    for i in range(R):
        kf = _rc(c, i, 'kf')
        th = xp.clip(_rc(c, i, 'a0') * cap + att[i], 0.0, _rc(c, i, 'umax'))
        u.append(xp.where(kf > 0, xp.sqrt(th / xp.where(kf > 0, kf, 1.0)), 0.0 * th))
    return xp.stack(u, axis=-1), sat


@_njit
def nb_control(x, sp, p, c, u):
    """control() on one env's row: writes u [R]; returns 1.0 where the collective was cut, else 0.0."""
    R = x.shape[0] - A.BODY
    hold = sp[6]
    vsx = hold * c[0] * (sp[0] - x[0]) + sp[3]
    vsy = hold * c[0] * (sp[1] - x[1]) + sp[4]
    n = math.sqrt(vsx * vsx + vsy * vsy)
    if n > c[6]:
        vsx, vsy = vsx * (c[6] / n), vsy * (c[6] / n)
    vz = min(max(c[1] * (sp[2] - x[2]), -c[7]), c[7])
    ax, ay = c[2] * (vsx - x[3]), c[2] * (vsy - x[4])
    n = math.sqrt(ax * ax + ay * ay)
    k = c[4] / n if n > c[4] else 1.0
    az = min(max(c[3] * (vz - x[5]), -c[5]), c[5])
    m, g = p[0], p[1]
    f0, f1, f2 = m * ax * k, m * ay * k, m * (az + g)
    Rm = np.empty((3, 3))
    A.nb_matrix(x[6], x[7], x[8], x[9], Rm)
    T = max(f0 * Rm[0, 2] + f1 * Rm[1, 2] + f2 * Rm[2, 2], 0.0)
    nf = math.sqrt(f0 * f0 + f1 * f1 + f2 * f2)
    z0, z1, z2 = f0 / nf, f1 / nf, f2 / nf
    c0, c1 = math.cos(sp[5]), math.sin(sp[5])
    y0, y1, y2 = z1 * 0.0 - z2 * c1, z2 * c0 - z0 * 0.0, z0 * c1 - z1 * c0
    ny = math.sqrt(y0 * y0 + y1 * y1 + y2 * y2)
    y0, y1, y2 = y0 / ny, y1 / ny, y2 / ny
    Rd = np.empty((3, 3))
    Rd[0, 0], Rd[1, 0], Rd[2, 0] = y1 * z2 - y2 * z1, y2 * z0 - y0 * z2, y0 * z1 - y1 * z0
    Rd[0, 1], Rd[1, 1], Rd[2, 1] = y0, y1, y2
    Rd[0, 2], Rd[1, 2], Rd[2, 2] = z0, z1, z2
    e = np.empty(3)
    for a, (i, j) in enumerate(((2, 1), (0, 2), (1, 0))):
        s1 = Rd[0, i] * Rm[0, j] + Rd[1, i] * Rm[1, j] + Rd[2, i] * Rm[2, j]
        s2 = Rm[0, i] * Rd[0, j] + Rm[1, i] * Rd[1, j] + Rm[2, i] * Rd[2, j]
        e[a] = 0.5 * (s1 - s2)
    al = np.empty(3)
    for j in range(3):
        ws = min(max(-c[8 + j] * e[j], -c[14 + j]), c[14 + j])
        al[j] = c[11 + j] * (ws - x[10 + j])
    o0, o1, o2 = x[10], x[11], x[12]
    Iw0 = p[2] * o0 + p[3] * o1 + p[4] * o2
    Iw1 = p[5] * o0 + p[6] * o1 + p[7] * o2
    Iw2 = p[8] * o0 + p[9] * o1 + p[10] * o2
    t0 = p[2] * al[0] + p[3] * al[1] + p[4] * al[2] + (o1 * Iw2 - o2 * Iw1)
    t1 = p[5] * al[0] + p[6] * al[1] + p[7] * al[2] + (o2 * Iw0 - o0 * Iw2)
    t2 = p[8] * al[0] + p[9] * al[1] + p[10] * al[2] + (o0 * Iw1 - o1 * Iw0)
    cap = T
    for i in range(R):
        b = CG + CK * i
        att = c[b + 1] * t0 + c[b + 2] * t1 + c[b + 3] * t2
        u[i] = att
        if c[b] > 0:
            cap = min(cap, max(c[b + 4] - att, 0.0) / c[b])
    for i in range(R):
        b = CG + CK * i
        th = min(max(c[b] * cap + u[i], 0.0), c[b + 4])
        u[i] = math.sqrt(th / c[b + 5]) if c[b + 5] > 0 else 0.0
    return 1.0 if cap < T else 0.0


@_njit
def nb_fly(x, sp, p, c, dt, n, wind):
    """n plant steps of dt under the controller at a held setpoint, in place on x; returns the steps the
    collective was cut."""
    u = np.empty(x.shape[0] - A.BODY)
    cut = 0.0
    for _ in range(n):
        cut += nb_control(x, sp, p, c, u)
        A.nb_step(x, u, p, dt, wind)
    return cut


def fly(x, sp, p, c, dt, n, wind=None, xp=np):
    """nb_fly on a batch (numpy or jax.numpy): (x, collective-cut steps [E])."""
    cut = 0.0 * x[..., 0]
    for _ in range(n):
        u, sat = control(x, sp, p, c, xp)
        x = A.step(x, u, p, dt, wind, xp)
        cut = cut + sat
    return x, cut
