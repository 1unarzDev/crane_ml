"""The 6-DOF marine kernel: Fossen 2011 eq. 10.1 (``marine6_ref.Simulator``) for E vehicles at once.

    p = pack(profile, dt=0.02, current_n=(0, 0, 0))   # [P] float64 from a vehicle profile (stack rows: [E, P])
    x = init(p[None], pos=(2, 7, 1), rpy=(0, 0, 0))     # [E, S]: pos (NED) | quat b->n (w x y z) | nu (FRD) | thrust
    f = allocate(tau, p)                                # [E, 6] body wrench -> [E, NT] forces (min-norm, scaled)
    x = advance(x, u, p, dt)                            # u [E, 6 + NT]: tau_ext body wrench, then force commands
    nb_advance(x[e], u[e], p[e], dt, w)                 # the same on one env's row (numba; w = workspace(p))

One ``advance`` is ``nsub`` classical RK4 steps of ``dt / nsub`` (fixed per card: the lag's stability bound,
``dt / (2 min tau)``, rounded up), each the reference's ``Simulator.step``: thrusters without a lag snap to the
clipped command, lagged ones integrate ``(clip(f) - T) / tau``; the stage quaternions are normalised keeping
their sign and the step's result is put in the w >= 0 hemisphere. The current is constant in NED (per env, in
``p``). ``step``/``advance`` take numpy or jax.numpy (``xp``); ``nb_*`` are numba scalars over one row and run as
plain Python too. Thrusters past the profile's are padded with zero columns and zero limits.

Per-env randomness is counter-based: ``normal(key, n)`` is a standard normal from (the env's key, a counter),
so env i's draws never depend on the batch or on how many draws other envs made.
"""

import math
from pathlib import Path

import numpy as np

from engine.jit import numba
from engine.kernels import marine6_profile as VP, marine6_ref as REF

NT = 8                                           # thruster slots (a profile has at most NT)
POS, QUAT, NU, THR = 0, 3, 7, 13
S = THR + NT                                     # state columns
U = 6 + NT                                       # input columns: tau_ext (6), then f_cmd (NT)

FIELDS = (('Minv', 36), ('Mrb', 36), ('Ma', 36), ('Dl', 36), ('Dq', 6), ('rg', 3), ('rb', 3), ('W', 1), ('B', 1),
          ('cur', 3), ('T', 6 * NT), ('Tpinv', NT * 6), ('fmin', NT), ('fmax', NT), ('tc', NT), ('db', NT),
          ('nsub', 1), ('nthr', 1))
IX = {}
_o = 0
for _name, _n in FIELDS:
    IX[_name] = _o
    _o += _n
P = _o
MINV, MRB, MA, DL, DQ = (IX[k] for k in ('Minv', 'Mrb', 'Ma', 'Dl', 'Dq'))
RG, RB, WW, BB, CUR = (IX[k] for k in ('rg', 'rb', 'W', 'B', 'cur'))
TT, TPINV, FMIN, FMAX, TC, DB, NSUB, NTHR = (IX[k] for k in ('T', 'Tpinv', 'fmin', 'fmax', 'tc', 'db', 'nsub', 'nthr'))


# ---- parameters ---------------------------------------------------------------------------------------------

def load(profile):
    """(profile dict, VehicleParams, [Thruster]) of a profile JSON path or dict (citations checked)."""
    if isinstance(profile, (str, Path)):
        path = Path(profile)
        data = VP.load(path)
    else:
        data = profile
        if data.get('schema') != VP.SCHEMA:
            raise VP.ProfileError(f'schema must be {VP.SCHEMA}')
    return data, VP.to_params(data), VP.to_thrusters(data)


def deadband_n(data, thrusters):
    """|force command| below which a thruster gives nothing: the force at the profile's
    ``thrusters.deadbandFraction`` of the command range, through its ``thrustCurve`` when that is the odd
    polynomial (``coefficients_V9_V7_V5_V3_V1``), else that fraction of the forward maximum; 0 without one."""
    th = data.get('thrusters') or {}
    node = th.get('deadbandFraction')
    if node is None:
        return [0.0] * len(thrusters)
    frac = float(VP._cited(node, data['sources'], 'thrusters.deadbandFraction'))
    if not 0.0 <= frac < 1.0:
        raise VP.ProfileError(f'thrusters.deadbandFraction must be in [0, 1), not {frac}')
    curve = (th.get('thrustCurve') or {}).get('value')
    if isinstance(curve, dict) and 'coefficients_V9_V7_V5_V3_V1' in curve:
        f = abs(sum(c * frac ** k for c, k in zip(curve['coefficients_V9_V7_V5_V3_V1'], (9, 7, 5, 3, 1))))
        return [f] * len(thrusters)
    return [frac * t.max_forward for t in thrusters]


def substeps(thrusters, dt):
    """RK4 substeps per control period: ``dt / (2 min tau)`` rounded up (the lag's stability bound), at least 1."""
    lags = [t.time_constant for t in thrusters if t.time_constant > 0.0]
    if not lags:
        return 1
    return max(1, int(math.ceil(dt / (2.0 * min(lags)) - 1e-9)))


def pack(profile, dt, current_n=(0.0, 0.0, 0.0), thrust_gain=1.0):
    """One parameter row from a vehicle profile (path or dict): ``pack_params`` of its rigid body, hydrodynamics
    and thrusters, with its deadband and the substeps for a control period ``dt``."""
    data, params, thrusters = load(profile)
    return pack_params(params, thrusters, current_n, thrust_gain, deadband_n(data, thrusters),
                       substeps(thrusters, dt))


def pack_params(params, thrusters, current_n=(0.0, 0.0, 0.0), thrust_gain=1.0, deadband=None, nsub=1):
    """One parameter row from reference objects (``VehicleParams``, [``Thruster``]): the constants computed as
    ``Simulator`` computes them (so they are its bits). ``thrust_gain`` scales the realised thrust columns; the
    allocation keeps the nominal ones."""
    n = len(thrusters)
    if n > NT:
        raise ValueError(f'the kernel has {NT} thruster slots, the profile {n}')
    sim = REF.Simulator(params, current_n=np.asarray(current_n, float), thrusters=thrusters)
    T = np.zeros((6, NT))
    T[:, :n] = sim._T
    Tpinv = np.zeros((NT, 6))
    Tpinv[:n] = np.linalg.pinv(sim._T, rcond=REF.ALLOC_RCOND)
    fmin, fmax, tc, db = (np.zeros(NT) for _ in range(4))
    fmin[:n] = [-t.max_reverse for t in thrusters]
    fmax[:n] = [t.max_forward for t in thrusters]
    tc[:n] = [t.time_constant for t in thrusters]
    db[:n] = deadband if deadband is not None else 0.0
    parts = {'Minv': sim._M_inv, 'Mrb': sim._M_rb, 'Ma': sim._M_a, 'Dl': params.linear_damping,
             'Dq': params.quadratic_damping, 'rg': params.r_g, 'rb': params.r_b, 'W': params.weight,
             'B': params.buoyancy, 'cur': sim.current_n, 'T': (T * float(thrust_gain)).T, 'Tpinv': Tpinv,
             'fmin': fmin, 'fmax': fmax, 'tc': tc, 'db': db, 'nsub': int(nsub), 'nthr': n}
    row = np.concatenate([np.asarray(parts[k], dtype=np.float64).ravel() for k, _ in FIELDS])
    assert row.shape == (P,)
    return row


def with_current(p, current_n):
    """Rows ``p`` [.., P] with the NED current set (per row when ``current_n`` is [.., 3])."""
    p = np.array(p, dtype=np.float64)
    p[..., CUR:CUR + 3] = current_n
    return p


def init(p, pos=(0.0, 0.0, 0.0), rpy=(0.0, 0.0, 0.0)):
    """[E, S] at rest: the pose(s) given (``rpy`` ZYX roll, pitch, yaw), zero velocity and thrust."""
    p = np.atleast_2d(p)
    E = p.shape[0]
    x = np.zeros((E, S))
    x[:, POS:POS + 3] = np.broadcast_to(np.asarray(pos, float), (E, 3))
    rpy = np.broadcast_to(np.asarray(rpy, float), (E, 3))
    x[:, QUAT:QUAT + 4] = [REF.euler_to_quat(*r) for r in rpy]
    return x


def workspace(p=None):
    """The scratch rows ``nb_step`` and ``nb_advance`` need (one per thread)."""
    return np.zeros((6, S))


# ---- the reference's equations on arrays (numpy or jax.numpy) ------------------------------------------------

def _m(p, at, rows=6, cols=6):
    return p[..., at:at + rows * cols].reshape(p.shape[:-1] + (rows, cols))


def _mv(xp, M, v):
    return xp.einsum('...ij,...j->...i', M, v)


def _cross(xp, a, b):
    return xp.stack([a[..., 1] * b[..., 2] - a[..., 2] * b[..., 1], a[..., 2] * b[..., 0] - a[..., 0] * b[..., 2],
                     a[..., 0] * b[..., 1] - a[..., 1] * b[..., 0]], axis=-1)


def rot(xp, q):
    """R_b^n [..., 3, 3] of quaternions [..., 4] (Fossen eq. 2.69)."""
    w, x, y, z = (q[..., k] for k in range(4))
    return xp.stack([xp.stack([1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)], -1),
                     xp.stack([2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)], -1),
                     xp.stack([2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)], -1)], -2)


def _c_nu(xp, M, nu):
    """C(nu) nu for a symmetric M (Fossen Thm 3.2): [nu2 x a, nu1 x a + nu2 x b]."""
    a = _mv(xp, M[..., :3, :3], nu[..., :3]) + _mv(xp, M[..., :3, 3:], nu[..., 3:])
    b = _mv(xp, M[..., 3:, :3], nu[..., :3]) + _mv(xp, M[..., 3:, 3:], nu[..., 3:])
    return xp.concatenate([_cross(xp, nu[..., 3:], a), _cross(xp, nu[..., :3], a) + _cross(xp, nu[..., 3:], b)], -1)


def deriv(xp, x, f_cmd, tau_ext, p):
    """The state's time derivative (Simulator._deriv) with the force commands already clipped."""
    q, nu, thr = x[..., QUAT:QUAT + 4], x[..., NU:NU + 6], x[..., THR:THR + NT]
    R = rot(xp, q)
    tau = tau_ext + _mv(xp, _m(p, TT, NT, 6).swapaxes(-1, -2), thr)
    tc = p[..., TC:TC + NT]
    lag = tc > 0.0
    fdot = xp.where(lag, (f_cmd - thr) / xp.where(lag, tc, 1.0), 0.0 * thr)
    posdot = _mv(xp, R, nu[..., :3])
    w, qx, qy, qz = (q[..., k] for k in range(4))
    pr, qr, rr = nu[..., 3], nu[..., 4], nu[..., 5]
    qdot = 0.5 * xp.stack([-qx * pr - qy * qr - qz * rr, w * pr - qz * qr + qy * rr,
                           qz * pr + w * qr - qx * rr, -qy * pr + qx * qr + w * rr], -1)
    v_c_b = _mv(xp, R.swapaxes(-1, -2), p[..., CUR:CUR + 3])
    zero = 0.0 * v_c_b
    nu_r = nu - xp.concatenate([v_c_b, zero], -1)
    nudot_c = xp.concatenate([-_cross(xp, nu[..., 3:], v_c_b), zero], -1)
    Ma = _m(p, MA)
    rhs = tau - _c_nu(xp, _m(p, MRB), nu) - _c_nu(xp, Ma, nu_r)
    rhs = rhs - (_mv(xp, _m(p, DL), nu_r) + p[..., DQ:DQ + 6] * xp.abs(nu_r) * nu_r)
    ez = R[..., 2, :]
    f_g = p[..., WW:WW + 1] * ez
    f_b = -p[..., BB:BB + 1] * ez
    g = -xp.concatenate([f_g + f_b, _cross(xp, p[..., RG:RG + 3], f_g) + _cross(xp, p[..., RB:RB + 3], f_b)], -1)
    rhs = rhs - g + _mv(xp, Ma, nudot_c)
    return posdot, qdot, _mv(xp, _m(p, MINV), rhs), fdot


def _unit(xp, q):
    return q / xp.sqrt(xp.sum(q * q, axis=-1, keepdims=True))


def step(x, u, p, h, xp=np):
    """One RK4 step of ``h`` [E] or scalar (Simulator.step) on [E, S] states and [E, U] inputs."""
    tau_ext, f = u[..., :6], u[..., 6:6 + NT]
    f = xp.minimum(xp.maximum(f, p[..., FMIN:FMIN + NT]), p[..., FMAX:FMAX + NT])
    lag = p[..., TC:TC + NT] > 0.0
    x = xp.concatenate([x[..., :THR], xp.where(lag, x[..., THR:], f)], -1)
    h = xp.asarray(h)[..., None] if xp.ndim(h) else h

    def add(k, hh, canonical=False):
        qn = _unit(xp, x[..., QUAT:QUAT + 4] + hh * k[1])
        if canonical:
            qn = xp.where(qn[..., :1] >= 0.0, qn, -qn)
        return xp.concatenate([x[..., :3] + hh * k[0], qn, x[..., NU:NU + 6] + hh * k[2],
                               x[..., THR:] + hh * k[3]], -1)

    k1 = deriv(xp, x, f, tau_ext, p)
    k2 = deriv(xp, add(k1, h / 2), f, tau_ext, p)
    k3 = deriv(xp, add(k2, h / 2), f, tau_ext, p)
    k4 = deriv(xp, add(k3, h), f, tau_ext, p)
    comb = [(a + 2 * b + 2 * c + d) / 6.0 for a, b, c, d in zip(k1, k2, k3, k4)]
    return add(comb, h, canonical=True)


def advance(x, u, p, dt, xp=np, nmax=None):
    """One control period: each env's ``nsub`` RK4 steps of ``dt / nsub`` holding ``u`` (``nmax``: the largest
    ``nsub`` in the batch; give it under jax.jit)."""
    n = p[..., NSUB]
    nmax = int(np.max(np.asarray(n))) if nmax is None else int(nmax)
    h = dt / n
    for k in range(nmax):
        y = step(x, u, p, h, xp)
        x = y if nmax == 1 else xp.where((k < n)[..., None], y, x)
    return x


def allocate(tau, p, xp=np):
    """[E, 6] wrenches -> [E, NT] forces: least-norm, then one uniform scale into the limits (REF.allocate)."""
    f = _mv(xp, _m(p, TPINV, NT, 6), tau)
    lo, hi = p[..., FMIN:FMIN + NT], p[..., FMAX:FMAX + NT]
    up = (f > hi) & (hi > 0)
    down = (f < lo) & (lo < 0)
    ratio = xp.where(up, hi / xp.where(up, f, 1.0), xp.where(down, lo / xp.where(down, f, 1.0), 1.0))
    return f * xp.min(ratio, axis=-1, keepdims=True)


# ---- the same map as numba scalars over one env's row (also plain Python) ------------------------------------

_jit = numba.njit(cache=True)


@_jit
def nb_deriv(x, f, tau_ext, p, k):
    """k[:] = d/dt x (one env)."""
    w, qx, qy, qz = x[3], x[4], x[5], x[6]
    r00, r01, r02 = 1 - 2 * (qy * qy + qz * qz), 2 * (qx * qy - qz * w), 2 * (qx * qz + qy * w)
    r10, r11, r12 = 2 * (qx * qy + qz * w), 1 - 2 * (qx * qx + qz * qz), 2 * (qy * qz - qx * w)
    r20, r21, r22 = 2 * (qx * qz - qy * w), 2 * (qy * qz + qx * w), 1 - 2 * (qx * qx + qy * qy)
    u0, u1, u2, pr, qr, rr = x[7], x[8], x[9], x[10], x[11], x[12]
    k[0] = r00 * u0 + r01 * u1 + r02 * u2
    k[1] = r10 * u0 + r11 * u1 + r12 * u2
    k[2] = r20 * u0 + r21 * u1 + r22 * u2
    k[3] = 0.5 * (-qx * pr - qy * qr - qz * rr)
    k[4] = 0.5 * (w * pr - qz * qr + qy * rr)
    k[5] = 0.5 * (qz * pr + w * qr - qx * rr)
    k[6] = 0.5 * (-qy * pr + qx * qr + w * rr)
    cx, cy, cz = p[CUR], p[CUR + 1], p[CUR + 2]
    vc0 = r00 * cx + r10 * cy + r20 * cz
    vc1 = r01 * cx + r11 * cy + r21 * cz
    vc2 = r02 * cx + r12 * cy + r22 * cz
    nu = np.empty(6)
    nr = np.empty(6)
    for i in range(6):
        nu[i] = x[NU + i]
        nr[i] = x[NU + i]
    nr[0] -= vc0
    nr[1] -= vc1
    nr[2] -= vc2
    rhs = np.empty(6)
    for i in range(6):
        t = tau_ext[i]
        for j in range(NT):
            t += p[TT + 6 * j + i] * x[THR + j]
        rhs[i] = t
    for m, v in ((MRB, nu), (MA, nr)):
        a0 = 0.0
        a1 = 0.0
        a2 = 0.0
        b0 = 0.0
        b1 = 0.0
        b2 = 0.0
        for j in range(6):
            a0 += p[m + j] * v[j]
            a1 += p[m + 6 + j] * v[j]
            a2 += p[m + 12 + j] * v[j]
            b0 += p[m + 18 + j] * v[j]
            b1 += p[m + 24 + j] * v[j]
            b2 += p[m + 30 + j] * v[j]
        n0, n1, n2, n3, n4, n5 = v[0], v[1], v[2], v[3], v[4], v[5]
        rhs[0] -= n4 * a2 - n5 * a1
        rhs[1] -= n5 * a0 - n3 * a2
        rhs[2] -= n3 * a1 - n4 * a0
        rhs[3] -= (n1 * a2 - n2 * a1) + (n4 * b2 - n5 * b1)
        rhs[4] -= (n2 * a0 - n0 * a2) + (n5 * b0 - n3 * b2)
        rhs[5] -= (n0 * a1 - n1 * a0) + (n3 * b1 - n4 * b0)
    for i in range(6):
        d = 0.0
        for j in range(6):
            d += p[DL + 6 * i + j] * nr[j]
        rhs[i] -= d + p[DQ + i] * abs(nr[i]) * nr[i]
    W, B = p[WW], p[BB]
    fg0, fg1, fg2 = W * r20, W * r21, W * r22
    fb0, fb1, fb2 = -B * r20, -B * r21, -B * r22
    gx, gy, gz = p[RG], p[RG + 1], p[RG + 2]
    bx, by, bz = p[RB], p[RB + 1], p[RB + 2]
    rhs[0] += fg0 + fb0
    rhs[1] += fg1 + fb1
    rhs[2] += fg2 + fb2
    rhs[3] += (gy * fg2 - gz * fg1) + (by * fb2 - bz * fb1)
    rhs[4] += (gz * fg0 - gx * fg2) + (bz * fb0 - bx * fb2)
    rhs[5] += (gx * fg1 - gy * fg0) + (bx * fb1 - by * fb0)
    dc0 = -(qr * vc2 - rr * vc1)
    dc1 = -(rr * vc0 - pr * vc2)
    dc2 = -(pr * vc1 - qr * vc0)
    for i in range(6):
        rhs[i] += p[MA + 6 * i] * dc0 + p[MA + 6 * i + 1] * dc1 + p[MA + 6 * i + 2] * dc2
    for i in range(6):
        a = 0.0
        for j in range(6):
            a += p[MINV + 6 * i + j] * rhs[j]
        k[NU + i] = a
    for j in range(NT):
        tc = p[TC + j]
        k[THR + j] = (f[j] - x[THR + j]) / tc if tc > 0.0 else 0.0


@_jit
def nb_step(x, u, p, h, w):
    """One RK4 step of ``h`` on one env's row ``x`` (in place). ``w``: workspace(p)."""
    f = w[5, :NT]
    for j in range(NT):
        f[j] = min(max(u[6 + j], p[FMIN + j]), p[FMAX + j])
        if not p[TC + j] > 0.0:
            x[THR + j] = f[j]
    tau = u[:6]
    k1, k2, k3, k4, y = w[0], w[1], w[2], w[3], w[4]
    nb_deriv(x, f, tau, p, k1)
    _stage(x, k1, h / 2, y, False)
    nb_deriv(y, f, tau, p, k2)
    _stage(x, k2, h / 2, y, False)
    nb_deriv(y, f, tau, p, k3)
    _stage(x, k3, h, y, False)
    nb_deriv(y, f, tau, p, k4)
    for i in range(S):
        k1[i] = (k1[i] + 2 * k2[i] + 2 * k3[i] + k4[i]) / 6.0
    _stage(x, k1, h, y, True)
    for i in range(S):
        x[i] = y[i]


@_jit
def _stage(x, k, h, y, canonical):
    for i in range(S):
        y[i] = x[i] + h * k[i]
    n = math.sqrt(y[3] * y[3] + y[4] * y[4] + y[5] * y[5] + y[6] * y[6])
    s = 1.0
    if canonical and y[3] / n < 0.0:
        s = -1.0
    for i in range(3, 7):
        y[i] = s * (y[i] / n)


@_jit
def nb_advance(x, u, p, dt, w):
    """One control period on one env (its ``nsub`` RK4 steps of ``dt / nsub``)."""
    n = int(p[NSUB])
    h = dt / p[NSUB]
    for _ in range(n):
        nb_step(x, u, p, h, w)


@_jit
def nb_allocate(tau, p, f):
    """f[:] = REF.allocate of the wrench ``tau`` (one env)."""
    scale = 1.0
    for i in range(NT):
        a = 0.0
        for j in range(6):
            a += p[TPINV + 6 * i + j] * tau[j]
        f[i] = a
        lo, hi = p[FMIN + i], p[FMAX + i]
        if a > hi and hi > 0:
            scale = min(scale, hi / a)
        elif a < lo and lo < 0:
            scale = min(scale, lo / a)
    for i in range(NT):
        f[i] *= scale


# ---- counter-based per-env draws ---------------------------------------------------------------------------

_GOLD = np.uint64(0x9E3779B97F4A7C15)
_M1, _M2 = np.uint64(0xBF58476D1CE4E5B9), np.uint64(0x94D049BB133111EB)
_S30, _S27, _S31, _S11 = np.uint64(30), np.uint64(27), np.uint64(31), np.uint64(11)


@_jit
def _mix(key, n):
    z = key + n * _GOLD
    z = (z ^ (z >> _S30)) * _M1
    z = (z ^ (z >> _S27)) * _M2
    return z ^ (z >> _S31)


@_jit
def normal(key, n):
    """A standard normal from (uint64 ``key``, counter ``n``): splitmix64 of two counters, Box-Muller."""
    c = np.uint64(2) * np.uint64(n)
    u1 = 1.0 - float(_mix(key, c) >> _S11) * (1.0 / 9007199254740992.0)      # (0, 1]
    u2 = float(_mix(key, c + np.uint64(1)) >> _S11) * (1.0 / 9007199254740992.0)
    return math.sqrt(-2.0 * math.log(u1)) * math.cos(2.0 * math.pi * u2)


def key(seed, name):
    """The uint64 key of a seed's named stream (fastsim.identity.derive_seed)."""
    from fastsim.identity import derive_seed
    return np.uint64(derive_seed(int(seed), 'engine', name) & 0xFFFFFFFFFFFFFFFF)
