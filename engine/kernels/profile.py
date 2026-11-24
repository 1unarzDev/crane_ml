"""The boat's T1 kernel: ``ProfilePlant``'s discrete map (Tools/Characterization/plant.py, the C#
``MarinePlanarModel`` twin) on a batch, and ``ProfileBoat``'s velocity loop and allocation.

    p = pack(profile, dt)                    # [P] float64 (stack rows for [E, P])
    x = init(p[None], pose=(0, 0, 0))        # [E, S]: x y psi vx vy r | T_i | ring index_i | rings
    x = step(x, pwm, p, dt)                  # one tick: PWM thrusters -> symplectic Euler, implicit damping
    pwm, cmd = loop_pwm(x, cmd, target, p, q, dt)   # ProfileBoat.pwm_for: smoother, FF + P, pinv, PWM

Every branch of the twin is a ``where``; every expression keeps the twin's order of operations, so the
float64 numpy path equals the twin bit for bit (engine/tests). ``nb_step``/``nb_loop_pwm`` are the same
map as numba scalars over one env's row. A thruster's dead time is a ring buffer of ``n_d = round(deadTime
/ dt)`` past commands (read the oldest, write the newest there), its lag the twin's first-order gain.
"""

import json
import math

import numba
import numpy as np

from engine import exact, nbexact

THRUSTERS = 4
AXES = ('surge', 'sway', 'yaw')
PWM_NEUTRAL, PWM_MIN, PWM_MAX, PWM_SPAN = 1500.0, 1100.0, 1900.0, 400.0

# ---- parameters [P] ----------------------------------------------------------------------------
HULL = ['m', 'iz', 'a11', 'a22', 'a26', 'a66', 'cx', 'cy']
DAMP = [f'{a}.{k}' for a in AXES for k in ('d1Pos', 'd1Neg', 'd2Pos', 'd2Neg')]
THRUST = ['cos', 'sin', 'x', 'y', 'offsetUs', 'deadbandUs', 'kForward', 'kReverse', 'volt', 'lag', 'n_d']
NAMES = HULL + DAMP + [f't{i}.{k}' for i in range(THRUSTERS) for k in THRUST]
IX = {n: i for i, n in enumerate(NAMES)}
P = len(NAMES)
_T0, _NT = len(HULL) + len(DAMP), len(THRUST)
# ---- velocity loop parameters [Q] ----------------------------------------------------------------
LOOP = (['meff0', 'meff1', 'meff2', 'kp0', 'kp1', 'kp2', 'up0', 'up1', 'up2', 'down0', 'down1', 'down2']
        + [f'B{i}{j}' for i in range(THRUSTERS) for j in range(3)] + ['smooth'])
LX = {n: i for i, n in enumerate(LOOP)}
Q = len(LOOP)
# ---- state [S] -----------------------------------------------------------------------------------
BODY = 6                                         # x y psi vx vy r (world velocity, unwrapped heading)
LAGS = BODY                                      # T_i, the lagged thrust of thruster i
IDX = BODY + THRUSTERS                           # ring index of thruster i
RINGS = BODY + 2 * THRUSTERS                     # then THRUSTERS rings of n_max slots each


def validate(profile):
    for t in profile['thrusters']:
        if t['kappa'] != 0.0:
            raise ValueError(f"{t['name']}: kappa must be 0")
    if len(profile['thrusters']) != THRUSTERS:
        raise ValueError(f'the kernel has {THRUSTERS} thrusters, the profile {len(profile["thrusters"])}')


def pack(profile, dt, current=(0.0, 0.0)):
    """The twin's numbers for a tick of ``dt`` as one parameter row (constants computed as the
    twin computes them, so they are its bits)."""
    if isinstance(profile, (str, bytes)) or hasattr(profile, 'read_text'):
        profile = json.loads(open(profile).read() if not hasattr(profile, 'read_text') else profile.read_text())
    validate(profile)
    am = profile['addedMass']
    row = [float(profile['mass']), float(profile['izz']), am['a11'], am['a22'], am['a26'], am['a66'],
           float(current[0]), float(current[1])]
    for a in AXES:
        row += [profile[a][k] for k in ('d1Pos', 'd1Neg', 'd2Pos', 'd2Neg')]
    for t in profile['thrusters']:
        alpha = math.radians(t['alphaDeg'])
        tau = t['tauT']
        row += [math.cos(alpha), math.sin(alpha), t['x'], t['y'], t['offsetUs'], t['deadbandUs'], t['kForward'],
                t['kReverse'], (t['vRef'] / t['vRef']) ** t['nV'], 1.0 if tau == 0.0 else 1.0 - math.exp(-dt / tau),
                float(int(round(t['deadTime'] / dt)))]
    return np.array(row, dtype=np.float64)


def pack_loop(profile, kp=(2.0, 2.0, 2.0), accel=(0.0,) * 3, decel=(0.0,) * 3, smooth=True):
    """ProfileBoat's loop numbers: effective mass, gains, the smoother's rates and the
    pseudo-inverse of the thruster columns (np.linalg.pinv, as ProfileBoat takes it)."""
    am = profile['addedMass']
    meff = (profile['mass'] + am['a11'], profile['mass'] + am['a22'], profile['izz'] + am['a66'])
    cols = []
    for t in profile['thrusters']:
        a = math.radians(t['alphaDeg'])
        cols.append((math.cos(a), math.sin(a), t['x'] * math.sin(a) - t['y'] * math.cos(a)))
    B = np.linalg.pinv(np.array(cols).T)
    return np.array([*meff, *(float(k) for k in kp), *accel, *decel, *B.ravel(), 1.0 if smooth else 0.0])


def n_max(p):
    p = np.atleast_2d(p)
    return int(max(1, p[:, [_T0 + i * _NT + THRUST.index('n_d') for i in range(THRUSTERS)]].max()))


def size(p):
    return RINGS + THRUSTERS * n_max(p)


def init(p, pose=(0.0, 0.0, 0.0), vel=(0.0, 0.0, 0.0), S=None):
    """[E, S] states at rest-state thrusters (ProfileBoat.__init__: world velocity from the body one)."""
    p = np.atleast_2d(p)
    x = np.zeros((p.shape[0], S or size(p)))
    X, Y, yaw = pose
    c, s = math.cos(yaw), math.sin(yaw)
    x[:, :BODY] = (X, Y, yaw, c * vel[0] - s * vel[1], s * vel[0] + c * vel[1], vel[2])
    return x


def _t(p, i, k):
    return p[..., _T0 + i * _NT + THRUST.index(k)]


def _p(p, k):
    return p[..., IX[k]]


# ---- the twin's pieces as arrays (numpy or jax.numpy) ----------------------------------------------

def _damping(xp, p, axis, s):
    d1p, d1n = _p(p, f'{axis}.d1Pos'), _p(p, f'{axis}.d1Neg')
    d2p, d2n = _p(p, f'{axis}.d2Pos'), _p(p, f'{axis}.d2Neg')
    a = xp.abs(s)
    pos, neg = s > 0.0, s < 0.0
    d1 = xp.where(pos, d1p, d1n)
    d2 = xp.where(pos, d2p, d2n)
    D = xp.where(pos | neg, (d1 + d2 * a) * s, 0.0 * s)
    J = xp.where(pos | neg, d1 + 2.0 * d2 * a, xp.maximum(d1p, d1n))
    return D, J


def _step_term(xp, meff, J, dt):
    x = dt * J / meff
    small = x < 1e-4
    safe = xp.where(x > 0.0, x, 1.0)
    big = meff * (safe / -xp.expm1(-safe) - 1.0)
    tiny = meff * (x / 2.0 + x * x / 12.0)
    return xp.where(x > 0.0, xp.where(small, tiny, big), 0.0 * x)


def thrust(xp, x, u, p, nm):
    """The thrusters' tick: (X, Y, N) and the new lag, ring index and ring columns."""
    X = Y = N = 0.0
    lags, idxs, rings = [], [], []
    for i in range(THRUSTERS):
        pw = u[..., i]
        pw = xp.where(xp.isfinite(pw), pw, PWM_NEUTRAL)
        pw = xp.minimum(xp.maximum(pw, PWM_MIN), PWM_MAX)
        e = pw - PWM_NEUTRAL - _t(p, i, 'offsetUs')
        q = xp.where(xp.abs(e) <= _t(p, i, 'deadbandUs'), 0.0 * e,
                     xp.minimum(xp.maximum(e / PWM_SPAN, -1.0), 1.0))
        nd = _t(p, i, 'n_d')
        idx = x[..., IDX + i]
        ring = x[..., RINGS + i * nm:RINGS + (i + 1) * nm]
        slot = xp.arange(nm)
        at = slot == idx[..., None]
        old = xp.sum(xp.where(at, ring, 0.0 * ring), axis=-1)
        delayed = nd > 0
        qd = xp.where(delayed, old, q)
        rings.append(xp.where(delayed[..., None] & at, q[..., None], ring))
        idxs.append(xp.where(delayed, xp.remainder(idx + 1.0, xp.where(delayed, nd, 1.0)), idx))
        kF, kR = _t(p, i, 'kForward'), _t(p, i, 'kReverse')
        t_ss = xp.where(qd > 0.0, kF * qd * qd, xp.where(qd < 0.0, -kR * qd * qd, 0.0 * qd))
        t_ss = t_ss * _t(p, i, 'volt')
        T = x[..., LAGS + i]
        T = T + _t(p, i, 'lag') * (t_ss - T)
        lags.append(T)
        F = T
        fx, fy = F * _t(p, i, 'cos'), F * _t(p, i, 'sin')
        X = X + fx
        Y = Y + fy
        N = N + (_t(p, i, 'x') * fy - _t(p, i, 'y') * fx)
    return (X, Y, N), lags, idxs, rings


def hull(xp, x, tau, p, dt):
    """ProfilePlant.step_wrench: the new (x, y, psi, vx, vy, r)."""
    m, iz = _p(p, 'm'), _p(p, 'iz')
    a11, a22, a26, a66 = _p(p, 'a11'), _p(p, 'a22'), _p(p, 'a26'), _p(p, 'a66')
    X, Y, psi, vx, vy, r = (x[..., k] for k in range(BODY))
    c, s = xp.cos(psi), xp.sin(psi)
    u, v = c * vx + s * vy, -s * vx + c * vy
    cx, cy = _p(p, 'cx'), _p(p, 'cy')
    uc, vc = c * cx + s * cy, -s * cx + c * cy
    ur, vr = u - uc, v - vc
    ndc0, ndc1 = r * vc, -r * uc
    D0, J0 = _damping(xp, p, 'surge', ur)
    D1, J1 = _damping(xp, p, 'sway', vr)
    D2, J2 = _damping(xp, p, 'yaw', r)
    w = a22 * vr + a26 * r
    ca0, ca1, ca2 = -w * r, a11 * ur * r, w * ur - a11 * ur * vr
    th0 = tau[0] + a11 * ndc0 - ca0 - D0
    th1 = tau[1] + a22 * ndc1 - ca1 - D1
    th2 = tau[2] + a26 * ndc1 - ca2 - D2
    x0, x1, x2 = th0 + m * v * r, th1 - m * u * r, th2
    J0 = _step_term(xp, m + a11, J0, dt)
    J1 = _step_term(xp, m + a22, J1, dt)
    J2 = _step_term(xp, iz + a66, J2, dt)
    m11 = m + a11 + J0
    m22 = m + a22 + J1
    m66 = iz + a66 + J2
    acc0 = x0 / m11
    det = m22 * m66 - a26 * a26
    acc1 = (m66 * x1 - a26 * x2) / det
    acc2 = (m22 * x2 - a26 * x1) / det
    fu0 = th0 - (a11 + J0) * acc0
    fu1 = th1 - ((a22 + J1) * acc1 + a26 * acc2)
    fu2 = th2 - (a26 * acc1 + (a66 + J2) * acc2)
    fx, fy = c * fu0 - s * fu1, s * fu0 + c * fu1
    vx = vx + dt * fx / m
    vy = vy + dt * fy / m
    r = r + dt * fu2 / iz
    return X + dt * vx, Y + dt * vy, psi + dt * r, vx, vy, r


def step(x, u, p, dt, xp=np):
    """One tick of ProfilePlant.step_pwm on [E, S] states, [E, 4] pulses, [E, P] params."""
    nm = (x.shape[-1] - RINGS) // THRUSTERS
    tau, lags, idxs, rings = thrust(xp, x, u, p, nm)
    body = hull(xp, x, tau, p, dt)
    return xp.concatenate([xp.stack(list(body) + lags + idxs, axis=-1)] + rings, axis=-1)


def body_velocity(xp, x):
    c, s = xp.cos(x[..., 2]), xp.sin(x[..., 2])
    return c * x[..., 3] + s * x[..., 4], -s * x[..., 3] + c * x[..., 4], x[..., 5]


def loop_pwm(x, cmd, target, p, q, dt, xp=np, blas=True):
    """ProfileBoat.pwm_for on a batch: (pwm [E, 4], the smoothed command [E, 3]). ``p`` holds the
    hull's damping and the thrusters, ``q`` the loop (pack_loop). ``blas``: the pseudo-inverse
    product in OpenBLAS's order (the float64 reference, bit for bit); else a plain sum."""
    smooth = q[..., LX['smooth']] > 0
    want = []
    for j in range(3):
        v, w = cmd[..., j], target[..., j]
        up, down = q[..., LX[f'up{j}']], q[..., LX[f'down{j}']]
        rate = xp.where((xp.abs(w) > xp.abs(v)) & (w * v >= 0), up, -down)
        want.append(xp.where(smooth, v + xp.maximum(-rate * dt, xp.minimum(rate * dt, w - v)), w))
    nu = body_velocity(xp, x)
    tau = []
    for j, axis in enumerate(AXES):
        w = want[j]
        pos = w >= 0
        d1 = xp.where(pos, _p(p, f'{axis}.d1Pos'), _p(p, f'{axis}.d1Neg'))
        d2 = xp.where(pos, _p(p, f'{axis}.d2Pos'), _p(p, f'{axis}.d2Neg'))
        tau.append((d1 + d2 * xp.abs(w)) * w + q[..., LX[f'kp{j}']] * q[..., LX[f'meff{j}']] * (w - nu[j]))
    B = [[q[..., LX[f'B{i}{j}']] for j in range(3)] for i in range(THRUSTERS)]
    if blas:
        f = [exact.fma(B[i][2], tau[2], exact.fma(B[i][0], tau[0], B[i][1] * tau[1], xp), xp) for i in range(THRUSTERS)]
    else:
        f = [B[i][0] * tau[0] + B[i][1] * tau[1] + B[i][2] * tau[2] for i in range(THRUSTERS)]
    caps = [xp.where(f[i] >= 0, _t(p, i, 'kForward'), _t(p, i, 'kReverse')) for i in range(THRUSTERS)]
    worst = xp.abs(f[0]) / caps[0]
    for i in range(1, THRUSTERS):
        r = xp.abs(f[i]) / caps[i]
        worst = xp.where(r > worst, r, worst)
    scale = xp.where(worst > 1.0, worst, 1.0)
    pwm = [PWM_NEUTRAL + _t(p, i, 'offsetUs') + PWM_SPAN * xp.copysign(xp.sqrt(xp.abs(f[i]) / scale / caps[i]), f[i])
           for i in range(THRUSTERS)]
    return xp.stack(pwm, axis=-1), xp.stack(want, axis=-1)


# ---- the same map as numba scalars over one env's row -------------------------------------------------

_C = THRUST.index('cos')
_SN, _X, _Y, _OFF, _DB, _KF, _KR, _V, _LAG, _ND = (THRUST.index(k) for k in (
    'sin', 'x', 'y', 'offsetUs', 'deadbandUs', 'kForward', 'kReverse', 'volt', 'lag', 'n_d'))
_D = len(HULL)  # damping block: axis j at _D + 4 j: d1Pos, d1Neg, d2Pos, d2Neg
_jit = numba.njit(cache=True, inline='always')


@_jit
def nb_damping(p, j, s):
    b = _D + 4 * j
    if s > 0.0:
        d1, d2 = p[b], p[b + 2]
    elif s < 0.0:
        d1, d2 = p[b + 1], p[b + 3]
    else:
        return 0.0, max(p[b], p[b + 1])
    a = abs(s)
    return (d1 + d2 * a) * s, d1 + 2.0 * d2 * a


@_jit
def nb_step_term(meff, J, dt):
    x = dt * J / meff
    if not x > 0.0:
        return 0.0
    if x < 1e-4:
        return meff * (x / 2.0 + x * x / 12.0)
    return meff * (x / -math.expm1(-x) - 1.0)


@_jit
def nb_step(x, u, p, dt):
    """One tick on one env's state row ``x`` (in place)."""
    nb_step_cs(x, u, p, dt, math.cos(x[2]), math.sin(x[2]))


@_jit
def nb_step_cs(x, u, p, dt, c, s):
    """nb_step with cos and sin of the heading x[2] given (the velocity loop has them)."""
    nm = (x.shape[0] - RINGS) // THRUSTERS
    X = 0.0
    Y = 0.0
    N = 0.0
    for i in range(THRUSTERS):
        b = _T0 + i * _NT
        pw = u[i]
        if not math.isfinite(pw):
            pw = PWM_NEUTRAL
        pw = min(max(pw, PWM_MIN), PWM_MAX)
        e = pw - PWM_NEUTRAL - p[b + _OFF]
        q = 0.0 if abs(e) <= p[b + _DB] else min(max(e / PWM_SPAN, -1.0), 1.0)
        nd = int(p[b + _ND])
        if nd == 0:
            qd = q
        else:
            k = int(x[IDX + i])
            qd = x[RINGS + i * nm + k]
            x[RINGS + i * nm + k] = q
            x[IDX + i] = float((k + 1) % nd)
        if qd > 0.0:
            t_ss = p[b + _KF] * qd * qd
        elif qd < 0.0:
            t_ss = -p[b + _KR] * qd * qd
        else:
            t_ss = 0.0
        t_ss *= p[b + _V]
        T = x[LAGS + i]
        T += p[b + _LAG] * (t_ss - T)
        x[LAGS + i] = T
        fx, fy = T * p[b + _C], T * p[b + _SN]
        X += fx
        Y += fy
        N += p[b + _X] * fy - p[b + _Y] * fx
    m, iz, a11, a22, a26, a66, cx, cy = p[0], p[1], p[2], p[3], p[4], p[5], p[6], p[7]
    px, py, psi, vx, vy, r = x[0], x[1], x[2], x[3], x[4], x[5]
    uu, v = c * vx + s * vy, -s * vx + c * vy
    uc, vc = c * cx + s * cy, -s * cx + c * cy
    ur, vr = uu - uc, v - vc
    ndc0, ndc1 = r * vc, -r * uc
    D0, J0 = nb_damping(p, 0, ur)
    D1, J1 = nb_damping(p, 1, vr)
    D2, J2 = nb_damping(p, 2, r)
    w = a22 * vr + a26 * r
    ca0, ca1, ca2 = -w * r, a11 * ur * r, w * ur - a11 * ur * vr
    th0 = X + a11 * ndc0 - ca0 - D0
    th1 = Y + a22 * ndc1 - ca1 - D1
    th2 = N + a26 * ndc1 - ca2 - D2
    x0, x1, x2 = th0 + m * v * r, th1 - m * uu * r, th2
    J0 = nb_step_term(m + a11, J0, dt)
    J1 = nb_step_term(m + a22, J1, dt)
    J2 = nb_step_term(iz + a66, J2, dt)
    m11 = m + a11 + J0
    m22 = m + a22 + J1
    m66 = iz + a66 + J2
    acc0 = x0 / m11
    det = m22 * m66 - a26 * a26
    acc1 = (m66 * x1 - a26 * x2) / det
    acc2 = (m22 * x2 - a26 * x1) / det
    fu0 = th0 - (a11 + J0) * acc0
    fu1 = th1 - ((a22 + J1) * acc1 + a26 * acc2)
    fu2 = th2 - (a26 * acc1 + (a66 + J2) * acc2)
    fx, fy = c * fu0 - s * fu1, s * fu0 + c * fu1
    vx += dt * fx / m
    vy += dt * fy / m
    r += dt * fu2 / iz
    x[0] = px + dt * vx
    x[1] = py + dt * vy
    x[2] = psi + dt * r
    x[3] = vx
    x[4] = vy
    x[5] = r


@_jit
def nb_loop_pwm(x, cmd, target, p, q, dt, pwm, blas):
    """ProfileBoat.pwm_for on one env: writes ``pwm`` [4] and updates ``cmd`` [3] in place; returns
    cos and sin of the heading."""
    for j in range(3):
        v, w = cmd[j], target[j]
        if q[LX_SMOOTH] > 0:
            rate = q[6 + j] if abs(w) > abs(v) and w * v >= 0 else -q[9 + j]
            w = v + max(-rate * dt, min(rate * dt, w - v))
        cmd[j] = w
    c, s = math.cos(x[2]), math.sin(x[2])
    nu0, nu1, nu2 = c * x[3] + s * x[4], -s * x[3] + c * x[4], x[5]
    tau = np.empty(3)
    for j in range(3):
        w = cmd[j]
        b = _D + 4 * j
        if w >= 0:
            d1, d2 = p[b], p[b + 2]
        else:
            d1, d2 = p[b + 1], p[b + 3]
        nu = nu0 if j == 0 else nu1 if j == 1 else nu2
        tau[j] = (d1 + d2 * abs(w)) * w + q[3 + j] * q[j] * (w - nu)
    f = np.empty(THRUSTERS)
    worst = 0.0
    for i in range(THRUSTERS):
        bq = 12 + 3 * i
        if blas:
            fi = nbexact.fma(q[bq + 2], tau[2], nbexact.fma(q[bq], tau[0], q[bq + 1] * tau[1]))
        else:
            fi = q[bq] * tau[0] + q[bq + 1] * tau[1] + q[bq + 2] * tau[2]
        f[i] = fi
        b = _T0 + i * _NT
        cap = p[b + _KF] if fi >= 0 else p[b + _KR]
        r = abs(fi) / cap
        if i == 0 or r > worst:
            worst = r
    scale = worst if worst > 1.0 else 1.0
    for i in range(THRUSTERS):
        b = _T0 + i * _NT
        fi = f[i]
        cap = p[b + _KF] if fi >= 0 else p[b + _KR]
        pwm[i] = PWM_NEUTRAL + p[b + _OFF] + PWM_SPAN * math.copysign(math.sqrt(abs(fi) / scale / cap), fi)
    return c, s


LX_SMOOTH = LX['smooth']
