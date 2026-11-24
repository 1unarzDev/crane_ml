# SPDX-License-Identifier: GPL-3.0-or-later
"""The drones' T1 kernel: the rigid-body multirotor of domains/drones/model.py (``step``) on a batch.

    p = pack(card.vehicle('x500'))          # [P] float64 from a plant card (stack rows, pad rotors: [E, P])
    x = init(p[None], pos=(0, 0, 10), vel=(5, 0, 0), yaw=0.3, rotor_speed=hover_speed(p))   # [E, S]
    x = step(x, u, p, dt)                    # u [E, R] per-rotor target speed (rad/s); wind [E, 3] optional
    nb_step(x[e], u[e], p[e], dt, wind[e])   # the same map as a numba scalar over one env's row, in place

State [S] = p(3) v(3) q(4, Hamilton w x y z, body FLU to world ENU) omega(3, body) soc t | rotor speed(R).
Parameters [P] = the globals of GLOBAL, then R rotors of ROTOR each. A card with fewer rotors than the batch
pads with zero rotors (kf = km = omega_max = 0: no force, no torque), so quads and hexes share one batch.

The map is model.step term for term: thrust kf w^2 (ground effect optional), reaction torque, rotor drag and
rolling moment on the hub's in-plane air velocity, body drag, the rotor gyroscopic torque, the battery's
Thevenin sag on omega_max, motors as the exact first-order lag (tau up / down) at each RK4 stage time, RK4 on
the rigid body with the quaternion renormalised per stage. Every branch is a ``where`` (numpy or jax.numpy);
``np.linalg.solve`` is the inverse inertia from ``pack``, so the batch agrees with model.step to round-off
(gate 1: <= 1e-10 over scripted commands, engine/tests/test_aerial.py).
"""

import math

import numpy as np

try:
    import numba
    _jit = numba.njit(cache=True, inline='always')
    _njit = numba.njit(cache=True)
except ImportError:          # numpy and jax backends need no numba
    numba = None
    _jit = _njit = lambda f: f

GLOBAL = ['m', 'g', *(f'I{i}{j}' for i in range(3) for j in range(3)), *(f'Iinv{i}{j}' for i in range(3) for j in range(3)),
          'dlin0', 'dlin1', 'dlin2', 'dquad', 'ground_effect', 'ge_ratio', 'ground_z', 'contact', 'zoh_first',
          'battery', 'v_full', 'v_empty', 'v_nominal', 'resistance', 'capacity_ah', 'efficiency', 'rotors']
ROTOR = ['px', 'py', 'pz', 'ax', 'ay', 'az', 'spin', 'kf', 'km', 'omega_max', 'omega_min', 'tau_up', 'tau_down',
         'drag', 'roll', 'radius', 'inertia']
GX = {n: i for i, n in enumerate(GLOBAL)}
RX = {n: i for i, n in enumerate(ROTOR)}
G, K = len(GLOBAL), len(ROTOR)
BODY = 15                    # p v q omega soc t
POS, VEL, QUAT, RATE, SOC, TIME = 0, 3, 6, 10, 13, 14


def pack(vehicle, rotors=None, contact=False, motor_scheme='exact_stage'):
    """One parameter row from a model.Vehicle (domains/drones/card.vehicle), padded to ``rotors`` rotors."""
    n = len(vehicle.rotors)
    rotors = rotors or n
    if rotors < n:
        raise ValueError(f'{vehicle.name} has {n} rotors, more than the batch width {rotors}')
    inertia = np.asarray(vehicle.inertia, dtype=float)
    b = vehicle.battery
    row = [vehicle.mass, vehicle.gravity, *inertia.ravel(), *np.linalg.inv(inertia).ravel(),
           *np.asarray(vehicle.drag_linear, dtype=float), vehicle.drag_quadratic, float(bool(vehicle.ground_effect)),
           vehicle.ground_effect_min_height_ratio, vehicle.ground_height, float(bool(contact)),
           {'exact_stage': 0.0, 'zoh_first': 1.0}[motor_scheme], float(b is not None),
           *((b.v_full, b.v_empty, b.v_nominal, b.resistance, b.capacity_ah, b.efficiency) if b else (0.0,) * 6),
           float(n)]
    for r in vehicle.rotors:
        axis = np.asarray(r.axis, dtype=float) / np.linalg.norm(r.axis)
        row += [*np.asarray(r.position, dtype=float), *axis, float(r.spin), r.kf, r.km, r.omega_max, r.omega_min,
                r.tau_up, r.tau_down, r.rotor_drag, r.rolling_moment, r.radius, r.inertia]
    row += [0.0] * (K * (rotors - n))
    return np.array(row, dtype=np.float64)


def rotors_of(p):
    return (np.shape(p)[-1] - G) // K


def hover_speed(p):
    """Equal-speed hover (model.hover_rotor_speed) per row: sqrt(m g / sum kf)."""
    p = np.atleast_2d(p)
    kf = p[:, G + RX['kf']::K][:, :rotors_of(p)].sum(axis=1)
    return np.sqrt(p[:, GX['m']] * p[:, GX['g']] / kf)


def init(p, pos=(0.0, 0.0, 1.0), vel=(0.0, 0.0, 0.0), yaw=0.0, rotor_speed=0.0, soc=1.0):
    """[E, S] states: level at ``yaw`` (scalar or [E]), body rates zero; arguments broadcast over E."""
    p = np.atleast_2d(p)
    E, R = p.shape[0], rotors_of(p)
    x = np.zeros((E, BODY + R))
    x[:, POS:POS + 3] = np.broadcast_to(np.asarray(pos, dtype=float), (E, 3))
    x[:, VEL:VEL + 3] = np.broadcast_to(np.asarray(vel, dtype=float), (E, 3))
    half = np.broadcast_to(np.asarray(yaw, dtype=float), (E,)) / 2.0
    x[:, QUAT] = np.cos(half)
    x[:, QUAT + 3] = np.sin(half)
    x[:, SOC] = soc
    w = np.broadcast_to(np.asarray(rotor_speed, dtype=float), (E,) if np.ndim(rotor_speed) <= 1 else (E, R))
    x[:, BODY:] = w[:, None] if w.ndim == 1 else w
    real = np.arange(R)[None, :] < p[:, GX['rotors']][:, None]
    x[:, BODY:] = np.where(real, x[:, BODY:], 0.0)
    return x


# ---- the map on arrays (numpy or jax.numpy): every quantity a list of [E] components ----------------------------

def _g(p, k):
    return p[..., GX[k]]


def _r(p, i, k):
    return p[..., G + K * i + RX[k]]


def _cross(a, b):
    return [a[1] * b[2] - a[2] * b[1], a[2] * b[0] - a[0] * b[2], a[0] * b[1] - a[1] * b[0]]


def matrix(xp, q):
    n = xp.sqrt(q[0] * q[0] + q[1] * q[1] + q[2] * q[2] + q[3] * q[3])
    w, x, y, z = q[0] / n, q[1] / n, q[2] / n, q[3] / n
    return [[1 - 2 * (y * y + z * z), 2 * (x * y - w * z), 2 * (x * z + w * y)],
            [2 * (x * y + w * z), 1 - 2 * (x * x + z * z), 2 * (y * z - w * x)],
            [2 * (x * z - w * y), 2 * (y * z + w * x), 1 - 2 * (x * x + y * y)]]


def _current(xp, p, speeds, soc):
    """model.electrical_current: the fixed point of I = P / (eta V(I)); 0 without a battery."""
    bat = _g(p, 'battery') > 0
    power = 0.0 * soc
    for i, w in enumerate(speeds):
        a = xp.abs(w)
        power = power + _r(p, i, 'km') * _r(p, i, 'kf') * (a * a * a)
    power = power / xp.where(bat, _g(p, 'efficiency'), 1.0)
    voc = _g(p, 'v_empty') + (_g(p, 'v_full') - _g(p, 'v_empty')) * xp.minimum(xp.maximum(soc, 0.0), 1.0)
    res = xp.where(bat, _g(p, 'resistance'), 1.0)
    disc = voc * voc - 4.0 * res * power
    cur = xp.where(disc < 0.0, voc / (2.0 * res), (voc - xp.sqrt(xp.maximum(disc, 0.0))) / (2.0 * res))
    return xp.where(bat, cur, 0.0 * soc), voc, res


def _limits(xp, p, speeds, soc):
    cur, voc, res = _current(xp, p, speeds, soc)
    bat = _g(p, 'battery') > 0
    volts = xp.maximum(0.0, voc - cur * res)
    scale = xp.minimum(1.0, volts / xp.where(bat, _g(p, 'v_nominal'), 1.0))
    return [xp.where(bat, _r(p, i, 'omega_max') * scale, _r(p, i, 'omega_max')) for i in range(len(speeds))]


def wrench(xp, p, pos, vel, q, om, speeds, wind):
    """model.wrench: body force and torque (no gravity)."""
    Rm = matrix(xp, q)
    rel = [vel[j] - wind[j] for j in range(3)]
    vb = [Rm[0][i] * rel[0] + Rm[1][i] * rel[1] + Rm[2][i] * rel[2] for i in range(3)]
    zero = 0.0 * vb[0]
    force, torque, h = [zero] * 3, [zero] * 3, [zero] * 3
    ge_on = _g(p, 'ground_effect') > 0
    for i, w in enumerate(speeds):
        rp = [_r(p, i, 'px'), _r(p, i, 'py'), _r(p, i, 'pz')]
        ax = [_r(p, i, 'ax'), _r(p, i, 'ay'), _r(p, i, 'az')]
        radius = _r(p, i, 'radius')
        hub_z = pos[2] + (Rm[2][0] * rp[0] + Rm[2][1] * rp[1] + Rm[2][2] * rp[2])
        height = hub_z - _g(p, 'ground_z')
        hh = xp.maximum(height, _g(p, 'ge_ratio') * radius)
        ratio = radius / (4.0 * xp.where(hh > 0.0, hh, 1.0))
        ge = xp.where(ge_on & (radius > 0.0), 1.0 / (1.0 - ratio * ratio), 1.0)
        thrust = _r(p, i, 'kf') * w * w * ge
        aw = xp.abs(w)
        wxr = _cross(om, rp)
        hub = [vb[j] + wxr[j] for j in range(3)]
        along = hub[0] * ax[0] + hub[1] * ax[1] + hub[2] * ax[2]
        perp = [hub[j] - along * ax[j] for j in range(3)]
        f = [thrust * ax[j] - aw * _r(p, i, 'drag') * perp[j] for j in range(3)]
        rxf = _cross(rp, f)
        spin, km, roll, J = _r(p, i, 'spin'), _r(p, i, 'km'), _r(p, i, 'roll'), _r(p, i, 'inertia')
        for j in range(3):
            force[j] = force[j] + f[j]
            torque[j] = torque[j] + rxf[j]
            torque[j] = torque[j] + -spin * km * thrust * ax[j]
            torque[j] = torque[j] + -aw * roll * perp[j]
            h[j] = h[j] + J * spin * w * ax[j]
    gyro = _cross(om, h)
    torque = [torque[j] - gyro[j] for j in range(3)]
    speed = xp.sqrt(vb[0] * vb[0] + vb[1] * vb[1] + vb[2] * vb[2])
    force = [force[j] + (-_g(p, f'dlin{j}') * vb[j] - _g(p, 'dquad') * speed * vb[j]) for j in range(3)]
    return force, torque, Rm


def derivative(xp, p, pos, vel, q, om, speeds, wind):
    F, T, Rm = wrench(xp, p, pos, vel, q, om, speeds, wind)
    m = _g(p, 'm')
    acc = [(Rm[i][0] * F[0] + Rm[i][1] * F[1] + Rm[i][2] * F[2]) / m for i in range(3)]
    acc[2] = acc[2] + -_g(p, 'g')
    I = [[_g(p, f'I{i}{j}') for j in range(3)] for i in range(3)]
    Iw = [I[i][0] * om[0] + I[i][1] * om[1] + I[i][2] * om[2] for i in range(3)]
    c = _cross(om, Iw)
    rhs = [T[i] - c[i] for i in range(3)]
    od = [_g(p, f'Iinv{i}0') * rhs[0] + _g(p, f'Iinv{i}1') * rhs[1] + _g(p, f'Iinv{i}2') * rhs[2] for i in range(3)]
    w, x, y, z = q
    a, b, cc = om
    qd = [0.5 * (-x * a - y * b - z * cc), 0.5 * (w * a + y * cc - z * b), 0.5 * (w * b - x * cc + z * a),
          0.5 * (w * cc + x * b - y * a)]
    return list(vel), acc, qd, od


def _unit(xp, q):
    n = xp.sqrt(q[0] * q[0] + q[1] * q[1] + q[2] * q[2] + q[3] * q[3])
    return [c / n for c in q]


def step(x, u, p, dt, wind=None, xp=np):
    """One model.step on [E, S] states, [E, R] rotor targets, [E, P] parameters (``wind`` [E, 3] world)."""
    R = x.shape[-1] - BODY
    col = [x[..., k] for k in range(BODY)]
    pos, vel, q, om, soc, t = col[0:3], col[3:6], col[6:10], col[10:13], col[13], col[14]
    w0 = [x[..., BODY + i] for i in range(R)]
    wind = [0.0 * soc] * 3 if wind is None else [wind[..., j] for j in range(3)]
    limit = _limits(xp, p, w0, soc)
    target, taus = [], []
    for i in range(R):
        tg = xp.minimum(xp.maximum(u[..., i], _r(p, i, 'omega_min')), limit[i])
        target.append(tg)
        taus.append(xp.where(tg >= w0[i], _r(p, i, 'tau_up'), _r(p, i, 'tau_down')))

    def at(h):
        return [xp.where(taus[i] > 0.0, target[i] + (w0[i] - target[i]) * xp.exp(-h / xp.where(taus[i] > 0.0, taus[i], 1.0)),
                         target[i]) for i in range(R)]

    zoh = _g(p, 'zoh_first') > 0
    w_end = at(dt)
    w_a = [xp.where(zoh, e, s) for e, s in zip(w_end, at(0.0))]
    w_b = [xp.where(zoh, e, s) for e, s in zip(w_end, at(dt / 2))]

    def shifted(k, h):
        return ([pos[j] + h * k[0][j] for j in range(3)], [vel[j] + h * k[1][j] for j in range(3)],
                _unit(xp, [q[j] + h * k[2][j] for j in range(4)]), [om[j] + h * k[3][j] for j in range(3)])

    k1 = derivative(xp, p, pos, vel, q, om, w_a, wind)
    k2 = derivative(xp, p, *shifted(k1, dt / 2), w_b, wind)
    k3 = derivative(xp, p, *shifted(k2, dt / 2), w_b, wind)
    k4 = derivative(xp, p, *shifted(k3, dt), w_end, wind)

    def rk(s, n, j):
        return s + dt / 6 * (k1[n][j] + 2 * k2[n][j] + 2 * k3[n][j] + k4[n][j])

    npos = [rk(pos[j], 0, j) for j in range(3)]
    nvel = [rk(vel[j], 1, j) for j in range(3)]
    nq = _unit(xp, [rk(q[j], 2, j) for j in range(4)])
    nom = [rk(om[j], 3, j) for j in range(3)]
    bat = _g(p, 'battery') > 0
    cur, _, _ = _current(xp, p, w_b, soc)
    nsoc = xp.where(bat, soc - cur * dt / (3600.0 * xp.where(bat, _g(p, 'capacity_ah'), 1.0)), soc)
    down = (_g(p, 'contact') > 0) & (npos[2] < _g(p, 'ground_z'))
    npos[2] = xp.where(down, _g(p, 'ground_z'), npos[2])
    nvel[2] = xp.where(down, xp.maximum(0.0, nvel[2]), nvel[2])
    return xp.stack(npos + nvel + nq + nom + [nsoc, t + dt] + w_end, axis=-1)


# ---- the same map as numba scalars over one env's row (in place) -------------------------------------------------

@_jit
def nb_matrix(q0, q1, q2, q3, Rm):
    n = math.sqrt(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3)
    w, x, y, z = q0 / n, q1 / n, q2 / n, q3 / n
    Rm[0, 0], Rm[0, 1], Rm[0, 2] = 1 - 2 * (y * y + z * z), 2 * (x * y - w * z), 2 * (x * z + w * y)
    Rm[1, 0], Rm[1, 1], Rm[1, 2] = 2 * (x * y + w * z), 1 - 2 * (x * x + z * z), 2 * (y * z - w * x)
    Rm[2, 0], Rm[2, 1], Rm[2, 2] = 2 * (x * z - w * y), 2 * (y * z + w * x), 1 - 2 * (x * x + y * y)


@_jit
def nb_current(p, speeds, soc):
    """(current, open-circuit volts) of model.electrical_current; (0, 0) without a battery."""
    if p[29] <= 0:
        return 0.0, 0.0
    power = 0.0
    for i in range(speeds.shape[0]):
        a = abs(speeds[i])
        b = G + K * i
        power += p[b + 8] * p[b + 7] * (a * a * a)
    power /= p[35]
    voc = p[31] + (p[30] - p[31]) * min(max(soc, 0.0), 1.0)
    res = p[33]
    disc = voc * voc - 4.0 * res * power
    if disc < 0.0:
        return voc / (2.0 * res), voc
    return (voc - math.sqrt(disc)) / (2.0 * res), voc


@_jit
def nb_derivative(p, y, speeds, wind, out, Rm):
    """d/dt of y = p v q omega (13) into out (13)."""
    nb_matrix(y[6], y[7], y[8], y[9], Rm)
    r0, r1, r2 = y[3] - wind[0], y[4] - wind[1], y[5] - wind[2]
    vb0 = Rm[0, 0] * r0 + Rm[1, 0] * r1 + Rm[2, 0] * r2
    vb1 = Rm[0, 1] * r0 + Rm[1, 1] * r1 + Rm[2, 1] * r2
    vb2 = Rm[0, 2] * r0 + Rm[1, 2] * r1 + Rm[2, 2] * r2
    o0, o1, o2 = y[10], y[11], y[12]
    F0 = F1 = F2 = T0 = T1 = T2 = h0 = h1 = h2 = 0.0
    for i in range(speeds.shape[0]):
        b = G + K * i
        w = speeds[i]
        px, py, pz, ax, ay, az = p[b], p[b + 1], p[b + 2], p[b + 3], p[b + 4], p[b + 5]
        radius = p[b + 15]
        ge = 1.0
        if p[24] > 0 and radius > 0.0:
            hz = y[2] + (Rm[2, 0] * px + Rm[2, 1] * py + Rm[2, 2] * pz) - p[26]
            hh = max(hz, p[25] * radius)
            ratio = radius / (4.0 * hh)
            ge = 1.0 / (1.0 - ratio * ratio)
        thrust = p[b + 7] * w * w * ge
        aw = abs(w)
        hx = vb0 + (o1 * pz - o2 * py)
        hy = vb1 + (o2 * px - o0 * pz)
        hzv = vb2 + (o0 * py - o1 * px)
        along = hx * ax + hy * ay + hzv * az
        q0, q1, q2 = hx - along * ax, hy - along * ay, hzv - along * az
        dr = aw * p[b + 13]
        f0, f1, f2 = thrust * ax - dr * q0, thrust * ay - dr * q1, thrust * az - dr * q2
        F0 += f0
        F1 += f1
        F2 += f2
        spin, km, rl, J = p[b + 6], p[b + 8], aw * p[b + 14], p[b + 16]
        T0 += py * f2 - pz * f1
        T1 += pz * f0 - px * f2
        T2 += px * f1 - py * f0
        T0 += -spin * km * thrust * ax
        T1 += -spin * km * thrust * ay
        T2 += -spin * km * thrust * az
        T0 += -rl * q0
        T1 += -rl * q1
        T2 += -rl * q2
        h0 += J * spin * w * ax
        h1 += J * spin * w * ay
        h2 += J * spin * w * az
    T0 -= o1 * h2 - o2 * h1
    T1 -= o2 * h0 - o0 * h2
    T2 -= o0 * h1 - o1 * h0
    sp = math.sqrt(vb0 * vb0 + vb1 * vb1 + vb2 * vb2)
    F0 += -p[20] * vb0 - p[23] * sp * vb0
    F1 += -p[21] * vb1 - p[23] * sp * vb1
    F2 += -p[22] * vb2 - p[23] * sp * vb2
    m = p[0]
    out[0], out[1], out[2] = y[3], y[4], y[5]
    out[3] = (Rm[0, 0] * F0 + Rm[0, 1] * F1 + Rm[0, 2] * F2) / m
    out[4] = (Rm[1, 0] * F0 + Rm[1, 1] * F1 + Rm[1, 2] * F2) / m
    out[5] = (Rm[2, 0] * F0 + Rm[2, 1] * F1 + Rm[2, 2] * F2) / m + -p[1]
    Iw0 = p[2] * o0 + p[3] * o1 + p[4] * o2
    Iw1 = p[5] * o0 + p[6] * o1 + p[7] * o2
    Iw2 = p[8] * o0 + p[9] * o1 + p[10] * o2
    c0, c1, c2 = T0 - (o1 * Iw2 - o2 * Iw1), T1 - (o2 * Iw0 - o0 * Iw2), T2 - (o0 * Iw1 - o1 * Iw0)
    w, x, yy, z = y[6], y[7], y[8], y[9]
    out[6] = 0.5 * (-x * o0 - yy * o1 - z * o2)
    out[7] = 0.5 * (w * o0 + yy * o2 - z * o1)
    out[8] = 0.5 * (w * o1 - x * o2 + z * o0)
    out[9] = 0.5 * (w * o2 + x * o1 - yy * o0)
    out[10] = p[11] * c0 + p[12] * c1 + p[13] * c2
    out[11] = p[14] * c0 + p[15] * c1 + p[16] * c2
    out[12] = p[17] * c0 + p[18] * c1 + p[19] * c2


@_jit
def _nb_shift(y0, k, h, y):
    for j in range(13):
        y[j] = y0[j] + h * k[j]
    n = math.sqrt(y[6] * y[6] + y[7] * y[7] + y[8] * y[8] + y[9] * y[9])
    for j in range(6, 10):
        y[j] = y[j] / n


@_njit
def nb_step(x, u, p, dt, wind):
    """model.step on one env's row ``x`` [S] in place: ``u`` [R] rotor targets, ``p`` [P], ``wind`` [3]."""
    R = x.shape[0] - BODY
    soc = x[SOC]
    w0 = x[BODY:].copy()
    scale = 1.0
    if p[29] > 0:
        cur, voc = nb_current(p, w0, soc)
        scale = min(1.0, max(0.0, voc - cur * p[33]) / p[32])
    wa, wb, we = np.empty(R), np.empty(R), np.empty(R)
    zoh = p[28] > 0
    for i in range(R):
        b = G + K * i
        lim = p[b + 9] * scale if p[29] > 0 else p[b + 9]
        tg = min(max(u[i], p[b + 10]), lim)
        tau = p[b + 11] if tg >= w0[i] else p[b + 12]
        if tau > 0.0:
            we[i] = tg + (w0[i] - tg) * math.exp(-dt / tau)
            wa[i] = tg + (w0[i] - tg) * math.exp(-0.0 / tau)
            wb[i] = tg + (w0[i] - tg) * math.exp(-(dt / 2) / tau)
        else:
            we[i] = wa[i] = wb[i] = tg
        if zoh:
            wa[i] = wb[i] = we[i]
    y0 = x[:13].copy()
    y = np.empty(13)
    k1, k2, k3, k4 = np.empty(13), np.empty(13), np.empty(13), np.empty(13)
    Rm = np.empty((3, 3))
    nb_derivative(p, y0, wa, wind, k1, Rm)
    _nb_shift(y0, k1, dt / 2, y)
    nb_derivative(p, y, wb, wind, k2, Rm)
    _nb_shift(y0, k2, dt / 2, y)
    nb_derivative(p, y, wb, wind, k3, Rm)
    _nb_shift(y0, k3, dt, y)
    nb_derivative(p, y, we, wind, k4, Rm)
    for j in range(13):
        x[j] = y0[j] + dt / 6 * (k1[j] + 2 * k2[j] + 2 * k3[j] + k4[j])
    n = math.sqrt(x[6] * x[6] + x[7] * x[7] + x[8] * x[8] + x[9] * x[9])
    for j in range(6, 10):
        x[j] = x[j] / n
    if p[29] > 0:
        cur, _ = nb_current(p, wb, soc)
        x[SOC] = soc - cur * dt / (3600.0 * p[34])
    if p[27] > 0 and x[2] < p[26]:
        x[2] = p[26]
        x[5] = max(0.0, x[5])
    x[TIME] = x[TIME] + dt
    for i in range(R):
        x[BODY + i] = we[i]
