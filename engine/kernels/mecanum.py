"""The modular wheeled-assembly kernel: mecanum, omni and conventional wheels and ball casters on one rigid
assembly, with a per-wheel fault model, on a batch. Each env has its own assembly (wheels padded to W).

    p = pack(wheels, body, faults)       # [P]: body (mass, COM, yaw inertia, IMU, encoder counts) + W wheel rows
    x = init(p[None], pose)              # [E, S]: x y yaw vx vy wz | dvx dvy dwz | step | per wheel: angle, speed, duty
    x = step(x, u, p, dt)                # one plant step under the body-twist command u [E, 3] (the firmware's input)

The physics is the modular truth model's planar dynamics (``capplan.truth.model``, the owner's evaluator model on
the modular platform branch), restricted to what a batch can carry: one rigid body (no planar compliance), normal
loads fixed per env (``domains.modular.assembly.static_loads``; no load transfer), ball casters (no swivel). Per
step, as ``simulate_assembly``: the firmware maps the twist to wheel speeds by ideal rolling inverse kinematics
(clipped to phi_max); each wheel's speed balances motor and ground torque (massless wheels: 8 bisections, then
Illinois false position, per wheel until it converges); the contact wrench of the tanh slip model, a finite-difference
Jacobian (four contact evaluations) and one linearly-implicit Euler step on the twist; the exact SE(2) pose update.

Faults per wheel (from its onset time): 0 ok, 1 weak(g) (stall torque and no-load speed times g: a duty scale),
2 coast (unpowered: gearbox back-drive only), 3 brake (unpowered, plus the shorted winding's damping
tau_stall / w_nl), 4 locked (the wheel does not turn). Duty is the firmware's pre-fault duty, tau / tau_stall +
w / w_nl of the nominal motor, clipped to [-1, 1]; the wheel angle integrates its speed (encoder counts:
``counts``); ``imu`` gives gz and the body-frame acceleration at the IMU point.
"""

import math

import numba
import numpy as np

V_MIN, V_EPS, W_EPS = 0.02, 0.01, 0.05        # the truth model's regularisations (m/s, m/s, rad/s)
VEPS2 = V_EPS ** 2
H_FD = 1e-5                                   # the Jacobian's finite-difference step (twist units)
N_BIS, N_ILL, TOL = 8, 40, 1e-11              # the wheel-speed root: bisections, Illinois steps, |h| tolerance
CONV, MEC, BALL = 0, 1, 3                     # wheel kinds (omni is MEC with its rollers at 90 deg)
ONSET_EPS = 1e-3                              # steps: fault onsets compare robustly in float32 too
MODES = ('ok', 'weak', 'coast', 'brake', 'locked')
OK, WEAK, COAST, BRAKE, LOCKED = range(5)

BODY = ['mass', 'xc', 'yc', 'inertia', 'imu_x', 'imu_y', 'cpr']
WHEEL = ['px', 'py', 'ex', 'ey', 'tx', 'ty', 'kind', 'r', 'r_fw', 'mu_t', 'mu_f', 's_x0', 'alpha0', 'c_rr', 'N',
         'K', 'tau_stall', 'w_nl', 'b', 'tau_c', 'tau_bd', 'phimax', 'active', 'mode', 'gain', 'onset']
IB = {n: i for i, n in enumerate(BODY)}
IW = {n: i for i, n in enumerate(WHEEL)}
G, Q = len(BODY), len(WHEEL)
STATE = ['x', 'y', 'yaw', 'vx', 'vy', 'wz', 'dvx', 'dvy', 'dwz', 'k']
S0 = len(STATE)
PER = 3                                       # per wheel in the state: angle, speed, duty
PAD = {'r': 1.0, 'r_fw': 1.0, 's_x0': 1.0, 'alpha0': 1.0, 'w_nl': 1.0, 'tau_stall': 1.0, 'kind': BALL}


def pack(wheels, body, faults=None, W=None):
    """A parameter row: ``body`` {mass, xc, yc, inertia, imu_x, imu_y, cpr}, ``wheels`` [{WHEEL field: value}]
    (assembly frame), ``faults`` {mode: [n], gain: [n], onset: [n]} (default all ok); padded to W wheels."""
    n = len(wheels)
    W = W or n
    if n > W:
        raise ValueError(f'{n} wheels do not fit W={W}')
    p = np.zeros(G + W * Q)
    for k in BODY:
        p[IB[k]] = float(body[k])
    faults = faults or {}
    for j in range(W):
        row = p[G + j * Q: G + (j + 1) * Q]
        if j >= n:
            for k, v in PAD.items():
                row[IW[k]] = v
            continue
        for k in WHEEL:
            if k not in ('active', 'mode', 'gain', 'onset'):
                row[IW[k]] = float(wheels[j][k])
        row[IW['active']] = 1.0
        row[IW['mode']] = float(faults.get('mode', [OK] * n)[j])
        row[IW['gain']] = float(faults.get('gain', [1.0] * n)[j])
        row[IW['onset']] = float(faults.get('onset', [0.0] * n)[j])
    return p


def stack(rows):
    """Rows of different wheel counts as one [E, P], each padded to the widest."""
    W = max((len(r) - G) // Q for r in rows)
    out = np.zeros((len(rows), G + W * Q))
    for e, r in enumerate(rows):
        w = (len(r) - G) // Q
        out[e, :len(r)] = r
        for j in range(w, W):
            for k, v in PAD.items():
                out[e, G + j * Q + IW[k]] = v
    return out


def with_faults(p, mode, gain=None, onset=None):
    """p [E, P] with the per-wheel faults mode [E, W] (gain, onset alike) set; padded wheels stay padded."""
    p = np.array(p, dtype=np.float64)
    E, W = p.shape[0], wheels_of(p)
    wp = p[:, G:].reshape(E, W, Q)
    act = wp[..., IW['active']] > 0
    for name, v, d in (('mode', mode, OK), ('gain', gain, 1.0), ('onset', onset, 0.0)):
        v = np.broadcast_to(np.asarray(d if v is None else v, dtype=np.float64), (E, W))
        wp[..., IW[name]] = np.where(act, v, wp[..., IW[name]])
    return p


def wheels_of(p):
    return (np.shape(p)[-1] - G) // Q


def size(p):
    return S0 + PER * wheels_of(p)


def init(p, pose=(0.0, 0.0, 0.0), twist=(0.0, 0.0, 0.0)):
    p = np.atleast_2d(p)
    x = np.zeros((p.shape[0], size(p)))
    x[:, 0:3] = np.asarray(pose, dtype=np.float64)
    x[:, 3:6] = np.asarray(twist, dtype=np.float64)
    return x


# ---- the array form (numpy, JAX) -----------------------------------------------------------------------------

def _is_jax(xp):
    return xp.__name__.startswith('jax')


@numba.vectorize(['float64(float64)'], cache=True)
def _libm_tanh(v):
    return math.tanh(v)


def tanh(v, xp=np):
    """libm's tanh on numpy arrays (numpy's own SIMD tanh differs from libm in the last bit), jnp.tanh on JAX."""
    return xp.tanh(v) if _is_jax(xp) else _libm_tanh(v)


def _wheels(p, xp):
    E = p.shape[0]
    W = (p.shape[-1] - G) // Q
    wp = p[:, G:].reshape(E, W, Q)
    return {k: wp[..., i] for k, i in IW.items()}


def _state(w, t, xp):
    """Per-wheel effective parameters at time t [E] (faults from their onset)."""
    on = (w['mode'] != OK) & (t[:, None] >= w['onset'])
    mode = xp.where(on, w['mode'], 0.0 * w['mode'])
    weak, coast, brake, locked = (mode == WEAK), (mode == COAST), (mode == BRAKE), (mode == LOCKED)
    act = w['active'] > 0
    ball = w['kind'] == BALL
    g = w['gain']
    return {'ts': xp.where(weak, g * w['tau_stall'], w['tau_stall']), 'wnl': xp.where(weak, g * w['w_nl'], w['w_nl']),
            'powered': act & ~ball & ~coast & ~brake & ~locked, 'brake': brake, 'spins': act & ~ball & ~locked,
            'kbr': w['tau_stall'] / w['w_nl']}


def _traction(w, vcx, vcy, om, xp):
    tx, ty, ex, ey = w['tx'], w['ty'], w['ex'], w['ey']
    fdx, fdy = -ty, tx
    rw = w['r'] * om
    vt = tx * vcx + ty * vcy
    te = tx * ex + ty * ey
    s_t = vt - rw * te
    vhx, vhy = vcx - rw * ex, vcy - rw * ey
    v_f = fdx * vhx + fdy * vhy
    V = xp.maximum(xp.maximum(xp.abs(rw), xp.sqrt(vcx * vcx + vcy * vcy)), V_MIN)
    st = s_t / (w['s_x0'] * V)
    sf = v_f / (w['alpha0'] * V)
    rho = xp.sqrt(st * st + sf * sf) + 1e-12
    gf = tanh(rho, xp) / rho
    mu_t, mu_f, N = w['mu_t'], w['mu_f'], w['N']
    Ftc = -mu_t * N * gf * st
    Ffc = -mu_f * N * gf * sf
    Ftm = -mu_t * N * tanh(st, xp)
    Ffm = -mu_f * N * tanh(v_f / V_EPS, xp)
    q = xp.sqrt(Ftm * Ftm + Ffm * Ffm) / (mu_t * N + 1e-12)
    sc = xp.where(q > 1.0, 1.0 / xp.maximum(q, 1e-12), 1.0)
    mec = w['kind'] == MEC
    Ft = xp.where(mec, Ftm * sc, Ftc)
    Ff = xp.where(mec, Ffm * sc, Ffc)
    Fx = Ft * tx + Ff * fdx
    Fy = Ft * ty + Ff * fdy
    return Fx, Fy, ex * Fx + ey * Fy


def _motor(w, f, om, cmd, xp):
    tcmd = w['K'] * (cmd - om)
    motoring = tcmd * om > 0
    lim = f['ts'] * xp.where(motoring, xp.minimum(xp.maximum(1.0 - xp.abs(om) / f['wnl'], 0.0), 1.0), 1.0)
    tm = xp.minimum(xp.maximum(tcmd, -lim), lim)
    lc = xp.where(f['powered'], w['tau_c'], w['tau_bd'])
    out = xp.where(f['powered'], tm, 0.0) - lc * tanh(om / W_EPS, xp) - w['b'] * om
    return out - xp.where(f['brake'], f['kbr'] * om, 0.0)


def _loop(n, body, carry, xp, done):
    if _is_jax(xp):
        import jax
        return jax.lax.fori_loop(0, n, lambda i, c: body(c), carry)
    for _ in range(n):
        carry = body(carry)
        if done(carry):
            break
    return carry


def _wheel_speeds(w, f, vcx, vcy, cmd, xp):
    """The truth model's solve_wheels, per wheel: the speed where motor torque = r e.F (bisection, then Illinois)."""
    def h(om):
        return _motor(w, f, om, cmd, xp) - w['r'] * _traction(w, vcx, vcy, om, xp)[2]
    te = xp.abs(w['tx'] * w['ex'] + w['ty'] * w['ey'])
    Wb = 1.3 * f['wnl'] + 3.0 * xp.sqrt(vcx * vcx + vcy * vcy) / (w['r'] * xp.maximum(te, 0.3)) + 2.0
    Wb = Wb + xp.abs(cmd)
    lo, hi = -Wb, Wb
    hlo, hhi = h(lo), h(hi)
    for _ in range(N_BIS):
        mid = 0.5 * (lo + hi)
        hm = h(mid)
        pos = hm > 0
        lo, hlo = xp.where(pos, mid, lo), xp.where(pos, hm, hlo)
        hi, hhi = xp.where(pos, hi, mid), xp.where(pos, hhi, hm)
    side = 0.0 * lo
    x = 0.5 * (lo + hi)
    done = ~f['spins']

    def body(c):
        lo, hi, hlo, hhi, side, x, done = c
        den = hlo - hhi
        ok = xp.abs(den) > 1e-300
        xn = xp.where(ok, (lo * (-hhi) + hi * hlo) / xp.where(ok, den, 1.0), 0.5 * (lo + hi))
        xn = xp.minimum(xp.maximum(xn, xp.minimum(lo, hi)), xp.maximum(lo, hi))
        hx = h(xn)
        pos = hx > 0
        lo_n, hi_n = xp.where(pos, xn, lo), xp.where(pos, hi, xn)
        hlo_n = xp.where(pos, hx, xp.where(side == -1, hlo * 0.5, hlo))
        hhi_n = xp.where(pos, xp.where(side == 1, hhi * 0.5, hhi), hx)
        side_n = xp.where(pos, 1.0, -1.0)
        go = ~done
        out = tuple(xp.where(go, a, b) for a, b in ((lo_n, lo), (hi_n, hi), (hlo_n, hlo), (hhi_n, hhi),
                                                     (side_n, side), (xn, x)))
        done = done | (go & ((xp.abs(hx) < TOL) | (xp.abs(hi_n - lo_n) < 1e-13)))
        return out + (done,)
    c = _loop(N_ILL, body, (lo, hi, hlo, hhi, side, x, done), xp, lambda c: bool(c[-1].all()))
    return xp.where(f['spins'], c[5], 0.0)


def _contact(w, f, xi0, xi1, xi2, cmd, xp):
    """The wrench (Fx, Fy, Mz about the origin) [E] for twists [E] and the wheel speeds [E, W]."""
    px, py = w['px'], w['py']
    vcx = xi0[:, None] + xi2[:, None] * (-py)
    vcy = xi1[:, None] + xi2[:, None] * px
    om = _wheel_speeds(w, f, vcx, vcy, cmd, xp)
    Fx, Fy, _ = _traction(w, vcx, vcy, om, xp)
    ex, ey = w['ex'], w['ey']
    rr = w['c_rr'] * w['N'] * tanh((ex * vcx + ey * vcy) / V_EPS, xp)
    Fx, Fy = Fx - rr * ex, Fy - rr * ey
    sp = xp.sqrt(vcx * vcx + vcy * vcy + VEPS2)
    cn = -(w['c_rr'] * w['N'])
    ball = w['kind'] == BALL
    Fx, Fy = xp.where(ball, cn * vcx / sp, Fx), xp.where(ball, cn * vcy / sp, Fy)
    live = (w['N'] > 0) & (w['active'] > 0)
    Fx, Fy = xp.where(live, Fx, 0.0), xp.where(live, Fy, 0.0)
    mz = px * Fy - py * Fx
    W = Fx.shape[-1]
    sx, sy, sz = Fx[:, 0], Fy[:, 0], mz[:, 0]
    for j in range(1, W):
        sx, sy, sz = sx + Fx[:, j], sy + Fy[:, j], sz + mz[:, j]
    return (sx, sy, sz), om


def _bias(b, xi0, xi1, xi2):
    m, c0, c1 = b['mass'], b['xc'], b['yc']
    vcx = xi0 - xi2 * c1
    vcy = xi1 + xi2 * c0
    bl0 = m * (-xi2 * vcy)
    bl1 = m * (xi2 * vcx)
    return bl0, bl1, c0 * bl1 - c1 * bl0


def _solve3(A, r, xp):
    """A x = r for 3x3 A (rows of [E] arrays), Gaussian elimination with partial pivoting (first max)."""
    A = [list(row) for row in A]
    r = list(r)
    for k in range(3):
        for i in range(k + 1, 3):          # bring the largest |A[i][k]| (i >= k) to row k, first max kept
            swap = xp.abs(A[i][k]) > xp.abs(A[k][k])
            for j in range(3):
                A[k][j], A[i][j] = xp.where(swap, A[i][j], A[k][j]), xp.where(swap, A[k][j], A[i][j])
            r[k], r[i] = xp.where(swap, r[i], r[k]), xp.where(swap, r[k], r[i])
        for i in range(k + 1, 3):
            m = A[i][k] / A[k][k]
            for j in range(k, 3):
                A[i][j] = A[i][j] - m * A[k][j]
            r[i] = r[i] - m * r[k]
    x2 = r[2] / A[2][2]
    x1 = (r[1] - A[1][2] * x2) / A[1][1]
    x0 = (r[0] - A[0][1] * x1 - A[0][2] * x2) / A[0][0]
    return x0, x1, x2


def firmware(w, u, xp=np):
    """Ideal rolling IK of the twist u [E, 3]: wheel speed commands [E, W], clipped to +-phi_max."""
    px, py = w['px'], w['py']
    vcx = u[:, 0:1] + u[:, 2:3] * (-py)
    vcy = u[:, 1:2] + u[:, 2:3] * px
    te = w['tx'] * w['ex'] + w['ty'] * w['ey']
    cmd = (w['tx'] * vcx + w['ty'] * vcy) / (w['r_fw'] * xp.where(xp.abs(te) > 1e-6, te, 1.0))
    cmd = xp.minimum(xp.maximum(cmd, -w['phimax']), w['phimax'])
    return xp.where((w['kind'] != BALL) & (w['active'] > 0), cmd, 0.0)


def step(x, u, p, dt, xp=np):
    """One plant step of every env: x [E, S], u [E, 3] the body-twist command, p [E, P]."""
    w = _wheels(p, xp)
    b = {k: p[:, i] for k, i in IB.items()}
    E, W = x.shape[0], (p.shape[-1] - G) // Q
    t = (x[:, 9] + ONSET_EPS) * dt          # a fault is on from the step its onset falls in
    f = _state(w, t, xp)
    cmd = firmware(w, u, xp)
    xi = (x[:, 3], x[:, 4], x[:, 5])
    pert = [xi, (xi[0] + H_FD, xi[1] + 0.0, xi[2] + 0.0), (xi[0] + 0.0, xi[1] + H_FD, xi[2] + 0.0),
            (xi[0] + 0.0, xi[1] + 0.0, xi[2] + H_FD)]
    Wr, om = [], None
    for q in pert:
        wr, o = _contact(w, f, *q, cmd, xp)
        bs = _bias(b, *q)
        Wr.append(tuple(a - c for a, c in zip(wr, bs)))
        om = o if om is None else om
    J = [[(Wr[i + 1][r] - Wr[0][r]) / H_FD for i in range(3)] for r in range(3)]
    m, c0, c1, I = b['mass'], b['xc'], b['yc'], b['inertia']
    zero = 0.0 * m
    M = [[m, zero, -m * c1], [zero, m, m * c0], [-m * c1, m * c0, I]]
    A = [[M[r][c] - dt * J[r][c] for c in range(3)] for r in range(3)]
    d = _solve3(A, [dt * Wr[0][r] for r in range(3)], xp)
    v = [xi[i] + d[i] for i in range(3)]
    acc = [d[i] / dt for i in range(3)]
    # exact SE(2) step with the new twist
    vx, vy, wz = v[0] * dt, v[1] * dt, v[2] * dt
    small = xp.abs(wz) < 1e-9
    ws = xp.where(small, 1.0, wz)
    s, c = xp.sin(ws), xp.cos(ws)
    dx = xp.where(small, vx, (s * vx - (1 - c) * vy) / ws)
    dy = xp.where(small, vy, ((1 - c) * vx + s * vy) / ws)
    cs, sn = xp.cos(x[:, 2]), xp.sin(x[:, 2])
    pose = [x[:, 0] + cs * dx - sn * dy, x[:, 1] + sn * dx + cs * dy, x[:, 2] + wz]
    # the firmware's pre-fault duty and the wheel angles
    tcmd = w['K'] * (cmd - om)
    lim = w['tau_stall'] * xp.where(tcmd * om > 0, xp.minimum(xp.maximum(1.0 - xp.abs(om) / w['w_nl'], 0.0), 1.0), 1.0)
    tm = xp.minimum(xp.maximum(tcmd, -lim), lim)
    duty = xp.minimum(xp.maximum(tm / w['tau_stall'] + om / w['w_nl'], -1.0), 1.0)
    duty = xp.where((w['kind'] != BALL) & (w['active'] > 0), duty, 0.0)
    wx = x[:, S0:].reshape(E, W, PER)
    ang = wx[..., 0] + om * dt
    wheels = xp.stack([ang, om, duty], axis=-1).reshape(E, W * PER)
    body = xp.stack(pose + v + acc + [x[:, 9] + 1.0], axis=-1)
    return xp.concatenate([body, wheels], axis=-1)


def counts(x, p, xp=np):
    """Encoder counts [E, W] from the wheel angles: floor(angle / 2 pi * cpr)."""
    E, W = x.shape[0], wheels_of(p)
    ang = x[:, S0:].reshape(E, W, PER)[..., 0]
    return xp.floor(ang / (2 * math.pi) * p[:, IB['cpr']][:, None])


def imu(x, p, xp=np):
    """(gz, ax, ay) [E]: the yaw rate and the body-frame acceleration at the IMU point."""
    vx, vy, wz, dvx, dvy, dwz = (x[:, i] for i in range(3, 9))
    rx, ry = p[:, IB['imu_x']], p[:, IB['imu_y']]
    ax = dvx - wz * vy - dwz * ry - wz * wz * rx
    ay = dvy + wz * vx + dwz * rx - wz * wz * ry
    return wz, ax, ay


def buffers(W):
    """nb_step's work buffers for W wheels."""
    return np.zeros(W), np.zeros(W), np.zeros((4, W)), np.zeros((4, 3)), np.zeros((3, 3)), np.zeros(3)


# ---- numba: one env's row ------------------------------------------------------------------------------------

@numba.njit(cache=True, inline='always')
def _nb_traction(p, o, vcx, vcy, om):
    tx, ty, ex, ey = p[o + 4], p[o + 5], p[o + 2], p[o + 3]
    fdx, fdy = -ty, tx
    rw = p[o + 7] * om
    vt = tx * vcx + ty * vcy
    te = tx * ex + ty * ey
    s_t = vt - rw * te
    vhx, vhy = vcx - rw * ex, vcy - rw * ey
    v_f = fdx * vhx + fdy * vhy
    V = max(max(abs(rw), math.sqrt(vcx * vcx + vcy * vcy)), V_MIN)
    st = s_t / (p[o + 11] * V)
    sf = v_f / (p[o + 12] * V)
    mu_t, mu_f, N = p[o + 9], p[o + 10], p[o + 14]
    if p[o + 6] == MEC:
        Ft = -mu_t * N * math.tanh(st)
        Ff = -mu_f * N * math.tanh(v_f / V_EPS)
        q = math.sqrt(Ft * Ft + Ff * Ff) / (mu_t * N + 1e-12)
        if q > 1.0:
            sc = 1.0 / max(q, 1e-12)
            Ft, Ff = Ft * sc, Ff * sc
        else:
            Ft, Ff = Ft * 1.0, Ff * 1.0
    else:
        rho = math.sqrt(st * st + sf * sf) + 1e-12
        gf = math.tanh(rho) / rho
        Ft = -mu_t * N * gf * st
        Ff = -mu_f * N * gf * sf
    Fx = Ft * tx + Ff * fdx
    Fy = Ft * ty + Ff * fdy
    return Fx, Fy, ex * Fx + ey * Fy


@numba.njit(cache=True, inline='always')
def _nb_h(p, o, vcx, vcy, om, cmd, ts, wnl, powered, brake, kbr):
    tcmd = p[o + 15] * (cmd - om)
    if tcmd * om > 0:
        lim = ts * min(max(1.0 - abs(om) / wnl, 0.0), 1.0)
    else:
        lim = ts * 1.0
    tm = min(max(tcmd, -lim), lim)
    if powered:
        out = tm - p[o + 19] * math.tanh(om / W_EPS) - p[o + 18] * om
    else:
        out = 0.0 - p[o + 20] * math.tanh(om / W_EPS) - p[o + 18] * om
    if brake:
        out = out - kbr * om
    else:
        out = out - 0.0
    return out - p[o + 7] * _nb_traction(p, o, vcx, vcy, om)[2]


@numba.njit(cache=True, inline='always')
def _nb_speed(p, o, vcx, vcy, cmd, ts, wnl, powered, brake, kbr):
    te = abs(p[o + 4] * p[o + 2] + p[o + 5] * p[o + 3])
    Wb = 1.3 * wnl + 3.0 * math.sqrt(vcx * vcx + vcy * vcy) / (p[o + 7] * max(te, 0.3)) + 2.0
    Wb = Wb + abs(cmd)
    lo, hi = -Wb, Wb
    hlo = _nb_h(p, o, vcx, vcy, lo, cmd, ts, wnl, powered, brake, kbr)
    hhi = _nb_h(p, o, vcx, vcy, hi, cmd, ts, wnl, powered, brake, kbr)
    for _ in range(N_BIS):
        mid = 0.5 * (lo + hi)
        hm = _nb_h(p, o, vcx, vcy, mid, cmd, ts, wnl, powered, brake, kbr)
        if hm > 0:
            lo, hlo = mid, hm
        else:
            hi, hhi = mid, hm
    side = 0.0
    x = 0.5 * (lo + hi)
    for _ in range(N_ILL):
        den = hlo - hhi
        if abs(den) > 1e-300:
            x = (lo * (-hhi) + hi * hlo) / den
        else:
            x = 0.5 * (lo + hi)
        x = min(max(x, min(lo, hi)), max(lo, hi))
        hx = _nb_h(p, o, vcx, vcy, x, cmd, ts, wnl, powered, brake, kbr)
        if hx > 0:
            lo = x
            hlo = hx
            if side == 1:
                hhi = hhi * 0.5
            side = 1.0
        else:
            hi = x
            if side == -1:
                hlo = hlo * 0.5
            hhi = hx
            side = -1.0
        if abs(hx) < TOL or abs(hi - lo) < 1e-13:
            break
    return x


@numba.njit(cache=True, inline='always')
def _nb_contact(p, W, t, xi0, xi1, xi2, cmd, om):
    sx, sy, sz = 0.0, 0.0, 0.0
    for j in range(W):
        o = G + j * Q
        px, py = p[o], p[o + 1]
        vcx = xi0 + xi2 * (-py)
        vcy = xi1 + xi2 * px
        mode = p[o + 23] if (p[o + 23] != OK and t >= p[o + 25]) else 0.0
        act = p[o + 22] > 0
        ball = p[o + 6] == BALL
        spins = act and not ball and mode != LOCKED
        powered = spins and mode != COAST and mode != BRAKE
        brake = mode == BRAKE
        ts, wnl = p[o + 16], p[o + 17]
        if mode == WEAK:
            ts, wnl = p[o + 24] * ts, p[o + 24] * wnl
        kbr = p[o + 16] / p[o + 17]
        w = _nb_speed(p, o, vcx, vcy, cmd[j], ts, wnl, powered, brake, kbr) if spins else 0.0
        om[j] = w
        Fx, Fy, _ = _nb_traction(p, o, vcx, vcy, w)
        ex, ey = p[o + 2], p[o + 3]
        rr = p[o + 13] * p[o + 14] * math.tanh((ex * vcx + ey * vcy) / V_EPS)
        Fx, Fy = Fx - rr * ex, Fy - rr * ey
        if ball:
            sp = math.sqrt(vcx * vcx + vcy * vcy + VEPS2)
            cn = -(p[o + 13] * p[o + 14])
            Fx, Fy = cn * vcx / sp, cn * vcy / sp
        if not (p[o + 14] > 0 and act):
            Fx, Fy = 0.0, 0.0
        mz = px * Fy - py * Fx
        if j == 0:
            sx, sy, sz = Fx, Fy, mz
        else:
            sx, sy, sz = sx + Fx, sy + Fy, sz + mz
    return sx, sy, sz


@numba.njit(cache=True, inline='always')
def _nb_solve3(A, r):
    for k in range(3):
        for i in range(k + 1, 3):
            if abs(A[i, k]) > abs(A[k, k]):
                for j in range(3):
                    A[k, j], A[i, j] = A[i, j], A[k, j]
                r[k], r[i] = r[i], r[k]
        for i in range(k + 1, 3):
            m = A[i, k] / A[k, k]
            for j in range(k, 3):
                A[i, j] = A[i, j] - m * A[k, j]
            r[i] = r[i] - m * r[k]
    x2 = r[2] / A[2, 2]
    x1 = (r[1] - A[1, 2] * x2) / A[1, 1]
    x0 = (r[0] - A[0, 1] * x1 - A[0, 2] * x2) / A[0, 0]
    return x0, x1, x2


@numba.njit(cache=True, inline='always')
def nb_step(x, u, p, dt, cmd, om, scratch, Wr, A, r):
    """step() on one env's row in place; work buffers (``buffers(W)``): cmd, om [W], scratch [4, W], Wr [4, 3],
    A [3, 3], r [3]."""
    W = (p.shape[0] - G) // Q
    t = (x[9] + ONSET_EPS) * dt
    for j in range(W):
        o = G + j * Q
        if p[o + 6] == BALL or p[o + 22] <= 0:
            cmd[j] = 0.0
            continue
        px, py = p[o], p[o + 1]
        vcx = u[0] + u[2] * (-py)
        vcy = u[1] + u[2] * px
        te = p[o + 4] * p[o + 2] + p[o + 5] * p[o + 3]
        c = (p[o + 4] * vcx + p[o + 5] * vcy) / (p[o + 8] * (te if abs(te) > 1e-6 else 1.0))
        cmd[j] = min(max(c, -p[o + 21]), p[o + 21])
    m, c0, c1, I = p[0], p[1], p[2], p[3]
    xi0, xi1, xi2 = x[3], x[4], x[5]
    for q in range(4):
        a0 = xi0 + (H_FD if q == 1 else 0.0)
        a1 = xi1 + (H_FD if q == 2 else 0.0)
        a2 = xi2 + (H_FD if q == 3 else 0.0)
        sx, sy, sz = _nb_contact(p, W, t, a0, a1, a2, cmd, scratch[q])
        vcx = a0 - a2 * c1
        vcy = a1 + a2 * c0
        bl0 = m * (-a2 * vcy)
        bl1 = m * (a2 * vcx)
        Wr[q, 0], Wr[q, 1], Wr[q, 2] = sx - bl0, sy - bl1, sz - (c0 * bl1 - c1 * bl0)
    A[0, 0], A[0, 1], A[0, 2] = m, 0.0, -m * c1
    A[1, 0], A[1, 1], A[1, 2] = 0.0, m, m * c0
    A[2, 0], A[2, 1], A[2, 2] = -m * c1, m * c0, I
    for rr in range(3):
        for cc in range(3):
            A[rr, cc] = A[rr, cc] - dt * ((Wr[cc + 1, rr] - Wr[0, rr]) / H_FD)
        r[rr] = dt * Wr[0, rr]
    d0, d1, d2 = _nb_solve3(A, r)
    v0, v1, v2 = xi0 + d0, xi1 + d1, xi2 + d2
    vx, vy, wz = v0 * dt, v1 * dt, v2 * dt
    if abs(wz) < 1e-9:
        dx, dy = vx, vy
    else:
        s, c = math.sin(wz), math.cos(wz)
        dx = (s * vx - (1 - c) * vy) / wz
        dy = ((1 - c) * vx + s * vy) / wz
    cs, sn = math.cos(x[2]), math.sin(x[2])
    x[0], x[1], x[2] = x[0] + cs * dx - sn * dy, x[1] + sn * dx + cs * dy, x[2] + wz
    x[3], x[4], x[5] = v0, v1, v2
    x[6], x[7], x[8] = d0 / dt, d1 / dt, d2 / dt
    x[9] = x[9] + 1.0
    for j in range(W):
        o = G + j * Q
        w = scratch[0, j]
        om[j] = w
        k = S0 + PER * j
        x[k] = x[k] + w * dt
        x[k + 1] = w
        if p[o + 6] == BALL or p[o + 22] <= 0:
            x[k + 2] = 0.0
            continue
        tcmd = p[o + 15] * (cmd[j] - w)
        ts, wnl = p[o + 16], p[o + 17]
        lim = ts * (min(max(1.0 - abs(w) / wnl, 0.0), 1.0) if tcmd * w > 0 else 1.0)
        tm = min(max(tcmd, -lim), lim)
        x[k + 2] = min(max(tm / ts + w / wnl, -1.0), 1.0)
