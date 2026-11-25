"""The reference for the modular kernel: a port of the modular truth model's step-wise dynamics.

Provenance: ``Tools/Fleet/capplan/truth/{model.py, stepper.py}`` on the modular platform branch (the owner's
evaluator model): ``_traction``, ``_motor``, ``solve_wheels``, ``contact``, ``_bias``, the inner step of
``simulate_assembly`` and ``_se2_step``, and the stepper's firmware (ideal rolling IK of the body twist, clipped to
phi_max), ported over this domain's assembly and profile instead of the platform's module graph and World. The
port keeps the truth's expressions and their order; its departures, each needed so an env reads only its own rows
(batch invariance) and each well inside 1e-9 of the original (engine/tests/test_mecanum.py, the upstream check):

- libm's tanh (numpy's SIMD tanh differs from it in the last bit);
- the wheel-speed root converges per wheel (the original stops every wheel when the slowest converges);
- wheel sums are sequential and the 3x3 solve is explicit elimination with partial pivoting (not LAPACK);
- scope: one rigid assembly (no planar compliance), fixed normal loads (no load-transfer refresh), ball casters.

It adds what the batch needs from the robot (the kernel has the same): command latency, faults with an onset
(weak, coast, brake, locked), the firmware's pre-fault duty, wheel angles (encoders) and the IMU.

    A = Assembly(built)                   # domains.modular.assembly.build(...)
    out = run(A, timeline, nsub=4, dt=0.005, delay=0)     # the channels of mecanum_run.replay, one env
"""

import math

import numpy as np

from engine.kernels import mecanum as K

V_MIN, V_EPS, W_EPS = K.V_MIN, K.V_EPS, K.W_EPS
tanh = K.tanh                     # libm's tanh (numpy's SIMD tanh differs in the last bit)


def _perp(v):
    v = np.asarray(v, float)
    return np.stack([-v[..., 1], v[..., 0]], -1)


class Assembly:
    """The truth model's Assembly arrays for one built assembly, with per-wheel faults."""

    def __init__(self, built, mode=None, gain=None, onset=None):
        W = built['wheels']
        n = self.n = len(W)
        arr = lambda k: np.array([w[k] for w in W], float)   # noqa: E731
        self.pos = np.stack([arr('px'), arr('py')], -1)
        self.E = np.stack([arr('ex'), arr('ey')], -1)
        self.T = np.stack([arr('tx'), arr('ty')], -1)
        self.Fd = _perp(self.T)
        for k in ('r', 'r_fw', 'mu_t', 'mu_f', 's_x0', 'alpha0', 'c_rr', 'N', 'K', 'tau_stall', 'w_nl', 'b', 'tau_c',
                  'tau_bd', 'phimax'):
            setattr(self, k, arr(k))
        self.kind = arr('kind').astype(int)
        self.is_mec = self.kind == K.MEC
        self.is_caster = self.kind == K.BALL
        b = built['body']
        self.mass, self.com, self.I_o = b['mass'], np.array([b['xc'], b['yc']]), b['inertia']
        xc, yc = self.com
        mt = self.mass
        self.M = np.array([[mt, 0, -mt * yc], [0, mt, mt * xc], [-mt * yc, mt * xc, self.I_o]])
        self.imu = np.array([b['imu_x'], b['imu_y']])
        self.cpr = b['cpr']
        self.mode = np.zeros(n, int) if mode is None else np.asarray(mode, int)
        self.gain = np.ones(n) if gain is None else np.asarray(gain, float)
        self.onset = np.zeros(n) if onset is None else np.asarray(onset, float)

    def at(self, t):
        """(tau_stall, w_nl, powered, spins, brake) at time t: the faults whose onset has passed."""
        mode = np.where((self.mode != K.OK) & (t >= self.onset), self.mode, 0)
        weak = mode == K.WEAK
        ts = np.where(weak, self.gain * self.tau_stall, self.tau_stall)
        wnl = np.where(weak, self.gain * self.w_nl, self.w_nl)
        spins = ~self.is_caster & (mode != K.LOCKED)
        powered = spins & (mode != K.COAST) & (mode != K.BRAKE)
        return ts, wnl, powered, spins, mode == K.BRAKE


def _traction(A, vc, om, N, E, T, Fd):
    r = A.r
    rw = r * om
    vt = (T * vc).sum(-1)
    te = (T * E).sum(-1)
    s_t = vt - rw * te
    vh = vc - rw[..., None] * E
    v_f = (Fd * vh).sum(-1)
    V = np.maximum(np.maximum(np.abs(rw), np.sqrt((vc * vc).sum(-1))), V_MIN)
    st = s_t / (A.s_x0 * V)
    sf = v_f / (A.alpha0 * V)
    rho = np.sqrt(st * st + sf * sf) + 1e-12
    gf = tanh(rho) / rho
    Ftc = -A.mu_t * N * gf * st
    Ffc = -A.mu_f * N * gf * sf
    Ftm = -A.mu_t * N * tanh(st)
    Ffm = -A.mu_f * N * tanh(v_f / V_EPS)
    q = np.sqrt(Ftm * Ftm + Ffm * Ffm) / (A.mu_t * N + 1e-12)
    sc = np.where(q > 1.0, 1.0 / np.maximum(q, 1e-12), 1.0)
    Ft = np.where(A.is_mec, Ftm * sc, Ftc)
    Ff = np.where(A.is_mec, Ffm * sc, Ffc)
    F = Ft[..., None] * T + Ff[..., None] * Fd
    eF = (E * F).sum(-1)
    return F, eF


def _motor(A, om, cmd, f):
    ts, wnl, powered, _, brake = f
    tcmd = A.K * (cmd - om)
    motoring = tcmd * om > 0
    lim = ts * np.where(motoring, np.clip(1.0 - np.abs(om) / wnl, 0.0, 1.0), 1.0)
    tm = np.clip(tcmd, -lim, lim)
    lc = np.where(powered, A.tau_c, A.tau_bd)
    out = np.where(powered, tm, 0.0) - lc * tanh(om / W_EPS) - A.b * om
    return out - np.where(brake, A.tau_stall / A.w_nl * om, 0.0)


def solve_wheels(A, vc, cmd, N, E, T, Fd, f, n_bis=K.N_BIS, n_ill=K.N_ILL, tol=K.TOL):
    """Wheel speeds (B, n): bisection, then Illinois false position, each wheel until it converges."""
    wnl, spins = f[1], f[3]
    te = np.abs((T * E).sum(-1))
    Wb = 1.3 * wnl + 3.0 * np.sqrt((vc * vc).sum(-1)) / (A.r * np.maximum(te, 0.3)) + 2.0
    Wb = Wb + np.abs(cmd)

    def h(om):
        _, eF = _traction(A, vc, om, N, E, T, Fd)
        return _motor(A, om, cmd, f) - A.r * eF
    lo, hi = -Wb, Wb.copy()
    hlo, hhi = h(lo), h(hi)
    for _ in range(n_bis):
        mid = 0.5 * (lo + hi)
        hm = h(mid)
        pos = hm > 0
        lo, hlo = np.where(pos, mid, lo), np.where(pos, hm, hlo)
        hi, hhi = np.where(pos, hi, mid), np.where(pos, hhi, hm)
    side = np.zeros(lo.shape)
    x = 0.5 * (lo + hi)
    done = np.broadcast_to(~spins, lo.shape).copy()
    for _ in range(n_ill):
        den = hlo - hhi
        xn = np.where(np.abs(den) > 1e-300, (lo * (-hhi) + hi * hlo) / np.where(np.abs(den) > 1e-300, den, 1.0),
                      0.5 * (lo + hi))
        xn = np.clip(xn, np.minimum(lo, hi), np.maximum(lo, hi))
        hx = h(xn)
        pos = hx > 0
        lo_new = np.where(pos, xn, lo)
        hi_new = np.where(pos, hi, xn)
        hlo_new = np.where(pos, hx, np.where(side == -1, hlo * 0.5, hlo))
        hhi_new = np.where(pos, np.where(side == 1, hhi * 0.5, hhi), hx)
        side_new = np.where(pos, 1, -1)
        go = ~done
        x, lo, hi = np.where(go, xn, x), np.where(go, lo_new, lo), np.where(go, hi_new, hi)
        hlo, hhi, side = np.where(go, hlo_new, hlo), np.where(go, hhi_new, hhi), np.where(go, side_new, side)
        done |= go & ((np.abs(hx) < tol) | (np.abs(hi_new - lo_new) < 1e-13))
        if done.all():
            break
    return np.where(spins[None], x, 0.0)


def _rowsum(a):
    s = a[:, 0]
    for j in range(1, a.shape[1]):
        s = s + a[:, j]
    return s


def contact(A, xi, cmd_full, f):
    """Contact wrench (B, 3) and wheel speeds (B, n) for body twists xi (B, 3)."""
    xi = np.atleast_2d(xi)
    B = xi.shape[0]
    N = np.broadcast_to(A.N, (B, A.n))
    p = A.pos[None]
    vc = xi[:, None, :2] + xi[:, None, 2:3] * _perp(p)
    E, T, Fd = (np.broadcast_to(v[None], (B, A.n, 2)) for v in (A.E, A.T, A.Fd))
    om = solve_wheels(A, vc, cmd_full, N, E, T, Fd, f)
    F, _ = _traction(A, vc, om, N, E, T, Fd)
    ve = (E * vc).sum(-1)
    F = F - (A.c_rr * N * tanh(ve / V_EPS))[..., None] * E
    cm = A.is_caster
    if cm.any():
        sp = np.sqrt((vc * vc).sum(-1, keepdims=True) + K.VEPS2)
        Fc = -(A.c_rr * N)[..., None] * vc / sp
        F = np.where(cm[None, :, None], Fc, F)
    F = np.where((N > 0)[..., None], F, 0.0)
    Wr = np.stack([_rowsum(F[..., 0]), _rowsum(F[..., 1]), _rowsum(p[..., 0] * F[..., 1] - p[..., 1] * F[..., 0])], -1)
    return Wr, om


def _bias(A, xi):
    xi = np.atleast_2d(xi)
    c = A.com
    w = xi[:, 2]
    vcx = xi[:, 0] - w * c[1]
    vcy = xi[:, 1] + w * c[0]
    bl = A.mass * np.stack([-w * vcy, w * vcx], -1)
    ba = c[0] * bl[:, 1] - c[1] * bl[:, 0]
    return np.concatenate([bl, ba[:, None]], 1)


def _solve3(A, b):
    A, b = np.array(A, float), np.array(b, float)
    for k in range(3):
        for i in range(k + 1, 3):
            if abs(A[i, k]) > abs(A[k, k]):
                A[[k, i]] = A[[i, k]]
                b[[k, i]] = b[[i, k]]
        for i in range(k + 1, 3):
            m = A[i, k] / A[k, k]
            A[i, k:] = A[i, k:] - m * A[k, k:]
            b[i] = b[i] - m * b[k]
    x2 = b[2] / A[2, 2]
    x1 = (b[1] - A[1, 2] * x2) / A[1, 1]
    x0 = (b[0] - A[0, 1] * x1 - A[0, 2] * x2) / A[0, 0]
    return np.array([x0, x1, x2])


def se2_step(pose, xi, dt):
    x, y, th = pose
    vx, vy, w = xi * dt
    if abs(w) < 1e-9:
        dx, dy = vx, vy
    else:
        s, c = math.sin(w), math.cos(w)
        dx = (s * vx - (1 - c) * vy) / w
        dy = ((1 - c) * vx + s * vy) / w
    cs, sn = math.cos(th), math.sin(th)
    return (x + cs * dx - sn * dy, y + sn * dx + cs * dy, th + w)


def firmware(A, xi_cmd):
    """The base firmware: ideal rolling wheel-speed commands for a body twist, clipped to phi_max (0 on casters)."""
    xi = np.atleast_2d(np.asarray(xi_cmd, float))
    vc = xi[:, None, :2] + xi[:, None, 2:3] * _perp(A.pos)[None]
    te = (A.T * A.E).sum(-1)
    w = (A.T[None] * vc).sum(-1) / (A.r_fw * np.where(np.abs(te) > 1e-6, te, 1.0))
    w = np.clip(w, -A.phimax, A.phimax)
    return np.where(A.is_caster[None], 0.0, w)[0]


def plant_step(A, pose, xi, cmdf, t, dt):
    """simulate_assembly's step: the FD Jacobian of the contact wrench, linearly-implicit Euler, SE(2) pose."""
    f = A.at(t)
    X = np.stack([xi, xi + [1e-5, 0, 0], xi + [0, 1e-5, 0], xi + [0, 0, 1e-5]])
    Wc, om = contact(A, X, np.repeat(cmdf[None], 4, 0), f)
    Wr = Wc - _bias(A, X)
    J = np.stack([(Wr[i + 1] - Wr[0]) / 1e-5 for i in range(3)], -1)
    dxi = _solve3(A.M - dt * J, dt * Wr[0])
    xi_new = xi + dxi
    return se2_step(pose, xi_new, dt), xi_new, dxi / dt, om[0]


def duty(A, cmdf, om):
    """The firmware's pre-fault duty: tau / tau_stall + w / w_nl of the nominal motor, clipped to [-1, 1]."""
    tcmd = A.K * (cmdf - om)
    lim = A.tau_stall * np.where(tcmd * om > 0, np.clip(1.0 - np.abs(om) / A.w_nl, 0.0, 1.0), 1.0)
    tm = np.clip(tcmd, -lim, lim)
    return np.where(A.is_caster, 0.0, np.clip(tm / A.tau_stall + om / A.w_nl, -1.0, 1.0))


def step(A, pose, xi_cmd, dt, state=None, sub_dt=0.005):
    """The stepper's contract, dynamic mode: (new_pose, info, state) after dt under a held body-twist command."""
    state = dict(state or {})
    xi = state.get('xi', np.zeros(3))
    k = state.get('k', 0)
    cmdf = firmware(A, xi_cmd)
    for _ in range(max(1, int(round(dt / sub_dt)))):
        pose, xi, acc, om = plant_step(A, tuple(pose), xi, cmdf, (k + K.ONSET_EPS) * sub_dt, sub_dt)
        k += 1
    state.update(xi=xi, k=k, t=k * sub_dt, acc=acc, om=om)
    return tuple(float(v) for v in pose), {'twist': xi, 'wheel_cmd': cmdf, 'om': om}, state


def run(A, timeline, nsub=4, dt=0.005, delay=0, ticks=None, length=None):
    """One env replaying a timeline [T, 3] (twist per tick): the channels of mecanum_run.replay, [T, ...] each."""
    tl = np.asarray(timeline, float)
    L = len(tl) if length is None else int(length)
    T = int(ticks if ticks is not None else L)
    pose, xi, acc = (0.0, 0.0, 0.0), np.zeros(3), np.zeros(3)
    ang = np.zeros(A.n)
    out = {k: [] for k in ('cmd', 'pose', 'twist', 'duty', 'speed', 'counts', 'gz', 'ax', 'ay')}
    for j in range(T):
        for s in range(nsub):
            k = j * nsub + s
            idx = (k - delay) // nsub
            u = tl[idx] if 0 <= idx < L else np.zeros(3)
            cmdf = firmware(A, u)
            pose, xi, acc, om = plant_step(A, pose, xi, cmdf, (k + K.ONSET_EPS) * dt, dt)
            ang = ang + om * dt
            d = duty(A, cmdf, om)
        wz = xi[2]
        rx, ry = A.imu
        out['cmd'].append(tl[j] if j < L else np.zeros(3))
        out['pose'].append(pose)
        out['twist'].append(xi)
        out['duty'].append(d)
        out['speed'].append(om)
        out['counts'].append(np.floor(ang / (2 * math.pi) * A.cpr))
        out['gz'].append(wz)
        out['ax'].append(acc[0] - wz * xi[1] - acc[2] * ry - wz * wz * rx)
        out['ay'].append(acc[1] + wz * xi[0] + acc[2] * rx - wz * wz * ry)
    return {k: np.array(v, float) for k, v in out.items()}
