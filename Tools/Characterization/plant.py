"""Planar boat plants for crane_ml physics characterization.

Two models live here:

* ``ProfilePlant``: the exact twin of the C# ``MarinePlanarModel`` (design section 2). It is built
  from a profile JSON dict (``profiles/ideal.json`` schema) and steps in double precision exactly as
  Unity/PhysX does with the force the plant applies: linearized-implicit damping through the added-mass
  coupling, then symplectic Euler in the world frame. ``reference_run`` is the RK4 reference of the
  continuous ODE (default step 1 ms). ``ThrusterModel`` is the PWM thruster of section 2.
* ``VesselModel``: the identification fixture (from the owner's adversarial WIP) that the estimators,
  faults and excitation modules use. Its mass, yaw inertia, hull drag and thrust gains come from
  ``profiles/ideal.json``; its thruster mounts are FIXTURE-ONLY values, not the boat's geometry.

Frames: world ENU, psi CCW from +x. Body FLU at the CG: nu = [u, v, r].
"""
from __future__ import annotations

import json
import math
import os
from dataclasses import dataclass

HERE = os.path.dirname(os.path.abspath(__file__))
IDEAL_PATH = os.path.join(HERE, "profiles", "ideal.json")


def load_profile(path=IDEAL_PATH) -> dict:
    with open(path) as f:
        return json.load(f)


IDEAL = load_profile()

# ======================================================================================
# Exact twin of MarinePlanarModel
# ======================================================================================
AXES = ("surge", "sway", "yaw")
PWM_NEUTRAL, PWM_MIN, PWM_MAX, PWM_SPAN = 1500.0, 1100.0, 1900.0, 400.0


def validate_profile(profile: dict):
    """Raise on parameters the plant cannot run with (design section 7 edge cases)."""
    def finite(name, x):
        if not isinstance(x, (int, float)) or not math.isfinite(x):
            raise ValueError(f"{name} is not finite: {x!r}")
    for key in ("mass", "izz"):
        finite(key, profile[key])
        if profile[key] <= 0:
            raise ValueError(f"{key} must be > 0")
    for k, x in profile["addedMass"].items():
        finite(f"addedMass.{k}", x)
    for k in ("a11", "a22", "a66"):
        if profile["addedMass"][k] < 0:
            raise ValueError(f"addedMass.{k} must be >= 0")
    for axis in AXES:
        for k, x in profile[axis].items():
            finite(f"{axis}.{k}", x)
            if x < 0:
                raise ValueError(f"{axis}.{k} must be >= 0")
    for t in profile["thrusters"]:
        for k, x in t.items():
            if k != "name":
                finite(f"{t['name']}.{k}", x)
        if t["kappa"] != 0.0:
            raise ValueError(f"{t['name']}: kappa must be 0 (idle-prop drag is not modelled yet)")
        if t["tauT"] < 0 or t["deadTime"] < 0 or t["deadbandUs"] < 0:
            raise ValueError(f"{t['name']}: negative tauT, deadTime or deadbandUs")


def effective_mass(profile: dict):
    """M_eff per axis (surge, sway, yaw) = rigid body + diagonal added mass."""
    am = profile["addedMass"]
    return (profile["mass"] + am["a11"], profile["mass"] + am["a22"], profile["izz"] + am["a66"])


def damping_axis(c: dict, s: float):
    """(D_j, J_j) for one axis: D = (D1^sigma + D2^sigma |s|) s, J = D1^sigma + 2 D2^sigma |s|;
    J = max(D1Pos, D1Neg) at s == 0 exactly."""
    if s > 0.0:
        d1, d2 = c["d1Pos"], c["d2Pos"]
    elif s < 0.0:
        d1, d2 = c["d1Neg"], c["d2Neg"]
    else:
        return 0.0, max(c["d1Pos"], c["d1Neg"])
    a = abs(s)
    return (d1 + d2 * a) * s, d1 + 2.0 * d2 * a


def step_term(meff: float, J: float, dt: float) -> float:
    """c*J, which stands where dt*J stood in Mtil: M_eff (x/(1 - e^-x) - 1), x = dt J/M_eff.
    That is the review's h = (M_eff/J)(e^x - 1) with the acceleration scaled by h/dt (Unity
    steps dt), so linearized decay is exactly e^-x per tick. Twin of MarinePlanarModel.StepTerm."""
    x = dt * J / meff
    if not x > 0.0:
        return 0.0
    return meff * (x / 2.0 + x * x / 12.0 if x < 1e-4 else x / -math.expm1(-x) - 1.0)


def accuracy_ratio(profile: dict, dt: float):
    """dt*J(1.5 v_max)/M_eff per axis, worst sign. Above 0.5 is the design's accuracy limit
    (implicit damping stays stable there, it just stops being accurate)."""
    meff = effective_mass(profile)
    out = []
    for j, axis in enumerate(AXES):
        c, vmax = profile[axis], 1.5 * profile["envelope"][axis]
        J = max(damping_axis(c, vmax)[1], damping_axis(c, -vmax)[1])
        out.append(dt * J / meff[j])
    return tuple(out)


def c_a_times(am: dict, ur: float, vr: float, r: float):
    """C_A(nu_r) nu_r with r_r = r."""
    a11, a22, a26 = am["a11"], am["a22"], am["a26"]
    w = a22 * vr + a26 * r
    return (-w * r, a11 * ur * r, w * ur - a11 * ur * vr)


def c_rb_times(m: float, u: float, v: float, r: float):
    """C_RB(nu) nu."""
    return (-m * v * r, m * u * r, 0.0)


def steady_speed(d1: float, d2: float, T: float) -> float:
    """Steady speed under a constant force T >= 0."""
    if d2 == 0.0:
        return T / d1
    return (-d1 + math.sqrt(d1 * d1 + 4.0 * d2 * T)) / (2.0 * d2)


class ThrusterModel:
    """PWM thruster (design section 2). One ``step`` call per tick."""

    def __init__(self, spec: dict, dt: float):
        if spec["kappa"] != 0.0:
            raise ValueError(f"{spec['name']}: kappa must be 0")
        self.spec = spec
        self.name = spec["name"]
        self.dt = dt
        self.alpha = math.radians(spec["alphaDeg"])
        self.cos_a, self.sin_a = math.cos(self.alpha), math.sin(self.alpha)
        # round() is round-half-to-even, as C# Math.Round's default.
        self.n_d = int(round(spec["deadTime"] / dt))
        self.dead_time_rounding_error = spec["deadTime"] - self.n_d * dt
        tau = spec["tauT"]
        self.lag_gain = 1.0 if tau == 0.0 else 1.0 - math.exp(-dt / tau)
        self.events = 0
        self.reset()

    def reset(self):
        """Clears the lag state, the dead-time buffer and the last outputs (not the event count)."""
        self.T = 0.0
        self.buf = [0.0] * self.n_d
        self.idx = 0
        self.q = 0.0
        self.q_d = 0.0

    def step(self, p: float, beta: float = 1.0, voltage=None):
        """PWM pulse p [us] -> body (Fx, Fy, N) for this tick."""
        s = self.spec
        if not math.isfinite(p):
            p = PWM_NEUTRAL
            self.events += 1
        p = min(max(p, PWM_MIN), PWM_MAX)
        e = p - PWM_NEUTRAL - s["offsetUs"]
        q = 0.0 if abs(e) <= s["deadbandUs"] else min(max(e / PWM_SPAN, -1.0), 1.0)
        if self.n_d == 0:
            qd = q
        else:  # ring buffer holds q_{k-n_d} .. q_{k-1}; read the oldest, write the newest there
            qd = self.buf[self.idx]
            self.buf[self.idx] = q
            self.idx = (self.idx + 1) % self.n_d
        if qd > 0.0:
            t_ss = s["kForward"] * qd * qd
        elif qd < 0.0:
            t_ss = -s["kReverse"] * qd * qd
        else:
            t_ss = 0.0
        V = s["vRef"] if voltage is None else voltage
        t_ss *= (V / s["vRef"]) ** s["nV"]
        self.T += self.lag_gain * (t_ss - self.T)
        self.q, self.q_d = q, qd
        F = beta * self.T
        fx, fy = F * self.cos_a, F * self.sin_a
        return fx, fy, s["x"] * fy - s["y"] * fx


class ProfilePlant:
    """Exact twin of MarinePlanarModel. State [x, y, psi, vx, vy, r] (world velocity)."""

    def __init__(self, profile: dict, dt: float, current=(0.0, 0.0)):
        validate_profile(profile)
        self.profile = profile
        self.dt = float(dt)
        self.current = (float(current[0]), float(current[1]))
        self.m, self.iz = float(profile["mass"]), float(profile["izz"])
        self.am = dict(profile["addedMass"])
        self.damp = tuple(profile[a] for a in AXES)
        self.thrusters = [ThrusterModel(t, self.dt) for t in profile["thrusters"]]
        self.reset()

    def reset(self, state=(0.0, 0.0, 0.0, 0.0, 0.0, 0.0)):
        self.state = [float(x) for x in state]
        self.t = 0.0
        self.k = 0
        for th in self.thrusters:
            th.reset()

    def body_velocity(self, state=None):
        x, y, psi, vx, vy, r = self.state if state is None else state
        c, s = math.cos(psi), math.sin(psi)
        return (c * vx + s * vy, -s * vx + c * vy, r)

    def relative_velocity(self, state=None):
        st = self.state if state is None else state
        u, v, r = self.body_velocity(st)
        c, s = math.cos(st[2]), math.sin(st[2])
        cx, cy = self.current
        return (u - (c * cx + s * cy), v - (-s * cx + c * cy), r)

    def energy(self, state=None):
        """1/2 nu_r^T M nu_r with M = M_RB + M_A."""
        ur, vr, r = self.relative_velocity(state)
        am = self.am
        m11, m22, m66 = self.m + am["a11"], self.m + am["a22"], self.iz + am["a66"]
        return 0.5 * (m11 * ur * ur + m22 * vr * vr + m66 * r * r + 2.0 * am["a26"] * vr * r)

    def thrust(self, pwm, beta=None, voltage=None):
        """Sum of thruster wrenches for one tick (advances thruster state)."""
        X = Y = N = 0.0
        for i, th in enumerate(self.thrusters):
            b = 1.0 if beta is None else beta[i]
            fx, fy, n = th.step(pwm[i], b, voltage)
            X += fx
            Y += fy
            N += n
        return (X, Y, N)

    def step_pwm(self, pwm, beta=None, voltage=None):
        tau = self.thrust(pwm, beta, voltage)
        self.step_wrench(tau)
        return tau

    def step_wrench(self, tau):
        """One tick with body thrust tau = [X, Y, N]."""
        dt, m, iz, am = self.dt, self.m, self.iz, self.am
        a11, a22, a26, a66 = am["a11"], am["a22"], am["a26"], am["a66"]
        x, y, psi, vx, vy, r = self.state
        c, s = math.cos(psi), math.sin(psi)
        u, v = c * vx + s * vy, -s * vx + c * vy
        cx, cy = self.current
        uc, vc = c * cx + s * cy, -s * cx + c * cy
        ur, vr = u - uc, v - vc
        ndc0, ndc1 = r * vc, -r * uc  # nudot_c, third component 0
        D0, J0 = damping_axis(self.damp[0], ur)
        D1, J1 = damping_axis(self.damp[1], vr)
        D2, J2 = damping_axis(self.damp[2], r)
        ca0, ca1, ca2 = c_a_times(am, ur, vr, r)
        # 1. tau_h = tau + M_A nudot_c - C_A(nu_r) nu_r - D(nu_r)
        th0 = tau[0] + a11 * ndc0 - ca0 - D0
        th1 = tau[1] + a22 * ndc1 - ca1 - D1
        th2 = tau[2] + a26 * ndc1 - ca2 - D2
        # 2. Mtil = M_RB + M_A + diag(c J);  x = tau_h - C_RB(nu) nu  (c J: step_term)
        x0, x1, x2 = th0 + m * v * r, th1 - m * u * r, th2
        J0 = step_term(m + a11, J0, dt)
        J1 = step_term(m + a22, J1, dt)
        J2 = step_term(iz + a66, J2, dt)
        m11 = m + a11 + J0
        m22 = m + a22 + J1
        m66 = iz + a66 + J2
        # solve(Mtil, x): Mtil is block diagonal (surge | sway-yaw) because diag(J) keeps M_A's pattern
        acc0 = x0 / m11
        det = m22 * m66 - a26 * a26
        acc1 = (m66 * x1 - a26 * x2) / det
        acc2 = (m22 * x2 - a26 * x1) / det
        # 3. F_U = tau_h - (M_A + diag(c J)) solve(Mtil, x).
        # PINNED IMPLICIT FORM: c*J sits next to M_A in this correction (math review sign-off).
        # Moving it out of the correction silently reverts the step to explicit damping.
        fu0 = th0 - (a11 + J0) * acc0
        fu1 = th1 - ((a22 + J1) * acc1 + a26 * acc2)
        fu2 = th2 - (a26 * acc1 + (a66 + J2) * acc2)
        # 4. Symplectic Euler in the world frame (what PhysX does with the applied force).
        fx, fy = c * fu0 - s * fu1, s * fu0 + c * fu1
        vx += dt * fx / m
        vy += dt * fy / m
        r += dt * fu2 / iz
        x += dt * vx
        y += dt * vy
        psi += dt * r
        self.state = [x, y, psi, vx, vy, r]
        self.t += dt
        self.k += 1
        return (fu0, fu1, fu2)

    def run_wrench(self, tau_fn, n):
        """n ticks with tau_fn(k, t) -> body tau. Returns rows [t, x, y, psi, vx, vy, r]."""
        rows = []
        for _ in range(n):
            self.step_wrench(tau_fn(self.k, self.t))
            rows.append([self.t] + self.state)
        return rows

    def run_pwm(self, pwm_fn, n, beta=None):
        rows = []
        for _ in range(n):
            self.step_pwm(pwm_fn(self.k, self.t), beta)
            rows.append([self.t] + self.state)
        return rows


def ode_rhs(profile: dict, st, tau, current=(0.0, 0.0)):
    """Continuous ODE in body coordinates, state [x, y, psi, u, v, r]:
    M nudot = tau - C_RB nu - C_A(nu_r) nu_r - D(nu_r) + M_A nudot_c;  etadot = R(psi) nu."""
    m, iz, am = profile["mass"], profile["izz"], profile["addedMass"]
    a11, a22, a26, a66 = am["a11"], am["a22"], am["a26"], am["a66"]
    x, y, psi, u, v, r = st
    c, s = math.cos(psi), math.sin(psi)
    uc, vc = c * current[0] + s * current[1], -s * current[0] + c * current[1]
    ur, vr = u - uc, v - vc
    ndc0, ndc1 = r * vc, -r * uc
    D0 = damping_axis(profile["surge"], ur)[0]
    D1 = damping_axis(profile["sway"], vr)[0]
    D2 = damping_axis(profile["yaw"], r)[0]
    ca = c_a_times(am, ur, vr, r)
    crb = c_rb_times(m, u, v, r)
    f0 = tau[0] - crb[0] - ca[0] - D0 + a11 * ndc0
    f1 = tau[1] - crb[1] - ca[1] - D1 + a22 * ndc1
    f2 = tau[2] - crb[2] - ca[2] - D2 + a26 * ndc1
    m11, m22, m66 = m + a11, m + a22, iz + a66
    det = m22 * m66 - a26 * a26
    return (c * u - s * v, s * u + c * v, r,
            f0 / m11, (m66 * f1 - a26 * f2) / det, (m22 * f2 - a26 * f1) / det)


def reference_run(profile: dict, tau_fn, t_end: float, h: float = 1e-3, state0=None,
                  current=(0.0, 0.0), sample_dt=None):
    """RK4 of the continuous ODE. tau_fn(t) -> body tau (held over each RK4 step at its start).
    state0 and the returned rows use the plant's layout [t, x, y, psi, vx, vy, r] (world velocity).
    sample_dt: keep a row every sample_dt seconds (must be a multiple of h)."""
    x, y, psi, vx, vy, r = state0 if state0 is not None else (0.0,) * 6
    c, s = math.cos(psi), math.sin(psi)
    st = (x, y, psi, c * vx + s * vy, -s * vx + c * vy, r)
    n = int(round(t_end / h))
    every = 1 if sample_dt is None else int(round(sample_dt / h))
    rows = []
    for k in range(n):
        t = k * h
        tau = tau_fn(t)
        k1 = ode_rhs(profile, st, tau, current)
        k2 = ode_rhs(profile, tuple(a + 0.5 * h * b for a, b in zip(st, k1)), tau, current)
        k3 = ode_rhs(profile, tuple(a + 0.5 * h * b for a, b in zip(st, k2)), tau, current)
        k4 = ode_rhs(profile, tuple(a + h * b for a, b in zip(st, k3)), tau, current)
        st = tuple(a + h / 6.0 * (b + 2.0 * c2 + 2.0 * d + e) for a, b, c2, d, e in zip(st, k1, k2, k3, k4))
        if (k + 1) % every == 0:
            x, y, psi, u, v, r = st
            c, s = math.cos(psi), math.sin(psi)
            rows.append([(k + 1) * h, x, y, psi, c * u - s * v, s * u + c * v, r])
    return rows


def pwm_for_effort(surge=0.0, sway=0.0, yaw=0.0, profile=None):
    """Linear per-thruster PWM for a body effort on the nominal geometry (test helper, not the
    hardware mixer): q_i = sum_j sgn(B_ij) e_j, clamped, p_i = 1500 + 400 q_i."""
    profile = profile or IDEAL
    out = []
    for t in profile["thrusters"]:
        a = math.radians(t["alphaDeg"])
        col = (math.cos(a), math.sin(a), t["x"] * math.sin(a) - t["y"] * math.cos(a))
        q = sum(math.copysign(1.0, cj) * ej for cj, ej in zip(col, (surge, sway, yaw)) if abs(cj) > 1e-9)
        out.append(PWM_NEUTRAL + PWM_SPAN * min(max(q, -1.0), 1.0))
    return out


# ======================================================================================
# Identification fixture (owner's WIP adv_plant, defaults from profiles/ideal.json)
# ======================================================================================
DT = IDEAL["fixedDeltaTime"]  # s

MASS = IDEAL["mass"]                                  # kg
IZZ = IDEAL["izz"]                                    # kg m^2
T200_FWD = IDEAL["thrusters"][0]["kForward"]          # N at full forward
T200_REV = IDEAL["thrusters"][0]["kReverse"]          # N at full reverse
T200_SPINUP = 0.1  # s, the T200 spin-up prior of design section 3 (Ideal itself uses tauT = 0)
DRAG_SURGE = (IDEAL["surge"]["d1Pos"], IDEAL["surge"]["d2Pos"])  # N/(m/s), N/(m/s)^2
DRAG_SWAY = (IDEAL["sway"]["d1Pos"], IDEAL["sway"]["d2Pos"])
DRAG_YAW = (IDEAL["yaw"]["d1Pos"], IDEAL["yaw"]["d2Pos"])        # N m/(rad/s), N m/(rad/s)^2


def drag(velocity: float, c) -> float:
    """-(lin + quad*|v|)*v."""
    return -(c[0] + c[1] * abs(velocity)) * velocity


@dataclass(frozen=True)
class ThrusterSpec:
    """One fixture thruster. x, y, axis_deg are FIXTURE-ONLY (body frame, x fore, y port)."""
    name: str
    x: float
    y: float
    axis_deg: float
    sign: int = 1               # wiring sign applied to the command
    tau: float = T200_SPINUP    # s, first-order lag
    deadband: float = 0.0       # |cmd| below this is zeroed
    f_fwd: float = T200_FWD
    f_rev: float = T200_REV
    strength: float = 1.0
    mount_yaw_deg: float = 0.0  # CCW positive in this fixture


# FIXTURE-ONLY, not physical: X layout, orthogonal surge/sway/yaw columns.
NOMINAL_THRUSTERS = (
    ThrusterSpec("FL", 0.4, 0.3, -45.0),
    ThrusterSpec("FR", 0.4, -0.3, 45.0),
    ThrusterSpec("RR", -0.4, -0.3, -45.0),
    ThrusterSpec("RL", -0.4, 0.3, 45.0),
)


def column(spec: ThrusterSpec):
    """Unit-force wrench column (Fx, Fy, Nz) for a thruster, including mount yaw."""
    a = math.radians(spec.axis_deg + spec.mount_yaw_deg)
    s, c = math.sin(a), math.cos(a)
    return (c, s, spec.x * s - spec.y * c)


def clamp01(x: float) -> float:
    return 0.0 if x < 0.0 else 1.0 if x > 1.0 else x


@dataclass
class ThrustReport:
    """Per-step thrust accounting. requested -> allocated -> applied -> achieved."""
    t: float
    requested_cmd: tuple
    allocated_cmd: tuple
    saturated: tuple
    allocated_force: tuple
    applied_force: tuple
    achieved_wrench: tuple


@dataclass
class Allocation:
    requested_wrench: tuple
    commands: tuple
    scale: float
    saturated: bool
    steady_wrench: tuple


class VesselModel:
    """Base fixture plant (RK4 per tick, diagonal M). Hooks are used by the fault layer."""

    def __init__(self, thrusters=NOMINAL_THRUSTERS, dt: float = DT, mass: float = MASS,
                 izz: float = IZZ, added_mass=(0.0, 0.0, 0.0), drag_on: bool = True):
        self.thrusters = tuple(thrusters)
        self.added_mass = tuple(added_mass)
        self.drag_on = drag_on
        self.dt = dt
        self.mass = mass
        self.izz = izz
        self.reset()

    def reset(self, state=(0.0, 0.0, 0.0, 0.0, 0.0, 0.0), spools=None):
        self.state = list(state)  # x, y, psi, u, v, r (world pose, body velocities)
        self.spools = list(spools) if spools is not None else [0.0] * len(self.thrusters)
        self.t = 0.0
        self.k = 0

    def set_heading(self, psi: float):
        """Teleport the heading, keeping the WORLD velocity."""
        x, y, p, u, v, r = self.state
        c0, s0 = math.cos(p), math.sin(p)
        wx, wy = c0 * u - s0 * v, s0 * u + c0 * v
        c1, s1 = math.cos(psi), math.sin(psi)
        self.state = [x, y, psi, c1 * wx + s1 * wy, -s1 * wx + c1 * wy, r]

    def _delay_command(self, cmd):
        return cmd

    def _map_commands(self, cmd):
        return cmd

    def _effective(self, spec: ThrusterSpec, cmd: float):
        """clamp -> wiring sign -> deadband. Returns (effective command, saturated flag)."""
        sat = abs(cmd) > 1.0
        c = -1.0 if cmd < -1.0 else 1.0 if cmd > 1.0 else cmd
        c *= spec.sign
        if abs(c) < spec.deadband:
            c = 0.0
        return c, sat

    def _force(self, spec: ThrusterSpec, s: float) -> float:
        """sign(s)*s^2*K, reverse scaled by f_rev/f_fwd."""
        f = spec.f_fwd * spec.strength * s * abs(s)
        return f * (spec.f_rev / spec.f_fwd) if s < 0 else f

    def _current_world(self, t: float):
        return (0.0, 0.0)

    def _external(self, t: float, ur: float, vr: float, r: float):
        return (0.0, 0.0, 0.0)

    def _drag_terms(self, ur: float, vr: float, r: float, psi: float):
        return (drag(ur, DRAG_SURGE), drag(vr, DRAG_SWAY), drag(r, DRAG_YAW))

    def _inertia(self):
        return (self.mass + self.added_mass[0], self.mass + self.added_mass[1],
                self.izz + self.added_mass[2])

    def _water_body(self, psi: float, cx: float, cy: float):
        c, s = math.cos(psi), math.sin(psi)
        return (c * cx + s * cy, -s * cx + c * cy)

    def _accel(self, X: float, Y: float, N: float, u: float, v: float, r: float):
        """Diagonal-M dynamics with workless Coriolis (incl. the added-mass Munk moment)."""
        mu, mv, mr = self._inertia()
        return ((X + mv * v * r) / mu, (Y - mu * u * r) / mv, (N + (mu - mv) * u * v) / mr)

    def _deriv(self, t, st, W):
        x, y, psi, u, v, r = st
        cx, cy = self._current_world(t)
        c, s = math.cos(psi), math.sin(psi)
        wu, wv = self._water_body(psi, cx, cy)
        ur = u - wu
        vr = v - wv
        dX, dY, dN = self._drag_terms(ur, vr, r, psi)
        ex, ey, en = self._external(t, ur, vr, r)
        if not self.drag_on:
            dX = dY = dN = ex = ey = en = 0.0
        du, dv, dr = self._accel(W[0] + dX + ex, W[1] + dY + ey, W[2] + dN + en, u, v, r)
        return (c * u - s * v, s * u + c * v, r, du, dv, dr)

    def step(self, cmd) -> ThrustReport:
        requested = tuple(cmd)
        cmd = self._map_commands(self._delay_command(requested))
        eff, sats = [], []
        for spec, c in zip(self.thrusters, cmd):
            e, sat = self._effective(spec, c)
            eff.append(e)
            sats.append(sat)
        for i, spec in enumerate(self.thrusters):
            tau = max(spec.tau, self.dt)
            self.spools[i] += (eff[i] - self.spools[i]) * clamp01(self.dt / tau)
        applied = [self._force(sp, s) for sp, s in zip(self.thrusters, self.spools)]
        alloc_f = [self._force(sp, e) for sp, e in zip(self.thrusters, eff)]
        W = self.wrench(applied)
        y0 = tuple(self.state)
        t0, h = self.t, self.dt
        k1 = self._deriv(t0, y0, W)
        k2 = self._deriv(t0 + h / 2, tuple(a + h / 2 * b for a, b in zip(y0, k1)), W)
        k3 = self._deriv(t0 + h / 2, tuple(a + h / 2 * b for a, b in zip(y0, k2)), W)
        k4 = self._deriv(t0 + h, tuple(a + h * b for a, b in zip(y0, k3)), W)
        self.state = [a + h / 6 * (b + 2 * c2 + 2 * d + e)
                      for a, b, c2, d, e in zip(y0, k1, k2, k3, k4)]
        self.t = t0 + h
        self.k += 1
        return ThrustReport(self.t, requested, tuple(eff), tuple(sats), tuple(alloc_f),
                            tuple(applied), tuple(W))

    def wrench(self, forces):
        Fx = Fy = N = 0.0
        for spec, f in zip(self.thrusters, forces):
            cx, cy, cn = column(spec)
            Fx += f * cx
            Fy += f * cy
            N += f * cn
        return (Fx, Fy, N)

    def allocate(self, wrench) -> Allocation:
        """Pseudo-inverse allocation on the NOMINAL columns, then uniform desaturation in force
        space. The model's real (faulted) columns are deliberately not used."""
        import numpy as np
        B = np.array([column(s) for s in NOMINAL_THRUSTERS]).T  # 3 x 4
        f = np.linalg.pinv(B) @ np.asarray(wrench, dtype=float)
        lam = 1.0
        for spec, fi in zip(self.thrusters, f):
            cap = spec.f_rev if fi < 0 else spec.f_fwd
            if abs(fi) > cap:
                lam = min(lam, cap / abs(fi))
        f = f * lam
        cmds = []
        for spec, fi in zip(self.thrusters, f):
            cap = spec.f_rev if fi < 0 else spec.f_fwd
            cmds.append(math.copysign(math.sqrt(abs(fi) / cap), fi) * spec.sign)
        cmds = tuple(max(-1.0, min(1.0, c)) for c in cmds)
        steady = self.steady_wrench(cmds)
        return Allocation(tuple(wrench), cmds, lam, lam < 1.0, steady)

    def steady_wrench(self, cmds):
        eff = [self._effective(sp, c)[0] for sp, c in zip(self.thrusters, self._map_commands(tuple(cmds)))]
        return self.wrench([self._force(sp, e) for sp, e in zip(self.thrusters, eff)])

    def settle(self, cmds, velocity=(0.0, 0.0, 0.0)):
        eff = [self._effective(sp, c)[0] for sp, c in zip(self.thrusters, self._map_commands(tuple(cmds)))]
        self.spools = eff
        self.state[3:6] = list(velocity)

    def run(self, n: int, cmd_fn):
        """Run n steps. cmd_fn(k, t, model) -> 4 commands. Returns a Log."""
        import numpy as np
        rows, W, sat, fa = [], [], [], []
        for _ in range(n):
            rep = self.step(cmd_fn(self.k, self.t, self))
            rows.append([self.t] + self.state)
            W.append(rep.achieved_wrench)
            sat.append(any(rep.saturated))
            fa.append(rep.applied_force)
        a = np.array(rows)
        return Log(a[:, 0], a[:, 1], a[:, 2], a[:, 3], a[:, 4], a[:, 5], a[:, 6],
                   np.array(W), np.array(sat), np.array(fa), self.dt)


@dataclass
class Log:
    t: object
    x: object
    y: object
    psi: object
    u: object
    v: object
    r: object
    wrench: object
    saturated: object
    applied_force: object
    dt: float

    def array(self):
        import numpy as np
        return np.column_stack([self.t, self.x, self.y, self.psi, self.u, self.v, self.r])
