"""Analytic slope/traction references for CRANE ground-vehicle terrain fixtures.

Pure Python, no third-party dependencies. Every function is a closed-form or
quasi-static rigid-body reference intended as an *oracle* for simulator tests;
none of it is a calibrated vehicle model. Conventions:

* SI units, angles in radians unless a name ends in ``_deg``.
* Body frame x forward, y left, z up. ``theta`` is longitudinal pitch of the
  supporting plane (positive = nose up / driving uphill); ``phi`` is the
  cross-slope roll (positive = left side up).
* ``W`` is weight in newtons (m * g).
"""
from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Callable, Iterable, Sequence

G = 9.81


# ---------------------------------------------------------------------------
# 1. Static friction slope limit
# ---------------------------------------------------------------------------
def static_hold_limit(mu_s: float, braked_load_fraction: float = 1.0) -> float:
    """Steepest plane a rigid body can rest on without sliding.

    With all wheels braked (or a Coulomb block) the limit is tan(theta) <= mu.
    If only a fraction ``f`` of the normal load sits on braked/driven wheels
    (the rest on free casters), tan(theta) <= f * mu to first order. For a
    load fraction that itself depends on theta use ``max_climb_traction``.
    """
    if mu_s < 0 or not 0 <= braked_load_fraction <= 1:
        raise ValueError("mu_s must be >=0 and load fraction in [0,1]")
    return math.atan(mu_s * braked_load_fraction)


def holds_on_slope(theta: float, mu_s: float, braked_load_fraction: float = 1.0) -> bool:
    return math.tan(abs(theta)) <= mu_s * braked_load_fraction + 1e-12


# ---------------------------------------------------------------------------
# 2. Load transfer on a slope and tip-over
# ---------------------------------------------------------------------------
def longitudinal_axle_loads(W: float, wheelbase: float, cg_from_rear: float,
                            cg_height: float, theta: float,
                            ax: float = 0.0) -> tuple[float, float]:
    """Quasi-static (front, rear) normal loads on a plane pitched by ``theta``.

    ``ax`` is the body's along-slope acceleration (positive forward). Moments
    about the rear contact: N_f * L = W (x_r cos(theta) - h sin(theta)) - m ax h.
    A negative front load means the front has lifted (longitudinal tip-over).
    """
    m = W / G
    nf = (W * (cg_from_rear * math.cos(theta) - cg_height * math.sin(theta)) - m * ax * cg_height) / wheelbase
    nr = W * math.cos(theta) - nf
    return nf, nr


def lateral_side_loads(W: float, track: float, cg_lateral: float, cg_height: float,
                       phi: float, ay: float = 0.0) -> tuple[float, float]:
    """(left, right) normal loads on a cross slope rolled by ``phi`` (left up).

    ``cg_lateral`` is the CG offset toward +y (left). ``ay`` is lateral
    acceleration toward +y (e.g. centripetal for a left turn).
    """
    m = W / G
    half = track / 2.0
    # Moments about the right contact line (y = -half).
    nl = (W * ((half + cg_lateral) * math.cos(phi) - cg_height * math.sin(phi)) - m * ay * cg_height) / track
    nr = W * math.cos(phi) - nl
    return nl, nr


def lateral_tip_angle(track: float, cg_height: float, cg_lateral: float = 0.0,
                      downhill_side: str = "right") -> float:
    """Static cross-slope roll angle at which the uphill wheels unload.

    For a rigid chassis (no suspension compliance) the tip angle is
    atan(lever / h), with lever = track/2 -/+ lateral CG offset depending on
    which side is downhill.
    """
    half = track / 2.0
    lever = half - cg_lateral if downhill_side == "left" else half + cg_lateral
    if lever <= 0 or cg_height <= 0:
        raise ValueError("CG outside the support polygon or non-positive height")
    return math.atan(lever / cg_height)


def longitudinal_tip_angle(lever_to_downhill_axle: float, cg_height: float) -> float:
    return math.atan(lever_to_downhill_axle / cg_height)


# ---------------------------------------------------------------------------
# 3. Climb limits (traction and actuator)
# ---------------------------------------------------------------------------
def _bisect(f: Callable[[float], float], lo: float, hi: float, iterations: int = 200) -> float:
    flo = f(lo)
    if flo == 0:
        return lo
    for _ in range(iterations):
        mid = 0.5 * (lo + hi)
        fm = f(mid)
        if (fm > 0) == (flo > 0):
            lo, flo = mid, fm
        else:
            hi = mid
    return 0.5 * (lo + hi)


def max_climb_traction(mu: float, wheelbase: float, cg_from_rear: float, cg_height: float,
                       driven: str = "all", rolling_resistance: float = 0.0) -> float:
    """Steepest steady uphill climb limited by Coulomb traction on driven axles.

    Solves mu * N_driven(theta) = W (sin(theta) + Crr cos(theta)) for the
    axle-load model of ``longitudinal_axle_loads`` (front uphill). Free axles
    (casters) carry load but transmit no tractive force.
    """
    if driven not in {"all", "front", "rear"}:
        raise ValueError(driven)

    def margin(theta: float) -> float:
        nf, nr = longitudinal_axle_loads(1.0, wheelbase, cg_from_rear, cg_height, theta)
        nf, nr = max(nf, 0.0), max(nr, 0.0)
        n = nf + nr if driven == "all" else nf if driven == "front" else nr
        return mu * n - (math.sin(theta) + rolling_resistance * math.cos(theta))

    if margin(0.0) <= 0:
        return 0.0
    return _bisect(margin, 0.0, math.pi / 2 - 1e-9)


def max_climb_torque(total_wheel_torque: float, wheel_radius: float, W: float,
                     rolling_resistance: float = 0.0) -> float:
    """Steepest steady climb limited by actuator torque: F = T/r >= W(sin + Crr cos)."""
    F = total_wheel_torque / wheel_radius
    def margin(theta: float) -> float:
        return F - W * (math.sin(theta) + rolling_resistance * math.cos(theta))
    if margin(0.0) <= 0:
        return 0.0
    if margin(math.pi / 2) >= 0:
        return math.pi / 2
    return _bisect(margin, 0.0, math.pi / 2)


# ---------------------------------------------------------------------------
# 4. Stopping distance on a slope
# ---------------------------------------------------------------------------
def stopping_distance(v0: float, mu_k: float, theta: float, direction: str,
                      braked_load_fraction: float = 1.0) -> float:
    """Locked-wheel stopping distance along the slope (Coulomb, quasi-static).

    Uphill:   d = v^2 / (2 g (f mu cos(theta) + sin(theta)))
    Downhill: d = v^2 / (2 g (f mu cos(theta) - sin(theta)))  (inf if <= 0)
    """
    if direction not in {"uphill", "downhill"}:
        raise ValueError(direction)
    s = math.sin(abs(theta)) * (1 if direction == "uphill" else -1)
    decel = G * (braked_load_fraction * mu_k * math.cos(theta) + s)
    return math.inf if decel <= 0 else v0 * v0 / (2.0 * decel)


# ---------------------------------------------------------------------------
# 5. Skid-steer steady turn with Coulomb wheel friction
# ---------------------------------------------------------------------------
@dataclass(frozen=True)
class Wheel:
    x: float          # forward position of contact [m]
    y: float          # left position of contact [m]
    normal: float     # normal load [N]


@dataclass(frozen=True)
class SkidSteerResult:
    v: float                 # body forward speed [m/s]
    omega: float             # yaw rate [rad/s]
    slip_power: float        # power dissipated in wheel/ground sliding [W]
    effective_track: float   # (u_R - u_L)/omega ; inf if omega == 0
    geometric_track: float


def _wheel_forces(wheels: Sequence[Wheel], v: float, omega: float, u_left: float,
                  u_right: float, mu: float, eps: float):
    fx_sum = mz_sum = power = 0.0
    for w in wheels:
        u = u_left if w.y > 0 else u_right
        sx = v - omega * w.y - u          # longitudinal sliding velocity
        sy = omega * w.x                  # lateral sliding velocity (no body sideslip)
        mag = math.hypot(sx, sy)
        k = mu * w.normal / math.sqrt(mag * mag + eps * eps)
        fx, fy = -k * sx, -k * sy
        fx_sum += fx
        mz_sum += w.x * fy - w.y * fx
        power += mu * w.normal * mag
    return fx_sum, mz_sum, power


def skid_steer_steady_turn(wheels: Sequence[Wheel], u_left: float, u_right: float,
                           mu: float, eps: float = 1e-6) -> SkidSteerResult:
    """Steady planar motion of a rigid skid-steer body for given wheel rim speeds.

    Coulomb friction on each wheel (regularised by ``eps`` m/s), no body side
    slip (valid for fore/aft symmetric wheel layouts), no inertia (slow turn).
    Solves sum Fx = 0 and sum Mz = 0 by nested bisection; both residuals are
    monotone in their variable for this force law.

    ASSUMPTIONS of the closed form B_eff = B (1 + (L/B)^2): it is exact for
    viscous friction, and for Coulomb friction ONLY when all four wheels carry
    equal normal load and the layout is fore/aft and left/right symmetric (all
    sliding speeds are then equal). Unequal axle loads, a CG offset, slope load
    transfer or load-dependent mu break it; this solver still handles unequal
    ``Wheel.normal`` values but then no closed form applies. Agreement with
    published controller multipliers is corroboration, not calibration.
    """
    ys = [abs(w.y) for w in wheels]
    geometric_track = 2.0 * sum(ys) / len(ys)
    span = max(abs(u_left), abs(u_right), 1e-9)
    omega_max = 4.0 * span / max(min(ys), 1e-6)

    def v_for(omega: float) -> float:
        return _bisect(lambda v: _wheel_forces(wheels, v, omega, u_left, u_right, mu, eps)[0],
                       -2 * span, 2 * span)

    def mz(omega: float) -> float:
        return _wheel_forces(wheels, v_for(omega), omega, u_left, u_right, mu, eps)[1]

    omega = 0.0 if abs(u_right - u_left) < 1e-12 else _bisect(mz, -omega_max, omega_max)
    v = v_for(omega)
    _, _, power = _wheel_forces(wheels, v, omega, u_left, u_right, mu, eps)
    eff = math.inf if abs(omega) < 1e-12 else (u_right - u_left) / omega
    return SkidSteerResult(v, omega, power, eff, geometric_track)


def four_wheel_layout(W: float, wheelbase: float, track: float) -> list[Wheel]:
    a, b, n = wheelbase / 2, track / 2, W / 4
    return [Wheel(a, b, n), Wheel(-a, b, n), Wheel(a, -b, n), Wheel(-a, -b, n)]


# ---------------------------------------------------------------------------
# 6. Nonholonomic lateral-slip feasibility (incl. cross slope)
# ---------------------------------------------------------------------------
def lateral_slip_feasible(v: float, omega: float, mu_lat: float, phi: float = 0.0) -> bool:
    """True iff Coulomb lateral friction can supply centripetal + gravity loads.

    Required in-plane lateral force per unit mass: |v*omega - g sin(phi)| in
    the worst case sign; available: mu * g cos(phi). On a flat floor this
    reduces to |v*omega| <= mu g. A simulator that never side-slips when this
    returns False is enforcing an unphysical nonholonomic constraint.
    """
    required = max(abs(v * omega + G * math.sin(phi)), abs(v * omega - G * math.sin(phi)))
    return required <= mu_lat * G * math.cos(phi) + 1e-12


def cross_slope_hold_limit(mu_lat: float) -> float:
    return math.atan(mu_lat)


# ---------------------------------------------------------------------------
# 7. Step / ledge climb
# ---------------------------------------------------------------------------
def step_contact_angle(wheel_radius: float, step_height: float) -> float:
    """Angle from vertical of the step-edge contact normal (0 = flat ground)."""
    if not 0 <= step_height < wheel_radius:
        raise ValueError("A rigid wheel cannot climb a step >= its radius by rolling")
    return math.acos((wheel_radius - step_height) / wheel_radius)


def max_step_pushed_wheel(wheel_radius: float, push_force: float, wheel_load: float) -> float:
    """Undriven wheel pushed horizontally into a frictionless edge.

    Lifts when push/load >= tan(alpha) -> h_max = r (1 - cos(atan(P/N))).
    ASSUMPTIONS: quasi-static, single wheel, frictionless edge, ``wheel_load``
    is the weight share on that wheel only, P is horizontal.
    """
    alpha = math.atan(push_force / wheel_load)
    return wheel_radius * (1.0 - math.cos(alpha))


def max_step_driven_wheel(wheel_radius: float, mu_edge: float) -> float:
    """Driven wheel with no external push, Coulomb friction mu at the edge.

    Climbs while the friction cone at the edge contains the vertical:
    tan(alpha) <= mu -> h_max = r (1 - 1/sqrt(1+mu^2)).
    ASSUMPTIONS: weight-only loading of a single wheel (no chassis push, no
    torque coupling to other axles, no load transfer), quasi-static. It is a
    conservative per-wheel bound, not a general rover/4WD step-climb result.
    """
    return wheel_radius * (1.0 - 1.0 / math.sqrt(1.0 + mu_edge * mu_edge))


# ---------------------------------------------------------------------------
# 8. Identifiability: wheel-radius scale vs slope-induced slip
# ---------------------------------------------------------------------------
def slip_ratio_linear(theta: float, slip_stiffness: float, rolling_resistance: float = 0.0) -> float:
    """Driving slip i = F_x/(C N) in the linear brush regime on a slope.

    F_x/N = (sin(theta) + Crr cos(theta)) / cos(theta). ``slip_stiffness`` C is
    the normalised longitudinal stiffness (dimensionless, typically 5-30).
    """
    demand = (math.sin(theta) + rolling_resistance * math.cos(theta)) / math.cos(theta)
    i = demand / slip_stiffness
    if not 0 <= i < 1:
        raise ValueError("outside linear slip regime")
    return i


def odometry_ratio(radius_scale: float, theta: float, slip_stiffness: float,
                   rolling_resistance: float = 0.0) -> float:
    """Wheel-odometry distance / true along-slope distance = s / (1 - i)."""
    return radius_scale / (1.0 - slip_ratio_linear(theta, slip_stiffness, rolling_resistance))


def confounded_stiffness(observed_ratio: float, radius_scale: float, theta: float,
                         rolling_resistance: float = 0.0) -> float:
    """Slip stiffness that makes ``radius_scale`` reproduce ``observed_ratio`` on one slope."""
    i = 1.0 - radius_scale / observed_ratio
    if i <= 0:
        raise ValueError("radius scale alone over-explains the ratio")
    demand = (math.sin(theta) + rolling_resistance * math.cos(theta)) / math.cos(theta)
    return demand / i


def identify_scale_and_stiffness(observations: Iterable[tuple[float, float]],
                                 rolling_resistance: float = 0.0) -> tuple[float, float]:
    """Least-squares (s, C) from (theta, odometry_ratio) pairs.

    1/ratio = (1/s) - (1/(s C)) * demand(theta) is linear in demand, so two or
    more *distinct* slopes identify both parameters; one slope does not.
    """
    pts = []
    for theta, ratio in observations:
        demand = (math.sin(theta) + rolling_resistance * math.cos(theta)) / math.cos(theta)
        pts.append((demand, 1.0 / ratio))
    if len({round(d, 12) for d, _ in pts}) < 2:
        raise ValueError("non-identifiable: need at least two distinct slope demands")
    n = len(pts)
    mx = sum(d for d, _ in pts) / n
    my = sum(y for _, y in pts) / n
    sxx = sum((d - mx) ** 2 for d, _ in pts)
    sxy = sum((d - mx) * (y - my) for d, y in pts)
    slope = sxy / sxx
    intercept = my - slope * mx
    s = 1.0 / intercept
    C = -1.0 / (slope * s)
    return s, C


# ---------------------------------------------------------------------------
# Transcriptions of the current CRANE land plants (for counterexample tests)
# ---------------------------------------------------------------------------
def crane_force_branch_climb_limit(acceleration: float, traction_multiplier: float,
                                   dynamic_friction: float, motor_torque: float,
                                   wheel_radius: float, mass: float,
                                   rolling_resistance: float) -> dict:
    """Transcribes CraneCampusRobotModel.Step force branch (lines 175-188 @976ff1e+WIP).

    The scripted drive acceleration is clamped to
    min(Acceleration*traction, mu_d*g, 2T/(r m)) and is independent of slope;
    PhysX gravity acts separately and the chassis/sphere contacts use the
    zero-friction 'TurtleBot3 Chassis Slip' material, so this clamp is the only
    tangential support.
    """
    limit = min(acceleration * traction_multiplier, dynamic_friction * G,
                2 * motor_torque / (wheel_radius * mass))
    climb = math.asin(max(0.0, min(1.0, (limit - rolling_resistance * G) / G)))
    hold = math.asin(min(1.0, (limit + rolling_resistance * G) / G))
    return {"driveLimit": limit, "maxClimb": climb, "maxHoldAtZeroCommand": hold}


def crane_force_branch_cross_slope_drift(phi: float, gain_seconds: float = 0.1) -> float:
    """Steady lateral creep of the force branch: a_lat = -v/0.1 s balances g sin(phi)."""
    return gain_seconds * G * math.sin(phi)


def legacy_diff_drive_slope_trace(theta: float, dt: float, steps: int, v0: float = 0.0,
                                  clamp_accel: float = 10.0) -> list[tuple[float, float]]:
    """Transcribes DifferentialDriveDynamics.FixedUpdate (lines 49-68) on a slope.

    Zero command, robot facing up a plane pitched ``theta``, chassis friction 0
    (TurtleBot3 Chassis Slip, combine Minimum) so the only tangential effect
    besides gravity is the script. Per fixed step, in Unity order:
      1. script: delta = clamp(0 - v, -a*dt, +a*dt); VelocityChange -> v += delta
      2. PhysX:  v += -g sin(theta) dt (gravity along the plane)
      3. PhysX:  x += v dt (semi-implicit Euler)
    Returns [(x, v)] after each step (x positive uphill). The servo never sees
    the gravity increment before integration, so the steady state is a creep of
    g sin(theta) dt per second downhill rather than a hold. ``clamp_accel=inf``
    is the 'clamp removed' mutant.
    """
    x, v, out = 0.0, v0, []
    for _ in range(steps):
        step = clamp_accel * dt
        v += max(-step, min(step, -v))
        v -= G * math.sin(theta) * dt
        x += v * dt
        out.append((x, v))
    return out


def legacy_diff_drive_creep_speed(theta: float, dt: float) -> float:
    """Steady downhill creep (m/s) of the legacy drive: g sin(theta) dt, mu-independent."""
    return G * math.sin(theta) * dt


def ackermann_torque_climb(motor_torque_per_wheel: float, wheels: int, wheel_radius: float,
                           mass: float, rolling_coefficient: float = 0.015) -> dict:
    """Transcribes AckermannRoverDynamics.ApplyWheelCommands (lines 129-140).

    Under throttle the code applies NO rolling torque (rolling torque is used as
    a brake only when throttle == 0 and brake == 0), so the torque-limited grade
    is asin(nT/(r m g)). The coast threshold is where gravity exceeds the
    rolling-brake torque Crr m g r / n per wheel.
    """
    W = mass * G
    F = motor_torque_per_wheel * wheels / wheel_radius
    return {"climb": math.asin(min(1.0, F / W)),
            "climbIfCrrWereApplied": max_climb_torque(motor_torque_per_wheel * wheels, wheel_radius, W, rolling_coefficient),
            "coastCreepOnset": math.asin(rolling_coefficient)}
