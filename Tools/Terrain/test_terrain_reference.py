import math
import re
from pathlib import Path

import pytest

import robot_profiles
import terrain_reference as ref

D = math.radians


def test_static_hold_limit_is_atan_mu_and_boundary_is_sharp():
    assert ref.static_hold_limit(0.8) == pytest.approx(math.atan(0.8))
    lim = ref.static_hold_limit(0.6)
    assert ref.holds_on_slope(lim - 1e-6, 0.6)
    assert not ref.holds_on_slope(lim + 1e-6, 0.6)
    # Casters carrying half the load halve the effective coefficient.
    assert ref.static_hold_limit(0.6, 0.5) == pytest.approx(math.atan(0.3))


def test_longitudinal_load_transfer_conserves_load_and_shifts_downhill():
    W, L, xr, h = 500.0, 0.5, 0.25, 0.2
    nf0, nr0 = ref.longitudinal_axle_loads(W, L, xr, h, 0.0)
    assert nf0 == pytest.approx(nr0) == pytest.approx(W / 2)
    nf, nr = ref.longitudinal_axle_loads(W, L, xr, h, D(20))
    assert nf + nr == pytest.approx(W * math.cos(D(20)))
    assert nf < nf0 and nr > nr0 * math.cos(D(20))
    # Front unloads exactly at the longitudinal tip angle.
    tip = ref.longitudinal_tip_angle(xr, h)
    assert ref.longitudinal_axle_loads(W, L, xr, h, tip)[0] == pytest.approx(0.0, abs=1e-9)


def test_lateral_loads_vanish_on_uphill_side_at_tip_angle_and_turning_loads_outer_side():
    W, B, h = 400.0, 0.55, 0.19
    tip = ref.lateral_tip_angle(B, h, 0.0, "right")
    assert tip == pytest.approx(math.atan(B / 2 / h))
    nl, nr = ref.lateral_side_loads(W, B, 0.0, h, tip)
    assert nl == pytest.approx(0.0, abs=1e-9) and nr == pytest.approx(W * math.cos(tip))
    # Left turn (ay > 0): outer = right wheels gain load.
    nl, nr = ref.lateral_side_loads(W, B, 0.0, h, 0.0, ay=2.0)
    assert nr > nl
    # CG offset toward +y: tipping to the left needs less roll.
    assert ref.lateral_tip_angle(B, h, 0.05, "left") < ref.lateral_tip_angle(B, h, 0.05, "right")


def test_all_wheel_traction_climb_equals_coulomb_limit_without_rolling_resistance():
    for mu in (0.3, 0.6, 1.0):
        assert ref.max_climb_traction(mu, 0.5, 0.25, 0.2, "all") == pytest.approx(math.atan(mu), abs=1e-9)


def test_front_drive_climb_is_reduced_by_load_transfer_and_rolling_resistance():
    mu = 0.7
    allw = ref.max_climb_traction(mu, 0.5, 0.25, 0.2, "all")
    front = ref.max_climb_traction(mu, 0.5, 0.25, 0.2, "front")
    rear = ref.max_climb_traction(mu, 0.5, 0.25, 0.2, "rear")
    assert front < rear < allw
    assert ref.max_climb_traction(mu, 0.5, 0.25, 0.2, "all", 0.05) < allw


def test_torque_limited_climb():
    W = 60 * ref.G
    th = ref.max_climb_torque(4 * 6.0, 0.25, W)
    assert math.sin(th) == pytest.approx(96 / W)
    assert ref.max_climb_torque(1.0, 0.25, W, rolling_resistance=1.0) == 0.0


def test_stopping_distance_up_and_down_slope():
    v, mu = 1.0, 0.6
    flat = ref.stopping_distance(v, mu, 0.0, "uphill")
    assert flat == pytest.approx(v * v / (2 * ref.G * mu))
    up = ref.stopping_distance(v, mu, D(10), "uphill")
    down = ref.stopping_distance(v, mu, D(10), "downhill")
    assert up < flat < down
    assert ref.stopping_distance(v, mu, math.atan(mu) + 1e-3, "downhill") == math.inf


def test_skid_steer_coulomb_track_multiplier_closed_form_and_published_controller_values():
    # Equal loads, Coulomb friction: B_eff = B (1 + (L/B)^2) for a symmetric 4-wheel layout.
    for name, published in (("husky_a200", 1.875), ("jackal", 1.5)):
        p = robot_profiles.load(name)
        L, B = robot_profiles.value(p, "wheelbaseM"), robot_profiles.value(p, "trackM")
        W = robot_profiles.value(p, "totalMassKg") * ref.G
        res = ref.skid_steer_steady_turn(ref.four_wheel_layout(W, L, B), -0.2, 0.2, 0.6)
        assert res.v == pytest.approx(0.0, abs=1e-9)
        assert res.effective_track / B == pytest.approx(1 + (L / B) ** 2, rel=1e-4)
        # The independent Clearpath controller multipliers agree within 2 %.
        assert res.effective_track / B == pytest.approx(published, rel=0.02)
        assert published == robot_profiles.value(p, "wheelSeparationMultiplier")


def test_skid_steer_slip_power_scales_with_mu_and_vanishes_when_straight():
    wheels = ref.four_wheel_layout(500.0, 0.5, 0.55)
    straight = ref.skid_steer_steady_turn(wheels, 0.5, 0.5, 0.6)
    assert straight.slip_power == pytest.approx(0.0, abs=1e-3) and straight.v == pytest.approx(0.5)
    p1 = ref.skid_steer_steady_turn(wheels, -0.2, 0.2, 0.3).slip_power
    p2 = ref.skid_steer_steady_turn(wheels, -0.2, 0.2, 0.6).slip_power
    assert p2 == pytest.approx(2 * p1, rel=1e-6) and p1 > 0


def test_nonholonomic_lateral_slip_feasibility():
    assert ref.lateral_slip_feasible(1.0, 1.0, 0.6)          # 1 m/s^2 < 5.9
    assert not ref.lateral_slip_feasible(3.0, 3.0, 0.6)      # 9 m/s^2 > 5.9
    # A cross slope beyond atan(mu) cannot be held even at rest.
    assert not ref.lateral_slip_feasible(0.0, 0.0, 0.6, math.atan(0.6) + 1e-3)
    assert ref.lateral_slip_feasible(0.0, 0.0, 0.6, math.atan(0.6) - 1e-3)


def test_step_climb_limits():
    r = 0.1651
    assert ref.step_contact_angle(r, 0.0) == 0.0
    with pytest.raises(ValueError):
        ref.step_contact_angle(r, r)
    h_driven = ref.max_step_driven_wheel(r, 1.0)
    assert h_driven == pytest.approx(r * (1 - 1 / math.sqrt(2)))
    alpha = ref.step_contact_angle(r, h_driven)
    assert math.tan(alpha) == pytest.approx(1.0)
    assert ref.max_step_pushed_wheel(r, 100.0, 100.0) == pytest.approx(h_driven)
    assert ref.max_step_pushed_wheel(r, 50.0, 100.0) < h_driven


def test_wheel_scale_and_slip_are_confounded_on_one_slope_but_identified_by_two():
    theta = D(15)
    s_true, C_true = 1.02, 12.0
    observed = ref.odometry_ratio(s_true, theta, C_true)
    # Counterexample: a different (scale, stiffness) pair reproduces the single-slope observation exactly.
    s_alt = 1.0
    C_alt = ref.confounded_stiffness(observed, s_alt, theta)
    assert ref.odometry_ratio(s_alt, theta, C_alt) == pytest.approx(observed, rel=1e-12)
    assert abs(C_alt - C_true) > 1.0
    with pytest.raises(ValueError):
        ref.identify_scale_and_stiffness([(theta, observed), (theta, observed)])
    # Adding a flat run (or any second slope) separates them.
    obs = [(t, ref.odometry_ratio(s_true, t, C_true)) for t in (0.0, D(5), theta)]
    s_fit, C_fit = ref.identify_scale_and_stiffness(obs)
    assert s_fit == pytest.approx(s_true, rel=1e-9) and C_fit == pytest.approx(C_true, rel=1e-9)
    # And the flat run alone already disambiguates the two hypotheses.
    assert ref.odometry_ratio(s_alt, 0.0, C_alt) != pytest.approx(ref.odometry_ratio(s_true, 0.0, C_true), rel=1e-3)


# --------------------------------------------------------------------------
# Counterexamples against the current CRANE land plants (transcribed formulas)
# --------------------------------------------------------------------------
ROUGH_CONCRETE = dict(traction_multiplier=0.95, dynamic_friction=0.7225, rolling_resistance=0.025)


def test_counterexample_force_branch_cannot_climb_or_hold_the_campus_5deg_loading_ramp():
    r = ref.crane_force_branch_climb_limit(acceleration=0.6, motor_torque=0.35, wheel_radius=0.033,
                                           mass=1.5539075, **ROUGH_CONCRETE)
    assert r["driveLimit"] == pytest.approx(0.57)
    assert math.degrees(r["maxClimb"]) < 2.0
    assert math.degrees(r["maxHoldAtZeroCommand"]) < 5.0
    # Analytic Coulomb reference: the same robot/surface holds and climbs 5 deg easily.
    p = robot_profiles.load("turtlebot3_waffle_campus_prior")
    limits = robot_profiles.analytic_limits(p, mu=0.7225, rolling_resistance=0.025)
    assert limits["tractionClimbDeg"] > 30 and limits["torqueClimbDeg"] > 30


def test_counterexample_force_branch_creeps_on_cross_slopes_that_coulomb_holds():
    phi = D(10)
    assert ref.lateral_slip_feasible(0.0, 0.0, 0.7225, phi)
    assert ref.crane_force_branch_cross_slope_drift(phi) == pytest.approx(0.170, abs=1e-3)


ROOT = Path(__file__).resolve().parents[2]


def _cs_constant(path: str, pattern: str) -> float:
    text = (ROOT / path).read_text(encoding="utf-8")
    return float(re.search(pattern, text).group(1))


def legacy_clamp() -> float:
    return _cs_constant("Assets/Scripts/Physics/Land/DifferentialDriveDynamics.cs",
                        r"maximumLinearAcceleration\s*=\s*([0-9.]+)f;")


def legacy_prediction_holds(trace_fn, theta, dt, v0) -> bool:
    """Oracle for the transcribed C# step: the clamp sets the transient, gravity sets the creep."""
    a = legacy_clamp()
    trace = trace_fn(theta, dt, 400, v0)
    creep = ref.legacy_diff_drive_creep_speed(theta, dt)
    # Steady state: velocity just after integration is exactly -g sin(theta) dt, mu-independent.
    if trace[-1][1] != pytest.approx(-creep, rel=1e-9):
        return False
    if (trace[-1][0] - trace[-101][0]) / (100 * dt) != pytest.approx(-creep, rel=1e-9):
        return False
    # Transient from a downhill speed v0<0: net deceleration (a - g sin) per step until captured.
    capture_steps = math.ceil((abs(v0) - creep) / ((a - ref.G * math.sin(theta)) * dt))
    first_settled = next(i for i, (_, v) in enumerate(trace) if v == pytest.approx(-creep, rel=1e-9))
    return abs(first_settled - capture_steps) <= 1


@pytest.mark.parametrize("dt", [0.02, 0.01])
def test_counterexample_legacy_drive_creeps_downhill_mu_independent(dt):
    theta = D(45)
    assert legacy_clamp() == 10.0
    assert legacy_prediction_holds(ref.legacy_diff_drive_slope_trace, theta, dt, v0=-1.0)
    # At the project default dt=0.02 the creep is ~0.139 m/s; Coulomb (mu=0.6) says the robot slides
    # freely, and mu never enters the legacy plant at all (chassis friction 0).
    assert ref.legacy_diff_drive_creep_speed(theta, 0.02) == pytest.approx(0.1387, abs=1e-4)
    assert not ref.holds_on_slope(theta, 0.6)


def test_mutant_without_clamp_turns_legacy_prediction_red():
    def mutant(theta, dt, steps, v0):
        return ref.legacy_diff_drive_slope_trace(theta, dt, steps, v0, clamp_accel=math.inf)
    assert not legacy_prediction_holds(mutant, D(45), 0.02, v0=-1.0)
    real = lambda theta, dt, steps, v0: ref.legacy_diff_drive_slope_trace(theta, dt, steps, v0, legacy_clamp())
    assert legacy_prediction_holds(real, D(45), 0.02, v0=-1.0)


def test_ackermann_torque_cap_has_no_rolling_term_under_throttle():
    torque = _cs_constant("Assets/Scripts/Physics/Land/AckermannRoverDynamics.cs", r"maximumMotorTorque\s*=\s*([0-9.]+)f;")
    r = ref.ackermann_torque_climb(torque, 4, 0.25, 60.0)
    assert math.degrees(r["climb"]) == pytest.approx(9.39, abs=0.01)
    assert math.degrees(r["climbIfCrrWereApplied"]) == pytest.approx(8.52, abs=0.01)
    assert math.degrees(r["coastCreepOnset"]) == pytest.approx(0.86, abs=0.01)


def test_turtlebot_prior_cg_height_implies_implausible_tip_angle():
    p = robot_profiles.load("turtlebot3_waffle_campus_prior")
    tip = robot_profiles.analytic_limits(p, mu=0.8)["lateralTipDegRightDownhill"]
    assert tip > 80  # flags the 2 cm CG-height prior for physical measurement


def test_profiles_have_sources_and_compatible_licenses():
    for name in ("husky_a200", "jackal", "turtlebot3_waffle_campus_prior"):
        p = robot_profiles.load(name)
        assert p["license"]["gplCompatible"] is True
        assert all(f["source"] for f in p["parameters"].values())
