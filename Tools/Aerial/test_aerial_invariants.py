"""Analytic invariants for the reference multirotor model (FIXTURE_VALIDATED when green)."""

import math

import numpy as np
import pytest

import aerial_allocation as alloc
import aerial_model as am
from aerial_frames import axis_angle_matrix
from aerial_profiles import load_vehicle, profile_paths, read_profile, validate_profile

PROFILES = ["x500_px4_gz", "crazyflie21", "rotors_firefly_hex", "crane_reference_quad"]


def vehicle(name, **kw):
    return load_vehicle(name, with_battery=kw.pop("battery", False), **kw)


@pytest.mark.parametrize("path", profile_paths(), ids=lambda p: p.stem)
def test_profiles_cite_every_coefficient(path):
    assert validate_profile(read_profile(path)) == []


@pytest.mark.parametrize("name", PROFILES)
def test_hover_thrust_equals_weight_and_is_torque_free(name):
    v = vehicle(name)
    w_h = am.hover_rotor_speed(v)
    assert w_h < min(r.omega_max for r in v.rotors), "vehicle cannot hover"
    s = am.initial_state(v, p=(0, 0, 10), rotor_speed=w_h)
    force, torque = am.wrench(v, s)
    assert force[2] == pytest.approx(v.mass * v.gravity, rel=1e-12)
    assert np.allclose(torque, 0.0, atol=1e-12)
    for _ in range(500):
        s = am.step(v, s, [w_h] * len(v.rotors), 0.002)
    assert np.linalg.norm(s.p - [0, 0, 10]) < 1e-9
    assert np.linalg.norm(s.omega) < 1e-12


@pytest.mark.parametrize("name", ["x500_px4_gz", "crazyflie21"])
def test_thrust_to_weight_exceeds_one(name):
    v = vehicle(name)
    t_max = alloc.max_thrusts(v).sum()
    assert t_max / (v.mass * v.gravity) > 1.2


def test_motor_first_order_step_uses_up_and_down_constants():
    v = vehicle("x500_px4_gz")
    r = v.rotors[0]
    dt = r.tau_up / 50
    s = am.initial_state(v, rotor_speed=500.0)
    for _ in range(50):
        am.update_motors(v, s, [800.0] * 4, dt)
    assert s.rotor_speed[0] == pytest.approx(500 + 300 * (1 - math.exp(-1)), rel=1e-12)
    dt = r.tau_down / 50
    s = am.initial_state(v, rotor_speed=800.0)
    for _ in range(50):
        am.update_motors(v, s, [500.0] * 4, dt)
    assert s.rotor_speed[0] == pytest.approx(800 - 300 * (1 - math.exp(-1)), rel=1e-12)


def test_yaw_acceleration_from_reaction_torque_balance():
    v = vehicle("x500_px4_gz")
    w_h = am.hover_rotor_speed(v)
    d = 30.0
    # raise CCW rotors, lower CW rotors so that sum(T) is unchanged to first order
    wp = math.sqrt(w_h ** 2 + d ** 2)
    wm = math.sqrt(w_h ** 2 - d ** 2)
    speeds = [wp if r.spin > 0 else wm for r in v.rotors]
    s = am.initial_state(v, p=(0, 0, 10), rotor_speed=speeds)
    force, torque = am.wrench(v, s)
    kf, km = v.rotors[0].kf, v.rotors[0].km
    expected_tz = -km * kf * (2 * wp ** 2 - 2 * wm ** 2)
    assert force[2] == pytest.approx(v.mass * v.gravity, rel=1e-12)
    assert torque[2] == pytest.approx(expected_tz, rel=1e-12)
    assert torque[2] < 0, "CCW rotors speeding up must yaw the body clockwise (negative z)"
    s = am.step(v, s, speeds, 0.001)
    # rotor drag on the hub velocity omega x r adds a small yaw damping within the step
    assert s.omega[2] == pytest.approx(expected_tz / v.inertia[2, 2] * 0.001, rel=1e-3)


def _rotate_state(s, rot):
    out = s.copy()
    out.p = rot @ s.p
    out.v = rot @ s.v
    out.q = am.quat_mul(am.matrix_to_quat(rot), s.q)
    return out


@pytest.mark.parametrize("seed", [1, 2, 3])
def test_yaw_rotation_equivariance_with_wind_and_drag(seed):
    rng = np.random.default_rng(seed)
    v = vehicle("x500_px4_gz")
    v.drag_linear = np.array([0.2, 0.3, 0.4])
    v.drag_quadratic = 0.05
    s = am.initial_state(v, p=rng.normal(size=3) + [0, 0, 5])
    s.v = rng.normal(size=3)
    s.omega = rng.normal(size=3) * 0.5
    s.q = am.quat_from_axis_angle(rng.normal(size=3), 0.4)
    wind = rng.normal(size=3) * 3
    cmd = am.hover_rotor_speed(v) + rng.normal(size=4) * 20
    s.rotor_speed[:] = cmd
    rot = axis_angle_matrix([0, 0, 1], rng.uniform(-math.pi, math.pi))
    a, b = s, _rotate_state(s, rot)
    for _ in range(200):
        a = am.step(v, a, cmd, 0.002, wind)
        b = am.step(v, b, cmd, 0.002, rot @ wind)
    assert np.allclose(rot @ a.p, b.p, atol=1e-9)
    assert np.allclose(rot @ a.v, b.v, atol=1e-9)
    assert np.allclose(a.omega, b.omega, atol=1e-9)


def test_full_so3_equivariance_without_gravity():
    rng = np.random.default_rng(7)
    v = vehicle("rotors_firefly_hex")
    v.gravity = 0.0
    v.drag_linear = np.array([0.2, 0.2, 0.3])
    v.drag_quadratic = 0.1
    s = am.initial_state(v, p=(1, 2, 3))
    s.v = np.array([2.0, -1.0, 0.5])
    s.omega = np.array([0.3, -0.2, 0.1])
    cmd = np.full(6, 500.0) + rng.normal(size=6) * 30
    s.rotor_speed[:] = cmd
    wind = np.array([1.0, 3.0, -0.5])
    rot = axis_angle_matrix(rng.normal(size=3), 1.1)
    a, b = s, _rotate_state(s, rot)
    for _ in range(200):
        a = am.step(v, a, cmd, 0.002, wind)
        b = am.step(v, b, cmd, 0.002, rot @ wind)
    assert np.allclose(rot @ a.p, b.p, atol=1e-9)
    assert np.allclose(a.omega, b.omega, atol=1e-9)


def test_free_fall_conserves_energy_and_angular_momentum():
    v = vehicle("rotors_firefly_hex")
    s = am.initial_state(v, p=(0, 0, 100))
    s.v = np.array([1.0, 0.5, 3.0])
    s.omega = np.array([2.0, 0.1, 0.5])   # three-axis tumble, asymmetric inertia
    e0 = am.mechanical_energy(v, s)
    h0 = am.quat_to_matrix(s.q) @ (v.inertia @ s.omega)
    for _ in range(2000):
        s = am.step(v, s, np.zeros(6), 0.001)
    h1 = am.quat_to_matrix(s.q) @ (v.inertia @ s.omega)
    assert am.mechanical_energy(v, s) == pytest.approx(e0, rel=1e-9)
    assert np.allclose(h0, h1, atol=1e-9)
    assert s.v[2] == pytest.approx(3.0 - v.gravity * 2.0, abs=1e-9)


def test_drag_only_dissipates_energy():
    v = vehicle("x500_px4_gz")
    v.drag_linear = np.array([0.3, 0.3, 0.4])
    v.drag_quadratic = 0.1
    s = am.initial_state(v, p=(0, 0, 100))
    s.v = np.array([5.0, -3.0, 1.0])
    e = am.mechanical_energy(v, s)
    for _ in range(300):
        s = am.step(v, s, np.zeros(4), 0.005)
        e_new = am.mechanical_energy(v, s)
        assert e_new <= e + 1e-12
        e = e_new


def test_wind_force_is_air_relative_and_world_fixed():
    v = vehicle("x500_px4_gz")
    v.drag_linear = np.array([0.3, 0.3, 0.3])
    v.drag_quadratic = 0.1
    w_h = am.hover_rotor_speed(v)
    still = am.initial_state(v, p=(0, 0, 5), rotor_speed=w_h)
    moving = still.copy()
    moving.v = np.array([-4.0, 0.0, 0.0])
    f_wind, t_wind = am.wrench(v, still, wind_world=[4.0, 0, 0])
    f_move, t_move = am.wrench(v, moving)
    assert np.allclose(f_wind, f_move) and np.allclose(t_wind, t_move)
    for yaw in (0.0, 0.7, math.pi / 2, 2.5):
        s = still.copy()
        s.q = am.quat_from_axis_angle([0, 0, 1], yaw)
        f_b, _ = am.wrench(v, s, wind_world=[4.0, 0, 0])
        f_w = am.quat_to_matrix(s.q) @ f_b
        assert f_w[0] > 0 and abs(f_w[1]) < 1e-9, "drag must push downwind regardless of heading"
        assert f_w[2] == pytest.approx(v.mass * v.gravity, rel=1e-12)


def test_rotor_drag_scales_with_rotor_speed():
    v = vehicle("x500_px4_gz")
    s = am.initial_state(v, p=(0, 0, 5), rotor_speed=400.0)
    s.v = np.array([3.0, 0, 0])
    fx_slow = am.wrench(v, s)[0][0]
    s.rotor_speed[:] = 800.0
    fx_fast = am.wrench(v, s)[0][0]
    assert fx_fast == pytest.approx(2 * fx_slow, rel=1e-9) and fx_fast < 0


def test_ground_effect_is_monotone_and_vanishes_aloft():
    v = vehicle("x500_px4_gz", ground_effect=True)
    r = v.rotors[0]
    heights = np.linspace(0.05, 3.0, 60)
    f = [am.ground_effect_factor(v, r, h) for h in heights]
    assert all(a >= b for a, b in zip(f, f[1:]))
    assert am.ground_effect_factor(v, r, r.radius) == pytest.approx(1 / (1 - 1 / 16))
    assert am.ground_effect_factor(v, r, 20 * r.radius) - 1 < 1e-3
    assert max(f) == pytest.approx(1 / (1 - 0.25))     # clamp at h = R/2


def test_out_of_ground_effect_hover_command_climbs_near_pad():
    v = vehicle("x500_px4_gz", ground_effect=True)
    w_h = am.hover_rotor_speed(v)
    s = am.initial_state(v, p=(0, 0, 0.05), rotor_speed=w_h)  # rotor hubs at 0.11 m
    s = am.step(v, s, [w_h] * 4, 0.002)
    assert s.v[2] > 0
    high = am.initial_state(v, p=(0, 0, 10.0), rotor_speed=w_h)
    high = am.step(v, high, [w_h] * 4, 0.002)
    assert abs(high.v[2]) < 1e-5


def test_quad_allocation_feasibility_and_yaw_authority():
    v = vehicle("x500_px4_gz")
    b = alloc.effectiveness_matrix(v)
    u_max = alloc.max_thrusts(v)
    weight = v.mass * v.gravity
    assert alloc.wrench_feasible(b, u_max, [weight, 0, 0, 0])
    assert not alloc.wrench_feasible(b, u_max, [u_max.sum() * 1.01, 0, 0, 0])
    t_h = weight / 4
    km = v.rotors[0].km
    yaw_max = 4 * km * min(t_h, u_max[0] - t_h)
    assert alloc.wrench_feasible(b, u_max, [weight, 0, 0, 0.999 * yaw_max])
    assert not alloc.wrench_feasible(b, u_max, [weight, 0, 0, 1.001 * yaw_max])
    u, achieved, exact = alloc.allocate(b, u_max, [weight, 0, 0, 0.5 * yaw_max])
    assert exact and np.allclose(achieved, [weight, 0, 0, 0.5 * yaw_max])


def test_hex_zonotope_feasibility_is_sound_and_tight():
    v = vehicle("rotors_firefly_hex")
    b = alloc.effectiveness_matrix(v)
    u_max = alloc.max_thrusts(v)
    rng = np.random.default_rng(3)
    for _ in range(200):
        u = rng.uniform(0, 1, 6) * u_max
        assert alloc.wrench_feasible(b, u_max, b @ u)
    for _ in range(50):
        d = rng.normal(size=4)
        support_point = b @ (u_max * (b.T @ d > 0))
        assert alloc.wrench_feasible(b, u_max, 0.999 * support_point + 0.001 * b @ (u_max / 2))
        assert not alloc.wrench_feasible(b, u_max, support_point + 1e-3 * d / np.linalg.norm(d))


def _hover_margin(b, u_max, w):
    gens = b * u_max
    normals = alloc._facet_normals(gens)
    return min(np.sum(np.maximum(0, n @ gens)) - n @ w for n in normals)


def test_hex_single_rotor_failure_has_zero_hover_controllability_margin():
    """PNPNPN hexarotor with one failed rotor: the hover wrench lies ON the attainable-set
    boundary, so the controllability index is 0 (Du et al., J. Guid. Control Dyn. 2015,
    doi 10.2514/1.G000731). Index 0 means uncontrollable (no control authority left about
    hover), not that hover itself is infeasible: the wrench is still attainable on the boundary."""
    v = vehicle("rotors_firefly_hex")
    b = alloc.effectiveness_matrix(v)
    u_max = alloc.max_thrusts(v)
    hover = np.array([v.mass * v.gravity, 0, 0, 0])
    assert _hover_margin(b, u_max, hover) > 0.1
    failed = u_max.copy()
    failed[0] = 0.0
    assert abs(_hover_margin(b, failed, hover)) < 1e-9


def test_battery_sag_limits_rotor_speed_and_matches_vendor_table():
    doc = read_profile("crazyflie21")
    rows = np.array(doc["measurements"]["BCPWM_table_2015"]["rows"])
    current, thrust_g, volts, _, rpm = rows.T
    v = vehicle("crazyflie21", battery=True)
    b = v.battery
    predicted = b.v_full - current * b.resistance
    assert np.sqrt(np.mean((predicted - volts) ** 2)) < 0.02
    w = rpm[1:] * 2 * math.pi / 60
    per_rotor = thrust_g[1:] / 1000 * 9.81 / 4
    kf = float(w ** 2 @ per_rotor / (w ** 2 @ w ** 2))
    assert kf == pytest.approx(v.rotors[0].kf, rel=1e-9)
    shaft = 4 * v.rotors[0].km * kf * w[-1] ** 3
    assert shaft / (volts[-1] * current[-1]) == pytest.approx(b.efficiency, abs=0.005)
    s = am.initial_state(v, rotor_speed=am.hover_rotor_speed(v))
    i_hover = am.electrical_current(v, s)
    assert 1.0 < i_hover < 3.0          # table: 29 g lies between 1.83 A and 2.11 A rows
    s.rotor_speed[:] = v.rotors[0].omega_max
    assert am.rotor_speed_limit(v, v.rotors[0], s) < v.rotors[0].omega_max
    s.soc = 0.2
    low = am.rotor_speed_limit(v, v.rotors[0], s)
    s.soc = 1.0
    assert low < am.rotor_speed_limit(v, v.rotors[0], s)


def test_crazyflie_hover_falls_in_vendor_pwm_band():
    doc = read_profile("crazyflie21")
    rows = np.array(doc["measurements"]["BCPWM_table_2015"]["rows"])
    v = vehicle("crazyflie21")
    hover_rpm = am.hover_rotor_speed(v) * 60 / (2 * math.pi)
    lo = rows[rows[:, 3] == 50][0, 4]
    hi = rows[rows[:, 3] == 56.25][0, 4]
    assert lo <= hover_rpm <= hi
