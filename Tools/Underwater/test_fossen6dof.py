"""Invariant and analytic-fixture tests for the 6-DOF Fossen reference model.

Each test is a falsifier: it states a closed-form result from Fossen (2011)
and fails if the model departs from it.  The vehicles here are synthetic so the
tests do not depend on literature coefficients (see test_vehicle_profile.py).
"""
from __future__ import annotations

import numpy as np
import pytest

import identifiability as ident
from fossen6dof import (
    G,
    Simulator,
    State,
    Tether,
    Thruster,
    VehicleParams,
    allocate,
    allocation_matrix,
    coriolis_from_mass,
    damping_force,
    euler_to_quat,
    potential_energy,
    quat_rate_matrix,
    quat_to_euler,
    quat_to_rot,
    restoring,
)

RNG = np.random.default_rng(20261020)


def vehicle(
    m=11.5,
    bg=0.02,
    net_buoyancy=0.0,
    added=(5.5, 12.7, 14.6, 0.12, 0.12, 0.12),
    lin=(4.0, 6.2, 5.2, 0.07, 0.07, 0.07),
    quad=(18.2, 21.7, 37.0, 1.55, 1.55, 1.55),
    r_g=(0.0, 0.0, 0.0),
    inertia=(0.16, 0.16, 0.16),
    coupled_added_mass=False,
) -> VehicleParams:
    r_g = np.array(r_g, float)
    r_b = r_g - np.array([0.0, 0.0, bg])  # CB above CG (z down)
    ma = np.diag(added)
    if coupled_added_mass:
        ma[0, 4] = ma[4, 0] = 0.3  # symmetric surge-pitch coupling
        ma[1, 3] = ma[3, 1] = -0.2
    return VehicleParams(
        mass=m,
        inertia_cg=np.diag(inertia),
        r_g=r_g,
        r_b=r_b,
        buoyancy=m * G + net_buoyancy,
        added_mass=ma,
        linear_damping=np.diag(lin),
        quadratic_damping=np.array(quad),
    )


def vectored_layout(n_vertical: int) -> list[Thruster]:
    """4 horizontal thrusters at +-45 deg plus 2 or 4 vertical thrusters (FRD)."""
    c = np.sqrt(0.5)
    horiz = [
        Thruster([0.156, -0.111, 0.085], [c, -c, 0], 40.0, 30.0, 0.0, "fr"),
        Thruster([0.156, 0.111, 0.085], [c, c, 0], 40.0, 30.0, 0.0, "fl"),
        Thruster([-0.156, -0.111, 0.085], [c, c, 0], 40.0, 30.0, 0.0, "rr"),
        Thruster([-0.156, 0.111, 0.085], [c, -c, 0], 40.0, 30.0, 0.0, "rl"),
    ]
    if n_vertical == 2:
        vert = [Thruster([0.0, -0.11, 0.0], [0, 0, 1], 40.0, 30.0), Thruster([0.0, 0.11, 0.0], [0, 0, 1], 40.0, 30.0)]
    else:
        vert = [Thruster([x, y, 0.0], [0, 0, 1], 40.0, 30.0) for x, y in ((0.12, -0.22), (0.12, 0.22), (-0.12, -0.22), (-0.12, 0.22))]
    return horiz + vert


# --------------------------------------------------------------------------- #
# Kinematics and algebraic properties
# --------------------------------------------------------------------------- #
def test_euler_quaternion_round_trip():
    for _ in range(50):
        e = RNG.uniform([-np.pi, -1.4, -np.pi], [np.pi, 1.4, np.pi])
        np.testing.assert_allclose(quat_to_euler(euler_to_quat(*e)), e, atol=1e-12)


def test_rotation_is_orthonormal_and_rate_matrix_matches_finite_difference():
    q = euler_to_quat(0.3, -0.2, 1.1)
    R = quat_to_rot(q)
    np.testing.assert_allclose(R @ R.T, np.eye(3), atol=1e-12)
    w = np.array([0.2, -0.4, 0.7])
    h = 1e-7
    q2 = q + h * quat_rate_matrix(q) @ w
    R2 = quat_to_rot(q2 / np.linalg.norm(q2))
    # Rdot = R S(w) for body-frame angular velocity (Fossen eq. 2.56).
    np.testing.assert_allclose((R2 - R) / h, R @ np.array([[0, -w[2], w[1]], [w[2], 0, -w[0]], [-w[1], w[0], 0]]), atol=1e-5)


def test_coriolis_is_skew_symmetric_for_random_mass_and_velocity():
    p = vehicle(r_g=(0.01, -0.02, 0.03), coupled_added_mass=True)
    for M in (p.m_rb(), p.added_mass):
        for _ in range(20):
            nu = RNG.normal(size=6)
            C = coriolis_from_mass(M, nu)
            assert abs(nu @ C @ nu) < 1e-10
            np.testing.assert_allclose(C, -C.T, atol=1e-12)


def test_munk_counterexample_from_inventory_routes_to_force_and_yaw():
    # Inventory V2/D3: u=1, v=0.2, r=0.5, Xudot=6, Yvdot=8 -> X=+0.8, Y=-3.0, N=-0.4.
    ma = np.diag([6.0, 8.0, 2.0, 0.15, 0.25, 0.35])
    nu = np.array([1.0, 0.2, 0.0, 0.0, 0.0, 0.5])
    hydro = -coriolis_from_mass(ma, nu) @ nu
    np.testing.assert_allclose(hydro, [0.8, -3.0, 0.0, 0.0, 0.0, -0.4], atol=1e-12)


def test_rigid_body_mass_matrix_is_symmetric_positive_definite_and_uses_parallel_axis():
    p = vehicle(r_g=(0.0, 0.0, 0.05))
    M = p.m_rb()
    np.testing.assert_allclose(M, M.T)
    assert np.min(np.linalg.eigvalsh(M)) > 0
    assert M[3, 3] == pytest.approx(0.16 + 11.5 * 0.05**2)


def test_parameter_validation_rejects_nonphysical_values():
    with pytest.raises(ValueError):
        vehicle(added=(-5.5, 12.7, 14.6, 0.12, 0.12, 0.12))  # wrong SNAME sign stored
    with pytest.raises(ValueError):
        vehicle(quad=(-1, 0, 0, 0, 0, 0))
    with pytest.raises(ValueError):
        vehicle(inertia=(0.16, -0.1, 0.16))


def test_potential_energy_rate_equals_nu_dot_g():
    p = vehicle(r_g=(0.01, -0.02, 0.03), bg=0.05, net_buoyancy=3.0)
    for _ in range(20):
        pos = RNG.normal(size=3)
        q = euler_to_quat(*RNG.uniform(-1.0, 1.0, size=3))
        nu = RNG.normal(size=6)
        h = 1e-6
        pos2 = pos + h * quat_to_rot(q) @ nu[:3]
        q2 = q + h * quat_rate_matrix(q) @ nu[3:]
        q2 /= np.linalg.norm(q2)
        dU = (potential_energy(p, pos2[2], q2) - potential_energy(p, pos[2], q)) / h
        assert dU == pytest.approx(nu @ restoring(p, q), rel=1e-4, abs=1e-6)


# --------------------------------------------------------------------------- #
# Analytic fixtures
# --------------------------------------------------------------------------- #
def test_neutral_buoyancy_hover_is_an_equilibrium():
    p = vehicle(bg=0.02, r_g=(0.0, 0.0, 0.01))
    sim = Simulator(p)
    s0 = State.at_rest(pos=(0, 0, 5.0))
    s = sim.simulate(s0, 0.01, 1000)[-1]
    np.testing.assert_allclose(s.pos, s0.pos, atol=1e-12)
    np.testing.assert_allclose(s.nu, 0.0, atol=1e-12)
    np.testing.assert_allclose(quat_to_euler(s.q), 0.0, atol=1e-12)


def test_net_buoyancy_initial_heave_acceleration_includes_added_mass():
    p = vehicle(net_buoyancy=2.0)
    sim = Simulator(p, enable_damping=False)
    a = sim.acceleration(State.at_rest(), np.zeros(6))
    assert a[2] == pytest.approx(-2.0 / (p.mass + p.added_mass[2, 2]), rel=1e-12)


@pytest.mark.parametrize("phi", np.deg2rad([1.0, 10.0, 30.0, 60.0, 90.0]))
def test_restoring_moment_has_righting_arm_bg_sin_phi(phi):
    bg = 0.02
    p = vehicle(bg=bg)
    g_roll = restoring(p, euler_to_quat(phi, 0.0, 0.3))
    # LHS restoring term K = W*BG*sin(phi); the moment on the vehicle is -K.
    assert g_roll[3] == pytest.approx(p.weight * bg * np.sin(phi), rel=1e-12)
    assert abs(g_roll[3]) / p.weight == pytest.approx(bg * np.sin(phi), rel=1e-12)  # GZ = BG sin(phi)
    np.testing.assert_allclose(g_roll[:3], 0.0, atol=1e-12)  # W = B: pure couple
    g_pitch = restoring(p, euler_to_quat(0.0, phi * 0.9, 0.0))
    assert g_pitch[4] == pytest.approx(p.weight * bg * np.sin(phi * 0.9), rel=1e-12)


def test_top_heavy_vehicle_has_negative_bg_and_capsizing_moment():
    p = vehicle(bg=-0.00125)  # the CRANE Sub prefab geometry (see I1-underwater.md)
    assert p.bg < 0
    assert restoring(p, euler_to_quat(0.1, 0, 0))[3] < 0  # moment on vehicle = -g > 0: grows roll


def test_small_angle_roll_period_matches_closed_form():
    bg, Ix, Kp_dot = 0.02, 0.16, 0.12
    p = vehicle(bg=bg, inertia=(Ix, 0.16, 0.16))
    sim = Simulator(p, enable_damping=False)
    period = 2 * np.pi * np.sqrt((Ix + Kp_dot) / (p.weight * bg))
    dt = period / 400
    traj = sim.simulate(State.at_rest(euler=(np.deg2rad(2.0), 0, 0)), dt, 1200)
    roll = np.array([quat_to_euler(s.q)[0] for s in traj])
    crossings = np.where(np.diff(np.sign(roll)) != 0)[0]
    measured = 2 * np.mean(np.diff(crossings)) * dt
    assert measured == pytest.approx(period, rel=5e-3)


def test_added_mass_step_response_uses_m_plus_ma_and_not_legacy_sign():
    p = vehicle()
    sim = Simulator(p, enable_damping=False, enable_restoring=False)
    F = 10.0
    a = sim.acceleration(State.at_rest(), np.array([F, 0, 0, 0, 0, 0]))
    assert a[0] == pytest.approx(F / (p.mass + 5.5), rel=1e-12)
    legacy = F / (p.mass - 5.5)  # FossenDynamics.cs +M_A*nudot recurrence (inventory V1)
    assert legacy / a[0] > 1.9


def test_linear_damping_step_response_matches_exponential():
    p = vehicle(quad=(0, 0, 0, 0, 0, 0))
    sim = Simulator(p)
    F, Xu, Mx = 8.0, 4.0, p.mass + 5.5
    dt, n = 0.01, 2000
    traj = sim.simulate(State.at_rest(), dt, n, tau_fn=lambda t, s: np.array([F, 0, 0, 0, 0, 0]))
    t = np.arange(n + 1) * dt
    u = np.array([s.nu[0] for s in traj])
    np.testing.assert_allclose(u, F / Xu * (1 - np.exp(-t * Xu / Mx)), atol=1e-6)


def test_kinetic_plus_potential_energy_is_conserved_without_damping():
    p = vehicle(r_g=(0.01, -0.005, 0.02), bg=0.03, net_buoyancy=1.5, coupled_added_mass=True)
    sim = Simulator(p, enable_damping=False)
    s0 = State(np.zeros(3), euler_to_quat(0.2, -0.1, 0.4), np.array([0.5, -0.2, 0.1, 0.8, -0.6, 1.2]))
    traj = sim.simulate(s0, 0.002, 5000)
    e = np.array([sim.total_energy(s) for s in traj])
    assert np.max(np.abs(e - e[0])) / abs(e[0]) < 1e-6


def test_damping_decay_matches_dissipated_power_and_is_monotone():
    p = vehicle(bg=0.02)
    sim = Simulator(p)
    s0 = State(np.zeros(3), euler_to_quat(0.3, 0.1, 0.0), np.array([0.8, 0.3, -0.2, 0.5, 0.4, -0.9]))
    dt = 0.002
    traj = sim.simulate(s0, dt, 3000)
    e = np.array([sim.total_energy(s) for s in traj])
    pdis = np.array([sim.dissipated_power(s) for s in traj])
    assert np.all(np.diff(e) <= 1e-9)
    lost = e[0] - e[-1]
    assert lost == pytest.approx(np.trapezoid(pdis, dx=dt), rel=1e-4)
    assert lost > 0.5 * (e[0] - min(e))  # most energy is gone, not hidden elsewhere


def test_vehicle_drifts_with_current_and_relative_velocity_vanishes():
    p = vehicle()
    current = np.array([0.4, -0.1, 0.0])
    sim = Simulator(p, current_n=current)
    s = sim.simulate(State.at_rest(euler=(0, 0, np.deg2rad(70))), 0.02, 3000)[-1]
    v_ground = quat_to_rot(s.q) @ s.nu[:3]
    np.testing.assert_allclose(v_ground, current, atol=2e-3)
    np.testing.assert_allclose(sim.relative_velocity(s.q, s.nu), 0.0, atol=2e-3)


def test_damping_depends_only_on_relative_velocity():
    p = vehicle()
    still = Simulator(p)
    moving = Simulator(p, current_n=[0.3, 0.2, -0.05])
    q = euler_to_quat(0.0, 0.0, 0.5)
    nu_r = np.array([0.2, -0.1, 0.05, 0, 0, 0])
    nu = nu_r + np.concatenate([quat_to_rot(q).T @ moving.current_n, np.zeros(3)])
    a1 = still.acceleration(State(np.zeros(3), q, nu_r), np.zeros(6))
    a2 = moving.acceleration(State(np.zeros(3), q, nu), np.zeros(6))
    np.testing.assert_allclose(a1, a2, atol=1e-12)  # Galilean invariance for pure translation


def test_quadratic_damping_scales_with_speed_squared():
    p = vehicle(lin=(0, 0, 0, 0, 0, 0))
    f1 = damping_force(p, np.array([0.01, 0, 0, 0, 0, 0]))[0]
    f2 = damping_force(p, np.array([2.0, 0, 0, 0, 0, 0]))[0]
    assert f2 / f1 == pytest.approx(200.0**2)  # contrast GeneralDynamics.cs:148 (constant)


# --------------------------------------------------------------------------- #
# Thrusters, allocation, tether
# --------------------------------------------------------------------------- #
def test_eight_thruster_vectored_layout_is_fully_actuated_and_allocation_is_exact():
    T = allocation_matrix(vectored_layout(4))
    assert np.linalg.matrix_rank(T) == 6
    for _ in range(20):
        tau = RNG.normal(scale=[20, 20, 20, 2, 2, 2])
        f = np.linalg.pinv(T) @ tau
        np.testing.assert_allclose(T @ f, tau, atol=1e-9)


def test_six_thruster_layout_cannot_command_pitch():
    T = allocation_matrix(vectored_layout(2))
    assert np.linalg.matrix_rank(T) == 5
    # The unreachable wrench direction (left null space of T) lies in the surge-pitch
    # plane: horizontal thrusters sit 0.085 m below CO, so surge force and pitch
    # moment are produced together (M = z_t * X) and independent pitch is impossible.
    u, s, _ = np.linalg.svd(T)
    unreachable = u[:, -1]
    np.testing.assert_allclose(unreachable[[1, 2, 3, 5]], 0.0, atol=1e-9)
    assert abs(unreachable[4]) > 0.99
    assert unreachable[0] / unreachable[4] == pytest.approx(-0.085, rel=1e-9)


def test_saturated_allocation_preserves_wrench_direction():
    ts = vectored_layout(4)
    T = allocation_matrix(ts)
    tau = np.array([500.0, 120.0, -80.0, 0.0, 0.0, 10.0])
    f = allocate(T, tau, np.array([-t.max_reverse for t in ts]), np.array([t.max_forward for t in ts]))
    assert np.all(f <= 40.0 + 1e-9) and np.all(f >= -30.0 - 1e-9)
    achieved = T @ f
    cos = achieved @ tau / (np.linalg.norm(achieved) * np.linalg.norm(tau))
    assert cos == pytest.approx(1.0, abs=1e-12)


def test_thruster_first_order_lag_reaches_63_percent_at_tau():
    t = Thruster([0, 0, 0], [1, 0, 0], 40.0, 30.0, time_constant=0.1)
    p = vehicle()
    sim = Simulator(p, thrusters=[t], enable_damping=False, enable_restoring=False)
    s = State.at_rest(n_thrusters=1)
    traj = sim.simulate(s, 0.001, 100, f_cmd_fn=lambda tt, ss: np.array([20.0]))
    assert traj[-1].thrust[0] == pytest.approx(20.0 * (1 - np.exp(-1.0)), rel=1e-6)
    # Saturation clamps commands above the forward limit.
    traj = sim.simulate(s, 0.001, 2000, f_cmd_fn=lambda tt, ss: np.array([1000.0]))
    assert traj[-1].thrust[0] == pytest.approx(40.0, rel=1e-6)


def test_tether_is_zero_when_slack_and_restoring_when_taut():
    teth = Tether(anchor_n=np.zeros(3), attach_b=np.array([-0.2, 0, 0]), length=5.0, stiffness=200.0)
    q = euler_to_quat(0, 0, 0)
    np.testing.assert_allclose(teth.wrench(np.array([3.0, 0, 0]), q, np.zeros(6)), 0.0)
    w = teth.wrench(np.array([6.2, 0, 0]), q, np.zeros(6))
    assert w[0] == pytest.approx(-200.0 * 1.0)  # pulled back toward the anchor
    p = vehicle()
    sim = Simulator(p, tether=teth)
    s = sim.simulate(State(np.array([4.5, 0, 0]), q, np.array([1.0, 0, 0, 0, 0, 0])), 0.01, 2000)[-1]
    assert np.linalg.norm(s.pos + quat_to_rot(s.q) @ teth.attach_b) <= 5.0 + 0.05


# --------------------------------------------------------------------------- #
# Identifiability-failure cases
# --------------------------------------------------------------------------- #
def _surge_data(tau_fn, seconds=60.0, dt=0.01):
    p = vehicle()
    sim = Simulator(p)
    traj = sim.simulate(State.at_rest(), dt, int(seconds / dt), tau_fn=lambda t, s: np.array([tau_fn(t), 0, 0, 0, 0, 0]))
    u = np.array([s.nu[0] for s in traj])
    t = np.arange(len(u)) * dt
    udot = np.gradient(u, dt)
    tau = np.array([tau_fn(x) for x in t])
    return p, u[50:-50], udot[50:-50], tau[50:-50]


def test_rich_surge_excitation_recovers_effective_mass_and_damping():
    p, u, udot, tau = _surge_data(lambda t: 15 * np.sin(0.4 * t) + 10 * np.sin(1.7 * t) + 5)
    phi = ident.surge_regressor(udot, u)
    assert ident.analyse(phi).identifiable
    theta = ident.fit(phi, tau)
    np.testing.assert_allclose(theta, [p.mass + 5.5, 4.0, 18.2], rtol=2e-2)


def test_mass_and_added_mass_cannot_be_separated_from_motion_data():
    _, u, udot, _ = _surge_data(lambda t: 15 * np.sin(0.4 * t) + 5)
    d = ident.analyse(ident.surge_regressor_split_mass(udot, u))
    assert not d.identifiable and d.rank == 3
    # Null direction is (m, M_A) = (+1, -1): only the sum is observable.
    v = d.null_space[:, 0]
    assert v[0] == pytest.approx(-v[1]) and abs(v[2]) < 1e-9 and abs(v[3]) < 1e-9


def test_constant_speed_data_cannot_identify_added_mass_or_split_damping():
    _, u, udot, _ = _surge_data(lambda t: 12.0, seconds=150.0, dt=0.05)
    tail = slice(-1000, None)  # last 50 s: steady state only
    d = ident.analyse(ident.surge_regressor(udot[tail], u[tail]), rel_tol=1e-6)
    assert not d.identifiable


def test_static_hover_only_identifies_net_buoyancy_and_net_moment():
    assert ident.analyse(ident.hover_regressor(100)).rank == 1  # only W - B
    phi = np.deg2rad(np.linspace(-20, 20, 41))
    assert ident.analyse(ident.roll_static_regressor(phi)).rank == 1  # only z_g W - z_b B
