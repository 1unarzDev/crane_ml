"""Provenance and physical-consistency tests for the BlueROV2 Heavy profile."""
from __future__ import annotations

import copy
import json
from pathlib import Path

import numpy as np
import pytest

import vehicle_profile as vp
from fossen6dof import Simulator, State, allocation_matrix, euler_to_quat, quat_to_euler, quat_to_rot, restoring

PROFILE = Path(__file__).with_name("profiles") / "bluerov2_heavy.json"


@pytest.fixture(scope="module")
def data():
    return vp.load(PROFILE)


def _walk_cited(node, path=""):
    if isinstance(node, dict):
        if "value" in node:
            yield path, node
            return
        for k, v in node.items():
            if k in {"note", "sources", "frames", "evidenceNote"}:
                continue
            yield from _walk_cited(v, f"{path}.{k}")


def test_every_coefficient_cites_a_known_source_and_location(data):
    sections = ["rigidBody", "hydrodynamics", "thrusters", "tether", "publishedPerformance", "alternatePrior"]
    seen = 0
    for sec in sections:
        for path, node in _walk_cited(data[sec], sec):
            assert node["source"] in data["sources"], path
            assert node.get("location"), path
            seen += 1
    assert seen > 50
    for sid, s in data["sources"].items():
        assert s.get("license"), sid


def test_profile_is_not_claimed_as_calibrated(data):
    assert data["evidenceLabel"] == "UNKNOWN"
    bad = copy.deepcopy(data)
    bad["evidenceLabel"] = "PHYSICALLY_CALIBRATED"
    tmp = PROFILE.with_name("_tmp_bad.json")
    try:
        tmp.write_text(json.dumps(bad))
        with pytest.raises(vp.ProfileError):
            vp.load(tmp)
    finally:
        tmp.unlink(missing_ok=True)


def test_bare_numbers_and_unknown_sources_are_rejected(data):
    bare = copy.deepcopy(data)
    bare["hydrodynamics"]["addedMass"]["u"] = 6.36
    with pytest.raises(vp.ProfileError):
        vp.to_params(bare)
    unknown = copy.deepcopy(data)
    unknown["rigidBody"]["massKg"]["source"] = "forum_post"
    with pytest.raises(vp.ProfileError):
        vp.to_params(unknown)


def test_weight_and_buoyancy_follow_from_cited_primitives(data):
    p = vp.to_params(data)
    rb = data["rigidBody"]
    assert p.weight == pytest.approx(rb["weightN"]["value"], rel=1e-4)
    rho, nabla, g = rb["waterDensityKgM3"]["value"], rb["displacedVolumeM3"]["value"], rb["gravityMps2"]["value"]
    assert p.buoyancy == pytest.approx(rho * g * nabla, rel=1e-5)
    assert p.weight - p.buoyancy == pytest.approx(0.98, abs=0.01)  # slightly heavy
    assert p.bg == pytest.approx(0.01)


def test_heavy_layout_is_fully_actuated(data):
    T = allocation_matrix(vp.to_thrusters(data))
    assert T.shape == (6, 8)
    assert np.linalg.matrix_rank(T) == 6


def test_max_thrust_reproduces_published_table4_forces(data):
    ts = vp.to_thrusters(data)
    T = allocation_matrix(ts)
    surge = sum(t.max_forward * abs(t.direction[0]) for t in ts[:4])
    heave = sum(t.max_forward for t in ts[4:])
    assert surge == pytest.approx(data["publishedPerformance"]["maxSurgeForceN"]["value"], rel=0.03)
    assert heave == pytest.approx(data["publishedPerformance"]["maxHeaveForceN"]["value"], rel=0.03)
    # Pure surge needs no yaw/sway from a symmetric horizontal set.
    f = np.linalg.pinv(T) @ np.array([surge, 0, 0, 0, 0, 0])
    np.testing.assert_allclose((T @ f)[1:3], 0.0, atol=1e-9)


@pytest.mark.parametrize(
    "dof,force_key,speed_key",
    [(0, "maxSurgeForceN", "maxSurgeSpeedMps"), (2, "maxHeaveForceN", "maxHeaveSpeedMps")],
)
def test_simulated_terminal_speed_matches_published_table4(data, dof, force_key, speed_key):
    p = vp.to_params(data)
    sim = Simulator(p, enable_restoring=False)
    tau = np.zeros(6)
    tau[dof] = data["publishedPerformance"][force_key]["value"]
    s = sim.simulate(State.at_rest(), 0.01, 1500, tau_fn=lambda t, s: tau)[-1]
    assert abs(s.nu[dof]) == pytest.approx(data["publishedPerformance"][speed_key]["value"], rel=0.03)


def test_wu_alternate_prior_is_inconsistent_with_heavy_performance(data):
    """Falsifier for using standard-BlueROV2 numbers on a Heavy: surge speed ~2 m/s, not 0.72."""
    alt = data["alternatePrior"]
    xu, xuu = alt["linearDamping"]["value"][0], alt["quadraticDamping"]["value"][0]
    F = data["publishedPerformance"]["maxSurgeForceN"]["value"]
    u = (-xu + np.sqrt(xu**2 + 4 * xuu * F)) / (2 * xuu)
    assert u > 2.5 * data["publishedPerformance"]["maxSurgeSpeedMps"]["value"]


def test_heavy_roll_period_and_restoring(data):
    p = vp.to_params(data)
    np.testing.assert_allclose(restoring(p, euler_to_quat(0.2, 0, 0))[3], p.buoyancy * 0.01 * np.sin(0.2), rtol=1e-12)  # r_g = 0: K = -z_b B sin(phi)
    # Gravity-buoyancy couple uses W*z_g - B*z_b; with W != B the arm is not exactly BG.
    sim = Simulator(p, enable_damping=False)
    s0 = State.at_rest(euler=(np.deg2rad(2), 0, 0))
    period = 2 * np.pi * np.sqrt((0.26 + 0.189) / (p.buoyancy * 0.01))
    dt = period / 400
    # Hold the 0.98 N net weight with a world-fixed trim force (as a hover controller would);
    # otherwise the vehicle sinks and the heave-roll Munk coupling detunes the period.
    trim = lambda t, s: np.concatenate([quat_to_rot(s.q).T @ [0, 0, p.buoyancy - p.weight], np.zeros(3)])  # noqa: E731
    traj = sim.simulate(s0, dt, 1000, tau_fn=trim)
    roll = np.array([quat_to_euler(s.q)[0] for s in traj])
    z = np.where(np.diff(np.sign(roll)) != 0)[0]
    assert 2 * np.mean(np.diff(z)) * dt == pytest.approx(period, rel=1e-2)
