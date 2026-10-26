from __future__ import annotations

import numpy as np
import pytest

from fossen6dof import G, RHO_FRESH, RHO_SEA, euler_to_quat, quat_to_rot
from sensor_models import DVL, P_ATM, USBL, PressureSensor, beer_lambert, janus_beams


def test_pressure_depth_round_trip_and_port_lever_arm():
    s = PressureSensor(port_b=np.array([0.0, 0.0, 0.1]))
    q = euler_to_quat(0, 0, 0)
    p = s.measure(np.array([0, 0, 2.0]), q)
    assert p == pytest.approx(P_ATM + RHO_FRESH * G * 2.1)
    assert s.depth_from_pressure(p) == pytest.approx(2.1)
    # Pitching 90 deg nose-up moves a below-CO port to aft: depth returns to CO depth.
    q_up = euler_to_quat(0, np.pi / 2, 0)
    assert s.depth_from_pressure(s.measure(np.array([0, 0, 2.0]), q_up)) == pytest.approx(2.0, abs=1e-9)


def test_pressure_density_mismatch_is_a_depth_scale_error():
    fresh = PressureSensor(rho=RHO_FRESH)
    sea = PressureSensor(rho=RHO_SEA)
    p = sea.measure(np.array([0, 0, 10.0]), euler_to_quat(0, 0, 0))
    assert fresh.depth_from_pressure(p) / 10.0 == pytest.approx(RHO_SEA / RHO_FRESH)


def test_dvl_bottom_track_recovers_ground_velocity_and_water_track_relative_velocity():
    dvl = DVL(mount_b=np.array([-0.1, 0.0, 0.1]))
    q = euler_to_quat(0.05, -0.03, 1.0)
    nu = np.array([0.3, -0.1, 0.05, 0.02, -0.01, 0.2])
    cur = np.array([0.2, 0.1, 0.0])
    v_mount = nu[:3] + np.cross(nu[3:], dvl.mount_b)
    v, _, valid = dvl.measure(np.zeros(3), q, nu, seabed_z=5.0)
    assert valid.all()
    np.testing.assert_allclose(v, v_mount, atol=1e-12)
    vw, _, _ = dvl.measure(np.zeros(3), q, nu, seabed_z=5.0, current_n=cur, mode="water")
    np.testing.assert_allclose(vw, v_mount - quat_to_rot(q).T @ cur, atol=1e-12)


def test_dvl_loses_lock_out_of_range_and_with_too_few_beams():
    dvl = DVL(max_range=10.0)
    q = euler_to_quat(0, 0, 0)
    v, _, valid = dvl.measure(np.zeros(3), q, np.zeros(6), seabed_z=30.0)
    assert v is None and not valid.any()
    # Rolled 30 deg: the outward beams on one side lose range first.
    v, _, valid = dvl.measure(np.zeros(3), euler_to_quat(np.deg2rad(30), 0, 0), np.zeros(6), seabed_z=8.5)
    assert 0 < valid.sum() < 4


def test_dvl_noise_scales_as_expected_for_janus_geometry():
    dvl = DVL(sigma_beam=0.01)
    rng = np.random.default_rng(1)
    vs = np.array([dvl.measure(np.zeros(3), euler_to_quat(0, 0, 0), np.zeros(6), 5.0, rng=rng)[0] for _ in range(4000)])
    b = janus_beams()
    cov = 0.01**2 * np.linalg.inv(b.T @ b)
    np.testing.assert_allclose(vs.std(axis=0), np.sqrt(np.diag(cov)), rtol=0.06)
    assert vs.std(axis=0)[0] > vs.std(axis=0)[2]  # horizontal is noisier than vertical at 22.5 deg tilt


def test_usbl_geometry_and_two_way_latency():
    u = USBL(transceiver_n=np.array([0, 0, 0.5]))
    m = u.measure(np.array([30.0, 40.0, 0.5]))
    assert m["range"] == pytest.approx(50.0)
    assert m["azimuth"] == pytest.approx(np.arctan2(40, 30))
    assert m["latency"] == pytest.approx(2 * 50 / 1500)


def test_beer_lambert_limits():
    J = np.array([1.0, 0.8, 0.6])
    c = np.array([0.6, 0.12, 0.08])  # red attenuates fastest
    Binf = np.array([0.02, 0.2, 0.25])
    np.testing.assert_allclose(beer_lambert(J, 0.0, c, Binf), J)
    np.testing.assert_allclose(beer_lambert(J, 1e4, c, Binf), Binf, atol=1e-12)
    far = beer_lambert(J, 5.0, c, Binf)
    assert far[0] < far[2]  # colour shift toward blue-green with distance
