"""Mapping checks for the ORACLE projection (study_oracle_twin.project). Fast."""
import numpy as np

import hidden
import study_oracle_twin as so
import twin
from plant import Plant


def _fly(spec, T=6.0):
    n = spec.n
    p = Plant(spec, np.arange(n, dtype=np.uint64) + 1, t_max=T + 1)
    q = np.array([0.8, 0.3, 0.6, -0.2])
    for k in range(int(T / 0.02)):
        p.step(q if k < 150 else -0.5 * q)
    return p.st.copy()


def test_projection_exact_for_in_family_boat():
    """A hidden boat that IS in the family (identical thrusters, gamma 2, tau up = down, no slew, no env noise,
    thrust scale != 1) must be reproduced by the projection: same dynamics to integrator tolerance."""
    h = hidden.sample_hidden(1, 5151, 'timing')
    kb = 0.55 * twin.K_REF
    rev = 0.8
    kf = np.full((1, 4), kb * 2 / (1 + rev))
    h = h.with_(k_fwd=kf, k_rev=rev * kf, gamma=np.array([2.0]), deadband=np.full((1, 4), 0.03),
                tau_down=h.tau_up.copy(), slew=np.array([np.inf]), ou_sigma=np.zeros(1), spatial_amp=np.zeros(1),
                gust_sigma=np.zeros(1), wind=np.zeros((1, 2)), current=np.zeros((1, 2)),
                pos_sd=np.zeros(1), vel_sd=np.zeros(1), yaw_sd=np.zeros(1), gyro_sd=np.zeros(1))
    ts, info = so.oracle_spec(h, 0.0, (0.0, 0.0), 0.0, 0, M=2)
    assert abs(info['c'] - 1 / 0.55) < 1e-9
    assert np.allclose(ts.dead_time, h.dead_time[0]) and np.allclose(ts.odom_latency, h.odom_latency[0])
    sh, st = _fly(h)[0], _fly(ts)[0]
    # the twin fixes the rigid-body mass (Coriolis couples to it), so the match is tight but not bit-exact
    assert np.abs(sh[:3] - st[:3]).max() < 0.08 * max(1.0, np.abs(sh[:3]).max()), (sh, st)


def test_extra_delay_adds_to_dead_time_only():
    h = hidden.sample_hidden(1, 5151, 'timing')
    a, _ = so.oracle_spec(h, 0.0, (0, 0), 0.0, 0, M=1)
    b, _ = so.oracle_spec(h, 0.2, (0, 0), 0.0, 0, M=1)
    assert np.allclose(b.dead_time - a.dead_time, 0.2) and np.allclose(a.odom_latency, b.odom_latency)


def test_seed_guard():
    import pytest
    with pytest.raises(SystemExit):
        so.main(['--out', '/tmp/_x', '--seed', '20261112'])
