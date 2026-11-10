"""study_probe_rate: the parameterised generator reproduces probes.generate, and the regression operator matches
the log's operator under every measurement variant (in-family, noise-free boat -> the true grid point)."""
import numpy as np
import pytest

import hidden
import probes
import study_probe_rate as sp
import twin

TRUTHS = [dict(delay_s=0.25, tau_s=0.2, rev_hold_s=0.2), dict(delay_s=0.47, tau_s=0.0, rev_hold_s=0.5)]


def _boat(delay_s, tau_s, rev_hold_s):
    p = twin.nominal_particles(1)
    p.update(m11=np.array([90.]), m22=np.array([120.]), m66=np.array([12.]), delay_s=np.array([delay_s]),
             tau_s=np.array([tau_s]), rev_hold_s=np.array([rev_hold_s]), rev_ratio=np.array([0.75]),
             deadband_q=np.array([0.04]))
    return twin.to_spec(p, sensors=dict(pos_sd=0.0, vel_sd=0.0, yaw_sd=0.0, gyro_sd=0.0))


def test_v0_is_bit_exact_probes_generate():
    h = hidden.sample_hidden(1, 9595, 'timing')
    a, _ = probes.generate(h, 11)
    b, _ = sp.generate_variant(h, 11, sp.VARIANTS['V0'])
    for sa, sb in zip(a.segments, b.segments):
        assert np.array_equal(sa.x, sb.x) and np.array_equal(sa.u, sb.u)
        assert np.array_equal(sa.meta['gps_t'], sb.meta['gps_t']) and np.array_equal(sa.meta['gyro_t'], sb.meta['gyro_t'])


def test_variants_are_whole_ticks_and_share_common_noise():
    h = hidden.sample_hidden(1, 9595, 'timing')
    e0, _ = sp.generate_variant(h, 11, sp.VARIANTS['V0'])
    e1, _ = sp.generate_variant(h, 11, sp.VARIANTS['V1'])
    s0, s1 = e0.segments[0], e1.segments[0]
    assert len(s1.meta['gps_t']) == 5 * len(s0.meta['gps_t']) - 4 or len(s1.meta['gps_t']) > 4 * len(s0.meta['gps_t'])
    assert set(np.round(s0.meta['gps_t'], 6)) <= set(np.round(s1.meta['gps_t'], 6))      # V0 samples are a subset
    assert np.allclose(s0.t, s1.t)                                                       # same 0.1 s grid
    with pytest.raises(ValueError):
        sp.generate_variant(h, 1, dict(sp.VARIANTS['V1'], gps_dt=0.05))                  # 2.5 ticks
    with pytest.raises(ValueError):
        sp.generate_variant(h, 1, dict(sp.VARIANTS['V1'], gyro_box=7))                   # Regression hard-codes 5


def test_matched_operator_restores_twin():
    orig = twin.sg_velocity
    with sp.matched_operator(0.3):
        assert twin.sg_velocity is not orig
    assert twin.sg_velocity is orig


@pytest.mark.parametrize('truth', TRUTHS)
@pytest.mark.parametrize('vname', ['V0', 'V1', 'V2', 'V3'])
def test_joint_fit_recovers_true_grid_point_under_each_variant(vname, truth):
    ev, _ = sp.generate_variant(_boat(**truth), 1, sp.VARIANTS[vname])
    reg = sp.build_regression(ev)
    assert reg.smoothed
    _, rep = twin.fit(reg, 'joint', 8, 0)
    m = rep['map']
    assert (m['delay_s'], m['tau_s'], m['rev_hold_s'], m['rev_ratio'], m['deadband_q']) == (
        truth['delay_s'], truth['tau_s'], truth['rev_hold_s'], 0.75, 0.04), m
    assert abs(rep['block_map']['surge'][0] - 90.0) / 90.0 < 0.05, rep['block_map']['surge']


def test_mismatched_window_is_detectably_worse():
    """Fitting the narrow-window log with twin's default (+-1 s) operator biases the inertia; the matched
    operator does not. Guards against the patch silently not being applied."""
    ev, _ = sp.generate_variant(_boat(**TRUTHS[0]), 1, sp.VARIANTS['V2'])
    err = {}
    for half in (0.3, 1.0):
        _, rep = twin.fit(sp.build_regression(ev, half), 'joint', 8, 0)
        err[half] = abs(rep['block_map']['surge'][0] - 90.0)
    assert err[0.3] < 0.5 * err[1.0], err
