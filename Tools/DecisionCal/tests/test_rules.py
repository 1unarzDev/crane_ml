import numpy as np

import study_rules as sr


def test_rules_pick_expected_controllers():
    # c0: best mean but one bad particle; c1: slowest; c2: steady mediocre
    J = np.array([[10.0] * 7 + [100.0], [30.0] * 8, [25.0] * 8])
    assert sr.select(J, 'mean') == 0
    assert sr.select(J, 'worst') == 2
    assert sr.select(J, 'cvar25') == 2
    assert sr.select(J, 'dockfirst') == 0       # every particle docks: ties by mean


def test_dockfirst_prefers_dock_fraction_over_mean():
    J = np.array([[10.0, 10.0, 10.0, 200.0], [70.0, 70.0, 70.0, 70.0]])
    assert sr.select(J, 'mean') == 0 and sr.select(J, 'dockfirst') == 1
    assert sr.select(J, 'mean+sd') == 1


def test_timing_prior_overrides_only_timing():
    base = {'m11': np.ones(5), 'delay_s': np.full(5, 0.25), 'tau_s': np.zeros(5), 'rev_hold_s': np.zeros(5),
            'odom_frac': np.zeros(5)}
    p = sr.timing_prior_particles(base, 1)
    assert np.all(p['m11'] == 1)
    assert p['delay_s'].max() <= 0.6 and p['tau_s'].max() <= 0.5 and p['odom_frac'].max() <= 1
    assert not np.allclose(p['delay_s'], 0.25)


def test_forbidden_seed():
    import pytest
    with pytest.raises(SystemExit):
        sr.main(['--out', '/tmp/x_never', '--seed', '20261112'])
