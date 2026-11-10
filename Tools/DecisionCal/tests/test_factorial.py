"""Structure checks for study_factorial (fast; no missions)."""
import numpy as np
import pytest

import hidden
import study_factorial as sf
import study_oracle_twin as so
import twin


def _fake_fits(M=4, rng=None):
    rng = rng or np.random.default_rng(0)
    nom = twin.nominal_particles(M)
    rigid = {k: v * rng.uniform(0.8, 1.2, M) for k, v in nom.items()}
    rd = {k: v.copy() for k, v in rigid.items()}
    rd['delay_s'], rd['odom_frac'] = rng.uniform(0.1, 0.5, M), rng.uniform(0, 1, M)
    return {'rigid': (rigid, {}), 'rigid+delay': (rd, {}), 'joint': (rd, {})}


def test_sixteen_factorial_sets_and_names():
    assert len(sf.FACT) == 16 and len(set(sf.SETS)) == len(sf.SETS)


def test_true_cell_equals_oracle_proj():
    h = hidden.sample_hidden(1, 5252, 'timing')
    M = 4
    sets, info = sf.build_sets(h, _fake_fits(M), M)
    specs = sf.to_specs(sets, (0.1, 0.0), 0.05, 7, M)
    ref, _ = so.oracle_spec(h, 0.0, (0.1, 0.0), 0.05, 7, M)
    a, b = specs['T|t|L'], ref
    for f in ('mass', 'a11', 'a22', 'a66', 'd1', 'd2', 'k_fwd', 'k_rev', 'deadband', 'dead_time', 'odom_latency', 'tau_up',
              'tau_down', 'rev_hold', 'current'):
        assert np.allclose(getattr(a, f), getattr(b, f)), f


def test_delay_and_lag_levels():
    h = hidden.sample_hidden(1, 5252, 'transient')
    M = 4
    fits = _fake_fits(M)
    sets, info = sf.build_sets(h, fits, M)
    specs = sf.to_specs(sets, (0, 0), 0.0, 1, M)
    L = float(h.dead_time[0] + h.odom_latency[0])
    assert np.allclose(specs['F|n|0'].dead_time + specs['F|n|0'].odom_latency, 0)
    for k in ('t', 'r'):
        s = specs[f'F|{k}|0']
        assert np.allclose(s.dead_time + s.odom_latency, L)
    assert np.allclose(specs['F|t|0'].odom_latency, h.odom_latency[0])
    assert np.allclose(specs['F|r|0'].odom_latency, fits['rigid+delay'][0]['odom_frac'] * L)
    s = specs['F|f|0']
    assert np.allclose(s.dead_time + s.odom_latency, fits['rigid+delay'][0]['delay_s'])
    assert np.allclose(specs['F|t|0'].tau_up, 0) and np.allclose(specs['F|t|0'].rev_hold, 0)
    assert np.allclose(specs['F|t|L'].rev_hold, h.rev_hold[0])
    assert np.allclose(specs['F|t|L'].tau_up, 0.5 * (h.tau_up[0] + h.tau_down[0]))
    # body factor: F keeps particle spread, T has none
    assert np.ptp(specs['F|t|0'].a11) > 0 and np.ptp(specs['T|t|0'].a11) == 0


def test_refuses_confirmatory_seed(tmp_path):
    with pytest.raises(SystemExit):
        sf.main(['--out', str(tmp_path), '--seed', '20261110'])
