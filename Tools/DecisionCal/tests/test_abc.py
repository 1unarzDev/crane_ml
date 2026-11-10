"""ABC (sample-based) calibration arm: tests. The module is abc_fit.py (abc would shadow the stdlib)."""
import importlib.util
from pathlib import Path

import numpy as np
import pytest

import probes
import twin

import abc_fit as dabc


def _truth(**kw):
    p = twin.nominal_particles(1)
    p.update(kw)
    return {k: np.asarray(v, float).reshape(1) for k, v in p.items()}


def _evidence(seed, **kw):
    spec = twin.to_spec(_truth(**kw), sensors=twin.SENSORS)      # default sensor noise
    return probes.generate(spec, seed)[0]


@pytest.fixture(scope='module')
def ev_inf():
    n = twin.NOMINAL
    return _evidence(11, m11=2.5 * n['m11'], m22=2.5 * n['m22'], m66=2.5 * n['m66'], delay_s=0.25, tau_s=0.2,
                     rev_hold_s=0.2, rev_ratio=0.75, odom_frac=0.5)


def test_joint_posterior_moves_toward_truth(ev_inf):
    """Achievable at n_prop=512 (15 free dimensions, rejection-style ABC; checked on fit seeds 1-3, evidence
    seed 11): delay+tau RMS error falls 4-24 % vs the prior (asserted >= 10 % on seed 1), median position error
    falls ~20-35 %; inertia is only weakly identified by 512 samples (RMS error 0.57-0.68 vs prior 0.63), so
    it is asserted not to degrade (<= 1.1 x prior RMS) and the numbers are printed."""
    n = twin.NOMINAL
    p, rep = dabc.fit(ev_inf, 'joint', 200, 1, n_prop=512)
    assert rep['ess'] >= 50 - 1e-6
    prior = dabc.propose('joint', 512, np.random.default_rng(1))     # the proposal = the prior
    lag = lambda q: q['delay_s'] + q['tau_s']                      # truth 0.45
    inert = lambda q: np.mean([q[k] / n[k] for k in ('m11', 'm22', 'm66')], 0)   # truth 2.5
    rms = lambda x, t: float(np.sqrt(np.mean((x - t) ** 2)))
    pr_lag, po_lag = rms(lag(prior), 0.45), rms(lag(p), 0.45)
    pr_m, po_m = rms(inert(prior), 2.5), rms(inert(p), 2.5)
    print(f'\n[abc test1] delay+tau (truth 0.45): RMS err prior {pr_lag:.3f} post {po_lag:.3f} (median '
          f'{np.median(lag(prior)):.3f} -> {np.median(lag(p)):.3f}); inertia x (truth 2.5): RMS err prior {pr_m:.2f} '
          f'post {po_m:.2f} (median {np.median(inert(prior)):.2f} -> {np.median(inert(p)):.2f}); median pos err '
          f'prior {rep["prior_median_pos_err"]:.3f} post {rep["post_median_pos_err"]:.3f}; temper {rep["temper"]:.1f}')
    assert rep['post_median_pos_err'] < 0.9 * rep['prior_median_pos_err']
    assert po_lag < 0.9 * pr_lag
    assert po_m <= 1.1 * pr_m


def test_deterministic_by_seed(ev_inf):
    a, ra = dabc.fit(ev_inf, 'timing', 40, 5, n_prop=48)
    b, rb = dabc.fit(ev_inf, 'timing', 40, 5, n_prop=48)
    c, _ = dabc.fit(ev_inf, 'timing', 40, 6, n_prop=48)
    for k in a:
        assert np.array_equal(a[k], b[k])
    assert ra['best'] == rb['best']
    assert not np.array_equal(a['delay_s'], c['delay_s'])


def test_arms_vary_only_their_free_parameters(ev_inf):
    for arm in twin.ARMS:
        if arm in ('nominal', 'nominal_d100', 'broad'):
            with pytest.raises(ValueError):
                dabc.fit(ev_inf, arm, 20, 1, n_prop=16)
            continue
        p, rep = dabc.fit(ev_inf, arm, 30, 2, n_prop=48)
        assert set(p) == set(twin.NOMINAL) and rep['arm'] == arm
        free = set(dabc._free_names(arm)[0]) | ({'odom_frac'} if 'delay_s' in twin.FREE[arm][0] else set())
        for k, v in p.items():
            assert v.shape == (30,) and v.dtype == float and np.isfinite(v).all()
            if k not in free:
                assert np.all(v == twin.NOMINAL[k]) if k != 'odom_frac' else np.all(v == 0.0), (arm, k)
        assert any(np.ptp(p[k]) > 0 for k in free) or arm == 'x'


def test_timing_arm_aliasing_is_reported():
    ev = _evidence(21, delay_s=0.0, tau_s=0.45, rev_hold_s=0.5)
    p, rep = dabc.fit(ev, 'timing', 200, 4, n_prop=256)
    print(f'\n[abc test4] zero delay, tau=0.45, rev_hold=0.5: timing-arm delay median {np.median(p["delay_s"]):.3f} '
          f'(best {rep["best"]["delay_s"]:.3f}); ess {rep["ess"]:.1f}')
    assert len(p['delay_s']) == 200
