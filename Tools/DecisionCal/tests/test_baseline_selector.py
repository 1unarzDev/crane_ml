"""baseline_selector: leakage guards, selectors on synthetic data, feature extraction (fast, ~10 s)."""
import numpy as np
import pytest

import baseline_selector as bs
import hidden
import probes


def _synth(n=60, C=5, seed=0):
    rng = np.random.default_rng(seed)
    X = rng.normal(size=(n, 4))
    best = (X[:, 0] > 0).astype(int) * 2          # controller 0 or 2 depending on feature 0
    Jm = 30 + 5 * rng.random((n, C))
    Jm[np.arange(n), best] -= 20
    return X, Jm, best


def test_scored_seeds_refused():
    with pytest.raises(SystemExit):
        bs.check_seed(20261111)
    with pytest.raises(SystemExit):
        bs.check_seed(8484, 8484)


def test_selectors_learn_a_feature_rule_and_constant_does_not():
    X, Jm, best = _synth()
    Xt, Jt, bt = _synth(200, seed=1)
    for m in (bs.KNN(), bs.Ridge(False), bs.Ridge(True)):
        m.fit(X, Jm)
        assert (m.select(Xt) == bt).mean() > 0.8, m.name
    assert (bs.Constant().fit(X, Jm).select(Xt) == bt).mean() < 0.7


def test_scaler_fit_on_train_only_and_imputes():
    X = np.array([[1.0, 2.0], [2.0, np.nan], [3.0, 6.0], [100.0, 4.0]])
    sc = bs.Scaler().fit(X)
    Z = sc(np.array([[np.nan, np.nan]]))
    assert np.allclose(Z, 0.0)                    # missing -> training median
    assert np.abs(sc(np.array([[1e9, -1e9]]))).max() <= 5.0


def test_strata_counts_and_subset_prefix():
    assert sum(bs.strata_counts(6).values()) == 6
    data = {'stratum': np.array(['steady'] * 2 + ['transient'] * 2), 'idx': np.array([0, 1, 0, 1]), 'X': np.zeros((4, 1))}
    assert len(bs.subset(data, 2)['X']) == 2


def test_features_from_log_only_are_finite_and_separate_timing():
    f = {}
    for st in ('steady', 'timing'):
        row = hidden.sample_hidden(1, 7373, st)
        ev, _ = probes.generate(row, 5)
        f[st] = bs.extract_features(ev)
        assert f[st].shape == (bs.NF,)
        assert np.isfinite(f[st]).all()
    d = bs.FEATURES.index('delay_yaw')
    assert f['timing'][d] > f['steady'][d]
