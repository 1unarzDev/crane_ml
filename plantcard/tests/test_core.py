"""Core: NIG math (parity with the NLC study's blr.py, evidence by the chain rule), grid, store."""
import math
from pathlib import Path
import sys

import numpy as np
import pytest

from plantcard.blocks import NIG, Grid

NLC = Path.home() / 'nl-constraints-bench'


def _data(seed=0, n=200, d=3):
    rng = np.random.default_rng(seed)
    X = rng.normal(size=(n, d))
    theta = np.array([2.0, -1.0, 0.5])[:d]
    return X, X @ theta + 0.3 * rng.normal(size=n)


def test_update_matches_blr_formulas():
    X, y = _data()
    p = NIG.prior([1.0, 1.0, 1.0], 0.5, 2.0)
    post = p.update(X[:120], y[:120], 0.97).update(X[120:], y[120:], 0.97)
    L0 = 0.97 * p.Lam
    L1 = L0 + X[:120].T @ X[:120]
    m1 = np.linalg.solve(L1, L0 @ p.mu + X[:120].T @ y[:120])
    assert np.allclose(post.Lam, 0.97 * L1 + X[120:].T @ X[120:])
    assert np.allclose(m1, p.update(X[:120], y[:120], 0.97).mu)
    assert abs(post.mu[0] - 2.0) < 0.1


@pytest.mark.skipif(not (NLC / 'nlc' / 'calib' / 'blr.py').exists(), reason='NLC study not on this host')
def test_parity_with_nlc_blr():
    sys.path.insert(0, str(NLC))
    from nlc.calib.blr import NIG as BLR
    X, y = _data(3)
    a = NIG.prior([1.0, 0.5, 2.0], 0.5, 2.0).update(X, y, 0.99)
    b = BLR.prior([1.0, 0.5, 2.0], 0.5, 2.0).update(X, y, 0.99)
    assert np.array_equal(a.mu, b.mu) and np.array_equal(a.Lam, b.Lam) and a.a == b.a and a.b == b.b
    assert a.record() == b.record()


def _t_logpdf(x, m, s2, nu):
    return (math.lgamma((nu + 1) / 2) - math.lgamma(nu / 2) - 0.5 * math.log(nu * math.pi * s2)
            - (nu + 1) / 2 * math.log1p((x - m) ** 2 / (nu * s2)))


def test_log_evidence_is_the_chain_of_predictives():
    X, y = _data(1, n=40)
    p = NIG.prior([1.0, 1.0, 1.0], 0.5, 2.0)
    total, cur = 0.0, p
    for i in range(len(y)):
        x = X[i]
        s2 = cur.b / cur.a * (1 + x @ np.linalg.solve(cur.Lam, x))
        total += _t_logpdf(y[i], x @ cur.mu, s2, 2 * cur.a)
        cur = cur.update(X[i:i + 1], y[i:i + 1])
    assert p.log_evidence(X, y) == pytest.approx(total, rel=1e-9)


def test_sample_moments_and_eig():
    X, y = _data(2)
    post = NIG.prior([1.0, 1.0, 1.0], 0.5, 2.0).update(X, y)
    th, _ = post.sample(20000, np.random.default_rng(0))
    assert np.allclose(th.mean(0), post.mu, atol=3 * post.std().max())
    assert np.allclose(th.std(0), post.std(), rtol=0.05)
    assert post.eig(X[:10]) > 0 and post.eig(X[:10]) < NIG.prior([1, 1, 1], 0.5, 2.0).eig(X[:10])


def test_grid_posterior():
    g = Grid.uniform({'a': [0, 1, 2], 'b': [0.5, 1.0]})
    ll = np.zeros((3, 2))
    ll[2, 1] = 10.0
    post = g.posterior(ll)
    assert post.map() == {'a': 2, 'b': 1.0}
    assert post.prob().sum() == pytest.approx(1.0)
    assert Grid.from_dict(post.to_dict()).map() == post.map()
