"""Must-pass: optimizer correctness (pure numpy implementations)."""
import math

import numpy as np
import pytest

from crane_experiment.optimizers import REGISTRY, make_optimizer
from crane_experiment.optimizers.cmaes import CMA
from crane_experiment.optimizers.gp import GP, expected_improvement, matern52, norm_cdf
from crane_experiment.optimizers.sobol import MAX_DIM, SobolEngine, l2_star_discrepancy


def test_sobol_unscrambled_matches_known_points():
    P = SobolEngine(2, scramble=False).random(4)
    assert np.allclose(P, [[0, 0], [.5, .5], [.75, .25], [.25, .75]])
    P8 = SobolEngine(3, scramble=False).random(8)
    assert len({tuple(r) for r in P8}) == 8


def test_sobol_dimension_limit():
    with pytest.raises(ValueError):
        SobolEngine(MAX_DIM + 1)


@pytest.mark.parametrize("seed", [0, 1, 2])
def test_owen_scramble_preserves_stratification(seed):
    P = SobolEngine(4, scramble=True, seed=seed).random(64)
    assert P.min() >= 0 and P.max() < 1
    for d in range(4):
        bins = np.floor(P[:, d] * 64).astype(int)
        assert sorted(bins) == list(range(64)), "each 1D elementary interval must hold exactly one point"
    assert not np.allclose(P, SobolEngine(4, scramble=False).random(64))


def test_scrambled_sobol_has_lower_discrepancy_than_random():
    d, n = 4, 128
    sob = [l2_star_discrepancy(SobolEngine(d, True, s).random(n)) for s in range(8)]
    rnd = [l2_star_discrepancy(np.random.default_rng(s).random((n, d))) for s in range(8)]
    assert np.mean(sob) < 0.6 * np.mean(rnd), (np.mean(sob), np.mean(rnd))
    assert max(sob) < min(rnd)


def test_cmaes_converges_on_sphere():
    es = CMA(np.full(5, 3.0), 1.0, seed=1)
    for _ in range(300):
        X = es.sample()
        es.update(X, (X ** 2).sum(1))
    assert (es.mean ** 2).sum() < 1e-8


def test_cmaes_converges_on_rosenbrock():
    f = lambda X: 100 * (X[:, 1] - X[:, 0] ** 2) ** 2 + (1 - X[:, 0]) ** 2
    es = CMA(np.array([-1.0, 1.5]), 0.5, seed=3)
    for _ in range(600):
        X = es.sample()
        es.update(X, f(X))
    assert np.allclose(es.mean, [1, 1], atol=1e-3), es.mean


def test_matern_kernel_properties():
    A = np.random.default_rng(0).random((6, 3))
    K = matern52(A, A)
    assert np.allclose(np.diag(K), 1) and np.allclose(K, K.T)
    assert np.linalg.eigvalsh(K).min() > -1e-9


def test_normal_cdf_via_erf_and_ei():
    assert norm_cdf(0.0) == pytest.approx(0.5)
    assert norm_cdf(1.96) == pytest.approx(0.975, abs=1e-3)
    ei = expected_improvement(np.array([0.0, 1.0]), np.array([1.0, 1.0]), 0.0)
    assert ei[0] == pytest.approx(1 / math.sqrt(2 * math.pi), abs=1e-6) and ei[1] > ei[0]
    assert expected_improvement(np.array([-5.0]), np.array([1e-12]), 0.0)[0] < 1e-9


def test_gp_fits_a_smooth_function():
    rng = np.random.default_rng(0)
    X = rng.random((25, 2))
    f = lambda X: np.sin(3 * X[:, 0]) + X[:, 1] ** 2
    gp = GP([[0, 1]]).fit(X, f(X) + 0.01 * rng.standard_normal(25))
    Xt = rng.random((50, 2))
    mu, var = gp.predict(Xt)
    assert np.sqrt(np.mean((mu - f(Xt)) ** 2)) < 0.15 and (var > 0).all()


@pytest.mark.parametrize("name", sorted(REGISTRY))
def test_optimizers_are_deterministic_and_in_unit_cube(name):
    def run():
        o = make_optimizer(name, dim=3, seed=11, x0=np.full(3, .5), ctx_dim=1,
                           fidelities={"sim_low": .25, "sim_high": 1.0}, default_fidelity="sim_high")
        out = []
        for i in range(16):
            x, f = o.ask(np.array([.5]))
            assert x.shape == (3,) and (x >= 0).all() and (x <= 1).all()
            o.tell(x, np.array([.5]), f, -float(((x - .3) ** 2).sum()))
            out.append((tuple(np.round(x, 12)), f))
        r = o.recommend(np.array([.5]))
        assert (r >= 0).all() and (r <= 1).all()
        return out, tuple(np.round(r, 12))
    assert run() == run()


@pytest.mark.parametrize("name", ["gp_bo", "contextual_gp", "tpe", "cmaes"])
def test_model_based_optimizers_improve_on_a_smooth_noiseless_problem(name):
    f = lambda x: -float(((x - np.array([.3, .7])) ** 2).sum())
    regs = []
    for seed in range(4):
        o = make_optimizer(name, dim=2, seed=seed, x0=np.array([.8, .2]), ctx_dim=1,
                           fidelities={"sim_high": 1.0}, default_fidelity="sim_high")
        for _ in range(40):
            x, fid = o.ask(np.array([.5]))
            o.tell(x, np.array([.5]), fid, f(x))
        regs.append(-f(o.recommend(np.array([.5]))))
    assert np.median(regs) < 0.02, (name, regs)   # default point has regret 0.5


def test_mfbo_uses_both_fidelities_and_cheap_ones_more_often():
    o = make_optimizer("mfbo", dim=2, seed=0, x0=np.array([.5, .5]), fidelities={"sim_low": .25, "sim_high": 1.0},
                       default_fidelity="sim_high")
    f = lambda x, z: -float(((x - .4) ** 2).sum()) - (0.05 if z == "sim_low" else 0)
    fids = []
    for _ in range(30):
        x, z = o.ask(None)
        o.tell(x, None, z, f(x, z))
        fids.append(z)
    assert {"sim_low", "sim_high"} <= set(fids) and fids.count("sim_low") > fids.count("sim_high") * 0.5


def test_optuna_adapter_when_installed():
    pytest.importorskip("optuna")
    o = make_optimizer("optuna_tpe", dim=2, seed=0)
    x, _ = o.ask(None)
    o.tell(x, None, "sim_high", 0.0)


def test_botorch_adapter_when_installed():
    pytest.importorskip("botorch")
    o = make_optimizer("botorch_gp", dim=2, seed=0, n_init=2)
    for _ in range(4):
        x, f = o.ask(None)
        o.tell(x, None, f, float(-(x ** 2).sum()))
