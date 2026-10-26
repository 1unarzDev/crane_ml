"""Baseline competence on standard noise-free benchmarks (published expectations), 10 replicates each.
If one of these fails the implementation is wrong; do not loosen it, fix the optimizer."""
import numpy as np
import pytest

from crane_experiment import benchmarks as B
from crane_experiment.stats import bootstrap_ci, mannwhitney_less

REPS = 10
_cache = {}


def regrets(name, opt, dim, n):
    key = (name, opt, dim, n)
    if key not in _cache:
        _cache[key] = [B.run(name, opt, dim, n, seed) for seed in range(REPS)]
    return _cache[key]


def test_mannwhitney_sanity():
    assert mannwhitney_less(range(10), range(20, 30)) < 1e-3
    assert mannwhitney_less(range(20, 30), range(10)) > 0.99
    assert 0.3 < mannwhitney_less([1, 2, 3, 4, 5, 6], [1, 2, 3, 4, 5, 6]) < 0.7
    lo, hi = bootstrap_ci(np.arange(100.0))
    assert 35 < lo < 50 < hi < 65


def test_branin_gp_bo_beats_random_30_trials():
    g, r = regrets("branin", "gp_bo", 2, 30), regrets("branin", "random", 2, 30)
    assert np.median(g) < 0.25 * np.median(r), (np.median(g), np.median(r))
    assert mannwhitney_less(g, r) < 0.05


@pytest.mark.parametrize("opt", ["gp_bo", "tpe"])
def test_hartmann6_beats_random_60_trials(opt):
    g, r = regrets("hartmann6", opt, 6, 60), regrets("hartmann6", "random", 6, 60)
    assert np.median(g) < np.median(r)
    assert mannwhitney_less(g, r) < 0.05, (np.median(g), np.median(r))


@pytest.mark.parametrize("fn", ["ellipsoid", "rosenbrock"])
def test_cmaes_beats_random_10d_500_evals(fn):
    g, r = regrets(fn, "cmaes", 10, 500), regrets(fn, "random", 10, 500)
    assert np.median(g) < np.median(r)
    assert mannwhitney_less(g, r) < 0.05, (np.median(g), np.median(r))


def test_cmaes_default_population_is_hansen_lambda():
    from crane_experiment.optimizers.cmaes import CMA
    import math
    for n in (2, 5, 10):
        assert CMA(np.zeros(n), 1.0, 0).lam == 4 + int(3 * math.log(n))


def test_exact_mannwhitney_known_values():
    from math import comb
    assert mannwhitney_less(range(10), range(20, 30)) == pytest.approx(1 / comb(20, 10), rel=1e-12)   # 5.41e-6
    assert mannwhitney_less(range(3), range(10, 13)) == pytest.approx(1 / 20)
    assert mannwhitney_less([1, 2, 3, 4], [5, 6, 7, 8]) == pytest.approx(1 / 70)
    # 3 vs 3, one inversion: P(U<=1) = 2/20
    assert mannwhitney_less([1, 2, 5], [3, 4, 6]) == pytest.approx(4 / 20)    # U=2 -> (1+1+2)/20
    assert mannwhitney_less([1, 2, 4], [3, 5, 6]) == pytest.approx(2 / 20)    # U=1
    from crane_experiment.stats import u_counts
    for m, n in [(5, 7), (10, 10), (20, 20)]:
        assert u_counts(m, n).sum() == comb(m + n, m)
    assert mannwhitney_less(range(10), range(20, 30)) < 9.1e-5                # below the normal-approx floor
    assert 0.4 < mannwhitney_less([1, 2, 3, 4, 5, 6], [1, 2, 3, 4, 5, 6]) < 0.7   # ties -> approximation path


MF_REPS, MF_BUDGET = 12, 7.0     # Forrester, cost ratio 1:10, equal total cost (7 high-fidelity equivalents)


def test_mfbo_beats_single_fidelity_gp_bo_on_forrester_at_equal_cost():
    m = [B.run_forrester("mfbo", MF_BUDGET, s) for s in range(MF_REPS)]
    g = [B.run_forrester("gp_bo", MF_BUDGET, s) for s in range(MF_REPS)]
    assert np.median(m) < np.median(g), (np.median(m), np.median(g))
    assert mannwhitney_less(m, g) < 0.05, (mannwhitney_less(m, g), sorted(m), sorted(g))


def test_forrester_low_fidelity_relation_and_optimum():
    x = np.linspace(0, 1, 7)
    for v in x:
        assert B.forrester_low(v) == pytest.approx(0.5 * B.forrester_high(v) + 10 * (v - 0.5) - 5)
    grid = np.linspace(0, 1, 20001)
    assert min(B.forrester_high(v) for v in grid) == pytest.approx(B.FORRESTER_FMIN, abs=1e-3)
    assert B.FORRESTER_COST["sim_high"] / B.FORRESTER_COST["sim_low"] == pytest.approx(10)


def test_contextual_gp_beats_noncontextual_gp_bo_on_heldout_contexts():
    c = [B.run_contextual("contextual_gp", 40, s) for s in range(10)]
    g = [B.run_contextual("gp_bo", 40, s) for s in range(10)]
    assert np.median(c) < 0.1 * np.median(g), (np.median(c), np.median(g))
    assert mannwhitney_less(c, g) < 0.05
