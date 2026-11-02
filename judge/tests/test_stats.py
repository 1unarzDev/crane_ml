"""judge.stats: golden parity with the cluster's promotion statistics, known reference values."""

import random

import pytest

from judge import stats

promotion = pytest.importorskip('cluster.crane_cluster.promotion')


@pytest.mark.parametrize('seed', range(25))
def test_wilcoxon_and_holm_match_promotion(seed):
    rng = random.Random(seed)
    n = rng.choice([3, 8, 15, 30, 31, 45])
    diffs = [round(rng.gauss(0.2, 1.0), rng.choice([0, 1, 3])) for _ in range(n)]
    assert stats.wilcoxon_greater(diffs) == promotion.wilcoxon_greater(diffs)
    ps = {f'v{k}': rng.random() for k in range(rng.randint(1, 6))}
    assert stats.holm(ps) == promotion.holm(ps)
    scores = {f'v{k}': rng.random() for k in range(rng.randint(1, 7))}
    assert stats.halving_survivors(scores) == promotion.halving_survivors(scores)


def test_reference_intervals():
    lo, hi = stats.wilson(0, 20)
    assert lo == 0 and abs(hi - 0.1611) < 1e-3
    lo, hi = stats.clopper_pearson(0, 20)
    assert lo == 0 and abs(hi - 0.1684) < 1e-3          # NLC pilot's 0/20 bound
    lo, hi = stats.clopper_pearson(5, 20)
    assert abs(lo - 0.0866) < 1e-3 and abs(hi - 0.4910) < 1e-3
    d, (l2, h2) = stats.newcombe(10, 50, 20, 50)
    assert abs(d - 0.2) < 1e-12 and l2 < 0.2 < h2
    assert stats.mcnemar_exact(0, 6) == pytest.approx(0.03125)
    assert stats.t_cdf(2.0, 10) == pytest.approx(0.963306, abs=1e-5)


def test_paired_t_and_ni():
    m, (lo, hi), p, n = stats.paired_t([0.1, 0.2, 0.15, 0.12, 0.18])
    assert lo < m < hi and p < 0.01 and n == 5
    assert stats.ni_verdict(-0.02, (-0.04, 0.0), 0.05) == 'non-inferior'
    assert stats.ni_verdict(-0.2, (-0.3, -0.1), 0.05) == 'inferior'
    assert stats.ni_verdict(-0.02, (-0.1, 0.06), 0.05) == 'inconclusive'
    assert stats.ni_verdict(0.0, (-1, 1), None) == 'margin undeclared'
    assert stats.ni_verdict(0.02, (0.0, 0.04), 0.05, 'lower') == 'non-inferior'
    assert stats.ni_verdict(0.04, (0.0, 0.08), 0.05, 'lower') == 'inconclusive'


def test_rank_correlations():
    assert stats.kendall_tau([1, 2, 3, 4], [1, 2, 3, 4]) == 1.0
    assert stats.kendall_tau([1, 2, 3, 4], [4, 3, 2, 1]) == -1.0
    assert stats.spearman([1, 2, 3], [10, 20, 30]) == pytest.approx(1.0)
