"""Invariant tests for the two-searcher coordination model.

Run:  python3 -m pytest Tools/ForestSearch -q   (from the repo root)
"""
import itertools
import sys
from pathlib import Path

import numpy as np
import pytest

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

from ForestSearch.exact import ExactTeam  # noqa: E402
from ForestSearch.instances import grid_instance  # noqa: E402
from ForestSearch.links import GE, DirectedLink, LinkConfig  # noqa: E402
from ForestSearch.model import (DETECT, HINT, NULL, BeliefModel, Instance,  # noqa: E402
                                detect_prob, posterior, sample_outcome, sample_truth)
from ForestSearch.sim import STRATEGIES, run_episode  # noqa: E402

SEEDS = range(60)


def _key(r):
    return (r.detected, r.ttd, r.looks, r.redundant)


def test_posterior_normalised_and_prior_at_zero_counts():
    inst = grid_instance(3, 3, seed=2)
    z = np.zeros((inst.n, inst.V), int)
    np.testing.assert_allclose(posterior(inst, z, z), inst.prior)


def test_null_look_moves_mass_away_from_cell():
    inst = grid_instance(3, 3, seed=2)
    m0 = np.zeros((inst.n, inst.V), int)
    m0[4, 0] = 1
    assert posterior(inst, m0, 0 * m0)[4] < inst.prior[4]


def test_correlated_views_diminishing_returns():
    """Repeating the same view yields strictly less detection probability than
    the first look, and less than switching to an equally good fresh view."""
    inst = Instance(prior=[0.5, 0.5], g=[[0.5, 0.5], [0.5, 0.5]], q=[[0.9, 0.9]] * 2)
    z = np.zeros((2, 2), int)
    m0 = z.copy()
    m0[0, 0] = 1
    pd = detect_prob(inst, m0, z)
    assert pd[0, 0] < pd[0, 1]
    assert pd[0, 0] < detect_prob(inst, z, z)[0, 0]


def test_g_equal_one_correlated_equals_independent():
    """With always-good views there is no latent visibility, so the
    correlated and independent belief models coincide."""
    inst = grid_instance(3, 2, seed=5, g_range=(1.0, 1.0), h_good=0.0, h_bad=0.0, alpha=0.0)
    rng = np.random.default_rng(0)
    m0 = rng.integers(0, 3, (inst.n, inst.V))
    z = 0 * m0
    np.testing.assert_allclose(posterior(inst, m0, z, BeliefModel(True)),
                               posterior(inst, m0, z, BeliefModel(False)), rtol=1e-9)


def test_outcome_frequencies_match_model():
    inst = Instance(prior=[1.0], g=[[0.6, 0.6]], q=[[0.7, 0.7]], h_good=0.2, h_bad=0.1)
    rng = np.random.default_rng(1)
    counts = np.zeros(3)
    N = 40000
    for _ in range(N):
        tr = sample_truth(inst, rng)
        counts[sample_outcome(inst, tr, 0, 0, rng.random(3))] += 1
    p_det = 0.6 * 0.7
    p_hint = 0.6 * 0.3 * 0.2 + 0.4 * 0.1
    np.testing.assert_allclose(counts[[DETECT, HINT]] / N, [p_det, p_hint], atol=0.01)


@pytest.mark.parametrize("name", ["IMPLICIT", "EVENT", "RESERVE", "FULL"])
def test_proposition1_no_private_events_ideal_link_matches_centralised(name):
    """Proposition 1: with no private events (no aborts, no hints) and an
    ideal link, every coordinated protocol reproduces the centralised
    sequential-greedy schedule exactly; implicit ones send zero bytes."""
    inst = grid_instance(4, 3, seed=1, abort=0.0, h_good=0.0, h_bad=0.0, alpha=0.0)
    for s in SEEDS:
        ref = run_episode(inst, ("ORACLE", "ORACLE"), LinkConfig(), s)
        got = run_episode(inst, (name, name), LinkConfig(), s)
        assert _key(got) == _key(ref)
        if name in ("IMPLICIT", "EVENT"):
            assert got.bytes_sent == 0


def test_proposition1_implicit_is_link_independent_without_events():
    inst = grid_instance(4, 3, seed=1, abort=0.0, h_good=0.0, h_bad=0.0, alpha=0.0)
    bad = LinkConfig(GE.bursty(0.6, 8), delay=3, name="bad")
    for s in SEEDS:
        assert _key(run_episode(inst, ("IMPLICIT",) * 2, bad, s)) == \
            _key(run_episode(inst, ("IMPLICIT",) * 2, LinkConfig(), s))


def test_full_sharing_ideal_link_matches_centralised_with_events():
    inst = grid_instance(4, 3, seed=1, abort=0.15, h_good=0.3, h_bad=0.15, alpha=0.03)
    full_window = {"FULL": STRATEGIES["FULL"].__class__(**{**STRATEGIES["FULL"].__dict__, "window": 10**6}),
                   "ORACLE": STRATEGIES["ORACLE"]}
    for s in SEEDS:
        ref = run_episode(inst, ("ORACLE",) * 2, LinkConfig(), s, strategies=full_window)
        got = run_episode(inst, ("FULL",) * 2, LinkConfig(), s, strategies=full_window)
        assert _key(got) == _key(ref)


def test_sector_never_redundant():
    inst = grid_instance(4, 4, seed=3, abort=0.1)
    for s in SEEDS:
        assert run_episode(inst, ("SECTOR",) * 2, LinkConfig(), s).redundant == 0


def test_bursty_channel_statistics():
    ch = GE.bursty(0.3, 6.0)
    assert ch.mean_loss == pytest.approx(0.3)
    link = DirectedLink(LinkConfig(ch), 200000, np.random.default_rng(0))
    assert link.bad[0].mean() == pytest.approx(0.3, abs=0.02)


def test_two_hop_base_relay_loses_more():
    ch = GE.bernoulli(0.2)
    rng = np.random.default_rng(0)
    one = DirectedLink(LinkConfig(ch, hops=1), 20000, rng)
    two = DirectedLink(LinkConfig(ch, hops=2), 20000, rng)
    d1 = np.mean([one.send(t, None) for t in range(20000)])
    d2 = np.mean([two.send(t, None) for t in range(20000)])
    assert d1 == pytest.approx(0.8, abs=0.02)
    assert d2 == pytest.approx(0.64, abs=0.02)


def _tiny():
    return Instance(prior=[0.5, 0.3, 0.2], g=[[0.7, 0.4], [0.5, 0.8], [0.3, 0.6]],
                    q=np.full((3, 2), 0.8), h_good=0.3, h_bad=0.2, alpha=0.05)


def test_exact_outcome_probabilities_sum_to_at_most_one():
    E = ExactTeam(_tiny())
    key = tuple([0] * 12)
    for act in E.actions:
        p_nd = sum(p for p, _ in E._outcomes(key, act))
        assert 0.0 < p_nd <= 1.0 + 1e-12


def test_exact_first_step_detect_prob_matches_model():
    inst = _tiny()
    E = ExactTeam(inst, agents=1)
    z = np.zeros((3, 2), int)
    pd = detect_prob(inst, z, z)
    for a, (i, v) in enumerate(E.types):
        p_nd = sum(p for p, _ in E._outcomes(tuple([0] * 12), (a,)))
        assert 1 - p_nd == pytest.approx(pd[i, v], rel=1e-9)


def test_exact_optimal_dominates_greedy():
    E = ExactTeam(_tiny())
    for k in (1, 2, 3):
        assert E.value(k) >= E.greedy_value(k) - 1e-12
    # One step: greedy (sequential virtual-NULL) is optimal or very close.
    assert E.value(1) == pytest.approx(E.greedy_value(1), abs=0.02)
