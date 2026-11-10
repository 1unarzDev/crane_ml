"""Tests for the non-greedy rollout planner (lookahead.py)."""
import sys
from pathlib import Path

import numpy as np
import pytest

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

from ForestSearch.exact import ExactTeam  # noqa: E402
from ForestSearch.instances import grid_instance  # noqa: E402
from ForestSearch.links import LinkConfig  # noqa: E402
from ForestSearch.lookahead import (exact_value_of_planner, make_lookahead_strategies,  # noqa: E402
                                    rollout_decide)
from ForestSearch.model import Instance  # noqa: E402
from ForestSearch.sim import nominal_plan, run_episode  # noqa: E402


def _tiny():
    return Instance(prior=[0.5, 0.3, 0.2], g=[[0.7, 0.4], [0.5, 0.8], [0.3, 0.6]],
                    q=np.full((3, 2), 0.8), h_good=0.3, h_bad=0.2, alpha=0.05)


def _hard():
    # Random tiny instance with the largest greedy gap among 40 draws (gap ~0.012 at k=2).
    return Instance(prior=[0.009, 0.337, 0.654],
                    g=[[0.81, 0.34], [0.283, 0.643], [0.784, 0.919]],
                    q=[[0.483, 0.665], [0.892, 0.632], [0.724, 0.413]],
                    h_good=0.3, h_bad=0.15, alpha=0.03)


def _hard2():
    return Instance(prior=[0.22, 0.049, 0.73], g=[[0.494, 0.852], [0.369, 0.118], [0.802, 0.153]],
                    q=[[0.451, 0.93], [0.814, 0.586], [0.473, 0.613]],
                    h_good=0.3, h_bad=0.15, alpha=0.03)


KW = dict(k=0, n_samples=512, pairs=6, z=1.0)


@pytest.mark.parametrize("make", [_tiny, _hard, _hard2])
@pytest.mark.parametrize("steps", [1, 2, 3])
def test_matches_exact_dp_within_0002(make, steps):
    E = ExactTeam(make())
    opt = E.value(steps)
    got = exact_value_of_planner(E, steps, **KW)
    assert got <= opt + 1e-12
    assert opt - got <= 0.002


@pytest.mark.parametrize("make", [_hard, _hard2])
def test_closes_greedy_gap_at_two_steps(make):
    E = ExactTeam(make())
    opt, gr = E.value(2), E.greedy_value(2)
    assert opt - gr > 0.008                      # greedy is clearly suboptimal here
    assert exact_value_of_planner(E, 2, **KW) == pytest.approx(opt, abs=1e-9)


def test_rollout_decide_deterministic():
    inst = grid_instance(4, 4, seed=3, travel_scale=1.0, abort=0.1)
    m0 = np.zeros((inst.n, 2)); m1 = np.zeros((inst.n, 2))
    m0[5, 0] = 1; m1[6, 1] = 1
    a = rollout_decide(inst, m0, m1, 0, 0, 2, (15, 2, None), k=6, n_samples=16, window=12,
                       pairs=3, return_values=True)
    b = rollout_decide(inst, m0, m1, 0, 0, 2, (15, 2, None), k=6, n_samples=16, window=12,
                       pairs=3, return_values=True)
    assert a[0] == b[0]
    assert np.array_equal(a[1][1], b[1][1])


def test_episode_deterministic():
    inst = grid_instance(4, 3, seed=1, travel_scale=1.0, abort=0.1, h_good=0.3, h_bad=0.15, alpha=0.03)
    L = make_lookahead_strategies(k=4, n=8, window=10, pairs=2, z=1.0)
    nom = nominal_plan(inst, L["ORACLE"])
    assert nom == nominal_plan(inst, L["ORACLE"])
    for name in ("ORACLE", "IMPLICIT", "INDEP"):
        for s in range(3):
            r1 = run_episode(inst, (name, name), LinkConfig(), s, nominal=nom, strategies=L)
            r2 = run_episode(inst, (name, name), LinkConfig(), s, nominal=nom, strategies=L)
            assert (r1.detected, r1.ttd, r1.looks, r1.redundant) == (r2.detected, r2.ttd, r2.looks, r2.redundant)


def test_proposition1_silent_replay_with_lookahead():
    """No private events, ideal link: IMPLICIT with the lookahead planner replays
    the centralised lookahead team exactly (deterministic, state-seeded planner)."""
    inst = grid_instance(4, 3, seed=1, travel_scale=1.0, abort=0.0, h_good=0.0, h_bad=0.0, alpha=0.0)
    L = make_lookahead_strategies(k=4, n=8, window=10, pairs=2, z=1.0)
    nom = nominal_plan(inst, L["ORACLE"])
    for s in range(6):
        ref = run_episode(inst, ("ORACLE", "ORACLE"), LinkConfig(), s, nominal=nom, strategies=L)
        got = run_episode(inst, ("IMPLICIT", "IMPLICIT"), LinkConfig(), s, nominal=nom, strategies=L)
        assert (got.detected, got.ttd, got.looks, got.redundant) == (ref.detected, ref.ttd, ref.looks, ref.redundant)
        assert got.bytes_sent == 0

