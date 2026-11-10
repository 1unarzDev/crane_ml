"""Tests for nagent.py: N=2 equivalence with variants.run_any, Proposition 1 at N=3."""
import sys
from pathlib import Path

import numpy as np
import pytest

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

from ForestSearch.event_curve import LINKS, events  # noqa: E402
from ForestSearch.instances import grid_instance  # noqa: E402
from ForestSearch.links import GE, LinkConfig  # noqa: E402
from ForestSearch.nagent import NEpisode, nominal_plan_n, run_n  # noqa: E402
from ForestSearch.sim import nominal_plan  # noqa: E402
from ForestSearch.variants import make_all_strategies, run_any  # noqa: E402

S = make_all_strategies()
FIELDS = ("detected", "ttd", "looks", "redundant", "aborts", "hints",
          "bytes_sent", "msgs_sent", "msgs_delivered")
BURST = LinkConfig(GE.bursty(0.3, 5), delay=1, name="burst")


def _key(r):
    return tuple(getattr(r, f) for f in FIELDS)


def test_nominal_plan_matches_n2():
    inst = grid_instance(4, 4, seed=3, travel_scale=1, horizon=20, **events(2))
    a = nominal_plan(inst, S["ORACLE"])
    b = nominal_plan_n(inst, 2, S)
    assert [[(t.seq, t.cell, t.view, t.start, t.finish) for t in p] for p in a] == \
           [[(t.seq, t.cell, t.view, t.start, t.finish) for t in p] for p in b]


@pytest.mark.parametrize("name", ["ORACLE", "INDEP", "IMPLICIT", "HYBRID", "HYBRID_P"])
@pytest.mark.parametrize("link", [LINKS["d0_loss0"], LINKS["d1_loss0"], BURST])
@pytest.mark.parametrize("travel", [0, 1])
def test_n2_equals_run_any(name, link, travel):
    inst = grid_instance(5, 5, seed=401, travel_scale=travel,
                         horizon=16 if travel == 0 else 40, **events(3))
    nom = nominal_plan(inst, S["ORACLE"])
    nn = nominal_plan_n(inst, 2, S)
    for k in range(25):
        a = run_any(inst, (name,) * 2, link, k, S, nom)
        b = run_n(inst, 2, name, link, k, S, nn)
        assert _key(a) == _key(b), (name, k)


@pytest.mark.parametrize("N", [3, 4])
@pytest.mark.parametrize("name", ["IMPLICIT", "HYBRID", "HYBRID_P"])
@pytest.mark.parametrize("link", [LinkConfig(), LinkConfig(GE.bursty(0.5, 5), delay=2)])
@pytest.mark.parametrize("travel", [0, 1])
def test_proposition1_nagent(N, name, link, travel):
    """No private events: silence is on-plan, so every protocol equals ORACLE."""
    inst = grid_instance(5, 4, seed=1, travel_scale=travel, horizon=16 if travel == 0 else 40,
                         abort=0, h_good=0, h_bad=0, alpha=0)
    nom = nominal_plan_n(inst, N, S)
    for k in range(25):
        a = run_n(inst, N, name, link, k, S, nom)
        b = run_n(inst, N, "ORACLE", LinkConfig(), k, S, nom)
        assert (a.detected, a.ttd, a.looks) == (b.detected, b.ttd, b.looks)


def test_hybrid_p_differs_from_hybrid_with_events_only():
    inst = grid_instance(5, 5, seed=401, travel_scale=0, horizon=16, **events(3))
    nom = nominal_plan_n(inst, 3, S)
    link = LINKS["d1_loss0"]
    diff = sum(run_n(inst, 3, "HYBRID", link, k, S, nom).collisions !=
               run_n(inst, 3, "HYBRID_P", link, k, S, nom).collisions for k in range(30))
    assert diff > 0
    inst0 = grid_instance(5, 5, seed=401, travel_scale=0, horizon=16, **events(0))
    nom0 = nominal_plan_n(inst0, 3, S)
    for k in range(20):
        a = run_n(inst0, 3, "HYBRID", link, k, S, nom0)
        b = run_n(inst0, 3, "HYBRID_P", link, k, S, nom0)
        assert (a.ttd, a.looks, a.collisions) == (b.ttd, b.looks, b.collisions)


def test_role_rule_only_for_lower_priority_agents():
    """Prediction slots only occur for j < k: agent 0 never predicts, and a
    predicting agent k only asks about agents j < k."""
    inst = grid_instance(5, 5, seed=401, travel_scale=0, horizon=16, **events(3))
    nom = nominal_plan_n(inst, 4, S)
    calls = []
    for k in range(30):
        e = NEpisode(inst, (S["HYBRID_P"],) * 4, LINKS["d1_loss0"], np.random.default_rng(k), nom)
        orig = e._partner_pos_guess
        e._partner_pos_guess = lambda a, j, orig=orig: (calls.append((a.idx, j)), orig(a, j))[1]
        e.run()
    assert calls, "role rule never exercised"
    assert all(j < k for (k, j) in calls)
    assert {k for (k, _) in calls} >= {1, 2}
