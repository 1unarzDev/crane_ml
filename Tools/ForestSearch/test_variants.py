"""Tests for HYBRID / HYBRID_P (REVIEW_fresh.md: these were uncovered)."""
import sys
from pathlib import Path

import numpy as np
import pytest

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

from ForestSearch.event_curve import LINKS, events  # noqa: E402
from ForestSearch.instances import grid_instance  # noqa: E402
from ForestSearch.links import GE, LinkConfig  # noqa: E402
from ForestSearch.sim import make_sectors, nominal_plan  # noqa: E402
from ForestSearch.variants import HEpisode, make_all_strategies, run_any  # noqa: E402

S = make_all_strategies()


@pytest.mark.parametrize("name", ["HYBRID", "HYBRID_P"])
@pytest.mark.parametrize("link", [LinkConfig(), LinkConfig(GE.bursty(0.5, 5), delay=2)])
def test_hybrid_proposition1(name, link):
    inst = grid_instance(4, 3, seed=1, abort=0, h_good=0, h_bad=0, alpha=0)
    for k in range(30):
        a = run_any(inst, (name,) * 2, link, k, S)
        b = run_any(inst, ("ORACLE",) * 2, LinkConfig(), k, S)
        assert (a.detected, a.ttd, a.looks) == (b.detected, b.ttd, b.looks)


def _collisions(inst, name, link, seeds):
    nom, sec = nominal_plan(inst, S["ORACLE"]), make_sectors(inst)
    tot = 0
    for k in seeds:
        ep = HEpisode(inst, (S[name], S[name]), link, np.random.default_rng(k), nom, sec, keep_log=True)
        ep.run()
        by_t = {}
        for (t, ag, c, v, out) in ep.log:
            by_t.setdefault(t, {})[ag] = (c, v)
        tot += sum(1 for d in by_t.values() if len(d) == 2 and d[0] == d[1])
    return tot / len(seeds)


def test_role_prediction_reduces_lockstep_collisions():
    """Mechanism guard: with delay 1 and travel 0, HYBRID collides often;
    HYBRID_P (consistent partner prediction) and IMPLICIT rarely do."""
    inst = grid_instance(5, 5, seed=300, travel_scale=0.0, horizon=16, **events(1.0))
    link = LINKS["d1_loss0"]
    seeds = range(40)
    c_h = _collisions(inst, "HYBRID", link, seeds)
    c_p = _collisions(inst, "HYBRID_P", link, seeds)
    c_i = _collisions(inst, "IMPLICIT", link, seeds)
    assert c_h > 2.0
    assert c_p < 0.5 * c_h
    assert c_i < 0.5


def test_hybrid_p_only_differs_after_partner_deviation():
    """Without private events there is no deviation, so HYBRID_P == HYBRID."""
    inst = grid_instance(5, 5, seed=300, travel_scale=0.0, horizon=16, **events(0.0))
    for k in range(20):
        a = run_any(inst, ("HYBRID",) * 2, LINKS["d1_loss0"], k, S)
        b = run_any(inst, ("HYBRID_P",) * 2, LINKS["d1_loss0"], k, S)
        assert (a.ttd, a.looks) == (b.ttd, b.looks)
