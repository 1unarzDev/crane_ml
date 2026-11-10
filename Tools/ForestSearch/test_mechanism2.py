"""Tests for mechanism2 variants.

Includes tests that MUST FAIL if the variants are wrong: equivalence of
HP_M with HYBRID_P, inertness of every variant when no events occur
(Prop. 1), determinism, and the expected *ordering* of the intervention
effects on a fixed instance (these are the claims the paper rests on).
"""
import sys
from pathlib import Path

import numpy as np
import pytest

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

from ForestSearch.event_curve import LINKS, events  # noqa: E402
from ForestSearch.instances import grid_instance  # noqa: E402
from ForestSearch.links import GE, LinkConfig  # noqa: E402
from ForestSearch.mechanism2 import make_m_strategies, run_one  # noqa: E402
from ForestSearch.sim import make_sectors, nominal_plan  # noqa: E402
from ForestSearch.variants import run_any  # noqa: E402

S = make_m_strategies()
NEW = ["HP_M", "HY_M", "SWAP", "SWAP_F", "SYM", "SYM_F", "RND_A", "RND_T5",
       "DROP25", "DROP50", "DROP75", "DROP50_rx0", "DROP50_rx1", "DROP50_P", "SWAP_V", "SYM_V", "RND_A_V"]


def _setup(lam, seed=300):
    inst = grid_instance(5, 5, seed=seed, travel_scale=0.0, horizon=16, **events(lam))
    return inst, nominal_plan(inst, S["ORACLE"]), make_sectors(inst)


@pytest.mark.parametrize("name", NEW)
@pytest.mark.parametrize("link", [LinkConfig(), LinkConfig(GE.bursty(0.5, 5), delay=2),
                                  LINKS["d1_loss0"]])
def test_prop1_no_events(name, link):
    """No private events => no deviation => nothing fires, nothing is dropped
    (records are all-NULL on-plan), so every variant matches ORACLE/HYBRID."""
    inst = grid_instance(4, 3, seed=1, abort=0, h_good=0, h_bad=0, alpha=0)
    nom, sec = nominal_plan(inst, S["ORACLE"]), make_sectors(inst)
    for k in range(25):
        a = run_one(inst, name, link, k, S, nom, sec)
        o = run_one(inst, "ORACLE", LinkConfig(), k, S, nom, sec)
        assert (a["det"], a["ttd"]) == (o["det"], o["ttd"])


@pytest.mark.parametrize("name", NEW)
def test_inert_without_events_matches_hybrid_exactly(name):
    inst, nom, sec = _setup(0.0)
    for k in range(25):
        a = run_one(inst, name, LINKS["d1_loss0"], k, S, nom, sec)
        b = run_one(inst, "HYBRID", LINKS["d1_loss0"], k, S, nom, sec)
        assert (a["det"], a["ttd"], a["coll"]) == (b["det"], b["ttd"], b["coll"])
        assert a["dropped"] == 0 or name.startswith("DROP")


@pytest.mark.parametrize("name", ["HP_M", "RND_A", "RND_T5", "SYM_F", "DROP50"])
def test_deterministic(name):
    inst, nom, sec = _setup(1.0)
    a = [run_one(inst, name, LINKS["d1_loss0"], k, S, nom, sec) for k in range(20)]
    b = [run_one(inst, name, LINKS["d1_loss0"], k, S, nom, sec) for k in range(20)]
    assert a == b


def test_hpm_equals_hybrid_p_and_hym_equals_hybrid():
    """The generalised re-implementation must reproduce the originals exactly
    (detection, ttd, collisions), otherwise every placebo is uninterpretable."""
    inst, nom, sec = _setup(1.0)
    for k in range(60):
        for new, old in (("HP_M", "HYBRID_P"), ("HY_M", "HYBRID")):
            a = run_one(inst, new, LINKS["d1_loss0"], k, S, nom, sec)
            b = run_one(inst, old, LINKS["d1_loss0"], k, S, nom, sec)
            assert (a["det"], a["ttd"], a["coll"], a["red"]) == (b["det"], b["ttd"], b["coll"], b["red"])


def _mean_coll(name, lam=1.0, n=60, delay_link=None):
    inst, nom, sec = _setup(lam)
    link = delay_link or LINKS["d1_loss0"]
    return np.mean([run_one(inst, name, link, k, S, nom, sec)["coll"] for k in range(n)])


def test_mechanism_ordering_must_hold():
    """MUST FAIL if the stale-plan story is wrong on this instance:
    HYBRID collides a lot; correct-content prediction (HYBRID_P) and the
    in-flight oracle remove most of it; random-content prediction (RND_A)
    does NOT remove it as well as the argmax prediction."""
    c_h = _mean_coll("HYBRID")
    c_p = _mean_coll("HYBRID_P")
    c_o = _mean_coll("PH_orc")
    c_r = _mean_coll("RND_A")
    assert c_h > 2.0
    assert c_p < 0.5 * c_h
    assert c_o < 0.1 * c_h
    assert c_r > 2.0 * c_p


def test_no_delay_no_collisions_must_hold():
    """MUST FAIL if collisions were a delay-independent artefact: with delay 0
    the in-flight slot is known, so HYBRID should barely collide."""
    c0 = _mean_coll("HYBRID", delay_link=LINKS["d0_loss0"])
    c1 = _mean_coll("HYBRID")
    assert c0 < 0.25 * c1


def test_drop_actually_drops():
    inst, nom, sec = _setup(1.0)
    d = [run_one(inst, "DROP75", LINKS["d1_loss0"], k, S, nom, sec)["dropped"] for k in range(30)]
    z = [run_one(inst, "DROP50_P", LINKS["d1_loss0"], k, S, nom, sec)["dropped"] for k in range(30)]
    assert sum(d) > 0 and sum(z) > 0
    h = [run_one(inst, "HYBRID", LINKS["d1_loss0"], k, S, nom, sec)["dropped"] for k in range(30)]
    assert sum(h) == 0


def test_placebo_does_not_perturb_pairing():
    """Auxiliary RNG streams must not alter truth/link draws: the first
    outcome sequence of RND_A under no-event conditions equals HYBRID's."""
    inst, nom, sec = _setup(0.0)
    a = [run_one(inst, "RND_A", LINKS["d1_loss0"], k, S, nom, sec)["ttd"] for k in range(30)]
    b = [run_one(inst, "HYBRID", LINKS["d1_loss0"], k, S, nom, sec)["ttd"] for k in range(30)]
    assert a == b
