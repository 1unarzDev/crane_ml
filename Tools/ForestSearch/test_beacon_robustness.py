"""Tests for beacon_robustness.py (silent hedges, HB_TO, new fault kinds)."""
import sys
from pathlib import Path

import numpy as np
import pytest

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

from ForestSearch.beacon_robustness import (BEpisode, Ctx, LINKS, builders,  # noqa: E402
                                            detect_prob_batch, fault_for, hb_to,
                                            make_strategies, run_b)
from ForestSearch.falsify import regimes  # noqa: E402
from ForestSearch.loss_baselines import make_strategies as make_l, run_l  # noqa: E402
from ForestSearch.model import detect_prob  # noqa: E402
from ForestSearch.sim import HEADER_B  # noqa: E402

NOOP = {  # hedges whose trigger never fires -> must equal their base strategy
    "TK_never": dict(hedge="takeover", h_f=2.0, h_w=1.0),
    "RED_0": dict(hedge="redundant", h_rho=0.0),
    "BAYES_0": dict(hedge="bayes", h_pi=0.0),
    "SS_never": dict(hedge="secsweep", h_f=2.0),
}
LIVE = {
    "TK": dict(hedge="takeover", h_f=0.5, h_w=1.0),
    "RED": dict(hedge="redundant", h_rho=0.5),
    "BAYES": dict(hedge="bayes", h_pi=1.0),
    "SS": dict(hedge="secsweep", h_f=0.5),
    "HB": hb_to(3),
}
S = make_strategies({**NOOP, **LIVE})
SL = make_l()


def _sig(r):
    return (r.detected, r.ttd, r.looks, r.redundant, r.aborts, r.hints)


_ctx = {}


def ctx(bk, seed=500):
    if (bk, seed) not in _ctx:
        _ctx[(bk, seed)] = Ctx(builders()[bk](seed), S)
    return _ctx[(bk, seed)]


def test_detect_prob_batch_matches_model():
    c = ctx("D")
    rng = np.random.default_rng(0)
    M0 = rng.integers(0, 3, (3, c.inst.n, c.inst.V))
    M1 = rng.integers(0, 2, (3, c.inst.n, c.inst.V))
    pd = detect_prob_batch(c.inst, M0.astype(float), M1.astype(float))
    for k in range(3):
        assert np.allclose(pd[k], detect_prob(c.inst, M0[k], M1[k]))


@pytest.mark.parametrize("name,base", [("TK_never", "IMPLICIT"), ("RED_0", "IMPLICIT"),
                                       ("BAYES_0", "IMPLICIT"), ("SS_never", "SECTOR"),
                                       ("HB", "IMPLICIT")])
@pytest.mark.parametrize("bk", ["A", "D"])
def test_no_loss_no_trigger_equals_base(name, base, bk):
    """No loss and no hedge trigger -> identical episodes to IMPLICIT (SECTOR for SECSWEEP).
    HB_TO on an ideal link never times out, so it also equals IMPLICIT (but pays bytes)."""
    for ep in range(30):
        a = run_b(ctx(bk), name, ep, S, None, "d0_loss0")
        b = run_b(ctx(bk), base, ep, S, None, "d0_loss0")
        assert _sig(a) == _sig(b)
        if name != "HB":
            assert a.bytes_sent == 0


@pytest.mark.parametrize("name", ["TK_never", "RED_0", "BAYES_0"])
def test_noop_hedge_equals_implicit_under_loss(name):
    f = regimes()["R7a"][2]
    for ep in range(30):
        assert _sig(run_b(ctx("A"), name, ep, S, f, "d0_loss0")) == _sig(run_b(ctx("A"), "IMPLICIT", ep, S, f, "d0_loss0"))


@pytest.mark.parametrize("name", list(LIVE) + ["HEALTH", "HEARTBEAT"])
def test_deterministic(name):
    f = fault_for("total", 16) if name == "HB" else regimes()["R7a"][2]
    for ep in range(10):
        r1 = run_b(ctx("A"), name, ep, S, f, "d1_burst30")
        r2 = run_b(ctx("A"), name, ep, S, f, "d1_burst30")
        assert _sig(r1) == _sig(r2) and r1.bytes_sent == r2.bytes_sent


@pytest.mark.parametrize("name", ["HEALTH", "HEARTBEAT", "IMPLICIT", "INDEP", "HYBRID_P"])
@pytest.mark.parametrize("key", ["R7a", "R7d"])
def test_subclass_reproduces_loss_baselines(name, key):
    """BEpisode with falsify's 'loss' faults reproduces loss_baselines.LEpisode exactly."""
    _, build, f = regimes()[key]
    c = ctx("A" if key == "R7a" else "D")
    for ep in range(20):
        for ln in ("d0_loss0", "d1_burst30"):
            a = run_b(c, name, ep, S, f, ln)
            b = run_l(c.inst, name, ep, SL, c.nom, c.sec, f, ln)
            assert _sig(a) == _sig(b) and a.bytes_sent == b.bytes_sent


def test_perm_fault_equals_falsify_loss():
    """'perm' kind draws the same victim and time as falsify's 'loss'."""
    f_old = dict(regimes()["R7a"][2])
    f_new = dict(kind="perm", t_lo=f_old["t_lo"], t_hi=f_old["t_hi"])
    for ep in range(30):
        a = run_b(ctx("A"), "HEALTH", ep, S, f_new, "d0_loss0")
        b = run_b(ctx("A"), "HEALTH", ep, S, f_old, "d0_loss0")
        assert _sig(a) == _sig(b)


def _differs(n1, n2, fault, bk="A", link="d0_loss0", n=80):
    return sum(_sig(run_b(ctx(bk), n1, ep, S, fault, link)) != _sig(run_b(ctx(bk), n2, ep, S, fault, link))
               for ep in range(n))


@pytest.mark.parametrize("name", list(LIVE))
def test_live_hedges_are_not_vacuous(name):
    f = fault_for("total", 16) if name == "HB" else regimes()["R7a"][2]
    base = "SECTOR" if name == "SS" else "IMPLICIT"
    assert _differs(name, base, f) > 0


def test_total_loss_silences_health():
    """Radio+sensor death: HEALTH can send nothing and equals IMPLICIT; bytes = 0."""
    f = fault_for("total", 16)
    for ep in range(40):
        a = run_b(ctx("A"), "HEALTH", ep, S, f, "d0_loss0")
        assert a.bytes_sent == 0
        assert _sig(a) == _sig(run_b(ctx("A"), "IMPLICIT", ep, S, f, "d0_loss0"))


def test_lost_beacon_health_equals_implicit_but_heartbeat_differs():
    f = fault_for("beacon_lost", 16)
    assert _differs("HEALTH", "IMPLICIT", f) == 0
    assert _differs("HEARTBEAT", "IMPLICIT", f) > 0


@pytest.mark.xfail(strict=True, reason="MUST FAIL: under total loss the HB_TO timeout must change behaviour")
def test_hb_timeout_inert_under_total_loss_must_fail():
    assert _differs("HB", "IMPLICIT", fault_for("total", 16)) == 0


@pytest.mark.xfail(strict=True, reason="MUST FAIL: a hedged silent plan is not free on a healthy team")
def test_takeover_free_when_healthy_must_fail():
    assert _differs("TK", "IMPLICIT", None, bk="A") == 0


def test_hb_timeout_false_alarm_and_cancel():
    """On a link that drops everything, HB_TO infers the healthy partner dead (false alarm),
    so it must differ from IMPLICIT even with no fault."""
    from ForestSearch.links import GE, LinkConfig
    LINKS["dead"] = LinkConfig(GE.bernoulli(1.0), name="dead")
    try:
        assert _differs("HB", "IMPLICIT", None, link="dead") > 0
    finally:
        del LINKS["dead"]


def test_fault_kinds_shapes():
    c = ctx("A")
    S0 = (S["IMPLICIT"], S["IMPLICIT"])
    for cond in ("trans_m", "intermit", "total", "perm_late"):
        f = dict(fault_for(cond, c.inst.horizon)); f["seed"] = 10_007
        e = BEpisode(c.inst, S0, LINKS["d0_loss0"], np.random.default_rng(7), c.nom, c.sec,
                     fault=f, sec_nominal=c.snom)
        assert e.bad.sum(axis=1).min() == 0          # exactly one victim
        if cond == "trans_m":
            assert e.bad.sum() == f["L"] and not e.bad[:, -1].any()
        if cond == "total":
            assert (e.radio_dead == e.bad).all()
        if cond == "perm_late":
            assert e.t_dead[int(np.argmax(e.bad.any(1)))] >= f["t_lo"]


def test_health_one_frame_hb_bytes_multiple_of_header():
    f = regimes()["R7a"][2]
    for ep in range(20):
        assert run_b(ctx("A"), "HEALTH", ep, S, f, "d0_loss0").bytes_sent in (0, HEADER_B)
        assert run_b(ctx("A"), "HB", ep, S, f, "d0_loss0").bytes_sent % HEADER_B == 0
