"""Tests for deviation_comm.py (event-triggered prediction-deviation baseline)."""
import sys
from dataclasses import replace
from pathlib import Path

import numpy as np
import pytest

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

from ForestSearch.beacon_robustness import Ctx, LINKS, fault_for, run_b  # noqa: E402
from ForestSearch.deviation_comm import (FRAME_B, CommonGround, DEpisode,  # noqa: E402
                                         make_strategies, run_d)
from ForestSearch.event_curve import events  # noqa: E402
from ForestSearch.instances import grid_instance  # noqa: E402
from ForestSearch.loss_baselines import run_l  # noqa: E402

S = make_strategies()
DEVS = ["DEV", "DEV_TASK", "DEV_INFO", "DEV_SD", "DEV_B24", "DEV_SD_B12"]
_ctx = {}


def ctx(lam, tr, seed=200, g=5):
    k = (lam, tr, seed, g)
    if k not in _ctx:
        hz = 16 if tr == 0 else 2 * g * g
        _ctx[k] = Ctx(grid_instance(g, g, seed=seed, travel_scale=tr, horizon=hz, **events(lam)), S)
    return _ctx[k]


def sig(r):
    return (r.detected, r.ttd, r.looks, r.redundant, r.aborts, r.hints)


@pytest.mark.parametrize("tr,seed,g", [(0, 200, 5), (1, 201, 5), (1, 203, 6), (0, 205, 6)])
def test_cg_without_messages_is_nominal_plan(tr, seed, g):
    c = ctx(0, tr, seed, g)
    cg = CommonGround(c.inst)
    bm = S["IMPLICIT"].belief
    for t in range(c.inst.horizon):
        for i in (0, 1):
            cg.advance(i, t, bm)
    for i in (0, 1):
        got = [(k.seq, k.cell, k.view, k.start, k.finish) for k in sorted(cg.tasks[i].values(), key=lambda x: x.seq)]
        want = [(k.seq, k.cell, k.view, k.start, k.finish) for k in c.nom[i]]
        assert got[:len(want)] == want


@pytest.mark.parametrize("name", DEVS)
@pytest.mark.parametrize("tr", [0, 1])
@pytest.mark.parametrize("link", ["d0_loss0", "d1_loss0", "d1_burst30"])
def test_prop1_no_events_zero_bytes_equals_implicit(name, tr, link):
    """Prop 1 equivalence: no private events => no deviation => 0 bytes and == IMPLICIT."""
    c = ctx(0, tr)
    for ep in range(25):
        r = run_d(c, name, ep, S, None, link)
        q = run_b(c, "IMPLICIT", ep, S, None, link)
        assert r.bytes_sent == 0 and r.frame_log == []
        assert sig(r) == sig(q)


@pytest.mark.parametrize("tr", [0, 1])
def test_zero_budget_equals_implicit_with_events(tr):
    c = ctx(3, tr)
    for ep in range(25):
        assert sig(run_d(c, "DEV_B0", ep, S, None, "d0_loss0")) == sig(run_b(c, "IMPLICIT", ep, S, None, "d0_loss0"))


@pytest.mark.parametrize("name", ["DEV", "DEV_TASK", "DEV_INFO", "DEV_SD_B24"])
def test_deterministic(name):
    c = ctx(3, 1)
    f = fault_for("perm", c.inst.horizon)
    for ep in range(10):
        a = run_d(c, name, ep, S, f, "d1_burst30")
        b = run_d(c, name, ep, S, f, "d1_burst30")
        assert sig(a) == sig(b) and a.bytes_sent == b.bytes_sent and a.frame_log == b.frame_log


@pytest.mark.parametrize("link", ["d0_loss0", "d1_loss0", "d3_loss0"])
@pytest.mark.parametrize("name", ["DEV", "DEV_TASK", "DEV_INFO"])
def test_cg_copies_agree_on_lossless_links(link, name):
    c = ctx(3, 1)
    for ep in range(15):
        e = DEpisode(c.inst, (S[name], S[name]), LINKS[link], np.random.default_rng(ep), c.nom, c.sec,
                     fault={"seed": 10_000 + ep}, sec_nominal=c.snom)
        e.run()
        assert e.cg[0].copy_state() == e.cg[1].copy_state()


def test_events_do_trigger_messages():
    c = ctx(3, 1)
    assert sum(run_d(c, "DEV", ep, S, None, "d0_loss0").bytes_sent for ep in range(20)) > 0


@pytest.mark.parametrize("name,B", [("DEV_B24", 24), ("DEV_SD_B12", 12)])
def test_budget_respected(name, B):
    c = ctx(3, 1)
    f = fault_for("perm", c.inst.horizon)
    for ep in range(20):
        r = run_d(c, name, ep, S, f, "d0_loss0")
        assert r.bytes_sent % FRAME_B == 0
        for i in (0, 1):
            assert FRAME_B * sum(nf for (_, j, nf, _) in r.frame_log if j == i) <= B


def test_dev_sd_equals_dev_on_healthy_team():
    c = ctx(3, 1)
    for ep in range(20):
        a = run_d(c, "DEV_SD", ep, S, None, "d1_burst30")
        b = run_d(c, "DEV", ep, S, None, "d1_burst30")
        assert sig(a) == sig(b) and a.bytes_sent == b.bytes_sent


def test_dev_sd_is_health_without_other_events():
    """lam = 0, travel 0, permanent loss: the self-diagnosing deviation agent sends exactly one
    12 B 'idle' frame at the death step and reproduces HEALTH episode for episode."""
    c = ctx(0, 0)
    f = fault_for("perm", c.inst.horizon)
    Sl = S
    n1 = 0
    for ep in range(40):
        r = run_d(c, "DEV_SD", ep, S, f, "d0_loss0")
        h = run_b(c, "HEALTH", ep, Sl, f, "d0_loss0")
        assert sig(r) == sig(h)
        td = [x for x in r.t_dead if x is not None][0]
        # HEALTH transmits at the start of the death step, DEV_SD at that step's decision
        # phase, which is never reached if a detection ends the episode in that step.
        assert r.bytes_sent == h.bytes_sent or (r.ttd == td and r.bytes_sent == 0 and h.bytes_sent == FRAME_B)
        if r.frame_log:
            n1 += 1
            assert len(r.frame_log) == 1 and r.frame_log[0][0] == [x for x in r.t_dead if x is not None][0]
    assert n1 > 0


def test_total_loss_lost_agent_silent_after_death():
    c = ctx(3, 0)
    f = fault_for("total", c.inst.horizon)
    for ep in range(30):
        r = run_d(c, "DEV", ep, S, f, "d0_loss0")
        td = [x for x in r.t_dead if x is not None][0]
        who = r.t_dead.index(td)
        assert not any(i == who and t >= td for (t, i, _, _) in r.frame_log)


# --------------------------------------------------------------- must fail
@pytest.mark.xfail(strict=True, reason="MUST FAIL: without self-diagnosis a dead agent streams abort deviations, "
                                       "so naive DEV is not the beacon")
def test_naive_dev_is_health_must_fail():
    c = ctx(0, 0)
    f = fault_for("perm", c.inst.horizon)
    for ep in range(40):
        r = run_d(c, "DEV", ep, S, f, "d0_loss0")
        h = run_b(c, "HEALTH", ep, S, f, "d0_loss0")
        assert sig(r) == sig(h) and r.bytes_sent == h.bytes_sent


@pytest.mark.xfail(strict=True, reason="MUST FAIL: with private events DEV messages change behaviour on a delayed link")
def test_dev_inert_under_delay_must_fail():
    c = ctx(3, 0)
    for ep in range(40):
        assert sig(run_d(c, "DEV", ep, S, None, "d1_loss0")) == sig(run_b(c, "IMPLICIT", ep, S, None, "d1_loss0"))


@pytest.mark.xfail(strict=True, reason="MUST FAIL: a transient outage needs a second (recovery) frame")
def test_dev_sd_one_frame_under_transient_must_fail():
    c = ctx(0, 0)
    f = fault_for("trans_m", c.inst.horizon)
    for ep in range(40):
        assert len(run_d(c, "DEV_SD", ep, S, f, "d0_loss0").frame_log) <= 1


@pytest.mark.parametrize("bk,key", [("A", "R7a"), ("B", "R7b")])
def test_dev_sd_reproduces_oracle_on_ideal_link(bk, key):
    """Observed identity (fresh seeds 800-811: 4800/4800 episodes): on an ideal link the
    self-diagnosing deviation protocol reproduces the centralised greedy ORACLE exactly."""
    from ForestSearch.beacon_robustness import builders, r7_fault
    c = Ctx(builders()[bk](503), S)
    f = r7_fault(key)
    for ep in range(25):
        assert sig(run_d(c, "DEV_SD", ep, S, f, "d0_loss0")) == sig(run_b(c, "ORACLE", ep, S, f, "d0_loss0"))
