"""Tests for predictor.py (DRD / coordination regret)."""
import sys
from pathlib import Path

import numpy as np
import pytest

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

from ForestSearch.exact_dec import TinyModel, base_instance  # noqa: E402
from ForestSearch.predictor import ExactCR, _inst, run_probe  # noqa: E402


@pytest.mark.parametrize("hints,ab", [("none", 0.0), ("weak", 0.0), ("none", 0.2), ("strong", 0.2)])
def test_performance_difference_identity_and_bound(hints, ab):
    M = TinyModel(base_instance(2, hints, ab))
    E = ExactCR(M, 2)
    vd, p0, p1, _ = M.v_dec(2, True)
    voc = E.vc - vd
    for pols in (E.phi_policies(), E.tree_policies(p0, p1)):
        r = E.evaluate(pols)
        assert abs(r["pd_identity"]) < 1e-12           # V_cent - V(pi) = sum CR exactly
        assert voc <= r["cr"].sum() + 1e-12           # VoC <= E sum CR
        assert (r["cr"] >= -1e-14).all()
    r = E.evaluate(E.tree_policies(p0, p1))
    assert abs(r["cr"].sum() - voc) < 1e-12          # equality at an optimal decentralised pair


def test_no_private_events_drd_zero():
    M = TinyModel(base_instance(2, "none", 0.0))
    for k in (2, 3):
        E = ExactCR(M, k)
        r = E.evaluate(E.phi_policies())
        assert r["drd"].sum() == 0 and r["drdphi"].sum() == 0 and abs(r["cr"].sum()) < 1e-14


@pytest.mark.parametrize("hints,ab", [("weak", 0.0), ("strong", 0.2), ("none", 0.2)])
def test_cr_supported_after_private_events(hints, ab):
    M = TinyModel(base_instance(2, hints, ab))
    E = ExactCR(M, 2)
    r = E.evaluate(E.phi_policies())
    assert abs(r["split"]["no_event"]) < 1e-14


def test_beacon_cr_zero_and_no_frame_cr_after_death():
    M = TinyModel(base_instance(2, "none", 0.0))
    E = ExactCR(M, 2, death=0.2)
    rb = E.evaluate(E.phi_policies(True), frames=True)
    assert abs(rb["cr"].sum()) < 1e-14 and abs(rb["V"] - E.vc) < 1e-12
    rn = E.evaluate(E.phi_policies(False), frames=False)
    assert rn["cr"].sum() > 1e-4
    assert abs(rn["split"]["no_event"]) < 1e-14
    assert abs(rn["cr"].sum() - rn["split"]["dead_unknown"]) < 1e-14


def test_probe_does_not_change_trajectory():
    from ForestSearch.beacon_robustness import Ctx, fault_for, run_b
    from ForestSearch.deviation_comm import make_strategies
    S = make_strategies()
    ctx = Ctx(_inst("A", 1000, 1.0), S)
    for cond in ("healthy", "perm"):
        f = fault_for(cond, ctx.inst.horizon)
        for ep in range(8):
            r0 = run_b(ctx, "IMPLICIT", ep, S, f, "d0_loss0")
            r1, pr, _ = run_probe(ctx, ep, S, f, "d0_loss0", 6)
            assert (r0.detected, r0.ttd, r0.looks, r0.hints, r0.aborts) == \
                   (r1.detected, r1.ttd, r1.looks, r1.hints, r1.aborts)
            assert pr["cr"] >= 0


def test_probe_zero_without_events():
    from ForestSearch.beacon_robustness import Ctx
    from ForestSearch.deviation_comm import make_strategies
    S = make_strategies()
    ctx = Ctx(_inst("A", 1001, 0.0), S)
    for ep in range(5):
        _, pr, _ = run_probe(ctx, ep, S, None, "d0_loss0", 6)
        assert pr["drd"] == 0 and pr["cr"] == 0.0
