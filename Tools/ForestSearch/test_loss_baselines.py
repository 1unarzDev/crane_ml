"""Tests for HEALTH / HEARTBEAT (loss_baselines.py)."""
import sys
from pathlib import Path

import numpy as np
import pytest

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

from ForestSearch.event_curve import LINKS  # noqa: E402
from ForestSearch.falsify import regimes  # noqa: E402
from ForestSearch.loss_baselines import make_strategies, run_l  # noqa: E402
from ForestSearch.sim import HEADER_B, make_sectors, nominal_plan  # noqa: E402

S = make_strategies()


def _setup(key, seed=504):
    _, build, fault = regimes()[key]
    inst = build(seed)
    return inst, nominal_plan(inst, S["ORACLE"]), make_sectors(inst), fault


def _sig(r):
    return (r.detected, r.ttd, r.looks, r.redundant, r.aborts, r.hints)


@pytest.mark.parametrize("name", ["HEALTH", "HEARTBEAT"])
@pytest.mark.parametrize("link", list(LINKS))
@pytest.mark.parametrize("key", ["R0", "R4a"])
def test_health_equals_implicit_without_loss(name, link, key):
    inst, nom, sec, fault = _setup(key)
    assert fault is None
    for ep in range(25):
        a = run_l(inst, name, ep, S, nom, sec, fault, link)
        b = run_l(inst, "IMPLICIT", ep, S, nom, sec, fault, link)
        assert _sig(a) == _sig(b)
        if name == "HEALTH":
            assert a.bytes_sent == 0           # no failure -> no beacon


def test_beacon_costs_exactly_one_frame_on_loss():
    inst, nom, sec, fault = _setup("R7a")
    for ep in range(25):
        r = run_l(inst, "HEALTH", ep, S, nom, sec, fault, "d0_loss0")
        assert r.bytes_sent in (0, HEADER_B)    # 0 only if detected before death


def _differs(name_a, name_b, key, link="d0_loss0", n=80):
    inst, nom, sec, fault = _setup(key)
    return sum(_sig(run_l(inst, name_a, ep, S, nom, sec, fault, link)) !=
               _sig(run_l(inst, name_b, ep, S, nom, sec, fault, link)) for ep in range(n))


def test_health_differs_from_implicit_under_loss():
    """Non-vacuity: the equality test above can fail; under loss the beacon must matter."""
    assert _differs("HEALTH", "IMPLICIT", "R7c") > 0


@pytest.mark.xfail(strict=True, reason="MUST FAIL: HEALTH is not IMPLICIT once a sensor dies")
def test_health_equals_implicit_under_loss_must_fail():
    assert _differs("HEALTH", "IMPLICIT", "R7c") == 0


def test_dead_link_beacon_is_inert():
    """With a link that drops everything, HEALTH must equal IMPLICIT even under loss."""
    from ForestSearch import event_curve
    from ForestSearch.links import GE, LinkConfig
    event_curve.LINKS["dead"] = LinkConfig(GE.bernoulli(1.0), name="dead")
    try:
        assert _differs("HEALTH", "IMPLICIT", "R7c", link="dead") == 0
    finally:
        del event_curve.LINKS["dead"]
