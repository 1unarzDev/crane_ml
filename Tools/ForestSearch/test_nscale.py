"""Tests for nscale.py.

  * N = 2 equivalence with the 2-agent code for every protocol, healthy
    (variants.run_any) and under permanent loss (beacon_robustness.BEpisode,
    which contains falsify.FEpisode + loss_baselines.LEpisode), incl. HEALTH,
    HB_TO, radio-dead 'total' loss, Bernoulli / GE / relay links, travel and
    the rollout-lite planner.
  * N = 3, 4: equals nagent.run_n (subclass did not change healthy semantics).
  * Proposition 1 at N = 3, 5, 10: no private events => IMPLICIT = ORACLE, 0 bytes.
  * determinism; fault / beacon accounting.
  * MUST-FAIL (strict xfail): mutants and negative controls that the suite
    has to be able to tell apart.
"""
import sys
from pathlib import Path

import numpy as np
import pytest

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

from ForestSearch import nscale  # noqa: E402
from ForestSearch.beacon_robustness import BEpisode, hb_to  # noqa: E402
from ForestSearch.beacon_robustness import make_strategies as make_b  # noqa: E402
from ForestSearch.event_curve import events  # noqa: E402
from ForestSearch.instances import grid_instance  # noqa: E402
from ForestSearch.links import GE, LinkConfig  # noqa: E402
from ForestSearch.nagent import nominal_plan_n, run_n  # noqa: E402
from ForestSearch.nscale import (LINKS, NSEpisode, make_strategies, nominal_plan_ns,  # noqa: E402
                                 run_ns, starts_n)
from ForestSearch.sim import make_sectors, nominal_plan  # noqa: E402
from ForestSearch.variants import make_all_strategies, run_any  # noqa: E402

S = make_strategies("greedy")
S2 = make_b({"HB_TO": hb_to(nscale.HB_P, nscale.HB_K)})
FIELDS = ("detected", "ttd", "looks", "redundant", "aborts", "hints",
          "bytes_sent", "msgs_sent", "msgs_delivered")
LK = [LINKS["ideal"], LINKS["d1"], LINKS["burst30_d1"], LINKS["bern30_d1"], LINKS["relay_d1"]]
BASE5 = ["ORACLE", "INDEP", "IMPLICIT", "HYBRID", "HYBRID_P"]


def _key(r):
    return tuple(getattr(r, f) for f in FIELDS)


def _inst(travel, lam=3, w=5, seed=401):
    return grid_instance(w, w, seed=seed, travel_scale=travel,
                         horizon=16 if travel == 0 else 30, **events(lam))


def test_nominal_plan_matches_n2_and_nagent():
    inst = _inst(1)
    a = nominal_plan(inst, S["ORACLE"])
    b = nominal_plan_ns(inst, 2, S)
    c = nominal_plan_n(inst, 3, S)
    d = nominal_plan_ns(inst, 3, S)
    tup = lambda P: [[(t.seq, t.cell, t.view, t.start, t.finish) for t in p] for p in P]
    assert tup(a) == tup(b)
    assert tup(c) == tup(d)


@pytest.mark.parametrize("name", BASE5)
@pytest.mark.parametrize("li", range(len(LK)))
@pytest.mark.parametrize("travel", [0, 1])
def test_n2_healthy_equals_run_any(name, li, travel):
    inst = _inst(travel)
    nom = nominal_plan(inst, S["ORACLE"])
    nn = nominal_plan_ns(inst, 2, S)
    for relay in (False, True):
        for k in range(15):
            a = run_any(inst, (name,) * 2, LK[li], k, S, nom)
            b = run_ns(inst, 2, name, LK[li], k, S, nn, relay_shared=relay)
            assert _key(a) == _key(b), (name, k, relay)


@pytest.mark.parametrize("name", BASE5 + ["HEALTH", "HB_TO"])
@pytest.mark.parametrize("li", [0, 1, 2, 4])
@pytest.mark.parametrize("travel", [0, 1])
@pytest.mark.parametrize("total", [False, True])
def test_n2_perm_loss_equals_bepisode(name, li, travel, total):
    inst = _inst(travel, lam=1)
    H = inst.horizon
    nom = nominal_plan(inst, S["ORACLE"]); sec = make_sectors(inst)
    nn = nominal_plan_ns(inst, 2, S)
    for k in range(15):
        f = dict(kind="perm", t_lo=1, t_hi=H // 2, seed=10_000 + k, total=total)
        a = BEpisode(inst, (S2[name],) * 2, LK[li], np.random.default_rng(k), nom, sec,
                     fault=f).run()
        g = dict(kind="perm", k=1, t_lo=1, t_hi=H // 2, total=total)
        b = run_ns(inst, 2, name, LK[li], k, S, nn, fault=g)
        assert _key(a) == _key(b), (name, k)


@pytest.mark.parametrize("name", ["ORACLE", "IMPLICIT", "HYBRID_P"])
def test_n2_rollout_equals_run_any(name):
    Sr = make_all_strategies(rollout_k=3, rollout_budget=8)
    Sn = make_strategies("rollout")
    inst = _inst(1)
    nom = nominal_plan(inst, Sr["ORACLE"])
    nn = nominal_plan_ns(inst, 2, Sn)
    for k in range(10):
        a = run_any(inst, (name,) * 2, LINKS["d1"], k, Sr, nom)
        b = run_ns(inst, 2, name, LINKS["d1"], k, Sn, nn)
        assert _key(a) == _key(b)


@pytest.mark.parametrize("N", [3, 4])
@pytest.mark.parametrize("name", BASE5)
def test_n34_equals_nagent(N, name):
    inst = _inst(1)
    nom = nominal_plan_n(inst, N, S)
    for k in range(12):
        a = run_n(inst, N, name, LINKS["burst30_d1"], k, S, nom)
        b = run_ns(inst, N, name, LINKS["burst30_d1"], k, S, nom)
        assert _key(a) + (a.collisions,) == _key(b) + (b.collisions,)


@pytest.mark.parametrize("N", [3, 5, 10])
@pytest.mark.parametrize("travel", [0, 1])
@pytest.mark.parametrize("ln", ["ideal", "burst30_d1"])
def test_proposition1(N, travel, ln):
    w, h = nscale.GRID[N]
    inst = grid_instance(w, h, seed=7, travel_scale=travel, horizon=12 if travel == 0 else 20,
                         abort=0, h_good=0, h_bad=0, alpha=0)
    nom = nominal_plan_ns(inst, N, S)
    for k in range(6):
        o = run_ns(inst, N, "ORACLE", LINKS["ideal"], k, S, nom)
        for name in ("IMPLICIT", "HYBRID", "HYBRID_P", "HEALTH", "HB_TO"):
            if name == "HB_TO" and ln != "ideal":
                continue   # HB_TO can raise false death alarms on lossy links (see below)
            r = run_ns(inst, N, name, LINKS[ln], k, S, nom)
            assert (r.detected, r.ttd, r.looks) == (o.detected, o.ttd, o.looks), name
        assert run_ns(inst, N, "IMPLICIT", LINKS[ln], k, S, nom).bytes_sent == 0
        assert run_ns(inst, N, "HEALTH", LINKS[ln], k, S, nom).bytes_sent == 0


def test_hb_to_false_alarms_on_bursty_link():
    """Documented behaviour, not Prop 1: on a bursty link silence > K steps makes HB_TO
    infer a death that did not happen, so it can deviate from ORACLE without events."""
    w, h = nscale.GRID[10]
    inst = grid_instance(w, h, seed=7, travel_scale=0, horizon=12, abort=0, h_good=0, h_bad=0, alpha=0)
    nom = nominal_plan_ns(inst, 10, S)
    false = 0
    for k in range(6):
        e = NSEpisode(inst, (S["HB_TO"],) * 10, LINKS["burst30_d1"], np.random.default_rng(k), nom)
        e.run()
        false += sum(1 for j in range(10) for i in range(10) for iv in e.intervals[j][i] if iv[2])
    assert false > 0


def test_starts():
    for N, (w, h) in nscale.GRID.items():
        inst = grid_instance(w, h, seed=1)
        st = starts_n(inst, N)
        assert len(set(st)) == N
    inst = grid_instance(6, 6, seed=1)
    from ForestSearch.nagent import starts_for
    for N in (2, 3, 4):
        assert starts_n(inst, N) == starts_for(inst, N)


@pytest.mark.parametrize("N", [5, 10])
def test_determinism(N):
    inst = nscale.build(N, 1000, 3, 1, horizon=20)
    nom = nominal_plan_ns(inst, N, S)
    f = nscale.fault_spec("permk", 20, N)
    for name in ("HYBRID_P", "HB_TO", "HEALTH"):
        a = run_ns(inst, N, name, LINKS["burst30_d1"], 3, S, nom, fault=f)
        b = run_ns(inst, N, name, LINKS["burst30_d1"], 3, S, nominal_plan_ns(inst, N, S), fault=f)
        assert _key(a) + (a.collisions,) == _key(b) + (b.collisions,)


@pytest.mark.parametrize("N", [3, 5, 10])
def test_fault_and_beacon_accounting(N):
    inst = nscale.build(N, 1001, 1, 1, horizon=24)
    nom = nominal_plan_ns(inst, N, S)
    for kname in ("perm1", "permk"):
        f = nscale.fault_spec(kname, 24, N)
        k = f["k"]
        for ep in range(5):
            e = NSEpisode(inst, (S["HEALTH"],) * N, LINKS["ideal"], np.random.default_rng(ep),
                          nom, fault={**f, "seed": 10_000 + ep})
            assert int(e.bad.any(1).sum()) == k
            r = e.run()
            dead_before = sum(1 for td in e.t_dead if td is not None and td <= r.ttd)
            assert r.bytes_sent == 12 * (N - 1) * dead_before
            # every receiver learned every death that was broadcast
            for j in range(N):
                for i, td in enumerate(e.t_dead):
                    if td is not None and td <= r.ttd and i != j:
                        assert e.p_dead[j].get(i) == td


def test_healthy_health_equals_implicit():
    inst = nscale.build(5, 1002, 3, 1, horizon=24)
    nom = nominal_plan_ns(inst, 5, S)
    for ep in range(8):
        a = run_ns(inst, 5, "HEALTH", LINKS["burst30_d1"], ep, S, nom)
        b = run_ns(inst, 5, "IMPLICIT", LINKS["ideal"], ep, S, nom)
        assert _key(a) == _key(b)


def test_relay_shared_uplink_correlates_receivers():
    """With a shared uplink, a broadcast is lost by all receivers or none at the uplink."""
    inst = nscale.build(5, 1003, 1, 1, horizon=24)
    e = NSEpisode(inst, (S["HB_TO"],) * 5, LINKS["relay_d1"], np.random.default_rng(0), None,
                  relay_shared=True)
    for i in range(5):
        ups = {id(e.links[(i, j)].up) for j in range(5) if j != i}
        assert len(ups) == 1


def test_stats_helpers():
    assert abs(nscale.t_cdf(2.131, 15) - 0.975) < 1e-3
    assert abs(nscale.t_ppf(0.95, 15) - 1.753) < 1e-3
    adj = nscale.holm([0.01, 0.04, 0.03, 0.5])
    assert np.allclose(adj, [0.04, 0.09, 0.09, 0.5])
    b, c = nscale.power_fit([2, 3, 5, 10], [0.1 / n for n in (2, 3, 5, 10)])
    assert abs(b - 1) < 0.02 and abs(c - 0.1) < 0.005


# ------------------------------------------------------------- MUST FAIL
class _OracleNotTold(NSEpisode):
    """Mutant: ORACLE is not told that a partner died (keeps its look virtual)."""
    def _partner_counts(self, a, t):
        if self.oracle:
            saved, self.forced = self.forced, set()
            try:
                return super()._partner_counts(a, t)
            finally:
                self.forced = saved
        return super()._partner_counts(a, t)


@pytest.mark.xfail(strict=True, reason="mutant must differ from the 2-agent ORACLE under loss")
def test_mustfail_oracle_mutant_equals_bepisode():
    inst = _inst(0, lam=1)
    nom = nominal_plan(inst, S["ORACLE"]); sec = make_sectors(inst)
    for k in range(40):
        f = dict(kind="perm", t_lo=1, t_hi=8, seed=10_000 + k)
        a = BEpisode(inst, (S2["ORACLE"],) * 2, LINKS["ideal"], np.random.default_rng(k), nom,
                     sec, fault=f).run()
        b = _OracleNotTold(inst, (S["ORACLE"],) * 2, LINKS["ideal"], np.random.default_rng(k),
                           nom, fault=dict(kind="perm", k=1, t_lo=1, t_hi=8, seed=10_000 + k)).run()
        assert _key(a) == _key(b)


@pytest.mark.xfail(strict=True, reason="Prop 1 needs no events: with events IMPLICIT != ORACLE")
def test_mustfail_proposition1_with_events():
    inst = nscale.build(5, 1004, 3, 0, horizon=16)
    nom = nominal_plan_ns(inst, 5, S)
    for k in range(20):
        a = run_ns(inst, 5, "IMPLICIT", LINKS["ideal"], k, S, nom)
        b = run_ns(inst, 5, "ORACLE", LINKS["ideal"], k, S, nom)
        assert (a.detected, a.ttd, a.looks) == (b.detected, b.ttd, b.looks)


@pytest.mark.xfail(strict=True, reason="HEALTH under loss must differ from IMPLICIT")
def test_mustfail_health_noop_under_loss():
    inst = nscale.build(5, 1005, 1, 1, horizon=24)
    nom = nominal_plan_ns(inst, 5, S)
    f = nscale.fault_spec("perm1", 24, 5)
    for k in range(30):
        a = run_ns(inst, 5, "HEALTH", LINKS["ideal"], k, S, nom, fault=f)
        b = run_ns(inst, 5, "IMPLICIT", LINKS["ideal"], k, S, nom, fault=f)
        assert (a.detected, a.ttd, a.looks) == (b.detected, b.ttd, b.looks)


@pytest.mark.xfail(strict=True, reason="independent relay hops are not the shared-uplink model at N=3")
def test_mustfail_relay_shared_equals_independent_n3():
    inst = nscale.build(3, 1006, 3, 0, horizon=16)
    nom = nominal_plan_ns(inst, 3, S)
    for k in range(30):
        a = run_ns(inst, 3, "HB_TO", LINKS["relay_d1"], k, S, nom, relay_shared=True)
        b = run_ns(inst, 3, "HB_TO", LINKS["relay_d1"], k, S, nom, relay_shared=False)
        assert a.msgs_delivered == b.msgs_delivered
