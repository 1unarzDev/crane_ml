"""Tests for nbeacon.py (EXPLORATORY coordinated health beacon).

  * N = 2 equivalence: HEALTH_PART (every rule) and HEALTH_ONE (every rule) == HEALTH, under
    permanent loss, on ideal / d1 / bursty links, travel 0 and 1.
  * No events (healthy) => every beacon protocol == IMPLICIT, 0 bytes, N = 3, 5, 10.
  * determinism; assignment is a partition of the orphans, receiver independent; no extra bytes.
  * MUST-FAIL (strict xfail): negative controls the suite has to be able to tell apart.
"""
import sys
from pathlib import Path

import numpy as np
import pytest

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

from ForestSearch import nbeacon  # noqa: E402
from ForestSearch.nbeacon import (NBEpisode, assign_orphans, diag_metrics, run_nb)  # noqa: E402
from ForestSearch.nscale import LINKS, build, fault_spec, make_strategies, nominal_plan_ns, starts_n  # noqa: E402

S = make_strategies("greedy")
FIELDS = ("detected", "ttd", "looks", "redundant", "aborts", "hints", "bytes_sent",
          "msgs_sent", "msgs_delivered", "collisions")
VARIANTS = [f"HEALTH_PART:{r}" for r in nbeacon.PART_RULES] + [f"HEALTH_ONE:{r}" for r in nbeacon.ONE_RULES]


def key(r):
    return tuple(getattr(r, f) for f in FIELDS)


def setup(N, travel, seed=1000):
    inst = build(N, seed, 1, travel, nbeacon.HZ[(N, travel)])
    return inst, nominal_plan_ns(inst, N, S)


@pytest.mark.parametrize("travel", [0, 1])
@pytest.mark.parametrize("link", ["ideal", "d1", "burst30_d1"])
def test_n2_equivalence_with_health(travel, link):
    inst, nom = setup(2, travel)
    f = fault_spec("perm1", inst.horizon, 2)
    for v in VARIANTS:
        for ep in range(25):
            a = run_nb(inst, 2, "HEALTH", LINKS[link], ep, S, nom, f)
            b = run_nb(inst, 2, v, LINKS[link], ep, S, nom, f)
            assert key(a) == key(b), (v, ep)


@pytest.mark.parametrize("N", [3, 5, 10])
@pytest.mark.parametrize("travel", [0, 1])
def test_no_events_equals_implicit(N, travel):
    inst, nom = setup(N, travel, 1001)
    for v in ["HEALTH"] + VARIANTS:
        for ep in range(6):
            a = run_nb(inst, N, "IMPLICIT", LINKS["ideal"], ep, S, nom, None)
            b = run_nb(inst, N, v, LINKS["burst30_d1"], ep, S, nom, None)
            assert (a.detected, a.ttd, a.looks, a.redundant, a.aborts, a.hints) == \
                   (b.detected, b.ttd, b.looks, b.redundant, b.aborts, b.hints)
            assert b.bytes_sent == 0 and b.msgs_sent == 0


def test_plain_health_matches_nscale():
    from ForestSearch.nscale import run_ns
    inst, nom = setup(5, 1)
    f = fault_spec("perm1", inst.horizon, 5)
    for ep in range(15):
        a = run_nb(inst, 5, "HEALTH", LINKS["d1"], ep, S, nom, f)
        b = run_ns(inst, 5, "HEALTH", LINKS["d1"], ep, S, nom, f)
        assert key(a)[:9] == key(b)[:9]


def test_determinism():
    inst, nom = setup(5, 1, 1002)
    f = fault_spec("perm1", inst.horizon, 5)
    for v in VARIANTS:
        a = [key(run_nb(inst, 5, v, LINKS["burst30_d1"], ep, S, nom, f)) for ep in range(8)]
        b = [key(run_nb(inst, 5, v, LINKS["burst30_d1"], ep, S, nom, f)) for ep in range(8)]
        assert a == b


def test_no_extra_messages():
    inst, nom = setup(5, 1, 1003)
    f = fault_spec("perm1", inst.horizon, 5)
    for ep in range(10):
        h = run_nb(inst, 5, "HEALTH", LINKS["d1"], ep, S, nom, f)
        for v in VARIANTS:
            r = run_nb(inst, 5, v, LINKS["d1"], ep, S, nom, f)
            assert (r.bytes_sent, r.msgs_sent, r.msgs_delivered) == (h.bytes_sent, h.msgs_sent, h.msgs_delivered)


@pytest.mark.parametrize("N", [3, 5, 10])
def test_assignment_is_partition(N):
    inst, nom = setup(N, 1, 1004)
    st = starts_n(inst, N)
    for dead in range(N):
        for td in (1, 5, 9):
            surv = [s for s in range(N) if s != dead]
            orph = {tk.seq for tk in nom[dead] if tk.start >= td}
            for variant, rules in (("part", nbeacon.PART_RULES), ("one", nbeacon.ONE_RULES)):
                for rule in rules:
                    m = assign_orphans(inst, nom, st, dead, td, surv, variant, rule)
                    assert set(m) == orph                      # every orphan assigned exactly once
                    assert set(m.values()) <= set(surv)        # never to the dead agent
                    if variant == "one":
                        assert len(set(m.values())) <= 1
                    if rule == "nearbal" and orph:
                        cap = -(-len(orph) // len(surv))
                        assert max(list(np.bincount(list(m.values()), minlength=N))) <= cap
                    # deterministic: a second call is identical (common knowledge)
                    assert m == assign_orphans(inst, nom, st, dead, td, surv, variant, rule)


def test_variants_change_behaviour_at_n5():
    inst, nom = setup(5, 1, 1005)
    f = fault_spec("perm1", inst.horizon, 5)
    diff = 0
    for ep in range(30):
        h = key(run_nb(inst, 5, "HEALTH", LINKS["ideal"], ep, S, nom, f))
        diff += any(key(run_nb(inst, 5, v, LINKS["ideal"], ep, S, nom, f)) != h for v in VARIANTS)
    assert diff > 0


def test_diag_metrics_toy():
    class T:
        def __init__(self, seq, cell, view, start):
            self.seq, self.cell, self.view, self.start, self.finish = seq, cell, view, start, start + 1

    class E:
        N, t_dead = 3, [1, None, None]
        decisions = [(0, 0, 9, 0), (0, 1, 1, 0), (0, 2, 2, 0),
                     (1, 1, 5, 0), (1, 2, 5, 0), (2, 1, 9, 0), (2, 2, 7, 0)]
    nom = [[T(0, 9, 0, 0), T(1, 5, 0, 1), T(2, 6, 0, 2)], [T(0, 1, 0, 0), T(1, 7, 0, 2)],
           [T(0, 2, 0, 0), T(1, 8, 0, 2)]]
    m = diag_metrics(E, nom, 4)
    assert m["n_orph"] == 2 and m["n_post"] == 4
    assert m["dup_post"] == 2            # (5,0) by agent 2 after agent 1; (9,0) by agent 1 after the dead's valid look
    assert m["orph_dec"] == 2 and m["orph_cov"] == 1 and m["orph_dup"] == 1
    assert m["first_orph"] == 2 and m["orph_coll"] == 1
    assert m["own_total"] == 2 and m["own_missed"] == 1   # task (8,0) of agent 2 skipped; (7,0) done


# ------------------------------------------------------------------ MUST-FAIL
@pytest.mark.xfail(strict=True, reason="a coordinated beacon must change behaviour vs IMPLICIT under loss")
def test_mustfail_part_equals_implicit_under_loss():
    inst, nom = setup(5, 1, 1005)
    f = fault_spec("perm1", inst.horizon, 5)
    for ep in range(30):
        a = run_nb(inst, 5, "HEALTH_PART:near", LINKS["ideal"], ep, S, nom, f)
        b = run_nb(inst, 5, "IMPLICIT", LINKS["ideal"], ep, S, nom, f)
        assert key(a)[:6] == key(b)[:6]


@pytest.mark.xfail(strict=True, reason="at N=5 the partition rule must differ from plain HEALTH")
def test_mustfail_part_equals_health_at_n5():
    inst, nom = setup(5, 1, 1005)
    f = fault_spec("perm1", inst.horizon, 5)
    for ep in range(40):
        a = run_nb(inst, 5, "HEALTH_PART:rr", LINKS["ideal"], ep, S, nom, f)
        b = run_nb(inst, 5, "HEALTH", LINKS["ideal"], ep, S, nom, f)
        assert key(a)[:6] == key(b)[:6]


@pytest.mark.xfail(strict=True, reason="one taker is not a partition: ONE must differ from PART at N=10")
def test_mustfail_one_equals_part_at_n10():
    inst, nom = setup(10, 1, 1006)
    f = fault_spec("perm1", inst.horizon, 10)
    for ep in range(12):
        a = run_nb(inst, 10, "HEALTH_ONE:low", LINKS["ideal"], ep, S, nom, f)
        b = run_nb(inst, 10, "HEALTH_PART:near", LINKS["ideal"], ep, S, nom, f)
        assert key(a)[:6] == key(b)[:6]


@pytest.mark.xfail(strict=True, reason="diagnostic episodes have the target removed: no detection")
def test_mustfail_diag_episodes_never_detect_is_false():
    inst, nom = setup(5, 1, 1007)
    f = fault_spec("perm1", inst.horizon, 5)
    det = [run_nb(inst, 5, "IMPLICIT", LINKS["ideal"], ep, S, nom, f, diag=True).detected for ep in range(30)]
    assert any(det)


@pytest.mark.xfail(strict=True, reason="mutant: dropping the receiver filter (assignee test inverted) must not equal PART")
def test_mustfail_inverted_assignee_mutant_equals_part():
    class Inv(NBEpisode):
        def _partner_counts(self, a, t):
            if self.variant and self.p_dead[a.idx]:
                pd = self.p_dead[a.idx]
                kd = frozenset(pd)
                amap = {j: self._assignment(j, td, kd) for j, td in pd.items()}
                return self._filtered(a, t, lambda j, tk: j not in pd or tk.start < pd[j]
                                      or amap[j].get(tk.seq) == a.idx)       # inverted
            return super()._partner_counts(a, t)
    inst, nom = setup(5, 1, 1005)
    f = dict(fault_spec("perm1", inst.horizon, 5))
    for ep in range(30):
        ff = dict(f); ff["seed"] = 10_000 + ep
        a = Inv(inst, (S["HEALTH"],) * 5, LINKS["ideal"], np.random.default_rng(ep), nom, fault=ff,
                variant="part", rule="near").run()
        b = run_nb(inst, 5, "HEALTH_PART:near", LINKS["ideal"], ep, S, nom, f)
        assert key(a)[:6] == key(b)[:6]
