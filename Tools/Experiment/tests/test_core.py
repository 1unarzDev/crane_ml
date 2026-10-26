"""Must-pass: ledger, seeds, budget, splits, replay, abstention gates, role table."""
import json
import os

import numpy as np
import pytest

from crane_experiment import abstain
from crane_experiment.errors import LedgerCorrupt
from crane_experiment.fake_worker import FakeWorker
from crane_experiment.fixtures import demo_spec
from crane_experiment.ledger import Ledger, read_verified
from crane_experiment.runner import run_study
from crane_experiment.seeds import crn_seed, optimizer_seed
from crane_experiment.splits import make_split
from crane_experiment.study import Study
from scenario_helpers import make_study


def test_ledger_chain_verifies_and_detects_tampering(tmp_path):
    p = os.path.join(str(tmp_path), "l.jsonl")
    led = Ledger(p)
    for i in range(5):
        led.append("x", {"i": i})
    assert led.verify() == 5
    lines = open(p).read().splitlines()
    for mutate in (lambda L: [L[0], L[1].replace('"i":1', '"i":9')] + L[2:],      # edit
                   lambda L: L[:2] + L[3:],                                        # delete
                   lambda L: [L[1], L[0]] + L[2:]):                                # reorder
        open(p, "w").write("\n".join(mutate(lines)) + "\n")
        with pytest.raises(LedgerCorrupt):
            read_verified(p)


def test_evidence_ids_are_sha12(tmp_path):
    led = Ledger(os.path.join(str(tmp_path), "l.jsonl"))
    e = led.append("x", {"a": 1})
    assert ("ev:" + e["hash"][:12]) in led.evidence_ids()


@pytest.mark.parametrize("optimizer", ["default", "random", "sobol", "cmaes", "tpe", "gp_bo", "contextual_gp", "mfbo"])
def test_ledger_replay_reproduces_state(tmp_path, optimizer):
    spec = demo_spec(n_search=14)
    st = make_study(tmp_path, spec, optimizer=optimizer)
    run_study(st, FakeWorker())
    st2 = Study.open(st.path)
    assert st2.budget.spent == pytest.approx(st.budget.spent)
    assert {k: (t["status"], t["x"]) for k, t in st2.trials.items()} == {k: (t["status"], t["x"]) for k, t in st.trials.items()}
    a, b = st.report(), st2.report()
    assert a["decision"] == b["decision"] and a["recommended_config"] == b["recommended_config"]
    # replay is also a continuation point: the next ask is identical
    assert st.opt.ask(None)[0] == pytest.approx(st2.opt.ask(None)[0])


def test_crn_seeds_depend_only_on_context_and_replicate():
    assert crn_seed(5, "a", 0) == crn_seed(5, "a", 0)
    assert len({crn_seed(5, "a", 0), crn_seed(5, "a", 1), crn_seed(5, "b", 0), crn_seed(6, "a", 0)}) == 4
    assert optimizer_seed(1, "gp_bo") != optimizer_seed(1, "tpe")


def test_common_random_numbers_pair_baseline_and_confirm(tmp_path):
    st = make_study(tmp_path, optimizer="tpe")
    run_study(st, FakeWorker())
    base = {(t["context_id"], t["k"]): t for t in st.trials.values() if t["kind"] == "baseline"}
    conf = {(t["context_id"], t["k"]): t for t in st.trials.values() if t["kind"] == "confirm"}
    assert base.keys() == conf.keys() and len(base) == 6
    for key in base:
        assert base[key]["seed"] == conf[key]["seed"]


def test_budget_reconciles_with_charges_and_uses_reported_cost(tmp_path):
    st = make_study(tmp_path, optimizer="random")
    run_study(st, FakeWorker())
    acc = st.report()["accounting"]
    assert acc["charges_reconcile"]
    costs = [t["cost"] for t in st.trials.values()]
    assert len(set(round(c, 6) for c in costs)) > 3   # cost is worker-reported, not a constant
    assert st.budget.spent <= st.budget.total + 1.0


def test_equal_budget_across_optimizers(tmp_path):
    spent = {}
    for o in ("random", "gp_bo", "mfbo"):
        st = make_study(tmp_path, optimizer=o, name=o)
        run_study(st, FakeWorker())
        spent[o] = st.budget.spent
    assert max(spent.values()) - min(spent.values()) < 3.0, spent


def test_split_by_whole_site_and_session():
    ctxs = [{"id": f"c{i}", "site": f"S{i % 3}", "session": f"R{i}"} for i in range(6)]
    train, held = make_split(ctxs, "site", ["S2"])
    assert {c["site"] for c in held} == {"S2"} and not ({c["site"] for c in train} & {"S2"})
    from crane_experiment.errors import SplitError
    with pytest.raises(SplitError):   # holding out one session of a shared site still leaks the site
        make_split(ctxs, "session", ["R0"])
    own = [{"id": f"c{i}", "site": f"S{i}", "session": f"R{i}"} for i in range(4)]
    _, held = make_split(own, "session", ["R0"])
    assert [c["id"] for c in held] == ["c0"]


def test_freeze_unlocks_heldout_and_blocks_further_search(tmp_path):
    st = make_study(tmp_path, optimizer="tpe")
    run_study(st, FakeWorker())
    st.freeze("human")
    seen = set()
    for _ in range(8):
        t = st.ask("human", split="heldout")
        seen.add(t["context"]["id"])
        st.evaluate(t["trial_id"], FakeWorker(), "human")
    assert seen == {"held0", "held1"}
    rep = st.report()
    assert rep["heldout"]["n_pairs"] == 4
    with pytest.raises(Exception):
        st.ask("human")


@pytest.mark.parametrize("over,reason", [
    (dict(loophole_count=1), "LOOPHOLE_FLAGGED"), (dict(n_valid=0), "INSUFFICIENT_VALID"),
    (dict(valid_fraction=0.1), "INSUFFICIENT_VALID"), (dict(has_candidate=False), "NO_CANDIDATE"),
    (dict(profile_mismatch=True), "PROFILE_MISMATCH"), (dict(calibration_ok=False), "CALIBRATION_MISMATCH"),
    (dict(constraint_risk=0.5), "CONSTRAINT_RISK"), (dict(improvement_lcb=-0.1), "NO_IMPROVEMENT"),
    ({}, None)])
def test_abstain_reasons_are_exhaustive_and_ordered(over, reason):
    g = dict(loophole_count=0, n_trials=20, n_valid=18, valid_fraction=0.9, min_valid_fraction=0.5,
             has_candidate=True, profile_mismatch=False, calibration_ok=True, constraint_risk=0.05,
             alpha=0.2, improvement_lcb=0.3)
    g.update(over)
    action, why = abstain.decide(g)
    assert why == reason and action == ("RECOMMEND" if reason is None else "ABSTAIN")
    assert len(abstain.REASONS) == 7


def test_lcb_of_improvement():
    assert abstain.lcb_mean([1.0]) == float("-inf")
    assert abstain.lcb_mean([0.5, 0.6, 0.55]) > 0
    assert abstain.lcb_mean([-0.5, 0.6, 0.0]) < 0


def test_good_study_can_recommend(tmp_path):
    st = make_study(tmp_path, optimizer="cmaes")
    run_study(st, FakeWorker())
    rep = st.report()
    assert rep["decision"]["action"] == "RECOMMEND", rep["decision"]
    assert rep["label_ceiling"] == "FIXTURE_VALIDATED" and "FIXTURE_ONLY" in rep["note"]
