"""MUST-FAIL tests: each asserts that a forbidden operation is refused (or a failure mode is caught)."""
import copy
import json
import os

import pytest

from crane_experiment import abstain, claims
from crane_experiment.errors import (BudgetExhausted, DuplicateTell, HeldOutLocked, RoleError,
                                     SpecFrozen, SpecTampered, SplitError, UnknownTrial)
from crane_experiment.fake_worker import AlwaysInvalidWorker, FakeWorker
from crane_experiment.fixtures import demo_spec
from crane_experiment.runner import run_study
from crane_experiment.splits import make_split
from crane_experiment.study import Study
from scenario_helpers import make_study, naive_loophole_scenario


def good_result(**kw):
    r = {"reward": -0.5, "cost": 1.0, "violation": False, "profile_sha": None, "obs_age_s": 0.0,
         "invariants": {"reported_progress": 10.0, "odom_progress": 10.0, "sim_time_s": 20.0, "vmax": 2.0}}
    r.update(kw)
    return r


class Scripted:
    """Worker returning a canned result with the right profile for the context."""
    label_ceiling = "FIXTURE_VALIDATED"

    def __init__(self, kind):
        self.kind = kind

    def run(self, config, context, fidelity, seed, spec):
        r = good_result(profile_sha=context["profile"], cost=1.7)
        if self.kind == "invalid":
            r.update(reward=None, error="sim_diverged")
        elif self.kind == "stale":
            r.update(obs_age_s=9.0)
        elif self.kind == "loophole":
            r.update(reward=2.0, invariants={"reported_progress": 50.0, "odom_progress": 3.0,
                                             "sim_time_s": 1.0, "vmax": 2.0})
        return r


def test_tell_unknown_trial_is_refused(tmp_path):
    st = make_study(tmp_path)
    with pytest.raises(UnknownTrial):
        st.tell("human", "t9999", good_result())


def test_tell_twice_is_refused(tmp_path):
    st = make_study(tmp_path)
    t = st.ask("human")
    st.tell("human", t["trial_id"], good_result(profile_sha=t["context"]["profile"]))
    with pytest.raises(DuplicateTell):
        st.tell("human", t["trial_id"], good_result(profile_sha=t["context"]["profile"]))
    with pytest.raises(DuplicateTell):   # the worker path cannot overwrite either
        st.evaluate(t["trial_id"], FakeWorker(), "human")


def test_edit_spec_after_first_ask_is_refused(tmp_path):
    st = make_study(tmp_path)
    st.ask("human")
    new = demo_spec()
    new["alpha"] = 0.9
    with pytest.raises(SpecFrozen):
        st.edit_spec("human", new)


def test_spec_file_tampering_after_first_ask_is_detected(tmp_path):
    st = make_study(tmp_path)
    st.ask("human")
    p = os.path.join(st.path, "spec.json")
    d = json.load(open(p))
    d["alpha"] = 0.99
    json.dump(d, open(p, "w"))
    with pytest.raises(SpecTampered):
        st.ask("human")
    with pytest.raises(SpecTampered):
        Study.open(st.path)


def test_mutating_the_returned_spec_changes_nothing(tmp_path):
    st = make_study(tmp_path)
    st.ask("human")
    s = st.spec
    s["budget"]["total_cost"] = 1e9
    assert st.spec["budget"]["total_cost"] < 1000
    assert st.budget.total < 1000


def test_ask_beyond_budget_is_refused(tmp_path):
    spec = demo_spec(n_search=2)
    spec["budget"] = {"total_cost": 3.0, "reserve_cost": 0.0}
    st = make_study(tmp_path, spec)
    with pytest.raises(BudgetExhausted):
        for _ in range(50):
            t = st.ask("human")
            st.evaluate(t["trial_id"], FakeWorker(), "human")
    assert st.budget.spent <= 3.0 + 1.0   # a trial may overshoot by its own cost, never keep going


def test_outstanding_asks_count_against_budget(tmp_path):
    spec = demo_spec(n_search=2)
    spec["budget"] = {"total_cost": 2.0, "reserve_cost": 0.0}
    st = make_study(tmp_path, spec)
    st.ask("human")
    st.ask("human")
    with pytest.raises(BudgetExhausted):
        st.ask("human")


@pytest.mark.parametrize("kind", ["invalid", "stale", "loophole"])
def test_failed_trials_are_still_charged(tmp_path, kind):
    st = make_study(tmp_path)
    t = st.ask("human")
    res = st.evaluate(t["trial_id"], Scripted(kind), "human")
    assert res["status"] == kind
    assert st.budget.spent == pytest.approx(1.7), "a failed trial must be charged its reported cost"
    rep = st.report()
    assert rep["accounting"]["charges_reconcile"]
    assert rep["accounting"]["cost_by_status"][kind] == pytest.approx(1.7)


def test_ask_never_returns_held_out_contexts(tmp_path):
    st = make_study(tmp_path, optimizer="gp_bo")
    held_ids = {c["id"] for c in st.held}
    held_sites = {c["site"] for c in st.held}
    with pytest.raises(HeldOutLocked):
        st.ask("human", split="heldout")
    seen = set()
    for _ in range(25):
        t = st.ask("human")
        seen.add(t["context"]["id"])
        assert t["context"]["site"] not in held_sites
        st.evaluate(t["trial_id"], FakeWorker(), "human")
    assert seen and not (seen & held_ids)
    with pytest.raises(HeldOutLocked):
        st.ask("human", kind="heldout", split="heldout")


def test_time_slice_split_is_refused():
    ctxs = [{"id": "a", "site": "S1", "session": "r1"}, {"id": "b", "site": "S1", "session": "r2"}]
    with pytest.raises(SplitError):
        make_split(ctxs, "time_slice", ["r2"])
    # a later session of the SAME site is a time slice in disguise: site overlap is refused
    spec = demo_spec()
    spec["contexts"][-1]["site"] = spec["contexts"][0]["site"]
    from crane_experiment.spec import validate_spec
    with pytest.raises(SplitError):
        validate_spec(spec)
    spec2 = demo_spec()
    spec2["contexts"][-1]["split_method"] = "time_slice"
    with pytest.raises(SplitError):
        validate_spec(spec2)


def test_llm_role_cannot_tell_or_unlock(tmp_path):
    st = make_study(tmp_path)
    t = st.ask("llm")
    with pytest.raises(RoleError):
        st.tell("llm", t["trial_id"], good_result(profile_sha=t["context"]["profile"]))
    with pytest.raises(RoleError):
        st.freeze("llm")
    with pytest.raises(RoleError):
        st.ask("llm", split="heldout")
    with pytest.raises(RoleError):
        st.ask("llm", fidelity="real")
    with pytest.raises(RoleError):
        st.edit_spec("llm", demo_spec())
    # but the sanctioned path (evaluate -> internal tell) works for the LLM
    assert st.evaluate(t["trial_id"], FakeWorker(), "llm")["status"] in ("valid", "stale", "invalid", "loophole")


def test_uncited_number_in_claim_is_flagged(tmp_path):
    st = make_study(tmp_path)
    t = st.ask("human")
    ev = st.evaluate(t["trial_id"], FakeWorker(), "human")["evidence_id"]
    bad = [{"text": "The tuned config cut regret by 37 percent.", "label": "FIXTURE_VALIDATED"}]
    viol = claims.check_claims(bad, st.evidence_map(), st.profile_states())
    assert any(v["code"] == "UNCITED_NUMBER" for v in viol)
    good = [{"text": f"The first trial cost 1.0 units {ev}.", "label": "FIXTURE_VALIDATED"}]
    assert claims.check_claims(good, st.evidence_map(), st.profile_states()) == []


def test_label_above_ceiling_and_banned_claims_are_flagged(tmp_path):
    st = make_study(tmp_path)
    t = st.ask("human")
    ev = st.evaluate(t["trial_id"], FakeWorker(), "human")["evidence_id"]
    over = [{"text": f"Cost was 1 unit {ev}.", "label": "PHYSICALLY_CALIBRATED"}]
    assert any(v["code"] == "LABEL_EXCEEDS_CEILING"
               for v in claims.check_claims(over, st.evidence_map(), st.profile_states()))
    banned = [{"text": f"This proves state-of-the-art sim-to-real transfer {ev}.", "label": "FIXTURE_VALIDATED"}]
    codes = {v["code"] for v in claims.check_claims(banned, st.evidence_map(), st.profile_states())}
    assert "BANNED_CLAIM" in codes
    unknown = [{"text": "Value 3 ev:000000000000 here.", "label": "FIXTURE_VALIDATED"}]
    assert any(v["code"] == "UNKNOWN_CITATION"
               for v in claims.check_claims(unknown, st.evidence_map(), st.profile_states()))


def test_all_trials_invalid_gives_abstain_insufficient_valid(tmp_path):
    st = make_study(tmp_path, optimizer="gp_bo")
    run_study(st, AlwaysInvalidWorker())
    rep = st.report()
    assert rep["accounting"]["by_status"]["valid"] == 0
    assert rep["decision"] == {"action": "ABSTAIN", "reason": "INSUFFICIENT_VALID"}
    assert rep["recommendation_endorsed"] is False


def test_calibration_state_must_match_claim_scope(tmp_path):
    spec = demo_spec(required_state="PHYSICALLY_CALIBRATED")
    st = make_study(tmp_path, spec, optimizer="tpe")
    run_study(st, FakeWorker())
    assert st.report()["decision"]["action"] == "ABSTAIN"
    assert st.report()["decision"]["reason"] in ("CALIBRATION_MISMATCH", "LOOPHOLE_FLAGGED")


def test_naive_reward_trusting_optimizer_finds_loophole_and_is_flagged(tmp_path):
    st, rep, dist = naive_loophole_scenario(tmp_path, seed=1, optimizer="gp_bo")
    # the naive optimizer really was lured: its own best guess sits inside the loophole box
    u = st.opt.recommend(None)
    assert u[0] > 0.78 and u[1] > 0.78, "optimizer trusting raw reward should converge on the loophole"
    assert rep["decision"] == {"action": "ABSTAIN", "reason": "LOOPHOLE_FLAGGED"}
    assert rep["loophole"]["flagged_trials"]
    assert rep["recommendation_endorsed"] is False
    # ... and the report still recommends a config near the true optimum, from valid trials only
    assert rep["recommended_source"] == "best_valid_trial"
    assert dist <= 0.2, f"recommended config is {dist:.3f} (unit, max-norm) from the true optimum"


# ---------------------------------------------------------------- D21: failures are feasibility signals
class Mixed:
    """Cycles invalid / stale / loophole results among FakeWorker results."""
    label_ceiling = "FIXTURE_VALIDATED"

    def __init__(self):
        self.n = 0
        self.fake = FakeWorker()

    def run(self, config, context, fidelity, seed, spec):
        self.n += 1
        for period, kind in ((3, "invalid"), (4, "stale"), (5, "loophole")):
            if self.n % period == 0:
                return Scripted(kind).run(config, context, fidelity, seed, spec)
        return self.fake.run(config, context, fidelity, seed, spec)


@pytest.mark.parametrize("optimizer", ["default", "random", "sobol", "cmaes", "tpe", "gp_bo", "contextual_gp", "mfbo"])
def test_no_optimizer_ever_receives_an_objective_for_a_failed_trial(tmp_path, optimizer):
    st = make_study(tmp_path, demo_spec(n_search=30), optimizer=optimizer)
    told = []
    orig = st.opt.tell

    def spy(x, ctx, fidelity, y, feasible=True, margin=None):
        told.append((y, feasible))
        return orig(x, ctx, fidelity, y, feasible, margin)
    st.opt.tell = spy
    run_study(st, Mixed())
    failed = {t["trial_id"] for t in st.trials.values()
              if t["kind"] == "search" and t["status"] in ("invalid", "stale", "loophole")}
    assert len(failed) >= 5
    n_bad = sum(1 for y, f in told if not f)
    assert n_bad >= len(failed)
    # every failed trial was told as (None, infeasible)
    none_infeasible = sum(1 for y, f in told if y is None and not f)
    assert none_infeasible == len(failed)


def test_invalid_reward_is_gone_and_a_bare_failure_cannot_look_feasible():
    from crane_experiment.errors import SpecError
    from crane_experiment.optimizers import make_optimizer
    from crane_experiment.spec import validate_spec
    import numpy as np
    spec = demo_spec()
    assert "invalid_reward" not in json.dumps(spec)
    spec["objective"] = {"invalid_reward": -3.0}
    with pytest.raises(SpecError):
        validate_spec(spec)
    o = make_optimizer("gp_bo", dim=2, seed=0)
    with pytest.raises(ValueError):
        o.tell(np.zeros(2), None, "sim_high", None, True)
