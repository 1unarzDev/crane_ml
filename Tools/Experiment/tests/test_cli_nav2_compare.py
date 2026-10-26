"""Must-pass: CLI JSON contract + role enforcement, CRANE binding stub, comparison harness."""
import io
import json
import os

import pytest

from crane_experiment import cli, compare, crane_nav2
from crane_experiment.errors import NotRun, StudyError
from crane_experiment.fixtures import demo_spec

SAMPLES = os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "examples", "samples")


def run_cli(capsys, *argv):
    code = cli.main(list(argv))
    out = json.loads(capsys.readouterr().out)
    return code, out


def test_cli_roundtrip_and_role_enforcement(tmp_path, capsys):
    spec = os.path.join(str(tmp_path), "spec.json")
    json.dump(demo_spec(), open(spec, "w"))
    d = os.path.join(str(tmp_path), "study")
    code, out = run_cli(capsys, "--role", "human", "create", "--study", d, "--spec", spec, "--optimizer", "tpe")
    assert code == 0 and out["ok"]
    code, ask = run_cli(capsys, "--role", "llm", "ask", "--study", d)
    tid = ask["result"]["trial_id"]
    # the LLM cannot tell, freeze, edit the spec, or read held-out
    for argv in (["tell", "--study", d, "--trial", tid, "--result", "{}"], ["freeze", "--study", d],
                 ["edit-spec", "--study", d, "--spec", spec], ["ask", "--study", d, "--split", "heldout"],
                 ["ask", "--study", d, "--fidelity", "real"]):
        code, out = run_cli(capsys, "--role", "llm", *argv)
        assert code == 2 and out["error"] == "ROLE_DENIED", argv
    code, out = run_cli(capsys, "--role", "llm", "evaluate", "--study", d, "--trial", tid)
    assert code == 0 and out["result"]["evidence_id"].startswith("ev:")
    code, out = run_cli(capsys, "--role", "llm", "evaluate", "--study", d, "--trial", tid)
    assert out["error"] == "DUPLICATE_TELL"
    code, out = run_cli(capsys, "--role", "human", "tell", "--study", d, "--trial", "t9999", "--result", "{}")
    assert out["error"] == "UNKNOWN_TRIAL"
    code, out = run_cli(capsys, "--role", "llm", "verify", "--study", d)
    assert out["result"]["entries_verified"] >= 3
    code, rep = run_cli(capsys, "--role", "llm", "report", "--study", d)
    assert rep["result"]["decision"]["action"] == "ABSTAIN"


def test_cli_claims_check(tmp_path, capsys):
    spec = os.path.join(str(tmp_path), "spec.json")
    json.dump(demo_spec(), open(spec, "w"))
    d = os.path.join(str(tmp_path), "study")
    run_cli(capsys, "--role", "llm", "create", "--study", d, "--spec", spec)
    code, ask = run_cli(capsys, "--role", "llm", "ask", "--study", d)
    code, ev = run_cli(capsys, "--role", "llm", "evaluate", "--study", d, "--trial", ask["result"]["trial_id"])
    eid = ev["result"]["evidence_id"]
    good = json.dumps([{"text": f"One trial cost 1.0 units {eid}.", "label": "FIXTURE_VALIDATED"}])
    code, out = run_cli(capsys, "--role", "llm", "claims", "check", "--study", d, "--claims", good)
    assert code == 0 and out["result"]["ok"]
    bad = json.dumps([{"text": "Regret fell by 40%.", "label": "FIXTURE_VALIDATED"}])
    code, out = run_cli(capsys, "--role", "llm", "claims", "check", "--study", d, "--claims", bad)
    assert code == 1 and out["result"]["violations"][0]["code"] == "UNCITED_NUMBER"


def test_crane_nav2_is_not_run_and_never_launches():
    w = crane_nav2.CraneNav2Worker()
    with pytest.raises(NotRun, match="NOT_RUN"):
        w.run({}, {"profile": "p@abc"}, "real", 1, {})
    with pytest.raises(NotRun):
        w.launch()
    assert not crane_nav2.preregistration_frozen()   # PREREGISTRATION.md is DRAFT
    src = open(crane_nav2.__file__).read()
    assert "subprocess" not in src and "Popen" not in src and "os.system" not in src


@pytest.mark.parametrize("domain,ok", [(200, True), (214, True), (228, True), (199, False), (229, False),
                                       (42, False), (142, False), (0, False)])
def test_ros_domain_range(domain, ok):
    if ok:
        assert crane_nav2.validate_ros_domain(domain) == domain
    else:
        with pytest.raises(StudyError):
            crane_nav2.validate_ros_domain(domain)


def test_isolated_domain_is_always_in_range():
    assert {crane_nav2.isolated_domain(i) for i in range(500)} <= set(range(200, 229))


def test_artifact_parsers_on_constructed_samples():
    s = crane_nav2.parse_reset_summary(os.path.join(SAMPLES, "navigation-reset-summary.json"))
    assert s["valid"] and s["rosDomainId"] == 207 and s["actions"]["accepted"] == 412
    b = crane_nav2.parse_benchmark_counters(os.path.join(SAMPLES, "benchmark-counters.json"))
    assert b["counters"]["acceptedActions"] == 412
    r = crane_nav2.parse_replay_inspect(os.path.join(SAMPLES, "replay-inspect.json"))
    sha = r["calibrationProfileSha256"]
    assert crane_nav2.check_profile_match(r, f"campus@{sha}") == "OK"
    assert crane_nav2.check_profile_match(r, "campus@" + "f" * 64) == "PROFILE_MISMATCH"
    assert crane_nav2.check_profile_match({}, f"campus@{sha}") == "PROFILE_MISMATCH"


def test_reset_summary_in_forbidden_domain_is_rejected(tmp_path):
    p = os.path.join(str(tmp_path), "x.json")
    json.dump({"schema": "crane-nav2-scene-reload-v1", "transport": {"rosDomainId": 42}}, open(p, "w"))
    with pytest.raises(StudyError):
        crane_nav2.parse_reset_summary(p)
    json.dump({"schema": "something-else"}, open(p, "w"))
    with pytest.raises(StudyError):
        crane_nav2.parse_reset_summary(p)


def test_compare_small_run_is_labelled_and_equal_budget():
    rep = compare.compare(optimizers=["default", "random", "gp_bo"], replicates=2, n_search=10, seed=5)
    assert rep["label"] == "FIXTURE_ONLY" and rep["evidence"] == "NOT_EVIDENCE"
    t = rep["table"]
    assert set(t) == {"default", "random", "gp_bo"}
    assert all(len(v["regret"]) == 2 and v["iqr"] >= 0 for v in t.values())
    assert max(v["mean_spent"] for v in t.values()) - min(v["mean_spent"] for v in t.values()) < 2.0
    assert t["default"]["median_regret"] > 0.3
    json.dumps(rep)
