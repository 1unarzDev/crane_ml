"""import_yaml.py: the SIM_DERIVED import of crane's own hardware_crane.yaml, round trips, and what
an accepted characterization overwrites."""
import copy
import json
import os
import subprocess
import sys

import pytest

import gate as G
import import_yaml as I
import plant as P
import synthetic as S

SIM_YAML = "/home/lunarz/astro_dock/src/mhseals_testing/config/sim/hardware_crane.yaml"
HERE = os.path.dirname(os.path.abspath(__file__))


def physical(profile):
    """Every physical number of a profile (all but name and provenance), flattened."""
    out = {}

    def walk(prefix, node):
        if isinstance(node, dict):
            for k, v in node.items():
                walk(f"{prefix}.{k}" if prefix else k, v)
        elif isinstance(node, list):
            for i, v in enumerate(node):
                walk(f"{prefix}[{i}]", v)
        else:
            out[prefix] = node
    walk("", {k: v for k, v in profile.items() if k not in ("name", "provenance")})
    return out


def keys(node, prefix=""):
    if isinstance(node, dict):
        return {f"{prefix}.{k}" for k in node} | {x for k, v in node.items() for x in keys(v, f"{prefix}.{k}")}
    if isinstance(node, list) and node and isinstance(node[0], dict):
        return {x for v in node for x in keys(v, f"{prefix}[]")}
    return set()


def write_inputs(tmp_path, inp):
    paths = {}
    for k, ext in (("yaml", "yaml"), ("provenance", "json"), ("run", "json"), ("snapshot", "json")):
        p = tmp_path / f"{k}.{ext}"
        if k == "yaml":
            import yaml
            p.write_text(yaml.safe_dump(inp[k]))
        else:
            p.write_text(json.dumps(inp[k]))
        paths[k] = str(p)
    return paths


def run_import(tmp_path, paths, base=None, extra=()):
    out, rep = tmp_path / "profile.json", tmp_path / "report.json"
    argv = ["--yaml", paths["yaml"], "--out-profile", str(out), "--out-report", str(rep)]
    for flag, k in (("--provenance", "provenance"), ("--run", "run"), ("--config-snapshot", "snapshot")):
        if paths.get(k):
            argv += [flag, paths[k]]
    if base:
        argv += ["--base", base]
    assert I.main(argv + list(extra)) == 0
    return json.loads(out.read_text()), json.loads(rep.read_text()), str(rep)


@pytest.mark.skipif(not os.path.exists(SIM_YAML), reason=f"{SIM_YAML} not present (mhseals_testing not checked out)")
def test_sim_yaml_imports_as_ideal_with_sim_derived(tmp_path):
    prof, rep, rep_path = run_import(tmp_path, {"yaml": SIM_YAML})
    assert physical(prof) == physical(P.IDEAL)
    assert rep["summary"] == "prior" and not rep["characterized"]
    assert "SIM_DERIVED" in rep["reasons"]
    rows = {e["name"]: e for e in prof["provenance"]["parameters"]}
    for name in G.DIRECTIONS:
        assert rows[name]["status"] == "prior" and "SIM_DERIVED" in rows[name]["reason"]
    pv = prof["provenance"]
    assert pv["kind"] == "characterized" and pv["gateReport"] == rep_path and pv["summary"].startswith("prior")
    assert pv["inputs"][0] == {"path": SIM_YAML, "sha256": G.sha256_file(SIM_YAML)}
    # the gate still read the curves: what it would have imported is in the report, not the profile
    assert abs(rep["parameters"]["surge+"]["d1"] - 0.0857) < 1e-12


def test_profile_keeps_the_shared_schema(tmp_path):
    inp = S.analytic_inputs()
    prof, _, _ = run_import(tmp_path, write_inputs(tmp_path, inp))
    # Ideal's inputs list is empty, so its element fields are checked row by row below
    assert keys(prof) - {".provenance.inputs[].path", ".provenance.inputs[].sha256"} == keys(P.IDEAL)
    for row in prof["provenance"]["parameters"]:
        assert set(row) == {"name", "status", "reason"} and row["status"] in ("identified", "provisional", "prior")
    for row in prof["provenance"]["inputs"]:
        assert set(row) == {"path", "sha256"}


def test_nothing_identified_round_trips_identical_numbers(tmp_path):
    inp = S.analytic_inputs()
    inp["run"]["conditions"]["verdict"] = "poor"
    paths = write_inputs(tmp_path, inp)
    prof, rep, _ = run_import(tmp_path, paths)
    assert rep["summary"] == "prior" and physical(prof) == physical(P.IDEAL)
    # the generated profile as the next base: still the same numbers
    base = tmp_path / "base.json"
    base.write_text(json.dumps(prof))
    sub = tmp_path / "again"
    sub.mkdir()
    prof2, _, _ = run_import(sub, paths, base=str(base))
    assert physical(prof2) == physical(prof)


def test_accepted_characterization_overwrites_only_gated_parameters(tmp_path):
    curves = S.ideal_curves()
    for name in G.DIRECTIONS:  # a boat 20 % draggier than Ideal
        curves[name] = {k: v * 1.2 for k, v in curves[name].items()}
    mu = {a: v * 1.1 for a, v in S.ideal_mu().items()}
    inp = S.analytic_inputs(curves=curves, mu=mu)
    inp["run"]["directions"]["sway-"]["converged"] = False
    prof, rep, _ = run_import(tmp_path, write_inputs(tmp_path, inp))
    assert rep["characterized"] and prof["provenance"]["summary"].startswith("characterized")
    for axis in G.AXES:
        for suf in ("Pos", "Neg"):
            for d in ("d1", "d2"):
                want = P.IDEAL[axis][d + suf] * (1.0 if (axis, suf) == ("sway", "Neg") else 1.2)
                assert abs(prof[axis][d + suf] / want - 1) < 1e-9, (axis, d + suf)
    meff = P.effective_mass(P.IDEAL)
    assert abs(prof["mass"] + prof["addedMass"]["a11"] - 1.1 * meff[0]) < 1e-9
    assert abs(prof["izz"] + prof["addedMass"]["a66"] - 1.1 * meff[2]) < 1e-9
    assert prof["envelope"]["sway"] == P.IDEAL["envelope"]["sway"]      # sway- not taken
    assert prof["envelope"]["surge"] != P.IDEAL["envelope"]["surge"]
    for k in ("mass", "izz", "thrusters", "heave", "fixedDeltaTime"):
        assert prof[k] == P.IDEAL[k]
    rows = {e["name"]: e for e in prof["provenance"]["parameters"]}
    assert rows["sway-"]["status"] == "prior" and rows["sway-"]["reason"] == "NOT_CONVERGED"
    assert rows["sway"]["status"] == "prior" and rows["surge"]["status"] == "identified"
    assert {r["path"] for r in prof["provenance"]["inputs"]} >= {str(tmp_path / "yaml.yaml")}


def test_provisional_parameters_are_taken_and_marked(tmp_path):
    inp = S.analytic_inputs()
    inp["run"]["conditions"] = None
    prof, rep, _ = run_import(tmp_path, write_inputs(tmp_path, inp))
    rows = {e["name"]: e for e in prof["provenance"]["parameters"]}
    assert rep["summary"] == "partial"
    assert rows["surge+"]["status"] == "provisional" and rows["surge+"]["reason"] == "CONDITIONS_UNKNOWN"
    assert abs(prof["surge"]["d1Pos"] - P.IDEAL["surge"]["d1Pos"]) < 1e-9


def test_cli_runs_isolated_from_the_worktree_root(tmp_path):
    inp = S.analytic_inputs(amplitudes=(0.75, 0.25))
    paths = write_inputs(tmp_path, inp)
    root = os.path.dirname(os.path.dirname(HERE))
    out, rep = tmp_path / "p.json", tmp_path / "r.json"
    cmd = [sys.executable, "-I", "Tools/Characterization/import_yaml.py", "--yaml", paths["yaml"],
           "--provenance", paths["provenance"], "--run", paths["run"], "--config-snapshot", paths["snapshot"],
           "--out-profile", str(out), "--out-report", str(rep)]
    r = subprocess.run(cmd, cwd=root, capture_output=True, text=True, timeout=120)
    assert r.returncode == 0, r.stderr
    assert r.stdout.startswith("partial")
    prof = json.loads(out.read_text())
    assert {e["status"] for e in prof["provenance"]["parameters"] if e["name"] in G.DIRECTIONS} == {"provisional"}
