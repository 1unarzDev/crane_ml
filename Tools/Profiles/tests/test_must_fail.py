"""Must-fail suite: every case builds a valid document, breaks exactly one thing, and demands the named rule."""
from __future__ import annotations

import copy
import json

import pytest

from crane_profiles import canonical, validate, validate_file
from helpers import SHA_A, SHA_B, good_physical, good_prior, mutate, q


def _del(path_fn):
    return path_fn


def set_(d, path, value):
    cur = d
    parts = path.split(".")
    for p in parts[:-1]:
        cur = cur[p]
    cur[parts[-1]] = value


def pop_(d, path):
    cur = d
    parts = path.split(".")
    for p in parts[:-1]:
        cur = cur[p]
    cur.pop(parts[-1])


def phys(fn):
    return lambda: mutate(good_physical(), fn)


def prior(fn, reseal=True):
    return lambda: mutate(good_prior(), fn, reseal)


CASES = {
    # --- R1 units
    "missing_unit": (prior(lambda d: pop_(d, "dynamics.massKg.unit")), "R1"),
    "unparseable_unit": (prior(lambda d: set_(d, "dynamics.massKg.unit", "furlongs")), "R1"),
    "unit_dimension_mismatch_suffix": (prior(lambda d: set_(d, "dynamics.massKg.unit", "m")), "R1"),
    "unit_dimension_mismatch_force_as_time": (prior(lambda d: set_(d, "dynamics.tauUpS.unit", "N")), "R1"),
    "null_value_without_abstain": (prior(lambda d: set_(d, "dynamics.massKg.value", None)), "R1"),
    "non_numeric_value": (prior(lambda d: set_(d, "dynamics.massKg.value", "1.5")), "R1"),
    "nan_like_value_bool": (prior(lambda d: set_(d, "dynamics.massKg.value", True)), "R1"),
    # --- R2 frames
    "vector_without_frame": (prior(lambda d: pop_(d, "dynamics.dragLinear.frame")), "R2"),
    "vector_with_undeclared_frame": (prior(lambda d: set_(d, "dynamics.dragLinear.frame", "nowhere")), "R2"),
    "no_frames_declared": (prior(lambda d: set_(d, "frames", {})), "R2"),
    # --- R3 sources
    "unknown_source": (prior(lambda d: set_(d, "dynamics.massKg.source", "ghost")), "R3"),
    "source_without_license": (prior(lambda d: pop_(d, "sources.vendor.license")), "R3"),
    "source_with_blank_license": (prior(lambda d: set_(d, "sources.vendor.license", "  ")), "R3"),
    "cited_without_source": (prior(lambda d: pop_(d, "dynamics.massKg.source")), "R3"),
    "cited_without_locator": (prior(lambda d: pop_(d, "dynamics.massKg.locator")), "R3"),
    "bad_provenance_kind": (prior(lambda d: set_(d, "dynamics.massKg.provenance", "guessed")), "R3"),
    # --- R4 fitted
    "fitted_without_ci": (phys(lambda d: pop_(d, "dynamics.motorLagSeconds.uncertainty")), "R4"),
    "fitted_with_stddev_not_ci": (phys(lambda d: set_(d, "dynamics.motorLagSeconds.uncertainty", {"kind": "stddev", "value": 0.01})), "R4"),
    "fitted_value_outside_ci": (phys(lambda d: set_(d, "dynamics.motorLagSeconds.value", 0.5)), "R4"),
    "fitted_without_datasets": (phys(lambda d: pop_(d, "dynamics.motorLagSeconds.datasets")), "R4"),
    "fitted_with_undeclared_dataset": (phys(lambda d: set_(d, "dynamics.motorLagSeconds.datasets", ["nope"])), "R4"),
    # --- R5 PHYSICALLY_CALIBRATED
    "physically_calibrated_without_physical_dataset": (phys(lambda d: set_(d, "datasets", [])), "R5"),
    "physically_calibrated_synthetic_only": (phys(lambda d: [x.update(origin="synthetic-fixture") for x in d["datasets"]]), "R5"),
    "physically_calibrated_sim_truth_only": (phys(lambda d: [x.update(origin="sim_truth") for x in d["datasets"]]), "R5"),
    "physically_calibrated_without_heldout": (phys(lambda d: set_(d, "datasets", d["datasets"][:1])), "R5"),
    "physically_calibrated_fit_dataset_without_hash": (phys(lambda d: d["datasets"][0].pop("sha256")), "R5"),
    "physically_calibrated_unexcluded_sim_truth_coefficient": (
        phys(lambda d: (d["datasets"].append({"id": "sim1", "origin": "sim_truth", "split": "none", "sha256": SHA_A, "sites": [], "sessions": []}),
                        d.__setitem__("mixLabel", "SIM_TRUTH+PHYSICAL"),
                        d["dynamics"].__setitem__("tauUpS", q(0.05, "s", prov="fitted", datasets=["sim1"],
                            uncertainty={"kind": "ci", "level": 0.95, "lo": 0.04, "hi": 0.06}, identifiability={"status": "IDENTIFIED"}))
                        )), "R5"),
    "physically_calibrated_flagged_simtruth_coefficient": (phys(lambda d: d["dynamics"]["motorLagSeconds"].update(simTruth=True)), "R5"),
    "excluded_path_missing": (phys(lambda d: set_(d, "claimScope.excluded", ["dynamics.ghostS"])), "R5"),
    "bad_dataset_hash": (phys(lambda d: d["datasets"][0].update(sha256="xyz")), "R5"),
    "bad_dataset_origin": (phys(lambda d: d["datasets"][0].update(origin="lab")), "R5"),
    # --- R6 mixing
    "sim_truth_and_physical_mixed_without_label": (
        phys(lambda d: d["datasets"].append({"id": "sim1", "origin": "sim_truth", "split": "none", "sha256": SHA_A, "sites": [], "sessions": []})), "R6"),
    # --- R7 fixtureOnly
    "fixture_only_state_with_live_sim_label": (prior(lambda d: d.update(calibrationState="FIXTURE_ONLY", evidenceLabel="LIVE_SIM_VALIDATED")), "R7"),
    "fixture_only_coefficient_with_live_sim_label": (prior(lambda d: (d["dynamics"]["massKg"].update(provenance="fixtureOnly"), d.update(evidenceLabel="LIVE_SIM_VALIDATED"))), "R7"),
    "fixture_only_coefficient_under_physically_calibrated": (phys(lambda d: d["dynamics"]["dragLinear"].update(provenance="fixtureOnly")), "R7"),
    # --- R8 labels and state
    "unknown_evidence_label": (prior(lambda d: set_(d, "evidenceLabel", "CALIBRATED")), "R8"),
    "old_terrain_label_published_description_prior": (prior(lambda d: set_(d, "evidenceLabel", "PUBLISHED_DESCRIPTION_PRIOR")), "R8"),
    "old_terrain_label_full_legacy_string": (prior(lambda d: set_(d, "evidenceLabel", "PUBLISHED_DESCRIPTION_PRIOR (URDF values, not measured by CRANE)")), "R8"),
    "old_terrain_label_engineering_prior": (prior(lambda d: set_(d, "evidenceLabel", "IMPLEMENTED engineering prior; not PHYSICALLY_CALIBRATED")), "R8"),
    "unknown_calibration_state": (prior(lambda d: set_(d, "calibrationState", "CALIBRATED")), "R8"),
    "physically_calibrated_label_on_prior_state": (prior(lambda d: set_(d, "evidenceLabel", "PHYSICALLY_CALIBRATED")), "R8"),
    "wrong_schema": (prior(lambda d: set_(d, "schema", "crane.profile/v1")), "R8"),
    # --- R9 identifiability
    "abstain_without_reason": (prior(lambda d: pop_(d, "timing.gyroLatencyS.identifiability.reason")), "R9"),
    "abstain_with_blank_reason": (prior(lambda d: set_(d, "timing.gyroLatencyS.identifiability.reason", " ")), "R9"),
    "confounded_with_missing_path": (prior(lambda d: set_(d, "dynamics.tauUpS.identifiability.with", ["timing.nope"])), "R9"),
    "confounded_with_no_path": (prior(lambda d: set_(d, "dynamics.tauUpS.identifiability.with", [])), "R9"),
    "confounded_with_colon_form_missing_path": (prior(lambda d: set_(d, "dynamics.tauUpS.identifiability", {"status": "CONFOUNDED_WITH:timing.zzz", "reason": "r"})), "R9"),
    "combination_only_missing_path": (prior(lambda d: set_(d, "dynamics.massKg.identifiability.with", ["dynamics.ghostKg"])), "R9"),
    "unknown_identifiability_status": (prior(lambda d: set_(d, "dynamics.massKg.identifiability.status", "FINE")), "R9"),
    "non_identified_without_reason": (prior(lambda d: set_(d, "dynamics.dragLinear.identifiability", {"status": "SINGLE_SITE"})), "R9"),
    "fitted_without_identifiability": (phys(lambda d: pop_(d, "dynamics.motorLagSeconds.identifiability")), "R9"),
    # --- R10 hash
    "tampered_value_without_reseal": (prior(lambda d: set_(d, "dynamics.massKg.value", 99.0), reseal=False), "R10"),
    "tampered_hash_field": (prior(lambda d: set_(d, "contentHash", "0" * 64), reseal=False), "R10"),
    "missing_hash": (prior(lambda d: pop_(d, "contentHash"), reseal=False), "R10"),
    "malformed_hash": (prior(lambda d: set_(d, "contentHash", "ABC"), reseal=False), "R10"),
    "tampered_label_without_reseal": (prior(lambda d: set_(d, "evidenceLabel", "FIXTURE_VALIDATED"), reseal=False), "R10"),
    "bad_component_ref": (prior(lambda d: set_(d, "components", ["no-sha"])), "R10"),
    # --- R11 uncommitted
    "uncommitted_source_with_live_sim_label": (prior(lambda d: (d["sources"]["vendor"].update(uncommitted=True), d.update(evidenceLabel="LIVE_SIM_VALIDATED"))), "R11"),
    "uncommitted_source_with_fixture_validated_label": (prior(lambda d: (d["sources"]["vendor"].update(uncommitted=True), d.update(evidenceLabel="FIXTURE_VALIDATED"))), "R11"),
    # --- R12 split independence
    "fit_heldout_share_site": (phys(lambda d: d["datasets"][1].update(sites=["lot-a"])), "R12"),
    "fit_heldout_share_session": (phys(lambda d: d["datasets"][1].update(sessions=["s1"])), "R12"),
    "fit_heldout_share_runid": (phys(lambda d: d["datasets"][1].update(runIds=["r1"])), "R12"),
}


@pytest.mark.parametrize("name", sorted(CASES))
def test_must_fail(name):
    build, rule = CASES[name]
    rep = validate(build())
    assert not rep.ok, f"{name} unexpectedly validated"
    assert rule in rep.rules(), f"{name}: expected {rule}, got {rep.errors}"


def test_baselines_are_valid():
    assert validate(good_prior()).ok
    assert validate(good_physical()).ok


# --- manifests -------------------------------------------------------------------------------------------------
def manifest(trials, factors=("site",)):
    return canonical.seal({"schema": "crane.dataset_manifest/v2", "kind": "dataset_manifest", "origin": "physical",
                           "heldOutFactors": list(factors), "trials": trials})


def trial(i, split, site, session, run=None, **factors):
    return {"id": i, "split": split, "site": site, "session": session, "runId": run or i, "factors": {"site": site, **factors}}


@pytest.mark.parametrize("name,trials,rule", [
    ("manifest_share_site", [trial("a", "fit", "s1", "x"), trial("b", "heldout", "s1", "y")], "R12"),
    ("manifest_share_session", [trial("a", "fit", "s1", "x"), trial("b", "heldout", "s2", "x")], "R12"),
    ("manifest_share_runid", [trial("a", "fit", "s1", "x", "r"), trial("b", "heldout", "s2", "y", "r")], "R12"),
    ("manifest_no_heldout", [trial("a", "fit", "s1", "x"), trial("b", "fit", "s2", "y")], "R12"),
    ("manifest_duplicate_trial_id", [trial("a", "fit", "s1", "x"), trial("a", "heldout", "s2", "y")], "R12"),
])
def test_manifest_must_fail(name, trials, rule):
    rep = validate(manifest(trials))
    assert not rep.ok and rule in rep.rules()


def test_manifest_heldout_factor_overlap_fails():
    t = [trial("a", "fit", "s1", "x", surface="grass"), trial("b", "heldout", "s2", "y", surface="grass")]
    rep = validate(manifest(t, factors=("surface",)))
    assert "R12" in rep.rules()


# --- file names are never evidence ---------------------------------------------------------------------------------
def test_renaming_file_to_calibrated_changes_nothing(tmp_path):
    doc = good_prior()
    a, b = tmp_path / "profile.json", tmp_path / "profile_calibrated.json"
    for p in (a, b):
        p.write_text(json.dumps(doc))
    ra, rb = validate_file(a).to_dict(), validate_file(b).to_dict()
    assert ra == rb
    assert rb["calibrationState"] == "UNCALIBRATED_PRIOR" and rb["evidenceLabel"] == "IMPLEMENTED"


def test_calibrated_filename_does_not_rescue_invalid_claim(tmp_path):
    bad = mutate(good_prior(), lambda d: d.update(evidenceLabel="PHYSICALLY_CALIBRATED"))
    p = tmp_path / "physically_calibrated.json"
    p.write_text(json.dumps(bad))
    assert not validate_file(p).ok


def test_calibrated_directory_name_irrelevant(tmp_path):
    d = tmp_path / "PHYSICALLY_CALIBRATED"
    d.mkdir()
    p = d / "x_calibrated.json"
    p.write_text(json.dumps(good_prior()))
    rep = validate_file(p)
    assert rep.ok and rep.info["calibrationState"] == "UNCALIBRATED_PRIOR"


def test_unreadable_file_reported(tmp_path):
    rep = validate_file(tmp_path / "missing.json")
    assert not rep.ok


def test_non_object_document():
    assert not validate([1, 2]).ok
