import json
import math
import subprocess
import sys
from pathlib import Path

import pytest

from heldout_calibration import CalibrationValidationError, build_report, read_samples, residuals, validate_manifest


FACTORS = ["command", "route", "material", "day", "condition"]


def rows(time_key, offset=0.0):
    result = [
        {time_key: 0.0, "position": {"x": 0, "y": 0, "z": offset}, "yaw": 179, "velocity": {"x": 1, "y": 0, "z": 0}},
        {time_key: 1.0, "position": {"x": 1, "y": 0, "z": offset}, "yaw": -179, "velocity": {"x": 1, "y": 0, "z": 0}},
        {time_key: 2.0, "position": {"x": 2, "y": 0, "z": offset}, "yaw": -178, "velocity": {"x": 1, "y": 0, "z": 0}},
    ]
    if time_key == "acquisitionTime":
        for row in result:
            row["clockUncertaintySeconds"] = 0.001
    return result


def write_jsonl(path, values):
    path.write_text("".join(json.dumps(value) + "\n" for value in values))


def manifest(tmp_path, heldout_value="heldout", baseline=False):
    fit_factors = {factor: f"fit-{factor}" for factor in FACTORS}
    heldout_factors = {factor: f"{heldout_value}-{factor}" for factor in FACTORS}
    write_jsonl(tmp_path / "fit-sim.jsonl", rows("simulationTime"))
    write_jsonl(tmp_path / "fit-real.jsonl", rows("acquisitionTime", .02))
    write_jsonl(tmp_path / "held-sim.jsonl", rows("simulationTime"))
    write_jsonl(tmp_path / "held-real.jsonl", rows("acquisitionTime", .1))
    value = {
        "schema": "crane-physical-calibration-dataset-v1",
        "timeBasis": "acquisition",
        "coordinateFrame": "unity-right-up-forward",
        "sourceKind": "physical",
        "timing": {"clockSource": "test-monotonic", "synchronizationMethod": "shared-trigger", "maxClockUncertaintySeconds": 0.001},
        "frameTransform": {"sourceFrame": "test-flu", "targetFrame": "unity-right-up-forward", "method": "recorded-static-transform", "evidence": "fixture metadata"},
        "calibrationProfile": {"id": "test-profile", "version": "1", "parameterArtifactSha256": "0" * 64},
        "acceptance": {"minimumOverlapFraction": 0.99, "maxInterpolationGapSeconds": 1.01},
        "heldOutFactors": FACTORS,
        "trials": [
            {"id": "fit-1", "split": "fit", "sim": "fit-sim.jsonl", "real": "fit-real.jsonl", "factors": fit_factors},
            {"id": "held-1", "split": "heldout", "sim": "held-sim.jsonl", "real": "held-real.jsonl", "factors": heldout_factors},
        ],
    }
    if baseline:
        write_jsonl(tmp_path / "held-baseline.jsonl", rows("simulationTime", .3))
        value["trials"][1]["baseline"] = "held-baseline.jsonl"
    path = tmp_path / "manifest.json"
    import hashlib
    for trial in value["trials"]:
        trial.update(simTimeOffsetSeconds=0, physicalTimeOffsetSeconds=0, timeOriginEvidence="shared trigger at sample zero")
        trial["hashes"] = {key: hashlib.sha256((tmp_path / trial[key]).read_bytes()).hexdigest() for key in ("sim", "real")}
        if trial.get("baseline"):
            trial["hashes"]["baseline"] = hashlib.sha256((tmp_path / trial["baseline"]).read_bytes()).hexdigest()
    path.write_text(json.dumps(value))
    return path


def test_residuals_interpolate_and_wrap_yaw():
    sim = [{"simulationTime": .5, "position": {"x": .5, "y": 0, "z": 0}, "yaw": 180}]
    real = rows("acquisitionTime")
    result = residuals(sim, "simulationTime", real, "acquisitionTime")
    assert result["pairedSamples"] == 1
    assert math.isclose(result["positionRmseMeters"], 0, abs_tol=1e-12)
    assert result["yawRmseDegrees"] <= 1


def test_manifest_reports_heldout_residual_and_baseline(tmp_path):
    report = build_report(manifest(tmp_path, baseline=True))
    assert report["state"] == "RESIDUALS_COMPUTED"
    held = next(item for item in report["trials"] if item["split"] == "heldout")
    assert held["pairedSamples"] == 3
    assert held["positionRmseMeters"] < held["baselinePositionRmseMeters"]
    assert held["candidateBeatsBaseline"] is True
    assert len(held["physicalProvenance"]["sha256"]) == 64


def test_manifest_rejects_factor_leakage(tmp_path):
    path = manifest(tmp_path)
    payload = json.loads(path.read_text())
    payload["trials"][1]["factors"]["route"] = "fit-route"
    path.write_text(json.dumps(payload))
    with pytest.raises(CalibrationValidationError, match="overlap fit split"):
        validate_manifest(path)


def test_reader_rejects_nonmonotonic_timestamps(tmp_path):
    path = tmp_path / "bad.jsonl"
    write_jsonl(path, [rows("acquisitionTime")[1], rows("acquisitionTime")[0]])
    with pytest.raises(CalibrationValidationError, match="strictly increasing"):
        read_samples(path, "acquisitionTime")


def test_cli_keeps_invalid_or_missing_physical_data_explicit(tmp_path):
    path = manifest(tmp_path)
    path.unlink()
    # The command must produce a machine-readable invalid state instead of a fabricated pass.
    result = subprocess.run(
        [sys.executable, str(Path(__file__).with_name("validate_heldout_calibration.py")), str(path)],
        capture_output=True,
        text=True,
        check=False,
    )
    assert result.returncode == 2
    report = json.loads(path.with_suffix(".heldout.json").read_text())
    assert report["state"] == "INVALID_DATASET"


def test_preregistered_residual_threshold_rejects_large_error(tmp_path):
    path = manifest(tmp_path)
    payload = json.loads(path.read_text())
    payload["acceptance"].update(maxPositionRmseMeters=.05, maxYawRmseDegrees=1)
    path.write_text(json.dumps(payload))
    report = build_report(path)
    assert report["state"] == "HELDOUT_RESIDUAL_FAIL"


def test_sample_hash_mismatch_is_rejected_before_comparison(tmp_path):
    path = manifest(tmp_path)
    write_jsonl(tmp_path / "held-real.jsonl", rows("acquisitionTime", 99))
    with pytest.raises(CalibrationValidationError, match="hash does not match"):
        build_report(path)


def test_resolved_path_alias_cannot_cross_splits(tmp_path):
    path = manifest(tmp_path)
    payload = json.loads(path.read_text())
    payload["trials"][1]["real"] = "./fit-real.jsonl"
    path.write_text(json.dumps(payload))
    with pytest.raises(CalibrationValidationError, match="reused across trials"):
        validate_manifest(path)


def test_missing_clock_uncertainty_is_rejected(tmp_path):
    path = tmp_path / "physical.jsonl"
    values = rows("acquisitionTime")
    for value in values:
        del value["clockUncertaintySeconds"]
    write_jsonl(path, values)
    with pytest.raises(CalibrationValidationError, match="requires clockUncertaintySeconds"):
        read_samples(path, "acquisitionTime", require_clock_uncertainty=True)


def test_sparse_or_insufficient_overlap_is_rejected():
    sim = rows("simulationTime")
    real = rows("acquisitionTime")
    with pytest.raises(CalibrationValidationError, match="interpolation gap"):
        residuals(sim, "simulationTime", real, "acquisitionTime", .99, .5)
    short_real = real[1:]
    with pytest.raises(CalibrationValidationError, match="overlap fraction"):
        residuals(sim, "simulationTime", short_real, "acquisitionTime", .99, 2)


def test_legacy_real_flag_fails_closed_before_loading_telemetry(tmp_path):
    result = subprocess.run(
        [sys.executable, str(Path(__file__).with_name("analyze_calibration.py")), str(tmp_path), "--real", "arbitrary.jsonl"],
        capture_output=True, text=True, check=False,
    )
    assert result.returncode == 2
    assert "physical acquisition manifest" in result.stderr


def test_nonphysical_fixtures_cannot_produce_physical_pass(tmp_path):
    path = manifest(tmp_path)
    payload = json.loads(path.read_text())
    payload["sourceKind"] = "synthetic-fixture"
    payload["acceptance"].update(maxPositionRmseMeters=100, maxYawRmseDegrees=100)
    path.write_text(json.dumps(payload))
    assert build_report(path)["state"] == "SYNTHETIC_FIXTURE_RESIDUALS"


def test_raw_clock_offsets_are_applied_without_refitting(tmp_path):
    path = manifest(tmp_path)
    payload = json.loads(path.read_text())
    import hashlib
    for trial in payload["trials"]:
        values = rows("simulationTime")
        for value in values:
            value["simulationTime"] += 3.01
        sample_path = tmp_path / trial["sim"]
        write_jsonl(sample_path, values)
        trial["simTimeOffsetSeconds"] = 3.01
        trial["timeOriginEvidence"] = "fixed-step command event at Unity 3.01s"
        trial["hashes"]["sim"] = hashlib.sha256(sample_path.read_bytes()).hexdigest()
    path.write_text(json.dumps(payload))
    result = build_report(path)
    held = next(trial for trial in result["trials"] if trial["split"] == "heldout")
    assert math.isclose(held["positionRmseMeters"], .1, abs_tol=1e-12)


def test_baseline_must_use_same_grid(tmp_path):
    path = manifest(tmp_path, baseline=True)
    baseline_path = tmp_path / "held-baseline.jsonl"
    values = rows("simulationTime", .3)
    values.pop()
    write_jsonl(baseline_path, values)
    payload = json.loads(path.read_text())
    import hashlib
    payload["trials"][1]["hashes"]["baseline"] = hashlib.sha256(baseline_path.read_bytes()).hexdigest()
    path.write_text(json.dumps(payload))
    with pytest.raises(CalibrationValidationError, match="same acquisition comparison grid"):
        build_report(path)

