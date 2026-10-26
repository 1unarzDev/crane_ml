from __future__ import annotations

import hashlib
import json
from pathlib import Path

import pytest

from calibration_profile import CalibrationProfileError, fit_profile, manifest_fit_sha256, read_profile, write_profile
from heldout_calibration import build_report, sha256_file


PARAMETERS = {
    "massKg": 1.55,
    "motorLagSeconds": 0.15,
    "accelerationMps2": 0.6,
    "angularAccelerationRadPerSec2": 1.2,
    "maxMotorTorqueNm": 0.35,
    "leftWheelScale": 1.0,
    "rightWheelScale": 1.01,
    "lidar": {"sigmaMeters": 0.005, "dropoutProbability": 0.02, "latencyScans": 1, "materialClass": "reflective"},
}


def observation(scale: float = 1.0) -> dict:
    return {
        **{key: value * scale for key, value in PARAMETERS.items() if key != "lidar"},
        "lidar": {
            "sigmaMeters": PARAMETERS["lidar"]["sigmaMeters"] * scale,
            "dropoutProbability": PARAMETERS["lidar"]["dropoutProbability"] * scale,
            "latencyScans": PARAMETERS["lidar"]["latencyScans"],
            "materialClass": PARAMETERS["lidar"]["materialClass"],
        },
    }


def write_samples(path: Path, scale: float, uncertainty: bool) -> None:
    rows = []
    for index in range(3):
        row = {
            "acquisitionTime" if uncertainty else "simulationTime": float(index),
            "position": {"x": float(index) * scale, "y": 0.0, "z": 0.0},
            "yaw": 0.0,
            "profileObservation": observation(scale),
        }
        if uncertainty:
            row["clockUncertaintySeconds"] = 0.001
        rows.append(row)
    path.write_text("".join(json.dumps(row, sort_keys=True) + "\n" for row in rows))


def make_manifest(tmp_path: Path) -> Path:
    write_samples(tmp_path / "fit-sim.jsonl", 1.0, False)
    write_samples(tmp_path / "fit-real.jsonl", 1.0, True)
    write_samples(tmp_path / "held-sim.jsonl", 9.0, False)
    write_samples(tmp_path / "held-real.jsonl", 9.0, True)
    manifest = {
        "schema": "crane-physical-calibration-dataset-v1",
        "timeBasis": "acquisition",
        "coordinateFrame": "unity-right-up-forward",
        "sourceKind": "synthetic-fixture",
        "timing": {"clockSource": "fixture", "synchronizationMethod": "same process", "maxClockUncertaintySeconds": 0.01},
        "frameTransform": {"sourceFrame": "fixture", "targetFrame": "unity-right-up-forward", "method": "identity", "evidence": "fixture"},
        "acceptance": {"minimumOverlapFraction": 1.0, "maxInterpolationGapSeconds": 1.1},
        "heldOutFactors": ["route"],
        "trials": [
            {"id": "fit-1", "split": "fit", "sim": "fit-sim.jsonl", "real": "fit-real.jsonl", "factors": {"route": "fit-route"}, "hashes": {} , "simTimeOffsetSeconds": 0, "physicalTimeOffsetSeconds": 0, "timeOriginEvidence": "fixture origin"},
            {"id": "held-1", "split": "heldout", "sim": "held-sim.jsonl", "real": "held-real.jsonl", "factors": {"route": "held-route"}, "hashes": {}, "simTimeOffsetSeconds": 0, "physicalTimeOffsetSeconds": 0, "timeOriginEvidence": "fixture origin"},
        ],
    }
    for trial in manifest["trials"]:
        trial["hashes"] = {key: sha256_file(tmp_path / trial[key]) for key in ("sim", "real")}
    path = tmp_path / "manifest.json"
    path.write_text(json.dumps(manifest, indent=2) + "\n")
    return path


def attach_profile(manifest_path: Path, profile_path: Path, profile: dict) -> Path:
    manifest = json.loads(manifest_path.read_text())
    manifest["calibrationProfile"] = {
        "id": profile["id"],
        "version": profile["version"],
        "parameterArtifactSha256": sha256_file(profile_path),
        "artifactPath": profile_path.name,
        "fitManifestSha256": manifest_fit_sha256(manifest),
        "fitTrialIds": profile["fitProvenance"]["trialIds"],
    }
    manifest_path.write_text(json.dumps(manifest, indent=2) + "\n")
    return manifest_path


def test_fit_artifact_and_heldout_provenance(tmp_path: Path) -> None:
    manifest = make_manifest(tmp_path)
    profile = fit_profile(manifest, "fixture-profile", "1", "synthetic-fixture")
    profile_path = tmp_path / "profile.json"
    digest = write_profile(profile, profile_path)
    assert digest == hashlib.sha256(profile_path.read_bytes()).hexdigest()
    assert read_profile(profile_path)["fitProvenance"]["trialIds"] == ["fit-1"]
    attach_profile(manifest, profile_path, profile)
    report = build_report(manifest)
    assert report["state"] == "SYNTHETIC_FIXTURE_RESIDUALS"
    assert report["calibrationProfileArtifact"]["sha256"] == digest
    assert report["calibrationProfile"]["fitTrialIds"] == ["fit-1"]
    assert profile["parameters"]["massKg"] == PARAMETERS["massKg"]


def test_fit_fails_closed_without_explicit_observation(tmp_path: Path) -> None:
    manifest = make_manifest(tmp_path)
    sample = tmp_path / "fit-sim.jsonl"
    row = json.loads(sample.read_text().splitlines()[0])
    del row["profileObservation"]
    sample.write_text(json.dumps(row) + "\n" + "\n".join(sample.read_text().splitlines()[1:]) + "\n")
    manifest_value = json.loads(manifest.read_text())
    manifest_value["trials"][0]["hashes"]["sim"] = sha256_file(sample)
    manifest.write_text(json.dumps(manifest_value) + "\n")
    with pytest.raises(CalibrationProfileError, match="profileObservation"):
        fit_profile(manifest, "fixture-profile", "1", "synthetic-fixture")


def test_heldout_artifact_tamper_is_rejected(tmp_path: Path) -> None:
    manifest = make_manifest(tmp_path)
    profile = fit_profile(manifest, "fixture-profile", "1", "synthetic-fixture")
    profile_path = tmp_path / "profile.json"
    write_profile(profile, profile_path)
    attach_profile(manifest, profile_path, profile)
    profile_path.write_text(profile_path.read_text().replace('"massKg":1.55', '"massKg":1.56'))
    with pytest.raises(Exception, match="profile"):
        build_report(manifest)
