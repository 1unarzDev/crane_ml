"""Strict CRANE calibration-profile artifacts.

The profile fitter deliberately consumes only trials assigned to the ``fit`` split.
It does not infer missing physical parameters from recorder time, simulation priors,
or held-out samples.  A sample must contain an explicit ``profileObservation``
object produced by a measurement/identification step.  This keeps the artifact
path useful before hardware arrives without manufacturing physical measurements.
"""

from __future__ import annotations

import hashlib
import json
import math
import statistics
from pathlib import Path
from typing import Any


class CalibrationProfileError(ValueError):
    """Raised when a profile or fit input violates the profile contract."""


PARAMETER_FIELDS = (
    "massKg",
    "motorLagSeconds",
    "accelerationMps2",
    "angularAccelerationRadPerSec2",
    "maxMotorTorqueNm",
    "leftWheelScale",
    "rightWheelScale",
)
LIDAR_FIELDS = ("sigmaMeters", "dropoutProbability", "latencyScans", "materialClass")
PROFILE_FIELDS = ("schema", "id", "version", "sourceKind", "parameters", "fitProvenance")
PROVENANCE_FIELDS = (
    "trialIds",
    "manifestSha256",
    "fitNote",
)


def _finite(value: Any, label: str) -> float:
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        raise CalibrationProfileError(f"{label} must be numeric")
    value = float(value)
    if not math.isfinite(value):
        raise CalibrationProfileError(f"{label} must be finite")
    return value


def _hash(value: Any, label: str) -> str:
    if not isinstance(value, str) or len(value) != 64:
        raise CalibrationProfileError(f"{label} must be a SHA-256 hash")
    try:
        int(value, 16)
    except ValueError as exc:
        raise CalibrationProfileError(f"{label} must be a SHA-256 hash") from exc
    return value.lower()


def validate_profile(profile: dict[str, Any]) -> dict[str, Any]:
    """Validate and return a profile without filling defaults."""

    if not isinstance(profile, dict):
        raise CalibrationProfileError("profile must be an object")
    if set(profile) != set(PROFILE_FIELDS):
        raise CalibrationProfileError(f"profile keys must be exactly {PROFILE_FIELDS}")
    if profile["schema"] != "crane-calibration-profile-v1":
        raise CalibrationProfileError("profile schema must be crane-calibration-profile-v1")
    for key in ("id", "version"):
        if not isinstance(profile[key], str) or not profile[key].strip():
            raise CalibrationProfileError(f"profile {key} must be non-empty")
    if profile["sourceKind"] not in ("physical", "synthetic-fixture"):
        raise CalibrationProfileError("profile sourceKind must be physical or synthetic-fixture")

    parameters = profile["parameters"]
    if not isinstance(parameters, dict) or set(parameters) != set(PARAMETER_FIELDS) | {"lidar"}:
        raise CalibrationProfileError("profile parameters have unexpected keys")
    ranges = {
        "massKg": (0.5, 5.0),
        "motorLagSeconds": (0.001, 2.0),
        "accelerationMps2": (0.0001, 3.0),
        "angularAccelerationRadPerSec2": (0.0001, 20.0),
        "maxMotorTorqueNm": (0.0001, 2.0),
    }
    for key, (lower, upper) in ranges.items():
        value = _finite(parameters[key], f"parameters.{key}")
        if not lower <= value <= upper:
            raise CalibrationProfileError(f"parameters.{key} outside supported range")
    for key in ("leftWheelScale", "rightWheelScale"):
        if not 0.9 <= _finite(parameters[key], f"parameters.{key}") <= 1.1:
            raise CalibrationProfileError(f"parameters.{key} outside supported range")
    lidar = parameters["lidar"]
    if not isinstance(lidar, dict) or set(lidar) != set(LIDAR_FIELDS):
        raise CalibrationProfileError("parameters.lidar has unexpected keys")
    if not 0 <= _finite(lidar["sigmaMeters"], "parameters.lidar.sigmaMeters") <= 0.1:
        raise CalibrationProfileError("parameters.lidar.sigmaMeters outside [0, 0.1]")
    if not 0 <= _finite(lidar["dropoutProbability"], "parameters.lidar.dropoutProbability") <= 0.5:
        raise CalibrationProfileError("parameters.lidar.dropoutProbability outside [0, 0.5]")
    latency = lidar["latencyScans"]
    if isinstance(latency, bool) or not isinstance(latency, int) or not 0 <= latency <= 3:
        raise CalibrationProfileError("parameters.lidar.latencyScans must be an integer in [0, 3]")
    if not isinstance(lidar["materialClass"], str) or not lidar["materialClass"].strip():
        raise CalibrationProfileError("parameters.lidar.materialClass must be non-empty")

    provenance = profile["fitProvenance"]
    if not isinstance(provenance, dict) or set(provenance) != set(PROVENANCE_FIELDS):
        raise CalibrationProfileError("fitProvenance has unexpected keys")
    trial_ids = provenance["trialIds"]
    if not isinstance(trial_ids, list) or not trial_ids or any(not isinstance(v, str) or not v for v in trial_ids):
        raise CalibrationProfileError("fitProvenance.trialIds must be a non-empty list")
    if len(set(trial_ids)) != len(trial_ids):
        raise CalibrationProfileError("fitProvenance.trialIds must be unique")
    _hash(provenance["manifestSha256"], "fitProvenance.manifestSha256")
    if not isinstance(provenance["fitNote"], str) or not provenance["fitNote"].strip():
        raise CalibrationProfileError("fitProvenance.fitNote must be non-empty")
    return profile


def canonical_bytes(profile: dict[str, Any]) -> bytes:
    validate_profile(profile)
    return (json.dumps(profile, sort_keys=True, separators=(",", ":"), ensure_ascii=False) + "\n").encode("utf-8")


def profile_sha256(profile: dict[str, Any]) -> str:
    return hashlib.sha256(canonical_bytes(profile)).hexdigest()


def write_profile(profile: dict[str, Any], path: Path) -> str:
    payload = canonical_bytes(profile)
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_bytes(payload)
    digest = hashlib.sha256(payload).hexdigest()
    path.with_suffix(path.suffix + ".sha256").write_text(digest + "\n")
    return digest


def read_profile(path: Path) -> dict[str, Any]:
    try:
        value = json.loads(path.read_text())
    except (OSError, json.JSONDecodeError) as exc:
        raise CalibrationProfileError(f"cannot read profile {path}: {exc}") from exc
    validate_profile(value)
    actual = hashlib.sha256(canonical_bytes(value)).hexdigest()
    if path.read_bytes() != canonical_bytes(value):
        raise CalibrationProfileError(f"profile {path} is not canonical JSON")
    return value


def _median(values: list[float], label: str) -> float:
    if not values:
        raise CalibrationProfileError(f"fit samples contain no {label} observations")
    return float(statistics.median(values))


def _observation(row: dict[str, Any], path: Path, line_number: int) -> dict[str, Any]:
    value = row.get("profileObservation")
    if not isinstance(value, dict):
        raise CalibrationProfileError(
            f"{path}:{line_number}: missing profileObservation; no parameter is inferred from priors"
        )
    required = set(PARAMETER_FIELDS) | {"lidar"}
    if set(value) != required:
        raise CalibrationProfileError(f"{path}:{line_number}: profileObservation keys are incomplete")
    lidar = value["lidar"]
    if not isinstance(lidar, dict) or set(lidar) != set(LIDAR_FIELDS):
        raise CalibrationProfileError(f"{path}:{line_number}: profileObservation.lidar is invalid")
    return value


def manifest_fit_sha256(manifest: dict[str, Any]) -> str:
    """Hash the manifest identity before its generated profile reference is attached."""

    identity = dict(manifest)
    identity.pop("calibrationProfile", None)
    payload = (json.dumps(identity, sort_keys=True, separators=(",", ":"), ensure_ascii=False) + "\n").encode("utf-8")
    return hashlib.sha256(payload).hexdigest()


def fit_profile(manifest_path: Path, profile_id: str, version: str, source_kind: str = "physical") -> dict[str, Any]:
    """Fit medians from explicit profile observations in fit trials only."""

    # Imported lazily to keep this module usable as a standalone artifact validator.
    from heldout_calibration import validate_manifest

    checked = validate_manifest(manifest_path, require_profile=False)
    manifest = checked["manifest"]
    if source_kind not in ("physical", "synthetic-fixture"):
        raise CalibrationProfileError("source_kind must be physical or synthetic-fixture")
    trial_ids: list[str] = []
    values: dict[str, list[float]] = {key: [] for key in PARAMETER_FIELDS}
    lidar_values: dict[str, list[Any]] = {key: [] for key in LIDAR_FIELDS}
    for trial in checked["fitTrials"]:
        trial_ids.append(trial["id"])
        root = manifest_path.parent.resolve()
        sim_path, real_path = tuple((root / trial[key]).resolve() for key in ("sim", "real"))
        for path in (sim_path, real_path):
            for line_number, line in enumerate(path.read_text().splitlines(), 1):
                if not line.strip():
                    continue
                try:
                    row = json.loads(line)
                except json.JSONDecodeError as exc:
                    raise CalibrationProfileError(f"{path}:{line_number}: invalid JSON") from exc
                observation = _observation(row, path, line_number)
                for key in PARAMETER_FIELDS:
                    values[key].append(_finite(observation[key], f"{path}:{line_number}:{key}"))
                lidar = observation["lidar"]
                lidar_values["sigmaMeters"].append(_finite(lidar["sigmaMeters"], "lidar.sigmaMeters"))
                lidar_values["dropoutProbability"].append(_finite(lidar["dropoutProbability"], "lidar.dropoutProbability"))
                latency = lidar["latencyScans"]
                if isinstance(latency, bool) or not isinstance(latency, int):
                    raise CalibrationProfileError(f"{path}:{line_number}: lidar.latencyScans must be an integer")
                lidar_values["latencyScans"].append(latency)
                material = lidar["materialClass"]
                if not isinstance(material, str) or not material:
                    raise CalibrationProfileError(f"{path}:{line_number}: lidar.materialClass must be non-empty")
                lidar_values["materialClass"].append(material)
    material_counts = {value: lidar_values["materialClass"].count(value) for value in set(lidar_values["materialClass"])}
    material = max(material_counts, key=lambda value: (material_counts[value], value))
    profile = {
        "schema": "crane-calibration-profile-v1",
        "id": profile_id,
        "version": version,
        "sourceKind": source_kind,
        "parameters": {
            **{key: _median(values[key], key) for key in PARAMETER_FIELDS},
            "lidar": {
                "sigmaMeters": _median(lidar_values["sigmaMeters"], "lidar.sigmaMeters"),
                "dropoutProbability": _median(lidar_values["dropoutProbability"], "lidar.dropoutProbability"),
                "latencyScans": int(round(_median(lidar_values["latencyScans"], "lidar.latencyScans"))),
                "materialClass": material,
            },
        },
        "fitProvenance": {
            "trialIds": trial_ids,
            "manifestSha256": manifest_fit_sha256(manifest),
            "fitNote": "Parameters are medians of explicit profileObservation records from fit trials only; held-out trials are never read.",
        },
    }
    validate_profile(profile)
    return profile
