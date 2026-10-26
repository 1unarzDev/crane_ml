"""Utilities for validating acquisition-aligned CRANE calibration datasets.

The Unity calibration tools intentionally keep simulation truth and physical
measurements separate.  This module provides the small, explicit contract
needed before comparing them: a manifest names the split/factors, physical
samples carry acquisition timestamps, and comparisons interpolate only inside
the measured time range.  No parameter fitting or time re-registration is
performed here.
"""

from __future__ import annotations

import hashlib
import bisect
import json
import math
from pathlib import Path
from typing import Any, Iterable


class CalibrationValidationError(ValueError):
    """Raised when a dataset violates the calibration evidence contract."""


def sha256_file(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def _finite(value: Any, label: str) -> float:
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        raise CalibrationValidationError(f"{label} must be numeric")
    value = float(value)
    if not math.isfinite(value):
        raise CalibrationValidationError(f"{label} must be finite")
    return value


def _vector(record: dict[str, Any], name: str, required: bool = True) -> tuple[float, ...] | None:
    value = record.get(name)
    if value is None and not required:
        return None
    if not isinstance(value, dict):
        raise CalibrationValidationError(f"sample {name} must be an object")
    keys = ("x", "y", "z")
    if any(key not in value for key in keys):
        raise CalibrationValidationError(f"sample {name} requires x, y and z")
    return tuple(_finite(value[key], f"sample {name}.{key}") for key in keys)


def read_samples(path: Path, timestamp_field: str, require_clock_uncertainty: bool = False) -> list[dict[str, Any]]:
    """Read canonical JSONL samples and enforce acquisition ordering.

    ``timestamp_field`` is explicit because simulation telemetry uses
    ``simulationTime`` while a physical stream must use an acquisition clock.
    Receive or write time must not be silently substituted.
    """

    if not path.is_file():
        raise CalibrationValidationError(f"missing sample file: {path}")
    samples: list[dict[str, Any]] = []
    previous = None
    for line_number, line in enumerate(path.read_text().splitlines(), 1):
        if not line.strip():
            continue
        try:
            record = json.loads(line)
        except json.JSONDecodeError as exc:
            raise CalibrationValidationError(f"{path}:{line_number}: invalid JSON: {exc}") from exc
        if not isinstance(record, dict):
            raise CalibrationValidationError(f"{path}:{line_number}: sample must be an object")
        if timestamp_field not in record:
            raise CalibrationValidationError(
                f"{path}:{line_number}: missing {timestamp_field}; receive time is not a substitute"
            )
        timestamp = _finite(record[timestamp_field], f"{path}:{line_number}:{timestamp_field}")
        if previous is not None and timestamp <= previous:
            raise CalibrationValidationError(
                f"{path}:{line_number}: timestamps must be strictly increasing"
            )
        _vector(record, "position")
        _finite(record.get("yaw"), f"{path}:{line_number}:yaw")
        _vector(record, "velocity", required=False)
        if require_clock_uncertainty and "clockUncertaintySeconds" not in record:
            raise CalibrationValidationError(
                f"{path}:{line_number}: physical sample requires clockUncertaintySeconds"
            )
        if "clockUncertaintySeconds" in record:
            uncertainty = _finite(record["clockUncertaintySeconds"], "clockUncertaintySeconds")
            if uncertainty < 0:
                raise CalibrationValidationError("clockUncertaintySeconds must be non-negative")
        samples.append(record)
        previous = timestamp
    if len(samples) < 2:
        raise CalibrationValidationError(f"{path}: at least two samples are required")
    return samples


def _angle_delta(a: float, b: float) -> float:
    return (b - a + 180.0) % 360.0 - 180.0


def _interpolate(samples: list[dict[str, Any]], timestamp_field: str, t: float, times: list[float] | None = None) -> dict[str, Any] | None:
    """Interpolate a measured stream at *t*; return None outside its range."""

    if times is None:
        times = [float(row[timestamp_field]) for row in samples]
    if t < times[0] or t > times[-1]:
        return None
    hi = bisect.bisect_left(times, t)
    if hi == 0:
        return samples[0]
    lo = hi - 1
    span = times[hi] - times[lo]
    alpha = 0.0 if span == 0 else (t - times[lo]) / span
    left, right = samples[lo], samples[hi]
    lp, rp = _vector(left, "position"), _vector(right, "position")
    assert lp is not None and rp is not None
    out: dict[str, Any] = {
        "position": {key: lp[i] + alpha * (rp[i] - lp[i]) for i, key in enumerate(("x", "y", "z"))},
        "yaw": float(left["yaw"]) + alpha * _angle_delta(float(left["yaw"]), float(right["yaw"])),
    }
    lv, rv = _vector(left, "velocity", required=False), _vector(right, "velocity", required=False)
    if lv is not None and rv is not None:
        out["velocity"] = {key: lv[i] + alpha * (rv[i] - lv[i]) for i, key in enumerate(("x", "y", "z"))}
    return out


def _percentile(values: list[float], fraction: float) -> float | None:
    if not values:
        return None
    values = sorted(values)
    index = min(len(values) - 1, int(math.ceil(fraction * len(values))) - 1)
    return values[index]


def residuals(
    sim: list[dict[str, Any]],
    sim_time: str,
    real: list[dict[str, Any]],
    real_time: str,
    minimum_overlap_fraction: float = 0.0,
    max_interpolation_gap_seconds: float | None = None,
) -> dict[str, Any]:
    """Compute held-out residuals without extrapolation or temporal re-fitting."""

    position_errors: list[float] = []
    yaw_errors: list[float] = []
    velocity_errors: list[float] = []
    pairs = 0
    interpolation_gaps: list[float] = []
    real_times = [float(row[real_time]) for row in real]
    for row in sim:
        t = _finite(row[sim_time], sim_time)
        measured = _interpolate(real, real_time, t, real_times)
        if measured is None:
            continue
        hi = bisect.bisect_left(real_times, t)
        interpolation_gaps.append(0.0 if hi == 0 else float(real[hi][real_time]) - float(real[hi - 1][real_time]))
        predicted = _vector(row, "position")
        observed = _vector(measured, "position")
        assert predicted is not None and observed is not None
        position_errors.append(math.dist(predicted, observed))
        yaw_errors.append(abs(_angle_delta(float(row["yaw"]), float(measured["yaw"]))))
        pv, ov = _vector(row, "velocity", required=False), _vector(measured, "velocity", required=False)
        if pv is not None and ov is not None:
            velocity_errors.append(math.dist(pv, ov))
        pairs += 1
    if not position_errors:
        raise CalibrationValidationError("simulation and physical streams have no overlapping acquisition times")
    overlap_fraction = pairs / len(sim)
    if overlap_fraction < minimum_overlap_fraction:
        raise CalibrationValidationError(
            f"overlap fraction {overlap_fraction:.6f} is below required {minimum_overlap_fraction:.6f}"
        )
    maximum_gap = max(interpolation_gaps)
    if max_interpolation_gap_seconds is not None and maximum_gap > max_interpolation_gap_seconds:
        raise CalibrationValidationError(
            f"interpolation gap {maximum_gap:.6f}s exceeds required {max_interpolation_gap_seconds:.6f}s"
        )
    final_sim = _vector(sim[-1], "position")
    final_real = _interpolate(real, real_time, _finite(sim[-1][sim_time], sim_time), real_times)
    final_error = None
    if final_real is not None:
        final_vector = _vector(final_real, "position")
        assert final_sim is not None and final_vector is not None
        final_error = math.dist(final_sim, final_vector)
    return {
        "pairedSamples": pairs,
        "overlapFraction": overlap_fraction,
        "simSamples": len(sim),
        "physicalSamples": len(real),
        "positionRmseMeters": math.sqrt(sum(error * error for error in position_errors) / pairs),
        "positionMaeMeters": sum(position_errors) / pairs,
        "positionP95Meters": _percentile(position_errors, 0.95),
        "finalPositionErrorMeters": final_error,
        "yawRmseDegrees": math.sqrt(sum(error * error for error in yaw_errors) / pairs),
        "yawMaeDegrees": sum(yaw_errors) / pairs,
        "yawP95Degrees": _percentile(yaw_errors, 0.95),
        "velocityRmseMetersPerSecond": (
            math.sqrt(sum(error * error for error in velocity_errors) / len(velocity_errors))
            if velocity_errors
            else None
        ),
        "maximumInterpolationGapSeconds": maximum_gap,
        "timestampPolicy": "linear interpolation inside measured acquisition range; no extrapolation or re-registration",
    }


def _factor_value(trial: dict[str, Any], factor: str) -> str:
    factors = trial.get("factors", {})
    value = factors.get(factor, trial.get(factor)) if isinstance(factors, dict) else trial.get(factor)
    if value is None or isinstance(value, (dict, list)):
        raise CalibrationValidationError(f"trial {trial.get('id', '<unknown>')} missing scalar factor {factor}")
    return str(value)


def validate_manifest(manifest_path: Path, require_profile: bool = True) -> dict[str, Any]:
    """Validate and summarize a manifest, including split leakage checks."""

    try:
        manifest = json.loads(manifest_path.read_text())
    except (OSError, json.JSONDecodeError) as exc:
        raise CalibrationValidationError(f"cannot read manifest {manifest_path}: {exc}") from exc
    if manifest.get("schema") != "crane-physical-calibration-dataset-v1":
        raise CalibrationValidationError("manifest schema must be crane-physical-calibration-dataset-v1")
    if manifest.get("timeBasis") != "acquisition":
        raise CalibrationValidationError("manifest timeBasis must be acquisition")
    if manifest.get("coordinateFrame") != "unity-right-up-forward":
        raise CalibrationValidationError("manifest coordinateFrame must declare unity-right-up-forward")
    if manifest.get("sourceKind") not in ("physical", "synthetic-fixture"):
        raise CalibrationValidationError("manifest sourceKind must be physical or synthetic-fixture")
    timing = manifest.get("timing")
    if not isinstance(timing, dict) or not timing.get("clockSource") or not timing.get("synchronizationMethod"):
        raise CalibrationValidationError("manifest timing requires clockSource and synchronizationMethod")
    uncertainty = timing.get("maxClockUncertaintySeconds")
    if isinstance(uncertainty, bool) or not isinstance(uncertainty, (int, float)) or uncertainty < 0:
        raise CalibrationValidationError("timing.maxClockUncertaintySeconds must be non-negative")
    transform = manifest.get("frameTransform")
    if not isinstance(transform, dict) or not transform.get("sourceFrame") or transform.get("targetFrame") != "unity-right-up-forward" or not transform.get("method") or not transform.get("evidence"):
        raise CalibrationValidationError("frameTransform must document source, target, method and evidence")
    profile = manifest.get("calibrationProfile")
    if require_profile and (not isinstance(profile, dict) or not profile.get("id") or not profile.get("version")):
        raise CalibrationValidationError("calibrationProfile provenance is required")
    if profile is not None and not isinstance(profile, dict):
        raise CalibrationValidationError("calibrationProfile must be an object")
    if isinstance(profile, dict):
        if not profile.get("id") or not profile.get("version"):
            raise CalibrationValidationError("calibrationProfile requires id and version")
        artifact_hash = profile.get("parameterArtifactSha256")
        if require_profile and (not isinstance(artifact_hash, str) or len(artifact_hash) != 64):
            raise CalibrationValidationError("calibrationProfile.parameterArtifactSha256 must be a SHA-256 hash")
        if "artifactPath" in profile and (not isinstance(profile["artifactPath"], str) or not profile["artifactPath"]):
            raise CalibrationValidationError("calibrationProfile.artifactPath must be a non-empty path")
        if "fitManifestSha256" in profile and (not isinstance(profile["fitManifestSha256"], str) or len(profile["fitManifestSha256"]) != 64):
            raise CalibrationValidationError("calibrationProfile.fitManifestSha256 must be a SHA-256 hash")
        if "fitTrialIds" in profile and (not isinstance(profile["fitTrialIds"], list) or any(not isinstance(v, str) for v in profile["fitTrialIds"])):
            raise CalibrationValidationError("calibrationProfile.fitTrialIds must be a list of strings")
    acceptance = manifest.get("acceptance")
    if not isinstance(acceptance, dict):
        raise CalibrationValidationError("acceptance requires preregistered overlap and interpolation limits")
    min_overlap = acceptance.get("minimumOverlapFraction")
    max_gap = acceptance.get("maxInterpolationGapSeconds")
    if not isinstance(min_overlap, (int, float)) or not 0 < min_overlap <= 1:
        raise CalibrationValidationError("acceptance.minimumOverlapFraction must be in (0, 1]")
    if not isinstance(max_gap, (int, float)) or max_gap <= 0:
        raise CalibrationValidationError("acceptance.maxInterpolationGapSeconds must be positive")
    for key in ("maxPositionRmseMeters", "maxYawRmseDegrees"):
        if key in acceptance and _finite(acceptance[key], f"acceptance.{key}") < 0:
            raise CalibrationValidationError(f"acceptance.{key} must be non-negative")
    trials = manifest.get("trials")
    if not isinstance(trials, list) or not trials:
        raise CalibrationValidationError("manifest requires a non-empty trials array")
    heldout_factors = manifest.get("heldOutFactors", ["command", "route", "material", "day", "condition"])
    if not heldout_factors or any(not isinstance(factor, str) for factor in heldout_factors):
        raise CalibrationValidationError("heldOutFactors must list one or more factor names")
    ids: set[str] = set()
    paths: set[Path] = set()
    fit, heldout = [], []
    for trial in trials:
        if not isinstance(trial, dict):
            raise CalibrationValidationError("each trial must be an object")
        trial_id = trial.get("id")
        if not isinstance(trial_id, str) or not trial_id or trial_id in ids:
            raise CalibrationValidationError(f"invalid or duplicate trial id: {trial_id!r}")
        ids.add(trial_id)
        split = trial.get("split")
        if split not in ("fit", "heldout"):
            raise CalibrationValidationError(f"trial {trial_id} split must be fit or heldout")
        for factor in heldout_factors:
            _factor_value(trial, factor)
        for key in ("simTimeOffsetSeconds", "physicalTimeOffsetSeconds"):
            if key not in trial:
                raise CalibrationValidationError(f"trial {trial_id} requires explicit {key}")
            _finite(trial[key], f"trial {trial_id} {key}")
        if not isinstance(trial.get("timeOriginEvidence"), str) or not trial["timeOriginEvidence"]:
            raise CalibrationValidationError(f"trial {trial_id} requires timeOriginEvidence")
        for key in ("sim", "real"):
            rel = trial.get(key)
            if not isinstance(rel, str) or not rel:
                raise CalibrationValidationError(f"trial {trial_id} requires {key} path")
            resolved = (manifest_path.parent / rel).resolve()
            if resolved in paths:
                raise CalibrationValidationError(f"sample file is reused across trials: {resolved}")
            paths.add(resolved)
        if trial.get("baseline"):
            if not isinstance(trial["baseline"], str) or not trial["baseline"]:
                raise CalibrationValidationError(f"trial {trial_id} baseline path must be a string")
            resolved = (manifest_path.parent / trial["baseline"]).resolve()
            if resolved in paths:
                raise CalibrationValidationError(f"sample file is reused across trials: {resolved}")
            paths.add(resolved)
        hashes = trial.get("hashes")
        required_hashes = ["sim", "real"] + (["baseline"] if trial.get("baseline") else [])
        if not isinstance(hashes, dict) or any(not isinstance(hashes.get(key), str) or len(hashes[key]) != 64 for key in required_hashes):
            raise CalibrationValidationError(f"trial {trial_id} requires expected SHA-256 hashes for {required_hashes}")
        (heldout if split == "heldout" else fit).append(trial)
    if not fit or not heldout:
        raise CalibrationValidationError("manifest requires both fit and heldout trials")
    for factor in heldout_factors:
        fit_values = {_factor_value(trial, factor) for trial in fit}
        leaked = sorted({_factor_value(trial, factor) for trial in heldout} & fit_values)
        if leaked:
            raise CalibrationValidationError(f"heldout {factor} values overlap fit split: {leaked}")
    return {"manifest": manifest, "fitTrials": fit, "heldoutTrials": heldout, "heldOutFactors": heldout_factors}


def build_report(manifest_path: Path) -> dict[str, Any]:
    checked = validate_manifest(manifest_path)
    manifest = checked["manifest"]
    root = manifest_path.parent.resolve()
    profile = manifest.get("calibrationProfile")
    profile_value = None
    if isinstance(profile, dict) and profile.get("artifactPath"):
        # A generated profile is an immutable, canonical artifact.  Verify both
        # its bytes and its fit-only provenance before comparing any trial.
        from calibration_profile import CalibrationProfileError, manifest_fit_sha256, read_profile

        profile_path = (root / profile["artifactPath"]).resolve()
        expected_hash = profile.get("parameterArtifactSha256")
        if not isinstance(expected_hash, str) or len(expected_hash) != 64:
            raise CalibrationValidationError("calibrationProfile.parameterArtifactSha256 is required with artifactPath")
        try:
            actual_hash = sha256_file(profile_path)
            profile_value = read_profile(profile_path)
        except (OSError, CalibrationProfileError) as exc:
            raise CalibrationValidationError(f"invalid calibration profile artifact: {exc}") from exc
        if actual_hash != expected_hash.lower():
            raise CalibrationValidationError("calibration profile artifact hash does not match manifest")
        provenance = profile_value["fitProvenance"]
        expected_fit_ids = [trial["id"] for trial in checked["fitTrials"]]
        if provenance["trialIds"] != expected_fit_ids:
            raise CalibrationValidationError("calibration profile trialIds do not match manifest fit split")
        if provenance["manifestSha256"] != manifest_fit_sha256(manifest):
            raise CalibrationValidationError("calibration profile manifest provenance does not match dataset identity")
        if profile.get("fitManifestSha256") and profile["fitManifestSha256"] != provenance["manifestSha256"]:
            raise CalibrationValidationError("calibrationProfile.fitManifestSha256 does not match profile provenance")
        if profile_value["sourceKind"] != manifest["sourceKind"]:
            raise CalibrationValidationError("calibration profile sourceKind does not match dataset sourceKind")
    sim_time_default = manifest.get("simTimestampField", "simulationTime")
    real_time_default = manifest.get("realTimestampField", "acquisitionTime")
    report_trials: list[dict[str, Any]] = []
    acceptance = manifest["acceptance"]
    for trial in checked["fitTrials"] + checked["heldoutTrials"]:
        sim_path = (root / trial["sim"]).resolve()
        real_path = (root / trial["real"]).resolve()
        expected_hashes = trial["hashes"]
        try:
            actual_hashes = {"sim": sha256_file(sim_path), "real": sha256_file(real_path)}
        except OSError as exc:
            raise CalibrationValidationError(f"trial {trial['id']} missing input: {exc}") from exc
        if trial.get("baseline"):
            try:
                actual_hashes["baseline"] = sha256_file((root / trial["baseline"]).resolve())
            except OSError as exc:
                raise CalibrationValidationError(f"trial {trial['id']} missing baseline: {exc}") from exc
        if any(actual_hashes[key] != expected_hashes[key] for key in actual_hashes):
            raise CalibrationValidationError(f"trial {trial['id']} input hash does not match manifest")
        sim_time, real_time = trial.get("simTimestampField", sim_time_default), trial.get("realTimestampField", real_time_default)
        sim = read_samples(sim_path, sim_time)
        real = read_samples(real_path, real_time, require_clock_uncertainty=True)
        if any(row["clockUncertaintySeconds"] > manifest["timing"]["maxClockUncertaintySeconds"] for row in real):
            raise CalibrationValidationError(f"trial {trial['id']} exceeds declared clock uncertainty bound")
        sim_offset = float(trial.get("simTimeOffsetSeconds", 0.0))
        real_offset = float(trial.get("physicalTimeOffsetSeconds", 0.0))
        sim = [dict(row, **{sim_time: float(row[sim_time]) - sim_offset}) for row in sim]
        real = [dict(row, **{real_time: float(row[real_time]) - real_offset}) for row in real]
        result = residuals(sim, sim_time, real, real_time, acceptance["minimumOverlapFraction"], acceptance["maxInterpolationGapSeconds"])
        baseline_result = None
        if trial.get("baseline"):
            baseline_path = (root / trial["baseline"]).resolve()
            baseline = read_samples(baseline_path, sim_time)
            baseline = [dict(row, **{sim_time: float(row[sim_time]) - sim_offset}) for row in baseline]
            if [float(row[sim_time]) for row in baseline] != [float(row[sim_time]) for row in sim]:
                raise CalibrationValidationError(
                    f"trial {trial['id']} baseline and candidate must share the same acquisition comparison grid"
                )
            baseline_result = residuals(baseline, sim_time, real, real_time, acceptance["minimumOverlapFraction"], acceptance["maxInterpolationGapSeconds"])
            result["baselinePositionRmseMeters"] = baseline_result["positionRmseMeters"]
            result["candidateBeatsBaseline"] = result["positionRmseMeters"] < baseline_result["positionRmseMeters"]
            result["baselineProvenance"] = {"path": str(baseline_path), "sha256": sha256_file(baseline_path)}
        result.update({"id": trial["id"], "split": trial["split"], "factors": {factor: _factor_value(trial, factor) for factor in checked["heldOutFactors"]},
                       "simProvenance": {"path": str(sim_path), "sha256": sha256_file(sim_path)},
                       "physicalProvenance": {"path": str(real_path), "sha256": actual_hashes["real"]},
                       "timeFields": {"simulation": sim_time, "physical": real_time},
                       "timeOffsetsSeconds": {"simulation": sim_offset, "physical": real_offset},
                       "timeOriginEvidence": trial["timeOriginEvidence"],
                       "clockUncertaintyUpperBoundSeconds": manifest["timing"]["maxClockUncertaintySeconds"]})
        report_trials.append(result)
    heldout_reports = [result for result in report_trials if result["split"] == "heldout"]
    # Residuals are evidence even when no magnitude threshold was preregistered.
    # A pass state is reserved for an explicit acceptance threshold.
    state = "RESIDUALS_COMPUTED"
    max_position = acceptance.get("maxPositionRmseMeters")
    max_yaw = acceptance.get("maxYawRmseDegrees")
    if max_position is not None or max_yaw is not None:
        if max_position is None or max_yaw is None or not all(
            result["positionRmseMeters"] <= max_position and result["yawRmseDegrees"] <= max_yaw
            for result in heldout_reports
        ):
            state = "HELDOUT_RESIDUAL_FAIL"
        else:
            state = "HELDOUT_RESIDUAL_PASS"
    if manifest["sourceKind"] == "synthetic-fixture":
        state = "SYNTHETIC_FIXTURE_RESIDUALS"
    return {
        "schema": "crane-heldout-calibration-report-v1",
        "state": state,
        "manifestProvenance": {"path": str(manifest_path.resolve()), "sha256": sha256_file(manifest_path)},
        "timeBasis": manifest["timeBasis"],
        "coordinateFrame": manifest["coordinateFrame"],
        "heldOutFactors": checked["heldOutFactors"],
        "fitTrialCount": len(checked["fitTrials"]),
        "heldoutTrialCount": len(checked["heldoutTrials"]),
        "trials": report_trials,
        "boundary": "Residual validation only; this report does not fit parameters or establish closed-loop sim-to-real transfer.",
        "sourceKind": manifest["sourceKind"],
        "calibrationProfile": manifest.get("calibrationProfile"),
        "calibrationProfileArtifact": (
            {"path": str((root / profile["artifactPath"]).resolve()), "sha256": sha256_file((root / profile["artifactPath"]).resolve())}
            if isinstance(profile, dict) and profile.get("artifactPath") else None
        ),
    }

