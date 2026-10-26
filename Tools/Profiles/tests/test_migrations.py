"""Parity tests: each migrated v2 profile, exported back to v1, must drive the ORIGINAL loader to identical output."""
from __future__ import annotations

import copy
import dataclasses
import json
from pathlib import Path

import numpy as np
import pytest

from crane_profiles import canonical, cli, validate
from crane_profiles.migrations import aerial, campus, manifest, terrain, underwater

TOOLS = Path(__file__).resolve().parents[2]
UW = sorted((TOOLS / "Underwater" / "profiles").glob("*.json"))
AE = sorted((TOOLS / "Aerial" / "profiles").glob("*.json"))
TE = sorted((TOOLS / "Terrain" / "profiles").glob("*.json"))


def roundtrip(v2: dict, tmp_path) -> dict:
    """Write the v2 doc to disk and read it back (exercises JSON float round-trip)."""
    p = tmp_path / "v2.json"
    canonical.write_sealed(v2, p)
    return json.loads(p.read_text(encoding="utf-8"))


def deep_equal(a, b) -> bool:
    if dataclasses.is_dataclass(a):
        return type(a) is type(b) and all(deep_equal(getattr(a, f.name), getattr(b, f.name)) for f in dataclasses.fields(a))
    if isinstance(a, np.ndarray) or isinstance(b, np.ndarray):
        a, b = np.asarray(a), np.asarray(b)
        return a.shape == b.shape and a.tobytes() == b.tobytes()
    if isinstance(a, (list, tuple)):
        return len(a) == len(b) and all(deep_equal(x, y) for x, y in zip(a, b))
    return a == b


def test_source_sets_nonempty():
    assert len(UW) == 1 and len(AE) == 4 and len(TE) == 3


# ----------------------------------------------------------------------------------------- underwater
def test_bluerov2_params_bit_identical(tmp_path):
    import vehicle_profile as vp
    src = UW[0]
    v1 = json.loads(src.read_text())
    v2 = roundtrip(underwater.migrate(v1), tmp_path)
    exported = tmp_path / "exported_v1.json"
    exported.write_text(json.dumps(underwater.to_v1(v2)))
    a, b = vp.to_params(vp.load(src)), vp.to_params(vp.load(exported))
    for f in dataclasses.fields(a):
        x, y = getattr(a, f.name), getattr(b, f.name)
        assert deep_equal(x, y), f.name
    assert np.array(a.added_mass).tobytes() == np.array(b.added_mass).tobytes()
    ta, tb = vp.to_thrusters(vp.load(src)), vp.to_thrusters(vp.load(exported))
    assert len(ta) == len(tb) == 8 and all(deep_equal(x, y) for x, y in zip(ta, tb))


def test_bluerov2_v2_validates_with_design_statuses(tmp_path):
    v2 = underwater.migrate(json.loads(UW[0].read_text()))
    rep = validate(v2)
    assert rep.ok, rep.errors
    assert (v2["calibrationState"], v2["evidenceLabel"]) == ("UNCALIBRATED_PRIOR", "UNKNOWN")
    m = v2["rigidBody"]["massKg"]["identifiability"]
    assert m["status"] == "COMBINATION_ONLY" and "hydrodynamics.addedMass.XudotKg" in m["with"]
    assert v2["hydrodynamics"]["addedMass"]["ZwdotKg"]["identifiability"]["status"] == "COMBINATION_ONLY"
    assert v2["hydrodynamics"]["addedMass"]["KpdotKgM2"]["identifiability"]["status"] == "NO_PHYSICAL_DATA"
    assert v2["hydrodynamics"]["addedMass"]["XudotKg"]["alternatives"][0]["source"] == "wu2018"
    assert v2["rigidBody"]["rGMeters"]["frame"] == "body"


def test_bluerov2_v1_roundtrip_values_exact():
    v1 = json.loads(UW[0].read_text())
    back = underwater.to_v1(underwater.migrate(v1))
    for k in ("rigidBody", "hydrodynamics", "thrusters"):
        for path, node in _leaves(v1[k], k):
            assert _get(back, path) == node, path


def _leaves(d, prefix):
    for k, v in d.items():
        if isinstance(v, dict) and "value" not in v:
            yield from _leaves(v, f"{prefix}.{k}")
        elif isinstance(v, dict):
            yield f"{prefix}.{k}", v


def _get(d, path):
    for p in path.split("."):
        d = d[p]
    return d


# ----------------------------------------------------------------------------------------- aerial
@pytest.mark.parametrize("src", AE, ids=lambda p: p.stem)
def test_aerial_load_vehicle_parity(src, tmp_path):
    import aerial_profiles as ap
    v1 = json.loads(src.read_text())
    v2 = roundtrip(aerial.migrate(v1), tmp_path)
    back = aerial.to_v1(v2)
    exported = tmp_path / "exported.json"
    exported.write_text(json.dumps(back))
    for battery in (True, False):
        for ground in (False, True):
            a = ap.load_vehicle(src, with_battery=battery, ground_effect=ground)
            b = ap.load_vehicle(exported, with_battery=battery, ground_effect=ground)
            assert deep_equal(a, b)
    assert back["parameters"] == v1["parameters"]
    assert back["rotors"] == v1["rotors"] and back["sources"] == v1["sources"]


@pytest.mark.parametrize("src", AE, ids=lambda p: p.stem)
def test_aerial_v2_valid_and_statuses(src):
    v2 = aerial.migrate(json.loads(src.read_text()))
    rep = validate(v2)
    assert rep.ok, rep.errors
    assert v2["calibrationState"] == "UNCALIBRATED_PRIOR"
    for key in ("tauUpS", "tauDownS"):
        idf = v2["propulsion"][key]["identifiability"]
        assert idf["status"] == "CONFOUNDED_WITH" and idf["with"] == ["timing.gyroLatencyS"]
    assert v2["timing"]["gyroLatencyS"]["identifiability"]["status"] == "ABSTAIN"
    assert v2["timing"]["gyroLatencyS"]["value"] is None


def test_aerial_uncommitted_reference_quad_capped_to_implemented():
    src = next(p for p in AE if "crane_reference_quad" in p.name)
    v1 = json.loads(src.read_text())
    assert v1["evidenceLabel"].startswith("LIVE_SIM_VALIDATED")
    v2 = aerial.migrate(v1)
    assert v2["evidenceLabel"] == "IMPLEMENTED" and v2["sources"]["CRANE"]["uncommitted"] is True
    assert "R11" in v2["migratedFrom"]["labelNote"]
    forged = copy.deepcopy(v2)
    forged["evidenceLabel"] = "LIVE_SIM_VALIDATED"
    assert "R11" in validate(canonical.seal(forged)).rules()


def test_aerial_units_normalised_and_dimension_checked():
    v2 = aerial.migrate(json.loads(next(p for p in AE if "crane_reference_quad" in p.name).read_text()))
    assert v2["propulsion"]["kf"]["unit"] == "N/norm^2" and v2["propulsion"]["omegaMax"]["unit"] == "norm"
    assert v2["propulsion"]["rotorDrag"]["unit"] == "1"


# ----------------------------------------------------------------------------------------- terrain
@pytest.mark.parametrize("src", TE, ids=lambda p: p.stem)
def test_terrain_analytic_limits_parity(src, tmp_path, monkeypatch):
    import robot_profiles as rp
    v1 = json.loads(src.read_text())
    v2 = roundtrip(terrain.migrate(v1), tmp_path)
    back = terrain.to_v1(v2)
    d = tmp_path / "profiles"
    d.mkdir()
    (d / f"{src.stem}.json").write_text(json.dumps(back))
    monkeypatch.setattr(rp, "PROFILE_DIR", d)
    reloaded = rp.load(src.stem)
    orig = v1  # the original v1 document, exactly as the source loader parses it
    for mu in (0.1, 0.35, 0.5, 0.9, 1.0, 1.3):
        for crr in (0.0, 0.02, 0.1):
            assert rp.analytic_limits(reloaded, mu, crr) == rp.analytic_limits(orig, mu, crr)


@pytest.mark.parametrize("src", TE, ids=lambda p: p.stem)
def test_terrain_v2_valid_and_labels_fixed(src):
    v1 = json.loads(src.read_text())
    v2 = terrain.migrate(v1)
    assert validate(v2).ok
    assert v2["evidenceLabel"] == "IMPLEMENTED" and v2["calibrationState"] == "UNCALIBRATED_PRIOR"
    assert v2["migratedFrom"]["legacyEvidenceLabel"] == v1["evidenceLabel"]
    assert v2["parameters"]["wheelbaseM"]["unit"] == "m"
    assert v2["parameters"]["totalMassKg"]["unit"] == "kg"
    forged = copy.deepcopy(v2)
    forged["evidenceLabel"] = v1["evidenceLabel"]
    assert "R8" in validate(canonical.seal(forged)).rules()


def test_terrain_wheel_scale_is_single_site():
    v2 = terrain.migrate(json.loads(next(p for p in TE if "husky" in p.name).read_text()))
    assert v2["parameters"]["wheelSeparationMultiplier"]["identifiability"]["status"] == "SINGLE_SITE"
    assert v2["parameters"]["wheelbaseM"]["identifiability"]["status"] == "NO_PHYSICAL_DATA"


def test_terrain_turtlebot_uncommitted_source_flagged():
    v2 = terrain.migrate(json.loads(next(p for p in TE if "turtlebot" in p.name).read_text()))
    assert v2["sources"]["upstream"]["uncommitted"] is True


# ----------------------------------------------------------------------------------------- campus
def campus_v1(kind="synthetic-fixture", sha="ab" * 32):
    return {"schema": "crane-calibration-profile-v1", "id": "turtlebot-campus", "version": "2026-10-21", "sourceKind": kind,
            "parameters": {"massKg": 1.55, "motorLagSeconds": 0.15, "accelerationMps2": 0.6,
                           "angularAccelerationRadPerSec2": 1.2, "maxMotorTorqueNm": 0.35, "leftWheelScale": 1.0,
                           "rightWheelScale": 1.01,
                           "lidar": {"sigmaMeters": 0.005, "dropoutProbability": 0.02, "latencyScans": 1, "materialClass": "reflective"}},
            "fitProvenance": {"trialIds": ["t1", "t3", "t2"], "manifestSha256": sha, "fitNote": "median of ünicode note"}}


@pytest.mark.parametrize("kind", ["synthetic-fixture", "physical"])
@pytest.mark.parametrize("sha", ["ab" * 32, "AB" * 32])
def test_campus_v2_to_v1_is_byte_identical(kind, sha, tmp_path):
    import calibration_profile as cp
    v1 = campus_v1(kind, sha)
    cp.validate_profile(v1)
    v2 = roundtrip(campus.migrate(v1), tmp_path)
    back = campus.to_v1(v2)
    cp.validate_profile(back)
    assert campus.v1_canonical_bytes(back) == cp.canonical_bytes(v1) == campus.v1_canonical_bytes(v1)
    out = tmp_path / "p.json"
    cp.write_profile(back, out)
    assert out.read_bytes() == cp.canonical_bytes(v1)
    assert cp.read_profile(out) == v1
    assert cp.profile_sha256(back) == cp.profile_sha256(v1)


def test_campus_integer_and_float_types_preserved(tmp_path):
    v1 = campus_v1()
    v1["parameters"]["lidar"]["latencyScans"] = 2
    v1["parameters"]["massKg"] = 2
    back = campus.to_v1(roundtrip(campus.migrate(v1), tmp_path))
    assert type(back["parameters"]["lidar"]["latencyScans"]) is int and type(back["parameters"]["massKg"]) is int


def test_campus_states():
    fx = campus.migrate(campus_v1("synthetic-fixture"))
    ph = campus.migrate(campus_v1("physical"))
    assert validate(fx).ok and validate(ph).ok
    assert (fx["calibrationState"], fx["evidenceLabel"]) == ("FIXTURE_ONLY", "IMPLEMENTED")
    assert (ph["calibrationState"], ph["evidenceLabel"]) == ("UNCALIBRATED_PRIOR", "IMPLEMENTED")
    assert fx["actuator"]["leftWheelScale"]["identifiability"]["status"] == "SINGLE_SITE"
    assert fx["dynamics"]["massKg"]["provenance"] == "fixtureOnly"
    up = copy.deepcopy(fx)
    up["evidenceLabel"] = "LIVE_SIM_VALIDATED"
    assert "R7" in validate(canonical.seal(up)).rules()
    pc = copy.deepcopy(ph)
    pc.update(calibrationState="PHYSICALLY_CALIBRATED", evidenceLabel="PHYSICALLY_CALIBRATED")
    assert "R5" in validate(canonical.seal(pc)).rules()  # v1 'physical' alone never reaches PHYSICALLY_CALIBRATED


# ----------------------------------------------------------------------------------------- heldout manifest
def manifest_v1(share=None, with_site=True):
    def trial(i, split, site, day):
        t = {"id": i, "split": split, "sim": f"{i}.sim.jsonl", "real": f"{i}.real.jsonl",
             "hashes": {"sim": "a" * 64, "real": "b" * 64}, "simTimeOffsetSeconds": 0.0, "physicalTimeOffsetSeconds": 0.0,
             "timeOriginEvidence": "sync pulse", "factors": {"day": day, "command": f"c-{i}"}}
        if with_site:
            t["factors"]["site"] = site
        return t
    return {"schema": "crane-physical-calibration-dataset-v1", "timeBasis": "acquisition", "coordinateFrame": "unity-right-up-forward",
            "sourceKind": "physical", "timing": {"clockSource": "ptp", "synchronizationMethod": "pps", "maxClockUncertaintySeconds": 0.001},
            "frameTransform": {"sourceFrame": "map", "targetFrame": "unity-right-up-forward", "method": "m", "evidence": "e"},
            "calibrationProfile": {"id": "p", "version": "1", "parameterArtifactSha256": "c" * 64},
            "acceptance": {"minimumOverlapFraction": 0.8, "maxInterpolationGapSeconds": 0.1},
            "heldOutFactors": ["day", "command"] + (["site"] if with_site else []),
            "trials": [trial("a", "fit", "lot-a", "d1"), trial("b", "heldout", share or "lot-b", "d2")]}


def test_manifest_migration_parity_with_v1_validator(tmp_path):
    from heldout_calibration import CalibrationValidationError, validate_manifest
    for share in (None, "lot-a"):
        v1 = manifest_v1(share)
        p = tmp_path / f"m{share}.json"
        p.write_text(json.dumps(v1))
        v2 = manifest.migrate(v1)
        rep = validate(v2)
        if share is None:
            validate_manifest(p)
            assert rep.ok and rep.info["independence"] == "VERIFIED"
        else:
            with pytest.raises(CalibrationValidationError):
                validate_manifest(p)
            assert not rep.ok and "R12" in rep.rules()


def test_manifest_without_site_is_unverified_not_verified():
    rep = validate(manifest.migrate(manifest_v1(with_site=False)))
    assert rep.ok and rep.info["independence"] == "UNVERIFIED" and rep.warnings


def test_manifest_hash_tamper_detected():
    v2 = manifest.migrate(manifest_v1())
    v2["trials"][0]["split"] = "heldout"
    assert "R10" in validate(v2).rules()


# ----------------------------------------------------------------------------------------- on-disk artifacts
def test_migrated_dir_is_current_and_valid(tmp_path):
    cli.migrate_all(tmp_path)
    fresh = {p.name: p.read_bytes() for p in tmp_path.glob("*.json")}
    ondisk = {p.name: p.read_bytes() for p in (Path(cli.__file__).resolve().parents[1] / "migrated").glob("*.json")}
    assert fresh == ondisk and len(fresh) == 8
    for name, raw in ondisk.items():
        rep = validate(json.loads(raw))
        assert rep.ok, (name, rep.errors)
        assert rep.info["calibrationState"] == "UNCALIBRATED_PRIOR"
        assert rep.info["evidenceLabel"] in ("UNKNOWN", "IMPLEMENTED")
