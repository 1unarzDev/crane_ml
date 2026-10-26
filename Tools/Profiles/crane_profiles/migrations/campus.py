"""crane-calibration-profile-v1 <-> crane.profile/v2 (lossless: v2 -> v1 gives the byte-identical canonical form)."""
from __future__ import annotations

import copy
import json

from .. import model
from . import common as c

V1_SCHEMA = "crane-calibration-profile-v1"
PARAMS = [  # v1 key, v2 path, unit
    ("massKg", "dynamics.massKg", "kg"), ("accelerationMps2", "dynamics.accelerationMps2", "m/s^2"),
    ("angularAccelerationRadPerSec2", "dynamics.angularAccelerationRadPerSec2", "rad/s^2"),
    ("motorLagSeconds", "actuator.motorLagSeconds", "s"), ("maxMotorTorqueNm", "actuator.maxMotorTorqueNm", "N m"),
    ("leftWheelScale", "actuator.leftWheelScale", "1"), ("rightWheelScale", "actuator.rightWheelScale", "1"),
]
LIDAR = [("sigmaMeters", "m"), ("dropoutProbability", "1"), ("latencyScans", "1")]
SINGLE_SITE = {"leftWheelScale", "rightWheelScale"}


def v1_canonical_bytes(profile: dict) -> bytes:
    """Byte-for-byte the form written by Tools/Campus/calibration_profile.canonical_bytes."""
    return (json.dumps(profile, sort_keys=True, separators=(",", ":"), ensure_ascii=False) + "\n").encode("utf-8")


def migrate(v1: dict) -> dict:
    if v1.get("schema") != V1_SCHEMA:
        raise ValueError("not a calibration profile v1")
    fixture = v1["sourceKind"] == "synthetic-fixture"
    fp = v1["fitProvenance"]
    prov = "fixtureOnly" if fixture else "derived"
    def ident(key: str) -> dict:
        if key in SINGLE_SITE:
            return {"status": "SINGLE_SITE", "reason": "Wheel scale fitted on one site/session; v1 profile carries no "
                    "cross-site evidence and no slope diversity to separate scale from slip."}
        return c.no_data("v1 profile has no CI or identifiability analysis; medians of profileObservation records.")
    doc: dict = {
        "schema": model.PROFILE_SCHEMA, "kind": "robot_calibration", "domain": "campus", "id": v1["id"],
        "version": v1["version"], "frames": {"body": "robot base, x forward, y left, z up", "world": "unity-right-up-forward"},
        "calibrationState": "FIXTURE_ONLY" if fixture else "UNCALIBRATED_PRIOR",
        "evidenceLabel": "IMPLEMENTED",
        "sources": {"crane_fit": {"license": "GPL-3.0 (this repository)", "tool": "Tools/Campus/calibration_profile.py"}},
        "datasets": [{
            "id": "fit-manifest", "origin": "synthetic-fixture" if fixture else "physical",
            "sha256": fp["manifestSha256"].lower(), **({"sha256AsWritten": fp["manifestSha256"]} if fp["manifestSha256"] != fp["manifestSha256"].lower() else {}),
            "split": "fit", "trialIds": list(fp["trialIds"]), "sites": [], "sessions": [], "note": fp["fitNote"]}],
        "components": [], "claimScope": {"excluded": []},
        "attributes": {"lidarMaterialClass": v1["parameters"]["lidar"]["materialClass"]},
        "notes": ([] if fixture else ["v1 sourceKind 'physical' is an unverifiable claim: no held-out dataset, CI or "
                                      "site/session list is present, so the state is UNCALIBRATED_PRIOR, not PHYSICALLY_CALIBRATED."]),
        "migratedFrom": {"schema": V1_SCHEMA, "sourceKind": v1["sourceKind"]},
    }
    for key, path, unit in PARAMS:
        c.set_path(doc, path, c.Q(v1["parameters"][key], unit, prov=prov, ident=ident(key), legacy={"v1Key": key},
                                  locator="v1 median of profileObservation" if prov == "derived" else None))
    for key, unit in LIDAR:
        c.set_path(doc, f"sensors.lidar.{key}", c.Q(v1["parameters"]["lidar"][key], unit, prov=prov, ident=ident(key),
                                                    legacy={"v1Key": f"lidar.{key}"}))
    return c.finish(doc)


def to_v1(v2: dict) -> dict:
    ds = v2["datasets"][0]
    params = {key: c.get_path(v2, path)["value"] for key, path, _u in PARAMS}
    params["lidar"] = {key: c.get_path(v2, f"sensors.lidar.{key}")["value"] for key, _u in LIDAR}
    params["lidar"]["materialClass"] = v2["attributes"]["lidarMaterialClass"]
    return {"schema": V1_SCHEMA, "id": v2["id"], "version": v2["version"], "sourceKind": v2["migratedFrom"]["sourceKind"],
            "parameters": params,
            "fitProvenance": {"trialIds": list(ds["trialIds"]), "manifestSha256": ds.get("sha256AsWritten", ds["sha256"]),
                              "fitNote": ds["note"]}}
