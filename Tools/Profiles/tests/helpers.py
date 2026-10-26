"""Fixture builders for validator tests."""
from __future__ import annotations

import copy

from crane_profiles import canonical

SHA_A, SHA_B = "a" * 64, "b" * 64
FRAMES = {"body": "FLU", "world": "ENU"}


def q(value, unit, **kw):
    d = {"value": value, "unit": unit, "provenance": kw.pop("prov", "prior")}
    d.update(kw)
    return d


def good_prior() -> dict:
    """A valid UNCALIBRATED_PRIOR profile exercising scalar, vector, cited and ident fields."""
    doc = {
        "schema": "crane.profile/v2", "kind": "vehicle_dynamics", "domain": "test", "id": "unit-test-prior",
        "version": "1", "frames": dict(FRAMES), "calibrationState": "UNCALIBRATED_PRIOR", "evidenceLabel": "IMPLEMENTED",
        "sources": {"vendor": {"license": "CC-BY-4.0", "citation": "x"}},
        "datasets": [], "components": [], "claimScope": {"excluded": []},
        "dynamics": {
            "massKg": q(1.5, "kg", prov="cited", source="vendor", locator="table 1",
                        identifiability={"status": "COMBINATION_ONLY", "with": ["dynamics.addedMassKg"], "reason": "m + A only"}),
            "addedMassKg": q(0.2, "kg", prov="cited", source="vendor", locator="table 2",
                             identifiability={"status": "COMBINATION_ONLY", "with": ["dynamics.massKg"], "reason": "m + A only"}),
            "dragLinear": q([0.1, 0.1, 0.2], "N s/m", frame="body"),
            "tauUpS": q(0.05, "s", identifiability={"status": "CONFOUNDED_WITH", "with": ["timing.gyroLatencyS"], "reason": "rate response"}),
        },
        "timing": {"gyroLatencyS": q(None, "s", identifiability={"status": "ABSTAIN", "reason": "never measured"})},
    }
    return canonical.seal(doc)


def good_physical() -> dict:
    """A valid PHYSICALLY_CALIBRATED profile (physical fit + held-out datasets, fitted CI, sim-truth excluded)."""
    doc = good_prior()
    doc["calibrationState"] = "PHYSICALLY_CALIBRATED"
    doc["evidenceLabel"] = "PHYSICALLY_CALIBRATED"
    doc["datasets"] = [
        {"id": "fit1", "origin": "physical", "split": "fit", "sha256": SHA_A, "sites": ["lot-a"], "sessions": ["s1"], "runIds": ["r1"]},
        {"id": "held1", "origin": "physical", "split": "heldout", "sha256": SHA_B, "sites": ["lot-b"], "sessions": ["s2"], "runIds": ["r2"]},
    ]
    doc["dynamics"]["motorLagSeconds"] = q(0.12, "s", prov="fitted", datasets=["fit1"],
        uncertainty={"kind": "ci", "level": 0.95, "lo": 0.10, "hi": 0.14},
        identifiability={"status": "IDENTIFIED"})
    return canonical.seal(doc)


def mutate(doc: dict, fn, reseal: bool = True) -> dict:
    d = copy.deepcopy(doc)
    fn(d)
    return canonical.seal(d) if reseal else d
