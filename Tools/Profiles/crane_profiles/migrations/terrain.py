"""crane-ground-robot-profile-v1 <-> crane.profile/v2."""
from __future__ import annotations

import copy

from .. import model, units
from . import common as c

V1_SCHEMA = "crane-ground-robot-profile-v1"
SUFFIX_TO_UNIT = {"M": "m", "Kg": "kg", "Nm": "N m"}
SINGLE_SITE = {"wheelSeparationMultiplier"}


def _unit_for(key: str) -> str:
    suf = units.key_suffix(key)
    return SUFFIX_TO_UNIT.get(suf, "1") if suf else "1"


def migrate(v1: dict) -> dict:
    if v1.get("schema") != V1_SCHEMA:
        raise ValueError("not a terrain v1 profile")
    lic = v1["license"]
    upstream = {"license": lic["spdx"], "revision": v1.get("sourceRevision", ""),
                **{k: lic[k] for k in ("verifiedAt", "meshes", "gplCompatible") if k in lic}}
    unc = c.is_uncommitted(v1.get("sourceRevision"))
    if unc:
        upstream["uncommitted"] = True
    label, note = c.normalize_label(v1["evidenceLabel"], uncommitted=unc)
    doc: dict = {
        "schema": model.PROFILE_SCHEMA, "kind": "robot_geometry", "domain": "ground", "id": v1["id"],
        "version": "v1-migrated", "frames": {"body": "robot base, x forward, y left, z up (URDF convention)"},
        "calibrationState": "UNCALIBRATED_PRIOR", "evidenceLabel": label, "sources": {"upstream": upstream},
        "datasets": [], "components": [], "claimScope": {"excluded": []},
        "attributes": {"steering": v1["steering"], "drivenAxles": v1["drivenAxles"],
                       "drivenWheelCount": v1["drivenWheelCount"], **({"notes": v1["notes"]} if "notes" in v1 else {})},
        "migratedFrom": {"schema": V1_SCHEMA, "legacyEvidenceLabel": v1["evidenceLabel"], "labelNote": note},
        "parameters": {},
    }
    for key, entry in v1["parameters"].items():
        ident = c.no_data()
        if key in SINGLE_SITE:
            ident = {"status": "SINGLE_SITE", "reason": "Empirical skid-steer track/wheel scale inflation that is "
                     "valid only for the surface and site it was tuned on; slip and scale are confounded on one slope."}
        prov = "derived" if str(entry["source"]).startswith("derived") else "cited"
        doc["parameters"][key] = c.Q(entry["value"], _unit_for(key), prov=prov, source="upstream",
                                     locator=entry["source"], ident=ident)
    return c.finish(doc)


def to_v1(v2: dict) -> dict:
    a = v2["attributes"]
    return {"schema": V1_SCHEMA, "id": v2["id"], "steering": a["steering"], "drivenAxles": a["drivenAxles"],
            "drivenWheelCount": a["drivenWheelCount"],
            "parameters": {k: {"value": q["value"], "source": q["locator"]} for k, q in v2["parameters"].items()}}
