"""crane-aerial-vehicle-profile-v1 <-> crane.profile/v2."""
from __future__ import annotations

import copy

from .. import model
from . import common as c

V1_SCHEMA = "crane-aerial-vehicle-profile-v1"
# v1 key -> (v2 path, frame)
KEYS = {
    "mass": ("dynamics.massKg", None), "inertia": ("dynamics.inertiaKgM2", "body"),
    "kf": ("propulsion.kf", None), "km": ("propulsion.kmM", None),
    "tau_up": ("propulsion.tauUpS", None), "tau_down": ("propulsion.tauDownS", None),
    "omega_max": ("propulsion.omegaMax", None), "rotor_drag": ("propulsion.rotorDrag", None),
    "rolling_moment": ("propulsion.rollingMoment", None), "rotor_radius": ("propulsion.rotorRadiusM", None),
    "drag_linear": ("aero.dragLinear", "body"), "drag_quadratic": ("aero.dragQuadratic", None),
    "battery_v_full": ("battery.vFullV", None), "battery_v_empty": ("battery.vEmptyV", None),
    "battery_v_nominal": ("battery.vNominalV", None), "battery_resistance": ("battery.resistanceOhm", None),
    "battery_capacity_ah": ("battery.capacityAh", None), "battery_efficiency": ("battery.efficiency", None),
}
UNIT_FIX = {"N per normalized speed^2": "N/norm^2", "normalized": "norm", "-": "1"}
STATUS_PROV = {"MEASURED_PUBLISHED": "cited", "VENDOR_SPEC": "cited", "VENDOR_SIM_MODEL_CITING_SYSID": "cited",
               "SIMULATOR_DEFAULT": "prior", "DERIVED": "derived", "ASSUMED": "prior", "UNKNOWN": "prior",
               "NOT_REPRESENTABLE": "prior"}
GYRO = "timing.gyroLatencyS"
_STD = ("source", "locator", "status", "unit", "value", "alternatives")


def _unit(u: str) -> str:
    return UNIT_FIX.get(u, u)


def migrate(v1: dict) -> dict:
    if v1.get("schema") != V1_SCHEMA:
        raise ValueError("not an aerial v1 profile")
    sources = copy.deepcopy(v1["sources"])
    uncommitted = False
    for s in sources.values():
        if c.is_uncommitted(s.get("commit")):
            s["uncommitted"] = True
            uncommitted = True
    label, note = c.normalize_label(v1["evidenceLabel"], uncommitted=uncommitted)
    doc: dict = {
        "schema": model.PROFILE_SCHEMA, "kind": "vehicle_dynamics", "domain": "aerial", "id": v1["id"],
        "version": "v1-migrated", "frames": copy.deepcopy(v1["frames"]), "calibrationState": "UNCALIBRATED_PRIOR",
        "evidenceLabel": label, "sources": sources, "datasets": [], "components": [], "claimScope": {"excluded": []},
        "attributes": {k: copy.deepcopy(v1[k]) for k in ("description", "measurements", "sensors") if k in v1},
        "notes": list(v1.get("notes", [])),
        "migratedFrom": {"schema": V1_SCHEMA, "legacyEvidenceLabel": v1["evidenceLabel"], "labelNote": note},
        "structures": {"rotors": {"frame": "body", "data": copy.deepcopy(v1["rotors"]), "provenance": "cited"}},
    }
    for key, entry in v1["parameters"].items():
        path, frame = KEYS[key]
        ident = c.no_data()
        if key in ("tau_up", "tau_down"):
            ident = {"status": "CONFOUNDED_WITH", "with": [GYRO],
                     "reason": "Motor time constant is not separable from IMU/gyro latency in rate-response data "
                               "(see Tools/Aerial/aerial_identifiability.py)."}
        legacy = {"v1Key": key, "unit": entry["unit"], "status": entry["status"],
                  "extra": {k: copy.deepcopy(v) for k, v in entry.items() if k not in _STD}}
        alts = None
        if entry.get("alternatives"):
            legacy["alternatives"] = copy.deepcopy(entry["alternatives"])
            alts = [{k: _unit(a[k]) if k == "unit" else a[k] for k in ("value", "unit", "source", "locator") if k in a}
                    for a in entry["alternatives"]]
        q = c.Q(entry["value"], _unit(entry["unit"]), prov=STATUS_PROV[entry["status"]], source=entry.get("source"),
                locator=entry["locator"], frame=frame, ident=ident, alternatives=alts, legacy=legacy)
        c.set_path(doc, path, q)
    c.set_path(doc, GYRO, c.Q(None, "s", prov="prior", ident={
        "status": "ABSTAIN", "reason": "No gyro/IMU latency is recorded in the v1 profile; this quantity is the "
                                       "anchor for CONFOUNDED_WITH on the motor time constants, not a value."}))
    return c.finish(doc)


def to_v1(v2: dict) -> dict:
    out: dict = {"schema": V1_SCHEMA, "id": v2["id"], "evidenceLabel": v2["migratedFrom"]["legacyEvidenceLabel"],
                 "frames": copy.deepcopy(v2["frames"]), "sources": copy.deepcopy(v2["sources"]),
                 "parameters": {}, "rotors": copy.deepcopy(v2["structures"]["rotors"]["data"])}
    for s in out["sources"].values():
        s.pop("uncommitted", None)
    for key, (path, _f) in KEYS.items():
        try:
            q = c.get_path(v2, path)
        except KeyError:
            continue
        lg = q["legacy"]
        entry = {"value": q["value"], "unit": lg["unit"], "source": q.get("source"), "locator": q["locator"],
                 "status": lg["status"], **copy.deepcopy(lg.get("extra", {}))}
        if "alternatives" in lg:
            entry["alternatives"] = copy.deepcopy(lg["alternatives"])
        out["parameters"][key] = entry
    return out
