"""crane.underwater.vehicle_profile/v1 <-> crane.profile/v2."""
from __future__ import annotations

import copy

from .. import model
from . import common as c

V1_SCHEMA = "crane.underwater.vehicle_profile/v1"
DOFS = ("u", "v", "w", "p", "q", "r")
ADDED = dict(zip(DOFS, ("XudotKg", "YvdotKg", "ZwdotKg", "KpdotKgM2", "MqdotKgM2", "NrdotKgM2")))
LIN = dict(zip(DOFS, ("Xu", "Yv", "Zw", "Kp", "Mq", "Nr")))
QUAD = dict(zip(DOFS, ("Xuu", "Yvv", "Zww", "Kpp", "Mqq", "Nrr")))
_TRANS = ("u", "v", "w")

# (v1 path, v2 path, unit, frame)
ROWS: list = [
    ("rigidBody.massKg", "rigidBody.massKg", "kg", None),
    ("rigidBody.gravityMps2", "rigidBody.gravityMps2", "m/s^2", None),
    ("rigidBody.weightN", "rigidBody.weightN", "N", None),
    ("rigidBody.displacedVolumeM3", "rigidBody.displacedVolumeM3", "m^3", None),
    ("rigidBody.waterDensityKgM3", "rigidBody.waterDensityKgM3", "kg/m^3", None),
    ("rigidBody.buoyancyN", "rigidBody.buoyancyN", "N", None),
    ("rigidBody.rGMeters", "rigidBody.rGMeters", "m", "body"),
    ("rigidBody.rBMeters", "rigidBody.rBMeters", "m", "body"),
]
for _k, _n in (("Ixx", "IxxKgM2"), ("Iyy", "IyyKgM2"), ("Izz", "IzzKgM2")):
    ROWS.append((f"rigidBody.inertiaCgKgM2.{_k}", f"rigidBody.inertiaCg.{_n}", "kg m^2", None))
for _d in DOFS:
    t = _d in _TRANS
    ROWS.append((f"hydrodynamics.addedMass.{_d}", f"hydrodynamics.addedMass.{ADDED[_d]}", "kg" if t else "kg m^2", None))
    ROWS.append((f"hydrodynamics.linearDamping.{_d}", f"hydrodynamics.linearDamping.{LIN[_d]}", "N s/m" if t else "N m s/rad", None))
    ROWS.append((f"hydrodynamics.quadraticDamping.{_d}", f"hydrodynamics.quadraticDamping.{QUAD[_d]}",
                 "N s^2/m^2" if t else "N m s^2/rad^2", None))
ROWS += [
    ("thrusters.maxForwardN", "thrusters.maxForwardN", "N", None),
    ("thrusters.maxReverseN", "thrusters.maxReverseN", "N", None),
    ("thrusters.timeConstantS", "thrusters.timeConstantS", "s", None),
    ("tether.massPerLengthKgM", "tether.massPerLengthKgPerM", "kg/m", None),
    ("tether.diameterM", "tether.diameterM", "m", None),
    ("tether.normalDragCoefficient", "tether.normalDragCoefficient", "1", None),
    ("tether.tangentialDragCoefficient", "tether.tangentialDragCoefficient", "1", None),
    ("tether.internalDampingNsM", "tether.internalDampingNsPerM", "N s/m", None),
    ("tether.youngsModulusNM2", "tether.youngsModulusPa", "Pa", None),
    ("publishedPerformance.maxSurgeSpeedMps", "publishedPerformance.maxSurgeSpeedMps", "m/s", None),
    ("publishedPerformance.maxSwaySpeedMps", "publishedPerformance.maxSwaySpeedMps", "m/s", None),
    ("publishedPerformance.maxHeaveSpeedMps", "publishedPerformance.maxHeaveSpeedMps", "m/s", None),
    ("publishedPerformance.maxSurgeForceN", "publishedPerformance.maxSurgeForceN", "N", None),
    ("publishedPerformance.maxHeaveForceN", "publishedPerformance.maxHeaveForceN", "N", None),
    ("publishedPerformance.dimensionsLxHxWM", "publishedPerformance.dimensionsLxHxWM", "m", "body"),
]
STRUCTS = {"thrusters.layout": "thrusterLayout", "thrusters.thrustCurve": "thrustCurve",
           "thrusters.dynamicsTransferFunction": "thrusterTransferFunction"}

# alternatePrior (Wu 2018) -> alternatives on the matching v2 quantity
def _alt_map() -> dict:
    m = {"massKg": ["rigidBody.massKg"], "weightN": ["rigidBody.weightN"], "buoyancyN": ["rigidBody.buoyancyN"],
         "rGMeters": ["rigidBody.rGMeters"], "rBMeters": ["rigidBody.rBMeters"]}
    return m


def _get(d: dict, dotted: str):
    cur = d
    for p in dotted.split("."):
        cur = cur[p]
    return cur


def _prov(source: str) -> str:
    return "derived" if source == "crane_derived" else "cited"


def migrate(v1: dict) -> dict:
    if v1.get("schema") != V1_SCHEMA:
        raise ValueError("not an underwater v1 profile")
    sources = copy.deepcopy(v1["sources"])
    paths = {r[1] for r in ROWS}
    combo_added = [f"hydrodynamics.addedMass.{ADDED[d]}" for d in _TRANS]
    doc: dict = {
        "schema": model.PROFILE_SCHEMA, "kind": "vehicle_dynamics", "domain": "underwater",
        "id": v1["vehicle"], "version": "v1-migrated",
        "frames": {"world": v1["frames"]["world"], "body": v1["frames"]["body"]},
        "attributes": {"signConvention": v1["frames"].get("signConvention", ""), "vehicle": v1["vehicle"],
                       "thrusterModel": v1["thrusters"].get("model", "")},
        "notes": [v1.get("evidenceNote", "")],
        "sources": sources, "datasets": [], "components": [], "claimScope": {"excluded": []},
        "structures": {},
    }
    uncommitted = False
    for sid, s in sources.items():
        if c.is_uncommitted(s.get("commit"), s.get("revision")):
            s["uncommitted"] = True
            uncommitted = True
    label, note = c.normalize_label(v1["evidenceLabel"], uncommitted=uncommitted)
    doc["evidenceLabel"] = label
    doc["calibrationState"] = "UNCALIBRATED_PRIOR"
    doc["migratedFrom"] = {"schema": V1_SCHEMA, "legacyEvidenceLabel": v1["evidenceLabel"], "labelNote": note}
    for v1p, v2p, unit, frame in ROWS:
        node = _get(v1, v1p)
        ident = c.no_data()
        if v2p == "rigidBody.massKg":
            ident = {"status": "COMBINATION_ONLY", "with": combo_added,
                     "reason": "Only m + A_ii (effective inertia) is observable from force-to-acceleration data; "
                               "mass and added mass are not separately identifiable."}
        elif v2p in combo_added:
            ident = {"status": "COMBINATION_ONLY", "with": ["rigidBody.massKg"],
                     "reason": "Only m + A_ii (effective inertia) is observable; added mass is not separable from rigid-body mass."}
        q = c.Q(node["value"], unit, prov=_prov(node["source"]), source=node["source"], locator=node["location"],
                frame=frame, ident=ident, legacy={"v1Path": v1p})
        c.set_path(doc, v2p, q)
    # alternatePrior -> alternatives
    alt = v1.get("alternatePrior", {})
    def add_alt(v2p: str, node_value, source: str, loc: str, note: str | None = None) -> None:
        q = c.get_path(doc, v2p)
        q.setdefault("alternatives", []).append({"value": node_value, "unit": q["unit"], "source": source, "locator": loc})
    for key, targets in _alt_map().items():
        if key in alt:
            n = alt[key]
            add_alt(targets[0], n["value"], n["source"], n["location"])
    if "inertiaKgM2" in alt:
        n = alt["inertiaKgM2"]
        for i, k in enumerate(("IxxKgM2", "IyyKgM2", "IzzKgM2")):
            add_alt(f"rigidBody.inertiaCg.{k}", n["value"][i], n["source"], n["location"])
    for key, table in (("addedMass", ADDED), ("linearDamping", LIN), ("quadraticDamping", QUAD)):
        if key in alt:
            n = alt[key]
            for i, d in enumerate(DOFS):
                add_alt(f"hydrodynamics.{key}.{table[d]}", n["value"][i], n["source"], n["location"])
    for v1p, name in STRUCTS.items():
        n = _get(v1, v1p)
        doc["structures"][name] = {"frame": "body", "data": n["value"], "source": n["source"],
                                   "locator": n["location"], "provenance": _prov(n["source"]), "legacy": {"v1Path": v1p}}
    return c.finish(doc)


def to_v1(v2: dict) -> dict:
    """Rebuild the v1 document the original loader (`vehicle_profile.load/to_params/to_thrusters`) consumes."""
    out: dict = {"schema": V1_SCHEMA, "vehicle": v2["attributes"]["vehicle"],
                 "evidenceLabel": v2["migratedFrom"]["legacyEvidenceLabel"], "sources": copy.deepcopy(v2["sources"])}
    for s in out["sources"].values():
        s.pop("uncommitted", None)
    for v1p, v2p, _u, _f in ROWS:
        q = c.get_path(v2, v2p)
        node = {"value": q["value"], "source": q["source"], "location": q["locator"]}
        c.set_path(out, v1p, node)
    for v1p, name in STRUCTS.items():
        s = v2["structures"][name]
        c.set_path(out, v1p, {"value": s["data"], "source": s["source"], "location": s["locator"]})
    out["thrusters"]["model"] = v2["attributes"].get("thrusterModel", "")
    return out
