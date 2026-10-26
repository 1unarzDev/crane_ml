"""Load a CRANE underwater vehicle profile JSON into the reference model.

Every numeric coefficient in a profile must be an object
``{"value": ..., "source": <source id>, "location": <table/line>}``; the loader
rejects bare numbers and unknown source ids so that provenance cannot silently
drop out of a profile.
"""
from __future__ import annotations

import json
from pathlib import Path
from typing import Any

import numpy as np

from fossen6dof import G, Thruster, VehicleParams

SCHEMA = "crane.underwater.vehicle_profile/v1"
ALLOWED_LABELS = {"IMPLEMENTED", "FIXTURE_VALIDATED", "LIVE_SIM_VALIDATED", "PHYSICALLY_CALIBRATED", "UNKNOWN", "NOT_RUN"}


class ProfileError(ValueError):
    pass


def _cited(node: Any, sources: dict, path: str) -> Any:
    if not isinstance(node, dict) or "value" not in node:
        raise ProfileError(f"{path}: coefficient must be {{value, source, location}}")
    src = node.get("source")
    if src not in sources:
        raise ProfileError(f"{path}: unknown source id {src!r}")
    if not node.get("location"):
        raise ProfileError(f"{path}: missing location (table/equation/file line)")
    return node["value"]


def load(path: str | Path) -> dict:
    data = json.loads(Path(path).read_text())
    if data.get("schema") != SCHEMA:
        raise ProfileError(f"schema must be {SCHEMA}")
    if data.get("evidenceLabel") not in ALLOWED_LABELS:
        raise ProfileError("evidenceLabel must be a CONTRACTS.md label")
    if data.get("evidenceLabel") == "PHYSICALLY_CALIBRATED":
        raise ProfileError("a literature profile cannot claim PHYSICALLY_CALIBRATED")
    return data


def to_params(data: dict) -> VehicleParams:
    src = data["sources"]
    p = data["rigidBody"]
    h = data["hydrodynamics"]
    c = lambda key, node: _cited(node, src, key)  # noqa: E731
    mass = float(c("massKg", p["massKg"]))
    inertia = np.diag([float(c(f"inertia.{k}", p["inertiaCgKgM2"][k])) for k in ("Ixx", "Iyy", "Izz")])
    r_g = np.array(c("rG", p["rGMeters"]), float)
    r_b = np.array(c("rB", p["rBMeters"]), float)
    buoyancy = float(c("buoyancyN", p["buoyancyN"]))
    dofs = ("u", "v", "w", "p", "q", "r")
    ma = np.diag([float(c(f"addedMass.{k}", h["addedMass"][k])) for k in dofs])
    dl = np.diag([float(c(f"linearDamping.{k}", h["linearDamping"][k])) for k in dofs])
    dq = np.array([float(c(f"quadraticDamping.{k}", h["quadraticDamping"][k])) for k in dofs])
    g = float(c("gravityMps2", p["gravityMps2"])) if "gravityMps2" in p else G
    weight = p.get("weightN")
    if weight is not None:
        w = float(c("weightN", weight))
        if abs(w - mass * g) > 1e-3 * w:
            raise ProfileError(f"weightN {w} inconsistent with massKg*g {mass * g:.3f}")
    return VehicleParams(mass, inertia, r_g, r_b, buoyancy, ma, dl, dq, gravity=g, name=data.get("vehicle", ""))


def to_thrusters(data: dict) -> list[Thruster]:
    src = data["sources"]
    t = data["thrusters"]
    fwd = float(_cited(t["maxForwardN"], src, "thrusters.maxForwardN"))
    rev = float(_cited(t["maxReverseN"], src, "thrusters.maxReverseN"))
    tau = float(_cited(t["timeConstantS"], src, "thrusters.timeConstantS"))
    layout = _cited(t["layout"], src, "thrusters.layout")
    return [
        Thruster(np.array(e["positionM"], float), np.array(e["direction"], float), fwd, rev, tau, e.get("name", ""))
        for e in layout
    ]
