"""Load cited vehicle-profile JSON into aerial_model.Vehicle."""

import json
from pathlib import Path

import numpy as np

from aerial_model import Battery, Rotor, Vehicle

PROFILE_DIR = Path(__file__).resolve().parent / "profiles"
REQUIRED = ("mass", "inertia", "kf", "km", "tau_up", "tau_down", "omega_max",
            "rotor_drag", "rolling_moment", "rotor_radius", "drag_linear", "drag_quadratic")
STATUSES = {"MEASURED_PUBLISHED", "VENDOR_SPEC", "VENDOR_SIM_MODEL_CITING_SYSID",
            "SIMULATOR_DEFAULT", "DERIVED", "ASSUMED", "UNKNOWN", "NOT_REPRESENTABLE"}


def profile_paths():
    return sorted(PROFILE_DIR.glob("*.json"))


def read_profile(name_or_path):
    path = Path(name_or_path)
    if not path.suffix:
        path = PROFILE_DIR / f"{name_or_path}.json"
    return json.loads(path.read_text(encoding="utf-8"))


def validate_profile(doc):
    problems = []
    if doc.get("schema") != "crane-aerial-vehicle-profile-v1":
        problems.append("schema")
    sources = doc.get("sources", {})
    params = doc.get("parameters", {})
    for key in REQUIRED:
        if key not in params:
            problems.append(f"missing parameter {key}")
    for key, entry in params.items():
        if entry.get("status") not in STATUSES:
            problems.append(f"{key}: bad status {entry.get('status')}")
        if not entry.get("locator"):
            problems.append(f"{key}: no locator")
        src = entry.get("source")
        if src is not None and src not in sources:
            problems.append(f"{key}: unknown source {src}")
        if src is None and entry.get("status") not in {"ASSUMED", "UNKNOWN", "SIMULATOR_DEFAULT"}:
            problems.append(f"{key}: status {entry.get('status')} requires a source")
    for name, src in sources.items():
        if "license" not in src:
            problems.append(f"source {name}: no license")
    for rotor in doc.get("rotors", []):
        if rotor.get("source") not in sources:
            problems.append(f"rotor {rotor.get('name')}: unknown source")
        if rotor.get("spin") not in (-1, 1):
            problems.append(f"rotor {rotor.get('name')}: spin")
    return problems


def value(doc, key, default=None):
    entry = doc["parameters"].get(key)
    return default if entry is None else entry["value"]


def load_vehicle(name_or_path, with_battery=True, ground_effect=False):
    doc = read_profile(name_or_path)
    problems = validate_profile(doc)
    if problems:
        raise ValueError(f"invalid profile: {problems}")
    rotors = [Rotor(position=np.array(r["position"], dtype=float), spin=int(r["spin"]),
                    kf=value(doc, "kf"), km=value(doc, "km"),
                    omega_max=value(doc, "omega_max"), tau_up=value(doc, "tau_up"),
                    tau_down=value(doc, "tau_down"), rotor_drag=value(doc, "rotor_drag"),
                    rolling_moment=value(doc, "rolling_moment"),
                    radius=value(doc, "rotor_radius"))
              for r in doc["rotors"]]
    battery = None
    if with_battery and "battery_resistance" in doc["parameters"]:
        battery = Battery(v_full=value(doc, "battery_v_full"), v_empty=value(doc, "battery_v_empty"),
                          v_nominal=value(doc, "battery_v_nominal"),
                          resistance=value(doc, "battery_resistance"),
                          capacity_ah=value(doc, "battery_capacity_ah"),
                          efficiency=value(doc, "battery_efficiency"))
    return Vehicle(name=doc["id"], mass=value(doc, "mass"),
                   inertia=np.array(value(doc, "inertia"), dtype=float), rotors=rotors,
                   drag_linear=np.array(value(doc, "drag_linear"), dtype=float),
                   drag_quadratic=value(doc, "drag_quadratic"), ground_effect=ground_effect,
                   battery=battery)
