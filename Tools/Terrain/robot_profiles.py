"""Load cited robot-profile JSON files and derive analytic terrain limits."""
from __future__ import annotations

import json
import math
from pathlib import Path

import terrain_reference as ref

PROFILE_DIR = Path(__file__).resolve().parent / "profiles"
SCHEMA = "crane-ground-robot-profile-v1"


def load(name: str) -> dict:
    data = json.loads((PROFILE_DIR / f"{name}.json").read_text(encoding="utf-8"))
    if data.get("schema") != SCHEMA:
        raise ValueError(f"{name}: schema")
    for key, field in data["parameters"].items():
        if "value" not in field or "source" not in field:
            raise ValueError(f"{name}.{key}: every parameter needs value and source")
    return data


def value(profile: dict, key: str) -> float:
    return float(profile["parameters"][key]["value"])


def analytic_limits(profile: dict, mu: float, rolling_resistance: float = 0.0) -> dict:
    """Closed-form terrain limits for a rigid skid-steer/diff-drive profile."""
    m = value(profile, "totalMassKg")
    W = m * ref.G
    L = value(profile, "wheelbaseM")
    B = value(profile, "trackM")
    h = value(profile, "cgHeightAboveGroundM")
    x_r = value(profile, "cgFromRearAxleM")
    y = value(profile, "cgLateralOffsetM")
    r = value(profile, "wheelRadiusM")
    driven = profile.get("drivenAxles", "all")
    out = {
        "staticHoldDeg": math.degrees(ref.static_hold_limit(mu)),
        "tractionClimbDeg": math.degrees(ref.max_climb_traction(mu, L, x_r, h, driven, rolling_resistance)),
        "lateralTipDegRightDownhill": math.degrees(ref.lateral_tip_angle(B, h, y, "right")),
        "lateralTipDegLeftDownhill": math.degrees(ref.lateral_tip_angle(B, h, y, "left")),
        "longitudinalTipDegNoseUp": math.degrees(ref.longitudinal_tip_angle(x_r, h)),
        "drivenWheelStepM": ref.max_step_driven_wheel(r, mu),
    }
    if "peakWheelTorqueNm" in profile["parameters"]:
        n = int(profile.get("drivenWheelCount", 4))
        out["torqueClimbDeg"] = math.degrees(ref.max_climb_torque(n * value(profile, "peakWheelTorqueNm"), r, W, rolling_resistance))
    if profile.get("steering") == "skid":
        res = ref.skid_steer_steady_turn(ref.four_wheel_layout(W, L, B), -0.2, 0.2, mu)
        out["coulombEffectiveTrackM"] = res.effective_track
        out["coulombTrackMultiplier"] = res.effective_track / B
    return out
