#!/usr/bin/env python3
"""Export oracle reference values for the Unity EditMode slope tests.

Writes Assets/Tests/Editor/Terrain/Fixtures/terrain_oracle_references.json and a small
ramp-10deg course (manifest + .r16) for the course-builder tests. Values from
terrain_reference.py are pre-existing oracle outputs. The brush-creep and isotropic-creep
values are closed forms derived here (Pacejka brush: F = mu N (1 - (1-u)^3), u = C s / (3 mu)),
independent of the C# bisection solver.

    python -I Tools/Terrain/export_unity_references.py --out Assets/Tests/Editor/Terrain/Fixtures
"""
from __future__ import annotations

import argparse
import json
import math
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))  # -I drops the script dir
import terrain_generator as tg
import terrain_reference as tr

G = tr.G
MASS = 50.0
GEOM = {"wheelbase": 0.5, "track": 0.55, "cgFromRear": 0.25, "cgHeight": 0.2, "cgLateral": 0.03,
        "wheelRadius": 0.1, "mass": MASS}
MU = 0.7
CKAPPA = 12.0
CALPHA = 12.0   # equal stiffnesses: isotropic regime used by the creep and yaw cases
EPS = 0.05


def creep_speed(theta: float, mu: float, c: float, eps: float, crr: float = 0.0) -> float | None:
    """Steady downhill creep speed (m/s) of a vehicle with all wheels braked, speeds < eps.

    Per-wheel u = C s/(3 mu) is the same for every wheel, so with s = v/eps the force balance per
    unit normal load is  tan(theta) = mu (1-(1-u)^3) + Crr s  (rolling resistance is smoothed
    linearly below eps, as in the C# law). Solved by bisection on u. None if beyond capacity.
    """
    def demand(u: float) -> float:
        return mu * (1.0 - (1.0 - u) ** 3) + crr * (3.0 * mu * u / c)
    target = math.tan(theta)
    if demand(1.0) < target:
        return None
    lo, hi = 0.0, 1.0
    for _ in range(200):
        mid = 0.5 * (lo + hi)
        lo, hi = (mid, hi) if demand(mid) < target else (lo, mid)
    s = 3.0 * mu * 0.5 * (lo + hi) / c
    return s * eps if s <= 1.0 else None


def main() -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument("--out", required=True, type=Path)
    a = ap.parse_args()
    a.out.mkdir(parents=True, exist_ok=True)
    g, W = GEOM, MASS * G
    ref: dict = {"schema": "crane-terrain-unity-references-v1", "gravity": G, "geometry": GEOM,
                 "mu": MU, "cKappa": CKAPPA, "cAlpha": CALPHA, "velocityEpsilon": EPS}

    ref["holdLimit"] = [{"mu": mu, "angle": tr.static_hold_limit(mu)} for mu in (0.3, 0.5, 0.7, 1.0)]

    ref["tip"] = {
        "lateralRoll": tr.lateral_tip_angle(g["track"], g["cgHeight"], g["cgLateral"], "right"),
        "lateralRollNoOffset": tr.lateral_tip_angle(g["track"], g["cgHeight"], 0.0, "right"),
        "longitudinalPitch": tr.longitudinal_tip_angle(g["cgFromRear"], g["cgHeight"]),
    }

    ref["climbTorque"] = [
        {"totalTorque": T, "crr": c, "angle": tr.max_climb_torque(T, g["wheelRadius"], W, c)}
        for T in (15.0, 25.0, 35.0) for c in (0.0, 0.02)
    ]
    ref["climbTraction"] = [
        {"mu": mu, "driven": d, "crr": c, "angle": tr.max_climb_traction(mu, g["wheelbase"], cg_r, g["cgHeight"], d, c), "cgFromRear": cg_r}
        for mu in (0.3, 0.5, 0.7) for d in ("all", "front", "rear") for c in (0.0, 0.02) for cg_r in (0.25, 0.20)
    ]

    ref["loadTransfer"] = []
    for th in (-0.3, 0.0, 0.1, 0.25):
        for ax in (-2.0, 0.0, 1.5):
            nf, nr = tr.longitudinal_axle_loads(W, g["wheelbase"], g["cgFromRear"], g["cgHeight"], th, ax)
            ref["loadTransfer"].append({"kind": "long", "slope": th, "acc": ax, "front": nf, "rear": nr})
    for ph in (-0.3, 0.0, 0.1, 0.25):
        for ay in (-1.5, 0.0, 2.0):
            nl, nr = tr.lateral_side_loads(W, g["track"], g["cgLateral"], g["cgHeight"], ph, ay)
            ref["loadTransfer"].append({"kind": "lat", "slope": ph, "acc": ay, "left": nl, "right": nr})

    ref["crossSlopeHold"] = {"limit": tr.cross_slope_hold_limit(MU),
                             "creep": [{"slope": math.radians(d), "vy": creep_speed(math.radians(d), MU, CALPHA, EPS)}
                                       for d in (2, 5, 10, 20)]}
    ref["zeroCommand"] = {
        "holdLimit": tr.static_hold_limit(MU),
        "creep": [{"slopeDeg": d, "crr": c, "creep": creep_speed(math.radians(d), MU, CKAPPA, EPS, c)}
                  for d in (1, 5, 10, 20, 30) for c in (0.0, 0.02)],
    }
    ref["yawEquivariance"] = {"slopeDeg": 15.0, "headingsDeg": list(range(0, 360, 30)),
                              "creep": creep_speed(math.radians(15), MU, CKAPPA, EPS),
                              "planeForce": W * math.sin(math.radians(15))}
    ref["rollingResistance"] = [{"slopeDeg": d, "force": 0.02 * W * math.cos(math.radians(d))} for d in (0, 10, 25)]

    (a.out / "terrain_oracle_references.json").write_text(json.dumps(ref, indent=2, sort_keys=True), encoding="utf-8")

    manifest, raw = tg.generate_course("ramp-10deg", "ramp", {"angleDeg": 10, "surface": "normal-concrete"}, 1010)
    (a.out / "ramp-10deg.json").write_text(json.dumps(manifest, indent=2, sort_keys=True), encoding="utf-8")
    (a.out / "ramp-10deg.r16").write_bytes(raw)


if __name__ == "__main__":
    main()
