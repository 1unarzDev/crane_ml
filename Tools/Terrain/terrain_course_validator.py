#!/usr/bin/env python3
"""Label a land run on a terrain course according to DECISIONS.md D12.

D12: the campus force branch and the legacy diff-drive are not made to refuse
slopes in code. Instead, any run that uses either of them on a course whose
slope exceeds 2 deg is labelled ``INVALID_FOR_SLOPE`` and cannot serve as evidence.

The drive plant is inferred from the Unity command-line arguments, using the same
flags that ``CraneIndustrialCampus.cs`` and ``CraneCampusRobotModel.cs`` read:
  no --crane-campus-model                       -> legacy-diff-drive
  --crane-campus-model, no contact wheels       -> campus-force-branch
  --crane-campus-model --crane-campus-contact-wheels -> campus-contact-wheels
  --crane-campus-slope-traction (proposed, D12) -> slope-aware-traction
The Ackermann WheelCollider rover is identified by an explicit ``plant`` override.
"""
from __future__ import annotations

import argparse
import hashlib
import json
import shlex
import sys
from pathlib import Path

COURSE_SCHEMA = "crane-land-terrain-course-v1"
RESULT_SCHEMA = "crane-land-terrain-run-label-v1"
SLOPE_LIMIT_DEG = 2.0
SLOPE_TOLERANCE_DEG = 1e-6  # absorbs finite-difference rounding of an exactly-2-deg course
SLOPE_INVALID_PLANTS = frozenset({"legacy-diff-drive", "campus-force-branch"})
KNOWN_PLANTS = SLOPE_INVALID_PLANTS | {"campus-contact-wheels", "slope-aware-traction", "ackermann-wheelcollider"}
EVIDENCE_LABELS = {"IMPLEMENTED", "FIXTURE_VALIDATED", "LIVE_SIM_VALIDATED", "PHYSICALLY_CALIBRATED", "UNKNOWN", "NOT_RUN"}


class ValidationError(ValueError):
    pass


def infer_plant(args: list[str]) -> str:
    if "--crane-campus-slope-traction" in args:
        return "slope-aware-traction"
    if "--crane-campus-model" not in args:
        return "legacy-diff-drive"
    if "--crane-campus-contact-wheels" in args:
        return "campus-contact-wheels"
    return "campus-force-branch"


def course_slope_deg(course: dict) -> float:
    if course.get("schema") != COURSE_SCHEMA:
        raise ValidationError("course schema")
    gt = course.get("groundTruth") or {}
    values = [gt.get("measuredMaxSlopeDeg"), gt.get("analyticSlopeDeg")]
    values = [float(v) for v in values if v is not None]
    if not values:
        raise ValidationError("course lacks slope ground truth; cannot decide D12 validity")
    return max(values)


def verify_heightmap(course: dict, raw: bytes) -> None:
    n = course["heightmapResolution"]
    if len(raw) != n * n * 2:
        raise ValidationError("heightmap size does not match resolution")
    if hashlib.sha256(raw).hexdigest() != course["heightmapSha256"]:
        raise ValidationError("heightmap sha256 mismatch")


def label_run(course: dict, args: list[str] | None = None, plant: str | None = None,
              claimed_label: str = "NOT_RUN", heightmap: bytes | None = None) -> dict:
    if plant is None:
        if args is None:
            raise ValidationError("need run args or an explicit plant")
        plant = infer_plant(args)
    if plant not in KNOWN_PLANTS:
        raise ValidationError(f"unknown plant {plant}")
    if claimed_label not in EVIDENCE_LABELS:
        raise ValidationError(f"unknown evidence label {claimed_label}")
    if heightmap is not None:
        verify_heightmap(course, heightmap)
    slope = course_slope_deg(course)
    reasons = []
    label = claimed_label
    if plant in SLOPE_INVALID_PLANTS and slope > SLOPE_LIMIT_DEG + SLOPE_TOLERANCE_DEG:
        label = "INVALID_FOR_SLOPE"
        reasons.append(f"{plant} has no slope-correct traction (D12); course slope {slope:.3f} deg > {SLOPE_LIMIT_DEG} deg")
    if course.get("ablation") and label != "INVALID_FOR_SLOPE":
        reasons.append("deformable ablation course: PhysX cannot represent sinkage; no fidelity claim")
    return {"schema": RESULT_SCHEMA, "courseId": course.get("courseId"), "plant": plant,
            "courseSlopeDeg": slope, "slopeLimitDeg": SLOPE_LIMIT_DEG, "label": label,
            "evidenceUsable": label != "INVALID_FOR_SLOPE", "reasons": reasons,
            "heightmapVerified": heightmap is not None}


def main(argv: list[str] | None = None) -> int:
    ap = argparse.ArgumentParser(description="Apply D12 slope validity to a terrain run")
    ap.add_argument("--course", required=True, type=Path)
    ap.add_argument("--run-args", default=None, help="Unity command line (quoted string)")
    ap.add_argument("--plant", default=None, choices=sorted(KNOWN_PLANTS))
    ap.add_argument("--claimed-label", default="NOT_RUN")
    ap.add_argument("--heightmap", type=Path, default=None)
    a = ap.parse_args(argv)
    course = json.loads(a.course.read_text(encoding="utf-8"))
    try:
        result = label_run(course, shlex.split(a.run_args) if a.run_args is not None else None, a.plant,
                           a.claimed_label, a.heightmap.read_bytes() if a.heightmap else None)
    except ValidationError as error:
        print(f"terrain course validation failed: {error}", file=sys.stderr)
        return 2
    print(json.dumps(result, indent=2))
    return 0 if result["evidenceUsable"] else 1


if __name__ == "__main__":
    sys.exit(main())
