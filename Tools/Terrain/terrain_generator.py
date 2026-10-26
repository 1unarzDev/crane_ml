#!/usr/bin/env python3
"""Seeded procedural terrain courses with analytic ground truth.

Pure Python. Emits a ``crane-land-terrain-course-v1`` manifest (JSON) and an
optional little-endian 16-bit RAW heightmap compatible with Unity
``TerrainData`` import (square, resolution 2^n + 1). No Unity assets are
written. Every course records its generator parameters, seed, heightmap
SHA-256 and the analytic quantities a simulator run must reproduce.

Course kinds:
  ramp           planar ramp pitched ``angleDeg`` along +z (0-35 deg)
  cross_slope    planar side slope rolled ``angleDeg``; traverse is along +z
  moguls         h = A sin(2 pi x / lx) sin(2 pi z / lz)
  rock_field     flat plane + seeded hemispherical rocks (Poisson count)
  mixed_friction flat plane with a surface-id patch map (concrete -> dirt ...)
  deformable     flat plane + Bekker/Mohr-Coulomb soil patch; ABLATION ONLY
"""
from __future__ import annotations

import argparse
import hashlib
import json
import math
import random
import struct
from pathlib import Path

SCHEMA = "crane-land-terrain-course-v1"
GENERATOR_VERSION = "terrain-generator-1"
MAX_RAMP_DEG = 35.0

# Surface priors are referenced by id from the campus manifest; values are not
# duplicated here so the campus profile remains the single source.
CAMPUS_SURFACE_IDS = (
    "polished-concrete", "normal-concrete", "rough-concrete", "asphalt",
    "painted-epoxy", "metal-plate", "rubber-mat", "compacted-dirt",
)

# Published soil sets (see Docs/ResearchOrchestration/AGENT_HANDOFFS/I1-terrain.md).
SOIL_SETS = {
    "chrono-lete-sand": {
        "bekkerKphi": 5301e3, "bekkerKc": 102e3, "bekkerN": 0.793,
        "mohrCohesionPa": 1.3e3, "mohrFrictionDeg": 31.1, "janosiShearM": 1.2e-2,
        "elasticStiffnessPaPerM": 4e8, "dampingPaSPerM": 3e4,
        "source": "projectchrono/chrono@19257fe src/demos/vehicle/terrain/demo_VEH_SCMTerrain_RigidTire.cpp:227-235 (BSD-3-Clause)",
    },
    "chrono-demo-soft-soil": {
        "bekkerKphi": 0.2e6, "bekkerKc": 0.0, "bekkerN": 1.1,
        "mohrCohesionPa": 0.0, "mohrFrictionDeg": 30.0, "janosiShearM": 0.01,
        "elasticStiffnessPaPerM": 4e7, "dampingPaSPerM": 3e4,
        "source": "projectchrono/chrono@19257fe src/demos/vehicle/terrain/demo_VEH_SCMTerrain_RigidTire.cpp:216-224 (BSD-3-Clause)",
    },
}


class CourseError(ValueError):
    pass


def _grid(resolution: int) -> None:
    n = resolution - 1
    if resolution < 33 or n & (n - 1):
        raise CourseError("resolution must be 2^n + 1 and >= 33 (Unity TerrainData)")


def _heights(kind: str, p: dict, size: float, resolution: int, rng: random.Random):
    """Return (rows of heights [z][x], extras dict). Cell spacing = size/(res-1)."""
    step = size / (resolution - 1)
    coords = [i * step for i in range(resolution)]
    extras: dict = {}
    if kind in ("ramp", "cross_slope"):
        angle = p["angleDeg"]
        if not 0 <= angle <= MAX_RAMP_DEG:
            raise CourseError(f"angleDeg must be within [0, {MAX_RAMP_DEG}]")
        t = math.tan(math.radians(angle))
        flat = p.get("flatApproachM", 2.0)
        if kind == "ramp":   # rises along +z after a flat approach
            rows = [[max(0.0, z - flat) * t for _ in coords] for z in coords]
        else:                # rises along +x (left side up for a +z traverse in Unity, x right)
            rows = [[x * t for x in coords] for _ in coords]
        extras["analyticSlopeDeg"] = angle
        return rows, extras
    if kind == "moguls":
        A, lx, lz = p["amplitudeM"], p["wavelengthXM"], p["wavelengthZM"]
        if A <= 0 or lx <= 4 * step or lz <= 4 * step:
            raise CourseError("moguls need A>0 and wavelengths >= 4 cells")
        rows = [[A * math.sin(2 * math.pi * x / lx) * math.sin(2 * math.pi * z / lz) for x in coords]
                for z in coords]
        # |grad h|^2 = A^2 (kx^2 cos^2 sin^2 + kz^2 sin^2 cos^2); max = (A max(kx, kz))^2
        gmax = 2 * math.pi * A * max(1 / lx, 1 / lz)
        extras["analyticSlopeDeg"] = math.degrees(math.atan(gmax))
        extras["analyticCurvatureRadiusM"] = 1.0 / (A * (2 * math.pi) ** 2 * (1 / lx ** 2 + 1 / lz ** 2))
        return rows, extras
    if kind == "rock_field":
        density = p["rocksPerSquareMeter"]
        rmin, rmax = p["rockRadiusRangeM"]
        margin = p.get("clearMarginM", 1.0)
        if density < 0 or not 0 < rmin <= rmax:
            raise CourseError("invalid rock parameters")
        area = (size - 2 * margin) ** 2
        # Poisson draw (Knuth) for count; positions uniform; radii uniform.
        lam, k, prod, limit = density * area, 0, rng.random(), math.exp(-density * area)
        while prod > limit:
            k += 1
            prod *= rng.random()
        rocks = []
        for i in range(k):
            r = rng.uniform(rmin, rmax)
            rocks.append({"id": f"rock-{i:04d}", "x": rng.uniform(margin, size - margin),
                          "z": rng.uniform(margin, size - margin), "radiusM": r,
                          "heightM": r * p.get("embedFraction", 0.5)})
        rows = [[0.0] * resolution for _ in coords]
        for rock in rocks:
            r, hgt = rock["radiusM"], rock["heightM"]
            # Spherical cap of base radius r and height hgt.
            R = (r * r + hgt * hgt) / (2 * hgt)
            i0, i1 = max(0, int((rock["z"] - r) / step)), min(resolution - 1, int((rock["z"] + r) / step) + 1)
            j0, j1 = max(0, int((rock["x"] - r) / step)), min(resolution - 1, int((rock["x"] + r) / step) + 1)
            for i in range(i0, i1 + 1):
                for j in range(j0, j1 + 1):
                    d2 = (coords[j] - rock["x"]) ** 2 + (coords[i] - rock["z"]) ** 2
                    if d2 < r * r:
                        rows[i][j] = max(rows[i][j], math.sqrt(R * R - d2) - (R - hgt))
        extras.update(rocks=rocks, expectedRockCount=lam, maxStepHeightM=max((r["heightM"] for r in rocks), default=0.0))
        return rows, extras
    if kind in ("mixed_friction", "deformable"):
        return [[0.0] * resolution for _ in coords], extras
    raise CourseError(f"unknown kind {kind}")


def _measured_slope(rows, step: float) -> tuple[float, float]:
    """Central-difference max and mean slope (deg) on interior cells."""
    n = len(rows)
    worst, total, count = 0.0, 0.0, 0
    for i in range(1, n - 1):
        for j in range(1, n - 1):
            gx = (rows[i][j + 1] - rows[i][j - 1]) / (2 * step)
            gz = (rows[i + 1][j] - rows[i - 1][j]) / (2 * step)
            s = math.degrees(math.atan(math.hypot(gx, gz)))
            worst, total, count = max(worst, s), total + s, count + 1
    return worst, total / count


def _raw16(rows, height_range: float) -> bytes:
    lo = min(min(r) for r in rows)
    span = height_range if height_range > 0 else 1.0
    out = bytearray()
    for r in rows:
        for h in r:
            v = int(round((h - lo) / span * 65535))
            out += struct.pack("<H", max(0, min(65535, v)))
    return bytes(out)


def _surface_map(kind: str, p: dict, size: float) -> list[dict]:
    if kind == "mixed_friction":
        patches = p["patches"]
        for patch in patches:
            if patch["surface"] not in CAMPUS_SURFACE_IDS:
                raise CourseError(f"unknown surface {patch['surface']}")
        return patches
    if kind == "deformable":
        soil = p["soil"]
        if soil not in SOIL_SETS:
            raise CourseError(f"unknown soil set {soil}")
        return [{"surface": "normal-concrete", "zRangeM": [0, p["patchStartM"]]},
                {"surface": f"soil:{soil}", "zRangeM": [p["patchStartM"], p["patchStartM"] + p["patchLengthM"]],
                 "soil": SOIL_SETS[soil], "ablation": True,
                 "requiresSolver": "SCM-class deformable contact (not PhysX rigid)"},
                {"surface": "normal-concrete", "zRangeM": [p["patchStartM"] + p["patchLengthM"], size]}]
    return [{"surface": p.get("surface", "normal-concrete"), "zRangeM": [0, size]}]


def generate_course(course_id: str, kind: str, params: dict, seed: int,
                    size_m: float = 16.0, resolution: int = 129) -> tuple[dict, bytes]:
    _grid(resolution)
    rng = random.Random(seed)
    rows, extras = _heights(kind, params, size_m, resolution, rng)
    step = size_m / (resolution - 1)
    hmin, hmax = min(min(r) for r in rows), max(max(r) for r in rows)
    raw = _raw16(rows, hmax - hmin)
    measured_max, measured_mean = _measured_slope(rows, step)
    manifest = {
        "schema": SCHEMA, "generatorVersion": GENERATOR_VERSION, "courseId": course_id,
        "kind": kind, "seed": seed, "parameters": params,
        "sizeMeters": size_m, "heightmapResolution": resolution, "cellMeters": step,
        "heightRangeMeters": [hmin, hmax],
        "heightmapEncoding": "raw16-le, row-major z then x, value=(h-min)/(max-min)*65535",
        "heightmapSha256": hashlib.sha256(raw).hexdigest(),
        "surfaces": _surface_map(kind, params, size_m),
        "groundTruth": {"measuredMaxSlopeDeg": measured_max, "measuredMeanSlopeDeg": measured_mean, **extras},
        "ablation": kind == "deformable",
        "evidenceLabel": "NOT_RUN",
    }
    return manifest, raw


def default_catalog() -> list[dict]:
    """The proposed v1 catalog: parametric ramps, cross slopes, moguls, rocks, friction, soil."""
    specs = []
    for a in (0, 5, 10, 15, 20, 25, 30, 35):
        specs.append(dict(course_id=f"ramp-{a:02d}deg", kind="ramp", params={"angleDeg": a, "surface": "normal-concrete"}, seed=1000 + a))
    for a in (5, 10, 15, 20):
        specs.append(dict(course_id=f"cross-slope-{a:02d}deg", kind="cross_slope", params={"angleDeg": a, "surface": "normal-concrete"}, seed=2000 + a))
    for amp in (0.05, 0.10, 0.20):
        specs.append(dict(course_id=f"moguls-a{int(amp*100):02d}cm", kind="moguls",
                          params={"amplitudeM": amp, "wavelengthXM": 4.0, "wavelengthZM": 4.0, "surface": "compacted-dirt"}, seed=3000 + int(amp * 100)))
    for k, d in enumerate((0.05, 0.2, 0.5)):
        specs.append(dict(course_id=f"rocks-d{int(d*100):03d}", kind="rock_field",
                          params={"rocksPerSquareMeter": d, "rockRadiusRangeM": [0.03, 0.15], "embedFraction": 0.5, "surface": "compacted-dirt"}, seed=4000 + k))
    specs.append(dict(course_id="mixed-friction-concrete-dirt-epoxy", kind="mixed_friction",
                      params={"patches": [{"surface": "normal-concrete", "zRangeM": [0, 5]},
                                          {"surface": "compacted-dirt", "zRangeM": [5, 10]},
                                          {"surface": "painted-epoxy", "zRangeM": [10, 16]}]}, seed=5000))
    specs.append(dict(course_id="mixed-friction-split-mu", kind="mixed_friction",
                      params={"patches": [{"surface": "normal-concrete", "xRangeM": [0, 8], "zRangeM": [0, 16]},
                                          {"surface": "painted-epoxy", "xRangeM": [8, 16], "zRangeM": [0, 16]}]}, seed=5001))
    specs.append(dict(course_id="soil-lete-sand-ablation", kind="deformable",
                      params={"soil": "chrono-lete-sand", "patchStartM": 4.0, "patchLengthM": 6.0}, seed=6000))
    return specs


def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("--out", required=True, type=Path)
    ap.add_argument("--resolution", type=int, default=129)
    ap.add_argument("--raw", action="store_true", help="also write <course>.r16 heightmaps")
    a = ap.parse_args()
    a.out.mkdir(parents=True, exist_ok=True)
    index = []
    for spec in default_catalog():
        manifest, raw = generate_course(resolution=a.resolution, **spec)
        (a.out / f"{spec['course_id']}.json").write_text(json.dumps(manifest, indent=2, sort_keys=True), encoding="utf-8")
        if a.raw:
            (a.out / f"{spec['course_id']}.r16").write_bytes(raw)
        index.append({"courseId": spec["course_id"], "kind": spec["kind"], "heightmapSha256": manifest["heightmapSha256"]})
    (a.out / "catalog.json").write_text(json.dumps({"schema": "crane-land-terrain-catalog-v1", "generatorVersion": GENERATOR_VERSION, "courses": index}, indent=2), encoding="utf-8")


if __name__ == "__main__":
    main()
