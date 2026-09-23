#!/usr/bin/env python3
"""Generate distinct, versioned land geometries for the diagnostic study.

This is an offline catalog generator, not a simulator-side randomizer.  Every layout is materialized
in the resulting JSON before collection so its bytes, split, mechanism stratum, and geometry can be
frozen.  The generated catalog is consumed by the existing runtime proving-ground module.
"""

from __future__ import annotations

import argparse
from collections import deque
import hashlib
import json
import math
from pathlib import Path
import random


# Keep both side boundaries and a cross-arena obstruction jointly observable by the existing
# 3.5 m TurtleBot LiDAR.  The initial 8 m development arena left the boundary junctions outside
# sensor range, so Nav2 repeatedly planned through unobserved free space instead of exposing the
# intended grid-disconnection mechanism.
WIDTH_M = 5.0
LENGTH_M = 22.0
START = (0.0, 0.08, 0.0)
GOAL = (0.0, 0.08, 18.0)
EFFECTIVE_RADIUS_M = 0.77  # 0.22 m robot radius + 0.55 m configured inflation.
GRID_RESOLUTION_M = 0.10


def box(identifier: str, role: str, x: float, z: float, size_x: float, size_z: float,
        color: tuple[float, float, float]) -> dict:
    return {
        "id": identifier,
        "role": role,
        "center": [round(x, 3), 1.0, round(z, 3)],
        "size": [round(size_x, 3), 2.0, round(size_z, 3)],
        "color": list(color),
        "activeInitially": True,
        "activationAfterSeconds": -1.0,
        "removalAfterSeconds": -1.0,
    }


def layout(identifier: str, split: str, mechanism: str, seed: int,
           obstacles: list[dict], relevant: list[str], challenge: str,
           broad_outcome: str, alternatives: list[str]) -> dict:
    return {
        "id": identifier,
        "seed": seed,
        "robot": "TurtleBot3 Waffle-class",
        "start": list(START),
        "goal": list(GOAL),
        "approximateLengthMeters": 18.0 if not alternatives else 22.0,
        "alternatives": alternatives,
        "relevantObstacles": relevant,
        "expectedChallenge": challenge,
        "expectedBroadOutcome": broad_outcome,
        "studySplit": split,
        "diagnosticMechanism": mechanism,
        "generatorSeed": seed,
        "obstacles": obstacles,
    }


def generate_stratum(split: str, mechanism: str, count: int, first_seed: int) -> list[dict]:
    results = []
    for index in range(count):
        seed = first_seed + index
        rng = random.Random(seed)
        prefix = f"diagnostic-{split}-{mechanism}-{index + 1:03d}"
        if mechanism == "grid-disconnection":
            z = rng.uniform(6.0, 13.0)
            thickness = rng.uniform(0.45, 0.85)
            obstacle = box(
                prefix + "-wall", "complete-blockage", rng.uniform(-0.04, 0.04), z,
                8.0, thickness, (0.80, 0.12, 0.12),
            )
            value = layout(
                prefix, "development-calibration", "failed-induction-calibration", seed,
                [obstacle], [obstacle["id"]],
                "The offline canonical raster is disconnected, but retained live Nav2 runs did "
                "not establish planner-grid disconnection.",
                "calibration-only-no-predicted-runtime-outcome", [],
            )
            value["offlineGeometryClassification"] = "canonical-raster-disconnected"
            value["studyAdmission"] = "excluded-failed-runtime-induction"
            results.append(value)
        elif mechanism == "connected-detour":
            first_z = rng.uniform(4.5, 7.0)
            separation = rng.uniform(5.0, 7.0)
            opening = rng.uniform(1.90, 2.30)
            barrier_width = WIDTH_M - opening
            thickness = rng.uniform(0.45, 0.75)
            first_open_right = index % 2 == 0
            first_x = (-WIDTH_M / 2.0 + barrier_width / 2.0) if first_open_right else (
                WIDTH_M / 2.0 - barrier_width / 2.0)
            second_x = -first_x
            obstacles = [
                box(prefix + "-near", "partial-route-barrier", first_x, first_z,
                    barrier_width, thickness, (0.82, 0.38, 0.12)),
                box(prefix + "-far", "partial-route-barrier", second_x,
                    first_z + separation, barrier_width, thickness,
                    (0.82, 0.38, 0.12)),
            ]
            alternatives = (
                ["right opening then left opening"] if first_open_right
                else ["left opening then right opening"]
            )
            results.append(layout(
                prefix, split, mechanism, seed, obstacles,
                [value["id"] for value in obstacles],
                "Two alternating partial barriers restrict the direct route but retain a detour.",
                "avoidance-and-path-change", alternatives,
            ))
        elif mechanism == "nominal-clear-route":
            side = -1.0 if index % 2 == 0 else 1.0
            obstacles = []
            for obstacle_index in range(3):
                x = side * rng.uniform(1.65, 2.00)
                z = rng.uniform(3.0 + obstacle_index * 4.5, 5.5 + obstacle_index * 4.5)
                size = rng.uniform(0.45, 0.65)
                obstacles.append(box(
                    f"{prefix}-clutter-{obstacle_index + 1}", "irrelevant-side-clutter",
                    x, z, size, size, (0.18, 0.54, 0.76),
                ))
                side *= -1.0
            results.append(layout(
                prefix, split, mechanism, seed, obstacles,
                [value["id"] for value in obstacles],
                "Side clutter is visible while the requested center route remains clear.",
                "nominal-success", ["center route"],
            ))
        else:
            raise ValueError(f"unknown mechanism stratum: {mechanism}")
    return results


def _blocked(obstacles: list[dict], x: float, z: float) -> bool:
    for obstacle in obstacles:
        center_x, _, center_z = obstacle["center"]
        size_x, _, size_z = obstacle["size"]
        if (
            abs(x - center_x) <= size_x / 2.0 + EFFECTIVE_RADIUS_M
            and abs(z - center_z) <= size_z / 2.0 + EFFECTIVE_RADIUS_M
        ):
            return True
    return False


def connected(layout_value: dict) -> bool:
    min_x = -WIDTH_M / 2.0 + EFFECTIVE_RADIUS_M
    max_x = WIDTH_M / 2.0 - EFFECTIVE_RADIUS_M
    min_z = -1.0
    max_z = LENGTH_M
    width = math.floor((max_x - min_x) / GRID_RESOLUTION_M) + 1
    height = math.floor((max_z - min_z) / GRID_RESOLUTION_M) + 1

    def cell(point: tuple[float, float]) -> tuple[int, int]:
        return (
            round((point[0] - min_x) / GRID_RESOLUTION_M),
            round((point[1] - min_z) / GRID_RESOLUTION_M),
        )

    def position(value: tuple[int, int]) -> tuple[float, float]:
        return min_x + value[0] * GRID_RESOLUTION_M, min_z + value[1] * GRID_RESOLUTION_M

    start = cell((START[0], START[2]))
    goal = cell((GOAL[0], GOAL[2]))
    if _blocked(layout_value["obstacles"], *position(start)):
        return False
    queue = deque([start])
    visited = {start}
    while queue:
        current = queue.popleft()
        if current == goal:
            return True
        for delta_x, delta_z in ((-1, 0), (1, 0), (0, -1), (0, 1)):
            candidate = current[0] + delta_x, current[1] + delta_z
            if candidate in visited:
                continue
            if not (0 <= candidate[0] < width and 0 <= candidate[1] < height):
                continue
            if _blocked(layout_value["obstacles"], *position(candidate)):
                continue
            visited.add(candidate)
            queue.append(candidate)
    return False


def direct_route_blocked(layout_value: dict) -> bool:
    samples = int((GOAL[2] - START[2]) / GRID_RESOLUTION_M) + 1
    return any(
        _blocked(layout_value["obstacles"], 0.0, START[2] + index * GRID_RESOLUTION_M)
        for index in range(samples)
    )


def validate(catalog: dict) -> None:
    layouts = catalog["layouts"]
    identifiers = [value["id"] for value in layouts]
    if len(identifiers) != len(set(identifiers)):
        raise ValueError("layout IDs are not unique")
    signatures = {
        json.dumps(value["obstacles"], sort_keys=True, separators=(",", ":"))
        for value in layouts
    }
    if len(signatures) != len(layouts):
        raise ValueError("generated obstacle geometries are not unique")
    for value in layouts:
        is_connected = connected(value)
        direct_blocked = direct_route_blocked(value)
        mechanism = value["diagnosticMechanism"]
        if mechanism == "failed-induction-calibration" and (
            value.get("offlineGeometryClassification") != "canonical-raster-disconnected"
            or value.get("studyAdmission") != "excluded-failed-runtime-induction"
            or is_connected
            or not direct_blocked
        ):
            raise ValueError(f"invalid disconnection geometry: {value['id']}")
        if mechanism == "connected-detour" and (not is_connected or not direct_blocked):
            raise ValueError(f"invalid connected-detour geometry: {value['id']}")
        if mechanism == "nominal-clear-route" and (not is_connected or direct_blocked):
            raise ValueError(f"invalid nominal geometry: {value['id']}")


def generate() -> dict:
    layouts = []
    # Preserve the stable IDs of the failed grid-disconnection induction as calibration history.
    # Live Nav2 repeatedly produced plans and timed out instead of establishing a disconnected
    # planner grid, so these layouts are not diagnosable cases and have no confirmatory arm.
    layouts.extend(generate_stratum("development", "grid-disconnection", 8, 61000))
    # Development cases below are used only for end-to-end qualification and threshold/prompt
    # freeze.  The prospective detours remain candidates until their full protocol is frozen.
    layouts.extend(generate_stratum("development", "connected-detour", 8, 62000))
    layouts.extend(generate_stratum("development", "nominal-clear-route", 8, 63000))
    # Confirmatory identities are generated and frozen now, but remain sealed until collection.
    layouts.extend(generate_stratum("confirmatory", "connected-detour", 48, 72000))
    layouts.extend(generate_stratum("confirmatory", "nominal-clear-route", 24, 73000))
    catalog = {
        "schema": "crane-land-proving-ground-catalog-v1",
        # Runtime catalog selection binds catalog ``v4`` to this exact environment identity.
        # Keep the study/split metadata on each layout rather than creating a second identity
        # namespace that the existing proving-ground loader cannot authenticate.
        "environmentId": "crane-land-proving-ground-v4",
        "generatorVersion": "diagnostic-land-catalog-v1",
        "generator": {
            "algorithm": "fixed-stratum Python random.Random parameter sampling",
            "effectiveRadiusMeters": EFFECTIVE_RADIUS_M,
            "validationGridResolutionMeters": GRID_RESOLUTION_M,
            "developmentCount": 24,
            "developmentCalibrationOnlyCount": 8,
            "confirmatoryPrimaryDiagnosableCount": 48,
            "confirmatoryNominalControlCount": 24,
        },
        "dimensionsMeters": [WIDTH_M, LENGTH_M],
        "sharedBoxes": [
            box("diagnostic-floor", "traversable-floor", 0.0, 10.0, WIDTH_M + 2.0, 24.0,
                (0.24, 0.27, 0.30)) | {
                    "center": [0.0, -0.10, 10.0], "size": [WIDTH_M + 2.0, 0.20, 24.0]
                },
            box("diagnostic-west-boundary", "boundary-wall", -WIDTH_M / 2.0 - 0.125,
                10.0, 0.25, 24.0,
                (0.44, 0.48, 0.52)),
            box("diagnostic-east-boundary", "boundary-wall", WIDTH_M / 2.0 + 0.125,
                10.0, 0.25, 24.0,
                (0.44, 0.48, 0.52)),
            # Close the canonical arena so the rolling Nav2 costmap cannot route around the
            # longitudinal walls through free space outside their open ends.  The first live v4
            # calibration did exactly that, invalidating the intended disconnection mechanism.
            box("diagnostic-south-boundary", "boundary-wall", 0.0, -2.125,
                WIDTH_M + 0.5, 0.25,
                (0.44, 0.48, 0.52)),
            box("diagnostic-north-boundary", "boundary-wall", 0.0, 22.125,
                WIDTH_M + 0.5, 0.25,
                (0.44, 0.48, 0.52)),
        ],
        "layouts": layouts,
    }
    validate(catalog)
    canonical = json.dumps(catalog, indent=2, sort_keys=False) + "\n"
    catalog["generator"]["catalogContentSha256WithoutSelfHash"] = hashlib.sha256(
        canonical.encode("utf-8")
    ).hexdigest()
    return catalog


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    catalog = generate()
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(catalog, indent=2) + "\n", encoding="utf-8")


if __name__ == "__main__":
    main()
