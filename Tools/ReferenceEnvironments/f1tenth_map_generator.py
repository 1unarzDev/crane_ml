#!/usr/bin/env python3
"""Convert a ROS PNG/YAML occupancy map into deterministic Unity wall descriptors.

The source image is never embedded in the result. The output retains source hashes, ROS map
coordinates, merged occupied-boundary segments, and CRANE's ROS-FLU-to-Unity transform.
"""

from __future__ import annotations

import argparse
import hashlib
import json
import math
from pathlib import Path

import yaml
from PIL import Image


def sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def occupied_cells(image: Image.Image, negate: int, occupied_threshold: float) -> set[tuple[int, int]]:
    grayscale = image.convert("L")
    width, height = grayscale.size
    result: set[tuple[int, int]] = set()
    pixels = grayscale.load()
    for row in range(height):
        for column in range(width):
            value = pixels[column, row] / 255.0
            probability = value if negate else 1.0 - value
            if probability > occupied_threshold:
                # ROS map coordinates use a bottom-left origin; PNG rows start at the top.
                result.add((column, height - row - 1))
    return result


def boundary_edges(cells: set[tuple[int, int]]) -> tuple[set[tuple[int, int, int]], set[tuple[int, int, int]]]:
    """Return unit horizontal (x,y,length) and vertical (x,y,length) boundary edges."""
    horizontal: set[tuple[int, int, int]] = set()
    vertical: set[tuple[int, int, int]] = set()
    for x, y in cells:
        if (x, y - 1) not in cells:
            horizontal.add((x, y, 1))
        if (x, y + 1) not in cells:
            horizontal.add((x, y + 1, 1))
        if (x - 1, y) not in cells:
            vertical.add((x, y, 1))
        if (x + 1, y) not in cells:
            vertical.add((x + 1, y, 1))
    return horizontal, vertical


def merge_edges(edges: set[tuple[int, int, int]], horizontal: bool) -> list[tuple[int, int, int]]:
    grouped: dict[int, list[int]] = {}
    for x, y, _ in edges:
        fixed, varying = (y, x) if horizontal else (x, y)
        grouped.setdefault(fixed, []).append(varying)
    merged: list[tuple[int, int, int]] = []
    for fixed in sorted(grouped):
        values = sorted(grouped[fixed])
        start = previous = values[0]
        for value in values[1:]:
            if value == previous + 1:
                previous = value
                continue
            merged.append((start, fixed, previous - start + 1) if horizontal
                          else (fixed, start, previous - start + 1))
            start = previous = value
        merged.append((start, fixed, previous - start + 1) if horizontal
                      else (fixed, start, previous - start + 1))
    return merged


def ros_point(origin: list[float], resolution: float, x: float, y: float) -> tuple[float, float]:
    yaw = origin[2]
    scaled_x, scaled_y = x * resolution, y * resolution
    return (
        origin[0] + math.cos(yaw) * scaled_x - math.sin(yaw) * scaled_y,
        origin[1] + math.sin(yaw) * scaled_x + math.cos(yaw) * scaled_y,
    )


def segment_record(identifier: str, start: tuple[float, float], end: tuple[float, float],
                   thickness: float, height: float) -> dict:
    center_x = (start[0] + end[0]) * 0.5
    center_y = (start[1] + end[1]) * 0.5
    length = math.dist(start, end)
    yaw = math.atan2(end[1] - start[1], end[0] - start[0])
    return {
        "semanticId": identifier,
        "semanticRole": "track-boundary-wall",
        "ros": {"start": list(start), "end": list(end), "yawRadians": yaw},
        # ROS x-forward/y-left maps to Unity z-forward/x-right.
        "unityCollider": {
            "center": [-center_y, height * 0.5, center_x],
            "size": [thickness, height, length],
            "yawDegrees": -math.degrees(yaw),
        },
    }


def generate(yaml_path: Path, wall_height: float = 1.0,
             wall_thickness: float | None = None) -> dict:
    metadata = yaml.safe_load(yaml_path.read_text(encoding="utf-8"))
    required = {"image", "resolution", "origin", "negate", "occupied_thresh", "free_thresh"}
    missing = sorted(required - metadata.keys())
    if missing:
        raise ValueError(f"map YAML missing required keys: {', '.join(missing)}")
    image_path = (yaml_path.parent / metadata["image"]).resolve()
    image = Image.open(image_path)
    resolution = float(metadata["resolution"])
    origin = [float(value) for value in metadata["origin"]]
    thickness = wall_thickness if wall_thickness is not None else resolution
    cells = occupied_cells(image, int(metadata["negate"]), float(metadata["occupied_thresh"]))
    horizontal, vertical = boundary_edges(cells)
    merged = [(True, edge) for edge in merge_edges(horizontal, True)]
    merged += [(False, edge) for edge in merge_edges(vertical, False)]
    segments = []
    for index, (is_horizontal, (x, y, length)) in enumerate(merged, 1):
        grid_end = (x + length, y) if is_horizontal else (x, y + length)
        segments.append(segment_record(
            f"track-wall-{index:05d}", ros_point(origin, resolution, x, y),
            ros_point(origin, resolution, *grid_end), thickness, wall_height))
    width, height = image.size
    corners = [ros_point(origin, resolution, x, y)
               for x, y in ((0, 0), (width, 0), (width, height), (0, height))]
    return {
        "schema": "crane-f1tenth-map-v1",
        "source": {
            "yaml": {"name": yaml_path.name, "sha256": sha256(yaml_path)},
            "image": {"name": image_path.name, "sha256": sha256(image_path)},
            "formatReference": "ROS occupancy grid PNG/YAML",
        },
        "map": {
            "widthPixels": width,
            "heightPixels": height,
            "resolutionMetersPerPixel": resolution,
            "origin": origin,
            "negate": int(metadata["negate"]),
            "occupiedThreshold": float(metadata["occupied_thresh"]),
            "freeThreshold": float(metadata["free_thresh"]),
            "occupiedCellCount": len(cells),
            "rosCorners": corners,
        },
        "canonicalCollision": {
            "wallHeightMeters": wall_height,
            "wallThicknessMeters": thickness,
            "segments": segments,
        },
        "coordinateConvention": {
            "pngRows": "top-to-bottom",
            "rosMap": "bottom-left, origin yaw applied",
            "unity": "x=-ros_y, y=up, z=ros_x",
        },
    }


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("map_yaml", type=Path)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--wall-height", type=float, default=1.0)
    parser.add_argument("--wall-thickness", type=float)
    args = parser.parse_args()
    result = generate(args.map_yaml.resolve(), args.wall_height, args.wall_thickness)
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(result, indent=2, sort_keys=True) + "\n", encoding="utf-8")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
