#!/usr/bin/env python3
"""Convert a ROS PNG/YAML occupancy map into deterministic Unity wall descriptors.

The source image is never embedded in the result. The output retains source hashes, ROS map
coordinates, merged occupied-boundary segments, and CRANE's ROS-FLU-to-Unity transform.
"""

from __future__ import annotations

import argparse
import csv
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


Point = tuple[int, int]
Edge = tuple[Point, Point]


def boundary_edges(cells: set[tuple[int, int]]) -> set[Edge]:
    """Return directed unit edges with occupied space consistently on the left."""
    edges: set[Edge] = set()
    for x, y in cells:
        if (x, y - 1) not in cells:
            edges.add(((x, y), (x + 1, y)))
        if (x + 1, y) not in cells:
            edges.add(((x + 1, y), (x + 1, y + 1)))
        if (x, y + 1) not in cells:
            edges.add(((x + 1, y + 1), (x, y + 1)))
        if (x - 1, y) not in cells:
            edges.add(((x, y + 1), (x, y)))
    return edges


def direction(edge: Edge) -> int:
    dx = edge[1][0] - edge[0][0]
    dy = edge[1][1] - edge[0][1]
    return {(1, 0): 0, (0, 1): 1, (-1, 0): 2, (0, -1): 3}[(dx, dy)]


def trace_contours(edges: set[Edge]) -> list[list[Point]]:
    """Join directed cell edges into deterministic contours.

    At diagonal cell contacts a vertex has two exits. Choosing the strongest left turn keeps the
    occupied cell on the same side and produces separate, non-self-intersecting contours.
    """
    outgoing: dict[Point, list[Point]] = {}
    for start, end in edges:
        outgoing.setdefault(start, []).append(end)
    unused = set(edges)
    contours: list[list[Point]] = []
    while unused:
        first = min(unused)
        contour = [first[0]]
        current = first
        while current in unused:
            unused.remove(current)
            contour.append(current[1])
            if current[1] == contour[0]:
                break
            candidates = [(current[1], end) for end in outgoing.get(current[1], [])
                          if (current[1], end) in unused]
            if not candidates:
                break
            incoming = direction(current)
            # Left, straight, right, reverse. Reverse is only a final defensive option.
            turn_rank = {1: 0, 0: 1, 3: 2, 2: 3}
            current = min(candidates,
                          key=lambda edge: (turn_rank[(direction(edge) - incoming) % 4], edge[1]))
        contours.append(contour)
    return contours


def point_line_distance(point: Point, start: Point, end: Point) -> float:
    dx, dy = end[0] - start[0], end[1] - start[1]
    if dx == 0 and dy == 0:
        return math.dist(point, start)
    return abs(dy * point[0] - dx * point[1] + end[0] * start[1] - end[1] * start[0]) / math.hypot(dx, dy)


def simplify_open(points: list[Point], tolerance: float) -> list[Point]:
    if len(points) <= 2:
        return points
    distances = [point_line_distance(point, points[0], points[-1]) for point in points[1:-1]]
    maximum = max(distances, default=0.0)
    if maximum <= tolerance:
        return [points[0], points[-1]]
    index = distances.index(maximum) + 1
    return simplify_open(points[:index + 1], tolerance)[:-1] + simplify_open(points[index:], tolerance)


def simplify_contour(points: list[Point], tolerance: float) -> list[Point]:
    closed = len(points) > 2 and points[0] == points[-1]
    values = points[:-1] if closed else points
    if len(values) <= 3 or tolerance <= 0:
        return values + ([values[0]] if closed else [])
    anchor = min(range(len(values)), key=lambda index: values[index])
    values = values[anchor:] + values[:anchor]
    split = max(range(1, len(values)), key=lambda index: math.dist(values[0], values[index]))
    first = simplify_open(values[:split + 1], tolerance)
    second = simplify_open(values[split:] + [values[0]], tolerance)
    simplified = first[:-1] + second
    return simplified if simplified[-1] == simplified[0] else simplified + [simplified[0]]


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


def load_centerline(path: Path | None) -> tuple[dict | None, list[list[float]]]:
    if path is None:
        return None, []
    points: list[list[float]] = []
    with path.open(newline="", encoding="utf-8") as stream:
        for row in csv.reader(stream):
            if not row or row[0].lstrip().startswith("#"):
                continue
            if len(row) < 2:
                raise ValueError(f"centerline row needs x,y: {row!r}")
            points.append([float(row[0]), float(row[1])])
    if len(points) < 2:
        raise ValueError("centerline needs at least two points")
    return {"name": path.name, "sha256": sha256(path)}, points


def generate(yaml_path: Path, wall_height: float = 1.0,
             wall_thickness: float | None = None, simplification_tolerance: float = 0.10,
             environment_id: str = "f1tenth-occupancy-map-v1",
             upstream_project: str = "external-ros-occupancy-map",
             upstream_version: str = "unversioned", centerline_path: Path | None = None) -> dict:
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
    contours = [simplify_contour(contour, simplification_tolerance / resolution)
                for contour in trace_contours(boundary_edges(cells))]
    segments = []
    for contour_index, contour in enumerate(contours, 1):
        for segment_index, (start, end) in enumerate(zip(contour, contour[1:]), 1):
            if start == end:
                continue
            segments.append(segment_record(
                f"track-wall-c{contour_index:04d}-s{segment_index:04d}",
                ros_point(origin, resolution, *start),
                ros_point(origin, resolution, *end), thickness, wall_height))
    width, height = image.size
    corners = [ros_point(origin, resolution, x, y)
               for x, y in ((0, 0), (width, 0), (width, height), (0, height))]
    centerline_source, centerline = load_centerline(centerline_path)
    spawn = None
    if centerline:
        dx = centerline[1][0] - centerline[0][0]
        dy = centerline[1][1] - centerline[0][1]
        spawn = {
            "ros": {"position": centerline[0], "yawRadians": math.atan2(dy, dx)},
            "unity": {
                "position": [-centerline[0][1], 0.16, centerline[0][0]],
                "yawDegrees": -math.degrees(math.atan2(dy, dx)),
            },
        }
    center_x = sum(point[0] for point in corners) * 0.25
    center_y = sum(point[1] for point in corners) * 0.25
    return {
        "schema": "crane-f1tenth-map-v1",
        "environmentId": environment_id,
        "source": {
            "yaml": {"name": yaml_path.name, "sha256": sha256(yaml_path)},
            "image": {"name": image_path.name, "sha256": sha256(image_path)},
            "centerline": centerline_source,
            "formatReference": "ROS occupancy grid PNG/YAML",
            "upstreamProject": upstream_project,
            "upstreamVersion": upstream_version,
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
        "robotSpawn": spawn,
        "centerline": centerline,
        "canonicalCollision": {
            "wallHeightMeters": wall_height,
            "wallThicknessMeters": thickness,
            "simplificationToleranceMeters": simplification_tolerance,
            "sourceBoundaryEdgeCount": len(boundary_edges(cells)),
            "contourCount": len(contours),
            "floor": {
                "semanticId": "track-floor",
                "semanticRole": "drivable-track-floor",
                "unityCollider": {
                    "center": [-center_y, -0.05, center_x],
                    "size": [height * resolution, 0.1, width * resolution],
                    "yawDegrees": -math.degrees(origin[2]),
                },
            },
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
    parser.add_argument("--simplification-tolerance", type=float, default=0.10)
    parser.add_argument("--environment-id", default="f1tenth-occupancy-map-v1")
    parser.add_argument("--upstream-project", default="external-ros-occupancy-map")
    parser.add_argument("--upstream-version", default="unversioned")
    parser.add_argument("--centerline", type=Path)
    args = parser.parse_args()
    result = generate(args.map_yaml.resolve(), args.wall_height, args.wall_thickness,
                      args.simplification_tolerance, args.environment_id,
                      args.upstream_project, args.upstream_version,
                      args.centerline.resolve() if args.centerline else None)
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(result, indent=2, sort_keys=True) + "\n", encoding="utf-8")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
