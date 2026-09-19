#!/usr/bin/env python3
"""Resolve a static SDF world into a deterministic CRANE/Unity import manifest.

This is an offline converter, not a runtime SDF implementation. It resolves local model:// mesh
resources, copies only referenced source assets and Collada texture dependencies into a generated
(Git-ignored) Unity asset directory, and keeps collision and visual roles distinct.
"""

from __future__ import annotations

import argparse
import hashlib
import json
import math
import shutil
import struct
from pathlib import Path
from urllib.parse import urlparse
from xml.etree import ElementTree as ET


def sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def numbers(text: str | None, count: int, defaults: tuple[float, ...]) -> list[float]:
    if not text or not text.strip():
        return list(defaults)
    values = [float(value) for value in text.split()]
    if len(values) != count:
        raise ValueError(f"expected {count} values, got {len(values)}: {text!r}")
    return values


def resolve_model_uri(uri: str, resource_roots: list[Path]) -> tuple[Path, str]:
    parsed = urlparse(uri.strip())
    if parsed.scheme != "model":
        raise ValueError(f"only local model:// resources are supported offline: {uri}")
    relative = Path(parsed.netloc) / parsed.path.lstrip("/")
    for root in resource_roots:
        candidate = (root / relative).resolve()
        if candidate.is_file() and candidate.is_relative_to(root.resolve()):
            return candidate, relative.as_posix()
    raise FileNotFoundError(f"unresolved {uri}; searched {', '.join(map(str, resource_roots))}")


def collada_dependencies(path: Path) -> list[Path]:
    if path.suffix.lower() != ".dae":
        return []
    root = ET.parse(path).getroot()
    dependencies: set[Path] = set()
    for element in root.iter():
        if element.tag.rsplit("}", 1)[-1] != "init_from" or not element.text:
            continue
        value = element.text.strip()
        if not value or "://" in value or value.startswith("#"):
            continue
        candidate = (path.parent / value).resolve()
        if candidate.is_file():
            dependencies.add(candidate)
    return sorted(dependencies)


def copy_asset(source: Path, relative: str, output_root: Path,
               unity_asset_root: str, copied: dict[Path, dict]) -> dict:
    source = source.resolve()
    if source in copied:
        return copied[source]
    destination = output_root / "source" / relative
    destination.parent.mkdir(parents=True, exist_ok=True)
    shutil.copyfile(source, destination)
    record = {
        "sourcePath": relative,
        "sourceSha256": sha256(source),
        "bytes": source.stat().st_size,
        "unityAssetPath": f"{unity_asset_root}/source/{relative}",
        "format": source.suffix.lower().lstrip("."),
        "conversion": "source-preserved-native-unity-import",
    }
    copied[source] = record
    return record


def stl_triangles(path: Path) -> list[tuple[tuple[float, float, float], ...]]:
    data = path.read_bytes()
    if len(data) >= 84:
        count = struct.unpack_from("<I", data, 80)[0]
        if 84 + count * 50 == len(data):
            result = []
            offset = 84
            for _ in range(count):
                values = struct.unpack_from("<12fH", data, offset)
                result.append((values[3:6], values[6:9], values[9:12]))
                offset += 50
            return result
    vertices = []
    for line in data.decode("utf-8").splitlines():
        words = line.strip().split()
        if len(words) == 4 and words[0].lower() == "vertex":
            vertices.append(tuple(float(value) for value in words[1:]))
    if not vertices or len(vertices) % 3:
        raise ValueError(f"invalid STL: {path}")
    return [tuple(vertices[index:index + 3]) for index in range(0, len(vertices), 3)]


def prepare_mesh_asset(source: Path, relative: str, output_root: Path,
                       unity_asset_root: str, copied: dict[Path, dict]) -> dict:
    if source.suffix.lower() != ".stl":
        return copy_asset(source, relative, output_root, unity_asset_root, copied)
    source = source.resolve()
    if source in copied:
        return copied[source]
    output_relative = str(Path(relative).with_suffix(".obj")).replace("\\", "/")
    destination = output_root / "source" / output_relative
    destination.parent.mkdir(parents=True, exist_ok=True)
    triangles = stl_triangles(source)
    lines = [f"# Deterministic conversion from {relative}", "o crane_stl"]
    for triangle in triangles:
        for x, y, z in triangle:
            lines.append(f"v {x:.9g} {y:.9g} {z:.9g}")
    for index in range(len(triangles)):
        first = index * 3 + 1
        lines.append(f"f {first} {first + 1} {first + 2}")
    destination.write_text("\n".join(lines) + "\n", encoding="ascii")
    record = {
        "sourcePath": relative,
        "sourceSha256": sha256(source),
        "bytes": source.stat().st_size,
        "unityAssetPath": f"{unity_asset_root}/source/{output_relative}",
        "format": "obj",
        "conversion": "deterministic-stl-to-obj",
        "derivedSha256": sha256(destination),
    }
    copied[source] = record
    return record


def unity_pose(sdf_pose: list[float]) -> dict:
    x, y, z, roll, pitch, yaw = sdf_pose
    if abs(roll) > 1e-6 or abs(pitch) > 1e-6:
        raise ValueError("proof importer currently supports yaw-only SDF poses")
    return {
        "position": [x, z, y],
        "eulerDegrees": [0.0, -math.degrees(yaw), 0.0],
    }


def parse_geometry(element: ET.Element, role: str, resource_roots: list[Path],
                   output_root: Path, unity_asset_root: str,
                   copied: dict[Path, dict]) -> dict:
    geometry = element.find("geometry")
    if geometry is None:
        raise ValueError(f"{role} lacks geometry")
    mesh = geometry.find("mesh")
    if mesh is None:
        raise ValueError(f"proof importer currently requires mesh geometry for {role}")
    uri_element = mesh.find("uri")
    if uri_element is None or not uri_element.text:
        raise ValueError(f"mesh lacks URI for {role}")
    source, relative = resolve_model_uri(uri_element.text, resource_roots)
    asset = prepare_mesh_asset(source, relative, output_root, unity_asset_root, copied)
    dependencies = []
    for dependency in collada_dependencies(source):
        dependency_relative = (Path(relative).parent / dependency.name).as_posix()
        dependencies.append(copy_asset(dependency, dependency_relative, output_root,
                                       unity_asset_root, copied))
    scale = numbers(mesh.findtext("scale"), 3, (1.0, 1.0, 1.0))
    return {
        "name": element.get("name", role),
        "kind": "mesh",
        "asset": asset,
        "dependencies": dependencies,
        "unityScale": [scale[0], scale[2], scale[1]],
    }


def generate(world_path: Path, resource_roots: list[Path], output_root: Path,
             unity_asset_root: str, environment_id: str, upstream: dict) -> dict:
    root = ET.parse(world_path).getroot()
    world = root.find("world")
    if world is None:
        raise ValueError("SDF world element missing")
    copied: dict[Path, dict] = {}
    objects = []
    for model in world.findall("model"):
        model_name = model.get("name", "unnamed-model")
        pose = numbers(model.findtext("pose"), 6, (0, 0, 0, 0, 0, 0))
        collisions = []
        visuals = []
        for link in model.findall("link"):
            collisions.extend(parse_geometry(value, "collision", resource_roots, output_root,
                                               unity_asset_root, copied)
                              for value in link.findall("collision"))
            visuals.extend(parse_geometry(value, "visual", resource_roots, output_root,
                                           unity_asset_root, copied)
                           for value in link.findall("visual"))
        objects.append({
            "semanticId": f"clearpath-{model_name}",
            "semanticRole": "render-only-environment" if not collisions else
                            "static-environment-mesh",
            "sourceModel": model_name,
            "static": model.findtext("static", "false").strip().lower() == "true",
            "unityPose": unity_pose(pose),
            "collisions": collisions,
            "visuals": visuals,
        })
    manifest = {
        "schema": "crane-sdf-reference-import-v1",
        "environmentId": environment_id,
        "source": {
            **upstream,
            "worldPath": world_path.name,
            "worldSha256": sha256(world_path),
            "sdfVersion": root.get("version", ""),
        },
        "conversion": {
            "mode": "offline",
            "coordinateConversion": "Gazebo ENU (x,y,z) to Unity (x,z,y); yaw sign inverted",
            "meshConversion": "DAE preserved; STL converted deterministically to OBJ; Blender/FBX not run",
            "collisionPolicy": "source collision meshes instantiated separately from visuals",
            "materialPolicy": "Unity tooling replaces imported render materials with native HDRP materials",
        },
        "objects": objects,
        "assets": sorted(copied.values(), key=lambda value: value["sourcePath"]),
    }
    output_root.mkdir(parents=True, exist_ok=True)
    (output_root / "manifest.json").write_text(
        json.dumps(manifest, indent=2, sort_keys=True) + "\n", encoding="utf-8")
    return manifest


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("world", type=Path)
    parser.add_argument("--resource-root", type=Path, action="append", required=True)
    parser.add_argument("--output-root", type=Path, required=True)
    parser.add_argument("--unity-asset-root", required=True,
                        help="AssetDatabase path matching output-root, e.g. Assets/Generated/...")
    parser.add_argument("--environment-id", required=True)
    parser.add_argument("--upstream-project", required=True)
    parser.add_argument("--upstream-version", required=True)
    args = parser.parse_args()
    result = generate(args.world.resolve(), [value.resolve() for value in args.resource_root],
                      args.output_root.resolve(), args.unity_asset_root.rstrip("/"),
                      args.environment_id, {
                          "project": args.upstream_project,
                          "version": args.upstream_version,
                      })
    print(json.dumps({"environmentId": result["environmentId"],
                      "objects": len(result["objects"]),
                      "assets": len(result["assets"])}, sort_keys=True))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
