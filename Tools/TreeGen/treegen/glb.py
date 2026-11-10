# SPDX-License-Identifier: GPL-3.0-or-later
"""Minimal glTF 2.0 binary (.glb) and Wavefront OBJ writers (numpy only).

Mesh vertices passed in are in the internal right-handed Z-up frame. They are written as glTF/OBJ
coordinates (-x, z, y): a proper rotation (det +1, winding preserved) chosen so that an importer
which converts right-handed files to Unity by negating X (Unity's OBJ importer, glTFast) lands on the
TreeGen "unity" frame (x, z, y) used by every JSON sidecar.
"""
from __future__ import annotations

import json
import struct

import numpy as np


def to_file_frame(p: np.ndarray) -> np.ndarray:
    p = np.asarray(p, dtype=np.float64)
    return np.stack([-p[..., 0], p[..., 2], p[..., 1]], -1)


class MeshPart:
    def __init__(self, name, pos, nrm, uv, tri, material):
        self.name = name
        self.pos = np.asarray(pos, np.float32).reshape(-1, 3)
        self.nrm = np.asarray(nrm, np.float32).reshape(-1, 3)
        self.uv = np.asarray(uv, np.float32).reshape(-1, 2)
        self.tri = np.asarray(tri, np.uint32).reshape(-1, 3)
        self.material = material   # dict(name, color(rgb), double_sided, alpha_mask, texture_hint)

    @property
    def empty(self):
        return len(self.tri) == 0


def write_glb(path, parts: list[MeshPart], extras: dict | None = None) -> None:
    parts = [p for p in parts if not p.empty]
    bin_chunks = []
    offset = 0
    buffer_views, accessors, meshes, nodes, materials = [], [], [], [], []

    def add_view(arr: np.ndarray, target):
        nonlocal offset
        data = arr.tobytes()
        pad = (-len(data)) % 4
        bv = {"buffer": 0, "byteOffset": offset, "byteLength": len(data)}
        if target:
            bv["target"] = target
        buffer_views.append(bv)
        bin_chunks.append(data + b"\x00" * pad)
        offset += len(data) + pad
        return len(buffer_views) - 1

    def add_accessor(arr, comp, typ, target, minmax=False):
        v = add_view(arr, target)
        acc = {"bufferView": v, "componentType": comp, "count": int(arr.shape[0]), "type": typ}
        if minmax:
            acc["min"] = arr.min(0).astype(float).tolist()
            acc["max"] = arr.max(0).astype(float).tolist()
        accessors.append(acc)
        return len(accessors) - 1

    for part in parts:
        P = to_file_frame(part.pos).astype(np.float32)
        N = to_file_frame(part.nrm).astype(np.float32)
        uv = part.uv.copy()
        uv[:, 1] = 1.0 - uv[:, 1]           # glTF UV origin is top-left
        ia = add_accessor(P, 5126, "VEC3", 34962, minmax=True)
        na = add_accessor(N, 5126, "VEC3", 34962)
        ta = add_accessor(uv.astype(np.float32), 5126, "VEC2", 34962)
        xa = add_accessor(part.tri.reshape(-1).astype(np.uint32), 5125, "SCALAR", 34963)
        m = part.material
        mat = {"name": m["name"], "pbrMetallicRoughness": {"baseColorFactor": list(m["color"]) + [1.0],
                                                            "metallicFactor": 0.0, "roughnessFactor": 0.9},
               "doubleSided": bool(m.get("double_sided", False))}
        if m.get("alpha_mask"):
            mat["alphaMode"] = "MASK"
            mat["alphaCutoff"] = 0.5
        if m.get("texture_hint"):
            mat["extras"] = {"texture_hint": m["texture_hint"]}
        materials.append(mat)
        meshes.append({"name": part.name, "primitives": [{
            "attributes": {"POSITION": ia, "NORMAL": na, "TEXCOORD_0": ta}, "indices": xa,
            "material": len(materials) - 1, "mode": 4}]})
        nodes.append({"name": part.name, "mesh": len(meshes) - 1})
    root = {"name": "tree", "children": list(range(len(nodes)))}
    if extras:
        root["extras"] = extras
    nodes.append(root)
    gltf = {"asset": {"version": "2.0", "generator": "crane-treegen"}, "scene": 0,
            "scenes": [{"nodes": [len(nodes) - 1]}], "nodes": nodes, "meshes": meshes,
            "materials": materials, "accessors": accessors, "bufferViews": buffer_views,
            "buffers": [{"byteLength": offset}]}
    js = json.dumps(gltf, separators=(",", ":")).encode()
    js += b" " * ((-len(js)) % 4)
    binblob = b"".join(bin_chunks)
    total = 12 + 8 + len(js) + 8 + len(binblob)
    with open(path, "wb") as f:
        f.write(struct.pack("<III", 0x46546C67, 2, total))
        f.write(struct.pack("<II", len(js), 0x4E4F534A))
        f.write(js)
        f.write(struct.pack("<II", len(binblob), 0x004E4942))
        f.write(binblob)


def read_glb(path) -> dict:
    """Parse a .glb back into its JSON dict plus decoded attribute arrays (for tests)."""
    with open(path, "rb") as f:
        data = f.read()
    magic, ver, total = struct.unpack_from("<III", data, 0)
    assert magic == 0x46546C67 and ver == 2 and total == len(data)
    jl, jt = struct.unpack_from("<II", data, 12)
    assert jt == 0x4E4F534A
    gl = json.loads(data[20:20 + jl])
    bl, bt = struct.unpack_from("<II", data, 20 + jl)
    assert bt == 0x004E4942
    blob = data[28 + jl:28 + jl + bl]
    dt = {5126: np.float32, 5125: np.uint32}
    nc = {"SCALAR": 1, "VEC2": 2, "VEC3": 3}

    def acc(i):
        a = gl["accessors"][i]
        v = gl["buffer_views"][a["bufferView"]] if "buffer_views" in gl else gl["bufferViews"][a["bufferView"]]
        arr = np.frombuffer(blob, dtype=dt[a["componentType"]], count=a["count"] * nc[a["type"]],
                            offset=v["byteOffset"])
        return arr.reshape(a["count"], nc[a["type"]]) if nc[a["type"]] > 1 else arr

    out = {"json": gl, "meshes": {}}
    for m in gl["meshes"]:
        pr = m["primitives"][0]
        out["meshes"][m["name"]] = {k: acc(v) for k, v in pr["attributes"].items()}
        out["meshes"][m["name"]]["indices"] = acc(pr["indices"])
    return out


def write_obj(path, part: MeshPart, mtl_name: str | None = None) -> None:
    """One part per OBJ (+ .mtl). Same file frame as the glb."""
    P = to_file_frame(part.pos)
    N = to_file_frame(part.nrm)
    stem = str(path)[:-4]
    mtl = mtl_name or part.material["name"]
    with open(stem + ".mtl", "w") as f:
        c = part.material["color"]
        f.write(f"newmtl {mtl}\nKd {c[0]:.4f} {c[1]:.4f} {c[2]:.4f}\nKa 0 0 0\nKs 0 0 0\nd 1\nillum 1\n")
        if part.material.get("texture_hint"):
            f.write(f"# texture_hint {part.material['texture_hint']}\n")
    lines = [f"# crane-treegen {part.name}\nmtllib {stem.split('/')[-1]}.mtl\no {part.name}\n"]
    lines.append("\n".join(f"v {x:.5f} {y:.5f} {z:.5f}" for x, y, z in P))
    lines.append("\n".join(f"vt {u:.5f} {v:.5f}" for u, v in part.uv))
    lines.append("\n".join(f"vn {x:.4f} {y:.4f} {z:.4f}" for x, y, z in N))
    lines.append(f"usemtl {mtl}")
    t = part.tri + 1
    lines.append("\n".join(f"f {a}/{a}/{a} {b}/{b}/{b} {c}/{c}/{c}" for a, b, c in t))
    with open(path, "w") as f:
        f.write("\n".join(lines) + "\n")
