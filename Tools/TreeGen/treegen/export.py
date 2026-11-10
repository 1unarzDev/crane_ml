# SPDX-License-Identifier: GPL-3.0-or-later
"""Write every per-tree artefact (see CONTRACT.md) for one grown skeleton."""
from __future__ import annotations

import json
import os

import numpy as np

from . import colliders as col
from . import metrics
from .glb import write_glb, write_obj
from .mesh import build_lods
from .skeleton import to_unity


def bounds_unity(sk):
    lo = sk.pos.min(0) - sk.radius.max()
    hi = sk.pos.max(0) + sk.radius.max()
    a, b = to_unity(lo), to_unity(hi)
    return {"min": np.minimum(a, b).round(4).tolist(), "max": np.maximum(a, b).round(4).tolist()}


def export_tree(sk, species, out_dir, forest_seed, tree_id, collider_cfg=None, write_obj_files=True,
                lods=(0, 1, 2)):
    os.makedirs(out_dir, exist_ok=True)
    with_leaves = bool(sk.alive.any())
    lod_parts, leaf_centres, foliage = build_lods(sk, species, forest_seed, tree_id, lods, with_leaves)
    caps, softs, cfg = col.build_colliders(sk, collider_cfg)
    summ = metrics.summary(sk)
    sk.meta["metrics"] = summ
    sk.meta["bounds_unity"] = bounds_unity(sk)
    sk.save_json(os.path.join(out_dir, "skeleton.json"))
    cj = col.colliders_json(sk, caps, softs, cfg, foliage)
    with open(os.path.join(out_dir, "colliders.json"), "w") as f:
        json.dump(cj, f, separators=(",", ":"))
    files = {"skeleton": "skeleton.json", "colliders": "colliders.json", "lods": {}}
    stats = {}
    for lod, (bark, leaves) in lod_parts.items():
        parts = [bark] + ([leaves] if leaves is not None and not leaves.empty else [])
        name = f"lod{lod}.glb"
        write_glb(os.path.join(out_dir, name), parts,
                  extras={"tree_id": tree_id, "lod": lod, "skeleton_hash": sk.topology_hash(),
                          "frame_note": "file coords = (-x_u, y_u, z_u); importer X-negation yields unity frame"})
        entry = {"glb": name}
        if write_obj_files:
            os.makedirs(os.path.join(out_dir, "obj"), exist_ok=True)
            for p in parts:
                fn = f"obj/lod{lod}_{p.name}.obj"
                write_obj(os.path.join(out_dir, fn), p)
                entry[f"obj_{p.name}"] = fn
        files["lods"][str(lod)] = entry
        stats[str(lod)] = {"bark_tris": int(len(bark.tri)),
                           "leaf_tris": int(len(leaves.tri)) if leaves is not None else 0}
    return {"files": files, "metrics": summ, "tri_counts": stats, "n_capsules": len(caps),
            "n_soft": len(softs), "skeleton_hash": sk.topology_hash(), "bounds_unity": sk.meta["bounds_unity"],
            "leaf_centres": leaf_centres}
