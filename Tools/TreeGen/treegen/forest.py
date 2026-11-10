# SPDX-License-Identifier: GPL-3.0-or-later
"""Stand mode: seeded layout (Poisson-disk or Thomas cluster process) with a species mix, all trees
grown together in one shared shadow grid, then exported with a stand manifest."""
from __future__ import annotations

import json
import os
import time

import numpy as np

from . import GENERATOR_NAME, GENERATOR_VERSION
from .export import export_tree
from .growth import TreeGrower, make_grid
from .rng import stream
from .skeleton import params_hash
from .species import load_species
from .variants import make_log, make_snag

LAYOUT_ID = 0xFFFF   # tree_id slot used for the layout RNG stream


def poisson_disk(rng, w, d, r_min, n_max, tries=30):
    pts = []
    for _ in range(n_max * tries):
        if len(pts) >= n_max:
            break
        p = rng.uniform([0, 0], [w, d])
        if not pts or np.min(np.hypot(*(np.asarray(pts) - p).T)) >= r_min:
            pts.append(p)
    return np.asarray(pts).reshape(-1, 2)


def thomas_cluster(rng, w, d, n_parents, mean_children, sigma, r_min):
    par = rng.uniform([0, 0], [w, d], size=(n_parents, 2))
    pts = []
    for p in par:
        for _ in range(rng.poisson(mean_children)):
            q = p + rng.normal(0, sigma, 2)
            if 0 <= q[0] <= w and 0 <= q[1] <= d and (not pts or np.min(np.hypot(*(np.asarray(pts) - q).T)) >= r_min):
                pts.append(q)
    return np.asarray(pts).reshape(-1, 2)


DEFAULT_STAND = {
    "width_m": 30.0, "depth_m": 30.0, "n_trees": 20, "layout": "poisson", "min_spacing_m": 3.0,
    "cluster": {"n_parents": 5, "mean_children": 5, "sigma_m": 3.0},
    "species_mix": {"beech": 0.45, "spruce": 0.35, "birch": 0.2},
    "age_spread_seasons": 12, "p_snag": 0.08, "p_log": 0.08,
    "light": {"voxel": 0.5},
}


def layout(cfg, forest_seed):
    rng = stream(forest_seed, LAYOUT_ID, "layout")
    if cfg["layout"] == "cluster":
        c = cfg["cluster"]
        xy = thomas_cluster(rng, cfg["width_m"], cfg["depth_m"], c["n_parents"], c["mean_children"],
                            c["sigma_m"], cfg["min_spacing_m"])[: cfg["n_trees"]]
    else:
        xy = poisson_disk(rng, cfg["width_m"], cfg["depth_m"], cfg["min_spacing_m"], cfg["n_trees"])
    names = sorted(cfg["species_mix"])
    w = np.array([cfg["species_mix"][k] for k in names], dtype=float)
    sp_idx = rng.choice(len(names), size=len(xy), p=w / w.sum())
    age = rng.integers(0, cfg["age_spread_seasons"] + 1, len(xy))
    u = rng.random(len(xy))
    variant = np.where(u < cfg["p_snag"], "snag", np.where(u < cfg["p_snag"] + cfg["p_log"], "fallen_log", "live"))
    return [{"tree_id": i, "x": float(xy[i, 0]), "y": float(xy[i, 1]), "species": names[sp_idx[i]],
             "start_offset": int(age[i]), "variant": str(variant[i])} for i in range(len(xy))]


def grow_stand(cfg, forest_seed, out_dir=None, write_obj_files=True, log=print):
    cfg = {**DEFAULT_STAND, **cfg}
    items = layout(cfg, forest_seed)
    species = {n: load_species(n) for n in sorted({it["species"] for it in items})}
    seasons = max(sp["seasons"] for sp in species.values())
    growers = []
    for it in items:
        sp = species[it["species"]]
        start = min(it["start_offset"], seasons - 3)
        growers.append(TreeGrower(sp, it["tree_id"], forest_seed, origin=(it["x"], it["y"], 0.0),
                                  start_season=start, seasons=seasons - start))
    H = max(sp["height_max"] for sp in species.values()) * 1.25 + 4
    m = 6.0
    grid = make_grid((-m, -m, -0.5), (cfg["width_m"] + m, cfg["depth_m"] + m, H), cfg.get("light"))
    t0 = time.time()
    for s in range(seasons):
        live = [g for g in growers if g.start_season <= s]
        P = [g.active_bud_world_positions() for g in live]
        grid.compute(np.concatenate(P) if P else np.zeros((0, 3)))
        for g in live:
            g.step(s, grid)
        if log and (s % 6 == 0 or s == seasons - 1):
            log(f"season {s+1}/{seasons}: nodes={sum(g.n for g in growers)}")
    grid.compute(np.concatenate([g.active_bud_world_positions() for g in growers]))
    t_grow = time.time() - t0
    sks = []
    for g, it in zip(growers, items):
        g.finalize_light(grid)
        sk = g.to_skeleton({"stand_position_unity": [it["x"], 0.0, it["y"]]})
        if it["variant"] == "snag":
            sk = make_snag(sk, forest_seed, it["tree_id"])
        elif it["variant"] == "fallen_log":
            sk = make_log(sk, forest_seed, it["tree_id"])
        sk.meta["variant"] = it["variant"]
        sks.append(sk)
    result = {"skeletons": sks, "items": items, "t_grow_s": t_grow, "species": species}
    if out_dir:
        t1 = time.time()
        os.makedirs(out_dir, exist_ok=True)
        trees = []
        for sk, it in zip(sks, items):
            tdir = f"trees/tree_{it['tree_id']:04d}"
            info = export_tree(sk, species[it["species"]], os.path.join(out_dir, tdir), forest_seed,
                               it["tree_id"], write_obj_files=write_obj_files)
            trees.append({
                "tree_id": it["tree_id"], "species": it["species"], "variant": it["variant"],
                "position_unity": [round(it["x"], 4), 0.0, round(it["y"], 4)], "yaw_deg": 0.0,
                "dir": tdir, "files": info["files"], "skeleton_hash": info["skeleton_hash"],
                "bounds_unity_local": info["bounds_unity"], "metrics": info["metrics"],
                "n_capsules": info["n_capsules"], "n_soft_obstacles": info["n_soft"],
                "tri_counts": info["tri_counts"], "start_season": int(sk.meta.get("start_season", 0)),
                "params_hash": params_hash(species[it["species"]]),
            })
        manifest = {
            "schema": "crane.treegen.stand", "schema_version": 1, "generator": GENERATOR_NAME,
            "generator_version": GENERATOR_VERSION, "forest_seed": int(forest_seed),
            "frame": "unity_lh_yup_m_stand_local (origin = stand corner, ground at y=0)",
            "stand": {k: cfg[k] for k in ("width_m", "depth_m", "n_trees", "layout", "min_spacing_m",
                                          "species_mix", "age_spread_seasons", "p_snag", "p_log")},
            "seasons": seasons, "light": cfg.get("light"),
            "species_params_hash": {n: params_hash(sp) for n, sp in species.items()},
            "timing_s": {"grow": round(t_grow, 3), "export": round(time.time() - t1, 3)},
            "trees": trees,
        }
        with open(os.path.join(out_dir, "stand_manifest.json"), "w") as f:
            json.dump(manifest, f, indent=1)
        result["manifest"] = manifest
    return result
