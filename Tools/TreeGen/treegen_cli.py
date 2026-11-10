#!/usr/bin/env python3
# SPDX-License-Identifier: GPL-3.0-or-later
"""CRANE TreeGen command line.

  python treegen_cli.py library --out DIR [--species beech,spruce,birch,snag,fallen_log] [--count 2] [--seed 7]
  python treegen_cli.py stand   --out DIR [--config stand.json] [--seed 7] [--n 20] [--layout poisson|cluster]
  python treegen_cli.py preview --out PNG_DIR (--stand DIR | --library DIR)
  python treegen_cli.py atlas   --out leaf_atlas.png [--size 1024]
Run under the compute policy wrapper: fsrun <python> treegen_cli.py ...
"""
from __future__ import annotations

import argparse
import json
import os
import sys
import time

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

from treegen import GENERATOR_NAME, GENERATOR_VERSION  # noqa: E402
from treegen.export import export_tree  # noqa: E402
from treegen.forest import grow_stand  # noqa: E402
from treegen.growth import grow_tree  # noqa: E402
from treegen.skeleton import Skeleton, params_hash  # noqa: E402
from treegen.species import load_species  # noqa: E402
from treegen.variants import make_log, make_snag  # noqa: E402


def grow_variant(name, seed, tid, seasons=None):
    sp = load_species(name)
    sk = grow_tree(sp, seed, tid, seasons=seasons)
    v = sp.get("variant", "live")
    if v == "snag":
        sk = make_snag(sk, seed, tid)
    elif v == "fallen_log":
        sk = make_log(sk, seed, tid)
    sk.meta["variant"] = v
    sk.meta["species"] = sp["name"]
    return sp, sk


def cmd_library(a):
    os.makedirs(a.out, exist_ok=True)
    trees = []
    tid = 0
    ages = [float(x) for x in a.age_classes.split(",")] if a.age_classes else [1.0]
    for name in a.species.split(","):
        for k in range(a.count):
            t0 = time.time()
            base = load_species(name)
            f = ages[k % len(ages)] if base.get("variant", "live") == "live" else 1.0
            seasons = max(6, int(round(base["seasons"] * f))) if f != 1.0 else None
            sp, sk = grow_variant(name, a.seed, tid, seasons)
            tg = time.time() - t0
            tdir = f"trees/tree_{tid:04d}"
            info = export_tree(sk, sp, os.path.join(a.out, tdir), a.seed, tid, write_obj_files=not a.no_obj)
            te = time.time() - t0 - tg
            print(f"tree {tid} {name}: nodes={sk.n} H={info['metrics']['height_m']} dbh={info['metrics']['dbh_m']} "
                  f"caps={info['n_capsules']} grow={tg:.2f}s export={te:.2f}s")
            trees.append({"tree_id": tid, "species": name, "variant": sk.meta.get("variant", "live"),
                          "position_unity": [0.0, 0.0, 0.0], "yaw_deg": 0.0, "dir": tdir,
                          "files": info["files"], "skeleton_hash": info["skeleton_hash"],
                          "bounds_unity_local": info["bounds_unity"], "metrics": info["metrics"],
                          "n_capsules": info["n_capsules"], "n_soft_obstacles": info["n_soft"],
                          "tri_counts": info["tri_counts"], "params_hash": params_hash(sp),
                          "timing_s": {"grow": round(tg, 3), "export": round(te, 3)}})
            tid += 1
    man = {"schema": "crane.treegen.library", "schema_version": 1, "generator": GENERATOR_NAME,
           "generator_version": GENERATOR_VERSION, "forest_seed": a.seed,
           "frame": "unity_lh_yup_m_tree_local (each tree's root at its local origin)", "trees": trees}
    with open(os.path.join(a.out, "library_manifest.json"), "w") as f:
        json.dump(man, f, indent=1)


def cmd_stand(a):
    cfg = {}
    if a.config:
        with open(a.config) as f:
            cfg = json.load(f)
    if a.n:
        cfg["n_trees"] = a.n
    if a.layout:
        cfg["layout"] = a.layout
    if a.size:
        cfg["width_m"] = cfg["depth_m"] = a.size
    t0 = time.time()
    res = grow_stand(cfg, a.seed, a.out, write_obj_files=not a.no_obj)
    man = res["manifest"]
    print(f"stand: {len(man['trees'])} trees, grow {man['timing_s']['grow']}s export {man['timing_s']['export']}s "
          f"total {time.time()-t0:.1f}s")


def cmd_preview(a):
    from treegen.preview import render_trees
    import numpy as np
    os.makedirs(a.out, exist_ok=True)
    src = a.stand or a.library
    mf = "stand_manifest.json" if a.stand else "library_manifest.json"
    with open(os.path.join(src, mf)) as f:
        man = json.load(f)
    if a.stand:
        items = []
        for t in man["trees"]:
            sk = Skeleton.load_json(os.path.join(src, t["dir"], "skeleton.json"))
            pu = t["position_unity"]
            items.append((sk, (pu[0], pu[2], pu[1])))
        render_trees(items, os.path.join(a.out, f"stand_seed{man['forest_seed']}.png"),
                     f"stand seed {man['forest_seed']} ({len(items)} trees, shared light field)", min_r=0.004)
    else:
        for t in man["trees"]:
            sk = Skeleton.load_json(os.path.join(src, t["dir"], "skeleton.json"))
            m = t["metrics"]
            render_trees([(sk, (0, 0, 0))], os.path.join(a.out, f"tree_{t['tree_id']:04d}_{t['species']}.png"),
                         f"{t['species']} ({t['variant']}) id {t['tree_id']}: H {m['height_m']} m, DBH {m['dbh_m']} m, "
                         f"crown ratio {m['crown_ratio']}, nodes {m['nodes']}")


def cmd_atlas(a):
    from treegen.atlas import make_atlas
    os.makedirs(os.path.dirname(os.path.abspath(a.out)), exist_ok=True)
    make_atlas(a.out, a.size)
    print(f"atlas -> {a.out}")


def main(argv=None):
    p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    sub = p.add_subparsers(dest="cmd", required=True)
    l = sub.add_parser("library")
    l.add_argument("--out", required=True)
    l.add_argument("--species", default="beech,spruce,birch,snag,fallen_log")
    l.add_argument("--count", type=int, default=1)
    l.add_argument("--seed", type=int, default=7)
    l.add_argument("--no-obj", action="store_true")
    l.add_argument("--age-classes", help="comma list of season fractions cycled over the live variants of each species, "
                   "e.g. 0.45,0.7,1,1 gives young, mid and two mature trees (size classes for the forest provider)")
    s = sub.add_parser("stand")
    s.add_argument("--out", required=True)
    s.add_argument("--config")
    s.add_argument("--seed", type=int, default=7)
    s.add_argument("--n", type=int)
    s.add_argument("--size", type=float)
    s.add_argument("--layout", choices=["poisson", "cluster"])
    s.add_argument("--no-obj", action="store_true")
    v = sub.add_parser("preview")
    v.add_argument("--out", required=True)
    v.add_argument("--stand")
    v.add_argument("--library")
    t = sub.add_parser("atlas")
    t.add_argument("--out", required=True)
    t.add_argument("--size", type=int, default=1024)
    a = p.parse_args(argv)
    {"library": cmd_library, "stand": cmd_stand, "preview": cmd_preview, "atlas": cmd_atlas}[a.cmd](a)


if __name__ == "__main__":
    main()
