# SPDX-License-Identifier: GPL-3.0-or-later
"""Loaders for the logged rows used by the D4 retrospective (read-only; no run is started).

Fidelity labels: L1 = NavPilot 2.5D point mass; L2 = CRANE .NET harness (3D capsules, RK4 plant); L3 = CRANE Unity colliders.
Split labels: DEV, RETRO-HELDOUT (frozen held-out rows of an earlier study, reused for a retrospective analysis only:
no new held-out run is made and nothing is tuned on them).

CRANE rows live in /tmp (volatile): `snapshot()` copies them to ~/.cache/frontier/mf_data with a sha256 manifest.
"""
import glob
import hashlib
import json
import os
import shutil

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.normpath(os.path.join(HERE, "..", "..", ".."))
RAW = os.path.join(ROOT, "Tools", "NavPilot", "results", "raw")
SNAP = os.path.join(os.path.expanduser("~"), ".cache", "frontier", "mf_data")
TMP_SOURCES = {"dotnet_swA": "/tmp/ah_swA/results.jsonl", "dotnet_swB": "/tmp/ah_swB/results.jsonl",
               "dotnet_m5a": "/tmp/ah_m5a/results.jsonl", "dotnet_m5b": "/tmp/ah_m5b/results.jsonl",
               "unity_m5": "/tmp/autonomy_unity/m5/summary.jsonl", "navpilot_twin": os.path.join(os.path.expanduser("~"), ".cache", "frontier", "crane", "harness", "navpilot_fidelity.jsonl")}


def _sha(p):
    with open(p, "rb") as f:
        return hashlib.sha256(f.read()).hexdigest()


def snapshot():
    """Copy the volatile /tmp rows once; later runs read the snapshot. Returns the manifest."""
    os.makedirs(SNAP, exist_ok=True)
    mpath = os.path.join(SNAP, "manifest.json")
    man = json.load(open(mpath)) if os.path.exists(mpath) else {}
    for name, src in TMP_SOURCES.items():
        dst = os.path.join(SNAP, name + ".jsonl")
        if not os.path.exists(dst) and os.path.exists(src):
            shutil.copyfile(src, dst)
            man[name] = {"src": src, "sha256": _sha(dst), "rows": sum(1 for _ in open(dst))}
    json.dump(man, open(mpath, "w"), indent=1, sort_keys=True)
    return man


def _read(path):
    with open(path) as f:
        return [json.loads(l) for l in f if l.strip()]


def navpilot_e3(plan="heldout"):
    pat = "e3_heldout_0*.jsonl" if plan == "heldout" else "e3_tune_*.jsonl"
    rows = []
    for f in sorted(glob.glob(os.path.join(RAW, pat))):
        for r in _read(f):
            rows.append({"fid": "L1", "source": "navpilot_e3_" + plan, "split": "RETRO-HELDOUT" if plan == "heldout" else "DEV",
                         "config": r["stack"], "forest": int(r["layout"]), "scenario": r["family"], "speed": float(r["v_lim"]),
                         "seed": int(r["layout"]), "collision": bool(r["collision"]), "success": bool(r["success"]),
                         "outcome": r["outcome"], "progress": float(r["progress_rate"]), "min_clear": r["min_clear"],
                         "wall_s": r["wall_s"], "tune": r.get("cond")})
    return rows


def navpilot_e2(plan="heldout"):
    rows = []
    for f in sorted(glob.glob(os.path.join(RAW, "e2_heldout_shard*.jsonl"))):
        for r in _read(f):
            rows.append({"fid": "L1", "source": "navpilot_e2_heldout", "split": "RETRO-HELDOUT", "config": r["stack"],
                         "forest": int(r["layout"]), "scenario": r["family"], "speed": float(r["v_lim"]), "seed": int(r["layout"]),
                         "collision": bool(r["collision"]), "success": bool(r["success"]), "outcome": r["outcome"],
                         "progress": None, "min_clear": r.get("min_clear"), "wall_s": r["wall_s"], "tune": r.get("cond")})
    return rows


def navpilot_dev_sweep():
    rows = []
    for f in sorted(glob.glob(os.path.join(RAW, "dev_shard*.jsonl"))):
        for r in _read(f):
            rows.append({"fid": "L1", "source": "navpilot_dev_sweep", "split": "DEV", "config": "dev_sweep_stack",
                         "forest": int(r["seed"]), "scenario": "%s|%s|%s" % (r["branch"], r["sensor"], r["latency_tail"]),
                         "speed": float(r["v_limit"]), "seed": int(r["seed"]), "collision": bool(r["collision"]),
                         "success": bool(r["success"]), "outcome": r["outcome"], "progress": r.get("progress_m"),
                         "min_clear": r.get("near_miss_m"), "wall_s": r["wall_s"], "tune": None})
    return rows


def _crane_row(r, fid, source):
    return {"fid": fid, "source": source, "split": "DEV", "config": "map_rhp", "forest": int(r["ForestSeed"]),
            "scenario": r["Name"], "speed": float(r["VLimit"]), "seed": int(r["RunSeed"]), "collision": bool(r["Collision"]),
            "success": bool(r["Success"]), "outcome": r["Outcome"], "progress": r["ProgressFrac"],
            "min_clear": r["MinClearanceHardM"], "wall_s": r.get("WallS"), "tune": "flags_off", "mean_speed": r.get("MeanSpeed")}


def crane_dotnet():
    snapshot()
    rows, seen = [], set()
    for name in ("dotnet_swA", "dotnet_swB", "dotnet_m5a", "dotnet_m5b"):
        p = os.path.join(SNAP, name + ".jsonl")
        if os.path.exists(p):
            for r in _read(p):
                k = (r["Name"], r["VLimit"], r["RunSeed"])
                if k not in seen:  # first row wins, as Tools/CraneHarness/fidelity.py
                    seen.add(k)
                    rows.append(_crane_row(r, "L2", name))
    return rows


def crane_unity():
    snapshot()
    p = os.path.join(SNAP, "unity_m5.jsonl")
    rows, seen = [], set()
    for r in (_read(p) if os.path.exists(p) else []):
        k = (r["Name"], r["VLimit"], r["RunSeed"])
        if k not in seen:
            seen.add(k)
            rows.append(_crane_row(r, "L3", "unity_m5"))
    return rows


def navpilot_twin():
    """run_pending stage 2 (ops agent): NavPilot 2.5D twin on the 10 exported cross_k stands, 7 speeds, run seed 1."""
    snapshot()
    p = os.path.join(SNAP, "navpilot_twin.jsonl")
    out = []
    for r in (_read(p) if os.path.exists(p) else []):
        out.append({"fid": "L1", "source": "navpilot_twin", "split": "DEV", "config": "map_rhp", "forest": r["forest_seed"],
                    "scenario": r["mission"], "speed": float(r["speed"]), "seed": r["run_seed"], "collision": r["collision"],
                    "success": r["success"], "outcome": r["outcome"], "progress": r["progress_frac"], "min_clear": r["min_clear"],
                    "wall_s": r.get("wall_s"), "tune": "twin", "mean_speed": r.get("mean_speed")})
    return out


def load_all():
    return {"navpilot_e3_heldout": navpilot_e3("heldout"), "navpilot_e3_tune": navpilot_e3("tune"),
            "navpilot_e2_heldout": navpilot_e2(), "navpilot_dev_sweep": navpilot_dev_sweep(),
            "crane_dotnet": crane_dotnet(), "crane_unity": crane_unity(), "navpilot_twin": navpilot_twin()}
