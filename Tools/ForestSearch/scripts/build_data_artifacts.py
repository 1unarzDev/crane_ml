"""Compress raw per-episode CSVs into results/data/*.csv.gz (deterministic gzip, mtime=0) and write
results/data/DATA_MANIFEST.json (sha256 of every raw and compact file, Python/numpy versions).

  python3 scripts/build_data_artifacts.py [--raw DIR] [--extra DIR_WITH_dump_episodes_output]
Raw inputs: PerformanceResults/forest_search/{confirm_*,curve_dev_*,joint_heldout_*}.csv, joint_cells/out/*.csv.
The *_episodes.csv.gz dumps from scripts/dump_episodes.py are already compact; they are only hashed.
"""
import argparse, glob, gzip, hashlib, json, os, platform, shutil, sys
import numpy as np
HERE = os.path.dirname(os.path.abspath(__file__)); FS = os.path.abspath(os.path.join(HERE, ".."))
ap = argparse.ArgumentParser()
ap.add_argument("--raw", default=os.environ.get("FS_DATA", os.path.join(FS, "..", "..", "PerformanceResults", "forest_search")))
ap.add_argument("--out", default=os.path.join(FS, "results", "data"))
a = ap.parse_args(); os.makedirs(a.out, exist_ok=True)

def sha(p):
    h = hashlib.sha256()
    with open(p, "rb") as f:
        for b in iter(lambda: f.read(1 << 20), b""): h.update(b)
    return h.hexdigest()

raws = sorted(glob.glob(os.path.join(a.raw, "confirm_*_t?.csv")) + glob.glob(os.path.join(a.raw, "curve_dev_*.csv")) +
              glob.glob(os.path.join(a.raw, "joint_heldout_?.csv")) + glob.glob(os.path.join(a.raw, "joint_cells", "out", "*.csv")) +
              glob.glob(os.path.join(a.raw, "joint_cells", "extra", "*.csv")))
man = {"python": platform.python_version(), "numpy": np.__version__, "platform": platform.platform(), "files": []}
for r in raws:
    rel = os.path.relpath(r, a.raw); name = rel.replace(os.sep, "__") + ".gz"
    dst = os.path.join(a.out, name)
    with open(r, "rb") as i, open(dst, "wb") as o, gzip.GzipFile(fileobj=o, mode="wb", compresslevel=9, mtime=0, filename="") as g:
        shutil.copyfileobj(i, g)
    man["files"].append({"raw": rel, "raw_sha256": sha(r), "raw_bytes": os.path.getsize(r),
                         "compact": name, "compact_sha256": sha(dst), "compact_bytes": os.path.getsize(dst)})
for d in sorted(glob.glob(os.path.join(a.out, "*_episodes.csv.gz"))):
    man["files"].append({"raw": None, "compact": os.path.basename(d), "compact_sha256": sha(d), "compact_bytes": os.path.getsize(d),
                         "note": "written directly by scripts/dump_episodes.py (re-simulation; no out-of-repo raw file)"})
# summary-only artifacts without per-episode data (hash the markdown the paper reads)
for md in ("nagent_pilot.md", "loss_baselines.md", "controls.md", "phantom.md", "leasefix.md", "joint_heldout.md", "joint_heldout_all.md", "hypotheses.md", "confirm.md"):
    p = os.path.join(FS, "results", md)
    if os.path.exists(p):
        man["files"].append({"raw": None, "compact": "../" + md, "compact_sha256": sha(p), "compact_bytes": os.path.getsize(p), "note": "summary table (no per-episode data available)"})
json.dump(man, open(os.path.join(a.out, "DATA_MANIFEST.json"), "w"), indent=1)
print(len(man["files"]), "entries;", sum(f["compact_bytes"] for f in man["files"] if f["compact"].endswith(".gz")) / 1e6, "MB compact")
