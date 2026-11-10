"""Aggregate Table 3 row 1 (5x5, travel 1, seeds 200-207, ideal link) from joint_heldout_{a,b}.csv.
Bootstrap RNG seed 12345, B=2000 (the original aggregation script for joint_heldout.md was not saved;
the md numbers are matched up to bootstrap noise).   python3 scripts/joint_heldout_agg.py [DATA_DIR]"""
import csv, os, sys
import numpy as np
from collections import defaultdict
HERE = os.path.dirname(os.path.abspath(__file__))
DATA = sys.argv[1] if len(sys.argv) > 1 else os.environ.get("FS_DATA", os.path.join(HERE, "..", "..", "..", "PerformanceResults", "forest_search"))
d = defaultdict(lambda: defaultdict(dict))
for part in "ab":
    for r in csv.DictReader(open(os.path.join(DATA, f"joint_heldout_{part}.csv"))):
        if r["link"] == "ideal":
            d[(r["planner"], r["strategy"])][int(r["seed"])][int(r["episode"])] = int(r["detected"])
seeds = sorted({s for v in d.values() for s in v})
X = {k: np.array([[v[s][e] for e in range(300)] for s in seeds], float) for k, v in d.items()}
rng = np.random.default_rng(12345)
def ci(A, B, R=2000):
    D = A - B; S, E = D.shape; out = np.empty(R)
    for i in range(R):
        out[i] = np.take_along_axis(D[rng.integers(0, S, S)], rng.integers(0, E, (S, E)), 1).mean()
    return D.mean(), *np.percentile(out, [2.5, 97.5])
f = lambda t: f"{t[0]:+.3f} [{t[1]:+.3f},{t[2]:+.3f}]"
print("seeds", seeds)
for pl in ("greedy", "joint"): print(pl, "O-I", f(ci(X[(pl, "ORACLE")], X[(pl, "IMPLICIT")])))
print("jointO-greedyI", f(ci(X[("joint", "ORACLE")], X[("greedy", "IMPLICIT")])))
