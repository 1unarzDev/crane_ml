"""Exploratory: joint vs greedy on held-out cell. args: n travel horizon(or -1) out"""
import os, sys
HERE = os.path.dirname(os.path.abspath(__file__))
FS = os.path.abspath(os.path.join(HERE, ".."))                      # Tools/ForestSearch
sys.path.insert(0, os.path.dirname(FS))                              # Tools/ (package parent)
RES = os.environ.get("FS_RESULTS", os.path.join(FS, "results", "_regen"))  # regenerated tables; diff against results/*.md
os.makedirs(RES, exist_ok=True)
DATA = os.environ.get("FS_DATA", os.path.join(FS, "..", "..", "PerformanceResults", "forest_search"))
JH2 = os.environ.get("FS_JH2", os.path.join(DATA, "joint_cells"))    # per-seed joint-cell CSVs

import csv
from ForestSearch.instances import grid_instance
from ForestSearch.experiments import EVENTS, LINKS
from ForestSearch.sim import make_strategies, nominal_plan, make_sectors, run_episode
from ForestSearch.joint import make_joint_strategies
n_ = int(sys.argv[1]); ts = float(sys.argv[2]); hz = int(sys.argv[3]); out = sys.argv[4]
kw = dict(EVENTS["both"], travel_scale=ts)
if hz > 0: kw["horizon"] = hz
planners = {"greedy": make_strategies(), "joint": make_joint_strategies(6, 14)}
rows = []
for sd in range(200, 208):
    inst = grid_instance(n_, n_, seed=sd, **kw)
    for pn, S in planners.items():
        nom = nominal_plan(inst, S["ORACLE"]); sec = make_sectors(inst)
        for n in ("ORACLE", "IMPLICIT", "INDEP"):
            for ep in range(300):
                r = run_episode(inst, (n, n), LINKS["ideal"], ep, nominal=nom, sectors=sec, strategies=S)
                rows.append((pn, sd, n, ep, int(r.detected)))
    print("done seed", sd, flush=True)
with open(out, "w", newline="") as f:
    w = csv.writer(f); w.writerow(["planner","seed","strategy","episode","detected"]); w.writerows(rows)
