"""Exploratory: joint planner vs greedy on held-out seeds (5x5, travel 1, 'both' events)."""
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
seeds = [int(s) for s in sys.argv[1].split(',')]
out = sys.argv[2]
planners = {"greedy": make_strategies(), "joint": make_joint_strategies(6, 14)}
rows = []
for sd in seeds:
    inst = grid_instance(5, 5, seed=sd, **EVENTS["both"])
    for pn, S in planners.items():
        nom = nominal_plan(inst, S["ORACLE"]); sec = make_sectors(inst)
        for ln in ("ideal", "burst30_L10_d1"):
            for n in ("ORACLE", "IMPLICIT", "FULL", "INDEP"):
                for ep in range(300):
                    r = run_episode(inst, (n, n), LINKS[ln], ep, nominal=nom, sectors=sec, strategies=S)
                    rows.append((pn, sd, ln, n, ep, int(r.detected)))
    print("done seed", sd, flush=True)
with open(out, "w", newline="") as f:
    w = csv.writer(f); w.writerow(["planner", "seed", "link", "strategy", "episode", "detected"]); w.writerows(rows)
