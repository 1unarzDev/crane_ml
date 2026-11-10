"""Worker for the joint-vs-greedy held-out cells (Table 3 rows 2-4). Claims (cell,seed) jobs via mkdir so several
workers can share a directory:  fsrun python3 scripts/joint_cells_worker.py [first_seed last_seed_exclusive]
Writes $FS_JH2/out/<cell>_<seed>.csv (default PerformanceResults/forest_search/joint_cells)."""
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
for d in ("claims", "out"): os.makedirs(os.path.join(JH2, d), exist_ok=True)
CELLS = {"c2": (6, 0.0, 16), "c3": (6, 1.0, None), "c1": (5, 0.0, 16)}   # c1 5x5 t0 T16; c2 6x6 t0 T16; c3 6x6 t1
planners = {"greedy": make_strategies(), "joint": make_joint_strategies(6, 14)}
lo, hi = (int(sys.argv[1]), int(sys.argv[2])) if len(sys.argv) > 2 else (200, 208)
for sd in range(lo, hi):
    for c, (n, ts, hz) in CELLS.items():
        try: os.mkdir(os.path.join(JH2, "claims", f"{c}_{sd}"))
        except FileExistsError: continue
        kw = dict(EVENTS["both"], travel_scale=ts)
        if hz: kw["horizon"] = hz
        inst = grid_instance(n, n, seed=sd, **kw); rows = []
        for pn, S in planners.items():
            nom = nominal_plan(inst, S["ORACLE"]); sec = make_sectors(inst)
            for st in ("ORACLE", "IMPLICIT", "INDEP"):
                for ep in range(300):
                    r = run_episode(inst, (st, st), LINKS["ideal"], ep, nominal=nom, sectors=sec, strategies=S)
                    rows.append((pn, sd, st, ep, int(r.detected)))
        tmp = os.path.join(JH2, "out", f"{c}_{sd}.csv.tmp")
        with open(tmp, "w", newline="") as f:
            w = csv.writer(f); w.writerow(["planner", "seed", "strategy", "episode", "detected"]); w.writerows(rows)
        os.rename(tmp, os.path.join(JH2, "out", f"{c}_{sd}.csv"))
        print("done", c, sd, flush=True)
