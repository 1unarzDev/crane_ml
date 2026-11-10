"""Dev-seed (100-103) sizing of the lookahead planner: ORACLE greedy vs ORACLE lookahead."""
import sys, time, csv
sys.path.insert(0, "/home/lunarz/worktrees/crane-forest-sar/Tools")
from ForestSearch.instances import grid_instance
from ForestSearch.event_curve import events
from ForestSearch.experiments import LINKS
from ForestSearch.sim import make_strategies, run_episode
from ForestSearch.lookahead import make_lookahead_strategies
CFG = {"A": dict(k=6, n=16, window=16, pairs=3, z=1.0), "B": dict(k=8, n=32, window=24, pairs=4, z=1.0)}
G = make_strategies()
out = open(sys.argv[1], "w", newline=""); w = csv.writer(out)
w.writerow(["grid", "seed", "planner", "episode", "detected", "sec"])
EPS = int(sys.argv[2]) if len(sys.argv) > 2 else 50
for grid in (6, 5):
    for sd in (100, 101, 102, 103):
        inst = grid_instance(grid, grid, seed=sd, travel_scale=1.0, horizon=2 * grid * grid, **events(1.0))
        for name, S in [("greedy", G)] + [(c, make_lookahead_strategies(**v)) for c, v in CFG.items()]:
            for ep in range(EPS):
                t0 = time.time()
                r = run_episode(inst, ("ORACLE", "ORACLE"), LINKS["ideal"], ep, strategies=S)
                w.writerow([grid, sd, name, ep, int(r.detected), round(time.time() - t0, 4)])
            out.flush(); print(grid, sd, name, flush=True)
