"""Non-greedy held-out worker. Claims jobs via mkdir in OUT/claims; writes OUT/out/<job>.csv.
usage: ng_worker.py OUT EPISODES_LOOKAHEAD k n window pairs z  (greedy runs use 300 episodes)
Jobs: part A (greedy ORACLE/IMPLICIT/INDEP + lookahead ORACLE) for all (cell, lam, seed) first,
then part B (lookahead IMPLICIT with the lookahead nominal plan)."""
import os, sys, time, csv
sys.path.insert(0, "/home/lunarz/worktrees/crane-forest-sar/Tools")
from ForestSearch.instances import grid_instance
from ForestSearch.event_curve import events
from ForestSearch.experiments import LINKS
from ForestSearch.sim import make_strategies, nominal_plan, make_sectors, run_episode
from ForestSearch.lookahead import make_lookahead_strategies
OUT, EPS = sys.argv[1], int(sys.argv[2])
k, n, w, pairs, z = int(sys.argv[3]), int(sys.argv[4]), int(sys.argv[5]), int(sys.argv[6]), float(sys.argv[7])
for d in ("claims", "out"): os.makedirs(os.path.join(OUT, d), exist_ok=True)
G = make_strategies(); L = make_lookahead_strategies(k=k, n=n, window=w, pairs=pairs, z=z)
CELLS = {"6x6t1": 6, "5x5t1": 5}
jobs = [("A", c, lam, sd) for lam in (1.0, 3.0) for c in CELLS for sd in range(200, 208)]
jobs += [("B", c, lam, sd) for lam in (1.0, 3.0) for c in CELLS for sd in range(200, 208)]
link = LINKS["ideal"]
ONLY = sys.argv[8] if len(sys.argv) > 8 else ""   # optional job-name prefix filter
for part, c, lam, sd in jobs:
    name = f"{part}_{c}_l{int(lam)}_{sd}"
    if not name.startswith(ONLY): continue
    try: os.mkdir(os.path.join(OUT, "claims", name))
    except FileExistsError: continue
    g = CELLS[c]
    inst = grid_instance(g, g, seed=sd, travel_scale=1.0, horizon=2 * g * g, **events(lam))
    rows = []
    t0 = time.time(); nomL = nominal_plan(inst, L["ORACLE"]); t_nom = time.time() - t0
    if part == "A":
        nomG = nominal_plan(inst, G["ORACLE"]); sec = make_sectors(inst)
        runs = [("greedy", s, G, nomG) for s in ("ORACLE", "IMPLICIT", "INDEP")] + [("lookahead", "ORACLE", L, nomL)]
    else:
        runs = [("lookahead", "IMPLICIT", L, nomL)]
    for pl, st, S, nom in runs:
        for ep in range(300 if pl == "greedy" else EPS):
            t1 = time.time()
            r = run_episode(inst, (st, st), link, ep, nominal=nom, strategies=S)
            rows.append((c, lam, sd, pl, st, ep, int(r.detected), r.ttd, round(time.time() - t1, 4)))
    tmp = os.path.join(OUT, "out", name + ".csv.tmp")
    with open(tmp, "w", newline="") as f:
        wr = csv.writer(f); wr.writerow(["cell", "lam", "seed", "planner", "strategy", "episode", "detected", "ttd", "sec"])
        wr.writerows(rows)
    os.rename(tmp, os.path.join(OUT, "out", name + ".csv"))
    print("done", name, f"nominal {t_nom:.1f}s", f"{time.time()-t0:.0f}s", flush=True)
