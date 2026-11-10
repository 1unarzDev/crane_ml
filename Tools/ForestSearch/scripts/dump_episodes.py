"""Re-run a driver's simulations and dump PER-EPISODE outcomes to gzip CSV (the original drivers kept only
summary markdown).  Same instances, strategies, links and episode indices as the originals, so the numbers
in controls.md / phantom.md / leasefix.md / loss_baselines.md are recomputable from the dumps.

  fsrun python3 scripts/dump_episodes.py controls|phantom|leasefix|loss  [--out DIR]
Output: DIR/<name>_episodes.csv.gz (default results/data). Columns: set,lam,seed,link,strategy,episode,detected,collisions.
"""
import argparse, csv, gzip, os, sys
HERE = os.path.dirname(os.path.abspath(__file__))
FS = os.path.abspath(os.path.join(HERE, ".."))
sys.path.insert(0, os.path.dirname(FS))
import numpy as np
from ForestSearch.event_curve import events, LINKS
from ForestSearch.instances import grid_instance
from ForestSearch.sim import nominal_plan, make_sectors

ap = argparse.ArgumentParser(); ap.add_argument("which", choices=["controls", "phantom", "leasefix", "loss"])
ap.add_argument("--out", default=os.path.join(FS, "results", "data")); a = ap.parse_args()
os.makedirs(a.out, exist_ok=True)
fn = os.path.join(a.out, f"{a.which}_episodes.csv.gz")
w = gzip.open(fn, "wt", newline=""); cw = csv.writer(w)
cw.writerow(["set", "lam", "seed", "link", "strategy", "episode", "detected", "collisions"])

if a.which in ("controls", "phantom", "leasefix"):
    from ForestSearch import controls as C
    mk, run, names = {
        "controls": (C.make_control_strategies, C.run_ctrl, ["IMPLICIT", "HYBRID", "HYBRID_P", "HYBRID_O", "HYBRID_S"]),
        "phantom": (C.make_phantom_strategies, C.run_ph, ["IMPLICIT", "HYBRID", "HYBRID_P", "HYBRID_O", "PH_orc", "NP_none", "NP_nom", "NP_orc", "NP_pred"]),
        "leasefix": (C.make_fixed_strategies, C.run_ph, ["IMPLICIT", "PH_nom", "PH_pred", "PH_orc", "FX_nom", "FX_pred", "FX_orc"]),
    }[a.which]
    S = mk()
    for lam in (1.0, 3.0):
        for sd in (600, 601, 602, 603):
            inst = grid_instance(5, 5, seed=sd, travel_scale=0.0, horizon=16, **events(lam))
            nom = nominal_plan(inst, S["ORACLE"] if "ORACLE" in S else S["IMPLICIT"]); sec = make_sectors(inst)
            for n in names:
                for ep in range(300):
                    r, c = run(inst, (n, n), LINKS["d1_loss0"], ep, S, nom, sec, keep_log=True)
                    cw.writerow([a.which, lam, sd, "d1_loss0", n, ep, int(r.detected), c])
            print(a.which, lam, sd, flush=True)
else:
    from ForestSearch import loss_baselines as L
    for key in ["R7a", "R7b", "R7c", "R7d"]:
        # re-implements run_regime, keeping episodes
        desc, build, fault = L.regimes()[key]; S = L.make_strategies(); links = ["d0_loss0", "d1_burst30"]
        for sd in range(504, 512):
            inst = build(sd); nom = nominal_plan(inst, S["ORACLE"]); sec = make_sectors(inst)
            for ln in links:
                for n in L.ALL:
                    if ln != links[0] and n not in ("HYBRID_P", "HEALTH", "HEARTBEAT"): continue
                    for ep in range(200):
                        r = L.run_l(inst, n, ep, S, nom, sec, fault, ln)
                        cw.writerow([key, "", sd, ln, n, ep, int(r.detected), ""])
            print(key, sd, flush=True)
w.close(); print("wrote", fn)
