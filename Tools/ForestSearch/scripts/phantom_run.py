import os, sys
HERE = os.path.dirname(os.path.abspath(__file__))
FS = os.path.abspath(os.path.join(HERE, ".."))                      # Tools/ForestSearch
sys.path.insert(0, os.path.dirname(FS))                              # Tools/ (package parent)
RES = os.environ.get("FS_RESULTS", os.path.join(FS, "results", "_regen"))  # regenerated tables; diff against results/*.md
os.makedirs(RES, exist_ok=True)
DATA = os.environ.get("FS_DATA", os.path.join(FS, "..", "..", "PerformanceResults", "forest_search"))
JH2 = os.environ.get("FS_JH2", os.path.join(DATA, "joint_cells"))    # per-seed joint-cell CSVs
import numpy as np
from ForestSearch.controls import make_phantom_strategies, run_ph
from ForestSearch.event_curve import events, LINKS
from ForestSearch.instances import grid_instance
from ForestSearch.sim import nominal_plan, make_sectors
S=make_phantom_strategies()
names=["IMPLICIT","HYBRID","HYBRID_P","HYBRID_O","PH_orc","NP_none","NP_nom","NP_orc","NP_pred"]
seeds=[600,601,602,603]; rng=np.random.default_rng(0)
out=["# Phantom-look vs in-flight controls (EXPLORATORY; seeds 600-603, 5x5, travel 0, d1_loss0, 300 eps)","",
"PH_* = HYBRID counting (nominal looks for unconfirmed post-deviation slots); NP_* = no phantom looks. Suffix = in-flight partner model.","",
"| λ | strategy | P | Δ vs IMPLICIT [95% CI] | collisions/ep |","|---|---|---|---|---|"]
for lam in (1.0,3.0):
    det={n:{} for n in names}; col={n:[] for n in names}
    for sd in seeds:
        inst=grid_instance(5,5,seed=sd,travel_scale=0.0,horizon=16,**events(lam))
        nom=nominal_plan(inst,S["ORACLE"]); sec=make_sectors(inst)
        for n in names:
            for ep in range(300):
                r,c=run_ph(inst,(n,n),LINKS["d1_loss0"],ep,S,nom,sec,keep_log=True)
                det[n][(sd,ep)]=r.detected; col[n].append(c)
    for n in names:
        d={sd:np.array([det[n][(sd,e)]-det["IMPLICIT"][(sd,e)] for e in range(300)],float) for sd in seeds}
        m=np.mean(np.concatenate(list(d.values())))
        bs=[np.mean(np.concatenate([rng.choice(d[s],300) for s in rng.choice(seeds,4)])) for _ in range(2000)]
        out.append(f"| {lam:g} | {n} | {np.mean(list(det[n].values())):.3f} | {m:+.3f} [{np.percentile(bs,2.5):+.3f},{np.percentile(bs,97.5):+.3f}] | {np.mean(col[n]):.2f} |")
    print("\n".join(out[-len(names):]),flush=True)
open(RES+'/phantom.md','w').write("\n".join(out)+"\n")
