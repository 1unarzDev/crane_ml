"""Aggregate joint-cell CSVs: python3 scripts/joint_cells_agg.py c1 c2 c3  (bootstrap RNG seed 12345, B=2000).
Reads $FS_JH2/out/<cell>_*.csv; seed list is whatever is present (the paper uses the 200-204 prefix)."""
import os, sys
HERE = os.path.dirname(os.path.abspath(__file__))
FS = os.path.abspath(os.path.join(HERE, ".."))                      # Tools/ForestSearch
sys.path.insert(0, os.path.dirname(FS))                              # Tools/ (package parent)
RES = os.environ.get("FS_RESULTS", os.path.join(FS, "results", "_regen"))  # regenerated tables; diff against results/*.md
os.makedirs(RES, exist_ok=True)
DATA = os.environ.get("FS_DATA", os.path.join(FS, "..", "..", "PerformanceResults", "forest_search"))
JH2 = os.environ.get("FS_JH2", os.path.join(DATA, "joint_cells"))    # per-seed joint-cell CSVs
from collections import defaultdict
import glob, csv, numpy as np
def load(p):
    d = defaultdict(lambda: defaultdict(dict))  # key (planner,strat) -> seed -> ep -> det
    for fn in sorted(glob.glob(os.path.join(JH2, "out", f"{p}_*.csv"))) + (["/tmp/jh2/sp"] if 0 else []):
      for r in csv.DictReader(open(fn)):
          d[(r["planner"], r["strategy"])][int(r["seed"])][int(r["episode"])] = int(r["detected"])
    seeds = sorted({s for v in d.values() for s in v}); print("seeds", seeds)
    return {k: np.array([[v[s][e] for e in range(300)] for s in seeds], float) for k, v in d.items()}
rng = np.random.default_rng(12345)
def ci(A, B, R=2000):
    D = A - B  # seeds x eps paired
    S, E = D.shape; out = np.empty(R)
    for i in range(R):
        si = rng.integers(0, S, S)
        ei = rng.integers(0, E, (S, E))
        out[i] = np.take_along_axis(D[si], ei, 1).mean()
    return D.mean(), *np.percentile(out, [2.5, 97.5])
f = lambda t: f"{t[0]:+.3f} [{t[1]:+.3f},{t[2]:+.3f}]"
for p in sys.argv[1:]:
    X = load(p); P = lambda pl, s: X[(pl, s)]
    print("##", p)
    for pl in ("greedy", "joint"):
        print(pl, " ".join(f"{s}={P(pl,s).mean():.3f}" for s in ("ORACLE","IMPLICIT","INDEP")),
              "O-I", f(ci(P(pl,"ORACLE"), P(pl,"IMPLICIT"))))
    print("jointO-greedyI", f(ci(P("joint","ORACLE"), P("greedy","IMPLICIT"))))
    print("jointI-greedyI", f(ci(P("joint","IMPLICIT"), P("greedy","IMPLICIT"))))
