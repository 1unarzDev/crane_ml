"""Aggregate joint-vs-greedy table on all 8 held-out seeds (200-207). Paired cluster bootstrap
(resample seeds, then episodes within seed), 2000 resamples, RNG seed 12345 (as joint_cells_agg.py)."""
import sys, glob, gzip, csv, io, os
from collections import defaultdict
import numpy as np
D = "/home/lunarz/worktrees/crane-forest-sar/Tools/ForestSearch/results/data"
NEW = sys.argv[1]   # dir with out/<cell>_<seed>.csv for seeds 205-207
def rd(fn):
    op = gzip.open if fn.endswith(".gz") else open
    with op(fn, "rt") as f: return list(csv.DictReader(f))
def load(cell):
    d = defaultdict(lambda: defaultdict(dict)); src = []
    if cell == "t1_5x5":
        for fn in (f"{D}/joint_heldout_a.csv.gz", f"{D}/joint_heldout_b.csv.gz"):
            src.append(os.path.basename(fn))
            for r in rd(fn):
                if r["link"] == "ideal" and r["strategy"] in ("ORACLE", "IMPLICIT", "INDEP"):
                    d[(r["planner"], r["strategy"])][int(r["seed"])][int(r["episode"])] = int(r["detected"])
    else:
        fns = sorted(glob.glob(f"{D}/joint_cells__out__{cell}_*.csv.gz")) + sorted(glob.glob(f"{NEW}/out/{cell}_*.csv"))
        for fn in fns:
            src.append(os.path.basename(fn))
            for r in rd(fn):
                d[(r["planner"], r["strategy"])][int(r["seed"])][int(r["episode"])] = int(r["detected"])
    seeds = sorted({s for v in d.values() for s in v})
    X = {k: np.array([[v[s][e] for e in range(300)] for s in seeds], float) for k, v in d.items()}
    return X, seeds, src
rng = np.random.default_rng(12345)
def ci(A, B, R=2000):
    Dd = A - B; S, E = Dd.shape; out = np.empty(R)
    for i in range(R):
        si = rng.integers(0, S, S); ei = rng.integers(0, E, (S, E))
        out[i] = np.take_along_axis(Dd[si], ei, 1).mean()
    return Dd.mean(), *np.percentile(out, [2.5, 97.5])
f = lambda t: f"{t[0]:+.3f} [{t[1]:+.3f},{t[2]:+.3f}]"
NAMES = {"t1_5x5": "5x5 travel 1, default horizon", "c1": "5x5 travel 0, horizon 16",
         "c2": "6x6 travel 0, horizon 16", "c3": "6x6 travel 1, default horizon"}
main, cross = [], []
for cell in ("t1_5x5", "c1", "c2", "c3"):
    X, seeds, src = load(cell); P = lambda pl, s: X[(pl, s)]
    assert seeds == list(range(200, 208)), (cell, seeds)
    for pl in ("greedy", "joint"):
        main.append(f"| {NAMES[cell]} | {seeds[0]}-{seeds[-1]} | {pl} | " + " | ".join(f"{P(pl,s).mean():.3f}" for s in ("ORACLE", "IMPLICIT", "INDEP"))
                    + f" | {f(ci(P(pl,'ORACLE'), P(pl,'IMPLICIT')))} |")
    cross.append(f"| {NAMES[cell]} | {f(ci(P('joint','ORACLE'), P('greedy','IMPLICIT')))} | {f(ci(P('joint','IMPLICIT'), P('greedy','IMPLICIT')))} | {f(ci(P('joint','ORACLE'), P('greedy','ORACLE')))} |")
    print(cell, "sources:", ", ".join(src[:3]), "...", len(src), file=sys.stderr)
print("| cell | seeds | planner | P ORACLE | P IMPLICIT | P INDEP | ORACLE-IMPLICIT |\n|---|---|---|---|---|---|---|")
print("\n".join(main))
print("\n| cell | joint ORACLE - greedy IMPLICIT | joint IMPLICIT - greedy IMPLICIT | joint ORACLE - greedy ORACLE |\n|---|---|---|---|")
print("\n".join(cross))
