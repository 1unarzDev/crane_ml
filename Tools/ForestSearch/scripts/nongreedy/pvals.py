"""Two-sided cluster-bootstrap p-values (2000 resamples, RNG 777) for lookahead contrasts, with Holm over the 4 cells."""
import sys, glob, csv
from collections import defaultdict
import numpy as np
OUT = sys.argv[1]
d = defaultdict(lambda: defaultdict(dict))
for fn in sorted(glob.glob(f"{OUT}/out/*.csv")):
    for r in csv.DictReader(open(fn)):
        d[(r["cell"], float(r["lam"]), r["planner"], r["strategy"])][int(r["seed"])][int(r["episode"])] = int(r["detected"])
rng = np.random.default_rng(777)
def p2(D, R=2000):
    S, E = D.shape; out = np.empty(R)
    for i in range(R):
        si = rng.integers(0, S, S); ei = rng.integers(0, E, (S, E)); out[i] = np.take_along_axis(D[si], ei, 1).mean()
    return min(1.0, 2 * min((out <= 0).mean(), (out >= 0).mean()) + 1 / R)
for a, b in ((("lookahead", "ORACLE"), ("greedy", "IMPLICIT")), (("lookahead", "ORACLE"), ("greedy", "ORACLE"))):
    ps = []
    for cell in ("6x6t1", "5x5t1"):
        for lam in (1.0, 3.0):
            seeds = sorted(d[(cell, lam) + a]); E = 100
            A = np.array([[d[(cell, lam) + a][s][e] for e in range(E)] for s in seeds], float)
            B = np.array([[d[(cell, lam) + b][s][e] for e in range(E)] for s in seeds], float)
            ps.append((f"{cell} λ={int(lam)}", p2(A - B)))
    order = sorted(range(4), key=lambda i: ps[i][1]); m = 4; adj = [0] * 4; run = 0
    for r, i in enumerate(order):
        run = max(run, min(1, (m - r) * ps[i][1])); adj[i] = run
    print(f"{a[0]} {a[1]} − {b[0]} {b[1]}: " + "; ".join(f"{n} p={p:.4f} Holm={adj[i]:.4f}" for i, (n, p) in enumerate(ps)))
