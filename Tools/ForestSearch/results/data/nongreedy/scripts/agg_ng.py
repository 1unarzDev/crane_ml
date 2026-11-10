"""Aggregate non-greedy held-out runs. Paired cluster bootstrap (seeds, then episodes), 2000 resamples,
RNG 12345; comparisons use the episodes common to both arms (lookahead: first E episodes)."""
import sys, glob, csv
from collections import defaultdict
import numpy as np
OUT = sys.argv[1]
d = defaultdict(lambda: defaultdict(dict)); sec = defaultdict(list)
for fn in sorted(glob.glob(f"{OUT}/out/*.csv")):
    for r in csv.DictReader(open(fn)):
        key = (r["cell"], float(r["lam"]), r["planner"], r["strategy"])
        d[key][int(r["seed"])][int(r["episode"])] = int(r["detected"])
        sec[(r["cell"], r["planner"], r["strategy"])].append(float(r["sec"]))
rng = np.random.default_rng(12345)
def arr(key, seeds, E): return np.array([[d[key][s][e] for e in range(E)] for s in seeds], float)
def ci(D, R=2000):
    S, E = D.shape; out = np.empty(R)
    for i in range(R):
        si = rng.integers(0, S, S); ei = rng.integers(0, E, (S, E))
        out[i] = np.take_along_axis(D[si], ei, 1).mean()
    return D.mean(), *np.percentile(out, [2.5, 97.5])
f = lambda t: f"{t[0]:+.3f} [{t[1]:+.3f},{t[2]:+.3f}]"
gO, gI, gN, lO, lI = ("greedy", "ORACLE"), ("greedy", "IMPLICIT"), ("greedy", "INDEP"), ("lookahead", "ORACLE"), ("lookahead", "IMPLICIT")
rows1, rows2 = [], []
pool = defaultdict(list)
for cell in ("6x6t1", "5x5t1"):
    for lam in (1.0, 3.0):
        k = lambda a: (cell, lam) + a
        if not d[k(lO)]: continue
        seeds = sorted(d[k(lO)]); E = min(len(d[k(lO)][s]) for s in seeds)
        have_I = bool(d[k(lI)]) and sorted(d[k(lI)]) == seeds
        A = {n: arr(k(a), seeds, E) for n, a in (("gO", gO), ("gI", gI), ("gN", gN), ("lO", lO))}
        if have_I: A["lI"] = arr(k(lI), seeds, E)
        G300 = {n: arr(k(a), seeds, 300) for n, a in (("gO", gO), ("gI", gI))}
        tag = f"{cell[:3]} travel 1, λ={int(lam)}"
        ss = f"{seeds[0]}-{seeds[-1]}" if seeds == list(range(seeds[0], seeds[-1] + 1)) else ",".join(map(str, seeds))
        rows1.append(f"| {tag} | {ss} ({len(seeds)}) | {E} | {A['gO'].mean():.3f} | {A['gI'].mean():.3f} | {A['lO'].mean():.3f} | "
                     + (f"{A['lI'].mean():.3f}" if have_I else "–") + f" | {A['gN'].mean():.3f} |")
        rows2.append(f"| {tag} | {f(ci(A['lO']-A['gI']))} | {f(ci(A['lO']-A['gO']))} | {f(ci(G300['gO']-G300['gI']))} | "
                     + (f"{f(ci(A['lI']-A['gI']))} | {f(ci(A['lO']-A['lI']))}" if have_I else "– | –") + " |")
        for n in ("lO-gI", "lO-gO"):
            a, b = n.split("-"); pool[n].append(A[a] - A[b])
        if have_I:
            pool["lI-gI"].append(A["lI"] - A["gI"]); pool["lO-lI"].append(A["lO"] - A["lI"])
print("| cell | seeds | eps/seed (lookahead) | greedy ORACLE | greedy IMPLICIT | lookahead ORACLE | lookahead IMPLICIT | greedy INDEP |")
print("|---|---|---|---|---|---|---|---|"); print("\n".join(rows1))
print("\n| cell | lookahead ORACLE − greedy IMPLICIT | lookahead ORACLE − greedy ORACLE | greedy ORACLE − greedy IMPLICIT (300 eps) | lookahead IMPLICIT − greedy IMPLICIT | lookahead ORACLE − lookahead IMPLICIT |")
print("|---|---|---|---|---|---|"); print("\n".join(rows2))
print("\nPooled over cells (clusters = cell×λ×seed):")
for n, L in pool.items():
    E = min(x.shape[1] for x in L); D = np.concatenate([x[:, :E] for x in L])
    print(f"- {n}: {f(ci(D))}  ({D.shape[0]} clusters × {E} eps)")
print("\nCompute (wall s/episode, niced on a shared, overloaded host):")
for (cell, pl, st), v in sorted(sec.items()):
    print(f"- {cell} {pl} {st}: mean {np.mean(v):.3f}, median {np.median(v):.3f} (n={len(v)})")
