"""Extra read-only analysis of the finished pilot store for RESULTS.md (EXPLORATORY, SURROGATE).

Nothing here runs a simulation or fits a model; it re-reads learner_truth_v1.jsonl and
learner_pilot_v1.jsonl and prints markdown:

  ties       how many arms are statistically tied with the best one at each budget
  effects    2 x 2 main effects (inputs, sampling rule), paired by repeat
  constants  learners against the best no-data recommendation
  transfer   the decided narrow_tandem courses, and paired contrasts at each budget
  where      where the boundary-seeking arms spent their runs
  nav2       pool labels of both stand-in pairs on the w = 0.90 m slice (the Nav2 design's door width)

CLI:  extras.py            (prints everything)
"""
from __future__ import annotations

import os
import sys

import numpy as np

_HERE = os.path.dirname(os.path.abspath(__file__))
for _p in (os.path.dirname(_HERE), _HERE):
    if _p not in sys.path:
        sys.path.insert(0, _p)

import mechanics as M         # noqa: E402
import acquire as A           # noqa: E402
import features as F          # noqa: E402
import pilot as P             # noqa: E402
import truth as T             # noqa: E402

NS = P.CHECKPOINTS
SYM = {"A_wins": "A", "B_wins": "B", "indifferent": "=", "both_fail": "x", "unresolved": "?"}


def _ci(v):
    m, lo, hi = A.bootstrap_ci(v)
    return "%.2f [%.2f, %.2f]" % (m, lo, hi)


def _vals(res, arm, n, metric="R"):
    return np.array(res[arm][n][metric], float)


def ties(res, out):
    out.append("\narms tied with the best single-footprint arm (paired 95% interval of the difference includes 0), and the spread of arm means\n")
    out.append("| n | best arm | R best | R worst | arms tied with best (of %d) | sobol-raw tied? |" % len(P.ARMS))
    out.append("|---|---|---|---|---|---|")
    for n in NS:
        means = {a: float(_vals(res, a, n).mean()) for a in P.ARMS}
        best = min(means, key=means.get)
        tied = []
        for a in P.ARMS:
            if a == best:
                tied.append(a)
                continue
            _, lo, hi = A.bootstrap_ci(_vals(res, a, n) - _vals(res, best, n))
            if lo <= 0.0 <= hi:
                tied.append(a)
        out.append("| %d | %s | %.2f | %.2f | %d | %s |" % (n, best, means[best], max(means.values()), len(tied),
                                                         "yes" if "sobol-raw" in tied else "no"))


def effects(res, out):
    out.append("\n2 x 2 main effects on R (s), paired by repeat, mean [95% bootstrap interval]; negative = helps\n")
    out.append("| effect | " + " | ".join("n=%d" % n for n in NS) + " |")
    out.append("|---|" + "---|" * len(NS))
    rows = [("mechanics - raw, averaged over sobol / straddle / sur",
             lambda n: np.mean([_vals(res, d + "-mech", n) - _vals(res, d + "-raw", n) for d in ("sobol", "straddle", "sur")], axis=0)),
            ("straddle - sobol, averaged over raw / mech",
             lambda n: np.mean([_vals(res, "straddle-" + k, n) - _vals(res, "sobol-" + k, n) for k in ("raw", "mech")], axis=0)),
            ("sur - sobol, averaged over raw / mech",
             lambda n: np.mean([_vals(res, "sur-" + k, n) - _vals(res, "sobol-" + k, n) for k in ("raw", "mech")], axis=0)),
            ("random - sobol, averaged over raw / mech",
             lambda n: np.mean([_vals(res, "random-" + k, n) - _vals(res, "sobol-" + k, n) for k in ("raw", "mech")], axis=0))]
    for name, f in rows:
        out.append("| %s | " % name + " | ".join(_ci(f(n)) for n in NS) + " |")


def constants(D, res, out):
    recs = D.rule_recs("heldout")
    Rr = {k: A.wrong_cost(D.delta["heldout"], D.labels["heldout"], v) for k, v in recs.items()}
    best = min(Rr, key=Rr.get)
    out.append("\nlearners against the best no-data recommendation on the held-out set (%s, R = %.2f s; rule_2rho R = %.2f s): share of the 20 repeats with lower R\n"
               % (best, Rr[best], Rr["rule_2rho"]))
    out.append("| arm | " + " | ".join("n=%d" % n for n in NS) + " |")
    out.append("|---|" + "---|" * len(NS))
    for a in P.ARMS:
        out.append("| %s | " % a + " | ".join("%.2f" % float((_vals(res, a, n) < Rr[best] - 1e-9).mean()) for n in NS) + " |")
    out.append("\nsame against rule_2rho\n")
    out.append("| arm | " + " | ".join("n=%d" % n for n in NS) + " |")
    out.append("|---|" + "---|" * len(NS))
    for a in ("sobol-raw", "sobol-mech", "sur-raw", "sur-mech"):
        out.append("| %s | " % a + " | ".join("%.2f" % float((_vals(res, a, n) < Rr["rule_2rho"] - 1e-9).mean()) for n in NS) + " |")


def transfer(D, res, out):
    tand = D.robots["transfer"]
    two_rho = 2 * M.rho(tand.L, tand.W)
    lab = D.labels["transfer"]
    dec = np.isin(lab, ("A_wins", "B_wins"))
    out.append("\ndecided narrow_tandem courses: %d of %d (2 rho = %.3f m)\n" % (int(dec.sum()), len(lab), two_rho))
    if dec.any():
        out.append("| w | e | d | d / 2 rho | label | mean gap (s) | rule_2rho says | share of R if wrong |")
        out.append("|---|---|---|---|---|---|---|---|")
        tot = float(np.abs(D.delta["transfer"][dec]).sum())
        for a, l, dl, c in zip(D.aggs["transfer"], lab, D.delta["transfer"], D.courses["transfer"]):
            if l in ("A_wins", "B_wins"):
                out.append("| %.2f | %.2f | %.2f | %.3f | %s | %+.1f | %s | %.0f %% |" % (
                    a["w_cm"] / 100, a["e_cm"] / 100, a["d_cm"] / 100, a["d_cm"] / 100 / two_rho, SYM[l], dl,
                    F.rule_2rho(c, tand), 100 * abs(dl) / tot))
        out.append("\npaired contrasts on the held-out footprint, R (s), first minus second; negative = first is better\n")
        out.append("| contrast | " + " | ".join("n=%d" % n for n in NS) + " |")
        out.append("|---|" + "---|" * len(NS))
        cons = [("1 fp: mechanics - raw+dims", ("sobol-mech", "Rt_mech"), ("sobol-raw", "Rt_rawdim")),
                ("1 fp: mechanics - slack", ("sobol-mech", "Rt_mech"), ("sobol-raw", "Rt_slack")),
                ("1 fp: slack - raw+dims", ("sobol-raw", "Rt_slack"), ("sobol-raw", "Rt_rawdim")),
                ("3 fp: mechanics - raw+dims", ("mf3-sobol-mech", "Rt"), ("mf3-sobol-rawdim", "Rt")),
                ("3 fp: mechanics - slack", ("mf3-sobol-mech", "Rt"), ("mf3-sobol-slack", "Rt")),
                ("3 fp: slack - raw+dims", ("mf3-sobol-slack", "Rt"), ("mf3-sobol-rawdim", "Rt")),
                ("3 fp, in distribution: mechanics - raw+dims", ("mf3-sobol-mech", "R"), ("mf3-sobol-rawdim", "R"))]
        for name, (a1, m1), (a2, m2) in cons:
            out.append("| %s | " % name + " | ".join(_ci(_vals(res, a1, n, m1) - _vals(res, a2, n, m2)) for n in NS) + " |")
        recs = D.rule_recs("transfer")
        out.append("\nno-data recommendations on the held-out footprint: " + ", ".join(
            "%s %.2f s" % (k, A.wrong_cost(D.delta["transfer"], lab, v)) for k, v in recs.items()))


def where(D, runs, out, n_first=50):
    lab = D.labels["pool"]
    out.append("\nwhere the first %d paired runs went, by pool label (share of runs; pool share in the first row)\n" % n_first)
    keys = ("A_wins", "B_wins", "indifferent", "both_fail", "unresolved")
    out.append("| arm | " + " | ".join(keys) + " |")
    out.append("|---|" + "---|" * len(keys))
    out.append("| (pool) | " + " | ".join("%.2f" % float((lab == k).mean()) for k in keys) + " |")
    for arm in ("sobol-raw", "straddle-raw", "straddle-mech", "sur-raw", "sur-mech"):
        cnt = np.zeros(len(keys))
        for x in runs:
            if x["pair"] != D.pair or x["arm"] != arm:
                continue
            ch = x["chosen"][:n_first] if x["chosen"] else D.sobol_order(x["rep"])[:n_first]
            for c in ch:
                cnt[keys.index(lab[c])] += 1
        out.append("| %s | " % arm + " | ".join("%.2f" % v for v in cnt / max(cnt.sum(), 1)) + " |")


def chosen_counts(D, runs, arms=("straddle-raw", "straddle-mech", "sur-raw", "sur-mech"), n_first=100):
    cnt = np.zeros(D.P)
    for x in runs:
        if x["pair"] == D.pair and x["arm"] in arms and x["chosen"]:
            for c in x["chosen"][10:n_first]:          # skip the shared Sobol start
                cnt[c] += 1
    return cnt


def nav2_slice(Ds, runs, out, w_cm=90):
    comp = Ds["constructed"].robots["pool"]
    out.append("\npool labels on the w = %.2f m slice, constructed / independent pair (A, B, = indifferent, x both fail, ? unresolved); 2 rho = %.3f m\n"
               % (w_cm / 100, 2 * M.rho(comp.L, comp.W)))
    cells = {}
    for pair, D in Ds.items():
        cnt = chosen_counts(D, runs)
        for i, (a, l) in enumerate(zip(D.aggs["pool"], D.labels["pool"])):
            if a["w_cm"] == w_cm:
                cells[(pair, a["e_cm"], a["d_cm"])] = (SYM[l], float(np.mean(a["gap"])), cnt[i])
    ds = [d for d in T.POOL_D]
    out.append("| e \\ d | " + " | ".join("%.2f" % (d / 100) for d in ds) + " |")
    out.append("|---|" + "---|" * len(ds))
    for e in reversed(T.POOL_E):
        row = []
        for d in ds:
            c, i = cells.get(("constructed", e, d)), cells.get(("independent", e, d))
            row.append("%s / %s" % (c[0] if c else " ", i[0] if i else " "))
        out.append("| %.2f | " % (e / 100) + " | ".join(row) + " |")
    out.append("\nmean gap J_A - J_B (s) on the same slice, constructed / independent\n")
    out.append("| e \\ d | " + " | ".join("%.2f" % (d / 100) for d in ds) + " |")
    out.append("|---|" + "---|" * len(ds))
    for e in reversed(T.POOL_E):
        row = []
        for d in ds:
            c, i = cells.get(("constructed", e, d)), cells.get(("independent", e, d))
            row.append("%s / %s" % ("%+.0f" % c[1] if c else " ", "%+.0f" % i[1] if i else " "))
        out.append("| %.2f | " % (e / 100) + " | ".join(row) + " |")
    out.append("\ntimes the four boundary-seeking arms picked each course in runs 11-100 (80 arm-repeats per pair), constructed / independent\n")
    out.append("| e \\ d | " + " | ".join("%.2f" % (d / 100) for d in ds) + " |")
    out.append("|---|" + "---|" * len(ds))
    for e in reversed(T.POOL_E):
        row = []
        for d in ds:
            c, i = cells.get(("constructed", e, d)), cells.get(("independent", e, d))
            row.append("%s / %s" % ("%d" % c[2] if c else " ", "%d" % i[2] if i else " "))
        out.append("| %.2f | " % (e / 100) + " | ".join(row) + " |")


def main():
    rows = T.load_rows()
    runs = P.load_runs()
    out, Ds = [], {}
    for pair in T.PAIRS:
        D = P.Data(pair, rows)
        Ds[pair] = D
        res = P.collect(runs, pair, 1, "slog")
        out.append("\n### Pair: %s\n" % pair)
        ties(res, out)
        effects(res, out)
        constants(D, res, out)
        transfer(D, res, out)
        where(D, runs, out)
    nav2_slice(Ds, runs, out)
    print("\n".join(out))
    return 0


if __name__ == "__main__":
    sys.exit(main())
