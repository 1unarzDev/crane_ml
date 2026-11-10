# SPDX-License-Identifier: GPL-3.0-or-later
"""DEV-ONLY readout of the draft hypotheses H1-H6 (PREREG_ROLES.md) on exploration rows. EXPLORATORY, not a test.

  python dev_hypotheses.py --main results/explore_main_s1.jsonl,results/explore_main_s2.jsonl \
      --side results/explore_side.jsonl --ratio results/explore_ratio.jsonl
"""
import argparse
import json

import numpy as np

C_, E_, D_, A_, B_ = "HOMO_SHARED_REVISIT", "SPEC_ADAPTIVE", "SPEC_FIXED", "HOMO_FAST", "HOMO_THOROUGH"


def load(paths):
    rows = []
    for p in paths.split(","):
        if p:
            with open(p) as f:
                rows += [json.loads(l) for l in f]
    return rows


def fam(r):
    return r.get("family", r["policy"].split("(")[0])


def paired(rows, cond, a, b, key, by_label=False):
    """Per-layout mean of a - b over rows satisfying cond; returns per-layout diffs."""
    d = {}
    for r in rows:
        if not cond(r):
            continue
        k = r["policy"] if by_label else fam(r)
        d.setdefault((r["cell"], r["seed"], r.get("rep", 0)), {})[k] = r[key]
    per = {}
    for (cell, s, rp), v in d.items():
        if a in v and b in v:
            per.setdefault(s, []).append(v[a] - v[b])
    return np.array([np.mean(per[s]) for s in sorted(per)])


def ci(x, n_boot=2000, seed=0):
    if len(x) == 0:
        return float("nan"), float("nan"), float("nan"), 0
    r = np.random.default_rng(seed)
    bs = np.array([x[r.integers(0, len(x), len(x))].mean() for _ in range(n_boot)])
    return float(x.mean()), float(np.percentile(bs, 2.5)), float(np.percentile(bs, 97.5)), len(x)


def fmt(t):
    return "%+.3f [%+.3f, %+.3f] (n=%d layouts)" % t


def N_of(r):
    return int(r["cell"].split("_")[0][1:])


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--main", default="")
    ap.add_argument("--side", default="")
    ap.add_argument("--ratio", default="")
    ap.add_argument("--key", default="auc_rep")
    a = ap.parse_args()
    k = a.key
    M = load(a.main)
    print("# DEV-ONLY, EXPLORATORY readout (seeds 100-199; not evidence). key = %s\n" % k)
    if M:
        print("## H1/H2: (e) - (c) and (d) - (c) by N (pooled over occlusion, trade-off, comms)")
        gains = {}
        for n in sorted({N_of(r) for r in M}):
            for x in (E_, "B5_THRESHOLD", D_, "B3_JIN_CONFIRM", "B6_AUCTION", "B7_SOLO", A_, B_):
                t = ci(paired(M, lambda r: N_of(r) == n, x, C_, k))
                gains[(x, n)] = t
                print("- N=%d %s - (c): %s" % (n, x, fmt(t)))
        print("\n## Regime map: (e) - (c) per main cell")
        for cell in sorted({r["cell"] for r in M}, key=lambda c: (int(c.split("_")[0][1:]), c)):
            te = ci(paired(M, lambda r: r["cell"] == cell, E_, C_, k))
            td = ci(paired(M, lambda r: r["cell"] == cell, D_, C_, k))
            print("- %s: (e)-(c) %s; (d)-(c) %s" % (cell, fmt(te), fmt(td)))
        print("\n## H5: outage drop, (c) vs X: DiD = [(c)_ideal-(c)_outage] - [X_ideal-X_outage]")
        for X in (E_, "B5_THRESHOLD", D_, "B6_AUCTION"):
          for n in sorted({N_of(r) for r in M}):
              di = {}
              for r in M:
                  if N_of(r) != n or fam(r) not in (C_, X):
                      continue
                  base = r["cell"].replace("_outage", "").replace("_ideal", "")
                  cm = "outage" if "_outage" in r["cell"] else "ideal"
                  di.setdefault((base, r["seed"]), {})[(fam(r), cm)] = r[k]
              per = {}
              for (base, s), v in di.items():
                  if len(v) == 4:
                      per.setdefault(s, []).append((v[(C_, "ideal")] - v[(C_, "outage")])
                                                   - (v[(X, "ideal")] - v[(X, "outage")]))
              x = np.array([np.mean(per[s]) for s in sorted(per)])
              print("- %s N=%d DiD %s" % (X, n, fmt(ci(x))))
    S = load(a.side)
    if S:
        print("\n## Side cells (high occlusion, strong trade-off, outage unless noted)")
        for cell in sorted({r["cell"] for r in S}, key=lambda c: (int(c.split("_")[0][1:]), c)):
            for x in (E_, "B5_THRESHOLD", D_, A_, B_, "B3_JIN_CONFIRM", "B6_AUCTION", "B7_SOLO"):
                print("- %s %s - (c): %s" % (cell, x, fmt(ci(paired(S, lambda r: r["cell"] == cell, x, C_, k)))))
            if "deadline" in cell:
                print("- %s H6 recall_rep (b) - (e): %s" % (cell, fmt(ci(paired(S, lambda r: r["cell"] == cell, B_, E_,
                                                                                    "recall_rep")))))
            if "_none_" in cell:
                for x in (D_, E_, B_):
                    print("- %s H4 %s - (a): %s" % (cell, x, fmt(ci(paired(S, lambda r: r["cell"] == cell, x, A_, k)))))
    R = load(a.ratio)
    if R:
        print("\n## H3: SPEC_FIXED(r) - (c) by recon fraction r (verifier fraction 1 - r)")
        for cell in sorted({r["cell"] for r in R}, key=lambda c: (int(c.split("_")[0][1:]), c)):
            labs = sorted({r["policy"] for r in R if r["policy"].startswith("SPEC_FIXED")})
            best = None
            parts = []
            for lab in labs:
                t = ci(paired(R, lambda r: r["cell"] == cell, lab, C_, k, by_label=True))
                parts.append("%s %+.3f [%+.3f,%+.3f]" % (lab[11:-1], t[0], t[1], t[2]))
                if best is None or t[0] > best[0]:
                    best = (t[0], lab)
            print("- %s: best %s | %s" % (cell, best[1], "; ".join(parts)))


if __name__ == "__main__":
    main()
