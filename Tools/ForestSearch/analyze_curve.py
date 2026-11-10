"""Paired analysis of event_curve CSVs: Delta P(detect) vs IMPLICIT with
cluster bootstrap over instances x episodes (instances resampled, then
episodes within instance)."""
from __future__ import annotations

import argparse
import collections
import csv

import numpy as np


def load(path):
    D = collections.defaultdict(dict)   # (lam, link, strat) -> {(seed, ep): det}
    B = collections.defaultdict(list)
    for r in csv.DictReader(open(path)):
        k = (float(r["lam"]), r["link"], r["strategy"])
        D[k][(int(r["seed"]), int(r["episode"]))] = int(r["detected"])
        B[k].append(float(r["bytes"]))
    return D, B


def paired(D, lam, link, s, ref="IMPLICIT", B=2000, rng=None):
    rng = rng or np.random.default_rng(0)
    a, b = D[(lam, link, s)], D[(lam, link, ref)]
    keys = sorted(set(a) & set(b))
    seeds = sorted({k[0] for k in keys})
    diff = {sd: np.array([a[k] - b[k] for k in keys if k[0] == sd], float) for sd in seeds}
    m = np.mean(np.concatenate(list(diff.values())))
    boots = []
    for _ in range(B):
        ss = rng.choice(seeds, len(seeds))
        boots.append(np.mean(np.concatenate([rng.choice(diff[sd], len(diff[sd])) for sd in ss])))
    return m, np.percentile(boots, 2.5), np.percentile(boots, 97.5)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("csv", nargs="+")
    ap.add_argument("--fig", default=None)
    ap.add_argument("--md", default=None)
    a = ap.parse_args()
    lines = []
    panels = []
    for path in a.csv:
        D, Bt = load(path)
        lams = sorted({k[0] for k in D})
        links = sorted({k[1] for k in D})
        strats = [s for s in ["ORACLE", "SECTOR", "FULL", "EVENT", "HYBRID", "HYBRID_P"]
                  if any(k[2] == s for k in D)]
        lines += [f"## {path}", "", "Delta P(detect) vs IMPLICIT, mean [95% cluster-bootstrap CI]; "
                  "abs = IMPLICIT P; bytes = mean per episode", ""]
        for ln in links:
            lines += [f"### link {ln}", "", "| lam | IMPLICIT | " + " | ".join(strats) + " | bytes FULL/HYBRID |",
                      "|" + "---|" * (len(strats) + 3)]
            for lam in lams:
                imp = np.mean(list(D[(lam, ln, "IMPLICIT")].values()))
                cells = []
                for s in strats:
                    m, lo, hi = paired(D, lam, ln, s)
                    cells.append(f"{m:+.3f} [{lo:+.3f},{hi:+.3f}]")
                    panels.append((path, ln, s, lam, m, lo, hi))
                by = "/".join(f"{np.mean(Bt[(lam, ln, s)]):.0f}" for s in ("FULL", "HYBRID") if (lam, ln, s) in Bt)
                lines.append(f"| {lam:g} | {imp:.3f} | " + " | ".join(cells) + f" | {by} |")
            lines.append("")
    txt = "\n".join(lines)
    print(txt)
    if a.md:
        open(a.md, "w").write(txt + "\n")
    if a.fig:
        import matplotlib
        matplotlib.use("Agg")
        import matplotlib.pyplot as plt
        files = sorted({p[0] for p in panels})
        links = sorted({p[1] for p in panels})
        fig, axes = plt.subplots(len(files), len(links), figsize=(3.2 * len(links), 2.8 * len(files)),
                                 sharey=True, squeeze=False)
        colors = dict(ORACLE="#222222", SECTOR="#999999", FULL="#d62728", EVENT="#ff7f0e",
                      HYBRID="#1f77b4", HYBRID_P="#17becf")
        for i, f in enumerate(files):
            for j, ln in enumerate(links):
                ax = axes[i, j]
                ax.axhline(0, color="k", lw=0.6)
                for s, c in colors.items():
                    pts = sorted([p for p in panels if p[0] == f and p[1] == ln and p[2] == s], key=lambda p: p[3])
                    if not pts:
                        continue
                    x = [p[3] for p in pts]
                    ax.plot(x, [p[4] for p in pts], "-o", ms=3, color=c, label=s)
                    ax.fill_between(x, [p[5] for p in pts], [p[6] for p in pts], color=c, alpha=0.15)
                ax.set_title(f"{f.split('/')[-1].replace('.csv','')} | {ln}", fontsize=8)
                ax.set_xlabel("private-event scale λ")
                if j == 0:
                    ax.set_ylabel("ΔP(detect) vs IMPLICIT")
        axes[0, -1].legend(fontsize=6)
        fig.tight_layout()
        fig.savefig(a.fig, dpi=160)
        print("wrote", a.fig)


if __name__ == "__main__":
    main()
