"""Figure 1 for the paper: Delta P(detect) vs the silent common plan, on held-out 6x6.

Run from Tools/:  python -m ForestSearch.make_fig1 <results_dir> <out.pdf>
Fonts are embedded as TrueType (pdf.fonttype 42), because AAAI forbids Type 3.
"""
import sys

import matplotlib
import numpy as np

matplotlib.use("Agg")
matplotlib.rcParams["pdf.fonttype"] = 42
matplotlib.rcParams["ps.fonttype"] = 42
# Type 1 fonts via LaTeX text rendering (AAAI font check); needs pdflatex on PATH.
matplotlib.rcParams["text.usetex"] = True
matplotlib.rcParams["font.family"] = "serif"
matplotlib.rcParams["font.size"] = 9
import matplotlib.pyplot as plt  # noqa: E402

from .analyze_curve import load, paired  # noqa: E402

PANELS = [("6x6", "0", "d0_loss0", "no delay, no loss"),
          ("6x6", "0", "d1_loss0", "delay 1 step, no loss"),
          ("6x6", "0", "d1_burst30", r"delay 1 step, 30\% bursty loss"),
          ("6x6", "1", "d1_loss0", "with travel, delay 1 step")]
COL = dict(ORACLE=("#222222", "centralized"), FULL=("#d62728", "FULL (replaces plan)"),
           HYBRID=("#1f77b4", "HYBRID (plan + messages)"),
           HYBRID_P=("#17becf", "HYBRID + role tie-break"),
           INDEP=("#999999", "no plan, no messages"))


def main(rdir, out):
    fig, ax = plt.subplots(1, 4, figsize=(7.0, 2.3), sharey=True)
    cache = {}
    for j, (g, t, ln, title) in enumerate(PANELS):
        if (g, t) not in cache:
            cache[(g, t)] = load(f"{rdir}/confirm_{g}_t{t}.csv")[0]
        D = cache[(g, t)]
        rng = np.random.default_rng(0)
        a = ax[j]
        a.axhline(0, color="k", lw=.8)
        for s, (c, lab) in COL.items():
            xs = [0, 1, 2, 3]
            r = [paired(D, float(lam), ln, s, rng=rng, B=1000) for lam in xs]
            a.plot(xs, [x[0] for x in r], "-o", ms=3, color=c, label=lab)
            a.fill_between(xs, [x[1] for x in r], [x[2] for x in r], color=c, alpha=.15)
        a.set_title(title, fontsize=8)
        a.set_xlabel(r"private-event scale $\lambda$", fontsize=8)
        a.set_xticks([0, 1, 2, 3])
    ax[0].set_ylabel(r"$\Delta P$ vs.\ IMPLICIT", fontsize=8)
    h, lab = ax[0].get_legend_handles_labels()
    fig.legend(h, lab, loc="upper center", ncol=5, fontsize=7, frameon=False,
               bbox_to_anchor=(0.5, 1.0), handlelength=1.6, columnspacing=1.0)
    for a in ax:
        a.tick_params(labelsize=7)
    fig.tight_layout(rect=(0, 0, 1, 0.88))
    fig.savefig(out)
    fig.savefig(out.replace(".pdf", ".png"), dpi=200)


if __name__ == "__main__":
    main(sys.argv[1], sys.argv[2])
