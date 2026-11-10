"""Figure 1 for the paper: Delta P(detect) vs the silent default plan, on held-out 6x6.

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
matplotlib.rcParams["font.size"] = 8
import matplotlib.pyplot as plt  # noqa: E402

from .analyze_curve import load, paired  # noqa: E402

PANELS = [("6x6", "0", "d0_loss0", "(a) no delay, no loss"),
          ("6x6", "0", "d1_loss0", "(b) delay 1, no loss"),
          ("6x6", "0", "d1_burst30", r"(c) delay 1, 30\% burst loss"),
          ("6x6", "1", "d1_loss0", "(d) travel, delay 1, no loss")]
# Okabe-Ito palette; distinct markers/linestyles so it survives greyscale.
# strategy: (colour, marker, linestyle, label, label dy)
SER = {"FULL": ("#D55E00", "s", "-", "FULL", -0.012),
       "HYBRID": ("#0072B2", "o", "-", "HYBRID", 0.0),
       "HYBRID_P": ("#009E73", "^", "-", "HYBRID-P", 0.012),
       "ORACLE": ("#000000", "D", "--", "ORACLE", 0.0),
       "INDEP": ("#999999", "v", "--", "INDEP (no plan)", 0.0)}
YLIM = (-0.175, 0.055)


def main(rdir, out):
    fig = plt.figure(figsize=(7.0, 2.2))
    gs = fig.add_gridspec(1, 5, width_ratios=[1, 1, 1, 0.42, 1], wspace=0.1,
                          left=0.075, right=0.972, bottom=0.2, top=0.9)
    ax = [fig.add_subplot(gs[0, 0])]
    ax += [fig.add_subplot(gs[0, k], sharey=ax[0]) for k in (1, 2)]
    ax.append(fig.add_subplot(gs[0, 4], sharey=ax[0]))
    for a in ax[1:]:
        plt.setp(a.get_yticklabels(), visible=False)
    cache = {}
    ends = {}
    for j, (g, t, ln, title) in enumerate(PANELS):
        if (g, t) not in cache:
            cache[(g, t)] = load(f"{rdir}/confirm_{g}_t{t}.csv")[0]
        D = cache[(g, t)]
        rng = np.random.default_rng(0)
        a = ax[j]
        a.axhline(0, color="0.35", lw=.8, zorder=1)
        xs = [0, 1, 2, 3]
        for s in ("INDEP", "ORACLE", "FULL", "HYBRID", "HYBRID_P"):
            c, m, ls, lab, dy = SER[s]
            r = [paired(D, float(lam), ln, s, rng=rng, B=1000) for lam in xs]
            y = [x[0] for x in r]
            if s not in ("ORACLE", "INDEP"):
                a.fill_between(xs, [x[1] for x in r], [x[2] for x in r], color=c, alpha=.14, lw=0)
            if s == "INDEP":  # light grey dashed, behind the others (no band)
                a.plot(xs, y, ls, marker=m, ms=2.6, lw=.9, color=c, mfc="w", zorder=2)
            else:
                a.plot(xs, y, ls, marker=m, ms=3.2, lw=1.2, color=c, zorder=3)
            if j == 3:
                ends[s] = y[-1]
        a.set_title(title, fontsize=7.5, pad=3)
        a.set_xlabel(r"private-event scale $\lambda$", fontsize=7.5)
        a.set_xticks(xs)
        a.set_ylim(*YLIM)
        a.set_yticks([-.15, -.1, -.05, 0, .05])
        a.tick_params(labelsize=7, length=2.5)
        a.grid(axis="y", lw=.3, alpha=.5)
        for sp in ("top", "right"):
            a.spines[sp].set_visible(False)
    ax[0].set_ylabel(r"$\Delta P$ vs.\ IMPLICIT", fontsize=7.5)
    ax[0].annotate("0 = default plan (silent)", (0.0, 0.0), (-0.1, 0.04), fontsize=7, ha="left", va="center",
                   color="0.2", arrowprops=dict(arrowstyle="-", lw=.5, color="0.4"))
    # direct labels, panel (b) right edge (clearest separation), nudged to avoid overlap
    a = ax[2]
    rng = np.random.default_rng(0)
    D = cache[("6x6", "0")]
    ys = {s: paired(D, 3.0, "d1_burst30", s, rng=rng, B=200)[0] for s in SER}
    place = {"ORACLE": ys["ORACLE"] + 0.012, "HYBRID_P": ys["HYBRID_P"] - 0.004,
             "HYBRID": ys["HYBRID"] - 0.002, "FULL": ys["FULL"] - 0.004,
             "INDEP": ys["INDEP"]}
    for s, yy in place.items():
        c, m, ls, lab, dy = SER[s]
        a.text(3.12, yy, lab.split(" (")[0], color=c, fontsize=7, va="center", ha="left",
               clip_on=False, bbox=None)
    fig.savefig(out)
    fig.savefig(out.replace(".pdf", ".png"), dpi=200)


if __name__ == "__main__":
    main(sys.argv[1], sys.argv[2])
