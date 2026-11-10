"""Figure 1 for the paper: Delta P(detect) vs the silent default plan, on held-out 6x6.

Run from Tools/:  python -m ForestSearch.make_fig1 <results_dir> <out.pdf>
Text is rendered by LaTeX (usetex), so fonts are Type 1. Bootstrap B=2000.
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
          ("6x6", "0", "d1_burst30", r"(c) delay 1, 30\% bursty loss"),
          ("6x6", "1", "d1_loss0", "(d) travel, delay 1, no loss")]
# Okabe-Ito palette; distinct markers/linestyles so it survives greyscale.
# strategy: (colour, marker, linestyle, label, band?)
SER = {"INDEP": ("#666666", "v", "--", "INDEP", False),
       "ORACLE": ("#000000", "D", "--", "ORACLE", False),
       "FULL": ("#D55E00", "s", "-", "FULL", False),
       "HYBRID": ("#0072B2", "o", "-", "HYBRID", True),
       "HYBRID_P": ("#009E73", "^", "-", "HYBRID-P", True)}
YLIM = (-0.175, 0.055)
GAP = 0.018  # minimum vertical separation of direct labels (data units, ~7 pt)


def spread(ys, lo, hi, gap=GAP):
    """Nudge label positions apart (keeps order), staying inside [lo, hi]."""
    order = sorted(ys, key=lambda k: ys[k])
    pos = {k: ys[k] for k in order}
    for _ in range(50):
        for a_, b_ in zip(order, order[1:]):
            if pos[b_] - pos[a_] < gap:
                m = (pos[a_] + pos[b_]) / 2
                pos[a_], pos[b_] = m - gap / 2, m + gap / 2
        if pos[order[-1]] > hi:
            d = pos[order[-1]] - hi
            for k in order:
                pos[k] -= d
        if pos[order[0]] < lo:
            d = lo - pos[order[0]]
            for k in order:
                pos[k] += d
    return pos


def main(rdir, out):
    fig = plt.figure(figsize=(7.0, 2.3))
    gs = fig.add_gridspec(1, 4, wspace=0.08, left=0.1, right=0.995, bottom=0.18, top=0.9)
    ax = [fig.add_subplot(gs[0, 0])]
    ax += [fig.add_subplot(gs[0, k], sharey=ax[0]) for k in (1, 2, 3)]
    for a in ax[1:]:
        plt.setp(a.get_yticklabels(), visible=False)
    cache = {}
    for j, (g, t, ln, title) in enumerate(PANELS):
        if (g, t) not in cache:
            cache[(g, t)] = load(f"{rdir}/confirm_{g}_t{t}.csv")[0]
        D = cache[(g, t)]
        rng = np.random.default_rng(0)
        a = ax[j]
        a.axhline(0, color="0.35", lw=.8, zorder=1)
        xs = [0, 1, 2, 3]
        ends = {}
        for s in ("INDEP", "ORACLE", "FULL", "HYBRID", "HYBRID_P"):
            c, m, ls, lab, band = SER[s]
            r = [paired(D, float(lam), ln, s, rng=rng, B=2000) for lam in xs]
            y = [x[0] for x in r]
            if band:
                a.fill_between(xs, [x[1] for x in r], [x[2] for x in r], color=c, alpha=.10, lw=0)
            if s == "INDEP":
                a.plot(xs, y, ls, marker=m, ms=2.8, lw=1.0, color=c, mfc="w", zorder=2)
            else:
                a.plot(xs, y, ls, marker=m, ms=3.0, lw=1.1, color=c, zorder=3)
            ends[s] = y[-1]
        pos = spread(ends, YLIM[0] + 0.006, YLIM[1] - 0.006)
        for s, yy in pos.items():
            c = SER[s][0]
            a.text(3.18, yy, SER[s][3], color=c, fontsize=7.5, va="center", ha="left", clip_on=False)
        if j == 0:
            a.annotate("zero line:\nIMPLICIT (silence)", xy=(0.5, -0.002), xytext=(0.05, -0.052),
                       fontsize=7.5, color="0.25", ha="left", va="center",
                       arrowprops=dict(arrowstyle="-|>", lw=.6, color="0.35", shrinkA=1, shrinkB=1))
        a.set_title(title, fontsize=8, pad=3)
        a.set_xlabel(r"private-event scale $\lambda$", fontsize=8)
        a.set_xticks(xs)
        a.set_xlim(-0.2, 5.35)
        a.set_ylim(*YLIM)
        a.set_yticks([-.15, -.1, -.05, 0, .05])
        a.tick_params(labelsize=7.5, length=2.5)
        a.grid(axis="y", lw=.3, alpha=.5)
        for sp in ("top", "right"):
            a.spines[sp].set_visible(False)
    ax[0].set_ylabel(r"$\Delta P$ vs.\ IMPLICIT" + "\n" + r"(0 = silent default plan)", fontsize=8)
    fig.savefig(out)
    fig.savefig(out.replace(".pdf", ".png"), dpi=200)


if __name__ == "__main__":
    main(sys.argv[1], sys.argv[2])
