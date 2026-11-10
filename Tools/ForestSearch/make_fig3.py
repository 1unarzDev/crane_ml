"""Figure 3 (schematic): the stale in-flight partner model behind HYBRID's collisions.

Run from Tools/:  python -m ForestSearch.make_fig3 [out.pdf]
"""
import os
import sys

import matplotlib

matplotlib.use("Agg")
matplotlib.rcParams.update({"pdf.fonttype": 42, "ps.fonttype": 42, "text.usetex": True,
                            "font.family": "serif", "font.size": 7})
import matplotlib.pyplot as plt  # noqa: E402

BLUE, ORG, RED, GRN = "#0072B2", "#E69F00", "#D55E00", "#009E73"


def lane(ax, y0, title, fixed):
    ax.text(-2.1, y0 + 0.9, title, fontsize=7.5, va="center", ha="left")
    for k, nm in enumerate(("agent 0", "agent 1")):
        y = y0 - k * 0.55
        ax.plot([0, 9.6], [y, y], color="0.75", lw=3.5, solid_capstyle="butt", zorder=1)
        ax.text(-0.2, y, nm, ha="right", va="center", fontsize=6.8)
    if True:
        ax.text(9.7, y0 + 0.2, r"time $\rightarrow$", ha="right", va="bottom", fontsize=6.8, color="0.4")
    # clue found by agent 0 at t=2.2
    ax.plot(2.2, y0, "o", color=BLUE, ms=5, zorder=3)
    ax.text(2.2, y0 + 0.2, "clue found", ha="center", va="bottom", fontsize=6.8, color=BLUE,
            linespacing=.9)
    # record in flight (delay)
    ax.annotate("", (5.0, y0 - 0.55), (2.2, y0),
                arrowprops=dict(arrowstyle="-|>", lw=1, color=BLUE, ls="--"), zorder=2)
    ax.plot(5.0, y0 - 0.55, "o", color=BLUE, ms=5, zorder=3)
    ax.text(3.4, y0 - 0.55 - 0.17, "record (delayed)", fontsize=6.8, color=BLUE, ha="center", va="top")


def main(out):
    fig, ax = plt.subplots(figsize=(3.3, 2.5))
    ax.set_xlim(-2.2, 9.8)
    ax.set_ylim(-5.0, 1.3)
    ax.axis("off")
    # --- HYBRID
    y0 = 0.0
    lane(ax, y0, r"HYBRID: stale in-flight model", True)
    ax.text(5.1, y0 - 1.05 - 0.25, "", fontsize=6.8)
    # both pick same target
    for k, col in ((0, ORG), (1, ORG)):
        y = y0 - k * 0.55
        ax.annotate("", (8.4, y0 - 0.275), (5.6 + (0 if k else 0.0), y),
                    arrowprops=dict(arrowstyle="-|>", lw=1.1, color=RED, shrinkB=2), zorder=2)
    ax.plot(8.6, y0 - 0.275, "*", color=RED, ms=10, zorder=4)
    ax.text(9.0, y0 - 0.275, "", fontsize=6.8)
    ax.text(8.2, y0 + 0.42, "collision", fontsize=6.8, color=RED, ha="center", va="bottom")
    ax.text(3.8, y0 - 1.1, r"both assume stale partner plan $\rightarrow$ same argmax $\rightarrow$ collision",
            fontsize=6.8, ha="center", va="top", linespacing=1.15, color="0.15")
    # --- HYBRID-P
    y1 = -3.3
    lane(ax, y1, r"HYBRID-P: priority-order prediction", False)
    ax.annotate("", (8.4, y1), (5.6, y1), arrowprops=dict(arrowstyle="-|>", lw=1.1, color=GRN, shrinkB=2))
    ax.annotate("", (8.4, y1 - 0.55 - 0.0), (5.6, y1 - 0.55),
                arrowprops=dict(arrowstyle="-|>", lw=1.1, color=GRN, shrinkB=2))
    ax.text(8.9, y1, "X", color=GRN, fontsize=7, va="center")
    ax.text(8.9, y1 - 0.55, "Y", color=GRN, fontsize=7, va="center")
    ax.text(3.8, y1 - 1.1, r"agent 1 predicts 0 takes best target X $\rightarrow$ takes Y",
            fontsize=6.8, ha="center", va="top", linespacing=1.15, color="0.15")
    fig.subplots_adjust(left=0.01, right=0.995, top=0.995, bottom=0.005)
    fig.savefig(out)
    fig.savefig(out.replace(".pdf", ".png"), dpi=200)


if __name__ == "__main__":
    main(sys.argv[1] if len(sys.argv) > 1 else os.path.join(os.path.dirname(os.path.abspath(__file__)),
                                                            "paper", "figs", "fig3_mechanism.pdf"))
