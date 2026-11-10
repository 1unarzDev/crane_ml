"""Figure 3 (schematic): the stale in-flight partner model behind HYBRID's collisions.

Column width (3.3 in), all text >= 7 pt, numbered steps, targets X and Y drawn as shapes.
Colours follow Table 1: HYBRID #0072B2, HYBRID-P #009E73. Text via LaTeX (usetex) -> Type 1 fonts.
Run from Tools/:  python -m ForestSearch.make_fig3 [out.pdf]
"""
import os
import sys

import matplotlib

matplotlib.use("Agg")
matplotlib.rcParams.update({"pdf.fonttype": 42, "ps.fonttype": 42, "text.usetex": True,
                            "font.family": "serif", "font.size": 7})
import matplotlib.pyplot as plt  # noqa: E402

HYB, HYBP = "#0072B2", "#009E73"
NEU = "0.30"
FS = 7
DY = 0.62  # vertical distance between the two agent lanes
XT = 8.6   # x of the targets


def step(ax, x, y, n):
    ax.text(x, y, str(n), ha="center", va="center", fontsize=FS, color="w", zorder=6,
            bbox=dict(boxstyle="circle,pad=0.18", fc="0.2", ec="none"))


def panel(ax, y0, title, col, prio):
    ax.text(-1.95, y0 + 0.55, title, fontsize=7.5, va="bottom", ha="left", color=col)
    for k, nm in enumerate(("agent 0", "agent 1")):
        y = y0 - k * DY
        ax.plot([0, 6.4], [y, y], color="0.82", lw=3.2, solid_capstyle="butt", zorder=1)
        ax.text(-0.15, y, nm, ha="right", va="center", fontsize=FS)
    # step 1: clue found by agent 0, record sent (dashed, delayed)
    ax.plot(1.0, y0, "o", color=NEU, ms=4.5, zorder=3)
    step(ax, 1.0, y0 + 0.33, 1)
    ax.annotate("", (3.6, y0 - DY), (1.0, y0),
                arrowprops=dict(arrowstyle="-|>", lw=.9, color=NEU, ls="--", shrinkA=2, shrinkB=2), zorder=2)
    # step 2: record arrives, beliefs equal
    ax.plot(3.6, y0 - DY, "o", color=NEU, ms=4.5, zorder=3)
    step(ax, 3.6, y0 - DY - 0.36, 2)
    # step 3: decisions
    step(ax, 5.6, y0 + 0.33, 3)
    tx, ty = (y0 - DY / 2 + 0.24), (y0 - DY / 2 - 0.24)  # X above, Y below
    ax.plot(XT, tx, "*", color="k", ms=8, zorder=4)
    ax.text(XT + 0.3, tx, "X (best)", fontsize=FS, va="center", ha="left")
    ax.plot(XT, ty, "s", color="k", mfc="w", ms=4.5, zorder=4)
    ax.text(XT + 0.3, ty, "Y", fontsize=FS, va="center", ha="left")
    kw = dict(arrowstyle="-|>", lw=1.1, color=col, shrinkA=0, shrinkB=5)
    ax.annotate("", (XT, tx), (6.4, y0), arrowprops=kw, zorder=3)
    if prio:
        ax.annotate("", (XT, ty), (6.4, y0 - DY), arrowprops=kw, zorder=3)
    else:
        ax.annotate("", (XT, tx), (6.4, y0 - DY), arrowprops=kw, zorder=3)
        ax.text(XT, tx + 0.22, "collision", fontsize=FS, ha="center", va="bottom")


def main(out):
    fig, ax = plt.subplots(figsize=(3.3, 2.75))
    ax.set_xlim(-2.0, 10.4)
    ax.set_ylim(-4.7, 0.95)
    ax.axis("off")
    y0 = 0.0
    panel(ax, y0, r"HYBRID (stale in-flight model)", HYB, False)
    ax.text(-1.95, y0 - DY - 0.62,
            r"\textbf{3}: each assumes the partner is still on the stale plan,"
            "\n" r"so both pick the same best target X",
            fontsize=FS, va="top", ha="left", linespacing=1.1, color="0.15")
    y1 = -2.55
    panel(ax, y1, r"HYBRID-P (priority-order prediction)", HYBP, True)
    ax.text(-1.95, y1 - DY - 0.62,
            r"\textbf{3}: agent 1 predicts agent 0 takes X, so it takes Y",
            fontsize=FS, va="top", ha="left", color="0.15")
    # In-figure step key (replaces the caption narration).
    for x, yk, n, txt in ((-1.68, -4.18, 1, "clue found; record sent (dashed: delayed)"),
                          (-1.68, -4.50, 2, "record arrives: beliefs equal"),
                          (4.85, -4.50, 3, "next look chosen")):
        step(ax, x, yk, n)
        ax.text(x + 0.33, yk, txt, fontsize=FS, va="center", ha="left", color="0.15")
    fig.subplots_adjust(left=0.01, right=0.995, top=0.995, bottom=0.005)
    fig.savefig(out)
    fig.savefig(out.replace(".pdf", ".png"), dpi=200)


if __name__ == "__main__":
    main(sys.argv[1] if len(sys.argv) > 1 else os.path.join(os.path.dirname(os.path.abspath(__file__)),
                                                            "paper", "figs", "fig3_mechanism.pdf"))
