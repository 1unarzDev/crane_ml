"""Figure 2: agent loss on fresh seeds 800-811 (results/beacon_robustness.md, Part A, ideal link).

(a) dP vs IMPLICIT with 95% paired cluster-bootstrap CIs in the four permanent-loss regimes R7a-d
    (random-time loss, loss with travel, early loss, 8x8 with travel) for ORACLE, HYBRID_P, HEALTH,
    TAKEOVER_B (best balanced silent hedge, tuned on DEV) and INDEP.
(b) Trade-off: mean dP under loss (R7a-d) vs mean dP on the four healthy cells (R0, H-b, H-d, R3a);
    the other DEV-tuned silent hedges are drawn as small open markers.
(c) Bytes per episode under loss (mean over R7a-d, range as whiskers), log scale.

Parses the per-cell tables in results/beacon_robustness.md (no simulation is run).
Run from Tools/:  python -m ForestSearch.make_fig2 [beacon_robustness.md] [out.pdf]
"""
import os
import re
import sys

import matplotlib
import numpy as np

matplotlib.use("Agg")
matplotlib.rcParams.update({"pdf.fonttype": 42, "ps.fonttype": 42, "text.usetex": True,
                            "font.family": "serif", "font.size": 7})
import matplotlib.pyplot as plt  # noqa: E402

HERE = os.path.dirname(os.path.abspath(__file__))
ROW = re.compile(r"^\| (R7[a-d]|R0|H-b[^|]*|H-d[^|]*|R3a) \| d0_loss0 \| (\w+) \| [\d.]+ \| "
                 r"(?:([+-]\d\.\d+) \[([+-]\d\.\d+),([+-]\d\.\d+)\]\*?|-) \| [^|]* \| ([\d.]+) \|$")
LOSS = ["R7a", "R7b", "R7c", "R7d"]
HEALTHY = ["R0", "H-b", "H-d", "R3a"]


def parse(md):
    """{cell: {strategy: (d, lo, hi, bytes)}} for the ideal link (d0_loss0) of Part A."""
    part_a = open(md).read().split("## Part A")[1].split("## Part B")[0]
    out = {}
    for line in part_a.splitlines():
        m = ROW.match(line.strip())
        if not m:
            continue
        cell = m.group(1).split(" ")[0]
        d = tuple(float(x) for x in m.group(3, 4, 5)) if m.group(3) else (0.0, 0.0, 0.0)
        out.setdefault(cell, {})[m.group(2)] = d + (float(m.group(6)),)
    return out


# strategy: (colour, marker, label). Okabe-Ito plus greys; TAKEOVER grey-brown, distinct from INDEP grey.
TK = "#8C6D46"
S = [("ORACLE", "#000000", "D", "ORACLE"), ("HYBRID_P", "#009E73", "^", "HYBRID-P"),
     ("HEALTH", "#CC79A7", "o", "HEALTH"), ("TAKEOVER_B", TK, "s", "TAKEOVER"),
     ("INDEP", "#666666", "v", "INDEP")]
OTHER = ["TAKEOVER_L", "REDUNDANT_L", "BAYES_L", "SECSWEEP_L", "SECSWEEP_B"]


def main(md, out):
    d = parse(md)
    for c in LOSS + HEALTHY:
        assert all(s in d[c] for s, *_ in S), (c, sorted(d[c]))
    fig = plt.figure(figsize=(3.3, 3.85))
    gs = fig.add_gridspec(2, 2, height_ratios=[1.35, 1], width_ratios=[1.4, 1], hspace=0.62, wspace=0.62)
    a = fig.add_subplot(gs[0, :])
    b = fig.add_subplot(gs[1, 0])
    c = fig.add_subplot(gs[1, 1])

    # (a) gains under loss
    a.axhline(0, color="0.35", lw=.8)
    off = np.linspace(-.3, .3, len(S))
    for k, (s, col, m, lab) in enumerate(S):
        x = np.arange(4) + off[k]
        y = np.array([d[r][s][0] for r in LOSS])
        lo = np.array([d[r][s][1] for r in LOSS])
        hi = np.array([d[r][s][2] for r in LOSS])
        a.errorbar(x, y, yerr=[y - lo, hi - y], fmt=m, color=col, ms=3.2, lw=.9, capsize=0,
                   mfc="w" if s == "INDEP" else col, label=lab)
    a.set_ylim(-0.03, 0.13)
    a.set_xlim(-.6, 3.6)
    a.set_xticks(range(4))
    a.set_xticklabels(["random-time\nloss", "loss with\ntravel", "early\nloss",
                       r"$8{\times}8$ grid" + "\n" + "with travel"], fontsize=7)
    a.set_ylabel(r"$\Delta P$ vs.\ IMPLICIT")
    a.legend(loc="lower center", bbox_to_anchor=(0.5, 1.0), ncol=3, fontsize=7, frameon=False,
             handletextpad=.2, columnspacing=1.0, borderaxespad=.2)
    a.grid(axis="y", lw=.3, alpha=.5)

    # (b) loss gain vs healthy-team cost
    def means(s):
        return (np.mean([d[r][s][0] for r in LOSS]), np.mean([d[r][s][0] for r in HEALTHY]))
    b.axhline(0, color="0.35", lw=.6)
    b.axvline(0, color="0.35", lw=.6)
    for s in OTHER:
        if all(s in d[r] for r in LOSS + HEALTHY):
            x, y = means(s)
            b.plot(x, y, "s", ms=2.6, mfc="w", mec=TK, mew=.6, label=None)
    for s, col, m, lab in S:
        x, y = means(s)
        b.plot(x, y, m, ms=3.6, color=col, mfc="w" if s == "INDEP" else col)
    b.text(0.056, 0.006, "HEALTH", fontsize=7, color="#CC79A7", ha="center", va="bottom")
    b.text(0.021, -0.093, "INDEP", fontsize=7, color="#666666", ha="right", va="center")
    b.text(0.013, 0.006, "TAKEOVER", fontsize=7, color=TK, ha="center", va="bottom")
    b.text(0.041, -0.084, "other silent\nhedges", fontsize=7, color=TK, ha="left", va="center")
    b.set_xlim(-0.005, 0.075)
    b.set_ylim(-0.11, 0.03)
    b.set_xticks([0, 0.03, 0.06])
    b.set_xticklabels(["0", ".03", ".06"])
    b.set_yticks([-0.1, -0.05, 0])
    b.set_yticklabels(["$-$.10", "$-$.05", "0"])
    b.set_xlabel(r"mean $\Delta P$, loss")
    b.set_ylabel(r"mean $\Delta P$, healthy")
    b.grid(lw=.3, alpha=.5)

    # (c) bytes under loss
    bs = [("HYBRID_P", "#009E73", "HYB.-P"), ("HEALTH", "#CC79A7", "HEALTH")]
    for k, (s, col, lab) in enumerate(bs):
        v = np.array([d[r][s][3] for r in LOSS])
        c.bar(k, v.mean(), .6, color=col, lw=0)
        c.errorbar(k, v.mean(), yerr=[[v.mean() - v.min()], [v.max() - v.mean()]], color="0.2", lw=.7, capsize=1.5)
        txt = r"%.1f--%.1f kB" % (v.min() / 1000, v.max() / 1000) if v.min() >= 500 else r"%d--%d B" % (
            round(v.min()), round(v.max()))
        c.text(k, v.max() * 1.5, txt, ha="center", va="bottom", fontsize=7, color=col)
    c.set_yscale("log")
    c.set_ylim(1, 1e5)
    c.set_xlim(-.75, 1.75)
    c.set_xticks([0, 1])
    c.set_xticklabels([b_[2] for b_ in bs], fontsize=7, rotation=20)
    c.set_yticks([1, 10, 100, 1000])
    c.set_yticklabels(["1", "10", "100", "1000"])
    c.set_ylabel("bytes / episode")
    c.grid(axis="y", lw=.3, alpha=.5)

    for ax_ in (a, b, c):
        ax_.tick_params(labelsize=7, length=2.5)
        for sp in ("top", "right"):
            ax_.spines[sp].set_visible(False)
    a.text(-0.15, 1.2, "(a)", transform=a.transAxes, fontsize=8, fontweight="bold", va="bottom")
    b.text(-0.45, 1.04, "(b)", transform=b.transAxes, fontsize=8, fontweight="bold", va="bottom")
    c.text(-0.72, 1.04, "(c)", transform=c.transAxes, fontsize=8, fontweight="bold", va="bottom")
    fig.subplots_adjust(left=0.17, right=0.98, top=0.88, bottom=0.10)
    fig.savefig(out)
    fig.savefig(out.replace(".pdf", ".png"), dpi=200)


if __name__ == "__main__":
    md = sys.argv[1] if len(sys.argv) > 1 else os.path.join(HERE, "results", "beacon_robustness.md")
    main(md, sys.argv[2] if len(sys.argv) > 2 else os.path.join(HERE, "paper", "figs", "fig2_loss.pdf"))
