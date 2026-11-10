"""Figure 2: agent-loss regimes R7a-d, dP vs IMPLICIT (top) and bytes/episode (bottom).

Parses the compact table in results/loss_baselines.md (no simulation is run).
Run from Tools/:  python -m ForestSearch.make_fig2 [loss_baselines.md] [out.pdf]
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
CELL = re.compile(r"([+-]\d\.\d+) \[([+-]\d\.\d+),([+-]\d\.\d+)\](?: \((\d+(?:\.\d+)?)\))?")


def parse(md):
    txt = open(md).read()
    sec = txt.split("## Compact table")[1].split("\n\n")[1]
    rows = [r for r in sec.splitlines() if r.startswith("| R7")]
    hdr = [re.sub(r" \(.*", "", h.strip()) for h in re.search(r"\| regime.*", sec).group(0).strip("|").split("|")][1:]
    out = {}
    for r in rows:
        cells = [c.strip() for c in r.strip("|").split("|")]
        out[cells[0]] = {h: tuple(float(x) if x else 0.0 for x in CELL.match(c).groups())
                         for h, c in zip(hdr, cells[1:])}
    return out


# strategy: (colour, marker, label); Okabe-Ito
S = [("ORACLE", "#000000", "D", "ORACLE"), ("HYBRID_P", "#009E73", "^", "HYBRID-P"),
     ("HEALTH", "#CC79A7", "o", "HEALTH"), ("HEARTBEAT", "#CC79A7", "s", "HEARTBEAT"),
     ("INDEP", "#999999", "v", "INDEP")]


def fmt(b):
    return r"$\approx$%.1f kB" % (b / 1000) if b >= 500 else r"$\approx$%d B" % round(b)


def main(md, out):
    d = parse(md)
    regs = list(d)
    fig, (a, b) = plt.subplots(2, 1, figsize=(3.3, 4.5), gridspec_kw=dict(height_ratios=[1.7, 1]))
    off = np.linspace(-.3, .3, len(S))
    a.axhline(0, color="0.35", lw=.8)
    top = [t for t in S if t[0] != "HEARTBEAT"]  # identical to HEALTH on the ideal link
    off = np.linspace(-.27, .27, len(top))
    for k, (s, c, m, lab) in enumerate(top):
        x = np.arange(4) + off[k]
        y = np.array([d[r][s][0] for r in regs])
        lo = np.array([d[r][s][1] for r in regs])
        hi = np.array([d[r][s][2] for r in regs])
        a.errorbar(x, y, yerr=[y - lo, hi - y], fmt=m, color=c, ms=3.2, lw=.9, capsize=0,
                   mfc=c if s != "INDEP" else "w", label=lab if s != "HEALTH" else "HEALTH = HEARTBEAT")
    a.set_ylim(-0.045, 0.125)
    a.set_xlim(-.6, 3.6)
    a.set_xticks(range(4))
    labs = ["R7a\nno travel", "R7b\n+ travel", "R7c\nearly loss", r"R7d" + "\n" + r"$8{\times}8$ grid"]
    a.set_xticklabels(labs, fontsize=6.8)
    a.set_ylabel(r"$\Delta P$ vs.\ IMPLICIT")
    a.legend(loc="lower center", bbox_to_anchor=(0.5, 1.0), ncol=2, fontsize=6.8, frameon=False,
             handletextpad=.2, columnspacing=1.0, borderaxespad=.2)
    a.grid(axis="y", lw=.3, alpha=.5)
    a.text(2.3, -0.004, "default plan (silent)", fontsize=6.8, ha="center", va="top", color="0.25")
    # bytes panel
    bs = [("HYBRID_P", "#009E73", "HYBRID-P"), ("HEALTH", "#CC79A7", "HEALTH"),
          ("HEARTBEAT", "#CC79A7", "HEARTBEAT")]
    w = .25
    for k, (s, c, lab) in enumerate(bs):
        x = np.arange(4) + (k - 1) * w * 1.1
        v = np.array([d[r][s][3] for r in regs])
        b.bar(x, v, w, color=c if s != "HEARTBEAT" else "w", edgecolor=c, hatch="////" if s == "HEARTBEAT" else None,
              lw=.6, label=lab)
        if s != "HEARTBEAT":
            for xi, vi in zip(x, v):
                b.text(xi, vi * 1.25, fmt(vi), ha="center", va="bottom", fontsize=6.8, color=c, rotation=90,
                       )
    b.legend(loc="upper left", fontsize=6.8, frameon=False, ncol=3, handlelength=.8, handletextpad=.3,
             columnspacing=.8, bbox_to_anchor=(0, 1.22))
    b.set_yscale("log")
    b.set_ylim(3, 1e5)
    b.set_xlim(-.6, 3.6)
    b.set_xticks(range(4))
    b.set_xticklabels([l.replace("\n", " ") if False else l.split("\n")[0] for l in labs], fontsize=6.8)
    b.set_ylabel("bytes / episode")
    b.set_yticks([10, 100, 1000])
    b.set_yticklabels(["10", "100", "1000"])
    b.grid(axis="y", lw=.3, alpha=.5)
    for ax_ in (a, b):
        ax_.tick_params(labelsize=6.8, length=2.5)
        for sp in ("top", "right"):
            ax_.spines[sp].set_visible(False)
    fig.subplots_adjust(left=0.19, right=0.99, top=0.89, bottom=0.07, hspace=0.45)
    fig.savefig(out)
    fig.savefig(out.replace(".pdf", ".png"), dpi=200)


if __name__ == "__main__":
    md = sys.argv[1] if len(sys.argv) > 1 else os.path.join(HERE, "results", "loss_baselines.md")
    main(md, sys.argv[2] if len(sys.argv) > 2 else os.path.join(HERE, "paper", "figs", "fig2_loss.pdf"))
