"""Figures (Type 1 fonts via usetex).  python -m MissionExact.make_figs"""
from __future__ import annotations

import json
import os
from pathlib import Path

os.environ["PATH"] = "/home/lunarz/.cache/research/tinytex/.TinyTeX/bin/x86_64-linux:" + os.environ["PATH"]
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

matplotlib.rcParams.update({"text.usetex": True, "font.family": "serif", "font.size": 9,
                            "pdf.fonttype": 42, "axes.spines.top": False,
                            "axes.spines.right": False})
HERE = Path(__file__).resolve().parent
RES, FIG = HERE / "results", HERE / "figs"
C = {"oracle": "#1b4f72", "delayed": "#c0392b", "nocomm": "#7f8c8d", "cover": "#d4ac0d",
     "self": "#27ae60", "leave": "#8e44ad"}


def cdf_plot(name, curves, T, title):
    fig, ax = plt.subplots(figsize=(3.4, 2.4))
    for lab, (rd, col) in curves.items():
        ts = list(range(0, T + 1))
        acc, ys = 0.0, []
        for t in ts:
            acc += rd.get(str(t), rd.get(t, 0.0))
            ys.append(acc)
        ax.step(ts, ys, where="post", label=lab, color=col, lw=1.6)
    ax.set_xlabel(r"time $t$")
    ax.set_ylabel(r"$P(\mathrm{confirmed\ report\ at\ base} \le t)$")
    ax.set_title(title, fontsize=9)
    ax.legend(frameon=False, fontsize=8)
    fig.tight_layout()
    fig.savefig(FIG / name)
    plt.close(fig)


def q2_plot(spec):
    rows = json.loads((RES / f"q2_{spec}.json").read_text())
    ds = [r["d"] for r in rows]
    fig, ax = plt.subplots(figsize=(3.4, 2.5))
    ax.plot(ds, [r["oracle"] for r in rows], "o-", color=C["oracle"], label="ORACLE", lw=1.4, ms=3)
    ax.plot(ds, [r["delayed"] for r in rows], "s-", color=C["delayed"], label="best, delayed comm", lw=1.4, ms=3)
    ax.plot(ds, [r["nocomm"] for r in rows], "^-", color=C["nocomm"], label="best, no comm", lw=1.4, ms=3)
    ax.plot(ds, [r["forced"]["self"] for r in rows], ":", color=C["self"], label="forced: detector self-verifies", lw=1.4)
    ax.plot(ds, [r["forced"]["leave"] for r in rows], "--", color=C["leave"], label="forced: detector leaves", lw=1.4)
    sw = [r["d"] for r in rows if r["forced"]["leave"] > r["forced"]["self"] + 1e-12]
    if sw:
        ax.axvline(min(sw) - 0.5, color="k", lw=0.6, ls="-.")
    ax.set_xlabel(r"message / uplink delay $d$")
    ax.set_ylabel(r"$P(\mathrm{report\ by\ } T)$")
    ax.set_xticks(ds)
    ax.legend(frameon=False, fontsize=6.5)
    fig.tight_layout()
    fig.savefig(FIG / f"q2_delay_sweep_{spec}.pdf")
    plt.close(fig)


if __name__ == "__main__":
    FIG.mkdir(exist_ok=True)
    import sys
    for spec in sys.argv[1:] or ["base"]:
        if (RES / f"q2_{spec}.json").exists():
            q2_plot(spec)
    if (RES / "q1_pick.json").exists():
        pk = json.loads((RES / "q1_pick.json").read_text())
        cdf_plot("q1_report_cdf.pdf",
                 {"ORACLE": (pk["rd_oracle"], C["oracle"]),
                  "best no-comm": (pk["rd_nocomm_team"], C["nocomm"]),
                  "coverage-optimal sweep": (pk["rd_cover"], C["cover"])},
                 pk["inst"]["T"], "Q1")
