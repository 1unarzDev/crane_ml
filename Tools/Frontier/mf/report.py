# SPDX-License-Identifier: GPL-3.0-or-later
"""Format analysis.json and synthetic_study.json (written by analyze.py and run_retro.py) as Markdown tables."""
import json
import os
import sys

D = os.path.join(os.path.expanduser("~"), ".cache", "frontier", "mf")


def merge(paths):
    """Merge per-scenario pieces written by run_retro.py --scenarios X --out piece_X.json (and --power-only)."""
    r = json.load(open(paths[0]))
    for p in paths[1:]:
        q = json.load(open(p))
        r["scenarios"].update(q.get("scenarios", {}))
        if "tau_power" in q:
            r["tau_power"] = q["tau_power"]
    return r


def synth_tables(path):
    r = merge(path) if isinstance(path, list) else json.load(open(path))
    G = r["grid_h"]
    show = [h for h in (0.25, 0.5, 1, 2, 4, 6) if h in G]
    out = ["costs (s/query): L1 %.2f, L2 %.1f; tuned halving: eta=%d, first L1 rung n0=%d; reps=%d; eps=%.1f\n" % (
        r["costs_s"]["L1"], r["costs_s"]["L2"], r["sh_tuned"]["eta"], r["sh_tuned"]["n0"], r["reps"], r["eps"])]
    out.append("| scenario | tau(f1,f2) all / <=4 / >=6 m/s | regret of trusting L1 (face value) |\n|---|---|---|")
    for sc, d in r["scenarios"].items():
        t = d["tau_all_lo_hi"]
        out.append("| %s | %.2f / %.2f / %.2f | %.2f |" % (sc, t[0], t[1], t[2], d["face_value_regret"]))
    for sc, d in r["scenarios"].items():
        out.append("\n**%s**: mean simple regret at L2 (+- s.e.) vs compute-hours\n" % sc)
        out.append("| selector | " + " | ".join("%g h" % h for h in show) + " | h to median<=eps | h to mean<=1.0 | hours used |")
        out.append("|---|" + "---|" * (len(show) + 3))
        for name, s in d["selectors"].items():
            cells = ["%.2f (%.2f)" % (s["mean"][G.index(h)], s["se"][G.index(h)]) for h in show]
            hit = next((h for h, m in zip(G, s["median"]) if m <= r["eps"]), None)
            hit1 = next((h for h, m in zip(G, s["mean"]) if m <= 1.0), None)
            out.append("| %s | %s | %s | %s | %.2f |" % (name, " | ".join(cells), (">%g" % G[-1]) if hit is None else "%g" % hit,
                                                      (">%g" % G[-1]) if hit1 is None else "%g" % hit1, s["hours_used_mean"]))
    if "tau_power" in r:
        out.append("\n**Kendall tau power** (bootstrap 95 % CI half-width, bivariate-normal copula)\n")
        out.append("| n units | halfwidth at tau=0.6 | P(CI excl. 0, tau=0.6) | halfwidth at tau=0.2 | P(CI excl. 0, tau=0.2) |\n|---|---|---|---|---|")
        for row in r["tau_power"]:
            a, b = row["tau0.6"], row["tau0.2"]
            out.append("| %d | %.2f | %.2f | %.2f | %.2f |" % (row["n"], a["halfwidth"], a["P(CI excludes 0)"], b["halfwidth"], b["P(CI excludes 0)"]))
    return "\n".join(out)


if __name__ == "__main__":
    print(synth_tables(sys.argv[1:] if len(sys.argv) > 2 else (sys.argv[1] if len(sys.argv) > 1 else os.path.join(D, "synthetic_study.json"))))
