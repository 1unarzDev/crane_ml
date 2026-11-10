# SPDX-License-Identifier: GPL-3.0-or-later
"""Fidelity comparison: NavPilot 2.5D twin vs the CRANE .NET autonomy harness (3D) on the same scenario family.

  python fidelity.py --crane RESULTS.jsonl [...] --navpilot NAVPILOT.jsonl [--md OUT.md]

CRANE rows may be the lead's AutonomyHarness results.jsonl (MissionResult field names) or CraneExperiment rows
(r_ prefixed); only straight cross_k missions are compared (NavPilot's corridor convention). Per speed: success and
collision rates with cluster-bootstrap CIs (cluster = mission), the paired CRANE - NavPilot difference over the same
(mission, run seed) with a paired cluster bootstrap, perception-to-command age (per-run p50 / p99), true stopping-margin
violation fraction and min hard clearance. "agree" = the 95 % CI of the paired difference contains 0.
"""
import argparse
import json
import math
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from aggregate import _q, cluster_bootstrap, paired_cluster_bootstrap  # noqa: E402


def norm_crane(r):
    if "r_Outcome" in r:
        g = lambda k: r.get("r_" + k)  # noqa: E731
        mission, speed, seed = r["mission"], float(r["speed"]), int(r["run_seed"])
    else:
        g = r.get
        mission, speed, seed = r["Name"], float(r["VLimit"]), int(r["RunSeed"])
    out = g("Outcome")
    mt = g("MarginTicks") or 0
    return {"harness": "crane_3d", "mission": mission, "speed": speed, "run_seed": seed,
            "pair_key": "f%d|%s|r%d|d0" % (int(g("ForestSeed") or 1337), mission, seed), "cluster": mission,
            "outcome": out, "success": out == "success", "collision": out == "collision",
            "age_p50": g("AgeP50S"), "age_p99": g("AgeP99S"), "margin_viol_frac": (g("MarginViolations") or 0) / mt if mt else 0.0,
            "min_clear": g("MinClearanceHardM"), "mean_speed": g("MeanSpeed")}


def norm_np(r):
    return {"harness": "navpilot_2p5d", "mission": r["mission"], "speed": float(r["speed"]), "run_seed": int(r["run_seed"]),
            "pair_key": r["pair_key"], "cluster": r["mission"], "outcome": r["outcome"], "success": r["success"],
            "collision": r["collision"], "age_p50": r["age_p50"], "age_p99": r["age_p99"],
            "margin_viol_frac": r["margin_viol_frac"], "min_clear": r["min_clear"], "mean_speed": r["mean_speed"]}


def load(paths, f):
    rows = []
    for p in paths:
        for line in open(p):
            if line.strip():
                rows.append(f(json.loads(line)))
    return rows


def rate(key):
    return lambda rs: sum(1.0 for r in rs if r[key]) / len(rs) if rs else float("nan")


def mean(key):
    return lambda rs: sum(r[key] for r in rs if r[key] is not None) / max(1, sum(1 for r in rs if r[key] is not None))


def ci(e, c, d=2):
    f = "%%.%df" % d
    return (f + " [" + f + ", " + f + "]") % (e, c[0], c[1])


def report(crane, navp):
    crane = [r for r in crane if r["mission"].startswith("cross_")]
    # de-duplicate CRANE rows (several lead files): keep the first per (mission, speed, seed)
    seen, cr = set(), []
    for r in crane:
        k = (r["mission"], r["speed"], r["run_seed"])
        if k not in seen:
            seen.add(k)
            cr.append(r)
    speeds = sorted({r["speed"] for r in cr} & {r["speed"] for r in navp})
    L = ["| v_lim | n (pairs) | CRANE success | NavPilot success | paired diff success (C-N) | CRANE collision | NavPilot collision | paired diff collision (C-N) | verdict |",
         "|---|---|---|---|---|---|---|---|---|"]
    A = ["| v_lim | CRANE age p50 / p99 [ms] (median over runs) | NavPilot age p50 / p99 [ms] | paired diff p99 (C-N) [ms] | CRANE margin viol. [%] | NavPilot margin viol. [%] | paired diff viol. (C-N) [pp] | CRANE min hard clr p10 / min [m] | NavPilot min hard clr p10 / min [m] | CRANE mean v | NavPilot mean v |",
         "|---|---|---|---|---|---|---|---|---|---|---|"]
    for v in speeds:
        c = [r for r in cr if r["speed"] == v]
        n = [r for r in navp if r["speed"] == v]
        keys = {r["pair_key"] for r in c} & {r["pair_key"] for r in n}
        c = [r for r in c if r["pair_key"] in keys]
        n = [r for r in n if r["pair_key"] in keys]
        if not keys:
            continue
        cs, ns = cluster_bootstrap(c, rate("success")), cluster_bootstrap(n, rate("success"))
        cc, nc = cluster_bootstrap(c, rate("collision")), cluster_bootstrap(n, rate("collision"))
        ds = paired_cluster_bootstrap(n, c, lambda r: 1.0 if r["success"] else 0.0)
        dc = paired_cluster_bootstrap(n, c, lambda r: 1.0 if r["collision"] else 0.0)
        agree_s = ds[1][0] <= 0 <= ds[1][1]
        agree_c = dc[1][0] <= 0 <= dc[1][1]
        verdict = ("agree" if agree_s and agree_c else "DIVERGE (" + ", ".join(x for x, ok in (("success", agree_s), ("collision", agree_c)) if not ok) + ")")
        L.append("| %g | %d | %s | %s | %s | %s | %s | %s | %s |" % (v, len(keys), ci(*cs), ci(*ns), ci(ds[0], ds[1]), ci(*cc), ci(*nc), ci(dc[0], dc[1]), verdict))
        dp = paired_cluster_bootstrap(n, c, lambda r: 1e3 * r["age_p99"])
        dm = paired_cluster_bootstrap(n, c, lambda r: 100 * r["margin_viol_frac"])
        A.append("| %g | %.0f / %.0f | %.0f / %.0f | %s | %.2f | %.2f | %s | %.2f / %.2f | %.2f / %.2f | %.2f | %.2f |" % (
            v, 1e3 * _q([r["age_p50"] for r in c], 0.5), 1e3 * _q([r["age_p99"] for r in c], 0.5),
            1e3 * _q([r["age_p50"] for r in n], 0.5), 1e3 * _q([r["age_p99"] for r in n], 0.5), ci(dp[0], dp[1], 1),
            100 * mean("margin_viol_frac")(c), 100 * mean("margin_viol_frac")(n), ci(dm[0], dm[1], 2),
            _q([r["min_clear"] for r in c], 0.1), min(r["min_clear"] for r in c), _q([r["min_clear"] for r in n], 0.1), min(r["min_clear"] for r in n),
            mean("mean_speed")(c), mean("mean_speed")(n)))
    runs = ["| mission | v | CRANE | NavPilot |", "|---|---|---|---|"]
    by = {(r["mission"], r["speed"]): r for r in navp}
    for r in sorted(cr, key=lambda r: (r["speed"], r["mission"])):
        q = by.get((r["mission"], r["speed"]))
        if q and (r["outcome"] != q["outcome"]):
            runs.append("| %s | %g | %s (clr %.2f) | %s (clr %.2f) |" % (r["mission"], r["speed"], r["outcome"], r["min_clear"] or float("nan"), q["outcome"], q["min_clear"]))
    return "\n".join(["### Outcomes vs speed (cross_0..9, run seed 1, paired by mission)", "", *L, "",
                      "### Timing, stopping margin, clearance", "", *A, "", "### Runs whose outcome differs", "", *runs])


def main(argv=None):
    ap = argparse.ArgumentParser()
    ap.add_argument("--crane", nargs="+", required=True)
    ap.add_argument("--navpilot", nargs="+", required=True)
    ap.add_argument("--md")
    a = ap.parse_args(argv)
    md = report(load(a.crane, norm_crane), load(a.navpilot, norm_np))
    print(md)
    if a.md:
        with open(a.md, "w") as f:
            f.write(md + "\n")


if __name__ == "__main__":
    main()
