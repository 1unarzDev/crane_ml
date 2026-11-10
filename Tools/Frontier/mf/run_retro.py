# SPDX-License-Identifier: GPL-3.0-or-later
"""D4 synthetic-but-calibrated selector study + Kendall-tau power calculation (SYNTHETIC-CALIBRATED; not evidence of real transfer).

    ~/.cache/research/fs-scripts/fsrun ~/.cache/research/fs-venv/bin/python Tools/Frontier/mf/run_retro.py --reps 60

Outputs JSON to ~/.cache/frontier/mf/synthetic_study.json and prints Markdown tables. All cost numbers come from costs.jsonl
(audit seed: NavPilot 1.33 s/episode, .NET 32 s/run); `--costs measured` swaps in the logged medians (3.3 s, 48 s).
"""
import argparse
import json
import os
import sys
import time

import numpy as np

sys.dont_write_bytecode = True
HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
sys.path.insert(0, os.path.normpath(os.path.join(HERE, "..")))
import costs as costmod  # noqa: E402
import data  # noqa: E402
import replay  # noqa: E402
import synth  # noqa: E402

GRID_H = [0.05, 0.1, 0.17, 0.25, 0.5, 0.75, 1, 1.5, 2, 3, 4, 6]
BUDGET_H = 6.0
EPS = 0.5  # utility units (m/s-equivalent); about the resolution of 3 L2 runs at a 5 % collision rate


def one_rep(cfgs, p1, a1, sd, scenario, rep, name, cost, kw=None, seed_off=0):
    sid = rep + seed_off
    p2, a2 = synth.build_truth(cfgs, p1, a1, scenario, sid)
    tab = synth.Table(cfgs, p1, a1, sd, p2, a2, 40, 16, sid)
    env = replay.Env(tab, cost, np.random.default_rng([sid, 1]))
    sel = replay.make(name, tab.N, np.random.default_rng([sid, 2]), tab, **(kw or {}))
    out, hours = replay.run(sel, env, BUDGET_H * 3600, GRID_H)
    return [out[h] for h in GRID_H], hours, tab


def tune_sh(cfgs, p1, a1, sd, cost, reps=20):
    """Tune halving (eta, first L1 rung, include L1 rungs or not) on tuning seeds 1000+ of the 'moderate' scenario; AUC of mean regret."""
    best, rows = None, []
    for eta in (2, 3, 4):
        for n0 in (2, 4, 8):
            rungs = ((replay.LO, n0), (replay.LO, 2 * n0), (replay.HI, 2), (replay.HI, 4), (replay.HI, 8), (replay.HI, 16))
            auc = np.mean([np.mean(one_rep(cfgs, p1, a1, sd, "moderate", r, "succ-halving", cost, dict(eta=eta, rungs=rungs), 1000)[0])
                           for r in range(reps)])
            rows.append((eta, n0, float(auc)))
            if best is None or auc < best[2]:
                best = (eta, n0, float(auc))
    return best, rows


def tau_power(ns=(5, 10, 12, 20, 40), taus=(0.6, 0.2), reps=150, B=200, seed=0):
    """Monte Carlo: half-width of the 95 % bootstrap CI of Kendall tau-b with n units (bivariate normal copula)."""
    rng = np.random.default_rng(seed)
    out = []
    for n in ns:
        row = {"n": n}
        for t in taus:
            r = np.sin(np.pi * t / 2)
            hw, excl0, lo_all, hi_all = [], 0, [], []
            for _ in range(reps):
                z = rng.multivariate_normal([0, 0], [[1, r], [r, 1]], n)
                tau = synth.kendall(z[:, 0], z[:, 1])
                bs = []
                for _ in range(B):
                    i = rng.integers(0, n, n)
                    v = synth.kendall(z[i, 0], z[i, 1])
                    if not np.isnan(v):
                        bs.append(v)
                lo, hi = np.quantile(bs, [0.025, 0.975])
                hw.append((hi - lo) / 2)
                excl0 += lo > 0
                lo_all.append(lo)
                hi_all.append(hi)
            row["tau%.1f" % t] = {"halfwidth": float(np.mean(hw)), "P(CI excludes 0)": excl0 / reps,
                                  "lo_mean": float(np.mean(lo_all)), "hi_mean": float(np.mean(hi_all))}
        out.append(row)
    return out


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--reps", type=int, default=60)
    ap.add_argument("--costs", choices=["audit", "measured"], default="audit")
    ap.add_argument("--out", default=os.path.join(os.path.expanduser("~"), ".cache", "frontier", "mf", "synthetic_study.json"))
    ap.add_argument("--no-power", action="store_true")
    ap.add_argument("--scenarios", default=",".join(synth.SCENARIOS), help="comma list; run pieces separately and merge with report.py")
    ap.add_argument("--power-only", action="store_true")
    a = ap.parse_args()
    bb = synth.e3_backbone(data.navpilot_e3("heldout"))
    cfgs, p1, a1, sd = synth.build_configs(bb)
    if a.costs == "audit":
        cost = {"L1": costmod.cost_per_unit("navpilot"), "L2": costmod.cost_per_unit("dotnet_harness")}
    else:
        cost = {"L1": 3.33, "L2": 48.2}
    t0 = time.time()
    best, tune_rows = tune_sh(cfgs, p1, a1, sd, cost)
    eta, n0, _ = best
    sh_kw = dict(eta=eta, rungs=((replay.LO, n0), (replay.LO, 2 * n0), (replay.HI, 2), (replay.HI, 4), (replay.HI, 8), (replay.HI, 16)))
    selectors = [("random", {}), ("all-high", {}), ("low-only", {}), ("sh-seeds-L2", {"eta": eta}),
                 ("succ-halving", sh_kw), ("screen+topk", {}), ("AR1-BLR", {})]
    res = {"costs_s": cost, "sh_tuned": {"eta": eta, "n0": n0, "grid": tune_rows}, "reps": a.reps, "grid_h": GRID_H,
           "eps": EPS, "scenarios": {}}
    for sc in ([] if a.power_only else a.scenarios.split(",")):
        taus, regret_l1pick = [], []
        v = np.array([c[1] for c in cfgs])
        for r in range(a.reps):
            p2, a2 = synth.build_truth(cfgs, p1, a1, sc, r)
            f1, f2 = synth.utility(p1, a1), synth.utility(p2, a2)
            taus.append([synth.kendall(f1, f2), synth.kendall(f1[v <= 4], f2[v <= 4]), synth.kendall(f1[v >= 6], f2[v >= 6])])
            regret_l1pick.append(f2.max() - f2[np.argmax(f1)])
        d = {"tau_all_lo_hi": np.mean(taus, 0).tolist(), "face_value_regret": float(np.mean(regret_l1pick)), "selectors": {}}
        for name, kw in selectors:
            R, H = [], []
            for r in range(a.reps):
                reg, hrs, _ = one_rep(cfgs, p1, a1, sd, sc, r, name, cost, kw)
                R.append(reg)
                H.append(hrs)
            R = np.array(R)
            d["selectors"][name] = {"mean": R.mean(0).tolist(), "se": (R.std(0, ddof=1) / np.sqrt(len(R))).tolist(),
                                    "median": np.median(R, 0).tolist(), "p_le_eps": (R <= EPS).mean(0).tolist(),
                                    "hours_used_mean": float(np.mean(H))}
            print("%-16s %-14s %s  (%.0fs)" % (sc, name, " ".join("%.2f" % x for x in R.mean(0)), time.time() - t0), flush=True)
        res["scenarios"][sc] = d
    if a.power_only or not a.no_power:
        res["tau_power"] = tau_power()
    os.makedirs(os.path.dirname(a.out), exist_ok=True)
    json.dump(res, open(a.out, "w"), indent=1)
    print("wrote", a.out)


if __name__ == "__main__":
    main()
