# SPDX-License-Identifier: GPL-3.0-or-later
"""Mission-sim experiments (paired seeds; dev layouts for iteration, val layouts for the reported tables).

  run    --exp E1nat|E1forced|E3|ABL|P7 --split dev|val --layouts N --reps R --shard i --nshards n
  report --exp ...            writes results/<exp>.md from results/raw/*.jsonl
  logs                        example decision logs for one E1 seed (results/logs/)

HELD-OUT layouts (90000-99999) are refused unless --allow-heldout (final evaluation only; never used here).
"""
import argparse
import glob
import json
import os
import sys
import time
from collections import Counter, defaultdict
from dataclasses import replace

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
from missionsim import sim, world as W, stats as S, policies as P  # noqa: E402

RAW = os.path.join(HERE, "results", "raw")
SPLIT_BASE = {"dev": 0, "val": 70000, "heldout": 90000}

E1_POLS = ["static", "frontier", "pod", "coordinator", "lease", "rendezvous", "pod_maintain", "pod_deadzone",
           "b45_mule_recheck", "b6_gossip", "oracle"]
E3_POLS = ["lease", "pod_full", "pod", "rendezvous", "racer_lite", "coordinator", "pod_deadzone"]
ABL_POLS = ["pod", "lease", "pod_maintain", "pod_deadzone", "b45_mule_recheck", "rendezvous", "oracle"]
P7_POLS = ["b1_podgreedy", "b45_mule_recheck", "pod_deadzone", "oracle"]


def conditions(exp):
    """List of (variant_name, Scenario, policy, forced_option)."""
    out = []
    if exp == "E1nat":
        sc = W.Scenario(density="medium", e1=True)
        out = [("e1nat", sc, p, None) for p in E1_POLS]
    elif exp == "E1forced":
        for D in (120.0, 240.0, 480.0):
            for val, (conf, prior) in {"low": (0.6, "low"), "high": (0.9, "high")}.items():
                sc = W.Scenario(density="medium", e1_forced=True, e1_conf=conf, e1_prior=prior, deadline_s=D,
                                horizon_s=D + 20.0)
                v = "D%d_%s" % (D, val)
                for opt in P.EVIDENCE_OPTIONS:
                    out.append((v, sc, "pod_deadzone", opt))
                out += [(v, sc, "pod_deadzone", None), (v, sc, "pod", None), (v, sc, "b45_mule_recheck", None)]
    elif exp == "E3":
        sc = W.Scenario(density="medium", weak_target=True, asym_outage=((1, 0, 60, 540), (1, 2, 60, 540)))
        out = [("e3", sc, p, None) for p in E3_POLS]
    elif exp == "ABL":
        base = W.Scenario(density="medium")
        conds = {"baseline": base, "radio_only": replace(base, radio_mult=1.6), "drift_only": replace(base, drift_mult=5.0),
                 "stale_only": replace(base, stale=1.0),
                 "correlated_dense": replace(base, density="dense", correlated=True)}
        for v, sc in conds.items():
            out += [(v, sc, p, None) for p in ABL_POLS]
    elif exp == "P7":
        base = W.Scenario(density="medium")
        conds = {"fa_low": replace(base, fa_mult=0.25, decoy_mult=0.0),
                 "fa_high": replace(base, fa_mult=3.0, decoy_mult=2.5),
                 "align_high": replace(base, align_mult=5.0),
                 "outage_corr": replace(base, density="dense", correlated=True)}
        for v, sc in conds.items():
            out += [(v, sc, p, None) for p in P7_POLS]
    else:
        raise SystemExit("unknown exp " + exp)
    return out


def cmd_run(a):
    base = SPLIT_BASE[a.split]
    if a.split == "heldout" and not a.allow_heldout:
        raise SystemExit("HELD-OUT refused (pass --allow-heldout for the final evaluation only)")
    jobs = []
    for li in range(a.layouts):
        for rep in range(a.reps):
            for v, sc, pol, opt in conditions(a.exp):
                jobs.append((base + li, rep, v, sc, pol, opt))
    jobs = jobs[a.shard::a.nshards]
    os.makedirs(RAW, exist_ok=True)
    path = os.path.join(RAW, "%s_%s_s%d.jsonl" % (a.exp, a.split, a.shard))
    t0 = time.time()
    worlds = {}
    with open(path, "w") as f:
        for k, (lay, rep, v, sc, pol, opt) in enumerate(jobs):
            key = (lay, sc.key())
            if key not in worlds:
                worlds.clear()
                worlds[key] = W.World(lay, sc, allow_heldout=a.allow_heldout)
            try:
                M = sim.run_mission(lay, rep, pol, sc, forced_option=opt, world=worlds[key])
            except Exception as ex:  # record and continue; a crashed mission is reported, never silently dropped
                import traceback
                traceback.print_exc()
                M = {"policy": pol, "layout": lay, "rep": rep, "split": a.split, "success": 0,
                     "failure_cause": "SIM_ERROR:" + type(ex).__name__, "t_first_det": None, "t_local_conf": None,
                     "t_delivery": None, "false_alerts": 0, "false_confirm": 0, "near_miss": 0.0, "energy_wh": 0.0,
                     "dup_area_m2": 0.0, "disagree_m2_mean": 0.0, "t_urgent": None, "t_verify_after_urgent": None,
                     "glimpse_in_outage": None}
            M["variant"] = v
            M["pair_key"] = v
            M["deadline_s"] = sc.deadline_s
            if opt:
                M["policy"] = "forced_" + opt
            f.write(json.dumps(M, default=float) + "\n")
            if k % 50 == 0:
                print("shard %d %d/%d %.0fs" % (a.shard, k, len(jobs), time.time() - t0), flush=True)
    print("done", path, time.time() - t0)


def load(exp, split):
    rows = []
    for p in sorted(glob.glob(os.path.join(RAW, "%s_%s_s*.jsonl" % (exp, split)))):
        with open(p) as f:
            rows += [json.loads(l) for l in f]
    return rows


def fmt(x, nd=2):
    if x is None or (isinstance(x, float) and np.isnan(x)):
        return "n/a"
    return ("%%.%df" % nd) % x


def ci(t, nd=2):
    return "%s [%s, %s]" % (fmt(t[0], nd), fmt(t[1], nd), fmt(t[2], nd))


def qstr(vals, nd=0):
    q = S.quantiles(vals)
    return "/".join(fmt(x, nd) for x in q)


def table_main(rows, pols, ref, variant=None):
    rr = [r for r in rows if variant is None or r["variant"] == variant]
    lines = ["| policy | success (primary) | diff vs %s [95%% CI] | t first det p25/50/75 | t local conf | t delivery | "
             "false alerts | false confirm | near-miss | energy Wh | dup area m2 |" % ref,
             "|---|---|---|---|---|---|---|---|---|---|---|"]
    for p in pols:
        R = [r for r in rr if r["policy"] == p]
        if not R:
            continue
        m = S.cluster_boot_mean(rr, p, "success")
        d = S.cluster_boot_diff(rr, p, ref, "success") if p != ref else (0.0, 0.0, 0.0, 0, 0)
        lines.append("| %s | %s | %s | %s | %s | %s | %.1f | %.2f | %.2f | %.0f | %.0f |" % (
            p, ci(m[:3]), ci(d[:3]), qstr([r["t_first_det"] for r in R]), qstr([r["t_local_conf"] for r in R]),
            qstr([r["t_delivery"] for r in R]), np.mean([r["false_alerts"] for r in R]),
            np.mean([r["false_confirm"] for r in R]), np.mean([r["near_miss"] for r in R]),
            np.mean([r["energy_wh"] for r in R]), np.mean([r["dup_area_m2"] for r in R])))
    return lines


def causes(rows, pols, variant=None):
    rr = [r for r in rows if variant is None or r["variant"] == variant]
    allc = sorted({r["failure_cause"] for r in rr})
    lines = ["| policy | n | " + " | ".join(allc) + " |", "|---|---|" + "---|" * len(allc)]
    for p in pols:
        R = [r for r in rr if r["policy"] == p]
        if R:
            c = Counter(r["failure_cause"] for r in R)
            lines.append("| %s | %d | " % (p, len(R)) + " | ".join(str(c.get(k, 0)) for k in allc) + " |")
    return lines


def header(exp, rows):
    lay = sorted({r["layout"] for r in rows})
    sp = sorted({r["split"] for r in rows})
    return ["Evidence label: IMPLEMENTED (synthetic mission-level simulator, placeholder observation model; not "
            "LIVE_SIM_VALIDATED or PHYSICALLY_CALIBRATED).",
            "", "Split %s, %d layouts (%s..%s), %d missions; paired by (layout, rep); CIs are 95%% cluster bootstrap "
            "over layouts (2000 resamples). Failed missions are included in every rate." % (
                ",".join(sp), len(lay), lay[0] if lay else "-", lay[-1] if lay else "-", len(rows)), ""]


def report_E1(split):
    out = ["# E1: the unreported detection", ""]
    rn = load("E1nat", split)
    if rn:
        out += header("E1nat", rn)
        out += ["## E1-natural: full missions, person in a TRUE dead zone at the forecast-link boundary", "",
                "Deadline D = 720 s, tolerance E = 10 m. Times in s (quantiles over missions where the event happened).", ""]
        out += table_main(rn, E1_POLS, "pod")
        out += ["", "Failure causes:", ""] + causes(rn, E1_POLS)
        out += ["", "Glimpse to delivery (missions with a true detection): median first-glimpse, local-confirmation "
                "and delivery times, and the fraction of glimpsed missions delivered.", "",
                "| policy | n glimpsed | glimpse in outage | delivered/glimpsed | glimpse->conf p50 | glimpse->delivery p50 |",
                "|---|---|---|---|---|---|"]
        for p in E1_POLS:
            R = [r for r in rn if r["policy"] == p and r["t_first_det"] is not None]
            if not R:
                continue
            gc = [r["t_local_conf"] - r["t_first_det"] for r in R if r["t_local_conf"] is not None]
            gd = [r["t_delivery"] - r["t_first_det"] for r in R if r["t_delivery"] is not None]
            out.append("| %s | %d | %.2f | %.2f | %s | %s |" % (
                p, len(R), np.mean([bool(r["glimpse_in_outage"]) for r in R]), np.mean([r["success"] for r in R]),
                fmt(np.median(gc) if gc else None, 0), fmt(np.median(gd) if gd else None, 0)))
    rf = load("E1forced", split)
    if rf:
        out += ["", "## E1-forced: mission starts at the glimpse (drone 1 just inside the dead zone)", ""]
        out += header("E1forced", rf)
        out += ["Rows: deadline D (s) x detection value (low = conf 0.6 on a low-prior cell; high = conf 0.9 on a "
                "high-prior cell). Columns: P(primary success) per fixed evidence option (pod_deadzone with the "
                "first evidence decision forced), the VOI policy (argmax of its own estimate), pod (react) and B4+B5.",
                ""]
        opts = ["forced_" + o for o in P.EVIDENCE_OPTIONS] + ["pod_deadzone", "pod", "b45_mule_recheck"]
        out.append("| D, value | " + " | ".join(o.replace("forced_", "") for o in opts) + " | best fixed |")
        out.append("|---|" + "---|" * (len(opts) + 1))
        variants = sorted({r["variant"] for r in rf}, key=lambda v: (int(v.split("_")[0][1:]), v))
        best_by = {}
        for v in variants:
            R = [r for r in rf if r["variant"] == v]
            vals = {o: np.mean([r["success"] for r in R if r["policy"] == o]) if any(r["policy"] == o for r in R) else np.nan for o in opts}
            b = max(opts[:5], key=lambda o: vals[o])
            best_by[v] = b
            out.append("| %s | " % v + " | ".join(fmt(vals[o]) for o in opts) + " | %s |" % b.replace("forced_", ""))
        out += ["", "Paired differences (cluster bootstrap), best fixed option vs alternatives, per cell:", ""]
        out.append("| D, value | comparison | diff [95% CI] |")
        out.append("|---|---|---|")
        for v in variants:
            R = [r for r in rf if r["variant"] == v]
            b = best_by[v]
            for o in opts[:5] + ["pod_deadzone"]:
                if o == b:
                    continue
                out.append("| %s | %s - %s | %s |" % (v, b.replace("forced_", ""), o.replace("forced_", ""),
                                                      ci(S.cluster_boot_diff(R, b, o, "success")[:3])))
        out += ["", "Time distributions in the forced setting (p25/p50/p75 s; local confirmation, delivery):", "",
                "| D, value | option | t local conf | t delivery | success |", "|---|---|---|---|---|"]
        for v in variants:
            for o in opts:
                R = [r for r in rf if r["variant"] == v and r["policy"] == o]
                if R:
                    out.append("| %s | %s | %s | %s | %.2f |" % (v, o.replace("forced_", ""),
                                                               qstr([r["t_local_conf"] for r in R]),
                                                               qstr([r["t_delivery"] for r in R]),
                                                               np.mean([r["success"] for r in R])))
    return out


def report_E3(split):
    rows = load("E3", split)
    out = ["# E3: the duplicated search under asymmetric outage", ""] + header("E3", rows)
    out += ["Drone 1's uplink and its air-to-air transmit direction are forced dead for t in [60, 540) s; it still "
            "hears the base and drone 2 (asymmetric). Person harder to see (partial evidence).", ""]
    out += ["| policy | dup area m2 [95% CI] | diff vs lease | disagreement m2 (mean) | urgent det -> verify look p50 s | "
            "urgent det -> delivery p50 s | success | diff vs lease |", "|---|---|---|---|---|---|---|---|"]
    for p in E3_POLS:
        R = [r for r in rows if r["policy"] == p]
        if not R:
            continue
        dv = [r["t_verify_after_urgent"] - r["t_urgent"] for r in R if r["t_urgent"] is not None and r["t_verify_after_urgent"] is not None]
        dd = [r["t_delivery"] - r["t_urgent"] for r in R if r["t_urgent"] is not None and r["t_delivery"] is not None]
        nu = sum(r["t_urgent"] is not None for r in R)
        out.append("| %s | %s | %s | %.0f | %s (n=%d) | %s (n=%d of %d) | %s | %s |" % (
            p, ci(S.cluster_boot_mean(rows, p, "dup_area_m2")[:3], 0),
            ci(S.cluster_boot_diff(rows, p, "lease", "dup_area_m2")[:3], 0) if p != "lease" else "-",
            np.mean([r["disagree_m2_mean"] for r in R]), fmt(np.median(dv) if dv else None, 0), len(dv),
            fmt(np.median(dd) if dd else None, 0), len(dd), nu, ci(S.cluster_boot_mean(rows, p, "success")[:3]),
            ci(S.cluster_boot_diff(rows, p, "lease", "success")[:3]) if p != "lease" else "-"))
    out += ["", "Urgent-detection response (paired, missions where both policies had an urgent detection): "
            "time from urgent detection to base delivery, censored at the horizon (780 s).", "",
            "| comparison | diff s [95% CI] |", "|---|---|"]
    def resp(r):
        if r["t_urgent"] is None:
            return None
        return (r["t_delivery"] if r["t_delivery"] is not None else 780.0) - r["t_urgent"]
    for p in E3_POLS:
        if p != "lease":
            out.append("| %s - lease | %s |" % (p, ci(S.cluster_boot_diff(rows, p, "lease", resp)[:3], 0)))
    out += ["", "Failure causes:", ""] + causes(rows, E3_POLS)
    out += ["", "Full table:", ""] + table_main(rows, E3_POLS, "lease")
    return out


def report_ABL(split):
    rows = load("ABL", split)
    out = ["# Isolation ablations: radio loss vs localization drift vs stale map, then correlated failure", ""]
    out += header("ABL", rows)
    out += ["baseline = medium canopy; radio_only = foliage attenuation x1.6 (planners keep the nominal forecast); "
            "drift_only = odometry drift x5; stale_only = stale prior map (more unmapped blockages, phantom blockages, "
            "landmark offsets ~6.8 m, canopy error); correlated_dense = dense canopy degrading radio, drift and map "
            "together.", ""]
    vs = ["baseline", "radio_only", "drift_only", "stale_only", "correlated_dense"]
    out.append("| policy | " + " | ".join(vs) + " |")
    out.append("|---|" + "---|" * len(vs))
    for p in ABL_POLS:
        out.append("| %s | " % p + " | ".join(ci(S.cluster_boot_mean([r for r in rows if r["variant"] == v], p, "success")[:3]) for v in vs) + " |")
    out += ["", "Paired change vs baseline condition, same policy (success; cluster bootstrap over layouts):", ""]
    out.append("| policy | " + " | ".join(vs[1:]) + " |")
    out.append("|---|" + "---|" * (len(vs) - 1))
    for p in ABL_POLS:
        cells = []
        for v in vs[1:]:
            rr = [dict(r, policy=r["variant"]) for r in rows if r["policy"] == p and r["variant"] in (v, "baseline")]
            for r in rr:
                r["pair_key"] = ""
            cells.append(ci(S.cluster_boot_diff(rr, v, "baseline", "success")[:3]))
        out.append("| %s | " % p + " | ".join(cells) + " |")
    out += ["", "pod_deadzone minus comparator, per condition (success):", ""]
    out.append("| comparator | " + " | ".join(vs) + " |")
    out.append("|---|" + "---|" * len(vs))
    for p in ABL_POLS:
        if p == "pod_deadzone":
            continue
        out.append("| %s | " % p + " | ".join(ci(S.cluster_boot_diff([r for r in rows if r["variant"] == v], "pod_deadzone", p, "success")[:3]) for v in vs) + " |")
    for v in vs:
        out += ["", "### %s: failure causes" % v, ""] + causes(rows, ABL_POLS, v)
    return out


def report_P7(split):
    rows = load("P7", split)
    out = ["# P7: do find-then-mule (B4) + threshold re-check (B5) capture our planner's gain?", ""] + header("P7", rows)
    out += ["capture = (B45 - B1) / (ours - B1) on primary success, ours = pod_deadzone; CI by layout bootstrap of the "
            "ratio (undefined when ours - B1 CI includes 0).", "",
            "| condition | B1 | B4+B5 | ours | oracle | ours - B1 | ours - B45 | capture [95% CI] | false alerts/mission |",
            "|---|---|---|---|---|---|---|---|---|"]
    for v in ["fa_low", "fa_high", "align_high", "outage_corr"]:
        R = [r for r in rows if r["variant"] == v]
        if not R:
            continue
        m = {p: S.cluster_boot_mean(R, p, "success") for p in P7_POLS}
        d1 = S.cluster_boot_diff(R, "pod_deadzone", "b1_podgreedy", "success")
        d2 = S.cluster_boot_diff(R, "pod_deadzone", "b45_mule_recheck", "success")
        cap = capture_ci(R)
        out.append("| %s | %s | %s | %s | %s | %s | %s | %s | %.1f |" % (
            v, fmt(m["b1_podgreedy"][0]), fmt(m["b45_mule_recheck"][0]), fmt(m["pod_deadzone"][0]), fmt(m["oracle"][0]),
            ci(d1[:3]), ci(d2[:3]), ci(cap), np.mean([r["false_alerts"] for r in R if r["policy"] == "b1_podgreedy"])))
    out += ["", "Time to confirmed report at base (median s among delivered; failures excluded here only):", "",
            "| condition | " + " | ".join(P7_POLS) + " |", "|---|" + "---|" * len(P7_POLS)]
    for v in ["fa_low", "fa_high", "align_high", "outage_corr"]:
        R = [r for r in rows if r["variant"] == v]
        out.append("| %s | " % v + " | ".join(qstr([r["t_delivery"] for r in R if r["policy"] == p]) for p in P7_POLS) + " |")
    for v in ["fa_low", "fa_high", "align_high", "outage_corr"]:
        out += ["", "### %s failure causes" % v, ""] + causes([r for r in rows if r["variant"] == v], P7_POLS)
    return out


def capture_ci(R, n_boot=2000):
    per = defaultdict(lambda: defaultdict(list))
    for r in R:
        per[r["layout"]][r["policy"]].append(r["success"])
    L = [l for l in per if all(p in per[l] for p in ("b1_podgreedy", "b45_mule_recheck", "pod_deadzone"))]
    if not L:
        return (np.nan, np.nan, np.nan)
    A = np.array([[np.sum(per[l][p]) for p in ("b1_podgreedy", "b45_mule_recheck", "pod_deadzone")] for l in L], float)
    def ratio(a):
        s = a.sum(0)
        den = s[2] - s[0]
        return (s[1] - s[0]) / den if abs(den) > 1e-9 else np.nan
    est = ratio(A)
    r = np.random.default_rng(0)
    bs = [ratio(A[r.integers(0, len(L), len(L))]) for _ in range(n_boot)]
    bs = [b for b in bs if np.isfinite(b)]
    if not bs:
        return (est, np.nan, np.nan)
    return (est, float(np.percentile(bs, 2.5)), float(np.percentile(bs, 97.5)))


def cmd_report(a):
    os.makedirs(os.path.join(HERE, "results"), exist_ok=True)
    fn = {"E1": report_E1, "E3": report_E3, "ABL": report_ABL, "P7": report_P7}
    names = {"E1": "E1.md", "E3": "E3.md", "ABL": "ablations.md", "P7": "P7.md"}
    for e in a.exp.split(","):
        lines = fn[e](a.split)
        with open(os.path.join(HERE, "results", names[e]), "w") as f:
            f.write("\n".join(lines) + "\n")
        print("wrote", names[e])


def cmd_logs(a):
    d = os.path.join(HERE, "results", "logs")
    os.makedirs(d, exist_ok=True)
    seed = a.seed
    for v, sc in {"E1forced_D240_high": W.Scenario(density="medium", e1_forced=True, e1_conf=0.9, e1_prior="high",
                                                   deadline_s=240.0, horizon_s=260.0),
                  "E1forced_D480_low": W.Scenario(density="medium", e1_forced=True, e1_conf=0.6, e1_prior="low",
                                                  deadline_s=480.0, horizon_s=500.0),
                  "E1nat": W.Scenario(density="medium", e1=True)}.items():
        for pol in ("pod_deadzone", "pod", "b45_mule_recheck", "rendezvous"):
            p = os.path.join(d, "%s_seed%d_%s.jsonl" % (v, seed, pol))
            M = sim.run_mission(seed, 0, pol, sc, log_path=p)
            with open(p.replace(".jsonl", "_outcome.json"), "w") as f:
                json.dump({k: (float(x) if isinstance(x, (np.floating,)) else x) for k, x in M.items()}, f, indent=1,
                          default=float)
            print(v, pol, M["success"], M["failure_cause"], M["t_first_det"], M["t_local_conf"], M["t_delivery"])


def main():
    ap = argparse.ArgumentParser()
    sp = ap.add_subparsers(dest="cmd")
    r = sp.add_parser("run")
    r.add_argument("--exp", required=True)
    r.add_argument("--split", default="dev")
    r.add_argument("--layouts", type=int, default=8)
    r.add_argument("--reps", type=int, default=1)
    r.add_argument("--shard", type=int, default=0)
    r.add_argument("--nshards", type=int, default=1)
    r.add_argument("--allow-heldout", action="store_true")
    q = sp.add_parser("report")
    q.add_argument("--exp", required=True)
    q.add_argument("--split", default="dev")
    g = sp.add_parser("logs")
    g.add_argument("--seed", type=int, default=7)
    a = ap.parse_args()
    {"run": cmd_run, "report": cmd_report, "logs": cmd_logs}[a.cmd](a)


if __name__ == "__main__":
    main()
