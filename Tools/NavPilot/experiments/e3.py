# SPDX-License-Identifier: GPL-3.0-or-later
"""Experiment 3: maximum reliably achievable speed at controlled collision risk (protocol: PREREG_SPEED.md).

  PY=python; Q=~/.cache/navpilot_e3   (workers: experiments/worker.py under fsrun)
  $PY experiments/e3.py submit --plan smoke|tune|heldout [--allow-heldout] --queue $Q
  $PY experiments/e3.py tune-select                # -> results/raw/e3_tuned.json
  $PY experiments/e3.py report --plan heldout       # -> results/E3_heldout.md

Condition families (one layout yields all four; run seed = layout seed for every method/speed = paired):
  stale  S1 stale shortcut: prior shows a gap that two unmapped 2.5-4 cm branches block; heavy compute tail
  drift  S3 map-alignment drift 1.2-2.0 m before a wall with a narrow gap; heavy tail
  occl   S4 occlusion + local-estimator drift x10 + thin branch in the only gap; heavy tail
  stall  S0 geometry (correct prior) with the severe compute tail (p99 0.56 s, p99.9 2.5 s)
"""
import argparse
import glob
import json
import math
import os
import sys
import time

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.normpath(os.path.join(HERE, ".."))
sys.path.insert(0, ROOT)

import numpy as np  # noqa: E402

from experiments.e3_episode import default_cond, run as run_episode  # noqa: E402
from experiments.scenarios import build  # noqa: E402
from navpilot.forest import guard_seed  # noqa: E402
from stacks.advanced import METHODS  # noqa: E402

RAW = os.path.join(ROOT, "results", "raw")
RES = os.path.join(ROOT, "results")
TUNED = os.path.join(RAW, "e3_tuned.json")
E2_TUNED = os.path.join(RAW, "e2_tuned.json")
FAMS = {"stale": ("S1", "heavy"), "drift": ("S3", "heavy"), "occl": ("S4", "heavy"), "stall": ("S0", "severe")}
SPEEDS = [2.5, 4.0, 5.5, 7.0, 9.0]
TUNE_SPEEDS = [6.0, 9.0]
TUNE_LAYOUTS = [300, 301]
HELDOUT_LAYOUTS = list(range(90400, 90408))
ADV = ["opt_backup_t", "reactive_plus", "mpc_lc", "ua_map", "pas_opt", "pas_full"]
# held-out arms (compute-cut, see PREREG_SPEED.md): baseline + 3 new methods + the PAS variant with the higher DEV
# utility (pas_opt vs pas_full, recorded as "_pas_pick" in e3_tuned.json by tune-select)
HELD_BASE = ["opt_backup_t", "reactive_plus", "mpc_lc", "ua_map"]


def held_methods():
    try:
        return HELD_BASE + [json.load(open(TUNED))["_pas_pick"]]
    except (OSError, KeyError):
        return HELD_BASE + ["pas_full"]


HELD_METHODS = held_methods()
P_STAR = 0.05
LAMBDA = 40.0   # tuning penalty (m/s per unit collision rate): 5 % collisions cost 2 m/s of progress
N_CFG = 5


def cond_for(fam, v):
    return default_cond(v_lim=float(v), latency_tail=FAMS[fam][1])


def configs(method, n=N_CFG, seed=2027):
    """Equal budget per method: defaults + (n-1) seeded random draws over the method's own knobs + common knobs."""
    r = np.random.default_rng(seed + ADV.index(method))
    space = {"margin": [0.3, 0.5, 0.7], "inflate_extra": [0.1, 0.15, 0.25], "w_free": [0.4, 0.6, 1.0]}
    if method == "reactive_plus":
        space.update({"react_mem": [0.7, 1.5, 3.0], "H": [8.0, 10.0, 12.0]})
    if method in ("mpc_lc", "pas_full"):
        space.update({"tighten_k0": [0.0, 0.05, 0.1], "tighten_ks": [0.0, 0.01, 0.02]})
    if method in ("ua_map", "pas_full"):
        space.update({"ua_z": [1.0, 1.5, 2.5], "drift_tol": [0.1, 0.15, 0.3]})
    if method in ("pas_opt", "pas_full"):
        space.update({"R_conf": [3.0, 4.0, 5.5], "d_struct": [1.0, 1.5, 2.5], "look": [0.0, 1.0]})
    out = [dict()]
    while len(out) < n:
        out.append({k: (float(r.choice(v)) if not isinstance(v[0], str) else str(r.choice(v))) for k, v in space.items()})
    return out


def plan_jobs(plan, allow_heldout=False):
    """[(layout, fam, method, params, cname, v)]"""
    J = []
    if plan == "parity":
        return [(302, "stale", "reactive", "reactive"), (302, "stale", "hybrid_conf", "hybrid_conf"),
                (302, "stale", "opt_backup", "opt_backup_t")]
    if plan == "smoke":
        for m in ADV:
            J.append((300, "stale", m, {}, "smoke", 7.5))
        J.append((300, "occl", "pas_full", {}, "smoke", 7.5))
        return J
    if plan == "tune":
        for L in TUNE_LAYOUTS:
            guard_seed(L, allow_heldout=False)
        for m in ADV:
            for k, p in enumerate(configs(m)):
                for L in TUNE_LAYOUTS:
                    for fam in FAMS:
                        for v in TUNE_SPEEDS:
                            J.append((L, fam, m, p, "tune_%d" % k, v))
        return J
    if plan == "heldout":
        if not allow_heldout:
            raise SystemExit("held-out plan refused without --allow-heldout")
        for L in HELDOUT_LAYOUTS:
            guard_seed(L, allow_heldout=True)
        tuned = json.load(open(TUNED))
        e2 = json.load(open(E2_TUNED))
        for L in HELDOUT_LAYOUTS:
            for fam in FAMS:
                for v in SPEEDS:
                    for m in HELD_METHODS:
                        p = e2["conf"] if m == "hybrid_conf" else tuned.get(m, {}) if m in ADV else {}
                        J.append((L, fam, m, p, "speed", v))
        return J
    raise ValueError(plan)


def submit(args):
    jobs = plan_jobs(args.plan, args.allow_heldout)
    q = os.path.expanduser(args.queue)
    tag = time.strftime("%H%M%S")
    # interleave so both workers progress through layouts evenly; long (slow) jobs spread across files
    n = max(1, int(math.ceil(len(jobs) / args.chunk)))
    for i in range(n):
        part = jobs[i * args.chunk:(i + 1) * args.chunk] if args.plan == "heldout" else jobs[i::n]
        out = os.path.join(RAW, "e3_%s_%s_%03d.jsonl" % (args.plan, tag, i))
        spec = {"plan": args.plan, "allow_heldout": bool(args.allow_heldout), "out": out, "jobs": part}
        tmp = os.path.join(q, "e3_%s_%s_%03d.tmp" % (args.plan, tag, i))
        json.dump(spec, open(tmp, "w"))
        os.rename(tmp, os.path.join(q, "pending", "%s_%s_%03d.json" % (args.prio, args.plan, i)))
    print("submitted %d jobs in %d files" % (len(jobs), n))


def parity(spec):
    """E2 vs E3 episode on the same (layout, family): E3 must reproduce E2 exactly for E2 stacks and for
    opt_backup_t at defaults vs E2 OPT_BACKUP."""
    from experiments.e2_episode import run as run_e2
    out = []
    for L, fam, m2, m3 in spec["jobs"]:
        scn = build(L, FAMS[fam][0])
        a = run_e2(scn, m2, cond_for(fam, 7.5), L, {})
        scn = build(L, FAMS[fam][0])
        b = run_episode(scn, m3, cond_for(fam, 7.5), L, {})
        keys = ["outcome", "time_s", "progress_m", "min_clear", "path_len", "cycles", "watchdog"]
        out.append({"layout": L, "fam": fam, "e2": m2, "e3": m3, "identical": all(a[k] == b[k] for k in keys),
                    "e2v": [a[k] for k in keys], "e3v": [b[k] for k in keys]})
    json.dump(out, open(spec["out"], "w"), indent=1)
    print(json.dumps(out))


def run_file(path):
    spec = json.load(open(path))
    if spec.get("plan") == "parity":
        return parity(spec)
    cache = {}
    t0 = time.time()
    with open(spec["out"], "a") as f:
        for L, fam, m, p, cname, v in spec["jobs"]:
            sfam = FAMS[fam][0]
            key = (L, sfam)
            if key not in cache:
                cache = {key: build(L, sfam, allow_heldout=spec["allow_heldout"])}
            te = time.time()
            r = run_episode(cache[key], m, cond_for(fam, v), L, p)
            r.update({"layout": L, "fam": fam, "family": sfam, "stack": m, "cond": cname, "params": p, "v_lim": v,
                      "wall_s": time.time() - te})
            f.write(json.dumps(r) + "\n")
            f.flush()
    print("%s: %d episodes in %.0fs" % (path, len(spec["jobs"]), time.time() - t0))


def load(plan):
    rows = []
    for p in sorted(glob.glob(os.path.join(RAW, "e3_%s_*.jsonl" % plan))):
        rows += [json.loads(l) for l in open(p) if l.strip()]
    return rows


def utility(rs):
    return float(np.mean([r["progress_rate"] - LAMBDA * float(r["collision"]) for r in rs])) if rs else -1e9


def tune_select(args):
    rows = load("tune")
    out, log = {}, {}
    for m in ADV:
        cf = configs(m)
        sc = []
        for k, p in enumerate(cf):
            rs = [r for r in rows if r["stack"] == m and r["cond"] == "tune_%d" % k]
            sc.append((utility(rs) if len(rs) == len(TUNE_LAYOUTS) * len(FAMS) * len(TUNE_SPEEDS) else -1e9, k, len(rs),
                       float(np.mean([r["collision"] for r in rs])) if rs else float("nan"),
                       float(np.mean([r["progress_rate"] for r in rs])) if rs else float("nan")))
        best = max(sc)
        out[m] = cf[best[1]]
        log[m] = sc
    out["_scores"] = log
    u = {m: max(log[m])[0] for m in ("pas_opt", "pas_full")}
    out["_pas_pick"] = max(u, key=lambda k: (u[k], k == "pas_full"))
    json.dump(out, open(TUNED, "w"), indent=1)
    for m in ADV:
        print(m, out[m], ["k%d u=%.2f n=%d coll=%.2f prog=%.2f" % (s[1], s[0], s[2], s[3], s[4]) for s in log[m]])


# --------------------------------------------------------------------------- protocol (PREREG_SPEED.md)
def pava(y, w):
    """Weighted isotonic (non-decreasing) regression."""
    blocks = [[float(yi), float(wi), 1] for yi, wi in zip(y, w)]
    i = 0
    while i < len(blocks) - 1:
        if blocks[i][0] > blocks[i + 1][0] + 1e-12:
            a, b = blocks[i], blocks[i + 1]
            wt = a[1] + b[1]
            blocks[i] = [(a[0] * a[1] + b[0] * b[1]) / max(wt, 1e-12), wt, a[2] + b[2]]
            del blocks[i + 1]
            i = max(i - 1, 0)
        else:
            i += 1
    out = []
    for v, _, c in blocks:
        out += [v] * c
    return np.array(out)


def v_star(speeds, coll, n, p_star=P_STAR):
    """Speed at which isotonic P(collision) crosses p_star (linear interpolation on the grid).
    Returns (v*, censor) with censor -1 = below the grid (v* reported as the lowest speed), +1 = above."""
    p = pava(np.asarray(coll, float) / np.maximum(n, 1), n)
    if p[0] > p_star:
        return float(speeds[0]), -1
    for k in range(1, len(speeds)):
        if p[k] > p_star:
            f = (p_star - p[k - 1]) / max(p[k] - p[k - 1], 1e-12)
            return float(speeds[k - 1] + f * (speeds[k] - speeds[k - 1])), 0
    return float(speeds[-1]), 1


def achieved_at(speeds, ach, vs):
    return float(np.interp(vs, speeds, ach))


_AGG = {}


def _agg(rows):
    """Per (method, fam, layout, v): [collisions, n, sum progress (non-collision), n non-collision, sum progress]."""
    key = id(rows)
    if key not in _AGG:
        d = {}
        for r in rows:
            a = d.setdefault((r["stack"], r["fam"], r["layout"], r["v_lim"]), [0, 0, 0.0, 0, 0.0])
            a[0] += int(r["collision"])
            a[1] += 1
            a[4] += r["progress_rate"]
            if not r["collision"]:
                a[2] += r["progress_rate"]
                a[3] += 1
        _AGG.clear()
        _AGG[key] = d
    return _AGG[key]


def curve_stats(rows, method, layouts, fams, speeds=None):
    """Per-speed collision counts / n and achieved progress rate (mean progress_rate over non-collision episodes;
    carried forward from the previous setpoint if none) for rows restricted to layouts (with multiplicity, for the
    bootstrap) and fams."""
    speeds = SPEEDS if speeds is None else speeds
    d = _agg(rows)
    coll, n, ach_s, prog_all = [], [], [], []
    z = [0, 0, 0.0, 0, 0.0]
    for v in speeds:
        t = [0, 0, 0.0, 0, 0.0]
        for L in layouts:
            for f in fams:
                a = d.get((method, f, L, v), z)
                for i in range(5):
                    t[i] += a[i]
        coll.append(t[0])
        n.append(t[1])
        ach_s.append(t[2] / t[3] if t[3] else (ach_s[-1] if ach_s else 0.0))
        prog_all.append(t[4] / t[1] if t[1] else 0.0)
    return np.array(coll), np.array(n), np.array(ach_s), np.array(prog_all)


def endpoint(rows, method, layouts, fams):
    c, n, a, pa = curve_stats(rows, method, layouts, fams)
    vs, cen = v_star(SPEEDS, c, n)
    return {"v_star": vs, "censor": cen, "A_star": achieved_at(SPEEDS, a, vs), "coll": c, "n": n, "ach": a, "prog_all": pa}


def report(args):
    rows, seen = [], set()
    for r in load(args.plan):
        k = (r["layout"], r["fam"], r["stack"], r["v_lim"])
        if r["cond"] == "speed" and k not in seen:
            seen.add(k)
            rows.append(r)
    methods = [m for m in HELD_METHODS if any(r["stack"] == m for r in rows)]
    lay_all = sorted(set(r["layout"] for r in rows))
    # completeness guard: only layouts with every planned (method, fam, speed) present
    need = set((f, m, v) for f in FAMS for m in HELD_METHODS for v in SPEEDS)
    have = {}
    for r in rows:
        have.setdefault(r["layout"], set()).add((r["fam"], r["stack"], r["v_lim"]))
    lay = [L for L in lay_all if have[L] == need]
    rng = np.random.default_rng(0)
    B = 2000
    idx = rng.integers(0, len(lay), (B, len(lay))) if lay else np.zeros((B, 0), int)
    fam_sets = [("ALL", list(FAMS))] + [(f, [f]) for f in FAMS]
    res = {}
    for fname, fs in fam_sets:
        for m in methods:
            pt = endpoint(rows, m, lay, fs)
            bs_v, bs_a = [], []
            for b in range(B):
                ls = [lay[i] for i in idx[b]]
                e = endpoint(rows, m, ls, fs)
                bs_v.append(e["v_star"])
                bs_a.append(e["A_star"])
            res[(fname, m)] = (pt, np.array(bs_v), np.array(bs_a))
    base = "opt_backup_t"
    L = ["# E3 held-out: maximum reliably achievable speed at P(collision) <= 5 %", "",
         "_Generated by `experiments/e3.py report --plan %s`. Protocol: `PREREG_SPEED.md` (frozen before this run). "
         "Evidence label: IMPLEMENTED (2.5D point mass, synthetic sensors/forest/scenarios); not LIVE_SIM_VALIDATED, "
         "not PHYSICALLY_CALIBRATED._" % args.plan, "",
         "Complete layouts used: %d of %d seen (%s). Episodes: %d. Bootstrap: %d cluster resamples over layouts, seed 0." % (
             len(lay), len(lay_all), ", ".join(map(str, lay)), len(rows), B), "",
         "v* = setpoint speed where the isotonic (PAVA) collision probability crosses 5 %% (grid %s m/s, linear "
         "interpolation; `<` = above 5 %% already at the lowest setpoint, `>=` = never crosses). A* = achieved course "
         "progress rate (m/s, non-collision episodes) at v*. Diff = method - %s, paired bootstrap; **bold** = 95 %% CI "
         "excludes 0." % (SPEEDS, base), ""]
    for fname, fs in fam_sets:
        L.append("## %s" % ("Pooled over all four families (primary)" if fname == "ALL" else "Family: " + fname))
        L.append("")
        L.append("| method | v* (95% CI) | A* m/s (95% CI) | dA* vs baseline (95% CI) | dv* vs baseline (95% CI) | collisions by setpoint | achieved by setpoint |")
        L.append("|---|---|---|---|---|---|---|")
        bpt, bbv, bba = res[(fname, base)] if (fname, base) in res else (None, None, None)
        for m in methods:
            pt, bv, ba = res[(fname, m)]
            cs = {-1: "<", 0: "", 1: ">="}[pt["censor"]]
            s = "| %s | %s%.2f (%.2f-%.2f) | %.2f (%.2f-%.2f) |" % (m, cs, pt["v_star"], *np.quantile(bv, [0.025, 0.975]),
                                                                  pt["A_star"], *np.quantile(ba, [0.025, 0.975]))
            if bpt is not None and m != base:
                da, dv = ba - bba, bv - bbv
                qa, qv = np.quantile(da, [0.025, 0.975]), np.quantile(dv, [0.025, 0.975])
                fa = "**%+.2f (%+.2f, %+.2f)**" if (qa[0] > 0 or qa[1] < 0) else "%+.2f (%+.2f, %+.2f)"
                fv = "**%+.2f (%+.2f, %+.2f)**" if (qv[0] > 0 or qv[1] < 0) else "%+.2f (%+.2f, %+.2f)"
                s += " " + fa % (pt["A_star"] - bpt["A_star"], *qa) + " | " + fv % (pt["v_star"] - bpt["v_star"], *qv) + " |"
            else:
                s += " - | - |"
            s += " %s |" % " ".join("%d/%d" % (c, n) for c, n in zip(pt["coll"], pt["n"]))
            s += " %s |" % " ".join("%.1f" % a for a in pt["ach"])
            L.append(s)
        L.append("")
    # mechanism: first-detection range of scenario-block branches and contact kinds
    L.append("## Mechanism: thin-branch first-detection range (m, scenario-block branches; stale + occl) and contacts")
    L.append("")
    L.append("| method | v | det. range median | never detected | collisions | branch contacts | mean contact v |")
    L.append("|---|---|---|---|---|---|---|")
    for m in methods:
        for v in SPEEDS:
            rs = [r for r in rows if r["stack"] == m and r["v_lim"] == v and r["fam"] in ("stale", "occl") and r["layout"] in lay]
            dr = [d for r in rs for d in r.get("blk_det_range", []) if d is not None]
            nd = sum(1 for r in rs for d in r.get("blk_det_range", []) if d is None)
            cl = [r for r in rs if r["collision"]]
            L.append("| %s | %.1f | %s | %d | %d/%d | %d | %s |" % (
                m, v, "%.2f" % np.median(dr) if dr else "-", nd, len(cl), len(rs),
                sum(1 for r in cl if r["contact_kind"] == "branch"),
                "%.1f" % np.mean([r["contact_speed"] for r in cl]) if cl else "-"))
    L.append("")
    out = os.path.join(RES, "E3_%s.md" % args.plan)
    open(out, "w").write("\n".join(L) + "\n")
    json.dump({"%s|%s" % k: {"v_star": v[0]["v_star"], "censor": v[0]["censor"], "A_star": v[0]["A_star"],
                             "v_ci": list(np.quantile(v[1], [0.025, 0.975])), "A_ci": list(np.quantile(v[2], [0.025, 0.975]))}
               for k, v in res.items()}, open(os.path.join(RAW, "e3_%s_summary.json" % args.plan), "w"), indent=1)
    print("wrote", out)


def dev_curve(args):
    """Quick DEV look at the tuning rows: collision and progress by method/config/speed/family."""
    rows = load("tune") + load("smoke")
    for m in ADV:
        for k in range(N_CFG):
            rs = [r for r in rows if r["stack"] == m and r["cond"] == "tune_%d" % k]
            if not rs:
                continue
            parts = []
            for f in FAMS:
                for v in TUNE_SPEEDS:
                    q = [r for r in rs if r["fam"] == f and r["v_lim"] == v]
                    parts.append("%s@%g %d/%d %.1f" % (f, v, sum(r["collision"] for r in q), len(q),
                                                       np.mean([r["progress_rate"] for r in q]) if q else 0))
            print(m, k, "u=%.2f" % utility(rs), " | ".join(parts))


if __name__ == "__main__":
    ap = argparse.ArgumentParser()
    sp = ap.add_subparsers(dest="cmd", required=True)
    a = sp.add_parser("submit")
    a.add_argument("--plan", required=True)
    a.add_argument("--allow-heldout", action="store_true")
    a.add_argument("--queue", default="~/.cache/navpilot_e3")
    a.add_argument("--chunk", type=int, default=40)
    a.add_argument("--prio", default="5")
    a = sp.add_parser("run-file")
    a.add_argument("path")
    sp.add_parser("tune-select")
    sp.add_parser("dev-curve")
    a = sp.add_parser("report")
    a.add_argument("--plan", default="heldout")
    args = ap.parse_args()
    if args.cmd == "submit":
        submit(args)
    elif args.cmd == "run-file":
        run_file(args.path)
    elif args.cmd == "tune-select":
        tune_select(args)
    elif args.cmd == "dev-curve":
        dev_curve(args)
    else:
        report(args)
