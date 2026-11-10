# SPDX-License-Identifier: GPL-3.0-or-later
"""Experiment 2 ("the stale shortcut") runner.

  F=fsrun; PY=python
  for i in 0 1 2; do $F $PY Tools/NavPilot/experiments/e2.py run --plan tune --shard $i --nshards 3 & done; wait
  $F $PY Tools/NavPilot/experiments/e2.py tune-select           # -> results/raw/e2_tuned.json
  for i in 0 1 2; do $F $PY Tools/NavPilot/experiments/e2.py run --plan dev --shard $i --nshards 3 & done; wait
  $F $PY Tools/NavPilot/experiments/e2.py report --plan dev     # -> results/E2_dev.md
  # held-out: only after results/E2_HELDOUT_SPEC.md is frozen; run once
  ... run --plan heldout --allow-heldout ...; report --plan heldout  # -> results/E2_heldout.md

Plans (layout seeds are whole layouts; every stack/condition sees the same layout and run seed = paired):
  tune     DEV layouts 100-101, S1-S4, v 7, heavy tail, detector on: 8 HYBRID_CONF configs (defaults + seeded random search)
           and 8 HYBRID_FIXED configs (grid: signal x theta x v_fb). Equal budgets (8 configs x 8 episodes each).
  dev      DEV layouts 200-203: same arms as heldout (not run in this pass: compute-limited; tuning rows are the DEV check).
  heldout  HELD-OUT layouts 90200-90207: the frozen spec (results/E2_HELDOUT_SPEC.md).
"""
import argparse
import glob
import itertools
import json
import math
import os
import sys
import time

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.normpath(os.path.join(HERE, ".."))
sys.path.insert(0, ROOT)

import numpy as np  # noqa: E402

from experiments.e2_episode import default_cond, run as run_episode  # noqa: E402
from experiments.scenarios import build  # noqa: E402
from navpilot.forest import guard_seed  # noqa: E402
from stacks.policies import HYBRID_CONF, HYBRID_FIXED, MAP_RHP, MAP_RHP_N1, OPT_BACKUP, REACTIVE  # noqa: E402

RAW = os.path.join(ROOT, "results", "raw")
RES = os.path.join(ROOT, "results")
TUNED = os.path.join(RAW, "e2_tuned.json")


def conf_configs(n=8, seed=2026):
    r = np.random.default_rng(seed)
    out = [dict()]  # defaults first
    while len(out) < n:
        out.append({"s_align": float(r.choice([0.6, 1.0, 1.5])), "a0": float(r.choice([0.2, 0.3, 0.4])),
                    "a1": float(r.choice([0.55, 0.7, 0.85])), "o0": float(r.choice([0.3, 0.5, 0.7])),
                    "k_loc": float(r.choice([0.08, 0.15, 0.3])), "H_min": float(r.choice([6.0, 8.0, 10.0])),
                    "v_floor": float(r.choice([0.2, 0.35, 0.5])), "mem_min": float(r.choice([0.3, 1.0, 3.0])),
                    "m_loc": float(r.choice([0.2, 0.4, 0.8]))})
    return out


def fixed_configs():
    return [{"signal": s, "theta": th, "v_fb": vf} for s, th, vf in
            itertools.product(["conf", "agree"], [0.35, 0.6], [0.5, 0.8])]


N_HELDOUT = 8


def args_n_heldout():
    return N_HELDOUT


def _seeds(base, n, allow):
    s = [base + i for i in range(n)]
    for x in s:
        guard_seed(x, allow_heldout=allow)
    return s


def plan_jobs(plan, allow_heldout=False):
    """[(layout, family, stack, params, cond_name, cond)]"""
    J = []
    if plan == "tune":
        for L in _seeds(100, 2, False):
            for fam in ["S1", "S2", "S3", "S4"]:
                for k, p in enumerate(conf_configs()):
                    J.append((L, fam, HYBRID_CONF, p, "tune_c%d" % k, default_cond()))
                for k, p in enumerate(fixed_configs()):
                    J.append((L, fam, HYBRID_FIXED, p, "tune_f%d" % k, default_cond()))
        return J
    tuned = json.load(open(TUNED))
    if plan == "dev":
        layouts = _seeds(200, 4, False)
    elif plan == "heldout":
        if not allow_heldout:
            raise SystemExit("held-out plan refused without --allow-heldout")
        layouts = _seeds(90200, args_n_heldout(), True)
    else:
        raise ValueError(plan)

    def stack_params(st, fam):
        if st == HYBRID_CONF:
            return HYBRID_CONF, tuned["conf"]
        if st == HYBRID_FIXED:
            return HYBRID_FIXED, tuned["fixed"]
        if st == "hybrid_fixed_env":
            return HYBRID_FIXED, tuned["fixed_env"].get(fam, tuned["fixed"])
        return st, {}

    main = [REACTIVE, MAP_RHP, OPT_BACKUP, HYBRID_CONF, HYBRID_FIXED, "hybrid_fixed_env"]
    for L in layouts:
        # A. main comparison: S0 control + S1-S4 at v 7, heavy tail, detector on (+ N1-safety reference on S1, S3)
        for fam in ["S0", "S1", "S2", "S3", "S4"]:
            for st in main + ([MAP_RHP_N1] if fam in ("S1", "S3") else []):
                s2, p = stack_params(st, fam)
                J.append((L, fam, st, p, "main_v7", default_cond()))
        # B. speed sweep (P8 / T3): S0, S1, S3 at 3 m/s
        for fam in ["S0", "S1", "S3"]:
            for st in [MAP_RHP, OPT_BACKUP, HYBRID_CONF, "hybrid_fixed_env"]:
                s2, p = stack_params(st, fam)
                J.append((L, fam, st, p, "speed_v3", default_cond(v_lim=3.0)))
        # C. detector load and network loss (S1, S3)
        for fam in ["S1", "S3"]:
            s2, p = stack_params(MAP_RHP, fam)
            for cname, cond in (("det_isolated", default_cond(detector="isolated")),
                                ("det_heavy", default_cond(detector="heavy")), ("netloss", default_cond(netloss=True))):
                J.append((L, fam, MAP_RHP, p, cname, cond))
            s2, p = stack_params(HYBRID_CONF, fam)
            J.append((L, fam, HYBRID_CONF, p, "det_heavy", default_cond(detector="heavy")))
        # D. mission ablation on HYBRID_CONF (S0, S1, S3): uniform 4 m/s and detector-aware schedule vs fast (A)
        for fam in ["S0", "S1", "S3"]:
            s2, p = stack_params(HYBRID_CONF, fam)
            J.append((L, fam, HYBRID_CONF, p, "abl_uniform4", default_cond(v_lim=4.0)))
            J.append((L, fam, HYBRID_CONF, p, "abl_detaware", default_cond(det_beta=0.5)))
    return J


def _real_stack(st):
    return HYBRID_FIXED if st == "hybrid_fixed_env" else st


def run(args):
    os.makedirs(RAW, exist_ok=True)
    jobs = plan_jobs(args.plan, args.allow_heldout)
    if args.shards:   # several shard partitions in one process, in plan (layout) order; same jobs, same results
        keep = set(int(x) for x in args.shards.split(","))
        jobs = [j for i, j in enumerate(jobs) if i % args.nshards in keep]
        out = os.path.join(RAW, "e2_%s_shard%s.jsonl" % (args.plan, "".join(sorted(args.shards.split(",")))))
    else:
        jobs = jobs[args.shard::args.nshards]
        out = os.path.join(RAW, "e2_%s_shard%d.jsonl" % (args.plan, args.shard))
    done = set()
    if os.path.exists(out) and args.resume:
        for l in open(out):
            r = json.loads(l)
            done.add((r["layout"], r["family"], r["stack"], r["cond"]))
    cache = {}
    t0 = time.time()
    n = 0
    with open(out, "a" if args.resume else "w") as f:
        for L, fam, st, p, cname, cond in jobs:
            if (L, fam, st, cname) in done:
                continue
            key = (L, fam)
            if key not in cache:
                cache = {key: build(L, fam, allow_heldout=args.allow_heldout)}
            te = time.time()
            r = run_episode(cache[key], _real_stack(st), cond, L, p)
            r.update({"layout": L, "family": fam, "stack": st, "cond": cname, "params": p,
                      "v_lim": cond["v_lim"], "wall_s": time.time() - te})
            f.write(json.dumps(r) + "\n")
            f.flush()
            n += 1
    print("plan %s shard %d: %d episodes in %.0fs" % (args.plan, args.shard, n, time.time() - t0))


def load(plan):
    rows = []
    for p in sorted(glob.glob(os.path.join(RAW, "e2_%s_shard*.jsonl" % plan))):
        rows += [json.loads(l) for l in open(p) if l.strip()]
    return rows


# --------------------------------------------------------------------------- tuning objective
def objective(rs):
    """DEV tuning objective (same for both hybrids): success - 2 x collision - 0.02 x mean contact speed... per
    episode u = 1[success] - 2 * 1[collision] - 0.05 * time_s / 10, averaged."""
    u = [(1.0 if r["success"] else 0.0) - 2.0 * (1.0 if r["collision"] else 0.0) - 0.005 * r["time_s"] for r in rs]
    return float(np.mean(u))


def tune_select(args):
    rows = load("tune")
    out = {}
    for st, cfgs, pref in ((HYBRID_CONF, conf_configs(), "tune_c"), (HYBRID_FIXED, fixed_configs(), "tune_f")):
        scores = []
        for k, p in enumerate(cfgs):
            rs = [r for r in rows if r["cond"] == pref + str(k)]
            scores.append((objective(rs) if rs else -9, k, len(rs)))
        best = max(scores)
        out["conf" if st == HYBRID_CONF else "fixed"] = cfgs[best[1]]
        out[("conf" if st == HYBRID_CONF else "fixed") + "_scores"] = scores
        if st == HYBRID_FIXED:
            env = {}
            for fam in ["S1", "S2", "S3", "S4"]:
                sc = [(objective([r for r in rows if r["cond"] == pref + str(k) and r["family"] == fam]), k)
                      for k in range(len(cfgs))]
                env[fam] = cfgs[max(sc)[1]]
            # S0 has no tuning data (it is a control): use the global tuned switch
            out["fixed_env"] = env
    json.dump(out, open(TUNED, "w"), indent=1)
    print(json.dumps({k: v for k, v in out.items() if not k.endswith("scores")}, indent=1))
    for k in ("conf_scores", "fixed_scores"):
        print(k, sorted(out[k], reverse=True)[:5])


# --------------------------------------------------------------------------- report
def cboot(rows_a, rows_b, metric, n_boot=2000, seed=0):
    """Paired difference mean(a) - mean(b) over layouts with cluster bootstrap over layouts. Rows paired by
    (layout, family). Returns (diff, lo, hi, n_layouts)."""
    A = {(r["layout"], r["family"]): metric(r) for r in rows_a}
    B = {(r["layout"], r["family"]): metric(r) for r in rows_b}
    keys = [k for k in A if k in B and A[k] == A[k] and B[k] == B[k]]
    if not keys:
        return float("nan"), float("nan"), float("nan"), 0
    lay = sorted(set(k[0] for k in keys))
    per = {l: [A[k] - B[k] for k in keys if k[0] == l] for l in lay}
    sums = np.array([sum(per[l]) for l in lay])
    cnts = np.array([len(per[l]) for l in lay])
    d = sums.sum() / cnts.sum()
    r = np.random.default_rng(seed)
    idx = r.integers(0, len(lay), (n_boot, len(lay)))
    bs = sums[idx].sum(axis=1) / np.maximum(cnts[idx].sum(axis=1), 1)
    return float(d), float(np.quantile(bs, 0.025)), float(np.quantile(bs, 0.975)), len(lay)


def _m(rs, f):
    v = [f(r) for r in rs]
    v = [x for x in v if x is not None and x == x]
    return float(np.mean(v)) if v else float("nan")


COLS = [("n", None), ("succ", lambda r: r["success"]), ("coll", lambda r: r["collision"]),
        ("stuck", lambda r: r["stuck"]), ("contact v", lambda r: r["contact_speed"]),
        ("near-miss/ep", lambda r: r["near_misses"]), ("min clear", lambda r: r["min_clear"]),
        ("age p50", lambda r: r["age_p50"] * 1e3), ("age p95", lambda r: r["age_p95"] * 1e3),
        ("age p99", lambda r: r["age_p99"] * 1e3), ("prog m/s", lambda r: r["progress_rate"]),
        ("wasted m", lambda r: r["wasted_m"]), ("fallbacks", lambda r: r["fallback_activations"]),
        ("watchdog", lambda r: r["watchdog"]), ("t recover s", lambda r: r["t_backtrack"]),
        ("t stopped s", lambda r: r["t_stopped"]), ("stops", lambda r: r["stops"]),
        ("recall", lambda r: r["recall"]), ("rep@20s", lambda r: r["reported_20"]),
        ("rep@30s", lambda r: r["reported_30"])]


def table(L, groups, keyname):
    L.append("| %s | " % keyname + " | ".join(c for c, _ in COLS) + " |")
    L.append("|" + "---|" * (len(COLS) + 1))
    for k, rs in groups:
        cells = []
        for c, f in COLS:
            if f is None:
                cells.append(str(len(rs)))
            else:
                v = _m(rs, f)
                cells.append("%.0f%%" % (100 * v) if c in ("succ", "coll", "stuck") else
                             ("%.0f" % v if c.startswith("age") else "%.2f" % v))
        L.append("| %s | " % k + " | ".join(cells) + " |")


def report(args):
    rows = load(args.plan)
    if not rows:
        sys.exit("no rows for plan %s" % args.plan)
    per = {}
    for r in rows:
        per.setdefault(r["layout"], set()).add((r["family"], r["stack"], r["cond"]))
    n_full = max(len(v) for v in per.values())
    complete = sorted(l for l, v in per.items() if len(v) == n_full)
    n_all = len(rows)
    rows = [r for r in rows if r["layout"] in complete]
    L = []
    w = L.append
    label = "HELD-OUT (run once under results/E2_HELDOUT_SPEC.md)" if args.plan == "heldout" else \
        "DEV (iteration only; not reportable per splits.json)"
    w("# E2 the stale shortcut: stack comparison, %s\n" % label)
    w("_Generated by `Tools/NavPilot/experiments/e2.py report --plan %s`. Evidence label: IMPLEMENTED (2.5D point mass, "
      "synthetic sensors/forest/scenarios; not LIVE_SIM_VALIDATED, not PHYSICALLY_CALIBRATED). %d episodes, %d layouts._\n"
      % (args.plan, len(rows), len(set(r["layout"] for r in rows))))
    w("Complete layouts used (every planned job present): %s; %d of %d rows on disk used.\n" % (complete, len(rows), n_all))
    stacks = [REACTIVE, MAP_RHP, OPT_BACKUP, HYBRID_CONF, HYBRID_FIXED, "hybrid_fixed_env", MAP_RHP_N1]
    by = lambda c, fam=None, st=None: [r for r in rows if r["cond"] == c and (fam is None or r["family"] == fam)
                                      and (st is None or r["stack"] == st)]
    w("## 1. Main comparison (v_lim 7 m/s, heavy compute tail, detector on), by family\n")
    for fam in ["S0", "S1", "S2", "S3", "S4"]:
        w("### %s\n" % fam)
        table(L, [(st, by("main_v7", fam, st)) for st in stacks if by("main_v7", fam, st)], "stack")
        w("")
    w("### S1-S4 pooled\n")
    table(L, [(st, [r for r in by("main_v7", None, st) if r["family"] != "S0"]) for st in stacks], "stack")
    w("")
    w("## 2. Paired differences vs HYBRID_CONF (cluster bootstrap over layouts, 95% CI; positive = other stack higher)\n")
    w("| family | other stack | d success | d collision | d contact-v x coll | d time s | n layouts |")
    w("|---|---|---|---|---|---|---|")
    for fam in ["S0", "S1", "S2", "S3", "S4", "S1-S4"]:
        fs = (lambda r: r["family"] != "S0") if fam == "S1-S4" else (lambda r, f=fam: r["family"] == f)
        hc = [r for r in by("main_v7", None, HYBRID_CONF) if fs(r)]
        for st in stacks:
            if st == HYBRID_CONF:
                continue
            o = [r for r in by("main_v7", None, st) if fs(r)]
            cells = []
            for met in (lambda r: float(r["success"]), lambda r: float(r["collision"]),
                        lambda r: (r["contact_speed"] or 0.0), lambda r: r["time_s"]):
                d, lo, hi, n = cboot(o, hc, met)
                cells.append("%+.2f [%+.2f, %+.2f]" % (d, lo, hi))
            w("| %s | %s | %s | %d |" % (fam, st, " | ".join(cells), n))
    w("")
    w("## 3. Speed sweep (P8 / threat T3): 3 m/s vs 7 m/s (main)\n")
    w("| family | v_lim | stack | n | success | collision | contact v | time s | d coll vs hybrid_conf [CI] | d succ vs hybrid_conf [CI] |")
    w("|---|---|---|---|---|---|---|---|---|---|")
    for fam in ["S0", "S1", "S3"]:
        for cname, vl in (("speed_v3", 3), ("main_v7", 7)):
            hc = by(cname, fam, HYBRID_CONF)
            for st in [REACTIVE, MAP_RHP, OPT_BACKUP, HYBRID_CONF, HYBRID_FIXED, "hybrid_fixed_env"]:
                rs = by(cname, fam, st)
                if not rs:
                    continue
                dc = cboot(rs, hc, lambda r: float(r["collision"]))
                dsu = cboot(rs, hc, lambda r: float(r["success"]))
                w("| %s | %d | %s | %d | %.0f%% | %.0f%% | %.2f | %.1f | %+.2f [%+.2f, %+.2f] | %+.2f [%+.2f, %+.2f] |" % (
                    fam, vl, st, len(rs), 100 * _m(rs, lambda r: r["success"]), 100 * _m(rs, lambda r: r["collision"]),
                    _m(rs, lambda r: r["contact_speed"]), _m(rs, lambda r: r["time_s"]), dc[0], dc[1], dc[2],
                    dsu[0], dsu[1], dsu[2]))
    w("")
    w("## 4. Detector compute load and network loss (S1 + S3)\n")
    w("Detector GPU-share model: isolated rho 0; on 20 ms @ 10 Hz (rho 0.2); heavy 45 ms @ 10 Hz (rho 0.45); kappa 0.5 [U, "
      "no Tools/Vision timing in this tree].\n")
    groups = []
    for st in [MAP_RHP, HYBRID_CONF]:
        for cname, lab in (("det_isolated", "detector isolated"), ("main_v7", "detector on"),
                           ("det_heavy", "detector heavy"), ("netloss", "detector on + network loss")):
            rs = [r for r in by(cname, None, st) if r["family"] in ("S1", "S3")]
            if rs:
                groups.append(("%s / %s" % (st, lab), rs))
    table(L, groups, "stack / condition")
    w("")
    w("| stack | contrast | d age p99 ms [CI] | d collision [CI] | d success [CI] | d rep@20s [CI] |")
    w("|---|---|---|---|---|---|")
    for st in [MAP_RHP, HYBRID_CONF]:
        base = [r for r in by("main_v7", None, st) if r["family"] in ("S1", "S3")]
        for cname in ("det_isolated", "det_heavy", "netloss"):
            o = [r for r in by(cname, None, st) if r["family"] in ("S1", "S3")]
            if not o:
                continue
            cells = []
            for met in (lambda r: r["age_p99"] * 1e3, lambda r: float(r["collision"]), lambda r: float(r["success"]),
                        lambda r: r["reported_20"]):
                d, lo, hi, n = cboot(o, base, met)
                cells.append("%+.2f [%+.2f, %+.2f]" % (d, lo, hi))
            w("| %s | %s - detector on | %s |" % (st, cname, " | ".join(cells)))
    w("")
    w("## 5. Mission ablation: slower motion vs detector-aware speed (S0, S1, S3)\n")
    w("Arms: fast = v_lim 7 uniform; uniform4 = v_lim 4 uniform (safer/slower motion only); detaware = v_lim 7 with "
      "v_cap = 7 (1 - 0.5 V), V = max hint value (Gaussian, sigma 3 m, over noisy target hints + 2 decoys) at 0/3/6 m ahead.\n")
    groups = []
    for st in [HYBRID_CONF]:
        for cname, lab in (("main_v7", "fast"), ("abl_uniform4", "uniform4"), ("abl_detaware", "detaware")):
            rs = [r for r in by(cname, None, st) if r["family"] in ("S0", "S1", "S3")]
            if rs:
                groups.append(("%s / %s" % (st, lab), rs))
    table(L, groups, "stack / arm")
    w("")
    w("| stack | contrast | d collision [CI] | d near-miss [CI] | d recall [CI] | d rep@20s [CI] | d rep@30s [CI] | d time s [CI] |")
    w("|---|---|---|---|---|---|---|---|")
    for st in [HYBRID_CONF]:
        sub = lambda c: [r for r in by(c, None, st) if r["family"] in ("S0", "S1", "S3")]
        for a, b in (("abl_uniform4", "main_v7"), ("abl_detaware", "main_v7"), ("abl_detaware", "abl_uniform4")):
            cells = []
            for met in (lambda r: float(r["collision"]), lambda r: float(r["near_misses"]), lambda r: r["recall"],
                        lambda r: r["reported_20"], lambda r: r["reported_30"], lambda r: r["time_s"]):
                d, lo, hi, n = cboot(sub(a), sub(b), met)
                cells.append("%+.2f [%+.2f, %+.2f]" % (d, lo, hi))
            w("| %s | %s - %s | %s |" % (st, a, b, " | ".join(cells)))
    w("")
    w("## 6. Collision anatomy (main, S1-S4)\n")
    w("| stack | collisions | on scenario branch | wall | stem | other branch | mean contact v |")
    w("|---|---|---|---|---|---|---|")
    for st in stacks:
        cs = [r for r in by("main_v7", None, st) if r["collision"] and r["family"] != "S0"]
        if not by("main_v7", None, st):
            continue
        w("| %s | %d | %d | %d | %d | %d | %.2f |" % (
            st, len(cs), sum(1 for r in cs if r["contact_block"]), sum(1 for r in cs if r["contact_kind"] == "wall"),
            sum(1 for r in cs if r["contact_kind"] == "stem"),
            sum(1 for r in cs if r["contact_kind"] == "branch" and not r["contact_block"]),
            _m(cs, lambda r: r["contact_speed"])))
    w("")
    w("## 7. Adaptation trace (main): mean prior weight, mean speed cap, fallback fraction\n")
    w("| family | " + " | ".join(stacks) + " |")
    w("|---|" + "---|" * len(stacks))
    for fam in ["S0", "S1", "S2", "S3", "S4"]:
        cells = []
        for st in stacks:
            rs = by("main_v7", fam, st)
            cells.append("w %.2f / v %.1f / fb %.2f" % (_m(rs, lambda r: r["mean_w_prior"]), _m(rs, lambda r: r["mean_v_cap"]),
                                                        _m(rs, lambda r: r["fallback_frac"])) if rs else "-")
        w("| %s | %s |" % (fam, " | ".join(cells)))
    w("")
    if os.path.exists(TUNED):
        t = json.load(open(TUNED))
        w("## 8. Tuned configurations (DEV layouts 100-101, 8 configs per hybrid, equal budgets)\n")
        w("- HYBRID_CONF: `%s`" % json.dumps(t["conf"]))
        w("- HYBRID_FIXED (global): `%s`" % json.dumps(t["fixed"]))
        w("- HYBRID_FIXED per environment: `%s`\n" % json.dumps(t["fixed_env"]))
    out = os.path.join(RES, "E2_%s.md" % args.plan)
    with open(out, "w") as f:
        f.write("\n".join(L) + "\n")
    print("wrote", out)


def main(argv=None):
    ap = argparse.ArgumentParser()
    ap.add_argument("cmd", choices=["run", "tune-select", "report"])
    ap.add_argument("--plan", default="dev", choices=["tune", "dev", "heldout"])
    ap.add_argument("--shard", type=int, default=0)
    ap.add_argument("--nshards", type=int, default=1)
    ap.add_argument("--allow-heldout", action="store_true")
    ap.add_argument("--resume", action="store_true")
    ap.add_argument("--shards", default="", help="comma list of shard partitions to run in one process")
    a = ap.parse_args(argv)
    if a.plan == "heldout" and a.cmd == "run" and not a.allow_heldout:
        raise SystemExit("held-out plan refused without --allow-heldout")
    {"run": run, "tune-select": tune_select, "report": report}[a.cmd](a)


if __name__ == "__main__":
    main()
