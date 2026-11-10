# SPDX-License-Identifier: GPL-3.0-or-later
"""RoleSpec DEV experiments: equal-budget tuning, exploration grid, exploratory report.

DEV ONLY: seeds 0-199 (tuning 0-99, exploration 100-199). The RoleSpec held-out range 5000-5019 is refused by
rolespec.world.guard unless --allow-heldout (not exposed here on purpose). Nothing produced here is a claim.

  python experiments.py tune    --seeds 0:48 --out results/tuned.json
  python experiments.py explore --seeds 100:130 --tuned results/tuned.json --grid main --out results/explore_main.jsonl
  python experiments.py report  --inp results/explore_main.jsonl
"""
import argparse
import itertools
import json
import math
import os
import sys
import time

import numpy as np

_HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, _HERE)
sys.path.insert(0, os.path.normpath(os.path.join(_HERE, "..", "MissionSim")))
from rolespec import policies as P, sim, world as W  # noqa: E402
from missionsim import stats as MS  # noqa: E402  (paired cluster bootstrap, reused)

DEV_MAX = 199
K_TUNE = 10   # EQUAL tuning budget: configurations evaluated per policy family

SW = [0.0, 0.5, 0.9]   # soft pre-launch sector preference (shared scorer); B7 has its own hard sectors
SPACES = {
    "HOMO_FAST": {"alpha": [0.0, 0.25, 0.5, 1.0, 2.0], "beta": [0.0, 0.5, 0.8, 1.0], "sector_w": SW},
    "HOMO_THOROUGH": {"alpha": [0.0, 0.25, 0.5, 1.0, 2.0], "beta": [0.0, 0.5, 0.8, 1.0], "sector_w": SW},
    "HOMO_SHARED_REVISIT": {"alpha": [0.0, 0.25, 0.5, 1.0, 2.0], "beta": [0.0, 0.5, 0.8, 1.0], "sector_w": SW},
    "SPEC_FIXED": {"r": [0.5, 0.67, 0.8], "p_cand": [0.08, 0.15, 0.3], "lowconf": [0.4, 0.8, 99.0],
                   "fallback": ["slow_cover", "fast_cover", "any"], "alpha": [0.25, 0.5, 1.0], "beta": [0.5, 0.8], "sector_w": SW},
    "SPEC_ADAPTIVE": {"rule": ["threshold", "marginal"], "theta": [0.25, 0.5, 1.0, 2.0], "h": [0.0, 0.2, 0.5],
                      "p_cand": [0.08, 0.15, 0.3], "lowconf": [0.4, 0.8, 99.0], "stim_range_m": [0.0, 300.0],
                      "comm_self_verify": [True, False], "fallback": ["slow_cover", "any"],
                      "alpha": [0.25, 0.5, 1.0], "beta": [0.5, 0.8], "sector_w": SW},
    "B3_JIN_CONFIRM": {"p_cand": [0.08, 0.15, 0.3], "search_speeds": ["fast", "both"], "alpha": [0.25, 0.5, 1.0],
                       "beta": [0.5, 0.8], "sector_w": SW},
    "B6_AUCTION": {"p_cand": [0.08, 0.15, 0.3], "min_p_bid": [0.0, 0.3], "alpha": [0.25, 0.5, 1.0],
                   "beta": [0.5, 0.8], "sector_w": SW},
    "B7_SOLO": {"alpha": [0.0, 0.25, 0.5, 1.0, 2.0], "sectors": [True, False]},
}
LETTER = {"HOMO_FAST": "a", "HOMO_THOROUGH": "b", "HOMO_SHARED_REVISIT": "c", "SPEC_FIXED": "d",
          "SPEC_ADAPTIVE": "e"}


def configs(name, k=K_TUNE, seed=12345):
    """K configurations per family: the policy default first, then a seeded random sample without replacement."""
    sp = SPACES[name]
    keys = sorted(sp)
    full = [dict(zip(keys, vals)) for vals in itertools.product(*[sp[k_] for k_ in keys])]
    r = np.random.default_rng([seed, len(name)])
    idx = r.permutation(len(full))
    out = [{}]
    for i in idx:
        if len(out) >= k:
            break
        out.append(full[i])
    return out


def seeds_arg(s):
    a, b = s.split(":")
    out = list(range(int(a), int(b)))
    if out and max(out) > DEV_MAX:
        raise SystemExit("DEV phase: seeds must be within 0-%d" % DEV_MAX)
    return out


def grid(kind):
    base = dict(cells_per_drone=30)
    if kind == "main":
        return W.scenario_grid(**base)
    if kind == "tunemix":
        return W.scenario_grid(Ns=(2, 3, 5, 10), **base)
    if kind == "side":   # P3 / P4 / P6 / switching-penalty probes at N in {3, 5, 10}
        out = []
        for n in (3, 5, 10):
            b = W.Scenario(n_drones=n, occlusion="high", trade="strong", comms="outage", **base)
            out += [W.with_(b, trade="none"), W.with_(b, fa_scale=3.0, clutter=0.12), W.with_(b, switch_s=20.0),
                    W.with_(b, deadline_s=3600.0, battery_wh=400.0)]
        return out
    if kind == "ratio":
        return [W.Scenario(n_drones=n, occlusion="high", trade="strong", comms=c, fa_scale=f, **base)
                for n in (3, 5, 10) for c in ("ideal", "outage") for f in (1.0, 3.0)]
    raise SystemExit("unknown grid " + kind)


def run_one(seed, sc, name, kw, rep=0, tag=None):
    r = sim.run_mission(seed, sc, P.make(name, **kw), rep=rep)
    r["layout"] = seed
    r["family"] = tag or name
    if tag:
        r["policy"] = tag if not r["policy"].startswith("SPEC_FIXED(") else r["policy"]
    r["params"] = kw
    return r


def cmd_tune(a):
    seeds = seeds_arg(a.seeds)
    cells = grid("tunemix")
    fams = a.families.split(",") if a.families else list(SPACES)
    res = {}
    if os.path.exists(a.out):
        with open(a.out) as f:
            res = json.load(f)
    t0 = time.time()
    for name in fams:
        if name in res and not a.force:
            continue
        rows = []
        for ci, kw in enumerate(configs(name)):
            Js = []
            for s in seeds:
                sc = cells[s % len(cells)]
                Js.append(run_one(s, sc, name, kw)["J"])
            rows.append({"params": kw, "J": float(np.mean(Js)), "J_se": float(np.std(Js) / math.sqrt(len(Js)))})
            print("%s cfg%d %s J=%.4f (%.0fs)" % (name, ci, kw, rows[-1]["J"], time.time() - t0), flush=True)
        best = max(rows, key=lambda r: r["J"])
        res[name] = {"best": best["params"], "best_J": best["J"], "evaluated": rows, "k": len(rows),
                     "seeds": [seeds[0], seeds[-1]], "objective": "J = auc_rep - 0.25 * false_rep"}
        with open(a.out, "w") as f:
            json.dump(res, f, indent=1)


def arms(tuned):
    """(tag, family, params). B5_THRESHOLD = the best threshold-rule configuration among the SAME K evaluated
    SPEC_ADAPTIVE configurations (no extra tuning budget)."""
    out = []
    for name in ("HOMO_FAST", "HOMO_THOROUGH", "HOMO_SHARED_REVISIT", "SPEC_FIXED", "SPEC_ADAPTIVE",
                 "B3_JIN_CONFIRM", "B6_AUCTION", "B7_SOLO"):
        if name in tuned:
            out.append((name, name, tuned[name]["best"]))
    if "SPEC_ADAPTIVE" in tuned:
        thr = [r for r in tuned["SPEC_ADAPTIVE"]["evaluated"] if r["params"].get("rule", "threshold") == "threshold"]
        if thr:
            out.append(("B5_THRESHOLD", "SPEC_ADAPTIVE", max(thr, key=lambda r: r["J"])["params"]))
    out.append(("ORACLE", "ORACLE", {}))
    return out


def cmd_explore(a):
    seeds = seeds_arg(a.seeds)
    with open(a.tuned) as f:
        tuned = json.load(f)
    cells = grid(a.grid)
    A = arms(tuned)
    if a.grid == "ratio":
        base = dict(tuned["SPEC_FIXED"]["best"])
        A = [("HOMO_SHARED_REVISIT", "HOMO_SHARED_REVISIT", tuned["HOMO_SHARED_REVISIT"]["best"])]
        for r in (0.2, 0.34, 0.5, 0.67, 0.8):
            kw = dict(base)
            kw["r"] = r
            A.append(("SPEC_FIXED(r=%.2f)" % r, "SPEC_FIXED", kw))
    done = set()
    if os.path.exists(a.out):
        with open(a.out) as f:
            for line in f:
                d = json.loads(line)
                done.add((d["seed"], d.get("rep", 0), d["cell"], d["family"]))
    t0 = time.time()
    with open(a.out, "a") as f:
        for ci, sc in enumerate(cells):
            if a.shard and ci % a.nshard != a.shard - 1:
                continue
            for s in seeds:
                for rp in range(a.reps):
                    for tag, name, kw in A:
                        if (s, rp, sc.key(), tag) in done:
                            continue
                        r = run_one(s, sc, name, kw, rep=rp, tag=tag)
                        f.write(json.dumps(r) + "\n")
            f.flush()
            print("cell %s done (%.0fs)" % (sc.key(), time.time() - t0), flush=True)


METRICS = [("auc_rep", "AUC reported-confirmed"), ("recall_rep", "recall@D reported"),
           ("t_rep_mean_c", "t reported-confirmed (cens.)"), ("t_conf_mean_c", "t confirmed (cens.)"),
           ("t_first_det_c", "t first detection (cens.)"), ("false_rep", "false confirmed reports"),
           ("energy_wh", "energy Wh"), ("dup_frac", "duplicated effort")]


def holm(ps):
    order = np.argsort(ps)
    m = len(ps)
    adj = np.zeros(m)
    run = 0.0
    for k, i in enumerate(order):
        run = max(run, min(1.0, (m - k) * ps[i]))
        adj[i] = run
    return adj


def boot_p(rows, a, b, key, cell, n_boot=2000):
    """Two-sided paired cluster-bootstrap p (percentile inversion) and CI of mean(a - b) within one cell."""
    cond = lambda r: r["cell"] == cell
    est, lo, hi, nl, npairs = MS.cluster_boot_diff(rows, a, b, key, n_boot=n_boot, cond=cond)
    by = {}
    for r in rows:
        if r["cell"] == cell:
            by.setdefault((r["seed"], r.get("rep", 0)), {})[r["policy"]] = r[key]
    per = {}
    for (sd, _), v in by.items():
        if a in v and b in v:
            per.setdefault(sd, []).append(v[a] - v[b])
    if not per:
        return est, lo, hi, 1.0, 0
    L = sorted(per)
    m = np.array([np.mean(per[l]) for l in L])
    wgt = np.array([len(per[l]) for l in L], float)
    rr = np.random.default_rng(0)
    bs = []
    for _ in range(n_boot):
        i = rr.integers(0, len(L), len(L))
        bs.append(np.sum(m[i] * wgt[i]) / wgt[i].sum())
    bs = np.array(bs)
    d = m
    p = min(1.0, 2 * min((bs <= 0).mean(), (bs >= 0).mean()))
    return est, lo, hi, max(p, 1.0 / n_boot), int(wgt.sum())


def cmd_report(a):
    rows = []
    for path in a.inp.split(","):
        with open(path) as f:
            rows += [json.loads(l) for l in f]
    cells = sorted({r["cell"] for r in rows}, key=lambda k: (int(k.split("_")[0][1:]), k))
    pols = sorted({r["policy"] for r in rows})
    ref = a.ref
    print("# EXPLORATORY ONLY (DEV seeds; not a claim). ref = %s; key = %s" % (ref, a.key))
    print("\n## Per-cell means (n seeds)\n")
    hdr = ["cell", "n"] + pols
    print("| " + " | ".join(hdr) + " |")
    print("|" + "---|" * len(hdr))
    for c in cells:
        line = [c, str(len({r['seed'] for r in rows if r['cell'] == c}))]
        for p in pols:
            v = [r[a.key] for r in rows if r["cell"] == c and r["policy"] == p]
            line.append("%.3f" % np.mean(v) if v else "-")
        print("| " + " | ".join(line) + " |")
    print("\n## Paired differences vs %s on %s (cluster bootstrap over seeds, 95%% CI; Holm across all tests "
          "in this table, exploratory)\n" % (ref, a.key))
    tests = []
    for c in cells:
        for p in pols:
            if p == ref or p == "ORACLE":
                continue
            est, lo, hi, pv, n = boot_p(rows, p, ref, a.key, c, n_boot=a.n_boot)
            if n:
                tests.append([c, p, est, lo, hi, pv, n])
    adj = holm(np.array([t[5] for t in tests])) if tests else []
    print("| cell | policy | diff | 95% CI | p | p_Holm | n |")
    print("|---|---|---|---|---|---|---|")
    for t, pa in zip(tests, adj):
        flag = " **" if pa < 0.05 else ""
        print("| %s | %s | %+.3f | [%+.3f, %+.3f] | %.3f | %.3f%s | %d |" % (t[0], t[1], t[2], t[3], t[4], t[5], pa,
                                                                          flag, t[6]))
    if a.fail:
        print("\n## Failure labels\n")
        for p in pols:
            fl = {}
            for r in rows:
                if r["policy"] == p:
                    fl[r["fail"]] = fl.get(r["fail"], 0) + 1
            print("- %s: %s" % (p, ", ".join("%s %d" % kv for kv in sorted(fl.items(), key=lambda x: -x[1]))))
        print("\n## Other metrics (means over all cells)\n")
        print("| policy | " + " | ".join(m[1] for m in METRICS) + " |")
        print("|---|" + "---|" * len(METRICS))
        for p in pols:
            print("| %s | " % p + " | ".join("%.3f" % np.mean([r[m[0]] for r in rows if r["policy"] == p])
                                             for m in METRICS) + " |")


def main():
    ap = argparse.ArgumentParser()
    sp = ap.add_subparsers(dest="cmd")
    t = sp.add_parser("tune")
    t.add_argument("--seeds", default="0:48")
    t.add_argument("--out", default=os.path.join(_HERE, "results", "tuned.json"))
    t.add_argument("--families", default="")
    t.add_argument("--force", action="store_true")
    e = sp.add_parser("explore")
    e.add_argument("--seeds", default="100:130")
    e.add_argument("--tuned", default=os.path.join(_HERE, "results", "tuned.json"))
    e.add_argument("--grid", default="main")
    e.add_argument("--out", required=True)
    e.add_argument("--reps", type=int, default=1)
    e.add_argument("--shard", type=int, default=0)
    e.add_argument("--nshard", type=int, default=1)
    r = sp.add_parser("report")
    r.add_argument("--inp", required=True)
    r.add_argument("--ref", default="HOMO_SHARED_REVISIT")
    r.add_argument("--key", default="auc_rep")
    r.add_argument("--n-boot", dest="n_boot", type=int, default=2000)
    r.add_argument("--fail", action="store_true")
    a = ap.parse_args()
    {"tune": cmd_tune, "explore": cmd_explore, "report": cmd_report}[a.cmd](a)


if __name__ == "__main__":
    main()
