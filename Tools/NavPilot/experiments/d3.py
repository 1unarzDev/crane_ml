# SPDX-License-Identifier: GPL-3.0-or-later
"""D3 (DEV only): compute-aware high-speed navigation. Diagnosis (counterfactuals), adaptive-compute policies,
isotonic v* (E3 code), multi-rate metamorphic test.  Layouts are DEV seeds only (guard_seed refuses held-out).

  PY=~/.cache/research/fs-venv/bin/python
  $PY experiments/d3.py run diag|tune|train|eval|multirate   # sequential, resumable, one process
  $PY experiments/d3.py report                               # -> results/D3_dev.md

DEV layouts: diag 310-313, tune 330-331, train 340-341, eval 320-324, multirate = the eval episodes at 100/50 Hz.
Rows go to results/raw/d3_<stage>.jsonl, keyed by (layout, fam, stack, params-hash, cf, plant_hz, v).
"""
import argparse
import hashlib
import json
import math
import os
import sys
import time

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.normpath(os.path.join(HERE, ".."))
sys.path.insert(0, ROOT)

import numpy as np  # noqa: E402

from experiments import e3  # noqa: E402  (pava, v_star, curve_stats: E3 protocol code, imported not copied)
from experiments.d3_episode import default_cond, run as run_episode  # noqa: E402
from experiments.scenarios import build  # noqa: E402
from navpilot.forest import guard_seed  # noqa: E402
from stacks.compute_gov import D3_METHODS, N_FEAT  # noqa: E402

RAW = os.path.join(ROOT, "results", "raw")
RES = os.path.join(ROOT, "results")
FAMS = e3.FAMS
SPEEDS = e3.SPEEDS
DIAG_L, TUNE_L, TRAIN_L, EVAL_L = [310, 311, 312, 313], [330, 331], [340, 341], [320, 321, 322, 323, 324]
DIAG_V = [5.5, 7.0, 9.0]
ARMS = {"base": (), "lat0": ("lat0",), "lat50": ("lat50",), "brake": ("brake",), "lat0+brake": ("lat0", "brake"),
        "perc": ("perc",), "range": ("range",), "perc+range": ("perc", "range"), "map": ("map",)}
EVAL_M = ["opt_backup_t", "clear_only", "pas_full", "cc_opt", "cc_pas", "lp_pas", "rr_pas", "fix_l1", "fix_l2", "p99_nb", "cc_nb"]
MR_M, MR_V, MR_HZ = ["opt_backup_t", "pas_full"], [5.5, 7.0, 9.0], [200, 100, 50]


def tuned():
    return json.load(open(os.path.join(RAW, "e3_tuned.json")))


def base_params(m):
    t = tuned()
    b = D3_METHODS[m]["base"] if m in D3_METHODS else m
    return dict(t.get(b, {}))


def phash(p):
    return hashlib.sha1(json.dumps(p, sort_keys=True).encode()).hexdigest()[:8]


def extra_params(m):
    """D3 governor knobs picked by `tune` (results/raw/d3_tuned.json) or the CEM weights (d3_lp.json)."""
    out = {}
    try:
        if m in ("cc_opt", "cc_pas", "cc_nb"):
            out = dict(json.load(open(os.path.join(RAW, "d3_tuned.json"))).get("cc_opt" if m == "cc_nb" else m, {}))
        if m == "lp_pas":
            out = {"w": json.load(open(os.path.join(RAW, "d3_lp.json")))["best"]}
    except OSError:
        pass
    return out


def job(L, fam, m, v, cf=(), hz=200, params=None, profile="v1", eps=0.0):
    p = base_params(m)
    p.update(extra_params(m) if params is None else params)
    return {"layout": L, "fam": fam, "stack": m, "v_lim": v, "cf": list(cf), "plant_hz": hz, "params": p, "profile": profile, "eps": eps}


def key(j):
    return "%d|%s|%s|%.2f|%s|%d|%s%s" % (j["layout"], j["fam"], j["stack"], j["v_lim"], "+".join(j["cf"]), j["plant_hz"],
                                        phash(j["params"]), ("" if j.get("profile", "v1") == "v1" else "|" + j["profile"]) + ("|eps%g" % j["eps"] if j.get("eps") else ""))


def plan(stage):
    J = []
    if stage == "diag":
        for L in DIAG_L:
            for fam in FAMS:
                for v in DIAG_V:
                    for arm, cf in ARMS.items():
                        J.append(job(L, fam, "opt_backup_t", v, cf))
    elif stage == "eval":
        for L in EVAL_L:
            for fam in FAMS:
                for v in SPEEDS:
                    for m in EVAL_M:
                        J.append(job(L, fam, m, v))
    elif stage == "profile":   # CAD AOS7 profile (mass 1.843 kg, T/W 4.79) vs v1 (the frozen default), DEV, 3 eval layouts
        for L in EVAL_L[:3]:
            for fam in FAMS:
                for v in SPEEDS:
                    for m in ("opt_backup_t", "pas_full"):
                        J.append(job(L, fam, m, v, profile="cad"))
    elif stage == "chaos":   # chaos floor: 200 Hz plant, start y offset 1 mm, same 120 (layout, family, method, speed) cells
        for L in EVAL_L:
            for fam in FAMS:
                for v in MR_V:
                    for m in MR_M:
                        J.append(job(L, fam, m, v, eps=1e-3))
    elif stage == "multirate":
        for L in EVAL_L:
            for fam in FAMS:
                for v in MR_V:
                    for m in MR_M:
                        for hz in MR_HZ[1:]:
                            J.append(job(L, fam, m, v, hz=hz))
    else:
        raise SystemExit("stage %s" % stage)
    return J


def run_job(j, cache):
    guard_seed(j["layout"], allow_heldout=False)
    sfam = FAMS[j["fam"]][0]
    k = (j["layout"], sfam)
    if k not in cache:
        cache.clear()
        cache[k] = build(j["layout"], sfam)
    cond = default_cond(v_lim=float(j["v_lim"]), latency_tail=FAMS[j["fam"]][1], cf=tuple(j["cf"]),
                        plant_hz=float(j["plant_hz"]), profile=j.get("profile", "v1"), eps=float(j.get("eps", 0.0)))
    t0 = time.time()
    r = run_episode(cache[k], j["stack"], cond, j["layout"], j["params"])
    r.update({"layout": j["layout"], "fam": j["fam"], "stack": j["stack"], "v_lim": j["v_lim"], "cf": "+".join(j["cf"]),
              "plant_hz": j["plant_hz"], "profile": j.get("profile", "v1"), "eps": j.get("eps", 0.0), "params": j["params"], "key": key(j), "wall_s": time.time() - t0,
              "cpu_s": time.process_time()})
    return r


def load(stage):
    rows = {}
    p = os.path.join(RAW, "d3_%s.jsonl" % stage)
    if os.path.exists(p):
        for l in open(p):
            if l.strip():
                r = json.loads(l)
                rows[r["key"]] = r
    return rows


def run_stage(stage, limit=None):
    J = plan(stage)
    done = load(stage)
    todo = [j for j in J if key(j) not in done]
    print("[d3] %s: %d planned, %d done, %d to run" % (stage, len(J), len(done), len(todo)), flush=True)
    cache, t0 = {}, time.time()
    with open(os.path.join(RAW, "d3_%s.jsonl" % stage), "a") as f:
        for i, j in enumerate(todo[:limit]):
            f.write(json.dumps(run_job(j, cache)) + "\n")
            f.flush()
            if i % 20 == 0:
                print("[d3] %s %d/%d %.0fs cpu %.0fs" % (stage, i, len(todo), time.time() - t0, time.process_time()), flush=True)


# --------------------------------------------------------------------------- tuning (analytic) and CEM (learned)
def utility(rs):
    return float(np.mean([r["progress_rate"] - e3.LAMBDA * float(r["collision"]) for r in rs])) if rs else -1e9


def eval_cfg(m, params, layouts, speeds, cache):
    rs = []
    for L in layouts:
        for fam in FAMS:
            for v in speeds:
                rs.append(run_job(job(L, fam, m, v, params=dict(base_params(m), **params)), cache))
    return rs


def tune(limit=None):
    """Analytic governor knobs (delta, e_q, lam) on DEV layouts 330-331, speeds 7 and 9 (16 episodes/config)."""
    cfgs = [dict(delta=0.01, e_q=[0.0, 0.3, 0.8], lam=[0.0, 0.05, 0.12]),
            dict(delta=0.03, e_q=[0.0, 0.2, 0.5], lam=[0.0, 0.03, 0.08]),
            dict(delta=0.003, e_q=[0.0, 0.3, 0.8], lam=[0.0, 0.05, 0.12]),
            dict(delta=0.01, e_q=[0.0, 0.1, 0.3], lam=[0.0, 0.02, 0.05]),
            dict(delta=0.01, e_q=[0.0, 0.6, 1.5], lam=[0.0, 0.15, 0.3])]
    p = os.path.join(RAW, "d3_tune.jsonl")
    done = {json.loads(l)["tkey"]: json.loads(l) for l in open(p)} if os.path.exists(p) else {}
    cache = {}
    best = {}
    for m in ("cc_opt", "cc_pas"):
        sc = []
        for ci, cf in enumerate(cfgs):
            tk = "%s|%d" % (m, ci)
            if tk not in done:
                rs = eval_cfg(m, cf, TUNE_L, [7.0, 9.0], cache)
                rec = {"tkey": tk, "m": m, "cfg": cf, "n": len(rs), "util": utility(rs),
                       "coll": float(np.mean([r["collision"] for r in rs])), "prog": float(np.mean([r["progress_rate"] for r in rs]))}
                open(p, "a").write(json.dumps(rec) + "\n")
                done[tk] = rec
            sc.append(done[tk])
        best[m] = max(sc, key=lambda r: r["util"])["cfg"]
        print("[d3] tune", m, [(round(r["util"], 2), round(r["coll"], 2), round(r["prog"], 2)) for r in sc], flush=True)
    json.dump(best, open(os.path.join(RAW, "d3_tuned.json"), "w"), indent=1)


def train(iters=3, pop=6, elite=2, seed=11):
    """CEM over the lp_pas weights (2 heads x 3 x 5 = 30) on DEV layouts 340-341, speeds 7/9 (16 episodes/candidate).
    Mean initialised at 0 (= level 0, delta 0.01: pas_full with the p99-quantile rule). Resumable per iteration."""
    p = os.path.join(RAW, "d3_lp.json")
    st = json.load(open(p)) if os.path.exists(p) else {"it": 0, "mu": [0.0] * (2 * 3 * N_FEAT), "sd": [0.8] * (2 * 3 * N_FEAT),
                                                       "best": [0.0] * (2 * 3 * N_FEAT), "best_u": None, "log": []}
    cache = {}
    while st["it"] < iters:
        r = np.random.default_rng(seed + st["it"])
        mu, sd = np.array(st["mu"]), np.array(st["sd"])
        cands = [mu] + [mu + sd * r.standard_normal(len(mu)) for _ in range(pop - 1)]
        if st["it"] == 0:
            cands[0] = np.zeros(len(mu))
        res = []
        for w in cands:
            rs = eval_cfg("lp_pas", {"w": w.tolist()}, TRAIN_L, [7.0, 9.0], cache)
            res.append((utility(rs), float(np.mean([x["collision"] for x in rs])), float(np.mean([x["progress_rate"] for x in rs])), w))
        res.sort(key=lambda t: -t[0])
        E = np.array([t[3] for t in res[:elite]])
        st["mu"], st["sd"] = E.mean(0).tolist(), np.maximum(E.std(0), 0.15).tolist()
        if st["best_u"] is None or res[0][0] > st["best_u"]:
            st["best_u"], st["best"] = res[0][0], res[0][3].tolist()
        st["log"].append([[round(t[0], 3), round(t[1], 3), round(t[2], 3)] for t in res])
        st["it"] += 1
        json.dump(st, open(p, "w"))
        print("[d3] cem it", st["it"], st["log"][-1][0], flush=True)


# --------------------------------------------------------------------------- reports
def mcnemar(a, b):
    """Exact two-sided sign test on discordant pairs (a=1,b=0) vs (a=0,b=1)."""
    n = a + b
    if n == 0:
        return 1.0
    k = min(a, b)
    p = sum(math.comb(n, i) for i in range(k + 1)) / 2.0 ** n
    return min(1.0, 2 * p)


def wilson(k, n, z=1.96):
    if n == 0:
        return (float("nan"), float("nan"))
    p = k / n
    d = 1 + z * z / n
    c = (p + z * z / (2 * n)) / d
    h = z * math.sqrt(p * (1 - p) / n + z * z / (4 * n * n)) / d
    return (c - h, c + h)


def report_diag(L):
    rows = list(load("diag").values())
    if not rows:
        L.append("_diag not run_")
        return {}
    by = {}
    for r in rows:
        by[(r["layout"], r["fam"], r["v_lim"], r["cf"])] = r
    arms = {a: "+".join(cf) for a, cf in ARMS.items()}
    out = {}
    L += ["", "### Collision rate by counterfactual arm (opt_backup_t = the p99-age governor; 4 DEV layouts x 4 families = 16 episodes per cell)", "",
          "| arm | " + " | ".join("%g m/s" % v for v in DIAG_V) + " | pooled | removed (flips) | new collisions |", "|---|" + "---|" * (len(DIAG_V) + 3)]
    keys = sorted(set((r["layout"], r["fam"], r["v_lim"]) for r in rows))
    base = {k: by.get((k[0], k[1], k[2], ""), None) for k in keys}
    nb = sum(int(b["collision"]) for b in base.values() if b)
    for arm, cfk in arms.items():
        cells, tot, nn, rem, new = [], 0, 0, 0, 0
        for v in DIAG_V:
            ks = [k for k in keys if k[2] == v and by.get((k[0], k[1], k[2], cfk)) and base[k]]
            c = sum(int(by[(k[0], k[1], k[2], cfk)]["collision"]) for k in ks)
            cells.append("%d/%d" % (c, len(ks)))
            tot += c
            nn += len(ks)
        for k in keys:
            r = by.get((k[0], k[1], k[2], cfk))
            if r and base[k]:
                rem += int(base[k]["collision"] and not r["collision"])
                new += int((not base[k]["collision"]) and r["collision"])
        out[arm] = (tot, nn, rem, new)
        L.append("| %s | %s | %d/%d (%.0f%%) | %d | %d |" % (arm, " | ".join(cells), tot, nn, 100 * tot / max(nn, 1), rem, new))
    # per speed attribution shares: share of base collisions removed by the arm
    L += ["", "### Attribution: share of baseline collisions removed by each factor (paired episodes; McNemar exact p on flips)", "",
          "| factor (arms) | " + " | ".join("%g m/s" % v for v in DIAG_V) + " | pooled | McNemar p (pooled) |", "|---|" + "---|" * (len(DIAG_V) + 2)]
    groups = [("tau-tail only (lat50: p50 compute instead of heavy tail)", ["lat50"]),
              ("zero latency (lat0)", ["lat0"]), ("ideal braking (brake)", ["brake"]),
              ("tau-tail x braking envelope (lat0+brake)", ["lat0+brake"]), ("perfect perception (perc)", ["perc"]),
              ("infinite range (range)", ["range"]), ("occlusion-limited free space (perc+range)", ["perc+range"]),
              ("map staleness (map: prior = truth)", ["map"])]
    shares = {}
    for gname, g in groups:
        cells, rem_t, base_t, newt = [], 0, 0, 0
        for v in DIAG_V:
            ks = [k for k in keys if k[2] == v and base[k]]
            nbv = sum(int(base[k]["collision"]) for k in ks)
            rem = sum(int(base[k]["collision"] and not by[(k[0], k[1], k[2], arms[g[0]])]["collision"]) for k in ks if by.get((k[0], k[1], k[2], arms[g[0]])))
            cells.append("%d/%d" % (rem, nbv))
            rem_t += rem
            base_t += nbv
        new = out[g[0]][3]
        shares[gname] = (rem_t, base_t)
        L.append("| %s | %s | %d/%d (%.0f%%) | %.3f |" % (gname, " | ".join(cells), rem_t, base_t, 100 * rem_t / max(base_t, 1), mcnemar(rem_t, new)))
    def sh(name, v=None):
        rem, tot = 0, 0
        for vv in ([v] if v else DIAG_V):
            ks = [k for k in keys if k[2] == vv and base[k]]
            g = dict(groups)[name]
            tot += sum(int(base[k]["collision"]) for k in ks)
            rem += sum(int(base[k]["collision"] and not by[(k[0], k[1], k[2], arms[g[0]])]["collision"]) for k in ks if by.get((k[0], k[1], k[2], arms[g[0]])))
        return rem / max(tot, 1)
    lat_names = ["zero latency (lat0)", "tau-tail only (lat50: p50 compute instead of heavy tail)"]
    s_lat7 = max(sh(n, 7.0) for n in lat_names)
    s_per7 = sh("occlusion-limited free space (perc+range)", 7.0)
    s_brk7 = sh("ideal braking (brake)", 7.0)
    s_lb7 = sh("tau-tail x braking envelope (lat0+brake)", 7.0)
    verdict = ("**Verdict H1 vs NAV_LIT P1 at 7 m/s:** perfect latency removes %.0f%% of baseline collisions (plan H1 needs most of them), "
               "perfect perception+range removes %.0f%%, ideal braking %.0f%%, latency+braking together %.0f%%. " % (100 * s_lat7, 100 * s_per7, 100 * s_brk7, 100 * s_lb7))
    verdict += ("P1 (perception range, not latency) is favoured: latency removes < 50% and perception > 50% (NOVELTY_D3 P1)." if (s_lat7 < 0.5 and s_per7 > 0.5)
                else "NOVELTY_D3 P1 pattern (latency < 50%, perception > 50%) is NOT met; see the table for which factor dominates." )
    L += ["", verdict]
    det = {}
    for r in rows:
        if r["cf"] == "" and r["fam"] in ("stale", "occl"):
            det.setdefault(r["v_lim"], []).extend([b for b in r["blk_det_range"] if b is not None])
    L += ["First-detection range of the scenario thin branches, baseline, stale+occl (m, mean over detected branches; n): " + ", ".join(
        "%g m/s: %.1f (n=%d)" % (v, np.mean(x), len(x)) for v, x in sorted(det.items()) if x)]
    # residual and brake_infeasible / contact kind of baseline collisions
    bc = [b for b in base.values() if b and b["collision"]]
    allr = set()
    for k in keys:
        if base[k] and base[k]["collision"]:
            if all(by.get((k[0], k[1], k[2], arms[a])) and by[(k[0], k[1], k[2], arms[a])]["collision"] for a in arms if a != "base"):
                allr.add(k)
    L += ["", "Baseline collisions: %d of %d episodes. Not removed by ANY single counterfactual arm: %d (%.0f%%)." % (
        len(bc), len([b for b in base.values() if b]), len(allr), 100 * len(allr) / max(len(bc), 1))]
    kinds = {}
    for b in bc:
        kk = "%s%s" % (b["contact_kind"], "/block" if b["contact_block"] else "")
        kinds[kk] = kinds.get(kk, 0) + 1
    L += ["Contact kind of baseline collisions: %s." % ", ".join("%s %d" % kv for kv in sorted(kinds.items()))]
    L += ["brake_infeasible (NavPilot analogue: no-feasible-command onset <= 1 s before contact at > 1 m/s with true clearance < brake distance): %d of %d baseline collisions; with fallback/no-feasible events: %d." % (
        sum(int(b["brake_infeasible"]) for b in bc), len(bc), sum(int(b["nofeasible_events"] > 0) for b in bc))]
    L += ["", "By family (baseline collisions / episodes, then removed by lat0+brake, perc+range, map):", "",
          "| family | speed | base | lat0+brake | perc+range | map |", "|---|---|---|---|---|---|"]
    for fam in FAMS:
        for v in DIAG_V:
            ks = [k for k in keys if k[1] == fam and k[2] == v and base[k]]
            nbv = sum(int(base[k]["collision"]) for k in ks)
            cells = []
            for a in ("lat0+brake", "perc+range", "map"):
                cells.append("%d" % sum(int(by[(k[0], k[1], k[2], arms[a])]["collision"]) for k in ks if by.get((k[0], k[1], k[2], arms[a]))))
            L.append("| %s | %g | %d/%d | %s |" % (fam, v, nbv, len(ks), " | ".join(c + "/" + str(len(ks)) for c in cells)))
    # progress effect
    L += ["", "Mean progress rate (m/s) by arm (all episodes): " + ", ".join(
        "%s %.2f" % (a, np.mean([r["progress_rate"] for r in rows if r["cf"] == arms[a]])) for a in arms)]
    return out


def vstar_boot(rows, m, layouts, fams, pstar, B=2000, seed=0):
    rs = np.random.default_rng(seed)
    idx = rs.integers(0, len(layouts), (B, len(layouts)))
    pt = ep(rows, m, layouts, fams, pstar)
    bs = [ep(rows, m, [layouts[i] for i in idx[b]], fams, pstar) for b in range(B)]
    return pt, np.array(bs), idx


def ep(rows, m, layouts, fams, pstar):
    c, n, a, pa = e3.curve_stats(rows, m, layouts, fams)
    vs, cen = e3.v_star(SPEEDS, c, n, pstar)
    return {"v": vs, "cen": cen, "A": e3.achieved_at(SPEEDS, a, vs), "c": c, "n": n, "ach": a}


def fmt_v(e):
    return ("<%.1f" if e["cen"] < 0 else ">=%.1f" if e["cen"] > 0 else "%.2f") % e["v"]


def ci(x):
    return "%.2f-%.2f" % tuple(np.percentile(x, [2.5, 97.5]))


def report_eval(L):
    rows = list(load("eval").values())
    if not rows:
        L.append("_eval not run_")
        return {}
    lay = sorted(set(r["layout"] for r in rows))
    need = len(FAMS) * len(SPEEDS)
    cnt = {}
    for r in rows:
        cnt.setdefault((r["layout"], r["stack"]), 0)
        cnt[(r["layout"], r["stack"])] += 1
    lay = [x for x in lay if all(cnt.get((x, m), 0) == need for m in EVAL_M)]
    L.append("Complete DEV layouts: %s (%d). Episodes: %d. Cluster bootstrap over layouts (2000 resamples, seed 0). Only %d clusters, so CIs are wide and the numbers are DEV-grade." % (lay, len(lay), len(rows), len(lay)))
    res = {}
    for pstar, tag in ((0.05, "5 %"), (0.01, "1 %")):
        B = {}
        for m in EVAL_M:
            B[m] = vstar_boot(rows, m, lay, list(FAMS), pstar)
        res[pstar] = B
        base = B["opt_backup_t"]
        L += ["", "### Pooled over families, v* at P(collision) <= %s (isotonic, E3 code); dv* = method - p99 governor (opt_backup_t), paired" % tag, "",
              "| method | v* (95%% CI) | A* m/s (95%% CI) | dv* vs p99 gov (95%% CI) | dA* vs p99 gov (95%% CI) | collisions by setpoint %s | achieved by setpoint |" % SPEEDS,
              "|---|---|---|---|---|---|---|"]
        for m in EVAL_M:
            pt, bs, _ = B[m]
            vb = np.array([e["v"] for e in bs])
            ab = np.array([e["A"] for e in bs])
            if m == "opt_backup_t":
                dv = da = "-"
            else:
                dvb = vb - np.array([e["v"] for e in base[1]])
                dab = ab - np.array([e["A"] for e in base[1]])
                dv = "%+.2f (%+.2f, %+.2f)" % ((pt["v"] - base[0]["v"],) + tuple(np.percentile(dvb, [2.5, 97.5])))
                da = "%+.2f (%+.2f, %+.2f)" % ((pt["A"] - base[0]["A"],) + tuple(np.percentile(dab, [2.5, 97.5])))
            L.append("| %s | %s (%s) | %.2f (%s) | %s | %s | %s | %s |" % (
                m, fmt_v(pt), ci(vb), pt["A"], ci(ab), dv, da, " ".join("%d/%d" % (c, n) for c, n in zip(pt["c"], pt["n"])),
                " ".join("%.1f" % a for a in pt["ach"])))
    # backup vs no-backup deltas, fixed-c oracle (NOVELTY_D3 P2/P3) at 5 % and 1 %
    L += ["", "### Backup / no-backup and fixed-compute controls (NOVELTY_D3 P2, P3)", "",
          "Every method except p99_nb and cc_nb keeps the SUPER/FASTER-style known-free backup stop (opt_backup_t and pas_full are those baselines). p99_nb and cc_nb drop it: feasibility is evaluated on the optimistic belief (unknown = free) only.", "",
          "| contrast | dv* @5 % (95% CI) | dv* @1 % (95% CI) |", "|---|---|---|"]
    def dvs(m, b0, pstar):
        Bm, Bb = res[pstar][m], res[pstar][b0]
        d = np.array([x["v"] for x in Bm[1]]) - np.array([x["v"] for x in Bb[1]])
        return "%+.2f (%+.2f, %+.2f)" % ((Bm[0]["v"] - Bb[0]["v"],) + tuple(np.percentile(d, [2.5, 97.5])))
    for m, b0, lab in [("cc_nb", "p99_nb", "Delta (no backup): cc_nb - p99_nb"), ("cc_opt", "opt_backup_t", "Delta_b (with backup): cc_opt - opt_backup_t"),
                       ("cc_pas", "pas_full", "Delta_b (with backup + PAS): cc_pas - pas_full"), ("cc_pas", "opt_backup_t", "cc_pas - opt_backup_t (p99 governor, H2)"),
                       ("lp_pas", "opt_backup_t", "lp_pas - opt_backup_t"), ("rr_pas", "opt_backup_t", "RoboRun-style rr_pas - opt_backup_t"),
                       ("cc_pas", "rr_pas", "cc_pas - rr_pas"), ("cc_pas", "fix_l1", "cc_pas - fix_l1 (adaptive vs fixed level 1)"),
                       ("cc_pas", "fix_l2", "cc_pas - fix_l2 (adaptive vs fixed level 2)"), ("fix_l1", "pas_full", "fix_l1 - pas_full (compute-only)"),
                       ("fix_l2", "pas_full", "fix_l2 - pas_full (compute-only)")]:
        L.append("| %s | %s | %s |" % (lab, dvs(m, b0, 0.05), dvs(m, b0, 0.01)))
    fx = {m: res[0.05][m][0]["v"] for m in ("pas_full", "fix_l1", "fix_l2", "cc_pas")}
    gain_ad = fx["cc_pas"] - fx["pas_full"]
    gain_fx = max(fx["fix_l1"], fx["fix_l2"]) - fx["pas_full"]
    L += ["", "P3 check (5 %% crossing): adaptive cc_pas gain over level 0 (pas_full) = %+.2f m/s; best fixed level (1 or 2) gain = %+.2f m/s%s." % (
        gain_ad, gain_fx, (", ratio %.2f" % (gain_fx / gain_ad)) if abs(gain_ad) > 1e-6 else "")]
    # robust contrasts: collision rate and progress pooled over the >= 5.5 m/s setpoints (60 episodes per method), layout-cluster bootstrap
    hi = [5.5, 7.0, 9.0]
    def cell(m, Ls):
        rs = [r for r in rows if r["stack"] == m and r["layout"] in Ls and r["v_lim"] in hi]
        return np.mean([r["collision"] for r in rs]), np.mean([r["progress_rate"] for r in rs]), np.mean([r["stuck"] for r in rs])
    rsb = np.random.default_rng(1)
    bi = rsb.integers(0, len(lay), (2000, len(lay)))
    L += ["", "### Robust contrasts: pooled over the 5.5/7/9 m/s setpoints (60 episodes per method; v* from 20 episodes per setpoint is noisy: one collision moves it by metres)", "",
          "| method | collision | progress m/s | stuck | d collision vs opt_backup_t (95% CI) | d collision vs pas_full (95% CI) | d progress vs pas_full (95% CI) |", "|---|---|---|---|---|---|---|"]
    for m in EVAL_M:
        pt = cell(m, lay)
        d1 = np.array([cell(m, [lay[i] for i in b])[0] - cell("opt_backup_t", [lay[i] for i in b])[0] for b in bi])
        d2 = np.array([cell(m, [lay[i] for i in b])[0] - cell("pas_full", [lay[i] for i in b])[0] for b in bi])
        d3_ = np.array([cell(m, [lay[i] for i in b])[1] - cell("pas_full", [lay[i] for i in b])[1] for b in bi])
        f = lambda x: "%+.1f pp (%+.1f, %+.1f)" % (100 * (np.mean(x) if False else 0) , 0, 0)
        L.append("| %s | %.1f%% | %.2f | %.1f%% | %+.1f pp (%+.1f, %+.1f) | %+.1f pp (%+.1f, %+.1f) | %+.2f (%+.2f, %+.2f) |" % (
            (m, 100 * pt[0], pt[1], 100 * pt[2]) + (100 * (pt[0] - cell("opt_backup_t", lay)[0]),) + tuple(100 * np.percentile(d1, [2.5, 97.5])) +
            (100 * (pt[0] - cell("pas_full", lay)[0]),) + tuple(100 * np.percentile(d2, [2.5, 97.5])) +
            (pt[1] - cell("pas_full", lay)[1],) + tuple(np.percentile(d3_, [2.5, 97.5]))))
    # costs
    L += ["", "### Cost and diagnostics by method (all eval episodes)", "",
          "| method | collision | stuck | progress m/s | mean compute level | recall (persons) | brake_infeasible | fallback activations | stall-watchdog trips |", "|---|---|---|---|---|---|---|---|---|"]
    for m in EVAL_M:
        rs = [r for r in rows if r["stack"] == m]
        rec = [r["recall"] for r in rs if r["recall"] == r["recall"]]
        L.append("| %s | %.1f%% | %.1f%% | %.2f | %.2f | %.2f | %d | %d | %d |" % (
            m, 100 * np.mean([r["collision"] for r in rs]), 100 * np.mean([r["stuck"] for r in rs]),
            np.mean([r["progress_rate"] for r in rs]), np.mean([r["lev_mean"] for r in rs]), np.mean(rec) if rec else float("nan"),
            sum(int(r["brake_infeasible"]) for r in rs), sum(r["fallback_activations"] for r in rs), sum(r["watchdog"] for r in rs)))
    L += ["", "### Per family v* at 5 % (point estimates; 5 layouts per family, CIs omitted as uninformative)", "",
          "| family | " + " | ".join(EVAL_M) + " |", "|---|" + "---|" * len(EVAL_M)]
    for fam in FAMS:
        L.append("| %s | %s |" % (fam, " | ".join(fmt_v(ep(rows, m, lay, [fam], 0.05)) for m in EVAL_M)))
    return res


def report_multirate(L):
    ev, mr = load("eval"), load("multirate")
    if not mr:
        L.append("_multirate not run_")
        return
    ref = {}
    for r in ev.values():
        if r["stack"] in MR_M and r["v_lim"] in MR_V:
            ref[(r["layout"], r["fam"], r["stack"], r["v_lim"])] = r
    L += ["Reference = the 200 Hz eval episodes (same seeds, same code, event tick 5 ms in every arm); the plant is integrated every 1/2/4 ticks (200/100/50 Hz) with a zero-order-hold command and exactly aggregated OU wind noise. Planner, sensors, collision test (50 Hz swept segment) and RNG streams are identical.", "",
          "| plant | pairs | collision agreement | Wilson 95% | disagreements (ref>x / x>ref) | |dmin_clear| mean / median / p95 (cm), all pairs | same, pairs both non-collision | |d progress_m| mean (m) | verdict (>=99 % and <= 2 cm) |", "|---|---|---|---|---|---|---|---|---|"]
    ch = load("chaos")
    for hz in MR_HZ[1:] + ([0] if ch else []):
        pr = []
        for r in (ch if hz == 0 else mr).values():
            if hz == 0 or r["plant_hz"] == hz:
                k = (r["layout"], r["fam"], r["stack"], r["v_lim"])
                if k in ref:
                    pr.append((ref[k], r))
        if not pr:
            continue
        agree = sum(int(a["collision"] == b["collision"]) for a, b in pr)
        d1 = sum(int(a["collision"] and not b["collision"]) for a, b in pr)
        d2 = sum(int(b["collision"] and not a["collision"]) for a, b in pr)
        dc = np.array([abs(a["min_clear"] - b["min_clear"]) for a, b in pr]) * 100
        nc = np.array([abs(a["min_clear"] - b["min_clear"]) for a, b in pr if not a["collision"] and not b["collision"]]) * 100
        dp = np.mean([abs(a["progress_m"] - b["progress_m"]) for a, b in pr])
        w = wilson(agree, len(pr))
        ok = agree / len(pr) >= 0.99 and np.median(dc) <= 2.0 and np.mean(dc) <= 2.0
        pr_all = pr
        L.append("| %s | %d | %.1f%% | %.1f-%.1f%% | %d / %d | %.1f / %.1f / %.1f | %.1f / %.1f / %.1f | %.2f | %s |" % (
            ("%d Hz" % hz) if hz else "CHAOS FLOOR: 200 Hz, start y + 1 mm", len(pr), 100 * agree / len(pr), 100 * w[0], 100 * w[1], d1, d2, dc.mean(), np.median(dc), np.percentile(dc, 95),
            nc.mean() if len(nc) else float("nan"), np.median(nc) if len(nc) else float("nan"), np.percentile(nc, 95) if len(nc) else float("nan"), dp,
            "PASS" if ok else "FAIL"))
        rr_, rx_ = sum(int(a["collision"]) for a, b in pr), sum(int(b["collision"]) for a, b in pr)
        L.append("|  ... collision RATE ref %d/%d vs %d/%d (McNemar exact p = %.2f) | | | | | | | | |" % (rr_, len(pr), rx_, len(pr), mcnemar(sum(int(a["collision"] and not b["collision"]) for a, b in pr), sum(int(b["collision"] and not a["collision"]) for a, b in pr))))


def report_profile(L):
    ev, pf = load("eval"), load("profile")
    if not pf:
        L.append("_profile stage not run_")
        return
    lay = EVAL_L[:3]
    rows_v1 = [r for r in ev.values() if r["stack"] in ("opt_backup_t", "pas_full") and r["layout"] in lay]
    rows_cad = list(pf.values())
    L += ["DEV layouts %s, 4 families, 5 setpoints; v1 = frozen default (1.383 kg, T/W 6.38), CAD = `aos7_v5_f90_6s.json` (1.843 kg, T/W 4.79). In NavPilot the horizontal authority is tilt-limited (a_h_max = 8.23 m/s^2 for both: the thrust limit is 36 m/s^2 at T/W 4.79), so the profile enters only through drag/mass (quadratic drag term) and nothing else.".replace("%s", str(lay)), "",
          "| method | profile | v* @5 %% | v* @1 %% | collisions by setpoint %s | achieved |" % SPEEDS, "|---|---|---|---|---|---|"]
    for m in ("opt_backup_t", "pas_full"):
        for tag, rs in (("v1", rows_v1), ("CAD", rows_cad)):
            e5, e1 = ep(rs, m, lay, list(FAMS), 0.05), ep(rs, m, lay, list(FAMS), 0.01)
            L.append("| %s | %s | %s | %s | %s | %s |" % (m, tag, fmt_v(e5), fmt_v(e1), " ".join("%d/%d" % (c, n) for c, n in zip(e5["c"], e5["n"])),
                                                         " ".join("%.1f" % a for a in e5["ach"])))


def report(args):
    L = ["# D3 (DEV only): compute-aware high-speed navigation", "",
         "_Generated by `experiments/d3.py report`. DEV layouts only (diag 310-313, tune 330-331, train 340-341, eval 320-324); no E2/E3 held-out layout (90400-90407) and no E4 layout (90600-90615) was used. "
         "Evidence label: IMPLEMENTED (2.5D point mass, synthetic sensors/forest/scenarios); not LIVE_SIM_VALIDATED, not PHYSICALLY_CALIBRATED. Not reportable as a result: DEV._", ""]
    L += ["## 1. Diagnosis (H1)"]
    report_diag(L)
    L += ["", "## 3. DEV evaluation of the adaptive-compute policies (H2)"]
    report_eval(L)
    L += ["", "## 4. Multi-rate metamorphic test (NavPilot)"]
    report_multirate(L)
    L += ["", "## 5. Physics profile (v1 vs CAD AOS7)"]
    report_profile(L)
    open(os.path.join(RES, "D3_dev_tables.md"), "w").write("\n".join(L) + "\n")
    print("wrote results/D3_dev_tables.md")


if __name__ == "__main__":
    ap = argparse.ArgumentParser()
    ap.add_argument("cmd")
    ap.add_argument("stage", nargs="?")
    ap.add_argument("--limit", type=int, default=None)
    a = ap.parse_args()
    if a.cmd == "run":
        if a.stage == "eval":
            run_stage("diag")     # top-up of the rows dropped after the 10:38 profile swap (resumable, key-based)
        if a.stage == "tune":
            tune()
        elif a.stage == "train":
            train()
        else:
            run_stage(a.stage, a.limit)
            if a.stage == "multirate":
                run_stage("profile")
            if a.stage == "chaos":
                pass
    elif a.cmd == "report":
        report(a)
