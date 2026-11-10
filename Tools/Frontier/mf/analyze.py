# SPDX-License-Identifier: GPL-3.0-or-later
"""D4 retrospective: inventory of multi-fidelity cells, rank agreement (Kendall tau-b, bootstrap CI), noise ceilings.

    python analyze.py --out ~/.cache/frontier/mf/analysis.json     (seconds; reads logged rows only)

Kendall tau-b and split-half ceilings come from the DecisionCal judge (read-only import, ~/worktrees/decisioncal/judge/stats.py
and selection.py) when available, else from the local fallback below (same definition).
"""
import argparse
import collections
import json
import math
import os
import sys

import numpy as np

sys.dont_write_bytecode = True
HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
import data  # noqa: E402

_DC = os.path.join(os.path.expanduser("~"), "worktrees", "decisioncal")
try:
    sys.path.insert(0, _DC)
    from judge import stats as _jstats  # noqa: E402
    kendall = _jstats.kendall_tau
    JUDGE = True
except Exception:  # fallback with the same definition (tau-b)
    JUDGE = False

    def kendall(a, b):
        conc = disc = ta = tb = 0
        for i in range(len(a)):
            for j in range(i + 1, len(a)):
                da, db = a[i] - a[j], b[i] - b[j]
                if da == 0 and db == 0:
                    continue
                if da == 0:
                    ta += 1
                elif db == 0:
                    tb += 1
                elif da * db > 0:
                    conc += 1
                else:
                    disc += 1
        den = math.sqrt((conc + disc + ta) * (conc + disc + tb))
        return (conc - disc) / den if den else float("nan")


def tau_ci(x, y, B=2000, seed=0):
    """tau-b of paired vectors with a bootstrap CI that resamples the *units* (rows of x/y). NaN if undefined."""
    x, y = np.asarray(x, float), np.asarray(y, float)
    t = kendall(list(x), list(y))
    rng = np.random.default_rng(seed)
    vals = []
    for _ in range(B):
        i = rng.integers(0, len(x), len(x))
        v = kendall(list(x[i]), list(y[i]))
        if not math.isnan(v):
            vals.append(v)
    ci = (float(np.quantile(vals, 0.025)), float(np.quantile(vals, 0.975))) if len(vals) > 50 else (float("nan"), float("nan"))
    n = len(x)
    if n > 4 and not math.isnan(t) and abs(t) < 1:     # Fieller (1957): var(atanh tau) ~ 0.437 / (n - 4); conservative for small n
        z, se = math.atanh(t), math.sqrt(0.437 / (n - 4))
        fci = (math.tanh(z - 1.96 * se), math.tanh(z + 1.96 * se))
    else:
        fci = (float("nan"), float("nan"))
    perm = [kendall(list(x), list(rng.permutation(y))) for _ in range(2000)]
    p = float(np.mean([abs(v) >= abs(t) - 1e-12 for v in perm if not math.isnan(v)])) if not math.isnan(t) else float("nan")
    return {"tau": t, "ci95": ci, "ci95_fieller": fci, "p_perm": p, "n_units": len(x), "n_valid_boot": len(vals)}


def mean_by(rows, keyf, valf):
    d = collections.defaultdict(list)
    for r in rows:
        v = valf(r)
        if v is not None:
            d[keyf(r)].append(v)
    return {k: float(np.mean(v)) for k, v in d.items()}, {k: len(v) for k, v in d.items()}


# ------------------------------------------------------------------ inventory

def inventory(tables):
    out = {"sources": [], "strict_cells": [], "relaxed_cells": []}
    for name, rows in tables.items():
        if not rows:
            out["sources"].append({"source": name, "n": 0})
            continue
        out["sources"].append({"source": name, "fid": rows[0]["fid"], "split": sorted({r["split"] for r in rows}), "n": len(rows),
                               "configs": sorted({r["config"] for r in rows}), "n_forests": len({r["forest"] for r in rows}),
                               "speeds": sorted({r["speed"] for r in rows})})
    allrows = [r for rows in tables.values() for r in rows]
    for label, keyf, dest in (("strict (config, forest, speed)", lambda r: (r["config"], r["forest"], r["speed"]), "strict_cells"),
                              ("relaxed (config, speed)", lambda r: (r["config"], r["speed"]), "relaxed_cells")):
        cells = collections.defaultdict(lambda: collections.defaultdict(int))
        for r in allrows:
            cells[keyf(r)][r["fid"]] += 1
        for k, v in sorted(cells.items(), key=lambda kv: str(kv[0])):
            if len(v) >= 2:
                out[dest].append({"cell": list(k), "n_by_fidelity": dict(v)})
    # mission-level pairs: (config, forest, mission, speed, seed) present at two fidelities
    pk = collections.defaultdict(set)
    for r in allrows:
        if r["config"] == "map_rhp" and r["forest"] == 1337:
            pk[(r["scenario"], r["speed"], r["seed"])].add(r["fid"])
    pairs = collections.Counter()
    for (m, v, s), f in pk.items():
        if len(f) >= 2:
            pairs[(tuple(sorted(f)), v)] += 1
    out["run_level_pairs"] = [{"fidelities": list(k[0]), "speed": k[1], "n_pairs": n} for k, n in sorted(pairs.items())]
    return out


# ------------------------------------------------------------------ L2 vs L3 per-mission ranking

def mission_tau(l2, l3, speed, metric, B=2000):
    """Rank the missions common to both tiers by the per-mission mean of `metric` (over run seeds), same speed. a = first tier, b = second."""
    a = [r for r in l2 if r["speed"] == speed]
    b = [r for r in l3 if r["speed"] == speed]
    f = {"progress": lambda r: r["progress"], "min_clear": lambda r: r["min_clear"],
         "fail": lambda r: 0.0 if r["success"] else 1.0}[metric]
    ma, na = mean_by(a, lambda r: r["scenario"], f)
    mb, nb = mean_by(b, lambda r: r["scenario"], f)
    ms = sorted(set(ma) & set(mb))
    if len(ms) < 4:
        return {"speed": speed, "metric": metric, "n_missions": len(ms), "tau": None}
    x, y = [ma[m] for m in ms], [mb[m] for m in ms]
    res = tau_ci(x, y, B)
    res.update({"speed": speed, "metric": metric, "n_missions": len(ms), "n_runs_a": len(a), "n_runs_b": len(b),
                "const_side": "a" if len(set(x)) == 1 else ("b" if len(set(y)) == 1 else None),
                "mean_a": float(np.mean(x)), "mean_b": float(np.mean(y)),
                "split_half_ceiling": split_half(a, b, f)})
    return res


def split_half(a, b, f, n_splits=100, seed=0):
    """Noise ceiling: tau between mission rankings of seed-1 vs seed-2 runs *within* each fidelity, averaged (sqrt of product
    when both positive). Run seeds are CRN-paired, so this is the seed-to-seed rank stability of each fidelity."""
    out = {}
    for name, rows in (("a", a), ("b", b)):
        seeds = sorted({r["seed"] for r in rows})
        if len(seeds) < 2:
            out[name] = None
            continue
        s0, s1 = seeds[0], seeds[1]
        m0, _ = mean_by([r for r in rows if r["seed"] == s0], lambda r: r["scenario"], f)
        m1, _ = mean_by([r for r in rows if r["seed"] == s1], lambda r: r["scenario"], f)
        ms = sorted(set(m0) & set(m1))
        out[name] = kendall([m0[m] for m in ms], [m1[m] for m in ms]) if len(ms) >= 4 else None
    return out


def agreement_rates(l2, l3, a_name="L2", b_name="L3"):
    """Run-level paired outcome agreement at the same (mission, speed, seed): rows of `l2` (first tier) vs `l3` (second tier)."""
    d3 = {(r["scenario"], r["speed"], r["seed"]): r for r in l3}
    res = collections.defaultdict(lambda: [0, 0, 0, 0])  # n, same success flag, both collide, only one collides
    for r in l2:
        o = d3.get((r["scenario"], r["speed"], r["seed"]))
        if o is None:
            continue
        c = res[r["speed"]]
        c[0] += 1
        c[1] += r["success"] == o["success"]
        c[2] += r["collision"] and o["collision"]
        c[3] += r["collision"] != o["collision"]
    return {str(v): {"n_pairs": c[0], "same_success_flag": c[1], "both_collide": c[2], "collision_discordant": c[3]}
            for v, c in sorted(res.items())}


# ------------------------------------------------------------------ L1 internal: ranking stability across speed

def l1_speed_tau(e3, B=2000, seed=0):
    """Stack ranking at speed v1 vs v2 within NavPilot E3 held-out (RETRO-HELDOUT). Cluster bootstrap over the 8 layouts
    (all stacks, families, speeds of a layout move together), as the E3 report."""
    stacks = sorted({r["config"] for r in e3})
    layouts = sorted({r["forest"] for r in e3})

    def rank_vec(rows, v, metric):
        out = []
        for s in stacks:
            rr = [r for r in rows if r["config"] == s and r["speed"] == v]
            if metric == "collision":
                out.append(-np.mean([r["collision"] for r in rr]))      # higher = better (fewer collisions)
            else:
                out.append(np.mean([0.0 if r["collision"] else r["progress"] for r in rr]))  # safe progress rate
        return out

    res = []
    speeds = sorted({r["speed"] for r in e3})
    pairs = [(speeds[i], speeds[i + 1]) for i in range(len(speeds) - 1)] + [(speeds[0], speeds[-1]), (speeds[1], speeds[3])]
    rng = np.random.default_rng(seed)
    by_layout = {l: [r for r in e3 if r["forest"] == l] for l in layouts}
    for metric in ("collision", "safe_progress"):
        for v1, v2 in pairs:
            t = kendall(rank_vec(e3, v1, metric), rank_vec(e3, v2, metric))
            vals = []
            for _ in range(B):
                rows = [r for l in rng.integers(0, len(layouts), len(layouts)) for r in by_layout[layouts[l]]]
                v = kendall(rank_vec(rows, v1, metric), rank_vec(rows, v2, metric))
                if not math.isnan(v):
                    vals.append(v)
            ci = (float(np.quantile(vals, 0.025)), float(np.quantile(vals, 0.975))) if len(vals) > 50 else (None, None)
            res.append({"metric": metric, "v1": v1, "v2": v2, "tau": t, "ci95": ci, "n_stacks": len(stacks), "n_layouts": len(layouts)})
    return res


def l1_speed_tau_cells(e3, seed=0):
    """Same, over the 20 (stack, family) cells instead of 5 stacks (more units, but layouts still the only replication)."""
    cells = sorted({(r["config"], r["scenario"]) for r in e3})
    res = []
    speeds = sorted({r["speed"] for r in e3})
    for v1, v2 in [(speeds[i], speeds[i + 1]) for i in range(len(speeds) - 1)] + [(speeds[0], speeds[-1])]:
        x = [np.mean([-r["collision"] for r in e3 if (r["config"], r["scenario"]) == c and r["speed"] == v1]) for c in cells]
        y = [np.mean([-r["collision"] for r in e3 if (r["config"], r["scenario"]) == c and r["speed"] == v2]) for c in cells]
        res.append(dict(tau_ci(x, y, 1000, seed), v1=v1, v2=v2, n_cells=len(cells)))
    return res


# ------------------------------------------------------------------ L1 vs L2 level comparison (unpaired)

def rate(rows):
    return (sum(r["collision"] for r in rows), len(rows))


def wilson(k, n, z=1.96):
    if n == 0:
        return (float("nan"),) * 2
    p = k / n
    d = 1 + z * z / n
    c = (p + z * z / (2 * n)) / d
    h = z * math.sqrt(p * (1 - p) / n + z * z / (4 * n * n)) / d
    return (c - h, c + h)


def level_gap(t):
    out = []
    l1 = {"E2 map_rhp (main_v7 + det/netloss conds, RETRO-HELDOUT)": [r for r in t["navpilot_e2_heldout"] if r["config"] == "map_rhp"],
          "NavPilot dev sweep (all branch/sensor/tail conds, DEV)": t["navpilot_dev_sweep"]}
    l2 = t["crane_dotnet"]
    for name, rows in l1.items():
        for v in sorted({r["speed"] for r in rows}):
            k1, n1 = rate([r for r in rows if r["speed"] == v])
            e = {"l1": name, "l1_speed": v, "l1_k": k1, "l1_n": n1, "l1_ci": wilson(k1, n1)}
            near = [r for r in l2 if abs(r["speed"] - v) <= 1.0]
            if near:
                sp = sorted({r["speed"] for r in near})
                k2, n2 = rate(near)
                e.update({"l2_speeds": sp, "l2_k": k2, "l2_n": n2, "l2_ci": wilson(k2, n2)})
            out.append(e)
    return out


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--out", default=os.path.join(os.path.expanduser("~"), ".cache", "frontier", "mf", "analysis.json"))
    a = ap.parse_args()
    t = data.load_all()
    res = {"judge_stats_used": JUDGE, "inventory": inventory(t)}
    l2, l3 = t["crane_dotnet"], t["crane_unity"]
    res["l2_l3_mission_tau"] = [mission_tau(l2, l3, v, m) for v in (2.5, 4.0, 6.0) for m in ("progress", "min_clear", "fail")]
    res["l2_l3_run_agreement"] = agreement_rates(l2, l3)
    tw = t["navpilot_twin"]
    res["l1_l2_twin_tau"] = [mission_tau(tw, l2, v, m) for v in (2.0, 2.5, 3.0, 4.0, 5.0, 6.0, 8.0) for m in ("progress", "min_clear", "fail")]
    res["l1_l2_twin_run_agreement"] = agreement_rates(tw, l2, a_name="L1 twin", b_name="L2")
    res["l1_speed_tau_stacks"] = l1_speed_tau(t["navpilot_e3_heldout"])
    res["l1_speed_tau_cells"] = l1_speed_tau_cells(t["navpilot_e3_heldout"])
    res["l1_vs_l2_levels"] = level_gap(t)
    res["wall_s"] = {k: (float(np.median([r["wall_s"] for r in v if r["wall_s"]])), float(np.mean([r["wall_s"] for r in v if r["wall_s"]])))
                     for k, v in t.items() if any(r["wall_s"] for r in v)}
    os.makedirs(os.path.dirname(a.out), exist_ok=True)
    json.dump(res, open(a.out, "w"), indent=1, default=lambda o: None if o is None else str(o))
    print(json.dumps(res, indent=1, default=str)[:20000])


if __name__ == "__main__":
    main()
