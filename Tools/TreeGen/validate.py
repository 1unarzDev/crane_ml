#!/usr/bin/env python3
# SPDX-License-Identifier: GPL-3.0-or-later
"""Validate generated trees against the preset target ranges and, optionally, Wytham Woods TLS/QSM data.

  fsrun <python> -I Tools/TreeGen/validate.py --dir OUT [--wytham /path/to/trees_summary.csv]
                    [--json report.json] [--plots PNGDIR]

The Wytham CSV (Calders et al., Zenodo 7307956, CC-BY-4.0, file analysis_and_figures/trees_summary.csv)
is untrusted input. Only numeric columns are parsed. The 39 GB QSM archive is NOT used.
Checks per tree: height, DBH, crown ratio, and order-1 branching angle against preset targets [U].
Checks per dataset: the DBH-height relation (generated height percentile among Wytham trees of
similar DBH) and the share of woody volume by QSM diameter class.
"""
from __future__ import annotations

import argparse
import csv
import json
import math
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

import numpy as np  # noqa: E402

from treegen import metrics  # noqa: E402
from treegen.skeleton import Skeleton  # noqa: E402
from treegen.species import load_species  # noqa: E402

DCLASS_COLS = ["Vol_QSM_D0_25mm_avg_[m3]", "Vol_QSM_D25_50mm_avg_[m3]", "Vol_QSM_D50_75mm_avg_[m3]",
               "Vol_QSM_D75_100mm_avg_[m3]", "Vol_QSM_D100_200mm_avg_[m3]", "Vol_QSM_D200_500mm_avg_[m3]",
               "Vol_QSM_D500_1000mm_avg_[m3]", "Vol_QSM_D1000mm_avg_[m3]"]
DCLASS_LABELS = ["0-25", "25-50", "50-75", "75-100", "100-200", "200-500", "500-1000", ">1000"]


def _f(x):
    try:
        v = float(x)
        return v if math.isfinite(v) else float("nan")
    except (TypeError, ValueError):
        return float("nan")


def load_wytham(path):
    rows = []
    with open(path, newline="") as f:
        for r in csv.DictReader(f):
            dbh = _f(r.get("DBH_TLS_[m]"))
            h = _f(r.get("Hgt_pts_[m]"))
            if not (dbh > 0 and h > 0):
                continue
            vols = np.array([_f(r.get(c)) for c in DCLASS_COLS])
            rows.append({"species": str(r.get("species", ""))[:12], "dead": str(r.get("Dead", ""))[:1],
                         "dbh": dbh, "h": h, "cpa": _f(r.get("VerticalCrownProjectedArea_pts_[m2]")),
                         "vols": vols})
    return rows


def check_range(v, rng):
    if rng is None or not np.isfinite(v):
        return None
    return bool(rng[0] <= v <= rng[1])


def main(argv=None):
    ap = argparse.ArgumentParser()
    ap.add_argument("--dir", required=True, help="library or stand output directory")
    ap.add_argument("--wytham")
    ap.add_argument("--json")
    ap.add_argument("--plots")
    a = ap.parse_args(argv)
    mf = next(m for m in ("stand_manifest.json", "library_manifest.json") if os.path.exists(os.path.join(a.dir, m)))
    man = json.load(open(os.path.join(a.dir, mf)))
    wy = load_wytham(a.wytham) if a.wytham else None
    report = {"source": os.path.join(a.dir, mf), "trees": [], "wytham": None}
    gen_vol = np.zeros(8)
    all_ang = {}
    all_d = []
    for t in man["trees"]:
        sk = Skeleton.load_json(os.path.join(a.dir, t["dir"], "skeleton.json"))
        sp = load_species(t["species"])
        tg = sp.get("targets", {})
        s = metrics.summary(sk)
        ang = metrics.branching_angles(sk)
        a1 = float(np.median(ang[1])) if 1 in ang and len(ang[1]) else float("nan")
        n_fit, _ = metrics.pipe_exponent_fit(sk)
        entry = {"tree_id": t["tree_id"], "species": t["species"], "variant": t["variant"], **s,
                 "pipe_exponent_fit": round(n_fit, 2), "pipe_exponent_preset": sp["n_pipe"],
                 "checks": {}}
        if t["variant"] == "live":
            entry["checks"] = {"height": check_range(s["height_m"], tg.get("height_m")),
                               "dbh": check_range(s["dbh_m"], tg.get("dbh_m")),
                               "crown_ratio": check_range(s["crown_ratio"], tg.get("crown_ratio")),
                               "branch_angle_order1": check_range(a1, tg.get("branch_angle_order1_deg"))}
            gen_vol += metrics.volume_by_diameter_class(sk)
            for o, v in ang.items():
                all_ang.setdefault(t["species"], {}).setdefault(o, []).extend(v.tolist())
            d, _ = metrics.branch_base_diameters(sk)
            all_d.extend(d.tolist())
            if wy:
                sim = [r for r in wy if r["dead"] != "Y" and abs(r["dbh"] - s["dbh_m"]) <= 0.25 * s["dbh_m"]]
                if len(sim) >= 5 and np.isfinite(s["dbh_m"]):
                    hs = np.array([r["h"] for r in sim])
                    entry["wytham_similar_dbh_n"] = len(sim)
                    entry["wytham_height_percentile"] = round(float((hs < s["height_m"]).mean() * 100), 1)
                    entry["wytham_height_p10_p90"] = [round(float(np.percentile(hs, 10)), 1),
                                                      round(float(np.percentile(hs, 90)), 1)]
        report["trees"].append(entry)
    if wy:
        live = [r for r in wy if r["dead"] != "Y"]
        V = np.nansum(np.array([r["vols"] for r in live]), 0)
        report["wytham"] = {
            "n_trees": len(live),
            "volume_share_by_diameter_class_mm": dict(zip(DCLASS_LABELS, np.round(V / V.sum(), 4).tolist())),
            "generated_volume_share_by_diameter_class_mm": dict(zip(DCLASS_LABELS, np.round(gen_vol / max(gen_vol.sum(), 1e-12), 4).tolist())),
            "note": "Wytham is a mature 1.4 ha broadleaf stand (max DBH > 1 m); generated trees are 26-36 seasons old, "
                    "so large classes are expected to be under-represented. QSM < 5-10 cm is unreliable [U].",
        }
    # human summary
    for e in report["trees"]:
        c = e["checks"]
        flags = " ".join(f"{k}={'OK' if v else ('OUT' if v is False else '-')}" for k, v in c.items())
        extra = f" wytham_h_pct={e.get('wytham_height_percentile', '-')}" if wy else ""
        print(f"{e['tree_id']:4d} {e['species']:<14} {e['variant']:<10} H={e['height_m']:6.2f} DBH={e['dbh_m']:.3f} "
              f"CR={e['crown_ratio']} a1={e['branch_angle_median_deg_by_order'].get(1, '-')} "
              f"n_fit={e['pipe_exponent_fit']} {flags}{extra}")
    if report["wytham"]:
        print("volume share by diameter class (mm)  Wytham vs generated:")
        for k in DCLASS_LABELS:
            print(f"  {k:>9}: {report['wytham']['volume_share_by_diameter_class_mm'][k]:.3f}  "
                  f"{report['wytham']['generated_volume_share_by_diameter_class_mm'][k]:.3f}")
    if a.json:
        with open(a.json, "w") as f:
            json.dump(report, f, indent=1, default=float)
    if a.plots:
        import matplotlib
        matplotlib.use("Agg")
        import matplotlib.pyplot as plt
        os.makedirs(a.plots, exist_ok=True)
        fig, ax = plt.subplots(1, 3, figsize=(16, 5))
        if wy:
            ax[0].scatter([r["dbh"] for r in wy], [r["h"] for r in wy], s=4, c="0.6", label="Wytham TLS (CC-BY-4.0)")
        for e in report["trees"]:
            if e["variant"] == "live" and np.isfinite(e["dbh_m"]):
                ax[0].scatter(e["dbh_m"], e["height_m"], s=30, label=e["species"])
        h, l = ax[0].get_legend_handles_labels()
        uniq = dict(zip(l, h))
        ax[0].legend(uniq.values(), uniq.keys(), fontsize=7)
        ax[0].set_xlabel("DBH (m)")
        ax[0].set_ylabel("height (m)")
        ax[0].set_title("DBH-height")
        for spn, d in all_ang.items():
            orders = sorted(d)
            ax[1].boxplot([d[o] for o in orders], positions=np.array(orders) + 0.2 * list(all_ang).index(spn),
                          widths=0.18, showfliers=False)
            ax[1].plot([], [], label=spn)
        ax[1].set_xlabel("branch order")
        ax[1].set_ylabel("branching angle (deg)")
        ax[1].set_title("branching angle by order (boxes per species)")
        ax[1].legend(fontsize=7)
        if all_d:
            bins = np.logspace(np.log10(0.005), np.log10(1.0), 25)
            ax[2].hist(all_d, bins=bins, color="tab:brown")
            ax[2].set_xscale("log")
            ax[2].set_yscale("log")
            ax[2].set_xlabel("branch base diameter (m)")
            ax[2].set_title("branch-diameter distribution (log bins)")
        fig.tight_layout()
        fig.savefig(os.path.join(a.plots, "validation.png"), dpi=100)
    return report


if __name__ == "__main__":
    main()
