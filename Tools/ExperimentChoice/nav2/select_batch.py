"""First Nav2 batch for the offset-doorway study: course list, features, surrogate lookup (EXPLORATORY).

Writes, next to this file:
  courses_v0/course_<id>.json   truth manifests (course.py), the input of the Nav2-side design builder
  batch_v0.json                 the list with features for three footprints and the surrogate's nearest cell
  batch_v0.md                   the same as a table

Nothing here runs a simulator. Run from anywhere:
  ulimit -v 1500000; OPENBLAS_NUM_THREADS=1 nice -n 19 python3 -I Tools/ExperimentChoice/nav2/select_batch.py
"""
from __future__ import annotations

import json
import math
import os
import sys
from dataclasses import replace

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.dirname(HERE))
import course as C          # noqa: E402
import mechanics as M       # noqa: E402
import robots as R          # noqa: E402

GRID = os.path.expanduser("~/.cache/expchoice/grid_compact_diff_small_v2.jsonl")
# Three rectangles for the same robot (modular study, morphologies.yaml at 00ac590e):
#   catalog  = morphologies.compact_diff.footprint (physical envelope + 0.07-0.08 m per side); the
#              surrogate grid and robots.py use this one.
#   nav2     = navigation_footprint (0.66 x 0.56) + Nav2's default footprint_padding 0.01 m per side;
#              what both costmaps and both Nav2 arms collision-check.
#   physical = pods at x = +-0.23 (0.18 long), y = +-0.20 (0.175 wide over the wheels); what Gazebo
#              collides and what the post-hoc contact inference uses.
FOOTPRINTS = {"catalog": (0.80, 0.72), "nav2": (0.68, 0.58), "physical": (0.64, 0.575)}
SMAC_TURNING_RADIUS = 0.40
W_DOOR, T_WALL = 0.90, 0.20
# (d, e) in metres. Every wall edge is a multiple of 5 cm, so the 0.01 m map and a 5x downsample are exact.
COURSES = [(0.85, 0.35), (0.85, 0.70), (0.85, 1.05), (0.90, 0.35), (0.90, 0.70),
           (0.95, 0.70), (1.00, 0.70), (1.05, 0.70), (1.10, 0.70), (1.20, 0.70)]
CANARY = (1.20, 0.70)


def surrogate_cell(rows, eps_over_w, e_over_w):
    """Nearest surrogate grid cell in (eps/W, e/W) of the catalog footprint; the grid has w/W = 1.3."""
    best = min(rows, key=lambda r: (r["features"]["eps_over_W"] - eps_over_w) ** 2
               + (r["features"]["e_over_W"] - e_over_w) ** 2)
    res = best["result"]
    n = len(res["gap"])
    ok_a, ok_b = res["status_A"].count("success"), res["status_B"].count("success")
    gap = sum(res["gap"]) / n
    label = ("both fail" if ok_a == 0 and ok_b == 0 else "tie" if abs(gap) < 1.0 else "B" if gap > 0 else "A")
    return {"grid_d_cm": best["parameters"]["d_cm"], "grid_e_cm": best["parameters"]["e_cm"],
            "grid_w_cm": best["parameters"]["w_cm"], "success_A": "%d/%d" % (ok_a, n),
            "success_B": "%d/%d" % (ok_b, n), "mean_J_A": round(sum(res["J_A"]) / n, 1),
            "mean_J_B": round(sum(res["J_B"]) / n, 1), "mean_gap_s": round(gap, 1), "winner": label}


def reeds_shepp_strokes(e, d, L, radius):
    """UNVERIFIED back-of-envelope for a car-like follower (configuration C has no in-place rotation): one S-shaped
    stroke of longitudinal travel eps = d - L shifts sideways by 2R(1 - cos phi), sin phi = eps / 2R. It
    ignores the heading swing's own footprint extent, so it undercounts. Not a mechanics.py result."""
    eps = d - L
    if eps <= 0:
        return None
    if eps >= 2 * radius:
        return 1
    phi = math.asin(eps / (2 * radius))
    return math.ceil(e / (2 * radius * (1 - math.cos(phi))))


def main():
    base = R.load_robot("compact_diff")
    rows = [json.loads(line) for line in open(GRID, encoding="utf-8")]
    out_dir = os.path.join(HERE, "courses_v0")
    os.makedirs(out_dir, exist_ok=True)
    batch = []
    for d, e in COURSES:
        crs = C.make_course(w=W_DOOR, e=e, d=d, t=T_WALL)
        with open(os.path.join(out_dir, "course_%s.json" % crs.course_id), "w", encoding="utf-8") as f:
            f.write(C.manifest_json(crs, base))
        entry = {"course_id": crs.course_id, "d_m": crs.d, "e_m": crs.e, "w_m": crs.w, "t_m": crs.t,
                 "canary": (d, e) == CANARY, "in_admissible_box_v0": all(
                     C.ADMISSIBLE_BOX_V0[k][0] <= getattr(crs, k) <= C.ADMISSIBLE_BOX_V0[k][1]
                     for k in ("w_cm", "e_cm", "d_cm", "t_cm", "B_cm")), "footprints": {}}
        for name, (L, W) in FOOTPRINTS.items():
            rob = replace(base, L=L, W=W)
            ft = M.features(crs, rob)
            entry["footprints"][name] = {
                "L": L, "W": W, "two_rho": round(2 * M.rho(L, W), 4), "regime": ft["regime"],
                "d_over_2rho": round(ft["d_over_2rho"], 3), "eps_over_W": round(ft["eps_over_W"], 3),
                "e_over_W": round(ft["e_over_W"], 3), "w_over_W": round(ft["w_over_W"], 3),
                "alpha_max_deg": round(math.degrees(M.alpha_max(crs.d, L, W)), 1) if crs.d > L else 0.0,
                "four_flow_cycles": ft["predicted_cycles"], "min_reversals": ft["min_reversals_exact"],
                "door_heading_tolerance_rad": round(ft["heading_tolerance"], 3),
                "certificate": ft["certificate"]["verdict"]}
        cat = entry["footprints"]["catalog"]
        entry["surrogate_catalog_footprint"] = surrogate_cell(rows, cat["eps_over_W"], cat["e_over_W"])
        entry["config_C_strokes_unverified"] = reeds_shepp_strokes(crs.e, crs.d, FOOTPRINTS["nav2"][0],
                                                                 SMAC_TURNING_RADIUS)
        batch.append(entry)
    doc = {"label": "exploratory", "schema": "expchoice.nav2_batch/v0", "robot": "compact_diff",
           "footprints": FOOTPRINTS, "surrogate_grid": GRID, "smac_turning_radius": SMAC_TURNING_RADIUS,
           "sources": R.source_hashes(), "courses": batch}
    with open(os.path.join(HERE, "batch_v0.json"), "w", encoding="utf-8") as f:
        json.dump(doc, f, indent=1, sort_keys=True)
    head = ("| # | course | d | e | d/2rho cat / nav / phys | regime cat / nav / phys | four-flow cycles cat / nav / phys "
            "| configuration C strokes (unverified) | surrogate (catalog footprint, nearest cell) |\n|---|---|---|---|---|---|---|---|---|\n")
    lines = []
    for i, b in enumerate(batch, 1):
        fp, s = b["footprints"], b["surrogate_catalog_footprint"]
        tri = lambda key: " / ".join(str(fp[n][key]) for n in ("catalog", "nav2", "physical"))  # noqa: E731
        lines.append("| %d | `%s`%s | %.2f | %.2f | %s | %s | %s | %s | %s (A %s, B %s, gap %+.1f s; cell d=%.2f e=%.2f) |" % (
            i, b["course_id"], " (canary)" if b["canary"] else "", b["d_m"], b["e_m"], tri("d_over_2rho"),
            tri("regime"), tri("four_flow_cycles"), b["config_C_strokes_unverified"], s["winner"], s["success_A"],
            s["success_B"], s["mean_gap_s"], s["grid_d_cm"] / 100, s["grid_e_cm"] / 100))
    with open(os.path.join(HERE, "batch_v0.md"), "w", encoding="utf-8") as f:
        f.write("<!-- generated by select_batch.py; exploratory -->\n" + head + "\n".join(lines) + "\n")
    print(head + "\n".join(lines))


if __name__ == "__main__":
    main()
