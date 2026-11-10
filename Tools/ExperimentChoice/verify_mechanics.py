"""Numerical verdict table for the section-2 mechanics claims (EXPLORATORY).

    python3 -I verify_mechanics.py [--robot compact_diff] [--e-over-W 1.0]

Compares, for several slacks eps/W:
  * the lattice planner's minimum number of stretches (reversals + 1) for a sideways shift e;
  * the exact count ceil(e / Delta(d)) from mechanics.stretch_gain_exact;
  * the plan's first-order claim 2 W e / eps^2;
  * the best four-flow (exact cycles n, stretches 2n - 1) and its first-order form 4 W e / eps^2;
  * the same planner with the rotation centre offset from the footprint centre.
Writes a JSON copy to ~/.cache/expchoice/. Memory: lattices are capped by mechanics.MAX_TABLE_CELLS.
"""
from __future__ import annotations

import argparse
import json
import math
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

import mechanics as M   # noqa: E402
import robots as R      # noqa: E402

EPS_OVER_W = [0.01, 0.02, 0.05, 0.10, 0.20, 0.30, 0.38]


def table(robot_name="compact_diff", e_over_W=1.0, offset_frac=0.125, nx=401, ntheta=1201):
    rob = R.load_robot(robot_name)
    L, W = rob.L, rob.W
    e = e_over_W * W
    rows = []
    for x in EPS_OVER_W:
        eps = x * W
        d = L + eps
        if d >= 2 * M.rho(L, W):
            continue
        am = M.alpha_max(d, L, W)
        a_star, d_star = M.four_flow_best(d, L, W)
        Delta = M.stretch_gain_exact(d, L, W)
        n_exact = M.min_stretches_exact(e, d, L, W)
        plan = M.plan_min_reversals(e, d, L, W, nx=nx, ntheta=ntheta, x_start="rear")
        a = offset_frac * L
        nested = am <= math.atan(W / (L + 2 * a))
        plan_off = M.plan_min_reversals(e, d, L, W, a=a, nx=nx, ntheta=ntheta, x_start="rear")
        cyc = M.four_flow_cycles(e, d, L, W)
        lb1 = M.lower_bound_small(e, eps, W)
        rows.append({
            "eps_over_W": x, "d": d, "alpha_max": am,
            "alpha_max_over_first_order": am / M.alpha_max_small(eps, W),
            "alpha_star_over_first_order": a_star / M.alpha_star_small(eps, W),
            "delta_star_over_first_order": d_star / M.delta_star_small(eps, W),
            "Delta_over_first_order": Delta / (eps ** 2 / (2 * W)),
            "planner_stretches": plan["stretches"], "exact_stretches": n_exact,
            "first_order_bound_2We_eps2": lb1,
            "planner_over_first_order_bound": plan["stretches"] / lb1,
            "first_order_bound_violated": bool(plan["stretches"] < math.ceil(lb1 - 1e-9)),
            "four_flow_cycles": cyc, "four_flow_stretches": 2 * cyc - 1,
            "four_flow_first_order_4We_eps2": M.four_flow_cycles_small(e, eps, W),
            "four_flow_stretches_over_planner": (2 * cyc - 1) / plan["stretches"],
            "offset_a": a, "offset_nested": bool(nested), "planner_stretches_offset": plan_off["stretches"],
        })
    return {"label": "exploratory", "robot": robot_name, "L": L, "W": W, "e": e, "lattice": [nx, ntheta], "rows": rows}


def render(t):
    out = ["EXPLORATORY mechanics check: robot %s (L=%.2f, W=%.2f), shift e=%.3f m, lattice %dx%d, rear start"
           % (t["robot"], t["L"], t["W"], t["e"], t["lattice"][0], t["lattice"][1]),
           "counts are STRETCHES (reversals + 1); a four-flow of n cycles is 2n - 1 stretches",
           "eps/W | planner  exact  2We/eps^2  planner/claim | 4-flow cyc  4We/eps^2  4flow-str/planner | "
           "offset a=%.2f | amax/(eps/W)  d*/(eps^2/4W)  D/(eps^2/2W)" % t["rows"][0]["offset_a"]]
    for r in t["rows"]:
        out.append("%5.2f | %7d %6d %10.1f %10.3f%s | %10d %10.1f %12.2f      | %8d%s  | %10.3f %13.3f %13.3f" % (
            r["eps_over_W"], r["planner_stretches"], r["exact_stretches"], r["first_order_bound_2We_eps2"],
            r["planner_over_first_order_bound"], "*" if r["first_order_bound_violated"] else " ",
            r["four_flow_cycles"], r["four_flow_first_order_4We_eps2"], r["four_flow_stretches_over_planner"],
            r["planner_stretches_offset"], " " if r["offset_nested"] else "!",
            r["alpha_max_over_first_order"], r["delta_star_over_first_order"], r["Delta_over_first_order"]))
    out.append("* planner uses fewer stretches than ceil(2We/eps^2): the first-order claim is not a bound there")
    out.append("! offset large enough that the admissible depth intervals are no longer nested")
    return "\n".join(out)


def main(argv=None):
    ap = argparse.ArgumentParser()
    ap.add_argument("--robot", default="compact_diff")
    ap.add_argument("--e-over-W", type=float, default=1.0)
    a = ap.parse_args(argv)
    t = table(a.robot, a.e_over_W)
    print(render(t))
    out_dir = os.path.expanduser("~/.cache/expchoice")
    os.makedirs(out_dir, exist_ok=True)
    path = os.path.join(out_dir, "mechanics_check_%s.json" % a.robot)
    with open(path, "w", encoding="utf-8") as f:
        json.dump(t, f, indent=1, sort_keys=True)
    print("wrote", path)
    return 0


if __name__ == "__main__":
    sys.exit(main())
