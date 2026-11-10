"""Sensitivity of the independent pair to its own settings, on one slice of the pool (EXPLORATORY).

One door width (w = 1.00 m), the pool's 8 x 14 (e, d) grid, ONE seed (1000). For each variant of the
lattice settings: how many courses each arm completes, how many courses B' wins / A' wins by more
than 1 s, and the smallest bay depth at which each arm completes anything.

CLI:  sensitivity.py            (resumable; store ~/.cache/expchoice/learner_sensitivity_v1.jsonl)
"""
from __future__ import annotations

import json
import os
import sys
import time

_HERE = os.path.dirname(os.path.abspath(__file__))
for _p in (os.path.dirname(_HERE), _HERE):
    if _p not in sys.path:
        sys.path.insert(0, _p)

import course as C            # noqa: E402
import robots as R            # noqa: E402
import surrogate as S         # noqa: E402
import latticenav as LN       # noqa: E402
import truth as T             # noqa: E402

STORE = os.path.join(S.CACHE_DIR, "learner_sensitivity_v1.jsonl")
VARIANTS = {"default": {}, "pad_0": {"lattice_pad": 0.0}, "pad_0.05": {"lattice_pad": 0.05},
            "reverse_penalty_1": {"lattice_reverse_penalty": 1.0}, "recoveries_20": {"lattice_max_recoveries": 20}}
W_CM, SEED = 100, 1000


def run(budget_s=900.0, log=print):
    done = {}
    if os.path.exists(STORE):
        for line in open(STORE, encoding="utf-8"):
            try:
                r = json.loads(line)
                done[(r["variant"], r["e_cm"], r["d_cm"])] = r
            except ValueError:
                continue
    robot = R.load_robot("compact_diff")
    t0 = time.time()
    for name, cfg in VARIANTS.items():
        for e in T.POOL_E:
            for d in T.POOL_D:
                if (name, e, d) in done:
                    continue
                if time.time() - t0 > budget_s:
                    log("budget reached; rerun to resume")
                    return done
                c = C.make_course(W_CM / 100, e / 100, d / 100, t=T.T_CM / 100)
                g = S.paired_gap(c, robot, [SEED], cfg=cfg, arms=LN.INDEPENDENT_ARMS)
                r = {"variant": name, "e_cm": e, "d_cm": d, "J_A": g["J_A"][0], "J_B": g["J_B"][0],
                     "status_A": g["status_A"][0], "status_B": g["status_B"][0], "reversals_B": g["reversals_B"][0]}
                with open(STORE, "a", encoding="utf-8") as f:
                    f.write(json.dumps(r, sort_keys=True) + "\n")
                done[(name, e, d)] = r
    return done


def table(done):
    out = ["| variant | courses | A' completes | B' completes | B' wins (> 1 s) | A' wins (> 1 s) | both fail | smallest d completed, e >= 0.43 (A' / B') |",
           "|---|---|---|---|---|---|---|---|"]
    for name in VARIANTS:
        rows = [r for (v, _, _), r in done.items() if v == name]
        if not rows:
            continue
        ok_a = [r for r in rows if r["status_A"] == "success"]
        ok_b = [r for r in rows if r["status_B"] == "success"]
        gap = [r["J_A"] - r["J_B"] for r in rows]
        da = [r["d_cm"] for r in ok_a if r["e_cm"] >= 43]
        db = [r["d_cm"] for r in ok_b if r["e_cm"] >= 43]
        out.append("| %s | %d | %d | %d | %d | %d | %d | %s / %s |" % (
            name, len(rows), len(ok_a), len(ok_b), sum(g > 1 for g in gap), sum(g < -1 for g in gap),
            sum(r["status_A"] != "success" and r["status_B"] != "success" for r in rows),
            "%.2f" % (min(da) / 100) if da else "none", "%.2f" % (min(db) / 100) if db else "none"))
    return "\n".join(out)


if __name__ == "__main__":
    print(table(run()))
