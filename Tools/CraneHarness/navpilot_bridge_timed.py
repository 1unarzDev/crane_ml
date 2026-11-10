# SPDX-License-Identifier: GPL-3.0-or-later
"""Timing wrapper around navpilot_bridge.py (original untouched): adds wall_s and cpu_s to every row and appends a
cost line {sim, fidelity, config, seed, wall_s, cpu_s} to Tools/Frontier/costs.jsonl. Same CLI as navpilot_bridge."""
import json
import os
import sys
import time

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
import navpilot_bridge as nb  # noqa: E402

COSTS = os.path.normpath(os.path.join(HERE, "..", "Frontier", "costs.jsonl"))
_orig = nb.run


def timed(st, v, s, *a, **k):
    w0, c0 = time.perf_counter(), time.process_time()
    r = _orig(st, v, s, *a, **k)
    r["wall_s"], r["cpu_s"] = time.perf_counter() - w0, time.process_time() - c0
    with open(COSTS, "a") as f:
        f.write(json.dumps({"kind": "run", "sim": "navpilot_2p5d_twin", "fidelity": "low", "config": "v%g|f%s" % (v, st.get("seed")),
                            "seed": int(s), "wall_s": round(r["wall_s"], 3), "cpu_s": round(r["cpu_s"], 3)}) + "\n")
    return r


nb.run = timed
if __name__ == "__main__":
    nb.main()
