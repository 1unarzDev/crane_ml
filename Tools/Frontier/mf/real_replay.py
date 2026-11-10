# SPDX-License-Identifier: GPL-3.0-or-later
"""Replay of the REAL paired logs: 7 candidate speed limits for map_rhp on forest 1337 (cross_0..9, run seed 1).

Config = speed limit (2, 2.5, 3, 4, 5, 6, 8 m/s). Cheap tier L1 = NavPilot 2.5D twin (10 runs per speed, ops stage 2); expensive tier
L2 = CRANE .NET harness (10 matching runs per speed, M5 / sweep rows). Utility per run: y = mean_speed * progress_frac if no collision, else -8.
Costs: measured per-run medians from the logs (twin 4.5 s, .NET 48 s). The L2 'truth' is the pool mean, so the table is in-sample:
it measures how fast each selector would have found the speed that the logged L2 data call best, not generalisation.

    python real_replay.py [--reps 300]
"""
import argparse
import os
import sys

import numpy as np

sys.dont_write_bytecode = True
HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
sys.path.insert(0, os.path.normpath(os.path.join(HERE, "..")))
import data  # noqa: E402
import replay  # noqa: E402
import synth  # noqa: E402

C = 8.0
SPEEDS = [2.0, 2.5, 3.0, 4.0, 5.0, 6.0, 8.0]


class RealTable:
    def __init__(self, tw, l2):
        self.cfgs = [("map_rhp", v, 0) for v in SPEEDS]
        self.N = len(SPEEDS)
        z = (np.array(SPEEDS) - 4.5) / 2.5
        self.features = np.stack([np.ones(self.N), z, z * z], 1)
        self.pools = {}
        for i, v in enumerate(SPEEDS):
            for fid, rows in (("L1", tw), ("L2", l2)):
                rr = sorted((r for r in rows if r["speed"] == v and r["scenario"].startswith("cross_") and r["seed"] == 1), key=lambda r: r["scenario"])
                self.pools[(i, fid)] = np.array([-C if r["collision"] else (r["mean_speed"] or 0.0) * (r["progress"] or 0.0) for r in rr])
        self.f1 = np.array([self.pools[(i, "L1")].mean() for i in range(self.N)])
        self.f2 = np.array([self.pools[(i, "L2")].mean() for i in range(self.N)])


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--reps", type=int, default=300)
    a = ap.parse_args()
    tab = RealTable(data.navpilot_twin(), data.crane_dotnet())
    print("speed  n1 n2   f1(twin)  f2(.NET)")
    for i, v in enumerate(SPEEDS):
        print("%4.1f  %2d %2d  %8.2f  %8.2f" % (v, len(tab.pools[(i, 'L1')]), len(tab.pools[(i, 'L2')]), tab.f1[i], tab.f2[i]))
    print("argmax L1 (twin at face value): %.1f m/s; argmax L2 (.NET): %.1f m/s; regret of face value = %.2f; tau(f1,f2) over speeds = %.2f"
          % (SPEEDS[int(np.argmax(tab.f1))], SPEEDS[int(np.argmax(tab.f2))], tab.f2.max() - tab.f2[np.argmax(tab.f1)], synth.kendall(tab.f1, tab.f2)))
    cost = {"L1": 4.5, "L2": 48.2}
    grid = [0.1, 0.25, 0.5, 1.0]
    print("\nmean regret at L2 (in-sample truth) vs compute-hours, %d replays of query order; budget 1 h (= 75 L2 runs)" % a.reps)
    print("selector        " + "  ".join("%4.2fh" % h for h in grid) + "   P(find L2 best by 1h)")
    for name in ("random", "all-high", "low-only", "sh-seeds-L2", "succ-halving", "screen+topk", "AR1-BLR"):
        kw = {"rungs": ((replay.LO, 3), (replay.LO, 10), (replay.HI, 2), (replay.HI, 4), (replay.HI, 10)), "eta": 2} if name == "succ-halving" else {}
        if name == "screen+topk":
            kw = {"m": 5, "k": 3}
        if name == "AR1-BLR":
            kw = {"init_lo": 2, "init_hi": 3, "m_lo": 3, "q": 2}
        R, hit = [], []
        for r in range(a.reps):
            env = replay.Env(tab, cost, np.random.default_rng([r, 1]))
            sel = replay.make(name, tab.N, np.random.default_rng([r, 2]), tab, **kw)
            out, _ = replay.run(sel, env, 3600, grid)
            R.append([out[h] for h in grid])
            hit.append(out[grid[-1]] < 1e-9)
        print("%-15s " % name + "  ".join("%5.2f" % x for x in np.mean(R, 0)) + "   %.2f" % np.mean(hit))


if __name__ == "__main__":
    main()
