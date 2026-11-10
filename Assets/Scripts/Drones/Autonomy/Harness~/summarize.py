# SPDX-License-Identifier: GPL-3.0-or-later
"""Summarize autonomy results (.NET harness results.jsonl or Unity summary.jsonl) into markdown tables.
   python summarize.py FILE [FILE...] [--by VLimit] [--runs]"""
import json
import math
import sys


def load(paths):
    rows = []
    for p in paths:
        for line in open(p):
            line = line.strip()
            if line:
                rows.append(json.loads(line))
    return rows


def q(xs, p):
    xs = sorted(x for x in xs if x is not None and not (isinstance(x, float) and math.isnan(x)))
    if not xs:
        return float("nan")
    k = p * (len(xs) - 1)
    i = int(math.floor(k))
    return xs[i] if i + 1 >= len(xs) else xs[i] * (1 - (k - i)) + xs[i + 1] * (k - i)


def wilson(k, n, z=1.96):
    if n == 0:
        return (float("nan"), float("nan"))
    p = k / n
    d = 1 + z * z / n
    c = (p + z * z / (2 * n)) / d
    h = z * math.sqrt(p * (1 - p) / n + z * z / (4 * n * n)) / d
    return (max(0.0, c - h), min(1.0, c + h))


def group_table(rows, key):
    out = ["| %s | n | success | 95%% CI | collisions | stuck/timeout | progress mean | min hard clearance (min / p10) [m] | twig contacts / run | mean speed [m/s] | age p50 / p99 / max [ms] | no-safe stops / run | watchdog / run | margin viol. [%%] |" % key,
           "|---|---|---|---|---|---|---|---|---|---|---|---|---|---|"]
    keys = sorted({r[key] for r in rows})
    for k in keys:
        g = [r for r in rows if r[key] == k]
        n = len(g)
        s = sum(1 for r in g if r["Success"])
        col = sum(1 for r in g if r["Collision"])
        st = sum(1 for r in g if r["Outcome"] in ("stuck", "timeout"))
        lo, hi = wilson(s, n)
        mc = [r["MinClearanceHardM"] for r in g if r["MinClearanceHardM"] is not None]
        mt = sum(r["MarginTicks"] for r in g)
        mv = sum(r["MarginViolations"] for r in g)
        out.append("| %s | %d | %d/%d | %.2f-%.2f | %d | %d | %.2f | %.2f / %.2f | %.1f | %.2f | %.0f / %.0f / %.0f | %.1f | %.1f | %.2f |" % (
            k, n, s, n, lo, hi, col, st, sum(r["ProgressFrac"] for r in g) / n, min(mc) if mc else float("nan"), q(mc, 0.1),
            sum(r["TwigContacts"] for r in g) / n, sum(r["MeanSpeed"] for r in g) / n,
            1000 * q([r["AgeP50S"] for r in g], 0.5), 1000 * max(r["AgeP99S"] for r in g), 1000 * max(r["AgeMaxS"] for r in g),
            sum(r["NoSafeStops"] for r in g) / n, sum(r["WatchdogTrips"] for r in g) / n, 100.0 * mv / max(mt, 1)))
    return "\n".join(out)


def run_table(rows):
    out = ["| mission | v_lim | seed | outcome | time [s] | progress | min hard clr [m] | mean v | age p99 [ms] | no-safe | wd | recov | note |",
           "|---|---|---|---|---|---|---|---|---|---|---|---|---|"]
    for r in rows:
        out.append("| %s | %.1f | %d | %s | %.1f | %.2f | %.2f | %.2f | %.0f | %d | %d | %d | %s |" % (
            r["Name"], r["VLimit"], int(r["RunSeed"]), r["Outcome"], r["TimeS"], r["ProgressFrac"], r["MinClearanceHardM"] if r["MinClearanceHardM"] is not None else float("nan"),
            r["MeanSpeed"], 1000 * r["AgeP99S"], r["NoSafeStops"], r["WatchdogTrips"], r["Recoveries"], r["CollisionObject"] or r["FailureCause"]))
    return "\n".join(out)


if __name__ == "__main__":
    args = [a for a in sys.argv[1:] if not a.startswith("--")]
    key = "VLimit"
    if "--by" in sys.argv:
        key = sys.argv[sys.argv.index("--by") + 1]
        args = [a for a in args if a != key]
    rows = load(args)
    print(group_table(rows, key))
    if "--runs" in sys.argv:
        print()
        print(run_table(rows))
