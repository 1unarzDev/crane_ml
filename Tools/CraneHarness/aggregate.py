# SPDX-License-Identifier: GPL-3.0-or-later
"""Aggregate CRANE harness results with a paired cluster bootstrap.

  python aggregate.py DIR [DIR...] [--by speed] [--pair speed 4 6] [--md OUT.md]

Rows: results_shard*.jsonl written by `CraneExperiment run` (one row per run). Clusters are (forest seed, mission):
runs of the same route are not independent, so the bootstrap resamples whole clusters with replacement. Pairing uses
`pair_key` (forest, mission, run seed, DR sample): two conditions are compared only on keys present in both, and a
resampled cluster contributes all of its pairs (paired cluster bootstrap). Intervals are percentile 95 % CIs, B = 4000,
fixed seed. Wilson intervals are also printed for reference (they ignore clustering).
"""
import argparse
import glob
import json
import math
import os
import random
import sys
from collections import Counter, defaultdict

LABELS = ["thin_branch_miss", "sensor_miss", "occlusion", "map_lag", "latency_stall", "estimator_drift",
          "geometry_graze", "brake_infeasible", "tracking_lag", "local_minimum", "comms", "other", "unlabelled"]


def load(paths):
    rows = []
    for p in paths:
        files = sorted(glob.glob(os.path.join(p, "results*.jsonl"))) if os.path.isdir(p) else [p]
        for f in files:
            for line in open(f):
                line = line.strip()
                if line:
                    rows.append(json.loads(line))
    return rows


def wilson(k, n, z=1.96):
    if n == 0:
        return (float("nan"), float("nan"))
    p = k / n
    d = 1 + z * z / n
    c = (p + z * z / (2 * n)) / d
    h = z * math.sqrt(p * (1 - p) / n + z * z / (4 * n * n)) / d
    return (max(0.0, c - h), min(1.0, c + h))


def _q(xs, p):
    xs = sorted(xs)
    if not xs:
        return float("nan")
    k = p * (len(xs) - 1)
    i = int(math.floor(k))
    return xs[i] if i + 1 >= len(xs) else xs[i] * (1 - (k - i)) + xs[i + 1] * (k - i)


def cluster_bootstrap(rows, stat, B=4000, seed=12345, cluster="cluster"):
    """Percentile CI of stat(rows) resampling clusters with replacement."""
    by = defaultdict(list)
    for r in rows:
        by[r[cluster]].append(r)
    keys = sorted(by)
    if not keys:
        return float("nan"), (float("nan"), float("nan"))
    rng = random.Random(seed)
    est = stat(rows)
    bs = []
    for _ in range(B):
        sample = []
        for _k in keys:
            sample.extend(by[keys[rng.randrange(len(keys))]])
        v = stat(sample)
        if v == v:
            bs.append(v)
    return est, (_q(bs, 0.025), _q(bs, 0.975))


def paired_cluster_bootstrap(rows_a, rows_b, metric, B=4000, seed=12345):
    """Mean paired difference metric(b) - metric(a) over pair_keys present in both, clusters resampled."""
    a = {r["pair_key"]: r for r in rows_a}
    b = {r["pair_key"]: r for r in rows_b}
    pairs = defaultdict(list)
    for k in sorted(set(a) & set(b)):
        pairs[a[k]["cluster"]].append(metric(b[k]) - metric(a[k]))
    keys = sorted(pairs)
    if not keys:
        return float("nan"), (float("nan"), float("nan")), 0
    flat = [d for k in keys for d in pairs[k]]
    est = sum(flat) / len(flat)
    rng = random.Random(seed)
    bs = []
    for _ in range(B):
        s = []
        for _k in keys:
            s.extend(pairs[keys[rng.randrange(len(keys))]])
        bs.append(sum(s) / len(s))
    return est, (_q(bs, 0.025), _q(bs, 0.975)), len(flat)


def outcome(r):
    return r.get("outcome") or r.get("Outcome")


def is_success(r):
    return 1.0 if outcome(r) == "success" else 0.0


def is_collision(r):
    return 1.0 if outcome(r) == "collision" else 0.0


def rate(f):
    return lambda rs: sum(f(r) for r in rs) / len(rs) if rs else float("nan")


def fmt_ci(est, ci, pct=True):
    if pct:
        return "%.2f [%.2f, %.2f]" % (est, ci[0], ci[1])
    return "%.3f [%.3f, %.3f]" % (est, ci[0], ci[1])


def table(rows, by):
    groups = defaultdict(list)
    for r in rows:
        groups[r.get(by)].append(r)
    out = ["| %s | n | clusters | success (cluster-boot 95%% CI) | collision (cluster-boot 95%% CI) | Wilson success | labels |" % by,
           "|---|---|---|---|---|---|---|"]
    for g in sorted(groups, key=lambda x: (str(type(x)), x)):
        rs = groups[g]
        s = cluster_bootstrap(rs, rate(is_success))
        c = cluster_bootstrap(rs, rate(is_collision))
        k = int(sum(is_success(r) for r in rs))
        w = wilson(k, len(rs))
        lab = Counter(r.get("label", "") for r in rs if outcome(r) != "success")
        out.append("| %s | %d | %d | %s | %s | %.2f-%.2f | %s |" % (
            g, len(rs), len({r["cluster"] for r in rs}), fmt_ci(*s), fmt_ci(*c), w[0], w[1],
            ", ".join("%s %d" % (l, lab[l]) for l in sorted(lab)) or "-"))
    return "\n".join(out)


def label_table(rows):
    fails = [r for r in rows if outcome(r) != "success"]
    out = ["| run | outcome | label | t [s] | struck | D [cm] | evidence |", "|---|---|---|---|---|---|---|"]
    for r in sorted(fails, key=lambda r: r["run_id"]):
        ev = r.get("label_evidence") or {}
        d = ev.get("struck_diam_m")
        out.append("| %s | %s | %s | %.2f | %s | %s | %s |" % (
            r["run_id"], outcome(r), r.get("label", ""), (r.get("end_tick") or 0) * 0.005, r.get("r_CollisionObject", "") or "-",
            "%.1f" % (100 * d) if d else "-", (r.get("label_detail") or "")[:160]))
    return "\n".join(out)


def main(argv=None):
    ap = argparse.ArgumentParser()
    ap.add_argument("paths", nargs="+")
    ap.add_argument("--by", default="speed")
    ap.add_argument("--pair", nargs=3, metavar=("FIELD", "A", "B"))
    ap.add_argument("--md")
    a = ap.parse_args(argv)
    rows = load(a.paths)
    parts = ["## Outcomes by %s (%d runs)" % (a.by, len(rows)), "", table(rows, a.by), "", "## Failures and labels", "", label_table(rows)]
    if a.pair:
        f, va, vb = a.pair
        conv = (lambda x: x) if not all(isinstance(r.get(f), (int, float)) for r in rows) else float
        ra = [r for r in rows if r.get(f) == conv(va)]
        rb = [r for r in rows if r.get(f) == conv(vb)]
        for name, m in (("success", is_success), ("collision", is_collision)):
            est, ci, n = paired_cluster_bootstrap(ra, rb, m)
            parts += ["", "Paired %s difference (%s=%s minus %s=%s, %d pairs): %s" % (name, f, vb, f, va, n, fmt_ci(est, ci))]
    md = "\n".join(parts)
    print(md)
    if a.md:
        with open(a.md, "w") as fo:
            fo.write(md + "\n")


if __name__ == "__main__":
    sys.exit(main())
