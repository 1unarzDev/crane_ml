# SPDX-License-Identifier: GPL-3.0-or-later
"""Failure-map sweep: speed limit x thin-branch density x latency tail x sensor, on whole DEV layouts.

  fsrun python Tools/NavPilot/sweep.py run --shard 0 --nshards 3 --layouts 8
  fsrun python Tools/NavPilot/sweep.py report            # -> Tools/NavPilot/results/nav_failure_map.md
  fsrun python Tools/NavPilot/sweep.py episode --seed 3 --sensor lidar --v-limit 7 --branch high --log /tmp/run

Splits (Tools/DroneData/splits.json): --split dev (seeds 0..), val (70000..), heldout (90000.., refused unless
--allow-heldout). The pilot only ever runs DEV; per splits.json policy DEV numbers are for iteration, not reporting.
"""
import argparse
import glob
import itertools
import json
import math
import os
import sys
import time

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)

from navpilot import sensors as S  # noqa: E402
from navpilot.episode import default_config, run_episode  # noqa: E402
from navpilot.forest import World, family_spec, generate_stand, guard_seed  # noqa: E402

SPLIT_BASE = {"dev": 0, "train": 2000, "val": 70000, "test": 80000, "heldout": 90000}
AXES = {"v_limit": [3.0, 5.0, 7.0, 10.0], "branch": ["none", "low", "mid", "high"],
        "latency_tail": ["nominal", "heavy", "severe"], "sensor": ["stereo", "lidar", "fusion"]}
RAW = os.path.join(HERE, "results", "raw")
CAUSES = ["thin_branch_miss", "latency_stall", "map_lag", "occlusion", "local_minimum", "tracking_lag",
          "sensor_miss", "detection_miss"]


def layout_seeds(split, n, allow_heldout=False, offset=0):
    base = SPLIT_BASE[split]
    seeds = [base + offset + i for i in range(n)]
    for s in seeds:
        guard_seed(s, allow_heldout=allow_heldout)
    return seeds


def cells():
    return list(itertools.product(AXES["branch"], AXES["v_limit"], AXES["latency_tail"], AXES["sensor"]))


def run(args):
    os.makedirs(RAW, exist_ok=True)
    seeds = layout_seeds(args.split, args.layouts, args.allow_heldout)
    jobs = [(s, c) for s in seeds for c in cells()]
    jobs = jobs[args.shard::args.nshards]
    out = os.path.join(RAW, "%s_shard%d.jsonl" % (args.split, args.shard))
    worlds = {}
    t0 = time.time()
    n = 0
    with open(out, "w") as f:
        for seed, (branch, vl, tail, sensor) in jobs:
            key = (seed, branch)
            if key not in worlds:
                worlds = {key: World(generate_stand(seed, family_spec(stem=args.stem, branch=branch),
                                                    wind=args.wind, allow_heldout=args.allow_heldout))}
            cfg = default_config(sensor=sensor, v_limit=vl, latency_tail=tail, wind=args.wind)
            te = time.time()
            r = run_episode(worlds[key], cfg, seed)
            r.update({"seed": seed, "branch": branch, "v_limit": vl, "latency_tail": tail, "sensor": sensor,
                      "stem": args.stem, "wall_s": time.time() - te})
            f.write(json.dumps(r) + "\n")
            n += 1
    dt = time.time() - t0
    print("shard %d: %d episodes in %.1fs (%.2f ep/s)" % (args.shard, n, dt, n / max(dt, 1e-9)))


def _load(split):
    rows = []
    for p in sorted(glob.glob(os.path.join(RAW, "%s_shard*.jsonl" % split))):
        with open(p) as f:
            rows += [json.loads(l) for l in f if l.strip()]
    return rows


def _agg(rs):
    n = len(rs)
    succ = sum(r["success"] for r in rs) / n
    col = sum(r["collision"] for r in rs) / n
    stuck = sum(r["outcome"] == "stuck" for r in rs) / n
    cs = [r["contact_speed"] for r in rs if r["contact_speed"] is not None]
    causes = {}
    for r in rs:
        if r["cause"]:
            causes[r["cause"]] = causes.get(r["cause"], 0) + 1
    fails = {k: v for k, v in causes.items() if k != "detection_miss"}
    dom = max(fails.items(), key=lambda kv: kv[1])[0] if fails else "-"
    rec = [r["recall"] for r in rs if r["recall"] == r["recall"]]
    return {"n": n, "succ": succ, "col": col, "stuck": stuck, "cspeed": sum(cs) / len(cs) if cs else float("nan"),
            "speed": sum(r["mean_speed"] for r in rs) / n, "near": sorted(r["near_miss_m"] for r in rs)[n // 2],
            "mviol": sum(r["margin_viol_frac"] for r in rs) / n,
            "dl": sum(r["deadline_misses"] / max(r["cycles"], 1) for r in rs) / n,
            "age": sum(r["age_gate_misses"] / max(r["cycles"], 1) for r in rs) / n,
            "recall": sum(rec) / len(rec) if rec else float("nan"), "causes": causes, "dom": dom,
            "ttg": [r["time_to_goal_s"] for r in rs if r["time_to_goal_s"]], "bviol": sum(r["belief_violations"] for r in rs),
            "wall": sum(r["wall_s"] for r in rs)}


def _group(rows, keys):
    g = {}
    for r in rows:
        g.setdefault(tuple(r[k] for k in keys), []).append(r)
    return g


def _causes_str(c, n):
    return ", ".join("%s %d" % (k, v) for k, v in sorted(c.items(), key=lambda kv: -kv[1])) or "-"


def report(args):
    rows = _load(args.split)
    if not rows:
        sys.exit("no raw results for split %s" % args.split)
    A = _agg(rows)
    L = []
    w = L.append
    seeds = sorted({r["seed"] for r in rows})
    w("# Navigation pilot: conventional-baseline failure map (DEV layouts)\n")
    w("_Generated by `Tools/NavPilot/sweep.py report`. Evidence label: IMPLEMENTED pilot (2.5D point mass, synthetic "
      "sensors, synthetic forest; not LIVE_SIM_VALIDATED, not PHYSICALLY_CALIBRATED). DEV seeds %d-%d, which "
      "splits.json reserves for iteration: these numbers locate failures, they are not reportable results._\n"
      % (seeds[0], seeds[-1]))
    w("**Setup.** %d episodes = %d whole DEV layouts x %d cells (branch density %s x speed limit %s m/s x latency tail "
      "%s x sensor %s). Stem density `%s` (800 stems/ha), 60 m corridor, wind `light_breeze`, 6 person targets per "
      "layout. Same layout seed = same run seed across cells (common random numbers). Baseline: log-odds 2.5D map, "
      "inflation r_body + 0.10 m, 48 headings x 5 speeds forward-simulated through the tracking loop, stopping rule "
      "d_free >= v tau + v^2/2a + 0.5 m with tau = 0.10 s and a = %.1f m/s^2 (0.8 x tilt-limited), unknown = blocked.\n"
      % (len(rows), len(seeds), len(cells()), AXES["branch"], AXES["v_limit"], AXES["latency_tail"], AXES["sensor"],
         rows[0]["stem"], 0.8 * 9.80665 * math.tan(math.radians(40))))
    q = {t: S.latency_quantiles(t) for t in AXES["latency_tail"]}
    w("Compute-time tails per planner cycle (period 50 ms): " + "; ".join(
        "`%s` p50 %.0f / p95 %.0f / p99 %.0f / p99.9 %.0f ms" % (t, 1e3 * v["p50"], 1e3 * v["p95"], 1e3 * v["p99"],
                                                                1e3 * v["p999"]) for t, v in q.items()) + ".\n")
    w("Throughput: %.2f episodes/s per process (%.1f s total wall in episodes); belief stopping-rule violations: %d.\n"
      % (len(rows) / A["wall"], A["wall"], A["bviol"]))
    w("Overall: success %.0f%%, collision %.0f%%, stuck %.0f%%. Causes: %s.\n"
      % (100 * A["succ"], 100 * A["col"], 100 * A["stuck"], _causes_str(A["causes"], A["n"])))

    w("\n## 1. Failure map: success % (collision %) [dominant failure cause]\n")
    w("Rows: branch density x speed limit. Columns: sensor / latency tail. Cause codes: T thin_branch_miss, "
      "S latency_stall, M map_lag, O occlusion, L local_minimum, K tracking_lag, X sensor_miss.\n")
    code = {"thin_branch_miss": "T", "latency_stall": "S", "map_lag": "M", "occlusion": "O", "local_minimum": "L",
            "tracking_lag": "K", "sensor_miss": "X", "-": "-"}
    cols = [(s, t) for s in AXES["sensor"] for t in AXES["latency_tail"]]
    w("| branch | v_lim | " + " | ".join("%s/%s" % (s, t[:3]) for s, t in cols) + " |")
    w("|---|---|" + "---|" * len(cols))
    g = _group(rows, ["branch", "v_limit", "sensor", "latency_tail"])
    for b in AXES["branch"]:
        for v in AXES["v_limit"]:
            cells_ = []
            for s, t in cols:
                rs = g.get((b, v, s, t))
                if not rs:
                    cells_.append("")
                    continue
                a = _agg(rs)
                cells_.append("%d (%d) %s" % (round(100 * a["succ"]), round(100 * a["col"]), code[a["dom"]]))
            w("| %s | %g | " % (b, v) + " | ".join(cells_) + " |")

    for title, keys in [("2. By sensor x branch density", ["sensor", "branch"]),
                        ("3. By speed limit x latency tail", ["v_limit", "latency_tail"]),
                        ("4. By sensor x speed limit", ["sensor", "v_limit"])]:
        w("\n## %s\n" % title)
        w("| %s | n | success | collision | stuck | contact v | mean v | median near-miss m | margin-viol ticks | "
          "deadline miss | age>120ms | recall | causes |" % " | ".join(keys))
        w("|" + "---|" * (len(keys) + 12))
        for k, rs in sorted(_group(rows, keys).items(), key=lambda kv: tuple(
                AXES[kk].index(vv) if kk in AXES else vv for kk, vv in zip(keys, kv[0]))):
            a = _agg(rs)
            w("| " + " | ".join(str(x) for x in k) + " | %d | %.0f%% | %.0f%% | %.0f%% | %.1f | %.1f | %.2f | %.1f%% | "
              "%.1f%% | %.0f%% | %.2f | %s |" % (a["n"], 100 * a["succ"], 100 * a["col"], 100 * a["stuck"], a["cspeed"],
                                                a["speed"], a["near"], 100 * a["mviol"], 100 * a["dl"], 100 * a["age"],
                                                a["recall"], _causes_str(a["causes"], a["n"])))

    w("\n## 5. Collision causes by struck-object type and contact speed\n")
    w("| cause | n | stem | branch | mean contact v | mean struck diam cm |")
    w("|---|---|---|---|---|---|")
    cg = _group([r for r in rows if r["collision"]], ["cause"])
    for k, rs in sorted(cg.items(), key=lambda kv: -len(kv[1])):
        w("| %s | %d | %d | %d | %.1f | %.1f |" % (k[0], len(rs), sum(r["contact_kind"] == "stem" for r in rs),
                                               sum(r["contact_kind"] == "branch" for r in rs),
                                               sum(r["contact_speed"] for r in rs) / len(rs),
                                               100 * sum(r["contact_diam_m"] for r in rs) / len(rs)))
    w("\n## 6. Person-target misses (targets passed within 15 m, not confirmed by 2 detections)\n")
    mr = {}
    for r in rows:
        for k, v in r["target_miss_reasons"].items():
            mr.setdefault((r["v_limit"], k), 0)
            mr[(r["v_limit"], k)] += v
    reasons = sorted({k for _, k in mr})
    w("| v_lim | targets along | recall | " + " | ".join(reasons) + " |")
    w("|---|---|---|" + "---|" * len(reasons))
    for v in AXES["v_limit"]:
        rs = [r for r in rows if r["v_limit"] == v]
        ta = sum(r["targets_along"] for r in rs)
        rec = [r["recall"] for r in rs if r["recall"] == r["recall"]]
        w("| %g | %d | %.2f | " % (v, ta, sum(rec) / max(len(rec), 1)) + " | ".join(str(mr.get((v, k), 0)) for k in reasons) + " |")
    if args.notes and os.path.exists(args.notes):
        with open(args.notes) as f:
            w("\n" + f.read())
    out = os.path.join(HERE, "results", "nav_failure_map.md")
    with open(out, "w") as f:
        f.write("\n".join(L) + "\n")
    print(out)


def episode(args):
    st = generate_stand(args.seed, family_spec(stem=args.stem, branch=args.branch), wind=args.wind,
                        allow_heldout=args.allow_heldout)
    cfg = default_config(sensor=args.sensor, v_limit=args.v_limit, latency_tail=args.latency_tail, wind=args.wind)
    t0 = time.time()
    r = run_episode(World(st), cfg, args.seed, log_dir=args.log)
    r["wall_s"] = time.time() - t0
    print(json.dumps(r, indent=1))


def main(argv=None):
    ap = argparse.ArgumentParser()
    ap.add_argument("cmd", choices=["run", "report", "episode"])
    ap.add_argument("--split", default="dev", choices=list(SPLIT_BASE))
    ap.add_argument("--allow-heldout", action="store_true")
    ap.add_argument("--layouts", type=int, default=8)
    ap.add_argument("--shard", type=int, default=0)
    ap.add_argument("--nshards", type=int, default=1)
    ap.add_argument("--stem", default="medium")
    ap.add_argument("--wind", default="light_breeze")
    ap.add_argument("--notes", default=os.path.join(HERE, "results", "nav_failure_map_notes.md.in"))
    ap.add_argument("--seed", type=int, default=0)
    ap.add_argument("--branch", default="mid")
    ap.add_argument("--sensor", default="stereo")
    ap.add_argument("--v-limit", type=float, default=5.0)
    ap.add_argument("--latency-tail", default="nominal")
    ap.add_argument("--log", default=None)
    a = ap.parse_args(argv)
    {"run": run, "report": report, "episode": episode}[a.cmd](a)


if __name__ == "__main__":
    main()
