#!/usr/bin/env python3
"""Compare two or more run directories of the same seed/scene/trajectory.

Exit 0 if every run matches the first within tolerance. Reports bitwise equality, the maximum absolute
deviation per column group, and the time of first divergence. Wall clock, run id and Unity paths are
ignored; the sim-clock streams (t_ns) must match EXACTLY (integers), other logs are checked per kind.
"""
import argparse
import hashlib
import json
import math
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from drone_log import DroneLog  # noqa: E402

IGNORED_MANIFEST = {"run_id", "wall_clock_start_utc", "end"}
# Documented tolerances (Docs/DroneVisuals/LOG_FORMAT.md, "Reproducibility"); overridden by --tol-*.
GROUPS = {
    "pos": (["px", "py", "pz"], 1e-6),       # m
    "quat": (["qw", "qx", "qy", "qz"], 1e-6),
    "vel": (["vx", "vy", "vz"], 1e-6),        # m/s
    "rate": (["wx", "wy", "wz"], 1e-5),       # rad/s
    "acc": (["ax", "ay", "az"], 1e-2),        # m/s^2 (finite difference at 1 kHz amplifies by 1000)
    "rotor": (["rpm%d" % i for i in range(16)] + ["cmd%d" % i for i in range(16)], 1e-3),
    "batt": (["batt_v", "batt_a", "batt_soc"], 1e-6),
}


def sha(path):
    return hashlib.sha256(open(path, "rb").read()).hexdigest()


def manifest_diff(a, b):
    ka = {k: v for k, v in a.items() if k not in IGNORED_MANIFEST}
    kb = {k: v for k, v in b.items() if k not in IGNORED_MANIFEST}
    return [k for k in sorted(set(ka) | set(kb)) if ka.get(k) != kb.get(k)]


def compare(a_dir, b_dir, tol):
    A, B = DroneLog(a_dir), DroneLog(b_dir)
    rep = {"a": a_dir, "b": b_dir, "problems": [], "groups": {}}
    md = manifest_diff(A.manifest, B.manifest)
    if md:
        rep["problems"].append("manifest differs in: %s" % md)
    if A.state["t_ns"] != B.state["t_ns"]:
        rep["problems"].append("state time bases differ (lengths %d vs %d)" % (len(A.state["t_ns"]), len(B.state["t_ns"])))
        rep["bitwise_state_equal"] = False
        return rep
    bitwise = True
    first_div = None
    tmax_all = 0.0
    for g, (cols, _) in GROUPS.items():
        worst, t_at = 0.0, None
        for c in cols:
            if c not in A.state:
                continue
            for i, (x, y) in enumerate(zip(A.state[c], B.state[c])):
                if x == y or (math.isnan(x) and math.isnan(y)):
                    continue
                bitwise = False
                d = abs(x - y)
                if first_div is None or A.state["t_ns"][i] < first_div:
                    first_div = A.state["t_ns"][i]
                if d > worst:
                    worst, t_at = d, A.state["t_ns"][i]
        limit = tol[g]
        rep["groups"][g] = {"max_abs": worst, "at_t_ns": t_at, "tol": limit, "ok": worst <= limit}
        if worst > limit:
            rep["problems"].append("%s deviation %.3e > %.3e (at t=%s ns)" % (g, worst, limit, t_at))
    rep["bitwise_state_equal"] = bitwise
    rep["first_divergence_ns"] = first_div
    # IMU: time base exact, values within the accel group tolerance scaled (noise is seeded, so should match)
    if A.imu.get("t_ns") != B.imu.get("t_ns"):
        rep["problems"].append("imu time bases differ")
    # events: same kinds in same order with identical timestamps
    ea = [(e["kind"], e["t_ns"]) for e in A.events]
    eb = [(e["kind"], e["t_ns"]) for e in B.events]
    rep["events_equal"] = ea == eb
    if ea != eb:
        rep["problems"].append("event sequence differs (%d vs %d events)" % (len(ea), len(eb)))
    # sensors: index timestamps exact; payload files hashed
    for kind, sa, sb in (("camera", A.cameras, B.cameras), ("lidar", A.lidars, B.lidars)):
        if sorted(sa) != sorted(sb):
            rep["problems"].append("%s sets differ" % kind)
            continue
        for name in sa:
            if sa[name]["acq_ns"] != sb[name]["acq_ns"]:
                rep["problems"].append("%s %s acquisition timestamps differ" % (kind, name))
                continue
            same = sum(sha(os.path.join(a_dir, f)) == sha(os.path.join(b_dir, g))
                       for f, g in zip(sa[name]["file"], sb[name]["file"]))
            rep["%s_%s_payload_identical" % (kind, name)] = "%d/%d" % (same, len(sa[name]["file"]))
    return rep


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("runs", nargs="+", help="run directories (first is the reference)")
    for g, (_, t) in GROUPS.items():
        ap.add_argument("--tol-" + g, type=float, default=t)
    ap.add_argument("--json", action="store_true")
    a = ap.parse_args(argv)
    if len(a.runs) < 2:
        ap.error("need at least two runs")
    tol = {g: getattr(a, "tol_" + g) for g in GROUPS}
    ok = True
    reports = [compare(a.runs[0], r, tol) for r in a.runs[1:]]
    for r in reports:
        ok &= not r["problems"]
        if a.json:
            print(json.dumps(r, indent=2))
        else:
            print("%s vs %s: bitwise_state_equal=%s first_divergence_ns=%s" % (
                os.path.basename(r["a"]), os.path.basename(r["b"]), r.get("bitwise_state_equal"), r.get("first_divergence_ns")))
            for g, v in r["groups"].items():
                print("  %-6s max|d|=%.3e tol=%.1e %s" % (g, v["max_abs"], v["tol"], "ok" if v["ok"] else "FAIL"))
            for p in r["problems"]:
                print("  PROBLEM:", p)
    print("REPRODUCIBLE" if ok else "NOT REPRODUCIBLE")
    return 0 if ok else 1


if __name__ == "__main__":
    sys.exit(main())
