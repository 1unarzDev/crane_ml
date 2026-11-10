#!/usr/bin/env python3
"""Validate, align and summarize one crane-drone-log/1 run; plot it against the obstacles.

  flight_summary.py RUN_DIR --png summary.png [--json summary.json]

Panels: top-down trajectory over trunks (obstacles.json, radius = dbh/2) with camera acquisition poses and
near-miss marks; altitude and speed vs time; nearest-trunk clearance at camera stamps (aligned with
DroneLog.camera_poses, i.e. ground truth interpolated at ACQUISITION time). Needs matplotlib.
"""
import argparse
import json
import math
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from drone_log import DroneLog  # noqa: E402


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("run")
    ap.add_argument("--png", required=True)
    ap.add_argument("--json")
    a = ap.parse_args(argv)
    log = DroneLog(a.run)
    problems = log.validate()
    summ = log.summary()
    summ["problems"] = problems
    summ["valid"] = not problems
    s = log.state
    t = [x * 1e-9 for x in s["t_ns"]]
    spd = [math.sqrt(vx * vx + vy * vy + vz * vz) for vx, vy, vz in zip(s["vx"], s["vy"], s["vz"])]
    summ["max_speed_mps"] = max(spd)
    # camera / imu alignment checks: every camera frame gets a ground-truth pose at its acquisition time
    cam_align = {}
    for name in log.cameras:
        poses = log.camera_poses(name)
        cam_align[name] = {"frames": len(poses["px"]), "first_acq_ns": int(log.cameras[name]["acq_ns"][0]) if poses["px"] else None}
    summ["camera_alignment"] = cam_align
    xs, ys = s["px"], s["py"]
    pad = 8.0
    x0, x1, y0, y1 = min(xs) - pad, max(xs) + pad, min(ys) - pad, max(ys) + pad
    near = [o for o in log.obstacles if x0 <= o["x"] <= x1 and y0 <= o["y"] <= y1]

    def clearance(px, py, pz):
        best = float("inf")
        for o in near:
            if "z_base" in o and not (o["z_base"] <= pz <= o["z_base"] + o.get("height_m", 1e9)):
                continue
            best = min(best, math.hypot(px - o["x"], py - o["y"]) - o.get("r", 0.0))
        return best

    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    fig = plt.figure(figsize=(13, 7.5))
    ax = fig.add_axes([0.04, 0.07, 0.5, 0.86])
    for o in near:
        ax.add_patch(plt.Circle((o["x"], o["y"]), max(o.get("r", 0.2), 0.05), color="saddlebrown", alpha=0.75, lw=0))
    ax.plot(xs, ys, "b-", lw=1.2, label="drone (state.csv, 1 kHz)")
    wp = log.manifest.get("trajectory_spec", {}).get("waypoints_enu")
    if wp:
        ax.plot([p[0] for p in wp], [p[1] for p in wp], "c--", lw=0.8, label="corridor waypoints")
    cname = "oakd_rgb" if "oakd_rgb" in log.cameras else (next(iter(log.cameras)) if log.cameras else None)
    if cname:
        cp = log.camera_poses(cname)
        ax.plot(cp["px"], cp["py"], ".", color="orange", ms=3, label="%s acquisition poses" % cname)
    nm = [e for e in log.events if e["kind"] == "near_miss"]
    for e in nm:
        p = log.align([e["t_min_ns"]], ("px", "py"))
        ax.plot(p["px"][0], p["py"][0], "rx", ms=8)
    ax.plot(xs[:1], ys[:1], "go", ms=7, label="start")
    ax.set_xlim(x0, x1)
    ax.set_ylim(y0, y1)
    ax.set_aspect("equal")
    ax.set_xlabel("east [m] (Unity x)")
    ax.set_ylabel("north [m] (Unity z)")
    ax.legend(loc="best", fontsize=8)
    m = log.manifest
    ax.set_title("%s  seed %s (%s)  %s\ntrunks from forest manifest (r = dbh/2); red x = near miss (< %s m)" % (
        m["run_id"], m["scene_seed"], m.get("split"), m.get("physics_profile"),
        nm[0]["threshold_m"] if nm else "0.5"), fontsize=9)
    a2 = fig.add_axes([0.6, 0.71, 0.37, 0.22])
    a2.plot(t, s["pz"], "k-", lw=1)
    a2.set_ylabel("altitude ENU z [m]")
    a2b = a2.twinx()
    a2b.plot(t, spd, "b-", lw=0.8)
    a2b.set_ylabel("speed [m/s]", color="b")
    a3 = fig.add_axes([0.6, 0.39, 0.37, 0.22])
    if cname:
        cl = [clearance(x, y, z) for x, y, z in zip(cp["px"], cp["py"], cp["pz"])]
        tc = [x * 1e-9 for x in log.cameras[cname]["acq_ns"]]
        a3.plot(tc, cl, "g-", lw=1)
        summ["min_trunk_axis_clearance_minus_r_m"] = min(cl)
    a3.set_ylabel("nearest trunk surface [m]")
    a3.set_xlabel("sim time [s]")
    a4 = fig.add_axes([0.6, 0.07, 0.37, 0.22])
    a4.axis("off")
    txt = ["valid: %s" % summ["valid"], "duration %.2f s, state %d rows (%.1f Hz), imu %d rows" % (
        summ["duration_s"], summ["state_rows"], summ["state_hz"], summ["imu_rows"]),
        "cameras: %s" % summ["cameras"], "lidars: %s" % (summ["lidars"] or m.get("extra", {}).get("lidar", "none")[:60]),
        "path %.1f m, max speed %.2f m/s, final soc %.4f" % (summ["path_length_m"], summ["max_speed_mps"], summ["final_soc"]),
        "events: %s" % summ["events"], "git %s" % m.get("crane_git_sha", "")[:12],
        "forest sha256 %s  physics sha256 %s" % (m.get("forest_manifest_sha256", "")[:12], m.get("physics_params_sha256", "")[:12])]
    a4.text(0, 1, "\n".join(txt), va="top", family="monospace", fontsize=8)
    fig.savefig(a.png, dpi=110)
    out = json.dumps(summ, indent=2, default=str)
    if a.json:
        with open(a.json, "w") as f:
            f.write(out + "\n")
    print(out)
    print("VALID" if not problems else "INVALID")
    return 0 if not problems else 1


if __name__ == "__main__":
    sys.exit(main())
