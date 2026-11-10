# SPDX-License-Identifier: GPL-3.0-or-later
"""Write a navpilot episode as a crane-drone-log/1 run directory (Docs/DroneVisuals/LOG_FORMAT.md).

Written: manifest.json, state.csv (200 Hz plant rate), events.jsonl (collision, near_miss, detection, marker),
obstacles.json. Not written: imu.csv, camera/, lidar/ (the pilot has no images or point files; the sensor
models are summarised in manifest.extra). The point-mass has no rotors: rotor_count = 1 virtual collective channel,
rpm0 = 0, cmd0 = collective thrust fraction |a_T + g| / (TW g). Attitude: yaw from the heading, roll/pitch from the
thrust direction. Validate with Tools/DroneData/drone_log.py.
"""
import json
import math
import os

from .forest import manifest_sha256
from .vehicle import G, load_params

DT_NS = 5_000_000


def _quat(yaw, atx, aty):
    # body z along thrust (atx, aty, g); body x as close to heading as possible
    zx, zy, zz = atx, aty, G
    n = math.sqrt(zx * zx + zy * zy + zz * zz)
    zx, zy, zz = zx / n, zy / n, zz / n
    hx, hy = math.cos(yaw), math.sin(yaw)
    # y = z x h, x = y x z
    yx, yy, yz = zy * 0 - zz * hy, zz * hx - zx * 0, zx * hy - zy * hx
    n = math.sqrt(yx * yx + yy * yy + yz * yz)
    yx, yy, yz = yx / n, yy / n, yz / n
    xx, xy, xz = yy * zz - yz * zy, yz * zx - yx * zz, yx * zy - yy * zx
    m00, m11, m22 = xx, yy, zz
    tr = m00 + m11 + m22
    if tr > 0:
        s = 0.5 / math.sqrt(tr + 1.0)
        w, x, y, z = 0.25 / s, (yz - zy) * s, (zx - xz) * s, (xy - yx) * s
    elif m00 > m11 and m00 > m22:
        s = 2.0 * math.sqrt(1.0 + m00 - m11 - m22)
        w, x, y, z = (yz - zy) / s, 0.25 * s, (yx + xy) / s, (zx + xz) / s
    elif m11 > m22:
        s = 2.0 * math.sqrt(1.0 + m11 - m00 - m22)
        w, x, y, z = (zx - xz) / s, (yx + xy) / s, 0.25 * s, (zy + yz) / s
    else:
        s = 2.0 * math.sqrt(1.0 + m22 - m00 - m11)
        w, x, y, z = (xy - yx) / s, (zx + xz) / s, (zy + yz) / s, 0.25 * s
    if w < 0:
        w, x, y, z = -w, -x, -y, -z
    n = math.sqrt(w * w + x * x + y * y + z * z)
    return w / n, x / n, y / n, z / n


def write_run(out, world, cfg, run_seed, states, events, res):
    os.makedirs(out, exist_ok=True)
    p = load_params(tilt_max_deg=cfg["tilt_max_deg"])
    st = world.stand
    man = {"schema": "crane-drone-log/1", "run_id": "navpilot-%d-%d" % (st["seed"], run_seed),
           "crane_git_sha": None, "scene": "navpilot-2.5d", "scene_seed": st["seed"], "split": st["split"],
           "forest_manifest_sha256": manifest_sha256(st), "physics_profile": p["profile_id"],
           "physics_params_sha256": None, "controller": "navpilot-baseline-rayprim",
           "trajectory_spec": {"start": list(world.start), "goal": list(world.goal), "v_limit": cfg["v_limit"],
                               "fly_h": world.fly_h},
           "disturbance": {"wind": cfg["wind"], "lighting": None}, "unity_version": None,
           "wall_clock_start_utc": None,
           "clock": {"unit": "ns", "fixed_dt_ns": DT_NS, "substeps": 1, "substep_dt_ns": DT_NS, "substep_hz": 200.0,
                     "state_every_substeps": 1, "state_hz": 200.0},
           "frames": {"world": "ENU", "body": "FLU", "quat": "wxyz world_from_body"}, "rotor_count": 1,
           "imu": {"rate_hz": 0, "noise": {}}, "cameras": [], "lidars": [],
           "extra": {"generator": "Tools/NavPilot", "sensor": cfg["sensor"], "latency_tail": cfg["latency_tail"],
                     "run_seed": run_seed, "result": res, "note": "2.5D point-mass pilot; rpm0/cmd0 are a virtual collective"}}
    with open(os.path.join(out, "state.csv"), "w") as f:
        f.write("tick,sub,t_ns,px,py,pz,qw,qx,qy,qz,vx,vy,vz,wx,wy,wz,ax,ay,az,rpm0,cmd0,batt_v,batt_a,batt_soc\n")
        for row in states:
            k = int(row[0])
            t, x, y, yaw, vx, vy, ax, ay, yr, atx, aty = (float(q) for q in row[1:])
            qw, qx, qy, qz = _quat(yaw, atx, aty)
            c, s = math.cos(yaw), math.sin(yaw)
            bax, bay = c * ax + s * ay, -s * ax + c * ay
            cmd = math.sqrt(G * G + atx * atx + aty * aty) / (p["thrust_to_weight"] * G)
            f.write("%d,0,%d,%r,%r,%r,%r,%r,%r,%r,%r,%r,0.0,0.0,0.0,%r,%r,%r,0.0,0.0,%r,nan,nan,nan\n" % (
                k, k * DT_NS, x, y, world.fly_h, qw, qx, qy, qz, vx, vy, yr, bax, bay, cmd))
    with open(os.path.join(out, "events.jsonl"), "w") as f:
        for e in events:
            f.write(json.dumps(e) + "\n")
    with open(os.path.join(out, "obstacles.json"), "w") as f:
        json.dump(world.obstacles_json(), f)
    last = states[-1][0] if states else 0
    man["end"] = {"end_tick": last, "end_t_ns": last * DT_NS, "counts": {"state": len(states), "events": len(events)}}
    with open(os.path.join(out, "manifest.json"), "w") as f:
        json.dump(man, f, indent=1)
