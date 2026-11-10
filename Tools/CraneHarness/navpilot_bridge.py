# SPDX-License-Identifier: GPL-3.0-or-later
"""2.5D NavPilot twin of a CRANE harness run, for fidelity validation (Docs/DroneVisuals/HARNESS.md section 3).

Input: a stand exported by the .NET harness (`CraneExperiment export-navpilot`): the same forest manifest, the same
straight cross_k route, rotated into NavPilot's corridor frame (start at the origin, goal on +x), heights as AGL with
the CRANE cruise altitude mapped onto NavPilot's 2.0 m slab.

Episode loop: a trimmed copy of Tools/NavPilot/experiments/e2_episode.run (E2-frozen file, not edited): same plant,
sensors, compute-tail model, RNG streams and MAP_RHP stack (stacks/policies.Stack). Differences, all to match the CRANE
harness conventions so that the comparison isolates 3D vs 2.5D:
  * r_body = CRANE body radius (0.2479 m, AutonomousMission.BodyRadius on the AOS7 profile) instead of NavPilot's 0.25 m;
  * success = within 1.5 m (xy) of the goal; stuck = < 1 m along-route progress in 12 s; time limit = L / 0.5 + 30 s;
  * prior = stems of d >= 6 cm (CRANE ManifestPriorMap: stems/limbs >= 6 cm), no phantoms, no alignment drift,
    local drift 0.03 m/sqrt(m) (CRANE default);
  * detector isolated (no GPU share), no network model, calm wind, nominal latency tail;
  * adds the true stopping-margin violation metric of navpilot.episode (fraction of planner cycles with
    d_true < StopRule.dist(v)) and logs every perception-to-command age.
Known structural differences that remain (they ARE the fidelity question): 2.5D slab (+-0.35 m) vs 3D capsules and 3D
primitives with climb; NavPilot's angular z-buffer stereo/LiDAR vs per-ray 3D raycasts; point-mass plant vs the RK4
rotor plant and cascaded controller; NavPilot's prior enters the belief (w_prior = 1) while CRANE uses it for the
cost-to-go only (PriorUse = route).
"""
import argparse
import json
import math
import os
import sys

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
NAVPILOT = os.path.normpath(os.path.join(HERE, "..", "NavPilot"))
if NAVPILOT not in sys.path:
    sys.path.insert(0, NAVPILOT)

from navpilot import sensors as S  # noqa: E402
from navpilot.forest import World, guard_seed  # noqa: E402
from navpilot.vehicle import Vehicle, load_params  # noqa: E402
from stacks.core import TimedOccMap  # noqa: E402
from stacks.policies import MAP_RHP, Stack  # noqa: E402
from experiments.scenarios import prior_grid  # noqa: E402

DT = 0.005
CRANE_R_BODY = 0.2479  # AutonomousMission.BodyRadius measured on aos7_v5_f90_6s (replay evidence; AUTONOMY_STATUS says 0.268)


def _rng(seed, stream):
    return np.random.Generator(np.random.PCG64(np.random.SeedSequence([int(seed) & 0xFFFFFFFF, 0x4E50, stream])))


def _true_free(world, x, y, ux, uy, rb, R):
    """Truth free distance along (ux, uy) for a disc of radius rb (navpilot.episode._true_free)."""
    idx = world.query(x + 0.5 * R * ux, y + 0.5 * R * uy, 0.5 * R + rb + 0.5)
    if len(idx) == 0:
        return R
    dx, dy = world.cx[idx] - x, world.cy[idx] - y
    along = dx * ux + dy * uy
    perp = np.abs(-dx * uy + dy * ux)
    rr = world.cr[idx] + rb
    hit = (along > 0) & (perp < rr) & world.hard[idx]
    if not hit.any():
        return R
    d = along[hit] - np.sqrt(np.maximum(rr[hit] ** 2 - perp[hit] ** 2, 0.0))
    return float(max(min(d.min(), R), 0.0))


def _swept(world, x0, y0, x1, y1):
    """Hard-only clearance (TREEGEN collider contract: d >= 2 cm; CRANE: r >= 1 cm) over the last 20 ms segment."""
    best, bi = math.inf, -1
    for u in (0.0, 0.5, 1.0):
        x, y = x0 + u * (x1 - x0), y0 + u * (y1 - y0)
        idx = world.query(x, y, 1.5)
        if len(idx) == 0:
            continue
        idx = idx[world.hard[idx]]
        if len(idx) == 0:
            continue
        d = np.hypot(world.cx[idx] - x, world.cy[idx] - y) - world.cr[idx]
        k = int(np.argmin(d))
        if d[k] < best:
            best, bi = float(d[k]), int(idx[k])
    return best, bi


def load_stand(path):
    with open(path) as f:
        st = json.load(f)
    guard_seed(st["seed"])   # split guard (Tools/DroneData/splits.json); exported stands are DEV
    return st


def run(stand, v_lim, run_seed, r_body=CRANE_R_BODY, world=None, latency_tail="nominal", stuck_s=12.0):
    world = world or World(stand)
    veh = load_params(r_body=r_body)
    stack = Stack(MAP_RHP, veh)
    r_sense, r_comp, r_wind = _rng(run_seed, 1), _rng(run_seed, 2), _rng(run_seed, 3)
    r_loc = _rng(run_seed, 6)
    bounds = stand["corridor"]["bounds"]
    acc = TimedOccMap(bounds, world.half_width)
    prior_stems = [{"x": s["x"], "y": s["y"], "dbh_m": s["dbh_m"]} for s in stand["stems"] if s["dbh_m"] >= 0.06]
    pL, pO, _ = prior_grid(prior_stems, acc)
    goal = world.goal
    L = float(stand["corridor"]["length_m"])
    tlim = L / 0.5 + 30.0
    v = Vehicle(veh, *world.start, yaw=0.0, wind="calm")
    st_cfg, li_cfg = dict(S.STEREO), dict(S.LIDAR)
    st_period, li_period, period = 1.0 / st_cfg["rate_hz"], 1.0 / li_cfg["rate_hz"], 0.05
    ex = ey = 0.0
    last_loc = (v.x, v.y)
    t, step = 0.0, 0
    next_st, next_li, next_plan = 0.0, 0.013, 0.06
    queue, pending = [], None
    vcx = vcy = yaw_cmd = 0.0
    t_cmd, wd_active, n_wd = 0.0, False, 0
    ages, comps, speeds = [], [], []
    n_cycles = n_margin = margin_ticks = n_nofeas = 0
    nofeas_prev = False
    min_clear = math.inf
    px_prev, py_prev = v.x, v.y
    best_prog, best_prog_t = 0.0, 0.0
    outcome, contact = None, None
    wbuf = []
    sig = {"sigma_align_rep": 0.1, "rep_age": 0.0, "agree": 1.0, "q_obs": 1.0, "sigma_loc_rate": 0.03}
    while True:
        ds = math.hypot(v.x - last_loc[0], v.y - last_loc[1])
        if ds > 0.25:
            ex += 0.03 * math.sqrt(ds) * r_loc.standard_normal()
            ey += 0.03 * math.sqrt(ds) * r_loc.standard_normal()
            last_loc = (v.x, v.y)
        if t >= next_st:
            fr = S.stereo_frame(world, t, v.x, v.y, v.yaw, v.vx, v.vy, v.yaw_rate, r_sense, st_cfg)
            queue.append((t + st_cfg["latency_s"], fr, ex, ey))
            next_st += st_period
        if t >= next_li:
            fr = S.lidar_frame(world, t, v.x, v.y, v.yaw, r_sense, li_cfg)
            queue.append((t + li_cfg["latency_s"], fr, ex, ey))
            next_li += li_period
        if t >= next_plan:
            ready = [q for q in queue if q[0] <= t]
            queue = [q for q in queue if q[0] > t]
            newest = -math.inf
            si, sj = int(round(ex / acc.res)), int(round(ey / acc.res))
            for sname in ("stereo", "lidar"):
                for q in [q for q in ready if q[1].sensor == sname][-3:]:
                    acc.integrate(q[1], t, q[2], q[3])
                    newest = max(newest, q[1].t_acq)
            xh, yh = v.x + ex, v.y + ey
            out = stack.cycle(t, xh, yh, v.vx, v.vy, (goal[0] + ex, goal[1] + ey), v_lim, acc, None, (pL, pO), (si, sj), sig)
            c = S.compute_time(latency_tail, *r_comp.random(3))
            comps.append(c)
            n_cycles += 1
            if newest > -math.inf:
                age = t + c - newest
                ages.append(age)
                stack.rule.observe_age(age)
            nf = not out["feasible_any"]
            n_nofeas += int(nf and not nofeas_prev)
            nofeas_prev = nf
            pending = (t + c, out["v_cmd"] * math.cos(out["theta"]), out["v_cmd"] * math.sin(out["theta"]), out["theta"])
            next_plan = t + max(period, c)
            sp = v.speed
            if sp > 0.5:
                margin_ticks += 1
                d_true = _true_free(world, v.x, v.y, v.vx / sp, v.vy / sp, veh["r_body"], 15.0)
                if d_true < float(stack.rule.dist(sp)) - stack.rule.margin:
                    n_margin += 1
        if pending is not None and t >= pending[0]:
            _, vcx, vcy, yaw_cmd = pending
            pending, t_cmd, wd_active = None, t, False
        if not wd_active and t - t_cmd > 0.15 and (vcx or vcy):
            vcx = vcy = 0.0
            wd_active, n_wd = True, n_wd + 1
        if not wbuf:
            wbuf = r_wind.standard_normal(4096).tolist()[::-1]
        nx_, ny_ = wbuf.pop(), wbuf.pop()
        v.step(DT, vcx, vcy, yaw_cmd, nx_, ny_)
        t += DT
        step += 1
        if step % 4 == 0:
            sp = v.speed
            speeds.append(sp)
            cl, ci = _swept(world, px_prev, py_prev, v.x, v.y)
            cl -= veh["r_body"]
            px_prev, py_prev = v.x, v.y
            min_clear = min(min_clear, cl)
            if cl < 0:
                oid = int(world.cobj[ci])
                contact = {"speed": sp, "kind": "stem" if world.okind[oid] == 0 else "branch", "diam": float(world.odiam[oid]), "t": t}
                outcome = "collision"
                break
            prog = max(0.0, min(v.x, L))
            if prog > best_prog + 1.0:
                best_prog, best_prog_t = prog, t
            if math.hypot(v.x - goal[0], v.y - goal[1]) < 1.5:
                outcome = "success"
                break
            if t - best_prog_t > stuck_s:
                outcome = "stuck"
                break
            if t > tlim:
                outcome = "timeout"
                break
    a = np.asarray(ages) if ages else np.zeros(1)
    return {"outcome": outcome, "success": outcome == "success", "collision": outcome == "collision",
            "time_s": t, "progress_frac": 1.0 if outcome == "success" else float(max(v.x, 0.0) / L),
            "min_clear": float(min_clear), "mean_speed": float(np.mean(speeds)) if speeds else 0.0,
            "contact": contact, "age_p50": float(np.quantile(a, 0.5)), "age_p99": float(np.quantile(a, 0.99)),
            "age_max": float(a.max()), "ages_ms_hist": np.histogram(a * 1e3, bins=np.arange(0, 302, 4))[0].tolist(),
            "margin_viol_frac": n_margin / max(margin_ticks, 1), "margin_ticks": margin_ticks, "watchdog": n_wd,
            "nofeasible_events": n_nofeas, "tau_final": float(stack.rule.tau), "cycles": n_cycles, "r_body": veh["r_body"]}


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    ap.add_argument("--stands", required=True, help="directory of stand_f<seed>_<mission>.json")
    ap.add_argument("--speeds", default="2,2.5,3,4,5,6,8")
    ap.add_argument("--seeds", default="1")
    ap.add_argument("--only", default="")
    ap.add_argument("--out", required=True)
    ap.add_argument("--shard", type=int, default=0)
    ap.add_argument("--nshards", type=int, default=1)
    a = ap.parse_args(argv)
    files = sorted(f for f in os.listdir(a.stands) if f.startswith("stand_") and f.endswith(".json"))
    only = [s for s in a.only.split(",") if s]
    done = set()
    if os.path.exists(a.out):
        for line in open(a.out):
            r = json.loads(line)
            done.add((r["mission"], r["speed"], r["run_seed"]))
    jobs = []
    for f in files:
        mission = f[:-5].split("_", 2)[2]
        if only and mission not in only:
            continue
        for v in [float(x) for x in a.speeds.split(",")]:
            for s in [int(x) for x in a.seeds.split(",")]:
                jobs.append((f, mission, v, s))
    with open(a.out, "a") as out:
        cache = {}
        for k, (f, mission, v, s) in enumerate(jobs):
            if k % a.nshards != a.shard or (mission, v, s) in done:
                continue
            if f not in cache:
                cache.clear()
                st = load_stand(os.path.join(a.stands, f))
                cache[f] = (st, World(st))
            st, w = cache[f]
            r = run(st, v, s, world=w)
            r.update({"harness": "navpilot_2p5d", "forest_seed": st["seed"], "mission": mission, "speed": v, "run_seed": s,
                      "pair_key": "f%d|%s|r%d|d0" % (st["seed"], mission, s), "cluster": "f%d|%s" % (st["seed"], mission)})
            out.write(json.dumps(r) + "\n")
            out.flush()
            print("[navpilot] %-8s v=%.1f s=%d -> %-9s t=%5.1f clr=%.2f age99=%.0fms" % (mission, v, s, r["outcome"], r["time_s"], r["min_clear"], r["age_p99"] * 1e3), flush=True)


if __name__ == "__main__":
    main()
