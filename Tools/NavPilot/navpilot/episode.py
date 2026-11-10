# SPDX-License-Identifier: GPL-3.0-or-later
"""One closed-loop episode: plant at 200 Hz, sensors at their own rates, planner cycles with sampled compute time.

Timing model (per planner cycle k, nominal period 50 ms):
  start t_k -> consume queued frames with t_acq + latency <= t_k (bounded queue, newest 3 per sensor) -> map insert
  -> plan from the true state at t_k -> command becomes active at t_k + c_k (c_k ~ compute_time(tail)).
  Next cycle starts at t_k + max(period, c_k). Deadline miss: c_k > period. Age-gate miss: sensor-to-setpoint age
  (t_k + c_k - t_acq of the newest frame used) > 120 ms (AUTONOMY_DESIGN §2 p99 gate). During a stall the plant keeps
  tracking the last velocity command (PX4 holds the last trajectory setpoint).

Failure cause (primary label; first matching rule):
  collision:
    t_need = t_contact - stop_distance(v_recent, tau_belief, a_belief, margin) / max(v_recent, 0.5)
    mapped(t)    = some cell within 0.15 m of the struck object's circles was occupied by time t
    in_view(t)   = the object was geometrically visible (in FOV, in range, unoccluded) by time t
    detected(t)  = the sensor returned a point on the object by time t
    1. mapped by t_need, and a deadline miss/age miss in the last 1.0 s     -> latency_stall
    2. mapped by t_need, map still had it at contact                         -> tracking_lag (plan/dynamics; extra label)
    3. mapped by t_need but erased by free-space carving before contact      -> thin_branch_miss if thin else map_lag
    4. not mapped by t_need, detected by t_need                              -> map_lag (or latency_stall if a stall
                                                                                 hit the last 1.0 s)
    5. not detected by t_need, in view by t_need                             -> thin_branch_miss (D < 4 cm) / sensor_miss
    6. not in view by t_need                                                  -> occlusion (incl. outside FOV)
  no collision, no goal: progress < 1 m over stuck_s or time limit           -> local_minimum
  success but target recall < 1                                              -> detection_miss (secondary label)
"""
import json
import math
import os

import numpy as np

from . import sensors as S
from .forest import World, manifest_sha256
from .planner import MAP, PLANNER, OccMap, plan, stop_distance
from .vehicle import Vehicle, load_params

DT = 0.005
THIN_D = 0.04
AGE_GATE = 0.12
NEAR_MISS_M = 0.3
STALL_S = 0.10       # a compute cycle longer than 2 periods counts as a stall for cause labelling
STALL_AGE_S = 0.25   # or a sensor-to-setpoint age above 250 ms


def default_config(**kw):
    c = {"sensor": "stereo", "v_limit": 5.0, "latency_tail": "nominal", "wind": "light_breeze",
         "time_limit_s": None, "planner": dict(PLANNER), "stereo": dict(S.STEREO), "lidar": dict(S.LIDAR),
         "rgb": dict(S.RGB), "tilt_max_deg": 40.0}
    c.update(kw)
    return c


def _rng(seed, stream):
    return np.random.Generator(np.random.PCG64(np.random.SeedSequence([int(seed) & 0xFFFFFFFF, 0x4E50, stream])))


def run_episode(world, cfg, run_seed, log_dir=None):
    if not isinstance(world, World):
        world = World(world)
    veh_p = load_params(tilt_max_deg=cfg["tilt_max_deg"])
    pc = cfg["planner"]
    r_sense, r_comp, r_wind, r_det = _rng(run_seed, 1), _rng(run_seed, 2), _rng(run_seed, 3), _rng(run_seed, 4)
    use_st = cfg["sensor"] in ("stereo", "fusion")
    use_li = cfg["sensor"] in ("lidar", "fusion")
    L = world.length
    tlim = cfg["time_limit_s"] or (L / 1.0 + 20.0)
    v = Vehicle(veh_p, *world.start, yaw=0.0, wind=cfg["wind"])
    bounds = world.stand["corridor"]["bounds"]
    occ = OccMap(bounds, world.half_width)
    goal = world.goal
    first_view = np.full(world.n_obj, np.inf)
    first_det = np.full(world.n_obj, np.inf)
    period = 1.0 / pc["rate_hz"]
    st_period, li_period, det_period = 1.0 / cfg["stereo"]["rate_hz"], 1.0 / cfg["lidar"]["rate_hz"], 1.0 / cfg["rgb"]["rate_hz"]
    t = 0.0
    step = 0
    next_st, next_li, next_det, next_plan = 0.0, 0.013, 0.021, 0.06
    queue = []                     # (t_avail, frame)
    pending = None                 # (t_apply, vcx, vcy, yaw_cmd)
    vcx = vcy = 0.0
    yaw_cmd = 0.0
    n_cycles = n_deadline = n_age = n_margin_viol = n_belief_viol = n_emerg = 0
    margin_ticks = 0
    stall_times = []               # times of deadline/age misses (output time)
    ages = []
    comp = []
    min_clear = math.inf
    speeds = []
    last_prog_t, last_prog_x = 0.0, world.start[0]
    outcome, cause, contact = None, None, None
    tgt_hits = np.zeros(len(world.targets), int)
    tgt_best = [""] * len(world.targets)   # best miss reason seen
    tgt_minr = np.full(len(world.targets), np.inf)
    a_belief = None
    near_zone, near_min, near_t, near_obj = False, math.inf, 0, -1
    events, states = [], []
    logging = log_dir is not None
    px_prev, py_prev = v.x, v.y
    hist = []                      # (t, speed) for v_recent
    soft_hit = set()               # soft (< 2 cm) twigs brushed: counted, not terminal
    wbuf = []                      # gust normals as Python floats (scalar plant loop)
    while True:
        # ---------------- sensors
        if use_st and t >= next_st:
            fr = S.stereo_frame(world, t, v.x, v.y, v.yaw, v.vx, v.vy, v.yaw_rate, r_sense, cfg["stereo"])
            first_view[fr.visible_obj] = np.minimum(first_view[fr.visible_obj], t)
            first_det[fr.detected_obj] = np.minimum(first_det[fr.detected_obj], t)
            queue.append((t + cfg["stereo"]["latency_s"], fr))
            next_st += st_period
        if use_li and t >= next_li:
            fr = S.lidar_frame(world, t, v.x, v.y, v.yaw, r_sense, cfg["lidar"])
            first_view[fr.visible_obj] = np.minimum(first_view[fr.visible_obj], t)
            first_det[fr.detected_obj] = np.minimum(first_det[fr.detected_obj], t)
            queue.append((t + cfg["lidar"]["latency_s"], fr))
            next_li += li_period
        if t >= next_det and len(world.targets):
            _detect(world, v, t, cfg["rgb"], r_det, tgt_hits, tgt_best, tgt_minr, events if logging else None)
            next_det += det_period
        # ---------------- planner cycle
        if t >= next_plan:
            ready = [q for q in queue if q[0] <= t]
            queue = [q for q in queue if q[0] > t]
            newest = -math.inf
            for sname in ("stereo", "lidar"):
                fs = [f for _, f in ready if f.sensor == sname][-3:]   # bounded queue
                for f in fs:
                    occ.integrate(f, t)
                    newest = max(newest, f.t_acq)
            out = plan(occ, v.x, v.y, v.vx, v.vy, goal, cfg["v_limit"], veh_p, pc)
            a_belief = out["a_brake"]
            if out["v_cmd"] > 0 and stop_distance(out["v_rule"], out["tau"], out["a_brake"], out["margin"]) > out["d_free_cmd"] + 1e-9:
                n_belief_viol += 1
            n_emerg += out["emergency"]
            c = S.compute_time(cfg["latency_tail"], *r_comp.random(3))
            comp.append(c)
            n_cycles += 1
            if c > period:
                n_deadline += 1
            if newest > -math.inf:
                age = t + c - newest
                ages.append(age)
                if age > AGE_GATE:
                    n_age += 1
            if c > STALL_S or (newest > -math.inf and t + c - newest > STALL_AGE_S):
                stall_times.append(t + c)
            pending = (t + c, out["v_cmd"] * math.cos(out["theta"]), out["v_cmd"] * math.sin(out["theta"]), out["theta"])
            next_plan = t + max(period, c)
            # true stopping margin along the current velocity (ground truth, belief tau/a)
            sp = v.speed
            if sp > 0.5:
                margin_ticks += 1
                d_true = _true_free(world, v.x, v.y, v.vx / sp, v.vy / sp, veh_p["r_body"], 15.0)
                if d_true < stop_distance(sp, out["tau"], out["a_brake"], out["margin"]):
                    n_margin_viol += 1
        if pending is not None and t >= pending[0]:
            _, vcx, vcy, yaw_cmd = pending
            pending = None
        # ---------------- plant
        if not wbuf:
            wbuf = r_wind.standard_normal(4096).tolist()[::-1]
        nx, ny = wbuf.pop(), wbuf.pop()
        v.step(DT, vcx, vcy, yaw_cmd, nx, ny)
        t += DT
        step += 1
        sp = v.speed
        if logging:
            states.append((step, t, v.x, v.y, v.yaw, v.vx, v.vy, v.ax, v.ay, v.yaw_rate, v.atx, v.aty))
        if step % 4 == 0:
            speeds.append(sp)
            hist.append((t, sp))
            cl, ci, cls, csi = _swept_clearance(world, px_prev, py_prev, v.x, v.y)
            cl -= veh_p["r_body"]
            if cls - veh_p["r_body"] < 0:
                soft_hit.add(int(world.cobj[csi]))
            px_prev, py_prev = v.x, v.y
            min_clear = min(min_clear, cl)
            if cl < NEAR_MISS_M and cl >= 0:
                if not near_zone:
                    near_zone, near_min = True, math.inf
                if cl < near_min:
                    near_min, near_t, near_obj = cl, t, ci
            elif near_zone and cl >= NEAR_MISS_M:
                near_zone = False
                if logging:
                    events.append({"t_ns": _ns(t), "kind": "near_miss", "distance_m": near_min,
                                   "obstacle": _obj_name(world, near_obj), "threshold_m": NEAR_MISS_M,
                                   "t_min_ns": _ns(near_t)})
            if cl < 0:
                oid = int(world.cobj[ci])
                vrec = max([s for tt, s in hist if tt > t - 2.0] + [sp])
                cause = _label_collision(world, occ, oid, v.x, v.y, t, vrec, first_view, first_det, stall_times,
                                         pc["tau_belief_s"], a_belief or 6.0, pc["margin_m"])
                nrm = [v.x - world.cx[ci], v.y - world.cy[ci], 0.0]
                nn = math.hypot(nrm[0], nrm[1]) or 1.0
                contact = {"t": t, "speed": sp, "object": oid, "kind": "stem" if world.okind[oid] == 0 else "branch",
                           "diam_m": float(world.odiam[oid])}
                if logging:
                    events.append({"t_ns": _ns(t), "kind": "collision", "other": _obj_name(world, ci),
                                   "impact_speed_mps": sp, "normal_enu": [nrm[0] / nn, nrm[1] / nn, 0.0]})
                outcome = "collision"
                break
            if v.x - last_prog_x > 1.0:
                last_prog_t, last_prog_x = t, v.x
            if math.hypot(v.x - goal[0], v.y - goal[1]) < 1.5:
                outcome = "success"
                break
            if t - last_prog_t > pc["stuck_s"] or t > tlim:
                outcome, cause = "stuck", "local_minimum"
                break
    # ---------------- detection recall over targets the drone passed within 15 m
    along = tgt_minr <= 15.0
    n_along = int(along.sum())
    confirmed = tgt_hits >= cfg["rgb"]["confirm"]
    recall = float((confirmed & along).sum() / n_along) if n_along else float("nan")
    miss_reasons = {}
    for k in np.nonzero(along & ~confirmed)[0]:
        miss_reasons[tgt_best[k] or "never_in_fov"] = miss_reasons.get(tgt_best[k] or "never_in_fov", 0) + 1
    if outcome == "success" and n_along and recall < 1.0:
        cause = "detection_miss"
    comp_a = np.array(comp) if comp else np.zeros(1)
    ages_a = np.array(ages) if ages else np.zeros(1)
    res = {"outcome": outcome, "success": outcome == "success", "collision": outcome == "collision",
           "cause": cause, "contact_speed": contact["speed"] if contact else None,
           "contact_kind": contact["kind"] if contact else None, "contact_diam_m": contact["diam_m"] if contact else None,
           "near_miss_m": float(min_clear), "mean_speed": float(np.mean(speeds)) if speeds else 0.0,
           "time_s": t, "time_to_goal_s": t if outcome == "success" else None, "progress_m": float(v.x),
           "margin_viol_frac": n_margin_viol / max(margin_ticks, 1), "belief_violations": n_belief_viol,
           "emergency_frac": n_emerg / max(n_cycles, 1),
           "cycles": n_cycles, "deadline_misses": n_deadline, "age_gate_misses": n_age,
           "compute_p99_s": float(np.quantile(comp_a, 0.99)), "age_p99_s": float(np.quantile(ages_a, 0.99)),
           "soft_contacts": len(soft_hit), "targets_along": n_along, "recall": recall, "target_miss_reasons": miss_reasons}
    if logging:
        from .logio import write_run
        write_run(log_dir, world, cfg, run_seed, states, events, res)
    return res


# --------------------------------------------------------------------------- helpers
def _ns(t):
    return int(round(t / DT)) * int(DT * 1e9)


def _obj_name(world, ci):
    if ci < 0:
        return "none"
    o = int(world.cobj[ci])
    return ("stem_%d" % o) if world.okind[o] == 0 else ("branchobj_%d" % o)


def _swept_clearance(world, x0, y0, x1, y1):
    mx, my = 0.5 * (x0 + x1), 0.5 * (y0 + y1)
    seg = math.hypot(x1 - x0, y1 - y0)
    idx = world.query(mx, my, 0.5 * seg + 1.5)
    if len(idx) == 0:
        return math.inf, -1, math.inf, -1
    cx, cy = world.cx[idx], world.cy[idx]
    if seg < 1e-9:
        d = np.hypot(cx - x1, cy - y1)
    else:
        ux, uy = (x1 - x0) / seg, (y1 - y0) / seg
        s = np.clip((cx - x0) * ux + (cy - y0) * uy, 0.0, seg)
        d = np.hypot(cx - (x0 + s * ux), cy - (y0 + s * uy))
    d = d - world.cr[idx]
    h = world.hard[idx]
    dh = np.where(h, d, np.inf)
    ds = np.where(h, np.inf, d)
    k, j = int(np.argmin(dh)), int(np.argmin(ds))
    return float(dh[k]), int(idx[k]), float(ds[j]), int(idx[j])


def _true_free(world, x, y, ux, uy, rb, R):
    """Distance a disc of radius rb can travel along (ux, uy) before touching any circle (ground truth)."""
    idx = world.query(x + 0.5 * R * ux, y + 0.5 * R * uy, 0.5 * R + 1.0)
    if len(idx) == 0:
        return R
    dx, dy = world.cx[idx] - x, world.cy[idx] - y
    along = dx * ux + dy * uy
    lat = np.abs(dx * uy - dy * ux)
    rr = world.cr[idx] + rb
    m = (along > -rr) & (lat < rr)
    if not m.any():
        return R
    d = along[m] - np.sqrt(rr[m] ** 2 - lat[m] ** 2)
    return float(max(min(d.min(), R), 0.0))


def _label_collision(world, occ, oid, x, y, t, vrec, first_view, first_det, stall_times, tau, a, margin):
    # only the struck part of the object (a branch base touches its stem, whose cells must not count)
    sel = (world.cobj == oid) & world.hard & (np.hypot(world.cx - x, world.cy - y) - world.cr < 0.4)
    if not sel.any():
        sel = world.cobj == oid
    xs, ys = world.cx[sel], world.cy[sel]
    t_need = t - stop_distance(vrec, tau, a, margin) / max(vrec, 0.5)
    t_map = occ.first_occupied_near(xs, ys, 0.15)
    stalled = any(t - 1.0 <= s <= t for s in stall_times)
    thin = world.okind[oid] == 1 and world.odiam[oid] < THIN_D
    if t_map <= t_need:
        if stalled:
            return "latency_stall"
        if occ.occupied_now_near(xs, ys, 0.15):
            return "tracking_lag"
        return "thin_branch_miss" if thin else "map_lag"
    if first_det[oid] <= t_need:
        return "latency_stall" if stalled else "map_lag"
    if first_view[oid] <= t_need:
        return "thin_branch_miss" if thin else "sensor_miss"
    return "occlusion"


def _detect(world, v, t, cfg, rng, hits, best, minr, events):
    half = math.radians(0.5 * cfg["hfov_deg"])
    rank = {"": 0, "never_in_fov": 1, "out_of_range": 2, "occluded": 3, "blur_or_size": 4}
    for k, (tx, ty, sz) in enumerate(world.targets):
        dx, dy = tx - v.x, ty - v.y
        r = math.hypot(dx, dy)
        minr[k] = min(minr[k], r)
        if r > cfg["range_max"] + 10:
            continue
        b = (math.atan2(dy, dx) - v.yaw + math.pi) % (2 * math.pi) - math.pi
        if abs(b) > half:
            reason = "never_in_fov"
        elif r > cfg["range_max"]:
            reason = "out_of_range"
        else:
            vis = S.target_visibility(world, v.x, v.y, tx, ty, sz)
            if vis < 0.2:
                reason = "occluded"
            else:
                vlat = abs(v.vx * dy - v.vy * dx) / max(r, 1e-6)
                p = S.detection_p(r, sz, vis, vlat, v.yaw_rate, cfg)
                if events is not None:
                    events.append({"t_ns": _ns(t), "kind": "detection", "target": "person_%d" % k,
                                   "camera": "oakd_rgb", "range_m": r, "u": float(0.5 - 0.5 * b / half), "v": 0.5})
                if rng.random() < p:
                    hits[k] += 1
                    continue
                reason = "blur_or_size"
        if rank[reason] > rank[best[k]]:
            best[k] = reason
