# SPDX-License-Identifier: GPL-3.0-or-later
"""Experiment-2 closed-loop episode (plant 200 Hz, sensors at their rates, planner cycles with sampled compute).

Same plant, sensors, compute-tail model and RNG-stream convention as navpilot.episode; additions:
  * estimator: local estimate x_hat = x + e(t) (fresh frames registered at x_hat), map alignment error delta(t)
    (prior displaced by e + delta in the planner frame). The two are independent processes (scenarios.py).
  * network: link outages (cond netloss) freeze the alignment-quality report and queue person reports; nothing in
    local navigation depends on the link except that report.
  * detector GPU share (cond detector): planner compute c -> c (1 + kappa rho / (1 - rho)) + [U < rho] U' t_inf,
    rho = t_inf * f_det (M/G/1-style slowdown + residual wait). No Tools/Vision timing results exist in this
    tree, so t_inf is parametrized [U]: isolated (DLA/other accelerator, rho 0), on (20 ms @ 10 Hz, rho 0.2),
    heavy (45 ms @ 10 Hz, rho 0.45). kappa = 0.5 (half of the planner cycle contends for the GPU) [U].
  * arbiter stall watchdog: no new command for WD_S -> zero-velocity (brake) command (all stacks but MAP_RHP_N1).
  * success = truth crosses x >= goal_x - 1 (corridor traversal; alignment error must not move the finish line).
"""
import math

import numpy as np

from navpilot import sensors as S
from navpilot.forest import World
from navpilot.vehicle import Vehicle, load_params
from stacks.core import TimedOccMap
from stacks.policies import REACTIVE, Stack

from .scenarios import align_offset, hint_value, prior_grid

DT = 0.005
NEAR_MISS_M = 0.3
WD_S = 0.15
DETECTOR = {"isolated": (0.0, 10.0), "on": (0.020, 10.0), "heavy": (0.045, 10.0)}
KAPPA = 0.5
REPORT_WINDOWS = (20.0, 30.0)


def default_cond(**kw):
    c = {"latency_tail": "heavy", "detector": "on", "netloss": False, "v_lim": 7.0, "det_beta": 0.0,
         "wind": "light_breeze", "time_limit_s": 80.0, "stuck_s": 10.0}
    c.update(kw)
    return c


def _rng(seed, stream):
    return np.random.Generator(np.random.PCG64(np.random.SeedSequence([int(seed) & 0xFFFFFFFF, 0x4E50, stream])))


def _outages(r, T):
    """Link outage intervals: Poisson onsets (1 per 12 s), durations U(3, 8) s, plus one forced outage 6-14 s."""
    out = [(6.0, 14.0)]
    t = 0.0
    while t < T:
        t += r.exponential(12.0)
        out.append((t, t + r.uniform(3.0, 8.0)))
    return out


def _link_up(out, t):
    return not any(a <= t < b for a, b in out)


def _next_up(out, t):
    while not _link_up(out, t):
        t = max(b for a, b in out if a <= t < b)
    return t


def scenario_cache(scn):
    """World, prior grid and object metadata, built once per (layout, family)."""
    if "_cache" not in scn:
        w = World(scn["truth"])
        m = TimedOccMap(scn["truth"]["corridor"]["bounds"], w.half_width)
        pL, pO, pD = prior_grid(scn["prior_stems"], m)
        meta = [("wall" if s.get("wall") else "stem", False) for s in scn["truth"]["stems"]]
        for b in scn["truth"]["branches"]:
            p0, p1 = np.array(b["p0"]), np.array(b["p1"])
            n = max(2, int(math.ceil(np.linalg.norm(p1 - p0) / 0.1)) + 1)
            z = p0[2] + np.linspace(0, 1, n) * (p1[2] - p0[2])
            if (np.abs(z - w.fly_h) < w.slab_half).any():
                meta.append(("branch", bool(b.get("scenario_block"))))
        scn["_cache"] = {"world": w, "prior": (pL, pO, pD), "meta": meta}
    return scn["_cache"]


def run(scn, stack_name, cond, run_seed, params=None):
    C = scenario_cache(scn)
    world, (pL, pO, pD), meta = C["world"], C["prior"], C["meta"]
    veh = load_params()
    stack = Stack(stack_name, veh, params)
    r_sense, r_comp, r_wind, r_det = _rng(run_seed, 1), _rng(run_seed, 2), _rng(run_seed, 3), _rng(run_seed, 4)
    r_gpu, r_loc, r_net, r_rep = _rng(run_seed, 5), _rng(run_seed, 6), _rng(run_seed, 7), _rng(run_seed, 8)
    bounds = scn["truth"]["corridor"]["bounds"]
    acc = TimedOccMap(bounds, world.half_width)
    eph = TimedOccMap(bounds, world.half_width) if stack_name == REACTIVE else None
    goal = world.goal
    v = Vehicle(veh, *world.start, yaw=0.0, wind=cond["wind"])
    st_cfg0, li_cfg0, rgb = dict(S.STEREO), dict(S.LIDAR), dict(S.RGB)
    st_period, li_period, det_period = 1.0 / st_cfg0["rate_hz"], 1.0 / li_cfg0["rate_hz"], 1.0 / rgb["rate_hz"]
    period = 0.05
    t_inf, f_det = DETECTOR[cond["detector"]]
    rho = t_inf * f_det
    tlim = cond["time_limit_s"]
    outages = _outages(r_net, tlim + 10) if cond["netloss"] else []
    # estimator state
    ex = ey = 0.0
    last_loc_x, last_loc_y = v.x, v.y
    delta = align_offset(scn["align"], v.x)
    rep_sigma, rep_t = 0.1, 0.0
    next_rep = 0.0
    agree_ema, q_obs_ema = 0.7, 1.0
    q_st, q_li = 1.0, 1.0
    # loop state
    t, step = 0.0, 0
    next_st, next_li, next_det, next_plan = 0.0, 0.013, 0.021, 0.06
    queue, pending = [], None
    latest = {"stereo": None, "lidar": None}
    vcx = vcy = 0.0
    yaw_cmd = 0.0
    t_cmd = 0.0
    wd_active = False
    n_wd = n_nofeas_ev = n_cycles = n_fb_ev = 0
    nofeas_prev = fb_prev = False
    ages, comps = [], []
    min_clear = math.inf
    near_zone, n_near = False, 0
    speeds = []
    path_len = 0.0
    x_max = v.x
    t_back = t_stop = 0.0
    n_stops, stopped = 0, False
    last_prog_t, last_prog_x = 0.0, v.x
    outcome, contact = None, None
    nT = len(world.targets)
    hits = np.zeros(nT, int)
    conf_t = np.full(nT, np.inf)
    minr = np.full(nT, np.inf)
    px_prev, py_prev = v.x, v.y
    wbuf = []
    v_lim = cond["v_lim"]
    while True:
        xp = v.x
        # ---------------- estimator processes
        ds = math.hypot(v.x - last_loc_x, v.y - last_loc_y)
        if ds > 0.25:
            lz = scn["loc_zone"]
            sig_loc = lz[2] if (lz and lz[0] <= v.x <= lz[1]) else 0.03
            ex += sig_loc * math.sqrt(ds) * r_loc.standard_normal()
            ey += sig_loc * math.sqrt(ds) * r_loc.standard_normal()
            last_loc_x, last_loc_y = v.x, v.y
        lz = scn["loc_zone"]
        sigma_loc_rate = lz[2] if (lz and lz[0] <= v.x <= lz[1]) else 0.03
        oz = scn["obs_zone"]
        in_oz = bool(oz and oz[0] <= v.x <= oz[1])
        # ---------------- sensors
        if t >= next_st:
            cfg = st_cfg0
            if in_oz:
                cfg = dict(st_cfg0, invalid_p=oz[2])
            fr = S.stereo_frame(world, t, v.x, v.y, v.yaw, v.vx, v.vy, v.yaw_rate, r_sense, cfg)
            queue.append((t + cfg["latency_s"], fr, ex, ey))
            next_st += st_period
        if t >= next_li:
            cfg = li_cfg0
            if in_oz:
                cfg = dict(li_cfg0, pts_per_s=li_cfg0["pts_per_s"] * oz[3])
            fr = S.lidar_frame(world, t, v.x, v.y, v.yaw, r_sense, cfg)
            fr.carve = fr.carve  # noqa
            queue.append((t + cfg["latency_s"], fr, ex, ey, cfg["pts_per_s"] / li_cfg0["pts_per_s"]))
            next_li += li_period
        if t >= next_det and nT:
            _detect(world, v, t, rgb, r_det, hits, conf_t, minr, t_inf * (1.0 + rho))
            next_det += det_period
        # ---------------- alignment-quality report (over the network)
        if t >= next_rep:
            next_rep += 1.0
            if not outages or _link_up(outages, t):
                d_lag = align_offset(scn["align"], max(xp - 1.0 * max(v.speed, 0.0), 0.0))
                rep_sigma = 0.1 + math.hypot(d_lag[0] - scn["align"]["c0"][0], d_lag[1] - scn["align"]["c0"][1]) * \
                    float(r_rep.lognormal(0.0, 0.5))
                rep_t = t
        # ---------------- planner cycle
        if t >= next_plan:
            ready = [q for q in queue if q[0] <= t]
            queue = [q for q in queue if q[0] > t]
            newest = -math.inf
            hxs, hys = [], []
            delta = align_offset(scn["align"], v.x)
            si = int(round((ex + delta[0]) / acc.res))
            sj = int(round((ey + delta[1]) / acc.res))
            for sname in ("stereo", "lidar"):
                fs = [q for q in ready if q[1].sensor == sname][-3:]
                for q in fs:
                    f = q[1]
                    hx, hy = acc.integrate(f, t, q[2], q[3])
                    hxs.append(hx)
                    hys.append(hy)
                    newest = max(newest, f.t_acq)
                    latest[sname] = q
                    if sname == "stereo":
                        q_st = float(np.mean(f.carve >= 0.0)) / (1.0 - st_cfg0["invalid_p"])
                    else:
                        q_li = q[4]
            if newest == -math.inf:
                newest = max([q[1].t_acq for q in latest.values() if q is not None] + [-math.inf])
            q_obs_ema = 0.7 * q_obs_ema + 0.3 * min(1.0, 0.5 * (q_st + q_li))
            if hxs:
                hx, hy = np.concatenate(hxs), np.concatenate(hys)
                if len(hx) >= 5:
                    ii = ((hx - acc.x0) / acc.res).astype(np.int64) - si
                    jj = ((hy - acc.y0) / acc.res).astype(np.int64) - sj
                    ok = (ii >= 0) & (ii < acc.nx) & (jj >= 0) & (jj < acc.ny)
                    if ok.sum() >= 5:
                        a = float(np.mean(pD[ii[ok], jj[ok]] < 0.35))
                        agree_ema = 0.8 * agree_ema + 0.2 * a
            if eph is not None:
                eph.clear()
                for q in latest.values():
                    if q is not None:
                        eph.integrate(q[1], t, q[2], q[3])
            sig = {"sigma_align_rep": rep_sigma, "rep_age": t - rep_t, "agree": agree_ema, "q_obs": q_obs_ema,
                   "sigma_loc_rate": sigma_loc_rate}
            xh, yh = v.x + ex, v.y + ey
            gx, gy = goal[0] + ex + delta[0], goal[1] + ey + delta[1]
            vcap_ext = math.inf
            if cond["det_beta"] > 0:
                sp_ = max(v.speed, 0.1)
                vals = [hint_value(scn["hints"], v.x + ex + s * v.vx / sp_, v.y + ey + s * v.vy / sp_) for s in (0.0, 3.0, 6.0)]
                vcap_ext = v_lim * (1.0 - cond["det_beta"] * max(vals))
            out = stack.cycle(t, xh, yh, v.vx, v.vy, (gx, gy), v_lim, acc, eph, (pL, pO), (si, sj), sig, vcap_ext)
            c = S.compute_time(cond["latency_tail"], *r_comp.random(3))
            u1, u2 = r_gpu.random(2)
            if rho > 0:
                c = c * (1.0 + KAPPA * rho / (1.0 - rho)) + (u2 * t_inf if u1 < rho else 0.0)
            comps.append(c)
            n_cycles += 1
            if newest > -math.inf:
                age = t + c - newest
                ages.append(age)
                stack.rule.observe_age(age)
            nf = not out["feasible_any"]
            n_nofeas_ev += int(nf and not nofeas_prev)
            nofeas_prev = nf
            n_fb_ev += int(out["fallback"] and not fb_prev)
            fb_prev = out["fallback"]
            pending = (t + c, out["v_cmd"] * math.cos(out["theta"]), out["v_cmd"] * math.sin(out["theta"]), out["theta"])
            next_plan = t + max(period, c)
        if pending is not None and t >= pending[0]:
            _, vcx, vcy, yaw_cmd = pending
            pending = None
            t_cmd = t
            wd_active = False
        if stack.watchdog and not wd_active and t - t_cmd > WD_S and (vcx or vcy):
            vcx = vcy = 0.0
            wd_active = True
            n_wd += 1
        # ---------------- plant
        if not wbuf:
            wbuf = r_wind.standard_normal(4096).tolist()[::-1]
        nx_, ny_ = wbuf.pop(), wbuf.pop()
        v.step(DT, vcx, vcy, yaw_cmd, nx_, ny_)
        t += DT
        step += 1
        sp = v.speed
        if step % 4 == 0:
            speeds.append(sp)
            seg = math.hypot(v.x - px_prev, v.y - py_prev)
            path_len += seg
            cl, ci = _swept(world, px_prev, py_prev, v.x, v.y)
            cl -= veh["r_body"]
            px_prev, py_prev = v.x, v.y
            min_clear = min(min_clear, cl)
            if 0 <= cl < NEAR_MISS_M and not near_zone:
                near_zone = True
                n_near += 1
            elif cl >= NEAR_MISS_M + 0.05:
                near_zone = False
            if t > 1.0:
                if v.x < x_max - 0.3:
                    t_back += 4 * DT
                if sp < 0.2:
                    t_stop += 4 * DT
                    if not stopped:
                        stopped, n_stops = True, n_stops + 1
                elif sp > 0.5:
                    stopped = False
            x_max = max(x_max, v.x)
            if cl < 0:
                oid = int(world.cobj[ci])
                kind, block = meta[oid] if oid < len(meta) else ("?", False)
                contact = {"speed": sp, "kind": kind, "diam": float(world.odiam[oid]), "block": block, "t": t}
                outcome = "collision"
                break
            if v.x - last_prog_x > 1.0:
                last_prog_t, last_prog_x = t, v.x
            if v.x >= goal[0] - 1.0:
                outcome = "success"
                break
            if t - last_prog_t > cond["stuck_s"] or t > tlim:
                outcome = "stuck"
                break
    # ---------------- detection / reporting
    along = minr <= 15.0
    n_along = int(along.sum())
    confirmed = np.isfinite(conf_t)
    deliv = np.array([_next_up(outages, ct) if (np.isfinite(ct) and outages) else ct for ct in conf_t]) if nT else np.zeros(0)
    rec = float((confirmed & along).sum() / n_along) if n_along else float("nan")
    res = {"outcome": outcome, "success": outcome == "success", "collision": outcome == "collision",
           "stuck": outcome == "stuck", "contact_speed": contact["speed"] if contact else None,
           "contact_kind": contact["kind"] if contact else None, "contact_block": contact["block"] if contact else None,
           "contact_diam": contact["diam"] if contact else None,
           "min_clear": float(min_clear), "near_misses": n_near, "time_s": t, "progress_m": float(v.x),
           "progress_rate": float(max(v.x - world.start[0], 0.0) / max(t, 1e-6)),
           "path_len": path_len, "wasted_m": float(max(path_len - max(v.x - world.start[0], 0.0), 0.0)),
           "mean_speed": float(np.mean(speeds)) if speeds else 0.0,
           "age_p50": _q(ages, 0.5), "age_p95": _q(ages, 0.95), "age_p99": _q(ages, 0.99),
           "compute_p99": _q(comps, 0.99), "tau_final": stack.rule.tau,
           "watchdog": n_wd, "nofeasible_events": n_nofeas_ev, "fallback_events": n_fb_ev,
           "fallback_frac": stack.fallback_cycles / max(n_cycles, 1),
           "fallback_activations": n_wd + n_nofeas_ev + n_fb_ev,
           "t_backtrack": t_back, "t_stopped": t_stop, "stops": n_stops, "cycles": n_cycles,
           "mean_w_prior": stack.trace["w_prior"] / max(stack.trace["n"], 1),
           "mean_v_cap": stack.trace["v_cap"] / max(stack.trace["n"], 1),
           "targets": nT, "targets_along": n_along, "recall": rec,
           "final_err_e": math.hypot(ex, ey), "final_delta": math.hypot(*delta)}
    for T in REPORT_WINDOWS:
        res["reported_%d" % int(T)] = float((deliv <= T).sum() / nT) if nT else float("nan")
    res["reported_all"] = float(np.isfinite(deliv).sum() / nT) if nT else float("nan")
    res["report_delay_mean"] = float(np.mean((deliv - conf_t)[np.isfinite(conf_t)])) if confirmed.any() else 0.0
    return res


def _q(a, q):
    return float(np.quantile(np.asarray(a), q)) if len(a) else float("nan")


def _swept(world, x0, y0, x1, y1):
    mx, my = 0.5 * (x0 + x1), 0.5 * (y0 + y1)
    seg = math.hypot(x1 - x0, y1 - y0)
    idx = world.query(mx, my, 0.5 * seg + 1.5)
    if len(idx) == 0:
        return math.inf, -1
    idx = idx[world.hard[idx]]
    if len(idx) == 0:
        return math.inf, -1
    cx, cy = world.cx[idx], world.cy[idx]
    if seg < 1e-9:
        d = np.hypot(cx - x1, cy - y1)
    else:
        ux, uy = (x1 - x0) / seg, (y1 - y0) / seg
        s = np.clip((cx - x0) * ux + (cy - y0) * uy, 0.0, seg)
        d = np.hypot(cx - (x0 + s * ux), cy - (y0 + s * uy))
    d = d - world.cr[idx]
    k = int(np.argmin(d))
    return float(d[k]), int(idx[k])


def _detect(world, v, t, cfg, rng, hits, conf_t, minr, latency):
    half = math.radians(0.5 * cfg["hfov_deg"])
    for k, (tx, ty, sz) in enumerate(world.targets):
        dx, dy = tx - v.x, ty - v.y
        r = math.hypot(dx, dy)
        minr[k] = min(minr[k], r)
        if r > cfg["range_max"] or hits[k] >= cfg["confirm"]:
            continue
        b = (math.atan2(dy, dx) - v.yaw + math.pi) % (2 * math.pi) - math.pi
        if abs(b) > half:
            continue
        vis = S.target_visibility(world, v.x, v.y, tx, ty, sz)
        if vis < 0.2:
            continue
        vlat = abs(v.vx * dy - v.vy * dx) / max(r, 1e-6)
        if rng.random() < S.detection_p(r, sz, vis, vlat, v.yaw_rate, cfg):
            hits[k] += 1
            if hits[k] >= cfg["confirm"]:
                conf_t[k] = t + latency
