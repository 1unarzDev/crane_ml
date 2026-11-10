# SPDX-License-Identifier: GPL-3.0-or-later
"""D3 closed-loop episode (DEV): e3_episode.py (E3-frozen, not edited) copied with hooks for (i) counterfactuals
(cond["cf"]: lat0, lat50, perc, map, range, brake), (ii) plant integration rate decoupled from the 5 ms event tick
(cond["plant_hz"]: 200/100/50), (iii) compute levels chosen per cycle by D3 stacks (stacks/compute_gov.py), and
(iv) a NavPilot analogue of the harness `brake_infeasible` label. With cf=() , plant_hz=200 and an E3 stack the output
equals e3_episode.run (tested: experiments/test_d3.py).

Original E3 docstring:
Experiment-3 closed-loop episode: a copy of e2_episode.py (E2-frozen, not edited) with hooks for AdvStack
(RangeOccMap, current command + compute-time feedback for latency compensation, decoupled yaw) and a mechanism
metric: range at which each scenario-block thin branch first appeared occupied in the fresh map.

Original E2 docstring:
Experiment-2 closed-loop episode (plant 200 Hz, sensors at their rates, planner cycles with sampled compute).

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
import os

import numpy as np

from navpilot import sensors as S
from navpilot.forest import World
from navpilot.vehicle import Vehicle, load_params
from stacks.advanced import METHODS, AdvStack, RangeOccMap
from stacks.compute_gov import D3_METHODS, LEVELS, CCStack, level_compute
from stacks.core import TimedOccMap, brake_table
from stacks.policies import REACTIVE, Stack

from .scenarios import align_offset, hint_value, prior_grid

CAD_PROFILE = os.path.normpath(os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "..", "Aerial", "profiles",
                                            "aos7_v5_f90_6s.json"))   # CAD/URDF AOS7 (1.843 kg, T/W 4.79), live default of Tools/Aerial
_LAST_PROFILE = None
DT = 0.005   # event tick (sensors, planner, command latch); the plant may integrate at a coarser step (plant_hz)
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
    cf = set(cond.get("cf", ()))
    if "map" in cf:
        scn = perfect_map(scn)
    C = scenario_cache(scn)
    world, (pL, pO, pD), meta = C["world"], C["prior"], C["meta"]
    veh = load_params(profile_path=CAD_PROFILE) if cond.get("profile") == "cad" else load_params()   # default = v1 (pinned in navpilot/vehicle.py)
    global _LAST_PROFILE
    if _LAST_PROFILE != cond.get("profile"):   # stacks.core._BRAKE is keyed without drag/mass: reset when the profile changes
        from stacks import core as _core
        _core._BRAKE.clear()
        _LAST_PROFILE = cond.get("profile")
    if "brake" in cf:   # ideal braking: 2.5x horizontal authority, 3x jerk (the stopping rule sees it too)
        veh = dict(veh, a_h_max=veh["a_h_max"] * 2.5, jerk_max=veh["jerk_max"] * 3.0, tau_track=0.15)
    kplant = max(1, int(round(200.0 / float(cond.get("plant_hz", 200.0)))))
    cc = stack_name in D3_METHODS
    adv = stack_name in METHODS or cc
    stack = (CCStack(stack_name, veh, params) if cc else AdvStack(stack_name, veh, params)) if adv else Stack(stack_name, veh, params)
    lev = 0
    vs_b, ds_b = brake_table(veh)
    r_sense, r_comp, r_wind, r_det = _rng(run_seed, 1), _rng(run_seed, 2), _rng(run_seed, 3), _rng(run_seed, 4)
    r_gpu, r_loc, r_net, r_rep = _rng(run_seed, 5), _rng(run_seed, 6), _rng(run_seed, 7), _rng(run_seed, 8)
    bounds = scn["truth"]["corridor"]["bounds"]
    acc = (RangeOccMapCap if "range" in cf else RangeOccMap)(bounds, world.half_width)
    blk = _block_circles(scn, world)
    blk_first = [None] * len(blk)
    eph = TimedOccMap(bounds, world.half_width) if stack_name == REACTIVE else None
    goal = world.goal
    v = Vehicle(veh, world.start[0], world.start[1] + float(cond.get("eps", 0.0)), yaw=0.0, wind=cond["wind"])   # eps: chaos-floor control
    st_cfg0, li_cfg0, rgb = dict(S.STEREO), dict(S.LIDAR), dict(S.RGB)
    if "lat0" in cf:
        st_cfg0["latency_s"] = 0.0
        li_cfg0["latency_s"] = 0.0
    if "perc" in cf:    # no dropout: every thin object matched, no invalid pixels, dense LiDAR; exact local estimate
        st_cfg0.update(w50_px=0.01, invalid_p=0.0)
        li_cfg0["pts_per_s"] *= 20.0
    if "range" in cf:   # unlimited sensing/free-evidence range (map carve cap lifted to 60 m)
        st_cfg0.update(range_max=60.0, free_evidence_m=60.0)
        li_cfg0["range_max"] = 60.0
        acc_cap = 60.0
    else:
        acc_cap = 12.0
    st_period, li_period, det_period = 1.0 / st_cfg0["rate_hz"], 1.0 / li_cfg0["rate_hz"], 1.0 / rgb["rate_hz"]
    period = 0.05
    t_inf, f_det0 = DETECTOR[cond["detector"]]
    f_det = f_det0
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
    bi = False
    nT = len(world.targets)
    hits = np.zeros(nT, int)
    conf_t = np.full(nT, np.inf)
    minr = np.full(nT, np.inf)
    px_prev, py_prev = v.x, v.y
    wbuf = []
    lev_hist = [0, 0, 0]
    nf_on = None
    n_dec = 0
    a5 = math.exp(-DT / v.w_tau) if v.w_tau > 0 else 0.0
    wn_x, wn_y = [], []
    v_lim = cond["v_lim"]
    while True:
        xp = v.x
        # ---------------- estimator processes
        ds = math.hypot(v.x - last_loc_x, v.y - last_loc_y)
        if ds > 0.25 and "perc" not in cf:
            lz = scn["loc_zone"]
            sig_loc = lz[2] if (lz and lz[0] <= v.x <= lz[1]) else 0.03
            ex += sig_loc * math.sqrt(ds) * r_loc.standard_normal()
            ey += sig_loc * math.sqrt(ds) * r_loc.standard_normal()
            last_loc_x, last_loc_y = v.x, v.y
        lz = scn["loc_zone"]
        sigma_loc_rate = lz[2] if (lz and lz[0] <= v.x <= lz[1]) else 0.03
        oz = scn["obs_zone"]
        in_oz = bool(oz and oz[0] <= v.x <= oz[1]) and "perc" not in cf
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
            next_det += det_period / (LEVELS[lev]["det_scale"] if cc else 1.0)
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
                fs = [q for q in ready if q[1].sensor == sname][-LEVELS[lev]["frames"]:]
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
            if adv:
                out = stack.cycle(t, xh, yh, v.vx, v.vy, (gx, gy), v_lim, acc, eph, (pL, pO), (si, sj), sig, vcap_ext,
                                  cmd=(vcx, vcy))
            else:
                out = stack.cycle(t, xh, yh, v.vx, v.vy, (gx, gy), v_lim, acc, eph, (pL, pO), (si, sj), sig, vcap_ext)
            for kb, (bx, by) in enumerate(blk):
                if blk_first[kb] is None:
                    ii = ((bx + ex - acc.x0) / acc.res).astype(np.int64)
                    jj = ((by + ey - acc.y0) / acc.res).astype(np.int64)
                    okb = (ii >= 0) & (ii < acc.nx) & (jj >= 0) & (jj < acc.ny)
                    if okb.any() and (acc.L[ii[okb], jj[okb]] > acc.cfg["l_occ"]).any():
                        blk_first[kb] = float(np.min(np.hypot(bx - v.x, by - v.y)))
            u3 = r_comp.random(3)
            c = level_compute(cond["latency_tail"], lev, *u3)
            u1, u2 = r_gpu.random(2)
            if rho > 0:
                c = c * (1.0 + KAPPA * rho / (1.0 - rho)) + (u2 * t_inf if u1 < rho else 0.0)
            if "lat0" in cf:
                c = 0.002
            elif "lat50" in cf:
                c = S.LATENCY_TAILS[cond["latency_tail"]]["median_s"]
            comps.append(c)
            if adv:
                stack.observe_compute(c) if not cc else stack.observe_compute(c, lev)
            n_cycles += 1
            if newest > -math.inf:
                age = t + c - newest
                ages.append(age)
                stack.rule.observe_age(age)
                if cc:
                    stack.observe_age_split(age, c)
            if cc:
                lev = out["level_next"]
                f_det = f_det0 * LEVELS[lev]["det_scale"]
                rho = t_inf * f_det
                lev_hist[lev] += 1
            nf = not out["feasible_any"]
            if nf and not nofeas_prev:
                sp_now = v.speed
                ux, uy = (v.vx / sp_now, v.vy / sp_now) if sp_now > 0.2 else (1.0, 0.0)
                clr_on, _ = _swept(world, v.x, v.y, v.x + 25.0 * ux, v.y + 25.0 * uy)
                nf_on = (t, sp_now, clr_on - veh["r_body"], float(np.interp(sp_now, vs_b, ds_b)))
            n_nofeas_ev += int(nf and not nofeas_prev)
            nofeas_prev = nf
            n_fb_ev += int(out["fallback"] and not fb_prev)
            fb_prev = out["fallback"]
            pending = (t + c, out["v_cmd"] * math.cos(out["theta"]), out["v_cmd"] * math.sin(out["theta"]),
                       out.get("yaw", out["theta"]))
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
        wn_x.append(wbuf.pop())
        wn_y.append(wbuf.pop())
        t += DT
        step += 1
        if step % kplant == 0:   # plant integration step = kplant event ticks (ZOH command); OU noise aggregated exactly
            dtp = kplant * DT
            if kplant == 1:
                nx_, ny_ = wn_x[0], wn_y[0]
            else:
                ak = math.exp(-dtp / v.w_tau)
                sc = math.sqrt(1.0 - a5 * a5) / math.sqrt(1.0 - ak * ak)
                nx_ = sc * sum(a5 ** (kplant - 1 - j) * wn_x[j] for j in range(kplant))
                ny_ = sc * sum(a5 ** (kplant - 1 - j) * wn_y[j] for j in range(kplant))
            wn_x, wn_y = [], []
            v.step(dtp, vcx, vcy, yaw_cmd, nx_, ny_)
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
                bi = bool(nf_on is not None and t - nf_on[0] <= 1.0 and nf_on[1] > 1.0 and nf_on[2] < nf_on[3])
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
           "final_err_e": math.hypot(ex, ey), "final_delta": math.hypot(*delta),
           "blk_det_range": [b if b is None else round(b, 2) for b in blk_first],
           "untrusted_cut": stack.trace.get("untrusted_cut", 0), "x_contact": float(v.x) if contact else None,
           "brake_infeasible": bool(bi), "lev_hist": lev_hist, "lev_mean": (lev_hist[1] + 2 * lev_hist[2]) / max(sum(lev_hist), 1),
           "mean_det_hz": f_det0}
    for T in REPORT_WINDOWS:
        res["reported_%d" % int(T)] = float((deliv <= T).sum() / nT) if nT else float("nan")
    res["reported_all"] = float(np.isfinite(deliv).sum() / nT) if nT else float("nan")
    res["report_delay_mean"] = float(np.mean((deliv - conf_t)[np.isfinite(conf_t)])) if confirmed.any() else 0.0
    return res


class RangeOccMapCap(RangeOccMap):
    """RangeOccMap whose carve cap (12 m in the frozen map code) is 60 m (cf 'range': unlimited sensing range)."""

    def integrate(self, fr, t, ex=0.0, ey=0.0):
        hx, hy = super().integrate(fr, t, ex, ey)            # frozen path (caps at 12 m)
        far = fr.carve > 12.0
        if far.any():
            ox, oy = fr.ox + ex, fr.oy + ey
            ang, car = fr.angles[far], np.minimum(fr.carve[far], 60.0)
            steps = np.arange(12.0, 60.0, 0.8 * self.res)
            msk = (steps[None, :] < car[:, None])
            px = (ox + steps[None, :] * np.cos(ang)[:, None])[msk]
            py = (oy + steps[None, :] * np.sin(ang)[:, None])[msk]
            i = ((px - self.x0) / self.res).astype(np.int64)
            j = ((py - self.y0) / self.res).astype(np.int64)
            ok = (i >= 0) & (i < self.nx) & (j >= 0) & (j < self.ny)
            i, j = i[ok], j[ok]
            c = self.cfg
            self.L[i, j] += c["l_miss"]
            self.obs[i, j] = True
            self.last[i, j] = t
            self.L[i, j] = np.maximum(self.L[i, j], c["l_min"])
            self.rmin[i, j] = np.minimum(self.rmin[i, j], 12.0 + 0 * i)
        return hx, hy


def perfect_map(scn):
    """cf 'map': prior = truth (stems and branch slab circles, no phantoms) and no alignment error."""
    if "_perfect" in scn:
        return scn["_perfect"]
    t = scn["truth"]
    ps = [{"x": s["x"], "y": s["y"], "dbh_m": s["dbh_m"]} for s in t["stems"]]
    for b in t["branches"]:
        p0, p1 = np.array(b["p0"]), np.array(b["p1"])
        n = max(2, int(math.ceil(np.linalg.norm(p1 - p0) / 0.1)) + 1)
        for u in np.linspace(0.0, 1.0, n):
            q = p0 + u * (p1 - p0)
            if abs(q[2] - 2.0) < 0.35:   # FLY_H, as in scenarios.build S0
                ps.append({"x": float(q[0]), "y": float(q[1]), "dbh_m": float(b["d0_m"] + u * (b["d1_m"] - b["d0_m"]))})
    n = dict(scn)
    n.pop("_cache", None)
    n["prior_stems"] = ps
    n["align"] = {"kind": "const", "c0": (0.0, 0.0), "sigma0": 0.1}
    scn["_perfect"] = n
    return n


def _block_circles(scn, world):
    """Slab circles (truth x, y arrays) of each scenario-block thin branch."""
    out = []
    for b in scn["truth"]["branches"]:
        if not b.get("scenario_block"):
            continue
        p0, p1 = np.array(b["p0"]), np.array(b["p1"])
        n = max(2, int(math.ceil(np.linalg.norm(p1 - p0) / 0.1)) + 1)
        u = np.linspace(0.0, 1.0, n)
        q = p0[None, :] + u[:, None] * (p1 - p0)[None, :]
        k = np.abs(q[:, 2] - world.fly_h) < world.slab_half
        # interior samples only: drop samples within 0.4 m of any stem/wall surface (attachment points would
        # register as "detected" through the already-occupied wall/stem cells)
        st = scn["truth"]["stems"]
        if st:
            sx = np.array([s_["x"] for s_ in st]); sy = np.array([s_["y"] for s_ in st])
            sr = np.array([0.5 * s_["dbh_m"] for s_ in st])
            dmin = np.min(np.hypot(q[:, 0, None] - sx[None, :], q[:, 1, None] - sy[None, :]) - sr[None, :], axis=1)
            k &= dmin > 0.4
        if k.any():
            out.append((q[k, 0], q[k, 1]))
    return out


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
