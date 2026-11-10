# SPDX-License-Identifier: GPL-3.0-or-later
"""Mission loop (1 s ticks): truth-side drone execution (motion over the true graph, energy, odometry drift,
alignment error, observations), the radio network, base fusion / coordinator, metrics and decision logs.

Planners see only AgentView. The base sees only what arrives over the radio. Truth is used here for physics,
observations and scoring, and by the ORACLE policy (labelled) through an oracle-built PublicInfo.
"""
import copy
import json
import math

import numpy as np

from . import world as W
from . import comms as C
from . import policies as P
from .agent import AgentView, Knowledge, Candidate, merge_payload, TAU, sigm


class TruthDrone:
    def __init__(self, did, cell, r, align_mult=1.0):
        self.id = did
        self.cell = cell
        self.err_odom = np.zeros(2)
        self.theta = r.normal(0, math.radians(1.0) * align_mult)
        self.t_align = r.normal(0, 0.5, 2)
        bd = r.uniform(0, 2 * math.pi)
        self.bias = np.array([math.cos(bd), math.sin(bd)])
        self.action = None
        self.t_done = 0
        self.energy = 0.0
        self.dist = 0.0
        self.nearmiss = 0.0
        self.landed = False
        self.dead = False

    def err(self, pose, base):
        p = pose - base
        c, s = math.cos(self.theta), math.sin(self.theta)
        rot = np.array([c * p[0] - s * p[1], s * p[0] + c * p[1]]) - p
        return self.err_odom + rot + self.t_align


def _oracle_public(w):
    pub = copy.copy(w.public)
    p0 = np.full(W.NC, 1e-4)
    p0[w.target_cell] = 1.0
    p0[w.blocked] = 0
    pub.p0 = p0 / p0.sum()
    pub.blocked_map = w.blocked.copy()
    pub.rho_map = w.rho.copy()
    pub.forecast_up = w.margin_up.copy()
    pub.forecast_down = w.margin_down.copy()
    pub.forecast_sigma = 1.0
    pub.alpha_db_per_m = w.alpha
    return pub


def run_mission(layout_seed, rep, policy, sc, log_path=None, allow_heldout=False, obs=None, priority=True,
                forced_option=None, world=None, return_world=False):
    w = world or W.World(layout_seed, sc, obs=obs, allow_heldout=allow_heldout)
    obs = w.obs
    pol = P.make(policy, forced_option=forced_option)
    pub = _oracle_public(w) if pol.uses_truth else w.public
    H = int(sc.horizon_s)
    ro = W.rng(layout_seed, "obs", rep)
    rdr = W.rng(layout_seed, "drift", rep)
    agents, truth = {}, {}
    for d in (1, 2):
        av = AgentView(d, pub, obs, pol)
        av.outbox_requests = []
        av.custody_pending = 0
        av.last_search_rate = 1e-4
        av.partner_lease_exp = 1e9
        av.oracle_views = w.view_good.copy() if pol.uses_truth else None
        av.cand_by_id = (lambda a: (lambda cid: next((c for c in a.k.cands if c.id == cid), None)))(av)
        av.cand_by_id_or_est = (lambda a: (lambda p: a.k.find_cand(np.array(p["est"]))))(av)
        av.partner_cell = (lambda a: (lambda: int(a.partner_hb["cell"]) if a.partner_hb else a.pub.start_cells[a.partner - 1]))(av)
        av.partner_goal_cell = (lambda a: (lambda: a.partner_hb.get("goal") if a.partner_hb and a.policy.comm != "none" else None))(av)
        av.pdq = {v: np.array([obs.expected_pd(av.k.rho[c], v, W.r_eff(av.k.rho[c])) for c in range(W.NC)]) for v in P.SPEEDS}
        av.scan_t = {v: np.array([W.scan_time(av.k.rho[c], v) for c in range(W.NC)]) for v in P.SPEEDS}
        agents[d] = av
        truth[d] = TruthDrone(d, av.cell, rdr, sc.align_mult)
        av.sig_th *= sc.align_mult
    net = C.Network(w, layout_seed, rep, H, priority=priority, forced=sc.asym_outage)
    base = Knowledge(pub, obs)
    base_hb = {1: None, 2: None}
    base_assign_t = -1e9
    M = {"policy": policy, "layout": int(layout_seed), "rep": int(rep), "split": w.split, "t_first_det": None,
         "t_local_conf": None, "t_delivery": None, "t_base_conf_any": None, "false_alerts": 0, "true_alerts": 0,
         "false_confirm": 0, "dup_area_m2": 0.0, "scan_area_m2": 0.0, "disagree_m2_mean": 0.0, "near_miss": 0.0,
         "energy_wh": 0.0, "t_urgent": None, "t_verify_after_urgent": None, "base_conf_err_m": None,
         "glimpse_in_outage": None, "battery_rtb": 0, "new_obstacles": 0, "forced_option": forced_option,
         "label_at_delivery": None, "local_conf_label": None}
    scanned_by = np.zeros((W.NC, 3), bool)
    dis_acc, dis_n = 0.0, 0
    logf = open(log_path, "w") if log_path else None
    E = sc.err_tol_m

    if sc.e1_forced:
        _e1_setup(w, agents, truth, ro, M, obs, sc)
    if pol.uses_truth:  # ORACLE also knows which lane orientation sees a good view of the person
        tc = w.target_cell
        re_ = W.r_eff(w.rho[tc])
        lo_ = W.CENTERS[tc] - W.CELL / 2
        for o in (0, 1):
            view = o + 2 * int(((w.target_xy[1 - o] - lo_[1 - o]) % (2 * re_)) > re_)
            if not w.view_good[view]:
                for d in (1, 2):
                    agents[d].k.S[tc, o] = -30.0

    def true_pos(d):
        return agents[d].pose + truth[d].err(agents[d].pose, w.base_xy)

    def record_alert(d, kind, est, conf, view, t, rng_m):
        av = agents[d]
        c = W.xy_cell(est)
        cand = av.k.find_cand(est)
        if cand is None:
            cand = Candidate(1000 * d + av.k.next_cid, est, c, av.k.lo_prior_for(c, 4.0), t)
            av.k.next_cid += 1
            av.k.cands.append(cand)
        e = cand.drone_ev(d)
        same = view in e["views"]
        lr = obs.log_lr(conf)
        e["lo"] += lr * (0.5 if same else 1.0)
        e["views"].append(view)
        e["n"] += 1
        e["sum_est"] += est
        e["s_ind"] = obs.loc_sigma(rng_m, 2.0) ** 2
        e["s_com"] = av.pose_var()
        e["t"] = t
        cand.delivered_t = None
        av.events.append(("detection", cand.id))
        av.outbox_requests.append(("DET", cand.id))
        if kind == "true":
            M["true_alerts"] += 1
            if M["t_first_det"] is None:
                M["t_first_det"] = t
                M["glimpse_in_outage"] = bool(t - av.heard_base_t > 5)
            if conf >= 0.75 and M["t_urgent"] is None:
                M["t_urgent"] = t
        else:
            M["false_alerts"] += 1
        return cand

    def null_review(d, cand, view):
        av = agents[d]
        e = cand.drone_ev(d)
        n = e["nulls"].get(view, 0)
        rho = av.k.rho[cand.cell]
        q = av.k.g[cand.cell] * obs.expected_pd(rho, P.REVIEW_V, W.r_eff(rho), mode="review")
        e["lo"] += math.log(max(1 - q, 0.05)) * (1.0 if n == 0 else 0.3)
        e["nulls"][view] = n + 1
        e["t"] = t
        cand.delivered_t = None
        av.outbox_requests.append(("DET", cand.id))

    def complete(d, t):
        td, av = truth[d], agents[d]
        a = td.action
        k = a["kind"]
        if k == "move":
            if w.blocked[a["cell"]]:
                if av.k.mark_blocked(a["cell"]):
                    av.new_blk.append(a["cell"])
                    av.events.append(("obstacle", a["cell"]))
                    M["new_obstacles"] += 1
                return
            av.cell = a["cell"]
            av.pose = W.CENTERS[av.cell].copy()
        elif k == "scan":
            c, o, v = a["cell"], a["o"], a["v"]
            vv = min(v, W.vmax(w.rho[c]))
            q_exp = av.k.g_post(c, o) * 0 + obs.expected_pd(av.k.rho[c], vv, W.r_eff(av.k.rho[c]))
            M["scan_area_m2"] += W.CELL ** 2
            if scanned_by[c, 3 - d]:
                M["dup_area_m2"] += W.CELL ** 2
            scanned_by[c, d] = True
            e = td.err(av.pose, w.base_xy)
            lo = W.CENTERS[c] - W.CELL / 2 + e
            hi = lo + W.CELL
            re = W.r_eff(w.rho[c])
            s = 2 * re
            tx = w.target_xy
            detected = False
            u = ro.random()  # always drawn: the RNG stream must not depend on where the (hidden) person is
            if np.all(tx >= lo) and np.all(tx < hi):
                lat = abs(((tx[1 - o] - lo[1 - o]) % s) - s / 2) if True else 0.0
                rng_m = math.hypot(min(lat, s / 2), 2.0)
                view = o + 2 * int(((tx[1 - o] - lo[1 - o]) % s) > s / 2)
                pg = obs.p_detect_pass(rng_m, vv, w.rho[c], re)
                if w.view_good[view] and u < pg:
                    conf = obs.sample_conf("true", pg, ro)
                    est = tx - e + ro.normal(0, obs.loc_sigma(rng_m, vv), 2)
                    record_alert(d, "true", est, conf, view, t, rng_m)
                    detected = True
                elif not w.view_good[view] and u < obs.p_glimpse_bad(pg):
                    conf = obs.sample_conf("glimpse", pg, ro)
                    est = tx - e + ro.normal(0, obs.loc_sigma(rng_m, vv) * 1.5, 2)
                    record_alert(d, "true", est, conf, view, t, rng_m)
                    detected = True
            dc = w.decoys.get(c)
            if dc is not None and np.all(dc[0] >= lo) and np.all(dc[0] < hi):
                view = o + 2 * int(((dc[0][1 - o] - lo[1 - o]) % s) > s / 2)
                if dc[1][view] and ro.random() < obs.p_decoy_detect(vv) * sc.decoy_mult:
                    record_alert(d, "decoy", dc[0] - e + ro.normal(0, 1.0, 2), obs.sample_conf("decoy", 0, ro),
                                 view, t, 5.0)
            if ro.random() < obs.p_false_alert(w.rho[c], vv, 400.0) * sc.fa_mult:
                p = lo + ro.random(2) * W.CELL
                record_alert(d, "fa", p - e, obs.sample_conf("fa", 0, ro), int(ro.integers(4)), t, 5.0)
            if not detected:
                av.k.add_null_scan(c, o, q_exp)
            else:
                av.k.scanned[c] = True
                av.k.n_scans[c, o] += 1
            av.new_cov.append((c, o, q_exp if not detected else 0.0))
        elif k == "review":
            cand = av.cand_by_id(a["cid"])
            if cand is None:
                return
            cand.reviews += 1
            view = a["view"]
            e = td.err(av.pose, w.base_xy)
            look = cand.est + e  # where the drone actually looks (true frame)
            rr = 6.0
            got = False
            u = ro.random()
            if np.hypot(*(w.target_xy - look)) < rr:
                pg = obs.p_detect_pass(4.0, P.REVIEW_V, w.rho[w.target_cell], W.r_eff(w.rho[w.target_cell]))
                if w.view_good[view] and u < pg:
                    conf = obs.sample_conf("true", pg, ro)
                    est = w.target_xy - e + ro.normal(0, obs.loc_sigma(4.0, P.REVIEW_V), 2)
                    record_alert(d, "true", est, conf, view, t, 4.0)
                    got = True
                if M["t_urgent"] is not None and M["t_verify_after_urgent"] is None:
                    M["t_verify_after_urgent"] = t
            for c2, dc in w.decoys.items():
                if np.hypot(*(dc[0] - look)) < rr and dc[1][view] and ro.random() < obs.p_decoy_detect(1.5) * sc.decoy_mult:
                    record_alert(d, "decoy", dc[0] - e + ro.normal(0, 0.8, 2), obs.sample_conf("decoy", 0, ro), view, t, 4.0)
                    got = True
            if ro.random() < obs.p_false_alert(w.rho[cand.cell], 1.5, 113.0) * sc.fa_mult:
                record_alert(d, "fa", look - e + ro.normal(0, 3.0, 2), obs.sample_conf("fa", 0, ro), view, t, 5.0)
            if not got:
                null_review(d, cand, view)
        elif k == "land":
            td.landed = True
        # landmark re-alignment (loop closure against a mapped clearing; stale map -> wrong fix, overconfident)
        if k in ("move", "scan") and av.cell in w.clear_cells:
            p = av.pose - w.base_xy
            c_, s_ = math.cos(td.theta), math.sin(td.theta)
            rot = np.array([c_ * p[0] - s_ * p[1], s_ * p[0] + c_ * p[1]]) - p
            td.err_odom = np.zeros(2)
            td.t_align = -w.landmark_err[av.cell] - rot + rdr.normal(0, 0.3, 2)
            av.var_odom = 0.0
            av.sig_t = 1.0

    def start(d, t):
        av, td = agents[d], truth[d]
        dec = pol.decide(av, t)
        a = dec.action
        k = a["kind"]
        if k == "move":
            c0, c1 = av.cell, a["cell"]
            dist = float(np.hypot(*(W.CENTERS[c1] - W.CENTERS[c0])))
            v = min(a["v"], 0.5 * (W.vmax(w.rho[c0]) + W.vmax(w.rho[c1])))
            dur = dist / v
            if w.blocked[c1]:
                dur *= 0.8  # approach, discover, return
                dist *= 0.8
        elif k == "scan":
            v = min(a["v"], W.vmax(w.rho[a["cell"]]))
            dur = W.scan_time(w.rho[a["cell"]], v)
            dist = dur * v
        elif k == "review":
            v, dur, dist = P.REVIEW_V, P.REVIEW_S, P.REVIEW_S * P.REVIEW_V
        elif k == "hold":
            v, dur, dist = 0.0, a.get("s", 2.0), 0.0
        else:  # land
            v, dur, dist = 0.0, 1.0, 0.0
        dur = max(1, int(math.ceil(dur)))
        td.action, td.t_done = a, t + dur
        td.energy += W.power_w(v, hover=(k in ("hold", "review"))) * dur / 3600.0
        av.energy = pub.battery_wh - td.energy
        # odometry drift (truth) and its believed variance (the estimator reports its own covariance; the
        # systematic heading-bias part is not in it)
        if dist > 0:
            rate = w.drift_rate[av.cell]
            td.err_odom += rdr.normal(0, rate * math.sqrt(dist * 20.0), 2) + 0.5 * rate * dist * td.bias
            av.var_odom += (rate ** 2) * dist * 20.0
            td.dist += dist
            td.nearmiss += W.near_miss_rate(w.rho[av.cell], v) * dist
        if logf:
            post = av.k.posterior()
            top = np.argsort(-post)[:3]
            rec = {"t": t, "t_ns": int(t) * 1_000_000_000, "drone": d, "policy": policy,
                   "pose": [round(float(av.pose[0]), 1), round(float(av.pose[1]), 1)],
                   "pose_sigma_m": round(math.sqrt(av.pose_var()), 2), "cell": int(av.cell),
                   "energy_wh": round(av.energy, 2), "link": {"heard_base_s_ago": round(t - av.heard_base_t, 1),
                                                              "heard_partner_s_ago": round(t - av.heard_partner_t, 1),
                                                              "forecast_p_link": round(float(av.k.p_link[av.cell]), 2)},
                   "belief": {"max_cell_post": round(float(post.max()), 4),
                              "top_cells": [[int(c), round(float(post[c]), 4)] for c in top],
                              "searched_cells": int(av.k.scanned.sum()),
                              "candidates": [c.summary(E) for c in av.k.cands if c.pi() > 0.05],
                              "excursion": _jsonable(av.plan.get("excursion"))},
                   "candidate_actions": [_jsonable(s) for s in dec.scored[:8]],
                   "chosen": _jsonable(a), "goal": _jsonable(dec.goal), "reason": dec.reason,
                   "truth_debug": {"pose_err_m": round(float(np.hypot(*td.err(av.pose, w.base_xy))), 2)}}
            logf.write(json.dumps(rec) + "\n")

    def flush_requests(d, t):
        av = agents[d]
        for kind, x in av.outbox_requests:
            if kind == "DET":
                cand = av.cand_by_id(x)
                if cand is None:
                    continue
                for dst in (0, av.partner):
                    net.send(t, d, dst, "DET", cand.payload(d))
                if pol.comm == "full":
                    net.send(t, d, 0, "IMG", {"cid": x})
            elif kind == "VERIFY":
                cand = av.cand_by_id(x)
                if cand is not None:
                    net.send(t, d, av.partner, "VERIFY", {"cand": cand.payload(d)})
            elif kind == "MULE":
                cand = av.cand_by_id(x)
                if cand is not None:
                    net.send(t, d, av.partner, "VERIFY", {"cand": cand.payload(d), "mule": True, "cell": int(av.cell)})
            elif kind == "LEASE" and av.lease:
                for dst in (0, av.partner):
                    net.send(t, d, dst, "LEASE", {"cells": av.lease[0], "expiry": av.lease[1], "id": d})
        av.outbox_requests = []

    for t in range(H):
        for d in (1, 2):
            td = truth[d]
            if td.landed:
                continue
            if td.action is not None and t >= td.t_done:
                complete(d, t)
                td.action = None
            if td.action is None and not td.landed:
                start(d, t)
            flush_requests(d, t)
        # periodic messages (same budget for every policy; FULL adds bulk MAP)
        for d in (1, 2):
            av = agents[d]
            hb = {"id": d, "pose": av.pose.tolist(), "cell": int(av.cell), "t": t, "energy": round(av.energy, 1),
                  "goal": (av.goal or {}).get("cell"), "heard_partner_t": av.heard_partner_t,
                  "lease": av.lease, "lease_exp": t + P.LEASE_S}
            for dst in (0, av.partner):
                net.send(t, d, dst, "HB", hb)
            if t - av.last_cov_t >= 5 and (av.new_cov or av.new_blk):
                net.send(t, d, 0, "COV", {"scans": list(av.new_cov), "blk": list(av.new_blk), "id": d})
                if pol.name != "racer_lite":
                    net.send(t, d, av.partner, "COV", {"scans": list(av.new_cov), "blk": list(av.new_blk), "id": d})
                else:
                    if t - av.heard_partner_t <= 2:
                        net.send(t, d, av.partner, "COV", {"scans": list(av.new_cov), "blk": list(av.new_blk), "id": d})
                av.new_cov, av.new_blk = [], []
                av.last_cov_t = t
            if pol.comm == "full" and t - av.last_map_t >= 10:
                pl = {"S": av.k.S.tolist(), "scanned": av.k.scanned.tolist(), "id": d, "goal": (av.goal or {}).get("cell")}
                net.send(t, d, av.partner, "MAP", pl)
                net.send(t, d, 0, "MAP", pl)
                av.last_map_t = t
        pos = {d: true_pos(d) for d in (1, 2)}
        active = {d: not truth[d].dead for d in (1, 2)}
        for node, frm, msg in net.tick(t, pos, active):
            if node == 0:
                _base_receive(base, base_hb, msg, frm, t, net)
            else:
                _agent_receive(agents[node], msg, frm, t, M)
        for d in (1, 2):
            av = agents[d]
            av.custody_pending = sum(1 for m in net.outbox[d] if m.kind == "DET" and m.origin != d and m.dst == 0)
            if net.heard[d][0] > av.heard_base_t:
                av.heard_base_t = net.heard[d][0]
                av.k.link_seen[av.cell] = (t, 0)
            elif t - av.heard_base_t > 1:
                th, nf = av.k.link_seen.get(av.cell, (-1e9, 0))
                av.k.link_seen[av.cell] = (th, nf + 1)
            if net.heard[d][d ^ 3] > av.heard_partner_t:
                av.heard_partner_t = net.heard[d][d ^ 3]
        # coordinator: the base re-assigns search / verify / report as information arrives
        if pol.name == "coordinator" and t - base_assign_t >= 5:
            base_assign_t = t
            _coordinate(base, base_hb, net, t)
        # local confirmation and base delivery (truth scoring)
        for d in (1, 2):
            for cand in agents[d].k.cands:
                if cand.pi() >= TAU and M["t_local_conf"] is None:
                    m, _ = cand.fused()
                    if np.hypot(*(m - w.target_xy)) <= 2 * E + 5:
                        M["t_local_conf"] = t
                        M["local_conf_label"] = cand.summary(E)["label"]
        for cand in base.cands:
            if cand.pi() >= TAU:
                m, _ = cand.fused()
                err = float(np.hypot(*(m - w.target_xy)))
                if M["t_base_conf_any"] is None:
                    M["t_base_conf_any"] = t
                if err <= E and M["t_delivery"] is None:
                    M["t_delivery"] = t
                    M["base_conf_err_m"] = err
                    M["label_at_delivery"] = cand.summary(E)["label"]
                elif err > 2 * E + 5 and not getattr(cand, "_fc", False):
                    cand._fc = True
                    M["false_confirm"] += 1
                elif err > E and M["base_conf_err_m"] is None:
                    M["base_conf_err_m"] = err
        if t % 5 == 0:
            dis_acc += float(np.sum(agents[1].k.scanned ^ agents[2].k.scanned)) * W.CELL ** 2
            dis_n += 1
        if all(truth[d].landed for d in (1, 2)):
            break
        if M["t_delivery"] is not None and t > M["t_delivery"] + 5 and not logf:
            break
    if logf:
        logf.close()
    M["disagree_m2_mean"] = dis_acc / max(dis_n, 1)
    M["energy_wh"] = sum(truth[d].energy for d in (1, 2))
    M["near_miss"] = sum(truth[d].nearmiss for d in (1, 2))
    M["success"] = int(M["t_delivery"] is not None and M["t_delivery"] <= sc.deadline_s)
    M["failure_cause"] = _cause(M, sc)
    M["bytes"] = net.stats["sent_bytes"]
    M["target_dead_up"] = bool(w.margin_up[w.target_cell] < 0)
    if return_world:
        return M, w
    return M


def _cause(M, sc):
    if M["success"]:
        return "success"
    if M["t_delivery"] is not None:
        return "late_delivery"
    if M["t_first_det"] is None:
        return "not_detected" if M["false_confirm"] == 0 else "false_confirm"
    if M["t_local_conf"] is None and M["t_base_conf_any"] is None:
        return "detected_not_confirmed"
    if M["base_conf_err_m"] is not None:
        return "location_error"
    if M["false_confirm"]:
        return "false_confirm"
    return "confirmed_not_delivered"


def _jsonable(x):
    if x is None:
        return None
    if isinstance(x, dict):
        return {str(k): _jsonable(v) for k, v in x.items()}
    if isinstance(x, (list, tuple, set)):
        return [_jsonable(v) for v in x]
    if isinstance(x, (np.integer,)):
        return int(x)
    if isinstance(x, (np.floating, float)):
        return round(float(x), 4)
    if isinstance(x, np.ndarray):
        return _jsonable(x.tolist())
    return x


def _apply_cov(k, pl):
    for c, o, q in pl["scans"]:
        if q > 0:
            k.add_null_scan(c, o, q)
        else:
            k.scanned[c] = True
    for c in pl["blk"]:
        k.mark_blocked(c)


def _agent_receive(av, msg, frm, t, M):
    pl = msg.payload
    kind = msg.kind
    if kind == "HB":
        if pl["id"] == av.id:
            return
        if pl["t"] >= av.partner_hb_t:
            av.partner_hb = pl
            av.partner_hb_t = pl["t"]
            av.partner_heard_me_t = pl["heard_partner_t"]
            av.partner_lease_exp = pl["lease_exp"]
            if pl.get("lease"):
                av.partner_lease = (set(pl["lease"][0]), pl["lease"][1])
    elif kind == "COV":
        if pl["id"] != av.id:
            _apply_cov(av.k, pl)
    elif kind == "MAP":
        if pl["id"] != av.id and msg.t_created >= av.plan.get("map_t", -1):
            av.plan["map_t"] = msg.t_created
            S = np.array(pl["S"])
            av.k.S = np.minimum(av.k.S, S) if False else np.where(S < av.k.S, S, av.k.S)
            av.k.scanned |= np.array(pl["scanned"])
    elif kind == "DET":
        if pl["drone"] == av.id:
            return
        merge_payload(av.k.cands, pl)
    elif kind == "VERIFY":
        c = merge_payload(av.k.cands, pl["cand"])
        if pl.get("mule"):
            av.plan["mule_req"] = (pl["cell"], t)
        else:
            av.verify_req.append(c.id)
    elif kind == "LEASE":
        if pl["id"] != av.id:
            av.partner_lease = (set(pl["cells"]), pl["expiry"])
    elif kind == "ASSIGN":
        if pl["drone"] == av.id:
            av.assign = (pl, t)
    elif kind == "ACKDET":
        cand = av.cand_by_id(pl["cid"])
        if cand is not None and av.id in cand.ev and pl["lo"] >= cand.ev[av.id]["lo"] - 1e-6 \
                and pl["n_nulls"] >= sum(cand.ev[av.id]["nulls"].values()):
            cand.delivered_t = t


def _base_receive(base, base_hb, msg, frm, t, net):
    pl = msg.payload
    if msg.kind == "HB":
        base_hb[pl["id"]] = pl
        if not pl.get("relayed"):
            q = dict(pl, relayed=True)
            net.send(t, 0, 3 - pl["id"], "HB", q)
    elif msg.kind == "COV":
        _apply_cov(base, pl)
    elif msg.kind == "DET":
        merge_payload(base.cands, pl)
        if msg.dst == 0 and pl["ev"] is not None:
            # end-to-end receipt so the reporting drone knows this version of its evidence reached the base
            net.send(t, 0, pl["drone"], "ACKDET", {"cid": pl["cid"], "lo": pl["ev"]["lo"],
                                                    "n_nulls": sum(pl["ev"]["nulls"].values())})
    elif msg.kind == "MAP":
        S = np.array(pl["S"])
        base.S = np.where(S < base.S, S, base.S)
        base.scanned |= np.array(pl["scanned"])


def _coordinate(base, base_hb, net, t):
    post = base.posterior()
    hb = {d: base_hb[d] for d in (1, 2) if base_hb[d] is not None and t - base_hb[d]["t"] < 20}
    if not hb:
        return
    taken = set()
    open_c = [c for c in base.cands if 0.15 <= c.pi() < TAU]
    verifier = None
    if open_c:
        cand = max(open_c, key=lambda c: c.pi())
        ver = min(hb, key=lambda d: base.D[hb[d]["cell"], cand.cell] + (30 if d in cand.ev else 0))
        verifier = ver
        net.send(t, 0, ver, "ASSIGN", {"drone": ver, "role": "verify", "est": cand.est.tolist(), "cid": cand.id})
    for d in sorted(hb):
        if d == verifier:
            continue
        D = base.D[hb[d]["cell"]]
        sc = post / (D + 10.0)
        sc[base.blocked | base.scanned] = -1
        for c in taken:
            sc[c] = -1
            sc[W.NEIGH[c]] *= 0.3
        c = int(np.argmax(sc))
        taken.add(c)
        net.send(t, 0, d, "ASSIGN", {"drone": d, "role": "search", "cell": c})


def _e1_setup(w, agents, truth, ro, M, obs, sc):
    """E1 forced glimpse: drone 1 has just entered the person's (dead) cell from the W and glimpsed the person."""
    tc = w.target_cell
    av = agents[1]
    av.cell = tc
    av.pose = W.CENTERS[tc].copy()
    truth[1].cell = tc
    truth[1].energy = 0.35 * w.public.battery_wh
    av.energy = w.public.battery_wh - truth[1].energy
    av.var_odom = 1.5 ** 2
    truth[1].err_odom = ro.normal(0, 1.5, 2)
    av.heard_base_t = -10.0
    e = truth[1].err(av.pose, w.base_xy)
    est = w.target_xy - e + ro.normal(0, obs.loc_sigma(6.0, 4.0), 2)
    cand = Candidate(11, est, W.xy_cell(est), av.k.lo_prior_for(tc, 4.0), 0.0)
    av.k.next_cid = 2
    ev = cand.drone_ev(1)
    ev["lo"] = obs.log_lr(sc.e1_conf)
    ev["views"] = [0]
    ev["n"] = 1
    ev["sum_est"] = est.copy()
    ev["s_ind"] = obs.loc_sigma(6.0, 4.0) ** 2
    ev["s_com"] = av.pose_var()
    av.k.cands.append(cand)
    av.events.append(("detection", cand.id))
    av.outbox_requests.append(("DET", cand.id))
    M["t_first_det"] = 0
    M["glimpse_in_outage"] = True
