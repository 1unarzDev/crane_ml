# SPDX-License-Identifier: GPL-3.0-or-later
"""Mission policies. Every policy gets only an AgentView (knowledge) and returns (goal-step action, scored
alternatives, reason). Battery, sensing and comms budgets are identical across policies: same airframe energy,
same speeds, same detector, same radio and the same HB 1 Hz + COV 0.2 Hz + event DET message set (the FULL-sharing
variant adds bulk MAP/IMG, which compete for the same bandwidth).

  static        fixed two-sector boustrophedon, fixed speed, one re-view per alert, react-after-loss delivery
  frontier      nearest team-unscanned cell, one re-view per alert, react-after-loss
  pod           target-aware probability-of-detection rate planner (cell x orientation x speed), greedy confirm,
                react-after-loss  [= outage strategy (i) REACT]
  coordinator   base re-assigns search / verify / report as reports arrive; drones fall back to pod when stale
  lease         decentralized partition-robust: deterministic default plan, time-limited sector leases, health
                beacon with "last heard you" field (asymmetric-outage aware), takeover after lease expiry
  rendezvous    ADAPTED from da Silva et al. 2023-style intermittent rendezvous: fixed schedule of meetings at
                forecast-connected points on the sector boundary, store-and-forward data muling. Our adaptation,
                not a reproduction of their algorithm.
  pod_maintain  outage strategy (ii): pod restricted to forecast-connected cells (+1-cell tether)
  pod_deadzone  outage strategy (iii): lease + deliberate dead-zone excursions with onboard assignment, planned
                reconnect point, time-limited lease, revision on obstacle / detection / longer-than-expected
                outage; evidence decisions by value of delivery by the deadline (VOI)
  pod_full      pod with FULL sharing (bulk MAP + IMG, partner-goal avoidance, no leases)  [E3 comparator]
  oracle        ORACLE (labelled): pod_deadzone logic with perfect knowledge of person cell and view visibility,
                true blockages and true RF field. Same sensing, energy, comms and localization.
"""
import math

import numpy as np

from . import world as W
from .agent import TAU, logit, sigm, phi

SPEEDS = (2.0, 4.0, 6.0)
REVIEW_S = 14.0
REVIEW_V = 1.5
LEASE_S = 90.0
T1_S = 20.0
RDV_PERIOD = 150.0
DIRS = [(-1, 0), (0, -1), (1, 0), (0, 1)]  # view classes: approach from W, S, E, N
EVIDENCE_OPTIONS = ("review", "continue", "transmit", "store_rdv", "call_partner")


def view_cell(c, v, blocked):
    i, j = divmod(c, W.NY)
    di, dj = DIRS[v]
    ii, jj = i + di, j + dj
    if 0 <= ii < W.NX and 0 <= jj < W.NY and not blocked[ii * W.NY + jj]:
        return ii * W.NY + jj
    return c


def default_partition(pub):
    """Deterministic two-way split by bearing from the base, balanced by prior mass (both drones recompute it)."""
    ang = np.arctan2(W.CENTERS[:, 1] - pub.base_xy[1], W.CENTERS[:, 0] - pub.base_xy[0])
    order = np.argsort(ang, kind="stable")
    cum = np.cumsum(pub.p0[order])
    own = np.zeros(W.NC, int)
    own[order[cum > 0.5 * cum[-1]]] = 1
    return own + 1  # owner drone id per cell


def boustrophedon(pub, owner, d):
    cells = [c for c in range(W.NC) if owner[c] == d and not pub.blocked_map[c]]
    def key(c):
        i, j = divmod(c, W.NY)
        return (i, j if i % 2 == 0 else -j)
    return sorted(cells, key=key)


class Decision:
    __slots__ = ("action", "scored", "reason", "goal")

    def __init__(self, action, scored, reason, goal=None):
        self.action, self.scored, self.reason, self.goal = action, scored, reason, goal


class Policy:
    name = "base"
    uses_truth = False
    comm = "selective"
    leases = False
    review_mode = "greedy"   # once | greedy | voi
    outage = "react"
    speed_fixed = None

    def __init__(self, forced_option=None):
        self.forced_option = forced_option
        self.owner = None
        self.order = None

    def setup(self, av):
        self.owner = default_partition(av.pub)

    # ------------------------------------------------------------ helpers
    def move_or_do(self, av, goal, t, scored, reason):
        c = goal["cell"]
        if av.cell == c:
            k = goal["kind"]
            if k == "search":
                act = {"kind": "scan", "cell": c, "o": goal["o"], "v": goal["v"]}
            elif k == "review":
                act = {"kind": "review", "cid": goal["cid"], "view": goal["view"], "cell": c}
            elif k in ("transmit", "rdv", "hold"):
                act = {"kind": "hold", "s": 2.0, "cell": c}
            elif k == "rtb":
                act = {"kind": "land", "cell": c}
            else:
                act = {"kind": "hold", "s": 2.0, "cell": c}
        else:
            nh = W.next_hop(av.k.D, av.k.blocked, av.k.rho, av.cell, c)
            v = goal.get("v_transit", 6.0)
            act = {"kind": "move", "cell": nh, "v": v, "goal_cell": c}
        av.goal = goal
        return Decision(act, scored, reason, goal)

    def allowed_mask(self, av, t):
        return ~av.k.blocked

    def cell_weight(self, av, t):
        return np.ones(W.NC)

    def search_scores(self, av, t, k_top=6):
        post = av.k.posterior()
        mask = self.allowed_mask(av, t)
        wgt = self.cell_weight(av, t)
        D = av.k.D[av.cell]
        speeds = (self.speed_fixed,) if self.speed_fixed else SPEEDS
        best = []
        for v in speeds:
            for o in (0, 1):
                g = av.k.g
                gp = g * np.exp(av.k.S[:, o]) / (g * np.exp(av.k.S[:, o]) + 1 - g)
                q = np.array([av.obs_pd(c, v) for c in range(W.NC)]) if False else av.pdq[v]
                ts = av.scan_t[v]
                rate = post * gp * q / (D + ts + 1.0) * wgt
                rate = np.where(mask, rate, -1.0)
                for c in np.argsort(-rate)[:k_top]:
                    if rate[c] > 0:
                        best.append({"action": "SCAN", "cell": int(c), "o": o, "v": v, "score": float(rate[c]),
                                     "terms": {"post": round(float(post[c]), 4), "pd": round(float(gp[c] * q[c]), 3),
                                               "travel_s": round(float(D[c]), 1), "scan_s": round(float(ts[c]), 1)}})
        best.sort(key=lambda x: -x["score"])
        return best[:k_top], post

    def search_goal(self, av, t):
        scored, post = self.search_scores(av, t)
        if not scored:
            return None, scored
        b = scored[0]
        g = av.goal
        if g and g.get("kind") == "search" and not av.k.blocked[g["cell"]]:
            for s in scored:
                if s["cell"] == g["cell"] and s["score"] >= 0.8 * b["score"]:
                    b = s
                    break
        return {"kind": "search", "cell": b["cell"], "o": b["o"], "v": b["v"]}, scored

    def best_view(self, av, cand):
        tried = cand.views() | set(cand.ev.get(av.id, {}).get("nulls", {}).keys())
        if getattr(av, "oracle_views", None) is not None:
            good = [v for v in range(4) if av.oracle_views[v]]
            pref = [v for v in good if v not in tried] or good or list(range(4))
            return min(pref, key=lambda v: av.k.D[av.cell, view_cell(cand.cell, v, av.k.blocked)])
        opts = [v for v in range(4) if v not in tried] or list(range(4))
        return min(opts, key=lambda v: av.k.D[av.cell, view_cell(cand.cell, v, av.k.blocked)])

    def review_goal(self, av, cand):
        v = self.best_view(av, cand)
        return {"kind": "review", "cid": cand.id, "view": v, "cell": view_cell(cand.cell, v, av.k.blocked),
                "v_transit": 4.0}

    def my_open_cands(self, av):
        return [c for c in av.k.cands if c.pi() >= 0.15 and c.delivered_t is None]

    # ------------------------------------------------------------ evidence value model (planner side, VOI)
    def p_time(self, t_ready, now, D):
        return phi((D - t_ready) / (0.25 * max(t_ready - now, 0.0) + 5.0))

    def evidence_options(self, av, t, cand):
        """Estimated P(confirmed, correctly located report at base by D) for each evidence option."""
        Dl = av.pub.deadline_s
        E = av.pub.err_tol_m
        pi = cand.pi()
        lr_hi = av.k.obs.log_lr(0.85)
        need = max(0, math.ceil((logit(TAU) - cand.lo()) / lr_hi))
        rho = av.k.rho[cand.cell]
        q_rev = av.k.g[cand.cell] * av.k.obs.expected_pd(rho, REVIEW_V, W.r_eff(rho), mode="review")
        if getattr(av, "oracle_views", None) is not None:
            q_rev = av.k.obs.expected_pd(rho, REVIEW_V, W.r_eff(rho), mode="review")
        v = self.best_view(av, cand)
        vc = view_cell(cand.cell, v, av.k.blocked)
        t_rev1 = av.k.D[av.cell, vc] + REVIEW_S
        p_conf_self = q_rev ** max(need, 1) if need > 0 else 1.0
        p_conf_self = min(1.0, p_conf_self * (1.6 if need >= 1 else 1.0))
        t_rev = t_rev1 + max(need - 1, 0) * (REVIEW_S + 8.0)
        p_loc_self = cand.p_loc_ok(E)
        # partner path: a second drone's look also shrinks the common-mode alignment error
        _, var = cand.fused()
        var_two = 1.0 / (1.0 / var + 1.0 / (var + 4.0)) if cand.ev else var
        p_loc_two = 1.0 - math.exp(-E * E / (2.0 * var_two))
        pc = av.partner_cell()
        t_part_view = av.k.D[pc, view_cell(cand.cell, (v + 2) % 4, av.k.blocked)] + REVIEW_S
        heard = av.heard_base_t
        rate = max(av.last_search_rate, 1e-5)
        def v_search(t_rem):
            return (1.0 - math.exp(-rate * max(t_rem, 0.0))) * 0.5
        out = {}
        # REVIEW now (slow down / re-view), then deliver
        Td, _ = av.k.t_deliver(vc, t + t_rev, heard if t_rev < 3 else -1e9)
        s_true = p_conf_self * p_loc_self * self.p_time(t + t_rev + Td, t, Dl)
        out["review"] = (pi * s_true + (1 - pi) * v_search(Dl - t - t_rev),
                         {"pi": round(pi, 3), "need": need, "q_rev": round(q_rev, 2), "t_review_s": round(t_rev, 1),
                          "t_deliver_s": round(Td, 1), "p_loc": round(p_loc_self, 2)})
        # CONTINUE search; deliver the glimpse whenever the link returns (planned reconnect if any); verify later
        exc = av.plan.get("excursion")
        t_back = exc["recon_t"] if exc else t + av.k.t_deliver(av.cell, t, heard)[0] + 30.0
        Tp = t_back + 5.0 + t_part_view + av.k.t_deliver(cand.cell, t, -1e9)[0]
        s_true = p_conf_self * p_loc_two * self.p_time(Tp, t, Dl) * 0.85
        out["continue"] = (pi * s_true + (1 - pi) * v_search(Dl - t), {"t_report_s": round(t_back - t, 1),
                                                                      "t_verified_s": round(Tp - t, 1)})
        # TRANSMIT now: carry the unconfirmed glimpse to a link point so the partner can verify; then return
        Td0, txc = av.k.t_deliver(av.cell, t, heard)
        t_ready = t + Td0
        Tpart = t_ready + 3.0 + t_part_view + av.k.t_deliver(cand.cell, t, -1e9)[0]
        Tself = t_ready + av.k.D[txc, vc] + t_rev - t_rev1 + REVIEW_S + av.k.t_deliver(vc, t, -1e9)[0]
        s_true = p_conf_self * max(p_loc_two * self.p_time(Tpart, t, Dl), p_loc_self * self.p_time(Tself, t, Dl))
        out["transmit"] = (pi * s_true + (1 - pi) * v_search(Dl - t - 2 * Td0),
                           {"t_tx_s": round(Td0, 1), "tx_cell": int(txc), "t_partner_verify_s": round(Tpart - t, 1)})
        # STORE for the next rendezvous (scheduled meeting); verify after it
        t_rdv = (math.floor(t / RDV_PERIOD) + 1) * RDV_PERIOD
        Tr = t_rdv + t_part_view + av.k.t_deliver(cand.cell, t, -1e9)[0]
        s_true = p_conf_self * p_loc_two * self.p_time(Tr, t, Dl) * 0.9
        out["store_rdv"] = (pi * s_true + (1 - pi) * v_search(Dl - t), {"t_rdv_s": round(t_rdv - t, 1)})
        # CALL the partner directly (air-to-air forecast) and continue searching
        pa = W.CENTERS[pc]
        dd = max(float(np.hypot(*(pa - av.pose))), 5.0)
        m_a2a = av.pub.m0_a2a - 20 * math.log10(dd / 10.0) - av.pub.alpha_db_per_m * 0.5 * dd * float(av.k.rho.mean())
        p_a2a = phi(m_a2a / av.pub.forecast_sigma)
        if t - av.heard_partner_t <= 3:
            p_a2a = max(p_a2a, 0.9)
        Tc = t + 3.0 + t_part_view + av.k.t_deliver(cand.cell, t, -1e9)[0]
        s_true = p_conf_self * p_loc_two * (p_a2a * self.p_time(Tc, t, Dl) + (1 - p_a2a) * 0.5 * self.p_time(Tp, t, Dl))
        out["call_partner"] = (pi * s_true + (1 - pi) * v_search(Dl - t), {"p_a2a": round(p_a2a, 2),
                                                                          "t_partner_verify_s": round(Tc - t, 1)})
        return out

    def start_option(self, av, t, cand, opt, scored, reason):
        """Commit to an evidence option; returns a Decision."""
        av.plan["evidence"] = {"cid": cand.id, "opt": opt, "t": t}
        if opt == "review":
            return self.move_or_do(av, self.review_goal(av, cand), t, scored, reason)
        if opt == "transmit":
            _, txc = av.k.t_deliver(av.cell, t, av.heard_base_t)
            return self.move_or_do(av, {"kind": "transmit", "cell": txc, "cid": cand.id, "t0": t}, t, scored, reason)
        if opt == "call_partner":
            av.outbox_requests.append(("VERIFY", cand.id))
        if opt == "store_rdv":
            av.plan["rdv_hold"] = True
        g, s2 = self.search_goal(av, t)
        if g is None:
            g = {"kind": "hold", "cell": av.cell}
        return self.move_or_do(av, g, t, scored + s2, reason)

    # ------------------------------------------------------------ main decision
    def battery_check(self, av, t):
        home = av.pub.start_cells[av.id - 1]
        need = av.k.D[av.cell, home] * W.power_w(6.0) / 3600.0 * 1.3 + 3.0
        return av.energy < need

    def decide(self, av, t):
        if self.owner is None:
            self.setup(av)
        if self.battery_check(av, t) or (av.goal and av.goal.get("kind") == "rtb"):
            home = av.pub.start_cells[av.id - 1]
            return self.move_or_do(av, {"kind": "rtb", "cell": home}, t,
                                   [{"action": "RTB", "score": 1.0, "terms": {"energy_wh": round(av.energy, 1)}}],
                                   "battery reserve: energy-to-home x1.3 + 3 Wh")
        g = av.goal
        # active commitments
        if g and g["kind"] == "review":
            cand = av.cand_by_id(g["cid"])
            if cand is not None and cand.pi() < TAU and cand.reviews < 4 and cand.pi() > 0.05:
                return self.move_or_do(av, g, t, [{"action": "REVIEW", "cid": g["cid"], "score": 1.0}],
                                       "continue committed re-view")
        if g and g["kind"] == "transmit":
            cand = av.cand_by_id(g.get("cid"))
            done = cand is None or cand.delivered_t is not None
            if not done and t - g["t0"] < 90:
                if av.cell == g["cell"] and t - av.heard_base_t > 20 and t - g.get("t_arr", t) > 20:
                    av.k.link_seen[av.cell] = (-1e9, 99)
                    _, txc = av.k.t_deliver(av.cell, t, -1e9)
                    g = dict(g, cell=txc)
                    return self.move_or_do(av, g, t, [{"action": "TRANSMIT", "cell": txc, "score": 1.0}],
                                           "outage longer than expected at transmit point: next best link cell")
                if av.cell == g["cell"] and "t_arr" not in g:
                    g["t_arr"] = t
                return self.move_or_do(av, g, t, [{"action": "TRANSMIT", "cell": g["cell"], "score": 1.0}],
                                       "holding at transmit point until the base acks the report")
            av.goal = None
            ev = av.plan.get("evidence")
            if cand is not None and ev and ev["opt"] == "transmit" and cand.pi() < TAU:
                ev["opt"] = "review"
                return self.move_or_do(av, self.review_goal(av, cand), t,
                                       [{"action": "REVIEW", "cid": cand.id, "score": 1.0}],
                                       "report delivered; return to re-view")
        if self.review_mode != "voi":
            d = self.special(av, t)
            if d is not None:
                return d
        # evidence handling (the VOI policy decides on evidence before its excursion / lease logic)
        for cand in self.my_open_cands(av):
            if cand.id in av.plan.get("handled", set()) and cand.pi() < TAU and self.review_mode != "voi":
                continue
            d = self.handle_evidence(av, t, cand)
            if d is not None:
                return d
        if self.review_mode == "voi":
            d = self.special(av, t)
            if d is not None:
                return d
        # verify requests from partner / base
        for cid in list(av.verify_req):
            cand = av.cand_by_id(cid)
            av.verify_req.remove(cid)
            if cand is not None and cand.pi() > 0.1 and av.id not in cand.ev and \
                    (cand.pi() < TAU or cand.p_loc_ok(av.pub.err_tol_m) < 0.7):
                return self.move_or_do(av, self.review_goal(av, cand), t,
                                       [{"action": "VERIFY", "cid": cid, "score": 1.0}], "partner/base asked to verify")
        g, scored = self.search_goal(av, t)
        if g is None:
            home = av.pub.start_cells[av.id - 1]
            return self.move_or_do(av, {"kind": "rtb", "cell": home}, t, [], "nothing left to search")
        av.last_search_rate = scored[0]["score"] if scored else 1e-5
        return self.move_or_do(av, g, t, scored, "search: max P(detect)/time")

    def special(self, av, t):
        return None

    def maybe_call_for_location(self, av, t, cand):
        """Confident detection with a poor location (alignment error is common to all of my looks): ask the partner
        for an independent look, once."""
        asked = av.plan.setdefault("loc_asked", set())
        if cand.id not in asked and cand.p_loc_ok(av.pub.err_tol_m) < 0.7 and av.partner not in cand.ev:
            asked.add(cand.id)
            av.outbox_requests.append(("VERIFY", cand.id))

    def handle_evidence(self, av, t, cand):
        pi = cand.pi()
        handled = av.plan.setdefault("handled", set())
        if pi >= TAU and self.review_mode != "once":
            self.maybe_call_for_location(av, t, cand)
        if pi >= TAU:
            if cand.delivered_t is None:
                tc = av.plan.setdefault("t_conf", {}).setdefault(cand.id, t)
                if t - tc > 20 and t - av.heard_base_t > 5:  # react after loss
                    _, txc = av.k.t_deliver(av.cell, t, -1e9)
                    return self.move_or_do(av, {"kind": "transmit", "cell": txc, "cid": cand.id, "t0": t}, t,
                                           [{"action": "TRANSMIT", "cell": txc, "score": 1.0}],
                                           "react: confirmed report undelivered 20 s; go to link point")
            return None
        if self.review_mode == "once":
            if cand.reviews == 0 and av.id in cand.ev:
                handled.add(cand.id)
                return self.move_or_do(av, self.review_goal(av, cand), t,
                                       [{"action": "REVIEW", "cid": cand.id, "score": 1.0}], "one re-view per alert")
            return None
        if self.review_mode == "greedy":
            if cand.reviews < 3 and av.id in cand.ev:
                return self.move_or_do(av, self.review_goal(av, cand), t,
                                       [{"action": "REVIEW", "cid": cand.id, "score": 1.0}], "greedy confirm")
            return None
        return None


class Static(Policy):
    name = "static"
    review_mode = "once"
    speed_fixed = 4.0

    def setup(self, av):
        super().setup(av)
        self.order = {d: boustrophedon(av.pub, self.owner, d) for d in (1, 2)}

    def search_goal(self, av, t):
        for c in self.order[av.id]:
            if not av.k.scanned[c] and not av.k.blocked[c]:
                return ({"kind": "search", "cell": c, "o": 0, "v": 4.0},
                        [{"action": "SCAN", "cell": c, "score": 1.0, "terms": {"rule": "next lawnmower cell"}}])
        return None, []


class Frontier(Policy):
    name = "frontier"
    review_mode = "once"
    speed_fixed = 4.0

    def search_goal(self, av, t):
        D = av.k.D[av.cell].copy()
        D[av.k.scanned | av.k.blocked] = np.inf
        pc = av.partner_goal_cell()
        if pc is not None:
            D[pc] = np.inf
        if not np.isfinite(D).any():
            return None, []
        order = np.argsort(D)[:4]
        sc = [{"action": "SCAN", "cell": int(c), "score": float(-D[c]), "terms": {"travel_s": round(float(D[c]), 1)}}
              for c in order if np.isfinite(D[c])]
        return {"kind": "search", "cell": int(order[0]), "o": 0, "v": 4.0}, sc


class POD(Policy):
    name = "pod"

    def cell_weight(self, av, t):
        w = np.ones(W.NC)
        pc = av.partner_goal_cell()
        if pc is not None:
            w[pc] = 0.05
            for n in W.NEIGH[pc]:
                w[n] *= 0.5
        return w


class PODFull(POD):
    name = "pod_full"
    comm = "full"


class Coordinator(POD):
    name = "coordinator"
    review_mode = "once"

    def special(self, av, t):
        a = av.assign
        if a is None or t - a[1] > 25:
            return None
        p = a[0]
        if p["role"] == "verify":
            cand = av.cand_by_id_or_est(p)
            if cand is not None and cand.pi() < TAU and cand.reviews < 3:
                return self.move_or_do(av, self.review_goal(av, cand), t,
                                       [{"action": "VERIFY", "score": 1.0, "terms": {"assigned_t": a[1]}}],
                                       "base assigned VERIFY")
        if p["role"] == "report":
            return None
        if p["role"] == "search" and not av.k.scanned[p["cell"]] and not av.k.blocked[p["cell"]]:
            return self.move_or_do(av, {"kind": "search", "cell": p["cell"], "o": 0, "v": 4.0}, t,
                                   [{"action": "SCAN", "cell": p["cell"], "score": 1.0,
                                     "terms": {"assigned_t": a[1]}}], "base assigned SEARCH cell")
        return None


class Lease(POD):
    name = "lease"
    leases = True

    def partner_alive(self, av, t):
        return t - av.partner_hb_t <= T1_S

    def lease_valid(self, av, t):
        """Partner's sector lease is honoured until its announced expiry, renewed by heard beacons."""
        exp = av.partner_lease_exp
        return t <= exp

    def allowed_mask(self, av, t):
        m = ~av.k.blocked
        if self.lease_valid(av, t):
            m &= (self.owner == av.id)
            pl = av.partner_lease
            if pl is not None and pl[1] >= t:
                m[list(pl[0])] = False
            if not (m & ~av.k.scanned).any():  # own sector exhausted: help in partner sector
                m = ~av.k.blocked
        return m

    def cell_weight(self, av, t):
        w = POD.cell_weight(self, av, t)
        # asymmetric outage awareness: the partner says it has not heard me -> it will take over my sector after my
        # lease expires; work my sector from the end far from the partner to avoid duplicated search
        if av.partner_hb is not None and t - av.partner_heard_me_t > T1_S and self.partner_alive(av, t):
            pp = np.array(av.partner_hb["pose"])
            dist = np.hypot(W.CENTERS[:, 0] - pp[0], W.CENTERS[:, 1] - pp[1])
            w = w * (0.3 + dist / dist.max())
        return w


class Rendezvous(Lease):
    """B3 intermittent rendezvous, logic REIMPLEMENTED after da Silva et al. (IROS 2024; GPL-3.0 code not copied):
    precomputed periodic meetings at a forecast-connected cell on the sector boundary, store-and-forward muling."""
    name = "rendezvous"
    review_mode = "once"

    def setup(self, av):
        super().setup(av)
        b = [c for c in range(W.NC) if not av.pub.blocked_map[c] and
             any(self.owner[n] != self.owner[c] for n in W.NEIGH[c])]
        pl = av.k.p_link
        self.rdv_cell = int(max(b, key=lambda c: pl[c] - 0.002 * av.k.D[av.pub.start_cells[0], c]))

    def lease_valid(self, av, t):
        return True  # fixed responsibilities; information flows at rendezvous

    def special(self, av, t):
        tk = (math.floor(t / RDV_PERIOD) + 1) * RDV_PERIOD
        tprev = tk - RDV_PERIOD
        R = self.rdv_cell
        travel = av.k.D[av.cell, R]
        g = av.goal
        if g and g["kind"] == "rdv":
            met = t - av.heard_partner_t <= 2 and av.cell == R
            if (met and t >= g["t"]) or t > g["t"] + 20:
                av.goal = None
                av.plan["rdv_done"] = g["t"]
                return None
            return self.move_or_do(av, g, t, [{"action": "RENDEZVOUS", "cell": R, "score": 1.0,
                                               "terms": {"t_meet": g["t"]}}], "scheduled rendezvous")
        if av.plan.get("rdv_done") == tk:
            return None
        if t + travel + 6 >= tk and t < tk + 20 and av.plan.get("rdv_done") != tk:
            return self.move_or_do(av, {"kind": "rdv", "cell": R, "t": tk}, t,
                                   [{"action": "RENDEZVOUS", "cell": R, "score": 1.0,
                                     "terms": {"t_meet": tk, "travel_s": round(float(travel), 1)}}],
                                   "leave for scheduled rendezvous")
        return None

    def handle_evidence(self, av, t, cand):
        if cand.pi() >= TAU:
            return None  # store and forward at the next rendezvous (no deviation)
        return Policy.handle_evidence(self, av, t, cand)


class PODMaintain(POD):
    name = "pod_maintain"
    outage = "maintain"

    def allowed_mask(self, av, t):
        ok = av.k.p_link >= 0.5
        for c, (th, nf) in av.k.link_seen.items():
            if nf > 8:
                ok[c] = False
        teth = ok.copy()
        for c in np.nonzero(ok)[0]:
            teth[W.NEIGH[c]] = True
        return teth & ~av.k.blocked


class PODDeadzone(Lease):
    name = "pod_deadzone"
    review_mode = "voi"
    outage = "deadzone"
    EXC_BUDGET = 120.0

    def allowed_mask(self, av, t):
        m = Lease.allowed_mask(self, av, t)
        exc = av.plan.get("excursion")
        if exc:
            mm = np.zeros(W.NC, bool)
            mm[[c for c in exc["cells"] if not av.k.scanned[c]]] = True
            mm &= ~av.k.blocked
            if mm.any():
                return mm
        return m

    def plan_excursion(self, av, t, first):
        """Onboard assignment inside the dead zone: greedy PoD sequence within a time budget, a planned reconnect
        cell and time, announced as a time-limited lease before entry."""
        post = av.k.posterior()
        cells, tcur, cur = [first], t + av.k.D[av.cell, first] + av.scan_t[4.0][first], first
        dead = av.k.p_link < 0.5
        while True:
            cand = [c for c in range(W.NC) if dead[c] and not av.k.scanned[c] and not av.k.blocked[c]
                    and c not in cells and self.owner_ok(av, c, t)]
            if not cand:
                break
            c = max(cand, key=lambda c: post[c] / (av.k.D[cur, c] + av.scan_t[4.0][c] + 1.0))
            dt = av.k.D[cur, c] + av.scan_t[4.0][c]
            if tcur + dt - t > self.EXC_BUDGET:
                break
            cells.append(c)
            tcur += dt
            cur = c
        Td, rc = av.k.t_deliver(cur, tcur, -1e9)
        exc = {"cells": cells, "recon_cell": int(rc), "recon_t": float(tcur + Td), "t0": t,
               "expiry": float(tcur + Td + 30.0)}
        av.plan["excursion"] = exc
        av.lease = (list(cells), exc["expiry"])
        av.outbox_requests.append(("LEASE", None))
        return exc

    def owner_ok(self, av, c, t):
        if not self.lease_valid(av, t):
            return True
        pl = av.partner_lease
        if pl is not None and pl[1] >= t and c in pl[0]:
            return False
        return self.owner[c] == av.id

    def special(self, av, t):
        exc = av.plan.get("excursion")
        if exc:
            # revision triggers
            for ev in list(av.events):
                av.events.remove(ev)
                if ev[0] == "obstacle":
                    left = [c for c in exc["cells"] if not av.k.scanned[c] and not av.k.blocked[c]]
                    if left:
                        exc2 = self.plan_excursion(av, t, left[0])
                        exc2["revised"] = "new obstacle"
            remaining = [c for c in exc["cells"] if not av.k.scanned[c] and not av.k.blocked[c]]
            if not remaining or t > exc["expiry"]:
                if t - av.heard_base_t <= 3 or t > exc["recon_t"] + 60:
                    av.plan.pop("excursion", None)
                    av.lease = None
                    return None
                g = {"kind": "hold", "cell": exc["recon_cell"]}
                if av.cell == exc["recon_cell"] and t > exc["recon_t"] + 15:
                    av.k.link_seen[av.cell] = (-1e9, 99)
                    _, rc = av.k.t_deliver(av.cell, t, -1e9)
                    exc["recon_cell"] = int(rc)
                    return self.move_or_do(av, {"kind": "hold", "cell": rc}, t,
                                           [{"action": "RECONNECT", "cell": int(rc), "score": 1.0}],
                                           "outage longer than expected at reconnect point: move to next link cell")
                return self.move_or_do(av, g, t, [{"action": "RECONNECT", "cell": exc["recon_cell"], "score": 1.0,
                                                   "terms": {"recon_t": round(exc["recon_t"], 1)}}],
                                       "excursion done: go to planned reconnect point")
            return None
        # deliberate entry: best search cell is forecast-dead -> plan an excursion
        g, scored = self.search_goal(av, t)
        if g is not None and av.k.p_link[g["cell"]] < 0.5:
            exc = self.plan_excursion(av, t, g["cell"])
            av.last_search_rate = scored[0]["score"] if scored else 1e-5
            return self.move_or_do(av, g, t, scored + [{"action": "ENTER_DEADZONE", "score": 1.0, "terms": {
                "cells": exc["cells"], "recon_cell": exc["recon_cell"], "recon_t": round(exc["recon_t"], 1),
                "lease_expiry": round(exc["expiry"], 1)}}], "deliberate dead-zone entry with lease + reconnect plan")
        return None

    def handle_evidence(self, av, t, cand):
        pi = cand.pi()
        if pi >= TAU:
            self.maybe_call_for_location(av, t, cand)
            if cand.delivered_t is None:
                Td, txc = av.k.t_deliver(av.cell, t, av.heard_base_t)
                exc = av.plan.get("excursion")
                deliver_now = self.p_time(t + Td, t, av.pub.deadline_s)
                later_t = exc["recon_t"] if exc else t + Td + 30
                deliver_later = self.p_time(later_t, t, av.pub.deadline_s)
                gain_search = (1 - math.exp(-av.last_search_rate * max(later_t - t - Td, 0))) * 0.2
                scored = [{"action": "DELIVER_NOW", "cell": int(txc), "score": round(deliver_now, 3),
                           "terms": {"t_deliver_s": round(Td, 1)}},
                          {"action": "STORE_UNTIL_RECONNECT", "score": round(deliver_later + gain_search, 3),
                           "terms": {"t_recon_s": round(later_t - t, 1)}}]
                if t - av.heard_base_t > 3 and deliver_now >= deliver_later + gain_search:
                    return self.move_or_do(av, {"kind": "transmit", "cell": txc, "cid": cand.id, "t0": t}, t, scored,
                                           "confirmed: delivering now beats storing until reconnect")
            return None
        if cand.reviews >= 4 or av.id not in cand.ev and cand.id in av.plan.get("handled", set()):
            return None
        ev = av.plan.get("evidence")
        if ev and ev["cid"] == cand.id and ev["opt"] != "review" and t - ev["t"] < 120:
            return None
        opts = self.evidence_options(av, t, cand)
        scored = [{"action": k.upper(), "cid": cand.id, "score": round(v[0], 4), "terms": v[1]} for k, v in opts.items()]
        scored.sort(key=lambda x: -x["score"])
        if self.forced_option and cand.id not in av.plan.get("forced_done", set()):
            av.plan.setdefault("forced_done", set()).add(cand.id)
            opt = self.forced_option
            reason = "FORCED option %s (E1 ablation)" % opt
        else:
            opt = max(opts, key=lambda k: opts[k][0])
            reason = "evidence VOI argmax: %s" % opt
        av.plan.setdefault("handled", set()).add(cand.id)
        av.events = [e for e in av.events if e[0] != "detection"]
        return self.start_option(av, t, cand, opt, scored, reason)


# ---------------------------------------------------------------- literature baselines (LITREVIEW.md B1-B6)
class B1PodGreedy(Policy):
    """B1 POD-greedy Bayesian search, adapted from Lin & Goodrich 2009 (probability collected per flight time) with
    the shared belief of Bourgault et al. 2003; ADAPTATION: our 20 m cell grid, fixed 4 m/s, one re-view per alert,
    react-after-loss delivery. The floor."""
    name = "b1_podgreedy"
    speed_fixed = 4.0
    review_mode = "once"


class B2RacerLite(Policy):
    """B2 RACER-lite, REIMPLEMENTED from the paper description (Zhou et al. 2023; repo has no license, no code used):
    disjoint partitions, coverage tour (nearest unscanned own cell), coverage exchanged ONLY when in air-to-air
    range, remaining cells re-balanced by bearing on each contact. ADAPTATION: 2D cell grid, no CVRP solver."""
    name = "racer_lite"
    speed_fixed = 4.0
    review_mode = "once"

    def search_goal(self, av, t):
        if t - av.heard_partner_t <= 2 and av.plan.get("rebal_t", -1e9) < t - 20:
            av.plan["rebal_t"] = t
            rem = [c for c in range(W.NC) if not av.k.scanned[c] and not av.k.blocked[c]]
            if rem:
                ang = np.arctan2(W.CENTERS[rem, 1] - av.pub.base_xy[1], W.CENTERS[rem, 0] - av.pub.base_xy[0])
                order = [rem[i] for i in np.argsort(ang, kind="stable")]
                half = len(order) // 2
                for i, c in enumerate(order):
                    self.owner[c] = 1 if i < half else 2
        D = av.k.D[av.cell].copy()
        D[av.k.scanned | av.k.blocked | (self.owner != av.id)] = np.inf
        if not np.isfinite(D).any():
            D = av.k.D[av.cell].copy()
            D[av.k.scanned | av.k.blocked] = np.inf
            if not np.isfinite(D).any():
                return None, []
        order = np.argsort(D)[:4]
        sc = [{"action": "SCAN", "cell": int(c), "score": float(-D[c]), "terms": {"travel_s": round(float(D[c]), 1)}}
              for c in order if np.isfinite(D[c])]
        return {"kind": "search", "cell": int(order[0]), "o": 0, "v": 4.0}, sc


class B4Mule(Policy):
    """B4 find-then-mule / relay, ADAPTED from Hayat et al. 2020 (finder holds; partner ferries the report or
    relays). ADAPTATION: the finder holds up to 45 s calling the partner over air-to-air; a partner that receives
    the call flies toward the finder, takes custody of the report and carries it to a forecast link cell; if nobody
    answers, the finder carries it itself. B1 search."""
    name = "b4_mule"
    speed_fixed = 4.0
    review_mode = "once"
    HOLD_S = 45.0

    def special(self, av, t):
        if av.custody_pending > 0 and t - av.heard_base_t > 3:
            _, txc = av.k.t_deliver(av.cell, t, -1e9)
            return self.move_or_do(av, {"kind": "hold", "cell": txc}, t,
                                   [{"action": "MULE_DELIVER", "cell": int(txc), "score": 1.0}],
                                   "mule: carrying partner report to a link cell")
        req = av.plan.get("mule_req")
        if req and t - req[1] < 90 and t - av.heard_partner_t > 2 and av.custody_pending == 0:
            return self.move_or_do(av, {"kind": "hold", "cell": req[0]}, t,
                                   [{"action": "MULE_FETCH", "cell": int(req[0]), "score": 1.0}],
                                   "mule: flying to the finder to take custody of its report")
        return None

    def handle_evidence(self, av, t, cand):
        if cand.pi() >= TAU and cand.delivered_t is None and t - av.heard_base_t > 5:
            t0 = av.plan.setdefault("hold_t0", {}).setdefault(cand.id, t)
            if t - t0 < self.HOLD_S:
                if int(t - t0) % 10 < 3:
                    av.outbox_requests.append(("MULE", cand.id))
                return self.move_or_do(av, {"kind": "hold", "cell": av.cell}, t,
                                       [{"action": "FINDER_HOLD", "score": 1.0, "terms": {"held_s": t - t0}}],
                                       "finder holds and calls the partner to mule")
            _, txc = av.k.t_deliver(av.cell, t, -1e9)
            return self.move_or_do(av, {"kind": "transmit", "cell": txc, "cid": cand.id, "t0": t}, t,
                                   [{"action": "TRANSMIT", "cell": int(txc), "score": 1.0}],
                                   "no mule answered: finder carries the report")
        return Policy.handle_evidence(self, av, t, cand)


class B45MuleRecheck(B4Mule):
    """B4 + B5: find-then-mule delivery plus threshold re-check before reporting, ADAPTED from Schedl et al. 2021
    (re-sample detections whose confidence is in [tau_lo, tau_hi]); ADAPTATION: thresholds on the candidate
    posterior (tau_lo = 0.15, tau_hi = 0.95), at most 3 re-checks."""
    name = "b45_mule_recheck"
    review_mode = "greedy"


class B5Recheck(B1PodGreedy):
    """B5 threshold re-check alone (Schedl et al. 2021 adaptation) on B1 search."""
    name = "b5_recheck"
    review_mode = "greedy"


class B6Gossip(B1PodGreedy):
    """B6 connectivity-free gossip with ACHORD-style priority queues (Cladera 2024 MOCHA / Saboia 2022 ACHORD,
    ADAPTED): no comms planning at all; data syncs whenever a link happens, confirmed finds first."""
    name = "b6_gossip"
    review_mode = "greedy"

    def handle_evidence(self, av, t, cand):
        if cand.pi() >= TAU:
            return None
        return Policy.handle_evidence(self, av, t, cand)


class Oracle(PODDeadzone):
    name = "oracle"
    uses_truth = True


POLICIES = {c.name: c for c in (Static, Frontier, POD, Coordinator, Lease, Rendezvous, PODMaintain, PODDeadzone,
                                PODFull, Oracle, B1PodGreedy, B2RacerLite, B4Mule, B45MuleRecheck, B5Recheck, B6Gossip)}


def make(name, forced_option=None):
    return POLICIES[name](forced_option=forced_option)
