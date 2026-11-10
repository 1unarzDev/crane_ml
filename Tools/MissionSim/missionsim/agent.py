# SPDX-License-Identifier: GPL-3.0-or-later
"""What a drone (or the base) KNOWS: prior map, team-known coverage, cell posterior with correlated views,
candidates with confidence AND location covariance, partner model, leases, learned link experience.

Nothing in this module may hold a reference to the World (truth). `AgentView` is the only object a planner sees.
"""
import math

import numpy as np

from . import world as W

TAU = 0.95          # local confirmation threshold on candidate posterior
ASSOC_M = 9.0       # candidate association gate (m, map frame)


def logit(p):
    p = min(max(p, 1e-9), 1 - 1e-9)
    return math.log(p / (1 - p))


def sigm(x):
    return 1.0 / (1.0 + math.exp(-max(min(x, 50.0), -50.0)))


def phi(x):
    return 0.5 * (1.0 + math.erf(x / math.sqrt(2.0)))


class Candidate:
    """A possible person. Evidence is kept per contributing drone so the alignment error (common to all detections
    of one drone) does not shrink when that drone re-views; a second drone's detection does shrink it."""

    def __init__(self, cid, est, cell, lo_prior, t):
        self.id = cid
        self.est = np.array(est, float)
        self.cell = int(cell)
        self.lo_prior = lo_prior
        self.t_created = t
        self.ev = {}            # drone -> {lo, views, n, sum_est, s_ind, s_com, nulls{view: n}, t}
        self.delivered_t = None  # (drone-side) time the base acked a DET carrying this candidate
        self.reviews = 0

    def drone_ev(self, d):
        if d not in self.ev:
            self.ev[d] = {"lo": 0.0, "views": [], "n": 0, "sum_est": np.zeros(2), "s_ind": 0.0, "s_com": 0.0,
                          "nulls": {}, "t": 0.0}
        return self.ev[d]

    def lo(self):
        return self.lo_prior + sum(e["lo"] for e in self.ev.values())

    def pi(self):
        return sigm(self.lo())

    def fused(self):
        """(mean, isotropic variance) combining drones by inverse variance; within a drone, measurement noise
        averages down but the common (odometry + alignment) part does not."""
        num, den = np.zeros(2), 0.0
        for e in self.ev.values():
            if e["n"] == 0:
                continue
            m = e["sum_est"] / e["n"]
            var = e["s_ind"] / e["n"] + e["s_com"]
            num += m / var
            den += 1.0 / var
        if den == 0:
            return self.est.copy(), 100.0
        return num / den, 1.0 / den

    def p_loc_ok(self, E):
        _, var = self.fused()
        return 1.0 - math.exp(-E * E / (2.0 * var))

    def views(self):
        s = set()
        for e in self.ev.values():
            s.update(e["views"])
        return s

    def summary(self, E):
        m, var = self.fused()
        pi = self.pi()
        return {"id": self.id, "pi": round(pi, 3), "est": [round(float(m[0]), 1), round(float(m[1]), 1)],
                "sigma_m": round(math.sqrt(var), 2), "p_loc_ok": round(self.p_loc_ok(E), 3),
                "views": sorted(self.views()), "confirmed": pi >= TAU,
                "label": ("CONFIRMED_GOOD_LOC" if pi >= TAU and self.p_loc_ok(E) >= 0.5 else
                          "CONFIDENT_POOR_LOC" if pi >= TAU else "UNCONFIRMED"),
                "delivered": self.delivered_t is not None}

    def payload(self, d):
        """DET payload: this drone's evidence summary (compact report; the base fuses across drones)."""
        e = self.ev.get(d)
        return {"cid": self.id, "lo_prior": self.lo_prior, "drone": d,
                "ev": None if e is None else {"lo": e["lo"], "views": list(e["views"]), "n": e["n"],
                                              "mean": (e["sum_est"] / max(e["n"], 1)).tolist(), "s_ind": e["s_ind"],
                                              "s_com": e["s_com"], "nulls": dict(e["nulls"]), "t": e["t"]},
                "est": self.est.tolist(), "cell": self.cell}


def merge_payload(cands, pl):
    """Fuse a received DET payload into a candidate list (association by distance); returns the candidate."""
    est = np.array(pl["est"])
    if pl["ev"] is not None and pl["ev"]["n"] > 0:
        est = np.array(pl["ev"]["mean"])
    c = None
    for k in cands:
        if k.id == pl["cid"] or np.hypot(*(k.est - est)) < ASSOC_M:
            c = k
            break
    if c is None:
        c = Candidate(pl["cid"], est, pl["cell"], pl["lo_prior"], pl["ev"]["t"] if pl["ev"] else 0.0)
        cands.append(c)
    c.lo_prior = max(c.lo_prior, pl["lo_prior"])
    if pl["ev"] is not None:
        e = pl["ev"]
        if pl["drone"] not in c.ev or c.ev[pl["drone"]]["t"] <= e["t"] or c.ev[pl["drone"]]["n"] < e["n"] \
                or len(c.ev[pl["drone"]]["nulls"]) < len(e["nulls"]):
            c.ev[pl["drone"]] = {"lo": e["lo"], "views": list(e["views"]), "n": e["n"],
                                 "sum_est": np.array(e["mean"]) * e["n"], "s_ind": e["s_ind"], "s_com": e["s_com"],
                                 "nulls": dict(e["nulls"]), "t": e["t"]}
    return c


class Knowledge:
    """Shared belief machinery for drones and the base."""

    def __init__(self, pub, obs):
        self.pub = pub
        self.obs = obs
        self.blocked = pub.blocked_map.copy()
        self.rho = pub.rho_map.copy()
        self.S = np.zeros((W.NC, 2))         # sum log(1 - q) of null scans per (cell, orientation)
        self.scanned = np.zeros(W.NC, bool)   # team-known scanned
        self.n_scans = np.zeros((W.NC, 2), int)
        self.cands = []
        self.next_cid = 1
        self.g = np.array([obs.expected_g(r) for r in self.rho])
        self._D = None
        self.p_link = np.array([phi(m / pub.forecast_sigma) for m in pub.forecast_up])
        self.link_seen = {}                  # cell -> (t_last_heard_base, n_fail_ticks)

    @property
    def D(self):
        if self._D is None:
            self._D = W.travel_matrix(self.blocked, self.rho)
        return self._D

    def mark_blocked(self, c):
        if not self.blocked[c]:
            self.blocked[c] = True
            self._D = None
            return True
        return False

    def add_null_scan(self, c, o, q):
        self.S[c, o] += math.log(max(1.0 - q, 1e-6))
        self.n_scans[c, o] += 1
        self.scanned[c] = True

    def posterior(self):
        g = self.g[:, None]
        L = np.prod(g * np.exp(self.S) + (1 - g), axis=1)
        p = self.pub.p0 * L
        s = p.sum()
        return p / s if s > 0 else p

    def g_post(self, c, o):
        g = self.g[c]
        a = g * math.exp(self.S[c, o])
        return a / (a + 1 - g)

    def pd(self, c, o, v):
        rho = self.rho[c]
        return self.g_post(c, o) * self.obs.expected_pd(rho, v, W.r_eff(rho))

    def lo_prior_for(self, c, v, post=None):
        post = self.posterior() if post is None else post
        rho = self.rho[c]
        a = post[c] * self.g[c] * self.obs.expected_pd(rho, v, W.r_eff(rho))
        b = self.obs.p_false_alert(rho, v, 400.0) * self.pub.fa_mult + (0.06 + 0.10 * rho) * 0.35 * self.obs.p_decoy_detect(v)
        return logit(a / (a + b + 1e-12))

    def find_cand(self, est):
        for k in self.cands:
            if np.hypot(*(k.est - est)) < ASSOC_M:
                return k
        return None

    def t_deliver(self, cur, now, heard_base_t):
        """(expected time to get a compact report to the base from cell `cur`, transmit cell)."""
        if now - heard_base_t <= 3:
            return 3.0, cur
        p = self.p_link.copy()
        for c, (th, nf) in self.link_seen.items():
            if now - th < 60:
                p[c] = max(p[c], 0.95)
            if nf > 8:
                p[c] *= 0.3
        cost = self.D[cur] + 4.0 / np.maximum(p, 0.02) + np.where(self.blocked, 1e4, 0.0)
        c = int(np.argmin(cost))
        return float(cost[c]), c


class AgentView:
    """The only object a planner receives: knowledge + own believed state + inbox-derived partner model."""

    def __init__(self, did, pub, obs, policy):
        self.id = did
        self.partner = 3 - did
        self.k = Knowledge(pub, obs)
        self.pub = pub
        self.policy = policy
        self.cell = pub.start_cells[did - 1]
        self.pose = W.CENTERS[self.cell].copy()
        self.var_odom = 0.0
        self.sig_t, self.sig_th = 0.5, math.radians(1.0)
        self.energy = pub.battery_wh
        self.heard_base_t = 0.0
        self.heard_partner_t = 0.0
        self.partner_hb = None
        self.partner_hb_t = -1e9
        self.partner_heard_me_t = 0.0
        self.partner_lease = None      # (cells, expiry)
        self.lease = None
        self.goal = None               # policy-owned dict
        self.plan = {}                 # policy-owned scratch (excursion, evidence commitments, ...)
        self.assign = None             # coordinator assignment (payload, t)
        self.verify_req = []           # received verification requests
        self.events = []               # pending revision triggers: ("obstacle"|"detection"|"outage", ...)
        self.landed = False
        self.new_cov = []              # scans not yet reported
        self.new_blk = []
        self.last_cov_t = 0.0
        self.last_map_t = 0.0

    def pose_var(self):
        r2 = float(np.sum((self.pose - self.pub.base_xy) ** 2))
        return self.var_odom + self.sig_t ** 2 + self.sig_th ** 2 * r2
