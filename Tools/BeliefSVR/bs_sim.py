# SPDX-License-Identifier: GPL-3.0-or-later
"""Belief-level search / verify / report simulator on MissionSim layouts (DEV only until PREREG_BELIEF is frozen).

Evidence label: IMPLEMENTED (synthetic mission level; TPR/decoy numbers from Tools/Vision/obs_model.py, the
cross-view correlation and the background false-alert rate are PRIOR and flagged for the Vision rendered rerun).

Reuses MissionSim: missionsim.world (layout, prior, canopy, blockages, travel matrix, scan time) and the Vision ObsModel
(per-pass marginal alert probability).  It does NOT reuse missionsim.sim (its observation process draws a per-view
Bernoulli latent with no correlation knob, it carries a full comms stack, and it has no verify/report action API).
Mechanics: one searching drone (1 s resolution, travel times from the MissionSim travel graph), actions
scan(cell, class), review(cell, class), partner(cell), report(cell); a partner drone answers a request after d0 s with
an independent class-2 look and does not occupy the searcher.  A wrong report costs c_FR in the objective, rules the
cell out and the mission continues.  Common random numbers across policies: all latents / look noises are drawn once
per (layout, rep) and indexed by (cell, class, look number).
"""
import math
import os
import sys
from dataclasses import dataclass, field, replace

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
MS = os.path.normpath(os.path.join(HERE, "..", "MissionSim"))
if MS not in sys.path:
    sys.path.insert(0, MS)
if HERE not in sys.path:
    sys.path.insert(0, HERE)
from missionsim import world as W  # noqa: E402
from missionsim import obs_model as MO  # noqa: E402
import bs_copula as C  # noqa: E402

NC = W.NC
K = C.K_NODES
NL = 16  # looks per (cell, class) that have pre-drawn noise
FLEVELS = {  # f level -> (background alert prob per look alpha, decoy fire prob fd, decoy density multiplier)
    "low": (0.01, 0.10, 1.0),
    "measured": (0.05, 0.28, 1.0),   # fd = Vision p_decoy_detect (0.8) x MissionSim decoy per-view visibility (0.35); alpha PRIOR
    "high": (0.15, 0.50, 2.0),
}
_OBS = None


def obs_model():
    global _OBS
    if _OBS is None:
        _OBS = MO.load(prefer_vision=True, vis_table=W.vis_calibration())
    return _OBS


@dataclass(frozen=True)
class Regime:
    name: str = "coverage"
    K: int = 100              # search area: top-K cells by public prior (target drawn inside)
    D: float = 900.0          # deadline s
    rev_s: float = 8.0        # one review look (hover re-view)
    sw_s: float = 12.0        # reposition to an independent own view class
    d0: float = 25.0          # partner response delay
    cFR: float = 300.0        # false-report cost (s) in J


@dataclass(frozen=True)
class Cond:
    rho: float = 0.6          # true repeat-look ICC (target and decoy alerts)
    f: str = "measured"       # key of FLEVELS
    regime: Regime = field(default_factory=Regime)
    rho_cross: float = 0.0    # not used (cross-view independence assumed; flagged)


class Ctx:
    """Everything fixed by (layout, condition) except the observation noise (rep)."""

    def __init__(self, layout, cond: Cond):
        self.cond, self.layout = cond, layout
        obs = obs_model()
        w = W.World(layout, W.Scenario(), obs=obs)
        self.w = w
        reg = cond.regime
        p0 = w.p0.copy()
        if reg.K < NC:
            keep = np.argsort(-p0)[:reg.K]
            m = np.zeros(NC, bool); m[keep] = True
            p0 = np.where(m, p0, 0.0)
            p0 /= p0.sum()
            rq = W.rng(layout, "person", 17)
            tc = int(rq.choice(NC, p=p0))
        else:
            tc = w.target_cell
            if p0[tc] == 0:
                p0[tc] = 1e-6; p0 /= p0.sum()
        self.prior = p0
        self.target = tc
        self.D = W.travel_matrix(w.blocked, w.rho)
        rho = w.rho
        self.rho_c = rho
        v = np.minimum(W.vmax(rho), 4.0)
        self.scan_t = np.array([W.scan_time(rho[c], v[c]) for c in range(NC)])
        # planner's marginal TPRs (map canopy, expected occlusion): scan pass and review look
        rm = w.rho_map
        self.g_plan = np.array([obs.expected_g(rm[c]) for c in range(NC)])
        self.pd_scan = np.array([obs.expected_pd(rm[c], min(W.vmax(rm[c]), 4.0), W.r_eff(rm[c])) for c in range(NC)])
        self.pd_rev = np.array([obs.expected_pd(rm[c], 1.5, W.r_eff(rm[c]), "review") for c in range(NC)])
        self.p_scan = np.clip(self.g_plan * self.pd_scan, 0.02, 0.95)
        self.p_rev = np.clip(self.g_plan * self.pd_rev, 0.02, 0.95)
        # truth TPR for the target cell (true occlusion, true canopy)
        gt = obs.p_view_good(rho[tc], w.person_occl)
        self.tp_scan = float(np.clip(gt * obs.expected_pd(rho[tc], min(W.vmax(rho[tc]), 4.0), W.r_eff(rho[tc])), 0.02, 0.95))
        self.tp_rev = float(np.clip(gt * obs.expected_pd(rho[tc], 1.5, W.r_eff(rho[tc]), "review"), 0.02, 0.95))
        a_bg, fd, dm = FLEVELS[cond.f]
        self.alpha, self.fd = a_bg, fd
        self.pid_true = np.clip(dm * (0.06 + 0.10 * rho), 0, 0.6)
        self.pid_plan = np.clip(dm * (0.06 + 0.10 * rm), 0, 0.6)
        rd = W.rng(layout, "decoy", 29)
        dec = rd.random(NC) < self.pid_true
        dec[tc] = False
        self.decoys = dec
        # latent correlations (calibrated so the binary ICC at p_ref equals cond.rho)
        self.rT = C.latent_for_icc(0.6, cond.rho)
        self.rD = C.latent_for_icc(self.fd, cond.rho)
        self.t_scan_T = C.probit(self.tp_scan)
        self.t_rev_T = C.probit(self.tp_rev)
        self.t_fd = C.probit(self.fd)
        self.blocked = w.blocked

    def noise(self, rep):
        r = W.rng(self.layout, "obs", 1000 + rep)
        return dict(zT=r.standard_normal((NC, 3)), zD=r.standard_normal((NC, 3)),
                    e=r.standard_normal((NC, 3, NL)), u=r.random((NC, 3, NL)))


class Truth:
    """Look outcomes under the true correlated model; counts looks per (cell, class)."""

    def __init__(self, ctx: Ctx, noise):
        self.c, self.n = ctx, noise
        self.k = np.zeros((NC, 3), int)

    def look(self, c, v, kind):
        x = self.c
        k = min(self.k[c, v], NL - 1)
        self.k[c, v] += 1
        if c == x.target:
            t = x.t_scan_T if kind == "scan" else x.t_rev_T
            r = x.rT
            return bool(math.sqrt(r) * self.n["zT"][c, v] + math.sqrt(1 - r) * self.n["e"][c, v, k] < t)
        if x.decoys[c]:
            r = x.rD
            return bool(math.sqrt(r) * self.n["zD"][c, v] + math.sqrt(1 - r) * self.n["e"][c, v, k] < x.t_fd)
        return bool(self.n["u"][c, v, k] < x.alpha)


class Belief:
    """Exact factored belief (target cell x per-cell decoy x latent z on K nodes) under an assumed ICC rb.
    rb = 0 is the independence-assuming belief.  Planner parameters only (map canopy, public decoy density)."""

    def __init__(self, ctx: Ctx, rb):
        self.x, self.rb = ctx, rb
        x = ctx
        rT = C.latent_for_icc(0.6, rb)
        rD = C.latent_for_icc(x.fd, rb)
        nodes, w = C.nodes()
        self.w0 = w
        Phi = C.Phi
        self.Q = {}
        for kind, p in (("scan", x.p_scan), ("rev", x.p_rev)):
            t = np.array([C.probit(v) for v in p])
            if rT <= 1e-12:
                self.Q[kind] = np.repeat(p[:, None], K, 1)
            else:
                self.Q[kind] = Phi((t[:, None] - math.sqrt(rT) * nodes[None, :]) / math.sqrt(1 - rT))
        self.QD = C.q_of_z(C.probit(x.fd), rD)
        self.WT = np.tile(w, (NC, 3, 1))
        self.WD = np.tile(w, (NC, 3, 1))
        self.lnLT = np.zeros((NC, 3))
        self.lnLD = np.zeros((NC, 3))
        self.lnLE = np.zeros(NC)
        self.lnpi = np.log(np.maximum(x.pid_plan, 1e-9))
        self.ln1pi = np.log1p(-x.pid_plan)
        self.lnprior = np.log(np.maximum(x.prior, 1e-300))
        self.ruled = np.zeros(NC, bool)
        self.n_alert = np.zeros(NC, int)
        self.n_look = np.zeros((NC, 3), int)
        self.lam = np.zeros(NC)       # log (T_c / E_c)
        self.hit = np.zeros((NC, 3), int)

    def clone(self):
        b = object.__new__(Belief)
        b.__dict__.update(self.__dict__)
        for k in ("WT", "WD", "lnLT", "lnLD", "lnLE", "ruled", "n_alert", "n_look", "lam", "hit"):
            setattr(b, k, getattr(self, k).copy())
        return b

    def update(self, c, v, kind, y):
        x = self.x
        qT = self.Q[kind][c]
        f = qT if y else 1 - qT
        row = self.WT[c, v] * f
        s = row.sum()
        self.lnLT[c, v] += math.log(max(s, 1e-300))
        self.WT[c, v] = row / max(s, 1e-300)
        qD = self.QD
        f = qD if y else 1 - qD
        row = self.WD[c, v] * f
        s = row.sum()
        self.lnLD[c, v] += math.log(max(s, 1e-300))
        self.WD[c, v] = row / max(s, 1e-300)
        self.lnLE[c] += math.log(x.alpha if y else 1 - x.alpha)
        self.n_look[c, v] += 1
        if y:
            self.n_alert[c] += 1
            self.hit[c, v] += 1
        e = np.logaddexp(self.ln1pi[c] + self.lnLE[c], self.lnpi[c] + self.lnLD[c].sum())
        self.lam[c] = self.lnLT[c].sum() - e

    def post(self):
        lp = self.lnprior + self.lam
        lp = np.where(self.ruled, -np.inf, lp)
        m = lp.max()
        p = np.exp(lp - m)
        return p / p.sum()

    def pT_pred(self, kind, v, c=None):
        if c is None:
            return (self.WT[:, v, :] * self.Q[kind]).sum(-1)
        return float((self.WT[c, v] * self.Q[kind][c]).sum())

    def pN_pred(self, kind, v, c):
        x = self.x
        ld = self.lnLD[c].sum()
        e = np.logaddexp(self.ln1pi[c] + self.lnLE[c], self.lnpi[c] + ld)
        pid = math.exp(self.lnpi[c] + ld - e)
        pd_ = float((self.WD[c, v] * self.QD).sum())
        return pid * pd_ + (1 - pid) * x.alpha


# ====================================================================== mission engine
class Mission:
    """One mission. `policy.decide(m)` returns an action tuple:
    ('scan', c, v) ('rev', c, v) ('partner', c) ('report', c) or None (idle)."""

    def __init__(self, ctx: Ctx, noise, policy):
        self.x, self.policy = ctx, policy
        self.T = Truth(ctx, noise)
        self.t = 0.0
        self.pos = int(ctx.w.start_cells[0])
        self.w = 0                      # view class currently held at pos
        self.pend = None                # (cell, t_arrive)
        self.B = Belief(ctx, policy.rb) if getattr(policy, "rb", None) is not None else None
        self.ruled = np.zeros(NC, bool)
        self.visited = np.zeros(NC, bool)
        self.cnt = np.zeros((NC, 3))                 # looks per (cell, class)
        self.hits = np.zeros((NC, 3))
        self.llr = np.zeros((NC, 3))                 # iid-model LLR per (cell, class)
        self.done = set()                            # cells dropped by a verification rule
        self.ps = {}                                 # policy scratch
        self.FR = 0
        self.succ_t = None
        self.n_scan = self.n_rev = self.n_indep = self.n_partner = 0
        self.target_looks = 0
        self.target_alerts = 0
        self.end_pend_target = False
        self.n_reports = 0
        self.fr_cells = []
        self.rep_log = []
        self.t_rev = 0.0

    # --- helpers for policies
    def travel(self, c):
        return 0.0 if c == self.pos else float(self.x.D[self.pos, c])

    def tau(self, kind, c, v):
        x = self.x
        base = x.scan_t[c] if kind == "scan" else x.cond.regime.rev_s
        sw = x.cond.regime.sw_s if (v == 1 and not (c == self.pos and self.w == 1)) else 0.0
        if kind == "rev" and v == 0 and c == self.pos and self.w == 1:
            sw = x.cond.regime.sw_s
        return self.travel(c) + base + sw

    def iid_llr(self, c, kind, y):
        x = self.x
        p = x.p_scan[c] if kind == "scan" else x.p_rev[c]
        pn = x.pid_plan[c] * x.fd + (1 - x.pid_plan[c]) * x.alpha
        return math.log(p / pn) if y else math.log((1 - p) / (1 - pn))

    def _look(self, c, v, kind):
        y = self.T.look(c, v, kind)
        if c == self.x.target:
            self.target_looks += 1
            self.target_alerts += int(y)
        self.cnt[c, v] += 1
        self.hits[c, v] += int(y)
        self.llr[c, v] += self.iid_llr(c, kind, y)
        if self.B is not None:
            self.B.update(c, v, kind, y)
        return y

    def report(self, c):
        self.n_reports += 1
        if self.B is not None:
            self.rep_log.append((round(float(self.B.post()[c]), 3), int(c == self.x.target)))
        if c == self.x.target:
            self.succ_t = self.t
            return True
        self.FR += 1
        self.fr_cells.append(c)
        self.ruled[c] = True
        if self.B is not None:
            self.B.ruled[c] = True
        self.done.add(c)
        return False

    def run(self):
        x = self.x
        D = x.cond.regime.D
        guard = 0
        while self.t < D and self.succ_t is None and guard < 4000:
            guard += 1
            if self.pend is not None and self.t >= self.pend[1]:
                c = self.pend[0]
                self.pend = None
                self._look(c, 2, "rev")
                continue
            a = self.policy.decide(self)
            if a is None:   # idle until the partner answers (or the end)
                if self.pend is not None:
                    self.t = self.pend[1]
                    continue
                break
            k = a[0]
            if k == "report":
                self.report(a[1])
                continue
            if k == "partner":
                c = a[1]
                if self.pend is not None or self.t + x.cond.regime.d0 > D:
                    a = None
                    break
                self.pend = (c, self.t + x.cond.regime.d0)
                self.n_partner += 1
                continue
            c, v = a[1], a[2]
            cost = self.tau(k, c, v)
            if self.t + cost > D:
                self.t = D
                break
            self.t += cost
            if k == "rev":
                self.t_rev += cost
            self.pos, self.w = c, v
            self.visited[c] = True
            if k == "scan":
                self.n_scan += 1
            else:
                self.n_rev += 1
                if v == 1:
                    self.n_indep += 1
            self._look(c, v, k)
        if self.pend is not None and self.pend[0] == x.target and self.succ_t is None:
            self.end_pend_target = True
        return self.result()

    def result(self):
        x = self.x
        D = x.cond.regime.D
        ok = self.succ_t is not None
        cause = ""
        if not ok:
            if self.target_looks == 0:
                cause = "route"
            elif self.target_alerts == 0:
                cause = "visibility"
            elif self.end_pend_target:
                cause = "communication"
            elif self.FR > 0:
                cause = "association"
            else:
                cause = "inference"
        verifs = self.n_rev + self.n_partner
        return dict(ok=int(ok), T=float(self.succ_t if ok else D), FR=self.FR, reports=self.n_reports,
                    verifs=verifs, n_scan=self.n_scan, n_rev=self.n_rev, n_indep=self.n_indep, n_partner=self.n_partner,
                    cause=cause, t_end=float(self.t), t_rev=float(self.t_rev), rlog=self.rep_log,
                    fr_decoy=int(sum(bool(self.x.decoys[c]) for c in self.fr_cells)))


def run_mission(ctx, rep, policy_factory):
    return Mission(ctx, ctx.noise(rep), policy_factory()).run()
