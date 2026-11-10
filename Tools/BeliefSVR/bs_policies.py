# SPDX-License-Identifier: GPL-3.0-or-later
"""Policies for the belief-space search / verify / report simulator (sim.py).  Every adaptation is labelled.

COVER       coverage-optimal greedy sweep (prior mass per second, nearest-weighted); reports the first alert (no verify).
FIXK(k)     same sweep; an alerted cell is re-viewed k times from the same class and reported only if all k alert.
SPRT        Wald SPRT per cell on the iid-per-look LLR (no prior), repeat looks from the same class.  Thresholds log A / log B.
SPRT_DEFL   same, but each class's LLR is scaled by n_eff(k)/k with n_eff = k/(1+(k-1) rho_a) (ESS deflation, the cheap fix).
SPRT_DEFL_ALT  the deflated SPRT that also alternates view classes (a Velez-style decorrelating rule, no planning).
CI_THR      covariance-intersection style: each class contributes its MEAN per-look LLR (worst-case correlation, omega=1/k),
            classes add (independent); alternates classes.
IND_BAYES   exact Bayes belief assuming independent looks (rb=0), report at posterior >= theta, rate-based verify/explore.
CORR        the same machinery with the correlation-aware belief (rb=rho): view class chosen by KL information per second;
            optional partner option.  This is the analytic correlation-aware threshold policy.
ORACLE      knows the target cell (upper bound on time).
"""
import math

import numpy as np

from bs_sim import NC, K


def _sweep_next(m):
    x = m.x
    best, bc, bs = -1.0, None, 0.0
    for c in range(NC):
        if m.visited[c] or x.prior[c] <= 0 or m.ruled[c] or c in m.done:
            continue
        tau = m.travel(c) + x.scan_t[c]
        s = x.prior[c] / tau
        if s > best:
            best, bc = s, c
    return bc


class Cover:
    name = "COVER"
    rb = None

    def decide(self, m):
        for c in np.nonzero(m.hits.sum(1) > 0)[0]:
            if not m.ruled[c]:
                return ("report", int(c))
        c = _sweep_next(m)
        return ("scan", c, 0) if c is not None else None


class FixK:
    rb = None

    def __init__(self, k):
        self.k = k
        self.name = f"FIXK{k}"

    def decide(self, m):
        k = self.k
        for c in np.nonzero(m.hits.sum(1) > 0)[0]:
            c = int(c)
            if m.ruled[c] or c in m.done:
                continue
            nrev = m.cnt[c, 0] - 1                      # looks after the first (scan) alert
            if m.cnt[c, 0] - m.hits[c, 0] > 0:          # any miss: the k-of-k confirmation failed, drop the cell
                m.done.add(c)
                continue
            if nrev >= k:
                return ("report", c)
            return ("rev", c, 0)
        c = _sweep_next(m)
        return ("scan", c, 0) if c is not None else None


class SPRT:
    rb = None

    def __init__(self, A, B=-1.0, defl=None, alt=False, ci=False, name=None):
        self.A, self.B, self.defl, self.alt, self.ci = A, B, defl, alt, ci
        self.name = name or "SPRT"

    def cell_llr(self, m, c):
        tot = 0.0
        for v in range(3):
            k = m.cnt[c, v]
            if k == 0:
                continue
            s = m.llr[c, v]
            if self.ci:
                s = s / k
            elif self.defl is not None and k > 1:
                s = s / (1.0 + (k - 1) * self.defl)
            tot += s
        return tot

    def decide(self, m):
        best, bc = -1e9, None
        for c in np.nonzero(m.hits.sum(1) > 0)[0]:
            c = int(c)
            if m.ruled[c] or c in m.done:
                continue
            l = self.cell_llr(m, c)
            if l >= self.A:
                return ("report", c)
            if l <= self.B:
                m.done.add(c)
                continue
            if m.cnt[c].sum() >= 14:
                m.done.add(c)
                continue
            if l > best:
                best, bc = l, c
        if bc is not None:
            v = 0
            if self.alt or self.ci:
                v = 1 if m.cnt[bc, 0] > m.cnt[bc, 1] else 0
            return ("rev", bc, v)
        c = _sweep_next(m)
        return ("scan", c, 0) if c is not None else None


def _kl(pT, pN):
    pT = min(max(pT, 1e-4), 1 - 1e-4)
    pN = min(max(pN, 1e-4), 1 - 1e-4)
    return pT * math.log(pT / pN) + (1 - pT) * math.log((1 - pT) / (1 - pN))


class Belief:
    """IND_BAYES (rb=0) and CORR (rb=rho): exact belief + rate rule."""

    def __init__(self, rb, theta, name, rule="mi", partner=False, pw=0.5, max_rev=10, lam_drop=0.0):
        self.rb, self.theta, self.name, self.rule, self.partner, self.pw, self.max_rev = rb, theta, name, rule, partner, pw, max_rev
        self.lam_drop = lam_drop
        self.lt = math.log(theta / (1 - theta))

    def verify_option(self, m, P, c):
        B = m.B
        x = m.x
        reg = x.cond.regime
        opts = []
        views = (0,) if self.rule == "same" else (0, 1)
        for v in views:
            tau = m.tau("rev", c, v)
            pT, pN = B.pT_pred("rev", v, c), B.pN_pred("rev", v, c)
            opts.append((_kl(pT, pN) / tau, "rev", v, _kl(pT, pN), tau))
        if self.partner and m.pend is None and m.t + reg.d0 <= reg.D:
            pT, pN = B.pT_pred("rev", 2, c), B.pN_pred("rev", 2, c)
            tau = reg.rev_s + self.pw * reg.d0
            opts.append((_kl(pT, pN) / tau, "partner", 2, _kl(pT, pN), tau))
        return max(opts)

    def decide(self, m):
        """report at posterior >= theta; else keep verifying the alerted cell with the largest posterior until its evidence
        is no better than a fresh cell (lam <= lam_drop: dropped) or max_rev looks; else explore (belief-greedy)."""
        B = m.B
        x = m.x
        P = B.post()
        c = int(np.argmax(P))
        if P[c] >= self.theta:
            return ("report", c)
        pend_c = m.pend[0] if m.pend else -1
        cand, best = None, -1.0
        for c in np.nonzero(B.n_alert > 0)[0]:
            c = int(c)
            if B.ruled[c] or c in m.done or c == pend_c or P[c] <= 0:
                continue
            if B.lam[c] <= self.lam_drop or B.n_look[c].sum() >= self.max_rev + 1:
                m.done.add(c)
                continue
            if P[c] > best:
                best, cand = P[c], c
        if cand is not None:
            rate_i, kind, v, kl, tau = self.verify_option(m, P, cand)
            return ("partner", cand) if kind == "partner" else ("rev", cand, v)
        return self.explore(m, P)[0]

    def explore(self, m, P):
        B, x = m.B, m.x
        s0 = B.pT_pred("scan", 0)
        s1 = B.pT_pred("scan", 1)
        tr = np.where(np.arange(NC) == m.pos, 0.0, x.D[m.pos])
        tau0 = tr + x.scan_t
        tau1 = tau0 + x.cond.regime.sw_s
        r0 = P * s0 / tau0
        r1 = P * s1 / tau1 if self.rule != "same" else np.zeros(NC)
        r0 = np.where(P > 0, r0, -1)
        c0, c1 = int(np.argmax(r0)), int(np.argmax(r1))
        rexp, ce, ve = (r0[c0], c0, 0) if r0[c0] >= r1[c1] else (r1[c1], c1, 1)
        return ("scan", ce, ve), rexp


class _Local:
    """Single-cell copy of the belief for rollouts (exact for the cell, other cells enter only through the prior odds)."""

    def __init__(self, B, c, P):
        self.B, self.c = B, c
        self.WT, self.WD = B.WT[c].copy(), B.WD[c].copy()
        self.lnLT, self.lnLD, self.lnLE = B.lnLT[c].copy(), B.lnLD[c].copy(), float(B.lnLE[c])
        self.lam0 = float(B.lam[c])
        self.lam = self.lam0
        self.P0 = float(P[c])
        self.lo0 = math.log(max(P[c], 1e-12) / max(1 - P[c], 1e-12))

    def update(self, v, kind, y):
        B, x = self.B, self.B.x
        qT = B.Q[kind][self.c]
        f = qT if y else 1 - qT
        row = self.WT[v] * f; s = row.sum()
        self.lnLT[v] += math.log(max(s, 1e-300)); self.WT[v] = row / max(s, 1e-300)
        f = B.QD if y else 1 - B.QD
        row = self.WD[v] * f; s = row.sum()
        self.lnLD[v] += math.log(max(s, 1e-300)); self.WD[v] = row / max(s, 1e-300)
        self.lnLE += math.log(x.alpha if y else 1 - x.alpha)
        e = np.logaddexp(B.ln1pi[self.c] + self.lnLE, B.lnpi[self.c] + self.lnLD.sum())
        self.lam = self.lnLT.sum() - e

    def P(self):
        lo = self.lo0 + (self.lam - self.lam0)
        return 1.0 / (1.0 + math.exp(-lo))


class PomcpLite(Belief):
    """POMCP-lite: the correlation-aware threshold policy decides explore vs verify; when it chooses to verify (or the
    candidate is near the report threshold) the verification action {report, review same class, review other class,
    partner} is chosen by particle rollouts (common random numbers) of a single-cell subgame: truth sampled from the
    belief (target / decoy, latent z), default rollout policy = alternate classes until posterior >= theta (report),
    <= 0.4 x initial (drop) or 6 looks.  Cost = elapsed + c_FR * wrong report + R_hat (time to resume the search) on a
    drop or wrong report, + the remaining deadline if the real target was dropped.  Not a UCT tree: one-step improvement
    of the default rollout policy (labelled POMCP-lite)."""

    def __init__(self, rb, theta, name="POMCP_LITE", n_part=16, partner=True, pw=0.5):
        super().__init__(rb, theta, name, rule="mi", partner=partner, pw=pw)
        self.n_part = n_part
        self.rng = np.random.default_rng(12345)

    def decide(self, m):
        a = super().decide(m)
        if a[0] not in ("rev", "partner"):
            return a
        c = a[1]
        B, x = m.B, m.x
        reg = x.cond.regime
        P = B.post()
        opts = [("rev", c, 0), ("rev", c, 1)]
        if self.partner and m.pend is None and m.t + reg.d0 <= reg.D:
            opts.append(("partner", c))
        opts.append(("report", c))
        Rhat = self.resume_cost(m, P, c)
        U = self.rng.random((self.n_part, 3, 12))
        us = self.rng.random(self.n_part)
        vals = []
        for o in opts:
            tot = 0.0
            for i in range(self.n_part):
                tot += self.rollout(m, B, P, c, o, Rhat, U[i], us[i], reg)
            vals.append(tot / self.n_part)
        return opts[int(np.argmin(vals))]

    def resume_cost(self, m, P, c):
        x = m.x
        w = P.copy(); w[c] = 0.0
        s = w.sum()
        if s <= 0:
            return 0.0
        w /= s
        tau = np.where(w > 0, x.scan_t + x.D[c], 1e9)
        o = np.argsort(-w / np.maximum(tau, 1.0))
        cum = np.cumsum(np.where(w[o] > 0, tau[o], 0.0))
        return float(min(np.sum(w[o] * cum), x.cond.regime.D - m.t))

    def rollout(self, m, B, P, c, opt, Rhat, u, us, reg):
        x = m.x
        Lc = _Local(B, c, P)
        isT = us < P[c]
        # truth latents for the three classes
        gT = [int(np.searchsorted(np.cumsum(Lc.WT[v]), u[v, 0])) for v in range(3)]
        gT = [min(g, K - 1) for g in gT]
        pid = math.exp(B.lnpi[c] + Lc.lnLD.sum() - np.logaddexp(B.ln1pi[c] + Lc.lnLE, B.lnpi[c] + Lc.lnLD.sum()))
        isD = (not isT) and (u[0, 1] < pid)
        gD = [min(int(np.searchsorted(np.cumsum(Lc.WD[v]), u[v, 2])), K - 1) for v in range(3)]
        k = [0, 0, 0]

        def look(v, kind):
            j = 3 + k[v]
            k[v] += 1
            uu = u[v, 3 + (j % 8)]
            if isT:
                y = uu < B.Q[kind][c][gT[v]]
            elif isD:
                y = uu < B.QD[gD[v]]
            else:
                y = uu < x.alpha
            Lc.update(v, kind, bool(y))

        elapsed = 0.0
        first = True
        n_looks = 0
        cur = opt
        v_last = m.w if m.pos == c else 0
        while True:
            if cur[0] == "report":
                return elapsed + (0.0 if isT else reg.cFR + Rhat)
            if cur[0] == "partner":
                elapsed += reg.rev_s + self.pw * reg.d0
                look(2, "rev")
                v_last = 2
            else:
                v = cur[2]
                sw = reg.sw_s if (v == 1 and v_last != 1) else 0.0
                elapsed += reg.rev_s + sw + (m.travel(c) if first and c != m.pos else 0.0)
                look(v, "rev")
                v_last = v
            first = False
            n_looks += 1
            p = Lc.P()
            if p >= self.theta:
                return elapsed + (0.0 if isT else reg.cFR + Rhat)
            if p <= 0.4 * Lc.P0 or n_looks >= 6:
                return elapsed + Rhat + ((reg.D - m.t) if isT else 0.0)
            cur = ("rev", c, 1 if v_last != 1 else 0)


class Oracle:
    name = "ORACLE"
    rb = None

    def decide(self, m):
        c = m.x.target
        if m.hits[c].sum() > 0:
            return ("report", c)
        return ("scan", c, 0) if m.cnt[c].sum() == 0 else ("rev", c, 0 if m.cnt[c, 0] < 2 else 1)


def make_factory(spec):
    kind = spec["kind"]
    if kind == "COVER":
        return lambda: Cover()
    if kind == "FIXK":
        return lambda: FixK(spec["k"])
    if kind == "SPRT":
        return lambda: SPRT(spec["A"], name=spec.get("name", "SPRT"))
    if kind == "SPRT_DEFL":
        return lambda: SPRT(spec["A"], defl=spec["rho_a"], alt=spec.get("alt", False), name=spec.get("name", "SPRT_DEFL"))
    if kind == "CI":
        return lambda: SPRT(spec["A"], ci=True, name=spec.get("name", "CI_THR"))
    if kind == "BAYES":
        return lambda: Belief(spec["rb"], spec["theta"], spec.get("name", "BAYES"), rule=spec.get("rule", "mi"),
                              partner=spec.get("partner", False), pw=spec.get("pw", 0.5))
    if kind == "POMCP":
        return lambda: PomcpLite(spec["rb"], spec["theta"], spec.get("name", "POMCP_LITE"), n_part=spec.get("n_part", 16))
    if kind == "ORACLE":
        return lambda: Oracle()
    raise KeyError(kind)
