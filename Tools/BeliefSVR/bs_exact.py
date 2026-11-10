# SPDX-License-Identifier: GPL-3.0-or-later
"""Exact expectimax for the correlated search / verify / report problem on tiny instances (n <= 6 cells, 2 view classes,
horizon <= 6 steps).  Evidence label: IMPLEMENTED (synthetic; parameters from Tools/Vision/obs_model.py are inputs).

World.  One target in cell i ~ prior.  Each non-target cell holds a persistent decoy w.p. pid.  Per (cell, view class)
a latent z (Gaussian copula, module copula.py) is shared by repeat looks of that class and independent across classes:
  target cell : alert w.p. pT per look (class 0/1, own drone) or pP (class 2, partner)
  decoy cell  : alert w.p. fd per look, same ICC (persistent false positive)
  empty cell  : alert w.p. alpha iid.
Actions (unit step each): look(v) at the current cell (+s steps if the class changes: reposition to an independent view),
move(j) (mv steps, arrive in class 0), partner(c) (free for the drone, result of a class-2 look at c arrives after d
steps; one request at a time), wait, report(c) (instant; correct ends the episode, wrong costs cFR and rules c out).
Objective J = E[time of the correct report] + (1 - P(success)) (H + cmiss) + cFR * E[#false reports].

Belief rho_b (the ICC the planner assumes) may differ from the true rho: plan with rho_b, evaluate that policy under rho.
Sufficient statistic = per-(cell, class) alert / non-alert counts, so beliefs are exact products (see Z()).
"""
from __future__ import annotations

import itertools
import sys
from dataclasses import dataclass, replace
from functools import lru_cache

import numpy as np

import bs_copula as C


@dataclass(frozen=True)
class XI:
    n: int = 3
    prior: tuple = (1 / 3, 1 / 3, 1 / 3)
    pT: float = 0.6
    pP: float = 0.6
    fd: float = 0.6
    pid: float = 0.3
    alpha: float = 0.02
    rho: float = 0.6          # TRUE ICC (target and decoy alerts)
    mv: int = 1
    s: int = 1                # extra steps to look from the other own view class
    d: int = 2                # partner delay (steps)
    cFR: float = 6.0
    H: int = 6
    cmiss: float = 6.0
    allow_indep: bool = True
    allow_partner: bool = False
    allow_move: bool = True


class Model:
    """Likelihood tables for a belief ICC rb."""

    def __init__(self, I: XI, rb: float):
        self.I, self.rb = I, rb
        H = I.H + 1
        rT = C.latent_for_icc(I.pT, rb)
        rP = C.latent_for_icc(I.pP, rb)
        rD = C.latent_for_icc(I.fd, rb)
        self.LT = [self._tab(I.pT, rT), self._tab(I.pT, rT), self._tab(I.pP, rP)]
        self.LD = [self._tab(I.fd, rD)] * 3
        self.H = H
        pr = np.array(I.prior, float)
        self.prior = pr / pr.sum()

    def _tab(self, p, r):
        n = self.I.H + 1
        return [[C.count_lik(p, r, h, m) if h + m <= self.I.H else 0.0 for m in range(n)] for h in range(n)]

    def cell_terms(self, cnt, c):
        I = self.I
        T = D = 1.0
        hh = mm = 0
        for v in range(3):
            h, m = cnt[3 * c + v]
            T *= self.LT[v][h][m]
            D *= self.LD[v][h][m]
            hh += h
            mm += m
        Ebg = I.alpha ** hh * (1 - I.alpha) ** mm
        return T, (1 - I.pid) * Ebg + I.pid * D

    def Z(self, cnt, ruled):
        n = self.I.n
        TE = [self.cell_terms(cnt, c) for c in range(n)]
        E = [e for _, e in TE]
        tot, post = 0.0, np.zeros(n)
        for i in range(n):
            if ruled >> i & 1 or self.prior[i] == 0:
                continue
            w = self.prior[i] * TE[i][0] / TE[i][1]
            post[i] = w
        pe = float(np.prod(E))
        return pe * post.sum(), post

    def posterior(self, cnt, ruled):
        _, post = self.Z(cnt, ruled)
        s = post.sum()
        return post / s if s > 0 else post

    def p_alert(self, cnt, ruled, c, v):
        """Predictive P(next look at (c, class v) alerts | counts)."""
        a = list(cnt); h, m = a[3 * c + v]; a[3 * c + v] = (h + 1, m)
        b = list(cnt); b[3 * c + v] = (h, m + 1)
        z1, _ = self.Z(tuple(a), ruled)
        z0, _ = self.Z(tuple(b), ruled)
        return z1 / (z1 + z0) if z1 + z0 > 0 else 0.0


def _bump(cnt, c, v, y):
    a = list(cnt)
    h, m = a[3 * c + v]
    a[3 * c + v] = (h + 1, m) if y else (h, m + 1)
    return tuple(a)


class Solver:
    def __init__(self, I: XI, rb=None):
        self.I = I
        self.rb = I.rho if rb is None else rb
        self.plan_m = Model(I, self.rb)
        self.true_m = self.plan_m if abs(self.rb - I.rho) < 1e-12 else Model(I, I.rho)
        self._V, self._A, self._E = {}, {}, {}
        self.n_states = 0

    # ---- state: (t, pos, w, cnt, ruled, pend); pend = (cell, t_arrive) | None
    def start(self, pos=0):
        return (0, pos, 0, tuple((0, 0) for _ in range(3 * self.I.n)), 0, None)

    def _resolve(self, st):
        t, pos, w, cnt, ruled, pend = st
        if pend is not None and t >= pend[1]:
            return pend[0]
        return None

    def actions(self, st):
        I = self.I
        t, pos, w, cnt, ruled, pend = st
        acts = []
        for c in range(I.n):
            if not (ruled >> c & 1) and self.plan_m.prior[c] > 0:
                acts.append(("R", c))
        for v in (0, 1):
            if v == 1 and not I.allow_indep:
                continue
            cost = 1 + (I.s if v != w else 0)
            if t + cost <= I.H:
                acts.append(("L", v, cost))
        if I.allow_partner and pend is None and t + I.d <= I.H:
            for c in range(I.n):
                acts.append(("P", c))
        if pend is not None and t + 1 <= I.H and pend[1] <= I.H:
            acts.append(("W",))
        if I.allow_move and t + I.mv <= I.H:
            for j in range(I.n):
                if j != pos:
                    acts.append(("M", j))
        return acts

    def _next_after_report(self, st, c):
        t, pos, w, cnt, ruled, pend = st
        return (t, pos, w, cnt, ruled | (1 << c), pend)

    def _apply(self, st, a):
        """List of (key, next_state, prob_fn_tag) for chance outcomes; deterministic actions return one branch."""
        t, pos, w, cnt, ruled, pend = st
        if a[0] == "L":
            return [(pos, a[1], a[2])]
        raise AssertionError

    # ---- planning value (belief plan_m)
    def V(self, st):
        if st in self._V:
            return self._V[st]
        self.n_states += 1
        I = self.I
        t, pos, w, cnt, ruled, pend = st
        rc = self._resolve(st)
        if rc is not None:
            p1 = self.plan_m.p_alert(cnt, ruled, rc, 2)
            s0 = (t, pos, w, cnt, ruled, None)
            val = 0.0
            for y, py in ((1, p1), (0, 1 - p1)):
                if py > 0:
                    val += py * self.V(self._with(s0, rc, 2, y))
            self._V[st] = val
            self._A[st] = ("RESOLVE",)
            return val
        best, besta = I.H + I.cmiss, ("STOP",)
        P = self.plan_m.posterior(cnt, ruled)
        for a in self.actions(st):
            q = self._Q(st, a, P, self.V)
            if q < best - 1e-12:
                best, besta = q, a
        self._V[st], self._A[st] = best, besta
        return best

    def _with(self, st, c, v, y):
        t, pos, w, cnt, ruled, pend = st
        return (t, pos, w, _bump(cnt, c, v, y), ruled, pend)

    def _Q(self, st, a, P, rec):
        I = self.I
        t, pos, w, cnt, ruled, pend = st
        if a[0] == "R":
            c = a[1]
            pc = P[c]
            rest = 0.0
            if pc < 1 - 1e-12:
                rest = I.cFR + rec(self._next_after_report(st, c))
            return pc * t + (1 - pc) * rest
        if a[0] == "L":
            v, cost = a[1], a[2]
            pa = self.plan_m.p_alert(cnt, ruled, pos, v)
            val = 0.0
            for y, py in ((1, pa), (0, 1 - pa)):
                if py > 1e-15:
                    val += py * rec((t + cost, pos, v, _bump(cnt, pos, v, y), ruled, pend))
            return val
        if a[0] == "M":
            return rec((t + I.mv, a[1], 0, cnt, ruled, pend))
        if a[0] == "P":
            return rec((t, pos, w, cnt, ruled, (a[1], t + I.d)))
        if a[0] == "W":
            return rec((t + 1, pos, w, cnt, ruled, pend))
        raise AssertionError(a)

    def choose(self, st):
        self.V(st)
        return self._A[st]

    # ---- evaluation of the planned policy under the TRUE model: vector [E t*1(correct), E #FR, P(success)]
    def E(self, st):
        if st in self._E:
            return self._E[st]
        a = self.choose(st)
        t, pos, w, cnt, ruled, pend = st
        tm = self.true_m
        out = np.zeros(7)
        if a[0] == "STOP":
            pass
        elif a[0] == "RESOLVE":
            rc = pend[0]
            p1 = tm.p_alert(cnt, ruled, rc, 2)
            s0 = (t, pos, w, cnt, ruled, None)
            for y, py in ((1, p1), (0, 1 - p1)):
                if py > 0:
                    out += py * self.E(self._with(s0, rc, 2, y))
        else:
            P = tm.posterior(cnt, ruled)
            if a[0] == "R":
                c = a[1]
                pc = P[c]
                out += np.array([pc * t, 0.0, pc, 0, 0, 0, 0])
                if pc < 1 - 1e-12:
                    out += (1 - pc) * (np.array([0.0, 1.0, 0.0, 0, 0, 0, 0]) + self.E(self._next_after_report(st, c)))
            elif a[0] == "L":
                v, cost = a[1], a[2]
                pa = tm.p_alert(cnt, ruled, pos, v)
                if v == 1:
                    kind = 5
                elif sum(cnt[3 * pos + 0]) > 0:
                    kind = 4  # repeat from the same class
                else:
                    kind = 3  # first look from class 0 = explore
                e = np.zeros(7); e[kind] = 1.0
                out += e
                for y, py in ((1, pa), (0, 1 - pa)):
                    if py > 1e-15:
                        out += py * self.E((t + cost, pos, v, _bump(cnt, pos, v, y), ruled, pend))
            elif a[0] == "M":
                out += self.E((t + self.I.mv, a[1], 0, cnt, ruled, pend))
            elif a[0] == "P":
                e = np.zeros(7); e[6] = 1.0
                out += e + self.E((t, pos, w, cnt, ruled, (a[1], t + self.I.d)))
            elif a[0] == "W":
                out += self.E((t + 1, pos, w, cnt, ruled, pend))
        self._E[st] = out
        return out

    def summary(self):
        s = self.start()
        self.V(s)
        Tc, fr, succ, nE, nR, nI, nP = self.E(s)
        I = self.I
        J = Tc + (1 - succ) * (I.H + I.cmiss) + I.cFR * fr
        return dict(J=float(J), plan_J=float(self.V(s)), succ=float(succ), FR=float(fr),
                    T_given_succ=float(Tc / succ) if succ > 0 else float("nan"), n_explore=float(nE),
                    n_repeat=float(nR), n_indep=float(nI), n_partner=float(nP), states=self.n_states)

    def policy_trace(self, st=None, depth=3):
        """Greedy most-likely-path action sequence from a state (for the minimal-example write-up)."""
        return self._A.get(st or self.start())


class RuleSolver(Solver):
    """A fixed threshold rule (not optimised): report the best cell once its belief (ICC rb) >= theta, else look at the
    best cell with `views` = 'same' (repeat class 0) or 'alt' (alternate class 0,1).  Evaluated under the TRUE model."""

    def __init__(self, I, rb, theta, views="same"):
        super().__init__(I, rb)
        self.theta, self.views = theta, views

    def choose(self, st):
        I = self.I
        t, pos, w, cnt, ruled, pend = st
        if pend is not None and t >= pend[1]:
            return ("RESOLVE",)
        P = self.plan_m.posterior(cnt, ruled)
        c = int(np.argmax(P))
        if P[c] >= self.theta:
            return ("R", c)
        if pos != c:
            return ("M", c) if t + I.mv <= I.H else ("STOP",)
        v = 0
        if self.views == "alt" and I.allow_indep:
            n0 = sum(cnt[3 * c + 0]); n1 = sum(cnt[3 * c + 1])
            v = 1 if n0 > n1 else 0
        cost = 1 + (I.s if v != w else 0)
        return ("L", v, cost) if t + cost <= I.H else ("STOP",)

    def V(self, st):  # no planning value needed
        return 0.0


def solve_rule(I: XI, rb, theta, views="same"):
    S = RuleSolver(I, rb, theta, views)
    S._A = {}
    s0 = S.start()
    Tc, fr, succ = S.E(s0)[:3]
    return dict(Tc=float(Tc), FR=float(fr), succ=float(succ),
                T_given_succ=float(Tc / succ) if succ > 0 else float("nan"))


def solve(I: XI, rb=None):
    return Solver(I, rb).summary()


def first_action_after(I: XI, rb, obs):
    """Action chosen at the state reached by `obs` = list of (kind, arg...) forced prefix (looks at cell 0).
    obs items: ('L', v, y) look class v at current cell with outcome y."""
    S = Solver(I, rb)
    st = S.start()
    for (_, v, y) in obs:
        t, pos, w, cnt, ruled, pend = st
        cost = 1 + (I.s if v != w else 0)
        st = (t + cost, pos, v, _bump(cnt, pos, v, y), ruled, pend)
    S.V(st)
    return S._A[st], S.V(st)


if __name__ == "__main__":
    I = XI()
    print(solve(I), solve(I, rb=0.0))
