"""Exact best decentralised policy WITHOUT communication (failure-vector sets).

Given tau = (target node, visibility of its two views), the two drones'
outcome processes are independent when they cannot communicate, and the
report time of the team is the min of the two drones' own report times.  So

    P(no report by T) = sum_tau p(tau) f_0(tau) f_1(tau),
    f_j(tau) = P(drone j does not get its own report to base by T | tau, pi_j).

F(node, t) = set of failure vectors achievable by a deterministic single-drone
policy tree started free at ``node`` at time t, built by

    F(node, t) = U_a [ P(CONF|a) * fail_conf  +  P(NULL|a) F(node, t')
                                              (+) P(WEAK|a) F(node, t') ]

and reduced to its componentwise-Pareto-minimal subset (exact: the objective
is monotone in every coordinate and Pareto(A (+) B) = Pareto(Pareto A (+)
Pareto B)).  This mirrors ForestSearch/exact_dec.py, extended with travel,
speed and the reporting rule.  Objective: P(report by T) only.
"""
from __future__ import annotations

import numpy as np

from .model import CONF, NULL, WEAK, Instance, fmt_action


def pareto(X, tags, tol=1e-13):
    if len(X) <= 1:
        return X, list(tags)
    order = np.argsort(X.sum(1), kind="stable")
    kept, K = [], np.empty((0, X.shape[1]))
    for i in order:
        x = X[i]
        if len(K) and np.any(np.all(K <= x + tol, axis=1)):
            continue
        kept.append(i)
        K = np.vstack([K, x])
    return X[kept], [tags[i] for i in kept]


class NoComm:
    def __init__(self, inst: Instance):
        self.inst = inst
        self.Tn = len(inst.truths)
        self.memo = {}

    def F(self, node, t):
        key = (node, t)
        if key in self.memo:
            return self.memo[key]
        inst = self.inst
        ones = np.ones((1, self.Tn))
        if inst.hopeless(t):
            res = (ones, [None])
            self.memo[key] = res
            return res
        cand_v, cand_p = [ones], [None]   # "do nothing" is always available
        for a in inst.actions(node):
            t2 = t + inst.duration(node, a)
            if a[0] == "M":
                Fv, Fp = self.F(a[1], t2)
                cand_v.append(Fv)
                cand_p += [(a, {"next": p}) for p in Fp]
                continue
            li = inst.look_idx[(node, a[1], a[2])]
            fconf = 0.0 if inst.report_time(t2, node) <= inst.T else 1.0
            Fv, Fp = self.F(node, t2)
            S = inst.P[li, CONF][None, :] * fconf + inst.P[li, NULL][None, :] * Fv
            idx = [(j,) for j in range(len(Fv))]
            S, idx = pareto(S, idx)
            B = inst.P[li, WEAK][None, :] * Fv
            Sum = (S[:, None, :] + B[None, :, :]).reshape(-1, self.Tn)
            nidx = [i + (j,) for i in idx for j in range(len(Fv))]
            S, idx = pareto(Sum, nidx)
            cand_v.append(S)
            cand_p += [(a, {NULL: Fp[i[0]], WEAK: Fp[i[1]]}) for i in idx]
        V, P = pareto(np.vstack(cand_v), cand_p)
        self.memo[key] = (V, P)
        return V, P

    def solve(self):
        inst = self.inst
        F0, P0 = self.F(inst.starts[0], 0)
        F1, P1 = self.F(inst.starts[1], 0)
        M = (F0 * inst.p[None, :]) @ F1.T
        i = int(np.argmin(M))
        r, c = divmod(i, M.shape[1])
        self.best = (P0[r], P1[c])
        self.sizes = (len(F0), len(F1))
        return 1.0 - float(M[r, c])

    def policy_lines(self, j):
        inst = self.inst
        pol = self.best[j]
        lines = []

        def rec(p, node, t, ind):
            if p is None:
                lines.append(ind + f"t={t}: stop")
                return
            a, ch = p
            if a[0] == "M":
                lines.append(ind + f"t={t}: {fmt_action(a)}")
                rec(ch["next"], a[1], t + inst.duration(node, a), ind)
                return
            t2 = t + inst.duration(node, a)
            lines.append(ind + f"t={t}: {fmt_action(a, node)}  (CONF -> report)")
            same = _same(ch[NULL], ch[WEAK])
            if same:
                rec(ch[NULL], node, t2, ind)
            else:
                lines.append(ind + "  null ->")
                rec(ch[NULL], node, t2, ind + "    ")
                lines.append(ind + "  WEAK ->")
                rec(ch[WEAK], node, t2, ind + "    ")
        rec(pol, inst.starts[j], 0, "")
        return lines


def _same(p, q):
    if p is None or q is None:
        return p is q
    if p[0] != q[0]:
        return False
    if p[0][0] == "M":
        return _same(p[1]["next"], q[1]["next"])
    return _same(p[1][NULL], q[1][NULL]) and _same(p[1][WEAK], q[1][WEAK])
