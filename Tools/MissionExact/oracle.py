"""ORACLE: exact centralised optimum with full, instant information sharing
between the drones.  The base link and its delay d still apply and the
confirming drone itself must carry its evidence to a link node (same report
rule as every other policy), so ORACLE upper-bounds all decentralised
policies.

Exact finite-horizon DP over the joint belief.  The belief is a function only
of per-look-type (node, view, speed) counts of NULL and WEAK outcomes (CONF is
terminal), so the DP is memoised on (t, drone states, counts).  Time is
event-skipped to the next action completion.
"""
from __future__ import annotations

import itertools
from functools import lru_cache

import numpy as np

from .model import CONF, NULL, WEAK, Instance, fmt_action, u_reward, ONAME

FREE = ()


class Oracle:
    def __init__(self, inst: Instance, obj: str = "p"):
        self.inst, self.obj = inst, obj
        self.K = len(inst.looks)
        self.logP = np.log(np.maximum(inst.P[:, :2, :], 1e-300))
        self._V = {}
        self._b = {}

    # counts: tuple length 2K, (nNULL, nWEAK) per look type
    def weights(self, counts):
        w = self._b.get(counts)
        if w is None:
            c = np.asarray(counts, float).reshape(self.K, 2)
            lw = np.einsum("ko,kot->t", c, self.logP)
            w = self.inst.p * np.exp(lw)
            self._b[counts] = w
        return w

    def belief(self, counts):
        w = self.weights(counts)
        s = w.sum()
        return w / s if s > 0 else w

    @staticmethod
    def canon(drones):
        return tuple(sorted(drones))

    def V(self, t, drones, counts):
        """Optimal normalised value; drones = tuple of (node, end, pend)."""
        key = (t, self.canon(drones), counts)
        v = self._V.get(key)
        if v is not None:
            return v[0]
        best, barg = self._solve(t, drones, counts)
        self._V[key] = (best, barg)
        return best

    def joint_actions(self, t, drones):
        inst = self.inst
        opts = []
        if len(drones) == 2 and drones[0] == drones[1] and drones[0][2] == FREE:
            node = drones[0][0]
            acts = [(node, t + inst.duration(node, a), a) for a in inst.actions(node)]
            return list(itertools.combinations_with_replacement(acts, 2))
        for (node, end, pend) in drones:
            if pend == FREE:
                opts.append([(node, t + inst.duration(node, a), a) for a in inst.actions(node)])
            else:
                opts.append([(node, end, pend)])
        return list(itertools.product(*opts))

    def _solve(self, t, drones, counts):
        inst = self.inst
        if inst.hopeless(t):
            return 0.0, None
        best, barg = -1.0, None
        for ja in self.joint_actions(t, drones):
            q = self.Q(t, ja, counts)
            if q > best + 1e-15:
                best, barg = q, ja
        return best, barg

    def transitions(self, t, ja, counts):
        """Yield (prob, kind, payload): kind 'succ' payload=reward,
        kind 'cont' payload=(t2, drones, counts)."""
        inst = self.inst
        t2 = min(e for (_, e, _) in ja)
        b = self.belief(counts)
        comp = [k for k, (_, e, _) in enumerate(ja) if e == t2]
        looks = [k for k in comp if ja[k][2][0] == "L"]
        base = []
        for k, (node, e, pend) in enumerate(ja):
            if e == t2:
                nn = pend[1] if pend[0] == "M" else node
                base.append((nn, t2, FREE))
            else:
                base.append((node, e, pend))
        if not looks:
            yield 1.0, "cont", (t2, tuple(base), counts)
            return
        lidx = [inst.look_idx[(ja[k][0], ja[k][2][1], ja[k][2][2])] for k in looks]
        for outs in itertools.product((NULL, WEAK, CONF), repeat=len(looks)):
            pr = b.copy()
            for li, o in zip(lidx, outs):
                pr = pr * inst.P[li, o]
            p = float(pr.sum())
            if p <= 0:
                continue
            if CONF in outs:
                r = min(inst.report_time(t2, ja[k][0]) for k, o in zip(looks, outs)
                        if o == CONF)
                yield p, "succ", u_reward(inst, r, self.obj)
            else:
                c = list(counts)
                for li, o in zip(lidx, outs):
                    c[2 * li + o] += 1
                yield p, "cont", (t2, tuple(base), tuple(c))

    def Q(self, t, ja, counts):
        tot = 0.0
        for p, kind, pay in self.transitions(t, ja, counts):
            tot += p * (pay if kind == "succ" else self.V(*pay))
        return tot

    # ----------------------------------------------------------------- API
    def root(self):
        inst = self.inst
        drones = tuple((s, 0, FREE) for s in inst.starts)
        return 0, drones, tuple([0] * (2 * self.K))

    def value(self):
        return self.V(*self.root())

    def policy_lines(self, max_depth=12, min_prob=1e-3):
        """Readable joint policy tree from the root (drone order preserved)."""
        inst = self.inst
        lines = []

        def rec(t, drones, counts, prob, depth, ind):
            if depth > max_depth:
                lines.append(ind + "...")
                return
            self.V(t, drones, counts)
            _, ja = self._V[(t, self.canon(drones), counts)]
            if ja is None:
                lines.append(ind + f"t={t}: (no useful action before deadline)")
                return
            # map canonical argmax back to this drone order
            ja = self._match(drones, ja)
            acts = []
            for k, ((node, e, pend), (n2, e2, p2)) in enumerate(zip(drones, ja)):
                if pend == FREE:
                    acts.append(f"D{k}@n{node}: {fmt_action(p2, node)}")
            lines.append(ind + f"t={t}: " + "; ".join(acts) +
                         f"   [value {self.V(t, drones, counts):.4f}]")
            for p, kind, pay in self.transitions(t, ja, counts):
                if p * prob < min_prob:
                    continue
                desc = self._outdesc(t, ja, counts, pay, kind)
                if kind == "succ":
                    lines.append(ind + f"  {desc} (p={p:.3f}) -> CONFIRMED, report value {pay:g}")
                else:
                    t2, d2, c2 = pay
                    if desc:
                        lines.append(ind + f"  {desc} (p={p:.3f}):")
                        rec(t2, d2, c2, prob * p, depth + 1, ind + "    ")
                    else:
                        rec(t2, d2, c2, prob * p, depth + 1, ind)
        rec(*self.root(), 1.0, 0, "")
        return lines

    def _match(self, drones, ja_canon):
        """ja_canon is in canonical drone order; reorder to ``drones``."""
        order = sorted(range(len(drones)), key=lambda k: drones[k])
        out = [None] * len(drones)
        for pos, k in enumerate(order):
            out[k] = ja_canon[pos]
        return tuple(out)

    def _outdesc(self, t, ja, counts, pay, kind):
        # recompute outcome labels for completing looks
        t2 = min(e for (_, e, _) in ja)
        looks = [k for k, (_, e, p) in enumerate(ja) if e == t2 and p[0] == "L"]
        if not looks:
            return ""
        if kind == "succ":
            return "some look CONF"
        c2 = pay[2]
        parts = []
        for k in looks:
            node, e, pend = ja[k]
            li = self.inst.look_idx[(node, pend[1], pend[2])]
            dn = c2[2 * li] - counts[2 * li]
            dw = c2[2 * li + 1] - counts[2 * li + 1]
            # two drones on the same look type: report the multiset
            parts.append((k, li, dn, dw))
        seen, labs = set(), []
        for k, li, dn, dw in parts:
            if li in seen:
                continue
            seen.add(li)
            labs.append(f"look n{ja[k][0]}/v{ja[k][2][1]}: " +
                        "+".join(["null"] * dn + ["WEAK"] * dw))
        return "; ".join(labs)
