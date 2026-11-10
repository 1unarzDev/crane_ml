"""Exact best team policy under a general information structure.

Information modes
-----------------
* 'none'    : each drone sees only its own history (no communication).
* 'delayed' : link-gated store-and-forward messaging with delay d (see
              model.py).  A drone's information set = (own history, latest
              partner-history snapshot it has downloaded, with its timestamp).
* 'full'    : both drones see the joint history instantly (= ORACLE
              information; used only as a cross-check of the ORACLE DP).

Method: exact search over the *designer's* problem (Witsenhausen standard
form).  The state is the finite set of joint histories ("branches"), each
carrying an unnormalised weight vector over tau (prior x likelihood; given
tau all look outcomes are independent).  At every integer time the designer
assigns one action to every reachable (drone, information set); branches
sharing an information set necessarily get the same action, which is exactly
the decentralisation constraint.  The search is a depth-first
branch-and-bound over information sets with the ORACLE Q-values as an
admissible upper bound (the centralised optimum given the joint history
dominates any decentralised continuation).  Pruning only discards
assignments whose upper bound is <= the incumbent, so the result is the
exact optimum.  ``bnb=False`` disables pruning (brute force, for tests).
"""
from __future__ import annotations

import itertools

import numpy as np

from .model import CONF, NULL, WEAK, Instance, fmt_action, u_reward, ONAME
from .oracle import FREE, Oracle


class Branch:
    __slots__ = ("w", "dr", "hist", "lt")

    def __init__(self, w, dr, hist, lt):
        self.w, self.dr, self.hist, self.lt = w, dr, hist, lt
    # dr[j] = (node, end, pend, active); hist[j] = tuple of events
    # (t_end, action, node, outcome); lt[j] = tuple of integer link times


class Team:
    def __init__(self, inst: Instance, mode: str = "delayed", obj: str = "p",
                 bnb: bool = True):
        assert mode in ("none", "delayed", "full")
        self.inst, self.mode, self.obj, self.bnb = inst, mode, obj, bnb
        self.oracle = Oracle(inst, obj)
        self.K = len(inst.looks)
        self.nodes_expanded = 0
        self.restrict = None   # optional callable(key) -> allowed actions or None

    # ------------------------------------------------------------ info sets
    def recv(self, b: Branch, j: int, t: int):
        if self.mode == "none":
            return None
        k = 1 - j
        if self.mode == "full":
            return b.hist[k]
        lj = b.lt[j]
        if not lj:
            return None
        u = lj[-1]                        # latest time j was at a link node
        cand = [s for s in b.lt[k] if s <= u - self.inst.d]
        if not cand:
            return None
        S = cand[-1]
        return (S, tuple(e for e in b.hist[k] if e[0] <= S))

    def key(self, b, j, t):
        return (j, t, b.dr[j][0], b.hist[j], self.recv(b, j, t))

    def actions(self, node):
        return self.inst.actions(node, comm=(self.mode == "delayed"))

    # ---------------------------------------------------------- dynamics
    def record_presence(self, b, t):
        inst = self.inst
        lt = list(b.lt)
        for j in (0, 1):
            node, end, pend, active = b.dr[j]
            if not active:
                continue
            in_transit = pend != FREE and pend[0] == "M" and t < end
            if not in_transit and inst.link[node]:
                lt[j] = lt[j] + (t,)
        b.lt = tuple(lt)

    def advance(self, t, branches, assign):
        """Apply actions (assign: key -> action) at time t, step to t+1.
        Returns (new branches, absorbed reward mass, report-time masses)."""
        inst = self.inst
        out, gain, rdist = [], 0.0, {}
        for b in branches:
            dr = list(b.dr)
            for j in (0, 1):
                node, end, pend, active = dr[j]
                if active and pend == FREE:
                    a = assign[self.key(b, j, t)]
                    dr[j] = (node, t + inst.duration(node, a), a, True)
            # completions at t+1
            parts = [(b.w, dr, list(b.hist))]
            for j in (0, 1):
                node, end, pend, active = dr[j]
                if not active or end != t + 1:
                    continue
                newparts = []
                for (w, d2, h2) in parts:
                    if pend[0] == "L":
                        li = inst.look_idx[(node, pend[1], pend[2])]
                        for o in (NULL, WEAK, CONF):
                            w2 = w * inst.P[li, o]
                            if not w2.any():
                                continue
                            d3, h3 = list(d2), list(h2)
                            h3[j] = h3[j] + ((t + 1, pend, node, o),)
                            if o == CONF:
                                r = inst.report_time(t + 1, node)
                                if r <= inst.T:
                                    m = float(w2.sum())
                                    gain += m * u_reward(inst, r, self.obj)
                                    rdist[r] = rdist.get(r, 0.0) + m
                                    continue
                                d3[j] = (node, t + 1, FREE, False)   # useless holder
                            else:
                                d3[j] = (node, t + 1, FREE, True)
                            newparts.append((w2, d3, h3))
                    else:
                        d3, h3 = list(d2), list(h2)
                        nn = pend[1] if pend[0] == "M" else node
                        d3[j] = (nn, t + 1, FREE, True)
                        h3[j] = h3[j] + ((t + 1, pend, nn, None),)
                        newparts.append((w, d3, h3))
                parts = newparts
            for (w, d2, h2) in parts:
                if not any(x[3] for x in d2):
                    continue
                nb = Branch(w, tuple(d2), tuple(h2), b.lt)
                self.record_presence(nb, t + 1)
                out.append(nb)
        return out, gain, rdist

    # ---------------------------------------------------------- bounds
    def counts(self, b):
        c = [0] * (2 * self.K)
        for j in (0, 1):
            for (te, a, node, o) in b.hist[j]:
                if a[0] == "L" and o in (NULL, WEAK):
                    c[2 * self.inst.look_idx[(node, a[1], a[2])] + o] += 1
        return tuple(c)

    def odrones(self, b, t, ja=None):
        out = []
        for j in (0, 1):
            node, end, pend, active = b.dr[j]
            if ja is not None and pend == FREE:
                a = ja[j]
                out.append((node, t + self.inst.duration(node, a), a))
            else:
                out.append((node, end, pend))
        return tuple(out)

    def qtab(self, b, t, keys):
        """Upper-bound table: joint assignment of this branch's free drones ->
        weighted ORACLE Q (or trivial bound W if a drone is inactive)."""
        W = float(b.w.sum())
        free = [j for j in (0, 1) if b.dr[j][3] and b.dr[j][2] == FREE]
        acts = [self.actions(b.dr[j][0]) for j in free]
        tab = {}
        trivial = not all(x[3] for x in b.dr) or not self.bnb
        c = None if trivial else self.counts(b)
        for combo in itertools.product(*acts):
            if trivial:
                tab[combo] = W
                continue
            ja = [None, None]
            for j, a in zip(free, combo):
                ja[j] = a
            od = self.odrones(b, t, ja)
            if self.inst.hopeless(t):
                tab[combo] = 0.0
            else:
                tab[combo] = W * self.oracle.Q(t, od, c)
        return free, tab

    # ---------------------------------------------------------- search
    def solve(self, policy=None):
        """Exact optimum (or value of a fixed ``policy``: key -> action)."""
        inst = self.inst
        w0 = inst.p.copy()
        dr = tuple((s, 0, FREE, True) for s in inst.starts)
        b0 = Branch(w0, dr, ((), ()), ((), ()))
        self.record_presence(b0, 0)
        self.fixed = policy
        self.best_policy = {}
        val, pol, rd = self._dfs(0, [b0], 0.0, -1.0)
        self.best_policy = pol
        self.rdist = rd
        return val

    def _dfs(self, t, branches, acc, incumbent):
        """Return (best total value, policy dict, report-time dist) from here,
        or (-inf, None, None) if it cannot beat ``incumbent``."""
        self.nodes_expanded += 1
        inst = self.inst
        if not branches or inst.hopeless(t):
            return acc, {}, {}
        keys, kinfo = [], {}
        for b in branches:
            for j in (0, 1):
                if b.dr[j][3] and b.dr[j][2] == FREE:
                    k = self.key(b, j, t)
                    if k not in kinfo:
                        kinfo[k] = 0.0
                        keys.append(k)
                    kinfo[k] += float(b.w.sum())
        if not keys:
            nb, g, rd = self.advance(t, branches, {})
            v, p, rd2 = self._dfs(t + 1, nb, acc + g, incumbent)
            return v, p, _merge(rd, rd2)
        if self.fixed is not None:
            assign = {k: self.fixed(k) for k in keys}
            nb, g, rd = self.advance(t, branches, assign)
            v, p, rd2 = self._dfs(t + 1, nb, acc + g, incumbent)
            return v, {**assign, **p}, _merge(rd, rd2)
        # bound tables
        tabs = []
        for b in branches:
            free, tab = self.qtab(b, t, keys)
            ks = [self.key(b, j, t) for j in free]
            tabs.append((ks, tab))
        keys.sort(key=lambda k: -kinfo[k])
        kpos = {k: i for i, k in enumerate(keys)}
        # per-key action order: by marginal bound contribution
        order = {}
        for k in keys:
            score = {}
            for ks, tab in tabs:
                if k not in ks:
                    continue
                pos = ks.index(k)
                best = {}
                for combo, q in tab.items():
                    a = combo[pos]
                    if q > best.get(a, -1):
                        best[a] = q
                for a, q in best.items():
                    score[a] = score.get(a, 0.0) + q
            order[k] = sorted(score, key=lambda a: (-score[a], repr(a)))
            if self.restrict is not None:
                allowed = self.restrict(k)
                if allowed is not None:
                    order[k] = [a for a in order[k] if a in allowed]
        best = [incumbent, None, None]
        assign = {}

        def bound():
            tot = acc
            for ks, tab in tabs:
                m = -1.0
                for combo, q in tab.items():
                    ok = True
                    for kk, a in zip(ks, combo):
                        aa = assign.get(kk)
                        if aa is not None and aa != a:
                            ok = False
                            break
                    if ok and q > m:
                        m = q
                tot += m
            return tot

        def rec(i):
            if self.bnb and bound() <= best[0] + 1e-12:
                return
            if i == len(keys):
                nb, g, rd = self.advance(t, branches, assign)
                v, p, rd2 = self._dfs(t + 1, nb, acc + g, best[0])
                if p is not None and v > best[0] + 1e-12:
                    best[0], best[1], best[2] = v, {**assign, **p}, _merge(rd, rd2)
                return
            k = keys[i]
            for a in order[k]:
                assign[k] = a
                rec(i + 1)
            del assign[k]

        rec(0)
        if best[1] is None:
            return -np.inf, None, None
        return best[0], best[1], best[2]

    # ---------------------------------------------------------- printing
    def policy_lines(self, j, min_w=1e-9):
        """Drone j's policy: one line per reached information set."""
        inst = self.inst
        rows = []
        for k, a in sorted(self.best_policy.items(), key=lambda kv: (kv[0][1], len(kv[0][3]))):
            if k[0] != j:
                continue
            _, t, node, hist, rcv = k
            hs = " ".join(_ev(e) for e in hist if e[3] is not None) or "(nothing seen)"
            rs = ""
            if rcv is not None:
                if self.mode == "delayed":
                    S, ph = rcv
                    rs = f" | heard partner@t{S}: " + (" ".join(_ev(e) for e in ph if e[3] is not None) or "nothing")
                else:
                    rs = " | partner: " + (" ".join(_ev(e) for e in rcv if e[3] is not None) or "nothing")
            rows.append(f"t={t} @n{node} | own: {hs}{rs}  =>  {fmt_action(a, node)}")
        return rows


def _ev(e):
    t, a, node, o = e
    return f"[n{node}v{a[1]}{'F' if a[2]==0 else 'S'}:{ONAME[o]}]"


def _merge(a, b):
    if not b:
        return dict(a)
    out = dict(a)
    for k, v in b.items():
        out[k] = out.get(k, 0.0) + v
    return out
