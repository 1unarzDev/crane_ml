"""Travel-aware joint base planner (new; does not modify sim.py).

``JointStrategy`` is a ``Strategy`` with ``joint=True``.  When an agent
decides, each of its top-K greedy candidates is scored by simulating BOTH
agents forward for ``rollout_budget`` time units under all-NULL outcomes:
the partner finishes its *believed* in-flight task (as given by
``Episode._partner_counts`` and the same information source) and then both
agents continue greedily (P_detect / (travel + 1)).  The score is the joint
P(detect within budget) from the agent's own belief, sum_j P(no detect before
look j) * pd_j, over the committed look sequence (the total is a set function,
so commit order is exact).  The partner's in-flight look is a sunk constant
and is not scored.

Only information the agent has is used: counts and in-flight looks from
``_partner_counts``; the partner's believed position and finish time from the
same sources (oracle state, nominal plan, received records, leases).  No
randomness: argmax with stable tie-breaking, so Proposition 1 is preserved when
``nominal_plan`` uses the same planner (it does: this module patches
``Episode._choose`` to dispatch on ``strategy.joint``; non-joint strategies run
the original code unchanged).
"""
from __future__ import annotations

from dataclasses import dataclass

import numpy as np

from . import sim
from .model import detect_prob


@dataclass(frozen=True)
class JointStrategy(sim.Strategy):
    joint: bool = True


def make_joint_strategies(K: int = 6, B: int = 8) -> dict:
    """The 7 named strategies of ``sim.make_strategies`` with the joint planner."""
    base = dict(
        ORACLE=dict(planner="oracle"),
        INDEP=dict(),
        SECTOR=dict(planner="sector"),
        IMPLICIT=dict(predict_nominal=True),
        RESERVE=dict(send="reserve"),
        FULL=dict(send="full"),
        EVENT=dict(predict_nominal=True, send="event"),
    )
    return {k: JointStrategy(k, **v, rollout_k=K, rollout_budget=B) for k, v in base.items()}


def _partner_state(ep, a, t):
    """(pos, free_at, cell_inflight) of the partner as agent ``a`` believes it.
    Mirrors the information sources of ``Episode._partner_counts``."""
    inst = ep.inst
    p = ep.agents[1 - a.idx]
    if ep.oracle:
        if p.task is not None:
            return p.task.cell, max(p.task.finish, t), True
        return p.pos, t, False
    done = []       # (finish, cell)
    flight = []     # (finish, cell)
    seen = set()
    if a.s.predict_nominal and ep.nominal is not None:
        for tk in ep.nominal[p.idx]:
            if a.p_explicit and tk.finish >= a.p_tdev:
                break
            if tk.finish <= t:
                done.append((tk.finish, tk.cell)); seen.add(tk.seq)
            elif (tk.start, p.idx) < (t, a.idx):
                flight.append((tk.finish, tk.cell)); seen.add(tk.seq)
            else:
                break
    for seq, r in a.p_records.items():
        if seq in seen:
            continue
        seen.add(seq)
        done.append((r.finish, r.cell))
    for seq, (c, v, exp) in a.p_leases.items():
        if seq in seen:
            continue
        fin = exp - a.s.lease_slack
        if t <= exp:
            flight.append((max(fin, t), c))
        else:
            done.append((fin, c))
    if flight:
        fin, c = max(flight)
        return c, max(fin, t), True
    pos = max(done)[1] if done else inst.starts[p.idx]
    return pos, t, False


def _choose_joint(ep, a, t):
    inst = ep.inst
    pm0, pm1, virtual = ep._partner_counts(a, t)
    m0 = a.m0 + pm0
    m1 = a.m1 + pm1
    for (c, v) in virtual:
        m0[c, v] += 1
    bm = a.s.belief
    pd = detect_prob(inst, m0, m1, bm)
    score = pd / (inst.dist[a.pos][:, None] + 1.0)
    K = a.s.rollout_k
    cand = np.argsort(-score, axis=None, kind="stable")[:K]
    if len(cand) == 1:
        return divmod(int(cand[0]), inst.V)
    B = a.s.rollout_budget
    ppos, pfree, _ = _partner_state(ep, a, t)
    me_first_on_tie = a.idx < 1 - a.idx
    dist, V = inst.dist, inst.V
    best, best_val = int(cand[0]), -1.0
    for k in cand:
        c, v = divmod(int(k), V)
        mm0 = m0.copy()
        val = pd[c, v]
        p_none = 1.0 - val
        mm0[c, v] += 1
        pos_me, free_me = c, t + int(dist[a.pos, c]) + 1
        pos_p, free_p = ppos, pfree
        while True:
            me_turn = free_me < free_p or (free_me == free_p and me_first_on_tie)
            free = free_me if me_turn else free_p
            if free >= t + B:
                break
            pos = pos_me if me_turn else pos_p
            pdk = detect_prob(inst, mm0, m1, bm)
            ci, vi = divmod(int(np.argmax(pdk / (dist[pos][:, None] + 1.0))), V)
            val += p_none * pdk[ci, vi]
            p_none *= 1.0 - pdk[ci, vi]
            mm0[ci, vi] += 1
            nf = free + int(dist[pos, ci]) + 1
            if me_turn:
                pos_me, free_me = ci, nf
            else:
                pos_p, free_p = ci, nf
        if val > best_val + 1e-12:
            best, best_val = int(k), val
    return divmod(best, V)


if not getattr(sim.Episode, "_joint_patched", False):
    _orig_choose = sim.Episode._choose

    def _choose(self, a, t):
        s = a.s
        if getattr(s, "joint", False) and s.rollout_k > 0 and s.planner != "sector":
            return _choose_joint(self, a, t)
        return _orig_choose(self, a, t)

    sim.Episode._choose = _choose
    sim.Episode._joint_patched = True
