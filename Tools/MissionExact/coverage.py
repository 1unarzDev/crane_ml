"""Coverage-time-optimal plan (fastest sweep) for contrast.

A sweep = every positive-prior node gets one FAST look (view 0) by some drone.
Coverage-optimal = minimum makespan (time the last node is covered).  After
the sweep the same per-drone route repeats with the views alternating
(0, 1, 0, ...), until the deadline.  A coverage plan ignores WEAK
candidates; a CONF is reported automatically (holder flies to a link node).
Among all minimum-makespan sweeps we keep the one with the best objective,
i.e. the strongest coverage baseline.
"""
from __future__ import annotations

import itertools

from .model import FAST, Instance
from .team import Team


def routes(inst: Instance):
    """All (route0, route1): ordered node lists partitioning the search nodes."""
    nodes = [i for i in range(inst.n) if inst.prior[i] > 0]
    for lab in itertools.product((0, 1), repeat=len(nodes)):
        A = [x for x, l in zip(nodes, lab) if l == 0]
        B = [x for x, l in zip(nodes, lab) if l == 1]
        for pa in itertools.permutations(A):
            for pb in itertools.permutations(B):
                yield list(pa), list(pb)


def route_time(inst, start, route):
    t, here = 0, start
    for x in route:
        t += int(inst.tt[here][x]) + inst.dur[FAST]
        here = x
    return t


def action_seq(inst, start, route, horizon):
    """Open-loop action list (moves and fast looks) long enough for horizon."""
    seq, here, rep = [], start, 0
    if not route:
        return seq
    while len(seq) < 4 * horizon:
        for x in route:
            if x != here:
                seq.append(("M", x))
                here = x
            seq.append(("L", rep % 2, FAST))
        rep += 1
        if len(route) > 1:
            route = route[::-1]
    return seq


def plan_policy(seqs):
    def pol(key):
        j, t, node, hist, rcv = key
        s = seqs[j]
        k = len(hist)
        return s[k] if k < len(s) else ("W",) if False else s[-1]
    return pol


def coverage_optimal(inst: Instance, obj: str = "p"):
    """Return (value, makespan, (route0, route1), report-time dist)."""
    cands = []
    for r0, r1 in routes(inst):
        mk = max(route_time(inst, inst.starts[0], r0), route_time(inst, inst.starts[1], r1))
        cands.append((mk, r0, r1))
    mk = min(c[0] for c in cands)
    best = None
    seen = set()
    for m, r0, r1 in cands:
        if m != mk:
            continue
        key = (tuple(r0), tuple(r1))
        if key in seen:
            continue
        seen.add(key)
        seqs = [action_seq(inst, inst.starts[0], r0, inst.T),
                action_seq(inst, inst.starts[1], r1, inst.T)]
        # a drone with an empty route idles at its start
        for j in (0, 1):
            if not seqs[j]:
                seqs[j] = [("W",)] * (4 * inst.T)
        tm = Team(inst, "none", obj, bnb=False)
        tm.actions = lambda node: inst.actions(node, comm=True)
        v = tm.solve(policy=plan_policy(seqs))
        if best is None or v > best[0] + 1e-12:
            best = (v, mk, (r0, r1), tm.rdist)
    return best
