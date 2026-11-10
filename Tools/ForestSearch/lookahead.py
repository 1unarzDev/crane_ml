"""Non-greedy base planner: one-agent-at-a-time rollout with belief-sampled futures.

New module; does not modify sim.py, joint.py or variants.py.

``LookaheadStrategy`` is a ``Strategy`` with ``lookahead=True``.  When an idle
agent decides at time t it considers its top-``la_k`` greedy candidates
(0 = all n*V looks).  Each candidate is scored by ``la_n`` simulated futures
of BOTH agents (the deciding agent's first look is the candidate; afterwards
both follow the centralised greedy rule P_detect/(travel+1), partner in-flight
looks counted as virtual NULLs).  This is Bertsekas' multiagent rollout
("one agent at a time"), whose base policy is the paper's greedy planner, so
with exact Q-values it is guaranteed to do no worse than greedy.

Unlike ``joint.py`` (all-NULL futures), the futures here sample HINT, NULL and
ABORT outcomes from the agent's current Bayesian predictive distribution.
DETECT is handled analytically (survival weighting): the score is

    E[ sum_j S_j * pdet_j ],  S_j = prod_{i<j} (1 - pdet_i),

with the non-detect outcomes of each look sampled from P(o | counts, no
detect).  This is an unbiased estimate of P(detect by the window end) under the
rollout policy and has much lower variance than a 0/1 indicator.  Common random
numbers are shared across candidates.  The window is ``la_window`` time units
(0 = to the horizon).

Information: the planner uses only what the agent has, exactly as
``Episode._partner_counts`` provides (oracle: everything; IMPLICIT: the nominal
plan; INDEP: nothing, single-agent rollout).  Its internal model treats the
future as centralised, as the joint planner does.

Determinism: the RNG is seeded from a hash of the decision inputs (time, agent,
counts, positions, believed partner state, configuration).  Identical inputs
give identical decisions, so ``nominal_plan`` with this planner is reproducible
offline and Proposition 1 (silent replay) holds when there are no private
events.

Also provides ``rollout_decide_sync`` / ``exact_value_of_planner`` for checking
the planner against the exact DP of ``exact.ExactTeam`` (travel-free,
synchronous, abort-free).
"""
from __future__ import annotations

import hashlib
from dataclasses import dataclass

import numpy as np

from . import sim
from .model import BeliefModel, Instance


@dataclass(frozen=True)
class LookaheadStrategy(sim.Strategy):
    lookahead: bool = True
    la_k: int = 8          # candidate first looks (top-k by greedy score); 0 = all
    la_n: int = 32         # sampled futures per candidate (common random numbers)
    la_window: int = 0     # rollout window in time units; 0 = to the horizon
    la_seed: int = 0
    la_pairs: int = 0      # if the partner is idle now too: its candidate first looks (joint actions)
    la_z: float = 0.0      # accept a non-greedy action only if paired gain > la_z * SE


def make_lookahead_strategies(k: int = 8, n: int = 32, window: int = 0, seed: int = 0,
                              pairs: int = 0, z: float = 0.0) -> dict:
    base = dict(
        ORACLE=dict(planner="oracle"),
        INDEP=dict(),
        IMPLICIT=dict(predict_nominal=True),
    )
    return {name: LookaheadStrategy(name, **v, la_k=k, la_n=n, la_window=window, la_seed=seed,
                                    la_pairs=pairs, la_z=z)
            for name, v in base.items()}


# ------------------------------------------------------------ belief (batched)
class _Belief:
    """Vectorised posterior / predictive outcome probabilities for the true
    (correlated, hint-using) belief model.  Counts have shape (B, n, V)."""

    def __init__(self, inst: Instance):
        self.inst = inst
        g, q = inst.g, inst.q
        h1, hb, a = inst.h_good, inst.h_bad, inst.alpha
        self.g, self.q, self.h1, self.hb, self.a = g, q, h1, hb, a
        self.gn = (1 - q) * (1 - h1)     # good view, NULL
        self.gh = (1 - q) * h1           # good view, HINT
        self.la0 = np.log(1 - a)
        self.la1 = np.log(max(a, 1e-300))
        self.logprior = np.log(inst.prior)

    def probs(self, m0, m1):
        """Return (pdet, phint) per (B, n, V) for one more look."""
        with np.errstate(divide="ignore", invalid="ignore"):
            good = self.g * self.gn ** m0 * self.gh ** m1
            bad = (1 - self.g) * (1 - self.hb) ** m0 * self.hb ** m1
            tot = good + bad
            present = np.log(tot)
            absent = m0 * self.la0 + m1 * self.la1
            lp = self.logprior + (present - absent).sum(-1)
            lp -= lp.max(-1, keepdims=True)
            w = np.exp(lp)
            post = (w / w.sum(-1, keepdims=True))[..., None]
            pg = np.where(tot > 0, good / tot, 0.0)
        pdet = post * pg * self.q
        phint = post * (pg * self.gh + (1 - pg) * self.hb) + (1 - post) * self.a
        return pdet, phint


# ------------------------------------------------------------ core planner
def _seed_from(*parts) -> int:
    h = hashlib.blake2b(digest_size=8)
    for p in parts:
        h.update(np.ascontiguousarray(p).tobytes() if isinstance(p, np.ndarray) else repr(p).encode())
    return int.from_bytes(h.digest(), "little")


def rollout_decide(inst: Instance, m0, m1, me_idx: int, me_pos: int, t: int,
                   partner, k: int = 8, n_samples: int = 32, window: int = 0,
                   seed: int = 0, pairs: int = 0, z: float = 0.0,
                   return_values: bool = False):
    """Choose (cell, view) for agent ``me_idx`` idle at time ``t``.

    m0, m1   : (n, V) belief counts (own + partner completed looks; NOT the
               partner's in-flight look).
    partner  : None (ignore partner) or (pos, free_at, inflight) where inflight
               is (cell, view) of a look completing at ``free_at`` (pos is
               then that cell), or None if the partner is idle at ``t``.
    pairs    : if the partner is idle at ``t`` (it decides right after me),
               also roll out joint first actions (mine x its top-``pairs``);
               my action is scored by its best joint continuation.
    z        : the greedy action (with greedy partner response) is kept unless
               the best alternative's paired mean gain exceeds z standard errors.
    """
    n, V = inst.n, inst.V
    H = inst.horizon
    end = H if window <= 0 else min(H, t + window)
    bel = _Belief(inst)
    dist = inst.dist
    m0 = np.asarray(m0, float)
    m1 = np.asarray(m1, float)
    # Greedy score for candidate selection (partner in-flight as virtual NULL).
    v0 = m0.copy()
    if partner is not None and partner[2] is not None:
        v0[partner[2]] += 1
    pd0, _ = bel.probs(v0[None], m1[None])
    score = pd0[0] / (dist[me_pos][:, None] + 1.0)
    order = np.argsort(-score, axis=None, kind="stable")
    cand = order if k <= 0 else order[:k]
    if len(cand) == 1 or t >= H:
        c, v = divmod(int(cand[0]), V)
        return ((c, v), None) if return_values else (c, v)

    # Candidate list: (my flat look, partner flat look or -1 = greedy response, group).
    has_p = partner is not None
    pc_list = []
    if has_p and partner[2] is None and pairs > 0:
        sp = pd0[0] / (dist[partner[0]][:, None] + 1.0)
        pc_list = list(np.argsort(-sp, axis=None, kind="stable")[:pairs])
    C = [(int(c), -1, gi) for gi, c in enumerate(cand)]
    C += [(int(c), int(pc), gi) for gi, c in enumerate(cand) for pc in pc_list]
    my_flat = np.array([x[0] for x in C]); p_flat = np.array([x[1] for x in C])
    group = np.array([x[2] for x in C])
    K, N = len(C), n_samples
    B = K * N
    rng = np.random.default_rng(_seed_from(seed, k, n_samples, window, pairs, t, me_idx, me_pos,
                                           repr(partner), m0, m1))
    slots = end - t + 3
    U = rng.random((N, 2, slots, 2))           # per (sample, agent, look slot): abort, hint
    jidx = np.tile(np.arange(N), K)            # sample index of each rollout
    rows = np.arange(B)

    # Incremental belief state per rollout: good/bad visibility likelihood
    # factors per (cell, view) and per-cell log likelihood ratio.
    g, q, h1, hb = inst.g, inst.q, inst.h_good, inst.h_bad
    gn, gh = bel.gn, bel.gh
    G0 = g * gn ** m0 * gh ** m1
    B0 = (1 - g) * (1 - hb) ** m0 * hb ** m1
    A0 = (m0 * bel.la0 + m1 * bel.la1).sum(-1)
    with np.errstate(divide="ignore"):
        LR0 = np.log(G0 + B0).sum(-1) - A0
    GOOD = np.broadcast_to(G0, (B, n, V)).copy()
    BAD = np.broadcast_to(B0, (B, n, V)).copy()
    ABS = np.broadcast_to(A0, (B, n)).copy()
    LR = np.broadcast_to(LR0, (B, n)).copy()
    logprior = bel.logprior

    def post_of(lr):
        x = logprior + lr
        x = x - x.max(-1, keepdims=True)
        w = np.exp(x)
        return w / w.sum(-1, keepdims=True)

    def pgood(Gs, Bs):
        tot = Gs + Bs
        with np.errstate(divide="ignore", invalid="ignore"):
            return np.where(tot > 0, Gs / tot, 0.0)

    S = np.ones(B)
    val = np.zeros(B)
    big = 1 << 30
    # agent slots: 0 = me, 1 = partner (decision order follows true indices)
    pos = np.zeros((2, B), int)
    free = np.full((2, B), big, int)
    tc = np.zeros((2, B), int)
    tv = np.zeros((2, B), int)
    nlook = np.zeros((2, B), int)
    cc, vv = np.divmod(my_flat, V)
    tc[0] = np.repeat(cc, N)
    tv[0] = np.repeat(vv, N)
    free[0] = t + dist[me_pos, tc[0]] + 1
    pos[0] = tc[0]
    partner_choose_now = False
    if has_p:
        ppos, pfree, pin = partner
        if pin is not None:
            tc[1], tv[1] = pin
            free[1] = max(int(pfree), t + 1)
            pos[1] = pin[0]
        else:
            pos[1] = ppos
            partner_choose_now = True
    order_agents = (0, 1) if me_idx == 0 else (1, 0)   # true index order

    def choose(ag, sel, tau):
        other = 1 - ag
        lr = LR[sel].copy()
        Gs, Bs = GOOD[sel], BAD[sel]
        pg = pgood(Gs, Bs)
        if has_p:
            r = np.nonzero(free[other, sel] > tau)[0]
            if len(r):
                oc, ov = tc[other, sel[r]], tv[other, sel[r]]
                go, bo = Gs[r, oc, ov], Bs[r, oc, ov]
                gnw, bnw = go * gn[oc, ov], bo * (1 - hb)
                with np.errstate(divide="ignore", invalid="ignore"):
                    lr[r, oc] += np.log(gnw + bnw) - np.log(go + bo) - bel.la0
                    pg[r, oc, ov] = np.where(gnw + bnw > 0, gnw / (gnw + bnw), 0.0)
        pd = post_of(lr)[:, :, None] * pg * q
        sc = pd / (dist[pos[ag, sel]][:, :, None] + 1.0)
        a = np.argmax(sc.reshape(len(sel), -1), axis=1)
        c, v = np.divmod(a, V)
        tc[ag, sel] = c
        tv[ag, sel] = v
        free[ag, sel] = tau + dist[pos[ag, sel], c] + 1
        pos[ag, sel] = c

    if partner_choose_now and t < end:
        pf = np.repeat(p_flat, N)
        fixed = np.nonzero(pf >= 0)[0]
        if len(fixed):
            c1, v1 = np.divmod(pf[fixed], V)
            tc[1, fixed], tv[1, fixed] = c1, v1
            free[1, fixed] = t + dist[ppos, c1] + 1
            pos[1, fixed] = c1
        resp = np.nonzero(pf < 0)[0]
        if len(resp):
            choose(1, resp, t)   # partner's greedy response, decided after me at t

    tau = int(min(free[0].min(), free[1].min()))
    ab = inst.abort
    while tau <= end:
        # Rao-Blackwellisation of simultaneous completions: where both looks
        # finish at tau, credit S * P(det_first or det_second | pre-tau belief)
        # (exact) instead of the sampled sequential terms.  The survival weight
        # S is still updated with the realised factors, which keeps the
        # estimator unbiased.
        both = np.nonzero((free[0] == tau) & (free[1] == tau))[0] if has_p else np.zeros(0, int)
        if len(both):
            po = post_of(LR[both])
            ii = np.arange(len(both))
            c0, v0, c1, v1 = tc[0, both], tv[0, both], tc[1, both], tv[1, both]
            g0, b0 = GOOD[both, c0, v0], BAD[both, c0, v0]
            g1, b1 = GOOD[both, c1, v1], BAD[both, c1, v1]
            pg0 = np.where(g0 + b0 > 0, g0 / np.maximum(g0 + b0, 1e-300), 0.0)
            pg1 = np.where(g1 + b1 > 0, g1 / np.maximum(g1 + b1, 1e-300), 0.0)
            e0, e1 = (1 - ab) * q[c0, v0], (1 - ab) * q[c1, v1]
            pj = np.where(c0 != c1, po[ii, c0] * pg0 * e0 + po[ii, c1] * pg1 * e1,
                          np.where(v0 != v1, po[ii, c0] * (1 - (1 - pg0 * e0) * (1 - pg1 * e1)),
                                   po[ii, c0] * pg0 * (1 - (1 - e0) * (1 - e1))))
            val[both] += S[both] * pj
        rb = np.zeros(B, bool)
        rb[both] = True
        for ag in order_agents:
            if ag == 1 and not has_p:
                continue
            sel = np.nonzero(free[ag] == tau)[0]
            if len(sel) == 0:
                continue
            c, v = tc[ag, sel], tv[ag, sel]
            pc = post_of(LR[sel])[np.arange(len(sel)), c]
            gs, bs = GOOD[sel, c, v], BAD[sel, c, v]
            pgl = np.where(gs + bs > 0, gs / np.maximum(gs + bs, 1e-300), 0.0)
            qq = q[c, v]
            pd_raw = pc * pgl * qq
            phl = pc * (pgl * gh[c, v] + (1 - pgl) * hb) + (1 - pc) * inst.alpha
            pdl = pd_raw * (1 - inst.abort)
            val[sel] += np.where(rb[sel], 0.0, S[sel] * pdl)
            S[sel] *= 1 - pdl
            sl = np.minimum(nlook[ag, sel], slots - 1)
            u = U[jidx[sel], ag, sl]
            nlook[ag, sel] += 1
            aborted = u[:, 0] < inst.abort
            p_h = np.where(pd_raw < 1, phl / np.maximum(1 - pd_raw, 1e-300), 0.0)
            hint = (~aborted) & (u[:, 1] < p_h)
            null = (~aborted) & ~hint
            for msk, fg, fb, la in ((null, gn, 1 - hb, bel.la0), (hint, gh, hb, bel.la1)):
                if not msk.any():
                    continue
                r, cr, vr = sel[msk], c[msk], v[msk]
                GOOD[r, cr, vr] *= fg[cr, vr]
                BAD[r, cr, vr] *= fb
                ABS[r, cr] += la
                with np.errstate(divide="ignore"):
                    LR[r, cr] = np.log(GOOD[r, cr] + BAD[r, cr]).sum(-1) - ABS[r, cr]
        if tau >= end:
            break
        for ag in order_agents:
            if ag == 1 and not has_p:
                continue
            sel = np.nonzero(free[ag] == tau)[0]
            if len(sel):
                choose(ag, sel, tau)
        nxt = free[0].min()
        if has_p:
            nxt = min(nxt, free[1].min())
        tau = int(nxt)
    vals = val.reshape(K, N)
    Q = vals.mean(1)
    # First maximum in candidate order (greedy first): later ones must strictly improve.
    best = 0
    for i in range(1, K):
        if Q[i] > Q[best] + 1e-12:
            best = i
    if best != 0 and z > 0:
        d = vals[best] - vals[0]
        if d.mean() <= z * d.std(ddof=1) / np.sqrt(N):
            best = 0
    c, v = divmod(int(my_flat[best]), V)
    if return_values:
        Qg = np.array([Q[group == gi].max() for gi in range(len(cand))])
        return (c, v), (cand, Qg)
    return (c, v)


# ------------------------------------------------------------ sim adapter
def _partner_belief(ep, a, t):
    """(pos, free_at, inflight(cell, view) | None) of the partner as agent
    ``a`` believes it; mirrors Episode._partner_counts / joint._partner_state."""
    inst = ep.inst
    p = ep.agents[1 - a.idx]
    if ep.oracle:
        if p.task is not None:
            return (p.task.cell, int(p.task.finish), (p.task.cell, p.task.view))
        return (int(p.pos), t, None)
    s = a.s
    if s.send == "none" and not s.predict_nominal:
        return None                                  # INDEP ignores the partner
    done, flight, seen = [], [], set()
    if s.predict_nominal and ep.nominal is not None:
        for tk in ep.nominal[p.idx]:
            if a.p_explicit and tk.finish >= a.p_tdev:
                break
            if tk.finish <= t:
                done.append((tk.finish, tk.cell)); seen.add(tk.seq)
            elif (tk.start, p.idx) < (t, a.idx):
                flight.append((tk.finish, tk.cell, tk.view)); seen.add(tk.seq)
            else:
                break
    for seq, r in a.p_records.items():
        if seq not in seen:
            seen.add(seq); done.append((r.finish, r.cell))
    for seq, (c, v, exp) in a.p_leases.items():
        if seq in seen:
            continue
        fin = exp - s.lease_slack
        if t <= exp:
            flight.append((max(fin, t + 1), c, v))
        else:
            done.append((fin, c))
    if flight:
        fin, c, v = max(flight)
        return (c, int(fin), (c, v))
    pos = max(done)[1] if done else inst.starts[p.idx]
    return (int(pos), t, None)


def _choose_lookahead(ep, a, t):
    inst = ep.inst
    s = a.s
    pm0, pm1, virtual = ep._partner_counts(a, t)
    m0 = a.m0 + pm0
    m1 = a.m1 + pm1
    partner = _partner_belief(ep, a, t)
    # Virtual looks other than the simulated partner in-flight look stay as
    # virtual NULLs (only arises with several unexpired leases).
    pin = partner[2] if partner is not None else None
    extra = list(virtual)
    if pin is not None and pin in extra:
        extra.remove(pin)
    for (c, v) in extra:
        m0[c, v] += 1
    return rollout_decide(inst, m0, m1, a.idx, int(a.pos), t, partner,
                          k=s.la_k, n_samples=s.la_n, window=s.la_window, seed=s.la_seed,
                          pairs=s.la_pairs, z=s.la_z)


if not getattr(sim.Episode, "_lookahead_patched", False):
    _prev_choose = sim.Episode._choose     # may already be joint.py's dispatcher

    def _choose(self, a, t):
        s = a.s
        if getattr(s, "lookahead", False) and s.planner != "sector":
            return _choose_lookahead(self, a, t)
        return _prev_choose(self, a, t)

    sim.Episode._choose = _choose
    sim.Episode._lookahead_patched = True


# ------------------------------------------------------------ exact check
def rollout_decide_sync(inst: Instance, key, n_types_V, **kw):
    """Joint action of the travel-free synchronous team at counts ``key``
    (ExactTeam layout) with ``kw['t']`` and horizon from inst: agent 0 decides
    (partner idle, decides next), then agent 1 with agent 0's look in flight."""
    n, V = inst.n, n_types_V
    m = np.array(key, int).reshape(2, n, V)
    t = kw.pop("t")
    a0 = rollout_decide(inst, m[0], m[1], 0, 0, t, (0, t, None), **kw)
    a1 = rollout_decide(inst, m[0], m[1], 1, 0, t, (a0[0], t + 1, a0), **kw)
    return a0, a1


def exact_value_of_planner(E, steps: int, **kw) -> float:
    """Exact P(detect within ``steps`` steps) of the rollout planner on ExactTeam's
    travel-free synchronous model (the planner's own sampling is part of the
    deterministic policy; outcome trees are enumerated exactly)."""
    from dataclasses import replace
    k = steps
    inst = replace(E.inst, horizon=k, dist=np.zeros((E.n, E.n), int), abort=0.0)
    cache = {}

    def ev(kk, key):
        if kk == 0:
            return 0.0
        if (kk, key) in cache:
            return cache[(kk, key)]
        a0, a1 = rollout_decide_sync(inst, key, E.V, t=k - kk, **kw)
        act = tuple(sorted((E.types.index(a0), E.types.index(a1))))
        v = E._q(kk, key, act, ev)
        cache[(kk, key)] = v
        return v
    return ev(k, tuple([0] * (2 * E.n * E.V)))
