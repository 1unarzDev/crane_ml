"""Silent (message-free) repairs for the partner-forecasting joint planner.

Problem: joint IMPLICIT (0.730) trails joint ORACLE (0.770) on held-out
5x5/travel-1/'both', while greedy IMPLICIT = greedy ORACLE.  Hypothesis: after
private events the nominal-plan forecast of the partner is wrong and the joint
rollout leans on it.

All variants use ONLY common-knowledge model parameters and the agent's own
history; no messages.  Every variant computes a ``trust`` in (0, 1] in the
nominal forecast of the partner:

  time: trust = (1 - r*r_scale) ** k   k = partner nominal looks completed by t
        (the partner has had k chances to have a private event; r = abort+alpha
        is the per-look private-event rate from the instance model)
  own : trust = decay ** n_own         n_own = my own private events (hint/abort)

If trust == 1 (always when the instance has no events, r = 0) every variant
defers to the unmodified joint planner, so Proposition 1 holds.

Modes
  disc  (i)   partner's FUTURE rollout looks are weighted by trust, in both the
              detection score and the counts they add (expected-value form).
  mix   (ii)  score = trust * V(nominal forecast) + (1-trust) * V(alt forecast),
              alt = partner idle at its last completed nominal cell, its
              in-flight look NOT counted, then greedy from MY belief.
  own   (iii) trust < tau => plan only my own path (single-agent rollout, the
              partner = in-flight slot as greedy models it); else joint.
"""
from __future__ import annotations

from dataclasses import dataclass, replace

import numpy as np

from . import joint, sim
from .joint import JointStrategy, _choose_joint, _partner_state
from .model import ABORT, HINT, detect_prob


@dataclass(frozen=True)
class SilentStrategy(JointStrategy):
    silent_mode: str = "none"      # disc | mix | own
    trust_mode: str = "time"       # time | own
    r_scale: float = 1.0
    decay: float = 0.6
    tau: float = 0.5
    floor: float = 0.0


def make_silent(K=6, B=14, **kw):
    base = joint.make_joint_strategies(K, B)
    imp = base["IMPLICIT"]
    return base, SilentStrategy(**{**imp.__dict__, "name": "IMPLICIT", **kw})


def event_rate(inst):
    return float(inst.abort + inst.alpha)


def _k_partner_done(ep, a, t):
    p = ep.agents[1 - a.idx]
    return sum(1 for tk in ep.nominal[p.idx] if tk.finish <= t)


def trust(ep, a, t):
    s = a.s
    if ep.nominal is None:
        return 1.0
    if s.trust_mode == "own":
        n = sum(1 for r in a.records if r.outcome in (HINT, ABORT))
        w = s.decay ** n
    else:
        r = min(1.0, event_rate(ep.inst) * s.r_scale)
        w = (1.0 - r) ** _k_partner_done(ep, a, t)
    return max(s.floor, w)


def _alt_partner_state(ep, a, t):
    """Partner idle now at its last completed nominal cell."""
    p = ep.agents[1 - a.idx]
    pos = ep.inst.starts[p.idx]
    best = -1
    for tk in ep.nominal[p.idx]:
        if tk.finish <= t and tk.finish >= best:
            pos, best = tk.cell, tk.finish
    return pos, t


def _roll(ep, a, t, c, v, m0, m1, pd, ppos, pfree, w):
    """Value of first look (c,v) followed by an alternating greedy rollout;
    partner looks weighted by w (w==1 reproduces joint._choose_joint)."""
    inst = ep.inst
    dist, V = inst.dist, inst.V
    B = a.s.rollout_budget
    bm = a.s.belief
    me_first_on_tie = a.idx < 1 - a.idx
    mm0 = m0.copy() if w == 1.0 else m0.astype(float)
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
        ww = 1.0 if me_turn else w
        val += p_none * ww * pdk[ci, vi]
        p_none *= 1.0 - ww * pdk[ci, vi]
        mm0[ci, vi] += ww
        nf = free + int(dist[pos, ci]) + 1
        if me_turn:
            pos_me, free_me = ci, nf
        else:
            pos_p, free_p = ci, nf
    return val


def _counts(ep, a, t, with_virtual=True):
    pm0, pm1, virtual = ep._partner_counts(a, t)
    m0 = a.m0 + pm0
    m1 = a.m1 + pm1
    if with_virtual:
        for (c, v) in virtual:
            m0[c, v] += 1
    return m0, m1


def _choose_silent(ep, a, t):
    s = a.s
    w = trust(ep, a, t)
    if w >= 1.0:
        return _choose_joint(ep, a, t)
    inst = ep.inst
    if s.silent_mode == "own":
        if w < s.tau:
            return joint._orig_choose(ep, a, t)
        return _choose_joint(ep, a, t)
    m0, m1 = _counts(ep, a, t)
    bm = s.belief
    pd = detect_prob(inst, m0, m1, bm)
    score = pd / (inst.dist[a.pos][:, None] + 1.0)
    cand = np.argsort(-score, axis=None, kind="stable")[:s.rollout_k]
    if len(cand) == 1:
        return divmod(int(cand[0]), inst.V)
    ppos, pfree, _ = _partner_state(ep, a, t)
    if s.silent_mode == "mix":
        m0a, _ = _counts(ep, a, t, with_virtual=False)
        pda = detect_prob(inst, m0a, m1, bm)
        apos, afree = _alt_partner_state(ep, a, t)
    best, best_val = int(cand[0]), -1.0
    for k in cand:
        c, v = divmod(int(k), inst.V)
        if s.silent_mode == "disc":
            val = _roll(ep, a, t, c, v, m0, m1, pd, ppos, pfree, w)
        else:
            v1 = _roll(ep, a, t, c, v, m0, m1, pd, ppos, pfree, 1.0)
            v2 = _roll(ep, a, t, c, v, m0a, m1, pda, apos, afree, 1.0)
            val = w * v1 + (1.0 - w) * v2
        if val > best_val + 1e-12:
            best, best_val = int(k), val
    return divmod(best, inst.V)


if not getattr(sim.Episode, "_silent_patched", False):
    _prev_choose = sim.Episode._choose

    def _choose(self, a, t):
        if (getattr(a.s, "silent_mode", "none") != "none" and not self.oracle
                and a.s.rollout_k > 0 and a.s.planner != "sector"):
            return _choose_silent(self, a, t)
        return _prev_choose(self, a, t)

    sim.Episode._choose = _choose
    sim.Episode._silent_patched = True


# ------------------------------------------------------------------ driver
VARIANTS = {
    "A1_disc_time":  dict(silent_mode="disc", trust_mode="time", r_scale=1.0),
    "A2_disc_time_half": dict(silent_mode="disc", trust_mode="time", r_scale=0.5),
    "A3_disc_own":   dict(silent_mode="disc", trust_mode="own", decay=0.6),
    "B1_mix_time":   dict(silent_mode="mix", trust_mode="time", r_scale=1.0),
    "B2_mix_time_half": dict(silent_mode="mix", trust_mode="time", r_scale=0.5),
    "B3_mix_own":    dict(silent_mode="mix", trust_mode="own", decay=0.6),
    "C1_own_time":   dict(silent_mode="own", trust_mode="time", r_scale=1.0, tau=0.5),
    "C2_own_own":    dict(silent_mode="own", trust_mode="own", decay=0.6, tau=0.5),
}


def _cluster_ci(per_cluster, B=4000, seed=0):
    """per_cluster: list of per-instance arrays of paired differences (equal
    length).  Resample instances, then episodes within instance."""
    rng = np.random.default_rng(seed)
    X = np.array(per_cluster)
    C, N = X.shape
    out = np.empty(B)
    for b in range(B):
        ci = rng.integers(0, C, C)
        ei = rng.integers(0, N, (C, N))
        out[b] = np.take_along_axis(X[ci], ei, axis=1).mean()
    return float(X.mean()), float(np.percentile(out, 2.5)), float(np.percentile(out, 97.5))


def run(seeds, variants, episodes=300, K=6, B=14):
    from .experiments import EVENTS, evaluate
    from .instances import grid_instance
    from .links import LinkConfig
    from .sim import STRATEGIES
    J = joint.make_joint_strategies(K, B)
    S = dict(J)
    for n, kw in variants.items():
        S[n] = make_silent(K, B, **kw)[1]
    res = {}
    for sd in seeds:
        inst = grid_instance(5, 5, seed=sd, **EVENTS["both"], travel_scale=1.0)
        r = evaluate(inst, [("ORACLE",) * 2, ("IMPLICIT",) * 2] + [(n,) * 2 for n in variants],
                     LinkConfig(), range(episodes), strategies=S)
        g = evaluate(inst, [("IMPLICIT",) * 2], LinkConfig(), range(episodes),
                     strategies=STRATEGIES)
        res[sd] = {"jORACLE": r[("ORACLE",) * 2]["det"], "jIMPLICIT": r[("IMPLICIT",) * 2]["det"],
                   "gIMPLICIT": g[("IMPLICIT",) * 2]["det"],
                   **{n: r[(n,) * 2]["det"] for n in variants}}
        print(sd, {k: round(float(v.mean()), 3) for k, v in res[sd].items()}, flush=True)
    return res


def report(res, variants):
    seeds = list(res)
    names = ["jORACLE", "jIMPLICIT", "gIMPLICIT"] + list(variants)
    lines = ["| strategy | P(detect) | -jORACLE | -jIMPLICIT | -gIMPLICIT |", "|---|---|---|---|---|"]
    for n in names:
        P = np.mean([res[s][n].mean() for s in seeds])
        cells = []
        for ref in ("jORACLE", "jIMPLICIT", "gIMPLICIT"):
            if n == ref:
                cells.append("-")
            else:
                m, lo, hi = _cluster_ci([res[s][n] - res[s][ref] for s in seeds])
                cells.append(f"{m:+.3f} [{lo:+.3f},{hi:+.3f}]")
        lines.append(f"| {n} | {P:.3f} | " + " | ".join(cells) + " |")
    return "\n".join(lines)


if __name__ == "__main__":
    import argparse
    ap = argparse.ArgumentParser()
    ap.add_argument("mode", choices=["dev", "heldout"])
    ap.add_argument("--variants", default=",".join(VARIANTS))
    ap.add_argument("--out", default=None)
    args = ap.parse_args()
    vs = {k: VARIANTS[k] for k in args.variants.split(",") if k}
    seeds = range(100, 103) if args.mode == "dev" else range(200, 208)
    res = run(seeds, vs)
    txt = report(res, vs)
    print(txt)
    if args.out:
        open(args.out, "w").write(txt + "\n")
