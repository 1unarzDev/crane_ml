"""Extra theory checks for the [workshop] paper (response to theory review A).

Run from Tools/:  python -m ForestSearch.theory_extra <task> [...]

Tasks
  landscape   exact VoC(h_bad, alpha) at n=2, T=2 (and a coarse T=3 grid)
  beacon      Prop. 3 (health beacon): V_cent with sensor death vs the optimal
              decentralised team with one death frame (ideal / lossy link,
              with and without hints)
  bound       VoC <= V_cent - V_open-loop (and the tighter V_cent - V_pbp) for
              k = 2..6
  pbp         person-by-person optimality of the IMPLICIT trees
  all         everything (writes results/theory_extra_raw.json)

Everything is exact enumeration / DP on the travel-free synchronous model of
exact_dec.py.  Notation as there: a truth tau = (target cell, visibility bits
of its V views); given tau the looks are independent.
"""
from __future__ import annotations

import itertools
import json
import sys
import time
from dataclasses import replace
from functools import lru_cache
from pathlib import Path

import numpy as np

from .exact_dec import (HINTS, Policy, TinyModel, base_instance, pareto,
                        random_instance)
from .model import ABORT, HINT, NULL, Instance

DEAD = 9  # outcome code: sensor died on this look (private, permanent)
HERE = Path(__file__).resolve().parent

# The reviewer's adversarial instance (REVIEW_A_theory.md, spot checks).
REVIEWER = dict(prior=[0.37, 0.63], g=[[0.93, 0.05], [0.95, 0.11]],
                q=[[0.5, 0.87], [0.37, 0.49]], h_good=0.0, h_bad=1.0, alpha=0.0)


def reviewer_instance(**kw) -> Instance:
    d = dict(REVIEWER)
    d.update(kw)
    return Instance(**d, name="reviewer")


def base2(h_good=0.3, h_bad=0.15, alpha=0.03, abort=0.0) -> Instance:
    """The n=2 tiny base instance of exact_dec with the paper's event rates."""
    return Instance(prior=[0.6, 0.4], g=[[0.7, 0.4], [0.5, 0.6]], q=np.full((2, 2), 0.8),
                    h_good=h_good, h_bad=h_bad, alpha=alpha, abort=abort, name="base2")


def paper_rates(lam: float) -> dict:
    """Paper Sec. 2: rho=.10, h_g=.30, h_b=.15, alpha=.03, all scaled by lambda."""
    return dict(h_good=min(1, 0.3 * lam), h_bad=min(1, 0.15 * lam),
                alpha=min(1, 0.03 * lam), abort=min(1, 0.10 * lam))


# =================================================================== V_cent
def v_cent_fast(M: TinyModel, k: int, death: float = 0.0, return_policy=False):
    """Exact centralised optimum.  The joint belief depends only on the
    per-action counts of NULL and HINT outcomes (ABORT and DEAD carry no
    information about tau), so the DP is memoised on (counts, alive flags).

    With ``death`` > 0 every look of a live agent independently kills its
    sensor with probability ``death`` (outcome DEAD, the look is wasted and the
    agent makes no further looks).  The team observes deaths (centralised).
    Agents are labelled: alive = (alive0, alive1); with both alive the joint
    action is an unordered pair (a, b), a <= b, agent 0 taking a.
    """
    A = M.A
    ab = M.inst.abort
    iN = M.outs.index(NULL)
    iH = M.outs.index(HINT) if HINT in M.outs else None
    # per-look truth-dependent factors (without the abort/death constants)
    PN = M.P[:, iN, :] / max(1 - ab, 1e-300) if ab < 1 else np.zeros_like(M.P[:, 0, :])
    PH = (M.P[:, iH, :] / max(1 - ab, 1e-300)) if iH is not None else None
    cN = (1 - death) * (1 - ab)
    branches = [("N", cN), ("A", (1 - death) * ab), ("D", death)]
    if PH is not None:
        branches.insert(1, ("H", cN))
    branches = [b for b in branches if b[1] > 0]
    logPN = np.log(np.maximum(PN, 1e-300))
    logPH = np.log(np.maximum(PH, 1e-300)) if PH is not None else None
    zeroN, zeroH = PN <= 0, (PH <= 0 if PH is not None else None)
    pairs = list(itertools.combinations_with_replacement(range(A), 2))

    def base(key):
        m = np.array(key)
        m0, m1 = m[:A], m[A:]
        lw = np.log(M.p) + m0 @ logPN
        bad = (m0[:, None] > 0) & zeroN
        if PH is not None:
            lw = lw + m1 @ logPH
            bad |= (m1[:, None] > 0) & zeroH
        lw[bad.any(0)] = -np.inf
        return float(np.exp(lw).sum())

    def child(key, a, o):
        if o in ("A", "D"):
            return key
        lst = list(key)
        lst[a + (A if o == "H" else 0)] += 1
        return tuple(lst)

    @lru_cache(maxsize=None)
    def fail(kk, key, alive):
        if kk == 0 or not any(alive):
            return base(key), None
        best, arg = np.inf, None
        if all(alive):
            for a0, a1 in pairs:
                tot = 0.0
                for o0, c0 in branches:
                    k0 = child(key, a0, o0)
                    for o1, c1 in branches:
                        k1 = child(k0, a1, o1)
                        al = (o0 != "D", o1 != "D")
                        tot += c0 * c1 * fail(kk - 1, k1, al)[0]
                if tot < best - 1e-15:
                    best, arg = tot, (a0, a1)
        else:
            j = alive.index(True)
            for a in range(A):
                tot = 0.0
                for o, c in branches:
                    al = [False, False]
                    al[j] = o != "D"
                    tot += c * fail(kk - 1, child(key, a, o), tuple(al))[0]
                if tot < best - 1e-15:
                    best, arg = tot, a
        return best, arg

    k0 = tuple([0] * (2 * A))
    val = 1.0 - fail(k, k0, (True, True))[0]
    if return_policy:
        return val, fail, child
    return val


# ============================================================= open loop
def v_open_loop(M: TinyModel, k: int) -> float:
    """Best decentralised open-loop pair = best multiset of 2k looks (order is
    irrelevant: P(no detection) = sum_tau p(tau) prod_looks P(no detect|tau,a))."""
    nd = M.P.sum(1)  # (A, T) P(non-detect | tau, a), aborts included
    best = np.inf
    for ms in itertools.combinations_with_replacement(range(M.A), 2 * k):
        cnt = np.bincount(ms, minlength=M.A)
        f = float((M.p * np.prod(nd ** cnt[:, None], axis=0)).sum())
        best = min(best, f)
    return 1.0 - best


# ======================================================== best response / PBP
def best_response(M: TinyModel, w: np.ndarray, k: int):
    """min over single-agent policy trees f of <w, f> (w >= 0), exact DP over
    the agent's own outcome histories.  The continuation weight is
    w * prod P[a, o] over the history, so it depends on the history only
    through the multiset of (look, outcome) pairs: memoise on that.
    Returns (min, Policy)."""
    m = len(M.outs)
    Pf = M.P.reshape(M.A * m, -1)

    @lru_cache(maxsize=None)
    def rec(kk, key):
        if kk == 0:
            return None, None
        ww = w * np.prod(Pf ** np.array(key)[:, None], axis=0)
        if kk == 0 or ww.sum() <= 0:
            return float(ww.sum()), None
        best, bp = np.inf, None
        for a in range(M.A):
            tot, ch = 0.0, {}
            for oi, o in enumerate(M.outs):
                lst = list(key)
                lst[a * m + oi] += 1
                v, sub = val(kk - 1, tuple(lst))
                tot += v
                ch[o] = sub
            if tot < best - 1e-15:
                best, bp = tot, Policy(a, ch)
        return best, bp

    def val(kk, key):
        if kk == 0:
            return float((w * np.prod(Pf ** np.array(key)[:, None], axis=0)).sum()), None
        v, p = rec(kk, key)
        if p is None:
            p = _any_tree(M, kk)
        return v, p

    return val(k, tuple([0] * (M.A * m)))


def _any_tree(M, k):
    if k == 0:
        return None
    return Policy(0, {o: _any_tree(M, k - 1) for o in M.outs})


def pbp_check(M: TinyModel, k: int, p0: Policy, p1: Policy, iters: int = 20):
    """Unilateral gains of each agent against the other's fixed tree, and the
    value reached by alternating best responses from (p0, p1)."""
    v = M.pair_value(p0, p1, k)
    f0, f1 = M.policy_fail(p0, k), M.policy_fail(p1, k)
    g0 = (1 - best_response(M, M.p * f1, k)[0]) - v
    g1 = (1 - best_response(M, M.p * f0, k)[0]) - v
    cur = [p0, p1]
    vv = v
    for it in range(iters):
        improved = False
        for j in (0, 1):
            fo = M.policy_fail(cur[1 - j], k)
            fb, pb = best_response(M, M.p * fo, k)
            if 1 - fb > vv + 1e-13:
                cur[j], vv, improved = pb, 1 - fb, True
        if not improved:
            break
    return dict(v=v, gain0=g0, gain1=g1, v_pbp=vv, pbp_pols=cur)


# ============================================================ health beacon
def _histories(outs, k, frames: bool):
    """Alive decision nodes of one agent: histories of (own outcome, frame bit)
    of length 0..k-1 (own outcome never DEAD; at most one frame ever)."""
    step_obs = [(o, f) for o in outs for f in ((0, 1) if frames else (0,))]
    nodes = [()]
    layer = [()]
    for _ in range(k - 1):
        nxt = []
        for h in layer:
            seen = any(f for _, f in h)
            for o, f in step_obs:
                if f and seen:
                    continue
                nxt.append(h + ((o, f),))
        nodes += nxt
        layer = nxt
    return nodes


def beacon_scenarios(k, death, loss, frames):
    """Exogenous (D0, D1, L0, L1): death time in 1..k or k+1 (= never within
    k), and frame delivery.  Yields (prob, d0, r0, d1, r1) with r_j the step
    *after* which agent j sees the partner frame (0 = never)."""
    pd = {d: death * (1 - death) ** (d - 1) for d in range(1, k + 1)}
    pd[k + 1] = (1 - death) ** k
    pl = {1: 1 - loss, 0: loss} if frames else {0: 1.0}
    for d0, d1 in itertools.product(pd, pd):
        for l0, l1 in itertools.product(pl, pl):
            pr = pd[d0] * pd[d1] * pl[l0] * pl[l1]
            if pr == 0:
                continue
            # agent 0 sees agent 1's frame after step d1 (if delivered, d1 < k)
            r0 = d1 if (l1 and d1 < k) else 0
            r1 = d0 if (l0 and d0 < k) else 0
            yield pr, d0, r0, d1, r1


def _fail_given_scen(M, pol: dict, k, d, r):
    """Failure vector of a dict-policy (history -> action) for own death time
    d and partner-frame step r.  Looks at steps >= d are wasted."""
    T = len(M.truths)

    def rec(t, h):  # t = number of looks done
        if t == k or t + 1 >= d:
            return np.ones(T)
        a = pol[h]
        out = np.zeros(T)
        for oi, o in enumerate(M.outs):
            out += M.P[a, oi] * rec(t + 1, h + ((o, 1 if r == t + 1 else 0),))
        return out
    return rec(0, ())


def beacon_dec(M: TinyModel, k, death, loss, frames=True, brute=True, pols=None):
    """Exact optimal decentralised value with private sensor death and (if
    ``frames``) one death frame per agent, delivered w.p. 1-loss before the
    partner's next decision.  Brute force over all deterministic policy pairs
    (tiny k only).  Returns (value, argmax pair)."""
    nodes = _histories(M.outs, k, frames)
    scen = list(beacon_scenarios(k, death, loss, frames))
    keys = sorted({(d0, r0) for _, d0, r0, _, _ in scen} | {(d1, r1) for _, _, _, d1, r1 in scen})
    kidx = {kk: i for i, kk in enumerate(keys)}
    if pols is None:
        pols = [dict(zip(nodes, acts)) for acts in itertools.product(range(M.A), repeat=len(nodes))]
    F = np.array([[_fail_given_scen(M, pl, k, d, r) for (d, r) in keys] for pl in pols])
    # F: (Npol, Nkeys, T)
    Mx = np.zeros((len(pols), len(pols)))
    agg = {}
    for pr, d0, r0, d1, r1 in scen:
        agg[(kidx[(d0, r0)], kidx[(d1, r1)])] = agg.get((kidx[(d0, r0)], kidx[(d1, r1)]), 0) + pr
    for (i0, i1), pr in agg.items():
        Mx += pr * (F[:, i0, :] * M.p) @ F[:, i1, :].T
    i = int(np.argmin(Mx))
    r, c = divmod(i, Mx.shape[1])
    return 1.0 - float(Mx[r, c]), (pols[r], pols[c]), len(pols)


def beacon_pair_value(M, k, death, loss, pol0, pol1, frames=True):
    tot = 0.0
    for pr, d0, r0, d1, r1 in beacon_scenarios(k, death, loss, frames):
        tot += pr * float((M.p * _fail_given_scen(M, pol0, k, d0, r0)
                           * _fail_given_scen(M, pol1, k, d1, r1)).sum())
    return 1.0 - tot


def centralised_to_beacon_pair(M: TinyModel, k, death):
    """Prop. 3 construction: each agent replays the centralised optimal policy
    on the joint history it can reconstruct from its own history plus the
    partner's death frame (outcomes NULL/DETECT only).  Returns dict policies."""
    assert M.outs == [NULL]
    vc, fail, child = v_cent_fast(M, k, death, return_policy=True)
    pols = [{}, {}]
    for j in (0, 1):
        for h in _histories(M.outs, k, True):
            # reconstruct joint state: own looks all NULL; partner alive until
            # its frame step, NULL before it, DEAD at it.
            key = tuple([0] * (2 * M.A))
            alive = (True, True)
            frame_step = next((t + 1 for t, (_, f) in enumerate(h) if f), None)
            ok = True
            for t in range(len(h)):
                act = fail(k - t, key, alive)[1]
                acts = list(act) if all(alive) else [None, None]
                if not all(alive):
                    acts[alive.index(True)] = act
                al = list(alive)
                for who in (0, 1):
                    if not alive[who]:
                        continue
                    if who == j:
                        key = child(key, acts[who], "N")
                    elif frame_step == t + 1:
                        al[who] = False
                    else:
                        key = child(key, acts[who], "N")
                alive = tuple(al)
            act = fail(k - len(h), key, alive)[1]
            if all(alive):
                pols[j][h] = act[j]
            else:
                pols[j][h] = act if alive[j] else 0
            del ok
    return vc, pols


# ================================================================== tasks
def task_landscape(T=2, nb=21, na=21, amax=0.5, which=("base", "reviewer"),
                   aborts=(0.0, 0.1)):
    hbs = np.linspace(0, 1, nb)
    als = np.linspace(0, amax, na)
    out = {"T": T, "h_bad": hbs.tolist(), "alpha": als.tolist(), "grids": {}}
    for nm in which:
        for ab in aborts:
            G = np.zeros((nb, na))
            t0 = time.time()
            for i, hb in enumerate(hbs):
                for j, al in enumerate(als):
                    inst = (base2(0.3, hb, al, ab) if nm == "base"
                            else reviewer_instance(h_bad=hb, alpha=al, abort=ab))
                    M = TinyModel(inst)
                    G[i, j] = v_cent_fast(M, T) - M.v_dec(T, prune=pareto)
            out["grids"][f"{nm}|{ab}"] = G.tolist()
            print(f"landscape {nm} ab={ab} T={T}: max {G.max():.4e} at "
                  f"h_b={hbs[np.unravel_index(G.argmax(), G.shape)[0]]:.2f} "
                  f"alpha={als[np.unravel_index(G.argmax(), G.shape)[1]]:.3f} "
                  f"({time.time() - t0:.0f}s)", flush=True)
    return out


def task_points(T_list=(2, 3)):
    """Named points: paper base rates along lambda, reviewer max, etc."""
    rows = []
    pts = [("base2 weak (exact_dec)", base_instance(2, "weak", 0.0))]
    for lam in (1, 2, 3):
        r = paper_rates(lam)
        pts.append((f"base2, paper rates lambda={lam}", base2(**r)))
        pts.append((f"base2, paper rates lambda={lam}, no abort", base2(**{**r, "abort": 0.0})))
    pts.append(("reviewer max (h_g=0,h_b=1,a=0)", reviewer_instance()))
    pts.append(("reviewer, h_b=.9, a=.02", reviewer_instance(h_bad=0.9, alpha=0.02)))
    pts.append(("reviewer, paper rates lambda=1", reviewer_instance(**{k_: v for k_, v in paper_rates(1).items()})))
    for nm, inst in pts:
        for T in T_list:
            if T > 2 and inst.abort > 0:
                continue  # Pareto set with 3 outcomes at T=3 is too large for the budget
            M = TinyModel(inst)
            t0 = time.time()
            vc = v_cent_fast(M, T)
            vd = M.v_dec(T, prune=pareto)
            vi = M.v_implicit(T)
            rows.append(dict(name=nm, T=T, vc=vc, vd=vd, vi=vi, voc=vc - vd, rel=(vc - vd) / vc,
                             s=time.time() - t0))
            print(f"| {nm} | {T} | {vc:.6f} | {vd:.6f} | {vi:.6f} | {vc - vd:.4e} | "
                  f"{(vc - vd) / vc:.2%} |", flush=True)
    return rows


def task_beacon(death=0.2):
    rows = []

    def row(**kw):
        rows.append(kw)
        print(" | ".join(f"{k_}={v:.6g}" if isinstance(v, float) else f"{k_}={v}"
                         for k_, v in kw.items()), flush=True)

    cases = [("n2", 2, base_instance(2, "none", 0.0)), ("n3", 3, base_instance(3, "none", 0.0)),
             ("rev-nohint", 2, reviewer_instance(h_bad=0.0))]
    for nm, n, inst in cases:
        M = TinyModel(inst)
        for k in (2, 3):
            if n == 3 and k == 3:
                continue
            vc = v_cent_fast(M, k, death)
            vd_ideal, _, npol = beacon_dec(M, k, death, 0.0, True)
            vd_none, _, _ = beacon_dec(M, k, death, 0.0, False)
            vd_lossy, _, _ = beacon_dec(M, k, death, 0.5, True)
            row(case=nm, k=k, death=death, V_cent=vc, V_dec_frame_ideal=vd_ideal,
                V_dec_noframe=vd_none, V_dec_frame_loss50=vd_lossy,
                VoC_ideal=vc - vd_ideal, VoC_noframe=vc - vd_none, VoC_loss50=vc - vd_lossy,
                n_policies=npol)
        # constructive check (any k): the replayed centralised policy
        for k in (2, 3, 4, 5):
            if n == 3 and k > 4:
                continue
            vc, pols = centralised_to_beacon_pair(M, k, death)
            vpair = beacon_pair_value(M, k, death, 0.0, pols[0], pols[1], True)
            vlossy = beacon_pair_value(M, k, death, 0.5, pols[0], pols[1], True)
            row(case=nm, k=k, check="construction", V_cent=vc, V_constructed_pair=vpair,
                diff=vc - vpair, same_pair_loss50=vlossy)
    # hints present: the frame does not close the gap
    for nm, inst in (("n2 weak", base_instance(2, "weak", 0.0)),
                     ("n2 strong", base_instance(2, "strong", 0.0)),
                     ("reviewer", reviewer_instance())):
        M = TinyModel(inst)
        k = 2
        vc = v_cent_fast(M, k, death)
        vd_ideal, _, npol = beacon_dec(M, k, death, 0.0, True)
        vd_none, _, _ = beacon_dec(M, k, death, 0.0, False)
        row(case=nm + " +hints", k=k, death=death, V_cent=vc, V_dec_frame_ideal=vd_ideal,
            V_dec_noframe=vd_none, VoC_ideal=vc - vd_ideal, VoC_noframe=vc - vd_none,
            n_policies=npol)
    # sanity: death=0 reduces to exact_dec
    M = TinyModel(base_instance(2, "weak", 0.0))
    row(case="sanity death=0 weak", k=2, V_cent_fast=v_cent_fast(M, 2), V_cent=M.v_cent(2),
        V_dec_bf=beacon_dec(M, 2, 0.0, 0.0, False)[0], V_dec=M.v_dec(2, prune=pareto))
    return rows


def task_bound(kmax=6, exact_k=(2, 3)):
    rows = []
    insts = [(f"n{n} {h} ab{ab}", base_instance(n, h, ab)) for n in (2, 3)
             for h in ("none", "weak", "strong") for ab in (0.0, 0.2)]
    insts += [("n2 paper lambda=1", base2(**paper_rates(1))),
              ("reviewer max", reviewer_instance()),
              ("reviewer ab .1", reviewer_instance(abort=0.1))]
    for nm, inst in insts:
        M = TinyModel(inst)
        n = inst.n
        for k in range(2, kmax + 1):
            if n == 3 and k > 5:
                continue
            t0 = time.time()
            vc = v_cent_fast(M, k)
            vol = v_open_loop(M, k)
            (p0, p1), _ = M.implicit_policies(k)
            r = pbp_check(M, k, p0, p1)
            vlow = max(vol, r["v"], r["v_pbp"])
            vd = None
            if k in exact_k and (n == 2 or k == 2) and not (k == 3 and len(M.outs) == 3):
                vd = M.v_dec(k, prune=pareto)
            row = dict(name=nm, k=k, vc=vc, vol=vol, vimp=r["v"], vpbp=r["v_pbp"],
                       bound_ol=vc - vol, bound_best=vc - vlow,
                       voc=(vc - vd) if vd is not None else None, s=time.time() - t0)
            rows.append(row)
            print(f"| {nm} | {k} | {vc:.5f} | {vol:.5f} | {r['v']:.5f} | {r['v_pbp']:.5f} | "
                  f"{vc - vol:.4e} | {vc - vlow:.4e} | "
                  f"{'' if vd is None else f'{vc - vd:.4e}'} | {time.time() - t0:.1f} |", flush=True)
    return rows


def task_pbp(nrandom=600):
    rows = []
    for n, ks in ((2, (2, 3, 4)), (3, (2, 3))):
        for h in HINTS:
            for ab in (0.0, 0.2):
                M = TinyModel(base_instance(n, h, ab))
                for k in ks:
                    (p0, p1), _ = M.implicit_policies(k)
                    r = pbp_check(M, k, p0, p1)
                    rows.append(dict(name=f"n{n} {h} ab{ab}", k=k, v=r["v"], gain0=r["gain0"],
                                     gain1=r["gain1"], v_pbp=r["v_pbp"]))
                    print(f"| n{n} {h} ab{ab} | {k} | {r['v']:.6f} | {r['gain0']:.3e} | "
                          f"{r['gain1']:.3e} | {r['v_pbp']:.6f} |", flush=True)
    for nm, inst in (("reviewer max", reviewer_instance()),):
        M = TinyModel(inst)
        for k in (2, 3):
            (p0, p1), _ = M.implicit_policies(k)
            r = pbp_check(M, k, p0, p1)
            rows.append(dict(name=nm, k=k, v=r["v"], gain0=r["gain0"], gain1=r["gain1"],
                             v_pbp=r["v_pbp"]))
            print(f"| {nm} | {k} | {r['v']:.6f} | {r['gain0']:.3e} | {r['gain1']:.3e} | "
                  f"{r['v_pbp']:.6f} |", flush=True)
    # random instances (same generator / seed as exact_dec --random)
    rng = np.random.default_rng(2026)
    gains, vdgap = [], []
    for _ in range(nrandom):
        inst = random_instance(rng, 2)
        M = TinyModel(inst)
        (p0, p1), _ = M.implicit_policies(2)
        r = pbp_check(M, 2, p0, p1)
        vd = M.v_dec(2, prune=pareto)
        gains.append(max(r["gain0"], r["gain1"]))
        vdgap.append((vd - r["v"], vd - r["v_pbp"]))
    gains = np.array(gains)
    vdgap = np.array(vdgap)
    rnd = dict(n=nrandom, frac_not_pbp=float((gains > 1e-9).mean()), max_gain=float(gains.max()),
               mean_gain=float(gains.mean()), median_gain_when_pos=float(np.median(gains[gains > 1e-9]))
               if (gains > 1e-9).any() else 0.0,
               frac_pbp_from_imp_is_dec_opt=float((vdgap[:, 1] < 1e-9).mean()),
               max_gap_dec_minus_pbp=float(vdgap[:, 1].max()),
               max_gap_dec_minus_imp=float(vdgap[:, 0].max()))
    print("random:", rnd, flush=True)
    return rows, rnd


def main(argv=None):
    argv = argv or sys.argv[1:]
    task = argv[0] if argv else "all"
    out = {}
    path = HERE / "results" / f"theory_extra_{task}.json"
    if task in ("landscape", "all"):
        out["landscape_T2"] = task_landscape(2)
    if task in ("landscape3",):
        out["landscape_T3"] = task_landscape(3, nb=6, na=6, amax=0.5, which=("base", "reviewer"), aborts=(0.0,))
    if task == "plot":
        plot_landscape()
        return
    if task in ("points", "all"):
        out["points"] = task_points()
    if task in ("beacon", "all"):
        out["beacon"] = task_beacon()
    if task in ("bound", "all"):
        out["bound"] = task_bound()
    if task in ("pbp", "all"):
        rows, rnd = task_pbp()
        out["pbp"] = rows
        out["pbp_random"] = rnd
    path.write_text(json.dumps(out, indent=1, default=float))
    print("wrote", path)



# ================================================================== figure
def plot_landscape(src=None, out=None):
    """Appendix figure: exact VoC(h_b, alpha) at n=2, T=2 (Type 1 fonts via usetex)."""
    import matplotlib
    matplotlib.use("Agg")
    matplotlib.rcParams.update({"text.usetex": True, "font.family": "serif", "font.size": 7,
                                "pdf.fonttype": 42})
    import matplotlib.pyplot as plt
    src = src or HERE / "results" / "theory_extra_landscape.json"
    out = out or HERE / "paper" / "figs" / "figA_voc.pdf"
    d = json.loads(Path(src).read_text())["landscape_T2"]
    hb, al = np.array(d["h_bad"]), np.array(d["alpha"])
    panels = [("base|0.0", r"base instance, $\rho=0$"), ("base|0.1", r"base instance, $\rho=0.1$"),
              ("reviewer|0.0", r"adversarial instance, $\rho=0$"),
              ("reviewer|0.1", r"adversarial instance, $\rho=0.1$")]
    vmax = max(np.max(d["grids"][k]) for k, _ in panels)
    fig, axs = plt.subplots(1, 4, figsize=(7.0, 1.85), sharey=True, constrained_layout=True)
    for ax, (key, title) in zip(axs, panels):
        G = np.clip(np.array(d["grids"][key]), 0, None)
        im = ax.pcolormesh(al, hb, G, cmap="Blues", vmin=0, vmax=vmax, shading="nearest",
                           rasterized=False)
        ax.set_title(title, fontsize=7)
        ax.set_xlabel(r"false-hint rate $\alpha$")
        if key.startswith("base"):
            lam = np.array([1, 2, 3])
            ax.plot(0.03 * lam, 0.15 * lam, "o", ms=3.5, mfc="white", mec="#D55E00", mew=0.9)
            for l_ in lam:
                ax.annotate(rf"$\lambda{{=}}{l_}$", (0.03 * l_, 0.15 * l_), xytext=(4, -2),
                            textcoords="offset points", fontsize=6, color="#D55E00")
        i, j = np.unravel_index(np.argmax(G), G.shape)
        ax.plot(al[j], hb[i], "*", ms=6, mfc="#D55E00", mec="white", mew=0.4, clip_on=False)
        ax.annotate(f"{G[i, j]:.3f}", (al[j], hb[i]), xytext=(5, -7), textcoords="offset points",
                    fontsize=6, color="0.15")
    axs[0].set_ylabel(r"bad-view hint rate $h_b$")
    cb = fig.colorbar(im, ax=axs, shrink=0.9, pad=0.01)
    cb.set_label(r"VoC $=V_{\rm cent}-V_{\rm dec}$")
    fig.savefig(out)
    print("wrote", out)


if __name__ == "__main__":
    main()
