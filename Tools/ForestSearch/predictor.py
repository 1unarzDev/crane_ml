"""Decision-relevant divergence (DRD) and coordination regret (CR) as predictors of the
value of communication (new file; no existing file is edited).

Definitions (team-decision terms; full statements in results/predictor.md)
----------------------------------------------------------------------------
Joint history h_t, local histories h^j_t.  mu* = a centralised optimal policy with value-to-go
F*_t(h) (here: unnormalised FAILURE mass, so smaller is better) and
Q*_t(h, a) = sum_o F*_{t+1}(h a o).  A decentralised pair pi = (pi^j) plays a_t = pi(h_t) =
(pi^j(h^j_t))_j.

  CR_t(h)   = Q*_t(h, pi(h)) - F*_t(h) >= 0        (coordination regret of acting on local info)
  DRD_t     = P_pi( pi(h_t) not in argmin_a Q*_t(h_t, a) )  (decision-relevant divergence)
  DRD^phi_t = P_pi( pi(h_t) != mu*(h_t) )         (with pi = pi^phi: the partner's action changes
              when its prediction phi^j(h^j) of the joint history is replaced by the truth)

  Performance-difference identity:  V_cent - V(pi) = E_pi[ sum_t CR_t ]  (exact).
  Hence VoC = V_cent - V_dec <= E_pi[sum_t CR_t] for EVERY decentralised pi, with equality at
  an optimal decentralised pair; CR_t = 0 off the DRD event, so DRD == 0  =>  VoC = 0.

Part 1 (``exact``): exact check on tiny travel-free instances (exact_dec / theory_extra models),
including sensor death with and without the one-frame beacon.
Part 2 (``sim``): instrumented simulator.  At every IMPLICIT decision the receiver's greedy
argmax is recomputed with the partner's TRUE state (records + in-flight task; ORACLE's
information) and with the information a HEALTH frame would carry (delayed by the link
delay; nothing if the radio is dead or the frame dropped).  DRD = argmax changed; CR = short
rollout value difference under the true joint belief (only computed when the argmax changed).

cd Tools && fsrun python3 -m ForestSearch.predictor exact
cd Tools && fsrun python3 -m ForestSearch.predictor sim --split dev --out /tmp/pred_dev.json
cd Tools && fsrun python3 -m ForestSearch.predictor sim --split heldout --out /tmp/pred_ho.json
python3 -m ForestSearch.predictor report --dev /tmp/pred_dev.json --heldout /tmp/pred_ho.json
"""
from __future__ import annotations

import argparse
import json
import math
import time
from dataclasses import replace

import numpy as np

from .model import ABORT, DETECT, HINT, NULL, Instance, detect_prob

# =========================================================================== exact
OUTC = {"N": NULL, "H": HINT, "A": ABORT}


class ExactCR:
    """Exact CR / DRD / VoC for 2 agents, synchronous, travel-free, k steps, optional
    per-look sensor death ``death`` (private DEAD outcome) and optional one-frame beacon."""

    def __init__(self, M, k, death=0.0):
        from .theory_extra import v_cent_fast
        self.M, self.k, self.death = M, k, death
        self.vc, self.fail, self.child = v_cent_fast(M, k, death, return_policy=True)
        ab = M.inst.abort
        cN = (1 - death) * (1 - ab)
        br = [("N", cN), ("A", (1 - death) * ab), ("D", death)]
        if HINT in M.outs:
            br.insert(1, ("H", cN))
        self.branches = [b for b in br if b[1] > 0]
        self.key0 = tuple([0] * (2 * M.A))

    # ------------------------------------------------------------ centralised pieces
    def F(self, kk, key, alive):
        return self.fail(kk, key, alive)[0]

    def Q(self, kk, key, alive, acts):
        """Unnormalised failure mass of joint action ``acts`` (None for a dead agent),
        optimal continuation."""
        tot = 0.0
        alive_idx = [j for j in (0, 1) if alive[j]]
        if len(alive_idx) == 2:
            for o0, c0 in self.branches:
                k0 = self.child(key, acts[0], o0)
                for o1, c1 in self.branches:
                    k1 = self.child(k0, acts[1], o1)
                    tot += c0 * c1 * self.F(kk - 1, k1, (o0 != "D", o1 != "D"))
        else:
            j = alive_idx[0]
            for o, c in self.branches:
                al = [False, False]
                al[j] = o != "D"
                tot += c * self.F(kk - 1, self.child(key, acts[j], o), tuple(al))
        return tot

    def mu(self, kk, key, alive):
        arg = self.fail(kk, key, alive)[1]
        if all(alive):
            return list(arg)
        out = [None, None]
        out[alive.index(True)] = arg
        return out

    def base(self, key):
        return self.F(0, key, (False, False))

    # ---------------------------------------------------------------- evaluation
    def evaluate(self, pols, frames=False):
        """pols[j](own_hist) -> action; own_hist = tuple of (outcome, frame) with frame = 1 iff
        the partner's look at that step was DEAD and its frame is delivered (``frames``).
        Returns dict with V(pi), per-step CR, DRD, DRD^phi (vs mu*), CR split by whether a
        private event (H/A/D) has occurred before the step, and CR at nodes with a dead agent."""
        k = self.k
        cr = np.zeros(k); drd = np.zeros(k); drdphi = np.zeros(k)
        split = {"no_event": 0.0, "after_event": 0.0, "dead_unknown": 0.0, "dead_known": 0.0,
                 "first_after_death": 0.0}
        fail_tot = [0.0]

        def rec(t, key, alive, C, hs, ev, dead_since):
            mass = C * self.base(key)
            if t == k or not any(alive):
                fail_tot[0] += mass
                return
            kk = k - t
            acts = [pols[j](hs[j]) if alive[j] else None for j in (0, 1)]
            Fs = self.F(kk, key, alive)
            Qa = self.Q(kk, key, alive, acts)
            c = C * (Qa - Fs)
            cr[t] += c
            if Qa - Fs > 1e-13 * max(1.0, Fs):
                drd[t] += mass
            if acts != self.mu(kk, key, alive):
                drdphi[t] += mass
            split["after_event" if ev else "no_event"] += c
            if not all(alive):
                known = frames
                split["dead_known" if known else "dead_unknown"] += c
                if dead_since == t:
                    split["first_after_death"] += c
            # children
            alive_idx = [j for j in (0, 1) if alive[j]]
            combos = [[]]
            for j in alive_idx:
                combos = [cb + [(j, o, cc)] for cb in combos for (o, cc) in self.branches]
            for cb in combos:
                k2, C2, al2, hs2, ev2 = key, C, list(alive), list(hs), ev
                died = {j for (j, o, _) in cb if o == "D"}
                for (j, o, cc) in cb:
                    k2 = self.child(k2, acts[j], o)
                    C2 *= cc
                    if o == "D":
                        al2[j] = False
                    if o in ("H", "A", "D"):
                        ev2 = True
                for j in alive_idx:
                    if j in died:
                        continue
                    o = next(o for (jj, o, _) in cb if jj == j)
                    f = 1 if (frames and (1 - j) in died) else 0
                    hs2[j] = hs[j] + ((o, f),)
                ds = dead_since if dead_since is not None else (t + 1 if died else None)
                rec(t + 1, k2, tuple(al2), C2, tuple(hs2), ev2, ds)

        rec(0, self.key0, (True, True), 1.0, ((), ()), False, None)
        V = 1.0 - fail_tot[0]
        return dict(V=V, cr=cr, drd=drd, drdphi=drdphi, split=split,
                    pd_identity=(self.vc - V) - cr.sum())

    # -------------------------------------------------------------- policies
    def phi_policies(self, frames=False):
        """pi^phi: agent j replays mu* on the joint history it PREDICTS from its own history:
        partner outcomes all NULL (alive) unless a death frame says otherwise."""
        def make(j):
            cache = {}

            def pol(h):
                if h in cache:
                    return cache[h]
                key, alive = self.key0, (True, True)
                for t, (o, f) in enumerate(h):
                    acts = self.mu(self.k - t, key, alive)
                    al = list(alive)
                    for who in (0, 1):
                        if not alive[who]:
                            continue
                        if who == j:
                            key = self.child(key, acts[who], o)
                        elif f:
                            al[who] = False
                        else:
                            key = self.child(key, acts[who], "N")
                    alive = tuple(al)
                a = self.mu(self.k - len(h), key, alive)[j]
                cache[h] = a
                return a
            return pol
        return [make(0), make(1)]

    @staticmethod
    def tree_policies(p0, p1):
        """Adapt exact_dec.Policy trees (keyed by own outcome code) to own_hist callables."""
        def make(p):
            def pol(h):
                node = p
                for (o, _f) in h:
                    node = node.children[OUTC[o]]
                return node.action
            return pol
        return [make(p0), make(p1)]


def exact_task(out_json=None, part="all"):
    from .exact_dec import TinyModel, base_instance
    from .theory_extra import base2, paper_rates, reviewer_instance
    rows = []

    def row(**r):
        rows.append(r)
        if out_json:
            json.dump(rows, open(out_json, "w"), indent=1)
        print({k: (round(v, 8) if isinstance(v, float) else v) for k, v in r.items()}, flush=True)

    cases = []
    for n, k in ([] if part == "death" else ((2, 2), (3, 2), (2, 3))):
        for hints in ("none", "weak", "strong", "noisy"):
            for ab in (0.0, 0.2):
                if (n, k) == (2, 3) and hints in ("strong", "noisy") and ab > 0:
                    continue
                cases.append((f"n{n}-{hints}-ab{ab}", base_instance(n, hints, ab), k))
    if part == "death":
        cases = []
    cases.append(("paper-lam1", base2(**paper_rates(1.0)), 2))
    cases.append(("adversarial", reviewer_instance(), 2))
    cases.append(("adversarial", reviewer_instance(), 3))
    for nm, inst, k in cases:
        M = TinyModel(inst)
        E = ExactCR(M, k)
        t0 = time.time()
        vd, p0, p1, _ = M.v_dec(k, True)
        voc = E.vc - vd
        res = {}
        res["phi"] = E.evaluate(E.phi_policies())
        res["dec*"] = E.evaluate(E.tree_policies(p0, p1))
        if nm != "adversarial" or k == 2:
            try:
                (i0, i1), _ = M.implicit_policies(k)
                res["implicit"] = E.evaluate(E.tree_policies(i0, i1))
            except Exception as e:  # posterior 0*log0 on the adversarial instance
                print("implicit failed", nm, e)
        for pn, r in res.items():
            dQ = r["cr"].sum()
            row(case=nm, k=k, pol=pn, V_cent=E.vc, V_dec=vd, VoC=voc, V_pi=r["V"],
                sumCR=float(dQ), identity_err=float(r["pd_identity"]),
                bound_ok=bool(voc <= dQ + 1e-12), sumDRD=float(r["drd"].sum()),
                sumDRDphi=float(r["drdphi"].sum()),
                CR_no_event=r["split"]["no_event"], CR_after_event=r["split"]["after_event"],
                cr_t=r["cr"].tolist(), drd_t=r["drd"].tolist(), s=round(time.time() - t0, 1))
    # ---- sensor death: beacon / no beacon (outcomes NULL/DETECT/DEAD)
    for nm, inst, ks in (("n2-none", base_instance(2, "none", 0.0), (2, 3)),
                         ("n3-none", base_instance(3, "none", 0.0), (2,)),
                         ("n2-weak", base_instance(2, "weak", 0.0), (2,))):
        M = TinyModel(inst)
        for k in ks:
            E = ExactCR(M, k, death=0.2)
            from .theory_extra import beacon_dec
            vd_none = beacon_dec(M, k, 0.2, 0.0, frames=False)[0] if k <= 2 or nm == "n2-none" else float("nan")
            for frames in (False, True):
                r = E.evaluate(E.phi_policies(frames), frames=frames)
                row(case=nm + "+death0.2", k=k, pol="phi+beacon" if frames else "phi(no frame)",
                    V_cent=E.vc, V_dec_noframe=vd_none, VoC_noframe=E.vc - vd_none, V_pi=r["V"],
                    sumCR=float(r["cr"].sum()), identity_err=float(r["pd_identity"]),
                    sumDRD=float(r["drd"].sum()), CR_no_event=r["split"]["no_event"],
                    CR_dead_unknown=r["split"]["dead_unknown"], CR_dead_known=r["split"]["dead_known"],
                    CR_first_after_death=r["split"]["first_after_death"],
                    cr_t=r["cr"].tolist(), drd_t=r["drd"].tolist())
    if out_json:
        json.dump(rows, open(out_json, "w"), indent=1)
    return rows


# ============================================================================= sim
def _rollout_value(inst, m0, m1, pos, first, budget, bm):
    """P(detect within ``budget`` steps) of the receiver's sequence: ``first`` then greedy,
    NULL-assumed continuation, under the belief (m0, m1).  Same rollout as sim._choose."""
    c, v = first
    mm0 = m0.copy()
    used, p_none, val = 0, 1.0, 0.0
    pd = detect_prob(inst, mm0, m1, bm)
    while True:
        dt = int(inst.dist[pos, c]) + 1
        if used + dt > budget and used > 0:
            break
        used += dt
        val += p_none * pd[c, v]
        p_none *= 1.0 - pd[c, v]
        mm0[c, v] += 1
        pos = c
        if used >= budget:
            break
        pd = detect_prob(inst, mm0, m1, bm)
        sc = pd / (inst.dist[pos][:, None] + 1.0)
        c, v = divmod(int(np.argmax(sc)), inst.V)
    return val


def _greedy(inst, m0, m1, virtual, pos, bm):
    m0 = m0.copy()
    for (c, v) in virtual:
        m0[c, v] += 1
    pd = detect_prob(inst, m0, m1, bm)
    return divmod(int(np.argmax(pd / (inst.dist[pos][:, None] + 1.0))), inst.V), m0


def _make_probe_episode():
    from .beacon_robustness import BEpisode
    from .sim import Episode

    class PEpisode(BEpisode):
        """IMPLICIT episode (any fault) with a per-decision DRD/CR probe.  The probe consumes
        no randomness, so the trajectory equals the plain IMPLICIT run (tested)."""
        budget = 6
        hdelay = 0          # link delay of the HEALTH frame
        hdeliver = 1.0      # P(frame delivered) used to weight CR_health

        def _true_counts(self, a):
            inst = self.inst
            p = self.agents[1 - a.idx]
            m0 = a.m0.copy(); m1 = a.m1.copy()
            for r in p.records:
                if r.outcome == NULL:
                    m0[r.cell, r.view] += 1
                elif r.outcome == HINT:
                    m1[r.cell, r.view] += 1
            virtual = []
            if p.task is not None and (p.idx, p.task.seq) not in self.forced:
                virtual.append((p.task.cell, p.task.view))
            return m0, m1, virtual

        def _health_counts(self, a, t):
            """What a HEALTH receiver would believe at t (None = no frame yet)."""
            p = 1 - a.idx
            td = self.t_dead[p] if hasattr(self, "t_dead") else None
            if td is None or t < td + self.hdelay or self.nominal is None:
                return None
            if self.radio_dead[p, min(td, self.radio_dead.shape[1] - 1)] or self.fault.get("drop_first"):
                return None
            pm0, pm1, virt = self._counts_filtered(a, t, lambda tk: tk.start < td)
            return a.m0 + pm0, a.m1 + pm1, virt

        def _choose(self, a, t):
            c_imp = super()._choose(a, t)
            if (a.idx, a.seq) in self.forced:
                return c_imp            # this look is wasted (dead sensor): no decision value
            inst, bm = self.inst, a.s.belief
            m0, m1, virt = self._true_counts(a)
            c_full, tm0 = _greedy(inst, m0, m1, virt, a.pos, bm)
            st = self.probe
            st["dec"] += 1
            cache = {}

            def val(c):
                if c not in cache:
                    cache[c] = _rollout_value(inst, tm0, m1, a.pos, c, self.budget, bm)
                return cache[c]
            if c_full != c_imp:
                d = val(c_full) - val(c_imp)
                st["drd"] += 1
                st["cr"] += max(0.0, d)
                st["cr_raw"] += d
                st["events"].append((t, a.idx, round(d, 6)))
                td = [x for x in getattr(self, "t_dead", [None, None]) if x is not None]
                if td:
                    if t <= td[0]:
                        st["cr_pre"] += max(0.0, d)
                    elif t <= td[0] + 2:
                        st["cr_fail"] += max(0.0, d)
            hc = self._health_counts(a, t)
            if hc is not None:
                c_h, _ = _greedy(inst, hc[0], hc[1], hc[2], a.pos, bm)
                if c_h != c_imp:
                    st["drd_h"] += 1
                    st["cr_h"] += self.hdeliver * (val(c_h) - val(c_imp))
            return c_imp

    return PEpisode


def _new_probe():
    return dict(dec=0, drd=0, cr=0.0, cr_raw=0.0, drd_h=0, cr_h=0.0, cr_pre=0.0, cr_fail=0.0, events=[])


def run_probe(ctx, ep, S, fault, link, budget):
    from .beacon_robustness import LINKS
    PE = _make_probe_episode()
    f = dict(fault or {}); f["seed"] = 10_000 + ep
    L = LINKS[link]
    e = PE(ctx.inst, (S["IMPLICIT"], S["IMPLICIT"]), L, np.random.default_rng(ep),
           ctx.nom, ctx.sec, fault=f, sec_nominal=ctx.snom)
    e.budget = budget
    e.hdelay = L.delay * L.hops
    e.hdeliver = 1.0 - L.channel.mean_loss
    e.probe = _new_probe()
    r = e.run()
    bad = getattr(e, "bad", np.zeros((2, 1), bool))
    return r, e.probe, float(bad.mean())


# ---------------------------------------------------------------- N agents
def _make_probe_n():
    from .nagent import NEpisode

    class PN(NEpisode):
        budget = 6

        def _choose(self, a, t):
            c_imp = super()._choose(a, t)
            inst, bm = self.inst, a.s.belief
            m0, m1 = a.m0.copy(), a.m1.copy()
            virt = []
            for p in self.agents:
                if p.idx == a.idx:
                    continue
                for r in p.records:
                    self._add(m0, m1, r)
                if p.task is not None:
                    virt.append((p.task.cell, p.task.view))
            c_full, tm0 = _greedy(inst, m0, m1, virt, a.pos, bm)
            st = self.probe
            st["dec"] += 1
            if c_full != c_imp:
                d = (_rollout_value(inst, tm0, m1, a.pos, c_full, self.budget, bm)
                     - _rollout_value(inst, tm0, m1, a.pos, c_imp, self.budget, bm))
                st["drd"] += 1
                st["cr"] += max(0.0, d)
                st["cr_raw"] += d
                st["events"].append((t, a.idx, round(d, 6)))
            return c_imp
    return PN


# --------------------------------------------------------------- conditions
def _inst(kind, seed, lam=1.0, **ov):
    from .event_curve import events
    from .instances import grid_instance
    e = dict(events(lam)); e.update(ov)
    if kind == "A":
        return grid_instance(5, 5, seed=seed, travel_scale=0, horizon=16, **e)
    if kind == "B":
        return grid_instance(5, 5, seed=seed, travel_scale=1, horizon=30, **e)
    raise ValueError(kind)


NOHINT = dict(h_good=0.0, h_bad=0.0, alpha=0.0)
# name -> (N, instance kind, lam, overrides, fault cond, link)
CONDS = {
    "healthy_l0": (2, "A", 0.0, {}, "healthy", "d0_loss0"),
    "healthy_l1": (2, "A", 1.0, {}, "healthy", "d0_loss0"),
    "healthy_l2": (2, "A", 2.0, {}, "healthy", "d0_loss0"),
    "healthy_l3": (2, "A", 3.0, {}, "healthy", "d0_loss0"),
    "hints_l3": (2, "A", 3.0, dict(abort=0.0), "healthy", "d0_loss0"),
    "aborts_r3": (2, "A", 1.0, dict(abort=0.3, **NOHINT), "healthy", "d0_loss0"),
    "travel_l1": (2, "B", 1.0, {}, "healthy", "d0_loss0"),
    "travel_l3": (2, "B", 3.0, {}, "healthy", "d0_loss0"),
    "perm": (2, "A", 1.0, {}, "perm", "d0_loss0"),
    "perm_early": (2, "A", 1.0, {}, "perm_early", "d0_loss0"),
    "perm_late": (2, "A", 1.0, {}, "perm_late", "d0_loss0"),
    "perm_travel": (2, "B", 1.0, {}, "perm", "d0_loss0"),
    "perm_noev": (2, "A", 0.0, {}, "perm", "d0_loss0"),
    "trans_m": (2, "A", 1.0, {}, "trans_m", "d0_loss0"),
    "trans_l": (2, "A", 1.0, {}, "trans_l", "d0_loss0"),
    "intermit": (2, "A", 1.0, {}, "intermit", "d0_loss0"),
    "total": (2, "A", 1.0, {}, "total", "d0_loss0"),
    "beacon_lost": (2, "A", 1.0, {}, "beacon_lost", "d0_loss0"),
    "perm_d1": (2, "A", 1.0, {}, "perm", "d1_loss0"),
    "perm_d3": (2, "A", 1.0, {}, "perm", "d3_loss0"),
    "perm_burst": (2, "A", 1.0, {}, "perm", "d1_burst30"),
    "n3_l1": (3, "A", 1.0, {}, "healthy", "d0_loss0"),
    "n3_l3": (3, "A", 3.0, {}, "healthy", "d0_loss0"),
    "n3_aborts": (3, "A", 1.0, dict(abort=0.3, **NOHINT), "healthy", "d0_loss0"),
    "n4_l3": (4, "A", 3.0, {}, "healthy", "d0_loss0"),
}
# conditions that differ from another only in the link: ORACLE / IMPLICIT are link-free
LINK_DUP = {"perm_d1", "perm_d3", "perm_burst", "total", "beacon_lost"}  # same ORACLE/IMPLICIT as perm
SPLITS = {"dev": list(range(1000, 1012)), "heldout": list(range(4000, 4016))}


def run_condition(name, seeds, eps, budget, with_dev=True):
    from .beacon_robustness import LINKS, Ctx, fault_for, run_b
    from .deviation_comm import make_strategies, run_d
    N, kind, lam, ov, cond, link = CONDS[name]
    out = {}
    if N == 2:
        S = make_strategies()
        for sd in seeds:
            ctx = Ctx(_inst(kind, sd, lam, **ov), S)
            fault = fault_for(cond, ctx.inst.horizon)
            rows = []
            for ep in range(eps):
                ri, pr, lossrate = run_probe(ctx, ep, S, fault, link, budget)
                ro = run_b(ctx, "ORACLE", ep, S, fault, link)
                rh = run_b(ctx, "HEALTH", ep, S, fault, link)
                rd = run_d(ctx, "DEV", ep, S, fault, link) if with_dev else None
                rows.append(dict(imp=int(ri.detected), ora=int(ro.detected), hea=int(rh.detected),
                                 dev=int(rd.detected) if rd else None,
                                 dev_frames=int(rd.msgs_sent) if rd else None, hea_bytes=int(rh.bytes_sent),
                                 hints=int(ri.hints), aborts=int(ri.aborts), lossrate=lossrate,
                                 ttd=int(ri.ttd), **{k: v for k, v in pr.items()}))
            out[sd] = rows
    else:
        from .links import LinkConfig
        from .nagent import nominal_plan_n, run_n
        from .variants import make_all_strategies
        S = make_all_strategies()
        PN = _make_probe_n()
        for sd in seeds:
            inst = _inst(kind, sd, lam, **ov)
            nom = nominal_plan_n(inst, N, S)
            rows = []
            for ep in range(eps):
                e = PN(inst, (S["IMPLICIT"],) * N, LinkConfig(), np.random.default_rng(ep), nom)
                e.budget = budget
                e.probe = _new_probe()
                ri = e.run()
                ro = run_n(inst, N, "ORACLE", LinkConfig(), ep, S, nominal=nom)
                rows.append(dict(imp=int(ri.detected), ora=int(ro.detected), hea=None, dev=None,
                                 dev_frames=None, hea_bytes=None, hints=int(ri.hints),
                                 aborts=int(ri.aborts), lossrate=0.0, ttd=int(ri.ttd), **e.probe))
            out[sd] = rows
    return out


def sim_task(split, eps, budget, names, out, with_dev=True):
    seeds = SPLITS[split]
    res = {}
    if out:
        try:
            res = json.load(open(out))["res"]
        except Exception:
            res = {}
    t0 = time.time()
    for nm in names:
        if nm in res:
            continue
        r = run_condition(nm, seeds, eps, budget, with_dev)
        res[nm] = {str(k): v for k, v in r.items()}
        print(f"[{time.time() - t0:6.0f}s] {split} {nm}: "
              f"imp {np.mean([x['imp'] for v in r.values() for x in v]):.3f} "
              f"ora {np.mean([x['ora'] for v in r.values() for x in v]):.3f} "
              f"CR {np.mean([x['cr'] for v in r.values() for x in v]):.4f}", flush=True)
        if out:
            json.dump(dict(split=split, eps=eps, budget=budget, seeds=seeds, res=res), open(out, "w"))
    return res


# ------------------------------------------------------------------ analysis
def _spearman(a, b):
    a, b = np.asarray(a, float), np.asarray(b, float)
    if np.std(a) == 0 or np.std(b) == 0:
        return float("nan")
    ra = np.argsort(np.argsort(a, kind="stable"), kind="stable").astype(float)
    rb = np.argsort(np.argsort(b, kind="stable"), kind="stable").astype(float)
    # average ranks for ties
    def avg(x, r):
        out = r.copy()
        for v in np.unique(x):
            m = x == v
            out[m] = r[m].mean()
        return out
    ra, rb = avg(a, ra), avg(b, rb)
    return float(np.corrcoef(ra, rb)[0, 1])


FEATS = {
    "CR": lambda x: x["cr"],
    "DRD": lambda x: x["drd"],
    "CR_health": lambda x: x["cr_h"],
    "DRD_health": lambda x: x["drd_h"],
    "events": lambda x: x["hints"] + x["aborts"],
    "lossrate": lambda x: x["lossrate"],
    "dev_frames": lambda x: x["dev_frames"] if x["dev_frames"] is not None else np.nan,
}
TARGETS = {"G_oracle": lambda x: x["ora"] - x["imp"],
           "G_health": lambda x: (x["hea"] - x["imp"]) if x["hea"] is not None else np.nan}
LAM = {nm: c[2] * (1.0 if not c[3] else (0.0 if c[3].get("abort") == 0.0 and False else 1.0))
       for nm, c in CONDS.items()}


def cond_table(res, names, rng=None, idx=None):
    """Condition-level means; ``idx`` = per-condition resampled instance keys."""
    T = {}
    cache = _IM.setdefault(id(res), {})
    for nm in names:
        R = res[nm]
        if nm not in cache:      # per-instance means (equal episodes per instance)
            cache[nm] = {k: {**{f: float(np.nanmean([FEATS[f](x) for x in R[k]])) for f in FEATS},
                             **{g: float(np.nanmean([TARGETS[g](x) for x in R[k]])) for g in TARGETS}}
                         for k in R}
        keys = idx[nm] if idx is not None else sorted(R)
        d = {f: float(np.mean([cache[nm][k][f] for k in keys])) for f in list(FEATS) + list(TARGETS)}
        d["lam"] = CONDS[nm][2]
        T[nm] = d
    return T


_IM: dict = {}


def boot_cond(res, names, stat, B=1000, seed=7):
    """Two-level bootstrap: resample conditions, then instances within each condition."""
    rng = np.random.default_rng(seed)
    vals = []
    for _ in range(B):
        nms = list(rng.choice(names, len(names)))
        idx = {nm: list(rng.choice(sorted(res[nm]), len(res[nm]))) for nm in sorted(set(nms))}
        T = cond_table(res, sorted(set(nms)), idx=idx)
        v = stat([T[n] for n in nms])
        if v is not None and not np.isnan(v):
            vals.append(v)
    if not vals:
        return (float("nan"),) * 2
    return float(np.percentile(vals, 2.5)), float(np.percentile(vals, 97.5))


def report(dev_path, ho_path, out, fig=None):
    D = json.load(open(dev_path))["res"]
    H = json.load(open(ho_path))["res"]
    L = []
    common = [n for n in CONDS if n in D and n in H]
    ora_names = [n for n in common if n not in LINK_DUP]
    hea_names = [n for n in common if CONDS[n][0] == 2]
    TD, TH = cond_table(D, common), cond_table(H, common)

    L.append("## Condition-level table (held-out seeds 4000-4015)\n")
    L.append("| condition | N | lam | fault | link | IMP | G_oracle | G_health | DRD/ep | CR/ep | CR_health/ep | events/ep | lossrate | DEV frames/ep |")
    L.append("|---|---|---|---|---|---|---|---|---|---|---|---|---|---|")
    for nm in common:
        c = CONDS[nm]; t = TH[nm]
        imp = np.mean([x["imp"] for v in H[nm].values() for x in v])
        L.append(f"| {nm} | {c[0]} | {c[2]:g} | {c[4]} | {c[5]} | {imp:.3f} | {t['G_oracle']:+.4f} | "
                 f"{t['G_health']:+.4f} | {t['DRD']:.3f} | {t['CR']:.4f} | {t['CR_health']:+.4f} | "
                 f"{t['events']:.2f} | {t['lossrate']:.3f} | {t['dev_frames']:.2f} |")

    def corr_stat(f, g, kind="spearman"):
        def s(rows):
            a = [r[f] for r in rows]; b = [r[g] for r in rows]
            if kind == "spearman":
                return _spearman(a, b)
            return float(np.corrcoef(a, b)[0, 1]) if np.std(a) > 0 and np.std(b) > 0 else float("nan")
        return s

    summary = {}
    L.append("\n## Condition-level predictiveness (held-out; 95% two-level bootstrap CIs, B=1000)\n")
    for g, names in (("G_oracle", ora_names), ("G_health", hea_names)):
        L.append(f"\n### Target {g} ({len(names)} conditions)\n")
        L.append("| predictor | Spearman [CI] | Pearson [CI] | dev-fit slope b (G = b x) | held-out calib. slope [CI] | held-out MAE | MAE of mean-only |")
        L.append("|---|---|---|---|---|---|---|")
        feats = ["CR", "DRD", "events", "lam", "lossrate", "dev_frames"]
        if g == "G_health":
            feats = ["CR_health", "DRD_health", "CR"] + feats[1:]
        for f in feats:
            rows = [TH[n] for n in names]
            if any(np.isnan(r[f]) for r in rows):
                continue
            sp = corr_stat(f, g)(rows)
            pe = corr_stat(f, g, "pearson")(rows)
            sci = boot_cond(H, names, corr_stat(f, g))
            pci = boot_cond(H, names, corr_stat(f, g, "pearson"))
            # calibration: fit G = b * x through the origin on dev, evaluate on held-out
            xd = np.array([TD[n][f] for n in names]); yd = np.array([TD[n][g] for n in names])
            b = float((xd @ yd) / max(xd @ xd, 1e-300))
            xh = np.array([TH[n][f] for n in names]); yh = np.array([TH[n][g] for n in names])
            pred = b * xh
            slope = float((pred @ yh) / max(pred @ pred, 1e-300))

            def cal(rows, b=b, f=f, g=g):
                p = np.array([r[f] for r in rows]) * b; y = np.array([r[g] for r in rows])
                return float((p @ y) / max(p @ p, 1e-300))
            cci = boot_cond(H, names, cal)
            mae = float(np.mean(np.abs(pred - yh)))
            mae0 = float(np.mean(np.abs(yd.mean() - yh)))
            summary[(g, f)] = dict(sp=sp, sci=sci, pe=pe, pci=pci, b=b, slope=slope, cci=cci, mae=mae, mae0=mae0)
            L.append(f"| {f} | {sp:+.2f} [{sci[0]:+.2f},{sci[1]:+.2f}] | {pe:+.2f} [{pci[0]:+.2f},{pci[1]:+.2f}] | "
                     f"{b:.3g} | {slope:.2f} [{cci[0]:.2f},{cci[1]:.2f}] | {mae:.4f} | {mae0:.4f} |")

    # -------------------------------------------------------------- episode level
    L.append("\n## Episode-level: cumulative CR vs paired episode gain (held-out)\n")
    L.append("Pooled over conditions; Spearman within condition (ranks), then averaged, and pooled AUC "
             "for P(gain=+1) vs P(gain=-1) episodes.  CI: bootstrap over (condition, instance) clusters.\n")
    L.append("| target | predictor | mean within-cond Spearman [CI] | AUC(+1 vs -1) [CI] | P(gain>0 or <0) when predictor = 0 | P(gain != 0) when predictor > 0 |")
    L.append("|---|---|---|---|---|---|")
    rng = np.random.default_rng(11)
    for g, names in (("G_oracle", ora_names), ("G_health", hea_names)):
        for f in (["CR", "DRD", "events"] if g == "G_oracle" else ["CR_health", "CR", "events"]):
            clusters = [(nm, k) for nm in names for k in sorted(H[nm])]

            def stats(cl):
                byc = {}
                for nm, k in cl:
                    byc.setdefault(nm, []).extend(H[nm][k])
                sps, pos, neg = [], [], []
                for nm, rows in byc.items():
                    x = np.array([FEATS[f](r) for r in rows], float)
                    y = np.array([TARGETS[g](r) for r in rows], float)
                    s = _spearman(x, y)
                    if not np.isnan(s):
                        sps.append(s)
                    # within-condition centring for pooled AUC
                    pos += list(x[y > 0]); neg += list(x[y < 0])
                auc = float("nan")
                if pos and neg:
                    P, Nn = np.array(pos), np.array(neg)
                    auc = float(((P[:, None] > Nn[None]).mean() + 0.5 * (P[:, None] == Nn[None]).mean()))
                return (np.mean(sps) if sps else float("nan")), auc
            sp, auc = stats(clusters)
            bs = [stats([clusters[i] for i in rng.integers(len(clusters), size=len(clusters))]) for _ in range(300)]
            bsp = np.nanpercentile([b[0] for b in bs], [2.5, 97.5]); bau = np.nanpercentile([b[1] for b in bs], [2.5, 97.5])
            allr = [r for nm in names for v in H[nm].values() for r in v]
            x = np.array([FEATS[f](r) for r in allr], float); y = np.array([TARGETS[g](r) for r in allr], float)
            z = x == 0 if f != "CR_health" else np.abs(x) < 1e-12
            L.append(f"| {g} | {f} | {sp:+.3f} [{bsp[0]:+.3f},{bsp[1]:+.3f}] | {auc:.3f} [{bau[0]:.3f},{bau[1]:.3f}] | "
                     f"{np.mean(y[z] != 0):.3f} (n={z.sum()}) | {np.mean(y[~z] != 0):.3f} (n={(~z).sum()}) |")
            summary[("ep", g, f)] = dict(sp=sp, bsp=bsp.tolist(), auc=auc, bau=bau.tolist())

    # -------------------------------------------------- CR-trigger offline analysis
    L.append("\n## A CR-threshold trigger, offline (held-out, all N=2 conditions)\n")
    L.append("Each DRD event is one decision where the full partner state would change the receiver's argmax "
             "(what DEV's info trigger approximates from common ground).  Keeping only events with CR > c:\n")
    L.append("| c | share of DRD events kept | share of total CR kept |")
    L.append("|---|---|---|")
    ev = np.array([e[2] for nm in hea_names for v in H[nm].values() for r in v for e in r["events"]], float)
    ev = np.maximum(ev, 0.0)
    for c in (0.0, 0.005, 0.01, 0.02, 0.05, 0.1):
        k = ev > c
        L.append(f"| {c:g} | {k.mean():.3f} | {ev[k].sum() / max(ev.sum(), 1e-300):.3f} |")
    # timing of CR in loss conditions
    L.append("\n## Where CR sits in time under permanent loss (held-out)\n")
    L.append("| condition | CR before/at failure | CR within 2 steps after failure | CR later |")
    L.append("|---|---|---|---|")
    for nm in ("perm", "perm_early", "perm_late", "perm_noev", "perm_travel"):
        if nm not in H:
            continue
        rows = [r for v in H[nm].values() for r in v]
        pre = np.mean([r["cr_pre"] for r in rows]); fl = np.mean([r["cr_fail"] for r in rows])
        tot = np.mean([r["cr"] for r in rows])
        L.append(f"| {nm} | {pre:.4f} | {fl:.4f} | {tot - pre - fl:.4f} |")
    txt = "\n".join(L) + "\n"
    if out:
        open(out, "w").write(txt)
    print(txt)
    if fig:
        plot(TH, ora_names, hea_names, summary, fig)
    return summary


def plot(TH, ora_names, hea_names, summary, out):
    import os
    os.environ["PATH"] = "/home/lunarz/.cache/research/tinytex/.TinyTeX/bin/x86_64-linux:" + os.environ["PATH"]
    import matplotlib
    matplotlib.use("pdf")
    import matplotlib.pyplot as plt
    plt.rcParams.update({"text.usetex": True, "font.family": "serif", "font.size": 8,
                         "pdf.fonttype": 42, "axes.spines.top": False, "axes.spines.right": False})
    fig, ax = plt.subplots(1, 3, figsize=(7.0, 2.0), constrained_layout=True)
    col = {"healthy": "#4c72b0", "loss": "#c44e52", "trans": "#dd8452", "N3": "#55a868"}

    def grp(nm):
        c = CONDS[nm]
        if c[0] > 2:
            return "N3"
        if c[4] == "healthy":
            return "healthy"
        if c[4] in ("trans_m", "trans_l", "intermit"):
            return "trans"
        return "loss"
    for a, (f, g, names, xl, yl) in zip(ax, (
            ("CR", "G_oracle", ora_names, r"mean $\sum_t \mathrm{CR}_t$ per episode", r"$P_{\textsc{oracle}}-P_{\textsc{implicit}}$"),
            ("CR_health", "G_health", hea_names, r"mean $\sum_t \mathrm{CR}^{\mathrm{beacon}}_t$", r"$P_{\textsc{health}}-P_{\textsc{implicit}}$"),
            ("events", "G_oracle", ora_names, r"private events per episode", r"$P_{\textsc{oracle}}-P_{\textsc{implicit}}$"))):
        for nm in names:
            a.scatter(TH[nm][f], TH[nm][g], s=14, color=col[grp(nm)], zorder=3, lw=0)
        s = summary.get((g, f))
        if s:
            xs = np.linspace(min(0, min(TH[n][f] for n in names)), max(TH[n][f] for n in names), 10)
            a.plot(xs, s["b"] * xs, color="0.5", lw=0.8, ls="--", zorder=1)
            a.set_title(rf"Spearman ${s['sp']:+.2f}$ [{s['sci'][0]:+.2f}, {s['sci'][1]:+.2f}]", fontsize=7.5)
        a.axhline(0, color="0.8", lw=0.6, zorder=0)
        a.set_xlabel(xl); a.set_ylabel(yl)
    for k, c in col.items():
        ax[0].scatter([], [], color=c, s=14, label={"healthy": "healthy", "loss": "perm./total loss",
                                                    "trans": "transient", "N3": "$N{\\geq}3$"}[k])
    ax[0].legend(frameon=False, fontsize=6.5, loc="upper left", handletextpad=0.2)
    fig.savefig(out)
    print("wrote", out)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("task", choices=["exact", "sim", "report"])
    ap.add_argument("--split", choices=list(SPLITS), default="dev")
    ap.add_argument("--episodes", type=int, default=60)
    ap.add_argument("--budget", type=int, default=6)
    ap.add_argument("--names", nargs="+", default=list(CONDS))
    ap.add_argument("--no-dev", action="store_true")
    ap.add_argument("--part", default="all", choices=["all", "death"])
    ap.add_argument("--dev", default=None)
    ap.add_argument("--heldout", default=None)
    ap.add_argument("--fig", default=None)
    ap.add_argument("--out", default=None)
    a = ap.parse_args()
    if a.task == "exact":
        exact_task(a.out, a.part)
    elif a.task == "sim":
        sim_task(a.split, a.episodes, a.budget, a.names, a.out, not a.no_dev)
    else:
        report(a.dev, a.heldout, a.out, a.fig)


if __name__ == "__main__":
    main()
