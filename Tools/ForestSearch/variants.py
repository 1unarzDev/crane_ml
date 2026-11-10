"""Herding ablation: explicit protocols with a symmetry-breaking fallback.

Hypothesis under test: explicit-state protocols (RESERVE/FULL/EVENT) collapse
under loss because two deterministic planners with near-identical beliefs pick
the same task whenever partner information is missing.  The fix tested here
is a *role convention*: agent 0 has priority; when agent 1 has no fresh
knowledge of agent 0's in-flight task (no unexpired lease, no nominal), it
predicts agent 0's greedy choice from its own belief and avoids it.

Also provides SALT variants: each agent breaks ties/near-ties with a private,
fixed per-cell preference (cheap randomised symmetry breaking, no roles).
"""
from __future__ import annotations

from dataclasses import dataclass, replace

import numpy as np

from .model import detect_prob
from .sim import Episode, STRATEGIES, Strategy, make_strategies


@dataclass(frozen=True)
class VStrategy(Strategy):
    role_predict: bool = False
    salt: float = 0.0        # relative score perturbation amplitude


class VEpisode(Episode):
    def _partner_counts(self, a, t):
        m0, m1, virtual = super()._partner_counts(a, t)
        s = a.s
        if getattr(s, "role_predict", False) and a.idx == 1 and not self.oracle:
            p = self.agents[0]
            known_inflight = any(
                exp >= t for (_, _, exp) in a.p_leases.values())
            if not virtual and not known_inflight:
                # Predict partner's greedy choice from my own belief.
                pos = self._partner_pos_guess(a)
                mm0 = a.m0 + m0
                mm1 = a.m1 + m1
                pd = detect_prob(self.inst, mm0, mm1, s.belief)
                sc = pd / (self.inst.dist[pos][:, None] + 1.0)
                c, v = divmod(int(np.argmax(sc)), self.inst.V)
                virtual = virtual + [(c, v)]
        return m0, m1, virtual

    def _partner_pos_guess(self, a):
        if a.p_records:
            r = max(a.p_records.values(), key=lambda r: r.finish)
            return r.cell
        return self.inst.starts[1 - a.idx]

    def _choose(self, a, t):
        s = a.s
        if getattr(s, "salt", 0.0) <= 0:
            return super()._choose(a, t)
        # Salted greedy: multiply scores by a fixed private per-(cell, view)
        # factor in [1 - salt, 1 + salt].
        inst = self.inst
        pm0, pm1, virtual = self._partner_counts(a, t)
        m0 = a.m0 + pm0
        m1 = a.m1 + pm1
        for (c, v) in virtual:
            m0[c, v] += 1
        pd = detect_prob(inst, m0, m1, s.belief)
        score = pd / (inst.dist[a.pos][:, None] + 1.0)
        if not hasattr(self, "_salt"):
            rng = np.random.default_rng(12345)
            self._salt = 1.0 + s.salt * (2 * rng.random((2, inst.n, inst.V)) - 1)
        score = score * self._salt[a.idx]
        return divmod(int(np.argmax(score)), inst.V)


def make_variant_strategies(**planner_kw) -> dict:
    base = make_strategies(**planner_kw)
    out = dict(base)
    for k in ("INDEP", "RESERVE", "FULL", "EVENT"):
        b = base[k]
        fields = {f: getattr(b, f) for f in b.__dataclass_fields__}
        fields["name"] = k + "_R"
        out[k + "_R"] = VStrategy(**fields, role_predict=True)
        fields["name"] = k + "_S"
        out[k + "_S"] = VStrategy(**fields, salt=0.1)
    return out


def run_variant_episode(inst, pair, link, seed, strategies, nominal=None, sectors=None):
    from .sim import make_sectors, nominal_plan
    strats = (strategies[pair[0]], strategies[pair[1]])
    if nominal is None and any(s.predict_nominal for s in strats):
        nominal = nominal_plan(inst, strats[0])
    if sectors is None and any(s.planner == "sector" for s in strats):
        sectors = make_sectors(inst)
    rng = np.random.default_rng(seed)
    return VEpisode(inst, strats, link, rng, nominal, sectors).run()


# --------------------------------------------------------------------------
# HYBRID: messages layered on the shared default plan (reviewer proposal,
# REVIEW_skeptic.md).  Partner looks are taken from received records where
# available, else from leases, else from the nominal plan (aligned by task
# sequence number).  HYBRID_P additionally replaces the nominal guess for the
# partner's in-flight slot with a role-based greedy prediction once the
# partner is known to have deviated (agent 1 predicts agent 0).
# --------------------------------------------------------------------------
from .model import HINT, NULL  # noqa: E402


@dataclass(frozen=True)
class HStrategy(Strategy):
    hybrid: bool = True
    predict_after_dev: bool = False


class HEpisode(VEpisode):
    def _partner_counts(self, a, t):
        s = a.s
        if not getattr(s, "hybrid", False) or self.oracle or self.nominal is None:
            return super()._partner_counts(a, t)
        inst = self.inst
        p = self.agents[1 - a.idx]
        m0 = np.zeros((inst.n, inst.V), int)
        m1 = np.zeros_like(m0)
        virtual, seen = [], set()

        def add(r):
            if r.outcome == NULL:
                m0[r.cell, r.view] += 1
            elif r.outcome == HINT:
                m1[r.cell, r.view] += 1

        nom_by_seq = {tk.seq: tk for tk in self.nominal[p.idx]}
        deviated = any(r.outcome != NULL or seq not in nom_by_seq or
                       (nom_by_seq[seq].cell, nom_by_seq[seq].view) != (r.cell, r.view)
                       for seq, r in a.p_records.items())
        inflight_guess = None
        for tk in self.nominal[p.idx]:
            if tk.seq in a.p_records:
                add(a.p_records[tk.seq]); seen.add(tk.seq); continue
            if tk.seq in a.p_leases:
                c, v, e = a.p_leases[tk.seq]
                # Default: a leased look is done at its expected finish.
                if t < e - s.lease_slack or (t == e - s.lease_slack and p.idx > a.idx):
                    virtual.append((c, v))
                else:
                    m0[c, v] += 1
                seen.add(tk.seq); continue
            if tk.finish <= t:
                m0[tk.cell, tk.view] += 1; seen.add(tk.seq)
            elif (tk.start, p.idx) < (t, a.idx):
                inflight_guess = (tk.cell, tk.view); seen.add(tk.seq)
            else:
                break
        for sq, r in a.p_records.items():
            if sq not in seen:
                seen.add(sq); add(r)
        for sq, (c, v, e) in a.p_leases.items():
            if sq not in seen:
                if t < e - s.lease_slack or (t == e - s.lease_slack and p.idx > a.idx):
                    virtual.append((c, v))
                else:
                    m0[c, v] += 1
        if inflight_guess is not None:
            if s.predict_after_dev and deviated and a.idx == 1 and not virtual:
                pos = self._partner_pos_guess(a)
                pd = detect_prob(inst, a.m0 + m0, a.m1 + m1, s.belief)
                sc = pd / (inst.dist[pos][:, None] + 1.0)
                inflight_guess = divmod(int(np.argmax(sc)), inst.V)
            if not virtual:
                virtual.append(inflight_guess)
        return m0, m1, virtual


def make_all_strategies(**planner_kw) -> dict:
    out = make_variant_strategies(**planner_kw)
    base = out["FULL"]
    fields = {f: getattr(base, f) for f in base.__dataclass_fields__}
    fields.update(name="HYBRID", predict_nominal=True)
    out["HYBRID"] = HStrategy(**fields)
    fields["name"] = "HYBRID_P"
    out["HYBRID_P"] = HStrategy(**fields, predict_after_dev=True)
    return out


def run_any(inst, pair, link, seed, strategies, nominal=None, sectors=None):
    from .sim import make_sectors, nominal_plan
    strats = (strategies[pair[0]], strategies[pair[1]])
    if nominal is None:
        nominal = nominal_plan(inst, strats[0])
    if sectors is None:
        sectors = make_sectors(inst)
    return HEpisode(inst, strats, link, np.random.default_rng(seed), nominal, sectors).run()
