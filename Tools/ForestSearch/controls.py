"""Mechanism controls for the stale-plan collision claim (NOVELTY2.md).

HYBRID_O : HYBRID, but the partner's in-flight action is read from the true
           partner state (an oracle for the current action only; counts are
           unchanged).  The prediction is that it matches HYBRID_P.
HYBRID_S : HYBRID with a random symmetry-breaking salt (a fixed private
           per-agent score perturbation, +/-10%) instead of priority
           prediction.  The prediction is that it removes most collisions
           but does worse than HYBRID_P.
"""
from __future__ import annotations

from dataclasses import dataclass

import numpy as np

from .variants import HEpisode, HStrategy, make_all_strategies


@dataclass(frozen=True)
class CStrategy(HStrategy):
    oracle_inflight: bool = False
    salt: float = 0.0


class CEpisode(HEpisode):
    def _partner_counts(self, a, t):
        m0, m1, virtual = super()._partner_counts(a, t)
        if getattr(a.s, "oracle_inflight", False):
            p = self.agents[1 - a.idx]
            virtual = []
            if p.task is not None and (p.task.start, p.idx) < (t, a.idx):
                virtual = [(p.task.cell, p.task.view)]
        return m0, m1, virtual


def make_control_strategies() -> dict:
    S = make_all_strategies()
    h = S["HYBRID"]
    f = {k: getattr(h, k) for k in h.__dataclass_fields__}
    f["name"] = "HYBRID_O"
    S["HYBRID_O"] = CStrategy(**f, oracle_inflight=True)
    f["name"] = "HYBRID_S"
    S["HYBRID_S"] = CStrategy(**f, salt=0.1)
    return S


def run_ctrl(inst, pair, link, seed, S, nominal, sectors, keep_log=False):
    ep = CEpisode(inst, (S[pair[0]], S[pair[1]]), link, np.random.default_rng(seed),
                  nominal, sectors, keep_log=keep_log)
    r = ep.run()
    coll = 0
    if keep_log:
        by_t = {}
        for (tt, ag, c, v, out) in ep.log:
            by_t.setdefault(tt, {})[ag] = (c, v)
        coll = sum(1 for d in by_t.values() if len(d) == 2 and d[0] == d[1])
    return r, coll


# ---------------------------------------------------------------------------
# Phantom-look hypothesis: after a known deviation, HYBRID counts the
# partner's *nominal* looks for slots whose records have not arrived yet.
# Those are looks at cells the partner never searched.
#   no_phantom : after deviation, unconfirmed nominal slots are not counted
#   inflight   : 'nominal' (HYBRID), 'predict' (greedy, agent 1 only),
#                'oracle', or 'none'
# ---------------------------------------------------------------------------
from .model import HINT, NULL, detect_prob  # noqa: E402


@dataclass(frozen=True)
class PStrategy(HStrategy):
    no_phantom: bool = False
    inflight: str = "nominal"


class PEpisode(HEpisode):
    def _partner_counts(self, a, t):
        s = a.s
        if not isinstance(s, PStrategy) or self.oracle or self.nominal is None:
            return super()._partner_counts(a, t)
        inst, p = self.inst, self.agents[1 - a.idx]
        m0 = np.zeros((inst.n, inst.V), int)
        m1 = np.zeros_like(m0)
        virtual, seen = [], set()
        nom_by_seq = {tk.seq: tk for tk in self.nominal[p.idx]}
        dev_seqs = [sq for sq, r in a.p_records.items()
                    if r.outcome != NULL or sq not in nom_by_seq
                    or (nom_by_seq[sq].cell, nom_by_seq[sq].view) != (r.cell, r.view)]
        first_dev = min(dev_seqs) if dev_seqs else None

        def add(r):
            if r.outcome == NULL:
                m0[r.cell, r.view] += 1
            elif r.outcome == HINT:
                m1[r.cell, r.view] += 1
        inflight_guess = None
        for tk in self.nominal[p.idx]:
            if tk.seq in a.p_records:
                add(a.p_records[tk.seq]); seen.add(tk.seq); continue
            if tk.seq in a.p_leases:
                c, v, e = a.p_leases[tk.seq]
                if t < e - s.lease_slack or (t == e - s.lease_slack and p.idx > a.idx and not getattr(s, 'lease_fix', False)):
                    virtual.append((c, v))
                else:
                    m0[c, v] += 1
                seen.add(tk.seq); continue
            after_dev = first_dev is not None and tk.seq > first_dev
            if tk.finish <= t:
                if not (s.no_phantom and after_dev):
                    m0[tk.cell, tk.view] += 1
                seen.add(tk.seq)
            elif (tk.start, p.idx) < (t, a.idx):
                inflight_guess = (tk.cell, tk.view); seen.add(tk.seq)
            else:
                break
        for sq, r in a.p_records.items():
            if sq not in seen:
                seen.add(sq); add(r)
        for sq, (c, v, e) in a.p_leases.items():
            if sq not in seen:
                if t < e - s.lease_slack or (t == e - s.lease_slack and p.idx > a.idx and not getattr(s, 'lease_fix', False)):
                    virtual.append((c, v))
                else:
                    m0[c, v] += 1
        if not virtual:
            mode = s.inflight
            if mode == "oracle":
                if p.task is not None and (p.task.start, p.idx) < (t, a.idx):
                    virtual.append((p.task.cell, p.task.view))
            elif mode == "predict" and first_dev is not None and a.idx == 1 and inflight_guess is not None:
                pos = self._partner_pos_guess(a)
                pd = detect_prob(inst, a.m0 + m0, a.m1 + m1, s.belief)
                sc = pd / (inst.dist[pos][:, None] + 1.0)
                virtual.append(divmod(int(np.argmax(sc)), inst.V))
            elif mode in ("nominal", "predict") and inflight_guess is not None:
                if not (s.no_phantom and first_dev is not None and mode == "nominal"):
                    virtual.append(inflight_guess)
        return m0, m1, virtual


def make_phantom_strategies() -> dict:
    S = make_control_strategies()
    h = S["HYBRID"]
    f = {k: getattr(h, k) for k in h.__dataclass_fields__}
    for name, nph, inf in [("PH_nom", False, "nominal"), ("PH_pred", False, "predict"),
                           ("NP_none", True, "none"), ("NP_nom", True, "nominal"),
                           ("NP_orc", True, "oracle"), ("NP_pred", True, "predict"),
                           ("PH_orc", False, "oracle")]:
        f["name"] = name
        S[name] = PStrategy(**f, no_phantom=nph, inflight=inf)
    return S


def run_ph(inst, pair, link, seed, S, nominal, sectors, keep_log=False):
    ep = PEpisode(inst, (S[pair[0]], S[pair[1]]), link, np.random.default_rng(seed),
                  nominal, sectors, keep_log=keep_log)
    r = ep.run()
    coll = 0
    if keep_log:
        by_t = {}
        for (tt, ag, c, v, out) in ep.log:
            by_t.setdefault(tt, {})[ag] = (c, v)
        coll = sum(1 for d in by_t.values() if len(d) == 2 and d[0] == d[1])
    return r, coll


# Lease-timing fix: a leased look is complete at its expected finish for both
# agents (completions are processed before decisions), so the index tie rule
# inherited from HEpisode is dropped.
@dataclass(frozen=True)
class FStrategy(PStrategy):
    lease_fix: bool = True


def make_fixed_strategies() -> dict:
    S = make_phantom_strategies()
    h = S["HYBRID"]
    f = {k: getattr(h, k) for k in h.__dataclass_fields__}
    for name, inf in [("FX_nom", "nominal"), ("FX_pred", "predict"), ("FX_orc", "oracle")]:
        f["name"] = name
        S[name] = FStrategy(**f, no_phantom=False, inflight=inf)
    return S
