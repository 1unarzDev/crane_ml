"""N-agent (N >= 2) episode for ORACLE, INDEP, IMPLICIT, HYBRID, HYBRID_P.

Semantics are identical to sim.py / variants.py at N = 2 (same RNG draw order,
same step structure, same tie-breaks); tested episode-for-episode.

Differences needed for N > 2 (all reduce to the 2-agent code at N = 2):
  * One DirectedLink per ordered pair (i -> j); a broadcast is N-1 independent
    sends.  Links are drawn from the rng in (i, j) lexicographic order, which
    at N = 2 is link[0->1], link[1->0], as in sim.Episode.
  * Per-source partner ledgers (records / leases).
  * Role rule for HYBRID_P: fixed priority = agent index.  Agent k replaces
    the stale nominal in-flight guess of every higher-priority agent j < k that
    is known to have deviated (and has no fresh lease in flight) by sequential
    greedy on k's own belief: j = 0, 1, ... each predicted choice is added as a
    virtual NULL look before predicting the next one.  Lower-priority agents
    keep the nominal guess.  At N = 2 this is exactly "agent 1 predicts 0".

collisions = number of same-step task starts on an identical (cell, view),
counted as pairs.
"""
from __future__ import annotations

from dataclasses import dataclass, replace

import numpy as np

from .links import DirectedLink, LinkConfig
from .model import (ABORT, DETECT, HINT, NULL, Instance, Truth, detect_prob,
                    sample_outcome, sample_truth)
from .sim import HEADER_B, LEASE_B, RECORD_B, Message, Record, Task
from .variants import make_all_strategies


def starts_for(inst: Instance, N: int) -> tuple:
    """Start cells: inst.starts at N = 2, else spread over grid corners."""
    if N == 2:
        return tuple(inst.starts)
    n = inst.n
    w = int(inst.coords[:, 0].max()) + 1 if inst.coords is not None else n
    corners = [0, n - 1, w - 1, n - w]
    if N > 4:
        raise ValueError("N <= 4 supported")
    return tuple(corners[:N])


@dataclass
class NResult:
    detected: bool
    ttd: int
    looks: int
    redundant: int
    aborts: int
    hints: int
    bytes_sent: int
    msgs_sent: int
    msgs_delivered: int
    collisions: int


class _A:
    def __init__(self, idx, strat, inst, pos, N):
        self.idx, self.s, self.pos = idx, strat, pos
        self.task = None
        self.seq = 0
        self.records: list[Record] = []
        self.p_records = {j: {} for j in range(N) if j != idx}
        self.p_leases = {j: {} for j in range(N) if j != idx}
        self.explicit = strat.send in ("full", "reserve")
        self.m0 = np.zeros((inst.n, inst.V), int)
        self.m1 = np.zeros((inst.n, inst.V), int)


class NEpisode:
    def __init__(self, inst, strats, link, rng, nominal=None, force_null=False,
                 keep_log=False):
        self.inst = inst
        self.N = N = len(strats)
        self.force_null = force_null
        self.truth: Truth = sample_truth(inst, rng)
        self.u = rng.random((N, inst.horizon + 1, 3))
        self.links = {}
        for i in range(N):
            for j in range(N):
                if i != j:
                    self.links[(i, j)] = DirectedLink(link, inst.horizon, rng)
        self.nominal = nominal
        st = starts_for(inst, N)
        self.agents = [_A(i, strats[i], inst, st[i], N) for i in range(N)]
        self.oracle = any(s.planner == "oracle" for s in strats)
        self.log = [] if keep_log else None
        self.bytes = self.msgs = self.delivered = 0
        self.looked_by = [set() for _ in range(N)]
        self._framed = set()
        self.collisions = 0

    # ------------------------------------------------------------- belief
    def _zeros(self):
        z = np.zeros((self.inst.n, self.inst.V), int)
        return z, z.copy()

    def _partner_pos_guess(self, a, j):
        recs = a.p_records[j]
        if recs:
            return max(recs.values(), key=lambda r: r.finish).cell
        return starts_for(self.inst, self.N)[j]

    @staticmethod
    def _add(m0, m1, r):
        if r.outcome == NULL:
            m0[r.cell, r.view] += 1
        elif r.outcome == HINT:
            m1[r.cell, r.view] += 1

    def _partner_counts(self, a, t):
        m0, m1 = self._zeros()
        virtual = []
        if self.oracle:
            for p in self.agents:
                if p.idx == a.idx:
                    continue
                for r in p.records:
                    self._add(m0, m1, r)
                if p.task is not None:
                    virtual.append((p.task.cell, p.task.view))
            return m0, m1, virtual
        s = a.s
        if getattr(s, "hybrid", False) and self.nominal is not None:
            return self._hybrid_counts(a, t)
        if s.send == "none" and not s.predict_nominal:
            return m0, m1, virtual
        # IMPLICIT: nominal plan of every other agent.
        for p in self.agents:
            if p.idx == a.idx or not (s.predict_nominal and self.nominal is not None):
                continue
            for tk in self.nominal[p.idx]:
                if tk.finish <= t:
                    m0[tk.cell, tk.view] += 1
                elif (tk.start, p.idx) < (t, a.idx):
                    virtual.append((tk.cell, tk.view))
                else:
                    break
        return m0, m1, virtual

    def _hybrid_counts(self, a, t):
        s, inst = a.s, self.inst
        M0, M1 = self._zeros()
        virtual = []
        slots = []   # (j, nominal in-flight guess, deviated) awaiting role rule
        for p in self.agents:
            if p.idx == a.idx:
                continue
            j = p.idx
            recs, leases = a.p_records[j], a.p_leases[j]
            m0, m1 = self._zeros()
            pv, seen = [], set()
            nom_by_seq = {tk.seq: tk for tk in self.nominal[j]}
            deviated = any(r.outcome != NULL or seq not in nom_by_seq or
                           (nom_by_seq[seq].cell, nom_by_seq[seq].view) != (r.cell, r.view)
                           for seq, r in recs.items())
            guess = None
            for tk in self.nominal[j]:
                if tk.seq in recs:
                    self._add(m0, m1, recs[tk.seq]); seen.add(tk.seq); continue
                if tk.seq in leases:
                    c, v, e = leases[tk.seq]
                    if t < e - s.lease_slack or (t == e - s.lease_slack and j > a.idx):
                        pv.append((c, v))
                    else:
                        m0[c, v] += 1
                    seen.add(tk.seq); continue
                if tk.finish <= t:
                    m0[tk.cell, tk.view] += 1; seen.add(tk.seq)
                elif (tk.start, j) < (t, a.idx):
                    guess = (tk.cell, tk.view); seen.add(tk.seq)
                else:
                    break
            for sq, r in recs.items():
                if sq not in seen:
                    seen.add(sq); self._add(m0, m1, r)
            for sq, (c, v, e) in leases.items():
                if sq not in seen:
                    if t < e - s.lease_slack or (t == e - s.lease_slack and j > a.idx):
                        pv.append((c, v))
                    else:
                        m0[c, v] += 1
            M0 += m0; M1 += m1
            if guess is not None and not pv:
                if s.predict_after_dev and deviated and j < a.idx:
                    slots.append((j, len(virtual)))   # placeholder position
                    virtual.append(None)
                else:
                    virtual.append(guess)
            virtual.extend(pv)
        if slots:
            # Role rule: sequential greedy over higher-priority deviated agents.
            for (j, pos_in_list) in slots:
                mm0 = a.m0 + M0
                mm1 = a.m1 + M1
                for vv in virtual:
                    if vv is not None:
                        mm0[vv[0], vv[1]] += 1
                pd = detect_prob(inst, mm0, mm1, s.belief)
                pos = self._partner_pos_guess(a, j)
                sc = pd / (inst.dist[pos][:, None] + 1.0)
                virtual[pos_in_list] = divmod(int(np.argmax(sc)), inst.V)
        virtual = [x for x in virtual if x is not None]
        return M0, M1, virtual

    def _choose(self, a, t):
        inst = self.inst
        pm0, pm1, virtual = self._partner_counts(a, t)
        m0 = a.m0 + pm0
        m1 = a.m1 + pm1
        for (c, v) in virtual:
            m0[c, v] += 1
        pd = detect_prob(inst, m0, m1, a.s.belief)
        score = pd / (inst.dist[a.pos][:, None] + 1.0)
        return divmod(int(np.argmax(score)), inst.V)

    # ---------------------------------------------------------- messaging
    def _send(self, a, t, records, lease):
        msg = Message(a.idx, t, 0, records, lease)
        for j in range(self.N):
            if j == a.idx:
                continue
            nb = msg.nbytes()
            if (a.idx, j, t) in self._framed:
                nb -= HEADER_B
            else:
                self._framed.add((a.idx, j, t))
                self.msgs += 1
            self.bytes += nb
            if self.links[(a.idx, j)].send(t, msg):
                self.delivered += 1

    def _deliver(self, t):
        for a in self.agents:
            for src in range(self.N):
                if src == a.idx:
                    continue
                for msg in self.links[(src, a.idx)].receive(t):
                    for r in msg.records:
                        a.p_records[src][r.seq] = r
                        a.p_leases[src].pop(r.seq, None)
                    if msg.lease is not None:
                        seq, c, v, exp = msg.lease
                        if seq not in a.p_records[src]:
                            a.p_leases[src][seq] = (c, v, exp)

    def _lease(self, a):
        if a.task is None:
            return None
        return (a.task.seq, a.task.cell, a.task.view, a.task.finish + a.s.lease_slack)

    # --------------------------------------------------------------- run
    def run(self) -> NResult:
        inst = self.inst
        looks = redundant = aborts = hints = 0
        for t in range(inst.horizon + 1):
            self._deliver(t)
            for a in self.agents:
                if a.task is None or a.task.finish != t:
                    continue
                tk = a.task
                out = NULL if self.force_null else sample_outcome(
                    inst, self.truth, tk.cell, tk.view, self.u[a.idx, min(tk.seq, inst.horizon)])
                rec = Record(tk.seq, tk.cell, tk.view, out, t)
                a.records.append(rec)
                a.task = None
                a.pos = tk.cell
                if out == ABORT:
                    aborts += 1
                else:
                    looks += 1
                    if any((tk.cell, tk.view) in self.looked_by[j]
                           for j in range(self.N) if j != a.idx):
                        redundant += 1
                    self.looked_by[a.idx].add((tk.cell, tk.view))
                if out == NULL:
                    a.m0[tk.cell, tk.view] += 1
                elif out == HINT:
                    a.m1[tk.cell, tk.view] += 1
                    hints += 1
                if self.log is not None:
                    self.log.append((t, a.idx, tk.cell, tk.view, out))
                if out == DETECT:
                    return NResult(True, t, looks, redundant, aborts, hints,
                                   self.bytes, self.msgs, self.delivered, self.collisions)
                if a.explicit and a.s.send == "full":
                    recent = [r for r in a.records if r.finish >= 0][-a.s.window:]
                    self._send(a, t, recent, None)
            self._deliver(t)
            if t == inst.horizon:
                break
            chosen = []
            for a in self.agents:
                if a.task is not None:
                    continue
                c, v = self._choose(a, t)
                travel = int(inst.dist[a.pos, c])
                a.task = Task(a.seq, c, v, t, t + travel + 1)
                a.seq += 1
                chosen.append((c, v))
                if a.explicit and a.s.send != "none":
                    self._send(a, t, [], self._lease(a))
                    self._deliver(t)
            for x in range(len(chosen)):
                for y in range(x + 1, len(chosen)):
                    if chosen[x] == chosen[y]:
                        self.collisions += 1
        return NResult(False, inst.horizon, looks, redundant, aborts, hints,
                       self.bytes, self.msgs, self.delivered, self.collisions)


def nominal_plan_n(inst: Instance, N: int, S: dict) -> list:
    """Joint all-NULL greedy plan for N agents (common knowledge offline)."""
    o = replace(S["ORACLE"], name="ORACLE", planner="oracle", send="none",
                predict_nominal=False)
    ep = NEpisode(replace(inst, abort=0.0), (o,) * N, LinkConfig(),
                  np.random.default_rng(0), force_null=True)
    plans = [[] for _ in range(N)]
    orig = ep._choose

    def choose(a, t):
        c, v = orig(a, t)
        plans[a.idx].append(Task(a.seq, c, v, t, t + int(inst.dist[a.pos, c]) + 1))
        return c, v
    ep._choose = choose
    ep.run()
    return plans


def run_n(inst, N, name, link, seed, S, nominal=None, keep_log=False):
    if nominal is None:
        nominal = nominal_plan_n(inst, N, S)
    return NEpisode(inst, (S[name],) * N, link, np.random.default_rng(seed),
                    nominal, keep_log=keep_log).run()


# ------------------------------------------------------------------ pilot
PROTOS = ["ORACLE", "INDEP", "IMPLICIT", "HYBRID", "HYBRID_P"]


def pilot(out_path, seeds=(400, 401, 402, 403), episodes=200, grid=(6, 6),
          travels=(0, 1), lams=(1, 3), link_names=("d0_loss0", "d1_loss0"),
          Ns=(2, 3, 4), nboot=4000):
    import sys
    import time
    from .event_curve import LINKS, events
    from .instances import grid_instance
    S = make_all_strategies()
    w, h = grid
    t0 = time.time()
    # res[(tr, lam, link, N, proto)] -> (P[seed], coll[seed])
    res = {}
    for tr in travels:
        for lam in lams:
            for seed in seeds:
                hz = 16 if tr == 0 else 2 * w * h
                inst = grid_instance(w, h, seed=seed, travel_scale=tr, horizon=hz, **events(lam))
                for N in Ns:
                    nom = nominal_plan_n(inst, N, S)
                    for ln in link_names:
                        for p in PROTOS:
                            det = np.zeros(episodes)
                            col = np.zeros(episodes)
                            for ep in range(episodes):
                                r = run_n(inst, N, p, LINKS[ln], ep, S, nom)
                                det[ep], col[ep] = r.detected, r.collisions
                            res.setdefault((tr, lam, ln, N, p), []).append((det, col))
                print(f"[{time.time()-t0:6.0f}s] travel={tr} lam={lam} seed={seed}",
                      file=sys.stderr, flush=True)
    rng = np.random.default_rng(0)
    L = ["# N-agent pilot (nagent.py)", "",
         f"grid {w}x{h}, instance seeds {list(seeds)}, {episodes} episodes/instance, paired "
         "(episode seed shared across protocols). dP = P(detect) - IMPLICIT, per-instance paired "
         "mean; CI = percentile cluster bootstrap over the 4 instances (coarse: only 4 clusters, "
         f"{nboot} resamples). coll = same-step identical (cell,view) task-start pairs per episode.",
         "", "| travel | lam | link | N | proto | P | dP vs IMPLICIT | 95% CI | coll/ep |",
         "|---|---|---|---|---|---|---|---|---|"]
    for key in sorted(k for k in res if k[4] == "IMPLICIT"):
        tr, lam, ln, N, _ = key
        imp = np.array([d.mean() for d, _ in res[key]])
        for p in PROTOS:
            runs = res[(tr, lam, ln, N, p)]
            P = np.array([d.mean() for d, _ in runs])
            col = np.mean([c.mean() for _, c in runs])
            dif = P - imp
            if p == "IMPLICIT":
                ci = "-"; ds = "-"
            else:
                bs = [dif[rng.integers(len(dif), size=len(dif))].mean() for _ in range(nboot)]
                lo, hi = np.percentile(bs, [2.5, 97.5])
                ds, ci = f"{dif.mean():+.3f}", f"[{lo:+.3f}, {hi:+.3f}]"
            L.append(f"| {tr} | {lam} | {ln} | {N} | {p} | {P.mean():.3f} | {ds} | {ci} | {col:.2f} |")
    open(out_path, "w").write("\n".join(L) + "\n")
    print("wrote", out_path)


if __name__ == "__main__":
    import argparse
    ap = argparse.ArgumentParser()
    ap.add_argument("--out", required=True)
    ap.add_argument("--episodes", type=int, default=200)
    ap.add_argument("--travels", type=float, nargs="+", default=[0, 1])
    ap.add_argument("--Ns", type=int, nargs="+", default=[2, 3, 4])
    a = ap.parse_args()
    pilot(a.out, episodes=a.episodes, travels=a.travels, Ns=a.Ns)
