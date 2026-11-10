"""Episode simulator for two searchers under different coordination protocols.

Coordination is expressed as a *ledger*: each agent knows its own look
records, plus whatever it has learned about its partner's looks through
records, reservations (leases) or, for implicit strategies, a nominal plan that
both agents can compute offline.

Strategies (see ``STRATEGIES``):
  ORACLE    centralised; both agents share all records and tasks instantly.
  INDEP     no messages, ignores the partner entirely.
  SECTOR    static partition of cells, no messages.
  IMPLICIT  no messages; predicts the partner's nominal plan (the joint greedy
            plan under all-NULL outcomes). "Silence means on-plan."
  RESERVE   broadcasts a lease (cell, view, expiry) for each new task; expired
            leases are assumed to be completed NULL looks.
  FULL      broadcasts look records + leases for every task (windowed
            store-and-forward of the last W records).
  EVENT     IMPLICIT until a private event (hint/abort) or a partner message,
            then FULL with a deviation time stamp.

Decision rule for every planner: greedy detection-probability per unit time,
P_detect(i, v) / (travel + 1), with partner's in-flight tasks counted as
virtual NULL looks.  Agents decide in index order inside a step.
"""
from __future__ import annotations

from dataclasses import dataclass, field, replace

import numpy as np

from .links import DirectedLink, LinkConfig
from .model import (ABORT, DETECT, HINT, NULL, BeliefModel, Instance, Truth,
                    detect_prob, sample_outcome, sample_truth)

HEADER_B, RECORD_B, LEASE_B = 12, 8, 8


@dataclass(frozen=True)
class Strategy:
    name: str
    planner: str = "ledger"          # oracle | ledger | sector
    predict_nominal: bool = False
    send: str = "none"               # none | reserve | full | event
    window: int = 4                  # records carried per message
    lease_slack: int = 1             # steps a lease outlives its expected finish
    belief: BeliefModel = BeliefModel()
    rollout_k: int = 0               # 0 = greedy; else candidates for rollout
    rollout_budget: int = 8          # steps of greedy continuation per candidate


def make_strategies(**planner_kw) -> dict:
    """Strategy table sharing one base planner configuration."""
    base = dict(
        ORACLE=dict(planner="oracle"),
        INDEP=dict(),
        SECTOR=dict(planner="sector"),
        IMPLICIT=dict(predict_nominal=True),
        RESERVE=dict(send="reserve"),
        FULL=dict(send="full"),
        EVENT=dict(predict_nominal=True, send="event"),
    )
    return {k: Strategy(k, **v, **planner_kw) for k, v in base.items()}


STRATEGIES = make_strategies()


@dataclass
class Task:
    seq: int
    cell: int
    view: int
    start: int
    finish: int


@dataclass
class Record:
    seq: int
    cell: int
    view: int
    outcome: int
    finish: int


@dataclass
class Message:
    src: int
    t_sent: int
    t_dev: int                       # partner's plan matched nominal before this
    records: list = field(default_factory=list)
    lease: tuple | None = None       # (seq, cell, view, expiry)

    def nbytes(self) -> int:
        return HEADER_B + RECORD_B * len(self.records) + (LEASE_B if self.lease else 0)


@dataclass
class EpisodeResult:
    detected: bool
    ttd: int                  # time to detection (horizon if censored)
    looks: int
    redundant: int            # looks at a (cell, view) the partner already did
    aborts: int
    hints: int
    bytes_sent: int
    msgs_sent: int
    msgs_delivered: int
    log: list | None = None


class _Agent:
    def __init__(self, idx: int, strat: Strategy, inst: Instance):
        self.idx = idx
        self.s = strat
        self.pos = inst.starts[idx]
        self.task: Task | None = None
        self.seq = 0
        self.records: list[Record] = []          # own completed looks
        self.p_records: dict[int, Record] = {}   # partner looks learned by msg
        self.p_leases: dict[int, tuple] = {}     # seq -> (cell, view, expiry)
        self.p_explicit = False
        self.p_tdev = 0
        self.explicit = strat.send in ("full", "reserve")
        self.t_dev = 0
        self.m0 = np.zeros((inst.n, inst.V), int)
        self.m1 = np.zeros((inst.n, inst.V), int)


class Episode:
    def __init__(self, inst: Instance, strats: tuple[Strategy, Strategy],
                 link: LinkConfig, rng: np.random.Generator,
                 nominal: list[list[Task]] | None = None,
                 sectors: np.ndarray | None = None,
                 force_null: bool = False, keep_log: bool = False):
        self.inst = inst
        self.strats = strats
        self.force_null = force_null
        self.truth: Truth = sample_truth(inst, rng)
        self.u = rng.random((2, inst.horizon + 1, 3))
        self.links = [DirectedLink(link, inst.horizon, rng),
                      DirectedLink(link, inst.horizon, rng)]
        self.nominal = nominal
        self.sectors = sectors
        self.agents = [_Agent(0, strats[0], inst), _Agent(1, strats[1], inst)]
        self.oracle = strats[0].planner == "oracle" or strats[1].planner == "oracle"
        self.log = [] if keep_log else None
        self.bytes = self.msgs = self.delivered = 0
        self.looked_by = [set(), set()]
        self._framed: set = set()

    # ---------------------------------------------------------------- belief
    def _partner_counts(self, a: _Agent, t: int):
        """Counts contributed by partner looks as agent ``a`` believes them,
        plus the partner's in-flight (virtual) tasks."""
        inst = self.inst
        p = self.agents[1 - a.idx]
        m0 = np.zeros((inst.n, inst.V), int)
        m1 = np.zeros_like(m0)
        virtual = []
        if self.oracle:
            for r in p.records:
                if r.outcome == NULL:
                    m0[r.cell, r.view] += 1
                elif r.outcome == HINT:
                    m1[r.cell, r.view] += 1
            if p.task is not None:
                virtual.append((p.task.cell, p.task.view))
            return m0, m1, virtual

        if a.s.planner == "sector" or (a.s.send == "none" and not a.s.predict_nominal):
            return m0, m1, virtual

        seen = set()
        if a.s.predict_nominal and self.nominal is not None:
            for tk in self.nominal[p.idx]:
                if a.p_explicit and tk.finish >= a.p_tdev:
                    break  # after the partner's deviation only messages count
                if tk.finish <= t:
                    m0[tk.cell, tk.view] += 1          # nominal look done
                    seen.add(tk.seq)
                elif (tk.start, p.idx) < (t, a.idx):
                    virtual.append((tk.cell, tk.view))  # nominal look in flight
                    seen.add(tk.seq)
                else:
                    break
        for seq, r in a.p_records.items():
            if seq in seen:
                continue
            seen.add(seq)
            if r.outcome == NULL:
                m0[r.cell, r.view] += 1
            elif r.outcome == HINT:
                m1[r.cell, r.view] += 1
        for seq, (c, v, exp) in a.p_leases.items():
            if seq in seen:
                continue
            if t <= exp:
                virtual.append((c, v))
            else:
                m0[c, v] += 1   # expired lease assumed to be a NULL look
        return m0, m1, virtual

    def _choose(self, a: _Agent, t: int) -> tuple[int, int]:
        inst = self.inst
        pm0, pm1, virtual = self._partner_counts(a, t)
        m0 = a.m0 + pm0
        m1 = a.m1 + pm1
        for (c, v) in virtual:
            m0[c, v] += 1
        bm = a.s.belief
        mask = None
        if a.s.planner == "sector" and self.sectors is not None:
            mask = (self.sectors == a.idx)[:, None]
        pd = detect_prob(inst, m0, m1, bm)
        score = pd / (inst.dist[a.pos][:, None] + 1.0)
        if mask is not None:
            score = np.where(mask, score, -1.0)
        if a.s.rollout_k <= 0:
            return divmod(int(np.argmax(score)), inst.V)
        # Rollout: for each of the top-k first tasks, continue greedily for a
        # fixed time budget assuming NULL outcomes; maximise P(detect within
        # budget) of the whole sequence.  Ties broken by greedy score.
        budget = a.s.rollout_budget
        cand = np.argsort(-score, axis=None, kind="stable")[:a.s.rollout_k]
        best, best_val = int(cand[0]), -1.0
        for k in cand:
            c, v = divmod(int(k), inst.V)
            mm0 = m0.copy()
            pos, used, p_none, val = a.pos, 0, 1.0, 0.0
            pdk = pd
            while True:
                dt = int(inst.dist[pos, c]) + 1
                if used + dt > budget and used > 0:
                    break
                used += dt
                val += p_none * pdk[c, v]
                p_none *= 1.0 - pdk[c, v]
                mm0[c, v] += 1
                pos = c
                if used >= budget:
                    break
                pdk = detect_prob(inst, mm0, m1, bm)
                sc = pdk / (inst.dist[pos][:, None] + 1.0)
                if mask is not None:
                    sc = np.where(mask, sc, -1.0)
                c, v = divmod(int(np.argmax(sc)), inst.V)
            if val > best_val + 1e-12:
                best, best_val = int(k), val
        return divmod(best, inst.V)

    # ------------------------------------------------------------- messaging
    def _send(self, a: _Agent, t: int, records: list, lease):
        msg = Message(a.idx, t, a.t_dev, records, lease)
        # Records and lease sent by one agent in one step share a frame.
        nb = msg.nbytes()
        if (a.idx, t) in self._framed:
            nb -= HEADER_B
        else:
            self._framed.add((a.idx, t))
            self.msgs += 1
        self.bytes += nb
        if self.links[a.idx].send(t, msg):
            self.delivered += 1

    def _deliver(self, t: int):
        for a in self.agents:
            for msg in self.links[1 - a.idx].receive(t):
                if not a.p_explicit:
                    a.p_explicit = True
                    a.p_tdev = msg.t_dev
                for r in msg.records:
                    a.p_records[r.seq] = r
                    a.p_leases.pop(r.seq, None)
                if msg.lease is not None:
                    seq, c, v, exp = msg.lease
                    if seq not in a.p_records:
                        a.p_leases[seq] = (c, v, exp)
                # Receiving news of a deviation changes my own plan, so an
                # EVENT agent must become explicit too.
                if a.s.send == "event" and not a.explicit:
                    a.explicit = True
                    a.t_dev = t
                    self._send(a, t, [], self._lease(a))

    def _lease(self, a: _Agent):
        if a.task is None:
            return None
        return (a.task.seq, a.task.cell, a.task.view, a.task.finish + a.s.lease_slack)

    # ------------------------------------------------------------------ run
    def run(self) -> EpisodeResult:
        inst = self.inst
        looks = redundant = aborts = hints = 0
        for t in range(inst.horizon + 1):
            self._deliver(t)
            # 1) complete tasks finishing now
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
                    if (tk.cell, tk.view) in self.looked_by[1 - a.idx]:
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
                    return EpisodeResult(True, t, looks, redundant, aborts, hints,
                                         self.bytes, self.msgs, self.delivered, self.log)
                if a.s.send == "event" and not a.explicit and out in (HINT, ABORT):
                    a.explicit = True
                    a.t_dev = t
                if a.explicit and a.s.send in ("full", "event"):
                    recent = [r for r in a.records if r.finish >= a.t_dev][-a.s.window:]
                    self._send(a, t, recent, None)
            self._deliver(t)
            if t == inst.horizon:
                break
            # 2) idle agents choose new tasks (index order) and send leases
            for a in self.agents:
                if a.task is not None:
                    continue
                c, v = self._choose(a, t)
                travel = int(inst.dist[a.pos, c])
                a.task = Task(a.seq, c, v, t, t + travel + 1)
                a.seq += 1
                if a.explicit and a.s.send != "none":
                    self._send(a, t, [], self._lease(a))
                    self._deliver(t)
        return EpisodeResult(False, inst.horizon, looks, redundant, aborts, hints,
                             self.bytes, self.msgs, self.delivered, self.log)


# ---------------------------------------------------------------- helpers
def nominal_plan(inst: Instance, like: Strategy | None = None) -> list[list[Task]]:
    """Joint plan under all-NULL outcomes and no aborts, using the same base
    planner as ``like``; computable offline by both agents (common knowledge)."""
    like = like or STRATEGIES["ORACLE"]
    o = replace(like, name="ORACLE", planner="oracle", send="none", predict_nominal=False)
    ep = Episode(replace(inst, abort=0.0), (o, o), LinkConfig(),
                 np.random.default_rng(0), force_null=True, keep_log=False)
    plans: list[list[Task]] = [[], []]
    orig_choose = ep._choose

    def choose(a, t):
        c, v = orig_choose(a, t)
        travel = int(inst.dist[a.pos, c])
        plans[a.idx].append(Task(a.seq, c, v, t, t + travel + 1))
        return c, v
    ep._choose = choose
    ep.run()
    return plans


def make_sectors(inst: Instance) -> np.ndarray:
    """Split cells into two contiguous halves of roughly equal prior mass
    along the longer coordinate axis (or by index if no coordinates)."""
    if inst.coords is not None:
        span = inst.coords.max(0) - inst.coords.min(0)
        key = inst.coords[:, int(np.argmax(span))]
    else:
        key = np.arange(inst.n)
    order = np.argsort(key, kind="stable")
    cum = np.cumsum(inst.prior[order])
    sec = np.zeros(inst.n, int)
    sec[order[cum > 0.5]] = 1
    # Agent 0 takes the half nearer its start.
    if inst.dist[inst.starts[0]][sec == 0].mean() > inst.dist[inst.starts[0]][sec == 1].mean():
        sec = 1 - sec
    return sec


def run_episode(inst: Instance, strat_names: tuple[str, str], link: LinkConfig,
                seed: int, nominal=None, sectors=None, keep_log=False,
                strategies: dict | None = None) -> EpisodeResult:
    S = strategies or STRATEGIES
    strats = (S[strat_names[0]], S[strat_names[1]])
    if nominal is None and any(s.predict_nominal for s in strats):
        nominal = nominal_plan(inst, strats[0])
    if sectors is None and any(s.planner == "sector" for s in strats):
        sectors = make_sectors(inst)
    rng = np.random.default_rng(seed)
    return Episode(inst, strats, link, rng, nominal, sectors, keep_log=keep_log).run()
