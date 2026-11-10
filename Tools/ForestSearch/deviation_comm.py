"""Event-triggered, prediction-deviation communication (ConTaCT / Roth et al. style).

New file; no existing file is edited.  Subclasses beacon_robustness.BEpisode, so every
fault kind there (perm, transient, intermit, total, beacon_lost) applies unchanged.

cd Tools && fsrun python3 -m ForestSearch.deviation_comm a --out /tmp/dev_a.json
cd Tools && fsrun python3 -m ForestSearch.deviation_comm b --out /tmp/dev_b.json
cd Tools && fsrun python3 -m ForestSearch.deviation_comm c --out /tmp/dev_c.json
python3 -m ForestSearch.deviation_comm report --inputs /tmp/dev_a.json ... --out ForestSearch/results/deviation_comm.md

Common ground (CG).  Each agent keeps a copy of the "common-ground simulation": what its
partner can predict about both agents.  It starts as the shared default plan.  Whenever
a CG agent is idle, CG predicts its next task by the SAME greedy rule as the real
planner, using only common-ground looks: CG tasks of both agents, with outcome NULL unless
a message corrected it, plus the other agent's CG tasks in flight.  With no messages the CG
simulation is exactly nominal_plan() (tested), so a DEV agent that never sends is IMPLICIT.
A real agent plans with its own true records plus its CG copy's model of the partner,
which is the IMPLICIT rule with "nominal" replaced by "CG".

Messages.  A frame is 12 B: a 4 B header plus two 4 B items.  An item is one of:
  task(seq, cell, view, start, finish)   "my task #seq is this"; CG replaces task seq and
                                          drops this agent's later predicted tasks
  rec(seq, cell, view, outcome)          "my look #seq had this outcome" (HINT / ABORT /
                                          a NULL look CG did not predict)
  idle(t0)                               "I do no looks from t0 on until I declare a task"
                                          (only DEV_SD, i.e. the agent self-diagnoses a dead
                                          sensor, as HEALTH assumes)
The sender applies its own frame to its own CG copy at t + delay, assuming delivery.  It has
no acks, so loss makes the copies diverge.  The receiver applies the frame on delivery.
With a deterministic delay and no loss the two copies are identical (tested).

Triggers.
  task : at each own decision, send task(...) iff the actual new task (cell, view, start,
         finish) differs from the CG prediction of that task.  DEV_SD: a dead-sensor decision
         is "no look", and idle(t) is sent iff CG still predicts looks.
  info : after completions in every step, collect own records that differ from CG's model of
         me.  Recompute the partner's greedy choice "as if it decided now" from CG, with and
         without these records (exact argmax of the same score).  If the argmax changes,
         send all of them.
  both : task trigger, plus the info trigger.  A task frame's spare slot carries the newest
         pending record at no extra cost.
Budget: dev_budget bytes per AGENT per episode (-1 = unlimited).  Once it is spent the agent
falls silent, and its partner keeps predicting it from CG.
"""
from __future__ import annotations

import argparse
import json
import math
import time
from dataclasses import dataclass, field

import numpy as np

from .beacon_robustness import (LINKS, R7_CELLS, BEpisode, BStrategy, Ctx, builders,
                                fault_for, hb_to, make_ctxs, make_strategies as make_b_strategies)
from .event_curve import events
from .falsify import boot
from .instances import grid_instance
from .model import ABORT, DETECT, HINT, NULL, detect_prob, sample_outcome
from .sim import EpisodeResult, Record, Task

FRAME_B = 12
ITEMS_PER_FRAME = 2


@dataclass(frozen=True)
class DStrategy(BStrategy):
    dev: str = ""              # "" | task | info | both
    dev_budget: int = -1       # bytes per agent per episode; -1 = unlimited
    selfdiag: bool = False     # agent knows its sensor is dead (HEALTH's assumption)


# ------------------------------------------------------------ common ground
@dataclass
class CGTask:
    seq: int
    cell: int
    view: int
    start: int
    finish: int
    outcome: int = NULL


class CommonGround:
    def __init__(self, inst):
        self.inst = inst
        self.tasks: list[dict[int, CGTask]] = [{}, {}]
        self.idle = [False, False]

    def copy_state(self):
        return ([{k: (v.cell, v.view, v.start, v.finish, v.outcome) for k, v in d.items()} for d in self.tasks],
                list(self.idle))

    def last(self, i):
        d = self.tasks[i]
        return d[max(d)] if d else None

    def _counts(self, t, override=None, override_agent=None, all_inflight=False, me=None):
        inst = self.inst
        m0 = np.zeros((inst.n, inst.V), int)
        m1 = np.zeros_like(m0)
        virtual = []
        for j in (0, 1):
            d = self.tasks[j]
            if override is not None and j == override_agent:
                d = dict(d)
                for r in override:
                    d[r.seq] = CGTask(r.seq, r.cell, r.view, -1, r.finish, r.outcome)
            for tk in d.values():
                if tk.finish <= t:
                    if tk.outcome == NULL:
                        m0[tk.cell, tk.view] += 1
                    elif tk.outcome == HINT:
                        m1[tk.cell, tk.view] += 1
                elif all_inflight or (j != me and (tk.start, j) < (t, me)):
                    if tk.outcome != ABORT:
                        virtual.append((tk.cell, tk.view))
        return m0, m1, virtual

    def partner_counts(self, me, t):
        """IMPLICIT's partner-count rule applied to CG's model of the partner."""
        inst = self.inst
        p = 1 - me
        m0 = np.zeros((inst.n, inst.V), int)
        m1 = np.zeros_like(m0)
        virtual = []
        for tk in self.tasks[p].values():
            if tk.finish <= t:
                if tk.outcome == NULL:
                    m0[tk.cell, tk.view] += 1
                elif tk.outcome == HINT:
                    m1[tk.cell, tk.view] += 1
            elif (tk.start, p) < (t, me) and tk.outcome != ABORT:
                virtual.append((tk.cell, tk.view))
        return m0, m1, virtual

    def _greedy(self, i, m0, m1, virtual, bm):
        inst = self.inst
        m0 = m0.copy()
        for (c, v) in virtual:
            m0[c, v] += 1
        last = self.last(i)
        pos = last.cell if last is not None else inst.starts[i]
        pd = detect_prob(inst, m0, m1, bm)
        score = pd / (inst.dist[pos][:, None] + 1.0)
        return divmod(int(np.argmax(score)), inst.V)

    def advance(self, i, t, bm):
        """Predict CG agent i's next task at t if it is idle (same greedy as the planner)."""
        if self.idle[i]:
            return
        last = self.last(i)
        if last is not None and last.finish > t:
            return
        m0, m1, virtual = self._counts(t, me=i)
        c, v = self._greedy(i, m0, m1, virtual, bm)
        seq = last.seq + 1 if last is not None else 0
        self.tasks[i][seq] = CGTask(seq, c, v, t, t + int(self.inst.dist[last.cell if last else self.inst.starts[i], c]) + 1)

    def choice_now(self, i, t, bm, override=None, override_agent=None):
        """Agent i's greedy choice if it decided at t from CG (every in-flight CG task counted
        as a virtual look), optionally with ``override`` records of ``override_agent``."""
        m0, m1, virtual = self._counts(t, override, override_agent, all_inflight=True)
        return self._greedy(i, m0, m1, virtual, bm)

    def apply(self, src, items):
        for it in items:
            kind = it[0]
            d = self.tasks[src]
            if kind == "task":
                _, seq, c, v, s, f = it
                for k in [k for k in d if k > seq]:
                    del d[k]
                d[seq] = CGTask(seq, c, v, s, f, NULL)
                self.idle[src] = False
            elif kind == "rec":
                _, seq, c, v, out, s, f = it
                tk = d.get(seq)
                if tk is not None and (tk.cell, tk.view) == (c, v):
                    tk.outcome = out
                else:
                    d[seq] = CGTask(seq, c, v, s, f, out)
            elif kind == "idle":
                _, t0 = it
                for k in [k for k, tk in d.items() if tk.start >= t0]:
                    del d[k]
                self.idle[src] = True
            else:
                raise ValueError(kind)


@dataclass
class DevMsg:
    src: int
    t_sent: int
    items: list = field(default_factory=list)


# ------------------------------------------------------------------ episode
class DEpisode(BEpisode):
    def __init__(self, *a, **k):
        super().__init__(*a, **k)
        self.cg = [CommonGround(self.inst), CommonGround(self.inst)]
        self.self_queue: list[tuple[int, int, list]] = []   # (t_effective, src, items)
        self.left = [self._mode_budget()] * 2
        self.frame_log: list[tuple[int, int, int, str]] = []  # (t, src, frames, reason)
        self._drop_used = False
        self.dev_starts: list[int] = []

    def _mode(self):
        ms = {getattr(s, "dev", "") for s in self.strats}
        return ms.pop() if len(ms) == 1 else ""

    def _mode_budget(self):
        b = getattr(self.strats[0], "dev_budget", -1)
        return math.inf if b < 0 else b

    # ----------------------------------------------------------- planning
    def _partner_counts(self, a, t):
        if not self._mode() or self.oracle:
            return super()._partner_counts(a, t)
        return self.cg[a.idx].partner_counts(a.idx, t)

    # -------------------------------------------------------------- radio
    def _dev_send(self, i, t, items, reason):
        if not items:
            return
        nf = math.ceil(len(items) / ITEMS_PER_FRAME)
        nf = int(min(nf, self.left[i] // FRAME_B)) if self.left[i] != math.inf else nf
        if nf <= 0:
            return
        items = items[:nf * ITEMS_PER_FRAME]
        if self.radio_dead[i, min(t, self.radio_dead.shape[1] - 1)]:
            return                                           # total loss: nothing transmitted
        self.left[i] -= nf * FRAME_B
        self.bytes += nf * FRAME_B
        self.msgs += nf
        self.frame_log.append((t, i, nf, reason))
        lost = False
        td = self.t_dead[i] if hasattr(self, "t_dead") else None
        if (self.fault.get("drop_first") and not self._drop_used and td is not None and t >= td):
            self._drop_used = True
            lost = True                                      # first post-failure frame dropped
        msg = DevMsg(i, t, list(items))
        if not lost and self.links[i].send(t, msg):
            self.delivered += 1
        te = t + self.links[i].cfg.hops * self.links[i].cfg.delay
        if te <= t:
            self.cg[i].apply(i, items)
        else:
            self.self_queue.append((te, i, list(items)))

    def _deliver(self, t):
        if not self._mode():
            return super()._deliver(t)
        keep = []
        for (te, src, items) in self.self_queue:
            if te <= t:
                self.cg[src].apply(src, items)
            else:
                keep.append((te, src, items))
        self.self_queue = keep
        for a in self.agents:
            for m in self.links[1 - a.idx].receive(t):
                self.cg[a.idx].apply(m.src, m.items)

    # ----------------------------------------------------------- triggers
    def _pending(self, a, t):
        """Own records that differ from CG's model of me (newest last)."""
        d = self.cg[a.idx].tasks[a.idx]
        out = []
        for r in a.records:
            if r.finish > t:
                continue
            ct = d.get(r.seq)
            if r.outcome == ABORT:
                if ct is not None and ct.outcome != ABORT:
                    out.append(r)
            elif ct is None or (ct.cell, ct.view, ct.outcome) != (r.cell, r.view, r.outcome):
                out.append(r)
        return out

    @staticmethod
    def _rec_item(r, start):
        return ("rec", r.seq, r.cell, r.view, r.outcome, start, r.finish)

    def _info_check(self, a, t):
        pend = self._pending(a, t)
        if not pend:
            return
        cg = self.cg[a.idx]
        b = 1 - a.idx
        bm = a.s.belief
        c0 = cg.choice_now(b, t, bm)
        c1 = cg.choice_now(b, t, bm, override=pend, override_agent=a.idx)
        if c0 != c1:
            items = [self._rec_item(r, self._starts.get((a.idx, r.seq), r.finish - 1)) for r in reversed(pend)]
            self._dev_send(a.idx, t, items, "info")

    def _task_check(self, a, t, mode):
        cg = self.cg[a.idx]
        tk = a.task
        dead_sd = a.s.selfdiag and bool(self.bad[a.idx, min(t, self.inst.horizon)])
        if dead_sd:
            if cg.idle[a.idx]:
                return
            items = [("idle", t)]
        else:
            pred = cg.tasks[a.idx].get(tk.seq)
            if (not cg.idle[a.idx] and pred is not None
                    and (pred.cell, pred.view, pred.start, pred.finish) == (tk.cell, tk.view, tk.start, tk.finish)):
                return
            items = [("task", tk.seq, tk.cell, tk.view, tk.start, tk.finish)]
        if mode == "both":
            pend = self._pending(a, t)
            if pend:
                r = pend[-1]
                items.append(self._rec_item(r, self._starts.get((a.idx, r.seq), r.finish - 1)))
        self._dev_send(a.idx, t, items, "task")
        self.dev_starts.append(t)

    # ---------------------------------------------------------------- run
    def run(self) -> EpisodeResult:
        mode = self._mode()
        if not mode:
            return super().run()
        inst = self.inst
        bm = self.strats[0].belief
        self._starts = {}
        looks = redundant = aborts = hints = 0
        res = None
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
                    if (tk.cell, tk.view) in self.looked_by[1 - a.idx]:
                        redundant += 1
                    self.looked_by[a.idx].add((tk.cell, tk.view))
                if out == NULL:
                    a.m0[tk.cell, tk.view] += 1
                elif out == HINT:
                    a.m1[tk.cell, tk.view] += 1
                    hints += 1
                if out == DETECT:
                    res = EpisodeResult(True, t, looks, redundant, aborts, hints,
                                        self.bytes, self.msgs, self.delivered, None)
                    break
            if res is not None:
                break
            if mode in ("info", "both"):
                for a in self.agents:
                    self._info_check(a, t)
                    self._deliver(t)
            self._deliver(t)
            if t == inst.horizon:
                break
            for a in self.agents:
                for c in self.cg:
                    c.advance(a.idx, t, bm)
                if a.task is not None:
                    continue
                c, v = self._choose(a, t)
                travel = int(inst.dist[a.pos, c])
                a.task = Task(a.seq, c, v, t, t + travel + 1)
                self._starts[(a.idx, a.seq)] = t
                a.seq += 1
                if mode in ("task", "both"):
                    self._task_check(a, t, mode)
                    self._deliver(t)
        if res is None:
            res = EpisodeResult(False, inst.horizon, looks, redundant, aborts, hints,
                                self.bytes, self.msgs, self.delivered, None)
        res.frame_log = list(self.frame_log)
        res.t_dead = list(self.t_dead) if hasattr(self, "t_dead") else [None, None]
        return res


# --------------------------------------------------------------- strategies
DEV_SPECS = {
    "DEV": dict(dev="both"),
    "DEV_TASK": dict(dev="task"),
    "DEV_INFO": dict(dev="info"),
    "DEV_SD": dict(dev="both", selfdiag=True),
    **{f"DEV_B{b}": dict(dev="both", dev_budget=b) for b in (12, 24, 48, 96)},
    **{f"DEV_SD_B{b}": dict(dev="both", selfdiag=True, dev_budget=b) for b in (12, 24, 48)},
    "DEV_B0": dict(dev="both", dev_budget=0),
}


def make_strategies() -> dict:
    S = make_b_strategies({"HB_TO": hb_to(3)})       # HB_TO = the DEV-tuned P=3 (beacon_tuning.json)
    base = S["IMPLICIT"]
    f = {k: getattr(base, k) for k in base.__dataclass_fields__}
    for nm, kw in DEV_SPECS.items():
        g = dict(f); g["name"] = nm
        S[nm] = DStrategy(**g, **kw)
    return S


def run_d(ctx, name, ep, S, fault, link):
    """Same seeding as beacon_robustness.run_b (paired design)."""
    f = dict(fault or {}); f["seed"] = 10_000 + ep
    return DEpisode(ctx.inst, (S[name], S[name]), LINKS[link], np.random.default_rng(ep),
                    ctx.nom, ctx.sec, fault=f, sec_nominal=ctx.snom).run()


def is_dev(n):
    return n.startswith("DEV")


SENDERS_ALL = ("HEALTH", "HEARTBEAT", "HB_TO", "HYBRID_P")


def sends(n):
    return is_dev(n) or n in SENDERS_ALL


def frame_stats(r):
    """Frames of the lost agent before / after its death and lag of the first post-death frame."""
    td = [x for x in r.t_dead if x is not None]
    if not td:
        return None
    who = r.t_dead.index(td[0])
    pre = sum(nf for (t, i, nf, _) in r.frame_log if i == who and t < td[0])
    post = [(t, nf) for (t, i, nf, _) in r.frame_log if i == who and t >= td[0]]
    partner = sum(nf for (t, i, nf, _) in r.frame_log if i != who)
    alive_at_end = r.ttd >= td[0]
    return dict(pre=pre, post=sum(nf for _, nf in post), lag=(post[0][0] - td[0]) if post else None,
                partner=partner, reached=bool(alive_at_end))


ONLY0: set = set()      # strategies run on the first link only (compute cap)


def run_cell(S, ctxs, fault_of, names, links, eps):
    det, byt, fst = {}, {}, {}
    for sd, ctx in ctxs.items():
        fault = fault_of(ctx.inst)
        for ln in links:
            for n in names:
                if ln != links[0] and (not sends(n) or n in ONLY0):
                    continue
                rs = [run_d(ctx, n, ep, S, fault, ln) for ep in range(eps)]
                det.setdefault(f"{ln}|{n}", {})[sd] = [int(r.detected) for r in rs]
                byt.setdefault(f"{ln}|{n}", {})[sd] = [int(r.bytes_sent) for r in rs]
                if is_dev(n) and fault:
                    fst.setdefault(f"{ln}|{n}", []).extend(
                        x for x in (frame_stats(r) for r in rs) if x is not None)
    return dict(det=det, byt=byt, fst=fst)


# --------------------------------------------------------------- experiments
A_NAMES = ["ORACLE", "IMPLICIT", "HYBRID_P", "DEV", "DEV_TASK", "DEV_INFO", "DEV_B12", "DEV_B24", "DEV_B48"]
B_NAMES = ["ORACLE", "IMPLICIT", "HEALTH", "HYBRID_P", "DEV", "DEV_TASK", "DEV_INFO", "DEV_SD",
           "DEV_SD_B12", "DEV_SD_B24", "DEV_SD_B48", "DEV_B24"]
C_NAMES = ["IMPLICIT", "HEALTH", "HB_TO", "DEV", "DEV_SD", "DEV_SD_B24"]
C_CONDS = ["trans_m", "intermit", "total", "beacon_lost", "perm_late"]


def part_a(seeds, eps, links, grids, travels, lams, names, out, t0):
    S = make_strategies()
    res = {}
    for g in grids:
        w, h = map(int, g.split("x"))
        for tr in travels:
            for lam in lams:
                hz = 16 if tr == 0 else 2 * w * h
                ctxs = {sd: Ctx(grid_instance(w, h, seed=sd, travel_scale=tr, horizon=hz, **events(lam)), S)
                        for sd in seeds}
                key = f"{g}|t{tr:g}|lam{lam:g}"
                res[key] = run_cell(S, ctxs, lambda inst: None, names, links, eps)
                print(f"[{time.time()-t0:6.0f}s] A {key}", flush=True)
                json.dump(dict(part="a", seeds=seeds, eps=eps, links=links, names=names, res=res), open(out, "w"))
    return res


def part_b(seeds, eps, links, names, out, t0):
    S = make_strategies()
    res, cache = {}, {}
    for key, bk, f in R7_CELLS:
        if bk not in cache:
            cache[bk] = make_ctxs(bk, seeds, S)
        res[key] = run_cell(S, cache[bk], lambda inst, f=f: f, names, links, eps)
        print(f"[{time.time()-t0:6.0f}s] B {key}", flush=True)
        json.dump(dict(part="b", seeds=seeds, eps=eps, links=links, names=names, res=res), open(out, "w"))
    return res


def part_c(seeds, eps, links, names, conds, out, t0):
    S = make_strategies()
    ctxs = make_ctxs("A", seeds, S)
    res = {}
    for cond in conds:
        res[cond] = run_cell(S, ctxs, lambda inst, c=cond: fault_for(c, inst.horizon), names, links, eps)
        print(f"[{time.time()-t0:6.0f}s] C {cond}", flush=True)
        json.dump(dict(part="c", seeds=seeds, eps=eps, links=links, names=names, res=res), open(out, "w"))
    return res


# ------------------------------------------------------------------- report
def _arr(d):
    return {int(s): np.asarray(v, float) for s, v in d.items()}


def _get(det, ln, n, l0):
    return _arr(det[f"{ln}|{n}"] if f"{ln}|{n}" in det else det[f"{l0}|{n}"])


def _fmt(c):
    m, lo, hi = c
    return f"{m:+.3f} [{lo:+.3f},{hi:+.3f}]" + ("*" if lo > 0 or hi < 0 else "")


def _pool(cells, ln, n, ref, l0):
    """Pooled paired diff over cells; clusters = (cell, instance)."""
    diff = {}
    for ck, cr in cells.items():
        D = _get(cr["det"], ln, n, l0); R = _get(cr["det"], ln, ref, l0)
        for s in D:
            diff[f"{ck}#{s}"] = D[s] - R[s]
    keys = sorted(diff)
    idx = {k: i for i, k in enumerate(keys)}
    return boot({idx[k]: diff[k] for k in keys})


def _bytes(cr, ln, n, l0):
    k = f"{ln}|{n}" if f"{ln}|{n}" in cr["byt"] else f"{l0}|{n}"
    return float(np.mean(np.concatenate([np.asarray(v, float) for v in cr["byt"][k].values()])))


def report(inputs, out):
    L = []
    for path in inputs:
        J = json.load(open(path))
        part, res, links, names = J["part"], J["res"], J["links"], J["names"]
        l0 = links[0]
        refs = {"a": ["IMPLICIT", "ORACLE"], "b": ["IMPLICIT", "HEALTH", "HYBRID_P"],
                "c": ["IMPLICIT", "HEALTH"]}[part]
        L.append(f"\n## Part {part.upper()}: seeds {J['seeds'][0]}-{J['seeds'][-1]}, {J['eps']} episodes/cell\n")
        for ck, cr in res.items():
            L.append(f"\n### {ck}\n")
            L.append("| link | strategy | P(detect) | " + " | ".join(f"d vs {r}" for r in refs) + " | bytes/ep |")
            L.append("|---|---|---|" + "---|" * len(refs) + "---|")
            for ln in links:
                for n in names:
                    if f"{ln}|{n}" not in cr["det"] and (ln != l0 or f"{l0}|{n}" not in cr["det"]):
                        continue
                    if f"{ln}|{n}" not in cr["det"] and f"{l0}|{n}" not in cr["det"]:
                        continue
                    D = _get(cr["det"], ln, n, l0)
                    P = float(np.mean(np.concatenate(list(D.values()))))
                    cells = []
                    for r in refs:
                        if r == n:
                            cells.append("-"); continue
                        R = _get(cr["det"], ln, r, l0)
                        cells.append(_fmt(boot({s: D[s] - R[s] for s in D})))
                    L.append(f"| {ln} | {n} | {P:.3f} | " + " | ".join(cells) + f" | {_bytes(cr, ln, n, l0):.1f} |")
            fs = cr.get("fst", {})
            if fs:
                L.append("\nLost-agent frames (episodes that reached the failure): ")
                L.append("| link | strategy | frames before death | frames after death | P(any post-death frame) | median lag of 1st (steps) | partner frames |")
                L.append("|---|---|---|---|---|---|---|")
                for k, v in fs.items():
                    v = [x for x in v if x["reached"]]
                    if not v:
                        continue
                    lags = [x["lag"] for x in v if x["lag"] is not None]
                    L.append(f"| {k.split('|')[0]} | {k.split('|')[1]} | {np.mean([x['pre'] for x in v]):.2f} | "
                             f"{np.mean([x['post'] for x in v]):.2f} | {np.mean([x['lag'] is not None for x in v]):.2f} | "
                             f"{(np.median(lags) if lags else float('nan')):.0f} | {np.mean([x['partner'] for x in v]):.2f} |")
        if part in ("a", "b"):
            L.append(f"\n### Part {part.upper()} pooled over cells (clusters = cell x instance)\n")
            L.append("| link | strategy | " + " | ".join(f"d vs {r}" for r in refs) + " | mean bytes/ep |")
            L.append("|---|---|" + "---|" * len(refs) + "---|")
            for ln in links:
                for n in names:
                    if any(f"{ln}|{n}" not in cr["det"] and (ln != l0 or f"{l0}|{n}" not in cr["det"]) for cr in res.values()):
                        continue
                    cells = ["-" if r == n else _fmt(_pool(res, ln, n, r, l0)) for r in refs]
                    mb = np.mean([_bytes(cr, ln, n, l0) for cr in res.values()])
                    L.append(f"| {ln} | {n} | " + " | ".join(cells) + f" | {mb:.1f} |")
    txt = "\n".join(L) + "\n"
    if out:
        open(out, "w").write(txt)
    print(txt)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("part", choices=["a", "b", "c", "report"])
    ap.add_argument("--seeds", type=int, nargs="+", default=None)
    ap.add_argument("--episodes", type=int, default=None)
    ap.add_argument("--links", nargs="+", default=None)
    ap.add_argument("--names", nargs="+", default=None)
    ap.add_argument("--grids", nargs="+", default=["5x5", "6x6"])
    ap.add_argument("--travels", type=float, nargs="+", default=[0, 1])
    ap.add_argument("--lams", type=float, nargs="+", default=[1, 3])
    ap.add_argument("--conds", nargs="+", default=C_CONDS)
    ap.add_argument("--inputs", nargs="+", default=[])
    ap.add_argument("--only0", nargs="*", default=[])
    ap.add_argument("--out", default=None)
    a = ap.parse_args()
    t0 = time.time()
    ONLY0.update(a.only0)
    if a.part == "a":
        part_a(a.seeds or list(range(200, 208)), a.episodes or 300, a.links or ["d0_loss0", "d1_loss0"],
               a.grids, a.travels, a.lams, a.names or A_NAMES, a.out, t0)
    elif a.part == "b":
        part_b(a.seeds or list(range(800, 812)), a.episodes or 200, a.links or ["d0_loss0", "d1_burst30"],
               a.names or B_NAMES, a.out, t0)
    elif a.part == "c":
        part_c(a.seeds or list(range(800, 812)), a.episodes or 200, a.links or ["d0_loss0", "d1_burst30"],
               a.names or C_NAMES, a.conds, a.out, t0)
    else:
        report(a.inputs, a.out)


if __name__ == "__main__":
    main()
