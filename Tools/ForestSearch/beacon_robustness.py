"""Adversarial test of the health-beacon claim (new file; no existing file edited).

Claim under test (paper): under permanent agent loss one ~12-byte HEALTH frame
recovers essentially all of full messaging's gain over IMPLICIT and, unlike
INDEP, costs nothing when nobody fails.

cd Tools && fsrun python3 -m ForestSearch.beacon_robustness tune  --seeds 500 501 502 503 --episodes 200
cd Tools && fsrun python3 -m ForestSearch.beacon_robustness final --seeds 800 ... 811 --episodes 250 --out ...

A. Silent hedges (no radio at all, bytes = 0):
  TAKEOVER(f, w)  IMPLICIT until T = round(f*H); from T on each agent scores
                  max(score_IMPLICIT, w * score_if_partner_died_at_T), i.e.
                  it takes up the partner's remaining default-plan cells at
                  priority w.  T > H  ==  IMPLICIT exactly.
  REDUNDANT(rho)  planned overlap: P(detect) = (1-rho)*pd_IMPLICIT + rho*pd_INDEP.
                  rho = 0 == IMPLICIT, rho = 1 == INDEP.
  BAYES(pi)       Bayes-mixture myopic planner that KNOWS the loss model shape:
                  with prob pi one uniformly random agent is lost at a time
                  uniform on [1, H/2] (the R7a/b/d design).  Conditioning on its
                  own survival, each agent weights "partner alive" and
                  "partner dead since tau" (tau <= t) and plans on the mixture of
                  detection probabilities.  pi = 0 == IMPLICIT.
  SECSWEEP(f)     static SECTOR partition until T = round(f*H), then an
                  unrestricted greedy sweep using own looks plus the partner's
                  nominal sector looks finished before T.  T > H == SECTOR.
B. Messaging variants:
  HEALTH, HEARTBEAT  exactly loss_baselines.py (subclassed, unchanged logic).
  HB_TO(P, K)     every agent with a working radio sends a 12 B status frame
                  every P steps (alive, or dead+stamp) and immediately on a
                  sensor state change (death stamp / recovery).  A receiver
                  that hears nothing for K steps infers death at
                  last_alive_stamp + 1; a later 'alive' frame cancels an
                  inferred death; a reported outage is closed by the recovery
                  frame (partner's nominal looks started inside the outage
                  are dropped, later ones assumed again).
Fault kinds added (all with fault seed 10000 + episode, as falsify.py):
  perm        permanent sensor loss at t in [t_lo, t_hi] (== falsify 'loss')
  transient   sensor outage of length L starting in [t_lo, t_hi]
  intermit    p_bad abort probability for L steps starting in [t_lo, t_hi]
  + flags: total=True (radio dies with the sensor: nothing sent from t_dead),
           drop_first=True (first dead-status frame of the lost agent is lost).
"""
from __future__ import annotations

import argparse
import json
import time
from dataclasses import dataclass, replace

import numpy as np

from .event_curve import LINKS as BASE_LINKS
from .falsify import boot, regimes
from .links import GE, LinkConfig
from .loss_baselines import LEpisode, LStrategy, make_strategies as make_l_strategies
from .model import HINT, NULL, Instance
from .sim import HEADER_B, Episode, LinkConfig as _LC, Task, make_sectors, nominal_plan

LINKS = dict(BASE_LINKS)
LINKS["d3_loss0"] = LinkConfig(GE.bernoulli(0.0), delay=3, name="d3_loss0")


# ------------------------------------------------------------- strategies
@dataclass(frozen=True)
class BStrategy(LStrategy):
    hedge: str = ""              # "" | takeover | redundant | bayes | secsweep
    h_f: float = 2.0             # takeover / secsweep switch time as fraction of H
    h_w: float = 1.0             # takeover priority
    h_rho: float = 0.0           # redundant overlap weight
    h_pi: float = 0.0            # bayes prior loss probability
    hb_period: int = 0           # HB_TO period
    hb_timeout: int = 0          # HB_TO silence timeout (steps)


def detect_prob_batch(inst: Instance, M0: np.ndarray, M1: np.ndarray) -> np.ndarray:
    """detect_prob for a stack (K, n, V) of counts; correlated belief with hints
    (the default BeliefModel used by every strategy here)."""
    g, q = inst.g, inst.q
    h1, hb, al = inst.h_good, inst.h_bad, inst.alpha
    with np.errstate(divide="ignore", invalid="ignore"):
        good = g * ((1 - q) * (1 - h1)) ** M0 * ((1 - q) * h1) ** M1
        bad = (1 - g) * (1 - hb) ** M0 * hb ** M1
        absent = M0 * np.log(1 - al) + M1 * np.log(max(al, 1e-300))
        lr = (np.log(good + bad) - absent).sum(axis=2)
        lp = np.log(inst.prior)[None] + lr
        lp -= lp.max(axis=1, keepdims=True)
        w = np.exp(lp)
        post = w / w.sum(axis=1, keepdims=True)
        pg = np.where(good + bad > 0, good / (good + bad), 0.0)
    return post[:, :, None] * pg * q[None]


def make_strategies(tuned: dict | None = None) -> dict:
    """All loss_baselines strategies plus the silent hedges / HB_TO.
    ``tuned`` maps name -> dict of BStrategy fields (from the DEV tuning)."""
    S = make_l_strategies()
    base = S["IMPLICIT"]
    f = {k: getattr(base, k) for k in base.__dataclass_fields__}
    for k in ("HEALTH", "HEARTBEAT"):
        g = dict(f); g.update(name=k, health=S[k].health)
        S[k] = BStrategy(**g)
    sec = S["SECTOR"]
    fs = {k: getattr(sec, k) for k in sec.__dataclass_fields__}
    for nm, kw in (tuned or {}).items():
        kw = dict(kw)
        src = fs if kw.get("hedge") == "secsweep" else f
        g = dict(src); g["name"] = nm
        S[nm] = BStrategy(**g, **kw)
    return S


def hb_to(P, K=None):
    return dict(health="hb_timeout", hb_period=P, hb_timeout=K if K is not None else 2 * P + 1)


@dataclass
class Status:
    t_sent: int
    t_dead: int | None           # None = alive


# ---------------------------------------------------------------- episode
class BEpisode(LEpisode):
    def __init__(self, *a, fault=None, sec_nominal=None, **k):
        super().__init__(*a, fault=fault, **k)
        f = self.fault
        H = self.inst.horizon + 1
        kind = f.get("kind")
        self.radio_dead = np.zeros((2, H), bool)
        if kind in ("perm", "transient", "intermit"):
            r = np.random.default_rng(f.get("seed", 0))
            who = int(r.integers(2)) if f.get("who") is None else f["who"]
            tl = int(r.integers(f["t_lo"], f["t_hi"] + 1))
            bad = np.zeros((2, H), bool)
            if kind == "perm":
                bad[who, tl:] = True
            elif kind == "transient":
                bad[who, tl:tl + f["L"]] = True
            else:
                u = r.random(H)
                win = np.zeros(H, bool); win[tl:tl + f["L"]] = True
                bad[who] = win & (u < f["p_bad"])
            self.bad = bad
            self.t_dead = [int(np.argmax(bad[i])) if bad[i].any() else None for i in range(2)]
            if f.get("total"):
                assert kind == "perm"
                self.radio_dead[who, tl:] = True
        self._dropped_first = set()
        self.sec_nominal = sec_nominal
        # HB_TO receiver state (index = receiver)
        self.last_alive = [0, 0]
        self.intervals = [[], []]        # list of [td, tr|None, inferred]
        self._prev_bad = [False, False]

    # ---------------------------------------------------------- radio
    def _hsend(self, i, t, msg):
        if (i, t) in self._hsent:
            return
        if self.radio_dead[i, min(t, self.radio_dead.shape[1] - 1)]:
            self._hsent.add((i, t))
            return                                   # dead radio: nothing transmitted
        if (self.fault.get("drop_first") and getattr(msg, "t_dead", None) is not None
                and i not in self._dropped_first):
            self._dropped_first.add(i)
            self._hsent.add((i, t))
            self.bytes += HEADER_B
            self.msgs += 1
            return                                   # transmitted but lost
        super()._hsend(i, t, msg)

    def _kind(self):
        hs = {getattr(s, "health", "") for s in self.strats}
        return hs.pop() if len(hs) == 1 else ""

    def _deliver(self, t):
        if self._kind() != "hb_timeout":
            return super()._deliver(t)
        s = self.strats[0]
        tt = min(t, self.inst.horizon)
        for i in range(2):
            b = bool(self.bad[i, tt])
            changed = b != self._prev_bad[i]
            self._prev_bad[i] = b
            if changed or (t > 0 and t % s.hb_period == 0):
                self._hsend(i, t, Status(t, self._stamp(i, t) if b else None))
        for j in range(2):
            ivs = self.intervals[j]
            for m in self.links[1 - j].receive(t):
                if m.t_dead is None:
                    self.last_alive[j] = max(self.last_alive[j], m.t_sent)
                    for iv in list(ivs):
                        if iv[1] is None:
                            if iv[2]:
                                ivs.remove(iv)       # inferred death was false
                            elif m.t_sent > iv[0]:
                                iv[1] = m.t_sent     # reported outage over
                else:
                    if any(iv[1] is None and not iv[2] for iv in ivs):
                        continue
                    for iv in list(ivs):
                        if iv[1] is None and iv[2]:
                            ivs.remove(iv)           # replace inference by report
                    if not any(iv[0] == m.t_dead for iv in ivs):
                        ivs.append([m.t_dead, None, False])
            if (t - self.last_alive[j] > s.hb_timeout
                    and not any(iv[1] is None for iv in ivs)):
                ivs.append([self.last_alive[j] + 1, None, True])

    def _stamp(self, i, t):
        """Start of the current bad run of agent i."""
        k = min(t, self.inst.horizon)
        while k > 0 and self.bad[i, k - 1]:
            k -= 1
        return k

    # ------------------------------------------------------- belief
    def _counts_filtered(self, a, t, keep):
        """Base IMPLICIT partner counts with the partner's nominal list
        filtered by keep(task).  Returns (m0, m1, virtual)."""
        p = 1 - a.idx
        saved = self.nominal
        self.nominal = list(saved)
        self.nominal[p] = [tk for tk in saved[p] if keep(tk)]
        try:
            return Episode._partner_counts(self, a, t)
        finally:
            self.nominal = saved

    def _partner_counts(self, a, t):
        if self._kind() == "hb_timeout" and not self.oracle and self.nominal is not None:
            ivs = self.intervals[a.idx]
            if ivs:
                def keep(tk):
                    return not any(tk.start >= iv[0] and (iv[1] is None or tk.start < iv[1]) for iv in ivs)
                return self._counts_filtered(a, t, keep)
            return Episode._partner_counts(self, a, t)
        return super()._partner_counts(a, t)

    @staticmethod
    def _total(a, pm0, pm1, virtual):
        m0 = a.m0 + pm0
        m1 = a.m1 + pm1
        for (c, v) in virtual:
            m0[c, v] += 1
        return m0, m1

    def _choose(self, a, t):
        s = a.s
        hedge = getattr(s, "hedge", "")
        if not hedge or self.oracle:
            return super()._choose(a, t)
        if self.bad[a.idx, t]:                         # FEpisode fault marking
            self.u[a.idx, min(a.seq, self.inst.horizon), 0] = -1.0
            self.forced.add((a.idx, a.seq))
        inst = self.inst
        H = inst.horizon
        dist = inst.dist[a.pos][:, None] + 1.0
        if hedge == "takeover":
            T = int(round(s.h_f * H))
            if t < T:
                return Episode._choose(self, a, t)
            M0, M1 = [], []
            for keep in (lambda tk: True, lambda tk: tk.start < T):
                m0, m1 = self._total(a, *self._counts_filtered(a, t, keep))
                M0.append(m0); M1.append(m1)
            pd = detect_prob_batch(inst, np.array(M0, float), np.array(M1, float))
            score = np.maximum(pd[0], s.h_w * pd[1]) / dist
        elif hedge == "redundant":
            m0i, m1i = self._total(a, *Episode._partner_counts(self, a, t))
            pd = detect_prob_batch(inst, np.array([m0i, a.m0], float), np.array([m1i, a.m1], float))
            score = ((1 - s.h_rho) * pd[0] + s.h_rho * pd[1]) / dist
        elif hedge == "bayes":
            t_lo, t_hi = 1, H // 2
            ntau = t_hi - t_lo + 1
            taus = list(range(t_lo, min(t, t_hi) + 1))
            pi = s.h_pi
            # joint with "I am alive at t": partner dead since tau: pi*0.5/ntau each;
            # nobody dead yet: 1 - pi*F(t)  (F = fraction of the window elapsed)
            w_dead = pi * 0.5 / ntau
            ws = np.array([1.0 - pi * len(taus) / ntau] + [w_dead] * len(taus))
            ws = ws / ws.sum()
            M0, M1 = [], []
            m0, m1 = self._total(a, *Episode._partner_counts(self, a, t)); M0.append(m0); M1.append(m1)
            for tau in taus:
                m0, m1 = self._total(a, *self._counts_filtered(a, t, lambda tk, tau=tau: tk.start < tau))
                M0.append(m0); M1.append(m1)
            pd = detect_prob_batch(inst, np.array(M0, float), np.array(M1, float))
            score = np.tensordot(ws, pd, axes=1) / dist
        elif hedge == "secsweep":
            T = int(round(s.h_f * H))
            if t < T:
                return Episode._choose(self, a, t)     # sector planner, mask on
            m0 = a.m0.astype(float).copy()
            for tk in self.sec_nominal[1 - a.idx]:
                if tk.finish <= T:
                    m0[tk.cell, tk.view] += 1
            pd = detect_prob_batch(inst, m0[None], a.m1[None].astype(float))[0]
            score = pd / dist
        else:
            raise ValueError(hedge)
        return divmod(int(np.argmax(score)), inst.V)


def sector_nominal(inst: Instance, S: dict, sec: np.ndarray):
    """Both agents' SECTOR plans under all-NULL outcomes, no aborts (common knowledge)."""
    o = S["SECTOR"]
    ep = Episode(replace(inst, abort=0.0), (o, o), _LC(), np.random.default_rng(0), None, sec,
                 force_null=True)
    plans = [[], []]
    orig = ep._choose

    def choose(a, t):
        c, v = orig(a, t)
        plans[a.idx].append(Task(a.seq, c, v, t, t + int(inst.dist[a.pos, c]) + 1))
        return c, v
    ep._choose = choose
    ep.run()
    return plans


class Ctx:
    """Per-instance precomputation shared by all strategies (paired design)."""
    def __init__(self, inst, S):
        self.inst = inst
        self.nom = nominal_plan(inst, S["ORACLE"])
        self.sec = make_sectors(inst)
        self.snom = sector_nominal(inst, S, self.sec)


def run_b(ctx, name, ep, S, fault, link):
    f = dict(fault or {}); f["seed"] = 10_000 + ep
    return BEpisode(ctx.inst, (S[name], S[name]), LINKS[link], np.random.default_rng(ep),
                    ctx.nom, ctx.sec, fault=f, sec_nominal=ctx.snom).run()


# ------------------------------------------------------------ conditions
def builders():
    R = regimes()
    return {"A": R["R7a"][1], "B": R["R7b"][1], "D": R["R7d"][1], "R3a": R["R3a"][1]}


def fault_for(cond: str, H: int):
    h2, h4 = H // 2, max(1, H // 4)
    C = {
        "healthy": None,
        "perm": dict(kind="perm", t_lo=1, t_hi=h2),
        "perm_early": dict(kind="perm", t_lo=1, t_hi=h4),
        "perm_mid": dict(kind="perm", t_lo=h4 + 1, t_hi=h2),
        "perm_late": dict(kind="perm", t_lo=h2 + 1, t_hi=int(0.8 * H)),
        "trans_s": dict(kind="transient", t_lo=1, t_hi=h2, L=max(1, H // 8)),
        "trans_m": dict(kind="transient", t_lo=1, t_hi=h2, L=h4),
        "trans_l": dict(kind="transient", t_lo=1, t_hi=h2, L=h2),
        "intermit": dict(kind="intermit", t_lo=1, t_hi=h4, L=h2, p_bad=0.5),
        "total": dict(kind="perm", t_lo=1, t_hi=h2, total=True),
        "beacon_lost": dict(kind="perm", t_lo=1, t_hi=h2, drop_first=True),
    }
    return C[cond]


SENDERS = ("HEALTH", "HEARTBEAT", "HB_TO", "HYBRID_P")


def is_sender(n):
    return n in SENDERS or n.startswith("HB_TO")


def run_cell(S, ctxs, fault_of, names, links, eps):
    """fault_of(inst) -> fault.  Returns det[(link, name)][seed] arrays and bytes."""
    det, byt = {}, {}
    for sd, ctx in ctxs.items():
        fault = fault_of(ctx.inst)
        for ln in links:
            for n in names:
                if ln != links[0] and not is_sender(n):
                    continue
                rs = [run_b(ctx, n, ep, S, fault, ln) for ep in range(eps)]
                det.setdefault((ln, n), {})[sd] = np.array([r.detected for r in rs], float)
                byt.setdefault((ln, n), []).extend(r.bytes_sent for r in rs)
    return det, byt


def getd(det, ln, n, l0):
    return det[(ln, n)] if (ln, n) in det else det[(l0, n)]


def summ(det, byt, ln, n, l0, ref):
    D = getd(det, ln, n, l0)
    seeds = sorted(D)
    P = float(np.mean(np.concatenate([D[s] for s in seeds])))
    out = {"P": P, "bytes": float(np.mean(byt[(ln, n) if (ln, n) in byt else (l0, n)]))}
    for r in ref:
        if r == n:
            continue
        R = getd(det, ln, r, l0)
        out["d_" + r] = boot({s: D[s] - R[s] for s in seeds})
    return out


def fmt(c):
    m, lo, hi = c
    star = "*" if lo > 0 or hi < 0 else ""
    return f"{m:+.3f} [{lo:+.3f},{hi:+.3f}]{star}"


# ---------------------------------------------------------------- tuning
GRID = {
    **{f"TAKEOVER_f{f}_w{w}": dict(hedge="takeover", h_f=f, h_w=w)
       for f in (0.25, 0.5, 0.75) for w in (0.5, 1.0)},
    **{f"REDUNDANT_r{r}": dict(hedge="redundant", h_rho=r) for r in (0.1, 0.25, 0.5)},
    **{f"BAYES_pi{p}": dict(hedge="bayes", h_pi=p) for p in (0.1, 0.25, 0.5, 1.0)},
    **{f"SECSWEEP_f{f}": dict(hedge="secsweep", h_f=f) for f in (0.25, 0.5, 0.75, 1.01)},
}
LOSS_CELLS = [("A", "perm"), ("B", "perm"), ("D", "perm"), ("A", "perm_early")]   # R7a, R7b, R7d, R7c-like
HEALTHY_CELLS = [("A", "healthy"), ("B", "healthy"), ("D", "healthy"), ("R3a", "healthy")]


def r7_fault(key):
    return regimes()[key][2]


# The four R7 regimes exactly as falsify.py (fault kind 'loss'), plus healthy cells.
R7_CELLS = [("R7a", "A", r7_fault("R7a")), ("R7b", "B", r7_fault("R7b")),
            ("R7c", "A", r7_fault("R7c")), ("R7d", "D", r7_fault("R7d"))]
H_CELLS = [("R0", "A"), ("H-b (5x5 t1 H30)", "B"), ("H-d (8x8 t1 H40)", "D"), ("R3a", "R3a")]


def make_ctxs(bkey, seeds, S):
    b = builders()[bkey]
    return {sd: Ctx(b(sd), S) for sd in seeds}


def tune(seeds, eps, out):
    grid = dict(GRID)
    for P in (2, 3, 5):
        grid[f"HB_TO_P{P}"] = hb_to(P)
    S = make_strategies(grid)
    names = ["IMPLICIT", "HEALTH"] + list(grid)
    res = {n: {} for n in names}
    cells = [(k, b, (lambda inst, f=f: f)) for k, b, f in R7_CELLS] + \
            [(k, b, (lambda inst: None)) for k, b in H_CELLS] + \
            [("total(A)", "A", lambda inst: fault_for("total", inst.horizon)),
             ("total(D)", "D", lambda inst: fault_for("total", inst.horizon))]
    ctx_cache = {}
    t0 = time.time()
    for key, bk, fo in cells:
        if bk not in ctx_cache:
            ctx_cache[bk] = make_ctxs(bk, seeds, S)
        links = ["d0_loss0", "d1_burst30"]
        det, byt = run_cell(S, ctx_cache[bk], fo, names, links, eps)
        for ln in links:
            imp = getd(det, ln, "IMPLICIT", links[0])
            for n in names:
                if ln != links[0] and not is_sender(n):
                    continue
                D = getd(det, ln, n, links[0])
                d = float(np.mean(np.concatenate([D[s] - imp[s] for s in seeds])))
                res[n][(key, ln)] = d
        print(f"[{time.time()-t0:.0f}s] {key}", flush=True)
    lines = [f"DEV tuning seeds {seeds} episodes {eps}: mean d vs IMPLICIT (ideal link unless noted)\n",
             "| strategy | " + " | ".join(k for k, _, _ in cells) + " | burst R7 mean | burst healthy mean | loss mean | healthy mean | balanced |",
             "|---|" + "---|" * (len(cells) + 5)]
    summary = {}
    for n in names:
        L = np.mean([res[n][(k, "d0_loss0")] for k, _, _ in R7_CELLS])
        Hh = np.mean([res[n][(k, "d0_loss0")] for k, _ in H_CELLS])
        bl = np.mean([res[n].get((k, "d1_burst30"), res[n][(k, "d0_loss0")]) for k, _, _ in R7_CELLS])
        bh = np.mean([res[n].get((k, "d1_burst30"), res[n][(k, "d0_loss0")]) for k, _ in H_CELLS])
        summary[n] = dict(loss=L, healthy=Hh, balanced=0.5 * (L + Hh), burst_loss=bl, burst_healthy=bh)
        lines.append(f"| {n} | " + " | ".join(f"{res[n][(k, 'd0_loss0')]:+.3f}" for k, _, _ in cells)
                     + f" | {bl:+.3f} | {bh:+.3f} | {L:+.3f} | {Hh:+.3f} | {0.5*(L+Hh):+.3f} |")
    # choose per family
    chosen = {}
    for fam in ("TAKEOVER", "REDUNDANT", "BAYES", "SECSWEEP"):
        cand = [n for n in grid if n.startswith(fam)]
        lt = max(cand, key=lambda n: summary[n]["loss"])
        bt = max(cand, key=lambda n: summary[n]["balanced"])
        chosen[fam + "_L"] = grid[lt]; chosen[fam + "_B"] = grid[bt]
        lines.append(f"\n{fam}: loss-tuned = {lt}, balanced = {bt}")
    cand = [n for n in grid if n.startswith("HB_TO")]
    # HB_TO: maximise burst-link total-loss gain minus burst healthy cost
    tot = {n: np.mean([res[n][(k, "d1_burst30")] for k in ("total(A)", "total(D)")]) + summary[n]["burst_healthy"]
           for n in cand}
    hb = max(cand, key=lambda n: tot[n])
    chosen["HB_TO"] = grid[hb]
    lines.append(f"HB_TO: chosen {hb} (max burst-link total-loss gain + burst healthy delta: "
                 + ", ".join(f"{n} {v:+.3f}" for n, v in tot.items()) + ")")
    txt = "\n".join(lines)
    print(txt)
    if out:
        open(out, "w").write(txt + "\n")
        json.dump(chosen, open(out.replace(".md", ".json"), "w"), indent=1)


# ----------------------------------------------------------------- final
def final(seeds, eps, tuned_path, out, parts):
    chosen = json.load(open(tuned_path))
    for fam in ("TAKEOVER", "REDUNDANT", "BAYES", "SECSWEEP"):   # identical L/B picks -> keep one
        if chosen.get(fam + "_L") == chosen.get(fam + "_B"):
            chosen.pop(fam + "_B", None)
    S = make_strategies(chosen)
    silent = [n for n in chosen if not n.startswith("HB_TO")]
    lines = [f"FRESH seeds {seeds}, {eps} episodes/cell, paired by episode index (same truth, fault, link draws), "
             "95% cluster bootstrap (instances then episodes, B=2000). * = CI excludes 0.\n"]
    t0 = time.time()
    ctx_cache = {}

    def ctxs(bk):
        if bk not in ctx_cache:
            ctx_cache[bk] = make_ctxs(bk, seeds, S)
        return ctx_cache[bk]

    raw = {}
    if "A" in parts:
        names = ["ORACLE", "IMPLICIT", "INDEP", "SECTOR", "HYBRID_P", "HEALTH", "HEARTBEAT", "HB_TO"] + silent
        links = ["d0_loss0", "d1_burst30"]
        lines.append("\n## Part A: silent hedges vs HEALTH, R7 (loss) and healthy cells\n")
        hdr = "| cell | link | strategy | P(detect) | d vs IMPLICIT | d vs HEALTH | bytes/ep |"
        cells = [(k, b, f) for k, b, f in R7_CELLS] + [(k, b, None) for k, b in H_CELLS]
        for key, bk, f in cells:
            det, byt = run_cell(S, ctxs(bk), lambda inst, f=f: f, names, links, eps)
            raw[key] = (det, byt)
            lines += [f"\n### {key}\n", hdr, "|---|---|---|---|---|---|---|"]
            for ln in links:
                for n in names:
                    if ln != links[0] and not is_sender(n):
                        continue
                    sm = summ(det, byt, ln, n, links[0], ["IMPLICIT", "HEALTH"])
                    lines.append(f"| {key} | {ln} | {n} | {sm['P']:.3f} | {fmt(sm['d_IMPLICIT']) if 'd_IMPLICIT' in sm else '-'} | "
                                 f"{fmt(sm['d_HEALTH']) if 'd_HEALTH' in sm else '-'} | {sm['bytes']:.1f} |")
            print("\n".join(lines[-40:]), f"\n[{time.time()-t0:.0f}s]", flush=True)
            if out:
                open(out, "w").write("\n".join(lines) + "\n")
    if "B" in parts:
        names = ["ORACLE", "IMPLICIT", "INDEP", "HEALTH", "HEARTBEAT", "HB_TO",
                 "TAKEOVER_L", "BAYES_L"]
        names = [n for n in names if n in S]
        lines.append("\n## Part B: loss and abort conditions x links (builders A = 5x5 t0 H16, D = 8x8 t1 H40)\n")
        # compute budget (75 min cap): perm_early/mid, trans_s/l and builder D omitted from the final run
        conds = ["perm_late", "trans_m", "intermit", "total", "beacon_lost"]
        links = ["d0_loss0", "d1_burst30", "d1_loss0", "d3_loss0"]
        D_CONDS = ()
        for bk in ("A", "D"):
            for cond in conds:
                if bk == "D" and cond not in D_CONDS:
                    continue
                lk = links if cond in ("perm_late", "total") else ["d0_loss0", "d1_burst30"]
                det, byt = run_cell(S, ctxs(bk), lambda inst, c=cond: fault_for(c, inst.horizon), names, lk, eps)
                raw[(bk, cond)] = (det, byt)
                fl = fault_for(cond, builders()[bk](seeds[0]).horizon)
                lines += [f"\n### {bk} / {cond}: {fl}\n",
                          "| link | strategy | P(detect) | d vs IMPLICIT | d vs HEALTH | bytes/ep |", "|---|---|---|---|---|---|"]
                for ln in lk:
                    for n in names:
                        if ln != lk[0] and not is_sender(n):
                            continue
                        sm = summ(det, byt, ln, n, lk[0], ["IMPLICIT", "HEALTH"])
                        lines.append(f"| {ln} | {n} | {sm['P']:.3f} | {fmt(sm['d_IMPLICIT']) if 'd_IMPLICIT' in sm else '-'} | "
                                     f"{fmt(sm['d_HEALTH']) if 'd_HEALTH' in sm else '-'} | {sm['bytes']:.1f} |")
                print("\n".join(lines[-25:]), f"\n[{time.time()-t0:.0f}s]", flush=True)
                if out:
                    open(out, "w").write("\n".join(lines) + "\n")
    if out:
        open(out, "w").write("\n".join(lines) + "\n")
        np.save(out.replace(".md", "_raw.npy"),
                np.array([{str(k): ({f"{a}|{b}": {int(s): v.tolist() for s, v in d.items()} for (a, b), d in det.items()},
                                    {f"{a}|{b}": float(np.mean(v)) for (a, b), v in byt.items()})
                           for k, (det, byt) in raw.items()}], dtype=object), allow_pickle=True)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("mode", choices=["tune", "final"])
    ap.add_argument("--seeds", type=int, nargs="+", default=[500, 501, 502, 503])
    ap.add_argument("--episodes", type=int, default=200)
    ap.add_argument("--tuned", default="ForestSearch/results/beacon_tuning.json")
    ap.add_argument("--parts", nargs="+", default=["A", "B"])
    ap.add_argument("--out", default=None)
    a = ap.parse_args()
    if a.mode == "tune":
        tune(a.seeds, a.episodes, a.out)
    else:
        final(a.seeds, a.episodes, a.tuned, a.out, a.parts)


if __name__ == "__main__":
    main()
