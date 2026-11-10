"""Team-size scaling study: N in {2, 3, 5, 10} searchers (new file; nothing else edited).

Extends nagent.NEpisode (subclass) with
  * starts for N > 4: the 4 nagent corners, then farthest-point sampling
    (Manhattan on grid coordinates, lowest index on ties).  N <= 4 == nagent.
  * permanent sensor loss (falsify 'loss' == beacon_robustness 'perm'):
    k agents lose their sensor at independent random times t ~ U[t_lo, t_hi];
    every look started while dead aborts (private to the looker).  The radio
    stays up unless total=True.  k = 1 uses exactly the 2-agent draw order
    (who = r.integers(N), then t), so N = 2 equals FEpisode/BEpisode.
    ORACLE is told the partner is dead (its in-flight look is not virtual).
  * HEALTH (loss_baselines): a dead agent broadcasts ONE 12 B frame per peer
    carrying the death stamp; receivers drop the dead agent's nominal tasks
    starting at/after the stamp.
  * HB_TO (beacon_robustness): 12 B status frame to each peer every P steps
    and on a sensor state change; silence for K steps => inferred death at
    last_alive + 1.  State is kept per directed pair.
  * links: Bernoulli / Gilbert-Elliott per directed pair (links.LinkConfig);
    base relay (hops = 2) either per-pair independent (nagent default) or
    shared (relay_shared=True: source i's uplink channel is shared by all its
    receivers, receiver j's downlink is shared by all sources; equal to the
    per-pair model at N = 2, draw for draw).
  * travel (inst.dist; travel_scale) and the rollout-lite planner of sim.py
    (rollout_k candidates, greedy continuation for rollout_budget steps).

CLI
  cd Tools && fsrun python3 -m ForestSearch.nscale run --phase dev --N 5 --out X.csv.gz ...
  python3 -m ForestSearch.nscale analyze --inputs a.csv.gz ... --out results/nscale_heldout.md
"""
from __future__ import annotations

import argparse
import csv
import gzip
import math
import sys
import time
from dataclasses import dataclass, replace

import numpy as np

from .event_curve import events
from .instances import grid_instance
from .links import GE, DirectedLink, LinkConfig
from .model import (ABORT, DETECT, HINT, NULL, Truth, detect_prob, sample_truth)
from .nagent import NEpisode, starts_for as _starts4
from .sim import HEADER_B, Strategy, Task
from .variants import make_all_strategies

# ---------------------------------------------------------------- config
LINKS = {
    "ideal": LinkConfig(name="ideal"),
    "d1": LinkConfig(GE.bernoulli(0.0), delay=1, name="d1"),
    "burst30_d1": LinkConfig(GE.bursty(0.3, 10), delay=1, name="burst30_d1"),
    "bern30_d1": LinkConfig(GE.bernoulli(0.3), delay=1, name="bern30_d1"),
    "relay_d1": LinkConfig(GE.bursty(0.3, 10), delay=1, hops=2, name="relay_d1"),
}
HB_P, HB_K = 3, 7                      # tuned in beacon_tuning.json (dev seeds 500-503)
PROTOS = ["ORACLE", "INDEP", "IMPLICIT", "HYBRID", "HYBRID_P", "HEALTH", "HB_TO"]
LINK_FREE = {"ORACLE", "INDEP", "IMPLICIT"}      # never transmit
SILENT_WHEN_HEALTHY = {"HEALTH"}                  # transmits only on a fault


@dataclass(frozen=True)
class NSStrategy(Strategy):
    hybrid: bool = False
    predict_after_dev: bool = False
    health: str = ""          # "" | beacon | hb_timeout
    hb_period: int = 0
    hb_timeout: int = 0


def make_strategies(planner: str = "greedy") -> dict:
    kw = {} if planner == "greedy" else dict(rollout_k=3, rollout_budget=8)
    S = make_all_strategies(**kw)
    base = S["IMPLICIT"]
    f = {k: getattr(base, k) for k in base.__dataclass_fields__}
    S["HEALTH"] = NSStrategy(**{**f, "name": "HEALTH"}, health="beacon")
    S["HB_TO"] = NSStrategy(**{**f, "name": "HB_TO"}, health="hb_timeout",
                            hb_period=HB_P, hb_timeout=HB_K)
    return S


def starts_n(inst, N: int) -> tuple:
    if N <= 4:
        return _starts4(inst, N)
    xy = inst.coords
    D = np.abs(xy[:, None, :] - xy[None, :, :]).sum(-1)
    ch = list(_starts4(inst, 4))
    while len(ch) < N:
        md = D[:, ch].min(1)
        md[ch] = -1
        ch.append(int(np.argmax(md)))
    return tuple(ch)


@dataclass
class HealthMsg:
    t_dead: int | None


@dataclass
class Status:
    t_sent: int
    t_dead: int | None


class RelayLink:
    """Two-hop link whose uplink belongs to the source and downlink to the receiver."""

    def __init__(self, cfg, up: DirectedLink, down: DirectedLink):
        self.cfg, self.up, self.down = cfg, up, down
        self.queue = []

    def send(self, t, msg):
        cfg, ch = self.cfg, self.cfg.channel
        tt = t
        for k, L in ((0, self.up), (1, self.down)):
            loss = ch.loss_bad if L.bad[k, tt] else ch.loss_good
            if L.u_loss[k, tt] < loss:
                return False
            tt += cfg.delay
        self.queue.append((t + 2 * cfg.delay, msg))
        return True

    def receive(self, t):
        out = [m for (ta, m) in self.queue if ta <= t]
        self.queue = [(ta, m) for (ta, m) in self.queue if ta > t]
        return out


# ---------------------------------------------------------------- episode
class NSEpisode(NEpisode):
    def __init__(self, inst, strats, link, rng, nominal=None, force_null=False,
                 keep_log=False, fault=None, relay_shared=False):
        # Same draw order as NEpisode.__init__ (truth, u, links (i, j) lexicographic).
        self.inst = inst
        self.N = N = len(strats)
        self.strats = strats
        self.force_null = force_null
        self.truth: Truth = sample_truth(inst, rng)
        self.u = rng.random((N, inst.horizon + 1, 3))
        self.links = {}
        for i in range(N):
            for j in range(N):
                if i != j:
                    self.links[(i, j)] = DirectedLink(link, inst.horizon, rng)
        if relay_shared and link.hops == 2:
            base = dict(self.links)
            for (i, j) in base:
                up = base[(i, min(x for x in range(N) if x != i))]
                down = base[(min(x for x in range(N) if x != j), j)]
                self.links[(i, j)] = RelayLink(link, up, down)
        self.nominal = nominal
        self.starts = starts_n(inst, N)
        from .nagent import _A
        self.agents = [_A(i, strats[i], inst, self.starts[i], N) for i in range(N)]
        self.oracle = any(s.planner == "oracle" for s in strats)
        self.log = [] if keep_log else None
        self.bytes = self.msgs = self.delivered = 0
        self.looked_by = [set() for _ in range(N)]
        self._framed = set()
        self.collisions = 0
        # ---- faults (separate rng, as falsify / beacon_robustness)
        self.fault = f = fault or {}
        H = inst.horizon + 1
        r = np.random.default_rng(f.get("seed", 0))
        bad = np.zeros((N, H), bool)
        self.radio_dead = np.zeros((N, H), bool)
        if f.get("kind") == "perm":
            k = f.get("k", 1)
            if k == 1:
                who = [int(r.integers(N)) if f.get("who") is None else f["who"]]
                tls = [int(r.integers(f["t_lo"], f["t_hi"] + 1))]
            else:
                who = [int(x) for x in r.choice(N, k, replace=False)]
                tls = [int(x) for x in r.integers(f["t_lo"], f["t_hi"] + 1, size=k)]
            for w, tl in zip(who, tls):
                bad[w, tl:] = True
                if f.get("total"):
                    self.radio_dead[w, tl:] = True
        self.bad = bad
        self.forced = set()
        self.t_dead = [int(np.argmax(bad[i])) if bad[i].any() else None for i in range(N)]
        # ---- health state (index [receiver][source])
        self.p_dead = [dict() for _ in range(N)]
        self._hsent = set()
        self.last_alive = [[0] * N for _ in range(N)]
        self.intervals = [[[] for _ in range(N)] for _ in range(N)]
        self._prev_bad = [False] * N
        hs = {getattr(s, "health", "") for s in strats}
        self.hkind = hs.pop() if len(hs) == 1 else ""

    def _partner_pos_guess(self, a, j):
        recs = a.p_records[j]
        if recs:
            return max(recs.values(), key=lambda r: r.finish).cell
        return self.starts[j]

    # -------------------------------------------------------- belief
    def _filtered(self, a, t, keep):
        saved = self.nominal
        self.nominal = [[tk for tk in saved[j] if keep(j, tk)] if j != a.idx else saved[j]
                        for j in range(self.N)]
        try:
            return NEpisode._partner_counts(self, a, t)
        finally:
            self.nominal = saved

    def _partner_counts(self, a, t):
        if self.oracle:
            m0, m1 = self._zeros()
            virtual = []
            for p in self.agents:
                if p.idx == a.idx:
                    continue
                for r in p.records:
                    self._add(m0, m1, r)
                if p.task is not None and (p.idx, p.task.seq) not in self.forced:
                    virtual.append((p.task.cell, p.task.view))
            return m0, m1, virtual
        if self.nominal is not None and self.hkind == "beacon" and self.p_dead[a.idx]:
            pd = self.p_dead[a.idx]
            return self._filtered(a, t, lambda j, tk: j not in pd or tk.start < pd[j])
        if self.nominal is not None and self.hkind == "hb_timeout":
            ivs = self.intervals[a.idx]
            if any(ivs[j] for j in range(self.N)):
                def keep(j, tk):
                    return not any(tk.start >= iv[0] and (iv[1] is None or tk.start < iv[1])
                                   for iv in ivs[j])
                return self._filtered(a, t, keep)
        return NEpisode._partner_counts(self, a, t)

    def _choose(self, a, t):
        inst = self.inst
        if self.bad[a.idx, t]:
            self.u[a.idx, min(a.seq, inst.horizon), 0] = -1.0
            self.forced.add((a.idx, a.seq))
        pm0, pm1, virtual = self._partner_counts(a, t)
        m0 = a.m0 + pm0
        m1 = a.m1 + pm1
        for (c, v) in virtual:
            m0[c, v] += 1
        bm = a.s.belief
        pd = detect_prob(inst, m0, m1, bm)
        score = pd / (inst.dist[a.pos][:, None] + 1.0)
        if a.s.rollout_k <= 0:
            return divmod(int(np.argmax(score)), inst.V)
        # rollout-lite: identical to sim.Episode._choose (mask = None)
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
                c, v = divmod(int(np.argmax(sc)), inst.V)
            if val > best_val + 1e-12:
                best, best_val = int(k), val
        return divmod(best, inst.V)

    # ------------------------------------------------------- radio
    def _hsend(self, i, t, msg):
        if (i, t) in self._hsent:
            return
        self._hsent.add((i, t))
        if self.radio_dead[i, min(t, self.radio_dead.shape[1] - 1)]:
            return
        for j in range(self.N):
            if j == i:
                continue
            self.bytes += HEADER_B
            self.msgs += 1
            if self.links[(i, j)].send(t, msg):
                self.delivered += 1

    def _stamp(self, i, t):
        k = min(t, self.inst.horizon)
        while k > 0 and self.bad[i, k - 1]:
            k -= 1
        return k

    def _deliver(self, t):
        N = self.N
        if self.hkind == "beacon":
            for i in range(N):
                td = self.t_dead[i]
                if td is not None and td <= t and (i, "b") not in self._hsent:
                    self._hsent.add((i, "b"))
                    self._hsend(i, t, HealthMsg(td))
            for j in range(N):
                for i in range(N):
                    if i == j:
                        continue
                    for m in self.links[(i, j)].receive(t):
                        if m.t_dead is not None and i not in self.p_dead[j]:
                            self.p_dead[j][i] = m.t_dead
            return
        if self.hkind == "hb_timeout":
            s = self.strats[0]
            tt = min(t, self.inst.horizon)
            for i in range(N):
                b = bool(self.bad[i, tt])
                changed = b != self._prev_bad[i]
                self._prev_bad[i] = b
                if changed or (t > 0 and t % s.hb_period == 0):
                    self._hsend(i, t, Status(t, self._stamp(i, t) if b else None))
            for j in range(N):
                for i in range(N):
                    if i == j:
                        continue
                    ivs = self.intervals[j][i]
                    for m in self.links[(i, j)].receive(t):
                        if m.t_dead is None:
                            self.last_alive[j][i] = max(self.last_alive[j][i], m.t_sent)
                            for iv in list(ivs):
                                if iv[1] is None:
                                    if iv[2]:
                                        ivs.remove(iv)
                                    elif m.t_sent > iv[0]:
                                        iv[1] = m.t_sent
                        else:
                            if any(iv[1] is None and not iv[2] for iv in ivs):
                                continue
                            for iv in list(ivs):
                                if iv[1] is None and iv[2]:
                                    ivs.remove(iv)
                            if not any(iv[0] == m.t_dead for iv in ivs):
                                ivs.append([m.t_dead, None, False])
                    if (t - self.last_alive[j][i] > s.hb_timeout
                            and not any(iv[1] is None for iv in ivs)):
                        ivs.append([self.last_alive[j][i] + 1, None, True])
            return
        return NEpisode._deliver(self, t)


def nominal_plan_ns(inst, N: int, S: dict) -> list:
    o = replace(S["ORACLE"], name="ORACLE", planner="oracle", send="none",
                predict_nominal=False)
    ep = NSEpisode(replace(inst, abort=0.0), (o,) * N, LinkConfig(),
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


def run_ns(inst, N, name, link, ep, S, nominal=None, fault=None, relay_shared=False,
           keep_log=False):
    if nominal is None:
        nominal = nominal_plan_ns(inst, N, S)
    f = None
    if fault:
        f = dict(fault); f["seed"] = 10_000 + ep
    return NSEpisode(inst, (S[name],) * N, link, np.random.default_rng(ep), nominal,
                     keep_log=keep_log, fault=f, relay_shared=relay_shared).run()


# ---------------------------------------------------------------- design
GRID = {2: (5, 5), 3: (6, 6), 5: (8, 8), 10: (11, 11)}    # ~12 cells per agent
# Horizons per (N, travel): set on dev seeds (results/nscale_dev.md); frozen in PREREG_SCALING.md.
HORIZON = {(2, 0): 16, (3, 0): 16, (5, 0): 16, (10, 0): 16,
           (2, 1): 30, (3, 1): 30, (5, 1): 30, (10, 1): 30}


def build(N, seed, lam, travel, horizon=None):
    w, h = GRID[N]
    hz = horizon or HORIZON[(N, travel)]
    return grid_instance(w, h, seed=seed, travel_scale=travel, horizon=hz, **events(lam))


def fault_spec(name, H, N):
    if name == "none":
        return None
    if name == "perm1":
        return dict(kind="perm", k=1, t_lo=1, t_hi=H // 2)
    if name == "permk":
        return dict(kind="perm", k=max(1, math.ceil(N / 5)), t_lo=1, t_hi=H // 2)
    raise ValueError(name)


FIELDS = ["N", "travel", "lam", "planner", "fault", "link", "proto", "seed", "ep",
          "detected", "ttd", "H", "bytes", "collisions"]


def run_grid(N, seeds, lams, travels, planners, faults, links, protos, E, out, horizon=None):
    t0 = time.time()
    rows = []
    for planner in planners:
        S = make_strategies(planner)
        for travel in travels:
            for lam in lams:
                for seed in seeds:
                    inst = build(N, seed, lam, travel, horizon)
                    H = inst.horizon
                    nom = nominal_plan_ns(inst, N, S)
                    for fname in faults:
                        fs = fault_spec(fname, H, N)
                        for p in protos:
                            lks = [links[0]] if (p in LINK_FREE or
                                                 (fs is None and p in SILENT_WHEN_HEALTHY)) else links
                            for ln in lks:
                                for ep in range(E):
                                    r = run_ns(inst, N, p, LINKS[ln], ep, S, nom, fs)
                                    rec = (N, travel, lam, planner, fname, ln, p, seed, ep,
                                           int(r.detected), r.ttd, H, r.bytes_sent, r.collisions)
                                    rows.append(rec)
                                    if ln == links[0] and len(lks) == 1:
                                        for ln2 in links[1:]:   # link-free: copy row
                                            rows.append(rec[:5] + (ln2,) + rec[6:])
                    print(f"[{time.time()-t0:7.0f}s] N={N} {planner} travel={travel} lam={lam} "
                          f"seed={seed}", file=sys.stderr, flush=True)
    with gzip.open(out, "wt", newline="") as fh:
        wr = csv.writer(fh)
        wr.writerow(FIELDS)
        wr.writerows(rows)
    print(f"wrote {out} ({len(rows)} rows, {time.time()-t0:.0f}s)", file=sys.stderr)


# ---------------------------------------------------------------- stats
def _betacf(a, b, x):
    qab, qap, qam = a + b, a + 1.0, a - 1.0
    c, d = 1.0, 1.0 - qab * x / qap
    d = 1.0 / (d if abs(d) > 1e-300 else 1e-300)
    h = d
    for m in range(1, 300):
        m2 = 2 * m
        aa = m * (b - m) * x / ((qam + m2) * (a + m2))
        d = 1.0 + aa * d; d = 1.0 / (d if abs(d) > 1e-300 else 1e-300)
        c = 1.0 + aa / c if abs(c) > 1e-300 else 1e-300
        h *= d * c
        aa = -(a + m) * (qab + m) * x / ((a + m2) * (qap + m2))
        d = 1.0 + aa * d; d = 1.0 / (d if abs(d) > 1e-300 else 1e-300)
        c = 1.0 + aa / c if abs(c) > 1e-300 else 1e-300
        de = d * c
        h *= de
        if abs(de - 1.0) < 1e-12:
            break
    return h


def _ibeta(a, b, x):
    if x <= 0:
        return 0.0
    if x >= 1:
        return 1.0
    bt = math.exp(math.lgamma(a + b) - math.lgamma(a) - math.lgamma(b)
                  + a * math.log(x) + b * math.log(1 - x))
    if x < (a + 1) / (a + b + 2):
        return bt * _betacf(a, b, x) / a
    return 1.0 - bt * _betacf(b, a, 1 - x) / b


def t_cdf(t, df):
    x = df / (df + t * t)
    p = 0.5 * _ibeta(df / 2.0, 0.5, x)
    return 1.0 - p if t > 0 else p


def t_ppf(q, df):
    lo, hi = -50.0, 50.0
    for _ in range(100):
        mid = (lo + hi) / 2
        if t_cdf(mid, df) < q:
            lo = mid
        else:
            hi = mid
    return (lo + hi) / 2


def inst_t(x):
    """Instance-level mean, se, df of per-instance means x."""
    x = np.asarray(x, float)
    n = len(x)
    sd = x.std(ddof=1) if n > 1 else 0.0
    return float(x.mean()), float(sd / math.sqrt(n)) if n > 1 else 0.0, n - 1


def p_greater(x, margin=0.0):
    """One-sided p for H1: mean > margin (instance-level t)."""
    m, se, df = inst_t(x)
    if se == 0:
        return 0.0 if m > margin else 1.0
    return 1.0 - t_cdf((m - margin) / se, df)


def p_less(x, margin=0.0):
    m, se, df = inst_t(x)
    if se == 0:
        return 0.0 if m < margin else 1.0
    return t_cdf((m - margin) / se, df)


def holm(ps):
    """Holm step-down adjusted p-values."""
    ps = np.asarray(ps, float)
    order = np.argsort(ps)
    adj = np.empty_like(ps)
    run = 0.0
    m = len(ps)
    for r, i in enumerate(order):
        run = max(run, min(1.0, (m - r) * ps[i]))
        adj[i] = run
    return adj


def cluster_boot(diff: dict, B=2000, rng=None, stat=np.mean):
    """diff: seed -> per-episode array.  Returns bootstrap replicates of the pooled mean
    (instances resampled, then episodes within instance)."""
    rng = rng or np.random.default_rng(1)
    seeds = sorted(diff)
    arrs = [np.asarray(diff[s], float) for s in seeds]
    k = len(seeds)
    out = np.empty(B)
    for b in range(B):
        ss = rng.integers(k, size=k)
        out[b] = np.mean(np.concatenate([arrs[s][rng.integers(len(arrs[s]), size=len(arrs[s]))]
                                         for s in ss]))
    return out


def power_fit(Ns, y, w=None, bgrid=np.linspace(-2, 3, 501)):
    """Weighted LS fit y = c * N^-b by profile over b.  Returns (b, c)."""
    Ns = np.asarray(Ns, float); y = np.asarray(y, float)
    w = np.ones_like(y) if w is None else np.asarray(w, float)
    best = (np.inf, 0.0, 0.0)
    for b in bgrid:
        x = Ns ** -b
        c = (w * x * y).sum() / (w * x * x).sum()
        sse = (w * (y - c * x) ** 2).sum()
        if sse < best[0]:
            best = (sse, b, c)
    return best[1], best[2]


def lin_fit(xs, y):
    xs = np.asarray(xs, float); y = np.asarray(y, float)
    A = np.vstack([np.ones_like(xs), xs]).T
    coef, *_ = np.linalg.lstsq(A, y, rcond=None)
    return float(coef[0]), float(coef[1])



# ---------------------------------------------------------------- analysis
CONTRASTS = [("ORACLE", "IMPLICIT"), ("HEALTH", "IMPLICIT"), ("HB_TO", "IMPLICIT"),
             ("HYBRID", "IMPLICIT"), ("HYBRID_P", "IMPLICIT"), ("INDEP", "IMPLICIT")]
NS_ALL = (2, 3, 5, 10)


def load(paths):
    """-> D[(N, travel, lam, planner, fault, link, proto)] = (det (16,E), ttd/H (16,E), bytes, seeds)"""
    raw = {}
    for p in paths:
        with gzip.open(p, "rt") as fh:
            for r in csv.DictReader(fh):
                k = (int(r["N"]), float(r["travel"]), float(r["lam"]), r["planner"], r["fault"],
                     r["link"], r["proto"])
                raw.setdefault(k, {}).setdefault(int(r["seed"]), {})[int(r["ep"])] = (
                    int(r["detected"]), int(r["ttd"]) / int(r["H"]), float(r["bytes"]),
                    float(r["collisions"]))
    D = {}
    for k, bys in raw.items():
        seeds = sorted(bys)
        eps = sorted(bys[seeds[0]])
        A = np.array([[bys[s][e] for e in eps] for s in seeds], float)   # (S, E, 4)
        D[k] = dict(det=A[..., 0], ttd=A[..., 1], bytes=A[..., 2], coll=A[..., 3], seeds=seeds)
    return D


def boot_means(X, B, rng):
    """X (S, E) per-episode values; cluster bootstrap replicates of the pooled mean."""
    S_, E = X.shape
    ss = rng.integers(S_, size=(B, S_))
    ee = rng.integers(E, size=(B, S_, E))
    return X[ss[:, :, None], ee].mean(axis=(1, 2))


def contrast(D, key, a, b, metric="det", coef=None):
    """Per-episode paired difference a - b (or a linear combination coef: {proto: w})."""
    if coef is None:
        coef = {a: 1.0, b: -1.0}
    X = None
    for p, w in coef.items():
        Y = D[key + (p,)][metric] * w
        X = Y if X is None else X + Y
    return X


def summarise(X, B=2000, seed=0):
    rng = np.random.default_rng(seed)
    bs = boot_means(X, B, rng)
    inst = X.mean(1)
    m, se, df = inst_t(inst)
    return dict(m=float(X.mean()), lo95=float(np.percentile(bs, 2.5)), hi95=float(np.percentile(bs, 97.5)),
                lo90=float(np.percentile(bs, 5)), hi90=float(np.percentile(bs, 95)), inst=inst,
                se=se, df=df, boots=bs)


def fmt_ci(s, lvl=95):
    lo, hi = (s["lo95"], s["hi95"]) if lvl == 95 else (s["lo90"], s["hi90"])
    star = "*" if lo > 0 or hi < 0 else ""
    return f"{s['m']:+.3f} [{lo:+.3f},{hi:+.3f}]{star}"


def fit_boot(D, keyfn, coef, B=2000, seed=0, metric="det"):
    """Scaling fits over N for a contrast: power law y = c N^-b and linear y = a + s log2 N.
    Bootstrap resamples instances (then episodes) independently within each N."""
    Ns = [N for N in NS_ALL if keyfn(N) + (list(coef)[0],) in D]
    Xs = [contrast(D, keyfn(N), None, None, metric, coef) for N in Ns]
    y = np.array([X.mean() for X in Xs])
    rng = np.random.default_rng(seed)
    BS = np.array([boot_means(X, B, rng) for X in Xs])        # (nN, B)
    var = BS.var(1) + 1e-12
    b, c = power_fit(Ns, y, 1 / var)
    a0, s0 = lin_fit(np.log2(Ns), y)
    bb, ss_ = [], []
    for k in range(B):
        yk = BS[:, k]
        bb.append(power_fit(Ns, yk, 1 / var, bgrid=np.linspace(-2, 3, 101))[0])
        ss_.append(lin_fit(np.log2(Ns), yk)[1])
    bb, ss_ = np.array(bb), np.array(ss_)
    return dict(Ns=Ns, y=y, se=np.sqrt(var), b=b, c=c, b_lo=float(np.percentile(bb, 2.5)),
                b_hi=float(np.percentile(bb, 97.5)), s=s0, a=a0, s_lo=float(np.percentile(ss_, 2.5)),
                s_hi=float(np.percentile(ss_, 97.5)), Ny=y * np.array(Ns))


def analyze(paths, out, B=2000):
    D = load(paths)
    keys = sorted({k[:6] for k in D})
    conds = sorted({(k[1], k[2], k[3]) for k in keys})          # travel, lam, planner
    links = [l for l in ("ideal", "d1", "burst30_d1") if any(k[5] == l for k in keys)]
    faults = [f for f in ("none", "perm1", "permk") if any(k[4] == f for k in keys)]
    L = ["# Team-size scaling: " + ", ".join(paths), "",
         "Per-episode paired differences (common random numbers). Mean [95% cluster-bootstrap CI] "
         f"(instances, then episodes; B={B}); * = CI excludes 0. Instance-level t uses the 16 "
         "per-instance means (df = 15). Holm across N within each hypothesis family.", ""]
    # ---------------- all cells
    L += ["## 1. Every cell: P(detect) and differences vs IMPLICIT", ""]
    for (tr, lam, pl) in conds:
        for fa in faults:
            for ln in links:
                L += [f"### travel={tr:g} lam={lam:g} planner={pl} fault={fa} link={ln}", "",
                      "| N | P IMPLICIT | ORACLE-IMPL | HEALTH-IMPL | HB_TO-IMPL | HYBRID-IMPL | "
                      "HYBRID_P-IMPL | INDEP-IMPL | TTD/H IMPL | TTD/H ORACLE-IMPL | bytes HEALTH/HB_TO/HYBRID_P "
                      "| coll/ep HYBRID/HYBRID_P |",
                      "|---|---|---|---|---|---|---|---|---|---|---|---|"]
                for N in NS_ALL:
                    key = (N, tr, lam, pl, fa, ln)
                    if key + ("IMPLICIT",) not in D:
                        continue
                    cells = []
                    for a, b in CONTRASTS:
                        if key + (a,) in D:
                            cells.append(fmt_ci(summarise(contrast(D, key, a, b), B)))
                        else:
                            cells.append("-")
                    t_imp = D[key + ("IMPLICIT",)]["ttd"].mean()
                    t_o = fmt_ci(summarise(contrast(D, key, "ORACLE", "IMPLICIT", "ttd"), B))
                    by = "/".join(f"{D[key + (p,)]['bytes'].mean():.0f}" if key + (p,) in D else "-"
                                  for p in ("HEALTH", "HB_TO", "HYBRID_P"))
                    co = "/".join(f"{D[key + (p,)]['coll'].mean():.2f}" if key + (p,) in D else "-"
                                  for p in ("HYBRID", "HYBRID_P"))
                    L.append(f"| {N} | {D[key + ('IMPLICIT',)]['det'].mean():.3f} | " + " | ".join(cells)
                             + f" | {t_imp:.3f} | {t_o} | {by} | {co} |")
                L.append("")
    # ---------------- hypotheses
    V = []          # verdict lines
    L += ["## 2. Hypotheses", ""]
    # S1
    L += ["### S1: healthy, ORACLE - IMPLICIT within +-0.03 (TOST, 90% CI; Holm over N)", "",
          "| travel | lam | planner | N | diff [90% CI] | p_TOST | p_Holm | verdict |", "|---|---|---|---|---|---|---|---|"]
    for (tr, lam, pl) in conds:
        rows, ps = [], []
        for N in NS_ALL:
            key = (N, tr, lam, pl, "none", links[0])
            if key + ("ORACLE",) not in D:
                continue
            s = summarise(contrast(D, key, "ORACLE", "IMPLICIT"), B)
            p = max(p_greater(s["inst"], -0.03), p_less(s["inst"], 0.03))
            rows.append((N, s)); ps.append(p)
        if not rows:
            continue
        adj = holm(ps)
        allp = True
        for (N, s), p, pa in zip(rows, ps, adj):
            ok = pa < 0.05 and s["lo90"] > -0.03 and s["hi90"] < 0.03
            allp &= ok
            L.append(f"| {tr:g} | {lam:g} | {pl} | {N} | {fmt_ci(s, 90)} | {p:.4f} | {pa:.4f} | {'PASS' if ok else 'FAIL'} |")
        V.append(f"S1 travel={tr:g} lam={lam:g} {pl}: {'PASS' if allp else 'FAIL'} (all N)")
    L.append("")
    # S2
    L += ["### S2: loss (perm1), HEALTH recovers >= 80% of ORACLE's gain", "",
          "D = (HEALTH-IMPL) - 0.8 (ORACLE-IMPL). Primary: non-inferiority, one-sided 95% lower bound of D > -0.01 "
          "(instance t, Holm over N, and bootstrap 5th percentile). Strict: 5th percentile > 0. "
          "Ratio = (HEALTH-IMPL)/(ORACLE-IMPL), bootstrap ratio of means.", "",
          "| travel | lam | planner | link | N | ORACLE-IMPL | HEALTH-IMPL | D [90% CI] | ratio [95% CI] | p_NI | p_Holm | verdict | strict |",
          "|---|---|---|---|---|---|---|---|---|---|---|---|---|"]
    if "perm1" in faults:
        for (tr, lam, pl) in conds:
            for ln in links:
                rows, ps = [], []
                for N in NS_ALL:
                    key = (N, tr, lam, pl, "perm1", ln)
                    if key + ("HEALTH",) not in D:
                        continue
                    so = summarise(contrast(D, key, "ORACLE", "IMPLICIT"), B, seed=5)
                    sh = summarise(contrast(D, key, "HEALTH", "IMPLICIT"), B, seed=5)
                    sd = summarise(contrast(D, key, None, None, coef={"HEALTH": 1.0, "ORACLE": -0.8,
                                                                    "IMPLICIT": -0.2}), B, seed=5)
                    rat = sh["boots"] / np.where(np.abs(so["boots"]) < 1e-9, np.nan, so["boots"])
                    rl, rh = np.nanpercentile(rat, [2.5, 97.5])
                    p = p_greater(sd["inst"], -0.01)
                    rows.append((N, so, sh, sd, (sh["m"] / so["m"] if so["m"] else np.nan, rl, rh)))
                    ps.append(p)
                if not rows:
                    continue
                adj = holm(ps)
                allp = allst = True
                for (N, so, sh, sd, r), p, pa in zip(rows, ps, adj):
                    ok = pa < 0.05 and sd["lo90"] > -0.01
                    st = sd["lo90"] > 0
                    allp &= ok; allst &= st
                    L.append(f"| {tr:g} | {lam:g} | {pl} | {ln} | {N} | {fmt_ci(so)} | {fmt_ci(sh)} | {fmt_ci(sd, 90)} | "
                             f"{r[0]:.2f} [{r[1]:.2f},{r[2]:.2f}] | {p:.4f} | {pa:.4f} | {'PASS' if ok else 'FAIL'} | {'yes' if st else 'no'} |")
                V.append(f"S2 travel={tr:g} lam={lam:g} {pl} {ln}: {'PASS' if allp else 'FAIL'} (strict {'PASS' if allst else 'FAIL'})")
    L.append("")
    # S3
    L += ["### S3a: healthy, stale-plan penalty IMPLICIT - HYBRID grows with N (slope in log2 N > 0)", "",
          "| travel | lam | planner | link | penalty at N=2,3,5,10 | slope per doubling [95% CI] | verdict |",
          "|---|---|---|---|---|---|---|"]
    for (tr, lam, pl) in conds:
        for ln in [l for l in links if l != "ideal"]:
            kf = lambda N: (N, tr, lam, pl, "none", ln)
            if kf(2) + ("HYBRID",) not in D:
                continue
            f = fit_boot(D, kf, {"IMPLICIT": 1.0, "HYBRID": -1.0}, B)
            ok = f["s_lo"] > 0
            L.append(f"| {tr:g} | {lam:g} | {pl} | {ln} | " + ", ".join(f"{v:+.3f}" for v in f["y"])
                     + f" | {f['s']:+.4f} [{f['s_lo']:+.4f},{f['s_hi']:+.4f}] | {'PASS' if ok else 'FAIL'} |")
            V.append(f"S3a travel={tr:g} lam={lam:g} {pl} {ln}: {'PASS' if ok else 'FAIL'}")
    L += ["", "### S3b: healthy, HYBRID_P - IMPLICIT within +-0.015 (TOST, 90% CI; Holm over N)", "",
          "| travel | lam | planner | link | N | diff [90% CI] | p_TOST | p_Holm | verdict |", "|---|---|---|---|---|---|---|---|---|"]
    for (tr, lam, pl) in conds:
        for ln in [l for l in links if l != "ideal"]:
            rows, ps = [], []
            for N in NS_ALL:
                key = (N, tr, lam, pl, "none", ln)
                if key + ("HYBRID_P",) not in D:
                    continue
                s = summarise(contrast(D, key, "HYBRID_P", "IMPLICIT"), B)
                rows.append((N, s)); ps.append(max(p_greater(s["inst"], -0.015), p_less(s["inst"], 0.015)))
            if not rows:
                continue
            adj = holm(ps)
            allp = True
            for (N, s), p, pa in zip(rows, ps, adj):
                ok = pa < 0.05 and s["lo90"] > -0.015 and s["hi90"] < 0.015
                allp &= ok
                L.append(f"| {tr:g} | {lam:g} | {pl} | {ln} | {N} | {fmt_ci(s, 90)} | {p:.4f} | {pa:.4f} | {'PASS' if ok else 'FAIL'} |")
            V.append(f"S3b travel={tr:g} lam={lam:g} {pl} {ln}: {'PASS' if allp else 'FAIL'}")
    L.append("")
    # S4
    L += ["### S4: loss (perm1), VoC = ORACLE - IMPLICIT scales as N^-b with b = 1", "",
          "PASS iff the 95% bootstrap CI of b contains 1 and its lower bound > 0. N*VoC should be flat if b = 1.", "",
          "| travel | lam | planner | VoC at N=2,3,5,10 | N*VoC | b [95% CI] | c | verdict |", "|---|---|---|---|---|---|---|---|"]
    if "perm1" in faults:
        for (tr, lam, pl) in conds:
            kf = lambda N: (N, tr, lam, pl, "perm1", links[0])
            if kf(2) + ("ORACLE",) not in D:
                continue
            f = fit_boot(D, kf, {"ORACLE": 1.0, "IMPLICIT": -1.0}, B)
            ok = f["b_lo"] <= 1 <= f["b_hi"] and f["b_lo"] > 0
            L.append(f"| {tr:g} | {lam:g} | {pl} | " + ", ".join(f"{v:+.3f}" for v in f["y"]) + " | "
                     + ", ".join(f"{v:+.3f}" for v in f["Ny"]) + f" | {f['b']:.2f} [{f['b_lo']:.2f},{f['b_hi']:.2f}] | {f['c']:.3f} | {'PASS' if ok else 'FAIL'} |")
            V.append(f"S4 travel={tr:g} lam={lam:g} {pl}: {'PASS' if ok else 'FAIL'} (b={f['b']:.2f} [{f['b_lo']:.2f},{f['b_hi']:.2f}])")
    L.append("")
    # ---------------- fits
    L += ["## 3. Scaling-law fits (all conditions, all contrasts)", "",
          "y_N = c N^-b (weighted LS, weights 1/bootstrap var) and y_N = a + s log2 N; 95% CIs by "
          "bootstrap within N.", "",
          "| travel | lam | planner | fault | link | contrast | y at N=2,3,5,10 | b [95% CI] | s per doubling [95% CI] |",
          "|---|---|---|---|---|---|---|---|---|"]
    for (tr, lam, pl) in conds:
        for fa in faults:
            for ln in links:
                for a, b in CONTRASTS:
                    kf = lambda N: (N, tr, lam, pl, fa, ln)
                    if kf(2) + (a,) not in D or len([N for N in NS_ALL if kf(N) + (a,) in D]) < 3:
                        continue
                    if a in LINK_FREE and ln != links[0]:
                        continue
                    if fa == "none" and a == "HEALTH":
                        continue
                    f = fit_boot(D, kf, {a: 1.0, b: -1.0}, B, seed=3)
                    L.append(f"| {tr:g} | {lam:g} | {pl} | {fa} | {ln} | {a}-{b} | " + ", ".join(f"{v:+.3f}" for v in f["y"])
                             + f" | {f['b']:.2f} [{f['b_lo']:.2f},{f['b_hi']:.2f}] | {f['s']:+.4f} [{f['s_lo']:+.4f},{f['s_hi']:+.4f}] |")
    L += ["", "## 4. Verdict summary", ""] + [f"- {v}" for v in V]
    open(out, "w").write("\n".join(L) + "\n")
    print("\n".join(V))
    print("wrote", out)



# ---------------------------------------------------------------- phases
import hashlib  # noqa: E402
import json  # noqa: E402
import os  # noqa: E402

HERE = os.path.dirname(os.path.abspath(__file__))
H_CAND = {0: [8, 12, 16, 20], 1: [16, 24, 32, 40]}
H_TARGET = 0.60      # dev rule: P(IMPLICIT, healthy, lam=1) closest to 0.60


def sha(path):
    return hashlib.sha256(open(path, "rb").read()).hexdigest()


def dev_phase(out_dir, seeds_h=(1000, 1001, 1002, 1003), E_h=20, seeds=tuple(range(1000, 1006)),
              E=25, Ns=NS_ALL, travels=(0, 1), scan_only=False):
    t0 = time.time()
    S = make_strategies("greedy")
    info = {"horizon": {}, "P_scan": {}, "ms_per_ep": {}, "rollout": {}}
    log = lambda m: print(f"[{time.time()-t0:7.0f}s] {m}", file=sys.stderr, flush=True)
    for N in Ns:
        for tr in travels:
            best = None
            for H in H_CAND[tr]:
                det = []
                for sd in seeds_h:
                    inst = build(N, sd, 1, tr, H)
                    nom = nominal_plan_ns(inst, N, S)
                    det += [run_ns(inst, N, "IMPLICIT", LINKS["ideal"], ep, S, nom).detected for ep in range(E_h)]
                P = float(np.mean(det))
                info["P_scan"][f"{N},{tr},{H}"] = P
                if best is None or abs(P - H_TARGET) < abs(best[1] - H_TARGET):
                    best = (H, P)
            info["horizon"][f"{N},{tr}"] = best[0]
            H = best[0]
            # timing at the chosen horizon (lam 3, worst case for messaging)
            inst = build(N, seeds_h[0], 3, tr, H)
            nom = nominal_plan_ns(inst, N, S)
            fs = fault_spec("perm1", H, N)
            for p in PROTOS:
                tt = time.time()
                for ep in range(6):
                    run_ns(inst, N, p, LINKS["burst30_d1"], ep, S, nom, fs)
                info["ms_per_ep"][f"{N},{tr},{p}"] = (time.time() - tt) / 6 * 1000
            log(f"N={N} travel={tr} H={H} P={best[1]:.2f} ms/ep " +
                " ".join(f"{p}:{info['ms_per_ep'][f'{N},{tr},{p}']:.0f}" for p in PROTOS))
    # rollout-lite timing and effect (small)
    Sr = make_strategies("rollout")
    for N in Ns:
        H = info["horizon"][f"{N},1"]
        res = {}
        for p in ("ORACLE", "IMPLICIT", "HYBRID_P"):
            tt = time.time(); det = []; n = 0
            for sd in seeds_h[:2]:
                inst = build(N, sd, 1, 1, H)
                nom = nominal_plan_ns(inst, N, Sr)
                for ep in range(8):
                    det.append(run_ns(inst, N, p, LINKS["d1"], ep, Sr, nom).detected); n += 1
            res[p] = (float(np.mean(det)), (time.time() - tt) / n * 1000)
        info["rollout"][str(N)] = res
        log(f"rollout N={N}: " + " ".join(f"{p}:P={v[0]:.2f},{v[1]:.0f}ms" for p, v in res.items()))
    json.dump(info, open(os.path.join(out_dir, "nscale_dev_info.json"), "w"), indent=1)
    if scan_only:
        return
    outs = []
    for N in Ns:
        for tr in travels:
            o = os.path.join(HERE, "results", "data", f"nscale_dev_N{N}_t{tr}.csv.gz")
            run_grid(N, list(seeds), [1, 3], [tr], ["greedy"], ["none", "perm1"],
                     ["ideal", "d1", "burst30_d1"], PROTOS, E, o, info["horizon"][f"{N},{tr}"])
            outs.append(o)
            log(f"dev grid N={N} travel={tr} done")
    analyze(outs, os.path.join(out_dir, "nscale_dev_analysis.md"), B=500)
    log("dev done")


def heldout_phase(part, index=None):
    res = os.path.join(HERE, "results")
    man = os.path.join(res, "nscale_manifest.txt")
    params_p = os.path.join(res, "nscale_heldout_params.json")
    lines = dict(l.split()[::-1] for l in open(man) if l.strip() and not l.startswith("#"))
    for f in ("nscale.py", "PREREG_SCALING.md", "results/nscale_heldout_params.json"):
        h = sha(os.path.join(HERE, f))
        if lines.get(f) != h:
            sys.exit(f"hash mismatch for {f}: refusing to run held-out")
    P = json.load(open(params_p))
    tasks = [P["tasks"][index]] if index is not None else P["parts"][part]
    for task in tasks:
        o = os.path.join(res, "data", task["out"])
        if os.path.exists(o):
            print("exists, skip", o, file=sys.stderr)
            continue
        run_grid(task["N"], P["seeds"], P["lams"], [task["travel"]], task["planners"], task["faults"],
                 P["links"], task.get("protos", PROTOS), task["E"], o + ".part", task["H"])
        os.replace(o + ".part", o)



def make_params(budget_cpu_min, travels, planners, info_path, out_path):
    """Frozen rule (PREREG_SCALING.md): episodes per instance E for N < 10 and
    E10 = max(40, int(0.6 E)) for N = 10, with E the largest multiple of 10 in [40, 200]
    whose estimated CPU (dev ms/episode, lam 3, burst link, perm1) fits the budget;
    if none fits, E = 40 / E10 = 30.  Horizons come from the dev scan rule."""
    info = json.load(open(info_path))

    def cost(N, tr, pl, E):
        f = 1.0
        if pl != "greedy":
            f = info["rollout"][str(N)]["HYBRID_P"][1] / max(1e-9, info["ms_per_ep"][f"{N},1,HYBRID_P"])
        ms = lambda p: info["ms_per_ep"][f"{N},{tr},{p}"] * f
        per = (2 * sum(ms(p) for p in ("ORACLE", "INDEP", "IMPLICIT"))
               + 3 * sum(ms(p) for p in ("HYBRID", "HYBRID_P", "HB_TO"))
               + 3 * sum(ms(p) for p in ("HYBRID", "HYBRID_P", "HB_TO", "HEALTH")))
        return 16 * 2 * E * per / 60e3
    choice = None
    for E in range(200, 39, -10):
        Es = {N: (E if N < 10 else max(40, int(0.6 * E))) for N in NS_ALL}
        tot = sum(cost(N, tr, pl, Es[N]) for N in NS_ALL for tr in travels for pl in planners)
        if tot <= budget_cpu_min:
            choice = (Es, tot); break
    if choice is None:
        Es = {2: 40, 3: 40, 5: 40, 10: 30}
        choice = (Es, sum(cost(N, tr, pl, Es[N]) for N in NS_ALL for tr in travels for pl in planners))
    Es, tot = choice
    tasks = []
    for pl in planners:
        for tr in travels:
            for N in (2, 3, 5, 10):
                tasks.append(dict(N=N, travel=tr, planners=[pl], faults=["none", "perm1"], E=Es[N],
                                  H=info["horizon"][f"{N},{tr}"], out=f"nscale_heldout_N{N}_t{tr}_{pl}.csv.gz",
                                  est_cpu_min=round(cost(N, tr, pl, Es[N]), 1)))
    P = dict(seeds=list(range(3000, 3016)), lams=[1, 3], links=["ideal", "d1", "burst30_d1"],
             tasks=tasks, est_total_cpu_min=round(tot, 1), parts={}, budget_cpu_min=budget_cpu_min)
    json.dump(P, open(out_path, "w"), indent=1)
    print(json.dumps(P, indent=1))


def main():
    ap = argparse.ArgumentParser()
    sub = ap.add_subparsers(dest="cmd", required=True)
    r = sub.add_parser("run")
    r.add_argument("--N", type=int, required=True)
    r.add_argument("--seeds", type=int, nargs="+", required=True)
    r.add_argument("--lams", type=float, nargs="+", default=[1, 3])
    r.add_argument("--travels", type=float, nargs="+", default=[1])
    r.add_argument("--planners", nargs="+", default=["greedy"])
    r.add_argument("--faults", nargs="+", default=["none", "perm1"])
    r.add_argument("--links", nargs="+", default=["ideal", "d1", "burst30_d1"])
    r.add_argument("--protos", nargs="+", default=PROTOS)
    r.add_argument("--episodes", type=int, default=100)
    r.add_argument("--horizon", type=int, default=0)
    r.add_argument("--out", required=True)
    z = sub.add_parser("analyze")
    z.add_argument("--inputs", nargs="+", required=True)
    z.add_argument("--out", required=True)
    z.add_argument("--B", type=int, default=2000)
    d = sub.add_parser("dev")
    d.add_argument("--out-dir", default="/tmp/nscale_dev")
    d.add_argument("--scan-only", action="store_true")
    hh = sub.add_parser("heldout")
    hh.add_argument("--part", default=None)
    hh.add_argument("--index", type=int, default=None)
    pp = sub.add_parser("params")
    pp.add_argument("--budget", type=float, required=True)
    pp.add_argument("--travels", default="1,0")
    pp.add_argument("--planners", default="greedy")
    pp.add_argument("--info", default="/tmp/nscale_dev/nscale_dev_info.json")
    a = ap.parse_args()
    if a.cmd == "params":
        make_params(a.budget, [int(x) for x in a.travels.split(",")], a.planners.split(","), a.info,
                    os.path.join(HERE, "results", "nscale_heldout_params.json"))
        return
    if a.cmd == "dev":
        dev_phase(a.out_dir, scan_only=a.scan_only)
        return
    if a.cmd == "heldout":
        heldout_phase(a.part, a.index)
        return
    if a.cmd == "analyze":
        analyze(a.inputs, a.out, a.B)
        return
    if a.cmd == "run":
        if any(3000 <= s <= 3015 for s in a.seeds):
            import hashlib, os
            here = os.path.dirname(os.path.abspath(__file__))
            pr = os.path.join(here, "PREREG_SCALING.md")
            if not os.path.exists(pr):
                sys.exit("held-out seeds require a frozen PREREG_SCALING.md")
        trav = [int(t) if float(t).is_integer() else t for t in a.travels]
        run_grid(a.N, a.seeds, a.lams, trav, a.planners, a.faults, a.links, a.protos,
                 a.episodes, a.out, a.horizon or None)


if __name__ == "__main__":
    main()
