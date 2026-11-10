"""Stronger causal evidence for the stale-plan mechanism (reviewer B, Sec. 6).

New file; nothing existing is modified.  Everything here is EXPLORATORY.

Variants (all HYBRID-based, built on a re-implementation of
``variants.HEpisode._partner_counts`` that is generalised in three ways):

  predictors   which agents replace the stale nominal in-flight guess by a
               greedy prediction of the partner's choice (HYBRID_P = (1,)).
  pred_mode    'argmax' (HYBRID_P), 'rand_all' (uniform over all cell-views),
               'rand_top5' (uniform over the 5 best-scoring cell-views).
  forced       fire the prediction after a known deviation even when no
               nominal in-flight slot exists (matters for agent 0, which
               decides first and so never sees a same-step in-flight task).
  drop_q       each received partner record is permanently discarded with
               probability q (decided once per receiver and record), so the
               two agents' beliefs diverge although messages are delivered.
  drop_rx      which receivers drop.

Auxiliary randomness uses its own generator so that truth, outcomes and link
draws (and therefore pairing across strategies) are untouched.

Usage (always through the slot wrapper, one process at a time):
  fsrun python3 -m ForestSearch.mechanism2 --part 1 ...   (run from Tools/)
  fsrun python3 -m ForestSearch.mechanism2 --report
"""
from __future__ import annotations

import argparse
import json
import time
from dataclasses import dataclass
from pathlib import Path

import numpy as np

from .controls import PEpisode, PStrategy, make_phantom_strategies
from .event_curve import events
from .instances import grid_instance
from .links import GE, LinkConfig
from .model import HINT, NULL, detect_prob
from .sim import make_sectors, nominal_plan
from .variants import HEpisode, HStrategy

CACHE = Path("/home/lunarz/.cache/research/fs-scripts/mech2")
RESULTS = Path(__file__).resolve().parent / "results" / "mechanism2.md"
SEEDS = list(range(900, 908))
T975 = {1: 12.706, 2: 4.303, 3: 3.182, 4: 2.776, 5: 2.571, 6: 2.447, 7: 2.365,
        8: 2.306, 9: 2.262, 10: 2.228}


# --------------------------------------------------------------------------
@dataclass(frozen=True)
class MStrategy(HStrategy):
    predictors: tuple = (1,)
    pred_mode: str = "argmax"
    forced: bool = False
    drop_q: float = 0.0
    drop_rx: tuple = (0, 1)
    fire_over: bool = False   # predict even when a (stale) lease already yields a virtual look


class MEpisode(HEpisode):
    def __init__(self, *a, aux_seed: int = 0, **kw):
        super().__init__(*a, **kw)
        self.aux = np.random.default_rng([aux_seed, 4242])
        self.drop_rng = np.random.default_rng([aux_seed, 4243])
        self._drop_dec: dict = {}
        self.fires = [0, 0]
        self.dropped = 0

    # ------------------------------------------------------------ delivery
    def _deliver(self, t: int):
        for a in self.agents:
            q = getattr(a.s, "drop_q", 0.0)
            for msg in self.links[1 - a.idx].receive(t):
                if q > 0 and a.idx in a.s.drop_rx:
                    kept = []
                    for r in msg.records:
                        key = (a.idx, r.seq)
                        if key not in self._drop_dec:
                            self._drop_dec[key] = bool(self.drop_rng.random() < q)
                            self.dropped += self._drop_dec[key]
                        if not self._drop_dec[key]:
                            kept.append(r)
                    msg.records = kept
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
                if a.s.send == "event" and not a.explicit:
                    a.explicit = True
                    a.t_dev = t
                    self._send(a, t, [], self._lease(a))

    # --------------------------------------------------------- partner model
    def _predict(self, a, m0, m1, mode):
        inst, s = self.inst, a.s
        pos = self._partner_pos_guess(a)
        pd = detect_prob(inst, a.m0 + m0, a.m1 + m1, s.belief)
        sc = pd / (inst.dist[pos][:, None] + 1.0)
        if mode == "argmax":
            return divmod(int(np.argmax(sc)), inst.V)
        if mode == "rand_all":
            return divmod(int(self.aux.integers(inst.n * inst.V)), inst.V)
        if mode == "rand_top5":
            top = np.argsort(-sc, axis=None, kind="stable")[:5]
            return divmod(int(top[int(self.aux.integers(len(top)))]), inst.V)
        raise ValueError(mode)

    def _partner_counts(self, a, t):
        s = a.s
        if not isinstance(s, MStrategy) or self.oracle or self.nominal is None:
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
        if not virtual or s.fire_over:
            fire = (a.idx in s.predictors and deviated
                    and (inflight_guess is not None or s.forced or s.fire_over))
            if fire:
                virtual.append(self._predict(a, m0, m1, s.pred_mode))
                self.fires[a.idx] += 1
            elif inflight_guess is not None and not virtual:
                virtual.append(inflight_guess)
        return m0, m1, virtual


def make_m_strategies() -> dict:
    S = make_phantom_strategies()
    h = S["HYBRID"]
    f = {k: getattr(h, k) for k in h.__dataclass_fields__}

    def add(name, **kw):
        g = dict(f); g["name"] = name
        S[name] = MStrategy(**g, **kw)

    add("HP_M", predictors=(1,))                                   # == HYBRID_P
    add("SWAP", predictors=(0,))                                   # agent 0 predicts agent 1
    add("SWAP_F", predictors=(0,), forced=True)
    add("SYM", predictors=(0, 1))
    add("SYM_F", predictors=(0, 1), forced=True)
    add("RND_A", predictors=(1,), pred_mode="rand_all")
    add("RND_T5", predictors=(1,), pred_mode="rand_top5")
    add("SWAP_V", predictors=(0,), fire_over=True)
    add("SYM_V", predictors=(0, 1), fire_over=True)
    add("RND_A_V", predictors=(0,), pred_mode="rand_all", fire_over=True)
    add("HY_M", predictors=())                                     # == HYBRID
    for q in (0.25, 0.5, 0.75):
        add(f"DROP{int(q*100)}", predictors=(), drop_q=q)
    add("DROP50_rx1", predictors=(), drop_q=0.5, drop_rx=(1,))
    add("DROP50_rx0", predictors=(), drop_q=0.5, drop_rx=(0,))
    add("DROP50_P", predictors=(1,), drop_q=0.5)
    return S


# --------------------------------------------------------------------------
def make_link(delay: int) -> LinkConfig:
    return LinkConfig(GE.bernoulli(0.0), delay=delay, name=f"d{delay}_loss0")


def make_inst(seed, lam, travel):
    hz = 16 if travel == 0 else 50
    return grid_instance(5, 5, seed=seed, travel_scale=float(travel), horizon=hz, **events(lam))


def run_one(inst, name, link, seed, S, nom, sec):
    """Returns dict(det, ttd, coll, red, fires, dropped).  coll = number of
    steps at which both agents *started* the same (cell, view)."""
    st = S[name]
    strat = (st, st)
    if isinstance(st, MStrategy):
        ep = MEpisode(inst, strat, link, np.random.default_rng(seed), nom, sec, aux_seed=seed)
    elif isinstance(st, PStrategy):
        ep = PEpisode(inst, strat, link, np.random.default_rng(seed), nom, sec)
    else:
        ep = HEpisode(inst, strat, link, np.random.default_rng(seed), nom, sec)
    starts: dict = {}
    orig = ep._choose

    def ch(a, t):
        c, v = orig(a, t)
        starts.setdefault(t, {})[a.idx] = (c, v)
        return c, v
    ep._choose = ch
    r = ep.run()
    coll = sum(1 for d in starts.values() if len(d) == 2 and d[0] == d[1])
    return dict(det=int(r.detected), ttd=r.ttd, coll=coll, red=r.redundant,
                fires=sum(getattr(ep, "fires", [0, 0])), dropped=getattr(ep, "dropped", 0))


def run_cell(names, lam, delay, travel, episodes, seeds=SEEDS):
    S = make_m_strategies()
    link = make_link(delay)
    out = {n: {k: np.zeros((len(seeds), episodes)) for k in
               ("det", "ttd", "coll", "red", "fires", "dropped")} for n in names}
    for i, sd in enumerate(seeds):
        inst = make_inst(sd, lam, travel)
        nom, sec = nominal_plan(inst, S["ORACLE"]), make_sectors(inst)
        for n in names:
            for e in range(episodes):
                r = run_one(inst, n, link, e, S, nom, sec)
                for k, v in r.items():
                    out[n][k][i, e] = v
    return out


# --------------------------------------------------------------------------
# statistics: clusters = instances; pairing = same episode seed.
def t_ci(x):
    x = np.asarray(x, float)
    n = len(x)
    m = x.mean()
    if n < 2:
        return m, m, m
    h = T975.get(n - 1, 1.96) * x.std(ddof=1) / np.sqrt(n)
    return m, m - h, m + h


def boot_ci(A, B=4000, seed=1):
    """A: (instances, episodes) paired differences.  Two-stage paired cluster
    bootstrap: resample instances, then episodes within each."""
    rng = np.random.default_rng(seed)
    I, E = A.shape
    ii = rng.integers(0, I, (B, I))
    ee = rng.integers(0, E, (B, I, E))
    vals = A[ii[:, :, None], ee].mean(axis=(1, 2))
    return np.percentile(vals, 2.5), np.percentile(vals, 97.5)


def fmt(m, lo, hi, d=3, sign=True):
    f = f"{{:{'+' if sign else ''}.{d}f}}"
    return f"{f.format(m)} [{f.format(lo)},{f.format(hi)}]"


def table(res, names, ref="IMPLICIT", cref="HYBRID", boot=True, with_fires=False):
    hdr = ("| strategy | P(detect) | dP vs %s (t7 CI) | dP boot CI | coll/ep (t7 CI) | "
           "dColl vs %s (t7 CI) | sign |" % (ref, cref))
    lines = [hdr, "|---|---|---|---|---|---|---|"]
    for n in names:
        d = res[n]["det"] - res[ref]["det"]
        di = d.mean(1)
        m, lo, hi = t_ci(di)
        bl = boot_ci(d) if (boot and n != ref) else (0, 0)
        pm, plo, phi = t_ci(res[n]["det"].mean(1))
        cm, clo, chi = t_ci(res[n]["coll"].mean(1))
        dc = (res[n]["coll"] - res[cref]["coll"]).mean(1)
        dm, dlo, dhi = t_ci(dc)
        pos = int((di > 0).sum()); neg = int((di < 0).sum())
        lines.append(f"| {n} | {pm:.3f} | {fmt(m, lo, hi)} | "
                     f"{fmt(m, *bl) if n != ref else '-'} | {cm:.2f} [{clo:.2f},{chi:.2f}] | "
                     f"{fmt(dm, dlo, dhi, 2)} | {pos}+/{neg}- |")
    return "\n".join(lines)


# --------------------------------------------------------------------------
PARTS = {
    1: dict(names=["IMPLICIT", "HYBRID", "HYBRID_P", "PH_orc", "NP_none", "NP_nom"],
            confs=[(1.0, 1, 0), (3.0, 1, 0)]),
    2: dict(names=["IMPLICIT", "HYBRID", "HYBRID_P", "HP_M", "SWAP", "SWAP_F", "SYM", "SYM_F",
                   "RND_A", "RND_T5"],
            confs=[(1.0, 1, 0), (3.0, 1, 0)]),
    3: dict(names=["IMPLICIT", "HYBRID", "HYBRID_P", "DROP25", "DROP50", "DROP75",
                   "DROP50_rx1", "DROP50_rx0", "DROP50_P"],
            confs=[(1.0, 1, 0), (3.0, 1, 0)]),
    5: dict(names=["IMPLICIT", "HYBRID", "HYBRID_P", "SWAP_V", "SYM_V", "RND_A_V", "HYBRID_S"],
            confs=[(1.0, 1, 0), (3.0, 1, 0)]),
    4: dict(names=["IMPLICIT", "HYBRID", "HYBRID_P", "PH_orc"],
            confs=[(1.0, 2, 0), (1.0, 3, 0), (1.0, 1, 1), (1.0, 2, 1)]),
}


def run_part(k, episodes):
    CACHE.mkdir(parents=True, exist_ok=True)
    t0 = time.time()
    for (lam, delay, travel) in PARTS[k]["confs"]:
        res = run_cell(PARTS[k]["names"], lam, delay, travel, episodes)
        fn = CACHE / f"part{k}_lam{lam:g}_d{delay}_tr{travel}.npz"
        np.savez(fn, **{f"{n}|{key}": v for n, d in res.items() for key, v in d.items()})
        print(f"[{time.time()-t0:6.0f}s] part {k} lam={lam:g} delay={delay} travel={travel}", flush=True)


def load(fn):
    z = np.load(fn)
    res = {}
    for key in z.files:
        n, k = key.split("|")
        res.setdefault(n, {})[k] = z[key]
    return res


def report():
    out = ["# Mechanism evidence v2 (EXPLORATORY; fresh seeds 900-907, 5x5, link d*_loss0)", "",
           "Cluster = instance (8). Intervals: t_7 = 2.365 on instance means (primary); "
           "'boot' = two-stage paired percentile bootstrap (B=4000), shown for comparison and "
           "known to under-cover with 8 clusters. Pairing = same episode seed across strategies. "
           "Collisions = steps at which both agents *started* the same (cell, view). "
           "'sign' = instances with dP>0 / dP<0 vs IMPLICIT.", ""]
    titles = {1: "Part 1 - key controls", 2: "Part 2 - placebo controls for HYBRID_P",
              3: "Part 3 - shared-belief necessity (record drops)",
              5: "Part 5 - role-asymmetry placebo with stale-lease override",
              4: "Part 4 - dose-response in delay and travel"}
    for k in (1, 2, 3, 5, 4):
        for (lam, delay, travel) in PARTS[k]["confs"]:
            fn = CACHE / f"part{k}_lam{lam:g}_d{delay}_tr{travel}.npz"
            if not fn.exists():
                continue
            res = load(fn)
            out += [f"## {titles[k]}: lambda={lam:g}, delay={delay}, travel={travel}", "",
                    table(res, PARTS[k]["names"]), ""]
            if k == 2:
                out += ["prediction firings/episode: " + ", ".join(
                    f"{n}={res[n]['fires'].mean():.2f}" for n in PARTS[k]["names"][2:]), ""]
            if k == 3:
                out += ["records dropped/episode: " + ", ".join(
                    f"{n}={res[n]['dropped'].mean():.2f}" for n in PARTS[k]["names"][3:]), ""]
    RESULTS.write_text("\n".join(out) + "\n")
    print("\n".join(out))


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--part", type=int)
    ap.add_argument("--episodes", type=int, default=300)
    ap.add_argument("--report", action="store_true")
    a = ap.parse_args()
    if a.report:
        report()
    else:
        run_part(a.part, a.episodes)


if __name__ == "__main__":
    main()
