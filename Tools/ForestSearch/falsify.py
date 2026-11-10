"""Falsification sweep: look for regimes where ORACLE / HYBRID_P beat IMPLICIT.

cd Tools && fsrun python3 -m ForestSearch.falsify --seeds 500 501 502 503 --episodes 200 --regimes R0 ...
New mechanics (subclass of HEpisode, no existing file edited):
  persist : per-agent on/off Markov 'degraded sensor' state; all looks started
            while degraded abort (private to the looker).
  loss    : agent stops permanently (all later looks abort) at a random time.
ORACLE is told about degraded/dead partner (does not count its in-flight look
as virtual); that is exactly the information communication could carry.
"""
from __future__ import annotations

import argparse
import dataclasses
import time

import numpy as np

from .event_curve import BASE, LINKS, events
from .instances import grid_instance
from .model import Instance
from .sim import make_sectors, nominal_plan
from .variants import HEpisode, make_all_strategies

NAMES = ["ORACLE", "IMPLICIT", "HYBRID_P", "INDEP"]


class FEpisode(HEpisode):
    def __init__(self, *a, fault=None, **k):
        super().__init__(*a, **k)
        self.fault = fault or {}
        H = self.inst.horizon + 1
        r = np.random.default_rng(self.fault.get("seed", 0))
        bad = np.zeros((2, H), bool)
        kind = self.fault.get("kind")
        if kind == "persist":
            for i in range(2):
                on = False
                for t in range(H):
                    on = (r.random() < self.fault["p_on"]) if not on else (r.random() >= self.fault["p_off"])
                    bad[i, t] = on
        elif kind == "loss":
            who = int(r.integers(2)) if self.fault.get("who") is None else self.fault["who"]
            tl = int(r.integers(self.fault["t_lo"], self.fault["t_hi"] + 1))
            bad[who, tl:] = True
        self.bad = bad
        self.forced = set()      # (agent, seq) forced aborts

    def _choose(self, a, t):
        if self.bad[a.idx, t]:
            self.u[a.idx, min(a.seq, self.inst.horizon), 0] = -1.0
            self.forced.add((a.idx, a.seq))
        return super()._choose(a, t)

    def _partner_counts(self, a, t):
        m0, m1, virtual = super()._partner_counts(a, t)
        if self.oracle:
            p = self.agents[1 - a.idx]
            if p.task is not None and (p.idx, p.task.seq) in self.forced:
                virtual = []
        return m0, m1, virtual


def run_f(inst, name, ep, S, nom, sec, fault):
    st = (S[name], S[name])
    f = dict(fault or {}); f["seed"] = 10_000 + ep
    link = LINKS["d0_loss0"]
    return FEpisode(inst, st, link, np.random.default_rng(ep), nom, sec, fault=f).run()


def mk(w, h, seed, ev=None, prior_fn=None, **kw):
    e = dict(events(1.0)); e.update(ev or {})
    inst = grid_instance(w, h, seed=seed, **e, **kw)
    if prior_fn is not None:
        inst = dataclasses.replace(inst, prior=prior_fn(inst.prior))
    return inst


def peak(k):      # sharpen prior
    return lambda p: p ** k
def sparse(m):    # mass on m cells only
    def f(p):
        q = np.full_like(p, 1e-4); q[np.argsort(-p)[:m]] = p[np.argsort(-p)[:m]]; return q
    return f

HI = dict(abort=0.1, h_good=0.9, h_bad=0.6, alpha=0.2)
# name -> (description, builder(seed)->Instance, fault)
def regimes():
    R = {}
    R["R0"] = ("baseline 5x5 t0 H16", lambda s: mk(5, 5, s, travel_scale=0, horizon=16), None)
    for H in (4, 8):
        R[f"R1a{H}"] = (f"short horizon t0 H{H}", lambda s, H=H: mk(5, 5, s, travel_scale=0, horizon=H), None)
    for H in (8, 14):
        R[f"R1b{H}"] = (f"short horizon travel1 H{H}", lambda s, H=H: mk(5, 5, s, travel_scale=1, horizon=H), None)
    R["R2a"] = ("peaked prior (^3) t0 H12", lambda s: mk(5, 5, s, travel_scale=0, horizon=12, prior_fn=peak(3)), None)
    R["R2b"] = ("sparse prior (3 cells) t1 H20", lambda s: mk(5, 5, s, travel_scale=1, horizon=20, prior_fn=sparse(3)), None)
    R["R3a"] = ("high hint+false hint t0 H16", lambda s: mk(5, 5, s, ev=HI, travel_scale=0, horizon=16), None)
    R["R3b"] = ("high hint+false hint t1 H30", lambda s: mk(5, 5, s, ev=HI, travel_scale=1, horizon=30), None)
    R["R4a"] = ("8x8 travel1 H40", lambda s: mk(8, 8, s, travel_scale=1, horizon=40), None)
    R["R4b"] = ("10x10 travel1 H50", lambda s: mk(10, 10, s, travel_scale=1, horizon=50), None)
    R["R5"] = ("asym starts 8x8 t1 H40 (both at far corner)", lambda s: mk(8, 8, s, travel_scale=1, horizon=40, starts=(63, 63)), None)
    R["R5b"] = ("asym starts 8x8 t2 H60 (0 at 0, 1 at 63)", lambda s: mk(8, 8, s, travel_scale=2, horizon=60), None)
    R["R6a"] = ("persistent degraded sensor t0 H16", lambda s: mk(5, 5, s, travel_scale=0, horizon=16),
                dict(kind="persist", p_on=0.08, p_off=0.08))
    R["R6b"] = ("persistent degraded sensor t1 H30", lambda s: mk(5, 5, s, travel_scale=1, horizon=30),
                dict(kind="persist", p_on=0.08, p_off=0.08))
    R["R7a"] = ("agent loss t0 H16", lambda s: mk(5, 5, s, travel_scale=0, horizon=16),
                dict(kind="loss", t_lo=1, t_hi=8))
    R["R7b"] = ("agent loss t1 H30", lambda s: mk(5, 5, s, travel_scale=1, horizon=30),
                dict(kind="loss", t_lo=1, t_hi=15))
    R["R7c"] = ("agent 0 loss early (t in 1..4) t0 H16", lambda s: mk(5, 5, s, travel_scale=0, horizon=16),
                dict(kind="loss", t_lo=1, t_hi=4, who=0))
    R["R7d"] = ("agent loss 8x8 t1 H40", lambda s: mk(8, 8, s, travel_scale=1, horizon=40),
                dict(kind="loss", t_lo=1, t_hi=20))
    R["R8a"] = ("low q=0.3 t0 H24", lambda s: mk(5, 5, s, travel_scale=0, horizon=24, q=0.3), None)
    R["R8b"] = ("low q=0.15 t1 H40", lambda s: mk(5, 5, s, travel_scale=1, horizon=40, q=0.15), None)
    return R


def boot(diff, B=2000, rng=None):
    """diff: dict seed -> array of paired diffs. Cluster bootstrap (instances, then episodes)."""
    rng = rng or np.random.default_rng(1)
    seeds = sorted(diff)
    m = float(np.mean(np.concatenate([diff[s] for s in seeds])))
    bs = []
    for _ in range(B):
        ss = rng.choice(seeds, len(seeds))
        bs.append(np.mean(np.concatenate([rng.choice(diff[s], len(diff[s])) for s in ss])))
    return m, float(np.percentile(bs, 2.5)), float(np.percentile(bs, 97.5))


def run_regime(key, seeds, eps, lines):
    desc, build, fault = regimes()[key]
    S = make_all_strategies()
    det = {n: {} for n in NAMES}
    for sd in seeds:
        inst = build(sd)
        nom = nominal_plan(inst, S["ORACLE"]); sec = make_sectors(inst)
        for n in NAMES:
            det[n][sd] = np.array([run_f(inst, n, ep, S, nom, sec, fault).detected for ep in range(eps)], float)
    imp = np.mean(np.concatenate(list(det["IMPLICIT"].values())))
    cells = []
    flag = ""
    for n in ("ORACLE", "HYBRID_P", "INDEP"):
        m, lo, hi = boot({s: det[n][s] - det["IMPLICIT"][s] for s in seeds})
        cells.append(f"{m:+.3f} [{lo:+.3f},{hi:+.3f}]")
        if n != "INDEP" and m >= 0.03 and lo > 0:
            flag = " **HIT**"
    line = f"| {key} | {desc} | {imp:.3f} | " + " | ".join(cells) + f" |{flag}"
    print(line, flush=True)
    lines.append(line)


if __name__ == "__main__":
    ap = argparse.ArgumentParser()
    ap.add_argument("--seeds", type=int, nargs="+", default=[500, 501, 502, 503])
    ap.add_argument("--episodes", type=int, default=200)
    ap.add_argument("--regimes", nargs="+", default=list(regimes()))
    ap.add_argument("--out", default=None)
    a = ap.parse_args()
    lines = ["| id | regime | IMPLICIT P | ORACLE-IMPL | HYBRID_P-IMPL | INDEP-IMPL |", "|---|---|---|---|---|---|"]
    print("\n".join(lines))
    t0 = time.time()
    for k in a.regimes:
        run_regime(k, a.seeds, a.episodes, lines)
    print(f"[{time.time()-t0:.0f}s]")
    if a.out:
        open(a.out, "a").write(f"\nseeds {a.seeds} episodes {a.episodes}\n\n" + "\n".join(lines) + "\n")
