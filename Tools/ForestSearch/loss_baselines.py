"""Minimal-communication baselines for the agent-loss counterexample (R7a-d).

cd Tools && fsrun python3 -m ForestSearch.loss_baselines --out ForestSearch/results/loss_baselines.md

HEALTH    : IMPLICIT + one-time health beacon.  When an agent's sensor dies it
            sends ONE frame (HEADER_B bytes, no records, no lease) carrying the
            death time stamp.  If delivered, the receiver drops the dead agent's
            nominal tasks that START at or after the stamp (neither counted as
            done nor predicted as in flight).  Looks started before the stamp
            are still counted (they ran with a live sensor).
HEARTBEAT : HEALTH plus a 1-bit status frame from each agent every 5 steps
            (alive/dead; a dead status carries the death stamp), so a lost
            beacon is retransmitted.  Costs one frame per agent per 5 steps
            even when nothing fails.

Assumptions (stated, not hidden): the agent self-diagnoses its sensor failure
instantly at the first step with bad[idx, t]; the radio stays up (same as the
falsify.py loss model); a frame is HEADER_B = 12 bytes.
No existing file is edited; this subclasses FEpisode.
"""
from __future__ import annotations

import argparse
import time
from dataclasses import dataclass

import numpy as np

from .event_curve import LINKS
from .falsify import FEpisode, boot, regimes
from .sim import HEADER_B, Strategy, make_sectors, nominal_plan
from .variants import make_all_strategies

HB_PERIOD = 5


@dataclass(frozen=True)
class LStrategy(Strategy):
    health: str = ""          # "" | "beacon" | "heartbeat"


@dataclass
class HealthMsg:
    t_dead: int | None        # None = alive status


class LEpisode(FEpisode):
    def __init__(self, *a, **k):
        super().__init__(*a, **k)
        self.t_dead = [int(np.argmax(self.bad[i])) if self.bad[i].any() else None for i in range(2)]
        self.p_dead = [None, None]          # what agent i has learned about its partner
        self._hsent = set()

    def _health(self):
        return all(getattr(s, "health", "") for s in self.strats)

    def _hsend(self, i, t, msg):
        if (i, t) in self._hsent:
            return
        self._hsent.add((i, t))
        self.bytes += HEADER_B
        self.msgs += 1
        if self.links[i].send(t, msg):
            self.delivered += 1

    def _deliver(self, t):
        if not self._health():
            return super()._deliver(t)
        for i in range(2):
            td = self.t_dead[i]
            dead = td is not None and td <= t
            if dead and (i, "b") not in self._hsent:
                self._hsent.add((i, "b"))
                self._hsend(i, t, HealthMsg(td))
            if self.strats[i].health == "heartbeat" and t > 0 and t % HB_PERIOD == 0:
                self._hsend(i, t, HealthMsg(td if dead else None))
        for a in self.agents:
            for m in self.links[1 - a.idx].receive(t):
                if m.t_dead is not None and self.p_dead[a.idx] is None:
                    self.p_dead[a.idx] = m.t_dead

    def _partner_counts(self, a, t):
        td = self.p_dead[a.idx] if self._health() else None
        if td is None or self.oracle or self.nominal is None:
            return super()._partner_counts(a, t)
        p = 1 - a.idx
        saved = self.nominal
        self.nominal = list(saved)
        self.nominal[p] = [tk for tk in saved[p] if tk.start < td]
        try:
            return super()._partner_counts(a, t)
        finally:
            self.nominal = saved


def make_strategies():
    S = make_all_strategies()
    base = S["IMPLICIT"]
    f = {k: getattr(base, k) for k in base.__dataclass_fields__}
    for nm, h in (("HEALTH", "beacon"), ("HEARTBEAT", "heartbeat")):
        g = dict(f); g["name"] = nm
        S[nm] = LStrategy(**g, health=h)
    return S


def run_l(inst, name, ep, S, nom, sec, fault, link):
    f = dict(fault or {}); f["seed"] = 10_000 + ep
    st = (S[name], S[name])
    return LEpisode(inst, st, LINKS[link], np.random.default_rng(ep), nom, sec, fault=f).run()


ALL = ["ORACLE", "IMPLICIT", "INDEP", "HYBRID_P", "HEALTH", "HEARTBEAT"]


def cell(d):
    m, lo, hi = d
    return f"{m:+.3f} [{lo:+.3f},{hi:+.3f}]"


def run_regime(key, seeds, eps, links):
    desc, build, fault = regimes()[key]
    S = make_strategies()
    det, byt, dlv = {}, {}, {}
    for sd in seeds:
        inst = build(sd)
        nom = nominal_plan(inst, S["ORACLE"]); sec = make_sectors(inst)
        for ln in links:
            for n in ALL:
                if ln != links[0] and n not in ("HYBRID_P", "HEALTH", "HEARTBEAT"):
                    continue          # non-sending strategies are link-independent
                rs = [run_l(inst, n, ep, S, nom, sec, fault, ln) for ep in range(eps)]
                det.setdefault((ln, n), {})[sd] = np.array([r.detected for r in rs], float)
                byt.setdefault((ln, n), []).extend(r.bytes_sent for r in rs)
                dlv.setdefault((ln, n), []).append((sum(r.msgs_delivered for r in rs), sum(r.msgs_sent for r in rs)))
    return desc, det, byt, dlv


def get(det, ln, n, base_ln):
    return det[(ln, n)] if (ln, n) in det else det[(base_ln, n)]


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--seeds", type=int, nargs="+", default=list(range(504, 512)))
    ap.add_argument("--episodes", type=int, default=200)
    ap.add_argument("--regimes", nargs="+", default=["R7a", "R7b", "R7c", "R7d"])
    ap.add_argument("--links", nargs="+", default=["d0_loss0", "d1_burst30"])
    ap.add_argument("--out", default=None)
    a = ap.parse_args()
    L0 = a.links[0]
    out = [f"seeds {a.seeds} episodes {a.episodes} (paired by episode index; B=2000 cluster bootstrap)\n"]
    t0 = time.time()
    for key in a.regimes:
        desc, det, byt, dlv = run_regime(key, a.seeds, a.episodes, a.links)
        out.append(f"\n### {key}: {desc}\n")
        for ln in a.links:
            out.append(f"\nlink {ln}\n")
            out.append("| strategy | P(detect) | d vs IMPLICIT | d vs INDEP | bytes/episode | delivered/sent |")
            out.append("|---|---|---|---|---|---|")
            for n in ALL:
                if ln != L0 and n not in ("HYBRID_P", "HEALTH", "HEARTBEAT"):
                    continue
                D = get(det, ln, n, L0)
                P = float(np.mean(np.concatenate(list(D.values()))))
                vi = "-" if n == "IMPLICIT" else cell(boot({s: D[s] - get(det, ln, "IMPLICIT", L0)[s] for s in a.seeds}))
                vn = "-" if n == "INDEP" else cell(boot({s: D[s] - get(det, ln, "INDEP", L0)[s] for s in a.seeds}))
                key2 = (ln, n) if (ln, n) in byt else (L0, n)
                b = float(np.mean(byt[key2])) if n in ("HYBRID_P", "HEALTH", "HEARTBEAT") else 0.0
                dl = dlv.get((ln, n))
                dd = f"{sum(x for x, _ in dl)}/{sum(y for _, y in dl)}" if dl and sum(y for _, y in dl) else "-"
                out.append(f"| {n} | {P:.3f} | {vi} | {vn} | {b:.1f} | {dd} |")
        print("\n".join(out[-30:]), flush=True)
        print(f"[{time.time()-t0:.0f}s]", flush=True)
    if a.out:
        open(a.out, "a").write("\n".join(out) + "\n")


if __name__ == "__main__":
    main()
