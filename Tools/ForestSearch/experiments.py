"""Pilot 1 sweeps: value of messages under private events and lossy links.

Usage (from Tools/):
    python3 -m ForestSearch.experiments sweep  --out ../PerformanceResults/forest_search
    python3 -m ForestSearch.experiments game   --out ...
    python3 -m ForestSearch.experiments exact  --out ...

All policy comparisons are paired: the same seeds give the same target,
visibility, look uniforms and channel realisation to every strategy.
Score = P(confirmed detection within horizon); ttd is censored at horizon.
"""
from __future__ import annotations

import argparse
import csv
import itertools
import json
import time
from dataclasses import asdict
from pathlib import Path

import numpy as np

from .instances import grid_instance
from .links import GE, LinkConfig
from .sim import STRATEGIES, make_sectors, make_strategies, nominal_plan, run_episode

LINKS = {
    "ideal": LinkConfig(name="ideal"),
    "iid30_d1": LinkConfig(GE.bernoulli(0.3), delay=1, name="iid30_d1"),
    "burst30_L3_d1": LinkConfig(GE.bursty(0.3, 3), delay=1, name="burst30_L3_d1"),
    "burst30_L10_d1": LinkConfig(GE.bursty(0.3, 10), delay=1, name="burst30_L10_d1"),
    "burst60_L10_d1": LinkConfig(GE.bursty(0.6, 10), delay=1, name="burst60_L10_d1"),
    "base_burst30_L10_d1": LinkConfig(GE.bursty(0.3, 10), delay=1, hops=2,
                                      name="base_burst30_L10_d1"),
}

EVENTS = {
    "none": dict(abort=0.0, h_good=0.0, h_bad=0.0, alpha=0.0),
    "abort10": dict(abort=0.10, h_good=0.0, h_bad=0.0, alpha=0.0),
    "hints": dict(abort=0.0, h_good=0.3, h_bad=0.15, alpha=0.03),
    "both": dict(abort=0.10, h_good=0.3, h_bad=0.15, alpha=0.03),
    "harsh": dict(abort=0.25, h_good=0.4, h_bad=0.2, alpha=0.06),
}


def _boot_ci(x: np.ndarray, B: int = 2000, seed: int = 0):
    rng = np.random.default_rng(seed)
    idx = rng.integers(0, len(x), (B, len(x)))
    m = x[idx].mean(1)
    return float(np.percentile(m, 2.5)), float(np.percentile(m, 97.5))


def evaluate(inst, pairs, link, seeds, strategies=None):
    S = strategies or STRATEGIES
    nominal = nominal_plan(inst, S["ORACLE"])
    sectors = make_sectors(inst)
    out = {}
    for pair in pairs:
        rs = [run_episode(inst, pair, link, s, nominal=nominal, sectors=sectors,
                          strategies=S) for s in seeds]
        out[pair] = dict(
            det=np.array([r.detected for r in rs], float),
            ttd=np.array([r.ttd for r in rs], float),
            red=np.array([r.redundant for r in rs], float),
            bytes=np.array([r.bytes_sent for r in rs], float),
        )
    return out


def cmd_sweep(args):
    out = Path(args.out)
    out.mkdir(parents=True, exist_ok=True)
    seeds = range(args.episodes)
    names = ["ORACLE", "INDEP", "SECTOR", "IMPLICIT", "RESERVE", "FULL", "EVENT"]
    pairs = [(n, n) for n in names]
    rows = []
    t0 = time.time()
    for gs in args.grids:
        w, h = map(int, gs.split("x"))
        for iseed in range(args.instances):
            for ev_name, ev in EVENTS.items():
                inst = grid_instance(w, h, seed=100 + iseed, **ev)
                for ln, link in LINKS.items():
                    if ev_name == "none" and ln != "ideal" and ln != "burst60_L10_d1":
                        continue
                    res = evaluate(inst, pairs, link, seeds)
                    ref = res[("SECTOR", "SECTOR")]["det"]
                    orc = res[("ORACLE", "ORACLE")]["det"]
                    for pair in pairs:
                        r = res[pair]
                        d_sec = r["det"] - ref
                        d_orc = r["det"] - orc
                        rows.append(dict(
                            grid=gs, inst=iseed, events=ev_name, link=ln, strategy=pair[0],
                            P=r["det"].mean(), ttd=r["ttd"].mean(), redundant=r["red"].mean(),
                            bytes=r["bytes"].mean(),
                            dP_vs_sector=d_sec.mean(), dP_vs_sector_lo=_boot_ci(d_sec)[0],
                            dP_vs_sector_hi=_boot_ci(d_sec)[1],
                            dP_vs_cgreedy=d_orc.mean(), dP_vs_cgreedy_lo=_boot_ci(d_orc)[0],
                            dP_vs_cgreedy_hi=_boot_ci(d_orc)[1]))
                    print(f"[{time.time()-t0:6.0f}s] {gs} i{iseed} {ev_name:7s} {ln:20s} " +
                          " ".join(f"{p[0]}={res[p]['det'].mean():.3f}" for p in pairs), flush=True)
    path = out / "sweep.csv"
    with path.open("w", newline="") as f:
        wr = csv.DictWriter(f, fieldnames=list(rows[0].keys()))
        wr.writeheader()
        wr.writerows(rows)
    _summarise(rows, out / "sweep_summary.md")
    (out / "sweep_meta.json").write_text(json.dumps(dict(
        episodes=args.episodes, instances=args.instances, grids=args.grids,
        links={k: asdict(v) for k, v in LINKS.items()}, events=EVENTS), indent=1, default=str))
    print("wrote", path)


def _summarise(rows, path):
    import collections
    agg = collections.defaultdict(list)
    for r in rows:
        agg[(r["grid"], r["events"], r["link"], r["strategy"])].append(r)
    keys = sorted({(g, e, l) for (g, e, l, _) in agg})
    strats = ["ORACLE", "INDEP", "SECTOR", "IMPLICIT", "RESERVE", "FULL", "EVENT"]
    lines = ["# Pilot 1 sweep summary", "",
             "Mean P(detect by horizon) over instances; bytes = mean per episode.", "",
             "| grid | events | link | " + " | ".join(strats) + " | bytes FULL/EVENT/RESERVE |",
             "|" + "---|" * (4 + len(strats))]
    for k in keys:
        cells = [f"{np.mean([r['P'] for r in agg[k + (s,)]]):.3f}" for s in strats]
        b = [f"{np.mean([r['bytes'] for r in agg[k + (s,)]]):.0f}" for s in ("FULL", "EVENT", "RESERVE")]
        lines.append(f"| {k[0]} | {k[1]} | {k[2]} | " + " | ".join(cells) + f" | {'/'.join(b)} |")
    path.write_text("\n".join(lines) + "\n")


def cmd_game(args):
    """Common-payoff 'protocol choice' game: each aircraft independently picks
    a protocol; payoff = P(detect) - c * kB sent.  Lists pure Nash equilibria
    (person-by-person optima) and the team optimum."""
    out = Path(args.out)
    out.mkdir(parents=True, exist_ok=True)
    names = ["INDEP", "IMPLICIT", "RESERVE", "FULL", "EVENT"]
    inst = grid_instance(4, 4, seed=7, **EVENTS["both"])
    link = LINKS["burst30_L10_d1"]
    seeds = range(args.episodes)
    pairs = list(itertools.product(names, names))
    res = evaluate(inst, pairs, link, seeds)
    lines = ["# Protocol-choice game (common payoff)", "",
             f"instance={inst.name}, link={link.name}, episodes={args.episodes}", ""]
    for c in args.costs:
        U = np.array([[res[(a, b)]["det"].mean() - c * res[(a, b)]["bytes"].mean() / 1000
                       for b in names] for a in names])
        ne = [(names[i], names[j]) for i in range(len(names)) for j in range(len(names))
              if U[i, j] >= U[:, j].max() - 1e-12 and U[i, j] >= U[i, :].max() - 1e-12]
        opt = np.unravel_index(np.argmax(U), U.shape)
        worst = min(ne, key=lambda p: U[names.index(p[0]), names.index(p[1])])
        lines += [f"## cost c = {c} per kB", "",
                  "| row \\ col | " + " | ".join(names) + " |", "|" + "---|" * (len(names) + 1)]
        for i, a in enumerate(names):
            lines.append(f"| {a} | " + " | ".join(f"{U[i, j]:.3f}" for j in range(len(names))) + " |")
        lines += ["", f"team optimum: {names[opt[0]]},{names[opt[1]]} U={U[opt]:.3f}",
                  f"pure NE: {ne}",
                  f"worst NE / optimum: {U[names.index(worst[0]), names.index(worst[1])]:.3f} / {U[opt]:.3f}",
                  "(Monte Carlo noise ~ +/-0.01; ties within noise are not meaningful.)", ""]
    (out / "game.md").write_text("\n".join(lines))
    print("\n".join(lines))


def cmd_exact(args):
    from .exact import ExactTeam
    from .model import BeliefModel, Instance
    out = Path(args.out)
    out.mkdir(parents=True, exist_ok=True)
    rng = np.random.default_rng(11)
    lines = ["# Exact DP vs centralised greedy (travel-free, 2 agents, n=3, V=2)", "",
             "| inst | k | optimal | greedy | gap | greedy w/ independent-view belief |",
             "|---|---|---|---|---|---|"]
    for t in range(args.instances):
        inst = Instance(prior=rng.dirichlet(np.ones(3)), g=rng.uniform(.2, .9, (3, 2)),
                        q=np.full((3, 2), .8), h_good=.3, h_bad=.2, alpha=.05)
        E = ExactTeam(inst)
        for k in args.k:
            o, g = E.value(k), E.greedy_value(k)
            gi = E.greedy_value(k, BeliefModel(correlated=False))
            lines.append(f"| {t} | {k} | {o:.4f} | {g:.4f} | {o - g:.4f} | {gi:.4f} |")
            print(lines[-1], flush=True)
    (out / "exact.md").write_text("\n".join(lines) + "\n")


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("cmd", choices=["sweep", "game", "exact"])
    ap.add_argument("--out", default="../PerformanceResults/forest_search")
    ap.add_argument("--episodes", type=int, default=400)
    ap.add_argument("--instances", type=int, default=3)
    ap.add_argument("--grids", nargs="+", default=["4x3", "5x5"])
    ap.add_argument("--costs", nargs="+", type=float, default=[0.0, 0.05, 0.2])
    ap.add_argument("--k", nargs="+", type=int, default=[2, 3])
    args = ap.parse_args()
    dict(sweep=cmd_sweep, game=cmd_game, exact=cmd_exact)[args.cmd](args)


if __name__ == "__main__":
    main()
