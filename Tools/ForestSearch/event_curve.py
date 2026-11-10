"""Value of communication vs private-event rate (exploratory or confirmatory).

python3 -m ForestSearch.event_curve --travel 0 --seeds 100 101 102 --episodes 300 --out X.csv

Private-event scale lam multiplies the base rates (abort 0.10, h_good 0.30,
h_bad 0.15, alpha 0.03), capped.  Reports paired differences vs IMPLICIT.
"""
from __future__ import annotations

import argparse
import csv
import time

import numpy as np

from .instances import grid_instance
from .links import GE, LinkConfig
from .sim import make_sectors, nominal_plan
from .variants import make_all_strategies, run_any

BASE = dict(abort=0.10, h_good=0.30, h_bad=0.15, alpha=0.03)
LINKS = {
    "d0_loss0": LinkConfig(name="d0_loss0"),
    "d1_loss0": LinkConfig(GE.bernoulli(0.0), delay=1, name="d1_loss0"),
    "d0_burst30": LinkConfig(GE.bursty(0.3, 10), delay=0, name="d0_burst30"),
    "d1_burst30": LinkConfig(GE.bursty(0.3, 10), delay=1, name="d1_burst30"),
}
NAMES = ["ORACLE", "IMPLICIT", "SECTOR", "FULL", "EVENT", "HYBRID", "HYBRID_P"]


def events(lam: float) -> dict:
    return dict(abort=min(0.6, BASE["abort"] * lam), h_good=min(0.9, BASE["h_good"] * lam),
                h_bad=min(0.6, BASE["h_bad"] * lam), alpha=min(0.2, BASE["alpha"] * lam))


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--travel", type=float, default=0.0)
    ap.add_argument("--grid", default="5x5")
    ap.add_argument("--seeds", type=int, nargs="+", default=[100, 101, 102])
    ap.add_argument("--episodes", type=int, default=300)
    ap.add_argument("--lams", type=float, nargs="+", default=[0, 1, 2, 3])
    ap.add_argument("--links", nargs="+", default=list(LINKS))
    ap.add_argument("--names", nargs="+", default=NAMES)
    ap.add_argument("--horizon", type=int, default=0)
    ap.add_argument("--out", required=True)
    a = ap.parse_args()
    w, h = map(int, a.grid.split("x"))
    S = make_all_strategies()
    rows, t0 = [], time.time()
    for lam in a.lams:
        for seed in a.seeds:
            hz = a.horizon or (16 if a.travel == 0 else 2 * w * h)
            inst = grid_instance(w, h, seed=seed, travel_scale=a.travel, horizon=hz, **events(lam))
            nom = nominal_plan(inst, S["ORACLE"])
            sec = make_sectors(inst)
            for ln in a.links:
                for n in a.names:
                    for ep in range(a.episodes):
                        r = run_any(inst, (n, n), LINKS[ln], ep, S, nom, sec)
                        rows.append((lam, seed, ln, n, ep, int(r.detected), r.ttd, r.bytes_sent))
            print(f"[{time.time()-t0:6.0f}s] lam={lam} seed={seed}", flush=True)
    with open(a.out, "w", newline="") as f:
        wr = csv.writer(f)
        wr.writerow(["lam", "seed", "link", "strategy", "episode", "detected", "ttd", "bytes"])
        wr.writerows(rows)
    print("wrote", a.out)


if __name__ == "__main__":
    main()
