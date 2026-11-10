# SPDX-License-Identifier: GPL-3.0-or-later
"""DEV driver.  Layout seeds are DEV only (MissionSim dev range 0-1999); the layout guard of missionsim.world refuses
anything else, and this file additionally refuses 91000-91019 and 92000-92031 (held-out for RoleSpec / BeliefSVR).

  python bs_sweep.py tune                      difficulty tuning on layouts 0-23 (writes results/raw/tune_*.jsonl)
  python bs_sweep.py run [--shard i/n]         the rho x f x regime sweep on layouts 100-163
Run through ~/.cache/research/fs-scripts/fsrun only.
"""
import argparse, json, os, sys, time, hashlib
from dataclasses import asdict, replace

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
sys.path.insert(0, os.path.join(HERE, "..", "Frontier"))
import bs_sim as S, bs_policies as P  # noqa: E402

FORBIDDEN = list(range(91000, 91020)) + list(range(92000, 92032))
TUNE_LAYOUTS = list(range(0, 24))
SWEEP_LAYOUTS = list(range(100, 220))
RHOS = (0.0, 0.3, 0.6, 0.8)
FLEV = ("low", "measured", "high")
REPS = 2
# regimes; the values below are the DEV-tuned ones (see results/DEV.md section 1)
REGIMES = {
    "coverage": S.Regime(name="coverage", K=100, D=900.0),
    "verification": S.Regime(name="verification", K=30, D=1500.0),
}
THETAS = (0.2, 0.35, 0.5, 0.7, 0.9)
THETAS_IND_EXT = (0.95, 0.99, 0.999)   # the independence belief is over-confident: its frontier needs higher thresholds
SPRT_A = (1.0, 2.0, 3.0, 4.5, 6.0)


def policy_specs(rho, include_partner=True, pomcp=False):
    sp = [dict(kind="COVER", name="COVER"), dict(kind="ORACLE", name="ORACLE")]
    sp += [dict(kind="FIXK", k=k, name=f"FIXK{k}") for k in (1, 2, 3, 4)]
    for a in SPRT_A:
        sp.append(dict(kind="SPRT", A=a, name=f"SPRT@{a}"))
        sp.append(dict(kind="SPRT_DEFL", A=a, rho_a=rho, alt=False, name=f"SPRT_DEFL@{a}"))
        sp.append(dict(kind="SPRT_DEFL", A=a, rho_a=rho, alt=True, name=f"SPRT_DEFL_ALT@{a}"))
        sp.append(dict(kind="CI", A=a, name=f"CI_THR@{a}"))
    for th in THETAS:
        sp.append(dict(kind="BAYES", rb=0.0, theta=th, rule="mi", name=f"IND_BAYES@{th}"))
        sp.append(dict(kind="BAYES", rb=rho, theta=th, rule="mi", name=f"CORR@{th}"))
        sp.append(dict(kind="BAYES", rb=rho, theta=th, rule="same", name=f"CORR_SAMEVIEW@{th}"))
        if include_partner:
            sp.append(dict(kind="BAYES", rb=rho, theta=th, rule="mi", partner=True, pw=0.5, name=f"CORR_PARTNER@{th}"))
    for th in THETAS_IND_EXT:
        sp.append(dict(kind="BAYES", rb=0.0, theta=th, rule="mi", name=f"IND_BAYES@{th}"))
    if pomcp:
        for th in (0.35, 0.5):
            sp.append(dict(kind="POMCP", rb=rho, theta=th, n_part=12, name=f"POMCP@{th}"))
    return sp


def run_layout(config, layout, _fid):
    """config: dict(rho, f, regime(dict), specs, reps). Returns list of mission rows."""
    reg = S.Regime(**config["regime"])
    cond = S.Cond(rho=config["rho"], f=config["f"], regime=reg)
    ctx = S.Ctx(layout, cond)
    rows = []
    for spec in config["specs"]:
        fac = P.make_factory(spec)
        for rep in range(config["reps"]):
            t0 = time.process_time()
            r = S.run_mission(ctx, rep, fac)
            r.update(policy=spec["name"], layout=layout, rep=rep, cpu=time.process_time() - t0,
                     rho=cond.rho, f=cond.f, regime=reg.name)
            rows.append(r)
    return rows


def guard(layouts):
    bad = [l for l in layouts if l in set(FORBIDDEN) or not (0 <= l <= 1999)]
    if bad:
        raise SystemExit(f"refusing non-DEV layouts {bad[:5]}")


TUNE_CANDIDATES = [S.Regime(name=f"cand_K{k}_D{int(d)}", K=k, D=d) for k, d in
                   ((100, 600.0), (100, 900.0), (100, 1500.0), (100, 2400.0), (30, 600.0), (30, 900.0), (30, 1500.0),
                    (15, 600.0), (15, 900.0))]
# second tuning pass (verification time share): short search area, slow reviews and costly view changes
TUNE_CANDIDATES2 = [S.Regime(name=f"cand2_K{k}_D{int(d)}_rev{int(r)}_sw{int(w)}", K=k, D=d, rev_s=r, sw_s=w, d0=40.0)
                    for k, d, r, w in ((15, 900.0, 20.0, 25.0), (15, 1500.0, 20.0, 25.0), (15, 1500.0, 30.0, 25.0),
                                       (15, 1500.0, 30.0, 40.0), (8, 1500.0, 30.0, 25.0), (30, 1500.0, 30.0, 25.0))]


def tune(a):
    cands = TUNE_CANDIDATES2 if a.regime == "tune2" else TUNE_CANDIDATES
    """Difficulty tuning on DEV layouts 0-23 at the measured condition (rho 0.6, f measured), policy subset only."""
    guard(TUNE_LAYOUTS)
    names = ("COVER", "ORACLE", "FIXK2", "SPRT@3.0", "SPRT_DEFL@2.0", "IND_BAYES@0.5", "CORR@0.5", "CORR_PARTNER@0.5")
    specs = [s for s in policy_specs(0.6) if s["name"] in names]
    out = a.out or os.path.join(HERE, "results", "raw", "tune2.jsonl" if a.regime == "tune2" else "tune.jsonl")
    with open(out, "a") as fh:
        for reg in cands:
            for f in ("measured",):
                cfg = dict(rho=0.6, f=f, regime=asdict(reg), specs=specs, reps=REPS)
                t0 = time.process_time()
                for L in TUNE_LAYOUTS:
                    for r in run_layout(cfg, L, "tune"):
                        fh.write(json.dumps(r) + "\n")
                fh.flush()
                print(reg.name, f, "cpu", round(time.process_time() - t0, 1), flush=True)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("mode", choices=["tune", "run", "delay", "pomcp", "indext"])
    ap.add_argument("--shard", default="0/1")
    ap.add_argument("--nocache", action="store_true")
    ap.add_argument("--regime", default="")
    ap.add_argument("--out", default="")
    ap.add_argument("--budget-s", type=float, default=0.0, help="stop cleanly after this many wall seconds (cache makes reruns resume)")
    a = ap.parse_args()
    if a.mode == "tune":
        return tune(a)
    try:
        import cache as FC
    except Exception:
        FC = None
    try:
        import costs as CO
    except Exception:
        CO = None
    layouts = TUNE_LAYOUTS if a.mode == "tune" else (SWEEP_LAYOUTS if a.mode in ("run", "indext") else SWEEP_LAYOUTS[:60])
    guard(layouts)
    i, n = map(int, a.shard.split("/"))
    out = a.out or os.path.join(HERE, "results", "raw", f"{a.mode}_{i}of{n}.jsonl")
    os.makedirs(os.path.dirname(out), exist_ok=True)
    files = [os.path.join(HERE, f) for f in ("bs_sim.py", "bs_policies.py", "bs_copula.py", "bs_sweep.py")]
    cells = [(r, f, g) for g in (REGIMES if not a.regime else [a.regime]) for r in RHOS for f in FLEV]
    if a.mode == "pomcp":
        cells = [c for c in cells if c[1] == "measured"]
    if a.mode == "delay":   # partner delay sweep at the measured condition, verification-bound regime
        for d0 in (10.0, 25.0, 60.0, 120.0):
            REGIMES[f"verification_d0={int(d0)}"] = replace(REGIMES["verification"], name=f"verification_d0={int(d0)}", d0=d0)
        cells = [(0.6, "measured", f"verification_d0={int(d0)}") for d0 in (10.0, 25.0, 60.0, 120.0)]
    t0 = time.time(); c0 = time.process_time()
    done = 0
    with open(out, "a") as fh:
        for k, (rho, f, g) in enumerate(cells):
            if k % n != i:
                continue
            specs = policy_specs(rho, pomcp=(a.mode == "pomcp"))
            if a.mode == "pomcp":
                specs = [s_ for s_ in specs if s_["name"].startswith("POMCP") or s_["name"].startswith(("CORR@", "IND_BAYES@"))]
            if a.mode == "delay":
                specs = [s_ for s_ in specs if s_["name"].startswith("CORR")]
            if a.mode == "indext":
                specs = [s_ for s_ in specs if s_["name"] in [f"IND_BAYES@{t}" for t in THETAS_IND_EXT]]
            if a.mode == "run":
                specs = [s_ for s_ in specs if s_["name"] not in [f"IND_BAYES@{t}" for t in THETAS_IND_EXT]]
            if a.mode == "tune":
                specs = [s for s in specs if s["name"] in ("COVER", "ORACLE", "FIXK2", "SPRT_DEFL@4.5", "IND_BAYES@0.9", "CORR@0.9",
                                                           "CORR_PARTNER@0.9", "SPRT@4.5")]
            cfg = dict(rho=rho, f=f, regime=asdict(REGIMES[g]), specs=specs, reps=REPS)
            tr = CO.track("belief_svr", unit="mission", n=1, fidelity="L1", note=f"{a.mode} rho={rho} f={f} {g}") if CO else None
            if tr:
                tr.__enter__()
            ncell = 0
            for L in layouts:
                if FC is not None and not a.nocache:
                    rows = FC.get_or_run("belief_svr_dev", run_layout, cfg, L, a.mode, files, split="dev")
                else:
                    rows = run_layout(cfg, L, a.mode)
                for r in rows:
                    fh.write(json.dumps(r) + "\n")
                fh.flush()
                done += len(rows)
                ncell += len(rows)
                if a.budget_s and time.time() - t0 > a.budget_s:
                    print(f"budget reached at cell {k} layout {L}; rerun to resume", flush=True)
                    if tr:
                        tr.n = ncell
                        tr.__exit__(None, None, None)
                    return
            if tr:
                tr.n = ncell
                tr.__exit__(None, None, None)
            fh.flush()
            print(f"cell {k}: rho={rho} f={f} {g}  rows={done}  wall={time.time()-t0:.0f}s cpu={time.process_time()-c0:.0f}s", flush=True)
    print(f"DONE rows={done} cpu={time.process_time()-c0:.1f}s wall={time.time()-t0:.1f}s", flush=True)


if __name__ == "__main__":
    main()
