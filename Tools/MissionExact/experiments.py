"""Searches and sweeps for the minimal examples.  Usage:
    python -m MissionExact.experiments q1|q2|q3
Writes results/<q>.json (deterministic: fixed seeds, fixed enumeration order).
"""
from __future__ import annotations

import json
import sys
import time
from pathlib import Path

import numpy as np

from .model import Instance
from .oracle import Oracle
from .nocomm import NoComm
from .team import Team
from .coverage import coverage_optimal
from .analysis import first_weak_keys, heard_weak_keys, classify, oracle_after_first_weak

RES = Path(__file__).resolve().parent / "results"


def inst_dict(inst):
    return dict(tt=inst.tt.tolist(), link=inst.link.tolist(), prior=inst.prior.tolist(),
                g=inst.g.tolist(), pc=list(inst.pc), pw=list(inst.pw), wb=list(inst.wb),
                alpha=inst.alpha, dur=list(inst.dur), starts=list(inst.starts), T=inst.T,
                d=inst.d, name=inst.name)


def gen(rng, n):
    if n == 2:
        tt = [[0, 1], [1, 0]]
        link = [True, False]
    else:
        a, b, c = (int(x) for x in rng.integers(1, 3, size=3))   # 0-1, 0-2, 1-2
        if a + c < b or b + c < a or a + b < c:
            return None
        tt = [[0, a, b], [a, 0, c], [b, c, 0]]
        link = [True] + [bool(x) for x in rng.integers(0, 2, size=2)]
    prior = [0.0] + list(rng.choice([1, 2, 3, 4], size=n - 1).astype(float))
    g = [[0.5, 0.5]] + [list(rng.choice([0.2, 0.5, 0.8], size=2)) for _ in range(n - 1)]
    return dict(tt=tt, link=link, prior=prior, g=g, alpha=0.1)


def q1():
    rng = np.random.default_rng(1)
    rows = []
    for n in (2, 3):
        for k in range(16 if n == 3 else 8):
            base = gen(rng, n)
            if base is None:
                continue
            for T in (5, 6):
                for d in (0, 1):
                    inst = Instance(**base, T=T, d=d, name=f"q1-n{n}-{k}")
                    vo = Oracle(inst).value()
                    cov = coverage_optimal(inst)
                    vn = NoComm(inst).solve()
                    eo = Oracle(inst, "e").value()
                    ec = coverage_optimal(inst, "e")[0]
                    rows.append(dict(inst=inst_dict(inst), oracle=vo, cover=cov[0],
                                     cover_makespan=cov[1], cover_routes=cov[2], nocomm=vn,
                                     oracle_E=T + 1 - eo, cover_E=T + 1 - ec))
                    print(n, k, T, d, f"or={vo:.4f} cov={cov[0]:.4f} nc={vn:.4f} "
                          f"E_or={T+1-eo:.3f} E_cov={T+1-ec:.3f}", flush=True)
    (RES / "q1.json").write_text(json.dumps(rows, indent=1))


def q1pick():
    rows = json.loads((RES / "q1.json").read_text())
    for r in rows:
        r["gap"] = r["oracle"] - r["cover"]
        r["n"] = len(r["inst"]["prior"])
    good = [r for r in rows if r["gap"] >= 0.03]
    good.sort(key=lambda r: (r["n"], -r["gap"]))
    out = []
    for r in good[:1] + [g for g in good if g["n"] == 3][:1]:
        kw = dict(r["inst"])
        kw["pc"], kw["pw"], kw["wb"], kw["dur"] = map(tuple, (kw["pc"], kw["pw"], kw["wb"], kw["dur"]))
        kw["starts"] = tuple(kw["starts"])
        inst = Instance(**kw)
        tf = Team(inst, "full"); vf = tf.solve()
        tn = Team(inst, "none"); vn = tn.solve()
        cov = coverage_optimal(inst)
        o = Oracle(inst)
        o.value()
        pk = dict(inst=r["inst"], oracle=vf, nocomm=vn, cover=cov[0], cover_routes=cov[2],
                  cover_makespan=cov[1], oracle_E=r["oracle_E"], cover_E=r["cover_E"],
                  rd_oracle=tf.rdist, rd_nocomm_team=tn.rdist, rd_cover=cov[3],
                  oracle_tree=o.policy_lines(max_depth=8, min_prob=0.01),
                  nocomm_pol=[tn.policy_lines(0), tn.policy_lines(1)],
                  after_weak=[(t, p, s) for t, p, s, _ in oracle_after_first_weak(o)])
        out.append(pk)
        print(r["n"], r["gap"], vf, vn, cov[0], flush=True)
    (RES / "q1_pick.json").write_text(json.dumps(out[0], indent=1))
    (RES / "q1_pick_all.json").write_text(json.dumps(out, indent=1))


# ----------------------------------------------------------------- Q2
def q2_instance(d, T=7, d_base=None):
    """Hand-built 3-node dead-zone instance (see MINIMAL_EXAMPLES.md)."""
    return Instance(tt=[[0, 1, 1], [1, 0, 1], [1, 1, 0]], link=[True, False, True],
                    prior=[0, 0.5, 0.5], g=[[.5, .5], [.8, .5], [.5, .5]],
                    alpha=0.1, starts=(1, 2), T=T, d=d, name=f"Q2 d={d}")


def team_summary(tm, inst):
    out = {}
    for j in (0, 1):
        out[f"D{j}_after_first_weak"] = [(k[1], k[2], classify(inst, k, a), repr(a))
                                         for k, a in first_weak_keys(tm, j)]
        out[f"D{j}_after_hearing_weak"] = [(k[1], k[2], repr(a)) for k, a in heard_weak_keys(tm, j)]
    return out


def q2(spec="base"):
    rows = []
    for d in range(0, 5):
        inst = q2_builder(spec)(d)
        t0 = time.time()
        vo = Oracle(inst).value()
        tm = Team(inst, "delayed")
        vd = tm.solve()
        vn = NoComm(inst).solve()
        forced = {}
        for label, allow in (("self", lambda a: a[0] == "L"), ("leave", lambda a: a[0] != "L")):
            tf = Team(inst, "delayed")
            tf.restrict = (lambda k, allow=allow: _first_weak_restrict(inst, k, allow))
            forced[label] = tf.solve()
        row = dict(d=d, inst=inst_dict(inst), oracle=vo, delayed=vd, nocomm=vn, forced=forced,
                   secs=time.time() - t0, policy0=tm.policy_lines(0), policy1=tm.policy_lines(1),
                   **team_summary(tm, inst))
        rows.append(row)
        print(d, f"or={vo:.4f} del={vd:.4f} nc={vn:.4f} forced={forced} {time.time()-t0:.1f}s", flush=True)
        print("   ", row["D0_after_first_weak"][:3], row["D1_after_first_weak"][:3],
              row["D0_after_hearing_weak"][:2], row["D1_after_hearing_weak"][:2], flush=True)
    (RES / f"q2_{spec}.json").write_text(json.dumps(rows, indent=1))


def _first_weak_restrict(inst, k, allow):
    """Constrain only the info sets 'my last look was my first WEAK, and I
    have heard of no partner WEAK'."""
    j, t, node, hist, rcv = k
    if not hist or hist[-1][3] != 1 or sum(1 for e in hist if e[3] == 1) != 1:
        return None
    if rcv is not None and any(e[3] == 1 for e in rcv[1]):
        return None
    return [a for a in inst.actions(node, comm=True) if allow(a)]


Q2_SPECS = {}


def q2_builder(spec):
    return Q2_SPECS[spec]


Q2_SPECS["base"] = lambda d: q2_instance(d)


# ----------------------------------------------------------------- Q3
def q3():
    rows = []
    for pc_hi in (0.8, 0.2):
        for T in (5, 7):
            inst = Instance(tt=[[0, 1, 2], [1, 0, 1], [2, 1, 0]], link=[True, False, False],
                            prior=[0, pc_hi, 1 - pc_hi], g=[[.5, .5], [.5, .5], [.5, .5]],
                            alpha=0.2, starts=(0, 0), T=T, d=0, name=f"Q3 prior(n1)={pc_hi} T={T}")
            o = Oracle(inst)
            v = o.value()
            aw = oracle_after_first_weak(o)
            rows.append(dict(inst=inst_dict(inst), oracle=v,
                             after_weak=[(t, p, s) for t, p, s, _ in aw],
                             tree=o.policy_lines(max_depth=6, min_prob=0.02)))
            print(pc_hi, T, f"{v:.4f}", [(t, round(p, 3), s) for t, p, s, _ in aw], flush=True)
    (RES / "q3.json").write_text(json.dumps(rows, indent=1))


if __name__ == "__main__":
    RES.mkdir(exist_ok=True)
    cmd = sys.argv[1]
    if cmd == "q1":
        q1()
        q1pick()
    elif cmd == "q1pick":
        q1pick()
    elif cmd == "q2":
        q2(sys.argv[2] if len(sys.argv) > 2 else "base")
    elif cmd == "q3":
        q3()
