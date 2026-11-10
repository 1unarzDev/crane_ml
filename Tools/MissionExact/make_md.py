"""Render results/*.json into markdown fragments (results/_auto_*.md) that
MINIMAL_EXAMPLES.md quotes.  python -m MissionExact.make_md"""
from __future__ import annotations

import json
from pathlib import Path

from .model import Instance

RES = Path(__file__).resolve().parent / "results"


def inst_of(d):
    kw = dict(d)
    for k in ("pc", "pw", "wb", "dur", "starts"):
        kw[k] = tuple(kw[k])
    return Instance(**kw)


def code(lines):
    return "```\n" + "\n".join(lines) + "\n```"


def q1_md():
    pk = json.loads((RES / "q1_pick.json").read_text())
    inst = inst_of(pk["inst"])
    L = [inst.describe(), "",
         f"| policy | P(report by T={inst.T}) | E[min(R, T+1)] |", "|---|---|---|",
         f"| ORACLE | {pk['oracle']:.4f} | {pk['oracle_E']:.3f} (separately optimised for E) |",
         f"| best decentralised, no comm | {pk['nocomm']:.4f} | |",
         f"| coverage-optimal sweep (makespan {pk['cover_makespan']}, routes {pk['cover_routes']}) | {pk['cover']:.4f} | {pk['cover_E']:.3f} |",
         "", "Report-time distribution (mass delivered at each t):", "",
         "| t | " + " | ".join(str(t) for t in range(inst.T + 1)) + " |",
         "|---|" + "---|" * (inst.T + 1)]
    for lab, key in (("ORACLE", "rd_oracle"), ("no-comm", "rd_nocomm_team"), ("coverage", "rd_cover")):
        rd = pk[key]
        L.append(f"| {lab} | " + " | ".join(f"{rd.get(str(t), 0):.3f}" for t in range(inst.T + 1)) + " |")
    L += ["", "ORACLE policy tree (branches with prob >= 0.01):", "", code(pk["oracle_tree"]),
          "", "Best no-comm policies (one line per reached information set):", "",
          code(["D0:"] + pk["nocomm_pol"][0] + ["D1:"] + pk["nocomm_pol"][1])]
    return "\n".join(L)


def q2_md(spec):
    rows = json.loads((RES / f"q2_{spec}.json").read_text())
    inst = inst_of(rows[0]["inst"])
    L = [inst.describe().replace(f"d={inst.d}", "d swept"), "",
         "| d | ORACLE | best delayed-comm | best no-comm | forced self-verify | forced leave | detector's action after first WEAK (delayed-comm optimum) | partner's action after hearing WEAK |",
         "|---|---|---|---|---|---|---|---|"]
    for r in rows:
        a0 = "; ".join(f"D0 t={t}@n{n}: {c}" for t, n, c, _ in r["D0_after_first_weak"][:2])
        a1 = "; ".join(f"D1 t={t}@n{n}: {c}" for t, n, c, _ in r["D1_after_first_weak"][:2])
        h = "; ".join(f"D{j} t={t}@n{n}: {a}" for j in (0, 1) for t, n, a in r[f"D{j}_after_hearing_weak"][:2])
        L.append(f"| {r['d']} | {r['oracle']:.4f} | {r['delayed']:.4f} | {r['nocomm']:.4f} | "
                 f"{r['forced']['self']:.4f} | {r['forced']['leave']:.4f} | {a0} {a1} | {h or '-'} |")
    for r in rows:
        L += ["", f"Delayed-comm optimal policy at d={r['d']}:", "",
              code(["D0:"] + r["policy0"] + ["D1:"] + r["policy1"])]
    return "\n".join(L)


def q3_md():
    rows = json.loads((RES / "q3.json").read_text())
    L = [inst_of(rows[0]["inst"]).describe(), "",
         "| prior(n1) | T | ORACLE P(report by T) | ORACLE action right after the first WEAK |", "|---|---|---|---|"]
    for r in rows:
        aw = "<br>".join(f"t={t} (p={p:.3f}): {s}" for t, p, s in r["after_weak"])
        L.append(f"| {r['inst']['prior'][1]:.2f} | {r['inst']['T']} | {r['oracle']:.4f} | {aw} |")
    for r in rows:
        L += ["", f"ORACLE tree, prior(n1)={r['inst']['prior'][1]:.2f}, T={r['inst']['T']}:", "", code(r["tree"])]
    return "\n".join(L)


if __name__ == "__main__":
    import sys
    if (RES / "q1_pick.json").exists():
        (RES / "_auto_q1.md").write_text(q1_md())
    for spec in sys.argv[1:] or ["base"]:
        if (RES / f"q2_{spec}.json").exists():
            (RES / f"_auto_q2_{spec}.md").write_text(q2_md(spec))
    if (RES / "q3.json").exists():
        (RES / "_auto_q3.md").write_text(q3_md())
