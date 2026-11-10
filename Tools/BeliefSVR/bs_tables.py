# SPDX-License-Identifier: GPL-3.0-or-later
"""Markdown fragments for results/DEV.md from results/raw/tables.json, tune*.jsonl, delay.jsonl, pomcp.jsonl.
Usage: python bs_tables.py <fragment> [args];  fragments: tuning, tuning2, H, policies REGIME F RHO, causes, calib, delay, pomcp, cost."""
import collections, json, os, sys
import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
RAW = os.path.join(HERE, "results", "raw")
T = json.load(open(os.path.join(RAW, "tables.json")))


def md(h, rows):
    return "\n".join(["| " + " | ".join(h) + " |", "|" + "---|" * len(h)] + ["| " + " | ".join(map(str, r)) + " |" for r in rows])


def g(h, k, pct=False, d=2):
    v = h["point"].get(k)
    c = h["ci"].get(k)
    if v is None or isinstance(v, dict) or v != v:
        return "n/a"
    if pct:
        s = f"{v*100:.0f}%"
        if c and c[0] == c[0]:
            s += f" [{c[0]*100:.0f}, {c[1]*100:.0f}]"
        return s
    s = f"{v:.{d}f}"
    if c and c[0] == c[0]:
        s += f" [{c[0]:.{d}f}, {c[1]:.{d}f}]"
    return s


def rows_of(path):
    return [json.loads(l) for l in open(path)] if os.path.exists(path) else []


def tuning(path, pols, label):
    rows = rows_of(os.path.join(RAW, path))
    by = collections.defaultdict(list)
    for r in rows:
        by[(r["regime"], r["policy"])].append(r)
    regs = list(dict.fromkeys(r["regime"] for r in rows))
    out = []
    for g_ in regs:
        line = [g_]
        for p in pols:
            rr = by[(g_, p)]
            line.append(f"{np.mean([r['ok'] for r in rr]):.2f} / {np.mean([r['FR'] for r in rr]):.1f}")
        cv = collections.Counter(r["cause"] for r in by[(g_, "COVER")] if not r["ok"])
        line.append(", ".join(f"{k} {v}" for k, v in sorted(cv.items())))
        sh = [r["t_rev"] / max(r["t_end"], 1) for r in by[(g_, "CORR@0.5")] if "t_rev" in r]
        line.append(f"{np.mean(sh):.2f}" if sh else "n/a")
        out.append(line)
    return md([label] + [p + " (succ / FR)" for p in pols] + ["COVER miss causes", "verif. time share (CORR@0.5)"], out)


def H(regs, keys, titles, lam=300):
    rows = []
    for h in T["H"]:
        if h["regime"] not in regs:
            continue
        rows.append([h["regime"], h["f"], h["rho"]] + [g(h, k, pct=p) for k, p in keys])
    rows.sort(key=lambda r: (r[0], ["low", "measured", "high"].index(r[1]), r[2]))
    return md(["regime", "f", "rho"] + titles, rows)


def policies(regime, f, rho, names):
    rows = []
    P = {r["policy"]: r for r in T["policy"] if r["regime"] == regime and r["f"] == f and r["rho"] == rho}
    for n in names:
        r = P.get(n)
        if not r:
            continue
        vt = r["verifs_per_true"]
        rows.append([n, f"{r['ok']:.2f} [{r['ok_ci'][0]:.2f}, {r['ok_ci'][1]:.2f}]", f"{r['T']:.0f} [{r['T_ci'][0]:.0f}, {r['T_ci'][1]:.0f}]",
                     f"{r['FR']:.2f} [{r['FR_ci'][0]:.2f}, {r['FR_ci'][1]:.2f}]", f"{r['T'] + 300 * r['FR']:.0f}",
                     "n/a" if vt is None else f"{vt:.1f}", ", ".join(f"{k} {v}" for k, v in sorted(r["causes"].items()))])
    return md(["policy", "success", "T_cens s (misses = D)", "false reports / mission", "J = T + 300 FR", "verifications per true report", "miss causes (n missions)"], rows)


if __name__ == "__main__":
    a = sys.argv[1:]
    if a[0] == "H":
        print(H(a[1].split(","), [("H1", False), ("H1_theta", False), ("gain_vs_fixk@300", True), ("gain_vs_ind@300", True),
                                  ("gain_vs_cheap@300", True), ("cheap_over_best@300", True), ("viewclass@300", True), ("partner@300", True)],
                ["H1 FR inflation, matched T", "FR(IND)/FR(CORR) at matched theta", "J gain vs FIXK", "J gain vs IND_BAYES", "J gain vs deflated SPRT/CI",
                 "cheap-fix J over best (kill if < 5%)", "view-class choice (CORR vs same-view)", "partner (CORR_PARTNER vs CORR)"]))
    elif a[0] == "policies":
        print(policies(a[1], a[2], float(a[3]), a[4].split(",")))


def _tun():
    pols = ["ORACLE", "COVER", "FIXK2", "SPRT@3.0", "SPRT_DEFL@2.0", "IND_BAYES@0.5", "CORR@0.5", "CORR_PARTNER@0.5"]
    return tuning("tune.jsonl", pols, "candidate regime (K cells, D s)") + "\n\n" + tuning("tune2.jsonl", pols, "second pass (K, D, review s, view-change s)")
