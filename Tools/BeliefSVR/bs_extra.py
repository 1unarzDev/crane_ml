# SPDX-License-Identifier: GPL-3.0-or-later
"""Partner-delay sweep and POMCP-lite comparison (reads results/raw/delay.jsonl, pomcp.jsonl, run_chunk2.jsonl)."""
import collections, json, os
import numpy as np
HERE = os.path.dirname(os.path.abspath(__file__)); RAW = os.path.join(HERE, "results", "raw")
LAM = 300

def load(p): return [json.loads(l) for l in open(os.path.join(RAW, p))]

def boot_best(rows, fams, rng, B=300):
    """best-grid-point J per family with layout-cluster bootstrap; returns dict fam -> (J, lo, hi, name)"""
    by = collections.defaultdict(lambda: collections.defaultdict(list))
    for r in rows:
        by[r["policy"]][r["layout"]].append(r["T"] + LAM * r["FR"])
    layouts = sorted({l for d in by.values() for l in d})
    A = {p: np.array([np.mean(d[l]) for l in layouts]) for p, d in by.items() if len(d) == len(layouts)}
    out = {}
    for fam, pre in fams.items():
        names = [p for p in A if p.startswith(pre)]
        if not names: continue
        J = min((A[n].mean(), n) for n in names)
        bs = []
        for _ in range(B):
            i = rng.integers(0, len(layouts), len(layouts))
            bs.append(min(A[n][i].mean() for n in names))
        out[fam] = (J[0], float(np.percentile(bs, 2.5)), float(np.percentile(bs, 97.5)), J[1])
    return out, A, layouts

def main():
    rng = np.random.default_rng(1)
    lines = []
    d = load("delay.jsonl")
    lines.append("DELAY (verification regime, rho 0.6, f measured): best-grid J = T_cens + 300 FR; gain of partner over solo CORR")
    for reg in sorted({r["regime"] for r in d}, key=lambda s: int(s.split("=")[1])):
        rr = [r for r in d if r["regime"] == reg]
        o, A, L = boot_best(rr, {"CORR": "CORR@", "CORR_SAMEVIEW": "CORR_SAMEVIEW@", "CORR_PARTNER": "CORR_PARTNER@"}, rng)
        # paired gain with bootstrap
        bn, pn = o["CORR"][3], o["CORR_PARTNER"][3]
        diff = (A[bn] - A[pn]) / A[bn]
        bs = [float(np.mean(diff[rng.integers(0, len(L), len(L))])) for _ in range(500)]
        lines.append(f"  {reg}: J CORR {o['CORR'][0]:.0f}, J PARTNER {o['CORR_PARTNER'][0]:.0f}  paired gain {np.mean(diff)*100:.1f}% [{np.percentile(bs,2.5)*100:.1f}, {np.percentile(bs,97.5)*100:.1f}]  (n layouts {len(L)}, same grid point pairs {bn} vs {pn})")
    p = load("pomcp.jsonl"); base = load("run_chunk2.jsonl")
    lines.append("POMCP-lite vs analytic (measured f, both regimes; first 60 layouts)")
    for reg in ("coverage", "verification"):
        for rho in (0.0, 0.3, 0.6, 0.8):
            rr = [r for r in p if r["regime"] == reg and r["rho"] == rho and r["f"] == "measured"]
            if not rr: continue
            L = {r["layout"] for r in rr}
            b = [r for r in base if r["regime"] == reg and r["rho"] == rho and r["f"] == "measured" and r["layout"] in L and r["policy"].startswith(("CORR@", "IND_BAYES@", "FIXK", "SPRT", "CI_THR", "COVER", "CORR_PARTNER@"))]
            o, A, Ls = boot_best(rr + b, {"POMCP": "POMCP@", "CORR": "CORR@", "CORR_PARTNER": "CORR_PARTNER@", "IND": "IND_BAYES@", "FIXK": "FIXK", "cheap": "SPRT_DEFL", "COVER": "COVER"}, rng)
            lines.append(f"  {reg} rho={rho}: " + ", ".join(f"{k} {v[0]:.0f}[{v[1]:.0f},{v[2]:.0f}]" for k, v in o.items()))
    print("\n".join(lines))
    open(os.path.join(RAW, "extra.txt"), "w").write("\n".join(lines) + "\n")

if __name__ == "__main__":
    main()
