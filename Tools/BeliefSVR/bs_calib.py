# SPDX-License-Identifier: GPL-3.0-or-later
"""Reliability of the stated report posterior: nominal vs realised precision, rho 0.6, f measured, DEV layouts 100-139."""
import collections, json, os, sys
HERE = os.path.dirname(os.path.abspath(__file__)); sys.path.insert(0, HERE)
import bs_sim as S, bs_policies as P, bs_sweep as W
out = ["| regime | belief | n reports | mean stated posterior | realised precision | by bin [stated -> realised (n)] |", "|---|---|---|---|---|---|"]
for rname in ("coverage", "verification"):
    reg = W.REGIMES[rname]
    for name, spec in (("IND_BAYES (rb=0)", dict(kind="BAYES", rb=0.0, theta=0.5, name="i")), ("CORR (rb=rho)", dict(kind="BAYES", rb=0.6, theta=0.5, name="c")),
                       ("IND_BAYES theta 0.9", dict(kind="BAYES", rb=0.0, theta=0.9, name="i9")), ("CORR theta 0.9", dict(kind="BAYES", rb=0.6, theta=0.9, name="c9"))):
        fac = P.make_factory(spec)
        bins = collections.defaultdict(lambda: [0, 0.0, 0])
        for L in range(100, 140):
            ctx = S.Ctx(L, S.Cond(rho=0.6, f="measured", regime=reg))
            for rep in range(2):
                r = S.run_mission(ctx, rep, fac)
                for p, ok in r["rlog"]:
                    b = min(int(p * 5), 4); bins[b][0] += 1; bins[b][1] += p; bins[b][2] += ok
        n = sum(v[0] for v in bins.values())
        if not n: continue
        mp = sum(v[1] for v in bins.values()) / n; pr = sum(v[2] for v in bins.values()) / n
        out.append(f"| {rname} | {name} | {n} | {mp:.2f} | {pr:.2f} | " + "; ".join(f"{v[1]/v[0]:.2f} -> {v[2]/v[0]:.2f} ({v[0]})" for b, v in sorted(bins.items())) + " |")
text = "\n".join(out)
open(os.path.join(HERE, "results", "raw", "calib.md"), "w").write(text + "\n")
print(text)
