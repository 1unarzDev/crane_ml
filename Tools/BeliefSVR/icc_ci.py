# SPDX-License-Identifier: GPL-3.0-or-later
"""How many rendered sequences does the Vision rerun need to pin down rho?  Simulates the copula model (copula.py):
n_seq sequences, each with m looks per view class from 2 classes; target visible p.  Estimators: ANOVA ICC within a class
(pooled over classes) and the cross-class correlation of per-class means.  Reports the empirical 95% CI half-width
(2.5/97.5 percentiles of the estimates, 400 replicates) so that  'CI width needed'  can be read off.
Run through fsrun-quick (about 20 s)."""
import json, os, sys
import numpy as np
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import bs_copula as C

def sim(n_seq, m, p, rho_icc, rho_cross, rng):
    r = C.latent_for_icc(p, rho_icc)
    t = C.probit(p)
    z0 = rng.standard_normal(n_seq)
    z1 = rho_cross * z0 + np.sqrt(1 - rho_cross ** 2) * rng.standard_normal(n_seq)
    out = []
    for z in (z0, z1):
        e = rng.standard_normal((n_seq, m))
        out.append((np.sqrt(r) * z[:, None] + np.sqrt(1 - r) * e < t).astype(float))
    return out

def icc_anova(y):
    n, m = y.shape
    mb = y.mean(1); gm = y.mean()
    msb = m * np.sum((mb - gm) ** 2) / max(n - 1, 1)
    msw = np.sum((y - mb[:, None]) ** 2) / max(n * (m - 1), 1)
    return (msb - msw) / (msb + (m - 1) * msw) if msb + (m - 1) * msw > 0 else np.nan

def main():
    rng = np.random.default_rng(3)
    rows = []
    for p in (0.3, 0.6):
        for rho in (0.3, 0.6):
            for n_seq in (16, 32, 64, 128):
                for m in (4, 8):
                    est = []
                    for _ in range(400):
                        a, b = sim(n_seq, m, p, rho, 0.0, rng)
                        est.append(np.nanmean([icc_anova(a), icc_anova(b)]))
                    est = np.array(est)
                    rows.append(dict(p=p, rho=rho, n_seq=n_seq, m=m, mean=float(np.nanmean(est)),
                                     lo=float(np.nanpercentile(est, 2.5)), hi=float(np.nanpercentile(est, 97.5))))
    json.dump(rows, open(os.path.join(os.path.dirname(os.path.abspath(__file__)), "results", "raw", "icc_ci.json"), "w"))
    for r in rows:
        print(r["p"], r["rho"], r["n_seq"], r["m"], round(r["mean"], 2), "half-width", round((r["hi"] - r["lo"]) / 2, 2))

if __name__ == "__main__":
    main()
