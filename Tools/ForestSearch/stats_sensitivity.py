"""Interval-method sensitivity for the preregistered H1-H5 verdicts (reviewer B).

For every non-trivial H1-H5 point (lambda=0 ORACLE-IMPLICIT is exactly 0 by Prop. 1 and is excluded):
  (a) preregistered cluster bootstrap (analyze_curve.paired, B=2000, same RNG stream as make_hypotheses.py)
  (b) instance-level paired t interval, df = instances - 1
  (c) BCa on instance means (B=20000, seed 12345)
  (d) H1: standard TOST (90% t CI within +-0.03, plus both one-sided p-values)
  (e) H1 superiority (ORACLE > IMPLICIT): one-sided t p-values, Holm over the 12 non-trivial points.
Pure numpy (no scipy); the t cdf/quantile are implemented here.

Usage (from Tools/):  python3 -m ForestSearch.stats_sensitivity [--data DIR] [--out FILE]
"""
from __future__ import annotations

import argparse
import math
import os

import numpy as np

from .analyze_curve import load, paired
from .make_hypotheses import DEFAULT_DATA, HERE, LAMS, ROWS, passes, resolve

MARGIN = 0.03


# ---- t distribution -------------------------------------------------------------------------
def _betacf(a, b, x):
    tiny = 1e-300
    qab, qap, qam = a + b, a + 1, a - 1
    c, d = 1.0, 1 - qab * x / qap
    d = 1 / (d if abs(d) > tiny else tiny)
    h = d
    for m in range(1, 400):
        m2 = 2 * m
        aa = m * (b - m) * x / ((qam + m2) * (a + m2))
        d = 1 + aa * d; d = 1 / (d if abs(d) > tiny else tiny)
        c = 1 + aa / c; c = c if abs(c) > tiny else tiny
        h *= d * c
        aa = -(a + m) * (qab + m) * x / ((a + m2) * (qap + m2))
        d = 1 + aa * d; d = 1 / (d if abs(d) > tiny else tiny)
        c = 1 + aa / c; c = c if abs(c) > tiny else tiny
        de = d * c
        h *= de
        if abs(de - 1) < 1e-15:
            break
    return h


def _betainc(a, b, x):
    if x <= 0: return 0.0
    if x >= 1: return 1.0
    bt = math.exp(math.lgamma(a + b) - math.lgamma(a) - math.lgamma(b) + a * math.log(x) + b * math.log(1 - x))
    return bt * _betacf(a, b, x) / a if x < (a + 1) / (a + b + 2) else 1 - bt * _betacf(b, a, 1 - x) / b


def tcdf(t, df):
    x = df / (df + t * t)
    p = 0.5 * _betainc(df / 2, 0.5, x)
    return 1 - p if t > 0 else p


def tppf(q, df):
    lo, hi = -50.0, 50.0
    for _ in range(200):
        mid = (lo + hi) / 2
        if tcdf(mid, df) < q: lo = mid
        else: hi = mid
    return (lo + hi) / 2


def Phi(z): return 0.5 * (1 + math.erf(z / math.sqrt(2)))
def Phinv(p):
    lo, hi = -10.0, 10.0
    for _ in range(100):
        mid = (lo + hi) / 2
        if Phi(mid) < p: lo = mid
        else: hi = mid
    return (lo + hi) / 2


# ---- intervals ------------------------------------------------------------------------------
def tint(x, level):
    n = len(x); m = x.mean(); se = x.std(ddof=1) / math.sqrt(n)
    q = tppf(0.5 + level / 2, n - 1)
    return m - q * se, m + q * se


def bca(x, alpha=0.05, B=20000, seed=12345):
    n = len(x); th = x.mean()
    if np.ptp(x) == 0:
        return th, th
    bs = x[np.random.default_rng(seed).integers(0, n, (B, n))].mean(1)
    z0 = Phinv(min(max((bs < th).mean(), 1e-9), 1 - 1e-9))
    jk = np.array([np.delete(x, i).mean() for i in range(n)]); d = jk.mean() - jk
    acc = (d ** 3).sum() / (6 * ((d ** 2).sum()) ** 1.5 + 1e-300)
    qs = [Phi(z0 + (z0 + Phinv(a)) / (1 - acc * (z0 + Phinv(a)))) for a in (alpha / 2, 1 - alpha / 2)]
    return tuple(np.percentile(bs, [100 * q for q in qs]))


def inst_means(D, lam, link, s, ref):
    a, b = D[(lam, link, s)], D[(lam, link, ref)]
    keys = sorted(set(a) & set(b)); seeds = sorted({k[0] for k in keys})
    return np.array([np.mean([a[k] - b[k] for k in keys if k[0] == sd]) for sd in seeds])


def holm(p):
    p = np.asarray(p); o = np.argsort(p); m = len(p); adj = np.empty(m); run = 0.0
    for r, i in enumerate(o):
        run = max(run, (m - r) * p[i]); adj[i] = min(1.0, run)
    return adj


def f3(x): return f"{x:+.3f}"
def ci(lo, hi): return f"[{f3(lo)},{f3(hi)}]"


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--data", default=DEFAULT_DATA)
    ap.add_argument("--out", default=os.path.join(HERE, "results", "stats_sensitivity.md"))
    a = ap.parse_args()

    pts = []   # dicts
    for g in ("5x5", "6x6"):
        for t in (0, 1):
            D, _ = load(resolve(a.data, f"confirm_{g}_t{t}.csv"))
            rng = np.random.default_rng(0)      # identical stream to make_hypotheses.py
            for label, link, s, ref, lams, h, t0only in ROWS:
                if t0only and t != 0: continue
                for lam in LAMS:
                    if lam not in lams: continue
                    m, lo, hi = paired(D, lam, link, s, ref=ref, B=2000, rng=rng)   # always drawn, to keep the stream aligned
                    if h == "descr" or (h == "H1" and lam == 0): continue
                    x = inst_means(D, lam, link, s, ref); n = len(x)
                    se = x.std(ddof=1) / math.sqrt(n)
                    pt = dict(g=g, t=t, h=h, lam=lam, link=link, s=s, ref=ref, n=n, m=x.mean(), x=x,
                              boot=(lo, hi), t95=tint(x, .95), bca=bca(x), t90=tint(x, .90), se=se)
                    pt["p_sup"] = 1 - tcdf(pt["m"] / se, n - 1)                       # H_a: mean > 0
                    pt["p_two"] = 2 * min(tcdf(pt["m"] / se, n - 1), 1 - tcdf(pt["m"] / se, n - 1))
                    pt["p_lo"] = 1 - tcdf((pt["m"] + MARGIN) / se, n - 1)             # H0: mean <= -margin
                    pt["p_hi"] = tcdf((pt["m"] - MARGIN) / se, n - 1)                 # H0: mean >= +margin
                    pt["p_tost"] = max(pt["p_lo"], pt["p_hi"])
                    pts.append(pt)

    h1 = [p for p in pts if p["h"] == "H1"]
    adj_sup = holm([p["p_sup"] for p in h1]); adj_two = holm([p["p_two"] for p in h1])
    for p, s_, t_ in zip(h1, adj_sup, adj_two): p["holm_sup"], p["holm_two"] = s_, t_

    L = ["# Interval-method sensitivity for H1-H5 (reviewer B)", "",
         "Generated by `python3 -m ForestSearch.stats_sensitivity` from the confirmatory CSVs (seeds 200-207, 8 instances, "
         "300 episodes each). **This is a robustness analysis, not a re-registration:** the verdicts in `hypotheses.md` "
         "and the paper remain the preregistered ones.", "",
         "Methods. (a) **Cluster bootstrap**: preregistered, `analyze_curve.paired`, B=2000, percentile; instances resampled, "
         "then episodes within instance. (b) **t**: paired per-instance mean differences (8 clusters), 95% t interval, df=7. "
         "(c) **BCa** 95% on the 8 instance means (B=20000). (d) **TOST** (H1 only): standard alpha=0.05 equivalence test "
         "against +-0.03, equal to the 90% t interval inside +-0.03; p = max of the two one-sided t p-values. "
         "(e) **Holm** over the 12 non-trivial H1 points (lambda in {1,2,3} x 4 cells) for the superiority direction "
         "(ORACLE > IMPLICIT, one-sided t); the four lambda=0 points are exactly 0 by Prop. 1 (identical trajectories), so H1 is 12 "
         "tests, not 16.", "",
         "**Why the bootstrap is conservative.** The preregistered two-stage bootstrap resamples instances and then episodes "
         "inside each resampled instance. The variance of the mean of per-instance means already contains the within-instance "
         "(episode) noise once; resampling episodes again adds it a second time. Its intervals are therefore wider than the "
         "instance-level ones (width ratio below), the opposite of the undercoverage the Limitations section warns about. The t interval "
         "has its own weakness: with 8 clusters it relies on approximate normality of instance means and the nominal 95% level "
         "is not guaranteed. BCa on 8 points is also unreliable in the tails. No method here is the 'true' one; "
         "the table shows which verdicts are method-dependent.", "",
         "## 1. Every non-trivial point", "",
         "| cell | test | lambda | link | mean | (a) bootstrap 95% | (b) t 95% | (c) BCa 95% | width ratio a/b |",
         "|---|---|---|---|---|---|---|---|---|"]
    ratios = []
    for p in pts:
        wr = (p["boot"][1] - p["boot"][0]) / (p["t95"][1] - p["t95"][0]); ratios.append(wr)
        L.append(f"| {p['g']} t{p['t']} | {p['h']} {p['s']}-{p['ref']} | {p['lam']:g} | {p['link']} | {f3(p['m'])} | "
                 f"{ci(*p['boot'])} | {ci(*p['t95'])} | {ci(*p['bca'])} | {wr:.2f} |")
    ratios = np.array(ratios)
    L += ["", f"{len(pts)} non-trivial points (the reviewer counted 49; the difference is bookkeeping of which "
              f"H4/H5 lambda=0 rows are counted). Width ratio bootstrap/t: median {np.median(ratios):.2f}, "
              f"range {ratios.min():.2f}-{ratios.max():.2f}; bootstrap wider at {int((ratios > 1).sum())}/{len(ratios)} points.", ""]

    # H1 detail
    L += ["## 2. H1 equivalence and superiority", "",
          "| cell | lambda | mean | (a) bootstrap 95% | within +-0.03 | (b) t 90% | within +-0.03 (TOST) | p_lower | p_upper | TOST p | p sup (1-sided t) | Holm sup | p two-sided | Holm two-sided |",
          "|---|---|---|---|---|---|---|---|---|---|---|---|---|---|"]
    for p in h1:
        wb = p["boot"][0] >= -MARGIN and p["boot"][1] <= MARGIN
        w90 = p["t90"][0] >= -MARGIN and p["t90"][1] <= MARGIN
        L.append(f"| {p['g']} t{p['t']} | {p['lam']:g} | {f3(p['m'])} | {ci(*p['boot'])} | {'yes' if wb else '**no**'} | "
                 f"{ci(*p['t90'])} | {'yes' if w90 else '**no**'} | {p['p_lo']:.4f} | {p['p_hi']:.4f} | {p['p_tost']:.4f} | "
                 f"{p['p_sup']:.4f} | {p['holm_sup']:.3f} | {p['p_two']:.4f} | {p['holm_two']:.3f} |")
    nsig = sum(p["p_sup"] < .05 for p in h1); nholm = sum(p["holm_sup"] < .05 for p in h1)
    L += ["", f"Uncorrected one-sided superiority p<0.05: {nsig}/12. After Holm over 12: {nholm}/12 "
              f"(smallest adjusted p = {min(p['holm_sup'] for p in h1):.3f}). The conjunction over lambda within a cell is an "
              "intersection-union test, so the TOST verdict per cell is 'all three lambda have p<0.05' with no multiplicity correction.", ""]

    # verdict by method
    def verdict(h, which, group):
        ok = []
        for p in group:
            if which == "boot": lo, hi = p["boot"]
            elif which == "t": lo, hi = p["t95"]
            elif which == "bca": lo, hi = p["bca"]
            elif which == "tost": lo, hi = p["t90"]
            ok.append(passes(h, p["m"], lo, hi))
        return "PASS" if all(ok) else "FAIL"

    cells = [("5x5", 0), ("5x5", 1), ("6x6", 0), ("6x6", 1)]
    L += ["## 3. Verdict by method", "",
          "Verdict per preregistered unit (hypothesis x grid x travel; conjunction over lambda / links). "
          "PASS rules are those in PREREG.md applied to the interval named in the column. "
          "TOST column applies only to H1 (90% t interval inside +-0.03). Cells with a verdict different from the preregistered one are in bold.", "",
          "| test | cell | (a) prereg bootstrap 95% | (b) t 95% | (c) BCa 95% | (d) TOST 90% (H1) |", "|---|---|---|---|---|---|"]
    tally = {k: [0, 0] for k in ("boot", "t", "bca")}
    for h in ("H1", "H2", "H3", "H4", "H5"):
        for g, t in cells:
            grp = [p for p in pts if p["h"] == h and p["g"] == g and p["t"] == t]
            if not grp: continue
            v = {k: verdict(h, k, grp) for k in ("boot", "t", "bca")}
            for k in v: tally[k][0] += v[k] == "PASS"; tally[k][1] += 1
            cell = lambda k: v[k] if v[k] == v["boot"] else f"**{v[k]}**"
            td = verdict(h, "tost", grp) if h == "H1" else "-"
            if h == "H1" and td != v["boot"]: td = f"**{td}**"
            L.append(f"| {h} | {g} t{t} | {v['boot']} | {cell('t')} | {cell('bca')} | {td} |")
    L += ["", f"Cells passed (of {tally['boot'][1]}): bootstrap {tally['boot'][0]}, t {tally['t'][0]}, BCa {tally['bca'][0]}.", ""]
    for h in ("H1", "H2", "H3", "H4", "H5"):
        for g, t in cells:
            grp = [p for p in pts if p["h"] == h and p["g"] == g and p["t"] == t]
            if not grp: continue
            vs = {k: verdict(h, k, grp) for k in ("boot", "t", "bca")}
            if len(set(vs.values())) > 1:
                bad = [p for p in grp if not passes(h, p["m"], *(p["t95"] if vs["t"] == "FAIL" else p["bca"] if vs["bca"] == "FAIL" else p["boot"]))]
                L.append(f"- {h} {g} t{t}: bootstrap {vs['boot']}, t {vs['t']}, BCa {vs['bca']}. Offending point(s): " +
                         "; ".join(f"lam={p['lam']:g} {p['link']} mean {f3(p['m'])}, boot {ci(*p['boot'])}, t {ci(*p['t95'])}, BCa {ci(*p['bca'])}" for p in bad))
    L += ["", "## 4. Reading", "",
          "- H2, H3, H5 (the substantive effects) pass under all three interval methods in every cell.",
          "- H1: see section 2. The failing preregistered cell (6x6 t0, lambda=2) passes standard TOST; at the same time the "
          "uncorrected t test for ORACLE > IMPLICIT is nominally significant there, and does not survive Holm over 12 points. "
          "The defensible wording is 'no H1 point is significant after Holm correction', not 'never significant'.",
          "- H4: pass/fail pattern across cells depends on the method (section 3); report it as method-dependent.",
          "- The 0.03 equivalence margin (about 4-5% relative to P(IMPLICIT) of 0.58-0.79) is a judgement; effects the paper calls meaningful under H2 are 0.03-0.06, "
          "the same order, so H1 equivalence is a statement about 'no larger than the smallest harm we report'.", ""]
    open(a.out, "w").write("\n".join(L) + "\n")
    print("wrote", a.out)
    print("\n".join(l for l in L if l.startswith("|") and "PASS" in l or "FAIL" in l))


if __name__ == "__main__":
    main()
