"""Multiplicity check for the agent-loss (health beacon) result on fresh seeds 800-811.

Reads results/beacon_final_A_raw.npy (layout written by beacon_robustness.py:
  arr[0][regime] = (det, bytes); det["<link>|<STRATEGY>"][seed] = list of 200 per-episode 0/1 detections,
  episodes paired by index across strategies).

For HEALTH - IMPLICIT and HEALTH - INDEP in R7a-d (ideal link d0_loss0; also the bursty link for HEALTH - IMPLICIT):
  * instance-level paired t-test (12 instance means of the paired differences, df = 11), two-sided
  * cluster bootstrap (instances, then episodes; same scheme as falsify.boot), two-sided percentile p-value
    p = 2 * min(P*(mean <= 0), P*(mean >= 0)), B = 20000 (resolution 1e-4), plus the B=2000 seed-1 CI that
    reproduces the published table.
Holm and Benjamini-Hochberg adjustment within each 4-regime family, and over the combined 8-test family.
No scipy: Student-t CDF via the regularized incomplete beta function (continued fraction).

Run:  python3 scripts/loss_multiplicity.py  (from Tools/ForestSearch)
"""
import math
import os
import sys

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
RAW = os.path.join(HERE, "..", "results", "beacon_final_A_raw.npy")
REGIMES = ["R7a", "R7b", "R7c", "R7d"]


def _betacf(a, b, x, itmax=500, eps=1e-15):
    qab, qap, qam = a + b, a + 1.0, a - 1.0
    c, d = 1.0, 1.0 - qab * x / qap
    d = 1.0 / (d if abs(d) > 1e-300 else 1e-300)
    h = d
    for m in range(1, itmax + 1):
        m2 = 2 * m
        aa = m * (b - m) * x / ((qam + m2) * (a + m2))
        d = 1.0 + aa * d; d = 1.0 / (d if abs(d) > 1e-300 else 1e-300)
        c = 1.0 + aa / c; c = c if abs(c) > 1e-300 else 1e-300
        h *= d * c
        aa = -(a + m) * (qab + m) * x / ((a + m2) * (qap + m2))
        d = 1.0 + aa * d; d = 1.0 / (d if abs(d) > 1e-300 else 1e-300)
        c = 1.0 + aa / c; c = c if abs(c) > 1e-300 else 1e-300
        de = d * c
        h *= de
        if abs(de - 1.0) < eps:
            break
    return h


def betainc(a, b, x):
    if x <= 0:
        return 0.0
    if x >= 1:
        return 1.0
    lbt = math.lgamma(a + b) - math.lgamma(a) - math.lgamma(b) + a * math.log(x) + b * math.log(1 - x)
    if x < (a + 1) / (a + b + 2):
        return math.exp(lbt) * _betacf(a, b, x) / a
    return 1.0 - math.exp(lbt) * _betacf(b, a, 1 - x) / b


def t_two_sided_p(t, df):
    return betainc(df / 2.0, 0.5, df / (df + t * t))


def t_quantile(p, df):
    lo, hi = 0.0, 50.0
    for _ in range(200):
        mid = (lo + hi) / 2
        if 1 - 0.5 * t_two_sided_p(mid, df) < p:
            lo = mid
        else:
            hi = mid
    return (lo + hi) / 2


def holm(ps):
    ps = np.asarray(ps, float); m = len(ps); o = np.argsort(ps)
    adj = np.empty(m); run = 0.0
    for r, i in enumerate(o):
        run = max(run, min(1.0, (m - r) * ps[i])); adj[i] = run
    return adj


def bh(ps):
    ps = np.asarray(ps, float); m = len(ps); o = np.argsort(ps)[::-1]
    adj = np.empty(m); run = 1.0
    for k, i in enumerate(o):
        r = m - k
        run = min(run, ps[i] * m / r); adj[i] = run
    return adj


def cluster_boot(diff, B, seed):
    rng = np.random.default_rng(seed)
    seeds = sorted(diff)
    bs = np.empty(B)
    for b in range(B):
        ss = rng.choice(seeds, len(seeds))
        bs[b] = np.mean(np.concatenate([rng.choice(diff[s], len(diff[s])) for s in ss]))
    return bs


def analyse(det, a, b):
    seeds = sorted(det[a])
    diff = {s: np.asarray(det[a][s], float) - np.asarray(det[b][s], float) for s in seeds}
    m = float(np.mean(np.concatenate([diff[s] for s in seeds])))
    inst = np.array([diff[s].mean() for s in seeds])
    n = len(inst); se = inst.std(ddof=1) / math.sqrt(n); t = inst.mean() / se
    q = t_quantile(0.975, n - 1)
    pt = t_two_sided_p(t, n - 1)
    bs_ci = cluster_boot(diff, 2000, 1)  # published CI convention (falsify.boot, rng seed 1)
    bs = cluster_boot(diff, 20000, 12345)
    pb = min(1.0, 2 * min(np.mean(bs <= 0), np.mean(bs >= 0)))
    pb = max(pb, 1.0 / len(bs))
    return dict(mean=m, ci=(np.percentile(bs_ci, 2.5), np.percentile(bs_ci, 97.5)), t=t, pt=pt,
                tci=(inst.mean() - q * se, inst.mean() + q * se), pb=pb, npos=int((inst > 0).sum()), n=n)


def main():
    arr = np.load(RAW, allow_pickle=True)[0]
    fams = [("HEALTH - IMPLICIT, ideal link", "d0_loss0|HEALTH", "d0_loss0|IMPLICIT"),
            ("HEALTH - INDEP, ideal link", "d0_loss0|HEALTH", "d0_loss0|INDEP"),
            ("HEALTH - IMPLICIT, bursty link d1_burst30 (secondary)", "d1_burst30|HEALTH", "d0_loss0|IMPLICIT")]
    out = []
    allres = {}
    for title, a, b in fams:
        res = [analyse(arr[r][0], a, b) for r in REGIMES]
        allres[title] = res
    combined_t = np.concatenate([[r["pt"] for r in allres[f[0]]] for f in fams[:2]])
    combined_b = np.concatenate([[r["pb"] for r in allres[f[0]]] for f in fams[:2]])
    ht8, hb8 = holm(combined_t), holm(combined_b)
    for fi, (title, a, b) in enumerate(fams):
        res = allres[title]
        pt = [r["pt"] for r in res]; pb = [r["pb"] for r in res]
        H_t, B_t, H_b, B_b = holm(pt), bh(pt), holm(pb), bh(pb)
        out.append(f"\n### {title}\n")
        hdr = "| regime | d | boot 95% CI (B=2000) | t_11 95% CI | inst>0 | t | p_t | Holm4 p_t | BH4 p_t | p_boot | Holm4 p_boot | BH4 p_boot |"
        if fi < 2:
            hdr += " Holm8 p_t | Holm8 p_boot |"
        out.append(hdr)
        out.append("|" + "---|" * (hdr.count("|") - 1))
        for k, (r, x) in enumerate(zip(REGIMES, res)):
            row = (f"| {r} | {x['mean']:+.3f} | [{x['ci'][0]:+.3f},{x['ci'][1]:+.3f}] | [{x['tci'][0]:+.3f},{x['tci'][1]:+.3f}] | "
                   f"{x['npos']}/{x['n']} | {x['t']:.2f} | {x['pt']:.2g} | {H_t[k]:.2g} | {B_t[k]:.2g} | {x['pb']:.2g} | "
                   f"{H_b[k]:.2g} | {B_b[k]:.2g} |")
            if fi < 2:
                row += f" {ht8[fi*4+k]:.2g} | {hb8[fi*4+k]:.2g} |"
            out.append(row)
    txt = "\n".join(out)
    print(txt)
    return txt


if __name__ == "__main__":
    sys.exit(0 if main() else 1)
