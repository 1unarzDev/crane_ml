"""One statistics module for the judge, the referee and the cluster's promotion.

Pure Python (no numpy needed) so the stdlib-only cluster side can vendor or import it. Golden
parity tests (``tests/test_stats.py`` here; downstream parity tests pin these formulas from their
side) keep a promote decision and a non-inferiority verdict from disagreeing on one data set. The
unit of analysis is the caller's: scenario blocks or sessions, never samples.
"""

from __future__ import annotations

import math
import random
from typing import Callable, Mapping, Sequence

Z95 = 1.959964
EXACT_MAX_N = 30


def z_upper(alpha):
    """Upper-tail standard normal quantile (bisection on erfc), as reliability._z."""
    lo, hi = 0.0, 10.0
    for _ in range(100):
        mid = (lo + hi) / 2
        lo, hi = (mid, hi) if math.erfc(mid / math.sqrt(2)) / 2 > alpha else (lo, mid)
    return (lo + hi) / 2


# ------------------------------------------------------------------ proportions

def wilson(k, n, z=Z95):
    p = k / n
    centre = (p + z * z / (2 * n)) / (1 + z * z / n)
    half = z * math.sqrt(p * (1 - p) / n + z * z / (4 * n * n)) / (1 + z * z / n)
    return max(0.0, centre - half), min(1.0, centre + half)


def _betainc(a, b, x):
    """Regularised incomplete beta I_x(a, b) (Lentz continued fraction)."""
    if x <= 0:
        return 0.0
    if x >= 1:
        return 1.0
    if x > (a + 1) / (a + b + 2):
        return 1.0 - _betainc(b, a, 1 - x)
    lbeta = math.lgamma(a + b) - math.lgamma(a) - math.lgamma(b)
    front = math.exp(math.log(x) * a + math.log(1 - x) * b + lbeta) / a
    tiny, f, c, d = 1e-300, 1.0, 1.0, 0.0
    for i in range(400):
        m = i // 2
        if i == 0:
            num = 1.0
        elif i % 2 == 0:
            num = m * (b - m) * x / ((a + 2 * m - 1) * (a + 2 * m))
        else:
            num = -((a + m) * (a + b + m) * x) / ((a + 2 * m) * (a + 2 * m + 1))
        d = 1.0 + num * d
        d = 1.0 / (d if abs(d) > tiny else tiny)
        c = 1.0 + num / c
        c = c if abs(c) > tiny else tiny
        f *= c * d
        if abs(1.0 - c * d) < 1e-12:
            break
    return front * (f - 1.0)


def clopper_pearson(k, n, alpha=0.05):
    """Exact two-sided (1 - alpha) interval for k of n."""
    def solve(target, lo_side):
        lo, hi = 0.0, 1.0
        for _ in range(100):
            mid = (lo + hi) / 2
            # P(X >= k | mid) for the lower bound, P(X <= k | mid) for the upper
            val = _betainc(k, n - k + 1, mid) if lo_side else 1 - _betainc(k + 1, n - k, mid)
            if lo_side:
                lo, hi = (mid, hi) if val < target else (lo, mid)
            else:
                lo, hi = (mid, hi) if val > target else (lo, mid)
        return (lo + hi) / 2
    lower = 0.0 if k == 0 else solve(alpha / 2, True)
    upper = 1.0 if k == n else solve(alpha / 2, False)
    return lower, upper


def newcombe(k1, n1, k2, n2, z=Z95):
    """p2 - p1 and the Newcombe hybrid score interval (method 10, unpaired)."""
    l1, u1 = wilson(k1, n1, z)
    l2, u2 = wilson(k2, n2, z)
    p1, p2 = k1 / n1, k2 / n2
    d = p2 - p1
    return d, (d - math.hypot(p2 - l2, u1 - p1), d + math.hypot(u2 - p2, p1 - l1))


def newcombe_paired(both, cand_only, base_only, neither, z=Z95):
    """p_candidate - p_baseline for paired 0/1 outcomes, Newcombe's paired score interval
    (1998, method 10, continuity-corrected phi). Same convention as reliability.py."""
    a, b, c, d = both, cand_only, base_only, neither
    n = a + b + c + d
    p1, p2 = (a + b) / n, (a + c) / n
    l1, u1 = wilson(a + b, n, z)
    l2, u2 = wilson(a + c, n, z)
    margins = (a + b) * (c + d) * (a + c) * (b + d)
    if not margins:
        phi = 0.0
    else:
        num = a * d - b * c
        phi = (max(num - n / 2, 0.0) if num > 0 else num) / math.sqrt(margins)
    theta = p1 - p2
    lo = theta - math.sqrt(max(0.0, (p1 - l1) ** 2 - 2 * phi * (p1 - l1) * (u2 - p2) + (u2 - p2) ** 2))
    hi = theta + math.sqrt(max(0.0, (u1 - p1) ** 2 - 2 * phi * (u1 - p1) * (p2 - l2) + (p2 - l2) ** 2))
    return theta, (lo, hi)


def mcnemar_exact(b, c):
    """Two-sided exact McNemar p on the discordant counts."""
    n = b + c
    if n == 0:
        return 1.0
    k = min(b, c)
    tail = sum(math.comb(n, i) for i in range(k + 1)) / 2 ** n
    return min(1.0, 2 * tail)


# ------------------------------------------------------------------ paired continuous

def ranks(values):
    """1-based ranks with ties averaged."""
    order = sorted(range(len(values)), key=lambda i: values[i])
    out = [0.0] * len(values)
    i = 0
    while i < len(order):
        j = i
        while j + 1 < len(order) and values[order[j + 1]] == values[order[i]]:
            j += 1
        for k in range(i, j + 1):
            out[order[k]] = (i + j) / 2 + 1
        i = j + 1
    return out


def wilcoxon_greater(diffs):
    """One-sided signed-rank test of H1 median > 0: (p, n_used, w_plus). Exact for n <= 30
    (doubled ranks for ties), normal with tie and continuity correction above; zeros dropped.
    Identical to cluster promotion.wilcoxon_greater (golden test)."""
    d = [x for x in diffs if x != 0]
    n = len(d)
    if n == 0:
        return 1.0, 0, 0.0
    r = ranks([abs(x) for x in d])
    w_plus = sum(rk for rk, x in zip(r, d) if x > 0)
    if n <= EXACT_MAX_N:
        doubled = [int(round(2 * rk)) for rk in r]
        total = sum(doubled)
        counts = [0] * (total + 1)
        counts[0] = 1
        for rk in doubled:
            for s in range(total, rk - 1, -1):
                counts[s] += counts[s - rk]
        obs = int(round(2 * w_plus))
        return sum(counts[obs:]) / 2 ** n, n, w_plus
    mean = n * (n + 1) / 4
    ties = {}
    for rk in r:
        ties[rk] = ties.get(rk, 0) + 1
    var = n * (n + 1) * (2 * n + 1) / 24 - sum(t ** 3 - t for t in ties.values()) / 48
    z = (w_plus - mean - 0.5) / math.sqrt(var) if var > 0 else 0.0
    return 0.5 * math.erfc(z / math.sqrt(2)), n, w_plus


def wilcoxon_two_sided(diffs):
    p_hi, n, _ = wilcoxon_greater(diffs)
    p_lo, _, _ = wilcoxon_greater([-x for x in diffs])
    return min(1.0, 2 * min(p_hi, p_lo)), n


def t_cdf(t, df):
    x = df / (df + t * t)
    tail = 0.5 * _betainc(df / 2, 0.5, x)
    return 1 - tail if t > 0 else tail


def t_quantile(p, df):
    lo, hi = -100.0, 100.0
    for _ in range(200):
        mid = (lo + hi) / 2
        lo, hi = (mid, hi) if t_cdf(mid, df) < p else (lo, mid)
    return (lo + hi) / 2


def paired_t(diffs, alpha=0.05):
    """(mean, (lo, hi) two-sided 1-alpha CI, two-sided p, n)."""
    n = len(diffs)
    if n < 2:
        m = diffs[0] if diffs else float('nan')
        return m, (float('-inf'), float('inf')), 1.0, n
    m = sum(diffs) / n
    sd = math.sqrt(sum((x - m) ** 2 for x in diffs) / (n - 1))
    se = sd / math.sqrt(n)
    if se == 0:
        return m, (m, m), (0.0 if m != 0 else 1.0), n
    q = t_quantile(1 - alpha / 2, n - 1)
    t = m / se
    return m, (m - q * se, m + q * se), 2 * min(t_cdf(t, n - 1), 1 - t_cdf(t, n - 1)), n


def welch_t(a, b, alpha=0.05):
    """mean(b) - mean(a), Welch CI and two-sided p."""
    na, nb = len(a), len(b)
    if na < 2 or nb < 2:
        return float('nan'), (float('-inf'), float('inf')), 1.0
    ma, mb = sum(a) / na, sum(b) / nb
    va = sum((x - ma) ** 2 for x in a) / (na - 1)
    vb = sum((x - mb) ** 2 for x in b) / (nb - 1)
    se = math.sqrt(va / na + vb / nb)
    if se == 0:
        return mb - ma, (mb - ma, mb - ma), 1.0 if mb == ma else 0.0
    df = (va / na + vb / nb) ** 2 / ((va / na) ** 2 / (na - 1) + (vb / nb) ** 2 / (nb - 1))
    q = t_quantile(1 - alpha / 2, df)
    d = mb - ma
    t = d / se
    return d, (d - q * se, d + q * se), 2 * min(t_cdf(t, df), 1 - t_cdf(t, df))


def bootstrap_ci(units: Sequence, stat: Callable[[Sequence], float], B=2000, alpha=0.05, seed=0):
    """Percentile CI of ``stat`` resampling whole units (scenario blocks), never samples."""
    rng = random.Random(seed)
    n = len(units)
    if n == 0:
        return float('nan'), (float('nan'), float('nan'))
    vals = sorted(stat([units[rng.randrange(n)] for _ in range(n)]) for _ in range(B))
    lo, hi = vals[int(alpha / 2 * B)], vals[min(B - 1, int((1 - alpha / 2) * B))]
    return stat(list(units)), (lo, hi)


# ------------------------------------------------------------------ multiplicity, ranking

def holm(pvalues: Mapping[str, float]) -> dict:
    items = sorted(pvalues.items(), key=lambda kv: kv[1])
    m, out, running = len(items), {}, 0.0
    for i, (k, p) in enumerate(items):
        running = max(running, min(1.0, (m - i) * p))
        out[k] = running
    return out


def halving_survivors(scores: Mapping[str, float], eta: int = 2) -> list:
    if not scores:
        return []
    keep = max(1, math.ceil(len(scores) / max(2, eta)))
    return sorted(scores, key=lambda k: (-scores[k], k))[:keep]


def kendall_tau(a, b):
    """Kendall tau-b."""
    n = len(a)
    conc = disc = ta = tb = 0
    for i in range(n):
        for j in range(i + 1, n):
            da, db = a[i] - a[j], b[i] - b[j]
            if da == 0 and db == 0:
                continue
            if da == 0:
                ta += 1
            elif db == 0:
                tb += 1
            elif da * db > 0:
                conc += 1
            else:
                disc += 1
    den = math.sqrt((conc + disc + ta) * (conc + disc + tb))
    return (conc - disc) / den if den else float('nan')


def pearson(a, b):
    n = len(a)
    if n < 2:
        return float('nan')
    ma, mb = sum(a) / n, sum(b) / n
    sa = math.sqrt(sum((x - ma) ** 2 for x in a))
    sb = math.sqrt(sum((x - mb) ** 2 for x in b))
    return sum((x - ma) * (y - mb) for x, y in zip(a, b)) / (sa * sb) if sa and sb else float('nan')


def spearman(a, b):
    return pearson(ranks(list(a)), ranks(list(b)))


# ------------------------------------------------------------------ decisions

def ni_verdict(estimate, interval, margin, direction='higher'):
    """Non-inferiority of candidate - baseline with a declared margin (metric units, > 0).
    'higher' is better: non-inferior when the lower bound > -margin; inferior when the upper bound
    < -margin; otherwise inconclusive. Without a margin the answer is 'margin undeclared'."""
    if margin is None:
        return 'margin undeclared'
    lo, hi = interval
    if direction == 'lower':
        lo, hi = -hi, -lo
    if lo > -margin:
        return 'superior' if lo > 0 else 'non-inferior'
    if hi < -margin:
        return 'inferior'
    return 'inconclusive'
