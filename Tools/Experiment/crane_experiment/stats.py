"""Small statistics helpers in numpy: Mann-Whitney U (one-sided) and percentile bootstrap."""
import math

import numpy as np


def _ranks(v):
    order = np.argsort(v, kind="mergesort")
    r = np.empty(len(v))
    sv = np.asarray(v)[order]
    i = 0
    while i < len(sv):
        j = i
        while j + 1 < len(sv) and sv[j + 1] == sv[i]:
            j += 1
        r[order[i:j + 1]] = (i + j) / 2.0 + 1.0
        i = j + 1
    return r


def u_counts(m, n):
    """Number of arrangements giving each Mann-Whitney U in 0..m*n (exact DP, no ties).
    N(u; m, n) = N(u - n; m - 1, n) + N(u; m, n - 1)."""
    prev = [np.zeros(1, dtype=np.int64) for _ in range(n + 1)]       # row i-1 = 0 -> N(0;0,j)=1
    for j in range(n + 1):
        prev[j][0] = 1
    for i in range(1, m + 1):
        cur = [None] * (n + 1)
        cur[0] = np.array([1], dtype=np.int64)
        for j in range(1, n + 1):
            out = np.zeros(i * j + 1, dtype=np.int64)
            out[: len(cur[j - 1])] += cur[j - 1]                     # N(u; i, j-1)
            out[j: j + len(prev[j])] += prev[j]                      # N(u-j; i-1, j)
            cur[j] = out
        prev = cur
    return prev[n]


def mannwhitney_less(a, b, exact_max=20):
    """One-sided p for H1: a stochastically LESS than b.
    Exact (counting DP) when both samples <= exact_max and there are no ties; otherwise a
    tie- and continuity-corrected normal approximation (floor ~1e-4 at 10 vs 10)."""
    a, b = np.asarray(a, float), np.asarray(b, float)
    n1, n2 = len(a), len(b)
    if n1 <= exact_max and n2 <= exact_max and len(np.unique(np.concatenate([a, b]))) == n1 + n2:
        u = int((a[:, None] > b[None, :]).sum())
        c = u_counts(n1, n2)
        return float(c[: u + 1].sum() / c.sum())
    allv = np.concatenate([a, b])
    r = _ranks(allv)
    u1 = r[:n1].sum() - n1 * (n1 + 1) / 2.0
    n = n1 + n2
    _, counts = np.unique(allv, return_counts=True)
    tie = (counts ** 3 - counts).sum()
    var = n1 * n2 / 12.0 * ((n + 1) - tie / (n * (n - 1)))
    if var <= 0:
        return 1.0
    z = (u1 - n1 * n2 / 2.0 + 0.5) / math.sqrt(var)    # small U <=> a < b
    return 0.5 * (1.0 + math.erf(z / math.sqrt(2.0)))


def bootstrap_ci(x, stat=np.median, n_boot=4000, alpha=0.05, seed=0):
    x = np.asarray(x, float)
    rng = np.random.default_rng(seed)
    idx = rng.integers(0, len(x), (n_boot, len(x)))
    s = np.array([stat(x[i]) for i in idx])
    return float(np.percentile(s, 100 * alpha / 2)), float(np.percentile(s, 100 * (1 - alpha / 2)))
