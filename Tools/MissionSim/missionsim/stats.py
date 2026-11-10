# SPDX-License-Identifier: GPL-3.0-or-later
"""Paired comparisons with cluster bootstrap over layouts (missions within a layout are not independent)."""
import numpy as np


def cluster_boot_diff(rows, pol_a, pol_b, key, n_boot=2000, seed=0, cond=None):
    """Mean over layouts of (a - b) on `key`, paired by (layout, rep, variant); CI by resampling layouts.

    rows: list of mission dicts with fields policy, layout, rep, pair_key (optional), key.
    Returns (mean_diff, lo, hi, n_layouts, n_pairs)."""
    by = {}
    for r in rows:
        if cond is not None and not cond(r):
            continue
        pk = (r["layout"], r["rep"], r.get("pair_key", ""))
        by.setdefault(pk, {})[r["policy"]] = r
    per_layout = {}
    for pk, d in by.items():
        if pol_a in d and pol_b in d:
            va, vb = _val(d[pol_a], key), _val(d[pol_b], key)
            if va is None or vb is None:
                continue
            per_layout.setdefault(pk[0], []).append(va - vb)
    if not per_layout:
        return (float("nan"),) * 3 + (0, 0)
    L = sorted(per_layout)
    means = np.array([np.mean(per_layout[l]) for l in L])
    w = np.array([len(per_layout[l]) for l in L], float)
    est = float(np.sum(means * w) / w.sum())
    r = np.random.default_rng(seed)
    bs = []
    for _ in range(n_boot):
        idx = r.integers(0, len(L), len(L))
        bs.append(np.sum(means[idx] * w[idx]) / w[idx].sum())
    lo, hi = np.percentile(bs, [2.5, 97.5])
    return est, float(lo), float(hi), len(L), int(w.sum())


def cluster_boot_mean(rows, pol, key, n_boot=2000, seed=0, cond=None):
    per = {}
    for r in rows:
        if r["policy"] != pol or (cond is not None and not cond(r)):
            continue
        v = _val(r, key)
        if v is None:
            continue
        per.setdefault(r["layout"], []).append(v)
    if not per:
        return float("nan"), float("nan"), float("nan"), 0
    L = sorted(per)
    means = np.array([np.mean(per[l]) for l in L])
    w = np.array([len(per[l]) for l in L], float)
    est = float(np.sum(means * w) / w.sum())
    r = np.random.default_rng(seed)
    bs = [np.sum(means[i] * w[i]) / w[i].sum() for i in (r.integers(0, len(L), len(L)) for _ in range(n_boot))]
    lo, hi = np.percentile(bs, [2.5, 97.5])
    return est, float(lo), float(hi), int(w.sum())


def _val(r, key):
    if callable(key):
        return key(r)
    v = r.get(key)
    if v is None:
        return None
    return float(v)


def quantiles(vals, qs=(0.25, 0.5, 0.75)):
    v = [x for x in vals if x is not None]
    if not v:
        return [None] * len(qs)
    return [float(np.quantile(v, q)) for q in qs]
