"""Failure signatures and clustering with bootstrap stability. Clusters are not causes.

A signature is a set of tokens: outcome, the first violated Property, the phase it happened in,
the last few decision labels before it, the divergence split class and condition bins. Runs are
grouped by average-linkage agglomeration on Jaccard distance; a cluster is reported only when
its bootstrap stability (mean best-Jaccard match across resamples, Hennig 2007) clears 0.6.
Rates from adaptive or falsification batches are flagged as non-operational by the caller.
"""

from __future__ import annotations

from collections import Counter
import random

from judge.insight import Insight

STABLE = 0.6


def jaccard(a, b):
    return len(a & b) / len(a | b) if a | b else 1.0


def agglomerate(sigs, cut=0.5):
    """Average-linkage clusters (lists of indices) with Jaccard distance below ``cut``."""
    clusters = [[i] for i in range(len(sigs))]

    def dist(c1, c2):
        return sum(1 - jaccard(sigs[i], sigs[j]) for i in c1 for j in c2) / (len(c1) * len(c2))
    while len(clusters) > 1:
        best, pair = None, None
        for x in range(len(clusters)):
            for y in range(x + 1, len(clusters)):
                d = dist(clusters[x], clusters[y])
                if best is None or d < best:
                    best, pair = d, (x, y)
        if best is None or best >= cut:
            break
        x, y = pair
        clusters[x] = clusters[x] + clusters[y]
        del clusters[y]
    return clusters


def stability(sigs, clusters, B=50, seed=0, cut=0.5):
    rng = random.Random(seed)
    n = len(sigs)
    scores = [[] for _ in clusters]
    for _ in range(B):
        idx = sorted(set(rng.randrange(n) for _ in range(n)))
        sub = [sigs[i] for i in idx]
        bc = [set(idx[k] for k in c) for c in agglomerate(sub, cut)]
        for ci, c in enumerate(clusters):
            members = set(c) & set(idx)
            if not members:
                continue
            scores[ci].append(max(jaccard(members, b) for b in bc))
    return [sum(s) / len(s) if s else 0.0 for s in scores]


def cluster_failures(items, cut=0.5, B=50, seed=0, min_size=2):
    """``items`` [(run_id, signature set)] of failing runs -> stable cluster insights."""
    if len(items) < min_size:
        return [], []
    sigs = [frozenset(s) for _, s in items]
    clusters = agglomerate(sigs, cut)
    stab = stability(sigs, clusters, B, seed, cut)
    out, rows = [], []
    for c, s in sorted(zip(clusters, stab), key=lambda cs: -len(cs[0])):
        common = frozenset.intersection(*[sigs[i] for i in c])
        tokens = Counter(t for i in c for t in sigs[i])
        row = {'runs': [items[i][0] for i in c], 'size': len(c), 'stability': s,
               'common': sorted(common), 'top_tokens': tokens.most_common(6)}
        rows.append(row)
        if len(c) >= min_size and s >= STABLE:
            out.append(Insight('clusters.failure', 'warn',
                               f'{len(c)} failing runs share {", ".join(sorted(common)) or "no single token"} '
                               f'(stability {s:.2f})',
                               scope={'runs': row['runs']}, confidence={'bootstrap_stability': s, 'n': len(c)},
                               data=row, basis='statistical'))
    return out, rows
