"""Promotion analysis: paired tests against the baseline on identical seeds, and
successive halving across variants. Pure functions over plain data; no I/O.

A stage's results are rows ``{arm, seed, name, value}``. Each variant is paired
with the baseline seed by seed, so scenario randomness cancels; the Wilcoxon
signed-rank test then asks whether the variant moves the metric in the claimed
direction. With several variants, p-values are Holm-corrected, and successive
halving can first spend seeds on the most promising variants.
"""
from __future__ import annotations

import math
from dataclasses import dataclass, field

EXACT_MAX_N = 30


def paired_diffs(rows, metric, variant, direction="increase"):
    """Direction-adjusted differences variant - baseline on seeds both arms ran (positive = better)."""
    by_arm: dict[str, dict[int, float]] = {}
    for r in rows:
        if r.get("name") == metric and r.get("value") is not None and r.get("status", "ok") == "ok":
            by_arm.setdefault(r["arm"], {})[int(r["seed"])] = float(r["value"])
    base, var = by_arm.get("baseline", {}), by_arm.get(variant, {})
    sign = 1.0 if direction == "increase" else -1.0
    return [sign * (var[s] - base[s]) for s in sorted(set(base) & set(var))]


def _ranks(values):
    """1-based ranks with ties averaged."""
    order = sorted(range(len(values)), key=lambda i: values[i])
    ranks = [0.0] * len(values)
    i = 0
    while i < len(order):
        j = i
        while j + 1 < len(order) and values[order[j + 1]] == values[order[i]]:
            j += 1
        for k in range(i, j + 1):
            ranks[order[k]] = (i + j) / 2 + 1
        i = j + 1
    return ranks


def wilcoxon_greater(diffs):
    """One-sided Wilcoxon signed-rank test of H1: median difference > 0.

    Zero differences are dropped. Exact null distribution for n <= 30 (ties
    handled through doubled ranks), normal approximation with tie and continuity
    correction above that. Returns (p_value, n_used, w_plus).
    """
    d = [x for x in diffs if x != 0]
    n = len(d)
    if n == 0:
        return 1.0, 0, 0.0
    ranks = _ranks([abs(x) for x in d])
    w_plus = sum(r for r, x in zip(ranks, d) if x > 0)
    if n <= EXACT_MAX_N:
        doubled = [int(round(2 * r)) for r in ranks]
        total = sum(doubled)
        counts = [0] * (total + 1)  # counts[s]: sign assignments with doubled W+ == s
        counts[0] = 1
        for r in doubled:
            for s in range(total, r - 1, -1):
                counts[s] += counts[s - r]
        obs = int(round(2 * w_plus))
        return sum(counts[obs:]) / 2**n, n, w_plus
    mean = n * (n + 1) / 4
    ties: dict[float, int] = {}
    for r in ranks:
        ties[r] = ties.get(r, 0) + 1
    var = n * (n + 1) * (2 * n + 1) / 24 - sum(t**3 - t for t in ties.values()) / 48
    z = (w_plus - mean - 0.5) / math.sqrt(var) if var > 0 else 0.0
    return 0.5 * math.erfc(z / math.sqrt(2)), n, w_plus


def holm(pvalues: dict[str, float]) -> dict[str, float]:
    """Holm-Bonferroni adjusted p-values (family-wise error control across variants)."""
    items = sorted(pvalues.items(), key=lambda kv: kv[1])
    m, out, running = len(items), {}, 0.0
    for i, (k, p) in enumerate(items):
        running = max(running, min(1.0, (m - i) * p))
        out[k] = running
    return out


def halving_survivors(scores: dict[str, float], eta: int = 2) -> list[str]:
    """Successive halving: keep the best ceil(k / eta) variants by score (higher is better)."""
    if not scores:
        return []
    keep = max(1, math.ceil(len(scores) / max(2, eta)))
    return sorted(scores, key=lambda k: (-scores[k], k))[:keep]


@dataclass
class VariantStats:
    n: int
    mean_diff: float
    median_diff: float
    p_better: float
    p_worse: float
    p_better_adj: float = 1.0
    p_worse_adj: float = 1.0


@dataclass
class Decision:
    action: str  # wait | extend | promote | reject
    reason: str
    variants: list[str] | None = None
    seeds: list[int] | None = None
    stats: dict[str, VariantStats] = field(default_factory=dict)

    def evidence(self) -> dict:
        return {"reason": self.reason, "stats": {k: vars(v) for k, v in self.stats.items()}}


def variant_stats(rows, metric, direction, variants) -> dict[str, VariantStats]:
    out = {}
    for v in variants:
        d = paired_diffs(rows, metric, v, direction)
        p_b, n, _ = wilcoxon_greater(d)
        p_w, _, _ = wilcoxon_greater([-x for x in d])
        s = sorted(d)
        med = (s[(len(s) - 1) // 2] + s[len(s) // 2]) / 2 if s else 0.0
        out[v] = VariantStats(n=len(d), mean_diff=sum(d) / len(d) if d else 0.0, median_diff=med, p_better=p_b, p_worse=p_w)
    for attr in ("better", "worse"):
        adj = holm({v: getattr(st, f"p_{attr}") for v, st in out.items()})
        for v, p in adj.items():
            setattr(out[v], f"p_{attr}_adj", p)
    return out


def decide(results: dict, hyp: dict, runs_used: int | None = None) -> Decision:
    """Next step for a hypothesis stage.

    results: the coordinator's /v1/hyp/:id/results body (rows, pending, dead).
    hyp: the hypothesis (metric, direction, variants, seeds, budget, promotion).
    promotion settings: alpha (0.05), min_pairs (6), min_effect (0, in metric units,
    direction-adjusted), halving_eta (2; 0 disables), max_seeds (stage cap on seeds).
    """
    if results.get("pending", 0) > 0:
        return Decision("wait", f"{results['pending']} jobs still queued or running")
    cfg = hyp.get("promotion") or {}
    alpha = float(cfg.get("alpha", 0.05))
    min_pairs = int(cfg.get("min_pairs", 6))
    min_effect = float(cfg.get("min_effect", 0.0))
    eta = int(cfg.get("halving_eta", 2))
    rows = results.get("rows", [])
    active = [v["name"] for v in hyp["variants"] if v.get("active", True)]
    seeds_run = sorted({int(r["seed"]) for r in rows})
    max_seeds = int(cfg.get("max_seeds", max(4 * len(hyp["seeds"]), min_pairs)))
    budget = (hyp.get("budget") or {}).get("max_runs")

    def more_seeds(arms: int) -> list[int] | None:
        n_new = min(max(len(seeds_run), min_pairs), max_seeds - len(seeds_run))
        if n_new <= 0:
            return None
        if budget is not None and runs_used is not None:
            n_new = min(n_new, (int(budget) - runs_used) // (arms + 1))
        if n_new <= 0:
            return None
        start = max(seeds_run + list(hyp["seeds"]) + [0]) + 1
        return list(range(start, start + n_new))

    stats = variant_stats(rows, hyp["metric"], hyp["direction"], active)
    if eta >= 2 and len(active) > 1:
        survivors = halving_survivors({v: s.mean_diff for v, s in stats.items()}, eta)
        seeds = more_seeds(len(survivors))
        if seeds:
            return Decision("extend", f"successive halving keeps {survivors} of {active}", survivors, seeds, stats)
        active = survivors  # out of budget: judge the survivors on what they have
    winners = [v for v in active
               if stats[v].n >= min_pairs and stats[v].p_better_adj < alpha and stats[v].median_diff >= min_effect]
    if winners:
        winners.sort(key=lambda v: -stats[v].mean_diff)
        return Decision("promote", f"{winners} beat the baseline (paired Wilcoxon, Holm alpha {alpha})", winners, None, stats)
    hopeless = all(stats[v].n >= min_pairs and stats[v].p_worse_adj < alpha for v in active)
    if hopeless:
        return Decision("reject", "every active variant is significantly worse than the baseline", None, None, stats)
    seeds = more_seeds(len(active))
    if seeds:
        return Decision("extend", "inconclusive; adding seeds", active, seeds, stats)
    return Decision("reject", "inconclusive within the seed cap or budget", None, None, stats)
