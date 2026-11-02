"""Comparisons: paired variants, cross-tier agreement (trust gates G3/G4), exclusion accounting.

Every comparison carries its design (exact / block / loose / unpaired), its unit count and what
was excluded and why. Invalid runs (``Validity.valid`` False or the gate in ``excluded_from``)
never enter a gate silently: they are counted.

G3 (does the low tier pick what the high tier would pick?) is decided by selection regret,
MMRV (SIMPLER's mean maximum rank violation) and top-k recall; Kendall tau and Pearson are context.
G4 (are the low tier's numbers close enough?) is a per-metric paired gap: bias with a unit-level
bootstrap CI, spread, band coverage against the objective's declared band, plus a failure-mode
confusion table, because rank agreement and failure-mode agreement are different questions.
"""

from __future__ import annotations

from collections import Counter
from dataclasses import asdict, dataclass, field
import math
from typing import Any, Mapping, Optional, Sequence

from judge import stats
from judge.metric import MetricValue
from judge.record import RunRecord
from judge.task import Objective


@dataclass
class Scored:
    rec: RunRecord
    values: Mapping[str, MetricValue]
    outcome: str = 'unknown'

    def get(self, metric):
        v = self.values.get(metric)
        return v.value if v is not None and v.usable else None


@dataclass
class Comparison:
    metric: str
    design: str
    n_units: int
    estimate: Optional[float]
    interval: tuple
    p: Optional[float]
    test: str
    verdict: str
    role: str = 'task'
    direction: str = 'higher'
    margin: Optional[float] = None
    p_holm: Optional[float] = None
    excluded: Mapping[str, int] = field(default_factory=dict)
    a_mean: Optional[float] = None
    b_mean: Optional[float] = None

    def as_dict(self):
        return asdict(self)


def gate_ok(s: Scored, gate):
    return s.rec.validity.valid and gate not in s.rec.validity.excluded_from


def _is_binary(xs):
    return all(x in (0, 1, 0.0, 1.0, True, False) for x in xs)


def compare_paired(pairs: Sequence, objective: Objective, design='exact', alpha=0.05, gate='compare'):
    """``pairs`` [(key, a: Scored, b: Scored)], estimate b - a (b = candidate)."""
    m = objective.metric
    excluded = Counter()
    da, db = [], []
    for _, a, b in pairs:
        if not (gate_ok(a, gate) and gate_ok(b, gate)):
            excluded['invalid'] += 1
            continue
        va, vb = a.get(m), b.get(m)
        if va is None or vb is None:
            excluded['unscorable'] += 1
            continue
        da.append(float(va))
        db.append(float(vb))
    n = len(da)
    base = dict(metric=m, design=design, n_units=n, role=objective.role,
                direction=objective.direction, margin=objective.ni_margin, excluded=dict(excluded),
                a_mean=sum(da) / n if n else None, b_mean=sum(db) / n if n else None)
    if n == 0:
        return Comparison(estimate=None, interval=(None, None), p=None, test='none',
                          verdict='no usable pairs', **base)
    if _is_binary(da + db):
        both = sum(1 for x, y in zip(da, db) if x and y)
        cand = sum(1 for x, y in zip(da, db) if y and not x)
        bas = sum(1 for x, y in zip(da, db) if x and not y)
        # newcombe_paired returns p_candidate - p_baseline = (cand - bas) / n, i.e. b - a
        est, ci = stats.newcombe_paired(both, cand, bas, n - both - cand - bas,
                                        stats.z_upper(alpha / 2))
        p = stats.mcnemar_exact(cand, bas)
        test = 'paired binary: Newcombe paired score CI, exact McNemar'
    else:
        diffs = [y - x for x, y in zip(da, db)]
        est, ci, p_t, _ = stats.paired_t(diffs, alpha)
        p, _ = stats.wilcoxon_two_sided(diffs)
        test = 'paired: t CI on differences, Wilcoxon signed-rank p'
    verdict = stats.ni_verdict(est, ci, objective.ni_margin, objective.direction)
    if objective.ni_margin is None:
        better = (est > 0) == (objective.direction == 'higher')
        verdict = (('better' if better else 'worse') if p is not None and p < alpha
                   else 'no detected difference (not equivalence)')
    return Comparison(estimate=est, interval=tuple(ci), p=p, test=test, verdict=verdict, **base)


def holm_adjust(comparisons: Sequence[Comparison]):
    ps = {c.metric: c.p for c in comparisons if c.p is not None}
    adj = stats.holm(ps)
    for c in comparisons:
        c.p_holm = adj.get(c.metric)
    return comparisons


# ------------------------------------------------------------------ G3: ranking agreement

def tier_agreement(low: Mapping[str, Sequence[float]], high: Mapping[str, Sequence[float]],
                   direction='higher', k=1):
    """G3 over a roster of variants measured on both tiers (means per variant).

    regret: high-tier value lost by deploying the low tier's pick (metric units, >= 0);
    mmrv: SIMPLER's mean maximum rank violation; topk_recall: share of the high tier's top k the
    low tier also puts in its top k; kendall/pearson: context."""
    roster = sorted(v for v in low if v in high and low[v] and high[v])
    if len(roster) < 2:
        return {'roster': roster, 'n_variants': len(roster),
                'verdict': 'unmeasurable: fewer than 2 variants on both tiers'}
    sgn = 1 if direction == 'higher' else -1
    lo = {v: sgn * sum(low[v]) / len(low[v]) for v in roster}
    hi = {v: sgn * sum(high[v]) / len(high[v]) for v in roster}
    pick = max(roster, key=lambda v: (lo[v], v))
    best = max(roster, key=lambda v: (hi[v], v))
    regret = hi[best] - hi[pick]
    viol = []
    for i in roster:
        worst = 0.0
        for j in roster:
            if i != j and (lo[i] < lo[j]) != (hi[i] < hi[j]):
                worst = max(worst, abs(hi[i] - hi[j]))
        viol.append(worst)
    k = min(k, len(roster))
    top_lo = set(sorted(roster, key=lambda v: -lo[v])[:k])
    top_hi = set(sorted(roster, key=lambda v: -hi[v])[:k])
    return {'roster': roster, 'n_variants': len(roster), 'low_pick': pick, 'high_best': best,
            'regret': regret, 'mmrv': sum(viol) / len(viol), 'topk_recall': len(top_lo & top_hi) / k,
            'k': k, 'kendall_tau': stats.kendall_tau([lo[v] for v in roster], [hi[v] for v in roster]),
            'pearson': stats.pearson([lo[v] for v in roster], [hi[v] for v in roster]),
            'n_per_variant': {v: [len(low[v]), len(high[v])] for v in roster}}


# ------------------------------------------------------------------ G4: value agreement

def tier_gap(pairs: Sequence, objectives: Sequence[Objective], gate='G4', seed=0):
    """G4 over paired (low, high) runs: per objective metric the gap low - high."""
    usable = [(k, a, b) for k, a, b in pairs if gate_ok(a, gate) and gate_ok(b, gate)]
    out = {'n_pairs': len(pairs), 'n_usable': len(usable),
           'excluded': dict(Counter(_why_excluded(a, b, gate) for _, a, b in pairs
                                    if not (gate_ok(a, gate) and gate_ok(b, gate)))),
           'metrics': {}, 'failure_modes': failure_confusion(usable)}
    for obj in objectives:
        m = obj.metric
        d = [(a.get(m), b.get(m)) for _, a, b in usable]
        d = [(x, y) for x, y in d if x is not None and y is not None]
        if not d:
            out['metrics'][m] = {'n': 0, 'verdict': 'unscorable on one tier'}
            continue
        gaps = [x - y for x, y in d]
        bias, ci = stats.bootstrap_ci(gaps, lambda g: sum(g) / len(g), seed=seed)
        sd = math.sqrt(sum((g - bias) ** 2 for g in gaps) / (len(gaps) - 1)) if len(gaps) > 1 else 0.0
        row = {'n': len(gaps), 'low_mean': sum(x for x, _ in d) / len(d),
               'high_mean': sum(y for _, y in d) / len(d), 'bias': bias, 'bias_ci95': list(ci),
               'sd': sd, 'max_abs': max(abs(g) for g in gaps), 'band': obj.band}
        pos = [(x, y) for x, y in d if y]
        if pos:
            ratios = sorted(x / y for x, y in pos)
            row['median_ratio'] = ratios[len(ratios) // 2]
        if obj.band is not None:
            inside = sum(1 for g in gaps if abs(g) <= obj.band)
            lo, hi = stats.wilson(inside, len(gaps))
            row.update(band_coverage=inside / len(gaps), band_coverage_ci95=[lo, hi],
                       verdict='PASS' if ci[0] >= -obj.band and ci[1] <= obj.band else
                       ('FAIL' if ci[0] > obj.band or ci[1] < -obj.band else 'INCONCLUSIVE'))
        else:
            row['verdict'] = 'no band declared'
        out['metrics'][m] = row
    return out


def _why_excluded(a: Scored, b: Scored, gate):
    for s in (a, b):
        if not s.rec.validity.valid:
            flags = ','.join(sorted(s.rec.validity.flags)) or 'invalid'
            return f'{s.rec.tier}:{flags}'
        if gate in s.rec.validity.excluded_from:
            return f'{s.rec.tier}:excluded_from_{gate}'
    return 'other'


def failure_confusion(pairs):
    """Outcome category low x high, and the agreement rate."""
    table = Counter((a.outcome, b.outcome) for _, a, b in pairs)
    n = sum(table.values())
    agree = sum(v for (x, y), v in table.items() if x == y)
    return {'table': {f'{x} | {y}': v for (x, y), v in sorted(table.items())},
            'agreement': agree / n if n else None, 'n': n}


def morphology_ok(metric_obj, a: RunRecord, b: RunRecord):
    """Cross-morphology comparisons are refused for metrics that are not morphology invariant."""
    if getattr(metric_obj, 'morphology_invariant', True):
        return True
    return (a.header.robot.morphology or {}) == (b.header.robot.morphology or {})
