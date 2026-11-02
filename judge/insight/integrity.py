"""Telemetry integrity before any explanation: is this record fit to be compared at all?"""

from __future__ import annotations

from collections import Counter
import json

import numpy as np

from judge.insight import Insight
from judge.metric import Status
from judge.record import RunRecord

GAP_FACTOR = 5.0          # a hole longer than 5 median sample periods is a gap
CLOCK_RESIDUAL_WARN = 0.25  # s


def record_checks(rec: RunRecord, values=None, required=('pose.se2',)):
    out = []
    scope = {'run': rec.run_id, 'tier': rec.tier}
    if not rec.validity.valid:
        flags = ', '.join(f'{k}={v}' for k, v in sorted(rec.validity.flags.items())) or 'no flags'
        out.append(Insight('integrity.invalid', 'fail',
                           f'{rec.tier} run {rec.run_id} is invalid ({flags}); excluded from '
                           f'{", ".join(sorted(rec.validity.excluded_from)) or "all gates"}',
                           scope=scope, data={'flags': dict(rec.validity.flags)}))
    for name in required:
        if not rec.has(name):
            out.append(Insight('integrity.missing_channel', 'warn',
                               f'{rec.tier} run {rec.run_id} has no {name}', scope=scope,
                               data={'channel': name}))
    seen = []
    for name, ch in rec.channels.items():
        if len(ch) < 3 or any(len(ch.t) == len(t) and np.array_equal(ch.t, t) for t in seen):
            continue                                   # derived channels share their source's clock
        seen.append(ch.t)
        dt = np.diff(ch.t)
        if (dt < 0).any():
            out.append(Insight('integrity.time_reversal', 'fail',
                               f'{name} time runs backwards {int((dt < 0).sum())} times in {rec.run_id}',
                               scope={**scope, 'channel': name}))
        med = float(np.median(dt[dt > 0])) if (dt > 0).any() else 0.0
        mean = float((ch.t[-1] - ch.t[0]) / (len(ch.t) - 1))
        if 0 < med < 0.25 * mean:
            out.append(Insight('integrity.bursty', 'info',
                               f'{name} in {rec.run_id} arrives in bursts: median spacing {med * 1000:.0f} ms '
                               f'vs mean {mean * 1000:.0f} ms (bunched timestamps; rates from raw differences are inflated)',
                               scope={**scope, 'channel': name}, data={'median_s': med, 'mean_s': mean}))
        med = max(med, mean)
        if med > 0:
            holes = np.nonzero(dt > GAP_FACTOR * med)[0]
            if len(holes):
                worst = int(holes[np.argmax(dt[holes])])
                out.append(Insight('integrity.gap', 'warn',
                                   f'{name} in {rec.run_id} has {len(holes)} gaps > {GAP_FACTOR:g}x its '
                                   f'{med * 1000:.0f} ms period; longest {dt[worst]:.2f} s at t={ch.t[worst]:.1f}',
                                   scope={**scope, 'channel': name},
                                   data={'gaps': len(holes), 'longest_s': float(dt[worst]),
                                         'period_s': med}))
    c = rec.header.clock
    if c.base == 'wall' and c.residual_s is None and rec.tier == 'real':
        out.append(Insight('integrity.clock_unaligned', 'warn',
                           f'{rec.run_id}: record clock alignment residual not measured', scope=scope))
    elif c.residual_s is not None and c.residual_s > CLOCK_RESIDUAL_WARN:
        out.append(Insight('integrity.clock_residual', 'warn',
                           f'{rec.run_id}: clock alignment residual {c.residual_s:.2f} s', scope=scope,
                           data={'residual_s': c.residual_s}))
    if values:
        uns = sorted(n for n, v in values.items() if v.status == Status.UNSCORABLE)
        if uns:
            why = Counter(values[n].why.split(':')[0][:60] for n in uns)
            out.append(Insight('integrity.unscorable', 'info',
                               f'{len(uns)}/{len(values)} metrics unscorable on {rec.tier} '
                               f'({"; ".join(f"{k} x{v}" for k, v in why.most_common(3))})',
                               scope=scope, data={'metrics': uns}))
    return out


def pair_checks(a: RunRecord, b: RunRecord):
    """Provenance or condition mismatch inside a pair (the pair is then not what it claims)."""
    out = []
    sa, sb = a.header.scenario, b.header.scenario
    scope = {'pair': [a.run_id, b.run_id], 'tiers': [a.tier, b.tier]}
    if json.dumps(sa.condition, sort_keys=True, default=str) != json.dumps(sb.condition, sort_keys=True, default=str):
        out.append(Insight('integrity.condition_mismatch', 'warn',
                           f'pair {a.run_id}/{b.run_id}: conditions differ', scope=scope,
                           data={'a': dict(sa.condition), 'b': dict(sb.condition)}))
    ga, gb = (a.header.provenance or {}).get('code'), (b.header.provenance or {}).get('code')
    if ga and gb:
        diff = sorted(k for k in set(ga) & set(gb) if ga[k] and gb[k] and
                      not (str(ga[k]).startswith(str(gb[k])) or str(gb[k]).startswith(str(ga[k]))))
        if diff:
            out.append(Insight('integrity.code_mismatch', 'warn',
                               f'pair {a.run_id}/{b.run_id} ran different code for {", ".join(diff)}',
                               scope=scope, data={k: [ga[k], gb[k]] for k in diff}))
    return out


def batch_validity(records):
    """One insight for the whole batch: how many runs are invalid per tier and why."""
    by = Counter()
    flags = Counter()
    n = Counter()
    for r in records:
        n[r.tier] += 1
        if not r.validity.valid:
            by[r.tier] += 1
            for k in r.validity.flags:
                flags[(r.tier, k)] += 1
    out = []
    for tier, k in by.items():
        top = ', '.join(f'{f} x{c}' for (t, f), c in flags.most_common() if t == tier)
        out.append(Insight('integrity.batch_invalid', 'fail' if k == n[tier] else 'warn',
                           f'{k}/{n[tier]} {tier} runs invalid ({top}); counted, excluded from gates',
                           scope={'tier': tier}, data={'invalid': k, 'n': n[tier]}))
    return out
