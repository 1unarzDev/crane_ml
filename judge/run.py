"""The judge's three questions as functions returning ``crane.judge.report/1`` dicts.

  judge_runs     one tier, one batch: metrics, outcomes, integrity, failure clusters, causes
  judge_tiers    paired runs of one scenario on two tiers: G4 gaps, failure-mode confusion,
                 per-pair divergence and event diffs, the split summary, hints (G3 when a roster
                 of variants was run on both tiers)
  judge_variants paired variants on one tier: per-objective comparisons with Holm, NI verdicts
"""

from __future__ import annotations

from collections import Counter
import time

from judge import SCHEMA_REPORT
from judge.compare import Scored, compare_paired, holm_adjust, tier_agreement, tier_gap
from judge.insight import clusters, divergence, events, hints, integrity, rank
from judge.insight.cause import first_cause
from judge.metric import Status, verdict_of
from judge.pack import evaluate, outcome_of
from judge.record import header_dict


def score(records, task, packs):
    out = []
    for r in records:
        vals = evaluate(r, task, packs)
        out.append(Scored(r, vals, outcome_of(r, vals, packs)))
    return out


def signature(s: Scored, extra=()):
    toks = {f'outcome:{s.outcome}'}
    viol = sorted(((v.extra or {}).get('first_violation_t') or 0, n) for n, v in s.values.items()
                  if verdict_of(v) == 'violated')
    if viol:
        toks.add(f'first_violation:{viol[0][1]}')
    for k, v in (s.rec.header.scenario.condition or {}).items():
        if isinstance(v, (str, int, bool)) and not isinstance(v, float):
            toks.add(f'cond:{k}={v}')
    return toks | set(extra)


def _run_row(s: Scored):
    return {'run_id': s.rec.run_id, 'tier': s.rec.tier, 'source': s.rec.header.source,
            'valid': s.rec.validity.valid, 'outcome': s.outcome,
            'values': {n: v.as_dict() for n, v in sorted(s.values.items())}}


def _base(kind, task, packs, **kw):
    return {'schema': SCHEMA_REPORT, 'kind': kind, 'created': time.strftime('%Y-%m-%dT%H:%M:%SZ', time.gmtime()),
            'task': task.id if task else None, 'packs': [p.name for p in packs], **kw}


def judge_runs(records, task, packs, success=('success',), pose='pose.se2'):
    scored = score(records, task, packs)
    ins = integrity.batch_validity(records)
    for s in scored:
        ins += integrity.record_checks(s.rec, s.values, required=(pose,))
    fails = [s for s in scored if s.outcome not in success]
    cl, rows = clusters.cluster_failures([(s.rec.run_id, signature(s)) for s in fails])
    ins += cl
    for s in fails:
        for p in packs:
            c = first_cause(s.rec, s.values, p)
            if c:
                ins.append(c)
    ins += hints.apply(ins, packs, {'n_units': len(scored)})
    outcomes = Counter(s.outcome for s in scored)
    return _base('runs', task, packs, n=len(scored), outcomes=dict(outcomes),
                 runs=[_run_row(s) for s in scored], clusters=rows,
                 insights=[i.as_dict() for i in rank(ins)],
                 headers=[header_dict(s.rec) for s in scored])


def judge_tiers(pairs, task, packs, replayer=None, decision_kinds=(), segmenter=None, pose='pose.se2',
                tol=0.5, roster_low=None, roster_high=None, anchor=None):
    """``pairs`` [(key, low RunRecord, high RunRecord)]. Pairs that share a high-tier run are one
    unit for the G4 bootstrap (one Unity twin paired with several fast variants is one twin)."""
    if anchor is None:
        anchor = next((p.anchor_event for p in packs if p.anchor_event), '')
    scored = [(k, *score([a, b], task, packs)) for k, a, b in pairs]
    objectives = list(task.ordered()) if task else []
    gap = tier_gap(scored, objectives, unit=lambda s: s.rec.run_id)
    ins = integrity.batch_validity([r for _, a, b in pairs for r in (a, b)])
    per_pair = []
    for (k, a, b), (_, sa, sb) in zip(pairs, scored):
        row = {'key': str(k), 'low': a.run_id, 'high': b.run_id, 'valid': [a.validity.valid, b.validity.valid],
               'outcome': [sa.outcome, sb.outcome]}
        pc = integrity.pair_checks(a, b)
        ins += pc
        row['code_mismatch'] = [i.data for i in pc if i.kind == 'integrity.code_mismatch']
        try:
            segs = segmenter(b) if segmenter else None
            pi, summ = divergence.pair_insights(a, b, replayer, segs, pose, tol, anchor=anchor)
            ins += pi
            row['divergence'] = summ
        except LookupError as e:
            row['divergence'] = {'unavailable': str(e)}
        if decision_kinds:
            ei, info = events.first_difference(a, b, decision_kinds, pose, anchor=anchor)
            row['events'] = info
            if ei:
                ins.append(ei)
        per_pair.append(row)
    ins = _rollups(per_pair, a_tier=pairs[0][1].tier if pairs else 'low',
                   b_tier=pairs[0][2].tier if pairs else 'high') + ins
    g3 = tier_agreement(roster_low, roster_high) if roster_low and roster_high else None
    ins += hints.apply(ins, packs, {'n_units': gap['n_usable']})
    return _base('tiers', task, packs, n_pairs=len(pairs), g4=gap, g3=g3, pairs=per_pair,
                 insights=[i.as_dict() for i in rank(ins)],
                 split=_split_rollup(per_pair), decisions=_decision_rollup(per_pair))


def _rollups(per_pair, a_tier, b_tier):
    """Batch-level insights over all pairs (the per-pair ones stay as evidence)."""
    from judge.insight import Insight
    out, n = [], len(per_pair)
    sp = [(p.get('divergence') or {}).get('split') or {} for p in per_pair]
    dom = Counter(s.get('dominant') for s in sp if s.get('dominant'))
    if dom:
        top, k = dom.most_common(1)[0]
        ex = [s['explained'] for s in sp if s.get('dominant') == top and 'explained' in s]
        out.append(Insight('tiers.origin', 'warn' if top != 'none' else 'info',
                           f'{a_tier} vs {b_tier}: gap origin {top} in {k}/{n} pairs'
                           + (f' (dynamics explain median {sorted(ex)[len(ex) // 2]:.0%})' if ex else '')
                           + (f'; others: {", ".join(f"{d} {c}" for d, c in dom.most_common()[1:])}' if len(dom) > 1 else ''),
                           scope={'tiers': [a_tier, b_tier], 'pairs': n}, data=dict(dom), basis='model-relative:replay'))
    twins = {p['high'] for p in per_pair}
    if len(twins) < n:
        out.append(Insight('tiers.shared_twins', 'warn',
                           f'{n} pairs but only {len(twins)} distinct {b_tier} runs: the effective sample is '
                           f'{len(twins)} (G4 bootstraps by {b_tier} run)',
                           scope={'tiers': [a_tier, b_tier]}, data={'pairs': n, 'distinct_high': len(twins)}))
    ev = [p.get('events') or {} for p in per_pair]
    dec = [e for e in ev if e.get('kind') == 'decision']
    if dec:
        firsts = Counter(f'"{e["low"]}" vs "{e["high"]}"' for e in dec)
        ts = sorted(e['first_t'] for e in dec)
        f, c = firsts.most_common(1)[0]
        out.append(Insight('tiers.decision', 'warn',
                           f'{len(dec)}/{n} pairs first differ by a decision with the robots close together; '
                           f'most often {f} ({c}x), median t+{ts[len(ts) // 2]:.1f} s',
                           scope={'tiers': [a_tier, b_tier], 'pairs': n}, data={'first_pairs': firsts.most_common(5)}))
    agree = [e['agreement'] for e in ev if e.get('agreement') is not None]
    if agree:
        out.append(Insight('tiers.timeline_agreement', 'info',
                           f'decision timelines agree {sum(agree) / len(agree):.0%} on average over {len(agree)} pairs',
                           scope={'tiers': [a_tier, b_tier]}, data={'mean': sum(agree) / len(agree)}))
    cm = [p for p in per_pair if p.get('code_mismatch')]
    if cm:
        repos = Counter(k for p in cm for d in p['code_mismatch'] for k in d)
        out.append(Insight('tiers.code_mismatch', 'warn',
                           f'{len(cm)}/{n} pairs ran different code on the two tiers '
                           f'({", ".join(f"{k} {v}x" for k, v in repos.most_common())}): their gap mixes tier and code',
                           scope={'tiers': [a_tier, b_tier]}, data=dict(repos)))
    pace = [(p.get('divergence') or {}) for p in per_pair]
    pr = [(d['time_rms'], d['path_rms']) for d in pace if 'time_rms' in d]
    if pr:
        t_m = sorted(x for x, _ in pr)[len(pr) // 2]
        p_m = sorted(y for _, y in pr)[len(pr) // 2]
        out.append(Insight('tiers.residuals', 'info',
                           f'median residual {t_m:.2f} m on time vs {p_m:.2f} m on path'
                           + (' -> mostly pace (same route, different speed)' if t_m > 2.5 * max(p_m, 1e-6) else ''),
                           scope={'tiers': [a_tier, b_tier]}, data={'time_rms_median': t_m, 'path_rms_median': p_m}))
    return out


def _split_rollup(per_pair):
    dom = Counter()
    by_motion = {}
    for p in per_pair:
        sp = (p.get('divergence') or {}).get('split') or {}
        if sp.get('dominant') and sp['dominant'] != 'none':
            dom[sp['dominant']] += 1
        for m, v in (sp.get('free_by_motion') or {}).items():
            by_motion.setdefault(m, []).append(v)
    return {'dominant': dict(dom), 'free_by_motion': {m: sum(v) / len(v) for m, v in by_motion.items()}}


def _decision_rollup(per_pair):
    kinds = Counter((p.get('events') or {}).get('kind') for p in per_pair if (p.get('events') or {}).get('kind'))
    firsts = Counter(f"{(p.get('events') or {}).get('low')} | {(p.get('events') or {}).get('high')}"
                     for p in per_pair if (p.get('events') or {}).get('kind'))
    return {'kinds': dict(kinds), 'first_pairs': firsts.most_common(5)}


def judge_variants(pairs, task, packs, design='exact', alpha=0.05):
    """``pairs`` [(key, baseline RunRecord, candidate RunRecord)]."""
    scored = [(k, *score([a, b], task, packs)) for k, a, b in pairs]
    comps = holm_adjust([compare_paired(scored, o, design, alpha) for o in task.ordered()])
    ins = integrity.batch_validity([r for _, a, b in pairs for r in (a, b)])
    ins += hints.apply(ins, packs, {'n_units': len(pairs)})
    return _base('variants', task, packs, design=design, n_pairs=len(pairs),
                 comparisons=[c.as_dict() for c in comps], insights=[i.as_dict() for i in rank(ins)])
