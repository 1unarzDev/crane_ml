"""Event-timeline diff: global alignment (Needleman-Wunsch) of two runs' decision events.

Events match when their labels are equal and their relative times are within ``tol_s``; time
slack costs a little. The first non-match in the alignment is the first differing decision; the
pose residual at that moment separates a *decision divergence* (small residual: the robots were
in the same place and chose differently, so perception/world model/planner) from a decision that
follows dynamics (large residual: they already were elsewhere).
"""

from __future__ import annotations

import numpy as np

from judge.insight import Insight
from judge.metric import Span
from judge.record import RunRecord


def label(e, keys=('kind', 'why', 'stage', 'reason_class')):
    p = e.payload or {}
    extra = next((str(p[k]) for k in keys if p.get(k)), '')
    return f'{e.kind}:{extra}' if extra else e.kind


def timeline(rec: RunRecord, kinds, labeler=label):
    t0 = rec.header.window[0]
    return [(float(e.t - t0), labeler(e), e) for e in rec.events if e.kind in kinds]


def align(a, b, tol_s=5.0, gap=-1.0, mismatch=-1.0):
    """[(i | None, j | None, 'match' | 'mismatch' | 'gap')] over two [(t, label, ev)] lists."""
    n, m = len(a), len(b)
    S = np.zeros((n + 1, m + 1))
    S[:, 0] = gap * np.arange(n + 1)
    S[0, :] = gap * np.arange(m + 1)

    def sc(i, j):
        (ta, la, _), (tb, lb, _) = a[i], b[j]
        if la == lb:
            return 2.0 - min(1.0, abs(ta - tb) / tol_s) if abs(ta - tb) <= 3 * tol_s else 0.5
        return mismatch
    for i in range(1, n + 1):
        for j in range(1, m + 1):
            S[i, j] = max(S[i - 1, j - 1] + sc(i - 1, j - 1), S[i - 1, j] + gap, S[i, j - 1] + gap)
    out, i, j = [], n, m
    while i > 0 or j > 0:
        if i > 0 and j > 0 and np.isclose(S[i, j], S[i - 1, j - 1] + sc(i - 1, j - 1)):
            ok = a[i - 1][1] == b[j - 1][1] and abs(a[i - 1][0] - b[j - 1][0]) <= tol_s
            out.append((i - 1, j - 1, 'match' if ok else 'mismatch'))
            i, j = i - 1, j - 1
        elif i > 0 and np.isclose(S[i, j], S[i - 1, j] + gap):
            out.append((i - 1, None, 'gap'))
            i -= 1
        else:
            out.append((None, j - 1, 'gap'))
            j -= 1
    return out[::-1]


def first_difference(low: RunRecord, high: RunRecord, kinds, pose='pose.se2', tol_s=5.0,
                     near_m=1.0, labeler=label):
    """Insight for the first differing decision (or None when the timelines agree)."""
    a, b = timeline(low, kinds, labeler), timeline(high, kinds, labeler)
    if not a and not b:
        return None, {'n_low': 0, 'n_high': 0}
    al = align(a, b, tol_s)
    matches = sum(1 for *_, s in al if s == 'match')
    info = {'n_low': len(a), 'n_high': len(b), 'matched': matches,
            'agreement': matches / max(len(a), len(b))}
    first = next((x for x in al if x[2] != 'match'), None)
    if first is None:
        return None, info
    i, j, _ = first
    t_rel = a[i][0] if i is not None else b[j][0]
    la = a[i][1] if i is not None else '(nothing)'
    lb = b[j][1] if j is not None else '(nothing)'
    resid = None
    try:
        lc, hc = low.channel(pose), high.channel(pose)
        pa = np.array([np.interp(t_rel + low.header.window[0], lc.t, lc.v[:, k]) for k in range(2)])
        pb = np.array([np.interp(t_rel + high.header.window[0], hc.t, hc.v[:, k]) for k in range(2)])
        resid = float(np.linalg.norm(pa - pb))
    except LookupError:
        pass
    decision = resid is not None and resid <= near_m
    info.update(first_t=t_rel, low=la, high=lb, residual_m=resid,
                kind='decision' if decision else 'downstream')
    claim = (f'first differing decision at t+{t_rel:.1f} s: {low.tier} "{la}" vs {high.tier} "{lb}"'
             + (f' with the robots {resid:.2f} m apart' if resid is not None else '')
             + (' -> decision divergence (perception/world model/planner), not dynamics' if decision
                else ' -> follows an earlier state divergence' if resid is not None else ''))
    ev = (Span(high.run_id, high.header.window[0] + t_rel, high.header.window[0] + t_rel + 5, (pose,), 'decision'),)
    return Insight('events.first_difference', 'warn' if decision else 'info', claim, evidence=ev,
                   scope={'pair': [low.run_id, high.run_id], 'tiers': [low.tier, high.tier]},
                   data=info, basis='descriptive'), info
