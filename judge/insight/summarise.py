"""Citation validator for any LLM-written summary of a report (the LLM itself is optional, last,
and never blocks a verdict).

Input: [{sentence, cites: [insight_id]}]. A sentence survives only when every cited id exists in
the report and every number in the sentence appears in a cited insight (claim or data)."""

from __future__ import annotations

import json
import re

NUM = re.compile(r'-?\d+(?:\.\d+)?')


def _numbers(text):
    return {float(x) for x in NUM.findall(text)}


def validate(sentences, insights):
    by_id = {i['id'] if isinstance(i, dict) else i.id: (i if isinstance(i, dict) else i.as_dict())
             for i in insights}
    kept, dropped = [], []
    for s in sentences:
        cites = s.get('cites') or []
        if not cites or any(c not in by_id for c in cites):
            dropped.append({**s, 'why': 'missing or unknown citation'})
            continue
        pool = set()
        for c in cites:
            pool |= _numbers(json.dumps(by_id[c], default=str))
        stray = [x for x in _numbers(s.get('sentence', '')) if not any(abs(x - y) <= 0.051 * max(1, abs(y)) for y in pool)]
        if stray:
            dropped.append({**s, 'why': f'numbers not in cited insights: {stray}'})
        else:
            kept.append(s)
    return kept, dropped
