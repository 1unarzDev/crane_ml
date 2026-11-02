"""Pairing runs across variants or tiers. The design label travels with every comparison, so a
real-robot comparison never claims exact pairing.

  exact   same (pair_key, truth_sha256, seed, condition): the fastsim.pairs identity
  block   same scenario family and condition, seeds not matched (Unity reseeded, sessions)
  loose   same task only; conditions recorded but uncontrolled (the real robot)
"""

from __future__ import annotations

import json
from typing import Iterable, Sequence

from judge.record import RunRecord

DESIGNS = ('exact', 'block', 'loose')


def _cond(c):
    return json.dumps(c or {}, sort_keys=True, default=str)


def key(rec: RunRecord, design='exact'):
    s = rec.header.scenario
    if design == 'exact':
        return (s.pair_key, s.truth_sha256, s.seed, _cond(s.condition))
    if design == 'block':
        return (s.family or s.pair_key, _cond(s.condition))
    if design == 'loose':
        return (rec.header.task_id,)
    raise ValueError(design)


def pair(a: Sequence[RunRecord], b: Sequence[RunRecord], design='exact'):
    """[(key, a_run, b_run)]: exact pairs one to one in order of appearance; block and loose pair
    every run of a key with the same-index run of the other side (unused runs reported by
    ``unpaired``). A truth mismatch under one pair_key is an error, as in fastsim.pairs."""
    if design == 'exact':
        for ra in a:
            for rb in b:
                sa, sb = ra.header.scenario, rb.header.scenario
                if sa.pair_key == sb.pair_key and sa.truth_sha256 != sb.truth_sha256:
                    raise ValueError(f'pair_key {sa.pair_key}: truth_sha256 differs, not a pair')
    ia, ib = {}, {}
    for r in a:
        ia.setdefault(key(r, design), []).append(r)
    for r in b:
        ib.setdefault(key(r, design), []).append(r)
    out = []
    for k in ia:
        for x, y in zip(ia[k], ib.get(k, [])):
            out.append((k, x, y))
    return out


def unpaired(a: Iterable[RunRecord], b: Iterable[RunRecord], pairs):
    used = {id(x) for _, x, _ in pairs} | {id(y) for _, _, y in pairs}
    return [r.run_id for r in list(a) + list(b) if id(r) not in used]
