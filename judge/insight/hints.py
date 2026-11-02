"""Hints: the pack's rules turn insights into the next experiment. Generic rules live here; packs
add domain ones (``Pack.hints``)."""

from __future__ import annotations

from collections import defaultdict

from judge.insight import Insight
from judge.pack import HintRule


def _by_kind(insights):
    d = defaultdict(list)
    for i in insights:
        d[i.kind].append(i)
    return d


def _plant(by, ctx):
    splits = [i.data for i in by.get('divergence.split', [])]
    plant = [s for s in splits if s.get('dominant') == 'plant']
    if splits and len(plant) * 2 >= len(splits):
        motions = defaultdict(int)
        for s in plant:
            motions[s.get('worst_motion')] += 1
        m = max(motions, key=motions.get)
        return (f'plant error dominates in {len(plant)}/{len(splits)} pairs, worst in {m} segments: '
                f'excite {m} manoeuvres on the higher tier and refit the plant card before tuning control')
    return None


def _decision(by, ctx):
    dec = [i for i in by.get('events.first_difference', []) if i.data.get('kind') == 'decision']
    if dec:
        return (f'{len(dec)} pairs diverge by decision with the robots close together: check the low '
                f'tier\'s perception/world-model inputs (what each tier saw) before dynamics')
    return None


def _invalid(by, ctx):
    bad = by.get('integrity.batch_invalid', [])
    if bad:
        return 'fix source validity first: gates exclude invalid runs, so G3/G4 below rest on the remainder'
    return None


def _underpowered(by, ctx):
    n = ctx.get('n_units')
    if n is not None and n < 8:
        return f'only {n} paired units: treat verdicts as screening; G3 needs >= 8 variants x 5 seeds'
    return None


GENERIC = (HintRule('invalid-first', _invalid), HintRule('plant-dominant', _plant),
           HintRule('decision-divergence', _decision), HintRule('underpowered', _underpowered))


def apply(insights, packs=(), ctx=None):
    by, ctx = _by_kind(insights), ctx or {}
    out = []
    for rule in tuple(GENERIC) + tuple(r for p in packs for r in p.hints):
        text = rule.when(by, ctx)
        if text:
            out.append(Insight('hint.' + rule.id, 'info', text, basis='rule', scope={}))
    return out
