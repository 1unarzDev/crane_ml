"""Seeded-fault tests: every insight generator must find a planted divergence, localise it and
stay quiet on a clean control (design-judge 6.3)."""

import numpy as np
import pytest

from judge import render
from judge.compare import Scored, tier_agreement, tier_gap
from judge.insight import clusters, summarise
from judge.packs import generic_motion, toy_quad as tq
from judge.packs.toy_quad.sim import variant
from judge.record import Validity
from judge.run import judge_runs, judge_tiers, judge_variants

PACKS = [tq.PACK, generic_motion.make('pose.xz')]
SEEDS = range(5)


def _tiers(high_params, high_tier='high'):
    pairs = [(s, tq.simulate(tq.PARAMS_LOW, s, 'low'), tq.simulate(high_params, s, high_tier)) for s in SEEDS]
    return judge_tiers(pairs, tq.TASK, PACKS, replayer=tq.QuadReplayer(), decision_kinds=('gate.target',),
                       segmenter=tq.segments, pose='pose.xz')


def _kinds(rep):
    return [i['kind'] for i in rep['insights']]


def test_plant_fault_is_attributed_to_the_plant():
    rep = _tiers(tq.PARAMS_HIGH)
    origin = next(i for i in rep['insights'] if i['kind'] == 'tiers.origin')
    assert 'plant' in origin['claim']
    assert rep['split']['dominant'].get('plant', 0) >= 4
    assert 'tiers.decision' not in _kinds(rep)


def test_decision_fault_is_a_decision_divergence_at_the_skip():
    rep = _tiers(variant(tq.PARAMS_LOW, skip_gate=2), 'skip')
    dec = next(i for i in rep['insights'] if i['kind'] == 'tiers.decision')
    assert dec['data']['first_pairs'][0][0] == '"gate.target:g2" vs "(nothing)"'
    firsts = [p['events']['first_t'] for p in rep['pairs']]
    assert all(6.0 < t < 8.0 for t in firsts)                   # the g1 -> g2 switch
    assert all(p['events']['residual_m'] < 0.05 for p in rep['pairs'])
    assert not rep['split']['dominant'].get('plant')


def test_clean_control_is_quiet():
    rep = _tiers(tq.PARAMS_LOW, 'low2')
    assert not [i for i in rep['insights'] if i['severity'] != 'info']
    assert rep['g4']['failure_modes']['agreement'] == 1.0


def test_invalid_runs_are_counted_not_dropped():
    pairs = []
    for s in SEEDS:
        hi = tq.simulate(tq.PARAMS_HIGH, s, 'high')
        if s < 2:
            hi = hi.with_validity(valid=False, flags={'invalidWaterSearches': 21}, excluded_from=frozenset({'G4'}))
        pairs.append((s, tq.simulate(tq.PARAMS_LOW, s, 'low'), hi))
    rep = judge_tiers(pairs, tq.TASK, PACKS, pose='pose.xz')
    assert rep['g4']['n_usable'] == 3 and rep['g4']['excluded'] == {'high:invalidWaterSearches': 2}
    bad = next(i for i in rep['insights'] if i['kind'] == 'integrity.batch_invalid')
    assert '2/5 high runs invalid' in bad['claim']
    assert 'excl 2 high:invalidWaterSearches' in render.oneline(rep)


def test_first_cause_walks_the_dependency_graph():
    rep = judge_runs([tq.simulate(variant(tq.PARAMS_LOW, skip_gate=2), s, 'low') for s in SEEDS], tq.TASK, PACKS,
                     pose='pose.xz')
    causes = [i for i in rep['insights'] if i['kind'] == 'cause.first']
    assert causes and all(c['data']['node'] == 'planner' for c in causes)
    assert all(c['basis'] == 'model-relative:quad-v1' for c in causes)


def test_clusters_need_stability():
    items = [(f'a{k}', {'outcome:crash', 'phase:dock'}) for k in range(6)] + \
            [(f'b{k}', {'outcome:missed', 'phase:gate'}) for k in range(5)]
    ins, rows = clusters.cluster_failures(items)
    assert len(ins) == 2 and {r['size'] for r in rows} == {6, 5}
    rng = np.random.default_rng(0)
    noise = [(f'n{k}', {f't{x}' for x in rng.choice(40, 3, replace=False)}) for k in range(8)]
    ins, _ = clusters.cluster_failures(noise)
    assert not ins


def test_g3_regret_mmrv_topk():
    low = {'a': [1.0], 'b': [2.0], 'c': [3.0]}
    high = {'a': [1.0], 'b': [3.0], 'c': [2.0]}
    g3 = tier_agreement(low, high)
    assert g3['low_pick'] == 'c' and g3['high_best'] == 'b'
    assert g3['regret'] == pytest.approx(1.0) and g3['topk_recall'] == 0.0
    assert g3['mmrv'] == pytest.approx(2 / 3)
    same = tier_agreement(low, low)
    assert same['regret'] == 0 and same['mmrv'] == 0 and same['kendall_tau'] == 1.0


def test_variants_report_and_oneline():
    pairs = [(s, tq.simulate(tq.PARAMS_LOW, s, 'base'), tq.simulate(variant(tq.PARAMS_LOW, skip_gate=2), s, 'cand'))
             for s in SEEDS]
    rep = judge_variants(pairs, tq.TASK, PACKS)
    comp = {c['metric']: c for c in rep['comparisons']}
    assert comp['quad.completion']['estimate'] == -1.0
    assert comp['quad.completion']['verdict'] == 'inferior'
    line = render.oneline(rep)
    assert line.startswith('JUDGE ') and len(line) <= 160


def test_summary_citations_are_validated():
    rep = _tiers(variant(tq.PARAMS_LOW, skip_gate=2), 'skip')
    dec = next(i for i in rep['insights'] if i['kind'] == 'tiers.decision')
    ok = {'sentence': 'All 5 pairs part on a skipped gate.', 'cites': [dec['id']]}
    bad_num = {'sentence': 'All 9 pairs part.', 'cites': [dec['id']]}
    no_cite = {'sentence': 'It is fine.', 'cites': []}
    kept, dropped = summarise.validate([ok, bad_num, no_cite], rep['insights'])
    assert kept == [ok] and len(dropped) == 2


def test_modular_morphology_refusal():
    from dataclasses import replace
    from judge.compare import morphology_ok
    from judge.metric import metric

    @metric('module.joint_effort', morphology_invariant=False)
    def effort(rec, task):
        return 1.0
    a = tq.simulate(tq.PARAMS_LOW, 0, 'low')
    b = replace(a, header=replace(a.header, robot=replace(a.header.robot, morphology={'modules': ['m1', 'm2']})))
    assert not morphology_ok(effort, a, b) and morphology_ok(effort, a, a)
    assert morphology_ok(generic_motion.PACK.metrics['motion.duration'], a, b)
