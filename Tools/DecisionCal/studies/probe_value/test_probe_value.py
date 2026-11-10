"""probe_value: cost accounting, passive timing identifiability, design/evaluation seed separation, CRN.

    cd /home/lunarz/worktrees/decisioncal && nice -n 19 python3 -I -W ignore -m pytest \\
        Tools/DecisionCal/studies/probe_value/test_probe_value.py -q -p no:cacheprovider
"""
import json
import math
import sys
from pathlib import Path

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))
sys.path.insert(0, str(HERE.parents[1]))

import numpy as np  # noqa: E402
import pytest  # noqa: E402

import probe_value as pv  # noqa: E402


def _row(stratum='timing', seed=pv.SEED, i=0):
    return pv.hidden.sample_hidden(2, seed, stratum).take([i])


def _fake_design():
    """A design.json-shaped dict built from the candidate lists (no flights)."""
    ch = {}
    for b in pv.BUDGETS:
        c = pv.candidates(b)
        stage = [k for k, d in c.items() if pv.STAGE1 in d]
        ch[f'task@{b}'] = {'label': 'pulse', 'design': [list(k) for k in c['pulse']]}
        ch[f'dopt@{b}'] = {'label': 'reversal', 'design': [list(k) for k in c['reversal']]}
        ch[f'adaptive@{b}'] = {'branches': {bn: {'label': stage[j], 'design': [list(k) for k in c[stage[j]]]}
                                            for j, bn in enumerate(('lo', 'hi'))}}
    return {'chosen': ch, 'stopwatch': {'pad': 0.05, 'nan_level': 0.47}, 'design_seed': pv.DESIGN_SEED}


# ------------------------------------------------------------------------------------------- cost accounting
def test_every_design_fits_its_budget_and_counts_settle_time():
    for b in pv.BUDGETS:
        designs = dict(pv.candidates(b))
        designs.update(pv.fixed_designs(b))
        for lab, d in designs.items():
            assert pv.design_seconds(d) <= b + 1e-9, (b, lab)
            assert len(set(d)) == len(d), f'{lab}: a segment key repeats (would reuse one realization twice)'
    for key in [('ystep0.3', 0), ('srev0.8', 1), ('coast0.9', 0), ('ytrain0.7', 0), ('turn0.4', 1)]:
        inf = pv.seg_info(key)
        assert inf['dur'] <= 30.0 and inf['dur'] >= pv.BASE_S + inf['pulse_s']        # baseline counted


def test_simulated_evidence_duration_matches_accounting():
    """Every arm's evidence (as simulated, not as declared) spans at most its budget."""
    row = _row()
    design = _fake_design()
    arms = pv.eval_arms(design)
    keys = set()
    for a, (rn, d, fa) in arms.items():
        if rn in ('short', 'passive') and d is not None:
            keys |= set(d)
    for b in pv.BUDGETS:
        for br in design['chosen'][f'adaptive@{b}']['branches'].values():
            keys |= set(map(tuple, br['design']))
    segs = pv.simulate(row, sorted(keys), 11, 'cost')
    for k, s in segs.items():
        span = (s.t[-1] - s.t[0]) + pv.GRID_DT + 2 * pv.EDGE_S                    # grid + both trimmed edges
        assert abs(span - s.meta['boat_s']) < 1e-6, (k, span, s.meta['boat_s'])
        assert abs(s.meta['boat_s'] - pv.seg_seconds(k)) < 1e-9
    for a, (rn, d, fa) in arms.items():
        b = pv.arm_budget(a)
        if rn == 'full':
            assert pv.FULL_S == 624 and b == 624
            continue
        dd = [d] if d is not None else [tuple(map(tuple, br['design']))
                                        for br in design['chosen'][f'adaptive@{b}']['branches'].values()]
        for x in dd:
            assert sum(segs[k].meta['boat_s'] for k in x) <= b + 1e-9, a
    pv.validate_design(design)
    bad = json.loads(json.dumps(design))
    bad['chosen']['task@60']['design'] = [list(k) for k in pv.candidates(180)['pulse']]
    with pytest.raises(SystemExit):
        pv.validate_design(bad)


# ------------------------------------------------------------------------------------------- passive
def test_passive_timing_posterior_is_the_prior_and_pulse_is_not():
    row = _row('timing')
    pk = pv.passive_design(60)
    ps = pv.simulate(row, pk, 11, 'passive')
    reg = pv.Reg(pv.Evidence([ps[k] for k in pk], {}))
    assert reg.n_rows['surge'] > 50 and reg.n_rows['yaw'] == 0                 # surge excited, no steps
    gidx, w = pv.grid_posterior(reg, 'joint')
    m = pv.timing_marginals(reg, gidx, w)
    assert m['H_timing'] >= 0.99 * m['H_timing_prior']
    for ax in ('delay_s', 'tau_s', 'rev_hold_s'):
        assert np.allclose(m[ax], 1.0 / len(pv.twin.GRID[ax]), atol=0.02), (ax, m[ax])
    d = pv.fixed_designs(60)['pulse']
    segs = pv.simulate(row, d, 11, 'passive')
    r2 = pv.Reg(pv.Evidence([segs[k] for k in d], {}))
    g2, w2 = pv.grid_posterior(r2, 'joint')
    m2 = pv.timing_marginals(r2, g2, w2)
    assert m2['H_timing'] < 0.6 * m2['H_timing_prior']


def test_grid_posterior_map_matches_twin_fit():
    row = _row('mixed')
    d = pv.candidates(60)['P3+ystep0.3']
    segs = pv.simulate(row, d, 5, 'map')
    reg = pv.Reg(pv.Evidence([segs[k] for k in d], {}))
    gidx, w = pv.grid_posterior(reg, 'joint')
    _, rep = pv.twin.fit(reg, 'joint', 4, 1)
    assert tuple(rep['map'].values()) == reg.points[gidx[int(np.argmax(w))]]
    assert abs(rep['grid_entropy'] - float(-(w[w > 0] * np.log(w[w > 0])).sum())) < 1e-9


# ------------------------------------------------------------------------------------------- seeds / CRN
def test_seed_guards():
    for s in range(20261109, 20261117):
        with pytest.raises(SystemExit):
            pv.check_seeds(s, pv.DESIGN_SEED)
        with pytest.raises(SystemExit):
            pv.check_seeds(pv.SEED, s)
    with pytest.raises(SystemExit):
        pv.check_seeds(3131, 3131)
    pv.check_seeds(pv.SEED, pv.DESIGN_SEED)
    with pytest.raises(SystemExit):
        pv.main(['eval', '--out', '/tmp/pv-never', '--seed', '20261112'])


def test_design_and_evaluation_boats_are_disjoint(tmp_path):
    dev = pv.boats(2, pv.DESIGN_SEED, prefix='dev-')
    ev = pv.boats(2, pv.SEED)
    fp = lambda j: tuple(np.round(np.concatenate([np.ravel(v) for v in j['spec'].row(0).values()]), 9))
    assert not ({fp(j) for j in dev} & {fp(j) for j in ev})
    assert not ({j['id'] for j in dev} & {j['id'] for j in ev})
    k = ('ystep0.3', 0)
    sd = {pv._seed(j['seed'], j['id'], 'seg', *k) for j in dev}
    se = {pv._seed(j['seed'], j['id'], 'seg', *k) for j in ev}
    assert not (sd & se)
    # the evaluation phase refuses a design made with its own seed
    d = _fake_design()
    d['design_seed'] = pv.SEED
    (tmp_path / 'design.json').write_text(json.dumps(d))
    with pytest.raises(SystemExit):
        pv.run_eval(tmp_path, pv.SEED, 1, 1)


def test_segment_realization_independent_of_batch():
    row = _row('transient')
    a = pv.simulate(row, [('ystep0.3', 0)], 9, 'crn')
    b = pv.simulate(row, [('coast0.9', 0), ('ystep0.3', 0), ('transit', 0)], 9, 'crn')
    sa, sb = a[('ystep0.3', 0)], b[('ystep0.3', 0)]
    assert np.array_equal(sa.x, sb.x) and np.array_equal(sa.u, sb.u)
    c = pv.simulate(row, [('ystep0.3', 1)], 9, 'crn')[('ystep0.3', 1)]
    assert not np.array_equal(sa.x, c.x)                                       # a repeat is a new realization


def test_ambiguous_stratum():
    spec, share, total = pv.sample_ambiguous(20, pv.SEED)
    tot = spec.dead_time + spec.odom_latency + spec.tau_up
    assert np.all((tot >= 0.30 - 1e-9) & (tot <= 0.40 + 1e-9))
    assert np.allclose((spec.dead_time + spec.odom_latency) / tot, share)
    assert share.min() < 0.1 and share.max() > 0.9
    assert np.histogram(share, bins=4, range=(0, 1))[0].min() >= 4              # stratified share
    assert np.all(spec.rev_hold <= 0.3)
