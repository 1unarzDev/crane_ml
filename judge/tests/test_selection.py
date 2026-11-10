"""judge.selection: the study's definitions pinned to Tools/DecisionCal/score.py on random inputs,
then the cross-layer analysis (regret with CI and drivers, same winner with ties, rank agreement vs
its ceiling, screening) on constructed cases, the record adapter, and the CLI end to end."""

import json
import math
from pathlib import Path
import sys

import numpy as np
import pytest

DC = Path(__file__).resolve().parents[2] / 'Tools' / 'DecisionCal'
if str(DC) not in sys.path:
    sys.path.insert(0, str(DC))
import score as study  # noqa: E402  (Tools/DecisionCal/score.py, tests only)

import judge.record as JR  # noqa: E402
from judge import selection as SEL  # noqa: E402

RNG = np.random.default_rng(20261109)


# ------------------------------------------------------------------ pinned to score.py

@pytest.mark.parametrize('K', [2, 7, 8, 12])
def test_cross_fit_regret_selection_and_band_match_the_study(K):
    for _ in range(25):
        C = int(RNG.integers(2, 17))
        J = RNG.normal(60, 15, (C, K))
        J[RNG.random((C, K)) < 0.05] = np.nan
        Jt = RNG.normal(60, 15, (C, 5))
        Jt[RNG.random((C, 5)) < 0.1] = np.inf
        sel = int(RNG.integers(0, C))
        assert SEL.cross_fit_regret(J, sel) == study.cross_fit_regret(J, sel) or (
            math.isnan(SEL.cross_fit_regret(J, sel)) and math.isnan(study.cross_fit_regret(J, sel)))
        assert SEL.selection(Jt) == study.selection(Jt)
        assert np.array_equal(SEL.finite_scores(Jt), study.finite_scores(Jt))
        Jf = study.finite_scores(J)
        ps = study.plant_scores(Jf, {'nominal': Jt})
        assert SEL.noise_band(Jf) == ps['band']
        assert SEL.cross_fit_regret(Jf, ps['arms']['nominal']['sel']) == ps['arms']['nominal']['regret']


def test_sign_flip_holm_ranks_match_the_study():
    for n in (5, 12, 16, 23):
        d = RNG.normal(0.3, 1.0, n)
        assert SEL.sign_flip_p(d) == study.sign_flip_p(d)
    p = {f'h{i}': float(x) for i, x in enumerate(RNG.random(9))}
    assert SEL.holm(p) == pytest.approx(study.holm(p))
    for _ in range(30):
        n = int(RNG.integers(3, 17))
        a, b = RNG.normal(size=n), RNG.normal(size=n)
        a[:2] = 1.0                                              # ties, broken the study's way
        assert SEL.spearman_study(a, b) == study.spearman(a, b) or (
            math.isnan(SEL.spearman_study(a, b)) and math.isnan(study.spearman(a, b)))
        assert SEL.kendall_study(list(a), list(b)) == study.kendall_tau(list(a), list(b))


def test_bootstrap_matches_the_study_summary():
    reg = RNG.gamma(1.5, 3.0, 25)
    rows = [{'stratum': 's', 'traj_rmse': {'nominal': 1.0},
             'scores': {'arms': {'nominal': {'regret': float(r), 'within_band': True, 'covered': True,
                                             'rank_corr': 0.5}}}} for r in reg]
    want = study.summarize(rows, ['nominal'], n_boot=2000, seed=0)['table']['s']['nominal']['mean_regret_ci90']
    assert SEL.bootstrap_mean_ci(reg, 2000, 0) == pytest.approx(want, abs=0)


# ------------------------------------------------------------------ analysis on constructed cases

def _arm(means, K=8, sd=1.0, seed=0, fail=None, prefix='r'):
    rng = np.random.default_rng(seed)
    C = len(means)
    noise = rng.normal(0, sd, K)                                 # common random numbers across controllers
    J = np.asarray(means, float)[:, None] + noise[None, :] + rng.normal(0, 0.2 * sd, (C, K))
    ids = tuple(f'c{i:02d}' for i in range(C))
    run_ids = [[f'{prefix}-{ids[c]}-s{k}' for k in range(K)] for c in range(C)]
    failed = None if fail is None else np.repeat(np.asarray(fail, bool)[:, None], K, 1)
    return SEL.Arm(ids, J, failed, run_ids, tuple(range(K)))


def test_wrong_pick_costs_regret_with_ci_and_named_drivers():
    fast = _arm([40, 45, 50, 60], seed=1, prefix='fast')          # fast layer prefers c00
    eng = _arm([55, 42, 50, 60], seed=2, prefix='eng')            # the engine layer prefers c01
    eng.J[0, 3] += 30.0                                           # one bad repeat of c00 on the engine
    out = SEL.analyze_scenario({'fast': fast, 'engine': eng}, n_boot=400, seed=0)
    p = out['pairs']['fast->engine']
    assert p['pick'] == 'c00' and p['oracle'] == 'c01' and not p['same_winner'] and not p['same_winner_within_band']
    assert p['regret'] == pytest.approx(SEL.cross_fit_regret(eng.J, 0))
    assert p['regret_ci90'][0] <= p['regret'] <= p['regret_ci90'][1] and p['regret_ci90'][0] > 0
    assert p['drivers'][0]['repeat'] == 3 and p['drivers'][0]['pick_run'] == 'eng-c00-s3'
    assert p['drivers'][0]['oracle_run'] == 'eng-c01-s3'
    assert sum(d['contribution'] for d in p['drivers']) <= p['regret_raw'] + 1e-9 + 30.0 / 8
    back = out['pairs']['engine->fast']
    assert back['pick'] == 'c01' and back['regret'] > 0
    assert SEL.analyze_scenario({'fast': fast, 'engine': eng}, n_boot=400, seed=0) == out     # deterministic


def test_ties_inside_the_noise_band_count_as_the_same_winner():
    a = _arm([40.0, 40.3, 60, 70], sd=3.0, seed=3)
    b = _arm([40.3, 40.0, 60, 70], sd=3.0, seed=4)
    p = SEL.pick_cost(a, b, n_boot=200)
    assert p['band'] > 0.3 and set(p['tied_winners']) >= {'c00', 'c01'}
    assert p['same_winner_within_band']


def test_rank_agreement_reports_its_seed_to_seed_ceiling():
    means = np.linspace(40, 80, 10)
    clean_a, clean_b = _arm(means, sd=0.5, seed=5), _arm(means, sd=0.5, seed=6)
    r = SEL.rank_agreement(clean_a, clean_b)
    assert r['spearman'] > 0.95 and r['ceiling_split_half']['sqrt_product'] > 0.9
    rng = np.random.default_rng(7)
    noisy = SEL.Arm(clean_b.controllers, means[:, None] + rng.normal(0, 80, (10, 8)))
    r2 = SEL.rank_agreement(clean_a, noisy)
    assert r2['ceiling_split_half']['b'] < 0.6 and r2['ceiling_split_half']['sqrt_product'] < 0.8


def test_screening_names_missed_failures_and_false_rejections():
    a = _arm([40, 45, 50, 60], seed=8, fail=[False, False, True, False])
    b = _arm([42, 41, 50, 60], seed=9, fail=[False, True, False, False])
    s = SEL.screen(a, b)
    assert s['missed_failures'] == ['c01'] and s['false_rejections'] == ['c02']
    assert s['top_k_contains_winner'][3]
    v, why = SEL.screening_verdict(1.0, 1, 1)
    assert v == 'cannot_screen' and why
    assert SEL.screening_verdict(1.0, 0, 2)[0] == 'can_screen'
    assert SEL.screening_verdict(1.0, 0, 0)[0] == 'can_screen_rank_only'


# ------------------------------------------------------------------ records, across boats, CLI

def _track(t_arrive, docks=True, dt=0.1, x0=6.0):
    """Straight run into the slip arriving at ``t_arrive`` then at rest 3 s (docks), or stalling at
    x = 4.5 (never docks)."""
    n = int(round(t_arrive / dt))
    x_end = 2.7 if docks else 4.5
    x = np.concatenate([np.linspace(x0, x_end, n), np.full(int(3.0 / dt) + 1, x_end)])
    u = np.concatenate([np.full(n, (x0 - x_end) / t_arrive), np.zeros(int(3.0 / dt) + 1)])
    t = np.arange(len(x)) * dt
    return t, x, u


def _record(boat, arm, ctrl, seed, t_arrive, docks=True, dt=0.1):
    t, x, u = _track(t_arrive, docks, dt)
    ch = {'truth.pose': JR.truth_pose(t, x, np.full_like(t, -5.0), np.full_like(t, math.pi), tick=np.arange(len(t))),
          'truth.twist': JR.truth_twist(t, u, np.zeros_like(t), np.zeros_like(t), tick=np.arange(len(t))),
          'obs.pose': JR.obs_pose(t, x + 0.01, np.full_like(t, -5.0), np.full_like(t, math.pi), tick=np.arange(len(t)))}
    layer = 'fast' if arm == 'fast' else 'integrated'
    h = JR.Header(f'{arm}-{boat}-{ctrl}-s{seed}', layer, 'synthetic', 'marine.asv', JR.Robot('b'),
                  scenario=JR.Scenario(pair_key=f'{boat}:s{seed}', seed=seed, family='dock',
                                       condition={'stratum': 'steady' if boat < 'boat2' else 'timing'}),
                  window=(0.0, float(t[-1]), 'docked' if docks else 'timeout'),
                  backend=JR.Backend(layer, arm), controller=JR.Controller(ctrl), calibration=JR.Calibration(),
                  clock_mode=JR.ClockMode('fixed_step', dt), code=JR.Code('crane', 'x', False),
                  evidence=JR.Evidence('exploratory'), reset=JR.Reset())
    return JR.RunRecord(h, ch)


ARRIVE = {'fast': {'c0': 20.0, 'c1': 24.0, 'c2': 30.0, 'c3': 34.0},
          'engine': {'c0': 31.0, 'c1': 23.0, 'c2': 29.0, 'c3': 35.0}}


def _records():
    rng = np.random.default_rng(11)
    out = []
    for boat in ('boat0', 'boat1', 'boat2'):
        for seed in range(4):
            jit = float(rng.normal(0, 0.4))                              # CRN: shared by every controller
            for arm, by in ARRIVE.items():
                for ctrl, ta in by.items():
                    docks = not (arm == 'engine' and ctrl == 'c2')          # c2 fails on the engine
                    out.append(_record(boat, arm, ctrl, 100 + seed, ta + jit, docks))
    return out


def test_selection_from_records_scored_by_the_pack():
    from judge.pack import evaluate
    from judge.packs import dock_mission as D
    recs = _records()
    score = lambda r: evaluate(r, None, [D.PACK])['dock.J'].value
    failed = lambda r: evaluate(r, None, [D.PACK])['dock.docked'].value != 1.0
    table = SEL.table_from_records(recs, score, failed)
    assert sorted(table) == ['boat0', 'boat1', 'boat2'] and table['boat2']['condition'] == 'timing'
    arm = table['boat0']['arms']['fast']
    assert arm.controllers == ('c0', 'c1', 'c2', 'c3') and arm.J.shape == (4, 4) and arm.seeds == (100, 101, 102, 103)
    res = SEL.analyze(table, n_boot=200)
    allp = res['by_condition']['all']['fast->engine']
    assert allp['n_boats'] == 3 and allp['n_runs'] == 3 * 2 * 16
    assert allp['same_winner']['rate'] == 0.0
    assert allp['mean_regret'] == pytest.approx(31.0 - 23.0, abs=0.6)        # c0 picked, c1 best on the engine
    lo, hi = allp['mean_regret_ci90_boats']
    assert lo <= allp['mean_regret'] <= hi
    assert allp['drivers'] and allp['drivers'][0]['top_run']['pick_run'].startswith('engine-')
    sc = allp['screen']
    assert sc['missed_failure']['k'] == 3 and sc['missed_failure']['n'] == 3   # fast says c2 docks; it fails
    assert sc['verdict'] == 'cannot_screen'
    assert 'fast->engine' in SEL.table_text(res)


def test_unscorable_runs_and_unmatched_seeds_are_reported():
    recs = _records()[:32]
    phys = _record('boat0', 'fast', 'c0', 999, 20.0)
    phys = phys.__class__(phys.header, {'truth.pose': JR.declare_absent('truth.pose', 'pulse log')})
    table = SEL.table_from_records(recs + [phys], lambda r: (r.channel('truth.pose').t[-1] if r.has('truth.pose')
                                                             else None))
    assert table['boat0']['skipped'] == [phys.run_id]


def test_cli_score_diverge_select(tmp_path, capsys):
    from judge import cli_layers
    recs = _records()
    for r in recs:
        JR.save(r, tmp_path / 'recs' / r.run_id)
    assert cli_layers.main(['score', str(tmp_path / 'recs' / recs[0].run_id), '--out', str(tmp_path / 'sc')]) == 0
    out = capsys.readouterr().out
    assert recs[0].run_id in out and 'docked' in out
    rows = json.loads((tmp_path / 'sc' / 'score.json').read_text())['rows']
    assert rows[0]['outcome'] == 'docked' and rows[0]['J'] is not None and 'valid' in rows[0]
    a = tmp_path / 'recs' / 'fast-boat0-c0-s100'
    b = tmp_path / 'recs' / 'engine-boat0-c0-s100'
    assert cli_layers.main(['diverge', str(a), str(b), '--out', str(tmp_path / 'dv')]) == 0
    for f in ('divergence.json', 'divergence.txt', 'evidence.csv'):
        assert (tmp_path / 'dv' / f).is_file()
    d = json.loads((tmp_path / 'dv' / 'divergence.json').read_text())
    assert d['verdict'] == 'diverged' and d['outcome']['a'] == 'docked'
    assert cli_layers.main(['diverge', str(a), str(tmp_path / 'recs' / 'fast-boat0-c1-s100')]) == 2   # refused
    capsys.readouterr()
    assert cli_layers.main(['select', str(tmp_path / 'recs'), '--out', str(tmp_path / 'sel'), '--n-boot', '100']) == 0
    out = capsys.readouterr().out
    assert 'fast->engine' in out and 'cannot_screen' in out
    assert json.loads((tmp_path / 'sel' / 'select.json').read_text())['by_condition']['all']
