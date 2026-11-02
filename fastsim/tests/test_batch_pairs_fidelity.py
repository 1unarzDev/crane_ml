"""Batch (20 seeds with workers in well under a minute), the twin-pair archive, the fidelity loop."""

import json
import math
import shutil
import time

import pytest
import yaml

from fastsim import batch, config, fidelity, pairs
from fastsim.identity import tree_sha
from fastsim.plant import Goal, vehicle
from fastsim.run import simulate
from fastsim.scenario import JsonScenarioFormat
from test_core import HERE, limits
import toy


def test_twenty_seeds_run_on_workers_in_under_a_minute_and_match_a_serial_run(toy_cfg, tmp_path):
    cfg, base, folder = toy_cfg
    for seed in range(1, 21):
        toy.write_scenario(folder, seed)
    started = time.perf_counter()
    results = batch.run_batch(cfg, base, range(1, 21), tmp_path / 'out', workers=4)
    assert time.perf_counter() - started < 60
    assert [r.seed for r in results] == list(range(1, 21))
    assert {r.outcome for r in results} == {'SUCCEEDED'}
    assert len({r.run_sha256 for r in results}) == 20
    serial = batch.run_batch(cfg, base, [7], tmp_path / 'serial', workers=1)[0]
    assert serial.run_sha256 == results[6].run_sha256
    assert tree_sha(serial.out) == tree_sha(results[6].out)


def test_seed_ranges_parse():
    assert batch.parse_seeds('3') == [3] and batch.parse_seeds('1:4') == [1, 2, 3]
    assert batch.parse_seeds('5,9') == [5, 9]


def _run(cfg, base, adapter, folder, seed, out, **over):
    scenario = JsonScenarioFormat().load(toy.write_scenario(folder, seed))
    return simulate(scenario, adapter, config.merge(cfg, over), seed, out, base)


def test_the_pair_archive_matches_scenario_and_condition_and_never_rewrites(toy_cfg, toy_adapter, tmp_path):
    cfg, base, folder = toy_cfg
    root = tmp_path / 'archive'
    _run(cfg, base, toy_adapter, folder, 2, tmp_path / 'fast')
    # the twin: the same scenario on another vehicle (stands in for another simulator's run)
    _run(cfg, base, toy_adapter, folder, 2, tmp_path / 'twin', vehicle={'gain': 0.5, 'speed_scale': 0.8})
    _run(cfg, base, toy_adapter, folder, 3, tmp_path / 'other')
    assert len(pairs.wait([tmp_path / 'fast'], root)) == 1
    assert pairs.wait([tmp_path / 'fast'], root) == []
    assert pairs.pair([tmp_path / 'other'], root) == []
    made = pairs.pair([tmp_path / 'twin'], root)
    assert len(made) == 1 and pairs.pair([tmp_path / 'twin'], root) == []
    diff = json.loads((root / 'pairs' / made[0] / 'diff.json').read_text())
    d = diff['run.duration_s']
    assert d['fast_minus_twin'] == pytest.approx(d['fast'] - d['twin']) and d['fast'] != d['twin']
    edited = tmp_path / 'edited'
    shutil.copytree(tmp_path / 'twin', edited)
    run = json.loads((edited / 'run.json').read_text())
    truth = run['scenario']['truth_sha256']
    run['scenario']['truth_sha256'] = 'different'
    (edited / 'run.json').write_text(json.dumps(run))
    with pytest.raises(ValueError, match='not a pair'):
        pairs.pair([edited], root)
    run['scenario']['truth_sha256'], run['condition']['time_limit_s'] = truth, 1.0
    (edited / 'run.json').write_text(json.dumps(run))
    assert pairs.pair([edited], root) == []  # another condition never pairs


TRUE = {'model': 'kinematic', 'gain': 0.6, 'delay_s': [0.4, 0.0, 0.2], 'speed_scale': 0.8}
START = {'model': 'kinematic', 'gain': 1.0, 'delay_s': [0.0, 0.0, 0.0], 'speed_scale': 1.0}


def twin_session(name, goals):
    """A twin session from a known vehicle: goal events and a 10 Hz state track, one clock."""
    lim = limits()
    tracker, plant = vehicle(TRUE, lim)
    track, events, t, dt = [], [], 0.0, fidelity.DT
    for k, g in enumerate(goals):
        events.append({'t': round(t, 3), 'x': g[0], 'y': g[1], 'yaw': g[2]})
        goal = Goal([g])
        for n in range(240):  # 12 s per goal
            if n % 2 == 0:
                track.append([round(t, 3), *plant.pose, *plant.vel])
            plant.step(tracker.command(goal, plant.pose, t), dt)
            t += dt
    return {'session': name, 'segments': fidelity.windows(track, events, stops=[]), 'cuts': []}


def test_the_fidelity_loop_scores_once_refits_toward_the_twin_and_never_rewrites(tmp_path):
    lim = limits()
    e1 = tmp_path / 'cal-e1.yaml'
    fidelity.write_epoch(e1, 1, 'twin', START, {'sessions': ['20260101_000000']})
    fidelity.keep(tmp_path / 'store', [twin_session('20260101_000000', [(6.0, 1.0, 0.5)])])
    batch_a = twin_session('20260201_000000', [(8.0, 2.0, 0.0), (12.0, -4.0, -1.0), (4.0, -2.0, 3.0)])
    assert len(batch_a['segments']) == 3
    bounds = {'gain': [0.1, 2.0], 'speed_scale': [0.4, 1.2], 'delay_s.0': [0.0, 1.0]}
    row = fidelity.score_batch(e1, tmp_path / 'store', [batch_a], lim, START, bounds=bounds,
                               ratios=[1.1, 0.9, 2.0])
    assert row['score']['n'] == 3 and row['duration_band']['outside'] == 1
    e2 = tmp_path / 'cal-e2.yaml'
    fitted = yaml.safe_load(e2.read_text())
    assert fitted['epoch'] == 2 and fitted['fit']['sessions'] == ['20260101_000000', '20260201_000000']
    assert fitted['fit']['segments'] == 4
    start, fit = fitted['fit']['rms_mean_m']['start'], fitted['fit']['rms_mean_m']['fit']
    assert fit < start / 2
    assert abs(fitted['plant']['speed_scale'] - TRUE['speed_scale']) < 0.1
    with pytest.raises(ValueError, match='scored once'):
        fidelity.score_batch(e1, tmp_path / 'store', [batch_a], lim, START)
    old = twin_session('20251231_000000', [(5.0, 0.0, 0.0)])
    with pytest.raises(ValueError, match='not newer'):
        fidelity.score_batch(e1, tmp_path / 'store', [old], lim, START)
    with pytest.raises(FileExistsError):
        fidelity.write_epoch(e2, 2, 'twin', START, {})
    fidelity.trend_row(tmp_path / 'note.md', row)
    assert '| 1 | 3 |' in (tmp_path / 'note.md').read_text()


def test_clopper_pearson_is_exact():
    lo, hi = fidelity.clopper_pearson(3, 10, 0.05)
    assert (round(lo, 4), round(hi, 4)) == (0.0667, 0.6525)
    assert fidelity.clopper_pearson(0, 5, 0.05)[0] == 0.0 and fidelity.clopper_pearson(5, 5, 0.05)[1] == 1.0
    assert math.isclose(fidelity.clopper_pearson(0, 10, 0.05)[1], 1 - 0.025 ** 0.1, rel_tol=1e-6)


def test_a_seed_file_reference_resolves_beside_the_yaml(toy_cfg):
    cfg, base, _ = toy_cfg
    assert base == HERE and batch.adapter_of(cfg, base).options['reach_m'] == 1.5
