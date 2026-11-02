"""Core: determinism, the plant's step response against a calibration, formats, no domain names.

Run from the crane root: python3 -m pytest fastsim/tests
"""

import json
import math
from pathlib import Path
import re

import pytest
import yaml

from fastsim import config, identity
from fastsim.plant import FirstOrderPlant, KinematicPlant, Limits, vehicle
from fastsim.run import simulate
from fastsim.scenario import CraneTruthFormat, JsonScenarioFormat
from fastsim.world import World
import toy

HERE = Path(__file__).parent
CORE = HERE.parent
DT = 0.05


def limits(**over):
    d = yaml.safe_load((HERE / 'toy.yaml').read_text())['limits']
    return Limits.from_dict({**d, **over})


def run_once(cfg, base, adapter, folder, seed, out):
    scenario = JsonScenarioFormat().load(toy.write_scenario(folder, seed))
    return simulate(scenario, adapter, cfg, seed, out, base)


def test_a_run_is_identical_byte_for_byte(toy_cfg, toy_adapter, tmp_path):
    cfg, base, folder = toy_cfg
    a = run_once(cfg, base, toy_adapter, folder, 3, tmp_path / 'a')
    b = run_once(cfg, base, toy_adapter, folder, 3, tmp_path / 'b')
    c = run_once(cfg, base, toy_adapter, folder, 4, tmp_path / 'c')
    assert identity.tree_sha(tmp_path / 'a') == identity.tree_sha(tmp_path / 'b')
    assert a['identity'] == b['identity'] and a['identity']['run_sha256'] != c['identity']['run_sha256']
    assert a['outcome'] == 'SUCCEEDED' and (tmp_path / 'a' / 'toy.json').exists()
    with pytest.raises(FileExistsError):  # a run directory is written once
        run_once(cfg, base, toy_adapter, folder, 3, tmp_path / 'a')


def test_seed_streams_are_independent_and_stable():
    assert identity.derive_seed(1, 'perception') != identity.derive_seed(1, 'controller')
    assert identity.derive_seed(1, 'perception') != identity.derive_seed(2, 'perception')
    assert identity.rng(5, 'x').random() == identity.rng(5, 'x').random()


def test_the_calibration_epoch_sets_the_vehicle_and_is_recorded(tmp_path, toy_adapter):
    folder = tmp_path / 'scenarios'
    cfg, base = config.load(HERE / 'toy.yaml', {'scenarios': {'dir': str(folder)},
                                                'calibration': 'calibration-e1.yaml'})
    assert cfg['vehicle'] == {'model': 'kinematic', 'gain': 0.15, 'delay_s': [1.2, 0.0, 0.6],
                              'speed_scale': 0.7}
    run = run_once(cfg, base, toy_adapter, folder, 1, tmp_path / 'r')
    assert run['plant'] == {'gain': 0.15, 'delay_s': [1.2, 0.0, 0.6], 'speed_scale': 0.7}
    assert run['config']['calibration']['epoch'] == 1
    assert len(run['config']['calibration']['sha256']) == 64


def test_the_kinematic_plant_step_response_matches_the_calibration():
    """A constant command from rest: each axis holds zero for its delay, then ramps at its
    accel limit to the scaled command (the calibration's form, step-quantised)."""
    cal = yaml.safe_load((HERE / 'calibration-e1.yaml').read_text())['plant']
    lim = limits()
    plant = KinematicPlant(lim, delay_s=cal['delay_s'], speed_scale=cal['speed_scale'])
    target = (1.0, 0.0, 0.4)
    for n in range(1, 120):
        plant.step(target, DT)
        vx_on = n - round(cal['delay_s'][0] / DT)  # steps since the delayed command arrived
        wz_on = n - round(cal['delay_s'][2] / DT)
        vx = min(cal['speed_scale'] * target[0], lim.accel[0] * DT * vx_on) if vx_on > 0 else 0.0
        wz = min(target[2], lim.accel[2] * DT * wz_on) if wz_on > 0 else 0.0
        assert plant.vel[0] == pytest.approx(vx, abs=1e-9), n
        assert plant.vel[2] == pytest.approx(wz, abs=1e-9), n
    assert plant.vel[0] == pytest.approx(0.7) and plant.vel[1] == 0.0


def test_the_calibrated_gain_sets_the_tracker_yaw_rate():
    tracker, _ = vehicle({'model': 'kinematic', 'gain': 0.15}, limits())
    from fastsim.plant import Goal
    goal = Goal([(10.0, 3.0, 0.0)])
    err = math.atan2(3.0, 10.0)
    assert tracker.command(goal, (0.0, 0.0, 0.0), 0.0)[2] == pytest.approx(0.15 * err)


def test_the_first_order_plant_lags_by_tau_and_reads_a_plant_card(tmp_path):
    fast = limits(accel=[100.0, 100.0, 100.0], decel=[-100.0, -100.0, -100.0])
    plant = FirstOrderPlant(fast, tau_s=0.5)
    for _ in range(10):  # 0.5 s = tau
        plant.step((1.0, 0.0, 0.0), DT)
    assert plant.vel[0] == pytest.approx(1 - math.exp(-1), abs=1e-9)
    (tmp_path / 'card.json').write_text(json.dumps({'thrusters': {'spool': {'tau_s': 0.25, 'L_cmd': 0.1}}}))
    _, carded = vehicle({'model': 'first_order', 'gain': 1.0, 'card': 'card.json',
                         'card_map': {'tau_s': 'thrusters.spool.tau_s', 'dead_time_s': 'thrusters.spool.L_cmd'}},
                        fast, base=tmp_path)
    assert carded.params() == {'tau_s': [0.25] * 3, 'dead_time_s': [0.1] * 3, 'speed_scale': 1.0}
    carded.step((1.0, 0.0, 0.0), DT)
    carded.step((1.0, 0.0, 0.0), DT)
    assert carded.vel[0] == 0.0  # still inside the dead time


def test_crane_truth_is_read_in_the_start_frame_and_a_bad_identity_is_refused(tmp_path):
    truth = {'name': 't', 'startX': 10.0, 'startY': 5.0, 'startYaw': math.pi / 2, 'boundary': [],
             'elements': [{'id': 'a', 'x': 10.0, 'y': 15.0, 'radius': 0.3, 'label': 'thing', 'state': None},
                          {'id': 'p', 'x': 0.0, 'y': 0.0, 'radius': 0.0, 'label': '', 'state': None,
                           'length': 4.0, 'width': 1.0, 'yaw': math.pi / 2}],
             'states': [], 'generator': {'identity': {'pair_key': 'k', 'truth_sha256': 's'}}}
    path = tmp_path / 'course_truth.json'
    path.write_text(json.dumps(truth))
    scenario = CraneTruthFormat().load(path)
    world = World(scenario)
    assert world.xy[0].tolist() == pytest.approx([10.0, 0.0])
    assert world.box.tolist() == [False, True] and world.box_dims[1][2] == pytest.approx(0.0)
    assert scenario.pair_key == ('k', 's') and scenario.filename == 'course_truth.json'
    assert world.to_world((10.0, 0.0, 0.0)) == pytest.approx((10.0, 15.0, math.pi / 2))
    with pytest.raises(ValueError, match='identity'):
        CraneTruthFormat(identity_check=lambda t: ['edited']).load(path)


DOMAIN_WORDS = ('roboboat', 'mhseals', 'boat', 'buoy', 'nav2', 'mppi', 'ros', 'blastoise',
                'tranquility', 'dingus', 'referee', 'horn', 'marina', 'dock')


def test_the_core_names_no_domain():
    for path in sorted(CORE.rglob('*')):
        if path.suffix in ('.py', '.yaml') and 'tests' not in path.relative_to(CORE).parts:
            text = path.read_text().lower()
            found = [w for w in DOMAIN_WORDS if re.search(rf'\b{w}s?\b', text)]
            assert not found, f'{path.name}: {found}'
