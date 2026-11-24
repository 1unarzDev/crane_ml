"""The boat domain: deterministic T0/T1 episodes, the T1 plant against the twin's parity cases,
and the reference mission on the committed course truths.

Run from the crane root: python3 -m pytest domains
"""

import json
import math
from pathlib import Path

import pytest

from fastsim import config, identity
from fastsim.batch import adapter_of
from fastsim.plant import Limits
from fastsim.run import simulate
from fastsim.scenario import ScenarioSource

from domains.boat.plant import ROOT, ProfileBoat, characterization

BOAT = ROOT / 'domains' / 'boat'
SEED = 1


def episode(tier, seed, out):
    cfg, base = config.load(BOAT / f'fastsim_{tier.lower()}.yaml')
    source = ScenarioSource(**cfg['scenarios'], base=base)
    summary = simulate(source.load(seed), adapter_of(cfg, base), cfg, seed, out, base)
    return summary, json.loads((Path(out) / 'boat.json').read_text())


@pytest.mark.parametrize('tier', ['T0', 'T1'])
def test_an_episode_finishes_and_repeats_byte_for_byte(tier, tmp_path):
    a, score = episode(tier, SEED, tmp_path / 'a')
    b, _ = episode(tier, SEED, tmp_path / 'b')
    assert a['outcome'] == 'SUCCEEDED' and score['success'] == 1 and score['tier'] == tier
    assert a['identity']['run_sha256'] == b['identity']['run_sha256']
    assert identity.tree_sha(tmp_path / 'a') == identity.tree_sha(tmp_path / 'b')


def test_the_reference_mission_passes_every_gate_and_docks_at_T1(tmp_path):
    """Every committed course: all gate passes in order, a full turn round the mark, docked."""
    for seed in range(5):
        summary, score = episode('T1', seed, tmp_path / f's{seed}')
        assert summary['outcome'] == 'SUCCEEDED', (seed, summary['error'])
        assert score['gates_passed'] == score['gates_total'] >= 6, (seed, score)
        assert score['mark_sweep_deg'] > 300, (seed, score)
        assert score['dock_error_m'] <= 0.5 and score['success'] == 1, (seed, score)


def test_the_T1_plant_is_the_twin_on_its_parity_cases():
    """Tools/Characterization/parity/cases.json (the C# parity gate) through ProfileBoat's own PWM
    path: every sampled state equal to the twin's to 1e-12."""
    cases = json.loads((ROOT / 'Tools' / 'Characterization' / 'parity' / 'cases.json').read_text())
    profile = ROOT / 'Tools' / 'Characterization' / cases['profileFile']
    lim = Limits.from_dict(config.load(BOAT / 'fastsim_t1.yaml')[0]['limits'])
    for case in cases['cases']:
        dt = case['dt']
        plant = ProfileBoat(lim, profile=profile, tick_s=dt, current=(case['currentX'], case['currentY']))
        wanted = dict(zip(case['expectedTicks'], range(len(case['expectedTicks']))))
        worst = 0.0
        for k in range(case['steps']):
            plant.apply_pwm(case['pwm'][4 * k:4 * k + 4], dt)
            if k + 1 in wanted:
                j = wanted[k + 1]
                expected = case['expected'][6 * j:6 * j + 6]
                worst = max(worst, max(abs(a - b) for a, b in zip(plant.twin.state, expected)))
        assert worst <= 1e-12, (case['name'], worst)


def test_a_closed_loop_step_is_the_twin_fed_the_same_pulses():
    """10 s of velocity targets through ProfileBoat.step (two ticks per step), replayed as PWM into
    a fresh ProfilePlant: the same states; the pose is the twin's with the heading wrapped."""
    cfg = config.load(BOAT / 'fastsim_t1.yaml')[0]
    lim = Limits.from_dict(cfg['limits'])
    plant = ProfileBoat(lim, profile=cfg['vehicle']['profile'], kp=cfg['vehicle']['kp'])
    twin = characterization('ProfilePlant')(plant.profile, plant.tick_s)
    dt = 2 * plant.tick_s
    for k in range(int(round(10.0 / dt))):
        t = k * dt
        plant.step((1.2 if t < 6 else -0.3, 0.3 * math.sin(t), 0.6 if 2 < t < 8 else -0.2), dt)
        for _ in range(2):
            twin.step_pwm(plant.pwm)
        assert plant.twin.state == twin.state, k
    x, y, psi = twin.state[:3]
    assert plant.pose == (x, y, math.atan2(math.sin(psi), math.cos(psi)))
    assert math.hypot(x, y) > 5.0  # it went somewhere: the loop drives the twin
