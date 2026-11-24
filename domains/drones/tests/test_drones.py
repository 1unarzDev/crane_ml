"""The drones domain: the T1 fly-to-waypoint-and-stop task through fastsim, and the braking study task.

Run from the crane root: python3 -m pytest domains/drones
"""

import json
import math
from pathlib import Path

import numpy as np
import pytest

from crane.run import tier_config
from domains.drones import braking, card
from fastsim import config
from fastsim.batch import adapter_of
from fastsim.run import simulate
from fastsim.scenario import ScenarioSource


def test_T1_flies_to_the_waypoint_and_stops(tmp_path):
    """crane run --domain drones --tier T1's config: one seed reaches its waypoint, holds, and records the stop."""
    _, path, over = tier_config('drones', 'T1')
    over = config.merge(over, {'scenarios': {'dir': str(tmp_path / 'scenarios')}})
    cfg, base = config.load(path, over)
    source = ScenarioSource(**cfg['scenarios'], base=base)
    source.generate([3])
    summary = simulate(source.load(3), adapter_of(cfg, base), cfg, 3, tmp_path / 'run', base)
    stop = json.loads((tmp_path / 'run' / 'drones.json').read_text())
    assert summary['outcome'] == 'SUCCEEDED', summary
    assert summary['plant']['card_sha256'] == card.sha256(card.read('x500'))
    assert 0.0 <= stop['overshoot_m'] < 2.0 and stop['final_error_m'] < 2.0 and stop['alt_dev_m'] < 1.0
    assert 20.0 < stop['max_tilt_deg'] < 45.0 and stop['arrive_s'] > 2.0


def test_the_study_task_is_the_batch_row_and_a_variant_names_itself():
    """braking.stop (one unit) equals that seed's row of a batched call; a card variant replaces the value and
    says so; a parameter the card lacks is an error naming the card's parameters."""
    args = {'card': 'x500', 'v0': 6.0, 'variant': {'mass': 2.5}}
    row = braking.stop(args, 70)
    batch = braking.stops('x500', 6.0, [69, 70], {'mass': 2.5})
    for k in braking.METRICS:
        if math.isfinite(batch[k][1]):
            assert row[k] == float(batch[k][1]), k
    assert row['outcome'] == 'STOPPED' and row['stopped'] == 1.0
    doc = card.override('x500', {'mass': 2.5})
    entry = doc['parameters']['mass']
    assert entry['value'] == 2.5 and entry['status'] == 'VARIANT' and '2.064' in entry['locator']
    assert card.validate(doc) == [] and card.read('x500')['parameters']['mass']['value'] != 2.5
    with pytest.raises(KeyError, match='no parameter'):
        card.override('x500', {'massx': 1.0})
    assert np.isfinite(card.vehicle(doc).mass)
