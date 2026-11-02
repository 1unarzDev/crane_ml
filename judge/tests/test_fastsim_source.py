"""A fastsim run directory becomes a RunRecord (domain-agnostic)."""

import json

from judge.metric import Status
from judge.packs import generic_motion
from judge.sources.fastsim import FastsimSource


def test_fastsim_run_dir(tmp_path):
    run = tmp_path / 'r1'
    run.mkdir()
    (run / 'run.json').write_text(json.dumps({
        'seed': 3, 'condition': {'task': 'nav'}, 'contacts': [{'t': 1.0, 'id': 'buoy'}],
        'scenario': {'pair_key': 'pk', 'truth_sha256': 'ts'}, 'identity': {}, 'provenance': {}}))
    (run / 'track.jsonl').write_text(''.join(json.dumps({'t': k * 0.1, 'x': k * 0.05, 'y': 0.0, 'yaw': 0.0}) + '\n'
                                             for k in range(50)))
    rec = FastsimSource('demo').load(run)
    assert rec.header.scenario.pair_key == 'pk' and rec.header.scenario.seed == 3
    assert [e.kind for e in rec.events] == ['contact']
    v = generic_motion.PACK.metrics['motion.mean_speed'](rec, None)
    assert v.status == Status.OK and abs(v.value - 0.5) < 1e-6
    assert rec.channel('twist.body').spec.observability == 'estimated'
