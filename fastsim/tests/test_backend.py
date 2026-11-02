"""The cluster's fastsim backend: prepare a job, run seeds, read metrics."""

from cluster.backends.fastsim import FastsimBackend
from test_core import HERE
import toy


def test_the_backend_runs_a_job_seed_by_seed(tmp_path):
    for seed in (1, 2):
        toy.write_scenario(tmp_path / 'scenarios', seed)
    job = {'id': 'j1', 'workdir': str(HERE), 'config': 'toy.yaml', 'out': str(tmp_path / 'out'),
           'variant': 'v', 'overrides': {'scenarios': {'dir': str(tmp_path / 'scenarios')}},
           'calibration': 'calibration-e1.yaml'}
    backend = FastsimBackend().prepare(job)
    run_dir = backend.run(2)
    assert run_dir == tmp_path / 'out' / 'v' / 's2'
    m = backend.metrics(run_dir)
    assert m['outcome'] == 'SUCCEEDED' and len(m['run_sha256']) == 64
    assert m['run.duration_s'] == m['run.sim_s'] > 0
    assert backend.cfg['vehicle']['gain'] == 0.15
