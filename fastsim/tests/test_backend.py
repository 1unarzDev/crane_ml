"""The cluster's fastsim backend: prepare a job, run seeds, read metrics."""

from cluster.backends.fastsim import ClusterBackend, FastsimBackend
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


def test_crane_worker_runs_it_from_a_job_spec(tmp_path):
    from types import SimpleNamespace
    toy.write_scenario(tmp_path / 'scenarios', 3)
    spec = {'repo': 'crane', 'workdir': 'fastsim/tests', 'config': 'toy.yaml',
            'overrides': {'scenarios': {'dir': str(tmp_path / 'scenarios')}}}
    ctx = SimpleNamespace(job={'job_id': 'j2', 'spec': spec}, spec=spec, work_dir=tmp_path / 'job',
                          repos={'crane': HERE.parents[1]})
    backend = ClusterBackend('fastsim', {'variant': 'v'})
    backend.prepare(ctx)
    run_dir = backend.run(3)
    m = backend.metrics(run_dir)
    assert run_dir == tmp_path / 'job' / 'runs' / 'v' / 's3'
    assert m['succeeded'] == 1.0 and all(isinstance(v, float) for v in m.values())
    assert ('run.json', 'summary') in [(n, r) for _, n, r in backend.artifacts(run_dir)]


TOY_GENERATE = ['{python}', '-c', 'import sys; sys.path[:0] = [sys.argv[1], sys.argv[1] + "/../.."]; import toy; '
                '[toy.write_scenario(sys.argv[2], s) for s in range(int(sys.argv[3]), int(sys.argv[4]))]',
                '{base}', '{dir}', '{seeds_from}', '{seeds_to}']


def test_a_worker_without_git_resolves_workdir_under_its_root_and_generates_scenarios(tmp_path):
    from types import SimpleNamespace
    spec = {'workdir': 'fastsim/tests', 'config': 'toy.yaml',
            'overrides': {'scenarios': {'dir': str(tmp_path / 'scenarios'), 'generate': TOY_GENERATE}}}
    ctx = SimpleNamespace(job={'job_id': 'j3', 'spec': spec, 'seeds': [4, 5]}, spec=spec, work_dir=tmp_path / 'job',
                          repos={})
    backend = ClusterBackend('fastsim', {'root': str(HERE.parents[1])})
    backend.prepare(ctx)
    assert (tmp_path / 'scenarios' / 'toy-s5.json').is_file()
    assert backend.metrics(backend.run(5))['succeeded'] == 1.0
