"""crane.resources: the CPU budget from env, cgroup v2/v1 quota and affinity; prepare's pool follows it."""

import os

import pytest

from crane import resources


@pytest.fixture
def host(tmp_path, monkeypatch):
    monkeypatch.delenv('CRANE_CPUS', raising=False)
    monkeypatch.setattr(resources, 'CGROUP', tmp_path)
    monkeypatch.setattr(os, 'cpu_count', lambda: 128)
    monkeypatch.setattr(os, 'sched_getaffinity', lambda pid: set(range(128)), raising=False)
    return tmp_path


def test_cgroup_v2_quota_rounds_down(host):
    (host / 'cpu.max').write_text('2457600 100000\n')
    assert resources.cpus() == 24
    (host / 'cpu.max').write_text('250000 100000\n')
    assert resources.cpus() == 2
    (host / 'cpu.max').write_text('10000 100000\n')
    assert resources.cpus() == 1
    (host / 'cpu.max').write_text('max 100000\n')
    assert resources.cpus() == 128


def test_cgroup_v1_affinity_and_precedence(host, monkeypatch):
    (host / 'cpu').mkdir()
    (host / 'cpu' / 'cpu.cfs_quota_us').write_text('800000\n')
    (host / 'cpu' / 'cpu.cfs_period_us').write_text('100000\n')
    assert resources.cpus() == 8
    monkeypatch.setattr(os, 'sched_getaffinity', lambda pid: {0, 1, 2}, raising=False)
    assert resources.cpus() == 3
    (host / 'cpu' / 'cpu.cfs_quota_us').write_text('-1\n')
    (host / 'cpu.max').write_text('2457600 100000\n')
    assert resources.cpus() == 3
    monkeypatch.setenv('CRANE_CPUS', '5')
    assert resources.cpus() == 5          # the env wins, even above the quota


def test_pids_and_prep_processes(host, monkeypatch):
    assert resources.pids_budget() is None
    (host / 'pids.max').write_text('6144\n')
    assert resources.pids_budget() == 6144
    (host / 'pids.max').write_text('max\n')
    assert resources.pids_budget() is None
    monkeypatch.setenv('CRANE_CPUS', '24')
    assert resources.prep_processes(100) == 24
    assert resources.prep_processes(100, gpu=True) == 8
    assert resources.prep_processes(3) == 3
    (host / 'pids.max').write_text('64\n')
    assert resources.prep_processes(100) * 4 * 4 <= 64


def test_prepare_pool_honours_crane_cpus(monkeypatch):
    from engine import prepare as P
    sizes = []

    class FakePool:
        def __init__(self, n):
            sizes.append((n, os.environ.get('NUMBA_NUM_THREADS')))

        def __enter__(self):
            return self

        def __exit__(self, *a):
            return False

        def map(self, fn, jobs, chunksize=1):
            return [([], None, None, []) for _ in jobs]

    class Ctx:
        Pool = FakePool

    monkeypatch.setenv('CRANE_CPUS', '3')
    monkeypatch.setenv('NUMBA_NUM_THREADS', '7')
    monkeypatch.setattr(P, 'get_context', lambda name: Ctx)
    monkeypatch.setattr('fastsim.scenario.ScenarioSource.generate', lambda self, seeds: None)
    monkeypatch.setattr('engine.world.Worlds.stack', staticmethod(lambda x: None))
    monkeypatch.setattr('engine.boat.BoatPlans.stack', staticmethod(lambda x: None))
    P.prepare({'scenarios': {'dir': 'x'}}, '.', list(range(5 * P.CHUNK)))
    assert sizes == [(3, '1')]                     # 5 chunks, 3 cpus: 3 processes of one thread each
    assert os.environ['NUMBA_NUM_THREADS'] == '7'    # restored afterwards
