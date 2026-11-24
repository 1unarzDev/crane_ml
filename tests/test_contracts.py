"""The cross-study hard contracts (CRANE_TOOLING_COORDINATION.md), as executable checks.

Each test is one contract other sessions build on: the fastsim import paths, judge.stats, the cluster backends and
their entry points, the plant card, sim_up, the help.yaml command names and the rented boxes' sparse list.
"""
import importlib
import json
import re
import shutil
import subprocess
import sys
import textwrap
from pathlib import Path

import pytest
import yaml

ROOT = Path(__file__).resolve().parents[1]
BACKENDS = {'fastsim': 'crane_backends.fastsim:ClusterBackend', 'unity': 'crane_backends.unity:ClusterBackend',
            'theater-render': 'crane_backends.theater:ClusterBackend',
            'crane-command': 'crane_backends.command:ClusterBackend',
            'crane-batch': 'crane_backends.batch:ClusterBackend'}
HELP_NAMES = ['crane doctor', 'crane run', 'crane view', 'crane board', 'crane fit', 'crane compare',
              'crane sim check', 'crane bench player', 'crane bench fast', 'crane build player', 'crane card show',
              'crane card export']   # card and bench: one entry per verb (astro's rule, 2026-11-24)


def test_fastsim_import_paths_and_their_shapes():
    from fastsim import config  # noqa: F401
    from fastsim.batch import adapter_of, run_batch
    from fastsim.controller import Controller, DomainAdapter
    from fastsim.pairs import flat_numbers
    from fastsim.plant import Limits
    from fastsim.scenario import JsonScenarioFormat, ScenarioSource
    assert callable(adapter_of) and callable(run_batch) and isinstance(ScenarioSource, type)
    assert issubclass(JsonScenarioFormat, importlib.import_module('fastsim.scenario').ScenarioFormat)
    assert isinstance(Controller, type)
    for hook in ('identity', 'limits', 'controller', 'time_limit', 'finalize', 'metrics'):
        assert callable(getattr(DomainAdapter, hook)), hook
    assert flat_numbers({'a': {'b': 1.5, 'c': 'x'}, 'n': 2}) == {'a.b': 1.5, 'n': 2}
    limits = yaml.safe_load((ROOT / 'fastsim' / 'tests' / 'toy.yaml').read_text())['limits']
    lim = Limits.from_dict(limits)
    assert lim.vmax == (1.5, 0.3, 0.6) and lim.footprint[0] == (0.5, 0.4)


def test_judge_stats_api_and_reference_values():
    from judge import stats
    mean, p = stats.permutation_paired([1.0, 2.0, 3.0, 4.0])                    # exact: 2 of 16 flips as extreme
    assert mean == 2.5 and p == pytest.approx(2 / 16)
    assert stats.permutation_paired([1.0, 2.0, 3.0, 4.0], alternative='greater')[1] == pytest.approx(1 / 16)
    big = [0.3 + 0.01 * k for k in range(30)]                                   # sampled: seeded and deterministic
    assert stats.permutation_paired(big, B=2000, seed=3) == stats.permutation_paired(big, B=2000, seed=3)
    assert stats.permutation_paired(big, B=2000)[1] == pytest.approx(1 / 2001)
    est, (lo, hi) = stats.bootstrap_ci([1.0, 2.0, 3.0, 4.0, 5.0], lambda u: sum(u) / len(u), B=500)
    assert est == 3.0 and lo < 3.0 < hi
    assert stats.holm({'a': 0.01, 'b': 0.04, 'c': 0.03}) == pytest.approx({'a': 0.03, 'b': 0.06, 'c': 0.06})
    assert stats.kendall_tau([1, 2, 3, 4], [1, 2, 3, 4]) == 1.0
    assert stats.kendall_tau([1, 2, 3, 4], [4, 3, 2, 1]) == -1.0


def _pyproject_backends():
    """{name: target} of [project.entry-points."research_cluster.backends"] (no tomllib on Python 3.10)."""
    text = (ROOT / 'pyproject.toml').read_text()
    body = text.split('[project.entry-points."research_cluster.backends"]', 1)[1].split('\n[', 1)[0]
    return dict(re.findall(r'^([\w-]+)\s*=\s*"([^"]+)"', body, re.M))


def test_cluster_backends_and_entry_points():
    assert _pyproject_backends() == BACKENDS                          # exactly these; never a bare 'command'
    from importlib import metadata
    installed = {e.name: e.value for e in metadata.entry_points(group='research_cluster.backends')
                 if e.value.startswith('crane_backends.')}
    assert not installed or installed == BACKENDS, 'stale install: rerun ./install.sh --yes --no-clone'
    # constructible in a child process (the worker's isolate: process), by the entry point's own module:attr
    probe = textwrap.dedent(f"""
        import importlib, sys
        sys.path.insert(0, {str(ROOT)!r})
        for name, target in {BACKENDS!r}.items():
            mod, attr = target.split(':')
            b = getattr(importlib.import_module(mod), attr)(name, {{}})
            assert callable(b.run) and callable(b.metrics) and callable(b.prepare), name
        print('ok')
    """)
    r = subprocess.run([sys.executable, '-I', '-c', probe], capture_output=True, text=True, timeout=120)
    assert r.returncode == 0 and r.stdout.strip() == 'ok', r.stderr


def test_characterized_plant_card_is_untouched():
    # The contract is the path and the profile shape its readers use (plantcard.marine.seed, domains/boat/plant.py),
    # not its bytes: a refit legitimately rewrites the numbers, so no content hash is pinned here.
    doc = json.loads((ROOT / 'Tools' / 'Characterization' / 'profiles' / 'characterized.json').read_bytes())
    assert {'name', 'mass', 'surge', 'sway', 'yaw', 'thrusters'} <= set(doc)


def test_sim_up_path_verbs_and_exit_codes():
    sim_up = ROOT / 'Tools' / 'RoboBoat' / 'sim_up'
    assert sim_up.is_file() and sim_up.stat().st_mode & 0o111, 'sim_up must stay an executable at this path'
    assert subprocess.run(['bash', '-n', str(sim_up)]).returncode == 0
    r = subprocess.run([str(sim_up), '--help'], capture_output=True, text=True, timeout=30)
    assert r.returncode == 0
    for verb in ('up', 'down', 'status', 'check'):
        assert re.search(rf'^\s*sim_up \[?{verb}\b', r.stdout, re.M), verb
    bad = subprocess.run([str(sim_up), 'no-such-verb'], capture_output=True, text=True, timeout=30)
    assert bad.returncode == 2 and 'sim_up' in bad.stdout                # an unknown verb: usage, exit 2


def test_help_yaml_keeps_the_contract_names():
    names = [c['name'] for c in yaml.safe_load((ROOT / 'help.yaml').read_text())['commands']]
    assert set(HELP_NAMES) <= set(names), sorted(set(HELP_NAMES) - set(names))


def _sparse_dirs():
    lines = (ROOT / 'crane' / 'runtime.sparse').read_text().splitlines()
    return [s.strip() for s in lines if s.strip() and not s.lstrip().startswith('#')]


def test_runtime_sparse_copy_runs_a_fastsim_seed(tmp_path):
    dirs = _sparse_dirs()
    assert {'crane_backends', 'fastsim', 'theater', 'plantcard', 'judge'} <= set(dirs)
    missing = [d for d in dirs if not (ROOT / d).is_dir()]
    assert not missing, f'crane/runtime.sparse lists missing dirs: {missing}'
    box = tmp_path / 'box'                                # what a cone sparse checkout has: top-level files + dirs
    box.mkdir()
    for f in ROOT.iterdir():
        if f.is_file():
            shutil.copy2(f, box / f.name)
    for d in dirs:
        shutil.copytree(ROOT / d, box / d, ignore=shutil.ignore_patterns('__pycache__', '*.pyc', 'bin', 'obj'))
    run = textwrap.dedent(f"""
        import sys
        box = {str(box)!r}
        sys.path[:0] = [box, box + '/fastsim/tests']
        sys.dont_write_bytecode = True
        import toy
        from crane_backends.fastsim import ClusterBackend, FastsimBackend
        toy.write_scenario({str(tmp_path / 'sc')!r}, 1)
        b = FastsimBackend().prepare({{'workdir': box + '/fastsim/tests', 'config': 'toy.yaml',
                                       'out': {str(tmp_path / 'out')!r},
                                       'overrides': {{'scenarios': {{'dir': {str(tmp_path / 'sc')!r}}}}}}})
        m = b.metrics(b.run(1))
        assert m['outcome'] == 'SUCCEEDED', m
        for tier in ('t0', 't1'):                         # the boat domain: plant card and course truth included
            b = FastsimBackend().prepare({{'workdir': box, 'config': f'domains/boat/fastsim_{{tier}}.yaml',
                                           'out': {str(tmp_path / 'boat')!r}, 'variant': tier}})
            m = b.metrics(b.run(1))
            assert m['outcome'] == 'SUCCEEDED', (tier, m)
        from fastsim.batch import run_batch  # noqa: F401
        outside = sorted(n for n, mod in sys.modules.items()
                         if (getattr(mod, '__file__', None) or '').startswith({str(ROOT) + '/'!r}))
        assert not outside, outside
        print('ok')
    """)
    r = subprocess.run([sys.executable, '-I', '-c', run], capture_output=True, text=True, timeout=300, cwd=tmp_path)
    assert r.returncode == 0 and r.stdout.strip().endswith('ok'), r.stdout[-2000:] + r.stderr[-2000:]
