"""The run index (crane.runs.index), its --where language, runs diff, and crane repro on a toy run."""

import importlib
import json
import os
import re
from pathlib import Path
import shutil
import subprocess
import sys
import tempfile

import pytest

from crane import cli, record
from crane.runs import repro
import crane.runs.index as rindex
from fastsim import batch, config
from fastsim.identity import tree_sha
import toy

rquery = importlib.import_module('crane.runs.query')   # crane.runs.query itself is the API function
TOY = Path(toy.__file__).parent
ROOT = Path(__file__).resolve().parent.parent
ENU = {'world': 'enu', 'origin': 'dock', 'body': 'flu'}
SIM = {'source': 'sim', 't0': 0.0, 'time_multiplier': None}


def toy_runs(tmp, seeds, out, variant='base', over=None):
    cfg, base = config.load(TOY / 'toy.yaml', {'scenarios': {'dir': str(tmp / 'scenarios')}, **(over or {})})
    for s in seeds:
        toy.write_scenario(tmp / 'scenarios', s)
    batch.run_batch(cfg, base, seeds, out, variant, workers=1, progress=False,
                    provenance={'config_file': str(TOY / 'toy.yaml')})
    return [out / variant / f's{s}' for s in seeds]


def crane_run(d, run_id, tier, outcome, metrics, **extra):
    m = record.meta(run_id, domain='boat', tier=tier, kind={'T1': 'dynamic', 'T2': 'unity'}[tier], frame=ENU,
                    clock=SIM, seed=3, backend='unity' if tier == 'T2' else 'fastsim', outcome=outcome,
                    provenance={'git': {'core': {'commit': 'abc123' + '0' * 34, 'dirty': False}}}, **extra)
    return record.write_run(d, m, metrics=metrics).path


@pytest.fixture
def two_roots(tmp_path):
    a, b = tmp_path / 'a', tmp_path / 'b'
    fast = toy_runs(tmp_path, [0, 1], a)
    shutil.copytree(fast[0], a / 'base' / 's0.stale-1')            # superseded copies and partials are skipped
    shutil.copytree(fast[0], a / 'base' / '.s7.partial-1')
    slow = crane_run(b / 'r1', 'r1-timeout', 'T1', 'TIMEOUT', {'time': 130.0, 'score': {'a': 1}},
                     study='s1', arm='a', block='dev')
    quick = crane_run(b / 'r2', 'r2-timeout', 'T1', 'TIMEOUT', {'time': 100.0})
    t2 = crane_run(b / 'r3', 'r3-ok', 'T2', 'SUCCEEDED', {'time': 200.0})
    job = b / 'out' / 'nlc08' / 'job-7'
    job.mkdir(parents=True)
    (job / 'manifest.json').write_text(json.dumps({
        'schema': 'crane.data.manifest/1', 'columns': {'seed': 'int', 'success': 'float'},
        'files': [{'path': 'part-0.csv', 'rows': 2, 'sha256': '0' * 64}], 'study': 'nlc08', 'stage': 'R',
        'arm': 'b', 'block': 'held-out', 'job_id': 'job-7', 'tier': 'T1', 'commit': 'feed' + '0' * 36}))
    (job / 'metrics.json').write_text(json.dumps({'success_rate': 0.75, 'n': 2}))
    return {'a': a, 'b': b, 'fast': fast, 'slow': slow, 'quick': quick, 't2': t2, 'job': job,
            'idx': rindex.Index(tmp_path / 'index.sqlite')}


def ids(rows):
    return sorted(r['run_id'] for r in rows)


def test_index_two_roots_incrementally_and_query_them(two_roots, monkeypatch):
    r, idx = two_roots, two_roots['idx']
    before = {p: tree_sha(p) for p in r['fast']}
    stats = idx.update([r['a'], r['b']])
    assert (stats['added'], stats['seen']) == (6, 6)                # 2 fastsim + 3 crane.run/1 + 1 manifest
    assert {p: tree_sha(p) for p in r['fast']} == before            # a fastsim dir is read, never written
    fast = idx.rows("r.source = 'fastsim'", metrics=True)
    assert [x['seed'] for x in sorted(fast, key=lambda x: x['seed'])] == [0, 1]
    assert {x['domain'] for x in fast} == {'toy'} and fast[0]['metrics']['time'] > 0 and fast[0]['arm'] == 'base'
    job = idx.rows('r.path = ?', [str(r['job'].resolve())], metrics=True)[0]
    assert (job['study'], job['stage'], job['arm'], job['block'], job['run_id']) == ('nlc08', 'R', 'b', 'held-out',
                                                                                     'job-7')
    assert job['metrics'] == {'n': 2.0, 'success_rate': 0.75}

    def q(where):
        return ids(rquery.query(where, idx))
    assert q('outcome=TIMEOUT and tier=T1 and metric.time>120') == ['r1-timeout']
    assert q('outcome=TIMEOUT and tier=dynamic') == ['r1-timeout', 'r2-timeout']
    assert q('study=s1 and block=dev and metric.score.a=1') == ['r1-timeout']
    assert q('(tier=T2 or metric.time<=100) and not source=fastsim') == ['r2-timeout', 'r3-ok']
    assert q('commit=abc123 and run_id~"r?-*"') == ['r1-timeout', 'r2-timeout', 'r3-ok']
    assert q('arm in (b, "base") and dirty=false') == sorted([x['run_id'] for x in fast] + ['job-7'])
    assert q('seed in (0, 1) and card=null') == ids(fast) and q('metric.success_rate!=null') == ['job-7']

    # incremental: a touched file with the same bytes is not reread; changed bytes and removed dirs are
    assert idx.update([r['a'], r['b']])['unchanged'] == 6
    os.utime(r['quick'] / 'metrics.json', ns=(1, 1))
    assert idx.update([r['b']])['unchanged'] == 4
    (r['quick'] / 'metrics.json').write_text(json.dumps({'time': 125.0}))
    shutil.rmtree(r['t2'])
    stats = idx.update([r['b']])
    assert (stats['changed'], stats['removed']) == (1, 1)
    assert q('metric.time>120 and tier=T1') == ['r1-timeout', 'r2-timeout'] and idx.count() == 5

    # the CLI over the same file, and the cluster skipped cleanly when the coordinator is unreachable
    monkeypatch.setenv('CRANE_INDEX', str(idx.path))
    monkeypatch.setenv('CRANE_COORDINATOR', 'http://127.0.0.1:9')
    assert rquery.cluster_rows('nlc08')[0] == [] and 'unreachable' in rquery.cluster_rows('nlc08')[1]
    assert cli.main(['runs', 'ls', '--tier', 'T1', '--where', 'metric.time>120', '--cluster', '--study', 's1',
                     '--json']) == 0


def test_query_language_edge_cases_and_injection(two_roots, capsys, monkeypatch):
    idx = two_roots['idx']
    idx.update([two_roots['b']])
    assert rquery.compile('') == ('1', [])
    sql, params = rquery.compile('tier = "T1" AND (Outcome=x OR not seed>=2)'.replace('Outcome', 'outcome'))
    assert params == ['T1', 'x', 2] and '?' in sql
    for bad, why in [('foo=1', "no field 'foo'"), ('tier=', 'expected a value'), ('tier=T9', 'tier wants T0'),
                     ('seed=x', 'seed wants an integer'), ('metric.time~3', '~ matches text'),
                     ('(tier=T1', "expected ')'"), ('tier=T1 tier=T0', 'join conditions'),
                     ('metric.time>abc', 'wants a number'), ('tier>null', 'null takes'), ('x' * 5000, 'at most'),
                     ('(' * 60 + 'tier=T1' + ')' * 60, 'nests deeper'), ("outcome='x' or 1=1 --", "no field '1'"),
                     ('outcome=x; DROP TABLE runs', "unexpected ';"), ("metric.x') OR 1=1 --", 'unexpected')]:
        with pytest.raises(rquery.QueryError, match=re.escape(why)):
            rquery.compile(bad)
    evil = "x'; DROP TABLE runs; --"
    sql, params = rquery.compile(f'outcome="{evil}" or study=\'" OR 1=1\'')
    assert evil not in sql and 'DROP' not in sql and evil in params
    assert rquery.query(f'outcome="{evil}"', idx) == [] and idx.count() == 4    # runs is still there
    monkeypatch.setenv('CRANE_INDEX', str(idx.path))
    assert cli.main(['runs', 'query', '--where', 'tier=T7']) == 2
    assert 'crane runs query: tier wants T0, T1 or T2' in capsys.readouterr().err


def test_diff_explains_identity_differences(tmp_path, capsys):
    base = toy_runs(tmp_path, [0, 1], tmp_path / 'one')
    tuned = toy_runs(tmp_path, [0], tmp_path / 'two', over={'limits': {'vmax': [1.2, 0.3, 0.6]}})
    copy = tmp_path / 'copy'
    shutil.copytree(base[0], copy)
    a, b, s1, c = (rindex.find(p) for p in (base[0], tuned[0], base[1], copy))
    same = rquery.diff(a, c)
    assert same['same_run'] and same['identity'] == [] and same['metrics'] == []
    cfg = rquery.diff(a, b)
    assert [p['part'] for p in cfg['identity']] == ['config']
    assert any(line.startswith('limits.vmax [1.5, 0.3, 0.6] -> [1.2, 0.3, 0.6]') for line in cfg['identity'][0]['why'])
    assert {m['key'] for m in cfg['metrics']} >= {'time'} and all(
        m['delta'] == pytest.approx(m['b'] - m['a']) for m in cfg['metrics'])
    seed = rquery.diff(a, s1)
    assert [p['part'] for p in seed['identity']] == ['scenario', 'seed']
    assert seed['identity'][1]['why'] == ['0 -> 1'] and {'field': 'seed', 'a': 0, 'b': 1} in seed['meta']
    assert cli.main(['runs', 'diff', str(base[0]), str(tuned[0])]) == 0
    text = capsys.readouterr().out
    assert 'identity differs: config' in text and 'limits.vmax' in text and 'next: crane view' in text


def _git(repo, *args):
    return subprocess.run(['git', '-C', str(repo), *args], check=True, capture_output=True, text=True).stdout


@pytest.mark.skipif(not shutil.which('git'), reason='git: not installed')
def test_repro_reproduces_at_the_commit_and_detects_a_mutated_metric(tmp_path, capsys, monkeypatch):
    (tmp_path / 'tmp').mkdir()
    monkeypatch.setattr(tempfile, 'tempdir', str(tmp_path / 'tmp'))   # kept reruns land under tmp_path
    repo = tmp_path / 'repo'                       # the toy's code state, committed in a repo of its own
    tracked = _git(ROOT, 'ls-files', 'fastsim', 'theater').split()
    for rel in tracked:
        if (ROOT / rel).is_file():
            (repo / rel).parent.mkdir(parents=True, exist_ok=True)
            shutil.copy2(ROOT / rel, repo / rel)
    _git(tmp_path, 'init', '-q', str(repo))
    for k, v in (('user.email', 'repro@test'), ('user.name', 'repro'), ('commit.gpgsign', 'false')):
        _git(repo, 'config', k, v)
    _git(repo, 'add', '-A')
    _git(repo, 'commit', '-q', '-m', 'toy')
    for s in (0, 1):
        toy.write_scenario(repo / 'fastsim' / 'tests' / 'scenarios', s)    # untracked: the tree stays clean
    env = {**os.environ, 'PYTHONPATH': str(repo)}

    def run(seed, *flags):
        subprocess.run([sys.executable, '-m', 'fastsim.batch', '--config', str(repo / 'fastsim/tests/toy.yaml'),
                        '--seeds', str(seed), '--out', str(tmp_path / 'out'), '--workers', '1', *flags],
                       cwd=repo, env=env, check=True, capture_output=True)
        return tmp_path / 'out' / 'base' / f's{seed}'
    clean = run(0)
    row = rindex.find(clean)
    doc = repro.reproduce(row)
    assert doc['reproduced'] and doc['run_sha256']['recorded'] == doc['run_sha256']['rerun']
    assert doc['tree_sha']['recorded'] == doc['tree_sha']['rerun'] and doc['kept'] is None
    assert _git(repo, 'worktree', 'list').count('\n') == 1                # the temporary worktree is gone
    assert cli.main(['repro', str(clean), '--check']) == 0 and 'repro: reproduced' in capsys.readouterr().out

    native = json.loads((clean / 'run.json').read_text())                 # a metric changed after the fact
    (clean / 'run.json').write_text(json.dumps({**native, 'sim_s': native['sim_s'] + 1.0}, indent=2) + '\n')
    doc = repro.reproduce(rindex.find(clean))
    assert not doc['reproduced'] and doc['first_difference'] == {'file': 'run.json', 'why': 'bytes differ'}
    assert {m['key'] for m in doc['metrics']} == {'time', 'sim_s'} and Path(doc['kept']).is_dir()
    shutil.rmtree(Path(doc['kept']).parent.parent)
    assert cli.main(['repro', str(clean), '--check']) == 1
    assert 'first differing file: run.json' in capsys.readouterr().out
    assert cli.main(['repro', str(clean)]) == 0                           # without --check: reported, exit 0
    capsys.readouterr()

    (repo / 'fastsim' / 'README.md').write_text('local edit\n')           # a dirty tree is refused
    dirty = run(1, '--allow-dirty')
    with pytest.raises(repro.Refused, match='dirty tree'):
        repro.recipe(rindex.find(dirty))
    gone = tmp_path / 'gone'
    shutil.copytree(clean, gone)
    native = json.loads((gone / 'run.json').read_text())
    native['identity']['core']['commit'] = 'f' * 40
    (gone / 'run.json').write_text(json.dumps(native))
    assert cli.main(['repro', str(gone)]) == 2
    assert 'crane repro: commit ffffffffffff is in no repository here' in capsys.readouterr().err
    assert _git(repo, 'worktree', 'list').count('\n') == 1
