"""The cluster side: study -> rc-job/1, the crane-batch backend, crane monitor and crane cluster fetch.

A tiny fake Study stands in for crane.study (w3-study owns it): Study.cells() -> Cell{study, stage, arm, axes,
condition, tier, seeds, block, key}, plus name, purpose and blocks.
"""
import json
import math
import subprocess
import textwrap
from dataclasses import dataclass, field
from pathlib import Path
from types import SimpleNamespace

import pytest

from crane import output, progress
from crane.cache import UnitCache
from crane.cluster import compile as cc, fetch as cf
from crane.cluster.submit import write_plan
from crane.data.rows import RowWriter, read_rows, write_manifest
from crane import monitor
from crane_backends.batch import ClusterBackend

ROOT = Path(__file__).resolve().parents[1]
PINS = {'crane_ml': {'name': 'crane_ml', 'url': str(ROOT), 'ref': 'a' * 40},
        '_spec': {'repo': 'crane_ml', 'path': 'studies/s.yaml'}}


@dataclass
class Cell:
    study: str
    stage: str
    arm: str
    seeds: list
    block: str = 'dev'
    tier: str = 'T1'
    axes: dict = field(default_factory=dict)
    condition: dict = None
    est_unit_s: float = None

    @property
    def key(self):
        return {'study': self.study, 'stage': self.stage, 'arm': self.arm, 'axes': self.axes, 'block': self.block}


class Study:
    def __init__(self, cells, blocks=None):
        self.name, self.purpose = 'toy01', 'Pairs two toy arms on shared seeds to test the cluster compiler end to end.'
        self._cells, self.blocks = cells, blocks or {'dev': {'role': 'dev'}, 'test': {'role': 'held-out'}}

    def cells(self):
        return iter(self._cells)


def _compile(study, **kw):
    return cc.compile(study, pins=dict(PINS), **kw)


def test_compile_sizes_jobs_pairs_arms_dedups_and_refuses_field_data_and_held_out(tmp_path):
    seeds = list(range(100))
    study = Study([Cell('toy01', 'A', arm, seeds) for arm in ('base', 'tuned')])
    jobs = _compile(study, est_unit_s=60.0)
    # 60 s units: 15 per job (15 min), 100 seeds -> 7 even chunks; every job inside 5-30 min, est honest
    assert len(jobs) == 14 and not jobs.warnings
    for j in jobs:
        n = len(j['spec']['seeds'])
        assert j['est_seconds'] == pytest.approx(60.0 * n + cc.OVERHEAD_S)
        assert cc.JOB_MIN_S <= j['est_seconds'] <= cc.JOB_MAX_S
        assert j['schema'] == 'rc-job/1' and j['kind'] == 'crane_batch' and j['seeds'] == [0]
        assert j['repos'] == [PINS['crane_ml']] and 'split' not in j['spec']
        assert '--job' in j['spec']['argv'] and 'crane_backends.batch' in j['spec']['run']
        assert len(j['purpose']) >= cc.PURPOSE_MIN and j['campaign'] == 'toy01'
    by_arm = {arm: [j['spec']['seeds'] for j in jobs if j['spec']['arm'] == arm] for arm in ('base', 'tuned')}
    assert by_arm['base'] == by_arm['tuned'] and sum(by_arm['base'], []) == seeds   # paired chunks, nothing lost
    assert len({j['job_id'] for j in jobs}) == 14 and all(len(j['job_id']) <= 128 for j in jobs)
    inner = json.loads(jobs[0]['spec']['argv'][-1])
    assert inner['cell'] == jobs[0]['spec']['cell'] and inner['seeds'] == jobs[0]['spec']['seeds']

    # dedup: the same plan has the same ids; another commit, other ids; done units are left out
    assert [j['job_id'] for j in _compile(study, est_unit_s=60.0)] == [j['job_id'] for j in jobs]
    other = cc.compile(study, est_unit_s=60.0, pins={**PINS, 'crane_ml': {**PINS['crane_ml'], 'ref': 'b' * 40}})
    assert not {j['job_id'] for j in other} & {j['job_id'] for j in jobs}
    cache = UnitCache(tmp_path / 'units')
    base = study._cells[0]
    for s in range(30):
        cache.put(cc.unit_key(base, s), {'success': 1})
    left = _compile(study, est_unit_s=60.0, done=cache)
    assert sorted(sum((j['spec']['seeds'] for j in left if j['spec']['arm'] == 'base'), [])) == seeds[30:]
    assert jobs[0]['spec']['dedup'] != left[0]['spec']['dedup']

    # sizing at the edges: a 40-min unit gets one seed per job (warned); a tiny cell is one job (warned)
    assert all(len(j['spec']['seeds']) == 1 for j in _compile(Study([Cell('toy01', 'A', 'big', [1, 2])]),
                                                               est_unit_s=2400.0))
    small = _compile(Study([Cell('toy01', 'A', 'small', list(range(30)))]), est_unit_s=1.0)
    assert len(small) == 1 and 'under the 5 min' in small.warnings[0]
    with pytest.raises(cc.CompileError, match='no time estimate'):
        _compile(study)
    with pytest.raises(cc.CompileError, match='over the 240 min'):
        _compile(study, est_unit_s=cc.SEED_CAP_S + 1)

    # field data never goes to the cluster
    for axes, cond in [({'log': '~/field_corpus_boat/2026-11/a.mcap'}, None),
                       ({}, {'source': '/home/x/data/lake_bags/run3'}),
                       ({'replay': '/tmp/session.db3'}, None)]:
        bad = Study([Cell('toy01', 'A', 'replay', [1], axes=axes, condition=cond)])
        with pytest.raises(cc.CompileError, match='field data'):
            _compile(bad, est_unit_s=60.0)

    # a held-out block: refused without --confirm; with it, the jobs carry the role (monitor and metrics hide outcomes)
    held = Study([Cell('toy01', 'A', 'base', [1, 2, 3], block='test', est_unit_s=200.0)])
    with pytest.raises(cc.CompileError, match='held-out.*--confirm'):
        _compile(held)
    ok = _compile(held, confirm=True)
    assert {j['spec']['role'] for j in ok} == {'held-out'}

    # pins: a dirty tracked file in the pinned repo is refused (the job would run the commit)
    repo = tmp_path / 'repo'
    repo.mkdir()
    git = lambda *a: subprocess.run(['git', '-C', str(repo), *a], check=True, capture_output=True)  # noqa: E731
    git('init', '-q')
    (repo / 'study.yaml').write_text('name: toy01\n')
    git('add', '.')
    git('-c', 'user.email=t@t', '-c', 'user.name=t', 'commit', '-qm', 'x')
    assert cc.pin_repo(repo, 'r')['ref'] == git('rev-parse', 'HEAD').stdout.decode().strip()
    (repo / 'study.yaml').write_text('name: changed\n')
    with pytest.raises(cc.CompileError, match='uncommitted'):
        cc.pin_repo(repo, 'r')


FAKE_RUN_CELL = textwrap.dedent('''
    import math, os, sys, time
    from crane.progress import Progress
    from crane.data.rows import RowWriter, write_manifest
    spec, cell, seeds, out = sys.argv[1:5]
    seeds = [int(s) for s in seeds.split(',')]
    if cell.endswith('crash'):
        raise RuntimeError('the plant diverged at t=3.2 s')
    if cell.endswith('stall'):
        time.sleep(60)
    schema = {'seed': 'int', 'arm': 'str', **{f'm{i}': 'float' for i in range(1200)}, 'bad': 'float'}
    w = RowWriter(out, schema, fmt='csv', part=0)
    with Progress(None, total=len(seeds), every_s=0) as p:
        for s in seeds:
            for ep in range(3):
                w.write({'seed': s, 'arm': 'base', **{f'm{i}': s + ep + i for i in range(1200)}, 'bad': math.nan})
            p.step()
    write_manifest(out, schema, w.close(), {})
''')


def _backend(tmp_path, cell, role='dev', stall_s=None):
    script = tmp_path / 'fake_run_cell.py'
    script.write_text(FAKE_RUN_CELL)
    spec = {'schema': 'crane.cluster.job/1', 'study': 'toy01', 'stage': 'A', 'arm': 'base', 'block': 'dev',
            'role': role, 'cell': cell, 'cell_dir': f'A/base/{cell}', 'seeds': [4, 5, 6],
            'spec_file': {'text': 'name: toy01\n', 'name': 'study.yaml'}, 'stall_s': stall_s or 600}
    b = ClusterBackend('crane_batch', {'command': ['{python}', str(script), '{spec}', '{cell}', '{seeds}', '{out}'],
                                       'poll_s': 0.1, 'stall_s': stall_s})
    work = tmp_path / f'work-{cell}'
    b.prepare(SimpleNamespace(job={'job_id': f'crane/toy01.A.base.c0.{cell}', 'spec': spec}, work_dir=work,
                              repos={'crane_ml': str(ROOT)}))
    return b


def test_batch_backend_runs_a_cell_and_keeps_the_compute_contract(tmp_path, monkeypatch):
    monkeypatch.delenv('CRANE_RUN_DIR', raising=False)
    monkeypatch.delenv('CRANE_OUT_ROOT', raising=False)
    b = _backend(tmp_path, 'A.base.ok')
    rd = b.run(0)
    state = progress.read(rd)
    assert state['phase'] == 'done' and state['done'] == state['total'] == 3
    m = json.loads((rd / 'metrics.json').read_text())
    assert len(m) <= 1000 and all(isinstance(v, (int, float)) and math.isfinite(v) for v in m.values())
    assert m['rows'] == 9 and m['units'] == 3 and m['m0.mean'] == pytest.approx(6.0) and 'bad.mean' not in m
    assert b.metrics(rd) == pytest.approx({k: float(v) for k, v in m.items()})
    cell = json.loads((rd / 'cell.json').read_text())
    assert cell['role'] == 'dev' and cell['job_id'] == 'toy01.A.base.c0.A.base.ok' and cell['host']
    man = json.loads((rd / 'artifacts' / 'rows' / 'manifest.json').read_text())
    assert man['rows'] == 9 and man['cell'] == 'A.base.ok' and man['seeds'] == [4, 5, 6]
    names = {name for _, name, _ in b.artifacts(rd)}
    assert {'metrics.json', 'progress.json', 'cell.json', 'artifacts/rows/manifest.json',
            'artifacts/rows/rows-0.csv'} <= names
    assert len(list(read_rows(rd / 'artifacts' / 'rows'))) == 9

    # with an output root (a rented box's /workspace/out) the rows go to OUT_ROOT/<study>/<job_id>/
    monkeypatch.setenv('CRANE_OUT_ROOT', str(tmp_path / 'box-out'))
    rd = _backend(tmp_path, 'A.base.ok2').run(0)
    assert (tmp_path / 'box-out' / 'toy01' / 'toy01.A.base.c0.A.base.ok2' / 'rows-0.csv').is_file()
    assert not (rd / 'artifacts' / 'rows' / 'rows-0.csv').exists() and (rd / 'artifacts/rows/manifest.json').is_file()
    monkeypatch.delenv('CRANE_OUT_ROOT')

    # held-out: the feed publishes metric means, so a held-out job reports counts only
    rd = _backend(tmp_path, 'A.base.held', role='held-out').run(0)
    assert set(json.loads((rd / 'metrics.json').read_text())) == {'rows', 'failed', 'units', 'wall_s', 'units_per_s'}

    # failures leave their reason and a traceback sample
    with pytest.raises(RuntimeError, match='crashed.*plant diverged'):
        _backend(tmp_path, 'A.base.crash').run(0)
    fail = json.loads((tmp_path / 'work-A.base.crash' / 'runs' / 's0' / 'failure.json').read_text())
    assert fail['reason'] == 'crashed' and fail['traceback'].startswith('Traceback')
    assert progress.read(tmp_path / 'work-A.base.crash' / 'runs' / 's0')['phase'] == 'failed'
    with pytest.raises(RuntimeError, match='timed out without progress'):
        _backend(tmp_path, 'A.base.stall', stall_s=0.5).run(0)


def _cell(root, rel, meta, done, total, metrics=None, failure=None):
    d = root / rel
    d.mkdir(parents=True)
    p = progress.Progress(d, total=total, every_s=0, label=meta['cell'])
    p.step(done)
    (d / 'cell.json').write_text(json.dumps(meta))
    if metrics:
        (d / 'metrics.json').write_text(json.dumps(metrics))
    if failure:
        (d / 'failure.json').write_text(json.dumps(failure))
    return d


def test_monitor_merges_local_and_cluster_and_shows_progress_only_on_held_out(tmp_path):
    study = tmp_path / 'toy01'
    base = {'study': 'toy01', 'stage': 'A', 'arm': 'base'}
    _cell(study, 'A/base/c1', {**base, 'block': 'dev', 'role': 'dev', 'cell': 'c1', 'job_id': 'j1', 'host': 'h1'},
          3, 10, {'rows': 30, 'success.mean': 0.5})
    _cell(study, 'A/base/c2', {**base, 'block': 'dev', 'role': 'dev', 'cell': 'c2', 'job_id': 'j2', 'host': 'h2'},
          5, 10, {'rows': 10, 'success.mean': 0.9},
          {'reason': 'timed out without progress', 'traceback': 'Traceback\nTimeoutError: stuck', 'host': 'h2'})
    _cell(study, 'A/base/h1', {**base, 'block': 'test', 'role': 'held-out', 'cell': 'h1', 'job_id': 'j3'},
          2, 8, {'rows': 20, 'success.mean': 0.777})
    # the cluster also knows j1 (counted once: the local copy is fresher) and a queued j4
    plan = {'project': 'crane', 'campaign': 'toy01', 'jobs': [
        {'job_id': 'j1', 'spec': {**base, 'block': 'dev', 'role': 'dev', 'cell': 'c1', 'seeds': [1, 2]}},
        {'job_id': 'j4', 'spec': {**base, 'block': 'dev', 'role': 'dev', 'cell': 'c3', 'seeds': [3, 4]}}]}
    feed = {'jobs': [{'id': 'crane/j1', 'state': 'running', 'progress': {'fraction': 0.5}, 'worker': 'h1'},
                     {'id': 'crane/j4', 'state': 'queued', 'eta': {'remaining_s': 600}}]}
    entries = monitor.local_entries(study) + monitor.cluster_entries(plan, feed, None)
    groups = {g['block']: g for g in monitor.merge(entries)}
    dev, held = groups['dev'], groups['test']
    assert (dev['done'], dev['total']) == (8, 22) and dev['jobs']['queued'] == 1 and dev['hosts'] == ['h1', 'h2']
    assert dev['metrics'] == {'success': pytest.approx((0.5 * 30 + 0.9 * 10) / 40)}
    assert dev['failures'][0]['reason'] == 'timed out without progress' and 'stuck' in dev['failures'][0]['traceback']
    assert (held['done'], held['total'], held['metrics']) == (2, 8, None)

    doc = monitor.snapshot(str(study), cluster=False)
    assert doc['schema'] == 'crane.monitor/1' and len(doc['groups']) == 2 and not doc['finished']
    lines = []
    out = output.Output(stdout=SimpleNamespace(write=lambda s: lines.append(s), flush=lambda: None))
    monitor.render(doc, out)
    text = ''.join(lines)
    assert 'held-out: progress only' in text and '0.777' not in text and 'success 0.6' in text


def _part(d, job, rows, study='toy01'):
    schema = {'seed': 'int', 'success': 'float'}
    w = RowWriter(d, schema, fmt='csv', part=0)
    for s in rows:
        w.write({'seed': s, 'success': s / 10})
    return write_manifest(d, schema, w.close(), {'study': study, 'job_id': job})


class FakeClient:
    def __init__(self, root):
        self.root, self.store = root, {}

    def add(self, job, files, attempt=1):
        for name, data in files.items():
            key = f'{job}/a{attempt}/{name}'
            self.store.setdefault(job, []).append({'name': name, 'key': key, 'run_id': f'crane/{job}.a{attempt}.s0'})
            self.store[key] = data

    def artifacts(self, job):
        return self.store.get(job, [])

    def get(self, key):
        return self.store[key]


def test_fetch_puts_box_and_cluster_outputs_in_the_local_layout(tmp_path):
    out, vast = tmp_path / 'studies', tmp_path / 'vast-out'
    spec = {'study': 'toy01', 'stage': 'A', 'arm': 'base', 'block': 'dev', 'cell': 'A.base.x',
            'cell_dir': 'A/base/A.base.x'}
    plan = {'study': 'toy01', 'project': 'crane', 'jobs': [
        {'job_id': f'j{i}', 'campaign': 'toy01', 'spec': {**spec, 'seeds': seeds}}
        for i, seeds in enumerate([[1, 2], [3, 4], [5], [6]])]}
    write_plan(out, 'toy01', plan['jobs'], 'crane')
    _part(vast / 'box7' / 'toy01' / 'j0', 'j0', [1, 2])                       # j0: a rented box's pull
    staged = tmp_path / 'staged'
    _part(staged, 'j1', [3, 4])                                              # j1: coordinator artifacts
    client = FakeClient(tmp_path)
    client.add('j1', {'artifacts/rows/manifest.json': (staged / 'manifest.json').read_bytes(),
                      'artifacts/rows/rows-0.csv': (staged / 'rows-0.csv').read_bytes(),
                      'metrics.json': b'{"rows": 2}'})
    client.add('j2', {'artifacts/rows/manifest.json': (staged / 'manifest.json').read_bytes(),   # j2: corrupt rows
                      'artifacts/rows/rows-0.csv': b'seed,success\n9,9\n'})
    rows = {r['job']: r for r in cf.fetch(plan, out, client, vast)}
    assert [rows[j]['status'] for j in ('j0', 'j1', 'j2', 'j3')] == ['fetched', 'fetched', 'failed', 'missing']
    assert rows['j0']['source'] == 'box box7' and rows['j1']['source'] == 'cluster' and 'sha256' in rows['j2']['source']
    cell = out / 'toy01' / 'A' / 'base' / 'A.base.x'
    man = json.loads((cell / 'manifest.json').read_text())
    assert man['schema'] == 'crane.data.manifest/1' and man['jobs'] == ['j0', 'j1'] and man['rows'] == 4
    assert sorted(r['seed'] for r in read_rows(cell)) == [1, 2, 3, 4]
    assert (cell / 'parts' / 'j1' / 'metrics.json').is_file() and not (cell / 'parts' / 'j2' / 'rows-0.csv').exists()
    again = {r['job']: r['status'] for r in cf.fetch(plan, out, client, vast)}          # idempotent
    assert again['j0'] == again['j1'] == 'fetched' and json.loads((cell / 'manifest.json').read_text()) == man
