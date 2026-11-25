"""crane.study/1: spec errors, common random numbers, the held-out guard, resume after a kill, adaptive stages and
the paired report."""

import io
import json
import math
import os
from pathlib import Path
import random
import signal
import subprocess
import sys
import textwrap
import time

import numpy as np
import pytest

from crane import cli, study
from crane.cache import UnitCache
from crane.data.rows import RowWriter, read_rows, write_manifest
from crane.study import exec as ex, plan, report as rp
from engine import exact as engine_exact

ROOT = Path(__file__).resolve().parents[1]
TASKS = '''\
import json, os, random, time


def noisy(args, seed, cell):
    """A seeded task: y = x + noise(seed); logs each call when CALLS is set."""
    if os.environ.get('CALLS'):
        with open(os.environ['CALLS'], 'a') as f:
            f.write(json.dumps({'arm': cell['arm'], 'seed': seed}) + '\\n')
    time.sleep(float(args.get('sleep', 0)))
    r = random.Random(seed)
    x = float(args.get('x', 0.0))
    return {'y': x + r.gauss(0, 1), 'x': x, 'neg_loss': -(x - 1.0) ** 2 + 0.01 * r.gauss(0, 1)}
'''


def write(tmp, name, text):
    path = Path(tmp) / name
    path.write_text(textwrap.dedent(text))
    return path


def task_spec(tmp, extra='', seeds="{dev: {role: dev, seeds: '0:6'}}"):
    """Two task arms (a: x=0, b: x=0.5) over seed blocks ``seeds`` (a YAML flow mapping), plus ``extra`` lines."""
    write(tmp, 'tasks.py', TASKS)
    return write(tmp, 'study.yaml', textwrap.dedent('''\
        name: tasks
        arms:
          a: {task: tasks.py:noisy, args: {x: 0.0}}
          b: {task: tasks.py:noisy, args: {x: 0.5}}
        ''') + f'seeds: {seeds}\n{extra}\n')


def git(where, *argv, env=None):
    subprocess.run(['git', '-C', str(where), *argv], check=True, capture_output=True,
                   env={**os.environ, 'GIT_AUTHOR_NAME': 't', 'GIT_AUTHOR_EMAIL': 't@t', 'GIT_COMMITTER_NAME': 't',
                        'GIT_COMMITTER_EMAIL': 't@t', **(env or {})})


def test_spec_errors_are_one_line_naming_the_key_and_the_fix(tmp_path, capsys):
    good = task_spec(tmp_path).read_text()
    cases = [
        (good.replace('arms:', 'armz:'), 'armz: unknown key'),
        (good.replace("role: dev", "role: devv"), "seeds.dev.role: 'devv'"),
        (good.replace("'0:6'", "'6:2'"), 'seeds.dev.seeds: seed range'),
        (good.replace('tasks.py:noisy, args', 'tasks.py, args'), 'arms.a.task'),
        (good.replace("'0:6'}}", "'0:6'}, test: {role: held-out, seeds: '50:59'}}"), 'targets: a held-out block'),
        (good.replace("'0:6'}}", "'0:6'}, test: {role: held-out, seeds: '5:9'}}") + 'targets: t.yaml\n',
         'seed 5 is also in held-out block test'),
        (good + 'stages: {V: {after: R}}\n', "stages.V.after: no stage 'R'"),
        (good + 'stages: {R: {after: V}, V: {after: R}}\n', 'a cycle'),
        (good + 'axes: {seed: {param: x, levels: [1, 2]}}\n', 'axes.seed: an axis name'),
        (good + 'tier: L1\n', 'L0-L7 are the boat control layers'),
    ]
    for text, want in cases:
        path = write(tmp_path, 'bad.yaml', text)
        with pytest.raises(study.StudyError) as caught:
            study.load(path)
        message = str(caught.value)
        assert want in message and '\n' not in message and message.startswith('bad.yaml: '), (want, message)
    assert cli.main(['study', 'plan', str(path)]) == 2               # the CLI: exit 2, one line on stderr
    err = capsys.readouterr().err.strip()
    assert err.startswith('crane study plan: bad.yaml: ') and '\n' not in err


def test_arms_pair_on_the_same_seed_in_every_cell(tmp_path):
    # toy at T0: `twin` is base under another name, so common random numbers make its rows equal to base's
    spec = write(tmp_path, 'toy.yaml', '''\
        name: crn
        domain: toy
        arms: {base: {}, twin: {}, far: {set: {adapter.options.reach_m: 2.5}}}
        axes: {fov: {param: perception.hfov_deg, levels: [60, 90]}}
        seeds: {dev: {role: dev, seeds: '0:3'}}
        ''')
    s = study.load(spec)
    out = tmp_path / 'out'
    summary = ex.run_study(s, out, max_workers=2)
    assert summary['ran'] == 18 and summary['failed'] == 0
    rows = list(read_rows(out))
    assert len(rows) == 18 and {r['arm'] for r in rows} == {'base', 'twin', 'far'}
    by_pair = {}
    for r in rows:
        by_pair.setdefault(r['pair'], {}).setdefault(r['arm'], {})[r['seed']] = r
    assert len(by_pair) == 2                                  # one pair group per fov level, across the 3 arms
    for arms in by_pair.values():
        assert {tuple(sorted(v)) for v in arms.values()} == {(0, 1, 2)}
        for seed, r in arms['base'].items():
            assert r['run_seed'] == arms['far'][seed]['run_seed'] == seed
            assert {k: v for k, v in r.items() if k not in ('arm', 'cell', 'wall_s')} == \
                   {k: v for k, v in arms['twin'][seed].items() if k not in ('arm', 'cell', 'wall_s')}
    # crn: derive gives each (seed, condition) its own stream seed, still the same across arms
    write(tmp_path, 'conds.csv', 'id,x\nc1,0.1\nc2,0.2\n')
    derived = study.load(task_spec(tmp_path, 'crn: derive\nconditions: {csv: conds.csv, id: id}'))
    run_seeds = {}
    for cell in derived.cells():
        run_seeds.setdefault(cell.condition['id'], set()).add(tuple(cell.run_seeds.items()))
    assert all(len(v) == 1 for v in run_seeds.values())       # arms a and b: one mapping per condition
    c1, c2 = (dict(next(iter(run_seeds[c]))) for c in ('c1', 'c2'))
    assert c1 != c2 and c1[0] != 0


def test_held_out_runs_only_with_confirm_and_an_older_committed_targets_file(tmp_path, capsys):
    repo = tmp_path / 'repo'
    repo.mkdir()
    git(repo, 'init', '-q')
    spec = task_spec(repo, 'targets: targets.yaml\npack: {unit_s: 0.01}\n'
                     'purpose: Held-out guard test study with two seeded task arms.',
                     seeds="{dev: {role: dev, seeds: '0:4'}, test: {role: held-out, seeds: '100:104'}}")
    s, out = study.load(spec), tmp_path / 'out'

    def run(*extra):
        code = cli.main(['study', 'run', str(spec), '--out', str(out), *extra])
        return code, capsys.readouterr()

    code, cap = run()                                         # dev runs; the held-out block waits
    assert code == 0 and 'held-out main:test skipped' in cap.out
    code, cap = run('--block', 'test')
    assert code == 2 and 'needs --confirm' in cap.err and len(cap.err.strip().splitlines()) == 1
    code, cap = run('--block', 'test', '--confirm')
    assert code == 2 and 'targets.yaml is missing' in cap.err
    (repo / 'targets.yaml').write_text('- {contrast: b - a, metric: y, direction: higher}\n')
    code, cap = run('--block', 'test', '--confirm')
    assert code == 2 and 'is not committed' in cap.err
    git(repo, 'add', '.')
    git(repo, 'commit', '-qm', 'targets', env={'GIT_COMMITTER_DATE': f'{int(time.time()) + 3600} +0000'})
    code, cap = run('--block', 'test', '--confirm')
    assert code == 2 and 'committed at or after now' in cap.err
    git(repo, 'commit', '-q', '--amend', '--no-edit', env={'GIT_COMMITTER_DATE': f'{int(time.time()) - 60} +0000'})
    status = rp.status(s, out)                                # before: held-out progress only, dev means only
    assert status['stages']['main']['blocks']['test'] == {'role': 'held-out', 'units': 8, 'done': 0}
    assert set(status['interim']['main']) == {'a', 'b'}
    assert rp.report(out)['held_out_waiting'] == {'main:test': {'done': 0, 'total': 8}}
    from crane import monitor                                 # crane monitor: dev means per arm, held-out progress
    groups = {g['block']: g for g in monitor.snapshot(str(out), cluster=False)['groups']}
    assert groups['test']['metrics'] is None and {'a.y', 'b.y'} <= set(groups['dev']['metrics'])
    (repo / 'targets.yaml').write_text('changed\n')
    code, cap = run('--block', 'test', '--confirm')
    assert code == 2 and 'holds no targets' in cap.err
    (repo / 'targets.yaml').write_text('- {contrast: b - a, metric: y, direction: lower}\n')
    code, cap = run('--block', 'test', '--confirm')
    assert code == 2 and 'uncommitted changes' in cap.err
    git(repo, 'checkout', '--', 'targets.yaml')
    code, cap = run('--block', 'test', '--confirm', '--json')
    doc = json.loads(cap.out)
    assert code == 0 and doc['touched_held_out'] and doc['ran'] == 8
    rows = list(read_rows(out))
    assert {r['touched_held_out'] for r in rows if r['role'] == 'held-out'} == {True}
    assert {r['touched_held_out'] for r in rows if r['role'] == 'dev'} == {False}
    manifest = json.loads((out / 'manifest.json').read_text())
    assert manifest['touched_held_out'] and manifest['held_out']['targets']['path'].endswith('targets.yaml')
    assert manifest['spec'] == 'study.yaml' and manifest['held_out']['targets']['path'] == 'targets.yaml'
    metrics, status = json.loads((out / 'metrics.json').read_text()), rp.status(s, out)
    for arm in 'ab':                                          # monitoring means: dev rows only, held-out never
        dev = [r['y'] for r in rows if r['arm'] == arm and r['role'] == 'dev']
        assert metrics[f'mean.main.{arm}.y'] == pytest.approx(sum(dev) / len(dev), abs=1e-5)
        assert status['interim']['main'][arm]['y']['n'] == len(dev)
    doc = rp.report(out)                                      # done: the held-out block enters, and so do targets
    assert not doc['held_out_waiting'] and {x['block'] for x in doc['pooled']} == {'dev', 'test'}
    assert [(t['contrast'], t['verdict']) for t in doc['targets']] == [('b - a', 'met')]
    (repo / 'targets.yaml').write_text('- {contrast: b - a, metric: y, direction: lower}\n')
    git(repo, 'commit', '-qam', 'move', env={'GIT_COMMITTER_DATE': f'{int(time.time()) - 30} +0000'})
    code, cap = run('--block', 'test', '--confirm')           # targets moved after held-out outcomes existed
    assert code == 2 and 'changed since' in cap.err
    git(repo, 'revert', '--no-edit', 'HEAD', env={'GIT_COMMITTER_DATE': f'{int(time.time()) - 20} +0000'})

    # a rented box: the spec and its files, no git. compile checked git here and pinned the targets sha256
    from crane.cluster import compile as cc
    from crane_backends.batch import Runner
    pins = {'crane_ml': {'name': 'crane_ml', 'url': str(ROOT), 'ref': 'a' * 40},
            '_spec': {'repo': 'study', 'path': 'study.yaml'}}
    jobs = cc.compile(s, block='test', pins=pins)
    assert not jobs and jobs.skipped and '--confirm' in jobs.skip_reason
    jobs = cc.compile(s, block='test', pins=pins, confirm=True)
    job = jobs[0]
    assert job['schema'] == 'rc-job/1' and job['kind'] == 'crane_batch' and job['est_seconds'] >= 1
    assert job['spec']['targets']['sha256'] and job['spec']['touched_held_out']
    box = tmp_path / 'box'
    box.mkdir()
    for name in ('study.yaml', 'tasks.py', 'targets.yaml'):
        (box / name).write_bytes((repo / name).read_bytes())
    argv = Runner(job['spec'], tmp_path / 'run').argv(box / 'study.yaml', tmp_path / 'job')[3:]   # crane-batch's
    assert argv[:2] == ['study', 'run-cell'] and '--targets-sha256' in argv                       # run-cell line
    assert cli.main(argv) == 0, capsys.readouterr().err
    got = json.loads((tmp_path / 'job' / 'manifest.json').read_text())
    assert got['held_out']['targets']['pinned'] and got['rows'] == len(job['spec']['seeds'])
    capsys.readouterr()
    bare = argv[:argv.index('--targets-sha256')]
    assert cli.main([*bare, '--out', str(tmp_path / 'job2')]) == 2   # no pin: git, which the box lacks
    assert 'not in a git repository' in capsys.readouterr().err
    assert cli.main([*bare[:-1], '--out', str(tmp_path / 'job2')]) == 2   # no --confirm: the run-cell fix
    assert 'fix: crane study run-cell' in capsys.readouterr().err
    (box / 'targets.yaml').write_text('- {contrast: b - a, metric: y, direction: lower}\n')
    assert cli.main([*argv, '--out', str(tmp_path / 'job3')]) == 2
    assert 'but the job pinned' in capsys.readouterr().err


def test_a_killed_run_resumes_with_only_the_missing_units(tmp_path):
    spec = task_spec(tmp_path, seeds="{dev: {role: dev, seeds: '0:40'}}")
    text = spec.read_text().replace('args: {x: 0.0}', 'args: {x: 0.0, sleep: 0.05}').replace(
        'args: {x: 0.5}', 'args: {x: 0.5, sleep: 0.05}')
    spec.write_text(text)
    out, calls = tmp_path / 'out', tmp_path / 'calls.jsonl'
    env = {**os.environ, 'CALLS': str(calls), 'PYTHONPATH': str(ROOT)}
    argv = [str(ROOT / 'bin' / 'crane'), 'study', 'run', str(spec), '--out', str(out), '--max-workers', '2']
    proc = subprocess.Popen(argv, env=env, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL,
                            start_new_session=True)
    records = out / 'cache' / 'records.jsonl'
    deadline = time.time() + 60
    while time.time() < deadline and (not records.is_file() or len(records.read_bytes().splitlines()) < 20):
        time.sleep(0.05)
    os.killpg(proc.pid, signal.SIGKILL)                       # the run and its workers, mid-run
    proc.wait(10)
    cached = {(e['unit']['arm'], e['unit']['seed']) for e in UnitCache(out / 'cache')}
    assert 20 <= len(cached) < 80
    calls.write_text('')
    done = subprocess.run([*argv, '--json'], env=env, capture_output=True, text=True, timeout=120)
    assert done.returncode == 0, done.stderr
    summary = json.loads(done.stdout)
    second = [json.loads(x) for x in calls.read_text().splitlines()]
    assert summary['reused'] == len(cached) and summary['ran'] == 80 - len(cached)
    assert len(second) == 80 - len(cached) and not {(c['arm'], c['seed']) for c in second} & cached
    assert len(list(read_rows(out))) == 80 and len(UnitCache(out / 'cache')) == 80
    assert json.loads((out / 'progress.json').read_text())['done'] == 80


def test_an_adaptive_stage_selects_the_top_k_of_the_stage_it_reads(tmp_path, capsys):
    spec = task_spec(tmp_path)
    text = spec.read_text().split('arms:')[0] + textwrap.dedent('''\
        arms:
          r: {task: tasks.py:noisy, sample: {n: 6, seed: 3, set: {x: {uniform: [-1.0, 3.0]}}}}
        seeds:
          dev: {role: dev, seeds: '0:4'}
          check: {role: check, seeds: '4:7'}
        stages:
          R: {blocks: [dev]}
          V: {after: R, select: {top_k: 2, metric: neg_loss}, blocks: [check], tier: T1}
        ''')
    spec.write_text(text)
    s, out = study.load(spec), tmp_path / 'out'
    assert list(s.arms) == [f'r-{i}' for i in range(6)]
    assert {c.stage for c in s.cells()} == {'R'} and set(plan.pending(s)) == {'V'}
    with pytest.raises(plan.StageNotReady, match='stage R'):
        s.cells(stage='V', out=out)
    assert ex.run_study(s, out, stage='V')['refused'].endswith('--stage R')
    summary = ex.run_study(s, out)                            # R, then V from R's rows
    assert summary['stages']['R']['ran'] == 24 and summary['stages']['V']['ran'] == 6
    best = sorted(s.arms, key=lambda a: (s.arms[a].set['x'] - 1.0) ** 2)[:2]
    v = s.cells(stage='V', out=out)
    assert {c.arm for c in v} == set(best) and {c.tier for c in v} == {'T1'} and {c.block for c in v} == {'check'}
    assert cli.main(['study', 'promote', str(spec), '--stage', 'V', '--out', str(out), '--json']) == 0
    promoted = study.load(json.loads(capsys.readouterr().out)['written'])
    assert promoted.stages['V'].arms == [a for a, _ in sorted(((a, (s.arms[a].set['x'] - 1) ** 2) for a in best),
                                                              key=lambda t: t[1])]
    assert {c.id for c in promoted.cells(stage='V')} == {c.id for c in v}  # the same cells, so the cache is shared
    assert plan.cached_rows(promoted.cells(stage='V'), out).keys() == {c.id for c in v}


def test_report_finds_a_synthetic_paired_effect_and_holm_adjusts_across_cells(tmp_path):
    rng = random.Random(5)
    rows = []
    for cell in range(4):
        for seed in range(16):
            world = rng.gauss(0, 2)                           # the seed's own difficulty, shared by every arm
            for arm, effect in (('a', 0.0), ('b', 0.5), ('c', 0.0)):
                rows.append({'study': 's', 'stage': 'main', 'arm': arm, 'cell': f'{arm}{cell}', 'pair': f'p{cell}',
                             'block': 'dev', 'split': None, 'role': 'dev', 'tier': 'T0', 'seed': seed,
                             'run_seed': seed, 'condition': None, 'level': f'l{cell}', 'outcome': 'ok',
                             'error': None, 'y': world + effect + rng.gauss(0, 0.2), 'wall_s': 0.1, 'commit': 'x',
                             'card_sha': None, 'touched_held_out': False})
    doc = rp.analyse(rows, 'a', ['y'], axes=['level'], B=1000)
    pooled = {x['arm']: x for x in doc['pooled']}
    b, c = pooled['b'], pooled['c']
    assert b['seeds'] == 16 and b['cells'] == 4 and abs(b['diff'] - 0.5) < 0.1
    assert b['ci'][0] > 0.3 and b['p'] < 0.001 and b['cells_holm_below_alpha'] == 4
    assert c['ci'][0] < 0 < c['ci'][1] and c['p'] > 0.05
    cells = [x for x in doc['cells'] if x['arm'] == 'b']
    assert len(cells) == 4 and all(x['p_holm'] >= x['p'] for x in cells) and cells[0]['axes'] == {'level': 'l0'}
    # the same through rows on disk and `crane study report` (a held-out block still running stays out)
    out = tmp_path / 'out'
    schema = {k: ('int' if k in ('seed', 'run_seed') else 'bool' if k == 'touched_held_out' else 'float'
                  if k in ('y', 'wall_s') else 'str') for k in rows[0]}
    w = RowWriter(out / 'rows', schema, part='main')
    for r in rows + [{**rows[0], 'role': 'held-out', 'block': 'test', 'y': 99.0}]:
        w.write(r)
    files = [{**f, 'path': 'rows/' + f['path']} for f in w.close()]
    write_manifest(out, schema, files, {'study': 's', 'baseline': 'a', 'axes': ['level'],
                                        'units': {'main': {'dev': {'role': 'dev', 'done': 192, 'total': 192},
                                                           'test': {'role': 'held-out', 'done': 1, 'total': 9}}}})
    assert cli.main(['study', 'report', str(out), '--metric', 'y', '--json']) == 0
    report = json.loads((out / 'report.json').read_text())
    assert report['held_out_waiting'] == {'main:test': {'done': 1, 'total': 9}}
    assert {x['arm']: round(x['diff'], 6) for x in report['pooled']} == {k: round(v['diff'], 6)
                                                                         for k, v in pooled.items()}
    assert '| main | dev | b | y |' in (out / 'report.md').read_text()


if __name__ == '__main__':
    sys.exit(pytest.main([__file__, '-q']))


# ---- the batched engine, plugins, card axes, population scorers, censoring, session splits ---------------------

def test_a_batched_engine_cell_gives_the_scalar_rows(tmp_path, capsys):
    """Boat T1 cells on the batched engine (numpy, float64: numpy alone) and on the scalar path, over two derived
    cards and two condition rows (one cap the course fits in, one it does not): the same rows, events and decimated
    track; successes observed, timeouts censored at the cap. A record level the engine cannot serve is refused."""
    write(tmp_path, 'conds.csv', '''\
        id,time_limit_s
        short,20
        long,300
        ''')
    spec = write(tmp_path, 'boat.yaml', '''\
        name: engine_rows
        domain: boat
        tier: T1
        arms:
          scalar: {}
          batched: {engine: batched, backend: numpy}
        card: ideal
        card_axes:
          lag: {fields: [thrusters.*.tauT], levels: {pre: base, slow: {values: {thrusters.*.tauT: 0.3}}}}
        conditions: {csv: conds.csv, id: id, set: {time_limit_s: time_limit_s}}
        seeds: {dev: {role: dev, seeds: '0:2'}}
        record: {level: traj_decimated, hz: 2}
        ''')
    s, out = study.load(spec), tmp_path / 'out'
    assert {c.arm: ex.engine_label(c) for c in s.cells()} == {'scalar': 'scalar', 'batched': 'batched/numpy/float64'}
    assert len({c.pair for c in s.cells()}) == 4 and len({c.id for c in s.cells()}) == 8   # pair, own unit keys
    assert ex.run_study(s, out)['ran'] == 16
    rows = {(r['arm'], r['card_sha'], r['cond.id'], r['seed']): r for r in read_rows(out)}
    cards = {k[1] for k in rows}
    assert len(cards) == 2
    exact = engine_exact.openblas()    # bit for bit on OpenBLAS; on another BLAS the scalar path rounds differently
    for (arm, card, cond, seed), a in rows.items():
        if arm != 'scalar':
            continue
        b = rows['batched', card, cond, seed]
        differ = {k for k in a if a[k] != b[k] and not (not exact and isinstance(a[k], float)
                                                         and math.isclose(a[k], b[k], rel_tol=1e-9, abs_tol=1e-9))}
        assert differ == {'arm', 'cell', 'wall_s', 'engine'}, (card, cond, seed)
        assert all(v == v for v in a.values() if isinstance(v, float))                       # never NaN
        if cond == 'short':                                   # cut off at the cap, time_limit_s + grace_s
            assert a['censored'] is True and a['time_cap_s'] == 25.0 and a['outcome'] == 'TIMEOUT'
        else:
            assert a['censored'] is False and a['outcome'] == 'SUCCEEDED' and a['run.sim_s'] < 300
    by_card = {c: sorted(r['run.sim_s'] for k, r in rows.items() if k[1] == c and k[2] == 'long') for c in cards}
    assert len({tuple(v) for v in by_card.values()}) == 2           # each derived card drives its own boat
    cache, pair_of, blobs = UnitCache(out / 'cache'), {c.key['cell']: c.pair for c in s.cells()}, {}
    for e in cache:
        blobs.setdefault((pair_of[e['unit']['cell']], e['unit']['seed']), []).append(e['record']['blobs'])
    assert len(blobs) == 8
    for one, other in blobs.values():
        assert set(one) == set(other) == {'traj', 'events'} and one['events'] == other['events']
        if exact:
            assert one['traj'] == other['traj']
        else:
            t1, t2 = (np.load(io.BytesIO(cache.get_blob(x['traj'], '.npz'))) for x in (one, other))
            assert all(np.allclose(t1[k], t2[k], atol=1e-4) for k in ('t', 'x', 'y', 'yaw'))
    write(tmp_path, 'obs.yaml', spec.read_text().replace('level: traj_decimated', 'level: obs'))
    with pytest.raises(study.StudyError) as caught:
        study.load(tmp_path / 'obs.yaml')
    assert "record: level 'obs' is not served by the batched engine" in str(caught.value)
    assert cli.main(['study', 'plan', str(spec)]) == 0
    assert 'engine: stage main: scalar 4 cells; batched/numpy/float64 4 cells' in capsys.readouterr().out


PLUGIN_TASK = '''\
import json


def episode(args, seed, cell):
    """Time to capture = distance / (surge gain), capped: past the cap the unit is censored at it."""
    card = json.loads(open(args['card']).read())
    speed = card['thrusters'][0]['kf'] * card['surge']['scale'] * (1.0 - card['deadband'])
    t = 10.0 / speed + args.get('latcomp_s', 0.0) + 0.01 * (seed % 3)
    cap = args['time_limit_s']
    if t > cap:
        return {'censored': True, 'time_cap_s': cap, 'time_s': cap}
    return {'time_s': t, 'time_cap_s': cap}


def parts(result, cell):
    return {'fast': 1.0 if result['metrics'].get('time_s', 99) < 8 else 0.0}
'''


def test_plugins_card_axes_and_censoring(tmp_path, capsys):
    """A plugin dir on the study's sys.path, its git state in the manifest; 5 binary card axes x a 3-level surge
    scale x latcomp = 192 cells, each derived card written once and its sha on its rows; censored units carry
    the flag and the cap, never NaN."""
    repo = tmp_path / 'repo'
    (repo / 'plug').mkdir(parents=True)
    git(repo, 'init', '-q')
    write(repo / 'plug', 'plug_task.py', PLUGIN_TASK)
    (repo / 'base.json').write_text(json.dumps({'thrusters': [{'kf': 1.0, 'kr': 1.0}] * 4, 'deadband': 0.0,
                                                'delay_s': 0.0, 'izz': 4.0, 'surge': {'scale': 1.0, 'd1': 10.0}}))
    (repo / 'measured.json').write_text(json.dumps({'thrusters': [{'kf': 0.8, 'kr': 0.7}] * 4}))
    spec = write(repo, 'cards.yaml', '''\
        name: cards
        plugins: [plug]
        arms: {a: {task: 'plug_task:episode', args: {time_limit_s: 12.0}}}
        scorer: 'plug_task:parts'
        card: base.json
        card_axes:
          thrust: {fields: [thrusters.*.kf, thrusters.*.kr], levels: {pre: base, measured: measured.json}}
          deadband: {fields: [deadband], levels: {pre: base, doc: 0.05}}
          delay: {fields: [delay_s], levels: {pre: base, measured: {values: {delay_s: 0.3}}}}
          yaw: {fields: [izz], levels: {pre: base, measured: 5.2}}
          slow: {fields: [surge.scale], levels: {pre: base, half: {scale: 0.5}}}
          surge: {fields: [surge.scale, surge.d1], transform: scale, levels: [0.5, 1.0, 2.0]}
        axes: {latcomp: {param: latcomp_s, levels: [0.0, 0.2]}}
        seeds: {dev: {role: dev, seeds: '0:1'}}
        ''')
    git(repo, 'add', '.')
    git(repo, 'commit', '-qm', 'plugin')
    s, out = study.load(spec), tmp_path / 'out'
    assert s.card_count() == 96 and len(s.cells()) == 192
    write(repo / 'plug', 'plug_task.py', PLUGIN_TASK + '\n# an edit\n')        # dirty: recorded, and it reruns
    s = study.load(spec)
    assert cli.main(['study', 'plan', str(spec)]) == 0
    assert 'cards: 96 derived' in capsys.readouterr().out
    assert ex.run_study(s, out)['ran'] == 192
    rows = list(read_rows(out))
    assert len(rows) == 192 and len({r['card_sha'] for r in rows}) == 96
    from crane.ids import sha256
    for r in rows[:20]:
        path = out / 'cards' / f"{r['card_sha']}.json"
        assert sha256(path.read_bytes()) == r['card_sha']
    one = next(r for r in rows if (r['thrust'], r['deadband'], r['slow'], r['surge']) == ('measured', 'doc', 'half',
                                                                                          'x2'))
    card = json.loads((out / 'cards' / f"{one['card_sha']}.json").read_text())
    assert card['thrusters'][2] == {'kf': 0.8, 'kr': 0.7} and card['deadband'] == 0.05
    assert card['surge'] == {'scale': 1.0, 'd1': 20.0} and card['crane_derived']['levels']['surge'] == 'x2'
    censored = [r for r in rows if r['censored']]
    assert censored and all(r['time_cap_s'] == 12.0 and r['time_s'] == 12.0 for r in censored)
    assert all(v == v for r in rows for v in r.values() if isinstance(v, float))
    manifest = json.loads((out / 'manifest.json').read_text())
    [plug] = manifest['plugins']
    assert plug['path'] == 'plug' and len(plug['commit']) == 40 and plug['dirty'] and plug['diff_sha']
    assert len(manifest['cards']) == 96


def test_population_scorers_rank_cards_with_a_session_held_out_split(tmp_path, capsys):
    """Conditions split by session (s5, s6 held out); a population scorer per card x block; the report ranks the
    cards on calib and gives Spearman vs held-out with a session-cluster bootstrap CI. The held-out sessions run
    only with --confirm and committed targets."""
    repo = tmp_path / 'repo'
    repo.mkdir()
    git(repo, 'init', '-q')
    write(repo, 'pop.py', '''\
        import random


        def ep(args, seed, cell):
            """y = the card's gain times the condition's level, plus session-shared and per-seed noise."""
            import json
            g = json.loads(open(args['card']).read())['gain']
            r = random.Random(seed * 1000 + hash(cell['condition']['session_id']) % 997)
            return {'y': g * args['level'] + 0.05 * r.gauss(0, 1)}


        def fit(rows, group):
            errs = [abs(r['y'] - r['cond.observed']) for r in rows]
            return {'loss': sum(errs) / len(errs), 'n': len(rows)}
        ''')
    (repo / 'base.json').write_text('{"gain": 1.0}')
    lines = ['id,session_id,level,observed']
    for i in range(12):
        lines.append(f'c{i:02d},s{i // 2 + 1},{1 + i % 3},{1.1 * (1 + i % 3):.3f}')
    (repo / 'conds.csv').write_text('\n'.join(lines) + '\n')
    spec = write(repo, 'pop.yaml', '''\
        name: pop
        arms: {a: {task: pop.py:ep}}
        card: base.json
        card_axes: {gain: {fields: [gain], levels: {g08: 0.8, g10: base, g11: 1.1, g13: 1.3, g20: 2.0}}}
        conditions: {csv: conds.csv, id: id, set: {level: level}}
        splits:
          column: session_id
          groups: {test: [s5, s6], calib: '*'}
          roles: {calib: calib, test: held-out}
        seeds: {dev: {role: dev, seeds: '0:3'}}
        targets: targets.yaml
        scorers: [{fn: pop.py:fit, over: card x block}]
        report: {population: {part: fit.loss, order: min, cluster: cond.session_id, B: 100}}
        ''')
    s, out = study.load(spec), tmp_path / 'out'
    assert {c.split for c in s.cells()} == {'calib', 'test'}
    assert {c.condition['session_id'] for c in s.cells() if c.held_out} == {'s5', 's6'}

    def run(*extra):
        return cli.main(['study', 'run', str(spec), '--out', str(out), *extra]), capsys.readouterr()
    code, cap = run()
    assert code == 0 and 'held-out main:test skipped' in cap.out
    code, cap = run('--block', 'test')
    assert code == 2 and 'held-out test needs --confirm' in cap.err and len(cap.err.strip().splitlines()) == 1
    doc = rp.report(out, study=s)
    rk = doc['population']['ranking']
    assert [t['label'] for t in rk['cards']][0] == 'gain=g11' and rk['spearman'] is None and 'ranking only' in rk['why']
    (repo / 'targets.yaml').write_text('- {contrast: a - a, metric: y, direction: not lower}\n')
    git(repo, 'add', '.')
    git(repo, 'commit', '-qm', 'targets', env={'GIT_COMMITTER_DATE': f'{int(time.time()) - 60} +0000'})
    code, cap = run('--block', 'test', '--confirm')
    assert code == 0, cap.err
    assert cli.main(['study', 'report', str(out)]) == 0
    text = capsys.readouterr().out
    assert 'cards by fit.loss (min first on calib): Spearman' in text and 'cond.session_id clusters' in text
    doc = json.loads((out / 'report.json').read_text())
    rk = doc['population']['ranking']
    assert len(rk['cards']) == 5 and all(t['test'] is not None for t in rk['cards'])
    assert rk['spearman'] > 0.8 and rk['ci'][0] <= rk['spearman'] <= rk['ci'][1] and rk['clusters'] == 6
    groups = doc['population']['groups']
    assert {(g['key']['block'], g['role']) for g in groups} == {('dev/calib', 'calib'), ('dev/test', 'held-out')}
    from judge import stats                        # the cluster bootstrap resamples sessions, not rows
    units = list(range(40))
    clusters = [u // 10 for u in units]
    vals = {u: float(u // 10) for u in units}
    _, (lo, hi) = stats.cluster_bootstrap_ci(units, clusters, lambda xs: sum(vals[x] for x in xs) / len(xs), B=400)
    _, (blo, bhi) = stats.bootstrap_ci(units, lambda xs: sum(vals[x] for x in xs) / len(xs), B=400)
    assert hi - lo > 1.5 * (bhi - blo)


# ---- numeric held-out targets, designs, batch task callables, sampled events -----------------------------------

def _paired_rows(effects, cells=4, seeds=30, noise=None):
    """Synthetic held-out rows: each seed's world w is shared by every arm (common random numbers); arm value
    = w + effect (a number, or one per cell) + its own noise (``noise[arm]``, default 0.02)."""
    rng = random.Random(11)
    rows = []
    for cell in range(cells):
        for seed in range(seeds):
            w = 5.0 + rng.gauss(0, 1)
            for arm, effect in effects.items():
                e = effect[cell] if isinstance(effect, list) else effect
                rows.append({'stage': 'main', 'pair': f'p{cell}', 'block': 'test', 'split': None, 'role': 'held-out',
                             'arm': arm, 'seed': seed, 'y': w + e + rng.gauss(0, (noise or {}).get(arm, 0.02))})
    return rows


def test_numeric_targets_give_met_not_met_and_inconclusive_on_paired_data(tmp_path):
    # F fixed, T treated (closes 0.8 of the gap to R), R reference, N noisy, Z: T's effect in 3 of 4 cells
    rows = _paired_rows({'F': 1.0, 'T': 0.2, 'R': 0.0, 'N': 0.6, 'Z': [0.2, 0.2, 0.2, 1.0]}, noise={'N': 3.0})
    s = study.load(task_spec(tmp_path).parent / write(tmp_path, 's.yaml', '''\
        name: kinds
        arms: {F: {task: tasks.py:noisy}, T: {task: tasks.py:noisy}, R: {task: tasks.py:noisy},
               N: {task: tasks.py:noisy}, Z: {task: tasks.py:noisy}}
        seeds: {dev: {role: dev, seeds: '0:2'}}
        '''))
    con = '{contrast: %s - F, metric: y, %s}'
    cases = {
        'threshold': [(con % ('T', 'expect: {threshold: -0.5, direction: below}'), 'met'),
                      (con % ('T', 'expect: {threshold: -0.9, direction: below}'), 'not met'),
                      (con % ('N', 'expect: {threshold: 0.0, direction: below}'), 'inconclusive')],
        'interval': [(con % ('T', 'expect: {interval: [-1.0, -0.6]}'), 'met'),
                     (con % ('T', 'expect: {interval: [0.0, 1.0]}'), 'not met'),
                     (con % ('N', 'expect: {interval: [-0.1, 0.1]}'), 'inconclusive')],
        'relative': [(con % ('T', 'relative: true, expect: {interval: [-0.2, -0.1]}'), 'met'),   # -0.8 / 6
                     (con % ('T', 'relative: true, expect: {threshold: -0.2, direction: below}'), 'not met'),
                     (con % ('N', 'relative: true, expect: {threshold: 0.0, direction: above}'), 'inconclusive')],
        'gap_closed': [('{gap_closed: {fixed: F, treated: T, reference: R}, metric: y, min: 0.7}', 'met'),
                       ('{gap_closed: {fixed: F, treated: T, reference: R}, metric: y, min: 0.9}', 'not met'),
                       ('{gap_closed: {fixed: F, treated: N, reference: R}, metric: y, min: 0.3}', 'inconclusive')],
        'share': [('{share: {condition: {contrast: Z - F, metric: y, direction: lower}, at_least: 0.75}}', 'met'),
                  ('{share: {condition: {contrast: Z - F, metric: y, direction: higher}, at_least: 0.5}}', 'not met'),
                  ('{share: {condition: {contrast: Z - F, metric: y, direction: lower}, at_least: 1.0}}',
                   'inconclusive')],
    }
    from crane.study import guard
    for kind, items in cases.items():
        path = write(tmp_path, 'targets.yaml', ''.join(f'- {t}\n' for t, _ in items))
        entries, reason = guard.read_targets(s, path)
        assert reason is None, reason
        got = rp.verdicts(entries, [], rows, B=500)
        assert [t['verdict'] for t in got] == [v for _, v in items], (kind, got)
    gap = rp.verdicts(guard.read_targets(s, write(tmp_path, 'g.yaml', '- ' + cases['gap_closed'][0][0]))[0],
                      [], rows, B=500)[0]['lines'][0]
    assert abs(gap['diff'] - 0.8) < 0.05 and gap['seeds'] == 30 and gap['ci'][0] > 0.7
    share = rp.verdicts(guard.read_targets(s, write(tmp_path, 'h.yaml', '- ' + cases['share'][0][0]))[0], [], rows,
                        B=500)[0]['lines'][0]
    assert share['cells'] == 4 and share['diff'] == 0.75
    assert rp.verdicts(entries, [], [{**r, 'role': 'dev'} for r in rows])[0]['verdict'] == 'no data'   # held-out only
    for bad, want in (('{contrast: T - F, metric: y, expect: {interval: [1, 0]}}', 'expect'),
                      ('{gap_closed: {fixed: F, treated: T}, metric: y, min: 0.7}', 'gap_closed'),
                      ('{share: {condition: {contrast: Q - F, metric: y, direction: lower}, at_least: 0.8}}', "'Q'")):
        _, reason = guard.read_targets(s, write(tmp_path, 'bad.yaml', f'- {bad}\n'))
        assert want in reason and '\n' not in reason


TARGETS_FN = '''\
def verdicts(rows):
    ys = {}
    for r in rows:
        ys.setdefault(r['arm'], []).append(r['y'])
    mean = {a: sum(v) / len(v) for a, v in ys.items()}
    return {'b_above_a': 'met' if mean['b'] > mean['a'] else 'not met', 'n': {'verdict': 'met', 'rows': len(rows)}}
'''


def test_a_targets_fn_is_pinned_like_the_targets_file_and_refused_when_dirty(tmp_path, capsys):
    from crane.ids import sha256
    from crane.study import guard
    repo = tmp_path / 'repo'
    repo.mkdir()
    git(repo, 'init', '-q')
    spec = task_spec(repo, 'targets: targets.yaml', seeds="{dev: {role: dev, seeds: '0:4'}, "
                     "test: {role: held-out, seeds: '100:106'}}")
    (repo / 'targets.yaml').write_text('- {targets_fn: judge_fn.py:verdicts, name: fn}\n'
                                       '- {contrast: b - a, metric: y, expect: {threshold: 0.0, direction: above}}\n')
    git(repo, 'add', 'study.yaml', 'tasks.py', 'targets.yaml')
    git(repo, 'commit', '-qm', 'targets', env={'GIT_COMMITTER_DATE': f'{int(time.time()) - 120} +0000'})
    s, out = study.load(spec), tmp_path / 'out'
    assert 'targets_fn file judge_fn.py is missing' in guard.targets_state(s)[1]
    write(repo, 'judge_fn.py', TARGETS_FN)
    assert 'targets_fn file judge_fn.py is not committed' in guard.targets_state(s)[1]
    git(repo, 'add', 'judge_fn.py')
    git(repo, 'commit', '-qm', 'fn', env={'GIT_COMMITTER_DATE': f'{int(time.time()) - 60} +0000'})
    state, reason = guard.targets_state(s)
    fn_sha = sha256((repo / 'judge_fn.py').read_bytes())
    assert reason is None and state['entries'][0]['sha256'] == fn_sha and len(state['entries'][0]['commit']) == 40
    assert state['sha256'] != sha256((repo / 'targets.yaml').read_bytes())      # the pin covers the script too
    assert guard.pinned_state(s, state['sha256'])[1] is None
    (repo / 'judge_fn.py').write_text(TARGETS_FN + '# edited\n')                  # dirty: refused, and the pin moves
    assert 'judge_fn.py has uncommitted changes' in guard.targets_state(s)[1]
    assert 'but the job pinned' in guard.pinned_state(s, state['sha256'])[1]
    assert cli.main(['study', 'run', str(spec), '--out', str(out), '--confirm']) == 2
    assert 'judge_fn.py has uncommitted changes' in capsys.readouterr().err
    git(repo, 'checkout', '--', 'judge_fn.py')
    assert cli.main(['study', 'run', str(spec), '--out', str(out), '--confirm']) == 0
    capsys.readouterr()
    doc = rp.report(out, study=s, B=300)
    got = {t['contrast']: t for t in doc['targets']}
    assert got['fn:b_above_a']['verdict'] == 'met' and got['fn:n']['detail'] == {'rows': 12}   # held-out rows only
    assert got['b - a']['verdict'] == 'met' and got['b - a']['direction'] == 'above 0'
    manifest = json.loads((out / 'manifest.json').read_text())
    assert manifest['held_out']['targets']['entries'][0]['sha256'] == fn_sha
    (repo / 'judge_fn.py').write_text(TARGETS_FN.replace("'met' if", "'not met' if"))   # not the pinned script
    got = rp.report(out, study=s, B=300)['targets'][0]
    assert got['verdict'] == 'no data' and 'not the pinned script' in got['why']


ECHO = '''\
import json, os, random


def echo(args, seed, cell):
    """The args it got, as numbers, plus a seeded y; logs each call when CALLS is set."""
    if os.environ.get('CALLS'):
        with open(os.environ['CALLS'], 'a') as f:
            f.write(json.dumps({'arm': cell['arm'], 'seed': seed}) + '\\n')
    r = random.Random(seed)
    w = args.get('world', {})
    return {'y': float(w.get('x', 0)) + r.gauss(0, 1), 'gain': float(args.get('axes', {}).get('gain', -1)),
            'x': float(w.get('x', -1)), 'z': float(w.get('z', -1)), 'events': [{'k': i} for i in range(3)]}


def echo_batch(args, seeds, cell):
    if os.environ.get('STOP') and os.path.exists(os.environ['STOP']) and seeds[0] >= 10:
        raise KeyboardInterrupt
    return [echo(args, s, cell) for s in seeds]
'''


def test_oat_and_lhs_designs_and_condition_columns_as_task_args(tmp_path):
    write(tmp_path, 'echo.py', ECHO)
    write(tmp_path, 'conds.csv', 'id,gain\nc1,0.5\nc2,2.0\n')
    oat = write(tmp_path, 'oat.yaml', '''\
        name: oat
        arms: {a: {task: echo.py:echo}, b: {task: echo.py:echo, args: {k: 1}}}
        design:
          oat: {base: {x: 1.0, y: 0, z: 3}, axes: {x: [0.5, 1.0, 2.0], z: [3, 4]}}
          param: {x: world.x, z: world.z}
        conditions: {csv: conds.csv, id: id, set: {gain: axes.gain}}
        seeds: {dev: {role: dev, seeds: '0:3'}}
        ''')
    s = study.load(oat)
    cells = s.cells()
    points = {(c.design['x'], c.design['y'], c.design['z']) for c in cells}
    assert points == {(1.0, 0, 3), (0.5, 0, 3), (2.0, 0, 3), (1.0, 0, 4)}                  # base + 2 + 1
    assert len(cells) == 4 * 2 * 2                                                          # x 2 arms x 2 conditions
    for c in cells:
        assert c.axes == {k: str(v) for k, v in c.design.items()}
        assert plan.nest(c.params)['world'] == {'x': c.design['x'], 'z': c.design['z']} and c.params['y'] == 0
    pairs = {}
    for c in cells:
        pairs.setdefault(c.pair, set()).add(c.arm)
    assert len(pairs) == 8 and all(v == {'a', 'b'} for v in pairs.values())               # CRN: arms share a pair
    # a design point hashes as the factorial cell of the same values would: the same unit keys, the cache shared
    fact = write(tmp_path, 'fact.yaml', oat.read_text().split('design:')[0] + textwrap.dedent('''\
        axes: {x: {param: world.x, levels: [0.5]}, y: {param: y, levels: [0]}, z: {param: world.z, levels: [3]}}
        conditions: {csv: conds.csv, id: id, set: {gain: axes.gain}}
        seeds: {dev: {role: dev, seeds: '0:3'}}
        ''').replace('name: oat', 'name: oat'))
    want = {(c.arm, c.condition['id']): (c.key, c.pair) for c in study.load(fact).cells()}
    assert {(c.arm, c.condition['id']): (c.key, c.pair) for c in cells if c.design['x'] == 0.5} == want
    out = tmp_path / 'out'
    assert ex.run_study(s, out)['ran'] == 48
    rows = list(read_rows(out))
    assert {(r['cond.id'], r['gain']) for r in rows} == {('c1', 0.5), ('c2', 2.0)}         # CSV column -> axes.gain
    assert all(r['x'] == r['x'] and isinstance(r['x'], float) and r['z'] in (3.0, 4.0) for r in rows)
    # LHS: deterministic from the seed, one point per stratum of each range, categories spread evenly
    lhs = oat.read_text().replace(
        "oat: {base: {x: 1.0, y: 0, z: 3}, axes: {x: [0.5, 1.0, 2.0], z: [3, 4]}}",
        "lhs: {n: 20, seed: 4, ranges: {x: [0.0, 2.0], z: {values: [a, b, c, d]}}}")

    def points(text):
        st = study.load(write(tmp_path, 'lhs.yaml', text))
        return st.design
    one, again = points(lhs), points(lhs)
    assert one == again and len(one) == 20
    assert points(lhs.replace('seed: 4', 'seed: 5')) != one
    assert sorted(int(p['x'] / 2.0 * 20) for p in one) == list(range(20))                  # one per stratum
    assert sorted(sum(p['z'] == v for p in one) for v in 'abcd') == [5, 5, 5, 5]
    wider = points(lhs.replace('z: {values: [a, b, c, d]}', 'z: {values: [a, b, c, d]}, w: [0, 1]'))
    assert [p['x'] for p in wider] == [p['x'] for p in one]                                 # a new range: same x
    assert len(study.load(write(tmp_path, 'lhs.yaml', lhs)).cells()) == 20 * 2 * 2


def test_batch_task_rows_equal_per_unit_rows_resume_and_sampled_events(tmp_path):
    write(tmp_path, 'echo.py', ECHO)
    spec = write(tmp_path, 'batch.yaml', '''\
        name: batch
        arms:
          unit: {task: echo.py:echo, args: {world: {x: 0.5}}}
          batch: {task: echo.py:echo_batch, batch: true, args: {world: {x: 0.5}}}
        seeds: {dev: {role: dev, seeds: '0:40'}}
        record: {level: events, sample: 0.25}
        pack: {local_chunk: 5}
        ''')
    s, out = study.load(spec), tmp_path / 'out'
    calls, stop = tmp_path / 'calls.jsonl', tmp_path / 'stop'
    stop.write_text('')
    os.environ.update(CALLS=str(calls), STOP=str(stop))
    try:
        with pytest.raises(KeyboardInterrupt):                    # the batch arm stops at its chunk of seeds 10..14
            ex.run_study(s, out, stage=None)
        cached = {(e['unit']['arm'], e['unit']['seed']) for e in UnitCache(out / 'cache')}
        assert {x for a, x in cached if a == 'batch'} == set(range(10))
        stop.unlink()
        calls.write_text('')
        summary = ex.run_study(s, out)
        again = [json.loads(x) for x in calls.read_text().splitlines()]
    finally:
        for k in ('CALLS', 'STOP'):
            os.environ.pop(k, None)
    assert summary['reused'] == len(cached) and summary['ran'] == 80 - len(cached)
    assert not {(c['arm'], c['seed']) for c in again} & cached                              # only the missing units
    rows = {(r['arm'], r['seed']): r for r in read_rows(out)}
    assert len(rows) == 80
    for seed in range(40):
        a, b = rows[('unit', seed)], rows[('batch', seed)]
        assert a['pair'] == b['pair']
        assert {k: v for k, v in a.items() if k not in ('arm', 'cell', 'wall_s')} == \
               {k: v for k, v in b.items() if k not in ('arm', 'cell', 'wall_s')}
    events = [json.loads(x) for x in (out / 'events.jsonl').read_text().splitlines()]
    kept = {seed for seed in range(40) if ex.sampled(seed, 0.25)}
    assert kept and {(e['arm'], e['seed']) for e in events} == {(a, x) for a in ('unit', 'batch') for x in kept}
    assert all(e['stage'] == 'main' and e['cell'].startswith(f"main.{e['arm']}.") for e in events)
    assert len(events) == 3 * 2 * len(kept)
    for f in (0.02, 0.25):                                       # deterministic, and the rate is about right
        assert [ex.sampled(x, f) for x in range(500)] == [ex.sampled(x, f) for x in range(500)]
        rate = sum(ex.sampled(x, f) for x in range(20000)) / 20000
        assert abs(rate - f) < 0.15 * f


def test_gap_closed_is_a_ratio_of_means_and_inconclusive_without_a_gap():
    """gap_closed estimates mean(fixed - treated) / mean(fixed - reference) over seeds (the bootstrap redraws seeds);
    a reference gap whose CI includes 0 is 'no gap to close'."""
    from crane.study import report
    t = {'arms': {'fixed': 'F', 'treated': 'T', 'reference': 'R'}, 'metric': 'm', 'min': 0.7}

    def rows(gaps, closed):
        out = []
        for s, (g, c) in enumerate(zip(gaps, closed)):
            out += [{'arm': 'F', 'seed': s, 'm': 10.0}, {'arm': 'T', 'seed': s, 'm': 10.0 - c},
                    {'arm': 'R', 'seed': s, 'm': 10.0 - g}]
        return out
    # seed gaps 0.01 and 4: per-seed ratios would blow up on the first; the ratio of means is (0.01+3.2)/(0.01+4)
    line = report._gap_line(t, rows([0.01, 4.0] * 20, [0.01, 3.2] * 20), 500, 0.05)
    assert abs(line['diff'] - 3.21 / 4.01) < 1e-9 and line['verdict'] == 'met'
    flat = report._gap_line(t, rows([1.0, -1.0] * 20, [0.5, -0.5] * 20), 500, 0.05)
    assert flat['verdict'] == 'inconclusive' and 'no gap to close' in flat['why']
