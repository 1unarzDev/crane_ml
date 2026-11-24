"""crane.data: rows and manifests in every format, the fastsim recorder hook, export and schema, held-out-safe splits."""

import json
from pathlib import Path

import numpy as np
import pytest

from crane import cli
from crane.data import recorder as REC, rows as R, splits as S
from fastsim import batch, config, identity
from fastsim.run import simulate
from fastsim.scenario import JsonScenarioFormat
import toy

TOY = Path(toy.__file__).parent
FORMATS = ['csv', 'npz', 'jsonl'] + (['parquet'] if R.have_parquet() else [])


def _same(a, b):
    if isinstance(a, np.ndarray) or isinstance(b, np.ndarray):
        return a is not None and b is not None and a.dtype == b.dtype and np.array_equal(a, b, equal_nan=True)
    if isinstance(a, float) and isinstance(b, float) and a != a:
        return b != b                                  # NaN stays NaN, apart from null
    return a == b


@pytest.mark.parametrize('fmt', FORMATS)
def test_rows_round_trip_with_nulls_arrays_rollover_and_a_sha256_manifest(fmt, tmp_path):
    schema = {'seed': 'int', 'arm': 'str', 'score': 'float', 'ok': 'bool', 'cond': 'json', 'obs': 'array:float16'}
    rng = np.random.default_rng(0)
    rows = [{'seed': s, 'arm': 'ab'[s % 2], 'score': None if s == 3 else float('nan') if s == 5 else s / 7, 'ok': s % 3 == 0,
             'cond': {'mass': s, 'tags': ['x']} if s != 4 else None,
             'obs': rng.normal(size=(10 + s, 3)).astype(np.float16) if s != 2 else None} for s in range(7)]
    w = R.RowWriter(tmp_path / 'ds', schema, fmt=fmt, max_rows=3)      # 7 rows: three parts
    for row in rows:
        w.write(row)
    files = w.close()
    R.write_manifest(tmp_path / 'ds', schema, files, {'study': 'unit'})
    assert sum(f['rows'] for f in files if not f['path'].endswith(R.ARRAYS)) == 7
    assert len([f for f in files if not f['path'].endswith(R.ARRAYS)]) == 3
    assert R.verify(tmp_path / 'ds') == [] and R.read_manifest(tmp_path / 'ds')['rows'] == 7
    back = list(R.read_rows(tmp_path / 'ds'))
    assert len(back) == 7
    for want, got in zip(rows, back):
        assert set(got) == set(schema) and all(_same(want[k], got[k]) for k in schema), (want['seed'], got)
    assert R.columns_of(tmp_path / 'ds') == schema
    (tmp_path / 'ds' / files[0]['path']).write_bytes(b'changed')
    assert R.verify(tmp_path / 'ds') == [(files[0]['path'], 'sha256 differs')]
    with pytest.raises(R.SchemaError, match=r"unknown column 'extra'; schema columns: seed, arm"):
        R.RowWriter(tmp_path / 'x', schema, fmt=fmt).write({'seed': 1, 'extra': 2})
    with pytest.raises(R.SchemaError, match="column 'seed' \\(int\\): cannot store 'x'"):
        R.RowWriter(tmp_path / 'y', schema, fmt=fmt).write({'seed': 'x'})
    if fmt == 'npz':                                   # equal rows give equal bytes (fixed zip timestamps)
        again = R.RowWriter(tmp_path / 'again', schema, fmt=fmt, max_rows=3)
        for row in rows:
            again.write(row)
        assert [f['sha256'] for f in again.close()] == [f['sha256'] for f in files]


def test_any_jsonl_of_objects_exports_like_a_dataset(tmp_path, capsys):
    from crane.cache import UnitCache
    cache = UnitCache(tmp_path / 'cache')               # the unit cache's records.jsonl is a row source as it is
    for s in range(3):
        cache.put({'seed': s, 'arm': 'a'}, {'score': s / 2, 'ok': s != 1})
    src = next((tmp_path / 'cache').rglob('records.jsonl'))
    assert cli.main(['data', 'export', str(src), '--format', 'csv', '--out', str(tmp_path / 'o'), '--json']) == 0
    doc = json.loads(capsys.readouterr().out)
    assert doc['rows'] == 3 and doc['held_count'] == 0 and R.verify(tmp_path / 'o') == []
    assert len(list(R.read_rows(tmp_path / 'o'))) == 3
    (tmp_path / 'bad.jsonl').write_text('{"a": 1}\n[1, 2]\n')
    assert cli.main(['data', 'export', str(tmp_path / 'bad.jsonl'), '--out', str(tmp_path / 'b')]) == 2
    assert 'bad.jsonl:2: a list, not an object' in capsys.readouterr().err


def _cfg(record=None):
    over = {'scenarios': {'dir': 'scenarios'}, **({'record': record} if record is not None else {})}
    return config.load(TOY / 'toy.yaml', over)


def _run(tmp_path, name, seed=3, record=None, recorder=None):
    cfg, base = _cfg(record)
    scenario = JsonScenarioFormat().load(toy.write_scenario(tmp_path / 'scenarios', seed))
    adapter = toy.ToyAdapter(base=TOY, reach_m=1.5, time_limit_s=240)
    return simulate(scenario, adapter, cfg, seed, tmp_path / name, base, recorder=recorder), tmp_path / name


def test_the_recorder_off_changes_nothing_and_on_records_full_rate_obs(tmp_path):
    off, d_off = _run(tmp_path, 'off')
    assert not (d_off / 'record').exists() and 'record' not in off
    assert 'obs' not in json.loads((d_off / 'crane.run.json').read_text())['files']
    cfg, base = _cfg()
    adapter = toy.ToyAdapter(base=TOY, reach_m=1.5, time_limit_s=240)
    scenario = JsonScenarioFormat().load(tmp_path / 'scenarios' / 'toy-s3.json')
    assert off['identity'] == batch.expected_identity(cfg, adapter, scenario, 3)    # resume still matches

    spec = [{'obs': {'channels': ['cmd', 'vel', 'tracks.n'], 'hz': 20, 'dtype': 'float16'}, 'labels': {'fault': 'none'}},
            {'traj': {'hz': 5, 'window': 12}}, 'events']
    on, d_on = _run(tmp_path, 'on', recorder=REC.Recorder(spec))     # an explicit recorder: same identity, same sim
    assert on['identity'] == off['identity'] and on['outcome'] == off['outcome']
    for f in d_off.iterdir():
        if f.name not in ('run.json', 'crane.run.json'):
            assert f.read_bytes() == (d_on / f.name).read_bytes(), f.name
    assert {k: v for k, v in json.loads((d_on / 'run.json').read_text()).items() if k != 'record'} == \
        json.loads((d_off / 'run.json').read_text())
    arrays = REC.load_run(d_on)
    obs, traj = arrays['obs'], arrays['traj']
    steps = round(on['sim_s'] / 0.05)                  # toy dt 0.05: 20 Hz is every step
    assert obs['obs'].dtype == np.float16 and obs['obs'].shape in {(steps, 7), (steps + 1, 7)}
    assert obs['channels'] == ['cmd.vx', 'cmd.vy', 'cmd.wz', 'vel.vx', 'vel.vy', 'vel.wz', 'tracks.n']
    assert obs['hz'] == 20.0 and obs['labels'] == {'fault': 'none'}
    assert traj['traj'].shape == (61, 6) and traj['hz'] == 5.0          # 0..12 s at 5 Hz, both ends
    track = [json.loads(line) for line in (d_on / 'track.jsonl').read_text().splitlines()]
    assert np.allclose(traj['traj'][:, 0], [r['x'] for r in track[:121:2]], atol=1e-3)   # the same world pose
    assert json.loads((d_on / 'crane.run.json').read_text())['files']['obs'] == 'record/obs.npz'

    viacfg, d_cfg = _run(tmp_path, 'cfg', record={'obs': {'channels': ['pose'], 'hz': 10}})   # the config key
    assert viacfg['identity']['run_sha256'] != off['identity']['run_sha256']    # recorded runs never reuse plain ones
    assert REC.load_run(d_cfg)['obs']['obs'].shape[1] == 3
    with pytest.raises(REC.RecordError, match="no channel 'wheel'; channels: pose, vel"):
        _run(tmp_path, 'bad', record={'obs': {'channels': ['wheel']}})
    with pytest.raises(REC.RecordError, match='unknown level'):
        REC.parse({'video': {}})


def test_export_and_schema_over_batch_runs_and_a_study_store(tmp_path, capsys):
    cfg, base = config.load(TOY / 'toy.yaml', {'scenarios': {'dir': str(tmp_path / 'sc')},
                                               'record': {'obs': {'channels': ['cmd', 'vel'], 'hz': 10,
                                                                  'dtype': 'float16'}}})
    for s in range(4):
        toy.write_scenario(tmp_path / 'sc', s)
    results = batch.run_batch(cfg, base, range(4), tmp_path / 'runs', workers=1, progress=False)
    out = tmp_path / 'ds'
    assert cli.main(['data', 'export', str(tmp_path / 'runs'), '--out', str(out), '--json']) == 0
    doc = json.loads(capsys.readouterr().out)
    assert doc['rows'] == 4 and R.verify(out) == []
    manifest = R.read_manifest(out)
    assert list(manifest['columns'])[:5] == ['run', 'variant', 'seed', 'outcome', 'sim_s']
    assert manifest['columns']['obs'] == 'array:float16' and manifest['arrays']['obs']['hz'] == 10.0
    with np.load(out / manifest['files'][0]['path']) as z:            # envs x T x channels, float16
        lengths = [round(r.sim_s * 10) for r in results]
        assert z['obs'].shape[0] == 4 and z['obs'].shape[2] == 6 and z['obs'].dtype == np.float16
        assert abs(z['obs'].shape[1] - max(lengths)) <= 1 and list(z['seed']) == [0, 1, 2, 3]
    rows = list(R.read_rows(out))
    assert [(r['outcome'], r['run_sha256']) for r in rows] == [(r.outcome, r.run_sha256) for r in results]

    sub = tmp_path / 'sub'
    assert cli.main(['data', 'export', str(out), '--format', 'csv', '--select', 'seed,outcome', '--select', 'obs',
                     '--where', 'seed>=2', '--out', str(sub)]) == 0
    assert 'next: crane data schema' in capsys.readouterr().out
    got = list(R.read_rows(sub))
    assert [r['seed'] for r in got] == [2, 3] and set(got[0]) == {'seed', 'outcome', 'obs'}
    assert np.array_equal(got[0]['obs'], rows[2]['obs']) and (sub / 'part-0000.arrays.npz').is_file()

    assert cli.main(['data', 'schema', str(out), '--json']) == 0
    schema = json.loads(capsys.readouterr().out)
    assert schema['rows'] == 4 and schema['columns']['seed'] == 'int'
    assert cli.main(['data', 'export', str(out), '--where', 'nope=1', '--out', str(tmp_path / 'z')]) == 2
    assert "no column 'nope'" in capsys.readouterr().err
    assert cli.main(['data', 'export', str(out), '--out', str(sub)]) == 2           # written once
    assert 'is not empty' in capsys.readouterr().err

    store = tmp_path / 'store'                         # a study-level store: jobs append rows keyed by cell/seed
    for job, seeds in (('job-a', (0, 1)), ('job-b', (2,))):
        st = REC.Store(store, part=job, extra={'study': 'unit'})
        for s in seeds:
            rec = REC.Recorder({'obs': {'channels': ['cmd'], 'hz': 10}, 'labels': {'gain': 0.5}})
            simulate(JsonScenarioFormat().load(tmp_path / 'sc' / f'toy-s{s}.json'), batch.adapter_of(cfg, base),
                     cfg, s, tmp_path / 'st' / f's{s}', base, recorder=rec)
            st.add({'cell': 'c1', 'seed': s}, rec)
        st.close()
    keyed = list(R.read_rows(store / 'obs'))
    assert [(r['cell'], r['seed'], r['label.gain']) for r in keyed] == [('c1', 0, 0.5), ('c1', 1, 0.5), ('c1', 2, 0.5)]
    assert R.verify(store / 'obs') == [] and R.read_manifest(store / 'obs')['channels'] == ['cmd.vx', 'cmd.vy', 'cmd.wz']


def test_splits_never_leak_held_out_and_never_move_a_unit_into_train(tmp_path, capsys):
    schema = {'seed': 'int', 'timeline': 'str', 'source': 'str', 'role': 'str', 'score': 'float'}
    held_seeds = set(range(0, 200, 9))

    def dataset(name, seeds):
        w = R.RowWriter(tmp_path / name, schema, fmt='csv')
        for s in seeds:
            for arm in range(2):                       # two rows per seed: a unit is many rows
                w.write({'seed': s, 'timeline': f'tl{s % 40}', 'source': 'real' if s % 40 < 4 else 'synthetic',
                         'role': 'held-out' if s in held_seeds and arm == 1 else 'dev', 'score': s + arm})
        R.write_manifest(tmp_path / name, schema, w.close())
        return tmp_path / name

    full = dataset('full', range(200))
    assert cli.main(['data', 'splits', str(full), '--by', 'seed', '--json']) == 0
    doc = json.loads(capsys.readouterr().out)
    sp = doc['splits']
    assert set(sp['held-out']['units']) == held_seeds                      # one held-out row makes the unit held-out
    assert not held_seeds & set(sp['train']['units'] + sp['calib']['units'] + sp['test']['units'])
    assert sum(s['n_units'] for s in sp.values()) == 200 and 0.6 < sp['train']['fraction'] < 0.8
    assert json.loads((full / 'splits.json').read_text())['splits'] == sp

    by_tl = S.splits([full], by='timeline', ratios='train=0.8,calib=0.2', held_out=['source=real'])
    real = {f'tl{k}' for k in range(4)}
    assert real <= set(by_tl['splits']['held-out']['units'])              # real sessions never enter train/calib
    assert not real & set(by_tl['splits']['train']['units'] + by_tl['splits']['calib']['units'])

    part = dataset('part', range(0, 200, 2))           # fewer units: each keeps the split it had
    small = S.splits([part], by='seed')
    where = {u: name for name, s in sp.items() for u in s['units']}
    assert all(where[u] == name for name, s in small['splits'].items() for u in s['units'])

    assert cli.main(['data', 'splits', str(full), '--salt', 'other']) == 2             # would move test units to train
    assert 'was test in the earlier split file and would move to' in capsys.readouterr().err
    assert cli.main(['data', 'splits', str(full), '--ratios', 'train=0.9,held-out=0.1', '--out',
                     str(tmp_path / 's.json')]) == 2
    assert 'set by the held-out tags' in capsys.readouterr().err
    assert cli.main(['data', 'export', str(full), '--split', f'{full / "splits.json"}:train', '--out',
                     str(tmp_path / 'train')]) == 0
    capsys.readouterr()
    tiny = S.splits([dataset('tiny', [1, 2])], by='seed')       # two units cannot fill three splits
    assert tiny['warnings'] and all('is empty' in w for w in tiny['warnings'])
    assert cli.main(['data', 'export', str(full), '--out', str(tmp_path / 'all')]) == 2     # held-out rows: refused
    assert f'{len(held_seeds)} of 400 rows are held-out' in capsys.readouterr().err
    assert cli.main(['data', 'export', str(full), '--where', 'role!=held-out', '--out', str(tmp_path / 'dev')]) == 0
    assert cli.main(['data', 'export', str(full), '--confirm', '--out', str(tmp_path / 'all')]) == 0
    assert f'{len(held_seeds)} held-out row(s) exported' in capsys.readouterr().out
    train = list(R.read_rows(tmp_path / 'train'))
    assert {r['seed'] for r in train} == set(sp['train']['units']) and not any(r['role'] == 'held-out' for r in train)
