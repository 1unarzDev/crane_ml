"""The shared library layer: crane.ids, crane.cache, crane.progress, and fastsim.batch resume."""

import json
from multiprocessing import get_context
import shutil

import pytest

from crane import ids, progress, select
from crane.cache import UnitCache
from fastsim import batch, config, identity
from fastsim.identity import tree_sha
import toy

TOY = toy.__file__.rsplit('/', 1)[0]


def test_ids_reexport_fastsim_identity_and_parse_every_seed_form(tmp_path):
    assert ids.derive_seed is identity.derive_seed and ids.digest is identity.digest
    for seed, stream in [(0, ()), (7, ('perception',)), (2**31, ('a', 'b', 3))]:
        assert ids.derive_seed(seed, *stream) == identity.derive_seed(seed, *stream)
    assert ids.digest({'b': 1, 'a': [1, 2]}) == ids.digest({'a': [1, 2], 'b': 1})
    (tmp_path / 'held.txt').write_text('# held-out\n40 41,42\n50:52  # tail\n')
    blocks = {'dev': '1:4', 'check': [9, 8], 'held': {'seeds': '@held.txt'}}
    assert ids.parse_seeds('3') == [3] and ids.parse_seeds('1:4') == [1, 2, 3]
    assert ids.parse_seeds is select.parse_seeds          # one parser; a repeated seed is refused, not dropped
    assert ids.parse_seeds('5,9') == [5, 9] and ids.parse_seeds('1:3,7') == [1, 2, 7]
    assert ids.parse_seeds('@held.txt', base=tmp_path) == [40, 41, 42, 50, 51]
    assert ids.parse_seeds('block:held', blocks=blocks, base=tmp_path) == [40, 41, 42, 50, 51]
    assert ids.parse_seeds('block:dev,block:check', blocks=blocks) == [1, 2, 3, 9, 8]
    for bad, why in [('x', 'bad seed spec'), ('4:2', 'empty'), ('5,9,5', 'twice'), ('block:nope', 'blocks: check, dev, held'),
                     ('@missing.txt', 'missing.txt')]:
        with pytest.raises(ValueError, match=why):
            ids.parse_seeds(bad, blocks=blocks, base=tmp_path)
    state = ids.git_state(TOY)
    assert set(state) >= {'commit', 'dirty', 'diff_sha'} and (state['diff_sha'] is None) == (not state['dirty'])


def _writer(root, worker, n):
    cache = UnitCache(root)
    for i in range(n):
        # every worker writes the shared units 0..n-1 and its own ones: first writer wins on the shared ones
        cache.put({'seed': i, 'arm': 'shared'}, {'by': worker, 'blob': 'x' * 2000})
        cache.put({'seed': i, 'arm': f'w{worker}'}, {'by': worker, 'i': i})


def test_unit_cache_survives_four_concurrent_writers_and_a_torn_line(tmp_path):
    root, n = tmp_path / 'cache', 60
    ctx = get_context('spawn')
    procs = [ctx.Process(target=_writer, args=(str(root), w, n)) for w in range(4)]
    for p in procs:
        p.start()
    for p in procs:
        p.join(60)
        assert p.exitcode == 0
    lines = (root / 'records.jsonl').read_bytes().splitlines()
    entries = [json.loads(raw) for raw in lines]  # every line whole
    assert len(entries) == len({e['key'] for e in entries}) == n * 5
    cache = UnitCache(root)
    assert len(cache) == n * 5 and cache.get({'arm': 'w2', 'seed': 7}) == {'by': 2, 'i': 7}
    assert cache.get({'arm': 'shared', 'seed': 3})['by'] in range(4) and not cache.has({'arm': 'w9', 'seed': 0})
    with open(root / 'records.jsonl', 'ab') as f:
        f.write(b'{"key": "torn')  # a writer killed mid-line
    assert cache.put({'arm': 'late', 'seed': 0}, {'ok': 1}) == {'ok': 1}
    assert UnitCache(root).get({'arm': 'late', 'seed': 0}) == {'ok': 1} and len(UnitCache(root)) == n * 5 + 1
    sha = cache.put_blob(b'\x00npz bytes', '.npz')
    assert cache.blob_path(sha).name == f'{sha}.npz' and cache.get_blob(sha) == b'\x00npz bytes'
    assert cache.put_blob(cache.blob_path(sha), '.npz') == sha


def test_progress_writes_atomically_with_a_sane_rate_and_eta(tmp_path, monkeypatch, capsys):
    now = [0.0]
    monkeypatch.delenv(progress.ENV, raising=False)
    p = progress.Progress(tmp_path, total=100, every_s=15, clock=lambda: now[0])
    assert progress.read(tmp_path)['done'] == 0 and progress.read(tmp_path)['eta_s'] is None
    p.skip(20)                                  # reused: shortens the ETA, not the rate
    for _ in range(30):
        now[0] += 0.5
        p.step(failed=_ == 0)
    state = progress.read(tmp_path)             # 15 s in: written once at 15 s
    assert state['done'] == 50 and state['failed'] == 1 and state['rate_per_s'] == pytest.approx(2.0)
    assert state['eta_s'] == pytest.approx(25.0) and state['age_s'] < 5
    assert [f.name for f in tmp_path.iterdir()] == ['progress.json']   # no temp file left
    assert capsys.readouterr().out == ''        # under a minute: no human line yet
    now[0] = 61.0
    p.step()
    out = capsys.readouterr().out
    assert out.startswith('progress  51/100  0.5 runs/s  eta 1m36s  failed 1')
    p.close()
    assert progress.read(tmp_path)['phase'] == 'done'
    assert progress.Progress(tmp_path, 1, every_s=600).every_s == progress.MAX_EVERY_S
    monkeypatch.setenv(progress.ENV, str(tmp_path / 'forwarded'))
    progress.Progress(tmp_path / 'ignored', total=3).close()
    assert progress.read(tmp_path / 'forwarded')['total'] == 3 and not (tmp_path / 'ignored').exists()


@pytest.fixture
def toy_batch(tmp_path):
    folder = tmp_path / 'scenarios'
    cfg, base = config.load(f'{TOY}/toy.yaml', {'scenarios': {'dir': str(folder)}})
    for seed in range(6):
        toy.write_scenario(folder, seed)
    return cfg, base, tmp_path / 'out'


def test_an_interrupted_batch_resumes_and_reruns_only_the_missing_seed(toy_batch):
    cfg, base, out = toy_batch
    first = batch.run_batch(cfg, base, range(6), out, workers=3)
    trees = {r.seed: tree_sha(r.out) for r in first}
    shutil.rmtree(out / 'base' / 's4')          # the interruption
    (out / 'base' / '.s2.partial-999999999').mkdir()   # and a dead worker's leftovers
    (out / 'base' / '.s3.partial-1').mkdir()           # a live process's partial (pid 1) is left alone
    again = batch.run_batch(cfg, base, range(6), out, workers=3)
    assert [r.seed for r in again] == list(range(6))
    assert [r.run_sha256 for r in again] == [r.run_sha256 for r in first]
    assert [r.wall_s == 0.0 for r in again] == [s != 4 for s in range(6)]   # only s4 ran
    assert {r.seed: tree_sha(r.out) for r in again} == trees                # byte for byte
    assert sorted(p.name for p in (out / 'base').iterdir()) == ['.s3.partial-1', *(f's{s}' for s in range(6))]
    assert progress.read(out)['done'] == 6 and progress.read(out)['label'] == 'base'


def test_a_conflicting_or_partial_run_directory_is_refused_unless_forced(toy_batch):
    cfg, base, out = toy_batch
    batch.run_batch(cfg, base, [1, 2], out, workers=1)
    other = config.merge(cfg, {'vehicle': {'gain': 0.5}})
    with pytest.raises(batch.RunDirConflict, match=r's1 holds another run .*--force'):
        batch.run_batch(other, base, [1, 2], out, workers=1)
    (out / 'base' / 's2' / 'crane.run.json').unlink()
    with pytest.raises(batch.RunDirConflict, match='s2 is incomplete'):
        batch.run_batch(cfg, base, [2], out, workers=1)
    forced = batch.run_batch(other, base, [1, 2], out, workers=2, force=True)
    assert {r.wall_s > 0 for r in forced} == {True}
    run = out / 'base' / 's1' / 'run.json'                          # this run, but it errored: rerun
    run.write_text(json.dumps({**json.loads(run.read_text()), 'outcome': 'ERROR'}))
    [again] = batch.run_batch(other, base, [1], out, workers=1)
    assert again.wall_s > 0 and again.outcome != 'ERROR' and len(list((out / 'base').glob('s1.stale-*'))) == 2
    assert (out / 'base' / 's1.stale-1' / 'run.json').exists() and (out / 'base' / 's2.stale-1').exists()
