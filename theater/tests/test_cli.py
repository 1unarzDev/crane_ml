"""crane-theater select/fetch/info against a stand-in coordinator."""

import hashlib
import json
from types import SimpleNamespace

import pytest

from theater import cli, traj


def run(i, it, score, outcome='finished', **kw):
    d = {'run_id': f'j{it}.a1.s{i}', 'job_id': f'j{it}', 'seed': i, 'iteration': it, 'score': score, 'outcome': outcome,
         'valid': True, 'params': {'p': it}, 'course_sha256': 'c' * 64, 'course_family': 'roboboat_2026_t12',
         'water_mode': 'locked', 'traj_key': f'artifacts/j{it}/j{it}.a1.s{i}/run.ctraj', 'traj_sha256': None,
         'traj_size': 10, 'insight': f'iter {it}', 'recorder': 'unity'}
    d.update(kw)
    return d


class FakeCoord(cli.Coordinator):
    def __init__(self, runs, blobs=None):
        super().__init__('http://x', None)
        self.all, self.blobs, self.calls = runs, blobs or {}, []

    def get_json(self, path, **q):
        self.calls.append(q)
        off, lim = q['offset'], q['limit']
        page = self.all[off:off + lim]
        return {'runs': page, 'next_offset': off + lim if off + lim < len(self.all) else None}

    def get_bytes(self, path):
        return self.blobs[path.split('/v1/artifact/', 1)[1].replace('%2F', '/')]


def args(**kw):
    base = dict(study='s', hypothesis=None, iterations=None, top_k=None, best_per_iteration=False, failures=False,
                failures_only=False, outcome=None, valid_only=False, sample=None, seed=0, recorder=None, course=None,
                lowest_best=False, from_file=None, out=None, coordinator=None, token=None)
    base.update(kw)
    return SimpleNamespace(**base)


RUNS = [run(1, 0, 0.2, 'failed', failure_reason='buoy_hit'), run(2, 0, 0.5), run(3, 0, None, 'timeout'),
        run(4, 1, 0.9), run(5, 1, 0.7, water_mode='free'), run(6, 1, 0.1, 'invalid', valid=False, traj_size=None),
        run(7, 2, 0.95, course_sha256='d' * 64)]


def test_top_k_failures_and_flags(tmp_path, monkeypatch):
    monkeypatch.setattr(cli, 'PAGE', 3)  # paging: three pages
    coord = FakeCoord(RUNS)
    doc = cli.cmd_select(args(top_k=1, failures=True, iterations='0..2', out=str(tmp_path / 'm.json')), coord)
    assert len(coord.calls) == 3 and coord.calls[0]['iter_from'] == 0 and coord.calls[0]['iter_to'] == 2
    roles = {r['seed']: r['roles'] for r in doc['runs']}
    assert roles == {2: ['top'], 1: ['failure'], 3: ['failure'], 4: ['top'], 6: ['failure'], 7: ['top']}
    assert [r['seed'] for r in doc['runs']] == [2, 1, 3, 4, 6, 7]
    flags = {r['seed']: r['flags'] for r in doc['runs']}
    assert flags[6] == ['traj_missing', 'invalid'] and flags[7] == ['course_mismatch'] and flags[2] == []
    assert doc['course']['truth_sha256'] == 'c' * 64 and doc['water']['mode'] == 'locked'
    it0 = doc['iterations'][0]
    assert (it0['best_run'], it0['best_score'], it0['failed'], it0['selected']) == ('j0.a1.s2', 0.5, 2, 3)
    assert json.loads((tmp_path / 'm.json').read_text()) == doc
    assert any('traj_missing' in w for w in doc['warnings'])


def test_best_lowest_sampling_and_determinism():
    a = cli.cmd_select(args(best_per_iteration=True, lowest_best=True), FakeCoord(RUNS))
    assert [r['seed'] for r in a['runs']] == [1, 6, 7] and a['score_order'] == 'asc'
    s1 = cli.cmd_select(args(sample=3, seed=5), FakeCoord(RUNS))
    s2 = cli.cmd_select(args(sample=3, seed=5), FakeCoord(list(reversed(RUNS))))
    assert s1 == s2 and len(s1['runs']) == 3 and all(r['roles'] == ['sample'] for r in s1['runs'])
    everything = cli.cmd_select(args(), FakeCoord(RUNS))
    assert len(everything['runs']) == len(RUNS) and {tuple(r['roles']) for r in everything['runs']} == {('all',)}
    assert {r['seed'] for r in cli.cmd_select(args(failures_only=True), FakeCoord(RUNS))['runs']} == {1, 3, 6}


def test_parse_iterations():
    assert cli.parse_iterations('3') == (3, 3) and cli.parse_iterations('3..9') == (3, 9)
    assert cli.parse_iterations('..9') == (None, 9) and cli.parse_iterations('4:') == (4, None)


def test_fetch_checks_sha256_and_info_reads(tmp_path, capsys):
    p = tmp_path / 'a.ctraj'
    traj.write(p, {'outcome': 'finished'}, {'r.x': [0.0, 1.0], 'r.y': [0.0, 0.0]}, hz=10)
    blob = p.read_bytes()
    good = run(1, 0, 1.0, traj_sha256=hashlib.sha256(blob).hexdigest())
    bad = run(2, 0, 0.5, traj_sha256='0' * 64)
    coord = FakeCoord([good, bad], {good['traj_key']: blob, bad['traj_key']: blob})
    cli.cmd_select(args(out=str(tmp_path / 'm.json')), coord)
    with pytest.raises(SystemExit):
        cli.cmd_fetch(SimpleNamespace(manifest=str(tmp_path / 'm.json'), dir=str(tmp_path / 'trajs')), coord)
    assert (tmp_path / 'trajs' / 'j0.a1.s1.ctraj').read_bytes() == blob
    assert not (tmp_path / 'trajs' / 'j0.a1.s2.ctraj').exists()
    cli.cmd_info(SimpleNamespace(file=str(p)))
    assert '2 ticks at 10.0 Hz' in capsys.readouterr().out


def test_from_file_and_missing_study(tmp_path):
    (tmp_path / 'q.json').write_text(json.dumps({'runs': RUNS}))
    assert len(cli.cmd_select(args(study=None, from_file=str(tmp_path / 'q.json')))['runs']) == len(RUNS)
    with pytest.raises(SystemExit):
        cli.cmd_select(args(study=None))
