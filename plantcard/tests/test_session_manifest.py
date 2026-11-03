"""The boat session manifest reader: exclusions, conditions and calibration windows."""

import json

import pytest

from plantcard.marine import session_manifest as M


def write(folder, **doc):
    folder.mkdir(parents=True, exist_ok=True)
    (folder / M.FILE).write_text(json.dumps({'schema': M.SCHEMA, 'segments': [], 'conditions': [],
                                             'calibration': [], **doc}))


def test_find_walks_up_and_reads_exclusions(tmp_path):
    write(tmp_path / 's1', conditions=[{'t': 10.0, 'wind': 2, 'waves': 1, 'current': 0}], segments=[
        {'kind': 'bad', 't0': 20.0, 't1': 30.0, 'status': 'pending'},
        {'kind': 'intervention', 't0': 40.0, 't1': 40.0, 'status': 'rejected'},
        {'kind': 'mark', 't0': 50.0, 't1': 50.0, 'effect': 'invalid', 'span': [48.0, 55.0], 'label': 'contact'}],
        calibration=[{'kind': 'neutral', 'quality': 'ok', 't0': 0, 't1': 9},
                     {'kind': 'steady', 'quality': 'suspect', 't0': 60, 't1': 70}])
    trial = tmp_path / 's1' / 'agent_x.files' / 'bag'
    trial.mkdir(parents=True)
    doc = M.find(trial)
    assert M.overlaps(doc, 25, 26) == ['bad'] and M.overlaps(doc, 39, 41) == [] and M.overlaps(doc, 54, 60) == ['contact']
    assert M.conditions_at(doc, 5) == {'wind': 2, 'waves': 1, 'current': 0}
    assert [w['kind'] for w in M.calibration(doc)] == ['neutral']
    assert M.find(tmp_path / 'elsewhere') is None


def test_another_major_is_refused(tmp_path):
    (tmp_path / M.FILE).write_text(json.dumps({'schema': 'mhseals.session_manifest/2'}))
    with pytest.raises(ValueError):
        M.load(tmp_path)
