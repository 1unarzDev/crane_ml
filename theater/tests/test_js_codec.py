"""theater/web/ctraj.js (the browser and Worker reader) decodes what theater/traj.py writes: the golden
file, a 6-DOF run with its LOD, and a fleet pack. Skipped without node >= 18."""

import json
import shutil
import struct
import subprocess
from pathlib import Path

import pytest

from theater import traj
from theater.tests import golden_data
from theater.tests.test_traj import boat_run

CHECK = Path(__file__).resolve().parents[1] / 'web' / 'check.mjs'
NODE = shutil.which('node')
pytestmark = pytest.mark.skipif(not NODE, reason='node not installed')


def run_check(*args):
    out = subprocess.run([NODE, str(CHECK), *map(str, args)], capture_output=True, text=True, timeout=120)
    assert out.returncode == 0, out.stderr
    return json.loads(out.stdout)


def same(py: traj.Traj, js: dict):
    assert js['header'] == py.header and js['ticks'] == py.ticks and js['events'] == py.events
    assert set(js['columns']) == set(py.columns)
    for k, v in py.columns.items():
        assert max((abs(a - b) for a, b in zip(v, js['columns'][k])), default=0) < 1e-9, k


def test_js_reads_the_golden_file_a_run_and_its_lod(tmp_path):
    cols = {k: v for k, v in boat_run(minutes=2.0).items() if not k.endswith('speed')}
    full = traj.encode({'robot': {'agents': ['boat']}, 'outcome': 'finished'}, cols,
                       [{'t': 1.5, 'kind': 'gate', 'index': 0}], hz=20)
    (tmp_path / 'run.ctraj').write_bytes(full)
    (tmp_path / 'run.lod.ctraj').write_bytes(traj.lod(full))
    files = [golden_data.HERE / 'small.ctraj', tmp_path / 'run.ctraj', tmp_path / 'run.lod.ctraj']
    got = run_check(*files)
    for f, js in zip(files, got['files']):
        same(traj.read(f), js)
    run = got['files'][1]
    assert run['blocks'][0][2] > 0 and run['blocks'][1][0] == 600 and run['window_first_tick'] == 1800
    assert isinstance(run['partial_need'], int) and run['partial_need'] > 20
    assert run['agents'] == ['boat'] and 0.5 < run['speed0'] < 3.0
    assert 'crc' in got['corrupt']
    # block offsets in the index are absolute: Python's own parse agrees
    _h, _s, _t, _bt, index, r = traj._prefix(full)
    assert run['blocks'][0][2] == r.pos and run['blocks'][0][3] == index[0][2]


def test_js_reads_a_fleet_pack(tmp_path):
    lods = [traj.lod(traj.encode({}, {'r.x': [float(i)] * 40, 'r.y': [0.0] * 40}, hz=20)) for i in range(3)]
    (tmp_path / 'fleet.bin').write_bytes(traj.fleet_pack(lods))
    got = run_check('--fleet', tmp_path / 'fleet.bin')
    assert [f['columns']['r.x'][0] for f in got['files']] == [0.0, 1.0, 2.0]
    assert struct.unpack('<I', traj.fleet_pack(lods)[:4])[0] == len(lods[0])
