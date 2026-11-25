"""A rented box's clean sparse copy (crane/runtime.sparse): committed courses only, no generator.

The pooled benchmark works there, a seed without a course is refused in one line, and a scenario pack made on a
host with the courses (crane env pack) is found through CRANE_SCENARIOS.
"""
import json
import os
from pathlib import Path
import shutil
import subprocess
import sys

import pytest

ROOT = Path(__file__).resolve().parents[1]
COURSES = ROOT / 'domains' / 'boat' / 'courses'


def _box(tmp_path):
    dirs = [s.strip() for s in (ROOT / 'crane' / 'runtime.sparse').read_text().splitlines()
            if s.strip() and not s.lstrip().startswith('#')]
    box = tmp_path / 'box'
    box.mkdir()
    for f in ROOT.iterdir():
        if f.is_file():
            shutil.copy2(f, box / f.name)
    for d in dirs:
        shutil.copytree(ROOT / d, box / d, ignore=shutil.ignore_patterns('__pycache__', '*.pyc', 'bin', 'obj'))
    return box


def _run(cwd, *argv, env=None):
    e = {**os.environ, 'PYTHONDONTWRITEBYTECODE': '1', 'PYTHONPATH': str(cwd)}
    e.pop('CRANE_SCENARIOS', None)
    e.update(env or {})
    return subprocess.run([sys.executable, *argv], cwd=cwd, env=e, capture_output=True, text=True)


def test_clean_copy_pooled_bench_refusal_and_pack(tmp_path):
    pytest.importorskip('numba')      # the engine compiles with it
    committed = sorted(COURSES.glob('roboboat_2026-s*.json'))
    assert len(committed) >= 5
    box = _box(tmp_path)
    (box / 'domains' / 'boat' / 'courses' / 'roboboat_2026-s3.json').unlink()      # a course this copy lacks

    bench = _run(box, '-m', 'bench.fast', '--engine', 'batched', '--tier', 'T0', '--backend', 'numpy', '--E', '8',
                 '--seeds', '0:16', '--course-pool', '3', '--out', str(tmp_path / 'bench'))
    assert bench.returncode == 0, bench.stderr[-800:]
    report = json.loads((tmp_path / 'bench' / 'batched.json').read_text())
    assert report['course_pool'] == 3 and 'course pool 3' in (tmp_path / 'bench' / 'batched.md').read_text().lower()

    ref = _run(box, '-m', 'bench.fast', '--engine', 'batched', '--tier', 'T0', '--backend', 'numpy', '--E', '8',
               '--out', str(tmp_path / 'nopool'))                  # seeds 0..7 need s5..s7: not here
    assert ref.returncode != 0 and ref.stderr.strip().count('\n') == 0, ref.stderr[-800:]
    assert ref.stderr.startswith('crane: no scenario for seed 3 ') and 'no generator here' in ref.stderr

    run = _run(box, '-m', 'crane', 'run', '--domain', 'boat', '--tier', 'T0', '--seeds', '3', '--out', str(tmp_path / 'r'))
    assert run.returncode != 0 and run.stderr.strip().count('\n') == 0
    assert run.stderr.startswith('crane: no scenario for seed 3 in ') and 'crane env pack' in run.stderr

    pack = tmp_path / 'courses.tar.zst'
    made = _run(ROOT, '-m', 'crane', 'env', 'pack', '--domain', 'boat', '--seeds', '0:5', '--out', str(pack))
    assert made.returncode == 0, made.stderr[-800:]
    packed = next(tmp_path.glob('courses.tar.*'))
    unpacked = tmp_path / 'unpacked'
    assert _run(ROOT, '-m', 'crane', 'env', 'unpack', str(packed), str(unpacked)).returncode == 0
    manifest = json.loads((unpacked / 'manifest.json').read_text())
    assert sorted(manifest['files']) == [c.name for c in committed[:5]] and manifest['domain'] == 'boat'

    ok = _run(box, '-m', 'crane', 'run', '--domain', 'boat', '--tier', 'T0', '--seeds', '3', '--out', str(tmp_path / 'r2'), '--allow-dirty',
              env={'CRANE_SCENARIOS': str(unpacked)})
    assert ok.returncode == 0, ok.stderr[-800:]

    from crane import pack as packs
    junk = tmp_path / 'junk.tar.zst'
    junk.write_bytes(b'nope nope')
    with pytest.raises(packs.PackError, match='not a crane pack'):
        packs.unpack(junk, tmp_path / 'x')
