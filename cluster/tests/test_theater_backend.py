"""The theater-render backend against a stand-in player and a stand-in coordinator client."""
import hashlib
import json
import os
import stat
import sys
import tempfile
import textwrap
import unittest
from pathlib import Path
from unittest import mock

from backends import theater
from crane_cluster.backends import JobContext

FAKE_PLAYER = textwrap.dedent("""\
    #!{python}
    import json, os, sys
    a = sys.argv
    arg = lambda k: a[a.index(k) + 1]
    out = arg('--crane-theater-out')
    open(arg('-logFile'), 'w').write(json.dumps({{'argv': a, 'display': os.environ.get('DISPLAY')}}))
    lo, hi = map(int, arg('--crane-theater-frames').split(':'))
    if lo >= 1000:
        sys.exit(4)
    open(os.path.join(out, 'video.mp4'), 'wb').write(b'mp4')
    json.dump({{'ok': True, 'frames_written': hi - lo, 'fps': 30, 'errors': 0, 'render_ms_median': 12.5,
               'checks': {{'teleports': 0, 'nan_poses': 0}}}}, open(os.path.join(out, 'render_report.json'), 'w'))
    open(os.path.join(out, 'frames.jsonl'), 'w').write('{{}}\\n')
""")


class FakeClient:
    def __init__(self, blobs):
        self.blobs = blobs

    def request(self, method, path, **kw):
        return self.blobs[path.rsplit('/', 1)[1]]


class TheaterBackendTest(unittest.TestCase):
    def setUp(self):
        self.tmp = Path(tempfile.mkdtemp())
        self.player = self.tmp / 'CRANE.x86_64'
        self.player.write_text(FAKE_PLAYER.format(python=sys.executable))
        self.player.chmod(self.player.stat().st_mode | stat.S_IEXEC)
        self.ffmpeg = self.tmp / 'ffmpeg'
        self.ffmpeg.write_text('#!/bin/sh\nexit 0\n')
        self.ffmpeg.chmod(0o755)
        manifest = json.dumps({'schema': 'crane-theater/1', 'world': {'plugin': 'roboboat', 'args':
                               ['--crane-scene', 'Roboboat Course', '--crane-course-replay', '{dir}/course_truth.json']}}).encode()
        truth = b'{"name": "c"}'
        self.blobs = {hashlib.sha256(b).hexdigest(): b for b in (manifest, truth)}
        self.inputs = {'manifest.json': hashlib.sha256(manifest).hexdigest(), 'course_truth.json': hashlib.sha256(truth).hexdigest()}

    def backend(self, spec, blobs=None, **options):
        b = theater.ClusterBackend('theater-render', {'player': str(self.player), 'ffmpeg': str(self.ffmpeg), **options})
        ctx = JobContext(job={'job_id': 'j', 'spec': spec, 'seeds': [0, 1]}, work_dir=self.tmp / 'job',
                         client=FakeClient(blobs or self.blobs))
        b.prepare(ctx)
        return b

    @unittest.skipIf(sys.platform == 'win32', 'shebang player')
    def test_a_chunk_renders_its_frames_headless_with_the_world_args(self):
        with mock.patch.dict(os.environ, {'DISPLAY': ':0'}):
            b = self.backend({'inputs': self.inputs, 'chunk_frames': 300, 'fps': 30})
        run_dir = b.run(1)
        log = json.loads((run_dir / 'player.log').read_text())
        argv = log['argv']
        self.assertIsNone(log['display'], 'no display reaches the player unless the worker names one')
        self.assertIn('-batchmode', argv)
        self.assertEqual(argv[argv.index('--crane-theater-frames') + 1], '300:600')
        self.assertEqual(Path(argv[argv.index('--crane-course-replay') + 1]).read_text(), '{"name": "c"}')
        self.assertEqual(argv[argv.index('--crane-theater-fps') + 1], '30')
        m = b.metrics(run_dir)
        self.assertEqual((m['ok'], m['frames_written'], m['checks.teleports']), (1.0, 300.0, 0.0))
        names = [n for _, n, _ in b.artifacts(run_dir)]
        self.assertEqual(names[:3], ['video.mp4', 'frames.jsonl', 'render_report.json'])

    @unittest.skipIf(sys.platform == 'win32', 'shebang player')
    def test_a_display_option_is_passed(self):
        b = self.backend({'inputs': self.inputs, 'chunk_frames': 10}, display=':1')
        self.assertEqual(json.loads((b.run(0) / 'player.log').read_text())['display'], ':1')

    @unittest.skipIf(sys.platform == 'win32', 'shebang player')
    def test_a_failing_chunk_fails_with_its_log(self):
        b = self.backend({'inputs': self.inputs, 'chunk_frames': 1000})
        with self.assertRaisesRegex(RuntimeError, 'exited 4'):
            b.run(1)

    def test_inputs_are_hash_checked(self):
        bad = dict(self.blobs)
        bad[self.inputs['course_truth.json']] = b'tampered'
        with self.assertRaisesRegex(RuntimeError, 'does not match'):
            self.backend({'inputs': self.inputs, 'chunk_frames': 10}, blobs=bad)

    def test_spec_needs_chunk_frames_and_a_manifest(self):
        with self.assertRaisesRegex(RuntimeError, 'chunk_frames'):
            self.backend({'inputs': self.inputs})
        with self.assertRaisesRegex(RuntimeError, 'no manifest.json'):
            self.backend({'inputs': {'course_truth.json': self.inputs['course_truth.json']}, 'chunk_frames': 5})


if __name__ == '__main__':
    unittest.main()
