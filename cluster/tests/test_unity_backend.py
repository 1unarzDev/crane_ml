"""The unity backend against a stand-in player (a script that answers the player's flags)."""
import json
import os
import stat
import sys
import tempfile
import textwrap
import unittest
from unittest import mock
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))  # the crane root: theater
from backends import unity  # noqa: E402
from crane_cluster.backends import JobContext  # noqa: E402
from theater import traj  # noqa: E402

CRANE_ROOT = str(Path(__file__).resolve().parents[2])

FAKE_PLAYER = textwrap.dedent("""\
    #!{python}
    import json, sys
    sys.path.insert(0, {root!r})
    from theater import traj
    a = sys.argv
    arg = lambda k: a[a.index(k) + 1]
    open(arg('-logFile'), 'w').write(' '.join(a))
    if arg('--crane-course-seed') == '13':
        sys.exit(4)
    json.dump({{'name': 'c', 'generator': {{'seed': int(arg('--crane-course-seed')), 'truth_sha256': 'f' * 64,
               'identity': {{'pair_key': 'pk'}}}}}}, open(arg('--crane-course-truth'), 'w'))
    if '--crane-traj' in a:
        traj.write(arg('--crane-traj'), {{'seed': arg('--crane-seed'), 'outcome': 'unknown',
                   'build': {{'recorder': 'unity', 'build_hash': 'b' * 64}}, 'robot': {{'agents': ['robot']}}}},
                   {{'robot.x': [0.0, 0.5], 'robot.y': [0.0, 0.0]}}, hz=float(arg('--crane-traj-hz')))
    json.dump({{'schema': 'crane-benchmark-v1', 'scene': arg('--crane-scene'), 'simulatedSeconds': float(arg('--crane-duration')),
               'valid': True, 'markers': [{{'name': 'Physics', 'meanMillisecondsPerFrame': 1.5}}], 'samples': [1, 2]}},
              open(arg('--crane-output'), 'w'))
""")


class UnityBackendTest(unittest.TestCase):
    def setUp(self):
        self.tmp = Path(tempfile.mkdtemp())
        self.player = self.tmp / 'CRANE.x86_64'
        self.player.write_text(FAKE_PLAYER.format(python=sys.executable, root=CRANE_ROOT))
        self.player.chmod(self.player.stat().st_mode | stat.S_IEXEC)

    def backend(self, spec, job=None, **options):
        b = unity.ClusterBackend('unity', {'player': str(self.player), **options})
        b.prepare(JobContext(job={'job_id': 'j', 'spec': spec, 'seeds': [7], **(job or {})}, work_dir=self.tmp / 'job'))
        return b

    def test_a_seed_runs_the_player_headless_and_scores_its_result(self):
        b = self.backend({'duration': 12, 'set': ['nav.gates=3']})
        run_dir = b.run(7)
        argv = (run_dir / 'player.log').read_text().split()
        self.assertIn('-batchmode', argv)
        self.assertEqual(argv[argv.index('--crane-course-family') + 1], 'roboboat_2026')
        self.assertEqual(argv[argv.index('--crane-course-set') + 1], 'nav.gates=3')
        self.assertIn('--crane-disable-ros', argv)
        m = b.metrics(run_dir)
        self.assertEqual((m['simulatedSeconds'], m['valid'], m['course_ready']), (12.0, 1.0, 1.0))
        self.assertEqual(m['markers.Physics.meanMillisecondsPerFrame'], 1.5)
        self.assertNotIn('samples', m)
        self.assertEqual([(n, r) for _, n, r in b.artifacts(run_dir)],
                         [('result.json', 'summary'), ('course_truth.json', 'summary'), ('run.ctraj', 'summary'),
                          ('player.log', 'raw')])

    def test_the_trajectory_is_stamped_and_indexed(self):
        spec = {'metric': 'simulatedSeconds', 'params': {'nav': {'v': 1.5}}, 'theater': {'study': 'demo', 'insight': 'go'},
                'set': ['nav.gates=3']}
        b = self.backend(spec, job={'hypothesis_id': 'h1', 'stage_index': 3, 'arm': 'baseline'})
        run_dir = b.run(7)
        argv = (run_dir / 'player.log').read_text().split()
        self.assertEqual(argv[argv.index('--crane-traj') + 1], str(run_dir / 'run.ctraj'))
        m = b.run_meta(run_dir, b.metrics(run_dir))
        self.assertEqual((m['study'], m['iteration'], m['score'], m['outcome'], m['valid']), ('demo', 3, 30.0, 'finished', True))
        self.assertEqual((m['course_sha256'], m['recorder'], m['traj_name'], m['seed']), ('f' * 64, 'unity', 'run.ctraj', 7))
        self.assertEqual(m['params'], {'nav.v': 1.5})
        h = traj.read_header(run_dir / 'run.ctraj')
        self.assertEqual((h['arm'], h['course']['pair_key'], h['course']['set'], h['insight']), ('baseline', 'pk', ['nav.gates=3'], 'go'))
        self.assertEqual(m['traj_sha256'], __import__('hashlib').sha256((run_dir / 'run.ctraj').read_bytes()).hexdigest())
        lod = traj.read_header(run_dir / 'run.lod.ctraj')
        self.assertEqual((lod['parent_sha256'], lod['outcome'], lod['hz']), (m['traj_sha256'], 'finished', 2.0))
        self.assertNotIn('params', lod)
        self.assertIn(('run.lod.ctraj', 'summary'), [(n, r) for _, n, r in b.artifacts(run_dir)])

    def test_no_trajectory_when_turned_off(self):
        b = self.backend({'traj': False})
        run_dir = b.run(7)
        self.assertNotIn('--crane-traj', (run_dir / 'player.log').read_text().split())
        self.assertIsNone(b.run_meta(run_dir, {}))

    def test_a_failing_player_fails_the_seed_with_its_log(self):
        with self.assertRaisesRegex(RuntimeError, 'exited 4'):
            self.backend({}).run(13)

    def test_a_visible_window_is_refused(self):
        if sys.platform == 'win32':
            self.skipTest('session-dependent on Windows')
        with mock.patch.dict(os.environ, {'DISPLAY': ':99'}), \
                self.assertRaisesRegex(RuntimeError, 'window would be visible'):
            self.backend({'batchmode': False})

    def test_flatten_keeps_numbers_only(self):
        self.assertEqual(unity.flatten({'a': {'b': 2, 's': 'x'}, 't': False, 'l': [3]}), {'a.b': 2.0, 't': 0.0})


if __name__ == '__main__':
    unittest.main()
