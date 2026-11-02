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

from backends import unity
from crane_cluster.backends import JobContext

FAKE_PLAYER = textwrap.dedent("""\
    #!{python}
    import json, sys
    a = sys.argv
    arg = lambda k: a[a.index(k) + 1]
    open(arg('-logFile'), 'w').write(' '.join(a))
    if arg('--crane-course-seed') == '13':
        sys.exit(4)
    bad = 21 if arg('--crane-course-seed') == '21' else 0
    json.dump({{'name': 'c', 'generator': {{'seed': int(arg('--crane-course-seed'))}}}}, open(arg('--crane-course-truth'), 'w'))
    json.dump({{'schema': 'crane-benchmark-v1', 'scene': arg('--crane-scene'), 'simulatedSeconds': float(arg('--crane-duration')),
               'valid': not bad, 'invalidWaterSearches': bad, 'validationSamplesCaptured': 21,
               'validation': [{{'water': [{{'simulationTime': 0.0, 'valid': not bad}}]}}], 'markers': [{{'name': 'Physics', 'meanMillisecondsPerFrame': 1.5}}], 'samples': [1, 2]}},
              open(arg('--crane-output'), 'w'))
""")


class UnityBackendTest(unittest.TestCase):
    def setUp(self):
        self.tmp = Path(tempfile.mkdtemp())
        self.player = self.tmp / 'CRANE.x86_64'
        self.player.write_text(FAKE_PLAYER.format(python=sys.executable))
        self.player.chmod(self.player.stat().st_mode | stat.S_IEXEC)

    def backend(self, spec, **options):
        b = unity.ClusterBackend('unity', {'player': str(self.player), **options})
        b.prepare(JobContext(job={'job_id': 'j', 'spec': spec, 'seeds': [7]}, work_dir=self.tmp / 'job'))
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
        self.assertEqual([n for _, n, _ in b.artifacts(run_dir)], ['result.json', 'course_truth.json', 'player.log'])

    def test_invalid_water_fails_the_seed_unless_allowed(self):
        with self.assertRaisesRegex(RuntimeError, '21 invalid water searches in 21 validation samples'):
            self.backend({}).run(21)
        b = self.backend({'require_valid_water': False})
        m = b.metrics(b.run(21))
        self.assertEqual((m['water_valid'], m['invalidWaterSearches'], m['valid']), (0.0, 21.0, 0.0))
        self.assertEqual(self.backend({}).metrics(self.backend({}).run(7))['water_valid'], 1.0)

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
