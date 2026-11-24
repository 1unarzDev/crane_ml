"""The crane-command backend runs a configured argv per seed and reads its metrics.json."""
import sys
import tempfile
import unittest
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))
from crane_backends import command  # noqa: E402
from crane_cluster.backends import JobContext  # noqa: E402

TASK = ("import json, os, sys; seed = int(sys.argv[1]); assert seed != 13, 'unlucky'; "
        "json.dump({'score': seed * 2, 'ok': True, 'in_wd': int(os.path.basename(os.getcwd()) == 'wd'), "
        "'env_seed': int(os.environ['CRANE_SEED'])}, open(os.path.join(sys.argv[2], 'metrics.json'), 'w'))")


class CommandBackendTest(unittest.TestCase):
    def test_seeds_fill_the_argv_and_metrics_are_numbers_only(self):
        tmp = Path(tempfile.mkdtemp())
        (tmp / 'wd').mkdir()
        b = command.ClusterBackend('task', {'argv': [sys.executable, '-c', TASK, '{seed}', '{out}'], 'root': str(tmp)})
        b.prepare(JobContext(job={'job_id': 'j', 'spec': {'workdir': 'wd'}, 'seeds': [7]}, work_dir=tmp / 'job'))
        run_dir = b.run(7)
        self.assertEqual(run_dir, tmp / 'job' / 'runs' / 's7')
        self.assertEqual(b.metrics(run_dir), {'score': 14.0, 'in_wd': 1.0, 'env_seed': 7.0})   # booleans dropped
        with self.assertRaisesRegex(RuntimeError, 'unlucky'):
            b.run(13)


if __name__ == '__main__':
    unittest.main()
