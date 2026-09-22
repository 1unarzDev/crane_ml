#!/usr/bin/env python3

import json
from pathlib import Path
import tempfile
import unittest

from Tools.Performance.nav2_path_spec import load_path_spec


class Nav2PathSpecTests(unittest.TestCase):
    def write(self, value):
        directory = tempfile.TemporaryDirectory()
        path = Path(directory.name) / 'path.json'
        path.write_text(json.dumps(value), encoding='utf-8')
        self.addCleanup(directory.cleanup)
        return path

    def test_loads_finite_odom_path_and_retains_digest(self):
        path = self.write({
            'schema': 'crane-nav2-path-v1',
            'frameId': 'odom',
            'poses': [{'x': 1, 'y': 2.5, 'yaw': -1.57}],
        })
        result = load_path_spec(path)
        self.assertEqual(result['frameId'], 'odom')
        self.assertEqual(result['poses'], [{'x': 1.0, 'y': 2.5, 'yaw': -1.57}])
        self.assertEqual(len(result['sourceSha256']), 64)

    def test_rejects_wrong_schema(self):
        path = self.write({'schema': 'wrong', 'frameId': 'odom', 'poses': []})
        with self.assertRaisesRegex(ValueError, 'schema'):
            load_path_spec(path)

    def test_rejects_nonfinite_pose(self):
        path = self.write({
            'schema': 'crane-nav2-path-v1',
            'frameId': 'odom',
            'poses': [{'x': float('nan'), 'y': 0, 'yaw': 0}],
        })
        with self.assertRaisesRegex(ValueError, 'finite'):
            load_path_spec(path)


if __name__ == '__main__':
    unittest.main()
