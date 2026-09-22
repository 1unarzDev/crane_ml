#!/usr/bin/env python3

import ast
from pathlib import Path
import unittest


class RoboBoatCommandResponseFixtureTests(unittest.TestCase):
    def test_full_forward_suite_applies_and_then_releases_full_surge(self):
        source = Path(__file__).with_name('roboboat_command_response_fixture.py').read_text()
        tree = ast.parse(source)
        assignment = next(
            node for node in tree.body
            if isinstance(node, ast.Assign) and
            any(isinstance(target, ast.Name) and target.id == 'FULL_FORWARD_PHASES'
                for target in node.targets))
        phases = ast.literal_eval(assignment.value)
        active = next(phase for phase in phases if phase[0] == 'full_forward')
        stopped = phases[-1]
        self.assertEqual(active[2:], (1.0, 0.0, 0.0))
        self.assertGreaterEqual(active[1], 15.0)
        self.assertEqual(stopped[2:], (0.0, 0.0, 0.0))
        self.assertGreaterEqual(stopped[1], 15.0)


if __name__ == '__main__':
    unittest.main()
