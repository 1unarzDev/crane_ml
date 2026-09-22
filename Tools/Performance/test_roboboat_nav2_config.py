#!/usr/bin/env python3

from pathlib import Path
import unittest

import yaml


class RoboBoatNav2ConfigTests(unittest.TestCase):
    def test_fast_cruise_retains_slow_terminal_heading_control(self):
        path = Path(__file__).with_name('nav2_controller_fixture.yaml')
        config = yaml.safe_load(path.read_text(encoding='utf-8'))
        controller = config['controller_server']['ros__parameters']
        follow_path = controller['FollowPath']

        self.assertEqual(
            follow_path['plugin'],
            'nav2_regulated_pure_pursuit_controller::RegulatedPurePursuitController')
        self.assertEqual(follow_path['desired_linear_vel'], 0.8)
        self.assertEqual(follow_path['lookahead_dist'], 2.0)
        self.assertFalse(follow_path['use_velocity_scaled_lookahead_dist'])
        self.assertTrue(follow_path['use_rotate_to_heading'])
        self.assertEqual(follow_path['rotate_to_heading_angular_vel'], 0.2)
        self.assertEqual(follow_path['min_approach_linear_velocity'], 0.02)
        self.assertEqual(follow_path['approach_velocity_scaling_dist'], 2.0)

        goal_checker = controller['goal_checker']
        self.assertEqual(goal_checker['plugin'], 'nav2_controller::StoppedGoalChecker')
        self.assertEqual(goal_checker['xy_goal_tolerance'], 0.20)
        self.assertEqual(goal_checker['yaw_goal_tolerance'], 0.35)


if __name__ == '__main__':
    unittest.main()
