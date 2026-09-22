#!/usr/bin/env python3

import math
import unittest

from Tools.Performance.plot_nav2_fixture import dock_polygon, nearest_path_error


class PlotNav2FixtureTests(unittest.TestCase):
    def test_nearest_path_error_reports_lateral_and_heading_error(self):
        planned = [
            {'x': 0.0, 'y': 0.0, 'yaw': 0.0},
            {'x': 2.0, 'y': 0.0, 'yaw': 0.0},
        ]
        distance, heading = nearest_path_error(
            {'x': 1.0, 'y': 0.25, 'yaw': 0.1}, planned)
        self.assertAlmostEqual(distance, 0.25)
        self.assertAlmostEqual(heading, 0.1)

    def test_dock_polygon_rotates_depth_axis_with_goal_yaw(self):
        polygon = dock_polygon(1.0, 2.0, math.pi / 2, depth=4.0, width=2.0)
        xs = [point[0] for point in polygon]
        ys = [point[1] for point in polygon]
        self.assertAlmostEqual(max(xs) - min(xs), 2.0)
        self.assertAlmostEqual(max(ys) - min(ys), 4.0)
        self.assertEqual(polygon[0], polygon[-1])


if __name__ == '__main__':
    unittest.main()
