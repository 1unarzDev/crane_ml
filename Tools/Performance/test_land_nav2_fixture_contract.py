#!/usr/bin/env python3
"""Static regression checks for the graphics-free land Nav2 launcher."""

from pathlib import Path
import unittest


ROOT = Path(__file__).resolve().parents[2]
LAUNCHER = ROOT / "Tools" / "Performance" / "run_land_nav2_fixture.sh"
SHARED_LAUNCHER = ROOT / "Tools" / "Performance" / "run_nav2_controller_fixture.sh"
FIXTURE = ROOT / "Tools" / "Performance" / "nav2_follow_path_fixture.py"
PARAMETERS = ROOT / "Tools" / "Performance" / "nav2_land_fixture.yaml"


class LandNav2FixtureContractTests(unittest.TestCase):
    def test_launcher_selects_the_land_scene_and_graphics_free_profile(self) -> None:
        text = LAUNCHER.read_text(encoding="utf-8")
        shared_text = SHARED_LAUNCHER.read_text(encoding="utf-8")
        self.assertIn("CRANE_SCENE=\"Land Vehicle Validation\"", text)
        self.assertIn("CRANE_NOGRAPHICS=1", text)
        self.assertIn("CRANE_NAV2_PROFILE=train-cpu", text)
        self.assertIn("--crane-profile ${runtime_profile}", shared_text)
        self.assertIn("--crane-land-nav2", text)

    def test_launcher_does_not_route_through_the_aquatic_profile(self) -> None:
        text = LAUNCHER.read_text(encoding="utf-8")
        self.assertNotIn("--crane-profile train-gpu", text)
        self.assertNotIn("Roboboat Course", text)

    def test_costmap_capture_uses_topic_and_service_observation_paths(self) -> None:
        launcher_text = LAUNCHER.read_text(encoding="utf-8")
        shared_text = SHARED_LAUNCHER.read_text(encoding="utf-8")
        fixture_text = FIXTURE.read_text(encoding="utf-8")
        self.assertIn("/local_costmap/costmap}", launcher_text)
        self.assertNotIn("/local_costmap/costmap_raw}", launcher_text)
        self.assertIn("DurabilityPolicy.VOLATILE", fixture_text)
        self.assertIn("OccupancyGrid, args.costmap_topic", fixture_text)
        self.assertIn("GetCostmap, args.costmap_service", fixture_text)
        self.assertIn("costmapServiceSnapshots", fixture_text)
        self.assertIn("CRANE_REQUIRE_OCCUPIED_COSTMAP=1", launcher_text)
        self.assertIn("--require-occupied-costmap", shared_text)
        self.assertIn("--expected-navigation-status", shared_text)

    def test_laser_sources_retain_points_above_the_ground_plane(self) -> None:
        text = PARAMETERS.read_text(encoding="utf-8")
        self.assertEqual(text.count("max_obstacle_height: 2.0"), 2)
        self.assertEqual(text.count("min_obstacle_height: 0.0"), 2)
        self.assertEqual(text.count("always_send_full_costmap: false"), 2)


if __name__ == "__main__":
    unittest.main()
