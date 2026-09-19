#!/usr/bin/env python3
"""Static regression checks for the graphics-free land Nav2 launcher."""

from pathlib import Path
import unittest


ROOT = Path(__file__).resolve().parents[2]
LAUNCHER = ROOT / "Tools" / "Performance" / "run_land_nav2_fixture.sh"
SHARED_LAUNCHER = ROOT / "Tools" / "Performance" / "run_nav2_controller_fixture.sh"


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


if __name__ == "__main__":
    unittest.main()
