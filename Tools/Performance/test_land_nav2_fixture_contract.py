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
        self.assertIn('CRANE_SCENE="${CRANE_SCENE:-Land Vehicle Validation}"', text)
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
        self.assertIn("latestCostmapSnapshot", fixture_text)
        self.assertIn("CRANE_NAV2_COSTMAP_SERVICE", shared_text)
        self.assertIn("CRANE_DOCKING_EVALUATOR", shared_text)
        self.assertIn("dockingSuccessObserved", fixture_text)
        self.assertIn("CRANE_NAV2_PATH_FILE", shared_text)
        self.assertIn("suppliedPathFileSha256", fixture_text)
        self.assertIn("CRANE_REQUIRE_OCCUPIED_COSTMAP=1", launcher_text)
        self.assertIn("--require-occupied-costmap", shared_text)
        self.assertIn("--expected-navigation-status", shared_text)

    def test_laser_sources_retain_points_above_the_ground_plane(self) -> None:
        text = PARAMETERS.read_text(encoding="utf-8")
        self.assertEqual(text.count("max_obstacle_height: 2.0"), 2)
        self.assertEqual(text.count("min_obstacle_height: 0.0"), 2)
        self.assertEqual(text.count("always_send_full_costmap: false"), 2)

    def test_controller_overrides_are_tokenized_without_eval(self) -> None:
        text = SHARED_LAUNCHER.read_text(encoding="utf-8")
        self.assertIn("CRANE_NAV2_CONTROLLER_EXTRA_ARGS", text)
        self.assertIn('read -r -a controller_extra', text)
        self.assertIn('"${controller_extra[@]}"', text)
        self.assertNotIn("eval ", text)

    def test_optional_behavior_tree_is_scoped_to_the_repository(self) -> None:
        text = SHARED_LAUNCHER.read_text(encoding="utf-8")
        self.assertIn("CRANE_NAV2_BT_XML", text)
        self.assertIn("Behavior tree XML must be inside the CRANE repository", text)
        self.assertIn("default_nav_to_pose_bt_xml", text)

    def test_identity_is_republished_at_the_accepted_goal_boundary(self) -> None:
        text = FIXTURE.read_text(encoding="utf-8")
        self.assertIn("def publish_identity(self, reason)", text)
        self.assertIn("self.publish_identity('initial_observation')", text)
        self.assertIn("self.publish_identity('accepted_goal_republication')", text)
        self.assertIn("delivered_to_fixture_not_proven_consumed_by_nav2", text)

    def test_harness_boundary_events_survive_dds_discovery(self) -> None:
        text = FIXTURE.read_text(encoding="utf-8")
        harness_section = text.split("harness_qos = QoSProfile(", 1)[1].split(")", 1)[0]
        self.assertIn("ReliabilityPolicy.RELIABLE", harness_section)
        self.assertIn("DurabilityPolicy.TRANSIENT_LOCAL", harness_section)
        self.assertIn("depth=20", harness_section)

    def test_land_bootstrap_provides_simulated_clock_for_nav2(self) -> None:
        text = (ROOT / "Assets/Scripts/Physics/Land/CraneLandNav2Bootstrap.cs").read_text(
            encoding="utf-8"
        )
        self.assertIn("FindAnyObjectByType<ROSClock>()", text)
        self.assertIn('AddComponent<ROSClock>()', text)

    def test_timed_blocker_removal_is_evaluator_only_and_uses_simulation_time(self) -> None:
        bootstrap = (ROOT / "Assets/Scripts/Physics/Land/CraneLandNav2Bootstrap.cs").read_text(
            encoding="utf-8"
        )
        removal = (ROOT / "Assets/Scripts/Physics/Land/CraneTimedBlockerRemoval.cs").read_text(
            encoding="utf-8"
        )
        self.assertIn('ReadFloat("--crane-land-blocker-remove-after"', bootstrap)
        self.assertIn('ReadFloat("--crane-land-blocker-enable-after"', bootstrap)
        self.assertIn("Time.fixedTimeAsDouble", removal)
        self.assertIn("blockerActivationActualSimulationTime", bootstrap)
        self.assertIn("blockerRemovalActualSimulationTime", bootstrap)
        self.assertIn("target.SetActive(true)", removal)
        self.assertIn("target.SetActive(false)", removal)
        self.assertIn('"corridor-blocker"', bootstrap)

    def test_reference_import_documentation_is_graphics_free(self) -> None:
        text = (ROOT / "Docs/ReferenceEnvironments.md").read_text(encoding="utf-8")
        clearpath_section = text.split("## Clearpath pipeline offline import", 1)[1]
        command = clearpath_section.split("```bash", 1)[1].split("```", 1)[0]
        self.assertIn("-nographics", command)

    def test_corridor_bootstrap_supports_existing_turtlebot3_differential_scene(self) -> None:
        bootstrap = (ROOT / "Assets/Scripts/Physics/Land/CraneLandNav2Bootstrap.cs").read_text(
            encoding="utf-8"
        )
        launcher = (ROOT / "Tools/Performance/run_turtlebot3_nav2_fixture.sh").read_text(
            encoding="utf-8"
        )
        self.assertIn('"TurtleBot3 Warehouse Validation"', bootstrap)
        self.assertIn("FindAnyObjectByType<DifferentialDriveDynamics>", bootstrap)
        self.assertIn("CraneReferenceWarehouse", bootstrap)
        self.assertIn('CRANE_SCENE="TurtleBot3 Warehouse Validation"', launcher)
        self.assertIn("--crane-ros-differential-cmd-vel", launcher)
        self.assertIn("base_scan", launcher)

    def test_mobility_hold_is_fixed_time_and_evaluator_owned(self) -> None:
        bootstrap = (ROOT / "Assets/Scripts/Physics/Land/CraneLandNav2Bootstrap.cs").read_text(
            encoding="utf-8"
        )
        hold = (ROOT / "Assets/Scripts/Physics/Land/CraneTimedMobilityHold.cs").read_text(
            encoding="utf-8"
        )
        self.assertIn('ReadFloat("--crane-land-mobility-hold-after"', bootstrap)
        self.assertIn('ReadFloat("--crane-land-mobility-release-after"', bootstrap)
        self.assertIn("mobilityHoldActualSimulationTime", bootstrap)
        self.assertIn("mobilityReleaseActualSimulationTime", bootstrap)
        self.assertIn("Time.fixedTimeAsDouble", hold)
        self.assertIn("RigidbodyConstraints.FreezePositionX", hold)
        self.assertIn("RigidbodyConstraints.FreezePositionZ", hold)
        self.assertIn("target.constraints = originalConstraints", hold)
        self.assertIn("releaseAfterSeconds >= 0f", hold)
        self.assertIn("ScheduledReleaseSimulationTime < 0d", hold)
        self.assertIn("Mobility release requires a configured hold boundary", bootstrap)
        self.assertNotIn(
            "(mobilityHoldAfter >= 0f) != (mobilityReleaseAfter >= 0f)", bootstrap
        )


if __name__ == "__main__":
    unittest.main()
