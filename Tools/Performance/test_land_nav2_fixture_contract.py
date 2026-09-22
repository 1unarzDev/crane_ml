#!/usr/bin/env python3
"""Static regression checks for the graphics-free land Nav2 launcher."""

from pathlib import Path
import unittest


ROOT = Path(__file__).resolve().parents[2]
LAUNCHER = ROOT / "Tools" / "Performance" / "run_land_nav2_fixture.sh"
SHARED_LAUNCHER = ROOT / "Tools" / "Performance" / "run_nav2_controller_fixture.sh"
FIXTURE = ROOT / "Tools" / "Performance" / "nav2_follow_path_fixture.py"
PARAMETERS = ROOT / "Tools" / "Performance" / "nav2_land_fixture.yaml"
WAREHOUSE_PARAMETERS = ROOT / "Tools" / "Performance" / "nav2_warehouse_fixture.yaml"
CLEARPATH_LAUNCHER = ROOT / "Tools" / "Performance" / "run_clearpath_pipeline_nav2_fixture.sh"
WAREHOUSE_LAUNCHER = ROOT / "Tools" / "Performance" / "run_warehouse_nav2_fixture.sh"
WAREHOUSE_BLOCKAGE_LAUNCHER = (
    ROOT / "Tools" / "Performance" / "run_warehouse_blockage_nav2_fixture.sh"
)
WAREHOUSE_NO_PATH_LAUNCHER = (
    ROOT / "Tools" / "Performance" / "run_warehouse_no_path_nav2_fixture.sh"
)
WAREHOUSE_DEADLINE_LAUNCHER = (
    ROOT / "Tools" / "Performance" / "run_warehouse_deadline_nav2_fixture.sh"
)
WAREHOUSE_DEADLINE_BLOCKAGE_LAUNCHER = (
    ROOT / "Tools" / "Performance" / "run_warehouse_deadline_blockage_nav2_fixture.sh"
)
WAREHOUSE_DEADLINE_BT = (
    ROOT / "Tools" / "Performance" / "nav2_warehouse_replanning_deadline.xml"
)
WAREHOUSE_RECOVERY_LAUNCHER = (
    ROOT / "Tools" / "Performance" / "run_warehouse_recovery_nav2_fixture.sh"
)
PROVING_GROUND_LAUNCHER = (
    ROOT / "Tools" / "Performance" / "run_land_proving_ground_nav2_fixture.sh"
)
PROVING_GROUND_SOURCE = (
    ROOT / "Assets" / "Scripts" / "Physics" / "Land" / "CraneLandProvingGround.cs"
)
PROVING_GROUND_PARAMETERS = (
    ROOT / "Tools" / "Performance" / "nav2_land_proving_ground_fixture.yaml"
)


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

    def test_fixture_records_bounded_bt_and_recovery_feedback_metrics(self) -> None:
        text = FIXTURE.read_text(encoding="utf-8")
        self.assertIn("BehaviorTreeLog", text)
        self.assertIn("feedback_callback=self.on_feedback", text)
        self.assertIn("'maximumRecoveryCount'", text)
        self.assertIn("'recoveryCountSequence'", text)
        self.assertIn("'behaviorTreeTransitionCounts'", text)
        self.assertIn("'behaviorTreeCapture'", text)
        self.assertIn("--bt-max-transitions", text)
        self.assertIn("--bt-max-invocations", text)
        self.assertIn("--bt-terminal-drain-seconds", text)
        self.assertIn("mark_goal_accepted(goal_id)", text)
        self.assertIn("may-omit-terminal-tick-not-proof-of-completeness", text)
        self.assertIn("'trajectorySamples'", text)
        self.assertIn("sampled-delivered-odometry-not-proven-nav2-internal-state", text)

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

    def test_proving_ground_is_manifest_selected_without_changing_corridor_truth(self) -> None:
        bootstrap = (ROOT / "Assets/Scripts/Physics/Land/CraneLandNav2Bootstrap.cs").read_text(
            encoding="utf-8"
        )
        source = PROVING_GROUND_SOURCE.read_text(encoding="utf-8")
        launcher = PROVING_GROUND_LAUNCHER.read_text(encoding="utf-8")
        parameters = PROVING_GROUND_PARAMETERS.read_text(encoding="utf-8")
        self.assertIn('ReadString("--crane-land-proving-ground-layout", null)', bootstrap)
        self.assertIn('ReadString("--crane-land-proving-ground-catalog"', bootstrap)
        self.assertIn("CraneLandProvingGround.Build", bootstrap)
        self.assertIn('schema = "crane-land-corridor-truth-v1"', bootstrap)
        self.assertIn('schema = "crane-land-proving-ground-truth-v1"', source)
        self.assertIn("CanonicalGeometry", source)
        self.assertIn("VisualPresentation", source)
        self.assertIn("configurationSha256", source)
        self.assertIn("CraneTimedWarehouseObstacle", source)
        self.assertIn("ConfigureInspection(body.transform, manifest.environmentId, layout)", source)
        self.assertIn("CraneReferenceInspectionController", source)
        self.assertIn("layout.relevantObstacles", source)
        self.assertIn('CRANE_PROVING_GROUND_LAYOUT:-alternate-corridors-v1', launcher)
        self.assertIn('CRANE_PROVING_GROUND_CATALOG:-v1', launcher)
        self.assertIn("--crane-land-proving-ground-catalog", launcher)
        self.assertIn("--crane-land-proving-ground-layout", launcher)
        self.assertIn("nav2_land_proving_ground_fixture.yaml", launcher)
        self.assertIn("width: 44", parameters)
        self.assertIn("height: 12", parameters)
        self.assertNotIn("Roboboat Course", bootstrap + source + launcher)

    def test_differential_command_converts_ros_yaw_to_unity_yaw(self) -> None:
        source = (ROOT / "Assets/Scripts/Physics/Land/ROSDifferentialCommand.cs").read_text(
            encoding="utf-8"
        )
        self.assertIn("robot.SetCommand(command.Linear, -command.Angular)", source)

    def test_clearpath_launcher_preserves_the_imported_reference_environment(self) -> None:
        bootstrap = (ROOT / "Assets/Scripts/Physics/Land/CraneLandNav2Bootstrap.cs").read_text(
            encoding="utf-8"
        )
        launcher = CLEARPATH_LAUNCHER.read_text(encoding="utf-8")
        self.assertIn('"Clearpath Pipeline Validation"', bootstrap)
        self.assertIn("preserveReferenceEnvironment", bootstrap)
        self.assertIn("if (!preserveReferenceEnvironment)", bootstrap)
        self.assertIn('CRANE_SCENE="Clearpath Pipeline Validation"', launcher)
        self.assertIn("--crane-ros-differential-cmd-vel", launcher)
        self.assertIn("base_scan", launcher)
        self.assertIn('CRANE_NAV2_GOAL_DISTANCE="${CRANE_NAV2_GOAL_DISTANCE:-1.0}"', launcher)
        self.assertNotIn("Roboboat Course", launcher)

    def test_ecological_warehouse_launcher_is_separate_from_frozen_corridor(self) -> None:
        bootstrap = (ROOT / "Assets/Scripts/Physics/Land/CraneLandNav2Bootstrap.cs").read_text(
            encoding="utf-8"
        )
        ecological = WAREHOUSE_LAUNCHER.read_text(encoding="utf-8")
        frozen = (ROOT / "Tools/Performance/run_turtlebot3_nav2_fixture.sh").read_text(
            encoding="utf-8"
        )
        self.assertIn("--crane-preserve-reference-environment", bootstrap)
        self.assertIn("turtlebotScene && preserveRequested", bootstrap)
        self.assertIn("--crane-preserve-reference-environment", ecological)
        self.assertIn("warehouse-cross-aisle-detour-v1", ecological)
        self.assertIn("nav2_warehouse_fixture.yaml", ecological)
        self.assertIn('CRANE_NAV2_GOAL_DISTANCE="${CRANE_NAV2_GOAL_DISTANCE:-13.0}"', ecological)
        self.assertIn('CRANE_NAV2_ACTION_DURATION="${CRANE_NAV2_ACTION_DURATION:-75}"', ecological)
        self.assertIn('CRANE_DURATION="${CRANE_DURATION:-90}"', ecological)
        self.assertNotIn("--crane-preserve-reference-environment", frozen)

    def test_warehouse_blockage_launcher_preserves_observed_client_timeout(self) -> None:
        ecological = WAREHOUSE_LAUNCHER.read_text(encoding="utf-8")
        blockage = WAREHOUSE_BLOCKAGE_LAUNCHER.read_text(encoding="utf-8")
        bootstrap = (ROOT / "Assets/Scripts/Physics/Land/CraneLandNav2Bootstrap.cs").read_text(
            encoding="utf-8"
        )
        self.assertIn("CRANE_WAREHOUSE_SCENARIO_ID", ecological)
        self.assertIn("--crane-warehouse-scenario-id", ecological)
        self.assertIn("warehouse-cross-aisle-complete-blockage-v1", blockage)
        self.assertIn('CRANE_EXPECTED_NAV_STATUS="${CRANE_EXPECTED_NAV_STATUS:-timeout}"', blockage)
        self.assertIn("not a Nav2 abort", blockage)
        self.assertIn('ReadString("--crane-warehouse-scenario-id"', bootstrap)
        self.assertIn("CopyWarehouseScenarioTruth", bootstrap)
        self.assertNotIn("Roboboat Course", blockage)

    def test_warehouse_no_path_launcher_uses_visible_occupied_goal_calibration(self) -> None:
        text = WAREHOUSE_NO_PATH_LAUNCHER.read_text(encoding="utf-8")
        self.assertIn("warehouse-occupied-goal-no-path-v1", text)
        self.assertIn('CRANE_NAV2_GOAL_DISTANCE="${CRANE_NAV2_GOAL_DISTANCE:-3.0}"', text)
        self.assertIn('CRANE_NAV2_ACTION_DURATION="${CRANE_NAV2_ACTION_DURATION:-45}"', text)
        self.assertIn('CRANE_EXPECTED_NAV_STATUS="${CRANE_EXPECTED_NAV_STATUS:-timeout}"', text)
        self.assertIn("must not be relabeled as Nav2 failure", text)
        self.assertNotIn("Roboboat Course", text)

    def test_warehouse_deadline_policy_is_explicit_and_separate_from_frozen_tree(self) -> None:
        launcher = WAREHOUSE_DEADLINE_LAUNCHER.read_text(encoding="utf-8")
        blockage = WAREHOUSE_DEADLINE_BLOCKAGE_LAUNCHER.read_text(encoding="utf-8")
        tree = WAREHOUSE_DEADLINE_BT.read_text(encoding="utf-8")
        frozen_tree = (ROOT / "Tools/Performance/nav2_land_progress_recovery.xml").read_text(
            encoding="utf-8"
        )
        self.assertIn("nav2_warehouse_replanning_deadline.xml", launcher)
        self.assertIn('CRANE_NAV2_ACTION_DURATION="${CRANE_NAV2_ACTION_DURATION:-80}"', launcher)
        self.assertIn('CRANE_DURATION="${CRANE_DURATION:-95}"', launcher)
        self.assertIn('<Timeout msec="70000">', tree)
        self.assertNotIn("<TimeExpired", tree)
        self.assertIn('<RateController hz="1.0">', tree)
        self.assertIn('<RecoveryNode number_of_retries="6" name="NavigateRecovery">', tree)
        self.assertIn("warehouse-cross-aisle-complete-blockage-v1", blockage)
        self.assertIn('CRANE_EXPECTED_NAV_STATUS="${CRANE_EXPECTED_NAV_STATUS:-aborted}"', blockage)
        self.assertNotEqual(tree, frozen_tree)
        self.assertNotIn("<Timeout", frozen_tree)
        self.assertNotIn("Roboboat Course", launcher + blockage + tree)

    def test_warehouse_recovery_launcher_selects_temporary_physical_enclosure(self) -> None:
        text = WAREHOUSE_RECOVERY_LAUNCHER.read_text(encoding="utf-8")
        tree = (
            ROOT / "Tools/Performance/nav2_warehouse_replanning_recovery.xml"
        ).read_text(encoding="utf-8")
        self.assertIn("warehouse-temporary-enclosure-recovery-v1", text)
        self.assertIn("nav2_warehouse_replanning_recovery.xml", text)
        self.assertIn('CRANE_NAV2_ACTION_DURATION="${CRANE_NAV2_ACTION_DURATION:-100}"', text)
        self.assertIn('<Timeout msec="90000">', tree)
        self.assertIn('CRANE_EXPECTED_NAV_STATUS="${CRANE_EXPECTED_NAV_STATUS:-succeeded}"', text)
        self.assertNotIn("Roboboat Course", text + tree)

    def test_warehouse_costmap_matches_manifest_robot_and_preserves_frozen_params(self) -> None:
        warehouse = WAREHOUSE_PARAMETERS.read_text(encoding="utf-8")
        controlled = PARAMETERS.read_text(encoding="utf-8")
        self.assertEqual(warehouse.count("robot_radius: 0.22"), 2)
        self.assertEqual(warehouse.count("inflation_radius: 0.55"), 2)
        self.assertIn("desired_linear_vel: 0.26", warehouse)
        self.assertIn("yaw_goal_tolerance: 3.14", warehouse)
        self.assertEqual(controlled.count("robot_radius: 0.75"), 2)
        self.assertEqual(controlled.count("inflation_radius: 0.9"), 2)
        self.assertIn("desired_linear_vel: 0.8", controlled)
        self.assertIn("yaw_goal_tolerance: 0.35", controlled)

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
