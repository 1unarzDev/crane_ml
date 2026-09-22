#!/usr/bin/env python3
import json
from pathlib import Path
import unittest


ROOT = Path(__file__).resolve().parents[2]
MANIFEST = (
    ROOT
    / "Assets"
    / "Resources"
    / "ReferenceEnvironments"
    / "unity_turtlebot3_industrial_warehouse_v2.json"
)


class WarehouseManifestTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls) -> None:
        cls.manifest = json.loads(MANIFEST.read_text(encoding="utf-8"))

    def test_manifest_has_versioned_identity_and_large_canonical_footprint(self) -> None:
        self.assertEqual(self.manifest["schema"], "crane-environment-scenario-catalog-v1")
        self.assertEqual(self.manifest["environmentId"], "crane-industrial-warehouse-v2")
        self.assertEqual(self.manifest["generatorVersion"], "2.1.0")
        self.assertEqual(self.manifest["canonical"]["dimensionsMeters"], [20.0, 26.0])

    def test_semantic_ids_are_unique_and_route_references_resolve(self) -> None:
        canonical = self.manifest["canonical"]
        objects = canonical["boxes"] + canonical["regions"]
        ids = [value["id"] for value in objects]
        route_ids = [value["id"] for value in self.manifest["routes"]]
        self.assertEqual(len(ids), len(set(ids)))
        self.assertEqual(len(route_ids), len(set(route_ids)))
        self.assertFalse(set(ids) & set(route_ids))
        box_ids = {value["id"] for value in canonical["boxes"]}
        scenario_obstacle_ids = {
            obstacle["id"]
            for scenario in self.manifest["scenarios"]
            for obstacle in scenario["obstacles"]
        }
        for route in self.manifest["routes"]:
            self.assertTrue(
                set(route["relevantObstacles"]).issubset(box_ids | scenario_obstacle_ids)
            )

    def test_layout_contains_required_navigation_motifs(self) -> None:
        roles = {value["role"] for value in self.manifest["canonical"]["boxes"]}
        region_roles = {value["role"] for value in self.manifest["canonical"]["regions"]}
        self.assertTrue(
            {
                "shelving-obstacle",
                "route-divider",
                "structural-column",
                "movable-clutter",
                "distractor-obstacle",
                "dead-end-wall",
                "narrow-gate",
            }.issubset(roles)
        )
        self.assertTrue(
            {
                "navigation-corridor",
                "navigation-cross-aisle",
                "open-work-zone",
                "dead-end-region",
            }.issubset(region_roles)
        )

    def test_primary_route_requires_a_long_multi_route_detour(self) -> None:
        route = next(
            value for value in self.manifest["routes"]
            if value["id"] == "warehouse-detour-north-v1"
        )
        self.assertGreaterEqual(route["approximateLengthMeters"], 20.0)
        self.assertGreaterEqual(len(route["alternatives"]), 2)
        self.assertIn("rack-center-blocker", route["relevantObstacles"])
        self.assertEqual(route["expectedBroadOutcome"], "success-with-multi-turn-detour")

    def test_complete_blockage_spans_the_traversable_warehouse_width(self) -> None:
        scenario = next(
            value for value in self.manifest["scenarios"]
            if value["id"] == "warehouse-cross-aisle-complete-blockage-v1"
        )
        obstacle = scenario["obstacles"][0]
        center_x = obstacle["center"][0]
        half_width = obstacle["size"][0] * 0.5
        # Boundary-wall inner faces are x=-9.9 and x=+9.9.
        self.assertLessEqual(center_x - half_width, -9.9)
        self.assertGreaterEqual(center_x + half_width, 9.9)
        self.assertTrue(obstacle["activeInitially"])
        self.assertEqual(obstacle["activationAfterSeconds"], -1.0)
        self.assertEqual(obstacle["removalAfterSeconds"], -1.0)
        self.assertEqual(scenario["routeId"], "warehouse-cross-aisle-detour-v1")

    def test_dynamic_scenarios_have_explicit_non_overlapping_boundaries(self) -> None:
        scenarios = {value["id"]: value for value in self.manifest["scenarios"]}
        delayed = scenarios["warehouse-cross-aisle-delayed-blockage-v1"]["obstacles"][0]
        temporary = scenarios["warehouse-cross-aisle-temporary-blockage-v1"]["obstacles"][0]
        self.assertFalse(delayed["activeInitially"])
        self.assertGreaterEqual(delayed["activationAfterSeconds"], 0.0)
        self.assertEqual(delayed["removalAfterSeconds"], -1.0)
        self.assertTrue(temporary["activeInitially"])
        self.assertEqual(temporary["activationAfterSeconds"], -1.0)
        self.assertGreaterEqual(temporary["removalAfterSeconds"], 0.0)

    def test_scenario_identities_and_obstacles_are_unique(self) -> None:
        scenario_ids = [value["id"] for value in self.manifest["scenarios"]]
        obstacle_ids = [
            obstacle["id"]
            for scenario in self.manifest["scenarios"]
            for obstacle in scenario["obstacles"]
        ]
        self.assertEqual(len(scenario_ids), len(set(scenario_ids)))
        self.assertEqual(len(obstacle_ids), len(set(obstacle_ids)))
        route_ids = {value["id"] for value in self.manifest["routes"]}
        for scenario in self.manifest["scenarios"]:
            self.assertIn(scenario["routeId"], route_ids)
            self.assertTrue(scenario["robot"])
            self.assertTrue(scenario["expectedChallenge"])
            self.assertTrue(scenario["expectedBroadOutcome"])

    def test_occupied_goal_is_inside_obstacle_beyond_goal_tolerance(self) -> None:
        route = next(
            value for value in self.manifest["routes"]
            if value["id"] == "warehouse-blocked-goal-v1"
        )
        scenario = next(
            value for value in self.manifest["scenarios"]
            if value["id"] == "warehouse-occupied-goal-no-path-v1"
        )
        obstacle = scenario["obstacles"][0]
        self.assertEqual(route["goal"], obstacle["center"][:1] + [0.08] + obstacle["center"][2:])
        self.assertGreater(obstacle["size"][0] * 0.5, 0.55)
        self.assertGreater(obstacle["size"][2] * 0.5, 0.55)
        self.assertEqual(scenario["routeId"], route["id"])


if __name__ == "__main__":
    unittest.main()
