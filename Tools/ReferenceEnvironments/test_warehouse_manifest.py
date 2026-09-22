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
        self.assertEqual(self.manifest["generatorVersion"], "2.0.0")
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
        for route in self.manifest["routes"]:
            self.assertTrue(set(route["relevantObstacles"]).issubset(box_ids))

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


if __name__ == "__main__":
    unittest.main()
