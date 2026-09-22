#!/usr/bin/env python3
import argparse
import hashlib
import json
import tempfile
import unittest
from pathlib import Path

from summarize_environment_qa import summarize


class EnvironmentQaSummaryTests(unittest.TestCase):
    def test_merges_independent_nominal_gates_without_inventing_interactive_or_failure_pass(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            manifest = {
                "environmentId": "warehouse-v2",
                "routes": [{"id": "detour", "approximateLengthMeters": 12.0}],
            }
            manifest_path = root / "manifest.json"
            manifest_path.write_text(json.dumps(manifest), encoding="utf-8")
            structural = {
                "environmentId": "warehouse-v2",
                "manifestSha256": hashlib.sha256(manifest_path.read_bytes()).hexdigest(),
                "structuralStatus": "STRUCTURAL_PASS",
                "physicsStatus": "PHYSICS_PASS",
                "sensorStatus": "SENSOR_PASS",
                "headlessStatus": "HEADLESS_PASS",
                "explanationStatus": "EXPLANATION_READY",
            }
            navigation = {
                "valid": True,
                "navigation": {"status": "succeeded", "displacementMeters": 11.5},
            }
            truth = {
                "environmentId": "warehouse-v2",
                "scenarioId": "detour",
                "platform": "turtlebot3",
                "referenceEnvironmentPreserved": True,
            }
            runtime = {"validation": [
                {"simulatedSeconds": 0, "bodies": [{"position": {"x": 0, "z": 0}}]},
                {"simulatedSeconds": 1, "bodies": [{"position": {"x": -2, "z": 3}}]},
                {"simulatedSeconds": 2, "bodies": [{"position": {"x": 0, "z": 6}}]},
            ]}
            paths = {}
            for name, value in (("structural", structural), ("navigation", navigation),
                                ("truth", truth), ("runtime", runtime)):
                paths[name] = root / f"{name}.json"
                paths[name].write_text(json.dumps(value), encoding="utf-8")
            result = summarize(argparse.Namespace(
                structural=paths["structural"], navigation=paths["navigation"],
                evaluator_truth=paths["truth"], runtime_result=paths["runtime"],
                scenario_manifest=manifest_path,
            ))
        self.assertTrue(result["identityValid"])
        self.assertTrue(result["nominalRouteReady"])
        self.assertEqual(result["gates"]["navigation"], "NAVIGATION_PASS")
        self.assertEqual(result["gates"]["interactive"], "NOT_RUN")
        self.assertEqual(result["gates"]["failureRecovery"], "NOT_RUN")
        self.assertAlmostEqual(result["trajectory"]["maximumLateralExcursionMeters"], 2.0)
        self.assertGreater(result["trajectory"]["sampledPathLengthMeters"], 7.0)


if __name__ == "__main__":
    unittest.main()
