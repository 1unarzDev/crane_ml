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
            structural_hash = hashlib.sha256(paths["structural"].read_bytes()).hexdigest()
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
        self.assertEqual(result["scenarioKind"], "warehouse-route")
        self.assertEqual(result["artifactSha256"]["structural"], structural_hash)

    def test_merges_proving_ground_layout_and_predeclared_failure_outcome(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            manifest = {
                "environmentId": "proving-v1",
                "generatorVersion": "1.0.0",
                "layouts": [{
                    "id": "blocked-v1", "robot": "turtlebot3",
                    "alternatives": [], "relevantObstacles": ["wall"],
                    "expectedChallenge": "blocked", "expectedBroadOutcome": "abort",
                }],
            }
            manifest_path = root / "manifest.json"
            manifest_path.write_text(json.dumps(manifest), encoding="utf-8")
            manifest_hash = hashlib.sha256(manifest_path.read_bytes()).hexdigest()
            seed = 17
            configuration_hash = hashlib.sha256(
                f"{manifest_hash}\nblocked-v1\n{seed}".encode("utf-8")
            ).hexdigest()
            values = {
                "structural": {
                    "environmentId": "proving-v1", "manifestSha256": manifest_hash,
                    "structuralStatus": "STRUCTURAL_PASS", "physicsStatus": "PHYSICS_PASS",
                    "sensorStatus": "SENSOR_PASS", "headlessStatus": "HEADLESS_PASS",
                    "explanationStatus": "EXPLANATION_READY",
                },
                "navigation": {
                    "valid": True, "expectedNavigationStatus": "aborted",
                    "expectedOutcomeObserved": True,
                    "navigation": {"status": "aborted", "displacementMeters": 4.0,
                                   "maximumRecoveryCount": 0},
                },
                "truth": {
                    "schema": "crane-land-proving-ground-truth-v1",
                    "environmentId": "proving-v1", "generatorVersion": "1.0.0",
                    "manifestSha256": manifest_hash,
                    "configurationSha256": configuration_hash, "layoutId": "blocked-v1",
                    "seed": seed, "robot": "turtlebot3", "alternatives": [],
                    "relevantObstacles": ["wall"], "expectedChallenge": "blocked",
                    "expectedBroadOutcome": "abort",
                },
                "runtime": {"validation": [
                    {"simulatedSeconds": 0, "bodies": [{"position": {"x": 0, "z": 0}}]},
                    {"simulatedSeconds": 1, "bodies": [{"position": {"x": 1, "z": 4}}]},
                    {"simulatedSeconds": 2, "bodies": [{"position": {"x": 2, "z": 3}}]},
                ]},
            }
            paths = {}
            for name, value in values.items():
                paths[name] = root / f"{name}.json"
                paths[name].write_text(json.dumps(value), encoding="utf-8")
            result = summarize(argparse.Namespace(
                structural=paths["structural"], navigation=paths["navigation"],
                evaluator_truth=paths["truth"], runtime_result=paths["runtime"],
                scenario_manifest=manifest_path,
            ))
        self.assertTrue(result["identityValid"])
        self.assertTrue(result["routeReady"])
        self.assertEqual(result["scenarioKind"], "proving-ground-layout")
        self.assertEqual(result["gates"]["navigation"], "NAVIGATION_PASS")
        self.assertEqual(result["gates"]["failureRecovery"], "FAILURE_RECOVERY_PASS")
        self.assertEqual(result["trajectory"]["longitudinalReversalSampleCount"], 1)

    def test_behavioral_gate_rejects_success_that_does_not_exercise_declared_route_shape(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            manifest = {
                "environmentId": "proving-v1", "generatorVersion": "1.0.0",
                "layouts": [{
                    "id": "slalom-v1", "robot": "turtlebot3", "alternatives": [],
                    "relevantObstacles": ["post"], "expectedChallenge": "S-turn",
                    "expectedBroadOutcome": "success",
                }],
            }
            manifest_path = root / "manifest.json"
            manifest_path.write_text(json.dumps(manifest), encoding="utf-8")
            manifest_hash = hashlib.sha256(manifest_path.read_bytes()).hexdigest()
            seed = 5
            values = {
                "structural": {
                    "environmentId": "proving-v1", "manifestSha256": manifest_hash,
                    "structuralStatus": "STRUCTURAL_PASS", "physicsStatus": "PHYSICS_PASS",
                    "sensorStatus": "SENSOR_PASS", "headlessStatus": "HEADLESS_PASS",
                    "explanationStatus": "EXPLANATION_READY",
                },
                "navigation": {"valid": True, "navigation": {
                    "status": "succeeded", "displacementMeters": 18.0,
                    "maximumRecoveryCount": 0,
                }},
                "truth": {
                    "schema": "crane-land-proving-ground-truth-v1",
                    "environmentId": "proving-v1", "generatorVersion": "1.0.0",
                    "manifestSha256": manifest_hash, "layoutId": "slalom-v1", "seed": seed,
                    "configurationSha256": hashlib.sha256(
                        f"{manifest_hash}\nslalom-v1\n{seed}".encode("utf-8")
                    ).hexdigest(),
                    "robot": "turtlebot3", "alternatives": [],
                    "relevantObstacles": ["post"], "expectedChallenge": "S-turn",
                    "expectedBroadOutcome": "success",
                },
                "runtime": {"validation": [
                    {"simulatedSeconds": 0, "bodies": [{"position": {"x": 0, "z": 0}}]},
                    {"simulatedSeconds": 1, "bodies": [{"position": {"x": 0, "z": 9}}]},
                    {"simulatedSeconds": 2, "bodies": [{"position": {"x": 0, "z": 18}}]},
                ]},
                "gates": {"environmentId": "proving-v1", "layouts": {"slalom-v1": {
                    "minimumPositiveLateralMeters": 0.5,
                    "maximumNegativeLateralMeters": -0.5,
                    "minimumLateralDirectionChanges": 2,
                }}},
            }
            paths = {}
            for name, value in values.items():
                paths[name] = root / f"{name}.json"
                paths[name].write_text(json.dumps(value), encoding="utf-8")
            result = summarize(argparse.Namespace(
                structural=paths["structural"], navigation=paths["navigation"],
                evaluator_truth=paths["truth"], runtime_result=paths["runtime"],
                scenario_manifest=manifest_path, navigation_gates=paths["gates"],
            ))
        self.assertTrue(result["identityValid"])
        self.assertEqual(result["routeAcceptance"]["status"], "FAIL")
        self.assertFalse(result["routeReady"])
        self.assertEqual(result["gates"]["navigation"], "PARTIAL")
        self.assertEqual(result["verdict"], "BLOCKED")


if __name__ == "__main__":
    unittest.main()
