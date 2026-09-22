import importlib.util
import json
import tempfile
import unittest
from pathlib import Path


SCRIPT = Path(__file__).with_name("summarize_environment_repetitions.py")
CONTRACT = Path(__file__).with_name("land_proving_ground_repetition_contract_v1.json")
WAREHOUSE_CONTRACT = Path(__file__).with_name("warehouse_repetition_contract_v1.json")
SPEC = importlib.util.spec_from_file_location("repetition_summary", SCRIPT)
MODULE = importlib.util.module_from_spec(SPEC)
assert SPEC.loader is not None
SPEC.loader.exec_module(MODULE)


class EnvironmentRepetitionSummaryTests(unittest.TestCase):
    def test_committed_contract_covers_priority_mechanisms(self) -> None:
        contract = json.loads(CONTRACT.read_text(encoding="utf-8"))
        self.assertEqual(contract["schema"], "crane-environment-repetition-contract-v1")
        scenarios = {
            (value["environmentId"], value["scenarioId"]): value
            for value in contract["scenarios"]
        }
        self.assertEqual(len(scenarios), 3)
        self.assertEqual(
            set(scenarios),
            {
                ("crane-land-proving-ground-v2", "slalom-s-turn-v2"),
                ("crane-land-proving-ground-v1", "dynamic-gate-v1"),
                ("crane-land-proving-ground-v1", "complete-blockage-v1"),
            },
        )
        self.assertTrue(all(value["minimumRuns"] == 3 for value in scenarios.values()))
        self.assertEqual(
            scenarios[("crane-land-proving-ground-v1", "dynamic-gate-v1")]
            ["minimumRecoveryCount"],
            1,
        )

    def test_warehouse_contract_requires_recovery_success_repetition(self) -> None:
        contract = json.loads(WAREHOUSE_CONTRACT.read_text(encoding="utf-8"))
        self.assertEqual(contract["schema"], "crane-environment-repetition-contract-v1")
        self.assertEqual(len(contract["scenarios"]), 1)
        scenario = contract["scenarios"][0]
        self.assertEqual(scenario["environmentId"], "crane-industrial-warehouse-v2")
        self.assertEqual(
            scenario["scenarioId"], "warehouse-temporary-enclosure-recovery-v1"
        )
        self.assertEqual(scenario["minimumRuns"], 3)
        self.assertEqual(scenario["expectedNavigationStatus"], "succeeded")
        self.assertEqual(scenario["minimumRecoveryCount"], 1)

    def setUp(self) -> None:
        self.temporary = tempfile.TemporaryDirectory()
        self.root = Path(self.temporary.name)
        self.contract = self.root / "contract.json"
        self.contract.write_text(json.dumps({
            "schema": "crane-environment-repetition-contract-v1",
            "scenarios": [{
                "environmentId": "world-v1",
                "scenarioId": "route-v1",
                "minimumRuns": 3,
                "expectedNavigationStatus": "succeeded",
                "minimumRecoveryCount": 1,
                "requiredGates": {
                    "navigation": "NAVIGATION_PASS",
                    "headless": "HEADLESS_PASS",
                },
            }],
        }), encoding="utf-8")

    def tearDown(self) -> None:
        self.temporary.cleanup()

    def write_summary(self, index: int, *, recovery: int = 1,
                      status: str = "succeeded") -> Path:
        path = self.root / f"summary-{index}.json"
        path.write_text(json.dumps({
            "environmentId": "world-v1",
            "scenarioId": "route-v1",
            "identityValid": True,
            "configurationSha256": "configuration",
            "gates": {
                "navigation": "NAVIGATION_PASS",
                "headless": "HEADLESS_PASS",
            },
            "navigation": {
                "status": status,
                "wallSeconds": 10 + index,
                "displacementMeters": 5 + index,
                "maximumRecoveryCount": recovery,
            },
            "trajectory": {
                "sampledPathLengthMeters": 8 + index,
                "lateralDirectionChangeCount": index,
            },
            "routeAcceptance": {"passed": True},
            "artifactSha256": {
                "runtimeResult": f"runtime-{index}",
                "navigation": f"navigation-{index}",
            },
        }), encoding="utf-8")
        return path

    def test_three_distinct_matching_runs_pass(self) -> None:
        paths = [self.write_summary(index) for index in range(3)]
        result = MODULE.summarize(
            self.contract, "world-v1", "route-v1", paths
        )
        self.assertTrue(result["passed"])
        self.assertEqual(result["status"], "REPETITION_PASS")
        self.assertEqual(result["metrics"]["wallSeconds"]["median"], 11.0)

    def test_insufficient_runs_remain_partial(self) -> None:
        paths = [self.write_summary(index) for index in range(2)]
        result = MODULE.summarize(
            self.contract, "world-v1", "route-v1", paths
        )
        self.assertFalse(result["passed"])
        self.assertFalse(result["checks"]["minimumRunCount"])

    def test_duplicate_runtime_artifact_is_rejected(self) -> None:
        paths = [self.write_summary(index) for index in range(3)]
        duplicate = json.loads(paths[2].read_text(encoding="utf-8"))
        duplicate["artifactSha256"]["runtimeResult"] = "runtime-1"
        paths[2].write_text(json.dumps(duplicate), encoding="utf-8")
        result = MODULE.summarize(
            self.contract, "world-v1", "route-v1", paths
        )
        self.assertFalse(result["passed"])
        self.assertFalse(result["checks"]["runtimeArtifactsDistinct"])

    def test_wrong_outcome_or_missing_recovery_is_rejected(self) -> None:
        paths = [self.write_summary(index) for index in range(3)]
        first = json.loads(paths[0].read_text(encoding="utf-8"))
        first["navigation"]["status"] = "aborted"
        first["navigation"]["maximumRecoveryCount"] = 0
        paths[0].write_text(json.dumps(first), encoding="utf-8")
        result = MODULE.summarize(
            self.contract, "world-v1", "route-v1", paths
        )
        self.assertFalse(result["checks"]["navigationStatusMatches"])
        self.assertFalse(result["checks"]["minimumRecoveryCount"])


if __name__ == "__main__":
    unittest.main()
