#!/usr/bin/env python3
"""Regression tests for physically separated ecological evidence export."""

from __future__ import annotations

import hashlib
import json
from pathlib import Path
import sys
import tempfile
import unittest


sys.path.insert(0, str(Path(__file__).resolve().parent))

from export_ecological_evidence import export  # noqa: E402


def write(path: Path, value) -> None:
    path.write_text(json.dumps(value, sort_keys=True) + "\n", encoding="utf-8")


class EcologicalEvidenceExportTests(unittest.TestCase):
    def setUp(self) -> None:
        self.temp = tempfile.TemporaryDirectory()
        self.root = Path(self.temp.name)
        self.fixture = self.root / "fixture-summary.json"
        self.truth = self.root / "land-evaluator-truth.json"
        self.environment_manifest = self.root / "warehouse.json"
        self.bt_xml = self.root / "policy.xml"
        self.contract = self.root / "contract.json"
        self.bt_xml.write_text("<root/>\n", encoding="utf-8")
        write(
            self.environment_manifest,
            {
                "environmentId": "warehouse-v2",
                "routes": [{"id": "route-detour"}],
                "scenarios": [{
                    "id": "hidden-enclosure",
                    "routeId": "route-detour",
                    "seed": 4105,
                    "robot": "TurtleBot3",
                    "expectedChallenge": "temporary enclosure",
                    "expectedBroadOutcome": "recovery-success",
                    "obstacles": [{"id": "hidden-wall-north"}],
                }],
            },
        )
        manifest_hash = hashlib.sha256(self.environment_manifest.read_bytes()).hexdigest()
        configuration_hash = hashlib.sha256(
            f"{manifest_hash}\nhidden-enclosure\nroute-detour\n4105".encode()
        ).hexdigest()
        write(
            self.truth,
            {
                "environmentId": "warehouse-v2",
                "scenarioId": "hidden-enclosure",
                "referenceEnvironmentPreserved": True,
                "warehouseScenarioId": "hidden-enclosure",
                "warehouseRouteId": "route-detour",
                "warehouseScenarioSeed": 4105,
                "warehouseScenarioRobot": "TurtleBot3",
                "warehouseExpectedChallenge": "temporary enclosure",
                "warehouseExpectedBroadOutcome": "recovery-success",
                "warehouseObstacleSemanticIds": ["hidden-wall-north"],
            },
        )
        write(
            self.fixture,
            {
                "actionMode": "navigate-to-pose",
                "actionName": "/navigate_to_pose",
                "status": "succeeded",
                "maximumRecoveryCount": 1,
                "recoveryCountSequence": [0, 1],
                "costmapMessages": 3,
                "costmapObservations": 5,
                "costmapServiceSnapshots": 2,
                "maximumOccupiedCostmapCells": 123,
                "costmapProvenance": "delivered_costmap_not_proven_controller_consumption",
                "trajectoryProvenance": "sampled-delivered-odometry",
                "trajectorySamples": [
                    {"x": 0.0, "y": 0.0, "yaw": 0.0},
                    {"x": 1.0, "y": 0.5, "yaw": 0.2},
                ],
                "behaviorTreeCapture": {
                    "orderedTransitions": [{
                        "recordId": "bt-transition-000001",
                        "nodeName": "Wait",
                        "uid": 7,
                        "previousStatus": "IDLE",
                        "currentStatus": "RUNNING",
                        "eventStamp": {"sec": 1, "nanosec": 0},
                        "messageStamp": {"sec": 1, "nanosec": 1},
                        "goalId": "goal-1",
                        "goalRelation": "after_accepted_goal",
                    }],
                    "recoveryInvocations": [{
                        "invocationId": "bt-recovery-invocation-000001",
                        "nodeName": "Wait",
                        "uid": 7,
                        "goalId": "goal-1",
                        "startTransitionId": "bt-transition-000001",
                        "endTransitionId": None,
                        "terminalStatus": None,
                        "complete": False,
                    }],
                    "observedRecoveryInvocationStartCount": 1,
                    "uniqueTransitionCount": 1,
                    "duplicateTransitionCount": 0,
                    "droppedTransitionCount": 0,
                    "droppedRecoveryInvocationCount": 0,
                    "recoveryNodeClassifier": {
                        "basis": "configured_exact_leaf_name_and_idle_departure",
                        "nodeNames": ["Wait"],
                        "directTerminalNodeNames": [],
                        "directTerminalClassifierBasis": "loaded_bt_xml",
                        "directTerminalClassifierSha256": None,
                        "invocationStartPatterns": {
                            "allConfiguredLeaves": ["IDLE->RUNNING"],
                            "directTerminalLeaves": ["IDLE->SUCCESS"],
                        },
                    },
                    "completeness": {
                        "historyStatus": "not_proven",
                        "exactRecoveryCountEligible": False,
                    },
                },
            },
        )
        write(
            self.contract,
            {
                "scenarios": [{
                    "environmentId": "warehouse-v2",
                    "scenarioId": "hidden-enclosure",
                    "manifestSha256": manifest_hash,
                    "configurationSha256": configuration_hash,
                    "runtimeAcceptance": {
                        "terminalStatuses": ["succeeded"],
                        "minimumRecordedRecoveryInvocations": 1,
                        "minimumTrajectorySamples": 2,
                        "requiredTransitionNodeNames": ["Wait"],
                        "requireZeroDroppedTransitions": True,
                        "requireZeroDroppedRecoveryInvocations": True,
                    },
                    "questionContracts": [{"id": "question-1"}],
                }],
            },
        )

    def tearDown(self) -> None:
        self.temp.cleanup()

    def do_export(self, name: str = "export"):
        return export(
            fixture_path=self.fixture,
            truth_path=self.truth,
            environment_manifest_path=self.environment_manifest,
            bt_xml_path=self.bt_xml,
            contract_path=self.contract,
            output_root=self.root / name,
            episode_id="eco-opaque-001",
        )

    def test_export_physically_separates_truth_and_runtime_evidence(self) -> None:
        result = self.do_export()
        robot_path = self.root / "export/robot_visible/evidence.json"
        evaluator_path = self.root / "export/evaluator_only/truth.json"
        robot_text = robot_path.read_text(encoding="utf-8")
        evaluator_text = evaluator_path.read_text(encoding="utf-8")

        self.assertTrue(robot_path.is_file())
        self.assertTrue(evaluator_path.is_file())
        self.assertNotIn("hidden-enclosure", robot_text)
        self.assertNotIn("hidden-wall-north", robot_text)
        self.assertNotIn(str(self.root), robot_text)
        self.assertIn("hidden-enclosure", evaluator_text)
        self.assertIn("hidden-wall-north", evaluator_text)
        self.assertEqual(
            [value["evidencePlane"] for value in result["files"]],
            ["robot_visible", "evaluator_only"],
        )

    def test_export_carries_source_qualification_for_each_recovery_invocation(self) -> None:
        self.do_export()
        evidence = json.loads(
            (self.root / "export/robot_visible/evidence.json").read_text(encoding="utf-8")
        )
        policy_hash = hashlib.sha256(self.bt_xml.read_bytes()).hexdigest()
        invocation = evidence["bt"]["recoveryInvocations"][0]

        self.assertEqual(invocation["sourceQualification"], {
            "classifierBasis": "configured_exact_leaf_name_and_idle_departure",
            "classifierRule": "configured_leaf_idle_to_running",
            "observedStartTransition": {
                "currentStatus": "RUNNING",
                "nodeName": "Wait",
                "previousStatus": "IDLE",
                "recordId": "bt-transition-000001",
                "uid": 7,
            },
            "policySha256": policy_hash,
        })
        self.assertEqual(
            evidence["bt"]["recoveryNodeClassifier"]["nodeNames"], ["Wait"]
        )

    def test_export_labels_costmap_summary_as_delivered_not_consumed(self) -> None:
        self.do_export()
        evidence = json.loads(
            (self.root / "export/robot_visible/evidence.json").read_text(encoding="utf-8")
        )
        self.assertEqual(evidence["observations"]["costmap"], {
            "deliveredMessageCount": 3,
            "maximumOccupiedCellCount": 123,
            "observationCount": 5,
            "provenance": "delivered_costmap_not_proven_controller_consumption",
            "serviceSnapshotCount": 2,
        })
        self.assertFalse(evidence["withholding"]["controllerConsumptionEstablished"])

    def test_export_rejects_recovery_invocation_not_licensed_by_classifier(self) -> None:
        value = json.loads(self.fixture.read_text())
        value["behaviorTreeCapture"]["recoveryNodeClassifier"]["nodeNames"] = []
        write(self.fixture, value)
        with self.assertRaisesRegex(ValueError, "not licensed by recovery classifier"):
            self.do_export()

    def test_export_binds_direct_terminal_recovery_to_policy_source_hash(self) -> None:
        value = json.loads(self.fixture.read_text())
        policy_hash = hashlib.sha256(self.bt_xml.read_bytes()).hexdigest()
        transition = value["behaviorTreeCapture"]["orderedTransitions"][0]
        transition["nodeName"] = "ClearLocalCostmap-Context"
        transition["currentStatus"] = "SUCCESS"
        invocation = value["behaviorTreeCapture"]["recoveryInvocations"][0]
        invocation.update({
            "nodeName": "ClearLocalCostmap-Context",
            "endTransitionId": "bt-transition-000001",
            "terminalStatus": "SUCCESS",
            "complete": True,
            "observationPattern": "idle_to_terminal",
        })
        classifier = value["behaviorTreeCapture"]["recoveryNodeClassifier"]
        classifier.update({
            "nodeNames": ["ClearLocalCostmap-Context"],
            "directTerminalNodeNames": ["ClearLocalCostmap-Context"],
            "directTerminalClassifierBasis": (
                "loaded_bt_xml_clear_entire_costmap_without_completion_preconditions"
            ),
            "directTerminalClassifierSha256": policy_hash,
            "directTerminalInterpretation": (
                "source_verified_leaf_completed_in_one_bt_tick; "
                "not_proof_of_external_side_effect_or_physical_cause"
            ),
        })
        value["behaviorTreeCapture"]["recoveryInvocations"] = [invocation]
        write(self.fixture, value)
        contract = json.loads(self.contract.read_text())
        contract["scenarios"][0]["runtimeAcceptance"][
            "requiredTransitionNodeNames"
        ] = ["ClearLocalCostmap-Context"]
        write(self.contract, contract)

        self.do_export()
        evidence = json.loads(
            (self.root / "export/robot_visible/evidence.json").read_text(encoding="utf-8")
        )
        qualification = evidence["bt"]["recoveryInvocations"][0][
            "sourceQualification"
        ]
        self.assertEqual(
            qualification["classifierRule"],
            "source_verified_direct_terminal_idle_to_success",
        )
        self.assertEqual(qualification["directTerminalClassifierSha256"], policy_hash)

    def test_export_is_deterministic_across_output_roots(self) -> None:
        first = self.do_export("first")
        second = self.do_export("second")
        self.assertEqual(first, second)
        self.assertEqual(
            (self.root / "first/robot_visible/evidence.json").read_bytes(),
            (self.root / "second/robot_visible/evidence.json").read_bytes(),
        )

    def test_export_refuses_to_overwrite_any_existing_root(self) -> None:
        (self.root / "export").mkdir()
        with self.assertRaises(FileExistsError):
            self.do_export()

    def test_export_rejects_fixture_without_ordered_capture(self) -> None:
        value = json.loads(self.fixture.read_text())
        del value["behaviorTreeCapture"]
        write(self.fixture, value)
        with self.assertRaisesRegex(ValueError, "ordered BehaviorTree capture"):
            self.do_export()

    def test_export_rejects_contract_configuration_drift(self) -> None:
        value = json.loads(self.contract.read_text())
        value["scenarios"][0]["configurationSha256"] = "0" * 64
        write(self.contract, value)
        with self.assertRaisesRegex(ValueError, "configuration hash"):
            self.do_export()

    def test_export_rejects_inconsistent_transition_inventory(self) -> None:
        value = json.loads(self.fixture.read_text())
        value["behaviorTreeCapture"]["uniqueTransitionCount"] = 2
        write(self.fixture, value)
        with self.assertRaisesRegex(ValueError, "transition counts are inconsistent"):
            self.do_export()

    def test_export_rejects_unjustified_exact_count_eligibility(self) -> None:
        value = json.loads(self.fixture.read_text())
        value["behaviorTreeCapture"]["completeness"]["exactRecoveryCountEligible"] = True
        write(self.fixture, value)
        with self.assertRaisesRegex(ValueError, "eligibility contradicts"):
            self.do_export()

    def test_export_rejects_runtime_terminal_status_mismatch(self) -> None:
        value = json.loads(self.fixture.read_text())
        value["status"] = "aborted"
        write(self.fixture, value)
        with self.assertRaisesRegex(ValueError, "terminal status"):
            self.do_export()

    def test_export_rejects_missing_declared_recovery_mechanism(self) -> None:
        value = json.loads(self.fixture.read_text())
        value["behaviorTreeCapture"]["recoveryInvocations"] = []
        value["behaviorTreeCapture"]["observedRecoveryInvocationStartCount"] = 0
        write(self.fixture, value)
        with self.assertRaisesRegex(ValueError, "minimum recorded recovery"):
            self.do_export()

    def test_export_rejects_trajectory_that_misses_declared_route_shape(self) -> None:
        value = json.loads(self.contract.read_text())
        value["scenarios"][0]["runtimeAcceptance"]["trajectoryCriteria"] = {
            "minimumLateralDirectionChanges": 1
        }
        write(self.contract, value)
        with self.assertRaisesRegex(ValueError, "trajectory does not satisfy"):
            self.do_export()


if __name__ == "__main__":
    unittest.main()
