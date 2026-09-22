#!/usr/bin/env python3

import copy
import importlib.util
import json
import tempfile
import unittest
from pathlib import Path


SCRIPT = Path(__file__).with_name("validate_ecological_explanation_contract.py")
CONTRACT = Path(__file__).with_name("ecological_explanation_contract_v1.json")
SPEC = importlib.util.spec_from_file_location("ecological_contract", SCRIPT)
MODULE = importlib.util.module_from_spec(SPEC)
assert SPEC.loader is not None
SPEC.loader.exec_module(MODULE)


class EcologicalExplanationContractTests(unittest.TestCase):
    def test_repository_contract_resolves_exact_manifests_and_questions(self) -> None:
        result = MODULE.validate(CONTRACT)
        self.assertTrue(result["valid"])
        self.assertEqual(result["scenarioCount"], 5)
        self.assertEqual(result["questionCount"], 10)
        self.assertEqual(result["partialAnswerQuestionCount"], 5)
        self.assertFalse(result["frozenStudyAffected"])

    def test_rejects_evaluator_only_evidence_as_robot_visible(self) -> None:
        contract = json.loads(CONTRACT.read_text(encoding="utf-8"))
        changed = copy.deepcopy(contract)
        changed["scenarios"][0]["questionContracts"][0][
            "requiredRobotVisible"
        ].append("world.obstacle_schedule")
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "contract.json"
            path.write_text(json.dumps(changed), encoding="utf-8")
            with self.assertRaisesRegex(ValueError, "Unlicensed robot-visible evidence"):
                MODULE.validate(path)

    def test_rejects_manifest_or_configuration_drift(self) -> None:
        contract = json.loads(CONTRACT.read_text(encoding="utf-8"))
        for field, message in (
            ("manifestSha256", "Manifest hash mismatch"),
            ("configurationSha256", "Configuration hash mismatch"),
        ):
            with self.subTest(field=field):
                changed = copy.deepcopy(contract)
                changed["scenarios"][0][field] = "0" * 64
                with tempfile.TemporaryDirectory() as directory:
                    path = Path(directory) / "contract.json"
                    path.write_text(json.dumps(changed), encoding="utf-8")
                    with self.assertRaisesRegex(ValueError, message):
                        MODULE.validate(path)

    def test_rejects_evaluator_object_identity_in_question_text(self) -> None:
        contract = json.loads(CONTRACT.read_text(encoding="utf-8"))
        changed = copy.deepcopy(contract)
        changed["scenarios"][0]["questionContracts"][0]["question"] += (
            " Was recovery-enclosure-north responsible?"
        )
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "contract.json"
            path.write_text(json.dumps(changed), encoding="utf-8")
            with self.assertRaisesRegex(ValueError, "object ID leaked"):
                MODULE.validate(path)

    def test_repetition_pass_requires_three_runs(self) -> None:
        contract = json.loads(CONTRACT.read_text(encoding="utf-8"))
        changed = copy.deepcopy(contract)
        changed["scenarios"][0]["qualification"]["runCount"] = 2
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "contract.json"
            path.write_text(json.dumps(changed), encoding="utf-8")
            with self.assertRaisesRegex(ValueError, "at least three runs"):
                MODULE.validate(path)

    def test_rejects_missing_runtime_acceptance_criteria(self) -> None:
        contract = json.loads(CONTRACT.read_text(encoding="utf-8"))
        changed = copy.deepcopy(contract)
        del changed["scenarios"][0]["runtimeAcceptance"]
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "contract.json"
            path.write_text(json.dumps(changed), encoding="utf-8")
            with self.assertRaisesRegex(ValueError, "Missing runtime acceptance"):
                MODULE.validate(path)

    def test_rejects_unknown_runtime_trajectory_criterion(self) -> None:
        contract = json.loads(CONTRACT.read_text(encoding="utf-8"))
        changed = copy.deepcopy(contract)
        changed["scenarios"][0]["runtimeAcceptance"]["trajectoryCriteria"] = {
            "looksCurvy": True
        }
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "contract.json"
            path.write_text(json.dumps(changed), encoding="utf-8")
            with self.assertRaisesRegex(ValueError, "Invalid trajectory criteria"):
                MODULE.validate(path)


if __name__ == "__main__":
    unittest.main()
