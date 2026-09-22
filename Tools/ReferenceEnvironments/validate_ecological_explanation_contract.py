#!/usr/bin/env python3
"""Validate ecological question contracts against exact environment manifests."""

from __future__ import annotations

import argparse
import hashlib
import json
from pathlib import Path
from typing import Any


ROOT = Path(__file__).resolve().parents[2]
EXPECTED_SCHEMA = "crane-ecological-explanation-contract-v1"
ALLOWED_ANSWER_MODES = {"full", "partial", "full_when_complete_otherwise_partial"}
ALLOWED_QUALIFICATION = {"REPETITION_PASS", "SINGLE_RUN_NAVIGATION_PASS"}
ALLOWED_TERMINAL_STATUSES = {"succeeded", "aborted", "canceled", "timeout"}


def load(path: Path) -> dict[str, Any]:
    return json.loads(path.read_text(encoding="utf-8"))


def sha256(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


def configuration_hash(manifest_hash: str, entry_type: str,
                       entry: dict[str, Any], runtime_seed: int) -> str:
    scenario_id = entry["id"]
    if entry_type == "scenario":
        value = f"{manifest_hash}\n{scenario_id}\n{entry['routeId']}\n{runtime_seed}"
    else:
        value = f"{manifest_hash}\n{scenario_id}\n{runtime_seed}"
    return hashlib.sha256(value.encode("utf-8")).hexdigest()


def require_hash(value: Any, label: str) -> None:
    if not isinstance(value, str) or len(value) != 64:
        raise ValueError(f"{label} must be a SHA-256 hex digest")
    int(value, 16)


def resolve_entry(manifest: dict[str, Any], entry_type: str,
                  scenario_id: str) -> dict[str, Any]:
    key = "scenarios" if entry_type == "scenario" else "layouts"
    matches = [value for value in manifest.get(key, []) if value.get("id") == scenario_id]
    if len(matches) != 1:
        raise ValueError(
            f"Expected one {entry_type} {scenario_id!r}, found {len(matches)}"
        )
    return matches[0]


def validate(contract_path: Path, root: Path = ROOT) -> dict[str, Any]:
    contract = load(contract_path)
    if contract.get("schema") != EXPECTED_SCHEMA:
        raise ValueError("Unsupported ecological explanation contract schema")
    if contract.get("status") != "DEVELOPMENT_ONLY_NOT_FROZEN":
        raise ValueError("Ecological contract must remain outside the frozen study")
    boundary = contract.get("studyBoundary", {})
    if boundary.get("frozenStudyAffected") is not False:
        raise ValueError("Ecological contract may not amend the frozen study")

    planes = contract.get("evidencePlanes", {})
    robot_visible = set(planes.get("robotVisible", []))
    evaluator_only = set(planes.get("evaluatorOnly", []))
    if not robot_visible or not evaluator_only or robot_visible & evaluator_only:
        raise ValueError("Evidence planes must be nonempty and disjoint")

    scenario_keys: set[tuple[str, str]] = set()
    question_ids: set[str] = set()
    manifest_hashes: dict[str, str] = {}
    question_count = 0
    partial_count = 0
    for scenario in contract.get("scenarios", []):
        environment_id = scenario.get("environmentId")
        scenario_id = scenario.get("scenarioId")
        key = (environment_id, scenario_id)
        if not all(isinstance(value, str) and value for value in key):
            raise ValueError("Every scenario needs environmentId and scenarioId")
        if key in scenario_keys:
            raise ValueError(f"Duplicate ecological scenario contract: {key}")
        scenario_keys.add(key)

        entry_type = scenario.get("catalogEntryType")
        if entry_type not in {"layout", "scenario"}:
            raise ValueError(f"Unsupported catalog entry type for {scenario_id}")
        relative_manifest = scenario.get("manifest")
        manifest_path = root / relative_manifest
        if not manifest_path.is_file():
            raise ValueError(f"Missing scenario manifest: {relative_manifest}")
        actual_manifest_hash = sha256(manifest_path)
        if scenario.get("manifestSha256") != actual_manifest_hash:
            raise ValueError(f"Manifest hash mismatch for {scenario_id}")
        manifest = load(manifest_path)
        if manifest.get("environmentId") != environment_id:
            raise ValueError(f"Environment identity mismatch for {scenario_id}")
        entry = resolve_entry(manifest, entry_type, scenario_id)
        hidden_object_ids = {
            value.get("id", "").lower()
            for value in entry.get("obstacles", [])
            if value.get("id")
        }
        runtime_seed = scenario.get("runtimeSeed")
        if not isinstance(runtime_seed, int) or runtime_seed < 0:
            raise ValueError(f"Invalid runtime seed for {scenario_id}")
        expected_configuration = configuration_hash(
            actual_manifest_hash, entry_type, entry, runtime_seed
        )
        if scenario.get("configurationSha256") != expected_configuration:
            raise ValueError(f"Configuration hash mismatch for {scenario_id}")
        manifest_hashes[relative_manifest] = actual_manifest_hash

        qualification = scenario.get("qualification", {})
        if qualification.get("status") not in ALLOWED_QUALIFICATION:
            raise ValueError(f"Unsupported qualification for {scenario_id}")
        require_hash(qualification.get("artifactSha256"),
                     f"qualification artifact for {scenario_id}")
        run_count = qualification.get("runCount")
        if not isinstance(run_count, int) or run_count < 1:
            raise ValueError(f"Invalid qualification run count for {scenario_id}")
        if qualification["status"] == "REPETITION_PASS" and run_count < 3:
            raise ValueError(f"Repetition pass needs at least three runs for {scenario_id}")

        acceptance = scenario.get("runtimeAcceptance")
        if not isinstance(acceptance, dict):
            raise ValueError(f"Missing runtime acceptance criteria for {scenario_id}")
        terminal_statuses = acceptance.get("terminalStatuses")
        if (
            not isinstance(terminal_statuses, list)
            or not terminal_statuses
            or len(terminal_statuses) != len(set(terminal_statuses))
            or not set(terminal_statuses) <= ALLOWED_TERMINAL_STATUSES
        ):
            raise ValueError(f"Invalid terminal statuses for {scenario_id}")
        for field in (
            "minimumRecordedRecoveryInvocations",
            "minimumTrajectorySamples",
        ):
            value = acceptance.get(field, 0)
            if not isinstance(value, int) or isinstance(value, bool) or value < 0:
                raise ValueError(f"Invalid {field} for {scenario_id}")
        required_nodes = acceptance.get("requiredTransitionNodeNames", [])
        if (
            not isinstance(required_nodes, list)
            or any(not isinstance(value, str) or not value for value in required_nodes)
            or len(required_nodes) != len(set(required_nodes))
        ):
            raise ValueError(f"Invalid required transition nodes for {scenario_id}")
        for field in (
            "requireZeroDroppedTransitions",
            "requireZeroDroppedRecoveryInvocations",
        ):
            if acceptance.get(field) is not True:
                raise ValueError(f"{field} must be true for {scenario_id}")

        questions = scenario.get("questionContracts", [])
        if not questions:
            raise ValueError(f"No question contracts for {scenario_id}")
        for question in questions:
            question_id = question.get("id")
            if not isinstance(question_id, str) or not question_id:
                raise ValueError(f"Question without ID in {scenario_id}")
            if question_id in question_ids:
                raise ValueError(f"Duplicate question ID: {question_id}")
            question_ids.add(question_id)
            if question.get("answerMode") not in ALLOWED_ANSWER_MODES:
                raise ValueError(f"Unsupported answer mode for {question_id}")
            question_text = question.get("question")
            if not isinstance(question_text, str) or not question_text.strip():
                raise ValueError(f"Missing question text for {question_id}")
            leaked_ids = [
                value for value in hidden_object_ids if value in question_text.lower()
            ]
            if leaked_ids:
                raise ValueError(
                    f"Evaluator-only object ID leaked into {question_id}: {leaked_ids}"
                )
            required = set(question.get("requiredRobotVisible", []))
            if not required or not required <= robot_visible:
                raise ValueError(f"Unlicensed robot-visible evidence in {question_id}")
            if required & evaluator_only:
                raise ValueError(f"Evaluator-only evidence leaked into {question_id}")
            if not question.get("supportedClaimClasses"):
                raise ValueError(f"No supported claim classes for {question_id}")
            must_withhold = question.get("mustWithhold", [])
            if not must_withhold:
                raise ValueError(f"No withholding boundary for {question_id}")
            if question["answerMode"] == "partial":
                partial_count += 1
            question_count += 1

    if not scenario_keys:
        raise ValueError("Ecological explanation contract contains no scenarios")
    return {
        "schema": "crane-ecological-explanation-contract-validation-v1",
        "valid": True,
        "contract": str(contract_path),
        "contractSha256": sha256(contract_path),
        "scenarioCount": len(scenario_keys),
        "questionCount": question_count,
        "partialAnswerQuestionCount": partial_count,
        "manifestSha256": manifest_hashes,
        "frozenStudyAffected": False,
    }


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "contract", type=Path, nargs="?",
        default=Path(__file__).with_name("ecological_explanation_contract_v1.json"),
    )
    parser.add_argument("--root", type=Path, default=ROOT)
    parser.add_argument("--output", type=Path)
    args = parser.parse_args()
    result = validate(args.contract, args.root)
    rendered = json.dumps(result, indent=2, sort_keys=True) + "\n"
    if args.output:
        args.output.parent.mkdir(parents=True, exist_ok=True)
        args.output.write_text(rendered, encoding="utf-8")
    print(rendered, end="")


if __name__ == "__main__":
    main()
