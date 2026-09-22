#!/usr/bin/env python3
"""Export one ecological land run into physically separated evidence planes.

The fixture summary is robot-observed runtime evidence. The Unity truth artifact and scenario
manifest are evaluator-only. This exporter binds both planes by hashes and an opaque episode ID;
it never copies physical scenario identity into the robot-visible record.
"""

from __future__ import annotations

import argparse
import hashlib
import json
import math
import re
from pathlib import Path
from typing import Any, Iterable

from summarize_environment_qa import scenario_contract


ROOT = Path(__file__).resolve().parents[2]
DEFAULT_CONTRACT = Path(__file__).with_name("ecological_explanation_contract_v1.json")
EPISODE_ID = re.compile(r"^[A-Za-z0-9._-]+$")


def load(path: Path) -> dict[str, Any]:
    value = json.loads(path.read_text(encoding="utf-8"))
    if not isinstance(value, dict):
        raise ValueError(f"Expected a JSON object: {path}")
    return value


def sha256(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


def render(path: Path, value: dict[str, Any]) -> None:
    path.write_text(json.dumps(value, indent=2, sort_keys=True) + "\n", encoding="utf-8")


def trajectory_summary(samples: list[dict[str, Any]]) -> dict[str, Any]:
    points = [(float(value["x"]), float(value["y"])) for value in samples]
    if not points:
        return {
            "sampleCount": 0,
            "sampledPathLengthMeters": 0.0,
            "endpointDisplacementMeters": 0.0,
            "maximumLateralExcursionMeters": 0.0,
        }
    first = points[0]
    last = points[-1]
    return {
        "sampleCount": len(points),
        "sampledPathLengthMeters": sum(
            math.hypot(second[0] - first_value[0], second[1] - first_value[1])
            for first_value, second in zip(points, points[1:])
        ),
        "endpointDisplacementMeters": math.hypot(last[0] - first[0], last[1] - first[1]),
        "maximumLateralExcursionMeters": max(abs(value[1] - first[1]) for value in points),
    }


def strings(value: Any) -> Iterable[str]:
    if isinstance(value, str):
        yield value
    elif isinstance(value, dict):
        for key, child in value.items():
            yield str(key)
            yield from strings(child)
    elif isinstance(value, list):
        for child in value:
            yield from strings(child)


def hidden_terms(resolved: dict[str, Any], truth: dict[str, Any], source_paths: Iterable[Path]) -> set[str]:
    terms = {
        str(resolved.get("id", "")),
        str(truth.get("environmentId", "")),
        str(truth.get("layoutId", "")),
        str(truth.get("warehouseScenarioId", "")),
        str(truth.get("warehouseRouteId", "")),
    }
    for key in ("warehouseObstacleSemanticIds", "relevantObstacles"):
        terms.update(str(value) for value in truth.get(key, []) if value)
    for path in source_paths:
        terms.update((str(path), str(path.resolve()), path.name))
    return {value for value in terms if value}


def assert_robot_visible_safe(value: dict[str, Any], forbidden: set[str]) -> None:
    findings = []
    normalized_forbidden = {term.lower() for term in forbidden}
    for candidate in strings(value):
        lowered = candidate.lower()
        if any(term in lowered for term in normalized_forbidden):
            findings.append(candidate)
    forbidden_keys = {
        "scenarioid",
        "layoutid",
        "routeid",
        "obstacleid",
        "obstacleids",
        "worldtruth",
        "evaluatortruth",
        "sourcepath",
        "sourcepaths",
        "expectedoutcome",
    }
    for candidate in strings(value):
        if candidate.lower().replace("_", "").replace("-", "") in forbidden_keys:
            findings.append(candidate)
    if findings:
        raise ValueError(
            "Evaluator-only identity leaked into robot-visible export: "
            + ", ".join(sorted(set(findings)))
        )


def select_contract(
    contract: dict[str, Any], environment_id: str, scenario_id: str,
    manifest_hash: str, configuration_hash: str,
) -> dict[str, Any]:
    matches = [
        value for value in contract.get("scenarios", [])
        if value.get("environmentId") == environment_id
        and value.get("scenarioId") == scenario_id
    ]
    if len(matches) != 1:
        raise ValueError(
            f"Expected one ecological contract for {environment_id}/{scenario_id}, "
            f"found {len(matches)}"
        )
    selected = matches[0]
    if selected.get("manifestSha256") != manifest_hash:
        raise ValueError("Ecological contract manifest hash does not match export input")
    if selected.get("configurationSha256") != configuration_hash:
        raise ValueError("Ecological contract configuration hash does not match runtime truth")
    return selected


def validate_capture(capture: dict[str, Any]) -> None:
    transitions = capture["orderedTransitions"]
    invocations = capture["recoveryInvocations"]
    completeness = capture["completeness"]
    transition_ids = [value.get("recordId") for value in transitions]
    invocation_ids = [value.get("invocationId") for value in invocations]
    if any(not isinstance(value, str) or not value for value in transition_ids):
        raise ValueError("Every retained BT transition needs a stable record ID")
    if len(transition_ids) != len(set(transition_ids)):
        raise ValueError("BT transition record IDs are not unique")
    if any(not isinstance(value, str) or not value for value in invocation_ids):
        raise ValueError("Every retained recovery invocation needs a stable invocation ID")
    if len(invocation_ids) != len(set(invocation_ids)):
        raise ValueError("Recovery invocation IDs are not unique")

    dropped_transitions = int(capture.get("droppedTransitionCount", 0))
    unique_transitions = int(capture.get("uniqueTransitionCount", -1))
    if unique_transitions != len(transitions) + dropped_transitions:
        raise ValueError("BT retained/dropped transition counts are inconsistent")
    dropped_invocations = int(capture.get("droppedRecoveryInvocationCount", 0))
    observed_invocations = int(capture.get("observedRecoveryInvocationStartCount", -1))
    if observed_invocations != len(invocations) + dropped_invocations:
        raise ValueError("BT retained/dropped recovery invocation counts are inconsistent")

    if dropped_transitions == 0:
        retained = set(transition_ids)
        for invocation in invocations:
            if invocation.get("startTransitionId") not in retained:
                raise ValueError("Recovery invocation start is absent from complete retained stream")
            end = invocation.get("endTransitionId")
            if end is not None and end not in retained:
                raise ValueError("Recovery invocation end is absent from complete retained stream")

    if completeness.get("exactRecoveryCountEligible") is True:
        exact_requirements = (
            completeness.get("historyStatus") == "complete",
            completeness.get("messageLossDetectable") is True,
            not dropped_transitions,
            not dropped_invocations,
            not completeness.get("openRecoveryInvocationIds"),
            not completeness.get("incompleteRecoveryInvocationIds"),
        )
        if not all(exact_requirements):
            raise ValueError("Exact recovery-count eligibility contradicts capture completeness")


def export(
    *, fixture_path: Path, truth_path: Path, environment_manifest_path: Path,
    bt_xml_path: Path, contract_path: Path, output_root: Path, episode_id: str,
) -> dict[str, Any]:
    if not EPISODE_ID.fullmatch(episode_id):
        raise ValueError("Episode ID may contain only letters, numbers, dot, underscore, and dash")
    if output_root.exists():
        raise FileExistsError(f"Refusing existing export root: {output_root}")
    for path in (fixture_path, truth_path, environment_manifest_path, bt_xml_path, contract_path):
        if not path.is_file():
            raise FileNotFoundError(path)

    fixture = load(fixture_path)
    truth = load(truth_path)
    environment_manifest = load(environment_manifest_path)
    ecological_contract = load(contract_path)
    capture = fixture.get("behaviorTreeCapture")
    if not isinstance(capture, dict):
        raise ValueError("Fixture lacks ordered BehaviorTree capture")
    transitions = capture.get("orderedTransitions")
    invocations = capture.get("recoveryInvocations")
    completeness = capture.get("completeness")
    if not isinstance(transitions, list) or not isinstance(invocations, list):
        raise ValueError("Fixture lacks BT transition/invocation arrays")
    if not isinstance(completeness, dict):
        raise ValueError("Fixture lacks BT completeness metadata")
    validate_capture(capture)
    samples = fixture.get("trajectorySamples")
    if not isinstance(samples, list):
        raise ValueError("Fixture lacks trajectory samples")

    manifest_hash = sha256(environment_manifest_path)
    resolved = scenario_contract(environment_manifest, truth, manifest_hash)
    if not resolved.get("contractMatches"):
        raise ValueError("Evaluator truth does not match its environment scenario contract")
    environment_id = truth.get("environmentId") or environment_manifest.get("environmentId")
    scenario_id = resolved.get("id")
    configuration_hash = resolved.get("configurationSha256")
    selected_contract = select_contract(
        ecological_contract, environment_id, scenario_id,
        manifest_hash, configuration_hash,
    )

    goal_ids = sorted({
        value.get("goalId") for value in transitions if value.get("goalId") is not None
    })
    if len(goal_ids) != 1:
        raise ValueError(f"Expected one accepted goal ID in ordered BT records, found {goal_ids}")

    robot_visible = {
        "schema": "crane-ecological-robot-visible-evidence-v1",
        "episodeId": episode_id,
        "action": {
            "mode": fixture.get("actionMode"),
            "name": fixture.get("actionName"),
            "goalId": goal_ids[0],
            "terminalStatus": fixture.get("status"),
        },
        "bt": {
            "transitionSequence": transitions,
            "recoveryInvocations": invocations,
            "observedRecoveryInvocationStartCount": capture.get(
                "observedRecoveryInvocationStartCount"
            ),
            "uniqueTransitionCount": capture.get("uniqueTransitionCount"),
            "duplicateTransitionCount": capture.get("duplicateTransitionCount"),
            "droppedTransitionCount": capture.get("droppedTransitionCount"),
            "droppedRecoveryInvocationCount": capture.get(
                "droppedRecoveryInvocationCount"
            ),
            "completeness": completeness,
        },
        "feedback": {
            "maximumObservedRecoveryCount": fixture.get("maximumRecoveryCount"),
            "observedRecoveryCountSequence": fixture.get("recoveryCountSequence", []),
            "provenance": "nav2_feedback_count_not_bt_invocation_identity",
        },
        "trajectory": {
            "samples": samples,
            "summary": trajectory_summary(samples),
            "provenance": fixture.get("trajectoryProvenance"),
        },
        "runtime": {
            "btPolicySha256": sha256(bt_xml_path),
            "environmentManifestSha256": manifest_hash,
            "configurationSha256": configuration_hash,
        },
        "withholding": {
            "evaluatorTruthAvailableToMethods": False,
            "physicalCauseEstablished": False,
            "controllerConsumptionEstablished": False,
            "exactRecoveryCountEligible": completeness.get("exactRecoveryCountEligible") is True,
        },
    }
    forbidden = hidden_terms(
        resolved, truth,
        (fixture_path, truth_path, environment_manifest_path, bt_xml_path, contract_path),
    )
    assert_robot_visible_safe(robot_visible, forbidden)

    evaluator_only = {
        "schema": "crane-ecological-evaluator-only-truth-v1",
        "episodeId": episode_id,
        "environmentId": environment_id,
        "scenarioId": scenario_id,
        "scenarioKind": resolved.get("kind"),
        "configurationSha256": configuration_hash,
        "worldTruth": truth,
        "scenarioContract": resolved.get("contract"),
        "questionContracts": selected_contract.get("questionContracts", []),
        "inputArtifactSha256": {
            "fixtureSummary": sha256(fixture_path),
            "worldTruth": sha256(truth_path),
            "environmentManifest": manifest_hash,
            "btPolicy": sha256(bt_xml_path),
            "ecologicalContract": sha256(contract_path),
        },
    }

    robot_dir = output_root / "robot_visible"
    evaluator_dir = output_root / "evaluator_only"
    robot_dir.mkdir(parents=True)
    evaluator_dir.mkdir(parents=True)
    robot_path = robot_dir / "evidence.json"
    evaluator_path = evaluator_dir / "truth.json"
    render(robot_path, robot_visible)
    render(evaluator_path, evaluator_only)
    export_manifest = {
        "schema": "crane-ecological-evidence-export-manifest-v1",
        "episodeId": episode_id,
        "files": [
            {
                "path": "robot_visible/evidence.json",
                "sha256": sha256(robot_path),
                "bytes": robot_path.stat().st_size,
                "evidencePlane": "robot_visible",
            },
            {
                "path": "evaluator_only/truth.json",
                "sha256": sha256(evaluator_path),
                "bytes": evaluator_path.stat().st_size,
                "evidencePlane": "evaluator_only",
            },
        ],
    }
    render(output_root / "export-manifest.json", export_manifest)
    return export_manifest


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--fixture-summary", type=Path, required=True)
    parser.add_argument("--evaluator-truth", type=Path, required=True)
    parser.add_argument("--environment-manifest", type=Path, required=True)
    parser.add_argument("--bt-xml", type=Path, required=True)
    parser.add_argument("--contract", type=Path, default=DEFAULT_CONTRACT)
    parser.add_argument("--output-root", type=Path, required=True)
    parser.add_argument("--episode-id", required=True)
    args = parser.parse_args()
    result = export(
        fixture_path=args.fixture_summary,
        truth_path=args.evaluator_truth,
        environment_manifest_path=args.environment_manifest,
        bt_xml_path=args.bt_xml,
        contract_path=args.contract,
        output_root=args.output_root,
        episode_id=args.episode_id,
    )
    print(json.dumps(result, indent=2, sort_keys=True))


if __name__ == "__main__":
    main()
