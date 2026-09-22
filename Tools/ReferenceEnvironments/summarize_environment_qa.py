#!/usr/bin/env python3
"""Merge independently produced environment and navigation QA into one gate record."""

from __future__ import annotations

import argparse
import hashlib
import json
import math
from pathlib import Path
from typing import Any


def load(path: Path) -> dict[str, Any]:
    return json.loads(path.read_text(encoding="utf-8"))


def sha256(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


def trajectory_metrics(runtime: dict[str, Any]) -> dict[str, Any]:
    points: list[tuple[float, float, float]] = []
    for sample in runtime.get("validation", []):
        bodies = sample.get("bodies", [])
        if not bodies:
            continue
        position = bodies[0].get("position", {})
        points.append(
            (
                float(sample.get("simulatedSeconds", 0.0)),
                float(position.get("x", 0.0)),
                float(position.get("z", 0.0)),
            )
        )
    if not points:
        return {"sampleCount": 0, "sampledPathLengthMeters": 0.0,
                "maximumLateralExcursionMeters": 0.0,
                "longitudinalReversalSampleCount": 0}
    start_x = points[0][1]
    return {
        "sampleCount": len(points),
        "sampledPathLengthMeters": sum(
            math.hypot(second[1] - first[1], second[2] - first[2])
            for first, second in zip(points, points[1:])
        ),
        "maximumLateralExcursionMeters": max(abs(point[1] - start_x) for point in points),
        "minimumUnityX": min(point[1] for point in points),
        "maximumUnityX": max(point[1] for point in points),
        "minimumUnityZ": min(point[2] for point in points),
        "maximumUnityZ": max(point[2] for point in points),
        "longitudinalReversalSampleCount": sum(
            second[2] < first[2] - 0.01 for first, second in zip(points, points[1:])
        ),
    }


def scenario_contract(manifest: dict[str, Any], truth: dict[str, Any]) -> dict[str, Any]:
    """Resolve a warehouse route or proving-ground layout behind one internal seam."""
    schema = truth.get("schema")
    if schema == "crane-land-proving-ground-truth-v1":
        scenario_id = truth.get("layoutId")
        contract = next(
            (value for value in manifest.get("layouts", []) if value.get("id") == scenario_id),
            None,
        )
        seed = truth.get("seed")
        configuration_hash = hashlib.sha256(
            f"{truth.get('manifestSha256')}\n{scenario_id}\n{seed}".encode("utf-8")
        ).hexdigest()
        contract_matches = bool(contract) and all((
            truth.get("generatorVersion") == manifest.get("generatorVersion"),
            truth.get("robot") == contract.get("robot"),
            truth.get("alternatives") == contract.get("alternatives"),
            truth.get("relevantObstacles") == contract.get("relevantObstacles"),
            truth.get("expectedChallenge") == contract.get("expectedChallenge"),
            truth.get("expectedBroadOutcome") == contract.get("expectedBroadOutcome"),
            truth.get("configurationSha256") == configuration_hash,
        ))
        return {
            "kind": "proving-ground-layout",
            "id": scenario_id,
            "contract": contract,
            "robot": truth.get("robot"),
            "contractMatches": contract_matches,
            "configurationSha256": truth.get("configurationSha256"),
        }

    scenario_id = truth.get("scenarioId")
    contract = next(
        (value for value in manifest.get("routes", []) if value.get("id") == scenario_id),
        None,
    )
    return {
        "kind": "warehouse-route",
        "id": scenario_id,
        "contract": contract,
        "robot": truth.get("platform"),
        "contractMatches": bool(contract) and truth.get("referenceEnvironmentPreserved") is True,
        "configurationSha256": None,
    }


def summarize(args: argparse.Namespace) -> dict[str, Any]:
    structural = load(args.structural)
    navigation_record = load(args.navigation)
    truth = load(args.evaluator_truth)
    manifest = load(args.scenario_manifest)
    runtime = load(args.runtime_result)
    navigation = navigation_record.get("navigation", navigation_record)
    environment_id = structural.get("environmentId")
    scenario = scenario_contract(manifest, truth)
    scenario_id = scenario["id"]
    truth_manifest_hash = truth.get("manifestSha256") or structural.get("manifestSha256", "")
    identity_valid = (
        environment_id
        and environment_id == truth.get("environmentId") == manifest.get("environmentId")
        and structural.get("manifestSha256", "").lower() == sha256(args.scenario_manifest)
        and str(truth_manifest_hash).lower() == structural.get("manifestSha256", "").lower()
        and scenario["contractMatches"]
    )
    expected_status = navigation_record.get("expectedNavigationStatus", "succeeded")
    expected_observed = navigation_record.get(
        "expectedOutcomeObserved", navigation.get("status") == expected_status
    )
    navigation_pass = (
        navigation_record.get("valid") is True
        and expected_observed is True
        and navigation.get("status") == expected_status
        and float(navigation.get("displacementMeters", 0.0)) > 2.0
        and identity_valid
    )
    trajectory = trajectory_metrics(runtime)
    recovery_count = int(navigation.get("maximumRecoveryCount", 0))
    failure_recovery = "NOT_RUN"
    if navigation_pass and (expected_status != "succeeded" or recovery_count > 0):
        failure_recovery = "FAILURE_RECOVERY_PASS"
    gates = {
        "structural": structural.get("structuralStatus", "NOT_RUN"),
        "physics": structural.get("physicsStatus", "NOT_RUN"),
        "sensor": structural.get("sensorStatus", "NOT_RUN"),
        "navigation": "NAVIGATION_PASS" if navigation_pass else "PARTIAL",
        "failureRecovery": failure_recovery,
        "interactive": "NOT_RUN",
        "headless": "HEADLESS_PASS" if (
            structural.get("headlessStatus") == "HEADLESS_PASS" and navigation_pass
        ) else "PARTIAL",
        "explanation": "EXPLANATION_READY" if (
            structural.get("explanationStatus") == "EXPLANATION_READY" and identity_valid
        ) else "PARTIAL",
    }
    required_nominal = ("structural", "physics", "sensor", "navigation", "headless", "explanation")
    nominal_ready = all(gates[name].endswith("PASS") or gates[name] == "EXPLANATION_READY"
                        for name in required_nominal)
    return {
        "schema": "crane-environment-qa-summary-v1",
        "environmentId": environment_id,
        "manifestSha256": structural.get("manifestSha256"),
        "configurationSha256": scenario["configurationSha256"],
        "scenarioKind": scenario["kind"],
        "scenarioId": scenario_id,
        "robot": scenario["robot"],
        "identityValid": bool(identity_valid),
        "nominalRouteReady": nominal_ready,
        "routeReady": nominal_ready,
        "verdict": "PARTIAL" if nominal_ready else "BLOCKED",
        "gates": gates,
        "routeContract": scenario["contract"],
        "navigation": navigation,
        "trajectory": trajectory,
        "artifacts": {
            "structural": str(args.structural),
            "navigation": str(args.navigation),
            "evaluatorTruth": str(args.evaluator_truth),
            "runtimeResult": str(args.runtime_result),
            "scenarioManifest": str(args.scenario_manifest),
        },
        "artifactSha256": {
            "structural": sha256(args.structural),
            "navigation": sha256(args.navigation),
            "evaluatorTruth": sha256(args.evaluator_truth),
            "runtimeResult": sha256(args.runtime_result),
            "scenarioManifest": sha256(args.scenario_manifest),
        },
        "limitations": [
            "Interactive visual inspection is not included in this headless summary.",
            "Failure and recovery behavior requires separate predeclared scenarios.",
            "Costmap and topic delivery do not prove internal controller consumption.",
        ],
    }


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--structural", type=Path, required=True)
    parser.add_argument("--navigation", type=Path, required=True)
    parser.add_argument("--evaluator-truth", type=Path, required=True)
    parser.add_argument("--runtime-result", type=Path, required=True)
    parser.add_argument("--scenario-manifest", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    result = summarize(args)
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(result, indent=2, sort_keys=True) + "\n", encoding="utf-8")
    print(json.dumps(result, sort_keys=True))


if __name__ == "__main__":
    main()
