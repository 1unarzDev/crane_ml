#!/usr/bin/env python3
"""Gate repeated environment QA runs without treating repeats as independent episodes."""

from __future__ import annotations

import argparse
import hashlib
import json
import statistics
from pathlib import Path
from typing import Any


def load(path: Path) -> dict[str, Any]:
    return json.loads(path.read_text(encoding="utf-8"))


def sha256(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


def range_summary(values: list[float]) -> dict[str, float] | None:
    if not values:
        return None
    return {
        "minimum": min(values),
        "median": statistics.median(values),
        "maximum": max(values),
    }


def resolve_contract(contract: dict[str, Any], environment_id: str,
                     scenario_id: str) -> dict[str, Any]:
    if contract.get("schema") != "crane-environment-repetition-contract-v1":
        raise ValueError("Unsupported repetition contract schema")
    matches = [
        value for value in contract.get("scenarios", [])
        if value.get("environmentId") == environment_id
        and value.get("scenarioId") == scenario_id
    ]
    if len(matches) != 1:
        raise ValueError(
            f"Expected one repetition contract for {environment_id}/{scenario_id}, "
            f"found {len(matches)}"
        )
    return matches[0]


def summarize(contract_path: Path, environment_id: str, scenario_id: str,
              summary_paths: list[Path]) -> dict[str, Any]:
    contract_root = load(contract_path)
    contract = resolve_contract(contract_root, environment_id, scenario_id)
    records = [load(path) for path in summary_paths]
    minimum_runs = int(contract["minimumRuns"])
    expected_status = contract["expectedNavigationStatus"]
    minimum_recovery = int(contract.get("minimumRecoveryCount", 0))
    required_gates = contract.get("requiredGates", {})

    configuration_hashes = {
        record.get("configurationSha256") for record in records
    }
    runtime_hashes = [
        record.get("artifactSha256", {}).get("runtimeResult") for record in records
    ]
    navigation_hashes = [
        record.get("artifactSha256", {}).get("navigation") for record in records
    ]
    checks = {
        "minimumRunCount": len(records) >= minimum_runs,
        "identityMatches": all(
            record.get("environmentId") == environment_id
            and record.get("scenarioId") == scenario_id
            and record.get("identityValid") is True
            for record in records
        ),
        "configurationMatches": (
            len(configuration_hashes) == 1 and None not in configuration_hashes
        ),
        "runtimeArtifactsDistinct": (
            len(runtime_hashes) == len(set(runtime_hashes)) and None not in runtime_hashes
        ),
        "navigationArtifactsDistinct": (
            len(navigation_hashes) == len(set(navigation_hashes))
            and None not in navigation_hashes
        ),
        "navigationStatusMatches": all(
            record.get("navigation", {}).get("status") == expected_status
            for record in records
        ),
        "routeAcceptancePasses": all(
            record.get("routeAcceptance", {}).get("passed") is True
            for record in records
        ),
        "minimumRecoveryCount": all(
            int(record.get("navigation", {}).get("maximumRecoveryCount", 0))
            >= minimum_recovery
            for record in records
        ),
        "requiredGatesPass": all(
            record.get("gates", {}).get(name) == expected
            for record in records
            for name, expected in required_gates.items()
        ),
    }
    passed = bool(records) and all(checks.values())

    def numbers(*keys: str) -> list[float]:
        values: list[float] = []
        for record in records:
            current: Any = record
            for key in keys:
                current = current.get(key) if isinstance(current, dict) else None
            if isinstance(current, (int, float)) and not isinstance(current, bool):
                values.append(float(current))
        return values

    return {
        "schema": "crane-environment-repetition-summary-v1",
        "environmentId": environment_id,
        "scenarioId": scenario_id,
        "status": "REPETITION_PASS" if passed else "PARTIAL",
        "passed": passed,
        "runCount": len(records),
        "minimumRuns": minimum_runs,
        "expectedNavigationStatus": expected_status,
        "minimumRecoveryCount": minimum_recovery,
        "configurationSha256": (
            next(iter(configuration_hashes)) if len(configuration_hashes) == 1 else None
        ),
        "checks": checks,
        "metrics": {
            "wallSeconds": range_summary(numbers("navigation", "wallSeconds")),
            "displacementMeters": range_summary(numbers("navigation", "displacementMeters")),
            "sampledPathLengthMeters": range_summary(
                numbers("trajectory", "sampledPathLengthMeters")
            ),
            "maximumRecoveryCount": range_summary(
                numbers("navigation", "maximumRecoveryCount")
            ),
            "lateralDirectionChangeCount": range_summary(
                numbers("trajectory", "lateralDirectionChangeCount")
            ),
        },
        "contract": str(contract_path),
        "contractSha256": sha256(contract_path),
        "artifacts": [
            {"path": str(path), "sha256": sha256(path)} for path in summary_paths
        ],
        "limitations": [
            "Exact-condition repetitions measure operational reproducibility, not independent "
            "scenario diversity or statistical sample size.",
            "Delivered topic, costmap, and odometry evidence does not prove controller consumption.",
            "A repeated temporal association does not by itself establish physical causation."
        ],
    }


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--contract", type=Path, required=True)
    parser.add_argument("--environment-id", required=True)
    parser.add_argument("--scenario-id", required=True)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("summaries", type=Path, nargs="+")
    args = parser.parse_args()
    result = summarize(
        args.contract, args.environment_id, args.scenario_id, args.summaries
    )
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(
        json.dumps(result, indent=2, sort_keys=True) + "\n", encoding="utf-8"
    )
    print(json.dumps(result, sort_keys=True))
    raise SystemExit(0 if result["passed"] else 1)


if __name__ == "__main__":
    main()
