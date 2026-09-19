#!/usr/bin/env python3
"""Write a bounded, machine-readable summary for the live Nav2 reset fixture."""

import argparse
import csv
import json
from pathlib import Path


def load_json(path):
    with path.open("r", encoding="utf-8") as stream:
        return json.load(stream)


def memory_bytes(value):
    raw = value.split("/", 1)[0].strip()
    units = {
        "B": 1,
        "KiB": 1024,
        "MiB": 1024 ** 2,
        "GiB": 1024 ** 3,
        "kB": 1000,
        "MB": 1000 ** 2,
        "GB": 1000 ** 3,
    }
    for unit in sorted(units, key=len, reverse=True):
        if raw.endswith(unit):
            return float(raw[:-len(unit)].strip()) * units[unit]
    return float(raw)


def load_external_resources(path):
    if not path.exists():
        return {"sampleCount": 0, "roles": {}}
    roles = {}
    sample_count = 0
    with path.open("r", encoding="utf-8", newline="") as stream:
        for row in csv.DictReader(stream):
            role = row["role"]
            cpu = float(row["cpuPercent"].rstrip("%"))
            memory = memory_bytes(row["memoryUsage"])
            values = roles.setdefault(role, {"cpu": [], "memory": []})
            values["cpu"].append(cpu)
            values["memory"].append(memory)
            sample_count += 1
    return {
        "sampleCount": sample_count,
        "roles": {
            role: {
                "sampleCount": len(values["cpu"]),
                "meanCpuPercent": sum(values["cpu"]) / len(values["cpu"]),
                "maximumCpuPercent": max(values["cpu"]),
                "maximumMemoryBytes": max(values["memory"]),
            }
            for role, values in roles.items()
        },
    }


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("result_root", type=Path)
    parser.add_argument("--require-reset", action="store_true")
    parser.add_argument("--require-occupied-costmap", action="store_true")
    parser.add_argument(
        "--expected-navigation-status",
        choices=("succeeded", "timeout", "canceled", "aborted"),
        default="succeeded",
    )
    parser.add_argument("--worker-id", type=int, default=0)
    parser.add_argument("--ros-port", type=int)
    parser.add_argument("--ros-domain-id", type=int)
    parser.add_argument("--max-action-lag-ticks", type=int, default=10)
    parser.add_argument("--command-timeout-ticks", type=int, default=25)
    args = parser.parse_args()
    root = args.result_root

    fixture = load_json(root / "fixture-summary.json")
    worker = load_json(root / f"worker-{args.worker_id}" / "result.json")
    endpoint_lines = (root / "endpoint.log").read_text(encoding="utf-8").splitlines()
    controller_lines = (root / "controller.log").read_text(
        encoding="utf-8", errors="replace").splitlines()

    active = 0
    maximum_active = 0
    connections = 0
    disconnects = 0
    for line in endpoint_lines:
        if "Connection from " in line:
            connections += 1
            active += 1
            maximum_active = max(maximum_active, active)
        elif "Disconnected from " in line:
            disconnects += 1
            active = max(0, active - 1)

    duplicate_registrations = sum(
        "already registered for node name" in line for line in endpoint_lines)
    endpoint_errors = sum("[ERROR]" in line for line in endpoint_lines)
    clock_rewinds = sum("Detected jump back in time" in line for line in controller_lines)
    goal_succeeded = any("Goal succeeded" in line for line in controller_lines)
    goal_canceled = any("Goal canceled" in line for line in controller_lines)
    cancellation_requested = any(
        "Client requested to cancel the goal" in line for line in controller_lines)
    expected_outcome_observed = {
        "succeeded": goal_succeeded,
        "timeout": cancellation_requested,
        "canceled": goal_canceled,
        "aborted": any("Goal failed" in line for line in controller_lines),
    }[args.expected_navigation_status]
    reset = worker.get("resetProbe", {})
    external_resources = load_external_resources(root / "external-resources.csv")
    action_timing = worker.get("actionTiming", {})
    accepted_actions = worker.get("acceptedActions")

    valid = all((
        fixture.get("status") == args.expected_navigation_status,
        expected_outcome_observed,
        worker.get("valid") is True,
        worker.get("rejectedActions") == 0,
        worker.get("staleActions") == 0,
        worker.get("crossEpisodeActions") == 0,
        worker.get("loggedErrors") == 0,
        worker.get("loggedExceptions") == 0,
        worker.get("invalidWaterSearches") == 0,
        duplicate_registrations == 0,
        endpoint_errors == 0,
        maximum_active <= 1,
        active == 0,
        action_timing.get("acceptedActions") == accepted_actions,
        action_timing.get("knownSourceActions") == accepted_actions,
        action_timing.get("maximumSourceToApplicationTicks", -1) <=
            args.max_action_lag_ticks,
        action_timing.get("maximumReceiveToApplicationTicks", -1) <=
            args.max_action_lag_ticks,
        not args.require_reset or reset.get("executed") is True,
        not args.require_occupied_costmap or (
            fixture.get("costmapObservations", 0) > 0 and
            fixture.get("maximumOccupiedCostmapCells", 0) > 0
        ),
    ))

    summary = {
        "schema": ("crane-nav2-scene-reload-v1" if args.require_reset else
                   "crane-nav2-worker-fixture-v1"),
        "valid": valid,
        "scope": ("authoritative-odometry-nav2-navigate-to-pose-after-scene-reload"
                  if args.require_reset else
                  "authoritative-odometry-nav2-navigate-to-pose"),
        "resetRequired": args.require_reset,
        "occupiedCostmapRequired": args.require_occupied_costmap,
        "expectedNavigationStatus": args.expected_navigation_status,
        "expectedOutcomeObserved": expected_outcome_observed,
        "workerId": args.worker_id,
        "navigation": fixture,
        "transport": {
            "rosTcpPort": args.ros_port,
            "rosDomainId": args.ros_domain_id,
            "connections": connections,
            "disconnects": disconnects,
            "maximumConcurrentUnityConnections": maximum_active,
            "activeConnectionsAtShutdown": active,
            "duplicateNodeRegistrations": duplicate_registrations,
            "endpointErrors": endpoint_errors,
        },
        "clock": {
            "rewindWarnings": clock_rewinds,
            "recovered": clock_rewinds > 0 and goal_succeeded,
        },
        "actions": {
            "accepted": accepted_actions,
            "rejected": worker.get("rejectedActions"),
            "stale": worker.get("staleActions"),
            "crossEpisode": worker.get("crossEpisodeActions"),
            "policy": "bounded-lag-latest-value",
            "maximumLagTicks": args.max_action_lag_ticks,
            "commandTimeoutTicks": args.command_timeout_ticks,
            "timing": action_timing,
        },
        "observations": {
            "depthAcquisitions": worker.get("depthCamera", {}).get("acquisitionCount"),
            "detectionAcquisitions": worker.get("detections", {}).get("acquisitionCount"),
            "lidarScans": worker.get("lidar", {}).get("scanCount"),
            "stale": worker.get("staleObservations"),
            "failed": worker.get("failedObservations"),
            "invalidWaterSearches": worker.get("invalidWaterSearches"),
        },
        "performance": {
            "measurementWallSeconds": worker.get("wallSeconds"),
            "simulatedSeconds": worker.get("simulatedSeconds"),
            "processCpuUtilizationPercent": worker.get("processCpuUtilizationPercent"),
            "meanGpuFrameMilliseconds": worker.get("meanGpuFrameMilliseconds"),
            "monoUsedBytes": worker.get("monoUsedBytes"),
            "totalAllocatedMemoryBytes": worker.get("totalAllocatedMemoryBytes"),
        },
        "externalResources": external_resources,
        "reset": reset,
        "realTimeFactor": worker.get("realTimeFactor"),
    }
    output = root / "navigation-reset-summary.json"
    output.write_text(json.dumps(summary, indent=2) + "\n", encoding="utf-8")
    print(json.dumps(summary, sort_keys=True))
    raise SystemExit(0 if valid else 1)


if __name__ == "__main__":
    main()
