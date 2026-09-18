#!/usr/bin/env python3
"""Write a bounded, machine-readable summary for the live Nav2 reset fixture."""

import argparse
import json
from pathlib import Path


def load_json(path):
    with path.open("r", encoding="utf-8") as stream:
        return json.load(stream)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("result_root", type=Path)
    parser.add_argument("--require-reset", action="store_true")
    args = parser.parse_args()
    root = args.result_root

    fixture = load_json(root / "fixture-summary.json")
    worker = load_json(root / "worker-0" / "result.json")
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
    reset = worker.get("resetProbe", {})

    valid = all((
        fixture.get("status") == "succeeded",
        goal_succeeded,
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
        not args.require_reset or reset.get("executed") is True,
    ))

    summary = {
        "schema": "crane-nav2-scene-reload-v1",
        "valid": valid,
        "scope": ("authoritative-odometry-nav2-navigate-to-pose-after-scene-reload"
                  if args.require_reset else
                  "authoritative-odometry-nav2-navigate-to-pose"),
        "resetRequired": args.require_reset,
        "navigation": fixture,
        "transport": {
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
            "accepted": worker.get("acceptedActions"),
            "rejected": worker.get("rejectedActions"),
            "stale": worker.get("staleActions"),
            "crossEpisode": worker.get("crossEpisodeActions"),
        },
        "observations": {
            "depthAcquisitions": worker.get("depthCamera", {}).get("acquisitionCount"),
            "detectionAcquisitions": worker.get("detections", {}).get("acquisitionCount"),
            "lidarScans": worker.get("lidar", {}).get("scanCount"),
            "invalidWaterSearches": worker.get("invalidWaterSearches"),
        },
        "reset": reset,
        "realTimeFactor": worker.get("realTimeFactor"),
    }
    output = root / "navigation-reset-summary.json"
    output.write_text(json.dumps(summary, indent=2) + "\n", encoding="utf-8")
    print(json.dumps(summary, sort_keys=True))
    raise SystemExit(0 if valid else 1)


if __name__ == "__main__":
    main()
