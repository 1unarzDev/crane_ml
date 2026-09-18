#!/usr/bin/env python3
"""Combine the UDP peer and Unity benchmark into a bounded SITL acceptance record."""

import argparse
import json
from pathlib import Path


def load(path):
    with path.open("r", encoding="utf-8") as stream:
        return json.load(stream)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("result_root", type=Path)
    parser.add_argument("--worker-id", type=int, default=0)
    parser.add_argument("--maximum-lag-ticks", type=int, default=5)
    args = parser.parse_args()

    fixture = load(args.result_root / "fixture-summary.json")
    worker = load(args.result_root / f"worker-{args.worker_id}" / "result.json")
    sitl = worker.get("sitl", {})
    timing = worker.get("actionTiming", {})
    accepted = worker.get("acceptedActions")
    received = worker.get("unknownSourceActions")
    valid = all((
        fixture.get("valid") is True,
        worker.get("valid") is True,
        accepted is not None and accepted > 0,
        worker.get("rejectedActions") == 0,
        worker.get("staleActions") == 0,
        worker.get("crossEpisodeActions") == 0,
        received == sitl.get("validServoPackets"),
        accepted <= received,
        timing.get("acceptedActions") == accepted,
        timing.get("knownSourceActions") == 0,
        timing.get("maximumReceiveToApplicationTicks", -1) <= args.maximum_lag_ticks,
        sitl.get("validServoPackets", 0) >= accepted,
        sitl.get("invalidServoPackets", 0) >= 1,
        sitl.get("telemetryPackets", 0) > 0,
        sitl.get("lastFrame") == fixture.get("lastFrame"),
    ))
    result = {
        "schema": "crane-sitl-protocol-acceptance-v1",
        "valid": valid,
        "scope": "ardupilot-json-udp-aquatic-omni-x-loopback",
        "workerId": args.worker_id,
        "realTimeFactor": worker.get("realTimeFactor"),
        "actions": {
            "accepted": accepted,
            "received": received,
            "replacedBeforeApplication": received - accepted,
            "sourceObservationProvenance": "unavailable-in-servo-protocol",
            "rejected": worker.get("rejectedActions"),
            "stale": worker.get("staleActions"),
            "maximumLagTicks": args.maximum_lag_ticks,
            "timing": timing,
        },
        "sitl": sitl,
        "peer": fixture,
        "observations": {
            "stale": worker.get("staleObservations"),
            "failed": worker.get("failedObservations"),
            "invalidWaterSearches": worker.get("invalidWaterSearches"),
        },
        "limitations": [
            "protocol loopback only; no ArduPilot/PX4 process",
            "servo packets have no source-observation timestamp",
            "aquatic Omni-X PWM mapping; no aerial flight controller",
        ],
    }
    output = args.result_root / "sitl-summary.json"
    output.write_text(json.dumps(result, indent=2) + "\n", encoding="utf-8")
    print(json.dumps(result, sort_keys=True))
    raise SystemExit(0 if valid else 1)


if __name__ == "__main__":
    main()
