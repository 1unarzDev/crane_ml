#!/usr/bin/env python3
"""Aggregate isolated Nav2 worker fixtures into one validity/throughput record."""

import argparse
import json
from pathlib import Path


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("result_root", type=Path)
    parser.add_argument("--workers", type=int, required=True)
    parser.add_argument("--wall-seconds", type=float, required=True)
    args = parser.parse_args()

    records = []
    for worker_id in range(args.workers):
        path = args.result_root / f"nav2-worker-{worker_id}" / "navigation-reset-summary.json"
        with path.open("r", encoding="utf-8") as stream:
            records.append(json.load(stream))

    ports = [record["transport"]["rosTcpPort"] for record in records]
    domains = [record["transport"]["rosDomainId"] for record in records]
    aggregate_rtf = sum(record["realTimeFactor"] for record in records)
    measured_simulated_seconds = sum(
        record["performance"]["simulatedSeconds"] for record in records)
    valid = all(record["valid"] for record in records)
    valid = valid and len(set(ports)) == args.workers and len(set(domains)) == args.workers
    valid = valid and all(record["externalResources"]["sampleCount"] > 0
                          for record in records)
    external_roles = {}
    for record in records:
        for role, values in record["externalResources"]["roles"].items():
            totals = external_roles.setdefault(role, {
                "sumMeanCpuPercent": 0.0,
                "sumMaximumMemoryBytes": 0.0,
            })
            totals["sumMeanCpuPercent"] += values["meanCpuPercent"]
            totals["sumMaximumMemoryBytes"] += values["maximumMemoryBytes"]

    summary = {
        "schema": "crane-nav2-worker-sweep-v1",
        "valid": valid,
        "workerCount": args.workers,
        "wallSeconds": args.wall_seconds,
        "aggregateMeasuredRealTimeFactor": aggregate_rtf,
        "measuredSimulatedSeconds": measured_simulated_seconds,
        "measuredSecondsPerCampaignWallSecond": (
            measured_simulated_seconds / args.wall_seconds if args.wall_seconds > 0 else 0.0),
        "uniqueRosTcpPorts": len(set(ports)),
        "uniqueRosDomainIds": len(set(domains)),
        "externalResourceSamples": sum(
            record["externalResources"]["sampleCount"] for record in records),
        "externalRoleTotals": external_roles,
        "workers": records,
        "limitations": [
            "authoritative odometry; localization and SLAM excluded",
            "latest-delivered provenance; internal Nav2 observation consumption unproven",
            "container CPU percentages are Docker interval samples and can exceed 100%",
        ],
    }
    output = args.result_root / "sweep-summary.json"
    output.write_text(json.dumps(summary, indent=2) + "\n", encoding="utf-8")
    print(json.dumps(summary, sort_keys=True))
    raise SystemExit(0 if valid else 1)


if __name__ == "__main__":
    main()
