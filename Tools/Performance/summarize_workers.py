#!/usr/bin/env python3
import argparse
import json
from pathlib import Path

parser = argparse.ArgumentParser()
parser.add_argument("root", type=Path)
parser.add_argument("--output", type=Path)
args = parser.parse_args()

results = []
for path in sorted(args.root.glob("worker-*/result.json")):
    value = json.loads(path.read_text())
    value["resultPath"] = str(path)
    results.append(value)

valid_results = [item for item in results if bool(item.get("valid"))]
invalid_results = [item for item in results if not bool(item.get("valid"))]


def maximum_observation_age(item):
    ages = [item.get("maximumObservationQueueAgeTicks", -1),
            item.get("observationQueueAgeTicks", -1)]
    ages.extend(sample.get("observationQueueAgeTicks", -1)
                for sample in item.get("validation", []))
    return max(ages, default=-1)


summary = {
    "workerCount": len(results),
    "validWorkers": len(valid_results),
    "invalidWorkers": len(invalid_results),
    "allWorkersValid": len(results) > 0 and not invalid_results,
    "aggregateSimulatedSecondsPerSecond": sum(item.get("realTimeFactor", 0) for item in results),
    "aggregateValidSimulatedSecondsPerSecond": sum(
        item.get("realTimeFactor", 0) for item in valid_results
    ),
    "aggregateRejectedSimulatedSecondsPerSecond": sum(
        item.get("realTimeFactor", 0) for item in invalid_results
    ),
    "aggregateSimulatedSeconds": sum(item.get("simulatedSeconds", 0) for item in results),
    "maximumObservationQueueAgeTicks": max(
        (maximum_observation_age(item) for item in results), default=-1
    ),
    "staleObservations": sum(item.get("staleObservations", 0) for item in results),
    "failedObservations": sum(item.get("failedObservations", 0) for item in results),
    "workers": [
        {
            "workerId": item.get("workerId"),
            "valid": item.get("valid"),
            "realTimeFactor": item.get("realTimeFactor"),
            "processCpuUtilizationPercent": item.get("processCpuUtilizationPercent"),
            "meanGpuFrameMilliseconds": item.get("meanGpuFrameMilliseconds"),
            "totalAllocatedMemoryBytes": item.get("totalAllocatedMemoryBytes"),
            "resultPath": item["resultPath"],
        }
        for item in results
    ],
}
encoded = json.dumps(summary, indent=2) + "\n"
if args.output:
    args.output.write_text(encoded)
print(encoded, end="")
