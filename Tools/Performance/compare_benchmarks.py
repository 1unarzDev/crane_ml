#!/usr/bin/env python3
"""Compare CRANE benchmark trajectories at equal simulated timestamps."""

import argparse
import bisect
import json
import math
from pathlib import Path


VECTOR_FIELDS = ("position", "linearVelocity", "angularVelocity")


def vector(value):
    return value["x"], value["y"], value["z"]


def interpolate(samples, time, body_name):
    times = [sample["simulatedSeconds"] for sample in samples]
    upper = bisect.bisect_left(times, time)
    if upper == 0 or upper == len(samples):
        return None
    before, after = samples[upper - 1], samples[upper]
    fraction = (time - before["simulatedSeconds"]) / (
        after["simulatedSeconds"] - before["simulatedSeconds"]
    )
    first = next(body for body in before["bodies"] if body["name"] == body_name)
    second = next(body for body in after["bodies"] if body["name"] == body_name)
    result = {}
    for field in VECTOR_FIELDS:
        result[field] = tuple(
            left + (right - left) * fraction
            for left, right in zip(vector(first[field]), vector(second[field]))
        )
    q1 = vector(first["rotation"]) + (first["rotation"]["w"],)
    q2 = vector(second["rotation"]) + (second["rotation"]["w"],)
    if sum(left * right for left, right in zip(q1, q2)) < 0:
        q2 = tuple(-value for value in q2)
    rotation = tuple(left + (right - left) * fraction for left, right in zip(q1, q2))
    magnitude = math.sqrt(sum(value * value for value in rotation))
    result["rotation"] = tuple(value / magnitude for value in rotation)

    water_before = before["water"][0]["projected"]["y"]
    water_after = after["water"][0]["projected"]["y"]
    result["waterHeight"] = water_before + (water_after - water_before) * fraction
    return result


def rms(values):
    return math.sqrt(sum(value * value for value in values) / len(values))


def compare(reference, candidate, period):
    reference_samples = reference["validation"]
    candidate_samples = candidate["validation"]
    names = sorted(
        {body["name"] for body in reference_samples[0]["bodies"]}
        & {body["name"] for body in candidate_samples[0]["bodies"]}
    )
    start = max(reference_samples[0]["simulatedSeconds"], candidate_samples[0]["simulatedSeconds"])
    end = min(reference_samples[-1]["simulatedSeconds"], candidate_samples[-1]["simulatedSeconds"])
    values = {field: [] for field in (*VECTOR_FIELDS, "attitudeDegrees", "waterHeight")}
    time = math.ceil(start / period) * period
    compared_times = 0
    while time < end:
        compared_times += 1
        for name in names:
            first = interpolate(reference_samples, time, name)
            second = interpolate(candidate_samples, time, name)
            if first is None or second is None:
                continue
            for field in VECTOR_FIELDS:
                values[field].append(math.dist(first[field], second[field]))
            dot = min(1.0, abs(sum(a * b for a, b in zip(first["rotation"], second["rotation"]))))
            values["attitudeDegrees"].append(math.degrees(2 * math.acos(dot)))
            values["waterHeight"].append(abs(first["waterHeight"] - second["waterHeight"]))
        time += period
    return {
        "reference": {
            "scenario": reference.get("scenario"),
            "realTimeFactor": reference.get("realTimeFactor"),
            "valid": reference.get("valid"),
        },
        "candidate": {
            "scenario": candidate.get("scenario"),
            "realTimeFactor": candidate.get("realTimeFactor"),
            "valid": candidate.get("valid"),
        },
        "comparedBodies": len(names),
        "comparedTimes": compared_times,
        "metrics": {
            name: {"rms": rms(samples), "max": max(samples), "count": len(samples)}
            for name, samples in values.items() if samples
        },
    }


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("reference", type=Path)
    parser.add_argument("candidate", type=Path)
    parser.add_argument("--period", type=float, default=0.1)
    parser.add_argument("--output", type=Path)
    arguments = parser.parse_args()
    result = compare(
        json.loads(arguments.reference.read_text()),
        json.loads(arguments.candidate.read_text()),
        arguments.period,
    )
    encoded = json.dumps(result, indent=2)
    if arguments.output:
        arguments.output.parent.mkdir(parents=True, exist_ok=True)
        arguments.output.write_text(encoded + "\n")
    print(encoded)


if __name__ == "__main__":
    main()
