#!/usr/bin/env python3
"""Exercise CRANE's ArduPilot JSON/SITL UDP servo and telemetry contract."""

import argparse
import json
import math
from pathlib import Path
import socket
import struct
import time


SERVO_MAGIC = 18458
SERVO_CHANNELS = 16


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--host", default="127.0.0.1")
    parser.add_argument("--port", type=int, default=9002)
    parser.add_argument("--duration", type=float, default=4.0)
    parser.add_argument("--frame-rate", type=int, default=50)
    parser.add_argument("--pwm", type=int, default=1600)
    parser.add_argument("--mode", choices=("aquatic", "aerial"), default="aquatic")
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()

    destination = (args.host, args.port)
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.bind((args.host, 0))
    sock.setblocking(False)

    # The bridge must count and reject this datagram without treating it as a peer.
    sock.sendto(b"invalid", destination)
    started = time.monotonic()
    next_send = started
    frame = 1
    sent = 0
    received = 0
    malformed_telemetry = 0
    monotonic = True
    timestamp_regressions = 0
    maximum_timestamp_regression = 0.0
    telemetry_shape_valid = True
    timestamps = []

    while time.monotonic() - started < args.duration:
        now = time.monotonic()
        if now >= next_send:
            channels = [1500] * SERVO_CHANNELS
            channels[:4] = [args.pwm] * 4
            packet = struct.pack("<HHI16H", SERVO_MAGIC, args.frame_rate,
                                 frame, *channels)
            sock.sendto(packet, destination)
            frame += 1
            sent += 1
            next_send += 1.0 / args.frame_rate

        while True:
            try:
                payload, _ = sock.recvfrom(65535)
            except BlockingIOError:
                break
            try:
                message = json.loads(payload.decode("utf-8").strip())
                timestamp = float(message["timestamp"])
                shapes = (
                    len(message["imu"]["gyro"]) == 3 and
                    len(message["imu"]["accel_body"]) == 3 and
                    len(message["position"]) == 3 and
                    len(message["attitude"]) == 3 and
                    len(message["velocity"]) == 3
                )
                finite = all(math.isfinite(float(value)) for values in (
                    message["imu"]["gyro"], message["imu"]["accel_body"],
                    message["position"], message["attitude"], message["velocity"])
                             for value in values)
                telemetry_shape_valid &= shapes and finite and math.isfinite(timestamp)
                if timestamps and timestamp + 1e-6 < timestamps[-1]:
                    monotonic = False
                    timestamp_regressions += 1
                    maximum_timestamp_regression = max(
                        maximum_timestamp_regression, timestamps[-1] - timestamp)
                timestamps.append(timestamp)
                received += 1
            except (KeyError, TypeError, ValueError, UnicodeDecodeError, json.JSONDecodeError):
                malformed_telemetry += 1
        time.sleep(0.001)

    sock.close()
    wall_seconds = time.monotonic() - started
    timestamp_span = timestamps[-1] - timestamps[0] if len(timestamps) > 1 else 0.0
    result = {
        "schema": "crane-sitl-udp-fixture-v1",
        "valid": (
            sent > 0 and received > 0 and malformed_telemetry == 0 and
            monotonic and telemetry_shape_valid and timestamp_span > 0
        ),
        "destination": f"{args.host}:{args.port}",
        "wallSeconds": wall_seconds,
        "servoPacketsSent": sent,
        "firstFrame": 1,
        "lastFrame": frame - 1,
        "telemetryPacketsReceived": received,
        "malformedTelemetryPackets": malformed_telemetry,
        "telemetryShapeValid": telemetry_shape_valid,
        "timestampMonotonic": monotonic,
        "timestampRegressions": timestamp_regressions,
        "maximumTimestampRegression": maximum_timestamp_regression,
        "firstSimulatedTimestamp": timestamps[0] if timestamps else -1,
        "lastSimulatedTimestamp": timestamps[-1] if timestamps else -1,
        "timestampSpanSimulatedSeconds": timestamp_span,
        "measuredTelemetryClockRate": timestamp_span / wall_seconds,
        "limitations": [
            "protocol loopback only; no ArduPilot/PX4 process",
            ("direct multirotor PWM mapping; no flight-controller process" if
             args.mode == "aerial" else
             "aquatic Omni-X PWM mapping; no aerial flight controller"),
        ],
    }
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(result, indent=2) + "\n", encoding="utf-8")
    print(json.dumps(result, sort_keys=True))
    raise SystemExit(0 if result["valid"] else 1)


if __name__ == "__main__":
    main()
