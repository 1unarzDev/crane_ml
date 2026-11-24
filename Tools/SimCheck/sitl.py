"""SITL UDP acceptance: the player's ArduPilot JSON servo/telemetry contract over loopback.

``peer`` plays the flight controller: it sends one malformed datagram (the player must count and
reject it), then 16-channel servo packets at ``frame_rate`` and checks every telemetry reply
(JSON with finite imu.gyro, imu.accel_body, position, attitude and velocity triples and a
monotonic timestamp). ``acceptance`` joins the peer's record with the player's benchmark result.
"""

import json
import math
import socket
import struct
import time
from pathlib import Path

from . import report

SERVO_MAGIC = 18458
SERVO_CHANNELS = 16
TRIPLES = (('imu', 'gyro'), ('imu', 'accel_body'), ('position',), ('attitude',), ('velocity',))
LIMITATIONS = {
    'aquatic': 'aquatic Omni-X PWM mapping; no aerial flight controller',
    'aerial': 'direct multirotor PWM mapping; no flight-controller process',
}


def servo_packet(frame, frame_rate, pwm):
    channels = [1500] * SERVO_CHANNELS
    channels[:4] = [pwm] * 4
    return struct.pack('<HHI16H', SERVO_MAGIC, frame_rate, frame, *channels)


def telemetry(payload):
    """(timestamp, shape and values valid) of one telemetry datagram; raises on a malformed one."""
    message = json.loads(payload.decode('utf-8').strip())
    timestamp = float(message['timestamp'])
    triples = []
    for path in TRIPLES:
        value = message
        for key in path:
            value = value[key]
        triples.append([float(v) for v in value])
    valid = all(len(t) == 3 and all(math.isfinite(v) for v in t) for t in triples) and math.isfinite(timestamp)
    return timestamp, valid


def peer(host='127.0.0.1', port=9002, duration=4.0, frame_rate=50, pwm=1600, mode='aquatic'):
    destination = (host, port)
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.bind((host, 0))
    sock.setblocking(False)
    sock.sendto(b'invalid', destination)
    started = next_send = time.monotonic()
    frame, sent, malformed, regressions, worst_regression = 1, 0, 0, 0, 0.0
    shape_valid, stamps = True, []
    while time.monotonic() - started < duration:
        if time.monotonic() >= next_send:
            sock.sendto(servo_packet(frame, frame_rate, pwm), destination)
            frame, sent = frame + 1, sent + 1
            next_send += 1.0 / frame_rate
        while True:
            try:
                payload, _ = sock.recvfrom(65535)
            except BlockingIOError:
                break
            try:
                stamp, ok = telemetry(payload)
            except (KeyError, TypeError, ValueError, UnicodeDecodeError):
                malformed += 1
                continue
            shape_valid &= ok
            if stamps and stamp + 1e-6 < stamps[-1]:
                regressions += 1
                worst_regression = max(worst_regression, stamps[-1] - stamp)
            stamps.append(stamp)
        time.sleep(0.001)
    sock.close()
    wall = time.monotonic() - started
    span = stamps[-1] - stamps[0] if len(stamps) > 1 else 0.0
    return {
        'schema': 'crane-sitl-udp-fixture-v1',
        'valid': sent > 0 and bool(stamps) and malformed == 0 and regressions == 0 and shape_valid and span > 0,
        'destination': f'{host}:{port}',
        'wallSeconds': wall,
        'servoPacketsSent': sent,
        'firstFrame': 1,
        'lastFrame': frame - 1,
        'telemetryPacketsReceived': len(stamps),
        'malformedTelemetryPackets': malformed,
        'telemetryShapeValid': shape_valid,
        'timestampMonotonic': regressions == 0,
        'timestampRegressions': regressions,
        'maximumTimestampRegression': worst_regression,
        'firstSimulatedTimestamp': stamps[0] if stamps else -1,
        'lastSimulatedTimestamp': stamps[-1] if stamps else -1,
        'timestampSpanSimulatedSeconds': span,
        'measuredTelemetryClockRate': span / wall if wall > 0 else 0.0,
        'limitations': ['protocol loopback only; no ArduPilot/PX4 process', LIMITATIONS[mode]],
    }


def acceptance(fixture, worker, maximum_lag_ticks, scope='aquatic', climb=None, min_climb=0.1):
    """The peer's record and the player's result agree; ``climb`` is the aerial vertical displacement."""
    sitl, timing = worker.get('sitl', {}), worker.get('actionTiming', {})
    accepted, received = worker.get('acceptedActions'), worker.get('unknownSourceActions')
    conditions = {
        'peer valid': fixture.get('valid') is True,
        'player result valid': worker.get('valid') is True,
        'actions accepted': accepted is not None and accepted > 0,
        'no rejected actions': worker.get('rejectedActions') == 0,
        'no stale actions': worker.get('staleActions') == 0,
        'no cross-episode actions': worker.get('crossEpisodeActions') == 0,
        'every valid servo packet received': received == sitl.get('validServoPackets'),
        'accepted within received': accepted is not None and received is not None and accepted <= received,
        'timing counts every accepted action': timing.get('acceptedActions') == accepted,
        'no known-source actions': timing.get('knownSourceActions') == 0,
        'action lag within bound': timing.get('maximumReceiveToApplicationTicks', -1) <= maximum_lag_ticks,
        'servo packets cover accepted actions': accepted is not None and sitl.get('validServoPackets', 0) >= accepted,
        'malformed datagram rejected': sitl.get('invalidServoPackets', 0) >= 1,
        'telemetry sent': sitl.get('telemetryPackets', 0) > 0,
        'last frame matches the peer': sitl.get('lastFrame') == fixture.get('lastFrame'),
    }
    if scope == 'aerial':
        conditions['aerial climb'] = climb is not None and climb > min_climb
    failures = [name for name, ok in conditions.items() if not ok]
    return report('sitl', failures, schema='crane-sitl-protocol-acceptance-v1',
                  scope=f'ardupilot-json-udp-{scope}-loopback', realTimeFactor=worker.get('realTimeFactor'),
                  actions={'accepted': accepted, 'received': received,
                           'replacedBeforeApplication': None if None in (accepted, received) else received - accepted,
                           'rejected': worker.get('rejectedActions'), 'stale': worker.get('staleActions'),
                           'maximumLagTicks': maximum_lag_ticks, 'timing': timing},
                  sitl=sitl, peer=fixture, aerialVerticalDisplacement=climb,
                  observations={'stale': worker.get('staleObservations'), 'failed': worker.get('failedObservations'),
                                'invalidWaterSearches': worker.get('invalidWaterSearches')},
                  limitations=['protocol loopback only; no ArduPilot/PX4 process',
                               'servo packets have no source-observation timestamp', LIMITATIONS[scope]])


def climb_of(validation_jsonl, body='Reference Quadrotor'):
    heights = []
    with open(validation_jsonl, encoding='utf-8') as stream:
        for line in stream:
            for b in json.loads(line).get('bodies', []):
                if b.get('name') == body:
                    heights.append(float(b['position']['y']))
    return heights[-1] - heights[0] if len(heights) >= 2 else None


def acceptance_dir(result_root, maximum_lag_ticks, scope='aquatic', worker_id=0, min_climb=0.1):
    """``acceptance`` over a result root: fixture-summary.json and worker-N/result.json."""
    root = Path(result_root)
    worker_dir = root / f'worker-{worker_id}'
    fixture = json.loads((root / 'fixture-summary.json').read_text(encoding='utf-8'))
    worker = json.loads((worker_dir / 'result.json').read_text(encoding='utf-8'))
    climb = climb_of(worker_dir / 'result.validation.jsonl') if scope == 'aerial' else None
    return acceptance(fixture, worker, maximum_lag_ticks, scope, climb, min_climb)
