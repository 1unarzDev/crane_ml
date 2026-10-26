#!/usr/bin/env python3
"""Passive ROS 2 raw capture; reception timing is never substituted for acquisition."""
import argparse
import base64
import gzip
import hashlib
import json
import math
from pathlib import Path
import signal
import time
import traceback


def clean(value):
    """JSON view uses null for nonfinite floats; original bytes remain in cdrBase64."""
    if isinstance(value, float) and not math.isfinite(value):
        return None
    if isinstance(value, dict):
        return {k: clean(v) for k, v in value.items()}
    if isinstance(value, (list, tuple)):
        return [clean(v) for v in value]
    if hasattr(value, 'tolist'):
        return clean(value.tolist())
    return value


def parse_topics(values):
    result = []
    for value in values:
        topic, separator, message_type = value.partition('=')
        if not separator or not topic.startswith('/') or len(message_type.split('/')) != 3 or '/msg/' not in message_type:
            raise ValueError('Topic must be /name=package/msg/Type: ' + value)
        if any(x['topic'] == topic for x in result):
            raise ValueError('Duplicate topic: ' + topic)
        result.append({'topic': topic, 'messageType': message_type})
    return result


def envelope(topic, message_type, sequence, message, cdr, ros_ns, monotonic_ns, wall_ns, source):
    header = message.get('header') if isinstance(message, dict) else None
    stamp = header.get('stamp') if isinstance(header, dict) else None
    return {'schema': 'crane-physical-ros-record-v1', 'dataSource': source,
            'topic': topic, 'messageType': message_type, 'sequence': sequence,
            'acquisitionStamp': stamp, 'receptionRosNanoseconds': ros_ns,
            'receptionMonotonicNanoseconds': monotonic_ns, 'receptionUnixNanoseconds': wall_ns,
            'cdrBase64': base64.b64encode(cdr).decode('ascii'),
            'cdrSha256': hashlib.sha256(cdr).hexdigest(), 'message': clean(message)}


def file_hash(path):
    digest = hashlib.sha256()
    with path.open('rb') as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b''):
            digest.update(block)
    return digest.hexdigest()


def capture(args, ros_args):
    output = args.output
    output.mkdir(parents=True, exist_ok=True)
    manifest_path = output / 'manifest.json'
    records_path = output / 'records.jsonl.gz'
    if manifest_path.exists() or records_path.exists():
        raise FileExistsError('Capture output already contains evidence; choose an empty directory')
    topics = parse_topics(args.topic)
    metadata = json.loads(args.metadata.read_text()) if args.metadata else {}
    if args.source == 'physical' and not all(metadata.get(k) for k in ('platformId', 'sessionId', 'siteId', 'measurementSource')):
        raise ValueError('Physical metadata requires platformId, sessionId, siteId, measurementSource')
    if not math.isfinite(args.seconds) or args.seconds <= 0:
        raise ValueError('seconds must be positive and finite')
    counts = {x['topic']: 0 for x in topics}
    start_ns = time.time_ns()
    manifest = {'schema': 'crane-physical-ros-capture-v1', 'dataSource': args.source,
                'physicalRobotEvidence': args.source == 'physical', 'metadata': metadata,
                'topics': topics, 'startUnixNanoseconds': start_ns,
                'timingBoundary': 'Header acquisition stamp is preserved; publish/application timestamps are unavailable unless present in messages. Reception ROS/system and monotonic clocks are independent. Recorder does not establish device-clock synchronization.',
                'status': 'starting', 'useSimTime': None, 'errors': []}
    node = None
    rclpy = None
    termination = 'duration'
    running = [True]
    for sig in (signal.SIGINT, signal.SIGTERM):
        signal.signal(sig, lambda *_: running.__setitem__(0, False))
    try:
        import rclpy
        from rclpy.node import Node
        from rclpy.qos import qos_profile_sensor_data
        from rclpy.serialization import serialize_message
        from rclpy.signals import SignalHandlerOptions
        from rosidl_runtime_py.convert import message_to_ordereddict
        from rosidl_runtime_py.utilities import get_message
        rclpy.init(args=ros_args, signal_handler_options=SignalHandlerOptions.NO)
        node = Node('crane_fidelity_passive_capture')
        manifest['useSimTime'] = bool(node.get_parameter('use_sim_time').value)
        if manifest['useSimTime']:
            raise ValueError('Physical capture rejects use_sim_time=true; recorder must use a wall/system ROS clock')
        subscriptions = []
        with gzip.open(records_path, 'wt', encoding='utf-8') as stream:
            for spec in topics:
                topic, message_type = spec['topic'], spec['messageType']
                cls = get_message(message_type)
                def receive(msg, topic=topic, message_type=message_type):
                    # Reception timestamps precede serialization, conversion and disk work.
                    ros_ns = node.get_clock().now().nanoseconds
                    mono_ns, wall_ns = time.monotonic_ns(), time.time_ns()
                    counts[topic] += 1
                    row = envelope(topic, message_type, counts[topic], message_to_ordereddict(msg),
                                   serialize_message(msg), ros_ns, mono_ns, wall_ns, args.source)
                    stream.write(json.dumps(row, allow_nan=False, separators=(',', ':')) + '\n')
                subscriptions.append(node.create_subscription(cls, topic, receive, qos_profile_sensor_data))
            manifest['status'] = 'capturing'
            deadline = time.monotonic() + args.seconds
            while running[0] and time.monotonic() < deadline and rclpy.ok():
                if node.get_parameter('use_sim_time').value:
                    raise ValueError('use_sim_time changed to true during physical capture')
                rclpy.spin_once(node, timeout_sec=min(.1, max(0, deadline-time.monotonic())))
            if not running[0]:
                termination = 'signal'
        manifest['status'] = 'captured' if all(counts.values()) else 'partial_missing_topics'
    except Exception as error:
        termination = 'error'
        manifest['status'] = 'dependency_failure' if isinstance(error, ImportError) else 'capture_failure'
        manifest['errors'].append({'type': type(error).__name__, 'message': str(error)})
        manifest['traceback'] = traceback.format_exc()
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy is not None and rclpy.ok():
            rclpy.shutdown()
        manifest.update(endUnixNanoseconds=time.time_ns(), termination=termination,
                        topicCounts=counts, missingTopics=[k for k, v in counts.items() if not v],
                        recorderSha256=file_hash(Path(__file__)),
                        recordsFile=records_path.name if records_path.exists() else None,
                        recordsSha256=file_hash(records_path) if records_path.exists() else None)
        manifest_path.write_text(json.dumps(manifest, indent=2, allow_nan=False) + '\n')
    print(json.dumps({'status': manifest['status'], 'manifest': str(manifest_path), 'topicCounts': counts}))
    return 0 if manifest['status'] == 'captured' else 2


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--seconds', type=float, default=120)
    parser.add_argument('--topic', action='append', required=True, help='/topic=package/msg/Type (repeatable)')
    parser.add_argument('--metadata', type=Path)
    parser.add_argument('--source', choices=('physical', 'synthetic-fixture'), default='physical')
    args, ros_args = parser.parse_known_args()
    return capture(args, ros_args)


if __name__ == '__main__':
    raise SystemExit(main())
