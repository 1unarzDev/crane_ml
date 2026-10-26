#!/usr/bin/env python3
"""Disposable recorder acceptance test. All messages are explicitly synthetic."""
import gzip
import hashlib
import json
from pathlib import Path
import subprocess
import sys
import time
import signal

import rclpy
from rclpy.serialization import deserialize_message
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan, Imu, Image, PointCloud2
from rosgraph_msgs.msg import Clock


def main():
    output = Path(sys.argv[1]); output.mkdir(parents=True, exist_ok=True)
    capture = output / 'synthetic-loopback'
    topic_types = [('/fidelity/odom', 'nav_msgs/msg/Odometry', Odometry),
                   ('/fidelity/cmd_vel', 'geometry_msgs/msg/Twist', Twist),
                   ('/fidelity/scan', 'sensor_msgs/msg/LaserScan', LaserScan),
                   ('/fidelity/points', 'sensor_msgs/msg/PointCloud2', PointCloud2),
                   ('/fidelity/imu', 'sensor_msgs/msg/Imu', Imu),
                   ('/fidelity/image', 'sensor_msgs/msg/Image', Image),
                   ('/fidelity/clock', 'rosgraph_msgs/msg/Clock', Clock)]
    command = [sys.executable, str(Path(__file__).with_name('physical_ros_capture.py')),
               '--output', str(capture), '--seconds', '4', '--source', 'synthetic-fixture']
    for topic, message_type, _ in topic_types:
        command += ['--topic', topic + '=' + message_type]
    command += ['--topic', '/fidelity/missing=std_msgs/msg/String']
    recorder = subprocess.Popen(command)
    rclpy.init(); node = rclpy.create_node('crane_fidelity_synthetic_publishers')
    publishers = [(topic, cls, node.create_publisher(cls, topic, 10)) for topic, _, cls in topic_types]
    start = time.monotonic(); sent = 0
    try:
        while recorder.poll() is None and time.monotonic()-start < 8:
            sent += 1
            for topic, cls, publisher in publishers:
                msg = cls()
                if hasattr(msg, 'header'):
                    msg.header.stamp.sec = 100
                    msg.header.stamp.nanosec = sent * 1000000
                    msg.header.frame_id = 'synthetic_fixture'
                if cls is Odometry:
                    msg.pose.pose.position.x = sent * .01
                    msg.pose.pose.orientation.w = 1.
                if cls is LaserScan:
                    msg.ranges = [1., float('nan'), 2.]
                if cls is Image:
                    msg.width = 2; msg.height = 1; msg.encoding = 'mono8'; msg.step = 2; msg.data = [sent % 256, 42]
                if cls is PointCloud2:
                    msg.height = 1; msg.width = 1; msg.point_step = 0; msg.row_step = 0; msg.data = []
                if cls is Clock:
                    msg.clock.sec = 100
                publisher.publish(msg)
            rclpy.spin_once(node, timeout_sec=.03)
            time.sleep(.02)
        code = recorder.wait(timeout=5)
    finally:
        if recorder.poll() is None:
            recorder.terminate(); recorder.wait(timeout=5)
        node.destroy_node(); rclpy.shutdown()
    manifest = json.loads((capture / 'manifest.json').read_text())
    with gzip.open(capture / 'records.jsonl.gz', 'rt') as stream:
        rows = [json.loads(line) for line in stream]
    checks = {'allConfiguredTopicsReceived': all(manifest['topicCounts'][t] > 0 for t, _, _ in topic_types),
              'missingTopicReported': manifest['missingTopics'] == ['/fidelity/missing'],
              'partialExitCode': code == 2 and manifest['status'] == 'partial_missing_topics',
              'sourceExplicitlySynthetic': manifest['dataSource'] == 'synthetic-fixture' and not manifest['physicalRobotEvidence'],
              'hashMatches': hashlib.sha256((capture / 'records.jsonl.gz').read_bytes()).hexdigest() == manifest['recordsSha256'],
              'receptionMonotonic': all(a['receptionMonotonicNanoseconds'] <= b['receptionMonotonicNanoseconds'] for a, b in zip(rows, rows[1:])),
              'rawOdomDeserializes': False,
              'acquisitionIndependentFromReception': any(x['acquisitionStamp'] and x['acquisitionStamp']['sec'] == 100 and x['receptionRosNanoseconds'] > 1000000000000 for x in rows)}
    import base64
    odom = next(x for x in rows if x['messageType'] == 'nav_msgs/msg/Odometry')
    restored = deserialize_message(base64.b64decode(odom['cdrBase64']), Odometry)
    checks['rawOdomDeserializes'] = restored.pose.pose.position.x == odom['message']['pose']['pose']['position']['x']
    rejected = output / 'synthetic-simtime-rejection'
    rejection_command = command[:]
    rejection_command[rejection_command.index('--output')+1] = str(rejected)
    rejection_command += ['--ros-args', '-p', 'use_sim_time:=true']
    rejection_code = subprocess.run(rejection_command, check=False).returncode
    rejection = json.loads((rejected / 'manifest.json').read_text())
    checks['simTimeRejectedWithManifest'] = rejection_code == 2 and rejection['status'] == 'capture_failure' and rejection['useSimTime'] is True
    signal_dir = output / 'synthetic-signal-termination'
    signal_command = [sys.executable, str(Path(__file__).with_name('physical_ros_capture.py')),
                      '--output', str(signal_dir), '--seconds', '30', '--source', 'synthetic-fixture',
                      '--topic', '/fidelity/absent=std_msgs/msg/String']
    signal_process = subprocess.Popen(signal_command)
    time.sleep(.5)
    signal_process.send_signal(signal.SIGTERM)
    signal_code = signal_process.wait(timeout=5)
    signal_manifest = json.loads((signal_dir / 'manifest.json').read_text())
    checks['signalTerminationWritesManifest'] = signal_code == 2 and signal_manifest['termination'] == 'signal' and signal_manifest['missingTopics'] == ['/fidelity/absent']
    report = {'schema': 'crane-passive-capture-loopback-v1', 'dataSource': 'synthetic-fixture',
              'physicalRobotEvidence': False, 'checks': checks, 'valid': all(checks.values()),
              'recordCount': len(rows), 'publisherRounds': sent,
              'boundary': 'ROS 2 Humble Docker transport/serialization fixture only; no hardware collection, sensor validity, device synchronization, or sim-to-real validation.'}
    (output / 'passive-capture-loopback-result.json').write_text(json.dumps(report, indent=2)+'\n')
    print(json.dumps(report, indent=2))
    return 0 if report['valid'] else 2


if __name__ == '__main__':
    raise SystemExit(main())
