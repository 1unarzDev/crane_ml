#!/usr/bin/env python3
"""Characterize the RoboBoat body response at the existing TwistStamped boundary."""

import argparse
import csv
import json
import math
import time

import rclpy
from geometry_msgs.msg import TwistStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node


FULL_PHASES = (
    ('settle', 2.0, 0.0, 0.0, 0.0),
    ('surge_positive', 3.0, 0.2, 0.0, 0.0),
    ('surge_positive_stop', 4.0, 0.0, 0.0, 0.0),
    ('surge_negative', 3.0, -0.2, 0.0, 0.0),
    ('surge_negative_stop', 4.0, 0.0, 0.0, 0.0),
    ('sway_positive', 3.0, 0.0, 0.2, 0.0),
    ('sway_positive_stop', 4.0, 0.0, 0.0, 0.0),
    ('sway_negative', 3.0, 0.0, -0.2, 0.0),
    ('sway_negative_stop', 4.0, 0.0, 0.0, 0.0),
    ('yaw_positive', 3.0, 0.0, 0.0, 0.2),
    ('yaw_positive_stop', 4.0, 0.0, 0.0, 0.0),
    ('yaw_negative', 3.0, 0.0, 0.0, -0.2),
    ('yaw_negative_stop', 4.0, 0.0, 0.0, 0.0),
    ('combined_positive', 3.0, 0.15, 0.15, 0.15),
    ('combined_positive_stop', 5.0, 0.0, 0.0, 0.0),
)

REGRESSION_PHASES = (
    ('settle', 1.0, 0.0, 0.0, 0.0),
    ('surge_positive', 1.5, 0.2, 0.0, 0.0),
    ('surge_positive_stop', 1.0, 0.0, 0.0, 0.0),
    ('sway_positive', 1.5, 0.0, 0.2, 0.0),
    ('sway_positive_stop', 1.0, 0.0, 0.0, 0.0),
    ('yaw_positive', 1.5, 0.0, 0.0, 0.2),
    ('yaw_positive_stop', 1.0, 0.0, 0.0, 0.0),
)

YAW_SWEEP_PHASES = (
    ('settle', 1.0, 0.0, 0.0, 0.0),
    ('yaw_0025', 2.0, 0.0, 0.0, 0.025),
    ('yaw_0025_stop', 2.0, 0.0, 0.0, 0.0),
    ('yaw_0050', 2.0, 0.0, 0.0, 0.05),
    ('yaw_0050_stop', 2.0, 0.0, 0.0, 0.0),
    ('yaw_0100', 2.0, 0.0, 0.0, 0.1),
    ('yaw_0100_stop', 2.0, 0.0, 0.0, 0.0),
    ('yaw_0200', 2.0, 0.0, 0.0, 0.2),
    ('yaw_0200_stop', 2.0, 0.0, 0.0, 0.0),
    ('yaw_0400', 2.0, 0.0, 0.0, 0.4),
    ('yaw_0400_stop', 3.0, 0.0, 0.0, 0.0),
)

FULL_FORWARD_PHASES = (
    ('settle', 2.0, 0.0, 0.0, 0.0),
    ('full_forward', 15.0, 1.0, 0.0, 0.0),
    ('full_forward_stop', 15.0, 0.0, 0.0, 0.0),
)


def yaw_from_quaternion(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                      1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def stamp_seconds(stamp):
    return float(stamp.sec) + float(stamp.nanosec) * 1e-9


class CommandResponseFixture(Node):
    def __init__(self, args):
        super().__init__('crane_roboboat_command_response_fixture')
        self.args = args
        self.phases = {
            'full': FULL_PHASES,
            'regression': REGRESSION_PHASES,
            'yaw-sweep': YAW_SWEEP_PHASES,
            'full-forward': FULL_FORWARD_PHASES,
        }[args.suite]
        self.publisher = self.create_publisher(TwistStamped, args.command_topic, 10)
        self.create_subscription(Odometry, args.odom_topic, self.on_odom, 20)
        self.latest_odom = None
        self.latest_sim_seconds = None
        self.phase_index = 0
        self.phase_started_sim_seconds = None
        self.started = time.monotonic()
        self.samples = []
        self.done = False
        self.timer = self.create_timer(0.05, self.tick)

    def on_odom(self, message):
        self.latest_odom = message
        self.latest_sim_seconds = stamp_seconds(message.header.stamp)
        if self.phase_started_sim_seconds is None:
            return
        phase = self.phases[self.phase_index]
        pose = message.pose.pose
        twist = message.twist.twist
        self.samples.append({
            'wall_seconds': time.monotonic() - self.started,
            'sim_seconds': self.latest_sim_seconds,
            'phase': phase[0],
            'command_surge': phase[2],
            'command_sway': phase[3],
            'command_yaw': phase[4],
            'pose_x': float(pose.position.x),
            'pose_y': float(pose.position.y),
            'pose_yaw': yaw_from_quaternion(pose.orientation),
            'body_surge': float(twist.linear.x),
            'body_sway': float(twist.linear.y),
            'body_yaw_rate': float(twist.angular.z),
        })

    def publish(self, surge, sway, yaw):
        if self.latest_odom is None:
            return
        message = TwistStamped()
        message.header.stamp = self.latest_odom.header.stamp
        message.header.frame_id = self.latest_odom.child_frame_id
        message.twist.linear.x = surge
        message.twist.linear.y = sway
        message.twist.angular.z = yaw
        self.publisher.publish(message)

    def tick(self):
        if self.done:
            return
        if time.monotonic() - self.started > self.args.timeout:
            self.finish('timeout')
            return
        if self.latest_odom is None:
            return
        if self.phase_started_sim_seconds is None:
            self.phase_started_sim_seconds = self.latest_sim_seconds
        phase = self.phases[self.phase_index]
        self.publish(phase[2], phase[3], phase[4])
        if self.latest_sim_seconds - self.phase_started_sim_seconds < phase[1]:
            return
        self.phase_index += 1
        self.phase_started_sim_seconds = self.latest_sim_seconds
        if self.phase_index >= len(self.phases):
            self.publish(0.0, 0.0, 0.0)
            self.finish('completed')

    def finish(self, status):
        if self.done:
            return
        self.done = True
        self.write_outputs(status)
        rclpy.shutdown()

    def write_outputs(self, status):
        with open(self.args.samples, 'w', encoding='utf-8', newline='') as stream:
            writer = csv.DictWriter(stream, fieldnames=self.samples[0].keys() if self.samples else (
                'wall_seconds', 'sim_seconds', 'phase', 'command_surge', 'command_sway', 'command_yaw',
                'pose_x', 'pose_y', 'pose_yaw', 'body_surge', 'body_sway', 'body_yaw_rate'))
            writer.writeheader()
            writer.writerows(self.samples)

        phase_summaries = []
        for name, duration, surge, sway, yaw in self.phases:
            rows = [sample for sample in self.samples if sample['phase'] == name]
            tail = rows[-max(1, min(len(rows), 20)):] if rows else []
            summary = {
                'name': name,
                'durationSeconds': duration,
                'command': {'surge': surge, 'sway': sway, 'yaw': yaw},
                'sampleCount': len(rows),
            }
            for field in ('body_surge', 'body_sway', 'body_yaw_rate'):
                values = [row[field] for row in rows]
                tail_values = [row[field] for row in tail]
                summary[field] = {
                    'meanTail': sum(tail_values) / len(tail_values) if tail_values else None,
                    'minimum': min(values) if values else None,
                    'maximum': max(values) if values else None,
                }
            if rows:
                summary['deltaPose'] = {
                    'x': rows[-1]['pose_x'] - rows[0]['pose_x'],
                    'y': rows[-1]['pose_y'] - rows[0]['pose_y'],
                    'yaw': math.remainder(rows[-1]['pose_yaw'] - rows[0]['pose_yaw'],
                                          2.0 * math.pi),
                }
            phase_summaries.append(summary)

        result = {
            'schema': 'crane-roboboat-command-response-v1',
            'status': status,
            'suite': self.args.suite,
            'commandTopic': self.args.command_topic,
            'odometryTopic': self.args.odom_topic,
            'sampleCount': len(self.samples),
            'phases': phase_summaries,
        }
        with open(self.args.output, 'w', encoding='utf-8') as stream:
            json.dump(result, stream, indent=2, sort_keys=True)
            stream.write('\n')
        print(json.dumps(result, sort_keys=True), flush=True)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--command-topic', default='/crane/cmd_vel_stamped')
    parser.add_argument('--odom-topic', default='/crane/odom')
    parser.add_argument('--timeout', type=float, default=65.0)
    parser.add_argument('--suite',
                        choices=('full', 'regression', 'yaw-sweep', 'full-forward'),
                        default='full')
    parser.add_argument('--samples', required=True)
    parser.add_argument('--output', required=True)
    args = parser.parse_args()
    rclpy.init()
    node = CommandResponseFixture(args)
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
