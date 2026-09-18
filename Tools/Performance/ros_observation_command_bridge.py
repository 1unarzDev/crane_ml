#!/usr/bin/env python3
"""Stamp returned velocity commands with the observation that produced/preceded them.

Modes:
  feedback: emit a bounded deterministic command for every Detection3DArray observation.
  nav2: pair each Nav2 Twist with the newest received detection stamp.

The feedback mode is a transport/causality fixture, not a navigation algorithm. The nav2 mode
does not claim Nav2 consumed that exact observation; it records the newest observation delivered
to this bridge when Nav2's command arrived, which is the strongest provenance available from the
standard geometry_msgs/Twist contract.
"""

import argparse
import json
import time

import rclpy
from geometry_msgs.msg import Twist, TwistStamped
from rclpy.node import Node
from vision_msgs.msg import Detection3DArray


class ObservationCommandBridge(Node):
    def __init__(self, args):
        super().__init__('crane_observation_command_bridge')
        self.args = args
        self.latest_stamp = None
        self.observations = 0
        self.inputs = 0
        self.outputs = 0
        self.started = time.monotonic()
        self.publisher = self.create_publisher(TwistStamped, args.output_topic, 10)
        self.create_subscription(Detection3DArray, args.observation_topic,
                                 self.on_observation, 10)
        if args.mode == 'nav2':
            self.create_subscription(Twist, args.input_topic, self.on_nav2_command, 10)
        self.timer = self.create_timer(0.25, self.maybe_finish)

    def on_observation(self, message):
        self.observations += 1
        self.latest_stamp = message.header.stamp
        if self.args.mode != 'feedback':
            return
        command = TwistStamped()
        command.header.stamp = message.header.stamp
        command.header.frame_id = message.header.frame_id
        command.twist.linear.x = self.args.forward
        command.twist.angular.z = self.args.yaw
        self.publisher.publish(command)
        self.outputs += 1

    def on_nav2_command(self, message):
        self.inputs += 1
        if self.latest_stamp is None:
            return
        command = TwistStamped()
        command.header.stamp = self.latest_stamp
        command.header.frame_id = 'base_link'
        command.twist = message
        self.publisher.publish(command)
        self.outputs += 1

    def maybe_finish(self):
        if time.monotonic() - self.started < self.args.duration:
            return
        summary = {
            'schema': 'crane-ros-observation-command-bridge-v1',
            'mode': self.args.mode,
            'observationTopic': self.args.observation_topic,
            'inputTopic': self.args.input_topic if self.args.mode == 'nav2' else None,
            'outputTopic': self.args.output_topic,
            'observations': self.observations,
            'inputCommands': self.inputs,
            'outputCommands': self.outputs,
            'durationSeconds': time.monotonic() - self.started,
        }
        print(json.dumps(summary, sort_keys=True), flush=True)
        rclpy.shutdown()


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--mode', choices=('feedback', 'nav2'), default='feedback')
    parser.add_argument('--observation-topic', default='/detections')
    parser.add_argument('--input-topic', default='/cmd_vel')
    parser.add_argument('--output-topic', default='/crane/cmd_vel_stamped')
    parser.add_argument('--forward', type=float, default=0.15)
    parser.add_argument('--yaw', type=float, default=0.05)
    parser.add_argument('--duration', type=float, default=12.0)
    args = parser.parse_args()
    rclpy.init()
    node = ObservationCommandBridge(args)
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
