#!/usr/bin/env python3
"""Exercise Nav2 controller_server FollowPath against CRANE authoritative odometry.

This is deliberately a controller-level fixture: it does not start a planner, behavior tree,
localizer, or map server. It stamps returned commands with the newest odometry delivered to this
node; that is useful bounded-lag provenance, but is not proof of Nav2's internal sample choice.
"""

import argparse
import json
import math
import time

import rclpy
from geometry_msgs.msg import PoseStamped, Twist, TwistStamped
from nav2_msgs.action import FollowPath
from nav_msgs.msg import Odometry, Path
from rclpy.action import ActionClient
from rclpy.node import Node


ACTION_STATUS = {
    2: 'executing',
    4: 'succeeded',
    5: 'canceled',
    6: 'aborted',
}


class FollowPathFixture(Node):
    def __init__(self, args):
        super().__init__('crane_nav2_follow_path_fixture')
        self.args = args
        self.latest_odom = None
        self.initial_odom = None
        self.odom_count = 0
        self.command_count = 0
        self.output_count = 0
        self.first_command_wall = None
        self.started_wall = time.monotonic()
        self.goal_sent_wall = None
        self.goal_handle = None
        self.result_status = None
        self.done = False
        self.publisher = self.create_publisher(
            TwistStamped, args.output_topic, 10)
        self.create_subscription(Odometry, args.odom_topic, self.on_odom, 20)
        if args.input_type == 'stamped':
            self.create_subscription(
                TwistStamped, args.input_topic, self.on_stamped_command, 20)
        else:
            self.create_subscription(Twist, args.input_topic, self.on_command, 20)
        self.action = ActionClient(self, FollowPath, args.action_name)
        self.timer = self.create_timer(0.05, self.tick)

    def on_odom(self, message):
        self.latest_odom = message
        self.odom_count += 1
        if self.initial_odom is None:
            self.initial_odom = message

    def forward(self, twist):
        # Ignore commands from an older/preempted action while this fixture is waiting to send.
        if self.goal_sent_wall is None:
            return
        self.command_count += 1
        if self.first_command_wall is None:
            self.first_command_wall = time.monotonic()
        if self.latest_odom is None:
            return
        output = TwistStamped()
        output.header.stamp = self.latest_odom.header.stamp
        output.header.frame_id = self.latest_odom.child_frame_id
        output.twist = twist
        self.publisher.publish(output)
        self.output_count += 1

    def on_command(self, message):
        self.forward(message)

    def on_stamped_command(self, message):
        self.forward(message.twist)

    def tick(self):
        elapsed = time.monotonic() - self.started_wall
        if self.done or elapsed >= self.args.duration:
            self.finish('timeout' if self.result_status is None else self.result_status)
            return
        if self.goal_handle is not None or self.initial_odom is None:
            return
        if not self.action.server_is_ready():
            return
        self.send_goal()

    def send_goal(self):
        odom = self.initial_odom
        yaw = yaw_from_quaternion(odom.pose.pose.orientation)
        path = Path()
        path.header.frame_id = odom.header.frame_id
        path.header.stamp = odom.header.stamp
        for index in range(1, self.args.path_points + 1):
            distance = self.args.distance * index / self.args.path_points
            pose = PoseStamped()
            pose.header = path.header
            pose.pose.position.x = odom.pose.pose.position.x + math.cos(yaw) * distance
            pose.pose.position.y = odom.pose.pose.position.y + math.sin(yaw) * distance
            pose.pose.position.z = odom.pose.pose.position.z
            pose.pose.orientation = odom.pose.pose.orientation
            path.poses.append(pose)
        goal = FollowPath.Goal()
        goal.path = path
        goal.controller_id = 'FollowPath'
        goal.goal_checker_id = 'goal_checker'
        self.goal_sent_wall = time.monotonic()
        future = self.action.send_goal_async(goal)
        future.add_done_callback(self.on_goal_response)

    def on_goal_response(self, future):
        self.goal_handle = future.result()
        if not self.goal_handle.accepted:
            self.finish('rejected')
            return
        result = self.goal_handle.get_result_async()
        result.add_done_callback(self.on_result)

    def on_result(self, future):
        status_code = future.result().status
        self.result_status = ACTION_STATUS.get(status_code, f'action-status-{status_code}')
        self.finish(self.result_status)

    def finish(self, status):
        if self.done:
            return
        self.done = True
        if status == 'timeout' and self.goal_handle is not None:
            self.goal_handle.cancel_goal_async()
        final = self.latest_odom or self.initial_odom
        dx = dy = displacement = 0.0
        if self.initial_odom is not None and final is not None:
            dx = final.pose.pose.position.x - self.initial_odom.pose.pose.position.x
            dy = final.pose.pose.position.y - self.initial_odom.pose.pose.position.y
            displacement = math.hypot(dx, dy)
        summary = {
            'schema': 'crane-nav2-controller-fixture-v1',
            'scope': 'nav2-controller-server-follow-path',
            'status': status,
            'odomTopic': self.args.odom_topic,
            'inputTopic': self.args.input_topic,
            'inputType': self.args.input_type,
            'outputTopic': self.args.output_topic,
            'odometryMessages': self.odom_count,
            'controllerCommands': self.command_count,
            'returnedCommands': self.output_count,
            'goalToFirstCommandWallSeconds': (
                self.first_command_wall - self.goal_sent_wall
                if self.first_command_wall is not None and self.goal_sent_wall is not None
                else None),
            'wallSeconds': time.monotonic() - self.started_wall,
            'displacementMeters': displacement,
            'deltaX': dx,
            'deltaY': dy,
            'provenance': 'latest-delivered-odometry-not-proven-internal-consumption',
        }
        if self.args.output:
            with open(self.args.output, 'w', encoding='utf-8') as stream:
                json.dump(summary, stream, indent=2, sort_keys=True)
                stream.write('\n')
        print(json.dumps(summary, sort_keys=True), flush=True)
        rclpy.shutdown()


def yaw_from_quaternion(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                      1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--odom-topic', default='/crane/odom')
    parser.add_argument('--input-topic', default='/nav2/cmd_vel')
    parser.add_argument('--input-type', choices=('twist', 'stamped'), default='stamped')
    parser.add_argument('--output-topic', default='/crane/cmd_vel_stamped')
    parser.add_argument('--action-name', default='/follow_path')
    parser.add_argument('--distance', type=float, default=0.5)
    parser.add_argument('--path-points', type=int, default=20)
    parser.add_argument('--duration', type=float, default=25.0)
    parser.add_argument('--output')
    args = parser.parse_args()
    rclpy.init()
    node = FollowPathFixture(args)
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
