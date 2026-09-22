#!/usr/bin/env python3
"""Exercise Nav2 controller_server FollowPath against CRANE authoritative odometry.

This is deliberately a controller-level fixture: it does not start a planner, behavior tree,
localizer, or map server. It stamps returned commands with the newest odometry delivered to this
node; that is useful bounded-lag provenance, but is not proof of Nav2's internal sample choice.
"""

import argparse
import hashlib
import json
import math
from pathlib import Path as FilePath
import time

from bt_transition_capture import (
    BehaviorTreeTransitionCapture,
    direct_terminal_recovery_nodes_from_bt_xml,
)
import rclpy
from geometry_msgs.msg import PoseStamped, Twist, TwistStamped
from nav2_msgs.action import FollowPath, NavigateToPose
from nav2_msgs.msg import BehaviorTreeLog
from nav2_msgs.srv import GetCostmap
from nav_msgs.msg import OccupancyGrid, Odometry, Path
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import String


ACTION_STATUS = {
    2: 'executing',
    4: 'succeeded',
    5: 'canceled',
    6: 'aborted',
}

DEFAULT_RECOVERY_NODE_NAMES = (
    'BackUp',
    'ClearGlobalCostmap-Context',
    'ClearGlobalCostmap-Subtree',
    'ClearLocalCostmap-Context',
    'ClearLocalCostmap-Subtree',
    'Spin',
    'Wait',
)

class FollowPathFixture(Node):
    def __init__(self, args):
        super().__init__('crane_nav2_follow_path_fixture')
        self.args = args
        self.latest_odom = None
        self.initial_odom = None
        self.odom_count = 0
        self.command_count = 0
        self.maximum_linear_command = 0.0
        self.maximum_angular_command = 0.0
        self.output_count = 0
        self.costmap_count = 0
        self.costmap_service_snapshot_count = 0
        self.maximum_occupied_costmap_cells = 0
        self.costmap_request = None
        self.next_costmap_request_wall = 0.0
        self.first_command_wall = None
        self.started_wall = time.monotonic()
        self.goal_sent_wall = None
        self.goal_handle = None
        self.goal_attempts = 0
        self.next_goal_attempt_wall = 0.0
        self.result_status = None
        self.result_received_wall = None
        self.feedback_count = 0
        self.maximum_recovery_count = 0
        self.recovery_count_sequence = []
        self.bt_log_message_count = 0
        self.bt_transition_count = 0
        self.bt_transition_counts = {}
        self.bt_latest_transition_by_node = {}
        configured_recovery_nodes = args.bt_recovery_node or DEFAULT_RECOVERY_NODE_NAMES
        configured_direct_terminal_nodes = args.bt_direct_terminal_recovery_node
        direct_terminal_classifier_basis = "explicit_cli_allowlist"
        direct_terminal_classifier_sha256 = None
        if not configured_direct_terminal_nodes and args.bt_xml:
            bt_xml_path = FilePath(args.bt_xml)
            derived_nodes = direct_terminal_recovery_nodes_from_bt_xml(bt_xml_path)
            configured_direct_terminal_nodes = tuple(
                name for name in derived_nodes if name in configured_recovery_nodes
            )
            direct_terminal_classifier_basis = (
                "loaded_bt_xml_clear_entire_costmap_without_completion_preconditions"
            )
            direct_terminal_classifier_sha256 = hashlib.sha256(
                bt_xml_path.read_bytes()
            ).hexdigest()
        elif not configured_direct_terminal_nodes:
            direct_terminal_classifier_basis = "none_without_loaded_tree_provenance"
        self.bt_capture = BehaviorTreeTransitionCapture(
            max_transitions=args.bt_max_transitions,
            max_invocations=args.bt_max_invocations,
            recovery_node_names=configured_recovery_nodes,
            direct_terminal_recovery_node_names=configured_direct_terminal_nodes,
            direct_terminal_classifier_basis=direct_terminal_classifier_basis,
            direct_terminal_classifier_sha256=direct_terminal_classifier_sha256,
            terminal_node_names=args.bt_terminal_node or ('NavigateRecovery',),
        )
        self.trajectory_samples = []
        self.next_trajectory_sample_wall = self.started_wall
        self.done = False
        self.goal_description = None
        self.publisher = self.create_publisher(
            TwistStamped, args.output_topic, 10)
        harness_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=20,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.harness_publisher = self.create_publisher(
            String, args.harness_topic, harness_qos)
        self.create_subscription(Odometry, args.odom_topic, self.on_odom, 20)
        costmap_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            # Topic delivery is optional evidence beside the bounded GetCostmap snapshots.
            # Volatile durability matches the previously validated ROS-container path and avoids
            # relying on cross-container transient-local replay behavior.
            durability=DurabilityPolicy.VOLATILE,
        )
        self.create_subscription(OccupancyGrid, args.costmap_topic,
                                 self.on_costmap, costmap_qos)
        self.create_subscription(BehaviorTreeLog, args.bt_topic, self.on_bt_log, 10)
        self.costmap_client = self.create_client(GetCostmap, args.costmap_service)
        if args.input_type == 'stamped':
            self.create_subscription(
                TwistStamped, args.input_topic, self.on_stamped_command, 20)
        else:
            self.create_subscription(Twist, args.input_topic, self.on_command, 20)
        self.action_name = args.action_name or (
            '/navigate_to_pose' if args.action_mode == 'navigate-to-pose' else '/follow_path')
        action_type = NavigateToPose if args.action_mode == 'navigate-to-pose' else FollowPath
        self.action = ActionClient(self, action_type, self.action_name)
        self.timer = self.create_timer(0.05, self.tick)

    def on_odom(self, message):
        self.latest_odom = message
        self.odom_count += 1
        if self.initial_odom is None:
            self.initial_odom = message
            self.publish_identity('initial_observation')

    def publish_identity(self, reason):
        """Republish bounded identity facts so late DDS discovery does not erase provenance."""
        if self.initial_odom is None:
            return
        message = self.initial_odom
        self.publish_event({
            'type': 'crane_identity',
            'episode_id': self.args.episode_id,
            'run_id': self.args.run_id,
            'publication_reason': reason,
        })
        self.publish_event({
            'type': 'observation_identity',
            'observation': 'initial_odometry',
            'topic': self.args.odom_topic,
            'stamp': stamp_dict(message.header.stamp),
            'frame_id': message.header.frame_id,
            'child_frame_id': message.child_frame_id,
            'consumption_status': 'delivered_to_fixture_not_proven_consumed_by_nav2',
            'publication_reason': reason,
        })

    def publish_event(self, event):
        event['wall_time_ns'] = time.time_ns()
        message = String()
        message.data = json.dumps(event, sort_keys=True, separators=(',', ':'))
        self.harness_publisher.publish(message)

    def on_costmap(self, message):
        self.costmap_count += 1
        self.record_costmap(message.data)

    def on_bt_log(self, message):
        self.bt_log_message_count += 1
        ordered_events = []
        for event in message.event_log:
            self.bt_transition_count += 1
            key = f'{event.node_name}:{event.previous_status}->{event.current_status}'
            self.bt_transition_counts[key] = self.bt_transition_counts.get(key, 0) + 1
            self.bt_latest_transition_by_node[event.node_name] = {
                'uid': int(event.uid),
                'previousStatus': event.previous_status,
                'currentStatus': event.current_status,
                'eventStamp': stamp_dict(event.timestamp),
                'messageStamp': stamp_dict(message.timestamp),
            }
            ordered_events.append({
                'uid': int(event.uid),
                'nodeName': event.node_name,
                'previousStatus': event.previous_status,
                'currentStatus': event.current_status,
                'eventStamp': stamp_dict(event.timestamp),
            })
        self.bt_capture.record_message(stamp_dict(message.timestamp), ordered_events)

    def on_feedback(self, message):
        self.feedback_count += 1
        feedback = message.feedback
        recoveries = int(feedback.number_of_recoveries)
        self.maximum_recovery_count = max(self.maximum_recovery_count, recoveries)
        if not self.recovery_count_sequence or self.recovery_count_sequence[-1] != recoveries:
            self.recovery_count_sequence.append(recoveries)

    def record_costmap(self, data):
        occupied = sum(1 for value in data if value > 0)
        self.maximum_occupied_costmap_cells = max(
            self.maximum_occupied_costmap_cells, occupied)

    def request_costmap(self):
        if self.costmap_request is not None or not self.costmap_client.service_is_ready():
            return
        if time.monotonic() < self.next_costmap_request_wall:
            return
        self.next_costmap_request_wall = time.monotonic() + self.args.costmap_sample_period
        self.costmap_request = self.costmap_client.call_async(GetCostmap.Request())
        self.costmap_request.add_done_callback(self.on_costmap_service)

    def on_costmap_service(self, future):
        self.costmap_request = None
        try:
            response = future.result()
        except Exception as error:  # pragma: no cover - depends on live ROS service failure
            self.get_logger().warning(f'GetCostmap request failed: {error}')
            return
        if response is None:
            self.get_logger().warning('GetCostmap request returned no response')
            return
        self.costmap_service_snapshot_count += 1
        self.record_costmap(response.map.data)

    def forward(self, twist):
        # Ignore commands from an older/preempted action while this fixture is waiting to send.
        if self.goal_sent_wall is None:
            return
        self.command_count += 1
        self.maximum_linear_command = max(
            self.maximum_linear_command, abs(float(twist.linear.x)))
        self.maximum_angular_command = max(
            self.maximum_angular_command, abs(float(twist.angular.z)))
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
        self.sample_trajectory(elapsed)
        if self.done:
            return
        if (
                self.result_received_wall is not None
                and time.monotonic() - self.result_received_wall
                >= self.args.bt_terminal_drain_seconds):
            self.finish(self.result_status)
            return
        if elapsed >= self.args.duration:
            self.finish('timeout' if self.result_status is None else self.result_status)
            return
        self.request_costmap()
        if self.goal_handle is not None or self.initial_odom is None:
            return
        if time.monotonic() < self.next_goal_attempt_wall:
            return
        if not self.action.server_is_ready():
            return
        self.send_goal()

    def sample_trajectory(self, elapsed, force=False):
        if self.latest_odom is None:
            return
        now = time.monotonic()
        if not force and now < self.next_trajectory_sample_wall:
            return
        pose = self.latest_odom.pose.pose
        sample = {
            'wallSeconds': elapsed,
            'stamp': stamp_dict(self.latest_odom.header.stamp),
            'x': float(pose.position.x),
            'y': float(pose.position.y),
            'yaw': yaw_from_quaternion(pose.orientation),
        }
        if not self.trajectory_samples or (
                sample['stamp'] != self.trajectory_samples[-1]['stamp']):
            self.trajectory_samples.append(sample)
        self.next_trajectory_sample_wall = now + self.args.trajectory_sample_period

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
        if self.args.action_mode == 'navigate-to-pose':
            goal = NavigateToPose.Goal()
            goal.pose = path.poses[-1]
            self.goal_description = {
                'frame_id': goal.pose.header.frame_id,
                'stamp': stamp_dict(goal.pose.header.stamp),
                'position': {
                    'x': goal.pose.pose.position.x,
                    'y': goal.pose.pose.position.y,
                    'z': goal.pose.pose.position.z,
                },
            }
        else:
            goal = FollowPath.Goal()
            goal.path = path
            goal.controller_id = 'FollowPath'
            goal.goal_checker_id = 'goal_checker'
            if hasattr(goal, 'progress_checker_id'):
                goal.progress_checker_id = 'progress_checker'
        self.goal_sent_wall = time.monotonic()
        self.goal_attempts += 1
        self.bt_capture.mark_goal_sent()
        future = self.action.send_goal_async(goal, feedback_callback=self.on_feedback)
        future.add_done_callback(self.on_goal_response)

    def on_goal_response(self, future):
        self.goal_handle = future.result()
        if not self.goal_handle.accepted:
            # The action server is discoverable while controller_server is still transitioning
            # through its lifecycle. Retry this bounded startup condition instead of converting a
            # normal activation race into a fixture failure.
            self.goal_handle = None
            self.goal_sent_wall = None
            self.next_goal_attempt_wall = time.monotonic() + 0.5
            return
        # The harness topic is volatile. Repeat identity after action discovery so a capture node
        # that joined during fixture startup still receives the episode/observation boundary.
        self.publish_identity('accepted_goal_republication')
        goal_id = bytes(self.goal_handle.goal_id.uuid).hex()
        self.bt_capture.mark_goal_accepted(goal_id)
        self.publish_event({
            'type': 'navigate_to_pose_goal',
            'action_name': self.action_name,
            'action_mode': self.args.action_mode,
            'goal_id': goal_id,
            'goal_attempt': self.goal_attempts,
            'accepted': True,
            'goal': self.goal_description,
        })
        result = self.goal_handle.get_result_async()
        result.add_done_callback(self.on_result)

    def on_result(self, future):
        response = future.result()
        status_code = response.status
        self.result_status = ACTION_STATUS.get(status_code, f'action-status-{status_code}')
        self.result_received_wall = time.monotonic()
        payload = response.result
        self.publish_event({
            'type': 'navigate_to_pose_result',
            'action_name': self.action_name,
            'goal_id': bytes(self.goal_handle.goal_id.uuid).hex(),
            'status_code': status_code,
            'status': self.result_status,
            'error_code': getattr(payload, 'error_code', None),
            'error_msg': getattr(payload, 'error_msg', None),
        })
        # The action result and final BehaviorTreeLog transition travel on separate ROS topics.
        # Keep spinning for one small, bounded interval so executor callback ordering does not
        # systematically discard an already-published terminal BT record.

    def finish(self, status):
        if self.done:
            return
        self.done = True
        self.sample_trajectory(time.monotonic() - self.started_wall, force=True)
        if status == 'timeout' and self.goal_handle is not None:
            goal_id = bytes(self.goal_handle.goal_id.uuid).hex()
            self.publish_event({
                'type': 'client_deadline',
                'action_name': self.action_name,
                'goal_id': goal_id,
                'duration_s': self.args.duration,
            })
            self.publish_event({
                'type': 'client_cancel',
                'action_name': self.action_name,
                'goal_id': goal_id,
                'reason': 'fixture_deadline',
            })
            self.goal_handle.cancel_goal_async()
        final = self.latest_odom or self.initial_odom
        dx = dy = displacement = 0.0
        if self.initial_odom is not None and final is not None:
            dx = final.pose.pose.position.x - self.initial_odom.pose.pose.position.x
            dy = final.pose.pose.position.y - self.initial_odom.pose.pose.position.y
            displacement = math.hypot(dx, dy)
        summary = {
            'schema': 'crane-nav2-controller-fixture-v1',
            'scope': ('nav2-navigate-to-pose' if self.args.action_mode == 'navigate-to-pose'
                      else 'nav2-controller-server-follow-path'),
            'actionMode': self.args.action_mode,
            'actionName': self.action_name,
            'status': status,
            'odomTopic': self.args.odom_topic,
            'inputTopic': self.args.input_topic,
            'inputType': self.args.input_type,
            'outputTopic': self.args.output_topic,
            'odometryMessages': self.odom_count,
            'controllerCommands': self.command_count,
            'maximumLinearCommand': self.maximum_linear_command,
            'maximumAngularCommand': self.maximum_angular_command,
            'returnedCommands': self.output_count,
            'goalAttempts': self.goal_attempts,
            'navigateToPoseFeedbackMessages': self.feedback_count,
            'maximumRecoveryCount': self.maximum_recovery_count,
            'recoveryCountSequence': self.recovery_count_sequence,
            'behaviorTreeTopic': self.args.bt_topic,
            'behaviorTreeLogMessages': self.bt_log_message_count,
            'behaviorTreeTransitions': self.bt_transition_count,
            'behaviorTreeTransitionCounts': self.bt_transition_counts,
            'behaviorTreeLatestTransitionByNode': self.bt_latest_transition_by_node,
            'behaviorTreeCapture': self.bt_capture.summary(),
            'behaviorTreeTerminalDrainWallSeconds': self.args.bt_terminal_drain_seconds,
            'behaviorTreeProvenance': (
                'delivered-topic-transitions-may-omit-terminal-tick-not-proof-of-completeness'),
            'trajectorySamples': self.trajectory_samples,
            'trajectorySamplePeriodWallSeconds': self.args.trajectory_sample_period,
            'trajectoryProvenance': (
                'sampled-delivered-odometry-not-proven-nav2-internal-state'),
            'costmapTopic': self.args.costmap_topic,
            'costmapMessages': self.costmap_count,
            'costmapService': self.args.costmap_service,
            'costmapServiceSnapshots': self.costmap_service_snapshot_count,
            'costmapObservations': self.costmap_count + self.costmap_service_snapshot_count,
            'maximumOccupiedCostmapCells': self.maximum_occupied_costmap_cells,
            'goalToFirstCommandWallSeconds': (
                self.first_command_wall - self.goal_sent_wall
                if self.first_command_wall is not None and self.goal_sent_wall is not None
                else None),
            'wallSeconds': time.monotonic() - self.started_wall,
            'displacementMeters': displacement,
            'deltaX': dx,
            'deltaY': dy,
            'provenance': 'latest-delivered-odometry-not-proven-internal-consumption',
            'costmapProvenance': (
                'nav2-get-costmap-snapshot-not-proven-controller-consumption'),
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


def stamp_dict(value):
    return {'sec': int(value.sec), 'nanosec': int(value.nanosec)}


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--odom-topic', default='/crane/odom')
    parser.add_argument('--input-topic', default='/nav2/cmd_vel')
    parser.add_argument('--costmap-topic', default='/local_costmap/costmap')
    parser.add_argument('--costmap-service', default='/local_costmap/get_costmap')
    parser.add_argument('--costmap-sample-period', type=float, default=0.5)
    parser.add_argument('--input-type', choices=('twist', 'stamped'), default='stamped')
    parser.add_argument('--output-topic', default='/crane/cmd_vel_stamped')
    parser.add_argument('--action-mode', choices=('follow-path', 'navigate-to-pose'),
                        default='follow-path')
    parser.add_argument('--action-name')
    parser.add_argument('--distance', type=float, default=0.5)
    parser.add_argument('--path-points', type=int, default=20)
    parser.add_argument('--duration', type=float, default=25.0)
    parser.add_argument('--trajectory-sample-period', type=float, default=1.0)
    parser.add_argument('--output')
    parser.add_argument('--harness-topic', default='/crane/explanation_event')
    parser.add_argument('--bt-topic', default='/behavior_tree_log')
    parser.add_argument('--bt-max-transitions', type=int, default=4096)
    parser.add_argument('--bt-max-invocations', type=int, default=1024)
    parser.add_argument('--bt-recovery-node', action='append', default=[])
    parser.add_argument('--bt-direct-terminal-recovery-node', action='append', default=[])
    parser.add_argument('--bt-xml')
    parser.add_argument('--bt-terminal-node', action='append', default=[])
    parser.add_argument('--bt-terminal-drain-seconds', type=float, default=0.5)
    parser.add_argument('--episode-id', required=True)
    parser.add_argument('--run-id', required=True)
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
