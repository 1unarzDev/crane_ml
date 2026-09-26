#!/usr/bin/env python3
"""Exercise Nav2 controller_server FollowPath against CRANE authoritative odometry.

This is deliberately a controller-level fixture: it does not start a planner, behavior tree,
localizer, or map server. It stamps returned commands with the newest odometry delivered to this
node; that is useful bounded-lag provenance, but is not proof of Nav2's internal sample choice.
"""

import argparse
import base64
import hashlib
import json
import math
from pathlib import Path as FilePath
import time
import zlib

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

from nav2_path_spec import load_path_spec


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


def feedback_recovery_count(feedback):
    """Return a recovery count only for action feedback schemas that define one."""
    value = getattr(feedback, 'number_of_recoveries', None)
    return None if value is None else int(value)

class FollowPathFixture(Node):
    def __init__(self, args):
        super().__init__('crane_nav2_follow_path_fixture')
        self.args = args
        self.supplied_path = load_path_spec(args.path_file) if args.path_file else None
        self.latest_odom = None
        self.initial_odom = None
        self.odom_count = 0
        self.command_count = 0
        self.maximum_linear_command = 0.0
        self.maximum_lateral_command = 0.0
        self.maximum_angular_command = 0.0
        self.output_count = 0
        self.costmap_count = 0
        self.costmap_service_snapshot_count = 0
        self.maximum_occupied_costmap_cells = 0
        self.latest_costmap_snapshot = None
        self.docking_evaluations = []
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
        self.action_result_pose = None
        self.feedback_count = 0
        self.recovery_count_feedback_available = False
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
        self.planned_path = []
        self.plan_history = []
        self.trajectory = []
        self.latest_command = {'surge': 0.0, 'sway': 0.0, 'yaw': 0.0}
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
        self.create_subscription(Path, args.plan_topic, self.on_plan, 20)
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
        self.create_subscription(String, args.docking_evaluator_topic,
                                 self.on_docking_evaluation, 20)
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
        if self.goal_sent_wall is not None:
            self.trajectory.append(self.trajectory_sample(message))

    def on_plan(self, message):
        if not message.poses:
            return
        first = message.poses[0].pose
        terminal = message.poses[-1].pose
        plan_points = [{
            'x': float(item.pose.position.x),
            'y': float(item.pose.position.y),
            'yaw': yaw_from_quaternion(item.pose.orientation),
        } for item in message.poses]
        plan_summary = summarize_delivered_plan(
            plan_points,
            self.initial_odom.pose.pose.position if self.initial_odom is not None else None,
            self.goal_description.get('position') if self.goal_description else None,
        )
        terminal_tangent = None
        if len(message.poses) >= 2:
            previous = message.poses[-2].pose.position
            dx = terminal.position.x - previous.x
            dy = terminal.position.y - previous.y
            if math.hypot(dx, dy) > 1e-6:
                terminal_tangent = math.atan2(dy, dx)
        self.plan_history.append({
            'wallSeconds': time.monotonic() - self.started_wall,
            'stamp': stamp_dict(message.header.stamp),
            'frameId': message.header.frame_id,
            'poseCount': len(message.poses),
            **plan_summary,
            'poses': plan_points,
            'first': {
                'x': float(first.position.x),
                'y': float(first.position.y),
                'yaw': yaw_from_quaternion(first.orientation),
            },
            'terminal': {
                'x': float(terminal.position.x),
                'y': float(terminal.position.y),
                'yaw': yaw_from_quaternion(terminal.orientation),
                'incomingTangentYaw': terminal_tangent,
            },
        })

    def trajectory_sample(self, message):
        pose = message.pose.pose
        twist = message.twist.twist
        return {
            'phase': ('action' if self.result_received_wall is None else 'post_result'),
            'simSeconds': stamp_seconds(message.header.stamp),
            'wallSeconds': time.monotonic() - self.started_wall,
            'x': float(pose.position.x),
            'y': float(pose.position.y),
            'yaw': yaw_from_quaternion(pose.orientation),
            'bodySurge': float(twist.linear.x),
            'bodySway': float(twist.linear.y),
            'bodyYawRate': float(twist.angular.z),
            'commandSurge': self.latest_command['surge'],
            'commandSway': self.latest_command['sway'],
            'commandYaw': self.latest_command['yaw'],
        }

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

    def on_docking_evaluation(self, message):
        try:
            evaluation = json.loads(message.data)
        except (TypeError, ValueError) as error:
            self.get_logger().warning(f'Invalid docking evaluation JSON: {error}')
            return
        if evaluation.get('schema') != 'crane-roboboat-docking-evaluation-v1':
            return
        self.docking_evaluations.append(evaluation)

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
        recoveries = feedback_recovery_count(feedback)
        if recoveries is None:
            return
        self.recovery_count_feedback_available = True
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
        self.latest_costmap_snapshot = encode_costmap_snapshot(response.map)

    def forward(self, twist):
        # Ignore commands from an older/preempted action while this fixture is waiting to send.
        if self.goal_sent_wall is None:
            return
        self.command_count += 1
        self.maximum_linear_command = max(
            self.maximum_linear_command, abs(float(twist.linear.x)))
        self.maximum_lateral_command = max(
            self.maximum_lateral_command, abs(float(twist.linear.y)))
        self.maximum_angular_command = max(
            self.maximum_angular_command, abs(float(twist.angular.z)))
        self.latest_command = {
            'surge': float(twist.linear.x),
            'sway': float(twist.linear.y),
            'yaw': float(twist.angular.z),
        }
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
                >= max(self.args.post_result_seconds,
                       self.args.bt_terminal_drain_seconds)):
            self.finish(self.result_status)
            return
        if elapsed >= self.args.duration:
            self.finish('timeout' if self.result_status is None else self.result_status)
            return
        self.request_costmap()
        if self.result_received_wall is not None:
            return
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
        path_yaw = wrapped_angle(yaw + self.args.path_heading_offset)
        path = Path()
        path.header.frame_id = (self.supplied_path['frameId']
                                if self.supplied_path else odom.header.frame_id)
        path.header.stamp = odom.header.stamp
        if path.header.frame_id != odom.header.frame_id:
            raise ValueError(
                f"supplied path frame {path.header.frame_id} does not match odometry "
                f"frame {odom.header.frame_id}")
        if self.supplied_path:
            samples = self.supplied_path['poses']
        else:
            samples = []
            for index in range(1, self.args.path_points + 1):
                fraction = index / self.args.path_points
                forward, lateral, tangent = path_sample(
                    self.args.path_shape, self.args.distance, fraction,
                    self.args.path_lateral_amplitude, self.args.path_turn_angle)
                samples.append({
                    'x': (odom.pose.pose.position.x +
                          math.cos(path_yaw) * forward - math.sin(path_yaw) * lateral),
                    'y': (odom.pose.pose.position.y +
                          math.sin(path_yaw) * forward + math.cos(path_yaw) * lateral),
                    'yaw': wrapped_angle(path_yaw + tangent),
                })
        for sample in samples:
            pose = PoseStamped()
            pose.header = path.header
            pose.pose.position.x = sample['x']
            pose.pose.position.y = sample['y']
            pose.pose.position.z = odom.pose.pose.position.z
            pose.pose.orientation.z = math.sin(sample['yaw'] / 2.0)
            pose.pose.orientation.w = math.cos(sample['yaw'] / 2.0)
            path.poses.append(pose)
        self.planned_path = [{
            'x': float(odom.pose.pose.position.x),
            'y': float(odom.pose.pose.position.y),
            'yaw': path_yaw,
        }] + [{
            'x': float(pose.pose.position.x),
            'y': float(pose.pose.position.y),
            'yaw': yaw_from_quaternion(pose.pose.orientation),
        } for pose in path.poses]
        if self.args.action_mode == 'navigate-to-pose':
            if self.args.goal_x is not None:
                path.poses[-1].pose.position.x = self.args.goal_x
                path.poses[-1].pose.position.y = self.args.goal_y
                goal_yaw = yaw if self.args.goal_yaw is None else self.args.goal_yaw
                path.poses[-1].pose.orientation.x = 0.0
                path.poses[-1].pose.orientation.y = 0.0
                path.poses[-1].pose.orientation.z = math.sin(goal_yaw / 2.0)
                path.poses[-1].pose.orientation.w = math.cos(goal_yaw / 2.0)
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
                'yaw': yaw_from_quaternion(goal.pose.pose.orientation),
            }
        else:
            goal = FollowPath.Goal()
            goal.path = path
            goal.controller_id = 'FollowPath'
            goal.goal_checker_id = 'goal_checker'
            if hasattr(goal, 'progress_checker_id'):
                goal.progress_checker_id = 'progress_checker'
        self.goal_sent_wall = time.monotonic()
        self.trajectory = [self.trajectory_sample(self.latest_odom)]
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
        self.action_result_pose = pose_dict(self.latest_odom)
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
        # Keep spinning for the greater of the requested post-result observation and bounded BT
        # drain intervals so neither terminal motion nor an already-published BT record is lost.

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
        action_trajectory = [
            sample for sample in self.trajectory if sample['phase'] == 'action']
        docking_success = next((sample for sample in self.docking_evaluations
                                if sample.get('success')), None)
        docking_clearances = [sample.get('minimumRegionClearance')
                              for sample in self.docking_evaluations
                              if sample.get('hullInsideDockRegion') and
                              sample.get('minimumRegionClearance') is not None]
        summary = {
            'schema': 'crane-nav2-controller-fixture-v1',
            'scope': ('nav2-navigate-to-pose' if self.args.action_mode == 'navigate-to-pose'
                      else 'nav2-controller-server-follow-path'),
            'actionMode': self.args.action_mode,
            'pathShape': self.args.path_shape,
            'suppliedPathFile': (self.supplied_path['sourcePath']
                                 if self.supplied_path else None),
            'suppliedPathFileSha256': (self.supplied_path['sourceSha256']
                                       if self.supplied_path else None),
            'actionName': self.action_name,
            'status': status,
            'odomTopic': self.args.odom_topic,
            'inputTopic': self.args.input_topic,
            'inputType': self.args.input_type,
            'outputTopic': self.args.output_topic,
            'odometryMessages': self.odom_count,
            'controllerCommands': self.command_count,
            'maximumLinearCommand': self.maximum_linear_command,
            'maximumLateralCommand': self.maximum_lateral_command,
            'maximumAngularCommand': self.maximum_angular_command,
            'returnedCommands': self.output_count,
            'goalAttempts': self.goal_attempts,
            'navigateToPoseFeedbackMessages': self.feedback_count,
            'actionFeedbackMessages': self.feedback_count,
            'recoveryCountFeedbackAvailable': self.recovery_count_feedback_available,
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
            'goal': self.goal_description,
            'planTopic': self.args.plan_topic,
            'planHistory': self.plan_history,
            'planHistoryProvenance': (
                'delivered-nav-msgs-path-not-proven-controller-consumed'),
            'costmapTopic': self.args.costmap_topic,
            'costmapMessages': self.costmap_count,
            'costmapService': self.args.costmap_service,
            'costmapServiceSnapshots': self.costmap_service_snapshot_count,
            'costmapObservations': self.costmap_count + self.costmap_service_snapshot_count,
            'maximumOccupiedCostmapCells': self.maximum_occupied_costmap_cells,
            'latestCostmapSnapshot': self.latest_costmap_snapshot,
            'dockingEvaluatorTopic': self.args.docking_evaluator_topic,
            'dockingEvaluatorMessages': len(self.docking_evaluations),
            'dockingEvaluation': (self.docking_evaluations[-1]
                                  if self.docking_evaluations else None),
            'dockingSuccessObserved': docking_success is not None,
            'dockingSuccessFirstSimulationTime': (
                docking_success.get('simulationTime') if docking_success else None),
            'minimumDockRegionClearanceMeters': (
                min(docking_clearances) if docking_clearances else None),
            'maximumProhibitedContactCount': max(
                (sample.get('prohibitedContactCount', 0)
                 for sample in self.docking_evaluations), default=0),
            'goalToFirstCommandWallSeconds': (
                self.first_command_wall - self.goal_sent_wall
                if self.first_command_wall is not None and self.goal_sent_wall is not None
                else None),
            'wallSeconds': time.monotonic() - self.started_wall,
            'displacementMeters': displacement,
            'deltaX': dx,
            'deltaY': dy,
            'initialPose': pose_dict(self.initial_odom),
            'finalPose': pose_dict(final),
            'actionResultPose': self.action_result_pose,
            'postResultSecondsRequested': self.args.post_result_seconds,
            'postResultSecondsObserved': (
                time.monotonic() - self.result_received_wall
                if self.result_received_wall is not None else None),
            'motionAtActionResult': terminal_motion(action_trajectory),
            'terminalMotion': terminal_motion(self.trajectory),
            'postResultCoastDistanceMeters': coast_distance(
                self.action_result_pose, final),
            'plannedPath': self.planned_path if self.args.action_mode == 'follow-path' else None,
            'actionPathMetrics': (path_metrics(self.planned_path, action_trajectory)
                                  if self.args.action_mode == 'follow-path' else None),
            'pathMetrics': (path_metrics(self.planned_path, self.trajectory)
                            if self.args.action_mode == 'follow-path' else None),
            'trajectory': self.trajectory,
            'dockingEvaluations': self.docking_evaluations,
            'provenance': 'latest-delivered-odometry-not-proven-internal-consumption',
            'costmapProvenance': (
                'nav2-get-costmap-snapshot-not-proven-controller-consumption'),
        }
        if self.args.output:
            with open(self.args.output, 'w', encoding='utf-8') as stream:
                json.dump(summary, stream, indent=2, sort_keys=True)
                stream.write('\n')
        console_summary = dict(summary)
        console_summary.pop('plannedPath', None)
        console_summary.pop('trajectory', None)
        console_summary.pop('latestCostmapSnapshot', None)
        console_summary.pop('dockingEvaluations', None)
        console_summary.pop('planHistory', None)
        console_summary['planCount'] = len(self.plan_history)
        console_summary['trajectorySampleCount'] = len(self.trajectory)
        print(json.dumps(console_summary, sort_keys=True), flush=True)
        rclpy.shutdown()


def yaw_from_quaternion(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                      1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def encode_costmap_snapshot(message):
    """Encode one nav2_msgs/Costmap without expanding fixture JSON by tens of thousands of cells."""
    metadata = message.metadata
    raw = bytes(int(value) & 0xff for value in message.data)
    return {
        'frameId': message.header.frame_id,
        'stamp': stamp_dict(message.header.stamp),
        'resolution': float(metadata.resolution),
        'sizeX': int(metadata.size_x),
        'sizeY': int(metadata.size_y),
        'origin': {
            'x': float(metadata.origin.position.x),
            'y': float(metadata.origin.position.y),
            'yaw': yaw_from_quaternion(metadata.origin.orientation),
        },
        'dataEncoding': 'base64+zlib+uint8-row-major',
        'dataSha256': hashlib.sha256(raw).hexdigest(),
        'data': base64.b64encode(zlib.compress(raw, level=9)).decode('ascii'),
    }


def stamp_dict(value):
    return {'sec': int(value.sec), 'nanosec': int(value.nanosec)}


def stamp_seconds(value):
    return float(value.sec) + float(value.nanosec) * 1e-9


def wrapped_angle(value):
    return math.remainder(value, 2.0 * math.pi)


def summarize_delivered_plan(points, requested_start=None, requested_goal=None):
    """Return bounded geometry for a delivered plan without retaining all poses.

    Signed deviation is positive to the left of the requested start-to-goal direction.  These
    measurements establish what the fixture received on the plan topic, not that a controller
    consumed the plan or that a particular costmap observation caused its geometry.
    """

    canonical = json.dumps(points, sort_keys=True, separators=(',', ':')).encode('utf-8')
    summary = {
        'pathId': f"sha256:{hashlib.sha256(canonical).hexdigest()}",
        'plannedLengthMeters': sum(
            math.hypot(end['x'] - start['x'], end['y'] - start['y'])
            for start, end in zip(points, points[1:])),
        'requestedRouteGeometryAvailable': False,
        'maximumAbsLateralDeviationFromRequestedRouteMeters': None,
        'minimumSignedLateralDeviationFromRequestedRouteMeters': None,
        'maximumSignedLateralDeviationFromRequestedRouteMeters': None,
    }
    if requested_start is None or requested_goal is None:
        return summary
    start_x = float(requested_start.x)
    start_y = float(requested_start.y)
    goal_x = float(requested_goal['x'])
    goal_y = float(requested_goal['y'])
    route_dx = goal_x - start_x
    route_dy = goal_y - start_y
    route_length = math.hypot(route_dx, route_dy)
    if route_length <= 1e-9:
        return summary
    signed = [
        (route_dx * (point['y'] - start_y) - route_dy * (point['x'] - start_x))
        / route_length
        for point in points
    ]
    summary.update({
        'requestedRouteGeometryAvailable': True,
        'maximumAbsLateralDeviationFromRequestedRouteMeters': max(map(abs, signed)),
        'minimumSignedLateralDeviationFromRequestedRouteMeters': min(signed),
        'maximumSignedLateralDeviationFromRequestedRouteMeters': max(signed),
    })
    return summary


def path_metrics(path, trajectory):
    if len(path) < 2 or not trajectory:
        return None
    path_length = sum(math.hypot(end['x'] - start['x'], end['y'] - start['y'])
                      for start, end in zip(path, path[1:]))
    canonical = json.dumps(path, sort_keys=True, separators=(',', ':')).encode('utf-8')
    cross_tracks = []
    heading_errors = []
    actual_length = 0.0
    previous = None
    for sample in trajectory:
        if previous is not None:
            actual_length += math.hypot(
                sample['x'] - previous['x'], sample['y'] - previous['y'])
        previous = sample
        best_distance = math.inf
        best_heading = path[0]['yaw']
        for start, end in zip(path, path[1:]):
            dx = end['x'] - start['x']
            dy = end['y'] - start['y']
            length_squared = dx * dx + dy * dy
            if length_squared <= 1e-12:
                continue
            projection = ((sample['x'] - start['x']) * dx
                          + (sample['y'] - start['y']) * dy) / length_squared
            projection = min(1.0, max(0.0, projection))
            nearest_x = start['x'] + projection * dx
            nearest_y = start['y'] + projection * dy
            distance = math.hypot(sample['x'] - nearest_x, sample['y'] - nearest_y)
            if distance < best_distance:
                best_distance = distance
                best_heading = math.atan2(dy, dx)
        cross_tracks.append(best_distance)
        heading_errors.append(abs(wrapped_angle(sample['yaw'] - best_heading)))

    goal = path[-1]
    final = trajectory[-1]
    terminal_start = path[-2]
    terminal_dx = goal['x'] - terminal_start['x']
    terminal_dy = goal['y'] - terminal_start['y']
    terminal_length = math.hypot(terminal_dx, terminal_dy)
    overshoot = 0.0
    if terminal_length > 1e-12:
        overshoot = max(0.0, ((final['x'] - goal['x']) * terminal_dx
                             + (final['y'] - goal['y']) * terminal_dy)
                        / terminal_length)
    return {
        'pathId': f"sha256:{hashlib.sha256(canonical).hexdigest()}",
        'plannedLengthMeters': path_length,
        'actualLengthMeters': actual_length,
        'rmsCrossTrackErrorMeters': math.sqrt(
            sum(value * value for value in cross_tracks) / len(cross_tracks)),
        'maximumCrossTrackErrorMeters': max(cross_tracks),
        'rmsHeadingErrorRadians': math.sqrt(
            sum(value * value for value in heading_errors) / len(heading_errors)),
        'maximumHeadingErrorRadians': max(heading_errors),
        'finalXYErrorMeters': math.hypot(final['x'] - goal['x'], final['y'] - goal['y']),
        'finalYawErrorRadians': abs(wrapped_angle(final['yaw'] - goal['yaw'])),
        'terminalOvershootMeters': overshoot,
        'maximumBodySpeedMetersPerSecond': max(
            math.hypot(row['bodySurge'], row['bodySway']) for row in trajectory),
        'maximumAbsBodyYawRateRadiansPerSecond': max(
            abs(row['bodyYawRate']) for row in trajectory),
        'sampleCount': len(trajectory),
    }


def terminal_motion(trajectory):
    if not trajectory:
        return None
    final = trajectory[-1]
    return {
        'bodySpeedMetersPerSecond': math.hypot(final['bodySurge'], final['bodySway']),
        'absBodyYawRateRadiansPerSecond': abs(final['bodyYawRate']),
        'bodySurgeMetersPerSecond': final['bodySurge'],
        'bodySwayMetersPerSecond': final['bodySway'],
        'bodyYawRateRadiansPerSecond': final['bodyYawRate'],
    }


def coast_distance(start_pose, final_odometry):
    if start_pose is None or final_odometry is None:
        return None
    final_pose = pose_dict(final_odometry)
    return math.hypot(
        final_pose['x'] - start_pose['x'], final_pose['y'] - start_pose['y'])


def path_sample(shape, length, fraction, lateral_amplitude, turn_angle):
    """Return forward, left, and tangent-yaw offsets in the initial path frame."""
    if shape == 'straight':
        return length * fraction, 0.0, 0.0
    if shape == 'gentle-turn':
        angle = turn_angle * fraction
        radius = length / max(abs(turn_angle), 1e-6)
        signed_radius = math.copysign(radius, turn_angle)
        return (signed_radius * math.sin(angle),
                signed_radius * (1.0 - math.cos(angle)), angle)
    if shape == 's-turn':
        forward = length * fraction
        lateral = 0.5 * lateral_amplitude * (1.0 - math.cos(2.0 * math.pi * fraction))
        slope = (lateral_amplitude * math.pi / max(length, 1e-6) *
                 math.sin(2.0 * math.pi * fraction))
        return forward, lateral, math.atan(slope)
    raise ValueError(f'unsupported path shape: {shape}')


def pose_dict(odometry):
    if odometry is None:
        return None
    pose = odometry.pose.pose
    return {
        'x': float(pose.position.x),
        'y': float(pose.position.y),
        'yaw': yaw_from_quaternion(pose.orientation),
    }


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--odom-topic', default='/crane/odom')
    parser.add_argument('--input-topic', default='/nav2/cmd_vel')
    parser.add_argument('--costmap-topic', default='/local_costmap/costmap')
    parser.add_argument('--plan-topic', default='/plan')
    parser.add_argument('--costmap-service', default='/local_costmap/get_costmap')
    parser.add_argument('--costmap-sample-period', type=float, default=0.5)
    parser.add_argument('--docking-evaluator-topic', default='/crane/docking_evaluator')
    parser.add_argument('--input-type', choices=('twist', 'stamped'), default='stamped')
    parser.add_argument('--output-topic', default='/crane/cmd_vel_stamped')
    parser.add_argument('--action-mode', choices=('follow-path', 'navigate-to-pose'),
                        default='follow-path')
    parser.add_argument('--action-name')
    parser.add_argument('--distance', type=float, default=0.5)
    parser.add_argument('--goal-x', type=float)
    parser.add_argument('--goal-y', type=float)
    parser.add_argument('--goal-yaw', type=float)
    parser.add_argument('--path-points', type=int, default=20)
    parser.add_argument('--path-file')
    parser.add_argument('--path-heading-offset', type=float, default=0.0,
                        help='FollowPath heading offset from initial body yaw, radians')
    parser.add_argument('--path-shape', choices=('straight', 'gentle-turn', 's-turn'),
                        default='straight')
    parser.add_argument('--path-lateral-amplitude', type=float, default=2.0)
    parser.add_argument('--path-turn-angle', type=float, default=math.pi / 4.0)
    parser.add_argument('--duration', type=float, default=25.0)
    parser.add_argument('--post-result-seconds', type=float, default=0.0)
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
    if (args.goal_x is None) != (args.goal_y is None):
        parser.error('--goal-x and --goal-y must be supplied together')
    if args.goal_x is not None and args.action_mode != 'navigate-to-pose':
        parser.error('absolute goals are supported only for navigate-to-pose')
    if args.path_heading_offset != 0.0 and args.action_mode != 'follow-path':
        parser.error('--path-heading-offset is supported only for follow-path')
    if args.path_shape != 'straight' and args.action_mode != 'follow-path':
        parser.error('--path-shape is supported only for follow-path')
    if args.path_file and args.action_mode != 'follow-path':
        parser.error('--path-file is supported only for follow-path')
    if args.post_result_seconds < 0.0:
        parser.error('--post-result-seconds must be non-negative')
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
