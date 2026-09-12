#!/usr/bin/env python3

import json
import math
import time
import threading

import rclpy

from rclpy.action import ActionClient
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.node import Node

from std_msgs.msg import String
from geometry_msgs.msg import Pose

from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint

from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import Constraints
from moveit_msgs.msg import PositionConstraint
from moveit_msgs.msg import OrientationConstraint
from moveit_msgs.msg import BoundingVolume

from shape_msgs.msg import SolidPrimitive

from tf2_ros import Buffer
from tf2_ros import TransformListener


class ERCManipulationNode(Node):

    def __init__(self):

        super().__init__('erc_manipulation_node')

        self.cb_group = ReentrantCallbackGroup()

        # ==========================================================
        # ROS
        # ==========================================================

        self.status_pub = self.create_publisher(
            String,
            '/erc/manipulation/status',
            10,
        )

        self.command_sub = self.create_subscription(
            String,
            '/erc/manipulation/command',
            self.command_callback,
            10,
            callback_group=self.cb_group,
        )

        self.target_sub = self.create_subscription(
            String,
            '/erc/perception/book_target_3d',
            self.target_callback,
            10,
            callback_group=self.cb_group,
        )

        # ==========================================================
        # ACTIONS
        # ==========================================================

        self.move_group_client = ActionClient(
            self,
            MoveGroup,
            '/move_action',
            callback_group=self.cb_group,
        )

        self.gripper_client = ActionClient(
            self,
            FollowJointTrajectory,
            '/gripper_left_controller/follow_joint_trajectory',
            callback_group=self.cb_group,
        )

        # ==========================================================
        # TF
        # ==========================================================

        self.tf_buffer = Buffer()

        self.tf_listener = TransformListener(
            self.tf_buffer,
            self,
        )

        # ==========================================================
        # STATE
        # ==========================================================

        self.latest_targets = {}
        self.latest_target_times = {}

        self.busy = False
        self.requested_color = None
        self.active_target = None

        self.sequence_state = 'IDLE'

        self.grasp_start_lock = threading.Lock()

        # ==========================================================
        # TARGET
        # ==========================================================

        self.max_target_age = 3.0

        # ==========================================================
        # GEOMETRY
        # ==========================================================
        #
        # The perception target is approximately the visible centre
        # of the BLUE book.
        #
        # Keep a useful pre-grasp distance, then move almost to the
        # object centre before closing.
        # ==========================================================

        self.pregrasp_offset_x = 0.12

        self.final_offset_x = 0.010

        # Slight downward correction because RGB-D often sees the
        # upper visible part of the front book face.
        self.final_offset_z = -0.015

        # ==========================================================
        # GRIPPER
        # ==========================================================

        self.gripper_open_position = 0.070

        self.gripper_closed_position = 0.014

        self.gripper_tight_position = 0.010

        self.gripper_open_time = 0.70
        self.gripper_close_time = 1.10
        self.gripper_tight_time = 0.70

        self.grasp_settle_time = 0.60

        # ==========================================================
        # LIFT
        # ==========================================================

        self.test_lift = 0.025

        self.full_lift = 0.12

        # ==========================================================
        # SPEED
        # ==========================================================

        self.pregrasp_velocity = 0.35
        self.pregrasp_acceleration = 0.30

        self.approach_velocity = 0.10
        self.approach_acceleration = 0.10

        self.test_lift_velocity = 0.10
        self.test_lift_acceleration = 0.10

        self.full_lift_velocity = 0.25
        self.full_lift_acceleration = 0.20

        self.get_logger().info(
            '================================================'
        )

        self.get_logger().info(
            'ERC PHASE 8 ROBUST GRASP NODE'
        )

        self.get_logger().info(
            'Free-orientation pregrasp: ENABLED'
        )

        self.get_logger().info(
            'Loose-orientation final approach: ENABLED'
        )

        self.get_logger().info(
            'Precision slow contact approach: ENABLED'
        )

        self.get_logger().info(
            'Firm close + tightening: ENABLED'
        )

        self.get_logger().info(
            'Test lift: ENABLED'
        )

        self.get_logger().info(
            '================================================'
        )

        self.publish_status(
            'READY',
            'Robust manipulation node ready.',
        )

    # ==============================================================
    # STATUS
    # ==============================================================

    def publish_status(
        self,
        state,
        message,
        extra=None,
    ):

        payload = {
            'state': state,
            'message': message,
        }

        if extra is not None:
            payload['data'] = extra

        msg = String()

        msg.data = json.dumps(payload)

        self.status_pub.publish(msg)

        self.get_logger().info(
            f'{state}: {message}'
        )

    # ==============================================================
    # TARGET CALLBACK
    # ==============================================================

    def target_callback(
        self,
        msg,
    ):

        try:
            target = json.loads(msg.data)

        except Exception as exc:

            self.get_logger().error(
                f'Invalid target JSON: {exc}'
            )

            return

        color = str(
            target.get(
                'color',
                'UNKNOWN',
            )
        ).upper()

        required = (
            'x',
            'y',
            'z',
            'reachable',
        )

        if not all(
            key in target
            for key in required
        ):
            return

        try:

            x = float(target['x'])
            y = float(target['y'])
            z = float(target['z'])

        except Exception:
            return

        if not all(
            math.isfinite(v)
            for v in (
                x,
                y,
                z,
            )
        ):
            return

        self.latest_targets[color] = target

        self.latest_target_times[color] = (
            time.monotonic()
        )

        if (
            self.busy
            and
            self.sequence_state == 'WAIT_TARGET'
            and
            self.requested_color == color
        ):

            self.try_start_grasp(color)

    # ==============================================================
    # COMMAND
    # ==============================================================

    def command_callback(
        self,
        msg,
    ):

        try:
            command = json.loads(msg.data)

        except Exception as exc:

            self.publish_status(
                'COMMAND_ERROR',
                f'Invalid command: {exc}',
            )

            return

        action = str(
            command.get(
                'action',
                '',
            )
        ).upper()

        if action == 'GRASP_BOOK':

            if self.busy:

                self.publish_status(
                    'BUSY',
                    'Manipulation already active.',
                )

                return

            color = str(
                command.get(
                    'color',
                    'BLUE',
                )
            ).upper()

            self.busy = True
            self.requested_color = color
            self.sequence_state = 'WAIT_TARGET'

            self.publish_status(
                'GRASP_REQUESTED',
                (
                    f'Grasp requested for '
                    f'{color} book.'
                ),
            )

            self.try_start_grasp(color)

            return

        if action == 'OPEN_GRIPPER':

            if self.busy:

                self.publish_status(
                    'BUSY',
                    'Manipulation already active.',
                )

                return

            self.busy = True
            self.sequence_state = 'MANUAL_OPEN'

            self.send_gripper(
                self.gripper_open_position,
                self.gripper_open_time,
                self.manual_gripper_done,
            )

            return

        if action == 'CLOSE_GRIPPER':

            if self.busy:

                self.publish_status(
                    'BUSY',
                    'Manipulation already active.',
                )

                return

            self.busy = True
            self.sequence_state = 'MANUAL_CLOSE'

            self.send_gripper(
                self.gripper_closed_position,
                self.gripper_close_time,
                self.manual_gripper_done,
            )

            return

        if action == 'RESET':

            self.reset_sequence()

            self.publish_status(
                'READY',
                'Manipulation reset.',
            )

            return

        self.publish_status(
            'UNKNOWN_COMMAND',
            f'Unknown action: {action}',
        )

    # ==============================================================
    # START GRASP
    # ==============================================================

    def try_start_grasp(
        self,
        color,
    ):

        with self.grasp_start_lock:

            # Only one callback may transition WAIT_TARGET -> OPENING.
            if self.sequence_state != 'WAIT_TARGET':
                return

            target = self.latest_targets.get(color)

            timestamp = self.latest_target_times.get(
                color
            )

            if (
                target is None
                or
                timestamp is None
                or
                time.monotonic() - timestamp
                >
                self.max_target_age
            ):

                self.sequence_state = 'WAIT_TARGET'

                self.publish_status(
                    'WAITING_FOR_TARGET',
                    'Waiting for fresh target.',
                )

                return

            if not bool(
                target.get(
                    'reachable',
                    False,
                )
            ):

                self.publish_status(
                    'TARGET_UNREACHABLE',
                    'Target marked unreachable.',
                )

                self.reset_sequence()

                return

            try:

                x = float(target['x'])
                y = float(target['y'])
                z = float(target['z'])

            except Exception:

                self.publish_status(
                    'TARGET_ERROR',
                    'Invalid target coordinates.',
                )

                self.reset_sequence()

                return

            if not self.target_safe(
                x,
                y,
                z,
            ):

                self.publish_status(
                    'TARGET_UNSAFE',
                    (
                        f'Unsafe target '
                        f'X={x:.3f}, '
                        f'Y={y:.3f}, '
                        f'Z={z:.3f}'
                    ),
                )

                self.reset_sequence()

                return

            # Freeze coordinates for complete grasp.

            self.active_target = {
                'x': x,
                'y': y,
                'z': z,
                'color': color,
            }

            self.sequence_state = 'OPENING'

            self.publish_status(
                'OPENING_GRIPPER',
                'Opening gripper.',
                self.active_target,
            )

            self.send_gripper(
                self.gripper_open_position,
                self.gripper_open_time,
                self.open_done,
            )

    # ==============================================================
    # SAFETY
    # ==============================================================

    def target_safe(
        self,
        x,
        y,
        z,
    ):

        return (
            0.20 <= x <= 0.72
            and
            -0.45 <= y <= 0.45
            and
            0.20 <= z <= 1.25
        )

    # ==============================================================
    # GRIPPER
    # ==============================================================

    def send_gripper(
        self,
        position,
        duration,
        done_callback,
    ):

        if not self.gripper_client.wait_for_server(
            timeout_sec=3.0
        ):

            self.publish_status(
                'GRIPPER_ERROR',
                'Gripper server unavailable.',
            )

            self.reset_sequence()

            return

        trajectory = JointTrajectory()

        trajectory.joint_names = [
            'gripper_left_finger_joint',
        ]

        point = JointTrajectoryPoint()

        point.positions = [
            float(position),
        ]

        seconds = int(duration)

        nanoseconds = int(
            (
                duration
                -
                seconds
            )
            *
            1_000_000_000
        )

        point.time_from_start.sec = seconds
        point.time_from_start.nanosec = nanoseconds

        trajectory.points = [
            point,
        ]

        goal = FollowJointTrajectory.Goal()

        goal.trajectory = trajectory

        future = self.gripper_client.send_goal_async(
            goal
        )

        future.add_done_callback(
            lambda f:
                self.gripper_goal_response(
                    f,
                    done_callback,
                )
        )

    def gripper_goal_response(
        self,
        future,
        done_callback,
    ):

        try:
            handle = future.result()

        except Exception as exc:

            self.publish_status(
                'GRIPPER_ERROR',
                str(exc),
            )

            self.reset_sequence()

            return

        if not handle.accepted:

            self.publish_status(
                'GRIPPER_ERROR',
                'Gripper goal rejected.',
            )

            self.reset_sequence()

            return

        future_result = (
            handle.get_result_async()
        )

        future_result.add_done_callback(
            lambda f:
                self.gripper_result(
                    f,
                    done_callback,
                )
        )

    def gripper_result(
        self,
        future,
        done_callback,
    ):

        try:

            result = future.result().result

            error_code = int(
                result.error_code
            )

        except Exception as exc:

            self.publish_status(
                'GRIPPER_ERROR',
                str(exc),
            )

            self.reset_sequence()

            return

        if error_code != 0:

            self.publish_status(
                'GRIPPER_ERROR',
                (
                    f'Controller error '
                    f'{error_code}'
                ),
            )

            self.reset_sequence()

            return

        done_callback()

    # ==============================================================
    # MOVEIT
    # ==============================================================

    def send_moveit_position(
        self,
        x,
        y,
        z,
        velocity,
        acceleration,
        done_callback,
        orientation_mode='FREE',
    ):

        if not self.move_group_client.wait_for_server(
            timeout_sec=5.0
        ):

            self.publish_status(
                'MOVEIT_ERROR',
                '/move_action unavailable.',
            )

            self.reset_sequence()

            return

        goal = MoveGroup.Goal()

        goal.request.group_name = (
            'arm_left_torso'
        )

        goal.request.num_planning_attempts = 4

        goal.request.allowed_planning_time = 5.0

        goal.request.max_velocity_scaling_factor = float(
            velocity
        )

        goal.request.max_acceleration_scaling_factor = float(
            acceleration
        )

        goal.request.start_state.is_diff = True

        # ==========================================================
        # POSITION
        # ==========================================================

        pc = PositionConstraint()

        pc.header.frame_id = 'base_link'

        pc.link_name = 'gripper_left_grasping_link'

        pc.weight = 1.0

        primitive = SolidPrimitive()

        primitive.type = SolidPrimitive.BOX

        if orientation_mode == 'FREE':

            primitive.dimensions = [
                0.035,
                0.035,
                0.035,
            ]

        else:

            primitive.dimensions = [
                0.018,
                0.018,
                0.018,
            ]

        pose = Pose()

        pose.position.x = float(x)
        pose.position.y = float(y)
        pose.position.z = float(z)

        pose.orientation.w = 1.0

        region = BoundingVolume()

        region.primitives.append(
            primitive
        )

        region.primitive_poses.append(
            pose
        )

        pc.constraint_region = region

        constraints = Constraints()

        constraints.position_constraints.append(
            pc
        )

        # ==========================================================
        # ORIENTATION
        #
        # PREGRASP:
        #   no orientation constraint.
        #
        # CONTACT/LIFT:
        #   loose constraint based on the orientation MoveIt reached
        #   at pregrasp. This prevents huge wrist flips while not
        #   freezing the original bad orientation.
        # ==========================================================

        if orientation_mode != 'FREE':

            orientation = (
                self.get_tool_orientation()
            )

            if orientation is not None:

                oc = OrientationConstraint()

                oc.header.frame_id = 'base_link'

                oc.link_name = (
                    'gripper_left_grasping_link'
                )

                oc.orientation.x = orientation['x']
                oc.orientation.y = orientation['y']
                oc.orientation.z = orientation['z']
                oc.orientation.w = orientation['w']

                # Loose but prevents wild flips.

                oc.absolute_x_axis_tolerance = 0.55
                oc.absolute_y_axis_tolerance = 0.55
                oc.absolute_z_axis_tolerance = 0.55

                oc.weight = 0.50

                constraints.orientation_constraints.append(
                    oc
                )

        goal.request.goal_constraints.append(
            constraints
        )

        goal.planning_options.plan_only = False

        goal.planning_options.look_around = False

        goal.planning_options.replan = True

        goal.planning_options.replan_attempts = 1

        goal.planning_options.replan_delay = 0.10

        self.get_logger().info(
            (
                'MOVEIT -> '
                f'X={x:.3f}, '
                f'Y={y:.3f}, '
                f'Z={z:.3f}, '
                f'mode={orientation_mode}, '
                f'v={velocity:.2f}'
            )
        )

        future = self.move_group_client.send_goal_async(
            goal
        )

        future.add_done_callback(
            lambda f:
                self.moveit_goal_response(
                    f,
                    done_callback,
                )
        )

    def get_tool_orientation(self):

        try:

            transform = (
                self.tf_buffer.lookup_transform(
                    'base_link',
                    'gripper_left_grasping_link',
                    rclpy.time.Time(),
                )
            )

            q = transform.transform.rotation

            return {
                'x': float(q.x),
                'y': float(q.y),
                'z': float(q.z),
                'w': float(q.w),
            }

        except Exception:
            return None

    def moveit_goal_response(
        self,
        future,
        done_callback,
    ):

        try:
            handle = future.result()

        except Exception as exc:

            self.publish_status(
                'MOVEIT_ERROR',
                str(exc),
            )

            self.reset_sequence()

            return

        if not handle.accepted:

            self.publish_status(
                'MOVEIT_ERROR',
                'MoveIt goal rejected.',
            )

            self.reset_sequence()

            return

        result_future = (
            handle.get_result_async()
        )

        result_future.add_done_callback(
            lambda f:
                self.moveit_result(
                    f,
                    done_callback,
                )
        )

    def moveit_result(
        self,
        future,
        done_callback,
    ):

        try:

            result = (
                future.result().result
            )

            code = int(
                result.error_code.val
            )

        except Exception as exc:

            self.publish_status(
                'MOVEIT_ERROR',
                str(exc),
            )

            self.reset_sequence()

            return

        if code != 1:

            self.publish_status(
                'MOVEIT_FAILED',
                (
                    f'MoveIt error '
                    f'code {code}.'
                ),
            )

            self.reset_sequence()

            return

        done_callback()

    # ==============================================================
    # SEQUENCE
    # ==============================================================

    def open_done(self):

        if self.active_target is None:
            self.reset_sequence()
            return

        t = self.active_target

        pre_x = (
            t['x']
            -
            self.pregrasp_offset_x
        )

        self.sequence_state = 'PREGRASP'

        self.publish_status(
            'MOVING_TO_PREGRASP',
            (
                'Moving to flexible '
                'pre-grasp position.'
            ),
        )

        self.send_moveit_position(
            pre_x,
            t['y'],
            t['z'],
            self.pregrasp_velocity,
            self.pregrasp_acceleration,
            self.pregrasp_done,
            orientation_mode='FREE',
        )

    def pregrasp_done(self):

        if self.active_target is None:
            self.reset_sequence()
            return

        t = self.active_target

        grasp_x = (
            t['x']
            -
            self.final_offset_x
        )

        grasp_z = (
            t['z']
            +
            self.final_offset_z
        )

        self.sequence_state = 'APPROACH'

        self.publish_status(
            'APPROACHING_BOOK',
            (
                'Slow precision approach '
                'to book centre.'
            ),
        )

        self.send_moveit_position(
            grasp_x,
            t['y'],
            grasp_z,
            self.approach_velocity,
            self.approach_acceleration,
            self.approach_done,
            orientation_mode='HOLD',
        )

    def approach_done(self):

        self.sequence_state = 'CLOSE'

        self.publish_status(
            'CLOSING_GRIPPER',
            'Closing firmly around book.',
        )

        self.send_gripper(
            self.gripper_closed_position,
            self.gripper_close_time,
            self.close_done,
        )

    def close_done(self):

        self.sequence_state = 'SETTLING'

        self.publish_status(
            'SETTLING_GRASP',
            'Allowing contacts to settle.',
        )

        time.sleep(
            self.grasp_settle_time
        )

        self.sequence_state = 'TIGHTEN'

        self.publish_status(
            'TIGHTENING_GRIPPER',
            'Tightening grasp.',
        )

        self.send_gripper(
            self.gripper_tight_position,
            self.gripper_tight_time,
            self.tight_done,
        )

    def tight_done(self):

        if self.active_target is None:
            self.reset_sequence()
            return

        t = self.active_target

        grasp_x = (
            t['x']
            -
            self.final_offset_x
        )

        grasp_z = (
            t['z']
            +
            self.final_offset_z
            +
            self.test_lift
        )

        self.sequence_state = 'TEST_LIFT'

        self.publish_status(
            'TEST_LIFTING_BOOK',
            'Performing small verification lift.',
        )

        self.send_moveit_position(
            grasp_x,
            t['y'],
            grasp_z,
            self.test_lift_velocity,
            self.test_lift_acceleration,
            self.test_lift_done,
            orientation_mode='HOLD',
        )

    def test_lift_done(self):

        self.sequence_state = 'RETIGHTEN'

        self.publish_status(
            'RESECURING_GRIPPER',
            'Re-tightening after test lift.',
        )

        self.send_gripper(
            self.gripper_tight_position,
            self.gripper_tight_time,
            self.retighten_done,
        )

    def retighten_done(self):

        if self.active_target is None:
            self.reset_sequence()
            return

        t = self.active_target

        grasp_x = (
            t['x']
            -
            self.final_offset_x
        )

        lift_z = (
            t['z']
            +
            self.final_offset_z
            +
            self.full_lift
        )

        self.sequence_state = 'FULL_LIFT'

        self.publish_status(
            'LIFTING_BOOK',
            'Performing full book lift.',
        )

        self.send_moveit_position(
            grasp_x,
            t['y'],
            lift_z,
            self.full_lift_velocity,
            self.full_lift_acceleration,
            self.lift_done,
            orientation_mode='HOLD',
        )

    def lift_done(self):

        target = self.active_target

        color = (
            self.requested_color
            if self.requested_color
            else 'UNKNOWN'
        )

        self.publish_status(
            'GRASP_SUCCEEDED',
            (
                f'Secure grasp completed '
                f'for {color}.'
            ),
            target,
        )

        self.reset_sequence()

    # ==============================================================
    # MANUAL GRIPPER
    # ==============================================================

    def manual_gripper_done(self):

        if self.sequence_state == 'MANUAL_OPEN':

            self.publish_status(
                'GRIPPER_OPENED',
                'Left gripper opened.',
            )

        else:

            self.publish_status(
                'GRIPPER_CLOSED',
                'Left gripper closed.',
            )

        self.reset_sequence()

    # ==============================================================
    # RESET
    # ==============================================================

    def reset_sequence(self):

        self.busy = False
        self.requested_color = None
        self.active_target = None
        self.sequence_state = 'IDLE'


def main(args=None):

    rclpy.init(args=args)

    node = ERCManipulationNode()

    executor = (
        rclpy.executors.MultiThreadedExecutor(
            num_threads=4
        )
    )

    executor.add_node(node)

    try:
        executor.spin()

    except KeyboardInterrupt:
        pass

    finally:

        executor.shutdown()

        node.destroy_node()

        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
