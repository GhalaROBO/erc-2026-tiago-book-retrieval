#!/usr/bin/env python3

import time

from control_msgs.action import FollowJointTrajectory
from rclpy.action import ActionClient
from trajectory_msgs.msg import JointTrajectoryPoint


class HeadScanner:

    def __init__(self, node):

        self.node = node

        self.client = ActionClient(
            node,
            FollowJointTrajectory,
            '/head_controller/follow_joint_trajectory',
        )

        self.scan_positions = [
            (0.00, -0.30),
            (0.55, -0.30),
            (-0.55, -0.30),
            (0.00, -0.15),
            (0.00, -0.45),
        ]

        self.current_index = 0

        self.running = False
        self.goal_active = False

        self.last_move_time = 0.0

        # Faster than before.
        self.move_duration = 0.65
        self.pause_between_moves = 0.15

        self.goal_handle = None

    def start(self):

        if self.running:
            return

        self.current_index = 0
        self.running = True
        self.goal_active = False

        self.node.get_logger().info(
            'Fast automatic head scan started.'
        )

    def stop(self):

        self.running = False
        self.goal_active = False

        if self.goal_handle is not None:

            try:
                self.goal_handle.cancel_goal_async()
            except Exception:
                pass

        self.goal_handle = None

        self.node.get_logger().info(
            'Automatic head scan stopped.'
        )

    def center(self):

        self.running = False
        self.goal_active = False

        self._send_position(
            0.0,
            -0.25,
        )

    def tick(self):

        if not self.running:
            return

        if self.goal_active:
            return

        now = time.monotonic()

        if (
            now - self.last_move_time
            <
            self.pause_between_moves
        ):
            return

        pan, tilt = self.scan_positions[
            self.current_index
        ]

        self.node.get_logger().info(
            (
                f'HEAD SCAN: '
                f'pan={pan:.2f}, '
                f'tilt={tilt:.2f}'
            )
        )

        self._send_position(
            pan,
            tilt,
        )

        self.current_index += 1

        if (
            self.current_index
            >= len(self.scan_positions)
        ):
            self.current_index = 0

    def _send_position(
        self,
        pan,
        tilt,
    ):

        if not self.client.wait_for_server(
            timeout_sec=0.5
        ):

            self.node.get_logger().warn(
                'Head controller action server unavailable.'
            )

            self.goal_active = False
            return

        goal = FollowJointTrajectory.Goal()

        goal.trajectory.joint_names = [
            'head_1_joint',
            'head_2_joint',
        ]

        point = JointTrajectoryPoint()

        point.positions = [
            float(pan),
            float(tilt),
        ]

        seconds = int(
            self.move_duration
        )

        nanoseconds = int(
            (
                self.move_duration
                -
                seconds
            )
            *
            1_000_000_000
        )

        point.time_from_start.sec = seconds
        point.time_from_start.nanosec = nanoseconds

        goal.trajectory.points = [
            point
        ]

        self.goal_active = True
        self.last_move_time = time.monotonic()

        future = self.client.send_goal_async(
            goal
        )

        future.add_done_callback(
            self._goal_response_callback
        )

    def _goal_response_callback(
        self,
        future,
    ):

        try:
            goal_handle = future.result()

        except Exception as exc:

            self.node.get_logger().warn(
                f'Head command error: {exc}'
            )

            self.goal_active = False
            return

        if not goal_handle.accepted:

            self.node.get_logger().warn(
                'Head scan goal rejected.'
            )

            self.goal_active = False
            return

        self.goal_handle = goal_handle

        result_future = (
            goal_handle.get_result_async()
        )

        result_future.add_done_callback(
            self._result_callback
        )

    def _result_callback(
        self,
        future,
    ):

        self.goal_active = False
        self.goal_handle = None
        self.last_move_time = time.monotonic()

        try:
            future.result()

        except Exception as exc:

            self.node.get_logger().warn(
                f'Head movement result error: {exc}'
            )
