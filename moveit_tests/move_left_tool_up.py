#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient

from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import (
    Constraints,
    PositionConstraint,
    OrientationConstraint,
    BoundingVolume,
)
from shape_msgs.msg import SolidPrimitive
from geometry_msgs.msg import Pose, PoseStamped
from tf2_ros import Buffer, TransformListener


class MoveLeftToolUp(Node):

    def __init__(self):
        super().__init__('move_left_tool_up_test')

        self.action_client = ActionClient(
            self,
            MoveGroup,
            '/move_action'
        )

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.timer = self.create_timer(1.0, self.start_once)
        self.started = False

    def start_once(self):
        if self.started:
            return

        self.started = True
        self.timer.cancel()

        self.get_logger().info('Waiting for MoveIt /move_action server...')

        if not self.action_client.wait_for_server(timeout_sec=10.0):
            self.get_logger().error('MoveIt /move_action server not available.')
            rclpy.shutdown()
            return

        self.get_logger().info('MoveIt action server available.')

        self.send_goal()

    def send_goal(self):

        # Current known tool pose from tf2_echo:
        #
        # position:
        # x = 0.283
        # y = 0.208
        # z = 0.480
        #
        # orientation:
        # x = 0.211
        # y = -0.001
        # z = 0.974
        # w = 0.079
        #
        # We move only +0.03 m in Z.

        target_pose = PoseStamped()
        target_pose.header.frame_id = 'base_link'
        target_pose.header.stamp = self.get_clock().now().to_msg()

        target_pose.pose.position.x = 0.283
        target_pose.pose.position.y = 0.208
        target_pose.pose.position.z = 0.510

        target_pose.pose.orientation.x = 0.211
        target_pose.pose.orientation.y = -0.001
        target_pose.pose.orientation.z = 0.974
        target_pose.pose.orientation.w = 0.079

        goal_msg = MoveGroup.Goal()

        goal_msg.request.group_name = 'arm_left_torso'
        goal_msg.request.num_planning_attempts = 10
        goal_msg.request.allowed_planning_time = 10.0
        goal_msg.request.max_velocity_scaling_factor = 0.15
        goal_msg.request.max_acceleration_scaling_factor = 0.15

        goal_msg.request.start_state.is_diff = True

        # Position constraint.
        position_constraint = PositionConstraint()
        position_constraint.header.frame_id = 'base_link'
        position_constraint.link_name = 'arm_left_tool_link'
        position_constraint.weight = 1.0

        primitive = SolidPrimitive()
        primitive.type = SolidPrimitive.BOX

        # Tight tolerance box around target position.
        primitive.dimensions = [
            0.02,
            0.02,
            0.02,
        ]

        primitive_pose = Pose()
        primitive_pose.position.x = target_pose.pose.position.x
        primitive_pose.position.y = target_pose.pose.position.y
        primitive_pose.position.z = target_pose.pose.position.z
        primitive_pose.orientation.w = 1.0

        bounding_volume = BoundingVolume()
        bounding_volume.primitives.append(primitive)
        bounding_volume.primitive_poses.append(primitive_pose)

        position_constraint.constraint_region = bounding_volume

        # Orientation constraint.
        orientation_constraint = OrientationConstraint()
        orientation_constraint.header.frame_id = 'base_link'
        orientation_constraint.link_name = 'arm_left_tool_link'

        orientation_constraint.orientation = target_pose.pose.orientation

        orientation_constraint.absolute_x_axis_tolerance = 0.15
        orientation_constraint.absolute_y_axis_tolerance = 0.15
        orientation_constraint.absolute_z_axis_tolerance = 0.15
        orientation_constraint.weight = 1.0

        constraints = Constraints()
        constraints.name = 'left_tool_target'
        constraints.position_constraints.append(position_constraint)
        constraints.orientation_constraints.append(orientation_constraint)

        goal_msg.request.goal_constraints.append(constraints)

        goal_msg.planning_options.plan_only = False
        goal_msg.planning_options.look_around = False
        goal_msg.planning_options.replan = True
        goal_msg.planning_options.replan_attempts = 3
        goal_msg.planning_options.replan_delay = 0.5

        self.get_logger().info(
            'Sending MoveIt goal: arm_left_tool_link +3 cm upward.'
        )

        future = self.action_client.send_goal_async(
            goal_msg,
            feedback_callback=self.feedback_callback,
        )

        future.add_done_callback(self.goal_response_callback)

    def feedback_callback(self, feedback_msg):
        feedback = feedback_msg.feedback

        self.get_logger().info(
            f'MoveIt state: {feedback.state}'
        )

    def goal_response_callback(self, future):

        goal_handle = future.result()

        if not goal_handle.accepted:
            self.get_logger().error('MoveIt goal was rejected.')
            rclpy.shutdown()
            return

        self.get_logger().info('MoveIt goal accepted.')

        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self.result_callback)

    def result_callback(self, future):

        result = future.result().result

        self.get_logger().info(
            f'MoveIt error code: {result.error_code.val}'
        )

        if result.error_code.val == 1:
            self.get_logger().info(
                'SUCCESS: MoveIt planned and executed the motion.'
            )
        else:
            self.get_logger().error(
                'MoveIt did not complete successfully.'
            )

        rclpy.shutdown()


def main(args=None):

    rclpy.init(args=args)

    node = MoveLeftToolUp()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    if rclpy.ok():
        rclpy.shutdown()


if __name__ == '__main__':
    main()
