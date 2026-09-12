#!/usr/bin/env python3

import json
import math
import time

import rclpy

from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import PoseWithCovarianceStamped
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import QoSProfile
from rclpy.qos import QoSDurabilityPolicy
from rclpy.qos import QoSHistoryPolicy
from rclpy.qos import QoSReliabilityPolicy
from std_msgs.msg import String
from std_srvs.srv import Empty


class NavigationNode(Node):

    def __init__(self):
        super().__init__('erc_navigation_node')

        # ==========================================================
        # CONFIGURATION
        # ==========================================================

        self.auto_initial_pose = {
            'x': 0.20,
            'y': 0.10,
            'yaw': 0.0,
        }

        self.initial_xy_variance = 0.25
        self.initial_yaw_variance = 0.0685

        self.max_x_variance = 1.0
        self.max_y_variance = 1.0
        self.max_yaw_variance = 0.5

        self.required_healthy_samples = 4

        self.initial_pose_retry_period = 2.0
        self.max_initial_pose_attempts = 20

        self.shelf_candidates_relative = [
            (0.60, 0.00, 0.00),
            (0.50, 0.20, 0.00),
            (0.50, -0.20, 0.00),
            (0.40, 0.00, 0.00),
            (0.30, 0.15, 0.00),
            (0.30, -0.15, 0.00),
        ]

        self.bin_candidates_relative = [
            (0.15, -0.60, -1.57),
            (0.15, 0.60, 1.57),
            (-0.40, 0.00, 3.14),
            (0.30, -0.35, -0.80),
            (0.30, 0.35, 0.80),
        ]

        # ==========================================================
        # STATE
        # ==========================================================

        self.latest_pose = None
        self.home_pose = None

        self.healthy_sample_count = 0
        self.localization_ready = False

        self.initial_pose_attempts = 0
        self.last_initial_pose_time = 0.0

        self.busy = False
        self.current_goal_handle = None
        self.current_destination = None

        self.retry_candidates = []
        self.retry_index = 0

        self.cancel_due_to_localization = False

        # ==========================================================
        # NAV2 ACTION
        # ==========================================================

        self.nav_client = ActionClient(
            self,
            NavigateToPose,
            '/navigate_to_pose',
        )

        # ==========================================================
        # TOPICS
        # ==========================================================

        self.command_sub = self.create_subscription(
            String,
            '/erc/navigation/command',
            self.command_callback,
            10,
        )

        self.status_pub = self.create_publisher(
            String,
            '/erc/navigation/status',
            10,
        )

        self.amcl_sub = self.create_subscription(
            PoseWithCovarianceStamped,
            '/amcl_pose',
            self.amcl_callback,
            10,
        )

        initial_pose_qos = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.VOLATILE,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=10,
        )

        self.initial_pose_pub = self.create_publisher(
            PoseWithCovarianceStamped,
            '/initialpose',
            initial_pose_qos,
        )

        # ==========================================================
        # SERVICES
        # ==========================================================

        self.nomotion_client = self.create_client(
            Empty,
            '/request_nomotion_update',
        )

        # ==========================================================
        # TIMERS
        # ==========================================================

        self.localization_timer = self.create_timer(
            1.0,
            self.localization_startup_timer,
        )

        self.health_timer = self.create_timer(
            0.5,
            self.localization_health_timer,
        )

        # ==========================================================
        # STARTUP
        # ==========================================================

        self.get_logger().info(
            '================================================'
        )
        self.get_logger().info(
            'ERC AUTOMATIC NAVIGATION NODE'
        )
        self.get_logger().info(
            'Automatic AMCL initialization enabled.'
        )
        self.get_logger().info(
            'Automatic HOME capture enabled.'
        )
        self.get_logger().info(
            'Automatic localization recovery enabled.'
        )
        self.get_logger().info(
            'Automatic Nav2 waypoint fallback enabled.'
        )
        self.get_logger().info(
            '================================================'
        )

        self.publish_status(
            'INITIALIZING',
            'Waiting for AMCL and initializing localization.',
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

        if self.current_destination is not None:
            payload['destination'] = self.current_destination

        if extra is not None:
            payload['data'] = extra

        msg = String()
        msg.data = json.dumps(payload)

        self.status_pub.publish(msg)

        self.get_logger().info(
            f'{state}: {message}'
        )

    # ==============================================================
    # ANGLE HELPERS
    # ==============================================================

    def quaternion_to_yaw(
        self,
        z,
        w,
    ):
        return 2.0 * math.atan2(
            float(z),
            float(w),
        )

    def yaw_to_quaternion(
        self,
        yaw,
    ):
        return (
            math.sin(yaw / 2.0),
            math.cos(yaw / 2.0),
        )

    def normalize_angle(
        self,
        angle,
    ):
        while angle > math.pi:
            angle -= 2.0 * math.pi

        while angle < -math.pi:
            angle += 2.0 * math.pi

        return angle

    # ==============================================================
    # AUTOMATIC INITIAL POSE
    # ==============================================================

    def publish_automatic_initial_pose(self):

        pose = PoseWithCovarianceStamped()

        pose.header.frame_id = 'map'
        pose.header.stamp = self.get_clock().now().to_msg()

        pose.pose.pose.position.x = float(
            self.auto_initial_pose['x']
        )
        pose.pose.pose.position.y = float(
            self.auto_initial_pose['y']
        )
        pose.pose.pose.position.z = 0.0

        qz, qw = self.yaw_to_quaternion(
            self.auto_initial_pose['yaw']
        )

        pose.pose.pose.orientation.x = 0.0
        pose.pose.pose.orientation.y = 0.0
        pose.pose.pose.orientation.z = qz
        pose.pose.pose.orientation.w = qw

        covariance = [0.0] * 36

        covariance[0] = self.initial_xy_variance
        covariance[7] = self.initial_xy_variance
        covariance[35] = self.initial_yaw_variance

        pose.pose.covariance = covariance

        self.initial_pose_pub.publish(pose)

        self.initial_pose_attempts += 1
        self.last_initial_pose_time = time.monotonic()

        self.get_logger().info(
            (
                f'AUTO INITIAL POSE sent '
                f'(attempt {self.initial_pose_attempts}/'
                f'{self.max_initial_pose_attempts}) '
                f'x={self.auto_initial_pose["x"]:.2f}, '
                f'y={self.auto_initial_pose["y"]:.2f}, '
                f'yaw={self.auto_initial_pose["yaw"]:.2f}'
            )
        )

        if self.nomotion_client.service_is_ready():
            request = Empty.Request()
            self.nomotion_client.call_async(request)

    # ==============================================================
    # LOCALIZATION STARTUP / RECOVERY
    # ==============================================================

    def localization_startup_timer(self):

        if self.localization_ready:
            return

        now = time.monotonic()

        if (
            self.initial_pose_attempts
            >= self.max_initial_pose_attempts
        ):
            self.publish_status(
                'LOCALIZATION_FAILED',
                'Automatic localization failed after maximum retries.',
            )

            self.localization_timer.cancel()
            return

        if (
            now - self.last_initial_pose_time
            >= self.initial_pose_retry_period
        ):
            self.publish_automatic_initial_pose()

    def start_localization_recovery(self):

        self.get_logger().warn(
            'Starting automatic AMCL recovery.'
        )

        self.localization_ready = False
        self.healthy_sample_count = 0

        self.initial_pose_attempts = 0
        self.last_initial_pose_time = 0.0

        self.publish_status(
            'RECOVERING_LOCALIZATION',
            'Automatically recovering AMCL localization.',
            self.latest_pose,
        )

    # ==============================================================
    # AMCL CALLBACK
    # ==============================================================

    def amcl_callback(
        self,
        msg,
    ):

        pose = msg.pose.pose
        covariance = msg.pose.covariance

        yaw = self.quaternion_to_yaw(
            pose.orientation.z,
            pose.orientation.w,
        )

        self.latest_pose = {
            'x': float(pose.position.x),
            'y': float(pose.position.y),
            'yaw': float(yaw),
            'x_var': float(covariance[0]),
            'y_var': float(covariance[7]),
            'yaw_var': float(covariance[35]),
        }

        if self.current_localization_is_healthy():
            self.healthy_sample_count += 1
        else:
            self.healthy_sample_count = 0

        if (
            not self.localization_ready
            and
            self.healthy_sample_count
            >= self.required_healthy_samples
        ):

            self.localization_ready = True

            # Only capture HOME once.
            if self.home_pose is None:
                self.home_pose = {
                    'x': self.latest_pose['x'],
                    'y': self.latest_pose['y'],
                    'yaw': self.latest_pose['yaw'],
                }

                self.get_logger().info(
                    '================================================'
                )
                self.get_logger().info(
                    'HOME CAPTURED AUTOMATICALLY'
                )
                self.get_logger().info(
                    f'HOME X   = {self.home_pose["x"]:.3f}'
                )
                self.get_logger().info(
                    f'HOME Y   = {self.home_pose["y"]:.3f}'
                )
                self.get_logger().info(
                    f'HOME YAW = {self.home_pose["yaw"]:.3f}'
                )
                self.get_logger().info(
                    '================================================'
                )

            self.publish_status(
                'READY',
                'Localization healthy.',
                {
                    'home': self.home_pose,
                    'localization': self.latest_pose,
                },
            )

    # ==============================================================
    # LOCALIZATION HEALTH
    # ==============================================================

    def current_localization_is_healthy(self):

        if self.latest_pose is None:
            return False

        values = [
            self.latest_pose['x'],
            self.latest_pose['y'],
            self.latest_pose['yaw'],
            self.latest_pose['x_var'],
            self.latest_pose['y_var'],
            self.latest_pose['yaw_var'],
        ]

        if not all(
            math.isfinite(value)
            for value in values
        ):
            return False

        return (
            self.latest_pose['x_var']
            < self.max_x_variance
            and
            self.latest_pose['y_var']
            < self.max_y_variance
            and
            self.latest_pose['yaw_var']
            < self.max_yaw_variance
        )

    def localization_health_timer(self):

        if not self.busy:
            return

        if self.current_localization_is_healthy():
            return

        if (
            self.current_goal_handle is not None
            and
            not self.cancel_due_to_localization
        ):

            self.cancel_due_to_localization = True

            self.get_logger().warn(
                'Localization degraded during navigation.'
            )

            self.publish_status(
                'RECOVERING_LOCALIZATION',
                'Canceling current goal and recovering localization.',
                self.latest_pose,
            )

            self.current_goal_handle.cancel_goal_async()

    # ==============================================================
    # RELATIVE POSE
    # ==============================================================

    def relative_to_map(
        self,
        dx,
        dy,
        dyaw,
    ):

        home_x = self.home_pose['x']
        home_y = self.home_pose['y']
        home_yaw = self.home_pose['yaw']

        map_dx = (
            dx * math.cos(home_yaw)
            -
            dy * math.sin(home_yaw)
        )

        map_dy = (
            dx * math.sin(home_yaw)
            +
            dy * math.cos(home_yaw)
        )

        return {
            'x': home_x + map_dx,
            'y': home_y + map_dy,
            'yaw': self.normalize_angle(
                home_yaw + dyaw
            ),
        }

    # ==============================================================
    # COMMAND CALLBACK
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
                f'Invalid command JSON: {exc}',
            )
            return

        action = str(
            command.get(
                'action',
                ''
            )
        ).upper()

        if action == 'STATUS':

            self.publish_status(
                (
                    'READY'
                    if self.localization_ready
                    else 'RECOVERING_LOCALIZATION'
                ),
                'Navigation status.',
                {
                    'home': self.home_pose,
                    'localization': self.latest_pose,
                    'busy': self.busy,
                },
            )
            return

        if action == 'CANCEL':

            if self.current_goal_handle is None:
                self.publish_status(
                    'IDLE',
                    'No active navigation goal.',
                )
                return

            self.current_goal_handle.cancel_goal_async()

            self.publish_status(
                'CANCELING',
                'Canceling active navigation.',
            )
            return

        if self.busy:
            self.publish_status(
                'BUSY',
                'Navigation is already active.',
            )
            return

        if self.home_pose is None:
            self.publish_status(
                'RECOVERING_LOCALIZATION',
                'HOME is not available yet.',
            )
            self.start_localization_recovery()
            return

        if not self.current_localization_is_healthy():
            self.start_localization_recovery()
            return

        if action == 'GO_TO_SHELF':

            shelf = int(
                command.get(
                    'shelf',
                    4
                )
            )

            if shelf != 4:
                self.publish_status(
                    'COMMAND_ERROR',
                    'Automatic integration mode supports SHELF_4.',
                )
                return

            self.begin_relative_navigation(
                'SHELF_4',
                self.shelf_candidates_relative,
            )
            return

        if action == 'GO_TO_BIN':

            self.begin_relative_navigation(
                'BIN',
                self.bin_candidates_relative,
            )
            return

        if action == 'GO_HOME':

            self.retry_candidates = [
                self.home_pose.copy()
            ]

            self.retry_index = 0
            self.current_destination = 'HOME'

            self.send_current_candidate()
            return

        if action == 'GO_TO_POSE':

            try:
                pose = {
                    'x': float(command['x']),
                    'y': float(command['y']),
                    'yaw': float(
                        command.get(
                            'yaw',
                            0.0
                        )
                    ),
                }

            except Exception as exc:
                self.publish_status(
                    'COMMAND_ERROR',
                    f'Invalid GO_TO_POSE: {exc}',
                )
                return

            self.retry_candidates = [pose]
            self.retry_index = 0

            self.current_destination = str(
                command.get(
                    'name',
                    'CUSTOM'
                )
            )

            self.send_current_candidate()
            return

        self.publish_status(
            'COMMAND_ERROR',
            f'Unknown action: {action}',
        )

    # ==============================================================
    # AUTOMATIC WAYPOINTS
    # ==============================================================

    def begin_relative_navigation(
        self,
        destination,
        relative_candidates,
    ):

        self.retry_candidates = []

        for dx, dy, dyaw in relative_candidates:
            self.retry_candidates.append(
                self.relative_to_map(
                    dx,
                    dy,
                    dyaw,
                )
            )

        self.retry_index = 0
        self.current_destination = destination

        self.send_current_candidate()

    # ==============================================================
    # SEND CURRENT CANDIDATE
    # ==============================================================

    def send_current_candidate(self):

        if (
            self.retry_index
            >= len(self.retry_candidates)
        ):
            destination = self.current_destination

            self.publish_status(
                'FAILED',
                (
                    f'All waypoint candidates failed '
                    f'for {destination}.'
                ),
            )

            self.reset_navigation()
            return

        if not self.current_localization_is_healthy():
            self.start_localization_recovery()
            return

        if not self.nav_client.wait_for_server(
            timeout_sec=5.0
        ):
            self.publish_status(
                'NAV2_UNAVAILABLE',
                '/navigate_to_pose action server unavailable.',
            )

            self.reset_navigation()
            return

        pose = self.retry_candidates[
            self.retry_index
        ]

        qz, qw = self.yaw_to_quaternion(
            pose['yaw']
        )

        goal = NavigateToPose.Goal()

        goal.pose = PoseStamped()

        goal.pose.header.frame_id = 'map'
        goal.pose.header.stamp = (
            self.get_clock().now().to_msg()
        )

        goal.pose.pose.position.x = float(
            pose['x']
        )
        goal.pose.pose.position.y = float(
            pose['y']
        )
        goal.pose.pose.position.z = 0.0

        goal.pose.pose.orientation.x = 0.0
        goal.pose.pose.orientation.y = 0.0
        goal.pose.pose.orientation.z = qz
        goal.pose.pose.orientation.w = qw

        self.busy = True
        self.cancel_due_to_localization = False

        self.publish_status(
            'NAVIGATING',
            (
                f'{self.current_destination}: '
                f'candidate {self.retry_index + 1}/'
                f'{len(self.retry_candidates)}.'
            ),
            {
                'pose': pose,
            },
        )

        future = self.nav_client.send_goal_async(
            goal,
            feedback_callback=self.feedback_callback,
        )

        future.add_done_callback(
            self.goal_response_callback
        )

    # ==============================================================
    # GOAL RESPONSE
    # ==============================================================

    def goal_response_callback(
        self,
        future,
    ):

        try:
            goal_handle = future.result()

        except Exception as exc:
            self.get_logger().error(
                f'Nav2 goal transmission failed: {exc}'
            )

            self.retry_goal()
            return

        if not goal_handle.accepted:
            self.get_logger().warn(
                (
                    f'Nav2 rejected candidate '
                    f'{self.retry_index + 1}.'
                )
            )

            self.retry_goal()
            return

        self.current_goal_handle = goal_handle

        self.publish_status(
            'ACCEPTED',
            (
                f'Nav2 accepted '
                f'{self.current_destination} '
                f'candidate {self.retry_index + 1}.'
            ),
        )

        result_future = (
            goal_handle.get_result_async()
        )

        result_future.add_done_callback(
            self.navigation_result_callback
        )

    # ==============================================================
    # FEEDBACK
    # ==============================================================

    def feedback_callback(
        self,
        feedback_msg,
    ):

        try:
            distance = float(
                feedback_msg.feedback.distance_remaining
            )

            self.get_logger().info(
                (
                    f'{self.current_destination}: '
                    f'{distance:.2f} m remaining'
                )
            )

        except Exception:
            pass

    # ==============================================================
    # RESULT
    # ==============================================================

    def navigation_result_callback(
        self,
        future,
    ):

        try:
            wrapped = future.result()
            status = int(wrapped.status)

        except Exception as exc:
            self.get_logger().error(
                f'Unable to read Nav2 result: {exc}'
            )

            self.retry_goal()
            return

        if status == GoalStatus.STATUS_SUCCEEDED:

            destination = self.current_destination

            self.publish_status(
                'SUCCEEDED',
                f'Reached {destination}.',
                {
                    'candidate':
                        self.retry_index + 1,
                },
            )

            self.reset_navigation()
            return

        if status == GoalStatus.STATUS_CANCELED:

            if self.cancel_due_to_localization:

                self.get_logger().warn(
                    'Goal canceled because localization degraded.'
                )

                self.reset_navigation()

                self.start_localization_recovery()
                return

            self.publish_status(
                'CANCELED',
                'Navigation canceled.',
            )

            self.reset_navigation()
            return

        self.get_logger().warn(
            (
                f'Nav2 failed candidate '
                f'{self.retry_index + 1} '
                f'for {self.current_destination}.'
            )
        )

        self.retry_goal()

    # ==============================================================
    # RETRY
    # ==============================================================

    def retry_goal(self):

        self.current_goal_handle = None
        self.busy = False

        self.retry_index += 1

        if (
            self.retry_index
            < len(self.retry_candidates)
        ):

            self.publish_status(
                'RETRYING',
                (
                    f'Trying another waypoint for '
                    f'{self.current_destination}.'
                ),
                {
                    'next_candidate':
                        self.retry_index + 1,
                },
            )

            self.send_current_candidate()
            return

        destination = self.current_destination

        self.publish_status(
            'FAILED',
            (
                f'No automatic waypoint could be reached '
                f'for {destination}.'
            ),
        )

        self.reset_navigation()

    # ==============================================================
    # RESET
    # ==============================================================

    def reset_navigation(self):

        self.current_goal_handle = None
        self.current_destination = None

        self.retry_candidates = []
        self.retry_index = 0

        self.busy = False
        self.cancel_due_to_localization = False


def main(args=None):

    rclpy.init(args=args)

    node = NavigationNode()

    try:
        rclpy.spin(node)

    except KeyboardInterrupt:
        pass

    finally:
        node.destroy_node()

        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
