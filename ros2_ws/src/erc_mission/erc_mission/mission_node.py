#!/usr/bin/env python3

import json
import math
import time

import rclpy

from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.time import Time
from std_msgs.msg import String
from tf2_ros import Buffer
from tf2_ros import TransformException
from tf2_ros import TransformListener

from erc_mission.head_scanner import HeadScanner


class MissionNode(Node):

    def __init__(self):
        super().__init__('erc_mission_node')

        # ==========================================================
        # TARGET
        # ==========================================================

        self.target_shelf = 4
        self.target_color = 'BLUE'

        # ==========================================================
        # TIMEOUTS
        # ==========================================================

        self.startup_timeout = 180.0
        self.navigation_timeout = 360.0
        self.search_timeout = 300.0

        self.max_book_age = 3.0
        self.max_target_age = 3.0

        self.grasp_stall_timeout = 420.0
        self.grasp_absolute_timeout = 1200.0

        self.release_timeout = 120.0

        # ==========================================================
        # GRASP TARGET
        # ==========================================================

        self.preferred_grasp_x = 0.43
        self.preferred_grasp_y = 0.20

        self.min_grasp_x = 0.25
        self.max_grasp_x = 0.65

        self.min_grasp_y = -0.35
        self.max_grasp_y = 0.40

        self.min_grasp_z = 0.20
        self.max_grasp_z = 1.10

        # ==========================================================
        # AUTOMATIC FINAL APPROACH
        # ==========================================================

        self.min_approach_x = 0.25
        self.max_approach_x = 1.80

        self.max_approach_abs_y = 1.00

        self.min_approach_z = 0.15
        self.max_approach_z = 1.30

        self.max_single_approach_move = 0.80

        self.max_final_approach_attempts = 4
        self.final_approach_attempts = 0

        # ==========================================================
        # STATE
        # ==========================================================

        self.state = 'WAIT_FOR_SYSTEM'
        self.state_enter_time = time.monotonic()

        self.command_sent = False

        # ==========================================================
        # NAVIGATION
        # ==========================================================

        self.navigation_state = None
        self.navigation_destination = None
        self.navigation_ready = False

        self.last_nav_status_request = 0.0

        # Critical recovery control.
        #
        # We NEVER spam navigation commands during recovery.
        #
        # When recovery begins:
        #
        #   nav_waiting_for_recovery = True
        #
        # When READY appears:
        #
        #   exactly one retry is issued.
        #
        self.nav_waiting_for_recovery = False
        self.nav_retry_command = None

        self.final_approach_goal = None

        # ==========================================================
        # MANIPULATION
        # ==========================================================

        self.manipulation_state = None

        self.grasp_start_time = None
        self.last_grasp_progress_time = None

        # ==========================================================
        # PERCEPTION
        # ==========================================================

        self.latest_books = []
        self.latest_books_time = None

        self.latest_target = None
        self.latest_target_time = None

        self.perception_not_before = None

        # ==========================================================
        # TF
        # ==========================================================

        self.tf_buffer = Buffer()

        self.tf_listener = TransformListener(
            self.tf_buffer,
            self,
        )

        # ==========================================================
        # HEAD SCANNER
        # ==========================================================

        self.head_scanner = HeadScanner(self)

        # ==========================================================
        # SUBSCRIBERS
        # ==========================================================

        self.create_subscription(
            String,
            '/erc/navigation/status',
            self.navigation_status_callback,
            10,
        )

        self.create_subscription(
            String,
            '/erc/manipulation/status',
            self.manipulation_status_callback,
            10,
        )

        self.create_subscription(
            String,
            '/erc/perception/books',
            self.books_callback,
            10,
        )

        self.create_subscription(
            String,
            '/erc/perception/book_target_3d',
            self.book_target_callback,
            10,
        )

        # ==========================================================
        # PUBLISHERS
        # ==========================================================

        self.navigation_command_pub = self.create_publisher(
            String,
            '/erc/navigation/command',
            10,
        )

        self.manipulation_command_pub = self.create_publisher(
            String,
            '/erc/manipulation/command',
            10,
        )

        self.mission_status_pub = self.create_publisher(
            String,
            '/erc/mission/status',
            10,
        )

        # ==========================================================
        # TIMER
        # ==========================================================

        self.timer = self.create_timer(
            0.25,
            self.run_state_machine,
        )

        # ==========================================================
        # STARTUP
        # ==========================================================

        self.get_logger().info(
            '================================================'
        )

        self.get_logger().info(
            'ERC PHASE 8 FINAL AUTONOMOUS MISSION'
        )

        self.get_logger().info(
            'Real RGB-D perception: ENABLED'
        )

        self.get_logger().info(
            'Automatic final alignment: ENABLED'
        )

        self.get_logger().info(
            'Recovery-safe navigation: ENABLED'
        )

        self.get_logger().info(
            'Navigation command spam protection: ENABLED'
        )

        self.get_logger().info(
            'Real MoveIt grasp: ENABLED'
        )

        self.get_logger().info(
            'Manual intervention: NOT REQUIRED'
        )

        self.get_logger().info(
            '================================================'
        )

        self.publish_status(
            'Waiting for navigation system.'
        )

    # ==============================================================
    # STATUS
    # ==============================================================

    def publish_status(
        self,
        message,
        extra=None,
    ):

        payload = {
            'state': self.state,
            'message': message,
            'target_shelf': self.target_shelf,
            'target_color': self.target_color,
            'approach_attempts': self.final_approach_attempts,
        }

        if extra is not None:
            payload['data'] = extra

        msg = String()
        msg.data = json.dumps(payload)

        self.mission_status_pub.publish(msg)

        self.get_logger().info(
            f'{self.state}: {message}'
        )

    # ==============================================================
    # TRANSITION
    # ==============================================================

    def transition(
        self,
        new_state,
        message,
    ):

        self.state = new_state
        self.command_sent = False

        self.state_enter_time = time.monotonic()

        self.publish_status(message)

    def state_age(self):

        return (
            time.monotonic()
            -
            self.state_enter_time
        )

    # ==============================================================
    # COMMANDS
    # ==============================================================

    def send_navigation_command(
        self,
        payload,
    ):

        msg = String()
        msg.data = json.dumps(payload)

        self.navigation_command_pub.publish(msg)

        self.get_logger().info(
            f'NAV COMMAND -> {msg.data}'
        )

    def send_manipulation_command(
        self,
        payload,
    ):

        msg = String()
        msg.data = json.dumps(payload)

        self.manipulation_command_pub.publish(msg)

        self.get_logger().info(
            f'MANIP COMMAND -> {msg.data}'
        )

    # ==============================================================
    # NAVIGATION CALLBACK
    # ==============================================================

    def navigation_status_callback(
        self,
        msg,
    ):

        try:
            data = json.loads(msg.data)

        except Exception:
            return

        state = data.get('state')
        destination = data.get('destination')

        self.navigation_state = state
        self.navigation_destination = destination

        self.get_logger().info(
            (
                'NAV STATUS <- '
                f'{state} ({destination})'
            )
        )

        # ----------------------------------------------------------
        # STARTUP READY
        # ----------------------------------------------------------

        if state == 'READY':

            self.navigation_ready = True

            # ------------------------------------------------------
            # If navigation previously entered recovery, READY means
            # recovery has completed.
            #
            # Retry exactly once.
            # ------------------------------------------------------

            if (
                self.nav_waiting_for_recovery
                and
                self.nav_retry_command is not None
            ):

                retry_command = (
                    self.nav_retry_command.copy()
                )

                self.nav_waiting_for_recovery = False
                self.nav_retry_command = None

                self.get_logger().info(
                    (
                        'Localization recovered. '
                        'Retrying navigation exactly once.'
                    )
                )

                self.send_navigation_command(
                    retry_command
                )

            return

        # ----------------------------------------------------------
        # RECOVERY
        # ----------------------------------------------------------

        if state in (
            'INITIALIZING',
            'LOCALIZATION_BAD',
            'RECOVERING_LOCALIZATION',
        ):

            self.navigation_ready = False

            # Do NOT send another navigation command here.
            return

        # ----------------------------------------------------------
        # Active navigation confirms recovery retry succeeded.
        # ----------------------------------------------------------

        if state in (
            'NAVIGATING',
            'ACCEPTED',
        ):

            self.nav_waiting_for_recovery = False
            self.nav_retry_command = None

    # ==============================================================
    # REGISTER NAVIGATION RECOVERY
    # ==============================================================

    def register_navigation_recovery(
        self,
        command,
    ):

        if self.nav_waiting_for_recovery:
            return

        self.nav_waiting_for_recovery = True

        self.nav_retry_command = (
            command.copy()
        )

        self.get_logger().warn(
            (
                'Navigation localization recovery detected. '
                'Waiting for READY before retrying.'
            )
        )

    # ==============================================================
    # MANIPULATION CALLBACK
    # ==============================================================

    def manipulation_status_callback(
        self,
        msg,
    ):

        try:
            data = json.loads(msg.data)

        except Exception:
            return

        state = data.get('state')

        self.manipulation_state = state

        self.get_logger().info(
            f'MANIP STATUS <- {state}'
        )

        progress_states = (
            'GRASP_REQUESTED',
            'OPENING_GRIPPER',
            'GRIPPER_OPENED',
            'MOVING_TO_PREGRASP',
            'APPROACHING_BOOK',
            'CLOSING_GRIPPER',
            'GRIPPER_CLOSED',
            'LIFTING_BOOK',
            'GRASP_SUCCEEDED',
        )

        if state in progress_states:

            self.last_grasp_progress_time = (
                time.monotonic()
            )

    # ==============================================================
    # BOOK CALLBACK
    # ==============================================================

    def books_callback(
        self,
        msg,
    ):

        try:
            data = json.loads(msg.data)

        except Exception:
            return

        books = data.get(
            'books',
            []
        )

        if not isinstance(
            books,
            list,
        ):
            return

        self.latest_books = books
        self.latest_books_time = time.monotonic()

        for book in books:

            color = str(
                book.get(
                    'color',
                    ''
                )
            ).upper()

            if color == self.target_color:

                self.get_logger().info(
                    (
                        'REAL BLUE BOOK FOUND at pixel '
                        f'({book.get("center_x")},'
                        f'{book.get("center_y")})'
                    )
                )

                return

    # ==============================================================
    # TARGET CALLBACK
    # ==============================================================

    def book_target_callback(
        self,
        msg,
    ):

        try:
            target = json.loads(msg.data)

        except Exception:
            return

        color = str(
            target.get(
                'color',
                ''
            )
        ).upper()

        if color != self.target_color:
            return

        self.latest_target = target
        self.latest_target_time = time.monotonic()

        self.get_logger().info(
            (
                'REAL BLUE 3D TARGET <- '
                f'x={target.get("x")}, '
                f'y={target.get("y")}, '
                f'z={target.get("z")}, '
                f'reachable={target.get("reachable")}'
            )
        )

    # ==============================================================
    # PERCEPTION RESET
    # ==============================================================

    def reset_perception_window(self):

        self.perception_not_before = (
            time.monotonic()
        )

        self.latest_books = []
        self.latest_books_time = None

        self.latest_target = None
        self.latest_target_time = None

    # ==============================================================
    # BOOK FRESHNESS
    # ==============================================================

    def fresh_blue_book_visible(self):

        if self.latest_books_time is None:
            return False

        if self.perception_not_before is None:
            return False

        if (
            self.latest_books_time
            <=
            self.perception_not_before
        ):
            return False

        if (
            time.monotonic()
            -
            self.latest_books_time
            >
            self.max_book_age
        ):
            return False

        for book in self.latest_books:

            if str(
                book.get(
                    'color',
                    ''
                )
            ).upper() == self.target_color:

                return True

        return False

    # ==============================================================
    # TARGET FRESHNESS
    # ==============================================================

    def fresh_target_available(self):

        if self.latest_target is None:
            return False

        if self.latest_target_time is None:
            return False

        if self.perception_not_before is None:
            return False

        if (
            self.latest_target_time
            <=
            self.perception_not_before
        ):
            return False

        return (
            time.monotonic()
            -
            self.latest_target_time
            <=
            self.max_target_age
        )

    # ==============================================================
    # TARGET XYZ
    # ==============================================================

    def target_xyz(self):

        if not self.fresh_target_available():
            return None

        try:

            x = float(
                self.latest_target['x']
            )

            y = float(
                self.latest_target['y']
            )

            z = float(
                self.latest_target['z']
            )

        except Exception:
            return None

        if not all(
            math.isfinite(value)
            for value in (
                x,
                y,
                z,
            )
        ):
            return None

        return (
            x,
            y,
            z,
        )

    # ==============================================================
    # DIRECT GRASP
    # ==============================================================

    def target_in_grasp_workspace(self):

        xyz = self.target_xyz()

        if xyz is None:
            return False

        if str(
            self.latest_target.get(
                'frame_id',
                ''
            )
        ) != 'base_link':
            return False

        if not bool(
            self.latest_target.get(
                'reachable',
                False
            )
        ):
            return False

        x, y, z = xyz

        return (
            self.min_grasp_x
            <= x
            <= self.max_grasp_x

            and

            self.min_grasp_y
            <= y
            <= self.max_grasp_y

            and

            self.min_grasp_z
            <= z
            <= self.max_grasp_z
        )

    # ==============================================================
    # APPROACH VALIDATION
    # ==============================================================

    def target_can_be_approached(self):

        xyz = self.target_xyz()

        if xyz is None:
            return False

        if str(
            self.latest_target.get(
                'frame_id',
                ''
            )
        ) != 'base_link':
            return False

        x, y, z = xyz

        return (
            self.min_approach_x
            <= x
            <= self.max_approach_x

            and

            abs(y)
            <= self.max_approach_abs_y

            and

            self.min_approach_z
            <= z
            <= self.max_approach_z
        )

    # ==============================================================
    # QUATERNION -> YAW
    # ==============================================================

    def quaternion_to_yaw(
        self,
        x,
        y,
        z,
        w,
    ):

        siny_cosp = (
            2.0
            *
            (
                w * z
                +
                x * y
            )
        )

        cosy_cosp = (
            1.0
            -
            2.0
            *
            (
                y * y
                +
                z * z
            )
        )

        return math.atan2(
            siny_cosp,
            cosy_cosp,
        )

    # ==============================================================
    # FINAL APPROACH CALCULATION
    # ==============================================================

    def calculate_final_approach_goal(self):

        xyz = self.target_xyz()

        if xyz is None:
            return None

        target_x_base, target_y_base, _ = xyz

        try:

            transform = (
                self.tf_buffer.lookup_transform(
                    'map',
                    'base_link',
                    Time(),
                    timeout=Duration(
                        seconds=2.0
                    ),
                )
            )

        except TransformException as exc:

            self.get_logger().warn(
                f'Final approach TF unavailable: {exc}'
            )

            return None

        base_x = float(
            transform.transform.translation.x
        )

        base_y = float(
            transform.transform.translation.y
        )

        rotation = (
            transform.transform.rotation
        )

        base_yaw = self.quaternion_to_yaw(
            rotation.x,
            rotation.y,
            rotation.z,
            rotation.w,
        )

        cos_yaw = math.cos(base_yaw)
        sin_yaw = math.sin(base_yaw)

        target_map_x = (
            base_x
            +
            cos_yaw * target_x_base
            -
            sin_yaw * target_y_base
        )

        target_map_y = (
            base_y
            +
            sin_yaw * target_x_base
            +
            cos_yaw * target_y_base
        )

        desired_dx = (
            cos_yaw * self.preferred_grasp_x
            -
            sin_yaw * self.preferred_grasp_y
        )

        desired_dy = (
            sin_yaw * self.preferred_grasp_x
            +
            cos_yaw * self.preferred_grasp_y
        )

        goal_x = (
            target_map_x
            -
            desired_dx
        )

        goal_y = (
            target_map_y
            -
            desired_dy
        )

        goal_yaw = base_yaw

        movement_distance = math.hypot(
            goal_x - base_x,
            goal_y - base_y,
        )

        if (
            movement_distance
            >
            self.max_single_approach_move
        ):

            self.get_logger().warn(
                (
                    'Final alignment rejected: '
                    f'{movement_distance:.3f} m '
                    'is larger than safety limit.'
                )
            )

            return None

        self.get_logger().info(
            '================================================'
        )

        self.get_logger().info(
            'AUTOMATIC FINAL ALIGNMENT'
        )

        self.get_logger().info(
            (
                'Target now: '
                f'X={target_x_base:.3f}, '
                f'Y={target_y_base:.3f}'
            )
        )

        self.get_logger().info(
            (
                'Desired: '
                f'X={self.preferred_grasp_x:.3f}, '
                f'Y={self.preferred_grasp_y:.3f}'
            )
        )

        self.get_logger().info(
            (
                'Nav2 goal: '
                f'X={goal_x:.3f}, '
                f'Y={goal_y:.3f}, '
                f'Yaw={goal_yaw:.3f}'
            )
        )

        self.get_logger().info(
            (
                'Movement: '
                f'{movement_distance:.3f} m'
            )
        )

        self.get_logger().info(
            '================================================'
        )

        return {
            'x': goal_x,
            'y': goal_y,
            'yaw': goal_yaw,
        }

    # ==============================================================
    # NAV STATUS REQUEST
    # ==============================================================

    def request_navigation_status(self):

        self.send_navigation_command(
            {
                'action': 'STATUS'
            }
        )

        self.last_nav_status_request = (
            time.monotonic()
        )

    # ==============================================================
    # NAV RECOVERY CHECK
    # ==============================================================

    def navigation_is_recovering(self):

        # READY is intentionally NOT here.
        return self.navigation_state in (
            'INITIALIZING',
            'LOCALIZATION_BAD',
            'RECOVERING_LOCALIZATION',
        )

    # ==============================================================
    # GRASP WATCHDOG
    # ==============================================================

    def grasp_stalled(self):

        if self.last_grasp_progress_time is None:
            return False

        return (
            time.monotonic()
            -
            self.last_grasp_progress_time
            >
            self.grasp_stall_timeout
        )

    def grasp_absolute_timeout_reached(self):

        if self.grasp_start_time is None:
            return False

        return (
            time.monotonic()
            -
            self.grasp_start_time
            >
            self.grasp_absolute_timeout
        )

    # ==============================================================
    # STATE MACHINE
    # ==============================================================

    def run_state_machine(self):

        if self.state in (
            'SEARCH_FOR_BLUE',
            'WAIT_AFTER_APPROACH',
        ):

            self.head_scanner.tick()

        # ==========================================================
        # WAIT SYSTEM
        # ==========================================================

        if self.state == 'WAIT_FOR_SYSTEM':

            if (
                time.monotonic()
                -
                self.last_nav_status_request
                >
                2.0
            ):

                self.request_navigation_status()

            if self.navigation_ready:

                self.transition(
                    'NAVIGATE_TO_SHELF',
                    'Navigation ready. Starting Phase 8.',
                )

                return

            if self.state_age() > self.startup_timeout:

                self.transition(
                    'FAILED',
                    'Navigation startup timed out.',
                )

            return

        # ==========================================================
        # SHELF
        # ==========================================================

        if self.state == 'NAVIGATE_TO_SHELF':

            if not self.command_sent:

                command = {
                    'action': 'GO_TO_SHELF',
                    'shelf': self.target_shelf,
                }

                self.navigation_state = None

                self.send_navigation_command(
                    command
                )

                self.command_sent = True

                self.transition(
                    'WAIT_FOR_SHELF',
                    'Navigating to shelf.',
                )

            return

        # ==========================================================
        # WAIT SHELF
        # ==========================================================

        if self.state == 'WAIT_FOR_SHELF':

            if self.navigation_state == 'SUCCEEDED':

                self.nav_waiting_for_recovery = False
                self.nav_retry_command = None

                self.reset_perception_window()

                self.head_scanner.start()

                self.transition(
                    'SEARCH_FOR_BLUE',
                    (
                        'Shelf reached. '
                        'Searching for real BLUE book.'
                    ),
                )

                return

            if self.navigation_is_recovering():

                self.register_navigation_recovery(
                    {
                        'action': 'GO_TO_SHELF',
                        'shelf': self.target_shelf,
                    }
                )

                return

            if self.navigation_state in (
                'FAILED',
                'NAV2_UNAVAILABLE',
                'CANCELED',
                'REJECTED',
            ):

                self.transition(
                    'FAILED',
                    (
                        'Shelf navigation failed: '
                        f'{self.navigation_state}'
                    ),
                )

                return

            if self.state_age() > self.navigation_timeout:

                self.transition(
                    'FAILED',
                    'Shelf navigation timed out.',
                )

            return

        # ==========================================================
        # SEARCH
        # ==========================================================

        if self.state == 'SEARCH_FOR_BLUE':

            if (
                self.fresh_blue_book_visible()
                and
                self.fresh_target_available()
            ):

                self.transition(
                    'EVALUATE_TARGET',
                    'Fresh real BLUE RGB-D target available.',
                )

                return

            if self.state_age() > self.search_timeout:

                self.head_scanner.stop()

                self.transition(
                    'FAILED',
                    'BLUE search timed out.',
                )

            return

        # ==========================================================
        # EVALUATE
        # ==========================================================

        if self.state == 'EVALUATE_TARGET':

            xyz = self.target_xyz()

            if xyz is None:

                self.transition(
                    'SEARCH_FOR_BLUE',
                    'Waiting for fresh RGB-D target.',
                )

                return

            x, y, z = xyz

            self.get_logger().info(
                (
                    'TARGET EVALUATION: '
                    f'X={x:.3f}, '
                    f'Y={y:.3f}, '
                    f'Z={z:.3f}'
                )
            )

            if self.target_in_grasp_workspace():

                self.head_scanner.stop()

                self.transition(
                    'GRASP_BOOK',
                    (
                        'Target inside safe '
                        'grasp workspace.'
                    ),
                )

                return

            if self.target_can_be_approached():

                if (
                    self.final_approach_attempts
                    >=
                    self.max_final_approach_attempts
                ):

                    self.head_scanner.stop()

                    self.transition(
                        'FAILED',
                        (
                            'Maximum final alignment '
                            'attempts reached.'
                        ),
                    )

                    return

                goal = self.calculate_final_approach_goal()

                if goal is None:

                    self.transition(
                        'SEARCH_FOR_BLUE',
                        (
                            'Unable to calculate safe alignment. '
                            'Getting new target.'
                        ),
                    )

                    return

                self.final_approach_goal = goal

                self.final_approach_attempts += 1

                self.head_scanner.stop()

                self.transition(
                    'FINAL_APPROACH',
                    (
                        'Correcting base position '
                        'for grasp.'
                    ),
                )

                return

            self.transition(
                'SEARCH_FOR_BLUE',
                (
                    'Target outside safe approach envelope. '
                    'Continuing search.'
                ),
            )

            return

        # ==========================================================
        # FINAL APPROACH
        # ==========================================================

        if self.state == 'FINAL_APPROACH':

            if not self.command_sent:

                goal = self.final_approach_goal

                command = {
                    'action': 'GO_TO_POSE',
                    'name': 'FINAL_APPROACH',
                    'x': goal['x'],
                    'y': goal['y'],
                    'yaw': goal['yaw'],
                }

                self.navigation_state = None

                self.send_navigation_command(
                    command
                )

                self.command_sent = True

                self.transition(
                    'WAIT_FOR_FINAL_APPROACH',
                    'Performing final base alignment.',
                )

            return

        # ==========================================================
        # WAIT FINAL APPROACH
        # ==========================================================

        if self.state == 'WAIT_FOR_FINAL_APPROACH':

            if self.navigation_state == 'SUCCEEDED':

                self.nav_waiting_for_recovery = False
                self.nav_retry_command = None

                self.reset_perception_window()

                self.head_scanner.start()

                self.transition(
                    'WAIT_AFTER_APPROACH',
                    (
                        'Alignment complete. '
                        'Re-detecting BLUE book.'
                    ),
                )

                return

            if self.navigation_is_recovering():

                goal = self.final_approach_goal

                self.register_navigation_recovery(
                    {
                        'action': 'GO_TO_POSE',
                        'name': 'FINAL_APPROACH',
                        'x': goal['x'],
                        'y': goal['y'],
                        'yaw': goal['yaw'],
                    }
                )

                return

            if self.navigation_state in (
                'FAILED',
                'NAV2_UNAVAILABLE',
                'CANCELED',
                'REJECTED',
            ):

                self.transition(
                    'FAILED',
                    (
                        'Final alignment failed: '
                        f'{self.navigation_state}'
                    ),
                )

                return

            if self.state_age() > self.navigation_timeout:

                self.transition(
                    'FAILED',
                    'Final alignment timed out.',
                )

            return

        # ==========================================================
        # AFTER APPROACH
        # ==========================================================

        if self.state == 'WAIT_AFTER_APPROACH':

            if (
                self.fresh_blue_book_visible()
                and
                self.fresh_target_available()
            ):

                self.transition(
                    'EVALUATE_TARGET',
                    (
                        'Fresh target received '
                        'after alignment.'
                    ),
                )

                return

            if self.state_age() > self.search_timeout:

                self.head_scanner.stop()

                self.transition(
                    'FAILED',
                    (
                        'Could not re-detect BLUE '
                        'after alignment.'
                    ),
                )

            return

        # ==========================================================
        # GRASP
        # ==========================================================

        if self.state == 'GRASP_BOOK':

            if not self.command_sent:

                self.manipulation_state = None

                now = time.monotonic()

                self.grasp_start_time = now
                self.last_grasp_progress_time = now

                self.send_manipulation_command(
                    {
                        'action': 'GRASP_BOOK',
                        'color': self.target_color,
                    }
                )

                self.command_sent = True

                self.transition(
                    'WAIT_FOR_GRASP',
                    'Executing real MoveIt grasp.',
                )

            return

        # ==========================================================
        # WAIT GRASP
        # ==========================================================

        if self.state == 'WAIT_FOR_GRASP':

            if self.manipulation_state == 'GRASP_SUCCEEDED':

                self.transition(
                    'NAVIGATE_TO_BIN',
                    'BLUE book grasp succeeded.',
                )

                return

            if self.manipulation_state in (
                'MOVEIT_FAILED',
                'MOVEIT_ERROR',
                'GRIPPER_ERROR',
                'TARGET_UNSAFE',
                'TARGET_UNREACHABLE',
                'TF_ERROR',
            ):

                self.transition(
                    'FAILED',
                    (
                        'Manipulation failed: '
                        f'{self.manipulation_state}'
                    ),
                )

                return

            if self.grasp_stalled():

                self.transition(
                    'FAILED',
                    'Manipulation stalled.',
                )

                return

            if self.grasp_absolute_timeout_reached():

                self.transition(
                    'FAILED',
                    'Manipulation emergency timeout.',
                )

            return

        # ==========================================================
        # BIN
        # ==========================================================

        if self.state == 'NAVIGATE_TO_BIN':

            if not self.command_sent:

                self.navigation_state = None

                self.send_navigation_command(
                    {
                        'action': 'GO_TO_BIN'
                    }
                )

                self.command_sent = True

                self.transition(
                    'WAIT_FOR_BIN',
                    'Navigating to BIN.',
                )

            return

        if self.state == 'WAIT_FOR_BIN':

            if self.navigation_state == 'SUCCEEDED':

                self.nav_waiting_for_recovery = False
                self.nav_retry_command = None

                self.transition(
                    'RELEASE_BOOK',
                    'BIN reached.',
                )

                return

            if self.navigation_is_recovering():

                self.register_navigation_recovery(
                    {
                        'action': 'GO_TO_BIN'
                    }
                )

                return

            if self.navigation_state in (
                'FAILED',
                'NAV2_UNAVAILABLE',
                'CANCELED',
                'REJECTED',
            ):

                self.transition(
                    'FAILED',
                    (
                        'BIN navigation failed: '
                        f'{self.navigation_state}'
                    ),
                )

                return

            if self.state_age() > self.navigation_timeout:

                self.transition(
                    'FAILED',
                    'BIN navigation timed out.',
                )

            return

        # ==========================================================
        # RELEASE
        # ==========================================================

        if self.state == 'RELEASE_BOOK':

            if not self.command_sent:

                self.manipulation_state = None

                self.send_manipulation_command(
                    {
                        'action': 'OPEN_GRIPPER'
                    }
                )

                self.command_sent = True

                self.transition(
                    'WAIT_FOR_RELEASE',
                    'Opening gripper at BIN.',
                )

            return

        if self.state == 'WAIT_FOR_RELEASE':

            if self.manipulation_state == 'GRIPPER_OPENED':

                self.transition(
                    'RETURN_HOME',
                    'Book released.',
                )

                return

            if self.manipulation_state == 'GRIPPER_ERROR':

                self.transition(
                    'FAILED',
                    'Gripper release failed.',
                )

                return

            if self.state_age() > self.release_timeout:

                self.transition(
                    'FAILED',
                    'Gripper release timed out.',
                )

            return

        # ==========================================================
        # HOME
        # ==========================================================

        if self.state == 'RETURN_HOME':

            if not self.command_sent:

                self.navigation_state = None

                self.send_navigation_command(
                    {
                        'action': 'GO_HOME'
                    }
                )

                self.command_sent = True

                self.transition(
                    'WAIT_FOR_HOME',
                    'Returning HOME.',
                )

            return

        if self.state == 'WAIT_FOR_HOME':

            if self.navigation_state == 'SUCCEEDED':

                self.nav_waiting_for_recovery = False
                self.nav_retry_command = None

                self.head_scanner.center()

                self.transition(
                    'MISSION_SUCCEEDED',
                    (
                        'PHASE 8 COMPLETE: '
                        'real perception -> alignment -> '
                        'grasp -> BIN -> HOME.'
                    ),
                )

                return

            if self.navigation_is_recovering():

                self.register_navigation_recovery(
                    {
                        'action': 'GO_HOME'
                    }
                )

                return

            if self.navigation_state in (
                'FAILED',
                'NAV2_UNAVAILABLE',
                'CANCELED',
                'REJECTED',
            ):

                self.transition(
                    'FAILED',
                    (
                        'HOME navigation failed: '
                        f'{self.navigation_state}'
                    ),
                )

                return

            if self.state_age() > self.navigation_timeout:

                self.transition(
                    'FAILED',
                    'HOME navigation timed out.',
                )

            return


def main(args=None):

    rclpy.init(args=args)

    node = MissionNode()

    try:
        rclpy.spin(node)

    except KeyboardInterrupt:
        pass

    finally:

        try:
            node.head_scanner.stop()
        except Exception:
            pass

        node.destroy_node()

        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
