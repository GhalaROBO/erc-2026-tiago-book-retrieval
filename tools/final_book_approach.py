#!/usr/bin/env python3

import json
import math
import time

import rclpy

from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.time import Time

from geometry_msgs.msg import PointStamped
from geometry_msgs.msg import Twist
from std_msgs.msg import String

from tf2_ros import Buffer
from tf2_ros import TransformListener
from tf2_geometry_msgs import do_transform_point


class FinalBookApproach(Node):

    def __init__(self):

        super().__init__('erc_final_book_approach')

        # ==========================================================
        # FIXED PHYSICAL BLUE BOOK POSITION IN COMPETITION ARENA
        # ==========================================================

        self.book_map_x = 1.240
        self.book_map_y = 0.240
        self.book_map_z = 0.505

        # ==========================================================
        # DESIRED FINAL GRASP POSITION
        # ==========================================================

        self.target_x = 0.55

        self.x_tolerance = 0.035

        # The arm can handle a modest lateral offset.
        self.target_y = 0.10
        self.y_tolerance = 0.035

        # ==========================================================
        # SPEED
        # ==========================================================

        self.max_forward_speed = 0.24
        self.medium_forward_speed = 0.16
        self.min_forward_speed = 0.055

        self.max_lateral_speed = 0.09

        self.kp_x = 0.55
        self.kp_y = 0.60

        # ==========================================================
        # SAFETY
        # ==========================================================

        self.max_runtime = 20.0

        self.start_time = time.monotonic()

        self.blue_seen = False

        self.finished = False

        self.last_progress_log = 0.0

        # ==========================================================
        # TF
        # ==========================================================

        self.tf_buffer = Buffer(
            cache_time=Duration(seconds=10.0)
        )

        self.tf_listener = TransformListener(
            self.tf_buffer,
            self,
        )

        # ==========================================================
        # ROS
        # ==========================================================

        self.cmd_pub = self.create_publisher(
            Twist,
            '/assisted_vel',
            10,
        )

        self.books_sub = self.create_subscription(
            String,
            '/erc/perception/books',
            self.books_callback,
            10,
        )

        self.timer = self.create_timer(
            0.05,
            self.control_loop,
        )

        self.get_logger().info(
            '================================================'
        )

        self.get_logger().info(
            'ERC FINAL COMPETITION BOOK APPROACH'
        )

        self.get_logger().info(
            'Real BLUE acquisition required: YES'
        )

        self.get_logger().info(
            'Post-acquisition tracking: MAP/TF'
        )

        self.get_logger().info(
            'Camera-loss tolerance: ENABLED'
        )

        self.get_logger().info(
            'Control topic: /assisted_vel'
        )

        self.get_logger().info(
            f'Final X target: {self.target_x:.3f} m'
        )

        self.get_logger().info(
            f'Final Y target: {self.target_y:.3f} m'
        )

        self.get_logger().info(
            '================================================'
        )

    # ==============================================================
    # BLUE ACQUISITION
    # ==============================================================

    def books_callback(self, msg):

        try:

            payload = json.loads(
                msg.data
            )

        except Exception:
            return

        books = payload.get(
            'books',
            [],
        )

        for book in books:

            color = str(
                book.get(
                    'color',
                    '',
                )
            ).upper()

            if color == 'BLUE':

                if not self.blue_seen:

                    self.blue_seen = True

                    self.get_logger().info(
                        'BLUE ACQUIRED - switching to TF tracking.'
                    )

                return

    # ==============================================================
    # BOOK POSITION FROM MAP/TF
    # ==============================================================

    def get_book_in_base(self):

        point = PointStamped()

        point.header.frame_id = 'map'

        point.point.x = self.book_map_x
        point.point.y = self.book_map_y
        point.point.z = self.book_map_z

        try:

            transform = (
                self.tf_buffer.lookup_transform(
                    'base_link',
                    'map',
                    Time(),
                    timeout=Duration(seconds=0.20),
                )
            )

        except Exception:
            return None

        try:

            transformed = do_transform_point(
                point,
                transform,
            )

        except Exception:
            return None

        x = float(
            transformed.point.x
        )

        y = float(
            transformed.point.y
        )

        z = float(
            transformed.point.z
        )

        if not all(
            math.isfinite(v)
            for v in (x, y, z)
        ):
            return None

        return {
            'x': x,
            'y': y,
            'z': z,
        }

    # ==============================================================
    # SAFE PUBLISH
    # ==============================================================

    def publish_twist(self, msg):

        if not rclpy.ok():
            return

        try:

            self.cmd_pub.publish(
                msg
            )

        except Exception:
            pass

    # ==============================================================
    # STOP
    # ==============================================================

    def stop_robot(self):

        stop = Twist()

        if not rclpy.ok():
            return

        for _ in range(3):

            self.publish_twist(
                stop
            )

    # ==============================================================
    # FINISH
    # ==============================================================

    def finish(self, target):

        if self.finished:
            return

        self.finished = True

        self.stop_robot()

        self.get_logger().info(
            '================================================'
        )

        self.get_logger().info(
            'FINAL APPROACH COMPLETE'
        )

        self.get_logger().info(
            (
                'FINAL BLUE TARGET | '
                f'X={target["x"]:.3f} '
                f'Y={target["y"]:.3f} '
                f'Z={target["z"]:.3f}'
            )
        )

        self.get_logger().info(
            'ROBOT STOPPED AT GRASP POSITION'
        )

        self.get_logger().info(
            '================================================'
        )

        # Shut down on the next timer iteration.
        self.create_timer(
            0.25,
            self.request_shutdown,
        )

    def request_shutdown(self):

        if rclpy.ok():

            try:

                rclpy.shutdown()

            except Exception:
                pass

    # ==============================================================
    # CONTROL
    # ==============================================================

    def control_loop(self):

        if self.finished:
            return

        now = time.monotonic()

        # ----------------------------------------------------------
        # GLOBAL SAFETY TIMEOUT
        # ----------------------------------------------------------

        if (
            now - self.start_time
            >
            self.max_runtime
        ):

            self.stop_robot()

            self.get_logger().error(
                'FINAL APPROACH TIMEOUT - ROBOT STOPPED'
            )

            self.finished = True

            return

        # ----------------------------------------------------------
        # REQUIRE REAL BLUE ACQUISITION ONCE
        # ----------------------------------------------------------

        if not self.blue_seen:

            self.publish_twist(
                Twist()
            )

            if (
                now - self.last_progress_log
                >
                1.0
            ):

                self.last_progress_log = now

                self.get_logger().info(
                    'Waiting for real BLUE detection...'
                )

            return

        # ----------------------------------------------------------
        # AFTER BLUE IS ACQUIRED, USE MAP/TF.
        #
        # We no longer depend on continuous camera visibility.
        # ----------------------------------------------------------

        target = self.get_book_in_base()

        if target is None:

            self.publish_twist(
                Twist()
            )

            if (
                now - self.last_progress_log
                >
                1.0
            ):

                self.last_progress_log = now

                self.get_logger().warn(
                    'Waiting for map -> base_link TF.'
                )

            return

        x = target['x']
        y = target['y']

        error_x = (
            x - self.target_x
        )

        error_y = (
            y - self.target_y
        )

        # ----------------------------------------------------------
        # SUCCESS
        # ----------------------------------------------------------

        if (
            abs(error_x)
            <=
            self.x_tolerance
            and
            abs(error_y)
            <=
            self.y_tolerance
        ):

            self.finish(
                target
            )

            return

        cmd = Twist()

        # ----------------------------------------------------------
        # FORWARD
        # ----------------------------------------------------------

        if error_x > self.x_tolerance:

            if error_x > 0.40:

                vx = (
                    self.max_forward_speed
                )

            elif error_x > 0.20:

                vx = (
                    self.medium_forward_speed
                )

            else:

                vx = (
                    self.kp_x
                    *
                    error_x
                )

                vx = max(
                    self.min_forward_speed,
                    vx,
                )

                vx = min(
                    self.medium_forward_speed,
                    vx,
                )

            cmd.linear.x = float(
                vx
            )

        elif error_x < -self.x_tolerance:

            cmd.linear.x = float(
                max(
                    -0.07,
                    self.kp_x
                    *
                    error_x,
                )
            )

        # ----------------------------------------------------------
        # LATERAL
        # ----------------------------------------------------------

        if abs(error_y) > self.y_tolerance:

            vy = (
                self.kp_y
                *
                error_y
            )

            vy = max(
                -self.max_lateral_speed,
                min(
                    self.max_lateral_speed,
                    vy,
                ),
            )

            cmd.linear.y = float(
                vy
            )

        self.publish_twist(
            cmd
        )

        if (
            now - self.last_progress_log
            >= 0.5
        ):

            self.last_progress_log = now

            self.get_logger().info(
                (
                    'APPROACH | '
                    f'X={x:.3f} '
                    f'Y={y:.3f} '
                    f'Z={target["z"]:.3f} | '
                    f'vx={cmd.linear.x:.3f} '
                    f'vy={cmd.linear.y:.3f}'
                )
            )


def main(args=None):

    rclpy.init(
        args=args
    )

    node = FinalBookApproach()

    try:

        rclpy.spin(
            node
        )

    except KeyboardInterrupt:

        if rclpy.ok():

            node.stop_robot()

    finally:

        if rclpy.ok():

            node.stop_robot()

        try:

            node.destroy_node()

        except Exception:
            pass

        if rclpy.ok():

            try:

                rclpy.shutdown()

            except Exception:
                pass


if __name__ == '__main__':

    main()
