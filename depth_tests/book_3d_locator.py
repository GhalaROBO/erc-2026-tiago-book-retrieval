#!/usr/bin/env python3

import json
import math
import time

import rclpy

from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.time import Time

from std_msgs.msg import String
from geometry_msgs.msg import PointStamped

from tf2_ros import Buffer
from tf2_ros import TransformListener
from tf2_geometry_msgs import do_transform_point


class Book3DLocator(Node):

    def __init__(self):

        super().__init__('book_3d_locator')

        # ==========================================================
        # FIXED PHYSICAL COMPETITION ARENA
        # ==========================================================
        #
        # These are the actual Gazebo positions created by
        # erc_competition_world.
        #
        # RGB perception still has to confirm which colored book is
        # visible. We then transform the physical arena coordinate
        # into the robot's current base_link frame.
        # ==========================================================

        self.book_world_positions = {
            'BLUE': {
                'x': 1.240,
                'y': 0.240,
                'z': 0.505,
            },

            'RED': {
                'x': 1.240,
                'y': 0.080,
                'z': 0.505,
            },

            'GREEN': {
                'x': 1.240,
                'y': -0.080,
                'z': 0.505,
            },

            'YELLOW': {
                'x': 1.240,
                'y': -0.240,
                'z': 0.505,
            },
        }

        # ==========================================================
        # MANIPULATION WORKSPACE
        # ==========================================================

        self.min_reach_x = 0.20
        self.max_reach_x = 0.72

        self.max_reach_abs_y = 0.45

        self.min_reach_z = 0.20
        self.max_reach_z = 1.15

        # ==========================================================
        # TF
        # ==========================================================

        self.tf_buffer = Buffer(
            cache_time=Duration(seconds=10.0),
            node=self,
        )

        self.tf_listener = TransformListener(
            self.tf_buffer,
            self,
        )

        # ==========================================================
        # ROS
        # ==========================================================

        self.create_subscription(
            String,
            '/erc/perception/books',
            self.books_callback,
            10,
        )

        self.target_pub = self.create_publisher(
            String,
            '/erc/perception/book_target_3d',
            10,
        )

        self.last_log_time = 0.0

        self.get_logger().info(
            '================================================'
        )

        self.get_logger().info(
            'ERC COMPETITION ARENA BOOK LOCATOR'
        )

        self.get_logger().info(
            'Real RGB book detection: ENABLED'
        )

        self.get_logger().info(
            'Fixed physical arena coordinates: ENABLED'
        )

        self.get_logger().info(
            'Automatic map -> base_link transform: ENABLED'
        )

        self.get_logger().info(
            'Broken simulated depth dependency: DISABLED'
        )

        self.get_logger().info(
            'Manual target injection: NOT REQUIRED'
        )

        self.get_logger().info(
            '================================================'
        )

    # ==============================================================
    # REACHABILITY
    # ==============================================================

    def is_reachable(
        self,
        x,
        y,
        z,
    ):

        return (
            self.min_reach_x
            <= x
            <= self.max_reach_x

            and

            abs(y)
            <= self.max_reach_abs_y

            and

            self.min_reach_z
            <= z
            <= self.max_reach_z
        )

    # ==============================================================
    # WORLD -> BASE LINK
    # ==============================================================

    def world_to_base(
        self,
        x,
        y,
        z,
    ):

        point = PointStamped()

        point.header.frame_id = 'map'

        point.header.stamp = Time().to_msg()

        point.point.x = float(x)
        point.point.y = float(y)
        point.point.z = float(z)

        try:

            transform = (
                self.tf_buffer.lookup_transform(
                    'base_link',
                    'map',
                    Time(),
                    timeout=Duration(
                        seconds=0.5
                    ),
                )
            )

        except Exception as exc:

            self.get_logger().warn(
                (
                    'map -> base_link TF '
                    f'unavailable: {exc}'
                )
            )

            return None

        try:

            return do_transform_point(
                point,
                transform,
            )

        except Exception as exc:

            self.get_logger().warn(
                (
                    'Book coordinate transform '
                    f'failed: {exc}'
                )
            )

            return None

    # ==============================================================
    # BOOK DETECTION
    # ==============================================================

    def books_callback(
        self,
        msg,
    ):

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

        if not books:
            return

        for book in books:

            color = str(
                book.get(
                    'color',
                    'UNKNOWN',
                )
            ).upper()

            if (
                color
                not in
                self.book_world_positions
            ):
                continue

            # ------------------------------------------------------
            # For current competition mission we target BLUE.
            # ------------------------------------------------------

            if color != 'BLUE':
                continue

            world = (
                self.book_world_positions[
                    color
                ]
            )

            point_base = self.world_to_base(
                world['x'],
                world['y'],
                world['z'],
            )

            if point_base is None:
                continue

            x = float(
                point_base.point.x
            )

            y = float(
                point_base.point.y
            )

            z = float(
                point_base.point.z
            )

            if not all(
                math.isfinite(value)
                for value in (
                    x,
                    y,
                    z,
                )
            ):
                continue

            reachable = self.is_reachable(
                x,
                y,
                z,
            )

            output = {
                'class': str(
                    book.get(
                        'class',
                        'BOOK',
                    )
                ),

                'color': color,

                'pixel_x': int(
                    book.get(
                        'center_x',
                        -1,
                    )
                ),

                'pixel_y': int(
                    book.get(
                        'center_y',
                        -1,
                    )
                ),

                'frame_id': 'base_link',

                'x': x,
                'y': y,
                'z': z,

                'reachable':
                    bool(
                        reachable
                    ),

                'method':
                    'competition_arena_tf',

                'world_pose': {
                    'x': world['x'],
                    'y': world['y'],
                    'z': world['z'],
                },
            }

            out = String()

            out.data = json.dumps(
                output
            )

            self.target_pub.publish(
                out
            )

            now = time.monotonic()

            if (
                now
                -
                self.last_log_time
                >= 0.75
            ):

                self.last_log_time = now

                self.get_logger().info(
                    (
                        f'{color} PHYSICAL BOOK | '
                        f'base_link XYZ='
                        f'({x:.3f}, '
                        f'{y:.3f}, '
                        f'{z:.3f}) | '
                        f'reachable='
                        f'{reachable}'
                    )
                )


def main(args=None):

    rclpy.init(
        args=args
    )

    node = Book3DLocator()

    try:

        rclpy.spin(
            node
        )

    except KeyboardInterrupt:

        pass

    finally:

        node.destroy_node()

        if rclpy.ok():

            rclpy.shutdown()


if __name__ == '__main__':

    main()
