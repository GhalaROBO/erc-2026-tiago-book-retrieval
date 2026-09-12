#!/usr/bin/env python3

import math

import rclpy
from rclpy.node import Node
from rclpy.qos import (
    QoSProfile,
    QoSReliabilityPolicy,
    QoSHistoryPolicy,
    QoSDurabilityPolicy,
)

from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import PointStamped
from cv_bridge import CvBridge

from tf2_ros import Buffer, TransformListener
from tf2_geometry_msgs import do_transform_point


class DepthPointToBase(Node):

    def __init__(self):
        super().__init__('depth_point_to_base_test')

        self.bridge = CvBridge()

        self.fx = None
        self.fy = None
        self.cx = None
        self.cy = None

        # Camera center pixel for the first test.
        self.u = 320
        self.v = 240

        self.processed = False

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(
            self.tf_buffer,
            self
        )

        # Gazebo sensor topics commonly use BEST_EFFORT.
        sensor_qos = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=5,
            durability=QoSDurabilityPolicy.VOLATILE,
        )

        self.camera_info_sub = self.create_subscription(
            CameraInfo,
            '/head_front_camera/depth/camera_info',
            self.camera_info_callback,
            sensor_qos,
        )

        self.depth_sub = self.create_subscription(
            Image,
            '/head_front_camera/depth/image_rect_raw',
            self.depth_callback,
            sensor_qos,
        )

        self.get_logger().info(
            'Depth -> base_link 3D test started.'
        )

        self.get_logger().info(
            f'Testing center pixel u={self.u}, v={self.v}'
        )

    def camera_info_callback(self, msg):

        self.fx = float(msg.k[0])
        self.fy = float(msg.k[4])
        self.cx = float(msg.k[2])
        self.cy = float(msg.k[5])

    def get_valid_depth(self, depth_image, u, v):

        height, width = depth_image.shape

        # Search a small window instead of trusting exactly one pixel.
        radius = 3

        valid_values = []

        for y in range(
            max(0, v - radius),
            min(height, v + radius + 1)
        ):
            for x in range(
                max(0, u - radius),
                min(width, u + radius + 1)
            ):
                value = float(depth_image[y, x])

                if math.isfinite(value) and value > 0.0:
                    valid_values.append(value)

        if not valid_values:
            return None

        # Median makes the measurement more robust.
        valid_values.sort()

        return valid_values[len(valid_values) // 2]

    def depth_callback(self, msg):

        if self.processed:
            return

        if None in (
            self.fx,
            self.fy,
            self.cx,
            self.cy,
        ):
            self.get_logger().info(
                'Waiting for camera info...'
            )
            return

        try:
            depth_image = self.bridge.imgmsg_to_cv2(
                msg,
                desired_encoding='32FC1',
            )

        except Exception as exc:
            self.get_logger().error(
                f'cv_bridge failed: {exc}'
            )
            return

        height, width = depth_image.shape

        if not (
            0 <= self.u < width
            and
            0 <= self.v < height
        ):
            self.get_logger().error(
                f'Pixel ({self.u}, {self.v}) '
                f'is outside {width}x{height}.'
            )
            return

        z = self.get_valid_depth(
            depth_image,
            self.u,
            self.v,
        )

        if z is None:
            self.get_logger().error(
                f'No valid depth near '
                f'({self.u}, {self.v}).'
            )
            return

        # Pinhole camera projection.
        x = (
            (self.u - self.cx)
            * z
            / self.fx
        )

        y = (
            (self.v - self.cy)
            * z
            / self.fy
        )

        self.get_logger().info(
            'Camera-frame point: '
            f'X={x:.3f}, '
            f'Y={y:.3f}, '
            f'Z={z:.3f} m'
        )

        point_camera = PointStamped()

        point_camera.header.frame_id = (
            msg.header.frame_id
        )

        point_camera.header.stamp = (
            msg.header.stamp
        )

        point_camera.point.x = x
        point_camera.point.y = y
        point_camera.point.z = z

        try:
            transform = (
                self.tf_buffer.lookup_transform(
                    'base_link',
                    msg.header.frame_id,
                    rclpy.time.Time(),
                )
            )

            point_base = do_transform_point(
                point_camera,
                transform,
            )

        except Exception as exc:
            self.get_logger().error(
                f'TF transform failed: {exc}'
            )
            return

        self.get_logger().info(
            'base_link point: '
            f'X={point_base.point.x:.3f}, '
            f'Y={point_base.point.y:.3f}, '
            f'Z={point_base.point.z:.3f} m'
        )

        self.processed = True

        self.get_logger().info(
            '3D DEPTH + TF TEST SUCCESS.'
        )

        self.create_timer(
            0.5,
            self.shutdown_once,
        )

    def shutdown_once(self):

        if rclpy.ok():
            rclpy.shutdown()


def main(args=None):

    rclpy.init(args=args)

    node = DepthPointToBase()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    if rclpy.ok():
        rclpy.shutdown()


if __name__ == '__main__':
    main()
