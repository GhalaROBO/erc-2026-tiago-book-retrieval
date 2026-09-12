#!/usr/bin/env python3

import json

import cv2
import numpy as np
import rclpy

from cv_bridge import CvBridge
from rclpy.node import Node
from rclpy.qos import (
    QoSProfile,
    QoSReliabilityPolicy,
    QoSHistoryPolicy,
    QoSDurabilityPolicy,
)
from sensor_msgs.msg import Image
from std_msgs.msg import String


class PerceptionNode(Node):

    def __init__(self):
        super().__init__('erc_perception_node')

        # ==========================================================
        # CONFIGURATION
        # ==========================================================

        self.target_color = 'BLUE'

        # Ignore very tiny blue regions.
        self.minimum_book_area = 250

        # Maximum number of book candidates published per frame.
        self.maximum_books = 10

        # ----------------------------------------------------------
        # BLUE HSV ranges
        #
        # OpenCV:
        # H = 0..179
        # S = 0..255
        # V = 0..255
        #
        # This range is deliberately broad for Gazebo rendering.
        # ----------------------------------------------------------

        self.blue_lower = np.array(
            [90, 70, 40],
            dtype=np.uint8,
        )

        self.blue_upper = np.array(
            [140, 255, 255],
            dtype=np.uint8,
        )

        # ==========================================================
        # CV BRIDGE
        # ==========================================================

        self.bridge = CvBridge()

        # ==========================================================
        # SENSOR QoS
        #
        # Critical Phase-8 fix:
        #
        # Gazebo camera publishes BEST_EFFORT.
        # A RELIABLE subscription is incompatible.
        #
        # Therefore subscribe BEST_EFFORT.
        # ==========================================================

        sensor_qos = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            durability=QoSDurabilityPolicy.VOLATILE,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=5,
        )

        # ==========================================================
        # CAMERA SUBSCRIPTION
        # ==========================================================

        self.image_subscriber = self.create_subscription(
            Image,
            '/head_front_camera/color/image_raw',
            self.image_callback,
            sensor_qos,
        )

        # ==========================================================
        # PUBLISHERS
        # ==========================================================

        self.books_publisher = self.create_publisher(
            String,
            '/erc/perception/books',
            10,
        )

        self.bin_publisher = self.create_publisher(
            String,
            '/erc/perception/bin',
            10,
        )

        self.shelf_publisher = self.create_publisher(
            String,
            '/erc/perception/shelf_columns',
            10,
        )

        self.debug_image_publisher = self.create_publisher(
            Image,
            '/erc/perception/debug_image',
            sensor_qos,
        )

        # ==========================================================
        # STATE
        # ==========================================================

        self.frame_count = 0

        self.last_detection_count = -1

        # ==========================================================
        # STARTUP
        # ==========================================================

        self.get_logger().info(
            '================================================'
        )

        self.get_logger().info(
            'ERC PHASE 8 REAL PERCEPTION NODE'
        )

        self.get_logger().info(
            'Camera QoS: BEST_EFFORT'
        )

        self.get_logger().info(
            'BLUE book detection: ENABLED'
        )

        self.get_logger().info(
            'Publishing: /erc/perception/books'
        )

        self.get_logger().info(
            'Publishing: /erc/perception/debug_image'
        )

        self.get_logger().info(
            '================================================'
        )

    # ==============================================================
    # IMAGE CALLBACK
    # ==============================================================

    def image_callback(
        self,
        message,
    ):

        self.frame_count += 1

        try:

            image = self.bridge.imgmsg_to_cv2(
                message,
                desired_encoding='bgr8',
            )

        except Exception as exc:

            self.get_logger().error(
                f'cv_bridge conversion failed: {exc}'
            )

            return

        if image is None:
            return

        if image.size == 0:
            return

        # ----------------------------------------------------------
        # Detect blue book candidates.
        # ----------------------------------------------------------

        books, debug_image = self.detect_blue_books(
            image
        )

        # ----------------------------------------------------------
        # Publish book detections every frame.
        #
        # Publishing empty detections is intentional. It prevents
        # downstream nodes from treating an old book as current.
        # ----------------------------------------------------------

        self.publish_books(
            books
        )

        # ----------------------------------------------------------
        # Debug image
        # ----------------------------------------------------------

        self.publish_debug_image(
            debug_image,
            message,
        )

        # ----------------------------------------------------------
        # Log only when detection count changes.
        # ----------------------------------------------------------

        detection_count = len(books)

        if detection_count != self.last_detection_count:

            self.last_detection_count = detection_count

            if detection_count > 0:

                self.get_logger().info(
                    (
                        f'REAL BLUE detection: '
                        f'{detection_count} candidate(s).'
                    )
                )

                for book in books:

                    self.get_logger().info(
                        (
                            'BLUE BOOK | '
                            f'pixel=({book["center_x"]},'
                            f'{book["center_y"]}) | '
                            f'area={book["area"]:.0f}'
                        )
                    )

            else:

                self.get_logger().info(
                    'No BLUE book currently visible.'
                )

    # ==============================================================
    # BLUE DETECTION
    # ==============================================================

    def detect_blue_books(
        self,
        image,
    ):

        debug_image = image.copy()

        # ----------------------------------------------------------
        # Slight blur helps with Gazebo aliasing / noisy edges.
        # ----------------------------------------------------------

        blurred = cv2.GaussianBlur(
            image,
            (5, 5),
            0,
        )

        hsv = cv2.cvtColor(
            blurred,
            cv2.COLOR_BGR2HSV,
        )

        mask = cv2.inRange(
            hsv,
            self.blue_lower,
            self.blue_upper,
        )

        # ----------------------------------------------------------
        # Morphological cleanup.
        # ----------------------------------------------------------

        kernel = np.ones(
            (5, 5),
            np.uint8,
        )

        mask = cv2.morphologyEx(
            mask,
            cv2.MORPH_OPEN,
            kernel,
        )

        mask = cv2.morphologyEx(
            mask,
            cv2.MORPH_CLOSE,
            kernel,
        )

        contours, _ = cv2.findContours(
            mask,
            cv2.RETR_EXTERNAL,
            cv2.CHAIN_APPROX_SIMPLE,
        )

        candidates = []

        for contour in contours:

            area = float(
                cv2.contourArea(contour)
            )

            if area < self.minimum_book_area:
                continue

            x, y, width, height = cv2.boundingRect(
                contour
            )

            if width <= 0 or height <= 0:
                continue

            center_x = int(
                x + width / 2
            )

            center_y = int(
                y + height / 2
            )

            aspect_ratio = (
                float(height)
                /
                float(width)
            )

            # ------------------------------------------------------
            # Do not make the shape constraint too aggressive.
            #
            # A physical book is normally rectangular, but its
            # apparent aspect ratio changes with perspective.
            # ------------------------------------------------------

            if aspect_ratio < 0.30:
                continue

            if aspect_ratio > 8.0:
                continue

            candidate = {
                'class': 'BOOK',
                'color': 'BLUE',
                'center_x': center_x,
                'center_y': center_y,
                'bbox_x': int(x),
                'bbox_y': int(y),
                'bbox_width': int(width),
                'bbox_height': int(height),
                'area': area,
            }

            candidates.append(
                candidate
            )

        # ----------------------------------------------------------
        # Largest regions first.
        # ----------------------------------------------------------

        candidates.sort(
            key=lambda item: item['area'],
            reverse=True,
        )

        candidates = candidates[
            :self.maximum_books
        ]

        # ----------------------------------------------------------
        # Draw debug overlays.
        # ----------------------------------------------------------

        for index, book in enumerate(
            candidates
        ):

            x = book['bbox_x']
            y = book['bbox_y']
            width = book['bbox_width']
            height = book['bbox_height']

            center_x = book['center_x']
            center_y = book['center_y']

            cv2.rectangle(
                debug_image,
                (x, y),
                (
                    x + width,
                    y + height,
                ),
                (255, 0, 0),
                2,
            )

            cv2.circle(
                debug_image,
                (
                    center_x,
                    center_y,
                ),
                5,
                (0, 255, 255),
                -1,
            )

            cv2.putText(
                debug_image,
                f'BLUE BOOK {index + 1}',
                (
                    x,
                    max(
                        20,
                        y - 8,
                    ),
                ),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.55,
                (255, 0, 0),
                2,
                cv2.LINE_AA,
            )

        return (
            candidates,
            debug_image,
        )

    # ==============================================================
    # PUBLISH BOOKS
    # ==============================================================

    def publish_books(
        self,
        books,
    ):

        payload = {
            'count': len(books),
            'books': books,
        }

        message = String()

        message.data = json.dumps(
            payload
        )

        self.books_publisher.publish(
            message
        )

    # ==============================================================
    # PUBLISH DEBUG IMAGE
    # ==============================================================

    def publish_debug_image(
        self,
        image,
        source_message,
    ):

        try:

            debug_message = (
                self.bridge.cv2_to_imgmsg(
                    image,
                    encoding='bgr8',
                )
            )

            debug_message.header = (
                source_message.header
            )

            self.debug_image_publisher.publish(
                debug_message
            )

        except Exception as exc:

            self.get_logger().warn(
                (
                    'Unable to publish debug image: '
                    f'{exc}'
                )
            )


def main(args=None):

    rclpy.init(
        args=args
    )

    node = PerceptionNode()

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
