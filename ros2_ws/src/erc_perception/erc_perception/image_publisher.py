import os

import cv2
import rclpy

from ament_index_python.packages import get_package_share_directory
from cv_bridge import CvBridge
from rclpy.node import Node
from sensor_msgs.msg import Image


class ImagePublisher(Node):

    def __init__(self):
        super().__init__("erc_image_publisher")

        self.publisher = self.create_publisher(
            Image,
            "/erc/camera/image_raw",
            10,
        )

        self.bridge = CvBridge()

        package_share = get_package_share_directory(
            "erc_perception"
        )

        image_path = os.path.join(
            package_share,
            "images",
            "competition_test_scene.png",
        )

        self.image = cv2.imread(image_path)

        if self.image is None:
            raise RuntimeError(
                f"Could not load simulation image: {image_path}"
            )

        # 10 Hz simulated camera.
        self.timer = self.create_timer(
            0.1,
            self.publish_image,
        )

        self.get_logger().info(
            "ERC simulated camera started."
        )

        self.get_logger().info(
            "Publishing: /erc/camera/image_raw"
        )

    def publish_image(self):
        message = self.bridge.cv2_to_imgmsg(
            self.image,
            encoding="bgr8",
        )

        message.header.stamp = (
            self.get_clock().now().to_msg()
        )

        message.header.frame_id = "erc_camera"

        self.publisher.publish(message)


def main(args=None):
    rclpy.init(args=args)

    node = ImagePublisher()

    try:
        rclpy.spin(node)

    except KeyboardInterrupt:
        pass

    finally:
        node.destroy_node()

        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
