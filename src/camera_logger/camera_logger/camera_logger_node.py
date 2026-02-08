#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import os
from datetime import datetime


class CameraLogger(Node):

    def __init__(self):
        super().__init__('camera_logger')

        self.bridge = CvBridge()

        # Create log directory
        self.save_dir = os.path.expanduser("~/ascend_ws/camera_logs")
        os.makedirs(self.save_dir, exist_ok=True)

        # Subscriber (RealSense RGB)
        self.create_subscription(
            Image,
            '/camera/color/image_raw',
            self.image_cb,
            10
        )

        self.get_logger().info("Camera Logger started (RealSense RGB)")

    def image_cb(self, msg):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as e:
            self.get_logger().error(f"CV bridge error: {e}")
            return

        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S_%f")
        filename = os.path.join(self.save_dir, f"{timestamp}.png")

        cv2.imwrite(filename, cv_image)

        # Optional: show preview (comment out if not needed)
        cv2.imshow("Camera Logger", cv_image)
        cv2.waitKey(1)


def main():
    rclpy.init()
    node = CameraLogger()
    rclpy.spin(node)


if __name__ == '__main__':
    main()
