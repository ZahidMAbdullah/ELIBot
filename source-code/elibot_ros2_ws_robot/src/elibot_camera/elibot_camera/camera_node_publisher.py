#!/usr/bin/env python3
"""
Camera publisher node.

Opens the onboard USB camera with OpenCV and publishes RGB frames to
/camera/image_raw (sensor_msgs/Image) plus a matching, uncalibrated
/camera/camera_info, for the vision pipeline and RViz2 to consume.
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge
import cv2

class CameraPublisherNode(Node):
    def __init__(self):
        super().__init__('camera_publisher_node')
        self.bridge = CvBridge()
        self.cap    = cv2.VideoCapture(0)

        # Request a higher capture resolution than the driver default (often
        # 640x480). More pixels-per-QR-module and more detail for the YOLO
        # traffic light detector to work with at range. The driver will clamp
        # to the nearest resolution the camera actually supports.
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1280)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 720)

        # Keep only 1 frame in the driver's internal buffer. Without this,
        # OpenCV/V4L2 queues several frames internally, so cap.read() can
        # return a frame that's already several frames old whenever the
        # capture loop falls even slightly behind the camera's own rate —
        # adding latency before the frame even reaches ROS.
        self.cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)

        # Image topic — subscribe to this in RViz2 using the "Image" display type
        self.image_pub = self.create_publisher(Image, '/camera/image_raw', 10)

        # CameraInfo — RViz2 needs this alongside the image
        self.info_pub  = self.create_publisher(CameraInfo, '/camera/camera_info', 10)

        # Grab frame size from the camera
        self.frame_w = int(self.cap.get(cv2.CAP_PROP_FRAME_WIDTH))
        self.frame_h = int(self.cap.get(cv2.CAP_PROP_FRAME_HEIGHT))

        self.get_logger().info(
            f'Camera started at {self.frame_w}x{self.frame_h} — '
            f'publishing to /camera/image_raw'
        )

    def _make_camera_info(self, stamp):
        """
        Minimal CameraInfo so RViz2 is happy.
        These are uncalibrated values — good enough for viewing,
        not for metric measurements.
        """
        msg                 = CameraInfo()
        msg.header.stamp    = stamp
        msg.header.frame_id = 'camera_link'
        msg.width           = self.frame_w
        msg.height          = self.frame_h
        msg.distortion_model = 'plumb_bob'
        # D, K, R, P all zeros → tells RViz the camera is uncalibrated
        msg.d = [0.0, 0.0, 0.0, 0.0, 0.0]
        msg.k = [0.0, 0.0, 0.0,
                 0.0, 0.0, 0.0,
                 0.0, 0.0, 1.0]
        msg.r = [1.0, 0.0, 0.0,
                 0.0, 1.0, 0.0,
                 0.0, 0.0, 1.0]
        msg.p = [0.0, 0.0, 0.0, 0.0,
                 0.0, 0.0, 0.0, 0.0,
                 0.0, 0.0, 1.0, 0.0]
        return msg

    def run(self):
        while rclpy.ok():
            ret, frame = self.cap.read()
            if not ret:
                self.get_logger().warn('Failed to grab frame')
                continue

            stamp = self.get_clock().now().to_msg()

            # Convert OpenCV BGR → ROS Image message
            img_msg                 = self.bridge.cv2_to_imgmsg(frame, encoding='bgr8')
            img_msg.header.stamp    = stamp
            img_msg.header.frame_id = 'camera_link'

            self.image_pub.publish(img_msg)
            self.info_pub.publish(self._make_camera_info(stamp))

        self.cap.release()


def main(args=None):
    rclpy.init(args=args)
    node = CameraPublisherNode()
    try:
        node.run()
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
