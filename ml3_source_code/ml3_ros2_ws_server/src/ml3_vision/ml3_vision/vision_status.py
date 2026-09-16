#!/usr/bin/env python3
"""
Combined vision status ROS2 node — QR presence + traffic light colour.

DEPLOYMENT: this node is the "remote compute" half of an onboard-capture /
remote-compute split. Run it on a separate, more powerful machine (e.g. a
laptop) on the same ROS2 domain as the robot's Raspberry Pi, connected over
the wifi network — the robot side only needs ml3_ros2_ws_robot's ml3_camera
package running to publish /camera/image_raw. This was used during
development to offload heavier detection than the Pi could run in real
time; the version that ships on the robot itself is
ml3_ros2_ws_robot/.../ml3_vision/vision_status_node.py, which adds
threading, an NCNN-exported model, and other latency optimizations that
matter on the Pi's weaker CPU but aren't necessary on a full machine.

This node publishes to the SAME topic/schema as vision_status_node.py, so
it is a drop-in replacement for it: run this one here instead of
vision_status_node.py on the Pi, and ml3_ros2_ws_robot's qr_led_indicator.py
node still reacts to it normally. One caveat: qr_led_indicator.py subscribes
with default (VOLATILE) QoS rather than matching this publisher's
TRANSIENT_LOCAL durability, so if it starts up AFTER this node has already
published its one-time startup baseline, it will miss that specific message
(harmless — it just waits for the next real state change) rather than
receiving the retained one.

Subscribes to /camera/image_raw, runs BOTH detectors on each frame:
  * QR:            pyzbar  -> "Detected" / "Not Detected"
  * Traffic light: YOLOv8 + HSV -> "red"/"green"/"yellow"/"unknown"/"none"

On startup it immediately publishes the baseline once:
    {"QR_Code_Status": "Not Detected", "Traffic_Light_Signal": "none"}
After that, each value is debounced independently and the full status is
re-published ONLY when either debounced value changes.

ROS 2 has no native dict message, so the dict is JSON-encoded in a String.
A subscriber recovers it with:  data = json.loads(msg.data)

Setup:  pip install pyzbar ultralytics   and   sudo apt-get install libzbar0
Optional --show opens an annotated preview window.
"""

import argparse
import json

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import (qos_profile_sensor_data, QoSProfile,
                       ReliabilityPolicy, DurabilityPolicy)
from sensor_msgs.msg import Image
from std_msgs.msg import String
from cv_bridge import CvBridge, CvBridgeError
from ultralytics import YOLO
from pyzbar import pyzbar

TRAFFIC_LIGHT_CLASS = 9   # COCO class id for "traffic light"
STATUS_TOPIC = 'vision_status'
TL_ANNOUNCE_TOPIC = '/traffic_light_topic'
TL_ANNOUNCE_MSG = 'Team12_Green'


class DebouncedState:
    """Holds a state that only flips after a new value holds for N frames."""
    def __init__(self, confirm_frames, initial):
        self.state = initial
        self.candidate = None
        self.counter = 0
        self.confirm_frames = confirm_frames

    def update(self, raw):
        """Feed this frame's raw observation. Return True if the state changed."""
        if raw == self.state:
            self.candidate = None
            self.counter = 0
            return False
        if raw == self.candidate:
            self.counter += 1
        else:
            self.candidate = raw
            self.counter = 1
        if self.counter >= self.confirm_frames:
            self.state = raw
            self.candidate = None
            self.counter = 0
            return True
        return False


def classify_colour(roi):
    """Return 'red' / 'green' / 'yellow' / 'unknown' for a cropped light ROI."""
    if roi is None or roi.size == 0:
        return 'unknown'
    hsv = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)

    mask_red    = cv2.inRange(hsv, np.array([0, 120, 70]),   np.array([10, 255, 255]))
    mask_green  = cv2.inRange(hsv, np.array([45, 100, 50]),  np.array([75, 255, 255]))
    mask_yellow = cv2.inRange(hsv, np.array([20, 100, 100]), np.array([30, 255, 255]))

    red    = cv2.countNonZero(mask_red)
    green  = cv2.countNonZero(mask_green)
    yellow = cv2.countNonZero(mask_yellow)

    if red > green and red > yellow:
        return 'red'
    if green > red and green > yellow:
        return 'green'
    if yellow > red and yellow > green:
        return 'yellow'
    return 'unknown'


class VisionStatusNode(Node):
    def __init__(self, confirm_frames=5, show=False):
        super().__init__('vision_status_node')
        self.bridge = CvBridge()
        self.show = show
        self.model = YOLO('yolov8n.pt')   # auto-downloads on first run if missing

        # Latched (transient-local) publisher: the last status is re-delivered
        # to any transient-local subscriber that joins later, so the baseline
        # can't be missed due to launch order.
        status_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.status_pub = self.create_publisher(String, STATUS_TOPIC, status_qos)

        # Announcement publisher: sends TL_ANNOUNCE_MSG once each time the
        # debounced traffic light state transitions to 'green'.
        self.tl_announce_pub = self.create_publisher(String, TL_ANNOUNCE_TOPIC, 10)

        self.image_sub = self.create_subscription(
            Image, '/camera/image_raw', self.image_callback, qos_profile_sensor_data)

        # Baseline = "camera on, nothing seen yet"; after the startup publish
        # below, messages go out only on real (debounced) changes.
        self.qr = DebouncedState(confirm_frames, initial='Not Detected')
        self.tl = DebouncedState(confirm_frames, initial='none')

        # Publish the baseline once, ~1 s after startup: publishing directly in
        # __init__ races DDS discovery (no subscriber is matched yet, so the
        # message is silently lost). The one-shot timer gives discovery time.
        self._baseline_timer = self.create_timer(1.0, self._publish_baseline)

    def _publish_baseline(self):
        self._baseline_timer.cancel()      # one-shot
        self.publish_status()

    def publish_status(self):
        payload = {
            'QR_Code_Status': self.qr.state,
            'Traffic_Light_Signal': self.tl.state,
        }
        print(payload)
        msg = String()
        msg.data = json.dumps(payload)
        self.status_pub.publish(msg)

    def image_callback(self, msg):
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except CvBridgeError as e:
            self.get_logger().warn(f'cv_bridge conversion failed: {e}')
            return

        # --- QR presence (pyzbar) ---
        qr_codes = [c for c in pyzbar.decode(frame) if c.type == 'QRCODE']
        qr_raw = 'Detected' if qr_codes else 'Not Detected'

        # --- Traffic light colour (YOLO + HSV over the detected box) ---
        results = self.model(frame, classes=[TRAFFIC_LIGHT_CLASS],
                             imgsz=256, verbose=False)
        boxes = results[0].boxes
        if boxes is not None and len(boxes) > 0:
            x1, y1, x2, y2 = (int(v) for v in boxes.xyxy[0].tolist())
            h, w = frame.shape[:2]
            x1, y1 = max(0, x1), max(0, y1)
            x2, y2 = min(w, x2), min(h, y2)
            roi = frame[y1:y2, x1:x2] if (x2 > x1 and y2 > y1) else None
            tl_raw = classify_colour(roi)
        else:
            tl_raw = 'none'

        # Publish once if EITHER debounced state confirmed a change this frame.
        qr_changed = self.qr.update(qr_raw)
        tl_changed = self.tl.update(tl_raw)

        # Announce on the confirmed transition INTO green (once per green event).
        if tl_changed and self.tl.state == 'green':
            announce = String()
            announce.data = TL_ANNOUNCE_MSG
            self.tl_announce_pub.publish(announce)
            print(f'-> {TL_ANNOUNCE_TOPIC}: {TL_ANNOUNCE_MSG}')

        if qr_changed or tl_changed:
            self.publish_status()

        if self.show:
            annotated = results[0].plot()
            for c in qr_codes:
                x, y, bw, bh = c.rect
                cv2.rectangle(annotated, (x, y), (x + bw, y + bh), (0, 255, 0), 2)
            cv2.imshow('vision', annotated)
            cv2.waitKey(1)


def main(args=None):
    parser = argparse.ArgumentParser(
        description='Combined QR + traffic light status from /camera/image_raw')
    parser.add_argument('--frames', type=int, default=5,
                        help='consecutive frames needed to confirm a change (default 5)')
    parser.add_argument('--show', action='store_true',
                        help='show an annotated preview window (debug)')
    parsed, ros_args = parser.parse_known_args()

    rclpy.init(args=ros_args)
    node = VisionStatusNode(confirm_frames=parsed.frames, show=parsed.show)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if parsed.show:
            cv2.destroyAllWindows()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
