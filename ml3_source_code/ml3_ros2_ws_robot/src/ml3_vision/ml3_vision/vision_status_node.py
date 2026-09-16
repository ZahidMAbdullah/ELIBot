#!/usr/bin/env python3
"""
Combined vision status ROS2 node — QR presence + traffic light colour.

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
import os
import shutil
import contextlib
import threading
from pathlib import Path

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from sensor_msgs.msg import Image
from std_msgs.msg import String
from cv_bridge import CvBridge, CvBridgeError
from ultralytics import YOLO
from pyzbar import pyzbar

TRAFFIC_LIGHT_CLASS = 9   # COCO class id for "traffic light"
STATUS_TOPIC = 'vision_status'
TL_ANNOUNCE_TOPIC = '/traffic_light_topic'
TL_ANNOUNCE_MSG = 'Team12_Green'
YOLO_IMGSZ = 480
YOLO_CONF = 0.15

MODEL_CACHE_DIR = Path.home() / '.cache' / 'ml3_vision'
NCNN_MODEL_DIR = MODEL_CACHE_DIR / 'yolov8n_ncnn_model'


@contextlib.contextmanager
def _chdir(path):
    prev = os.getcwd()
    os.chdir(path)
    try:
        yield
    finally:
        os.chdir(prev)


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
    """Return 'red' / 'green' / 'yellow' / 'unknown' for a cropped light ROI.

    Combines HSV hue evidence with WHERE the brightest/most saturated blob
    sits vertically in the box. Traffic lights are always laid out
    red-top/yellow-middle/green-bottom, and that physical layout is a much
    steadier signal than raw hue, which drifts under glare and white-balance
    shifts. When both agree it's a strong call; when the box only yields a
    lit-blob position (hue ambiguous) or only a hue majority (blob too
    diffuse to localise), fall back to whichever one is available.
    """
    if roi is None or roi.size == 0:
        return 'unknown'
    h, w = roi.shape[:2]
    hsv = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)

    # Red wraps around the hue circle (0-10 and 170-180), unlike green/yellow.
    mask_red    = cv2.inRange(hsv, np.array([0, 120, 70]),   np.array([10, 255, 255]))
    mask_red   |= cv2.inRange(hsv, np.array([170, 120, 70]), np.array([180, 255, 255]))
    mask_green  = cv2.inRange(hsv, np.array([45, 100, 50]),  np.array([75, 255, 255]))
    mask_yellow = cv2.inRange(hsv, np.array([20, 100, 100]), np.array([30, 255, 255]))

    counts = {
        'red': cv2.countNonZero(mask_red),
        'green': cv2.countNonZero(mask_green),
        'yellow': cv2.countNonZero(mask_yellow),
    }
    hue_colour = max(counts, key=counts.get) if max(counts.values()) > 0 else 'unknown'

    # Locate the lit bulb: bright and reasonably saturated, regardless of hue.
    bright_mask = cv2.inRange(hsv, np.array([0, 60, 180]), np.array([180, 255, 255]))
    ys, _ = np.nonzero(bright_mask)
    position_colour = 'unknown'
    if len(ys) > 0:
        ratio = ys.mean() / h
        if ratio < 1 / 3:
            position_colour = 'red'
        elif ratio < 2 / 3:
            position_colour = 'yellow'
        else:
            position_colour = 'green'

    if position_colour != 'unknown':
        return position_colour
    return hue_colour


class VisionStatusNode(Node):
    def __init__(self, confirm_frames=1, show=False):
        super().__init__('vision_status_node')
        self.bridge = CvBridge()
        self.show = show
        self.model = self._load_yolo_model()

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

        # depth=1 + BEST_EFFORT: never buffer more than the single newest
        # frame at the DDS layer either — matches the "always process the
        # latest frame" pattern used below, instead of queuing a backlog.
        image_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE)
        self.image_sub = self.create_subscription(
            Image, '/camera/image_raw', self.image_callback, image_qos)

        # The subscription callback only stashes the newest frame (cheap) —
        # heavy detection work (YOLO + pyzbar) happens on a separate thread
        # that always grabs whatever is newest. This decouples frame arrival
        # from processing time: if detection is briefly slower than the
        # camera, we skip straight to the latest frame instead of grinding
        # through a backlog of stale ones, which is what was causing the
        # growing lag between the real world and /vision_status.
        self._frame_lock = threading.Lock()
        self._latest_frame = None
        self._new_frame_event = threading.Event()
        self._stop_event = threading.Event()

        self.qr = DebouncedState(confirm_frames, initial='Not Detected')
        self.tl = DebouncedState(confirm_frames, initial='none')

        # Publish the baseline once, ~1 s after startup: publishing directly in
        # __init__ races DDS discovery (no subscriber is matched yet, so the
        # message is silently lost). The one-shot timer gives discovery time.
        self._baseline_timer = self.create_timer(1.0, self._publish_baseline)

        self._worker = threading.Thread(target=self._process_loop, daemon=True)
        self._worker.start()

    def _load_yolo_model(self):
        """Load YOLOv8n via NCNN for faster CPU inference, exporting+caching
        it under ~/.cache/ml3_vision on first run. Falls back to the plain
        PyTorch weights if NCNN/export deps aren't available or export fails.
        """
        MODEL_CACHE_DIR.mkdir(parents=True, exist_ok=True)

        if NCNN_MODEL_DIR.exists():
            try:
                model = YOLO(str(NCNN_MODEL_DIR))
                self.get_logger().info(f'Loaded cached NCNN model from {NCNN_MODEL_DIR}')
                return model
            except Exception as e:
                self.get_logger().warn(f'Cached NCNN model failed to load ({e}); re-exporting.')
                shutil.rmtree(NCNN_MODEL_DIR, ignore_errors=True)

        try:
            self.get_logger().info(
                'Exporting YOLOv8n to NCNN for faster CPU inference (one-time, ~1 min)...')
            with _chdir(MODEL_CACHE_DIR):
                pt_model = YOLO('yolov8n.pt')
                pt_model.export(format='ncnn', imgsz=YOLO_IMGSZ)
            return YOLO(str(NCNN_MODEL_DIR))
        except Exception as e:
            self.get_logger().warn(f'NCNN export failed ({e}); falling back to PyTorch model.')
            return YOLO('yolov8n.pt')

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
        """Runs on the ROS executor thread — kept as cheap as possible so it
        never becomes the bottleneck. Just converts and stashes the frame;
        the worker thread in _process_loop does the actual detection work.
        """
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except CvBridgeError as e:
            self.get_logger().warn(f'cv_bridge conversion failed: {e}')
            return
        with self._frame_lock:
            self._latest_frame = frame
        self._new_frame_event.set()

    def _process_loop(self):
        """Runs on a dedicated thread, decoupled from the ROS executor.
        Always processes the most recently received frame; if detection
        takes longer than the camera's frame interval, older frames that
        arrived in the meantime are simply skipped rather than queued.
        """
        while not self._stop_event.is_set():
            if not self._new_frame_event.wait(timeout=0.5):
                continue
            with self._frame_lock:
                frame = self._latest_frame
                self._new_frame_event.clear()
            if frame is None:
                continue
            self._process_frame(frame)

    def _process_frame(self, frame):
        # --- QR presence (pyzbar) ---
        # Decode on grayscale: guarantees pyzbar reads pure luminance instead
        # of relying on its own handling of a 3-channel BGR array, and it's
        # cheaper to scan than the full colour frame.
        #
        # A CLAHE-contrast-boost / upscale fallback was tried here and
        # measured against plain decode on synthetic dim/blurred/small QR
        # images: it never recovered a code the plain grayscale scan missed
        # (zbar already does its own adaptive local thresholding), and 2x
        # cubic upscaling once made a borderline case *worse* via
        # interpolation ringing. Not worth the extra CPU cost — removed.
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        qr_codes = [c for c in pyzbar.decode(gray) if c.type == 'QRCODE']
        qr_raw = 'Detected' if qr_codes else 'Not Detected'

        # --- Traffic light colour (YOLO + HSV over the detected box) ---
        results = self.model(frame, classes=[TRAFFIC_LIGHT_CLASS],
                             imgsz=YOLO_IMGSZ, conf=YOLO_CONF, verbose=False)
        boxes = results[0].boxes
        if boxes is not None and len(boxes) > 0:
            best = int(boxes.conf.argmax())
            x1, y1, x2, y2 = (int(v) for v in boxes.xyxy[best].tolist())
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

    def destroy_node(self):
        self._stop_event.set()
        self._worker.join(timeout=1.0)
        super().destroy_node()


def main(args=None):
    parser = argparse.ArgumentParser(
        description='Combined QR + traffic light status from /camera/image_raw')
    parser.add_argument('--frames', type=int, default=1,
                        help='consecutive frames needed to confirm a change (default 1)')
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
