#!/usr/bin/env python3
"""
QR indicator LED node.

Subscribes to /vision_status (JSON-encoded String, published by
vision_status_node.py) and drives two GPIO LEDs to show, at a glance,
whether a QR code is currently in view:
    GPIO 24 (led_detected)     ON  when QR_Code_Status == "Detected"
    GPIO 23 (led_not_detected) ON  when QR_Code_Status == "Not Detected"
Only the QR_Code_Status field is used; Traffic_Light_Signal is ignored here.
"""

import json

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from gpiozero import LED


class QrLedIndicator(Node):
    def __init__(self):
        super().__init__('qr_led_indicator')
        self.led_not_detected = LED(23)
        self.led_detected = LED(24)
        self.create_subscription(
            String,
            '/vision_status',
            self.vision_status_callback,
            10,
        )

    def vision_status_callback(self, msg):
        try:
            data = json.loads(msg.data)
        except json.JSONDecodeError:
            self.get_logger().warn(f'Received invalid JSON: {msg.data}')
            return

        qr_status = data.get('QR_Code_Status')

        if qr_status == 'Detected':
            self.led_not_detected.off()
            self.led_detected.on()
        elif qr_status == 'Not Detected':
            self.led_detected.off()
            self.led_not_detected.on()
        else:
            self.get_logger().warn(f'Unknown QR_Code_Status: {qr_status}')

    def destroy_node(self):
        self.led_not_detected.close()
        self.led_detected.close()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = QrLedIndicator()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
