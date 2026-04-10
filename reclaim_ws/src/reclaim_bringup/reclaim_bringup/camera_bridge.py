#!/usr/bin/env python3
"""
camera_bridge.py — Bridges TRT server /frame endpoint to ROS2 image topic.

Publishes CompressedImage (JPEG passthrough, no decode) for low-latency
Foxglove visualization. ~30-50KB/frame instead of ~900KB raw.

Usage:
  ros2 run reclaim_bringup camera_bridge --ros-args -p trt_url:=http://localhost:8081 -p rate_hz:=30.0
"""

import threading
import time
import urllib.request

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage


class CameraBridge(Node):
    def __init__(self):
        super().__init__('camera_bridge')

        self.declare_parameter('trt_url', 'http://localhost:8081')
        self.declare_parameter('rate_hz', 30.0)

        self.trt_url = self.get_parameter('trt_url').value.rstrip('/')
        self.rate_hz = self.get_parameter('rate_hz').value

        self.pub = self.create_publisher(
            CompressedImage, '/perception/image_raw/compressed', 10)

        self.poll_thread = threading.Thread(target=self._poll_loop, daemon=True)
        self.poll_thread.start()

        self.get_logger().info(
            f'CameraBridge: {self.trt_url}/frame → /perception/image_raw/compressed @ {self.rate_hz}Hz')

    def _poll_loop(self):
        interval = 1.0 / self.rate_hz
        frame_url = f'{self.trt_url}/frame'
        pub_count = 0

        while rclpy.ok():
            try:
                resp = urllib.request.urlopen(frame_url, timeout=1.0)
                jpeg_data = resp.read()

                if len(jpeg_data) < 100:
                    time.sleep(interval)
                    continue

                msg = CompressedImage()
                msg.header.stamp = self.get_clock().now().to_msg()
                msg.header.frame_id = 'camera_optical_frame'
                msg.format = 'jpeg'
                msg.data = jpeg_data

                self.pub.publish(msg)
                pub_count += 1
                if pub_count == 1:
                    self.get_logger().info(f'Publishing compressed frames ({len(jpeg_data)} bytes)')
                elif pub_count % 300 == 0:
                    self.get_logger().info(f'Published {pub_count} frames')

            except Exception as e:
                if pub_count == 0:
                    self.get_logger().warn(f'CameraBridge error: {e}')

            time.sleep(interval)


def main(args=None):
    rclpy.init(args=args)
    node = CameraBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
