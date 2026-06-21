#!/usr/bin/env python3
"""camera_info_publisher.py — publishes CameraInfo calibration parameters for the simulated camera.

RViz's Camera display requires *both* an Image and a CameraInfo topic.
The Gazebo bridge only forwards image data (/camera/image_raw) and does not produce CameraInfo,
causing RViz's Camera panel to show "Status: Warn — Expecting Camera Info on...".

This node computes the intrinsic matrix K from the Pioneer URDF camera parameters
(640×480, HFOV=1.089 rad) and publishes sensor_msgs/CameraInfo at 10 Hz so RViz
can render the camera view correctly.

Intrinsic calculation:
    fx = fy = width / (2 * tan(hfov / 2))
           = 640 / (2 * tan(0.5445)) ≈ 530.0
    cx = width / 2  = 320
    cy = height / 2 = 240
"""

import math
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CameraInfo


class CameraInfoPublisher(Node):
    """ROS 2 node that continuously publishes CameraInfo calibration parameters."""

    def __init__(self):
        super().__init__('camera_info_publisher')

        # ── parameter declarations (can be overridden in launch file) ──────
        self.declare_parameter('width', 640)
        self.declare_parameter('height', 480)
        self.declare_parameter('horizontal_fov', 1.089)
        self.declare_parameter('frame_id', 'cam_optical_link')
        self.declare_parameter('publish_rate', 10.0)   # Hz
        self.declare_parameter('camera_info_topic', '/camera/camera_info')

        width = self.get_parameter('width').value
        height = self.get_parameter('height').value
        hfov = self.get_parameter('horizontal_fov').value
        frame_id = self.get_parameter('frame_id').value
        rate = self.get_parameter('publish_rate').value
        topic = self.get_parameter('camera_info_topic').value

        # ── compute intrinsic matrix K ─────────────────────────────────────
        # Assumes square pixels (fx = fy), no distortion, no skew.
        fx = width / (2.0 * math.tan(hfov / 2.0))
        fy = fx
        cx = width / 2.0
        cy = height / 2.0

        # ── build CameraInfo message ───────────────────────────────────────
        self.msg = CameraInfo()
        self.msg.header.frame_id = frame_id
        self.msg.width = width
        self.msg.height = height
        self.msg.distortion_model = 'plumb_bob'
        # all distortion coefficients zero (simulated camera has no distortion)
        self.msg.d = [0.0, 0.0, 0.0, 0.0, 0.0]
        # intrinsic matrix K (3×3 row-major)
        # [fx  0 cx]
        # [ 0 fy cy]
        # [ 0  0  1]
        self.msg.k = [
            fx, 0.0, cx,
            0.0, fy, cy,
            0.0, 0.0, 1.0,
        ]
        # projection matrix P (3×4) — monocular camera, no extrinsic offset
        self.msg.p = [
            fx, 0.0, cx, 0.0,
            0.0, fy, cy, 0.0,
            0.0, 0.0, 1.0, 0.0,
        ]
        # rectification matrix R (3×3) — identity
        self.msg.r = [
            1.0, 0.0, 0.0,
            0.0, 1.0, 0.0,
            0.0, 0.0, 1.0,
        ]

        # ── publisher + timer ──────────────────────────────────────────────
        self.publisher = self.create_publisher(CameraInfo, topic, 10)
        self.timer = self.create_timer(1.0 / rate, self.publish_callback)

        self.get_logger().info(
            f'CameraInfo publisher started → {topic} '
            f'({width}×{height}, fx={fx:.1f}, frame={frame_id})'
        )

    def publish_callback(self):
        """Timer callback: publish CameraInfo with a refreshed timestamp."""
        self.msg.header.stamp = self.get_clock().now().to_msg()
        self.publisher.publish(self.msg)


def main(args=None):
    rclpy.init(args=args)
    node = CameraInfoPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
