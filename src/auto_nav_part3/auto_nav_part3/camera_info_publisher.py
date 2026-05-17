#!/usr/bin/env python3
"""camera_info_publisher.py — 为仿真相机发布 CameraInfo 标定参数。

RViz 的 Camera 显示需要 *同时* 订阅 Image 和 CameraInfo 话题。
Gazebo 桥接只转发图像数据（/camera/image_raw），不生成 CameraInfo，
导致 RViz 中 Camera 面板显示 "Status: Warn — Expecting Camera Info on..."。

本节点从 Pioneer URDF 的相机参数（640×480，HFOV=1.089 rad）计算
内参矩阵 K，以 10 Hz 频率发布 sensor_msgs/CameraInfo，使 RViz 能
正确渲染相机画面。

内参计算：
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
    """持续发布 CameraInfo 标定参数的 ROS 2 节点。"""

    def __init__(self):
        super().__init__('camera_info_publisher')

        # ── 声明参数（可在 launch 文件中覆盖） ──────────────────────
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

        # ── 计算内参矩阵 K ─────────────────────────────────────────
        # 假设正方形像素 (fx = fy)，无畸变，无歪斜。
        fx = width / (2.0 * math.tan(hfov / 2.0))
        fy = fx
        cx = width / 2.0
        cy = height / 2.0

        # ── 构造 CameraInfo 消息 ───────────────────────────────────
        self.msg = CameraInfo()
        self.msg.header.frame_id = frame_id
        self.msg.width = width
        self.msg.height = height
        self.msg.distortion_model = 'plumb_bob'
        # 畸变系数全 0（仿真相机无畸变）
        self.msg.d = [0.0, 0.0, 0.0, 0.0, 0.0]
        # 内参矩阵 K (3×3 row-major)
        # [fx  0 cx]
        # [ 0 fy cy]
        # [ 0  0  1]
        self.msg.k = [
            fx, 0.0, cx,
            0.0, fy, cy,
            0.0, 0.0, 1.0,
        ]
        # 投影矩阵 P (3×4) — 单目相机，无外参偏移
        self.msg.p = [
            fx, 0.0, cx, 0.0,
            0.0, fy, cy, 0.0,
            0.0, 0.0, 1.0, 0.0,
        ]
        # 矫正矩阵 R (3×3) — 单位矩阵
        self.msg.r = [
            1.0, 0.0, 0.0,
            0.0, 1.0, 0.0,
            0.0, 0.0, 1.0,
        ]

        # ── 发布者 + 定时器 ────────────────────────────────────────
        self.publisher = self.create_publisher(CameraInfo, topic, 10)
        self.timer = self.create_timer(1.0 / rate, self.publish_callback)

        self.get_logger().info(
            f'CameraInfo publisher started → {topic} '
            f'({width}×{height}, fx={fx:.1f}, frame={frame_id})'
        )

    def publish_callback(self):
        """定时发布 CameraInfo，每次刷新时间戳。"""
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
