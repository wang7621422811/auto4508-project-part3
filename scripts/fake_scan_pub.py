#!/usr/bin/env python3
"""
fake_scan_pub.py — 发布模拟 LaserScan，用于离线测试感知节点。

配置与 URDF pioneer.urdf 中雷达参数一致：
  720 光束，±120°，Gaussian 噪声 σ=0.015m。

用法:
  python3 scripts/fake_scan_pub.py [距离m] [频率Hz]

示例:
  python3 scripts/fake_scan_pub.py          # 全场 1.5m，10Hz
  python3 scripts/fake_scan_pub.py 2.0      # 全场 2.0m
  python3 scripts/fake_scan_pub.py 1.5 20   # 全场 1.5m，20Hz

搭配 camera 模式使用：
  # 终端1: ./scripts/launch.sh camera --no-build
  # 终端2: python3 scripts/fake_scan_pub.py
  # 终端3: python3 scripts/pub_image.py artifacts/photos/colour_yellow_*.jpg
  # 终端4: ros2 topic echo /part3/perception/marker_event
"""

import math
import random
import sys

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan


def main() -> None:
    uniform_range = float(sys.argv[1]) if len(sys.argv) > 1 else 1.5
    rate_hz       = float(sys.argv[2]) if len(sys.argv) > 2 else 10.0

    rclpy.init()
    node = Node('fake_scan_pub')
    pub  = node.create_publisher(LaserScan, '/scan', 10)

    # 与 URDF 雷达参数一致
    N_BEAMS     = 720
    ANGLE_MIN   = -2.0944   # -120°
    ANGLE_MAX   =  2.0944   # +120°
    ANGLE_INC   =  (ANGLE_MAX - ANGLE_MIN) / N_BEAMS
    RANGE_MIN   =  0.35
    RANGE_MAX   = 12.0
    NOISE_SIGMA =  0.015    # 与 URDF stddev 一致

    msg = LaserScan()
    msg.header.frame_id = 'laser_frame'
    msg.angle_min       = ANGLE_MIN
    msg.angle_max       = ANGLE_MAX
    msg.angle_increment = ANGLE_INC
    msg.time_increment  = 0.0
    msg.scan_time       = 1.0 / rate_hz
    msg.range_min       = RANGE_MIN
    msg.range_max       = RANGE_MAX

    count = [0]

    def publish() -> None:
        msg.header.stamp = node.get_clock().now().to_msg()
        # 加 Gaussian 噪声模拟真实雷达
        msg.ranges = [
            max(RANGE_MIN, min(RANGE_MAX,
                uniform_range + random.gauss(0.0, NOISE_SIGMA)))
            for _ in range(N_BEAMS)
        ]
        pub.publish(msg)
        count[0] += 1
        if count[0] % int(rate_hz) == 0:
            node.get_logger().info(
                f'[fake_scan] 已发布 {count[0]} 帧  range={uniform_range}m  '
                f'noise_sigma={NOISE_SIGMA}m  rate={rate_hz}Hz'
            )

    node.create_timer(1.0 / rate_hz, publish)
    node.get_logger().info(
        f'[fake_scan] 启动: {N_BEAMS} 光束  ±120°  全场 {uniform_range}m  '
        f'@ {rate_hz}Hz  (Ctrl+C 停止)'
    )

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
