#!/usr/bin/env python3
"""
C_S.1 手动集成测试：模拟移动障碍靠近，验证 safety_monitor 急停行为。

运行方式（safety_monitor 节点必须已在跑）：
  python3 src/auto_nav_part3/test/test_safety_monitor_manual.py

预期结果：
  - 前 3 帧（距离从 1.5m 递减到 0.65m，delta ≈ 0.28m > 0.15m）→ 第 3 帧触发急停
  - /part3/safety/estop_event 收到消息
  - /cmd_vel_safety 开始以 10Hz 发零速
  - 后 3 帧（距离稳定在 2.0m，delta=0）→ 急停解除
  - /cmd_vel_safety 停止发布
"""

import math
import sys
import threading
import time

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String


BEAM_COUNT  = 360   # 与仿真 URDF lidar 一致
SAFE_RANGE  = 5.0   # 正常安全距离（米）
OBSTACLE_BEAM = 180 # 障碍物所在光束索引（正前方）


class SafetyTester(Node):
    def __init__(self):
        super().__init__('safety_monitor_tester')
        self._scan_pub   = self.create_publisher(LaserScan, '/scan', 10)
        self._estop_recv = []
        self._vel_count  = 0

        self.create_subscription(String, '/part3/safety/estop_event',
                                 lambda m: self._estop_recv.append(m.data), 10)
        self.create_subscription(Twist, '/cmd_vel_safety',
                                 lambda _: setattr(self, '_vel_count', self._vel_count + 1), 10)

    def _make_scan(self, obstacle_range: float) -> LaserScan:
        """生成一帧 scan：全部 SAFE_RANGE，仅 OBSTACLE_BEAM 处设为 obstacle_range。"""
        msg = LaserScan()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'laser_frame'
        msg.angle_min       = -math.pi
        msg.angle_max       = math.pi
        msg.angle_increment = 2 * math.pi / BEAM_COUNT
        msg.range_min       = 0.1
        msg.range_max       = 10.0
        msg.ranges          = [SAFE_RANGE] * BEAM_COUNT
        msg.ranges[OBSTACLE_BEAM] = obstacle_range
        return msg

    def run_test(self):
        self.get_logger().info('=== 阶段 1：障碍物从 1.5m 靠近到 0.65m（每帧 -0.28m）===')
        # delta ≈ 0.28m > moving_delta_m(0.15m)，且距离 < estop_distance_m(1.0m)
        approach_frames = [1.5, 1.22, 0.94, 0.66]  # 第 3 帧(0.94m)起满足条件
        for r in approach_frames:
            self._scan_pub.publish(self._make_scan(r))
            self.get_logger().info(f'  发布 scan: obstacle={r:.2f}m')
            time.sleep(0.15)   # 模拟 ~7Hz scan

        time.sleep(0.8)   # 等待 safety_monitor 响应 + 定时器发几帧零速

        if self._estop_recv:
            self.get_logger().info(f'[PASS] 急停事件收到: {self._estop_recv[-1]}')
        else:
            self.get_logger().error('[FAIL] 急停事件未收到！检查 safety_monitor 是否在运行')

        if self._vel_count > 0:
            self.get_logger().info(f'[PASS] /cmd_vel_safety 收到 {self._vel_count} 帧零速')
        else:
            self.get_logger().error('[FAIL] /cmd_vel_safety 无输出！检查 twist_mux')

        self.get_logger().info('\n=== 阶段 2：障碍消失（距离回到安全值，delta=0）===')
        vel_before = self._vel_count
        for _ in range(6):
            self._scan_pub.publish(self._make_scan(SAFE_RANGE))
            time.sleep(0.15)

        time.sleep(0.8)

        vel_after = self._vel_count
        if vel_after == vel_before:
            self.get_logger().info('[PASS] /cmd_vel_safety 停止发布（急停已解除）')
        else:
            # 急停解除需要 consecutive_frames 帧无靠近，可能还在等
            self.get_logger().warn(f'[INFO] /cmd_vel_safety 还在发（{vel_after - vel_before} 帧），急停可能未解除')


def main():
    rclpy.init()
    node = SafetyTester()
    spin_thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    spin_thread.start()

    time.sleep(0.5)   # 等订阅者注册到 DDS
    try:
        node.run_test()
    finally:
        node.destroy_node()
        rclpy.shutdown()
        spin_thread.join(timeout=2.0)


if __name__ == '__main__':
    main()
