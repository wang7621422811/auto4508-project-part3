"""
C_S.1 — Safety Monitor: 移动障碍检测 + 软件急停

算法（每帧 /scan 到达时）：
  1. 若机器人角速度 |wz| > rotation_skip_wz_rad → 重置 prev_ranges，跳过本帧
     （机器人转弯时激光束扫过墙角，range 突变不代表障碍靠近）
  2. delta[i] = prev_range[i] - curr_range[i]  （正值 = 靠近）
  3. moving_close = {i : delta[i] > moving_delta_m AND curr_range[i] < estop_distance_m}
  4. moving_close 非空 → confirm_count++，否则重置
  5. confirm_count >= consecutive_frames AND 冷却已过 → 触发急停
  6. 急停期间：定时器以 publish_rate_hz 持续向 /cmd_vel_safety 发零速
  7. 连续 consecutive_frames 帧无靠近光束 → 解除急停

急停通过 twist_mux 仲裁（priority=100）覆盖 Nav2 的 /cmd_vel_nav2（priority=10），
Nav2 goal 不被取消 → 障碍消失后导航自动恢复。

rotation_skip_wz_rad 参数作用：
  机器人旋转时，同一束激光在相邻帧指向不同方向。靠近墙角时 range 从 5m → 0.3m，
  delta = 4.7m 远超阈值，会误触急停。订阅 /odometry/filtered 获取 wz，
  转弯时不做帧间比较，避免误报。
"""

import math

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String


class SafetyMonitor(Node):
    """
    监听 /scan，检测正在靠近的移动障碍物；进入 1m 内时触发软件急停。

    发布目标：/cmd_vel_safety（由 twist_mux 以最高优先级路由到 /cmd_vel）。
    Nav2 goal 全程不被取消，障碍离开后机器人自动恢复导航。
    """

    def __init__(self):
        super().__init__('part3_safety_monitor')

        # ── 参数声明（可被 config/safety.yaml 覆盖）────────────────────────────
        self.declare_parameter('estop_distance_m',      1.0)
        self.declare_parameter('moving_delta_m',        0.15)
        self.declare_parameter('consecutive_frames',    3)
        self.declare_parameter('estop_cooldown_sec',    2.0)
        self.declare_parameter('publish_rate_hz',       10.0)
        # 机器人角速度超过此阈值（rad/s）时跳过帧间比较，防止转弯误报
        # MPPI wz_max=2.5 rad/s；设 0.3 rad/s 可过滤绝大多数 Nav2 转弯场景
        self.declare_parameter('rotation_skip_wz_rad', 0.3)

        self._dist        = float(self.get_parameter('estop_distance_m').value)
        self._delta       = float(self.get_parameter('moving_delta_m').value)
        self._req_frames  = int(self.get_parameter('consecutive_frames').value)
        self._cooldown    = float(self.get_parameter('estop_cooldown_sec').value)
        self._rate        = float(self.get_parameter('publish_rate_hz').value)
        self._skip_wz     = float(self.get_parameter('rotation_skip_wz_rad').value)

        # ── 状态变量 ────────────────────────────────────────────────────────────
        self._prev_ranges: list[float] | None = None
        self._confirm_count  = 0    # 连续检测到靠近帧数（触发计数）
        self._clear_count    = 0    # 连续无靠近帧数（解除计数）
        self._in_estop       = False
        self._last_estop_ts  = 0.0  # 上次急停触发时间戳（秒）
        self._curr_wz        = 0.0  # 最新角速度（来自 /odometry/filtered）

        self._zero_twist = Twist()  # 预分配零速消息（全零即停车）

        # ── 发布者 / 订阅者 ─────────────────────────────────────────────────────
        # /cmd_vel_safety → twist_mux（priority=100）→ /cmd_vel → Gazebo
        self._vel_pub   = self.create_publisher(Twist,  '/cmd_vel_safety',           10)
        self._event_pub = self.create_publisher(String, '/part3/safety/estop_event', 10)
        self._scan_sub  = self.create_subscription(LaserScan, '/scan', self._on_scan, 10)
        # 订阅 EKF 融合里程计，获取机器人角速度用于旋转过滤
        self._odom_sub  = self.create_subscription(
            Odometry, '/odometry/filtered', self._on_odom, 10
        )

        # 急停持续零速定时器（启动即暂停；trigger_estop 时 reset 激活）
        self._estop_timer = self.create_timer(1.0 / self._rate, self._publish_zero_vel)
        self._estop_timer.cancel()

        self.get_logger().info(
            f'SafetyMonitor ready — '
            f'dist={self._dist}m  delta={self._delta}m  '
            f'frames={self._req_frames}  cooldown={self._cooldown}s  '
            f'rate={self._rate}Hz  skip_wz={self._skip_wz}rad/s'
        )

    # ── 核心回调 ─────────────────────────────────────────────────────────────────

    def _on_odom(self, msg: Odometry) -> None:
        """从 EKF 里程计更新机器人角速度，用于旋转过滤。"""
        self._curr_wz = msg.twist.twist.angular.z

    def _on_scan(self, scan: LaserScan) -> None:
        ranges = list(scan.ranges)

        # 首帧或帧长变化时只记录，不比较
        if self._prev_ranges is None or len(self._prev_ranges) != len(ranges):
            self._prev_ranges = ranges
            return

        # ── 旋转过滤：转弯时激光束扫过墙角，range 突变≠障碍靠近 ───────────────
        # 机器人在旋转（|wz| > 阈值）时，更新 prev_ranges 但跳过障碍检测，
        # 避免激光束扫过墙角时 delta 突增误触急停。
        if abs(self._curr_wz) > self._skip_wz:
            self._prev_ranges = ranges
            return

        # ── 步骤 1-2：计算靠近光束列表 ─────────────────────────────────────────
        moving_close: list[tuple[float, float]] = []   # (距离, 方位角度)
        for i, (prev, curr) in enumerate(zip(self._prev_ranges, ranges)):
            if not (math.isfinite(prev) and math.isfinite(curr) and curr > 0.0):
                continue
            if (prev - curr) > self._delta and curr < self._dist:
                angle_deg = math.degrees(scan.angle_min + i * scan.angle_increment)
                moving_close.append((curr, angle_deg))

        self._prev_ranges = ranges
        now = self.get_clock().now().nanoseconds / 1e9

        if moving_close:
            # ── 步骤 3：累积确认帧，重置解除计数 ──────────────────────────────
            self._confirm_count += 1
            self._clear_count    = 0

            # ── 步骤 4：尚未急停 → 检查是否需要触发 ───────────────────────────
            if not self._in_estop:
                cooldown_ok = (now - self._last_estop_ts) >= self._cooldown
                if self._confirm_count >= self._req_frames and cooldown_ok:
                    self._trigger_estop(moving_close, now)
        else:
            # ── 无靠近光束：重置触发计数；若在急停中则累积解除帧 ──────────────
            self._confirm_count = 0
            if self._in_estop:
                self._clear_count += 1
                if self._clear_count >= self._req_frames:
                    self._release_estop()

    # ── 急停触发 / 解除 ──────────────────────────────────────────────────────────

    def _trigger_estop(self, moving_close: list[tuple[float, float]], now: float) -> None:
        self._in_estop      = True
        self._last_estop_ts = now
        self._clear_count   = 0

        # 启动定时器，持续发零速（覆盖 Nav2 的 /cmd_vel_nav2）
        self._estop_timer.reset()

        min_dist, bearing = min(moving_close, key=lambda x: x[0])
        event = String()
        event.data = (
            f'software_estop '
            f'timestamp={now:.3f} '
            f'min_dist={min_dist:.2f} '
            f'bearing_deg={bearing:.1f} '
            f'save_last_5s=true'
        )
        self._event_pub.publish(event)
        self.get_logger().warn(
            f'[ESTOP] moving obstacle at {min_dist:.2f}m / {bearing:.1f}° — zero-vel active'
        )

    def _release_estop(self) -> None:
        self._in_estop      = False
        self._confirm_count = 0
        self._clear_count   = 0
        self._estop_timer.cancel()
        self.get_logger().info('[ESTOP] released — no approaching obstacle, navigation resumes')

    def _publish_zero_vel(self) -> None:
        """急停定时器回调：持续向 /cmd_vel_safety 发零速。"""
        self._vel_pub.publish(self._zero_twist)


def main(args=None):
    rclpy.init(args=args)
    node = SafetyMonitor()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
