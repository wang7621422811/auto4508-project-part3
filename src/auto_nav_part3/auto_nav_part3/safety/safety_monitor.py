"""
C_S.1 — Safety Monitor: moving obstacle detection + software e-stop

Algorithm (on every /scan message):
  1. If robot angular velocity |wz| > rotation_skip_wz_rad → reset prev_ranges, skip frame
     (when turning, a beam sweeping past a wall corner produces a large range jump that
      does not represent an approaching obstacle)
  2. delta[i] = prev_range[i] - curr_range[i]  (positive = approaching)
  3. moving_close = {i : delta[i] > moving_delta_m AND curr_range[i] < estop_distance_m}
  4. moving_close non-empty → confirm_count++, else reset
  5. confirm_count >= consecutive_frames AND cooldown elapsed → trigger e-stop
  6. During e-stop: timer publishes zero velocity to /cmd_vel_safety at publish_rate_hz
  7. consecutive_frames with no approaching beams → release e-stop

The e-stop publishes via twist_mux (priority=100), overriding Nav2's /cmd_vel_nav2
(priority=10). The Nav2 goal is never cancelled, so navigation resumes automatically
when the obstacle clears.

rotation_skip_wz_rad rationale:
  When rotating, the same lidar beam points in different directions between frames.
  Near a wall corner range can jump 5 m → 0.3 m (delta = 4.7 m), far exceeding the
  threshold and causing a false e-stop. Subscribing to /odometry/filtered for wz and
  skipping frame comparison during turns eliminates these false positives.
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
    Subscribes to /scan, detects approaching moving obstacles, and triggers a software
    e-stop when an obstacle closes to within estop_distance_m.

    Publishes to /cmd_vel_safety (routed by twist_mux at highest priority to /cmd_vel).
    The Nav2 goal is never cancelled; navigation resumes automatically when the obstacle clears.
    """

    def __init__(self):
        super().__init__('part3_safety_monitor')

        # ── parameter declarations (can be overridden by config/safety.yaml) ──
        self.declare_parameter('estop_distance_m',      1.0)
        self.declare_parameter('moving_delta_m',        0.15)
        self.declare_parameter('consecutive_frames',    3)
        self.declare_parameter('estop_cooldown_sec',    2.0)
        self.declare_parameter('publish_rate_hz',       10.0)
        # skip inter-frame comparison when robot angular velocity exceeds this threshold (rad/s)
        # MPPI wz_max=2.5 rad/s; 0.3 rad/s filters the vast majority of Nav2 turning scenarios
        self.declare_parameter('rotation_skip_wz_rad', 0.3)

        self._dist        = float(self.get_parameter('estop_distance_m').value)
        self._delta       = float(self.get_parameter('moving_delta_m').value)
        self._req_frames  = int(self.get_parameter('consecutive_frames').value)
        self._cooldown    = float(self.get_parameter('estop_cooldown_sec').value)
        self._rate        = float(self.get_parameter('publish_rate_hz').value)
        self._skip_wz     = float(self.get_parameter('rotation_skip_wz_rad').value)

        # ── state variables ──────────────────────────────────────────────────────
        self._prev_ranges: list[float] | None = None
        self._confirm_count  = 0    # consecutive frames with approaching beams (trigger counter)
        self._clear_count    = 0    # consecutive frames with no approaching beams (release counter)
        self._in_estop       = False
        self._last_estop_ts  = 0.0  # timestamp of last e-stop trigger (seconds)
        self._curr_wz        = 0.0  # latest angular velocity (from /odometry/filtered)

        self._zero_twist = Twist()  # pre-allocated zero-velocity message (all zeros = stop)

        # ── publishers / subscribers ──────────────────────────────────────────────
        # /cmd_vel_safety → twist_mux (priority=100) → /cmd_vel → Gazebo
        self._vel_pub   = self.create_publisher(Twist,  '/cmd_vel_safety',           10)
        self._event_pub = self.create_publisher(String, '/part3/safety/estop_event', 10)
        self._scan_sub  = self.create_subscription(LaserScan, '/scan', self._on_scan, 10)
        # subscribe to EKF-fused odometry to get robot angular velocity for rotation filtering
        self._odom_sub  = self.create_subscription(
            Odometry, '/odometry/filtered', self._on_odom, 10
        )

        # zero-velocity timer for e-stop (starts paused; reset() activates it in trigger_estop)
        self._estop_timer = self.create_timer(1.0 / self._rate, self._publish_zero_vel)
        self._estop_timer.cancel()

        self.get_logger().info(
            f'SafetyMonitor ready — '
            f'dist={self._dist}m  delta={self._delta}m  '
            f'frames={self._req_frames}  cooldown={self._cooldown}s  '
            f'rate={self._rate}Hz  skip_wz={self._skip_wz}rad/s'
        )

    # ── core callbacks ────────────────────────────────────────────────────────────

    def _on_odom(self, msg: Odometry) -> None:
        """Update robot angular velocity from EKF odometry for rotation filtering."""
        self._curr_wz = msg.twist.twist.angular.z

    def _on_scan(self, scan: LaserScan) -> None:
        ranges = list(scan.ranges)

        # first frame, or frame length changed: record only, do not compare
        if self._prev_ranges is None or len(self._prev_ranges) != len(ranges):
            self._prev_ranges = ranges
            return

        # ── rotation filter: corner sweep during turns ≠ approaching obstacle ──
        # While rotating (|wz| > threshold), update prev_ranges but skip obstacle detection
        # to avoid a delta spike from beam-over-corner falsely triggering the e-stop.
        if abs(self._curr_wz) > self._skip_wz:
            self._prev_ranges = ranges
            return

        # ── steps 1-2: compute list of approaching beams ────────────────────────
        moving_close: list[tuple[float, float]] = []   # (range_m, bearing_deg)
        for i, (prev, curr) in enumerate(zip(self._prev_ranges, ranges)):
            if not (math.isfinite(prev) and math.isfinite(curr) and curr > 0.0):
                continue
            if (prev - curr) > self._delta and curr < self._dist:
                angle_deg = math.degrees(scan.angle_min + i * scan.angle_increment)
                moving_close.append((curr, angle_deg))

        self._prev_ranges = ranges
        now = self.get_clock().now().nanoseconds / 1e9

        if moving_close:
            # ── step 3: accumulate confirm frames, reset clear counter ───────────
            self._confirm_count += 1
            self._clear_count    = 0

            # ── step 4: not in e-stop → check whether to trigger ────────────────
            if not self._in_estop:
                cooldown_ok = (now - self._last_estop_ts) >= self._cooldown
                if self._confirm_count >= self._req_frames and cooldown_ok:
                    self._trigger_estop(moving_close, now)
        else:
            # ── no approaching beams: reset trigger counter; accumulate clear frames if in e-stop ──
            self._confirm_count = 0
            if self._in_estop:
                self._clear_count += 1
                if self._clear_count >= self._req_frames:
                    self._release_estop()

    # ── e-stop trigger / release ──────────────────────────────────────────────────

    def _trigger_estop(self, moving_close: list[tuple[float, float]], now: float) -> None:
        self._in_estop      = True
        self._last_estop_ts = now
        self._clear_count   = 0

        # start timer to continuously publish zero velocity (overrides Nav2's /cmd_vel_nav2)
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
        """E-stop timer callback: continuously publish zero velocity to /cmd_vel_safety."""
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
