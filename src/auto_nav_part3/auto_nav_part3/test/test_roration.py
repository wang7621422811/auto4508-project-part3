#!/usr/bin/env python3
"""
Rotation accuracy test for Pioneer skid-steer odometry.

Usage:
    python3 test_roration.py
    python3 test_roration.py --speed 0.3 --laps 1

Uses simulation time so results are correct even when Gazebo runs below 1.0x RT.
"""

import argparse
import math
import sys

import rclpy
import rclpy.parameter
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.duration import Duration
from rclpy.node import Node


# ── helpers ──────────────────────────────────────────────────────────────────

def yaw_from_quat(q) -> float:
    siny = 2.0 * (q.w * q.z + q.x * q.y)
    cosy = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny, cosy)


def angle_diff(a: float, b: float) -> float:
    """Shortest signed difference a - b, wrapped to [-pi, pi]."""
    return math.atan2(math.sin(a - b), math.cos(a - b))


def fmt_deg(rad: float) -> str:
    return f"{math.degrees(rad):+.2f}°"


# ── node ─────────────────────────────────────────────────────────────────────

class RotationTester(Node):
    def __init__(self):
        super().__init__(
            "rotation_tester",
            parameter_overrides=[
                rclpy.parameter.Parameter(
                    "use_sim_time",
                    rclpy.parameter.Parameter.Type.BOOL,
                    True,
                )
            ],
        )
        self._pub = self.create_publisher(Twist, "/cmd_vel", 10)
        self._sub = self.create_subscription(
            Odometry, "/odometry/filtered", self._odom_cb, 10
        )
        self.yaw: float | None = None

    def _odom_cb(self, msg: Odometry) -> None:
        self.yaw = yaw_from_quat(msg.pose.pose.orientation)

    # ── low-level helpers ─────────────────────────────────────────────────

    def spin_for(self, sim_seconds: float) -> None:
        """Spin (process callbacks) for `sim_seconds` of simulation time."""
        end = self.get_clock().now() + Duration(seconds=sim_seconds)
        while rclpy.ok() and self.get_clock().now() < end:
            rclpy.spin_once(self, timeout_sec=0.02)

    def send_vel(self, angular_z: float) -> None:
        msg = Twist()
        msg.angular.z = angular_z
        self._pub.publish(msg)

    def stop(self) -> None:
        self.get_logger().info("Stopping robot...")
        t_end = self.get_clock().now() + Duration(seconds=1.5)
        while rclpy.ok() and self.get_clock().now() < t_end:
            self.send_vel(0.0)
            rclpy.spin_once(self, timeout_sec=0.02)

    # ── main test ─────────────────────────────────────────────────────────

    def run(self, angular_speed: float, laps: int) -> float:
        """
        Rotate `laps` × 360° and return the total yaw error in radians.
        Positive error = overshot, negative = undershot.
        """
        log = self.get_logger()

        # 1. Wait for first /odometry/filtered message
        log.info("Waiting for /odometry/filtered ...")
        while rclpy.ok() and self.yaw is None:
            rclpy.spin_once(self, timeout_sec=0.1)
        assert self.yaw is not None
        log.info("  /odometry/filtered received.")

        # 2. Warm up publisher – give DiffDrive plugin time to subscribe
        log.info("Warming up cmd_vel publisher (2 sim-s) ...")
        self.spin_for(2.0)
        log.info("  Warm-up done.")

        # 3. Record start
        assert self.yaw is not None
        start_yaw: float = self.yaw
        target_delta = laps * 2 * math.pi          # total rotation commanded
        duration_sec = target_delta / angular_speed
        log.info(
            f"Starting rotation: angular_z={angular_speed:.2f} rad/s, "
            f"laps={laps}, duration={duration_sec:.2f} sim-s"
        )
        log.info(f"  Start yaw : {fmt_deg(start_yaw)}")

        # 4. Rotate
        t_start = self.get_clock().now()
        t_end   = t_start + Duration(seconds=duration_sec)
        last_log = t_start

        while rclpy.ok() and self.get_clock().now() < t_end:
            self.send_vel(angular_speed)
            rclpy.spin_once(self, timeout_sec=0.02)

            # Progress log every 2 sim-s
            now = self.get_clock().now()
            if (now - last_log).nanoseconds > 2_000_000_000:
                elapsed = (now - t_start).nanoseconds / 1e9
                log.info(
                    f"  [{elapsed:5.1f}/{duration_sec:.1f} sim-s]  "
                    f"current yaw = {fmt_deg(self.yaw)}"
                )
                last_log = now

        # 5. Stop and settle
        self.stop()
        self.spin_for(0.5)          # let odom settle

        # 6. Measure
        assert self.yaw is not None
        end_yaw: float = self.yaw
        actual_delta = target_delta + angle_diff(end_yaw, start_yaw)
        # angle_diff gives the residual error after accounting for full circles
        error = angle_diff(end_yaw, start_yaw)   # shortfall is negative

        log.info(f"  End yaw   : {fmt_deg(end_yaw)}")
        log.info(
            f"  Actual rotation ≈ {math.degrees(actual_delta):.1f}°  "
            f"(commanded {math.degrees(target_delta):.0f}°)"
        )

        return error


# ── entry point ───────────────────────────────────────────────────────────────

def parse_args():
    p = argparse.ArgumentParser(description="Pioneer rotation accuracy test")
    p.add_argument(
        "--speed", type=float, default=0.5,
        help="Angular speed in rad/s (default 0.5)"
    )
    p.add_argument(
        "--laps", type=int, default=1,
        help="Number of full 360° turns (default 1)"
    )
    # strip ROS args before argparse sees them
    args, _ = p.parse_known_args()
    return args


def main():
    args = parse_args()
    rclpy.init()
    node = RotationTester()

    try:
        error_rad = node.run(angular_speed=args.speed, laps=args.laps)
    except KeyboardInterrupt:
        node.stop()
        node.get_logger().info("Test interrupted.")
        rclpy.shutdown()
        sys.exit(1)

    error_deg = math.degrees(error_rad)
    passed = abs(error_deg) < 10.0

    print()
    print("╔══════════════════════════════════════╗")
    print("║      Rotation Test Result            ║")
    print("╠══════════════════════════════════════╣")
    print(f"║  Speed   : {args.speed:.2f} rad/s               ║")
    print(f"║  Laps    : {args.laps}                          ║")
    print(f"║  Error   : {error_deg:+.2f}°                  ║")
    print(f"║  Status  : {'✅ PASS (< 10°)' if passed else '❌ FAIL (≥ 10°)'}            ║")
    print("╚══════════════════════════════════════╝")
    print()

    if not passed:
        pct = abs(error_deg) / (args.laps * 360) * 100
        print(f"  Shortfall = {abs(error_deg):.1f}° = {pct:.1f}% of commanded rotation")
        if abs(error_deg) > 30:
            print("  Hint: check wheel friction (mu2) or wheel_separation in pioneer.urdf")
        else:
            print("  Hint: try tuning wheel_separation in pioneer.urdf")
        print()

    node.destroy_node()
    rclpy.shutdown()
    sys.exit(0 if passed else 1)


if __name__ == "__main__":
    main()
