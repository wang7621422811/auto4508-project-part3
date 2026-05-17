import math

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String


class SafetyMonitor(Node):
    """Minimal safety monitor: publishes STOP when lidar range is below 1m."""

    def __init__(self):
        super().__init__('part3_safety_monitor')
        self.subscription = self.create_subscription(LaserScan, '/scan', self.on_scan, 10)
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.event_pub = self.create_publisher(String, '/part3/safety/estop_event', 10)
        self.threshold_m = 1.0
        self.get_logger().info('Safety monitor watching /scan for obstacles within 1.0m')

    def on_scan(self, scan):
        valid_ranges = [r for r in scan.ranges if math.isfinite(r) and r > 0.0]
        if valid_ranges and min(valid_ranges) < self.threshold_m:
            self.cmd_pub.publish(Twist())
            event = String()
            event.data = 'software_estop: obstacle_within_1m save_last_5_seconds=true'
            self.event_pub.publish(event)


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
