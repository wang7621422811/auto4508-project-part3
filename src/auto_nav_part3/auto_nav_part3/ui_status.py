import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class UiStatus(Node):
    """Terminal UI placeholder that prints internal state and intended actions."""

    def __init__(self):
        super().__init__('part3_ui_status')
        self.create_subscription(String, '/part3/system/state', self.on_state, 10)
        self.create_subscription(String, '/part3/mapping/map_status', self.on_mapping, 10)
        self.create_subscription(String, '/part3/waypoint/plan', self.on_plan, 10)
        self.get_logger().info('UI placeholder subscribed to Part 3 status topics')

    def on_state(self, msg):
        self.get_logger().info(f'[STATE] {msg.data}')

    def on_mapping(self, msg):
        self.get_logger().info(f'[MAP] {msg.data}')

    def on_plan(self, msg):
        self.get_logger().info(f'[PLAN] {msg.data}')


def main(args=None):
    rclpy.init(args=args)
    node = UiStatus()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
