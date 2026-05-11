import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class StateManager(Node):
    """Publishes the robot's current high-level state for UI and logging."""

    def __init__(self):
        super().__init__('part3_state_manager')
        self.publisher = self.create_publisher(String, '/part3/system/state', 10)
        self.state = 'IDLE'
        self.timer = self.create_timer(1.0, self.publish_state)
        self.get_logger().info('State manager started: IDLE')

    def publish_state(self):
        msg = String()
        msg.data = self.state
        self.publisher.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = StateManager()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
