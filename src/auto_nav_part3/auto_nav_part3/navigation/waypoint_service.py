import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from std_srvs.srv import Trigger


class WaypointService(Node):
    """Starts the rapid waypoint driving phase through a ROS2 service."""

    def __init__(self):
        super().__init__('part3_waypoint_service')
        self.service = self.create_service(Trigger, '/part3/waypoint/start', self.start_waypoint_run)
        self.state_pub = self.create_publisher(String, '/part3/system/state', 10)
        self.plan_pub = self.create_publisher(String, '/part3/waypoint/plan', 10)
        self.get_logger().info('Waypoint service ready on /part3/waypoint/start')

    def start_waypoint_run(self, request, response):
        del request
        self._publish(self.state_pub, 'WAYPOINT_DRIVE')
        self._publish(self.plan_pub, 'placeholder_plan: home -> target_1 -> target_2 -> target_3 -> home')
        response.success = True
        response.message = 'Waypoint phase started. Replace placeholder with fastest-path planner.'
        return response

    @staticmethod
    def _publish(publisher, text):
        msg = String()
        msg.data = text
        publisher.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = WaypointService()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
