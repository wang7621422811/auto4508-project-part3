import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from std_srvs.srv import Trigger


class MappingService(Node):
    """Starts the mapping/discovery phase through a ROS2 service."""

    def __init__(self):
        super().__init__('part3_mapping_service')
        self.service = self.create_service(Trigger, '/part3/mapping/start', self.start_mapping)
        self.state_pub = self.create_publisher(String, '/part3/system/state', 10)
        self.map_pub = self.create_publisher(String, '/part3/mapping/map_status', 10)
        self.marker_pub = self.create_publisher(String, '/part3/perception/marker_event', 10)
        self.get_logger().info('Mapping service ready on /part3/mapping/start')

    def start_mapping(self, request, response):
        del request
        self._publish(self.state_pub, 'MAPPING')
        self._publish(self.map_pub, 'mapping_started: search_area=15x15m frame=map')
        self._publish(self.marker_pub, 'placeholder_event: colour=unknown label=none x=0.0 y=0.0')
        response.success = True
        response.message = 'Mapping phase started. Replace placeholder logic with SLAM/exploration implementation.'
        return response

    @staticmethod
    def _publish(publisher, text):
        msg = String()
        msg.data = text
        publisher.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = MappingService()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
