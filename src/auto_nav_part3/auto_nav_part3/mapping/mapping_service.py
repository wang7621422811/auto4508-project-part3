#!/usr/bin/env python3
"""
mapping_service.py — M5 orchestration node (C5.1)

Responsibility: pure orchestration — no SLAM or exploration logic.
  1. Receives /part3/mapping/start → publishes enable=true to activate exploration_node
  2. Monitors /part3/mapping/map_status → detects coverage=done → updates system state
  3. Maintains /part3/system/state (IDLE / MAPPING / COMPLETE)

Required nodes (must already be running):
  - exploration_node  (M4.C4.1): subscribes to /part3/exploration/enable, runs actual exploration
  - map_manager       (M4.C4.2): auto-listens to map_status, saves map when exploration finishes

Interface:
  service  /part3/mapping/start          std_srvs/Trigger  ← UI / command-line trigger
  publish  /part3/system/state           std_msgs/String   → system state
  publish  /part3/exploration/enable     std_msgs/Bool     → activate/stop exploration node
  subscribe /part3/mapping/map_status    std_msgs/String   ← exploration progress (from exploration_node)

response.success semantics: True = "command accepted", does not mean exploration is complete.
"""

import rclpy
import threading
import time
from geometry_msgs.msg import PoseStamped
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSProfile, QoSReliabilityPolicy
from std_msgs.msg import Bool, String
from std_srvs.srv import Trigger
from tf2_ros import Buffer, TransformException, TransformListener

# TRANSIENT_LOCAL (equivalent to a latched topic): the publisher retains the last message
# so late-joining subscribers (e.g. exploration_node starting after a 45 s delay) receive the current state immediately.
_ENABLE_QOS = QoSProfile(
    reliability=QoSReliabilityPolicy.RELIABLE,
    durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
    depth=1,
)


class MappingService(Node):
    """M5 orchestration: forwards /part3/mapping/start service calls to exploration_node."""

    def __init__(self):
        super().__init__('part3_mapping_service')

        # ── service: UI entry point ──────────────────────────────────────────
        self.create_service(Trigger, '/part3/mapping/start', self._start_cb)

        # ── publishers ───────────────────────────────────────────────────────
        # system state: IDLE / MAPPING / COMPLETE
        self._state_pub = self.create_publisher(String, '/part3/system/state', 10)
        # exploration toggle: true = start, false = stop (exploration_node subscribes)
        # TRANSIENT_LOCAL: exploration_node starts 45 s late; with default VOLATILE
        # the enable=true message would be lost before the subscriber joins, preventing exploration from ever starting.
        self._enable_pub = self.create_publisher(Bool, '/part3/exploration/enable', _ENABLE_QOS)
        self._home_pub = self.create_publisher(PoseStamped, '/part3/home_pose', _ENABLE_QOS)
        self._tf_buf = Buffer()
        self._tf_listener = TransformListener(self._tf_buf, self)

        # ── subscriber: monitor exploration progress ──────────────────────────
        # exploration_node periodically publishes "coverage=68% frontiers=3 area=15x15"
        # and publishes "coverage=done coverage_pct=XX%" when exploration finishes
        self.create_subscription(
            String, '/part3/mapping/map_status', self._status_cb, 10
        )

        # ── internal state ────────────────────────────────────────────────────
        self._active = False   # whether exploration is currently active

        self._pub_state('IDLE')
        self.get_logger().info('Mapping service ready — call /part3/mapping/start to begin.')

    # ── service callback ──────────────────────────────────────────────────────

    def _start_cb(self, _request, response):
        if self._active:
            response.success = True
            response.message = 'Mapping already in progress.'
            return response

        self._active = True
        threading.Thread(
            target=self._publish_home_pose_with_retry,
            daemon=True,
            name='capture_home_pose',
        ).start()
        self._pub_state('MAPPING')
        self._pub_enable(True)   # → tell exploration_node to start

        self.get_logger().info('[Start] exploration activated, state → MAPPING')
        response.success = True
        response.message = 'Mapping started. Monitor /part3/mapping/map_status for progress.'
        return response

    # ── subscription callback ─────────────────────────────────────────────────

    def _status_cb(self, msg: String):
        """
        Monitor exploration progress.
        When exploration_node finishes it publishes "coverage=done ..."; update state accordingly.
        map_manager handles map saving automatically — this node does not need to trigger it.
        """
        if self._active and 'coverage=done' in msg.data:
            self._active = False
            self._pub_enable(False)
            self._pub_state('COMPLETE')
            self.get_logger().info(
                f'[Done] exploration complete ({msg.data.strip()}), state → COMPLETE'
            )

    # ── utility methods ───────────────────────────────────────────────────────

    def _pub_state(self, state: str):
        msg = String()
        msg.data = state
        self._state_pub.publish(msg)

    def _pub_enable(self, enabled: bool):
        msg = Bool()
        msg.data = enabled
        self._enable_pub.publish(msg)

    def _publish_home_pose(self):
        try:
            transform = self._tf_buf.lookup_transform(
                'map',
                'base_link',
                rclpy.time.Time(),
                timeout=rclpy.duration.Duration(seconds=1.0),
            )
        except TransformException as exc:
            self.get_logger().warn(f'[Home] TF map->base_link unavailable: {exc}')
            return

        msg = PoseStamped()
        msg.header.frame_id = 'map'
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.pose.position.x = transform.transform.translation.x
        msg.pose.position.y = transform.transform.translation.y
        msg.pose.position.z = 0.0
        msg.pose.orientation = transform.transform.rotation
        self._home_pub.publish(msg)
        self.get_logger().info(
            f'[Home] saved exploration home in map frame: '
            f'({msg.pose.position.x:.3f}, {msg.pose.position.y:.3f})'
        )
        return True

    def _publish_home_pose_with_retry(self):
        for attempt in range(1, 11):
            if self._publish_home_pose():
                return
            self.get_logger().warn(f'[Home] capture retry {attempt}/10')
            time.sleep(0.5)
        self.get_logger().error(
            '[Home] failed to capture exploration home; waypoint return will need '
            'home_coordinate or a later /part3/home_pose publisher'
        )


def main(args=None):
    rclpy.init(args=args)
    node = MappingService()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
