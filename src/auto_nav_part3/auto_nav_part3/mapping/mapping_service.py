#!/usr/bin/env python3
"""
mapping_service.py — M5 编排节点 (C5.1)

职责：纯编排，不写任何 SLAM/探索逻辑。
  1. 收到 /part3/mapping/start → 发布 enable=true 激活 exploration_node
  2. 监听 /part3/mapping/map_status → 检测 coverage=done → 更新系统状态
  3. 维护 /part3/system/state（IDLE / MAPPING / COMPLETE）

依赖节点（必须已在运行）：
  - exploration_node  （M4.C4.1）：订阅 /part3/exploration/enable，执行实际探索
  - map_manager       （M4.C4.2）：自动监听 map_status，探索完成后保存地图

接口：
  服务  /part3/mapping/start          std_srvs/Trigger  ← UI / 命令行触发
  发布  /part3/system/state           std_msgs/String   → 系统状态
  发布  /part3/exploration/enable     std_msgs/Bool     → 激活/停止探索节点
  订阅  /part3/mapping/map_status     std_msgs/String   ← 探索进度（来自 exploration_node）

response.success 语义：True = "命令已接受"，不代表探索已完成。
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

# TRANSIENT_LOCAL（相当于 latched topic）：发布者保留最后一条消息，
# 新加入的订阅者（如延迟 45s 启动的 exploration_node）可立即收到当前状态。
_ENABLE_QOS = QoSProfile(
    reliability=QoSReliabilityPolicy.RELIABLE,
    durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
    depth=1,
)


class MappingService(Node):
    """M5 编排：把 /part3/mapping/start 服务调用转发给 exploration_node。"""

    def __init__(self):
        super().__init__('part3_mapping_service')

        # ── 服务：UI 调用入口 ────────────────────────────────────────────────
        self.create_service(Trigger, '/part3/mapping/start', self._start_cb)

        # ── 发布者 ───────────────────────────────────────────────────────────
        # 系统状态：IDLE / MAPPING / COMPLETE
        self._state_pub = self.create_publisher(String, '/part3/system/state', 10)
        # 探索开关：true = 开始，false = 停止（exploration_node 订阅此话题）
        # TRANSIENT_LOCAL：exploration_node 延迟 45s 启动，若用默认 VOLATILE，
        # enable=true 消息会在订阅者加入前丢失，导致探索永远无法启动。
        self._enable_pub = self.create_publisher(Bool, '/part3/exploration/enable', _ENABLE_QOS)
        self._home_pub = self.create_publisher(PoseStamped, '/part3/home_pose', _ENABLE_QOS)
        self._tf_buf = Buffer()
        self._tf_listener = TransformListener(self._tf_buf, self)

        # ── 订阅者：监听探索进度 ──────────────────────────────────────────────
        # exploration_node 周期发布 "coverage=68% frontiers=3 area=15x15"
        # 探索完成时发布 "coverage=done coverage_pct=XX%"
        self.create_subscription(
            String, '/part3/mapping/map_status', self._status_cb, 10
        )

        # ── 内部状态 ─────────────────────────────────────────────────────────
        self._active = False   # 当前是否处于探索中

        self._pub_state('IDLE')
        self.get_logger().info('Mapping service ready — call /part3/mapping/start to begin.')

    # ── 服务回调 ─────────────────────────────────────────────────────────────

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
        self._pub_enable(True)   # → exploration_node 开始探索

        self.get_logger().info('[Start] 探索已激活，state → MAPPING')
        response.success = True
        response.message = 'Mapping started. Monitor /part3/mapping/map_status for progress.'
        return response

    # ── 订阅回调 ─────────────────────────────────────────────────────────────

    def _status_cb(self, msg: String):
        """
        监听探索进度。
        exploration_node 完成时发布 "coverage=done ..."，此时更新状态。
        map_manager 自动处理地图保存，本节点无需主动调用。
        """
        if self._active and 'coverage=done' in msg.data:
            self._active = False
            self._pub_enable(False)
            self._pub_state('COMPLETE')
            self.get_logger().info(
                f'[Done] 探索完成（{msg.data.strip()}），state → COMPLETE'
            )

    # ── 工具方法 ─────────────────────────────────────────────────────────────

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
