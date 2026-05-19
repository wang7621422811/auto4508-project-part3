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
from rclpy.node import Node
from std_msgs.msg import Bool, String
from std_srvs.srv import Trigger


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
        self._enable_pub = self.create_publisher(Bool, '/part3/exploration/enable', 10)

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
