"""
waypoint_service.py — M_W 路点快速驾驶服务 (C_W.1)

功能
────
  1. 收到 /part3/waypoint/start (Trigger) 时，按优先级读取路点：
       优先级 1: waypoints_file  — 直接读 markers.json（第二趟无需感知节点在线）
       优先级 2: greek_markers 话题 — 等待 perception_adapter 发布（最多 marker_wait_sec）
  2. home 坐标确定逻辑：
       home_coordinate 非空 → 解析 "x,y" 字符串作为固定 home
       home_coordinate 为 "" → 服务触发时从 /odometry/filtered 取机器人当前位置
  3. 暴力枚举 TSP（3 点 = 6 种排列），求 home→p1→p2→p3→home 最短路径
  4. 调用 Nav2 /navigate_through_poses Action，按最优顺序驾驶后返回 home
  5. 全程发布 /part3/waypoint/plan 和 /part3/system/state

两阶段分离方式
──────────────
  第一阶段（建图）：
    ros2 launch ... use_exploration:=true use_waypoint:=false
    ros2 service call /part3/mapping/start std_srvs/srv/Trigger {}

  第二阶段（路点导航）：
    ros2 launch ... use_exploration:=false use_waypoint:=true
    ros2 service call /part3/waypoint/start std_srvs/srv/Trigger {}
    → 自动从 waypoints_file 加载上次探索保存的路点，无需感知节点在线

话题 / 服务
───────────
  服务（Server）：/part3/waypoint/start           std_srvs/Trigger
  订阅：          /odometry/filtered               nav_msgs/Odometry  （动态 home）
  订阅（备用）：  /part3/perception/greek_markers  geometry_msgs/PoseArray
  发布：         /part3/waypoint/plan              std_msgs/String
                 /part3/system/state               std_msgs/String
  Action 客户端：/navigate_through_poses           nav2_msgs/action/NavigateThroughPoses

设计决策
────────
  service callback 在 MultiThreadedExecutor 的 executor 线程执行；
  Nav2 action 的 send_goal / get_result 全部通过 threading.Event 等待 future
  完成，**不调用 rclpy.spin_until_future_complete**，避免 executor 重入死锁。

参数（config/waypoint.yaml）
────────────────────────────
  home_coordinate  str    ''      "x,y" 字符串；空 = 捕获当前机器人位置
  marker_types     str   'all'   从文件加载的类型：'all' / 'greek' / 'colour'
  nav_timeout_sec  float 120.0   整趟导航超时（秒）
  marker_wait_sec  float   3.0   等待 greek_markers 话题的最长秒数
  waypoints_file   str    ''     第一优先：直接读此 JSON 文件；空时退回话题
"""

from __future__ import annotations

import json
import math
import os
import threading
import time
from itertools import permutations
from typing import Optional

import rclpy
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseArray, PoseStamped
from nav_msgs.msg import Odometry
from nav2_msgs.action import NavigateThroughPoses
from nav2_msgs.srv import ClearEntireCostmap
from rclpy.action import ActionClient
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from std_msgs.msg import String
from std_srvs.srv import Trigger


# ─────────────────────────── 纯函数工具 ────────────────────────────────────

def _yaw_to_quat(yaw: float) -> tuple[float, float, float, float]:
    """yaw (rad) → 四元数 (x, y, z, w)，绕 Z 轴旋转。"""
    return 0.0, 0.0, math.sin(yaw / 2.0), math.cos(yaw / 2.0)


def _dist(a: tuple[float, float], b: tuple[float, float]) -> float:
    return math.hypot(a[0] - b[0], a[1] - b[1])


def _best_order(
    home: tuple[float, float],
    markers: list[tuple[float, float]],
) -> list[tuple[float, float]]:
    """暴力枚举 TSP：枚举 markers 的所有排列，选 home→…→home 总距离最短的顺序。"""
    def total(order: tuple) -> float:
        pts = [home, *order, home]
        return sum(_dist(pts[i], pts[i + 1]) for i in range(len(pts) - 1))

    return list(min(permutations(markers), key=total))


def _approach_point(
    prev: tuple[float, float],
    marker: tuple[float, float],
    approach_dist: float,
) -> tuple[float, float]:
    """prev→marker 方向，在 marker 前方 approach_dist 米处的接近点。
    若距离小于 approach_dist，则直接返回 marker（不反向走）。
    """
    dx = marker[0] - prev[0]
    dy = marker[1] - prev[1]
    d = math.hypot(dx, dy)
    if d < 1e-3 or approach_dist <= 0.0:
        return marker
    ratio = max(0.0, (d - approach_dist) / d)
    return (prev[0] + dx * ratio, prev[1] + dy * ratio)


def _format_plan(
    home: tuple[float, float],
    order: list[tuple[float, float]],
) -> str:
    """生成人可读的路径描述字符串，含总距离。"""
    labels = [chr(ord('A') + i) for i in range(len(order))]
    stops = [f'{lbl}({x:.2f},{y:.2f})' for lbl, (x, y) in zip(labels, order)]
    nodes = [f'home({home[0]:.1f},{home[1]:.1f})', *stops,
             f'home({home[0]:.1f},{home[1]:.1f})']
    coords = [home, *order, home]
    total = sum(_dist(coords[i], coords[i + 1]) for i in range(len(coords) - 1))
    return '→'.join(nodes) + f'  dist={total:.1f}m'


# ─────────────────────────── 主节点 ────────────────────────────────────────

class WaypointServiceNode(Node):
    """路点快速驾驶服务节点。"""

    def __init__(self) -> None:
        super().__init__('part3_waypoint_service')

        # ── 参数 ──────────────────────────────────────────────────────────
        # home_coordinate: "x,y" 字符串；空 = 服务触发时捕获当前里程计位置
        self.declare_parameter('home_coordinate',  '')
        # marker_types: 从 JSON 文件加载的路点类型 — 'all' / 'greek' / 'colour'
        self.declare_parameter('marker_types',    'all')
        # approach_dist: 在每个 marker 前方停车的距离（米），避开 costmap 障碍区
        self.declare_parameter('approach_dist',    0.5)
        self.declare_parameter('nav_timeout_sec', 120.0)
        self.declare_parameter('marker_wait_sec',   3.0)
        # 第一优先路点来源：直接读 markers.json 文件（第二趟无需感知节点在线）
        # 为空字符串时退回订阅 /part3/perception/greek_markers 话题
        self.declare_parameter('waypoints_file',    '')

        gp = self.get_parameter
        self._home_coordinate = gp('home_coordinate').get_parameter_value().string_value
        self._marker_types    = gp('marker_types').get_parameter_value().string_value.lower()
        self._approach_dist   = gp('approach_dist').get_parameter_value().double_value
        self._nav_timeout     = gp('nav_timeout_sec').get_parameter_value().double_value
        self._marker_wait     = gp('marker_wait_sec').get_parameter_value().double_value
        self._waypoints_file  = gp('waypoints_file').get_parameter_value().string_value

        # 解析固定 home（非空时）；空时运行时从里程计捕获
        self._fixed_home: Optional[tuple[float, float]] = None
        if self._home_coordinate:
            try:
                parts = self._home_coordinate.split(',')
                self._fixed_home = (float(parts[0].strip()), float(parts[1].strip()))
            except Exception:
                self.get_logger().error(
                    f'[WaypointService] home_coordinate 格式错误: "{self._home_coordinate}"，'
                    '期望 "x,y"，将退回里程计自动定位'
                )

        # ── Callback group：允许 service / subscription / action 并发 ─────
        self._cbg = ReentrantCallbackGroup()

        # ── 发布 ──────────────────────────────────────────────────────────
        self._state_pub = self.create_publisher(String, '/part3/system/state', 10)
        self._plan_pub  = self.create_publisher(String, '/part3/waypoint/plan', 10)

        # ── 订阅 /odometry/filtered：缓存机器人当前位置，服务触发时作为 home ──
        self._odom_x: float = 0.0
        self._odom_y: float = 0.0
        self._odom_received: bool = False
        self._odom_lock = threading.Lock()
        self.create_subscription(
            Odometry,
            '/odometry/filtered',
            self._on_odometry,
            10,
            callback_group=self._cbg,
        )

        # ── 订阅 greek_markers（perception_adapter 专用过滤话题）────────────
        self._greek_markers: list[tuple[float, float]] = []
        self._markers_lock = threading.Lock()
        self.create_subscription(
            PoseArray,
            '/part3/perception/greek_markers',
            self._on_greek_markers,
            10,
            callback_group=self._cbg,
        )

        # ── Nav2 Action 客户端 ────────────────────────────────────────────
        self._nav_client = ActionClient(
            self,
            NavigateThroughPoses,
            '/navigate_through_poses',
            callback_group=self._cbg,
        )

        # ── SLAM 暂停客户端 ───────────────────────────────────────────────
        # 调用 /part3/waypoint/start 时暂停 slam_toolbox 新测量处理，
        # 防止地图持续更新导致 Nav2 costmap 反复 resize 而打断导航 goal。
        # pause_new_measurements 只停建图，map→odom TF 继续发布，Nav2 不受影响。
        from slam_toolbox.srv import Pause as SlamPause  # noqa: PLC0415
        self._SlamPause = SlamPause
        self._slam_pause_cli = self.create_client(
            SlamPause,
            '/slam_toolbox/pause_new_measurements',
            callback_group=self._cbg,
        )

        # ── Costmap 清除客户端 ────────────────────────────────────────────
        # 探索后 global_costmap 可能还在用旧边界，机器人 pose 超出边界导致规划失败。
        # 导航前主动 clear，让 static_layer 用最新 /map 重建 costmap。
        self._costmap_clear_cli = self.create_client(
            ClearEntireCostmap,
            '/global_costmap/clear_entirely_global_costmap',
            callback_group=self._cbg,
        )

        # ── Service ───────────────────────────────────────────────────────
        self.create_service(
            Trigger,
            '/part3/waypoint/start',
            self._on_start,
            callback_group=self._cbg,
        )

        home_desc = (
            f'固定 home={self._fixed_home}' if self._fixed_home
            else '动态 home（服务触发时捕获里程计）'
        )
        src = self._waypoints_file if self._waypoints_file else '/part3/perception/greek_markers'
        self.get_logger().info(
            f'WaypointService 就绪  {home_desc}  '
            f'marker_types={self._marker_types}  approach_dist={self._approach_dist}m  '
            f'nav_timeout={self._nav_timeout}s  路点来源: {src}'
        )

    # ════════════════════════════════════════════════════════════════════
    # 订阅回调
    # ════════════════════════════════════════════════════════════════════

    def _on_odometry(self, msg: Odometry) -> None:
        with self._odom_lock:
            self._odom_x = msg.pose.pose.position.x
            self._odom_y = msg.pose.pose.position.y
            self._odom_received = True

    def _on_greek_markers(self, msg: PoseArray) -> None:
        with self._markers_lock:
            self._greek_markers = [(p.position.x, p.position.y) for p in msg.poses]

    # ════════════════════════════════════════════════════════════════════
    # Service 回调
    # ════════════════════════════════════════════════════════════════════

    def _on_start(self, _req, response: Trigger.Response) -> Trigger.Response:
        markers = self._collect_markers()
        if not markers:
            msg = (
                'No markers found. '
                'Check waypoints_file path or run exploration first.'
            )
            self.get_logger().warn(f'[WaypointService] {msg}')
            response.success = False
            response.message = msg
            return response

        # ── 确定 home 坐标 ────────────────────────────────────────────────
        if self._fixed_home is not None:
            home = self._fixed_home
            self.get_logger().info(
                f'[WaypointService] 使用固定 home: {home}'
            )
        else:
            with self._odom_lock:
                odom_ok = self._odom_received
                home = (self._odom_x, self._odom_y)
            if odom_ok:
                self.get_logger().info(
                    f'[WaypointService] 动态 home（当前里程计位置）: '
                    f'({home[0]:.3f}, {home[1]:.3f})'
                )
            else:
                self.get_logger().warn(
                    '[WaypointService] 未收到 /odometry/filtered，home 将使用 (0.0, 0.0)，'
                    '建议设置 home_coordinate 参数或等待里程计就绪'
                )

        # ── TSP 排序 ──────────────────────────────────────────────────────
        order = _best_order(home, markers) if len(markers) > 1 else list(markers)

        # ── 计算每个路点的接近点（在 marker 前方 approach_dist 处停车）──────
        approach_stops: list[tuple[float, float]] = []
        prev = home
        for wp in order:
            ap = _approach_point(prev, wp, self._approach_dist)
            approach_stops.append(ap)
            prev = ap

        plan_str = _format_plan(home, approach_stops)
        self._pub_str(self._plan_pub,  plan_str)
        self._pub_str(self._state_pub, 'WAYPOINT_DRIVE')
        self.get_logger().info(f'[WaypointService] 路径规划: {plan_str}')

        # 导航在后台线程运行，服务立即返回 success=True（"命令已接受"语义）
        threading.Thread(
            target=self._navigate,
            args=(approach_stops, home),
            daemon=True,
            name='waypoint_nav',
        ).start()

        response.success = True
        response.message = f'Waypoint run started: {plan_str}'
        return response

    # ════════════════════════════════════════════════════════════════════
    # marker 读取（含等待 + manual 回退）
    # ════════════════════════════════════════════════════════════════════

    def _collect_markers(self) -> list[tuple[float, float]]:
        """按优先级读取路点：
          1. waypoints_file 参数指定的 JSON 文件（第二趟推荐，无需感知节点在线）
          2. /part3/perception/greek_markers 话题（等待至多 marker_wait_sec 秒）
        """
        # ── 优先级 1：直接读 JSON 文件 ────────────────────────────────────
        if self._waypoints_file:
            pts = self._load_from_json(self._waypoints_file)
            if pts:
                return pts
            self.get_logger().warn(
                f'[WaypointService] {self._waypoints_file} 中无符合 '
                f'marker_types="{self._marker_types}" 的条目，'
                '尝试 /part3/perception/greek_markers 话题...'
            )

        # ── 优先级 2：等待感知话题 ────────────────────────────────────────
        deadline = time.monotonic() + self._marker_wait
        while time.monotonic() < deadline:
            with self._markers_lock:
                pts = list(self._greek_markers)
            if pts:
                self.get_logger().info(
                    f'[WaypointService] 从话题获取到 {len(pts)} 个 greek_marker: {pts}'
                )
                return pts
            time.sleep(0.1)

        self.get_logger().warn(
            f'[WaypointService] 等待 {self._marker_wait}s 后仍无路点。'
            '请设置 waypoints_file 参数指向 markers.json，或确认 perception_adapter 在线。'
        )
        return []

    def _load_from_json(self, path: str) -> list[tuple[float, float]]:
        """从 markers.json 按 marker_types 过滤加载路点，返回 (x, y) 列表。
        marker_types='all'    → 加载所有含 x/y 字段的条目
        marker_types='greek'  → 只加载 type=greek
        marker_types='colour' → 只加载 type=colour
        """
        path = os.path.expanduser(path)
        if not os.path.exists(path):
            self.get_logger().warn(f'[WaypointService] waypoints_file 不存在: {path}')
            return []
        try:
            with open(path, 'r', encoding='utf-8') as fh:
                data = json.load(fh)

            if self._marker_types == 'all':
                pts = [
                    (float(item['x']), float(item['y']))
                    for item in data
                    if 'x' in item and 'y' in item
                ]
            else:
                pts = [
                    (float(item['x']), float(item['y']))
                    for item in data
                    if item.get('type') == self._marker_types
                    and 'x' in item and 'y' in item
                ]

            self.get_logger().info(
                f'[WaypointService] 从文件加载 {len(pts)} 个路点'
                f'（marker_types={self._marker_types}）: {path}'
            )
            if not pts:
                self.get_logger().warn(
                    f'[WaypointService] {path} 中无 marker_types="{self._marker_types}" 条目'
                )
            return pts
        except Exception as exc:
            self.get_logger().error(f'[WaypointService] 读取 waypoints_file 失败: {exc}')
            return []

    # ════════════════════════════════════════════════════════════════════
    # 导航后台线程（event-based，不调用 spin_until_future_complete）
    # ════════════════════════════════════════════════════════════════════

    def _deactivate_slam(self) -> None:
        """暂停 slam_toolbox 新测量（pause_new_measurements），停止地图更新但保持 TF 发布。
        slam_toolbox 不在线时服务不存在，直接跳过。
        """
        if not self._slam_pause_cli.wait_for_service(timeout_sec=3.0):
            self.get_logger().info('[WaypointService] slam_toolbox/pause_new_measurements 不可用，跳过')
            return

        req = self._SlamPause.Request()
        done_event: threading.Event = threading.Event()
        future = self._slam_pause_cli.call_async(req)
        future.add_done_callback(lambda _: done_event.set())

        if done_event.wait(timeout=5.0):
            self.get_logger().info('[WaypointService] slam_toolbox 建图已暂停（TF 继续发布）')
        else:
            self.get_logger().warn('[WaypointService] slam_toolbox pause 超时，继续导航')

    def _navigate(
        self,
        order: list[tuple[float, float]],
        home: tuple[float, float],
    ) -> None:
        """后台线程：两阶段导航。
        阶段 1：navigate_through_poses 访问所有路点
        阶段 2：navigate_through_poses 返回 home（单点等效于 navigate_to_pose）
        两阶段分离：home 在地图范围外时阶段 1 仍能成功，不会连带取消路点访问。
        """

        # 停止 SLAM 建图，防止地图持续更新导致 Nav2 costmap 反复 resize 打断导航
        self._deactivate_slam()

        # 等待 action server（Nav2 bt_navigator 启动较慢）
        self.get_logger().info('[WaypointService] 等待 /navigate_through_poses server...')
        if not self._nav_client.wait_for_server(timeout_sec=30.0):
            self.get_logger().error(
                '[WaypointService] /navigate_through_poses server 未就绪，中止'
            )
            self._pub_str(self._state_pub, 'WAYPOINT_FAILED')
            return

        # 清除 global costmap：探索结束后机器人可能在地图远端，
        # costmap 边界尚未更新会导致 "Robot is out of bounds" 规划失败。
        # clear 后 static_layer 用最新 /map 重建，确保 robot pose 在新边界内。
        self._clear_global_costmap()

        # 计算所有路点的 PoseStamped，最后一个路点（home 前方的 approach stop）
        # 的朝向用 home 方向，因此先把 home 加入序列计算朝向，再拆开。
        all_poses = self._build_poses(order + [home])
        waypoint_poses = all_poses[:-1]   # 路点部分（不含 home）
        home_poses     = all_poses[-1:]   # home（单个，yaw=0）

        # ── 阶段 1：访问路点 ──────────────────────────────────────────────
        if waypoint_poses:
            self.get_logger().info(
                f'[WaypointService] 阶段1 发送 goal：{len(waypoint_poses)} 个路点'
            )
            ok = self._run_nav(waypoint_poses, timeout=self._nav_timeout)
            if not ok:
                self.get_logger().error('[WaypointService] 路点访问失败')
                self._pub_str(self._state_pub, 'WAYPOINT_FAILED')
                return
            self.get_logger().info('[WaypointService] 阶段1 完成，所有路点已到达')

        # ── 阶段 2：返回 home ─────────────────────────────────────────────
        self.get_logger().info(
            f'[WaypointService] 阶段2 返回 home ({home[0]:.2f}, {home[1]:.2f})'
        )
        ok = self._run_nav(home_poses, timeout=60.0)
        if ok:
            self.get_logger().info('[WaypointService] 已返回 home，任务完成')
            self._pub_str(self._state_pub, 'COMPLETE')
        else:
            self.get_logger().warn(
                f'[WaypointService] 路点已全部到达，但无法返回 home '
                f'({home[0]:.2f}, {home[1]:.2f})。'
                'home 可能在地图范围外，请在 waypoint.yaml 设置 home_coordinate。'
            )
            self._pub_str(self._state_pub, 'WAYPOINT_COMPLETE')

    def _run_nav(self, poses: list, timeout: float) -> bool:
        """向 /navigate_through_poses 发送 goal 并等待结果，返回是否成功。
        event-based 等待，不调用 spin_until_future_complete（避免 executor 重入）。
        """
        goal = NavigateThroughPoses.Goal(poses=poses)

        # ── 步骤 1：发送 goal，等待 goal handle ──────────────────────────
        gh_event: threading.Event = threading.Event()
        gh_box: list[Optional[object]] = [None]

        def _on_goal_response(future):
            gh_box[0] = future.result()
            gh_event.set()

        send_future = self._nav_client.send_goal_async(goal)
        send_future.add_done_callback(_on_goal_response)

        if not gh_event.wait(timeout=10.0):
            self.get_logger().error('[WaypointService] Goal 发送超时')
            return False

        goal_handle = gh_box[0]
        if goal_handle is None or not goal_handle.accepted:
            self.get_logger().error('[WaypointService] Goal 被 Nav2 拒绝')
            return False

        self.get_logger().info('[WaypointService] Goal 已接受，等待导航完成...')

        # ── 步骤 2：等待导航结果 ──────────────────────────────────────────
        result_event: threading.Event = threading.Event()
        result_box: list[Optional[object]] = [None]

        def _on_result(future):
            result_box[0] = future.result()
            result_event.set()

        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(_on_result)

        if not result_event.wait(timeout=timeout):
            self.get_logger().error(
                f'[WaypointService] 导航超时 ({timeout}s)，取消 goal'
            )
            goal_handle.cancel_goal_async()
            return False

        result = result_box[0]
        if result is None:
            self.get_logger().error('[WaypointService] 结果为空')
            return False

        if result.status == GoalStatus.STATUS_SUCCEEDED:
            return True

        self.get_logger().warn(
            f'[WaypointService] 导航未成功，GoalStatus={result.status}'
        )
        return False

    # ════════════════════════════════════════════════════════════════════
    # 辅助方法
    # ════════════════════════════════════════════════════════════════════

    def _clear_global_costmap(self) -> None:
        """清除 global costmap，让 static_layer 用最新 /map 重建边界。
        服务不可用或调用失败时只打 warn，不阻断导航流程。
        """
        if not self._costmap_clear_cli.wait_for_service(timeout_sec=3.0):
            self.get_logger().warn(
                '[WaypointService] clear_entirely_global_costmap service 不可用，跳过 clear'
            )
            return

        clear_event: threading.Event = threading.Event()

        def _done(future):
            clear_event.set()

        future = self._costmap_clear_cli.call_async(ClearEntireCostmap.Request())
        future.add_done_callback(_done)

        if clear_event.wait(timeout=5.0):
            self.get_logger().info('[WaypointService] global costmap 已清除，等待重建...')
            time.sleep(1.5)  # 等 static_layer 接收最新 /map 并重建
        else:
            self.get_logger().warn('[WaypointService] costmap clear 超时，继续导航')

    def _build_poses(
        self, waypoints: list[tuple[float, float]]
    ) -> list[PoseStamped]:
        """把坐标序列转为 PoseStamped 列表。每个 pose 朝向下一个路点。"""
        now = self.get_clock().now().to_msg()
        n = len(waypoints)
        poses: list[PoseStamped] = []

        for i, (x, y) in enumerate(waypoints):
            if i < n - 1:
                # 朝向下一个路点
                dx = waypoints[i + 1][0] - x
                dy = waypoints[i + 1][1] - y
                yaw = math.atan2(dy, dx)
            else:
                yaw = 0.0  # home：朝 +x（与 spawn 初始朝向一致）

            qx, qy, qz, qw = _yaw_to_quat(yaw)

            ps = PoseStamped()
            ps.header.frame_id = 'map'
            ps.header.stamp    = now
            ps.pose.position.x = x
            ps.pose.position.y = y
            ps.pose.position.z = 0.0
            ps.pose.orientation.x = qx
            ps.pose.orientation.y = qy
            ps.pose.orientation.z = qz
            ps.pose.orientation.w = qw
            poses.append(ps)

        return poses

    @staticmethod
    def _pub_str(pub, text: str) -> None:
        msg = String()
        msg.data = text
        pub.publish(msg)


# ─────────────────────────── entry point ───────────────────────────────────

def main(args=None) -> None:
    rclpy.init(args=args)
    node = WaypointServiceNode()
    # MultiThreadedExecutor：service / subscription / action 回调可并发执行
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
