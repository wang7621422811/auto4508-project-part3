#!/usr/bin/env python3
"""
exploration_node.py — Frontier-based 自主探索节点 (M4.C4.1)

================================================================================
算法概述 (Algorithm Overview)
================================================================================

Frontier-based 探索是机器人学中经典的自主建图策略：

  "Frontier" 定义：占用栅格地图中，值为 free (0) 且至少有一个
  4-连通邻居为 unknown (-1) 的格子。Frontier 正好在"已知空白区域"
  与"未知区域"的分界线上 —— 向那里导航就能探索新区域。

主循环流程：
  ┌─────────────────────────────────────────────────────────────────────┐
  │  1. 从 /map 读取最新 OccupancyGrid                                   │
  │  2. 用 numpy 向量运算找所有 frontier cells                           │
  │  3. BFS 对 frontier cells 做连通分量聚类，得到 frontier 区域列表      │
  │  4. 过滤：去掉太小、超出探索边界、已在黑名单里的聚类                  │
  │  5. 对每个候选 frontier 评分：score = cluster_size / distance        │
  │     （信息增益越大、路程越近越好）                                    │
  │  6. 向最高分 frontier 的质心发送 Nav2 NavigateToPose goal            │
  │  7. 等待结果：                                                        │
  │     - 成功 → 步骤 1（重新选）                                        │
  │     - 失败/超时 → 加入黑名单 → 步骤 1                               │
  │  8. 覆盖率 ≥ 阈值 OR 无 frontier → 发布 coverage=done，停止         │
  └─────────────────────────────────────────────────────────────────────┘

激活方式 (Activation)：
  - 参数 auto_start=true：节点启动后立即开始探索（调试用）
  - 话题 /part3/exploration/enable (std_msgs/Bool)：
      发 true  → 开始探索
      发 false → 停止探索（取消当前导航）
    mapping_service (C5.1) 通过此话题激活/停止本节点，不直接耦合。

接口 (Interfaces)：
  订阅：
    /map                       nav_msgs/OccupancyGrid    slam_toolbox 地图
    /part3/exploration/enable  std_msgs/Bool              外部开关
  发布：
    /part3/mapping/map_status  std_msgs/String            探索进度
  Action Client：
    /navigate_to_pose          nav2_msgs/action/NavigateToPose
  TF 查询：
    map → base_link            获取机器人在地图帧的位置

================================================================================
"""

import math
import time
from collections import deque

import numpy as np
import rclpy
import rclpy.duration
import rclpy.time
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import (
    QoSDurabilityPolicy,
    QoSHistoryPolicy,
    QoSProfile,
    QoSReliabilityPolicy,
)

from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import OccupancyGrid
from nav2_msgs.action import NavigateToPose
from std_msgs.msg import Bool, String
from tf2_ros import Buffer, TransformListener


class ExplorationNode(Node):
    """Frontier-based 自主探索节点。"""

    # OccupancyGrid 中各格子的语义值
    _CELL_FREE = 0       # 已知空闲（机器人可通行）
    _CELL_OCC = 100      # 已知占用（障碍）
    _CELL_UNK = -1       # 未知（int8 下为 -1；有时也用 255 uint8）

    def __init__(self):
        super().__init__('exploration_node')

        # ── 参数声明与读取 ──────────────────────────────────────────────────
        # 所有参数均可在 exploration.yaml 中覆盖，或通过命令行传入
        self.declare_parameter('auto_start', False)
        # 探索区域以 home 为中心的边长（米）
        self.declare_parameter('search_area_size', 15.0)
        # home 坐标（map 帧）：auto_set_home=true 时由 TF 自动确定，否则用此参数
        self.declare_parameter('home_x', 0.0)
        self.declare_parameter('home_y', 0.0)
        # auto_set_home=true：节点启动后用机器人当前位置作为 home
        self.declare_parameter('auto_set_home', True)
        # frontier 聚类最小尺寸（格子数）：太小的 frontier 噪声大，直接丢弃
        self.declare_parameter('min_frontier_size', 10)
        # 覆盖率完成阈值：free/(free+unknown) ≥ 此值则停止
        self.declare_parameter('coverage_done_threshold', 0.90)
        # 单次导航超时（秒）：超过后认为卡住，加黑名单换目标
        self.declare_parameter('nav_timeout_sec', 45.0)
        # 黑名单半径（米）：失败的导航目标附近的 frontier 也跳过
        self.declare_parameter('frontier_blacklist_radius', 0.8)
        # 主循环频率（Hz）：每秒几次检查地图和决策
        self.declare_parameter('loop_rate_hz', 1.0)

        # get_parameter().value 在 Pylance 类型推断中可能为 None，
        # 用显式 cast 告知类型检查器真实类型（运行时 declare_parameter 保证非 None）
        self._auto_start: bool = bool(self.get_parameter('auto_start').value)
        self._search_area: float = float(self.get_parameter('search_area_size').value)
        self._home_x: float = float(self.get_parameter('home_x').value)
        self._home_y: float = float(self.get_parameter('home_y').value)
        self._auto_set_home: bool = bool(self.get_parameter('auto_set_home').value)
        self._min_frontier_size: int = int(self.get_parameter('min_frontier_size').value)
        self._coverage_threshold: float = float(self.get_parameter('coverage_done_threshold').value)
        self._nav_timeout: float = float(self.get_parameter('nav_timeout_sec').value)
        self._blacklist_radius: float = float(self.get_parameter('frontier_blacklist_radius').value)

        # ── 内部状态 ────────────────────────────────────────────────────────
        self._active: bool = self._auto_start   # 是否处于探索模式
        self._map: OccupancyGrid | None = None  # 最新地图

        # 导航状态：用"目标代号" goal_id 避免旧回调干扰新目标
        self._goal_id: int = 0              # 单调递增，每次发新目标自增
        self._nav_goal_id: int = -1         # 当前正在等待的目标的 id
        self._nav_in_progress: bool = False
        self._nav_start_time: float = 0.0
        self._current_goal: tuple[float, float] | None = None

        # 黑名单：导航失败/超时的目标坐标列表，探索时跳过附近的 frontier
        self._blacklist: list[tuple[float, float]] = []

        # home 是否已初始化（第一次从 TF 获取）
        self._home_initialized: bool = not self._auto_set_home
        # 探索是否已完成
        self._exploration_done: bool = False

        # ── QoS 配置 ────────────────────────────────────────────────────────
        # slam_toolbox 用 TRANSIENT_LOCAL 发布 /map，订阅端必须一致才能收到历史消息
        _map_qos = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1,
        )

        # ── 订阅者 ──────────────────────────────────────────────────────────
        self.create_subscription(OccupancyGrid, '/map', self._map_cb, _map_qos)
        # 外部激活/停止信号（由 mapping_service C5.1 或命令行发布）
        # 使用 ROS2 默认 QoS（RELIABLE + VOLATILE），与 ros2 topic pub 默认行为兼容。
        # 命令行用法：
        #   ros2 topic pub --once /part3/exploration/enable std_msgs/msg/Bool '{data: true}'
        self.create_subscription(
            Bool, '/part3/exploration/enable', self._enable_cb, 10
        )

        # ── 发布者 ──────────────────────────────────────────────────────────
        self._status_pub = self.create_publisher(String, '/part3/mapping/map_status', 10)

        # ── TF ──────────────────────────────────────────────────────────────
        self._tf_buf = Buffer()
        self._tf_listener = TransformListener(self._tf_buf, self)

        # ── Nav2 Action 客户端 ───────────────────────────────────────────────
        # /navigate_to_pose 是 Nav2 BT navigator 的主要入口
        self._nav_client = ActionClient(self, NavigateToPose, '/navigate_to_pose')

        # ── 主循环定时器 ─────────────────────────────────────────────────────
        loop_hz: float = float(self.get_parameter('loop_rate_hz').value)
        self.create_timer(1.0 / loop_hz, self._loop)

        self.get_logger().info(
            f'exploration_node 启动完成。'
            f' auto_start={self._auto_start}'
            f', area={self._search_area}m×{self._search_area}m'
            f', coverage_threshold={self._coverage_threshold:.0%}'
            f', nav_timeout={self._nav_timeout}s'
        )

    # ═══════════════════════════════════════════════════════════════════════
    #  回调 (Callbacks)
    # ═══════════════════════════════════════════════════════════════════════

    def _map_cb(self, msg: OccupancyGrid) -> None:
        """接收最新 OccupancyGrid 地图。"""
        self._map = msg

    def _enable_cb(self, msg: Bool) -> None:
        """
        外部开关：true 开始探索，false 停止探索。

        mapping_service (C5.1) 收到 /part3/mapping/start 后发 true 到这里，
        不直接调用本节点内部方法，保持解耦。
        """
        if msg.data and not self._active:
            self.get_logger().info('[Enable] 收到启动信号，开始自主探索')
            self._active = True
            self._exploration_done = False
            self._blacklist.clear()
        elif not msg.data and self._active:
            self.get_logger().info('[Enable] 收到停止信号，中止探索')
            self._active = False
            self._cancel_nav()

    # ═══════════════════════════════════════════════════════════════════════
    #  主循环 (Main Loop)
    # ═══════════════════════════════════════════════════════════════════════

    def _loop(self) -> None:
        """
        主决策循环，由定时器驱动（默认 1 Hz）。

        状态机转换：
          未激活         → 直接返回
          无地图         → 等待
          home 未初始化  → 从 TF 读取 home 坐标，等待
          导航进行中     → 仅检查超时
          导航空闲       → 提取 frontier，计算覆盖率，选目标，发送 goal
        """
        if not self._active or self._exploration_done:
            return

        if self._map is None:
            self.get_logger().info('等待 /map...', throttle_duration_sec=5.0)
            return

        # ── 初始化 home 坐标 ─────────────────────────────────────────────
        if not self._home_initialized:
            pose = self._robot_pose()
            if pose is None:
                # TF 还没就绪，等下一轮
                return
            self._home_x, self._home_y = pose
            self._home_initialized = True
            self.get_logger().info(
                f'[Home] 初始化为机器人起始位置 ({self._home_x:.2f}, {self._home_y:.2f})'
            )
            return

        # ── 导航中：仅检查超时 ────────────────────────────────────────────
        if self._nav_in_progress:
            elapsed = time.monotonic() - self._nav_start_time
            if elapsed > self._nav_timeout:
                self.get_logger().warn(
                    f'[Timeout] 导航超时 {elapsed:.0f}s，放弃目标 {self._current_goal}'
                )
                self._cancel_nav()                       # 取消 Nav2 goal
                if self._current_goal:
                    self._blacklist.append(self._current_goal)
                self._current_goal = None
                # _nav_in_progress 已在 _cancel_nav 里清除，下轮重新选 frontier
            return  # 无论是否超时，本轮不重新选目标（等下轮）

        # ── 提取 frontier 并计算覆盖率 ───────────────────────────────────
        frontiers = self._extract_frontiers()
        coverage = self._compute_coverage()

        # 发布状态
        self._pub_status(coverage, frontiers)

        # ── 判断是否完成 ─────────────────────────────────────────────────
        if coverage >= self._coverage_threshold:
            self._finish(f'覆盖率 {coverage:.1%} ≥ 阈值 {self._coverage_threshold:.1%}')
            return
        if len(frontiers) == 0:
            self._finish('无更多可用 frontier（区域已全面覆盖或无法到达）')
            return

        # ── 获取机器人当前位置 ───────────────────────────────────────────
        robot_pose = self._robot_pose()
        if robot_pose is None:
            self.get_logger().warn('TF 查询失败，跳过本轮', throttle_duration_sec=3.0)
            return

        # ── 选择最优 frontier ────────────────────────────────────────────
        goal = self._select_frontier(frontiers, robot_pose)
        if goal is None:
            # 诊断日志：显示每个 frontier 的坐标和距离，便于调试
            diag = ', '.join(
                f'({wx:.1f},{wy:.1f}) d={math.hypot(wx-robot_pose[0], wy-robot_pose[1]):.1f}m sz={sz}'
                for wx, wy, sz in frontiers
            )
            self.get_logger().warn(
                f'所有 {len(frontiers)} 个 frontier 均被过滤 [{diag}]，等待地图更新...',
                throttle_duration_sec=5.0,
            )
            return

        # ── 发送导航目标 ─────────────────────────────────────────────────
        self._send_goal(*goal, *robot_pose)

    # ═══════════════════════════════════════════════════════════════════════
    #  Frontier 提取 (Frontier Extraction)
    # ═══════════════════════════════════════════════════════════════════════

    def _extract_frontiers(self) -> list[tuple[float, float, int]]:
        """
        从当前 OccupancyGrid 提取有效 frontier 列表。

        步骤：
          1. 将 map.data 转为 int8 numpy 矩阵 (height × width)
          2. 用向量化位移操作找 frontier cells：
               free cell 且至少一个 4-连通邻居为 unknown
          3. BFS 聚类：把相邻的 frontier cells 合并为同一 frontier 区域
          4. 过滤：面积过小 / 超出探索边界 / 在黑名单附近 → 丢弃
          5. 返回每个聚类的 (质心_x, 质心_y, 聚类大小) 列表

        Returns:
            list of (world_x, world_y, cluster_size)
        """
        assert self._map is not None  # caller guarantees this
        info = self._map.info
        W = info.width        # 列数
        H = info.height       # 行数
        res = info.resolution         # 分辨率（m/cell）
        ox = info.origin.position.x   # 地图左下角 x（map 帧）
        oy = info.origin.position.y   # 地图左下角 y（map 帧）

        # ── Step 1：构建 int8 numpy 矩阵 ───────────────────────────────────
        # OccupancyGrid.data 是 int8 列表，行优先（row 0 在最前）
        # -1 = unknown, 0 = free, 1-100 = occupied
        grid = np.array(self._map.data, dtype=np.int8).reshape((H, W))

        # ── Step 2：向量化 frontier 检测 ───────────────────────────────────
        # 布尔掩码：哪些格子是 free？哪些是 unknown？
        free_mask = (grid == self._CELL_FREE)
        unk_mask = (grid == self._CELL_UNK)

        # 判断每个格子的 4-连通邻居中是否含有 unknown
        # 技巧：将 unknown 掩码上下左右各偏移一格，取 OR 后与 free 做 AND
        #
        #   has_unk_nbr[r, c] = True iff 任意一个4-邻居 (r±1,c) 或 (r,c±1) 是 unknown
        #
        # 用切片偏移代替 np.roll（避免边界环绕的副作用）
        has_unk_nbr = np.zeros((H, W), dtype=bool)
        if H > 1:
            has_unk_nbr[1:, :] |= unk_mask[:-1, :]   # 上邻居（当前行 r，上邻 r-1）
            has_unk_nbr[:-1, :] |= unk_mask[1:, :]   # 下邻居
        if W > 1:
            has_unk_nbr[:, 1:] |= unk_mask[:, :-1]   # 左邻居
            has_unk_nbr[:, :-1] |= unk_mask[:, 1:]   # 右邻居

        # frontier 掩码：free cell 且有至少一个 unknown 邻居
        frontier_mask = free_mask & has_unk_nbr

        # ── Step 3：BFS 聚类 ───────────────────────────────────────────────
        # 对 frontier cells 做 8-连通聚类（允许对角连接，避免细线断裂）
        visited = np.zeros((H, W), dtype=bool)
        # 获取所有 frontier cell 的 (row, col) 下标对
        frontier_rows, frontier_cols = np.where(frontier_mask)
        clusters: list[list[tuple[int, int]]] = []

        for seed_r, seed_c in zip(frontier_rows, frontier_cols):
            if visited[seed_r, seed_c]:
                continue  # 已归属某聚类，跳过

            # BFS：从 seed 扩展，只向 frontier cells 扩展
            cluster: list[tuple[int, int]] = []
            q: deque[tuple[int, int]] = deque()
            q.append((seed_r, seed_c))
            visited[seed_r, seed_c] = True

            while q:
                r, c = q.popleft()
                cluster.append((r, c))

                # 8 方向邻居（包含对角线）
                for dr in (-1, 0, 1):
                    for dc in (-1, 0, 1):
                        if dr == 0 and dc == 0:
                            continue
                        nr, nc = r + dr, c + dc
                        if (0 <= nr < H and 0 <= nc < W
                                and not visited[nr, nc]
                                and frontier_mask[nr, nc]):
                            visited[nr, nc] = True
                            q.append((nr, nc))

            clusters.append(cluster)

        # ── Step 4：过滤并转换为世界坐标 ──────────────────────────────────
        half = self._search_area / 2.0
        results: list[tuple[float, float, int]] = []

        for cluster in clusters:
            size = len(cluster)

            # 过滤 1：面积太小 → 丢弃（噪声格子，不值得导航）
            if size < self._min_frontier_size:
                continue

            # 计算质心（pixel 坐标，取均值）
            rows_arr = [rc[0] for rc in cluster]
            cols_arr = [rc[1] for rc in cluster]
            cx_px = sum(cols_arr) / size   # 列均值（对应 x 方向）
            cy_px = sum(rows_arr) / size   # 行均值（对应 y 方向）

            # 转换为 map 帧世界坐标（格子中心 = 格子索引 + 0.5）
            wx = ox + (cx_px + 0.5) * res
            wy = oy + (cy_px + 0.5) * res

            # 过滤 2：超出 home±half 边界 → 丢弃（不在探索区域内）
            if abs(wx - self._home_x) > half or abs(wy - self._home_y) > half:
                continue

            # 过滤 3：在黑名单附近 → 丢弃（之前导航失败过的区域）
            if any(math.hypot(wx - bx, wy - by) < self._blacklist_radius
                   for bx, by in self._blacklist):
                continue

            results.append((wx, wy, size))

        return results

    # ═══════════════════════════════════════════════════════════════════════
    #  Frontier 选择 (Frontier Selection)
    # ═══════════════════════════════════════════════════════════════════════

    def _select_frontier(
        self,
        frontiers: list[tuple[float, float, int]],
        robot_pose: tuple[float, float],
    ) -> tuple[float, float] | None:
        """
        从候选 frontier 列表中选择最优目标。

        评分公式：score = cluster_size / distance_to_robot
          - cluster_size 大：代表该 frontier 连接更大的未知区域（信息增益大）
          - distance 小：机器人移动代价小

        这是信息增益 / 路程代价的简化比值，在实践中效果良好。

        Args:
            frontiers: [(wx, wy, cluster_size), ...]
            robot_pose: (rx, ry) 机器人在 map 帧的位置

        Returns:
            (goal_x, goal_y) 或 None
        """
        if not frontiers:
            return None

        rx, ry = robot_pose
        best_score = -1.0
        best_goal: tuple[float, float] | None = None

        for wx, wy, size in frontiers:
            dist = math.hypot(wx - rx, wy - ry)

                # 最小距离保护：避免导航到机器人已经站着的格子（字面意义上的零位移）。
            # 0.3m 仅用于防止数值退化，远比上一版的 1.0m 保守。
            # 真正的"原地打转"问题由 blacklist-on-success 机制处理：
            # 到达目标后把目标加入黑名单，下一轮 _extract_frontiers 会过滤掉它。
            if dist < 0.3:
                continue

            score = size / dist
            if score > best_score:
                best_score = score
                best_goal = (wx, wy)

        return best_goal

    # ═══════════════════════════════════════════════════════════════════════
    #  覆盖率计算 (Coverage Computation)
    # ═══════════════════════════════════════════════════════════════════════

    def _compute_coverage(self) -> float:
        """
        计算 home 周围 search_area×search_area 区域的探索覆盖率。

        覆盖率 = free_cells / (free_cells + unknown_cells)

        说明：
          - occupied cells 不计入分母（它们本身已被"已知"）
          - 只统计探索区域边界框内的格子（超出地图的部分截断）
          - 若区域内全为 occupied（地图刚初始化），返回 0.0

        Returns:
            float in [0.0, 1.0]
        """
        assert self._map is not None  # caller guarantees this
        info = self._map.info
        res = info.resolution
        ox = info.origin.position.x
        oy = info.origin.position.y
        W = info.width
        H = info.height

        half = self._search_area / 2.0

        # 计算探索区域在 pixel 坐标中的范围，并 clamp 到地图边界内
        col_min = max(0, int((self._home_x - half - ox) / res))
        col_max = min(W,  int((self._home_x + half - ox) / res) + 1)
        row_min = max(0, int((self._home_y - half - oy) / res))
        row_max = min(H,  int((self._home_y + half - oy) / res) + 1)

        if col_max <= col_min or row_max <= row_min:
            # 探索区域完全在地图之外（地图还没生长到这里）
            return 0.0

        grid = np.array(self._map.data, dtype=np.int8).reshape((H, W))
        region = grid[row_min:row_max, col_min:col_max]

        free_n = int(np.sum(region == self._CELL_FREE))
        unk_n = int(np.sum(region == self._CELL_UNK))
        total = free_n + unk_n

        if total == 0:
            return 1.0   # 区域内全是障碍格，视为已"已知"

        return free_n / total

    # ═══════════════════════════════════════════════════════════════════════
    #  辅助：获取机器人位置 (Robot Pose via TF)
    # ═══════════════════════════════════════════════════════════════════════

    def _robot_pose(self) -> tuple[float, float] | None:
        """
        通过 TF 查询机器人在 map 帧中的 (x, y)。

        使用 rclpy.time.Time()（对应 tf2_ros 的 "latest available transform"），
        容忍最多 0.5s 的 TF 延迟。

        Returns:
            (x, y) in map frame，或 None（TF 不可用时）
        """
        try:
            t = self._tf_buf.lookup_transform(
                'map',
                'base_link',
                rclpy.time.Time(),
                timeout=rclpy.duration.Duration(seconds=0.5),
            )
            x = t.transform.translation.x
            y = t.transform.translation.y
            return (x, y)
        except Exception as e:
            self.get_logger().warn(
                f'TF map→base_link 查询失败：{e}',
                throttle_duration_sec=5.0,
            )
            return None

    # ═══════════════════════════════════════════════════════════════════════
    #  导航：发送 Goal (Navigation: Send Goal)
    # ═══════════════════════════════════════════════════════════════════════

    def _send_goal(
        self,
        goal_x: float,
        goal_y: float,
        robot_x: float,
        robot_y: float,
    ) -> None:
        """
        向 Nav2 /navigate_to_pose action 发送导航目标。

        目标朝向（yaw）设为从机器人当前位置指向 frontier 的方向，
        使机器人正面朝向未知区域，让 SICK LiDAR（270° 前向 FOV）
        尽可能多地扫描新区域。

        Args:
            goal_x, goal_y: 目标坐标（map 帧）
            robot_x, robot_y: 机器人当前坐标（用于计算目标朝向）
        """
        if not self._nav_client.wait_for_server(timeout_sec=2.0):
            self.get_logger().error('/navigate_to_pose action server 不可用')
            return

        # 计算朝向：机器人 → frontier 方向的 yaw
        dx = goal_x - robot_x
        dy = goal_y - robot_y
        yaw = math.atan2(dy, dx)

        # 构造 NavigateToPose goal 消息
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = PoseStamped()
        goal_msg.pose.header.frame_id = 'map'
        goal_msg.pose.header.stamp = self.get_clock().now().to_msg()
        goal_msg.pose.pose.position.x = goal_x
        goal_msg.pose.pose.position.y = goal_y
        goal_msg.pose.pose.position.z = 0.0
        # yaw → 四元数（绕 Z 轴旋转，只有 z 和 w 分量）
        #   q.z = sin(yaw/2),  q.w = cos(yaw/2)
        goal_msg.pose.pose.orientation.x = 0.0
        goal_msg.pose.pose.orientation.y = 0.0
        goal_msg.pose.pose.orientation.z = math.sin(yaw / 2.0)
        goal_msg.pose.pose.orientation.w = math.cos(yaw / 2.0)

        # 自增目标代号，用于过滤过期的回调
        self._goal_id += 1
        my_id = self._goal_id

        self._nav_in_progress = True
        self._nav_goal_id = my_id
        self._nav_start_time = time.monotonic()
        self._current_goal = (goal_x, goal_y)

        self.get_logger().info(
            f'[Nav→] 目标 #{my_id}：({goal_x:.2f}, {goal_y:.2f})'
            f' yaw={math.degrees(yaw):.0f}°'
        )

        future = self._nav_client.send_goal_async(
            goal_msg,
            feedback_callback=self._fb_cb,
        )
        # 用闭包把 my_id 传入回调，避免回调执行时 self._goal_id 已变化
        future.add_done_callback(lambda f, gid=my_id: self._resp_cb(f, gid))

    # ── Nav2 回调：Goal 被接受/拒绝 ──────────────────────────────────────
    def _resp_cb(self, future, goal_id: int) -> None:
        """
        Nav2 确认/拒绝 goal 时触发。

        goal_id 与当前 self._nav_goal_id 不同时说明这是过期 goal 的回调，
        直接忽略（可能发生在导航超时后又发了新 goal 的情况）。
        """
        if goal_id != self._nav_goal_id:
            return  # 过期回调，忽略

        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().warn(f'[Nav] goal #{goal_id} 被 Nav2 拒绝，加入黑名单')
            if self._current_goal:
                self._blacklist.append(self._current_goal)
            self._nav_in_progress = False
            return

        self.get_logger().info(f'[Nav] goal #{goal_id} 已接受，等待到达...')
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(
            lambda f, gid=goal_id: self._result_cb(f, gid)
        )

    # ── Nav2 回调：导航完成（成功/失败/取消）────────────────────────────
    def _result_cb(self, future, goal_id: int) -> None:
        """
        Nav2 返回导航结果时触发。

        成功：清除黑名单中已过时的附近条目（该区域已探索）。
        失败：将目标加入黑名单，下次跳过附近的 frontier。
        取消：仅更新内部状态。
        """
        if goal_id != self._nav_goal_id:
            return  # 过期回调，忽略

        result = future.result()
        status = result.status

        if status == GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().info(f'[Nav✓] goal #{goal_id} 成功到达 {self._current_goal}')
            # 成功到达后也加入黑名单，防止因 SICK 90° 后向盲区导致该点附近的
            # frontier cells 残留，被下一轮循环重复选为目标（在原地打转）。
            # 使用与失败时相同的 blacklist_radius（默认 0.8m）。
            if self._current_goal:
                self._blacklist.append(self._current_goal)
        elif status == GoalStatus.STATUS_CANCELED:
            self.get_logger().info(f'[Nav] goal #{goal_id} 已取消')
        else:
            # ABORTED 或其他失败状态
            self.get_logger().warn(
                f'[Nav✗] goal #{goal_id} 失败 (status={status})，加入黑名单'
            )
            if self._current_goal:
                self._blacklist.append(self._current_goal)

        self._nav_in_progress = False
        self._current_goal = None

    # ── Nav2 回调：导航进度（距离剩余）────────────────────────────────
    def _fb_cb(self, feedback_msg) -> None:
        """
        导航过程中 Nav2 周期性推送进度。
        只在 DEBUG 级别打印，避免刷屏。
        """
        dist = feedback_msg.feedback.distance_remaining
        self.get_logger().debug(f'  → 剩余距离 {dist:.2f} m')

    # ═══════════════════════════════════════════════════════════════════════
    #  辅助：取消当前导航 / 探索完成
    # ═══════════════════════════════════════════════════════════════════════

    def _cancel_nav(self) -> None:
        """
        异步取消当前 Nav2 goal 并重置导航状态。

        注意：cancel_goal_async() 只是请求取消，Nav2 可能不会立即停止。
        本节点不等待取消确认——重置 _nav_in_progress 后，下一轮主循环
        会直接选择新 frontier 发送新 goal。旧 goal 的取消结果会触发
        _result_cb，但因为 goal_id 不匹配会被忽略。
        """
        # goal_id 递增，使旧回调在 _resp_cb/_result_cb 中被忽略
        self._nav_goal_id = -1
        self._nav_in_progress = False

    def _finish(self, reason: str) -> None:
        """探索完成：发布 done 状态并停止节点主循环。"""
        self.get_logger().info(f'[Done] 探索完成！原因：{reason}')
        coverage = self._compute_coverage()
        self._pub_status(coverage, [], done=True)
        self._active = False
        self._exploration_done = True

    # ═══════════════════════════════════════════════════════════════════════
    #  状态发布 (Status Publishing)
    # ═══════════════════════════════════════════════════════════════════════

    def _pub_status(
        self,
        coverage: float,
        frontiers: list,
        done: bool = False,
    ) -> None:
        """
        发布探索进度到 /part3/mapping/map_status。

        格式（进行中）：coverage=68% frontiers=3 area=15x15
        格式（完成）：  coverage=done coverage_pct=97%
        """
        if done:
            s = f'coverage=done coverage_pct={coverage:.1%}'
        else:
            s = (
                f'coverage={coverage:.0%}'
                f' frontiers={len(frontiers)}'
                f' area={self._search_area:.0f}x{self._search_area:.0f}'
            )
        msg = String()
        msg.data = s
        self._status_pub.publish(msg)
        self.get_logger().info(f'[Status] {s}')


# ════════════════════════════════════════════════════════════════════════════
#  入口
# ════════════════════════════════════════════════════════════════════════════

def main(args=None):
    rclpy.init(args=args)
    node = ExplorationNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
