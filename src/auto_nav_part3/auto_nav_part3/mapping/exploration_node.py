#!/usr/bin/env python3
"""
exploration_node.py — Frontier-based 自主探索节点 (M4.C4.1)

================================================================================
算法来源 (Algorithm Reference)
================================================================================

┌───────────────┬──────────────────────────────────┬────────────────────────────────────────────────────┐
│     模块      │               原版               │                  新版（参考代码）                  │
├───────────────┼──────────────────────────────────┼────────────────────────────────────────────────────┤
│ 地图预处理    │ 无                               │ _preprocess_map() 5次迭代形态学滤波                │
├───────────────┼──────────────────────────────────┼────────────────────────────────────────────────────┤
│ frontier 检测 │ numpy 位移（4-连通，与参考相同） │ _compute_frontier_cell_grid() 同算法，明确对应参考 │
├───────────────┼──────────────────────────────────┼────────────────────────────────────────────────────┤
│ 聚类          │ 8-连通 BFS                       │ _compute_frontier_regions() 同算法，明确对应参考   │
├───────────────┼──────────────────────────────────┼────────────────────────────────────────────────────┤
│ 评分          │ size / dist                      │ 0.99*(1/dist) + 0.01*size，极度偏近                │
├───────────────┼──────────────────────────────────┼────────────────────────────────────────────────────┤
│ 可达性检查    │ 无，直接发目标等 20s 超时        │ ComputePathToPose 异步验证，不可达立即跳下一候选   │
├───────────────┼──────────────────────────────────┼────────────────────────────────────────────────────┤
│ 多候选回退    │ 只选 1 个                        │ top-3 依次验证（参考 rank 0→1→2→3）                │
├───────────────┼──────────────────────────────────┼────────────────────────────────────────────────────┤
│ stuck 处理    │ 计数器清黑名单                   │ 保留，作为兜底                                     │
└───────────────┴──────────────────────────────────┴────────────────────────────────────────────────────┘



核心探测逻辑移植自 another_project_reference/frontier_exploration：

  1. preprocessMap       — 5次迭代形态学滤波，消除孤立 unknown 小孔
  2. computeFrontierCellGrid — 4-连通 free-unknown 边界检测
  3. computeFrontierRegions  — 8-连通 BFS 聚类，质心转世界坐标
  4. selectFrontier      — alpha=0.99 加权评分，极度偏近距离
  5. get_reachable_goal  — 发目标前调 ComputePathToPose 验证可达性，
                           最多尝试 top_n 个候选

主循环状态机：
  IDLE → 提取 frontier → 选 top-N 候选 → PATH_CHECK（异步）
  PATH_CHECK → 路径存在 → NAVIGATING
             → 路径不存在 → 试下一个候选
  NAVIGATING → 成功/失败/超时 → IDLE

接口 (Interfaces)：
  订阅：
    /map                       nav_msgs/OccupancyGrid    slam_toolbox 地图
    /part3/exploration/enable  std_msgs/Bool              外部开关
  发布：
    /part3/mapping/map_status  std_msgs/String            探索进度
  Action Client：
    /navigate_to_pose          nav2_msgs/action/NavigateToPose
    /compute_path_to_pose      nav2_msgs/action/ComputePathToPose  (可达性验证)
  TF 查询：
    map → base_link
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

# TRANSIENT_LOCAL：与 mapping_service 的 _enable_pub 匹配。
# 两端同时为 TRANSIENT_LOCAL 才能触发 late-join replay，
# 保证 exploration_node 在 mapping_service 发布 enable=true 后 45s 才启动时仍能收到消息。
_ENABLE_QOS = QoSProfile(
    reliability=QoSReliabilityPolicy.RELIABLE,
    durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
    depth=1,
)

from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import OccupancyGrid
from nav2_msgs.action import ComputePathToPose, NavigateToPose
from std_msgs.msg import Bool, String
from tf2_ros import Buffer, TransformListener


class ExplorationNode(Node):
    """Frontier-based 自主探索节点（参考代码算法移植版）。"""

    _CELL_FREE = 0
    _CELL_OCC  = 100
    _CELL_UNK  = -1

    def __init__(self):
        super().__init__('exploration_node')

        # ── 参数声明 ──────────────────────────────────────────────────────────
        self.declare_parameter('auto_start', False)
        # frontier 搜索边界（m）：17m 给边界 frontier 留余量
        self.declare_parameter('search_area_size', 17.0)
        # coverage 统计边界（m）：必须 ≤ arena 实际边长，避免墙外 unknown 压低分母
        self.declare_parameter('coverage_area_size', 15.0)
        self.declare_parameter('home_x', 0.0)
        self.declare_parameter('home_y', 0.0)
        self.declare_parameter('auto_set_home', True)
        # 参考代码默认 25 cells；15x15 较小场景用 10 避免过度过滤
        self.declare_parameter('min_frontier_size', 10)
        self.declare_parameter('coverage_done_threshold', 0.90)
        # 20s：Pioneer 1.5m/s × 对角线 21m ≈ 14s，留余量
        self.declare_parameter('nav_timeout_sec', 20.0)
        self.declare_parameter('frontier_blacklist_radius', 0.5)
        self.declare_parameter('loop_rate_hz', 1.0)
        # 参考代码 preprocessMap 迭代次数
        self.declare_parameter('preprocess_iters', 5)
        # 可达性检查最多尝试 top-N 候选（参考代码尝试 rank 0→3）
        self.declare_parameter('top_n_candidates', 3)
        # 评分权重：score = alpha*(1/dist) + (1-alpha)*size，0.99 极度偏近
        self.declare_parameter('alpha', 0.99)

        # get_parameter().value 在 Pylance 下推断为 Unknown|None，用 `or default` 消除警告
        # （运行时 declare_parameter 已保证非 None）
        self._auto_start: bool        = bool(self.get_parameter('auto_start').value)
        self._search_area: float      = float(self.get_parameter('search_area_size').value or 17.0)
        self._coverage_area: float    = float(self.get_parameter('coverage_area_size').value or 15.0)
        self._home_x: float           = float(self.get_parameter('home_x').value or 0.0)
        self._home_y: float           = float(self.get_parameter('home_y').value or 0.0)
        self._auto_set_home: bool     = bool(self.get_parameter('auto_set_home').value)
        self._min_frontier_size: int  = int(self.get_parameter('min_frontier_size').value or 10)
        self._coverage_threshold: float = float(self.get_parameter('coverage_done_threshold').value or 0.90)
        self._nav_timeout: float      = float(self.get_parameter('nav_timeout_sec').value or 20.0)
        self._blacklist_radius: float = float(self.get_parameter('frontier_blacklist_radius').value or 0.5)
        self._preprocess_iters: int   = int(self.get_parameter('preprocess_iters').value or 5)
        self._top_n: int              = int(self.get_parameter('top_n_candidates').value or 3)
        self._alpha: float            = float(self.get_parameter('alpha').value or 0.99)

        # ── 内部状态 ──────────────────────────────────────────────────────────
        self._active: bool            = self._auto_start
        self._map: OccupancyGrid | None = None

        # 导航状态
        self._goal_id: int            = 0
        self._nav_goal_id: int        = -1
        self._nav_in_progress: bool   = False
        self._nav_start_time: float   = 0.0
        self._current_goal: tuple[float, float] | None = None

        # 路径检查状态（参考代码 get_reachable_goal）
        self._checking_path: bool     = False
        self._path_check_id: int      = 0
        self._candidates: list[tuple[float, float]] = []
        self._candidate_idx: int      = 0

        # 接近目标时更新 heading 标志（参考代码 set_goal_heading）
        # 防止固定 yaw 导致机器人在到达位置后大幅旋转/倒车
        self._heading_updated: bool   = False
        # 距目标 ≤ 此距离时把目标朝向改为机器人当前朝向（参考代码 0.25m）
        self._heading_update_dist: float = 0.5

        # 黑名单
        self._blacklist: list[tuple[float, float]] = []
        self._visited:   list[tuple[float, float]] = []
        self._visited_radius: float   = 0.10

        # stuck 计数器：frontiers=0 连续 N 轮后清黑名单
        self._stuck_cycles: int       = 0
        self._stuck_reset_cycles: int = 15

        # 上一次计算的覆盖率，供自适应参数使用
        self._last_coverage: float    = 0.0

        self._home_initialized: bool  = not self._auto_set_home
        self._exploration_done: bool  = False

        # ── QoS ──────────────────────────────────────────────────────────────
        _map_qos = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1,
        )

        # ── 订阅 / 发布 ───────────────────────────────────────────────────────
        self.create_subscription(OccupancyGrid, '/map', self._map_cb, _map_qos)
        self.create_subscription(Bool, '/part3/exploration/enable', self._enable_cb, _ENABLE_QOS)
        self._status_pub = self.create_publisher(String, '/part3/mapping/map_status', 10)

        # ── TF ───────────────────────────────────────────────────────────────
        self._tf_buf      = Buffer()
        self._tf_listener = TransformListener(self._tf_buf, self)

        # ── Action 客户端 ─────────────────────────────────────────────────────
        self._nav_client  = ActionClient(self, NavigateToPose,    '/navigate_to_pose')
        self._path_client = ActionClient(self, ComputePathToPose, '/compute_path_to_pose')

        self._nav_server_ready: bool = False
        if self._nav_client.wait_for_server(timeout_sec=5.0):
            self._nav_server_ready = True
            self.get_logger().info('/navigate_to_pose 已就绪')
        else:
            self.get_logger().warn('/navigate_to_pose 5s 内未就绪，将在发目标前重试')

        # ── 主循环定时器 ──────────────────────────────────────────────────────
        loop_hz: float = float(self.get_parameter('loop_rate_hz').value)
        self.create_timer(1.0 / loop_hz, self._loop)

        self.get_logger().info(
            f'exploration_node 启动。 auto_start={self._auto_start}'
            f' search_area={self._search_area}m coverage_area={self._coverage_area}m'
            f' alpha={self._alpha} preprocess_iters={self._preprocess_iters}'
            f' top_n={self._top_n} nav_timeout={self._nav_timeout}s'
        )

    # ═══════════════════════════════════════════════════════════════════════
    #  回调
    # ═══════════════════════════════════════════════════════════════════════

    def _map_cb(self, msg: OccupancyGrid) -> None:
        self._map = msg

    def _enable_cb(self, msg: Bool) -> None:
        if msg.data and not self._active:
            self.get_logger().info('[Enable] 开始自主探索')
            self._active = True
            self._exploration_done = False
            self._blacklist.clear()
            self._visited.clear()
            self._stuck_cycles = 0
        elif not msg.data and self._active:
            self.get_logger().info('[Enable] 停止探索')
            self._active = False
            self._cancel_nav()
            self._checking_path = False
            self._candidates.clear()

    # ═══════════════════════════════════════════════════════════════════════
    #  主循环
    # ═══════════════════════════════════════════════════════════════════════

    def _loop(self) -> None:
        """
        状态机：
          未激活           → 直接返回
          无地图           → 等待
          home 未初始化   → 从 TF 读取，等待
          导航进行中       → 仅检查超时
          路径检查进行中   → 等待异步回调
          空闲             → 提取 frontier → 选 top-N → 异步路径检查
        """
        if not self._active or self._exploration_done:
            return
        if self._map is None:
            self.get_logger().info('Waiting for /map...', throttle_duration_sec=5.0)
            return

        # ── 初始化 home ───────────────────────────────────────────────────────
        if not self._home_initialized:
            pose = self._robot_pose()
            if pose is None:
                return
            self._home_x, self._home_y = pose
            self._home_initialized = True
            self.get_logger().info(f'[Home] ({self._home_x:.2f}, {self._home_y:.2f})')
            return

        # ── 导航中：仅检查超时 ────────────────────────────────────────────────
        if self._nav_in_progress:
            elapsed = time.monotonic() - self._nav_start_time
            if elapsed > self._nav_timeout:
                self.get_logger().warn(
                    f'[Timeout] {elapsed:.0f}s，放弃目标 {self._current_goal}'
                )
                self._cancel_nav()
                if self._current_goal:
                    self._blacklist.append(self._current_goal)
                self._current_goal = None
            return

        # ── 路径检查中：等待异步回调 ──────────────────────────────────────────
        if self._checking_path:
            return

        # ── 空闲：提取 frontier，计算覆盖率 ──────────────────────────────────
        frontiers = self._extract_frontiers()
        coverage  = self._compute_coverage()
        self._last_coverage = coverage   # 供自适应参数使用
        self._pub_status(coverage, frontiers)

        if coverage >= self._coverage_threshold:
            self._finish(f'coverage={coverage:.1%} ≥ {self._coverage_threshold:.1%}')
            return

        if len(frontiers) == 0:
            # frontiers 为空：区分"黑名单死锁"和"真正探索完"
            self._stuck_cycles += 1
            if self._stuck_cycles >= self._stuck_reset_cycles:
                self.get_logger().warn(
                    f'[Stuck×{self._stuck_cycles}] 清空 {len(self._blacklist)} 个黑名单点，强制重试'
                )
                self._blacklist.clear()
                self._stuck_cycles = 0
            else:
                self.get_logger().warn(
                    f'[Stuck {self._stuck_cycles}/{self._stuck_reset_cycles}]'
                    f' coverage={coverage:.1%}，等待地图更新...',
                    throttle_duration_sec=5.0,
                )
            self._pub_status(coverage, frontiers, stuck=True)
            return

        robot_pose = self._robot_pose()
        if robot_pose is None:
            self.get_logger().warn('TF 失败，跳过本轮', throttle_duration_sec=3.0)
            return

        # ── 选 top-N 候选，依次路径检查（参考代码 get_reachable_goal） ────────
        self._candidates  = self._select_frontiers_ranked(frontiers, robot_pose, self._top_n)
        if not self._candidates:
            self.get_logger().warn('所有 frontier 距离过近，跳过', throttle_duration_sec=5.0)
            return

        self._stuck_cycles  = 0
        self._candidate_idx = 0
        self._try_next_candidate(robot_pose)

    # ─── 依次尝试候选（参考代码 rank 0→1→2→3）───────────────────────────────

    def _try_next_candidate(self, robot_pose: tuple[float, float] | None = None) -> None:
        if self._candidate_idx >= len(self._candidates):
            self.get_logger().warn('[Candidates] 所有候选均不可达，等待下轮地图更新')
            return
        goal_x, goal_y = self._candidates[self._candidate_idx]
        self.get_logger().info(
            f'[PathCheck] 候选 {self._candidate_idx + 1}/{len(self._candidates)}'
            f': ({goal_x:.2f}, {goal_y:.2f})'
        )
        self._start_path_check(goal_x, goal_y, robot_pose)

    def _start_path_check(
        self,
        goal_x: float,
        goal_y: float,
        robot_pose: tuple[float, float] | None,
    ) -> None:
        """异步调用 ComputePathToPose 验证路径是否存在（参考代码 getPath）。"""
        if not self._path_client.wait_for_server(timeout_sec=0.5):
            # planner 暂不可用，跳过验证直接发目标
            self.get_logger().warn('[PathCheck] planner 不可用，跳过验证')
            if robot_pose is None:
                robot_pose = self._robot_pose()
            if robot_pose:
                self._do_send_goal(goal_x, goal_y, *robot_pose)
            return

        goal_msg = ComputePathToPose.Goal()
        goal_msg.goal = PoseStamped()
        goal_msg.goal.header.frame_id = 'map'
        goal_msg.goal.header.stamp = self.get_clock().now().to_msg()
        goal_msg.goal.pose.position.x = goal_x
        goal_msg.goal.pose.position.y = goal_y
        goal_msg.use_start = False

        self._checking_path = True
        self._path_check_id += 1
        my_id = self._path_check_id
        xy    = (goal_x, goal_y)

        future = self._path_client.send_goal_async(goal_msg)
        future.add_done_callback(
            lambda f, pid=my_id, g=xy: self._path_resp_cb(f, pid, g)
        )

    def _path_resp_cb(
        self, future, path_id: int, goal_xy: tuple[float, float]
    ) -> None:
        if path_id != self._path_check_id:
            return
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().warn(f'[PathCheck] 请求被拒绝 {goal_xy}，跳至下一候选')
            self._checking_path = False
            self._candidate_idx += 1
            self._try_next_candidate(self._robot_pose())
            return
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(
            lambda f, pid=path_id, g=goal_xy: self._path_result_cb(f, pid, g)
        )

    def _path_result_cb(
        self, future, path_id: int, goal_xy: tuple[float, float]
    ) -> None:
        if path_id != self._path_check_id:
            return
        self._checking_path = False

        result = future.result()
        status = result.status
        path   = result.result.path if result.result else None
        path_ok = (
            status == GoalStatus.STATUS_SUCCEEDED
            and path is not None
            and len(path.poses) > 0
        )

        if path_ok:
            self.get_logger().info(
                f'[PathCheck✓] ({goal_xy[0]:.2f},{goal_xy[1]:.2f})'
                f' 路径可达（{len(path.poses)} poses），开始导航'
            )
            robot_pose = self._robot_pose()
            if robot_pose:
                self._do_send_goal(goal_xy[0], goal_xy[1], *robot_pose)
        else:
            self.get_logger().warn(
                f'[PathCheck✗] ({goal_xy[0]:.2f},{goal_xy[1]:.2f})'
                f' 不可达 (status={status})，加入黑名单，试下一候选'
            )
            self._blacklist.append(goal_xy)
            self._candidate_idx += 1
            self._try_next_candidate(self._robot_pose())

    # ═══════════════════════════════════════════════════════════════════════
    #  Frontier 提取（参考代码三步算法）
    # ═══════════════════════════════════════════════════════════════════════

    def _extract_frontiers(self) -> list[tuple[float, float, int]]:
        """
        三步流程（完全按参考代码）：
          1. _preprocess_map         — preprocessMap，形态学滤波
          2. _compute_frontier_cell_grid — computeFrontierCellGrid，4-连通检测
          3. _compute_frontier_regions   — computeFrontierRegions，8-连通 BFS
        再叠加搜索边界 / 黑名单 / visited 过滤。
        """
        assert self._map is not None
        info = self._map.info
        W, H  = info.width, info.height
        res   = info.resolution
        ox    = info.origin.position.x
        oy    = info.origin.position.y

        # ── 自适应参数（高覆盖率阶段专用）──────────────────────────────────────
        # 70%+ 后剩余区域为角落/窄通道，需要：
        #   1. 减少形态学滤波次数（5→2），保留薄层 unknown，防止角落 frontier 被填充
        #   2. 降低最小 cluster 大小（10→4），允许找到小角落 frontier
        cov = self._last_coverage
        if cov >= 0.80:
            effective_iters    = 2
            effective_min_size = 3
        elif cov >= 0.65:
            effective_iters    = 3
            effective_min_size = max(4, self._min_frontier_size // 2)
        else:
            effective_iters    = self._preprocess_iters
            effective_min_size = self._min_frontier_size

        # Step 1：形态学预处理（自适应迭代次数）
        grid = np.array(self._map.data, dtype=np.int8).reshape((H, W))
        grid = self._preprocess_map(grid, effective_iters)

        # Step 2：4-连通 frontier cell 检测
        frontier_mask = self._compute_frontier_cell_grid(grid, H, W)

        # Step 3：8-连通 BFS 聚类（自适应最小 cluster 大小）
        raw_regions = self._compute_frontier_regions(
            frontier_mask, H, W, res, ox, oy, effective_min_size
        )

        # 搜索边界 + 黑名单 + visited 过滤
        half   = self._search_area / 2.0
        results: list[tuple[float, float, int]] = []
        n_oob = n_bl = n_vis = 0

        for wx, wy, size in raw_regions:
            if abs(wx - self._home_x) > half or abs(wy - self._home_y) > half:
                n_oob += 1
                continue
            if any(math.hypot(wx - bx, wy - by) < self._blacklist_radius
                   for bx, by in self._blacklist):
                n_bl += 1
                continue
            if any(math.hypot(wx - bx, wy - by) < self._visited_radius
                   for bx, by in self._visited):
                n_vis += 1
                continue
            results.append((wx, wy, size))

        if not results and raw_regions:
            self.get_logger().warn(
                f'[Frontier] 全部过滤！'
                f' 原始={len(raw_regions)} 越界={n_oob}'
                f' 失败黑名单={n_bl}(r={self._blacklist_radius}m,共{len(self._blacklist)}点)'
                f' visited={n_vis}(r={self._visited_radius}m,共{len(self._visited)}点)'
            )
        return results

    def _preprocess_map(self, grid: np.ndarray, iters: int | None = None) -> np.ndarray:
        """
        preprocessMap（参考代码）：
        对每个非占用格，统计 8-连通邻居中 free 与 unknown 数量：
          unknown >= free → 保持 unknown（不确定区域向外渗透）
          unknown <  free → 改为 free  （孤立 unknown 小孔被填充）
        iters：默认用 self._preprocess_iters；高覆盖率阶段传入更小值以保留角落 frontier。
        """
        H, W = grid.shape
        n = iters if iters is not None else self._preprocess_iters
        for _ in range(n):
            free_cnt = np.zeros((H, W), dtype=np.int16)
            unk_cnt  = np.zeros((H, W), dtype=np.int16)
            free_m = (grid == self._CELL_FREE).astype(np.int16)
            unk_m  = (grid == self._CELL_UNK).astype(np.int16)

            for dr in (-1, 0, 1):
                for dc in (-1, 0, 1):
                    if dr == 0 and dc == 0:
                        continue
                    rs = slice(max(0, -dr), H + min(0, -dr))
                    rd = slice(max(0,  dr), H + min(0,  dr))
                    cs = slice(max(0, -dc), W + min(0, -dc))
                    cd = slice(max(0,  dc), W + min(0,  dc))
                    free_cnt[rd, cd] += free_m[rs, cs]
                    unk_cnt[rd, cd]  += unk_m[rs, cs]

            out     = grid.copy()
            non_occ = (grid != self._CELL_OCC)
            out[non_occ & (unk_cnt >= free_cnt)] = self._CELL_UNK
            out[non_occ & (unk_cnt <  free_cnt)] = self._CELL_FREE
            grid = out
        return grid

    def _compute_frontier_cell_grid(
        self, grid: np.ndarray, H: int, W: int
    ) -> np.ndarray:
        """
        computeFrontierCellGrid（参考代码，4-连通）：
        free 格子且至少一个 4-连通邻居为 unknown → frontier cell。
        使用切片位移，O(H×W) 向量化，无循环。
        """
        free_mask = (grid == self._CELL_FREE)
        unk_mask  = (grid == self._CELL_UNK)
        has_unk_4 = np.zeros((H, W), dtype=bool)
        if H > 1:
            has_unk_4[1:,  :] |= unk_mask[:-1, :]   # 上邻居
            has_unk_4[:-1, :] |= unk_mask[1:,  :]   # 下邻居
        if W > 1:
            has_unk_4[:,  1:] |= unk_mask[:, :-1]   # 左邻居
            has_unk_4[:, :-1] |= unk_mask[:,  1:]   # 右邻居
        return free_mask & has_unk_4

    def _compute_frontier_regions(
        self,
        frontier_mask: np.ndarray,
        H: int, W: int,
        res: float, ox: float, oy: float,
        min_size: int | None = None,
    ) -> list[tuple[float, float, int]]:
        """
        computeFrontierRegions（参考代码，8-连通 BFS）：
        对 frontier cells 做 8-连通聚类，计算质心世界坐标，过滤小 region。
        min_size：默认用 self._min_frontier_size；高覆盖率阶段传入更小值以保留角落。
        坐标转换：world = origin + (avg_pixel + 0.5) * resolution
        """
        threshold = min_size if min_size is not None else self._min_frontier_size
        visited   = np.zeros((H, W), dtype=bool)
        seed_rows, seed_cols = np.where(frontier_mask)
        results: list[tuple[float, float, int]] = []

        for sr, sc in zip(seed_rows, seed_cols):
            if visited[sr, sc]:
                continue

            q: deque[tuple[int, int]] = deque()
            q.append((sr, sc))
            visited[sr, sc] = True
            sum_c = sum_r = size = 0

            while q:
                r, c = q.popleft()
                sum_c += c
                sum_r += r
                size  += 1
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

            if size < threshold:
                continue

            # 质心坐标（+0.5 对齐格子中心）
            wx = ox + (sum_c / size + 0.5) * res
            wy = oy + (sum_r / size + 0.5) * res
            results.append((wx, wy, size))

        return results

    # ═══════════════════════════════════════════════════════════════════════
    #  Frontier 选择（参考代码 selectFrontier，alpha 加权）
    # ═══════════════════════════════════════════════════════════════════════

    def _select_frontiers_ranked(
        self,
        frontiers: list[tuple[float, float, int]],
        robot_pose: tuple[float, float],
        top_n: int,
    ) -> list[tuple[float, float]]:
        """
        selectFrontier（参考代码 + 自适应 alpha）：
        score = alpha*(1/dist) + (1-alpha)*size
        低覆盖率：alpha=0.99（偏近，快速扩展已知区域）
        高覆盖率：alpha 逐渐降低，给 size 更大权重，让机器人主动前往
                  较远但信息量大的角落，而非在已探索区域周边反复微动。
        返回 top_n 个目标坐标（降序），供依次路径检查。
        """
        rx, ry = robot_pose

        # 自适应 alpha：65% → 0.99 线性降至 85% → 0.40
        cov = self._last_coverage
        if cov <= 0.65:
            alpha = self._alpha                      # 0.99，偏近
        elif cov >= 0.85:
            alpha = 0.40                             # 强调信息增益
        else:
            t     = (cov - 0.65) / (0.85 - 0.65)   # 0→1
            alpha = self._alpha * (1.0 - t) + 0.40 * t

        scored: list[tuple[float, float, float]] = []

        for wx, wy, size in frontiers:
            dist = math.hypot(wx - rx, wy - ry)
            if dist < 0.3:
                continue
            score = alpha * (1.0 / dist) + (1.0 - alpha) * size
            scored.append((score, wx, wy))

        scored.sort(reverse=True)
        return [(wx, wy) for _, wx, wy in scored[:top_n]]

    # ═══════════════════════════════════════════════════════════════════════
    #  覆盖率计算
    # ═══════════════════════════════════════════════════════════════════════

    def _compute_coverage(self) -> float:
        """
        coverage_area×coverage_area 区域内：free / (free + unknown)。
        coverage_area=15m（≤ arena 边长），避免墙外永久 unknown 压低分母。
        """
        assert self._map is not None
        info = self._map.info
        res  = info.resolution
        ox   = info.origin.position.x
        oy   = info.origin.position.y
        W, H = info.width, info.height
        half = self._coverage_area / 2.0

        col_min = max(0, int((self._home_x - half - ox) / res))
        col_max = min(W, int((self._home_x + half - ox) / res) + 1)
        row_min = max(0, int((self._home_y - half - oy) / res))
        row_max = min(H, int((self._home_y + half - oy) / res) + 1)

        if col_max <= col_min or row_max <= row_min:
            return 0.0

        grid   = np.array(self._map.data, dtype=np.int8).reshape((H, W))
        region = grid[row_min:row_max, col_min:col_max]
        free_n = int(np.sum(region == self._CELL_FREE))
        unk_n  = int(np.sum(region == self._CELL_UNK))
        total  = free_n + unk_n
        return 0.0 if total == 0 else free_n / total

    # ═══════════════════════════════════════════════════════════════════════
    #  辅助
    # ═══════════════════════════════════════════════════════════════════════

    def _robot_pose(self) -> tuple[float, float] | None:
        t = self._robot_tf()
        if t is None:
            return None
        return t.transform.translation.x, t.transform.translation.y

    def _robot_tf(self):
        """返回完整 TF（含姿态），供 heading 更新使用；失败返回 None。"""
        try:
            return self._tf_buf.lookup_transform(
                'map', 'base_link',
                rclpy.time.Time(),
                timeout=rclpy.duration.Duration(seconds=0.5),
            )
        except Exception as e:
            self.get_logger().warn(f'TF map→base_link 失败: {e}', throttle_duration_sec=5.0)
            return None

    def _do_send_goal(self, goal_x: float, goal_y: float, *_: float) -> None:
        if not self._nav_server_ready:
            if self._nav_client.wait_for_server(timeout_sec=0.5):
                self._nav_server_ready = True
            else:
                self.get_logger().warn('/navigate_to_pose 暂不可用，跳过本轮',
                                       throttle_duration_sec=5.0)
                return

        # 探索目标不指定到达朝向（单位四元数 w=1.0）：
        # - 探索只需到达位置，LiDAR 会在任意朝向扫描周围
        # - 强制 yaw 会导致机器人绕行后需要大幅旋转/倒车纠偏
        # - Nav2 yaw_goal_tolerance=0.20rad，单位四元数等效于"朝向东方"；
        #   实际意义：MPPI 规划时不会为了匹配朝向而产生倒车轨迹
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = PoseStamped()
        goal_msg.pose.header.frame_id = 'map'
        goal_msg.pose.header.stamp    = self.get_clock().now().to_msg()
        goal_msg.pose.pose.position.x = goal_x
        goal_msg.pose.pose.position.y = goal_y
        goal_msg.pose.pose.orientation.w = 1.0   # 单位四元数，不强制朝向

        self._goal_id += 1
        my_id = self._goal_id
        self._nav_in_progress  = True
        self._nav_goal_id      = my_id
        self._nav_start_time   = time.monotonic()
        self._current_goal     = (goal_x, goal_y)
        self._heading_updated  = False

        self.get_logger().info(
            f'[Nav→] #{my_id}: ({goal_x:.2f},{goal_y:.2f}) 朝向不锁定'
        )
        future = self._nav_client.send_goal_async(goal_msg, feedback_callback=self._fb_cb)
        future.add_done_callback(lambda f, gid=my_id: self._resp_cb(f, gid))

    def _resp_cb(self, future, goal_id: int) -> None:
        if goal_id != self._nav_goal_id:
            return
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().warn(f'[Nav] #{goal_id} 被 Nav2 拒绝，加入黑名单')
            if self._current_goal:
                self._blacklist.append(self._current_goal)
            self._nav_in_progress = False
            return
        self.get_logger().info(f'[Nav] #{goal_id} 已接受')
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(lambda f, gid=goal_id: self._result_cb(f, gid))

    def _result_cb(self, future, goal_id: int) -> None:
        if goal_id != self._nav_goal_id:
            return
        status = future.result().status
        if status == GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().info(f'[Nav✓] #{goal_id} 到达 {self._current_goal}')
            if self._current_goal:
                self._visited.append(self._current_goal)
        elif status == GoalStatus.STATUS_CANCELED:
            self.get_logger().info(f'[Nav] #{goal_id} 已取消')
        else:
            self.get_logger().warn(f'[Nav✗] #{goal_id} 失败 (status={status})，加入黑名单')
            if self._current_goal:
                self._blacklist.append(self._current_goal)
        self._nav_in_progress = False
        self._current_goal    = None

    def _fb_cb(self, feedback_msg) -> None:
        dist = feedback_msg.feedback.distance_remaining
        self.get_logger().debug(f'  → 剩余 {dist:.2f}m')

        # 参考代码 set_goal_heading()：接近目标时把目标朝向改为机器人当前朝向，
        # 避免到达位置后 Nav2 强制旋转至出发时锁定的 yaw，从而引发倒车。
        if not self._heading_updated and dist <= self._heading_update_dist:
            tf = self._robot_tf()
            if tf is not None and self._current_goal is not None:
                gx, gy = self._current_goal
                # 重发目标：位置不变，朝向改为机器人当前朝向
                goal_msg = NavigateToPose.Goal()
                goal_msg.pose = PoseStamped()
                goal_msg.pose.header.frame_id = 'map'
                goal_msg.pose.header.stamp    = self.get_clock().now().to_msg()
                goal_msg.pose.pose.position.x = gx
                goal_msg.pose.pose.position.y = gy
                goal_msg.pose.pose.orientation = tf.transform.rotation
                self._heading_updated = True
                self.get_logger().info(
                    f'[Heading] 剩余 {dist:.2f}m，更新目标朝向为当前朝向，避免倒车'
                )
                future = self._nav_client.send_goal_async(
                    goal_msg, feedback_callback=self._fb_cb
                )
                self._goal_id += 1
                my_id = self._goal_id
                self._nav_goal_id    = my_id
                self._nav_start_time = time.monotonic()
                future.add_done_callback(lambda f, gid=my_id: self._resp_cb(f, gid))

    def _cancel_nav(self) -> None:
        """取消当前导航（goal_id 递增使旧回调失效）。"""
        self._nav_goal_id     = -1
        self._nav_in_progress = False

    def _finish(self, reason: str) -> None:
        self.get_logger().info(f'[Done] 探索完成！{reason}')
        coverage = self._compute_coverage()
        self._pub_status(coverage, [], done=True)
        self._active           = False
        self._exploration_done = True
        self._return_home()

    def _return_home(self) -> None:
        """探索完成后自动导航回机器人启动位置（home）。"""
        if not self._nav_server_ready:
            if not self._nav_client.wait_for_server(timeout_sec=2.0):
                self.get_logger().warn('[Home] /navigate_to_pose 不可用，无法返回起点')
                return

        self.get_logger().info(
            f'[Home] 返回起点 ({self._home_x:.2f}, {self._home_y:.2f})'
        )
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = PoseStamped()
        goal_msg.pose.header.frame_id = 'map'
        goal_msg.pose.header.stamp    = self.get_clock().now().to_msg()
        goal_msg.pose.pose.position.x = self._home_x
        goal_msg.pose.pose.position.y = self._home_y
        goal_msg.pose.pose.orientation.w = 1.0

        future = self._nav_client.send_goal_async(goal_msg)
        future.add_done_callback(self._home_resp_cb)

    def _home_resp_cb(self, future) -> None:
        """_return_home 目标接受回调：等待导航结果。"""
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().warn('[Home] Nav2 拒绝返回请求')
            return
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self._home_result_cb)

    def _home_result_cb(self, future) -> None:
        """_return_home 导航结果回调：记录成功或失败。"""
        status = future.result().status
        if status == GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().info(
                f'[Home✓] 已到达起点 ({self._home_x:.2f}, {self._home_y:.2f})'
            )
        else:
            self.get_logger().warn(f'[Home✗] 返回起点失败 (status={status})')

    def _pub_status(
        self,
        coverage: float,
        frontiers: list,
        done: bool = False,
        stuck: bool = False,
    ) -> None:
        """
        发布 /part3/mapping/map_status：
          进行中：coverage=68% frontiers=3 area=15x15
          完成：  coverage=done coverage_pct=97%
          卡住：  coverage=stuck coverage_pct=73%
        """
        if done:
            s = f'coverage=done coverage_pct={coverage:.1%}'
        elif stuck:
            s = f'coverage=stuck coverage_pct={coverage:.1%}'
        else:
            s = (
                f'coverage={coverage:.0%}'
                f' frontiers={len(frontiers)}'
                f' area={self._coverage_area:.0f}x{self._coverage_area:.0f}'
            )
        msg      = String()
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
