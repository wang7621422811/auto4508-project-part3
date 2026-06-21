#!/usr/bin/env python3
"""
exploration_node.py — Frontier-based autonomous exploration node (M4.C4.1)

================================================================================
Algorithm Reference
================================================================================

Core frontier detection logic adapted from:
  https://github.com/adrian-soch/frontier_exploration  (MIT Licence)
  Author: Adrian Sochaniwsky
  Based on: B. Yamauchi, "A frontier-based approach for autonomous exploration,"
            Proc. CIRA'97, doi: 10.1109/CIRA.1997.613851

Differences from the reference implementation:

  Module              | Reference                | This Implementation
  --------------------|--------------------------|-------------------------------------------
  Map preprocessing   | None                     | _preprocess_map(): 5-iter morph filter
  Frontier detection  | numpy shift (4-conn)     | _compute_frontier_cell_grid() same alg.
  Clustering          | 8-connected BFS          | _compute_frontier_regions() same alg.
  Scoring             | size / dist              | 0.99*(1/dist)+0.01*size (distance-biased)
  Reachability check  | None (20 s timeout)      | ComputePathToPose async validation
  Multi-candidate     | Top-1 only               | Top-3 verified in sequence (rank 0→3)
  Stuck handling      | Counter clears blacklist | Retained as fallback

Ported functions (from reference):
  1. preprocessMap           — 5-iter morph filter, fills isolated unknown holes
  2. computeFrontierCellGrid — 4-connected free-unknown boundary detection
  3. computeFrontierRegions  — 8-connected BFS clustering, centroid → world coords
  4. selectFrontier          — alpha=0.99 weighted score (strongly distance-biased)
  5. get_reachable_goal      — ComputePathToPose check before sending goal,
                               tries up to top_n candidates

Main-loop state machine:
  IDLE → extract frontiers → select top-N candidates → PATH_CHECK (async)
  PATH_CHECK → path found     → NAVIGATING
             → path not found → try next candidate
  NAVIGATING → success / failure / timeout → IDLE

Interfaces:
  Subscriptions:
    /map                       nav_msgs/OccupancyGrid    SLAM map (slam_toolbox)
    /part3/exploration/enable  std_msgs/Bool             External enable/disable
  Publishers:
    /part3/mapping/map_status  std_msgs/String           Exploration progress
  Action Clients:
    /navigate_to_pose          nav2_msgs/action/NavigateToPose
    /compute_path_to_pose      nav2_msgs/action/ComputePathToPose  (reachability)
  TF:
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

# TRANSIENT_LOCAL matches mapping_service._enable_pub.
# Both ends must be TRANSIENT_LOCAL to trigger late-join replay so that
# exploration_node still receives the enable=true message even if it starts
# 45 s after mapping_service publishes it.
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
    """Frontier-based autonomous exploration node (algorithm ported from reference)."""

    _CELL_FREE = 0
    _CELL_OCC  = 100
    _CELL_UNK  = -1

    def __init__(self):
        super().__init__('exploration_node')

        # ── parameter declarations ─────────────────────────────────────────────
        self.declare_parameter('auto_start', False)
        # search_area_size (m): 17 m gives a margin for frontier cells near the arena wall
        self.declare_parameter('search_area_size', 17.0)
        # coverage_area_size (m): must be ≤ actual arena side length to avoid
        # permanent-unknown wall cells inflating the denominator
        self.declare_parameter('coverage_area_size', 15.0)
        self.declare_parameter('home_x', 0.0)
        self.declare_parameter('home_y', 0.0)
        self.declare_parameter('auto_set_home', True)
        # reference default is 25 cells; 10 suits the smaller 15x15 arena
        self.declare_parameter('min_frontier_size', 10)
        self.declare_parameter('coverage_done_threshold', 0.90)
        # 20 s: Pioneer at 1.5 m/s across 21 m diagonal ≈ 14 s, with margin
        self.declare_parameter('nav_timeout_sec', 20.0)
        self.declare_parameter('frontier_blacklist_radius', 0.5)
        self.declare_parameter('loop_rate_hz', 1.0)
        # number of morphological filter iterations (reference: preprocessMap)
        self.declare_parameter('preprocess_iters', 5)
        # max top-N candidates to try for reachability (reference tries rank 0→3)
        self.declare_parameter('top_n_candidates', 3)
        # scoring weight: score = alpha*(1/dist) + (1-alpha)*size; 0.99 = strongly distance-biased
        self.declare_parameter('alpha', 0.99)
        # return to actual start pose, not the coverage/search home centre
        self.declare_parameter('return_to_start_pose', True)
        # coverage=done triggers map_manager to save; delay before sending home goal
        # to avoid competing with Nav2/SLAM on the same CPU burst
        self.declare_parameter('return_home_delay_sec', 3.0)
        self.declare_parameter('return_home_max_retries', 2)
        self.declare_parameter('return_home_retry_delay_sec', 3.0)

        # get_parameter().value is inferred as Unknown|None by Pylance;
        # use `or default` to suppress the warning (declare_parameter guarantees non-None at runtime)
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
        self._return_to_start: bool   = bool(self.get_parameter('return_to_start_pose').value)
        self._return_home_delay: float = float(self.get_parameter('return_home_delay_sec').value or 3.0)
        self._return_home_max_retries: int = int(self.get_parameter('return_home_max_retries').value or 2)
        self._return_home_retry_delay: float = float(
            self.get_parameter('return_home_retry_delay_sec').value or 3.0
        )

        # ── internal state ────────────────────────────────────────────────────
        self._active: bool            = self._auto_start
        self._map: OccupancyGrid | None = None

        # navigation state
        self._goal_id: int            = 0
        self._nav_goal_id: int        = -1
        self._nav_in_progress: bool   = False
        self._nav_start_time: float   = 0.0
        self._current_goal: tuple[float, float] | None = None

        # path-check state (reference: get_reachable_goal)
        self._checking_path: bool     = False
        self._path_check_id: int      = 0
        self._candidates: list[tuple[float, float]] = []
        self._candidate_idx: int      = 0

        # heading-update flag (reference: set_goal_heading)
        # prevents a locked yaw from forcing a large spin / reverse on arrival
        self._heading_updated: bool   = False
        # update goal heading when distance remaining ≤ this threshold (reference: 0.25 m)
        self._heading_update_dist: float = 0.5

        # blacklist / visited
        self._blacklist: list[tuple[float, float]] = []
        self._visited:   list[tuple[float, float]] = []
        self._visited_radius: float   = 0.10

        # stuck counter: clear blacklist after N consecutive zero-frontier cycles
        self._stuck_cycles: int       = 0
        self._stuck_reset_cycles: int = 15

        # last computed coverage, used for adaptive parameter adjustment
        self._last_coverage: float    = 0.0

        self._home_initialized: bool  = not self._auto_set_home
        self._exploration_done: bool  = False
        self._start_pose: tuple[float, float] | None = None
        self._return_home_attempt: int = 0
        self._return_home_target: tuple[float, float] | None = None

        # ── QoS profiles ──────────────────────────────────────────────────────
        _map_qos = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1,
        )

        # ── subscriptions / publishers ────────────────────────────────────────
        self.create_subscription(OccupancyGrid, '/map', self._map_cb, _map_qos)
        self.create_subscription(Bool, '/part3/exploration/enable', self._enable_cb, _ENABLE_QOS)
        self._status_pub = self.create_publisher(String, '/part3/mapping/map_status', 10)

        # ── TF listener ──────────────────────────────────────────────────────
        self._tf_buf      = Buffer()
        self._tf_listener = TransformListener(self._tf_buf, self)

        # ── action clients ───────────────────────────────────────────────────
        self._nav_client  = ActionClient(self, NavigateToPose,    '/navigate_to_pose')
        self._path_client = ActionClient(self, ComputePathToPose, '/compute_path_to_pose')

        self._nav_server_ready: bool = False
        if self._nav_client.wait_for_server(timeout_sec=5.0):
            self._nav_server_ready = True
            self.get_logger().info('/navigate_to_pose ready')
        else:
            self.get_logger().warn('/navigate_to_pose not ready within 5 s; will retry before sending goal')

        # ── main-loop timer ──────────────────────────────────────────────────
        loop_hz: float = float(self.get_parameter('loop_rate_hz').value)
        self.create_timer(1.0 / loop_hz, self._loop)

        self.get_logger().info(
            f'exploration_node started. auto_start={self._auto_start}'
            f' search_area={self._search_area}m coverage_area={self._coverage_area}m'
            f' alpha={self._alpha} preprocess_iters={self._preprocess_iters}'
            f' top_n={self._top_n} nav_timeout={self._nav_timeout}s'
        )

    # ═══════════════════════════════════════════════════════════════════════
    #  Callbacks
    # ═══════════════════════════════════════════════════════════════════════

    def _map_cb(self, msg: OccupancyGrid) -> None:
        self._map = msg

    def _enable_cb(self, msg: Bool) -> None:
        if msg.data and not self._active:
            self.get_logger().info('[Enable] starting autonomous exploration')
            self._active = True
            self._exploration_done = False
            self._blacklist.clear()
            self._visited.clear()
            self._stuck_cycles = 0
            self._start_pose = None
            self._return_home_attempt = 0
            self._return_home_target = None
        elif not msg.data and self._active:
            self.get_logger().info('[Enable] stopping exploration')
            self._active = False
            self._cancel_nav()
            self._checking_path = False
            self._candidates.clear()

    # ═══════════════════════════════════════════════════════════════════════
    #  Main loop
    # ═══════════════════════════════════════════════════════════════════════

    def _loop(self) -> None:
        """
        State machine:
          inactive         → return immediately
          no map yet       → wait
          home not set     → read from TF, wait
          navigation active → check timeout only
          path-check active → wait for async callback
          idle             → extract frontiers → select top-N → async path check
        """
        if not self._active or self._exploration_done:
            return
        if self._map is None:
            self.get_logger().info('Waiting for /map...', throttle_duration_sec=5.0)
            return

        if self._start_pose is None:
            pose = self._robot_pose()
            if pose is not None:
                self._start_pose = pose
                self.get_logger().info(
                    f'[Start] recorded start pose ({pose[0]:.2f}, {pose[1]:.2f})'
                )

        # ── initialise home position ──────────────────────────────────────────
        if not self._home_initialized:
            pose = self._robot_pose()
            if pose is None:
                return
            self._home_x, self._home_y = pose
            self._home_initialized = True
            self.get_logger().info(f'[Home] ({self._home_x:.2f}, {self._home_y:.2f})')
            return

        # ── navigation in progress: check timeout only ────────────────────────
        if self._nav_in_progress:
            elapsed = time.monotonic() - self._nav_start_time
            if elapsed > self._nav_timeout:
                self.get_logger().warn(
                    f'[Timeout] {elapsed:.0f}s — abandoning goal {self._current_goal}'
                )
                self._cancel_nav()
                if self._current_goal:
                    self._blacklist.append(self._current_goal)
                self._current_goal = None
            return

        # ── path check in progress: wait for async callback ──────────────────
        if self._checking_path:
            return

        # ── idle: extract frontiers and compute coverage ──────────────────────
        frontiers = self._extract_frontiers()
        coverage  = self._compute_coverage()
        self._last_coverage = coverage   # used by adaptive parameters
        self._pub_status(coverage, frontiers)

        if coverage >= self._coverage_threshold:
            self._finish(f'coverage={coverage:.1%} ≥ {self._coverage_threshold:.1%}')
            return

        if len(frontiers) == 0:
            # no frontiers: could be blacklist deadlock or genuinely finished
            self._stuck_cycles += 1
            if self._stuck_cycles >= self._stuck_reset_cycles:
                self.get_logger().warn(
                    f'[Stuck×{self._stuck_cycles}] clearing {len(self._blacklist)} blacklist entries, forcing retry'
                )
                self._blacklist.clear()
                self._stuck_cycles = 0
            else:
                self.get_logger().warn(
                    f'[Stuck {self._stuck_cycles}/{self._stuck_reset_cycles}]'
                    f' coverage={coverage:.1%}, waiting for map update...',
                    throttle_duration_sec=5.0,
                )
            self._pub_status(coverage, frontiers, stuck=True)
            return

        robot_pose = self._robot_pose()
        if robot_pose is None:
            self.get_logger().warn('TF lookup failed, skipping this cycle', throttle_duration_sec=3.0)
            return

        # ── select top-N candidates, check reachability in order (ref: get_reachable_goal) ──
        self._candidates  = self._select_frontiers_ranked(frontiers, robot_pose, self._top_n)
        if not self._candidates:
            self.get_logger().warn('All frontier candidates too close, skipping', throttle_duration_sec=5.0)
            return

        self._stuck_cycles  = 0
        self._candidate_idx = 0
        self._try_next_candidate(robot_pose)

    # ─── try candidates in order (reference: rank 0→1→2→3) ─────────────────

    def _try_next_candidate(self, robot_pose: tuple[float, float] | None = None) -> None:
        if self._candidate_idx >= len(self._candidates):
            self.get_logger().warn('[Candidates] all candidates unreachable; waiting for next map update')
            return
        goal_x, goal_y = self._candidates[self._candidate_idx]
        self.get_logger().info(
            f'[PathCheck] candidate {self._candidate_idx + 1}/{len(self._candidates)}'
            f': ({goal_x:.2f}, {goal_y:.2f})'
        )
        self._start_path_check(goal_x, goal_y, robot_pose)

    def _start_path_check(
        self,
        goal_x: float,
        goal_y: float,
        robot_pose: tuple[float, float] | None,
    ) -> None:
        """Async ComputePathToPose call to verify a path exists (reference: getPath)."""
        if not self._path_client.wait_for_server(timeout_sec=0.5):
            # planner temporarily unavailable; skip check and send goal directly
            self.get_logger().warn('[PathCheck] planner unavailable, skipping path check')
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
            self.get_logger().warn(f'[PathCheck] request rejected {goal_xy}, trying next candidate')
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
                f' path reachable ({len(path.poses)} poses), starting navigation'
            )
            robot_pose = self._robot_pose()
            if robot_pose:
                self._do_send_goal(goal_xy[0], goal_xy[1], *robot_pose)
        else:
            self.get_logger().warn(
                f'[PathCheck✗] ({goal_xy[0]:.2f},{goal_xy[1]:.2f})'
                f' unreachable (status={status}), blacklisted, trying next candidate'
            )
            self._blacklist.append(goal_xy)
            self._candidate_idx += 1
            self._try_next_candidate(self._robot_pose())

    # ═══════════════════════════════════════════════════════════════════════
    #  Frontier extraction (3-step algorithm from reference)
    # ═══════════════════════════════════════════════════════════════════════

    def _extract_frontiers(self) -> list[tuple[float, float, int]]:
        """
        Three-step pipeline (matches reference exactly):
          1. _preprocess_map             — preprocessMap: morphological filter
          2. _compute_frontier_cell_grid — computeFrontierCellGrid: 4-conn detection
          3. _compute_frontier_regions   — computeFrontierRegions: 8-conn BFS
        Followed by search-area / blacklist / visited filtering.
        """
        assert self._map is not None
        info = self._map.info
        W, H  = info.width, info.height
        res   = info.resolution
        ox    = info.origin.position.x
        oy    = info.origin.position.y

        # ── adaptive parameters for high-coverage phase ───────────────────────
        # Above 70% coverage the remaining unexplored area is corners/narrow passages:
        #   1. reduce morph filter iterations (5→2) to preserve thin unknown layers
        #      and avoid corner frontiers being filled in
        #   2. lower minimum cluster size (10→4) to allow small corner frontiers
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

        # Step 1: morphological preprocessing (adaptive iteration count)
        grid = np.array(self._map.data, dtype=np.int8).reshape((H, W))
        grid = self._preprocess_map(grid, effective_iters)

        # Step 2: 4-connected frontier cell detection
        frontier_mask = self._compute_frontier_cell_grid(grid, H, W)

        # Step 3: 8-connected BFS clustering (adaptive minimum cluster size)
        raw_regions = self._compute_frontier_regions(
            frontier_mask, H, W, res, ox, oy, effective_min_size
        )

        # search-area + blacklist + visited filtering
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
                f'[Frontier] all filtered!'
                f' raw={len(raw_regions)} oob={n_oob}'
                f' blacklist={n_bl}(r={self._blacklist_radius}m, {len(self._blacklist)} entries)'
                f' visited={n_vis}(r={self._visited_radius}m, {len(self._visited)} entries)'
            )
        return results

    def _preprocess_map(self, grid: np.ndarray, iters: int | None = None) -> np.ndarray:
        """
        preprocessMap (from reference):
        For each non-occupied cell, count 8-connected neighbours that are free vs unknown:
          unknown >= free → keep unknown  (uncertainty spreads outward)
          unknown <  free → set to free   (isolated unknown holes are filled)
        iters: defaults to self._preprocess_iters; pass a smaller value in high-coverage
               phase to preserve corner frontiers.
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
        computeFrontierCellGrid (from reference, 4-connected):
        A free cell with at least one 4-connected unknown neighbour is a frontier cell.
        Implemented with slice shifts — O(H×W) vectorised, no Python loops.
        """
        free_mask = (grid == self._CELL_FREE)
        unk_mask  = (grid == self._CELL_UNK)
        has_unk_4 = np.zeros((H, W), dtype=bool)
        if H > 1:
            has_unk_4[1:,  :] |= unk_mask[:-1, :]   # top neighbour
            has_unk_4[:-1, :] |= unk_mask[1:,  :]   # bottom neighbour
        if W > 1:
            has_unk_4[:,  1:] |= unk_mask[:, :-1]   # left neighbour
            has_unk_4[:, :-1] |= unk_mask[:,  1:]   # right neighbour
        return free_mask & has_unk_4

    def _compute_frontier_regions(
        self,
        frontier_mask: np.ndarray,
        H: int, W: int,
        res: float, ox: float, oy: float,
        min_size: int | None = None,
    ) -> list[tuple[float, float, int]]:
        """
        computeFrontierRegions (from reference, 8-connected BFS):
        Clusters frontier cells, computes centroid in world coordinates, filters small regions.
        min_size: defaults to self._min_frontier_size; pass smaller value for high-coverage phase.
        Coordinate conversion: world = origin + (avg_pixel + 0.5) * resolution
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

            # centroid in world coords (+0.5 aligns to cell centre)
            wx = ox + (sum_c / size + 0.5) * res
            wy = oy + (sum_r / size + 0.5) * res
            results.append((wx, wy, size))

        return results

    # ═══════════════════════════════════════════════════════════════════════
    #  Frontier selection (reference: selectFrontier, alpha-weighted)
    # ═══════════════════════════════════════════════════════════════════════

    def _select_frontiers_ranked(
        self,
        frontiers: list[tuple[float, float, int]],
        robot_pose: tuple[float, float],
        top_n: int,
    ) -> list[tuple[float, float]]:
        """
        selectFrontier (reference + adaptive alpha):
        score = alpha*(1/dist) + (1-alpha)*size
        Low coverage:  alpha=0.99 (distance-biased, expands known area quickly)
        High coverage: alpha decreases so size gets more weight, driving the robot
                       toward distant information-rich corners instead of micro-jitter
                       around already-explored areas.
        Returns top_n goal coordinates (descending score) for sequential path checks.
        """
        rx, ry = robot_pose

        # adaptive alpha: linearly decreases from 0.99 at 65% coverage to 0.40 at 85%
        cov = self._last_coverage
        if cov <= 0.65:
            alpha = self._alpha                      # 0.99 — strongly distance-biased
        elif cov >= 0.85:
            alpha = 0.40                             # emphasise information gain
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
    #  Coverage computation
    # ═══════════════════════════════════════════════════════════════════════

    def _compute_coverage(self) -> float:
        """
        Coverage = free / (free + unknown) inside the coverage_area × coverage_area region.
        coverage_area=15 m (≤ arena side) avoids permanent-unknown wall cells
        deflating the denominator.
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
    #  Helpers
    # ═══════════════════════════════════════════════════════════════════════

    def _robot_pose(self) -> tuple[float, float] | None:
        t = self._robot_tf()
        if t is None:
            return None
        return t.transform.translation.x, t.transform.translation.y

    def _robot_tf(self):
        """Return the full map→base_link transform (including orientation) for heading updates; returns None on failure."""
        try:
            return self._tf_buf.lookup_transform(
                'map', 'base_link',
                rclpy.time.Time(),
                timeout=rclpy.duration.Duration(seconds=0.5),
            )
        except Exception as e:
            self.get_logger().warn(f'TF map→base_link failed: {e}', throttle_duration_sec=5.0)
            return None

    def _do_send_goal(self, goal_x: float, goal_y: float, *_: float) -> None:
        if not self._nav_server_ready:
            if self._nav_client.wait_for_server(timeout_sec=0.5):
                self._nav_server_ready = True
            else:
                self.get_logger().warn('/navigate_to_pose not yet available, skipping this cycle',
                                       throttle_duration_sec=5.0)
                return

        # Exploration goals do not lock the arrival heading (identity quaternion w=1.0):
        # - exploration only needs to reach a position; LiDAR scans in all directions
        # - forcing a yaw causes large rotation/reverse correction after the robot arrives
        # - Nav2 yaw_goal_tolerance=0.20 rad; identity quaternion is equivalent to "face east"
        #   and prevents MPPI from generating reverse trajectories just to match a locked yaw
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = PoseStamped()
        goal_msg.pose.header.frame_id = 'map'
        goal_msg.pose.header.stamp    = self.get_clock().now().to_msg()
        goal_msg.pose.pose.position.x = goal_x
        goal_msg.pose.pose.position.y = goal_y
        goal_msg.pose.pose.orientation.w = 1.0   # identity quaternion — heading not locked

        self._goal_id += 1
        my_id = self._goal_id
        self._nav_in_progress  = True
        self._nav_goal_id      = my_id
        self._nav_start_time   = time.monotonic()
        self._current_goal     = (goal_x, goal_y)
        self._heading_updated  = False

        self.get_logger().info(
            f'[Nav→] #{my_id}: ({goal_x:.2f},{goal_y:.2f}) heading unlocked'
        )
        future = self._nav_client.send_goal_async(goal_msg, feedback_callback=self._fb_cb)
        future.add_done_callback(lambda f, gid=my_id: self._resp_cb(f, gid))

    def _resp_cb(self, future, goal_id: int) -> None:
        if goal_id != self._nav_goal_id:
            return
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().warn(f'[Nav] #{goal_id} rejected by Nav2, adding to blacklist')
            if self._current_goal:
                self._blacklist.append(self._current_goal)
            self._nav_in_progress = False
            return
        self.get_logger().info(f'[Nav] #{goal_id} accepted')
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(lambda f, gid=goal_id: self._result_cb(f, gid))

    def _result_cb(self, future, goal_id: int) -> None:
        if goal_id != self._nav_goal_id:
            return
        status = future.result().status
        if status == GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().info(f'[Nav✓] #{goal_id} reached {self._current_goal}')
            if self._current_goal:
                self._visited.append(self._current_goal)
        elif status == GoalStatus.STATUS_CANCELED:
            self.get_logger().info(f'[Nav] #{goal_id} cancelled')
        else:
            self.get_logger().warn(f'[Nav✗] #{goal_id} failed (status={status}), adding to blacklist')
            if self._current_goal:
                self._blacklist.append(self._current_goal)
        self._nav_in_progress = False
        self._current_goal    = None

    def _fb_cb(self, feedback_msg) -> None:
        dist = feedback_msg.feedback.distance_remaining
        self.get_logger().debug(f'  → remaining {dist:.2f}m')

        # Reference: set_goal_heading() — when close to goal, update the goal heading to the
        # robot's current heading so Nav2 does not force a large yaw rotation on arrival.
        if not self._heading_updated and dist <= self._heading_update_dist:
            tf = self._robot_tf()
            if tf is not None and self._current_goal is not None:
                gx, gy = self._current_goal
                # Re-send goal: same position, heading updated to current robot orientation
                goal_msg = NavigateToPose.Goal()
                goal_msg.pose = PoseStamped()
                goal_msg.pose.header.frame_id = 'map'
                goal_msg.pose.header.stamp    = self.get_clock().now().to_msg()
                goal_msg.pose.pose.position.x = gx
                goal_msg.pose.pose.position.y = gy
                goal_msg.pose.pose.orientation = tf.transform.rotation
                self._heading_updated = True
                self.get_logger().info(
                    f'[Heading] {dist:.2f}m remaining — updating goal heading to current orientation to avoid reverse'
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
        """Cancel current navigation (incrementing goal_id invalidates stale callbacks)."""
        self._nav_goal_id     = -1
        self._nav_in_progress = False

    def _finish(self, reason: str) -> None:
        self.get_logger().info(f'[Done] Exploration complete! {reason}')
        coverage = self._compute_coverage()
        self._pub_status(coverage, [], done=True)
        self._active           = False
        self._exploration_done = True
        self._checking_path    = False
        self._path_check_id   += 1
        self._return_home()

    def _return_home(self) -> None:
        """Navigate back to the start position after exploration is complete."""
        self._return_home_attempt = 0
        if self._return_to_start and self._start_pose is not None:
            self._return_home_target = self._start_pose
        else:
            self._return_home_target = (self._home_x, self._home_y)
            if self._return_to_start:
                self.get_logger().warn('[Home] start pose not recorded; falling back to configured home coordinates')

        self.get_logger().info(
            f'[Home] returning in {self._return_home_delay:.1f}s to '
            f'({self._return_home_target[0]:.2f}, {self._return_home_target[1]:.2f})'
        )
        self._call_later(self._return_home_delay, self._send_home_goal)

    def _send_home_goal(self) -> None:
        """Send (or retry) the return-to-start goal."""
        if self._return_home_target is None:
            return

        if not self._nav_server_ready:
            if not self._nav_client.wait_for_server(timeout_sec=2.0):
                self.get_logger().warn('[Home] /navigate_to_pose unavailable; cannot return to start')
                self._schedule_home_retry()
                return

        home_x, home_y = self._return_home_target
        self._return_home_attempt += 1
        self.get_logger().info(
            f'[Home] returning to start attempt={self._return_home_attempt}/'
            f'{self._return_home_max_retries + 1} ({home_x:.2f}, {home_y:.2f})'
        )
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = PoseStamped()
        goal_msg.pose.header.frame_id = 'map'
        goal_msg.pose.header.stamp    = self.get_clock().now().to_msg()
        goal_msg.pose.pose.position.x = home_x
        goal_msg.pose.pose.position.y = home_y
        goal_msg.pose.pose.orientation.w = 1.0

        future = self._nav_client.send_goal_async(goal_msg)
        future.add_done_callback(self._home_resp_cb)

    def _home_resp_cb(self, future) -> None:
        """_return_home goal-accepted callback: wait for navigation result."""
        try:
            goal_handle = future.result()
        except Exception as exc:
            self.get_logger().warn(f'[Home] failed to send return goal: {exc}')
            self._schedule_home_retry()
            return
        if not goal_handle.accepted:
            self.get_logger().warn('[Home] Nav2 rejected the return goal')
            self._schedule_home_retry()
            return
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self._home_result_cb)

    def _home_result_cb(self, future) -> None:
        """_return_home navigation result callback: log success or failure."""
        try:
            status = future.result().status
        except Exception as exc:
            self.get_logger().warn(f'[Home] failed to get return result: {exc}')
            self._schedule_home_retry()
            return
        home_x, home_y = self._return_home_target or (self._home_x, self._home_y)
        if status == GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().info(
                f'[Home✓] arrived at start ({home_x:.2f}, {home_y:.2f})'
            )
        else:
            self.get_logger().warn(f'[Home✗] failed to return to start (status={status})')
            self._schedule_home_retry()

    def _schedule_home_retry(self) -> None:
        if self._return_home_attempt > self._return_home_max_retries:
            self.get_logger().warn('[Home] return-to-start retries exhausted')
            return
        self.get_logger().info(
            f'[Home] retrying return to start in {self._return_home_retry_delay:.1f}s'
        )
        self._call_later(self._return_home_retry_delay, self._send_home_goal)

    def _call_later(self, delay_sec: float, callback) -> None:
        """Create a one-shot timer."""
        timer_holder = {}

        def _fire() -> None:
            timer_holder['timer'].cancel()
            callback()

        timer_holder['timer'] = self.create_timer(max(0.1, delay_sec), _fire)

    def _pub_status(
        self,
        coverage: float,
        frontiers: list,
        done: bool = False,
        stuck: bool = False,
    ) -> None:
        """
        Publish /part3/mapping/map_status:
          in progress: coverage=68% frontiers=3 area=15x15
          done:        coverage=done coverage_pct=97%
          stuck:       coverage=stuck coverage_pct=73%
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
#  Entry point
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
