#!/usr/bin/env python3
"""
map_manager.py — map save manager (M4.C4.2)

================================================================================
Overview
================================================================================

Saves the SLAM-generated OccupancyGrid map to files on exploration completion,
for use in the UI, experiment report screenshots, and Task 8 (known-map shortest
path planning).

Two trigger modes (independent):
  1. Auto-save: subscribes to /part3/mapping/map_status; fires automatically
                when "coverage=done" is detected (published by exploration_node).
  2. Manual service: /part3/mapping/save_map (std_srvs/Trigger).
                     Callable from UI or command line at any time:
                     ros2 service call /part3/mapping/save_map std_srvs/srv/Trigger {}

Output files (all under save_dir):
  <map_filename>.pgm   — binary greyscale occupancy grid (directly loadable by nav2 map_server)
  <map_filename>.yaml  — map metadata (resolution / origin / thresholds)
  <map_filename>.png   — visual PNG (for report screenshots)

Save procedure (direct file write, no subprocess):
  ① map_manager subscribes to /map (TRANSIENT_LOCAL, matching slam_toolbox QoS).
     Caches the latest OccupancyGrid in _latest_map on each receipt.
  ② On save trigger: writes .pgm + .yaml directly from _latest_map (Python stdlib).
  ③ Converts .pgm → .png (Pillow preferred; stdlib struct+zlib fallback).
  ④ Publishes save result to /part3/mapping/map_status.

  ★ map_saver_cli subprocess is NOT used ★
    Reason: a subprocess must redo DDS discovery (5–15 s on ARM64/Parallels),
    and --timeout_ms 10000 is insufficient → returncode=255 (internal timeout).
    Direct subscription reuses the existing ROS2 connection with no discovery delay.

Interfaces:
  subscribe  /map                        nav_msgs/OccupancyGrid  SLAM map (TRANSIENT_LOCAL)
  subscribe  /part3/mapping/map_status   std_msgs/String         exploration progress (from exploration_node)
  service    /part3/mapping/save_map     std_srvs/Trigger        manual save trigger
  publish    /part3/mapping/map_status   std_msgs/String         save result feedback

Parameters:
  save_dir      str   output directory (default ~/auto4508_artifacts/maps)
  map_filename  str   file basename without extension (default discovery_map)
  auto_save     bool  automatically save when coverage=done is detected (default true)

================================================================================
"""

import os
import struct
import threading
import zlib

import rclpy
from rclpy.node import Node
from rclpy.qos import (
    QoSDurabilityPolicy,
    QoSHistoryPolicy,
    QoSProfile,
    QoSReliabilityPolicy,
)
from nav_msgs.msg import OccupancyGrid
from std_msgs.msg import String
from std_srvs.srv import Trigger

try:
    from slam_toolbox.srv import SerializePoseGraph as _SerializePoseGraph
    _HAS_SLAM_SERIALIZE = True
except ImportError:
    _HAS_SLAM_SERIALIZE = False


# /map topic QoS: must match the slam_toolbox publisher, otherwise ROS2 silently drops the connection.
# slam_toolbox publishes /map with TRANSIENT_LOCAL (latching); subscribers must use TRANSIENT_LOCAL too.
_MAP_QOS = QoSProfile(
    depth=1,
    reliability=QoSReliabilityPolicy.RELIABLE,
    durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
    history=QoSHistoryPolicy.KEEP_LAST,
)


class MapManager(Node):
    """Map save manager node (M4.C4.2).

    Design principles:
      - Subscribes directly to /map and caches the latest OccupancyGrid, avoiding subprocess DDS discovery delays.
      - Loosely coupled to exploration_node — tracks exploration progress via the shared /part3/mapping/map_status
        topic rather than calling or depending on exploration_node's internal state.
      - Thread-safe — _save_lock prevents concurrent saves (auto-save + manual trigger arriving simultaneously).
      - Reentrant — /part3/mapping/save_map can be called multiple times in one run (overwrites previous files).
    """

    def __init__(self) -> None:
        super().__init__('map_manager')

        # ── parameter declarations ─────────────────────────────────────────────
        # save_dir: output directory for map files.
        #   Default ~/auto4508_artifacts/maps for easy access during development.
        #   Launch file can override with parameters=[{'save_dir': '/abs/path/to/artifacts/maps'}]
        #   to use the project artifacts/maps/ absolute path.
        self.declare_parameter('save_dir',
                               os.path.expanduser('~/auto4508_artifacts/maps'))

        # map_filename: output file basename (no extension).
        #   All three formats (pgm / yaml / png) share the same basename.
        self.declare_parameter('map_filename', 'discovery_map')

        # auto_save: whether to save automatically when "coverage=done" is detected.
        #   true (default): saves without human intervention when exploration finishes.
        #   false: only responds to manual service calls (useful for debugging / step-by-step testing).
        self.declare_parameter('auto_save', True)

        # explicit type casts suppress Pylance Unknown warnings
        self._save_dir     = str(self.get_parameter('save_dir').value)
        self._map_filename = str(self.get_parameter('map_filename').value)
        self._auto_save    = bool(self.get_parameter('auto_save').value)

        # ensure output directory exists; exist_ok=True silently skips if already present
        os.makedirs(self._save_dir, exist_ok=True)

        # ── internal state ────────────────────────────────────────────────────
        # _latest_map: cache of the most recent OccupancyGrid from /map.
        #   None = no map message received yet (slam_toolbox has not published /map).
        self._latest_map: OccupancyGrid | None = None
        self._map_lock = threading.Lock()  # protects _latest_map reads and writes

        # _saving: flag indicating a save operation is in progress; used with _save_lock to prevent concurrency.
        self._saving = False
        self._save_lock = threading.Lock()

        # _exploration_done: records whether auto-save has already been triggered for this exploration run.
        # Prevents duplicate disk writes when exploration_node publishes coverage=done multiple times.
        self._exploration_done = False

        # ── /map subscriber (caches OccupancyGrid directly) ──────────────────
        # Uses TRANSIENT_LOCAL QoS to match the slam_toolbox publisher.
        # TRANSIENT_LOCAL (latching): even if this node starts after slam_toolbox,
        # it receives the latest map that slam_toolbox already published (callback fires on subscribe).
        self._map_sub = self.create_subscription(
            OccupancyGrid,
            '/map',
            self._on_map,
            _MAP_QOS,
        )

        # ── publisher: write save result back to map_status ───────────────────
        # Shares the same topic with exploration_node (both use the default depth=10 QoS).
        # map_manager publishes "map_saved: ..." here after saving; UI / state_manager
        # only needs to subscribe to one topic to get the full explore→save pipeline status.
        self._status_pub = self.create_publisher(
            String, '/part3/mapping/map_status', 10
        )

        # ── subscriber: monitor exploration progress ───────────────────────────
        # Uses exactly the same QoS as exploration_node._status_pub (depth=10, RELIABLE, VOLATILE)
        # to avoid ROS2 silently dropping the connection due to QoS incompatibility.
        # VOLATILE (not TRANSIENT_LOCAL): if this node starts after exploration_node and
        # exploration has already finished before this node launched, auto_save will not trigger.
        # Solution: ensure map_manager launches before or simultaneously with exploration_node (see launch file).
        self._status_sub = self.create_subscription(
            String,
            '/part3/mapping/map_status',
            self._on_map_status,
            10,  # depth=10, RELIABLE VOLATILE (default)
        )

        # ── service: manual save trigger ──────────────────────────────────────
        # /part3/mapping/save_map (std_srvs/Trigger)
        # Semantics: synchronous execution (save is complete or failed when the service returns).
        # Different from mapping_service's /part3/mapping/start (async, returns immediately on "accepted").
        self._save_service = self.create_service(
            Trigger,
            '/part3/mapping/save_map',
            self._on_save_map_service,
        )

        # ── slam_toolbox pose graph serialisation client (Phase 2 localisation reuse) ──
        # Serialises the pose graph (.posegraph + .data) after saving the pgm,
        # for slam_toolbox localisation mode to load when launched with use_localization:=true.
        # Silently skipped if the slam_toolbox package is unavailable (does not affect normal mapping).
        self._slam_serialize_cli = None
        if _HAS_SLAM_SERIALIZE:
            self._slam_serialize_cli = self.create_client(
                _SerializePoseGraph,
                '/slam_toolbox/serialize_map',
            )

        self.get_logger().info(
            f'[MapManager] ready | '
            f'save_dir={self._save_dir} | '
            f'map_filename={self._map_filename} | '
            f'auto_save={self._auto_save}'
        )

    # =========================================================================
    # Callback: /map cache
    # =========================================================================

    def _on_map(self, msg: OccupancyGrid) -> None:
        """Cache the latest OccupancyGrid message.

        slam_toolbox publishes this after each map update (low frequency, ~1–5 Hz).
        Lock ensures _save_impl never reads a partially written state.
        """
        with self._map_lock:
            self._latest_map = msg
        self.get_logger().debug(
            f'[MapManager] /map updated: '
            f'{msg.info.width}x{msg.info.height} '
            f'res={msg.info.resolution:.3f}m/cell'
        )

    # =========================================================================
    # Callback: exploration status listener
    # =========================================================================

    def _on_map_status(self, msg: String) -> None:
        """Callback subscribed to /part3/mapping/map_status.

        Triggers a save when auto_save=true and the message contains "coverage=done".
        Save runs in a dedicated daemon thread to avoid blocking the ROS2 spin callback thread.

        "coverage=done" is published by exploration_node when coverage target is reached or
        no frontiers remain, e.g.:
          "coverage=done coverage_pct=91.3%"
        """
        if not self._auto_save:
            return  # manual mode: do not respond to auto-trigger

        if 'coverage=done' not in msg.data:
            return  # normal progress message (coverage=68% etc.), ignore

        if self._exploration_done:
            return  # auto-save already triggered for this exploration run, ignore duplicates

        # mark triggered to prevent subsequent duplicate messages from starting another save
        self._exploration_done = True
        self.get_logger().info(
            f'[MapManager] exploration complete detected ({msg.data.strip()}), triggering auto map save...'
        )

        # run save in a daemon thread so ROS2 spin is not blocked
        t = threading.Thread(
            target=self._do_save,
            name='map_saver_thread',
            daemon=True,  # exits with main process to avoid orphan threads
        )
        t.start()

    # =========================================================================
    # Callback: manual service
    # =========================================================================

    def _on_save_map_service(
        self,
        request: Trigger.Request,
        response: Trigger.Response,
    ) -> Trigger.Response:
        """Handle /part3/mapping/save_map service request (synchronous).

        request has no fields (std_srvs/Trigger request body is empty).

        Returns:
          response.success = True   save succeeded (pgm + yaml written, png best-effort)
          response.success = False  save failed or blocked by concurrency guard
          response.message          file path description on success, error description on failure
        """
        del request  # std_srvs/Trigger 请求无字段，显式删除避免 lint 警告
        success, detail = self._do_save()
        response.success = success
        response.message = detail
        return response

    # =========================================================================
    # Core: save procedure (thread-safe entry point)
    # =========================================================================

    def _do_save(self) -> tuple[bool, str]:
        """Thread-safe save entry point.

        Uses _save_lock to ensure only one save runs at a time:
          - auto-trigger (_on_map_status thread) and
          - manual trigger (_on_save_map_service service thread)
          may arrive concurrently; the _saving flag ensures only the first one executes.

        Returns (success, detail_message).
        """
        # ── concurrency guard: try to acquire execution right ─────────────────
        with self._save_lock:
            if self._saving:
                # another save request is in progress; return immediately (do not queue)
                msg = 'save already in progress, skipping duplicate request'
                self.get_logger().warn(f'[MapManager] {msg}')
                return False, msg
            self._saving = True  # claim execution right

        try:
            return self._save_impl()
        finally:
            # release execution right on success or failure to allow the next save
            with self._save_lock:
                self._saving = False

    def _save_impl(self) -> tuple[bool, str]:
        """Execute the actual save (called only from _do_save, which holds _saving).

        Steps:
          1. Check whether a /map message has been cached.
          2. Write pgm + yaml directly from the cached OccupancyGrid (no subprocess, no DDS discovery delay).
          3. Convert pgm to png.
          4. Publish result to /part3/mapping/map_status.
        """
        # ── step 1: check cached map ──────────────────────────────────────────
        with self._map_lock:
            grid = self._latest_map

        if grid is None:
            msg = 'no /map message received yet (is slam_toolbox started and activated?)'
            self.get_logger().error(f'[MapManager] {msg}')
            self._publish_status(f'map_save_failed: {msg}')
            return False, msg

        # ── step 2: build file paths ──────────────────────────────────────────
        base_path = os.path.join(self._save_dir, self._map_filename)
        pgm_path  = base_path + '.pgm'
        yaml_path = base_path + '.yaml'
        png_path  = base_path + '.png'

        self.get_logger().info(
            f'[MapManager] saving map → {base_path}.* '
            f'({grid.info.width}x{grid.info.height} cells, '
            f'res={grid.info.resolution:.3f}m/cell)'
        )

        # ── step 3: write pgm + yaml ──────────────────────────────────────────
        try:
            self._write_pgm(grid, pgm_path)
            self._write_yaml(grid, pgm_path, yaml_path)
        except Exception as exc:
            msg = f'map file write failed: {exc}'
            self.get_logger().error(f'[MapManager] {msg}')
            self._publish_status(f'map_save_failed: {msg}')
            return False, msg

        self.get_logger().info(f'[MapManager] PGM/YAML written: {pgm_path}')

        # ── step 3b: serialise slam_toolbox pose graph (for Phase 2 load) ─────
        # Called in a background thread so it does not block the current save flow or service callback.
        # Output: <base_path>.posegraph + <base_path>.data
        threading.Thread(
            target=self._serialize_pose_graph,
            args=(base_path,),
            name='slam_serialize',
            daemon=True,
        ).start()

        # ── step 4: pgm → png ─────────────────────────────────────────────────
        # PNG conversion failure does not affect core functionality (pgm + yaml already saved); only logs a warning
        png_ok = self._convert_pgm_to_png(pgm_path, png_path)
        if png_ok:
            self.get_logger().info(f'[MapManager] PNG generated: {png_path}')
        else:
            self.get_logger().warn(
                '[MapManager] PNG generation failed (pgm/yaml are usable; '
                'install python3-pil to resolve PNG issues)'
            )

        # ── step 5: publish save result ───────────────────────────────────────
        saved_files = (
            f'{pgm_path}, {yaml_path}'
            + (f', {png_path}' if png_ok else '')
        )
        self._publish_status(
            f'map_saved: base={base_path} files=[{saved_files}]'
        )

        result_msg = (
            f'map saved: {base_path}'
            + ('.{pgm,yaml,png}' if png_ok else '.{pgm,yaml}')
        )
        self.get_logger().info(f'[MapManager] {result_msg}')
        return True, result_msg

    # =========================================================================
    # OccupancyGrid → PGM file
    # =========================================================================

    def _write_pgm(self, grid: OccupancyGrid, pgm_path: str) -> None:
        """Write a P5 binary PGM file directly from an OccupancyGrid.

        OccupancyGrid value to PGM grey-level mapping (matches map_saver_cli):
          grid.data[i] == -1   → 205  (unknown / grey)
          grid.data[i] == 0    → 254  (free / white)
          grid.data[i] > 0     → 0    (occupied / black; 100 = fully occupied)

        PGM origin is top-left; OccupancyGrid row=0 corresponds to the map origin (bottom-left).
        Vertical flip (rows written high-to-low) ensures correct orientation when nav2 map_server loads the file.
        """
        width  = grid.info.width
        height = grid.info.height
        data   = grid.data  # flat list, row-major, origin at bottom-left

        # pre-allocate pixel array (height × width bytes)
        pixels = bytearray(width * height)
        for row in range(height):
            # PGM row 0 = top of image; OccupancyGrid row 0 = map bottom — flip vertically
            pgm_row = height - 1 - row
            for col in range(width):
                val = data[row * width + col]
                if val == -1:
                    pixels[pgm_row * width + col] = 205  # unknown
                elif val == 0:
                    pixels[pgm_row * width + col] = 254  # free
                else:
                    pixels[pgm_row * width + col] = 0    # occupied

        with open(pgm_path, 'wb') as fh:
            # PGM P5 header
            fh.write(f'P5\n{width} {height}\n255\n'.encode('ascii'))
            fh.write(bytes(pixels))

    def _write_yaml(
        self,
        grid: OccupancyGrid,
        pgm_path: str,
        yaml_path: str,
    ) -> None:
        """Write the nav2 map_server metadata YAML file to accompany the pgm.

        Format follows the nav2 map_server specification:
          image           pgm file path (relative or absolute)
          mode            trinary (three values: free / occupied / unknown)
          resolution      metres/cell
          origin          [x, y, yaw] (map origin in world frame, yaw=0)
          negate          0 (do not invert greyscale)
          occupied_thresh grey value < this (normalised 0–1) → occupied
          free_thresh     grey value > this (normalised 0–1) → free
        """
        ox = grid.info.origin.position.x
        oy = grid.info.origin.position.y
        res = grid.info.resolution

        # absolute pgm_path so map_server can load it without relative-path inference
        content = (
            f'image: {pgm_path}\n'
            f'mode: trinary\n'
            f'resolution: {res}\n'
            f'origin: [{ox:.6f}, {oy:.6f}, 0.0]\n'
            f'negate: 0\n'
            f'occupied_thresh: 0.65\n'
            f'free_thresh: 0.25\n'
        )

        with open(yaml_path, 'w', encoding='utf-8') as fh:
            fh.write(content)

    # =========================================================================
    # PGM → PNG format conversion
    # =========================================================================

    def _convert_pgm_to_png(self, pgm_path: str, png_path: str) -> bool:
        """Convert a P5 binary PGM file to a greyscale PNG.

        Conversion strategy (in priority order):
          1. Pillow (PIL): handles all PGM variants; install with `sudo apt install python3-pil`
          2. stdlib fallback: pure Python struct + zlib; supports standard P5 8-bit only
        """
        # ── prefer Pillow ─────────────────────────────────────────────────────
        try:
            from PIL import Image  # type: ignore[import]
            img = Image.open(pgm_path)
            img.save(png_path)
            return True
        except ImportError:
            pass  # Pillow not installed, fall through to stdlib fallback
        except Exception as exc:
            self.get_logger().warn(f'[MapManager] Pillow conversion error: {exc}')
            return False

        # ── stdlib fallback ───────────────────────────────────────────────────
        try:
            return self._pgm_to_png_stdlib(pgm_path, png_path)
        except Exception as exc:
            self.get_logger().warn(f'[MapManager] stdlib PNG conversion failed: {exc}')
            return False

    def _pgm_to_png_stdlib(self, pgm_path: str, png_path: str) -> bool:
        """纯 stdlib 实现的 P5 PGM → 灰度 PNG 转换。

        PGM P5（二进制灰度）格式：
        ─────────────────────────
          行1：magic "P5"
          行2：可选注释（"# ..."，可有多行）
          行3："width height"
          行4："maxval"（最大像素值，本函数仅支持 ≤255 即 8-bit）
          其余：raw 字节，共 width×height 个

        PNG 灰度图格式（关键结构）：
        ─────────────────────────────
          8字节签名
          IHDR chunk：宽、高、位深(8)、颜色类型(0=灰度)、压缩(0)、过滤(0)、隔行(0)
          IDAT chunk：每行加 1 字节 filter type(0x00=无过滤)，然后用 zlib 压缩
          IEND chunk：空数据标记结束

        每个 PNG chunk 格式：
          4字节 数据长度(大端)
          4字节 chunk 类型(ASCII)
          N字节 数据
          4字节 CRC32(类型+数据，大端)
        """
        # ── 读取并解析 PGM 头部 ───────────────────────────────────────────────
        with open(pgm_path, 'rb') as fh:
            raw = fh.read()

        # 逐行扫描头部（跳过注释行），提取 magic / size / maxval
        parsed: list[bytes] = []
        idx = 0
        while len(parsed) < 3:
            newline_pos = raw.index(b'\n', idx)
            line = raw[idx:newline_pos].strip()
            idx = newline_pos + 1
            if line.startswith(b'#'):
                continue  # 注释行，跳过
            parsed.append(line)

        magic = parsed[0].decode('ascii', errors='replace')
        if magic != 'P5':
            raise ValueError(f'不支持的 PGM 格式: {magic}（仅支持 P5 二进制）')

        width, height = (int(v) for v in parsed[1].split())
        maxval = int(parsed[2])

        if maxval > 255:
            raise ValueError(
                f'maxval={maxval} > 255，stdlib fallback 不支持 16-bit PGM，'
                '请安装 python3-pil'
            )

        # 像素数据从头部之后开始
        pixels = raw[idx:idx + width * height]
        if len(pixels) < width * height:
            raise ValueError(
                f'PGM 像素数据不足：期望 {width * height} 字节，'
                f'实际 {len(pixels)} 字节'
            )

        # ── 构造 PNG ──────────────────────────────────────────────────────────
        def _make_chunk(tag: bytes, data: bytes) -> bytes:
            """生成一个 PNG chunk（长度 + 类型 + 数据 + CRC）。"""
            crc = zlib.crc32(tag + data) & 0xFFFFFFFF
            return struct.pack('>I', len(data)) + tag + data + struct.pack('>I', crc)

        # IHDR chunk（固定 13 字节数据）
        ihdr = struct.pack(
            '>IIBBBBB',
            width,   # 图像宽度（像素）
            height,  # 图像高度（像素）
            8,       # 位深 8-bit
            0,       # 颜色类型 0 = 灰度（无 alpha）
            0,       # 压缩方法 0 = deflate（PNG 唯一合法值）
            0,       # 过滤方法 0（PNG 唯一合法值）
            0,       # 隔行扫描 0 = 非隔行
        )

        # IDAT chunk：每扫描行前加 filter byte 0x00（无过滤），整体 zlib 压缩
        # filter byte 0x00 = None filter：像素值直接存储，不做差分预测
        scanlines = bytearray()
        for row_idx in range(height):
            scanlines.append(0x00)                         # filter byte
            start = row_idx * width
            scanlines.extend(pixels[start:start + width])  # 该行像素

        idat_data = zlib.compress(bytes(scanlines))

        # 拼装完整 PNG 二进制
        png_bytes = (
            b'\x89PNG\r\n\x1a\n'          # PNG 签名（固定 8 字节）
            + _make_chunk(b'IHDR', ihdr)   # 图像头
            + _make_chunk(b'IDAT', idat_data)  # 图像数据（压缩）
            + _make_chunk(b'IEND', b'')    # 文件结束标记（空数据）
        )

        with open(png_path, 'wb') as fh:
            fh.write(png_bytes)

        return True

    # =========================================================================
    # 工具方法
    # =========================================================================

    # =========================================================================
    # slam_toolbox 位姿图序列化（Phase 2 localization 模式支持）
    # =========================================================================

    def _serialize_pose_graph(self, path_no_ext: str) -> None:
        """调用 /slam_toolbox/serialize_map，把当前位姿图写到磁盘。
        输出文件：<path_no_ext>.posegraph + <path_no_ext>.data
        Phase 2 用 use_localization:=true 启动时由 slam_toolbox localization 模式加载。

        在独立线程中调用（不阻塞 ROS2 spin 或 save 流程）。
        event-based 等待：main 线程继续 spin，响应回调后唤醒本线程。
        """
        if self._slam_serialize_cli is None:
            self.get_logger().warn(
                '[MapManager] slam_toolbox Python 包不可用，跳过位姿图序列化'
            )
            return

        if not self._slam_serialize_cli.wait_for_service(timeout_sec=5.0):
            self.get_logger().warn(
                '[MapManager] /slam_toolbox/serialize_map 服务不可用，跳过序列化。'
                '（如需 Phase 2 localization 重新加载，请确保 slam_toolbox 在线）'
            )
            return

        req = _SerializePoseGraph.Request()
        req.filename = path_no_ext   # slam_toolbox 自动附加 .posegraph / .data

        done_event: threading.Event = threading.Event()
        result_box: list = [None]

        def _done(future):
            result_box[0] = future.result()
            done_event.set()

        future = self._slam_serialize_cli.call_async(req)
        future.add_done_callback(_done)

        if not done_event.wait(timeout=10.0):
            self.get_logger().warn('[MapManager] 位姿图序列化超时（10s）')
            return

        result = result_box[0]
        if result is not None and result.result:
            self.get_logger().info(
                f'[MapManager] 位姿图已保存: {path_no_ext}.posegraph\n'
                '  → Phase 2 可用 use_localization:=true 直接加载，无需重新建图'
            )
        else:
            self.get_logger().warn(
                '[MapManager] 位姿图序列化失败（slam_toolbox 返回 false），'
                'Phase 2 localization 将不可用'
            )

    # =========================================================================
    # 工具方法
    # =========================================================================

    def _publish_status(self, text: str) -> None:
        """向 /part3/mapping/map_status 发布一条状态消息。"""
        msg = String()
        msg.data = text
        self._status_pub.publish(msg)
        self.get_logger().debug(f'[MapManager] 发布状态: {text}')


# =============================================================================
# 入口
# =============================================================================

def main(args: list[str] | None = None) -> None:
    """ROS2 节点入口，由 setup.py console_scripts 注册为 map_manager。"""
    rclpy.init(args=args)
    node = MapManager()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass  # Ctrl-C 优雅退出
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
