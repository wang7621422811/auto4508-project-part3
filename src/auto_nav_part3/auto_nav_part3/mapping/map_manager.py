#!/usr/bin/env python3
"""
map_manager.py — 地图保存管理器 (M4.C4.2)

================================================================================
功能概述 (Overview)
================================================================================

探索完成时把 SLAM 生成的 OccupancyGrid 地图保存成文件，供 UI 展示、
实验报告截图以及 Task 8（已知地图最短路径规划）复用。

触发方式（两种，互不干扰）：
  1. 自动保存：订阅 /part3/mapping/map_status，检测到 "coverage=done"
               时自动触发（exploration_node 探索完成后发布此消息）。
  2. 手动服务：/part3/mapping/save_map (std_srvs/Trigger)。
               UI 或命令行可随时调用：
               ros2 service call /part3/mapping/save_map std_srvs/srv/Trigger {}

输出文件（均在 save_dir 目录下）：
  <map_filename>.pgm   — 二进制灰度栅格地图（nav2 map_server 可直接加载）
  <map_filename>.yaml  — 地图元数据（分辨率 / 原点 / 阈值）
  <map_filename>.png   — 可视化 PNG（报告截图用）

保存流程（直接写文件，无子进程）：
  ① map_manager 本身订阅 /map（TRANSIENT_LOCAL，与 slam_toolbox QoS 匹配）。
     收到后缓存最新的 OccupancyGrid 到 _latest_map。
  ② 触发保存时：从 _latest_map 直接写 .pgm + .yaml（Python stdlib）。
  ③ 读取 .pgm → 转换成 .png（Pillow 优先；fallback 用 struct + zlib）。
  ④ 向 /part3/mapping/map_status 发布保存结果。

  ★ 不再使用 map_saver_cli 子进程 ★
    原因：子进程需要重新做 DDS 发现（ARM64/Parallels 需 5-15s），
    --timeout_ms 10000 不足以完成发现 → returncode=255（内部超时）。
    直接订阅则复用已有的 ROS2 连接，无发现延迟。

接口 (Interfaces)：
  订阅  /map                        nav_msgs/OccupancyGrid  SLAM 地图（TRANSIENT_LOCAL）
  订阅  /part3/mapping/map_status   std_msgs/String         探索进度（来自 exploration_node）
  服务  /part3/mapping/save_map     std_srvs/Trigger        手动触发保存
  发布  /part3/mapping/map_status   std_msgs/String         保存结果反馈

参数 (Parameters)：
  save_dir      str   输出目录（默认 ~/auto4508_artifacts/maps）
  map_filename  str   文件基名，不含扩展名（默认 discovery_map）
  auto_save     bool  检测到 coverage=done 时自动保存（默认 true）

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


# /map 话题的 QoS：与 slam_toolbox 发布端保持一致，否则 ROS2 静默不连接。
# slam_toolbox 用 TRANSIENT_LOCAL（锁存）发布 /map，订阅端必须同样用 TRANSIENT_LOCAL。
_MAP_QOS = QoSProfile(
    depth=1,
    reliability=QoSReliabilityPolicy.RELIABLE,
    durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
    history=QoSHistoryPolicy.KEEP_LAST,
)


class MapManager(Node):
    """地图保存管理节点 (M4.C4.2)。

    设计原则：
      - 直接订阅 /map，缓存最新 OccupancyGrid，避免子进程 DDS 发现延迟。
      - 与 exploration_node 松耦合 —— 通过共享 topic /part3/mapping/map_status
        感知探索进度，而非直接调用或依赖 exploration_node 的内部状态。
      - 线程安全 —— _save_lock 防止同时触发两次保存（自动 + 手动并发）。
      - 可重入 —— 同一次运行可多次调用 /part3/mapping/save_map（覆盖旧文件）。
    """

    def __init__(self) -> None:
        super().__init__('map_manager')

        # ── 参数声明 ──────────────────────────────────────────────────────────
        # save_dir：地图文件保存目录。
        #   默认值 ~/auto4508_artifacts/maps，方便开发时找文件。
        #   launch 文件可用 parameters=[{'save_dir': '/abs/path/to/artifacts/maps'}]
        #   覆盖为项目 artifacts/maps/ 的绝对路径。
        self.declare_parameter('save_dir',
                               os.path.expanduser('~/auto4508_artifacts/maps'))

        # map_filename：输出文件基名（不含扩展名）。
        #   三种格式（pgm / yaml / png）共用同一基名，与 TOPICS.md 契约一致。
        self.declare_parameter('map_filename', 'discovery_map')

        # auto_save：是否在检测到 "coverage=done" 时自动保存。
        #   true（默认）：无需人工干预，探索完成即存图。
        #   false：只响应手动服务调用（用于调试 / 分步测试）。
        self.declare_parameter('auto_save', True)

        # 读取参数（显式类型转换避免 Pylance Unknown 警告）
        self._save_dir     = str(self.get_parameter('save_dir').value)
        self._map_filename = str(self.get_parameter('map_filename').value)
        self._auto_save    = bool(self.get_parameter('auto_save').value)

        # ── 确保输出目录存在 ──────────────────────────────────────────────────
        # exist_ok=True：多次启动不报错，目录已存在时静默跳过。
        os.makedirs(self._save_dir, exist_ok=True)

        # ── 内部状态 ──────────────────────────────────────────────────────────
        # _latest_map：缓存最新的 OccupancyGrid（来自 /map 订阅）。
        #   None = 尚未收到任何地图消息（slam_toolbox 还没发布 /map）。
        self._latest_map: OccupancyGrid | None = None
        self._map_lock = threading.Lock()  # 保护 _latest_map 的读写

        # _saving：标记当前是否有保存操作在运行，配合 _save_lock 防止并发。
        self._saving = False
        self._save_lock = threading.Lock()

        # _exploration_done：记录是否已为本次探索触发过自动保存。
        # 防止 exploration_node 多次发布 coverage=done 导致重复写盘。
        self._exploration_done = False

        # ── 订阅者：/map（直接缓存 OccupancyGrid）────────────────────────────
        # 用 TRANSIENT_LOCAL QoS 与 slam_toolbox 发布端匹配。
        # TRANSIENT_LOCAL（锁存）：即使本节点晚于 slam_toolbox 启动，
        # 也能收到 slam_toolbox 之前发布的最新地图（订阅即触发回调）。
        self._map_sub = self.create_subscription(
            OccupancyGrid,
            '/map',
            self._on_map,
            _MAP_QOS,
        )

        # ── 发布者：保存结果回写到 map_status ─────────────────────────────────
        # 与 exploration_node 共享同一 topic（均用 depth=10 默认 QoS）。
        # map_manager 保存完成后在此 topic 发布 "map_saved: ..." 消息，
        # UI / state_manager 只需订阅一个 topic 就能获得完整探索 → 存图流程状态。
        self._status_pub = self.create_publisher(
            String, '/part3/mapping/map_status', 10
        )

        # ── 订阅者：监听探索进度 ──────────────────────────────────────────────
        # 用与 exploration_node._status_pub 完全相同的 QoS（depth=10，RELIABLE，
        # VOLATILE），避免 QoS 不兼容导致 ROS2 静默断开连接。
        # VOLATILE（非 TRANSIENT_LOCAL）：如果本节点晚于 exploration_node 启动，
        # 且探索在本节点启动前就已完成，auto_save 不会被触发。
        # 解决方案：确保 map_manager 在 exploration_node 之前或同时启动（见 launch）。
        self._status_sub = self.create_subscription(
            String,
            '/part3/mapping/map_status',
            self._on_map_status,
            10,  # depth=10，RELIABLE VOLATILE（默认）
        )

        # ── 服务：手动触发保存 ────────────────────────────────────────────────
        # /part3/mapping/save_map (std_srvs/Trigger)
        # 语义：同步执行（服务返回时保存已完成或已失败）。
        # 与 mapping_service 的 /part3/mapping/start（异步，"已接受"即返回）不同。
        self._save_service = self.create_service(
            Trigger,
            '/part3/mapping/save_map',
            self._on_save_map_service,
        )

        # ── slam_toolbox 位姿图序列化客户端（Phase 2 localization 复用）────────
        # 保存 pgm 后同步序列化位姿图（.posegraph + .data），
        # 供 use_localization:=true 启动时由 slam_toolbox localization 模式加载。
        # slam_toolbox 包不可用时静默跳过（不影响正常建图功能）。
        self._slam_serialize_cli = None
        if _HAS_SLAM_SERIALIZE:
            self._slam_serialize_cli = self.create_client(
                _SerializePoseGraph,
                '/slam_toolbox/serialize_map',
            )

        self.get_logger().info(
            f'[MapManager] 就绪 | '
            f'save_dir={self._save_dir} | '
            f'map_filename={self._map_filename} | '
            f'auto_save={self._auto_save}'
        )

    # =========================================================================
    # 回调：/map 地图缓存
    # =========================================================================

    def _on_map(self, msg: OccupancyGrid) -> None:
        """缓存最新的 OccupancyGrid 消息。

        slam_toolbox 每次更新地图后发布此消息（频率较低，约 1-5 Hz）。
        用锁保证 _save_impl 读取时不会读到半写状态。
        """
        with self._map_lock:
            self._latest_map = msg
        self.get_logger().debug(
            f'[MapManager] /map 已更新: '
            f'{msg.info.width}x{msg.info.height} '
            f'res={msg.info.resolution:.3f}m/cell'
        )

    # =========================================================================
    # 回调：探索状态监听
    # =========================================================================

    def _on_map_status(self, msg: String) -> None:
        """订阅 /part3/mapping/map_status 的回调。

        当 auto_save=true 且消息包含 "coverage=done" 时触发保存。
        保存在独立守护线程中执行，避免阻塞 ROS2 spin 回调线程。

        "coverage=done" 由 exploration_node 在覆盖率达标或无 frontier 时发布，
        格式如：
          "coverage=done coverage_pct=91.3%"
        """
        if not self._auto_save:
            return  # 手动模式：不响应自动触发

        if 'coverage=done' not in msg.data:
            return  # 正常进度消息（coverage=68% 等），忽略

        if self._exploration_done:
            return  # 同一次探索已触发过保存，忽略重复消息

        # 标记已触发，防止同一次探索的后续重复消息再次启动保存
        self._exploration_done = True
        self.get_logger().info(
            f'[MapManager] 检测到探索完成（{msg.data.strip()}），自动触发地图保存...'
        )

        # 在守护线程中执行保存，不阻塞 ROS2 spin
        t = threading.Thread(
            target=self._do_save,
            name='map_saver_thread',
            daemon=True,  # 主进程退出时线程跟着退出，避免孤儿进程
        )
        t.start()

    # =========================================================================
    # 回调：手动服务
    # =========================================================================

    def _on_save_map_service(
        self,
        request: Trigger.Request,
        response: Trigger.Response,
    ) -> Trigger.Response:
        """处理 /part3/mapping/save_map 服务请求（同步执行）。

        参数 request 无字段（std_srvs/Trigger 请求体为空）。

        返回：
          response.success = True   保存成功（pgm + yaml 写出，png 尽力而为）
          response.success = False  保存失败或并发保护拦截
          response.message          成功时为文件路径描述，失败时为错误描述
        """
        del request  # std_srvs/Trigger 请求无字段，显式删除避免 lint 警告
        success, detail = self._do_save()
        response.success = success
        response.message = detail
        return response

    # =========================================================================
    # 核心：保存流程（线程安全入口）
    # =========================================================================

    def _do_save(self) -> tuple[bool, str]:
        """线程安全的保存入口。

        用 _save_lock 确保同一时刻只有一个保存进程在运行：
          - 自动触发（_on_map_status 线程）和
          - 手动触发（_on_save_map_service 服务线程）
          可能同时到达，_saving 标志保证只有第一个执行。

        返回 (success, detail_message)。
        """
        # ── 并发保护：尝试获得执行权 ──────────────────────────────────────────
        with self._save_lock:
            if self._saving:
                # 另一个保存请求正在进行，直接返回（不排队等待）
                msg = '保存进行中，跳过重复请求'
                self.get_logger().warn(f'[MapManager] {msg}')
                return False, msg
            self._saving = True  # 占用执行权

        try:
            return self._save_impl()
        finally:
            # 无论成功/失败，释放执行权，允许下次保存
            with self._save_lock:
                self._saving = False

    def _save_impl(self) -> tuple[bool, str]:
        """实际执行保存（仅从 _do_save 调用，已持有 _saving 标志）。

        步骤：
          1. 检查是否已缓存 /map 消息。
          2. 从缓存的 OccupancyGrid 直接写 pgm + yaml（无子进程，无 DDS 发现延迟）。
          3. 把 pgm 转换成 png。
          4. 发布结果到 /part3/mapping/map_status。
        """
        # ── 步骤 1：检查缓存的地图 ────────────────────────────────────────────
        with self._map_lock:
            grid = self._latest_map

        if grid is None:
            msg = '尚未收到 /map 消息（slam_toolbox 是否已启动并 activate？）'
            self.get_logger().error(f'[MapManager] {msg}')
            self._publish_status(f'map_save_failed: {msg}')
            return False, msg

        # ── 步骤 2：构建文件路径 ───────────────────────────────────────────────
        base_path = os.path.join(self._save_dir, self._map_filename)
        pgm_path  = base_path + '.pgm'
        yaml_path = base_path + '.yaml'
        png_path  = base_path + '.png'

        self.get_logger().info(
            f'[MapManager] 开始保存地图 → {base_path}.* '
            f'({grid.info.width}x{grid.info.height} cells, '
            f'res={grid.info.resolution:.3f}m/cell)'
        )

        # ── 步骤 3：写 pgm + yaml ─────────────────────────────────────────────
        try:
            self._write_pgm(grid, pgm_path)
            self._write_yaml(grid, pgm_path, yaml_path)
        except Exception as exc:
            msg = f'地图文件写入失败: {exc}'
            self.get_logger().error(f'[MapManager] {msg}')
            self._publish_status(f'map_save_failed: {msg}')
            return False, msg

        self.get_logger().info(f'[MapManager] PGM/YAML 已写出: {pgm_path}')

        # ── 步骤 3b：序列化 slam_toolbox 位姿图（供 Phase 2 加载）─────────────
        # 在后台线程调用，不阻塞当前 save 流程或服务回调。
        # 输出：<base_path>.posegraph + <base_path>.data
        threading.Thread(
            target=self._serialize_pose_graph,
            args=(base_path,),
            name='slam_serialize',
            daemon=True,
        ).start()

        # ── 步骤 4：pgm → png ─────────────────────────────────────────────────
        # PNG 转换失败不影响主要功能（pgm + yaml 已保存），只记录警告
        png_ok = self._convert_pgm_to_png(pgm_path, png_path)
        if png_ok:
            self.get_logger().info(f'[MapManager] PNG 已生成: {png_path}')
        else:
            self.get_logger().warn(
                '[MapManager] PNG 生成失败（pgm/yaml 可正常使用，'
                '安装 python3-pil 可解决 PNG 问题）'
            )

        # ── 步骤 5：发布保存结果 ───────────────────────────────────────────────
        saved_files = (
            f'{pgm_path}, {yaml_path}'
            + (f', {png_path}' if png_ok else '')
        )
        self._publish_status(
            f'map_saved: base={base_path} files=[{saved_files}]'
        )

        result_msg = (
            f'地图已保存: {base_path}'
            + ('.{pgm,yaml,png}' if png_ok else '.{pgm,yaml}')
        )
        self.get_logger().info(f'[MapManager] {result_msg}')
        return True, result_msg

    # =========================================================================
    # OccupancyGrid → PGM 文件
    # =========================================================================

    def _write_pgm(self, grid: OccupancyGrid, pgm_path: str) -> None:
        """从 OccupancyGrid 直接写出 P5 二进制 PGM 文件。

        OccupancyGrid 值域与 PGM 灰度值的对应关系（与 map_saver_cli 一致）：
          grid.data[i] == -1   → 205  (unknown / 灰色)
          grid.data[i] == 0    → 254  (free / 白色)
          grid.data[i] > 0     → 0    (occupied / 黑色，100=满占用)

        PGM 坐标原点在左上角，OccupancyGrid 的行列 (row=0) 对应地图原点（左下）。
        需要垂直翻转（row 从高到低写入），确保 nav2 map_server 加载后方向正确。
        """
        width  = grid.info.width
        height = grid.info.height
        data   = grid.data  # flat list, row-major, origin at bottom-left

        # 预分配像素数组（height×width 字节）
        pixels = bytearray(width * height)
        for row in range(height):
            # PGM 行0 = 图像顶部，OccupancyGrid row0 = 地图底部，需翻转
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
            # PGM P5 头部
            fh.write(f'P5\n{width} {height}\n255\n'.encode('ascii'))
            fh.write(bytes(pixels))

    def _write_yaml(
        self,
        grid: OccupancyGrid,
        pgm_path: str,
        yaml_path: str,
    ) -> None:
        """写出与 pgm 配套的 nav2 map_server 元数据 YAML 文件。

        格式遵循 nav2 map_server 规范：
          image       pgm 文件路径（相对路径或绝对路径）
          mode        trinary（三值：free / occupied / unknown）
          resolution  米/cell
          origin      [x, y, yaw]（地图原点在世界坐标系中的位置，yaw=0）
          negate      0（不反转灰度）
          occupied_thresh  灰度 < 该值（归一化到 0-1）→ occupied
          free_thresh      灰度 > 该值（归一化到 0-1）→ free
        """
        ox = grid.info.origin.position.x
        oy = grid.info.origin.position.y
        res = grid.info.resolution

        # pgm_path 使用绝对路径，map_server 可直接加载，无需相对路径推断
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
    # PGM → PNG 格式转换
    # =========================================================================

    def _convert_pgm_to_png(self, pgm_path: str, png_path: str) -> bool:
        """把 P5 二进制 PGM 文件转换成灰度 PNG。

        转换策略（按优先级）：
          1. Pillow (PIL)：能处理各种 PGM 变体，推荐安装 `sudo apt install python3-pil`
          2. stdlib fallback：纯 Python struct + zlib，仅支持标准 P5 8-bit
        """
        # ── 优先 Pillow ───────────────────────────────────────────────────────
        try:
            from PIL import Image  # type: ignore[import]
            img = Image.open(pgm_path)
            img.save(png_path)
            return True
        except ImportError:
            pass  # Pillow 未安装，继续用 stdlib fallback
        except Exception as exc:
            self.get_logger().warn(f'[MapManager] Pillow 转换异常: {exc}')
            return False

        # ── stdlib fallback ───────────────────────────────────────────────────
        try:
            return self._pgm_to_png_stdlib(pgm_path, png_path)
        except Exception as exc:
            self.get_logger().warn(f'[MapManager] stdlib PNG 转换失败: {exc}')
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
