"""
perception_adapter.py — 感知结果适配器 (C_P.1)

职责
────
  1. 订阅 /part3/perception/marker_event（String，来自 greek_detector / colour_detector）
  2. 解析 key=value 格式，优先用深度图 + TF 计算障碍物真实地图坐标：
       a. marker_event 含 cx_px/cy_px → 从深度帧采样 → 反投影到 cam_optical_link
          → TF 变换到 map（SLAM 未就绪时退回 odom 帧）
       b. 无像素坐标 → 退回 robot_position + odom→map 平移
  3. 按 dedup_radius_m 去重：同一位置相同 marker 时更新 confidence，不新增条目
  4. 定期发布 /part3/perception/markers (geometry_msgs/PoseArray, map 帧)
  5. 提供 /part3/perception/get_markers (std_srvs/Trigger) 服务

话题 / 服务契约
────────────────
  订阅：
    /part3/perception/marker_event  std_msgs/String
    <depth_topic>                   sensor_msgs/Image  (32FC1 m 或 uint16 mm)
  发布：
    /part3/perception/markers       geometry_msgs/PoseArray
  服务：
    /part3/perception/get_markers   std_srvs/Trigger

参数 (camera_bringup.launch.py)
────────────────────────────────
  dedup_radius_m   float   0.5                     同位置去重半径（m）
  publish_rate_hz  float   2.0                     定时发布 PoseArray 频率
  map_frame        str    'map'                    输出坐标帧
  odom_frame       str    'odom'                   检测器输出坐标所在帧
  depth_topic      str   '/camera/depth_image'     深度话题（仿真 Gazebo）
                                                   真机改为 /oak/stereo/depth
  depth_scale      float  1.0                      深度值倍率：1.0=m（仿真），0.001=mm→m（OAK-D）
  camera_hfov      float  1.089                    相机水平视角（rad），来自 pioneer URDF
  image_width      int    640                      图像宽度（px）
  image_height     int    480                      图像高度（px）
"""

from __future__ import annotations

import math
from typing import Optional

import numpy as np
import rclpy
from cv_bridge import CvBridge
from geometry_msgs.msg import Pose, Point, PoseArray, PointStamped, Quaternion
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String
from std_srvs.srv import Trigger

try:
    from tf2_ros import Buffer, TransformListener
    from tf2_ros import LookupException, ConnectivityException, ExtrapolationException
    _TF2_AVAILABLE = True
except ImportError:
    _TF2_AVAILABLE = False

try:
    import tf2_geometry_msgs  # noqa: F401 — 向 tf2 Buffer 注册 PointStamped 变换器
    _TF2_GEOMETRY_AVAILABLE = True
except ImportError:
    _TF2_GEOMETRY_AVAILABLE = False


class _MarkerEntry:
    """内存中的单条 marker 记录。"""
    __slots__ = ('marker_type', 'label', 'x', 'y', 'confidence', 'count')

    def __init__(self, marker_type: str, label: str, x: float, y: float, confidence: float):
        self.marker_type = marker_type
        self.label       = label
        self.x           = x
        self.y           = y
        self.confidence  = confidence
        self.count       = 1


class PerceptionAdapterNode(Node):
    """订阅 marker_event，去重后以 PoseArray 发布，并提供 get_markers 服务。"""

    def __init__(self) -> None:
        super().__init__('perception_adapter')

        # ── 参数 ──────────────────────────────────────────────────────────
        self.declare_parameter('dedup_radius_m',  0.5)
        self.declare_parameter('publish_rate_hz', 2.0)
        self.declare_parameter('map_frame',       'map')
        self.declare_parameter('odom_frame',      'odom')
        self.declare_parameter('depth_topic',     '/camera/depth_image')
        self.declare_parameter('depth_scale',     1.0)
        self.declare_parameter('camera_hfov',     1.089)
        self.declare_parameter('image_width',     640)
        self.declare_parameter('image_height',    480)

        gp = self.get_parameter
        self._dedup_r     = gp('dedup_radius_m').get_parameter_value().double_value
        self._rate        = gp('publish_rate_hz').get_parameter_value().double_value
        self._map_frame   = gp('map_frame').get_parameter_value().string_value
        self._odom_frame  = gp('odom_frame').get_parameter_value().string_value
        self._depth_topic = gp('depth_topic').get_parameter_value().string_value
        self._depth_scale = gp('depth_scale').get_parameter_value().double_value
        self._cam_hfov    = gp('camera_hfov').get_parameter_value().double_value
        self._img_w       = gp('image_width').get_parameter_value().integer_value
        self._img_h       = gp('image_height').get_parameter_value().integer_value

        # ── TF ────────────────────────────────────────────────────────────
        if _TF2_AVAILABLE:
            self._tf_buffer   = Buffer()
            self._tf_listener = TransformListener(self._tf_buffer, self)
        else:
            self._tf_buffer = None
            self.get_logger().warn('tf2_ros 不可用，将直接使用 odom 坐标')

        # ── 状态 ──────────────────────────────────────────────────────────
        self._markers: list[_MarkerEntry] = []
        self._depth_frame: Optional[np.ndarray] = None  # float32, 单位 m
        self._bridge = CvBridge()

        # ── 订阅 ──────────────────────────────────────────────────────────
        self.create_subscription(
            String,
            '/part3/perception/marker_event',
            self._on_marker_event,
            10,
        )
        self.create_subscription(
            Image,
            self._depth_topic,
            self._on_depth,
            10,
        )

        # ── 发布 ──────────────────────────────────────────────────────────
        self._pub = self.create_publisher(PoseArray, '/part3/perception/markers', 10)

        # ── 服务 ──────────────────────────────────────────────────────────
        self.create_service(
            Trigger,
            '/part3/perception/get_markers',
            self._on_get_markers,
        )

        # ── 定时发布 ──────────────────────────────────────────────────────
        self.create_timer(1.0 / self._rate, self._publish_markers)

        self.get_logger().info(
            f'PerceptionAdapter 就绪  dedup={self._dedup_r}m  '
            f'frame={self._map_frame}  rate={self._rate}Hz  '
            f'depth_topic={self._depth_topic}  depth_scale={self._depth_scale}'
        )

    # ════════════════════════════════════════════════════════════════════
    # 深度回调
    # ════════════════════════════════════════════════════════════════════

    def _on_depth(self, msg: Image) -> None:
        """缓存最新深度帧（统一转换为 float32，单位 m）。"""
        try:
            raw = self._bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
            if raw.dtype == np.float32:
                self._depth_frame = raw.copy()
            else:
                # uint16 mm (OAK-D) → float32 m
                self._depth_frame = raw.astype(np.float32) * self._depth_scale
        except Exception as exc:
            self.get_logger().warn(
                f'深度图解码失败: {exc}', throttle_duration_sec=5.0)

    # ════════════════════════════════════════════════════════════════════
    # 事件处理
    # ════════════════════════════════════════════════════════════════════

    def _on_marker_event(self, msg: String) -> None:
        fields = self._parse_event(msg.data.strip())
        if fields is None:
            self.get_logger().warn(
                f'无法解析 marker_event: "{msg.data}"',
                throttle_duration_sec=5.0,
            )
            return

        ox, oy = fields['x'], fields['y']  # 机器人位置（odom 帧，兜底用）

        # 优先使用深度图 + TF 计算障碍物真实坐标
        mx, my = None, None
        if 'cx_px' in fields and 'cy_px' in fields:
            mx, my = self._depth_to_map(int(fields['cx_px']), int(fields['cy_px']))

        if mx is None:
            # 深度不可用 — 退回机器人 odom 坐标 + odom→map 平移
            mx, my = self._transform_to_map(ox, oy)

        self._upsert(fields['type'], fields['label'], mx, my, fields['confidence'])

        self.get_logger().info(
            f"[ADAPTER] {fields['type']} {fields['label']} "
            f"robot=({ox:.2f},{oy:.2f}) map=({mx:.2f},{my:.2f}) "
            f"conf={fields['confidence']:.2f}  total={len(self._markers)}"
        )

    def _on_get_markers(self, _request, response: Trigger.Response) -> Trigger.Response:
        """服务回调：立即发布一次 PoseArray，返回当前 marker 数量。"""
        self._publish_markers()
        response.success = True
        response.message = f'markers={len(self._markers)}'
        return response

    # ════════════════════════════════════════════════════════════════════
    # 核心逻辑
    # ════════════════════════════════════════════════════════════════════

    def _depth_to_map(self, cx_px: int, cy_px: int) -> tuple[float, float] | None:
        """
        从深度帧采样障碍物深度，反投影到 cam_optical_link 帧，再经 TF 变换到
        map（或 odom）帧，得到障碍物的世界坐标。
        深度无效或 TF 不可用时返回 None，调用方退回机器人位置。
        """
        if self._depth_frame is None or self._tf_buffer is None:
            return None
        if not _TF2_GEOMETRY_AVAILABLE:
            return None

        h, w = self._depth_frame.shape[:2]
        cx_px = max(1, min(cx_px, w - 2))
        cy_px = max(1, min(cy_px, h - 2))

        # 3×3 邻域均值，降低深度噪声影响
        patch   = self._depth_frame[cy_px - 1:cy_px + 2, cx_px - 1:cx_px + 2]
        depth_m = float(np.nanmean(patch))
        if not (0.1 < depth_m < 15.0) or math.isnan(depth_m):
            return None

        # 反投影：像素 → cam_optical_link 帧（Z-forward 光学坐标约定）
        fx    = (w / 2.0) / math.tan(self._cam_hfov / 2.0)
        x_opt = (cx_px - w / 2.0) * depth_m / fx
        y_opt = (cy_px - h / 2.0) * depth_m / fx  # 假设等焦距（方形像素）
        z_opt = depth_m

        pt = PointStamped()
        pt.header.frame_id = 'cam_optical_link'
        pt.header.stamp    = rclpy.time.Time().to_msg()  # 请求最新可用变换
        pt.point.x = x_opt
        pt.point.y = y_opt
        pt.point.z = z_opt

        # 优先变换到 map 帧；SLAM 未就绪时退回 odom 帧
        for target in (self._map_frame, self._odom_frame):
            try:
                pt_out = self._tf_buffer.transform(
                    pt, target,
                    timeout=rclpy.duration.Duration(seconds=0.1),
                )
                return pt_out.point.x, pt_out.point.y
            except Exception:
                continue
        return None

    def _upsert(
        self,
        marker_type: str,
        label: str,
        x: float,
        y: float,
        confidence: float,
    ) -> None:
        """去重插入：同类 marker 在 dedup_radius_m 内则加权平均更新，否则新增。"""
        for entry in self._markers:
            if entry.marker_type != marker_type or entry.label != label:
                continue
            dist = math.hypot(entry.x - x, entry.y - y)
            if dist < self._dedup_r:
                w = entry.count
                entry.x          = (entry.x * w + x) / (w + 1)
                entry.y          = (entry.y * w + y) / (w + 1)
                entry.confidence = max(entry.confidence, confidence)
                entry.count     += 1
                return
        self._markers.append(_MarkerEntry(marker_type, label, x, y, confidence))

    def _transform_to_map(self, ox: float, oy: float) -> tuple[float, float]:
        """把 odom 帧 (ox, oy) 平移到 map 帧。TF 不可用时原样返回。"""
        if self._tf_buffer is None:
            return ox, oy
        try:
            t = self._tf_buffer.lookup_transform(
                self._map_frame,
                self._odom_frame,
                rclpy.time.Time(),
                timeout=rclpy.duration.Duration(seconds=0.1),
            )
            tx = t.transform.translation.x
            ty = t.transform.translation.y
            return ox + tx, oy + ty
        except (LookupException, ConnectivityException, ExtrapolationException):
            return ox, oy

    def _publish_markers(self) -> None:
        """定时将去重 marker 列表发布为 PoseArray（map 帧）。"""
        msg = PoseArray()
        msg.header.stamp    = self.get_clock().now().to_msg()
        msg.header.frame_id = self._map_frame

        for entry in self._markers:
            pose = Pose()
            pose.position    = Point(x=entry.x, y=entry.y, z=0.0)
            pose.orientation = Quaternion(x=0.0, y=0.0, z=0.0, w=1.0)
            msg.poses.append(pose)

        self._pub.publish(msg)

    # ════════════════════════════════════════════════════════════════════
    # 工具
    # ════════════════════════════════════════════════════════════════════

    @staticmethod
    def _parse_event(data: str) -> Optional[dict]:
        """解析 'key=value key=value ...' 格式。缺少必需字段时返回 None。"""
        fields: dict[str, str] = {}
        for token in data.split():
            if '=' in token:
                k, _, v = token.partition('=')
                fields[k.strip()] = v.strip()

        required = {'type', 'label', 'x', 'y', 'confidence'}
        if not required.issubset(fields.keys()):
            return None

        try:
            result: dict = {
                'type':       fields['type'],
                'label':      fields['label'],
                'x':          float(fields['x']),
                'y':          float(fields['y']),
                'confidence': float(fields['confidence']),
            }
            # 可选像素坐标，用于深度反投影定位
            if 'cx_px' in fields:
                result['cx_px'] = float(fields['cx_px'])
            if 'cy_px' in fields:
                result['cy_px'] = float(fields['cy_px'])
            return result
        except ValueError:
            return None

    def get_markers_list(self) -> list[_MarkerEntry]:
        """供外部代码直接读取当前 marker 列表（单测用）。"""
        return list(self._markers)


# ---------------------------------------------------------------------------
def main(args=None) -> None:
    rclpy.init(args=args)
    node = PerceptionAdapterNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
