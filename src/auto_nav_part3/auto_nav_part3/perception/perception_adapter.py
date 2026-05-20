"""
perception_adapter.py — 感知结果适配器 (C_P.1)

职责
────
  1. 订阅 /part3/perception/marker_event（String，来自 greek_detector / colour_detector）
  2. 解析 key=value 格式，把 detector 发布的 odom 坐标转换到 Nav2 map 坐标
  3. 按 dedup_radius_m 去重：同一位置相同 marker 时更新 confidence，不新增条目
  4. 定期发布 /part3/perception/markers (geometry_msgs/PoseArray, map 帧)
  5. 提供 /part3/perception/get_markers (std_srvs/Trigger) 服务

话题 / 服务契约
────────────────
  订阅：
    /part3/perception/marker_event  std_msgs/String
  发布：
    /part3/perception/markers       geometry_msgs/PoseArray
  服务：
    /part3/perception/get_markers   std_srvs/Trigger

参数 (camera_bringup.launch.py)
────────────────────────────────
  dedup_radius_m   float   1.0                     同位置去重半径（m）
  publish_rate_hz  float   2.0                     定时发布 PoseArray 频率
  map_frame        str    'map'                    输出坐标帧
  odom_frame       str    'odom'                   detector 输出坐标帧
"""

from __future__ import annotations

import json
import math
import os
from typing import Optional

import rclpy
from geometry_msgs.msg import Pose, Point, PoseArray, Quaternion
from rclpy.node import Node
from std_msgs.msg import String
from std_srvs.srv import Trigger

try:
    from tf2_ros import Buffer, TransformListener
    _TF2_OK = True
except ImportError:
    _TF2_OK = False


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
        self.declare_parameter('dedup_radius_m',  1.0)
        self.declare_parameter('publish_rate_hz', 2.0)
        self.declare_parameter('map_frame',       'map')
        self.declare_parameter('odom_frame',      'odom')
        # 持久化路径：markers.json 保存到此目录，节点重启后自动恢复
        self.declare_parameter('waypoints_save_dir', 'artifacts/waypoints')

        gp = self.get_parameter
        self._dedup_r     = gp('dedup_radius_m').get_parameter_value().double_value
        self._rate        = gp('publish_rate_hz').get_parameter_value().double_value
        self._map_frame   = gp('map_frame').get_parameter_value().string_value
        self._odom_frame  = gp('odom_frame').get_parameter_value().string_value
        _save_dir         = gp('waypoints_save_dir').get_parameter_value().string_value
        os.makedirs(_save_dir, exist_ok=True)
        self._markers_json_path = os.path.join(_save_dir, 'markers.json')

        if _TF2_OK:
            self._tf_buffer = Buffer()
            self._tf_listener = TransformListener(self._tf_buffer, self)
        else:
            self._tf_buffer = None
            self.get_logger().warn('tf2 不可用，无法把 marker 坐标转换到 map')

        # ── 状态 ──────────────────────────────────────────────────────────
        self._markers: list[_MarkerEntry] = []

        # 启动时从文件恢复上次探索的 marker（节点重启后第二趟仍可用）
        self._load_markers()

        # ── 订阅 ──────────────────────────────────────────────────────────
        self.create_subscription(
            String,
            '/part3/perception/marker_event',
            self._on_marker_event,
            10,
        )

        # ── 发布 ──────────────────────────────────────────────────────────
        # 全部 marker（greek_letter + colour_obstacle）
        self._pub = self.create_publisher(PoseArray, '/part3/perception/markers', 10)
        # 仅希腊字母 marker，供 waypoint_service 直接读取（无需二次过滤）
        self._greek_pub = self.create_publisher(PoseArray, '/part3/perception/greek_markers', 10)

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
            f'frame={self._map_frame}  odom_frame={self._odom_frame}  rate={self._rate}Hz  '
            f'json={self._markers_json_path}'
        )

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

        marker_frame = fields.get('frame', self._odom_frame)
        map_xy = self._to_map(fields['x'], fields['y'], marker_frame)
        if map_xy is None:
            self.get_logger().warn(
                f"marker_event 无法转换到 {self._map_frame}，跳过: {msg.data}",
                throttle_duration_sec=5.0,
            )
            return
        mx, my = map_xy

        self._upsert(fields['type'], fields['label'], mx, my, fields['confidence'])

        self.get_logger().info(
            f"[ADAPTER] {fields['type']} {fields['label']} "
            f"{marker_frame}->{self._map_frame} pos=({mx:.2f},{my:.2f}) "
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

    def _upsert(
        self,
        marker_type: str,
        label: str,
        x: float,
        y: float,
        confidence: float,
        count: int = 1,
        save: bool = True,
    ) -> None:
        """去重插入：同类 marker 在 dedup_radius_m 内则加权平均更新，否则新增。"""
        for entry in self._markers:
            if entry.marker_type != marker_type or entry.label != label:
                continue
            dist = math.hypot(entry.x - x, entry.y - y)
            if dist <= self._dedup_r:
                w = entry.count
                new_count        = max(1, int(count))
                entry.x          = (entry.x * w + x * new_count) / (w + new_count)
                entry.y          = (entry.y * w + y * new_count) / (w + new_count)
                entry.confidence = max(entry.confidence, confidence)
                entry.count     += new_count
                if save:
                    self._save_markers()
                return
        self._markers.append(_MarkerEntry(marker_type, label, x, y, confidence))
        self._markers[-1].count = max(1, int(count))
        if save:
            self._save_markers()

    def _to_map(self, x: float, y: float, frame: str) -> Optional[tuple[float, float]]:
        """把 detector 坐标转换成 Nav2 使用的 map 坐标。"""
        if not (math.isfinite(x) and math.isfinite(y)):
            return None
        if frame == self._map_frame:
            return x, y
        if self._tf_buffer is None:
            return None

        try:
            transform = self._tf_buffer.lookup_transform(
                self._map_frame,
                frame,
                rclpy.time.Time(),
                timeout=rclpy.duration.Duration(seconds=0.2),
            )
            t = transform.transform.translation
            q = transform.transform.rotation
            yaw = math.atan2(
                2.0 * (q.w * q.z + q.x * q.y),
                1.0 - 2.0 * (q.y * q.y + q.z * q.z),
            )
            cos_yaw = math.cos(yaw)
            sin_yaw = math.sin(yaw)
            return (
                t.x + x * cos_yaw - y * sin_yaw,
                t.y + x * sin_yaw + y * cos_yaw,
            )
        except Exception as exc:
            self.get_logger().warn(
                f'TF {frame}->{self._map_frame} 转换失败: {exc}',
                throttle_duration_sec=5.0,
            )
            return None

    def _publish_markers(self) -> None:
        """定时将去重 marker 列表发布为 PoseArray（map 帧）。
        同时向 /part3/perception/greek_markers 单独发布希腊字母 marker，
        供 waypoint_service 直接消费，无需在下游再做 type 过滤。
        """
        now = self.get_clock().now().to_msg()

        all_msg = PoseArray()
        all_msg.header.stamp    = now
        all_msg.header.frame_id = self._map_frame

        greek_msg = PoseArray()
        greek_msg.header.stamp    = now
        greek_msg.header.frame_id = self._map_frame

        for entry in self._markers:
            pose = Pose()
            pose.position    = Point(x=entry.x, y=entry.y, z=0.0)
            pose.orientation = Quaternion(x=0.0, y=0.0, z=0.0, w=1.0)
            all_msg.poses.append(pose)
            if entry.marker_type == 'greek':
                greek_msg.poses.append(pose)

        self._pub.publish(all_msg)
        self._greek_pub.publish(greek_msg)

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
                'frame':      fields.get('frame', 'odom'),
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

    # ════════════════════════════════════════════════════════════════════
    # 持久化（JSON 文件）
    # ════════════════════════════════════════════════════════════════════

    def _save_markers(self) -> None:
        """把当前 _markers 列表序列化到 markers.json，写失败只打 warn 不崩溃。"""
        try:
            data = [
                {
                    'type':       e.marker_type,
                    'label':      e.label,
                    'frame':      self._map_frame,
                    'x':          round(e.x, 4),
                    'y':          round(e.y, 4),
                    'confidence': round(e.confidence, 4),
                    'count':      e.count,
                }
                for e in self._markers
            ]
            with open(self._markers_json_path, 'w', encoding='utf-8') as fh:
                json.dump(data, fh, indent=2, ensure_ascii=False)
        except Exception as exc:
            self.get_logger().warn(
                f'[PerceptionAdapter] markers.json 写入失败: {exc}',
                throttle_duration_sec=10.0,
            )

    def _load_markers(self) -> None:
        """启动时从 markers.json 恢复上次探索的 marker 列表。"""
        if not os.path.exists(self._markers_json_path):
            return
        try:
            with open(self._markers_json_path, 'r', encoding='utf-8') as fh:
                data = json.load(fh)
            raw_count = 0
            for item in data:
                raw_count += 1
                self._upsert(
                    marker_type=item['type'],
                    label=item['label'],
                    x=float(item['x']),
                    y=float(item['y']),
                    confidence=float(item['confidence']),
                    count=int(item.get('count', 1)),
                    save=False,
                )
            if raw_count != len(self._markers):
                self._save_markers()
            self.get_logger().info(
                f'[PerceptionAdapter] 从文件恢复 {len(self._markers)} 个 marker '
                f'（原始 {raw_count} 条，已去重）: '
                f'{self._markers_json_path}'
            )
        except Exception as exc:
            self.get_logger().warn(
                f'[PerceptionAdapter] markers.json 加载失败（将从空列表开始）: {exc}'
            )

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
