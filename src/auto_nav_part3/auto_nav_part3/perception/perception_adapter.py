"""
perception_adapter.py — perception result adapter (C_P.1)

Responsibilities
────────────────
  1. Subscribe to /part3/perception/marker_event (String, from greek_detector / colour_detector)
  2. Parse key=value format and transform detector odom coordinates to Nav2 map coordinates
  3. Deduplicate by dedup_radius_m: update confidence for existing markers at the same position, do not add new entries
  4. Periodically publish /part3/perception/markers (geometry_msgs/PoseArray, map frame)
  5. Provide /part3/perception/get_markers (std_srvs/Trigger) service

Topic / service contract
────────────────────────
  subscribe:
    /part3/perception/marker_event  std_msgs/String
  publish:
    /part3/perception/markers       geometry_msgs/PoseArray
  service:
    /part3/perception/get_markers   std_srvs/Trigger

Parameters (camera_bringup.launch.py)
──────────────────────────────────────
  dedup_radius_m   float   1.0    deduplication radius for same-location markers (m)
  publish_rate_hz  float   2.0    periodic PoseArray publish rate (Hz)
  map_frame        str    'map'   output coordinate frame
  odom_frame       str    'odom'  coordinate frame used by detectors
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
    """Single marker record held in memory."""
    __slots__ = ('marker_type', 'label', 'x', 'y', 'confidence', 'count')

    def __init__(self, marker_type: str, label: str, x: float, y: float, confidence: float):
        self.marker_type = marker_type
        self.label       = label
        self.x           = x
        self.y           = y
        self.confidence  = confidence
        self.count       = 1


class PerceptionAdapterNode(Node):
    """Subscribe to marker_event, deduplicate, publish as PoseArray, and serve get_markers."""

    def __init__(self) -> None:
        super().__init__('perception_adapter')

        # ── parameters ────────────────────────────────────────────────────
        self.declare_parameter('dedup_radius_m',   2.0)
        self.declare_parameter('publish_rate_hz',  2.0)
        self.declare_parameter('map_frame',        'map')
        self.declare_parameter('odom_frame',       'odom')
        # persistence path: markers.json is saved here and restored automatically on node restart
        self.declare_parameter('waypoints_save_dir', 'artifacts/waypoints')
        # confirmation threshold: entries with count < N and confidence < 0.90 are treated as "unconfirmed" and not published downstream
        # an entry is confirmed after N observations or a single observation with confidence >= 0.90; confirmed=true is written to JSON
        self.declare_parameter('min_confirm_count', 2)

        gp = self.get_parameter
        self._dedup_r         = gp('dedup_radius_m').get_parameter_value().double_value
        self._rate            = gp('publish_rate_hz').get_parameter_value().double_value
        self._map_frame       = gp('map_frame').get_parameter_value().string_value
        self._odom_frame      = gp('odom_frame').get_parameter_value().string_value
        self._min_confirm     = gp('min_confirm_count').get_parameter_value().integer_value
        _save_dir         = gp('waypoints_save_dir').get_parameter_value().string_value
        os.makedirs(_save_dir, exist_ok=True)
        self._markers_json_path = os.path.join(_save_dir, 'markers.json')

        if _TF2_OK:
            self._tf_buffer = Buffer()
            self._tf_listener = TransformListener(self._tf_buffer, self)
        else:
            self._tf_buffer = None
            self.get_logger().warn('tf2 unavailable — cannot transform marker coordinates to map frame')

        # ── state ─────────────────────────────────────────────────────────
        self._markers: list[_MarkerEntry] = []

        # restore markers from the previous exploration run on startup (available after node restart)
        self._load_markers()

        # ── subscribers ───────────────────────────────────────────────────
        self.create_subscription(
            String,
            '/part3/perception/marker_event',
            self._on_marker_event,
            10,
        )

        # ── publishers ────────────────────────────────────────────────────
        # all markers (greek_letter + colour_obstacle)
        self._pub = self.create_publisher(PoseArray, '/part3/perception/markers', 10)
        # greek letter markers only, for waypoint_service to read without secondary filtering
        self._greek_pub = self.create_publisher(PoseArray, '/part3/perception/greek_markers', 10)

        # ── services ──────────────────────────────────────────────────────
        self.create_service(
            Trigger,
            '/part3/perception/get_markers',
            self._on_get_markers,
        )

        # ── periodic publish ───────────────────────────────────────────────
        self.create_timer(1.0 / self._rate, self._publish_markers)

        self.get_logger().info(
            f'PerceptionAdapter ready  dedup={self._dedup_r}m  '
            f'min_confirm={self._min_confirm}  '
            f'frame={self._map_frame}  odom_frame={self._odom_frame}  rate={self._rate}Hz  '
            f'json={self._markers_json_path}'
        )

    # ════════════════════════════════════════════════════════════════════
    # Event handling
    # ════════════════════════════════════════════════════════════════════

    def _on_marker_event(self, msg: String) -> None:
        fields = self._parse_event(msg.data.strip())
        if fields is None:
            self.get_logger().warn(
                f'cannot parse marker_event: "{msg.data}"',
                throttle_duration_sec=5.0,
            )
            return

        marker_frame = fields.get('frame', self._odom_frame)
        map_xy = self._to_map(fields['x'], fields['y'], marker_frame)
        if map_xy is None:
            self.get_logger().warn(
                f"cannot transform marker_event to {self._map_frame}, skipping: {msg.data}",
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
        """Service callback: publish one PoseArray immediately and return the current marker count."""
        self._publish_markers()
        response.success = True
        response.message = f'markers={len(self._markers)}'
        return response

    # ════════════════════════════════════════════════════════════════════
    # Core logic
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
        """Deduplicated upsert: same-type marker within dedup_radius_m gets a weighted average update, otherwise a new entry is added."""
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
        """Transform detector coordinates to Nav2 map coordinates."""
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
                f'TF {frame}->{self._map_frame} transform failed: {exc}',
                throttle_duration_sec=5.0,
            )
            return None

    def _publish_markers(self) -> None:
        """Periodically publish the deduplicated marker list as a PoseArray in the map frame.
        Only confirmed entries (count>=min_confirm_count or confidence>=0.90) are published,
        filtering out low-quality single-frame false positives to keep noise out of waypoint_service.
        Greek letter markers are also published separately on /part3/perception/greek_markers.
        """
        now = self.get_clock().now().to_msg()

        all_msg = PoseArray()
        all_msg.header.stamp    = now
        all_msg.header.frame_id = self._map_frame

        greek_msg = PoseArray()
        greek_msg.header.stamp    = now
        greek_msg.header.frame_id = self._map_frame

        for entry in self._markers:
            # skip unconfirmed entries: prevents single false-positive detections from polluting downstream waypoint planning
            if not self._should_export_marker(entry):
                continue
            pose = Pose()
            pose.position    = Point(x=entry.x, y=entry.y, z=0.0)
            pose.orientation = Quaternion(x=0.0, y=0.0, z=0.0, w=1.0)
            all_msg.poses.append(pose)
            if entry.marker_type == 'greek':
                greek_msg.poses.append(pose)

        self._pub.publish(all_msg)
        self._greek_pub.publish(greek_msg)

    # ════════════════════════════════════════════════════════════════════
    # Helpers
    # ════════════════════════════════════════════════════════════════════

    def _should_export_marker(self, entry: _MarkerEntry) -> bool:
        """Final export filter: Greek markers seen only once are not published and not written to markers.json.

        Note: entries are NOT removed from self._markers, so count can still accumulate from 1 to 2 at runtime.
        To remove this hard filter later, delete the greek/count check inside this function.
        """
        if entry.marker_type == 'greek' and entry.count <= 1:
            return False
        return entry.count >= self._min_confirm or entry.confidence >= 0.90

    @staticmethod
    def _parse_event(data: str) -> Optional[dict]:
        """Parse 'key=value key=value ...' format. Returns None if any required field is missing."""
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
            # optional pixel coordinates for depth back-projection localisation
            if 'cx_px' in fields:
                result['cx_px'] = float(fields['cx_px'])
            if 'cy_px' in fields:
                result['cy_px'] = float(fields['cy_px'])
            return result
        except ValueError:
            return None

    # ════════════════════════════════════════════════════════════════════
    # Persistence (JSON file)
    # ════════════════════════════════════════════════════════════════════

    def _save_markers(self) -> None:
        """Serialise the current _markers list to markers.json; a write failure logs a warning and does not crash.
        confirmed=true means count>=min_confirm_count or confidence>=0.90.
        Single-observation Greek detections are not saved, preventing false positives like eta from polluting final waypoints."""
        try:
            os.makedirs(os.path.dirname(self._markers_json_path), exist_ok=True)
            data = [
                {
                    'type':       e.marker_type,
                    'label':      e.label,
                    'frame':      self._map_frame,
                    'x':          round(e.x, 4),
                    'y':          round(e.y, 4),
                    'confidence': round(e.confidence, 4),
                    'count':      e.count,
                    'confirmed':  (e.count >= self._min_confirm or e.confidence >= 0.90),
                }
                for e in self._markers
                if self._should_export_marker(e)
            ]
            with open(self._markers_json_path, 'w', encoding='utf-8') as fh:
                json.dump(data, fh, indent=2, ensure_ascii=False)
        except Exception as exc:
            self.get_logger().warn(
                f'[PerceptionAdapter] markers.json write failed: {exc}',
                throttle_duration_sec=10.0,
            )

    def _load_markers(self) -> None:
        """Restore the marker list from markers.json on startup."""
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
                f'[PerceptionAdapter] restored {len(self._markers)} markers from file '
                f'(raw {raw_count} entries, after deduplication): '
                f'{self._markers_json_path}'
            )
        except Exception as exc:
            self.get_logger().warn(
                f'[PerceptionAdapter] markers.json load failed (starting from empty list): {exc}'
            )

    def get_markers_list(self) -> list[_MarkerEntry]:
        """Return a copy of the current marker list (for unit tests)."""
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
