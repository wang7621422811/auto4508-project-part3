"""
colour_detector.py — Detects red and yellow coloured obstacles using HSV
                     colour masking on the OAK-D camera feed.

Part 3 Task 4: "Take photos and note the location of any yellow or red
colour obstacles in the area as these are of special interest."

Subscribes:
  /oak/rgb/image_raw      sensor_msgs/Image   — colour camera feed
  /odom                   nav_msgs/Odometry   — robot position for location tagging

Publishes:
  /part3/perception/marker_event  std_msgs/String
    format: "type=colour label=<red|yellow> x=<f> y=<f>
             confidence=<f> image=<path> cx_px=<i> cy_px=<i> range_m=<f>"

Parameters
----------
  photo_dir          — directory to save annotated photos
                       default: "artifacts/perception_photos"
                       On robot set to absolute path e.g. /home/pioneer/artifacts/photos
  jpeg_quality       — JPEG save quality 0-100 (default 90)
  detection_cooldown_s — seconds before same colour re-detected (default 5.0)
  min_area_px        — minimum blob area in pixels (default 300)
  max_area_px        — maximum blob area in pixels (default 80000)
  image_width        — expected image width (default 640)
  image_height       — expected image height (default 480)
  new_object_dist_m  — minimum robot travel distance (m) before same label
                       triggers another save (default 1.5); prevents disk bloat
                       when stationary, yet allows detecting two separate objects

Test without robot:
  ros2 run auto_nav_part3 colour_detector --ros-args \
    -p photo_dir:=/tmp/colour_test

  # Publish a fake image to trigger detection
  ros2 run image_publisher image_publisher_node red_object.jpg \
    --ros-args -r /image:=/oak/rgb/image_raw
"""

from __future__ import annotations

import math
import os
import time
from datetime import datetime

import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge
from nav_msgs.msg import Odometry
from rclpy.node import Node
from sensor_msgs.msg import Image, LaserScan
from std_msgs.msg import String

from .perception_utils import lidar_to_odom

# ---------------------------------------------------------------------------
# HSV colour ranges — tested against OAK-D on James Oval
# Red wraps around hue=0 in OpenCV HSV so two ranges are needed
# ---------------------------------------------------------------------------
_COLOUR_RANGES: dict[str, list[tuple]] = {
    "red": [
        ((0,   87,  11), (10,  255, 255)),   # 低段红色（H 0-10）
        ((168, 87,  11), (180, 255, 255)),   # 高段红色（H 168-180）
        # 原下界 136 把紫色（H≈135-167）也纳入了；调整到 168 排除紫色/品红误检
    ],
    "yellow": [
        ((20, 100, 100), (30, 255, 255)),   # 黄色（H 20-30）
    ],
}

_TRUSTED_MIN_AREA_PX = 2500
_MAX_CENTER_OFFSET_FRAC = 0.35


class ColourDetectorNode(Node):
    """Detect red and yellow obstacles and publish marker events."""

    def __init__(self) -> None:
        super().__init__("colour_detector")

        # ── parameters ───────────────────────────────────────────────────
        self.declare_parameter("photo_dir",            "artifacts/perception_photos")
        self.declare_parameter("jpeg_quality",         90)
        self.declare_parameter("detection_cooldown_s", 5.0)
        self.declare_parameter("min_area_px",          300)
        self.declare_parameter("max_area_px",          80000)
        self.declare_parameter("image_width",          640)
        self.declare_parameter("image_height",         480)
        self.declare_parameter("new_object_dist_m",    1.5)

        gp = self.get_parameter
        self._photo_dir    = gp("photo_dir").get_parameter_value().string_value
        self._jpeg_q       = gp("jpeg_quality").get_parameter_value().integer_value
        self._cooldown     = gp("detection_cooldown_s").get_parameter_value().double_value
        self._min_area     = gp("min_area_px").get_parameter_value().integer_value
        self._max_area     = gp("max_area_px").get_parameter_value().integer_value
        self._img_w        = gp("image_width").get_parameter_value().integer_value
        self._img_h        = gp("image_height").get_parameter_value().integer_value
        self._new_obj_dist = gp("new_object_dist_m").get_parameter_value().double_value

        os.makedirs(self._photo_dir, exist_ok=True)

        # ── state ─────────────────────────────────────────────────────────
        self._bridge              = CvBridge()
        self._robot_x: float      = 0.0
        self._robot_y: float      = 0.0
        self._robot_yaw: float    = 0.0
        self._latest_scan         = None          # 最新 LaserScan 消息
        self._scan_stamp: float   = 0.0           # 上次 scan 到达的 monotonic 时间
        self._cam_hfov: float     = 1.089
        self._last_detected: dict[str, float] = {}  # label -> timestamp
        # label -> [(robot_x, robot_y), ...] 已存图时的机器人位置列表（空间去重）
        self._saved_positions: dict[str, list] = {}

        # ── subscribers ───────────────────────────────────────────────────
        self.create_subscription(
            Image, "/oak/rgb/image_raw", self._on_image, 10)
        self.create_subscription(
            Odometry, "/odom", self._on_odom, 10)
        self.create_subscription(
            LaserScan, "/scan", self._on_scan, 10)

        # ── publishers ────────────────────────────────────────────────────
        self._pub = self.create_publisher(
            String, "/part3/perception/marker_event", 10)

        self.get_logger().info("ColourDetector ready.")

    # ════════════════════════════════════════════════════════════════════
    # Subscribers
    # ════════════════════════════════════════════════════════════════════

    def _on_odom(self, msg: Odometry) -> None:
        self._robot_x = msg.pose.pose.position.x
        self._robot_y = msg.pose.pose.position.y
        q = msg.pose.pose.orientation
        self._robot_yaw = math.atan2(
            2.0 * (q.w * q.z + q.x * q.y),
            1.0 - 2.0 * (q.y * q.y + q.z * q.z),
        )

    def _on_scan(self, msg: LaserScan) -> None:
        self._latest_scan = msg
        self._scan_stamp  = time.monotonic()
        self.get_logger().debug(
            f"[SCAN] {len(msg.ranges)} 光束  "
            f"角度 [{math.degrees(msg.angle_min):.0f}°, {math.degrees(msg.angle_max):.0f}°]  "
            f"测距范围 [{msg.range_min:.2f}, {msg.range_max:.2f}]m",
            throttle_duration_sec=10.0,
        )

    def _on_image(self, msg: Image) -> None:
        try:
            bgr = self._bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        except Exception as exc:
            self.get_logger().warn(
                f"Image decode error: {exc}", throttle_duration_sec=5.0)
            return
        self._detect(bgr)

    # ════════════════════════════════════════════════════════════════════
    # Detection
    # ════════════════════════════════════════════════════════════════════

    def _detect(self, bgr: np.ndarray) -> None:
        # ── 雷达可用性检查（早期退出，避免后续无效计算）────────────────────
        if self._latest_scan is None:
            self.get_logger().warn(
                "[COLOUR] /scan 尚未收到，跳过本帧所有颜色检测",
                throttle_duration_sec=5.0,
            )
            return
        staleness = time.monotonic() - self._scan_stamp
        if staleness > 1.0:
            self.get_logger().warn(
                f"[COLOUR] 雷达数据已过期 {staleness:.1f}s（/scan 话题可能已停止），跳过帧",
                throttle_duration_sec=5.0,
            )
            return

        hsv    = cv2.cvtColor(bgr, cv2.COLOR_BGR2HSV)
        kernel = np.ones((5, 5), "uint8")

        for colour, ranges in _COLOUR_RANGES.items():
            # Build combined mask
            mask = np.zeros(bgr.shape[:2], dtype=np.uint8)
            for lo, hi in ranges:
                mask |= cv2.inRange(hsv, lo, hi)

            # Clean up mask
            mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN,  kernel, iterations=1)
            mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel, iterations=2)
            mask = cv2.dilate(mask, kernel, iterations=1)

            contours, _ = cv2.findContours(
                mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            candidates = [
                c for c in contours
                if self._min_area <= cv2.contourArea(c) <= self._max_area
            ]
            if not candidates or self._on_cooldown(colour):
                continue

            contour = max(candidates, key=cv2.contourArea)
            area = cv2.contourArea(contour)
            x, y, w, h = cv2.boundingRect(contour)
            cx_px = x + w // 2
            cy_px = y + h // 2
            center_offset = abs(cx_px - (bgr.shape[1] / 2.0)) / bgr.shape[1]
            if area < _TRUSTED_MIN_AREA_PX or center_offset > _MAX_CENTER_OFFSET_FRAC:
                continue

            # ── 用雷达测距估算障碍物 odom 坐标 ─────────────────────────────
            fx = (bgr.shape[1] / 2.0) / math.tan(self._cam_hfov / 2.0)
            # bearing: 正值=图像左侧=机器人左侧(+Y), 负值=右侧(-Y)
            bearing_rad = math.atan2((bgr.shape[1] / 2.0) - cx_px, fx)
            beam_idx = int(round(
                (bearing_rad - self._latest_scan.angle_min)
                / self._latest_scan.angle_increment
            ))
            obs = lidar_to_odom(
                self._latest_scan, cx_px, bgr.shape[1],
                self._cam_hfov,
                self._robot_x, self._robot_y, self._robot_yaw,
            )
            if obs is None:
                self.get_logger().warn(
                    f"[COLOUR] {colour}: 雷达方位角 {math.degrees(bearing_rad):.1f}° "
                    f"(beam {beam_idx}/{len(self._latest_scan.ranges)}) 无有效测距，跳过",
                    throttle_duration_sec=5.0,
                )
                continue
            obs_x, obs_y, range_m = obs

            self.get_logger().info(
                f"[COLOUR] {colour} | area={int(area)}px  cx_px={cx_px}  "
                f"bearing={math.degrees(bearing_rad):.1f}°  beam={beam_idx}  "
                f"range={range_m:.3f}m | "
                f"robot=({self._robot_x:.2f},{self._robot_y:.2f})  "
                f"yaw={math.degrees(self._robot_yaw):.1f}° | "
                f"obs=({obs_x:.3f},{obs_y:.3f})"
            )

            confidence = min(1.0, area / 5000.0)
            img_path = self._maybe_save_photo(bgr, colour, x, y, w, h, obs_x, obs_y)
            self._publish(colour, confidence, img_path, cx_px, cy_px, obs_x, obs_y, range_m)

    # ════════════════════════════════════════════════════════════════════
    # Helpers
    # ════════════════════════════════════════════════════════════════════

    def _maybe_save_photo(
        self, bgr: np.ndarray, colour: str, x: int, y: int, w: int, h: int,
        obs_x: float, obs_y: float,
    ) -> str:
        """只在新位置存图：用观测坐标去重，防止绕同一障碍物转圈时重复写入。"""
        if not self._is_new_location(colour, obs_x, obs_y):
            return ""
        self._saved_positions.setdefault(colour, []).append((obs_x, obs_y))
        return self._save_photo(bgr, colour, x, y, w, h, obs_x, obs_y)

    def _is_new_location(self, label: str, obs_x: float, obs_y: float) -> bool:
        """估算 marker 坐标与所有已记录位置均超过 new_object_dist_m 时返回 True。
        用观测坐标而非机器人坐标，避免绕障碍物行走时同一物体被重复上报。"""
        positions = self._saved_positions.get(label, [])
        if not positions:
            return True
        min_dist = min(math.hypot(obs_x - px, obs_y - py) for px, py in positions)
        return min_dist > self._new_obj_dist

    def _on_cooldown(self, label: str) -> bool:
        return (time.monotonic() - self._last_detected.get(label, 0.0)) < self._cooldown

    def _save_photo(
        self,
        bgr: np.ndarray,
        label: str,
        x: int, y: int, w: int, h: int,
        obs_x: float, obs_y: float,
    ) -> str:
        annotated  = bgr.copy()
        colour_bgr = (0, 0, 220) if label == "red" else (0, 220, 220)
        cv2.rectangle(annotated, (x, y), (x + w, y + h), colour_bgr, 2)
        cv2.putText(
            annotated,
            f"{label} @ ({obs_x:.2f},{obs_y:.2f})",
            (x, max(0, y - 8)),
            cv2.FONT_HERSHEY_SIMPLEX, 0.65, colour_bgr, 2,
        )
        ts   = datetime.now().strftime("%Y%m%d_%H%M%S_%f")[:-3]
        path = os.path.join(self._photo_dir, f"colour_{label}_{ts}.jpg")
        cv2.imwrite(path, annotated,
                    [cv2.IMWRITE_JPEG_QUALITY, self._jpeg_q])
        return path

    def _publish(self, label: str, confidence: float, img_path: str,
                 cx_px: int = 0, cy_px: int = 0,
                 obs_x: float = 0.0, obs_y: float = 0.0,
                 range_m: float = float("nan")) -> None:
        payload = (
            f"type=colour "
            f"label={label} "
            f"frame=odom "
            f"x={obs_x:.4f} "
            f"y={obs_y:.4f} "
            f"confidence={confidence:.3f} "
            f"image={img_path} "
            f"cx_px={cx_px} "
            f"cy_px={cy_px} "
            f"range_m={range_m:.3f}"
        )
        self._pub.publish(String(data=payload))
        self._last_detected[label] = time.monotonic()


# ---------------------------------------------------------------------------
def main(args=None) -> None:
    rclpy.init(args=args)
    node = ColourDetectorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
