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
             confidence=<f> image=<path> range_m=<f>"
 
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
from typing import Optional
 
import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge
from nav_msgs.msg import Odometry
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String
 
# ---------------------------------------------------------------------------
# HSV colour ranges — tested against OAK-D on James Oval
# Red wraps around hue=0 in OpenCV HSV so two ranges are needed
# ---------------------------------------------------------------------------
_COLOUR_RANGES: dict[str, list[tuple]] = {
    "red": [
        ((0,   87,  11), (10,  255, 255)),
        ((136, 87,  11), (180, 255, 255)),
    ],
    "yellow": [
        ((20, 100, 100), (30, 255, 255)),
    ],
}
 
 
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
 
        gp = self.get_parameter
        self._photo_dir  = gp("photo_dir").get_parameter_value().string_value
        self._jpeg_q     = gp("jpeg_quality").get_parameter_value().integer_value
        self._cooldown   = gp("detection_cooldown_s").get_parameter_value().double_value
        self._min_area   = gp("min_area_px").get_parameter_value().integer_value
        self._max_area   = gp("max_area_px").get_parameter_value().integer_value
        self._img_w      = gp("image_width").get_parameter_value().integer_value
        self._img_h      = gp("image_height").get_parameter_value().integer_value
 
        os.makedirs(self._photo_dir, exist_ok=True)
 
        # ── state ─────────────────────────────────────────────────────────
        self._bridge              = CvBridge()
        self._robot_x: float      = 0.0
        self._robot_y: float      = 0.0
        self._robot_yaw: float    = 0.0
        self._depth_image         = None
        self._last_detected: dict[str, float] = {}
 
        # ── subscribers ───────────────────────────────────────────────────
        self.create_subscription(
            Image, "/oak/rgb/image_raw", self._on_image, 10)
        self.create_subscription(
            Image, "/oak/stereo/depth", self._on_depth, 10)
        self.create_subscription(
            Odometry, "/odom", self._on_odom, 10)
 
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
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        self._robot_yaw = math.atan2(siny_cosp, cosy_cosp)

    def _on_depth(self, msg: Image) -> None:
        try:
            self._depth_image = self._bridge.imgmsg_to_cv2(
                msg, desired_encoding="passthrough")
        except Exception as exc:
            self.get_logger().warn(
                f"Depth error: {exc}", throttle_duration_sec=5.0)

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
                mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
 
            for contour in contours:
                area = cv2.contourArea(contour)
                if not (self._min_area <= area <= self._max_area):
                    continue
 
                if self._on_cooldown(colour):
                    continue
 
                x, y, w, h   = cv2.boundingRect(contour)
                confidence    = min(1.0, area / 5000.0)
                cx_f          = float(x + w / 2)
                cy_f          = float(y + h / 2)
                range_m       = self._estimate_range(cx_f, cy_f)
                obj_x, obj_y  = self._calc_object_position(cx_f, range_m)
                img_path      = self._save_photo(bgr, colour, x, y, w, h)
                self._publish(colour, confidence, img_path, range_m, obj_x, obj_y)

                range_str = f"{range_m:.2f}m" if math.isfinite(range_m) else "no depth"
                self.get_logger().info(
                    f"[COLOUR] {colour} | conf={confidence:.2f} | "
                    f"range={range_str} | obj=({obj_x:.2f},{obj_y:.2f})"
                )
                break
 
    # ════════════════════════════════════════════════════════════════════
    # Helpers
    # ════════════════════════════════════════════════════════════════════
 
    def _on_cooldown(self, label: str) -> bool:
        return (time.monotonic() - self._last_detected.get(label, 0.0)) < self._cooldown
 
    def _save_photo(
        self,
        bgr: np.ndarray,
        label: str,
        x: int, y: int, w: int, h: int,
    ) -> str:
        annotated = bgr.copy()
        colour_bgr = (0, 0, 220) if label == "red" else (0, 220, 220)
        cv2.rectangle(annotated, (x, y), (x + w, y + h), colour_bgr, 2)
        cv2.putText(
            annotated,
            f"{label} ({self._robot_x:.1f},{self._robot_y:.1f})",
            (x, max(0, y - 8)),
            cv2.FONT_HERSHEY_SIMPLEX, 0.65, colour_bgr, 2,
        )
        ts   = datetime.now().strftime("%Y%m%d_%H%M%S_%f")[:-3]
        path = os.path.join(self._photo_dir, f"colour_{label}_{ts}.jpg")
        cv2.imwrite(path, annotated,
                    [cv2.IMWRITE_JPEG_QUALITY, self._jpeg_q])
        return path
    
    def _estimate_range(self, cx: float, cy: float) -> float:
        """Sample depth at (cx,cy) — returns metres or nan."""
        if self._depth_image is None:
            return float("nan")
        h, w = self._depth_image.shape[:2]
        r  = 5
        x0 = max(0, int(cx) - r)
        x1 = min(w, int(cx) + r + 1)
        y0 = max(0, int(cy) - r)
        y1 = min(h, int(cy) + r + 1)
        roi = self._depth_image[y0:y1, x0:x1].astype(np.float32)
        roi = roi[np.isfinite(roi) & (roi > 0.01)]
        if roi.size == 0:
            return float("nan")
        val = float(np.median(roi))
        return val / 1000.0 if val > 20 else val

    def _calc_object_position(self, cx_px: float, range_m: float) -> tuple[float, float]:
        """Calculate object world coordinates from robot pose + depth + bearing."""
        if not math.isfinite(range_m):
            return self._robot_x, self._robot_y
        hfov_rad = math.radians(69.0)
        bearing  = -((cx_px / 640.0) - 0.5) * hfov_rad
        heading  = self._robot_yaw + bearing
        obj_x    = self._robot_x + range_m * math.cos(heading)
        obj_y    = self._robot_y + range_m * math.sin(heading)
        return obj_x, obj_y
 
    def _publish(self, label: str, confidence: float,
                 img_path: str, range_m: float = float("nan"),
                 obj_x: float = float("nan"),
                 obj_y: float = float("nan")) -> None:
        range_str = f"{range_m:.3f}" if math.isfinite(range_m) else "nan"
        ox_str    = f"{obj_x:.4f}"   if math.isfinite(obj_x)   else "nan"
        oy_str    = f"{obj_y:.4f}"   if math.isfinite(obj_y)   else "nan"
        payload = (
            f"type=colour "
            f"label={label} "
            f"x={self._robot_x:.4f} "
            f"y={self._robot_y:.4f} "
            f"confidence={confidence:.3f} "
            f"image={img_path} "
            f"range_m={range_str} "
            f"obj_x={ox_str} "
            f"obj_y={oy_str}"
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
