"""
greek_detector.py — Detects hand-drawn Greek letters on white paper stuck
                    to grey buckets using the OAK-D camera.

Part 3 Task 3: "Use image recognition to determine the [Greek letter]
and note its location."

Pipeline per frame:
  1. Detect white paper rectangle on bucket
  2. Perspective-correct the crop
  3. Isolate ink strokes (MNIST-style: blur → adaptive threshold → contour crop)
  4. Run ONNX classifier
  5. Publish marker event if confidence >= threshold

Subscribes:
  /oak/rgb/image_raw      sensor_msgs/Image   — colour camera feed
  /odom                   nav_msgs/Odometry   — robot position for location tagging
  /scan                   sensor_msgs/LaserScan — LiDAR range (replaces depth camera)

Publishes:
  /part3/perception/marker_event  std_msgs/String
    format: "type=greek label=<name> x=<f> y=<f>
             confidence=<f> image=<path> range_m=<f>"

Parameters
----------
  greek_model_path      — absolute path to greek_letters.onnx
                          Leave empty to disable (node still runs, just no detections)
  photo_dir             — directory to save annotated photos (default: artifacts/photos)
  jpeg_quality          — JPEG quality 0-100 (default 90)
  detection_cooldown_s  — seconds before same label re-detected (default 5.0)
  min_confidence        — minimum model confidence to publish (default 0.5)
  image_width           — expected image width (default 640)
  image_height          — expected image height (default 480)
  new_object_dist_m     — min distance (m) from previous saved position to save again (default 1.5)

Test without robot:
  ros2 run auto_nav_part3 greek_detector --ros-args \
    -p greek_model_path:=/path/to/greek_letters.onnx \
    -p photo_dir:=/tmp/greek_test

  ros2 run image_publisher image_publisher_node bucket_photo.jpg \
    --ros-args -r /image:=/oak/rgb/image_raw

  ros2 topic echo /part3/perception/marker_event
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
from sensor_msgs.msg import Image, LaserScan
from std_msgs.msg import String

from .perception_utils import lidar_to_odom

# Alphabetical order — must match training label order exactly
_LABELS: list[str] = sorted([
    "alpha", "beta", "delta", "eta", "gamma",
    "lambda", "mu", "psi", "rho", "tau",
])

IMG_SIZE = 64   # model input size


class GreekDetectorNode(Node):
    """Detect hand-drawn Greek letters and publish marker events."""

    def __init__(self) -> None:
        super().__init__("greek_detector")

        # ── parameters ───────────────────────────────────────────────────
        self.declare_parameter("greek_model_path",     "")
        self.declare_parameter("photo_dir",            "artifacts/photos")
        self.declare_parameter("jpeg_quality",         90)
        self.declare_parameter("detection_cooldown_s", 5.0)
        self.declare_parameter("min_confidence",       0.5)
        self.declare_parameter("image_width",          640)
        self.declare_parameter("image_height",         480)
        self.declare_parameter("new_object_dist_m",    1.5)

        gp = self.get_parameter
        self._model_path   = gp("greek_model_path").get_parameter_value().string_value
        self._photo_dir    = gp("photo_dir").get_parameter_value().string_value
        self._jpeg_q       = gp("jpeg_quality").get_parameter_value().integer_value
        self._cooldown     = gp("detection_cooldown_s").get_parameter_value().double_value
        self._min_conf     = gp("min_confidence").get_parameter_value().double_value
        self._img_w        = gp("image_width").get_parameter_value().integer_value
        self._img_h        = gp("image_height").get_parameter_value().integer_value
        self._new_obj_dist = gp("new_object_dist_m").get_parameter_value().double_value

        os.makedirs(self._photo_dir, exist_ok=True)

        # ── state ─────────────────────────────────────────────────────────
        self._bridge             = CvBridge()
        self._robot_x: float     = 0.0
        self._robot_y: float     = 0.0
        self._robot_yaw: float   = 0.0
        self._latest_scan        = None          # most recent LaserScan message
        self._scan_stamp: float  = 0.0           # monotonic time of last scan arrival
        self._cam_hfov: float    = 1.089         # OAK-D / sim rgbd_camera HFOV ≈ 62°
        self._last_detected: dict[str, float] = {}
        # label -> [(robot_x, robot_y), ...] positions where photo was saved
        self._saved_positions: dict[str, list] = {}
        self._model              = None
        self._model_loaded       = False

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

        self.get_logger().set_level(rclpy.logging.LoggingSeverity.DEBUG)
        self.get_logger().info(
            f"GreekDetector ready | model='{self._model_path}' "
            f"lidar=/scan  min_conf={self._min_conf} "
            f"cooldown={self._cooldown}s photo_dir='{self._photo_dir}'"
        )

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
        # self.get_logger().debug(
        #     f"[SCAN] {len(msg.ranges)} beams  "
        #     f"angle [{math.degrees(msg.angle_min):.0f}°, {math.degrees(msg.angle_max):.0f}°]  "
        #     f"range [{msg.range_min:.2f}, {msg.range_max:.2f}]m",
        #     throttle_duration_sec=10.0,
        # )

    def _on_image(self, msg: Image) -> None:
        if not self._model_loaded:
            self._load_model()

        try:
            bgr = self._bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        except Exception as exc:
            self.get_logger().warn(
                f"Image decode error: {exc}", throttle_duration_sec=5.0)
            return

        self._detect(bgr)

    # ════════════════════════════════════════════════════════════════════
    # Model loading
    # ════════════════════════════════════════════════════════════════════

    def _load_model(self) -> None:
        self._model_loaded = True
        if not self._model_path:
            self.get_logger().warn(
                "greek_model_path is not set — Greek detection DISABLED. "
                "Set parameter greek_model_path to your .onnx file path."
            )
            return
        if not os.path.exists(self._model_path):
            self.get_logger().error(
                f"Model file not found: {self._model_path}"
            )
            return
        try:
            import onnxruntime as ort
            self._model = ort.InferenceSession(
                self._model_path,
                providers=["CPUExecutionProvider"],
            )
            self.get_logger().info(f"Model loaded: {self._model_path}")
        except Exception as exc:
            self.get_logger().error(f"Failed to load ONNX model: {exc}")

    # ════════════════════════════════════════════════════════════════════
    # Detection pipeline
    # ════════════════════════════════════════════════════════════════════

    def _detect(self, bgr: np.ndarray) -> None:
        if self._model is None:
            self.get_logger().warn(
                "Greek model not loaded — set greek_model_path parameter.",
                throttle_duration_sec=10.0,
            )
            return

        # ── lidar availability check (early exit to avoid wasted computation) ──
        if self._latest_scan is None:
            self.get_logger().warn(
                "[GREEK] /scan not yet received, skipping detection this frame",
                throttle_duration_sec=5.0,
            )
            return
        staleness = time.monotonic() - self._scan_stamp
        if staleness > 1.0:
            self.get_logger().warn(
                f"[GREEK] lidar data stale by {staleness:.1f}s (/scan may have stopped), skipping frame",
                throttle_duration_sec=5.0,
            )
            return

        # Step 1 — detect the white/grey paper board before cropping the letter region
        paper_result = self._detect_paper(bgr)
        if paper_result is None:
            # self.get_logger().debug(
            #    "No paper/board detected in frame.",
            #    throttle_duration_sec=3.0,
            #)
            return
        src, cx_px, cy_px = paper_result
        # self.get_logger().debug(f"Paper detected at ({cx_px},{cy_px}).")

        # Step 2 — isolate ink strokes
        tensor = self._preprocess_letter(src)
        if tensor is None:
            self.get_logger().debug(
                "Ink preprocessing returned None (no contours in paper crop).",
                throttle_duration_sec=3.0,
            )
            return

        # Step 3 — ONNX inference
        label, confidence = self._infer(tensor)
        if label is None or confidence < self._min_conf:
            self.get_logger().info(
                f"[GREEK] Low confidence or no label: {label} conf={confidence:.2f} "
                f"(threshold={self._min_conf:.2f})",
                throttle_duration_sec=3.0,
            )
            return

        if self._on_cooldown(label):
            return

        # Step 4 — estimate obstacle odom position using lidar range
        fx = (bgr.shape[1] / 2.0) / math.tan(self._cam_hfov / 2.0)
        # bearing: positive = left in image = robot left (+Y), negative = right (-Y)
        bearing_rad = math.atan2((bgr.shape[1] / 2.0) - cx_px, fx)
        beam_idx = int(round(
            (bearing_rad - self._latest_scan.angle_min)
            / self._latest_scan.angle_increment
        ))
        obs = lidar_to_odom(
            self._latest_scan, cx_px, bgr.shape[1],
            self._cam_hfov,
            self._robot_x, self._robot_y, self._robot_yaw,
            half_depth_m=0.0,   # barrel depth compensation negligible
        )
        if obs is None:
            self.get_logger().warn(
                f"[GREEK] {label}: lidar bearing {math.degrees(bearing_rad):.1f}° "
                f"(beam {beam_idx}/{len(self._latest_scan.ranges)}) no valid range, skipping",
                throttle_duration_sec=5.0,
            )
            return
        obs_x, obs_y, range_m = obs

        self.get_logger().info(
            f"[GREEK] {label} | conf={confidence:.2f} | "
            f"cx_px={cx_px}  bearing={math.degrees(bearing_rad):.1f}°  beam={beam_idx}  "
            f"range={range_m:.3f}m | "
            f"robot=({self._robot_x:.2f},{self._robot_y:.2f})  "
            f"yaw={math.degrees(self._robot_yaw):.1f}° | "
            f"obs=({obs_x:.3f},{obs_y:.3f})"
        )

        # Step 5 — save photo only at a new location: deduplicate by observation coordinates to avoid re-saving when circling the barrel
        img_path = ""
        if self._is_new_location(label, obs_x, obs_y):
            ink_vis  = (tensor[0, 0] * 255).astype(np.uint8)
            ink_vis  = cv2.resize(ink_vis, (128, 128), interpolation=cv2.INTER_NEAREST)
            ts       = datetime.now().strftime("%Y%m%d_%H%M%S_%f")[:-3]
            ink_path = os.path.join(self._photo_dir, f"ink_{label}_{ts}.jpg")
            cv2.imwrite(ink_path, ink_vis)
            img_path = self._save_photo(bgr, label, obs_x, obs_y)
            self._saved_positions.setdefault(label, []).append((obs_x, obs_y))

        self._publish(label, confidence, img_path, cx_px, cy_px, obs_x, obs_y, range_m)

    @staticmethod
    def _detect_paper(bgr: np.ndarray) -> Optional[tuple[np.ndarray, int, int]]:
        """
        Find the white/grey board on the bucket and return a
        perspective-corrected crop plus its centre pixel.
        Returns None if not found.
        """
        h, w = bgr.shape[:2]
        hsv  = cv2.cvtColor(bgr, cv2.COLOR_BGR2HSV)
        # Simulated white paper value≈220-240; grey background wall/floor value≈100-160.
        # Threshold 190 cleanly separates paper from background, avoiding a merged blob that would be rejected.
        thresh = cv2.inRange(hsv, (0, 0, 190), (180, 80, 255))

        kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (5, 5))
        thresh = cv2.morphologyEx(thresh, cv2.MORPH_CLOSE, kernel, iterations=2)
        thresh = cv2.morphologyEx(thresh, cv2.MORPH_OPEN,  kernel, iterations=1)

        contours, _ = cv2.findContours(
            thresh, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        best_cnt  = None
        best_area = 0
        for cnt in contours:
            area = cv2.contourArea(cnt)
            # area lower bound 0.5% (allows detection at 4-5 m), upper bound 60% excludes background
            if area < 0.005 * h * w:
                continue
            if area > 0.60 * h * w:
                continue
            _bx, _by, _bw, _bh = cv2.boundingRect(cnt)
            if _by <= 3:                       # touching top edge → sky/background
                continue
            if _by + _bh >= h - 10:            # touching bottom edge → ground markings
                continue
            aspect = _bw / max(_bh, 1)
            if aspect > 5.0 or aspect < 0.2:   # extreme elongation → not a board
                continue
            peri   = cv2.arcLength(cnt, True)
            approx = cv2.approxPolyDP(cnt, 0.04 * peri, True)
            n = len(approx)
            if 4 <= n <= 6 and area > best_area:
                # allow 4-6 corner approximation (simulated cube at oblique angle may give 5-6 corners)
                best_area = area
                # force convex hull and fit as 4-corner rectangle
                hull = cv2.convexHull(cnt)
                hull_approx = cv2.approxPolyDP(
                    hull, 0.04 * cv2.arcLength(hull, True), True)
                best_cnt = hull_approx if len(hull_approx) == 4 else approx[:4]
            elif best_cnt is None and area > best_area:
                hull = cv2.convexHull(cnt)
                hull_approx = cv2.approxPolyDP(
                    hull, 0.04 * cv2.arcLength(hull, True), True)
                if len(hull_approx) >= 4:
                    best_area = area
                    best_cnt  = hull_approx[:4]

        if best_cnt is None:
            return None

        pts     = best_cnt.reshape(4, 2).astype(np.float32)
        cx_orig = int(pts[:, 0].mean())
        cy_orig = int(pts[:, 1].mean())
        s    = pts.sum(axis=1)
        d    = np.diff(pts, axis=1).ravel()
        rect = np.float32([
            pts[np.argmin(s)],
            pts[np.argmin(d)],
            pts[np.argmax(s)],
            pts[np.argmax(d)],
        ])
        ow = int(max(np.linalg.norm(rect[0] - rect[1]),
                     np.linalg.norm(rect[2] - rect[3])))
        oh = int(max(np.linalg.norm(rect[0] - rect[3]),
                     np.linalg.norm(rect[1] - rect[2])))
        ow, oh = max(ow, 64), max(oh, 64)
        dst = np.float32([[0, 0], [ow, 0], [ow, oh], [0, oh]])
        M   = cv2.getPerspectiveTransform(rect, dst)
        return cv2.warpPerspective(bgr, M, (ow, oh)), cx_orig, cy_orig

    @staticmethod
    def _preprocess_letter(bgr: np.ndarray) -> Optional[np.ndarray]:
        """
        MNIST-style ink isolation:
          greyscale → gaussian blur → adaptive threshold (inverted) →
          largest contour crop → resize 64×64 → normalise [0,1]

        Returns [1, 1, 64, 64] float32 tensor or None if no ink found.
        Must match train_model.py preprocess_to_tensor() exactly.
        """
        grey   = cv2.cvtColor(bgr, cv2.COLOR_BGR2GRAY)
        grey   = cv2.GaussianBlur(grey, (5, 5), 0)
        # blockSize=15, C=6 — simulated low-contrast ink strokes need a more permissive adaptive threshold
        thresh = cv2.adaptiveThreshold(
            grey, 255,
            cv2.ADAPTIVE_THRESH_GAUSSIAN_C,
            cv2.THRESH_BINARY_INV,
            blockSize=15, C=6,
        )
        contours, _ = cv2.findContours(
            thresh, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        if not contours:
            return None
        largest = max(contours, key=cv2.contourArea)
        x, y, w, h = cv2.boundingRect(largest)
        crop = thresh[y:y + h, x:x + w]
        if crop.size == 0:
            return None
        resized = cv2.resize(crop, (IMG_SIZE, IMG_SIZE),
                             interpolation=cv2.INTER_AREA)
        tensor  = resized.astype(np.float32) / 255.0
        return tensor[np.newaxis, np.newaxis, :, :]   # [1, 1, 64, 64]

    def _infer(self, tensor: np.ndarray) -> tuple[Optional[str], float]:
        """Run ONNX inference. Returns (label, confidence)."""
        try:
            input_name = self._model.get_inputs()[0].name
            logits     = self._model.run(None, {input_name: tensor})[0][0]
            probs      = np.exp(logits) / np.exp(logits).sum()
            idx        = int(np.argmax(probs))
            if idx >= len(_LABELS):
                return None, 0.0
            return _LABELS[idx], float(probs[idx])
        except Exception as exc:
            self.get_logger().warn(
                f"Inference error: {exc}", throttle_duration_sec=5.0)
            return None, 0.0

    # ════════════════════════════════════════════════════════════════════
    # Helpers
    # ════════════════════════════════════════════════════════════════════

    def _is_new_location(self, label: str, obs_x: float, obs_y: float) -> bool:
        """Return True if the estimated marker position is more than new_object_dist_m from all previously recorded positions.
        Uses observation coordinates rather than robot position to avoid re-reporting the same barrel when orbiting it."""
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
        obs_x: float = 0.0,
        obs_y: float = 0.0,
    ) -> str:
        annotated = bgr.copy()
        cv2.putText(
            annotated,
            f"{label} @ ({obs_x:.2f},{obs_y:.2f})",
            (10, 35),
            cv2.FONT_HERSHEY_SIMPLEX, 1.0, (0, 165, 255), 2,
        )
        ts   = datetime.now().strftime("%Y%m%d_%H%M%S_%f")[:-3]
        path = os.path.join(self._photo_dir, f"greek_{label}_{ts}.jpg")
        cv2.imwrite(path, annotated, [cv2.IMWRITE_JPEG_QUALITY, self._jpeg_q])
        return path

    def _publish(self, label: str, confidence: float, img_path: str,
                 cx_px: int = 0, cy_px: int = 0,
                 obs_x: float = 0.0, obs_y: float = 0.0,
                 range_m: float = float("nan")) -> None:
        payload = (
            f"type=greek "
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
    node = GreekDetectorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
