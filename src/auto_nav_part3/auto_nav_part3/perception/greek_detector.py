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
  <depth_topic>           sensor_msgs/Image   — depth image (float32 m)

Publishes:
  /part3/perception/marker_event  std_msgs/String
    format: "type=greek label=<name> x=<f> y=<f>
             confidence=<f> image=<path> range_m=<f>"

Parameters
----------
  greek_model_path      — absolute path to greek_letters.onnx
                          Leave empty to disable (node still runs, just no detections)
  depth_topic           — depth image topic (default: /camera/depth_image for sim;
                          use /oak/stereo/depth for real OAK-D hardware)
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
from sensor_msgs.msg import Image
from std_msgs.msg import String

from .perception_utils import depth_to_odom

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
        self.declare_parameter("depth_topic",          "/camera/depth_image")
        self.declare_parameter("photo_dir",            "artifacts/photos")
        self.declare_parameter("jpeg_quality",         90)
        self.declare_parameter("detection_cooldown_s", 5.0)
        self.declare_parameter("min_confidence",       0.5)
        self.declare_parameter("image_width",          640)
        self.declare_parameter("image_height",         480)
        self.declare_parameter("new_object_dist_m",    1.5)

        gp = self.get_parameter
        self._model_path   = gp("greek_model_path").get_parameter_value().string_value
        self._depth_topic  = gp("depth_topic").get_parameter_value().string_value
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
        self._depth_frame        = None      # latest depth frame, float32 m
        self._cam_hfov: float    = 1.089     # OAK-D / sim rgbd_camera HFOV ≈ 62°
        self._last_detected: dict[str, float] = {}
        self._last_debug_save: float = 0.0
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
            Image, self._depth_topic, self._on_depth, 10)

        # ── publishers ────────────────────────────────────────────────────
        self._pub = self.create_publisher(
            String, "/part3/perception/marker_event", 10)

        self.get_logger().set_level(rclpy.logging.LoggingSeverity.DEBUG)
        self.get_logger().info(
            f"GreekDetector ready | model='{self._model_path}' "
            f"depth='{self._depth_topic}' min_conf={self._min_conf} "
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

    def _on_depth(self, msg: Image) -> None:
        try:
            raw = self._bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
            if raw.dtype == np.float32:
                self._depth_frame = raw.copy()
            else:
                self._depth_frame = raw.astype(np.float32) * 0.001
        except Exception:
            pass

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

        # Step 1 — 必须先找到白色/灰色板块，才能裁切出字母区域
        paper_result = self._detect_paper(bgr)
        if paper_result is None:
            self.get_logger().debug(
                "No paper/board detected in frame.",
                throttle_duration_sec=3.0,
            )
            # 每 10s 保存一帧相机原图，供离线分析 HSV 分布
            now = time.monotonic()
            if now - self._last_debug_save > 10.0:
                self._last_debug_save = now
                ts = datetime.now().strftime("%Y%m%d_%H%M%S")
                dbg_path = os.path.join(self._photo_dir, f"debug_cam_{ts}.jpg")
                cv2.imwrite(dbg_path, bgr)
                self.get_logger().debug(f"Debug frame saved: {dbg_path}")
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

        # Step 4 — 用深度图估算障碍物 odom 坐标
        obs = depth_to_odom(
            self._depth_frame, cx_px, cy_px,
            self._robot_x, self._robot_y, self._robot_yaw,
            self._cam_hfov,
        )
        obs_x, obs_y, range_m = obs if obs else (self._robot_x, self._robot_y, float("nan"))

        # Step 5 — 新空间位置才存图：防止同一物体反复写入
        img_path = ""
        if self._is_new_location(label):
            ink_vis  = (tensor[0, 0] * 255).astype(np.uint8)
            ink_vis  = cv2.resize(ink_vis, (128, 128), interpolation=cv2.INTER_NEAREST)
            ts       = datetime.now().strftime("%Y%m%d_%H%M%S_%f")[:-3]
            ink_path = os.path.join(self._photo_dir, f"ink_{label}_{ts}.jpg")
            cv2.imwrite(ink_path, ink_vis)
            img_path = self._save_photo(bgr, label, obs_x, obs_y)
            self._saved_positions.setdefault(label, []).append(
                (self._robot_x, self._robot_y))

        self._publish(label, confidence, img_path, cx_px, cy_px, obs_x, obs_y, range_m)
        self.get_logger().info(
            f"[GREEK] {label} | conf={confidence:.2f} | "
            f"obs=({obs_x:.2f},{obs_y:.2f}) range={range_m:.2f}m"
        )

    @staticmethod
    def _detect_paper(bgr: np.ndarray) -> Optional[tuple[np.ndarray, int, int]]:
        """
        Find the white/grey board on the bucket and return a
        perspective-corrected crop plus its centre pixel.
        Returns None if not found.
        """
        h, w = bgr.shape[:2]
        hsv  = cv2.cvtColor(bgr, cv2.COLOR_BGR2HSV)
        # 仿真白纸 value≈220-240，灰色背景墙/地面 value≈100-160。
        # 阈值 190 可干净分离白纸与背景，避免背景与纸张 merge 成巨大 blob 被误拒。
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
            # 面积下限 0.5%（允许方块在 4-5m 外被检测），上限 60% 排除背景
            if area < 0.005 * h * w:
                continue
            if area > 0.60 * h * w:
                continue
            _bx, _by, _bw, _bh = cv2.boundingRect(cnt)
            if _by <= 3:                       # 顶部贴边 → 天空/背景
                continue
            if _by + _bh >= h - 10:            # 底部贴边 → 地面标线
                continue
            aspect = _bw / max(_bh, 1)
            if aspect > 5.0 or aspect < 0.2:   # 极细长条 → 非板块
                continue
            peri   = cv2.arcLength(cnt, True)
            approx = cv2.approxPolyDP(cnt, 0.04 * peri, True)
            n = len(approx)
            if 4 <= n <= 6 and area > best_area:
                # 允许 4-6 角近似（仿真方块斜视时可能 5-6 角）
                best_area = area
                # 强制取凸包并拟合为 4 角矩形
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
        # blockSize=15, C=6 — 仿真低对比度笔迹需要更宽松的自适应阈值
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

    def _is_new_location(self, label: str) -> bool:
        """当前机器人位置与所有已存图位置的最小距离 > new_object_dist_m 时返回 True。"""
        positions = self._saved_positions.get(label, [])
        if not positions:
            return True
        rx, ry = self._robot_x, self._robot_y
        min_dist = min(math.hypot(rx - px, ry - py) for px, py in positions)
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
