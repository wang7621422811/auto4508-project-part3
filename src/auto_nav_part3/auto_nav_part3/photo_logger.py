"""
photo_logger.py — Subscribes to /part3/perception/marker_event and maintains
                  a persistent manifest of all detections for report evidence.
 
Responsibilities:
  - Parse every marker event string from perception_node
  - Write one row per detection to artifacts/markers/manifest.csv
  - Copy the saved photo into artifacts/markers/photos/ for easy retrieval
  - Publish a summary count to /part3/perception/logger_status
 
All paths are ROS2 parameters — change them at launch or via command line
without touching this file.
 
Contract (TOPICS.md)
---------------------
  Subscribes:
    /part3/perception/marker_event  std_msgs/String
 
  Publishes:
    /part3/perception/logger_status std_msgs/String
      format: "detections=<N> greek=<N> colour=<N> manifest=<path>"
 
Parameters
----------
  artifact_dir   — root directory for all saved artifacts
                   default: "artifacts/markers"
                   On the robot, set to an absolute path, e.g.:
                   /home/user/ros2_ws/artifacts/markers
 
  manifest_name  — CSV filename inside artifact_dir
                   default: "manifest.csv"
 
  copy_photos    — if true, copy photos into artifact_dir/photos/
                   default: true
 
Usage
-----
  # Default paths (relative to working directory)
  ros2 run auto_nav_part3 photo_logger
 
  # Custom path for robot PC
  ros2 run auto_nav_part3 photo_logger --ros-args \
    -p artifact_dir:=/home/pioneer/artifacts/markers
 
  # Or set in launch file:
  Node(
      package='auto_nav_part3',
      executable='photo_logger',
      parameters=[{'artifact_dir': '/home/pioneer/artifacts/markers'}],
  )
"""
 
from __future__ import annotations
 
import csv
import os
import shutil
from datetime import datetime
 
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
 
 
# CSV column headers — do not change order (breaks existing CSVs)
_CSV_HEADERS = [
    "timestamp",
    "marker_type",
    "label",
    "x",
    "y",
    "confidence",
    "range_m",
    "image_path",
    "photo_copy_path",
]
 
 
class PhotoLoggerNode(Node):
    """Log marker detections to CSV and copy photos into artifact directory."""
 
    def __init__(self) -> None:
        super().__init__("photo_logger")
 
        # ── parameters (all paths configurable) ──────────────────────────
        self.declare_parameter(
            "artifact_dir",
            "artifacts/markers",
        )
        self.declare_parameter(
            "manifest_name",
            "manifest.csv",
        )
        self.declare_parameter(
            "copy_photos",
            True,
        )
 
        gp = self.get_parameter
        self._artifact_dir  = gp("artifact_dir").get_parameter_value().string_value
        self._manifest_name = gp("manifest_name").get_parameter_value().string_value
        self._copy_photos   = gp("copy_photos").get_parameter_value().bool_value
 
        # Derived paths
        self._photos_dir    = os.path.join(self._artifact_dir, "photos")
        self._manifest_path = os.path.join(self._artifact_dir, self._manifest_name)
 
        # Create directories
        os.makedirs(self._artifact_dir, exist_ok=True)
        if self._copy_photos:
            os.makedirs(self._photos_dir, exist_ok=True)
 
        # Open CSV — append mode so restarts don't overwrite existing data
        self._csv_file   = open(self._manifest_path, "a", newline="")
        self._csv_writer = csv.writer(self._csv_file)
 
        # Write headers only if file is new/empty
        if os.path.getsize(self._manifest_path) == 0:
            self._csv_writer.writerow(_CSV_HEADERS)
            self._csv_file.flush()
 
        # Detection counters
        self._total   = 0
        self._greek   = 0
        self._colour  = 0
 
        # ── subscribers / publishers ──────────────────────────────────────
        self.create_subscription(
            String,
            "/part3/perception/marker_event",
            self._on_marker_event,
            10,
        )
 
        self._pub_status = self.create_publisher(
            String,
            "/part3/perception/logger_status",
            10,
        )
 
        # Status publish timer — every 5 seconds
        self.create_timer(5.0, self._publish_status)
 
        self.get_logger().info(
            f"PhotoLogger ready — manifest={self._manifest_path}"
        )
 
    # ════════════════════════════════════════════════════════════════════
    # Event handler
    # ════════════════════════════════════════════════════════════════════
 
    def _on_marker_event(self, msg: String) -> None:
        """
        Parse the contracted marker event string and write to CSV.
 
        Expected format (from perception_node.py):
          type=greek label=alpha x=2.1 y=-0.4 confidence=0.82
          image=/path/to/photo.jpg range_m=3.450
        """
        data = msg.data.strip()
        fields = self._parse_event(data)
 
        if fields is None:
            self.get_logger().warn(
                f"Could not parse marker event: '{data}'",
                throttle_duration_sec=5.0,
            )
            return
 
        # Copy photo if requested
        photo_copy = ""
        if self._copy_photos and fields["image_path"]:
            photo_copy = self._copy_photo(
                fields["image_path"],
                fields["marker_type"],
                fields["label"],
            )
 
        # Write CSV row
        timestamp = datetime.now().strftime("%Y-%m-%d %H:%M:%S.%f")[:-3]
        self._csv_writer.writerow([
            timestamp,
            fields["marker_type"],
            fields["label"],
            fields["x"],
            fields["y"],
            fields["confidence"],
            fields["range_m"],
            fields["image_path"],
            photo_copy,
        ])
        self._csv_file.flush()
 
        # Update counters
        self._total += 1
        if fields["marker_type"] == "greek":
            self._greek += 1
        else:
            self._colour += 1
 
        self.get_logger().info(
            f"[LOG] {fields['marker_type']} {fields['label']} "
            f"conf={fields['confidence']} total={self._total}"
        )
 
    # ════════════════════════════════════════════════════════════════════
    # Helpers
    # ════════════════════════════════════════════════════════════════════
 
    @staticmethod
    def _parse_event(data: str) -> dict | None:
        """
        Parse 'key=value key=value ...' format into a dict.
        Returns None if required fields are missing.
        """
        fields: dict[str, str] = {}
        for token in data.split():
            if "=" in token:
                key, _, val = token.partition("=")
                fields[key.strip()] = val.strip()
 
        required = {"type", "label", "x", "y", "confidence"}
        if not required.issubset(fields.keys()):
            return None
 
        return {
            "marker_type": fields.get("type",       "unknown"),
            "label":       fields.get("label",      "unknown"),
            "x":           fields.get("x",          "0.0"),
            "y":           fields.get("y",          "0.0"),
            "confidence":  fields.get("confidence", "0.0"),
            "range_m":     fields.get("range_m",    "nan"),
            "image_path":  fields.get("image",      ""),
        }
 
    def _copy_photo(
        self, src_path: str, marker_type: str, label: str
    ) -> str:
        """
        Copy a photo into the managed photos directory.
        Returns the destination path, or empty string on failure.
 
        Filename format: <type>_<label>_<timestamp>.jpg
        This makes it easy to find photos by label in the report.
        """
        if not src_path or not os.path.exists(src_path):
            return ""
 
        ts  = datetime.now().strftime("%Y%m%d_%H%M%S_%f")[:-3]
        ext = os.path.splitext(src_path)[1] or ".jpg"
        dst = os.path.join(
            self._photos_dir,
            f"{marker_type}_{label}_{ts}{ext}",
        )
        try:
            shutil.copy2(src_path, dst)
            return dst
        except Exception as exc:  # noqa: BLE001
            self.get_logger().warn(f"Photo copy failed: {exc}")
            return ""
 
    def _publish_status(self) -> None:
        """Publish detection summary every 5 seconds."""
        msg = String()
        msg.data = (
            f"detections={self._total} "
            f"greek={self._greek} "
            f"colour={self._colour} "
            f"manifest={self._manifest_path}"
        )
        self._pub_status.publish(msg)
 
    def destroy_node(self) -> None:
        """Flush and close CSV on shutdown."""
        try:
            self._csv_file.flush()
            self._csv_file.close()
            self.get_logger().info(
                f"Manifest closed: {self._manifest_path} "
                f"({self._total} detections)"
            )
        except Exception:  # noqa: BLE001
            pass
        super().destroy_node()
 
 
# ---------------------------------------------------------------------------
def main(args=None) -> None:
    rclpy.init(args=args)
    node = PhotoLoggerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
 
 
if __name__ == "__main__":
    main()
