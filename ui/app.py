from flask import Flask, render_template, jsonify, request
import threading
import time
import math
import base64
import os
import json

import cv2
import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy

from std_msgs.msg import String
from std_srvs.srv import Trigger
from nav_msgs.msg import Odometry, OccupancyGrid, Path
from sensor_msgs.msg import LaserScan, Image
from tf2_ros import Buffer, TransformListener, TransformException
from geometry_msgs.msg import PoseArray, Pose, Twist


app = Flask(__name__)


# =========================
# Topic / service configuration
# =========================

MAP_TOPIC = "/map"
ODOM_TOPIC = "/odometry/filtered"
SCAN_TOPIC = "/scan"
USE_TF_POSE = True
ROBOT_MAP_FRAME = "map"
ROBOT_BASE_FRAME = "base_link"
CAMERA_TOPIC = "/camera/image"
CMD_VEL_TOPIC = "/cmd_vel"

SYSTEM_STATE_TOPIC = "/part3/system/state"
MAP_STATUS_TOPIC = "/part3/mapping/map_status"
WAYPOINT_PLAN_TEXT_TOPIC = "/part3/waypoint/plan"
ESTOP_EVENT_TOPIC = "/part3/safety/estop_event"
GREEK_MARKERS_TOPIC = "/part3/perception/greek_markers"

NAV_PATH_TOPIC = "/plan"

START_MAPPING_SERVICE = "/part3/mapping/start"

# New waypoint JSON function
WAYPOINTS_DIR = os.environ.get(
    "WAYPOINTS_DIR",
    os.path.expanduser("~/auto4508-project-part3/artifacts/waypoints")
)

# Fallback path for container/project layout
if not os.path.isdir(WAYPOINTS_DIR):
    WAYPOINTS_DIR = "/root/workspace/auto_nav_part3_team18/auto4508-project-part3/artifacts/waypoints"

SELECTED_WAYPOINTS_TOPIC = "/part3/perception/greek_markers"
START_WAYPOINT_SERVICE = "/part3/waypoint/start"


# =========================
# Shared data
# =========================

robot_data = {
    "connection": "Waiting for ROS2 data",

    "system_state": "IDLE",
    "map_status": "Waiting for mapping status",
    "waypoint_plan": "No waypoint plan received",
    "command_status": "No command sent",
    "last_estop_event": "No emergency stop event",

    "x": 0.0,
    "y": 0.0,
    "yaw": 0.0,

    "scan": {
        "ranges": [],
        "angle_min": 0.0,
        "angle_increment": 0.0,
        "range_max": 0.0
    },

    "markers": [],
    "greek_markers": [],

    "path": [],

    "last_update_time": 0.0
}


map_data = {
    "width": 0,
    "height": 0,
    "resolution": 0.0,
    "origin_x": 0.0,
    "origin_y": 0.0,
    "data": []
}


camera_data = {
    "available": False,
    "encoding": "",
    "width": 0,
    "height": 0,
    "frame": ""
}


web_ui_node = None
last_manual_command_time = 0.0


# =========================
# Helper functions
# =========================

def now_sec():
    return round(time.time(), 3)


def quaternion_to_yaw_deg(q):
    """
    Convert geometry_msgs Quaternion to yaw angle in degrees.
    """
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    yaw_rad = math.atan2(siny_cosp, cosy_cosp)
    return math.degrees(yaw_rad)


def pose_array_to_list(msg):
    """
    Convert geometry_msgs/PoseArray to JSON-friendly list.
    PoseArray only contains positions/orientations, so labels are generated in the UI.
    """
    result = []

    for i, pose in enumerate(msg.poses):
        result.append({
            "id": i,
            "x": round(float(pose.position.x), 3),
            "y": round(float(pose.position.y), 3),
            "z": round(float(pose.position.z), 3)
        })

    return result


def path_to_list(msg):
    """
    Convert nav_msgs/Path to list of x/y points.
    """
    result = []

    for pose_stamped in msg.poses:
        result.append({
            "x": round(float(pose_stamped.pose.position.x), 3),
            "y": round(float(pose_stamped.pose.position.y), 3)
        })

    return result


# =========================
# Waypoint JSON helpers
# =========================

def safe_waypoint_file_path(file_name):
    """
    Prevent path traversal. Only allow JSON files inside WAYPOINTS_DIR.
    """
    base_dir = os.path.abspath(WAYPOINTS_DIR)
    target_path = os.path.abspath(os.path.join(base_dir, file_name))

    if not target_path.startswith(base_dir):
        raise ValueError("Invalid waypoint file path")

    if not target_path.endswith(".json"):
        raise ValueError("Only JSON waypoint files are allowed")

    return target_path


def infer_waypoint_category(file_name, waypoints, raw_data=None):
    """
    Classify waypoint JSON into:
    - greek
    - color
    - other

    It checks filename and waypoint fields such as name/type/color/colour/label.
    """
    text_parts = [file_name.lower()]

    if isinstance(raw_data, dict):
        for key, value in raw_data.items():
            if isinstance(key, str):
                text_parts.append(key.lower())
            if isinstance(value, str):
                text_parts.append(value.lower())

    for wp in waypoints:
        for key in ["name", "id", "type", "label", "color", "colour", "class"]:
            value = wp.get(key)
            if value is not None:
                text_parts.append(str(value).lower())

    text = " ".join(text_parts)

    greek_keywords = [
        "greek", "alpha", "beta", "gamma", "delta", "epsilon",
        "zeta", "eta", "theta", "lambda", "mu", "omega",
        "phi", "psi", "sigma", "tau", "kappa",
        "α", "β", "γ", "δ", "ε", "θ", "λ", "ω", "φ", "ψ", "σ"
    ]

    color_keywords = [
        "color", "colour", "red", "yellow", "blue", "green",
        "orange", "purple", "black", "white", "obstacle"
    ]

    if any(k in text for k in greek_keywords):
        return "greek"

    if any(k in text for k in color_keywords):
        return "color"

    return "other"


def extract_waypoint_list(raw_data):
    """
    Convert different JSON structures into:
    [
        {"name": "A", "x": 1.2, "y": 3.4, "z": 0.0, ...},
        ...
    ]

    Supported examples:
    1. [{"x": 1.0, "y": 2.0}]
    2. {"waypoints": [{"x": 1.0, "y": 2.0}]}
    3. {"points": [{"x": 1.0, "y": 2.0}]}
    4. {"poses": [{"position": {"x": 1.0, "y": 2.0}}]}
    5. {"A": {"x": 1.0, "y": 2.0}, "B": {"x": 3.0, "y": 4.0}}
    """
    if isinstance(raw_data, list):
        raw_points = raw_data

    elif isinstance(raw_data, dict):
        if "waypoints" in raw_data:
            raw_points = raw_data["waypoints"]
        elif "points" in raw_data:
            raw_points = raw_data["points"]
        elif "poses" in raw_data:
            raw_points = raw_data["poses"]
        elif "goals" in raw_data:
            raw_points = raw_data["goals"]
        elif "markers" in raw_data:
            raw_points = raw_data["markers"]
        else:
            raw_points = []
            for key, value in raw_data.items():
                if isinstance(value, dict):
                    item = dict(value)
                    item.setdefault("name", key)
                    raw_points.append(item)
    else:
        raw_points = []

    waypoints = []

    for i, item in enumerate(raw_points):
        if not isinstance(item, dict):
            continue

        name = item.get("name", item.get("id", item.get("label", f"WP{i}")))

        if "position" in item and isinstance(item["position"], dict):
            pos = item["position"]
            x = pos.get("x", 0.0)
            y = pos.get("y", 0.0)
            z = pos.get("z", 0.0)
        elif "pose" in item and isinstance(item["pose"], dict):
            pose = item["pose"]
            if "position" in pose and isinstance(pose["position"], dict):
                pos = pose["position"]
                x = pos.get("x", 0.0)
                y = pos.get("y", 0.0)
                z = pos.get("z", 0.0)
            else:
                x = pose.get("x", 0.0)
                y = pose.get("y", 0.0)
                z = pose.get("z", 0.0)
        else:
            x = item.get("x", item.get("px", 0.0))
            y = item.get("y", item.get("py", 0.0))
            z = item.get("z", 0.0)

        try:
            wp = dict(item)
            wp["name"] = str(name)
            wp["x"] = float(x)
            wp["y"] = float(y)
            wp["z"] = float(z)
            waypoints.append(wp)
        except Exception:
            continue

    return waypoints


def load_waypoint_file(file_name):
    file_path = safe_waypoint_file_path(file_name)

    with open(file_path, "r", encoding="utf-8") as f:
        raw_data = json.load(f)

    waypoints = extract_waypoint_list(raw_data)
    category = infer_waypoint_category(file_name, waypoints, raw_data)

    return waypoints, category


def list_waypoint_json_files():
    """
    Return categorized waypoint files.
    """
    result = {
        "greek": [],
        "color": [],
        "other": []
    }

    if not os.path.isdir(WAYPOINTS_DIR):
        return result

    for file_name in sorted(os.listdir(WAYPOINTS_DIR)):
        if not file_name.endswith(".json"):
            continue

        try:
            waypoints, category = load_waypoint_file(file_name)

            entry = {
                "file": file_name,
                "category": category,
                "count": len(waypoints),
                "waypoints": waypoints
            }

            result.setdefault(category, []).append(entry)

        except Exception as e:
            result["other"].append({
                "file": file_name,
                "category": "other",
                "count": 0,
                "error": str(e),
                "waypoints": []
            })

    return result


# =========================
# ROS2 Node
# =========================

class Part3WebUINode(Node):
    def __init__(self):
        super().__init__("part3_web_ui_node")

        map_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )

        normal_qos = 10
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.pose_timer = self.create_timer(
            0.1,
            self.update_robot_pose_from_tf
        )
        # Subscribers
        self.create_subscription(
            OccupancyGrid,
            MAP_TOPIC,
            self.map_callback,
            map_qos
        )

        self.create_subscription(
            Odometry,
            ODOM_TOPIC,
            self.odom_callback,
            normal_qos
        )

        self.create_subscription(
            LaserScan,
            SCAN_TOPIC,
            self.scan_callback,
            normal_qos
        )

        self.create_subscription(
            Image,
            CAMERA_TOPIC,
            self.camera_callback,
            normal_qos
        )

        self.create_subscription(
            String,
            SYSTEM_STATE_TOPIC,
            self.system_state_callback,
            normal_qos
        )

        self.create_subscription(
            String,
            MAP_STATUS_TOPIC,
            self.map_status_callback,
            normal_qos
        )

        self.create_subscription(
            String,
            WAYPOINT_PLAN_TEXT_TOPIC,
            self.waypoint_plan_text_callback,
            normal_qos
        )

        self.create_subscription(
            String,
            ESTOP_EVENT_TOPIC,
            self.estop_event_callback,
            normal_qos
        )

        self.create_subscription(
            PoseArray,
            GREEK_MARKERS_TOPIC,
            self.greek_markers_callback,
            normal_qos
        )

        self.create_subscription(
            Path,
            NAV_PATH_TOPIC,
            self.path_callback,
            normal_qos
        )

        # Publishers
        self.cmd_vel_pub = self.create_publisher(
            Twist,
            CMD_VEL_TOPIC,
            normal_qos
        )

        self.selected_waypoints_pub = self.create_publisher(
            PoseArray,
            SELECTED_WAYPOINTS_TOPIC,
            normal_qos
        )

        # Service clients
        self.start_mapping_client = self.create_client(
            Trigger,
            START_MAPPING_SERVICE
        )

        self.start_waypoint_client = self.create_client(
            Trigger,
            START_WAYPOINT_SERVICE
        )

        # Dead-man timeout for keyboard/manual control
        self.manual_timeout_sec = 0.6
        self.safety_timer = self.create_timer(
            0.1,
            self.manual_timeout_check
        )

        self.get_logger().info("Part 3 Web UI node started.")
        self.get_logger().info(f"Subscribing map: {MAP_TOPIC}")
        self.get_logger().info(f"Subscribing odom: {ODOM_TOPIC}")
        self.get_logger().info(f"Subscribing scan: {SCAN_TOPIC}")
        self.get_logger().info(f"Subscribing camera: {CAMERA_TOPIC}")
        self.get_logger().info(f"Publishing manual/keyboard control: {CMD_VEL_TOPIC}")
        self.get_logger().info(f"Start mapping service: {START_MAPPING_SERVICE}")
        self.get_logger().info(f"Waypoint JSON directory: {WAYPOINTS_DIR}")
        self.get_logger().info(f"Publishing selected waypoints: {SELECTED_WAYPOINTS_TOPIC}")
        self.get_logger().info(f"Start waypoint service: {START_WAYPOINT_SERVICE}")

    def update_connection(self):
        robot_data["connection"] = "ROS2 connected"
        robot_data["last_update_time"] = now_sec()

    def map_callback(self, msg):
        global map_data

        map_data = {
            "width": int(msg.info.width),
            "height": int(msg.info.height),
            "resolution": float(msg.info.resolution),
            "origin_x": float(msg.info.origin.position.x),
            "origin_y": float(msg.info.origin.position.y),
            "data": list(msg.data)
        }

        self.update_connection()

    def update_robot_pose_from_tf(self):
        """
        Use TF map -> base_link for UI robot pose.
        This keeps the robot icon in the same frame as /map and /plan.
        """
        if not USE_TF_POSE:
            return

        try:
            transform = self.tf_buffer.lookup_transform(
                ROBOT_MAP_FRAME,
                ROBOT_BASE_FRAME,
                rclpy.time.Time()
            )

            t = transform.transform.translation
            q = transform.transform.rotation

            robot_data["x"] = round(float(t.x), 3)
            robot_data["y"] = round(float(t.y), 3)
            robot_data["yaw"] = round(float(quaternion_to_yaw_deg(q)), 2)

            self.update_connection()

        except TransformException:
            return
    
    def odom_callback(self, msg):
        # If TF pose is enabled, do not draw odom pose directly on /map.
        # /odometry/filtered is often in odom frame, while /map is in map frame.
        if USE_TF_POSE:
            return

        robot_data["x"] = round(float(msg.pose.pose.position.x), 3)
        robot_data["y"] = round(float(msg.pose.pose.position.y), 3)
        robot_data["yaw"] = round(float(quaternion_to_yaw_deg(msg.pose.pose.orientation)), 2)

        self.update_connection()

    def scan_callback(self, msg):
        # Downsample scan data to reduce browser load.
        step = 10
        ranges = []

        for r in msg.ranges[::step]:
            if math.isinf(r) or math.isnan(r):
                ranges.append(None)
            else:
                ranges.append(round(float(r), 3))

        robot_data["scan"] = {
            "ranges": ranges,
            "angle_min": round(float(msg.angle_min), 5),
            "angle_increment": round(float(msg.angle_increment * step), 5),
            "range_max": round(float(msg.range_max), 3)
        }

        self.update_connection()

    def camera_callback(self, msg):
        global camera_data

        try:
            height = int(msg.height)
            width = int(msg.width)
            encoding = msg.encoding

            image_np = np.frombuffer(msg.data, dtype=np.uint8)

            if encoding == "rgb8":
                image_np = image_np.reshape((height, width, 3))
                image_np = cv2.cvtColor(image_np, cv2.COLOR_RGB2BGR)

            elif encoding == "bgr8":
                image_np = image_np.reshape((height, width, 3))

            elif encoding == "mono8":
                image_np = image_np.reshape((height, width))
                image_np = cv2.cvtColor(image_np, cv2.COLOR_GRAY2BGR)

            elif encoding == "rgba8":
                image_np = image_np.reshape((height, width, 4))
                image_np = cv2.cvtColor(image_np, cv2.COLOR_RGBA2BGR)

            elif encoding == "bgra8":
                image_np = image_np.reshape((height, width, 4))
                image_np = cv2.cvtColor(image_np, cv2.COLOR_BGRA2BGR)

            else:
                self.get_logger().warn(f"Unsupported camera encoding: {encoding}")
                return

            # Resize image for browser performance.
            max_width = 320
            if width > max_width:
                scale = max_width / width
                new_width = int(width * scale)
                new_height = int(height * scale)
                image_np = cv2.resize(image_np, (new_width, new_height))

            success, jpeg = cv2.imencode(
                ".jpg",
                image_np,
                [cv2.IMWRITE_JPEG_QUALITY, 50]
            )

            if not success:
                return

            jpg_base64 = base64.b64encode(jpeg.tobytes()).decode("utf-8")

            camera_data = {
                "available": True,
                "encoding": encoding,
                "width": width,
                "height": height,
                "frame": "data:image/jpeg;base64," + jpg_base64
            }

            self.update_connection()

        except Exception as e:
            self.get_logger().warn(f"Camera conversion failed: {e}")

    def system_state_callback(self, msg):
        robot_data["system_state"] = msg.data
        self.update_connection()

    def map_status_callback(self, msg):
        robot_data["map_status"] = msg.data
        self.update_connection()

    def waypoint_plan_text_callback(self, msg):
        robot_data["waypoint_plan"] = msg.data
        self.update_connection()

    def estop_event_callback(self, msg):
        robot_data["last_estop_event"] = msg.data
        self.update_connection()

    def greek_markers_callback(self, msg):
        greek_list = pose_array_to_list(msg)

        robot_data["greek_markers"] = greek_list
        robot_data["markers"] = greek_list

        self.update_connection()

    def path_callback(self, msg):
        robot_data["path"] = path_to_list(msg)
        self.update_connection()

    def publish_cmd_vel(self, linear_x, angular_z):
        msg = Twist()
        msg.linear.x = float(linear_x)
        msg.linear.y = 0.0
        msg.linear.z = 0.0

        msg.angular.x = 0.0
        msg.angular.y = 0.0
        msg.angular.z = float(angular_z)

        self.cmd_vel_pub.publish(msg)

    def publish_stop(self):
        msg = Twist()
        msg.linear.x = 0.0
        msg.angular.z = 0.0
        self.cmd_vel_pub.publish(msg)

    def manual_timeout_check(self):
        global last_manual_command_time

        if last_manual_command_time <= 0.0:
            return

        if time.time() - last_manual_command_time > self.manual_timeout_sec:
            self.publish_stop()

    def call_start_mapping_service(self):
        if not self.start_mapping_client.wait_for_service(timeout_sec=1.0):
            return False, "Start mapping service is not available"

        request = Trigger.Request()
        future = self.start_mapping_client.call_async(request)

        start_time = time.time()
        timeout_sec = 3.0

        while not future.done():
            if time.time() - start_time > timeout_sec:
                return False, "Start mapping service call timed out"
            time.sleep(0.05)

        result = future.result()

        if result is None:
            return False, "Start mapping service returned no result"

        return bool(result.success), result.message

    def publish_waypoints_from_json(self, waypoints):
        """
        Publish loaded JSON waypoints as geometry_msgs/PoseArray.
        Frame is map.
        """
        msg = PoseArray()
        msg.header.frame_id = "map"
        msg.header.stamp = self.get_clock().now().to_msg()

        for wp in waypoints:
            pose = Pose()
            pose.position.x = float(wp.get("x", 0.0))
            pose.position.y = float(wp.get("y", 0.0))
            pose.position.z = float(wp.get("z", 0.0))

            pose.orientation.x = 0.0
            pose.orientation.y = 0.0
            pose.orientation.z = 0.0
            pose.orientation.w = 1.0

            msg.poses.append(pose)

        self.selected_waypoints_pub.publish(msg)

        robot_data["waypoint_plan"] = "JSON waypoints: " + " -> ".join(
            [f"{wp.get('name', 'WP')}({wp.get('x', 0):.2f},{wp.get('y', 0):.2f})" for wp in waypoints]
        )

        robot_data["command_status"] = f"Published {len(waypoints)} waypoint(s) from JSON"

        return True, f"Published {len(waypoints)} waypoint(s)"

    def call_start_waypoint_service(self):
        """
        Optional:
        Call /part3/waypoint/start if it exists.
        If it does not exist, publishing selected_goals still succeeds.
        """
        if not self.start_waypoint_client.wait_for_service(timeout_sec=0.5):
            return False, "Start waypoint service is not available"

        request = Trigger.Request()
        future = self.start_waypoint_client.call_async(request)

        start_time = time.time()
        timeout_sec = 3.0

        while not future.done():
            if time.time() - start_time > timeout_sec:
                return False, "Start waypoint service call timed out"
            time.sleep(0.05)

        result = future.result()

        if result is None:
            return False, "Start waypoint service returned no result"

        return bool(result.success), result.message


# =========================
# ROS thread
# =========================

def ros_spin():
    global web_ui_node

    rclpy.init()
    web_ui_node = Part3WebUINode()

    try:
        rclpy.spin(web_ui_node)
    except KeyboardInterrupt:
        pass
    finally:
        if web_ui_node is not None:
            web_ui_node.destroy_node()
        rclpy.shutdown()


# =========================
# Flask routes
# =========================

@app.route("/")
def index():
    return render_template("index.html")


@app.route("/api/status")
def status():
    return jsonify(robot_data)


@app.route("/api/map")
def get_map():
    return jsonify(map_data)


@app.route("/api/camera_frame")
def get_camera_frame():
    return jsonify(camera_data)


@app.route("/api/start_mapping", methods=["POST"])
def start_mapping():
    if web_ui_node is None:
        return jsonify({
            "success": False,
            "message": "ROS2 node is not ready"
        }), 503

    success, message = web_ui_node.call_start_mapping_service()

    if success:
        robot_data["system_state"] = "MAPPING"
        robot_data["map_status"] = "Mapping start command sent"
        robot_data["command_status"] = "Start mapping service success"
    else:
        robot_data["command_status"] = "Start mapping failed: " + message

    return jsonify({
        "success": success,
        "message": message
    })


@app.route("/api/waypoint_files")
def waypoint_files():
    categorized = list_waypoint_json_files()

    return jsonify({
        "directory": WAYPOINTS_DIR,
        "categories": categorized,
        "greek": categorized.get("greek", []),
        "color": categorized.get("color", []),
        "other": categorized.get("other", [])
    })


@app.route("/api/execute_waypoint_file", methods=["POST"])
def execute_waypoint_file():
    """
    Expected JSON:
    {
        "file": "xxx.json",
        "category": "greek" or "color" or "other",
        "auto_start": true
    }
    """
    data = request.get_json(silent=True)

    if data is None:
        return jsonify({
            "success": False,
            "message": "No JSON body received"
        }), 400

    file_name = data.get("file", "")
    auto_start = bool(data.get("auto_start", True))

    if not file_name:
        return jsonify({
            "success": False,
            "message": "No waypoint file selected"
        }), 400

    if web_ui_node is None:
        return jsonify({
            "success": False,
            "message": "ROS2 node is not ready"
        }), 503

    try:
        waypoints, category = load_waypoint_file(file_name)
    except Exception as e:
        return jsonify({
            "success": False,
            "message": f"Failed to load waypoint file: {e}"
        }), 400

    if len(waypoints) == 0:
        return jsonify({
            "success": False,
            "message": "Waypoint file contains no valid points"
        }), 400

    publish_success, publish_message = web_ui_node.publish_waypoints_from_json(waypoints)

    service_success = False
    service_message = "Start waypoint service was not called"

    if auto_start:
        service_success, service_message = web_ui_node.call_start_waypoint_service()

    if publish_success:
        robot_data["system_state"] = "WAYPOINT_DRIVE"
        robot_data["map_status"] = f"{category} waypoint command sent"
        robot_data["command_status"] = publish_message

    return jsonify({
        "success": publish_success,
        "message": publish_message,
        "file": file_name,
        "category": category,
        "waypoint_count": len(waypoints),
        "waypoints": waypoints,
        "start_service_success": service_success,
        "start_service_message": service_message
    })

@app.route("/api/execute_selected_waypoints", methods=["POST"])
def execute_selected_waypoints():
    """
    Execute only selected waypoint points from the UI.

    Expected JSON:
    {
        "waypoints": [
            {"name": "Alpha", "x": 1.2, "y": 3.4, "z": 0.0},
            {"name": "Beta", "x": 2.0, "y": 4.1, "z": 0.0}
        ],
        "auto_start": true
    }
    """
    data = request.get_json(silent=True)

    if data is None:
        return jsonify({
            "success": False,
            "message": "No JSON body received"
        }), 400

    waypoints = data.get("waypoints", [])
    auto_start = bool(data.get("auto_start", True))

    if len(waypoints) == 0:
        return jsonify({
            "success": False,
            "message": "No waypoint selected"
        }), 400

    if web_ui_node is None:
        return jsonify({
            "success": False,
            "message": "ROS2 node is not ready"
        }), 503

    # Reuse the existing publisher function
    publish_success, publish_message = web_ui_node.publish_waypoints_from_json(waypoints)

    service_success = False
    service_message = "Start waypoint service was not called"

    if auto_start:
        service_success, service_message = web_ui_node.call_start_waypoint_service()

    if publish_success:
        robot_data["system_state"] = "WAYPOINT_DRIVE"
        robot_data["map_status"] = "Selected waypoint command sent"
        robot_data["command_status"] = publish_message

    return jsonify({
        "success": publish_success,
        "message": publish_message,
        "waypoint_count": len(waypoints),
        "waypoints": waypoints,
        "start_service_success": service_success,
        "start_service_message": service_message
    })

@app.route("/api/manual_control", methods=["POST"])
def manual_control():
    """
    Receive keyboard/manual control command from web UI and publish /cmd_vel.

    Expected JSON:
    {
        "linear_x": 0.3,
        "angular_z": 0.0
    }
    """
    global last_manual_command_time

    data = request.get_json(silent=True)

    if data is None:
        return jsonify({
            "success": False,
            "message": "No JSON body received"
        }), 400

    linear_x = float(data.get("linear_x", 0.0))
    angular_z = float(data.get("angular_z", 0.0))

    # Simulation limits. Lower these for real robot.
    max_linear = 0.8
    max_angular = 1.2

    linear_x = max(min(linear_x, max_linear), -max_linear)
    angular_z = max(min(angular_z, max_angular), -max_angular)

    last_manual_command_time = time.time()

    if web_ui_node is not None:
        web_ui_node.publish_cmd_vel(linear_x, angular_z)

        return jsonify({
            "success": True,
            "linear_x": linear_x,
            "angular_z": angular_z
        })

    return jsonify({
        "success": False,
        "message": "ROS2 node is not ready"
    }), 503


@app.route("/api/stop", methods=["POST"])
def stop_robot():
    global last_manual_command_time

    last_manual_command_time = time.time()

    if web_ui_node is not None:
        web_ui_node.publish_stop()

    return jsonify({
        "success": True,
        "message": "Stop command published"
    })


@app.route("/api/debug")
def debug():
    categorized = list_waypoint_json_files()

    return jsonify({
        "robot_data": robot_data,
        "map_width": map_data["width"],
        "map_height": map_data["height"],
        "map_resolution": map_data["resolution"],
        "camera_available": camera_data["available"],
        "camera_encoding": camera_data["encoding"],
        "camera_width": camera_data["width"],
        "camera_height": camera_data["height"],
        "ros_node_ready": web_ui_node is not None,
        "start_mapping_service": START_MAPPING_SERVICE,
        "waypoints_dir": WAYPOINTS_DIR,
        "waypoint_file_count": (
            len(categorized.get("greek", []))
            + len(categorized.get("color", []))
            + len(categorized.get("other", []))
        ),
        "waypoint_categories": {
            "greek": len(categorized.get("greek", [])),
            "color": len(categorized.get("color", [])),
            "other": len(categorized.get("other", []))
        },
        "selected_waypoints_topic": SELECTED_WAYPOINTS_TOPIC,
        "start_waypoint_service": START_WAYPOINT_SERVICE
    })


# =========================
# Main
# =========================

if __name__ == "__main__":
    ros_thread = threading.Thread(target=ros_spin, daemon=True)
    ros_thread.start()

    app.run(host="0.0.0.0", port=5000)