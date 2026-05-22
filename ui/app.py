from flask import Flask, render_template, jsonify, request
import threading
import time
import math
import base64

import cv2
import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy

from std_msgs.msg import String
from std_srvs.srv import Trigger
from nav_msgs.msg import Odometry, OccupancyGrid, Path
from sensor_msgs.msg import LaserScan, Image
from geometry_msgs.msg import PoseArray, Twist


app = Flask(__name__)


# =========================
# Topic / service configuration
# =========================

MAP_TOPIC = "/map"
ODOM_TOPIC = "/odometry/filtered"
SCAN_TOPIC = "/scan"
CAMERA_TOPIC = "/camera/image"
CMD_VEL_TOPIC = "/cmd_vel"

SYSTEM_STATE_TOPIC = "/part3/system/state"
MAP_STATUS_TOPIC = "/part3/mapping/map_status"
WAYPOINT_PLAN_TEXT_TOPIC = "/part3/waypoint/plan"
ESTOP_EVENT_TOPIC = "/part3/safety/estop_event"
GREEK_MARKERS_TOPIC = "/part3/perception/greek_markers"

NAV_PATH_TOPIC = "/plan"

START_MAPPING_SERVICE = "/part3/mapping/start"


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

# Safety / dead-man state
# Stop Robot works as a hold switch.
# Auto E-Stop is triggered when a moving keyboard command loses heartbeat.
stop_hold_active = False
auto_estop_active = False
manual_motion_active = False


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
# ROS2 Node
# =========================

class Part3WebUINode(Node):
    def __init__(self):
        super().__init__("part3_web_ui_node")

        # /map from slam_toolbox uses RELIABLE + TRANSIENT_LOCAL.
        map_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )

        normal_qos = 10

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

        self.system_state_pub = self.create_publisher(
            String,
            SYSTEM_STATE_TOPIC,
            normal_qos
        )

        self.estop_event_pub = self.create_publisher(
            String,
            ESTOP_EVENT_TOPIC,
            normal_qos
        )

        # Service clients
        self.start_mapping_client = self.create_client(
            Trigger,
            START_MAPPING_SERVICE
        )

        # Dead-man timeout for manual control.
        self.manual_timeout_sec = 1.0
        self.safety_timer = self.create_timer(
            0.1,
            self.manual_timeout_check
        )

        self.get_logger().info("Part 3 Web UI node started.")
        self.get_logger().info(f"Subscribing map: {MAP_TOPIC}")
        self.get_logger().info(f"Subscribing odom: {ODOM_TOPIC}")
        self.get_logger().info(f"Subscribing scan: {SCAN_TOPIC}")
        self.get_logger().info(f"Subscribing camera: {CAMERA_TOPIC}")
        self.get_logger().info(f"Publishing manual control: {CMD_VEL_TOPIC}")
        self.get_logger().info(f"Start mapping service: {START_MAPPING_SERVICE}")

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

    def odom_callback(self, msg):
        robot_data["x"] = round(float(msg.pose.pose.position.x), 3)
        robot_data["y"] = round(float(msg.pose.pose.position.y), 3)
        robot_data["yaw"] = round(float(quaternion_to_yaw_deg(msg.pose.pose.orientation)), 2)

        self.update_connection()

    def scan_callback(self, msg):
        # Downsample scan data to reduce browser load.
        step = 5
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
            max_width = 640
            if width > max_width:
                scale = max_width / width
                new_width = int(width * scale)
                new_height = int(height * scale)
                image_np = cv2.resize(image_np, (new_width, new_height))

            success, jpeg = cv2.imencode(
                ".jpg",
                image_np,
                [cv2.IMWRITE_JPEG_QUALITY, 75]
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

        # For now, also show Greek markers in the general marker list.
        # If your team later adds /part3/perception/markers, we can separate this.
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
        """
        Publish one zero-velocity command.
        The safety timer calls this repeatedly when stop hold or auto E-stop is active.
        """
        msg = Twist()
        msg.linear.x = 0.0
        msg.linear.y = 0.0
        msg.linear.z = 0.0
        msg.angular.x = 0.0
        msg.angular.y = 0.0
        msg.angular.z = 0.0
        self.cmd_vel_pub.publish(msg)

    def trigger_auto_estop(self, reason):
        """
        Automatic E-Stop:
        - activates stop hold
        - continuously publishes zero velocity through the safety timer
        - updates UI state and publishes /part3/safety/estop_event
        """
        global auto_estop_active
        global stop_hold_active
        global manual_motion_active

        auto_estop_active = True
        stop_hold_active = True
        manual_motion_active = False

        self.publish_stop()

        state_msg = String()
        state_msg.data = "WAYPOINT_FAILED"
        self.system_state_pub.publish(state_msg)

        event_msg = String()
        event_msg.data = (
            f"auto_estop timestamp={time.time():.3f} "
            f"source=web_ui reason={reason}"
        )
        self.estop_event_pub.publish(event_msg)

        robot_data["system_state"] = "WAYPOINT_FAILED"
        robot_data["last_estop_event"] = event_msg.data
        robot_data["command_status"] = "Auto E-Stop: " + reason
        robot_data["map_status"] = "Robot stopped by automatic E-Stop"

        return event_msg.data

    def set_stop_hold(self, active):
        """
        Stop Robot works as a dead-man hold switch.
        When active, keyboard motion commands are blocked and zero velocity is published continuously.
        """
        global stop_hold_active
        global manual_motion_active

        stop_hold_active = bool(active)

        if stop_hold_active:
            manual_motion_active = False
            self.publish_stop()
            robot_data["command_status"] = "Stop hold active"
            robot_data["map_status"] = "Robot held by Stop Robot switch"
        else:
            robot_data["command_status"] = "Stop hold cleared"
            robot_data["map_status"] = "Stop hold cleared"

        return stop_hold_active

    def clear_auto_estop(self):
        """
        Clear both stop hold and automatic E-stop.
        """
        global auto_estop_active
        global stop_hold_active
        global manual_motion_active

        auto_estop_active = False
        stop_hold_active = False
        manual_motion_active = False

        self.publish_stop()

        robot_data["command_status"] = "Stop hold / Auto E-Stop cleared"
        robot_data["map_status"] = "Ready"

        return True

    def manual_timeout_check(self):
        global last_manual_command_time
        global stop_hold_active
        global auto_estop_active
        global manual_motion_active

        # Stop hold or Auto E-Stop: keep publishing zero velocity.
        if stop_hold_active or auto_estop_active:
            self.publish_stop()
            return

        if last_manual_command_time <= 0.0:
            return

        # If robot was moving but UI stopped sending heartbeat commands, trigger Auto E-Stop.
        if manual_motion_active:
            if time.time() - last_manual_command_time > self.manual_timeout_sec:
                self.trigger_auto_estop("manual_control_timeout")

    def call_start_mapping_service(self):
        """
        Call:
        ros2 service call /part3/mapping/start std_srvs/srv/Trigger {}
        """
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

    return jsonify({
        "success": success,
        "message": message
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

    Safety behavior:
    - stop_hold_active or auto_estop_active blocks all non-zero motion commands.
    - while a motion key is held, the frontend should resend commands continuously.
    - if that heartbeat stops, manual_timeout_check() triggers automatic E-Stop.
    """
    global last_manual_command_time
    global stop_hold_active
    global auto_estop_active
    global manual_motion_active

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

    is_motion_command = abs(linear_x) > 1e-4 or abs(angular_z) > 1e-4

    # Stop hold / Auto E-Stop blocks motion commands.
    # Stop commands are still allowed.
    if (stop_hold_active or auto_estop_active) and is_motion_command:
        if web_ui_node is not None:
            web_ui_node.publish_stop()

        return jsonify({
            "success": False,
            "message": "Stop hold or Auto E-Stop is active",
            "stop_hold_active": stop_hold_active,
            "auto_estop_active": auto_estop_active
        }), 423

    last_manual_command_time = time.time()
    manual_motion_active = is_motion_command

    if web_ui_node is not None:
        if is_motion_command:
            web_ui_node.publish_cmd_vel(linear_x, angular_z)
        else:
            web_ui_node.publish_stop()

        return jsonify({
            "success": True,
            "linear_x": linear_x,
            "angular_z": angular_z,
            "manual_motion_active": manual_motion_active,
            "stop_hold_active": stop_hold_active,
            "auto_estop_active": auto_estop_active
        })

    return jsonify({
        "success": False,
        "message": "ROS2 node is not ready"
    }), 503


@app.route("/api/stop", methods=["POST"])
def stop_robot():
    """
    Stop Robot is a hold switch:
    - activates stop_hold_active
    - safety timer keeps publishing /cmd_vel = 0
    - keyboard motion is blocked until /api/clear_stop_hold is called
    """
    global last_manual_command_time

    last_manual_command_time = time.time()

    if web_ui_node is None:
        return jsonify({
            "success": False,
            "message": "ROS2 node is not ready"
        }), 503

    active = web_ui_node.set_stop_hold(True)

    return jsonify({
        "success": True,
        "message": "Stop hold activated",
        "stop_hold_active": active
    })


@app.route("/api/clear_stop_hold", methods=["POST"])
def clear_stop_hold():
    """
    Clear Stop Hold and Auto E-Stop.
    """
    if web_ui_node is None:
        return jsonify({
            "success": False,
            "message": "ROS2 node is not ready"
        }), 503

    web_ui_node.clear_auto_estop()

    return jsonify({
        "success": True,
        "message": "Stop hold / Auto E-Stop cleared",
        "stop_hold_active": False,
        "auto_estop_active": False
    })


@app.route("/api/debug")
def debug():
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
        "stop_hold_active": stop_hold_active,
        "auto_estop_active": auto_estop_active,
        "manual_motion_active": manual_motion_active,
        "start_mapping_service": START_MAPPING_SERVICE
    })


# =========================
# Main
# =========================

if __name__ == "__main__":
    ros_thread = threading.Thread(target=ros_spin, daemon=True)
    ros_thread.start()

    app.run(host="0.0.0.0", port=5000)