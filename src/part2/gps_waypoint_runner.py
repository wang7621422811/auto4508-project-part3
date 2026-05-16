#!/usr/bin/env python3
"""
GPS Waypoint Runner - AUTO4508 Part 2
=======================================

启动时读取当前 GPS 作为原点, 把 waypoint 的 GPS 坐标转换成 odom 的 (x, y) 米坐标,
然后用 Nav2 导航.

前提:
  - GPS 在 /dev/ttyACM0 (NMEA 格式)
  - 车启动位置在 GPS 起点附近
  - 车朝向 X 正方向 (对应真北或约定方向)

用法:
  python3 gps_waypoint_runner.py
"""

import time
import json
import math
import os
import serial
import threading

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.duration import Duration

from nav2_msgs.action import NavigateToPose
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import Image
from std_msgs.msg import String
from nav_msgs.msg import OccupancyGrid

from cv_bridge import CvBridge
import cv2

from tf2_ros import Buffer, TransformListener, TransformException


STATUS_SUCCEEDED = 4
STATUS_ABORTED = 6
R_EARTH = 6371000


class GPSReader:
    """从串口读 NMEA GPS 数据"""

    def __init__(self, port='/dev/ttyACM0', baud=9600):
        self.port = port
        self.baud = baud
        self.lat = None
        self.lon = None
        self.fix = False
        self.lock = threading.Lock()
        self.running = True
        self.thread = threading.Thread(target=self._read_loop, daemon=True)
        self.thread.start()

    def _parse_gpgga(self, line):
        """解析 GPGGA 语句"""
        parts = line.split(',')
        if len(parts) < 7:
            return None

        fix_quality = parts[6]
        if fix_quality == '0':
            return None

        try:
            lat_str, lat_dir = parts[2], parts[3]
            lon_str, lon_dir = parts[4], parts[5]

            lat_deg = float(lat_str[:2])
            lat_min = float(lat_str[2:])
            lat = lat_deg + lat_min / 60.0
            if lat_dir == 'S':
                lat = -lat

            lon_deg = float(lon_str[:3])
            lon_min = float(lon_str[3:])
            lon = lon_deg + lon_min / 60.0
            if lon_dir == 'W':
                lon = -lon

            return lat, lon
        except Exception:
            return None

    def _read_loop(self):
        while self.running:
            try:
                with serial.Serial(self.port, self.baud, timeout=1.0) as ser:
                    while self.running:
                        try:
                            line = ser.readline().decode('ascii', errors='ignore').strip()
                            if line.startswith('$GPGGA') or line.startswith('$GNGGA'):
                                result = self._parse_gpgga(line)
                                if result:
                                    with self.lock:
                                        self.lat, self.lon = result
                                        self.fix = True
                        except Exception:
                            pass
            except Exception:
                time.sleep(2.0)

    def get(self):
        with self.lock:
            return self.lat, self.lon, self.fix

    def wait_for_fix(self, timeout=60.0):
        """等待 GPS 有效"""
        start = time.time()
        while time.time() - start < timeout:
            lat, lon, fix = self.get()
            if fix and lat is not None:
                return lat, lon
            time.sleep(1.0)
        return None

    def stop(self):
        self.running = False


def gps_to_xy(lat, lon, ref_lat, ref_lon):
    """GPS → (x, y) 米, 以 ref 为原点"""
    x = R_EARTH * math.radians(lon - ref_lon) * math.cos(math.radians(ref_lat))
    y = R_EARTH * math.radians(lat - ref_lat)
    return x, y


class WaypointRunner(Node):
    def __init__(self):
        super().__init__('gps_waypoint_runner')
        self.bridge = CvBridge()

        # ============================================
        # GPS Waypoints (demo 当天填入老师给的坐标)
        # ============================================
        self.gps_waypoints = [
            {'name': 'WP1', 'lat': -31.980408, 'lon': 115.817671},
            {'name': 'WP2', 'lat': -31.980291, 'lon': 115.817763},
            {'name': 'WP3', 'lat': -31.980207, 'lon': 115.817681},
            {'name': 'WP4', 'lat': -31.980197, 'lon': 115.817678},
            {'name': 'WP5', 'lat': -31.979961, 'lon': 115.817545},
            {'name': 'WP6', 'lat': -31.980168, 'lon': 115.817341},
            {'name': 'WP7', 'lat': -31.980512, 'lon': 115.817347},
        ]

        # GPS 读取器
        self.gps_reader = GPSReader('/dev/ttyACM0')

        # 这些在 setup() 里填
        self.ref_lat = None
        self.ref_lon = None
        self.waypoints = []
        self.start_pose = {'name': 'START', 'x': 0.0, 'y': 0.0, 'yaw': 0.0}

        # Nav2
        self.nav_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')

        # TF
        self.tf_buffer = Buffer(cache_time=Duration(seconds=30.0))
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # 订阅地图
        self.map_msg = None
        self.map_sub = self.create_subscription(
            OccupancyGrid, '/map', self.map_cb, 10)

        # 订阅检测
        self.latest_detections = []
        self.det_sub = self.create_subscription(
            String, '/detected_objects', self.detection_cb, 10)

        # 订阅图像
        self.latest_frame = None
        self.img_sub = self.create_subscription(
            Image, '/oak/rgb/image_raw', self.image_cb, 10)

        self.photo_dir = '/ros2_ws/photos'
        os.makedirs(self.photo_dir, exist_ok=True)

        self.journey_log = []
        self.start_time = None
        self.summary_printed = False

        self.get_logger().info('=== GPS Waypoint Runner 启动 ===')

    def setup_gps(self):
        """启动时读 GPS 作为原点, 转换所有 waypoint"""
        self.get_logger().info('等待 GPS 定位 (最多 60 秒)...')
        result = self.gps_reader.wait_for_fix(timeout=60.0)

        if result is None:
            self.get_logger().error('GPS 未获得定位! 退出')
            return False

        self.ref_lat, self.ref_lon = result
        self.get_logger().info(
            f'GPS 起点: lat={self.ref_lat:.6f}, lon={self.ref_lon:.6f}'
        )

        # 转换 waypoint
        self.waypoints = []
        prev_x, prev_y = 0.0, 0.0
        for i, wp in enumerate(self.gps_waypoints):
            x, y = gps_to_xy(wp['lat'], wp['lon'], self.ref_lat, self.ref_lon)

            # yaw: 面向下一个点
            if i < len(self.gps_waypoints) - 1:
                nx, ny = gps_to_xy(
                    self.gps_waypoints[i + 1]['lat'],
                    self.gps_waypoints[i + 1]['lon'],
                    self.ref_lat, self.ref_lon
                )
                yaw = math.atan2(ny - y, nx - x)
            else:
                yaw = 0.0

            dist = math.sqrt((x - prev_x) ** 2 + (y - prev_y) ** 2)
            self.waypoints.append({'name': wp['name'], 'x': x, 'y': y, 'yaw': yaw})

            self.get_logger().info(
                f'  {wp["name"]}: x={x:.1f}, y={y:.1f}, 距上={dist:.1f}m'
            )
            prev_x, prev_y = x, y

        return True

    def map_cb(self, msg):
        self.map_msg = msg

    def detection_cb(self, msg):
        try:
            self.latest_detections = json.loads(msg.data)
        except Exception:
            self.latest_detections = []

    def image_cb(self, msg):
        try:
            self.latest_frame = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
        except Exception:
            pass

    def create_pose(self, x, y, yaw):
        pose = PoseStamped()
        pose.header.frame_id = 'map'
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.pose.position.x = float(x)
        pose.pose.position.y = float(y)
        pose.pose.orientation.z = math.sin(yaw / 2.0)
        pose.pose.orientation.w = math.cos(yaw / 2.0)
        return pose

    def get_robot_pose_in_map(self):
        try:
            tf = self.tf_buffer.lookup_transform(
                'map', 'base_link', rclpy.time.Time(),
                timeout=Duration(seconds=1.0))
            return tf.transform.translation.x, tf.transform.translation.y
        except TransformException:
            return None

    def wait_for_nav(self, timeout_sec=60.0):
        if not self.nav_client.wait_for_server(timeout_sec=timeout_sec):
            return False

        start = time.time()
        while rclpy.ok() and (time.time() - start) < timeout_sec:
            rclpy.spin_once(self, timeout_sec=0.1)
            if self.map_msg is not None and self.get_robot_pose_in_map():
                return True
        return False

    def navigate_to(self, waypoint, retries=2):
        name = waypoint['name']
        for attempt in range(retries + 1):
            self.get_logger().info(
                f'导航到 {name}: ({waypoint["x"]:.1f}, {waypoint["y"]:.1f}), 第 {attempt + 1}/{retries + 1} 次'
            )

            goal = NavigateToPose.Goal()
            goal.pose = self.create_pose(waypoint['x'], waypoint['y'], waypoint['yaw'])

            send_future = self.nav_client.send_goal_async(goal)
            rclpy.spin_until_future_complete(self, send_future, timeout_sec=5.0)

            handle = send_future.result()
            if not handle or not handle.accepted:
                time.sleep(1.0)
                continue

            result_future = handle.get_result_async()
            rclpy.spin_until_future_complete(self, result_future, timeout_sec=300.0)

            result = result_future.result()
            if result and result.status == STATUS_SUCCEEDED:
                self.get_logger().info(f'{name}: 到达!')
                return True

            self.get_logger().warn(
                f'{name}: 失败 status={result.status if result else "timeout"}')
            time.sleep(1.0)
        return False

    def take_photo(self, label):
        if self.latest_frame is None:
            return None
        ts = time.strftime('%Y%m%d_%H%M%S')
        path = f'{self.photo_dir}/{label}_{ts}.jpg'
        cv2.imwrite(path, self.latest_frame)
        self.get_logger().info(f'照片: {path}')
        return path

    def detect_at_waypoint(self, wp_name):
        self.get_logger().info(f'{wp_name}: 检测附近物体...')
        time.sleep(2.0)

        photo = self.take_photo(f'{wp_name}_cone')
        detections = self.latest_detections.copy()
        cones = [d for d in detections if d.get('color') == 'orange']
        objects = [d for d in detections if d.get('color') != 'orange']

        result = {
            'waypoint': wp_name,
            'photo': photo,
            'cone_detected': len(cones) > 0,
            'cone_position': cones[0].get('position') if cones else None,
            'objects': [],
        }

        for obj in objects:
            obj_photo = self.take_photo(f'{wp_name}_{obj.get("color")}_{obj.get("shape")}')
            result['objects'].append({
                'color': obj.get('color'),
                'shape': obj.get('shape'),
                'distance_m': obj.get('distance_m'),
                'position': obj.get('position'),
                'photo': obj_photo,
            })
            self.get_logger().info(
                f'{wp_name}: {obj.get("color")} {obj.get("shape")}, '
                f'{obj.get("distance_m")}m, {obj.get("position")}'
            )

        if cones:
            self.get_logger().info(f'{wp_name}: 锥桶在 {cones[0].get("position")}')
        else:
            self.get_logger().warn(f'{wp_name}: 无锥桶')

        return result

    def print_summary(self):
        if self.summary_printed:
            return
        self.summary_printed = True

        elapsed = time.time() - self.start_time if self.start_time else 0
        self.get_logger().info('\n' + '=' * 50)
        self.get_logger().info('         行程总结')
        self.get_logger().info('=' * 50)
        self.get_logger().info(f'用时: {elapsed:.1f}s ({elapsed/60:.1f}min)')
        self.get_logger().info(f'完成: {len(self.journey_log)}/{len(self.waypoints)}')

        for log in self.journey_log:
            status = 'Y' if log.get('reached') else 'X'
            cone = '✓' if log.get('cone_detected') else '✗'
            self.get_logger().info(f'  {status} {log["waypoint"]}: 锥桶={cone}')

        summary_path = f'{self.photo_dir}/journey_summary.json'
        with open(summary_path, 'w') as f:
            json.dump({
                'ref_gps': {'lat': self.ref_lat, 'lon': self.ref_lon},
                'total_time_sec': elapsed,
                'waypoints_visited': len(self.journey_log),
                'waypoints_total': len(self.waypoints),
                'log': self.journey_log,
            }, f, indent=2, ensure_ascii=False)
        self.get_logger().info(f'保存: {summary_path}')

    def run(self):
        # 读 GPS 转换 waypoint
        if not self.setup_gps():
            return

        # 等 Nav2 就绪
        self.get_logger().info('等待 Nav2...')
        if not self.wait_for_nav(60.0):
            self.get_logger().error('Nav2 未就绪')
            return

        self.get_logger().info('开始导航!')
        self.start_time = time.time()

        for i, wp in enumerate(self.waypoints):
            self.get_logger().info(f'\n--- {i+1}/{len(self.waypoints)}: {wp["name"]} ---')
            reached = self.navigate_to(wp, retries=2)

            if reached:
                det = self.detect_at_waypoint(wp['name'])
                det['reached'] = True
                self.journey_log.append(det)
            else:
                self.journey_log.append({
                    'waypoint': wp['name'], 'reached': False,
                    'photo': None, 'cone_detected': False, 'objects': [],
                })

        # 回起点
        self.get_logger().info('\n--- 返回起点 ---')
        self.navigate_to(self.start_pose, retries=1)

        self.print_summary()
        self.get_logger().info('完成!')


def main():
    rclpy.init()
    node = WaypointRunner()
    try:
        node.run()
    except KeyboardInterrupt:
        pass
    finally:
        node.print_summary()
        node.gps_reader.stop()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
