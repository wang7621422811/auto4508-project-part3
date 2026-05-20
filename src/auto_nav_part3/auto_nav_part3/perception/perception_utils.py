"""Shared perception helpers."""

from __future__ import annotations

import math
from typing import Optional

import numpy as np

CAMERA_FORWARD_M = 0.24
OBSTACLE_HALF_DEPTH_M = 0.25

# laser_frame 相对 base_link 的前向偏移（URDF: chassis xyz="0 0 0.177" + laser xyz="0.2 0 0.104"）
# X 方向: 0 + 0.20 = 0.20 m
LASER_FORWARD_M = 0.20


def depth_to_odom(
    depth_frame: np.ndarray,
    cx_px: int,
    cy_px: int,
    robot_x: float,
    robot_y: float,
    robot_yaw: float,
    cam_hfov: float,
) -> Optional[tuple[float, float, float]]:
    """
    用机器人 odom 姿态和目标像素深度估算障碍物中心的 odom 坐标。

    这是给 detector 使用的轻量路径：不依赖 TF，只要求有 /odom 和深度图。
    深度图量到的是障碍物前表面，因此这里按仿真方块尺寸补偿 0.25m。
    返回 (x, y, depth_m)，深度不可用时返回 None。
    """
    if depth_frame is None:
        return None

    h, w = depth_frame.shape[:2]
    cx_px = max(2, min(int(cx_px), w - 3))
    cy_px = max(2, min(int(cy_px), h - 3))

    patch = depth_frame[cy_px - 2:cy_px + 3, cx_px - 2:cx_px + 3]
    valid = patch[np.isfinite(patch)]
    valid = valid[(valid > 0.1) & (valid < 15.0)]
    if valid.size == 0:
        return None

    depth_m = float(np.median(valid))
    if math.isnan(depth_m) or not (0.1 < depth_m < 15.0):
        return None

    fx = (w / 2.0) / math.tan(cam_hfov / 2.0)
    bearing = math.atan2(cx_px - (w / 2.0), fx)
    heading = robot_yaw + bearing

    camera_x = robot_x + CAMERA_FORWARD_M * math.cos(robot_yaw)
    camera_y = robot_y + CAMERA_FORWARD_M * math.sin(robot_yaw)
    center_range_m = depth_m + OBSTACLE_HALF_DEPTH_M

    return (
        camera_x + center_range_m * math.cos(heading),
        camera_y + center_range_m * math.sin(heading),
        depth_m,
    )


def scan_range_at_bearing(
    scan,
    bearing_rad: float,
    n_beams: int = 7,
) -> Optional[float]:
    """
    从 LaserScan 取指定方位角附近 n_beams 条光束的中位测距值。

    bearing_rad: 机器人坐标系方位角，正值=左(+Y), 负值=右(-Y)，与 ROS LaserScan 约定一致。
    返回中位距离（米），无有效光束时返回 None。
    """
    if scan is None:
        return None
    angle_inc = scan.angle_increment
    if angle_inc == 0.0:
        return None
    n = len(scan.ranges)
    center_idx = int(round((bearing_rad - scan.angle_min) / angle_inc))
    half = n_beams // 2
    valid: list[float] = []
    for i in range(center_idx - half, center_idx + half + 1):
        if 0 <= i < n:
            r = float(scan.ranges[i])
            if math.isfinite(r) and scan.range_min < r < scan.range_max:
                valid.append(r)
    if not valid:
        return None
    valid.sort()
    return valid[len(valid) // 2]


def lidar_to_odom(
    scan,
    cx_px: int,
    img_w: int,
    cam_hfov: float,
    robot_x: float,
    robot_y: float,
    robot_yaw: float,
    half_depth_m: float = OBSTACLE_HALF_DEPTH_M,
    n_beams: int = 7,
) -> Optional[tuple[float, float, float]]:
    """
    用相机像素 cx_px 确定方位角，用雷达测距代替深度相机，估算障碍物 odom 坐标。

    符号约定（与 ROS LaserScan 一致）：
      bearing = atan2(W/2 - cx_px, fx)
      正值 = 物体在图像左侧 = 机器人左侧(+Y)
      负值 = 物体在图像右侧 = 机器人右侧(-Y)

    half_depth_m: 障碍物半深度补偿（将雷达前表面测距转换到障碍物中心），
                  颜色障碍物取 0.25m，希腊字母桶取 0.0m。

    返回 (obs_x, obs_y, range_m)，雷达无有效读数时返回 None。
    """
    fx = (img_w / 2.0) / math.tan(cam_hfov / 2.0)
    # 正确符号：小 cx_px（图像左侧）= 机器人左侧 = 正方位角
    bearing = math.atan2((img_w / 2.0) - cx_px, fx)

    range_m = scan_range_at_bearing(scan, bearing, n_beams)
    if range_m is None:
        return None

    heading = robot_yaw + bearing
    # 以雷达安装点（前移 LASER_FORWARD_M）为起点计算障碍物位置
    laser_x = robot_x + LASER_FORWARD_M * math.cos(robot_yaw)
    laser_y = robot_y + LASER_FORWARD_M * math.sin(robot_yaw)
    total_range = range_m + half_depth_m
    return (
        laser_x + total_range * math.cos(heading),
        laser_y + total_range * math.sin(heading),
        range_m,
    )
