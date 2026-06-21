"""Shared perception helpers."""

from __future__ import annotations

import math
from typing import Optional

import numpy as np

CAMERA_FORWARD_M = 0.24
OBSTACLE_HALF_DEPTH_M = 0.25

# Forward offset of laser_frame relative to base_link (URDF: chassis xyz="0 0 0.177" + laser xyz="0.2 0 0.104")
# X: 0 + 0.20 = 0.20 m
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
    Estimate the odom coordinates of an obstacle's centre using robot odom pose and pixel depth.

    Lightweight path for detectors: no TF required, only /odom and a depth frame.
    Depth frames measure the obstacle's front surface, so 0.25 m (half the simulated cube size) is added.
    Returns (x, y, depth_m), or None if depth is unavailable.
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
    Return the median range of n_beams beams nearest to bearing_rad in a LaserScan.

    bearing_rad: bearing in robot frame, positive = left (+Y), negative = right (-Y),
                 consistent with ROS LaserScan convention.
    Returns median range in metres, or None if no valid beams are found.
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
    Estimate obstacle odom coordinates using camera pixel cx_px for bearing and lidar range instead of a depth camera.

    Sign convention (consistent with ROS LaserScan):
      bearing = atan2(W/2 - cx_px, fx)
      positive = object on left in image = robot left (+Y)
      negative = object on right in image = robot right (-Y)

    half_depth_m: half-depth compensation to convert lidar front-surface range to obstacle centre;
                  use 0.25 m for colour obstacles, 0.0 m for Greek-letter barrels.

    Returns (obs_x, obs_y, range_m), or None if lidar has no valid reading.
    """
    fx = (img_w / 2.0) / math.tan(cam_hfov / 2.0)
    # Correct sign: small cx_px (left of image) = robot left = positive bearing
    bearing = math.atan2((img_w / 2.0) - cx_px, fx)

    range_m = scan_range_at_bearing(scan, bearing, n_beams)
    if range_m is None:
        return None

    heading = robot_yaw + bearing
    # Compute obstacle position from the lidar mount point (offset LASER_FORWARD_M forward)
    laser_x = robot_x + LASER_FORWARD_M * math.cos(robot_yaw)
    laser_y = robot_y + LASER_FORWARD_M * math.sin(robot_yaw)
    total_range = range_m + half_depth_m
    return (
        laser_x + total_range * math.cos(heading),
        laser_y + total_range * math.sin(heading),
        range_m,
    )
