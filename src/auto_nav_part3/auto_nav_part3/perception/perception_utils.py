"""Shared perception helpers."""

from __future__ import annotations

import math
from typing import Optional

import numpy as np

CAMERA_FORWARD_M = 0.24
OBSTACLE_HALF_DEPTH_M = 0.25


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
