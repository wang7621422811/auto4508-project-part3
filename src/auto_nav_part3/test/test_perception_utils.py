#!/usr/bin/env python3
"""
test_perception_utils.py — perception_utils 数学逻辑单元测试

不需要 ROS 环境，用 pytest 直接运行：
  ./scripts/launch.sh test
  # 或直接调用 pytest：
  pytest src/auto_nav_part3/test/test_perception_utils.py -v
"""

import math
import sys
from pathlib import Path

# 让 pytest 能找到 auto_nav_part3 包（source install/setup.bash 后也可直接 import）
sys.path.insert(0, str(Path(__file__).parents[1]))

from auto_nav_part3.perception.perception_utils import (
    LASER_FORWARD_M,
    OBSTACLE_HALF_DEPTH_M,
    lidar_to_odom,
    scan_range_at_bearing,
)


# ─── Mock LaserScan（与 URDF 中雷达配置一致）─────────────────────────────────

class _FakeScan:
    """模拟 sensor_msgs/LaserScan：720 光束，±120°，全场统一距离。"""
    angle_min       = -2.0944          # -120°（右端，beam 0）
    angle_max       =  2.0944          # +120°（左端，beam 719）
    angle_increment =  4.1888 / 720    # ≈ 0.0058 rad/beam ≈ 0.33°
    range_min       =  0.35
    range_max       = 12.0

    def __init__(self, uniform_range: float = 2.0):
        self.ranges = [uniform_range] * 720


# 相机参数（与 URDF cam_frame horizontal_fov 一致）
_W    = 640
_HFOV = 1.089   # rad ≈ 62.4°


# ═══════════════════════════════════════════════════════════════════════════════
# scan_range_at_bearing 测试
# ═══════════════════════════════════════════════════════════════════════════════

def test_scan_range_forward():
    """正前方 bearing=0 应返回正确距离。"""
    r = scan_range_at_bearing(_FakeScan(2.0), 0.0)
    assert r is not None
    assert abs(r - 2.0) < 0.01, f"期望 2.0，实际 {r:.4f}"


def test_scan_range_left_30deg():
    """左侧 30°（正方位角）应返回正确距离。"""
    r = scan_range_at_bearing(_FakeScan(1.5), math.radians(30))
    assert r is not None
    assert abs(r - 1.5) < 0.01, f"期望 1.5，实际 {r:.4f}"


def test_scan_range_right_30deg():
    """右侧 30°（负方位角）应返回正确距离。"""
    r = scan_range_at_bearing(_FakeScan(3.0), math.radians(-30))
    assert r is not None
    assert abs(r - 3.0) < 0.01, f"期望 3.0，实际 {r:.4f}"


def test_scan_range_none_when_scan_is_none():
    """scan=None 时返回 None，不崩溃。"""
    assert scan_range_at_bearing(None, 0.0) is None


def test_scan_range_inf_single_beam_returns_none():
    """目标光束为 inf 且 n_beams=1 时返回 None。"""
    scan = _FakeScan(2.0)
    scan.ranges[360] = float('inf')
    assert scan_range_at_bearing(scan, 0.0, n_beams=1) is None


def test_scan_range_out_of_fov_returns_none():
    """超出 ±120° 扫描范围的 bearing 应返回 None（无对应 beam）。"""
    assert scan_range_at_bearing(_FakeScan(2.0), math.radians(150)) is None


def test_scan_range_median_ignores_inf_outliers():
    """周边有 inf 噪点时，中位值应仍为正常距离。"""
    scan = _FakeScan(2.0)
    scan.ranges[358] = float('inf')   # beam 360±2 中的 2 个异常
    scan.ranges[362] = float('inf')
    r = scan_range_at_bearing(scan, 0.0, n_beams=5)  # 5 个中 3 个有效
    assert r is not None
    assert abs(r - 2.0) < 0.01, f"中位值应为 2.0，实际 {r:.4f}"


def test_scan_range_all_inf_returns_none():
    """全为 inf 时返回 None。"""
    scan = _FakeScan(0.0)
    scan.ranges = [float('inf')] * 720
    assert scan_range_at_bearing(scan, 0.0) is None


# ═══════════════════════════════════════════════════════════════════════════════
# lidar_to_odom 测试
# ═══════════════════════════════════════════════════════════════════════════════

def test_lidar_forward_obstacle_position():
    """
    图像正中（cx=320），机器人在原点朝东（yaw=0），范围 2.0m。
    obs_x = LASER_FORWARD_M + range + OBSTACLE_HALF_DEPTH_M，obs_y ≈ 0。
    """
    obs = lidar_to_odom(_FakeScan(2.0), 320, _W, _HFOV, 0.0, 0.0, 0.0)
    assert obs is not None, "应返回有效位置"
    obs_x, obs_y, range_m = obs
    expected_x = LASER_FORWARD_M + 2.0 + OBSTACLE_HALF_DEPTH_M
    assert abs(obs_x - expected_x) < 0.05, f"obs_x 期望 {expected_x:.3f}，实际 {obs_x:.3f}"
    assert abs(obs_y) < 0.05,              f"obs_y 应接近 0，实际 {obs_y:.3f}"
    assert abs(range_m - 2.0) < 0.01,     f"range 期望 2.0，实际 {range_m:.3f}"


def test_lidar_left_obstacle():
    """
    图像左侧（cx=160 < W/2=320）→ bearing>0 → 机器人左侧（obs_y > 0，+Y 方向）。
    验证 bearing 符号：W/2 - cx_px > 0 → 正方位角 → 左侧。
    """
    obs = lidar_to_odom(_FakeScan(2.0), 160, _W, _HFOV, 0.0, 0.0, 0.0)
    assert obs is not None
    _, obs_y, _ = obs
    assert obs_y > 0.1, f"左侧物体 obs_y 应为正值（+Y），实际 {obs_y:.3f}"


def test_lidar_right_obstacle():
    """
    图像右侧（cx=480 > W/2=320）→ bearing<0 → 机器人右侧（obs_y < 0，-Y 方向）。
    """
    obs = lidar_to_odom(_FakeScan(2.0), 480, _W, _HFOV, 0.0, 0.0, 0.0)
    assert obs is not None
    _, obs_y, _ = obs
    assert obs_y < -0.1, f"右侧物体 obs_y 应为负值（-Y），实际 {obs_y:.3f}"


def test_lidar_none_when_scan_none():
    """scan=None 时返回 None，不崩溃。"""
    assert lidar_to_odom(None, 320, _W, _HFOV, 0.0, 0.0, 0.0) is None


def test_lidar_none_when_no_valid_range():
    """所有光束无效时返回 None，不 fallback 到机器人位置（这是修复核心）。"""
    scan = _FakeScan(0.0)
    scan.ranges = [float('inf')] * 720
    assert lidar_to_odom(scan, 320, _W, _HFOV, 0.0, 0.0, 0.0) is None


def test_lidar_robot_pose_offset():
    """机器人在 (3, 4) 朝东，正前方物体坐标应正确平移。"""
    obs = lidar_to_odom(_FakeScan(2.0), 320, _W, _HFOV, 3.0, 4.0, 0.0)
    assert obs is not None
    obs_x, obs_y, _ = obs
    expected_x = 3.0 + LASER_FORWARD_M + 2.0 + OBSTACLE_HALF_DEPTH_M
    assert abs(obs_x - expected_x) < 0.05, f"obs_x 期望 {expected_x:.3f}，实际 {obs_x:.3f}"
    assert abs(obs_y - 4.0) < 0.05,        f"obs_y 期望 4.0，实际 {obs_y:.3f}"


def test_lidar_robot_heading_north():
    """
    机器人在原点朝北（yaw=π/2），图像正中物体应在 +Y 方向。
    obs_x ≈ 0，obs_y > 2.0。
    """
    obs = lidar_to_odom(_FakeScan(2.0), 320, _W, _HFOV, 0.0, 0.0, math.pi / 2)
    assert obs is not None
    obs_x, obs_y, _ = obs
    assert abs(obs_x) < 0.1, f"朝北时 obs_x 应接近 0，实际 {obs_x:.3f}"
    assert obs_y > 2.0,      f"朝北时 obs_y 应 > 2.0，实际 {obs_y:.3f}"


def test_lidar_half_depth_zero():
    """half_depth_m=0 时，obs_x = LASER_FORWARD_M + range（不加障碍物半深）。"""
    obs = lidar_to_odom(_FakeScan(2.0), 320, _W, _HFOV, 0.0, 0.0, 0.0, half_depth_m=0.0)
    assert obs is not None
    obs_x, _, _ = obs
    expected_x = LASER_FORWARD_M + 2.0
    assert abs(obs_x - expected_x) < 0.05, f"obs_x 期望 {expected_x:.3f}，实际 {obs_x:.3f}"


def test_lidar_beam_index_boundary():
    """方位角接近扫描边界（±110°）时仍能正常工作。"""
    r = scan_range_at_bearing(_FakeScan(1.0), math.radians(110))
    assert r is not None, "±110° 在 ±120° 范围内，应有有效测距"
