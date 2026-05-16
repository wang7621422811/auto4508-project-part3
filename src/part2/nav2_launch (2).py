#!/usr/bin/env python3
"""
Pioneer 3-AT Nav2 启动 - 最终版
============================================

架构:
  rosaria2 → /odom (轮式里程计)
  Phidget IMU → /imu/data_raw + /imu/data (Madgwick 融合后)
  EKF → 融合 odom + IMU → /odometry/filtered + odom→base_link TF
  SICK Lidar → /scan (frame=laser_frame)
  Nav2 → 全部在 odom 坐标系工作 (不用 map, 不用 SLAM)

TF 树:
  odom → base_link (EKF 发)
    base_link → laser_frame (静态 TF)

Nav2 配置要求 (nav2_params.yaml):
  global_frame: odom
  所有 goal 的 frame_id: odom

避障: local_costmap 用 /scan 标记障碍物, 不依赖 map

用法:
  python3 /ros2_ws/src/part2w/nav2_launch.py
"""

import os
import subprocess
import signal
import sys
import time

processes = []


def start(name, cmd, delay=0):
    if delay > 0:
        time.sleep(delay)
    print(f'[启动] {name}')
    env = os.environ.copy()
    p = subprocess.Popen(
        cmd,
        shell=True,
        executable='/bin/bash',
        env=env,
        preexec_fn=os.setsid
    )
    processes.append((name, p))
    return p


def stop_all(sig=None, frame=None):
    print('\n[停止] 关闭所有节点...')
    for name, p in reversed(processes):
        try:
            os.killpg(os.getpgid(p.pid), signal.SIGTERM)
            print(f'  停止 {name}')
        except Exception:
            pass
    time.sleep(1)
    for name, p in reversed(processes):
        try:
            os.killpg(os.getpgid(p.pid), signal.SIGKILL)
        except Exception:
            pass
    sys.exit(0)


def ros_cmd(command: str, ws: str) -> str:
    return f'source {ws}/install/setup.bash && {command}'


def main():
    signal.signal(signal.SIGINT, stop_all)
    signal.signal(signal.SIGTERM, stop_all)

    ws = '/ros2_ws'
    part2w = f'{ws}/src/part2w'

    aria_lib = f'{ws}/src/AriaCoda/lib'
    if aria_lib not in os.environ.get('LD_LIBRARY_PATH', ''):
        os.environ['LD_LIBRARY_PATH'] = (
            aria_lib + ':' + os.environ.get('LD_LIBRARY_PATH', '')
        )

    print('=' * 60)
    print('  Pioneer 3-AT Nav2 启动 (纯 odom 模式)')
    print('=' * 60)

    # ========================================
    # 1. rosaria2 (底盘, 发 /odom, 收 /cmd_vel)
    # ========================================
    start(
        'rosaria2',
        ros_cmd(
            'ros2 run rosaria2 rosaria2_node '
            '--ros-args -p seria_port:=/dev/ttyUSB0',
            ws
        )
    )
    time.sleep(3)

    # ========================================
    # 2. SICK Lidar (发 /scan, 不发自己的 TF)
    # ========================================
    start(
        'lidar',
        ros_cmd(
            'ros2 run sick_scan_xd sick_generic_caller '
            '--ros-args '
            '-p scanner_type:=sick_tim_7xxS '
            '-p hostname:=192.168.0.1 '
            '-p frame_id:=laser_frame '
            '-p tf_publish_rate:=0.0 '
            '-r /sick_tim_7xxS/scan:=/scan',
            ws
        )
    )
    time.sleep(3)

    # ========================================
    # 3. Phidget IMU (发 /imu/data_raw + /imu/data)
    # /imu/data 由 imu_filter_madgwick 融合
    # ========================================
    start(
        'imu',
        ros_cmd(
            'ros2 launch phidgets_spatial spatial-launch.py',
            ws
        )
    )
    time.sleep(3)

    # ========================================
    # 4. OAK-D 摄像头
    # ========================================
    start(
        'camera',
        ros_cmd(
            f'python3 {part2w}/oakd_camera.py',
            ws
        )
    )
    time.sleep(2)

    # ========================================
    # 5. 手柄
    # ========================================
    start(
        'joy',
        ros_cmd(
            'ros2 run joy joy_node',
            ws
        )
    )
    time.sleep(1)

    start(
        'joy_controller',
        ros_cmd(
            f'python3 {part2w}/joy_controller.py',
            ws
        )
    )
    time.sleep(1)

    # ========================================
    # 6. 静态 TF: base_link → laser_frame
    # (Lidar 相对车体位置: x=0.2m, z=0.28m)
    # ========================================
    start(
        'laser_tf',
        ros_cmd(
            'ros2 run tf2_ros static_transform_publisher '
            '--frame-id base_link --child-frame-id laser_frame '
            '--x 0.2 --z 0.28',
            ws
        )
    )
    time.sleep(1)

    # ========================================
    # 7. EKF 融合 (发 odom → base_link TF)
    # /odom + /imu/data → /odometry/filtered
    # ========================================
    start(
        'ekf',
        ros_cmd(
            'ros2 run robot_localization ekf_node '
            f'--ros-args --params-file {part2w}/config/ekf.yaml',
            ws
        )
    )
    time.sleep(3)

    # ========================================
    # 8. Nav2 (全部在 odom 坐标系)
    # ========================================
    start(
        'nav2',
        ros_cmd(
            'ros2 launch nav2_bringup navigation_launch.py '
            'use_sim_time:=False '
            f'params_file:={part2w}/config/nav2_params.yaml',
            ws
        )
    )
    time.sleep(5)

    # ========================================
    # 9. 物体检测 (cone + obstacle)
    # ========================================
    start(
        'detector',
        ros_cmd(
            f'python3 {part2w}/object_detector.py',
            ws
        )
    )

    print()
    print('=' * 60)
    print('  所有节点已启动')
    print('=' * 60)
    print('  TF 树: odom → base_link → laser_frame')
    print('  EKF 融合: /odom + /imu/data → /odometry/filtered')
    print('  Nav2 工作坐标系: odom')
    print('  避障: local_costmap (实时 Lidar)')
    print()
    print('  手柄:')
    print('    X  键 = 自动模式')
    print('    O  键 = 手动模式')
    print('    方块 = 切换死人开关 (自动模式下启用 Nav2)')
    print('    L1  = 摇杆手动覆盖')
    print()
    print('  跑任务:')
    print('    python3 /ros2_ws/src/part2w/forward_10m.py      # 备用')
    print('    python3 /ros2_ws/src/part2w/gps_waypoint_runner.py  # GPS')
    print()
    print('  Ctrl+C 停止所有节点')
    print('=' * 60)

    try:
        while True:
            time.sleep(1)
            for name, p in processes:
                if p.poll() is not None:
                    print(f'[警告] {name} 已退出 (code={p.returncode})')
    except KeyboardInterrupt:
        stop_all()


if __name__ == '__main__':
    main()
