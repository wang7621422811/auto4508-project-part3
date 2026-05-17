"""
nav2_bringup.launch.py — Nav2 导航栈启动文件 (M3.C3.1 + C3.2)

通过 nav2_bringup/navigation_launch.py 启动完整 Nav2 栈，
传入我们为 Pioneer 3-AT 定制的 nav2_params.yaml。

节点列表（由 navigation_launch.py 内的 lifecycle_manager 管理）：
  controller_server   MPPI 控制器，DiffDrive 运动模型
  smoother_server     路径平滑（SimpleSmoother）
  planner_server      NavFn A* 全局规划
  route_server        路由（暂未使用图文件）
  behavior_server     恢复行为（spin / backup / wait）
  velocity_smoother   速度平滑
  collision_monitor   碰撞前停止
  bt_navigator        行为树导航（响应 /goal_pose 和 /navigate_to_pose action）
  waypoint_follower   路点序列跟随
  docking_server      对接（暂未配置对接点）

cmd_vel 链路：
  controller_server → cmd_vel_nav
  → velocity_smoother → cmd_vel_smoothed
  → collision_monitor → /cmd_vel → Gazebo / Pioneer

Launch arguments:
  use_sim_time    true/false  使用仿真时钟（default: true）

依赖前提（M1 + M2 必须先起）：
  - EKF 已发布 odom→base_link TF 和 /odometry/filtered
  - slam_toolbox 已 activate，正在发布 /map 和 map→odom TF
  - ros_gz_bridge 已桥接 /scan
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    pkg = get_package_share_directory('auto_nav_part3')
    nav2_bringup_dir = get_package_share_directory('nav2_bringup')

    nav2_params_path = os.path.join(pkg, 'config', 'nav2_params.yaml')

    use_sim_time = LaunchConfiguration('use_sim_time')

    args = [
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Use simulation (Gazebo) clock if true',
        ),
    ]

    nav2_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(nav2_bringup_dir, 'launch', 'navigation_launch.py')
        ),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'params_file': nav2_params_path,
            'autostart': 'true',
        }.items(),
    )

    return LaunchDescription([*args, nav2_launch])
