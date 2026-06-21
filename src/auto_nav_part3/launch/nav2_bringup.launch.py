"""
nav2_bringup.launch.py — Nav2 navigation stack launch file (M3.C3.1 + C3.2)

Starts the full Nav2 stack via nav2_bringup/navigation_launch.py, passing
the nav2_params.yaml customised for the Pioneer 3-AT.

Nodes managed by lifecycle_manager inside navigation_launch.py:
  controller_server   MPPI controller, DiffDrive motion model
  smoother_server     path smoothing (SimpleSmoother)
  planner_server      NavFn A* global planner
  route_server        route server (graph file not used)
  behavior_server     recovery behaviours (spin / backup / wait)
  velocity_smoother   velocity smoothing
  collision_monitor   stop-before-collision
  bt_navigator        behaviour-tree navigator (handles /goal_pose and /navigate_to_pose action)
  waypoint_follower   waypoint sequence follower
  docking_server      docking server (no docking points configured)

cmd_vel pipeline:
  controller_server → cmd_vel_nav
  → velocity_smoother → cmd_vel_smoothed
  → collision_monitor → /cmd_vel → Gazebo / Pioneer

Launch arguments:
  use_sim_time    true/false  use simulation clock (default: true)

Prerequisites (M1 + M2 must be up first):
  - EKF has published odom→base_link TF and /odometry/filtered
  - slam_toolbox is activated and publishing /map and map→odom TF
  - ros_gz_bridge has bridged /scan
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
