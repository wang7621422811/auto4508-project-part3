"""
teleop.launch.py — keyboard teleoperation for Pioneer 3-AT.

Run this in a SEPARATE terminal from sim_bringup so the terminal
keeps keyboard focus and stdin is not shared with other nodes.

    Terminal 1:  ./scripts/launch.sh start sim_bringup
    Terminal 2:  ./scripts/launch.sh start teleop [--no-build]

Data flow:
    keyboard → /cmd_vel → ros_gz_bridge → Gazebo diff-drive
             → wheels turn → /odom + /tf (odom→base_link)
"""

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='auto_nav_part3',
            executable='teleop_keyboard',
            name='teleop_keyboard',
            output='screen',
            emulate_tty=True,   # keep colour and \r updates in the terminal
        ),
    ])
