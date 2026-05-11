from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    use_robot_state_publisher = LaunchConfiguration('use_robot_state_publisher')
    robot_description_path = PathJoinSubstitution([
        FindPackageShare('auto_nav_part3'),
        'urdf',
        'pioneer.urdf',
    ])

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_robot_state_publisher',
            default_value='true',
            description='Start robot_state_publisher with the Pioneer URDF.',
        ),
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            parameters=[{'robot_description': Command(['cat ', robot_description_path])}],
        ),
        Node(package='joint_state_publisher', executable='joint_state_publisher', name='joint_state_publisher'),
        Node(package='auto_nav_part3', executable='state_manager', output='screen'),
        Node(package='auto_nav_part3', executable='mapping_service', output='screen'),
        Node(package='auto_nav_part3', executable='waypoint_service', output='screen'),
        Node(package='auto_nav_part3', executable='safety_monitor', output='screen'),
        Node(package='auto_nav_part3', executable='ui_status', output='screen'),
    ])
