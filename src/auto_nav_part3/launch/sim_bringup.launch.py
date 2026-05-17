"""
sim_bringup.launch.py — Full Gazebo simulation bringup for the Pioneer 3-AT.

Architecture
────────────
  Gazebo server (-s) and GUI (-g) are launched as SEPARATE processes so that
  a GUI crash (common in VMs) never kills the physics simulation or ROS nodes.

Startup sequence
────────────────
  1.  gz sim -s <world>            Gazebo physics server  (no display needed)
  2.  gz sim -g                        Gazebo GUI (ogre2 default — supports .dae)
  3.  robot_state_publisher        URDF → /robot_description + static TF
  4.  ros_gz_bridge                Gz ↔ ROS2 topic bridge   (after 2 s)
  5.  ros_gz_sim create            Spawn robot              (after 4 s)
  6.  rviz2                        Visualisation            (optional)

Robot spawn
───────────
  Placed at (0, -6.5, 0.18) — bottom-centre of the 15×15 arena.
  Faces positive-Y (into the arena / "up" in bird's-eye view).
  Yaw = π/2 ≈ 1.5708 rad.

ros_gz_bridge topic table
─────────────────────────
  Gz topic       Direction   ROS2 topic          ROS2 type
  /clock         Gz → ROS    /clock              rosgraph_msgs/Clock
  /cmd_vel       ROS → Gz    /cmd_vel            geometry_msgs/Twist
  /odom          Gz → ROS    /odom               nav_msgs/Odometry
  /tf            Gz → ROS    /tf                 tf2_msgs/TFMessage
  /joint_states  Gz → ROS    /joint_states       sensor_msgs/JointState
  /imu           Gz → ROS    /imu                sensor_msgs/Imu
  /scan          Gz → ROS    /scan               sensor_msgs/LaserScan
  /camera        Gz → ROS    /camera             sensor_msgs/Image

Launch arguments
────────────────
  use_gz_gui    true/false   Show Gazebo GUI            (default: true)
  use_rviz      true/false   Show RViz2                 (default: true)
  world         path         Override world SDF file
  x/y/z         float        Spawn position             (default: 0 0 0.18)
  gz_verbose    0-4          Gz server verbosity        (default: 3)
"""

import os
import math

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    OpaqueFunction,
    SetEnvironmentVariable,
    TimerAction,
)
from launch.conditions import IfCondition
from launch.substitutions import (
    Command,
    LaunchConfiguration,
    PathJoinSubstitution,
)
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def _make_spawn_action(context, spawn_x, spawn_y, spawn_z):
    """
    Read the installed URDF, replace every package://auto_nav_part3 URI with
    the absolute file:// path, then spawn via 'ros_gz_sim create -string'.

    Why: gz-common5-graphics (Gazebo's mesh loader) does not understand
    package:// URIs.  Without this substitution every .dae mesh loads
    silently as 'missing' and only primitive-geometry links are visible.
    """
    pkg = get_package_share_directory('auto_nav_part3')
    urdf_path = os.path.join(pkg, 'urdf', 'pioneer.urdf')

    with open(urdf_path, 'r') as fh:
        urdf = fh.read()

    urdf = urdf.replace('package://auto_nav_part3', f'file://{pkg}')

    x = context.perform_substitution(spawn_x)
    y = context.perform_substitution(spawn_y)
    z = context.perform_substitution(spawn_z)

    return [
        TimerAction(
            period=4.0,
            actions=[
                ExecuteProcess(
                    cmd=[
                        'ros2', 'run', 'ros_gz_sim', 'create',
                        '-name', 'pioneer3at',
                        '-string', urdf,
                        '-x', x,
                        '-y', y,
                        '-z', z,
                        '-R', '0',
                        '-P', '0',
                        '-Y', '0',
                    ],
                    output='screen',
                    name='spawn_pioneer',
                ),
            ],
        ),
    ]


def generate_launch_description():
    # ── Package paths ───────────────────────────────────────────────────────
    pkg = get_package_share_directory('auto_nav_part3')

    default_world  = os.path.join(pkg, 'simulation', 'worlds', 'discovery_15x15.sdf')
    simulation_dir = os.path.join(pkg, 'simulation')   # contains meshes/

    # robot_description: same pattern as part3_minimal.launch.py (Command + cat)
    # so that FindPackageShare resolves at runtime, not import time.
    robot_description_path = PathJoinSubstitution([
        FindPackageShare('auto_nav_part3'), 'urdf', 'pioneer.urdf',
    ])
    rviz_config_path = PathJoinSubstitution([
        FindPackageShare('auto_nav_part3'), 'rviz', 'pioneer.rviz',
    ])

    # ── Launch arguments ────────────────────────────────────────────────────
    args = [
        DeclareLaunchArgument(
            'use_gz_gui', default_value='true',
            description='Show Gazebo GUI. Set false to run physics-only (no window).',
        ),
        DeclareLaunchArgument(
            'use_rviz', default_value='true',
            description='Launch RViz2.',
        ),
        DeclareLaunchArgument(
            'world', default_value=default_world,
            description='Absolute path to the Gazebo SDF world file.',
        ),
        DeclareLaunchArgument('x', default_value='-7.0',  description='Spawn X (m)'),
        DeclareLaunchArgument('y', default_value='0.0',   description='Spawn Y (m)'),
        DeclareLaunchArgument('z', default_value='0.18',  description='Spawn Z (m)'),
        DeclareLaunchArgument(
            'gz_verbose', default_value='3',
            description='Gazebo server log verbosity (0 = silent, 4 = debug).',
        ),
    ]

    use_gz_gui = LaunchConfiguration('use_gz_gui')
    use_rviz   = LaunchConfiguration('use_rviz')
    world      = LaunchConfiguration('world')
    spawn_x    = LaunchConfiguration('x')
    spawn_y    = LaunchConfiguration('y')
    spawn_z    = LaunchConfiguration('z')
    gz_verbose = LaunchConfiguration('gz_verbose')

    # ── Environment: let Gazebo server find package meshes ──────────────────
    existing_gz_path = os.environ.get('GZ_SIM_RESOURCE_PATH', '')
    gz_resource_path = (
        simulation_dir + ':' + existing_gz_path if existing_gz_path
        else simulation_dir
    )
    set_gz_resource_path = SetEnvironmentVariable(
        'GZ_SIM_RESOURCE_PATH', gz_resource_path
    )

    # ── 1. Gazebo physics server (server-only, no GUI, no display required) ─
    #   --headless-rendering: sensors (Camera, Lidar) render via EGL without a
    #   display — prevents SIGSEGV on ARM/VM when the robot is spawned with
    #   GPU-dependent sensor plugins.
    #   --render-engine-server ogre: use ogre (not ogre2) for server-side sensor
    #   rendering; ogre is more stable under software/Mesa GL on aarch64.
    gz_server = ExecuteProcess(
        cmd=[
            'gz', 'sim', '-s', '-r', world,
            '--headless-rendering',
            '--render-engine-server', 'ogre',
            '--verbose', gz_verbose,
        ],
        output='screen',
        name='gz_server',
        # NOT on_exit=Shutdown() — server exit must NOT cascade to ROS nodes
    )

    # ── 2. Gazebo GUI — delayed 1 s ──────────────────────────────────────────
    #   --render-engine ogre: ogre2 SIGSEGV on ARM/Parallels (OpenGL driver).
    #   Mesh (.dae) loading uses gz-common5-graphics internally, not ogre's own
    #   COLLADA plugin, so ogre1 still renders .dae files correctly once the
    #   URDF mesh URIs are resolved to file:// paths (done in _make_spawn_action).
    gz_gui = TimerAction(
        period=1.0,
        actions=[
            ExecuteProcess(
                condition=IfCondition(use_gz_gui),
                cmd=['gz', 'sim', '-g', '--render-engine', 'ogre'],
                output='screen',
                name='gz_gui',
                # GUI crash is isolated — does NOT kill other processes
            ),
        ],
    )

    # ── 3a. joint_state_publisher — publishes zero states for continuous joints
    #   (wheels) so robot_state_publisher can compute their TF transforms and
    #   RViz2 can render the full robot model, even before Gazebo sends real data.
    joint_state_publisher = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        parameters=[{
            'robot_description': ParameterValue(
                Command(['cat ', robot_description_path]),
                value_type=str,
            ),
        }],
        output='screen',
    )

    # ── 3. robot_state_publisher ────────────────────────────────────────────
    #   Uses Command(['cat', ...]) like part3_minimal.launch.py so the path
    #   is resolved at launch time, not at Python import time.
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        parameters=[{
            'robot_description': ParameterValue(
                Command(['cat ', robot_description_path]),
                value_type=str,
            ),
        }],
        output='screen',
    )

    # ── 4. ros_gz_bridge — delayed 2 s (wait for Gz server /clock) ──────────
    bridge = TimerAction(
        period=2.0,
        actions=[
            Node(
                package='ros_gz_bridge',
                executable='parameter_bridge',
                name='ros_gz_bridge',
                arguments=[
                    '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
                    '/cmd_vel@geometry_msgs/msg/Twist]gz.msgs.Twist',
                    '/odom@nav_msgs/msg/Odometry[gz.msgs.Odometry',
                    '/tf@tf2_msgs/msg/TFMessage[gz.msgs.Pose_V',
                    '/joint_states@sensor_msgs/msg/JointState[gz.msgs.Model',
                    '/imu@sensor_msgs/msg/Imu[gz.msgs.IMU',
                    '/scan@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan',
                    '/camera@sensor_msgs/msg/Image[gz.msgs.Image',
                ],
                output='screen',
            ),
        ],
    )

    # ── 5. Spawn robot — OpaqueFunction pre-processes URDF at launch time ────
    #   Replaces package://auto_nav_part3 → file:///absolute/path so that
    #   gz-common5-graphics can load .dae meshes without ROS package resolution.
    #   Delayed 4 s inside _make_spawn_action so world is fully loaded first.
    spawn_robot = OpaqueFunction(
        function=_make_spawn_action,
        args=[spawn_x, spawn_y, spawn_z],
    )

    # ── 6. RViz2 (optional) ─────────────────────────────────────────────────
    rviz2 = Node(
        condition=IfCondition(use_rviz),
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config_path],
        output='screen',
    )

    return LaunchDescription([
        *args,
        set_gz_resource_path,
        gz_server,              # 1. physics first
        gz_gui,                 # 2. GUI after 1 s  (optional, ogre renderer)
        joint_state_publisher,  # 3a. zero wheel states → RSP can compute TF
        robot_state_publisher,  # 3b. URDF → TF immediately
        bridge,                 # 4. Gz↔ROS bridge after 2 s
        spawn_robot,            # 5. spawn after 4 s
        rviz2,                  # 6. visualisation
    ])
