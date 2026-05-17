"""
sim_bringup.launch.py — Full Gazebo simulation bringup for the Pioneer 3-AT.

架构说明 (Architecture)
────────────────────────
  Gazebo server (-s) 和 GUI (-g) 作为独立进程启动：GUI 崩溃不会连带杀死
  仿真物理引擎或 ROS 节点（Parallels/ARM 环境 GUI 崩溃较常见）。

启动顺序 (Startup sequence)
────────────────────────────
  1.  gz sim -s <world>        Gazebo 物理服务器（无界面）
  2.  gz sim -g                Gazebo GUI（1 s 后，可选）
  3.  robot_state_publisher    URDF → /robot_description + 静态 TF
  3a. joint_state_publisher    轮子关节零状态（让 RSP 能立即算出 TF）
  4.  ros_gz_bridge            Gz ↔ ROS2 话题桥（2 s 后）
  5.  ros_gz_sim create        在仿真中生成机器人（4 s 后）
  6.  ekf_filter_node          robot_localization EKF：odom+IMU 融合（3 s 后）
  7.  rviz2                    可视化（可选）

EKF 定位说明 (M1.C1.1)
────────────────────────
  EKF 节点接管 odom→base_link 的 TF 发布权。
  URDF 里 diff-drive 的 <tf_topic> 已改为不桥接的内部话题，
  所以 /tf 中只有 EKF 这一个来源，不会出现双源冲突。

ros_gz_bridge 话题表
─────────────────────
  Gz topic       Direction   ROS2 topic          ROS2 类型
  /clock         Gz → ROS    /clock              rosgraph_msgs/Clock
  /cmd_vel       ROS → Gz    /cmd_vel            geometry_msgs/Twist
  /odom          Gz → ROS    /odom               nav_msgs/Odometry
  /joint_states  Gz → ROS    /joint_states       sensor_msgs/JointState
  /imu           Gz → ROS    /imu                sensor_msgs/Imu
  /scan          Gz → ROS    /scan               sensor_msgs/LaserScan
  /camera        Gz → ROS    /camera             sensor_msgs/Image
  注：/tf 已从桥接表移除。所有 TF 均由 ROS 侧节点发布：
      静态 TF（base_link→laser_frame 等）由 robot_state_publisher 发布到 /tf_static
      动态 TF（odom→base_link）由 EKF 节点发布到 /tf

Launch arguments
────────────────
  use_gz_gui    true/false   Show Gazebo GUI            (default: true)
  use_rviz      true/false   Show RViz2                 (default: true)
  world         path         Override world SDF file
  x/y/z         float        Spawn position             (default: 0 0 0.18)
  gz_verbose    0-4          Gz server verbosity        (default: 3)
"""

import os

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
    # ── 包路径 (Package paths) ───────────────────────────────────────────────
    pkg = get_package_share_directory('auto_nav_part3')

    default_world  = os.path.join(pkg, 'simulation', 'worlds', 'discovery_15x15.sdf')
    simulation_dir = os.path.join(pkg, 'simulation')   # contains meshes/
    # EKF 配置文件路径（安装到 share/auto_nav_part3/config/）
    ekf_config_path = os.path.join(pkg, 'config', 'ekf.yaml')

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
    #
    #   use_sim_time=True：统一使用 Gazebo 仿真时间。
    #   原因：ros_gz_bridge 桥接的 /odom、/imu、/joint_states 消息的
    #   header.stamp 都是仿真时间（从 0 开始），若这些节点用墙上时间
    #   (~1.7×10⁹ s) 而传感器数据用仿真时间（~3 s），EKF 计算出
    #   负 dt ≈ -1.77×10⁹ s，滤波器数值爆炸，TF 发布中断。
    #   设置 use_sim_time=True 后，所有节点共用 /clock 仿真时钟，
    #   TF 树中所有时间戳一致，SLAM/Nav2 的 TF 查询才能正确工作。
    joint_state_publisher = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        parameters=[{
            'robot_description': ParameterValue(
                Command(['cat ', robot_description_path]),
                value_type=str,
            ),
            'use_sim_time': True,
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
            'use_sim_time': True,    # 与传感器消息时间戳保持一致（见 JSP 注释）
        }],
        output='screen',
    )

    # ── 4. ros_gz_bridge — delayed 2 s (wait for Gz server /clock) ──────────
    # 注意：/tf 已从桥接表中移除。
    #   原因：URDF 里 diff-drive 的 <tf_topic> 已改为 /gz/tf_not_bridged，
    #   Gazebo 不再向 ROS /tf 发布 odom→base_link；该 TF 由 EKF 节点接管。
    #   所有静态 TF（base_link→laser_frame 等）由 robot_state_publisher 发布
    #   到 /tf_static，无需从 Gazebo 桥接。
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
                    # /tf 已删除：EKF 节点负责发布 odom→base_link TF
                    '/joint_states@sensor_msgs/msg/JointState[gz.msgs.Model',
                    '/imu@sensor_msgs/msg/Imu[gz.msgs.IMU',
                    '/scan@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan',
                    '/camera@sensor_msgs/msg/Image[gz.msgs.Image',
                ],
                output='screen',
            ),
        ],
    )

    # ── 6. EKF 定位节点 (M1.C1.1) ─────────────────────────────────────────
    # robot_localization 的 EKF 节点：融合 /odom + /imu → /odometry/filtered
    # 并发布 odom→base_link TF（publish_tf: true 在 ekf.yaml 中配置）。
    #
    # 延迟 3 s 启动原因：
    #   /odom 和 /imu 由 ros_gz_bridge（2 s 后）桥接进来，
    #   EKF 需要等桥接就绪。robot_localization 能处理启动时短暂无数据的情况，
    #   只会用纯运动模型预测，不会崩溃，但延迟启动更干净。
    ekf_node = TimerAction(
        period=3.0,
        actions=[
            Node(
                package='robot_localization',
                executable='ekf_node',
                name='ekf_filter_node',
                output='screen',
                # ekf_config_path 提供 EKF 参数，{'use_sim_time': True} 覆盖时钟设置。
                # use_sim_time 必须为 True：否则 EKF 内部时钟用墙上时间(~1.7×10⁹s)，
                # 而 /odom、/imu 的时间戳是仿真时间(~3s)，EKF 认为数据来自
                # "56年前"，强制丢弃所有测量值，滤波器退化为零速纯预测然后崩溃。
                parameters=[ekf_config_path, {'use_sim_time': True}],
                remappings=[
                    ('odometry/filtered', '/odometry/filtered'),
                ],
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

    # ── 7. RViz2 (optional) ─────────────────────────────────────────────────
    rviz2 = Node(
        condition=IfCondition(use_rviz),
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config_path],
        parameters=[{'use_sim_time': True}],   # 与 TF/传感器时间戳一致
        output='screen',
    )

    return LaunchDescription([
        *args,
        set_gz_resource_path,
        gz_server,              # 1. Gazebo 物理服务器
        gz_gui,                 # 2. Gazebo GUI（1 s 后，可选）
        joint_state_publisher,  # 3a. 轮子关节零状态 → RSP 可立即计算 TF
        robot_state_publisher,  # 3b. URDF → 静态 TF 发布到 /tf_static
        bridge,                 # 4. Gz↔ROS 话题桥（2 s 后）
        spawn_robot,            # 5. 生成机器人（4 s 后）
        ekf_node,               # 6. EKF 融合定位，发布 odom→base_link TF（3 s 后）
        rviz2,                  # 7. 可视化
    ])
