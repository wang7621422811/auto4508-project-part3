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
  7.  slam_toolbox             在线异步建图（生命周期节点，10 s 后启动）
  7a. slam configure/activate  bash 重试循环（10.5s 后）→ configure+activate
  M3. nav2_bringup.launch.py   Nav2 导航栈（15 s 后，use_nav2:=true 时启用）
  M4. exploration_node         Frontier 自主探索（45 s 后，use_exploration:=true 时启用）
  8.  rviz2                    可视化（可选）

EKF 定位说明 (M1.C1.1)
────────────────────────
  EKF 节点接管 odom→base_link 的 TF 发布权。
  URDF 里 diff-drive 的 <tf_topic> 已改为不桥接的内部话题，
  所以 /tf 中只有 EKF 这一个来源，不会出现双源冲突。

SLAM 建图说明 (M2.C2.1)
────────────────────────
  slam_toolbox 订阅 /scan + TF odom→base_link，发布 /map 和 map→odom TF。
  完整 TF 链：map → odom → base_link → {laser_frame, imu_link, ...}
  ⚠️  async_slam_toolbox_node 在 Jazzy 中是生命周期节点（Lifecycle Node）：
      启动后处于 unconfigured 状态，此时无任何订阅/发布。
      必须显式调用 configure(11.5s) → activate(12.5s) 才能开始建图。

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
  use_slam      true/false   Run slam_toolbox mapping   (default: true)
  use_nav2      true/false   Run Nav2 navigation stack  (default: false)
  use_exploration true/false Run frontier exploration   (default: false)
  use_camera    true/false   Launch perception nodes    (default: false)
  use_safety    true/false   Run safety_monitor + twist_mux (default: true)
  use_recording true/false   Full-session rosbag2 recording (C_S.3/T9)  (default: false)
  world         path         Override world SDF file
  x/y/z         float        Spawn position             (default: -3.0 0 0.18)
  gz_verbose    0-4          Gz server verbosity        (default: 3)


./scripts/launch.sh start --clean sim_bringup use_nav2:=true use_exploration:=true \
      use_slam:=true use_rviz:=true use_safety:=true

"""

import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    IncludeLaunchDescription,
    OpaqueFunction,
    SetEnvironmentVariable,
    TimerAction,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
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
    # SLAM 配置文件路径
    slam_config_path = os.path.join(pkg, 'config', 'slam_toolbox.yaml')
    # Nav2 launch 文件路径（M3.C3.1）
    nav2_launch_path = os.path.join(pkg, 'launch', 'nav2_bringup.launch.py')
    # 感知子系统 launch 文件路径（Member 2: colour_detector / greek_detector / photo_logger）
    camera_bringup_launch_path = os.path.join(pkg, 'launch', 'camera_bringup.launch.py')

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
            'use_slam', default_value='true',
            description='Launch slam_toolbox online async mapping. Set false to skip SLAM (e.g. when loading a pre-built map).',
        ),
        DeclareLaunchArgument(
            'world', default_value=default_world,
            description='Absolute path to the Gazebo SDF world file.',
        ),
        DeclareLaunchArgument('x', default_value='-3.0',  description='Spawn X (m)'),
        DeclareLaunchArgument('y', default_value='0.0',   description='Spawn Y (m)'),
        DeclareLaunchArgument('z', default_value='0.18',  description='Spawn Z (m)'),
        DeclareLaunchArgument(
            'gz_verbose', default_value='3',
            description='Gazebo server log verbosity (0 = silent, 4 = debug).',
        ),
        DeclareLaunchArgument(
            'use_nav2', default_value='false',
            description='Launch Nav2 navigation stack (M3). Requires use_slam=true and SLAM to be active first.',
        ),
        DeclareLaunchArgument(
            'use_exploration', default_value='false',
            description='Launch frontier exploration node (M4). Requires use_nav2=true.',
        ),
        DeclareLaunchArgument(
            'use_camera', default_value='false',
            description='Launch perception nodes (colour_detector / greek_detector / photo_logger). '
                        'Requires /camera image bridge (auto-started at t=2s).',
        ),
        DeclareLaunchArgument(
            'use_safety', default_value='true',
            description='Launch safety_monitor (moving-obstacle estop) + twist_mux (C_S.1). '
                        'twist_mux arbitrates /cmd_vel_safety (priority=100) vs /cmd_vel_nav2 (10).',
        ),
        DeclareLaunchArgument(
            'use_recording', default_value='false',
            description='C_S.3 (T9)：全程 rosbag2 录包，保存到 artifacts/bags/session_<timestamp>/。'
                        '录制关键话题（/scan /odometry/filtered /tf /tf_static /cmd_vel /map 等），'
                        '不录 /camera（体积过大）。',
        ),
    ]

    use_gz_gui    = LaunchConfiguration('use_gz_gui')
    use_rviz      = LaunchConfiguration('use_rviz')
    use_slam      = LaunchConfiguration('use_slam')
    use_nav2      = LaunchConfiguration('use_nav2')
    use_exploration = LaunchConfiguration('use_exploration')
    use_camera    = LaunchConfiguration('use_camera')
    use_safety    = LaunchConfiguration('use_safety')
    use_recording = LaunchConfiguration('use_recording')
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

    # ── 4a. CameraInfo 发布器 ──────────────────────────────────────────────
    # RViz 的 Camera 显示需要同时订阅 Image 和 CameraInfo 话题。
    # Gazebo 桥接只转发 /camera（图像），不生成 CameraInfo 标定消息。
    # RViz 从 /camera 自动推导 CameraInfo 话题为 /camera_info，
    # 本节点发布 sensor_msgs/CameraInfo 到 /camera_info 以匹配。
    #
    # 本节点从 Pioneer URDF 相机参数（640×480, HFOV=1.089 rad）计算内参矩阵 K，
    # 使 RViz 能正确渲染相机画面。
    camera_info_publisher = TimerAction(
        period=2.5,
        actions=[
            Node(
                package='auto_nav_part3',
                executable='camera_info_publisher',
                name='camera_info_publisher',
                output='screen',
                parameters=[{
                    'use_sim_time': True,
                    'width': 640,
                    'height': 480,
                    'horizontal_fov': 1.089,
                    'frame_id': 'cam_optical_link',
                    'publish_rate': 10.0,
                    'camera_info_topic': '/camera_info',
                }],
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

    # ── 7. slam_toolbox 在线建图节点 (M2.C2.1) ────────────────────────────────
    # async_slam_toolbox_node 在 Jazzy 中是生命周期节点（Lifecycle Node）。
    # 启动后处于 unconfigured 状态：仅有 /clock 订阅和生命周期管理服务，
    # 没有 /scan 订阅、/map 发布或 map→odom TF。
    # 必须显式执行 configure → activate 后才能开始建图（见下方 slam_configure/activate）。
    #
    # ⚠️  condition 必须放在 TimerAction 上，不能放在内部的 Node 上。
    #     原因：TimerAction 回调中 LaunchConfiguration 的 context 求值顺序
    #     与 launch 主流程不同，Node 上的 IfCondition 会静默失败（节点不启动）。
    slam_node = TimerAction(
        condition=IfCondition(use_slam),   # 控制整个定时器是否触发
        period=10.0,
        actions=[
            Node(
                package='slam_toolbox',
                executable='async_slam_toolbox_node',
                name='slam_toolbox',
                output='screen',
                parameters=[slam_config_path, {'use_sim_time': True}],
            ),
        ],
    )

    # ── 7a. slam_toolbox 生命周期激活 ─────────────────────────────────────────
    # 两步生命周期迁移：unconfigured → configure → inactive → activate → active
    #
    # 用单个 bash 进程串行执行，避免两个独立 ExecuteProcess 因子进程 DDS 发现
    # 竞态而失败。重试循环（最多 30×0.5s = 15s）等待 /slam_toolbox 进入 ROS 图。
    slam_lifecycle = TimerAction(
        condition=IfCondition(use_slam),
        period=10.5,   # slam_toolbox 启动（10s）后 0.5s 开始尝试
        actions=[
            ExecuteProcess(
                cmd=[
                    'bash', '-c',
                    # 轮询等待节点可被发现（DDS 发现需要时间），然后 configure + activate
                    'i=0; '
                    'until ros2 lifecycle set /slam_toolbox configure 2>/dev/null; do '
                    '  i=$((i+1)); '
                    '  [ $i -ge 30 ] && echo "[slam_lifecycle] timeout waiting for /slam_toolbox" && exit 1; '
                    '  sleep 0.5; '
                    'done && '
                    'echo "[slam_lifecycle] configure OK" && '
                    'sleep 0.5 && '
                    'ros2 lifecycle set /slam_toolbox activate && '
                    'echo "[slam_lifecycle] activate OK"',
                ],
                output='screen',
                name='slam_lifecycle',
            ),
        ],
    )

    # ── M3. Nav2 导航栈（可选）────────────────────────────────────────────────
    # 延迟 15s 启动：等待 SLAM configure+activate（~12-13s）完成并发布第一帧 /map 后，
    # Nav2 的全局代价地图才能正确初始化。
    # use_nav2=false（默认）时 TimerAction 不触发，不影响 M0–M2 工作流。
    nav2_node = TimerAction(
        condition=IfCondition(use_nav2),
        period=15.0,
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(nav2_launch_path),
                launch_arguments={'use_sim_time': 'true'}.items(),
            ),
        ],
    )

    # ── M4. exploration_node + map_manager（可选）─────────────────────────────
    # exploration_node：延迟 45s 启动。
    #   Nav2 在 15s 启动，但 ARM64/Parallels 上 lifecycle_manager 激活全部节点
    #   （controller / planner / bt_navigator 等共 10 个）需要约 20-30 秒，
    #   因此 exploration 必须在 Nav2 完全 active 之后才能访问 /navigate_to_pose。
    #   45s = 15s(Nav2 start) + 30s(lifecycle 激活余量)。
    # map_manager：与 exploration_node 同时启动，订阅 /part3/mapping/map_status，
    #   探索完成时自动保存地图。
    exploration_config_path = os.path.join(pkg, 'config', 'exploration.yaml')
    map_manager_config_path = os.path.join(pkg, 'config', 'map_manager.yaml')
    exploration_node = TimerAction(
        condition=IfCondition(use_exploration),
        period=45.0,
        actions=[
            Node(
                package='auto_nav_part3',
                executable='exploration_node',
                name='exploration_node',
                output='screen',
                parameters=[exploration_config_path, {'use_sim_time': True}],
            ),
        ],
    )

    # map_manager：C4.2 地图保存节点。
    # 与 exploration_node 同条件、同延迟启动，确保在探索开始前就在监听
    # /part3/mapping/map_status，不会错过 coverage=done 消息。
    map_manager_node = TimerAction(
        condition=IfCondition(use_exploration),
        period=45.0,
        actions=[
            Node(
                package='auto_nav_part3',
                executable='map_manager',
                name='map_manager',
                output='screen',
                parameters=[map_manager_config_path, {'use_sim_time': True}],
            ),
        ],
    )

    # ── M_P. 感知子系统（Member 2，可选）─────────────────────────────────────
    # colour_detector + greek_detector + photo_logger，由 camera_bringup.launch.py 统一管理。
    #
    # 延迟 5s 启动原因：
    #   /camera 话题由 ros_gz_bridge（2s）从 Gazebo 桥接进来，
    #   再经过 Pioneer 的相机插件初始化约需 1-2s 才能稳定出帧。
    #   5s = 2s(bridge) + 3s(相机流稳定余量)，确保节点订阅后立即有图像可处理。
    #
    # 路径/模型参数已在 camera_bringup.launch.py 内部固定（artifacts/photos、
    # artifacts/models/greek_letters.onnx），此处只透传 detection_cooldown。
    camera_node = TimerAction(
        condition=IfCondition(use_camera),
        period=5.0,
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(camera_bringup_launch_path),
                launch_arguments={
                    'detection_cooldown': '5.0',   # 同一标签重检测冷却，单位秒
                }.items(),
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

    # ── C_S.1. twist_mux — 速度指令仲裁器（始终启动）──────────────────────────
    # Nav2 collision_monitor 现在输出 /cmd_vel_nav2（非 /cmd_vel），必须经 twist_mux
    # 合并后输出 /cmd_vel，Gazebo bridge 才能收到。因此 twist_mux 无条件启动，
    # 不与 use_safety 绑定；safety_monitor 才是可选的。
    #
    # 优先级：safety_monitor /cmd_vel_safety (100) > nav2 /cmd_vel_nav2 (10) > teleop (5)
    # ⚠️  teleop_twist_keyboard 默认发 /cmd_vel；若需要遥控功能，应 remap 到 /cmd_vel_teleop。
    twist_mux_config_path = os.path.join(pkg, 'config', 'twist_mux.yaml')
    twist_mux_node = Node(
        package='twist_mux',
        executable='twist_mux',
        name='twist_mux',
        output='screen',
        parameters=[twist_mux_config_path, {'use_sim_time': True}],
        # twist_mux 默认发布 cmd_vel_out，remap 到 /cmd_vel（ros_gz_bridge 桥接的话题）
        remappings=[('cmd_vel_out', '/cmd_vel')],
    )

    # ── C_S.1. safety_monitor — 移动障碍检测 + 软件急停（可选）──────────────
    # 检测到靠近障碍后向 /cmd_vel_safety（priority=100）持续发零速，
    # twist_mux 自动覆盖 Nav2 的 cmd_vel 输出。
    # Nav2 goal 不被取消 → 障碍消失后探索/导航自动恢复，无需人工干预。
    safety_config_path = os.path.join(pkg, 'config', 'safety.yaml')
    safety_monitor_node = Node(
        condition=IfCondition(use_safety),
        package='auto_nav_part3',
        executable='safety_monitor',
        name='part3_safety_monitor',
        output='screen',
        parameters=[safety_config_path, {'use_sim_time': True}],
    )

    # ── C_S.2. rolling_recorder — 5 秒滚动录包（C_S.1 的伴生节点）──────────
    # 始终在内存中保留最近 5 秒的传感器数据。
    # 收到 /part3/safety/estop_event（由 safety_monitor 发布）时，
    # 立即把快照写成 rosbag2 文件到 artifacts/bags/estop_<ts>/，供离线回放。
    # use_safety 为 false 时无 estop_event，录包永远不会触发，故与 safety 同条件。
    rolling_recorder_node = Node(
        condition=IfCondition(use_safety),
        package='auto_nav_part3',
        executable='rolling_recorder',
        name='part3_rolling_recorder',
        output='screen',
        parameters=[safety_config_path, {'use_sim_time': True}],
    )

    # ── C_S.3. session_recorder — 全程录包（T9）──────────────────────────────
    # 对比 C_S.2（5s 滚动缓冲，急停时写盘），本节点记录完整会话。
    # 使用 ros2 bag record 进程；延迟 5s 等待 ros_gz_bridge（2s）+ 机器人生成（4s）就绪，
    # 确保桥接话题（/scan /odom /imu）已开始发布再开始录制。
    # 保存路径：artifacts/bags/session_<unix_timestamp>/（相对于 ros2 launch 工作目录）
    # /camera 已注释：单帧 640×480 RGB ≈ 1 MB，全程录制体积过大；需要时取消下方注释。
    session_recorder = TimerAction(
        condition=IfCondition(use_recording),
        period=5.0,
        actions=[
            ExecuteProcess(
                cmd=[
                    'bash', '-c',
                    'mkdir -p artifacts/bags && '
                    'ros2 bag record '
                    '-o artifacts/bags/session_$(date +%s) '
                    '/scan '
                    '/odometry/filtered '
                    '/tf '
                    '/tf_static '
                    '/cmd_vel '
                    '/map '
                    '/part3/safety/estop_event '
                    '/part3/system/state',
                    # '/camera '          # ← 取消此行注释即可录制相机（体积大）
                ],
                output='screen',
                name='session_recorder',
            ),
        ],
    )

    # ── M5. mapping_service（M5.C5.1）─────────────────────────────────────────
    # 编排节点：提供 /part3/mapping/start 服务，激活/停止 exploration_node。
    # 随仿真启动即可注册服务，等 UI 或命令行调用时再触发探索。
    mapping_service_node = Node(
        package='auto_nav_part3',
        executable='mapping_service',
        name='part3_mapping_service',
        output='screen',
        parameters=[{'use_sim_time': True}],
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
        camera_info_publisher,  # 4a. CameraInfo 标定发布器（2.5 s 后）
        spawn_robot,            # 5. 生成机器人（4 s 后）
        ekf_node,               # 6. EKF 融合定位，发布 odom→base_link TF（3 s 后）
        slam_node,              # 7.  SLAM 节点（10s 后，unconfigured 状态）
        slam_lifecycle,         # 7a. configure+activate 重试循环（10.5s 后开始）
        nav2_node,              # M3. Nav2 导航栈（15s 后，use_nav2:=true 时启用）
        exploration_node,       # M4. Frontier 探索（45s 后，use_exploration:=true 时启用）
        map_manager_node,       # M4. 地图保存管理器（45s 后，随 exploration 一起启动）
        twist_mux_node,          # C_S.1. 速度仲裁（始终启动，路由 /cmd_vel_nav2 → /cmd_vel）
        safety_monitor_node,     # C_S.1. 移动障碍急停（use_safety:=true 时启用）
        rolling_recorder_node,   # C_S.2. 5s 滚动录包，急停时写盘（use_safety:=true 时启用）
        session_recorder,        # C_S.3. 全程录包，保存到 artifacts/bags/session_<ts>/（use_recording:=true 时启用）
        mapping_service_node,   # M5. 编排服务，提供 /part3/mapping/start
        camera_node,            # M_P. 感知节点（5s 后，use_camera:=true 时启用）
        rviz2,                  # 8.  可视化
    ])
