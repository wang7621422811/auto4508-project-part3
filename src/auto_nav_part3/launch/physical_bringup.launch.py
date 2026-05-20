"""
physical_bringup.launch.py — Pioneer 3-AT 物理机完整启动文件 (M6.C6.4)

硬件列表（来自 ROBOT_MOTION_DEV_PLAN.md）：
  底盘 : Pioneer 3-AT + ARIA 软件驱动
  IMU  : Phidget Spatial 3/3/3
  相机 : OAK-D V2 (depthai)
  激光 : SICK TIMS7xx / Lakibeam

与 sim_bringup.launch.py 的核心差异：
  1. 无 Gazebo：不启动 gz sim server/GUI，无 ros_gz_bridge，无 spawn robot
  2. use_sim_time = False：所有节点使用系统墙上时钟（非仿真 /clock）
  3. 底层驱动替换：
       sim: Gazebo diff-drive 插件 → real: TODO ros2aria 驱动 (C6.1)
       sim: Gazebo gpu_lidar     → real: TODO sick_scan_xd 驱动 (C6.2)
       sim: Gazebo rgbd_camera   → real: oakd_camera 节点 (C6.3)
  4. 配置文件替换：
       ekf.yaml           → ekf_physical.yaml    (frequency 30Hz, 噪声参数调整)
       slam_toolbox.yaml  → slam_toolbox_physical.yaml (max_range 20m, TF 容差 0.2s)
       nav2_params.yaml   → nav2_params_physical.yaml  (vx_max 0.7m/s, wz_max 1.5rad/s)
       safety.yaml        → safety_physical.yaml  (consecutive_frames 5, cooldown 3s)
  5. 感知节点 camera_bringup 使用真机图像话题（/oak/rgb/image_raw），不做 remap
  6. 启动时序更紧凑：无 Gazebo 等待，驱动就绪即可启动上层节点

启动时序：
  t=0s    robot_state_publisher + joint_state_publisher（URDF → TF）
  t=1s    TODO: Aria 底盘驱动节点（/odom, /cmd_vel, joint_states）
  t=1s    TODO: SICK 激光驱动节点（/scan）
  t=1s    oakd_camera（/oak/rgb/image_raw, /oak/stereo/depth）
  t=2s    ekf_filter_node（odom + IMU → /odometry/filtered, odom→base_link TF）
  t=5s    slam_toolbox（/scan → /map, map→odom TF）
  t=5.5s  slam configure + activate（生命周期激活）
  t=10s   Nav2（use_nav2:=true 时启用）
  t=20s   exploration + map_manager（use_exploration:=true 时启用）
  t=3s    camera_bringup（感知节点，use_camera:=true 时启用）
  随时    mapping_service + waypoint_service（始终启动，等待 service 调用）
  随时    twist_mux + safety_monitor + rolling_recorder

启动命令（示例）：
  # 完整建图流程（Phase 1）
  ros2 launch auto_nav_part3 physical_bringup.launch.py \
      use_nav2:=true use_exploration:=true use_slam:=true \
      use_safety:=true use_camera:=true use_rviz:=true

  # 路点导航流程（Phase 2，已有地图）
  ros2 launch auto_nav_part3 physical_bringup.launch.py \
      use_nav2:=true use_localization:=true \
      use_safety:=true use_camera:=true use_rviz:=true

⚠️  首次上机注意事项：
  1. 先以低速测试：在 nav2_params_physical.yaml 将 vx_max 改为 0.3m/s
  2. 确认 /scan、/odom、/imu 三个话题正常发布后再启动 SLAM
  3. 急停测试：用手推障碍靠近机器人 1m 内，确认 /cmd_vel 变零速
  4. TODO 项必须在上机前全部解决（见下方各节点注释）
"""

import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    IncludeLaunchDescription,
    OpaqueFunction,
    TimerAction,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.conditions import IfCondition
from launch.substitutions import (
    Command,
    LaunchConfiguration,
    PathJoinSubstitution,
    PythonExpression,
)
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # ── 包路径 ───────────────────────────────────────────────────────────────
    pkg = get_package_share_directory('auto_nav_part3')
    # workspace 根目录（install/auto_nav_part3/share/auto_nav_part3 上溯 4 层）
    _WS_ROOT = os.path.normpath(os.path.join(pkg, '..', '..', '..', '..'))

    # ── 物理机专用配置文件路径 ───────────────────────────────────────────────
    # 仿真版使用 ekf.yaml / slam_toolbox.yaml / nav2_params.yaml / safety.yaml
    # 物理机使用对应的 *_physical.yaml，内容说明见各配置文件头部注释
    ekf_config_path             = os.path.join(pkg, 'config', 'ekf_physical.yaml')
    slam_config_path            = os.path.join(pkg, 'config', 'slam_toolbox_physical.yaml')
    slam_localization_config    = os.path.join(pkg, 'config', 'slam_toolbox_localization.yaml')
    nav2_launch_path            = os.path.join(pkg, 'launch', 'nav2_bringup.launch.py')
    camera_bringup_launch_path  = os.path.join(pkg, 'launch', 'camera_bringup.launch.py')
    exploration_config_path     = os.path.join(pkg, 'config', 'exploration.yaml')
    map_manager_config_path     = os.path.join(pkg, 'config', 'map_manager.yaml')
    twist_mux_config_path       = os.path.join(pkg, 'config', 'twist_mux.yaml')
    # TODO: 物理机安全参数（consecutive_frames=5, moving_delta=0.10m 等）
    safety_config_path          = os.path.join(pkg, 'config', 'safety_physical.yaml')
    waypoint_config_path        = os.path.join(pkg, 'config', 'waypoint.yaml')

    # 位姿图路径（Phase 2 定位模式，slam_toolbox 加载此文件定位）
    _POSEGRAPH_PATH = os.path.join(_WS_ROOT, 'artifacts', 'maps', 'discovery_map')
    # markers.json（waypoint_service 第二趟直接读取，无需感知节点在线）
    _MARKERS_JSON   = os.path.join(_WS_ROOT, 'artifacts', 'waypoints', 'markers.json')

    # URDF 路径（robot_state_publisher 用，与仿真版相同）
    robot_description_path = PathJoinSubstitution([
        FindPackageShare('auto_nav_part3'), 'urdf', 'pioneer.urdf',
    ])
    rviz_config_path = PathJoinSubstitution([
        FindPackageShare('auto_nav_part3'), 'rviz', 'pioneer.rviz',
    ])

    # ── Launch arguments ─────────────────────────────────────────────────────
    args = [
        DeclareLaunchArgument(
            'use_rviz', default_value='true',
            description='启动 RViz2 可视化。',
        ),
        DeclareLaunchArgument(
            'use_slam', default_value='true',
            description='启动 slam_toolbox 在线建图。false 时跳过（配合 use_localization）。',
        ),
        DeclareLaunchArgument(
            'use_nav2', default_value='false',
            description='启动 Nav2 导航栈（M3）。需要 use_slam=true 且 SLAM 已激活。',
        ),
        DeclareLaunchArgument(
            'use_exploration', default_value='false',
            description='启动 Frontier 自主探索（M4）。需要 use_nav2=true。',
        ),
        DeclareLaunchArgument(
            'use_camera', default_value='false',
            description='启动感知节点（colour_detector / greek_detector / photo_logger）。'
                        '需要 oakd_camera 节点正常发布 /oak/rgb/image_raw。',
        ),
        DeclareLaunchArgument(
            'use_safety', default_value='true',
            description='启动 safety_monitor（移动障碍急停）+ twist_mux（速度仲裁）。',
        ),
        DeclareLaunchArgument(
            'use_recording', default_value='false',
            description='全程 rosbag2 录包，保存到 artifacts/bags/session_<timestamp>/。',
        ),
        DeclareLaunchArgument(
            'use_localization', default_value='false',
            description='Phase 2 定位模式：加载 artifacts/maps/discovery_map.posegraph，'
                        '用 slam_toolbox localization 模式定位，不重新建图。'
                        '与 use_slam=true 互斥。',
        ),
        # TODO: 确认 ARIA 驱动串口设备，默认 /dev/ttyUSB0，实际可能是 /dev/ttyS0 或其他
        DeclareLaunchArgument(
            'aria_port', default_value='/dev/ttyUSB0',
            description='TODO: Aria/ros2aria 底盘驱动串口设备路径。'
                        '上机前用 ls /dev/tty* 确认实际设备名。',
        ),
        # TODO: 确认 SICK 雷达 IP 地址
        DeclareLaunchArgument(
            'lidar_ip', default_value='192.168.0.1',
            description='TODO: SICK TIMS7xx / Lakibeam 激光雷达 IP 地址。'
                        '上机前用 ping 确认实际地址。',
        ),
    ]

    use_rviz         = LaunchConfiguration('use_rviz')
    use_slam         = LaunchConfiguration('use_slam')
    use_nav2         = LaunchConfiguration('use_nav2')
    use_exploration  = LaunchConfiguration('use_exploration')
    use_camera       = LaunchConfiguration('use_camera')
    use_safety       = LaunchConfiguration('use_safety')
    use_recording    = LaunchConfiguration('use_recording')
    use_localization = LaunchConfiguration('use_localization')

    # ── 3a. joint_state_publisher ───────────────────────────────────────────
    # 真机场景下 joint_states 来自 ARIA 驱动（轮速反馈）。
    # 在 ARIA 驱动启动前，joint_state_publisher 发布零状态让 RSP 能立即计算 TF。
    # TODO: 若 ARIA 驱动已发布 joint_states，可删除此节点避免双源冲突。
    joint_state_publisher = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        parameters=[{
            'robot_description': ParameterValue(
                Command(['cat ', robot_description_path]),
                value_type=str,
            ),
            'use_sim_time': False,   # 真机使用系统时钟，不用仿真 /clock
        }],
        output='screen',
    )

    # ── 3b. robot_state_publisher ───────────────────────────────────────────
    # 与仿真版相同：解析 URDF，发布静态 TF（base_link → laser_frame / imu_link / cam_* 等）。
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        parameters=[{
            'robot_description': ParameterValue(
                Command(['cat ', robot_description_path]),
                value_type=str,
            ),
            'use_sim_time': False,   # 真机使用系统时钟
        }],
        output='screen',
    )

    # ── TODO: Aria 底盘驱动节点 (C6.1) ──────────────────────────────────────
    # 功能：订阅 /cmd_vel，驱动 Pioneer 3-AT 电机；发布 /odom（nav_msgs/Odometry）。
    # 安装方式（上机前执行）：
    #   git clone https://github.com/MobileRobots/ros2aria.git src/ros2aria
    #   colcon build --packages-select ros2aria
    # 关键配置：
    #   - 串口设备：通过 aria_port 参数（默认 /dev/ttyUSB0）
    #   - 关闭 ARIA 自身的 odom→base_link TF 发布（由 EKF 接管）
    #   - wheel_separation=0.394m, wheel_radius=0.111m（与 URDF 一致）
    # TODO: 上机前解注释以下代码，确认 ros2aria 包名和节点名正确。
    #
    # aria_driver = TimerAction(
    #     period=1.0,
    #     actions=[
    #         Node(
    #             package='ros2aria',                 # TODO: 确认包名
    #             executable='ros2aria',              # TODO: 确认可执行文件名
    #             name='aria_driver',
    #             output='screen',
    #             parameters=[{
    #                 'use_sim_time': False,
    #                 'port': LaunchConfiguration('aria_port'),
    #                 # TODO: 确认 ros2aria 关闭 TF 发布的参数名
    #                 # 'publish_tf': False,           # EKF 接管 odom→base_link TF
    #             }],
    #             remappings=[
    #                 # TODO: 若 ros2aria 发布到 /RosAria/pose，remap 到 /odom
    #                 # ('/RosAria/pose', '/odom'),
    #             ],
    #         ),
    #     ],
    # )

    # ── TODO: SICK 激光雷达驱动节点 (C6.2) ──────────────────────────────────
    # 功能：通过 TCP 连接激光雷达，发布 /scan（sensor_msgs/LaserScan）。
    # 安装方式（上机前执行）：
    #   git clone https://github.com/SICKAG/sick_scan_xd.git src/sick_scan_xd
    #   colcon build --packages-select sick_scan_xd
    # 关键配置：
    #   - lidar_ip 参数（通过 Launch argument 传入）
    #   - frame_id 必须与 URDF 中 laser_frame 一致
    #   - SICK TIMS7xx 使用 sick_tim_7xx.launch.xml；Lakibeam 使用对应 launch
    # TODO: 上机前解注释以下代码，确认 lidar_ip 和 launch 文件路径正确。
    #
    # sick_lidar = TimerAction(
    #     period=1.0,
    #     actions=[
    #         Node(
    #             package='sick_scan_xd',             # TODO: 确认包名
    #             executable='sick_generic_caller',   # TODO: 确认可执行文件名
    #             name='sick_scan',
    #             output='screen',
    #             parameters=[{
    #                 'use_sim_time': False,
    #                 'hostname': LaunchConfiguration('lidar_ip'),
    #                 'frame_id': 'laser_frame',      # 必须与 URDF 一致
    #                 'scanner_name': 'sick_tim_7xx', # TODO: 根据实际型号修改
    #             }],
    #             remappings=[
    #                 # sick_scan_xd 默认发布 /scan，与系统约定一致，通常无需 remap
    #             ],
    #         ),
    #     ],
    # )

    # ── OAK-D V2 相机节点 (C6.3) ────────────────────────────────────────────
    # 功能：用 depthai 驱动 OAK-D V2，发布 /oak/rgb/image_raw 和 /oak/stereo/depth。
    # 依赖：pip install depthai（在 ROS2 环境内安装）
    # 真机发布话题 /oak/rgb/image_raw，colour_detector/greek_detector 直接订阅，无需 remap。
    # C6.3 bug 修复：oakd_camera.py 已删除重复的第二份代码（见文件注释）。
    oakd_camera_node = TimerAction(
        period=1.0,
        actions=[
            Node(
                package='auto_nav_part3',
                executable='oakd_camera',
                name='oakd_camera',
                output='screen',
                parameters=[{
                    'use_sim_time': False,   # 真机使用系统时钟
                }],
            ),
        ],
    )

    # ── EKF 定位节点 (M1.C1.1) ──────────────────────────────────────────────
    # 真机配置（ekf_physical.yaml）差异：
    #   frequency=30Hz（仿真 15Hz，受 ARM VM 限制），噪声参数适配真实打滑特性。
    # TODO: 确认 /imu 话题名与 Phidget 驱动发布一致（见 ekf_physical.yaml 注释）。
    # TODO: 确认 /odom 话题名与 ARIA 驱动发布一致（见 ekf_physical.yaml 注释）。
    ekf_node = TimerAction(
        period=2.0,   # 比仿真 3s 更早：无需等 ros_gz_bridge，驱动 1s 后即就绪
        actions=[
            Node(
                package='robot_localization',
                executable='ekf_node',
                name='ekf_filter_node',
                output='screen',
                parameters=[ekf_config_path, {'use_sim_time': False}],
                remappings=[
                    ('odometry/filtered', '/odometry/filtered'),
                ],
            ),
        ],
    )

    # ── SLAM 建图节点 (M2.C2.1) ─────────────────────────────────────────────
    # 真机配置（slam_toolbox_physical.yaml）差异：
    #   max_laser_range=20m（适配 SICK 量程），TF 容差 0.2s（真机时钟精确）。
    # TODO: 确认 /scan frame_id 与 URDF laser_frame 名称一致（见 slam_toolbox_physical.yaml 注释）。
    slam_node = TimerAction(
        condition=IfCondition(PythonExpression(["'", use_slam, "' == 'true' and '", use_localization, "' != 'true'"])),
        period=5.0,   # 比仿真 10s 更早：无需等 Gazebo 物理引擎和 bridge 就绪
        actions=[
            Node(
                package='slam_toolbox',
                executable='async_slam_toolbox_node',
                name='slam_toolbox',
                output='screen',
                parameters=[slam_config_path, {'use_sim_time': False}],
            ),
        ],
    )

    # SLAM 生命周期激活（configure → activate）
    # 与仿真版相同的重试循环；真机 DDS 发现更快，通常 1-2 次就成功。
    slam_lifecycle = TimerAction(
        condition=IfCondition(PythonExpression(["'", use_slam, "' == 'true' and '", use_localization, "' != 'true'"])),
        period=5.5,   # slam_node 启动（5s）后 0.5s 开始尝试
        actions=[
            ExecuteProcess(
                cmd=[
                    'bash', '-c',
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

    # ── SLAM 定位模式（Phase 2）──────────────────────────────────────────────
    # 用于第二趟（已有地图），加载 Phase 1 序列化的位姿图，不重新建图。
    # use_localization:=true 时启动，与 use_slam:=true 互斥。
    slam_localization_node = TimerAction(
        condition=IfCondition(use_localization),
        period=5.0,
        actions=[
            Node(
                package='slam_toolbox',
                executable='async_slam_toolbox_node',
                name='slam_toolbox',
                output='screen',
                parameters=[
                    slam_localization_config,
                    {
                        'use_sim_time': False,
                        'map_file_name': _POSEGRAPH_PATH,
                    },
                ],
            ),
        ],
    )

    slam_localization_lifecycle = TimerAction(
        condition=IfCondition(use_localization),
        period=5.5,
        actions=[
            ExecuteProcess(
                cmd=[
                    'bash', '-c',
                    'i=0; '
                    'until ros2 lifecycle set /slam_toolbox configure 2>/dev/null; do '
                    '  i=$((i+1)); '
                    '  [ $i -ge 30 ] && echo "[slam_localization_lifecycle] timeout" && exit 1; '
                    '  sleep 0.5; '
                    'done && '
                    'echo "[slam_localization_lifecycle] configure OK" && '
                    'sleep 0.5 && '
                    'ros2 lifecycle set /slam_toolbox activate && '
                    'echo "[slam_localization_lifecycle] activate OK"',
                ],
                output='screen',
                name='slam_localization_lifecycle',
            ),
        ],
    )

    # ── Nav2 导航栈（M3）────────────────────────────────────────────────────
    # 真机配置（nav2_params_physical.yaml）差异：
    #   vx_max=0.7m/s，wz_max=1.5rad/s（真机安全速度），bond_timeout=10s。
    # 延迟 10s：等待 SLAM configure+activate（~6-7s）完成后再初始化代价地图。
    # 真机比 ARM VM 启动更快，Nav2 lifecycle 约 5s（仿真需 20-30s）。
    nav2_node = TimerAction(
        condition=IfCondition(use_nav2),
        period=10.0,   # 仿真版 15s；真机 SLAM 启动更快，10s 足够
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(nav2_launch_path),
                # TODO: nav2_bringup.launch.py 当前只接受 use_sim_time 参数，
                #       通过 nav2_params_physical.yaml 传入物理机参数。
                #       若 nav2_bringup 不支持自定义 params_file，需修改 nav2_bringup.launch.py。
                launch_arguments={
                    'use_sim_time': 'false',
                    'nav2_params_file': os.path.join(pkg, 'config', 'nav2_params_physical.yaml'),
                }.items(),
            ),
        ],
    )

    # ── Frontier 探索节点 (M4.C4.1) ─────────────────────────────────────────
    # 与仿真版使用相同的 exploration.yaml 参数（探索逻辑与硬件无关）。
    # 延迟 20s：真机 Nav2 lifecycle 激活约 5s，20s = 10s(Nav2 start) + 10s(lifecycle 余量)。
    # 仿真版需要 45s 是因为 ARM VM lifecycle 激活需要 20-30s。
    exploration_node = TimerAction(
        condition=IfCondition(use_exploration),
        period=20.0,   # 仿真版 45s；真机 Nav2 启动更快
        actions=[
            Node(
                package='auto_nav_part3',
                executable='exploration_node',
                name='exploration_node',
                output='screen',
                parameters=[exploration_config_path, {'use_sim_time': False}],
            ),
        ],
    )

    map_manager_node = TimerAction(
        condition=IfCondition(use_exploration),
        period=20.0,
        actions=[
            Node(
                package='auto_nav_part3',
                executable='map_manager',
                name='map_manager',
                output='screen',
                parameters=[map_manager_config_path, {'use_sim_time': False}],
            ),
        ],
    )

    # ── twist_mux 速度仲裁（始终启动）──────────────────────────────────────
    # 优先级：safety(/cmd_vel_safety=100) > nav2(/cmd_vel_nav2=10) > teleop(/cmd_vel_teleop=5)
    # 真机输出 /cmd_vel → Aria 驱动订阅（与仿真 ros_gz_bridge 订阅相同话题）。
    twist_mux_node = Node(
        package='twist_mux',
        executable='twist_mux',
        name='twist_mux',
        output='screen',
        parameters=[twist_mux_config_path, {'use_sim_time': False}],
        remappings=[('cmd_vel_out', '/cmd_vel')],
    )

    # ── safety_monitor 移动障碍急停（C_S.1）────────────────────────────────
    # 真机配置（safety_physical.yaml）差异：
    #   consecutive_frames=5（更保守），moving_delta=0.10m（SICK 精度更高）。
    safety_monitor_node = Node(
        condition=IfCondition(use_safety),
        package='auto_nav_part3',
        executable='safety_monitor',
        name='part3_safety_monitor',
        output='screen',
        parameters=[safety_config_path, {'use_sim_time': False}],
    )

    # ── rolling_recorder 5s 滚动录包（C_S.2）───────────────────────────────
    rolling_recorder_node = Node(
        condition=IfCondition(use_safety),
        package='auto_nav_part3',
        executable='rolling_recorder',
        name='part3_rolling_recorder',
        output='screen',
        parameters=[safety_config_path, {'use_sim_time': False}],
    )

    # ── session_recorder 全程录包（C_S.3/T9）────────────────────────────────
    # 真机录制话题与仿真版相同，但不录 /camera（已桥接为 /oak/rgb/image_raw）。
    # 延迟 3s 等待驱动和相机节点就绪后再开始录制。
    session_recorder = TimerAction(
        condition=IfCondition(use_recording),
        period=3.0,   # 仿真版 5s；真机驱动启动更快
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
                    '/part3/system/state '
                    '/oak/rgb/image_raw',   # 真机相机话题（仿真版不录，真机可按需录制）
                    # TODO: 如果 /oak/rgb/image_raw 体积过大可注释掉上一行
                ],
                output='screen',
                name='session_recorder',
            ),
        ],
    )

    # ── mapping_service 编排节点（M5.C5.1）──────────────────────────────────
    # 与仿真版相同：提供 /part3/mapping/start service，激活/停止 exploration_node。
    mapping_service_node = Node(
        package='auto_nav_part3',
        executable='mapping_service',
        name='part3_mapping_service',
        output='screen',
        parameters=[{'use_sim_time': False}],
    )

    # ── waypoint_service 路点驾驶（M_W.C_W.1）───────────────────────────────
    # 与仿真版相同：提供 /part3/waypoint/start service，TSP 路径规划 + Nav2 导航。
    waypoint_service_node = Node(
        package='auto_nav_part3',
        executable='waypoint_service',
        name='part3_waypoint_service',
        output='screen',
        parameters=[waypoint_config_path, {
            'use_sim_time': False,
            'waypoints_file': _MARKERS_JSON,
        }],
    )

    # ── 感知子系统（M_P，可选）──────────────────────────────────────────────
    # 真机模式：image_topic='/oak/rgb/image_raw'，节点内部订阅此话题，无需 remap。
    # use_sim_time='false'：感知节点使用系统时钟（真机时间戳）。
    # 延迟 3s：等待 oakd_camera（1s 启动）+ 相机流稳定（约 2s）。
    camera_node = TimerAction(
        condition=IfCondition(use_camera),
        period=3.0,   # 仿真版 5s；真机相机启动更快
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(camera_bringup_launch_path),
                launch_arguments={
                    'detection_cooldown': '5.0',
                    # 真机：不做 remap，节点直接订阅 /oak/rgb/image_raw
                    'image_topic': '/oak/rgb/image_raw',
                    # 真机使用系统时钟
                    'use_sim_time': 'false',
                }.items(),
            ),
        ],
    )

    # ── RViz2（可选）────────────────────────────────────────────────────────
    # 真机上 RViz 通常在开发机运行（通过网络连接机器人 ROS2 节点）。
    # 若在机器人本机运行 RViz，需要显示器或 ssh -X 连接。
    # TODO: 若机器人无显示器，使用 use_rviz:=false 并在开发机单独启动 rviz2。
    rviz2 = Node(
        condition=IfCondition(use_rviz),
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config_path],
        parameters=[{'use_sim_time': False}],   # 真机使用系统时钟
        output='screen',
    )

    return LaunchDescription([
        *args,
        # ── 基础层（始终启动）──────────────────────────────────────────────
        joint_state_publisher,      # URDF 关节零状态 → RSP 可立即计算 TF
        robot_state_publisher,      # URDF → 静态 TF（base_link→laser_frame 等）
        # ── TODO: 底层驱动（上机前解注释）──────────────────────────────────
        # aria_driver,              # C6.1: Pioneer 底盘驱动（/odom, /cmd_vel）
        # sick_lidar,               # C6.2: SICK 激光雷达驱动（/scan）
        # ── 相机驱动（始终启动）────────────────────────────────────────────
        oakd_camera_node,           # C6.3: OAK-D V2 驱动（/oak/rgb/image_raw）
        # ── 定位层 ──────────────────────────────────────────────────────────
        ekf_node,                   # M1: EKF 融合定位（odom+IMU → /odometry/filtered）
        # ── 建图层 ──────────────────────────────────────────────────────────
        slam_node,                       # M2: SLAM 建图（use_slam:=true）
        slam_lifecycle,                  # M2: SLAM configure+activate
        slam_localization_node,          # M2: SLAM 定位模式（use_localization:=true）
        slam_localization_lifecycle,     # M2: 定位模式 configure+activate
        # ── 导航层 ──────────────────────────────────────────────────────────
        nav2_node,                  # M3: Nav2 导航栈（use_nav2:=true）
        # ── 探索层 ──────────────────────────────────────────────────────────
        exploration_node,           # M4: Frontier 探索（use_exploration:=true）
        map_manager_node,           # M4: 地图保存（与 exploration 同条件）
        # ── 安全层（始终启动仲裁，safety_monitor 可选）──────────────────────
        twist_mux_node,             # C_S.1: 速度仲裁（始终启动）
        safety_monitor_node,        # C_S.1: 移动障碍急停（use_safety:=true）
        rolling_recorder_node,      # C_S.2: 5s 滚动录包（use_safety:=true）
        session_recorder,           # C_S.3: 全程录包（use_recording:=true）
        # ── 编排服务层（始终启动）──────────────────────────────────────────
        mapping_service_node,       # M5: /part3/mapping/start 服务
        waypoint_service_node,      # M_W: /part3/waypoint/start 服务
        # ── 感知层（可选）──────────────────────────────────────────────────
        camera_node,                # M_P: 感知节点（use_camera:=true）
        # ── 可视化（可选）──────────────────────────────────────────────────
        rviz2,
    ])
