"""
camera_bringup.launch.py — 感知子系统启动（Member 2 节点）

架构说明
────────
  本文件只负责启动感知层的节点，不重复启动已在 sim_bringup 中运行的核心节点
  （robot_state_publisher / joint_state_publisher / state_manager / mapping_service 等）。
  由 sim_bringup.launch.py 通过 IncludeLaunchDescription 调用，参数由父 launch 透传。

节点列表
────────
  colour_detector      订阅 /oak/rgb/image_raw（remap→/camera），识别彩色障碍物（黄/红），
                       检测到后发布 /part3/perception/marker_event，保存带注解的 JPEG。
  greek_detector       订阅 /oak/rgb/image_raw（remap→/camera），用 ONNX 模型识别希腊字母，
                       检测到后发布 /part3/perception/marker_event，保存带注解的 JPEG。
                       greek_model_path 为空字符串时节点自动跳过推理（运行但不检测）。
  photo_logger         订阅 /part3/perception/marker_event，追加写入 manifest.csv，
                       并把注解照片复制到 artifacts/photos，供离线报告使用。
  perception_adapter   订阅 /part3/perception/marker_event，去重后发布
                       /part3/perception/markers (PoseArray)，供 waypoint_service 消费。

话题 remap 说明
────────────────
  检测器节点内部订阅 /oak/rgb/image_raw（真机 OAK-D 驱动话题）。
  仿真中图像由 ros_gz_bridge 桥接为 /camera，故在 launch 里做一次 remap：
    /oak/rgb/image_raw → /camera
  真机部署时去掉 remap 即可，无需修改节点代码。

存储路径（固定，无需命令行传参）
────────────────────────────────
  照片 / 存档目录 : <ws_root>/artifacts/photos
  ONNX 模型路径  : <pkg_share>/models/greek_letters.onnx
                   （由 setup.py data_files 从 resource/models/ 安装到 share/models/）
  ⚠️  artifacts/photos 目录由节点在首次运行时自动创建。
  ⚠️  模型不存在时 greek_detector 打印 warn 并跳过推理，不崩溃。

启动时序
────────
  本文件本身不设 Timer；调用方 sim_bringup 在 t=5s 后才 include 本文件，
  确保 ros_gz_bridge（2s）和 /camera/image 图像流已稳定。

Launch arguments（由 sim_bringup 透传，也可单独运行时直接传）
──────────────────────────────────────────────────────────
  detection_cooldown float    同一标签重复检测冷却秒数        (default: 5.0)
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

# ── 路径常量（模块级，import 时求值）─────────────────────────────────────────
# _PKG_SHARE = install/auto_nav_part3/share/auto_nav_part3
# 往上 4 级即工作空间根目录：share/auto_nav_part3 → share → auto_nav_part3 → install → ws_root
_PKG_SHARE    = get_package_share_directory('auto_nav_part3')
_WS_ROOT      = os.path.normpath(os.path.join(_PKG_SHARE, '..', '..', '..', '..'))
_PHOTO_DIR    = os.path.join(_WS_ROOT, 'artifacts', 'photos')
# artifact_dir 必须比 photo_dir 高一级；photo_logger 会在此目录下建 photos/ 子目录。
# 若两者相同会产生 artifacts/photos/photos/ 双层嵌套。
_ARTIFACT_DIR = os.path.join(_WS_ROOT, 'artifacts')
# 模型安装路径：setup.py 将 resource/models/*.onnx 安装到 share/auto_nav_part3/models/
_GREEK_MODEL  = os.path.join(_PKG_SHARE, 'models', 'greek_letters.onnx')


def generate_launch_description() -> LaunchDescription:

    # ── Launch arguments ─────────────────────────────────────────────────────
    # detection_cooldown 是唯一的运行时可调参数；
    # 由 sim_bringup 透传（固定为 '5.0'），也可命令行覆盖。
    # 路径类参数（photo_dir / artifact_dir / greek_model_path）均固定为模块级常量，
    # 无需从命令行传入，避免用户误拼路径。
    cooldown_arg = DeclareLaunchArgument(
        'detection_cooldown',
        default_value='5.0',
        description='Seconds before same label can be re-detected',
    )

    # ── colour_detector（彩色障碍物检测）────────────────────────────────────
    # 节点内部订阅 /oak/rgb/image_raw（OAK-D 真机话题），
    # 仿真中通过 remappings 重定向到 ros_gz_bridge 桥接出的 /camera。
    # 真机部署时去掉 remappings 即可，无需改节点代码。
    colour_detector = Node(
        package='auto_nav_part3',
        executable='colour_detector',
        name='colour_detector',
        output='screen',
        parameters=[{
            'use_sim_time':         True,           # 与仿真时钟同步
            'photo_dir':            _PHOTO_DIR,
            'detection_cooldown_s': LaunchConfiguration('detection_cooldown'),
            'min_area_px':          1500,           # 小于此面积视为噪点/边缘细条（仿真调参）
            'max_area_px':          80000,          # 大于此面积可能是背景
            'jpeg_quality':         90,
        }],
        # 仿真：/oak/rgb/image_raw → /camera（Gazebo bridge 输出话题）
        remappings=[('/oak/rgb/image_raw', '/camera/image')],
    )

    # ── greek_detector（希腊字母 ONNX 识别）─────────────────────────────────
    # 同上，/oak/rgb/image_raw remap 到仿真的 /camera。
    # greek_model_path 为空字符串时节点启动但跳过推理，不崩溃。
    greek_detector = Node(
        package='auto_nav_part3',
        executable='greek_detector',
        name='greek_detector',
        output='screen',
        parameters=[{
            'use_sim_time':         True,
            'greek_model_path':     _GREEK_MODEL,   # share/auto_nav_part3/models/ 下
            'photo_dir':            _PHOTO_DIR,
            'detection_cooldown_s': LaunchConfiguration('detection_cooldown'),
            'min_confidence':       0.8,            # ONNX 置信度阈值
            'jpeg_quality':         90,
        }],
        remappings=[('/oak/rgb/image_raw', '/camera/image')],
    )

    # ── photo_logger（检测结果持久化）──────────────────────────────────────
    # 订阅 /part3/perception/marker_event，追加写入 manifest.csv，并复制照片。
    photo_logger = Node(
        package='auto_nav_part3',
        executable='photo_logger',
        name='photo_logger',
        output='screen',
        parameters=[{
            'use_sim_time':  True,
            'artifact_dir':  _ARTIFACT_DIR,
            'manifest_name': 'manifest.csv',
            'copy_photos':   True,
        }],
    )

    # ── perception_adapter（marker 去重 + PoseArray 发布）──────────────────
    # 订阅 /part3/perception/marker_event，维护去重 marker 列表，
    # 发布 /part3/perception/markers (PoseArray)，供 waypoint_service (C_W.1) 消费。
    # dedup_radius_m：同一位置 1.0m 内的重复检测合并为一条记录。
    # waypoints_save_dir：用绝对路径（从 _WS_ROOT 计算），与 CWD 无关。
    _WAYPOINTS_DIR = os.path.join(_WS_ROOT, 'artifacts', 'waypoints')
    perception_adapter = Node(
        package='auto_nav_part3',
        executable='perception_adapter',
        name='perception_adapter',
        output='screen',
        parameters=[{
            'use_sim_time':       True,
            'dedup_radius_m':     1.0,                    # 去重距离阈值
            'publish_rate_hz':    2.0,                    # PoseArray 定期发布频率
            'map_frame':          'map',                  # 坐标输出帧
            'odom_frame':         'odom',                 # detector 输出坐标帧
            'waypoints_save_dir': _WAYPOINTS_DIR,         # 绝对路径，与 CWD 无关
        }],
    )

    return LaunchDescription([
        cooldown_arg,
        colour_detector,
        greek_detector,
        photo_logger,
        perception_adapter,
    ])
