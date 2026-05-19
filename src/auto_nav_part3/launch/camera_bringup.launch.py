"""
camera_bringup.launch.py — 感知子系统启动（Member 2 节点）

架构说明
────────
  本文件只负责启动感知层的三个节点，不重复启动已在 sim_bringup 中运行的核心节点
  （robot_state_publisher / joint_state_publisher / state_manager / mapping_service 等）。
  由 sim_bringup.launch.py 通过 IncludeLaunchDescription 调用，参数由父 launch 透传。

节点列表
────────
  colour_detector  订阅 /camera/image_raw，识别彩色障碍物（黄/红/蓝/绿），
                   检测到后发布 /part3/perception/marker_event，并保存带注解的 JPEG。
  greek_detector   订阅 /camera/image_raw，用 ONNX 模型识别希腊字母标记，
                   检测到后发布 /part3/perception/marker_event，并保存带注解的 JPEG。
                   greek_model_path 为空字符串时节点自动跳过推理（运行但不检测）。
  photo_logger     订阅 /part3/perception/marker_event，追加写入 manifest.csv，
                   并把注解照片复制到 artifacts/photos，供离线报告使用。

存储路径（固定，无需命令行传参）
────────────────────────────────
  照片 / 存档目录 : <ws_root>/artifacts/photos
  ONNX 模型路径  : <ws_root>/artifacts/models/greek_letters.onnx
  ⚠️  ws_root 由 get_package_share_directory 往上 4 级自动推算，无需硬编码绝对路径。
  ⚠️  artifacts/photos 目录由节点在首次运行时自动创建。
  ⚠️  greek_letters.onnx 须手动复制到 artifacts/models/；
      文件不存在时 greek_detector 启动后会打印 warn 并跳过推理，不崩溃。

启动时序
────────
  本文件本身不设 Timer；调用方 sim_bringup 在 t=5s 后才 include 本文件，
  确保 ros_gz_bridge（2s）和 /camera 图像流已稳定。

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
# get_package_share_directory 返回 install/auto_nav_part3/share/auto_nav_part3
# 往上 4 级即工作空间根目录（install/auto_nav_part3/share/auto_nav_part3 → 4 步 → ws_root）
_PKG_SHARE    = get_package_share_directory('auto_nav_part3')
_WS_ROOT      = os.path.normpath(os.path.join(_PKG_SHARE, '..', '..', '..', '..'))
_PHOTO_DIR    = os.path.join(_WS_ROOT, 'artifacts', 'photos')
_ARTIFACT_DIR = os.path.join(_WS_ROOT, 'artifacts', 'photos')
# 真机/仿真均从此处加载 ONNX；文件不存在则 greek_detector 禁用推理
_GREEK_MODEL  = os.path.join(_WS_ROOT, 'artifacts', 'models', 'greek_letters.onnx')


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
    # 订阅 /camera/image_raw（由 sim 的 ros_gz_bridge 桥接自 Gazebo /camera）。
    # 在 HSV 色彩空间做阈值分割，检测黄/红/蓝/绿区域，输出到 /part3/perception/marker_event。
    # min_area_px / max_area_px 过滤噪点和过大区域（墙面反光），根据仿真光照调整。
    # detection_cooldown_s：同一颜色在冷却期内不重复发布事件，防止高频刷屏。
    colour_detector = Node(
        package='auto_nav_part3',
        executable='colour_detector',
        name='colour_detector',
        output='screen',
        parameters=[{
            'use_sim_time':         True,           # 与仿真时钟同步，时间戳与 /camera 一致
            'photo_dir':            _PHOTO_DIR,
            'detection_cooldown_s': LaunchConfiguration('detection_cooldown'),
            'min_area_px':          300,            # 小于此像素面积视为噪点，忽略
            'max_area_px':          80000,          # 大于此像素面积可能是背景，忽略
            'jpeg_quality':         90,             # 保存注解照片的 JPEG 压缩质量
        }],
    )

    # ── greek_detector（希腊字母 ONNX 识别）─────────────────────────────────
    # 订阅 /camera/image_raw，对裁剪区域做 ONNX 推理，识别 α/β/γ/δ 等标记。
    # greek_model_path 为空字符串 → 节点启动但跳过推理（不报错），
    # 方便在仿真阶段不加载模型只测相机流。
    # min_confidence 过滤低置信度结果；0.5 是平衡误报/漏报的经验值，可根据模型调整。
    greek_detector = Node(
        package='auto_nav_part3',
        executable='greek_detector',
        name='greek_detector',
        output='screen',
        parameters=[{
            'use_sim_time':         True,           # 与仿真时钟同步
            'greek_model_path':     _GREEK_MODEL,
            'photo_dir':            _PHOTO_DIR,
            'detection_cooldown_s': LaunchConfiguration('detection_cooldown'),
            'min_confidence':       0.5,            # ONNX 置信度阈值，低于此值丢弃
            'jpeg_quality':         90,
        }],
    )

    # ── photo_logger（检测结果持久化）──────────────────────────────────────
    # 订阅 /part3/perception/marker_event（String），把照片复制到 artifacts/photos
    # 并追加写入 manifest.csv（字段：timestamp, type, label, x, y, photo_path）。
    # copy_photos=True：把注解照片一并复制到 artifact_dir，方便打包存档。
    # use_sim_time=True：确保 manifest 里的时间戳与仿真时间一致，便于与 bag 对齐。
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

    return LaunchDescription([
        # Launch arguments（父 launch 透传或命令行覆盖）
        cooldown_arg,
        # 感知节点（Member 2）
        colour_detector,
        greek_detector,
        photo_logger,
    ])
