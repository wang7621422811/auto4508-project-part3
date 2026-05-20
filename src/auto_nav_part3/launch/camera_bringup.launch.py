"""
camera_bringup.launch.py — 感知子系统启动（Member 2 节点）

架构说明
────────
  本文件只负责启动感知层的节点，不重复启动已在 sim/physical_bringup 中运行的核心节点
  （robot_state_publisher / joint_state_publisher / state_manager / mapping_service 等）。
  由 sim_bringup.launch.py 或 physical_bringup.launch.py 通过 IncludeLaunchDescription 调用。

节点列表
────────
  colour_detector      订阅图像话题（由 image_topic 参数控制），识别彩色障碍物（黄/红），
                       检测到后发布 /part3/perception/marker_event，保存带注解的 JPEG。
  greek_detector       订阅图像话题（由 image_topic 参数控制），用 ONNX 模型识别希腊字母，
                       检测到后发布 /part3/perception/marker_event，保存带注解的 JPEG。
                       greek_model_path 为空字符串时节点自动跳过推理（运行但不检测）。
  photo_logger         订阅 /part3/perception/marker_event，追加写入 manifest.csv，
                       并把注解照片复制到 artifacts/photos，供离线报告使用。
  perception_adapter   订阅 /part3/perception/marker_event，去重后发布
                       /part3/perception/markers (PoseArray)，供 waypoint_service 消费。

话题 remap 说明（仿真 vs 真机）
────────────────────────────────
  节点内部订阅 /oak/rgb/image_raw（OAK-D 真机驱动话题名）。
  仿真：ros_gz_bridge 把图像桥接到 /camera/image，需要 remap：
          image_topic=/camera/image → /oak/rgb/image_raw remap 到 /camera/image
  真机：OAK-D 直接发 /oak/rgb/image_raw，无需 remap：
          image_topic=/oak/rgb/image_raw → remap 到自身（等同于无 remap）
  切换方式：父 launch 传入不同的 image_topic 参数即可，节点代码无需修改。

存储路径（固定，无需命令行传参）
────────────────────────────────
  照片 / 存档目录 : <ws_root>/artifacts/photos
  ONNX 模型路径  : <pkg_share>/models/greek_letters.onnx
                   （由 setup.py data_files 从 resource/models/ 安装到 share/models/）
  ⚠️  artifacts/photos 目录由节点在首次运行时自动创建。
  ⚠️  模型不存在时 greek_detector 打印 warn 并跳过推理，不崩溃。

启动时序
────────
  本文件本身不设 Timer；调用方在适当延迟后才 include 本文件：
    sim_bringup:      t=5s（等 ros_gz_bridge 2s + 图像流稳定 3s）
    physical_bringup: t=3s（等 oakd_camera 1s 启动 + 图像流稳定 2s）

Launch arguments（由父 launch 透传，也可单独运行时直接传）
──────────────────────────────────────────────────────────
  detection_cooldown  float   同一标签重复检测冷却秒数               (default: 5.0)
  use_sim_time        bool    仿真时钟=true，真机墙上时钟=false       (default: true)
  image_topic         string  图像话题：仿真=/camera/image，真机=/oak/rgb/image_raw
                                                                      (default: /camera/image)
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

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
    # detection_cooldown: 由父 launch 透传（固定为 '5.0'），也可命令行覆盖。
    # use_sim_time: 仿真=true（sim_bringup 透传），真机=false（physical_bringup 透传）。
    # image_topic: 仿真=/camera/image，真机=/oak/rgb/image_raw（父 launch 透传）。
    #   节点内部订阅 /oak/rgb/image_raw，通过 remap 对齐到实际话题：
    #     仿真: remap /oak/rgb/image_raw → /camera/image
    #     真机: remap /oak/rgb/image_raw → /oak/rgb/image_raw（自身，等同于无 remap）
    # 路径类参数（photo_dir / artifact_dir / greek_model_path）均固定为模块级常量。
    cooldown_arg = DeclareLaunchArgument(
        'detection_cooldown',
        default_value='5.0',
        description='Seconds before same label can be re-detected',
    )
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation clock (true=Gazebo sim, false=real robot). '
                    'sim_bringup 传 true，physical_bringup 传 false。',
    )
    image_topic_arg = DeclareLaunchArgument(
        'image_topic',
        default_value='/camera/image',
        description='图像话题目标：仿真=/camera/image（ros_gz_bridge 输出），'
                    '真机=/oak/rgb/image_raw（oakd_camera 节点输出）。'
                    '节点内订阅 /oak/rgb/image_raw，通过 remap 对齐到此话题。',
    )

    use_sim_time = LaunchConfiguration('use_sim_time')
    image_topic  = LaunchConfiguration('image_topic')

    # ── colour_detector（彩色障碍物检测）────────────────────────────────────
    # 节点内部订阅 /oak/rgb/image_raw（OAK-D 真机话题名）。
    # remappings 通过 image_topic 参数动态控制：
    #   仿真（image_topic=/camera/image）: remap → /camera/image（ros_gz_bridge 输出）
    #   真机（image_topic=/oak/rgb/image_raw）: remap → 自身，等同于无 remap
    # use_sim_time 由父 launch 透传（仿真=true，真机=false），不再硬编码。
    colour_detector = Node(
        package='auto_nav_part3',
        executable='colour_detector',
        name='colour_detector',
        output='screen',
        parameters=[{
            'use_sim_time':         use_sim_time,   # 由父 launch 透传，仿真=true / 真机=false
            'photo_dir':            _PHOTO_DIR,
            'detection_cooldown_s': LaunchConfiguration('detection_cooldown'),
            'min_area_px':          1500,
            'max_area_px':          80000,
            'jpeg_quality':         90,
        }],
        # image_topic 控制 remap 目标：
        #   仿真: /oak/rgb/image_raw → /camera/image（sim_bringup 透传）
        #   真机: /oak/rgb/image_raw → /oak/rgb/image_raw（physical_bringup 透传，等同无 remap）
        # TODO: 若真机测试中检测器收不到图像，检查 image_topic 参数是否正确透传。
        remappings=[('/oak/rgb/image_raw', image_topic)],
    )

    # ── greek_detector（希腊字母 ONNX 识别）─────────────────────────────────
    # 与 colour_detector 相同的 remap 策略。
    # greek_model_path 为空字符串时节点启动但跳过推理，不崩溃。
    greek_detector = Node(
        package='auto_nav_part3',
        executable='greek_detector',
        name='greek_detector',
        output='screen',
        parameters=[{
            'use_sim_time':         use_sim_time,   # 由父 launch 透传
            'greek_model_path':     _GREEK_MODEL,
            'photo_dir':            _PHOTO_DIR,
            'detection_cooldown_s': LaunchConfiguration('detection_cooldown'),
            'min_confidence':       0.8,
            'jpeg_quality':         90,
        }],
        remappings=[('/oak/rgb/image_raw', image_topic)],
    )

    # ── photo_logger（检测结果持久化）──────────────────────────────────────
    # 订阅 /part3/perception/marker_event，追加写入 manifest.csv，并复制照片。
    # use_sim_time 由父 launch 透传，与其他节点时钟一致。
    photo_logger = Node(
        package='auto_nav_part3',
        executable='photo_logger',
        name='photo_logger',
        output='screen',
        parameters=[{
            'use_sim_time':  use_sim_time,   # 由父 launch 透传
            'artifact_dir':  _ARTIFACT_DIR,
            'manifest_name': 'manifest.csv',
            'copy_photos':   True,
        }],
    )

    # ── perception_adapter（marker 去重 + PoseArray 发布）──────────────────
    # 订阅 /part3/perception/marker_event，维护去重 marker 列表，
    # 发布 /part3/perception/markers (PoseArray)，供 waypoint_service (C_W.1) 消费。
    # dedup_radius_m：2.0m 范围内同标签合并，覆盖雷达+odom 融合误差（实测最大 1.39m）。
    # min_confirm_count：count < N 且 confidence < 0.90 的条目视为待确认，不发布给下游，
    #   防止单帧误识别（模型误分类）污染路点规划，同时跨重启仍可累计观测次数。
    # waypoints_save_dir：用绝对路径（从 _WS_ROOT 计算），与 CWD 无关。
    _WAYPOINTS_DIR = os.path.join(_WS_ROOT, 'artifacts', 'waypoints')
    perception_adapter = Node(
        package='auto_nav_part3',
        executable='perception_adapter',
        name='perception_adapter',
        output='screen',
        parameters=[{
            'use_sim_time':       use_sim_time,   # 由父 launch 透传
            'dedup_radius_m':     2.0,
            'min_confirm_count':  2,
            'publish_rate_hz':    2.0,
            'map_frame':          'map',
            'odom_frame':         'odom',
            'waypoints_save_dir': _WAYPOINTS_DIR,
        }],
    )

    return LaunchDescription([
        cooldown_arg,
        colour_detector,
        greek_detector,
        photo_logger,
        perception_adapter,
    ])
