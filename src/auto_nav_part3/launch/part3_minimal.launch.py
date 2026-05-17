from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description() -> LaunchDescription:

    # ── launch arguments ─────────────────────────────────────────────────
    greek_model_arg = DeclareLaunchArgument(
        "greek_model_path",
        default_value="",
        description="Absolute path to greek_letters.onnx — leave empty to disable",
    )
    photo_dir_arg = DeclareLaunchArgument(
        "photo_dir",
        default_value="artifacts/perception_photos",
        description="Directory where nodes save annotated photos",
    )
    artifact_dir_arg = DeclareLaunchArgument(
        "artifact_dir",
        default_value="artifacts/markers",
        description="Directory for manifest CSV and copied photos",
    )
    cooldown_arg = DeclareLaunchArgument(
        "detection_cooldown",
        default_value="5.0",
        description="Seconds before same label can be re-detected",
    )

    # ── robot description ────────────────────────────────────────────────
    robot_description_path = PathJoinSubstitution([
        FindPackageShare("auto_nav_part3"), "urdf", "pioneer.urdf",
    ])

    # ── core nodes ───────────────────────────────────────────────────────
    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        parameters=[{
            "robot_description": Command(["cat ", robot_description_path])
        }],
    )
    joint_state_publisher = Node(
        package="joint_state_publisher",
        executable="joint_state_publisher",
        name="joint_state_publisher",
    )
    state_manager = Node(
        package="auto_nav_part3",
        executable="state_manager",
        output="screen",
    )
    mapping_service = Node(
        package="auto_nav_part3",
        executable="mapping_service",
        output="screen",
    )
    waypoint_service = Node(
        package="auto_nav_part3",
        executable="waypoint_service",
        output="screen",
    )
    safety_monitor = Node(
        package="auto_nav_part3",
        executable="safety_monitor",
        output="screen",
    )
    ui_status = Node(
        package="auto_nav_part3",
        executable="ui_status",
        output="screen",
    )

    # ── Member 2: Perception nodes ───────────────────────────────────────
    colour_detector = Node(
        package="auto_nav_part3",
        executable="colour_detector",
        output="screen",
        parameters=[{
            "photo_dir":            LaunchConfiguration("photo_dir"),
            "detection_cooldown_s": LaunchConfiguration("detection_cooldown"),
            "min_area_px":          300,
            "max_area_px":          80000,
            "jpeg_quality":         90,
        }],
    )
    greek_detector = Node(
        package="auto_nav_part3",
        executable="greek_detector",
        output="screen",
        parameters=[{
            "greek_model_path":     LaunchConfiguration("greek_model_path"),
            "photo_dir":            LaunchConfiguration("photo_dir"),
            "detection_cooldown_s": LaunchConfiguration("detection_cooldown"),
            "min_confidence":       0.5,
            "jpeg_quality":         90,
        }],
    )
    photo_logger = Node(
        package="auto_nav_part3",
        executable="photo_logger",
        output="screen",
        parameters=[{
            "artifact_dir":  LaunchConfiguration("artifact_dir"),
            "manifest_name": "manifest.csv",
            "copy_photos":   True,
        }],
    )

    return LaunchDescription([
        # Arguments
        greek_model_arg,
        photo_dir_arg,
        artifact_dir_arg,
        cooldown_arg,
        # Core nodes
        robot_state_publisher,
        joint_state_publisher,
        state_manager,
        mapping_service,
        waypoint_service,
        safety_monitor,
        ui_status,
        # Perception nodes
        colour_detector,
        greek_detector,
        photo_logger,
    ])