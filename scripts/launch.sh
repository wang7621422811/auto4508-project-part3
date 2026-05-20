#!/usr/bin/env bash
# launch.sh — build, source, and run ROS2 launch files
#
# Usage:
#   ./scripts/launch.sh                        build only
#   ./scripts/launch.sh list                   list available launch files
#   ./scripts/launch.sh start <name> [args]    build then launch <name>.launch.py
#
# Options for 'start':
#   --no-build    skip colcon build step
#   --args "..."  extra launch arguments (can also be passed positionally after name)
#
# Examples:
#   ./scripts/launch.sh start part3_minimal
#   ./scripts/launch.sh start part3_minimal use_rviz:=false
#   ./scripts/launch.sh start part3_minimal --args "use_rviz:=false use_robot_state_publisher:=true"
#   ./scripts/launch.sh start part3_minimal --no-build

set -euo pipefail

# ── Constants ──────────────────────────────────────────────────────────────────
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WS_ROOT="$(cd "$SCRIPT_DIR/.." && pwd)"
SRC_DIR="$WS_ROOT/src"
INSTALL_SETUP="$WS_ROOT/install/setup.bash"
FASTDDS_PROFILE="$WS_ROOT/config/fastdds_no_shm.xml"

# Detect all packages in src/
PACKAGES=( $(ls "$SRC_DIR") )

# ── Colours ────────────────────────────────────────────────────────────────────
RED='\033[0;31m'; GREEN='\033[0;32m'; YELLOW='\033[1;33m'
CYAN='\033[0;36m'; BOLD='\033[1m'; RESET='\033[0m'

info()  { echo -e "${CYAN}[INFO]${RESET}  $*"; }
ok()    { echo -e "${GREEN}[OK]${RESET}    $*"; }
warn()  { echo -e "${YELLOW}[WARN]${RESET}  $*"; }
error() { echo -e "${RED}[ERROR]${RESET} $*" >&2; }
die()   { error "$*"; exit 1; }

# ── Help ───────────────────────────────────────────────────────────────────────
usage() {
    echo -e "${BOLD}Usage:${RESET}"
    echo -e "  $(basename "$0") [command] [options]"
    echo ""
    echo -e "${BOLD}Commands:${RESET}"
    echo -e "  ${CYAN}(none)${RESET}          Build the workspace (default)"
    echo -e "  ${CYAN}list${RESET}            List all available launch files"
    echo -e "  ${CYAN}start <name>${RESET}    Build and launch <name>.launch.py"
    echo ""
    echo -e "${BOLD}Options (for 'start'):${RESET}"
    echo -e "  ${CYAN}--clean${RESET}         Remove install/ and build/ before building"
    echo -e "  ${CYAN}--no-build${RESET}      Skip colcon build"
    echo -e "  ${CYAN}--args \"...\"${RESET}   Pass extra arguments to ros2 launch"
    echo ""
    echo -e "${BOLD}Examples:${RESET}"
    echo -e "  $(basename "$0")"
    echo -e "  $(basename "$0") list"
    echo -e "  $(basename "$0") start part3_minimal"
    echo -e "  $(basename "$0") start part3_minimal use_rviz:=false"
    echo -e "  $(basename "$0") start part3_minimal --args \"use_rviz:=false use_robot_state_publisher:=true\""
    echo -e "  $(basename "$0") start part3_minimal --no-build"
}

# ── Step 0: Clean ─────────────────────────────────────────────────────────────
do_clean() {
    echo ""
    echo -e "${BOLD}━━━ Clean ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━${RESET}"
    for dir in install build; do
        if [[ -d "$WS_ROOT/$dir" ]]; then
            info "Removing $WS_ROOT/$dir"
            rm -rf "$WS_ROOT/$dir"
        fi
    done
    ok "Clean done."
}

# ── Step 1: Build ──────────────────────────────────────────────────────────────
do_build() {
    echo ""
    echo -e "${BOLD}━━━ Build ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━${RESET}"
    info "Running colcon build in $WS_ROOT"
    cd "$WS_ROOT"
    if colcon build --symlink-install 2>&1; then
        ok "Build succeeded."
    else
        die "Build failed. Fix errors above before launching."
    fi
}

# ── Step 2: Source ─────────────────────────────────────────────────────────────
do_source() {
    echo ""
    echo -e "${BOLD}━━━ Source ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━${RESET}"
    if [[ ! -f "$INSTALL_SETUP" ]]; then
        die "install/setup.bash not found. Build the workspace first."
    fi
    info "Sourcing $INSTALL_SETUP"
    # Temporarily relax strict mode — setup.bash uses unset vars and non-zero returns
    set +euo pipefail
    # shellcheck source=/dev/null
    source "$INSTALL_SETUP"
    set -euo pipefail
    ok "Workspace sourced."
}

# ── List launch files ──────────────────────────────────────────────────────────
do_list() {
    echo ""
    echo -e "${BOLD}━━━ Available Launch Files ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━${RESET}"
    local found=0
    for pkg in "${PACKAGES[@]}"; do
        local launch_dirs=(
            "$SRC_DIR/$pkg/launch"
            "$WS_ROOT/install/$pkg/share/$pkg/launch"
        )
        for dir in "${launch_dirs[@]}"; do
            if [[ -d "$dir" ]]; then
                while IFS= read -r -d '' f; do
                    local name
                    name="$(basename "$f" .launch.py)"
                    echo -e "  ${GREEN}${pkg}${RESET}  →  ${CYAN}${name}${RESET}"
                    echo -e "    ${YELLOW}start:${RESET} $(basename "$0") start $name"
                    found=1
                done < <(find "$dir" -maxdepth 1 -name "*.launch.py" -print0 2>/dev/null)
                break  # prefer src over install
            fi
        done
    done
    if [[ $found -eq 0 ]]; then
        warn "No launch files found in src/*/launch/"
    fi
    echo ""
}

# ── Find which package owns a launch file ─────────────────────────────────────
find_package_for_launch() {
    local launch_name="$1"
    for pkg in "${PACKAGES[@]}"; do
        local candidate="$SRC_DIR/$pkg/launch/${launch_name}.launch.py"
        if [[ -f "$candidate" ]]; then
            echo "$pkg"
            return 0
        fi
    done
    # Fall back to installed location
    for pkg in "${PACKAGES[@]}"; do
        local candidate="$WS_ROOT/install/$pkg/share/$pkg/launch/${launch_name}.launch.py"
        if [[ -f "$candidate" ]]; then
            echo "$pkg"
            return 0
        fi
    done
    return 1
}

# ── Step 3: Run ────────────────────────────────────────────────────────────────
do_run() {
    local launch_name="$1"
    shift
    local extra_args=("$@")

    echo ""
    echo -e "${BOLD}━━━ Launch ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━${RESET}"

    local pkg
    if ! pkg=$(find_package_for_launch "$launch_name"); then
        error "Launch file '${launch_name}.launch.py' not found in any package."
        echo ""
        do_list
        exit 1
    fi

    info "Package : $pkg"
    info "Launch  : ${launch_name}.launch.py"
    [[ ${#extra_args[@]} -gt 0 ]] && info "Args    : ${extra_args[*]}"

    # Software rendering — required in VM/Parallels to prevent black screen and GPU crashes
    export LIBGL_ALWAYS_SOFTWARE=1
    export QT_OPENGL=software
    # FastDDS shared-memory transport commonly leaves stale /dev/shm locks in VMs.
    # Use UDP transport for local ROS discovery to avoid fastrtps_port* lock errors.
    if [[ -f "$FASTDDS_PROFILE" ]]; then
        export FASTDDS_DEFAULT_PROFILES_FILE="$FASTDDS_PROFILE"
        export FASTRTPS_DEFAULT_PROFILES_FILE="$FASTDDS_PROFILE"
        info "DDS     : FastDDS UDP profile (SHM disabled)"
    fi
    info "Rendering: LIBGL_ALWAYS_SOFTWARE=1  QT_OPENGL=software"
    echo ""

    # Teleop must own the terminal directly — ros2 launch does not forward
    # stdin to child processes, so termios keyboard reading fails with
    # "Inappropriate ioctl for device".  Run the executable directly instead.
    if [[ "$launch_name" == "teleop" ]]; then
        info "Teleop: running via 'ros2 run' (requires direct terminal stdin)"
        echo ""
        exec ros2 run "$pkg" teleop_keyboard "${extra_args[@]}"
    fi

    exec ros2 launch "$pkg" "${launch_name}.launch.py" "${extra_args[@]}"
}

# ── Argument parsing ───────────────────────────────────────────────────────────
COMMAND="${1:-build}"
shift || true

case "$COMMAND" in
    -h|--help|help)
        usage
        exit 0
        ;;

    # ── test: 运行感知工具单元测试（不需要 ROS / 仿真）──────────────────────
    test)
        NO_BUILD=false
        PATTERN=""
        while [[ $# -gt 0 ]]; do
            case "$1" in
                --no-build) NO_BUILD=true; shift ;;
                -k)         shift; PATTERN="$1"; shift ;;  # pytest -k 过滤
                *)          PATTERN="$1"; shift ;;
            esac
        done

        if [[ "$NO_BUILD" == false ]]; then
            do_build
        else
            warn "--no-build: 跳过 colcon build"
        fi

        echo ""
        echo -e "${BOLD}━━━ Perception Unit Tests ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━${RESET}"
        cd "$WS_ROOT"

        PYTEST_ARGS=("src/auto_nav_part3/test/test_perception_utils.py" "-v" "--tb=short")
        [[ -n "$PATTERN" ]] && PYTEST_ARGS+=("-k" "$PATTERN")

        info "pytest ${PYTEST_ARGS[*]}"
        echo ""
        set +e
        pytest "${PYTEST_ARGS[@]}"
        EXIT_CODE=$?
        set -e
        echo ""
        if [[ $EXIT_CODE -eq 0 ]]; then
            ok "全部测试通过。"
        else
            error "有测试失败，退出码 $EXIT_CODE"
        fi
        exit $EXIT_CODE
        ;;

    # ── camera: 单独启动感知节点（不需要仿真，用于离线/真机调试）─────────────
    # 节点依赖：
    #   /oak/rgb/image_raw  — 相机图像（用 image_publisher 或 USB 摄像头提供）
    #   /scan               — 雷达扫描（用 fake_scan_pub.py 或真实雷达提供）
    #   /odom               — 里程计（用 ros2 topic pub 提供）
    # 检测结果监听：ros2 topic echo /part3/perception/marker_event
    camera)
        NO_BUILD=false
        COOLDOWN="2.0"     # 测试时缩短冷却，方便重复触发
        MIN_CONF="0.4"     # 测试时降低置信度阈值，容易触发
        while [[ $# -gt 0 ]]; do
            case "$1" in
                --no-build)       NO_BUILD=true; shift ;;
                --cooldown)       shift; COOLDOWN="$1"; shift ;;
                --min-confidence) shift; MIN_CONF="$1"; shift ;;
                *) warn "未知选项: $1"; shift ;;
            esac
        done

        if [[ "$NO_BUILD" == false ]]; then
            do_build
        else
            warn "--no-build: 跳过 colcon build"
        fi
        do_source

        # 模型路径（从 install 目录找，build 后才存在）
        MODEL_PATH="$WS_ROOT/install/auto_nav_part3/share/auto_nav_part3/models/greek_letters.onnx"
        PHOTO_DIR="$WS_ROOT/artifacts/photos"

        echo ""
        echo -e "${BOLD}━━━ Camera Perception Test Mode ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━${RESET}"
        info "colour_detector  订阅: /oak/rgb/image_raw  /scan  /odom"
        info "greek_detector   订阅: /oak/rgb/image_raw  /scan  /odom"
        info "perception_adapter 去重并写入 artifacts/waypoints/markers.json"
        echo ""
        warn "需要在其他终端提供以下话题（见下方提示）："
        echo -e "  ${CYAN}/oak/rgb/image_raw${RESET}  — ros2 run image_publisher image_publisher_node <图片路径> --ros-args -r /image:=/oak/rgb/image_raw"
        echo -e "  ${CYAN}/scan${RESET}               — python3 scripts/fake_scan_pub.py"
        echo -e "  ${CYAN}/odom${RESET}               — ros2 topic pub /odom nav_msgs/msg/Odometry '{pose:{pose:{orientation:{w:1.0}}}}' --rate 10"
        echo -e "  ${CYAN}监听结果${RESET}            — ros2 topic echo /part3/perception/marker_event"
        echo ""

        # 启动 colour_detector（use_sim_time:=false，不等待 /clock）
        info "启动 colour_detector ..."
        ros2 run auto_nav_part3 colour_detector \
            --ros-args \
            -p use_sim_time:=false \
            -p photo_dir:="$PHOTO_DIR" \
            -p detection_cooldown_s:="$COOLDOWN" \
            -p min_area_px:=1500 &
        PID_COLOUR=$!

        sleep 1

        # 启动 greek_detector（greek_model_path 可能不存在，节点会打 warn 但不崩溃）
        info "启动 greek_detector ..."
        ros2 run auto_nav_part3 greek_detector \
            --ros-args \
            -p use_sim_time:=false \
            -p photo_dir:="$PHOTO_DIR" \
            -p greek_model_path:="$MODEL_PATH" \
            -p detection_cooldown_s:="$COOLDOWN" \
            -p min_confidence:="$MIN_CONF" &
        PID_GREEK=$!

        sleep 1

        # 启动 perception_adapter
        info "启动 perception_adapter ..."
        ros2 run auto_nav_part3 perception_adapter \
            --ros-args \
            -p use_sim_time:=false \
            -p dedup_radius_m:=1.0 \
            -p waypoints_save_dir:="$WS_ROOT/artifacts/waypoints" &
        PID_ADAPTER=$!

        ok "全部感知节点已启动。Ctrl+C 停止所有节点。"
        echo ""

        # 捕获 Ctrl+C，清理所有后台节点
        trap "info '停止所有节点...'; kill $PID_COLOUR $PID_GREEK $PID_ADAPTER 2>/dev/null; exit 0" INT TERM
        wait
        ;;

    build)
        do_build
        do_source
        ok "Ready. Run:  $(basename "$0") start <name>"
        ;;

    list)
        # Source so ros2 pkg commands work, but don't require a prior build
        if [[ -f "$INSTALL_SETUP" ]]; then
            set +euo pipefail
            source "$INSTALL_SETUP" 2>/dev/null || true
            set -euo pipefail
        fi
        do_list
        ;;

    start)
        # Parse 'start' arguments
        LAUNCH_NAME=""
        NO_BUILD=false
        DO_CLEAN=false
        EXTRA_ARGS=()

        while [[ $# -gt 0 ]]; do
            case "$1" in
                --clean)
                    DO_CLEAN=true
                    shift
                    ;;
                --no-build)
                    NO_BUILD=true
                    shift
                    ;;
                --args)
                    shift
                    # Split the quoted args string into array elements
                    read -ra _split <<< "$1"
                    EXTRA_ARGS+=("${_split[@]}")
                    shift
                    ;;
                --help|-h)
                    usage; exit 0
                    ;;
                --*)
                    die "Unknown option: $1"
                    ;;
                *)
                    if [[ -z "$LAUNCH_NAME" ]]; then
                        LAUNCH_NAME="$1"
                    else
                        # Positional args after the name are passed to ros2 launch
                        EXTRA_ARGS+=("$1")
                    fi
                    shift
                    ;;
            esac
        done

        [[ -z "$LAUNCH_NAME" ]] && { usage; die "Missing launch name. Use: $(basename "$0") start <name>"; }

        if [[ "$DO_CLEAN" == true ]]; then
            do_clean
        fi

        if [[ "$NO_BUILD" == false ]]; then
            do_build
        else
            warn "--no-build: skipping colcon build."
        fi

        do_source
        do_run "$LAUNCH_NAME" "${EXTRA_ARGS[@]}"
        ;;

    *)
        error "Unknown command: '$COMMAND'"
        echo ""
        usage
        exit 1
        ;;
esac
