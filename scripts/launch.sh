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
    info "Rendering: LIBGL_ALWAYS_SOFTWARE=1  QT_OPENGL=software"
    echo ""

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
        EXTRA_ARGS=()

        while [[ $# -gt 0 ]]; do
            case "$1" in
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
