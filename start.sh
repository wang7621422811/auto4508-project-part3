#!/usr/bin/env bash
# start.sh — AUTO4508 Part 3 Interactive Launcher

set -euo pipefail

# ══════════════════════════════════════════════════════════════════════════════
# Colours & helpers
# ══════════════════════════════════════════════════════════════════════════════
GREEN='\033[0;32m'; CYAN='\033[0;36m'; YELLOW='\033[1;33m'
RED='\033[0;31m'; BOLD='\033[1m'; DIM='\033[2m'; RESET='\033[0m'
BLUE='\033[0;34m'; WHITE='\033[1;37m'

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
UI_PORT=5000
ROS_SETUP="/opt/ros/jazzy/setup.bash"
WS_SETUP="$SCRIPT_DIR/install/setup.bash"
PIDS=()
MODE=""          # sim | physical
LAUNCH_NAME=""

header() { echo -e "\n${BOLD}${BLUE}━━━  $*  ━━━${RESET}"; }
info()   { echo -e "  ${CYAN}•${RESET} $*"; }
ok()     { echo -e "  ${GREEN}✔${RESET} $*"; }
warn()   { echo -e "  ${YELLOW}⚠${RESET}  $*"; }
die()    { echo -e "\n  ${RED}✘ Error: $*${RESET}\n" >&2; exit 1; }

# ── Progress bar ─────────────────────────────────────────────────────────────
# Usage: progress_bar <step> <total> <label>
progress_bar() {
    local step=$1 total=$2 label=$3
    local pct=$(( step * 100 / total ))
    local filled=$(( step * 30 / total ))
    local empty=$(( 30 - filled ))
    local bar=""
    for ((i=0; i<filled; i++)); do bar+="█"; done
    for ((i=0; i<empty;  i++)); do bar+="░"; done
    printf "\r  ${CYAN}[%s]${RESET} %3d%%  %s" "$bar" "$pct" "$label"
    if [[ $step -eq $total ]]; then echo; fi
}

# ── Cleanup on Ctrl+C ────────────────────────────────────────────────────────
cleanup() {
    echo -e "\n\n  ${YELLOW}⚠  Shutting down all services...${RESET}"
    for pid in "${PIDS[@]}"; do
        kill "$pid" 2>/dev/null || true
    done
    kill -- -$$ 2>/dev/null || true
    echo -e "  ${GREEN}✔  All services stopped safely.${RESET}\n"
    exit 0
}
trap cleanup INT TERM

# ══════════════════════════════════════════════════════════════════════════════
# 1. Welcome
# ══════════════════════════════════════════════════════════════════════════════
clear
echo ""
echo -e "${BOLD}${BLUE}"
echo "  ╔══════════════════════════════════════════════════════════╗"
echo "  ║       AUTO4508 Part 3 — Mapping & Discovery System      ║"
echo "  ║         University of Western Australia  Team 18         ║"
echo "  ╚══════════════════════════════════════════════════════════╝"
echo -e "${RESET}"

# ══════════════════════════════════════════════════════════════════════════════
# 2. Mode selection
# ══════════════════════════════════════════════════════════════════════════════
echo -e "${BOLD}  Select a launch mode:${RESET}\n"
echo -e "  ${BOLD}${GREEN}[1] Simulation Mode${RESET}"
echo -e "  ${DIM}      Runs a virtual robot on this computer — no hardware required."
echo -e "      Best for: demonstrations, software testing, classroom use.${RESET}"
echo ""
echo -e "  ${BOLD}${CYAN}[2] Physical Robot Mode${RESET}"
echo -e "  ${DIM}      Connects to and controls the Pioneer 3-AT robot in the lab."
echo -e "      Best for: real-world missions, outdoor mapping and exploration.${RESET}"
echo ""

while true; do
    read -rp "  Enter [1] or [2] and press Enter: " choice
    case "$choice" in
        1) MODE="sim";      LAUNCH_NAME="sim_bringup";      break ;;
        2) MODE="physical"; LAUNCH_NAME="physical_bringup"; break ;;
        *) echo -e "  ${RED}Invalid input — please enter 1 or 2.${RESET}" ;;
    esac
done

if [[ "$MODE" == "sim" ]]; then
    echo -e "\n  ${GREEN}✔  Selected: Simulation Mode${RESET}"
else
    echo -e "\n  ${CYAN}✔  Selected: Physical Robot Mode${RESET}"
fi
sleep 1

# ══════════════════════════════════════════════════════════════════════════════
# 3. Environment checks
# ══════════════════════════════════════════════════════════════════════════════
header "Environment Checks"
echo ""

CHECKS_TOTAL=6
CHECKS_PASS=0
CHECKS_FAIL=()

run_check() {
    local step=$1 label=$2
    shift 2
    progress_bar "$step" "$CHECKS_TOTAL" "$label..."
    sleep 0.3
    if eval "$@" &>/dev/null; then
        progress_bar "$step" "$CHECKS_TOTAL" "${GREEN}✔ $label${RESET}     "
        echo
        CHECKS_PASS=$((CHECKS_PASS + 1))
    else
        progress_bar "$step" "$CHECKS_TOTAL" "${RED}✘ $label${RESET}     "
        echo
        CHECKS_FAIL+=("$label")
    fi
}

run_check 1 "ROS 2 Jazzy"              "[[ -f '$ROS_SETUP' ]]"
run_check 2 "Python 3.12"             "command -v python3.12"
run_check 3 "Flask (Web UI)"          "python3.12 -c 'import flask'"
run_check 4 "colcon build tool"       "command -v colcon"
run_check 5 "Project source directory" "[[ -d '$SCRIPT_DIR/src' ]]"

if [[ "$MODE" == "sim" ]]; then
    run_check 6 "Gazebo simulator"    "command -v gz || command -v gazebo"
else
    run_check 6 "lsof (port check)"  "command -v lsof"
fi

echo ""

if [[ ${#CHECKS_FAIL[@]} -gt 0 ]]; then
    echo -e "  ${RED}${BOLD}${#CHECKS_FAIL[@]} check(s) failed:${RESET}"
    for item in "${CHECKS_FAIL[@]}"; do
        echo -e "    ${RED}✘${RESET} $item"
    done
    echo ""
    if [[ "$MODE" == "sim" ]] && [[ " ${CHECKS_FAIL[*]} " == *"Gazebo"* ]]; then
        warn "Gazebo is not installed. Simulation mode requires Gazebo."
        die "Cannot start — environment requirements not met."
    fi
    warn "Some components are missing. The system may not run correctly."
    echo ""
    read -rp "  Continue anyway? [y/N]: " confirm
    [[ "$confirm" =~ ^[Yy]$ ]] || { echo -e "\n  Cancelled.\n"; exit 0; }
else
    echo -e "  ${GREEN}${BOLD}✔  All ${CHECKS_TOTAL} checks passed!${RESET}"
fi
sleep 0.5

# ══════════════════════════════════════════════════════════════════════════════
# 4. Load ROS environment
# ══════════════════════════════════════════════════════════════════════════════
header "Loading ROS Environment"
echo ""
set +euo pipefail
source "$ROS_SETUP"
set -euo pipefail
ok "ROS 2 Jazzy loaded"

# ══════════════════════════════════════════════════════════════════════════════
# 5. Build workspace
# ══════════════════════════════════════════════════════════════════════════════
header "Building Workspace"
echo ""

cd "$SCRIPT_DIR"

if [[ "$MODE" == "sim" ]]; then
    info "Simulation mode: cleaning old build and rebuilding..."
    echo ""
    colcon build --symlink-install 2>&1 | while IFS= read -r line; do
        echo -e "  ${DIM}${line}${RESET}"
    done || die "Build failed. Please screenshot the error above and contact a technician."
else
    if [[ -f "$WS_SETUP" ]]; then
        info "Existing build detected — skipping rebuild. (Delete build/ and install/ to force a clean build.)"
    else
        info "First run — building workspace..."
        echo ""
        colcon build --symlink-install 2>&1 | while IFS= read -r line; do
            echo -e "  ${DIM}${line}${RESET}"
        done || die "Build failed. Please screenshot the error above and contact a technician."
    fi
fi

echo ""
[[ -f "$WS_SETUP" ]] || die "Build succeeded but install/setup.bash not found. Contact a technician."

set +euo pipefail
source "$WS_SETUP"
set -euo pipefail
ok "Workspace built and sourced"

# ══════════════════════════════════════════════════════════════════════════════
# 6. Free port
# ══════════════════════════════════════════════════════════════════════════════
if lsof -ti tcp:"$UI_PORT" &>/dev/null; then
    warn "Port $UI_PORT is already in use — releasing it..."
    lsof -ti tcp:"$UI_PORT" | xargs kill -9 2>/dev/null || true
    sleep 1
    ok "Port $UI_PORT freed"
fi

# ══════════════════════════════════════════════════════════════════════════════
# 7. Launch ROS nodes
# ══════════════════════════════════════════════════════════════════════════════
header "Starting System"
echo ""

export LIBGL_ALWAYS_SOFTWARE=1
export QT_OPENGL=software

info "Launching ROS navigation nodes (${LAUNCH_NAME})..."

if [[ "$MODE" == "sim" ]]; then
    "$SCRIPT_DIR/scripts/launch.sh" start --clean sim_bringup \
        use_nav2:=true \
        use_exploration:=true \
        use_slam:=true \
        use_rviz:=true \
        use_safety:=true \
        use_camera:=true \
        use_recording:=true \
        &>/tmp/part3_ros.log &
else
    ros2 launch auto_nav_part3 "${LAUNCH_NAME}.launch.py" \
        use_nav2:=true \
        use_exploration:=true \
        use_slam:=true \
        use_safety:=true \
        use_camera:=true \
        use_rviz:=false \
        use_recording:=true \
        &>/tmp/part3_ros.log &
fi

ROS_PID=$!
PIDS+=($ROS_PID)
ok "Navigation nodes started in background  (log: /tmp/part3_ros.log)"

# ══════════════════════════════════════════════════════════════════════════════
# 8. Launch Web UI
# ══════════════════════════════════════════════════════════════════════════════
info "Starting Web UI..."

cd "$SCRIPT_DIR"
python3.12 ui/app.py &>/tmp/part3_ui.log &
UI_PID=$!
PIDS+=($UI_PID)
ok "Web UI started in background  (log: /tmp/part3_ui.log)"

# ══════════════════════════════════════════════════════════════════════════════
# 9. Wait for Web UI to be ready
# ══════════════════════════════════════════════════════════════════════════════
echo ""
info "Waiting for system to be ready..."
WAIT=0
MAX_WAIT=30
SPINNER=('⠋' '⠙' '⠹' '⠸' '⠼' '⠴' '⠦' '⠧' '⠇' '⠏')

while ! curl -s "http://localhost:$UI_PORT" &>/dev/null; do
    SPIN="${SPINNER[$((WAIT % ${#SPINNER[@]}))]}"
    printf "\r  ${CYAN}%s${RESET}  Waiting for Web UI... (%d / %d s)" "$SPIN" "$WAIT" "$MAX_WAIT"
    sleep 1
    WAIT=$((WAIT + 1))

    if [[ $WAIT -ge $MAX_WAIT ]]; then
        echo ""
        echo -e "\n  ${RED}✘ Startup timed out!${RESET}"
        echo -e "  ${DIM}Last 20 lines of UI log:${RESET}"
        tail -20 /tmp/part3_ui.log | while IFS= read -r line; do
            echo -e "  ${RED}│${RESET} $line"
        done
        die "Please screenshot the above and contact a technician."
    fi

    if ! kill -0 "$UI_PID" 2>/dev/null; then
        echo ""
        echo -e "\n  ${RED}✘ Web UI process exited unexpectedly!${RESET}"
        echo -e "  ${DIM}Last 20 lines of UI log:${RESET}"
        tail -20 /tmp/part3_ui.log | while IFS= read -r line; do
            echo -e "  ${RED}│${RESET} $line"
        done
        die "Please screenshot the above and contact a technician."
    fi
done

printf "\r  ${GREEN}✔${RESET}  Web UI is ready!%-30s\n" ""

# ══════════════════════════════════════════════════════════════════════════════
# 10. Success summary
# ══════════════════════════════════════════════════════════════════════════════
LOCAL_IP=$(hostname -I | awk '{print $1}')
echo ""
echo -e "${BOLD}${GREEN}"
echo "  ╔══════════════════════════════════════════════════════════╗"
echo "  ║              System started successfully!  🚀            ║"
echo "  ╚══════════════════════════════════════════════════════════╝"
echo -e "${RESET}"

if [[ "$MODE" == "sim" ]]; then
    echo -e "  ${BOLD}Mode:${RESET} ${GREEN}Simulation${RESET} (virtual robot)"
    echo -e "  ${DIM}  Note: the exploration node starts after a 45-second delay.${RESET}"
    echo -e "  ${DIM}  Wait ~45 s after launch before pressing Start Mapping.${RESET}"
else
    echo -e "  ${BOLD}Mode:${RESET} ${CYAN}Physical Robot${RESET} (Pioneer 3-AT)"
fi

echo ""
echo -e "  ${BOLD}Open your browser and go to:${RESET}"
echo -e "  ${BOLD}${WHITE}  ➜  http://localhost:${UI_PORT}${RESET}"
echo -e "  ${DIM}     On another device: http://${LOCAL_IP}:${UI_PORT}${RESET}"
echo ""
echo -e "  ${BOLD}Mission results are saved to:${RESET}"
echo -e "  ${DIM}  📷 Photos      →  artifacts/photos/${RESET}"
echo -e "  ${DIM}  🗺  Map         →  artifacts/maps/${RESET}"
echo -e "  ${DIM}  📍 Markers     →  artifacts/waypoints/markers.json${RESET}"
echo -e "  ${DIM}  📦 Recordings  →  artifacts/bags/${RESET}"
echo ""
echo -e "  ${YELLOW}Press Ctrl+C to safely shut down all services.${RESET}"
echo -e "${BOLD}${BLUE}━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━${RESET}"
echo ""

# ── Auto-open browser ─────────────────────────────────────────────────────────
if command -v xdg-open &>/dev/null; then
    xdg-open "http://localhost:$UI_PORT" &>/dev/null &
elif command -v open &>/dev/null; then
    open "http://localhost:$UI_PORT" &>/dev/null &
fi

# ── Keep running ──────────────────────────────────────────────────────────────
wait
