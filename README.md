# AUTO4508 Part 3 — Autonomous Mapping and Waypoint Navigation

Team 18 | ROS2 Jazzy | Gazebo Harmonic | Pioneer 3-AT | Ubuntu 24.04 (ARM64)

This package implements a two-phase autonomous robot mission:

1. **Phase 1 — Exploration & Mapping**: the robot autonomously explores a 15 × 15 m unknown arena using frontier-based SLAM, detects Greek-letter markers and colour obstacles via an OAK-D camera, and saves a complete map + marker list on completion.
2. **Phase 2 — Waypoint Navigation**: on a second run the robot loads the saved map, reads the detected marker positions from disk, computes a TSP-optimal route, and visits every target waypoint before returning home.

---

## Quick Start

```bash
# Build (run once; --symlink-install means Python edits are live immediately)
colcon build --symlink-install

# Source the workspace (required in every new terminal)
source install/setup.bash

# Launch the full simulation stack
ros2 launch auto_nav_part3 sim_bringup.launch.py \
    use_slam:=true use_nav2:=true use_exploration:=true \
    use_safety:=true use_camera:=true use_rviz:=true
```

Wait ~45 s for all lifecycle nodes to reach `active`, then in a second terminal:

```bash
source install/setup.bash

# Phase 1: start autonomous mapping
ros2 service call /part3/mapping/start std_srvs/srv/Trigger {}

# Monitor progress
ros2 topic echo /part3/system/state
ros2 topic echo /part3/mapping/map_status

# Phase 2: start waypoint run (after mapping finishes)
ros2 service call /part3/waypoint/start std_srvs/srv/Trigger {}
```

---

## System State Machine

```
IDLE  ──/part3/mapping/start──►  MAPPING  ──coverage=done──►  COMPLETE
                                                                    │
                                              /part3/waypoint/start ▼
                                         WAYPOINT_PREPARE ──► WAYPOINT_DRIVE ──► COMPLETE
                                                                    │
                                                              (e-stop)
                                                                    ▼
                                                                  ESTOP
```

System state is published on `/part3/system/state` (std_msgs/String, 1 Hz).

---

## Architecture

### Node Map

| Module | Node | Responsibility |
|---|---|---|
| **Mapping** | `mapping_service` | M5 orchestrator — forwards `/part3/mapping/start` to `exploration_node` |
| | `exploration_node` | Frontier-based autonomous exploration (Yamauchi 1997) |
| | `map_manager` | Saves OccupancyGrid → .pgm/.yaml/.png on exploration completion |
| **Navigation** | `waypoint_service` | TSP-optimal waypoint driving via Nav2 NavigateThroughPoses |
| **Perception** | `greek_detector` | ONNX-based Greek-letter detection on OAK-D RGB frames |
| | `colour_detector` | HSV colour segmentation for red/yellow obstacle detection |
| | `perception_adapter` | Deduplicates and persists marker events; publishes PoseArray |
| **Safety** | `safety_monitor` | Lidar-based moving-obstacle detection + software e-stop via twist_mux |
| | `rolling_recorder` | 5-second rolling bag buffer; dumps to disk on e-stop |
| **System** | `state_manager` | Aggregates subsystem state into `/part3/system/state` |
| **UI** | Flask app (`ui/app.py`) | Web dashboard — map viewer, robot pose, marker overlay, controls |

### cmd_vel Pipeline

```
teleop_keyboard ──(priority 10)──┐
waypoint_service ──(priority 5)──┼──► twist_mux ──► /cmd_vel ──► robot
safety_monitor ──(priority 100)──┘
```

The safety monitor's zero-Twist e-stop overrides all other velocity sources.

---

## Key Interfaces

### Services (trigger actions)

| Service | Type | Effect |
|---|---|---|
| `/part3/mapping/start` | `std_srvs/Trigger` | Start autonomous exploration |
| `/part3/mapping/save_map` | `std_srvs/Trigger` | Manually save current map |
| `/part3/waypoint/start` | `std_srvs/Trigger` | Start waypoint navigation run |
| `/part3/perception/get_markers` | `std_srvs/Trigger` | Force a marker list publish |

### Topics (subscribe for status)

| Topic | Type | Rate | Content |
|---|---|---|---|
| `/part3/system/state` | `String` | 1 Hz | `IDLE` / `MAPPING` / `COMPLETE` / `ESTOP` / … |
| `/part3/mapping/map_status` | `String` | 1 Hz | `coverage=68% frontiers=3` / `coverage=done coverage_pct=96%` |
| `/part3/waypoint/plan` | `String` | event | JSON waypoint progress payload |
| `/part3/safety/estop_event` | `String` | event | `software_estop timestamp=… min_dist=0.45` |
| `/part3/perception/markers` | `PoseArray` | 2 Hz | All confirmed markers (map frame) |
| `/part3/perception/greek_markers` | `PoseArray` | 2 Hz | Greek-letter markers only |

> **Note:** `/map` requires `TRANSIENT_LOCAL + RELIABLE` QoS — use the matching profile or you will receive no data after a late subscribe.

---

## Launch Modes

### Full simulation (Phase 1 — Exploration)

```bash
ros2 launch auto_nav_part3 sim_bringup.launch.py \
    use_slam:=true use_nav2:=true use_exploration:=true \
    use_safety:=true use_camera:=true use_rviz:=true
```

### Phase 2 — Waypoint navigation on saved map

```bash
ros2 launch auto_nav_part3 sim_bringup.launch.py \
    use_localization:=true use_nav2:=true \
    use_slam:=false use_exploration:=false use_rviz:=true
ros2 service call /part3/waypoint/start std_srvs/srv/Trigger {}
```

### Nav2 only (no exploration)

```bash
ros2 launch auto_nav_part3 sim_bringup.launch.py \
    use_slam:=true use_nav2:=true use_exploration:=false use_rviz:=true
```

### Minimal (no SLAM/Nav2/camera — verify base hardware only)

```bash
ros2 launch auto_nav_part3 sim_bringup.launch.py \
    use_slam:=false use_nav2:=false use_exploration:=false \
    use_safety:=false use_camera:=false use_rviz:=true
```

### Keyboard teleop

```bash
ros2 launch auto_nav_part3 teleop.launch.py
```

### Web UI

```bash
cd ui && python3 app.py      # open http://localhost:5000
```

---

## Artifacts

After a successful exploration run the following files are written:

```
artifacts/
├── maps/
│   ├── discovery_map.pgm         # occupancy grid (nav2 map_server compatible)
│   ├── discovery_map.yaml        # map metadata (resolution, origin, thresholds)
│   ├── discovery_map.png         # visual PNG for report
│   ├── discovery_map.posegraph   # slam_toolbox pose graph (Phase 2 localisation)
│   └── discovery_map.data
├── waypoints/
│   └── markers.json              # deduplicated marker list (persists across restarts)
└── bags/
    └── estop_<timestamp>/        # 5-second rolling bag dumped on each e-stop
```

---

## Simulation Environment

- **Arena**: 15 × 15 m outdoor arena (`discovery_15x15.sdf`)
- **Robot spawn**: x = −3.0, y = 0.0
- **Obstacles**: 9 box obstacles arranged across the arena
- **SLAM**: slam_toolbox online async mode, 5 cm resolution
- **Localisation**: robot_localization EKF fusing `/odom` + `/imu`
- **Controller**: MPPI (Model Predictive Path Integral)
- **Exploration algorithm**: Frontier-based (Yamauchi 1997), adapted from [adrian-soch/frontier_exploration](https://github.com/adrian-soch/frontier_exploration) (MIT Licence)

---

## Development Notes

### Build and test

```bash
colcon build --symlink-install 2>&1 | tail -20
colcon test && colcon test-result --verbose
```

### SLAM not activating?

```bash
# Manually activate slam_toolbox lifecycle (retry loop handles DDS discovery latency)
until ros2 lifecycle set /slam_toolbox configure 2>/dev/null; do sleep 0.5; done
sleep 0.5 && ros2 lifecycle set /slam_toolbox activate
```

### Diagnose TF issues

```bash
ros2 run tf2_tools view_frames          # generates frames.pdf
ros2 run tf2_ros tf2_echo map odom      # check map→odom transform
ros2 topic info /tf -v                  # check for duplicate odom→base_link publishers
```

### Useful monitoring commands

```bash
ros2 topic hz /scan /odom /imu /map /odometry/filtered
ros2 action info /navigate_through_poses
ros2 lifecycle get /slam_toolbox
ros2 node list
```

---

## Repository Structure

```
auto4508-project-part3/
├── src/auto_nav_part3/
│   ├── auto_nav_part3/
│   │   ├── mapping/          # exploration_node, mapping_service, map_manager
│   │   ├── navigation/       # waypoint_service
│   │   ├── perception/       # greek_detector, colour_detector, perception_adapter
│   │   ├── safety/           # safety_monitor, rolling_recorder
│   │   └── system/           # state_manager, ui_status
│   ├── launch/               # sim_bringup, nav2_bringup, camera_bringup, teleop
│   └── config/               # nav2_params, slam_toolbox, ekf, twist_mux, waypoint, safety
├── ui/                       # Flask web dashboard
├── artifacts/                # runtime outputs (maps, waypoints, bags)
└── docs/                     # interface contract, test guide, debug plan
```

---

## Key Documentation

| File | Purpose |
|---|---|
| [`docs/TOPICS.md`](docs/TOPICS.md) | ROS2 topic and service API contract — update before changing any interface |
| [`docs/SIM_TEST_GUIDE.md`](docs/SIM_TEST_GUIDE.md) | Stage-by-stage simulation verification (M0 → full integration) |
| [`docs/TASK_ALLOCATION.md`](docs/TASK_ALLOCATION.md) | 3-member task split and ownership rules |
| [`docs/DEBUG_PLAN.md`](docs/DEBUG_PLAN.md) | Known bugs and root causes |
| [`docs/PART3_DEVELOPMENT_PLAN.md`](docs/PART3_DEVELOPMENT_PLAN.md) | Milestone plan and progress tracking |

---

## External Attribution

- **Frontier exploration algorithm**: adapted from [adrian-soch/frontier_exploration](https://github.com/adrian-soch/frontier_exploration) by Adrian Sochaniwsky (MIT Licence), implementing the frontier-based exploration method described in B. Yamauchi, "A Frontier-Based Approach for Autonomous Exploration," CIRA'97, doi: [10.1109/CIRA.1997.613851](https://doi.org/10.1109/CIRA.1997.613851).
