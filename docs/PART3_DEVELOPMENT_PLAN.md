# AUTO4508 Part 3 Implementation Plan

> For Hermes: Use subagent-driven-development skill to implement this plan task-by-task if execution is requested later.

Goal: build a ROS2 system that explores and maps a 15x15m unknown area, detects Greek letters and red/yellow objects with locations/photos, handles software estop and 5s data capture, provides UI state/path display, then performs the fastest second run through three selected waypoints and home.

Architecture: use ROS2 nodes as ownership boundaries. Member 1 owns mapping/exploration, Member 2 owns perception events, Member 3 owns UI/safety/logging/rapid waypoint. Cross-member integration occurs only through topics/services documented in `docs/TOPICS.md`.

Tech Stack: ROS2 Python (`rclpy`), standard ROS messages/services first, Pioneer URDF from `urdf/pioneer.urdf`, later Nav2/SLAM/OAK-D/OpenCV/sick_scan_xd as needed and referenced.

---

## Phase 0: Repository and team workflow

### Task 0.1: Confirm branches

Objective: ensure every member develops independently and integrates through `main`.

Files:
- Read: `docs/TASK_ALLOCATION.md`

Steps:
1. Each member chooses a real branch name, for example `member/alice`, `member/bob`, `member/charlie`.
2. Run `git checkout member/<name>` before coding.
3. Commit after each small working change.
4. Merge latest `main` before requesting integration.

Verification:
- `git branch` shows `main` and three member branches.

### Task 0.2: Protect the topic contract

Objective: prevent hidden interface conflicts.

Files:
- Modify first for any ROS interface change: `docs/TOPICS.md`

Steps:
1. Before using another member’s function, find its topic/service in `docs/TOPICS.md`.
2. If it is missing, add the proposed contract before coding.
3. Ask the owner to approve the topic name and message type.

Verification:
- `pytest src/auto_nav_part3/test/test_docs_contract.py -q` passes.

## Phase 1: Minimal executable baseline

### Task 1.1: Build the package

Objective: prove the repository is executable before adding algorithms.

Files:
- `src/auto_nav_part3/package.xml`
- `src/auto_nav_part3/setup.py`
- `src/auto_nav_part3/launch/part3_minimal.launch.py`

Command:
```bash
colcon build --symlink-install
source install/setup.bash
ros2 launch auto_nav_part3 part3_minimal.launch.py
```

Expected:
- state manager publishes `/part3/system/state`
- mapping and waypoint services are available

### Task 1.2: Validate service separation

Objective: satisfy requirement that mapping phase and waypoint phase can run from buttons/services without restarting the stack.

Command:
```bash
ros2 service call /part3/mapping/start std_srvs/srv/Trigger {}
ros2 service call /part3/waypoint/start std_srvs/srv/Trigger {}
ros2 topic echo /part3/system/state
```

Expected:
- mapping call returns success
- waypoint call returns success
- UI logs state/action updates

## Phase 2: Member 1 mapping/exploration

### Task 2.1: Replace placeholder mapping status with real map progress

Objective: publish mapping coverage/progress while exploring the 15x15m area.

Files:
- Modify: `src/auto_nav_part3/auto_nav_part3/mapping_service.py`
- Create later: `src/auto_nav_part3/auto_nav_part3/exploration_node.py`

Implementation notes:
- Subscribe to `/scan`, `/odom`, `/tf`.
- Keep the robot inside a 15x15m boundary from home.
- Publish map progress to `/part3/mapping/map_status`.

Verification:
- In simulation, robot covers the defined area and avoids static obstacles.

### Task 2.2: Add map output for UI and report

Objective: provide map artifact paths and marker overlays for UI/report.

Files:
- Future: `artifacts/maps/`
- Update: `docs/TOPICS.md` if new artifact topic is added.

Verification:
- After mapping run, a map image or saved occupancy grid exists.

## Phase 3: Member 2 perception

### Task 3.1: Create perception node skeleton

Objective: publish marker events without depending on mapping internals.

Files:
- Create: `src/auto_nav_part3/auto_nav_part3/perception_node.py`
- Modify: `src/auto_nav_part3/setup.py`
- Modify if needed: `docs/TOPICS.md`

Behaviour:
- Subscribe to confirmed OAK-D camera topic.
- Detect red/yellow objects and Greek-letter candidates.
- Publish `/part3/perception/marker_event`.

Verification:
- With a test image or live camera, marker events appear with label/location/photo path.

### Task 3.2: Record photos and locations

Objective: satisfy requirement to take photos and note locations of Greek letters and red/yellow obstacles.

Files:
- Future: `artifacts/markers/`
- Future: `artifacts/markers/manifest.csv`

Verification:
- Each event has image path, timestamp, estimated map location, confidence, and marker type.

## Phase 4: Member 3 safety, UI, logging, rapid waypoint run

### Task 4.1: Make software estop robust

Objective: stop if moving obstacle or any object comes within 1m and trigger last-5-seconds recording.

Files:
- Modify: `src/auto_nav_part3/auto_nav_part3/safety_monitor.py`
- Future: `src/auto_nav_part3/auto_nav_part3/data_recorder.py`

Verification:
- Inject a LaserScan under 1m and confirm `/cmd_vel` zero plus `/part3/safety/estop_event`.
- Confirm last 5 seconds of rosbag/data are saved.

### Task 4.2: Build UI state/action display

Objective: always display robot internal state and intended actions.

Files:
- Modify: `src/auto_nav_part3/auto_nav_part3/ui_status.py`

Verification:
- During mapping, UI shows map, marker photos/locations.
- During waypoint phase, UI shows planned path graphically.

### Task 4.3: Implement fastest path through selected waypoints

Objective: after first run, plan shortest route from home through 3 selected Greek-letter waypoints and back home.

Files:
- Modify: `src/auto_nav_part3/auto_nav_part3/waypoint_service.py`
- Future: `src/auto_nav_part3/auto_nav_part3/route_optimizer.py`

Algorithm:
- Input: home pose and candidate waypoint poses from marker events.
- For 3 targets, brute-force all 6 permutations and choose shortest collision-aware path estimate.
- Publish `/part3/waypoint/plan` for UI.

Verification:
- Unit test: for 3 known coordinates, route order is shortest.

## Phase 5: Integration and demonstration readiness

### Task 5.1: Simulation demonstration

Objective: demonstrate capability before real robot testing.

Steps:
1. Launch simulator with Pioneer URDF.
2. Start mapping service.
3. Show map coverage, marker detections, safety handling.
4. Start waypoint service and display route.

Verification:
- Record screen/video or rosbag for report evidence.

### Task 5.2: Real robot dry run checklist

Objective: reduce lab debugging risk.

Checklist:
- Confirm tires taped for indoor testing.
- Confirm joystick/manual override works.
- Confirm software estop threshold with lidar.
- Confirm OAK-D image topics and exposure.
- Confirm all rosbag/artifact paths have enough disk space.

## Risks and controls

- Conflict risk: reduced by branch ownership and topic contract.
- Interface drift: controlled by `docs/TOPICS.md` and tests.
- Late integration: controlled by keeping minimal stack launchable at all times.
- Perception uncertainty: start with red/yellow colour detection baseline before Greek-letter classification.
- Safety risk: software estop publishes zero `/cmd_vel`; manual/physical safety procedures still required.
