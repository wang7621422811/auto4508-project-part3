# Part 3 Task Allocation: Minimum-Conflict Split

Goal: split work by ROS2 interfaces instead of by shared files. Each member owns a small set of nodes and topics. Others use those topics/services instead of editing the owner’s code.

## Member 1 — Mapping, exploration, and robot motion integration

Primary goal: explore the 15x15m unknown area, maintain map, keep Part 1 autonomous navigation capability.

Owns files/directories:

- `src/auto_nav_part3/auto_nav_part3/mapping_service.py`
- future `src/auto_nav_part3/auto_nav_part3/exploration_node.py`
- future `src/auto_nav_part3/auto_nav_part3/map_manager.py`

Owns interfaces:

- `/part3/mapping/start`
- `/part3/mapping/map_status`
- standard navigation consumption: `/scan`, `/odom`, `/tf`

Avoid editing:

- Perception model code owned by Member 2.
- UI and safety code owned by Member 3 unless interface contract changes.

## Member 2 — Perception: Greek letters and red/yellow objects

Primary goal: detect hand-drawn Greek letters and special red/yellow obstacles, save photos, estimate/record locations.

Owns files/directories:

- future `src/auto_nav_part3/auto_nav_part3/perception_node.py`
- future `src/auto_nav_part3/auto_nav_part3/photo_logger.py`
- future `data/markers/` or `artifacts/markers/` if added

Owns interfaces:

- `/part3/perception/marker_event`
- camera/image input topics from OAK-D once confirmed

Avoid editing:

- Exploration and motion planner internals. Publish marker events instead.
- UI internals. UI subscribes to marker events.

## Member 3 — UI, safety, logging, and rapid waypoint phase

Primary goal: provide operator UI, software estop, rosbag/log recording, second-run fastest waypoint driving trigger and display.

Owns files/directories:

- `src/auto_nav_part3/auto_nav_part3/ui_status.py`
- `src/auto_nav_part3/auto_nav_part3/safety_monitor.py`
- `src/auto_nav_part3/auto_nav_part3/waypoint_service.py`
- future `src/auto_nav_part3/auto_nav_part3/data_recorder.py`
- future `src/auto_nav_part3/auto_nav_part3/route_optimizer.py`

Owns interfaces:

- `/part3/system/state`
- `/part3/safety/estop_event`
- `/part3/waypoint/start`
- `/part3/waypoint/plan`

Avoid editing:

- Mapping and perception internal implementation. Use documented topics/services.

## Shared rules

1. `docs/TOPICS.md` is the source of truth for ROS2 communication.
2. Every merge to `main` must build and launch the minimal stack.
3. If a change crosses ownership boundaries, create/update the interface in `docs/TOPICS.md` first.
4. Prefer adding a new node over modifying another member’s node.
5. Keep launch file changes small; if one branch needs a new node, add only one launch entry and test it.
