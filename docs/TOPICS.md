# ROS2 Topic and Service Contract for Part 3

Rule: if a node publishes, subscribes, or calls an interface owned by another member, update this file before coding the dependency. This is the team API contract.

## Naming conventions

- Prefix all Part 3 interfaces with `/part3/` unless using standard robot interfaces such as `/cmd_vel`, `/scan`, `/odom`, `/tf`.
- Use nouns for topics and verbs for services.
- Do not rename or change message types without team agreement.

## Standard robot interfaces

| Interface | Type | Direction | Owner | Purpose |
|---|---|---:|---|---|
| `/cmd_vel` | `geometry_msgs/msg/Twist` | publish | Navigation / Safety | Robot velocity command. Safety monitor may publish zero Twist for software stop. |
| `/scan` | `sensor_msgs/msg/LaserScan` | subscribe | Safety / Mapping | Lidar scan for obstacle avoidance and estop detection. |
| `/odom` | `nav_msgs/msg/Odometry` | subscribe | Mapping / Planning / Logging | Robot pose estimate. Add dependency in `package.xml` before using. |
| `/tf`, `/tf_static` | `tf2_msgs/msg/TFMessage` | subscribe/publish | Robot description / localization | Transform tree, including Pioneer base frames. |

## Part 3 phase control services

| Service | Type | Owner | Caller | Purpose | Response contract |
|---|---|---|---|---|---|
| `/part3/mapping/start` | `std_srvs/srv/Trigger` | Mapping member | UI / operator | Start the 15x15m mapping and discovery phase without restarting stack. | `success=true` means mapping command accepted, not necessarily completed. |
| `/part3/waypoint/start` | `std_srvs/srv/Trigger` | Planning member | UI / operator | Start rapid waypoint driving phase after target Greek-letter waypoint IDs are known. | `success=true` means waypoint run command accepted. |

## Part 3 status topics

| Topic | Type | Owner | Subscribers | Purpose | Example data |
|---|---|---|---|---|---|
| `/part3/system/state` | `std_msgs/msg/String` | Integration/UI | everyone | Current high-level state. | `IDLE`, `MAPPING`, `WAYPOINT_DRIVE`, `ESTOP`, `COMPLETE` |
| `/part3/mapping/map_status` | `std_msgs/msg/String` | Mapping member | UI / logger / planner | Mapping progress and map availability. Replace with structured msg only after team approval. | `mapping_started: search_area=15x15m frame=map` |
| `/part3/perception/marker_event` | `std_msgs/msg/String` | Perception member | mapping / logger / UI / planner | Detected Greek letter or red/yellow object with estimated location. | `type=greek label=alpha x=2.1 y=-0.4 confidence=0.82 image=...` |
| `/part3/waypoint/plan` | `std_msgs/msg/String` | Planning member | UI / logger / navigation | Ordered route for second fast run. | `home -> alpha -> beta -> gamma -> home` |
| `/part3/safety/estop_event` | `std_msgs/msg/String` | Safety/integration | logger / UI | Software estop incident and 5s data-save trigger. | `software_estop: obstacle_within_1m save_last_5_seconds=true` |

## Future structured interfaces

Start with standard messages to keep the project executable. Once behaviour is stable, replace high-value `String` topics with custom messages in a separate interface package, for example:

- `MarkerEvent.msg`: `string marker_type`, `string label`, `float64 x`, `float64 y`, `float64 confidence`, `string image_path`
- `WaypointPlan.msg`: `string[] labels`, `geometry_msgs/PoseStamped[] poses`, `float64 estimated_distance_m`
- `SystemState.msg`: enum-like integer state plus human-readable action text

Do not create custom messages until at least two nodes need the same structured fields.

## Interface change checklist

1. Update this document first.
2. Add or modify tests for any node affected by the change.
3. Update `package.xml` dependencies if a new ROS message package is used.
4. Communicate the change in the team chat before merging to `main`.
