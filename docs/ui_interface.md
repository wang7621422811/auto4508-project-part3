UI 接口文档 — AUT4508 Part 3
所有话题/服务均在机器人启动后可用。UI 使用 ROS2 Python/Web 客户端（roslibjs / rclpy 均可）订阅/发布/调用。

一、UI 需要订阅的话题（只读，UI 展示用）
话题	消息类型	发布频率	用途
/part3/system/state	std_msgs/String	1 Hz	系统总状态，值为：IDLE / MAPPING / COMPLETE / WAYPOINT_DRIVE / WAYPOINT_COMPLETE / WAYPOINT_FAILED
/part3/mapping/map_status	std_msgs/String	1 Hz	探索进度，格式见下表
/part3/waypoint/plan	std_msgs/String	事件触发	路点行驶计划，如 home(0.0,0.0)→A(1.23,4.56)→B(2.34,5.67)→home(0.0,0.0) dist=12.3m
/part3/safety/estop_event	std_msgs/String	事件触发	急停事件，格式：software_estop timestamp=xxx.xxx min_dist=0.45 bearing_deg=15.3 save_last_5s=true
/part3/perception/markers	geometry_msgs/PoseArray	2 Hz	所有识别到的 marker（希腊字母 + 颜色障碍），map 坐标系
/part3/perception/greek_markers	geometry_msgs/PoseArray	2 Hz	仅希腊字母 marker，map 坐标系，可直接作为路点显示
/map	nav_msgs/OccupancyGrid	按需（SLAM 更新时）	SLAM 地图，用于 UI 地图可视化（QoS: TRANSIENT_LOCAL，RELIABLE）
/odometry/filtered	nav_msgs/Odometry	50 Hz	机器人当前位置（EKF 融合），用于地图上显示机器人实时位置
/scan	sensor_msgs/LaserScan	10 Hz	激光雷达扫描，可用于 UI 叠加显示障碍
/oak/rgb/image_raw（实机）或 /camera/image（仿真）	sensor_msgs/Image	30 Hz	摄像头画面，用于 UI 实时视频流显示
/part3/mapping/map_status 字符串格式
状态	示例
探索中	coverage=68% frontiers=3 area=15x15
已完成	coverage=done coverage_pct=97%
卡住	coverage=stuck coverage_pct=73%
地图保存成功	map_saved path=/...
二、UI 需要调用的服务（触发动作）
服务	类型	用途
/part3/mapping/start	std_srvs/Trigger	开始自主建图（探索阶段入口，点一次即可）
/part3/mapping/save_map	std_srvs/Trigger	手动保存当前地图（也可探索完自动触发）
/part3/waypoint/start	std_srvs/Trigger	开始路点快速驾驶（第二阶段，探索完后调）
/part3/perception/get_markers	std_srvs/Trigger	立即刷新 marker 列表（主动请求一次发布）
调用示例（命令行验证用）：


ros2 service call /part3/mapping/start std_srvs/srv/Trigger {}
ros2 service call /part3/waypoint/start std_srvs/srv/Trigger {}
三、UI 需要发布的话题（如需手动控制）
话题	消息类型	用途
/part3/exploration/enable	std_msgs/Bool	发 true 启动探索，发 false 停止（比服务更灵活，可随时中断）
/cmd_vel	geometry_msgs/Twist	手动遥控机器人（会被 twist_mux 中 priority=100 的急停信号覆盖）
四、数据流总结图

[UI]
  │
  ├── 订阅 ──→ /part3/system/state       ← state_manager / waypoint_service
  ├── 订阅 ──→ /part3/mapping/map_status ← exploration_node / map_manager
  ├── 订阅 ──→ /part3/waypoint/plan      ← waypoint_service
  ├── 订阅 ──→ /part3/safety/estop_event ← safety_monitor
  ├── 订阅 ──→ /part3/perception/markers ← perception_adapter (2Hz)
  ├── 订阅 ──→ /map                      ← slam_toolbox
  ├── 订阅 ──→ /odometry/filtered        ← robot_localization EKF
  ├── 订阅 ──→ /camera/image             ← camera node
  │
  ├── 调用服务 → /part3/mapping/start    → 启动探索
  ├── 调用服务 → /part3/waypoint/start   → 启动路点驾驶
  ├── 调用服务 → /part3/mapping/save_map → 保存地图
  │
  └── 发布 ──→ /part3/exploration/enable → 实时开关探索
              /cmd_vel                  → 手动遥控
五、关键注意事项
/map 的 QoS：必须设 TRANSIENT_LOCAL + RELIABLE，否则订阅时 slam_toolbox 不会重发最新地图。
急停期间：/part3/safety/estop_event 触发后，机器人会自动零速并保存 5s rosbag，UI 只需展示事件即可，不需要额外操作。
两阶段流程：建图（调 /part3/mapping/start）→ 等 system/state=COMPLETE → 路点驾驶（调 /part3/waypoint/start）。
/part3/perception/markers 的 pose 无朝向：orientation 全为单位四元数，只有 position.x/y 有意义。
仿真摄像头话题：仿真用 /camera/image，实机 OAK-D 用 /oak/rgb/image_raw，UI 需做配置项区分。