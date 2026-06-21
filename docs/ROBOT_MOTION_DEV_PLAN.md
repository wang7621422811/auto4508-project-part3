# 机器人运动 / 建图 / 探索 开发计划 (Member 1)

> Robot Motion / Mapping / Exploration Development Plan
>
> 本文档面向 **自己手写代码** 的开发流程。每个组件只给：**功能 / 为什么 / 接口契约 / 逻辑与交互 / 自检方法**。
> 不提供成品代码 —— 按契约自己实现，出 bug 时对照契约逐层排查。
>
> **环境前提 (Environment)**：Parallels + Ubuntu 24.04 (ARM64)，ROS2 **Jazzy**，Gazebo **Harmonic** (`gz sim`)。
>
> **接口真相源 (Source of truth)**：`docs/TOPICS.md`。本计划新增的接口必须先写进 `TOPICS.md` 再编码。

---

## Issues on Macos

```
# 打开rviz2黑屏问题
export LIBGL_ALWAYS_SOFTWARE=1
export QT_OPENGL=software
rviz2
```

## Pioneer 硬件列表

- Pioneer 3-AT Outdoor Mobile Robot Platform 
- Industrial Linux PC with onboard screen 
- GPS: not useful for this project
- IMU: Phidget Spatial 3/3/3 
- Camera: Stereo Camera OAK-D V2 
- Lidar: SICK TIMS7XXs/Lakibeam (雷达可以创建一个adapter接口，在配置文件中指定用哪个) 
- Software: Ubuntu, ROS2 and Aria

## 0. 你的职责范围 (Scope)

| 你拥有 (Own) | 你依赖别人 (Consume) | 你不要碰 (Avoid) |
|---|---|---|
| 仿真地基 (URDF/Gazebo/world) | `/part3/perception/marker_event` (Member 2) | 感知模型代码 (Member 2) |
| 定位 (EKF/TF) | `/part3/waypoint/start` 触发 (Member 3) | UI / safety 内部实现 (Member 3) |
| SLAM 建图 | — | `waypoint_service.py` 内部 |
| Nav2 导航栈 (维持 Part 1 能力) | | |
| 自主探索 exploration | | |
| `mapping_service.py` 实装 | | |
| 物理机器人驱动 bringup | | |

**核心交付物 (Deliverables)**：PDF 任务 2（边走边建图 15×15m）、任务 1（维持 Part 1 自主导航）、为任务 8/10 提供可用的地图与导航能力。

---

## 1. 里程碑总览 (Milestone Overview)

| 里程碑 | 内容 | 完成标志 (Definition of Done) |
|---|---|---|
| **M0** 仿真地基 | URDF 修复 + Gazebo world + teleop | 键盘能驱动机器人在 15×15m world 里跑 |
| **M1** 定位 | EKF 融合 odom+IMU + TF 树 | `map→odom→base_link` TF 连续无跳变 |
| **M2** SLAM | slam_toolbox 在线建图 | RViz 里地图随驾驶实时生长 |
| **M3** Nav2 | 导航栈 + 本地控制器 | 在 RViz 点目标点能自主到达并避障 |
| **M4** 探索 | frontier 自主探索 + 地图保存 | 机器人自动覆盖 15×15m 后停止并存图 |
| **M5** service 实装 | `mapping_service` 接真实 SLAM/探索 | 调用 `/part3/mapping/start` 真正开始探索 |
| **M6** 物理部署 | 真机驱动 + sim/real 切换 | 同一套节点 launch 参数切换 sim↔real |
| **M7** 集成验证 | 全栈联调 + 录屏存证 | 一条命令跑通 demo 流程 |

**建议节奏**：M0–M3 是地基，必须扎实；M4 是任务核心；M5 是团队对接；M6 留到实验室前 1 周。

---

## M0 仿真地基 (Simulation Foundation)

### C0.1 — URDF mesh 路径修复

**1. 功能**：让老师给的 `urdf/pioneer.urdf` 在本项目里能正常加载可视化网格。

**2. 为什么要有他**：URDF 里 mesh 写的是 `package://auto_nav/simulation/meshes/...`（Part 1 的包名），本项目包名是 `auto_nav_part3`，不修会找不到网格文件，Gazebo/RViz 报错或显示空白。

**3. 接口 / 参数定义**：
- 输入：Part 1 的 meshes 目录
  `/Users/weibin/workspace/University/AUTO4508/workspace/auto_nav_team18_1/auto_nav/simulation/meshes/`
- 目标位置：`src/auto_nav_part3/simulation/meshes/`
- URDF 内替换：`package://auto_nav/simulation/` → `package://auto_nav_part3/simulation/`
- `setup.py` 的 `data_files` 增加一行安装 `simulation/meshes/**`

**4. 逻辑与交互**：
1. `cp -r` 复制 meshes 到 `src/auto_nav_part3/simulation/meshes/`
2. 全文替换 URDF 里的 `package://auto_nav/` 前缀
3. `setup.py` 增加：`('share/auto_nav_part3/simulation/meshes', glob('simulation/meshes/**', recursive=True))`
4. `package.xml` 无需改

**5. 自检**：
```bash
colcon build --symlink-install && source install/setup.bash
ros2 launch auto_nav_part3 part3_minimal.launch.py
# 另开终端：rviz2，Fixed Frame=base_link，加 RobotModel，能看到完整 Pioneer
```

---

### C0.2 — Gazebo Harmonic world + 仿真 launch

**1. 功能**：提供一个 15×15m 的仿真场景，并把 Pioneer spawn 进去，桥接 Gazebo↔ROS2 话题。

**2. 为什么要有他**：PDF 要求“build a simulation to demonstrate before real world testing”；没有 world 和 bridge，SLAM/Nav2/探索全都无法在 sim 验证。

**3. 接口 / 参数定义**：
- 新建 `src/auto_nav_part3/worlds/discovery_15x15.sdf`
  - 地面 15×15m（可加围栏墙），几个 cone/box 静态障碍
- 新建 `src/auto_nav_part3/launch/sim_bringup.launch.py`
- `ros_gz_bridge` 桥接表（**必须和 URDF 内 topic 一致**）：

| Gazebo topic | ROS2 topic | 类型 | 方向 |
|---|---|---|---|
| `/cmd_vel` | `/cmd_vel` | `geometry_msgs/Twist` | ROS→GZ |
| `/odom` | `/odom` | `nav_msgs/Odometry` | GZ→ROS |
| `/scan` | `/scan` | `sensor_msgs/LaserScan` | GZ→ROS |
| `/imu` | `/imu` | `sensor_msgs/Imu` | GZ→ROS |
| `/camera` | `/camera/image_raw` | `sensor_msgs/Image` | GZ→ROS |
| `/tf` | `/tf` | `tf2_msgs/TFMessage` | GZ→ROS |
| `/joint_states` | `/joint_states` | `sensor_msgs/JointState` | GZ→ROS |

- 依赖：`package.xml` 增加 `ros_gz_sim`、`ros_gz_bridge`

**4. 逻辑与交互**：
1. `sim_bringup.launch.py` 启动顺序：`gz sim <world>` → `robot_state_publisher`(URDF) → `ros_gz_sim create`(spawn 机器人) → `ros_gz_bridge`(按上表) → 可选 `rviz2`
2. `gpu_lidar` 在 Parallels+Metal 下可用；若 RViz 无 scan，先把 URDF 的 `gpu_lidar` 改 `lidar` 排查
3. 下游（SLAM/Nav2）只认 ROS2 侧 topic，不直接碰 Gazebo

**5. 自检**：
```bash
ros2 launch auto_nav_part3 sim_bringup.launch.py
ros2 topic hz /scan /odom /imu     # 都应有稳定频率
ros2 topic echo /odom --once       # pose 合理
```

---

### C0.3 — teleop 驱动验证

**1. 功能**：键盘遥控验证整条 cmd_vel→运动→odom 闭环。

**2. 为什么要有他**：这是 PDF Part 1 Task 1 的基础要求，也是 SLAM/Nav2 能工作的前提；闭环不通后面全错。

**3. 接口 / 参数定义**：
- 工具：`teleop_twist_keyboard`（`apt install ros-jazzy-teleop-twist-keyboard`）
- 话题：发布 `/cmd_vel`，观察 `/odom`、`/tf`

**4. 逻辑与交互**：teleop → `/cmd_vel` → Gazebo diff-drive 插件 → 轮子转动 → `/odom` + `/tf(odom→base_link)` 更新。

**5. 自检**：按键机器人前进/转向，RViz 里 odom 轨迹随动、TF 不跳变。**此处通过才进 M1。**

---

## M1 定位 (Localization & TF)

### C1.1 — robot_localization EKF (odom + IMU 融合)

**1. 功能**：用 EKF 融合轮式里程计与 IMU，输出平滑的 `odom→base_link`。

**2. 为什么要有他**：纯轮式 odom 在草地/打滑会漂；PDF 强调传感器不可理想化。SLAM/Nav2 需要稳定连续的 odom，否则地图错位。

**3. 接口 / 参数定义**：
- 包：`robot_localization`（`apt install ros-jazzy-robot-localization`）
- 配置：`src/auto_nav_part3/config/ekf.yaml`
- 输入：`/odom` (`nav_msgs/Odometry`)、`/imu` (`sensor_msgs/Imu`)
- 输出：`/odometry/filtered` (`nav_msgs/Odometry`) + TF `odom→base_link`
- 关键参数：`frequency=30`、`two_d_mode=true`、`odom0_config` 用 x/y/yaw + 速度，`imu0_config` 用 yaw 角速度+线加速度
- ⚠️ 关掉 Gazebo diff-drive 自己发的 `odom→base_link` TF（避免双 TF 源冲突），diff-drive 只发 `/odom` 话题不发 TF

**4. 逻辑与交互**：Gazebo 发 `/odom`(话题) + `/imu` → EKF 融合 → 发 `odom→base_link` TF + `/odometry/filtered` → SLAM 消费。

**5. 自检**：
```bash
ros2 run tf2_tools view_frames   # 检查 TF 树无重复发布者
ros2 topic echo /odometry/filtered --once
```
遥控转一圈，`/odometry/filtered` 的 yaw 平滑、回原点附近。

---

### C1.2 — TF 树验证

**1. 功能**：确认完整 TF 链 `map → odom → base_link → {laser_frame, cam_*, imu_link}`。

**2. 为什么要有他**：SLAM 提供 `map→odom`，EKF 提供 `odom→base_link`，URDF(static)提供其余。任何一环断裂 Nav2/SLAM 都静默失败 —— 这是最常见的“查不出的 bug”根源。

**3. 接口 / 参数定义**：无新接口，验证现有 TF。

**4. 逻辑与交互**：M2 启动 slam_toolbox 后才会有 `map→odom`；M1 阶段先确认 `odom→base_link→传感器` 完整。

**5. 自检**：`ros2 run tf2_ros tf2_echo odom base_link` 持续输出；`view_frames` 生成的 PDF 里无孤立 frame。

---

## M2 SLAM 建图

### C2.1 — slam_toolbox 在线异步建图

**1. 功能**：边驱动边实时构建 occupancy grid 地图，并提供 `map→odom` TF。

**2. 为什么要有他**：PDF Task 2 核心要求“explore an unknown area, mapping it as you drive”。后续探索、Nav2、任务 8 的路径规划都依赖这张地图。

**3. 接口 / 参数定义**：
- 包：`slam_toolbox`（`apt install ros-jazzy-slam-toolbox`）
- 配置：`src/auto_nav_part3/config/slam_toolbox.yaml`
- 输入：`/scan` (`sensor_msgs/LaserScan`)、TF `odom→base_link`
- 输出：`/map` (`nav_msgs/OccupancyGrid`)、TF `map→odom`、`/slam_toolbox/...` 服务
- 关键参数：`mode: mapping`、`odom_frame: odom`、`map_frame: map`、`base_frame: base_link`、`scan_topic: /scan`、`resolution: 0.05`
- 新增 ROS 接口写进 `docs/TOPICS.md`：`/map` (OccupancyGrid, owner=Mapping)

**4. 逻辑与交互**：`/scan`+TF → slam_toolbox → `/map`+`map→odom`。下游：Nav2 用 `/map` 做全局规划，exploration_node 用 `/map` 找 frontier，UI/report 用 `/map` 出图。

**5. 自检**：
```bash
ros2 launch auto_nav_part3 sim_bringup.launch.py
# 启动 slam_toolbox，rviz2 加 Map 显示
# 遥控走一圈，地图实时生长，闭环时不错位
```

---

## M3 Nav2 导航栈（维持 Part 1 自主导航能力）

### C3.1 — Nav2 bringup + 参数

**1. 功能**：提供全局/局部路径规划、避障、恢复行为，实现“给目标点自主到达”。

**2. 为什么要有他**：PDF Task 1 要求 Part 3 维持 Part 1 的自主导航能力（避开未知静态障碍、到达指定位姿）。任务 8 的快速路点驾驶也复用 Nav2。

**3. 接口 / 参数定义**：
- 包：`nav2_bringup`（`apt install ros-jazzy-navigation2 ros-jazzy-nav2-bringup`）
- 配置：`src/auto_nav_part3/config/nav2_params.yaml`
- 输入：`/map`、`/scan`、TF、`/odometry/filtered`
- 接口：
  - 订阅目标：`/goal_pose` (`geometry_msgs/PoseStamped`)
  - Action：`/navigate_to_pose`、`/navigate_through_poses` (`nav2_msgs/action`)
  - 输出：`/cmd_vel`
- 关键参数：`controller_frequency`、`global_costmap/robot_radius`(按 Pioneer ≈0.3)、`local_costmap`、`DWB` 或 `RPP` 控制器，速度上限按 Pioneer 实际（≈0.7 m/s）

**4. 逻辑与交互**：
- SLAM 建图阶段 Nav2 与 slam_toolbox 共用 `map`（用 SLAM 的 map 而非 amcl）
- 探索阶段：exploration_node 通过 `/navigate_to_pose` action 把 frontier 当目标点发给 Nav2
- Nav2 输出 `/cmd_vel` —— ⚠️ 与 `safety_monitor`(Member 3) 都发 `/cmd_vel`，约定：safety 发零速优先级最高（M3 阶段先单独测，集成时用 twist_mux 仲裁，写进 TOPICS.md）

**5. 自检**：RViz 用 “Nav2 Goal” 点一个目标，机器人自主规划路径、绕开障碍到达。

---

### C3.2 — 本地控制器验证（Part 1 Task 5） (这个Part3不需要，不做了)

**1. 功能**：确认机器人能到达目标点的**正确位置和朝向**。

**2. 为什么要有他**：PDF Part 1 Task 5 明确要求“ending in the correct position and orientation”。Nav2 默认 controller 通常够用，自写 controller 是加分项（Complexity/Innovation）。

**3. 接口 / 参数定义**：复用 C3.1 的 `/navigate_to_pose`；`goal_checker` 的 `xy_goal_tolerance`、`yaw_goal_tolerance` 调严。

**4. 逻辑与交互**：发带朝向的 PoseStamped → Nav2 → 到点后 yaw 误差在容差内。

**5. 自检**：发 3 个不同朝向目标，到点后 `tf2_echo map base_link` 的 yaw 与目标一致（±容差）。

---

## M4 自主探索 (Autonomous Exploration)

### C4.1 — exploration_node（frontier-based，限制 15×15m）

**1. 功能**：自动选择未探索边界(frontier)作为目标，驱动机器人覆盖 15×15m，完成后停止。

**2. 为什么要有他**：PDF Task 2 要求“从 home 出发自主探索”，不能靠遥控。这是 Part 3 30% 分数的核心功能。

**3. 接口 / 参数定义**：
- 新建 `src/auto_nav_part3/auto_nav_part3/exploration_node.py`
- 输入：`/map` (`OccupancyGrid`)、TF `map→base_link`
- 输出：调用 Nav2 `/navigate_to_pose` action
- 发布进度：`/part3/mapping/map_status` (`std_msgs/String`)（**已在 TOPICS.md，归 Mapping**），格式如 `coverage=68% frontiers=3 area=15x15`
- 参数（`config/exploration.yaml`）：`search_area_size=15.0`、`home_x/home_y`、`min_frontier_size=0.5`、`coverage_done_threshold=0.95`

**4. 逻辑与交互**：
1. 订阅 `/map` → 提取 frontier（free 与 unknown 交界 cell 聚类）
2. 过滤掉 home 为中心 15×15m 边界外的 frontier
3. 选最近/信息增益最大的 frontier → `/navigate_to_pose` 发给 Nav2
4. 到达/超时 → 重新选 frontier
5. 无有效 frontier 或覆盖率达标 → 发布 `coverage=done`，停止
6. 与 `mapping_service`(C5.1)：service 收到 `/part3/mapping/start` 后才激活本节点（用参数/topic 开关，不要硬耦合）

**5. 自检**：sim 里不碰遥控，调用 mapping start，机器人自动跑遍区域，`/map` 基本填满后停止。

运行
```
  ros2 topic pub --once /part3/exploration/enable std_msgs/Bool '{data: false}'
  ros2 topic pub --once /part3/exploration/enable std_msgs/Bool '{data: true}'
```
---

### C4.2 — map_manager（地图保存 + artifact）

**1. 功能**：探索结束时把地图存成文件（pgm/yaml + png），供 UI、报告、任务 8 复用。

**2. 为什么要有他**：PDF Task 8 第二趟要在“已知地图”上规划最快路径；报告也要地图截图作为证据。

**3. 接口 / 参数定义**：
- 新建 `src/auto_nav_part3/auto_nav_part3/map_manager.py`
- 输入：`/map`，slam_toolbox 的 `serialize_map` / `map_saver` 服务
- 输出：`artifacts/maps/discovery_map.{pgm,yaml,png}`
- 新增 service 写进 `TOPICS.md`：`/part3/mapping/save_map` (`std_srvs/Trigger`)

**4. 逻辑与交互**：探索完成 → exploration_node 触发或 UI 调 `/part3/mapping/save_map` → 调 `nav2_map_server` 的 map_saver 落盘 → 路径回报到 `/part3/mapping/map_status`。

**5. 自检**：探索后 `artifacts/maps/` 出现地图文件，`ros2 run nav2_map_server map_server` 能重新加载。

---

## M5 mapping_service 实装

### C5.1 — 把占位 mapping_service 接到真实 SLAM/探索

**1. 功能**：`/part3/mapping/start` 被调用时，真正激活 SLAM + exploration_node，并维护状态。

**2. 为什么要有他**：PDF Task 10 要求两个阶段(mapping/waypoint)做成可一键切换的 service，且不重启软件栈。这是 UI(Member 3) 触发你能力的唯一入口。

**3. 接口 / 参数定义**（对齐现有 `mapping_service.py` 与 TOPICS.md）：
- Service：`/part3/mapping/start` (`std_srvs/Trigger`) —— 已存在，保留签名
- 发布：`/part3/system/state` = `MAPPING`、`/part3/mapping/map_status`
- 不改 service 名/类型（Member 3 的 UI 依赖它）

**4. 逻辑与交互**：
1. 收到 start → 置状态 `MAPPING` → 激活 exploration_node（发开关 topic 或 set_parameter）
2. 转发 exploration 进度到 `/part3/mapping/map_status`
3. 探索完成 → 触发 map_manager 存图 → 状态回 `IDLE`/`COMPLETE`
4. 不直接在本节点写 SLAM 逻辑 —— 只做编排，实算在 exploration_node
5. ⚠️ 保持 `response.success=true` 语义为“命令已接受”，不是“已完成”（TOPICS.md 契约）

**5. 自检**：
```bash
ros2 launch auto_nav_part3 sim_bringup.launch.py   # + slam + nav2 + exploration
ros2 service call /part3/mapping/start std_srvs/srv/Trigger {}
ros2 topic echo /part3/system/state        # MAPPING
ros2 topic echo /part3/mapping/map_status  # coverage 递增
```

### M5 启动顺序

```
 t=0 s   gz server + robot_state_publisher + bridge
 t=3 s   ekf_node（odom→base_link TF）
 t=10 s  slam_toolbox（unconfigured）
 t=10.5s slam configure → activate（/map + map→odom TF 开始发布）
 t=15 s  nav2_bringup（use_nav2:=true）
 t=45 s  exploration_node + map_manager（use_exploration:=true）
            exploration_node 启动时 _active=false，等待激活信号

 随时    mapping_service（独立节点，随 sim_bringup 一同启动或单独启动）

 用户/UI → ros2 service call /part3/mapping/start ...
         → mapping_service 立即发 enable=true → exploration_node 开始探索
```

### M5 通信图

```
  UI / CLI
     │
     │  /part3/mapping/start (Trigger)
     ▼
┌──────────────────────┐
│   mapping_service    │──── /part3/system/state = "MAPPING" ──► 状态订阅者
│   (编排，无逻辑)     │
└──────────┬───────────┘
           │ /part3/exploration/enable = true
           ▼
┌──────────────────────┐     /navigate_to_pose (Action)
│   exploration_node   │ ──────────────────────────────► Nav2
│   (frontier 探索)    │
└──────────┬───────────┘
           │ /part3/mapping/map_status
           │  "coverage=68% frontiers=3 area=15x15"
           │  ...
           │  "coverage=done coverage_pct=97%"
           │
     ┌─────┴──────────────────────────────────────┐
     ▼                                            ▼
┌──────────────────────┐              ┌──────────────────────┐
│   mapping_service    │              │    map_manager       │
│   检测 coverage=done │              │  检测 coverage=done  │
│   → state=COMPLETE   │              │  → 自动保存地图文件  │
└──────────────────────┘              │  (.pgm / .yaml / .png│
                                      └──────────────────────┘
```

**数据流说明**：
- `mapping_service` 只做开关：收到 start → `enable=true`，检测到 done → `enable=false` + `COMPLETE`
- `exploration_node` 执行真正的 frontier 算法，进度发布到 `/part3/mapping/map_status`
- `map_manager` 独立监听 `map_status`，自动保存，无需 `mapping_service` 显式调用
- 三节点松耦合，任何一个可单独重启而不影响其他

---

## M_P 感知集成 (Perception Integration — T3/T4)

> 队友负责模型代码，你负责把他的节点集成进 launch，并统一接口格式供 T8 路点服务消费。

### C_P.1 — perception_adapter 接口适配节点

**1. 功能**：把队友感知节点的输出统一成 `/part3/perception/marker_event` 格式，并在内存里维护一份 marker 列表供 T8 查询。

**2. 为什么要有他**：
- 队友发布的格式可能与 TOPICS.md 约定不完全一致（消息类型、坐标系、字段名）。
- T8 需要按 marker 坐标规划路径，需要一个可靠的"已知 marker 列表"服务。
- 适配节点做格式转换，队友代码和路点服务互不依赖对方内部实现。

**3. 接口 / 参数定义**：
- **新建**：`src/auto_nav_part3/auto_nav_part3/perception/perception_adapter.py`
- 输入（来自队友节点，按实际确认后填写）：
  - 队友发布话题（待确认后替换）→ 转换为统一格式
- 输出（标准接口，写进 `TOPICS.md`）：
  - `/part3/perception/marker_event`（`std_msgs/String`，保留兼容）
  - `/part3/perception/markers`（`geometry_msgs/PoseArray`，新增，供 T8 直接读取）
- 服务（新增，写进 `TOPICS.md`）：
  - `/part3/perception/get_markers`（`std_srvs/Trigger`）→ 通过 `/part3/perception/markers` 话题回应最新 marker 列表
- marker 事件字符串格式约定：
  ```
  type=greek_letter label=alpha x=3.2 y=-1.5 confidence=0.92
  type=colour_obstacle colour=yellow x=1.0 y=4.5
  ```
- 参数（`config/perception.yaml`）：
  - `map_frame: map`（所有坐标必须转换到 map 帧后存储）

**4. 逻辑与交互**：
1. 启动时订阅队友节点的输出话题
2. 收到检测结果 → 查询 TF 把像素/相机坐标转为 map 帧 (x, y)
3. 去重：若新 marker 与已有 marker 距离 < 0.5m，视为同一目标，更新 confidence
4. 发布到 `/part3/perception/marker_event`（字符串，兼容现有订阅者）
5. 同时维护 `_markers` 列表（`{type, label, x, y}`），发布到 `/part3/perception/markers`
6. T8 的 `waypoint_service` 启动前读取此列表选出希腊字母 waypoint

**5. 集成步骤**：
```bash
# 1. 把队友代码拷贝进 src/auto_nav_part3/auto_nav_part3/perception/
# 2. 在 setup.py console_scripts 注册队友节点和 perception_adapter
# 3. sim_bringup.launch.py 添加两个节点
# 4. 确认 /part3/perception/markers 有输出
ros2 topic echo /part3/perception/markers
ros2 topic echo /part3/perception/marker_event
```

**6. 自检**：
```bash
# 仿真中机器人经过一个 marker → adapter 在 30s 内发布该 marker 的 map 坐标
# 同一 marker 多次检测 → 列表里只有 1 条记录（去重成功）
ros2 topic echo /part3/perception/marker_event
```

---

## M_S 安全与记录 (Safety, Estop & Recording — T5/T6/T9)

### C_S.1 — safety_monitor 升级：移动障碍急停（T5）

**1. 功能**：检测向机器人接近的移动障碍物，在其进入 1m 范围时触发软件急停（发零速），并发布急停事件。

**2. 为什么要有他**：
- PDF Task 5 明确要求"moving object comes within 1m → software estop"。
- Nav2 已处理静态障碍，safety_monitor 专门补充移动障碍这一层。
- 现有 `safety_monitor.py` 是占位代码：对静态障碍也误触发、无冷却、无移动检测。

**3. 接口 / 参数定义**：
- 修改：`src/auto_nav_part3/auto_nav_part3/safety/safety_monitor.py`
- 输入：`/scan`（`sensor_msgs/LaserScan`）
- 输出：
  - `/cmd_vel`（`geometry_msgs/Twist`，零速覆盖）—— 急停时发布
  - `/part3/safety/estop_event`（`std_msgs/String`）—— 事件记录
- 参数（`config/safety.yaml`，新建）：
  - `estop_distance_m: 1.0`（急停触发距离）
  - `moving_delta_m: 0.15`（相邻帧变化超过此值视为移动）
  - `consecutive_frames: 3`（需连续 N 帧确认，防误报）
  - `estop_cooldown_sec: 2.0`（急停后冷却时间，防止连续触发录包）
  - `publish_rate_hz: 10.0`（急停期间持续发零速频率）

**4. 逻辑与交互（移动检测算法）**：
```
每帧 /scan 到达时：
  1. 对每个 angle i：
       delta[i] = prev_range[i] - curr_range[i]   # 正值 = 距离缩短 = 靠近
  2. moving_close = {i : delta[i] > moving_delta_m AND curr_range[i] < estop_distance_m}
  3. 若 moving_close 非空：
       confirm_count += 1
     否则：
       confirm_count = 0（重置）
  4. 若 confirm_count >= consecutive_frames AND 冷却已过：
       发布零速 /cmd_vel（定时器持续发，直到障碍消失）
       发布 estop_event（含时间戳、最近距离、方位角）
       重置冷却计时
  5. 更新 prev_ranges = curr_ranges
```

急停事件格式：
```
software_estop timestamp=1234567890.123 min_dist=0.72 bearing_deg=45 save_last_5s=true
```

**5. twist_mux 仲裁**：
- 急停零速必须高于 Nav2 的 `/cmd_vel` 优先级
- 方案：`safety_monitor` 发布到 `/cmd_vel_safety`，`nav2` 发布到 `/cmd_vel_nav`，`twist_mux` 合并输出 `/cmd_vel`（优先级 safety > nav2 > teleop）
- `config/twist_mux.yaml` 配置三路优先级

**6. 自检**：
```bash
# 手动推障碍到 1m 内（仿真：生成一个动态 actor 或 teleop 另一个机器人）
ros2 topic echo /part3/safety/estop_event
ros2 topic echo /cmd_vel  # 应出现全零 Twist
```

---

### C_S.2 — rolling_recorder：5 秒滚动录包（T6）

**1. 功能**：始终在内存中保留最近 5 秒的传感器和系统数据；收到急停事件时立即把这 5 秒数据写到磁盘。

**2. 为什么要有他**：PDF Task 6 要求"save the last 5 seconds of recorded data"供团队事后回看——事故回放必须在急停触发 **之前** 就开始缓存。

**3. 接口 / 参数定义**：
- **新建**：`src/auto_nav_part3/auto_nav_part3/safety/rolling_recorder.py`
- 订阅（缓存）：`/scan`、`/camera`、`/odometry/filtered`、`/tf`、`/part3/system/state`、`/part3/safety/estop_event`
- 触发：订阅 `/part3/safety/estop_event`，收到后写盘
- 输出：`artifacts/bags/estop_<timestamp>.bag3/`（rosbags 格式）
- 参数（`config/safety.yaml` 追加）：
  - `buffer_duration_sec: 5.0`（滚动窗口大小）
  - `bag_save_dir: artifacts/bags/`

**4. 逻辑与交互**：
```
初始化：
  _buffer = deque()  # 元素: (topic, msg, timestamp)
  订阅以上话题，每条消息 append 到 _buffer
  定时器：每 0.1s 清理 _buffer 中早于 (now - 5s) 的条目

收到 estop_event：
  snapshot = list(_buffer)  # 拷贝当前 5s 窗口
  在线程中写入 rosbags Writer → artifacts/bags/estop_<ts>/
  日志：'[RollingRecorder] 已保存 estop bag: ...'
```

实现方式（推荐 rosbags Python 库）：
```bash
pip install rosbags   # 或 apt install python3-rosbags
```

**5. 自检**：
```bash
# 触发急停后：
ls artifacts/bags/estop_*/
ros2 bag info artifacts/bags/estop_<timestamp>/
# 应看到 /scan, /camera 等话题，duration ≈ 5s
```

---

### C_S.3 — 全程录包集成（T9）

**1. 功能**：机器人运行期间全程录包，供离线回放、答辩演示、报告截图使用。

**2. 为什么要有他**：PDF Task 9 要求"record the drives so it can be reviewed again offline"。

**3. 接口 / 参数定义**：
- 不新建节点，在 `sim_bringup.launch.py` 中加一个 `ExecuteProcess` 启动 `ros2 bag record`
- 录制话题（关键，不录全部避免磁盘爆满）：
  ```
  /map /scan /camera /odometry/filtered /tf /tf_static
  /cmd_vel /part3/system/state /part3/mapping/map_status
  /part3/safety/estop_event /part3/perception/marker_event
  /navigate_to_pose/_action/status
  ```
- 输出：`artifacts/bags/session_<launch_time>/`
- Launch 参数：`record_bag:=true/false`（默认 true）

**4. 逻辑与交互**：
```python
# sim_bringup.launch.py 追加：
bag_recorder = ExecuteProcess(
    condition=IfCondition(record_bag),
    cmd=['ros2', 'bag', 'record',
         '-o', f'artifacts/bags/session_{timestamp}',
         '/map', '/scan', '/camera', ...],
    output='screen',
)
```
录包进程与 launch 生命周期绑定，launch 停止时录包自动结束。

**5. 自检**：
```bash
ls artifacts/bags/session_*/
ros2 bag info artifacts/bags/session_<ts>/
ros2 bag play artifacts/bags/session_<ts>/   # RViz 内回放验证
```

---

## M_W 路点快速驾驶 (Waypoint Service — T8)

### C_W.1 — waypoint_service 实装：最快路径规划 + Nav2 顺序导航

**1. 功能**：
- 第一趟探索完成后，接收 3 个目标坐标（来自 Member 2 的 marker 检测）
- 用暴力枚举 TSP 找最短访问顺序（3个点只有 6 种排列，毫秒级）
- 调用 Nav2 `NavigateThroughPoses` 按最优顺序到达 3 点后返回 home
- 全程发布规划路径供 UI 显示

**2. 为什么要有他**：PDF Task 8 核心分数：第二趟在已知地图上尽可能快地访问 3 个 waypoint 并回家。`waypoint_service.py` 目前是占位符，需要接真实逻辑。

**3. 接口 / 参数定义**：
- 修改：`src/auto_nav_part3/auto_nav_part3/navigation/waypoint_service.py`
- 触发服务：`/part3/waypoint/start`（`std_srvs/Trigger`）—— 已存在，保留签名
- Waypoint 来源：在调用 start 前，由 Member 2 节点通过以下方式提供（按约定选一）：
  - **方案 A**（推荐）：从 `/part3/perception/markers` 话题读取，过滤 type=greek_letter 的条目
  - **方案 B**：通过 ros2 param set 在调用前设置 waypoints 参数
- 输出：
  - `/part3/waypoint/plan`（`std_msgs/String`）—— 发布排序后的路径描述
  - `/part3/system/state`（`std_msgs/String`）—— `WAYPOINT_DRIVE` / `COMPLETE`
- Action 客户端：`/navigate_through_poses`（`nav2_msgs/action/NavigateThroughPoses`）
- 参数（`config/waypoint.yaml`，新建）：
  - `home_x: -3.0`，`home_y: 0.0`（出发/返回原点，与 spawn 一致）
  - `nav_timeout_sec: 120.0`（单次导航超时）

**4. 逻辑与交互**：
```
收到 /part3/waypoint/start：
  1. 从 /part3/perception/markers 读取所有 type=greek_letter 的 marker
     （最多等待 3s；若无 marker，返回 success=False）
  2. 暴力枚举 3 个 marker 的所有 6 种访问顺序
     对每种顺序计算总路程 = Σ euclidean(home→p1→p2→p3→home)
     选路程最短的顺序 → best_order
  3. 发布 /part3/waypoint/plan（格式：home→α(3.2,-1.5)→β(1.0,4.5)→γ(-2.0,2.3)→home  dist=18.4m）
  4. 发布 state = WAYPOINT_DRIVE
  5. 构建 PoseStamped 列表：[best_order[0], best_order[1], best_order[2], home]
  6. 调用 /navigate_through_poses action（异步）
  7. 等待结果（回调）：
     成功 → state = COMPLETE，日志"已返回 home"
     失败 → state = WAYPOINT_FAILED，日志含错误
  8. 立即返回 response.success=True（"命令已接受"，导航异步进行）
```

TSP 暴力枚举（3点 = 6种排列）：
```python
from itertools import permutations
import math

def best_order(home, markers):
    # markers: [(x,y), (x,y), (x,y)]
    def dist(a, b): return math.hypot(a[0]-b[0], a[1]-b[1])
    def total(order):
        pts = [home] + list(order) + [home]
        return sum(dist(pts[i], pts[i+1]) for i in range(len(pts)-1))
    return min(permutations(markers), key=total)
```

**5. 通信图**：
```
Member 2 节点
  └─► /part3/perception/markers (PoseArray)
          │
          ▼
  waypoint_service (收到 /part3/waypoint/start)
  ├─► 读取 markers → TSP 排序 → /part3/waypoint/plan
  ├─► /part3/system/state = "WAYPOINT_DRIVE"
  └─► /navigate_through_poses (Action) ──► Nav2
                                              └─► /cmd_vel ──► 机器人
```

**6. 自检**：
```bash
# 手动发 3 个 marker（模拟 Member 2 输出）：
ros2 topic pub --once /part3/perception/markers geometry_msgs/msg/PoseArray \
  '{poses: [{position: {x: 3.0, y: 1.0}}, {position: {x: -2.0, y: 3.0}}, {position: {x: 1.5, y: -2.5}}]}'

# 触发第二趟：
ros2 service call /part3/waypoint/start std_srvs/srv/Trigger {}

# 观察：
ros2 topic echo /part3/waypoint/plan    # 输出最短路径顺序
ros2 topic echo /part3/system/state     # WAYPOINT_DRIVE → COMPLETE
# RViz 里看机器人按顺序到达 3 个点后回到 home
```

---

## M6 物理机器人部署 (Physical Bringup)

> 留到实验室前 1 周。原则：**同一套上层节点(SLAM/Nav2/exploration)不变，只换最底层传感器/底盘驱动**。

### C6.1 — Pioneer 底盘驱动

**1. 功能**：把 `/cmd_vel` 转成真实 Pioneer 电机命令，并发布真实 `/odom`。

**2. 为什么要有他**：仿真用 Gazebo diff-drive 插件，真机需要 ARIA/rosaria 驱动；这是 sim→real 的关键替换点。

**3. 接口 / 参数定义**：
- 驱动：ARIA (`github.com/cinvesrob/Aria`) + ROS2 包装节点
- 订阅 `/cmd_vel`，发布 `/odom` + `odom→base_link` TF（接口与 sim 完全一致）
- 参数：串口设备、波特率、轮距/轮径（与 URDF 一致：`wheel_separation=0.394`、`wheel_radius=0.111`）

**4. 逻辑与交互**：上层节点零改动，因为 topic 契约和 sim 一致。EKF 输入 `/odom` 来源从 Gazebo 换成真机驱动。

**5. 自检**：低速 teleop（室内务必按 PDF 用胶带缠轮防烧电机），`/odom` 随真实移动更新。

---

### C6.2 — SICK Lidar 驱动

**1. 功能**：发布真实 `/scan`。
**2. 为什么**：sim 的 `gpu_lidar` 换成真实 `sick_scan_xd`，话题名保持 `/scan` 不变。
**3. 接口/参数**：包 `sick_scan_xd`；输出 `/scan` (`LaserScan`)；参数：雷达 IP、`frame_id=laser_frame`（与 URDF 一致）。
**4. 逻辑与交互**：SLAM/Nav2/safety 全部零改动。
**5. 自检**：`ros2 topic echo /scan --once` 距离值合理；RViz LaserScan 与实物障碍吻合。

---

### C6.3 — 修复 oakd_camera.py 重复内容 bug

**1. 功能**：OAK-D 相机节点发布 `/oak/rgb/image_raw`、`/oak/stereo/depth`。

**2. 为什么要有他**：现文件 `bridge_2_robot/oakd_camera.py` **整段代码粘贴了两遍**（两个 `main()` / 两个 class），import 会报错或行为异常。必须删掉重复段。

**3. 接口 / 参数定义**：
- 发布：`/oak/rgb/image_raw` (`sensor_msgs/Image`)、`/oak/stereo/depth` (`sensor_msgs/Image`)
- 与 sim 的差异：sim 相机话题是 `/camera/image_raw`；建议给 Member 2 统一一个 remap，真机/ sim 都映射到同一逻辑话题（写进 TOPICS.md）
- 依赖：`depthai`（pip）

**4. 逻辑与交互**：只保留**前半段一份完整实现**，删除第二份重复（从第二个 `#!/usr/bin/env python3` 起到文件尾）。Member 2 的感知节点消费这些 image 话题。

**5. 自检**：`python3 -c "import ast; ast.parse(open('.../oakd_camera.py').read())"` 无语法问题；文件里只有一个 `class OakDCamera` 和一个 `main`。

---

### C6.4 — sim / real 切换机制

**1. 功能**：用一个 launch 参数 `use_sim:=true/false` 切换传感器/底盘来源，上层不变。

**2. 为什么要有他**：避免维护两套 launch 导致漂移；PDF 要求先 sim 验证再上真机，切换必须低成本。

**3. 接口 / 参数定义**：
- `launch/bringup.launch.py` 顶层参数 `use_sim`（默认 true）
- `use_sim=true` → 启动 Gazebo + bridge（C0.2）
- `use_sim=false` → 启动 C6.1/6.2/6.3 真机驱动
- 两边对上层暴露**完全相同的 topic/TF**

**4. 逻辑与交互**：SLAM/Nav2/exploration/mapping_service 的 launch 片段共用，只有底层 include 不同。

**5. 自检**：`use_sim:=true` 与 `false` 两种启动下，`ros2 topic list` 中 `/scan /odom /imu` 都存在且类型一致。

---

## M7 集成与验证 (Integration)

### C7.1 — 全栈联调

**1. 功能**：一条 launch 跑通 sim 地基 + EKF + SLAM + Nav2 + exploration + mapping_service。

**2. 为什么要有他**：分模块都过不代表集成过；TF 双发布、`/cmd_vel` 多源、话题频率不匹配等 bug 只在集成暴露。

**3. 接口 / 参数定义**：`launch/part3_full_sim.launch.py` 组合所有上述组件。

**4. 逻辑与交互**：启动顺序 Gazebo → robot_state_publisher → bridge → EKF → slam_toolbox → Nav2 → exploration → mapping_service → (UI/safety 由 Member 3)。

**5. 自检（Demo 脚本）**：
```bash
ros2 launch auto_nav_part3 part3_full_sim.launch.py
ros2 service call /part3/mapping/start std_srvs/srv/Trigger {}
# 观察：机器人自主探索 → /map 生长 → 覆盖完停止 → artifacts/maps 出图
ros2 bag record -a -o artifacts/bags/demo_$(date +%s)   # 存证供报告
```

### C7.2 — 把问题与决策记录到 docs/qa.md

每解决一个“查了很久的 bug”，在 `docs/qa.md` 追加：现象 / 根因 / 解决。这是报告“Software design description”和答辩问答的素材。

---

## 附录 A：常见“查不出的 bug”对照表

| 现象 | 最可能根因 | 排查命令 |
|---|---|---|
| RViz 无机器人模型 | mesh 路径未改 (C0.1) | 看 robot_state_publisher 日志 |
| `/scan` 无数据 | gpu_lidar / bridge 没桥 | `ros2 topic hz /scan` |
| 地图错位/漂移 | TF 双发布 odom→base_link (C1.1) | `ros2 run tf2_tools view_frames` |
| Nav2 不动 | `map→odom` 缺失 (SLAM 没起) | `ros2 run tf2_ros tf2_echo map odom` |
| 机器人乱转 | `/cmd_vel` 多源冲突 (C3.1) | `ros2 topic info /cmd_vel -v` |
| service 调了没反应 | 占位逻辑没替换 (C5.1) | 看节点日志 |
| 导入报错 | oakd_camera 重复段 (C6.3) | `python3 -m py_compile` |

## 附录 B：依赖安装清单 (Ubuntu 24.04 / Jazzy)

```bash
sudo apt update && sudo apt install -y \
  ros-jazzy-ros-gz-sim ros-jazzy-ros-gz-bridge \
  ros-jazzy-slam-toolbox ros-jazzy-navigation2 ros-jazzy-nav2-bringup \
  ros-jazzy-robot-localization ros-jazzy-teleop-twist-keyboard \
  ros-jazzy-tf2-tools ros-jazzy-nav2-map-server
# 真机额外：depthai (pip), sick_scan_xd (源码编译), ARIA
```

## 附录 C：与其他成员的接口边界（对齐 TOPICS.md）

- 你 **发布**：`/map`、`/part3/mapping/map_status`、`/part3/system/state(=MAPPING时)`、`/odometry/filtered`
- 你 **提供 service**：`/part3/mapping/start`、`/part3/mapping/save_map`(新增需写 TOPICS.md)
- 你 **消费**：`/part3/waypoint/start` 触发后让出 `/cmd_vel` 控制权（与 Member 3 约定 twist_mux 仲裁）
- 任何跨边界改动：**先改 `docs/TOPICS.md` 再编码**，并跑 `pytest src/auto_nav_part3/test/test_docs_contract.py`
