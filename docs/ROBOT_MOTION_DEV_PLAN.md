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
