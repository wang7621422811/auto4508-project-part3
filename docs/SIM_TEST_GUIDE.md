# 仿真全流程测试指南（物理部署前）

> **用途**：在把代码烧到真机之前，按本指南逐层验证每个里程碑。
> 每个阶段都有明确的通过标准（Pass Criteria）和常见失败根因，
> 只有当前阶段全部通过才进入下一阶段。
>
> **顺序**：构建检查 → M0 仿真地基 → M1 定位 → M2 SLAM →
>           M3 Nav2 → M_S 安全系统 → M_P 感知集成 → M4 探索 →
>           M5 服务编排 → M_W 路点导航 → 全链路集成
>
> **环境**：Parallels + Ubuntu 24.04 (ARM64)，ROS2 Jazzy，Gazebo Harmonic

---

## 公共前置准备（每次测试前执行）

```bash
# 进入项目根目录（所有命令的工作目录）
cd /home/parallels/workspace/projects/auto4508-project-part3

# 重新构建（确保最新代码生效；--symlink-install 让 Python 改动立即可见）
colcon build --symlink-install 2>&1 | tail -20

# Source 环境（每个新终端都要执行）
source install/setup.bash

# 确认包已安装
ros2 pkg list | grep auto_nav_part3
```

**预期**：`auto_nav_part3` 出现在列表中，`colcon build` 无 `ERROR`。

---

## 阶段 0 — 仿真地基 (M0)

> **目标**：Gazebo world 正常启动，机器人能被键盘遥控，所有基础话题就绪。
> 这是后续所有阶段的地基，不通则后续全错。

### 0-A 启动最小仿真（无 SLAM / Nav2 / 探索）

```bash
# 终端 1：启动仿真（关闭上层功能，只验证地基）
./scripts/launch.sh start --clean sim_bringup \
    use_slam:=false \
    use_nav2:=false \
    use_exploration:=false \
    use_safety:=false \
    use_camera:=false \
    use_rviz:=true
```

等待约 5s，观察：
- Gazebo GUI 弹出，显示 15×15m arena 和 9 个 box 障碍
- RViz 弹出，Robot Model 能看到 Pioneer 机器人模型（如黑屏用 `LIBGL_ALWAYS_SOFTWARE=1`）

### 0-B 基础话题验证

```bash
# 终端 2：检查基础话题频率（等 Gazebo 完全启动后执行）
source install/setup.bash

ros2 topic hz /scan        # 预期 ~10 Hz
ros2 topic hz /odom        # 预期 ~20 Hz
ros2 topic hz /imu         # 预期 ~100 Hz
ros2 topic hz /camera/image  # 预期 ~10 Hz（use_camera=false 时可无）
```

```bash
# 检查 spawn 位置是否正确（机器人应在 x≈-3.0, y≈0.0）
ros2 topic echo /odom --once | grep -A3 "position:"
```

### 0-C TF 静态帧完整性

```bash
ros2 run tf2_tools view_frames
# 查看生成的 frames.pdf，必须包含：
# base_link → laser_frame（或 chassis → laser_frame）
# base_link → imu_link
# base_link → cam_optical_link
```

### 0-D 遥控闭环验证

```bash
# 终端 3：遥控
ros2 launch auto_nav_part3 teleop.launch.py
```

驾驶机器人：
1. **直线行驶约 1m**：`/odom` 的 x 变化 ≈1.0m，y 漂移 < 0.05m
2. **原地旋转一圈**：`ros2 run tf2_ros tf2_echo odom base_link` 的 yaw 回到初始值 ±15°
3. **RViz 里 odom 轨迹跟随**（Fixed Frame = `odom`）

### M0 通过标准 ✅

| 检查项 | 预期 |
|--------|------|
| `/scan /odom /imu` 有数据 | 频率稳定，无 0Hz |
| spawn 位置 | x ≈ -3.0, y ≈ 0.0 (±0.1m) |
| TF 树 | 无孤立帧，laser_frame / imu_link 已连接 |
| 遥控直线 1m | x 变化 0.9–1.1m |
| 遥控旋转 360° | yaw 误差 < 15° |

**已知坑**（参见 `docs/DEBUG_PLAN.md`）：
- `/odom` 无数据 → `ros_gz_bridge` 没起，看 launch 日志
- TF 树出现两个 `odom→base_link` 来源 → EKF 和 diff-drive 冲突，检查 URDF `<tf_topic>` 是否改为 `/gz/tf_not_bridged`（Bug B7）
- 空仿真世界里 SLAM 之后地图很小 → 需要确认 9 个 box 障碍在 SDF 里有碰撞几何（Bug B5）

---

## 阶段 1 — EKF 定位 (M1)

> **目标**：EKF 输出平滑的 `/odometry/filtered`，只有 EKF 一个来源发布 `odom→base_link` TF。

```bash
# 在阶段 0 命令基础上，EKF 默认已在 sim_bringup 中开启
# 只需确认输出
source install/setup.bash

# 检查 EKF 输出
ros2 topic hz /odometry/filtered        # 预期 ~30 Hz
ros2 topic echo /odometry/filtered --once | head -20

# 确认 TF 只有 EKF 一个来源（不应出现 diff-drive 作为 odom→base_link 发布者）
ros2 topic info /tf -v | grep -A2 "Publisher"
```

遥控机器人绕一个完整圆，用 RViz 观察（Fixed Frame = `odom`）：
- 轨迹平滑，无跳变
- 完整圆后 `/odometry/filtered` 的位置接近出发点（误差 < 0.3m）

### M1 通过标准 ✅

| 检查项 | 预期 |
|--------|------|
| `/odometry/filtered` 频率 | ~30 Hz |
| TF `odom→base_link` 来源数 | 1（仅 EKF） |
| 绕圆后回到起点误差 | < 0.3m |
| 静止时 EKF 输出 | pose 几乎不变（drift < 0.001m/s） |

**已知坑**：
- EKF 完全无输出 / 跳变 → 检查 `use_sim_time: True` 是否在所有节点设置（Bug B6）
- 两个 `odom→base_link` → URDF diff-drive `<tf_topic>` 没改（Bug B7）

---

## 阶段 2 — SLAM 建图 (M2)

> **目标**：slam_toolbox 在线建图，地图随驾驶实时生长，`map→odom` TF 稳定。

```bash
# 终端 1：启动仿真 + SLAM（关闭 Nav2 和探索，专注验证地图质量）
./scripts/launch.sh start --clean sim_bringup \
    use_slam:=true \
    use_nav2:=false \
    use_exploration:=false \
    use_rviz:=true

# 观察日志，约 10–13s 内应出现：
# [slam_lifecycle] configure OK
# [slam_lifecycle] activate OK
```

```bash
# 终端 2：验证 SLAM 输出
source install/setup.bash

# map 是否在发布（activate 后 ~2s 开始）
ros2 topic hz /map              # 预期 ~1 Hz

# map→odom TF 是否存在
ros2 run tf2_ros tf2_echo map odom --timeout 5
```

### 2-A 建图质量测试

在 RViz 中（Fixed Frame = `map`，添加 Map 显示）：

```bash
# 遥控沿墙边慢速行驶一圈（速度 ≤ 0.3 m/s）
ros2 launch auto_nav_part3 teleop.launch.py
```

| 实验 | 预期地图质量 |
|------|-------------|
| 贴墙直线行驶 | 每面墙 = 1 条干净细线，无平行幽灵墙 |
| 慢速原地旋转 | 旋转后墙线不散射、不变粗 |
| 绕一圈回原点 | 闭环处地图无明显错位（< 0.2m） |

### 2-B SLAM 生命周期问题检查

```bash
# 如果 /map 无数据，检查 slam_toolbox 状态
ros2 lifecycle get /slam_toolbox
# 应为 active；若为 unconfigured/inactive → bash 重试循环未成功
# 手动激活：
until ros2 lifecycle set /slam_toolbox configure 2>/dev/null; do sleep 0.5; done
sleep 0.5 && ros2 lifecycle set /slam_toolbox activate
```

### M2 通过标准 ✅

| 检查项 | 预期 |
|--------|------|
| `/map` 频率 | ~1 Hz（activate 后 2s 内开始） |
| `map→odom` TF | 存在，静止时 translation 漂移 < 0.02m/s |
| 直线行驶后墙线 | 单条，无平行幽灵 |
| 闭环误差 | < 0.3m |

**已知坑**：
- slam_toolbox 不订阅 `/scan` → 检查 `IfCondition(use_slam)` 是否加在 `TimerAction` 上而非内层 Node（Bug B1）
- configure/activate 失败 → DDS 发现超时；用 bash 重试循环（Bug B2）

---

## 阶段 3 — Nav2 导航 (M3)

> **目标**：在 RViz 里发布目标点，机器人能自主规划路径并到达，能绕开障碍。

```bash
# 终端 1：启动完整导航栈
./scripts/launch.sh start --clean sim_bringup \
    use_slam:=true \
    use_nav2:=true \
    use_exploration:=false \
    use_rviz:=true

# 等待 Nav2 完全启动（约 20–25s）
# 日志出现：[lifecycle_manager_navigation] Managed nodes are active
```

```bash
# 终端 2：确认 Nav2 Action Server 就绪
source install/setup.bash

ros2 action info /navigate_to_pose
# 预期：有 server，client 数为 0
```

### 3-A 点对点导航测试

在 RViz 的 Nav2 Panel 里，点击 **"Nav2 Goal"** 发布目标：

| 测试 | 目标位置 | 预期结果 |
|------|----------|----------|
| 近距离直线 | (+2.0, 0.0) | 直接到达，< 30s |
| 近距离旋转 | (+2.0, +2.0) | 规划弧线路径，绕开障碍 |
| 障碍物后方 | (+5.0, +3.0) | 绕开 ob_ne，到达目标 |
| 带朝向 | (−2.0, 0.0, yaw=90°) | 到达后 yaw ≈ 90° |

```bash
# 命令行发目标（可选）
ros2 topic pub --once /goal_pose geometry_msgs/msg/PoseStamped \
  '{header: {frame_id: "map"}, pose: {position: {x: 2.0, y: 0.0}, orientation: {w: 1.0}}}'
```

### 3-B Costmap 检查

在 RViz 中添加 `global_costmap/costmap` 和 `local_costmap/costmap`：

- global costmap：在障碍物位置有膨胀区，**无幽灵墙**（若有幽灵墙检查 `obstacle_layer marking: False`，Bug B5/DEBUG_PLAN 阶段 5）
- local costmap：实时更新，机器人前方障碍即时反映

### 3-C Controller 验证

```bash
# 观察控制器是否产生合理速度
ros2 topic echo /cmd_vel
# 导航期间应有非零线速度（≤ 0.5 m/s）和角速度
```

### M3 通过标准 ✅

| 检查项 | 预期 |
|--------|------|
| Nav2 Action Server | active |
| 近距离导航成功率 | 3/3 次到达 |
| 带朝向目标 yaw 误差 | < 0.1 rad |
| global costmap 无幽灵墙 | 仅障碍膨胀区 |
| `CONTROLLING_FAILED` 错误 | 不应出现（若出现检查 `transform_tolerance: 0.3`，Bug B4） |

---

## 阶段 S — 安全系统 (M_S)

> **目标**：移动障碍进入 1m 范围触发软件急停，5 秒滚动录包正确保存。
> 此阶段需 `use_safety:=true`。

```bash
# 终端 1：启动带安全系统的仿真
./scripts/launch.sh start --clean sim_bringup \
    use_slam:=true \
    use_nav2:=true \
    use_safety:=true \
    use_rviz:=true
```

### S-A 急停触发测试

```bash
# 终端 2：监听急停事件
source install/setup.bash
ros2 topic echo /part3/safety/estop_event &
ros2 topic echo /cmd_vel &
```

**测试方法**：用 `fake_scan_pub.py` 模拟靠近的障碍：

```bash
# 终端 3：发布模拟 scan（模拟障碍在 0.8m 处持续 3 帧）
python3 scripts/fake_scan_pub.py
# 或手动发近距离 scan 话题触发：
ros2 topic pub --rate 15 /scan sensor_msgs/msg/LaserScan \
  '{header: {frame_id: "laser_frame"}, angle_min: -0.3, angle_max: 0.3,
    angle_increment: 0.1, range_min: 0.1, range_max: 15.0,
    ranges: [0.75, 0.75, 0.75, 0.75, 0.75, 0.75, 0.75]}'
```

**预期**：
1. 约 3 帧（0.3s）后 `/part3/safety/estop_event` 出现消息
2. `/cmd_vel` 出现全零 Twist（linear.x=0, angular.z=0）
3. 停止模拟 scan → 约 `estop_cooldown_sec=2s` 后恢复

### S-B 滚动录包验证

```bash
# 触发急停后（约 3s 内），检查录包文件
ls -la artifacts/bags/estop_*/
ros2 bag info artifacts/bags/estop_<timestamp>/
# 预期：包含 /scan /odometry/filtered 等话题，duration ≈ 5s
```

### S-C 全程录包检查（T9）

```bash
# 启动时加 use_recording:=true 验证全程录包
./scripts/launch.sh start --clean sim_bringup \
    use_slam:=true \
    use_nav2:=true \
    use_recording:=true \
    use_rviz:=false

# 运行 1 分钟后 Ctrl+C，检查录包
ls artifacts/bags/session_*/
ros2 bag info artifacts/bags/session_<timestamp>/
# 预期：/map /scan /odometry/filtered /cmd_vel /tf 等话题均在
```

### M_S 通过标准 ✅

| 检查项 | 预期 |
|--------|------|
| 急停触发时间 | ≤ 3 帧（`consecutive_frames=3`，约 0.2s） |
| 急停时 `/cmd_vel` | 全零 Twist |
| 急停 bag 时长 | ≈ 5s |
| 全程录包话题 | 包含 `/map /scan /cmd_vel /tf` |

---

## 阶段 P — 感知集成 (M_P)

> **目标**：感知节点检测到标记物，`perception_adapter` 输出统一格式的 marker 数据，
> 去重逻辑正常工作。

```bash
# 终端 1：启动带相机的仿真
./scripts/launch.sh start --clean sim_bringup \
    use_slam:=true \
    use_nav2:=false \
    use_camera:=true \
    use_rviz:=true
```

### P-A 感知节点输出检查

```bash
# 终端 2：监听感知输出
source install/setup.bash

ros2 topic hz /part3/perception/markers       # 预期 ~2 Hz
ros2 topic hz /part3/perception/greek_markers # 预期 ~2 Hz
ros2 topic echo /part3/perception/marker_event

# 同时检查颜色检测和希腊字母检测的原始输出
ros2 topic echo /part3/perception/colour_markers --once
ros2 topic echo /part3/perception/greek_markers --once
```

### P-B 去重测试

遥控机器人经过同一个标记物 2 次，检查 marker 列表是否只有 1 条记录：

```bash
# 检查 markers.json 去重结果
cat artifacts/waypoints/markers.json
# 同一位置的 marker 应只有 1 条（count 字段应 > 1）
```

### P-C 手动注入 Marker（无物理相机时）

```bash
# 模拟感知输出用于测试后续流程
ros2 topic pub --once /part3/perception/markers geometry_msgs/msg/PoseArray \
  '{header: {frame_id: "map"}, poses: [
    {position: {x: 3.0, y: 1.0, z: 0.0}, orientation: {w: 1.0}},
    {position: {x: -2.0, y: 3.0, z: 0.0}, orientation: {w: 1.0}},
    {position: {x: 1.5, y: -2.5, z: 0.0}, orientation: {w: 1.0}}
  ]}'
```

### M_P 通过标准 ✅

| 检查项 | 预期 |
|--------|------|
| `/part3/perception/markers` 频率 | ~2 Hz |
| 单个 marker 多次检测 | markers.json 只有 1 条记录（count > 1） |
| marker 坐标系 | `map` 帧（非相机/像素帧） |
| marker_event 格式 | 含 type/label/x/y/confidence 字段 |

---

## 阶段 4 — 自主探索 (M4)

> **目标**：机器人自主选 frontier 并覆盖 15×15m 区域，覆盖率 ≥ 95% 后停止并保存地图。

```bash
# 终端 1：启动完整探索栈
./scripts/launch.sh start --clean sim_bringup \
    use_slam:=true \
    use_nav2:=true \
    use_exploration:=true \
    use_safety:=true \
    use_rviz:=true
```

等待约 45s，exploration_node 启动完成。

### 4-A 触发探索

```bash
# 终端 2
source install/setup.bash

# 监控探索进度（另开终端）
ros2 topic echo /part3/mapping/map_status &
ros2 topic echo /part3/system/state &

# 触发探索
ros2 service call /part3/mapping/start std_srvs/srv/Trigger {}
```

**预期序列**：
```
/part3/system/state → "MAPPING"
/part3/mapping/map_status → "coverage=5% frontiers=12 area=15x15"
/part3/mapping/map_status → "coverage=35% frontiers=8 area=15x15"
...
/part3/mapping/map_status → "coverage=done coverage_pct=96.2%"
/part3/system/state → "COMPLETE"
```

### 4-B 探索途中检查

- RViz 中 `/map` 随机器人移动持续生长（灰色未知区域减少）
- 机器人不应卡在某个位置超过 60s（frontier 超时后应切换目标）
- Nav2 路径规划失败时应重新选 frontier，不应崩溃

### 4-C 探索完成后持久化检查

```bash
# 检查地图文件
ls -la artifacts/maps/
# 预期文件：
# discovery_map.pgm  discovery_map.yaml
# discovery_map.posegraph  discovery_map.data

# 检查地图可加载
ros2 run nav2_map_server map_server --ros-args \
  -p yaml_filename:=artifacts/maps/discovery_map.yaml \
  -p use_sim_time:=true &
sleep 3
ros2 topic echo /map --once | head -5  # 应有地图数据
kill %1

# 检查 waypoint markers 已保存
cat artifacts/waypoints/markers.json
```

### M4 通过标准 ✅

| 检查项 | 预期 |
|--------|------|
| 探索触发后状态 | `/part3/system/state` → `MAPPING` |
| 地图覆盖进度 | `map_status` 持续更新，coverage 递增 |
| 最终覆盖率 | ≥ 85%（目标 95%，15×15m 有障碍场景） |
| 探索结束后状态 | `COMPLETE` |
| 地图文件 | `artifacts/maps/` 有 .pgm/.yaml/.posegraph/.data |
| 机器人不卡死 | 整个过程无超过 90s 的停滞 |

**手动干预**（探索卡住时）：
```bash
# 强制切换 frontier（向 exploration_node 发重置信号）
ros2 topic pub --once /part3/exploration/enable std_msgs/msg/Bool '{data: false}'
sleep 1
ros2 topic pub --once /part3/exploration/enable std_msgs/msg/Bool '{data: true}'
```

---

## 阶段 5 — 服务编排集成 (M5)

> **目标**：验证 mapping_service 正确编排探索流程，状态机转换准确，
> 重启后可重新触发。

```bash
# 继续使用阶段 4 的启动命令（或重新启动）
```

### 5-A 状态机完整流程测试

```bash
source install/setup.bash

# 1. 初始状态
ros2 topic echo /part3/system/state --once  # 应为 IDLE

# 2. 触发建图
ros2 service call /part3/mapping/start std_srvs/srv/Trigger {}
ros2 topic echo /part3/system/state --once  # 应为 MAPPING

# 3. 等待完成（或手动模拟完成）
# 探索完成后：
ros2 topic echo /part3/system/state --once  # 应为 COMPLETE

# 4. 验证 response.success 语义（已接受命令，非已完成）
# success=true 应在 service call 立即返回，不等探索结束
```

### 5-B 重复调用测试

```bash
# 探索完成后再次调用 start，验证可重新激活
ros2 service call /part3/mapping/start std_srvs/srv/Trigger {}
# 预期：接受命令，重新进入 MAPPING 状态（不崩溃）
```

### M5 通过标准 ✅

| 检查项 | 预期 |
|--------|------|
| 初始状态 | `IDLE` |
| 触发后状态 | 立即切换为 `MAPPING`（< 1s） |
| service 返回时机 | 立即返回 `success=true`（异步，不阻塞） |
| 完成后状态 | `COMPLETE` |
| 重复触发 | 不崩溃，重新开始探索 |

---

## 阶段 W — 路点快速导航 (M_W)

> **目标**：第二趟在已知地图上访问 3 个 greek_letter waypoint 并返回 home，
> TSP 排序路径最短。
>
> **前提**：阶段 4 已完成（有 `artifacts/maps/` 和 `artifacts/waypoints/markers.json`）

### W-A 加载已知地图启动（localization 模式）

```bash
# 终端 1：用 localization 模式启动（加载探索保存的地图，不重新建图）
ros2 launch auto_nav_part3 sim_bringup.launch.py \
    use_localization:=true \
    use_nav2:=true \
    use_slam:=false \
    use_exploration:=false \
    use_rviz:=true

# 确认地图加载成功
ros2 topic echo /map --once | head -3  # 应有地图数据（非空）
```

### W-B 手动注入测试 Marker（若感知未就绪）

```bash
source install/setup.bash

# 模拟 3 个希腊字母 waypoint
ros2 topic pub --once /part3/perception/markers geometry_msgs/msg/PoseArray \
  '{header: {frame_id: "map"}, poses: [
    {position: {x: 3.0, y: 1.0, z: 0.0}, orientation: {w: 1.0}},
    {position: {x: -2.0, y: 3.0, z: 0.0}, orientation: {w: 1.0}},
    {position: {x: 1.5, y: -2.5, z: 0.0}, orientation: {w: 1.0}}
  ]}'
```

### W-C 触发路点导航

```bash
# 监控路点计划和状态
ros2 topic echo /part3/waypoint/plan &
ros2 topic echo /part3/system/state &

# 触发第二趟
ros2 service call /part3/waypoint/start std_srvs/srv/Trigger {}
```

**预期输出**：
```
/part3/waypoint/plan:
  "home(-3.0,0.0)→A(1.5,-2.5)→B(3.0,1.0)→C(-2.0,3.0)→home(-3.0,0.0)  dist=18.4m"

/part3/system/state: "WAYPOINT_DRIVE"
（机器人按顺序到达 3 个点）
/part3/system/state: "COMPLETE"
```

### W-D TSP 正确性验证

计算 3 个 marker 的最短路径（手动验证或用脚本）：

```bash
python3 - <<'EOF'
from itertools import permutations
import math

home = (-3.0, 0.0)
markers = [(3.0, 1.0), (-2.0, 3.0), (1.5, -2.5)]

def dist(a, b): return math.hypot(a[0]-b[0], a[1]-b[1])
def total(order):
    pts = [home] + list(order) + [home]
    return sum(dist(pts[i], pts[i+1]) for i in range(len(pts)-1))

best = min(permutations(markers), key=total)
print(f"最优顺序: {best}")
print(f"最短距离: {total(best):.2f}m")
for p in permutations(markers):
    print(f"  {p}  → {total(p):.2f}m")
EOF
```

对比 `/part3/waypoint/plan` 中的 dist 是否与手动计算的最短距离一致。

### M_W 通过标准 ✅

| 检查项 | 预期 |
|--------|------|
| 路点计划发布 | 在 `waypoint/start` 后 3s 内发布 |
| TSP 排序 | 与手动计算最短路径一致 |
| 到达所有 waypoint | 3/3 到达（仿真场景） |
| 最终回到 home | 距离 (−3.0, 0.0) < 0.5m |
| 最终状态 | `COMPLETE` |
| 无 marker 时行为 | 返回 `success=False`，不崩溃 |

---

## 全链路集成测试（最终验证）

> **目标**：从零开始，一条 launch 命令跑通完整 Demo 流程，
> 录制全程 bag 作为答辩证据。

### 全链路测试步骤

```bash
# === 终端 1：启动完整栈 ===
./scripts/launch.sh start --clean sim_bringup \
    use_nav2:=true \
    use_exploration:=true \
    use_slam:=true \
    use_rviz:=true \
    use_safety:=true \
    use_camera:=true \
    use_recording:=true   # 全程录包

# 等待以下日志（约 45–50s）：
# [slam_lifecycle] activate OK
# [lifecycle_manager_navigation] Managed nodes are active
# [exploration_node] 就绪，等待激活信号
```

```bash
# === 终端 2：监控（全程挂着） ===
source install/setup.bash
ros2 topic echo /part3/system/state &
ros2 topic echo /part3/mapping/map_status &
ros2 topic echo /part3/waypoint/plan &
ros2 topic echo /part3/safety/estop_event &
wait
```

```bash
# === 终端 3：操作序列 ===
source install/setup.bash

# --- 第一趟：自主建图 ---
echo "=== 触发建图 ==="
ros2 service call /part3/mapping/start std_srvs/srv/Trigger {}

# 等待 map_status 出现 "coverage=done"（约 10–20 分钟）
# 期间可在 RViz 观察地图生长和机器人轨迹

# --- 确认探索完成 ---
echo "=== 检查地图文件 ==="
ls -la artifacts/maps/
cat artifacts/waypoints/markers.json

# --- 第二趟：路点快速导航 ---
echo "=== 触发路点导航 ==="
ros2 service call /part3/waypoint/start std_srvs/srv/Trigger {}

# 等待 system/state 变为 COMPLETE
```

### 全链路通过标准 ✅

| 验证项 | 预期 |
|--------|------|
| 启动到探索激活 | < 50s，无崩溃 |
| 自主探索覆盖率 | ≥ 85% |
| 地图文件保存 | `artifacts/maps/` 有完整 4 个文件 |
| Waypoint 路径规划 | TSP 最短，plan 正确发布 |
| 路点导航全程 | 3 个 waypoint 全部到达 |
| 返回 home | 终点距 home < 0.5m |
| 最终状态 | `COMPLETE` |
| 全程录包 | `artifacts/bags/session_*/` 可正常 info |
| 无节点崩溃 | `ros2 node list` 最终所有节点仍在线 |

```bash
# 最终验证命令
echo "=== 节点存活检查 ==="
ros2 node list

echo "=== 录包验证 ==="
ros2 bag info artifacts/bags/session_*/

echo "=== 地图文件验证 ==="
ls -lh artifacts/maps/

echo "=== Marker 持久化验证 ==="
python3 -c "
import json
with open('artifacts/waypoints/markers.json') as f:
    markers = json.load(f)
print(f'总 marker 数: {len(markers)}')
greek = [m for m in markers if m.get(\"type\") == \"greek\"]
print(f'希腊字母 marker 数: {len(greek)}')
for m in greek:
    print(f'  {m}')
"
```

---

## 物理部署前最终检查清单

> 完成所有仿真阶段测试后，用此清单做最终确认再上真机。

```
仿真验证
□ M0 通过：基础话题 / TF / 遥控闭环
□ M1 通过：EKF 单源发布 odom→base_link，滤波平滑
□ M2 通过：SLAM 建图质量达标，无幽灵墙，闭环误差 < 0.3m
□ M3 通过：Nav2 点对点导航 3/3 成功，障碍绕行正常
□ M_S 通过：急停触发正确，5s 滚动录包正常，全程录包正常
□ M_P 通过：感知 adapter 输出 map 帧坐标，去重正常
□ M4 通过：自主探索覆盖 ≥ 85%，地图文件完整保存
□ M5 通过：状态机转换正确，重复调用不崩溃
□ M_W 通过：TSP 排序正确，3 个 waypoint 全部到达
□ 全链路 通过：端到端 Demo 流程无人干预完成

代码 & 配置
□ use_sim_time: True 在所有节点/配置中已设置（物理机改为 False）
□ URDF diff-drive <tf_topic> 已改为不桥接的内部话题（无双 TF 源）
□ discovery_15x15.sdf 的 9 个障碍有碰撞几何（非仅视觉）
□ nav2_params.yaml obstacle_layer marking: False（防幽灵墙）
□ EKF controller_frequency=30，transform_tolerance=0.3

真机切换提示（M6 部署时参考）
□ 将 use_sim:=false 传入 launch，切换到真机驱动（ARIA + SICK）
□ use_sim_time 改为 False（或删除，使用系统时钟）
□ spawn 位置参数无需修改（真机从 home 出发即可）
□ 检查真机 /scan /odom /imu 话题类型与 sim 完全一致（topic contract）
□ 室内低速测试（胶带缠轮防烧电机，速度 ≤ 0.3 m/s）
```

---

## 快速参考：常用诊断命令

```bash
# TF 树完整性
ros2 run tf2_tools view_frames

# 实时监控某段 TF
ros2 run tf2_ros tf2_echo map odom
ros2 run tf2_ros tf2_echo odom base_link

# 话题频率
ros2 topic hz /scan /odom /imu /map /odometry/filtered

# Nav2 Action Server 状态
ros2 action info /navigate_to_pose
ros2 action info /navigate_through_poses

# slam_toolbox 生命周期状态
ros2 lifecycle get /slam_toolbox

# SLAM 手动激活（lifecycle 未自动成功时）
until ros2 lifecycle set /slam_toolbox configure 2>/dev/null; do sleep 0.5; done
sleep 0.5 && ros2 lifecycle set /slam_toolbox activate

# 节点列表 & 日志
ros2 node list
ros2 node info /slam_toolbox
ros2 node info /exploration_node

# 话题冲突检查（多发布者）
ros2 topic info /cmd_vel -v
ros2 topic info /tf -v
```

---

*本文档依据 `docs/ROBOT_MOTION_DEV_PLAN.md` 各里程碑定义、
`docs/DEBUG_PLAN.md` 已知 bug 根因、以及 `docs/TOPICS.md` 接口契约生成。
真机部署时同步参考 `docs/ROBOT_MOTION_DEV_PLAN.md` M6 章节。*
