# 位置/角度对不上问题——从零开始排查计划

> 现象：RViz 中机器人位置和角度与 Gazebo 实际不符，手动驾驶时地图出现多条幽灵墙。
>
> 排查原则：**逐层开启，发现哪层出问题就在那层修**，不要同时开启所有功能。

---

## 快速定位图

```
Gazebo 物理世界
    │
    ▼ 轮子关节转速 → DiffDrive 插件
    /odom  (原始轮式里程计)
    │
    ▼ EKF 融合 /odom + /imu
    /odometry/filtered  →  TF: odom → base_link
    │
    ▼ SLAM 读取 /scan + odom→base_link
    /map  →  TF: map → odom
    │
    ▼ RViz 渲染：map → odom → base_link
    机器人在 RViz 里的显示位置
```

每一层都可能出错，按顺序从底向上排查。

---

## 阶段 0：确认编译和环境

```bash
cd ~/workspace/projects/auto4508-project-part3
colcon build --symlink-install
source install/setup.bash
```

预期：编译无报错，无 warning 也最好检查一遍。

---

## 阶段 1：只启动 Gazebo + 机器人模型（屏蔽 EKF、SLAM、Nav2）

### 目标
验证 Gazebo 物理仿真和机器人模型本身是否正常，URDF 是否正确加载。

### 启动命令
```bash
ros2 launch auto_nav_part3 sim_bringup.launch.py \
  use_slam:=false \
  use_nav2:=false \
  use_exploration:=false \
  use_rviz:=false

  ./scripts/launch.sh start sim_bringup use_nav2:=false use_exploration:=false  use_slam:=false use_rviz:=false

```

### 检查点 1-A：机器人是否在正确位置生成

默认 spawn 位置：`x=-3.0, y=0, z=0.18`

```bash
# 查看 Gazebo 里机器人的世界坐标
ros2 topic echo /odom --once
```

预期：`pose.pose.position.x ≈ -3.0, y ≈ 0.0`（允许 ±0.05m）

❌ 如果 x/y 与 -3/0 差很多 → **spawn 位置参数有问题**，检查 sim_bringup 里的 x/y/z 参数

### 检查点 1-B：TF 静态帧是否齐全

```bash
ros2 run tf2_tools view_frames
# 生成 frames.pdf，查看：
# base_link → chassis → laser_frame
# base_link → imu_link
# 以上必须都在
```

预期：所有静态 TF 连接，没有孤立的帧。

❌ 如果 laser_frame 没有 → URDF 或 robot_state_publisher 未正常启动

---

## 阶段 2：检查原始轮式里程计 /odom（EKF 之前）

### 目标
确认 Gazebo DiffDrive 插件输出的 /odom 是否准确反映机器人真实运动。
这一步要**手动驾驶**并比较。

### 检查方式

开启 teleop：
```bash
# 新终端
ros2 launch auto_nav_part3 teleop.launch.py
```

实验一：**直线行驶 1 米**
```bash
# 监控位置
ros2 topic echo /odom --field pose.pose.position
```
- 向前开 ~1m
- 预期：x 变化约 1.0m，y 基本不变（< 0.05m）

❌ 如果 y 漂移很大 → 轮子对称性有问题，检查左右轮 wheel_separation

实验二：**原地旋转 360°（顺时针）**
```bash
ros2 topic echo /odom --field pose.pose.orientation
# 用 ros2 run tf_transformations euler_from_quaternion 辅助查看 yaw
```
或者直接看 yaw：
```bash
ros2 run tf2_ros tf2_echo odom base_link
```
- 原地旋转一圈
- 预期：结束时 yaw 回到接近初始值（误差 < 10°）

❌ 如果旋转 360° 后 yaw 偏差 > 20° → **wheel_separation 不准**，Pioneer 4 轮滑动转向的
   有效轮距和物理轮距不同，可能需要把 0.394 调大到 0.42-0.46 m

### 检查点 2 结论

| 测试 | 通过 | 说明 |
|------|------|------|
| 直线 1m，y 漂移 < 5cm | ✅/❌ | |
| 原地 360°，yaw 误差 < 10° | ✅/❌ | |

---

## 阶段 3：加入 EKF，检查 odom → base_link

### 目标
EKF 在 /odom 基础上融合 /imu，输出 /odometry/filtered 和 odom→base_link TF。
这是 RViz 里机器人位置的直接来源（固定帧 = odom 时）。

### 启动命令
与阶段 1 相同（EKF 已在 sim_bringup 里默认开启，use_slam:=false）。

### 关键诊断：换固定帧

在 RViz 里：`Global Options → Fixed Frame 改为 odom`

然后手动驾驶，**看 RViz 里机器人和 Gazebo 里的机器人位置是否一致**。

- ✅ 一致 → EKF 正常，问题不在这里，跳去阶段 4
- ❌ 不一致 → EKF 引入了误差，继续下面排查

### 检查点 3-A：EKF 输出和原始 odom 对比

```bash
# 终端 1
ros2 topic echo /odom --field pose.pose.position --once

# 终端 2
ros2 topic echo /odometry/filtered --field pose.pose.position --once
```

两者在静止时应基本一致（误差 < 0.01m）。

❌ 差异很大 → EKF 的 `odom0_differential` 或 `odom0_relative` 配置有问题

### 检查点 3-B：EKF 的 IMU 融合是否引入干扰

在 `ekf.yaml` 里临时关掉 IMU，只用轮式里程计：

```yaml
# 临时测试：注释掉 imu0 相关配置
# imu0: /imu
# imu0_config: [...]
```

重新测试阶段 2 的两个实验，看位置/角度是否更准。

- ✅ 关掉 IMU 后更准 → IMU 融合配置有问题（目前只用了 ax/ay，可能引入噪声）
- ❌ 效果一样 → IMU 不是原因

---

## 阶段 4：加入 SLAM，检查 map → odom

### 目标
SLAM 读取激光雷达 + TF，输出 map→odom 变换。这是 RViz 固定帧为 map 时显示位置的关键。

### 启动命令（开启 SLAM，关闭 Nav2）
```bash
ros2 launch auto_nav_part3 sim_bringup.launch.py \
  use_slam:=true \
  use_nav2:=false \
  use_exploration:=false
```

### 检查点 4-A：SLAM 启动后 map→odom 是否稳定

```bash
# 监控 map→odom 变换（静止时应几乎不变）
ros2 run tf2_ros tf2_echo map odom
```

预期（机器人静止时）：
- translation 数值缓慢变化，变化幅度 < 0.02m/s
- rotation/yaw 缓慢变化，幅度 < 0.01 rad/s

❌ 如果 translation 每秒跳变 > 0.1m → **SLAM ICP 在当前环境下匹配失败**

原因：空矩形房间特征点极少，ICP 无法可靠匹配，退回 odom 估计时位姿跳变。

### 检查点 4-B：手动驾驶建图质量检测

在 RViz 里（固定帧 = map），用 teleop 沿房间边界**慢速贴墙直线行驶一圈（不转弯）**。

预期 /map 效果：

```
正常：四面墙各是一条干净的细线
异常：同一面墙出现 2-3 条平行线（漂移导致）
```

- ✅ 墙线干净 → SLAM 在直线时正常
- ❌ 直线行驶也出现多条墙 → 问题在 TF 链，回去检查阶段 3

### 检查点 4-C：旋转时的 SLAM 表现

原地慢速旋转（角速度 ≤ 0.5 rad/s），观察 /map 里的墙：

- ✅ 旋转后墙线仍然干净 → SLAM 旋转匹配正常
- ❌ 旋转后出现幽灵墙（从中心向外散射的多条线） → **ICP 旋转匹配失败**

这是当前问题的核心。空矩形房间 + 旋转 = ICP 极难匹配。

**临时解法**：
1. 减慢手动旋转速度（< 0.3 rad/s）
2. 在世界里添加 2-3 个不对称的特征物（箱子/柱子）

---

## 阶段 5：加入 Nav2 costmap，检查幽灵墙来源

仅在阶段 4 通过（SLAM 地图干净）后执行。

### 启动命令
```bash
ros2 launch auto_nav_part3 sim_bringup.launch.py \
  use_slam:=true \
  use_nav2:=true \
  use_exploration:=false
```

### 检查点 5-A：costmap 是否叠加幽灵墙

在 RViz 里分别开关以下图层，观察幽灵墙从哪层来：

| 操作 | 观察 |
|------|------|
| 只开 `/map`（static layer） | 看墙是否干净 |
| 开 `global_costmap`（obstacle layer） | 是否在墙旁边多出一层幽灵 |

- ❌ 开了 obstacle layer 后出现多条墙 → 参见下方修复方案

### 修复：global_costmap 的 obstacle_layer 改为只清除不标记

在 `config/nav2_params.yaml` 第 237-250 行，把：
```yaml
marking: True
```
改为：
```yaml
marking: False   # SLAM 已融合激光到 /map，obstacle_layer 再 marking 会因 TF 漂移产生幽灵墙
```

---

## 阶段 6：整体验证

全部阶段通过后，完整启动并手动驾驶一圈验证：

```bash
ros2 launch auto_nav_part3 sim_bringup.launch.py \
  use_slam:=true \
  use_nav2:=true \
  use_exploration:=false
```

验收标准：

| 检查项 | 预期结果 |
|--------|----------|
| RViz 固定帧 `odom`，机器人位置 | 与 Gazebo 位置误差 < 0.1m |
| RViz 固定帧 `map`，机器人位置 | 与 Gazebo 位置误差 < 0.2m |
| 直线行驶后 /map 里的墙 | 单条干净线，无平行幽灵 |
| 慢速旋转后 /map 里的墙 | 无从中心散射的多条线 |
| global_costmap | 只在实际墙位置有障碍标记 |

---

## 已知根因汇总

| 层级 | 问题 | 根因 | 修复方向 |
|------|------|------|----------|
| /odom 角度 | 旋转误差大 | Pioneer 4 轮滑动转向，有效轮距 > 物理轮距 0.394m | 将 wheel_separation 调大到 0.42-0.46m 后测试 |
| map→odom | 位置跳变 | 空矩形房间 ICP 特征不足，旋转时匹配失败 | 加内部特征物；降低旋转速度 |
| global_costmap | 幽灵墙叠加 | obstacle_layer marking=True 与 static_layer 因 TF 漂移错位 | obstacle_layer 改 marking=False |

---

## 常用调试命令速查

```bash
# 查看完整 TF 树
ros2 run tf2_tools view_frames

# 实时监控某段 TF
ros2 run tf2_ros tf2_echo map odom
ros2 run tf2_ros tf2_echo odom base_link

# 查看原始里程计
ros2 topic echo /odom --field pose.pose

# 查看 EKF 融合结果
ros2 topic echo /odometry/filtered --field pose.pose

# 查看激光扫描是否正常
ros2 topic hz /scan

# 查看 SLAM 健康状态
ros2 topic hz /map
ros2 node info /slam_toolbox

# 查看 EKF 诊断
ros2 topic echo /diagnostics
```
