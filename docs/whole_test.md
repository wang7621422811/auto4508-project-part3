完整链路测试（CLI）
准备：source 环境（每个终端都要执行）

cd /home/parallels/workspace/projects/auto4508-project-part3
source install/setup.bash
第一步：启动仿真全栈（终端 1）

ros2 launch auto_nav_part3 sim_bringup.launch.py \
    use_nav2:=true \
    use_exploration:=true \
    use_camera:=true \
    use_safety:=true \
    use_rviz:=true
等待时序（系统自动完成）：

时间	事件	验证方法
t=2s	bridge 就绪	/scan /odom /imu 有数据
t=10.5s	SLAM configure+activate	日志出现 [slam_lifecycle] activate OK
t=15s	Nav2 启动	日志出现 lifecycle_manager
t=45s	exploration_node 启动（等待激活信号）	日志出现 exploration_node 就绪
第二步：开监控（终端 2，一直开着）

# 在同一终端用 & 同时监控多个话题
source install/setup.bash

ros2 topic echo /part3/system/state &
ros2 topic echo /part3/mapping/map_status &
ros2 topic echo /part3/waypoint/plan &
ros2 topic echo /part3/perception/marker_event &
wait
或者分开四个终端各自 echo 一个。

第三步：验证各子系统就绪（终端 3）

source install/setup.bash

# SLAM 是否在发布地图
ros2 topic hz /map                        # 应有频率（~1Hz）

# Nav2 action server 是否可用
ros2 action info /navigate_to_pose        # 应显示 server 和 0 clients

# perception_adapter 是否在发布
ros2 topic hz /part3/perception/markers   # 应有 2Hz
第四步：触发第一趟——自主建图（终端 3）

ros2 service call /part3/mapping/start std_srvs/srv/Trigger {}
预期输出：

/part3/system/state → MAPPING
/part3/mapping/map_status → coverage=X% frontiers=N area=15x15 持续更新
RViz 里地图随机器人移动持续生长
相机看到希腊字母时 /part3/perception/marker_event 有输出
等待探索完成（直到出现）：


map_status: coverage=done coverage_pct=9X.X%
第五步：探索完成后检查持久化

# 查看保存的 waypoint 文件
cat artifacts/waypoints/markers.json
预期内容（示例）：


[
  {"type": "greek", "label": "alpha", "x": 2.43, "y": -1.50, "confidence": 0.94, "count": 7},
  {"type": "greek", "label": "beta",  "x": -1.88, "y": 3.21, "confidence": 0.91, "count": 4},
  {"type": "greek", "label": "delta", "x": 1.10, "y": 2.05, "confidence": 0.88, "count": 3}
]
如果文件存在且 type=greek 的条目 ≥ 1 个，则可继续。

第六步：触发第二趟——路点导航（终端 3）

ros2 service call /part3/waypoint/start std_srvs/srv/Trigger {}
预期输出：

/part3/waypoint/plan → home(-3.0,0.0)→A(2.43,-1.50)→B(1.10,2.05)→C(-1.88,3.21)→home(-3.0,0.0) dist=XX.Xm
/part3/system/state → WAYPOINT_DRIVE
RViz 里机器人开始按顺序前往各路点
完成后 /part3/system/state → COMPLETE
常见问题排查

# 问题：/part3/waypoint/start 返回 No greek_letter markers
# 检查 markers.json 是否存在
ls -la artifacts/waypoints/

# 检查 perception_adapter 是否在发布希腊字母 marker
ros2 topic echo /part3/perception/greek_markers   # 应有 poses

# 手动确认 perception_adapter 已从文件恢复
ros2 service call /part3/perception/get_markers std_srvs/srv/Trigger {}
# 返回 message 里有 markers=N（N > 0 则正常）

# 问题：WAYPOINT_FAILED
# 检查 Nav2 action server 状态
ros2 action info /navigate_through_poses   # 应显示 server

# 问题：exploration 不触发
ros2 topic pub --once /part3/exploration/enable std_msgs/msg/Bool '{data: true}'
关键：两趟之间重启节点时
如果你在两趟之间重启系统（不清空 artifacts/waypoints/markers.json），第二趟启动后 perception_adapter 会自动从文件恢复上次的 marker，waypoint_service 在 3s 内就能拿到路点，不需要重新探索。


====


全部完成，colcon build 后按下面流程用：

Phase 1 — 建图（一次）


./scripts/launch.sh start --clean sim_bringup use_nav2:=true \
    use_exploration:=true  use_slam:=true use_rviz:=true use_safety:=true use_camera:=true

./scripts/launch.sh start --clean sim_bringup use_nav2:=false \
    use_exploration:=false  use_slam:=false use_rviz:=false use_safety:=false \
        use_camera:=true

# 启动探索
ros2 service call /part3/mapping/start std_srvs/srv/Trigger {}
探索完成后，map_manager 自动保存三个文件：


artifacts/maps/discovery_map.pgm
artifacts/maps/discovery_map.yaml
artifacts/maps/discovery_map.posegraph   ← 新增，供 Phase 2 加载
artifacts/maps/discovery_map.data        ← 新增
Phase 2 — 重复导航（每次测试）


ros2 launch auto_nav_part3 sim_bringup.launch.py \
    use_localization:=true use_nav2:=true \
    use_slam:=false use_exploration:=false

# 直接调用路点服务（地图已加载）
ros2 service call /part3/waypoint/start std_srvs/srv/Trigger {}
slam_toolbox 以 localization 模式启动，加载位姿图，发布相同的 /map（不更新）和 map→odom TF，Nav2 完全正常工作，可以反复 use_localization:=true 启动测试。