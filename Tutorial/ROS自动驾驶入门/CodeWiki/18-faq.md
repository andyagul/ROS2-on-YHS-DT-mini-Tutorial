# 18 · 故障诊断 FAQ

[← 上一章：版本矩阵](17-version-matrix.md) · [返回索引](README.md) · [下一章：架构决策 →](19-adr.md)

> 按"症状 → 快速判断 → 根因 → 处理"组织。命令默认已 `source /opt/ros/humble/setup.bash` 与 `source install/setup.bash`。
> 标有 **Kx** 的条目对应 [10 已知问题](10-known-issues.md)。

---

## A. 构建与启动

### A1. `colcon build` 报找不到 `cartographer_ros_msgs`

**快速判断**

```bash
ros2 pkg prefix cartographer_ros_msgs
```

**根因**：消息包未先编译，或 `install/setup.bash` 未 source。

**处理**

```bash
colcon build --packages-up-to cartographer_ros
source install/setup.bash
```

### A2. 启动时报 `package 'xxx' not found`

**根因**：`package.xml` 缺运行依赖（`yhs_nav2` 本身就没有声明依赖，K10），或对应包没编译。

**处理**

```bash
ros2 pkg list | grep -E "yhs|nvilidar|serial_imu|orbbec"
rosdep install --from-paths src --ignore-src -r -y
colcon build --packages-select yhs_nav2 yhs_can_interfaces yhs_can_control
```

### A3. 串口打不开：`Permission denied`

```bash
ls -l /dev/lidar /dev/imu
sudo usermod -aG dialout $USER   # 重新登录生效
```

若设备名不存在，检查 udev 规则与 USB 连接；`dmesg -T | tail` 查看是否枚举为 `ttyUSB0/1`。

### A4. `ip link set can0 up` 失败

```bash
sudo modprobe can can_raw
ip link show can0
sudo ip link set can0 type can bitrate 500000
sudo ip link set can0 up
```

若报 `Device or resource busy`，先 `sudo ip link set can0 down`；若接口不存在，检查 CAN 适配器驱动与 `dmesg`。

---

## B. 传感器

### B1. `/scan` 没有数据

```bash
ros2 topic list | grep scan
ros2 topic hz /scan --qos-reliability best_effort
ros2 node info /nvilidar_ros2_node
```

**根因**：雷达未上电、串口被占用、波特率不匹配（默认 512000）、QoS 用错。

**处理**：确认 `/dev/lidar` 权限；用 `ros2 topic echo --qos-reliability best_effort`；检查 launch 中的 `serialport_name`。

### B2. `tf2_echo base_link laser_link` 超时

**根因**：雷达 launch 未启动静态 TF，或 `frame_id` 与 TF 不一致。

**处理**

```bash
ros2 run tf2_ros tf2_echo base_link laser_link
ros2 topic echo /tf_static --once
```

确认 `nvilidar.yaml` 的 `frame_id: laser_link` 与 launch 中 `0 0 0.33 0 0 0 1 base_link laser_link` 匹配。

### B3. `/imu_data` 无数据

```bash
ls -l /dev/imu
ros2 topic hz /imu_data
```

**根因**：设备名不是 `/dev/imu`、波特率不是 115200、串口被占用（K11）。

**处理**：当前代码硬编码，需改 [serial_port.cpp](../../../src/hardware/serial_imu/src/serial_port.cpp) 的 `IMU_SERIAL` / `BAUD` 并重新编译；或建立 udev 符号链接到 `/dev/imu`。

### B4. 静止时 yaw 缓慢漂移

**根因**：IMU 零偏未标定；底盘里程计直接使用绝对 yaw。

**处理**：静置标定零偏；确认 IMU 安装水平；必要时在底盘节点加入 yaw 偏移与低通滤波（K12）。

---

## C. 底盘与里程计

### C1. 发 `/cmd_vel` 车不动

按链路逐段排查：

```bash
ros2 topic echo /cmd_vel                 # 1. 指令是否到达
ros2 topic echo /chassis_info_fb         # 2. CAN 是否有反馈
candump can0 | grep 98C4D1EF             # 3. 反馈帧
```

| 断点 | 可能原因 |
| --- | --- |
| 无 `/cmd_vel` | Nav2 未 active、控制器未输出、topic 命名空间不一致 |
| 有 `/cmd_vel` 无反馈 | CAN 未 up、底盘未上电、急停按下、使能未开 |
| 有反馈但不动 | 挡位/模式不对、轮速换算、电机使能 |

> `yhs_can_control` 将 `geometry_msgs/Twist` 线性速度按轮径换算为 rpm，角速度换算为左右轮差速；`cmd_callback` 固定写挡位 3（K13）。若现场需要倒挡/空挡，当前代码不支持。

### C2. 车走反方向或原地打转

**根因**：左右轮 ID/符号接反，或轮径参数错误。

**处理**：架空车轮，单独发 `linear.x=0.1` 与 `angular.z=0.2` 观察方向；核对 `yhs_can_control_node.cpp` 中轮径与减速比常量。不要通过改 `cmd_vel` 符号"临时修"。

### C3. `/odom` 跳变、RViz 里车瞬移

**快速判断**

```bash
ros2 run tf2_tools view_frames
```

**根因**

1. `odom→base_link` 有两个发布者（Cartographer 与底盘同时 `use_tf=true`）。
2. IMU yaw 在 ±π 处回绕，里程计角度直接赋值导致跳变（K12）。

**处理**：建图时底盘 `use_tf:=false`；导航时 Cartographer 不启动；对 yaw 做连续化处理（unwrap）。

### C4. `/odom` 频率正常但 `chassis_info_fb` 没有

**根因**：`chassis_info_fb` 只在收到 `0x98C4D1EF` 后发布；反馈帧未配置或底盘固件不发送。

**处理**：`candump can0,98C4D1EF:0FFFFFFF` 确认帧存在；检查底盘配置。

---

## D. 建图

### D1. 建图时没有 `/map`

**快速判断**

```bash
ros2 node list | grep cartographer
ros2 topic hz /map
ros2 topic echo /clock --once
```

**根因**：`cartographer_occupancy_grid_node` 使用 `use_sim_time=true`，但没有 `/clock` 发布者（K2），栅格永不发布。

**处理**：把该节点的 `use_sim_time` 改为 `false`，或统一全系统使用仿真时钟。

### D2. 地图重影、墙体分叉

| 可能原因 | 检查 |
| --- | --- |
| 雷达外参错误 | `tf2_echo base_link laser_link`，确认 0.33 m 与实际一致 |
| 时间戳不同步 | `ros2 topic delay /scan`，检查 IMU 与雷达时间基 |
| 行驶过快 | 降速至 0.3 m/s 以下重测 |
| IMU 未使用 | `dgt_mini.lua` 中 `use_imu_data=false`，快速转向时匹配退化 |
| 回环参数过严 | `optimize_every_n_nodes`、`constraint_builder.min_score` |

### D3. `write_state` 保存 pbstream 失败

```bash
ros2 service list | grep write_state
ros2 service call /write_state cartographer_ros_msgs/srv/WriteState \
  "{filename: '/workspace/maps/a.pbstream', include_unfinished_submaps: true}"
```

**根因**：目录不存在/无写权限、文件名用了相对路径、Cartographer 未运行。

### D4. pbstream 转地图后尺寸不对

**根因**：`resolution` 或 `origin` 与建图时不一致。

**处理**：转换时显式指定分辨率，并在 YAML 中核对 `origin`；转换后先在 RViz 对照轨迹检查。

---

## E. 导航

### E1. `map_server` 启动失败

```bash
ros2 launch nav2_map_server map_server.launch.py yaml_filename:=/path/a.yaml
```

**根因**：YAML 的 `image` 指向不存在的文件（K1，`zgq-g.yaml → zgq.pgm`）。

**处理**：`ls` 校验 PGM 与 YAML 同目录同名；修正 `image` 后重启。

### E2. AMCL 粒子不收敛 / 定位飘

| 检查 | 命令/说明 |
| --- | --- |
| 初始位姿 | 默认 `(0,0,0)`，车不在原点需重设 |
| 地图质量 | 墙体是否闭合、是否有重影 |
| 激光与地图匹配 | RViz 叠加 `/scan` 与 `/map` |
| 粒子数 | 默认 500~2000，场地大时适当增加 |
| 里程计 | `/odom` 是否连续、方向正确 |

```bash
ros2 run yhs_nav2 set_init_amcl_pose.py --ros-args -p x:=1.0 -p y:=2.0 -p theta:=0.0
```

### E3. `NavigateToPose` 返回 `NO_VALID_PATH`

**根因**：起点/终点在障碍或未知区域、代价地图未收到激光、膨胀半径过大、全局规划器未加载。

**处理**

```bash
ros2 topic echo /global_costmap/costmap --once
ros2 param get /planner_server expected_planner_frequency
ros2 action send_goal /compute_path_to_pose nav2_msgs/action/ComputePathToPose \
  "{goal: {pose: {header: {frame_id: map}, pose: {position: {x: 1.0, y: 0.0}}}}, use_start: false}"
```

检查 `global_costmap` 的 `robot_radius/footprint`、`inflation_radius`、`observation_sources`。

### E4. 有全局路径但车不动

```bash
ros2 topic echo /cmd_vel_nav
ros2 topic echo /cmd_vel_smoothed
ros2 topic echo /cmd_vel
ros2 lifecycle get /controller_server
```

| 断点 | 原因 |
| --- | --- |
| 无 `/cmd_vel_nav` | 控制器未 active、TEB 无可行轨迹、路径为空 |
| 有 `_nav` 无 `_smoothed` | `velocity_smoother` 未启动/未 active |
| 有 `/cmd_vel` 车不动 | 底盘链路（见 C1） |

TEB 无可行轨迹的常见原因：`footprint_model` 与实际车体不符（K7）、`min_obstacle_dist` 过大、狭窄通道宽度小于膨胀后车宽。

### E5. 贴墙、擦边、过门困难

- 局部代价地图**没有膨胀层**（K3）：`plugins` 只有 `static_layer`、`obstacle_layer`。
- TEB `footprint_model` 半径为 0.1 m，严重低估车体（K7）。

**处理**：把 `inflation_layer` 加入 `local_costmap.plugins`，并把 TEB 改为 polygon 足迹。修改后必须确认窄通道仍可通过。

### E6. 卡住后不执行恢复行为

**根因**：YAML 使用旧键 `recoveries_server` / `recovery_plugins`，Nav2 1.1.5 读取 `behavior_server` / `behavior_plugins`（K4）。

**处理**：迁移键名，确认 `/spin`、`/backup`、`/wait` 动作存在；在 BT 的 recovery 分支中启用。

### E7. RViz 中地图/激光不显示

| 现象 | 原因 |
| --- | --- |
| 地图空白 | QoS：Map 需 `Transient Local`；或 `map_server` 未加载 |
| 激光不显示 | `/scan` 为 Best Effort，RViz QoS 需匹配 |
| 整体错位 | Fixed Frame 不是 `map`，或 TF 缺失 |
| 车模型不显示 | 未启动 `robot_state_publisher`，或 URDF 无关节（K14） |

### E8. 示教路径录制后无法导航

**检查**

```bash
ros2 topic echo /record_path --once          # 路径内容、frame_id
ros2 action list | grep navigate_follow
ros2 param get /bt_navigator default_nav_to_pose_bt_xml
```

**根因**

1. 默认 BT 是标准 `NavigateToPose`，**不会调用** `NavigateFollowRecordPath`（自定义节点已注册但未接入默认树）。
2. `path.header.frame_id` 不是 `map`，控制器无法变换。
3. 路径为空或点间距过大。

**处理**：使用自定义 BT XML，或在外部节点先 `RecordPath` 再以 `FollowPath` 动作下发；确保路径在 `map` 系且非空。详见 [14 §8](14-extension-guide.md)。

---

## F. 性能与稳定性

### F1. CPU 占用高、控制延迟

| 措施 | 说明 |
| --- | --- |
| 降低 Cartographer 发布频率 | K19：`submap_publish_period_sec`、`optimize_every_n_nodes` |
| 降低代价地图分辨率 | 0.05 → 0.1（会降低精度） |
| 减少观测源 | 关闭未使用的 `/scan1` |
| 关闭不必要的 RViz | 远程可视化最耗 CPU |
| 使用 Release 构建 | `-DCMAKE_BUILD_TYPE=Release` |
| 检查 DDS | 大消息场景切换 CycloneDDS |

### F2. launch 启动后出现两个 `nav2_container`

**根因**：`use_composition=True` 只启动一个空容器，而 `localization_launch.py` / `navigation_launch.py` 内部始终用独立 `Node`（K8）。

**处理**：当前版本把 `use_composition` 设为 `False`，避免空容器；或改造子 launch 真正使用 `ComposableNodeContainer`。

### F3. 同一网络多台车互相干扰

**根因**：TF 坐标系未加前缀，`ROS_DOMAIN_ID` 相同。

**处理**：短期为每台车设置不同 `ROS_DOMAIN_ID`；长期做 frame 前缀化（`tb1/map`、`tb1/base_link`）。

---

## G. 快速诊断脚本

```bash
#!/usr/bin/env bash
set +e
echo "== ROS =="; printenv ROS_DISTRO
echo "== packages =="; ros2 pkg list | wc -l
echo "== nodes =="; ros2 node list
echo "== topics =="; ros2 topic list | sort
echo "== scan hz =="; timeout 5 ros2 topic hz /scan --qos-reliability best_effort
echo "== odom hz =="; timeout 5 ros2 topic hz /odom
echo "== tf =="; timeout 3 ros2 run tf2_ros tf2_echo map base_link
echo "== lifecycle =="
for n in map_server amcl planner_server controller_server bt_navigator behavior_server; do
  printf "%-20s " "$n"; ros2 lifecycle get /$n 2>/dev/null
done
echo "== can =="; ip -details link show can0 2>/dev/null | head -5
echo "== devices =="; ls -l /dev/lidar /dev/imu 2>&1
```
