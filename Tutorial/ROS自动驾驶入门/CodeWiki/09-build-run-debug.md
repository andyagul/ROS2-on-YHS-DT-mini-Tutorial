# 09 · 编译、运行与调试

[← 上一章：机器人模型](08-robot-model.md) · [返回索引](README.md) · [下一章：已知问题 →](10-known-issues.md)

---

## 1. 环境要求

| 项 | 要求 |
| --- | --- |
| OS | Ubuntu 22.04 |
| ROS 2 | Humble Hawksbill |
| 编译器 | GCC 11 / CMake ≥ 3.22 |
| 硬件 | x86_64 或 ARM64（Jetson 需注意 Ceres/Protobuf 编译内存） |
| 磁盘 | ≥ 20 GB（Nav2 源码 + 编译产物较大） |
| 内存 | 建议 ≥ 8 GB；4 GB 机器需 `--parallel-workers 2` |

### 1.1 系统依赖

```bash
sudo apt update
sudo apt install -y \
  python3-colcon-common-extensions python3-rosdep python3-pip \
  build-essential cmake git wget curl \
  libeigen3-dev libboost-all-dev libyaml-cpp-dev \
  libceres-dev libprotobuf-dev protobuf-compiler liblua5.3-dev \
  libg2o-dev libpcl-dev liburdfdom-tools \
  libabsl-dev libsqlite3-dev libtinyxml2-dev

sudo apt install -y \
  ros-humble-navigation2 ros-humble-nav2-bringup \
  ros-humble-robot-state-publisher ros-humble-joint-state-publisher \
  ros-humble-xacro ros-humble-tf2-tools ros-humble-teleop-twist-keyboard \
  ros-humble-rosbag2 ros-humble-rviz2

python3 -m pip install transforms3d
```

> Cartographer 官方脚本位于 `src/cartographer/scripts/`：
> `install_abseil.sh`、`install_ceres.sh`、`install_proto3.sh`、`install_async_grpc.sh`、`install_cartographer_cmake.sh`。
> 若 `libabsl-dev` 版本不匹配，优先使用脚本安装固定版本。

### 1.2 初始化 rosdep（首次）

```bash
sudo rosdep init
rosdep update
```

---

## 2. 编译

```bash
cd ~/ros2_ws                # 工作空间根目录（本仓库根）
source /opt/ros/humble/setup.bash

rosdep install --from-paths src --ignore-src -r -y

# 全量编译
colcon build --symlink-install \
  --cmake-args -DCMAKE_BUILD_TYPE=Release

# 内存有限时
MAKEFLAGS="-j2" colcon build --symlink-install --parallel-workers 2

source install/setup.bash
```

### 2.1 按包编译（推荐排错）

```bash
# 底盘与接口
colcon build --packages-select yhs_can_interfaces yhs_can_control

# SLAM
colcon build --packages-select cartographer cartographer_ros_msgs cartographer_ros

# Nav2（先装依赖，后编译）
colcon build --packages-select nav2_msgs nav2_util nav2_core
colcon build --packages-up-to nav2_bringup

# 硬件
colcon build --packages-select nvilidar_ros2 serial_imu

# 编排/模型
colcon build --packages-select yhs_nav2 yhs_chassis_description
```

### 2.2 首次编译常见失败

| 错误 | 原因 | 处理 |
| --- | --- | --- |
| `Could not find package 'cartographer'` | 未编译核心库 | 先编 `cartographer` |
| `absl`/`ceres` 版本冲突 | 系统库与源码不匹配 | 用 `scripts/install_*.sh` |
| `nav2_behavior_tree` 找不到 `yhs_can_interfaces` | 自研动作依赖 | 先编 `yhs_can_interfaces` |
| `virtual memory exhausted` | 并行编译内存不足 | `--parallel-workers 2`、`-j2` |
| Python launch 报 `No module named 'launch'` | 未 source ROS | `source /opt/ros/humble/setup.bash` |
| `nvilidar_ros2` SDK 链接失败 | ARM 架构差异 | 检查 `sdk/src/impl` 的串口/socket 实现 |

---

## 3. 上电检查

```bash
# CAN
sudo modprobe can
sudo ip link set can0 up type can bitrate 500000
ip -details link show can0
candump can0                 # 应能看到 0x98C4xxxx 反馈帧

# 串口
ls -l /dev/lidar /dev/imu
sudo chmod 666 /dev/lidar /dev/imu     # 或配置 udev + dialout
```

---

## 4. 三条启动路线

### 4.1 Cartographer 建图

```bash
ros2 launch yhs_nav2 cartographer.launch.py
```

内部启动：

| 节点 | 关键参数 |
| --- | --- |
| `serial_imu` | `/dev/imu` |
| `nvilidar_ros2` | `/dev/lidar`，`laser_link` |
| `yhs_can_control` | **`use_tf=false`** |
| `cartographer_node` | `dgt_mini.lua`，remap `imu→imu_data` |
| `cartographer_occupancy_grid_node` | `resolution=0.05`，`use_sim_time=True` ⚠ |

预期话题：`/scan`、`/imu_data`、`/odom`、`/submap_list`、`/map`（可能不发布，见问题 #6）。
**不自动启动 RViz**，需要另开：

```bash
rviz2 -d $(ros2 pkg prefix yhs_chassis_description)/share/yhs_chassis_description/rviz/rviz.rviz
```

建图后保存：

```bash
ros2 run nav2_map_server map_saver_cli -f ~/maps/warehouse
```

### 4.2 GMapping 建图

```bash
ros2 launch yhs_nav2 gmapping.launch.py
```

内部启动：IMU、雷达、底盘（**`use_tf=true`**）、`slam_gmapping`。

预期：`/map`、`map→odom` TF。保存同上。

### 4.3 导航（AMCL + Nav2）

```bash
ros2 launch yhs_nav2 yhs_nav2.launch.py map:=$HOME/maps/warehouse.yaml
```

内部启动：

1. `nav2_container`（空容器，见问题 #3）
2. `serial_imu`、`nvilidar_ros2`
3. `yhs_can_control`（`use_tf=true`）
4. `localization_launch.py`：`map_server` + `amcl` + 生命周期管理器
5. `navigation_launch.py`：7 个服务器 + 生命周期管理器

设置初始位姿（二选一）：

```bash
# 方式一：脚本
ros2 run yhs_nav2 set_init_amcl_pose.py --ros-args -p x:=0.0 -p y:=0.0 -p theta:=0.0

# 方式二：RViz 工具栏 "2D Pose Estimate"
```

发送导航目标：

```bash
ros2 action send_goal /navigate_to_pose nav2_msgs/action/NavigateToPose \
  "{pose: {header: {frame_id: map}, pose: {position: {x: 2.0, y: 0.0, z: 0.0}, orientation: {w: 1.0}}}}" \
  --feedback
```

---

## 5. 启动后的自检清单

```bash
# 1. 节点是否齐全
ros2 node list
# 期望：serial_imu_node, nvilidar_ros2_node, yhs_can_control_node,
#       map_server, amcl, controller_server, planner_server,
#       behavior_server, bt_navigator, waypoint_follower,
#       velocity_smoother, lifecycle_manager_*

# 2. 生命周期
ros2 lifecycle get /amcl
ros2 lifecycle get /bt_navigator
ros2 lifecycle get /controller_server

# 3. TF
ros2 run tf2_ros tf2_echo map base_link
ros2 run tf2_ros tf2_echo odom base_link
ros2 run tf2_tools view_frames

# 4. 数据频率
ros2 topic hz /scan --qos-reliability best_effort
ros2 topic hz /odom
ros2 topic hz /amcl_pose

# 5. 控制链路
ros2 topic echo /cmd_vel_nav --once
ros2 topic echo /cmd_vel --once

# 6. 底盘反馈
ros2 topic echo /chassis_info_fb --once
candump can0
```

---

## 6. 手动单点验证

```bash
# 只启动底盘
ros2 launch yhs_can_control yhs_can_control.launch.py use_tf:=true
ros2 topic pub -r 10 /cmd_vel geometry_msgs/Twist \
  "{linear: {x: 0.1}, angular: {z: 0.0}}"
# 观察 /odom 与 candump，Ctrl+C 后应自动停车（零速下发依赖上层，需确认）

# 只启动雷达
ros2 launch nvilidar_ros2 nvilidar.launch.py
ros2 topic echo /scan --once --qos-reliability best_effort

# 只启动 IMU
ros2 launch serial_imu imu_spec_msg.launch.py
ros2 topic echo /imu_data --once
```

> ⚠ `yhs_tutorials/vel_pub` 会持续发布 `/cmd_vel`，**不要**在实车调试时误启动。

---

## 7. 故障速查表

| 症状 | 可能原因 | 处理 |
| --- | --- | --- |
| 编译不过，`ceres` 报错 | Ceres/Eigen 版本 | 用 `cartographer/scripts/install_ceres.sh` |
| `can0` 起不来 | 未 `modprobe can` / 无权限 | 见 §3；检查 USB-CAN 驱动 |
| `/scan` 无数据 | `/dev/lidar` 不存在 / 波特率错 | udev、`ls -l /dev/lidar`、雷达供电 |
| `/imu_data` 无数据 | `/dev/imu` 不存在 / 线序 | udev、确认 `B115200` |
| 建图无 `/map` | `occupancy_grid_node use_sim_time=true` | 改为 `False`（问题 #6） |
| 建图墙体倾斜 | `laser_link` 高度/角度错 | 改静态 TF 0.33 |
| 地图加载失败 | YAML 指向不存在的 PGM | 问题 #1（`zgq-g.yaml`） |
| RViz 显示"无 map" | `map_server` 未激活 | `ros2 lifecycle get /map_server` |
| 车不动 | 生命周期未激活 / `cmd_vel` 无 | §5 逐项检查 |
| 车抖动 | TEB 频率/速度限制过激 | 降 `max_vel_x`、检查 `/odom` 噪声 |
| 定位跳变 | AMCL 初始位姿错 / 雷达被遮挡 | 重设 `/initialpose` |
| 局部避障差 | local_costmap 缺膨胀层 | 问题 #7 |
| `scan1` 持续告警 | 相机未启动 | 启用相机或从 costmap 移除 `scan1` |
| 导航到点不停 | `xy_goal_tolerance=0.1` 过严 | 放宽 |
| TF 报"multiple authority" | `use_tf` 配置冲突 | 建图 `false`，导航 `true` |

---

## 8. 日志与性能

```bash
# 查看节点日志
ros2 run rqt_console rqt_console

# 记录关键话题
ros2 bag record /scan /odom /imu_data /tf /tf_static /cmd_vel /map

# 查看 CPU/内存
ros2 run rqt_top rqt_top 2>/dev/null || top

# 记录生命周期事件
ros2 topic echo /lifecycle_manager_navigation/transition_event
```

### 8.1 性能建议

| 项 | 建议 |
| --- | --- |
| 编译 | Release + `--parallel-workers` 按内存调整 |
| Nav2 | 控制 20 Hz 已足够；costmap 更新 5 Hz |
| 建图 | Cartographer 子图 60 帧；大场景可增到 90 |
| 日志 | 生产环境用 `--log-level warn` |
| 相机 | 深度图分辨率/帧率按需降低 |

---

## 9. 关闭顺序

1. `Ctrl+C` 结束 launch（lifecycle 会自行 deactivate）。
2. 确认底盘停止：`candump can0` 不再有速度帧。
3. `sudo ip link set can0 down`（可选）。
