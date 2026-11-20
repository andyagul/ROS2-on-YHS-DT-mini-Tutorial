# 17 · 版本与依赖矩阵

[← 上一章：部署运维](16-deployment-operations.md) · [返回索引](README.md) · [下一章：故障诊断 →](18-faq.md)

> 数据来源：`src/**/package.xml`、`scripts/install_*.sh`、`setup.py`、`CMakeLists.txt`。版本号均为**仓库声明值**，未执行构建验证。

---

## 1. 基线环境

| 项 | 声明/要求 | 证据 |
| --- | --- | --- |
| ROS 2 | **Humble**（Ubuntu 22.04 / `jammy`） | Nav2 1.1.5、`yhs_nav2` 的 launch API、`nav2_bringup` 参数 |
| Ubuntu | 22.04 LTS | README、`COLCON_BUILD.md` |
| Python | 3.10 | Ubuntu 22.04 默认，`nav2_simple_commander` |
| C++ | C++17 | Cartographer / Nav2 1.1.5 |
| 构建 | `colcon` + `ament_cmake` / `ament_cmake_python` | 所有包 |
| 中间件 | CycloneDDS 或 FastDDS（默认） | 未锁定，见 §6 |
| 架构 | x86_64 / arm64 | Orbbec 与部分 SDK 需按架构提供二进制 |

**不支持**：Foxy（Nav2 0.4 API 不兼容）、Iron/Jazzy（Nav2 ≥1.2 移除/重命名参数，BT.CPP v4）。

---

## 2. 包版本总表（63 个）

### 2.1 本项目与硬件（15）

| 包 | 版本 | 角色 |
| --- | --- | --- |
| `yhs_can_control` | 0.0.0 | CAN 底盘节点 |
| `yhs_can_interfaces` | 0.0.0 | 消息/动作定义 |
| `yhs_chassis_description` | 0.0.0 | URDF/STL/RViz |
| `yhs_nav2` | 0.0.0 | 编排、参数、地图、脚本 |
| `nvilidar_ros2` | 1.0.0 | 单线雷达驱动 |
| `serial_imu` | 0.0.0 | 串口 IMU 驱动 |
| `orbbec_camera` | 1.5.11 | 深度相机节点 |
| `orbbec_camera_msgs` | **1.2.2** | 相机消息（与主包版本不一致） |
| `orbbec_description` | 0.0.0 | 相机 URDF/STL |
| `pointcloud_to_laserscan` | 2.0.1 | 点云转 LaserScan |
| `lidar_pkg` / `nav_pkg` / `nav_pkg1` / `pc_pkg` / `vel_pub` | 0.0.0 | 教学示例（5 个） |

### 2.2 第三方算法（11）

| 包 | 版本 | 来源 |
| --- | --- | --- |
| `cartographer` | 2.0.9002 | Google Cartographer ROS 2 移植 |
| `cartographer_ros` | 2.0.9000 | 同上 |
| `cartographer_ros_msgs` / `cartographer_rviz` | 2.0.9000 | 同上 |
| `openslam_gmapping` | 0.1.2 | OpenSLAM |
| `slam_gmapping` | 0.0.0 | ROS 2 封装 |
| `teb_local_planner` | 0.9.1 | TU Dortmund |
| `teb_msgs` | 0.0.1 | TEB 消息 |
| `costmap_converter` / `costmap_converter_msgs` | 0.1.2 | TEB 依赖 |
| `behaviortree_cpp_v3` | 3.8.5 | BehaviorTree.CPP v3 |

### 2.3 Nav2（37 个，均 1.1.5）

`nav2_amcl`、`nav2_behavior_tree`、`nav2_behaviors`、`nav2_bringup`、`nav2_bt_navigator`、`nav2_collision_monitor`、`nav2_common`、`nav2_constrained_smoother`、`nav2_controller`、`nav2_core`、`nav2_costmap_2d`、`nav2_lifecycle_manager`、`nav2_map_server`、`nav2_msgs`、`nav2_navfn_planner`、`nav2_planner`、`nav2_regulated_pure_pursuit_controller`、`nav2_rotation_shim_controller`、`nav2_rviz_plugins`、`nav2_simple_commander`、`nav2_smac_planner`、`nav2_smoother`、`nav2_system_tests`、`nav2_theta_star_planner`、`nav2_util`、`nav2_velocity_smoother`、`nav2_voxel_grid`、`nav2_waypoint_follower`、`navigation2`，以及 DWB 子包 `nav2_dwb_controller`、`dwb_core`、`dwb_critics`、`dwb_msgs`、`dwb_plugins`、`costmap_queue`、`nav_2d_msgs`、`nav_2d_utils`。

> `nav2_simple_commander` 的 [setup.py](../../src/navigation2/nav2_simple_commander/setup.py) 仍写 `version='1.0.0'`，与 `package.xml` 的 1.1.5 不一致——只影响 `pip` 元数据，不影响运行。

---

## 3. 第三方编译期版本

Cartographer 的 `scripts/install_*.sh` 固定了以下版本（**可选一键脚本**；若使用系统包，以 apt 实际版本为准）：

| 依赖 | 版本/提交 | 脚本 |
| --- | --- | --- |
| Ceres Solver | `1.13.0` | [install_ceres.sh](../../src/cartographer/scripts/install_ceres.sh) |
| Protobuf | `v3.4.1` | [install_proto3.sh](../../src/cartographer/scripts/install_proto3.sh) |
| Abseil | `d902eb869bcfacc1bad14933ed9af4bed006d481` | [install_abseil.sh](../../src/cartographer/scripts/install_abseil.sh) |
| async_grpc | `771af45374af7f7bfc3b622ed7efbe29a4aba403` | [install_async_grpc.sh](../../src/cartographer/scripts/install_async_grpc.sh) |
| Eigen | 系统 `libeigen3-dev`（jammy 为 3.4.0） | [install_debs_cmake.sh](../../src/cartographer/scripts/install_debs_cmake.sh) |
| Lua | 系统 `liblua5.2-dev`（脚本） | 同上 |
| g2o | 系统 `libg2o-dev`（TEB 依赖） | TEB `package.xml` |
| BehaviorTree.CPP | 仓库内置 3.8.5 | `dependence/BehaviorTree.CPP` |
| Orbbec SDK | 随 `orbbec_camera` 捆绑 | `OrbbecSDK_ROS2` |

> Protobuf 3.4.1 是 Cartographer 上游脚本的历史版本，Ubuntu 22.04 自带 3.21。**不要**为了跑通 Cartographer 而降级系统 Protobuf，优先使用发行版包或仓库自带的 Bazel 构建。

---

## 4. 系统依赖（最小集）

```bash
# 构建工具
sudo apt install -y build-essential cmake ninja-build git python3-colcon-common-extensions \
                    python3-rosdep python3-vcstool

# Cartographer
sudo apt install -y libeigen3-dev libboost-all-dev liblua5.2-dev libceres-dev \
                    libprotobuf-dev protobuf-compiler libcairo2-dev libsuitesparse-dev \
                    libgoogle-glog-dev libgflags-dev

# Nav2 / TEB
sudo apt install -y libg2o-dev libsdl2-dev libyaml-cpp-dev

# 调试
sudo apt install -y ros-humble-tf2-tools ros-humble-rqt* can-utils
```

`rosdep` 仍是首选，可避免漏包：

```bash
sudo rosdep init 2>/dev/null || true
rosdep update
rosdep install --from-paths src --ignore-src -r -y
```

---

## 5. 运行时外部依赖

| 依赖 | 用途 | 缺失后果 |
| --- | --- | --- |
| `can-utils` | `candump` / `cansend` 排查 | 无法验证帧 |
| `tf2_tools` | `view_frames`、`tf2_echo` | 无法排查 TF |
| `nav2_simple_commander` 的 `transforms3d` | Python 示例 | 示例脚本导入失败 |
| `rviz2` + `nav2_rviz_plugins` | 可视化 | 无 GUI 调试 |
| `image_transport` / `camera_info_manager` | Orbbec | 相机节点起不来 |
| `robot_state_publisher` | URDF | 无 `robot_description` |

---

## 6. 兼容性风险矩阵

| 风险 | 影响 | 说明 |
| --- | --- | --- |
| Nav2 版本漂移（≥1.2） | **高** | `recoveries_server`、`default_bt_xml_filename` 等键被移除；BT.CPP 升 v4，插件 ABI 变化 |
| Cartographer ROS 2 移植版 | **高** | 2.0.9000 非官方主线，API/服务名与 ROS1 文档不同 |
| Orbbec 主包/消息包版本不一致 | 中 | 1.5.11 vs 1.2.2；重新编译时需整体替换，勿混用 deb 与源码 |
| 系统 Protobuf 与脚本版本冲突 | 中 | 同时安装源码 3.4.1 和 apt 3.21 会出现符号冲突 |
| DDS 实现未锁定 | 中 | 大点云/多机场景下 FastDDS 与 CycloneDDS 行为不同 |
| `serial_imu` 无参数 | 中 | 设备名/波特率硬编码，换硬件需改源码 |
| `use_sim_time` 不一致 | 高 | 见 K2，Cartographer 栅格节点可能不发布 |
| `nav2_simple_commander` 元数据版本 | 低 | 仅影响包管理信息 |

---

## 7. 锁定与升级策略

1. **锁定 ROS 发行版**：Humble 生命周期内不跨发行版升级。
2. **锁定 Nav2 小版本**：1.1.x；升级前用 `ros2 param dump` 对比参数键。
3. **第三方源码入库**：Cartographer、TEB、GMapping、BT.CPP 已 vendored，升级以替换目录 + 回归测试方式进行。
4. **记录二进制依赖**：Orbbec SDK、CUDA/MPP 解码库与架构绑定，升级需同步固件。
5. **每次交付记录**：`git commit`、`ros2 pkg list | wc -l`、`ros2 doctor --report`。

---

## 8. 版本核对命令

```bash
# ROS / 系统
printenv ROS_DISTRO
lsb_release -a
python3 --version

# 本仓库包版本
for f in $(find src -name package.xml); do
  printf "%-40s %s\n" "$(grep -oP '(?<=<name>)[^<]+' $f)" "$(grep -oP '(?<=<version>)[^<]+' $f)"
done | sort

# 第三方库
pkg-config --modversion eigen3
protoc --version
dpkg -l | grep -E "libceres|libg2o|liblua"

# 运行时
ros2 doctor --report
ros2 pkg executables yhs_can_control
```

---

## 9. 已验证/未验证边界

| 项 | 状态 |
| --- | --- |
| 包版本号一致性 | ✅ 已静态核对 63 个 `package.xml` |
| launch 参数与 YAML 键 | ✅ 已交叉检查，发现 K1/K3/K4/K5 |
| 实际编译通过 | ❌ 未执行 |
| 依赖版本可满足 | ❌ 未执行 `rosdep`/`apt` 安装验证 |
| 实车运行 | ❌ 未执行 |
