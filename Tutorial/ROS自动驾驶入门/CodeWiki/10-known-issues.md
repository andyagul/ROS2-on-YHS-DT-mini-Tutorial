# 10 · 已知问题与风险

[← 上一章：编译运行](09-build-run-debug.md) · [返回索引](README.md)

> 本章只收录**静态审计可证实**的问题（文件缺失、API 版本不匹配、配置引用错误、明显复制粘贴）。未实车验证的推测会标注"需实测"。

---

## 1. 问题总览

| ID | 严重度 | 模块 | 问题 | 影响 |
| --- | --- | --- | --- | --- |
| K1 | **P0** | 地图 | `zgq-g.yaml` 引用不存在的 `zgq.pgm` | 该地图无法加载 |
| K2 | **P0** | SLAM | `occupancy_grid_node` 使用仿真时间但无 `/clock` | `/map` 可能不发布 |
| K3 | P1 | 导航 | 局部代价地图未启用 `inflation_layer` | 局部避障偏激进 |
| K4 | P1 | 导航 | YAML 使用旧版 `recoveries_server` 参数 | 恢复行为自定义参数全部失效 |
| K5 | P1 | 导航 | 子 launch 默认 `params/nav2_params.yaml` 不存在 | 单独启动子 launch 直接失败 |
| K6 | P1 | 感知 | 相机与 `/scan1` 被注释但 costmap 仍订阅 | 持续告警、清障缺失 |
| K7 | P1 | 规划 | TEB `footprint_model` 与 costmap `footprint` 不一致 | 碰撞检测偏乐观 |
| K8 | P1 | 编排 | `use_composition=True` 只启动空容器 | 资源浪费、理解误导 |
| K9 | P1 | 模型 | `FR-mid.launch.py` 默认加载 DT-mini；`select_car.sh` D 选项复制错误 | 车型显示错误 |
| K10 | P1 | 打包 | `yhs_nav2` 未声明任何运行依赖 | 部署/rosdep 不完整 |
| K11 | P2 | 驱动 | IMU 设备与波特率硬编码 | 换硬件需改源码 |
| K12 | P2 | 底盘 | 里程计协方差为 0、IMU 可用性判断不可靠 | AMCL 过于自信、航向冻结 |
| K13 | P2 | 底盘 | `gear=3` 写死、话题为相对名 | 无法倒车、命名空间需 remap |
| K14 | P2 | 模型 | URDF 只有 `base_link`；`use_robot_state_pub` 不生效 | 无关节 TF、参数无效 |
| K15 | P2 | 工具 | `block_spawner.py` 依赖 Gazebo/`tb3_worlds` | 实车环境无法运行 |
| K16 | P2 | 工具 | `set_init_amcl_pose.py` 发送后不退出 | 需手动 Ctrl+C |
| K17 | P2 | 工具 | `select_car.sh` 硬编码 `/home/yhs` | 换用户即失效 |
| K18 | P2 | 感知 | 启用相机后可能出现重复 `base_link→camera_link` | TF 多源告警 |
| K19 | P2 | SLAM | `submap_publish_period_sec=5e-3`、`optimize_every_n_nodes=5` | CPU 偏高 |
| K20 | P3 | 文档 | 根 README 引用的部分教程目录不存在 | 链接失效 |

---

## 2. 详细说明

### K1（P0）地图 YAML 指向不存在的 PGM

**证据**

- [zgq-g.yaml](../../../src/yhs_nav2/map/zgq-g.yaml)：`image: zgq.pgm`
- 目录中实际只有 `zgq-c.pgm`，**没有 `zgq.pgm`**
- 其余 8 个 YAML 均能找到同名 PGM

**影响**：`map_server` 启动时 `loadMap()` 打开文件失败，节点 configure 失败，`lifecycle_manager_localization` 卡住，AMCL 不激活 → 导航整体不可用。

**修复**

```bash
cd src/yhs_nav2/map
# 二选一
mv zgq-c.pgm zgq.pgm
# 或
sed -i 's/image: zgq.pgm/image: zgq-c.pgm/' zgq-g.yaml
```

同时把文件名 `zgq-g.yaml` 改为与地图一致的命名，避免误用。

---

### K2（P0）Cartographer 栅格发布节点使用仿真时间

**证据**

- [dgt_mini.launch.py:46-52](../../../src/cartographer_ros/cartographer_ros/launch/dgt_mini.launch.py)：

```python
Node(
    ... executable='cartographer_occupancy_grid_node',
    parameters=[{'use_sim_time': True}, {'resolution': 0.05}]),
```

- `cartographer_node` 自身为 `use_sim_time=False`
- 工作空间没有任何 `/clock` 发布者，`yhs_nav2.yaml` 全部为 `False`

**影响**：ROS 时间恒为 0，基于定时器的 `/map` 发布不会推进（需实测确认）。表现为 RViz 只有 `submap_list` 可视化，没有栅格地图。

**修复**：把该参数改为 `False`，或由 `dgt_mini.launch.py` 接收外层 `use_sim_time`：

```python
parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}, {'resolution': 0.05}]
```

---

### K3（P1）局部代价地图缺少膨胀层

**证据**：[yhs_nav2.yaml:237-282](../../../src/yhs_nav2/param/yhs_nav2.yaml)

```yaml
plugins: ["static_layer", "obstacle_layer"]
inflation_layer:
  plugin: "nav2_costmap_2d::InflationLayer"
  inflation_radius: 1.0
```

`inflation_layer` 段存在，但**未加入 `plugins` 列表**。

**影响**：局部代价地图只有致命障碍代价，没有距离衰减场；TEB 虽然用 `min_obstacle_dist=0.45` 做硬约束，但缺少"提前远离"的软引导，贴边/绕障更激进。

**修复**

```yaml
plugins: ["static_layer", "obstacle_layer", "inflation_layer"]
```

并确认 `inflation_radius`（1.0）与全局（0.55）的差异是否符合预期。

---

### K4（P1）Nav2 参数使用旧版 API

**证据**（[yhs_nav2.yaml](../../../src/yhs_nav2/param/yhs_nav2.yaml)）

| 位置 | 旧配置 | Nav2 1.1.5 实际 |
| --- | --- | --- |
| `:61` | `default_bt_xml_filename` | `default_nav_to_pose_bt_xml`（[navigate_to_pose.cpp:71-79](../../../src/navigation2/nav2_bt_navigator/src/navigators/navigate_to_pose.cpp)）；且旧值 `navigate_w_replanning_and_recovery.xml` 在本仓库不存在 |
| `:401` | `recoveries_server:` | `behavior_server:` |
| `:406` | `recovery_plugins` | `behavior_plugins` |
| `:408-415` | `nav2_recoveries/Spin` 等 | `nav2_behaviors/Spin` 等 |

**影响**

- BT 仍能工作（默认树已内置），但 `default_bt_xml_filename` 完全被忽略。
- `behavior_server` 读不到任何参数，回退默认插件与默认参数：**恢复行为能用，但 `simulate_ahead_time`、`max_rotational_vel`、`transform_timeout` 等全部不生效**。
- 若只把 `recoveries_server` 改名为 `behavior_server` 却保留 `nav2_recoveries/*` 插件名，会因找不到插件而 configure 失败。

**修复**

```yaml
bt_navigator:
  ros__parameters:
    default_nav_to_pose_bt_xml: "navigate_to_pose_w_replanning_and_recovery.xml"  # 或无该行用默认

behavior_server:
  ros__parameters:
    costmap_topic: local_costmap/costmap_raw
    footprint_topic: local_costmap/published_footprint
    cycle_frequency: 10.0
    behavior_plugins: ["spin", "backup", "drive_on_heading", "wait"]
    spin:
      plugin: "nav2_behaviors/Spin"
    backup:
      plugin: "nav2_behaviors/BackUp"
    drive_on_heading:
      plugin: "nav2_behaviors/DriveOnHeading"
    wait:
      plugin: "nav2_behaviors/Wait"
    # …其余参数
```

---

### K5（P1）子 launch 默认参数文件不存在

**证据**

- [localization_launch.py:66-69](../../../src/yhs_nav2/launch/localization_launch.py)：默认 `bringup_dir/params/nav2_params.yaml`
- [navigation_launch.py:62-65](../../../src/yhs_nav2/launch/navigation_launch.py)：同上
- `yhs_nav2` 实际目录是 `param/`（单数），且只有 `yhs_nav2.yaml`

**影响**：直接 `ros2 launch yhs_nav2 localization_launch.py` 会因 `RewrittenYaml` 打开失败而报错。通过顶层 `yhs_nav2.launch.py` 启动时传入了正确路径，因此**导航主流程不受影响**。

**修复**：把两个子 launch 的默认值改为 `param/yhs_nav2.yaml`，或把目录统一为 `params/`。

---

### K6（P1）相机链路被注释但 costmap 仍依赖 `/scan1`

**证据**

- [yhs_nav2.launch.py:133-146](../../../src/yhs_nav2/launch/yhs_nav2.launch.py)：相机与点云转激光均被注释
- [yhs_nav2.yaml:259-271](../../../src/yhs_nav2/param/yhs_nav2.yaml)：`observation_sources: scan scan1`
- 全局 costmap 同样包含 `scan1`

**影响**：costmap 持续输出"observation buffer empty"类告警；局部/全局清障只依赖 `/scan`，矮障碍补盲缺失。

**修复**：二选一

1. 启用相机与 `pointcloud_to_laserscan`；
2. 从两处 `observation_sources` 删除 `scan1` 及对应配置。

---

### K7（P1）TEB 与 costmap 的外形不一致

**证据**

- [yhs_nav2.yaml:153-155](../../../src/yhs_nav2/param/yhs_nav2.yaml)：TEB `footprint_model.type: circular`，`radius: 0.1`
- 同文件 `local_costmap.footprint` / `global_costmap.footprint`：`[[0.30,0.26],...]`（0.6 × 0.52 m 矩形）

**影响**：TEB 以为车是半径 0.1 m 的圆，实际车体 0.6 m 长；在窄通道会给出"能过"的错误判断。

**修复**

```yaml
FollowPath:
  footprint_model:
    type: "polygon"
    vertices: "[[0.30,0.26],[0.30,-0.26],[-0.30,-0.26],[-0.30,0.26]]"
```

---

### K8（P1）组合模式只启动空容器

**证据**

- [yhs_nav2.launch.py:109-117](../../../src/yhs_nav2/launch/yhs_nav2.launch.py)：`use_composition=True` 时启动 `nav2_container`
- 子 launch 中所有服务器均为独立 `Node(...)`，没有 `LoadComposableNodes`

**影响**：多起一个 60 MB 左右的空容器；`use_composition` 参数不改变实际部署。不崩溃，但会误导性能优化。

**修复**：改为 `use_composition:=False` 默认，或真正实现 `LoadComposableNodes`。

---

### K9（P1）车型 launch 复制粘贴错误

**证据**

- [FR-mid.launch.py:21](../../../src/yhs_chassis_description/launch/FR-mid.launch.py)：默认 `urdf/DT-mini.urdf`
- `FR_mid.urdf` 存在但无任何 launch 引用
- [select_car.sh:36-41](../../../src/tools/select_car.sh)：D（FR-mini）复制的是 `DT-mini.launch.py`

**影响**：选择 FR-mid 实际显示 DT-mini；选择 FR-mini 实际也是 DT-mini。外观/尺寸误判会连带影响 footprint 标定。

**修复**：两处都改为正确文件名，并把 `FR_mid.urdf` 重命名为 `FR-mid.urdf` 保持一致。

---

### K10（P1）编排包没有运行依赖

**证据**：[package.xml](../../../src/yhs_nav2/package.xml) 只有 `ament_cmake` 与测试依赖，没有 `<exec_depend>`。

**影响**

- `rosdep install --from-paths src` 不会为 `yhs_nav2` 拉取 `nav2_bringup`、`nav2_map_server`、`cartographer_ros`、`serial_imu`、`nvilidar_ros2`、`yhs_can_control`。
- 单独分发 `yhs_nav2` 时启动即失败。

**修复**：至少补充：

```xml
<exec_depend>nav2_bringup</exec_depend>
<exec_depend>cartographer_ros</exec_depend>
<exec_depend>slam_gmapping</exec_depend>
<exec_depend>serial_imu</exec_depend>
<exec_depend>nvilidar_ros2</exec_depend>
<exec_depend>yhs_can_control</exec_depend>
<exec_depend>yhs_chassis_description</exec_depend>
<exec_depend>python3-transforms3d</exec_depend>
```

---

### K11（P2）IMU 设备与波特率硬编码

**证据**：[serial_port.cpp:17-18](../../../src/hardware/serial_imu/src/serial_port.cpp)

```c
#define IMU_SERIAL ("/dev/imu")
#define BAUD       (B115200)
```

**影响**：更换串口或波特率必须重新编译；无法通过 launch 参数适配不同硬件。

**修复**：改为 `declare_parameter("port", "/dev/imu")`、`declare_parameter("baud", 115200)`，并用 `termios` 动态设置。

---

### K12（P2）里程计与 IMU 判断的隐患

**证据**：[yhs_can_control_node.cpp:412](../../../src/yhs_can_control/src/yhs_can_control_node.cpp)

```cpp
auto publisher_count = node_->count_publishers("/imu_data");
```

以及 `publish_odom()` 中协方差矩阵未赋值。

**影响**

- `count_publishers` 只说明"有节点在发布"，IMU 节点假死时航向会冻结在最后一个值。
- 协方差全 0 会让 AMCL 完全信任 `odom`，遇到打滑/碰撞时不易纠正。
- IMU yaw 跨越 ±π 时位姿会跳变。

**修复**

- 记录最后 IMU 时间戳，超过阈值（如 0.2 s）回退到角速度积分。
- 填充 `pose.covariance` 与 `twist.covariance` 的对角线。
- 航向做连续化（unwrap）处理。

---

### K13（P2）挡位写死与相对话题名

**证据**：[yhs_can_control_node.cpp:153-161](../../../src/yhs_can_control/src/yhs_can_control_node.cpp)

```cpp
ctrl_cmd_msg->ctrl_cmd_angular = cmd_msg->angular.z / 3.14 * 180;
ctrl_cmd_msg->ctrl_cmd_gear = 3;
```

话题 `cmd_vel`、`odom`、`imu_data` 均为相对名。

**影响**：`/cmd_vel` 无法倒车（固定前进挡）；放入命名空间后话题变为 `/ns/cmd_vel`，与 Nav2 默认根命名空间不一致，需要 remap。

**修复**：`gear` 从参数或 `Twist` 的符号推导；话题改为绝对名或统一加 remap。

---

### K14（P2）URDF 只有 base_link，且参数未生效

**证据**

- [DT-mini.urdf](../../../src/yhs_chassis_description/urdf/DT-mini.urdf)：仅 1 个 `<link>`，无 `<joint>`
- [robot.launch.py:29-32](../../../src/yhs_chassis_description/launch/robot.launch.py)：声明 `use_robot_state_pub` 但未用于条件
- [robot.launch.py:52-56](../../../src/yhs_chassis_description/launch/robot.launch.py)：无条件启动 `joint_state_publisher`

**影响**：没有关节可发布，`robot_state_publisher` 只发布静态 `base_link`；`use_robot_state_pub:=False` 无效。

**修复**：补充轮子 `continuous joint` 与 `collision/inertial`（如需仿真）；给 `robot_state_publisher` 加 `IfCondition`；无关节车型不启动 `joint_state_publisher`。

---

### K15（P2）`block_spawner.py` 依赖 Gazebo

**证据**：[block_spawner.py:17-22](../../../src/yhs_nav2/scripts/block_spawner.py)

```python
from gazebo_msgs.srv import SpawnEntity
model_dir = os.path.join(get_package_share_directory("tb3_worlds"), "models")
```

**影响**：本工作空间没有 `tb3_worlds`、`gazebo_msgs`，且全栈面向实车；脚本一运行即 `ModuleNotFoundError` / `PackageNotFoundError`。

**修复**：实车部署时删除该脚本的安装；或抽象为仿真专用包，不要放在 `yhs_nav2` 主包。

---

### K16（P2）`set_init_amcl_pose.py` 发送后不退出

**证据**：[set_init_amcl_pose.py:56-65](../../../src/yhs_nav2/scripts/set_init_amcl_pose.py)

发送一次 `/initialpose` 后 `rclpy.spin(pub)` 常驻。

**影响**：一次性脚本变成常驻节点，用户容易误以为卡死；重复运行会重复发送。

**修复**：发送后延时 0.5 s 调用 `rclpy.shutdown()` 退出；或改为 `rclpy.spin_once` + `destroy_node`。

---

### K17（P2）车型切换脚本硬编码路径

**证据**：[select_car.sh:22](../../../src/tools/select_car.sh) 等 7 处 `/home/yhs/ros2_ws/...`。

**影响**：换用户名/工作空间即失败；脚本还会覆盖 `robot.launch.py`。

**修复**：使用 `$(dirname "$0")` 与 `ros2 pkg prefix` 动态定位；或改为通过 `urdf_model` 参数选择。

---

### K18（P2）启用相机后可能重复发布相机 TF

**证据**

- [pointcloud_to_laserscan.launch.py](../../../src/pointcloud_to_laserscan/launch/pointcloud_to_laserscan.launch.py) 发布静态 `base_link→camera_link`
- `orbbec_camera` 节点通常也会根据 URDF/参数发布 `camera_link` 及子坐标系

**影响**：两处同时发布 `base_link→camera_link`，`tf2` 报 "TF_REPEATED_DATA" 或 "multiple authority" 警告，极端情况下 TF 抖动。

**修复**：只保留一处发布者。若相机节点发布 TF，则删除 `pointcloud_to_laserscan.launch.py` 中的静态 TF；反之给相机设 `publish_tf:=false`。

---

### K19（P2）Cartographer 高频参数

**证据**：[dgt_mini.lua:34-35, 64](../../../src/cartographer_ros/cartographer_ros/configuration_files/dgt_mini.lua)

```lua
submap_publish_period_sec = 5e-3,   -- 200 Hz
pose_publish_period_sec = 30e-3,    -- 33 Hz
POSE_GRAPH.optimize_every_n_nodes = 5
```

**影响**：低算力平台（如 Jetson Nano/树莓派）CPU 占用高，可能拖慢 20 Hz 控制循环。

**修复**：`submap_publish_period_sec` 改为 `0.3`（约 3 Hz）即可满足可视化；`optimize_every_n_nodes` 提高到 `20~35`。

---

### K20（P3）README 引用的部分目录不存在

**证据**：根 README 的"建议学习路径"引用了 `Tutorial/ROS自动驾驶入门`，实际 `Tutorial/` 下只有 `ROS自动驾驶二次开发`。

**影响**：文档链接失效，不影响代码。

**修复**：补齐教程或修正 README。

---

## 3. 修复优先级建议

| 顺序 | 事项 | 理由 |
| --- | --- | --- |
| 1 | K1 修地图引用 | 直接阻塞导航 |
| 2 | K2 修 `use_sim_time` | 直接阻塞建图产物 |
| 3 | K4 迁移 Nav2 参数 | 影响恢复行为与后续升级 |
| 4 | K3 启用局部膨胀层 | 安全相关 |
| 5 | K7 统一 footprint | 安全相关 |
| 6 | K5/K8/K10 | 提升可维护性与部署正确性 |
| 7 | K6/K11/K12/K13 | 稳定性与硬件适配 |
| 8 | K9/K14~K20 | 代码与文档卫生 |

---

## 4. 审计方法说明

- 使用 `grep`/`find` 对配置引用做**存在性校验**（地图、launch、参数文件、插件类）。
- 对 Nav2 参数键逐一比对 1.1.5 的 `declare_parameter` 调用。
- 对 launch 的 `IncludeLaunchDescription` 递归展开，核对节点与参数。
- 对 CAN 协议按字节位手工解码并与消息字段对照。
- **未执行**编译、单元测试、实车运行；K2/K6/K12 等运行时行为需实机复核。
