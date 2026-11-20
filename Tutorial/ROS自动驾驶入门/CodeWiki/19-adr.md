# 19 · 架构决策记录（ADR）

[← 上一章：故障诊断](18-faq.md) · [返回索引](README.md) · [下一章：术语表 →](20-glossary.md)

> **说明**：仓库没有历史决策文档。以下 ADR 由**代码、launch、配置与依赖关系反推**，用于解释"为什么现在是这个样子"，不代表当时确实开过评审会。每条都给出证据与复审触发条件。
> 状态定义：**已落地** = 代码确实如此；**名义存在** = 配置声明但未真正生效。

---

## ADR-001 采用 ROS 2 Humble + Nav2 1.1.5 作为导航基线

**状态**：已落地

**背景**：项目需要在 2022 年后的 LTS 上长期维护，且要复用 Nav2 的生命周期、行为树、代价地图生态。

**决策**：全栈基于 ROS 2 Humble；Nav2 以源码形式 vendored，版本锁 1.1.5；不使用 ROS 1 Navigation。

**证据**

- [navigation2/package.xml](../../../src/navigation2/navigation2/package.xml) 版本 1.1.5
- 所有 launch 使用 `launch_ros`、`LifecycleNode`、`rclcpp_action`
- [yhs_nav2/package.xml](../../../src/yhs_nav2/package.xml) 为 `format="3"`

**后果**

- 可用 Humble 的长期支持与二进制依赖。
- 被锁定在 Nav2 1.1 API：`recoveries_server`、`default_bt_xml_filename` 等旧键仍在使用（K4）。
- 无法直接升级到 Jazzy（BT.CPP v4、行为服务器重构）。

**复审触发**：ROS 2 Humble EOL（2027-05）；或需要 Nav2 新行为（如 `DriveOnHeading` 的替代实现）。

---

## ADR-002 Cartographer 2D 为主建图方案，GMapping 为备选

**状态**：已落地

**背景**：室内单线雷达 + 轮式差速底盘，需要回环闭合与可保存的 pbstream；团队同时保留了轻量备选。

**决策**：默认 `cartographer.launch.py`；`gmapping.launch.py` 保留用于低算力或快速验证。

**证据**

- [cartographer.launch.py](../../../src/yhs_nav2/launch/cartographer.launch.py) 拉起 IMU、雷达、底盘与 Cartographer
- [gmapping.launch.py](../../../src/yhs_nav2/launch/gmapping.launch.py) 结构相同，替换为 `slam_gmapping`
- [dgt_mini.lua](../../../src/cartographer_ros/cartographer_ros/configuration_files/dgt_mini.lua) 为项目专用配置（`use_imu_data=false`）

**后果**

- Cartographer 提供子图、回环、pbstream，地图质量更好，但 CPU 占用高（K19）。
- GMapping 轻量但无回环、地图易漂移，只适合小场景。
- 两套地图保存方式不同（pbstream vs PGM），运维需区分。

**复审触发**：算力平台降级；或引入 3D 雷达/深度建图。

---

## ADR-003 底盘通过自研 CAN 节点直连，不用 `ros2_control`

**状态**：已落地

**背景**：底盘厂商提供固定 CAN 协议（0x98C4D1EF 等），需要低延迟、可定制的速度/转向/IO 控制。

**决策**：`yhs_can_control` 直接读写 SocketCAN，自行完成协议编解码、里程计积分与 TF 发布；不引入 `ros2_control` 的硬件接口/控制器管理器。

**证据**

- [yhs_can_control_node.cpp](../../../src/yhs_can_control/src/yhs_can_control_node.cpp)：`socket(PF_CAN, SOCK_RAW, CAN_RAW)`、`write`、`read`
- [yhs_can_control.hpp](../../../src/yhs_can_control/include/yhs_can_control/yhs_can_control_node.hpp)：`CanControl` 直接持有 CAN 句柄
- 依赖列表中没有 `hardware_interface` / `controller_manager`

**后果**

- 链路短、延迟低、易适配厂商协议。
- 失去 `ros2_control` 的资源管理、控制器切换、仿真接口。
- 协议硬编码在源码中，换底盘需改代码（K11/K13）。

**复审触发**：需要多底盘统一抽象、仿真（Gazebo/Ignition）或控制器热切换。

---

## ADR-004 轮速积分位置，IMU 直接提供航向

**状态**：已落地

**背景**：轮式差速底盘在打滑时航向误差大，而 IMU 的 yaw 短期更可靠。

**决策**：`publish_odom()` 用左右轮速积分 `x/y`，但 `theta` 直接取 IMU yaw（IMU 有数据时）；同时向 `/odom` 与 TF 发布。

**证据**

- [yhs_can_control_node.cpp](../../../src/yhs_can_control/src/yhs_can_control_node.cpp) 的 `publish_odom()`
- 订阅 `/imu_data` 仅使用 `orientation` 的 yaw
- `use_tf` 参数控制是否广播 `odom→base_link`

**后果**

- 航向稳定，短距离精度好。
- IMU 零偏/回绕会直接污染里程计；无 IMU 时退化为轮速航向。
- 没有协方差建模，Nav2 的 `odom` 不确定性被低估。

**复审触发**：引入 EKF（`robot_localization`）；或 IMU 更换为高频、带磁力计型号。

---

## ADR-005 局部规划器选 TEB

**状态**：已落地

**背景**：场地存在窄通道、动态障碍，需要满足非完整约束且能绕行的局部规划。

**决策**：`FollowPath` 插件使用 `teb_local_planner/TebLocalPlannerROS`；全局规划用 NavFN。

**证据**

- [yhs_nav2.yaml](../../../src/yhs_nav2/param/yhs_nav2.yaml)：`controller_plugins: ["FollowPath"]`，`plugin: "teb_local_planner/TebLocalPlannerROS"`
- [teb_local_planner_plugin.xml](../../../src/teb_local_planner/teb_local_planner/teb_local_planner_plugin.xml)

**后果**

- 支持多边形足迹、动态障碍、速度/加速度约束。
- 参数多、调参成本高；足迹配置错误会导致严重安全风险（K7）。
- 依赖 g2o，编译与运行开销高于 DWB。

**复审触发**：算力不足需要换 DWB/RPP；或需要学习型局部规划。

---

## ADR-006 全局规划器选 NavFN

**状态**：已落地

**背景**：二维栅格地图、差速底盘，需要稳定、可解释的最短路径。

**决策**：`planner_plugins: ["GridBased"]` → `nav2_navfn_planner/NavfnPlanner`，`use_astar: true`（A*），`allow_unknown: false`。

**证据**：[yhs_nav2.yaml](../../../src/yhs_nav2/param/yhs_nav2.yaml) 的 `planner_server` 段。

**后果**

- 稳定、成熟，对代价地图膨胀敏感；`allow_unknown: false` 意味着未知区域不可规划。
- 路径贴近障碍、非光滑，依赖 TEB 做平滑与避障。
- 大地图（>2000×2000）时 A* 耗时明显，可考虑 Smac（配置中已给出注释模板）。

**复审触发**：地图规模增大；需要考虑运动学代价的全局规划。

---

## ADR-007 用行为树编排导航，并扩展"示教路径"动作

**状态**：已落地（扩展节点）/ 名义存在（默认树未接入）

**背景**：需要可配置的导航流程（重规划、恢复、多点），并支持"人工示教一次、之后自动复现"。

**决策**：使用 Nav2 `bt_navigator` + XML 行为树；新增 `RecordPath` 动作、`NavigateFollowRecordPath` 动作服务器（内部作为 `RecordPath` 客户端），以及注册名为 `NavigateFollowRecordPath` 的 BT 动作节点。

**证据**

- [yhs_can_interfaces/action/RecordPath.action](../../../src/yhs_can_interfaces/action/RecordPath.action)
- [navigate_follow_record_path.cpp](../../../src/navigation2/nav2_bt_navigator/src/navigators/navigate_follow_record_path.cpp)
- [navigate_follow_record_path_action.cpp](../../../src/navigation2/nav2_behavior_tree/plugins/action/navigate_follow_record_path_action.cpp)
- 默认 BT 仍为 `navigate_to_pose_w_replanning_and_recovery.xml`，**未包含**自定义节点

**后果**

- 具备示教能力，但默认导航入口不会触发；需要自定义 BT 或外部客户端直接调用动作。
- 自定义代码侵入 Nav2 源码，升级 Nav2 时合并成本高。

**复审触发**：Nav2 升级；或示教功能正式产品化（应改为独立包 + 自定义 BT）。

---

## ADR-008 地图、参数、URDF 作为包内资产随 `install/` 分发

**状态**：已落地

**背景**：单车离线部署，无集中配置中心；需要 `ros2 launch` 开箱即用。

**决策**：地图放 `yhs_nav2/map`，参数放 `yhs_nav2/param`，由 `get_package_share_directory` 解析；URDF/STL 放 `yhs_chassis_description`。

**证据**

- [yhs_nav2.launch.py](../../../src/yhs_nav2/launch/yhs_nav2.launch.py) 的 `default_value=os.path.join(bringup_dir, 'map', 'ecarroom.yaml')`
- [CMakeLists.txt](../../../src/yhs_nav2/CMakeLists.txt) 安装 `map`、`param`、`launch`

**后果**

- 部署简单，不依赖外部路径。
- 修改 `src/` 后必须重新 `colcon build` 才生效（运维陷阱）。
- 多车/多场地需要多份安装或显式覆盖路径。

**复审触发**：多场地集中管理；或引入配置服务器。

---

## ADR-009 声明组合模式，实际以独立进程运行

**状态**：名义存在

**背景**：Nav2 官方 bringup 支持 `use_composition`，可减少进程与通信开销。

**决策**：`yhs_nav2.launch.py` 声明并传递 `use_composition=True`，但同时启动一个 `nav2_container`；子 launch 内部仍用独立 `Node`。

**证据**

- [yhs_nav2.launch.py:109-117](../../../src/yhs_nav2/launch/yhs_nav2.launch.py)：`component_container_isolated`
- [localization_launch.py](../../../src/yhs_nav2/launch/localization_launch.py) / [navigation_launch.py](../../../src/yhs_nav2/launch/navigation_launch.py)：未使用 `ComposableNodeContainer`

**后果**

- 实际是"独立进程 + 一个空容器"，组合模式没有收益（K8）。
- 多一个空闲进程；参数传递容易误解。

**复审触发**：需要降低 CPU/内存；应真正改造子 launch 或删除容器。

---

## ADR-010 第三方算法以源码 vendored 进工作空间

**状态**：已落地

**背景**：Cartographer ROS 2 移植版、TEB、GMapping、BT.CPP 在 Humble 上的二进制包不齐或版本不符。

**决策**：全部源码放入 `src/`，由 `colcon` 统一编译；不依赖 `apt` 的对应版本。

**证据**

- `src/cartographer`、`src/cartographer_ros`、`src/teb_local_planner`、`src/slam_gmapping`、`src/dependence`
- [COLCON_BUILD.md](../../../src/COLCON_BUILD.md) 的构建说明

**后果**

- 版本可控、可打补丁。
- 编译时间长、依赖冲突概率高（Protobuf/Eigen/g2o）。
- 升级需手工合并，缺少上游测试。

**复审触发**：上游发布兼容 Humble 的正式包；或需要频繁跟进上游。

---

## ADR-011 单机器人命名空间，不做 TF 前缀

**状态**：已落地（限制）

**背景**：项目交付以单车为主，优先保证话题与参数简单。

**决策**：提供 `namespace` / `use_namespace`，但不给 TF frame 加前缀；`map`、`odom`、`base_link` 全局唯一。

**证据**

- [yhs_nav2.launch.py:105-107](../../../src/yhs_nav2/launch/yhs_nav2.launch.py)：`PushRosNamespace`
- 参数中 `robot_base_frame: base_link`、`global_frame: map` 为固定值
- URDF/TF 发布无前缀

**后果**

- 单车部署简单。
- 同网多车 TF 冲突；只能靠 `ROS_DOMAIN_ID` 隔离。
- 若要多车协同，需要系统性改造（frame 前缀、参数化、地图命名）。

**复审触发**：多车调度/协同需求立项。

---

## 决策总览

| ADR | 主题 | 状态 | 主要代价 |
| --- | --- | --- | --- |
| 001 | Humble + Nav2 1.1.5 | 已落地 | 版本锁定 |
| 002 | Cartographer 主、GMapping 备 | 已落地 | CPU、双套运维 |
| 003 | 自研 CAN 直连 | 已落地 | 无仿真/控制器抽象 |
| 004 | 轮速位置 + IMU 航向 | 已落地 | IMU 故障直接影响里程计 |
| 005 | TEB 局部规划 | 已落地 | 调参复杂、足迹风险 |
| 006 | NavFN 全局规划 | 已落地 | 路径不光滑 |
| 007 | BT + 示教扩展 | 部分落地 | 默认树未接入 |
| 008 | 包内资产 | 已落地 | 改配置需重编译 |
| 009 | 组合模式 | 名义存在 | 空容器、无收益 |
| 010 | 第三方 vendored | 已落地 | 编译与升级成本 |
| 011 | 单车命名空间 | 已落地 | 多车受限 |
