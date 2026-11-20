# 02 · 包清单与依赖关系

[← 返回索引](README.md) · [下一章：底盘控制 →](03-can-chassis.md)

---

## 1. 统计口径

- `package.xml` 总数：**63**
- 顶层目录：17 个（14 个非隐藏目录，其中 13 个含 `package.xml`；`tools/` 仅脚本；另有 `.slidep/`、`.vscode/`、`.workbuddy/` 三个隐藏目录）
- 源码规模：约 **1605** 个 `.c/.cc/.cpp/.h/.hpp` 文件
- 体积占比：`navigation2` 112 MB、`hardware` 42 MB、`yhs_chassis_description` 23 MB（STL 为主）

---

## 2. 自研 / 教学包（9 个）

| 包 | 版本 | 职责 | 关键依赖 | 主要入口 |
| --- | --- | --- | --- | --- |
| `yhs_can_interfaces` | 0.0.0 | 底盘 msg/action 定义（10 msg + 1 action） | `std_msgs`、`nav_msgs`、`geometry_msgs` | 编译期生成 |
| `yhs_can_control` | 0.0.0 | SocketCAN 底盘驱动：`/cmd_vel`↔CAN、`/odom`、`/chassis_info_fb` | `yhs_can_interfaces`、`rclcpp`、`tf2_ros`、`nav_msgs` | `yhs_can_control_node` |
| `yhs_chassis_description` | 0.0.0 | 7 种车型 URDF + STL + RViz 配置 | 仅 `ament_cmake`（运行时用 `robot_state_publisher`） | 8 个 `*.launch.py` |
| `yhs_nav2` | 0.0.0 | 编排：启动组合、参数、地图、初始位姿脚本 | **未声明任何运行依赖**（见问题 #5） | 5 个 launch + 2 脚本 |
| `yhs_tutorials/vel_pub` | 0.0.0 | 教学：100 Hz 发布 `/cmd_vel` | `rclcpp`、`geometry_msgs` | `vel_pub_node` |
| `yhs_tutorials/lidar_pkg` | 0.0.0 | 教学：订阅激光 | `sensor_msgs` | `lidar_pkg_node` |
| `yhs_tutorials/pc_pkg` | 0.0.0 | 教学：点云订阅 | `sensor_msgs`/PCL | `pc_node` |
| `yhs_tutorials/nav_pkg` | 0.0.0 | 教学：`NavigateToPose` 动作客户端 + 状态订阅 | `nav2_msgs`、`tf2` | `nav_pkg_node` |
| `yhs_tutorials/nav_pkg1` | 0.0.0 | 教学：空壳包（仅 CMake/package.xml） | — | — |

> `yhs_tutorials/vel_pub` 会**持续发布 0.1 m/s 前进 + 0.1 rad/s 转向**，仅用于课堂演示，实车运行前必须先启动急停或改代码。

---

## 3. 硬件驱动包（5 个）

| 包 | 版本 | 职责 | 关键接口 |
| --- | --- | --- | --- |
| `nvilidar_ros2` | 1.0.0 | nvilidar 单线雷达驱动 + 内置 SDK | 发布 `scan`（`laser_link`），串口 `/dev/lidar` 512000 或 UDP |
| `serial_imu` | 0.0.0 | 串口 IMU，WIT 类协议解析 | 发布 `/imu_data`（`imu_link`），硬编码 `/dev/imu` |
| `orbbec_camera` | 1.5.11 | Orbbec 深度相机（组件化） | 发布 image/depth/points/camera_info |
| `orbbec_camera_msgs` | 1.5.11 | 相机私有 msg/srv | — |
| `orbbec_description` | 1.5.11 | 相机 URDF/mesh/view_model | — |

---

## 4. SLAM 包（6 个）

| 包 | 版本 | 职责 | 关键依赖 |
| --- | --- | --- | --- |
| `cartographer` | 2.0.9002 | SLAM 核心库：common/sensor/transform/mapping/io/ground_truth/cloud | `absl`、`ceres`、`protobuf`、`lua`、`eigen` |
| `cartographer_ros` | 2.0.9000 | ROS 2 桥接与节点（Cartographer 官方 ROS 2 分支） | `cartographer`、`cartographer_ros_msgs`、`tf2_ros`、`rosbag2` |
| `cartographer_ros_msgs` | 2.0.9000 | 子图/轨迹/状态相关 msg+srv | `std_msgs`、`geometry_msgs` |
| `cartographer_rviz` | 2.0.9000 | 子图可视化 RViz 插件 | `rviz_common`、`Ogre` |
| `openslam_gmapping` | 0.1.2 | GMapping 算法本体（grid/scanmatcher/particlefilter） | 纯 C++ |
| `slam_gmapping` | 0.0.0 | GMapping 的 ROS 2 封装 | `openslam_gmapping`、`tf2_ros`、`nav_msgs` |

---

## 5. 导航与规划（43 个）

### 5.1 Nav2 1.1.5（37 个包，含 DWB 子包）

| 分类 | 包 | 职责 |
| --- | --- | --- |
| 基础/接口 | `nav2_core` | 插件基类：`Controller`、`Planner`、`Behavior`、`Smoother`、`GoalChecker`、`ProgressChecker` |
| | `nav2_util` | `LifecycleNode`、`SimpleActionServer`、`RobotUtils`、`NodeThread` |
| | `nav2_common` | launch 工具（`RewrittenYaml`）、CMake 宏 |
| | `nav2_msgs` | Nav2 全部 msg/srv/action |
| 定位/地图 | `nav2_amcl` | 自适应蒙特卡洛定位（粒子滤波） |
| | `nav2_map_server` | 地图加载/保存、`map_saver_cli` |
| 全局规划 | `nav2_planner` | 规划服务器（插件宿主） |
| | `nav2_navfn_planner` | NavFn（Dijkstra/A*） |
| | `nav2_smac_planner` | SMAC 2D/Hybrid/Lattice |
| | `nav2_theta_star_planner` | Theta* |
| | `nav2_smoother` + `nav2_constrained_smoother` | 路径平滑 |
| 局部控制 | `nav2_controller` | 控制服务器（插件宿主） |
| | `nav2_regulated_pure_pursuit_controller` | RPP |
| | `nav2_rotation_shim_controller` | 起步旋转转接 |
| | `nav2_dwb_controller` | DWB 元包（另含 7 个子包：`dwb_core`、`dwb_critics`、`dwb_msgs`、`dwb_plugins`、`costmap_queue`、`nav_2d_msgs`、`nav_2d_utils`；本仓库未启用） |
| | `nav2_velocity_smoother` | 速度平滑与限幅 |
| 行为/决策 | `nav2_behavior_tree` | BT 节点库 + 自定义 `NavigateFollowRecordPath` |
| | `nav2_bt_navigator` | 行为树导航器（含自定义 `NavigateFollowRecordPathNavigator`） |
| | `nav2_behaviors` | `spin`/`backup`/`wait`/`drive_on_heading` |
| | `nav2_waypoint_follower` | 多点巡航 |
| 代价地图 | `nav2_costmap_2d` | 分层代价地图与插件 |
| | `nav2_voxel_grid` | 3D 体素栅格 |
| 生命周期 | `nav2_lifecycle_manager` | 节点批量状态迁移 |
| 监控 | `nav2_collision_monitor` | 碰撞监测（本仓库未启用） |
| 工具/可视化 | `nav2_rviz_plugins` | Nav2 RViz 面板（含自研"录制路径"按钮） |
| | `nav2_simple_commander` | Python 简易 API |
| | `nav2_system_tests` | 系统测试 |
| 元包 | `navigation2` | 依赖聚合 |

### 5.2 其他导航/依赖包（6 个）

| 包 | 版本 | 职责 |
| --- | --- | --- |
| `teb_local_planner` | 0.9.1 | 时间弹性带局部规划器（`nav2_core::Controller` 插件） |
| `teb_msgs` | — | TEB 相关消息（可选反馈） |
| `costmap_converter` | 0.1.2 | 代价地图→多边形/障碍物，供 TEB 使用 |
| `costmap_converter_msgs` | 0.1.2 | 上述消息定义 |
| `behavior_tree_cpp_v3`（目录 `dependence/BehaviorTree.CPP`） | v3 | 行为树引擎（Nav2 依赖） |
| `pointcloud_to_laserscan` | 2.0.1 | 点云转 LaserScan，产出 `/scan1` |

---

## 6. 依赖关系图

```
yhs_can_interfaces
   └── yhs_can_control ──► rclcpp/tf2/nav_msgs
                                  ▲
                                  │ /odom
cartographer ──► cartographer_ros_msgs ──► cartographer_ros
        ▲                                        ▲
        └──────── yhs_nav2 launch ───────────────┘

nav2_msgs ──► nav2_core/nav2_util ──► nav2_costmap_2d
                                   ├─► nav2_amcl
                                   ├─► nav2_planner ──► nav2_navfn_planner / smac / theta*
                                   ├─► nav2_controller ──► teb_local_planner
                                   ├─► nav2_bt_navigator ──► nav2_behavior_tree
                                   └─► nav2_behaviors / waypoint_follower / velocity_smoother

teb_msgs ──► teb_local_planner
costmap_converter_msgs ──► costmap_converter ──► teb_local_planner
BehaviorTree.CPP v3 ──► nav2_behavior_tree ──► nav2_bt_navigator
yhs_can_interfaces ──► nav2_behavior_tree（自定义 RecordPath 动作）
```

---

## 7. 构建顺序（colcon 自动推导）

1. `yhs_can_interfaces` → `yhs_can_control`
2. `cartographer` → `cartographer_ros_msgs` → `cartographer_ros` / `cartographer_rviz`
3. `nav2_msgs` → `nav2_util` / `nav2_core` → `nav2_costmap_2d` → 各 server → `nav2_bringup`
4. `BehaviorTree.CPP` → `nav2_behavior_tree` → `nav2_bt_navigator`
5. `teb_msgs` / `costmap_converter_msgs` → `costmap_converter` → `teb_local_planner`
6. `nvilidar_ros2`、`serial_imu`、`orbbec_*` 与上述并行
7. `yhs_nav2`、`yhs_chassis_description`（仅安装资源，无编译依赖）

> 由于 `yhs_nav2/package.xml` 没有声明 `<exec_depend>`，`rosdep install` 不会为它拉取 `serial_imu`、`nvilidar_ros2`、`yhs_can_control`、`nav2_*`。这些包必须在本工作空间内一起编译，或手动安装。

---

## 8. 包的三类来源

| 来源 | 包 | 维护方式 |
| --- | --- | --- |
| 本项目自研 | `yhs_*`、`tools` | 可直接改 |
| 厂商/课程提供 | `nvilidar_ros2`、`serial_imu`、`OrbbecSDK_ROS2`、`slam_gmapping` | 谨慎改，注意 SDK |
| 上游开源 | `cartographer*`、`navigation2`、`teb_local_planner`、`pointcloud_to_laserscan`、`BehaviorTree.CPP`、`costmap_converter` | 优先改配置，避免改库 |

许可证：自研部分随课程版权声明；第三方分别遵循 Apache-2.0 / BSD / LGPL 等，详见各包 `LICENSE` 与仓库根 README 的"内置开源组件许可证"。
