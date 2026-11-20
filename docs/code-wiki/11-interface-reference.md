# 11 · 接口参考字典

[← 上一章：已知问题](10-known-issues.md) · [返回索引](README.md) · [下一章：代码地图 →](12-code-map.md)

---

本章汇总系统运行时对外暴露的**话题、服务、动作、TF 与自定义消息**。命名默认以根命名空间 `/` 为准（`yhs_nav2.launch.py` 的 `namespace` 默认为空，且 `use_namespace=false`）。

> 约定：表中"发布者"指默认启动路线下的节点；括号内为可选的建图/导航差异。

---

## 1. 话题（Topics）

### 1.1 传感器与底盘

| 话题 | 类型 | 发布者 | 订阅者 | QoS / 备注 |
| --- | --- | --- | --- | --- |
| `/scan` | `sensor_msgs/msg/LaserScan` | `nvilidar_ros2_node` | Cartographer、GMapping、AMCL、代价地图 | **Best Effort**（`SensorDataQoS`） |
| `/scan1` | `sensor_msgs/msg/LaserScan` | `pointcloud_to_laserscan` | 全局/局部代价地图 | 需启用深度相机 |
| `/imu_data` | `sensor_msgs/msg/Imu` | `serial_imu_node` | `yhs_can_control`（仅用于 yaw） | 深度 20，reliable |
| `/odom` | `nav_msgs/msg/Odometry` | `yhs_can_control_node` | Nav2、AMCL、TEB、速度平滑器 | 始终发布；建图时需 `use_tf:=false` 避免与 Cartographer 争 `odom→base_link` |
| `/chassis_info_fb` | `yhs_can_interfaces/msg/ChassisInfoFb` | `yhs_can_control_node` | 上层监控/录包 | 仅在收到 `0x98C4D1EF` 后发布 |
| `/cmd_vel` | `geometry_msgs/msg/Twist` | Nav2 控制器 / `vel_pub_node` | `yhs_can_control_node` | 深度 1 |
| `/cmd_vel_nav` | `geometry_msgs/msg/Twist` | `controller_server` | `velocity_smoother` | 由 launch 重映射产生 |
| `/cmd_vel_smoothed` | `geometry_msgs/msg/Twist` | `velocity_smoother` | 重映射为 `/cmd_vel` 后到底盘 | 由 launch 重映射产生 |

> **建图关键差异**：Cartographer 路线必须给底盘传 `use_tf:=false`，否则 `/odom` 与 `odom→base_link` 会出现两个来源（详见 [05 SLAM](05-slam.md)）。

### 1.2 定位、地图与规划

| 话题 | 类型 | 发布者 | 订阅者 | 备注 |
| --- | --- | --- | --- | --- |
| `/map` | `nav_msgs/msg/OccupancyGrid` | `map_server` 或 `cartographer_occupancy_grid_node` | AMCL、全局代价地图、RViz | 地图为 **transient_local**（latched） |
| `/amcl_pose` | `geometry_msgs/msg/PoseWithCovarianceStamped` | `amcl` | RViz、上层 | transient_local + reliable |
| `/particle_cloud` | `nav2_msgs/msg/ParticleCloud` | `amcl` | RViz | **Best Effort** |
| `/initialpose` | `geometry_msgs/msg/PoseWithCovarianceStamped` | RViz / `set_init_amcl_pose.py` | `amcl` | 重定位入口 |
| `/plan` | `nav_msgs/msg/Path` | `planner_server` | RViz、行为树 | 全局路径 |
| `/record_path` | `nav_msgs/msg/Path` | RViz 录制工具 / 上层 | `bt_navigator` | **示教循迹入口**，收到即触发动作 |
| `/global_costmap/costmap` | `nav_msgs/msg/OccupancyGrid` | `global_costmap` | RViz | 代价地图可视化 |
| `/local_costmap/costmap` | `nav_msgs/msg/OccupancyGrid` | `local_costmap` | RViz | 局部代价地图 |
| `/camera/color/image_raw` | `sensor_msgs/msg/Image` | `ob_camera_node` | 上层 | 需启用相机 |
| `/camera/depth/points` | `sensor_msgs/msg/PointCloud2` | `ob_camera_node` | `pointcloud_to_laserscan` | 需启用相机 |

### 1.3 TF 与系统

| 话题 | 类型 | 发布者 | 备注 |
| --- | --- | --- | --- |
| `/tf` | `tf2_msgs/msg/TFMessage` | `yhs_can_control` / Cartographer / AMCL / `robot_state_publisher` | 动态变换 |
| `/tf_static` | `tf2_msgs/msg/TFMessage` | nvilidar / serial_imu / pointcloud_to_laserscan launch | 静态外参 |
| `/clock` | `rosgraph_msgs/msg/Clock` | **无** | 全系统应保持 `use_sim_time=false`（K2 例外） |
| `/rosout` | `rcl_interfaces/msg/Log` | 所有节点 | 日志 |
| `/parameter_events` | `rcl_interfaces/msg/ParameterEvent` | 所有节点 | 参数事件 |

---

## 2. 服务（Services）

### 2.1 Cartographer

| 服务 | 类型 | 作用 |
| --- | --- | --- |
| `/submap_query` | `cartographer_ros_msgs/srv/SubmapQuery` | 查询子图栅格与位姿 |
| `/trajectory_query` | `cartographer_ros_msgs/srv/TrajectoryQuery` | 查询轨迹节点 |
| `/start_trajectory` | `cartographer_ros_msgs/srv/StartTrajectory` | 启动新轨迹 |
| `/finish_trajectory` | `cartographer_ros_msgs/srv/FinishTrajectory` | 结束轨迹（保存 pbstream 前调用） |
| `/write_state` | `cartographer_ros_msgs/srv/WriteState` | 导出 `.pbstream` |
| `/get_trajectory_states` | `cartographer_ros_msgs/srv/GetTrajectoryStates` | 查询轨迹状态 |
| `/read_metrics` | `cartographer_ros_msgs/srv/ReadMetrics` | 读取内部指标 |

### 2.2 地图与定位

| 服务 | 类型 | 作用 |
| --- | --- | --- |
| `/map_server/map` | `nav_msgs/srv/GetMap` | 获取当前地图 |
| `/map_server/load_map` | `nav2_msgs/srv/LoadMap` | 运行时切换地图 |
| `/map_saver/save_map` | `nav2_msgs/srv/SaveMap` | 保存地图到磁盘 |
| `/reinitialize_global_localization` | `std_srvs/srv/Empty` | AMCL 全局重定位 |
| `/request_nomotion_update` | `std_srvs/srv/Empty` | 强制一次 AMCL 更新（机器人未动时） |

### 2.3 代价地图与生命周期

| 服务 | 类型 | 作用 |
| --- | --- | --- |
| `/global_costmap/clear_entirely_global_costmap` | `nav2_msgs/srv/ClearEntireCostmap` | 清空全局代价地图 |
| `/global_costmap/clear_around_global_costmap` | `nav2_msgs/srv/ClearCostmapAroundRobot` | 清除机器人周围 |
| `/local_costmap/clear_entirely_local_costmap` | `nav2_msgs/srv/ClearEntireCostmap` | 清空局部代价地图 |
| `/local_costmap/clear_around_local_costmap` | `nav2_msgs/srv/ClearCostmapAroundRobot` | 清除机器人周围 |
| `/lifecycle_manager_localization/change_state` | `lifecycle_msgs/srv/ChangeState` | 管理定位链路生命周期 |
| `/lifecycle_manager_navigation/change_state` | `lifecycle_msgs/srv/ChangeState` | 管理导航链路生命周期 |
| `/lifecycle_manager_*/get_state` | `lifecycle_msgs/srv/GetState` | 查询状态 |
| `/lifecycle_manager_*/get_available_transitions` | `lifecycle_msgs/srv/GetAvailableTransitions` | 查询可用切换 |

> 代价地图服务名由 `"clear_entirely_" + costmap.getName()` 拼接，节点命名空间为 `global_costmap` / `local_costmap`，因此出现"双名"属正常现象（[clear_costmap_service.cpp:51-58](../../src/navigation2/nav2_costmap_2d/src/clear_costmap_service.cpp)）。

---

## 3. 动作（Actions）

| 动作名 | 类型 | 服务节点 | 客户端 / 用途 |
| --- | --- | --- | --- |
| `/navigate_to_pose` | `nav2_msgs/action/NavigateToPose` | `bt_navigator` | RViz "2D Goal Pose"、`nav_pkg` |
| `/navigate_through_poses` | `nav2_msgs/action/NavigateThroughPoses` | `bt_navigator` | 多点巡航 |
| `/navigate_follow_record_path` | `yhs_can_interfaces/action/RecordPath` | `bt_navigator` | **示教路径跟踪** |
| `/follow_path` | `nav2_msgs/action/FollowPath` | `controller_server` | 行为树 `FollowPath` 节点 |
| `/compute_path_to_pose` | `nav2_msgs/action/ComputePathToPose` | `planner_server` | 行为树 `ComputePathToPose` |
| `/compute_path_through_poses` | `nav2_msgs/action/ComputePathThroughPoses` | `planner_server` | 多点规划 |
| `/smooth_path` | `nav2_msgs/action/SmoothPath` | `smoother_server` | 路径平滑（当前 BT 未调用） |
| `/follow_waypoints` | `nav2_msgs/action/FollowWaypoints` | `waypoint_follower` | 航点任务 |
| `/spin` | `nav2_msgs/action/Spin` | `behavior_server` | 原地旋转恢复 |
| `/backup` | `nav2_msgs/action/BackUp` | `behavior_server` | 后退恢复 |
| `/wait` | `nav2_msgs/action/Wait` | `behavior_server` | 等待恢复 |
| `/drive_on_heading` | `nav2_msgs/action/DriveOnHeading` | `behavior_server` | 定向前进 |

> **注意**：当前 `yhs_nav2.yaml` 使用旧键 `recoveries_server` / `recovery_plugins` / `nav2_recoveries/*`，Nav2 1.1.5 的 `behavior_server` 读取 `behavior_plugins`。因此这四个动作仍会以**默认插件列表**创建，但 YAML 中的恢复参数整段不生效（见 [10 K4](10-known-issues.md)）。

### 3.1 示教路径动作字段

`yhs_can_interfaces/action/RecordPath.action`：

```text
# Goal
nav_msgs/Path path
string behavior_tree
---
# Result
std_msgs/Empty result
---
# Feedback
geometry_msgs/PoseStamped current_pose
builtin_interfaces/Duration navigation_time
builtin_interfaces/Duration estimated_time_remaining
int16 number_of_recoveries
float32 distance_remaining
```

处理链：

```
/record_path (nav_msgs/Path)
   └─► NavigateFollowRecordPathNavigator::onGoalPoseReceived()
        └─► 自调用 /navigate_follow_record_path
             └─► 加载 BT（navigate_follow_record_path_w_replanning_and_recovery.xml）
                  └─► FollowPath → controller_server
```

---

## 4. TF 树

```
map
 └── odom                        发布者：AMCL（导航） / Cartographer（建图）
      └── base_link              发布者：yhs_can_control（navigation 路线）
                                 或 Cartographer（mapping 路线，use_tf=false）
           ├── laser_link        静态：0 0 0.33 0 0 0 1（nvilidar.launch.py）
           ├── imu_link          静态：0 0 0 0 0 0 1（imu_spec_msg.launch.py）
           └── camera_link       静态：0.23 0 0.2（pointcloud_to_laserscan.launch.py）
```

| 变换 | 权威来源 | 备注 |
| --- | --- | --- |
| `map→odom` | AMCL / Cartographer | **同一时刻只能有一个发布者** |
| `odom→base_link` | `yhs_can_control` 或 Cartographer | 由 `use_tf` 参数切换 |
| `base_link→laser_link` | 静态 TF | 必须与雷达实际安装高度一致 |
| `base_link→imu_link` | 静态 TF | 旋转全 0，安装偏差需自行修改 |
| `base_link→camera_link` | 静态 TF | 仅在启用深度相机时存在 |

---

## 5. 消息/服务/动作定义位置

| 包 | 目录 | 内容 |
| --- | --- | --- |
| `yhs_can_interfaces` | `msg/` | `CtrlCmd`、`CtrlFb`、`IoFb`、`LWheelFb`、`RWheelFb`、`BmsFb`、`BmsFlagFb`、`ChassisInfoFb`、`FreeCtrlCmd`、`IoCmd` |
| `yhs_can_interfaces` | `action/` | `RecordPath.action` |
| `nav2_msgs` | `action/`、`srv/` | 13 个动作、8 个服务 |
| `cartographer_ros_msgs` | `msg/`、`srv/` | 12 个消息、7 个服务 |

常用查看命令：

```bash
ros2 interface show yhs_can_interfaces/msg/ChassisInfoFb
ros2 interface show yhs_can_interfaces/action/RecordPath
ros2 interface list | grep yhs_can
ros2 topic info /scan --verbose
ros2 service list -t | grep -E "map|amcl|costmap"
ros2 action list -t
```

---

## 6. 接口排查清单

| 现象 | 优先检查 |
| --- | --- |
| 收不到 `/scan` | 串口设备、`frame_id`、QoS（best effort） |
| 收不到 `/odom` | CAN 是否收到 `0x98C4D1EF`、CRC 是否通过、`use_tf` 是否与路线匹配 |
| `/cmd_vel` 有但车不动 | CAN 发送是否成功、`ctrl_cmd` 齿轮/使能、`/chassis_info_fb` 反馈 |
| `map→odom` 不存在 | AMCL 是否激活、`set_initial_pose` 与 `/initialpose` |
| 动作服务器不存在 | 节点是否 active、插件是否加载、旧参数键（K4） |
| `/map` 不更新 | `map_server` 是否加载成功、YAML 指向的 PGM 是否存在（K1） |

---

[← 上一章：已知问题](10-known-issues.md) · [返回索引](README.md) · [下一章：代码地图 →](12-code-map.md)
