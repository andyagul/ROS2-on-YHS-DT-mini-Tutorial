# 01 · 整体架构

[← 返回索引](README.md)

---

## 1. 设计定位

本工作空间是一套**"单线激光 + 差速底盘 + 2D 栅格地图"的经典 ROS 2 导航栈**，核心思想是：

1. **底盘只做执行器与里程计**：`yhs_can_control` 把 `/cmd_vel` 转成 CAN 帧，把 CAN 反馈转成 `/odom` + `odom→base_link` TF。
2. **传感器独立成节点**：雷达、IMU、相机各自发布标准 ROS 消息，通过**静态 TF** 挂到 `base_link`。
3. **建图与导航二选一**：建图用 Cartographer/GMapping 产出 `/map`；导航时用 `map_server` 加载地图 + AMCL 定位。
4. **编排层集中管理**：`yhs_nav2` 不写算法，只负责 launch 组合、参数、地图与初始位姿脚本。

---

## 2. 五层架构

| 层 | 目录 | 职责 | 代表包 |
| --- | --- | --- | --- |
| L1 硬件/驱动 | `hardware/*`、`yhs_can_control` | 设备 I/O、协议解析、原始数据发布 | `nvilidar_ros2`、`serial_imu`、`orbbec_camera`、`yhs_can_control` |
| L2 接口定义 | `yhs_can_interfaces`、`cartographer_ros_msgs`、`nav2_msgs`、`costmap_converter_msgs` | msg / srv / action 契约 | `yhs_can_interfaces` |
| L3 SLAM | `cartographer`、`cartographer_ros`、`slam_gmapping` | 前端扫描匹配、子图、回环、位姿图优化、栅格地图 | `cartographer_ros` |
| L4 导航 | `navigation2`、`teb_local_planner`、`dependence/*` | 定位、全局规划、局部控制、恢复行为、行为树 | `nav2_*`、`teb_local_planner` |
| L5 编排/教学 | `yhs_nav2`、`yhs_tutorials`、`tools`、`yhs_chassis_description` | 启动组合、参数、地图、模型、示例 | `yhs_nav2` |

```
┌─────────────────────────────────────────────────────────────┐
│ L5  yhs_nav2 (launch / param / map / scripts)               │
├─────────────────────────────────────────────────────────────┤
│ L4  navigation2: amcl / planner / controller / bt_navigator │
│     teb_local_planner · costmap_converter · BehaviorTree.CPP│
├─────────────────────────────────────────────────────────────┤
│ L3  cartographer_ros ── cartographer 核心库                 │
│     slam_gmapping ── openslam_gmapping                      │
├─────────────────────────────────────────────────────────────┤
│ L2  yhs_can_interfaces · nav2_msgs · cartographer_ros_msgs  │
├─────────────────────────────────────────────────────────────┤
│ L1  nvilidar_ros2 · serial_imu · orbbec_camera · CAN 底盘    │
└─────────────────────────────────────────────────────────────┘
                            ▲
                            │ /dev/lidar · /dev/imu · can0 · USB
                            ▼
                        实车硬件
```

---

## 3. 两条运行主线

### 3.1 建图（Cartographer 主方案）

入口：[cartographer.launch.py](../../../src/yhs_nav2/launch/cartographer.launch.py)

```
serial_imu ─► /imu_data ─┐
nvilidar   ─► /scan ─────┤
yhs_can_control ─► /odom ┤
                         ▼
              cartographer_node (dgt_mini.lua)
                 ├─► /submap_list
                 ├─► /trajectory_node_list
                 ├─► /landmark_poses_list
                 ├─► TF: map→odom（provide_odom_frame=true）
                 └─► map_builder 服务（submap_query / write_state / finish_trajectory …）
                         │
        cartographer_occupancy_grid_node ─► /map (nav_msgs/OccupancyGrid)
```

关键取舍：`yhs_nav2/cartographer.launch.py` 默认 **`use_tf:=false`**，即底盘节点不发布 `odom→base_link`，避免与 Cartographer 的 `odom` 帧重复；此时 TF 由 Cartographer 提供（`tracking_frame=base_link`、`published_frame=base_link`、`provide_odom_frame=true`）。

### 3.2 导航（Nav2）

入口：[yhs_nav2.launch.py](../../../src/yhs_nav2/launch/yhs_nav2.launch.py)

```
serial_imu ─► /imu_data
nvilidar   ─► /scan
yhs_can_control：use_tf:=true ─► /odom + TF odom→base_link
                     │
map_server ─► /map ──► AMCL（/scan + TF）─► TF map→odom + /amcl_pose
                     │
bt_navigator ─► planner_server (NavFn) ─► /plan
                     │
              controller_server (TEB) ─► /cmd_vel_nav
                     │
              velocity_smoother ─► /cmd_vel
                     │
              yhs_can_control ─► CAN 帧
```

导航入口内部再拆成两个子 launch：

- [localization_launch.py](../../../src/yhs_nav2/launch/localization_launch.py)：`map_server` + `amcl` + `lifecycle_manager_localization`
- [navigation_launch.py](../../../src/yhs_nav2/launch/navigation_launch.py)：`controller_server`、`smoother_server`、`planner_server`、`behavior_server`、`bt_navigator`、`waypoint_follower`、`velocity_smoother` + `lifecycle_manager_navigation`

---

## 4. 话题 / 服务 / 动作总表

### 4.1 关键话题

| 话题 | 类型 | 发布者 | 订阅者 | 备注 |
| --- | --- | --- | --- | --- |
| `/scan` | `sensor_msgs/LaserScan` | `nvilidar_ros2_node` | Cartographer / AMCL / costmap | QoS SensorData（best effort） |
| `/scan1` | `sensor_msgs/LaserScan` | `pointcloud_to_laserscan` | 局部/全局 costmap | 深度相机投影，选配 |
| `/imu_data` | `sensor_msgs/Imu` | `serial_imu_node` | Cartographer（可选）/ 底盘节点 | 底盘用其 yaw 代替积分航向 |
| `/odom` | `nav_msgs/Odometry` | `yhs_can_control_node` | Cartographer / Nav2 | 由 CAN 反馈计算 |
| `/cmd_vel` | `geometry_msgs/Twist` | `velocity_smoother`（导航）/ 教学节点 | `yhs_can_control_node` | 最终执行指令 |
| `/cmd_vel_nav` | `geometry_msgs/Twist` | `controller_server` | `velocity_smoother` | 内部链路 |
| `/chassis_info_fb` | `yhs_can_interfaces/ChassisInfoFb` | `yhs_can_control_node` | 上层监控 | 含转角/IO/BMS/轮速 |
| `/io_cmd` | `yhs_can_interfaces/IoCmd` | 上层 | `yhs_can_control_node` | 灯光/门锁/喇叭 |
| `/plan` | `nav_msgs/Path` | `planner_server` | RViz / BT | 全局路径 |
| `/amcl_pose` | `geometry_msgs/PoseWithCovarianceStamped` | `amcl` | RViz / 上层 | 定位结果 |
| `/initialpose` | `geometry_msgs/PoseWithCovarianceStamped` | RViz / `set_init_amcl_pose.py` | `amcl` | 初始位姿 |
| `/map` | `nav_msgs/OccupancyGrid` | `map_server` 或 Cartographer | AMCL / costmap | 静态地图 |
| `/tf`、`/tf_static` | `tf2_msgs/TFMessage` | 各节点 | 全体 | 坐标系 |

### 4.2 底盘指令消息

| 消息 | 用途 | 关键字段 |
| --- | --- | --- |
| `yhs_can_interfaces/CtrlCmd` | 线速度 + 角速度控制 | `ctrl_cmd_gear`、`ctrl_cmd_linear`、`ctrl_cmd_angular` |
| `yhs_can_interfaces/FreeCtrlCmd` | 左右轮独立速度 | `ctrl_cmd_gear`、`free_ctrl_cmd_velocity_l/r` |
| `yhs_can_interfaces/IoCmd` | 灯光与 IO | 9 个 bool/uint8 字段 |

### 4.3 动作

| 动作 | 类型 | 说明 |
| --- | --- | --- |
| `navigate_to_pose` | `nav2_msgs/NavigateToPose` | 点到点导航 |
| `navigate_through_poses` | `nav2_msgs/NavigateThroughPoses` | 多点导航 |
| `follow_path` | `nav2_msgs/FollowPath` | 控制器执行路径 |
| `compute_path_to_pose` | `nav2_msgs/ComputePathToPose` | 全局规划 |
| `spin` / `backup` / `wait` / `drive_on_heading` | `nav2_msgs/*` | 恢复行为 |
| `navigate_follow_record_path` | `yhs_can_interfaces/RecordPath` | **自研示教循迹**（录制路径→复现） |

### 4.4 Cartographer 服务

`submap_query`、`trajectory_query`、`start_trajectory`、`finish_trajectory`、`write_state`、`get_trajectory_states`、`read_metrics`（见 [node.cpp](../../../src/cartographer_ros/cartographer_ros/src/node.cpp)）。

---

## 5. TF 坐标树

```
map
 └── odom                         （AMCL 或 Cartographer 发布）
      └── base_link               （底盘 use_tf=true 时发布；Cartographer 建图时由它提供）
           ├── laser_link         （static: 0, 0, 0.33, rpy=0）
           ├── imu_link           （static: 全部为 0）
           └── camera_link        （static: 0.23, 0, 0.20, rpy=0，仅相机/点云转激光路线）
```

要点：

- `yhs_chassis_description` 的 URDF **只有 `base_link` 一个 link**（无关节、无轮子），`robot_state_publisher` 不会生成底盘内部 TF；轮子只是 STL 视觉网格的一部分。
- 静态 TF 由各驱动 launch 分别发布：雷达 `nvilidar.launch.py`、IMU `imu_spec_msg.launch.py`、相机 `pointcloud_to_laserscan.launch.py`。
- **`map→odom` 只有一个发布者**：建图时是 Cartographer/GMapping，导航时是 AMCL。这是排查"定位漂移 / TF 重复"的第一检查点。
- **`use_tf` 开关**：`yhs_can_control` 的 `use_tf`（默认 `true`）决定是否发布 `odom→base_link`。Cartographer 路线必须传 `false`。

---

## 6. 生命周期与启动顺序

Nav2 节点均为 `nav2_util::LifecycleNode`，由 `nav2_lifecycle_manager` 依序 `configure → activate`：

```
map_server ─┐
amcl ───────┴─ lifecycle_manager_localization
controller_server ─┐
planner_server     │
behavior_server    ├─ lifecycle_manager_navigation
bt_navigator       │
waypoint_follower  │
velocity_smoother ─┘
```

`autostart:=true`（默认）时由 `lifecycle_manager` 自动完成状态迁移。任一节点 configure 失败，后续节点不会被激活，表现为"RViz 有地图但车不动"。

---

## 7. 架构层面的关键事实

1. **无仿真依赖**：所有 launch 均面向实车；`block_spawner.py` 依赖 Gazebo 的 `tb3_worlds`，在本工作空间不可直接使用。
2. **组合模式未真正启用**：`yhs_nav2.launch.py` 会启动 `nav2_container`，但 `localization_launch.py` / `navigation_launch.py` 用的是独立 `Node`，没有 `LoadComposableNodes`，因此 `use_composition` 实际只多起一个空容器（见 [10 文档](10-known-issues.md)）。
3. **参数以 `yhs_nav2.yaml` 为单点**：AMCL、BT、控制器、代价地图、规划器、恢复行为全部写在一个文件里；子 launch 默认路径却指向 `params/nav2_params.yaml`（不存在），因此**必须从顶层 launch 进入**，或显式传 `params_file`。
4. **数学链路**：`cmd_vel.angular.z(rad/s)` → `×180/π` 转 `deg/s` → CAN 帧按 `×100` 编码为 16 位；反馈再 `/100` 得 `deg/s`，`/180×π` 回 `rad/s`。
