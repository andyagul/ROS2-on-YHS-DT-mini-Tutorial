# ROS2 单线激光导航小车工作空间（src）架构分析

> 分析对象：`/Users/waiyu/Downloads/src`
> 项目来源：YHS（深圳煜禾森科技有限公司）OS-lite-ros2-navigation —— 基于 ROS2 的单线激光 SLAM 导航套件
> 版本基线：ROS 2 **Humble**，Nav2 **1.1.5**，TEB Local Planner **0.9.1**，Cartographer **2.0**，Orbbec SDK ROS2 **1.5.11**，Ubuntu 22.04
> 硬件平台：煜禾森（YHS）差速 CAN 底盘（DT-mini / DT-mid / FR-mid / FR-mini / FW-mid / MK-mid / TK-mid 多车型）+ 单线激光雷达（nvilidar）+ 串口 IMU（CH100 系列）+ 可选 Orbbec 深度相机

---

## 0. 系统总览

`src` 目录是一套**“感知 → 建图/定位 → 规划 → 控制 → 底盘执行”**完整闭环的 ROS 2 机器人软件栈，按功能可划分为六大类：

| 类别 | 目录 / 包 | 职责 |
|---|---|---|
| 自定义接口 | `yhs_can_interfaces` | 煜禾森（YHS）底盘 CAN 通信所需的 msg / action 定义 |
| 底盘驱动 | `yhs_can_control` | CAN 收发、速度指令下发、里程计与 TF 发布 |
| 机器人模型 | `yhs_chassis_description` | 多车型 URDF / STL 与可视化 launch |
| 顶层编排 | `yhs_nav2` | 总启动 launch、参数中心、地图、辅助脚本 |
| 教学示例 | `yhs_tutorials`、`tools` | 订阅/发布/控制的最小示例与工具脚本 |
| 传感器驱动 | `hardware/`（nvilidar_ros2、serial_imu、OrbbecSDK_ROS2） | 激光、IMU、深度相机数据接入 |
| 建图 | `cartographer(_ros)`、`slam_gmapping` | 2D SLAM（Cartographer 为主，Gmapping 备用） |
| 导航 | `navigation2/`、`teb_local_planner`、`pointcloud_to_laserscan` | Nav2 导航栈 + TEB 局部规划 + 点云转激光 |
| 依赖库 | `dependence/`（BehaviorTree.CPP、costmap_converter） | 行为树引擎、代价地图多边形转换 |

**核心链路（导航模式）**：

```
激光/scan ──► AMCL（map→odom TF）
             ──► global_costmap ──► NavFn 全局规划 /plan
             ──► local_costmap  ──► TEB 局部控制 ──► cmd_vel
底盘 yhs_can_control ──(CAN)──► 电机；回传 /odom + odom→base_link TF（闭环）
```

---

## 1. 逐包 / 逐文件功能详解

### 1.1 `yhs_can_interfaces` —— 底盘自定义接口（rosidl 接口包）

纯接口定义包，无节点，是 `yhs_can_control` 的编译依赖与数据契约。

| 文件 | 类型 | 功能 |
|---|---|---|
| [msg/CtrlCmd.msg](file:///Users/waiyu/Downloads/src/yhs_can_interfaces/msg/CtrlCmd.msg) | msg | 底盘控制指令：`ctrl_cmd_gear`（档位）、`ctrl_cmd_linear`（线速度 m/s）、`ctrl_cmd_angular`（角速度，驱动内部按“度”解析） |
| [msg/FreeCtrlCmd.msg](file:///Users/waiyu/Downloads/src/yhs_can_interfaces/msg/FreeCtrlCmd.msg) | msg | 自由控制：左右轮独立速度（`velocity_l` / `velocity_r`）+ 档位 |
| [msg/IoCmd.msg](file:///Users/waiyu/Downloads/src/yhs_can_interfaces/msg/IoCmd.msg) | msg | IO 控制：灯（远近光/转向/制动/雾灯）、解锁、喇叭 |
| [msg/CtrlFb.msg](file:///Users/waiyu/Downloads/src/yhs_can_interfaces/msg/CtrlFb.msg) | msg | 控制反馈：目标档位、实际线/角速度 |
| [msg/LWheelFb.msg](file:///Users/waiyu/Downloads/src/yhs_can_interfaces/msg/LWheelFb.msg) / [msg/RWheelFb.msg](file:///Users/waiyu/Downloads/src/yhs_can_interfaces/msg/RWheelFb.msg) | msg | 左右轮速度（m/s）与脉冲计数 |
| [msg/IoFb.msg](file:///Users/waiyu/Downloads/src/yhs_can_interfaces/msg/IoFb.msg) | msg | IO 状态反馈：各灯状态、6 路碰撞传感器、6 路跌落传感器、急停（estop）、手柄控制、充电状态 |
| [msg/BmsFb.msg](file:///Users/waiyu/Downloads/src/yhs_can_interfaces/msg/BmsFb.msg) / [msg/BmsFlagFb.msg](file:///Users/waiyu/Downloads/src/yhs_can_interfaces/msg/BmsFlagFb.msg) | msg | 电池管理系统反馈：电压/电流/剩余容量/SOC、过压欠压过流过温等标志、高低温度 |
| [msg/ChassisInfoFb.msg](file:///Users/waiyu/Downloads/src/yhs_can_interfaces/msg/ChassisInfoFb.msg) | msg | 聚合反馈：`ctrl_fb`+`io_fb`+`l/r_wheel_fb`+`bms_fb`+`bms_flag_fb` |
| [action/RecordPath.action](file:///Users/waiyu/Downloads/src/yhs_can_interfaces/action/RecordPath.action) | action | **记录路径导航动作**：Goal 为 `nav_msgs/Path` + `behavior_tree`；Feedback 为当前位姿、导航时长、剩余时间、恢复次数、剩余距离；Result 为 `Empty` |

**系统角色**：整个底盘通信的“协议层”，隔离 CAN 底层字节流与上层 ROS 语义。

---

### 1.2 `yhs_can_control` —— 底盘 CAN 驱动（核心自定义节点）

| 文件 | 功能 |
|---|---|
| [src/yhs_can_control_node.cpp](file:///Users/waiyu/Downloads/src/yhs_can_control/src/yhs_can_control_node.cpp) | 驱动主实现（513 行），详见下 |
| [include/yhs_can_control/yhs_can_control_node.hpp](file:///Users/waiyu/Downloads/src/yhs_can_control/include/yhs_can_control/yhs_can_control_node.hpp) | `CanControl` 类声明，含 `READ_PARAM` 参数读取宏 |
| [launch/yhs_can_control.launch.py](file:///Users/waiyu/Downloads/src/yhs_can_control/launch/yhs_can_control.launch.py) | 启动节点，参数 `params_file_ch`（默认 cfg.yaml）与 `use_tf`（默认 true） |
| [params/cfg.yaml](file:///Users/waiyu/Downloads/src/yhs_can_control/params/cfg.yaml) | 仅一项：`can_name: can0` |

`CanControl` 类的接口与职责：

- **订阅**（下行控制）：
  - `cmd_vel`（`Twist`）→ [cmd_callback](file:///Users/waiyu/Downloads/src/yhs_can_control/src/yhs_can_control_node.cpp#L153-L161)：把 `linear.x` 当作线速度、`angular.z` 转成“度”，档位固定 `3`，转成 `CtrlCmd` 再走 `ctrl_cmd_callback`。
  - `ctrl_cmd`（`CtrlCmd`）、`free_ctrl_cmd`（`FreeCtrlCmd`）、`io_cmd`（`IoCmd`）：直接组包发送。
  - `imu_data`（`Imu`）→ [imu_data_callback](file:///Users/waiyu/Downloads/src/yhs_can_control/src/yhs_can_control_node.cpp#L388-L396)：提取 IMU 偏航角 `imu_yaw_`，供里程计朝向使用。
- **发布**：`chassis_info_fb`（`ChassisInfoFb`）、`odom`（`Odometry`）。
- **TF**：`odom → base_link`（当 `use_tf=true`）。
- **CAN 底层**：[run()](file:///Users/waiyu/Downloads/src/yhs_can_control/src/yhs_can_control_node.cpp#L472-L500) 打开 `PF_CAN` 原始套接字绑定 `can0`，启动 [can_data_recv_callback](file:///Users/waiyu/Downloads/src/yhs_can_control/src/yhs_can_control_node.cpp#L232-L386) 接收线程，10ms 超时轮询。
- **下行 CAN ID**：`0x98C4D1D0`（控制）、`0x98C4D2D0`（自由控制）、`0x98C4D7D0`（IO）。
- **上行 CAN ID**：`0x98C4D1EF`（CtrlFb）、`0x98C4D7EF`（左轮）、`0x98C4D8EF`（右轮）、`0x98C4DAEF`（IoFb）、`0x98C4E1EF`（BmsFb）、`0x98C4E2EF`（BmsFlagFb）。每帧校验异或 CRC（data[7]）。
- **里程计**：[publish_odom](file:///Users/waiyu/Downloads/src/yhs_can_control/src/yhs_can_control_node.cpp#L398-L465)：对线速度积分得 `x/y`；朝向优先取 IMU 偏航（若 `/imu_data` 有发布者），否则对角速度积分。发布 `/odom`（`odom→base_link`）。

**系统角色**：唯一的“执行层”，把导航输出的 `cmd_vel` 翻译成 CAN 报文驱动电机，同时向上反馈里程计/位姿，构成运动闭环。

---

### 1.3 `yhs_chassis_description` —— 机器人模型

| 文件 | 功能 |
|---|---|
| [urdf/robot.urdf](file:///Users/waiyu/Downloads/src/yhs_chassis_description/urdf/robot.urdf)、[urdf/DT-mini.urdf](file:///Users/waiyu/Downloads/src/yhs_chassis_description/urdf/DT-mini.urdf) 及 `DT-mid/FR_mid/FR_mini/FW-mid/MK-mid/TK-mid.urdf` | 各车型 URDF，当前仅含 `base_link` 的视觉（STL 网格），用于 RViz 显示 |
| `meshes/*.STL` | 各车型外壳/轮子网格模型 |
| [launch/robot.launch.py](file:///Users/waiyu/Downloads/src/yhs_chassis_description/launch/robot.launch.py) | 启动 `robot_state_publisher` + `joint_state_publisher` + `rviz2`；默认加载 `DT-mini.urdf` |
| `launch/DT-mid.launch.py` 等 7 个车型 launch | 各车型对应的加载入口（配合 `tools/select_car.sh` 拷贝为 `robot.launch.py`） |
| `rviz/rviz.rviz` | RViz 显示配置 |

**系统角色**：提供机器人外观模型与 `base_link` 静态 TF 基准，供 RViz 可视化。

> 注意：[robot.urdf](file:///Users/waiyu/Downloads/src/yhs_chassis_description/urdf/robot.urdf) 引用的 `mesh/autocube.STL` 与实际 `meshes/DT-mini.STL` 等不一致，属遗留路径；实际以 `DT-mini.urdf` 等车型文件为准。

---

### 1.4 `yhs_nav2` —— 顶层编排 + 参数中心 + 地图 + 脚本

#### launch 文件

| 文件 | 功能 |
|---|---|
| [launch/yhs_nav2.launch.py](file:///Users/waiyu/Downloads/src/yhs_nav2/launch/yhs_nav2.launch.py) | **总入口（导航模式）**：声明 namespace/map/use_sim_time/params_file/autostart/use_composition 等参数 → 启动 `nav2_container`（组合容器）→ 依次 include：`serial_imu` → `nvilidar` → `yhs_can_control` → `localization_launch` → `navigation_launch`。默认地图 `map/ecarroom.yaml`。 |
| [launch/cartographer.launch.py](file:///Users/waiyu/Downloads/src/yhs_nav2/launch/cartographer.launch.py) | **Cartographer 建图入口**：IMU + 雷达 + 底盘 + `cartographer_ros/dgt_mini.launch.py` |
| [launch/gmapping.launch.py](file:///Users/waiyu/Downloads/src/yhs_nav2/launch/gmapping.launch.py) | **Gmapping 建图入口**：IMU + 雷达 + 底盘 + `slam_gmapping.launch.py` |
| [launch/localization_launch.py](file:///Users/waiyu/Downloads/src/yhs_nav2/launch/localization_launch.py) | 定位：`map_server` + `amcl` + `lifecycle_manager_localization`（管理 `[map_server, amcl]` 生命周期） |
| [launch/navigation_launch.py](file:///Users/waiyu/Downloads/src/yhs_nav2/launch/navigation_launch.py) | 导航：`controller_server` + `smoother_server` + `planner_server` + `behavior_server` + `bt_navigator` + `waypoint_follower` + `velocity_smoother` + `lifecycle_manager_navigation`。注意两个关键重映射：`controller_server` 的 `cmd_vel→cmd_vel_nav`，`velocity_smoother` 的 `cmd_vel→cmd_vel_nav`、`cmd_vel_smoothed→cmd_vel` |

#### 参数中心

[param/yhs_nav2.yaml](file:///Users/waiyu/Downloads/src/yhs_nav2/param/yhs_nav2.yaml) —— 全系统参数（详见第 5 节），要点：

- `amcl`：`likelihood_field` 模型、48 束、500~5000 粒子、`base_link`/`map`/`odom` 坐标系、`set_initial_pose: true`（默认原点初始化）。
- `bt_navigator`：默认 BT `navigate_w_replanning_and_recovery.xml`，加载 33 个 BT 插件，含自定义 **`nav2_compute_follow_record_path_action_bt_node`**。
- `controller_server`：20 Hz，`FollowPath` 插件 = **`teb_local_planner::TebLocalPlannerROS`**（TEB，带 `costmap_converter::CostmapToPolygonsDBSMCCH` 多边形转换）。
- `local_costmap`/`global_costmap`：`base_link` 系、多边形 footprint、`scan`+`scan1` 双观测源。
- `planner_server`：`GridBased` = `nav2_navfn_planner/NavfnPlanner`。
- `recoveries_server`：`spin`/`backup`/`wait` 恢复行为。
- `velocity_smoother`、`waypoint_follower`（`wait_at_waypoint` 200ms 停靠）。

#### map 与脚本

| 文件 | 功能 |
|---|---|
| `map/*.pgm + *.yaml`（ecarroom、110、113、116、75、hj110、qg2108、tw、zgq-c 等） | 预建好的栅格地图（多场地），`ecarroom.yaml` 为默认 |
| [scripts/set_init_amcl_pose.py](file:///Users/waiyu/Downloads/src/yhs_nav2/scripts/set_init_amcl_pose.py) | 向 `/initialpose` 发布 AMCL 初始位姿（x/y/theta/cov 参数），等待订阅者出现后发送 |
| [scripts/block_spawner.py](file:///Users/waiyu/Downloads/src/yhs_nav2/scripts/block_spawner.py) | Gazebo 仿真辅助：随机在指定位置 spawn 红/蓝/绿方块（依赖 tb3_worlds，属仿真遗留） |

**系统角色**：`yhs_nav2` 是“一键启动”的总控包，把硬件层、底盘层、导航层串起来，并提供参数与地图。

---

### 1.5 `hardware/` —— 传感器驱动

#### 1.5.1 `nvilidar_ros2`（单线激光雷达，v1.0.0）

| 文件 | 功能 |
|---|---|
| [launch/nvilidar.launch.py](file:///Users/waiyu/Downloads/src/hardware/nvilidar_ros2/launch/nvilidar.launch.py) | 启动 `nvilidar_ros2_node` + 静态 TF `base_link→laser_link`（`z=0.33 m`） |
| [params/nvilidar.yaml](file:///Users/waiyu/Downloads/src/hardware/nvilidar_ros2/params/nvilidar.yaml) | 串口 `/dev/lidar`、波特率 512000、`frame_id: laser_link`、`angle ±180°`、量程 0.001~64m、滤波（滑窗/拖尾）等 |
| `src/nvilidar_ros2_node.cpp`、`src/nvilidar_ros2_client.cpp` | 驱动主节点（发布 `/scan`，`SensorDataQoS`）与调试订阅端 |
| `sdk/`（厂商协议库）、`startup/initenv.sh` | 底层串口/UDP 协议实现与 udev 规则脚本 |

**系统角色**：唯一 2D 激光源，`/scan` 同时供建图（Cartographer/Gmapping）、定位（AMCL）、代价地图三层使用。

#### 1.5.2 `serial_imu`（串口 IMU）

| 文件 | 功能 |
|---|---|
| [launch/imu_spec_msg.launch.py](file:///Users/waiyu/Downloads/src/hardware/serial_imu/launch/imu_spec_msg.launch.py) | 启动 `serial_imu` + 静态 TF `base_link→imu_link`（`imu_frame_id` 默认 `imu_link`） |
| `src/serial_port.cpp` | 主节点：打开 [IMU_SERIAL = "/dev/imu"](file:///Users/waiyu/Downloads/src/hardware/serial_imu/src/serial_port.cpp#L17)（B115200），解帧后发布 `/imu_data`（四元数、角速度 °/s→rad/s、加速度 g→m/s²），`frame_id=imu_link` |
| `src/ch_serial.c/.h` | 超核（CH100/CH110）二进制协议解析器（帧头 0x90 / 加速度 0xA0 / 角速度 0xB0 / 四元数 0xD1 等） |
| `src/sub_spec.cpp` | 调试订阅示例 |

**系统角色**：提供 IMU 姿态，底盘驱动用它校准里程计朝向；Cartographer 当前 `use_imu_data=false` 未融合（预留）。

#### 1.5.3 `OrbbecSDK_ROS2`（深度相机，v1.5.11，可选）

`orbbec_camera`（彩色/深度/IR/点云/IMU 采集与发布）、`orbbec_camera_msgs`（接口）、`orbbec_description`（相机 URDF）三包。默认在 `yhs_nav2.launch.py` 中**注释掉**，按需启用（对应 `astra.launch.py` 等）。

**系统角色**：可选 RGB-D 感知，点云可经 `pointcloud_to_laserscan` 转为 `/scan1` 作为第二激光源。

---

### 1.6 `cartographer` + `cartographer_ros` —— 2D SLAM（主建图方案）

- `cartographer/`：SLAM 核心算法库（`mapping/` 子图与位姿图优化、`sensor/` 数据抽象、`io/` pbstream 序列化等）。
- `cartographer_ros/`：ROS 2 封装，含 `cartographer_ros`、`cartographer_ros_msgs`、`cartographer_rviz` 三包。

关键文件：

| 文件 | 功能 |
|---|---|
| [launch/dgt_mini.launch.py](file:///Users/waiyu/Downloads/src/cartographer_ros/cartographer_ros/launch/dgt_mini.launch.py) | 启动 `cartographer_node`（配置 `dgt_mini.lua`，重映射 `imu→imu_data`、`odom→odom`）+ `cartographer_occupancy_grid_node`（分辨率 0.05） |
| [configuration_files/dgt_mini.lua](file:///Users/waiyu/Downloads/src/cartographer_ros/cartographer_ros/configuration_files/dgt_mini.lua) | 2D 建图参数：`map`/`base_link`/`odom` 坐标系、`provide_odom_frame=true`、`use_odometry=true`、`use_imu_data=false`、每 60 帧一子图、每 5 节点优化、运动滤波 0.2m/0.1rad、`min_range 0.5 / max_range 50` |
| `src/node_main.cpp` + `src/node.cpp` | `cartographer_node`：订阅 scan/imu/odom，发布 submap/tracked_pose，提供轨迹与状态服务 |
| `src/occupancy_grid_node_main.cpp` | 订阅子图、拼接发布 `/map`（注意该节点 `use_sim_time` 硬编码 `True`，见第 5 节风险） |

**系统角色**：在线建图，输出 `/map` 及 `map→odom→base_link` 坐标链；建好的图交给 Nav2 导航。

---

### 1.7 `slam_gmapping` —— 备选建图方案

[slam_gmapping.launch.py](file:///Users/waiyu/Downloads/src/slam_gmapping/slam_gmapping/launch/slam_gmapping.launch.py) 启动 `slam_gmapping` 节点（依赖 `openslam_gmapping`）。粒子滤波 SLAM，通过 `gmapping.launch.py` 使用，作为 Cartographer 之外的轻量备选。

---

### 1.8 `navigation2/` —— Nav2 导航栈（1.1.5 完整源码）

包含 30+ 子包，此处按职责归类（均为 Nav2 官方实现 + 少量定制）：

| 类别 | 子包 | 关键文件/作用 |
|---|---|---|
| 接口 | `nav2_core`、`nav2_msgs` | 插件纯虚基类；`NavigateToPose`/`FollowPath`/`ComputePathToPose`/`Spin`/`BackUp`/`Wait`/`SmoothPath` 等 action、`Costmap`/`ParticleCloud` 等 msg、`LoadMap/SaveMap/ClearCostmap` 等 srv |
| 基础设施 | `nav2_common`、`nav2_lifecycle_manager`、`nav2_util`、`nav2_voxel_grid` | launch 期 YAML 改写（`RewrittenYaml`）；生命周期统一管理 + bond 心跳；工具与体素栅格 |
| 定位/地图 | `nav2_amcl`、`nav2_map_server` | 粒子滤波定位（发布 `map→odom` TF）；加载 `.yaml/.pgm` 地图并 latched 发布 `/map` |
| 代价地图 | `nav2_costmap_2d` | `static/obstacle/voxel/inflation` 图层 + `keepout/speed` 过滤器 |
| 全局规划 | `nav2_planner`、`nav2_navfn_planner`、`nav2_smac_planner`、`nav2_theta_star_planner` | 规划服务器与多种全局规划插件（本车用 NavFn） |
| 局部控制 | `nav2_controller`、`nav2_dwb_controller`、`nav2_regulated_pure_pursuit_controller`、`nav2_rotation_shim_controller` | 控制器服务器与可插拔局部规划（本车改用外部 TEB） |
| 决策/行为 | `nav2_behavior_tree`、`nav2_bt_navigator`、`nav2_behaviors` | 行为树节点封装、导航入口 Navigator、恢复行为（Spin/BackUp/Wait） |
| 平滑/跟随 | `nav2_smoother`、`nav2_constrained_smoother`、`nav2_waypoint_follower`、`nav2_velocity_smoother` | 路径平滑、航点跟随、速度平滑限幅 |
| 安全 | `nav2_collision_monitor` | 激光/点云安全区检测（本工程未在 launch 中启用） |
| 工具 | `nav2_rviz_plugins`、`nav2_simple_commander`、`nav2_system_tests` | RViz 面板、Python 高层 API、测试 |

#### 定制：记录路径导航（Record Path）

这是本工程对 Nav2 的核心定制，配合 `yhs_can_interfaces/action/RecordPath`：

| 文件 | 功能 |
|---|---|
| [navigation2/nav2_behavior_tree/plugins/action/navigate_follow_record_path_action.cpp](file:///Users/waiyu/Downloads/src/navigation2/nav2_behavior_tree/plugins/action/navigate_follow_record_path_action.cpp) | 自定义 BT 动作节点，注册名 `navigate_follow_record_path`，库名 `nav2_compute_follow_record_path_action_bt_node` |
| [navigation2/nav2_bt_navigator/src/navigators/navigate_follow_record_path.cpp](file:///Users/waiyu/Downloads/src/navigation2/nav2_bt_navigator/src/navigators/navigate_follow_record_path.cpp) | `NavigateFollowRecordPathNavigator`：订阅 `record_path`（`nav_msgs/Path`）作为目标，绑定默认 BT `navigate_follow_record_path_w_replanning_and_recovery.xml` |
| `behavior_trees/navigate_follow_record_path_w_replanning_and_recovery.xml` 及 `...1.xml` | 记录路径跟随行为树 |

**系统角色**：支持“先人工示教记录一条路径，再让小车自动循迹”，是巡检/固定路线场景的业务扩展。

---

### 1.9 `teb_local_planner`（0.9.1）

TEB（Timed Elastic Band）局部规划器，含 `teb_local_planner`（算法 + `TebLocalPlannerROS` 插件）与 `teb_msgs`（轨迹/反馈消息）。本工程把它注册为 `controller_server` 的 `FollowPath` 控制器，配合 `costmap_converter` 做多边形障碍约束。

**系统角色**：接收全局 `/plan` 与局部代价地图，做时间最优轨迹优化，输出 `cmd_vel_nav`。

---

### 1.10 `pointcloud_to_laserscan`

[pointcloud_to_laserscan.launch.py](file:///Users/waiyu/Downloads/src/pointcloud_to_laserscan/launch/pointcloud_to_laserscan.launch.py)：订阅 `/camera/depth/points`，发布 `/scan1`（角度 ±90°，量程 0.05~1m），并发布静态 TF `base_link→camera_link`（0.23, 0, 0.2）。对应代价地图 `observation_sources` 里的 `scan1`。

---

### 1.11 `dependence/` —— 第三方依赖库

| 目录 | 作用 |
|---|---|
| `BehaviorTree.CPP` | `behaviortree_cpp_v3`，Nav2 行为树引擎（XML 解析、Tick 调度、黑板） |
| `costmap_converter` | `costmap_converter` + `costmap_converter_msgs`，把代价地图栅格聚合成多边形/线段，供 TEB 使用 |

---

### 1.12 `yhs_tutorials` + `tools` —— 教学示例

| 文件 | 功能 |
|---|---|
| `yhs_tutorials/lidar_pkg/src/lidar_data_node.cpp` | 订阅 `/scan` 打印激光数据 |
| `yhs_tutorials/pc_pkg/src/pc_node.cpp` | 订阅点云数据 |
| `yhs_tutorials/vel_pub/src/vel_pub_node.cpp` | 发布 `/cmd_vel`（手动给速度） |
| `yhs_tutorials/nav_pkg/src/nav_pkg_node.cpp` | 发布 `goal_pose`、订阅 `GoalStatusArray`（导航目标演示） |
| `yhs_tutorials/nav_pkg1/` | 导航示例变体 |
| [tools/tf.py](file:///Users/waiyu/Downloads/src/tools/tf.py) | 欧拉角→四元数发布示例 |
| [tools/select_car.sh](file:///Users/waiyu/Downloads/src/tools/select_car.sh) | 交互式选择车型，把对应车型 launch 拷贝为 `robot.launch.py`（路径为 `/home/yhs/ros2_ws`，需按实际环境修改） |

---

## 2. 依赖关系、消息通信链路与调用关联逻辑

### 2.1 功能包依赖矩阵（编译期 / 运行期）

| 功能包 | 编译期依赖（关键项） | 运行期依赖 | 在链路中的位置 |
|---|---|---|---|
| `yhs_can_interfaces` | `ament_cmake`、`rosidl_default_generators`、`std_msgs`/`nav_msgs`/`geometry_msgs`/`action_msgs` | `rosidl_default_runtime` | 最底层接口，被 `yhs_can_control`、`nav2_behavior_tree` 引用 |
| `yhs_can_control` | `rclcpp`、`yhs_can_interfaces`、`tf2_ros`、`tf2_geometry_msgs`、`sensor_msgs`、`geometry_msgs`、`nav2_msgs`（声明未使用） | Linux `PF_CAN`/`can0` | 执行末端，唯一与电机通信的节点 |
| `yhs_chassis_description` | `ament_cmake` | `robot_state_publisher`、`joint_state_publisher`、`rviz2`、`xacro` | 只提供模型/静态 TF，不参与算法链路 |
| `yhs_nav2` | `ament_cmake`（**无任何 exec_depend**） | 隐式依赖全部运行包 | 顶层编排、参数、地图 |
| `nvilidar_ros2` | `rclcpp`、`sensor_msgs`、`visualization_msgs`、`geometry_msgs`、`std_srvs` + 厂商 SDK | 串口 `/dev/lidar` | 感知源，产出 `/scan` |
| `serial_imu` | `rclcpp`、`sensor_msgs`、`std_msgs` | 串口 `/dev/imu` | 感知源，产出 `/imu_data` |
| `orbbec_camera`（可选） | `rclcpp`、`image_transport`、`cv_bridge`、OrbbecSDK | USB3 | 可选感知，产出图像/点云/`/camera/imu` |
| `cartographer` | `ceres`、`protobuf`、`lua`、`eigen` | — | SLAM 算法库 |
| `cartographer_ros` | `cartographer`、`cartographer_ros_msgs`、`tf2_ros`、`urdf`、`pcl`、`nav_msgs` | `cartographer_ros_msgs` | 建图节点与地图输出 |
| `cartographer_ros_msgs` | `rosidl_default_generators`、`std_msgs`、`geometry_msgs`、`trajectory_msgs` | — | 子图/轨迹/服务接口 |
| `cartographer_rviz` | `rviz_common`、`cartographer_ros_msgs` | — | 建图可视化插件 |
| `slam_gmapping` + `openslam_gmapping` | `tf2`、`message_filters`、`openslam_gmapping` | — | 备选建图 |
| `nav2_core` / `nav2_msgs` / `nav2_util` / `nav2_common` | `rclcpp`、`rclcpp_lifecycle`、`pluginlib`、`nav2_msgs` | — | 插件接口、消息、生命周期与工具基座 |
| `nav2_amcl` | `nav2_util`、`nav2_msgs`、`message_filters`、`tf2_ros` | — | 定位，发布 `map→odom` |
| `nav2_map_server` | `nav2_util`、`nav2_msgs`、`yaml-cpp` | — | 加载/保存地图 |
| `nav2_costmap_2d` | `nav2_util`、`nav2_voxel_grid`、`nav2_msgs`、`pluginlib`、`tf2` | — | 代价地图，被规划/控制/行为共同消费 |
| `nav2_planner` + `nav2_navfn_planner` | `nav2_core`、`nav2_costmap_2d`、`nav2_util`、`nav_msgs` | — | 全局规划 |
| `nav2_controller` | `nav2_core`、`nav2_costmap_2d`、`nav2_msgs`、`nav2_util` | 动态加载控制器插件 | 局部控制宿主 |
| `nav2_behaviors` | `nav2_core`、`nav2_costmap_2d`、`nav2_util` | — | 恢复行为服务器 |
| `nav2_behavior_tree` | `behaviortree_cpp_v3`、`nav2_msgs`、**`yhs_can_interfaces`（定制）** | — | 行为树节点库 |
| `nav2_bt_navigator` | `nav2_behavior_tree`、`nav2_util`、**`yhs_can_interfaces`** | — | 导航入口（Action Server） |
| `nav2_velocity_smoother` | `nav2_util`、`nav2_msgs`、`tf2` | — | 速度限幅与平滑 |
| `nav2_waypoint_follower` | `nav2_core`、`nav2_msgs`、`nav2_util` | — | 航点任务 |
| `nav2_lifecycle_manager` | `nav2_msgs`、`lifecycle_msgs`、`bondcpp` | — | 生命周期与心跳 |
| `teb_local_planner` | `nav2_core`、`nav2_costmap_2d`、**`costmap_converter`**、`libg2o`、`teb_msgs` | 以插件形式被 `controller_server` 加载 | 局部控制器 |
| `costmap_converter` | `pluginlib`、`nav2_costmap_2d`、`costmap_converter_msgs` | 被 TEB 加载 | 栅格→多边形 |
| `pointcloud_to_laserscan` | `rclcpp`、`sensor_msgs`、`tf2_ros` | 相机点云 | 可选第二激光源 |

### 2.2 编译期依赖图（自底向上）

```
                     ROS 2 Humble 基础
     rclcpp / rclcpp_lifecycle / tf2_ros / pluginlib / message_filters
                              │
      ┌───────────────────────┼─────────────────────────────┐
      │                       │                             │
yhs_can_interfaces        nav2_msgs                     sensor_msgs
      │                       │                             │
yhs_can_control          nav2_core ── nav2_util ── nav2_common
      │                       │                             │
      │              ┌────────┴─────────┐          nvilidar_ros2 / serial_imu
      │              │                  │                   │
      │        nav2_costmap_2d     nav2_behavior_tree ──► yhs_can_interfaces
      │          │        │              │
      │   nav2_navfn_planner  nav2_bt_navigator ──► yhs_can_interfaces
      │          │        │
      │     nav2_planner  nav2_controller ──pluginlib──► teb_local_planner
      │                       │                              │
      │                  nav2_behaviors                costmap_converter
      │
cartographer ──► cartographer_ros ──► cartographer_ros_msgs
                              │
                        yhs_nav2（launch/param/map，无 C++ 依赖）
```

两类耦合必须分清：

- **编译期硬依赖（不加就无法 build）**：`yhs_can_control → yhs_can_interfaces`；`nav2_behavior_tree / nav2_bt_navigator → yhs_can_interfaces`；`teb_local_planner → costmap_converter`；`cartographer_ros → cartographer`。
- **运行期软依赖（通过话题/插件发现，不编译耦合）**：`controller_server → teb_local_planner`（pluginlib 按类名加载）；`nav2_behavior_tree → nav2_msgs` 动作；所有节点之间通过 `/scan`、`/odom`、`/cmd_vel`、TF 通信。

### 2.3 运行期节点—接口矩阵

| 节点（可执行文件） | 订阅 | 发布 | 服务 / 动作 | TF |
|---|---|---|---|---|
| `nvilidar_ros2_node` | — | `scan` | — | 不发布（launch 另起静态 TF） |
| `serial_imu_node` | — | `imu_data` | — | 不发布 |
| `yhs_can_control_node` | `cmd_vel`、`ctrl_cmd`、`free_ctrl_cmd`、`io_cmd`、`imu_data` | `odom`、`chassis_info_fb` | — | `odom→base_link`（`use_tf`） |
| `cartographer_node` | `scan`、`imu`(remap→`imu_data`)、`odom` | `submap_list`、`scan_matched_points2`、`tracked_pose`、约束/轨迹 Marker | `submap_query`、`trajectory_query`、`start/finish_trajectory`、`write_state`、`get_trajectory_states`、`read_metrics` | `map→odom`、`odom→base_link`（`provide_odom_frame=true`） |
| `cartographer_occupancy_grid_node` | `submap_list` | `map` | `submap_query`（客户端） | — |
| `map_server` | — | `map`（transient_local） | `load_map`、`save_map` | — |
| `amcl` | `scan`、`map`、`initialpose` | `amcl_pose`、`particle_cloud` | `/reinitialize_global_localization`、`/request_nomotion_update` | `map→odom`（`tf_broadcast=true`） |
| `planner_server` | `global_costmap/costmap`、`goal_pose`（RViz 工具） | `plan`、`global_costmap/published_footprint` | `compute_path_to_pose`、`compute_path_through_poses` | 消费 `map→base_link` |
| `controller_server`（TEB） | `plan`、`odom`、`local_costmap/costmap` | `cmd_vel_nav`（remap） | `follow_path` | 消费 `odom→base_link` |
| `behavior_server` | `local_costmap/costmap_raw`、`odom` | `cmd_vel`（行为内部） | `spin`、`back_up`、`drive_on_heading`、`wait`、`assisted_teleop` | 消费 `odom→base_link` |
| `bt_navigator` | `odom`、`goal_pose`、`record_path` | `behavior_tree_log` | `navigate_to_pose`、`navigate_through_poses`、`navigate_follow_record_path` | 消费 `map→base_link` |
| `waypoint_follower` | `odom` | — | `follow_waypoints` | 消费 `map→base_link` |
| `smoother_server` | `global_costmap/costmap` | `plan_smoothed` | `smooth_path` | 消费 `map→base_link` |
| `velocity_smoother` | `cmd_vel_nav`、`odom` | `cmd_vel` | — | — |
| `lifecycle_manager_localization` | — | — | `manage_nodes` | — |
| `lifecycle_manager_navigation` | — | — | `manage_nodes` | — |
| `pointcloud_to_laserscan_node`（可选） | `camera/depth/points` | `scan1` | — | `base_link→camera_link`（launch 静态） |

### 2.4 话题连接矩阵与 remap 关系

| 连接 | 话题（实际解析后） | 类型 / QoS | 来源与转发 |
|---|---|---|---|
| 雷达 → 定位/规划/建图 | `/scan` | `LaserScan` / `SensorDataQoS`（best effort） | [nvilidar_ros2_node.cpp](file:///Users/waiyu/Downloads/src/hardware/nvilidar_ros2/src/nvilidar_ros2_node.cpp#L110) 发布；AMCL 用 `rmw_qos_profile_sensor_data` 订阅，QoS 匹配 |
| 相机点云 → 第二激光 | `/scan1` | `LaserScan` | `pointcloud_to_laserscan` 将 `cloud_in` 重映射为 `camera/depth/points`，`scan` 重映射为 `scan1` |
| IMU → 底盘 | `/imu_data` | `Imu` | `serial_imu` 发布；`yhs_can_control` 订阅同一名字 |
| IMU → 建图 | `imu_data`（Cartographer 内部名） | `Imu` | [dgt_mini.launch.py](file:///Users/waiyu/Downloads/src/cartographer_ros/cartographer_ros/launch/dgt_mini.launch.py#L39-L42) `('imu','imu_data')` 重映射 |
| 底盘 → 定位/控制 | `/odom` | `Odometry` | `yhs_can_control` 发布，`odom` 帧 → `base_link` 帧 |
| 地图 → 定位/规划 | `/map` | `OccupancyGrid` / transient_local | `map_server` 或 `cartographer_occupancy_grid_node` 发布 |
| 规划 → 控制 | `/plan` | `Path` | `planner_server` 发布，`controller_server` 订阅 |
| 控制 → 平滑 | `cmd_vel_nav` | `Twist` | [navigation_launch.py](file:///Users/waiyu/Downloads/src/yhs_nav2/launch/navigation_launch.py#L93) 把 `controller_server` 的 `cmd_vel` 重映射为 `cmd_vel_nav` |
| 平滑 → 底盘 | `/cmd_vel` | `Twist` | [navigation_launch.py](file:///Users/waiyu/Downloads/src/yhs_nav2/launch/navigation_launch.py#L153-L154) 把 `velocity_smoother` 的 `cmd_vel` 收到 `cmd_vel_nav`，`cmd_vel_smoothed` 发出为 `cmd_vel` |
| 定位结果 | `/amcl_pose`、`/particle_cloud` | `PoseWithCovarianceStamped`、`ParticleCloud` | AMCL → RViz/监控 |
| 底盘状态 | `/chassis_info_fb` | `ChassisInfoFb` | 轮速、IO、BMS 聚合反馈 |
| 记录路径 | `/record_path` | `Path` | 示教端 → `NavigateFollowRecordPathNavigator` |

三个关键 remap 规则（读 launch 时必须注意）：

1. 控制输出统一走 `cmd_vel_nav`，只有 `velocity_smoother` 有资格发 `cmd_vel` —— 这样速度限幅对**所有**控制源（TEB、恢复行为）都生效。
2. `controller_server` 与 `velocity_smoother` 都做了 `('/tf','tf')`、`('/tf_static','tf_static')` 重映射，避免命名空间下 TF 话题被二次加前缀。
3. `RewrittenYaml` 把 launch 参数 `map` 注入为所有节点的 `yaml_filename`，因此 `yhs_nav2.yaml` 里写死的 `map.yaml` 不会生效（[yhs_nav2.launch.py](file:///Users/waiyu/Downloads/src/yhs_nav2/launch/yhs_nav2.launch.py#L43-L51)）。

### 2.5 调用关联逻辑

#### 2.5.1 launch 调用树（谁调用谁）

```
ros2 launch yhs_nav2 yhs_nav2.launch.py
└── bringup_cmd_group (GroupAction)
    ├── Node nav2_container（rclcpp_components/component_container_isolated）
    ├── Include serial_imu/launch/imu_spec_msg.launch.py
    ├── Include nvilidar_ros2/launch/nvilidar.launch.py
    ├── (注释) Include orbbec_camera/launch/astra.launch.py
    ├── (注释) Include pointcloud_to_laserscan/launch/pointcloud_to_laserscan.launch.py
    ├── Include yhs_can_control/launch/yhs_can_control.launch.py
    ├── Include yhs_nav2/launch/localization_launch.py
    │   ├── Node nav2_map_server/map_server
    │   ├── Node nav2_amcl/amcl
    │   └── Node nav2_lifecycle_manager/lifecycle_manager（node_names=[map_server, amcl]）
    └── Include yhs_nav2/launch/navigation_launch.py
        ├── Node nav2_controller/controller_server      （cmd_vel→cmd_vel_nav）
        ├── Node nav2_smoother/smoother_server
        ├── Node nav2_planner/planner_server
        ├── Node nav2_behaviors/behavior_server
        ├── Node nav2_bt_navigator/bt_navigator
        ├── Node nav2_waypoint_follower/waypoint_follower
        ├── Node nav2_velocity_smoother/velocity_smoother（cmd_vel←cmd_vel_nav，cmd_vel_smoothed→cmd_vel）
        └── Node nav2_lifecycle_manager/lifecycle_manager（node_names=7 个导航节点）
```

建图入口的调用树：

```
ros2 launch yhs_nav2 cartographer.launch.py
├── Include serial_imu/imu_spec_msg.launch.py
├── Include nvilidar_ros2/nvilidar.launch.py
├── Include yhs_can_control/yhs_can_control.launch.py   (use_tf 默认 false)
└── Include cartographer_ros/dgt_mini.launch.py
    ├── Node cartographer_ros/cartographer_node（-configuration_directory/-configuration_basename dgt_mini.lua）
    └── Node cartographer_ros/cartographer_occupancy_grid_node（resolution=0.05）
```

#### 2.5.2 生命周期与心跳调用顺序

1. `lifecycle_manager_localization` 对 `[map_server, amcl]` 依次调用 `configure → activate`；`map_server` 先加载地图，AMCL 才能收到 `map`（transient_local 保证晚加入也能拿到）。
2. `lifecycle_manager_navigation` 对 `[controller_server, smoother_server, planner_server, behavior_server, bt_navigator, waypoint_follower, velocity_smoother]` 依次 `configure → activate`。
3. 每个被管节点激活时向 manager 建立 `bond`（`bondcpp`），周期心跳；节点崩溃 → manager 收到断链并报错/退出（`use_respawn` 可重启独立进程）。
4. `bt_navigator` 的 `configure` 阶段完成三件事：加载 BT 插件库（`plugin_lib_names`）、创建 3 个 Navigator、为每个 Navigator 绑定默认 BT XML 与黑板键。

#### 2.5.3 Action / Service 调用链

**导航任务（标准链路）**

```
RViz「2D Goal Pose」/ simple_commander / 自定义脚本
  └─ action goal: /navigate_to_pose (nav2_msgs/NavigateToPose)
      └─ bt_navigator（Action Server）
          └─ BT: navigate_to_pose_w_replanning_and_recovery.xml
              ├─ action client: /compute_path_to_pose → planner_server（NavFn）
              │     └─ 读 global_costmap → 发布 /plan
              ├─ action client: /follow_path → controller_server（TEB）
              │     ├─ 订阅 /plan、/odom、local_costmap
              │     └─ 发布 cmd_vel_nav
              └─ 失败分支：
                  ├─ service: /local_costmap/clear_entirely_local_costmap
                  ├─ service: /global_costmap/clear_entirely_global_costmap
                  ├─ action: /spin → behavior_server
                  ├─ action: /wait → behavior_server
                  └─ action: /back_up → behavior_server
```

**记录路径（定制链路）**

```
示教端 ──发布 /record_path (nav_msgs/Path)──► NavigateFollowRecordPathNavigator
      或 action goal: /navigate_follow_record_path (yhs_can_interfaces/RecordPath)
  └─ BT: navigate_follow_record_path_w_replanning_and_recovery.xml
      └─ RateController(5Hz) → FollowPath(path={path}, controller_id="FollowPath")
          └─ controller_server(TEB) → cmd_vel_nav → velocity_smoother → cmd_vel
      └─ 恢复：ClearEntireCostmap → Spin → Wait → BackUp
```

与标准 BT 的差异：记录路径 BT **只做 `FollowPath`，不含 `ComputePathToPose`**，因此 `/record_path` 必须是连续、可跟踪的完整路径；重规划由外层以 5 Hz 重新下发路径实现。

**建图服务链**

```
cartographer_occupancy_grid_node ──service: /submap_query──► cartographer_node
保存：/finish_trajectory → /write_state (cartographer_ros_msgs/WriteState)
```

#### 2.5.4 pluginlib 类加载链

| 宿主节点 | 加载参数 | 实际类名 | 插件描述文件 |
|---|---|---|---|
| `controller_server` | `controller_plugins: [FollowPath]` + `FollowPath.plugin` | `teb_local_planner::TebLocalPlannerROS` | `teb_local_planner/teb_local_planner_plugin.xml` |
| `controller_server` | `progress_checker_plugin` | `nav2_controller::SimpleProgressChecker` | `nav2_controller` 插件库 |
| `controller_server` | `goal_checker_plugins: [general_goal_checker]` | `nav2_controller::SimpleGoalChecker` | 同上 |
| `planner_server` | `planner_plugins: [GridBased]` | `nav2_navfn_planner/NavfnPlanner` | `nav2_navfn_planner` |
| `behavior_server` | `behavior_plugins`（源码默认） | `nav2_behaviors/Spin`、`BackUp`、`DriveOnHeading`、`Wait` | `nav2_behaviors/behavior_plugin.xml` |
| `bt_navigator` | `plugin_lib_names`（33 项） | BT 节点库，含 `nav2_compute_follow_record_path_action_bt_node` | 各 `*_bt_node` 库 |
| `TebLocalPlannerROS` | `costmap_converter_plugin` | `costmap_converter::CostmapToPolygonsDBSMCCH` | `costmap_converter` 插件描述 |

#### 2.5.5 TF 依赖链（谁发布、谁消费）

```
map ──(AMCL 发布，tf_broadcast=true)──► odom
odom ──(yhs_can_control 发布，use_tf=true)──► base_link
base_link ──(nvilidar launch 静态)──► laser_link
base_link ──(serial_imu launch 静态)──► imu_link
base_link ──(pointcloud_to_laserscan launch 静态)──► camera_link
```

- **消费方**：AMCL（`map→base_link`）、两个 costmap（`map/odom→base_link`）、TEB（`odom→base_link`）、BT 的 `TransformAvailable` 条件、RViz。
- **发布方冲突规则**：建图时 `map→odom` 与 `odom→base_link` 都由 Cartographer 发布（`provide_odom_frame=true`），底盘必须 `use_tf=false`；导航时反过来，由 AMCL + 底盘发布。`cartographer.launch.py` 已把 `use_tf` 默认值改为 `false`，`yhs_nav2.launch.py` 保持 `true`，这一对默认值正是两条链路的切换开关。

### 2.6 全链路时序（从目标到轮速）

1. RViz 发出 `/navigate_to_pose` goal；`bt_navigator` 校验目标 frame（`map`）与 TF 可用性。
2. BT 的 `ComputePathToPose` 节点调用 `planner_server`；`planner_server` 等待 global costmap 完成一次更新后运行 NavFn，返回 `/plan`。
3. BT 的 `FollowPath` 节点以 20 Hz 调用 `controller_server`；TEB 订阅 `/plan`、`/odom`、`local_costmap/costmap`，并周期性调用 `costmap_converter` 把栅格转成多边形。
4. TEB 优化出速度指令，`controller_server` 发布 `cmd_vel_nav`。
5. `velocity_smoother` 以 20 Hz 接收 `cmd_vel_nav`，按 `max_velocity`/`max_accel` 限幅后发布 `cmd_vel`。
6. `yhs_can_control` 的 `cmd_vel` 回调把 `linear.x`、`angular.z`（转角度）打包为 `CtrlCmd`，经 CAN ID `0x98C4D1D0` 下发。
7. 底盘回传 `CtrlFb`/轮速/BMS 帧；驱动解析后：
   - 发布 `/odom`（`odom→base_link`），
   - 发布 `chassis_info_fb`，
   - 用轮速/角度更新积分位姿。
8. AMCL 消费 `/scan` + `/odom`，更新粒子并发布新的 `map→odom`。
9. costmap 消费 `/scan` + TF，刷新障碍层；TEB 下一周期据此避障。
10. BT 收到 `FollowPath` 的 feedback/result，判断到达（`xy_goal_tolerance=0.1`、`yaw_goal_tolerance=0.1`）或触发恢复分支。

### 2.7 建图链路与导航链路的区别（调用层面）

| 维度 | 建图链路 | 导航链路 |
|---|---|---|
| 入口 | `cartographer.launch.py` / `gmapping.launch.py` | `yhs_nav2.launch.py` |
| 是否加载 Nav2 | 否 | 是（7 个服务器 + 2 个 lifecycle_manager） |
| 里程计来源 | 底盘 `/odom` + Cartographer 前端 | 底盘 `/odom` |
| `map→odom` | Cartographer | AMCL |
| `odom→base_link` | Cartographer（`use_tf=false` 底盘） | 底盘（`use_tf=true`） |
| `/map` | `cartographer_occupancy_grid_node` / gmapping | `map_server` |
| 控制输出 | 无（人工遥控发 `cmd_vel`） | TEB → `cmd_vel_nav` → smoother → `cmd_vel` |

### 2.8 配置与代码一致性核对（会直接影响链路的 9 项）

1. **组合模式未真正生效**：`yhs_nav2.launch.py` 启动了 `nav2_container`，但 `localization_launch.py`/`navigation_launch.py` 用的是独立 `Node` 而非 `LoadComposableNodes`。因此 `use_composition:=true` 只会多出一个空容器，各服务器仍是独立进程（不影响功能，但 IPC 开销与预期不符）。
2. **`bt_navigator` 的 BT 文件名参数失效**：yaml 写的是旧键 `default_bt_xml_filename`，而源码声明的是 `default_nav_to_pose_bt_xml`（[navigate_to_pose.cpp](file:///Users/waiyu/Downloads/src/navigation2/nav2_bt_navigator/src/navigators/navigate_to_pose.cpp#L70-L79)）。且 yaml 指定的 `navigate_w_replanning_and_recovery.xml` 在 `behavior_trees/` 中**不存在**。实际生效的是源码默认值 `navigate_to_pose_w_replanning_and_recovery.xml`。
3. **恢复行为参数块失效**：yaml 的 `recoveries_server` 节点名与实际节点 `behavior_server` 不一致，且参数键应为 `behavior_plugins`（[behavior_server.cpp](file:///Users/waiyu/Downloads/src/navigation2/nav2_behaviors/src/behavior_server.cpp#L24-L41)），插件类名也已从 `nav2_recoveries/*` 改为 `nav2_behaviors/*`。结果：该段配置完全不生效，恢复行为改用源码默认值（`Spin/BackUp/DriveOnHeading/Wait`）。
4. **缺少 `velocity_smoother`、`smoother_server`、`behavior_server` 参数段**：这三个节点全部以默认参数运行。`velocity_smoother` 默认 `max_velocity=[0.5,0,2.5]`、`smoothing_frequency=20`、`feedback=OPEN_LOOP`，恰好与 TEB 的 0.5 m/s 上限一致，但显式配置更安全。
5. **局部代价地图没有膨胀层**：`local_costmap.plugins` 只列了 `static_layer`、`obstacle_layer`，但下面又写了 `inflation_layer` 配置。该配置不生效，局部避障完全依赖 TEB 的 `min_obstacle_dist=0.45` 与 `inflation_dist=0.6` 兜底。
6. **`/scan1` 观测源长期为空**：两个 costmap 都配置了 `scan1`，但总 launch 中相机与 `pointcloud_to_laserscan` 被注释（[yhs_nav2.launch.py](file:///Users/waiyu/Downloads/src/yhs_nav2/launch/yhs_nav2.launch.py#L133-L146)）。不影响主链路，但会让代价地图日志持续提示该源无数据。
7. **`yhs_nav2` 未声明运行依赖**：`package.xml` 只有 `ament_cmake` 与 lint 依赖，launch 实际依赖 `serial_imu`、`nvilidar_ros2`、`yhs_can_control`、`nav2_*`、`teb_local_planner` 均未声明，`rosdep install` 与依赖图排序会漏项。
8. **雷达 launch 硬编码命名空间**：`nvilidar.launch.py` 写死 `namespace='/'`，也没有声明 `use_sim_time`，总 launch 传入的这些参数对它无效；IMU launch 同样未声明 `namespace`。
9. **底盘参数文件未真正加载**：`yhs_can_control.launch.py` 写成 `parameters=[{parameter_file}, {'use_tf': use_tf}]`，第一个元素是**集合字面量**而非路径字符串，`cfg.yaml` 不会被读取。好在源码默认值 `can_name="can0"`、`use_tf=false` 与 `cfg.yaml`/launch 意图一致，`use_tf` 又由第二个 dict 覆盖，所以功能上看不出异常。

> 另外两个小问题：`cartographer_occupancy_grid_node` 的 `use_sim_time` 在 [dgt_mini.launch.py](file:///Users/waiyu/Downloads/src/cartographer_ros/cartographer_ros/launch/dgt_mini.launch.py#L49-L51) 中硬编码为 `True`；`yhs_can_interfaces/CMakeLists.txt` 的条件变量拼写为 `CMAKECOMPILER_IS_GNUCXX`（缺少下划线），使该编译选项块恒不生效。
>
> 上述问题的可执行补丁、优先级与回归步骤见 **第 8 章 修复方案**。

---

## 3. 完整启动流程

### 3.1 前置条件

1. **系统**：Ubuntu 22.04 + ROS 2 Humble；安装缺失系统包：
   ```bash
   sudo apt install ros-humble-navigation2 ros-humble-nav2-bringup ros-humble-slam-toolbox
   sudo apt install ros-humble-robot-state-publisher ros-humble-joint-state-publisher
   ```
2. **CAN 总线**：内核加载 `can` 模块，配置 `can0`（波特率需与底盘匹配，通常 500k），并 `sudo ip link set can0 up type can bitrate 500000`。
3. **设备权限**：用户加入 `dialout`；为雷达/IMU 固定设备名：
   - 雷达参数为 `/dev/lidar`（[nvilidar.yaml](file:///Users/waiyu/Downloads/src/hardware/nvilidar_ros2/params/nvilidar.yaml)），需 udev 规则或软链接。
   - IMU 源码硬编码 `/dev/imu`（[serial_port.cpp](file:///Users/waiyu/Downloads/src/hardware/serial_imu/src/serial_port.cpp#L17)），需 udev 规则。
4. **编译**：
   ```bash
   cd <工作空间>
   colcon build --symlink-install --packages-up-to yhs_nav2 yhs_can_control cartographer_ros teb_local_planner
   source install/setup.bash
   ```

### 3.2 建图（SLAM）阶段

```bash
# Cartographer 建图（一海推荐，一条命令拉起 IMU+雷达+底盘+建图）
ros2 launch yhs_nav2 cartographer.launch.py

# 或 Gmapping 建图
ros2 launch yhs_nav2 gmapping.launch.py
```

`cartographer.launch.py` 内部 include 顺序：`serial_imu` → `nvilidar` → `yhs_can_control`（`use_tf=false`，让 Cartographer 发 `map→odom`）→ `cartographer_ros/dgt_mini.launch.py`。

**保存地图**：
```bash
# 保存 Cartographer 发布的 /map
ros2 run nav2_map_server map_saver_cli -f ~/maps/my_map
# 或存 pbstream 后再转 ROS 地图
ros2 service call /finish_trajectory cartographer_ros_msgs/srv/FinishTrajectory "{trajectory_id: 0}"
ros2 service call /write_state cartographer_ros_msgs/srv/WriteState "{filename: '$HOME/maps/my_map.pbstream'}"
ros2 run cartographer_ros cartographer_pbstream_to_ros_map -pbstream_filename $HOME/maps/my_map.pbstream -map_filestem $HOME/maps/my_map
```

### 3.3 定位 + 导航阶段

```bash
# 一键启动（默认地图 ecarroom.yaml，可覆盖）
ros2 launch yhs_nav2 yhs_nav2.launch.py map:=<绝对路径>/my_map.yaml
```

`yhs_nav2.launch.py` 内部 include 顺序（对应启动先后逻辑）：

1. `nav2_container`（`component_container_isolated`，若 `use_composition=true`）
2. `serial_imu` → 发布 `/imu_data`
3. `nvilidar` → 发布 `/scan` + `base_link→laser_link`
4. `yhs_can_control`（`use_tf=true`）→ 发布 `/odom` + `odom→base_link`
5. `localization_launch` → `map_server` + `amcl` + `lifecycle_manager_localization`
6. `navigation_launch` → `controller_server`/`smoother_server`/`planner_server`/`behavior_server`/`bt_navigator`/`waypoint_follower`/`velocity_smoother` + `lifecycle_manager_navigation`

生命周期启动先后：

1. 各 LifecycleNode 先以 `unconfigured` 状态创建。
2. `lifecycle_manager_localization`（`[map_server, amcl]`）依次 `configure → activate`。
3. `lifecycle_manager_navigation`（`[controller_server, smoother_server, planner_server, behavior_server, bt_navigator, waypoint_follower, velocity_smoother]`）依次 `configure → activate`。
4. `autostart=true` 时全自动；配合 `bond` 心跳监控节点存活。

**初始化位姿 + 发目标**：
```bash
# 设置 AMCL 初始位姿（或用 RViz 的 2D Pose Estimate）
ros2 run yhs_nav2 set_init_amcl_pose.py --ros-args -p x:=0.0 -p y:=0.0 -p theta:=0.0
# RViz 中用 2D Goal Pose / Nav2 Panel 发目标；或 Python API
```

---

## 4. 关键配置项说明（`yhs_nav2.yaml`）

| 配置项 | 所属 | 作用 / 说明 |
|---|---|---|
| `use_sim_time` | 全局 | 实车必须 `false`；`RewrittenYaml` 在 launch 期统一注入 |
| `map` | 顶层 launch | 指向 `.yaml` 地图，经 `yaml_filename` 注入 map_server |
| `base_frame_id` / `robot_base_frame` | amcl / costmap / bt_navigator | 统一 `base_link`，与底盘驱动 `odom→base_link` 对齐 |
| `scan_topic: scan`、`laser_max_range: 48.0`、`laser_model_type: likelihood_field`、`max_beams: 48` | amcl | 激光定位模型与参数 |
| `set_initial_pose: true` + `initial_pose` | amcl | 默认从原点初始化，省去手动设位姿 |
| `controller_frequency: 20.0`、`FollowPath.plugin: teb_local_planner::TebLocalPlannerROS` | controller_server | 控制频率与 TEB 局部规划器 |
| `max_vel_x: 0.5`、`max_vel_theta: 0.3`、`acc_lim_*: 0.3` | TEB | 速度/加速度上限，需与底盘能力匹配 |
| `footprint_model: circular / radius 0.1`、`footprint` 多边形 | TEB / costmap | 机器人轮廓，影响避障安全距离 |
| `min_obstacle_dist: 0.45`、`inflation_dist: 0.6` | TEB | 障碍安全距离与膨胀 |
| `costmap_converter_plugin: CostmapToPolygonsDBSMCCH` | TEB | 代价地图转多边形，供 TEB 约束 |
| `planner_plugins.GridBased.plugin: nav2_navfn_planner/NavfnPlanner` | planner_server | 全局规划器 NavFn |
| `observation_sources: scan scan1` | costmap | `/scan` + `/scan1`（深度相机转换）双源 |
| `recovery_plugins: [spin, backup, wait]` | recoveries_server | 恢复行为集 |
| `serialport_name: /dev/lidar`、`serialport_baud: 512000`、`frame_id: laser_link` | nvilidar | 雷达串口与坐标系 |
| `/dev/imu`、`B115200` | serial_imu | IMU 串口 |
| `can_name: can0`、`use_tf: true` | yhs_can_control | CAN 接口名、是否广播 `odom→base_link` |
| `provide_odom_frame` / `use_odometry` / `use_imu_data` | dgt_mini.lua | 建图时坐标来源与 IMU 融合开关 |
| `submaps.num_range_data: 60`、`optimize_every_n_nodes: 5` | dgt_mini.lua | 子图帧数与位姿图优化频率 |

---

## 5. 各模块协同运行机制（运动控制 / 感知 / 决策）

1. **感知**：nvilidar 以 `SensorDataQoS` 发布 `/scan`（时间戳取雷达采样时刻）；IMU 发布 `/imu_data`；可选相机点云经 `pointcloud_to_laserscan` 生成 `/scan1`。
2. **定位（AMCL）**：`map_server` latched 发布 `/map`；AMCL 以 `likelihood_field` 模型 + 48 束更新粒子权重，输出 `map→odom` TF 与 `/amcl_pose`。
3. **建图（Cartographer）**：`cartographer_node` 用相关性扫描匹配 + Ceres 优化，每 60 帧建子图、每 5 节点位姿图优化；`occupancy_grid_node` 拼子图发布 `/map`。
4. **代价地图**：global（`map` 系，static+obstacle+inflation，1Hz）/ local（`odom` 系滚动窗，static+obstacle，5Hz）融合静态地图与实时激光。
5. **全局规划**：`planner_server` 在全局代价地图上运行 NavFn，输出 `/plan`。
6. **局部控制（TEB）**：`controller_server` 20Hz 调用 TEB，做时间最优轨迹优化（受 `costmap_converter` 多边形障碍约束），输出 `cmd_vel_nav`。
7. **速度平滑**：`velocity_smoother` 对 `cmd_vel_nav` 限幅限加速度后发 `cmd_vel`。
8. **执行（底盘）**：`yhs_can_control` 把 `cmd_vel` 组 CAN 帧下发，回读轮速/IO/BMS，积分发布 `/odom`，形成闭环。
9. **决策（行为树）**：`bt_navigator` 以 `navigate_w_replanning_and_recovery.xml` 编排“重规划 + 跟随 + 恢复”，失败时触发清代价地图/Spin/Wait/BackUp。
10. **记录路径导航**：`navigate_follow_record_path` 动作接受 `record_path`，按 `navigate_follow_record_path_w_replanning_and_recovery.xml` 循迹。

---

## 6. 风险点与上线核对清单

1. **设备名不一致**：参数用 `/dev/lidar`，`nvilidar/startup/initenv.sh` 生成的符号链接名可能不同；IMU 硬编码 `/dev/imu` 无 udev 规则。建议统一 udev：
   ```bash
   SUBSYSTEM=="tty", ATTRS{idVendor}=="10c4", ATTRS{idProduct}=="ea60", SYMLINK+="lidar", MODE="0777"
   # IMU 按实际 VID/PID 建 /dev/imu
   ```
2. **`use_sim_time` 硬编码**：[dgt_mini.launch.py](file:///Users/waiyu/Downloads/src/cartographer_ros/cartographer_ros/launch/dgt_mini.launch.py#L50) 中 `cartographer_occupancy_grid_node` 固定 `use_sim_time: True`，实车建图可能出现时间戳异常，建议改为参数化。
3. **TF 冲突**：建图时 Cartographer 发 `map→odom` 与 `odom→base_link`，导航时 AMCL 发 `map→odom`、底盘发 `odom→base_link`。切换阶段必须先停掉另一方；建图 launch 已把 `use_tf` 默认置 `false`。
4. **robot.urdf 网格路径**：[robot.urdf](file:///Users/waiyu/Downloads/src/yhs_chassis_description/urdf/robot.urdf) 引用 `mesh/autocube.STL` 不存在，实际用车型专用 URDF。
5. **select_car.sh 路径**：硬编码 `/home/yhs/ros2_ws`，与本机 `/Users/waiyu/Downloads/src` 不符，仅作参考。
6. **`/scan1` 依赖相机**：costmap 的 `scan1` 源只有在相机 + `pointcloud_to_laserscan` 启动后才存在；默认 launch 中相机已注释，故通常只有 `/scan`，不影响主流程。
7. **TEB 依赖 `costmap_converter`**：`dependence/costmap_converter` 必须编译安装，否则 TEB 加载 `CostmapToPolygonsDBSMCCH` 失败。
8. **recoveries_server 命名**：yaml 用旧键 `recoveries_server`，而 launch 启动的是 `behavior_server`（Nav2 新命名），二者是否对齐需按实际 Nav2 版本确认；本工程主要恢复由 `bt_navigator` 的 BT 触发。
9. **地图分辨率**：代价地图 `resolution: 0.05` 与建图 `/map` 分辨率需一致，否则导航精度下降。
10. **示例资源**：`nav2_bringup/maps`、`urdf/turtlebot3*` 是 TurtleBot3 示例，不是本车数据。

---

## 7. 符合 ROS 2 规范的设计要点（学习视角）

1. **节点/组件化**：所有功能以 `rclcpp::Node` / `LifecycleNode` 实现，支持独立进程与组合容器（`use_composition`）。
2. **生命周期管理**：Nav2 服务器均为 LifecycleNode，由 `nav2_lifecycle_manager` 统一 `configure/activate` 并靠 `bond` 监控。
3. **接口标准化**：模块间仅通过标准消息（`sensor_msgs/LaserScan`、`nav_msgs/Odometry`、`geometry_msgs/Twist`）、标准动作与 TF2 交互，底盘自定义部分收敛在 `yhs_can_interfaces`。
4. **插件化**：规划器/控制器/行为/平滑器经 `pluginlib` + `nav2_core` 接口加载，TEB、NavFn、恢复行为均可替换。
5. **参数外置**：集中在 YAML，经 `RewrittenYaml` 在 launch 期注入，避免硬编码。
6. **QoS 匹配**：激光用 `SensorDataQoS`，地图用 `transient_local`，符合传感器/静态地图 QoS 最佳实践。
7. **坐标规范**：统一 `map/odom/base_link/<sensor>_link`（REP-105），`frame_id` 与 TF 一致。
8. **分层解耦**：硬件层不依赖导航层，建图层（Cartographer↔Gmapping）与导航层（Nav2）可独立替换。

---

## 8. 修复方案（针对 2.8 节 6 个关键问题）

> 原则：**只改 launch / YAML，不改 Nav2、TEB 源码**，保证后续升级上游代码时冲突最小。
> 所有补丁以当前基线为准（Nav2 1.1.5 / Humble）。修改后用 `colcon build --symlink-install --packages-select yhs_nav2` 重新安装并在新终端 `source install/setup.bash`。

### 8.1 组合模式未真正生效（优先级 P2）

**根因**：[yhs_nav2.launch.py](file:///Users/waiyu/Downloads/src/yhs_nav2/launch/yhs_nav2.launch.py#L109-L117) 用 `rclcpp_components/component_container_isolated` 起了 `nav2_container`，但 `localization_launch.py`、`navigation_launch.py` 里全部是独立 `Node`，没有任何 `LoadComposableNodes`。容器是空的，`use_composition` 只增加了空进程。

**方案 A（推荐，最小改动）**：去掉容器，明确关闭组合模式。

```python
# yhs_nav2.launch.py：删除/注释以下 Node
# Node(
#     condition=IfCondition(use_composition),
#     name='nav2_container',
#     package='rclcpp_components',
#     executable='component_container_isolated',
#     parameters=[configured_params, {'autostart': autostart}],
#     arguments=['--ros-args', '--log-level', log_level],
#     remappings=remappings,
#     output='screen'),
```

```python
# 同时把默认值改为 False
declare_use_composition_cmd = DeclareLaunchArgument(
    'use_composition', default_value='False',
    description='Whether to use composed bringup')
```

`localization_launch.py` / `navigation_launch.py` 中未使用的 `container_name`、`LoadComposableNodes` 导入可保留（无副作用），也可一并清理。

**方案 B（真正启用组合，改动大）**：把 `Node` 换成 `LoadComposableNodes` + `ComposableNode`，组件类名必须与源码注册名一致：

| 节点 | 组件插件类名 | 注册位置 |
|---|---|---|
| map_server | `nav2_map_server::MapServer` | `nav2_map_server/CMakeLists.txt` |
| amcl | `nav2_amcl::AmclNode` | `nav2_amcl/CMakeLists.txt` |
| controller_server | `nav2_controller::ControllerServer` | `nav2_controller/CMakeLists.txt` |
| planner_server | `nav2_planner::PlannerServer` | `nav2_planner/CMakeLists.txt` |
| smoother_server | `nav2_smoother::SmootherServer` | `nav2_smoother/CMakeLists.txt` |
| behavior_server | `behavior_server::BehaviorServer` | `nav2_behaviors/CMakeLists.txt` |
| bt_navigator | `nav2_bt_navigator::BtNavigator` | `nav2_bt_navigator/CMakeLists.txt` |
| waypoint_follower | `nav2_waypoint_follower::WaypointFollower` | `nav2_waypoint_follower/CMakeLists.txt` |
| velocity_smoother | `nav2_velocity_smoother::VelocitySmoother` | `nav2_velocity_smoother/CMakeLists.txt` |

示例（localization 部分）：

```python
from launch_ros.actions import LoadComposableNodes
from launch_ros.descriptions import ComposableNode

load_nodes = GroupAction(actions=[
    LoadComposableNodes(
        target_container=container_name,
        composable_node_descriptions=[
            ComposableNode(
                package='nav2_map_server', plugin='nav2_map_server::MapServer',
                name='map_server', parameters=[configured_params],
                remappings=remappings, extra_arguments=[{'use_intra_process_comms': True}]),
            ComposableNode(
                package='nav2_amcl', plugin='nav2_amcl::AmclNode',
                name='amcl', parameters=[configured_params],
                remappings=remappings, extra_arguments=[{'use_intra_process_comms': True}]),
        ]),
    # lifecycle_manager 仍作为独立 Node
])
```

注意：组合后节点名必须与 `lifecycle_nodes` 列表完全一致，否则 `manage_nodes` 找不到服务。

**验证**：`ros2 node list` 中只应看到 `nav2_container`，且 `ros2 component list` 能列出各服务器；若选择方案 A，则应看到 9 个独立节点。

### 8.2 `bt_navigator` 的 BT 文件名参数失效（优先级 P1）

**根因**：YAML 使用旧键 `default_bt_xml_filename`，而源码声明的是 `default_nav_to_pose_bt_xml`（[navigate_to_pose.cpp](file:///Users/waiyu/Downloads/src/navigation2/nav2_bt_navigator/src/navigators/navigate_to_pose.cpp#L70-L79)）；且 `navigate_w_replanning_and_recovery.xml` 在 `behavior_trees/` 下不存在。

**补丁**：删除旧键，交给源码默认值（默认值已是指向 share 目录的绝对路径）。

```yaml
bt_navigator:
  ros__parameters:
    use_sim_time: False
    global_frame: map
    robot_base_frame: base_link
    odom_topic: /odom
    # 删除：default_bt_xml_filename: "navigate_w_replanning_and_recovery.xml"
    # 如需自定义，必须写绝对路径，否则 BT 加载会失败：
    # default_nav_to_pose_bt_xml: "/opt/ros/humble/share/nav2_bt_navigator/behavior_trees/navigate_to_pose_w_replanning_and_recovery.xml"
    # default_nav_through_poses_bt_xml: "/opt/ros/humble/share/nav2_bt_navigator/behavior_trees/navigate_through_poses_w_replanning_and_recovery.xml"
    bt_loop_duration: 50
    default_server_timeout: 20
```

**验证**：

```bash
ros2 param get /bt_navigator default_nav_to_pose_bt_xml
ros2 topic echo /behavior_tree_log --once
```

日志中应出现实际加载的 XML 路径，且不再有 “Couldn't open XML”。

### 8.3 恢复行为参数整段失效（优先级 P0，安全相关）

**根因**：YAML 节点名 `recoveries_server` 与实际节点 `behavior_server` 不符；参数键应为 `behavior_plugins`（[behavior_server.cpp](file:///Users/waiyu/Downloads/src/navigation2/nav2_behaviors/src/behavior_server.cpp#L24-L48)）；插件类名已从 `nav2_recoveries/*` 改为 `nav2_behaviors/*`。当前恢复行为全部跑源码默认值，其中 `max_rotational_vel=1.0 rad/s` 是 TEB 上限（0.3）的 3 倍。

**补丁**：整段替换 `recoveries_server:` 为以下内容。

```yaml
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
    global_frame: odom
    robot_base_frame: base_link
    transform_tolerance: 0.1
    # 以下 4 项被插件以无前缀参数名读取（见 spin.cpp onConfigure）
    simulate_ahead_time: 2.0
    max_rotational_vel: 0.3
    min_rotational_vel: 0.05
    rotational_acc_lim: 0.3
```

**注意**：`max_rotational_vel` 必须不小于 `min_rotational_vel`，否则 `std::min(std::max(v, min), max)` 会恒等于上限，转向退化为定速。

**验证**：

```bash
ros2 lifecycle get /behavior_server         # 应为 active
ros2 param get /behavior_server behavior_plugins
ros2 param get /behavior_server max_rotational_vel
ros2 action send_goal /spin nav2_msgs/action/Spin "{target_yaw: 1.57}"
```

观察 `ros2 topic echo /cmd_vel` 的角速度不超过 0.3。

### 8.4 关键服务器缺少参数段（优先级 P2）

**根因**：YAML 中没有 `velocity_smoother`、`smoother_server` 段，两节点以默认值运行。`velocity_smoother` 默认 `max_velocity=[0.5,0,2.5]`、`max_accel=[2.5,0,3.2]`，与 TEB 的 `max_vel_theta=0.3`、`acc_lim_theta=0.3` 不一致；默认值“恰好能跑”，但角速度限幅实际由 TEB 单方面保证。

**补丁**：在 YAML 末尾追加。

```yaml
velocity_smoother:
  ros__parameters:
    use_sim_time: False
    smoothing_frequency: 20.0
    scale_velocities: false
    feedback: "OPEN_LOOP"
    max_velocity: [0.5, 0.0, 0.3]
    min_velocity: [-0.5, 0.0, -0.3]
    max_accel: [0.3, 0.0, 0.3]
    max_decel: [-0.3, 0.0, -0.3]
    odom_topic: odom
    odom_duration: 0.1
    deadband_velocity: [0.0, 0.0, 0.0]
    velocity_timeout: 1.0

smoother_server:
  ros__parameters:
    use_sim_time: False
    smoother_plugins: ["simple_smoother"]
    simple_smoother:
      plugin: "nav2_smoother::SimpleSmoother"
      tolerance: 1.0e-10
      max_its: 1000
      w_data: 0.2
      w_smooth: 0.3
```

**验证**：

```bash
ros2 param get /velocity_smoother max_velocity
ros2 param get /smoother_server simple_smoother.plugin
ros2 topic hz /cmd_vel          # 约 20 Hz
```

### 8.5 局部代价地图缺少膨胀层（优先级 P0，安全相关）

**根因**：[yhs_nav2.yaml](file:///Users/waiyu/Downloads/src/yhs_nav2/param/yhs_nav2.yaml#L251-L255) 的 `local_costmap.plugins` 只有 `static_layer`、`obstacle_layer`，下方 `inflation_layer` 配置因未列入 `plugins` 而不加载。局部避障只能依赖 TEB 的 `min_obstacle_dist=0.45`，而 `controller_server` 的碰撞检查、恢复行为都直接读取局部代价地图，风险偏高。

**补丁**：

```yaml
local_costmap:
  local_costmap:
    ros__parameters:
      # 原为 ["static_layer", "obstacle_layer"]
      plugins: ["static_layer", "obstacle_layer", "inflation_layer"]
      inflation_layer:
        plugin: "nav2_costmap_2d::InflationLayer"
        inflation_radius: 0.6      # 与 TEB inflation_dist 对齐
        cost_scaling_factor: 3.0
```

**验证**：RViz 中局部代价地图应出现由障碍向外递减的膨胀梯度；`ros2 topic echo /local_costmap/costmap --once` 不应再整片为 0/100。

### 8.6 `/scan1` 观测源长期为空（优先级 P2）

**根因**：两个 costmap 都配置 `observation_sources: scan scan1`，但总 launch 中相机与 `pointcloud_to_laserscan` 被注释（[yhs_nav2.launch.py](file:///Users/waiyu/Downloads/src/yhs_nav2/launch/yhs_nav2.launch.py#L133-L146)）。`obstacle_layer` 会持续等待该源，产生告警并浪费 TF 查询。

**方案 A（推荐，无相机时）**：删掉 `scan1`。

```yaml
obstacle_layer:
  plugin: "nav2_costmap_2d::ObstacleLayer"
  enabled: True
  observation_sources: scan
  scan:
    topic: /scan
    max_obstacle_height: 2.0
    clearing: True
    marking: True
    data_type: "LaserScan"
```

**方案 B（启用深度相机）**：同时打开三处配置：

1. `yhs_nav2.launch.py` 中取消 `astra.launch.py`（或按实际型号改 `gemini2.launch.py`）与 `pointcloud_to_laserscan.launch.py` 的注释。
2. [pointcloud_to_laserscan.launch.py](file:///Users/waiyu/Downloads/src/pointcloud_to_laserscan/launch/pointcloud_to_laserscan.launch.py#L20-L32) 中 `range_max` 由 `1.0` 改为 `3.0`（当前 1 m 对导航避障过短），`target_frame` 与相机 TF 保持一致。
3. 确认 `/camera/depth/points` 有数据：`ros2 topic hz /camera/depth/points`。

**验证**：`ros2 topic hz /scan1`；无数据时 costmap 日志不再刷 “The scan1 observation buffer has not been updated”。

### 8.7 实施顺序与回归清单

| 顺序 | 修改文件 | 关联问题 | 风险 | 验证 |
|---|---|---|---|---|
| 1 | `yhs_nav2.yaml` | 8.5 局部膨胀 | 低 | RViz 局部代价地图出现梯度 |
| 2 | `yhs_nav2.yaml` | 8.3 恢复行为 | 低 | `/spin` 角速度 ≤ 0.3 |
| 3 | `yhs_nav2.yaml` | 8.2 BT 参数 | 低 | `ros2 param get /bt_navigator default_nav_to_pose_bt_xml` |
| 4 | `yhs_nav2.yaml` | 8.4 平滑/速度参数 | 低 | `/cmd_vel` 20 Hz、角速度 ≤ 0.3 |
| 5 | `yhs_nav2.yaml` + `pointcloud_to_laserscan.launch.py` | 8.6 `scan1` | 低 | `ros2 topic hz /scan1` |
| 6 | `yhs_nav2.launch.py`（及两个子 launch） | 8.1 组合模式 | 中（改启动结构） | `ros2 component list` 或节点数 |

整体回归步骤：

```bash
# 1) 重新安装（YAML/launch 变更）
colcon build --symlink-install --packages-select yhs_nav2
source install/setup.bash

# 2) 启动
ros2 launch yhs_nav2 yhs_nav2.launch.py

# 3) 结构与参数检查
ros2 node list
ros2 lifecycle get /behavior_server
ros2 param get /velocity_smoother max_velocity
ros2 param get /bt_navigator default_nav_to_pose_bt_xml

# 4) 链路检查
ros2 topic hz /scan
ros2 topic hz /odom
ros2 topic hz /cmd_vel

# 5) 功能检查：RViz 设初始位姿 → 发导航目标 → 观察 /plan、/local_costmap/costmap、/behavior_tree_log
```

若回归中出现 “Timed out waiting for transform” 或 BT 不启动，优先检查 `map→odom→base_link` 链是否完整、`use_tf` 是否与当前模式（建图/导航）匹配。

---

### 附：推荐学习顺序

1. `yhs_can_interfaces` + `yhs_can_control` → 底盘 CAN 驱动与接口设计
2. `hardware/nvilidar_ros2`、`hardware/serial_imu` → 设备数据 → `/scan`、`/imu_data`
3. `yhs_nav2/launch/yhs_nav2.launch.py` → 顶层编排与组合启动
4. `yhs_nav2/param/yhs_nav2.yaml` → 全系统参数中心
5. `cartographer_ros/launch/dgt_mini.launch.py` + `dgt_mini.lua` → 建图输入输出与参数
6. `navigation2/nav2_amcl` → 定位（粒子滤波）
7. `navigation2/nav2_costmap_2d` → 代价地图
8. `navigation2/nav2_planner` + `nav2_navfn_planner` → 全局规划
9. `teb_local_planner` → 局部控制
10. `navigation2/nav2_bt_navigator` + `navigate_follow_record_path` → 行为树决策与定制

> 面向技师学院教学场景的课程化拆解（模块划分、学时、任务情景、验收标准与排故手册）见 [ROS2自动驾驶入门课程标准.md](file:///Users/waiyu/Downloads/src/ROS2自动驾驶入门课程标准.md)。
