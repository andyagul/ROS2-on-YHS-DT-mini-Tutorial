# 20 · 术语表

[← 上一章：架构决策](19-adr.md) · [返回索引](README.md)

> 面向首次接触本项目的工程师。每条给出简明定义 + 在本项目中的具体所指。

---

## 1. 缩写速查

| 缩写 | 全称 | 本项目语境 |
| --- | --- | --- |
| AMCL | Adaptive Monte Carlo Localization | 粒子滤波定位，`nav2_amcl` |
| BT | Behavior Tree | 导航流程编排，BehaviorTree.CPP v3 |
| CAN | Controller Area Network | 底盘通信总线，500 kbit/s |
| DDS | Data Distribution Service | ROS 2 中间件 |
| DWB | Dynamic Window Based | Nav2 备选局部规划器（未启用） |
| EKF | Extended Kalman Filter | 未使用，当前为轮速+IMU 直接融合 |
| IMU | Inertial Measurement Unit | 串口姿态模块，`/imu_data` |
| MCAP | MCAP 录包格式 | 可选 bag 存储格式 |
| NavFN | Navigation Function | 全局规划器，A*（`use_astar: true`） |
| PGM | Portable GrayMap | 占据栅格地图图像格式 |
| QoS | Quality of Service | DDS 通信策略（reliable/best effort 等） |
| RPP | Regulated Pure Pursuit | Nav2 备选控制器（未启用） |
| SLAM | Simultaneous Localization and Mapping | 建图与定位 |
| SMAC | Search-based Motion planning with A* | Nav2 备选全局规划器（未启用） |
| TEB | Timed Elastic Band | 局部规划器，`teb_local_planner` |
| TF | Transform | 坐标变换系统 |
| URDF | Unified Robot Description Format | 机器人模型描述 |
| YAML | YAML Ain't Markup Language | 参数/地图元数据格式 |

---

## 2. ROS 2 基础

| 术语 | 定义 | 在本项目中的例子 |
| --- | --- | --- |
| **节点 Node** | 独立进程中的计算单元 | `yhs_can_control_node`、`amcl` |
| **话题 Topic** | 异步发布/订阅通道 | `/scan`、`/odom`、`/cmd_vel` |
| **服务 Service** | 同步请求/响应 | `/write_state`、`/map_server/load_map` |
| **动作 Action** | 长耗时、可反馈、可取消的任务 | `/navigate_to_pose`、`/follow_path` |
| **参数 Parameter** | 节点运行时配置 | `yhs_nav2.yaml` 中的 `max_vel_x` |
| **QoS** | 可靠性、持久性、队列深度等策略 | 激光用 Best Effort；地图用 Transient Local |
| **生命周期 Lifecycle** | `unconfigured→inactive→active→finalized` 状态机 | Nav2 各服务器由 `lifecycle_manager` 拉起 |
| **组合 Composition** | 多个节点装入同一进程 | 本项目声明但未真正使用（K8） |
| **命名空间 Namespace** | 话题/服务前缀 | `namespace:=robot1` → `/robot1/scan` |
| **重映射 Remap** | 运行时改话题名 | `cmd_vel_smoothed → cmd_vel` |
| **Launch** | 启动编排脚本 | `yhs_nav2.launch.py` |

---

## 3. TF 与坐标系

| 术语 | 定义 | 本项目取值 |
| --- | --- | --- |
| **frame** | 带名字的坐标系 | `map`、`odom`、`base_link`、`laser_link`、`imu_link` |
| **静态 TF** | 不随时间变化的变换 | `base_link→laser_link`（z=0.33） |
| **动态 TF** | 随时间广播的变换 | `odom→base_link`、`map→odom` |
| **`map`** | 全局固定坐标系 | 地图原点由 YAML `origin` 定义 |
| **`odom`** | 连续、无跳变的局部坐标系 | 由底盘/Cartographer 发布 |
| **`base_link`** | 车体中心 | 底盘与传感器安装基准 |
| **`laser_link`** | 雷达坐标系 | 与 `base_link` 差 0.33 m |
| **`imu_link`** | IMU 坐标系 | 当前与 `base_link` 重合 |
| **Fixed Frame** | RViz 显示基准 | 导航时应设为 `map` |
| **TF 树** | 坐标系父子关系 | `map→odom→base_link→laser_link/imu_link` |

> **易混**：`map→odom` 由定位（AMCL）发布，用于修正累积误差；`odom→base_link` 由里程计发布，必须连续。两者不能由同一来源重复发布，否则 TF 跳变。

---

## 4. SLAM 与地图

| 术语 | 定义 | 本项目 |
| --- | --- | --- |
| **scan matching** | 用激光扫描匹配估计位姿 | Cartographer 前端 |
| **submap** | 局部子地图 | Cartographer 的核心单元 |
| **pose graph** | 位姿图，节点为位姿、边为约束 | 后端优化 |
| **loop closure** | 回环检测，消除累积漂移 | Cartographer 自动回环 |
| **constraint** | 位姿图约束 | 回环/里程计约束 |
| **pbstream** | Cartographer 的二进制地图+轨迹格式 | `/write_state` 保存 |
| **occupancy grid** | 占据栅格（0~100） | `/map` 的 `nav_msgs/OccupancyGrid` |
| **trinary** | 三态地图（自由/占据/未知） | `zgq-g.yaml` 的 `mode` |
| **resolution** | 每像素米数 | 0.05 m/px |
| **origin** | 地图左下角在世界中的位姿 | `[-1.07, -8.36, 0]` |
| **GMapping** | 基于粒子滤波的 2D SLAM | 备选建图方案 |

---

## 5. Nav2 导航

| 术语 | 定义 | 本项目配置 |
| --- | --- | --- |
| **server** | 生命周期管理的能力节点 | planner/controller/behavior/smoother/velocity_smoother |
| **plugin** | 由 `pluginlib` 动态加载的算法实现 | `TebLocalPlannerROS`、`NavfnPlanner` |
| **costmap** | 代价地图，供规划/控制使用 | 全局 + 局部两层 |
| **static layer** | 叠加静态地图 | 全局/局部均启用 |
| **obstacle layer** | 叠加实时障碍 | 订阅 `/scan`、`/scan1` |
| **inflation layer** | 按距离膨胀障碍代价 | 全局启用；局部**未加入 plugins**（K3） |
| **voxel layer** | 3D 体素障碍 | 未启用 |
| **footprint** | 车体多边形轮廓 | `[[0.30,0.26],...]` |
| **robot_radius** | 圆形车体半径 | 与 footprint 二选一 |
| **inflation_radius** | 膨胀最大半径 | 全局 0.55 m；局部配置 1.0 m 但未启用（K3） |
| **lethal / inscribed** | 致命代价 / 内切代价 | 膨胀层输出 |
| **global planner** | 全局路径规划 | NavFN（A*） |
| **local controller** | 局部轨迹跟踪与避障 | TEB |
| **smoother** | 路径平滑 | `nav2_smoother`（SimpleSmoother） |
| **behavior server** | 恢复行为（spin/backup/wait） | 参数键仍为旧版（K4） |
| **velocity smoother** | 速度限幅与平滑 | `nav2_velocity_smoother` |
| **collision monitor** | 独立碰撞监控 | 已启动，参数为默认值 |
| **BT navigator** | 行为树导航入口 | `navigate_to_pose`、`navigate_through_poses` |
| **blackboard** | BT 节点间共享数据 | 路径、目标、错误码 |
| **recovery** | 失败后的恢复分支 | 清代价地图、旋转、后退 |

---

## 6. 底盘与控制

| 术语 | 定义 | 本项目 |
| --- | --- | --- |
| **差速驱动** | 左右轮独立驱动 | 全部车型 |
| **轮径** | 驱动轮直径 | 用于线速度→rpm 换算 |
| **减速比** | 电机→车轮的传动比 | 参与 rpm 换算 |
| **CAN ID** | 帧标识符（含优先级） | `0x98C4D1EF` 等 |
| **DLC** | 数据长度码（0~8 字节） | 协议均为 8 字节 |
| **小端序** | 低字节在前 | 速度/电流字段 |
| **使能** | 电机/底盘动力开关 | `io_cmd` 的 `enable` |
| **挡位** | 前进/空挡/后退 | 当前固定写 3（K13） |
| **里程计 odom** | 由轮速/IMU 推算的位姿与速度 | `/odom` |
| **协方差** | 位姿/速度不确定性 | 当前为固定小值，未建模 |

---

## 7. 项目专有名词

| 术语 | 含义 |
| --- | --- |
| **YHS** | 项目/厂商前缀，出现在包名与话题中 |
| **dgt_mini** | Cartographer 专用 Lua 配置名，对应小型差速车 |
| **ecarroom** | 默认地图名（`ecarroom.yaml`） |
| **示教路径** | 人工驾驶录制 `/record_path`，之后自动复现 |
| **补盲** | 用深度相机转 `/scan1` 补足低矮/近距障碍 |
| **`use_tf`** | 底盘是否广播 `odom→base_link`；建图时必须 `false` |
| **`scan1`** | 深度相机点云转出的辅助激光 |
| **`chassis_info_fb`** | 底盘综合反馈（电压、电流、温度、模式等） |
| **`io_cmd` / `free_ctrl_cmd`** | 灯光/使能等 IO 控制与自由控制指令 |

---

## 8. 单位与符号约定

| 量 | 单位 | 说明 |
| --- | --- | --- |
| 长度 | m | 地图分辨率、TF 平移 |
| 角度 | rad | ROS 消息与 TF |
| 角度（配置） | deg | 雷达量程、部分 YAML 参数 |
| 线速度 | m/s | `Twist.linear.x` |
| 角速度 | rad/s | `Twist.angular.z` |
| 电机转速 | rpm | CAN 协议字段 |
| 时间 | s（Float64） / ns（Header） | 注意换算 |
| 代价 | 0~255 | 代价地图；254/255 为致命/内切 |

---

## 9. 最易混淆的五组概念

| 组 | 区别 |
| --- | --- |
| `map` vs `odom` | `map` 会因定位修正跳变；`odom` 必须连续 |
| 静态 TF vs 动态 TF | 静态只发一次（`/tf_static`）；动态持续广播（`/tf`） |
| 全局代价地图 vs 局部代价地图 | 全局覆盖整张地图、更新慢；局部滚动窗口、更新快 |
| planner vs controller | planner 给路径；controller 给速度指令 |
| behavior server vs BT | behavior 是"能力"；BT 决定"何时调用哪个能力" |
