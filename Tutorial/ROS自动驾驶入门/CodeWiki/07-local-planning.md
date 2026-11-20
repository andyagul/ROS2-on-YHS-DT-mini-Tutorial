# 07 · 局部规划与依赖库

[← 上一章：Nav2](06-navigation2.md) · [返回索引](README.md) · [下一章：机器人模型 →](08-robot-model.md)

本章覆盖 Nav2 的三个关键依赖：**TEB 局部规划器**、**costmap_converter**、**BehaviorTree.CPP v3**。

---

## 1. TEB 局部规划器 `teb_local_planner` 0.9.1

### 1.1 定位

TEB = **Timed Elastic Band**（时间弹性带）。把局部轨迹表示为带时间戳的位姿序列，通过图优化同时最小化：

- 路径长度 / 运行时间
- 与障碍物的距离
- 速度、加速度、运动学约束
- 与全局路径的偏离（via-point）

在本项目以 `nav2_core::Controller` 插件形式接入，类名 `teb_local_planner::TebLocalPlannerROS`。

### 1.2 代码结构

```
teb_local_planner/teb_local_planner/
├── include/teb_local_planner/
│   ├── teb_local_planner_ros.h      # ROS 2 插件外壳
│   ├── teb_config.h                 # 全部参数
│   ├── timed_elastic_band.h         # 弹性带表示
│   ├── optimal_planner.h            # g2o 图优化
│   ├── homotopy_class_planner.h     # 多同伦类规划
│   ├── graph_search.h               # 同伦图搜索
│   ├── obstacles.h                  # 障碍物模型
│   ├── robot_footprint_model.h      # 点/圆/双圆/线/多边形
│   ├── recovery_behaviors.h         # 振荡/收缩恢复
│   └── visualization.h              # 可视化
└── src/
    ├── teb_local_planner_ros.cpp
    ├── teb_config.cpp
    ├── timed_elastic_band.cpp
    ├── optimal_planner.cpp
    ├── homotopy_class_planner.cpp
    ├── graph_search.cpp
    ├── obstacles.cpp
    ├── recovery_behaviors.cpp
    ├── visualization.cpp
    └── test_optim_node.cpp          # 独立优化测试节点
```

### 1.3 关键类

| 类 | 职责 |
| --- | --- |
| `TebLocalPlannerROS` | Nav2 插件入口：`configure/activate/deactivate/cleanup`、`setPlan`、`computeVelocityCommands`、`isGoalReached`、`setSpeedLimit` |
| `TebConfig` | 参数容器，从 ROS 参数树加载并校验 |
| `TimedElasticBand` | 位姿序列 + 时间差分，提供插入/删除/热启动 |
| `TebOptimalPlanner` | 构建 g2o 超图并求解，输出速度指令 |
| `HomotopyClassPlanner` | 生成多条同伦类候选轨迹并择优 |
| `ObstContainer` | 障碍物集合（点、线段、多边形） |
| `RobotFootprintModel` | `Point/ Circular/ TwoCircles/ Line/ Polygon` 碰撞模型 |
| `RecoveryBehavior` | `ShrinkHorizon`、`OscillationRecovery` |

### 1.4 单周期处理流程

```
computeVelocityCommands(pose, velocity)
  │
  ├─ 1. 取全局 plan，裁剪到局部范围（max_global_plan_lookahead_dist / prune_distance）
  ├─ 2. plan 变换到局部坐标系（odom / costmap frame）
  ├─ 3. 初始化/热启动 TimedElasticBand（teb_autosize、dt_ref）
  ├─ 4. 从 costmap 提取障碍（costmap_converter 多边形/线段）
  ├─ 5. 构建 g2o 超图：位姿顶点 + 时间差分 + 障碍/速度/加速度/运动学边
  ├─ 6. 多次内/外迭代求解（no_inner_iterations / no_outer_iterations）
  ├─ 7. 可行性检查（feasibility_check_no_poses）
  ├─ 8. 由前几个位姿差分出 v、ω（含 control_look_ahead_poses）
  └─ 9. 失败则触发恢复（shrink horizon / oscillation）
```

### 1.5 本仓库参数（[yhs_nav2.yaml:129-232](../../../src/yhs_nav2/param/yhs_nav2.yaml)）

| 类别 | 参数 | 值 | 说明 |
| --- | --- | --- | --- |
| 采样 | `dt_ref` / `dt_hysteresis` | 0.3 / 0.1 s | 轨迹时间分辨率 |
| | `max_samples` | 500 | 最大位姿数 |
| 全局路径 | `max_global_plan_lookahead_dist` | 3.0 m | 前视距离 |
| | `global_plan_prune_distance` | 1.0 m | 裁剪已走过部分 |
| | `global_plan_overwrite_orientation` | `true` | 用规划方向覆盖路径朝向 |
| 速度 | `max_vel_x` | 0.5 m/s | 最大线速度 |
| | `max_vel_theta` | 0.3 rad/s | 最大角速度 |
| | `acc_lim_x` / `acc_lim_theta` | 0.3 / 0.3 | 加速度限制 |
| | `min_turning_radius` | 0.0 | 差速可原地转 |
| 外形 | `footprint_model.type` | `circular` | 半径 0.1 m |
| 目标 | `xy_goal_tolerance` / `yaw_goal_tolerance` | 0.1 / 0.1 | 与 goal_checker 一致 |
| 障碍 | `min_obstacle_dist` | 0.45 m | 硬安全距离 |
| | `inflation_dist` | 0.6 m | 软膨胀距离 |
| | `include_costmap_obstacles` | `true` | 使用代价地图 |
| | `costmap_obstacles_behind_robot_dist` | 3.0 m | 后向感知范围 |
| 动态障碍 | `include_dynamic_obstacles` | `true` | — |
| 优化 | `no_inner/outer_iterations` | 5 / 4 | 求解迭代 |
| | `weight_obstacle` | 100 | 障碍权重 |
| | `weight_optimaltime` | 1.0 | 时间最优 |
| | `weight_kinematics_nh` | 1000 | 非完整约束 |
| | `weight_kinematics_forward_drive` | 1000 | 禁止倒车倾向 |
| | `weight_acc_lim_theta` | 10.5 | 角加速度 |
| 同伦 | `enable_homotopy_class_planning` | `false` | 关闭多类规划 |
| | `enable_multithreading` | `true` | 并行 |
| 恢复 | `shrink_horizon_backup` | `true` | 收缩时间窗 |
| | `oscillation_recovery` | `true` | 振荡检测 |

### 1.6 与底盘的匹配关系

- `max_vel_x=0.5`、`max_vel_theta=0.3` 是**保守值**；实车若动力更强，可逐步提高，同时观察 `/cmd_vel` 与 CAN 反馈是否饱和。
- `footprint_model` 用圆（r=0.1）而 costmap footprint 是 0.6×0.52 m 矩形 —— **两者不一致**。TEB 的碰撞检测会比 costmap 更乐观，建议改为 `polygon` 并填入与 costmap 一致的顶点。
- `min_obstacle_dist=0.45` 大于单线雷达安装高度覆盖范围时，矮障碍可能检测不到，需配合 `/scan1` 补盲。

### 1.7 独立调试

```bash
ros2 run teb_local_planner test_optim_node
```

该节点不依赖 Nav2，可在 RViz 中用 `Publish Point` 添加障碍，直观观察弹性带优化效果。

---

## 2. costmap_converter 0.1.2

### 2.1 作用

把栅格代价地图中的**离散占据栅格**转换为**几何障碍物**（点/线段/多边形），供 TEB 计算距离与碰撞。相比逐格计算，几何体更平滑、计算量更低。

### 2.2 包结构

```
dependence/costmap_converter/
├── costmap_converter/            # 插件实现
│   ├── include/costmap_converter/
│   └── src/
│       ├── costmap_to_polygons.cpp              # DBSMCCH
│       ├── costmap_to_polygons_concave.cpp      # DBSConcaveHull
│       ├── costmap_to_lines_convex_hull.cpp     # DBSMCCH 线段
│       ├── costmap_to_lines_ransac.cpp          # RANSAC 线段
│       └── costmap_to_dynamic_obstacles/        # 动态障碍跟踪
└── costmap_converter_msgs/       # ObstacleArrayMsg、ObstacleMsg
```

### 2.3 可用插件

| 插件类 | 输出 | 特点 |
| --- | --- | --- |
| `CostmapToPolygonsDBSMCCH` | 多边形 | **本项目使用**，DBSCAN + 凸包 |
| `CostmapToPolygonsDBSConcaveHull` | 凹多边形 | 更贴合 L 形障碍 |
| `CostmapToLinesDBSMCCH` | 线段集合 | 适合走廊 |
| `CostmapToLinesDBSRANSAC` | 线段集合 | 抗噪 |
| `CostmapToDynamicObstacles` | 动态障碍 | 需要额外跟踪参数 |

### 2.4 TEB 中的配置

```yaml
costmap_converter_plugin: "costmap_converter::CostmapToPolygonsDBSMCCH"
costmap_converter_spin_thread: True
costmap_converter_rate: 5
```

- `spin_thread: True`：转换器在独立线程以 5 Hz 运行，避免阻塞 20 Hz 控制循环。
- 转换器读取的是 **local_costmap**；因此局部代价地图缺少膨胀层会直接影响 TEB 的障碍距离场。

### 2.5 调参提示

| 问题 | 调整 |
| --- | --- |
| 多边形过于碎片化 | 增大 DBSCAN 聚类半径/最小点数 |
| 动态障碍抖动 | 降低 `costmap_converter_rate` 或改用 RANSAC |
| CPU 占用高 | 降低转换频率、缩小局部代价地图 |

---

## 3. BehaviorTree.CPP v3

### 3.1 定位

Nav2 的决策层引擎。行为树以 XML 描述，节点在运行时 tick，返回 `SUCCESS / FAILURE / RUNNING`。

目录：`dependence/BehaviorTree.CPP/`（`behaviortree_cpp_v3`）。

### 3.2 核心组件

| 组件 | 文件 | 作用 |
| --- | --- | --- |
| `BT::TreeNode` | `tree_node.cpp` | 所有节点基类：`tick()`、`status()`、端口 |
| `BT::BehaviorTreeFactory` | `bt_factory.cpp` | 注册节点类型，按 XML 构建树 |
| `BT::Blackboard` | `blackboard.cpp` | 键值共享内存（`{key}` 语法） |
| `BT::ControlNode` | `control_node.cpp` | Sequence / Fallback / Parallel / Reactive* |
| `BT::DecoratorNode` | `decorator_node.cpp` | Inverter / Retry / RateController / Timeout |
| `BT::ActionNodeBase` | `action_node.cpp` | 异步动作（Nav2 的绝大多数节点） |
| `BT::ConditionNode` | `condition_node.cpp` | 条件判断 |
| XML 解析 | `xml_parsing.cpp` | `<root>`、`<BehaviorTree>`、端口映射 |

### 3.3 节点类型与 Nav2 对应

| BT 类型 | Nav2 例子 | 作用 |
| --- | --- | --- |
| Action | `ComputePathToPose`、`FollowPath`、`Spin`、`BackUp` | 调用 ROS 动作 |
| Condition | `GoalReached`、`IsStuck`、`InitialPoseReceived` | 读取黑板/话题 |
| Control | `PipelineSequence`、`RecoveryNode`、`RoundRobin` | 编排 |
| Decorator | `RateController`、`DistanceController`、`SpeedController` | 节流/限速 |

### 3.4 默认导航树（`navigate_to_pose_w_replanning_and_recovery.xml`）

```
PipelineSequence
├── RateController(1 Hz) → ComputePathToPose
├── FollowPath
└── RecoveryNode
    ├── (失败时) ClearEntireCostmap → Spin → Wait → BackUp
    └── ...
```

实际 XML 位于 `navigation2/nav2_bt_navigator/behavior_trees/`，可通过 `default_nav_to_pose_bt_xml` 替换。

### 3.5 自定义节点接入步骤

1. 继承 `BT::ActionNodeBase`（或 `ConditionNode`），实现 `tick()`。
2. 用 `BT_REGISTER_NODES(factory)` 注册。
3. 在包 `CMakeLists.txt` 中编译为共享库。
4. 在 `bt_navigator.plugin_lib_names` 中加入库名。
5. 在 XML 中使用节点 ID。

本仓库的 `nav2_compute_follow_record_path_action_bt_node` 即按此方式扩展，库中注册的节点名为 `NavigateFollowRecordPath`（见 [06 文档 §6](06-navigation2.md)）。注意库名与节点名不一致，配置 `plugin_lib_names` 时要用**库名**。

### 3.6 可视化

`bt_navigator` 启用了 Groot 监控：

```yaml
enable_groot_monitoring: True
groot_zmq_publisher_port: 1666
groot_zmq_server_port: 1667
```

可用 [Groot](https://github.com/BehaviorTree/Groot) 连接 `localhost:1666` 实时查看树状态。

---

## 4. 三者协作关系

```
global_costmap ──► NavFn ──► /plan
                               │
local_costmap ──► costmap_converter ──► obstacles
                               │            │
                               ▼            ▼
                        TebLocalPlannerROS（20 Hz）
                               │
                        /cmd_vel_nav
                               │
                     velocity_smoother → /cmd_vel

bt_navigator（BehaviorTree.CPP）
   └─ 决定何时规划、何时跟随、何时恢复
```

- **BT 决定"做什么"**（规划/跟随/恢复）
- **NavFn 决定"走哪条全局路"**
- **TEB 决定"下一周期怎么走"**
- **costmap_converter 提供"障碍物几何"**
