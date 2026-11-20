# 06 · Nav2 导航栈

[← 上一章：SLAM](05-slam.md) · [返回索引](README.md) · [下一章：局部规划与依赖 →](07-local-planning.md)

---

## 1. 版本与位置

- 源码：[navigation2/](../../src/navigation2)，**Nav2 1.1.5**（Humble 分支）
- 参数单点：[yhs_nav2/param/yhs_nav2.yaml](../../src/yhs_nav2/param/yhs_nav2.yaml)
- 编排入口：[yhs_nav2/launch/yhs_nav2.launch.py](../../src/yhs_nav2/launch/yhs_nav2.launch.py)

Nav2 采用「**服务器 + 插件 + 行为树**」架构：每个服务器是生命周期节点，算法以插件形式加载，行为树负责编排。

```
                        ┌──────────────────┐
                        │   bt_navigator   │
                        └────────┬─────────┘
              ┌──────────────────┼──────────────────┐
              ▼                  ▼                  ▼
      planner_server      controller_server    behavior_server
        (NavFn)               (TEB)          spin/backup/wait
              │                  │                  │
              └────────┬─────────┴──────────────────┘
                       ▼
                 costmap_2d（global/local）
                       ▲
          map_server ──┤
             amcl ─────┘
```

---

## 2. 生命周期与启动

### 2.1 节点分组

| 管理器 | 管理节点 |
| --- | --- |
| `lifecycle_manager_localization` | `map_server`、`amcl` |
| `lifecycle_manager_navigation` | `controller_server`、`smoother_server`、`planner_server`、`behavior_server`、`bt_navigator`、`waypoint_follower`、`velocity_smoother` |

启动顺序：

```
configure → activate（逐个节点，bond 存活监测）
```

`autostart:=true`（默认）由 `lifecycle_manager` 自动完成。任一节点 configure 失败，后续节点不会激活。

### 2.2 组合模式现状

`yhs_nav2.launch.py:110-111` 会在 `use_composition:=True`（默认）时启动 `nav2_container`，但 `localization_launch.py` 与 `navigation_launch.py` 内部**始终用独立 `Node` 启动**，没有 `LoadComposableNodes`。结果：**容器为空**，`use_composition` 不改变实际部署。功能不受影响，但"以为在组合"会误导性能分析。

### 2.3 launch 参数

[yhs_nav2.launch.py](../../src/yhs_nav2/launch/yhs_nav2.launch.py)：

| 参数 | 默认 | 说明 |
| --- | --- | --- |
| `use_namespace` | `False` | 是否加命名空间（只对 `PushRosNamespace` 生效） |
| `map` | `yhs_nav2/map/ecarroom.yaml` | 地图 YAML |
| `use_sim_time` | `False` | 仿真时间 |
| `params_file` | `yhs_nav2/param/yhs_nav2.yaml` | 参数文件 |
| `autostart` | `True` | 自动状态迁移 |
| `use_composition` | `True` | 仅启动空容器（见上） |
| `use_respawn` | `False` | 崩溃重启 |
| `use_tf` | `True` | 传给底盘，控制 `odom→base_link` |

---

## 3. 服务器与关键类

### 3.1 `map_server`

- 包：`nav2_map_server`
- 类：`nav2_map_server::MapServer`（`LifecycleNode`）
- 入口：`map_server_main.cpp`；保存工具：`map_saver_cli`
- 关键函数：`loadMap()`（解析 YAML → `nav_msgs/OccupancyGrid`）、`mapCallback()`、`on_configure/on_activate`
- 参数：`yaml_filename` 由 launch 的 `map` 参数通过 `RewrittenYaml` 注入；YAML 中 `map.yaml` 只是占位默认值。
- 服务：`load_map`、`save_map`（`nav2_msgs/srv/*`）。

### 3.2 `nav2_amcl`（定位）

- 类：`nav2_amcl::AmclNode`（[amcl_node.hpp](../../src/navigation2/nav2_amcl/include/nav2_amcl/amcl_node.hpp)）
- 关键函数：

| 函数 | 作用 |
| --- | --- |
| `mapReceived()` | 接收 `/map`，初始化 `Map` |
| `laserReceived()` | 处理 `/scan`，按 `update_min_d/a` 决定是否更新 |
| `initialPoseReceived()` | 处理 `/initialpose`，重置粒子集 |
| `updateFilter()` | 运动更新 + 观测更新 + 重采样 |
| `publishAmclPose()` | 发布 `/amcl_pose` |
| `publishParticleCloud()` | 发布 `/particlecloud` |
| `sendMapToOdomTransform()` | 发布 `map→odom`（`tf_broadcast=true`） |

- 参数（[yhs_nav2.yaml:1-54](../../src/yhs_nav2/param/yhs_nav2.yaml)）：

| 参数 | 值 | 说明 |
| --- | --- | --- |
| `alpha1-4` / `alpha5` | 0.1 / 0 | 里程计噪声（差速模型用 1-4） |
| `robot_model_type` | `nav2_amcl::DifferentialMotionModel` | 差速 |
| `laser_model_type` | `likelihood_field` | 似然场 |
| `laser_max_range/min_range` | 48 / 0.5 | 与雷达量程匹配 |
| `max_beams` | 48 | 每帧使用光束数 |
| `max_particles/min_particles` | 5000 / 500 | 粒子数 |
| `update_min_d/a` | 0.1 m / 0.2 rad | 更新阈值 |
| `transform_tolerance` | 1.0 s | TF 容差 |
| `tf_broadcast` | `true` | 发布 `map→odom` |
| `set_initial_pose` | `True` | 启动即使用 `initial_pose`（0,0,0） |
| `scan_topic` | `scan` | 输入激光 |

> `set_initial_pose=True` 意味着 AMCL 启动后默认认为车在 `(0,0,0)`。实车若不在原点，必须通过 RViz **2D Pose Estimate** 或 [set_init_amcl_pose.py](../../src/yhs_nav2/scripts/set_init_amcl_pose.py) 重设，否则会出现"路径从地图外开始"。

### 3.3 `planner_server`

- 类：`nav2_planner::PlannerServer`
- 关键函数：`computePlan()`、`computePlanThroughPoses()`、`publishPlan()`、`transformPosesToGlobalFrame()`、`validatePath()`
- 插件：`planner_plugins: ["GridBased"]` → `nav2_navfn_planner/NavfnPlanner`
- 动作：`compute_path_to_pose`、`compute_path_through_poses`
- 参数：

```yaml
GridBased:
  plugin: "nav2_navfn_planner/NavfnPlanner"
  tolerance: 0.5
  use_astar: true
  allow_unknown: false
expected_planner_frequency: 20.0
```

> `allow_unknown: false` 表示未知区域不可通行；`global_costmap.track_unknown_space: true` 与之一致。

### 3.4 `controller_server`

- 类：`nav2_controller::ControllerServer`
- 关键函数：`setPlannerPath()`（接收 `/plan`）、`computeControl()`（20 Hz）、`computeAndPublishVelocity()`、`publishVelocity()`、`publishZeroVelocity()`、`isGoalReached()`
- 动作：`follow_path`
- 插件：
  - `progress_checker`: `nav2_controller::SimpleProgressChecker`（0.5 m / 10 s）
  - `goal_checker`: `nav2_controller::SimpleGoalChecker`（xy 0.1 m、yaw 0.1 rad）
  - `controller`: `teb_local_planner::TebLocalPlannerROS`
- 输出：`/cmd_vel_nav`（launch remap `cmd_vel→cmd_vel_nav`）

### 3.5 `behavior_server`

- 类：`nav2_behaviors::BehaviorServer`
- 默认插件：`spin`、`backup`、`drive_on_heading`、`wait`（参数名 `behavior_plugins`）
- 动作：`/spin`、`/backup`、`/drive_on_heading`、`/wait`
- **本仓库 YAML 写的是旧版 `recoveries_server` + `recovery_plugins` + `nav2_recoveries/*`**，与 1.1.5 的 `behavior_server` 参数名不匹配 → 自定义恢复参数全部被忽略，服务器回退到内置默认值（功能可用，参数不生效）。

### 3.6 `bt_navigator`

- 类：`nav2_bt_navigator::BtNavigator`
- 导航器：
  - `NavigateToPoseNavigator` → `navigate_to_pose`
  - `NavigateThroughPosesNavigator` → `navigate_through_poses`
  - **`NavigateFollowRecordPathNavigator`（自研）** → `navigate_follow_record_path`
- 行为树文件：
  - 默认：`nav2_bt_navigator/behavior_trees/navigate_to_pose_w_replanning_and_recovery.xml`
  - 自研：`navigate_follow_record_path_w_replanning_and_recovery.xml`
- 参数：`plugin_lib_names` 加载 BT 节点库；`default_bt_xml_filename` 在 1.1.5 **已废弃**，实际读取 `default_nav_to_pose_bt_xml`（默认已内置默认树，配置中的旧键被忽略）。

### 3.7 其他

| 服务器 | 类 | 作用 |
| --- | --- | --- |
| `smoother_server` | `nav2_smoother::SmootherServer` | 路径平滑（本仓库未配置插件） |
| `velocity_smoother` | `nav2_velocity_smoother::VelocitySmoother` | `/cmd_vel_nav → /cmd_vel`，默认限幅 |
| `waypoint_follower` | `nav2_waypoint_follower::WaypointFollower` | 多点巡航，`wait_at_waypoint` 插件 |
| `nav2_lifecycle_manager` | `LifecycleManager` | 状态迁移 + bond |

---

## 4. 速度指令链路

```
controller_server ──/cmd_vel_nav──► velocity_smoother ──/cmd_vel──► yhs_can_control ──► CAN
```

- `navigation_launch.py:93`：`controller_server` remap `cmd_vel → cmd_vel_nav`
- `navigation_launch.py:154`：`velocity_smoother` remap `cmd_vel → cmd_vel_nav`、`cmd_vel_smoothed → cmd_vel`
- 由于 `velocity_smoother` 未在 YAML 配置，使用默认最大速度 `[0.5, 0, 2.5]`、最小 `[-0.5, 0, -2.5]`。若 TEB 的 `max_vel_x=0.5` 已足够，无需额外配置；如需更严格限速，应在 YAML 增加 `velocity_smoother` 段。

---

## 5. 代价地图

### 5.1 局部 `local_costmap`

| 项 | 值 |
| --- | --- |
| `global_frame` | `odom` |
| `rolling_window` | `true` |
| 尺寸 | 6 m × 6 m |
| 分辨率 | 0.05 m |
| `footprint` | `[[0.30,0.26],[0.30,-0.26],[-0.30,-0.26],[-0.30,0.26]]`（0.6 × 0.52 m） |
| `plugins` | `["static_layer","obstacle_layer"]` ⚠ **缺 inflation_layer** |
| 观测源 | `/scan`、`/scan1` |
| 更新/发布 | 5 Hz / 2 Hz |

> 配置里写了 `inflation_layer` 段，但**没有加入 `plugins` 列表**，因此局部代价地图不会生成膨胀代价。TEB 的 `min_obstacle_dist=0.45` 仍能避障，但"软约束"缺失，贴边行驶会更激进。这是最值得优先修的一处。

### 5.2 全局 `global_costmap`

| 项 | 值 |
| --- | --- |
| `global_frame` | `map` |
| `track_unknown_space` | `true` |
| 分辨率 | 0.05 m |
| `plugins` | `static` + `obstacle` + `inflation` |
| 膨胀半径 / 代价系数 | 0.55 m / 3.0 |
| 观测源 | `/scan`（raytrace 3 m / obstacle 2.5 m）、`/scan1`（同上） |

> `/scan1` 实际由 `pointcloud_to_laserscan` 产生（默认 `range_max=1.0`）。相机 launch 当前被注释，因此 costmap 会持续警告"scan1 无数据"，属**非致命**告警。

### 5.3 代价地图分层

| 层 | 类 | 作用 |
| --- | --- | --- |
| StaticLayer | `nav2_costmap_2d::StaticLayer` | 订阅 `/map`，静态障碍 |
| ObstacleLayer | `nav2_costmap_2d::ObstacleLayer` | 激光/点云 marking & clearing |
| InflationLayer | `nav2_costmap_2d::InflationLayer` | 按距离衰减膨胀 |
| VoxelLayer | `nav2_costmap_2d::VoxelLayer` | 3D 体素（本仓库未用） |

`Costmap2DROS` 管理 `LayeredCostmap`，按 `update_frequency` 更新、`publish_frequency` 发布 `/costmap` 与 `published_footprint`。

---

## 6. 自研：示教循迹（Follow Record Path）

这是本仓库对 Nav2 的一处**功能扩展**，链路完整：

| 层 | 文件 | 作用 |
| --- | --- | --- |
| 消息 | [RecordPath.action](../../src/yhs_can_interfaces/action/RecordPath.action) | Goal 携带 `nav_msgs/Path` 与 `behavior_tree` |
| RViz | [nav2_panel.cpp](../../src/navigation2/nav2_rviz_plugins/src/nav2_panel.cpp) | 面板增加"录制/回放路径"按钮 |
| BT 节点 | [navigate_follow_record_path_action.cpp](../../src/navigation2/nav2_behavior_tree/plugins/action/navigate_follow_record_path_action.cpp) | 发送 `navigate_follow_record_path` 动作 |
| 导航器 | [navigate_follow_record_path.cpp](../../src/navigation2/nav2_bt_navigator/src/navigators/navigate_follow_record_path.cpp) | 加载专用行为树并执行 |
| 行为树 | `nav2_bt_navigator/behavior_trees/navigate_follow_record_path_w_replanning_and_recovery.xml` | 沿录制的 `Path` 逐点 FollowPath |
| 插件注册 | `nav2_behavior_tree/CMakeLists.txt`、`nav2_bt_navigator/CMakeLists.txt` | 编译并导出 BT 节点 |

> 因为 `nav2_behavior_tree` 依赖 `yhs_can_interfaces`，**必须先编译 `yhs_can_interfaces`**，否则 Nav2 编译失败。

---

## 7. 参数 ↔ 节点对照

| YAML 段 | 实际节点 | 备注 |
| --- | --- | --- |
| `amcl` | `amcl` | 生效 |
| `bt_navigator` | `bt_navigator` | `default_bt_xml_filename` 无效 |
| `controller_server` | `controller_server` | 生效 |
| `local_costmap.local_costmap` | `local_costmap` | 缺 inflation 插件 |
| `global_costmap.global_costmap` | `global_costmap` | 生效 |
| `map_server` | `map_server` | `yaml_filename` 被 launch 覆盖 |
| `planner_server` | `planner_server` | 生效 |
| `recoveries_server` | **无此节点** | 应为 `behavior_server` + `behavior_plugins` |
| `robot_state_publisher` | `robot_state_publisher` | 若底盘描述 launch 未启用则无节点 |
| `waypoint_follower` | `waypoint_follower` | 生效 |
| （缺失）`velocity_smoother` | `velocity_smoother` | 用默认参数 |
| （缺失）`smoother_server` | `smoother_server` | 用默认参数，且无插件 |

---

## 8. 调试命令

```bash
# 生命周期状态
ros2 lifecycle get /amcl
ros2 lifecycle get /bt_navigator

# 动作
ros2 action list
ros2 action send_goal /navigate_to_pose nav2_msgs/action/NavigateToPose \
  "{pose: {header: {frame_id: map}, pose: {position: {x: 1.0, y: 0.0}, orientation: {w: 1.0}}}}" --feedback

# 代价地图
ros2 topic echo /local_costmap/costmap --once
ros2 topic echo /global_costmap/costmap --once

# 规划/控制链路
ros2 topic echo /plan --once
ros2 topic hz /cmd_vel_nav
ros2 topic hz /cmd_vel

# BT 日志
ros2 param get /bt_navigator default_nav_to_pose_bt_xml
```

---

## 9. 常见故障定位

| 现象 | 可能原因 | 检查 |
| --- | --- | --- |
| 车不动，`/cmd_vel` 无数据 | 生命周期未激活 / BT 失败 | `ros2 lifecycle get`、`/bt_navigator` 日志 |
| 车原地转圈 | 目标 yaw 容差过小 / 定位跳变 | `amcl` 粒子、`/amcl_pose` |
| 规划失败 | `allow_unknown=false` 且目标在未知区 | 地图是否完整、目标点 |
| 局部避障差 | local costmap 无膨胀层 | `plugins` 列表 |
| 到点不结束 | `xy_goal_tolerance` 0.1 过严 | 放宽到 0.15~0.25 |
| `/scan1` 告警 | 相机未启动 | 注释掉的相机 launch |
| 导航起点漂移 | AMCL 初始位姿错误 | 重发 `/initialpose` |
