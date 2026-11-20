# 13 · 调参手册

[← 上一章：代码地图](12-code-map.md) · [返回索引](README.md) · [下一章：二次开发 →](14-extension-guide.md)

---

本章基于仓库**当前参数值**给出可执行的调参路径。所有"现状"均来自源码/配置；"建议"是经验值，必须在实车小速度验证。

---

## 1. 调参原则

1. **一次只改一类参数**，改完立即录包（`ros2 bag record -a`）并记录地图/轨迹对比。
2. **先硬件后算法**：轮径/轮距、雷达外参、IMU 安装角、时间同步没对齐时，调算法只会掩盖问题。
3. **先建图后定位再导航**：地图质量差时，AMCL 与代价地图的调参没有意义。
4. 保留一份可回滚的基线参数（建议复制 `yhs_nav2.yaml` 为 `yhs_nav2.baseline.yaml`）。
5. 每次只验证一个指标：建图看闭环/重影，定位看 `amcl_pose` 抖动，导航看跟踪误差与恢复次数。

---

## 2. 速度链路与安全上限

当前链路：

```
TEB max_vel_x = 0.5 m/s, max_vel_theta = 0.3 rad/s
   └─► controller_server → /cmd_vel_nav
        └─► velocity_smoother（未配置，默认 max 0.5 m/s / 2.5 rad/s）
             └─► /cmd_vel_smoothed → /cmd_vel
                  └─► yhs_can_control → CAN
```

| 环节 | 当前限制 | 备注 |
| --- | --- | --- |
| TEB 线速度 | 0.5 m/s | [yhs_nav2.yaml:148](../../../src/yhs_nav2/param/yhs_nav2.yaml) |
| TEB 角速度 | 0.3 rad/s | 同上 |
| TEB 线/角加速度 | 0.3 / 0.3 | 起步与转向较柔和 |
| 速度平滑器 | 默认 0.5 m/s、2.5 rad/s | YAML 无 `velocity_smoother` 段 |
| 底盘 | 无软件限幅 | 依赖上层；调试时务必留急停 |

> 首次实车建议把 `max_vel_x` 降到 **0.15~0.2 m/s**，确认定位、避障、CAN 反馈都正常后再逐步提升。

---

## 3. 建图调参（Cartographer）

配置文件：[dgt_mini.lua](../../../src/cartographer_ros/cartographer_ros/configuration_files/dgt_mini.lua)

### 3.1 当前基线

| 参数 | 当前值 | 含义 |
| --- | --- | --- |
| `TRAJECTORY_BUILDER_2D.min_range` | 0.5 m | 忽略 0.5 m 内点（车体/盲区） |
| `max_range` | 50 m | 超过 50 m 丢弃 |
| `missing_data_ray_length` | 5 m | 无回波方向按 5 m 自由空间处理 |
| `submaps.num_range_data` | 60 | 每 60 帧生成一个子图 |
| `use_imu_data` | **false** | 2D 建图不直接使用 IMU 姿态 |
| `use_online_correlative_scan_matching` | true | 前端实时相关匹配 |
| `ceres_scan_matcher.occupied_space_weight` | 1 | 占据栅格权重 |
| `translation_weight` / `rotation_weight` | 1e-2 / 1e-1 | 较低，更信任激光 |
| `POSE_GRAPH.optimize_every_n_nodes` | 5 | 高频后端优化 |
| `constraint_builder.min_score` | 0.34 | 回环接受阈值（偏低） |
| `motion_filter.max_distance_meters` | 0.2 m | 位移超过 0.2 m 插节点 |
| `motion_filter.max_angle_radians` | 0.1 rad | 角度超过 0.1 rad 插节点 |
| `odometry_translation/rotation_weight` | 1e2 / 1e2 | 较强里程计约束 |

### 3.2 症状 → 参数

| 症状 | 优先调整 | 方向与说明 |
| --- | --- | --- |
| 墙面重影、同一面墙出现两层 | `motion_filter.max_distance_meters`、`max_angle_radians`；`ceres_scan_matcher` 权重 | 减小运动阈值（如 0.15 m / 0.05 rad）；适当提高 `occupied_space_weight` |
| 长走廊/大回环后地图错位 | `constraint_builder.min_score`、`optimize_every_n_nodes` | 提高 `min_score`（0.45~0.55）减少误回环；降低 `optimize_every_n_nodes` 频率以省 CPU，或保持 5 并观察 |
| 回环始终不触发 | `min_score`、`submaps.num_range_data` | 降低 `min_score`（0.30 左右）或减小子图帧数，让回环更早出现 |
| 建图漂移明显 | `odometry_*_weight`、`use_odometry`、轮径/轮距 | 先校验底盘里程计标定，再提高里程计权重；不要盲目关 `use_odometry` |
| 地图边缘出现"扇形拖尾" | `missing_data_ray_length`、`max_range` | 减小 `missing_data_ray_length`（2~3 m），或降低 `max_range` 到实际可用距离（如 30 m） |
| 建图 CPU 占用高 | `num_subdivisions_per_laser_scan`、`optimize_every_n_nodes`、`max_range` | 降低激光频率/范围；提高优化间隔 |
| 玻璃/镜面导致假墙 | `min_range`、`max_range`、`missing_data_ray_length` | 结合现场几何调整，必要时加反光板/改布局 |
| 机器人原地旋转时地图扭曲 | IMU/外参、`use_imu_data` | 先校准 `base_link→imu_link`；2D 下可尝试开启 `use_imu_data=true`（需要可靠 IMU） |

### 3.3 建图流程建议

1. 低速、匀速、少原地旋转；每段直线尽量长。
2. 每走完一个闭环，回到起点附近停顿 2~3 s，让后端完成优化。
3. 观察 `/submap_list` 与 RViz 子图是否闭合，再结束轨迹。
4. 保存：

```bash
ros2 service call /finish_trajectory cartographer_ros_msgs/srv/FinishTrajectory "{trajectory_id: 0}"
ros2 service call /write_state cartographer_ros_msgs/srv/WriteState "{filename: '/tmp/map.pbstream', include_unfinished_submaps: false}"
ros2 run cartographer_ros cartographer_pbstream_to_ros_map -pbstream_filename /tmp/map.pbstream -map_filestem /tmp/map
```

---

## 4. 定位调参（AMCL）

配置：[yhs_nav2.yaml:1-54](../../../src/yhs_nav2/param/yhs_nav2.yaml)

### 4.1 当前基线

| 参数 | 当前值 | 说明 |
| --- | --- | --- |
| `robot_model_type` | `DifferentialMotionModel` | 差速模型 |
| `max_particles` / `min_particles` | 5000 / 500 | 粒子规模 |
| `update_min_d` / `update_min_a` | 0.1 m / 0.2 rad | 更新较频繁，CPU 压力偏大 |
| `laser_max_range` | 48 m | 小于雷达 64 m |
| `max_beams` | 48 | 每次参与更新的光束数 |
| `laser_model_type` | `likelihood_field` | 似然场模型 |
| `transform_tolerance` | 1.0 s | 非常宽松，可能掩盖 TF 延迟 |
| `set_initial_pose` | **True** | 启动即用 `initial_pose`（0,0,0） |
| `initial_pose` | x=0, y=0, yaw=0 | 默认在原点 |
| `recovery_alpha_slow/fast` | 0.0 / 0.0 | 关闭随机重定位恢复 |
| `alpha1~4` | 0.1 | 里程计噪声 |

> `set_initial_pose=True` 意味着**每次启动都假设车在原点**。如果车不在原点，必须先发 `/initialpose`，否则 AMCL 会从错误位置开始收敛。

### 4.2 症状 → 参数

| 症状 | 优先调整 | 方向 |
| --- | --- | --- |
| 初始位置错误、粒子不收敛 | `set_initial_pose`、`initial_pose`、`/initialpose` | 关闭自动初值或发正确初值；必要时调用 `/reinitialize_global_localization` |
| 定位跳动/瞬移 | `update_min_d/a`、`max_beams`、`alpha1~4` | 增大更新阈值、降低更新频率；减小噪声参数 |
| 定位滞后于机器人 | `update_min_d/a`、`transform_tolerance` | 减小更新阈值；把 `transform_tolerance` 从 1.0 降到 0.2~0.5 |
| 长走廊/对称环境丢失 | `max_beams`、`laser_max_range`、`recovery_alpha_*` | 增大光束数/量程；开启 recovery（如 0.001/0.1） |
| CPU 占用高 | `max_particles`、`max_beams`、`update_min_d/a` | 降粒子数（2000）与光束数（30） |
| 地图与激光对不上 | 地图质量、`base_link→laser_link` | 先修地图/外参，不要用 AMCL 硬调 |
| `map→odom` 抖动 | `transform_tolerance`、TF 时间戳 | 检查雷达/里程计时间戳是否同源 |

---

## 5. 导航调参

### 5.1 代价地图

| 地图 | 当前配置 | 现状 |
| --- | --- | --- |
| `global_costmap` | 分辨率 0.05、尺寸 20×20、`plugins=[static, obstacle, inflation]`、`inflation_radius=0.55`、`cost_scaling=3.0` | 基本可用 |
| `local_costmap` | 滚动窗口 6×6、`plugins=[static, obstacle]`、配置了 `inflation_layer` 但**未加入 plugins** | 无膨胀代价（K3） |

| 症状 | 调整 |
| --- | --- |
| 贴墙/贴障碍物太近 | 把 `inflation_layer` 加入 `local_costmap.plugins`；适度增大 `inflation_radius` |
| 窄通道无法通过 | 减小 `inflation_radius`、`cost_scaling_factor`；核对 `footprint` |
| 障碍物被"抹掉" | 检查 `observation_sources` 的 `scan`/`scan1`、`raytrace_max_range`、`obstacle_max_range` |
| 全局路径绕远 | 检查静态地图、`allow_unknown=false`、膨胀半径 |
| 局部代价地图不更新 | 检查 `rolling_window=true`、`update_frequency`、TF |

### 5.2 全局规划（NavFn）

| 参数 | 当前值 | 说明 |
| --- | --- | --- |
| `plugin` | `nav2_navfn_planner/NavfnPlanner` | 当前使用 |
| `tolerance` | 0.5 m | 目标容差 |
| `use_astar` | true | A* |
| `allow_unknown` | false | 未知区域不可规划 |

建议：室内结构化场景保持 `allow_unknown=false`；若地图边界外需要探索，再改为 true。

### 5.3 局部控制（TEB）

当前关键参数：

| 参数 | 当前值 | 影响 |
| --- | --- | --- |
| `max_vel_x` | 0.5 m/s | 线速度上限 |
| `max_vel_theta` | 0.3 rad/s | 角速度上限 |
| `acc_lim_x` / `acc_lim_theta` | 0.3 / 0.3 | 加速度上限 |
| `dt_ref` / `dt_hysteresis` | 0.3 / 0.1 | 轨迹离散时间 |
| `max_samples` | 500 | 采样上限 |
| `global_plan_lookahead_dist` | 3.0 m | 全局路径前视 |
| `control_look_ahead_poses` | 2 | 控制前视点 |
| `min_obstacle_dist` | 0.45 m | 最小避障距离 |
| `inflation_dist` | 0.6 m | 软膨胀距离 |
| `footprint_model` | `circular`, r=0.1 | **与代价地图矩形 footprint 不一致** |
| `include_costmap_obstacles` | true | 使用代价地图障碍 |
| `dynamic_obstacle_inflation_dist` | 0.6 m | 动态障碍膨胀 |
| `costmap_converter_plugin` | `CostmapToPolygonsDBSMCCH` | 多边形障碍 |

| 症状 | 调整 |
| --- | --- |
| 转弯犹豫/来回摆 | 降低 `max_vel_theta` 或 `acc_lim_theta`；增大 `dt_ref`；检查 `min_obstacle_dist` 是否过大 |
| 贴边/刮蹭 | `footprint_model` 改为与底盘一致的 polygon；增大 `min_obstacle_dist` |
| 窄门过不去 | 减小 `min_obstacle_dist`、`inflation_dist`；确认 `footprint_model` 不夸大 |
| 起步猛打方向 | 启用/调整 `nav2_rotation_shim_controller` 或降低 `acc_lim_theta` |
| 轨迹抖动 | 增大 `dt_ref`、`dt_hysteresis`；降低 `max_samples`；检查代价地图更新频率 |
| 规划失败 | 检查 `global_plan_lookahead_dist`、代价地图膨胀、`feasibility_check_no_poses` |
| 避障过于保守 | 降低 `min_obstacle_dist` / `inflation_dist`，但不得小于车体半径+安全余量 |

> **优先修复**：`footprint_model` 圆形半径 0.1 m 与代价地图矩形 `0.6×0.52 m` 不一致（K7）。TEB 会低估车体，建议改为 polygon，或至少把半径设为包络半径（约 0.4 m）。

### 5.4 行为树与恢复

- 默认单目标 BT：`navigate_to_pose_w_replanning_and_recovery.xml`
- 示教路径 BT：`navigate_follow_record_path_w_replanning_and_recovery.xml`（内部使用 `FollowPath`，不是自定义 `NavigateFollowRecordPath` 节点）
- 当前 YAML 的 `recoveries_server` / `recovery_plugins` 为旧键，恢复参数不生效（K4）

| 症状 | 调整 |
| --- | --- |
| 卡住后不恢复 | 修 K4 的键名；确认 `/spin`、`/backup`、`/wait` 动作存在 |
| 频繁恢复 | 降低 `RateController` 频率压力；检查定位/代价地图；调整 `number_of_retries` |
| 恢复动作幅度过大 | 调整 BT XML 中 `spin_dist`、`backup_dist`、`backup_speed` |

---

## 6. 参数如何生效

| 方式 | 命令 | 适用 |
| --- | --- | --- |
| 重启 launch | `Ctrl+C` 后重新 `ros2 launch` | 大多数参数 |
| 生命周期重配置 | `ros2 lifecycle set /controller_server cleanup` → `configure` | 服务器参数 |
| 动态参数 | `ros2 param set /amcl max_beams 60` | 节点声明为动态的参数 |
| 修改 YAML | 编辑后重启对应 launch | 推荐、可追溯 |

查看参数：

```bash
ros2 param list /amcl
ros2 param get /controller_server FollowPath.max_vel_x
ros2 param dump /controller_server
```

---

## 7. 观测与验收指标

| 指标 | 工具 | 建议阈值 |
| --- | --- | --- |
| 建图闭环误差 | RViz 对比起终点、测量已知走廊长度 | < 0.1 m（室内小场景） |
| 定位抖动 | `/amcl_pose` 静止 30 s 的方差 | 位置 < 2 cm，yaw < 1° |
| 跟踪误差 | RViz 局部路径 vs 实际轨迹 | < 0.2 m（低速） |
| 恢复次数 | BT 日志 `number_recoveries` | 单次任务 0~1 次 |
| 控制频率 | `ros2 topic hz /cmd_vel` | 约 20 Hz |
| 雷达频率 | `ros2 topic hz /scan` | 与雷达规格一致 |
| CAN 反馈 | `ros2 topic hz /chassis_info_fb` | 稳定、无超时 |

---

## 8. 变更记录模板

```text
日期：
车型/场地：
改动文件：
改动参数（旧 → 新）：
目的：
结果（地图/定位/导航）：
是否回滚：
备注：
```

---

[← 上一章：代码地图](12-code-map.md) · [返回索引](README.md) · [下一章：二次开发 →](14-extension-guide.md)
