# 05 · SLAM 建图

[← 上一章：硬件驱动](04-sensors-hardware.md) · [返回索引](README.md) · [下一章：Nav2 导航栈 →](06-navigation2.md)

---

## 1. 双方案对比

| 维度 | Cartographer（主） | GMapping（对照） |
| --- | --- | --- |
| 包 | `cartographer` + `cartographer_ros` | `slam_gmapping` + `openslam_gmapping` |
| 地图表示 | 子图 + 概率栅格，回环后位姿图优化 | 粒子滤波 + 扫描匹配，网格地图 |
| 输入 | 激光 + 里程计（+ 可选 IMU） | 激光 + `odom→base_link` TF |
| TF 输出 | `map→odom` 与 `odom→base_link`（可配） | `map→odom` |
| 大场景回环 | 强 | 弱 |
| 本项目入口 | [cartographer.launch.py](../../../src/yhs_nav2/launch/cartographer.launch.py) | [gmapping.launch.py](../../../src/yhs_nav2/launch/gmapping.launch.py) |

---

## 2. Cartographer 核心库

### 2.1 模块划分（`cartographer/cartographer/`）

| 模块 | 目录 | 职责 | 关键类型 |
| --- | --- | --- | --- |
| common | `common/` | 配置、Lua 参数、时间、任务、线程池、直方图 | `LuaParameterDictionary`、`ThreadPool`、`Task`、`Histogram`、`FixedRatioSampler` |
| sensor | `sensor/` | 传感器数据模型与收集器 | `PointCloud`、`RangeData`、`TimedPointCloudData`、`ImuData`、`OdometryData`、`LandmarkData`、`Collator` |
| transform | `transform/` | 刚体变换与插值 | `Rigid3d`、`TransformInterpolationBuffer`、`TimestampedTransform` |
| mapping | `mapping/` | 前端/后端/子图/位姿图 | `MapBuilder`、`TrajectoryBuilderInterface`、`PoseGraph`、`Submap2D`、`ProbabilityGrid`、`ImuTracker`、`PoseExtrapolator` |
| mapping/internal/2d | `mapping/internal/2d/` | 2D 前端与扫描匹配 | `LocalTrajectoryBuilder2D`、`CeresScanMatcher2D`、`RealTimeCorrelativeScanMatcher2D`、`RangeDataInserter2D` |
| mapping/internal/3d | `mapping/internal/3d/` | 3D 前端 | `LocalTrajectoryBuilder3D`、`CeresScanMatcher3D` |
| io | `io/` | 点云导出、proto 流、子图绘制 | `PointsBatch`、`ProtoStream`、`PcdWritingPointsProcessor`、`PlyWritingPointsProcessor`、`SubmapPainter` |
| ground_truth | `ground_truth/` | 真值关系与指标 | `RelationsTextFile`、`AutogenerateGroundTruth` |
| cloud | `cloud/` | gRPC 远程建图服务 | `MapBuilderServerInterface`、`MapBuilderServer`、`MapBuilderServerOptions` |
| metrics | `metrics/` | 运行时指标 | `Counter`、`Gauge`、`Histogram`、`FamilyFactory` |

### 2.2 关键类与函数

#### `mapping::MapBuilder`（[map_builder.h](../../../src/cartographer/cartographer/mapping/map_builder.h)）

门面类，持有 `PoseGraph` 与全部 `TrajectoryBuilder`。

| 方法 | 作用 |
| --- | --- |
| `MapBuilder(MapBuilderOptions, PoseGraphOptions)` | 构造并创建 2D/3D 轨迹构建器 |
| `AddTrajectoryBuilder(trajectory_id, expected_sensor_ids, callback)` | 注册一条轨迹，返回 `TrajectoryBuilderInterface::Ptr` |
| `FinishTrajectory(trajectory_id)` | 结束轨迹，触发最终优化 |
| `SubmapToProto(submap_id, SubmapQuery::Response*)` | 查询子图栅格/位姿（供 `submap_query` 服务） |
| `SerializeState()` / `LoadState()` | 保存/加载 `.pbstream` |
| `pose_graph()` | 访问位姿图 |
| `num_trajectory_nodes()` / `GetTrajectoryStates()` | 状态查询 |

#### `mapping::TrajectoryBuilderInterface`

| 方法 | 作用 |
| --- | --- |
| `AddSensorData(sensor_id, data)` | 多态入口：激光/点云/IMU/里程计/landmark |
| `AddLocalSlamResultCallback(callback)` | 每帧输出 `LocalSlamResultData`（位姿 + 子图更新） |
| `AddImuData/AddRangefinderData/AddOdometryData` | 便捷重载（内部转 `AddSensorData`） |

#### `mapping::PoseGraphInterface`

| 方法 | 作用 |
| --- | --- |
| `RunFinalOptimization()` | 回环优化收敛 |
| `GetAllSubmapPoses()` / `GetTrajectoryNodes()` | 地图与轨迹查询 |
| `GetLocalToGlobalTransform(trajectory_id)` | 前端→全局位姿 |
| `ComputeConstraintsForNode(...)` | 插入节点并构建约束 |
| `GetTrajectoryStates()` | `ACTIVE/FINISHED/FROZEN/DELETED` |

#### `sensor::Collator`

把不同频率的传感器数据按时间排序后成批送入前端（依据 `CollatedTrajectoryBuilder` 的 `expected_sensor_ids`）。

#### 2D 前端数据流

```
TimedPointCloudData ─┐
ImuData ─────────────┤
OdometryData ────────┴─► LocalTrajectoryBuilder2D::AddRangeData
                              ├─ PoseExtrapolator（IMU/odom 预测）
                              ├─ RealTimeCorrelativeScanMatcher2D（可选，在线相关匹配）
                              ├─ CeresScanMatcher2D（非线性优化）
                              ├─ MotionFilter（位移/角度阈值）
                              └─ RangeDataInserter2D（写入 Submap2D）
```

### 2.3 概率栅格与子图

- `Submap2D` 持有一个 `Grid2D`（默认 `ProbabilityGrid`），分辨率由 `TRAJECTORY_BUILDER_2D.submaps.grid_options_2d.resolution` 决定（`trajectory_builder.lua` 默认 0.05 m）。
- `ProbabilityGrid` 的 `free/occupied` 概率由 `probability_values.h` 中的查表转换。
- 子图完成条件：`num_range_data`（本项目 60 帧）或时间/内存限制。

---

## 3. `cartographer_ros` 桥接层

### 3.1 组件

| 类/文件 | 职责 |
| --- | --- |
| `Node`（[node.cpp](../../../src/cartographer_ros/cartographer_ros/src/node.cpp)） | 订阅 `scan/imu/odom/landmark`，调用 `MapBuilderBridge`，发布 TF 与可视化 |
| `MapBuilderBridge`（[map_builder_bridge.h](../../../src/cartographer_ros/cartographer_ros/include/cartographer_ros/map_builder_bridge.h)） | 封装 `MapBuilder`：`AddTrajectory`、`FinishTrajectory`、`SubmapToProto`、`HandleWriteState` |
| `SensorBridge` | `LaserScan/PointCloud2/Imu/Odometry` → Cartographer 数据类型 |
| `TFBridge` | 提供 `lookupTransform` 与坐标变换 |
| `OccupancyGridNode` | 定时把子图/轨迹栅格转成 `nav_msgs/OccupancyGrid` 发布 `/map` |
| `NodeOptions` / `TrajectoryOptions` | 解析 `-configuration_directory`、`-configuration_basename`、`load_state_filename` 等 |

### 3.2 可执行文件

| 可执行 | 作用 |
| --- | --- |
| `cartographer_node` | 在线建图主节点 |
| `cartographer_occupancy_grid_node` | 发布 `/map` |
| `cartographer_offline_node` | 回放 rosbag 离线建图 |
| `cartographer_assets_writer` | 导出点云（PCD/PLY）与 X-Ray |
| `cartographer_pbstream_to_ros_map` | `.pbstream` → PGM/YAML |
| `cartographer_pbstream_map_publisher` | 发布 pbstream 中的地图 |
| `cartographer_rosbag_validate` | bag 数据校验 |
| `cartographer_print_configuration` | 打印 Lua 合并后的完整配置（`print_configuration_main.cc`） |

### 3.3 ROS 接口

| 方向 | 名称 | 类型 |
| --- | --- | --- |
| 订阅 | `scan` | `sensor_msgs/LaserScan` |
| 订阅 | `imu`（remap 到 `imu_data`） | `sensor_msgs/Imu` |
| 订阅 | `odom` | `nav_msgs/Odometry` |
| 订阅 | `landmark_poses_list` | `cartographer_ros_msgs/LandmarkList` |
| 发布 | `submap_list` | `cartographer_ros_msgs/SubmapList` |
| 发布 | `trajectory_node_list` | `visualization_msgs/MarkerArray` |
| 发布 | `landmark_poses_list` | `visualization_msgs/MarkerArray` |
| 发布 | `constraint_list` | `visualization_msgs/MarkerArray` |
| 发布 | `scan_matched_points2` | `sensor_msgs/PointCloud2` |
| 发布 | TF `map→odom`（及可选 `odom→base_link`） | `tf2` |
| 服务 | `submap_query` / `trajectory_query` / `start_trajectory` / `finish_trajectory` / `write_state` / `get_trajectory_states` / `read_metrics` | `cartographer_ros_msgs/*` |

---

## 4. 本项目建图配置 `dgt_mini.lua`

文件：[dgt_mini.lua](../../../src/cartographer_ros/cartographer_ros/configuration_files/dgt_mini.lua)

| 配置项 | 值 | 影响 |
| --- | --- | --- |
| `map_frame` | `map` | 地图坐标系 |
| `tracking_frame` | `base_link` | 前端跟踪坐标系（**不是** `imu_link`） |
| `published_frame` | `base_link` | 发布 `map→base_link` 链路 |
| `odom_frame` | `odom` | 里程计坐标系 |
| `provide_odom_frame` | `true` | **由 Cartographer 发布 `odom→base_link`** |
| `use_odometry` | `true` | 使用 `/odom` 做预测 |
| `num_laser_scans` | `1` | 单线雷达 |
| `num_point_clouds` | `0` | 不直接用点云 |
| `submaps.num_range_data` | `60` | 每 60 帧生成一个子图 |
| `min_range` / `max_range` | 0.5 / 50 m | 过滤近处车体与远处噪声 |
| `missing_data_ray_length` | 5 m | 无回波时的射线长度 |
| `use_imu_data` | `false` | **IMU 不参与前端**（仅底盘用它算 yaw） |
| `use_online_correlative_scan_matching` | `true` | 提高旋转/快速运动鲁棒性 |
| `ceres_scan_matcher.translation_weight` | `1e-2` | 较低，更多信任激光 |
| `ceres_scan_matcher.rotation_weight` | `1e-1` | — |
| `optimize_every_n_nodes` | `5` | 高频优化 |
| `constraint_builder.min_score` | `0.34` | 回环阈值（偏低，易回环也易误匹配） |
| `motion_filter.max_angle_radians` | 0.1 rad | 角度变化超过 0.1 rad 才插入节点 |
| `motion_filter.max_distance_meters` | 0.2 m | 位移阈值 |
| `odometry_translation/rotation_weight` | `1e2 / 1e2` | 强里程计约束 |

### 4.1 启动文件 [dgt_mini.launch.py](../../../src/cartographer_ros/cartographer_ros/launch/dgt_mini.launch.py)

| 节点 | 参数 | remap |
| --- | --- | --- |
| `cartographer_node` | `use_sim_time=False` | `scan→scan`、`imu→imu_data`、`odom→odom` |
| `cartographer_occupancy_grid_node` | **`use_sim_time=True`**、`resolution=0.05` | — |

> ⚠ **严重配置问题**：`occupancy_grid_node` 使用仿真时间，而系统没有 `/clock` 发布者，ROS 时间恒为 0，定时器不会推进，`/map` 可能一直不发布。建图时若 RViz 只有子图没有栅格地图，优先检查此处（详见 [10 文档](10-known-issues.md)）。临时规避：把该参数改为 `False` 或通过命令行覆盖。

### 4.2 TF 关系

```
map ──(cartographer)──► odom ──(cartographer, provide_odom_frame)──► base_link
```

因此 [yhs_nav2/cartographer.launch.py](../../../src/yhs_nav2/launch/cartographer.launch.py) 默认 `use_tf:=false`，底盘节点不发布 `odom→base_link`。

---

## 5. 建图操作流程

```bash
# 1. 启动
ros2 launch yhs_nav2 cartographer.launch.py

# 2. 检查数据
ros2 topic hz /scan --qos-reliability best_effort
ros2 topic hz /odom
ros2 run tf2_ros tf2_echo map base_link

# 3. 遥控建图（另开终端）
ros2 run teleop_twist_keyboard teleop_twist_keyboard

# 4. 保存地图（两种方式）
# 4.1 纯栅格地图
ros2 run nav2_map_server map_saver_cli -f ~/maps/warehouse
# 4.2 保留 pbstream（可继续优化/重定位）
ros2 service call /write_state cartographer_ros_msgs/srv/WriteState \
  "{filename: '$HOME/maps/warehouse.pbstream', include_unfinished_submaps: true}"
```

产物：

| 文件 | 用途 |
| --- | --- |
| `warehouse.pgm` + `warehouse.yaml` | Nav2 `map_server` 加载 |
| `warehouse.pbstream` | Cartographer 状态（定位/继续建图） |

---

## 6. GMapping 方案

### 6.1 结构

```
slam_gmapping/
├── openslam_gmapping/    # 算法本体（grid/scanmatcher/particlefilter/log/utils）
└── slam_gmapping/        # ROS 2 封装：slam_gmapping.cpp + launch
```

核心类（封装层 `SlamGmapping`，源文件 [slam_gmapping.cpp](../../../src/slam_gmapping/slam_gmapping/src/slam_gmapping.cpp)）：

- 继承 `rclcpp::Node`，内部持有 `GMapping::GridSlamProcessor`
- `init()`：读取参数（`base_frame`、`odom_frame`、`map_frame`、`delta`、`particles`、`xmin/xmax/ymin/ymax` 等）、初始化激光缓存与地图
- `laserCallback()`：把 `LaserScan` 转成 `RangeReading`，调用 `processScan()`，发布 `map→odom` 与 `/map`
- `publishTransform()`：`map→odom` 由粒子最优位姿与里程计位姿计算
- `updateMap()`：栅格占据概率更新

### 6.2 启动

```bash
ros2 launch yhs_nav2 gmapping.launch.py
```

该 launch 会以 `use_tf:=true` 启动底盘（GMapping 需要 `odom→base_link`），并启动雷达、IMU、`slam_gmapping`。

### 6.3 与 Cartographer 的注意点

- GMapping 不做回环优化，长走廊/大场景易漂移。
- `use_sim_time` 必须与实际时钟一致。
- 若底盘 `use_tf=false`，GMapping 会因缺少 `odom→base_link` 直接不工作。
- 地图分辨率由 `delta` 决定（默认 0.05 m）。

---

## 7. 观测与调参

| 现象 | 优先检查 | 调整 |
| --- | --- | --- |
| 地图重影/墙体厚 | 雷达外参、时间同步、`max_range` | 校正 `laser_link` TF；减小 `max_range` |
| 回环错误 | `constraint_builder.min_score` | 提高到 0.5~0.55 |
| 快速转向丢定位 | 相关匹配、里程计权重 | 保持 `use_online_correlative_scan_matching=true` |
| 子图过多内存涨 | `num_range_data`、`optimize_every_n_nodes` | 增大子图帧数 / 降低优化频率 |
| `/map` 不发布 | `use_sim_time`、子图是否完成 | 修正 `use_sim_time=False` |
| 建图轨迹漂移 | 轮速标定、IMU yaw | 标定轮径/轮距；检查 IMU 安装方向 |
