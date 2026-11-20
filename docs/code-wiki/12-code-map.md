# 12 · 文件级代码地图

[← 上一章：接口参考](11-interface-reference.md) · [返回索引](README.md) · [下一章：调参手册 →](13-tuning-playbook.md)

---

本章回答"**改某个功能应该看哪个文件**"。仓库共 63 个包、约 1605 个 C/C++ 文件，这里只列与本车业务链路直接相关的入口、核心实现与可扩展点；Nav2/Cartographer 的完整清单遵循其上游目录结构。

---

## 1. 编排层 `yhs_nav2`

| 文件 | 作用 | 关键内容 |
| --- | --- | --- |
| [yhs_nav2.launch.py](../../src/yhs_nav2/launch/yhs_nav2.launch.py) | **总入口** | 参数声明、地图默认值、生命周期管理器、组合容器、底盘/雷达/IMU/定位/导航编排 |
| [cartographer.launch.py](../../src/yhs_nav2/launch/cartographer.launch.py) | 建图路线 A | 启动自有 radar/IMU/chassis（`use_tf=false`）+ Cartographer |
| [gmapping.launch.py](../../src/yhs_nav2/launch/gmapping.launch.py) | 建图路线 B | 启动自有 radar/IMU/chassis + `slam_gmapping` |
| [localization_launch.py](../../src/yhs_nav2/launch/localization_launch.py) | 定位子链路 | `map_server`、`amcl`、生命周期管理器 |
| [navigation_launch.py](../../src/yhs_nav2/launch/navigation_launch.py) | 导航子链路 | 7 个服务器 + `velocity_smoother` + 重映射 |
| [yhs_nav2.yaml](../../src/yhs_nav2/param/yhs_nav2.yaml) | 唯一实际参数文件 | AMCL、BT、controller/TEB、代价地图、planner、旧版 recoveries |
| [set_init_amcl_pose.py](../../src/yhs_nav2/scripts/set_init_amcl_pose.py) | 设置初始位姿 | 发布 `/initialpose` |
| [block_spawner.py](../../src/yhs_nav2/scripts/block_spawner.py) | Gazebo 障碍物生成 | 依赖 `tb3_worlds`/`gazebo_msgs`，实车不用 |

---

## 2. 底盘与消息

| 文件 | 作用 | 关键符号 |
| --- | --- | --- |
| [yhs_can_control_node.cpp](../../src/yhs_can_control/src/yhs_can_control_node.cpp) | SocketCAN 驱动主实现 | `CanControl`、`can_cmd_callback()`、`cmd_callback()`、`free_ctrl_cmd_callback()`、`publish_odom()`、`run()` |
| [yhs_can_control_node.hpp](../../src/yhs_can_control/include/yhs_can_control/yhs_can_control_node.hpp) | 类声明与成员 | `can_socket_`、`use_tf_`、`io_cmd_`、`ctrl_cmd_`、`chassis_info_fb_publisher_` |
| [yhs_can_control.launch.py](../../src/yhs_can_control/launch/yhs_can_control.launch.py) | 启动底盘 | `use_tf` 参数 |
| [cfg.yaml](../../src/yhs_can_control/params/cfg.yaml) | 底盘参数 | `can_name` |
| [msg/](../../src/yhs_can_interfaces/msg) | 10 个消息 | `CtrlCmd`、`CtrlFb`、`ChassisInfoFb` 等 |
| [action/RecordPath.action](../../src/yhs_can_interfaces/action/RecordPath.action) | 示教路径动作 | `path`、`behavior_tree`、反馈字段 |

---

## 3. 传感器

| 文件 | 作用 |
| --- | --- |
| [nvilidar_ros2_node.cpp](../../src/hardware/nvilidar_ros2/src/nvilidar_ros2_node.cpp) | 雷达节点：参数、SDK 采样、`/scan` 发布 |
| [nvilidar.yaml](../../src/hardware/nvilidar_ros2/params/nvilidar.yaml) | 串口、角度、量程、滤波参数 |
| [nvilidar.launch.py](../../src/hardware/nvilidar_ros2/launch/nvilidar.launch.py) | 节点 + `base_link→laser_link` 静态 TF |
| [serial_port.cpp](../../src/hardware/serial_imu/src/serial_port.cpp) | IMU 节点：串口、解包、`/imu_data` |
| [ch_serial.c](../../src/hardware/serial_imu/src/ch_serial.c) | 串口底层读写与协议解析 |
| [imu_spec_msg.launch.py](../../src/hardware/serial_imu/launch/imu_spec_msg.launch.py) | IMU 节点 + `base_link→imu_link` |
| [ob_camera_node.cpp](../../src/hardware/OrbbecSDK_ROS2/orbbec_camera/src/ob_camera_node.cpp) | 深度相机流配置与发布 |
| [ob_camera_node_driver.cpp](../../src/hardware/OrbbecSDK_ROS2/orbbec_camera/src/ob_camera_node_driver.cpp) | 设备枚举与生命周期 |
| [image_publisher.cpp](../../src/hardware/OrbbecSDK_ROS2/orbbec_camera/src/image_publisher.cpp) | 图像发布 |
| [ros_service.cpp](../../src/hardware/OrbbecSDK_ROS2/orbbec_camera/src/ros_service.cpp) | 相机参数服务 |
| [pointcloud_to_laserscan_node.cpp](../../src/pointcloud_to_laserscan/src/pointcloud_to_laserscan_node.cpp) | 点云→`/scan1` |
| [pointcloud_to_laserscan.launch.py](../../src/pointcloud_to_laserscan/launch/pointcloud_to_laserscan.launch.py) | 相机静态 TF + 参数 |

---

## 4. SLAM

### 4.1 Cartographer 核心库

| 模块 | 目录 | 代表文件 / 类 |
| --- | --- | --- |
| 公共基础 | [common/](../../src/cartographer/cartographer/common) | `LuaParameterDictionary`、`ThreadPool`、`Task`、`Histogram`、`math.h` |
| 传感器数据 | [sensor/](../../src/cartographer/cartographer/sensor) | `PointCloud`、`RangeData`、`TimedPointCloudData`、`MapByTime` |
| 坐标变换 | [transform/](../../src/cartographer/cartographer/transform) | `RigidTransform`、`TransformInterpolationBuffer` |
| 地图构建接口 | [mapping/](../../src/cartographer/cartographer/mapping) | `MapBuilder`、`PoseGraph`、`Submap`、`TrajectoryBuilder` |
| 2D 管线 | [mapping/internal/2d/](../../src/cartographer/cartographer/mapping/internal/2d) | `LocalTrajectoryBuilder2D`、`PoseGraph2D`、`Submap2D` |
| 2D 扫描匹配 | [mapping/internal/2d/scan_matching/](../../src/cartographer/cartographer/mapping/internal/2d/scan_matching) | `CeresScanMatcher2D`、`FastCorrelativeScanMatcher2D`、`RealTimeCorrelativeScanMatcher2D` |
| 3D 管线 | [mapping/internal/3d/](../../src/cartographer/cartographer/mapping/internal/3d) | `LocalTrajectoryBuilder3D`、`PoseGraph3D` |
| IO | [io/](../../src/cartographer/cartographer/io) | `PointsProcessor` 管线、PCD/PLY/XYZ 写出、子图绘制 |
| 真值评估 | [ground_truth/](../../src/cartographer/cartographer/ground_truth) | 关系指标、真值生成 |
| 云端 | [cloud/](../../src/cartographer/cartographer/cloud) | gRPC `MapBuilderServer` |

2D 建图主调用链：

```
Node (ROS 2)
 └─► MapBuilderBridge
      └─► MapBuilder::AddTrajectoryBuilder()
           └─► LocalTrajectoryBuilder2D::AddRangeData()
                ├─► 运动外推（PoseExtrapolator）
                ├─► FastCorrelativeScanMatcher2D（实时相关匹配）
                ├─► CeresScanMatcher2D（精配准）
                └─► Submap2D::InsertRangeData()
           └─► PoseGraph2D::AddNode / AddConstraint / Optimize
```

### 4.2 Cartographer ROS 2 桥接

| 文件 | 作用 |
| --- | --- |
| [node.cpp](../../src/cartographer_ros/cartographer_ros/src/node.cpp) | ROS 2 节点：订阅/发布/服务、轨迹管理 |
| [node.h](../../src/cartographer_ros/cartographer_ros/include/cartographer_ros/node.h) | `Node` 类定义 |
| [node_constants.h](../../src/cartographer_ros/cartographer_ros/include/cartographer_ros/node_constants.h) | 话题/服务名常量 |
| [map_builder_bridge.cpp](../../src/cartographer_ros/cartographer_ros/src/map_builder_bridge.cpp) | ROS 消息 ↔ Cartographer 数据、TF 发布 |
| [sensor_bridge.cpp](../../src/cartographer_ros/cartographer_ros/src/sensor_bridge.cpp) | LaserScan/PointCloud/IMU/Odom → 内部类型 |
| [msg_conversion.cpp](../../src/cartographer_ros/cartographer_ros/src/msg_conversion.cpp) | 时间、位姿、点云转换 |
| [tf_bridge.cpp](../../src/cartographer_ros/cartographer_ros/src/tf_bridge.cpp) | TF 查询与坐标系变换 |
| [submap.cpp](../../src/cartographer_ros/cartographer_ros/src/submap.cpp) | 子图纹理与栅格 |
| [occupancy_grid_node_main.cpp](../../src/cartographer_ros/cartographer_ros/src/occupancy_grid_node_main.cpp) | 发布 `/map` |
| [node_main.cpp](../../src/cartographer_ros/cartographer_ros/src/node_main.cpp) | `cartographer_node` 入口 |
| [offline_node.cpp](../../src/cartographer_ros/cartographer_ros/src/offline_node.cpp) | 离线 bag 建图 |
| [assets_writer.cpp](../../src/cartographer_ros/cartographer_ros/src/assets_writer.cpp) | pbstream → PCD/PLY/地图资产 |

### 4.3 GMapping

| 文件 | 作用 |
| --- | --- |
| [slam_gmapping.cpp](../../src/slam_gmapping/slam_gmapping/src/slam_gmapping.cpp) | ROS 2 封装 `SlamGmapping`：激光回调、地图发布、TF |
| [openslam_gmapping/scanmatcher/](../../src/slam_gmapping/openslam_gmapping/scanmatcher) | 扫描匹配、ICP、地图 |
| [openslam_gmapping/particlefilter/](../../src/slam_gmapping/openslam_gmapping/particlefilter) | 粒子滤波 |
| [openslam_gmapping/grid/](../../src/slam_gmapping/openslam_gmapping/grid) | 栅格地图与图结构 |
| [slam_gmapping.launch.py](../../src/slam_gmapping/slam_gmapping/launch/slam_gmapping.launch.py) | GMapping 启动 |

---

## 5. Nav2

### 5.1 导航器与行为树

| 文件 | 作用 |
| --- | --- |
| [bt_navigator.cpp](../../src/navigation2/nav2_bt_navigator/src/bt_navigator.cpp) | 创建三个 Navigator：位姿、多点、**示教路径** |
| [navigate_to_pose.cpp](../../src/navigation2/nav2_bt_navigator/src/navigators/navigate_to_pose.cpp) | 单目标导航器 |
| [navigate_through_poses.cpp](../../src/navigation2/nav2_bt_navigator/src/navigators/navigate_through_poses.cpp) | 多点导航器 |
| [navigate_follow_record_path.cpp](../../src/navigation2/nav2_bt_navigator/src/navigators/navigate_follow_record_path.cpp) | **示教路径导航器**：订阅 `/record_path`、自调用动作、反馈 |
| [navigate_follow_record_path.hpp](../../src/navigation2/nav2_bt_navigator/include/nav2_bt_navigator/navigators/navigate_follow_record_path.hpp) | 导航器声明 |
| [navigate_follow_record_path.xml](../../src/navigation2/nav2_bt_navigator/behavior_trees/navigate_follow_record_path_w_replanning_and_recovery.xml) | 示教路径默认 BT |
| [navigate_to_pose_w_replanning_and_recovery.xml](../../src/navigation2/nav2_bt_navigator/behavior_trees/navigate_to_pose_w_replanning_and_recovery.xml) | 单目标默认 BT |
| [navigate_follow_record_path_action.cpp](../../src/navigation2/nav2_behavior_tree/plugins/action/navigate_follow_record_path_action.cpp) | BT 动作节点（封装 `RecordPath`） |
| [navigate_follow_record_path_action.hpp](../../src/navigation2/nav2_behavior_tree/include/nav2_behavior_tree/plugins/action/navigate_follow_record_path_action.hpp) | 端口：`goal`、`behavior_tree` |

### 5.2 服务器

| 文件 | 作用 |
| --- | --- |
| [controller_server.cpp](../../src/navigation2/nav2_controller/src/controller_server.cpp) | 加载 Controller 插件、执行 `FollowPath`、发布 `cmd_vel` |
| [planner_server.cpp](../../src/navigation2/nav2_planner/src/planner_server.cpp) | 加载 Planner 插件、发布 `plan` |
| [behavior_server.cpp](../../src/navigation2/nav2_behaviors/src/behavior_server.cpp) | 恢复行为宿主，默认 `spin/backup/drive_on_heading/wait` |
| [smoother_server.cpp](../../src/navigation2/nav2_smoother/src/nav2_smoother.cpp) | 路径平滑宿主 |
| [waypoint_follower.cpp](../../src/navigation2/nav2_waypoint_follower/src/waypoint_follower.cpp) | 航点任务 |
| [velocity_smoother.cpp](../../src/navigation2/nav2_velocity_smoother/src/velocity_smoother.cpp) | 速度限幅/平滑，`cmd_vel_nav → cmd_vel_smoothed` |
| [lifecycle_manager.cpp](../../src/navigation2/nav2_lifecycle_manager/src/lifecycle_manager.cpp) | 批量配置/激活/关闭节点 |

### 5.3 定位与地图

| 文件 | 作用 |
| --- | --- |
| [amcl_node.cpp](../../src/navigation2/nav2_amcl/src/amcl_node.cpp) | AMCL 主节点：粒子滤波、`/amcl_pose`、`map→odom` |
| [differential_motion_model.cpp](../../src/navigation2/nav2_amcl/src/motion_model/differential_motion_model.cpp) | 差速运动模型 |
| [map_cspace.cpp](../../src/navigation2/nav2_amcl/src/map/map_cspace.cpp) | 距离变换/似然场 |
| [map_server.cpp](../../src/navigation2/nav2_map_server/src/map_server/map_server.cpp) | 地图加载、`/map`、`load_map` |
| [map_saver.cpp](../../src/navigation2/nav2_map_server/src/map_saver/map_saver.cpp) | 地图保存服务 |
| [main_cli.cpp](../../src/navigation2/nav2_map_server/src/map_saver/main_cli.cpp) | `map_saver_cli` |
| [set_init_amcl_pose.py](../../src/yhs_nav2/scripts/set_init_amcl_pose.py) | 初始位姿发布 |

### 5.4 代价地图

| 文件 | 作用 |
| --- | --- |
| [costmap_2d_ros.cpp](../../src/navigation2/nav2_costmap_2d/src/costmap_2d_ros.cpp) | Costmap2DROS：插件加载、TF、发布 |
| [layered_costmap.cpp](../../src/navigation2/nav2_costmap_2d/src/layered_costmap.cpp) | 分层组合与更新 |
| [obstacle_layer.cpp](../../src/navigation2/nav2_costmap_2d/plugins/obstacle_layer.cpp) | 激光/点云障碍 |
| [static_layer.cpp](../../src/navigation2/nav2_costmap_2d/plugins/static_layer.cpp) | 静态地图层 |
| [inflation_layer.cpp](../../src/navigation2/nav2_costmap_2d/plugins/inflation_layer.cpp) | 膨胀层 |
| [voxel_layer.cpp](../../src/navigation2/nav2_costmap_2d/plugins/voxel_layer.cpp) | 体素层（3D） |
| [clear_costmap_service.cpp](../../src/navigation2/nav2_costmap_2d/src/clear_costmap_service.cpp) | 清除服务 |

### 5.5 规划器与控制插件

| 文件 | 作用 |
| --- | --- |
| [navfn_planner.cpp](../../src/navigation2/nav2_navfn_planner/src/navfn_planner.cpp) | NavFn 全局规划（当前使用） |
| [smac_planner_2d.cpp](../../src/navigation2/nav2_smac_planner/src/smac_planner_2d.cpp) | SMAC 2D |
| [smac_planner_hybrid.cpp](../../src/navigation2/nav2_smac_planner/src/smac_planner_hybrid.cpp) | Hybrid-A* |
| [theta_star_planner.cpp](../../src/navigation2/nav2_theta_star_planner/src/theta_star_planner.cpp) | Theta* |
| [regulated_pure_pursuit_controller.cpp](../../src/navigation2/nav2_regulated_pure_pursuit_controller/src/regulated_pure_pursuit_controller.cpp) | RPP |
| [nav2_rotation_shim_controller.cpp](../../src/navigation2/nav2_rotation_shim_controller/src/nav2_rotation_shim_controller.cpp) | 起步旋转转接 |
| [dwb_core](../../src/navigation2/nav2_dwb_controller/dwb_core) | DWB（未启用） |
| [teb_local_planner_ros.cpp](../../src/teb_local_planner/teb_local_planner/src/teb_local_planner_ros.cpp) | **TEB 插件入口**（当前使用） |
| [optimal_planner.cpp](../../src/teb_local_planner/teb_local_planner/src/optimal_planner.cpp) | g2o 图优化 |
| [homotopy_class_planner.cpp](../../src/teb_local_planner/teb_local_planner/src/homotopy_class_planner.cpp) | 多同伦类规划 |

### 5.6 核心接口与工具

| 文件 | 作用 |
| --- | --- |
| [nav2_core/controller.hpp](../../src/navigation2/nav2_core/include/nav2_core/controller.hpp) | Controller 插件接口 |
| [nav2_core/global_planner.hpp](../../src/navigation2/nav2_core/include/nav2_core/global_planner.hpp) | Planner 插件接口 |
| [nav2_core/behavior.hpp](../../src/navigation2/nav2_core/include/nav2_core/behavior.hpp) | Behavior 插件接口 |
| [nav2_core/smoother.hpp](../../src/navigation2/nav2_core/include/nav2_core/smoother.hpp) | Smoother 接口 |
| [bt_action_node.hpp](../../src/navigation2/nav2_behavior_tree/include/nav2_behavior_tree/bt_action_node.hpp) | `BtActionNode<T>` 模板 |
| [bt_service_node.hpp](../../src/navigation2/nav2_behavior_tree/include/nav2_behavior_tree/bt_service_node.hpp) | `BtServiceNode<T>` 模板 |
| [navigator.hpp](../../src/navigation2/nav2_bt_navigator/include/nav2_bt_navigator/navigator.hpp) | `Navigator<T>` 模板 |
| [odometry_utils.hpp](../../src/navigation2/nav2_util/include/nav2_util/odometry_utils.hpp) | `OdomSmoother` |
| [geometry_utils.hpp](../../src/navigation2/nav2_util/include/nav2_util/geometry_utils.hpp) | 路径长度/最近点 |
| [robot_utils.hpp](../../src/navigation2/nav2_util/include/nav2_util/robot_utils.hpp) | `getCurrentPose` |

---

## 6. 依赖库（第三方）

| 库 | 入口 | 在本项目中的角色 |
| --- | --- | --- |
| `teb_local_planner` | [teb_local_planner_ros.cpp](../../src/teb_local_planner/teb_local_planner/src/teb_local_planner_ros.cpp) | 局部轨迹优化控制器 |
| `costmap_converter` | [costmap_converter](../../src/dependence/costmap_converter) | 代价地图→多边形障碍 |
| `BehaviorTree.CPP` v3 | [behavior_tree_cpp_v3](../../src/dependence/BehaviorTree.CPP) | BT 引擎 |
| `pointcloud_to_laserscan` | [pointcloud_to_laserscan_node.cpp](../../src/pointcloud_to_laserscan/src/pointcloud_to_laserscan_node.cpp) | 深度补盲 |
| `cartographer` | [mapping/map_builder.h](../../src/cartographer/cartographer/mapping/map_builder.h) | SLAM 核心 |
| `navigation2` | [navigation2](../../src/navigation2) | 导航框架 |

---

## 7. 模型、工具与教学包

| 路径 | 作用 |
| --- | --- |
| [yhs_chassis_description/urdf](../../src/yhs_chassis_description/urdf) | 7 种车型 URDF |
| [yhs_chassis_description/meshes](../../src/yhs_chassis_description/meshes) | STL 外观 |
| [yhs_chassis_description/launch](../../src/yhs_chassis_description/launch) | 车型 launch |
| [yhs_chassis_description/rviz](../../src/yhs_chassis_description/rviz/rviz.rviz) | RViz 配置 |
| [tools/select_car.sh](../../src/tools/select_car.sh) | 车型选择/安装脚本（有缺陷，见 K17） |
| [tools/tf.py](../../src/tools/tf.py) | TF 打印工具 |
| [yhs_nav2/scripts/block_spawner.py](../../src/yhs_nav2/scripts/block_spawner.py) | Gazebo 障碍物 |
| [yhs_tutorials/lidar_pkg](../../src/yhs_tutorials/lidar_pkg) | 激光订阅示例 |
| [yhs_tutorials/pc_pkg](../../src/yhs_tutorials/pc_pkg) | 点云订阅示例 |
| [yhs_tutorials/vel_pub](../../src/yhs_tutorials/vel_pub) | 100 Hz `/cmd_vel` 测试 |
| [yhs_tutorials/nav_pkg](../../src/yhs_tutorials/nav_pkg) | `NavigateToPose` 客户端示例 |

---

## 8. 关键符号索引

| 符号 | 定义位置 | 用途 |
| --- | --- | --- |
| `CanControl` | [yhs_can_control_node.hpp](../../src/yhs_can_control/include/yhs_can_control/yhs_can_control_node.hpp) | 底盘 CAN 控制类 |
| `CanControl::can_cmd_callback` | [yhs_can_control_node.cpp](../../src/yhs_can_control/src/yhs_can_control_node.cpp) | CAN 帧解析与反馈发布 |
| `CanControl::publish_odom` | 同上 | 里程计积分与 TF |
| `SlamGmapping` | [slam_gmapping.cpp](../../src/slam_gmapping/slam_gmapping/src/slam_gmapping.cpp) | GMapping ROS 2 封装 |
| `MapBuilder` | [map_builder.h](../../src/cartographer/cartographer/mapping/map_builder.h) | 轨迹/子图/位姿图总入口 |
| `PoseGraph2D` | [pose_graph_2d.h](../../src/cartographer/cartographer/mapping/internal/2d/pose_graph_2d.h) | 2D 位姿图优化 |
| `LocalTrajectoryBuilder2D` | [local_trajectory_builder_2d.h](../../src/cartographer/cartographer/mapping/internal/2d/local_trajectory_builder_2d.h) | 前端激光里程计 |
| `cartographer_ros::Node` | [node.h](../../src/cartographer_ros/cartographer_ros/include/cartographer_ros/node.h) | ROS 2 桥接节点 |
| `BtNavigator` | [bt_navigator.hpp](../../src/navigation2/nav2_bt_navigator/include/nav2_bt_navigator/bt_navigator.hpp) | 导航器宿主 |
| `NavigateFollowRecordPathNavigator` | [navigate_follow_record_path.hpp](../../src/navigation2/nav2_bt_navigator/include/nav2_bt_navigator/navigators/navigate_follow_record_path.hpp) | 示教路径导航 |
| `NavigateFollowRecordPathAction` | [navigate_follow_record_path_action.hpp](../../src/navigation2/nav2_behavior_tree/include/nav2_behavior_tree/plugins/action/navigate_follow_record_path_action.hpp) | 示教 BT 动作节点 |
| `AmclNode` | [amcl_node.cpp](../../src/navigation2/nav2_amcl/src/amcl_node.cpp) | AMCL 定位 |
| `ControllerServer` | [controller_server.cpp](../../src/navigation2/nav2_controller/src/controller_server.cpp) | 控制服务器 |
| `PlannerServer` | [planner_server.cpp](../../src/navigation2/nav2_planner/src/planner_server.cpp) | 规划服务器 |
| `Costmap2DROS` | [costmap_2d_ros.cpp](../../src/navigation2/nav2_costmap_2d/src/costmap_2d_ros.cpp) | 代价地图容器 |
| `TebLocalPlannerROS` | [teb_local_planner_ros.cpp](../../src/teb_local_planner/teb_local_planner/src/teb_local_planner_ros.cpp) | TEB 插件 |

---

## 9. 需求 → 文件速查

| 想做的事 | 首先修改 |
| --- | --- |
| 改底盘 CAN 协议/ID | `yhs_can_control_node.cpp`、`yhs_can_interfaces/msg/*` |
| 改里程计/TF 策略 | `yhs_can_control_node.cpp` 的 `publish_odom()` 与 `use_tf` |
| 换雷达/改安装高度 | `nvilidar.yaml`、`nvilidar.launch.py` 静态 TF |
| 换 IMU 端口/波特率 | `serial_imu/src/serial_port.cpp` 宏定义 |
| 改建图参数 | `cartographer_ros/configuration_files/dgt_mini.lua` |
| 改 GMapping 参数 | `yhs_nav2/launch/gmapping.launch.py` |
| 换全局规划器 | `yhs_nav2.yaml` → `planner_server.GridBased.plugin` |
| 换局部控制器 | `yhs_nav2.yaml` → `controller_server.FollowPath.plugin` |
| 改代价地图/膨胀 | `yhs_nav2.yaml` 的 `global_costmap` / `local_costmap` |
| 改定位参数 | `yhs_nav2.yaml` 的 `amcl` |
| 改示教循迹行为 | `navigate_follow_record_path.cpp`、`navigate_follow_record_path_*.xml` |
| 加车型 | `yhs_chassis_description/urdf` + `launch` + `CMakeLists.txt` |
| 加新传感器 | 新驱动包 + 静态 TF + 代价地图 `observation_sources` |
| 改启动组合 | `yhs_nav2.launch.py`、`localization_launch.py`、`navigation_launch.py` |

---

[← 上一章：接口参考](11-interface-reference.md) · [返回索引](README.md) · [下一章：调参手册 →](13-tuning-playbook.md)
