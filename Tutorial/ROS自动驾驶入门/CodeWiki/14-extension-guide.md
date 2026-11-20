# 14 · 二次开发扩展指南

[← 上一章：调参手册](13-tuning-playbook.md) · [返回索引](README.md) · [下一章：测试与质量 →](15-tests-quality.md)

---

本章说明本仓库最常见的五类扩展：**加车型、加传感器、换规划/控制插件、加行为树节点、加自定义导航器**，以及示教路径功能的端到端改造点。

---

## 1. 新增车型

### 1.1 文件准备

1. 在 [urdf/](../../../src/yhs_chassis_description/urdf) 新增 `XX-mid.urdf`。
2. 在 [meshes/](../../../src/yhs_chassis_description/meshes) 放入 STL，URDF 中使用：
   ```xml
   <mesh filename="package://yhs_chassis_description/meshes/XX-mid.STL"/>
   ```
3. 在 [launch/](../../../src/yhs_chassis_description/launch) 复制一个现有车型 launch，修改 `urdf_model` 默认值。

### 1.2 必须同步的点

| 项目 | 说明 |
| --- | --- |
| `base_link` 原点 | 建议与现有车型一致（底盘中心、地面投影） |
| 雷达安装高度 | 若变化，必须改 [nvilidar.launch.py](../../../src/hardware/nvilidar_ros2/launch/nvilidar.launch.py) 的 `0.33` |
| IMU 安装 | 改 [imu_spec_msg.launch.py](../../../src/hardware/serial_imu/launch/imu_spec_msg.launch.py) 静态 TF 或 URDF |
| 相机安装 | 改 [pointcloud_to_laserscan.launch.py](../../../src/pointcloud_to_laserscan/launch/pointcloud_to_laserscan.launch.py) 的 `0.23 0 0.2` |
| 代价地图 footprint | 改 [yhs_nav2.yaml](../../../src/yhs_nav2/param/yhs_nav2.yaml) 的 `footprint` |
| TEB 车体模型 | 改 `controller_server.FollowPath.footprint_model`，保持与 footprint 一致 |

CMake 目前整目录安装，无需逐文件登记：

```cmake
install(DIRECTORY launch meshes rviz urdf
    DESTINATION share/${PROJECT_NAME})
```

### 1.3 验证

```bash
ros2 launch yhs_chassis_description XX-mid.launch.py
ros2 run tf2_tools view_frames
ros2 topic echo /robot_description --once | head
```

> 现有 `select_car.sh` 有路径与复制错误（K17），建议手工复制 launch 并检查 `urdf_model` 默认值，不要直接依赖该脚本。

---

## 2. 新增传感器

以新增一个"后向超声波/ToF"为例：

### 2.1 驱动节点

新建 ROS 2 包（C++ 或 Python），要求：

| 要求 | 说明 |
| --- | --- |
| 发布 `sensor_msgs/LaserScan` 或 `Range` | 若用于代价地图，优先 `LaserScan` |
| `header.frame_id` 固定 | 如 `rear_tof_link` |
| 时间戳来自同一时钟 | 避免 TF 外推 |
| QoS 与消费者匹配 | 激光类常用 `SensorDataQoS`（best effort） |

### 2.2 静态 TF

在驱动 launch 中发布：

```python
Node(
    package='tf2_ros', executable='static_transform_publisher',
    arguments=['-0.30', '0', '0.20', '0', '0', '0', '1', 'base_link', 'rear_tof_link'])
```

### 2.3 接入代价地图

在 [yhs_nav2.yaml](../../../src/yhs_nav2/param/yhs_nav2.yaml) 的 `observation_sources` 增加：

```yaml
observation_sources: scan scan1 rear_tof
rear_tof:
  topic: /rear_tof_scan
  data_type: "LaserScan"
  clearing: true
  marking: true
  raytrace_max_range: 2.0
  raytrace_min_range: 0.05
  obstacle_max_range: 1.5
  obstacle_min_range: 0.05
```

### 2.4 验证

```bash
ros2 topic hz /rear_tof_scan
ros2 run tf2_ros tf2_echo base_link rear_tof_link
ros2 topic echo /local_costmap/costmap --once | head
```

---

## 3. 更换全局规划器 / 局部控制器

### 3.1 插件机制

Nav2 通过 pluginlib 加载：

| 类型 | 基类 | 插件描述文件 |
| --- | --- | --- |
| 全局规划 | `nav2_core::GlobalPlanner` | `global_planner_plugin.xml` |
| 局部控制 | `nav2_core::Controller` | `*.xml` |
| 恢复行为 | `nav2_core::Behavior` | `behavior_plugin.xml` |
| 平滑器 | `nav2_core::Smoother` | `plugins.xml` |

示例（RPP）：

```xml
<class_libraries>
  <library path="nav2_regulated_pure_pursuit_controller">
    <class type="nav2_regulated_pure_pursuit_controller::RegulatedPurePursuitController"
           base_class_type="nav2_core::Controller">
      <description>RPP controller</description>
    </class>
  </library>
</class_libraries>
```

### 3.2 切换到 RPP（示例）

修改 [yhs_nav2.yaml](../../../src/yhs_nav2/param/yhs_nav2.yaml)：

```yaml
controller_server:
  ros__parameters:
    controller_plugins: ["FollowPath"]
    FollowPath:
      plugin: "nav2_regulated_pure_pursuit_controller::RegulatedPurePursuitController"
      desired_linear_vel: 0.4
      lookahead_dist: 0.6
      min_approach_linear_velocity: 0.05
```

同时删除/保留 TEB 专属参数（`dt_ref`、`min_obstacle_dist` 等对 RPP 无效）。

### 3.3 切换全局规划器

```yaml
planner_server:
  ros__parameters:
    planner_plugins: ["GridBased"]
    GridBased:
      plugin: "nav2_smac_planner/SmacPlanner2D"   # 或 nav2_navfn_planner/NavfnPlanner
      tolerance: 0.5
```

### 3.4 自定义控制器骨架

```cpp
class MyController : public nav2_core::Controller
{
public:
  void configure(const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
                 std::string name, std::shared_ptr<tf2_ros::Buffer> tf,
                 std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros) override;
  void cleanup() override;
  void activate() override;
  void deactivate() override;
  geometry_msgs::msg::TwistStamped computeVelocityCommands(
      const geometry_msgs::msg::PoseStamped & pose,
      const geometry_msgs::msg::Twist & velocity) override;
  void setPlan(const nav_msgs::msg::Path & path) override;
};
```

注册：

```cpp
PLUGINLIB_EXPORT_CLASS(my_controller::MyController, nav2_core::Controller)
```

并在 `CMakeLists.txt` 安装插件 XML、在 `package.xml` 导出 `nav2_core`。

---

## 4. 新增行为树节点

### 4.1 动作节点骨架

```cpp
#include "nav2_behavior_tree/bt_action_node.hpp"
#include "my_msgs/action/my_action.hpp"

class MyAction : public nav2_behavior_tree::BtActionNode<my_msgs::action::MyAction>
{
public:
  MyAction(const std::string & name, const std::string & action_name,
           const BT::NodeConfiguration & conf)
  : BtActionNode<my_msgs::action::MyAction>(name, action_name, conf) {}

  void on_tick() override
  {
    // 从端口取值，填充 goal_
    getInput("target", goal_.target);
  }

  static BT::PortsList providedPorts()
  {
    return providedBasicPorts({
      BT::InputPort<std::string>("target", "target id")});
  }
};
```

### 4.2 注册与构建

```cpp
#include "behaviortree_cpp_v3/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<MyAction>("MyAction");
}
```

CMake 中**每个 BT 节点单独编成共享库**（Nav2 的约定）：

```cmake
add_library(my_action_bt_node SHARED plugins/action/my_action.cpp)
list(APPEND plugin_libs my_action_bt_node)
```

然后在 `yhs_nav2.yaml` 的 `bt_navigator.plugin_lib_names` 增加 `my_action_bt_node`。

### 4.3 在 BT XML 中使用

```xml
<MyAction target="dock" server_name="my_action"/>
```

### 4.4 自定义节点的现有样例

| 文件 | 说明 |
| --- | --- |
| [navigate_follow_record_path_action.cpp](../../../src/navigation2/nav2_behavior_tree/plugins/action/navigate_follow_record_path_action.cpp) | 注册名 `NavigateFollowRecordPath`，动作名 `navigate_follow_record_path` |
| 端口 | `goal`（`nav_msgs/Path`）、`behavior_tree`（string） |
| 构建库名 | `nav2_compute_follow_record_path_action_bt_node`（命名与节点名不一致，注意别写错） |

> 当前示教路径 BT **没有使用**该节点，而是直接用标准 `FollowPath`；因此该节点属于"可用但未接入默认树"的状态。

---

## 5. 新增自定义导航器

以现有 `NavigateFollowRecordPathNavigator` 为模板：

1. 继承 `nav2_bt_navigator::Navigator<MyAction>`。
2. 实现：
   - `configure(parent, odom_smoother)`
   - `cleanup()`
   - `getName()`（同时作为动作名）
   - `getDefaultBTFilepath()`
   - `goalReceived()`、`onLoop()`、`onPreempt()`、`goalCompleted()`
3. 在 [bt_navigator.cpp](../../../src/navigation2/nav2_bt_navigator/src/bt_navigator.cpp) 中：
   ```cpp
   my_navigator_ = std::make_unique<MyNavigator>();
   my_navigator_->on_configure(shared_from_this(), plugin_lib_names,
                               feedback_utils, &plugin_muxer_, odom_smoother_);
   ```
4. 在 `on_activate` / `on_deactivate` 中调用对应生命周期方法。
5. 在 `CMakeLists.txt` 源文件列表中加入新 cpp。

### 5.1 示教路径功能拆解

| 环节 | 实现 |
| --- | --- |
| 动作定义 | [RecordPath.action](../../../src/yhs_can_interfaces/action/RecordPath.action) |
| 动作服务器 | `bt_navigator` 内的 `NavigateFollowRecordPathNavigator`，动作名 `navigate_follow_record_path` |
| 触发方式 A | 向 `/record_path` 发布 `nav_msgs/Path`，导航器自动自调用动作 |
| 触发方式 B | 直接发送 `/navigate_follow_record_path` 动作 goal |
| 默认 BT | `default_follow_record_path_bt_xml` 指向 `navigate_follow_record_path_w_replanning_and_recovery.xml` |
| BT 内容 | 5 Hz `FollowPath` + 清除代价地图 + 恢复动作 |
| 反馈 | 当前位姿、已用时间、预计剩余时间、恢复次数、剩余距离 |

**已知注意点**：

- BT 中的 `controller_id="FollowPath"` 必须与 `controller_server.controller_plugins` 的名称一致。
- `/record_path` 的路径坐标系必须是 `map`，否则 `FollowPath` 无法执行。
- 该 BT 的 `RateController hz="5.0"` 与 `controller_frequency=20 Hz` 不同，实际重规划频率是 5 Hz。

---

## 6. 新增地图

1. 将 `xxx.pgm` 与 `xxx.yaml` 放入 [yhs_nav2/map](../../../src/yhs_nav2/map)。
2. YAML 中 `image` 必须与实际文件名完全一致（当前 `zgq-g.yaml` 即因指向不存在的 `zgq.pgm` 而失败，K1）。
3. 启动时指定：
   ```bash
   ros2 launch yhs_nav2 yhs_nav2.launch.py map:=/path/to/xxx.yaml
   ```
4. 校验：
   ```bash
   ros2 run nav2_map_server map_saver_cli -f /tmp/check
   ros2 topic echo /map --once | head
   ```

---

## 7. 扩展检查清单

| 检查项 | 命令/方法 |
| --- | --- |
| 包能被找到 | `ros2 pkg prefix <pkg>` |
| 插件能被加载 | `ros2 param get /controller_server FollowPath.plugin` + 启动日志 |
| BT 库能被加载 | `plugin_lib_names` 是否包含库名；启动日志无 "Couldn't load library" |
| 动作存在 | `ros2 action list -t` |
| TF 完整 | `ros2 run tf2_tools view_frames` |
| 参数生效 | `ros2 param dump <node>` |
| 依赖声明完整 | `package.xml` 的 `depend`/`exec_depend`，`rosdep check` |

---

[← 上一章：调参手册](13-tuning-playbook.md) · [返回索引](README.md) · [下一章：测试与质量 →](15-tests-quality.md)
