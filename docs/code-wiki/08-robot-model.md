# 08 · 机器人模型与 TF

[← 上一章：局部规划](07-local-planning.md) · [返回索引](README.md) · [下一章：编译运行 →](09-build-run-debug.md)

---

## 1. 包内容

`yhs_chassis_description` 是**纯资源包**（只有 `ament_cmake` + 安装规则），不含 C++ 节点：

```
yhs_chassis_description/
├── urdf/        # 7 种车型 + robot.urdf（当前默认车型副本）
├── meshes/      # 8 个 STL（base / wheel）
├── launch/      # 每个车型一个 launch + robot.launch.py
├── rviz/rviz.rviz
├── CMakeLists.txt
└── package.xml
```

| URDF | 对应 STL | link 数 | launch 默认指向 |
| --- | --- | --- | --- |
| `DT-mid.urdf` | `DT-mid-base.STL`、`DT-mid-wheel.STL` | 3 | `DT-mid.launch.py` |
| `DT-mini.urdf` | `DT-mini.STL` | 1 | `DT-mini.launch.py`、`robot.launch.py` |
| `FR_mid.urdf` | `FR-mid.STL` | 1 | **无**（`FR-mid.launch.py` 误指 DT-mini） |
| `FR_mini.urdf` | `FR-mini.STL` | 1 | `FR-mini.launch.py` |
| `FW-mid.urdf` | `FW-mid.STL` | 1 | `FW-mid.launch.py` |
| `MK-mid.urdf` | `MK-mid-base.STL`、`MK-mid-wheel.STL` | 3 | `MK-mid.launch.py` |
| `TK-mid.urdf` | `TK-mid.STL` | 1 | `TK-mid.launch.py` |
| `robot.urdf` | — | 1 | 未被 launch 引用（内容与单 link 车型一致） |

> **命名不一致**：文件名 `FR_mid.urdf` 用下划线，其他用连字符；且 `FR-mid.launch.py` 的默认 URDF 写成了 `DT-mini.urdf`，导致 `ros2 launch yhs_chassis_description FR-mid.launch.py` 实际显示 DT-mini 车型。

---

## 2. URDF 结构

以 [DT-mini.urdf](../../src/yhs_chassis_description/urdf/DT-mini.urdf) 为例（全文仅 18 行）：

```xml
<robot name="yhs_chassis_description">
  <link name="base_link">
    <visual>
      <origin xyz="-0.3 -0.25 0" rpy="0 0 0"/>
      <geometry>
        <mesh filename="package://yhs_chassis_description/meshes/DT-mini.STL"/>
      </geometry>
      <material name=""><color rgba="0.8 0.8 0.8 1"/></material>
    </visual>
  </link>
</robot>
```

关键事实：

1. **只有一个 `base_link`**，无 `joint`、无 `inertial`、无 `collision`。
2. 网格原点被平移 `(-0.3, -0.25, 0)`，用于把 STL 的几何中心对齐到 `base_link`。
3. `robot_state_publisher` 因此**不会发布任何 `base_link` 的子 TF**；轮子视觉只是网格的一部分，不会转动。
4. `joint_state_publisher` 被启动，但没有关节可发布，属于冗余节点。

`DT-mid` / `MK-mid` 有 3 个 link，通常是 `base_link` + 左右轮 `continuous joint`，但同样没有 inertial/collision，只用于 RViz 展示。

---

## 3. 启动文件

所有车型 launch 结构完全相同（见 [DT-mini.launch.py](../../src/yhs_chassis_description/launch/DT-mini.launch.py)）：

| 节点 | 作用 | 参数 |
| --- | --- | --- |
| `robot_state_publisher` | 读取 URDF，发布 link 静态结构与 `robot_description` | `use_sim_time` |
| `joint_state_publisher` | 发布关节状态（本 URDF 无关节） | — |
| `rviz2` | 加载 `rviz/rviz.rviz` | 受 `use_rviz` 控制 |

启动参数：

| 参数 | 默认 | 说明 |
| --- | --- | --- |
| `urdf_model` | `<pkg>/urdf/<车型>.urdf` | 绝对路径 |
| `rviz_config_file` | `<pkg>/rviz/rviz.rviz` | — |
| `use_robot_state_pub` | `True` | 声明了但**未用于条件判断** |
| `use_rviz` | `True` | 是否启动 RViz |
| `use_sim_time` | `False` | — |

> `use_robot_state_pub` 只声明、不生效：即使设为 `False` 仍会启动 `robot_state_publisher`。

---

## 4. TF 的真实来源

URDF 只提供"外观"，**运行时的核心 TF 由驱动节点提供**：

| TF | 发布者 | 何时 |
| --- | --- | --- |
| `map → odom` | AMCL（导航）/ Cartographer / GMapping（建图） | 定位/建图 |
| `odom → base_link` | `yhs_can_control`（`use_tf=true`）或 Cartographer（`provide_odom_frame=true`） | 始终 |
| `base_link → laser_link` | `nvilidar.launch.py` 静态 TF（z=0.33） | 雷达启动 |
| `base_link → imu_link` | `imu_spec_msg.launch.py` 静态 TF（全 0） | IMU 启动 |
| `base_link → camera_link` | `pointcloud_to_laserscan.launch.py` 静态 TF（x=0.23, z=0.2） | 相机启用 |

**模型包不是导航必需项**：导航只依赖上述 TF。`yhs_chassis_description` 主要用于 RViz 看车体外形、确认 footprint 尺寸。

---

## 5. 车型切换脚本 `tools/select_car.sh`

脚本把对应车型的 launch 复制覆盖为 `robot.launch.py`，实现"默认车型"切换：

```bash
cd src/tools
bash select_car.sh
# A: DT-mid  B: DT-mini  C: FR-mid
# D: FR-mini E: FW-mid   F: MK-mid  G: TK-mid
```

### 5.1 脚本问题

1. **硬编码路径**：`/home/yhs/ros2_ws/src/...`，换用户名/工作空间即失败。
2. **D 选项错误**：选择 FR-mini 时复制的是 `DT-mini.launch.py`（`select_car.sh:40`）。
3. **本质是复制覆盖**：不保留原文件，重复执行后 `robot.launch.py` 被改写；建议改为通过 `urdf_model` 参数选择，而不是覆盖文件。
4. **没有安装进 install/share**：脚本位于 `tools/`，不随 colcon 安装；修改后需手动同步到 `install/` 或重新 `colcon build`。

### 5.2 推荐用法（不改脚本）

```bash
ros2 launch yhs_chassis_description robot.launch.py \
  urdf_model:=$HOME/ros2_ws/src/yhs_chassis_description/urdf/MK-mid.urdf
```

---

## 6. `tools/tf.py`

一个**独立的小工具**（不属于任何 ROS 包）：

- 继承 `rclpy.node.Node`，发布 `geometry_msgs/Quaternion` 到 `quaternion_topic`。
- 从终端依次输入 roll/pitch/yaw（度），调用 `convert_angles_to_quaternion()` 转四元数并打印/发布。
- 用于给静态 TF 或 URDF 计算四元数，例如：

```bash
python3 src/tools/tf.py
# Enter roll angle: 0
# Enter pitch angle: 0
# Enter yaw angle: 90
```

> 小缺陷：`main()` 中先 `rclpy.spin()` 再 `destroy_node()`，正常退出时后者不可达；只适合 Ctrl+C 结束。

---

## 7. 新增车型的标准步骤

1. 把 STL 放入 `meshes/`，命名 `<车型>-base.STL` / `<车型>-wheel.STL`。
2. 复制一份 `DT-mid.urdf`，改 `<mesh filename>`、`origin`（用于把几何中心对齐 `base_link`）。
3. 复制 `DT-mid.launch.py` 为 `<车型>.launch.py`，把默认 `urdf_model` 改对（**不要**再出现 FR-mid 的复制粘贴问题）。
4. 在 `CMakeLists.txt` 的 `install(DIRECTORY ...)` 中确认 `urdf`、`meshes`、`launch`、`rviz` 已安装。
5. 实测 footprint：在 RViz 中量取车体长宽，同步更新 `yhs_nav2.yaml` 的 `footprint` 与 TEB 的 `footprint_model`。
6. `colcon build --packages-select yhs_chassis_description`。

---

## 8. 检查命令

```bash
# 预览模型
ros2 launch yhs_chassis_description robot.launch.py use_rviz:=true

# 检查 link
ros2 run tf2_tools view_frames

# 检查静态 TF
ros2 run tf2_ros tf2_echo base_link laser_link
ros2 run tf2_ros tf2_echo base_link imu_link

# 检查 URDF 解析
check_urdf install/yhs_chassis_description/share/yhs_chassis_description/urdf/DT-mini.urdf
```

> `check_urdf` 来自 `liburdfdom-tools`，未安装时 `sudo apt install liburdfdom-tools`。
