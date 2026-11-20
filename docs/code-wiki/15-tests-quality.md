# 15 · 测试与质量保障

[← 上一章：二次开发](14-extension-guide.md) · [返回索引](README.md)

---

本章说明仓库现有的测试资产、CI 配置、覆盖缺口，以及针对本车业务链路的**最小验收方案**。

---

## 1. 测试资产总览

统计口径：文件名匹配 `*_test.cc`、`*_test.cpp`、`test_*.py`。

| 区域 | 测试源文件数 | 主要形式 |
| --- | ---: | --- |
| `cartographer` | 91 | gtest / Bazel / CMake |
| `navigation2` | 50 | gtest + pytest + 系统测试 |
| `slam_gmapping` | 6 | gtest |
| `cartographer_ros` | 4 | gtest（配置、消息转换、时间转换） |
| `teb_local_planner` | 1 | gtest |
| `dependence/BehaviorTree.CPP` | 1 | gtest |
| **合计** | **153** | — |

> **业务包测试为 0**：`yhs_can_control`、`yhs_nav2`、`yhs_chassis_description`、`serial_imu`、`nvilidar_ros2` 均无测试目录。这是当前最大的质量缺口。

---

## 2. Cartographer 测试体系

### 2.1 三层测试

| 层级 | 位置 | 内容 |
| --- | --- | --- |
| 单元测试 | [cartographer/**/*_test.cc](../../src/cartographer) | 数学、变换、传感器、2D/3D 子图、扫描匹配、位姿图、IO |
| 集成测试 | [cartographer_ros](../../src/cartographer_ros) | `configuration_files_test`、`msg_conversion_test`、`time_conversion_test` |
| 性能/内存 | [scripts/build_test_asan.sh](../../src/cartographer/scripts/build_test_asan.sh) | ASan + LeakSanitizer + `ctest -j20` |

### 2.2 典型测试文件

| 文件 | 验证点 |
| --- | --- |
| [math_test.cc](../../src/cartographer/cartographer/common/math_test.cc) | 角度归一化、插值 |
| [rigid_transform_test.cc](../../src/cartographer/cartographer/transform/rigid_transform_test.cc) | 刚体变换 |
| [probability_values_test.cc](../../src/cartographer/cartographer/mapping/probability_values_test.cc) | 概率/对数几率 |
| [submaps_test.cc](../../src/cartographer/cartographer/mapping/submaps_test.cc) | 子图索引与分辨率 |
| [fast_correlative_scan_matcher_2d_test.cc](../../src/cartographer/cartographer/mapping/internal/2d/scan_matching/fast_correlative_scan_matcher_2d_test.cc) | 前端相关匹配 |
| [pose_graph_2d_test.cc](../../src/cartographer/cartographer/mapping/internal/2d/pose_graph_2d_test.cc) | 2D 后端优化 |
| [proto_stream_test.cc](../../src/cartographer/cartographer/io/proto_stream_test.cc) | pbstream 读写 |

### 2.3 运行方式

**独立 CMake：**

```bash
cd src/cartographer
mkdir -p build && cd build
cmake .. -G Ninja -DCMAKE_BUILD_TYPE=Release
ninja
ctest --output-on-failure
```

**Bazel：**

```bash
cd src/cartographer
bazel test //cartographer/...
```

**在 colcon 工作空间中：**

```bash
colcon build --packages-select cartographer cartographer_ros \
  --cmake-args -DBUILD_TESTING=ON
colcon test --packages-select cartographer cartographer_ros
colcon test-result --verbose
```

---

## 3. Nav2 测试体系

### 3.1 包级测试

20 个 Nav2 包带 `test/` 目录，覆盖：

| 包 | 测试重点 |
| --- | --- |
| `nav2_amcl` | 运动模型、激光模型、地图 |
| `nav2_costmap_2d` | 图层、膨胀、足迹、清除服务 |
| `nav2_navfn_planner` / `nav2_smac_planner` / `nav2_theta_star_planner` | 规划结果与边界 |
| `nav2_controller` / `nav2_regulated_pure_pursuit_controller` / `nav2_rotation_shim_controller` | 控制输出、目标检查、进度检查 |
| `nav2_behavior_tree` | BT 节点端口与行为 |
| `nav2_lifecycle_manager` | 生命周期切换与 bond |
| `nav2_map_server` | 地图加载/保存 |
| `nav2_waypoint_follower` | 航点任务 |
| `nav2_simple_commander` | Python API（pytest） |
| `nav2_system_tests` | Gazebo 集成测试 |

### 3.2 运行

```bash
# 单元测试（不启动 Gazebo）
colcon test --packages-select nav2_amcl nav2_costmap_2d nav2_behavior_tree
colcon test-result --verbose

# Python 测试
python3 -m pytest src/navigation2/nav2_simple_commander/test -v

# 系统测试（需要 Gazebo/TurtleBot3 环境）
colcon test --packages-select nav2_system_tests
```

> `nav2_system_tests` 依赖仿真环境与测试地图，在本仓库的实车工作空间中通常不执行。

---

## 4. 项目自有代码的测试缺口

| 模块 | 现状 | 风险 |
| --- | --- | --- |
| `yhs_can_control` | 无测试 | CAN 编解码、CRC、里程计积分错误只能实车发现 |
| `yhs_nav2` | 无 launch 测试 | 地图路径错误（K1）、参数键过期（K4）无法在 CI 发现 |
| `yhs_chassis_description` | 无 URDF 校验 | `FR-mid` 加载错车型（K8）无法自动发现 |
| `serial_imu` | 无解析测试 | 协议字段越界、单位换算错误 |
| `nvilidar_ros2` | 无解析测试 | 角度/距离索引错误 |
| 示教路径 | 无集成测试 | 动作链、BT 加载、路径坐标系问题只能现场调 |
| 依赖声明 | `yhs_nav2/package.xml` 无运行依赖 | 构建顺序与 `rosdep` 安装不完整 |

---

## 5. CI 配置现状

| 仓库 | 配置 | 作用 |
| --- | --- | --- |
| `cartographer` | [.bazelci/presubmit.yml](../../src/cartographer/.bazelci/presubmit.yml) | Ubuntu 16.04 上 `bazel build/test //...` |
| `cartographer` | [azure-pipelines.yml](../../src/cartographer/azure-pipelines.yml) | Windows + ROS Melodic：构建、`ctest`、JUnit 报告 |
| `cartographer_ros` | [azure-pipelines.yml](../../src/cartographer_ros/azure-pipelines.yml) | ROS 1 catkin 构建/测试（上游配置） |
| `navigation2` | [.circleci/config.yml](../../src/navigation2/.circleci/config.yml) | ROS 2 release 构建/测试 + nightly，含覆盖率上传 |
| `navigation2` | [tools/run_test_suite.bash](../../src/navigation2/tools/run_test_suite.bash) | 本地批量测试脚本 |
| `navigation2` | [tools/run_sanitizers](../../src/navigation2/tools/run_sanitizers) | Sanitizer 运行脚本 |

> 这些 CI 面向**上游独立仓库**，不会自动构建 `yhs_*` 业务包，也不会检查本项目的 launch/参数一致性。

---

## 6. 静态质量工具

| 工具 | 现状 |
| --- | --- |
| `ament_lint_auto` | 45 个 `CMakeLists.txt` 启用（主要在 Nav2/Cartographer 上游包） |
| `ament_uncrustify` / `cpplint` / `copyright` | 上游包按需启用；部分显式跳过 |
| `.clang-format` | [cartographer/cartographer/.clang-format](../../src/cartographer/cartographer/.clang-format)、[cartographer_ros/cartographer_ros/src/.clang-format](../../src/cartographer_ros/cartographer_ros/src/.clang-format) 等 |
| 编译告警 | 多数包开启 `-Wall -Wextra` |
| Sanitizer | Cartographer 提供 ASan 脚本 |

建议在业务包补充：

```bash
ament_cpplint
ament_uncrustify
ament_lint_auto
```

---

## 7. 最小验收方案（实车）

### 7.1 启动前

| 检查 | 命令 | 通过标准 |
| --- | --- | --- |
| 设备 | `ls -l /dev/can0 /dev/lidar /dev/imu` | 均存在 |
| CAN | `candump can0` | 有帧 |
| 包完整性 | `ros2 pkg list \| grep -E "yhs\|nvilidar\|serial_imu"` | 全部存在 |
| 地图文件 | 检查 YAML `image` 指向的文件 | 存在且可读 |
| 参数 | `ros2 param dump /amcl` | 与预期一致 |

### 7.2 建图验收

| 项 | 方法 | 标准 |
| --- | --- | --- |
| 雷达 | `ros2 topic hz /scan` | 稳定 |
| 里程计 | 手动推车，`/odom` 变化方向正确 | 前进为正、左转为正 |
| 地图 | RViz 观察 | 墙体闭合、无明显重影 |
| 闭环 | 回到起点 | 首尾重合 |
| 保存 | `write_state` + `pbstream_to_ros_map` | 生成 pgm/yaml |

### 7.3 定位验收

| 项 | 方法 | 标准 |
| --- | --- | --- |
| 初始位姿 | 发 `/initialpose` 或检查 `initial_pose` | 粒子收敛到真实位置 |
| 静止抖动 | 观察 `/amcl_pose` 30 s | 位置 < 2 cm，yaw < 1° |
| 行走 | 直行 3 m、原地转 360° | 无跳变、无丢失 |
| TF | `tf2_echo map base_link` | 连续、无外推告警 |

### 7.4 导航验收

| 项 | 方法 | 标准 |
| --- | --- | --- |
| 单点 | RViz `2D Goal Pose` | 到达，误差 < 0.1 m / 0.1 rad |
| 避障 | 路径放置障碍 | 绕行或停车，不碰撞 |
| 恢复 | 人为堵路 | 触发恢复并最终失败/绕行 |
| 示教 | 发布 `/record_path` | 沿录制路径行驶 |
| 急停 | 遥控/物理急停 | 立即停止 |

### 7.5 安全项

- 首次测试架空驱动轮。
- 限制 `max_vel_x ≤ 0.2 m/s`。
- 遥控器优先于 Nav2。
- 记录 rosbag，便于回溯。

---

## 8. 建议补充的测试（按优先级）

| 优先级 | 测试 | 做法 |
| --- | --- | --- |
| P0 | 地图 YAML 自检 | 启动前脚本解析所有 `map/*.yaml`，校验 `image` 存在 |
| P0 | CAN 编解码单元测试 | 把 `ctrl_cmd`/`io_cmd` 的打包逻辑抽为纯函数，gtest 校验字节序与 CRC |
| P0 | 参数键校验 | 启动测试读取 `yhs_nav2.yaml`，检查 Nav2 1.1.5 的关键键名 |
| P1 | launch 冒烟测试 | `launch_testing` 启动 `localization_launch.py`，断言 `/map`、`/amcl_pose` 存在 |
| P1 | TF 完整性测试 | 静态分析 URDF/launch，校验 `base_link→laser_link` 等 |
| P1 | 示教路径集成测试 | 录一段短路径，回放 `/record_path`，断言动作成功 |
| P2 | 里程计精度测试 | 直行/原地转，比较 `/odom` 与卷尺/角度 |
| P2 | 传感器解析测试 | 用固定字节流喂给 IMU/雷达解析函数 |

---

## 9. 已知问题 → 测试映射

| 问题 | 可自动化的测试 |
| --- | --- |
| K1 地图 PGM 缺失 | YAML 资源存在性测试 |
| K2 `use_sim_time` 不一致 | launch 参数一致性测试 |
| K3 局部膨胀层未启用 | YAML 插件列表断言 |
| K4 旧版 Nav2 键名 | 参数 schema 校验 |
| K5 子 launch 默认参数文件不存在 | launch 参数路径存在性测试 |
| K6 相机被注释但 costmap 依赖 `/scan1` | launch/参数交叉检查 |
| K7 TEB 与 costmap 外形不一致 | `footprint_model` 与 `footprint` 一致性断言 |
| K8 组合模式只启动空容器 | 节点/进程清单测试 |
| K9 车型 launch 复制粘贴错误 | launch 默认 `urdf_model` 断言 |
| K10 编排包无运行依赖 | `rosdep check` + `package.xml` 静态检查 |
| K11 IMU 设备硬编码 | 启动参数化测试 |

---

## 10. 质量门禁建议

```bash
# 1. 构建
colcon build --symlink-install --cmake-args -DBUILD_TESTING=ON

# 2. 上游单测
colcon test --packages-select cartographer cartographer_ros \
  nav2_amcl nav2_costmap_2d nav2_behavior_tree nav2_bt_navigator
colcon test-result --verbose

# 3. 静态检查（业务包逐步接入）
ament_cpplint src/yhs_can_control src/yhs_nav2
ament_uncrustify --reformat src/yhs_can_control

# 4. 配置自检（建议新增脚本）
python3 tools/check_config.py src/yhs_nav2
```

---

[← 上一章：二次开发](14-extension-guide.md) · [返回索引](README.md)
