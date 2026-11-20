# OS-lite ROS2 导航小车 · Code Wiki

> 本文档集基于对 `/workspace/src`（ROS 2 工作空间源码）的**静态代码审计**生成，覆盖整体架构、模块职责、关键类与函数、依赖关系、编译运行方式与已知问题。
> 审计快照：2026-11-20。未在本环境执行 `colcon build` 与实车运行，文中"运行方式"来自 launch/源码/仓库说明的交叉核对。
> 文档位置：`Tutorial/ROS自动驾驶入门/CodeWiki/` ｜ [返回课程一说明](../README.md)

---

## 1. 项目是什么

一个**单线激光雷达 + 差速底盘**的 ROS 2 割草/配送类小车软件栈，工作空间根目录即仓库根目录（约定为 `~/ros2_ws`），源码在 `src/`。

| 维度 | 内容 |
| --- | --- |
| 目标平台 | Ubuntu 22.04 + ROS 2 Humble |
| 硬件 | CAN 总线底盘、nvilidar 单线激光雷达、串口 IMU、（选配）Orbbec 深度相机 |
| 建图 | Cartographer（主）/ GMapping（对比） |
| 导航 | Nav2 1.1.5 + TEB 局部规划 + NavFn 全局规划 + AMCL 定位 |
| 自研部分 | `yhs_can_control`、`yhs_can_interfaces`、`yhs_chassis_description`、`yhs_nav2`、`yhs_tutorials`、`tools` |
| 第三方部分 | `cartographer`、`cartographer_ros`、`navigation2`、`slam_gmapping`、`teb_local_planner`、`pointcloud_to_laserscan`、`dependence/*`、`hardware/*` |

规模：**63 个 `package.xml`**、14 个非隐藏顶层目录（另有 3 个隐藏配置目录）、约 **1605 个 C/C++ 源文件**。

---

## 2. 三分钟上手

```bash
# 1) 依赖（详见 09 文档）
sudo apt install -y python3-colcon-common-extensions python3-rosdep \
  ros-humble-navigation2 ros-humble-nav2-bringup \
  ros-humble-robot-state-publisher ros-humble-joint-state-publisher ros-humble-xacro \
  python3-transforms3d libceres-dev libprotobuf-dev protobuf-compiler \
  liblua5.3-dev libg2o-dev libeigen3-dev libpcl-dev libyaml-cpp-dev

# 2) 编译
cd ~/ros2_ws
source /opt/ros/humble/setup.bash
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install          # 内存不足加 --parallel-workers 2
source install/setup.bash

# 3) 上电
sudo modprobe can
sudo ip link set can0 up type can bitrate 500000
ls -l /dev/lidar /dev/imu               # 雷达 512000 bps；IMU 115200 bps

# 4) 建图（Cartographer）
ros2 launch yhs_nav2 cartographer.launch.py
# 保存地图
ros2 run nav2_map_server map_saver_cli -f ~/maps/warehouse

# 5) 导航
ros2 launch yhs_nav2 yhs_nav2.launch.py map:=$HOME/maps/warehouse.yaml
ros2 run yhs_nav2 set_init_amcl_pose.py --ros-args -p x:=0.0 -p y:=0.0 -p theta:=0.0
```

---

## 3. 系统链路速览

```
 单线雷达 /scan ─┐
 串口 IMU /imu_data ─┤
 深度相机 /scan1（选配）─┤
                     ├──► Cartographer / GMapping ──► /map + map→odom
 CAN 底盘反馈 ──► /odom ─┘

 /map ──► AMCL ──► map→odom ──► global_costmap ──► NavFn ──► /plan
                                      │
                     local_costmap ◄───┘
                                      ▼
                    TEB (20 Hz) ──► /cmd_vel_nav ──► velocity_smoother
                                      ──► /cmd_vel ──► yhs_can_control ──► CAN 帧 ──► 电机
```

- CAN 控制帧：`0x98C4D1D0`（速度/转角）、`0x98C4D2D0`（左右轮速）、`0x98C4D7D0`（灯光 IO）。
- 反馈帧：`0x98C4D1EF`、`0x98C4D7EF`、`0x98C4D8EF`、`0x98C4DAEF`、`0x98C4E1EF`、`0x98C4E2EF`。
- TF 树：`map → odom → base_link → {laser_link, imu_link, camera_link}`。

---

## 4. 文档目录

| 文档 | 内容 | 适合谁 |
| --- | --- | --- |
| [01 整体架构](01-architecture.md) | 分层、进程拓扑、话题/服务/动作、TF 树、三条数据链 | 所有人，先读 |
| [02 包清单与依赖](02-packages.md) | 63 个包的职责、版本、依赖矩阵与构建顺序 | 架构/集成 |
| [03 底盘控制与 CAN](03-can-chassis.md) | `yhs_can_control` 类与函数、CAN 协议、里程计、消息定义 | 底盘/驱动 |
| [04 硬件与传感器](04-sensors-hardware.md) | nvilidar、serial_imu、Orbbec、pointcloud_to_laserscan | 驱动/集成 |
| [05 SLAM 建图](05-slam.md) | Cartographer 核心库与 ROS 桥、dgt_mini 配置、GMapping | 建图 |
| [06 Nav2 导航栈](06-navigation2.md) | 生命周期、核心服务器、代价地图、AMCL、行为树与示教路径定制 | 导航 |
| [07 局部规划与依赖库](07-local-planning.md) | TEB、costmap_converter、BehaviorTree.CPP | 控制/调参 |
| [08 机器人模型与 TF](08-robot-model.md) | 多车型 URDF/STL、launch、静态 TF、车型切换脚本 | 整车/模型 |
| [09 编译、运行与调试](09-build-run-debug.md) | 环境、编译、三条启动路线、命令速查、故障表 | 运维/实训 |
| [10 已知问题与风险](10-known-issues.md) | 静态审计发现的配置不一致、缺失文件与代码隐患 | 维护者 |
| [11 接口参考字典](11-interface-reference.md) | 全量话题/服务/动作/TF、QoS、消息定义与排查命令 | 联调/集成 |
| [12 文件级代码地图](12-code-map.md) | 按子系统列出入口/核心实现/关键符号，附"需求→文件"速查 | 开发者 |
| [13 调参手册](13-tuning-playbook.md) | 建图/定位/导航的基线、症状→参数矩阵、验收指标 | 算法/现场 |
| [14 二次开发扩展指南](14-extension-guide.md) | 加车型、加传感器、换插件、加 BT 节点/导航器、示教路径拆解 | 开发者 |
| [15 测试与质量保障](15-tests-quality.md) | 测试资产、CI、覆盖缺口、最小验收方案与质量门禁 | 测试/维护者 |
| [16 部署与运维手册](16-deployment-operations.md) | 首次部署、启动编排、systemd、录包、备份、升级回滚、多车限制 | 运维/交付 |
| [17 版本与依赖矩阵](17-version-matrix.md) | 63 个包版本、第三方编译版本、系统依赖、兼容性风险 | 架构/交付 |
| [18 故障诊断 FAQ](18-faq.md) | 按症状组织的排查树：构建、传感器、底盘、建图、导航、性能 | 现场/支持 |
| [19 架构决策记录](19-adr.md) | 11 条 ADR：为何选 Humble/Nav2、Cartographer、CAN 直连、TEB 等 | 架构/维护者 |
| [20 术语表](20-glossary.md) | ROS 2、TF、SLAM、Nav2、底盘与项目专有名词 | 新人，随时查 |

### 推荐阅读顺序

1. 初学者/实训：`01 → 03 → 04 → 08 → 09 → 20`
2. 导航调参：`01 → 06 → 07 → 13 → 18 → 10`
3. 建图：`01 → 04 → 05 → 08 → 13 → 18`
4. 二次开发：`02 → 06 → 12 → 14 → 11 → 19`
5. 接口联调：`01 → 11 → 06 → 09 → 18`
6. 质量/维护：`10 → 15 → 13 → 19`
7. 部署交付：`09 → 16 → 17 → 10 → 18`
8. 排障支持：`18 → 11 → 10 → 13`

---

## 5. 仓库目录结构

```
src/
├── yhs_can_interfaces/        # 自研：底盘 msg / action
├── yhs_can_control/           # 自研：CAN 底盘驱动（cmd_vel → CAN，CAN → odom）
├── yhs_chassis_description/   # 自研：7 种车型 URDF + STL + RViz
├── yhs_nav2/                  # 自研：编排入口（launch / param / map / scripts）
├── yhs_tutorials/             # 自研：教学示例节点
├── tools/                     # select_car.sh、tf.py
├── hardware/
│   ├── nvilidar_ros2/         # 激光雷达驱动 + SDK
│   ├── serial_imu/            # 串口 IMU
│   └── OrbbecSDK_ROS2/        # 深度相机
├── cartographer/              # SLAM 算法库
├── cartographer_ros/          # 建图节点、dgt_mini.lua
├── slam_gmapping/             # GMapping（OpenSlam 封装）
├── navigation2/               # Nav2 1.1.5（37 个包，含 DWB 子包）
├── teb_local_planner/         # TEB 局部规划器
├── pointcloud_to_laserscan/   # 点云转 LaserScan
├── dependence/                # BehaviorTree.CPP v3、costmap_converter
└── COLCON_BUILD.md            # colcon 选项说明
```

---

## 6. 范围与约定

- **事实来源**：源码、`package.xml`、launch、YAML/Lua 配置、`/workspace/README.md` 与 `CHANGELOG.md`。
- **行号**：文中 `文件:行号` 均对应当前快照，源码改动后可能偏移。
- **"已知问题"判定标准**：配置引用不存在文件、包与启动器版本 API 不匹配、明显复制粘贴错误、运行时必然告警/失效的配置。均在 [10-known-issues.md](10-known-issues.md) 给出证据。
- **未验证项**：所有"应发布/应加载"的结论来自代码路径，未经实车或仿真验证。
