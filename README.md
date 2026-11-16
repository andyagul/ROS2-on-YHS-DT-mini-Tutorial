# 《ROS2 自动驾驶入门》、《ROS自动驾驶二次开发》课程资源仓库

面向 **技师学院 · 智能网联汽车技术应用（中级工 / 高级工）** 的专业核心课配套资源仓库。以深圳煜禾森科技 **OS-lite ROS2 单线激光 SLAM 导航套件（DT-mini 等车型）** 为教学载体，包含可编译的 ROS 2 工作空间源码、课程标准与系统架构分析三部分内容。

课程以"交付一台能跑、能建图、能导航、能排故的无人车"为总任务，160 学时、10 学分，共 12 个情景化模块（M0–M12）。

![ROS 2](<https://img.shields.io/badge/ROS%202-Humble-22314E>)
![Ubuntu](https://img.shields.io/badge/Ubuntu-22.04-E95420)
![学时](<https://img.shields.io/badge/%E5%AD%A6%E6%97%B6-160%20%2F%2010%20%E5%AD%A6%E5%88%86-0A6EBD>)
![载体](https://img.shields.io/badge/%E6%95%99%E5%AD%A6%E8%BD%BD%E4%BD%93-DT--mini-green)

---

## 目录

- [仓库内容与使用方式](#仓库内容与使用方式)
- [课程体系](#课程体系)
- [课程定位](#课程定位)
- [课程目标](#课程目标)
- [教学条件](#教学条件)
- [课程内容与学时分配](#课程内容与学时分配)
- [模块—源码—验收要点](#模块源码验收要点)
- [实训环境准备](#实训环境准备)
- [常用命令速查](#常用命令速查)
- [关键参数速查](#关键参数速查)
- [考核评价与实施建议](#考核评价与实施建议)
- [课程二：ROS 自动驾驶二次开发](#课程二ros-自动驾驶二次开发)
- [仓库目录结构](#仓库目录结构)
- [许可证](#许可证)

---

## 仓库内容与使用方式

| 课程 | 目录 | 主要内容 | 适用对象 |
| --- | --- | --- | --- |
| 课程一：ROS 2 自动驾驶入门 | [Tutorial/ROS自动驾驶入门/](Tutorial/ROS自动驾驶入门/README.md) | 课程标准（M0–M12，160 学时）+ 系统架构分析 | 中级工 / 高级工学生、任课教师 |
| 课程二：ROS 自动驾驶二次开发 | [Tutorial/ROS自动驾驶二次开发/](Tutorial/ROS自动驾驶二次开发/README.md) | 33 项开发项目 + 33 个应用场景 + 33 份实验手册 | 竞赛学生、毕业设计、拓展课教师 |
| 公共资源 | `src/` | 可直接编译运行的 ROS 2 工作空间（底盘驱动、传感器、SLAM、Nav2、TEB 等） | 两门课的实车实训与备课演示 |
| 公共资源 | [src/COLCON_BUILD.md](src/COLCON_BUILD.md) | colcon 常用编译选项说明 | M1 部署与环境排错 |

推荐使用路径：

1. **教师（课程一）**：先读课程标准确定模块与学时 → 读架构分析准备讲解与故障注入点 → 在实车验证任务书指标。
2. **学生（课程一）**：按 M0 → M12 顺序推进，每模块完成"任务书 → 实操 → 自检 → 验收 → 复盘"闭环。
3. **竞赛 / 毕设（课程二）**：完成课程一 M0–M10 后，按[项目清单](Tutorial/ROS自动驾驶二次开发/二次开发项目清单.md)选题 → 用[场景方案](Tutorial/ROS自动驾驶二次开发/二次开发场景实施方案.md)设计业务背景 → 按[实验手册](Tutorial/ROS自动驾驶二次开发/README.md)逐步实施与验收。
4. **设备管理员**：按[实训环境准备](#实训环境准备)完成一次整机部署，固化环境与 udev / CAN 配置。

---

## 课程体系

两门课程共用同一套 `src/` 工作空间，按"先会用、再改造"递进：

| 维度 | 课程一：ROS 2 自动驾驶入门 | 课程二：ROS 自动驾驶二次开发 |
| --- | --- | --- |
| 课程性质 | 专业核心课（必修） | 拓展 / 竞赛方向课（选修） |
| 建议学时 | 160（理论 48 + 实操 112） | 48~64（可按项目裁剪） |
| 先修要求 | 无；建议具备 Linux 与 Python 基础 | 完成课程一 M0–M10 |
| 教学目标 | 能完成整机部署、建图、定位、导航调试 | 能独立完成参数优化、脚本节点与完整功能包开发 |
| 核心资料 | 课程标准、系统架构分析 | 项目清单、场景实施方案、实验手册 |
| 成果形式 | 12 个模块任务书与实车验收 | 33 份实验记录 + 1 个自建功能包 |
| 入口 | [课程一说明](Tutorial/ROS自动驾驶入门/README.md) | [课程二说明](Tutorial/ROS自动驾驶二次开发/README.md) |

> 说明：下文的"课程定位""课程目标""教学条件""课程内容与学时分配""模块—源码—验收要点""考核评价与实施建议"均针对**课程一**；课程二的教学安排见后文独立章节。

---

## 课程定位

### 课程性质

面向低速无人车（园区货运车、仓储搬运车、巡检车）装调与运维岗位群，以一台真实单线激光导航小车为教学载体，培养学生在真实工作场景中完成部署、操作、调试与故障处置的能力。

### 对接岗位与典型工作任务

| 岗位           | 典型工作任务                                     | 对应模块          |
| -------------- | ------------------------------------------------ | ----------------- |
| 无人车装调技师 | 传感器安装、CAN 联调、整车通电测试、参数配置     | M3、M4、M5、M6    |
| 无人车运维技师 | 地图采集与更新、定位失效处置、路径与避障参数维护 | M7、M8、M9、M10   |
| 无人车测试员   | 交付验收、功能验证、故障复现与记录               | M11、M12          |
| 现场安全员     | 作业区布置、急停值守、限速与清场                 | M0 及全部实操模块 |

### 设计思路

1. **场景主线**：以"园区货运—仓储搬运—变电站巡检"三类低速无人车场景贯穿全课程，每个模块对应一个可交付的现场任务。
2. **任务闭环**：每个模块按"接任务—做准备—做操作—验结果—写记录"组织，学生始终清楚"为什么做、做完交付什么"。
3. **能力递进**：安全认知 → 系统部署 → 执行与感知 → 系统集成 → 建图定位 → 规划控制 → 决策与验收；学时按难度分配（8~20 学时不等）。
4. **排故导向**：每个模块设置故障注入与排查环节，训练"先看数据、再看 TF、最后改参数"的工程习惯。
5. **硬件一致**：全部案例只使用本套件源码与设备，不引入 TurtleBot3、Gazebo 仿真、阿克曼车型、视觉深度学习等无关内容。

### 前后衔接

| 关系 | 课程                                                    | 支撑内容                           |
| ---- | ------------------------------------------------------- | ---------------------------------- |
| 前序 | Linux 基础、C/Python 程序设计、汽车电工电子、传感器技术 | 终端操作、代码阅读、CAN 与串口认知 |
| 后续 | 无人车运维综合实训、智能网联汽车装调考证                | 建图导航、故障诊断、交付验收       |

---

## 课程目标

### 总目标

学生应能遵循无人车企业质量管理、安全生产与 9S 管理制度，按照低速无人车装调与运维技术操作规程，面向园区货运、仓储搬运、变电站巡检三类场景，完成工控机与 ROS 2 环境部署、底盘 CAN 联调、激光雷达与 IMU 装调标定、整车系统集成、SLAM 建图、定位与代价地图配置、全局规划与局部避障调试、巡检路线示教及交付验收等工作任务，并具备规范意识、安全意识、质量意识与团队协作能力。

### 分目标（与模块对应）

| 序号 | 分目标要点                                                                                                                                                                            | 模块     |
| ---- | ------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- | -------- |
| 1    | 读懂交付验收单与整车接口图，完成整车点检、接口标注、作业区布置与急停演练（急停响应 ≤ 3 s）                                                                                           | M0       |
| 2    | 完成系统检查、依赖安装、`colcon build` 与环境变量固化，编译零失败，能处理 ≥ 2 类编译/依赖错误                                                                                      | M1       |
| 3    | 启动感知驱动与订阅节点，绘制 `nvilidar_ros2_node→/scan→lidar_pkg_node` 链路，记录频率与有效点数，完成示例修改与 bag 回放                                                          | M2       |
| 4    | 配置 `can0`、核对上下行报文、验证速度指令并完成里程计标定（直线 1 m 误差 ≤ 5%、转向 90° 误差 ≤ 5°、限速 0.2 m/s 生效）                                                          | M3       |
| 5    | 完成设备名与波特率配置、角度量程调整、静态 TF 标定与多车型模型核验（`/scan` ≥ 5 Hz 且有效点 ≥ 300、`/imu_data` 50~100 Hz、TF 偏差 ≤ 10 mm）                                    | M4、M5   |
| 6    | 按顺序启动感知、底盘、定位与导航节点，核查生命周期与参数注入（3 min 内启动、Nav2 服务器 ≥ 9 个、抽查节点均 active）                                                                  | M6       |
| 7    | 规划闭环采集路线，以 ≤ 0.2 m/s 完成 Cartographer 建图与 Gmapping 对比，交付闭合误差 ≤ 0.1 m 的地图包                                                                                | M7       |
| 8    | 完成地图加载、初始定位、粒子云收敛判断与局部膨胀层修正（3 s 内收敛、定位误差 ≤ 0.1 m）                                                                                               | M8       |
| 9    | 完成全局路径生成与通行性验证，并完成限速与窄通道避障调试（2 s 内出路径、3 个目标点误差 ≤ 0.15 m / 0.15 rad、绕障距离 ≥ 0.25 m、`/cmd_vel` 角速度 ≤ 0.3 rad/s 且 20 Hz 平滑输出） | M9、M10  |
| 10   | 完成行为树结构核验与修改、示教路径循迹与随机故障处置，并通过验收答辩（横向误差 ≤ 0.2 m、10 min 内定位并处置、一次修复率 ≥ 70%）                                                     | M11、M12 |

---

## 教学条件

### 硬件条件（每 2 名学生 1 套）

| 设备                 | 型号 / 规格                                  | 教学用途                            |
| -------------------- | -------------------------------------------- | ----------------------------------- |
| 导航底盘             | 煜禾森 DT-mini（或 DT-mid / FR-mid 等）      | 运动执行、里程计、CAN 通信          |
| 单线激光雷达         | nvilidar（串口，512000 bps）                 | `/scan`，建图/定位/避障唯一激光源 |
| 串口 IMU             | CH100 / CH110（115200 bps）                  | `/imu_data`，底盘航向校准         |
| 工控机               | Ubuntu 22.04 + ROS 2 Humble                  | 运行全部节点                        |
| CAN 适配器           | SocketCAN（`can0`）                        | 指令下发与状态反馈                  |
| 深度相机（选配）     | Orbbec Astra / Gemini2                       | `/scan1`、点云实验                |
| 急停开关             | 硬件急停                                     | 所有上车实验强制使用                |
| 测试场地             | ≥ 4 m × 4 m，含 2~3 处特征墙角、1 条窄通道 | 货运 / 仓储 / 巡检场景模拟          |
| 锥桶、挡板、警戒线   | 若干                                         | 动态障碍、作业区隔离                |
| 卷尺、角度尺、万用表 | —                                           | 里程计、定位精度与供电测量          |

### 软件条件

| 软件                            | 版本                                                     | 用途                               |
| ------------------------------- | -------------------------------------------------------- | ---------------------------------- |
| Ubuntu                          | 22.04 LTS                                                | 与 ROS 2 Humble 对应               |
| ROS 2                           | Humble                                                   | 通信、TF2、launch、RViz2           |
| Nav2                            | 1.1.5（工作空间源码）                                    | 定位、代价地图、规划、控制、行为树 |
| Cartographer / cartographer_ros | 2.0                                                      | 主建图方案                         |
| slam_gmapping                   | 工作空间源码                                             | 备选建图对比                       |
| teb_local_planner               | 0.9.1                                                    | 局部控制器                         |
| 调试工具                        | `rqt_graph`、`rqt_plot`、`ros2 bag`、`tf2_tools` | 观察、记录、复盘                   |

### 场地与安全条件

1. 场地平整、无积水、光照稳定，四周设警戒线；
2. 每台车配备有效急停，安全员全程持握；
3. 首次上车限速 0.2 m/s、角速度 ≤ 0.3 rad/s；
4. 架空车轮完成指令验证后，方可落地行驶；
5. 电池电压低于厂家阈值立即停止并充电，禁止过放。

### 不纳入的内容

TurtleBot3 模型与示例地图、Gazebo / Ignition 仿真、`tb3_simulation_launch.py`、阿克曼 / 四轮转向车型、SMAC 规划器调参、多雷达融合、视觉 SLAM 与深度学习目标检测、多机编队。

---

## 课程内容与学时分配

### 模块总览（160 学时 = 理论 48 + 实操 112）

| 模块 | 情景化模块名称                           | 场景      | 学时          | 理论         | 实操          | 对应源码                                                               |
| ---- | ---------------------------------------- | --------- | ------------- | ------------ | ------------- | ---------------------------------------------------------------------- |
| M0   | 园区货运无人车交付前的安全交底与整车认知 | 货运      | 8             | 4            | 4             | 全车实物、`yhs_chassis_description`                                  |
| M1   | 仓储搬运无人车工控机部署与工作空间构建   | 仓储      | 12            | 2            | 10            | 全工作空间、`COLCON_BUILD.md`                                        |
| M2   | 仓储 AGV 感知数据链路点检与通信验证      | 仓储      | 8             | 2            | 6             | `yhs_tutorials/*`                                                    |
| M3   | 园区货运车底盘 CAN 联调与里程计标定      | 货运      | 16            | 4            | 12            | `yhs_can_interfaces`、`yhs_can_control`                            |
| M4   | 变电站巡检车激光雷达与 IMU 装调标定      | 巡检      | 12            | 3            | 9             | `nvilidar_ros2`、`serial_imu`、`pointcloud_to_laserscan`         |
| M5   | 多车型交付前的模型与 TF 坐标链核验       | 货运/仓储 | 8             | 3            | 5             | `yhs_chassis_description`                                            |
| M6   | 无人车整机开机流程与系统联调             | 仓储      | 8             | 2            | 6             | `yhs_nav2/launch/*`                                                  |
| M7   | 仓储库区高精度地图采集与质检             | 仓储      | 20            | 6            | 14            | `cartographer_ros`、`slam_gmapping`                                |
| M8   | 巡检车地图加载、初始定位与代价地图配置   | 巡检      | 16            | 5            | 11            | `nav2_amcl`、`nav2_map_server`、`nav2_costmap_2d`                |
| M9   | 园区货运线路全局规划与通行性验证         | 货运      | 12            | 4            | 8             | `nav2_planner`、`nav2_navfn_planner`                               |
| M10  | 窄通道会车与动态避障的局部控制调试       | 仓储/货运 | 20            | 6            | 14            | `teb_local_planner`、`nav2_controller`、`nav2_velocity_smoother` |
| M11  | 变电站巡检路线示教、循迹与异常恢复       | 巡检      | 12            | 4            | 8             | `nav2_bt_navigator`、`nav2_behavior_tree`、`RecordPath.action`   |
| M12  | 无人车交付验收、故障应急处置与运维交接   | 综合      | 8             | 3            | 5             | 全栈                                                                   |
| 合计 | —                                       | —        | **160** | **48** | **112** | —                                                                     |

> 学时依据：基础认知类 8~12 学时；硬件联调与参数整定（M3、M4、M8、M10）12~20 学时；建图 M7 因场地采集与参数对比耗时最长，安排 20 学时；综合验收 M12 独立 8 学时，便于组织完整交付流程考核。

### 20 周教学进度（每周 8 学时）

| 周次 | 模块与学时         | 周任务成果                             |
| ---- | ------------------ | -------------------------------------- |
| 1    | M0（8）            | 安全考核通过、整车接口标注完成         |
| 2    | M1（8）            | Ubuntu 与 ROS 2 环境可用               |
| 3    | M1（4）+ M2（4）   | 工作空间编译通过；能启动雷达并观察话题 |
| 4    | M2（4）+ M3（4）   | 通信链路验收；CAN 配置完成             |
| 5    | M3（8）            | `/cmd_vel→/odom` 闭环打通           |
| 6    | M3（4）+ M4（4）   | 里程计标定完成；雷达出数               |
| 7    | M4（8）            | 雷达 / IMU 参数与 TF 验收通过          |
| 8    | M5（8）            | 车型模型与 TF 链核验通过               |
| 9    | M6（8）            | 全栈一键启动与生命周期验收             |
| 10   | M7（8）            | 完成首轮场地地图采集                   |
| 11   | M7（8）            | 完成参数对比与地图优化                 |
| 12   | M7（4）+ M8（4）   | 地图归档；地图加载成功                 |
| 13   | M8（8）            | 初始定位与代价地图调通                 |
| 14   | M8（4）+ M9（4）   | 定位精度验收；全局路径可视             |
| 15   | M9（8）            | 完成 3 条货运线路通行性验证            |
| 16   | M10（8）           | TEB 基础跟踪与限速验收                 |
| 17   | M10（8）           | 窄通道与动态避障调试                   |
| 18   | M10（4）+ M11（4） | 速度平滑验收；行为树结构核验           |
| 19   | M11（8）           | 巡检路线示教与循迹验收                 |
| 20   | M12（8）           | 交付验收综合考核                       |

---

## 模块—源码—验收要点

| 模块 | 关键实训任务                                             | 主要验收指标                                               | 主要源码 / 配置                                                                                                                                                                                        |
| ---- | -------------------------------------------------------- | ---------------------------------------------------------- | ------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------ |
| M0   | 整车点检、六类接口标注、急停演练                         | 接口标注零漏项、急停响应 ≤ 3 s                            | 实车、[urdf](src/yhs_chassis_description/urdf)、[robot.launch.py](src/yhs_chassis_description/launch/robot.launch.py)                                                                                    |
| M1   | 依赖安装、`colcon build`、环境变量固化                 | 无 failed 包、6 个关键包可检索、新终端免 source            | 全工作空间、[COLCON_BUILD.md](src/COLCON_BUILD.md)                                                                                                                                                      |
| M2   | 雷达/点云订阅、话题频率与 QoS 核对、bag 回放             | 链路图完整、频率与有效点数达标                             | `src/yhs_tutorials/lidar_pkg`、`pc_pkg`、`vel_pub`、`nav_pkg`                                                                                                                                  |
| M3   | `can0` 配置、报文核对、速度验证、里程计标定            | 直线 1 m 误差 ≤ 5%、转向 90° 误差 ≤ 5°                 | [yhs_can_interfaces/msg](src/yhs_can_interfaces/msg)、[yhs_can_control_node.cpp](src/yhs_can_control/src/yhs_can_control_node.cpp)、[cfg.yaml](src/yhs_can_control/params/cfg.yaml)                       |
| M4   | 雷达 / IMU 装调、角度量程、静态 TF 标定                  | `/scan` ≥ 5 Hz 且有效点 ≥ 300、`/imu_data` 50~100 Hz | `src/hardware/nvilidar_ros2`、`serial_imu`、[pointcloud_to_laserscan.launch.py](src/pointcloud_to_laserscan/launch/pointcloud_to_laserscan.launch.py)                                               |
| M5   | 多车型模型核对、TF 链核验                                | TF 与实测偏差 ≤ 10 mm、模型与实车一致                     | `urdf/*.urdf`、`meshes/*.STL`、`launch/DT-mini.launch.py`                                                                                                                                        |
| M6   | 按 SOP 启动整机、生命周期核查、`use_tf` 差异说明       | 3 min 内启动、Nav2 服务器 ≥ 9 个、节点均 active           | [yhs_nav2.launch.py](src/yhs_nav2/launch/yhs_nav2.launch.py)、`localization_launch.py`、`navigation_launch.py`                                                                                      |
| M7   | 闭环采集路线、Cartographer 建图、Gmapping 对比、地图保存 | 闭合误差 ≤ 0.1 m、交付 `.pgm/.yaml` 地图包              | [dgt_mini.launch.py](src/cartographer_ros/cartographer_ros/launch/dgt_mini.launch.py)、[dgt_mini.lua](src/cartographer_ros/cartographer_ros/configuration_files/dgt_mini.lua)、`src/slam_gmapping`     |
| M8   | 地图加载、初始定位、粒子云收敛判断、局部膨胀层修正       | 3 s 内收敛、定位误差 ≤ 0.1 m、膨胀梯度可见                | `src/navigation2/nav2_amcl`、`nav2_map_server`、[yhs_nav2.yaml](src/yhs_nav2/param/yhs_nav2.yaml)                                                                                                   |
| M9   | 三条货运线路全局规划与通行性验证                         | 2 s 内出路径、目标点误差 ≤ 0.15 m / 0.15 rad              | `src/navigation2/nav2_planner`、`nav2_navfn_planner`                                                                                                                                               |
| M10  | 限速、窄通道会车、动态避障、速度平滑                     | 绕障 ≥ 0.25 m、角速度 ≤ 0.3 rad/s、20 Hz 平滑输出        | `src/teb_local_planner`、[costmap_converter](src/dependence/costmap_converter)、`nav2_velocity_smoother`                                                                                            |
| M11  | 行为树结构核验、示教路径发布与循迹、故障处置             | 循迹横向误差 ≤ 0.2 m、10 min 内定位并处置                 | [navigate_follow_record_path_action.cpp](src/navigation2/nav2_behavior_tree/plugins/action/navigate_follow_record_path_action.cpp)、[RecordPath.action](src/yhs_can_interfaces/action/RecordPath.action) |
| M12  | 交付验收、故障应急、运维资料整理与答辩                   | 一次修复率 ≥ 70%、5 min 答辩                              | 全栈                                                                                                                                                                                                   |

### 教学用系统链路

M2 至 M6 模块的讲解主线，始终是这一条链路：

```
/scan ──► AMCL ──► map→odom ──► global_costmap ──► NavFn ──► /plan
                                                              │
/local_costmap ◄── /scan + /odom ─────────────────────────────┤
                                                              ▼
                                                  TEB (20 Hz) ──► cmd_vel_nav
                                                              │
                                                    velocity_smoother
                                                              ▼
                                   /odom + odom→base_link ◄── yhs_can_control ◄── cmd_vel
                                                              │
                                                     CAN → 电机
```

### 推荐学习顺序

1. 硬件与坐标系：`yhs_chassis_description` → `nvilidar_ros2` → `serial_imu`
2. 执行链路：`yhs_can_interfaces` → `yhs_can_control`
3. 编排与参数：`yhs_nav2`（launch + yaml + map）
4. 建图：`cartographer_ros` → `slam_gmapping`
5. 导航：`nav2_amcl` / `nav2_map_server` → `nav2_navfn_planner` → `teb_local_planner`
6. 决策扩展：`nav2_behavior_tree` + `RecordPath`

---

## 课程二：ROS 自动驾驶二次开发

### 课程性质与目标

面向已完成课程一、具备建图导航调试能力的学生。以"改参数 → 写脚本 → 建功能包"三级递进为主线，把实车能力转化为可交付的二次开发成果，适用于竞赛训练、毕业设计与教师拓展课。

### 三级项目结构

| 阶段 | 开发形式 | 项目数 | 典型内容 | 产出 |
| --- | --- | --- | --- | --- |
| 第一阶段 | 仅改参数（零代码） | 16 | 限速与加速度、到点容差、膨胀层、代价地图、AMCL、建图参数、TF 归属、规划器选型 | 参数变更单 + 前后对比数据 |
| 第二阶段 | 单文件脚本节点 | 10 | 状态看板、低电量告警、防跌落停车、灯光联动、速度看门狗、激光自检、打滑预警、巡航 | 可运行脚本 + 测试记录 |
| 第三阶段 | 完整功能包 | 7 | 健康管理、安全联锁、声光表达、巡检调度、示教复现、地图资产、数据黑匣子 | 可编译功能包 + 验收报告 |

### 教学资源

| 资源 | 说明 |
| --- | --- |
| [二次开发项目清单](Tutorial/ROS自动驾驶二次开发/二次开发项目清单.md) | 33 个功能项，逐项标注阶段、操作内容、预期效果与验证方式 |
| [场景实施方案](Tutorial/ROS自动驾驶二次开发/二次开发场景实施方案.md) | 33 个真实场景：应用环境、服务对象、业务价值、功能边界、实施步骤与量化验收 |
| [实验手册](Tutorial/ROS自动驾驶二次开发/README.md) | 33 份实验文档，统一 10 章结构：目录文件、完整代码、编译命令、运行验证、回滚与排故 |

### 学时与考核建议

- 学时：第一阶段 8 学时、第二阶段 12 学时、第三阶段 24~32 学时；也可只选一个阶段作为 16 学时拓展课。
- 考核：按实验文档内置的三维指标评分——功能完整性（场景通过率、覆盖点数）、安全性（停车响应 ≤ 0.3~0.5 s、最小间距）、稳定性（连续运行时长、任务成功率 ≥ 95%）。
- 安全边界：全部场景限定在园区、库区、变电站等非公开道路，限速 ≤ 0.5 m/s（首期 0.2 m/s），安全员持硬件急停随行。

---

## 实训环境准备

> 供 M1 模块及设备管理员使用。完整步骤以课程标准 M1 任务书为准。

### 1. 系统与 ROS 2

Ubuntu 22.04 + ROS 2 Humble（桌面版）：

```bash
sudo apt install -y software-properties-common curl
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
  -o /usr/share/keyrings/ros-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] \
http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" \
| sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null
sudo apt update && sudo apt install ros-humble-desktop
```

### 2. 依赖安装

```bash
sudo apt install -y python3-colcon-common-extensions python3-rosdep \
  ros-humble-navigation2 ros-humble-nav2-bringup \
  ros-humble-robot-state-publisher ros-humble-joint-state-publisher ros-humble-xacro \
  python3-transforms3d

# Cartographer / TEB 编译依赖
sudo apt install -y libceres-dev libprotobuf-dev protobuf-compiler \
  liblua5.3-dev libg2o-dev libeigen3-dev libpcl-dev libyaml-cpp-dev

sudo rosdep init     # 已初始化会报错，可忽略
rosdep update
```

### 3. 获取源码与编译

**路径约定**：本仓库克隆后即为 ROS 2 工作空间根目录，统一使用 `~/ros2_ws`；源码目录为 `~/ros2_ws/src`，`install/`、`build/` 由 `colcon` 在该目录下生成。Tutorial 与实验手册中出现的 `$WS` 均指 `~/ros2_ws`（`export WS="$HOME/ros2_ws"`），若放在其他位置，请同步修改该变量或对应路径。

```bash
git clone https://github.com/andyagul/ROS2-on-YHS-DT-mini-Tutorial.git ~/ros2_ws
cd ~/ros2_ws

source /opt/ros/humble/setup.bash
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install
source install/setup.bash
```

内存不足时加 `--parallel-workers 2`；只编单个模块用 `--packages-select <pkg>`，选项说明见 [COLCON_BUILD.md](src/COLCON_BUILD.md)。

### 4. 设备与总线上电检查

```bash
# CAN
sudo modprobe can
sudo ip link set can0 up type can bitrate 500000
candump can0                        # 有报文说明接线与波特率正确

# 串口权限与设备名
sudo usermod -aG dialout $USER      # 重新登录生效
ls -l /dev/lidar /dev/imu           # 雷达 512000 bps，IMU 115200 bps
```

> 雷达串口在 [nvilidar.yaml](src/hardware/nvilidar_ros2/params/nvilidar.yaml) 中配置（`/dev/lidar`、512000）；IMU 路径在源码中硬编码为 `/dev/imu`，需 udev 规则；CAN 接口名见 [cfg.yaml](src/yhs_can_control/params/cfg.yaml)。车型切换使用 [select_car.sh](src/tools/select_car.sh)（脚本内路径需按实际工作空间修改）。

### 5. 建图与导航入口

```bash
# 建图（Cartographer，主方案）
ros2 launch yhs_nav2 cartographer.launch.py

# 建图（Gmapping，对比用）
ros2 launch yhs_nav2 gmapping.launch.py

# 保存地图（M7 交付物）
mkdir -p ~/maps
ros2 run nav2_map_server map_saver_cli -f ~/maps/warehouse

# 导航（默认地图 ecarroom.yaml）
ros2 launch yhs_nav2 yhs_nav2.launch.py map:=$HOME/maps/warehouse.yaml

# 初始位姿
ros2 run yhs_nav2 set_init_amcl_pose.py --ros-args -p x:=0.0 -p y:=0.0 -p theta:=0.0
```

---

## 常用命令速查

```bash
# 环境
source /opt/ros/humble/setup.bash
source ~/ros2_ws/install/setup.bash
colcon build --symlink-install --packages-select yhs_nav2

# 底盘
sudo ip link set can0 up type can bitrate 500000
ros2 launch yhs_can_control yhs_can_control.launch.py use_tf:=true
ros2 topic echo /odom --once

# 传感器
ros2 launch nvilidar_ros2 nvilidar.launch.py
ros2 launch serial_imu imu_spec_msg.launch.py
ros2 topic hz /scan --qos-reliability best_effort
ros2 topic hz /imu_data

# 模型与 TF
ros2 launch yhs_chassis_description robot.launch.py
ros2 run tf2_tools view_frames
ros2 run tf2_ros tf2_echo map base_link

# 建图
ros2 launch yhs_nav2 cartographer.launch.py
ros2 run nav2_map_server map_saver_cli -f ~/maps/warehouse

# 导航
ros2 launch yhs_nav2 yhs_nav2.launch.py map:=$HOME/maps/warehouse.yaml
ros2 run yhs_nav2 set_init_amcl_pose.py --ros-args -p x:=0.0 -p y:=0.0 -p theta:=0.0

# 调试
ros2 node list
ros2 lifecycle get /amcl
ros2 topic hz /cmd_vel
ros2 topic echo /behavior_tree_log
rqt_graph
```

### 典型故障与处置（教学用）

| 现象                               | 原因                       | 处理                                                                | 关联模块 |
| ---------------------------------- | -------------------------- | ------------------------------------------------------------------- | -------- |
| `ros2` 命令不存在                | 未 source / 未安装         | `source /opt/ros/humble/setup.bash`                               | M1       |
| 缺少 `nav2_*`                    | 依赖未装                   | `sudo apt install ros-humble-navigation2 ros-humble-nav2-bringup` | M1       |
| `libg2o` / `transforms3d` 缺失 | TEB / Python 依赖缺失      | `libg2o-dev`、`python3-transforms3d`                            | M1、M10  |
| 编译内存不足                       | 并行度过高                 | `--parallel-workers 2`                                            | M1       |
| `can0` 起不来 / `/odom` 无数据 | CAN 未配置或接线异常       | `modprobe can`、`ip link`、`candump can0`                     | M3       |
| `/scan` 无数据                   | 设备名、波特率或 QoS 不符  | 检查 `/dev/lidar`、512000、best-effort 订阅                       | M2、M4   |
| `/imu_data` 无数据               | 硬编码 `/dev/imu` 不存在 | 配 udev 规则、确认 `dialout` 权限                                 | M4       |
| 定位漂移                           | 初始位姿或地图不匹配       | 重设初始位姿、确认加载对应场地地图                                  | M8       |
| 规划成功但车不动                   | `cmd_vel` 链路断开       | 查 `/cmd_vel_nav` → `/cmd_vel` → `candump can0`             | M9、M10  |
| 频繁触发恢复行为                   | 障碍距离 / 膨胀参数不当    | 调整 `min_obstacle_dist`、`inflation_dist`                      | M10      |

更多排故细节见《[ROS2小车系统架构分析](Tutorial/ROS自动驾驶入门/ROS2小车系统架构分析.md)》第 6、8 节。

---

## 关键参数速查

| 参数                                            | 所在文件                                                                              | 教学建议值                  | 作用                |
| ----------------------------------------------- | ------------------------------------------------------------------------------------- | --------------------------- | ------------------- |
| `max_vel_x`                                   | [yhs_nav2.yaml](src/yhs_nav2/param/yhs_nav2.yaml)（TEB）                               | 0.2 → 0.5                  | 线速度上限          |
| `max_vel_theta`                               | 同上                                                                                  | 0.3                         | 角速度上限          |
| `acc_lim_x` / `acc_lim_theta`               | 同上                                                                                  | 0.3                         | 加速度上限          |
| `min_obstacle_dist`                           | 同上                                                                                  | 0.45                        | 障碍安全距离        |
| `inflation_dist`                              | 同上                                                                                  | 0.6                         | 膨胀距离            |
| `xy_goal_tolerance`                           | 同上                                                                                  | 0.1                         | 到点位置容差        |
| `max_beams`                                   | 同上（AMCL）                                                                          | 48                          | 更新光束数          |
| `min_particles` / `max_particles`           | 同上（AMCL）                                                                          | 500 / 5000                  | 粒子数范围          |
| `serialport_name` / `serialport_baud`       | [nvilidar.yaml](src/hardware/nvilidar_ros2/params/nvilidar.yaml)                       | `/dev/lidar` / `512000` | 雷达串口            |
| `can_name`                                    | [cfg.yaml](src/yhs_can_control/params/cfg.yaml)                                        | `can0`                    | CAN 接口            |
| `num_range_data` / `optimize_every_n_nodes` | [dgt_mini.lua](src/cartographer_ros/cartographer_ros/configuration_files/dgt_mini.lua) | 60 / 5                      | 子图帧数 / 优化频率 |

---

## 考核评价与实施建议

### 考核评价

| 项目       | 比例 | 说明                               |
| ---------- | ---- | ---------------------------------- |
| 过程性考核 | 30%  | 出勤、安全规范、6S、每模块验收记录 |
| 模块实操   | 40%  | M1~M11 共 11 次，取平均            |
| 综合验收   | 30%  | M12 任务书评分                     |

### 分层要求

| 层次         | 要求                                       |
| ------------ | ------------------------------------------ |
| 基础（全体） | 完成 M0~M8，能部署、建图、定位、导航       |
| 进阶（多数） | 完成 M9~M11 参数对照与自定义行为树         |
| 拓展（竞赛） | 记录路径优化、动态避障策略、故障注入库建设 |

### 教学组织与教法

- **教学组织**：2 人 1 台，A 岗操作手、B 岗安全员兼记录员，每 20 min 轮换；每模块按"任务书下发 → 方案确认 → 实操 → 自检 → 教师验收 → 复盘"六步组织；教师台保留 1 套备用雷达、IMU、CAN 线用于快速排故。
- **教学方法**：任务驱动（交付单 / 验收单 / 排故单）、故障注入（每模块 1~2 个预设故障）、对照实验（每次只改一个参数）、角色扮演（客户方 / 交付方 / 安全员）、复盘会（每组 5 min）。
- **教学资源**：模块任务书、验收单、故障库、参数记录表、演示视频，配合本仓库源码与两份 Tutorial 文档。

---

## 仓库目录结构

```
~/ros2_ws/
├── README.md                          # 本文件：两门课程的资源导览
├── Tutorial/
│   ├── ROS自动驾驶入门/                # 课程一（必修，160 学时）
│   │   ├── README.md                  # 课程说明、资料导航与学习路径
│   │   ├── ROS2自动驾驶入门课程标准.md  # 定位/目标/条件/M0-M12/考核
│   │   └── ROS2小车系统架构分析.md      # 逐包/逐话题/逐 launch 剖析
│   └── ROS自动驾驶二次开发/             # 课程二（选修，48~64 学时）
│       ├── README.md                  # 实验手册索引与统一约定
│       ├── 二次开发项目清单.md          # 33 项：参数级/脚本级/功能包级
│       ├── 二次开发场景实施方案.md      # 33 个应用场景与实施步骤
│       ├── 第一阶段-参数级/             # S1-01 ~ S1-16
│       ├── 第二阶段-脚本级/             # S2-01 ~ S2-10
│       └── 第三阶段-功能包级/           # S3-01 ~ S3-07
└── src/                              # ROS 2 工作空间源码（~/ros2_ws/src）
    ├── yhs_can_interfaces/           # M3 底盘 msg / action 定义
    ├── yhs_can_control/              # M3 CAN 驱动（M3 核心）
    ├── yhs_chassis_description/      # M0 / M5 多车型 URDF 与 STL
    ├── yhs_nav2/                     # M6 编排入口
    │   ├── launch/                   # yhs_nav2 / cartographer / gmapping /
    │   │                             # localization / navigation
    │   ├── param/yhs_nav2.yaml       # M8-M11 参数中心
    │   ├── map/                      # 预建场地地图（8 套可用）
    │   └── scripts/                  # set_init_amcl_pose.py
    ├── hardware/
    │   ├── nvilidar_ros2/            # M2 / M4 激光雷达
    │   ├── serial_imu/               # M2 / M4 串口 IMU
    │   └── OrbbecSDK_ROS2/           # M4 选配深度相机
    ├── cartographer/                 # M7 SLAM 算法库
    ├── cartographer_ros/             # M7 建图节点与 dgt_mini.lua
    ├── slam_gmapping/                # M7 备选建图
    ├── navigation2/                  # M8-M11 Nav2 1.1.5（32 个子包）
    ├── teb_local_planner/            # M10 局部规划器
    ├── pointcloud_to_laserscan/      # M4 点云转激光（/scan1）
    ├── dependence/                   # BehaviorTree.CPP、costmap_converter
    ├── yhs_tutorials/                # M2 教学示例节点
    ├── tools/                        # select_car.sh、tf.py
    └── COLCON_BUILD.md               # M1 编译选项
```

---

## 许可证

本仓库为课程资源，根目录未声明统一许可证；`yhs_can_control`、`yhs_nav2` 等自研包的 `package.xml` 许可证字段仍为模板默认值，校内教学使用前请与煜禾森（YHS）确认授权方式，未经许可不得用于商业分发。

内置开源组件保留其原始许可证：Cartographer（Apache 2.0）、Navigation2（LGPL-2.1-or-later）、TEB Local Planner（BSD）、nvilidar_ros2（MIT）、OrbbecSDK_ROS2（Apache 2.0）、BehaviorTree.CPP（MIT）、costmap_converter（BSD），以各自目录内 `LICENSE` 为准。

教学载体与原始套件由深圳煜禾森科技有限公司提供；课程标准依据本工作空间《ROS2 小车系统架构分析》编制。
