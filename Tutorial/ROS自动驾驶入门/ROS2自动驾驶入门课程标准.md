# 《ROS2 自动驾驶入门》课程标准

> 适用专业：技师学院 · 智能网联汽车技术应用（中级工 / 高级工）
> 课程性质：专业核心课
> 教学载体：深圳煜禾森科技 OS-lite ROS2 单线激光 SLAM 导航套件（DT-mini 等车型）
> 软件基线：Ubuntu 22.04 + ROS 2 Humble + Nav2 1.1.5 + Cartographer 2.0 + TEB 0.9.1
> 学时学分：160 学时 / 10 学分（20 周 × 8 学时）
> 编制依据：本工作空间《ROS2 小车系统架构分析》及其“推荐学习顺序”

---

## 一、课程定位与设计思路

### 1.1 课程性质与定位

本课程面向低速无人车（园区货运车、仓储搬运车、巡检车）装调与运维岗位群，以一台真实单线激光导航小车为教学载体，以“交付一台能跑、能建图、能导航、能排故的无人车”为总任务，培养学生在真实工作场景中完成部署、操作、调试与故障处置的能力。

课程对接岗位的典型工作任务：

| 岗位 | 典型工作任务 | 对应模块 |
|---|---|---|
| 无人车装调技师 | 传感器安装、CAN 联调、整车通电测试、参数配置 | M3、M4、M5、M6 |
| 无人车运维技师 | 地图采集与更新、定位失效处置、路径与避障参数维护 | M7、M8、M9、M10 |
| 无人车测试员 | 交付验收、功能验证、故障复现与记录 | M11、M12 |
| 现场安全员 | 作业区布置、急停值守、限速与清场 | M0 及全部实操模块 |

### 1.2 设计思路

1. **场景主线**：以“园区货运—仓储搬运—变电站巡检”三类低速无人车场景贯穿全课程，每个模块对应一个可交付的现场任务，而不是孤立的知识点。
2. **任务闭环**：每个模块按“接任务—做准备—做操作—验结果—写记录”组织，学生始终知道“为什么做、做完交付什么”。
3. **能力递进**：按“安全认知 → 系统部署 → 执行与感知 → 系统集成 → 建图定位 → 规划控制 → 决策与验收”推进，模块学时按难度分配（8~20 学时不等），不搞平均主义。
4. **排故导向**：每个模块都设置故障注入与排查环节，训练“先看数据、再看 TF、最后改参数”的工程习惯。
5. **硬件一致**：全部案例只使用本套件源码与设备，不引入 TurtleBot3、Gazebo 仿真、阿克曼车型、视觉深度学习等无关内容。

### 1.3 与前序、后续课程衔接

| 关系 | 课程 | 支撑内容 |
|---|---|---|
| 前序 | Linux 基础、C/Python 程序设计、汽车电工电子、传感器技术 | 终端操作、代码阅读、CAN 与串口认知 |
| 后续 | 无人车运维综合实训、智能网联汽车装调考证 | 建图导航、故障诊断、交付验收 |

---

## 二、课程目标

### 2.1 总目标

学习完本课程后，学生应能基本遵循无人车企业质量管理、安全生产、9S 管理及设备工具材料使用制度，按照低速无人车装调与运维技术操作规程，在教师指导下，面向园区货运、仓储搬运、变电站巡检三类场景，完成工控机与 ROS 2 环境部署、底盘 CAN 联调、激光雷达与 IMU 装调标定、整车系统集成、SLAM 建图、定位与代价地图配置、全局规划与局部避障调试、巡检路线示教及交付验收等工作任务，具备一定的与人沟通、信息收集、自主学习和团队协作等通用能力，以及严谨细致、规范意识、标准意识、质量意识、安全意识和环保意识的职业素养，培育劳动精神、奋斗精神、规则意识、社会主义核心价值观和可持续发展的思政素养。具体包括：

### 2.2 分目标

1. 能详细阅读《无人车交付验收单》《整车接口图》和《安全作业规范》，与教师及客户方围绕作业内容、场地条件和交付要求进行沟通，准确获取车型、接口、限速、交付时间与验收标准等关键信息，按点检表完成整车点检、六类接口标注、作业区布置与急停演练，提交零漏项的接口标注图、现场布置记录与安全交底记录，急停响应不超过 3 s，具备规范意识、安全意识与沟通协作能力。

2. 能依据《工控机部署手册》、Ubuntu 22.04 与 ROS 2 Humble 安装说明及本工作空间源码，按部署流程完成系统检查、依赖安装、`colcon build` 与环境变量配置，做到编译零失败、关键功能包可检索、新终端无需手动 source 即可运行，并能处理至少 2 类依赖或编译错误，具备信息收集、自主学习和严谨细致的职业素养。

3. 能依据雷达驱动说明、`yhs_tutorials` 示例源码与《感知链路点检表》，启动驱动与订阅节点，观察节点图、话题频率与 QoS，绘制 `nvilidar_ros2_node→/scan→lidar_pkg_node` 链路图并记录频率与有效点数，完成示例代码修改与 bag 录制回放，具备数据留痕、质量意识和团队协作能力。

4. 能依据底盘 CAN 通信手册、`yhs_can_interfaces` 接口定义与《里程计标定作业指导书》，配置 `can0`、核对上下行报文、启动驱动并完成速度指令验证与里程计标定，做到 CAN 状态正常、直线 1 m 误差 ≤ 5%、转向 90° 误差 ≤ 5°、0.2 m/s 限速修改生效，具备安全底线、接线规范意识和数据真实性素养。

5. 能依据 nvilidar 与串口 IMU 手册、车型 URDF 和实测安装尺寸，完成设备名与波特率配置、角度与量程调整、静态 TF 标定及多车型模型核验，做到 `/scan` ≥ 5 Hz 且有效点 ≥ 300、`/imu_data` 50~100 Hz、TF 与实测偏差 ≤ 10 mm、模型与实车一致，具备精密装调、防静电规范、交付一致性和复核习惯。

6. 能依据《整机开机 SOP》、`yhs_nav2.launch.py` 与参数文件，按顺序启动感知、底盘、定位与导航节点，核查节点生命周期与参数注入结果，做到 3 min 内完成启动、Nav2 服务器 ≥ 9 个、抽查节点均为 active，并能说明建图与导航入口的 `use_tf` 差异，具备标准作业、时间管理和文档规范意识。

7. 能依据库区平面图、《建图任务书》与 `dgt_mini.lua` 参数说明，规划闭环采集路线，以不超过 0.2 m/s 的速度完成 Cartographer 建图、参数对照与 Gmapping 对比，交付闭合误差 ≤ 0.1 m 的 `.pgm/.yaml` 地图包、参数—现象对照表和选型报告，具备场地保护、质量责任和团队协作素养。

8. 能依据已建地图、AMCL 参数与代价地图配置，完成地图加载、初始定位、粒子云收敛判断及局部 `plugins` 膨胀层修正，做到 3 s 内收敛、定位误差 ≤ 0.1 m、膨胀梯度可见、局部地图随车滚动，并能处置初始位姿错误等定位失效，具备现场一致性保护、审慎处置和质量意识。

9. 能依据三条货运线路目标、NavFn 与 TEB 参数表，完成全局路径生成、通行性验证、限速与窄通道避障调试，做到可达目标 2 s 内出路径、3 个目标点误差 ≤ 0.15 m / 0.15 rad、绕障距离 ≥ 0.25 m、`/cmd_vel` 角速度 ≤ 0.3 rad/s 且 20 Hz 平滑输出，具备线路安全、风险预判、参数记录与规范意识。

10. 能依据行为树 XML、`record_path` 接口与《交付验收单》，完成行为树结构核验与修改、示教路径发布与循迹、随机故障处置及运维资料整理，做到循迹横向误差 ≤ 0.2 m、10 min 内完成故障定位与处置、一次修复率 ≥ 70%，并通过验收与 5 min 答辩，具备应急意识、客户意识、诚信记录、劳动精神和团队协作素养。

---

## 三、教学条件

### 3.1 硬件条件（每 2 名学生 1 套）

| 设备 | 型号 / 规格 | 教学用途 |
|---|---|---|
| 导航底盘 | 煜禾森 DT-mini（或 DT-mid / FR-mid 等） | 运动执行、里程计、CAN 通信 |
| 单线激光雷达 | nvilidar（串口，512000 bps） | `/scan`，建图/定位/避障唯一激光源 |
| 串口 IMU | CH100/CH110（115200 bps） | `/imu_data`，底盘航向校准 |
| 工控机 | Ubuntu 22.04 + ROS 2 Humble | 运行全部节点 |
| CAN 适配器 | SocketCAN（`can0`） | 指令下发与状态反馈 |
| 深度相机（选配） | Orbbec Astra / Gemini2 | `/scan1`、点云实验 |
| 急停开关 | 硬件急停 | 所有上车实验强制使用 |
| 测试场地 | ≥ 4 m × 4 m，含 2~3 处特征墙角、1 条窄通道 | 货运/仓储/巡检场景模拟 |
| 锥桶、挡板、警戒线 | 若干 | 动态障碍、作业区隔离 |
| 卷尺、角度尺、万用表 | — | 里程计、定位精度与供电测量 |

### 3.2 软件条件

| 软件 | 版本 | 用途 |
|---|---|---|
| Ubuntu | 22.04 LTS | 与 ROS 2 Humble 对应 |
| ROS 2 | Humble | 通信、TF2、launch、RViz2 |
| Nav2 | 1.1.5（工作空间源码） | 定位、代价地图、规划、控制、行为树 |
| Cartographer / cartographer_ros | 2.0 | 主建图方案 |
| slam_gmapping | 工作空间源码 | 备选建图对比 |
| teb_local_planner | 0.9.1 | 局部控制器 |
| 调试工具 | `rqt_graph`、`rqt_plot`、`ros2 bag`、`tf2_tools` | 观察、记录、复盘 |

### 3.3 场地与安全条件

1. 场地平整、无积水、光照稳定，四周设警戒线；
2. 每台车配备有效急停，安全员全程持握；
3. 首次上车限速 0.2 m/s、角速度 ≤ 0.3 rad/s；
4. 架空车轮完成指令验证后，方可落地行驶；
5. 电池电压低于厂家阈值立即停止并充电，禁止过放。

### 3.4 不纳入的内容

TurtleBot3 模型与示例地图、Gazebo/Ignition 仿真、`tb3_simulation_launch.py`、阿克曼/四轮转向车型、SMAC 规划器调参、多雷达融合、视觉 SLAM 与深度学习目标检测、多机编队。

---

## 四、课程内容与学时分配

### 4.1 模块总览

| 模块 | 情景化模块名称 | 任务场景 | 学时 | 理论 | 实操 | 对应源码 |
|---|---|---|---|---|---|---|
| M0 | 园区货运无人车交付前的安全交底与整车认知 | 货运 | 8 | 4 | 4 | 全车实物、`yhs_chassis_description` |
| M1 | 仓储搬运无人车工控机部署与工作空间构建 | 仓储 | 12 | 2 | 10 | 全工作空间、`COLCON_BUILD.md` |
| M2 | 仓储 AGV 感知数据链路点检与通信验证 | 仓储 | 8 | 2 | 6 | `yhs_tutorials/*` |
| M3 | 园区货运车底盘 CAN 联调与里程计标定 | 货运 | 16 | 4 | 12 | `yhs_can_interfaces`、`yhs_can_control` |
| M4 | 变电站巡检车激光雷达与 IMU 装调标定 | 巡检 | 12 | 3 | 9 | `nvilidar_ros2`、`serial_imu`、`pointcloud_to_laserscan` |
| M5 | 多车型交付前的模型与 TF 坐标链核验 | 货运/仓储 | 8 | 3 | 5 | `yhs_chassis_description` |
| M6 | 无人车整机开机流程与系统联调 | 仓储 | 8 | 2 | 6 | `yhs_nav2/launch/*` |
| M7 | 仓储库区高精度地图采集与质检 | 仓储 | 20 | 6 | 14 | `cartographer_ros`、`slam_gmapping` |
| M8 | 巡检车地图加载、初始定位与代价地图配置 | 巡检 | 16 | 5 | 11 | `nav2_amcl`、`nav2_map_server`、`nav2_costmap_2d` |
| M9 | 园区货运线路全局规划与通行性验证 | 货运 | 12 | 4 | 8 | `nav2_planner`、`nav2_navfn_planner` |
| M10 | 窄通道会车与动态避障的局部控制调试 | 仓储/货运 | 20 | 6 | 14 | `teb_local_planner`、`nav2_controller`、`nav2_velocity_smoother` |
| M11 | 变电站巡检路线示教、循迹与异常恢复 | 巡检 | 12 | 4 | 8 | `nav2_bt_navigator`、`nav2_behavior_tree`、`RecordPath.action` |
| M12 | 无人车交付验收、故障应急处置与运维交接 | 综合 | 8 | 3 | 5 | 全栈 |
| 合计 | — | — | **160** | **48** | **112** | — |

> 学时分配依据：基础认知类模块 8~12 学时；涉及硬件联调与参数整定的 M3、M4、M8、M10 为 12~20 学时；建图 M7 因场地采集与参数对比耗时最长，安排 20 学时；综合验收 M12 独立设 8 学时，便于组织完整交付流程考核。

### 4.2 20 周教学进度（每周 8 学时）

| 周次 | 模块与学时 | 周任务成果 |
|---|---|---|
| 1 | M0（8） | 安全考核通过、整车接口标注完成 |
| 2 | M1（8） | Ubuntu 与 ROS 2 环境可用 |
| 3 | M1（4）+ M2（4） | 工作空间编译通过；能启动雷达并观察话题 |
| 4 | M2（4）+ M3（4） | 通信链路验收；CAN 配置完成 |
| 5 | M3（8） | `/cmd_vel→/odom` 闭环打通 |
| 6 | M3（4）+ M4（4） | 里程计标定完成；雷达出数 |
| 7 | M4（8） | 雷达/IMU 参数与 TF 验收通过 |
| 8 | M5（8） | 车型模型与 TF 链核验通过 |
| 9 | M6（8） | 全栈一键启动与生命周期验收 |
| 10 | M7（8） | 完成首轮场地地图采集 |
| 11 | M7（8） | 完成参数对比与地图优化 |
| 12 | M7（4）+ M8（4） | 地图归档；地图加载成功 |
| 13 | M8（8） | 初始定位与代价地图调通 |
| 14 | M8（4）+ M9（4） | 定位精度验收；全局路径可视 |
| 15 | M9（8） | 完成 3 条货运线路通行性验证 |
| 16 | M10（8） | TEB 基础跟踪与限速验收 |
| 17 | M10（8） | 窄通道与动态避障调试 |
| 18 | M10（4）+ M11（4） | 速度平滑验收；行为树结构核验 |
| 19 | M11（8） | 巡检路线示教与循迹验收 |
| 20 | M12（8） | 交付验收综合考核 |

---

## 五、模块教学标准

### M0 园区货运无人车交付前的安全交底与整车认知（8 学时）

**任务情景**

某物流园区新到一台低速货运无人车，交付前需由装调班组完成安全交底与整车认知。学生扮演装调技师与安全员，依据交付验收单核对车辆外观、急停、接口与随机附件；在模拟货运通道内完成作业区布置、限速设置与急停演练；最终向“客户方”做一次 5 min 安全交底。

**前置依赖**：无。

**学习目标**

1. 学生能依据交付验收单、整车接口图与实车，按点检表完成 CAN、雷达、IMU、相机、充电口、急停六类接口的识别与标注，做到标注零漏项、位置准确。
2. 学生能依据园区货运作业规范，在模拟通道独立完成警戒线布置、0.2 m/s 限速区设置与 3 次急停演练，做到急停响应时间不超过 3 s、现场布置符合规范。
3. 学生能依据随机安全口令，在 3 s 内完成停机、断电与口头报告，做到 3 次演练全部合格并准确说明违规后果。

**学习内容**

1. 知识点
   - 1.1 低速货运无人车整车组成与六类接口的功能
   - 1.2 上下电顺序、急停回路与限速规定
2. 技能点
   - 2.1 整车点检与接口标注
   - 2.2 作业区布置与急停演练
3. 素养点
   - 3.1 安全第一、令行禁止
   - 3.2 双人协作与记录意识

**硬件操作步骤**

1. 按点检表检查轮胎、线束、急停、电量、传感器外观；
2. 急停按下 → 装电池/合闸 → 工控机开机 → 松开急停（车不动）；
3. 标注接口位置并拍照存档；
4. 收工：拍急停 → 关工控机 → 断电池 → 收纳线缆。

**实操任务**

| 任务 | 要求 | 产出 |
|---|---|---|
| 整车点检 | 逐项核对，记录异常 | 点检表 |
| 接口标注 | 照片标注 6 类接口 | 标注图 |
| 作业区布置 | 警戒线、限速区、急停点 | 现场照片 |
| 急停演练 | 3 次随机口令 | 演练记录 |

**验收标准**

- 点检表无漏项，异常项有处理意见；
- 接口标注 6 类全部正确；
- 急停响应 3 次均 ≤ 3 s；
- 口述上下电顺序与限速规定无误。

**常见问题排查**

| 现象 | 原因 | 处理 |
|---|---|---|
| 上电后蜂鸣 | 急停未复位或电量低 | 复位急停、充电 |
| 工控机不启动 | 电池电压不足 | 万用表测量后充电/换电池 |
| 急停按下仍能动 | 急停回路失效 | 立即断电，报修，禁止继续使用 |

---

### M1 仓储搬运无人车工控机部署与工作空间构建（12 学时）

**任务情景**

仓储客户要求在一台新工控机上部署搬运无人车的 ROS2 运行环境，并在 2 个班次内完成源码编译与自检，确保第二天可进场调试。学生扮演现场部署工程师，依据部署手册完成系统检查、依赖安装、工作空间编译、环境变量固化与交付自检。

**前置依赖**：M0。

**学习目标**

1. 学生能依据部署手册与工作空间源码，独立完成依赖安装与 `colcon build`，做到编译零失败、`install/` 目录完整。
2. 学生能依据环境变量配置要求，将 ROS 与工作空间 `setup.bash` 写入 `~/.bashrc` 并在新终端验证，做到无需手动 source 即可运行 `ros2 pkg prefix yhs_nav2`。
3. 学生能依据交付自检表与错误清单，处理至少 2 类编译/依赖错误，做到自检表全项通过、处理记录可复现。

**学习内容**

1. 知识点
   - 1.1 ROS 2 工作空间 `src/build/install/log` 的职责
   - 1.2 colcon 编译流程、依赖管理与环境变量机制
2. 技能点
   - 2.1 系统与磁盘检查、rosdep 安装、编译与自检
   - 2.2 编译/依赖错误的定位与处理
3. 素养点
   - 3.1 规范部署与版本管理意识
   - 3.2 交付自检与过程留痕习惯

**硬件操作步骤**

1. 工控机接显示器、键盘、电源，不启动底盘；
2. 检查磁盘剩余空间 ≥ 20 GB、内存 ≥ 8 GB；
3. 记录系统版本与网络配置，归档部署信息。

**实操任务**

1. 系统与 ROS 检查：
   ```bash
   lsb_release -a
   source /opt/ros/humble/setup.bash
   ros2 --help
   ```
2. 依赖与编译：
   ```bash
   cd ~/ros2_ws
   rosdep install --from-paths src --ignore-src -r -y
   colcon build --symlink-install
   source install/setup.bash
   ```
3. 固化环境变量，新终端验证：
   ```bash
   ros2 pkg list | grep -E "yhs|nvilidar|serial_imu|cartographer|teb"
   ```
4. 交付自检：启动 RViz，检查显示与插件是否正常。
5. 故障复现：人为制造“未 source”“依赖缺失”两类错误，记录报错并修复。

**源代码 / 配置修改要点**

- 仅修改 `~/.bashrc` 与环境变量，不改业务源码；
- 若工作空间路径不同，后续所有命令中的路径需同步替换；
- 编译参数可参考 [COLCON_BUILD.md](../../src/COLCON_BUILD.md)。

**验收标准**

| 项目 | 指标 |
|---|---|
| 编译 | `colcon build` 无 failed 包 |
| 包检索 | 6 个关键包全部可检索 |
| 环境 | 新终端无需手动 source 即可运行 |
| 自检 | 自检表全项通过，错误处理记录 ≥ 2 条 |

**常见问题排查**

| 现象 | 原因 | 处理 |
|---|---|---|
| `ros2` 命令不存在 | 未 source / 未安装 | source 后重试，检查 `/opt/ros/humble` |
| 缺少 `nav2_*` | 依赖未安装 | `sudo apt install ros-humble-navigation2 ros-humble-nav2-bringup` |
| `libg2o` 缺失 | TEB 依赖缺失 | 按 rosdep 提示安装 |
| `transforms3d` 缺失 | Python 依赖缺失 | `sudo apt install python3-transforms3d` |
| 编译内存不足 | 并行度过高 | `colcon build --parallel-workers 2` |

---

### M2 仓储 AGV 感知数据链路点检与通信验证（8 学时）

**任务情景**

仓储 AGV 进场前，运维人员需对“雷达采集—数据发布—业务订阅”链路做一次点检，确认数据频率、点数与订阅关系正常，并留存基线数据。学生扮演运维技师，使用套件自带示例节点完成链路验证并提交点检报告。

**前置依赖**：M1。

**学习目标**

1. 学生能依据雷达驱动与点检表，启动驱动与订阅节点并观察节点图、频率与 QoS，做到准确绘制 `nvilidar_ros2_node→/scan→lidar_pkg_node` 链路图并记录频率、点数。
2. 学生能依据 `vel_pub`、`pc_pkg`、`nav_pkg` 源码，完成速度值、点数统计与目标点修改，做到修改后节点可运行、现象符合预期并提交代码差异说明。
3. 学生能依据 `ros2 bag` 工具要求，录制并回放 `/odom`、`/cmd_vel`，做到 bag 文件可回放、`info` 信息完整。

**学习内容**

1. 知识点
   - 1.1 节点、话题、QoS、`rqt_graph` 与 bag 的作用
   - 1.2 雷达数据链路与示例节点的输入输出
2. 技能点
   - 2.1 链路启动、频率/点数观测与链路图绘制
   - 2.2 示例代码修改、编译与效果验证
   - 2.3 bag 录制、信息查看与回放
3. 素养点
   - 3.1 数据留痕与基线意识
   - 3.2 严谨记录与团队配合

**硬件操作步骤**

1. 雷达上电，确认 `/dev/lidar` 存在；
2. 整车架空，急停在手（涉及 `cmd_vel` 的环节）；
3. 记录环境温度、场地光照等可能影响雷达的现场条件。

**实操任务**

1. 链路观察（不引入示例包）：
   ```bash
   ros2 launch nvilidar_ros2 nvilidar.launch.py
   ros2 node list
   ros2 topic list
   ros2 topic echo /scan --once --qos-reliability best_effort
   ros2 topic hz /scan --qos-reliability best_effort
   ros2 topic info /scan --verbose
   rqt_graph
   ```
2. 激光订阅节点：
   ```bash
   ros2 run lidar_pkg lidar_pkg_node
   ```
   修改 [lidar_data_node.cpp](../../src/yhs_tutorials/lidar_pkg/src/lidar_data_node.cpp)：统计有效点数与最近距离。
3. 速度发布（架空）：
   ```bash
   ros2 run vel_pub vel_pub_node
   ```
   修改 [vel_pub_node.cpp](../../src/yhs_tutorials/vel_pub/src/vel_pub_node.cpp)：线速度改为 0.05，角速度改为 0。
4. 导航目标示例（整车栈启动后）：
   ```bash
   ros2 run nav_pkg nav_pkg_node
   ```
5. 数据留档：
   ```bash
   ros2 bag record /odom /cmd_vel -o ~/bags/m2
   ros2 bag info ~/bags/m2
   ```
   录制 `/scan` 需加 QoS 覆盖。

**源代码 / 配置修改要点**

- `lidar_data_node.cpp`：回调内累加 `ranges.size()`，用 `std::min_element` 求最近距离；
- `vel_pub_node.cpp`：只改速度数值，保持 100 Hz；
- 修改后执行 `colcon build --symlink-install --packages-select lidar_pkg vel_pub`。

**验收标准**

| 项目 | 指标 |
|---|---|
| 链路图 | 节点、话题、QoS 标注完整 |
| `/scan` | ≥ 5 Hz，有效点 ≥ 300 |
| 代码修改 | 输出点数与最近距离，速度可调 |
| 数据留档 | bag 可回放，info 信息完整 |

**常见问题排查**

| 现象 | 原因 | 处理 |
|---|---|---|
| `ros2 run` 找不到包 | 未 source install | source 后重试 |
| `topic echo` 无输出 | QoS 不匹配 | 加 `--qos-reliability best_effort` |
| `rqt_graph` 空白 | 未安装/未刷新 | 安装 `rqt_graph` 后刷新 |
| 修改代码无效 | 未重新编译 | 指定包重新 build |
| bag 无 `/scan` | 录制 QoS 不匹配 | 使用 QoS 覆盖文件 |

---

### M3 园区货运车底盘 CAN 联调与里程计标定（16 学时）

**任务情景**

园区货运车更换底盘控制器后，需完成 CAN 通信联调、速度指令验证与里程计标定，确保车辆按指令行驶且 `/odom` 可用于后续建图。学生扮演装调技师，在架空与落地两个阶段完成联调，并提交标定数据。

**前置依赖**：M2。

**学习目标**

1. 学生能依据 CAN 手册与 `yhs_can_interfaces` 接口文件，独立完成 `can0` 配置并用 `candump` 核对收发报文，做到 CAN 状态为 UP、准确指出至少 3 个上下行报文 ID 及含义。
2. 学生能依据 `/cmd_vel` 指令与标定方法，完成架空验证、1 m 直线与 90° 转向标定，做到直线误差 ≤ 5%、转向误差 ≤ 5°、标定表完整。
3. 学生能依据限速要求，在 `cmd_callback` 中加入 0.2 m/s 限幅并重新编译，做到下发 0.5 m/s 时实际不超过 0.2 m/s，并能说明代码差异。

**学习内容**

1. 知识点
   - 1.1 CAN 帧格式、上下行报文与 `can0` 配置
   - 1.2 `/cmd_vel` 到轮速的映射、里程计积分与 IMU 朝向
2. 技能点
   - 2.1 CAN 配置与报文观察
   - 2.2 架空/落地速度验证与里程计标定
   - 2.3 限速代码修改与验证
3. 素养点
   - 3.1 限速与急停的安全底线
   - 3.2 接线规范与数据真实性

**硬件操作步骤**

1. 架空驱动轮，场地清空，急停在手；
2. 连接 CAN 适配器，确认 `can0` 识别；
3. 上电，观察底盘状态灯与蜂鸣；
4. 落地前复查轮胎、地面与急停。

**实操任务**

1. CAN 配置与观察：
   ```bash
   ip -details link show can0
   sudo ip link set can0 up type can bitrate 500000
   candump can0
   ```
2. 启动底盘：
   ```bash
   ros2 launch yhs_can_control yhs_can_control.launch.py use_tf:=true
   ros2 node info /yhs_can_control_node
   ros2 topic echo /odom --once
   ros2 topic echo /chassis_info_fb --once
   ```
3. 架空速度验证：
   ```bash
   ros2 topic pub /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.1}, angular: {z: 0.0}}" -r 10
   ```
4. 里程计标定：1 m 直线、原地 90°，各测 3 次取平均。
5. 反馈解析：观察左右轮速、BMS 电压与急停状态。
6. 限速修改：编辑 [cmd_callback](../../src/yhs_can_control/src/yhs_can_control_node.cpp#L153-L161) 加入限幅，重编译验证。

**源代码 / 配置修改要点**

- [yhs_can_control_node.cpp](../../src/yhs_can_control/src/yhs_can_control_node.cpp)：`cmd_callback` 指令映射、`publish_odom` 积分与 IMU 朝向；
- [cfg.yaml](../../src/yhs_can_control/params/cfg.yaml)：`can_name`；
- [yhs_can_control.launch.py](../../src/yhs_can_control/launch/yhs_can_control.launch.py)：`use_tf`；
- 注意 launch 中参数文件以集合字面量传入，`cfg.yaml` 实际不生效，需按架构分析 2.8 修正或依赖源码默认值。

**验收标准**

| 项目 | 指标 |
|---|---|
| CAN | `can0` UP，`candump` 可见反馈帧 |
| 指令响应 | 方向正确，无异响、无过流 |
| 直线精度 | 1 m 误差 ≤ 5% |
| 转向精度 | 90° 误差 ≤ 5° |
| 限速 | 0.5 m/s 指令下实际 ≤ 0.2 m/s |
| 反馈 | `/chassis_info_fb` 各字段可解释 |

**常见问题排查**

| 现象 | 原因 | 处理 |
|---|---|---|
| `can0` 不存在 | 适配器未识别 | `lsusb`、`dmesg`，重新插拔 |
| `can0` 起不来 | 波特率/终端电阻 | 按手册设 500k，检查 120 Ω |
| 有 `/odom` 车不动 | 急停、档位、发送失败 | 查急停与 `candump` 发送帧 |
| 里程计漂移 | 轮径/打滑 | 检查轮胎与地面，重新标定 |
| 车反向 | 相序/符号 | 对照手册查线，禁止随意改代码取反 |

---

### M4 变电站巡检车激光雷达与 IMU 装调标定（12 学时）

**任务情景**

巡检车在变电站内需依靠单线雷达与 IMU 完成建图定位。装调班组需完成雷达与 IMU 的安装检查、串口配置、数据验证与静态 TF 标定，确保在金属设备密集环境中数据稳定。学生扮演装调技师，交付传感器装调记录与数据基线。

**前置依赖**：M3。

**学习目标**

1. 学生能依据 nvilidar、串口 IMU 手册与 `nvilidar.yaml`，完成设备名、波特率、角度与量程配置并启动驱动，做到 `/scan` ≥ 5 Hz 且有效点 ≥ 300、`/imu_data` 50~100 Hz。
2. 学生能依据实测安装尺寸与 launch 文件，修改静态 TF 的 `z` 值与 `frame_id` 并用 `tf2_echo` 验证，做到 `base_link→laser_link/imu_link` 可查且与实测偏差 ≤ 10 mm。
3. 学生能依据 IMU 静止与转动数据，核对方程与角速度符号并检查 `AT+URFR` 映射，做到静止加速度模长 9.8±0.3、转向符号符合右手定则。

**学习内容**

1. 知识点
   - 1.1 单线雷达测距原理、串口参数与扫描参数
   - 1.2 IMU 坐标系、加速度/角速度含义与安装方向
   - 1.3 静态 TF 与传感器安装尺寸的关系
2. 技能点
   - 2.1 设备名、波特率与驱动配置
   - 2.2 雷达角度/量程参数实验
   - 2.3 IMU 数据验证与静态 TF 标定
3. 素养点
   - 3.1 精密装调与防静电规范
   - 3.2 装调数据记录与复核习惯

**硬件操作步骤**

1. 雷达固定安装，镜头清洁，扫描面无遮挡；
2. IMU 按箭头朝前安装，紧固后记录安装高度与偏移；
3. 检查 12 V 供电与串口线序，确认 `/dev/lidar`、`/dev/imu`；
4. 金属设备密集区需记录可能的多路径反射位置。

**实操任务**

1. 雷达验证：
   ```bash
   ls -l /dev/lidar
   ros2 launch nvilidar_ros2 nvilidar.launch.py
   ros2 topic hz /scan --qos-reliability best_effort
   ros2 topic echo /scan --once --qos-reliability best_effort
   ```
2. 参数实验：角度 ±90°、量程 10 m，观察 RViz 扇形变化。
3. IMU 验证：
   ```bash
   ls -l /dev/imu
   ros2 launch serial_imu imu_spec_msg.launch.py
   ros2 topic hz /imu_data
   ros2 topic echo /imu_data --once
   ```
4. TF 验证：
   ```bash
   ros2 run tf2_ros tf2_echo base_link laser_link
   ros2 run tf2_ros tf2_echo base_link imu_link
   ```
5. 选做：启动 Orbbec 相机与 `pointcloud_to_laserscan`，验证 `/scan1`。

**源代码 / 配置修改要点**

- [nvilidar.yaml](../../src/hardware/nvilidar_ros2/params/nvilidar.yaml)：串口、波特率、`frame_id`、角度、量程、滤波；
- [nvilidar.launch.py](../../src/hardware/nvilidar_ros2/launch/nvilidar.launch.py#L30-L34)：静态 TF 的 `z=0.33` 按实测修改；
- [serial_port.cpp](../../src/hardware/serial_imu/src/serial_port.cpp#L17-L18)：`IMU_SERIAL`、`BAUD`；
- [pointcloud_to_laserscan.launch.py](../../src/pointcloud_to_laserscan/launch/pointcloud_to_laserscan.launch.py#L20-L32)：`range_max` 建议由 1.0 改 3.0。

**验收标准**

| 项目 | 指标 |
|---|---|
| `/scan` | ≥ 5 Hz，有效点 ≥ 300，墙面平直 |
| `/imu_data` | 50~100 Hz，静止模长 9.8±0.3 |
| 静态 TF | 与实测安装位置误差 ≤ 10 mm |
| 方向 | 角速度符号符合右手定则 |
| 记录 | 装调记录含安装尺寸、参数、基线数据 |

**常见问题排查**

| 现象 | 原因 | 处理 |
|---|---|---|
| 打不开 `/dev/lidar` | udev 未生效/设备名不同 | `ls -l /dev/ttyUSB*`，补规则 |
| `/scan` 无数据 | 波特率/供电 | 设 512000，检查 12 V |
| 点云缺口 | 遮挡/强光/金属反射 | 清洁、避开直射、调整安装 |
| IMU 无数据 | 设备名/线序/权限 | 查 `/dev/imu`、`dialout`、TX/RX |
| IMU 方向反 | 安装或 `AT+URFR` | 按手册调整，禁止代码取反 |
| TF 冲突 | URDF 与 launch 重复 | 只保留一处发布 |

---

### M5 多车型交付前的模型与 TF 坐标链核验（8 学时）

**任务情景**

同一批交付中包含 DT-mini 与 FR-mid 两种车型，客户要求每台车在 RViz 中的模型、轮距、传感器位置与实车一致，且 TF 链完整。学生扮演交付测试员，按车型选择 URDF、核验模型与 TF，并填写车型配置确认单。

**前置依赖**：M4。

**学习目标**

1. 学生能依据车型清单与 URDF，按车型启动 `robot_state_publisher` 并在 RViz 核对，做到模型与实车一致、无缺网格、配置确认单完整。
2. 学生能依据 TF 树与安装尺寸，使用 `view_frames`、`tf2_echo` 核验 `base_link→laser_link/imu_link`，做到 TF 树完整、与实测偏差 ≤ 10 mm。
3. 学生能依据 TF 断链情境，关闭指定传感器 launch 并观察，做到准确指出“哪一段变换由哪个节点发布”。

**学习内容**

1. 知识点
   - 1.1 URDF 模型组成与网格引用
   - 1.2 TF 树结构与静态变换发布
2. 技能点
   - 2.1 车型选择与 RViz 模型核验
   - 2.2 `view_frames`/`tf2_echo` 核验与断链排查
3. 素养点
   - 3.1 交付一致性与细致核验
   - 3.2 配置单据规范填写

**硬件操作步骤**

1. 整车就位，架空车轮；
2. 按交付车型选择 URDF；
3. 测量雷达/IMU 安装高度与前后偏移并记录。

**实操任务**

1. 启动模型：
   ```bash
   ros2 launch yhs_chassis_description robot.launch.py \
     urdf_model:=$(ros2 pkg prefix yhs_chassis_description)/share/yhs_chassis_description/urdf/DT-mini.urdf
   ```
2. TF 核验：
   ```bash
   ros2 run tf2_tools view_frames
   ros2 run tf2_ros tf2_echo base_link laser_link
   ```
3. RViz 配置：`RobotModel`、`TF`、`LaserScan`，Fixed Frame 设 `base_link`。
4. 断链实验：关闭雷达 launch，观察 `laser_link` 消失。
5. 车型切换：改为 `FR_mid.urdf`，核对轮距与外观。

**源代码 / 配置修改要点**

- [DT-mini.urdf](../../src/yhs_chassis_description/urdf/DT-mini.urdf)：网格路径、轮距、车体尺寸；
- [robot.launch.py](../../src/yhs_chassis_description/launch/robot.launch.py)：`urdf_model`、`use_rviz`；
- `tools/select_car.sh` 可切换车型，但脚本路径为厂家路径，需按本机修改；
- `robot.urdf` 引用的 `mesh/autocube.STL` 不存在，统一使用车型专用 URDF。

**验收标准**

| 项目 | 指标 |
|---|---|
| 模型 | 车型正确、无缺网格 |
| TF 树 | `base_link` 下有 `laser_link`、`imu_link` |
| 尺寸 | 与实测偏差 ≤ 10 mm |
| 配置单 | 车型、URDF、TF 参数完整 |

**常见问题排查**

| 现象 | 原因 | 处理 |
|---|---|---|
| 模型不显示 | Fixed Frame 错误 | 改 `base_link` |
| 网格丢失 | URDF 路径错误 | 换车型 URDF，查 `meshes/` |
| 两个父节点 | 重复发布 TF | 关闭重复静态 TF |
| `odom` 不存在 | 底盘未启动/`use_tf=false` | 启动底盘或改建图模式 |

---

### M6 无人车整机开机流程与系统联调（8 学时）

**任务情景**

仓储客户要求形成标准开机 SOP：从断电状态到导航栈全部就绪不超过 3 min，且各节点状态可核查。学生扮演现场运维，按 SOP 启动整车，记录节点、生命周期、参数注入结果，并编写本岗位的开机检查卡。

**前置依赖**：M3、M4、M5。

**学习目标**

1. 学生能依据 `yhs_nav2.launch.py` 与开机 SOP，启动全栈并记录日志顺序，做到 3 min 内完成启动、节点与生命周期状态正常。
2. 学生能依据 launch 调用树与参数注入机制，标出 5 个启用 Include、2 个注释 Include 及 `RewrittenYaml` 注入项，做到调用树与参数说明准确。
3. 学生能依据建图与导航两种入口，对比节点组成与 `use_tf` 默认值，做到准确说明两种模式互斥原因及切换步骤。

**学习内容**

1. 知识点
   - 1.1 launch/include 与参数注入（`RewrittenYaml`）
   - 1.2 生命周期、`bond` 与节点激活顺序
2. 技能点
   - 2.1 按 SOP 启动全栈与状态核查
   - 2.2 launch 调用树梳理与参数核查
   - 2.3 建图/导航入口对比与切换
3. 素养点
   - 3.1 标准作业与时间意识
   - 3.2 交接文档规范

**硬件操作步骤**

1. 整车就位，场地清空，急停在手；
2. 按“急停—上电—工控机—松急停”顺序开机；
3. 先只观察启动，不让车动；
4. 收工按标准顺序断电。

**实操任务**

1. 阅读 [yhs_nav2.launch.py](../../src/yhs_nav2/launch/yhs_nav2.launch.py#L120-L175)，标注 Include 与参数。
2. 启动全栈：
   ```bash
   ros2 launch yhs_nav2 yhs_nav2.launch.py
   ```
3. 状态核查：
   ```bash
   ros2 node list
   ros2 lifecycle get /map_server
   ros2 lifecycle get /amcl
   ros2 lifecycle get /bt_navigator
   ```
4. 参数核查：
   ```bash
   ros2 param get /map_server yaml_filename
   ros2 param get /amcl scan_topic
   ros2 param get /controller_server FollowPath.plugin
   ```
5. 对比建图入口 [cartographer.launch.py](../../src/yhs_nav2/launch/cartographer.launch.py#L76-L111)。

**源代码 / 配置修改要点**

- `yhs_nav2.launch.py`：`map`、`use_tf`、`use_composition`、`autostart`；
- 选做：打开相机与 `pointcloud_to_laserscan` 注释；
- 组合模式当前未真正生效（启动容器但子 launch 用独立 `Node`），可作为拓展讨论，见架构分析 8.1。

**验收标准**

| 项目 | 指标 |
|---|---|
| 启动时间 | ≤ 3 min |
| 节点 | Nav2 服务器 ≥ 9 个 |
| 生命周期 | 抽查 3 个节点均为 `active` |
| 参数 | 地图路径、扫描话题、控制器插件正确 |
| 文档 | 开机检查卡 1 份 |

**常见问题排查**

| 现象 | 原因 | 处理 |
|---|---|---|
| 启动中断 | 地图/参数路径错误 | 用绝对路径 |
| 节点 `unconfigured` | lifecycle 未启动 | 检查 `autostart` |
| 节点名冲突 | 旧进程残留 | `pkill -f nav2`、`pkill -f yhs` |
| 启动超过 3 min | 磁盘/CPU 慢、地图过大 | 清理日志，检查资源 |

---

### M7 仓储库区高精度地图采集与质检（20 学时）

**任务情景**

仓储客户要求为 30 m × 20 m 库区建立导航地图，货架通道狭窄、存在重复结构。学生扮演建图工程师，制定采集路线、完成 Cartographer 建图、做参数对比与地图质检，最终交付地图包与质检报告；并用 Gmapping 做一次对比，说明选型理由。

**前置依赖**：M6。

**学习目标**

1. 学生能依据库区平面图与建图任务书，规划闭环路线并低速采集、保存，做到交付闭合误差 ≤ 0.1 m 的 `.pgm/.yaml` 地图包。
2. 学生能依据单变量法完成至少 4 组关键参数对照实验，做到参数—现象对照表完整，并准确说明子图帧数、优化频率、量程、IMU 开关的影响。
3. 学生能依据同一路线对比 Gmapping 与 Cartographer，做到对比报告含闭合度、重影、耗时数据与选型建议。

**学习内容**

1. 知识点
   - 1.1 SLAM 原理、子图、回环与占据栅格
   - 1.2 Cartographer 关键参数与 Gmapping 差异
2. 技能点
   - 2.1 采集路线规划与低速建图
   - 2.2 地图保存、质检与参数对照
   - 2.3 Cartographer/Gmapping 对比与选型
3. 素养点
   - 3.1 场地保护与团队配合
   - 3.2 地图质量责任意识

**硬件操作步骤**

1. 清空库区人员，货架位置保持与交付一致；
2. 检查雷达高度与货架遮挡关系；
3. 一人遥控、一人观察 RViz、一人持急停；
4. 采集全程限速 0.2 m/s、角速度 ≤ 0.3 rad/s。

**实操任务**

1. 启动建图：
   ```bash
   ros2 launch yhs_nav2 cartographer.launch.py
   ```
2. 按“沿主通道一圈 → 支路往返 → 回到起点闭合”的路线采集。
3. 保存：
   ```bash
   mkdir -p ~/maps
   ros2 run nav2_map_server map_saver_cli -f ~/maps/warehouse
   ```
4. 参数实验：`num_range_data` 40/60/90；`optimize_every_n_nodes` 5/10；`min/max_range`；`use_imu_data`。
5. Gmapping 对比：
   ```bash
   ros2 launch yhs_nav2 gmapping.launch.py
   ```
6. 地图质检：闭合点实测、墙体直线度、通道宽度、重影。

**源代码 / 配置修改要点**

- [dgt_mini.lua](../../src/cartographer_ros/cartographer_ros/configuration_files/dgt_mini.lua)：坐标系、`provide_odom_frame`、`use_odometry`、`use_imu_data`、子图与优化频率、运动滤波；
- [dgt_mini.launch.py](../../src/cartographer_ros/cartographer_ros/launch/dgt_mini.launch.py)：分辨率 0.05；`cartographer_occupancy_grid_node` 的 `use_sim_time` 硬编码为 `True`，实车异常时改为参数化；
- [cartographer.launch.py](../../src/yhs_nav2/launch/cartographer.launch.py#L39)：`use_tf` 默认 `false`。

**验收标准**

| 项目 | 指标 |
|---|---|
| 完整性 | 库区边界闭合，无明显缺墙 |
| 精度 | 闭合误差 ≤ 0.1 m |
| 地图文件 | `.pgm/.yaml` 成对，分辨率 0.05 |
| 参数实验 | ≥ 4 组，有对照表 |
| 对比报告 | 含数据与选型结论 |

**常见问题排查**

| 现象 | 原因 | 处理 |
|---|---|---|
| 重影 | 车速快、转向急 | 降速，分段转向 |
| 长通道漂移 | 特征少、回环少 | 增加回环，开启 IMU |
| 地图跳变 | 回环误匹配/IMU 方向错 | 查 IMU，降低优化频率 |
| `/map` 不更新 | 占用栅格节点未起 | 检查 launch |
| 保存超时 | `/map` 未发布 | 先 `ros2 topic echo /map --once` |

---

### M8 巡检车地图加载、初始定位与代价地图配置（16 学时）

**任务情景**

巡检车进入变电站后，需加载既有地图、完成初始定位，并按现场设备密度调整代价地图，确保定位稳定、障碍表达完整。学生扮演运维技师，完成地图加载、AMCL 定位、代价地图图层配置与定位失效处置。

**前置依赖**：M7。

**学习目标**

1. 学生能依据已建地图与初始位姿，完成地图加载与 AMCL 定位，做到 3 s 内粒子云收敛、定位误差 ≤ 0.1 m。
2. 学生能依据全局/局部代价地图配置，修正局部 `plugins` 加入膨胀层并调整 footprint 与膨胀半径，做到 RViz 中膨胀梯度可见、局部地图随车滚动。
3. 学生能依据定位失效情境，通过话题、TF 与粒子云逐层判断，做到形成完整处置记录并恢复定位。

**学习内容**

1. 知识点
   - 1.1 AMCL 原理、粒子与 `map→odom`
   - 1.2 代价地图图层、footprint 与膨胀
2. 技能点
   - 2.1 地图加载与初始定位
   - 2.2 定位精度验证与失效处置
   - 2.3 代价地图配置与膨胀修正
3. 素养点
   - 3.1 现场一致性保护
   - 3.2 定位异常的审慎处置

**硬件操作步骤**

1. 场地保持与建图一致，不得移动设备；
2. 整车就位，标记起点；
3. 急停在手，先静态定位后动态验证。

**实操任务**

1. 加载地图并启动：
   ```bash
   ros2 launch yhs_nav2 yhs_nav2.launch.py map:=$HOME/maps/warehouse.yaml
   ```
2. 设置初始位姿：
   ```bash
   ros2 run yhs_nav2 set_init_amcl_pose.py --ros-args -p x:=0.0 -p y:=0.0 -p theta:=0.0
   ```
3. 定位验证：
   ```bash
   ros2 topic echo /amcl_pose --once
   ros2 run tf2_ros tf2_echo map base_link
   ```
4. 代价地图观察与参数实验：`max_beams`、粒子数、`inflation_radius`、footprint。
5. 修正局部膨胀层：把 `plugins` 改为 `["static_layer", "obstacle_layer", "inflation_layer"]`。
6. 故障注入：故意设错初始位姿，记录粒子云与 TF 表现，再恢复。

**源代码 / 配置修改要点**

- [yhs_nav2.yaml](../../src/yhs_nav2/param/yhs_nav2.yaml#L1-L53)：AMCL 模型、`max_beams`、粒子数、更新阈值、`set_initial_pose`；
- [local_costmap](../../src/yhs_nav2/param/yhs_nav2.yaml#L237-L282) 与 [global_costmap](../../src/yhs_nav2/param/yhs_nav2.yaml#L283-L334)：插件、footprint、膨胀、观测源；
- 必做修正：局部 `plugins` 增加 `inflation_layer`（架构分析 8.5）；
- `set_initial_pose: True` 且原点初始化，要求建图/导航起点一致。

**验收标准**

| 项目 | 指标 |
|---|---|
| 收敛 | ≤ 3 s 粒子云收敛 |
| 精度 | 静止误差 ≤ 0.1 m |
| 代价地图 | 全局与静态图一致，局部随车滚动 |
| 膨胀 | 修改半径后梯度明显变化 |
| 处置 | 能恢复错误初始位姿 |

**常见问题排查**

| 现象 | 原因 | 处理 |
|---|---|---|
| 不收敛 | 位姿错/地图不匹配 | 重设位姿，核对地图 |
| 定位跳变 | 动态障碍/雷达偏移 | 清场，检查安装 |
| 局部无膨胀 | `plugins` 未列膨胀层 | 按 8.5 修正 |
| `map` 不存在 | map_server 未激活 | 查 lifecycle 与路径 |
| yaml 打不开 | `.pgm/.yaml` 不匹配 | 成对修正 |

---

### M9 园区货运线路全局规划与通行性验证（12 学时）

**任务情景**

园区有三条货运线路（主环线、装卸区支线、充电桩回程线），需验证无人车能否规划出安全可行路径。学生扮演测试员，逐条线路下发目标、检查 `/plan`、调整 NavFn 参数，提交线路通行性报告。

**前置依赖**：M8。

**学习目标**

1. 学生能依据三条线路目标点与全局代价地图，逐条下发目标并观察 `/plan`，做到可达目标 2 s 内出路径、报告含截图与通行结论。
2. 学生能依据 NavFn 参数完成 `tolerance`、`use_astar`、`allow_unknown` 对照，做到准确说明参数对路径长度、绕行与成功率的影响。
3. 学生能依据不可达/障碍目标情境复现失败并读取日志与返回码，做到准确判断问题属于地图、代价地图、目标点还是规划器。

**学习内容**

1. 知识点
   - 1.1 全局规划与 NavFn 原理
   - 1.2 代价地图、坐标系与目标约束
2. 技能点
   - 2.1 目标下发与 `/plan` 观察
   - 2.2 NavFn 参数对照实验
   - 2.3 规划失败复现与归因
3. 素养点
   - 3.1 线路安全与通行规则
   - 3.2 测试记录与结论规范

**硬件操作步骤**

1. 场地清空，整车就位；
2. 先只看路径，不启动控制；
3. 目标点位置用地面标记固定。

**实操任务**

1. 接口观察：
   ```bash
   ros2 node info /planner_server
   ros2 action list | grep compute_path
   ```
2. 发送目标（RViz `2D Goal Pose` 或 `nav_pkg`）：
   ```bash
   ros2 run nav_pkg nav_pkg_node
   ```
3. 路径观察：
   ```bash
   ros2 topic echo /plan --once
   ```
4. 参数对照：`tolerance` 0.1/0.5/1.0；`use_astar` true/false；`allow_unknown` false/true。
5. 失败复现：目标设在障碍内、地图外，记录日志。
6. 三条线路逐条验证并填表。

**源代码 / 配置修改要点**

- [yhs_nav2.yaml](../../src/yhs_nav2/param/yhs_nav2.yaml#L348-L357)：`GridBased` 插件与参数；
- 注释区的 SMAC 配置不使用；
- `planner_server` 需等全局代价地图更新完成。

**验收标准**

| 项目 | 指标 |
|---|---|
| 路径生成 | 可达目标 2 s 内出路径 |
| 安全性 | 不穿越已知障碍 |
| 三条线路 | 全部完成验证并记录 |
| 参数解释 | 能说明三组参数影响 |
| 失败判断 | 能定位失败环节 |

**常见问题排查**

| 现象 | 原因 | 处理 |
|---|---|---|
| 一直 planning | 全局代价地图未就绪 | 等待/查 costmap |
| 路径穿墙 | 地图与现场不符 | 重建地图，查 `/scan` |
| 目标被拒 | 目标在障碍/边界外 | 重设目标 |
| 无 `/plan` | 未发目标/工具错误 | 检查 action 客户端 |

---

### M10 窄通道会车与动态避障的局部控制调试（20 学时）

**任务情景**

仓储通道宽度仅比车体宽 0.6 m，且存在人员/叉车动态通行。测试组需整定 TEB 与速度平滑参数，使无人车在窄通道内平稳通过、遇动态障碍能减速绕行或停车等待。学生扮演调试工程师，按“先限速、再障碍、最后权重”的顺序完成整定并提交参数记录。

**前置依赖**：M9。

**学习目标**

1. 学生能依据 TEB 配置与窄通道任务，完成限速与跟踪实验，做到 `/cmd_vel` 线速度 ≤ 设定值、角速度 ≤ 0.3 rad/s、直道无蛇形。
2. 学生能依据锥桶/移动障碍与障碍参数，调整 `min_obstacle_dist`、`inflation_dist`、footprint，做到绕障距离 ≥ 0.25 m、无碰撞。
3. 学生能依据震荡、贴障、打转三类异常现象，复现并对照参数归因，做到完成 ≥ 4 组对照表并给出推荐参数。
4. 学生能依据 `velocity_smoother` 默认参数与底盘能力，补齐 `max_velocity/max_accel` 等配置，做到速度无突变、20 Hz 平滑输出。

**学习内容**

1. 知识点
   - 1.1 TEB 原理、footprint 与障碍约束
   - 1.2 `cmd_vel_nav→cmd_vel` 链路与速度平滑
2. 技能点
   - 2.1 限速与跟踪调试
   - 2.2 窄通道与动态避障测试
   - 2.3 参数对照与推荐值确定
   - 2.4 smoother 参数配置与验证
3. 素养点
   - 3.1 安全距离与风险预判
   - 3.2 动态场景下的协作警戒

**硬件操作步骤**

1. 用锥桶搭出 0.6 m 余量窄通道；
2. 动态障碍由学生持挡板沿通道移动；
3. 急停在手，首次限速 0.2 m/s；
4. 每组参数测试前清场。

**实操任务**

1. 启动导航栈并低速跟踪。
2. 限速实验：`max_vel_x` 0.2/0.3/0.5，记录到达时间与超调。
3. 障碍实验：`min_obstacle_dist` 0.3/0.45/0.6。
4. 震荡复现：增大 `max_vel_theta`、降低 `weight_kinematics_nh`，再恢复。
5. 平滑验证：
   ```bash
   ros2 topic hz /cmd_vel
   ros2 param get /velocity_smoother max_velocity
   ```
6. 填写参数对照表：速度、到达时间、最小障碍距离、是否震荡、恢复次数。

**源代码 / 配置修改要点**

- [yhs_nav2.yaml](../../src/yhs_nav2/param/yhs_nav2.yaml#L129-L231)：TEB 速度、加速度、footprint、障碍距离、权重；
- `velocity_smoother` 当前无参数段，按架构分析 8.4 补齐；
- 控制链路 remap：[navigation_launch.py](../../src/yhs_nav2/launch/navigation_launch.py#L93) 与 [同文件](../../src/yhs_nav2/launch/navigation_launch.py#L153-L154)。

**验收标准**

| 项目 | 指标 |
|---|---|
| 平稳性 | 直道无蛇形，无高频抖动 |
| 限速 | 线速度 ≤ 设定值，角速度 ≤ 0.3 rad/s |
| 避障 | 与障碍距离 ≥ 0.25 m，无碰撞 |
| 平滑 | `/cmd_vel` 约 20 Hz，无突变 |
| 记录 | ≥ 4 组对照，推荐参数明确 |

**常见问题排查**

| 现象 | 原因 | 处理 |
|---|---|---|
| 有指令不动 | smoother/底盘未激活 | 查 lifecycle 与底盘 |
| 原地打转 | 目标朝向误差大 | 调整目标与容差 |
| 蛇形 | 角速度/角加速度过大 | 降 `max_vel_theta`、`acc_lim_theta` |
| 贴障太近 | 障碍距离小/无膨胀 | 增大距离，修局部膨胀 |
| 频繁恢复 | 进度超时/通道被堵 | 放宽时间，清障 |
| 速度突变 | smoother 默认加速度大 | 配置 `max_accel/max_decel` |

---

### M11 变电站巡检路线示教、循迹与异常恢复（12 学时）

**任务情景**

巡检任务要求无人车每天沿固定路线巡视设备区。现场采用“人工示教一次、之后自动循迹”的作业方式。学生扮演运维技师，完成行为树结构核验、示教路径发布、循迹调试与异常恢复，并交付巡检路线配置说明。

**前置依赖**：M10。

**学习目标**

1. 学生能依据默认行为树 XML 与 `/behavior_tree_log`，梳理“规划—跟随—恢复”结构，做到主干与恢复分支绘制正确并能指出失败节点。
2. 学生能依据 `record_path` 接口与示教路线，编写最小 Path 发布者并下发 `navigate_follow_record_path`，做到循迹横向误差 ≤ 0.2 m。
3. 学生能依据雷达断电、路径被堵等故障情境，注入故障并读取日志，做到触发并解释 Spin/Wait/BackUp 恢复并形成处置记录。

**学习内容**

1. 知识点
   - 1.1 行为树结构、`RecoveryNode` 与 `RateController`
   - 1.2 `record_path` 定制 navigator 与 RecordPath 动作
2. 技能点
   - 2.1 BT 阅读、修改与加载
   - 2.2 示教路径发布与循迹调试
   - 2.3 故障注入与恢复行为验证
3. 素养点
   - 3.1 巡检路线规范与应急意识
   - 3.2 日志记录与复盘习惯

**硬件操作步骤**

1. 场地布置巡检点与设备区边界；
2. 人工示教时低速行驶，路径连续无急弯；
3. 急停在手，动态障碍由专人控制。

**实操任务**

1. 观察行为树：
   ```bash
   ros2 topic echo /behavior_tree_log
   ```
   对照 [navigate_to_pose_w_replanning_and_recovery.xml](../../src/navigation2/nav2_bt_navigator/behavior_trees/navigate_to_pose_w_replanning_and_recovery.xml)。
2. 修改 BT：复制 XML，把重规划频率改为 0.5 Hz，用绝对路径参数加载（Humble 键名为 `default_nav_to_pose_bt_xml`）。
3. 恢复动作验证：
   ```bash
   ros2 action send_goal /spin nav2_msgs/action/Spin "{target_yaw: 1.57}"
   ```
4. 记录路径：
   - 阅读 [navigate_follow_record_path.cpp](../../src/navigation2/nav2_bt_navigator/src/navigators/navigate_follow_record_path.cpp#L48-L53)；
   - 编写 Path 发布者，向 `/record_path` 发布 3~5 个 `PoseStamped`；
   - 下发 `navigate_follow_record_path` 动作，观察循迹；
   - 对照 [记录路径 BT](../../src/navigation2/nav2_bt_navigator/behavior_trees/navigate_follow_record_path_w_replanning_and_recovery.xml)：只做 `FollowPath`，无全局规划。
5. 故障注入：拔雷达、堵路径、错误目标，记录日志与恢复次数。

**源代码 / 配置修改要点**

- BT XML：`RecoveryNode` 重试、`RateController` 频率、恢复顺序；
- [yhs_nav2.yaml](../../src/yhs_nav2/param/yhs_nav2.yaml#L55-L99)：`plugin_lib_names` 必须含 `nav2_compute_follow_record_path_action_bt_node`；
- 自定义 BT 必须用绝对路径；
- 记录路径必须连续可跟踪。

**验收标准**

| 项目 | 指标 |
|---|---|
| BT 结构 | 主干与恢复分支绘制正确 |
| BT 修改 | 新频率在日志中可见且导航可完成 |
| 恢复动作 | Spin 误差 ≤ 0.1 rad，BackUp 误差 ≤ 0.05 m |
| 循迹 | 横向误差 ≤ 0.2 m |
| 故障处置 | 日志可定位失败节点 |

**常见问题排查**

| 现象 | 原因 | 处理 |
|---|---|---|
| BT 不启动 | 参数名/路径错 | 用 `default_nav_to_pose_bt_xml` 绝对路径 |
| 一直恢复 | 路径堵/定位丢失 | 清障，查 TF |
| 记录路径不动 | 未发 goal/Path 空 | 查 `/record_path` 与 Navigator |
| 插件未加载 | `plugin_lib_names` 缺失 | 加入定制 BT 库 |

---

### M12 无人车交付验收、故障应急处置与运维交接（8 学时）

**任务情景**

客户组织交付验收：无人车需在模拟园区内完成“建图—重定位—多点货运—动态避障—巡检路线循迹—安全停车”全流程，并提交运维手册与备件清单。学生分组扮演交付方与客户方，按验收单逐项签字，客户方随机注入 1 个故障，交付方限时处置。

**前置依赖**：M0~M11 全部通过。

**学习目标**

1. 学生能依据验收单、场地与完整整车，按流程完成全功能演示，做到验收单全项通过、记录表完整。
2. 学生能依据客户随机注入的故障，按五层排查法定位并处置，做到 10 min 内给出原因与处理、一次修复率 ≥ 70%。
3. 学生能依据运维交接要求，整理开机卡、参数表、备件清单与故障手册，做到交接资料齐全并能完成 5 min 答辩。

**学习内容**

1. 知识点
   - 1.1 交付验收流程与评分标准
   - 1.2 五层排查法与运维交接内容
2. 技能点
   - 2.1 全流程功能演示
   - 2.2 随机故障处置
   - 2.3 运维资料整理与答辩
3. 素养点
   - 3.1 客户意识与诚信记录
   - 3.2 安全底线与团队协作

**硬件操作步骤**

1. 验收前完成整车点检与电量检查；
2. 按 M7 建图并保存 `~/maps/final_map`；
3. 重启系统，加载地图并完成初始定位；
4. 依次执行 3 个货运点、1 段动态避障、1 条巡检路线；
5. 结束时停在指定区域，速度归零。

**实操任务**

| 阶段 | 任务 | 交付物 |
|---|---|---|
| 建图 | 完成场地建图并保存 | 地图包 + 质检记录 |
| 重定位 | 加载地图、设初始位姿 | 定位截图 |
| 货运 | 3 点导航，误差 ≤ 0.15 m | 验收单 |
| 避障 | 绕行锥桶，距离 ≥ 0.25 m | 测试记录 |
| 巡检 | 记录路径循迹，横向误差 ≤ 0.2 m | 循迹截图 |
| 排故 | 限时处置随机故障 | 排故记录 |
| 交接 | 运维资料与答辩 | 交接资料 |

**验收与评分**

| 维度 | 分值 | 评分点 |
|---|---|---|
| 建图质量 | 15 | 闭合误差、墙体完整性 |
| 定位 | 15 | 收敛速度、全程稳定 |
| 导航 | 20 | 3 点到达与精度 |
| 避障 | 15 | 距离与平稳性 |
| 记录路径 | 10 | 循迹精度 |
| 安全规范 | 10 | 急停、限速、清场、双人作业 |
| 排故与交接 | 15 | 处置速度、记录完整性、答辩 |

总分 ≥ 60 合格，≥ 80 良好，≥ 90 优秀；出现碰撞、超速、未拍急停等安全项直接不合格。

**常见问题排查**

| 现象 | 原因 | 处理 |
|---|---|---|
| 验收中途停车 | 定位丢失/路径堵 | 查 TF 与代价地图，重置目标 |
| 循迹偏差大 | 示教路径急弯/定位漂移 | 重新示教，检查 AMCL |
| 排故超时 | 未按五层法 | 先电源/设备，再话题/TF/参数 |

---

## 六、课程实施建议

### 6.1 教学组织

- 2 人 1 台，分 A/B 岗：A 岗操作手，B 岗安全员兼记录员，每 20 min 轮换；
- 每个模块按“任务书下发 → 方案确认 → 实操 → 自检 → 教师验收 → 复盘”六步组织；
- 教师台保留 1 套备用雷达、IMU、CAN 线，用于快速排故。

### 6.2 教学方法

1. **任务驱动**：以交付单、验收单、排故单为载体，学生带着交付目标学习；
2. **故障注入**：教师按模块预设 1~2 个故障，训练排查思路；
3. **对照实验**：每次只改一个参数，记录现象，形成参数—效果对应关系；
4. **角色扮演**：设置客户方、交付方、安全员，训练沟通与规范意识；
5. **复盘会**：每组 5 min 汇报“做了什么、数据如何、问题在哪”。

### 6.3 考核评价

| 项目 | 比例 | 说明 |
|---|---|---|
| 过程性考核 | 30% | 出勤、安全规范、6S、每模块验收记录 |
| 模块实操 | 40% | M1~M11 共 11 次，取平均 |
| 综合验收 | 30% | M12 任务书评分 |

### 6.4 分层要求

| 层次 | 要求 |
|---|---|
| 基础（全体） | 完成 M0~M8，能部署、建图、定位、导航 |
| 进阶（多数） | 完成 M9~M11 参数对照与自定义行为树 |
| 拓展（竞赛） | 记录路径优化、动态避障策略、故障注入库建设 |

### 6.5 教材与资源

- 主教材：本课程校本讲义（按模块任务书组织）；
- 参考：本工作空间源码与《ROS2 小车系统架构分析》；
- 资源：模块任务书、验收单、故障库、参数记录表、演示视频。

---

## 七、附录

### 附录 A 常用命令速查

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

### 附录 B 模块—源码对照表

| 模块 | 主要源码 / 配置 |
|---|---|
| M1 | 全工作空间、`COLCON_BUILD.md` |
| M2 | `yhs_tutorials/lidar_pkg`、`pc_pkg`、`vel_pub`、`nav_pkg` |
| M3 | `yhs_can_interfaces/*.msg`、`yhs_can_control/src/yhs_can_control_node.cpp`、`params/cfg.yaml` |
| M4 | `hardware/nvilidar_ros2`、`hardware/serial_imu`、`pointcloud_to_laserscan` |
| M5 | `yhs_chassis_description/urdf/*.urdf`、`launch/robot.launch.py`、`meshes/*.STL` |
| M6 | `yhs_nav2/launch/yhs_nav2.launch.py`、`localization_launch.py`、`navigation_launch.py`、`cartographer.launch.py`、`gmapping.launch.py` |
| M7 | `cartographer_ros/launch/dgt_mini.launch.py`、`configuration_files/dgt_mini.lua`、`slam_gmapping` |
| M8 | `navigation2/nav2_amcl`、`nav2_map_server`、`nav2_costmap_2d`、`yhs_nav2/param/yhs_nav2.yaml` |
| M9 | `navigation2/nav2_planner`、`nav2_navfn_planner` |
| M10 | `teb_local_planner`、`dependence/costmap_converter`、`nav2_controller`、`nav2_velocity_smoother` |
| M11 | `navigation2/nav2_bt_navigator`、`nav2_behavior_tree`、`yhs_can_interfaces/action/RecordPath.action` |
| M12 | 全栈 |

### 附录 C 关键参数速查

| 参数 | 所在文件 | 教学建议值 | 作用 |
|---|---|---|---|
| `max_vel_x` | `yhs_nav2.yaml`（TEB） | 0.2 → 0.5 | 线速度上限 |
| `max_vel_theta` | 同上 | 0.3 | 角速度上限 |
| `acc_lim_x` / `acc_lim_theta` | 同上 | 0.3 | 加速度上限 |
| `min_obstacle_dist` | 同上 | 0.45 | 障碍安全距离 |
| `inflation_dist` | 同上 | 0.6 | 膨胀距离 |
| `xy_goal_tolerance` | 同上 | 0.1 | 到点位置容差 |
| `max_beams` | `yhs_nav2.yaml`（AMCL） | 48 | 更新光束数 |
| `min_particles` / `max_particles` | 同上 | 500 / 5000 | 粒子数范围 |
| `inflation_radius` | 全局/局部代价地图 | 0.55 / 0.6 | 膨胀半径 |
| `submaps.num_range_data` | `dgt_mini.lua` | 60 | 每子图帧数 |
| `optimize_every_n_nodes` | 同上 | 5 | 位姿图优化频率 |
| `provide_odom_frame` | 同上 | true（建图） | 是否发布 odom |
| `use_tf` | `yhs_can_control` | 建图 false / 导航 true | 是否发布 `odom→base_link` |
| `smoothing_frequency` | `velocity_smoother` | 20 Hz | 平滑频率 |

### 附录 D 模块验收记录表（模板）

| 项目 | 内容 |
|---|---|
| 模块 / 日期 / 组号 | |
| 任务情景 | |
| 参与学生与角色 | |
| 硬件状态 | 电池电压 ____ V；急停 正常/异常 |
| 关键操作与命令 | |
| 观测数据 | `/scan` ____ Hz；`/imu_data` ____ Hz；`/cmd_vel` ____ Hz；定位误差 ____ m |
| 交付物 | |
| 问题与处理 | 现象 → 原因 → 处理 → 结果 |
| 教师签字 | |

### 附录 E 术语表

| 术语 | 含义 |
|---|---|
| SLAM | 同步定位与建图 |
| AMCL | 自适应蒙特卡洛定位 |
| TF | 坐标变换树 |
| TEB | 时间弹性带局部规划器 |
| 生命周期节点 | 具备 configure/activate 等状态管理的节点 |
| 代价地图 | 用代价值表达障碍与通行成本的栅格地图 |
| 示教循迹 | 人工记录路径后自动跟随 |
