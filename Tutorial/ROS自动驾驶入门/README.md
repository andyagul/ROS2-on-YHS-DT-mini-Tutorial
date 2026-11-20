# 课程一：ROS 2 自动驾驶入门

> 面向技师学院智能网联汽车技术应用专业（中级工 / 高级工）的专业核心课配套资源。
> 教学载体：深圳煜禾森 OS-lite ROS2 单线激光 SLAM 导航套件（DT-mini 等车型）。
> 配套资源：[课程标准](ROS2自动驾驶入门课程标准.md) ｜ [系统架构分析](ROS2小车系统架构分析.md) ｜ [Code Wiki 源码百科](CodeWiki/README.md) ｜ [仓库总览](../../README.md)

---

## 一、课程概况

| 项目 | 内容 |
| --- | --- |
| 课程性质 | 专业核心课（必修） |
| 授课对象 | 智能网联汽车技术应用专业中级工 / 高级工 |
| 建议学时 | 160 学时（理论 48 + 实操 112），按 20 周排课 |
| 先修要求 | 无强制先修；建议具备 Linux 基础操作与 Python 编程入门 |
| 教学载体 | YHS DT-mini 差速底盘 + 单线激光 + IMU（选配深度相机） |
| 软件环境 | Ubuntu 22.04 + ROS 2 Humble + Nav2 1.1.5 + Cartographer + TEB |
| 后续课程 | [课程二：ROS 自动驾驶二次开发](../ROS自动驾驶二次开发/README.md) |

## 二、教学目标

完成本课程后，学生应能：

1. 独立在工控机上完成 ROS 2 工作空间部署、依赖安装与全量编译。
2. 说明整车四层软件架构（底盘 / 传感器 / 算法 / 应用）与关键话题链路。
3. 使用单线激光与 IMU 完成场地建图，并判断地图质量（重影、闭合误差）。
4. 完成 AMCL 定位、代价地图配置、全局与局部路径规划调试。
5. 处理底盘 CAN、雷达串口、IMU 串口等常见硬件与通信故障。
6. 按任务书完成实车导航验收，并规范记录实验数据与复盘结论。

## 三、模块结构（M0–M12）

| 阶段 | 模块 | 主题 | 对应源码 |
| --- | --- | --- | --- |
| 认知与部署 | M0 | 整车认知与安全规范 | `src/yhs_chassis_description/` |
| 认知与部署 | M1 | 工控机部署与工作空间构建 | [COLCON_BUILD.md](../../src/COLCON_BUILD.md) |
| 感知 | M2 | 底盘、雷达、IMU 驱动与话题 | `src/yhs_can_control/`、`src/hardware/` |
| 感知 | M3 | CAN 协议与底盘消息 | `src/yhs_can_interfaces/` |
| 感知 | M4 | 传感器标定与数据质量 | `src/hardware/nvilidar_ros2/` |
| 定位建图 | M5 | 坐标变换与车型模型 | `src/yhs_chassis_description/` |
| 定位建图 | M6 | launch 编排与启动流程 | `src/yhs_nav2/launch/` |
| 定位建图 | M7 | Cartographer 建图与地图保存 | `src/cartographer_ros/` |
| 导航 | M8 | Nav2 架构与代价地图 | `src/navigation2/` |
| 导航 | M9 | AMCL 定位与重定位 | `src/navigation2/nav2_amcl/` |
| 导航 | M10 | 全局 / 局部规划与 TEB 调参 | `src/teb_local_planner/` |
| 导航 | M11 | 行为树与恢复行为 | `src/navigation2/nav2_bt_navigator/` |
| 综合 | M12 | 综合任务与考核 | 全工作空间 |

各模块的理论学时、实操学时与验收指标以[课程标准](ROS2自动驾驶入门课程标准.md)为准。

**模块与 Code Wiki 章节对照**

| 模块 | 建议配套阅读 |
| --- | --- |
| M0–M1 整车认知与部署 | [01 整体架构](CodeWiki/01-architecture.md)、[08 机器人模型](CodeWiki/08-robot-model.md)、[09 编译运行](CodeWiki/09-build-run-debug.md)、[16 部署运维](CodeWiki/16-deployment-operations.md) |
| M2–M4 感知与标定 | [03 底盘 CAN](CodeWiki/03-can-chassis.md)、[04 传感器](CodeWiki/04-sensors-hardware.md)、[11 接口字典](CodeWiki/11-interface-reference.md) |
| M5–M7 坐标变换与建图 | [05 SLAM](CodeWiki/05-slam.md)、[08 机器人模型](CodeWiki/08-robot-model.md)、[13 调参手册](CodeWiki/13-tuning-playbook.md) |
| M8–M11 导航与调参 | [06 Nav2](CodeWiki/06-navigation2.md)、[07 局部规划](CodeWiki/07-local-planning.md)、[12 代码地图](CodeWiki/12-code-map.md)、[13 调参手册](CodeWiki/13-tuning-playbook.md) |
| M12 综合与排故 | [10 已知问题](CodeWiki/10-known-issues.md)、[18 故障诊断](CodeWiki/18-faq.md)、[20 术语表](CodeWiki/20-glossary.md) |

## 四、资料导航

| 资料 | 用途 | 建议使用方式 |
| --- | --- | --- |
| [ROS2自动驾驶入门课程标准.md](ROS2自动驾驶入门课程标准.md) | 课程定位、目标、教学条件、M0–M12 教学标准、学时分配、考核评价 | 教师备课与教学管理的第一依据 |
| [ROS2小车系统架构分析.md](ROS2小车系统架构分析.md) | 逐包、逐话题、逐 launch 的源码剖析，含已知缺陷与学习顺序 | 教师讲解、学生对照源码自学 |
| [Code Wiki 源码百科](CodeWiki/README.md) | 20 章结构化文档：架构、模块、类/函数、接口、依赖、编译运行、调参、扩展、测试、运维、排障、ADR、术语 | 教师备课、学生查源码、开发者交接 |
| [仓库根 README](../../README.md) | 两门课程总览、环境准备、常用命令与参数速查 | 首次接触仓库时先读 |
| [课程二：二次开发](../ROS自动驾驶二次开发/README.md) | 33 项进阶项目与实验手册 | 完成 M10 后按需选修 |

## 五、建议学习路径

1. **第 1–3 周**：M0–M1，认车、安全规范、工控机部署，跑通 `colcon build` 与全栈启动。
2. **第 4–7 周**：M2–M4，逐类传感器验证话题、频率与数据质量，掌握 `ros2 topic` / `tf2` 排查。
3. **第 8–11 周**：M5–M7，完成坐标变换理解与场地建图，产出可用地图并测量闭合误差。
4. **第 12–17 周**：M8–M11，依次完成代价地图、定位、规划控制、行为树调试。
5. **第 18–20 周**：M12，综合任务演练与考核，形成实验报告与复盘。

## 六、环境与安全要求

- 环境部署：见[仓库根 README · 实训环境准备](../../README.md#实训环境准备)（ROS 安装、依赖、编译、CAN 与串口配置）。
- 路径约定：工作空间统一为 `~/ros2_ws`，源码位于 `~/ros2_ws/src`。
- 安全要求：实车调试须架空验证先行，首次落地限速 0.2 m/s，安全员持硬件急停随行；仅在园区、库区等非公开道路运行。

## 七、考核与衔接

- 考核方式与比例见[课程标准](ROS2自动驾驶入门课程标准.md)及仓库 README 的[考核评价与实施建议](../../README.md#考核评价与实施建议)。
- 完成 M0–M10 且实车验收合格后，可进入[课程二：ROS 自动驾驶二次开发](../ROS自动驾驶二次开发/README.md)，按"参数级 → 脚本级 → 功能包级"继续进阶。
