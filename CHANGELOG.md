# 更新日志

本日志遵循 [Keep a Changelog](https://keepachangelog.com/zh-CN/1.1.0/)，
版本号遵循 [语义化版本](https://semver.org/lang/zh-CN/)。

---

## [Unreleased]

### 新增

- **`MobaXterm远程连接操作指引.md`** — Windows 端 MobaXterm SSH/SFTP 连接小车全流程，含 4 类 22 个异常场景排查

- **`.gitignore`** 扩充：新增 Python 编译产物、ROS2 `install/` `log/`、模型权重 `*.pt`/`*.onnx`/`*.engine`、离线包 `*.whl`/`offline_pip/`

### 环境参数统一

- 工作空间根目录：`~/ros2_ws`（移除原有 `~/yhs_ws` 及 `/home/ubuntu/*_ws` 引用）
- SSH 连接目标：`qg@192.168.1.102`（用户名 `qg`、IP `192.168.1.102`，所有指引文档中的 SSH/SCP/rsync 示例同步更新）

## [1.0.0] - 2026-11-18

### 基础功能

- ROS2 工作空间初始化与基础功能包搭建
- 底盘 CAN 控制包 `yhs_can_control`
- 激光雷达驱动与 SLAM 包
- Orbbec 相机驱动包
- Nav2 导航栈集成
