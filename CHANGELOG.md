# 更新日志

本日志遵循 [Keep a Changelog](https://keepachangelog.com/zh-CN/1.1.0/)，
版本号遵循 [语义化版本](https://semver.org/lang/zh-CN/)。

---

## [Unreleased]

### 新增

- **`src/yhs_traffic_sign/`** — 全新 ROS2 Python 功能包，实现交通标识牌识别与自动响应。
  - `traffic_sign_detector.py`：YOLOv8 推理引擎封装，含模拟模式降级（无模型时返回虚拟检测结果），推理耗时监控
  - `distance_estimator.py`：深度图中位数测距 + 单目几何针孔模型回退，移动平均滤波 + 跳变检测恢复
  - `vehicle_controller.py`：四态状态机（IDLE → CONFIRMING → TRIGGERED → COOLDOWN），连续 N 次确认防抖，冷却时间防重复触发
  - `anomaly_handler.py`：低置信度熔断、标识牌短时丢失跟踪保持、多标识牌智能选择、距离无效历史回退
  - `traffic_sign_response.py`：ROS2 主节点，订阅 `/camera/color/image_raw` + `/camera/depth/image_raw`，发布 `/ctrl_cmd` 和 `/traffic_sign/status`
  - `config.py`：配置常量集中管理（类别 ID 映射、触发阈值、相机内参等）
  - `setup.py` / `package.xml` / `param/traffic_sign_params.yaml` / `launch/traffic_sign_response.launch.py` / `pytest.ini` / `resource/yhs_traffic_sign`
  - `models/traffic_sign_dataset.yaml`：自定义 3 类（left_turn / straight / right_turn）数据集配置模板
  - `models/yolov8n.pt`：官方 YOLOv8n COCO 预训练权重（**已加入 .gitignore，使用时需自行下载或训练**）

- **单元测试**（`test/` 目录），共 5 个测试文件，覆盖检测、测距、控制、异常处理和全链路集成

- **三份实验操作指引文档**（中文，手把手式）
  - `MobaXterm远程连接操作指引.md`：Windows 端 MobaXterm SSH/SFTP 连接小车全流程，含 4 类 22 个异常场景排查
  - `src/yhs_traffic_sign/实验操作指引-交通标识牌识别.md`：从代码部署 → 工作空间编译 → 数据采集 → 模型训练 → 实车验证的完整五步实验流程
  - `src/yhs_traffic_sign/离线安装指南-Python依赖.md`：小车无外网场景下 ultralytics / OpenCV / torch 的离线安装三步法（Windows 下载 → SFTP 上传 → `pip3 install --no-index`）

- **`.gitignore`** 扩充：新增 Python 编译产物、ROS2 `install/` `log/`、模型权重 `*.pt`/`*.onnx`/`*.engine`、离线包 `*.whl`/`offline_pip/`

### 环境参数统一

- 工作空间根目录：`~/ros2_ws`（移除原有 `~/yhs_ws` 及 `/home/ubuntu/*_ws` 引用）
- SSH 连接目标：`qg@192.168.1.102`（用户名 `qg`、IP `192.168.1.102`，所有指引文档中的 SSH/SCP/rsync 示例同步更新）
- 类别 ID 映射：`0=left_turn, 1=straight, 2=right_turn`（代码、训练配置、测试用例三处一致）

### 已知限制

- `yolov8n.pt` 为 COCO 80 类通用检测权重，**不包含 left_turn / straight / right_turn 三类**，仅用于推理管道验证。生产部署需基于此权重做迁移学习微调
- 多数 ROS 小车工控机（Ubuntu 22.04）**无法访问外网**（DNS 解析失败、pypi.org 不可达），`pip3 install` 必须走离线三步法（详见离线安装指南）
- 官方 torch 权重约 200MB（CPU 版），建议在 Python 3.10 venv 内通过 `pip download --python-version 3.10` 下载 manylinux2014_x86_64 版本
