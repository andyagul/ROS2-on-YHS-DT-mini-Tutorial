# 16 · 部署与运维手册

[← 上一章：测试与质量](15-tests-quality.md) · [返回索引](README.md) · [下一章：版本矩阵 →](17-version-matrix.md)

> **适用前提**：Ubuntu 22.04 + ROS 2 Humble + 已按 [09 编译运行](09-build-run-debug.md) 完成工作空间编译。
> **范围说明**：仓库内**没有** systemd 单元、Docker 镜像或 OTA 脚本。本文中的服务化、备份、升级流程是**建议方案**，命令可直接使用，单元文件需按现场路径调整。

---

## 1. 部署拓扑

### 1.1 单车（默认）

```
┌──────────────────────── 车载工控机 (Ubuntu 22.04) ────────────────────────┐
│                                                                          │
│  /dev/lidar ── nvilidar_ros2 ──► /scan ─┐                                 │
│  /dev/imu   ── serial_imu    ──► /imu_data ──► yhs_can_control            │
│  can0       ◄─► yhs_can_control ──► /odom、/chassis_info_fb ◄─ /cmd_vel    │
│                                                                          │
│  yhs_nav2.launch.py:                                                     │
│    map_server + amcl + planner_server + controller_server + bt_navigator  │
│    + behavior_server + smoother_server + velocity_smoother               │
│                                                                          │
│  RViz2（可选） / 上层调度（动作客户端）                                     │
└──────────────────────────────────────────────────────────────────────────┘
```

### 1.2 多车

`yhs_nav2.launch.py` 暴露了 `namespace` / `use_namespace`，并通过 `PushRosNamespace` 作用于整个包含链，因此**话题可以隔离**。但以下内容**没有**机器人前缀，多车会互相覆盖：

- TF 坐标系：`map`、`odom`、`base_link`、`laser_link`、`imu_link`
- 参数文件中的 `robot_base_frame`、`global_frame`
- RViz 配置与固定坐标系

结论：当前代码只支持"**单机多进程**"，不支持"同网多机同名 TF"。多车必须做 frame 前缀化改造（见 [14 扩展指南 §6](14-extension-guide.md)）。

---

## 2. 首次部署清单

| 步骤 | 检查项 | 命令 / 说明 |
| --- | --- | --- |
| 1 | 系统与 ROS | `lsb_release -sc` 为 `jammy`；`ros2 --version` 可用 |
| 2 | 工作空间 | `/workspace` 下有 `src`、`build`、`install`、`log` |
| 3 | 依赖 | `rosdep install --from-paths src --ignore-src -r -y` |
| 4 | CAN | `ip link show can0`，500 kbit/s |
| 5 | 串口 | `/dev/lidar`、`/dev/imu` 存在且当前用户可读写 |
| 6 | 雷达/IMU | 单独启动后话题有数据 |
| 7 | 底盘 | `candump can0` 能看到反馈帧 |
| 8 | 地图 | `yhs_nav2/map/*.yaml` 的 `image` 文件存在 |
| 9 | 参数 | `param/yhs_nav2.yaml` 中无旧版 Nav2 键（K4） |
| 10 | 网络 | 固定 IP、关闭休眠、关闭自动更新 |

---

## 3. 启动编排

### 3.1 建图（Cartographer）

```bash
source /opt/ros/humble/setup.bash
source /workspace/install/setup.bash

ros2 launch yhs_nav2 cartographer.launch.py use_tf:=false
```

该 launch 会拉起：IMU、雷达、底盘（`use_tf=false`，不发布 `odom→base_link`）、Cartographer 节点。

另开终端保存：

```bash
ros2 service call /write_state cartographer_ros_msgs/srv/WriteState \
  "{filename: '/workspace/maps/site_a.pbstream', include_unfinished_submaps: true}"
```

### 3.2 建图（GMapping，备选）

```bash
ros2 launch yhs_nav2 gmapping.launch.py
ros2 run nav2_map_server map_saver_cli -f /workspace/maps/site_a
```

### 3.3 导航

```bash
ros2 launch yhs_nav2 yhs_nav2.launch.py \
  map:=/workspace/install/yhs_nav2/share/yhs_nav2/map/site_a.yaml \
  use_tf:=true \
  params_file:=/workspace/install/yhs_nav2/share/yhs_nav2/param/yhs_nav2.yaml
```

启动后按顺序确认：

```bash
ros2 lifecycle get /map_server          # active
ros2 lifecycle get /amcl                # active
ros2 lifecycle get /controller_server   # active
ros2 lifecycle get /bt_navigator        # active
ros2 topic hz /scan
ros2 topic hz /odom
ros2 run tf2_ros tf2_echo map base_link
```

### 3.4 设置初始位姿

`amcl` 配置了 `set_initial_pose: true`，默认位姿为 `(0,0,0)`。车不在原点时必须显式设置：

```bash
ros2 run yhs_nav2 set_init_amcl_pose.py --ros-args -p x:=1.0 -p y:=2.0 -p theta:=1.57
```

或在 RViz 使用 "2D Pose Estimate"。

---

## 4. 开机自启（建议方案）

### 4.1 CAN 接口

`/etc/systemd/network/80-can.network`：

```ini
[Match]
Name=can0

[CAN]
BitRate=500K
```

```bash
sudo systemctl enable --now systemd-networkd
ip -details link show can0
```

### 4.2 导航服务

`/etc/systemd/system/yhs-nav.service`：

```ini
[Unit]
Description=YHS Nav2 bringup
After=network-online.target
Wants=network-online.target

[Service]
Type=simple
User=yhs
WorkingDirectory=/workspace
Environment=ROS_DOMAIN_ID=0
ExecStart=/bin/bash -lc 'source /opt/ros/humble/setup.bash && \
  source /workspace/install/setup.bash && \
  exec ros2 launch yhs_nav2 yhs_nav2.launch.py'
Restart=on-failure
RestartSec=5
KillSignal=SIGINT
TimeoutStopSec=20

[Install]
WantedBy=multi-user.target
```

```bash
sudo systemctl daemon-reload
sudo systemctl enable --now yhs-nav
journalctl -u yhs-nav -f
```

> `KillSignal=SIGINT` 很重要：launch 需要 SIGINT 才能按顺序关闭生命周期节点。

---

## 5. 日志与数据采集

### 5.1 节点日志

```bash
ros2 launch ... --ros-args --log-level debug        # 临时
export RCUTILS_LOGGING_BUFFERED_STREAM=1            # launch 已设置
ls ~/.ros/log/                                      # 每次运行的日志目录
```

### 5.2 录包

```bash
ros2 bag record -o /data/bags/run_$(date +%Y%m%d_%H%M%S) \
  /scan /imu_data /odom /tf /tf_static \
  /cmd_vel /amcl_pose /plan /local_plan /map
```

Humble 默认 `sqlite3`；若安装了 MCAP 插件可加 `-s mcap`。

| 话题 | 频率量级 | 说明 |
| --- | --- | --- |
| `/scan` | 10~15 Hz | 单线雷达，体积最大 |
| `/imu_data` | 50~100 Hz | 体积中等 |
| `/tf` | 20~50 Hz | 必需，用于离线复现 |
| `/map` | 事件触发 | 建议单独保存 PGM/YAML，不必反复录 |

> 磁盘估算：以 10 Hz 雷达 + 100 Hz IMU 计，连续录制约每小时数 GB，取决于雷达点数和 bag 压缩。出发前用 `df -h` 确认剩余空间，并设置轮转清理。

### 5.3 复现

```bash
ros2 bag play /data/bags/run_xxx --clock
# 另开终端，use_sim_time:=true 启动算法节点
```

注意：`yhs_can_control` 会真实驱动底盘，回放时**不要**启动底盘节点，或先断开 CAN。

---

## 6. 配置与地图备份

需要备份的资产：

| 资产 | 路径 | 说明 |
| --- | --- | --- |
| 地图 | `src/yhs_nav2/map/*.pgm|yaml` | 现场采集，最不可再生 |
| 参数 | `src/yhs_nav2/param/yhs_nav2.yaml` | 调参结果 |
| 标定 | launch 中的静态 TF、URDF | 雷达/IMU/相机安装位置 |
| 设备规则 | `/etc/udev/rules.d/*.rules` | `/dev/lidar`、`/dev/imu` |
| 代码版本 | `git rev-parse HEAD` | 与地图/参数对应 |

```bash
tar czf /backup/yhs_$(date +%F).tgz \
  src/yhs_nav2/map src/yhs_nav2/param \
  src/hardware/*/launch src/yhs_chassis_description/urdf \
  /etc/udev/rules.d/99-yhs.rules
git rev-parse HEAD > /backup/yhs_$(date +%F).commit
sha256sum src/yhs_nav2/map/* | tee /backup/yhs_$(date +%F).sha256
```

> **安装副本陷阱**：launch 通过 `get_package_share_directory` 读取 `install/` 下的地图和参数。只改 `src/` 不重新 `colcon build --packages-select yhs_nav2`，运行时仍用旧文件。建议编译时使用 `--symlink-install`，或启动时显式传绝对路径。

---

## 7. 升级与回滚

### 7.1 升级

```bash
cd /workspace
git fetch --all
git checkout <tag-or-commit>
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release
source install/setup.bash
```

只改底盘/接口时，可缩短编译：

```bash
colcon build --packages-select yhs_can_interfaces yhs_can_control yhs_nav2
```

### 7.2 回滚

```bash
git checkout <previous-commit>
colcon build --symlink-install
```

地图与参数独立于代码版本，回滚代码后要确认 YAML 键名与旧版本 Nav2 兼容（见 K4）。

### 7.3 变更窗口检查表

- [ ] 备份当前地图与参数
- [ ] 记录 `git rev-parse HEAD`
- [ ] 新版本在台架验证 `/odom`、`/scan`、TF
- [ ] 低速（≤0.2 m/s）实车验证
- [ ] 确认恢复行为、急停可用
- [ ] 更新交接文档中的版本号

---

## 8. 运行监控与巡检

### 8.1 实时指标

```bash
ros2 topic hz /scan /odom /imu_data
ros2 topic delay /scan
ros2 lifecycle get /amcl
ros2 param dump /controller_server | head -40
top -b -n1 | head -20
free -h && df -h
dmesg -T | tail -50
```

### 8.2 日常巡检表

| 项目 | 正常范围 | 异常处理 |
| --- | --- | --- |
| `/scan` 频率 | 与雷达配置一致（约 10 Hz） | 检查串口、供电、`dmesg` |
| `/odom` 频率 | 约 20 Hz | 检查 CAN、底盘使能 |
| `/imu_data` 频率 | 50~100 Hz | 检查 `/dev/imu` |
| `map→base_link` TF | 连续、无跳变 | 检查 AMCL、雷达定位 |
| CPU | 持续 < 80% | 降低雷达频率/代价地图分辨率（K19） |
| 磁盘 | 使用率 < 80% | 清理 bag 与日志 |
| 电池/急停 | 按现场规程 | 立即停止任务 |

---

## 9. 交接清单

- [ ] 地图文件、`origin` 与实际场地一致
- [ ] 参数文件版本与代码 commit 对应
- [ ] 设备名、波特率、CAN 速率有记录
- [ ] 初始位姿设置方式已培训
- [ ] 急停、远程停止、重启流程已演练
- [ ] 日志与 bag 存放路径、清理策略已约定
- [ ] 已知问题清单（[10](10-known-issues.md)）已移交
