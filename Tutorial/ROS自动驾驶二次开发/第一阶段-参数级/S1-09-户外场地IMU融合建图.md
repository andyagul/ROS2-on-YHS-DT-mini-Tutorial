# S1-09 户外场地 IMU 融合建图

> 场景：变电站户外设备区，地面轻微起伏、连续转向多。核心动作：`dgt_mini.lua` 中 `use_imu_data` 由 `false` 改为 `true`。
> 全部命令在**实车工控机 Ubuntu 22.04** 上执行；`WS` 为工作空间根目录。

```bash
# 每个新终端开头都执行（下文不再重复）
export WS="$HOME/ros2_ws"
source /opt/ros/humble/setup.bash
source "$WS/install/setup.bash"
```

## 1. 实验信息

| 项目 | 内容 |
|---|---|
| 实验编号 | S1-09 |
| 所属阶段 | 第一阶段 · 参数级（零代码，只改参数值） |
| 对应清单条目 | 《二次开发项目清单》第一阶段 #9「启用 IMU 参与建图」 |
| 对应方案场景 | 《二次开发场景实施方案》S1-09「户外场地的 IMU 融合建图」 |
| 建议时长 | 4 学时（240 min：基线 1 轮 + 融合 3 轮） |
| 前置实验 | M4 雷达与 IMU 装调标定、M7 Cartographer 建图、S1-08 建图质量调参 |
| 硬件要求 | DT-mini 底盘（`can0`）；nvilidar（`/dev/lidar`，512000 bps）；串口 IMU（`/dev/imu`，115200 bps）；硬件急停 |
| 场地要求 | 平整户外区 ≥ 6 m × 6 m，含 2~3 处特征墙角，可**原地旋转 360°**，警戒线清场 |
| 改动文件 | `src/cartographer_ros/cartographer_ros/configuration_files/dgt_mini.lua`（1 处） |

## 2. 实验目标

1. 把 `TRAJECTORY_BUILDER_2D.use_imu_data` 由 `false` 改为 `true`，并用 `grep` 回读安装目录确认生效。
2. 实测 `/imu_data` 频率落在 50~100 Hz（记录实际值），确认 `base_link→imu_link` 静态 TF 存在且无时间戳告警。
3. 用"逆时针旋转时 `angular_velocity.z` 为正、`linear_acceleration.z ≈ +9.8`"判定 IMU 方向与实车一致。
4. 原地旋转 360° 建图后，同一墙体两次成像错层 ≤ 0.05 m（≤ 1 像素 @0.05 m/px）。
5. 闭环建图 3 轮，至少 2 轮闭合误差 ≤ 0.1 m，IMU 无掉线（时间戳丢包 ≤ 1%）。

## 3. 前置条件与开机检查

### 3.1 设备与权限（终端 1）

```bash
# 执行目录：任意
ls -l /dev/lidar /dev/imu
groups | grep dialout
sudo ip -details link show can0 | head -3
```

预期输出示例：

```
crw-rw---- 1 root dialout 188, 0 11月 16 09:12 /dev/imu
crw-rw---- 1 root dialout 188, 1 11月 16 09:12 /dev/lidar
dialout
3: can0: <NOARP,UP,LOWER_UP,ECHO> mtu 16 ... state UP ...
```

判定：两个设备存在且属组 `dialout`、当前用户在该组；`can0` 为 `UP`。否则执行 `sudo usermod -aG dialout $USER`（重新登录生效）与 `sudo modprobe can && sudo ip link set can0 up type can bitrate 500000`。

### 3.2 IMU 单测（终端 2 启动，终端 3 核查）

```bash
# 终端 2（执行目录：任意）
ros2 launch serial_imu imu_spec_msg.launch.py
```

```bash
# 终端 3（执行目录：任意）
ros2 topic hz /imu_data
ros2 topic echo /imu_data --field linear_acceleration --once
ros2 topic echo /imu_data --field orientation --once
```

预期输出示例：

```
average rate: 100.000
	min: 0.009s max: 0.011s std dev: 0.00041s window: 102
linear_acceleration: {x: 0.31, y: -0.12, z: 9.79}
orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}
```

判定：频率 50~100 Hz（若固件输出 200 Hz，**如实记录**并备注，不篡改参数凑数）；静止时 `linear_acceleration.z ≈ +9.8`。若为 `-9.8` 说明 IMU 倒装或 Z 轴朝下，**必须先修正安装**，禁止打开 `use_imu_data`。

方向一致性：手动把车体逆时针（俯视）旋转约 90°，`ros2 topic echo /imu_data --field angular_velocity` 中 `z` 应为正、量级约 0.5~1.5 rad/s。若为负，记入自检记录并停止实验。

### 3.3 TF 检查

```bash
# 终端 3（终端 2 的 IMU launch 保持运行）
ros2 run tf2_ros tf2_echo base_link imu_link
```

预期输出示例（该 TF 由 `imu_spec_msg.launch.py` 的 `static_transform_publisher` 发布，参数全零）：

```
At time 1763260000.123
- Translation: [0.000, 0.000, 0.000]
- Rotation: in Quaternion [0.000, 0.000, 0.000, 1.000]
```

判定：持续打印且不出现 `Lookup would require extrapolation into the future`；出现即先解决时间/负载问题再建图。

## 4. 目录与文件准备

```bash
# 执行目录：任意
mkdir -p ~/yhs_labs/S1-09/backup ~/yhs_labs/S1-09/result

# 执行目录：$WS —— 备份待改文件
cd "$WS"
cp src/cartographer_ros/cartographer_ros/configuration_files/dgt_mini.lua \
   ~/yhs_labs/S1-09/backup/dgt_mini.lua.orig
diff -q src/cartographer_ros/cartographer_ros/configuration_files/dgt_mini.lua \
        ~/yhs_labs/S1-09/backup/dgt_mini.lua.orig && echo "BACKUP OK"
```

预期输出：`BACKUP OK`

| 类型 | 绝对路径 | 说明 |
|---|---|---|
| 修改 | `$WS/src/cartographer_ros/cartographer_ros/configuration_files/dgt_mini.lua` | 唯一改动文件，第 52 行 |
| 只读参考 | `$WS/src/cartographer_ros/cartographer_ros/launch/dgt_mini.launch.py` | 加载 `dgt_mini.lua`，把 `imu` 重映射为 `imu_data` |
| 只读参考 | `$WS/src/yhs_nav2/launch/cartographer.launch.py` | 建图总入口（`use_tf` 默认 `false`） |
| 只读参考 | `$WS/src/hardware/serial_imu/launch/imu_spec_msg.launch.py` | 发布 `/imu_data` 与 `base_link→imu_link` |
| 新增 | `~/yhs_labs/S1-09/backup/`、`~/yhs_labs/S1-09/result/` | 仅实验产物，不新增源码文件 |

## 5. 实施步骤

### 步骤 1：基线复现（`use_imu_data = false`）

- 操作：未改参数时原地旋转 360° 建图一轮，作对比基线。
- 命令（终端 1，执行目录 `$WS`）：`ros2 launch yhs_nav2 cartographer.launch.py`
- 命令（终端 2，保存地图）：

```bash
ros2 run nav2_map_server map_saver_cli -f ~/yhs_labs/S1-09/result/baseline_no_imu
```

- 预期输出示例（终端 1）：`[cartographer_node-4] [INFO] [cartographer]: Added trajectory with ID '0'.`
- 预期输出（终端 2）：`[INFO] [map_saver]: Map saved successfully`
- 完成标志：`ls -l ~/yhs_labs/S1-09/result/baseline_no_imu.*` 同时出现 `.pgm` 与 `.yaml`；RViz Measure 测得旋转区错层 > 0.05 m（典型 0.1~0.3 m），截图存档。

### 步骤 2：打开 IMU 融合

- 操作：按第 6 节改 `dgt_mini.lua` 第 52 行，然后回读（执行目录：任意）：
  `grep -n "use_imu_data" "$WS/src/cartographer_ros/cartographer_ros/configuration_files/dgt_mini.lua"`
- 预期输出：`52:TRAJECTORY_BUILDER_2D.use_imu_data = true      -- 原值 false`
- 完成标志：grep 显示 `true`（`--` 之后是 Lua 注释，不影响取值）。

### 步骤 3：编译并回读安装目录

- 操作：按第 7 节编译后回读安装目录（执行目录 `$WS`）：
  `grep -n "use_imu_data" install/cartographer_ros/share/cartographer_ros/configuration_files/dgt_mini.lua`

- 预期输出：同样显示 `= true`。
- 完成标志：安装目录与源码一致；仍为 `false` 说明未编译或未 `--symlink-install`。

### 步骤 4：融合建图 3 轮

- 操作：清场 → 启动 → 原地旋转 360° → 闭环采集 ≥ 15 min → 保存 → 下车复位；重复 3 轮，文件名带序号（`imu_on_r1/r2/r3`）。
- 命令（终端 1，执行目录 `$WS`）：`ros2 launch yhs_nav2 cartographer.launch.py`
- 命令（终端 2）：`ros2 run nav2_map_server map_saver_cli -f ~/yhs_labs/S1-09/result/imu_on_r1`
- 预期：终端 1 无 `Failed to get Lidar Data!`、无 IMU 断连，`Ctrl+C` 无崩溃栈。
- 完成标志：`result/` 下 3 组共 6 个文件。

### 步骤 5：量测与归档

- 操作：RViz/图像工具量测错层与闭合误差。
- 判定方法：分辨率 0.05 m/px，错层像素数 × 0.05 m = 错层量；闭合误差取起点特征墙角两处成像的位置差。
- 完成标志：记录表含 baseline、r1、r2、r3 四行的「错层 / 闭合误差 / IMU 掉线次数 / 时长」，并写明结论「后续默认 `use_imu_data = true/false`」。

## 6. 关键配置与代码

**文件**：`src/cartographer_ros/cartographer_ros/configuration_files/dgt_mini.lua`
**位置**：第 44~53 行（`MAP_BUILDER.use_trajectory_builder_2d = true` 之后，`ceres_scan_matcher` 之前）
**改动**：仅 1 行。

```lua
MAP_BUILDER.use_trajectory_builder_2d = true

--TRAJECTORY_BUILDER_2D.num_accumulated_range_data = 2

TRAJECTORY_BUILDER_2D.submaps.num_range_data = 60
TRAJECTORY_BUILDER_2D.min_range = 0.5
TRAJECTORY_BUILDER_2D.max_range = 50
TRAJECTORY_BUILDER_2D.missing_data_ray_length = 5
TRAJECTORY_BUILDER_2D.use_imu_data = true      -- 原值 false
TRAJECTORY_BUILDER_2D.use_online_correlative_scan_matching = true
```

| 参数 | 原值 | 新值 | 含义与影响 |
|---|---|---|---|
| `TRAJECTORY_BUILDER_2D.use_imu_data` | `false` | `true` | 用 IMU 角速度与重力方向约束帧间姿态，连续转向不再纯靠激光猜旋转 → 抑制错层；若 IMU 方向/时间戳异常，地图会整体扭曲，比不开更差 |

配套参数（**本实验不改**，仅解释链路）：

| 位置 | 参数 | 当前值 | 作用 |
|---|---|---|---|
| `options` 表第 21 行 | `tracking_frame` | `"base_link"` | IMU 数据在此坐标系下使用，`base_link→imu_link` 由静态 TF 提供 |
| `options` 表第 40 行 | `imu_sampling_ratio` | `1.0` | IMU 采样比例，1.0 表示全部使用 |

> 注意：`dgt_mini.launch.py` 中 `cartographer_occupancy_grid_node` 的 `use_sim_time` 写死 `True`，`cartographer_node` 用 launch 参数（默认 `False`）。本实验保持现状，**不要顺手改**，否则引入非实验变量。

## 7. 编译与环境刷新

```bash
# 执行目录：$WS
cd "$WS"
source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-select cartographer_ros
source install/setup.bash
```

| 选项 | 作用 |
|---|---|
| `--symlink-install` | 安装目录符号链接指向 `src/`，改 lua 后通常无需重编；仍建议编一次刷新索引 |
| `--packages-select cartographer_ros` | 只编此包，避免全量 40+ 包十几分钟开销 |

预期输出示例：

```
Starting >>> cartographer_ros
Finished <<< cartographer_ros [42.3s]

Summary: 1 package finished [44.1s]
```

判定：出现 `Summary: 1 package finished` 且无 `Failed`。若报 `Could not find a package configuration file provided by "cartographer"`，先编同工作空间的 `src/cartographer`：`colcon build --symlink-install --packages-select cartographer`。

## 8. 运行验证

| 顺序 | 终端 | 职责 | 命令（执行目录 `$WS`） |
|---|---|---|---|
| 1 | 终端 1 | 建图总入口（IMU + 雷达 + 底盘 + Cartographer） | `ros2 launch yhs_nav2 cartographer.launch.py` |
| 2 | 终端 2 | 频率与 TF 监测 | `ros2 topic hz /imu_data`、`ros2 topic hz /scan --qos-reliability best_effort`、`ros2 topic hz /odom` |
| 3 | 终端 3 | TF 树与地图保存 | `ros2 run tf2_tools view_frames`、`ros2 run nav2_map_server map_saver_cli -f ...` |

> `cartographer.launch.py` 的 `use_tf` 默认 `false`：底盘不发布 TF，由 Cartographer 发布 `map→odom→base_link`。这是建图模式应有的 TF 归属，**不要**追加 `use_tf:=true`。

关键核查（终端 2，执行目录：任意）：

```bash
ros2 topic hz /imu_data
ros2 run tf2_ros tf2_echo base_link imu_link
ros2 run tf2_ros tf2_echo map base_link
```

预期输出示例：

```
average rate: 99.998
- Translation: [0.000, 0.000, 0.000]
```

TF 树核查：

```bash
# 执行目录：~/yhs_labs/S1-09/result
cd ~/yhs_labs/S1-09/result && ros2 run tf2_tools view_frames
```

预期：生成 `frames_<时间戳>.pdf` 与 `frames.gv`，树形为 `map → odom → base_link → laser_link / imu_link`。

判定标准（全部满足）：① `/imu_data` 50~100 Hz，30 min 无断流；② 两条 TF 链可持续查询、无 extrapolation 告警；③ 原地旋转时墙体不出现重影，错层 ≤ 1 像素；④ 保存的 `.pgm` 无断墙、无大块空洞。

## 9. 验收指标

| 维度 | 量化要求 | 测量方法 |
|---|---|---|
| 功能完整性 | 原地旋转 360° 错层 ≤ 0.05 m；回环闭合 ≤ 0.1 m | RViz Measure / pgm 数像素 × 0.05 m |
| 功能完整性 | `/imu_data` 50~100 Hz；`imu_link` TF 存在且方向一致 | `topic hz`、`tf2_echo`、逆时针旋转看 `angular_velocity.z` |
| 安全性 | 采集速度 ≤ 0.2 m/s；旋转区清场、安全员持急停 | 现场记录 + 速度指令核对 |
| 稳定性 | 3 轮、单轮 ≥ 15 min 不断图，IMU 无掉线，丢包 ≤ 1% | 三组地图 + 终端日志 |
| 交付物 | 基线 + 3 轮对比地图包、1 页选型建议 | `~/yhs_labs/S1-09/result/` |

## 10. 回滚与常见问题

回滚：

```bash
# 执行目录：$WS
cd "$WS"
cp ~/yhs_labs/S1-09/backup/dgt_mini.lua.orig \
   src/cartographer_ros/cartographer_ros/configuration_files/dgt_mini.lua
colcon build --symlink-install --packages-select cartographer_ros
grep -n "use_imu_data" install/cartographer_ros/share/cartographer_ros/configuration_files/dgt_mini.lua
```

预期回读 `= false--false`。仍为 `true` 时检查 `echo $AMENT_PREFIX_PATH` 是否混入另一套工作空间。

| 现象 / 报错 | 原因 | 处理 |
|---|---|---|
| `unable to open serial port: No such file or directory`，IMU 节点随即退出（退出码 0） | `/dev/imu` 不存在或权限不足（源码路径硬编码 `/dev/imu`，`B115200`） | 补 udev 规则映射串口；`dialout` 组重新登录 |
| `/imu_data` 有数据但地图扭曲、转角被"拉直" | IMU Z 轴朝下或方向不符 | 看 `linear_acceleration.z` 应为 `+9.8`；修正安装或先回滚 |
| 改完参数行为完全没变 | 改 `src/` 但运行旧 `install/` | 按第 7 节编译并 grep 安装目录 |
| `Lookup would require extrapolation into the future, when looking up transform from frame [imu_link]` | 时间戳超前 / CPU 过载 | 关重型节点、RViz 降频，确认未混用仿真时间 |
| `Failed to get Lidar Data!` 后退出 | `/dev/lidar` 掉线或波特率被改 | `ls -l /dev/lidar`；确认 `serialport_baud: 512000` |
| `map_saver_cli` 报 `Failed to save the map` | 输出目录权限或磁盘满 | 输出到 `~/yhs_labs/S1-09/result/`，`df -h` 确认空间 |
