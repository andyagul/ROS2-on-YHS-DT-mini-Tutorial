# 04 · 硬件与传感器

[← 上一章：底盘控制](03-can-chassis.md) · [返回索引](README.md) · [下一章：SLAM →](05-slam.md)

---

本工作空间的传感器分三类：

| 设备 | 包 | 接口 | 坐标系 | 在 launch 中的位置 |
| --- | --- | --- | --- | --- |
| 单线激光雷达 | `nvilidar_ros2` | `/dev/lidar` 512000 或 UDP | `laser_link` | 所有建图/导航入口均启用 |
| 串口 IMU | `serial_imu` | `/dev/imu` 115200（硬编码） | `imu_link` | 所有入口均启用 |
| 深度相机 | `OrbbecSDK_ROS2` | USB | `camera_link` | **已注释**，按需启用 |
| 点云转激光 | `pointcloud_to_laserscan` | 订阅点云 | `camera_link` → `/scan1` | **已注释** |

---

## 1. 激光雷达 `nvilidar_ros2` 1.0.0

### 1.1 目录

```
hardware/nvilidar_ros2/
├── src/
│   ├── nvilidar_ros2_node.cpp     # ROS 2 节点主入口
│   └── nvilidar_ros2_client.cpp   # 客户端示例
├── sdk/
│   ├── src/impl/                  # socket / serial 抽象（Win + Unix）
│   ├── src/nvilidar/              # 驱动、协议、滤波、网络配置
│   └── samples/main.cpp           # SDK 示例
├── params/nvilidar.yaml
├── launch/nvilidar.launch.py
└── startup/initenv.sh             # 设备初始化脚本
```

### 1.2 节点行为

`main()`（[nvilidar_ros2_node.cpp](../../src/hardware/nvilidar_ros2/src/nvilidar_ros2_node.cpp)）：

1. `READ_PARAM` 读取 `nvilidar.yaml` 中的全部参数。
2. 创建发布者 `scan`，QoS 为 `rclcpp::SensorDataQoS()`（**best effort**）。
3. 按 `serialport_name` 打开串口（或 `ip_addr` 走 UDP），启动 SDK 采样线程。
4. 每帧填充 `sensor_msgs/LaserScan`：
   - `header.frame_id = frame_id`（默认 `laser_link`）
   - `angle_min/max/increment`、`scan_time`、`time_increment`
   - `range_min/max`、`ranges[]`（按角度索引落点）
5. 发布 `scan`。

### 1.3 关键参数（[nvilidar.yaml](../../src/hardware/nvilidar_ros2/params/nvilidar.yaml)）

| 参数 | 值 | 说明 |
| --- | --- | --- |
| `serialport_name` | `/dev/lidar` | 串口设备（需 udev 规则） |
| `serialport_baud` | `512000` | 波特率 |
| `ip_addr` / `lidar_udp_port` / `config_udp_port` | `192.168.1.200` / 8100 / 8200 | 网络模式 |
| `frame_id` | `laser_link` | 必须与静态 TF 一致 |
| `resolution_fixed` | `true` | 固定角度分辨率 |
| `reversion` / `inverted` | `false` / `false` | 方向与镜像 |
| `angle_min` / `angle_max` | -180 / 180 | 度量程（°） |
| `range_min` / `range_max` | 0.001 / 64.0 | 距离量程（m） |
| `sampling_rate` | 10 | 采样频率 |
| `filter_*` | 多项 | 滑窗/尾部滤波、阈值 |

### 1.4 静态 TF

[nvilidar.launch.py](../../src/hardware/nvilidar_ros2/launch/nvilidar.launch.py) 同时启动：

```
static_transform_publisher 0 0 0.33 0 0 0 1 base_link laser_link
```

即雷达安装在 `base_link` 正上方 **0.33 m**。改动安装位置时必须同步改这里，否则建图会"墙体倾斜"。

### 1.5 验证

```bash
ros2 launch nvilidar_ros2 nvilidar.launch.py
ros2 topic hz /scan --qos-reliability best_effort
ros2 run tf2_ros tf2_echo base_link laser_link
```

---

## 2. 串口 IMU `serial_imu`

### 2.1 组成

| 文件 | 作用 |
| --- | --- |
| [serial_port.cpp](../../src/hardware/serial_imu/src/serial_port.cpp) | ROS 2 节点：打开串口、解析、发布 `/imu_data` |
| [ch_serial.c](../../src/hardware/serial_imu/src/ch_serial.c) / [ch_serial.h](../../src/hardware/serial_imu/src/ch_serial.h) | 串口读写与协议解包 |
| [sub_spec.cpp](../../src/hardware/serial_imu/src/sub_spec.cpp) | `listener` 可执行文件，打印 IMU 原始字段，调试用 |
| [imu_spec_msg.launch.py](../../src/hardware/serial_imu/launch/imu_spec_msg.launch.py) | 启动节点 + 静态 TF |

### 2.2 关键事实

- 设备路径**硬编码**：`#define IMU_SERIAL ("/dev/imu")`（`serial_port.cpp:17`）。
- 波特率**硬编码**：`#define BAUD (B115200)`（`:18`）。
- 发布话题：`/imu_data`（`serial_port.cpp:52`），类型 `sensor_msgs/Imu`。
- 单位换算：
  - 陀螺仪：`gyr × DEG_TO_RAD`
  - 加速度：`acc × GRA_ACC`
  - 姿态：直接使用模块输出的四元数
- `header.frame_id = "imu_link"`。
- **没有任何 ROS 参数**，改端口/波特率只能改源码。

### 2.3 启动与 TF

```bash
ros2 launch serial_imu imu_spec_msg.launch.py
```

launch 内容：

1. 启动 `serial_imu` 节点（名称 `serial_imu_node`）。
2. 启动静态 TF：`base_link → imu_link`，平移/旋转全为 0（`0 0 0 0 0 0 1`）。

> 如果 IMU 安装有偏移或旋转，需要在 launch 中修改这组参数，或在 URDF 中补充。

### 2.4 验证

```bash
ls -l /dev/imu                     # 不存在则配置 udev
ros2 topic hz /imu_data            # 期望 50~100 Hz
ros2 topic echo /imu_data --once
```

---

## 3. 深度相机 `OrbbecSDK_ROS2`

### 3.1 包结构

| 包 | 版本 | 职责 |
| --- | --- | --- |
| `orbbec_camera` | 1.5.11 | 组件化相机节点 `ob_camera_node`、驱动、服务、解码器（JPEG/MJPEG/RK MPP/Jetson） |
| `orbbec_camera_msgs` | 1.5.11 | `Metadata`、`IMUInfo`、`Extrinsics`、`DeviceInfo` 等 |
| `orbbec_description` | 1.5.11 | 相机 URDF/xacro 与 STL（Gemini 335/336、Femto Bolt、Astra 等） |

核心源文件：

- [ob_camera_node.cpp](../../src/hardware/OrbbecSDK_ROS2/orbbec_camera/src/ob_camera_node.cpp)：流配置、发布者创建、帧回调
- [ob_camera_node_driver.cpp](../../src/hardware/OrbbecSDK_ROS2/orbbec_camera/src/ob_camera_node_driver.cpp)：设备枚举与生命周期
- [image_publisher.cpp](../../src/hardware/OrbbecSDK_ROS2/orbbec_camera/src/image_publisher.cpp)：图像发布
- [ros_service.cpp](../../src/hardware/OrbbecSDK_ROS2/orbbec_camera/src/ros_service.cpp)：`get/set` 相机参数服务
- `d2c_viewer.cpp`、`jpeg_decoder.cpp`、`jetson_nv_decoder.cpp`、`rk_mpp_decoder.cpp`：对齐与硬件解码

### 3.2 典型输出

| 话题（默认命名空间 `/camera`） | 类型 |
| --- | --- |
| `color/image_raw` | `sensor_msgs/Image` |
| `depth/image_raw` | `sensor_msgs/Image` |
| `depth/points` | `sensor_msgs/PointCloud2` |
| `depth_registration/points` | `sensor_msgs/PointCloud2` |
| `*/camera_info` | `sensor_msgs/CameraInfo` |
| `imu` / `gyro_accel` | `sensor_msgs/Imu`（部分机型） |
| `depth_filter_status` | `std_msgs/String` |

### 3.3 在本项目中的状态

[yhs_nav2.launch.py:133-146](../../src/yhs_nav2/launch/yhs_nav2.launch.py) 中相机与点云转激光的 `IncludeLaunchDescription` **被注释**。启用步骤：

1. 取消相机 launch 注释，确认机型（如 `astra.launch.py`、`gemini2.launch.py`）与实机一致。
2. 取消 `pointcloud_to_laserscan.launch.py` 注释。
3. 确认 `/scan1` 出现在 `local_costmap` / `global_costmap` 的 `observation_sources` 中（当前 YAML 已配置 `scan` 与 `scan1`）。

---

## 4. 点云转激光 `pointcloud_to_laserscan` 2.0.1

### 4.1 类与流程

类：`pointcloud_to_laserscan::PointCloudToLaserScanNode`（[pointcloud_to_laserscan_node.cpp](../../src/pointcloud_to_laserscan/src/pointcloud_to_laserscan_node.cpp)）

```
cloud_in (PointCloud2)
   └─► 过滤高度 [min_height, max_height]
        └─► 变换到 target_frame
             └─► 按角度分箱，取最近点
                  └─► scan (LaserScan, SensorDataQoS)
```

订阅：`cloud_in`；发布：`scan`。

### 4.2 本项目 launch

[pointcloud_to_laserscan.launch.py](../../src/pointcloud_to_laserscan/launch/pointcloud_to_laserscan.launch.py)：

| 项 | 值 |
| --- | --- |
| 静态 TF | `base_link → camera_link`，`x=0.23, z=0.2` |
| 输入 remap | `cloud_in → /camera/depth/points` |
| 输出 remap | `scan → /scan1` |
| `target_frame` | `camera_link` |
| `min_height` / `max_height` | -0.1 / 2.0 |
| `angle_min` / `angle_max` / `angle_increment` | -1.5708 / 1.5708 / 0.0087 |
| `range_min` / `range_max` | 0.05 / 1.0 |
| `use_inf` / `inf_epsilon` | true / 1.0 |

> `range_max=1.0` 意味着相机只补盲 1 m 内的低矮障碍；这与 `global_costmap` 中 `scan1` 的 `raytrace_max_range=3.0` 不一致，属于预期内的"短距补盲"配置，不要误改。

---

## 5. 设备与权限清单

```bash
# CAN
sudo modprobe can
sudo ip link set can0 up type can bitrate 500000
candump can0

# 串口权限
sudo usermod -aG dialout $USER      # 重新登录

# 设备名（建议 udev 固定）
ls -l /dev/lidar /dev/imu
```

udev 规则示例（按实际 `idVendor/idProduct` 或串口序列号编写）：

```
KERNEL=="ttyUSB*", ATTRS{idVendor}=="xxxx", ATTRS{idProduct}=="xxxx", SYMLINK+="lidar", MODE="0666"
KERNEL=="ttyUSB*", ATTRS{idVendor}=="yyyy", ATTRS{idProduct}=="yyyy", SYMLINK+="imu",   MODE="0666"
```

---

## 6. 传感器层排查顺序

1. `/dev/*` 是否存在、权限是否 `dialout`。
2. 驱动节点是否启动、话题频率是否正常。
3. 静态 TF 是否发布（`tf2_echo base_link laser_link`）。
4. 时间戳是否与系统时间同源（IMU 使用 `rclcpp::Clock().now()`）。
5. QoS：激光为 best effort，`ros2 topic echo` 需加 `--qos-reliability best_effort`。
