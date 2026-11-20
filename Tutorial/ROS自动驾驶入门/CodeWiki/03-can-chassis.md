# 03 · 底盘控制与 CAN 协议

[← 上一章：包清单](02-packages.md) · [返回索引](README.md) · [下一章：硬件驱动 →](04-sensors-hardware.md)

---

## 1. 模块职责

`yhs_can_control` 是整车唯一的**执行与里程计出口**：

| 方向 | 输入 | 输出 |
| --- | --- | --- |
| 下行（控制） | `/cmd_vel`（Twist）、`/ctrl_cmd`、`/free_ctrl_cmd`、`/io_cmd` | CAN 帧（速度、轮速、灯光） |
| 上行（反馈） | CAN 反馈帧 | `/chassis_info_fb`、`/odom`、`odom→base_link` TF |

- 头文件：[yhs_can_control_node.hpp](../../../src/yhs_can_control/include/yhs_can_control/yhs_can_control_node.hpp)
- 实现：[yhs_can_control_node.cpp](../../../src/yhs_can_control/src/yhs_can_control_node.cpp)
- 参数：[cfg.yaml](../../../src/yhs_can_control/params/cfg.yaml)
- 启动：[yhs_can_control.launch.py](../../../src/yhs_can_control/launch/yhs_can_control.launch.py)

---

## 2. 类与函数说明

### 2.1 `class yhs::CanControl`

定义于头文件 `:46-98`，构造函数接收外部创建的 `rclcpp::Node::SharedPtr`（组合式设计）。

| 成员 | 类型 | 说明 |
| --- | --- | --- |
| `node_` | `rclcpp::Node::SharedPtr` | 宿主节点 `yhs_can_control_node` |
| `if_name_` | `std::string` | CAN 接口名，参数 `can_name`，默认 `can0` |
| `use_tf_` | `bool` | 是否发布 `odom→base_link`，参数 `use_tf`，默认 `false`（launch 默认 `true`） |
| `can_socket_` | `int` | SocketCAN 原始套接字 |
| `thread_` | `std::thread` | CAN 接收线程 |
| `mutex_` | `std::mutex` | 保护 `imu_roll/pitch/yaw_` |
| `imu_yaw_` | `double` | IMU 解算出的航向角（rad） |

| 方法 | 位置 | 行为 |
| --- | --- | --- |
| `CanControl(node)` | `:5-43` | 读参数、创建 5 个订阅、2 个发布、TF 广播器 |
| `run()` | `:472-500` | `socket(PF_CAN, SOCK_RAW, CAN_RAW)` → `ioctl(SIOCGIFINDEX)` → `bind` → 启动接收线程；失败返回 `false` |
| `stop()` | `:502-512` | 关闭套接字并 `join` 接收线程 |
| `io_cmd_callback()` | `:46-106` | 组装 `0x98C4D7D0` 灯光帧 |
| `ctrl_cmd_callback()` | `:108-151` | 组装 `0x98C4D1D0` 速度/转角帧 |
| `cmd_callback()` | `:153-161` | `Twist` → `CtrlCmd`：`linear.x` 直传、`angular.z×180/π`、`gear=3` |
| `free_ctrl_cmd_callback()` | `:163-206` | 组装 `0x98C4D2D0` 左右轮速帧 |
| `can_data_recv_callback()` | `:232-386` | 接收线程主循环：`select(10ms)` → `read` → 按 `can_id` 分发、校验、发布 |
| `wait_for_can_frame()` | `:209-230` | `select` 等待 10 ms；超时打印"检查 can0/接线/上电" |
| `imu_data_callback()` | `:388-396` | 四元数 → RPY，更新 `imu_yaw_` |
| `publish_odom(linear, angular)` | `:398-465` | 积分位姿、写协方差、发 `/odom`、可选发 TF |

### 2.2 订阅与发布

| 名称 | 类型 | 方向 | QoS |
| --- | --- | --- | --- |
| `cmd_vel` | `geometry_msgs/Twist` | 订阅 | 1 |
| `ctrl_cmd` | `yhs_can_interfaces/CtrlCmd` | 订阅 | 1 |
| `free_ctrl_cmd` | `yhs_can_interfaces/FreeCtrlCmd` | 订阅 | 1 |
| `io_cmd` | `yhs_can_interfaces/IoCmd` | 订阅 | 1 |
| `imu_data` | `sensor_msgs/Imu` | 订阅 | 1 |
| `chassis_info_fb` | `yhs_can_interfaces/ChassisInfoFb` | 发布 | 1 |
| `odom` | `nav_msgs/Odometry` | 发布 | 1 |
| `odom→base_link` | `tf2_msgs/TFMessage` | 发布（受 `use_tf` 控制） | — |

> 所有话题均为**相对名**，未加命名空间时位于根命名空间（`/cmd_vel`、`/odom`）。

---

## 3. CAN 协议

物理层：Linux SocketCAN，`can0`，**500 kbit/s**（由外部 `ip link` 配置，代码不设置波特率）。所有帧 `can_dlc=8`，`data[7]` 为 `data[0..6]` 的逐字节 XOR 校验。

### 3.1 发送帧（控制）

#### `0x98C4D1D0` — 速度/转角控制

| 字节 | 内容 |
| --- | --- |
| `data[0]` 低 4 位 | `gear`（挡位） |
| `data[0]` 高 4 位 | `linear` 的 bit0-3 |
| `data[1]` | `linear` 的 bit4-11 |
| `data[2]` 低 4 位 | `linear` 的 bit12-15 |
| `data[2]` 高 4 位 | `angular` 的 bit0-3 |
| `data[3]` | `angular` 的 bit4-11 |
| `data[4]` 低 4 位 | `angular` 的 bit12-15 |
| `data[6]` | `count << 4`（0..15 循环） |
| `data[7]` | XOR 校验 |

编码：`linear = ctrl_cmd_linear × 1000`（→ mm/s 的 16 位有符号数）；`angular = ctrl_cmd_angular × 100`（→ 0.01 °/s）。

#### `0x98C4D2D0` — 左右轮独立速度

与上面同构，只是把 `linear/angular` 换成 `free_ctrl_cmd_velocity_l/r × 1000`。

#### `0x98C4D7D0` — 灯光 / IO

| 字节 | 位 | 含义 |
| --- | --- | --- |
| `data[0]` | bit0 | 灯光总控 `io_cmd_lamp_ctrl` |
| | bit1 | 门锁 `io_cmd_unlock` |
| `data[1]` | bit0 | 近光灯 |
| | bit1 | 远光灯 |
| | bit2-3 | 转向灯（0 关 / 1 左 / 2 右） |
| | bit4 | 刹车灯 |
| | bit5 | 示廓灯 |
| | bit6 | 雾灯 |
| `data[2]` | — | 喇叭音量 |
| `data[6]` | — | `count << 4` |
| `data[7]` | — | XOR 校验 |

> 注意：源码中灯光位通过 `sendDataTemp[x] &= 0xfe/0xfd/...` 置位，而初值是 `0xff`，实际效果是"条件成立时清除该位"（低有效），与直觉相反。改动前先对照底盘协议文档。

### 3.2 接收帧（反馈）

| CAN ID | 消息 | 关键解码 |
| --- | --- | --- |
| `0x98C4D1EF` | `CtrlFb` → 触发发布 | 挡位 = `data[0]&0x0f`；线速度 16 位 `/1000`；角速度 16 位 `/100`（°/s） |
| `0x98C4D7EF` | `LWheelFb` | 速度 16 位 `/1000`；脉冲 32 位 |
| `0x98C4D8EF` | `RWheelFb` | 同上 |
| `0x98C4DAEF` | `IoFb` | 灯光、门锁、6 路碰撞、6 路跌落、急停、手柄、充电状态 |
| `0x98C4E1EF` | `BmsFb` | 电压 `/100`、电流 `/100`、剩余容量 `/100` |
| `0x98C4E2EF` | `BmsFlagFb` | SOC、过压/欠压/过流/温度保护标志、高低温 `/10` |

**发布时机**：`ChassisInfoFb` 只在收到 `0x98C4D1EF` 且校验通过时发布；其余反馈帧只更新同一个消息对象，因此下游看到的 `io_fb`/`bms_fb`/`l_wheel_fb` 是**最近一次收到的值**（消息对象声明在 while 循环外，`:236`）。

---

## 4. 里程计计算 `publish_odom`

源码 `:398-465`，算法要点：

```text
dt = now - last_time
x += v * cos(theta) * dt
y += v * sin(theta) * dt

if count_publishers("/imu_data") == 0:
    theta += omega * dt          # 无 IMU：角速度积分
else:
    theta = imu_yaw_             # 有 IMU：直接使用 IMU 绝对航向
```

- 位姿用 `static` 局部变量保存，**节点重启即归零**。
- 姿态只使用 yaw，roll/pitch 丢弃；`z` 恒为 0。
- 协方差矩阵**未填充**（默认全 0），AMCL 的 `odom` 运动模型会因此偏乐观。
- `use_tf_` 为 `true` 时才广播 `odom→base_link`。

**风险**：`count_publishers("/imu_data")` 只判断"有没有发布者"，不判断"数据是否新鲜"；IMU 节点挂死时航向会卡在最后一个值。IMU yaw 若在 ±π 跳变，里程计会瞬间跳变。

---

## 5. 典型时序

```
导航/遥控                 yhs_can_control                底盘
   │  /cmd_vel (0.3 m/s, 0.2 rad/s) │                       │
   ├───────────────────────────────►│                       │
   │                                │ 0x98C4D1D0 (300, 11.5)│
   │                                ├──────────────────────►│
   │                                │                       │ 执行
   │                                │ 0x98C4D1EF (300, 11.5)
   │                                │◄──────────────────────┤
   │                                │ 校验 → 发布 /odom      │
   │                                │ TF odom→base_link      │
   │                                │ /chassis_info_fb       │
```

---

## 6. 消息与动作定义

目录：[yhs_can_interfaces/msg](../../../src/yhs_can_interfaces/msg)、[action](../../../src/yhs_can_interfaces/action/RecordPath.action)

| 消息 | 字段 |
| --- | --- |
| `CtrlCmd` | `uint8 ctrl_cmd_gear`、`float32 ctrl_cmd_linear`、`float32 ctrl_cmd_angular` |
| `FreeCtrlCmd` | `uint8 ctrl_cmd_gear`、`float32 free_ctrl_cmd_velocity_l`、`float32 free_ctrl_cmd_velocity_r` |
| `CtrlFb` | `uint8 ctrl_fb_target_gear`、`float32 ctrl_fb_linear`、`float32 ctrl_fb_angular` |
| `LWheelFb` / `RWheelFb` | `float32 *_wheel_fb_velocity`、`int32 *_wheel_fb_pulse` |
| `IoCmd` | 灯光/门锁/远近光/转向灯/刹车/示廓/雾灯/喇叭 共 9 项 |
| `IoFb` | 灯光/门锁/6 路碰撞/6 路跌落/急停/手柄/充电 共 24 项 |
| `BmsFb` | 电压、电流、剩余容量 |
| `BmsFlagFb` | SOC、14 个保护标志、最高/最低温度 |
| `ChassisInfoFb` | `header` + `ctrl_fb` + `io_fb` + `l_wheel_fb` + `r_wheel_fb` + `bms_flag_fb` + `bms_fb` |
| `RecordPath.action` | Goal: `nav_msgs/Path path`、`string behavior_tree`；Result: `std_msgs/Empty`；Feedback: 当前位姿、导航时间、剩余时间、恢复次数、剩余距离 |

> `RecordPath.action` 是**自定义示教循迹**的契约，被 `nav2_behavior_tree` 与 `nav2_bt_navigator` 直接引用，因此 Nav2 的编译依赖了 `yhs_can_interfaces`。

---

## 7. 参数与启动

```yaml
# cfg.yaml
yhs_can_control_node:
  ros__parameters:
    can_name: "can0"
```

launch 额外传入 `use_tf`（默认 `true`）：

```bash
ros2 launch yhs_can_control yhs_can_control.launch.py use_tf:=true
ros2 topic echo /odom --once
```

**`use_tf` 使用矩阵**：

| 场景 | `use_tf` | 原因 |
| --- | --- | --- |
| 单独调底盘 / 导航（AMCL） | `true` | 需要 `odom→base_link` |
| Cartographer 建图 | `false`（launch 默认） | 由 Cartographer 发布 `odom→base_link` |
| Gmapping 建图 | `true` | Gmapping 只需要 `map→odom` |

---

## 8. 代码审查要点

1. **写返回值**：`write()` 只判断 `ret <= 0`，未处理短写（CAN 帧通常原子，但严格应校验 `ret == sizeof(frame)`）。
2. **线程退出**：`can_data_recv_callback` 依赖 `rclcpp::ok()`，主线程 `rclcpp::shutdown()` 后 `stop()` 才 `join`，顺序正确；但 `select` 超时 10 ms，退出延迟可接受。
3. **无报文超时保护**：底盘掉线时只打印警告，不会触发停车；安全联锁需上层实现。
4. **IO 位操作**：见 3.1 节，"与直觉相反"的低有效写法。
5. **`count_publishers` 语义**：应改为"最近 N 秒是否收到 IMU"，否则 IMU 假死会导致航向冻结。
6. **`gear=3` 写死**：`cmd_callback` 固定挡位 3，倒车需走 `/ctrl_cmd` 或改代码。
7. **相对话题名**：放入命名空间后 `cmd_vel` 会变成 `/ns/cmd_vel`，需同步 remap（Nav2 默认 `cmd_vel` 在根命名空间）。
