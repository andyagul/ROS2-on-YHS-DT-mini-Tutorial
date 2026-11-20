# MobaXterm 远程连接 YHS DT-mini 智能小车 · 标准化操作指引

> **适用对象：** 零基础学生、课程助教、实验室管理员
> **设备环境：** Windows 笔记本（学生端） + 煜禾森 DT-mini ROS2 小车（被控端）
> **小车工控机：** Ubuntu 22.04 LTS + ROS 2 Humble
> **最新更新：** 2026-11-19

---

## 目录

- [一、前置准备](#一前置准备)
- [二、分步操作](#二分步操作)
  - [步骤 1 — 下载安装 MobaXterm](#步骤-1--下载安装-mobaxterm)
  - [步骤 2 — 小车开机 & 确认网络](#步骤-2--小车开机--确认网络)
  - [步骤 3 — 学生机网络连通性检查](#步骤-3--学生机网络连通性检查)
  - [步骤 4 — MobaXterm 建立 SSH 会话](#步骤-4--mobaxterm-建立-ssh-会话)
  - [步骤 5 — 首次登录后的身份验证与环境确认](#步骤-5--首次登录后的身份验证与环境确认)
  - [步骤 6 — 文件传输（SFTP）验证](#步骤-6--文件传输sftp验证)
  - [步骤 7 — 基础 ROS2 指令测试](#步骤-7--基础-ros2-指令测试)
- [三、异常处理手册](#三异常处理手册)
  - [A 类：连接阶段问题](#a-类连接阶段问题)
  - [B 类：登录/权限问题](#b-类登录权限问题)
  - [C 类：ROS2 环境问题](#c-类ros2-环境问题)
  - [D 类：指令无响应问题](#d-类指令无响应问题)
- [四、验证标准清单](#四验证标准清单)
- [附录 A：账号信息表](#附录-a账号信息表)
- [附录 B：MobaXterm 会话快速恢复](#附录-bmobaxterm-会话快速恢复)
- [附录 C：命令速查卡](#附录-c命令速查卡)

---

## 一、前置准备

> **开始之前，按此清单逐项打钩，确保没有遗漏。** 任何一项未就绪都请先解决。

### 1.1 学生端（你的 Windows 笔记本）

| # | 项目 | 要求 | 自检方法 |
|---|------|------|----------|
| ☐ | 操作系统 | Windows 10 或 11 | `Win + R` 输入 `winver` 回车 |
| ☐ | MobaXterm | **Personal Edition v25.0+** | 下载地址见步骤 1 |
| ☐ | 以太网口 / WiFi | 能用就行 | 连上网开浏览器能看新闻 |
| ☐ | ping / telnet 工具 | Windows 默认有 ping | 见步骤 3 测试 |
| ☐ | 笔和便签纸 | 记录小车 IP 和密码 | — |

### 1.2 小车端

| # | 项目 | 要求 | 自检方法 |
|---|------|------|----------|
| ☐ | 总电源已开 | 底盘指示灯亮 | 小车底部或侧面看灯 |
| ☐ | 工控机已启动 | 等待约 1 分钟 | 听到工控机风扇转 |
| ☐ | 急停开关已拔出 | 底盘处于"可运行"状态 | 红色大蘑菇头弹起 |
| ☐ | CAN 适配器插好 | USB 线连到工控机 | 设备管理器能看到 |
| ☐ | 激光雷达 / 相机已接 | 数据线缆固定 | 观察接口无松动 |
| ☐ | 小车已固定架空 | 车轮悬空不会真的跑 | 或遥控器拔线禁用 |

### 1.3 网络条件

| # | 项目 | 要求 |
|---|------|------|
| ☐ | 小车 IP 地址已知 | 老师会发放，或通过路由器后台查看 |
| ☐ | 学生机与小车**同一网段** | 例：小车 192.168.1.102，学生机 192.168.1.x |
| ☐ | 无防火墙拦截 SSH 端口 (22) | 公司/校园网可能屏蔽 22 端口 |
| ☐ | 学生机关掉 VPN | VPN 可能改变网段导致连不上 |

### 1.4 账号凭据（课程统一约定）

> ⚠️ 以下为课程标准配置，**以老师当天发放为准**，不同小车 IP 不同，用户名密码统一。

| 项目 | 值 |
|------|-----|
| 小车用户名 | `ubuntu` |
| 默认密码 | `dtmini2024` |
| SSH 端口 | `22`（标准） |
| ROS 2 版本 | `humble` |
| 工作空间 | `~/ros2_ws` |
| Python 版本 | `3.10` |
| 操作系统 | Ubuntu 22.04 LTS |

---

## 二、分步操作

### 步骤 1 — 下载安装 MobaXterm

> MobaXterm 把"终端 + 文件传输 + X11 图形转发 + 多会话管理"合成一个软件，是 Windows 远程连 Linux 小车的最佳工具。

**步骤 1.1 — 下载**

浏览器打开官方下载页（推荐用 Chrome / Edge）：

```
https://mobaxterm.mobatek.net/download.html
```

找到 **Personal Edition (免费版)**，点击下方 **Download Now** 按钮。会跳转到一个新页面开始下载，文件名类似 `MobaXterm_Personal_25.3.exe`（版本号以最新为准）。

> 💡 如果学校网络访问不了 mobatek.net，备选方案：
> - 用手机热点下载
> - 找老师拿 U 盘拷贝
> - 用替代软件：`PuTTY`（终端）+ `WinSCP`（文件传输）组合

**步骤 1.2 — 安装**

双击下载好的 `.exe` 文件，依次：

1. 弹窗选 **Yes**（管理员权限）
2. 语言默认 English → **OK**
3. License Agreement → **I accept the agreement** → **Next**
4. 安装路径默认 `C:\Program Files\MobaXterm` → **Next**
5. Select Additional Tasks → 默认全部勾选 → **Next**
6. 等进度条跑完 → **Finish**

**预期结果：**
桌面上出现 MobaXterm 图标；开始菜单有 MobaXterm 文件夹。

**异常处理：**

| 问题 | 解决 |
|------|------|
| 安装弹窗报"Windows Defender SmartScreen 阻止" | 点"更多信息"→"仍要运行" |
| 装完打不开闪退 | 右键 MobaXterm 图标 → 属性 → 兼容性 → 以管理员身份运行 |

---

### 步骤 2 — 小车开机 & 确认网络

**步骤 2.1 — 给小车上电**

1. 打开小车侧面/底部的**总电源开关**（一般是一个红色拨码开关）
2. 等 **约 60 秒** 让工控机完成启动（风扇会先响一阵然后平稳）
3. 观察底盘上的指示灯：
   - 绿色常亮 → ✅ 工控机 OK
   - 红色闪烁 → ⚠️ 可能 CAN 通信问题
   - 不亮 → ❌ 检查保险丝/电池电压

**步骤 2.2 — 确认小车网络状态（如果有显示器的话）**

> 如果小车连了显示器和键盘，直接在小车上操作；
> 如果没显示器，跳过此步，直接进入步骤 3 在 Windows 上探测。

在小车的 Ubuntu 桌面打开终端（`Ctrl + Alt + T`），运行：

```bash
# 查看所有网络接口的 IP 地址
ip addr show

# 或者
hostname -I
```

**预期结果：**
```
192.168.1.102/24   # 记下来！这是小车 IP
```

如果显示 `127.0.0.1` 或空的：

```bash
# 查看网卡是否正常连接
ip link show

# 如果 eth0/ens33 之类的是 DOWN，尝试拉起
sudo ip link set eth0 up
sudo dhclient eth0     # 从路由器获取 IP
```

**如果小车没有显示器**，也没关系，在步骤 3 会教你"在 Windows 上用命令扫出小车 IP"。

---

### 步骤 3 — 学生机网络连通性检查

**在 Windows 上打开：** `Win + R` → 输入 `cmd` → 回车

**步骤 3.1 — 确认学生机自身 IP**

```cmd
ipconfig
```

在输出中找到"无线局域网适配器 Wi-Fi"或"以太网适配器"那一段：

```
以太网适配器 以太网:
   IPv4 地址 . . . . . . . . . . . . : 192.168.1.50    ← 记下来
   子网掩码  . . . . . . . . . . . . : 255.255.255.0
   默认网关 . . . . . . . . . . . . : 192.168.1.1
```

> ⚠️ **关键检查：** 学生机 IP 和小车 IP 的前三段必须完全一样！
> - 学生机 `192.168.1.50` → 小车必须是 `192.168.1.xxx` ✅
> - 学生机 `192.168.1.50` → 小车 `192.168.36.xxx` ❌ 网段不同！
>
> **网段不同怎么办？** 找网管员把小车 IP 改掉，或者把你的电脑切到同一 WiFi。

**步骤 3.2 — Ping 小车（确认能通）**

```cmd
ping 192.168.1.102
```

（把 `192.168.1.102` 换成你实际的小车 IP）

**预期结果（成功）：**
```
正在 Ping 192.168.1.102 具有 32 字节的数据:
来自 192.168.1.102 的回复: 字节=32 时间<1ms TTL=64
来自 192.168.1.102 的回复: 字节=32 时间<1ms TTL=64
来自 192.168.1.102 的回复: 字节=32 时间<1ms TTL=64

192.168.1.102 的 Ping 统计信息:
    数据包: 已发送 = 4，已接收 = 4，丢失 = 0%
最接近的往返时间(以毫秒为单位):
    最小值 = 0ms，最大值 = 1ms，平均值 = 0ms
```

**❌ 失败情况 1：请求超时**
```
请求超时。
请求超时。
请求超时。
```
→ **排查顺序：**
1. 小车电源开了没？LED 亮不亮？
2. 网线/WiFi 连好了没？
3. 学生机和小车在同一网段？
4. 路由器后台有没有看到小车的 MAC 地址？

**❌ 失败情况 2：找不到主机**
```
Ping 请求找不到主机 192.168.1.102。请检查该名称，然后重试。
```
→ IP 地址写错了，回到步骤 2 或问老师拿正确 IP。

**步骤 3.3 — 端口 22 探测（SSH 服务是否开启）**

```cmd
telnet 192.168.1.102 22
```

**预期结果（成功）：**
窗口变全黑或者显示 `SSH-2.0-OpenSSH_8.9` 之类的文字。
按 `Ctrl + ]` 然后输入 `quit` 退出 telnet。

**❌ 失败情况：**
```
正在连接到 192.168.1.102...无法打开到主机的连接。
```
→ 小车 SSH 服务可能没启动。换个方式再试（见步骤 4 用 MobaXterm 直接连，它的错误提示更友好）。

---

### 步骤 4 — MobaXterm 建立 SSH 会话

**步骤 4.1 — 打开 MobaXterm，建立第一个会话**

1. **双击桌面上的 MobaXterm 图标**
2. 左侧栏 Sessions 区域 → **点击 Session 按钮**（左上角第一个图标）
3. 在弹出的窗口选择 **SSH**（蓝色的那个图标，不是 Telnet、Serial、RDP）

**步骤 4.2 — 填写连接参数**

按以下表格填写：

| 参数 | 填写内容 | 说明 |
|------|----------|------|
| **Remote host** | `192.168.1.102` | 小车 IP（换成你的实际值） |
| **Specify username** | ✅ 勾选 | 显示用户名输入框 |
| **Username** | `ubuntu` | 标准用户名 |
| **Port** | `22` | SSH 标准端口，一般不用改 |
| **X11-Forwarding** | ✅ 勾选 | 让小车的 GUI 程序显示在你 Windows 上 |
| **Compression** | ❌ 不勾 | 局域网不需要压缩 |
| **Advanced SSH settings** | ⚙️ 点开看一下，不用改 | Keepalive 默认 60s 就够用 |

**步骤 4.3 — 保存会话（强烈推荐）**

为了下次一键连接，在 SSH 窗口里点 **Bookmarks** 标签页：

```
Name:       DT-mini #1
Folder:     ROS小车  (可以新建文件夹分组)
Description: 煜禾森 DT-mini 1号车 Ubuntu22.04 Humble
```

点 **OK** 保存。以后每次连接直接双击书签就行，不用再填。

**步骤 4.4 — 点击 OK 开始连接**

点窗口最下方的 **OK** 按钮。

MobaXterm 会弹出一个黑色终端窗口，第一次连接时会提示：

```
Are you sure you want to continue connecting (yes/no/[fingerprint])?
```

这是问你"确定要连这台电脑吗？它的指纹是 xx:xx:xx"，防止中间人攻击。因为是课堂环境，**直接输入 `yes` 回车**。

**步骤 4.5 — 输入密码**

接下来会提示：

```
qg@192.168.1.102's password:
```

输入密码：`dtmini2024`（标准密码，以老师为准）

> 💡 **注意：Linux 终端输入密码时不显示任何字符（没有星号、没有下划线、什么都没有），这是正常的！** 盲输完直接回车。

**预期结果（连接成功）：**
```
Welcome to Ubuntu 22.04.4 LTS (GNU/Linux 5.15.0-x86_64 x86_64)

 * Documentation:  https://help.ubuntu.com
 * Management:     https://landscape.canonical.com
 * Support:        https://ubuntu.com/advantage

qg@dt-mini:~$ _
```

✅ 看到 `qg@dt-mini:~$` 提示符，说明连接成功！

**❌ 常见报错：**

| 报错信息 | 原因 | 解决 |
|----------|------|------|
| `Connection refused` | 小车 SSH 服务没启动 | 用 `ssh qg@localhost` 在小车本机测试；或老师修 |
| `Operation timed out` | 网络不通 | 回到步骤 3 继续排查 |
| `Permission denied` | 用户名/密码错 | 确认用户名是 `ubuntu`，密码注意大小写 |
| `Connection reset by peer` | 网络不稳定 | 换网线；关闭 VPN |
| `Host key verification failed` | 小车换了指纹 | 终端里运行 `ssh-keygen -R 192.168.1.102`，然后重连输入 yes |

---

### 步骤 5 — 首次登录后的身份验证与环境确认

**SSH 会话连上后，逐条执行下面的命令。每执行一条，观察输出是否正常。**

**步骤 5.1 — 确认当前是谁、在哪**

```bash
whoami
```
预期：`ubuntu`

```bash
hostname
```
预期：`dt-mini`（或类似小车编号）

```bash
pwd
```
预期：`/home/ubuntu`

```bash
uname -a
```
预期：`Linux dt-mini 5.15.0-... Ubuntu 22.04...`

**步骤 5.2 — 确认 Ubuntu 版本**

```bash
lsb_release -a
```
预期：
```
Distributor ID: Ubuntu
Description:    Ubuntu 22.04.4 LTS
Release:        22.04
Codename:       jammy
```

**步骤 5.3 — 确认 ROS 2 环境**

```bash
ls /opt/ros/
```
预期：`humble`

```bash
source /opt/ros/humble/setup.bash
echo $ROS_DISTRO
```
预期：`humble`

```bash
ros2 --version
```
预期：`ROS 2 humble, ...`

**如果上面 source 命令报"找不到"或 ROS_DISTRO 为空**，修复一下 `.bashrc`：

```bash
echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
echo "source ~/ros2_ws/install/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

**步骤 5.4 — 确认工作空间存在**

```bash
ls ~/ros2_ws/
```

预期：
```
build  install  log  src
```

如果显示 `No such file or directory`，说明还没克隆项目，参考 README.md 里的 `git clone` 步骤补齐。

```bash
ls ~/ros2_ws/src/
```

预期能看到 `yhs_can_interfaces/`、`yhs_can_control/` 等包目录。

**步骤 5.5 — 确认 Python 版本**

```bash
python3 --version
```
预期：`Python 3.10.x`

---

### 步骤 6 — 文件传输（SFTP）验证

MobaXterm 有一个超好用的功能：**左边面板自动显示远程文件系统**，可以像 Windows 资源管理器一样拖拽文件。

**步骤 6.1 — 查看远程文件**

MobaXterm 界面左侧自动出现一个文件树，显示 `192.168.1.102 /home/ubuntu` 下的所有文件。展开就能浏览。

**步骤 6.2 — 从小车下载文件到你的电脑**

1. 左侧文件树里找到 `~/ros2_ws/src/yhs_can_control/` 目录
2. 看到 `package.xml` 或 `launch/` 子目录
3. **右键** → **Download**（或直接拖拽到你 Windows 的文件夹）
4. 选择本地保存位置

成功后 Windows 下载目录里就有这个文件了。

**步骤 6.3 — 从电脑上传文件到小车**

1. 找到你 Windows 上的某个文件（比如一个测试用的 `.launch.py`）
2. **拖入** 左侧文件树的 `~/ros2_ws/src/` 目录
3. 等待传输完成（右下角会显示进度）

> 💡 **也可以用 SCP 命令行方式：**
> ```bash
> scp C:\Users\你\Desktop\test.launch.py qg@192.168.1.102:~/ros2_ws/src/
> ```
> 但 MobaXterm 左侧面板拖拽更直观，推荐新手用。

**步骤 6.4 — 终端里用命令验证文件确实传过去了**

```bash
ls -lh ~/ros2_ws/src/
```

预期：能看到刚上传的文件出现在列表里。

---

### 步骤 7 — 基础 ROS2 指令测试

> 这是"小车大脑活着"的最终验证。执行前确保相机/雷达等硬件都接好了。

**步骤 7.1 — 确保 ROS2 环境已加载**

```bash
source /opt/ros/humble/setup.bash
source ~/ros2_ws/install/setup.bash
```

**步骤 7.2 — 查看当前有哪些节点在运行**

```bash
ros2 node list
```

**预期结果分两种情况：**

- **情况 A（第一次连，什么都没启动）：** 输出空
- **情况 B（已经有东西在跑）：** 显示类似 `/yhs_can_control_node`、`/orbbec_camera` 等节点名

空不是错误，说明还没启动任何程序。下面来启动一个。

**步骤 7.3 — 启动 CAN 底盘控制节点（终端 1）**

```bash
# 终端 1（MobaXterm 里开新标签）
cd ~/ros2_ws
source /opt/ros/humble/setup.bash
source ~/ros2_ws/install/setup.bash

ros2 launch yhs_can_control yhs_can_control.launch.py
```

**预期结果：**
```
[INFO] [launch]: Default logging verbosity is set to INFO
[INFO] [yhs_can_control_node-1]: process started with pid [12345]
[INFO] [yhs_can_control_node]: CAN 初始化成功, 设备 can0
[INFO] [yhs_can_control_node]: 等待控制指令...
```

如果看到 `[ERROR] can0 不存在` → CAN 适配器没接好，先跳步骤 7.4。

**步骤 7.4 — 启动 Orbbec 相机（终端 2）**

在 MobaXterm 顶部菜单 **Session → Multi-Exec → New Session Tab**（或直接点顶部那个 `+` 号）开新 SSH 标签。

```bash
# 终端 2
source /opt/ros/humble/setup.bash
source ~/ros2_ws/install/setup.bash

# 查看你的相机 launch 文件叫什么
ls ~/ros2_ws/src/OrbbecSDK_ROS2/orbbec_camera/launch/

# 假设是 gemini.launch.py，就启动它
ros2 launch orbbec_camera gemini.launch.py
```

**预期结果：**
```
[INFO] [orbbec_camera_node]: 相机初始化完成
[INFO] [orbbec_camera_node]: 发布话题 /camera/color/image_raw
```

如果报 `找不到 orbbec_camera` → 把 OrbbecSDK_ROS2 也编译进工作空间，或跳过这步（看话题时会为空）。

**步骤 7.5 — 查看当前所有话题（终端 3）**

开第三个 SSH 标签：

```bash
# 终端 3
source /opt/ros/humble/setup.bash
source ~/ros2_ws/install/setup.bash

ros2 topic list
```

**预期看到的话题（至少包含这些）：**

```
/camera/color/image_raw
/camera/depth/image_raw
/chassis_info            # 底盘反馈
/ctrl_cmd                # 控制指令
/scan                    # 激光雷达（如果接了）
/tf
/tf_static
/clock
```

如果话题列表是空的，说明节点没启动，回到步骤 7.3/7.4。

**步骤 7.6 — 实时查看一个话题的数据**

```bash
# 看底盘反馈（每秒更新）
ros2 topic echo /chassis_info --once

# 或者看控制指令频率
ros2 topic hz /camera/color/image_raw
```

**预期结果：**
```
---
header:
  stamp:
    sec: 1731900000
    nanosec: 123456789
chassis_speed:
  linear: 0.0
  angular: 0.0
battery_voltage: 24.5
---
```

```
average rate: 29.956 ± 0.522
  min: 28.8, max: 31.0, std dev: 0.522
```

帧率 ~30Hz 说明相机工作正常！

**步骤 7.7 — 发送一条测试指令（⚠️ 确保车轮架空！）**

> 这一步让小车真的动一下。**务必确认：车轮已架空离地，小车前方无遮挡物，遥控器在手里随时能按急停！**

```bash
# 先查看 CtrlCmd 消息格式
ros2 interface show yhs_can_interfaces/msg/CtrlCmd
```

输出里会告诉你字段名和类型。然后发一条停止指令：

```bash
# 发停车指令（最安全）
ros2 topic pub --once /ctrl_cmd yhs_can_interfaces/msg/CtrlCmd \
  "{ctrl_cmd_gear: 1, ctrl_cmd_linear: 0.0, ctrl_cmd_angular: 0.0}"
```

**预期结果：**
```
publisher: beginning loop
publishing #1: yhs_can_interfaces/msg/CtrlCmd: '{ctrl_cmd_gear: 1, ctrl_cmd_linear: 0.0, ctrl_cmd_angular: 0.0}'

publisher: done
```

如果小车没异常响应 → ✅ CAN 通路正常、底盘能接收指令。

**可以让小车低速前进 3 秒（测试用，必须架空！）：**

```bash
ros2 topic pub -r 10 /ctrl_cmd yhs_can_interfaces/msg/CtrlCmd \
  "{ctrl_cmd_gear: 1, ctrl_cmd_linear: 0.1, ctrl_cmd_angular: 0.0}" -n 30
```

> 这条命令：以 0.1 m/s 的低速持续发 30 次，每秒 10 次 → 约 3 秒内发出让小车缓慢前进的指令。

**步骤 7.8 — 断开连接**

测试完毕，每个终端按 `Ctrl + C` 停掉各自的节点，然后在 MobaXterm 窗口里输入：

```bash
exit
```

窗口关闭即断开。下次再连，直接双击书签即可。

---

## 三、异常处理手册

### A 类：连接阶段问题

| 编号 | 现象 | 最可能原因 | 排查顺序 | 最终解决 |
|------|------|-----------|----------|---------|
| A1 | `Connection refused` | 小车 sshd 没启 | ① 小车有显示器时本机 `sudo service ssh status` ② 没显示器用 USB 串口查 | `sudo systemctl enable ssh` |
| A2 | `Connection timed out` | 网络不通 | ① ping 网关 ② ping 小车 ③ 检查防火墙 | 同一 WiFi；关校园网 VPN |
| A3 | `Network is unreachable` | 学生机没网 | ipconfig 看自己 IP 是不是 0.0.0.0 | 重新连 WiFi/插网线 |
| A4 | 连上了但很快断开 | SSH keepalive 没开 | MobaXterm → 会话属性 → Advanced SSH → Keepalive 60s | 下次连接时启用 |
| A5 | 多台小车同时在线，连错 IP | 没看清 IP | `arp -a` 查看 MAC 地址对比 | 每台小车贴 MAC 标签 |

### B 类：登录/权限问题

| 编号 | 现象 | 最可能原因 | 排查顺序 | 最终解决 |
|------|------|-----------|----------|---------|
| B1 | `Permission denied`（反复 3 次） | 密码错 | ① 大写锁 ② 键盘布局 ③ 确认是不是 dtmini2024 | 问老师拿新密码 |
| B2 | `Permission denied`（只 1 次）然后成功 | SSH 指纹缓存 | 正常，不是问题 | — |
| B3 | `sudo: command not found` | 路径/权限 | 直接 `sudo ls` 再看 | root 可能禁用了 sudo，用 `su -` |
| B4 | `User is not in the sudoers file` | qg 没加 sudo 组 | 首次部署可能漏了 | 用 su 切换 root：`su -`，密码 root 或 dtmini2024 |
| B5 | 文件上传失败 `Permission denied` | 目标目录权限不够 | `ls -ld 目标目录` 看权限 | `chmod -R u+w ~/ros2_ws` |
| B6 | 文件上传后找不到 | 传到了错误目录 | `find ~ -name "文件名" 2>/dev/null` | 记好路径再传 |

### C 类：ROS2 环境问题

| 编号 | 现象 | 最可能原因 | 排查顺序 | 最终解决 |
|------|------|-----------|----------|---------|
| C1 | `ros2: command not found` | 没 source | ① 当前终端直接 `which ros2` ② echo $ROS_DISTRO | 永久化：写入 .bashrc |
| C2 | `No such file or directory` (ros2_ws) | 工作空间未克隆 | `ls ~/ | grep ros2` | 按 README.md git clone |
| C3 | `PackageNotFoundError` 编译时报依赖包缺 | 缺第三方包 | `rosdep install --from-paths src --ignore-src -r -y` | 或手动 apt install |
| C4 | `colcon: command not found` | colcon 未装 | `sudo apt install python3-colcon-common-extensions` | |
| C5 | `ModuleNotFoundError: <包名>` | Python 库未装 | `pip3 install <包名>` | 如果小车无外网，需在 Windows 上 `pip3 download <包名>` 后 SFTP 上传，再 `pip3 install --no-index --find-links=. <包名>` |
| C6 | launch 文件路径不存在 | colcon build 没跑过 | `cd ~/ros2_ws && colcon build && source install/setup.bash` | |

### D 类：指令无响应问题

| 编号 | 现象 | 最可能原因 | 排查顺序 | 最终解决 |
|------|------|-----------|----------|---------|
| D1 | ROS2 话题有数据但 CAN 发不动 | can0 没起来 | `ip link show can0` | `sudo ip link set can0 up type can bitrate 500000` |
| D2 | 相机话题空/只有 `/camera/parameter_events` | 相机没接好/驱动没启 | `dmesg \| tail` 看有没有相机设备 | 重新插 USB，换口 |
| D3 | 雷达话题 `/scan` 空 | 雷达没开/串口线松 | `ls /dev/ttyUSB*` | 检查波特率 512000 |
| D4 | 发 `/ctrl_cmd` 后底盘不动 | 齿轮没闭合/急停 | ① 遥控器试下能不能手动动 ② 急停弹起 | 老师修硬件 |
| D5 | `ros2 topic hz` 显示 0Hz | 发布者节点挂了 | ① 看这个节点是否还在 ② 重启节点 | Ctrl+C 后再起 |
| D6 | 节点启动后立刻崩溃 | 缺依赖/冲突 | 看终端日志最后 20 行 | Google 报错关键字 |

---

## 四、验证标准清单

> ✅ 全部打钩 = 你的连接/部署/测试流程合格，可以进入下一节。

### 模块 A — 连接验证（进入第二步前必须全部完成）

| # | 验证项 | 合格标准 | 自检方法 |
|---|--------|----------|----------|
| ✅ A1 | ping 小车 IP | 4 包 0 丢包 | `ping <IP>` |
| ✅ A2 | telnet 22 端口 | 能连上 | `telnet <IP> 22` |
| ✅ A3 | MobaXterm SSH 登录 | 出现 `qg@dt-mini:~$` | 看终端提示符 |
| ✅ A4 | 文件上传下载 | 一个文件成功往返 | 左侧面板拖拽测试 |
| ✅ A5 | `whoami` | 输出 `ubuntu` | 终端执行 |
| ✅ A6 | `lsb_release -a` | 22.04 LTS | 终端执行 |

### 模块 B — ROS2 环境验证（进入第三步前）

| # | 验证项 | 合格标准 | 自检方法 |
|---|--------|----------|----------|
| ✅ B1 | `echo $ROS_DISTRO` | `humble` | 终端执行 |
| ✅ B2 | `ros2 --version` | 无报错 | 终端执行 |
| ✅ B3 | `ls ~/ros2_ws/` | 有 src/install/build | 终端执行 |
| ✅ B4 | `ls ~/ros2_ws/src/` | 能看到 yhs 系列包 | 终端执行 |

### 模块 C — 节点与话题验证

| # | 验证项 | 合格标准 | 自检方法 |
|---|--------|----------|----------|
| ✅ C1 | CAN 节点启动 | 日志显示等待指令 | `ros2 launch yhs_can_control yhs_can_control.launch.py` |
| ✅ C2 | 相机话题存在 | `/camera/color/image_raw` 在 topic list 里 | `ros2 topic list` |
| ✅ C3 | 相机帧率 25-35Hz | 约 30Hz | `ros2 topic hz /camera/color/image_raw` |
| ✅ C4 | 发停车指令 | 底盘没有明显错误响应 | `ros2 topic pub --once /ctrl_cmd ...` |

### 模块 D — 性能验证（进阶）

| # | 验证项 | 合格标准 | 自检方法 |
|---|--------|----------|----------|
| ✅ D1 | SSH 延迟 | `< 2ms`（局域网） | MobaXterm 会话属性里看 RTT |
| ✅ D2 | SFTP 传输速度 | `> 5MB/s` | 传一个 10MB 文件计时 |
| ✅ D3 | 小车电源电压 | `> 23V` | `ros2 topic echo /chassis_info` |
| ✅ D4 | 系统负载 | CPU < 60% | `top -bn1 | head -20` |

---

## 附录 A：账号信息表

> 每次使用前填好、贴在白板上，方便所有人查看。**每次课后收回——安全！**

| 小车编号 | IP 地址 | 用户名 | 密码 | MAC 地址 | 备注 |
|----------|---------|--------|------|----------|------|
| DT-mini #1 | 192.168.1.102 | ubuntu | dtmini2024 | AA:BB:CC:DD:EE:01 | 激光雷达已接 |
| DT-mini #2 | 192.168.1.102 | ubuntu | dtmini2024 | AA:BB:CC:DD:EE:02 | Orbbec Gemini2 |
| DT-mini #3 | 192.168.1.102 | ubuntu | dtmini2024 | AA:BB:CC:DD:EE:03 | IMU 未校准 |
| DT-mini #4 | | | | | |
| DT-mini #5 | | | | | |

> ⚠️ 安全提示：
> 1. 课后从 SSH `~/.bash_history` 里清除密码痕迹
> 2. 下节课开始前改一次默认密码（`passwd`）
> 3. 密码不要发到微信群里——当面讲或写在实体纸上

---

## 附录 B：MobaXterm 会话快速恢复

**书签已保存的情况下，下次上课只需：**

1. 插网线 / 连 WiFi
2. 小车开机（等 60s）
3. 打开 MobaXterm
4. **双击左侧书签 `DT-mini #1`**
5. 自动连接 → 输入密码 → 回车
6. 开始干活 🚀

**推荐设置：自动 source .bashrc**

在 MobaXterm 会话属性 → SSH → Shell → 勾选 "Run login shell"，这样连接后 `.bashrc` 会自动执行，ROS2 环境自动加载，不用每次手动 source。

**标签页管理技巧：**
- MobaXterm 顶部可以开多个标签（每个标签都是独立 SSH 会话）
- 左侧 Session 文件夹分组管理多台小车
- 可以同时连 5 台小车统一启动测试脚本

---

## 附录 C：命令速查卡

### Windows 端（cmd/PowerShell）

| 目的 | 命令 |
|------|------|
| 看自己 IP | `ipconfig` |
| ping 小车 | `ping <小车IP>` |
| 测试 22 端口 | `telnet <小车IP> 22` |
| 远程拷贝文件（scp） | `scp local_file qg@<小车IP>:~/remote/path/` |

### 小车端（SSH 终端）

| 目的 | 命令 |
|------|------|
| 加载 ROS2 | `source /opt/ros/humble/setup.bash` |
| 加载工作空间 | `source ~/ros2_ws/install/setup.bash` |
| 编译 | `cd ~/ros2_ws && colcon build` |
| 编译单个包 | `colcon build --packages-select <pkg_name>` |
| 看节点列表 | `ros2 node list` |
| 看话题列表 | `ros2 topic list` |
| 看话题内容 | `ros2 topic echo <topic> --once` |
| 看话题频率 | `ros2 topic hz <topic>` |
| 启动 CAN 控制 | `ros2 launch yhs_can_control yhs_can_control.launch.py` |
| 启动激光雷达 | `ros2 launch nvilidar_ros2 nvilidar.launch.py` |
| 启动 Orbbec 相机 | `ros2 launch orbbec_camera gemini.launch.py` |
| 发一条测试指令 | `ros2 topic pub --once /ctrl_cmd yhs_can_interfaces/msg/CtrlCmd "{ctrl_cmd_gear:1, ctrl_cmd_linear:0.0, ctrl_cmd_angular:0.0}"` |
| 停止所有节点 | `Ctrl + C`（每个终端） |
| 断开 SSH | `exit` |

### 安全/工具命令

| 目的 | 命令 |
|------|------|
| CAN 口状态 | `ip link show can0` |
| 启动 CAN 口 | `sudo ip link set can0 up type can bitrate 500000` |
| 网络接口状态 | `ip link show` |
| 查看本机 IP | `hostname -I` |
| 系统负载 | `top`（q 退出） |
| 磁盘空间 | `df -h` |
| 内存使用 | `free -h` |
| 查看日志 | `dmesg \| tail -50` |
| 硬件重启 | `sudo reboot` |
| 查看电源电压 | `ros2 topic echo /chassis_info --once` |

---

> **操作流程总结：** 开机 → 等 60s → MobaXterm 双击书签 → 输密码 → 确认 prompt → source 环境 → `ros2 node list` → 启动节点 → 验证话题 → 测试指令 → 安全急停
> **记住：** 急停开关、遥控器、架空车轮——每上车前这三件事必须检查。
