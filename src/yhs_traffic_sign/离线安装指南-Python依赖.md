# Python 依赖离线安装指南（小车无外网）

> **前提：** 小车工控机（Ubuntu 22.04，Python 3.10）无法访问 pypi.org，DNS 解析失败
> **方案：** 有网的 Windows 电脑下载 .whl 离线包 → MobaXterm SFTP 传到小车 → 本地离线安装

---

## 一、Windows 侧（有网）— 下载离线包

### 前置条件检查

```cmd
REM 确认你的 Windows Python 版本（参考用，步骤 1.3 通用全版本，无需以此分支）
python --version
```

- 下载时如果遇到 `PermissionError: [WinError 32]` → 先执行**步骤 1.0 清理残留**
- 步骤 1.3 为**推荐默认方案**，Python 3.10 ~ 3.14 全适用，不依赖 `--platform` 过滤

### 步骤 1.0 — 清理上次失败的残留（报错时必做！）

```cmd
taskkill /F /IM python.exe 2>nul
taskkill /F /IM python3.exe 2>nul
rmdir /S /Q "%LOCALAPPDATA%\Temp\pip-*" 2>nul
rmdir /S /Q "%LOCALAPPDATA%\pip" 2>nul
echo 残留已清理
```

### 步骤 1.1 — 创建下载目录

```cmd
mkdir D:\offline_pip 2>nul & cd D:\offline_pip
```

### 步骤 1.2 — 检查 pip 是否有 PyPI 访问权限

```cmd
pip3 download --no-deps six -d D:\offline_pip --python-version 3.10
dir D:\offline_pip\six*.whl
```

能看到 `six-*.whl` 说明 PyPI 可达 ✅。如果看不到 → 检查网络/VPN。

### 步骤 1.3 — 推荐下载方案（通用，全 Python 版本适用）⭐

> 核心思路：**全部下载 → 手动筛选**。不依赖 `--platform` 过滤（该参数在不同 Python 版本上行为不稳定），所有 Windows Python 版本（3.10 ~ 3.14+）统一使用此方案。

```cmd
REM ====== 注意：每一行末尾要有 ^ 作为续行符，最后一行不要加 ======
REM ====== 不要加 --platform manylinux2014_x86_64 —— 让 pip 全拉下来 ======

REM 1) torch / torchvision / torchaudio（从 PyTorch 官网 CPU 索引拉，单独处理）
pip3 download torch torchvision torchaudio ^
  --index-url https://download.pytorch.org/whl/cpu ^
  --python-version 3.10 ^
  -d D:\offline_pip

REM 2) ultralytics + OpenCV + numpy + 所有依赖（不用 --platform，全部拉下来）
pip3 download ultralytics opencv-python-headless numpy ^
  pyyaml requests scipy pandas matplotlib tqdm psutil ^
  pillow jinja2 fsspec markupsafe mpmath filelock networkx ^
  charset-normalizer idna certifi urllib3 pyparsing packaging ^
  python-dateutil pytz kiwisolver contourpy fonttools cycler six threadpoolctl ^
  --python-version 3.10 ^
  -d D:\offline_pip

REM ========== 筛选：只留 Linux + Python 3.10 版本（关键！）==========
echo.
echo ====== 筛选 Linux x86_64 + Python 3.10 版本 ======
echo 保留条件：文件名含 cp310 且不含 win
echo 删除条件：含 win_amd64 / win32 / cp311~cp314

REM 3) 删掉 Windows 版本的 .whl（这些小车装不上！）
del /Q D:\offline_pip\*win_amd64*.whl   2>nul
del /Q D:\offline_pip\*win32*.whl       2>nul
del /Q D:\offline_pip\*cp311*win*.whl   2>nul
del /Q D:\offline_pip\*cp312*win*.whl   2>nul
del /Q D:\offline_pip\*cp313*win*.whl   2>nul
del /Q D:\offline_pip\*cp314*win*.whl   2>nul

echo.
echo ====== 筛选后结果 ======
dir D:\offline_pip\*.whl | find /c /v ""
echo 个 .whl 文件准备上传
```

### 步骤 1.3X — 备选：纯过滤方案（Python 3.10 ~ 3.12 可尝试）

如果你的 Windows 是 Python 3.10 ~ 3.12，可以尝试用 `--platform` 一次性过滤到位（省去手动筛选）。**但只要 Python ≥ 3.13 或遇到任何下载异常，请回到步骤 1.3 走推荐方案**：

```cmd
REM 每一行末尾要有 ^，最后一行不要加
REM 注意：Python 3.13+ 的 pip 对 --platform 过滤不稳定，不推荐

REM 1) torch CPU 版（PyTorch 官网）
pip3 download torch torchvision torchaudio ^
  --index-url https://download.pytorch.org/whl/cpu ^
  --python-version 3.10 ^
  --platform manylinux2014_x86_64 ^
  --only-binary=:all: ^
  -d D:\offline_pip

REM 2) ultralytics + OpenCV + numpy
pip3 download ultralytics opencv-python-headless numpy ^
  --python-version 3.10 ^
  --platform manylinux2014_x86_64 ^
  --only-binary=:all: ^
  -d D:\offline_pip

REM 3) 传递依赖
pip3 download pyyaml requests scipy pandas matplotlib tqdm psutil ^
  pillow jinja2 fsspec markupsafe mpmath filelock networkx ^
  charset-normalizer idna certifi urllib3 pyparsing packaging ^
  python-dateutil pytz kiwisolver contourpy fonttools cycler six threadpoolctl ^
  --python-version 3.10 ^
  --platform manylinux2014_x86_64 ^
  --only-binary=:all: ^
  -d D:\offline_pip

REM 4) 校验一下有没有混进 Windows 版本
dir D:\offline_pip\*win*.whl 2>nul
REM 如果上面这条有输出 → 过滤失败，回到步骤 1.3 重下
```

### 步骤 1.3C — torch 手动下载（彻底绕开 pip download）

如果 torch 这步反复出问题，**直接用浏览器下载**：

| 包 | 下载链接（浏览器打开自动下载） |
|----|--------------------------------|
| torch 2.4.1 CPU | `https://download.pytorch.org/whl/cpu/torch-2.4.1%2Bcpu-cp310-cp310-linux_x86_64.whl` |
| torchvision | `https://download.pytorch.org/whl/cpu/torchvision-0.19.1%2Bcpu-cp310-cp310-linux_x86_64.whl` |
| torchaudio | `https://download.pytorch.org/whl/cpu/torchaudio-2.4.1%2Bcpu-cp310-cp310-linux_x86_64.whl` |

下载后手动拖进 `D:\offline_pip\` 即可。

### 步骤 1.3D — 终极兜底：用 Python 3.10 venv 下载（100% 成功）

```cmd
REM 用 py launcher 看系统有没有 Python 3.10+
py -0p

REM 如果有 Python 3.10：
py -3.10 -m venv D:\py310_env
D:\py310_env\Scripts\activate.bat
python -m pip install --upgrade pip

REM 然后按步骤 1.3X 的命令正常下载（这时 --platform 过滤一定生效）
```

### 步骤 1.4 — 检查下载结果

```cmd
echo.
echo ====== 下载清单 ======
dir D:\offline_pip\*.whl /b
echo.
dir D:\offline_pip\*.whl | find /c /v ""
echo 个 .whl 文件
```

**必须确认文件里包含：**
- ✅ `torch-*cp310*linux_x86_64*.whl` 或 `torch-*cp310*manylinux*.whl`
- ✅ `ultralytics-*.whl`
- ✅ `opencv_python_headless-*cp310*.whl`
- ✅ `numpy-*cp310*.whl`

**绝对不能包含：**
- ❌ `torch-*cp314*win_amd64*.whl`（Windows 版，小车装不上）
- ❌ 任何 `*win_amd64*` 或 `*win32*` 后缀的文件

如果某个关键包没下载下来 → 回到步骤 1.3D 用 Python 3.10 venv 重试。

---

## 二、小车侧（无外网）— 离线安装

### 步骤 2.1 — 打开 MobaXterm，连到小车

双击书签 → 输入密码 → 等提示符出现。

### 步骤 2.2 — 在小车上创建离线包目录

```bash
mkdir -p ~/offline_pip
ls -ld ~/offline_pip
```

### 步骤 2.3 — 上传 .whl 文件（用 MobaXterm SFTP）

**方式 A — 左侧面板拖拽（推荐新手）：**

1. MobaXterm 左侧出现文件树 → 找到 `~/offline_pip/`
2. 打开 Windows 资源管理器 → 进入 `D:\offline_pip\`
3. **把所有 .whl 文件全选 → 拖到 MobaXterm 左侧**
4. 等待全部传完（右下角进度条消失）

**方式 B — 命令行 scp（如果有几百个文件）：**

在 Windows cmd 里执行：

```cmd
scp D:\offline_pip\*.whl qg@192.168.1.102:~/offline_pip/
```

### 步骤 2.4 — 验证上传完整

**回到 MobaXterm 终端：**

```bash
ls ~/offline_pip/*.whl | wc -l
```

预期输出和 Windows 上看到的数量一致（40-60 个）。

### 步骤 2.5 — 本地离线安装

```bash
# 先升级一下 pip 的离线安装能力（如果网络偶有恢复的话）
# 如果完全断网，跳过这步

# 离线安装 ultralytics 及其全部依赖
cd ~/offline_pip

pip3 install --no-index --find-links=. \
  ultralytics opencv-python-headless numpy

# 或者一次性安装所有离线包（推荐更稳）
pip3 install --no-index --find-links=. \
  $(ls *.whl | sed 's/\.whl$//' | sed 's/\(.*\)-[0-9].*$/\1/')
# 上面这个 bash 命令会自动把所有 .whl 名提取成包名列表
```

> 💡 **更简单的方式（推荐新手直接用）：**
> ```bash
> cd ~/offline_pip
> pip3 install --no-index --find-links=. -r <(ls *.whl)
> ```
> 不行就最笨的办法：
> ```bash
> for f in *.whl; do pip3 install --no-index "$f"; done
> ```

**预期结果：**
```
Processing /home/qg/offline_pip/ultralytics-8.3.0-py3-none-any.whl
Processing /home/qg/offline_pip/torch-2.4.1+cpu-cp310-cp310-linux_x86_64.whl
...
Successfully installed numpy-1.26.4 opencv_python_headless-4.10.0 ultralytics-8.3.0 torch-2.4.1 ...
```

### 步骤 2.6 — 验证安装成功

```bash
python3 -c "from ultralytics import YOLO; print('✅ ultralytics OK, 版本:', YOLO.__module__)"
python3 -c "import cv2; print('✅ cv2 OK, 版本:', cv2.__version__)"
python3 -c "import numpy as np; print('✅ numpy OK, 版本:', np.__version__)"
```

预期三条都输出 `✅` 开头的成功信息。

---

## 三、如果上面方式还不通的备用方案

### 方案 B — apt 装 OpenCV（最快）

如果只有 OpenCV 装不上，ultralytics 可以暂缓：

```bash
# apt 源走本地缓存/内网镜像通常能通
sudo apt update
sudo apt install -y python3-opencv python3-numpy python3-matplotlib python3-pil \
  python3-yaml python3-scipy python3-pandas
# 这些是 ultralytics 的大部分运行依赖，apt 装完后只需要单独装 ultralytics 本身
```

然后只下载 ultralytics 一个 .whl 传上去：
```bash
pip3 install --no-index --find-links=. ultralytics*.whl
```

### 方案 C — 小车连手机热点临时装

如果能借到一台手机：

1. 手机开热点
2. 小车 WiFi 连上
3. 临时改 DNS：
   ```bash
   sudo nano /etc/resolv.conf
   # 在末尾加一行: nameserver 114.114.114.114
   # 保存退出
   ```
4. 临时改 pip 镜像：
   ```bash
   pip3 config set global.index-url https://mirrors.aliyun.com/pypi/simple/
   pip3 config set global.trusted-host mirrors.aliyun.com
   ```
5. 然后 `pip3 install ultralytics` 应该就能通了
6. 装完再切回原来的网络

### 方案 D — 用 rosdep 拉 apt 依赖

ROS2 的 Python 依赖很多在 apt 里有现成包：

```bash
source /opt/ros/humble/setup.bash
cd ~/ros2_ws
rosdep install --from-paths src --ignore-src -r -y \
  --skip-keys="python3-ultralytics"   # ultralytics apt 没有，跳过

# 手动确认 apt 装了的
dpkg -l | grep python3-opencv
dpkg -l | grep python3-numpy
```

---

## 四、DNS 问题根因诊断（顺便解决外网）

你现在遇到的 `[Errno -3] 域名解析暂时失败` 是 DNS 问题。如果能修好 DNS，直接 `pip3 install` 也行。在小车上执行：

```bash
# 1) 查看当前 DNS
cat /etc/resolv.conf

# 2) 查看 systemd-resolved 真实上游（Ubuntu 22.04 用这个）
resolvectl status

# 3) 测试解析 pypi.org
getent hosts pypi.org
# 如果没输出 = DNS 真挂了

# 4) 修复：手动指定靠谱 DNS
sudo nano /etc/systemd/resolved.conf
# 把 DNS= 那行改成：
# DNS=114.114.114.114 223.5.5.5 8.8.8.8
# FallbackDNS=1.1.1.1 8.8.4.4
# 保存退出

sudo systemctl restart systemd-resolved

# 5) 再测
getent hosts pypi.org
# 能输出 IP = 修好了
```

---

## 五、常见坑

| 坑 | 说明 | 避坑 |
|----|------|------|
| **torch 版本选错** | 下载了 CUDA 版但小车没 GPU → 装得上但跑很慢/报错 | 用 `--index-url https://download.pytorch.org/whl/cpu` 只下 CPU 版 |
| **平台不匹配** | Windows 下载了 win_amd64 的包 → Linux 装不上 | 必须加 `--platform manylinux2014_x86_64` |
| **Python 版本不匹配** | 下载了 cp311 的包 → Python 3.10 装不上 | 必须加 `--python-version 3.10` |
| **源码包没编译** | 下载了 .tar.gz 源码包 → 装的时候需要 gcc | 必须加 `--only-binary=:all:` |
| **依赖不全** | 只下了 ultralytics，它依赖的 torch/numpy 没下 | 步骤 1.3 用 download 会自动拉全 |
| **离线包路径错** | `--find-links=` 指向了不对的目录 | 先 `cd ~/offline_pip` 再装 |

---

## 六、快速命令卡片（打印出来贴墙上）

**Windows 下载所有必要包：**
```cmd
pip3 download ultralytics opencv-python-headless numpy torch torchvision torchaudio ^
  -d D:\offline_pip --python-version 3.10 ^
  --platform manylinux2014_x86_64 --only-binary=:all:
```

**MobaXterm 上传（SFTP 拖拽）：**
```
D:\offline_pip\*.whl    →    ~/offline_pip/
```

**小车离线安装：**
```bash
pip3 install --no-index --find-links=~/offline_pip ultralytics opencv-python-headless numpy
```

**验证：**
```bash
python3 -c "from ultralytics import YOLO; print('OK')"
python3 -c "import cv2; print('OK')"
```
