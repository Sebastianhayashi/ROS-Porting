# openKylin 2.0 SP1 上用 colcon 验证 ROS 2 *Jazzy* Desktop 构建——环境准备与操作手册



## 1. 成果概览（TL;DR）

* 通过 **Python 虚拟环境** + **rosdep**（配合 `ROS_OS_OVERRIDE`）在 openKylin 上完成依赖解决。
* 已验证可构建（x86 + aarch64 + Risc-V）并运行 **turtlesim** 等基础功能；

---

## 2. 适用前提

* 系统：openKylin 2.0 SP1（x86_64）
* 具备 `sudo` 权限、可访问 GitHub 与 APT 源。

---

## 3. 快速开始（可复制执行）

> **说明**：以下默认工作区 `~/ros2_ws`，虚拟环境 `~/ros-venv`。请按需调整用户名/路径。

```bash
# 3.1 系统依赖（基础工具链 + 常见构建依赖）
sudo apt update
sudo apt install -y \
  build-essential cmake git curl gnupg2 lsb-release ninja-build \
  python3 python3-venv python3-pip locales software-properties-common \
  pkg-config zlib1g-dev libzstd-dev libzzip-dev libpugixml-dev \
  libfreetype6-dev libx11-dev libxrandr-dev libxinerama-dev libxcursor-dev libxxf86vm-dev libxmu-dev \
  libgl1-mesa-dev libglu1-mesa-dev libgles2-mesa-dev mesa-common-dev freeglut3-dev \
  libacl1-dev libattr1-dev libopencv-dev \
  libspdlog-dev libfmt-dev libeigen3-dev liborocos-kdl-dev \
  libcurl4-openssl-dev \
  'liblttng-*'  # lttng 跟踪栈

# 3.2 Python 虚拟环境 + 基础 Python 依赖
python3 -m venv ~/ros-venv
source ~/ros-venv/bin/activate
python -m pip install -U pip wheel setuptools
pip install -U colcon-common-extensions vcstool rosdep bloom
# ROS2 构建常用 Python 依赖
pip install -U catkin_pkg rospkg empy lark pyyaml
# 个别包需要的较新 numpy
pip install -U "numpy>=2.0"

# 3.3 初始化 rosdep（用虚拟环境里的 rosdep 执行 init）
sudo "$(command -v rosdep)" init || true
rosdep update
# openKylin 伪装为 Ubuntu 24.04 Noble（解决 rosdep 平台识别）
export ROS_OS_OVERRIDE=ubuntu:24.04:noble

# 3.4 拉取源码（以 Desktop 变体为目标）
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws
# 获取官方 ros2.repos 清单
git clone https://github.com/ros2/ros2.git -b jazzy ~/ros2
vcs import src < ~/ros2/ros2.repos
#（可选）获取变体仓库，便于按变体分组构建
git clone https://github.com/ros2/variants.git -b jazzy ~/variants

# 3.5 选择中间件：使用 CycloneDDS，移除/跳过 Connext
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
# 如仓库中含 connext 源码，先移出 src（否则 colcon 会强制找它）
[ -d src/rmw_connextdds ] && mv src/rmw_connextdds ~

# 3.6 使用 rosdep 安装依赖（跳过在 openKylin 不可用/不需要的键）
rosdep install -y --rosdistro jazzy --from-paths src --ignore-src \
  --skip-keys="\
rmw_connextdds \
rmw_connextdds_common \
rti-connext-dds-6.0.1 \
rosidl_typesupport_connext_c \
rosidl_typesupport_connext_cpp \
connext_cmake_module \
rti_connext_dds_cmake_module \
python3-vcstool \
python3-catkin-pkg-modules \
python3-rosdistro-modules \
python3-mypy \
ros-jazzy-example-interfaces \
ros-jazzy-gz-math-vendor \
ros-jazzy-diagnostic-updater \
ros-jazzy-rqt-action \
ros-jazzy-pcl-msgs \
libpcl-common \
ros-jazzy-resource-retriever \
libpcl-features \
libpcl-io"

# 3.7 首轮构建（先尽量跑通 Desktop，上来就跳过已知问题包）
cd ~/ros2_ws
colcon build \
  --merge-install --symlink-install \
  --parallel-workers "$(nproc)" \
  --cmake-args -G Ninja -DCMAKE_BUILD_TYPE=Release \
               -DPython3_EXECUTABLE="$VIRTUAL_ENV/bin/python" \
               -DPYTHON_EXECUTABLE="$VIRTUAL_ENV/bin/python" \
  --packages-up-to desktop \
  --packages-skip \
    python_orocos_kdl_vendor rmw_connextdds rmw_connextdds_common \
    rviz_ogre_vendor rviz_rendering rviz_common \
    rviz_default_plugins rviz_visual_testing_framework rviz2 \
    mimick_vendor

# 3.8 运行简单验证（示例：turtlesim）
source install/setup.bash
ros2 run turtlesim turtlesim_node  # 另开终端可用 teleop 控制
```

> **说明**：如需进一步收缩/扩展构建范围，可用 `--packages-select`/`--packages-up-to`/`--packages-skip` 灵活控制。

---

## 4. 重要说明与策略

### 4.1 ROS 平台伪装

* 通过 `export ROS_OS_OVERRIDE=ubuntu:24.04:noble` 将 openKylin 映射为 Ubuntu Noble，使 `rosdep` 走 Ubuntu 规则。此法**大体可行**，但可能产生：

  * **t64 命名**：如 `libqt5core5t64` 在本发行版不存在。解决：将此类键加入 `--skip-keys`，并按本地可用包名手动安装或暂时跳过。
  * **版本号差异**：例如 `libpcl-io1.14`，openKylin 可用 `1.13`。构建通常**不受影响**，可手动安装邻近版本并继续。

### 4.2 中间件选择

* 官方 Desktop 变体包含 **Connext (商业)**。本方案统一采用 **开源 CycloneDDS**：

  * 构建前导出 `RMW_IMPLEMENTATION=rmw_cyclonedds_cpp`；
  * **移除 `src/rmw_connextdds`** 或加入 `--packages-skip`，否则 colcon 会报缺少 Connext 相关包。

### 4.3 Python 依赖在虚拟环境内解决

* openKylin APT 源缺失若干 Python 包（如 `catkin_pkg`、`rospkg` 等）。统一在 **virtualenv** 内 `pip install`，并在 CMake 中显式指定：

  * `-DPython3_EXECUTABLE="$VIRTUAL_ENV/bin/python"`
  * `-DPYTHON_EXECUTABLE="$VIRTUAL_ENV/bin/python"`

### 4.4 图形/调试依赖与跳过策略

* 图形栈与跟踪栈建议预装：OpenGL/GLU/GLES、X11 头、Freetype、`libopencv-dev`、`liblttng-*` 等。
* **RViz** 一组包（`rviz_*`）在图形依赖不完整时易失败，先 **packages-skip** 跳过，后续再补齐。

---

## 5. 常见问题与解决

### 5.1 `iceoryx_hoofs` 缺少 `sys/acl.h`

* 现象：

  ```
  fatal error: sys/acl.h: 没有那个文件或目录
  ```
* 处理：

  ```bash
  sudo apt install -y libacl1-dev libattr1-dev
  ```

### 5.2 RViz 依赖补齐困难

* 可先跳过以下包：

  ```text
  rviz_ogre_vendor rviz_rendering rviz_common
  rviz_default_plugins rviz_visual_testing_framework rviz2
  ```
* 后续若要启用，再补齐 OGRE/GL/X11 相关依赖并去掉 skip。

### 5.3 LTTng/跟踪依赖缺失

```bash
sudo apt install -y 'liblttng-*'
```

### 5.4 `mimick_vendor`/`python_orocos_kdl_vendor` 等个别包构建失败

* 暂以 `--packages-skip` 跳过；
* `python_orocos_kdl_vendor` 疑似与系统 `pybind11` 头的接口兼容性相关，待后续单独排障。

### 5.5 PCL 版本号不匹配

* 例如 `libpcl-io1.14` 不存在，但 `1.13` 存在：

  ```bash
  apt search libpcl-io   # 确认本地可用版本
  sudo apt install -y libpcl-io1.13 libpcl-common1.13 || true
  ```
* 构建通常不受小版本差异影响。

### 5.6 通用补包清单（遇错再补）

```bash
sudo apt update
sudo apt install -y \
  libacl1-dev libattr1-dev \
  libopencv-dev \
  libspdlog-dev libfmt-dev libeigen3-dev liborocos-kdl-dev \
  libcurl4-openssl-dev \
  'liblttng-*'

# Python 侧
python -m pip install -U "numpy>=2.0"
```

---

## 6. 构建配方示例

### 6.1 “尽量多包但先跑通”的 Desktop 构建

```bash
cd ~/ros2_ws
colcon build \
  --merge-install --symlink-install \
  --parallel-workers "$(nproc)" \
  --cmake-args -G Ninja -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=OFF \
               -DPython3_EXECUTABLE="$VIRTUAL_ENV/bin/python" \
               -DPYTHON_EXECUTABLE="$VIRTUAL_ENV/bin/python" \
  --packages-up-to desktop \
  --packages-skip \
    python_orocos_kdl_vendor rmw_connextdds rmw_connextdds_common \
    rviz_ogre_vendor rviz_rendering rviz_common \
    rviz_default_plugins rviz_visual_testing_framework rviz2 \
    mimick_vendor
```

### 6.2 定点排障构建（示例：只编 `iceoryx_hoofs`）

```bash
colcon build --packages-select iceoryx_hoofs \
  --merge-install \
  --event-handlers console_cohesion+ \
  --cmake-args -G Ninja -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=OFF
```

---

## 7. 验证

```bash
source ~/ros2_ws/install/setup.bash
# 示例：turtlesim
ros2 run turtlesim turtlesim_node
# 另一个终端：
ros2 run turtlesim turtle_teleop_key
```

若能正常弹出界面并可操控，则基础 Desktop 路径已打通。

---

## 8. 附：rosdep 扫描辅助脚本（思路）

> 为减少来回试错时间，可用脚本对 `rosdep install --simulate` 结果做“缺包扫描 + 建议映射（t64 去后缀/相邻版本）”，自动生成：
>
> * 待安装全集、缺失清单（系统库/ROS 二进制分组）、建议替代映射；
> * 后续可生成 overlay YAML 做本地覆盖。

运行流程示例：

```bash
# tools/scan_rosdep.sh
./tools/scan_rosdep.sh
# 输出目录：.rosdep_scan/
# 关注：apt_missing_system.txt（系统层缺包）
```

> 依据扫描结果，增补 `--skip-keys` 或手工 `apt install` 邻近版本。

---

## 9. 交付与后续

* 本手册面向“先把构建打通”的阶段性目标；
* 若需进入 **bloom 打包 / deb 发布**，请基于当前依赖策略继续沉淀 rosdep overlay 与打包规则（特别是 `t64` 命名差异）。


