# 打包流水线

openEuler 上的打包简单而言，在本地生成 orig 压缩包、spec，上传至 gitee 后再导入构建平台。

在这个过程中会有许多的步骤，诸如生成 orig 压缩包，生成 spec，以及给 spec 进行规范化等流程。本文档旨在记录如何自动化完成这些工作，以及如何快速的进行迭代。

## 0 环境准备

首先需要使用 dnf 安装以下基本的软件包，准备好 python 相关环境：

```
sudo dnf install git python3 python3-devel python3-pip
```

接着创建一个虚拟环境，可选但是推荐，这样能够保证流水线的环境干净：

```
python3 -m venv ~/ros_test1
```

source 进去新创建的虚拟环境后，开始准备 ROS 官方工具链：

```
pip install -U rosdistro rosdep bloom vcstool colcon-common-extension
```

这时候我们需要对 openEuler 进行伪装，因为 openEuler 不是 ROS 官方支持的系统，所以需要使用变量将其伪装成 rhel9.

> 以前有同事将 ROS 官方工具链（bloom、rosdep 等）对 openEuler 进行适配，但是这样维护的成本很高，需要人工维护一份依赖映射清单。而实际上将 openEuler 伪装成 rhel9 的话软件包名是一致的，只要少数的例外，会在另一份文档中进行记录。所以这里直接进行伪装即可。

```
export ROS_OS_OVERRIDE=rhel:9
```
初始化 rosdep 并且将其进行更新：

```
sudo $(which rosdep) init

rosdep update
```

> rosdep 是 ROS 官方用于管理依赖相关的包管理器，这里用于将源码中的依赖映射成 rhel9 的依赖并配合 bloom 写入 spec中。

> 这里使用 $(which rosdep) 是因为使用了虚拟环境终端可能会报错找不到 rosdep，所以使用变量是最保险的做法。

接着，我们需要准备一个工作区，我的工作区结构如下：

```
➜  ros_rpm1 tree -L1
.
├── eulermaker_links.txt -> 生成的链接清单（下文会提及）
├── log -> 日志
├── pkg_list.txt -> 源码位置清单
├── repos -> 仓库
├── ros2.repos -> 源码 git 清单
├── scripts -> 工具
└── src -> 源码

5 directories, 3 files
➜  ros_rpm1
```

首先，我们先创建一个工作区，并准备好 src 文件夹：

```
mkdir ros_rpm1

cd ros_rpm1

mkdir src
```

> src 是用来存放从 GitHub 上拉取下来的源码。

接着下载 ros2.repos：

```
wget https://raw.githubusercontent.com/ros2/ros2/refs/heads/jazzy/ros2.repos
```

> ros2.repos 是一份 ROS 软件包源码清单，片段如下：

```
repositories:
  ament/ament_cmake:
    type: git
    url: https://github.com/ament/ament_cmake.git
    version: jazzy
  ament/ament_index:
    type: git
    url: https://github.com/ament/ament_index.git
    version: jazzy
  ament/ament_lint:
    type: git
    url: https://github.com/ament/ament_lint.git
    version: jazzy
  ament/ament_package:
    type: git
    url: https://github.com/ament/ament_package.git
    version: jazzy....
```

使用 vcstool 获取源码：

```
vcs import src < ros2.repos
```

到此为止，前置的环境以及所需的准备都做好了。

## 生成 tarball

### 背景

作为构建软件包最基础的材料也就是源码，这里参考 openeuler-src 下的样式对源码包进行管理。

> 也就是所有的软件包都要压缩成 .orig.tar.gz 的形式进行归档，同时命名规范为：ros-<distro>-<pkg_name>_<version>.orig.tar.gz

我们需要做的是将刚才批量下载到 src 下所有的源码都进行归档打包。

但这时候会面临几个问题：

1. 首先数量多，其次是获取下来的软件包是嵌套好几层的结果。这时候我们需要一个办法能够精确的定位到每一个软件包的位置。

2. 我们需要依照命名规范对生成的 tarball 进行命名

3. 解压出来的顶层文件夹名字需要与 tarball 包一致。

4. 需要有一个地方放。

根据上面的问题，开发了脚本：**gen_tarballs.py**。

### 用法

首先，需要在 ros_rpm1 文件夹下面准备一份源码位置清单，使用 colcon：

```
colcon list > pkg_list.txt
```

colcon list 会列出所有的软件包的位置，然后我们把列出来的内容放进 .txt 作为脚本文件夹参考的位置。

colcon list 片段参考：

```
action_msgs	src/ros2/rcl_interfaces/action_msgs	(ros.ament_cmake)
action_tutorials_cpp	src/ros2/demos/action_tutorials/action_tutorials_cpp	(ros.ament_cmake)
action_tutorials_interfaces	src/ros2/demos/action_tutorials/action_tutorials_interfaces	(ros.ament_cmake)
action_tutorials_py	src/ros2/demos/action_tutorials/action_tutorials_py	(ros.ament_python)
actionlib_msgs	src/ros2/common_interfaces/actionlib_msgs	(ros.ament_cmake)
ament_clang_format	src/ament/ament_lint/ament_clang_format	(ros.ament_python)
...
```

接着将脚本放入 ros_rpm1/scripts 下：

```
➜  ros_rpm1 python3 scripts/gen_tarballs.py \
  --workspace /home/orchid/ros_rpm1 \
  --pkg-list pkg_list.txt \
  --ros-distro jazzy \
```

> 可以根据具体情况灵活调整参数。

脚本会开始批量生成 tarball，并且输出在 ros_rpm1/repos 下。

产物参考：

```
➜  ros_rpm1 ls repos
actionlib_msgs                          rcl_logging_spdlog
action_msgs                             rclpy
action_tutorials_cpp                    rcl_yaml_param_parser
action_tutorials_interfaces             rcpputils
...
```
每一份文件夹中会有独立的源码压缩包，到这为止生成 tarball 阶段就算结束了。

### 脚本额外说明

默认情况下直接使用 git archive 进行归档的话，确实可以排除 `.git` 文件进行压缩，但是如果直接使用的话，在构建过程中会报错说找不到文件夹。原因是解压出来的顶层文件夹名字不对。

> 也就是，比如说 `ros-jazzy-ament-clang-format_0.17.3.orig.tar.gz` 解压出来名字需要是：`ros-jazzy-ament-clang-format_0.17.3`

同时，将生成的 tarball 包放在 repos 下面也是为了后续初始化 git 仓库做准备。现在有了独立的文件夹，后续将 spec 放进去就可以直接批量上传到 gitee 等云端了。

## 生成 spec

### 背景

该阶段使用 bloom 工具来批量生成 spec。生成出来的 spec 会在 src 的源码包下生成一份名为 rpm 的文件夹，然后生成一份 `template.spec`。接着将这一份 `template.spec` 放到 repos 下即可。

### 用法

```
python3 scripts/gen_template_specs.py \
  --workspace /home/orchid/ros_rpm1 \
  --pkg-list pkg_list.txt \
  --ros-distro jazzy \
  --os-name rhel \
  --os-version 9 \
  --timeout-seconds 60 \
  --workers 16
```

> 这里需要注意的是，这里 timeout 的设计是因为有些包可能存在 rosdep 无法解析的情况（即使已经伪装了系统）。所以超过一分钟无响应就跳过，如果不加 timeout 的话，遇到了无法解析的依赖脚本可能会一直卡在那。

脚本执行成功后，你会在 repos 下面的每一个文件夹下面得到 template.spec，那么这阶段就结束了。

> 如果因为意外中断了脚本等情况，无需删掉原来的内容，可以加入 `--force` 参数让脚本默认覆盖已经生成的内容，重新生成一份。

## spec 规范化

### 背景

在上一个阶段中生成的 spec 都是 `template.spec`，也就是照着模版写的 spec，这样的 spec 是无法使用的。

具体会存在下面几个问题：

1. 参考 openeuler-src 已有的 spec，默认都是关闭所有的测试，所以这里我们也需要效仿 openeuler-src 的做法，在 spec 头部加入：
     %bcond_without tests
     %bcond_without weak_deps
     %global debug_package %{nil}
以关闭所有的测试。

2. 将 source 的路径统一改成：Source0: %{name}_%{version}.orig.tar.gz

3. 将 Name 统一为：Name: ros-<ros_distro>-<pkg-name-with-hyphen>

4. 参考 openeuler-src 下 spec 的写法，需要将所有的东西都打包进去 `/opt/ros/jazzy`。这时候需要做两处改动。首先，将 %file 段进行修改。其次，将 cmake 和 python 处进行修改。

5. 将 template.spec 的名字改成：<pkg_name>.spec

### 用法

```
python3 scripts/fix_specs.py \
  --workspace /home/orchid/ros_rpm1 \
  --pkg-list pkg_list.txt \
  --ros-distro jazzy \
```

你能够发现你 repos 下面每一个文件夹下的 spec 文件名字已经被正确修改了，那就说明已经成功给 spec 规范化了。到此为止，本阶段结束。


## 上传 gitee

### 背景

这里默认上传至 gitee 仓库，也可以根据实际情况调整不同的代码管理平台。

需要做的事情就是：

1. 与 gitee 取得通信

2. 仓库本地初始化

3. 批量上传

### 用法

> 这里默认使用 token 进行通信。

请在环境中进行指定：

```
export GITEE_USER=<user_name>
export GITEE_TOKEN=<token>>
```

接着直接运行脚本即可：

```
python3 scripts/upload_to_gitee.py \
  --workspace /home/orchid/ros_rpm1 \
  --pkg-list pkg_list.txt \
  --ros-distro jazzy \
  --branch master \
  --workers 16
```

### 危险操作

这里由于 spec 会出现不同的问题，比如说出现了大量同样的 spec 问题，那么就需要修改 fix_spec 脚本来解决，这时候就需要将上游所有的软件包都删掉。

同时，这是实验用，账号等都是测试实验用的，所以为了快速删除、迭代脚本功能的目的，开发了一键删除的脚本：python3 scripts/gen_template_specs.py。

## 一键流水线

上述的流程可以通过下面的脚本快速的跑完：
