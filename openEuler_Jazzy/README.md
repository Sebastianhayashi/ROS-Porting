# 在 openEuler 24.03 上移植 ROS Jazzy 

## 源码管理

这一章节单独探讨在移植过程中所有涉及源码的内容，包括了如何获取源码，源码如何妥善的放置，以及在 gitee 中如何让源码对齐 openeuler-src 的仓库规范。

首先，我们需要为 ROS 单独准备一个 workspace：

```
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src
```
这是参考了[官方的做法](https://docs.ros.org/en/jazzy/Tutorials/Beginner-Client-Libraries/Creating-A-Workspace/Creating-A-Workspace.html)。在这个路径下，src 文件夹是用于后续存放源码。在此基础上，我增加了如 tools 等的文件夹，用于后续的流程的源码管理，详见下文。

### 获取源码

我们这里获取的源码都是来自于 [ros2](https://github.com/ros2/ros2.git) 下的 `ros2.repos` 文件，并且使用官方推荐的 vcstool 去批量获取源码。

将 clone 下来的仓库中的 `ros2.repos` 文件放置在 ~/ros2_ws 路径下，并执行：

```
 vcs import src < ros2.repos
```

vcstool 会自动将里面所有的源码批量的拉取到 src 中，如果因网络问题出现了 E，请重新多尝试几次，或配置好你的网络。

拉取下来的内容可以参考：

```
➜  src tree -L 1
.
├── ament
├── eclipse-cyclonedds
├── eclipse-iceoryx
├── eProsima
├── gazebo-release
├── osrf
├── ros
├── ros2
├── ros-perception
├── ros-planning
├── ros-tooling
└── ros-visualization

13 directories, 0 files
```

> ros2.repos 文件并不是一成不变的，官方一直在[持续的维护](https://docs.ros.org/en/jazzy/Installation/Maintaining-a-Source-Checkout.html#:~:text=there%20may%20have%20been%20changes%20made%20to%20the%20source)所以拉取下来的文件可能跟上面的内容有差异。

### 源码仓库管理



