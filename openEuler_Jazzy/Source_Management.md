# 源码管理

Created: September 27, 2025 2:03 PM

在本章节中，我们做的事情主要是将源码在本地进行 openEuler 社区规范化以及批量推送云端仓库（这里以 gitee 为主）做源码的管理，不涉及 spec 生成以及软件包构建。

主要做的事情：

- 拉取源码
- 版本锁定以及 orig 归档
- yaml 元数据
- README 统一
- 本地仓库以及推送
- 自动化
- vendor 包
- 构建平台对接

## 拉取源码

ROS 官方提供了不同发行版的软件包清单，这些清单皆为 .repos 文件，可以使用 vcstool 进行批量拉取。

源码拉取方式：

```jsx
git clone https://github.com/ros2/ros2.git -b jazzy
```

由于 ROS 官方并没有固定源码清单版本，这是一个动态的清单，所以我单独将我拉取下来的源码进行了归档，请见这里（todo）（ros2.repos 清单链接），拉取源码：

```jsx
 vcs import src < ros2.repos
```

拉取下来的源码参考：

```jsx
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

这时候会出现的问题是官方源码中会包含 vendor 包，而大多数的 vendor 包，如 tinyxml_vendor 是系统中已经存在的软件包，这是违反 openEuler 社区规范做法的，优先使用 openEuler 中系统已经存在的软件包。

## orig 归档

根据 openeuler-src 下的规范，这里以 [ament_index]([https://gitee.com/src-openeuler/ament_index.git](https://gitee.com/src-openeuler/ament_index.git)) 为例：

`ros-humble-ament-index-cpp_1.4.0.orig.tar.gz` ，命名规范为：`ros-<distro>-<pkg_name>_<version>.orig.tar.gz`。同样我们在进行 orig 归档的时候也需要遵守这个规范。

里面包含的内容为：

```jsx
➜  ros-humble-ament-index-cpp-1.4.0 ls
CHANGELOG.rst          Doxyfile               package.xml            README.md              test
CMakeLists.txt         include                QUALITY_DECLARATION.md src
```

同样的，首先解压后的文件夹名称为：`ros-<dirstro>-<pkg_name>-<version>`，里面仅包括 ROS 源码。

## yaml 元数据

每一个仓库都需要有一个 .yaml 文件用于溯源，我们同样参考 openeuler-src 下的 .yaml：

```jsx
---
version_control: git
src_repo: https://github.com/ament/ament_index.git
tag_prefix: 1.1.0
separator: "."
```

## README 统一

同样参考 openeuler-src 下的 REAMDE：

```jsx
# ament_index

#### 介绍

ament resource index 的API

#### 软件架构
软件架构说明

https://github.com/ament/ament_index.git

文件内容:
```
├── ament_index_cpp
│   ├── CMakeLists.txt
│   ├── Doxyfile
│   ├── include
│   ├── package.xml
│   ├── QUALITY_DECLARATION.md
│   ├── README.md
│   ├── src
│   └── test
├── ament_index_python
│   ├── ament_index_python
│   ├── colcon.pkg
│   ├── completion
│   ├── docs
│   ├── package.xml
│   ├── QUALITY_DECLARATION.md
│   ├── README.md
│   ├── resource
│   ├── setup.py
│   └── test
├── CONTRIBUTING.md
├── LICENSE
├── pytest.ini
└── README.md
```
`ament_index_cpp` ament resource index cpp的api接口.
`ament_index_python` access the ament resource index　python的api接口.

#### 安装教程

1. 下载rpm包

aarch64:

```
wget https://117.78.1.88/build/home:Chenjy3_22.03/openEuler_22.03_LTS_standard_aarch64/aarch64/ament_index/ros-foxy-ros-ament_index-1.1.0-2.oe2203.aarch64.rpm
```

x86_64:

```
wget https://117.78.1.88/build/home:Chenjy3_22.03/openEuler_22.03_LTS_standard_x86_64/x86_64/ament_index/ros-foxy-ros-ament_index-1.1.0-2.oe2203.x86_64.rpm
```

2. 安装rpm包

aarch64:

```
sudo rpm -ivh --nodeps --force ros-foxy-ros-ament_index-1.1.0-2.oe2203.aarch64.rpm
```

x86_64:

```
sudo rpm -ivh --nodeps --force ros-foxy-ros-ament_index-1.1.0-2.oe2203.x86_64.rpm
```

#### 使用说明

依赖环境安装:

```
sh /opt/ros/foxy/install_dependence.sh
```

安装完成以后，在/opt/ros/foxy/目录下如下输出,则表示安装成功。

输出:

```
./
├── bin
│   ├── ament_clang_format
│   ├── ament_clang_tidy
│   ├── ament_copyright
│   ├── ament_cppcheck
│   ├── ament_cpplint
│   ├── ament_flake8
│   ├── ament_index
│   ├── ament_lint_cmake
│   ├── ament_mypy
│   ├── ament_pclint
│   ├── ament_pep257
│   ├── ament_pycodestyle
│   ├── ament_pyflakes
│   ├── ament_uncrustify
│   └── ament_xmllint
├── COLCON_IGNORE
├── include
│   └── ament_index_cpp
├── lib
│   ├── libament_index_cpp.so
│   └── python3.9
├── local_setup.bash
├── local_setup.ps1
├── local_setup.sh
├── _local_setup_util_ps1.py
├── _local_setup_util_sh.py
├── local_setup.zsh
├── setup.bash
├── setup.ps1
├── setup.sh
├── setup.zsh
└── share
    ├── ament_clang_format
    ├── ament_clang_tidy
    ├── ament_cmake
    ├── ament_cmake_auto
    ├── ament_cmake_clang_format
    ├── ament_cmake_clang_tidy
    ├── ament_cmake_copyright
    ├── ament_cmake_core
    ├── ament_cmake_cppcheck
    ├── ament_cmake_cpplint
    ├── ament_cmake_export_definitions
    ├── ament_cmake_export_dependencies
    ├── ament_cmake_export_include_directories
    ├── ament_cmake_export_interfaces
    ├── ament_cmake_export_libraries
    ├── ament_cmake_export_link_flags
    ├── ament_cmake_export_targets
    ├── ament_cmake_flake8
    ├── ament_cmake_gmock
    ├── ament_cmake_google_benchmark
    ├── ament_cmake_gtest
    ├── ament_cmake_include_directories
    ├── ament_cmake_libraries
    ├── ament_cmake_lint_cmake
    ├── ament_cmake_mypy
    ├── ament_cmake_nose
    ├── ament_cmake_pclint
    ├── ament_cmake_pep257
    ├── ament_cmake_pycodestyle
    ├── ament_cmake_pyflakes
    ├── ament_cmake_pytest
    ├── ament_cmake_python
    ├── ament_cmake_target_dependencies
    ├── ament_cmake_test
    ├── ament_cmake_uncrustify
    ├── ament_cmake_version
    ├── ament_cmake_xmllint
    ├── ament_copyright
    ├── ament_cppcheck
    ├── ament_cpplint
    ├── ament_flake8
    ├── ament_index
    ├── ament_index_cpp
    ├── ament_index_python
    ├── ament_lint
    ├── ament_lint_auto
    ├── ament_lint_cmake
    ├── ament_lint_common
    ├── ament_mypy
    ├── ament_package
    ├── ament_pclint
    ├── ament_pep257
    ├── ament_pycodestyle
    ├── ament_pyflakes
    ├── ament_uncrustify
    ├── ament_xmllint
    └── colcon-core
```
#### 参与贡献

1.  Fork 本仓库
2.  新建 Feat_xxx 分支
3.  提交代码
4.  新建 Pull Request

#### 特技

1.  使用 Readme\_XXX.md 来支持不同的语言，例如 Readme\_en.md, Readme\_zh.md
2.  Gitee 官方博客 [blog.gitee.com](https://blog.gitee.com)
3.  你可以 [https://gitee.com/explore](https://gitee.com/explore) 这个地址来了解 Gitee 上的优秀开源项目
4.  [GVP](https://gitee.com/gvp) 全称是 Gitee 最有价值开源项目，是综合评定出的优秀开源项目
5.  Gitee 官方提供的使用手册 [https://gitee.com/help](https://gitee.com/help)
6.  Gitee 封面人物是一档用来展示 Gitee 会员风采的栏目 [https://gitee.com/gitee-stars/](https://gitee.com/gitee-stars/)

```

## 本地仓库以及推送

我们将源码拉取至 ros2_ws/src 下了之后，在 ros2_ws 下创建一个 repos 的文件夹，用于准备放置本地的仓库。

存放、规范化、推送至 gitee 都将在这个文件夹下发生。

## 自动化

### 准备仓库

这里我使用一个脚本用于将上面的提及的流程进行自动化，该脚本为： prepare_repos.sh（todo），这个脚本做了：

- 扫描 src 下的包（默认排除 *_vendor），为每个包在 repos/<pkg>/ 生成：
- 生成 ros-jazzy-<pkg-with-dashes>_<version>.orig.tar.gz
- 生成 <pkg>.yaml   （上游元数据）
- 生成 [README.md](http://readme.md/)
- 切换分支
- 生成 .orig

这个脚本可以将所有的 ros2_ws/src 中的源码按照上面的要求放置到 ros2_ws/repos 下面准备进一步的推送。

### 批量推送

在仓库准备好了之后，将仓库批量的推送到 ROS-RV 企业空间下，该脚本为：gitee_batch_push.sh。

主要的作用是：将本地仓库上传到企业仓库并且设置为公开。

## vendor 包

todo

## 构建平台对接

### eulermaker

如果使用 eulermaker， 这时候需要让