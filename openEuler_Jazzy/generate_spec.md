# 生成 spec

## 伪装

由于目前的 openeuler 并不是 ROS 的官方工具链支持的发行版，参考官方文档以下内容：

> **ROS_OS_OVERRIDE**
> 
> 
> Format: "OS_NAME:OS_VERSION_STRING:OS_CODENAME" This will force it to detect Ubuntu Bionic:
> 
> ```
> export ROS_OS_OVERRIDE=ubuntu:18.04:bionic
> ```
> 
> If defined, this will override the autodetection of an OS. This can be useful when debugging rosdep dependencies on alien platforms, when platforms are actually very similar and might need be forced, or of course if the autodetection is failing.
> 

可以采用 `export ROS_OS_OVERRIDE=` 指定变量的方式，将 openeuler 伪装成 rhel9：

```
export ROS_OS_OVERRIDE=rhel:9
```

或者也可以在使用 rosdep 的时候单独使用参数指定，如：

```
# rosdep 检查 src 下所有需要的系统依赖
rosdep check --from-paths src --ignore-src --os=rhel:9
```

由于目前 openeuler 24.03 LTS 上存在一些与 rhel9 命名不同的软件包，以下是发现差异的软件包列表，也正是因为这些差异所以导致了依赖的缺失：

```
dnf    python%{python3_pkgversion}-pygraphviz
dnf    opencv-devel
dnf    python3-flake8-builtins
dnf    python3-flake8-comprehensions
dnf    python3-flake8-quotes
dnf    python3-vcstool
```

上面每一个包的详细情况请见附录。

在使用 bloom 生成 spec 的时候可以参考：

```
bloom-generate rosrpm --ros-distro jazzy --os-name rhel --os-version 9
```

## 安装前缀问题

根据 openeuler-src 仓库下的规范来看，统一安装的前缀为：`/opt/ros/%{ros_distro}`。
由于在后续 EUR 中发现无法识别宏，同时


## spec 改名并提取

## 自动化方案

## 附录

软件包情况

> 说明：查看日期为：22/09/25，同时每查看一次也已经执行了 sudo dnf update。

1. dnf    python%{python3_pkgversion}-pygraphviz

```
➜  ~ dnf search graphviz
Last metadata expiration check: 0:00:12 ago on Mon 22 Sep 2025 14:18:16.
===================================================================== Name Exactly Matched: graphviz ======================================================================
graphviz.x86_64 : Graph Visualization Tools
graphviz.src : Graph Visualization Tools
==================================================================== Name & Summary Matched: graphviz =====================================================================
graphviz-debuginfo.x86_64 : Debug information for package graphviz
graphviz-debugsource.x86_64 : Debug sources for package graphviz
graphviz-devel.x86_64 : Development headers and libraries for interfacing to the graphviz
graphviz-docs.x86_64 : Documentation files for graphviz
graphviz-gd.x86_64 : Graphviz plugin for renderers based on gd
graphviz-graphs.x86_64 : Demo graphs for graphviz
graphviz-guile.x86_64 : Guile extension for graphviz
graphviz-java.x86_64 : Java extension for graphviz
graphviz-lua.x86_64 : Lua extension for graphviz
graphviz-ocaml.x86_64 : Ocaml extension for graphviz
graphviz-perl.x86_64 : Perl extension for graphviz
graphviz-python3.x86_64 : Python 3 extension for graphviz
graphviz-ruby.x86_64 : Ruby extension for graphviz
graphviz-tcl.x86_64 : Tcl extension & tools for graphviz
python-graphviz-help.noarch : Development documents and examples for graphviz
python3-graphviz.noarch : Simple Python interface for Graphviz
texlive-graphviz.noarch : Write graphviz (dot+neato) inline in LaTeX documents
texlive-graphviz-doc.noarch : Documentation for graphviz
======================================================================== Summary Matched: graphviz ========================================================================
python-pydot.src : Python interface to Graphviz's Dot
python-pydot-help.noarch : Python interface to Graphviz's Dot
python3-pydot.noarch : Python interface to Graphviz's Dot
python3-pydotplus.noarch : Python interface to Graphviz's Dot language
```
2. dnf    opencv-devel

```
➜  ~ dnf search opencv
Last metadata expiration check: 0:00:45 ago on Mon 22 Sep 2025 14:18:16.
===================================================================== Name & Summary Matched: opencv ======================================================================
opencv.x86_64 : OpenCV means Intel® Open Source Computer Vision Library.
```

3. dnf    python3-flake8-builtins

```
➜  ~ dnf search flake8
Last metadata expiration check: 0:01:09 ago on Mon 22 Sep 2025 14:18:16.
===================================================================== Name & Summary Matched: flake8 ======================================================================
flake8-virtual.noarch : Virtual package to satisfy flake8 dependencies
python-flake8-docstrings-help.noarch : Extension for flake8 which uses pep257 to check docstrings
python-flake8-help.noarch : Development documents and examples for flake8
python-flake8-import-order-help.noarch : Flake8 and pylama plugin that checks the ordering of import statements.
python-flake8-logging-format-help.noarch : Flake8 extension to validate (lack of) logging format strings
python3-flake8-docstrings.noarch : Extension for flake8 which uses pep257 to check docstrings
python3-flake8-import-order.noarch : Flake8 and pylama plugin that checks the ordering of import statements.
python3-flake8-logging-format.noarch : Flake8 extension to validate (lack of) logging format strings
ros-jazzy-ament-cmake-flake8.x86_64 : ROS ament_cmake_flake8 package
ros-jazzy-ament-flake8.x86_64 : ROS ament_flake8 package
========================================================================== Name Matched: flake8 ===========================================================================
python-flake8.src : the modular source code checker: pep8 pyflakes and co
python3-flake8.noarch : the modular source code checker: pep8 pyflakes and co
```

4. dnf    python3-flake8-comprehensions

同 3

5. dnf    python3-flake8-quotes

同3

6. dnf    python3-vcstool

```
➜  ~ dnf search vcstool
Last metadata expiration check: 0:01:52 ago on Mon 22 Sep 2025 14:18:16.
====================================================================== Name Exactly Matched: vcstool ======================================================================
vcstool.noarch : A command-line tool to manage multiple repositories
➜  ~
```