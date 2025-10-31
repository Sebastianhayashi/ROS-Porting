# ROS Jazzy 在 openEuler 24.03（LTS）上的 RPM 构建与提交流程

（阶段工作纪要 & 复现手册）

## TL;DR

* 目标：在 **openEuler 24.03** 上批量构建并发布 **ROS Jazzy** 的 RPM。
* 当前结论：**先生成基线 spec+源码归档并批量上传到云仓（Gitee/GitHub），再在 eulermaker 进行验证**，比在本地 Docker 反复折腾更快更稳，记录问题再迭代修复。
* 已完成：

  1. 标准化工作区与虚拟环境，形成 **可复现** 的 `ros-rpm` 根结构。
  2. `spilt.py`：从 `ros2_ws/src` **按拓扑顺序** 拆出包、抽取版本、生成 `ros-<distro>-<pkg>-<ver>.orig.tar.gz`，可指定 `--up-to desktop` 仅处理 desktop 闭包。
  3. `stage.py`：用 **bloom** 批量生成 **RPM spec** 并**自动修补**（禁测、规范 `Source0`、宏注入、Python 包 noarch 等），内置**超时 30s** 防止卡死。
  4. `upload_repos.py` / `gitee_make_public.py`：一键把 `repos/*` **初始化为独立仓库并推送**到 Gitee/GitHub，同时产出 eulermaker 导入用的清单 YAML。
* 已知情况：eulermaker 存在“**全量构建中断**但**单包构建正常**”的现象（需要进一步复现与沟通）。
* 工作思路：**做减法**——从早期七八个脚本与本地 Docker 方案，收敛到**官方工具链 + 两段式（分离“准备/上传”与“平台构建验证”）**、**可断点**、**可复现**的流水线。

---

## 1. 背景与目标

* 背景：ROS 官方未直接适配 openEuler；打包规则禁止构建时联网；eulermaker/OBS 是公司与社区的**权威构建平台**。
* 目标：在 openEuler 24.03 上**批量**构建 ROS Jazzy，产出标准化的 spec 与源码归档，**优先使用上游包**，必要时允许 vendor，但**不允许构建期拉网**。

---

## 2. 设计原则

1. **可复现**：所有目录、脚本、环境变量固定约定，走一遍即可还原现场。
2. **可断点**：分阶段产物入库（`repos/*`），失败可局部重跑，不推倒重来。
3. **最小化定制**：**不 fork bloom**，只做**后处理修补**；降低后续升级成本。
4. **先验证再规范**：先以“bloom 原生模板 + 轻量修补”形成**对照基线**，再按报错迭代。
5. **平台优先**：本地 Docker 仅作为实验；实际以 eulermaker 为准做“真构建”。

---

## 3. 目录与环境约定（固定）

**根路径**：`~/ros-rpm`（默认）

```
ros-rpm/
├─ ros2_ws/           # 官方工作区
│  └─ src/            # vcs 拉下来的源码
├─ repos/             # 每个 ROS 包一个仓库（spec + .orig.tar.gz）
│  └─ <pkg>/
│     ├─ original/    # 拆出的干净源码（临时/可清理）
│     ├─ ros-...orig.tar.gz
│     └─ ros-... .spec
├─ rpmbuild/          # 观察/调试用（SOURCES/SPECS 等）
├─ logs/
│  ├─ bloom/          # stage（bloom 生成与修补）的日志
│  └─ build/          # （曾用于本地容器构建，现阶段可忽略）
├─ state/             # 内部状态文件（拓扑、映射等）
├─ rosrpm.py          # deploy：环境/目录初始化
├─ spilt.py           # 拆包 & 生成 .orig
├─ stage.py           # 批量 bloom + 规范化修补 spec
├─ upload_repos.py    # 批量创建远端仓库并 push（产出 YAML）
└─ gitee_make_public.py  # 批量切仓库为 public
```

> 注：过去曾出现 `log/` 与 `logs/` 并存，**统一以后使用 `logs/`**。

---

## 4. 复现流程（一步步按下面跑）

### 4.1 环境与工作区初始化（`rosrpm.py deploy`）

* 作用：

  * 创建 `~/ros-rpm` 根结构、`python venv`（修正 `setuptools<80` 与 colcon 兼容）
  * `git clone https://github.com/ros2/ros2 -b jazzy`
  * `vcs import src < ros2/ros2.repos`
  * 可选生成 openEuler 24.03 基础 Dockerfile（**目前实际不再使用 Docker 构建**）

**示例：**

```bash
cd ~/ros-rpm
chmod +x rosrpm.py
./rosrpm.py deploy --distro jazzy        # 默认进入已激活 venv 的子 shell
# 若只初始化而不进子 shell： ./rosrpm.py deploy --distro jazzy --no-shell
```

> 关于“虚拟环境前缀不显示”：脚本会为 bash/zsh 创建临时 rc 文件并 `source` 激活；若终端主题覆盖了 `PS1`，显示可能与手动 `source` 略有差异，但 `VIRTUAL_ENV` 已生效。

---

### 4.2 拆包并生成 `.orig.tar.gz`（`spilt.py`）

* 作用：

  * 用 `colcon list --topological-order` 获取拓扑顺序。
  * 解析每个包 `package.xml` 的 `<version>`。
  * 输出形如：`ros-jazzy-<pkg-hyphen>-<version>.orig.tar.gz`，**解包顶层目录同名**。
  * 支持**只处理 desktop 闭包**（或任意 up-to 列表），并默认**排除测试包**。
  * 保留 `repos/<pkg>/original/<pkg>/` 作为 bloom 输入基线（可被 `stage.py` 之后清理）。

**常用：仅生成 desktop 闭包**

```bash
cd ~/ros-rpm
./spilt.py --distro jazzy --up-to desktop --jobs 16
# 或指定工作区： --workspace ~/ros-rpm/ros2_ws
# 如需包含测试包： --include-tests
```

产物：

* 清单：`~/.local/state/rosrpm/order.ros`
* 映射：`~/.local/state/rosrpm/order.map`
* 每包：`repos/<pkg>/{original/, ros-jazzy-<pkg>-<ver>.orig.tar.gz}`

---

### 4.3 批量生成 & 规范化 spec（`stage.py`）

* 作用（在每个 `repos/<pkg>`）：

  * `bloom-generate rosrpm --ros-distro jazzy --os-name rhel --os-version 9`
    （用 `ROS_OS_OVERRIDE=rhel:9` 伪装；**30 秒超时**避免卡死，如 `rti-connext-dds` 这类未映射中间件）
  * 将 `rpm/template.spec` 拷贝为 `ros-jazzy-<pkg>.spec` 并**“轻量修补”**：

    * **禁用测试**（`%bcond_without tests` + 规范 `%check`，仅在 `with_tests` 时执行）
    * **规范 `Source0`**：`%{name}-%{version}.orig.tar.gz`
    * 注入宏：`__os_install_post` 去掉 bytecompile、`__provides/__requires_exclude_from` 指向 `/opt/ros/jazzy`
    * 纯 Python（`ament_python`）包：`BuildArch: noarch`
    * 确保 `BuildRequires: python%{python3_pkgversion}-devel / -setuptools`
    * 默认将 `repos/<pkg>/original/` **清理**（可参数控制）

**示例：**

```bash
cd ~/ros-rpm
./stage.py --distro jazzy --jobs 16 --timeout 30
# 已有 spec 再跑会做“幂等修补”，不强制覆盖；如需强制：--force
# 如需保留 original：--rm-original=false （或在旧 argparse：去掉 --rm-original 并不要加 --keep-original）
```

---

### 4.4 批量创建远端仓库并推送（`upload_repos.py`）

> 目标：把 `repos/*` 变成独立的 git 仓库并**推到 Gitee/GitHub**；生成 eulermaker 导入用 YAML（**不含凭据**）。

**示例：**

```bash
cd ~/ros-rpm
./upload_repos.py
# 交互式选择 gitee/github、token、命名空间、默认分支（建议 main）
```

输出：

* 每个仓库已推送到远端（公开）。
* `~/ros-rpm/package_repos.yaml`（示例结构）：

```yaml
package_repos:
  - spec_name: ament_package
    spec_url: https://gitee.com/<ns>/ament_package.git
    spec_branch: main
  - spec_name: ament_lint
    spec_url: https://gitee.com/<ns>/ament_lint.git
    spec_branch: main
```

> 如需批量设为公开：`gitee_make_public.py --ns <your_namespace> --token <GITEE_TOKEN>`（支持 `--dry-run`）。

---

## 5. 构建验证策略：**eulermaker 优先**

* 我做过本地 Docker 方案（`oel-base` / `oel-prepped`、`dnf builddep` 预装 BuildRequires、无网 rpmbuild），但在**大批量**场景下：

  * 本地镜像维护成本高；offline 仓库镜像体积巨大；
  * 构建链与 eulermaker/OBS 存在环境偏差；
  * 需要额外实现“依赖闭包 + 断点续跑 + 已构建包反哺 BuildRequires”等机制。
* 结论：**直接把基线 spec+orig 上传到云仓 → 用 eulermaker 做权威验证** 更快更准，记录报错再回改（spec 修补 / 依赖映射 / vendor 归档）。

> 已知：eulermaker 的“全量构建”会出现**中断**，但**单包构建正常**。需要与平台对齐复现与排查。

---

## 6. 关键决策与理由

### 6.1 修改 bloom 模板 vs 事后修补 spec

**选择：事后修补（当前 `stage.py` 做法）。**

* 原因：不 fork/改动 bloom 源码，**降低维护成本**；上游更新不牵连自定义模板。
* 修补是**幂等**、**可审计**、**细粒度**的：禁测、Source0 规范化、宏注入、Python 包 noarch 等都在一处集中管理，diff 透明。

### 6.2 构建平台选型：本地 Docker vs eulermaker

**选择：eulermaker 主导；本地仅作为实验手段。**

* eulermaker/OBS 是**最终发布**的真环境；以平台实测反馈为准更准确。
* 本地离线仓成本极高（单 LTS 仓上百 GB），而平台端已有成熟缓存与调度。

### 6.3 中间件

* 已明确选用 **Fast DDS (fastrtps)** 作为替代（商用中间件不在开放打包路径）。
* **影响范围**：属于**运行/联测阶段**开关，对 RPM 规范化与 spec 生成不构成前置阻塞。

### 6.4 vendor 策略

* 原则：**能用上游包就不用 vendor**；上游没有时才 vendor，**严禁构建期联网下载**。
* 实施：通过 `BuildRequires` 指向上游；如缺失再 vendor，并将第三方源码**一并打入 .orig**。

### 6.5 “伪装 rhel:9” 与依赖映射风险

* 现状：`ROS_OS_OVERRIDE=rhel:9` 让 bloom/rosdep 按 RHEL9 规则生成 spec。
* 风险：openEuler 与 RHEL9 的包名/宏略有差异。
* 缓解：我们在 **后处理修补**层补齐关键宏/依赖；实测以 eulermaker 报错为准逐项校正。

---

## 7. 断点机制与幂等设计

* **spilt**：基于 `order.ros`，按包**并行**拆分；已存在 `.orig` 会跳过。
* **stage**：已有 spec 默认**不覆盖**，只做**幂等修补**（可 `--force` 重新生成）。
* **upload**：本地 init / commit / push 幂等，远端存在则**切公开**并更新；最终产出 YAML。
* **验证**：在 eulermaker 遇错**标记并回填**；只修失败包再跑 `stage`/`upload`，不影响已好的仓库。

---

## 8. 已知问题 & 经验库

* **eulermaker 全量构建中断**：单包 OK；需要平台侧复现/排错。
* **`rti-connext-dds` 无法解析**：商用中间件；已策略性选用 Fast DDS。
* **`Source0` 不匹配 / `SOURCES` 路径**：已通过 `Source0: %{name}-%{version}.orig.tar.gz` + 归档命名规范解决。
* **`redhat-rpm-config`**：openEuler 无此包；Dockerfile 中删除该依赖。
* **`colcon` 找不到/venv 冲突**：修正 `setuptools<80` 并确保 `~/.local/bin` 在 PATH；脚本进入子 shell 自动激活。
* **动态拉网识别**：不引入静态扫描器；**以无网构建直接报错**作为快速识别手段（平台端最准确）。

---

## 9. 下一步（建议）

1. **复现 eulermaker 全量中断现象**，最小化用例 + 平台侧沟通。
2. 完成 **desktop** 闭包的首次全链路验证（只上必要包）。
3. 形成 **问题分类清单**：依赖缺失/宏差异/vendor 必要性/测试禁用边界等，每类给固定修补规则。
4. 增加一个**导入辅助脚本**：用 `package_repos.yaml` 直接喂给 eulermaker 的批量入口（如果支持）。
5. 将 `logs/` 目录结构固化为：`logs/{bloom,upload,platform}` 并在脚本里统一输出位置。

---

## 10. 附录

### 10.1 关键命令速查

```bash
# 初始化（含 venv、ros2_ws/src、可选 dockerfile）
./rosrpm.py deploy --distro jazzy

# 仅 desktop 闭包：生成 .orig（解包顶层目录匹配）
./spilt.py --distro jazzy --up-to desktop --jobs 16

# 批量 bloom + 规范化修补 spec（30s 超时，幂等）
./stage.py --distro jazzy --jobs 16 --timeout 30

# 批量创建远端仓库并推送，产出 package_repos.yaml
./upload_repos.py

# 如需批量设为公开（Gitee）
./gitee_make_public.py --ns <your_ns> --token <GITEE_TOKEN>
```

### 10.2 规范化修补点（`stage.py` 自动完成）

* 关闭测试：`%bcond_without tests`，`%check` 置于 `with_tests` 条件块。
* `Source0: %{name}-%{version}.orig.tar.gz`。
* 宏：去掉 python bytecompile 后处理；`__provides/__requires_exclude_from` 指向 `/opt/ros/jazzy`。
* `ament_python`：`BuildArch: noarch` + Python BuildRequires 兜底。

### 10.3 eulermaker 侧已知现象（待复现）

* 全量构建会中断；单包构建正常。建议：短期按**批量导入 + 分组构建**规避中断，逐批收敛。

### 10.4 变更记录（从旧方案到新方案）

* **过去**：本地 Docker 预装 BuildRequires → 无网 rpmbuild；多脚本、多状态，复现成本高。
* **现在**：**两段式**（准备/上传 → 平台构建验证），**官方工具链 + 最小修补**，可断点、可审计。


