# 用 openKylin 构建 RISC-V 的 qcow2 镜像（可在 QEMU 启动）


> 目标：基于 openKylin 2.0 SP2（RuyiBook 资源包），在宿主机上**自制一个 riscv64 的 qcow2 镜像**，用于 QEMU 启动、进入系统做构建/测试。
> 读者预期：了解关键原理（qcow2、NBD、分区/挂载、从 ext4 映像回填 root/boot）、复用我的脚本化步骤并避坑。

---

## 一、我具体做了哪些工作（工作量一览）

* **选型与拆包分析**：确认 openKylin 提供的是**板卡资源包**（包含 `openKylin-*.ext4`、`boot.ext4` 等），不是可直接用的 QEMU 通用镜像 → 决定**自制 qcow2**。
* **磁盘制作路径打通**：从零创建 qcow2 → 用 **NBD** 挂成 `/dev/nbd0` → GPT 分区（ESP+root）→ FAT32/EXT4 格式化 → 挂载到 `/mnt/ok`。
* **内容回填**：把资源包内的 `openKylin-*.ext4`、`boot.ext4` **loop 只读挂载**，再 **rsync** 到我们新建的 root/boot 分区（避免把整个 tar 解到根分区导致塞满）。
* **fstab 与启动链**：按实际 **UUID** 写 `/etc/fstab`，并处理 QEMU 环境下 `/boot/efi` 的**容错**；验证 `root=/dev/vda2` 与 `root=UUID=` 两种启动参数。
* **内核/启动调试**：在需要时自编 RISC-V 内核（内建 virtio/EXT4/EFI_PARTITION 等关键项），定位 **UUID 解析→initramfs** 的关系；修正 `-append` 中的 root 写法避免 panic。
* **稳定性与清理**：解决 **NBD I/O 错误、nbd0 容量为 0、急救模式、磁盘占满** 等问题；加入镜像**瘦身**（`qemu-img convert -c zstd`）。
* **交付物组织**：产出 `Image`、`*.qcow2`、`run-openkylin.sh`（支持 `ROOT_ARG`），并记录可复用的命令序列。

---

## 二、核心概念（为什么要这么做）

* **qcow2 是“稀疏+可写快照”的虚拟磁盘格式**：创建后默认很小，写多少占多少，适合分发。
* **NBD（Network Block Device）**把“镜像文件”导出为**块设备**(`/dev/nbdX`)：分区/格式化工具只认块设备，NBD 就像“给镜像接上一根虚拟的 SATA 线”。
* openKylin RuyiBook 资源包里给的是 **root/boot 的 ext4 镜像文件**，我们要做的是：**新建空盘 → 分区 → 格式化 → 把 ext4 镜像里的内容拷入我们的新盘**。

---

## 三、前置准备

```bash
# 1) 下载 RISC-V 资源包（例）
wget https://releases.openkylin.top/2.0-SP2/openKylin-Embedded-V2.0-SP2-Release-RuyiBook-riscv64.tar.xz

# 2) 安装工具（以 Fedora 为例；Debian/Ubuntu 用 apt 替换包名）
sudo dnf install -y \
  qemu-system-riscv qemu-img qemu-nbd \
  edk2-riscv64 \
  parted dosfstools e2fsprogs rsync \
  util-linux coreutils
```

---

## 四、实操：从零制作 qcow2 并填充系统

> 下文默认工作目录为 `~/openkylin-rv`，根/ESP 的临时挂载点为 `/mnt/ok` 与 `/mnt/ok/boot/efi`。

### 4.1 新建 qcow2 并通过 NBD 暴露成块设备

```bash
mkdir -p ~/openkylin-rv && cd ~/openkylin-rv
qemu-img create -f qcow2 ok-openkylin-rv.qcow2 16G

# 关键：--fork 让 qemu-nbd 以守护方式常驻；--discard 便于后续打洞瘦身
sudo modprobe nbd max_part=16
sudo qemu-nbd -f qcow2 --cache=none --discard=unmap --fork \
  -c /dev/nbd0 ./ok-openkylin-rv.qcow2
sudo udevadm settle

# 确认内核看到了容量（应为 17179869184）
sudo blockdev --getsize64 /dev/nbd0
```

> **为什么有时显示 0 字节？**
> 常见原因：qemu-nbd 没常驻、udev 规则未生效、之前的挂载占用没清理。可先 `sudo qemu-nbd -d /dev/nbd0`、卸载所有 `/mnt/ok/*` 后重来。

### 4.2 GPT 分区 + 格式化（ESP + root）

```bash
# GPT：ESP(FAT32) 1MiB–513MiB，root(ext4) 余下全部
sudo parted -s /dev/nbd0 mklabel gpt
sudo parted -s /dev/nbd0 mkpart ESP fat32 1MiB 513MiB
sudo parted -s /dev/nbd0 set 1 esp on
sudo parted -s /dev/nbd0 mkpart root ext4 513MiB 100%

sudo partprobe /dev/nbd0
sudo udevadm settle
lsblk /dev/nbd0

# 格式化
sudo mkfs.vfat -F32 -n EFI   /dev/nbd0p1
sudo mkfs.ext4 -L rootfs     /dev/nbd0p2
```

### 4.3 挂载目标分区（我们的“新系统盘”）

```bash
sudo mkdir -p /mnt/ok/boot/efi
sudo mount /dev/nbd0p2 /mnt/ok
sudo mount /dev/nbd0p1 /mnt/ok/boot/efi
```

### 4.4 从资源包“回填”root/boot（关键技巧）

> 资源包不是一个可直接用的 root 目录，而是**多个 ext4 文件**。做法是**loop 只读挂载**这些 `*.ext4`，再把内容同步到我们的分区。

```bash
# 解到 staging 区
mkdir -p ./_stage
sudo tar -xpf ./openKylin-Embedded-*.tar.xz -C ./_stage

# 定位 root/boot 映像
ROOT_EXT=$(find ./_stage -maxdepth 2 -name 'openKylin-*.ext4' -print -quit)
BOOT_EXT=$(find ./_stage -maxdepth 2 -name 'boot.ext4' -print -quit)

# loop 只读挂载
sudo mkdir -p /mnt/srcroot /mnt/srcboot
sudo mount -o ro,loop "$ROOT_EXT" /mnt/srcroot
sudo mount -o ro,loop "$BOOT_EXT"  /mnt/srcboot

# 同步（保留权限/属主；无 xattr 的系统可加 --no-xattrs）
sudo rsync -aHAx --numeric-ids --info=progress2 /mnt/srcroot/ /mnt/ok/
sudo rsync -aHAx --numeric-ids --info=progress2 /mnt/srcboot/  /mnt/ok/boot/

# 验证：现在 /mnt/ok 里应有 /etc /bin /lib 等
test -d /mnt/ok/etc && echo "OK: root 已填充"
```

> **别把整个 tar 直接解到 `/mnt/ok`！** 那样会把几 GiB 的中间文件、多个映像文件原封不动塞进去，根分区会满。

### 4.5 写 `fstab`（按当前分区 UUID）

```bash
ROOT_UUID=$(sudo blkid -s UUID -o value /dev/nbd0p2)
ESP_UUID=$(sudo blkid -s UUID -o value /dev/nbd0p1)

sudo tee /mnt/ok/etc/fstab >/dev/null <<EOF
UUID=$ROOT_UUID  /         ext4  defaults,noatime,errors=remount-ro 0 1
UUID=$ESP_UUID   /boot/efi vfat  umask=0077 0 2
EOF
```

> **在 QEMU 环境中避免“急救模式”**：如 ESP 暂时不需要或可能缺席，可把第二行改为：
> `UUID=$ESP_UUID /boot/efi vfat umask=0077,nofail,x-systemd.device-timeout=1s 0 2`

### 4.6 清理挂载与解绑 NBD

```bash
sudo umount -R /mnt/srcroot /mnt/srcboot || true
sudo umount -R /mnt/ok
sudo qemu-nbd -d /dev/nbd0
```

---

## 五、启动与验证（QEMU 最小化示例）

### 5.1 使用自编或资源包中的内核 `Image`

> 若无 initramfs，**更稳妥写法是 `root=/dev/vda2`**（virtio 盘在 QEMU `-machine virt` 下通常是 `/dev/vda`）。

```bash
IMG="$HOME/openkylin-rv/ok-openkylin-rv.qcow2"
KIMG="/path/to/Image"   # 你编的或包里提供的内核镜像

qemu-system-riscv64 \
  -machine virt -cpu rv64 -m 4G -smp 4 \
  -nographic -bios default \
  -drive file="$IMG",format=qcow2,if=virtio \
  -device virtio-rng-pci \
  -netdev user,id=net0,hostfwd=tcp::10022-:22 \
  -device virtio-net-pci,netdev=net0 \
  -kernel "$KIMG" \
  -append "console=ttyS0 earlycon=sbi root=/dev/vda2 rootfstype=ext4 rootwait rw"
```

> 想用 `root=UUID=`？
> 需要确保**有 initramfs**（如用 `dracut` 生成）或把 blkid/udev 相关能力准备好，否则早期阶段无法解析 UUID → 内核 panic。

---

## 六、镜像瘦身与分发

```bash
# 用 zstd 压缩 + 1MiB 簇 + lazy_refcounts，并打洞剔除空洞
qemu-img convert -p -S 4k -O qcow2 -c \
  -o compression_type=zstd,cluster_size=1M,lazy_refcounts=on \
  "$IMG" tmp.qcow2 && mv -v tmp.qcow2 "$IMG"

qemu-img info "$IMG"
du -h "$IMG"
```

> **经验**：用前述参数二次转换后，镜像通常能显著变小，便于在团队内分发。

---

## 七、常见问题与快速排错

* **`parted` I/O 错误 / `/dev/nbd0` 容量为 0**

  1. `sudo qemu-nbd -d /dev/nbd0` 解绑；2) `sudo modprobe nbd max_part=16`；
  2. 重新 `qemu-nbd ... --fork -c /dev/nbd0`；4) `sudo udevadm settle`；
  3. 若仍失败，`umount -R /mnt/ok` 并 `fuser -vm /dev/nbd*` 查占用。
* **根分区 100%**
  清理误解包的 `openKylin-Embedded-*` 大目录；必要时 `sudo tune2fs -m 0 /dev/nbd0p2` 回收保留块，卸载后再试。
* **Kernel panic：VFS 无法挂载 root（UUID）**
  改用 `root=/dev/vda2`，或给系统准备 **initramfs**；确保内核内建 `VIRTIO_*`、`EXT4`、`EFI_PARTITION`、`DEVTMPFS(_MOUNT)` 等。
* **启动进“急救模式”**
  多半是 `/boot/efi` 未就绪造成挂载失败；给该行加 `nofail,x-systemd.device-timeout=1s`。

---

## 八、你可以怎么借鉴我的内容（复用建议）

* **模板化脚本**：把 4.1–4.6 的命令整理成两个脚本：

  1. `mkimg.sh`：创建 qcow2→NBD→分区→格式化→挂载；
  2. `fillroot.sh`：loop 挂载 `*.ext4` → rsync → 写 fstab → 清理。
* **可移植性**：同样的套路可用于**其他发行版**的“板卡/嵌入式资源包”（内含 `root.ext4`/`boot.ext4`）快速转制为可在 **QEMU 启动**的 qcow2。
* **启动脚本**：准备一个 `run-*.sh`，支持 `ROOT_ARG=/dev/vda2|UUID=...` 环境变量，便于切换。
* **CI 复现**：把镜像与 `run.sh` 放到工件仓库，CI 中用 `qemu-system-riscv64` 开机跑 smoke test（SSH 通过 `-netdev user,hostfwd=tcp::10022-:22`）。
* **瘦身发布**：统一用 `qemu-img convert -c zstd` 压缩后分发，明显节省下载时间。

---

## 九、附：最小命令清单（可直接黏贴跑）

```bash
# 创建与附着
qemu-img create -f qcow2 ok-openkylin-rv.qcow2 16G
sudo modprobe nbd max_part=16
sudo qemu-nbd -f qcow2 --cache=none --discard=unmap --fork -c /dev/nbd0 ./ok-openkylin-rv.qcow2
sudo udevadm settle
sudo blockdev --getsize64 /dev/nbd0

# 分区/格式化
sudo parted -s /dev/nbd0 mklabel gpt
sudo parted -s /dev/nbd0 mkpart ESP fat32 1MiB 513MiB
sudo parted -s /dev/nbd0 set 1 esp on
sudo parted -s /dev/nbd0 mkpart root ext4 513MiB 100%
sudo partprobe /dev/nbd0 && sudo udevadm settle
sudo mkfs.vfat -F32 -n EFI /dev/nbd0p1
sudo mkfs.ext4 -L rootfs /dev/nbd0p2

# 挂载目标盘
sudo mkdir -p /mnt/ok/boot/efi
sudo mount /dev/nbd0p2 /mnt/ok
sudo mount /dev/nbd0p1 /mnt/ok/boot/efi

# 解包与回填
mkdir -p ./_stage
sudo tar -xpf ./openKylin-Embedded-*.tar.xz -C ./_stage
ROOT_EXT=$(find ./_stage -maxdepth 2 -name 'openKylin-*.ext4' -print -quit)
BOOT_EXT=$(find ./_stage -maxdepth 2 -name 'boot.ext4' -print -quit)
sudo mkdir -p /mnt/srcroot /mnt/srcboot
sudo mount -o ro,loop "$ROOT_EXT" /mnt/srcroot
sudo mount -o ro,loop "$BOOT_EXT"  /mnt/srcboot
sudo rsync -aHAx --numeric-ids /mnt/srcroot/ /mnt/ok/
sudo rsync -aHAx --numeric-ids /mnt/srcboot/  /mnt/ok/boot/

# fstab
ROOT_UUID=$(sudo blkid -s UUID -o value /dev/nbd0p2)
ESP_UUID=$(sudo blkid -s UUID -o value /dev/nbd0p1)
sudo tee /mnt/ok/etc/fstab >/dev/null <<EOF
UUID=$ROOT_UUID  /         ext4  defaults,noatime,errors=remount-ro 0 1
UUID=$ESP_UUID   /boot/efi vfat  umask=0077,nofail,x-systemd.device-timeout=1s 0 2
EOF

# 清理
sudo umount -R /mnt/srcroot /mnt/srcboot || true
sudo umount -R /mnt/ok
sudo qemu-nbd -d /dev/nbd0
```

---

> 以上就是“如何制作可在 QEMU 启动的 openKylin riscv64 qcow2 镜像”的完整实践路径。
> 可以通过 `run-openkylin.sh` 启动脚本启动虚拟机，方便直接复用到 CI 或日常测试。
