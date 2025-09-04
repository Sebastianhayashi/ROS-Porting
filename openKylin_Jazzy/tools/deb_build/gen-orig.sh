#!/usr/bin/env bash
set -Eeuo pipefail

# ===== 可调参数 =====
ROS_DISTRO="${ROS_DISTRO:-jazzy}"            # ROS 发行
OS_NAME="${OS_NAME:-openkylin}"              # 供 bloom 用
OS_VERSION="${OS_VERSION:-nile}"             # 供 bloom 用
WORKSPACE="${WORKSPACE:-$(pwd)}"             # 工作区根，一般就是 ros2_ws
BASE_PATHS="${BASE_PATHS:-$WORKSPACE/src}"   # 包所在的基路径
OUT_ORIG="${OUT_ORIG:-$WORKSPACE/out/orig}"  # orig 输出目录
OUT_DSC="${OUT_DSC:-$WORKSPACE/out/dsc}"     # dsc 输出目录（仅 WITH_BLOOM=1 时用）
WITH_BLOOM="${WITH_BLOOM:-0}"                # 设 1 则 bloom+产 dsc

# ===== 依赖检查 =====
command -v colcon >/dev/null || { echo "需要 colcon"; exit 1; }
mkdir -p "$OUT_ORIG"
[[ "$WITH_BLOOM" == "1" ]] && { mkdir -p "$OUT_DSC"; command -v bloom-generate >/dev/null || sudo apt-get -y install python3-bloom; command -v dch >/dev/null || sudo apt-get -y install devscripts; }

# ===== 取所有包路径 =====
mapfile -t PKG_PATHS < <(colcon list --base-paths "$BASE_PATHS" --paths-only)

# ===== 主循环 =====
for P in "${PKG_PATHS[@]}"; do
  PKGXML="$P/package.xml"
  [[ -f "$PKGXML" ]] || { echo "跳过 $P: 没有 package.xml"; continue; }

  # 从 package.xml 取包名与版本
  NAME=$(sed -n 's|.*<name>\(.*\)</name>.*|\1|p' "$PKGXML" | head -n1)
  UPVER=$(sed -n 's|.*<version>\(.*\)</version>.*|\1|p' "$PKGXML" | head -n1)
  [[ -n "$NAME" && -n "$UPVER" ]] || { echo "跳过 $P: name/version 解析失败"; continue; }

  SRCNAME="$NAME"  # 上游名，保持下划线即可
  DEBNAME="ros-${ROS_DISTRO}-$(echo "$NAME" | tr '_' '-')"  # Debian 源包名用连字符
  ORIG="$OUT_ORIG/${DEBNAME}_${UPVER}.orig.tar.gz"

  echo "==> 生成 orig: $DEBNAME $UPVER  <-  $P"
  rm -f "$ORIG"

  # 用 tar 从工作树直接打包（稳，且不受 zsh 影响）
  (
    cd "$P"
    tar --exclude=./debian --exclude-vcs --exclude='**/__pycache__' \
        -czf "$ORIG" \
        --transform "s,^,${SRCNAME}-${UPVER}/," .
  )
  # 快速验包
  tar tzf "$ORIG" >/dev/null || { echo "ERROR: $DEBNAME 的 orig 损坏"; exit 1; }

  # ===== 可选：用 bloom 生成 debian 并产 .dsc =====
  if [[ "$WITH_BLOOM" == "1" ]]; then
    echo "   -> bloom 生成 debian 并产 .dsc"
    (
      cd "$P"
      # 生成 debian/
      bloom-generate rosdebian --ros-distro "$ROS_DISTRO" --os-name "$OS_NAME" --os-version "$OS_VERSION"

      # 修 changelog 顶行到 nile，并写一条变更，避免“拖曳符”
      dch -b -v "${UPVER}-0${OS_VERSION}" --distribution "$OS_VERSION" "Rebuild for ${OS_NAME} (${OS_VERSION})."

      # 防止 dpkg-source 误用当前目录的同名 orig
      rm -f "./${DEBNAME}_${UPVER}.orig.tar.gz"
      # 产 .dsc（不跑 debhelper，避免宿主机依赖）
      dpkg-source -b .
    )
    # 收集产物到 OUT_DSC
    shopt -s nullglob
    for f in "$P"/../${DEBNAME}_${UPVER}-0${OS_VERSION}.dsc \
             "$P"/../${DEBNAME}_${UPVER}-0${OS_VERSION}.debian.tar.* \
             "$P"/../${DEBNAME}_${UPVER}.orig.tar.* \
             "$P"/../${DEBNAME}_${UPVER}-0${OS_VERSION}_source.changes; do
      [[ -f "$f" ]] && cp -f "$f" "$OUT_DSC/"
    done
    shopt -u nullglob
  fi
done

echo "完成：orig 输出在 $OUT_ORIG"
[[ "$WITH_BLOOM" == "1" ]] && echo "完成：dsc 输出在 $OUT_DSC"