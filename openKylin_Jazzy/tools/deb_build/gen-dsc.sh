#!/usr/bin/env bash
set -Eeuo pipefail

# ===== 可调参数 =====
ROS_DISTRO="${ROS_DISTRO:-jazzy}"                 # 用于计算 Debian 源包名
WORKSPACE="${WORKSPACE:-$(pwd)}"                  # 你的工作区根（含 src）
BASE_PATHS="${BASE_PATHS:-$WORKSPACE/src}"        # 包所在路径
OUT_ORIG="${OUT_ORIG:-$WORKSPACE/out/orig}"       # 你已经生成的 orig 目录
OUT_DSC="${OUT_DSC:-$WORKSPACE/out/srcpkg}"       # 输出 dsc/changes/debian.tar 的目录
STOP_ON_MISSING="${STOP_ON_MISSING:-0}"           # 找不到 orig 时是否中止（1=中止，0=跳过）

# ===== 依赖检查 =====
command -v colcon >/dev/null || { echo "缺少 colcon"; exit 1; }
command -v dpkg-source >/dev/null || { echo "缺少 dpkg-dev（dpkg-source）"; exit 1; }
[[ -d "$OUT_ORIG" ]] || { echo "找不到 OUT_ORIG: $OUT_ORIG"; exit 1; }
mkdir -p "$OUT_DSC"

# ===== 拿到所有包路径 =====
mapfile -t PKG_PATHS < <(colcon list --base-paths "$BASE_PATHS" --paths-only)

# ===== 小函数：从 OUT_ORIG 找对应的 orig 路径 =====
find_orig() {
  local debname="$1" upver="$2"
  # 支持 .gz/.xz/.bz2，按找到的第一个用
  local cand
  shopt -s nullglob
  for cand in "$OUT_ORIG/${debname}_${upver}.orig.tar."{gz,xz,bz2}; do
    [[ -f "$cand" ]] && { echo "$cand"; shopt -u nullglob; return 0; }
  done
  shopt -u nullglob
  return 1
}

# ===== 主循环 =====
for P in "${PKG_PATHS[@]}"; do
  PKGXML="$P/package.xml"
  [[ -f "$PKGXML" ]] || { echo "跳过 $P：没有 package.xml"; continue; }
  [[ -d "$P/debian" ]] || { echo "跳过 $P：没有 debian/"; continue; }

  # 包名与上游版本
  NAME=$(sed -n 's|.*<name>\(.*\)</name>.*|\1|p' "$PKGXML" | head -n1)
  UPVER=$(sed -n 's|.*<version>\(.*\)</version>.*|\1|p' "$PKGXML" | head -n1)
  [[ -n "$NAME" && -n "$UPVER" ]] || { echo "跳过 $P：name/version 解析失败"; continue; }

  SRCNAME="$NAME"
  DEBNAME="ros-${ROS_DISTRO}-$(echo "$NAME" | tr '_' '-')"

  echo "==> 生成 dsc: $DEBNAME $UPVER  <-  $P"

  # 找到 OUT_ORIG 里的 orig
  ORIG_PATH="$(find_orig "$DEBNAME" "$UPVER" || true)"
  if [[ -z "${ORIG_PATH:-}" ]]; then
    echo "    警告：未找到 $DEBNAME_${UPVER}.orig.tar.* 于 $OUT_ORIG"
    [[ "$STOP_ON_MISSING" == "1" ]] && { echo "    设了 STOP_ON_MISSING=1，退出"; exit 2; }
    continue
  fi

  # 在包的上一级放一个指向 orig 的符号链接（dpkg-source 会从上一级取）
  PARENT="$(dirname "$P")"
  EXT="${ORIG_PATH##*.}"  # gz/xz/bz2
  LINK="${PARENT}/${DEBNAME}_${UPVER}.orig.tar.${EXT}"

  # 清掉当前目录里的同名 orig，免得 dpkg-source 先用错的那个
  rm -f "$P/${DEBNAME}_${UPVER}.orig.tar."{gz,xz,bz2} 2>/dev/null || true
  ln -sf "$ORIG_PATH" "$LINK"

  # 产源码包描述（只做 source，不跑 debhelper）
  (
    cd "$P"
    dpkg-source -b .
  )

  # 收集产物到 OUT_DSC
  shopt -s nullglob
  for f in "${PARENT}/${DEBNAME}_${UPVER}.orig.tar."{gz,xz,bz2} \
           "${PARENT}/${DEBNAME}_${UPVER}-"*.debian.tar.* \
           "${PARENT}/${DEBNAME}_${UPVER}-"*_source.changes \
           "${PARENT}/${DEBNAME}_${UPVER}-"*.dsc; do
    [[ -f "$f" ]] && cp -f "$f" "$OUT_DSC/"
  done
  shopt -u nullglob
done

echo "完成：dsc/changes/debian.tar 已输出到 $OUT_DSC"