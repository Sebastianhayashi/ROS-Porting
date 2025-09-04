#!/usr/bin/env bash
set -Eeuo pipefail

WORKSPACE="${WORKSPACE:-$(pwd)}"
ROS_DISTRO="${ROS_DISTRO:-jazzy}"
SUITE="${SUITE:-nile}"
CHROOT="${CHROOT:-nile-amd64-sbuild}"

DSC_DIR="${DSC_DIR:-$WORKSPACE/out/srcpkg}"
RESULTS_DIR="${RESULTS_DIR:-$WORKSPACE/out/build-results}"
REPO_DIR="${REPO_DIR:-$WORKSPACE/out/repo}"
REPO_URL="${REPO_URL:-http://127.0.0.1:8000/}"
SBUILD_EXTRA="${SBUILD_EXTRA:---no-run-lintian}"
JOBS="${JOBS:-$(nproc)}"
STOP_ON_MISSING="${STOP_ON_MISSING:-0}"

command -v colcon >/dev/null || { echo "缺少 colcon"; exit 1; }
command -v sbuild  >/dev/null || { echo "缺少 sbuild"; exit 1; }
command -v reprepro >/dev/null || { echo "缺少 reprepro"; exit 1; }
[[ -d "$REPO_DIR" ]] || { echo "仓库目录不存在：$REPO_DIR"; exit 1; }

mkdir -p "$RESULTS_DIR"

# 拓扑顺序
if colcon list --help | grep -q -- '--topological-order'; then
  mapfile -t PKG_NAMES < <(colcon list --base-paths "$WORKSPACE/src" --names-only --topological-order)
else
  mapfile -t PKG_NAMES < <(colcon list --base-paths "$WORKSPACE/src" --names-only)
fi

# 找某包的 .dsc（取版本最高的一个）
find_dsc() {
  local debname="$1"
  shopt -s nullglob
  local arr=( "$DSC_DIR/${debname}"_*.dsc )
  shopt -u nullglob
  [[ ${#arr[@]} -eq 0 ]] && return 1
  printf '%s\n' "${arr[@]}" | sort -V | tail -n1
}

export DEB_BUILD_OPTIONS="parallel=${JOBS}"

for NAME in "${PKG_NAMES[@]}"; do
  DEBNAME="ros-${ROS_DISTRO}-$(echo "$NAME" | tr '_' '-')"
  DSC_REL="$(find_dsc "$DEBNAME" || true)"
  if [[ -z "$DSC_REL" ]]; then
    echo "跳过 $NAME：$DSC_DIR 没有 ${DEBNAME}_*.dsc"
    [[ "$STOP_ON_MISSING" == "1" ]] && exit 2
    continue
  fi

  DSC_ABS="$(readlink -f "$DSC_REL")"

  echo "==> sbuild 构建：$DEBNAME"
  (
    cd "$RESULTS_DIR"
    sbuild -A -s -d "$SUITE" --chroot "$CHROOT" $SBUILD_EXTRA \
           --debbuildopts="-j${JOBS}" \
           --extra-repository="deb [trusted=yes] ${REPO_URL} ${SUITE} main" \
           "$DSC_ABS"
  )

  # include 最新的 changes 进仓库
  CHG=$(ls -1t "$RESULTS_DIR"/${DEBNAME}_*.changes 2>/dev/null | head -n1 || true)
  if [[ -n "$CHG" ]]; then
    echo "   -> include: $CHG"
    reprepro -b "$REPO_DIR" include "$SUITE" "$CHG" || true
  else
    echo "   -> 警告：没找到 ${DEBNAME} 的 .changes"
  fi
done

echo "完成：结果在 $RESULTS_DIR，仓库在 $REPO_DIR（HTTP: ${REPO_URL})"