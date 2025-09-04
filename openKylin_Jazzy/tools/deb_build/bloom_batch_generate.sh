#!/usr/bin/env bash
set -euo pipefail

# ====== 可调参数（可被环境变量覆盖） ======
WS="${WS:-$PWD}"
ROS_DISTRO="${ROS_DISTRO:-jazzy}"
OS_NAME="${OS_NAME:-ubuntu}"
OS_VERSION="${OS_VERSION:-noble}"
DIST_OK="${DIST_OK:-nile}"               # openKylin 发行代号
OK_SUFFIX="${OK_SUFFIX:-ok1}"             # openKylin 修订后缀，如 ok1
JOBS="${JOBS:-$(nproc)}"                  # 并行度
SKIP_EXISTING="${SKIP_EXISTING:-1}"       # 1=debian/ 存在则跳过；0=不跳过
LOGDIR_DEFAULT="/dev/shm/.bloom_logs"     # 没有 /dev/shm 就回退到工作区
LOGDIR="${LOGDIR:-$LOGDIR_DEFAULT}"
BLOOM_EXTRA_ARGS="${BLOOM_EXTRA_ARGS:-}"  # 额外 bloom 参数
FORCE="${FORCE:-0}"                       # 1=强制删除 debian/ 后再生成
ONLY="${ONLY:-}"                          # 仅处理这些包（逗号分隔）
SKIP_LIST="${SKIP_LIST:-rmw_connextdds,rmw_connextdds_common,python_orocos_kdl_vendor,rti_connext_dds_cmake_module,connext_cmake_module,rosidl_typesupport_connext_c,rosidl_typesupport_connext_cpp}"
MAINTAINER="${MAINTAINER:-}"              # 新维护人，形如：'OpenKylin ROS Team <pkg@example.com>'
SET_COMPAT="${SET_COMPAT:-0}"             # 1=将 compat 统一为 debhelper-compat (= 13) 并删 debian/compat

usage() {
  cat <<EOF
Usage: $(basename "$0") [options]

Options:
  --regenerate                 删除已有 debian/ 并重新生成（等价 FORCE=1, SKIP_EXISTING=0）
  --dist DIST                  openKylin 发行代号（默认: ${DIST_OK}）
  --ok-suffix SUFFIX           openKylin 修订后缀（默认: ${OK_SUFFIX}）
  -j, --jobs N                 并行度（默认: nproc）
  --only LIST                  仅处理白名单中的包（逗号分隔）
  --skip-list LIST             跳过这些包（逗号分隔）
  --extra ARGS                 追加给 bloom-generate 的参数（整体字符串）
  --maintainer "Name <mail>"   覆写 debian/control 的 Source 段 Maintainer
  --set-compat 13              使用 debhelper-compat (= 13) 并删除 debian/compat（默认不启用）
  -h, --help                   显示帮助

Env overrides:
  WS, ROS_DISTRO, OS_NAME, OS_VERSION, DIST_OK, OK_SUFFIX, LOGDIR,
  SKIP_EXISTING, FORCE, ONLY, SKIP_LIST, BLOOM_EXTRA_ARGS, JOBS,
  MAINTAINER, SET_COMPAT
EOF
}

# ====== 解析参数 ======
while [[ $# -gt 0 ]]; do
  case "$1" in
    --regenerate) FORCE=1; SKIP_EXISTING=0; shift;;
    --dist) DIST_OK="${2:-}"; shift 2;;
    --ok-suffix) OK_SUFFIX="${2:-}"; shift 2;;
    -j|--jobs) JOBS="${2:-}"; shift 2;;
    --only) ONLY="${2:-}"; shift 2;;
    --skip-list) SKIP_LIST="${2:-}"; shift 2;;
    --extra) BLOOM_EXTRA_ARGS="${2:-}"; shift 2;;
    --maintainer) MAINTAINER="${2:-}"; shift 2;;
    --set-compat) SET_COMPAT=1; shift;;
    -h|--help) usage; exit 0;;
    *) echo "[E] 未知参数: $1"; usage; exit 2;;
  esac
done

# ====== 预热 rosdistro 索引（降低网络开销） ======
CACHE_DIR="${WS}/.cache/rosdistro"
mkdir -p "$CACHE_DIR" "$LOGDIR"
INDEX_LOCAL="${CACHE_DIR}/index-v4.yaml"
if [ ! -s "$INDEX_LOCAL" ]; then
  echo "[I] 预取 rosdistro index 到本地缓存..."
  if command -v curl >/dev/null 2>&1; then
    curl -L --retry 3 -o "$INDEX_LOCAL" https://raw.githubusercontent.com/ros/rosdistro/master/index-v4.yaml
  else
    wget -O "$INDEX_LOCAL" https://raw.githubusercontent.com/ros/rosdistro/master/index-v4.yaml
  fi
fi
export BLOOM_ROSDISTRO_INDEX_URL="file://${INDEX_LOCAL}"

# 让 bloom 用 Ubuntu noble 的 rosdep 规则（作为上游基线）
export ROS_OS_OVERRIDE="ubuntu:24.04:noble"

# ====== 发现包目录（优先 colcon） ======
if command -v colcon >/dev/null 2>&1; then
  mapfile -t PKG_LINES < <(colcon list --base-paths "$WS/src")
  PKG_DIRS=()
  for line in "${PKG_LINES[@]}"; do
    path=$(awk '{print $2}' <<<"$line")
    [[ -n "$path" ]] && PKG_DIRS+=("$path")
  done
else
  mapfile -t PKG_DIRS < <(find "$WS/src" -name package.xml -print0 | xargs -0 -n1 dirname | sort -u)
fi

# ====== AWK 选择（兼容 mawk） ======
AWK_BIN="$(command -v gawk || command -v awk)"

# ====== 工具函数 ======
normalize_control() {
  # 仅改 Source 段：写入 Maintainer、保留 XSBC-Original-Maintainer，注入 Rules-Requires-Root: no
  # 可选：--set-compat 13 时，确保 debhelper-compat (= 13)，并删除 debian/compat
  local ctrl="$1" ; local maint="$2" ; local set_compat="${3:-0}"
  [[ -f "$ctrl" ]] || return 0

  cp -f "$ctrl" "${ctrl}.bak"
  awk -v RS='' 'NR==1{print > ".src.par"} NR>1{print > ".bin.par"}' "$ctrl"

  # 处理 Source 段
  if [[ -n "$maint" ]]; then
    local oldm=""; oldm=$(awk -F': *' 'BEGIN{IGNORECASE=1}/^Maintainer:/ {print $2}' .src.par || true)
    awk 'BEGIN{IGNORECASE=1} !/^Maintainer:/{print}' .src.par > .src.par.1

    if [[ -n "$oldm" && "$oldm" != "$maint" ]]; then
      if ! awk 'BEGIN{IGNORECASE=1}/^XSBC-Original-Maintainer:/{found=1} END{exit !found}' .src.par.1; then
        { echo "XSBC-Original-Maintainer: $oldm"; cat .src.par.1; } > .src.par.2
      else
        mv .src.par.1 .src.par.2
      fi
    else
      mv .src.par.1 .src.par.2
    fi

    # Rules-Requires-Root: no（幂等）
    if ! awk 'BEGIN{IGNORECASE=1}/^Rules-Requires-Root:/{found=1} END{exit !found}' .src.par.2; then
      { echo "Rules-Requires-Root: no"; cat .src.par.2; } > .src.par.3
    else
      cp .src.par.2 .src.par.3
    fi

    { echo "Maintainer: $maint"; cat .src.par.3; } > .src.final
  else
    # 仅确保 RRR
    if ! awk 'BEGIN{IGNORECASE=1}/^Rules-Requires-Root:/{found=1} END{exit !found}' .src.par; then
      { echo "Rules-Requires-Root: no"; cat .src.par; } > .src.final
    else
      cp .src.par .src.final
    fi
  fi

  # --set-compat 13：保证使用 debhelper-compat (= 13) 并删除 debian/compat（避免双重 compat）
  if [[ "$set_compat" = "1" ]]; then
    # 如果 Source 段缺 debhelper-compat，则追加一行
    if ! awk 'BEGIN{IGNORECASE=1}/^Build-Depends:/{bd=1} bd && /debhelper-compat[[:space:]]*\(= *13\)/{f=1} END{exit f?0:1}' .src.final; then
      # 简单处理：单独追加一行，reprepro/apt 解析正常
      { echo "Build-Depends: debhelper-compat (= 13)"; cat .src.final; } > .src.final2
      mv .src.final2 .src.final
    fi
    # 目录上删除 debian/compat
    rm -f "$(dirname "$ctrl")/compat" || true
  fi

  if [[ -s .bin.par ]]; then
    { cat .src.final; echo; cat .bin.par; } > "$ctrl"
  else
    cat .src.final > "$ctrl"
  fi
  rm -f .src.par .bin.par .src.par.* .src.final || true
}

normalize_rules() {
  local rules="$1"
  if [[ -f "$rules" ]]; then
    sed -i '/^export RMW_IMPLEMENTATION=/d' "$rules"
    if grep -q '^#!/usr/bin/make -f' "$rules"; then
      sed -i '1a export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp' "$rules"
    else
      { echo '#!/usr/bin/make -f'; echo 'export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp'; cat "$rules"; } > "${rules}.new"
      mv "${rules}.new" "$rules"
    fi
    chmod +x "$rules"
  fi
}

normalize_changelog() {
  # 仅修改第一行：
  # 1) 发行名 -> DIST_OK
  # 2) 版本修订段 -0noble( okN ) -> -0${DIST_OK}${OK_SUFFIX}
  local chlog="$1" ; local dist="$2" ; local ok="$3"
  [[ -f "$chlog" ]] || return 0
  "$AWK_BIN" -v d="$dist" -v s="$ok" '
    NR==1{
      # 发行名（右括号后到分号前）
      if ($0 ~ /\)[[:space:]]+[[:alnum:]_.+-]+;/) {
        sub(/\)[[:space:]]+[[:alnum:]_.+-]+;/, ") " d ";")
      }
      # 版本修订段替换：
      # 例： (...-0nobleok1) -> (...-0nileok1)
      if ($0 ~ /\(.*-0nobleok[0-9]+\)/) {
        re="-0nobleok[0-9]+\\)";
        rep="-0"d s")";
        gsub(re, rep);
      }
      # 例： (...-0noble) -> (...-0nileok1)
      else if ($0 ~ /\(.*-0noble\)/) {
        re="-0noble\\)";
        rep="-0"d s")";
        gsub(re, rep);
      }
      print; next
    }
    {print}
  ' "$chlog" > "${chlog}.tmp" && mv "${chlog}.tmp" "$chlog"
}

# ====== 生成一个包（子进程） ======
gen_one() {
  local pkgdir="$1"
  local ros_distro="$2"
  local os_name="$3"
  local os_version="$4"
  local extra="$5"
  local logdir="$6"
  local skip_existing="$7"
  local force="$8"
  local maint="$9"
  local dist_ok="${10}"
  local ok_suffix="${11}"
  local set_compat="${12}"

  local pkgxml="${pkgdir}/package.xml"
  [[ -f "$pkgxml" ]] || { echo "[SKIP] no package.xml in $pkgdir"; return 0; }
  local pkgname
  pkgname=$(grep -m1 -oP '(?<=<name>)[^<]+' "$pkgxml" || basename "$pkgdir")

  # —— 子进程内重建黑/白名单 ——
  declare -A _SKIPMAP=()
  IFS=',' read -ra __arr1 <<< "${SKIP_LIST:-}"
  for p in "${__arr1[@]}"; do p="${p//[[:space:]]/}"; [[ -n "$p" ]] && _SKIPMAP["$p"]=1; done
  declare -A _ONLYMAP=()
  if [[ -n "${ONLY:-}" ]]; then
    IFS=',' read -ra __arr2 <<< "${ONLY}"
    for p in "${__arr2[@]}"; do p="${p//[[:space:]]/}"; [[ -n "$p" ]] && _ONLYMAP["$p"]=1; done
  fi

  # 白/黑名单
  if [[ ${#_ONLYMAP[@]} -gt 0 && -z "${_ONLYMAP[$pkgname]:-}" ]]; then
    echo "[SKIP] $pkgname （不在 ONLY 白名单）"
    return 0
  fi
  if [[ -n "${_SKIPMAP[$pkgname]:-}" ]]; then
    echo "[SKIP] $pkgname （在 SKIP 黑名单）"
    return 0
  fi

  # 跳过/重生
  if [[ -d "${pkgdir}/debian" && "$skip_existing" = "1" && "$force" = "0" ]]; then
    echo "[HIT] $pkgname 已有 debian/，跳过（SKIP_EXISTING=1）"
    return 0
  fi
  if [[ "$force" = "1" ]]; then
    rm -rf "${pkgdir}/debian"
    echo "[REGEN] $pkgname 删除旧 debian/"
  fi

  local log="${logdir}/${pkgname}.log"
  pushd "$pkgdir" >/dev/null
  if bloom-generate rosdebian \
        --ros-distro "$ros_distro" \
        --os-name "$os_name" \
        --os-version "$os_version" \
        $extra >"$log" 2>&1; then

    # 规范：changelog / control / rules
    if [[ -f debian/changelog ]]; then
      normalize_changelog "debian/changelog" "$dist_ok" "$ok_suffix"
    fi
    if [[ -f debian/control ]]; then
      normalize_control "debian/control" "$maint" "$set_compat"
    fi
    normalize_rules "debian/rules"

    echo "[OK ] $pkgname"
  else
    echo "[ERR] $pkgname  （log: $log）"
    tail -n 40 "$log" | sed 's/^/      /'
  fi
  popd >/dev/null
}

export -f gen_one normalize_control normalize_rules normalize_changelog
export WS ROS_DISTRO OS_NAME OS_VERSION BLOOM_ROSDISTRO_INDEX_URL ROS_OS_OVERRIDE
export LOGDIR ONLY SKIP_LIST MAINTAINER DIST_OK OK_SUFFIX SET_COMPAT AWK_BIN

# ====== 并行跑起来 ======
printf '%s\0' "${PKG_DIRS[@]}" \
| xargs -0 -n1 -P "$JOBS" bash -c 'gen_one "$0" "'"$ROS_DISTRO"'" "'"$OS_NAME"'" "'"$OS_VERSION"'" "'"$BLOOM_EXTRA_ARGS"'" "'"$LOGDIR"'" "'"$SKIP_EXISTING"'" "'"$FORCE"'" "'"$MAINTAINER"'" "'"$DIST_OK"'" "'"$OK_SUFFIX"'" "'"$SET_COMPAT"'"'

echo
echo "[DONE] 并行=${JOBS}，日志目录：${LOGDIR}"