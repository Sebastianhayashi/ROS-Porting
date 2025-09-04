#!/usr/bin/env bash
set -Eeuo pipefail
WORKSPACE="${WORKSPACE:-$(pwd)}"
REPO_DIR="${REPO_DIR:-$WORKSPACE/out/repo}"
DIST="${DIST:-nile}"
COMP="${COMP:-main}"

mkdir -p "$REPO_DIR/conf"
cat >"$REPO_DIR/conf/distributions" <<EOF
Origin: Local
Label: Local
Codename: $DIST
Architectures: amd64 source
Components: $COMP
Description: Local repo for $DIST
# 没签名就不写 SignWith，后面用 trusted=yes
EOF
reprepro -b "$REPO_DIR" export || true

echo "本地仓库初始化在 $REPO_DIR"