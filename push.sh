#!/usr/bin/env bash
# push.sh — BNO080_HAND_DIY 固件推送到 GitHub
# 用法：bash push.sh [可选提交说明]
#   - 无参数：自动用 FW_VER 生成提交说明
#   - 带参数：bash push.sh "修复 BLE 丢帧"

set -e
cd "$(dirname "$0")"

SKETCH_DIR="BNO080_HAND_DIY"
INO="${SKETCH_DIR}/BNO080_HAND_DIY.ino"
BAK_SH="${SKETCH_DIR}/bak.sh"

# 提取 FW_VER
VER=$(sed -n 's/^#define FW_VER[[:space:]]*"\(.*\)"/\1/p' "$INO")
if [ -z "$VER" ]; then
    echo "ERROR: 无法提取 FW_VER，请检查 ${INO}"
    exit 1
fi

echo ">>> FW_VER: ${VER}"

# 本地快照
echo ">>> 运行 bak.sh ..."
bash "$BAK_SH"

# 提交说明
if [ -n "$1" ]; then
    MSG="BNO080_HAND_DIY ${VER}: $1"
else
    MSG="BNO080_HAND_DIY ${VER}"
fi

# 推送
echo ">>> git add ..."
git add .

if git diff --cached --quiet; then
    echo "SKIP: 没有变更，无需提交。"
    exit 0
fi

echo ">>> git commit: ${MSG}"
git commit -m "$MSG"

echo ">>> git push origin master"
git push origin master

echo "OK: 已推送 ${VER}"
