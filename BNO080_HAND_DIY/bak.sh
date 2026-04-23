#!/usr/bin/env bash
# bak.sh — BNO080_HAND_DIY 固件快照脚本
# 用法：bash bak.sh
# 检测当前 FW_VER，若 _backups/ 中无对应版本则创建快照，否则跳过。

set -e
cd "$(dirname "$0")"

SKETCH="BNO080_HAND_DIY"
INO="${SKETCH}.ino"
BAK_DIR="_backups"

# 提取 FW_VER（sed 兼容 Windows Git Bash，不用 grep -P）
VER=$(sed -n 's/^#define FW_VER[[:space:]]*"\(.*\)"/\1/p' "$INO")
if [ -z "$VER" ]; then
    echo "ERROR: 无法从 $INO 提取 FW_VER，请检查 #define FW_VER 格式。"
    exit 1
fi

DATE=$(date +%Y%m%d)
SNAP="${SKETCH}_${VER}_${DATE}"
SNAP_DIR="${BAK_DIR}/${SNAP}"

if [ -d "$SNAP_DIR" ]; then
    echo "SKIP: 备份已存在 → ${SNAP_DIR}"
    exit 0
fi

mkdir -p "$SNAP_DIR"
cp "$INO" "${SNAP_DIR}/${SNAP}.ino"
echo "OK:   快照已创建 → ${SNAP_DIR}/${SNAP}.ino"
