#!/usr/bin/env bash
# push_bin.sh — 查找最新编译产物，更新 firmware/ + version.txt，推送 GitHub
# 用法：bash push_bin.sh
#   Arduino IDE 编译完成后运行（Ctrl+R 编译 或 Ctrl+Alt+S 导出二进制文件）

set -e
cd "$(dirname "$0")"

INO="BNO080_HAND_DIY/BNO080_HAND_DIY.ino"
FIRMWARE_DIR="firmware"
BIN_DEST="${FIRMWARE_DIR}/BNO080_HAND_DIY.bin"
VER_FILE="version.txt"
BIN_NAME="BNO080_HAND_DIY.ino.bin"

# ── 提取 FW_VER ──────────────────────────────────────────────────
VER=$(sed -n 's/^#define FW_VER[[:space:]]*"\(.*\)"/\1/p' "$INO")
[ -n "$VER" ] || { echo "ERROR: 无法提取 FW_VER，请检查 ${INO}"; exit 1; }
echo ">>> FW_VER: ${VER}"

# ── 查找候选 .bin 文件 ───────────────────────────────────────────
# 候选 1: sketch build 目录（Arduino IDE 2.x 编译/导出产物，最可靠）
# 候选 2: Windows TEMP 下的 arduino 临时目录（Ctrl+R 编译时产物）
declare -a candidates

while IFS= read -r f; do candidates+=("$f"); done < <(
    find "BNO080_HAND_DIY/build" -name "$BIN_NAME" 2>/dev/null
)

WINTMP="${TEMP:-${TMP:-}}"
if [ -n "$WINTMP" ]; then
    while IFS= read -r f; do candidates+=("$f"); done < <(
        find "$WINTMP" -maxdepth 4 -name "$BIN_NAME" 2>/dev/null
    )
fi

if [ ${#candidates[@]} -eq 0 ]; then
    echo "ERROR: 未找到 ${BIN_NAME}"
    echo "  请先在 Arduino IDE 中编译（Ctrl+R）或导出（Sketch → 导出编译二进制文件）"
    exit 1
fi

# ── 取修改时间最新的 .bin ────────────────────────────────────────
LATEST_BIN=""
LATEST_MT=0
for f in "${candidates[@]}"; do
    mt=$(stat -c %Y "$f" 2>/dev/null) || continue
    if [ "$mt" -gt "$LATEST_MT" ]; then LATEST_MT=$mt; LATEST_BIN=$f; fi
done

[ -n "$LATEST_BIN" ] || { echo "ERROR: 候选文件均无法读取修改时间"; exit 1; }

# ── 新鲜度检查（超 60 分钟提示确认）────────────────────────────
NOW=$(date +%s)
AGE=$(( NOW - LATEST_MT ))
if [ "$AGE" -gt 3600 ]; then
    printf "WARNING: 该 .bin 距现在 %dm%ds，可能不是最新编译产物\n" "$(( AGE/60 ))" "$(( AGE%60 ))"
    printf "  路径: %s\n  继续？(y/N) " "$LATEST_BIN"
    read -r ans
    [[ "$ans" =~ ^[Yy]$ ]] || { echo "已取消。"; exit 0; }
else
    printf ">>> 找到 .bin  [%dm%ds 前编译]\n    %s\n" "$(( AGE/60 ))" "$(( AGE%60 ))" "$LATEST_BIN"
fi

# ── 复制到 firmware/ ─────────────────────────────────────────────
mkdir -p "$FIRMWARE_DIR"
cp "$LATEST_BIN" "$BIN_DEST"
SIZE=$(wc -c < "$BIN_DEST" | tr -d ' ')
printf ">>> 已复制到 %s  (%s bytes)\n" "$BIN_DEST" "$SIZE"

# ── 更新 version.txt ─────────────────────────────────────────────
printf '%s\n' "$VER" > "$VER_FILE"
echo ">>> version.txt → ${VER}"

# ── 本地快照 ─────────────────────────────────────────────────────
echo ">>> 运行 bak.sh ..."
bash "BNO080_HAND_DIY/bak.sh"

# ── git add / commit / push ──────────────────────────────────────
echo ">>> git add ..."
git add .

if git diff --cached --quiet; then
    echo "SKIP: 没有变更，无需提交。"
    exit 0
fi

MSG="BNO080_HAND_DIY ${VER} [bin+ver]"
echo ">>> git commit: ${MSG}"
git commit -m "$MSG"

echo ">>> git push origin master"
git push origin master

echo "OK: ${VER} 固件已推送 → 设备上电后自动 OTA 更新"
