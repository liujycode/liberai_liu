"""
bno_hand_ble.py  — BNO_HAND v2.0 PC BLE 接收脚本
依赖: pip install bleak
用法: python bno_hand_ble.py [--addr AA:BB:CC:DD:EE:FF]
"""

import asyncio
import struct
import time
import argparse
from collections import deque

from bleak import BleakClient, BleakScanner

# ── GATT UUID ─────────────────────────────────────────────────────
DEVICE_NAME = "BNO_HAND"
DATA_UUID   = "0000FFE1-0000-1000-8000-00805F9B34FB"   # NOTIFY
CMD_UUID    = "0000FFE2-0000-1000-8000-00805F9B34FB"   # WRITE_NO_RSP
INFO_UUID   = "0000FFE3-0000-1000-8000-00805F9B34FB"   # READ

# ── 帧格式常量（与固件 build_binary_frame() 一致）─────────────────
FRAME_LEN         = 91      # 单帧字节数（int8 四元数压缩格式）
NUM_CH            = 16      # 通道数
Q_SCALE           = 127.0   # 固件 Q_INT_SCALE（int8 ×127）
FRAMES_PER_NOTIFY = 1       # v2.11固件：每帧一个 notify

# ── 全局统计 ──────────────────────────────────────────────────────
_stat = {
    'notify': 0, 'frame': 0, 'xor_err': 0, 'magic_err': 0,
    't0': 0.0, 't_stat': 0.0,
}
_last_seq      = -1
_drop_total    = 0            # 累计丢帧数（seq 跳变）

# rx_fps：PC 端 BLE notify 到达时间戳滑动窗口（最近 5s 内的帧）
_arr_times: deque = deque()
RX_WINDOW_S = 5.0            # 滑动窗口长度（秒）

# fw_fps：用固件内嵌 ts_ms 时间戳计算真实输出速率
_fw_prev: tuple | None = None   # (seq, ts_ms) of last received frame
_fw_fps   = 0.0
_rx_fps   = 0.0

# 最近一帧解析结果（供定时打印用）
_last_frame = None


def parse_frame(buf: bytes, offset: int):
    """
    解析单帧（91B）。
    返回 dict {'seq', 'ts_ms', 'ch': [(qw,qx,qy,qz,acc) ×16]} 或 None。
    """
    if offset + FRAME_LEN > len(buf):
        return None

    b = buf[offset : offset + FRAME_LEN]

    if b[0] != 0xAA or b[1] != 0x55:
        _stat['magic_err'] += 1
        return None

    # XOR 校验：bytes[2:90] XOR == b[90]
    xv = 0
    for i in range(2, 90):
        xv ^= b[i]
    if xv != b[90]:
        _stat['xor_err'] += 1
        return None

    seq = struct.unpack_from('<I', b, 2)[0]
    ts  = struct.unpack_from('<I', b, 6)[0]

    channels = []
    p = 10
    for _ in range(NUM_CH):
        iqw, iqx, iqy, iqz = struct.unpack_from('<bbbb', b, p)
        acc = b[p + 4]
        channels.append((iqw / Q_SCALE, iqx / Q_SCALE,
                          iqy / Q_SCALE, iqz / Q_SCALE, acc))
        p += 5

    return {'seq': seq, 'ts_ms': ts, 'ch': channels}


def is_live(qw, qx, qy, qz):
    """四元数模长 > 0.5 认为是有效传感器数据（非全零）。"""
    return qw*qw + qx*qx + qy*qy + qz*qz > 0.5


def on_notify(_, data: bytearray):
    global _last_seq, _drop_total, _fw_prev, _fw_fps, _rx_fps, _last_frame

    now_pc = time.perf_counter()   # PC 单调时钟，精度 ~1μs
    now_wall = time.time()

    if _stat['notify'] == 0:
        print(f"  [FIRST NOTIFY]  len={len(data)}B")
    _stat['notify'] += 1

    # ── rx_fps：PC 端到达速率（滑动窗口）────────────────────────────
    _arr_times.append(now_pc)
    while _arr_times and now_pc - _arr_times[0] > RX_WINDOW_S:
        _arr_times.popleft()
    if len(_arr_times) >= 2:
        _rx_fps = (len(_arr_times) - 1) / (_arr_times[-1] - _arr_times[0])

    # ── 解析所有帧 ───────────────────────────────────────────────
    n = len(data) // FRAME_LEN
    for i in range(n):
        f = parse_frame(bytes(data), i * FRAME_LEN)
        if f is None:
            continue
        _stat['frame'] += 1
        _last_frame = f

        # fw_fps：用固件 ts_ms 时间戳计算（消除 PC 时钟误差）
        if _fw_prev is not None:
            seq_diff = f['seq'] - _fw_prev[0]
            ts_diff_ms = f['ts_ms'] - _fw_prev[1]
            if seq_diff > 0 and ts_diff_ms > 0:
                _fw_fps = seq_diff * 1000.0 / ts_diff_ms
            # 丢帧统计
            drop = seq_diff - 1
            if drop > 0:
                _drop_total += drop
                print(f"  [DROP] seq {_fw_prev[0]}→{f['seq']}  dropped={drop}")
        _fw_prev = (f['seq'], f['ts_ms'])

        # seq 连续性（仅首次丢帧时已上方打印，此处更新追踪）
        _last_seq = f['seq']

    # ── 每 5s 打印一次 FPS 摘要 ───────────────────────────────────
    if now_wall - _stat['t_stat'] >= 5.0:
        _stat['t_stat'] = now_wall
        elapsed = now_wall - _stat['t0']
        avg_fps = _stat['frame'] / elapsed if elapsed > 0 else 0.0

        total_expected = _stat['frame'] + _drop_total
        drop_pct = _drop_total * 100.0 / total_expected if total_expected > 0 else 0.0

        live_chs = []
        if _last_frame:
            for ch, (qw, qx, qy, qz, acc) in enumerate(_last_frame['ch']):
                if is_live(qw, qx, qy, qz):
                    live_chs.append(ch)

        ts_str = time.strftime('%H:%M:%S')
        print(f"[{ts_str}]  "
              f"rx={_rx_fps:.1f}fps  "
              f"fw={_fw_fps:.1f}fps  "
              f"avg={avg_fps:.1f}fps  "
              f"drop={_drop_total}({drop_pct:.1f}%)  "
              f"live={len(live_chs)}/16  "
              f"notify={_stat['notify']}  xor_err={_stat['xor_err']}")

        # 打印最新一帧各在线通道四元数
        if _last_frame and live_chs:
            for ch in live_chs:
                qw, qx, qy, qz, acc = _last_frame['ch'][ch]
                print(f"  CH{ch:2d}: qw={qw:+.4f} qx={qx:+.4f} "
                      f"qy={qy:+.4f} qz={qz:+.4f}  acc={acc}")


async def send_cmd(client: BleakClient, cmd: str):
    await client.write_gatt_char(CMD_UUID, cmd.encode(), response=False)
    print(f"  → sent cmd '{cmd}'")


async def main(target_addr: str | None):
    # ── 扫描 / 连接 ───────────────────────────────────────────────
    if target_addr:
        print(f"直连 {target_addr} ...")
        device = target_addr
    else:
        print(f"扫描前缀 '{DEVICE_NAME}' (最长 15s)...")
        device = await BleakScanner.find_device_by_filter(
            lambda d, _: bool(d.name and d.name.startswith(DEVICE_NAME)),
            timeout=15
        )
        if device is None:
            print("ERROR: 未找到设备，请确认 ESP32-S3 已上电且蓝牙已广播。")
            return
        print(f"发现设备: {device.name}  [{device.address}]")

    async with BleakClient(device) as client:
        print(f"已连接: {client.address}  MTU={client.mtu_size}")

        # ── 读 INFO ───────────────────────────────────────────────
        try:
            info_val = await client.read_gatt_char(INFO_UUID)
            print(f"设备：{info_val.decode(errors='replace')}")
        except Exception as e:
            print(f"读 INFO 失败: {e}")

        # ── 订阅 DATA NOTIFY ──────────────────────────────────────
        await client.start_notify(DATA_UUID, on_notify)
        now = time.time()
        _stat['t0'] = now
        _stat['t_stat'] = now
        print("已订阅 DATA(0xFFE1)。按 Ctrl+C 退出，输入命令字母后回车发送。")
        print("  命令: R=reset  Z=yaw_zero  S=save_dcd  C=query_acc")
        print("  命令: F/M=cycle_report  H=cycle_i2c_freq  A=toggle_ascii  B=ble_status")
        print(f"  FPS 说明: rx=PC接收速率  fw=固件真实输出速率  avg=全程均值  drop=丢帧\n")

        # ── 主循环：接受键盘命令 ──────────────────────────────────
        loop = asyncio.get_event_loop()
        while True:
            try:
                cmd_line = await asyncio.wait_for(
                    loop.run_in_executor(None, input, ""),
                    timeout=0.2
                )
                cmd_line = cmd_line.strip()
                if cmd_line:
                    await send_cmd(client, cmd_line[0])
            except asyncio.TimeoutError:
                pass
            except (EOFError, KeyboardInterrupt):
                break


def cli():
    parser = argparse.ArgumentParser(description="BNO_HAND BLE 接收工具")
    parser.add_argument("--addr", default=None,
                        help="直接指定 BLE 地址（如 AC:A7:04:31:52:C5），跳过扫描")
    args = parser.parse_args()

    try:
        asyncio.run(main(args.addr))
    except KeyboardInterrupt:
        pass
    finally:
        elapsed = time.time() - _stat['t0'] if _stat['t0'] else 1
        avg_fps = _stat['frame'] / elapsed if elapsed > 0 else 0.0
        total_expected = _stat['frame'] + _drop_total
        drop_pct = _drop_total * 100.0 / total_expected if total_expected > 0 else 0.0
        print(f"\n── 最终统计 ──────────────────────────────────────────")
        print(f"  frames={_stat['frame']}  avg_fps={avg_fps:.1f}")
        print(f"  drop={_drop_total}({drop_pct:.1f}%)  "
              f"notify={_stat['notify']}  "
              f"xor_err={_stat['xor_err']}  magic_err={_stat['magic_err']}")
        print(f"  rx={_rx_fps:.1f}fps  fw={_fw_fps:.1f}fps  elapsed={elapsed:.1f}s")


if __name__ == "__main__":
    cli()
