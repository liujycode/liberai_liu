"""
bno_hand_ble.py  — BNO_HAND v2.0 PC BLE 接收脚本
依赖: pip install bleak
用法: python bno_hand_ble.py [--addr AA:BB:CC:DD:EE:FF]
"""

import asyncio
import struct
import time
import argparse

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
FRAMES_PER_NOTIFY = 1       # v2.11固件：每帧一个 notify（同步修改）

# ── 全局统计 ──────────────────────────────────────────────────────
_stat = {
    'notify': 0, 'frame': 0, 'xor_err': 0, 'magic_err': 0,
    'frame_prev': 0, 't0': 0.0, 't_stat': 0.0,
}
_last_seq = -1


def parse_frame(buf: bytes, offset: int):
    """
    解析单帧（155B）。
    返回 dict {'seq', 'ts_ms', 'ch': [(qw,qx,qy,qz,acc) ×16]} 或 None。
    """
    if offset + FRAME_LEN > len(buf):
        return None

    b = buf[offset : offset + FRAME_LEN]

    # 魔数
    if b[0] != 0xAA or b[1] != 0x55:
        _stat['magic_err'] += 1
        return None

    # XOR 校验（字节 [2:90] 的 XOR == b[90]）
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
        iqw, iqx, iqy, iqz = struct.unpack_from('<bbbb', b, p)   # int8 有符号
        acc = b[p + 4]
        channels.append((iqw / Q_SCALE, iqx / Q_SCALE,
                          iqy / Q_SCALE, iqz / Q_SCALE, acc))
        p += 5

    return {'seq': seq, 'ts_ms': ts, 'ch': channels}


def is_live(qw, qx, qy, qz):
    """四元数模长 > 0.5 认为是有效传感器数据（非全零）。"""
    return qw*qw + qx*qx + qy*qy + qz*qz > 0.5


def on_notify(_, data: bytearray):
    global _last_seq
    if _stat['notify'] == 0:
        print(f"  [FIRST NOTIFY]  len={len(data)}B")   # 收到第一包立即打印
    _stat['notify'] += 1

    n = len(data) // FRAME_LEN
    for i in range(n):
        f = parse_frame(bytes(data), i * FRAME_LEN)
        if f is None:
            continue
        _stat['frame'] += 1

        # seq 连续性检查
        if _last_seq >= 0 and f['seq'] != _last_seq + 1:
            drop = f['seq'] - _last_seq - 1
            if drop > 0:
                print(f"  [SEQ GAP] {_last_seq} → {f['seq']}  dropped={drop}")
        _last_seq = f['seq']

    # 每 100 帧打印一次摘要
    if _stat['frame'] - _stat['frame_prev'] >= 100:
        _stat['frame_prev'] = _stat['frame']
        elapsed = time.time() - _stat['t0']
        fps = _stat['frame'] / elapsed if elapsed > 0 else 0

        # 最新一帧的 live 通道
        last_f = None
        for i in range(n - 1, -1, -1):
            last_f = parse_frame(bytes(data), i * FRAME_LEN)
            if last_f:
                break

        live_chs = []
        if last_f:
            for ch, (qw, qx, qy, qz, acc) in enumerate(last_f['ch']):
                if is_live(qw, qx, qy, qz):
                    live_chs.append(ch)

        print(f"[seq={f['seq']:7d}  ts={f['ts_ms']:8d}ms]  "
              f"fps={fps:6.1f}  notify={_stat['notify']}  "
              f"xor_err={_stat['xor_err']}  "
              f"live={len(live_chs)}/16 {live_chs}")

        if last_f and live_chs:
            for ch in live_chs:
                qw, qx, qy, qz, acc = last_f['ch'][ch]
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
        print(f"扫描 '{DEVICE_NAME}' (最长 15s)...")
        device = await BleakScanner.find_device_by_name(DEVICE_NAME, timeout=15)
        if device is None:
            print("ERROR: 未找到设备，请确认 ESP32-S3 已上电且蓝牙已广播。")
            return
        print(f"发现设备: {device.name}  [{device.address}]")

    async with BleakClient(device) as client:
        print(f"已连接: {client.address}  MTU={client.mtu_size}")

        # ── 读 INFO ───────────────────────────────────────────────
        try:
            info_val = await client.read_gatt_char(INFO_UUID)
            print(f"INFO: {info_val.decode(errors='replace')}")
        except Exception as e:
            print(f"读 INFO 失败: {e}")

        # ── 订阅 DATA NOTIFY ──────────────────────────────────────
        await client.start_notify(DATA_UUID, on_notify)
        _stat['t0'] = time.time()
        _stat['t_stat'] = _stat['t0']
        print("已订阅 DATA(0xFFE1)。按 Ctrl+C 退出，输入命令字母后回车发送。\n")
        print("  命令: R=reset  Z=yaw_zero  S=save_dcd  C=query_acc")
        print("        F/M=cycle_report  H=cycle_i2c_freq  A=toggle_ascii  B=ble_status\n")

        # ── 主循环：接受键盘命令 ──────────────────────────────────
        loop = asyncio.get_event_loop()
        t_hb = _stat['t0']   # 心跳计时
        while True:
            try:
                # 非阻塞读键盘（每 0.2s 检查一次）
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

            # 每 3s 打印一次心跳，方便确认是否收到通知
            now = time.time()
            if now - t_hb >= 3.0:
                t_hb = now
                print(f"  [HB]  notify={_stat['notify']}  frame={_stat['frame']}  "
                      f"xor_err={_stat['xor_err']}  magic_err={_stat['magic_err']}")


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
        fps = _stat['frame'] / elapsed
        print(f"\n统计: frames={_stat['frame']}  fps={fps:.1f}  "
              f"notify={_stat['notify']}  "
              f"xor_err={_stat['xor_err']}  magic_err={_stat['magic_err']}")


if __name__ == "__main__":
    cli()
