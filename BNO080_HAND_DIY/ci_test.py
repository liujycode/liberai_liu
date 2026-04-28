"""
ci_test.py  — BNO_HAND BLE 连接间隔（CI）自动化测试工具
功能：
  1. 连接设备
  2. 对每个测试 CI 值发送 I<ci> 命令，等待重新协商
  3. 采集 COLLECT_SEC 秒的 rx_fps / fw_fps / drop_rate
  4. 从 INFO 特征读取固件端实际 CI
  5. 生成 CSV + 可读文本报告

依赖: pip install bleak
用法: python ci_test.py [--addr AA:BB:CC:DD:EE:FF]
"""

import asyncio
import struct
import time
import argparse
import csv
import re
from collections import deque
from dataclasses import dataclass, field
from typing import Optional, List, Tuple

from bleak import BleakClient, BleakScanner

# ── GATT UUID ─────────────────────────────────────────────────────
DEVICE_NAME = "BNO_HAND"
DATA_UUID   = "0000FFE1-0000-1000-8000-00805F9B34FB"
CMD_UUID    = "0000FFE2-0000-1000-8000-00805F9B34FB"
INFO_UUID   = "0000FFE3-0000-1000-8000-00805F9B34FB"

# ── 帧常量 ────────────────────────────────────────────────────────
FRAME_LEN = 91
NUM_CH    = 16
Q_SCALE   = 127.0

# ── 测试参数 ──────────────────────────────────────────────────────
# 待测 CI 值列表（单位：1.25ms）  0 = 恢复默认范围 6-24，排在最后
CI_VALUES    = [6, 8, 10, 12, 16, 20, 24, 40, 0]
NEGOTIATE_SEC = 8    # 发命令后等协商完成的时间（秒）
COLLECT_SEC   = 20   # 每个 CI 值的采集时长（秒）
RX_WIN_S      = 5.0  # rx_fps 滑动窗口
NUM_RUNS      = 1    # 测试循环次数（每次完整遍历所有 CI）


# ── 单次测试结果 ──────────────────────────────────────────────────
@dataclass
class CIResult:
    requested_ci:  int           # 请求的 CI 单位值（0=默认范围）
    firmware_ci:   int   = 0     # 固件端读到的实际 CI（从 INFO 解析）
    rx_fps:        float = 0.0   # 接收 FPS（滑动窗口均值）
    fw_fps:        float = 0.0   # 固件真实输出 FPS（seq+ts_ms 计算）
    drop_pct:      float = 0.0   # 丢帧率（%）
    drop_total:    int   = 0     # 累计丢帧数
    frame_total:   int   = 0     # 收到帧数
    xor_err:       int   = 0     # XOR 校验错误数
    note:          str   = ""    # 备注（如 Windows 是否接受）


# ── 采集状态（全局，供 on_notify 回调访问）────────────────────────
class Collector:
    def __init__(self):
        self.reset()

    def reset(self):
        self.frames      = 0
        self.xor_err     = 0
        self.drop_total  = 0
        self.fw_prev: Optional[Tuple[int, int]] = None
        self.arr_times: deque = deque()
        self.fw_fps_list: List[float] = []
        self.rx_fps      = 0.0
        self.fw_fps      = 0.0
        self._t0         = None

    def on_notify(self, _, data: bytearray):
        now_pc = time.perf_counter()
        if self._t0 is None:
            self._t0 = now_pc

        # rx_fps
        self.arr_times.append(now_pc)
        while self.arr_times and now_pc - self.arr_times[0] > RX_WIN_S:
            self.arr_times.popleft()
        if len(self.arr_times) >= 2:
            self.rx_fps = (len(self.arr_times) - 1) / (
                self.arr_times[-1] - self.arr_times[0])

        n = len(data) // FRAME_LEN
        for i in range(n):
            f = self._parse(bytes(data), i * FRAME_LEN)
            if f is None:
                continue
            self.frames += 1
            if self.fw_prev is not None:
                sd = f[0] - self.fw_prev[0]
                td = f[1] - self.fw_prev[1]
                if sd > 0 and td > 0:
                    self.fw_fps = sd * 1000.0 / td
                    self.fw_fps_list.append(self.fw_fps)
                drop = sd - 1
                if drop > 0:
                    self.drop_total += drop
            self.fw_prev = (f[0], f[1])

    def _parse(self, buf: bytes, off: int):
        if off + FRAME_LEN > len(buf):
            return None
        b = buf[off: off + FRAME_LEN]
        if b[0] != 0xAA or b[1] != 0x55:
            return None
        xv = 0
        for i in range(2, 90):
            xv ^= b[i]
        if xv != b[90]:
            self.xor_err += 1
            return None
        seq  = struct.unpack_from('<I', b, 2)[0]
        ts   = struct.unpack_from('<I', b, 6)[0]
        return seq, ts

    def get_result(self, requested_ci: int, firmware_ci: int) -> CIResult:
        total_expected = self.frames + self.drop_total
        drop_pct = (self.drop_total * 100.0 / total_expected
                    if total_expected > 0 else 0.0)
        fw_fps_avg = (sum(self.fw_fps_list) / len(self.fw_fps_list)
                      if self.fw_fps_list else 0.0)
        note = ""
        if requested_ci != 0:
            if firmware_ci == requested_ci:
                note = "Windows 接受"
            elif firmware_ci > 0:
                note = f"Windows 改为 CI={firmware_ci}({firmware_ci*1.25:.1f}ms)"
            else:
                note = "未读到实际 CI"
        else:
            note = "默认范围 6-24"
        return CIResult(
            requested_ci=requested_ci,
            firmware_ci=firmware_ci,
            rx_fps=self.rx_fps,
            fw_fps=fw_fps_avg,
            drop_pct=drop_pct,
            drop_total=self.drop_total,
            frame_total=self.frames,
            xor_err=self.xor_err,
            note=note,
        )


def parse_info_ci(info_str: str) -> int:
    """从 INFO 特征字符串解析实际 CI。
    格式：fps=33.2 CI=24/30.0ms notify=505 poll=180us
    返回 CI 单位值，失败返回 0。
    """
    m = re.search(r'CI=(\d+)/', info_str)
    return int(m.group(1)) if m else 0


async def send_cmd(client: BleakClient, cmd: str):
    await client.write_gatt_char(CMD_UUID, cmd.encode(), response=False)


async def run_ci_test(client: BleakClient, ci: int,
                      collector: Collector) -> CIResult:
    ci_str  = str(ci) if ci > 0 else "0 (default)"
    ci_ms   = f"{ci * 1.25:.1f}ms" if ci > 0 else "7.5-30ms (range)"
    print(f"\n{'='*50}")
    print(f"  测试 CI = {ci_str}  ({ci_ms})")
    print(f"{'='*50}")

    # 发送 CI 命令
    cmd = f"I{ci}"
    print(f"  → 发送命令 '{cmd}'")
    await send_cmd(client, cmd)

    # 等待协商
    print(f"  等待 {NEGOTIATE_SEC}s 协商...")
    await asyncio.sleep(NEGOTIATE_SEC)

    # 读取 INFO 获取实际 CI
    firmware_ci = 0
    try:
        info_val = await client.read_gatt_char(INFO_UUID)
        info_str = info_val.decode(errors='replace')
        firmware_ci = parse_info_ci(info_str)
        print(f"  INFO: {info_str.strip()}")
        if firmware_ci:
            print(f"  实际 CI = {firmware_ci} ({firmware_ci * 1.25:.1f}ms)  "
                  f"→ {'与请求一致' if firmware_ci == ci else '与请求不同'}")
    except Exception as e:
        print(f"  读 INFO 失败: {e}")

    # 重置采集器，开始采集
    collector.reset()
    print(f"  采集 {COLLECT_SEC}s...")
    t_end = time.time() + COLLECT_SEC
    while time.time() < t_end:
        remaining = t_end - time.time()
        bar_w  = 30
        filled = int((COLLECT_SEC - remaining) / COLLECT_SEC * bar_w)
        bar    = '#' * filled + '-' * (bar_w - filled)
        print(f"\r  [{bar}] {COLLECT_SEC - remaining:.0f}/{COLLECT_SEC}s  "
              f"rx={collector.rx_fps:.1f}fps  fw={collector.fw_fps:.1f}fps  "
              f"drop={collector.drop_total}",
              end='', flush=True)
        await asyncio.sleep(0.5)
    print()  # 换行

    result = collector.get_result(ci, firmware_ci)
    print(f"  结果: rx={result.rx_fps:.1f}fps  fw={result.fw_fps:.1f}fps  "
          f"drop={result.drop_pct:.1f}%({result.drop_total})  "
          f"frames={result.frame_total}  [{result.note}]")
    return result


async def main(target_addr: Optional[str]):
    # ── 扫描/连接 ────────────────────────────────────────────────
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
            print("ERROR: 未找到设备")
            return
        print(f"发现设备: {device.name}  [{device.address}]")

    async with BleakClient(device) as client:
        print(f"已连接: {client.address}  MTU={client.mtu_size}")

        # 读 INFO 获取版本
        try:
            info_val = await client.read_gatt_char(INFO_UUID)
            info_str = info_val.decode(errors='replace')
            print(f"设备信息: {info_str}")
        except Exception as e:
            print(f"读 INFO 失败: {e}")

        # 订阅 DATA notify
        collector = Collector()
        await client.start_notify(DATA_UUID, collector.on_notify)

        # 等首包到达
        print("\n等待首包...")
        for _ in range(50):
            await asyncio.sleep(0.1)
            if collector.frames > 0:
                break
        if collector.frames == 0:
            print("WARNING: 10s 内未收到任何帧，请检查连接")

        # ── 多轮循环测试 ──────────────────────────────────────────
        all_runs: List[List[CIResult]] = []
        for run_idx in range(1, NUM_RUNS + 1):
            if NUM_RUNS > 1:
                print(f"\n{'='*60}")
                print(f"  第 {run_idx}/{NUM_RUNS} 轮测试")
                print(f"{'='*60}")
            else:
                print()

            run_results: List[CIResult] = []
            for ci in CI_VALUES:
                try:
                    r = await run_ci_test(client, ci, collector)
                    run_results.append(r)
                except Exception as e:
                    print(f"  测试 CI={ci} 出错: {e}")

            all_runs.append(run_results)

            # 轮次间隔（最后一轮不等待）
            if run_idx < NUM_RUNS:
                gap = 5
                print(f"\n  第 {run_idx} 轮结束，{gap}s 后开始第 {run_idx+1} 轮...")
                await asyncio.sleep(gap)

        # ── 恢复默认 CI ──────────────────────────────────────────
        try:
            await send_cmd(client, "I0")
        except Exception:
            pass

        # ── 打印/保存报告 ─────────────────────────────────────────
        print_report(all_runs[0])
        if NUM_RUNS > 1:
            print_aggregate_report(all_runs)
        save_report(all_runs)


def _agg(vals: List[float]) -> Tuple[float, float]:
    n = len(vals)
    mean = sum(vals) / n
    std  = (sum((v - mean)**2 for v in vals) / n) ** 0.5 if n > 1 else 0.0
    return mean, std


def print_aggregate_report(all_runs: List[List[CIResult]]):
    ts      = time.strftime('%Y-%m-%d %H:%M:%S')
    sep     = '=' * 78
    n_runs  = len(all_runs)
    print(f"\n{sep}")
    print(f"  {n_runs} 轮聚合报告  —  {ts}")
    print(sep)
    hdr = (f"{'请求CI':>6}  {'请求ms':>7}  {'实际CI':>6}  "
           f"{'rx_fps μ±σ':>14}  {'fw_fps μ±σ':>14}  "
           f"{'drop% μ±σ':>12}  备注")
    print(hdr)
    print('-' * 78)

    for ci_idx in range(len(all_runs[0])):
        rs  = [run[ci_idx] for run in all_runs]          # 该 CI 的 n_runs 次结果
        r0  = rs[0]
        req_ms = f"{r0.requested_ci * 1.25:.1f}" if r0.requested_ci else "range"

        rx_vals = [r.rx_fps    for r in rs]
        fw_vals = [r.fw_fps    for r in rs]
        dr_vals = [r.drop_pct  for r in rs]

        rx_m, rx_s = _agg(rx_vals)
        fw_m, fw_s = _agg(fw_vals)
        dr_m, dr_s = _agg(dr_vals)

        note = r0.note
        print(f"{r0.requested_ci if r0.requested_ci else 'range':>6}  "
              f"{req_ms:>7}  "
              f"{r0.firmware_ci if r0.firmware_ci else '?':>6}  "
              f"{rx_m:>6.1f}±{rx_s:<5.1f}  "
              f"{fw_m:>6.1f}±{fw_s:<5.1f}  "
              f"{dr_m:>5.1f}±{dr_s:<5.1f}  "
              f"{note}")
    print(sep)

    # 最优 CI
    usable = [r for rs in all_runs
                   for r in rs
                   if r.frame_total > 0 and r.requested_ci != 0]
    if usable:
        best = min(usable, key=lambda r: (r.drop_pct, -r.rx_fps))
        print(f"\n  最优 CI = {best.requested_ci} ({best.requested_ci*1.25:.1f}ms)  "
              f"rx={best.rx_fps:.1f}fps  drop={best.drop_pct:.1f}%")
    print()


def print_report(results: List[CIResult]):
    ts = time.strftime('%Y-%m-%d %H:%M:%S')
    sep = '=' * 78
    print(f"\n{sep}")
    print(f"  BNO_HAND CI 测试报告  —  {ts}")
    print(sep)
    hdr = (f"{'请求CI':>6}  {'请求ms':>7}  {'实际CI':>6}  {'实际ms':>7}  "
           f"{'rx_fps':>7}  {'fw_fps':>7}  {'drop%':>6}  {'帧数':>6}  备注")
    print(hdr)
    print('-' * 78)
    for r in results:
        req_ms = f"{r.requested_ci * 1.25:.1f}" if r.requested_ci else "range"
        act_ms = f"{r.firmware_ci * 1.25:.1f}" if r.firmware_ci else "?"
        print(f"{r.requested_ci if r.requested_ci else 'range':>6}  "
              f"{req_ms:>7}  "
              f"{r.firmware_ci if r.firmware_ci else '?':>6}  "
              f"{act_ms:>7}  "
              f"{r.rx_fps:>7.1f}  "
              f"{r.fw_fps:>7.1f}  "
              f"{r.drop_pct:>5.1f}%  "
              f"{r.frame_total:>6}  "
              f"{r.note}")
    print(sep)

    # 找最优（drop_pct 最低且 rx_fps 最高）
    usable = [r for r in results if r.frame_total > 0 and r.requested_ci != 0]
    if usable:
        best = min(usable, key=lambda r: (r.drop_pct, -r.rx_fps))
        print(f"\n  最优 CI = {best.requested_ci} ({best.requested_ci*1.25:.1f}ms)  "
              f"rx={best.rx_fps:.1f}fps  drop={best.drop_pct:.1f}%")
    print()


def save_report(all_runs: List[List[CIResult]]):
    ts      = time.strftime('%Y%m%d_%H%M%S')
    n_runs  = len(all_runs)
    csv_fn  = f"ci_test_{ts}.csv"
    txt_fn  = f"ci_test_{ts}.txt"

    # CSV（每行对应一轮测试的每个 CI）
    fields = ['run', 'ci_idx', 'requested_ci', 'req_ms', 'firmware_ci',
              'actual_ms', 'rx_fps', 'fw_fps', 'drop_pct', 'drop_total',
              'frame_total', 'xor_err', 'note']
    with open(csv_fn, 'w', newline='', encoding='utf-8-sig') as f:
        w = csv.DictWriter(f, fieldnames=fields)
        w.writeheader()
        for run_idx, run in enumerate(all_runs, 1):
            for ci_idx, r in enumerate(run, 1):
                w.writerow({
                    'run':          run_idx,
                    'ci_idx':       ci_idx,
                    'requested_ci': r.requested_ci,
                    'req_ms':       f"{r.requested_ci * 1.25:.2f}" if r.requested_ci else "range",
                    'firmware_ci':  r.firmware_ci,
                    'actual_ms':    f"{r.firmware_ci * 1.25:.2f}" if r.firmware_ci else "",
                    'rx_fps':       f"{r.rx_fps:.2f}",
                    'fw_fps':       f"{r.fw_fps:.2f}",
                    'drop_pct':     f"{r.drop_pct:.2f}",
                    'drop_total':   r.drop_total,
                    'frame_total':  r.frame_total,
                    'xor_err':      r.xor_err,
                    'note':         r.note,
                })

    # CSV 聚合均值（仅多轮时生成）
    agg_csv_fn = None
    if n_runs > 1:
        agg_csv_fn = f"ci_test_{ts}_agg.csv"
        agg_fields = ['requested_ci', 'req_ms', 'firmware_ci',
                      'rx_fps_mean', 'rx_fps_std', 'fw_fps_mean', 'fw_fps_std',
                      'drop_pct_mean', 'drop_pct_std', 'drop_total_sum', 'note']
        with open(agg_csv_fn, 'w', newline='', encoding='utf-8-sig') as f:
            w = csv.DictWriter(f, fieldnames=agg_fields)
            w.writeheader()
            for ci_idx in range(len(all_runs[0])):
                rs   = [run[ci_idx] for run in all_runs]
                r0   = rs[0]
                rx_m, rx_s = _agg([r.rx_fps   for r in rs])
                fw_m, fw_s = _agg([r.fw_fps   for r in rs])
                dr_m, dr_s = _agg([r.drop_pct for r in rs])
                w.writerow({
                    'requested_ci':  r0.requested_ci,
                    'req_ms':        f"{r0.requested_ci * 1.25:.2f}" if r0.requested_ci else "range",
                    'firmware_ci':   r0.firmware_ci,
                    'rx_fps_mean':   f"{rx_m:.2f}",
                    'rx_fps_std':    f"{rx_s:.2f}",
                    'fw_fps_mean':   f"{fw_m:.2f}",
                    'fw_fps_std':    f"{fw_s:.2f}",
                    'drop_pct_mean': f"{dr_m:.2f}",
                    'drop_pct_std':  f"{dr_s:.2f}",
                    'drop_total_sum': sum(r.drop_total for r in rs),
                    'note':          r0.note,
                })

    # 文本（最后一轮的 print_report 输出 + 聚合部分）
    with open(txt_fn, 'w', encoding='utf-8') as f:
        import io, sys
        old_stdout = sys.stdout
        sys.stdout = io.StringIO()
        print_report(all_runs[-1])
        if n_runs > 1:
            print_aggregate_report(all_runs)
        out = sys.stdout.getvalue()
        sys.stdout = old_stdout
        f.write(out)

    files = [csv_fn]
    if agg_csv_fn:
        files.append(agg_csv_fn)
    files.append(txt_fn)
    print(f"  报告已保存: {'  /  '.join(files)}")


def cli():
    global CI_VALUES, COLLECT_SEC, NEGOTIATE_SEC, NUM_RUNS
    parser = argparse.ArgumentParser(description="BNO_HAND CI 自动测试工具")
    parser.add_argument("--addr", default=None,
                        help="直接指定 BLE 地址（跳过扫描）")
    parser.add_argument("--ci", nargs='+', type=int, default=None,
                        help="自定义 CI 测试列表，例如 --ci 6 12 24")
    parser.add_argument("--collect", type=int, default=COLLECT_SEC,
                        help=f"每个 CI 采集秒数（默认 {COLLECT_SEC}）")
    parser.add_argument("--negotiate", type=int, default=NEGOTIATE_SEC,
                        help=f"协商等待秒数（默认 {NEGOTIATE_SEC}）")
    parser.add_argument("--runs", type=int, default=NUM_RUNS,
                        help=f"测试循环次数（默认 {NUM_RUNS}，每次完整遍历所有 CI）")
    args = parser.parse_args()
    if args.ci:
        CI_VALUES = args.ci
    COLLECT_SEC   = args.collect
    NEGOTIATE_SEC = args.negotiate
    NUM_RUNS      = args.runs

    print(f"CI 测试列表: {CI_VALUES}")
    print(f"采集时长: {COLLECT_SEC}s/个  协商等待: {NEGOTIATE_SEC}s  循环次数: {NUM_RUNS}\n")

    try:
        asyncio.run(main(args.addr))
    except KeyboardInterrupt:
        print("\n用户中断")


if __name__ == "__main__":
    cli()
