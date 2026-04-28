# RK3588 BLE 帧率瓶颈分析

## 背景

固件 `BNO080_HAND_DIY.ino` v2.38 输出稳定 **165 fps**（BNO080 每 6ms 轮询），但 BLE 传输到主机后帧率大幅下降。Windows BLE 实测约 95 fps（丢帧率 41%），RK3588 上也观测到类似现象。

本分析针对 **RK3588 + BlueZ** 的 BLE 接收端性能瓶颈。

---

## 一、BLE 帧传输路径

```
BNO080传感器 → ESP32-S3固件(165fps) → BLE空中传输 → RK3588接收 → BlueZ → 应用层
```

固件输出 165 fps 不变，问题全在 BLE 传输 + 主机接收端。

---

## 二、瓶颈层级分析

### 2.1 协议层（BLE Physical / ATT）

**ATT MTU 是最关键限制。**

BLE ATT 协议每个数据包最大有效载荷：

| ATT MTU | 有效载荷（MTU - ATT Header） | 固件帧 91B 所需包数 |
|---------|----------------------------|-------------------|
| 23（默认） | ≈ 20 B | **5 包** |
| 185 | ≈ 181 B | 1 包 |
| 512 | ≈ 508 B | 1 包 |

```python
# 理论吞吐量计算
CI_ms = 7.5  # 连接间隔 7.5ms
CI_hz = 1000 / CI_ms  # ≈ 133Hz

MTU = 23
payload_per_packet = MTU - 3  # ATT header = 3B

# 若每次连接只发 1 个包
max_fps = CI_hz * 1  # ≈ 133 fps（但受 payload 限制）
max_throughput = max_fps * 91  # 91B/帧
# = 133 × 91 ≈ 12 KB/s ← 这已经超过 20B/packet 的极限
# 实际上 MTU=23 时，91B 需要分 5 包，在 CI 内发完 5 包的概率很低
```

**结论：若 ATT MTU=23，固件 91B 帧需要拆成 5 个 ATT 包；在 CI=7.5ms 内发完 5 包的概率极低，导致大量丢帧或降帧。**

### 2.2 BlueZ 架构层

BlueZ 是 Linux 官方蓝牙协议栈，运行在用户空间：

```
BLE硬件 → HCI内核驱动(bluetooth.ko) → bluetoothd(用户空间) → DBus → 应用程序
         ↑                           ↑
      内核态                       用户态（上下文切换）
```

**每收到一个 BLE 通知，至少经过 4 个软件层：**
1. HCI 内核驱动解析 BLE 物理包
2. bluetoothd 解析 ATT 协议
3. DBus 消息投递
4. 应用程序读取数据

在 RK3588（Cortex-A76 @ 2.4GHz）上，bluetoothd 作为普通优先级用户进程，高帧率下可能被调度延迟，导致接收缓冲溢出。

### 2.3 RK3588 硬件实现

RK3588 通常搭配 COMBO 芯片（AP6275 / AP6398S）：

```
┌─────────────┐
│  RK3588 SoC │
│  (Cortex-A76)│
└──────┬──────┘
       │ UART HCI (4 Mbps)
┌──────▼──────┐
│ AP6275       │ ← WiFi + BLE 共用射频
│ COMBO 芯片   │
└─────────────┘
```

**潜在问题：**
- WiFi + BLE 共用天线，高负载时射频干扰
- UART HCI 带宽 4 Mbps 本身足够，但 COMBO 芯片固件对 BLE 通知分包有限制
- AP6275 BLE 固件可能限制每连接间隔最大包数（某些芯片固件限制为 4 包/interval）

---

## 三、诊断方法

### 3.1 btmon 抓包分析（必做）

在 RK3588 上用 BlueZ 内置工具抓取 HCI 层数据：

```bash
# 1. 找到蓝牙设备
hciconfig
# 输出示例：hci0: ... State: UP

# 2. 启动抓包
sudo btmon -i hci0 -w ble_capture.bts

# 3. 连接设备后运行测试，然后 Ctrl+C 停止

# 4. 分析抓包文件
sudo btmon -i hci0 -r ble_capture.bts
```

**关键观察项：**

| 观察项 | 正常值 | 异常指示 |
|--------|--------|---------|
| ATT MTU Exchange | 应为 512 或 247 | 若为 23 → 严重瓶颈 |
| Connection Interval | 应为 7.5ms | 若为 15-30ms → 帧率腰斩 |
| ATT Write Request / Notification | 每帧 91B | 若分多个包 → MTU 问题 |
| HCI Number of Completed Packets | 应与固件输出匹配 | 若远低于 165 → 主机端丢包 |

### 3.2 bluetoothctl 连接参数

```bash
# 连接后查看协商的连接参数
bluetoothctl
[bluetooth]# connect AC:A7:04:31:53:01
[bluetooth]# info AC:A7:04:31:53:01
```

关键字段：
- `Connected`: yes
- `Connection Parameters`：interval / latency / timeout

### 3.3 BlueZ 日志级别

```bash
# 提高 bluetoothd 日志级别，查看 ATT 层处理情况
sudo killall bluetoothd
sudo /usr/lib/bluetooth/bluetoothd -d -n 2>&1 | grep -E "(ATT|MTU|interval)"
```

### 3.4 测试 ATT MTU 协商脚本

```python
# test_mtu.py — 测试 RK3588 上 BlueZ 的 MTU 协商能力
import subprocess

result = subprocess.run(
    ["hcitool", "lesc"],
    capture_output=True, text=True
)
# 或使用 pybluez / bleak 的 MTU 协商接口

# 关键：用 bleak 连接后检查 negotiated MTU
import asyncio
from bleak import BleakClient

async def test_mtu():
    async with BleakClient("AC:A7:04:31:53:01") as client:
        # bleak 默认会协商 MTU，但可能受 BlueZ 版本限制
        mtu = client.remote_idx  # 检查实际协商 MTU
        print(f"Negotiated MTU: {mtu}")
```

---

## 四、可能的根因与处理措施

### 措施 1：强制协商大 ATT MTU（最优先）

**问题**：BlueZ 默认 MTU=23，91B 帧需 5 个包才能传完。

**处理**：固件在连接后主动发送 GATT MTU Exchange Request：
```cpp
// 在 onConnect() 或第一条通知前
// ESP32 NimBLE 支持设置本地 MTU
nimble/host/src/ble_gattc.c 或在连接参数中包含 MTU suggestion

// 实际上固件端 NimBLE 默认会协商，但 BlueZ 可能拒绝更大的 MTU
// 需要在固件侧也请求较大 MTU
```

BlueZ 侧需检查是否限制了最大 MTU：
```bash
# 修改 BlueZ 配置
sudo nano /etc/bluetooth/main.conf
[GATT]
  # Default MTU = 517
  # 或者设为 0 让系统自适应
  DefaultMTU=517
```

重启 bluetoothd：
```bash
sudo systemctl restart bluetooth
```

### 措施 2：减小固件帧大小（备选）

若 MTU 协商失败，固件帧从 91B 减至 20B（适配 MTU=23）：
- 只发关键字段（四元数 4×4B = 16B + 2B header = 18B）
- 精度不变（int8 → int8）
- 理论帧率上限提升到 665 fps（远超 BLE 实际上限）

但这会破坏与现有上位机的兼容性，需同步更新 IMU_plotter_all.py。

### 措施 3：BlueZ 进程优先级提升

```bash
# 将 bluetoothd 设为实时优先级（高优先级调度）
sudo systemctl edit bluetooth
```

添加：
```ini
[Service]
Nice=-10
CPUSchedulingPolicy=rr
CPUSchedulingPriority=50
```

```bash
sudo systemctl daemon-reload
sudo systemctl restart bluetooth
```

### 措施 4：检查 COMBO 芯片固件更新

AP6275 / AP6398S 的 BLE 固件可能有版本更新，带来更好的多连接间隔支持。

查看当前固件版本：
```bash
# 通过 HCI 命令读取本地芯片信息
sudo hciconfig -i hci0
# 查看芯片型号和固件版本字符串
```

---

## 五、理论帧率上限估算

| 条件 | 理论上限 | 说明 |
|------|---------|------|
| MTU=23, CI=7.5ms, 1包/interval | ~133 fps | 不可达，91B 需要 5 包 |
| MTU=23, CI=7.5ms, 4包/interval | ~532 fps | 协议层可达，但主机处理跟不上 |
| MTU=185, CI=7.5ms, 1包/interval | ~133 fps | 每帧一个包，效率高 |
| MTU=512, CI=7.5ms, 1包/interval | ~133 fps | 最优配置 |

**结论**：RK3588 BLE 帧率上限 ≈ **133 fps**（受 CI=7.5ms 限制）。实测如果远低于此，问题在 MTU 或 BlueZ 处理。

---

## 六、下一步行动计划

| 步骤 | 操作 | 预期结果 |
|------|------|---------|
| 1 | RK3588 上用 `btmon` 抓包，确认 ATT MTU 实际协商值 | MTU=23 → 根因确认；MTU≥185 → 继续排查 |
| 2 | 若 MTU=23，修改 `/etc/bluetooth/main.conf` 设置 `DefaultMTU=517` | 重启后重新连接，再次抓包验证 |
| 3 | 运行 ci_test.py 收集 `rx_fps`，对比 MTU 前后的帧率差异 | 帧率显著提升 → 修复成功 |
| 4 | 若 MTU 已≥185 但帧率仍低，排查 BlueZ 调度优先级 | 提高 Nice 值后测试 |

---

## 七、相关文件

| 文件 | 说明 |
|------|------|
| `BNO080_HAND_DIY.ino` v2.38 | 固件，输出 165fps，帧长 91B |
| `ci_test.py` | BLE CI 测试脚本，--runs 支持多轮统计 |
| `CHANGELOG.md` | 固件版本变更记录 |
| `/etc/bluetooth/main.conf` | BlueZ 配置文件（RK3588 上需确认路径） |
| `ble_capture.bts` | btmon 抓包文件（诊断时生成） |