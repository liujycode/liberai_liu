# BNO080_TEST v1.2

自研 ESP32-S3 PCB 传感器快速测试固件（工厂批量测试用）。

---

## 硬件要求

- 自研 ESP32-S3 PCB
- BNO080 / BNO085 传感器模块（I2C 地址 0x4B，ADDR=VCC）
- 接口 J1（默认）：CH0 / CH1 / CH2 对应 TCA1(0x70) 端口 0 / 1 / 2

---

## LED 状态说明

RGB LED DL1（共阳，低电平点亮）：

| 引脚 | 颜色 | 电阻 |
|------|------|------|
| IO47 | 红 | R42 240Ω |
| IO48 | 蓝 | R44 62Ω  |
| IO21 | 绿 | —        |

---

## 测试流程

### 1. 上电
- **红灯常亮**：正在初始化（Wire 启动 + BNO080 init）
- 初始化完成后红灯熄灭，自动进入循环状态显示

### 2. LED 循环显示

依次显示 CH0 → CH1 → CH2 状态，一轮结束后间隔 1.2s 重新循环：

| 状态 | 含义 | LED 表现 |
|------|------|----------|
| **LIVE** | 传感器正常工作 | 绿灯闪 1 下 |
| **OFFLINE** | 未插入 / 断开连接 | 红灯闪 2 下 |
| **INITFAIL** | 芯片能通信但初始化失败（BNO080 异常） | 红灯闪 3 下 |

> 每次"闪" = 亮 150ms，灭 150ms；通道之间间隔 400ms。

### 3. 结果判断示例

**3 路全部正常：**
```
绿×1 → 400ms → 绿×1 → 400ms → 绿×1 → 1.2s → 重复
```

**CH1 未插传感器：**
```
绿×1 → 400ms → 红×2 → 400ms → 绿×1 → 1.2s → 重复
```

**CH2 BNO080 芯片损坏：**
```
绿×1 → 400ms → 绿×1 → 400ms → 红×3 → 1.2s → 重复
```

---

## 热插拔

- **拔出**：连续 15 次 I2C 失败后自动切换为 OFFLINE（红×2），无需复位
- **插入**：每 800ms 自动探测一次，插入后自动初始化，成功则变为 LIVE（绿×1）

---

## 串口（可选，调试用）

波特率：**2 Mbaud**

| 命令 | 功能 |
|------|------|
| `R` | 重新初始化所有通道 |

上电后串口输出每通道状态及四元数数据，可用于排查具体异常原因。

---

## 切换测试接口

修改 `BNO080_TEST.ino` 中以下三个数组即可切换到其他接口：

```cpp
// J2
static TwoWire * const CH_BUS[]  = { &Wire,  &Wire,  &Wire  };
static const uint8_t   CH_TCA[]  = { 0x70,   0x70,   0x70   };
static const uint8_t   CH_PORT[] = { 3,      4,      5      };

// J4
static TwoWire * const CH_BUS[]  = { &Wire1, &Wire1, &Wire1 };
static const uint8_t   CH_TCA[]  = { 0x77,   0x77,   0x77   };
static const uint8_t   CH_PORT[] = { 1,      2,      3      };

// J5
static TwoWire * const CH_BUS[]  = { &Wire1, &Wire1, &Wire1 };
static const uint8_t   CH_TCA[]  = { 0x77,   0x77,   0x77   };
static const uint8_t   CH_PORT[] = { 4,      5,      6      };

// J3 跨总线
static TwoWire * const CH_BUS[]  = { &Wire,  &Wire,  &Wire1 };
static const uint8_t   CH_TCA[]  = { 0x70,   0x70,   0x77   };
static const uint8_t   CH_PORT[] = { 6,      7,      0      };
```
