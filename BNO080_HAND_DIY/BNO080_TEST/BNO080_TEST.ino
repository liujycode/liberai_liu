/*!
 * BNO080_TEST.ino  v1.2  2026-04-07
 * 自研 ESP32-S3 PCB — 传感器快速测试固件
 * 热插拔检测 + RGB LED 状态指示（工厂批量测试用）
 *
 * -- LED 状态指示（DL1 共阳 RGB，IO21=Green）--
 *   循环依次显示 CH0 → CH1 → CH2 状态：
 *     LIVE     : 绿灯闪 1 下（正常）
 *     OFFLINE  : 红灯闪 2 下（未插入 / 断连）
 *     INITFAIL : 红灯闪 3 下（芯片异常）
 *   一轮结束后间隔 1.2s 重新循环
 *
 * -- 测试接口（默认 J1：CH0/CH1/CH2）-------------------------------
 *   修改 CH_BUS / CH_TCA / CH_PORT 可切换到其他接口：
 *   J2: {&Wire, &Wire, &Wire}   / {0x70,0x70,0x70} / {3,4,5}
 *   J4: {&Wire1,&Wire1,&Wire1}  / {0x77,0x77,0x77} / {1,2,3}
 *   J5: {&Wire1,&Wire1,&Wire1}  / {0x77,0x77,0x77} / {4,5,6}
 *   J3 跨总线: {&Wire,&Wire,&Wire1} / {0x70,0x70,0x77} / {6,7,0}
 *
 * -- 自研 ESP32-S3 PCB 硬件 ----------------------------------------
 *   Bus A: GPIO8(SDA)  / GPIO9(SCL)  → TCA1(0x70 A0/A1/A2=GND) → CH0-7
 *   Bus B: GPIO13(SDA) / GPIO14(SCL) → TCA2(0x77 A0/A1/A2=VCC) → CH8-15
 *   BNO080 I2C 地址：0x4B（ADDR=VCC）
 *   LED: IO47→R42(240Ω)→Red  IO48→R44(62Ω)→Blue  IO21→Green
 *
 * -- 串口命令（2Mbaud，可选）---------------------------------------
 *   R  重新初始化所有通道
 */

#include <Wire.h>
#include "SparkFun_BNO080_Arduino_Library.h"

#define FW_VER  "v1.2"

// ── 硬件引脚（自研 PCB）──────────────────────────────────────
#define PIN_SDA_A   8
#define PIN_SCL_A   9
#define PIN_SDA_B   13
#define PIN_SCL_B   14
#define BNO_ADDR    0x4B
#define I2C_FREQ    1000000UL

// ── RGB LED（共阳，低电平点亮）──────────────────────────────
// 实测核查（LED_TEST v1.0 2026-04-21）：原理图标注全部错误，三通道颜色对调
#define LED_R   21    // IO21 → R42(240Ω) → 红（实测）
#define LED_G   48    // IO48 → R43(62Ω)  → 绿（实测）
#define LED_B   47    // IO47 → R44(62Ω)  → 蓝（实测）

// ── 测试接口配置（默认 J1：CH0/CH1/CH2）────────────────────
#define TEST_CH  3
static TwoWire * const CH_BUS[TEST_CH]  = { &Wire,  &Wire,  &Wire  };
static const uint8_t   CH_TCA[TEST_CH]  = { 0x70,   0x70,   0x70   };
static const uint8_t   CH_PORT[TEST_CH] = { 0,      1,      2      };

// ── 运行参数 ─────────────────────────────────────────────────
#define OFFLINE_THRESH     15     // 连续 I2C 失败次数 → OFFLINE
#define PROBE_INTERVAL_MS  800    // 非 LIVE 通道探测周期 ms
#define BNO_REPORT_MS      5      // BNO 报告周期（~200Hz）

// ── LED 时间参数（ms）───────────────────────────────────────
#define LED_T_ON      150U   // 每次闪亮持续
#define LED_T_OFF     150U   // 每次闪灭间隔
#define LED_T_CH_GAP  400U   // 通道间隔
#define LED_T_LONG    1200U  // 一轮结束后的长间隔

// ── 通道状态 ─────────────────────────────────────────────────
enum ChState : uint8_t { ST_OFFLINE, ST_INITFAIL, ST_LIVE };
static const char * const ST_STR[] = { "OFFLINE ", "INITFAIL", "LIVE    " };

struct Chan {
    ChState  state;
    uint8_t  fail_cnt;
    float    qw, qx, qy, qz;
    uint8_t  acc;
    uint32_t last_data_ms;
    BNO080   bno;
};
static Chan s_ch[TEST_CH];

static uint8_t  s_port_A   = 0xFF;
static uint8_t  s_port_B   = 0xFF;
static uint32_t s_probe_ms = 0;

// ── LED 状态机 ───────────────────────────────────────────────
enum LedPhase : uint8_t {
    LED_BLINK_ON,    // 亮
    LED_BLINK_OFF,   // 灭（blink 间隔）
    LED_CH_GAP,      // 通道间隔
    LED_LONG_GAP,    // 一轮结束间隔
};
static struct {
    LedPhase phase;
    uint8_t  cur_ch;       // 当前显示的通道
    uint8_t  blink_idx;    // 已完成 blink 数
    uint8_t  blink_total;  // 本通道需要 blink 总次数
    uint8_t  pin;          // 使用的 LED 引脚
    uint32_t t;            // 当前阶段起始时间
} s_led;


// ==============================================================
// I2C / 传感器
// ==============================================================
static uint8_t &port_cache(uint8_t ch) {
    return (CH_BUS[ch] == &Wire) ? s_port_A : s_port_B;
}

static bool tca_select(uint8_t ch) {
    uint8_t &cache = port_cache(ch);
    if (cache == CH_PORT[ch]) return true;
    CH_BUS[ch]->beginTransmission(CH_TCA[ch]);
    CH_BUS[ch]->write(1u << CH_PORT[ch]);
    bool ok = (CH_BUS[ch]->endTransmission() == 0);
    cache = ok ? CH_PORT[ch] : 0xFF;
    return ok;
}

static bool init_ch(uint8_t ch) {
    if (!tca_select(ch)) return false;
    if (!s_ch[ch].bno.begin(BNO_ADDR, *CH_BUS[ch])) return false;
    s_ch[ch].bno.enableRotationVector(BNO_REPORT_MS);
    s_ch[ch].fail_cnt     = 0;
    s_ch[ch].last_data_ms = millis();
    return true;
}

static void poll_ch(uint8_t ch) {
    if (!tca_select(ch)) { s_ch[ch].fail_cnt++; return; }
    CH_BUS[ch]->beginTransmission(BNO_ADDR);
    if (CH_BUS[ch]->endTransmission() != 0) { s_ch[ch].fail_cnt++; return; }
    s_ch[ch].fail_cnt = 0;
    if (s_ch[ch].bno.dataAvailable()) {
        s_ch[ch].qw           = s_ch[ch].bno.getQuatReal();
        s_ch[ch].qx           = s_ch[ch].bno.getQuatI();
        s_ch[ch].qy           = s_ch[ch].bno.getQuatJ();
        s_ch[ch].qz           = s_ch[ch].bno.getQuatK();
        s_ch[ch].acc          = s_ch[ch].bno.getQuatAccuracy();
        s_ch[ch].last_data_ms = millis();
    }
}


// ==============================================================
// LED 状态机（非阻塞）
// ==============================================================
static void led_off() {
    digitalWrite(LED_R, HIGH);
    digitalWrite(LED_B, HIGH);
    digitalWrite(LED_G, HIGH);
}

// 开始展示某通道的状态（触发第一个 blink）
static void led_ch_begin(uint8_t ch) {
    s_led.cur_ch    = ch;
    s_led.blink_idx = 0;
    switch (s_ch[ch].state) {
        case ST_LIVE:
            s_led.blink_total = 1;
            s_led.pin         = LED_G;   // 绿 1 下 = 正常
            break;
        case ST_OFFLINE:
            s_led.blink_total = 2;
            s_led.pin         = LED_R;   // 红 2 下 = 未接入/断连
            break;
        case ST_INITFAIL:
            s_led.blink_total = 3;
            s_led.pin         = LED_R;   // 红 3 下 = 芯片异常
            break;
    }
    led_off();
    digitalWrite(s_led.pin, LOW);        // 第一个 blink 亮起
    s_led.phase = LED_BLINK_ON;
    s_led.t     = millis();
}

static void led_tick() {
    uint32_t el = millis() - s_led.t;

    switch (s_led.phase) {

    case LED_BLINK_ON:
        if (el >= LED_T_ON) {
            led_off();
            s_led.blink_idx++;
            s_led.phase = LED_BLINK_OFF;
            s_led.t     = millis();
        }
        break;

    case LED_BLINK_OFF:
        if (el >= LED_T_OFF) {
            if (s_led.blink_idx < s_led.blink_total) {
                // 继续下一个 blink
                digitalWrite(s_led.pin, LOW);
                s_led.phase = LED_BLINK_ON;
            } else {
                // 本通道闪烁完毕
                s_led.phase = (s_led.cur_ch < TEST_CH - 1) ? LED_CH_GAP : LED_LONG_GAP;
            }
            s_led.t = millis();
        }
        break;

    case LED_CH_GAP:
        if (el >= LED_T_CH_GAP) led_ch_begin(s_led.cur_ch + 1);
        break;

    case LED_LONG_GAP:
        if (el >= LED_T_LONG) led_ch_begin(0);   // 重新开始循环
        break;
    }
}


// ==============================================================
// 串口输出（可选，调试用）
// ==============================================================
static void print_ch_detail(uint8_t ch) {
    Serial.print(F("# CH")); Serial.print(ch);
    Serial.print(F(" [")); Serial.print(ST_STR[s_ch[ch].state]); Serial.print(F("]"));
    if (s_ch[ch].state == ST_LIVE) {
        float mag = sqrtf(s_ch[ch].qw * s_ch[ch].qw + s_ch[ch].qx * s_ch[ch].qx +
                          s_ch[ch].qy * s_ch[ch].qy + s_ch[ch].qz * s_ch[ch].qz);
        char buf[64];
        snprintf(buf, sizeof(buf), " acc=%d mag=%.3f qw=%+.3f qx=%+.3f qy=%+.3f qz=%+.3f",
                 s_ch[ch].acc, mag, s_ch[ch].qw, s_ch[ch].qx, s_ch[ch].qy, s_ch[ch].qz);
        Serial.print(buf);
    }
    Serial.println();
}

static void notify(uint8_t ch, const __FlashStringHelper *msg) {
    Serial.print(F("# CH")); Serial.print(ch);
    Serial.print(F(" [")); Serial.print(ST_STR[s_ch[ch].state]);
    Serial.print(F("] ")); Serial.println(msg);
}


// ==============================================================
// setup / loop
// ==============================================================
void setup() {
    // LED 先亮红灯表示初始化中
    pinMode(LED_R, OUTPUT); pinMode(LED_B, OUTPUT); pinMode(LED_G, OUTPUT);
    digitalWrite(LED_R, LOW); digitalWrite(LED_B, HIGH); digitalWrite(LED_G, HIGH);

    Serial.begin(2000000);
#if ARDUINO_USB_CDC_ON_BOOT
    Serial.setTxTimeoutMs(0);
#endif
    delay(100);

    Wire.begin(PIN_SDA_A, PIN_SCL_A);   Wire.setClock(I2C_FREQ);  Wire.setTimeOut(10);
    Wire1.begin(PIN_SDA_B, PIN_SCL_B);  Wire1.setClock(I2C_FREQ); Wire1.setTimeOut(10);

    Wire.beginTransmission(0x70);  Wire.write(0x00);  Wire.endTransmission();
    Wire1.beginTransmission(0x77); Wire1.write(0x00); Wire1.endTransmission();
    delay(500);

    Serial.println(F("# BNO080_TEST ESP32S3-自研 " FW_VER));
    Serial.println(F("# ver: BNO080/BNO085 ESP32-S3 " FW_VER));
    Serial.println(F("# LED: Green×1=LIVE  Red×2=OFFLINE  Red×3=INITFAIL"));
    Serial.println(F("# cmd: R=reset_all"));
    Serial.println(F("# ---- init ----"));

    for (uint8_t ch = 0; ch < TEST_CH; ch++) {
        s_ch[ch].state     = ST_OFFLINE;
        s_ch[ch].fail_cnt  = 0;
        s_ch[ch].qw = 1.0f; s_ch[ch].qx = s_ch[ch].qy = s_ch[ch].qz = 0.0f;
        s_ch[ch].acc = 0;   s_ch[ch].last_data_ms = 0;

        if (init_ch(ch)) {
            s_ch[ch].state = ST_LIVE;
        } else {
            CH_BUS[ch]->beginTransmission(BNO_ADDR);
            s_ch[ch].state = (CH_BUS[ch]->endTransmission() == 0) ? ST_INITFAIL : ST_OFFLINE;
        }
        print_ch_detail(ch);
    }

    Serial.println(F("# ---- running ----"));
    led_off();
    led_ch_begin(0);   // 启动 LED 循环
    s_probe_ms = millis();
}

void loop() {
    // ── 命令处理 ──────────────────────────────────────────────
    while (Serial.available()) {
        char c = (char)Serial.read();
        if (c == 'R' || c == 'r') {
            Serial.println(F("# RESET ALL"));
            s_port_A = s_port_B = 0xFF;
            for (uint8_t ch = 0; ch < TEST_CH; ch++) {
                s_ch[ch].state = ST_OFFLINE; s_ch[ch].fail_cnt = 0;
            }
            s_probe_ms = millis() - PROBE_INTERVAL_MS;
        }
    }

    uint32_t now = millis();

    // ── 轮询 LIVE 通道 ────────────────────────────────────────
    for (uint8_t ch = 0; ch < TEST_CH; ch++) {
        if (s_ch[ch].state != ST_LIVE) continue;
        poll_ch(ch);
        if (s_ch[ch].fail_cnt >= OFFLINE_THRESH) {
            s_ch[ch].state = ST_OFFLINE;
            port_cache(ch) = 0xFF;
            notify(ch, F("disconnected"));
        }
    }

    // ── 探测非 LIVE 通道 ──────────────────────────────────────
    if (now - s_probe_ms >= PROBE_INTERVAL_MS) {
        s_probe_ms = now;
        for (uint8_t ch = 0; ch < TEST_CH; ch++) {
            if (s_ch[ch].state == ST_LIVE) continue;
            port_cache(ch) = 0xFF;
            if (!tca_select(ch)) continue;
            CH_BUS[ch]->beginTransmission(BNO_ADDR);
            if (CH_BUS[ch]->endTransmission() != 0) { s_ch[ch].state = ST_OFFLINE; continue; }
            if (init_ch(ch)) {
                s_ch[ch].state = ST_LIVE;
                notify(ch, F("hot-plug OK"));
            } else {
                if (s_ch[ch].state != ST_INITFAIL) {
                    s_ch[ch].state = ST_INITFAIL;
                    notify(ch, F("init failed (BNO defective?)"));
                }
            }
        }
    }

    // ── LED 状态机 tick ───────────────────────────────────────
    led_tick();
}
