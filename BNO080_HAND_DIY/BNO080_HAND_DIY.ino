/*!
 * BNO080_HAND_DIY.ino  v2.28  2026-04-27
 * 自研 ESP32-S3 PCB + 2× TCA9548A + 最多 16× BNO080/BNO085
 * 手势捕捉固件：16 通道帧（CH7 永久禁用，实际 15 路在线），FreeRTOS 双核 + BLE。
 * 配套上位机：IMU_Lab_CalibView（BLE/串口双通道）/ bno_hand_ble.py（纯 BLE 调试）
 *
 * v2.28 2026-04-27 — GH OTA 双路 fallback（CDN → RAW）
 *   [新增] GH_OTA 版本检查和固件下载均先走 jsDelivr CDN，失败自动 fallback
 *     到 raw.githubusercontent.com，串口打印 [CDN]/[RAW] 标识当前用哪路。
 * 完整版本历史见 CHANGELOG.md
 *
 * -- 硬件连接（自研 ESP32-S3 PCB）------------------------------
 *   Bus A: GPIO8(SDA)  / GPIO9(SCL)  → TCA1(0x77, A0/A1/A2=VCC) → CH0-7   [Core 1]
 *   Bus B: GPIO13(SDA) / GPIO14(SCL) → TCA2(0x70, A0/A1/A2=GND) → CH8-15  [Core 0]
 *   BNO080 I2C 地址：0x4B（ADDR=VCC）
 *   RGB LED（共阳，低电平点亮）：IO21=红  IO48=绿  IO47=蓝（实测核查）
 *
 * -- 二进制帧协议（91B/帧，16 通道槽，CH7 固定零）-----------
 *   [0xAA][0x55][seq:4B LE][ts_ms:4B LE][16ch×5B][xor:1B] = 91B
 *   每通道 5B: qw/qx/qy/qz (int8 ×127, 精度≈0.5°) + acc (uint8)
 *
 * -- 串口命令（字符 + 回车）------------------------------------
 *   R=reset  Z=yaw_zero  S=save_dcd  C=query_acc
 *   F/M=cycle_report  H=cycle_i2c_freq  A=toggle_ascii  B=ble_status  O=ota_status  G=gh_ota
 *   0-3=filter_preset  N<SN>=set_serial_no（重启生效）
 *
 * -- LED 状态 --------------------------------------------------
 *   蓝0.5Hz=BLE广播  红4Hz=无传感器  青1Hz=在线未校准
 *   绿/黄1Hz=已校准电量良/中  红双闪1.3s周期=低电
 *   白2Hz=校准模式（长按3s进入，acc≥2自动保存，短按取消）
 */

#include <Wire.h>
#include "SparkFun_BNO080_Arduino_Library.h"
#include <Preferences.h>
#include <NimBLEDevice.h>

// ── OTA 无线烧录（宏控开关）────────────────────────────────────
// OTA_ENABLED=1    ArduinoOTA：连局域网 WiFi，Arduino IDE 端口列表出现设备，一键推送
// AP_OTA_ENABLED=1 AP 热点+网页上传：无需外部 WiFi，电脑连热点→浏览器 http://192.168.4.1
// GH_OTA_ENABLED=1 GitHub 自动 OTA（需 OTA_ENABLED=1）：WiFi 连通后拉取 version.txt，
//                  版本不符则下载 firmware/BNO080_HAND_DIY.bin 并烧录重启
// 注：OTA_ENABLED 与 AP_OTA_ENABLED 不要同时置 1（共用无线模块）
#define OTA_ENABLED     1           // ArduinoOTA 无线烧录（需配套 Minimal SPIFFS 分区表）
#define AP_OTA_ENABLED  0           // 无 WiFi 环境时置 1；开发时保持 0
#define GH_OTA_ENABLED  1           // GitHub 自动拉取固件（需 OTA_ENABLED=1）
#if GH_OTA_ENABLED && !OTA_ENABLED
  #error "GH_OTA_ENABLED requires OTA_ENABLED=1"
#endif
#if OTA_ENABLED
  #define OTA_SSID  "LIBERAI"       // ← WiFi SSID
  #define OTA_PASS  "liberai1111"    // ← WiFi 密码
  #include <WiFi.h>
  #include <ArduinoOTA.h>
  static TaskHandle_t s_task_ota = NULL;
  #if GH_OTA_ENABLED
    #include <WiFiClientSecure.h>
    #include <HTTPUpdate.h>
    // jsDelivr CDN（国内首选，有节点缓存）；RAW 为 fallback（直连 GitHub，无缓存延迟）
    #define GH_VER_URL_CDN  "https://cdn.jsdelivr.net/gh/liujycode/liberai_liu@master/version.txt"
    #define GH_BIN_URL_CDN  "https://cdn.jsdelivr.net/gh/liujycode/liberai_liu@master/firmware/BNO080_HAND_DIY.bin"
    #define GH_VER_URL_RAW  "https://raw.githubusercontent.com/liujycode/liberai_liu/master/version.txt"
    #define GH_BIN_URL_RAW  "https://raw.githubusercontent.com/liujycode/liberai_liu/master/firmware/BNO080_HAND_DIY.bin"
  #endif
#endif
#if AP_OTA_ENABLED && !OTA_ENABLED
  #include <WiFi.h>
#endif
#if AP_OTA_ENABLED
  #define AP_OTA_SSID  "BNO_HAND_OTA"  // 热点名
  #define AP_OTA_PASS  ""               // 空字符串 = 开放热点（不加密）
  #include <WebServer.h>
  #include <Update.h>
  static WebServer    s_ap_server(80);
  static TaskHandle_t s_task_ap_ota = NULL;
#endif

#define FW_VER  "v2.28"

// ── 硬件引脚（自研 PCB）─────────────────────────────────────
// Bus A: Wire (GPIO8/9) → TCA1(0x70) → CH0-7  [Core 1]
#define PIN_SDA     8
#define PIN_SCL     9
// Bus B: Wire1 (GPIO13/14) → TCA2(0x77) → CH8-15  [Core 0]
#define PIN_SDA2    13
#define PIN_SCL2    14

// ── I2C 频率档位 ─────────────────────────────────────────────
static const uint32_t  I2C_FREQ_TABLE[] = {400000UL, 600000UL, 800000UL, 1000000UL};
static const char     *I2C_FREQ_NAMES[] = {"400kHz", "600kHz", "800kHz", "1MHz"};
#define I2C_FREQ_COUNT  4
static uint8_t  s_i2c_idx = 3;   // 默认 1MHz
#define I2C_FREQ  (I2C_FREQ_TABLE[s_i2c_idx])

// ── TCA9548A / BNO080 地址 ───────────────────────────────────
// TCA1(0x77): A0/A1/A2=VCC → Bus A (Wire,  GPIO8/9)  → CH0-7  ★实测0x77（原理图标注有误）
// TCA2(0x70): A0/A1/A2=GND → Bus B (Wire1, GPIO13/14)→ CH8-15 ★实测0x70（原理图标注有误）
#define TCA1_ADDR   0x77
#define TCA2_ADDR   0x70
#define BNO_ADDR    0x4B

// ── RGB LED（共阳，低电平点亮）──────────────────────────────
// 实测核查（LED_TEST v1.0 2026-04-21）：原理图标注全部错误，三通道颜色对调
// IO47 实为蓝，IO48 实为绿，IO21 实为红
#define LED_R   21    // IO21 → R42(240Ω) → 红（实测）
#define LED_G   48    // IO48 → R43(62Ω)  → 绿（实测）
#define LED_B   47    // IO47 → R44(62Ω)  → 蓝（实测）

// ── 通道数量与映射 ────────────────────────────────────────────
#define NUM_CH  16

// ── 永久禁用通道（硬件未焊接传感器，跳过初始化与轮询）──────────
// Bus A TCA1 port 7 = CH7；物理接口未安装，设为 MASK 跳过。
#define DISABLED_CH_MASK  ((uint32_t)(1u << 7))

// CH_TCA: 0=TCA1(Wire /Bus A/GPIO8-9 /CH0-7)
//         1=TCA2(Wire1/Bus B/GPIO13-14/CH8-15)
static const uint8_t CH_TCA[NUM_CH]  = {0,0,0,0,0,0,0,0, 1,1,1,1,1,1,1,1};
static const uint8_t CH_PORT[NUM_CH] = {0,1,2,3,4,5,6,7, 0,1,2,3,4,5,6,7};

// ── 整数缩放 ─────────────────────────────────────────────────
#define Q_INT_SCALE  127

// ── 报告类型 ─────────────────────────────────────────────────
static uint8_t s_rpt = 0;   // 默认 ARVR_RV（9-DOF，Yaw 绝对，需磁力计校准）
static const char *RPT_NAMES[] = {"ARVR_RV", "RV", "GRV", "ARVR_GRV"};
static const uint8_t RPT_IDS[4] = {
    SENSOR_REPORTID_AR_VR_STABILIZED_ROTATION_VECTOR,       // 0x28 → s_rpt=0
    SENSOR_REPORTID_ROTATION_VECTOR,                         // 0x05 → s_rpt=1
    SENSOR_REPORTID_GAME_ROTATION_VECTOR,                    // 0x08 → s_rpt=2
    SENSOR_REPORTID_AR_VR_STABILIZED_GAME_ROTATION_VECTOR,  // 0x29 → s_rpt=3
};
#define RPT_COUNT   4
#define REPORT_INTERVAL_MS  10  // 10ms → 100Hz，读到无新数据返回快(36μs)，消除时钟拉伸瓶颈

// ── 滤波预设 ─────────────────────────────────────────────────
struct FilterPreset { float alpha_move; float alpha_still; const char *name; };
static const FilterPreset FILTER_PRESETS[] = {
    { 1.00f, 1.00f, "OFF(raw)"       },
    { 0.30f, 0.30f, "fixed(0.3)"    },
    { 0.45f, 0.85f, "gesture(rec.)" },  // ★ 默认
    { 0.15f, 0.60f, "smooth"         },
};
#define FILTER_COUNT  4
static uint8_t s_flt_idx = 2;

// ── NLERP 静止检测 ────────────────────────────────────────────
#define STILL_THRESH  4e-7f
#define STILL_FRAMES  8

// ── 电压监测 ADC 引脚（原理图核查结果）──────────────────────
// AD_BAT 和 AD_VUSB 分压电路：200kΩ + 200kΩ → 比值 2.0
// 原理图中 AD_BAT/AD_VUSB 位于 IO8/IO9 之后，最可能为 IO10/IO11
// !! 请对照原理图 My_Project.pdf 中 U2 第 18/19 引脚确认 GPIO 编号 !!
#define PIN_BAT_ADC     10      // GPIO10（IO10，ADC1_CH9）← 待确认
#define PIN_USB_ADC     11      // GPIO11（IO11，ADC2_CH0）← 待确认
#define BAT_DIV_RATIO   2.0f   // 200kΩ / (200kΩ+200kΩ) = 0.5 → 倒数 2.0（原理图已确认）
#define USB_DIV_RATIO   2.0f   // 同上
#define ADC_REF_MV      3300   // ESP32-S3 ADC 参考电压 mV（3.3V 供电）
#define ADC_REPORT_MS   5000   // 每 5s 串口输出一次电压

// ── 校准触发按键（原理图核查结果）──────────────────────────
// SW2：3V3 → R41(10kΩ) → IO12 → C12(0.1uF) → GND；低电平有效。
// 原理图已核查（2026-04-21）。
#define PIN_BTN         12     // GPIO12（SW2，低电平有效，原理图已确认）
#define BTN_HOLD_MS     3000   // 长按多少 ms 触发校准模式（3s 防误触）
#define CAL_ACC_THRESH  2      // 校准完成阈值（acc >= 2 = BNO080 "reasonable"）
#define CAL_MIN_MS      3000   // 校准模式最短持续 ms（防止同帧进退，确保白闪可见）
#define BAT_MV_GOOD     3800   // 电池 > 3800mV = 绿（良好）
#define BAT_MV_MED      3500   // 电池 3500~3800mV = 黄（中）；< 3500 = 红（低电）
#define USB_MV_THRESH   4000   // USB > 4000mV = 视为接入（LED 叠加蓝色分量）

// ── BNO080 对象 ───────────────────────────────────────────────
static BNO080            s_bno[NUM_CH];
static volatile uint32_t s_live      = 0;
static volatile uint32_t s_ever_live = 0;

// ── 热插拔：连续失败计数 ─────────────────────────────────────
static volatile uint8_t  s_fail_cnt[NUM_CH];
static volatile uint32_t s_last_data_ms[NUM_CH];  // 最近一次 rr>0 时间戳，用于超时检测
static volatile uint32_t s_pending       = 0;      // hot-plug init OK，等 rr>0 确认再加入 s_live
static          uint32_t s_pending_ms[NUM_CH];     // 进入 pending 的时间（ms，Core1 写）
#define OFFLINE_THRESH      50     // 连续 I2C NACK 次数 → 判定离线
#define DATA_STALE_MS       3000   // ms：live 通道无 rr>0 数据 → 判定离线（防空端口误判）
#define PENDING_CONFIRM_MS  500    // ms：pending 超时无 rr>0 → 丢弃（防假 ACK 重上线）
#define PROBE_INTERVAL_MS   1000   // 离线通道探测间隔 ms
static uint32_t          s_probe_ms = 0;

// ── 校准模式状态 ─────────────────────────────────────────────
static bool     s_cal_mode            = false;
static uint32_t s_cal_entry_ms        = 0;      // 进入校准模式的时间（用于 CAL_MIN_MS 检查）
static bool     s_cal_waiting_release = false;  // 触发校准的长按尚未松开时为 true，防止松开误判为取消
static uint32_t s_btn_down_ms         = 0;
static bool     s_btn_pressed         = false;
static uint32_t s_adc_last_ms  = 0;
static float    s_bat_mv       = 4200.0f; // 最近一次电池电压 mV（ADC 每 5s 更新；初始假设满电）
static float    s_usb_mv       = 0.0f;   // 最近一次 USB 电压 mV

// ── 四元数缓存 ───────────────────────────────────────────────
static float   s_qw[NUM_CH], s_qx[NUM_CH],
               s_qy[NUM_CH], s_qz[NUM_CH];
static uint8_t s_acc[NUM_CH];

// ── NLERP 状态 ───────────────────────────────────────────────
static float   s_sq[NUM_CH][4];
static float   s_qprev[NUM_CH][4];
static uint8_t s_still_cnt[NUM_CH];
static bool    s_sq_inited[NUM_CH];

// ── 输出缓冲 ─────────────────────────────────────────────────
static char    s_buf[2048];
static uint8_t s_binbuf[96];   // 91B 帧 + 5B 预留
static bool    s_ascii_mode = false;

// ── OTA 状态 ─────────────────────────────────────────────────
static volatile bool s_ota_rainbow  = false;  // true → LED 7色彩虹循环（OTA 下载/烧录中）
static volatile bool s_ota_checking = false;  // true → LED 紫2Hz快闪（连WiFi/检查版本中）

// ── Wire 热路径端口追踪 ──────────────────────────────────────
// Bus A (Wire,  TCA1 0x70, GPIO8/9):   Core 1 独占，无竞争
// Bus B (Wire1, TCA2 0x77, GPIO13/14): Core 0 独占，无竞争
static uint8_t s_wire_portA = 0xFF;  // 当前已选通的 TCA1 端口（0xFF=未选通）
static uint8_t s_wire_portB = 0xFF;  // 当前已选通的 TCA2 端口（0xFF=未选通）

// ── FreeRTOS 双核同步 ─────────────────────────────────────────
static TaskHandle_t s_task_busB = NULL;  // Core 0: 轮询 Bus B TCA2 CH8-15
static TaskHandle_t s_main_task = NULL;  // Core 1: 主循环（loop）
static TaskHandle_t s_task_led  = NULL;  // Core 1: LED 独立任务（高优先级）

// ── 统计 ─────────────────────────────────────────────────────
static uint32_t s_pkt_cnt  = 0;
static uint32_t s_last_ms  = 0;
static uint32_t s_last_cnt = 0;
static uint32_t s_poll_min_us  = 0xFFFFFFFFUL;
static uint32_t s_poll_max_us  = 0;
static uint32_t s_poll_sum_us  = 0;
static uint32_t s_frame_min_us = 0xFFFFFFFFUL;
static uint32_t s_frame_max_us = 0;
static uint32_t s_frame_sum_us = 0;

// ── NVS 设置持久化 ────────────────────────────────────────────
static Preferences s_prefs;

// ── BLE（NimBLE-Arduino）────────────────────────────────────
#define BLE_DEVICE_NAME        "BNO_HAND"
#define DEVICE_SN_DEFAULT      "001"      // 默认序列号；批量烧录时直接改此宏，无需串口 N 命令

// ── 设备序列号（NVS "sn"，最长 8 字符，默认 "001"）──────────────
static char s_sn[9]           = DEVICE_SN_DEFAULT;
static char s_ble_full_name[24];   // "BNO_HAND_" + s_sn，ble_init() 前构造

// ── 串口命令行缓冲（支持多字符命令，如 N001）──────────────────
static char    s_serial_buf[32];
static uint8_t s_serial_pos = 0;
#define BLE_SVC_UUID           "0000FFE0-0000-1000-8000-00805F9B34FB"
#define BLE_DATA_UUID          "0000FFE1-0000-1000-8000-00805F9B34FB"  // NOTIFY
#define BLE_CMD_UUID           "0000FFE2-0000-1000-8000-00805F9B34FB"  // WRITE_NO_RSP
#define BLE_INFO_UUID          "0000FFE3-0000-1000-8000-00805F9B34FB"  // READ
#define BLE_MTU                512
// 帧/notify 缓冲上限（动态值 s_ble_fpn 在 onConnParamsUpdate 中根据 CI 计算）
// 4帧×91B=364B < 1 mbuf(~292B payload)...实测 NimBLE mtu=512时可容纳
#define BLE_FPN_MAX            4    // 缓冲区最大帧数（不能超过 MTU/91 ≈ 5）
// notify 最小间隔（ms）：保留但 s_ble_fpn 自然控速后实际不再限制
#define BLE_NOTIFY_MIN_MS      2

static bool                 s_ble_connected      = false;
static uint16_t             s_ble_conn_interval  = 36;   // 1.25ms 单位；初始保守值=45ms
static uint16_t             s_ble_conn_handle    = 0xFFFF; // 当前连接句柄
static uint32_t             s_ble_ci_retry_ms    = 0;    // CI 补发时刻（0=不触发）
static NimBLEServer        *s_ble_server         = nullptr;
static NimBLECharacteristic *s_ble_data_chr      = nullptr;
static NimBLECharacteristic *s_ble_cmd_chr       = nullptr;
static NimBLECharacteristic *s_ble_info_chr      = nullptr;
// 多帧打包缓冲（BLE_FPN_MAX × 91B；实际发帧数由 s_ble_fpn 决定）
static uint8_t  s_ble_pktbuf[BLE_FPN_MAX * 91];
static uint8_t  s_ble_fpn            = 1;     // 自适应帧/notify（onConnParamsUpdate 更新）
static uint8_t  s_ble_pkt_n          = 0;     // 当前已缓冲的帧数
static uint32_t s_ble_notify_last_ms = 0;     // 上次 notify 时刻（速率限制用）
static uint32_t s_ble_drop_cnt       = 0;     // 固件侧缓冲溢出丢帧计数
// BLE 统计（用于 'B' 命令）
static uint32_t s_ble_notify_cnt = 0;
static uint32_t s_ble_last_cnt   = 0;
static uint32_t s_ble_last_ms    = 0;

// 前向声明（BLE 回调中需要调用）
static void handle_cmd(char cmd);


// ==============================================================
// BLE 服务器回调（NimBLE-Arduino）
// ==============================================================
class BleServerCallbacks : public NimBLEServerCallbacks {
    void onConnect(NimBLEServer *pSvr, NimBLEConnInfo &info) override {
        s_ble_conn_interval = info.getConnInterval(); // 读取实际初始 CI（由 Windows 决定，通常 24-80）
        s_ble_conn_handle   = info.getConnHandle();
        s_ble_pkt_n         = 0;       // 清空打包缓冲
        // 立即用实际 CI 计算 fpn，在 s_ble_connected=true 之前完成赋值，
        // 避免 onConnParamsUpdate 未触发时 fpn 停在默认值 1（Windows 常见场景）
        {
            uint32_t ci_ms = (uint32_t)s_ble_conn_interval * 5 / 4;
            if (ci_ms < REPORT_INTERVAL_MS) ci_ms = REPORT_INTERVAL_MS; // 保守下限
            uint8_t fpn = (uint8_t)((ci_ms + REPORT_INTERVAL_MS - 1) / REPORT_INTERVAL_MS);
            if (fpn < 1)           fpn = 1;
            if (fpn > BLE_FPN_MAX) fpn = BLE_FPN_MAX;
            s_ble_fpn = fpn;
        }
        s_ble_connected = true;  // fpn 先就位，再开放发送路径（防 Core1 用旧 fpn）
        // 连接后立即刷新 INFO 特征为版本信息，供上位机连接后 read 获取
        if (s_ble_info_chr) {
            char ver[48];
            snprintf(ver, sizeof(ver), "%s " FW_VER " 15ch BNO080", s_ble_full_name);
            s_ble_info_chr->setValue((uint8_t*)ver, strlen(ver));
        }
        // 重置 STAT 定时器：保证上位机连接后有完整 5s 窗口读到版本信息，
        // 防止旧 STAT 计时残留导致连接后立即被 STAT 覆盖 INFO。
        s_last_ms = millis();
        // 请求 CI 范围 6-24（7.5-30ms）：给 Windows 选择余地；
        // 固定请求 CI=6 往往被 Windows 直接拒绝并反提 CI=40；
        // 允许最大 CI=24(30ms) 时 Windows 通常接受 CI=16-24，对应 100-150 fps。
        pSvr->updateConnParams(s_ble_conn_handle, 6, 24, 0, 400);
        s_ble_ci_retry_ms = millis() + 3000;  // 3s 后补发一次，避免首次被忽略
        if (Serial) {
            char tmp[80];
            snprintf(tmp, sizeof(tmp),
                     "# BLE: connected CI=%u(%.1fms) init_fpn=%u, requesting 7.5-30ms",
                     s_ble_conn_interval, s_ble_conn_interval * 1.25f, s_ble_fpn);
            Serial.println(tmp);
        }
    }
    void onDisconnect(NimBLEServer *pSvr, NimBLEConnInfo &info, int reason) override {
        s_ble_connected   = false;
        s_ble_conn_handle = 0xFFFF;
        s_ble_ci_retry_ms = 0;          // 取消补发
        s_ble_pkt_n       = 0;
        if (Serial) Serial.println(F("# BLE: disconnected, restarting advertising"));
        NimBLEDevice::startAdvertising();
    }
    void onConnParamsUpdate(NimBLEConnInfo &info) override {
        // 连接参数协商完成，记录实际 CI 并更新自适应 fpn
        s_ble_conn_interval = info.getConnInterval();
        // fpn = ceil(CI_ms / REPORT_INTERVAL_MS)，限制在 [1, BLE_FPN_MAX]
        // CI_ms = CI × 1.25 = CI × 5/4（整数除法向下取整）
        uint32_t ci_ms = (uint32_t)s_ble_conn_interval * 5 / 4;
        uint8_t  fpn   = (uint8_t)((ci_ms + REPORT_INTERVAL_MS - 1) / REPORT_INTERVAL_MS);
        if (fpn < 1)          fpn = 1;
        if (fpn > BLE_FPN_MAX) fpn = BLE_FPN_MAX;
        s_ble_fpn   = fpn;
        s_ble_pkt_n = 0;    // CI 变化时清空缓冲
        if (Serial) {
            char tmp[72];
            snprintf(tmp, sizeof(tmp), "# BLE: CI=%u(%.1fms) → fpn=%u(pack %u frames/notify)",
                     s_ble_conn_interval, s_ble_conn_interval * 1.25f, fpn, fpn);
            Serial.println(tmp);
        }
    }
    void onMTUChange(uint16_t mtu, NimBLEConnInfo &info) override {
        if (Serial) {
            char tmp[32]; snprintf(tmp, sizeof(tmp), "# BLE: MTU=%u", mtu);
            Serial.println(tmp);
        }
    }
};

// ==============================================================
// BLE CMD 特征值回调（接收单字节命令，等同于串口命令）
// ==============================================================
class BleCmdCallbacks : public NimBLECharacteristicCallbacks {
    void onWrite(NimBLECharacteristic *pChr, NimBLEConnInfo &info) override {
        std::string v = pChr->getValue();
        if (!v.empty()) handle_cmd((char)v[0]);
    }
};


// ==============================================================
// BLE 初始化（在 setup() 中于传感器初始化之前调用）
// ==============================================================
static void ble_init() {
    snprintf(s_ble_full_name, sizeof(s_ble_full_name), "BNO_HAND_%s", s_sn);
    NimBLEDevice::init(s_ble_full_name);
    NimBLEDevice::setMTU(BLE_MTU);
    NimBLEDevice::setPower(ESP_PWR_LVL_P9);   // 最大发射功率

    s_ble_server = NimBLEDevice::createServer();
    s_ble_server->setCallbacks(new BleServerCallbacks());
    s_ble_server->advertiseOnDisconnect(false);  // 在 onDisconnect 回调中手动重启

    NimBLEService *svc = s_ble_server->createService(BLE_SVC_UUID);

    // DATA 特征：NOTIFY，最多 BLE_FPN_MAX×91B/次
    s_ble_data_chr = svc->createCharacteristic(
        BLE_DATA_UUID, NIMBLE_PROPERTY::NOTIFY);

    // CMD 特征：WRITE_NR（Write No Response），接收单字节命令
    s_ble_cmd_chr = svc->createCharacteristic(
        BLE_CMD_UUID, NIMBLE_PROPERTY::WRITE_NR);
    s_ble_cmd_chr->setCallbacks(new BleCmdCallbacks());

    // INFO 特征：READ，固件信息字符串
    s_ble_info_chr = svc->createCharacteristic(
        BLE_INFO_UUID, NIMBLE_PROPERTY::READ);
    {
        char info[32];
        snprintf(info, sizeof(info), "%s " FW_VER " 15ch BNO080", s_ble_full_name);
        s_ble_info_chr->setValue((uint8_t *)info, (size_t)strlen(info));
    }

    svc->start();
    s_ble_server->start();   // 必须：注册 GATT profile 并启用连接事件回调

    // 广播包只含设备名，不含 128-bit UUID（UUID 占 18B，与 "BNO_HAND_XXX" 同放超 31B 上限，
    // NimBLE 自动移入扫描响应包导致 Windows 被动扫描收不到）。
    // 客户端连接后通过 GATT discover 找到 0xFFE0 服务，无需广播包含 UUID。
    {
        NimBLEAdvertising *adv = NimBLEDevice::getAdvertising();
        adv->setName(s_ble_full_name);   // 显式写广播名，不依赖 NimBLE 默认行为
        NimBLEDevice::startAdvertising();
    }
    { char tmp[40]; snprintf(tmp, sizeof(tmp), "# BLE: advertising as %s", s_ble_full_name); Serial.println(tmp); }
}


// ==============================================================
// LED 工具（共阳，非阻塞）
// ==============================================================
static void led_off() {
    digitalWrite(LED_R, HIGH);
    digitalWrite(LED_B, HIGH);
    digitalWrite(LED_G, HIGH);
}

// ==============================================================
// LED 独立任务（Core 1，优先级高于 loop，每 50ms tick 一次）
// 完全不受 loop() 阻塞影响
// ==============================================================
static void task_led_fn(void *) {
    for (;;) {
        led_tick();
        vTaskDelay(pdMS_TO_TICKS(50));
    }
}

static void led_tick() {
    uint32_t now  = millis();
    bool     on   = ((now / 500) & 1) == 0;   // 1Hz 慢闪相位
    bool     fast = ((now / 125) & 1) == 0;   // 4Hz 快闪相位

    // ── 0. OTA 下载/烧录中 → 7色彩虹循环（最高优先级）─────────
    // 颜色顺序：红→黄→绿→青→蓝→洋红→白，每色 200ms
    // 共阳 LED：LOW=亮，HIGH=灭
    if (s_ota_rainbow) {
        // R G B 电平（0=LOW/亮，1=HIGH/灭）
        static const uint8_t RB[7][3] = {
            {0,1,1}, // 红
            {0,0,1}, // 黄
            {1,0,1}, // 绿
            {1,0,0}, // 青
            {1,1,0}, // 蓝
            {0,1,0}, // 洋红
            {0,0,0}, // 白
        };
        uint8_t c = (now / 200) % 7;
        digitalWrite(LED_R, RB[c][0] ? HIGH : LOW);
        digitalWrite(LED_G, RB[c][1] ? HIGH : LOW);
        digitalWrite(LED_B, RB[c][2] ? HIGH : LOW);
        return;
    }

    // ── 1. OTA 检查中（连WiFi/拉version.txt）→ 紫色 2Hz 快闪 ──
    // 紫 = R+B，区别于蓝(BLE广播)/青(未校准)/白(校准模式)
    if (s_ota_checking) {
        bool blink = ((now / 250) & 1) == 0;  // 2Hz
        digitalWrite(LED_R, blink ? LOW : HIGH);
        digitalWrite(LED_G, HIGH);
        digitalWrite(LED_B, blink ? LOW : HIGH);
        return;
    }

    // ── 2. 校准模式（白色 2Hz 快闪，BLE 连接状态不影响）────────
    if (s_cal_mode) {
        bool w = ((now / 250) & 1) == 0;
        digitalWrite(LED_R, w ? LOW : HIGH);
        digitalWrite(LED_G, w ? LOW : HIGH);
        digitalWrite(LED_B, w ? LOW : HIGH);
        return;
    }

    // ── 3. 无传感器在线 → 红 4Hz 快闪（优先于 BLE 状态，避免红+蓝混色）
    uint32_t live = s_live;
    if (live == 0) {
        digitalWrite(LED_R, fast ? LOW : HIGH);
        digitalWrite(LED_G, HIGH);
        digitalWrite(LED_B, HIGH);
        return;
    }

    // ── 4. BLE 广播中（未连接）→ 蓝 0.5Hz 极慢闪 ──────────────
    if (!s_ble_connected) {
        bool blink = ((now / 1000) & 1) == 0;  // 0.5Hz
        digitalWrite(LED_R, HIGH);
        digitalWrite(LED_G, HIGH);
        digitalWrite(LED_B, blink ? LOW : HIGH);
        return;
    }

    // ── 5. 正常运行 ───────────────────────────────────────────────

    // 统计在线 + 已校准（acc≥2）通道数
    uint8_t live_cnt = 0, cal_cnt = 0;
    for (uint8_t ch = 0; ch < NUM_CH; ch++) {
        if (live & (1u << ch)) {
            live_cnt++;
            if (s_acc[ch] >= CAL_ACC_THRESH) cal_cnt++;
        }
    }

    if (cal_cnt < live_cnt) {
        // 有通道未校准（acc<2）→ 青（G+B）1Hz 慢闪（区别于广播中蓝 0.5Hz）
        digitalWrite(LED_R, HIGH);
        digitalWrite(LED_G, on ? LOW : HIGH);
        digitalWrite(LED_B, on ? LOW : HIGH);
    } else {
        // 所有在线通道已校准 → 颜色常显电池电量
        if (s_bat_mv > BAT_MV_GOOD) {
            // 绿 1Hz = 电量充足（>3800mV）
            digitalWrite(LED_R, HIGH);
            digitalWrite(LED_G, on ? LOW : HIGH);
            digitalWrite(LED_B, HIGH);
        } else if (s_bat_mv > BAT_MV_MED) {
            // 黄(红+绿) 1Hz = 电量中等（3500~3800mV）
            digitalWrite(LED_R, on ? LOW : HIGH);
            digitalWrite(LED_G, on ? LOW : HIGH);
            digitalWrite(LED_B, HIGH);
        } else {
            // 红 双闪+1s停 = 电量低（<3500mV）
            // 周期 1300ms：0-99ms亮 / 100-199ms灭 / 200-299ms亮 / 300-1299ms停（1s）
            // 区别于无传感器红 4Hz（持续均匀快闪）
            uint32_t phase = now % 1300;
            bool bat_on = (phase < 100) || (phase >= 200 && phase < 300);
            digitalWrite(LED_R, bat_on ? LOW : HIGH);
            digitalWrite(LED_G, HIGH);
            digitalWrite(LED_B, HIGH);
        }
    }
}


// ==============================================================
// NVS 读写
// ==============================================================
static void prefs_save() {
    s_prefs.begin("imu_cfg", false);
    s_prefs.putUChar("rpt", s_rpt);
    s_prefs.putUChar("i2c", s_i2c_idx);
    s_prefs.putUChar("flt", s_flt_idx);
    s_prefs.end();
}


// ==============================================================
// Wire TCA 工具（init 阶段，允许延时）
// ==============================================================
static void tca_close_all() {
    Wire.beginTransmission(TCA1_ADDR);  Wire.write((uint8_t)0x00); Wire.endTransmission(); delay(2);
    Wire1.beginTransmission(TCA2_ADDR); Wire1.write((uint8_t)0x00); Wire1.endTransmission(); delay(2);
}

static void tca_select(uint8_t ch) {
    uint8_t port = CH_PORT[ch];
    uint8_t mask = (uint8_t)(1u << port);
    if (CH_TCA[ch] == 0) {
        // TCA1, Bus A, Wire
        Wire.beginTransmission(TCA1_ADDR);
        Wire.write(mask);
        Wire.endTransmission();
    } else {
        // TCA2, Bus B, Wire1
        Wire1.beginTransmission(TCA2_ADDR);
        Wire1.write(mask);
        Wire1.endTransmission();
    }
    delay(2);
}


// ==============================================================
// Wire TCA 选通（热路径，双总线各自独占，同端口短路优化）
// Bus A (Wire,  TCA1 0x77, GPIO8/9):   Core 1 独占
// Bus B (Wire1, TCA2 0x70, GPIO13/14): Core 0 独占
// ==============================================================
static void wire_tca_select_A(uint8_t port) {
    if (s_wire_portA == port) return;
    Wire.beginTransmission(TCA1_ADDR);
    Wire.write((uint8_t)(1u << port));
    Wire.endTransmission();
    s_wire_portA = port;
}

static void wire_tca_select_B(uint8_t port) {
    if (s_wire_portB == port) return;
    Wire1.beginTransmission(TCA2_ADDR);
    Wire1.write((uint8_t)(1u << port));
    Wire1.endTransmission();
    s_wire_portB = port;
}


// ==============================================================
// Wire BNO080 SHTP 读（单次 requestFrom 读完 header + payload）
// 返回：1=新四元数数据  0=无新数据（正常）  -1=I2C总线错误
// ==============================================================
static int8_t wire_bno_read(TwoWire &w,
                             float *qw, float *qx, float *qy, float *qz,
                             uint8_t *acc, bool *reset_detected)
{
    *reset_detected = false;

    uint8_t n = w.requestFrom((uint8_t)BNO_ADDR, (uint8_t)23);
    uint8_t buf[23] = {0};
    uint8_t got = 0;
    while (w.available() && got < 23) buf[got++] = (uint8_t)w.read();
    while (w.available()) w.read();

    if (n == 0)      return -1;   // NACK：I2C 总线错误
    if (got < 4)     return  0;   // 数据不足：无报告

    uint16_t pktLen = (uint16_t)buf[0] | ((uint16_t)(buf[1] & 0x7F) << 8);
    if (pktLen == 0)     return  0;   // 无数据，BNO080 暂无新报告
    if (pktLen > 0x3FFF) return -1;   // 包长异常，疑似总线噪声
    if (pktLen <= 4)     return  0;   // header-only，无 payload

    uint8_t  chan    = buf[2];
    uint8_t *payload = buf + 4;
    uint16_t dataLen = (uint16_t)(pktLen - 4);

    if (chan == CHANNEL_EXECUTABLE && dataLen >= 1 && payload[0] == EXECUTABLE_RESET_COMPLETE) {
        *reset_detected = true;
        return 0;
    }

    if (chan != CHANNEL_REPORTS && chan != CHANNEL_WAKE_REPORTS) return 0;
    if (dataLen < 17) return 0;
    if (payload[0] != SHTP_REPORT_BASE_TIMESTAMP) return 0;

    uint8_t rptID = payload[5];
    if (rptID != SENSOR_REPORTID_ROTATION_VECTOR                        &&
        rptID != SENSOR_REPORTID_GAME_ROTATION_VECTOR                   &&
        rptID != SENSOR_REPORTID_AR_VR_STABILIZED_ROTATION_VECTOR       &&
        rptID != SENSOR_REPORTID_AR_VR_STABILIZED_GAME_ROTATION_VECTOR) return 0;

    *acc = payload[7] & 0x03;

    const float Q14 = 1.0f / 16384.0f;
    *qx = (int16_t)((uint16_t)payload[9]  | ((uint16_t)payload[10] << 8)) * Q14;
    *qy = (int16_t)((uint16_t)payload[11] | ((uint16_t)payload[12] << 8)) * Q14;
    *qz = (int16_t)((uint16_t)payload[13] | ((uint16_t)payload[14] << 8)) * Q14;
    *qw = (int16_t)((uint16_t)payload[15] | ((uint16_t)payload[16] << 8)) * Q14;

    return 1;
}


// ==============================================================
// NLERP 平滑（半球对齐 → 毛刺剔除 → NLERP）
// ==============================================================
static void nlerp_update(uint8_t ch, float qw, float qx, float qy, float qz) {
    if (!s_sq_inited[ch]) {
        if (qw*qw + qx*qx + qy*qy + qz*qz < 0.25f) return;  // 首帧模长校验
        s_sq[ch][0]=qw; s_sq[ch][1]=qx; s_sq[ch][2]=qy; s_sq[ch][3]=qz;
        s_qprev[ch][0]=qw; s_qprev[ch][1]=qx; s_qprev[ch][2]=qy; s_qprev[ch][3]=qz;
        s_sq_inited[ch] = true; return;
    }
    // ① 半球对齐（BNO080 偶发 antipodal flip 纠正，必须在 dsq 计算前）
    float dot = s_sq[ch][0]*qw + s_sq[ch][1]*qx + s_sq[ch][2]*qy + s_sq[ch][3]*qz;
    if (dot < 0.0f) { qw=-qw; qx=-qx; qy=-qy; qz=-qz; }
    // ② 帧间差分
    float dw=qw-s_qprev[ch][0], dx=qx-s_qprev[ch][1],
          dy=qy-s_qprev[ch][2], dz=qz-s_qprev[ch][3];
    float dsq = dw*dw + dx*dx + dy*dy + dz*dz;
    // ③ 毛刺剔除：dsq>0.1 对应 ~36°/帧，超出人手运动极限，视为 I2C 错帧
    if (dsq > 0.1f) {
        s_qprev[ch][0]=qw; s_qprev[ch][1]=qx; s_qprev[ch][2]=qy; s_qprev[ch][3]=qz;
        s_still_cnt[ch] = 0;
        return;
    }
    s_qprev[ch][0]=qw; s_qprev[ch][1]=qx; s_qprev[ch][2]=qy; s_qprev[ch][3]=qz;
    if (dsq < STILL_THRESH) { if (s_still_cnt[ch] < STILL_FRAMES) s_still_cnt[ch]++; }
    else { s_still_cnt[ch] = 0; }
    bool still = (s_still_cnt[ch] >= STILL_FRAMES);
    float alpha = still ? FILTER_PRESETS[s_flt_idx].alpha_still
                        : FILTER_PRESETS[s_flt_idx].alpha_move;
    float w=s_sq[ch][0]+alpha*(qw-s_sq[ch][0]), x=s_sq[ch][1]+alpha*(qx-s_sq[ch][1]),
          y=s_sq[ch][2]+alpha*(qy-s_sq[ch][2]), z=s_sq[ch][3]+alpha*(qz-s_sq[ch][3]);
    float nm = sqrtf(w*w + x*x + y*y + z*z);
    if (nm < 1e-8f) { s_sq_inited[ch] = false; return; }
    s_sq[ch][0]=w/nm; s_sq[ch][1]=x/nm; s_sq[ch][2]=y/nm; s_sq[ch][3]=z/nm;
}

static void nlerp_reset(uint8_t ch) {
    s_sq[ch][0]=1.0f; s_sq[ch][1]=0.0f; s_sq[ch][2]=0.0f; s_sq[ch][3]=0.0f;
    s_qprev[ch][0]=1.0f; s_qprev[ch][1]=0.0f; s_qprev[ch][2]=0.0f; s_qprev[ch][3]=0.0f;
    s_still_cnt[ch]=0; s_sq_inited[ch]=false;
}


// ==============================================================
// SparkFun enable_report（init 阶段调用）
// ==============================================================
static void enable_report(uint8_t ch) {
    switch (s_rpt) {
    case 1: s_bno[ch].enableRotationVector(REPORT_INTERVAL_MS); break;
    case 2: s_bno[ch].enableGameRotationVector(REPORT_INTERVAL_MS); break;
    case 3: s_bno[ch].enableARVRStabilizedGameRotationVector(REPORT_INTERVAL_MS); break;
    default: s_bno[ch].enableARVRStabilizedRotationVector(REPORT_INTERVAL_MS); break;
    }
}


// ==============================================================
// 单通道初始化（Wire 模式）
// ==============================================================
static bool init_channel(uint8_t ch) {
    tca_select(ch);
    delay(5);
    TwoWire &w = (CH_TCA[ch] == 0) ? Wire : Wire1;
    w.beginTransmission(BNO_ADDR);
    uint8_t probe = w.endTransmission();
    if (probe != 0) {
        Serial.print(F("[probe=")); Serial.print(probe); Serial.print(F("] "));
        tca_close_all(); return false;
    }
    if (!s_bno[ch].begin(BNO_ADDR, w)) {
        Serial.print(F("[begin_fail] "));
        tca_close_all(); return false;
    }
    enable_report(ch);
    s_qw[ch]=1.0f; s_qx[ch]=0.0f; s_qy[ch]=0.0f; s_qz[ch]=0.0f; s_acc[ch]=0;
    nlerp_reset(ch);
    tca_close_all();
    return true;
}


// ==============================================================
// 全通道初始化（三轮重试）
// ==============================================================
static void init_all() {
    s_live = 0;
    for (uint8_t ch = 0; ch < NUM_CH; ch++) {
        s_qw[ch]=1.0f; s_qx[ch]=0.0f; s_qy[ch]=0.0f; s_qz[ch]=0.0f; s_acc[ch]=0;
        s_last_data_ms[ch] = 0;
    }
    tca_close_all();
    delay(50);

    uint8_t ok_cnt = 0;
    // 所有可用通道（排除永久禁用）都成功才提前退出
    const uint32_t EXPECTED_LIVE = ((uint32_t)((1u << NUM_CH) - 1u)) & ~DISABLED_CH_MASK;
    // 第一轮
    for (uint8_t ch = 0; ch < NUM_CH; ch++) {
        Serial.print(F("# CH")); Serial.print(ch); Serial.print(F(": "));
        if ((uint32_t)(1u << ch) & DISABLED_CH_MASK) { Serial.println(F("DISABLED")); continue; }
        if (init_channel(ch)) {
            s_live |= (uint32_t)(1u << ch); ok_cnt++;
            s_last_data_ms[ch] = millis();
            Serial.print(F("OK (")); Serial.print(RPT_NAMES[s_rpt]); Serial.println(')');
        } else { Serial.println(F("wait...")); }
        delay(10);
    }
    // 第二轮（200ms 后重试未成功通道）
    if (s_live != EXPECTED_LIVE) {
        delay(200);
        for (uint8_t ch = 0; ch < NUM_CH; ch++) {
            if ((uint32_t)(1u << ch) & DISABLED_CH_MASK) continue;
            if (s_live & (uint32_t)(1u << ch)) continue;
            Serial.print(F("# CH")); Serial.print(ch); Serial.print(F(" retry2: "));
            if (init_channel(ch)) {
                s_live |= (uint32_t)(1u << ch); ok_cnt++;
                s_last_data_ms[ch] = millis();
                Serial.println(F("OK"));
            } else { Serial.println(F("wait...")); }
        }
    }
    // 第三轮（500ms 后最终重试）
    if (s_live != EXPECTED_LIVE) {
        delay(500);
        for (uint8_t ch = 0; ch < NUM_CH; ch++) {
            if ((uint32_t)(1u << ch) & DISABLED_CH_MASK) continue;
            if (s_live & (uint32_t)(1u << ch)) continue;
            Serial.print(F("# CH")); Serial.print(ch); Serial.print(F(" retry3: "));
            if (init_channel(ch)) {
                s_live |= (uint32_t)(1u << ch); ok_cnt++;
                s_last_data_ms[ch] = millis();
                Serial.println(F("OK"));
            } else { Serial.println(F("MISS")); }
        }
    }
    Serial.print(F("# init done: ")); Serial.print(ok_cnt); Serial.println(F(" channels OK"));
    Serial.print(F("# live_mask=0x")); Serial.println(s_live, HEX);
    s_ever_live |= s_live;
    // 重置所有 live 通道的 last_data_ms，从 init_all() 完成时刻起算 DATA_STALE_MS
    // 避免因多通道串行初始化（~2s）导致早期通道启动即触发 stale→offline
    {
        uint32_t t = millis();
        for (uint8_t ch = 0; ch < NUM_CH; ch++) {
            if ((s_live >> ch) & 1u) s_last_data_ms[ch] = t;
        }
    }
}


// ==============================================================
// 二进制帧构建（91B → s_binbuf，BLE 与串口共用）
// ==============================================================
static void build_binary_frame() {
    uint8_t *p = s_binbuf;

    *p++ = 0xAA; *p++ = 0x55;

    uint32_t seq = s_pkt_cnt;
    *p++ = (uint8_t)(seq);        *p++ = (uint8_t)(seq >> 8);
    *p++ = (uint8_t)(seq >> 16);  *p++ = (uint8_t)(seq >> 24);

    uint32_t ts = millis();
    *p++ = (uint8_t)(ts);         *p++ = (uint8_t)(ts >> 8);
    *p++ = (uint8_t)(ts >> 16);   *p++ = (uint8_t)(ts >> 24);

    for (uint8_t ch = 0; ch < NUM_CH; ch++) {
        float qw = s_sq_inited[ch] ? s_sq[ch][0] : s_qw[ch];
        float qx = s_sq_inited[ch] ? s_sq[ch][1] : s_qx[ch];
        float qy = s_sq_inited[ch] ? s_sq[ch][2] : s_qy[ch];
        float qz = s_sq_inited[ch] ? s_sq[ch][3] : s_qz[ch];

        int8_t iqw = (int8_t)constrain((int)roundf(qw * Q_INT_SCALE), -127, 127);
        int8_t iqx = (int8_t)constrain((int)roundf(qx * Q_INT_SCALE), -127, 127);
        int8_t iqy = (int8_t)constrain((int)roundf(qy * Q_INT_SCALE), -127, 127);
        int8_t iqz = (int8_t)constrain((int)roundf(qz * Q_INT_SCALE), -127, 127);

        *p++ = (uint8_t)iqw;
        *p++ = (uint8_t)iqx;
        *p++ = (uint8_t)iqy;
        *p++ = (uint8_t)iqz;
        *p++ = s_acc[ch];
    }

    uint8_t xor_val = 0;
    for (int i = 2; i < 90; i++) xor_val ^= s_binbuf[i];
    *p++ = xor_val;
}

// ==============================================================
// 二进制帧输出（串口路径，91B）
// ==============================================================
static void write_binary_frame() {
    build_binary_frame();
    Serial.write(s_binbuf, 91);
}


// ==============================================================
// BLE 帧通知（速率限制单帧发送，CI_ms 间隔）
// 每次直接发送最新帧 s_binbuf（91B），以 CI_ms 限速，不再批量打包。
// 无论 notify() 成败均推进计时器，彻底消除"缓冲卡死"丢帧场景：
//   旧逻辑：fpn=3 时 notify 失败 → pkt_n 停在 3 → 后续 ~150ms 所有新帧丢弃。
//   新逻辑：无 pkt_n 概念 → notify 失败仅跳过本 CI 窗口，下窗口发最新帧。
// ==============================================================
static void notify_ble_frame() {
    if (!s_ble_connected || !s_ble_data_chr) return;

    uint32_t now   = millis();
    uint32_t ci_ms = (uint32_t)s_ble_conn_interval * 5 / 4;  // CI × 1.25ms
    if (ci_ms < BLE_NOTIFY_MIN_MS) ci_ms = BLE_NOTIFY_MIN_MS;  // 下限 2ms（BNO080输出率由REPORT_INTERVAL_MS独立控制）

    if (now - s_ble_notify_last_ms < ci_ms) return;  // 未到下一 CI 窗口
    s_ble_notify_last_ms = now;  // 无论成败均推进，防止快速重试冲击 NimBLE mbuf

    s_ble_data_chr->setValue(s_binbuf, 91);
    bool ok = s_ble_data_chr->notify();
    if (ok) {
        s_ble_notify_cnt++;
    } else {
        s_ble_drop_cnt++;  // notify 失败（mbuf 繁忙），本 CI 窗口跳过
    }
}


// ==============================================================
// ASCII 帧输出（调试用）
// ==============================================================
static void write_ascii_frame() {
    int pos = snprintf(s_buf, sizeof(s_buf), "%lu,%lu,", s_pkt_cnt, millis());
    for (uint8_t ch = 0; ch < NUM_CH; ch++) {
        float qw = s_sq_inited[ch] ? s_sq[ch][0] : s_qw[ch];
        float qx = s_sq_inited[ch] ? s_sq[ch][1] : s_qx[ch];
        float qy = s_sq_inited[ch] ? s_sq[ch][2] : s_qy[ch];
        float qz = s_sq_inited[ch] ? s_sq[ch][3] : s_qz[ch];
        pos += snprintf(s_buf + pos, sizeof(s_buf) - pos,
            "%s%ld,%ld,%ld,%ld,%d",
            ch == 0 ? "" : ",",
            (long)roundf(qw * Q_INT_SCALE), (long)roundf(qx * Q_INT_SCALE),
            (long)roundf(qy * Q_INT_SCALE), (long)roundf(qz * Q_INT_SCALE),
            (int)(s_acc[ch] * 100));
    }
    s_buf[pos++] = '\r'; s_buf[pos++] = '\n';
    Serial.write((const uint8_t*)s_buf, (size_t)pos);
}


// ==============================================================
// Core 0 任务：轮询 Bus B TCA2 CH8-15（Wire1，GPIO13/14）
// ==============================================================
static void task_poll_busB(void *pvParameters) {
    while (true) {
        // 等待主任务（Core 1）触发
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

        // 热路径：CH8-15，Bus B（Wire1，TCA2 0x77）
        for (uint8_t ch = 8; ch < NUM_CH; ch++) {
            bool ch_live = (s_live    >> ch) & 1u;
            bool ch_pend = (s_pending >> ch) & 1u;
            if (!ch_live && !ch_pend) continue;

            wire_tca_select_B(CH_PORT[ch]);

            float qw, qx, qy, qz; uint8_t acc; bool rst = false;
            int8_t rr = wire_bno_read(Wire1, &qw, &qx, &qy, &qz, &acc, &rst);
            if (rr > 0) {
                s_fail_cnt[ch] = 0;
                s_acc[ch] = acc;
                s_qw[ch]=qw; s_qx[ch]=qx; s_qy[ch]=qy; s_qz[ch]=qz;
                nlerp_update(ch, qw, qx, qy, qz);
                s_last_data_ms[ch] = millis();
                if (ch_pend) {
                    // 收到真实数据，从 pending 晋升为 live
                    s_pending  &= ~(uint32_t)(1u << ch);
                    s_live     |=  (uint32_t)(1u << ch);
                    s_ever_live |= (uint32_t)(1u << ch);
                    nlerp_reset(ch);
                    Serial.print(F("# CH")); Serial.print(ch); Serial.println(F(" online"));
                }
            } else if (rr < 0 && ch_live) {
                if (++s_fail_cnt[ch] >= OFFLINE_THRESH) {
                    s_live &= ~(uint32_t)(1u << ch);
                    s_fail_cnt[ch] = 0;
                    Serial.print(F("# CH")); Serial.print(ch); Serial.println(F(" offline"));
                }
            }
            if (rst && ch_live) {
                vTaskDelay(pdMS_TO_TICKS(2));    // BNO080 收到 RESET_COMPLETE 即可接指令，2ms 足够
                s_fail_cnt[ch] = 0;
                wire_tca_select_B(CH_PORT[ch]);
                enable_report(ch);
                s_sq_inited[ch] = false;
                s_last_data_ms[ch] = millis();
            }
        }
        // 热路径不关闭 TCA：下帧 wire_tca_select_B 同端口短路，节省一次 TCA 写操作

        // 通知主任务 CH8-15 轮询完成
        xTaskNotifyGive(s_main_task);
    }
}


// ==============================================================
// 串口命令处理（R/S/F/H 命令先暂停 Core0 任务避免总线冲突）
// ==============================================================
// ==============================================================
// GitHub OTA：检查 version.txt → 版本不符 → 下载 .bin → 烧录重启
// 仓库结构：
//   version.txt                            ← 仅含版本字符串，如 "v2.16"
//   firmware/BNO080_HAND_DIY.bin           ← Arduino IDE "导出已编译的二进制文件" 产物
// 更新流程：编译→导出.bin→放入 firmware/ → 更新 version.txt → git push
// ==============================================================
#if GH_OTA_ENABLED
static void gh_ota_check() {
    if (WiFi.status() != WL_CONNECTED) {
        if (Serial) Serial.println(F("# GH_OTA: WiFi not connected, skip"));
        return;
    }
    if (Serial) Serial.print(F("# GH_OTA: checking version..."));

    // ── 版本检查（先 CDN，失败 fallback RAW）─────────────────────
    const char* ver_urls[2] = {GH_VER_URL_CDN, GH_VER_URL_RAW};
    String remote_ver;
    for (int i = 0; i < 2; i++) {
        WiFiClientSecure sec;
        sec.setInsecure();
        HTTPClient http;
        http.begin(sec, ver_urls[i]);
        http.setTimeout(6000);
        int code = http.GET();
        if (code == 200) {
            remote_ver = http.getString();
            http.end();
            if (Serial) { char t[12]; snprintf(t, sizeof(t), " [%s]", i==0?"CDN":"RAW"); Serial.print(t); }
            break;
        }
        if (Serial) { char t[28]; snprintf(t, sizeof(t), " [%s %d, fb]", i==0?"CDN":"RAW", code); Serial.print(t); }
        http.end();
    }
    if (remote_ver.isEmpty()) {
        if (Serial) Serial.println(F("\n# GH_OTA: version fetch failed, skip"));
        return;
    }
    remote_ver.trim();
    {
        char tmp[64];
        snprintf(tmp, sizeof(tmp), "\n# GH_OTA: remote=%s  local=%s",
                 remote_ver.c_str(), FW_VER);
        if (Serial) Serial.print(tmp);
    }
    if (remote_ver == FW_VER) {
        if (Serial) Serial.println(F("  [up-to-date]"));
        return;
    }
    if (Serial) Serial.println(F("  [UPDATING → downloading .bin]"));
    s_ota_rainbow = true;
    if (s_task_busB) vTaskSuspend(s_task_busB);
    httpUpdate.rebootOnUpdate(true);

    // ── 固件下载（先 CDN，失败 fallback RAW）─────────────────────
    const char* bin_urls[2] = {GH_BIN_URL_CDN, GH_BIN_URL_RAW};
    t_httpUpdate_return ret = HTTP_UPDATE_FAILED;
    for (int i = 0; i < 2; i++) {
        if (Serial) { char t[32]; snprintf(t, sizeof(t), "# GH_OTA: [%s] downloading...", i==0?"CDN":"RAW"); Serial.println(t); }
        WiFiClientSecure sec2;
        sec2.setInsecure();
        ret = httpUpdate.update(sec2, bin_urls[i]);
        if (ret != HTTP_UPDATE_FAILED) break;   // OK 或 NO_UPDATES，停止重试
        char tmp[80];
        snprintf(tmp, sizeof(tmp), "# GH_OTA: [%s] FAILED(%d) %s",
                 i==0?"CDN":"RAW",
                 httpUpdate.getLastError(),
                 httpUpdate.getLastErrorString().c_str());
        if (Serial) Serial.println(tmp);
    }
    if (ret == HTTP_UPDATE_FAILED) {
        if (s_task_busB) vTaskResume(s_task_busB);  // 两路均失败，恢复 I2C
    }
    // rebootOnUpdate=true 时成功不会执行到此
}
#endif  // GH_OTA_ENABLED

// ==============================================================
// WiFi ArduinoOTA 任务（OTA_ENABLED=1）
// Core 0，低优先级。BLE 正常使用时不影响；OTA 烧录期间 BLE 可能短暂抖动。
// ==============================================================
#if OTA_ENABLED
static void task_ota_fn(void*) {
    s_ota_checking = true;                     // LED → 紫色快闪（连WiFi中）
    WiFi.mode(WIFI_STA);
    WiFi.setHostname(s_ble_full_name);   // IDE 端口显示名 = BLE 广播名
    WiFi.begin(OTA_SSID, OTA_PASS);

    if (Serial) Serial.print(F("# OTA: WiFi connecting"));
    uint32_t t0 = millis();
    while (WiFi.status() != WL_CONNECTED && millis() - t0 < 15000) {
        vTaskDelay(pdMS_TO_TICKS(500));
        if (Serial) Serial.print('.');
    }
    if (WiFi.status() != WL_CONNECTED) {
        if (Serial) Serial.println(F("\n# OTA: WiFi timeout, OTA disabled"));
        s_ota_checking = false;                // LED → 恢复正常
        vTaskDelete(NULL);
        return;
    }
    {
        char tmp[80];
        snprintf(tmp, sizeof(tmp), "\n# OTA: WiFi OK  IP=%s  host=%s",
                 WiFi.localIP().toString().c_str(), s_ble_full_name);
        if (Serial) Serial.println(tmp);
    }

    ArduinoOTA.setHostname(s_ble_full_name);
    ArduinoOTA.onStart([]() {
        // 暂停 I2C 轮询，防止 Update.write() 与 Wire 竞争导致 crash
        if (s_task_busB) vTaskSuspend(s_task_busB);
        if (Serial) Serial.println(F("# OTA: flashing start..."));
    });
    ArduinoOTA.onEnd([]() {
        if (Serial) Serial.println(F("# OTA: done, rebooting..."));
    });
    ArduinoOTA.onProgress([](unsigned int done, unsigned int total) {
        // 每 25% 打印一次，避免刷屏
        static unsigned int s_last_q = 99;
        unsigned int q = done * 4 / total;   // 0~3
        if (q != s_last_q) {
            s_last_q = q;
            char tmp[28]; snprintf(tmp, sizeof(tmp), "# OTA: %u%%", done * 100 / total);
            if (Serial) Serial.println(tmp);
        }
    });
    ArduinoOTA.onError([](ota_error_t e) {
        char tmp[32]; snprintf(tmp, sizeof(tmp), "# OTA: error(%u)", (unsigned)e);
        if (Serial) Serial.println(tmp);
        if (s_task_busB) vTaskResume(s_task_busB);   // 恢复轮询
    });
    ArduinoOTA.begin();
    if (Serial) Serial.println(F("# OTA: ready (IDE → 端口 → BNO_HAND_xxx (OTA))"));

#if GH_OTA_ENABLED
    gh_ota_check();     // 上电 WiFi 连通后自动检查一次 GitHub 更新
#endif
    s_ota_checking = false;                    // 检查完毕，LED → 恢复正常（无更新时）

    for (;;) {
        ArduinoOTA.handle();
        vTaskDelay(pdMS_TO_TICKS(50));   // 20 次/秒，足够 IDE 发现设备
    }
}
#endif  // OTA_ENABLED


// ==============================================================
// AP 热点 + 网页上传 OTA 任务（AP_OTA_ENABLED=1）
// 上电后设备创建热点 AP_OTA_SSID，电脑连接后浏览器打开 192.168.4.1 选 .bin 上传。
// 使用 Arduino IDE：项目 → 导出已编译的二进制文件，得到 .bin 再上传。
// ==============================================================
#if AP_OTA_ENABLED
static const char AP_OTA_PAGE[] PROGMEM =
    "<!DOCTYPE html><html><head><meta charset='utf-8'>"
    "<title>BNO_HAND OTA</title><style>"
    "body{font-family:sans-serif;max-width:440px;margin:48px auto;"
    "background:#1a1a1a;color:#eee;padding:0 16px}"
    "h2{color:#4af;margin-bottom:8px}"
    "p{font-size:13px;color:#aaa;margin:4px 0 16px}"
    "input[type=file]{display:block;margin:12px 0;color:#eee}"
    "button{background:#4af;border:none;padding:9px 28px;"
    "border-radius:5px;color:#000;cursor:pointer;font-size:15px;font-weight:bold}"
    "#msg{margin-top:14px;font-size:13px}"
    "</style></head><body>"
    "<h2>&#128268; BNO_HAND Firmware OTA</h2>"
    "<p>选择 .bin 文件（Arduino IDE → 项目 → 导出已编译的二进制文件）</p>"
    "<form id='f' method='POST' action='/update' enctype='multipart/form-data'>"
    "<input type='file' name='firmware' accept='.bin' required>"
    "<button type='submit'>上传并烧录</button>"
    "</form>"
    "<div id='msg'></div>"
    "<script>"
    "document.getElementById('f').onsubmit=function(){"
    "document.getElementById('msg').innerText='烧录中，请等待设备重启...';"
    "};"
    "</script>"
    "</body></html>";

static void task_ap_ota_fn(void*) {
    WiFi.mode(WIFI_AP);
    bool ap_ok = WiFi.softAP(AP_OTA_SSID,
                              AP_OTA_PASS[0] ? AP_OTA_PASS : nullptr);
    {
        char tmp[72];
        snprintf(tmp, sizeof(tmp), "# AP-OTA: 热点\"%s\" %s  IP=192.168.4.1",
                 AP_OTA_SSID, ap_ok ? "OK" : "FAIL");
        if (Serial) Serial.println(tmp);
    }

    s_ap_server.on("/", HTTP_GET, []() {
        s_ap_server.send_P(200, "text/html", AP_OTA_PAGE);
    });
    // POST /update：接收 .bin 并写入 OTA 分区
    s_ap_server.on("/update", HTTP_POST,
        []() {
            s_ap_server.send(200, "text/plain",
                             Update.hasError() ? "FAIL" : "OK, rebooting...");
            delay(300);
            ESP.restart();
        },
        []() {
            HTTPUpload &up = s_ap_server.upload();
            if (up.status == UPLOAD_FILE_START) {
                if (s_task_busB) vTaskSuspend(s_task_busB);
                if (Serial) {
                    char t[48];
                    snprintf(t, sizeof(t), "# AP-OTA: start  file=%s", up.filename.c_str());
                    Serial.println(t);
                }
                Update.begin(UPDATE_SIZE_UNKNOWN);
            } else if (up.status == UPLOAD_FILE_WRITE) {
                Update.write(up.buf, up.currentSize);
            } else if (up.status == UPLOAD_FILE_END) {
                Update.end(true);
                if (Serial) {
                    char t[48];
                    snprintf(t, sizeof(t), "# AP-OTA: done  %u bytes", up.totalSize);
                    Serial.println(t);
                }
            }
        }
    );
    s_ap_server.begin();
    if (Serial) Serial.println(F("# AP-OTA: server ready → http://192.168.4.1"));

    for (;;) {
        s_ap_server.handleClient();
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}
#endif  // AP_OTA_ENABLED


static void handle_cmd(char cmd) {
    switch (cmd) {

    // ── R：软复位 ─────────────────────────────────────────
    case 'R': case 'r':
        Serial.println(F("# CMD: Reset"));
        if (s_task_busB) vTaskSuspend(s_task_busB);
        tca_close_all();
        s_wire_portA = 0xFF; s_wire_portB = 0xFF;
        delay(1000);
        init_all();
        s_wire_portA = 0xFF; s_wire_portB = 0xFF;
        if (s_task_busB) vTaskResume(s_task_busB);
        Serial.println(F("# Reset done"));
        break;

    // ── Z：Yaw 归零提示 ──────────────────────────────────
    case 'Z': case 'z':
        Serial.println(F("# CMD: Yaw zero (use host [中立位] button)"));
        break;

    // ── S：保存 DCD ──────────────────────────────────────
    case 'S': case 's':
        Serial.println(F("# CMD: Save DCD..."));
        if (s_task_busB) vTaskSuspend(s_task_busB);
        for (uint8_t ch = 0; ch < NUM_CH; ch++) {
            if (!(s_live & (uint32_t)(1u << ch))) continue;
            tca_select(ch);
            s_bno[ch].saveCalibration();
            delay(10);
        }
        tca_close_all();
        s_wire_portA = 0xFF; s_wire_portB = 0xFF;
        if (s_task_busB) vTaskResume(s_task_busB);
        Serial.println(F("# DCD saved"));
        break;

    // ── C：查询精度 ──────────────────────────────────────
    case 'C': case 'c':
        Serial.print(F("# ACC:"));
        for (uint8_t ch = 0; ch < NUM_CH; ch++) {
            if (ch > 0) Serial.print(',');
            Serial.print(F("ch")); Serial.print(ch); Serial.print('='); Serial.print(s_acc[ch]);
        }
        Serial.println();
        Serial.print(F("# live_mask=0x")); Serial.println(s_live, HEX);
        Serial.println(F("# path=Wire-dual-DIY"));
        break;

    // ── F / M：循环报告类型 ──────────────────────────────
    case 'F': case 'f': case 'M': case 'm':
        s_rpt = (s_rpt + 1) % RPT_COUNT;
        { char tmp[48]; snprintf(tmp, sizeof(tmp), "# REPORT: %s", RPT_NAMES[s_rpt]); Serial.println(tmp); }
        if (s_task_busB) vTaskSuspend(s_task_busB);
        init_all();
        s_wire_portA = 0xFF; s_wire_portB = 0xFF;
        if (s_task_busB) vTaskResume(s_task_busB);
        prefs_save();
        break;

    // ── H：循环切换 I2C 频率 ─────────────────────────────
    case 'H': case 'h':
        s_i2c_idx = (s_i2c_idx + 1) % I2C_FREQ_COUNT;
        { char tmp[48]; snprintf(tmp, sizeof(tmp), "# I2C: %s", I2C_FREQ_NAMES[s_i2c_idx]); Serial.println(tmp); }
        if (s_task_busB) vTaskSuspend(s_task_busB);
        Wire.setClock(I2C_FREQ); Wire1.setClock(I2C_FREQ);
        init_all();
        s_wire_portA = 0xFF; s_wire_portB = 0xFF;
        if (s_task_busB) vTaskResume(s_task_busB);
        prefs_save();
        break;

    // ── A：切换 ASCII/Binary 输出模式 ────────────────────
    case 'A': case 'a':
        s_ascii_mode = !s_ascii_mode;
        Serial.print(F("# MODE: "));
        Serial.println(s_ascii_mode ? F("ASCII (debug)") : F("Binary (91B/frame)"));
        break;

    // ── 0-3：选择滤波预设 ────────────────────────────────
    case '0': case '1': case '2': case '3': {
        uint8_t idx = (uint8_t)(cmd - '0');
        if (idx < FILTER_COUNT) {
            s_flt_idx = idx;
            char tmp[48]; snprintf(tmp, sizeof(tmp), "# FILTER: %s", FILTER_PRESETS[s_flt_idx].name);
            Serial.println(tmp);
            prefs_save();
        }
        break;
    }

    // ── O：打印 OTA / WiFi 状态 ─────────────────────────
    case 'O': case 'o':
#if OTA_ENABLED
        {
            char tmp[80];
            snprintf(tmp, sizeof(tmp), "# OTA: WiFi=%s  IP=%s  host=%s",
                     WiFi.status() == WL_CONNECTED ? "connected" : "disconnected",
                     WiFi.status() == WL_CONNECTED ? WiFi.localIP().toString().c_str() : "N/A",
                     s_ble_full_name);
            Serial.println(tmp);
        }
#elif AP_OTA_ENABLED
        Serial.println(F("# OTA: AP mode  SSID=" AP_OTA_SSID "  http://192.168.4.1"));
#else
        Serial.println(F("# OTA: disabled (OTA_ENABLED=0 AP_OTA_ENABLED=0)"));
#endif
        break;

    // ── G：手动触发 GitHub OTA 版本检查 ─────────────────
    case 'G': case 'g':
#if GH_OTA_ENABLED
        gh_ota_check();
#else
        if (Serial) Serial.println(F("# GH_OTA: disabled (GH_OTA_ENABLED=0)"));
#endif
        break;

    // ── B：打印 BLE 状态 / 连接间隔 / notify Hz ──────────
    case 'B': case 'b': {
        uint32_t now_b    = millis();
        uint32_t elapsed  = now_b > s_ble_last_ms ? now_b - s_ble_last_ms : 1;
        uint32_t ndiff    = s_ble_notify_cnt - s_ble_last_cnt;
        uint32_t nhz10    = ndiff * 10000UL / elapsed;   // Hz×10
        char tmp[112];
        snprintf(tmp, sizeof(tmp),
                 "# BLE: %s  notify=%lu  notify_Hz=%lu.%lu  CI=%u(%.1fms)  fpn=%u  drop=%lu",
                 s_ble_connected ? "connected" : "advertising",
                 s_ble_notify_cnt, nhz10/10, nhz10%10,
                 s_ble_conn_interval, s_ble_conn_interval * 1.25f,
                 s_ble_fpn, s_ble_drop_cnt);
        Serial.println(tmp);
        s_ble_last_cnt = s_ble_notify_cnt;
        s_ble_last_ms  = now_b;
        break;
    }
    }
}


// ==============================================================
// setup()
// ==============================================================
void setup() {
    // ── LED 初始化（红灯常亮 = 初始化中）───────────────────
    pinMode(LED_R, OUTPUT); pinMode(LED_B, OUTPUT); pinMode(LED_G, OUTPUT);
    digitalWrite(LED_R, LOW); digitalWrite(LED_B, HIGH); digitalWrite(LED_G, HIGH);

    // ── 按键 + ADC 引脚初始化 ─────────────────────────────
    pinMode(PIN_BTN, INPUT_PULLUP);
    analogReadResolution(12);   // 12-bit ADC（0-4095）

    Serial.begin(2000000);
#if ARDUINO_USB_CDC_ON_BOOT
    Serial.setTxTimeoutMs(0);       // USB CDC 非阻塞：无串口连接时写入直接丢弃，不卡 loop
#endif
    delay(100);

    s_main_task = xTaskGetCurrentTaskHandle();

    // ── NVS 读取持久化设置 ───────────────────────────────────
    s_prefs.begin("imu_cfg", true);
    s_rpt     = s_prefs.getUChar("rpt", 0);    // 默认 ARVR_RV
    s_i2c_idx = s_prefs.getUChar("i2c", 3);    // 默认 1MHz
    s_flt_idx = s_prefs.getUChar("flt", 2);    // 默认 gesture
    s_prefs.getString("sn", s_sn, sizeof(s_sn));  // 默认 "001"
    s_prefs.end();

    // ── BLE 初始化（NimBLE，在传感器初始化前启动，立即开始广播）──
    ble_init();
    s_ble_last_ms = millis();

    // ── LED 任务提前启动：初始化期间显示蓝 0.5Hz（BLE 广播中）──────
    // 15×BNO080 初始化可能耗时数十秒；提前创建 task_led_fn 使 LED 在
    // 整个初始化过程中持续闪烁，可直观确认设备存活。
    // （task_led_fn 仅调用 digitalWrite，不使用 Wire，无冲突）
    led_off();
    xTaskCreatePinnedToCore(
        task_led_fn, "led",
        1024, NULL, 10,
        &s_task_led, 1   // Core 1 (APP_CPU)，优先级 10 > loop 的 1
    );

    // ── Wire A 初始化（GPIO8/9 → TCA1 → CH0-7）──────────────
    Wire.begin(PIN_SDA, PIN_SCL);
    Wire.setClock(I2C_FREQ);
    Wire.setTimeOut(10);

    // ── Wire B 初始化（GPIO13/14 → TCA2 → CH8-15）───────────
    Wire1.begin(PIN_SDA2, PIN_SCL2);
    Wire1.setClock(I2C_FREQ);
    Wire1.setTimeOut(10);

    // ── 关闭所有 TCA 端口，等待 BNO080 稳定 ─────────────────
    tca_close_all();
    delay(800);

    // ── NLERP 状态初始化 ────────────────────────────────────
    for (uint8_t ch = 0; ch < NUM_CH; ch++) nlerp_reset(ch);

    // ── 启动信息 ─────────────────────────────────────────────
    Serial.println();
    Serial.println(F("# BNO080_HAND_DIY " FW_VER " @ 2Mbaud | 自研 ESP32-S3 PCB | 2xTCA9548A | 16xBNO080"));
    Serial.println(F("# ver: BNO080/BNO085 ESP32-S3 " FW_VER));
    Serial.println(F("# BusA: SDA=GPIO8  SCL=GPIO9  → TCA1(0x77) → CH0-7   [Core 1]"));
    Serial.println(F("# BusB: SDA=GPIO13 SCL=GPIO14 → TCA2(0x70) → CH8-15  [Core 0]"));
    Serial.println(F("# LED:  IO21=Red(R42/240Ω) IO48=Green(R43/62Ω) IO47=Blue(R44/62Ω) 共阳（实测核查）"));
    Serial.println(F("#       蓝0.5Hz=BLE广播 红4Hz=无传感器 青1Hz=在线未校准 绿/黄1Hz=已校准电量良/中 红双闪=低电"));
    Serial.println(F("#       白色2Hz闪=校准模式（长按按键进入）"));
    Serial.println(F("# BLE:  SVC=0xFFE0  DATA=0xFFE1(NOTIFY)  CMD=0xFFE2(WRITE)  INFO=0xFFE3(READ)"));
    Serial.println(F("# Binary frame: [0xAA 0x55][seq:4B][ts:4B][16ch×5B][xor:1B] = 91B  (v2.10 int8 ×127)"));
#if OTA_ENABLED
    Serial.println(F("# OTA:  ArduinoOTA 启用 (SSID=" OTA_SSID ")，连接中..."));
    Serial.println(F("#       Arduino IDE → 端口 → BNO_HAND_xxx (OTA) → 上传"));
  #if GH_OTA_ENABLED
    Serial.println(F("# GH_OTA: 启用 → 连上 WiFi 后自动检查 GitHub version.txt"));
    Serial.println(F("#         仓库: github.com/liujycode/ESP32_IMU  命令 G=手动检查"));
  #endif
#elif AP_OTA_ENABLED
    Serial.println(F("# OTA:  AP 热点模式 SSID=" AP_OTA_SSID "，连接后浏览器 http://192.168.4.1"));
#else
    Serial.println(F("# OTA:  disabled (set OTA_ENABLED=1 to enable)"));
#endif
    Serial.println(F("# commands: R=reset  Z=yaw_zero  S=save_dcd  C=query_acc"));
    Serial.println(F("#           F/M=cycle_report  H=cycle_i2c_freq  A=toggle_ascii  B=ble_status  O=ota_status  G=gh_ota"));
    Serial.println(F("#           0-3=filter_preset  N<SN>=set_serial_no (e.g. N001, reboot)"));
    Serial.println(F("# BTN:  长按 GPIO12（PIN_BTN）>= 3s 进入校准模式，acc>=2 自动保存 DCD"));
    Serial.println(F("# VOLT: 每 5s 串口输出 # VOLT: bat=xxxmV usb=xxxmV（TODO: 核对 ADC 引脚与分压比）"));
    {
        char tmp[80];
        snprintf(tmp, sizeof(tmp), "# default: rpt=%s  i2c=%s  flt=%s  mode=binary  sn=%s  ble=%s",
                 RPT_NAMES[s_rpt], I2C_FREQ_NAMES[s_i2c_idx], FILTER_PRESETS[s_flt_idx].name,
                 s_sn, s_ble_full_name);
        Serial.println(tmp);
    }

    // ── I2C 总线扫描 ────────────────────────────────────────
    Serial.println(F("# Bus A scan (GPIO8/9 → TCA1 0x77):"));
    bool tca1_ok = false;
    for (uint8_t a = 1; a < 128; a++) {
        Wire.beginTransmission(a);
        if (Wire.endTransmission() == 0) {
            Serial.print(F("#  0x")); if (a < 0x10) Serial.print('0'); Serial.print(a, HEX);
            if (a == TCA1_ADDR) { Serial.print(F(" <- TCA1")); tca1_ok = true; }
            Serial.println();
        }
    }
    if (!tca1_ok) Serial.println(F("# WARNING: TCA1(0x77) not found on BusA!"));

    Serial.println(F("# Bus B scan (GPIO13/14 → TCA2 0x70):"));
    bool tca2_ok = false;
    for (uint8_t a = 1; a < 128; a++) {
        Wire1.beginTransmission(a);
        if (Wire1.endTransmission() == 0) {
            Serial.print(F("#  0x")); if (a < 0x10) Serial.print('0'); Serial.print(a, HEX);
            if (a == TCA2_ADDR) { Serial.print(F(" <- TCA2")); tca2_ok = true; }
            Serial.println();
        }
    }
    if (!tca2_ok) Serial.println(F("# WARNING: TCA2(0x70) not found on BusB! (CH8-15 will be MISS)"));

    // ── 各通道子总线扫描 ────────────────────────────────────
    Serial.println(F("# Per-channel scan:"));
    for (uint8_t ch = 0; ch < NUM_CH; ch++) {
        tca_select(ch); delay(2);
        Serial.print(F("#  CH")); Serial.print(ch); Serial.print(':');
        TwoWire &w = (CH_TCA[ch] == 0) ? Wire : Wire1;
        bool found = false;
        for (uint8_t a = 1; a < 128; a++) {
            if (a == TCA1_ADDR || a == TCA2_ADDR) continue;
            w.beginTransmission(a);
            if (w.endTransmission() == 0) {
                Serial.print(F(" 0x")); if (a < 0x10) Serial.print('0'); Serial.print(a, HEX);
                found = true;
            }
        }
        if (!found) Serial.print(F(" (none)"));
        Serial.println();
        tca_close_all(); delay(2);
    }

    // ── 初始化所有 BNO080 ────────────────────────────────────
    Serial.println(F("# Initializing BNO080 channels (3-round retry)..."));
    init_all();

    // ── FreeRTOS Core 0 任务：轮询 Bus B TCA2 CH8-15 ────────
    xTaskCreatePinnedToCore(
        task_poll_busB, "poll_busB",
        4096, NULL, 3,
        &s_task_busB, 0   // Core 0 (PRO_CPU)，优先级 3 < NimBLE host(5) 让出 CPU
    );

    // ── OTA 任务（Core 0，低优先级，与 NimBLE 共存）───────────
#if OTA_ENABLED
    xTaskCreatePinnedToCore(
        task_ota_fn, "ota",
        8192, NULL, 1,          // 8K stack：WiFi + TLS + ArduinoOTA 回调
        &s_task_ota, 0          // Core 0，优先级 1 < NimBLE host(5)
    );
#endif
#if AP_OTA_ENABLED
    xTaskCreatePinnedToCore(
        task_ap_ota_fn, "ap_ota",
        8192, NULL, 1,
        &s_task_ap_ota, 0
    );
#endif

    s_last_ms = millis(); s_last_cnt = 0;
    Serial.println(F("# Ready. Streaming binary frames (send 'A' to switch ASCII)..."));

    // 初始 ADC 读取（校准完成后 LED 常显电量，确保首帧有数据）
    {
        uint32_t br = (uint32_t)analogRead(PIN_BAT_ADC);
        uint32_t ur = (uint32_t)analogRead(PIN_USB_ADC);
        s_bat_mv = (float)br * ADC_REF_MV / 4095.0f * BAT_DIV_RATIO;
        s_usb_mv = (float)ur * ADC_REF_MV / 4095.0f * USB_DIV_RATIO;
    }
    s_adc_last_ms = millis();
}


// ==============================================================
// loop()  — Core 1 (APP_CPU)
// 双核并行流程：
//   1. 触发 Core 0 轮询 CH8-15（Bus B/Wire1）
//   2. Core 1 并行轮询 CH0-7（Bus A/Wire）
//   3. 等待 Core 0 完成
//   4. 组帧输出
// ==============================================================

// ── 串口行命令处理（支持多字符命令）──────────────────────────
static void handle_line(const char *line) {
    char first = line[0];
    // N<SN>：设置序列号（最长 8 字符，写 NVS 后重启）
    if (first == 'N' || first == 'n') {
        const char *sn = line + 1;
        if (sn[0] == '\0') {
            // N 单独：打印当前 SN
            char tmp[32]; snprintf(tmp, sizeof(tmp), "# SN: %s", s_sn);
            Serial.println(tmp);
        } else if (strlen(sn) > 8) {
            Serial.println(F("# SN: too long (max 8 chars)"));
        } else {
            strncpy(s_sn, sn, sizeof(s_sn) - 1);
            s_sn[sizeof(s_sn) - 1] = '\0';
            s_prefs.begin("imu_cfg", false);
            s_prefs.putString("sn", s_sn);
            s_prefs.end();
            char tmp[48]; snprintf(tmp, sizeof(tmp), "# SN: %s saved, rebooting...", s_sn);
            Serial.println(tmp);
            delay(300);
            ESP.restart();
        }
        return;
    }
    // 其他：沿用单字符命令
    handle_cmd(first);
}
void loop() {

    // ── 1. 串口命令处理（行缓冲，支持 N<SN> 多字符命令）────
    while (Serial.available() > 0) {
        char c = (char)Serial.read();
        if (c == '\n' || c == '\r') {
            if (s_serial_pos > 0) {
                s_serial_buf[s_serial_pos] = '\0';
                handle_line(s_serial_buf);
                s_serial_pos = 0;
            }
        } else if (s_serial_pos < (uint8_t)(sizeof(s_serial_buf) - 1)) {
            s_serial_buf[s_serial_pos++] = c;
        }
    }

    // ── 1b. BLE CI 补发（连接 3s 后再请求一次，避免首次被 Windows 忽略）──
    if (s_ble_ci_retry_ms && millis() >= s_ble_ci_retry_ms) {
        s_ble_ci_retry_ms = 0;
        if (s_ble_connected && s_ble_conn_handle != 0xFFFF) {
            s_ble_server->updateConnParams(s_ble_conn_handle, 6, 24, 0, 400);
            // onConnParamsUpdate 可能未触发，按当前记录的 CI 重算一次 fpn
            {
                uint32_t ci_ms = (uint32_t)s_ble_conn_interval * 5 / 4;
                if (ci_ms >= REPORT_INTERVAL_MS) {
                    uint8_t fpn = (uint8_t)((ci_ms + REPORT_INTERVAL_MS - 1) / REPORT_INTERVAL_MS);
                    if (fpn < 1)           fpn = 1;
                    if (fpn > BLE_FPN_MAX) fpn = BLE_FPN_MAX;
                    if (fpn != s_ble_fpn) { s_ble_fpn = fpn; s_ble_pkt_n = 0; }
                }
            }
            if (Serial) {
                char tmp[72];
                snprintf(tmp, sizeof(tmp),
                         "# BLE: CI retry CI=%u(%.1fms) fpn=%u",
                         s_ble_conn_interval, s_ble_conn_interval * 1.25f, s_ble_fpn);
                Serial.println(tmp);
            }
        }
    }

    uint32_t t_loop_start = micros();

    // ── 2. 触发 Core 0 并行轮询 CH8-15 ──────────────────
    uint32_t t_poll_start = micros();
    if (s_task_busB) xTaskNotifyGive(s_task_busB);

    // Core 1 轮询 CH0-7（Bus A/Wire/TCA1 0x70）
    for (uint8_t ch = 0; ch < 8; ch++) {
        if ((uint32_t)(1u << ch) & DISABLED_CH_MASK) continue;   // 跳过永久禁用通道
        bool ch_live = (s_live    >> ch) & 1u;
        bool ch_pend = (s_pending >> ch) & 1u;
        if (!ch_live && !ch_pend) continue;

        wire_tca_select_A(CH_PORT[ch]);

        float qw, qx, qy, qz; uint8_t acc; bool rst = false;
        int8_t rr = wire_bno_read(Wire, &qw, &qx, &qy, &qz, &acc, &rst);
        if (rr > 0) {
            s_fail_cnt[ch] = 0;
            s_acc[ch] = acc;
            s_qw[ch]=qw; s_qx[ch]=qx; s_qy[ch]=qy; s_qz[ch]=qz;
            nlerp_update(ch, qw, qx, qy, qz);
            s_last_data_ms[ch] = millis();
            if (ch_pend) {
                // 收到真实数据，从 pending 晋升为 live
                s_pending  &= ~(uint32_t)(1u << ch);
                s_live     |=  (uint32_t)(1u << ch);
                s_ever_live |= (uint32_t)(1u << ch);
                nlerp_reset(ch);
                Serial.print(F("# CH")); Serial.print(ch); Serial.println(F(" online"));
            }
        } else if (rr < 0 && ch_live) {
            if (++s_fail_cnt[ch] >= OFFLINE_THRESH) {
                s_live &= ~(uint32_t)(1u << ch);
                s_fail_cnt[ch] = 0;
                Serial.print(F("# CH")); Serial.print(ch); Serial.println(F(" offline"));
            }
        }
        if (rst && ch_live) {
            Serial.print(F("# WARN: CH")); Serial.print(ch); Serial.println(F(" reset, re-enabling"));
            vTaskDelay(pdMS_TO_TICKS(2));   // BNO080 收到 RESET_COMPLETE 即可接指令，2ms 足够
            s_fail_cnt[ch] = 0;
            wire_tca_select_A(CH_PORT[ch]);
            enable_report(ch);
            s_sq_inited[ch] = false;
            s_last_data_ms[ch] = millis();
        }
    }

    // 等待 Core 0 完成 CH8-15 轮询（超时 20ms 防死锁）
    if (s_task_busB) ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(20));
    uint32_t poll_us = micros() - t_poll_start;
    if (poll_us < s_poll_min_us) s_poll_min_us = poll_us;
    if (poll_us > s_poll_max_us) s_poll_max_us = poll_us;
    s_poll_sum_us += poll_us;

    // ── 2b. 数据超时：live 通道若 DATA_STALE_MS 内无 rr>0 → 离线 ─
    {
        uint32_t now_stale = millis();
        for (uint8_t ch = 0; ch < NUM_CH; ch++) {
            if (!(s_live & (uint32_t)(1u << ch))) continue;
            if (s_last_data_ms[ch] == 0) continue;   // 尚未赋初值，跳过
            if (now_stale - s_last_data_ms[ch] > DATA_STALE_MS) {
                s_live &= ~(uint32_t)(1u << ch);
                s_fail_cnt[ch] = 0;
                s_wire_portA = 0xFF; s_wire_portB = 0xFF;
                Serial.print(F("# CH")); Serial.print(ch); Serial.println(F(" stale→offline"));
            }
        }
    }

    // ── 3. 组帧输出：仅 BLE（串口不发送数据）────────────────────
    bool frame_sent = false;
    if (s_ble_connected) {
        build_binary_frame();
        notify_ble_frame();
        frame_sent = true;
    }

    uint32_t frame_us = micros() - t_loop_start;
    if (frame_sent) {
        if (frame_us < s_frame_min_us) s_frame_min_us = frame_us;
        if (frame_us > s_frame_max_us) s_frame_max_us = frame_us;
        s_frame_sum_us += frame_us;
        s_pkt_cnt++;
    }

    // ── 4. 采样率统计（每 5 秒）──────────────────────────
    uint32_t now_ms = millis();
    if (now_ms - s_last_ms >= 5000UL) {
        uint32_t elapsed   = now_ms - s_last_ms;
        uint32_t cnt_diff  = s_pkt_cnt - s_last_cnt;
        uint32_t fps10     = cnt_diff * 10000UL / elapsed;
        uint32_t poll_avg  = cnt_diff ? s_poll_sum_us  / cnt_diff : 0;
        uint32_t frame_avg = cnt_diff ? s_frame_sum_us / cnt_diff : 0;
        char tmp[320];
        snprintf(tmp, sizeof(tmp),
                 "# STAT: %lu.%lu Hz  pkt=%lu  live=0x%04X  rpt=%s  i2c=%s  flt=%s  mode=%s"
                 "  ble=%s(CI=%u/%.1fms)  notify=%lu"
                 "  poll=%lu/%lu/%luus  frame=%lu/%lu/%luus",
                 fps10/10, fps10%10, s_pkt_cnt,
                 (unsigned)s_live,
                 RPT_NAMES[s_rpt],
                 I2C_FREQ_NAMES[s_i2c_idx],
                 FILTER_PRESETS[s_flt_idx].name,
                 s_ascii_mode ? "ASCII" : "Bin",
                 s_ble_connected ? "conn" : "adv",
                 s_ble_conn_interval, s_ble_conn_interval * 1.25f,
                 s_ble_notify_cnt,
                 s_poll_min_us, poll_avg, s_poll_max_us,
                 s_frame_min_us, frame_avg, s_frame_max_us);
        Serial.println(tmp);
        // INFO 特征每 5s 刷新为 STAT 摘要（PC 端无需串口即可读取实际 CI/fps）
        if (s_ble_connected && s_ble_info_chr) {
            char istat[64];
            snprintf(istat, sizeof(istat),
                     "fps=%lu.%lu CI=%u/%.1fms notify=%lu poll=%luus",
                     fps10/10, fps10%10,
                     s_ble_conn_interval, s_ble_conn_interval * 1.25f,
                     s_ble_notify_cnt, poll_avg);
            s_ble_info_chr->setValue((uint8_t*)istat, strlen(istat));
        }
        s_last_ms  = now_ms;
        s_last_cnt = s_pkt_cnt;
        s_poll_min_us  = 0xFFFFFFFFUL; s_poll_max_us  = 0; s_poll_sum_us  = 0;
        s_frame_min_us = 0xFFFFFFFFUL; s_frame_max_us = 0; s_frame_sum_us = 0;
    }

    // ── 5. 热插拔：定期探测并重初始化离线通道 ────────────
    uint32_t now_probe = millis();
    uint32_t offline = s_ever_live & ~s_live & ~s_pending;   // 排除已在 pending 的通道
    if (offline && (now_probe - s_probe_ms >= PROBE_INTERVAL_MS)) {
        s_probe_ms = now_probe;
        if (s_task_busB) vTaskSuspend(s_task_busB);
        for (uint8_t ch = 0; ch < NUM_CH; ch++) {
            if (!(offline & (1u << ch))) continue;
            if (init_channel(ch)) {
                // 进入 pending 状态，等待 rr>0 真实数据确认后才晋升 s_live
                s_pending |= (uint32_t)(1u << ch);
                s_pending_ms[ch] = millis();
                s_fail_cnt[ch] = 0;
                nlerp_reset(ch);
                Serial.print(F("# CH")); Serial.print(ch); Serial.println(F(" pending..."));
            }
        }
        s_wire_portA = 0xFF; s_wire_portB = 0xFF;
        if (s_task_busB) vTaskResume(s_task_busB);
    }

    // ── 5b. pending 超时：500ms 无 rr>0 确认 → 丢弃（假 ACK 通道）──
    {
        uint32_t now_pend = millis();
        for (uint8_t ch = 0; ch < NUM_CH; ch++) {
            if (!((s_pending >> ch) & 1u)) continue;
            if (now_pend - s_pending_ms[ch] > PENDING_CONFIRM_MS) {
                s_pending &= ~(uint32_t)(1u << ch);
                Serial.print(F("# CH")); Serial.print(ch); Serial.println(F(" pending→drop(no data)"));
            }
        }
    }

    // ── 6. 按键轮询 ──────────────────────────────────────────────
    //   非校准模式：长按 BTN_HOLD_MS(3s) → 进入校准模式
    //   校准模式中：任意按下后松开 → 取消校准（不保存 DCD）
    //   注意：触发校准的长按本身松开时不触发取消（s_cal_waiting_release 保护）
    {
        bool btn_now = (digitalRead(PIN_BTN) == LOW);
        if (btn_now && !s_btn_pressed) {
            // 下降沿：记录按下时刻
            s_btn_down_ms = millis();
            s_btn_pressed = true;
        } else if (!btn_now && s_btn_pressed) {
            // 上升沿：松开
            s_btn_pressed = false;
            if (s_cal_waiting_release) {
                // 触发校准的那次长按刚松开，忽略，等待下一次新按键
                s_cal_waiting_release = false;
            } else if (s_cal_mode) {
                // 校准模式中的新一次按键松开 → 取消校准，不保存 DCD
                s_cal_mode = false;
                if (Serial) Serial.println(F("# CAL: 已取消，DCD 未保存"));
            }
        }
        // 非校准模式下长按达到阈值 → 进入校准模式
        if (s_btn_pressed && !s_cal_mode && (millis() - s_btn_down_ms >= BTN_HOLD_MS)) {
            s_cal_mode            = true;
            s_cal_entry_ms        = millis();
            s_cal_waiting_release = true;   // 等待本次长按松开后才响应取消操作
            s_btn_pressed         = false;
            if (Serial) Serial.println(F("# CAL: 校准模式已启动，摇动手部完成校准，acc>=2 自动保存..."));
        }
    }

    // ── 7. 校准完成检测：所有在线通道 acc >= CAL_ACC_THRESH ──
    if (s_cal_mode) {
        uint8_t live_cnt = 0, cal_cnt = 0;
        for (uint8_t ch = 0; ch < NUM_CH; ch++) {
            if ((s_live >> ch) & 1u) {
                live_cnt++;
                if (s_acc[ch] >= CAL_ACC_THRESH) cal_cnt++;
            }
        }
        if (live_cnt > 0 && cal_cnt >= live_cnt &&
            (millis() - s_cal_entry_ms >= CAL_MIN_MS)) {
            Serial.println(F("# CAL: 校准完成，自动保存 DCD..."));
            if (s_task_busB) vTaskSuspend(s_task_busB);
            for (uint8_t ch = 0; ch < NUM_CH; ch++) {
                if (!(s_live & (uint32_t)(1u << ch))) continue;
                tca_select(ch);
                s_bno[ch].saveCalibration();
                delay(10);
            }
            tca_close_all();
            s_wire_portA = 0xFF; s_wire_portB = 0xFF;
            if (s_task_busB) vTaskResume(s_task_busB);
            Serial.println(F("# CAL: DCD 保存完成"));
            s_cal_mode = false;
        }
    }

    // ── 8. 电压监测（每 ADC_REPORT_MS ms 串口输出）─────────
    {
        uint32_t now_adc = millis();
        if (now_adc - s_adc_last_ms >= ADC_REPORT_MS) {
            s_adc_last_ms = now_adc;
            uint32_t bat_raw = (uint32_t)analogRead(PIN_BAT_ADC);
            uint32_t usb_raw = (uint32_t)analogRead(PIN_USB_ADC);
            s_bat_mv = (float)bat_raw * ADC_REF_MV / 4095.0f * BAT_DIV_RATIO;
            s_usb_mv = (float)usb_raw * ADC_REF_MV / 4095.0f * USB_DIV_RATIO;
            char tmp[64];
            snprintf(tmp, sizeof(tmp), "# VOLT: bat=%.0fmV usb=%.0fmV", s_bat_mv, s_usb_mv);
            Serial.println(tmp);
        }
    }

    // ── 9. LED 状态由独立任务 task_led_fn 驱动，此处无需调用 ──
}
