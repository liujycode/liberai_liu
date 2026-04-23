/*!
 * BTN_TEST.ino  v1.0  2026-04-21
 * 自研 ESP32-S3 PCB — 按键 GPIO 原始电平诊断固件
 *
 * 每 200ms 打印一次 GPIO0（SW2）当前电平。
 * 按下时应从 HIGH→LOW；若一直为 HIGH，说明按键未接通。
 *
 * -- 原理图说明 ---------------------------------
 *   SW2：3V3 → R41(10kΩ) → IO12；SW2另一端 → GND
 *   低电平有效（按下 = LOW）
 *   （原误标 GPIO0/BOOT，原理图核查后更正为 GPIO12）
 *
 * -- 串口输出 -----------------------------------
 *   GPIO0=HIGH  （未按下）
 *   GPIO0=LOW * （按下，带 * 标记）
 *   PRESS DETECTED  （检测到下降沿）
 *   RELEASE DETECTED held=XXXms  （检测到释放，含按住时长）
 */

#define PIN_BTN   12   // GPIO12 — SW2（R41 10kΩ 上拉，低电平有效）

static bool     s_prev      = HIGH;
static uint32_t s_down_ms   = 0;
static uint32_t s_print_ms  = 0;

void setup() {
    pinMode(PIN_BTN, INPUT_PULLUP);

    Serial.begin(2000000);
#if ARDUINO_USB_CDC_ON_BOOT
    Serial.setTxTimeoutMs(0);
#endif
    delay(200);

    Serial.println(F("# BTN_TEST v1.0 — GPIO12 原始电平诊断"));
    Serial.println(F("# 按下 SW2 应看到 GPIO12=LOW"));
    Serial.println(F("# 若始终为 HIGH → 按键未接通 / 引脚接错"));
    Serial.println(F("# ─────────────────────────────────────────"));
}

void loop() {
    uint32_t now = millis();
    bool cur = (digitalRead(PIN_BTN) == HIGH);   // HIGH=未按, LOW=按下

    // 边沿检测
    if (!cur && s_prev) {
        // 下降沿 = 按下
        s_down_ms = now;
        Serial.println(F("PRESS DETECTED"));
    } else if (cur && !s_prev) {
        // 上升沿 = 释放
        char buf[48];
        snprintf(buf, sizeof(buf), "RELEASE DETECTED held=%lums", (unsigned long)(now - s_down_ms));
        Serial.println(buf);
    }
    s_prev = cur;

    // 每 200ms 打印一次当前电平
    if (now - s_print_ms >= 200) {
        s_print_ms = now;
        if (!cur) {
            // 按下中
            char buf[40];
            snprintf(buf, sizeof(buf), "GPIO12=LOW *  (held %lums)", (unsigned long)(now - s_down_ms));
            Serial.println(buf);
        } else {
            Serial.println(F("GPIO12=HIGH"));
        }
    }
}
