/*!
 * LED_TEST.ino  v1.0  2026-04-21
 * 自研 ESP32-S3 PCB — RGB LED 管脚颜色核查固件
 *
 * 依次单独点亮 IO47 / IO48 / IO21，每步 2s，串口提示应亮颜色。
 * 根据实际看到的颜色与串口提示对比，判断原理图标注是否正确。
 *
 * -- 原理图标注（待核查）------
 *   IO47 → R44(62Ω)  → 应为 红(Red)
 *   IO48 → R43(62Ω)  → 应为 蓝(Blue)
 *   IO21 → R42(240Ω) → 应为 绿(Green)
 *   （共阳 RGB，低电平点亮）
 */

#define LED_R   47
#define LED_B   48
#define LED_G   21

#define STEP_MS  2000   // 每步显示 2 秒

struct Step {
    uint8_t     pin;
    const char *io_name;
    const char *expected_color;
};

static const Step STEPS[] = {
    { LED_R, "IO47", "RED   (红)" },
    { LED_B, "IO48", "BLUE  (蓝)" },
    { LED_G, "IO21", "GREEN (绿)" },
    // 混色验证
    { 0xFF, "IO47+IO48", "MAGENTA (品红 = 红+蓝)" },
    { 0xFF, "IO47+IO21", "YELLOW  (黄   = 红+绿)" },
    { 0xFF, "IO48+IO21", "CYAN    (青   = 蓝+绿)" },
    { 0xFF, "ALL",       "WHITE   (白   = 红+绿+蓝)" },
};
#define STEP_COUNT  7

static void all_off() {
    digitalWrite(LED_R, HIGH);
    digitalWrite(LED_B, HIGH);
    digitalWrite(LED_G, HIGH);
}

void setup() {
    pinMode(LED_R, OUTPUT);
    pinMode(LED_B, OUTPUT);
    pinMode(LED_G, OUTPUT);
    all_off();

    Serial.begin(2000000);
#if ARDUINO_USB_CDC_ON_BOOT
    Serial.setTxTimeoutMs(0);
#endif
    delay(200);

    Serial.println(F("# LED_TEST v1.0 — ESP32-S3 自研 PCB RGB 管脚核查"));
    Serial.println(F("# 串口提示 '点亮 IOxx -> 应为 XXX 色'，对比实际灯色"));
    Serial.println(F("# 若颜色不符，原理图/固件 define 有误，需更正 LED_R/LED_G/LED_B"));
    Serial.println(F("# ─────────────────────────────────────────────────────"));
}

static uint8_t  s_step    = 0;
static uint32_t s_step_ms = 0;

void loop() {
    uint32_t now = millis();

    // 初始触发
    if (s_step_ms == 0) {
        s_step_ms = now;
        // 打印第一步
        goto print_step;
    }

    if (now - s_step_ms < STEP_MS) return;

    s_step = (s_step + 1) % STEP_COUNT;
    s_step_ms = now;

print_step:
    all_off();

    switch (s_step) {
        case 0:  // IO47 单独
            digitalWrite(LED_R, LOW);
            break;
        case 1:  // IO48 单独
            digitalWrite(LED_B, LOW);
            break;
        case 2:  // IO21 单独
            digitalWrite(LED_G, LOW);
            break;
        case 3:  // IO47+IO48
            digitalWrite(LED_R, LOW);
            digitalWrite(LED_B, LOW);
            break;
        case 4:  // IO47+IO21
            digitalWrite(LED_R, LOW);
            digitalWrite(LED_G, LOW);
            break;
        case 5:  // IO48+IO21
            digitalWrite(LED_B, LOW);
            digitalWrite(LED_G, LOW);
            break;
        case 6:  // ALL
            digitalWrite(LED_R, LOW);
            digitalWrite(LED_B, LOW);
            digitalWrite(LED_G, LOW);
            break;
    }

    Serial.print(F("# 点亮 "));
    Serial.print(STEPS[s_step].io_name);
    Serial.print(F("  →  应为 "));
    Serial.println(STEPS[s_step].expected_color);
}
