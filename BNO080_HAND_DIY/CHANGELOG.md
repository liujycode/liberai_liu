# BNO080_HAND_DIY 版本变更记录

固件：`BNO080_HAND_DIY.ino`  
硬件：自研 ESP32-S3 PCB + 2× TCA9548A + 最多 16× BNO080/BNO085  
配套上位机：IMU_Lab_CalibView / bno_hand_ble.py

---

## v2.35 — 2026-04-27
**修复 OTA 版本比较：防止降级**
- [修复] 原逻辑 `remote_ver == FW_VER` 才跳过，任何不一致（含远端比本地旧）都触发下载，导致 GitHub `version.txt` 未更新时把旧固件刷回来。
- [修复] 改为解析 `v{major}.{minor}` 数值比较，仅 `remote > local` 时才下载；远端等于或小于本地一律跳过，打印 `[up-to-date or newer, skip]`。

## v2.34 — 2026-04-27
**修复 OTA 看门狗重启死循环**
- [修复] 90s 看门狗触发 `ESP.restart()` → 重启后再次版本检查 → 再次下载 → 再次超时 → 彩虹灯永久闪（无限循环）。根因：未判断重启来源。
- [修复] `gh_ota_check()` 调用前检测 `esp_reset_reason()`：`ESP_RST_SW`（含看门狗超时 / OTA 烧录后重启）→ 跳过自动检查并打印提示；Power-on / 外部 reset → 正常检查。此后只有重新上电才会触发自动 OTA，避免死循环。

## v2.33 — 2026-04-27
**GH OTA 下载卡死根因修复**
- [修复] 下载失败（CDN + RAW 均失败）后 `s_ota_rainbow` 未清零，导致彩虹灯永久亮、误判"一直在下载"；现在两路均失败时正确置 `s_ota_rainbow = false`。
- [修复] `onProgress` 内的 3 分钟看门狗对 TLS 握手阶段卡死无效（握手期间 `onProgress` 永不触发）；改用 **FreeRTOS 软件定时器**（90s）在 `httpUpdate.update()` 阻塞外部强制 `ESP.restart()`。
- [优化] 移除 `setFollowRedirects`，避免被重定向到更慢节点；二进制下载固定 CDN→RAW 顺序（移除 `ver_ok_idx` 对下载路径的影响）。

## v2.32 — 2026-04-27
**GH OTA 卡死防护**
- [修复] `onProgress` 回调中加绝对时间保护：下载累计超过 3 分钟调用 `ESP.restart()` 跳过本次 OTA，避免弱网 / CDN 异常时永久卡在彩虹灯状态。
- [优化] 加 `setFollowRedirects(HTTPC_FORCE_FOLLOW_REDIRECTS)`，正确跟随 CDN 返回的 3xx 重定向。
- [优化] CDN socket 超时收紧到 10s（CDN 应快速响应），RAW 保持 30s；两路参数差异化。

## v2.31 — 2026-04-27
**GH OTA 下载提速**
- [优化] 版本检查时记录成功路径索引（`ver_ok_idx`），固件下载从同一路开始，跳过注定失败的路径，节省一次完整 TLS 握手（~2-3s）。

## v2.30 — 2026-04-27
**修复 BLE 延迟广播：蓝灯先于紫灯出现**
- [修复] `setup()` 在 `xTaskCreatePinnedToCore(task_ota_fn)` 之前提前置位 `s_ota_checking=true`，消除 FreeRTOS 首次任务调度延迟导致的"蓝灯先闪"窗口。
- [修复] `ble_init()` 末尾添加 `NimBLEDevice::stopAdvertising()`，防止 NimBLE 的 `server->start()` 内部自动触发广播，绕过延迟逻辑。

## v2.29 — 2026-04-27
**OTA UX 优化：BLE 延迟广播 + GH OTA 下载进度**
- [优化] `OTA_ENABLED=1` 时 `ble_init()` 不再立即调用 `startAdvertising()`；广播推迟到 `task_ota_fn()` 完成 WiFi 连接 + 版本检查后再开启。消除"蓝灯已亮 → OTA 突然重启"引起的用户误判。
- [优化] WiFi 超时未连接时同样在 `vTaskDelete()` 前调用 `startAdvertising()`，保证任意路径下 BLE 广播都能启动。
- [优化] `gh_ota_check()` 固件下载前注册 `httpUpdate.onProgress()` 回调，每 5% 打印一次进度（格式：`# GH_OTA: 25%  320KB/1280KB  512kbps`）；`httpUpdate.setTimeout(30000)` 防止弱网卡死。

## v2.28 — 2026-04-27
**GH OTA 双路 fallback（CDN → RAW）**
- [新增] 版本检查和固件下载均先走 jsDelivr CDN（6s 超时），失败自动 fallback 到 `raw.githubusercontent.com`；串口打印 `[CDN]` / `[RAW]` 标识实际用哪路。

## v2.27 — 2026-04-27
**恢复 OTA；换用 Minimal SPIFFS 分区表（1.9MB APP）**
- [修复] 分区表改为 Minimal SPIFFS，APP 分区 1.25MB→1.9MB，彻底解决 Flash 溢出。
- [恢复] `OTA_ENABLED` / `GH_OTA_ENABLED` 恢复为 1，ArduinoOTA + GitHub 自动 OTA 功能恢复。
- [操作] Arduino IDE → Tools → Partition Scheme → **Minimal SPIFFS (1.9MB APP with OTA/190KB SPIFFS)**，首次须 USB 烧录。

## v2.26 — 2026-04-27
**关闭 OTA 编译开关，恢复 Flash 余量**
- [修复] `OTA_ENABLED` / `GH_OTA_ENABLED` 置 0；WiFi + ArduinoOTA + WiFiClientSecure + HTTPUpdate 库从编译中移除，Flash 占用从 102% 降至正常范围。需要 OTA 时重新置 1 编译。

## v2.25 — 2026-04-27
**修复初次连接 INFO 被 STAT 立即覆盖**
- [修复] `onConnect()` 末尾新增 `s_last_ms = millis()` 重置 STAT 定时器。  
  根因：断开重连时 s_last_ms 保留上次 STAT 时间戳，距上次 STAT 已过 4+ 秒时，
  新连接后不足 1 秒 STAT 就覆盖 INFO；Mac/Bleak 初次配对 service discovery
  可能需要 2-5 秒，read_gatt_char() 时 INFO 已变成 STAT 数据，版本信息丢失。
  修复后：每次连接后保证有完整 5s 窗口供上位机读取版本字符串。

## v2.24 — 2026-04-24
**BLE 连接后推送版本信息；移除串口数据输出**
- [新增] `onConnect` 时立即将 INFO 特征（0xFFE3）刷新为版本字符串，格式：`BNO_HAND_001 v2.24 15ch BNO080`；上位机连接后 read INFO 即可获取固件版本，5s 后被 STAT 覆盖（正常）。
- [移除] `loop()` 中串口数据输出（write_ascii_frame / write_binary_frame）；串口仅保留状态/诊断打印（# STAT / # CH / # 等前缀行）。

## v2.23 — 2026-04-24
**LED 优先级修复：无传感器红灯高于 BLE 蓝灯**
- [修复] `led_tick()` 中"无传感器→红4Hz"判断提至"BLE广播→蓝0.5Hz"之前，消除两路状态同时触发时混色为粉色的问题。  
  新优先级：彩虹 > 紫(OTA检查) > 白(校准) > 红4Hz(无传感器) > 蓝(BLE广播) > 正常。

## v2.21 — 2026-04-24
**OTA 测试版本**
- [测试] 版本号 v2.20 → v2.21，用于验证 GitHub OTA 自动检测 + 彩虹 LED 流程。

## v2.20 — 2026-04-24
**修复 GitHub OTA URL + OTA 彩虹 LED**
- [修复] GH_VER_URL / GH_BIN_URL 由错误的 ESP32_IMU/main 改为 liberai_liu/master。
- [新增] OTA 检测到版本不符时，LED 切换为 7 色彩虹循环（200ms/色）直至烧录完成重启；移除原 httpUpdate.setLedPin() 蓝灯。
- [新增] 仓库根目录增加 version.txt 和 firmware/BNO080_HAND_DIY.bin，OTA 自动检查流程正式可用。

## v2.19 — 2026-04-23
**BLE 通知下限解耦：突破 100fps 上限**
- [根因] `notify_ble_frame()` 用 REPORT_INTERVAL_MS(10ms) 作 ci_ms 下限；WinRT 将 CI 压至 7.5ms 时仍被截断到 10ms，实际通知率限死在 100fps。
- [修复] 下限改为 BLE_NOTIFY_MIN_MS(2ms)，BNO080 输出率继续由 REPORT_INTERVAL_MS(10ms) 独立控制。  
  效果：CI=7.5ms(6单位) → ci_ms=7 > 2 → ~143fps；CI=10ms(8单位) → 100fps。

## v2.18 — 2026-04-23
**彻底消除卡死缓冲丢帧：改为速率限制单帧发送**
- [根因] v2.17 fpn=3 批量打包后，notify() 偶发失败时 pkt_n 停在 3，后续所有新帧全部丢弃，实测有效帧率仅 21fps / 延时 ~150ms。
- [修复] 彻底移除批量打包逻辑，改为速率限制单帧发送：每次直接用 s_binbuf（最新帧）；以 CI_ms = conn_interval × 1.25 作发送间隔上限；notify() 无论成败均推进计时器。  
  效果：CI=30ms → 33fps，CI=7.5ms → 100fps。

## v2.17 — 2026-04-23
**onConnect 读实际 CI → 彻底消除丢帧**
- [根因] v2.16 fpn 自适应依赖 onConnParamsUpdate 回调，Windows 接受 CI 更新时该回调经常不触发，fpn 停在默认值 1；CI=30ms+fpn=1 → mbuf 持续溢出，实测丢帧率 67~78%。
- [修复①] onConnect 改用 `info.getConnInterval()` 读取实际初始 CI，立即计算 fpn 并在 s_ble_connected=true 之前赋值，不再依赖 onConnParamsUpdate。
- [修复②] CI 补发定时器（3s 后）同步重算 fpn，覆盖极少数 onConnParamsUpdate 未触发的场景。
- [优化] 传感器复位等待 10ms → 2ms；'A' 命令帧大小勘误 155B → 91B。

## v2.16 — 2026-04-23
**BLE 丢帧修复：自适应 s_ble_fpn**
- [修复] 引入 `s_ble_fpn = ceil(CI_ms / REPORT_INTERVAL_MS)`，onConnParamsUpdate 更新。
- [修复] notify() 失败保留缓冲下次重发；s_ble_drop_cnt 统计溢出丢帧数。
- [参数] BLE_FPN_MAX=4（364B/notify，1 mbuf 内）。

## v2.15 — 2026-04-23
**GitHub 自动 OTA**
- [新增] GH_OTA_ENABLED=1：WiFi 连通后自动拉取 GitHub version.txt，版本不符则下载 firmware/BNO080_HAND_DIY.bin 并烧录重启。
- [新增] 串口命令 'G'：手动触发 GitHub 版本检查。
- [流程] 更新固件：编译 → 导出.bin → 放入仓库 firmware/ → 更新 version.txt → git push。

## v2.14 — 2026-04-23
**无线 OTA 烧录：ArduinoOTA + AP 网页上传**
- [新增] OTA_ENABLED=1：连局域网 WiFi 后 Arduino IDE 端口列表出现设备，一键推送。
- [新增] AP_OTA_ENABLED=0（预留）：开启后设备创建 AP 热点，浏览器 http://192.168.4.1 选 .bin 上传。
- [新增] 串口命令 'O'：打印 OTA/WiFi 状态。
- [规则] OTA_ENABLED 与 AP_OTA_ENABLED 不得同时置 1（共用无线模块）。
- [实现] OTA 运行于独立 FreeRTOS 任务（Core 0，优先级 1）；OTA 烧录开始时自动 vTaskSuspend(s_task_busB)。

## v2.13 — 2026-04-23
**STAT → INFO 特征，PC 端实时读取连接状态**
- [新增] 每 5s 将 STAT 摘要写入 INFO 特征（0xFFE3），格式：`fps=153.2 CI=16/20.0ms notify=12345 poll=180us`。
- [同步] imu_lab_calibview.py：BLE_INFO_UUID 修正为 0xFFE3；_ble_main() while 循环每 5s 读一次并 emit 'ble_stat'。

## v2.12 — 2026-04-22
**去固件限速**
- [优化] BLE_NOTIFY_MIN_MS 5→2：固件侧 notify 节流上限从 200fps 提至 500fps，实际 fps 由 CI/mbuf 决定。

## v2.11 — 2026-04-22
**100fps 双瓶颈修复**
- [修复①] REPORT_INTERVAL_MS 2→10：2ms 时 BNO080 ARVR_RV 几乎每次 poll 都在计算，时钟拉伸 ~3ms/通道，Bus B 8ch×3ms=24ms → 循环 38fps；改 10ms 后无新数据直接返回(36μs)，可达 100fps。
- [修复②] BLE_FRAMES_PER_NOTIFY 5→1：455B/notify 占 2 个 mbuf，并发只有 6 个 notify；改为 91B/notify 占 1 个 mbuf，并发 12 个，CI=24ms → 100fps+。

## v2.10 — 2026-04-22
**帧压缩：int8 四元数，91B/帧**
- [优化] 二进制帧 155B→91B：int16(×10000) → int8(×127)，精度 1/127≈0.5°；5B/通道×16通道+11B=91B。
- [同步] 串口二进制输出同步改为 91B（上位机同步更新）。

## v2.9 — 2026-04-22
**BLE 采样率优化：CI 范围请求 + 3s 补发**
- [优化] CI 请求从固定 6(7.5ms) 改为范围 6-24(7.5-30ms)，Windows 通常接受 CI=16-24，对应 100-150fps。
- [优化] 连接 3s 后自动补发一次 CI 参数更新请求，避免 Windows 忽略首次协商。

## v2.8 — 2026-04-22
**深度优化：编译修复 + 5 项运行时 Bug 修复**
- [修复] setup() setTxTimeoutMs() 用 `#if ARDUINO_USB_CDC_ON_BOOT` 保护。
- [修复①] Banner TCA 地址印反（0x77/0x70）已更正。
- [修复②] notify_ble_frame() 速率限制时丢帧：仅在发送成功后清零 s_ble_pkt_n。
- [修复③] task_poll_busB() delay(10) → vTaskDelay(pdMS_TO_TICKS(10))。
- [修复④] init_all() 三轮重试条件改为 EXPECTED_LIVE，CH7 禁用时不再空跑两轮，节省 ~700ms。
- [修复⑤] 新增 onConnParamsUpdate()，记录协商后实际 CI 值。

## v2.7 — 2026-04-22
**修复校准灯不亮 + BLE 广播名稳定**
- [修复] led_tick() 优先级：校准白灯提为最高优先级，BLE 状态判断降为次级。
- [修复] BLE 广播名恢复 adv->setName() 显式写入，Windows 被动扫描可靠收到设备名。

## v2.6 — 2026-04-22
**修复 BLE 扫描不到设备名**
- 广播包移除 128-bit UUID（18B），避免设备名被溢出至扫描响应包；Windows 被动扫描现可直接收到设备名。

## v2.5 — 2026-04-22
**宏定义默认序列号**
- [新增] DEVICE_SN_DEFAULT "001"，批量烧录时直接改宏即可。
- [精简] v1.x 历史注释合并为摘要。

## v2.4 — 2026-04-22
**设备序列号**
- [新增] NVS 键 "sn" 存储序列号（默认 "001"，最长 8 字符）；BLE 广播名 "BNO_HAND" → "BNO_HAND_\<SN\>"。
- [新增] 串口命令 N\<SN\>↵：写入 NVS 并重启生效。

## v2.3 — 2026-04-22
**低电双闪**
- 100ms 亮/灭/亮 + 1s 停，周期 1.3s，区别于 4Hz 均匀快闪。

## v2.2 — 2026-04-22
**串口断开稳定 + 初始化可见**
- Serial.enableReboot(false) 关闭 USB CDC 自动重启；LED 任务提前至 ble_init() 后创建，初始化期间持续显示蓝 0.5Hz。

## v2.1 — 2026-04-22
**CH7 永久禁用**
- DISABLED_CH_MASK 跳过未焊传感器，INFO 改 "15ch"。

## v2.0 — 2026-04-21
**BLE NimBLE 主数据通路**
- GATT 服务 0xFFE0：DATA 0xFFE1(NOTIFY) / CMD 0xFFE2(WRITE_NR) / INFO 0xFFE3(READ)。
- 请求 7.5ms 连接间隔；BLE_FRAMES_PER_NOTIFY=3 帧打包；BLE 断开自动回落串口。

## v1.0 ~ v1.14 — 2026-04-21
**硬件调试与基础功能建立（共 14 个版本）**
- 引脚/TCA 地址修正 → RGB LED 状态机 → 长按 3s 校准 → 热插拔 pending 防假 ACK → 数据超时离线 → FreeRTOS 双核并行 → 电压 ADC 监测。
