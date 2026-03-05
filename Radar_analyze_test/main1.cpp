/**
 * HLK-LD6002 雷达 UART 通信诊断程序
 * 目标板：ESP32-WROOM (esp32dev)
 *
 * 诊断项目：
 *   1. 自环测试（GPIO16短接GPIO17）验证ESP32 UART本身是否正常
 *   2. 实际接收雷达原始字节并打印HEX
 *   3. 尝试多个波特率确认配置是否正确
 *   4. 统计每秒收到的字节数判断是否有数据流
 *
 * ===================== 接线检查 =====================
 * 正确的交叉接法（数据方向交叉）：
 *   ESP32 GPIO16 (UART RX) ←── KIT板 TX0  (雷达输出数据)
 *   ESP32 GPIO17 (UART TX) ──→ KIT板 RX0  (雷达接收命令，可不接)
 *   ESP32 GND              ───  KIT板 GND
 *
 * 注意：如果你之前的接法是 GPIO16→RX0, GPIO17→TX0，
 *       则 TX 对 TX、RX 对 RX，永远收不到数据，请对调。
 *
 * ===================== 自环测试接法 =====================
 * 拔掉所有雷达连线，用杜邦线短接 GPIO16 ↔ GPIO17，
 * 串口监视器看到 "LOOPBACK OK" 说明ESP32 UART正常。
 */

#include <Arduino.h>

// ==================== 配置 ====================
// 阶段1：自环测试时设为 true（GPIO16短接GPIO17，不接雷达）
// 阶段2：实际接雷达时设为 false
#define LOOPBACK_TEST   false

#define RX_PIN  2
#define TX_PIN  4

// 目标波特率（雷达规格书指定 1382400）
#define RADAR_BAUD  1382400

// 同时测试的备选波特率（用于排查配置问题）
static const uint32_t BAUD_CANDIDATES[] = {
    1382400,
    115200,
    921600,
    460800,
};
#define NUM_BAUD_CANDIDATES 4

// ==================== 全局 ====================
static uint32_t bytes_this_second = 0;
static uint32_t total_bytes       = 0;
static uint32_t last_stat_ms      = 0;

// ==================== 自环测试 ====================
static void run_loopback_test() {
    Serial.println("\n========== 自环测试模式 ==========");
    Serial.println("请确认 GPIO16 和 GPIO17 已用杜邦线短接。");
    Serial.println("测试将发送字节并检查是否回收到相同数据...\n");

    // 使用任意可用波特率做自环
    Serial2.begin(115200, SERIAL_8N1, RX_PIN, TX_PIN);
    delay(100);

    uint8_t tx_data[16] = {0x01,0x02,0x03,0x04,0x05,0xAA,0xBB,0xCC,
                           0xDD,0xEE,0xFF,0x10,0x20,0x30,0x40,0x50};
    uint8_t rx_data[16] = {};

    Serial2.write(tx_data, 16);
    delay(50);

    int recv = 0;
    uint32_t t = millis();
    while (millis() - t < 500) {
        while (Serial2.available() && recv < 16) {
            rx_data[recv++] = Serial2.read();
        }
    }

    Serial.printf("发送 %d 字节，回收 %d 字节\n", 16, recv);

    bool ok = (recv == 16);
    if (ok) {
        for (int i = 0; i < 16; i++) {
            if (rx_data[i] != tx_data[i]) { ok = false; break; }
        }
    }

    if (ok) {
        Serial.println("✅ LOOPBACK OK — ESP32 UART 硬件正常");
        Serial.println("   请拔掉短接线，按正确方向连接雷达，并将 LOOPBACK_TEST 改为 false 重新烧录。");
    } else {
        Serial.println("❌ LOOPBACK FAIL — ESP32 UART 可能有问题");
        Serial.print("   发送: ");
        for (int i = 0; i < 16; i++) Serial.printf("%02X ", tx_data[i]);
        Serial.println();
        Serial.print("   回收: ");
        for (int i = 0; i < recv; i++) Serial.printf("%02X ", rx_data[i]);
        if (recv == 0) Serial.print("（无数据）");
        Serial.println();
        Serial.println("   可能原因：短接线接触不良，或 GPIO16/17 在此板被占用。");
        Serial.println("   请尝试换用 GPIO4/5 并修改 RX_PIN/TX_PIN。");
    }

    while (true) { delay(5000); }  // 自环测试完成后停止
}

// ==================== 多波特率扫描 ====================
static void scan_baud_rates() {
    Serial.println("\n========== 波特率扫描（每个测试3秒）==========");
    Serial.println("如果某个波特率下每秒字节数稳定 > 0，说明该波特率正确。\n");

    for (int b = 0; b < NUM_BAUD_CANDIDATES; b++) {
        uint32_t baud = BAUD_CANDIDATES[b];
        Serial2.end();
        delay(100);
        Serial2.begin(baud, SERIAL_8N1, RX_PIN, TX_PIN);
        delay(200);

        // 清空可能的残留数据
        while (Serial2.available()) Serial2.read();

        uint32_t count = 0;
        uint32_t start = millis();
        while (millis() - start < 3000) {
            while (Serial2.available()) {
                Serial2.read();
                count++;
            }
        }

        float rate = count / 3.0f;
        Serial.printf("  %7u bps → 3秒内收到 %5u 字节 (%.0f 字节/秒)",
                      baud, count, rate);

        if (count > 30) {
            Serial.println("  ← ✅ 有数据流，波特率可能正确");
        } else if (count > 0) {
            Serial.println("  ← ⚠ 少量字节，可能是干扰或错误波特率");
        } else {
            Serial.println("  ← ❌ 无数据");
        }
    }

    Serial.println("\n扫描完成。现在用目标波特率开始实时监听...\n");
}

// ==================== 实时监听 ====================
static void print_stats() {
    uint32_t now = millis();
    if (now - last_stat_ms >= 1000) {
        last_stat_ms = now;

        if (bytes_this_second == 0) {
            Serial.println("[无数据] 检查接线方向和供电");
        } else {
            Serial.printf("[统计] 本秒 %4u 字节 / 累计 %u 字节  (预期约 %u B/s @1382400bps)\n",
                bytes_this_second,
                total_bytes,
                // 1382400bps，8N1，10bit/byte → 约138240 B/s 理论最大
                // 实际雷达帧率约20Hz，每帧约20字节 → 约400 B/s
                400u);
        }
        bytes_this_second = 0;
    }
}

// ==================== setup ====================
void setup() {
    Serial.begin(115200);
    delay(500);
    Serial.println("\n========================================");
    Serial.println("  HLK-LD6002 UART 通信诊断程序");
    Serial.println("========================================");
    Serial.printf("RX_PIN=%d  TX_PIN=%d\n", RX_PIN, TX_PIN);
    Serial.println();

    Serial.println("【接线提示】正确方向：");
    Serial.printf("  ESP32 GPIO%d (RX) ←── KIT板 TX0\n", RX_PIN);
    Serial.printf("  ESP32 GPIO%d (TX) ──→ KIT板 RX0  (可选)\n", TX_PIN);
    Serial.printf("  ESP32 GND        ───  KIT板 GND\n");
    Serial.println();
    Serial.println("【BOOT1提示】雷达规格书要求 Pin3(P19/BOOT1) 接GND才能从Flash启动。");
    Serial.println("  KIT测试板通常板上已处理，若仍无数据可检查此引脚。");
    Serial.println();

    if (LOOPBACK_TEST) {
        run_loopback_test();   // 不返回
        return;
    }

    // 波特率扫描
    scan_baud_rates();

    // 切换到目标波特率开始持续监听
    Serial2.end();
    delay(100);
    Serial2.begin(RADAR_BAUD, SERIAL_8N1, RX_PIN, TX_PIN);
    Serial2.setRxBufferSize(2048);
    delay(200);
    while (Serial2.available()) Serial2.read();  // 清空

    last_stat_ms = millis();
    Serial.printf("开始以 %u bps 监听原始字节（HEX格式）...\n", RADAR_BAUD);
    Serial.println("每行最多16字节，每秒打印统计。\n");
}

// ==================== loop ====================
void loop() {
    static uint8_t  line_buf[16];
    static uint8_t  line_pos = 0;
    static uint32_t last_byte_ms = 0;

    // 读取所有可用字节
    while (Serial2.available()) {
        uint8_t b = Serial2.read();
        bytes_this_second++;
        total_bytes++;
        last_byte_ms = millis();

        line_buf[line_pos++] = b;
        if (line_pos >= 16) {
            // 打印一行HEX + ASCII
            Serial.print("HEX: ");
            for (int i = 0; i < 16; i++) Serial.printf("%02X ", line_buf[i]);
            Serial.print(" | ");
            for (int i = 0; i < 16; i++) {
                char c = (char)line_buf[i];
                Serial.print((c >= 0x20 && c < 0x7F) ? c : '.');
            }
            Serial.println();
            line_pos = 0;
        }
    }

    // 超过200ms没新字节，把当前不完整行冲印出来
    if (line_pos > 0 && millis() - last_byte_ms > 200) {
        Serial.print("HEX: ");
        for (int i = 0; i < line_pos; i++) Serial.printf("%02X ", line_buf[i]);
        Serial.println("  (不完整行)");
        line_pos = 0;
    }

    print_stats();
}
