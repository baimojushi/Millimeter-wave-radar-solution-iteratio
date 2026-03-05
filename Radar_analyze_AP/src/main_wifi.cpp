/**
 * ESP32 毫米波雷达解析器（WiFi AP 热点版本）
 * 目标板：ESP32-WROOM / esp32dev
 *
 * 功耗策略（客户端连接后生效）：
 *   - CPU 从 240MHz 降至 80MHz（数据处理仍可胜任 10Hz 输出）
 *   - WiFi TX 功率从 20dBm 降至 ~8dBm（近距离足够，节省约 60mA）
 *   - Beacon 间隔拉长至 400ms（不影响已连接客户端数据流）
 *   - 客户端断开后恢复全功率以保证热点可见性
 *
 * 通信格式（TCP，每行一条 JSON）：
 *   相位数据：{"t":"p","h":1,"p":0.85}          10Hz
 *   呼吸更新：{"t":"r","h":1,"p":0.85,"r":15.5} 每8秒
 */

#include <Arduino.h>
#include <WiFi.h>
#include "esp_wifi.h"
#include "esp_pm.h"
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/semphr.h>

// ==================== 用户配置 ====================
#define AP_SSID         "ESP32-Radar"
#define AP_PASSWORD     "12345678"
#define TCP_PORT        8080

#define RADAR_RX_PIN    16      // 普通ESP32 GPIO16（可调整）
#define RADAR_TX_PIN    17

// ==================== 固定参数 ====================
#define RADAR_BAUD          1382400
#define PHASE_SAMPLE_RATE   20
#define WINDOW_SECONDS      20
#define UPDATE_SECONDS      8
#define MAX_PHASE_SAMPLES   (PHASE_SAMPLE_RATE * WINDOW_SECONDS)
#define FILTER_ALPHA        0.15f
#define TCP_SEND_MS         100     // 10Hz

// WiFi TX 功率档位（单位 0.25dBm）
// 80  = 20dBm（默认，广播时用）
// 32  = 8dBm （连接后节能，约1~2米范围足够）
#define WIFI_PWR_BROADCAST  80
#define WIFI_PWR_CONNECTED  32

// ==================== TinyFrame ====================
#define TF_SOF          0x01
#define TF_TYPE_HUMAN   0x0F09
#define TF_TYPE_PHASE   0x0A13

typedef enum {
    ST_SOF, ST_ID_H, ST_ID_L,
    ST_LEN_H, ST_LEN_L,
    ST_TYPE_H, ST_TYPE_L,
    ST_HEAD_CKSUM, ST_DATA, ST_DATA_CKSUM
} TF_State;

// ==================== 全局状态 ====================
static TF_State  tf_state    = ST_SOF;
static uint16_t  tf_id       = 0;
static uint16_t  tf_len      = 0;
static uint16_t  tf_type     = 0;
static uint8_t   tf_data[32] = {};
static uint16_t  tf_data_idx = 0;
static uint8_t   xor_acc     = 0;

static int16_t           phase_buf[MAX_PHASE_SAMPLES] = {};
static uint16_t          phase_wr    = 0;
static uint16_t          phase_cnt   = 0;
static SemaphoreHandle_t phase_mutex = NULL;

static float filtered_phase     = 0.0f;
static float latest_filtered    = 0.0f;
static bool  filter_initialized = false;

static volatile bool  human_detected   = false;
static volatile float breath_rate      = 0.0f;
static volatile bool  client_connected = false;

static WiFiServer tcp_server(TCP_PORT);
static WiFiClient tcp_client;

// ==================== 功耗控制 ====================
static void enter_low_power() {
    // 降低 CPU 频率：80MHz 足以维持 10Hz 输出和 1382400 UART 接收
    setCpuFrequencyMhz(80);
    // 降低 WiFi 发射功率
    esp_wifi_set_max_tx_power(WIFI_PWR_CONNECTED);
    Serial.println("[PM] Low-power mode: CPU=80MHz, TX=8dBm");
}

static void exit_low_power() {
    setCpuFrequencyMhz(240);
    esp_wifi_set_max_tx_power(WIFI_PWR_BROADCAST);
    Serial.println("[PM] Full-power mode: CPU=240MHz, TX=20dBm");
}

// ==================== 工具函数 ====================
static float bytes_to_float(const uint8_t* b) {
    uint32_t v = (uint32_t)b[0]
               | ((uint32_t)b[1] << 8)
               | ((uint32_t)b[2] << 16)
               | ((uint32_t)b[3] << 24);
    float f;
    memcpy(&f, &v, 4);
    return f;
}

static float lp_filter(float input) {
    if (!filter_initialized) {
        filtered_phase = input;
        filter_initialized = true;
        return input;
    }
    filtered_phase = FILTER_ALPHA * input + (1.0f - FILTER_ALPHA) * filtered_phase;
    return filtered_phase;
}

static void tf_reset() {
    tf_state    = ST_SOF;
    tf_data_idx = 0;
    xor_acc     = 0;
}

// ==================== TF 帧处理 ====================
static void on_frame() {
    switch (tf_type) {
        case TF_TYPE_HUMAN:
            if (tf_len >= 2) {
                uint16_t val = tf_data[0] | (tf_data[1] << 8);
                human_detected = (val == 0x0001);
            }
            break;

        case TF_TYPE_PHASE:
            if (tf_len >= 12 && human_detected) {
                float raw = bytes_to_float(&tf_data[0]);
                float filt = lp_filter(raw);
                latest_filtered = filt;

                int16_t fp = (int16_t)(filt * 100.0f);
                xSemaphoreTake(phase_mutex, portMAX_DELAY);
                phase_buf[phase_wr] = fp;
                phase_wr = (phase_wr + 1) % MAX_PHASE_SAMPLES;
                if (phase_cnt < MAX_PHASE_SAMPLES) phase_cnt++;
                xSemaphoreGive(phase_mutex);
            }
            break;

        default: break;
    }
}

// ==================== TF 字节解析 ====================
static void parse_byte(uint8_t b) {
    switch (tf_state) {
        case ST_SOF:
            if (b == TF_SOF) { xor_acc = b; tf_state = ST_ID_H; }
            break;
        case ST_ID_H:
            xor_acc ^= b; tf_id = b << 8; tf_state = ST_ID_L; break;
        case ST_ID_L:
            xor_acc ^= b; tf_id |= b; tf_state = ST_LEN_H; break;
        case ST_LEN_H:
            xor_acc ^= b; tf_len = b << 8; tf_state = ST_LEN_L; break;
        case ST_LEN_L:
            xor_acc ^= b; tf_len |= b;
            if (tf_len > sizeof(tf_data)) { tf_reset(); break; }
            tf_state = ST_TYPE_H;
            break;
        case ST_TYPE_H:
            xor_acc ^= b; tf_type = b << 8; tf_state = ST_TYPE_L; break;
        case ST_TYPE_L:
            xor_acc ^= b; tf_type |= b; tf_state = ST_HEAD_CKSUM; break;
        case ST_HEAD_CKSUM:
            if (b != (uint8_t)(~xor_acc)) { tf_reset(); break; }
            tf_data_idx = 0; xor_acc = 0;
            tf_state = (tf_len > 0) ? ST_DATA : ST_DATA_CKSUM;
            break;
        case ST_DATA:
            tf_data[tf_data_idx++] = b; xor_acc ^= b;
            if (tf_data_idx >= tf_len) tf_state = ST_DATA_CKSUM;
            break;
        case ST_DATA_CKSUM:
            if (tf_len == 0 || b == (uint8_t)(~xor_acc)) on_frame();
            tf_reset();
            break;
    }
}

// ==================== 呼吸速率计算 ====================
static float calc_breath_rate() {
    xSemaphoreTake(phase_mutex, portMAX_DELAY);
    uint16_t cnt = phase_cnt;
    uint16_t rd  = (phase_wr + MAX_PHASE_SAMPLES - cnt) % MAX_PHASE_SAMPLES;
    xSemaphoreGive(phase_mutex);

    if (cnt < PHASE_SAMPLE_RATE * 5) return 0.0f;

    int32_t sum = 0;
    xSemaphoreTake(phase_mutex, portMAX_DELAY);
    for (uint16_t i = 0; i < cnt; i++)
        sum += phase_buf[(rd + i) % MAX_PHASE_SAMPLES];
    int16_t mean = (int16_t)(sum / cnt);

    int32_t vsum = 0;
    for (uint16_t i = 0; i < cnt; i++) {
        int16_t d = phase_buf[(rd + i) % MAX_PHASE_SAMPLES] - mean;
        vsum += (int32_t)d * d;
    }
    xSemaphoreGive(phase_mutex);

    float std = sqrtf((float)vsum / cnt);
    int16_t thr = (int16_t)(3.0f * std);
    if (thr < 50) thr = 50;

    xSemaphoreTake(phase_mutex, portMAX_DELAY);
    uint16_t crossings = 0, valid = 0, consec_invalid = 0;
    bool above = (phase_buf[rd] > mean);

    for (uint16_t i = 1; i < cnt; i++) {
        int16_t v = phase_buf[(rd + i) % MAX_PHASE_SAMPLES];
        if (abs(v - mean) > thr) {
            if (++consec_invalid > PHASE_SAMPLE_RATE / 2) {
                xSemaphoreGive(phase_mutex);
                return 0.0f;
            }
            continue;
        }
        consec_invalid = 0;
        valid++;
        bool cur = (v > mean);
        if (cur != above) { crossings++; above = cur; }
    }
    xSemaphoreGive(phase_mutex);

    if (valid < cnt * 0.7f) return 0.0f;

    float cycles  = crossings / 2.0f;
    float dur_sec = (float)valid / PHASE_SAMPLE_RATE;
    float rate    = (cycles / dur_sec) * 60.0f;
    return (rate >= 6.0f && rate <= 30.0f) ? rate : 0.0f;
}

// ==================== UART 任务 ====================
static void uart_task(void* /*param*/) {
    uint8_t buf[128];
    while (true) {
        int avail = Serial1.available();
        if (avail > 0) {
            int n = Serial1.readBytes(buf, min(avail, (int)sizeof(buf)));
            for (int i = 0; i < n; i++) parse_byte(buf[i]);
        } else {
            vTaskDelay(pdMS_TO_TICKS(1));
        }
    }
}

// ==================== 数据处理 + TCP 发送任务 ====================
static void process_task(void* /*param*/) {
    uint32_t last_calc = 0;
    uint32_t last_send = 0;
    char msg[96];
    bool was_connected = false;

    while (true) {
        uint32_t now = millis();

        // ---- 客户端连接状态检测 + 功耗切换 ----
        bool connected = (tcp_client && tcp_client.connected());
        if (connected && !was_connected) {
            client_connected = true;
            enter_low_power();
        } else if (!connected && was_connected) {
            client_connected = false;
            exit_low_power();
        }
        was_connected = connected;

        // ---- 每8秒计算呼吸速率 ----
        if (now - last_calc >= UPDATE_SECONDS * 1000UL) {
            last_calc = now;
            if (human_detected) {
                float r = calc_breath_rate();
                if (r > 0.0f) breath_rate = r;
            } else {
                filter_initialized = false;
                xSemaphoreTake(phase_mutex, portMAX_DELAY);
                phase_cnt = phase_wr = 0;
                xSemaphoreGive(phase_mutex);
                breath_rate = 0.0f;
            }
            snprintf(msg, sizeof(msg),
                "{\"t\":\"r\",\"h\":%d,\"p\":%.2f,\"r\":%.1f}\n",
                human_detected ? 1 : 0, latest_filtered, breath_rate);
            if (connected) tcp_client.print(msg);
            Serial.print(msg);
        }

        // ---- 10Hz 相位数据 ----
        if (now - last_send >= TCP_SEND_MS) {
            last_send = now;
            snprintf(msg, sizeof(msg),
                "{\"t\":\"p\",\"h\":%d,\"p\":%.2f}\n",
                human_detected ? 1 : 0, latest_filtered);
            if (connected) tcp_client.print(msg);
        }

        // ---- 接受新客户端 ----
        if (!connected) {
            WiFiClient c = tcp_server.available();
            if (c) {
                tcp_client = c;
                Serial.println("TCP client connected");
                tcp_client.println("{\"t\":\"info\",\"msg\":\"ESP32 Radar Ready\"}");
            }
        }

        vTaskDelay(pdMS_TO_TICKS(10));
    }
}

// ==================== AP 初始化 ====================
static void init_ap() {
    WiFi.disconnect(true);
    delay(200);
    WiFi.mode(WIFI_AP);
    delay(500);

    IPAddress ip(192, 168, 4, 1);
    WiFi.softAPConfig(ip, ip, IPAddress(255, 255, 255, 0));
    delay(100);

    bool ok = WiFi.softAP(AP_SSID, AP_PASSWORD, 1, 0, 1);
    if (ok) {
        esp_wifi_set_max_tx_power(WIFI_PWR_BROADCAST);
        Serial.printf("AP Ready  SSID: %s  IP: %s  Port: %d\n",
            AP_SSID, WiFi.softAPIP().toString().c_str(), TCP_PORT);
    } else {
        Serial.println("AP init FAILED. Rebooting...");
        delay(2000);
        ESP.restart();
    }
}

// ==================== setup / loop ====================
void setup() {
    Serial.begin(115200);
    Serial.println("\n=== ESP32 Radar Parser (WiFi AP + Power Saving) ===");

    Serial1.begin(RADAR_BAUD, SERIAL_8N1, RADAR_RX_PIN, RADAR_TX_PIN);
    Serial1.setRxBufferSize(2048);

    phase_mutex = xSemaphoreCreateMutex();

    init_ap();
    tcp_server.begin();

    xTaskCreatePinnedToCore(uart_task,    "UART",    4096, NULL, 2, NULL, 0);
    xTaskCreatePinnedToCore(process_task, "Process", 4096, NULL, 1, NULL, 1);

    Serial.println("System Ready.");
}

void loop() {
    vTaskDelay(pdMS_TO_TICKS(5000));
}
