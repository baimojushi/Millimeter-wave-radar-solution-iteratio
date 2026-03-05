/**
 * ESP32-C3 毫米波雷达数据解析程序（WiFi TCP版本）
 *
 * 功能：
 *   解析HLK-LD6002雷达TinyFrame协议数据
 *   解析有无人检测 (0x0F09) 和相位数据 (0x0A13)
 *   对总相位进行低通滤波（截止约0.5Hz，去除心跳影响）
 *   20秒滑动窗口计算呼吸速率，每8秒更新一次
 *   剔除人体轻微运动导致的异常数据
 *   通过WiFi TCP发送：滤波后相位（10Hz）+ 呼吸速率（每8秒）
 *
 * 通信格式（TCP，每行一条JSON）：
 *   相位数据：{"t":"p","h":1,"p":0.85}
 *   呼吸更新：{"t":"r","h":1,"p":0.85,"r":15.5}
 *
 * UART：ESP32-C3 Core0高优先级任务接收雷达数据
 * 数据处理：Core0低优先级任务，每8秒计算呼吸速率
 * TCP发送：10Hz，嵌入数据处理任务
 */

#include <Arduino.h>
#include <WiFi.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/semphr.h>

// ==================== 用户配置 ====================
#define WIFI_SSID     "RADAR"
#define WIFI_PASSWORD "12345678"
#define TCP_PORT      8080

// 雷达串口引脚（根据实际接线调整）
#define RADAR_RX_PIN  4
#define RADAR_TX_PIN  5

// ==================== 固定参数 ====================
#define RADAR_BAUD          1382400
#define PHASE_SAMPLE_RATE   20              // 雷达相位输出频率（Hz），根据实测调整
#define WINDOW_SECONDS      20
#define UPDATE_SECONDS      8
#define MAX_PHASE_SAMPLES   (PHASE_SAMPLE_RATE * WINDOW_SECONDS)  // 400个样本
#define FILTER_ALPHA        0.15f           // 低通滤波系数，越小越平滑
#define TCP_SEND_HZ         10              // TCP发送频率（Hz）
#define TCP_SEND_MS         (1000 / TCP_SEND_HZ)

// ==================== TinyFrame 协议定义 ====================
#define TF_SOF              0x01
#define TF_TYPE_HUMAN       0x0F09
#define TF_TYPE_PHASE       0x0A13

typedef enum {
    ST_SOF, ST_ID_H, ST_ID_L,
    ST_LEN_H, ST_LEN_L,
    ST_TYPE_H, ST_TYPE_L,
    ST_HEAD_CKSUM, ST_DATA, ST_DATA_CKSUM
} TF_State;

// ==================== 全局状态 ====================
// TF解析
static TF_State  tf_state    = ST_SOF;
static uint16_t  tf_id       = 0;
static uint16_t  tf_len      = 0;
static uint16_t  tf_type     = 0;
static uint8_t   tf_data[32] = {};
static uint16_t  tf_data_idx = 0;
static uint8_t   xor_acc     = 0;

// 相位环形缓冲区（int16定点×100，节省内存）
static int16_t           phase_buf[MAX_PHASE_SAMPLES] = {};
static uint16_t          phase_wr    = 0;
static uint16_t          phase_cnt   = 0;
static SemaphoreHandle_t phase_mutex = NULL;

// 滤波
static float filtered_phase      = 0.0f;
static float latest_filtered     = 0.0f;
static bool  filter_initialized  = false;

// 输出结果
static volatile bool    human_detected    = false;
static volatile float   breath_rate       = 0.0f;

// WiFi / TCP
static WiFiServer  tcp_server(TCP_PORT);
static WiFiClient  tcp_client;

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
        filtered_phase     = input;
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

// ==================== TF帧处理 ====================
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
                float raw_total = bytes_to_float(&tf_data[0]);
                float filt = lp_filter(raw_total);
                latest_filtered = filt;

                int16_t fp = (int16_t)(filt * 100.0f);
                xSemaphoreTake(phase_mutex, portMAX_DELAY);
                phase_buf[phase_wr] = fp;
                phase_wr = (phase_wr + 1) % MAX_PHASE_SAMPLES;
                if (phase_cnt < MAX_PHASE_SAMPLES) phase_cnt++;
                xSemaphoreGive(phase_mutex);
            }
            break;

        default:
            break;
    }
}

// ==================== TF字节解析 ====================
static void parse_byte(uint8_t b) {
    switch (tf_state) {
        case ST_SOF:
            if (b == TF_SOF) { xor_acc = b; tf_state = ST_ID_H; }
            break;
        case ST_ID_H:
            xor_acc ^= b; tf_id = b << 8; tf_state = ST_ID_L;
            break;
        case ST_ID_L:
            xor_acc ^= b; tf_id |= b; tf_state = ST_LEN_H;
            break;
        case ST_LEN_H:
            xor_acc ^= b; tf_len = b << 8; tf_state = ST_LEN_L;
            break;
        case ST_LEN_L:
            xor_acc ^= b; tf_len |= b;
            if (tf_len > sizeof(tf_data)) { tf_reset(); break; }
            tf_state = ST_TYPE_H;
            break;
        case ST_TYPE_H:
            xor_acc ^= b; tf_type = b << 8; tf_state = ST_TYPE_L;
            break;
        case ST_TYPE_L:
            xor_acc ^= b; tf_type |= b; tf_state = ST_HEAD_CKSUM;
            break;
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

    if (cnt < PHASE_SAMPLE_RATE * 5) return 0.0f;  // 至少5秒数据

    // 均值
    int32_t sum = 0;
    xSemaphoreTake(phase_mutex, portMAX_DELAY);
    for (uint16_t i = 0; i < cnt; i++)
        sum += phase_buf[(rd + i) % MAX_PHASE_SAMPLES];
    int16_t mean = (int16_t)(sum / cnt);

    // 方差 → 标准差
    int32_t vsum = 0;
    for (uint16_t i = 0; i < cnt; i++) {
        int16_t d = phase_buf[(rd + i) % MAX_PHASE_SAMPLES] - mean;
        vsum += (int32_t)d * d;
    }
    xSemaphoreGive(phase_mutex);

    float std = sqrtf((float)vsum / cnt);
    int16_t thr = (int16_t)(3.0f * std);
    if (thr < 50) thr = 50;

    // 过零检测
    xSemaphoreTake(phase_mutex, portMAX_DELAY);
    uint16_t crossings      = 0;
    uint16_t valid          = 0;
    uint16_t consec_invalid = 0;
    bool above = (phase_buf[rd] > mean);

    for (uint16_t i = 1; i < cnt; i++) {
        int16_t v = phase_buf[(rd + i) % MAX_PHASE_SAMPLES];
        if (abs(v - mean) > thr) {
            if (++consec_invalid > PHASE_SAMPLE_RATE / 2) {
                xSemaphoreGive(phase_mutex);
                return 0.0f;  // 连续0.5秒异常：运动干扰
            }
            continue;
        }
        consec_invalid = 0;
        valid++;
        bool cur = (v > mean);
        if (cur != above) { crossings++; above = cur; }
    }
    xSemaphoreGive(phase_mutex);

    if (valid < cnt * 0.7f) return 0.0f;  // 有效率<70%

    float cycles  = crossings / 2.0f;
    float dur_sec = (float)valid / PHASE_SAMPLE_RATE;
    float rate    = (cycles / dur_sec) * 60.0f;

    return (rate >= 6.0f && rate <= 30.0f) ? rate : 0.0f;
}

// ==================== 发送数据到TCP客户端 ====================
static void tcp_send(const char* msg) {
    if (tcp_client && tcp_client.connected()) {
        tcp_client.print(msg);
    }
}

// ==================== UART接收任务（高优先级） ====================
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

// ==================== 数据处理 + TCP发送任务 ====================
static void process_task(void* /*param*/) {
    uint32_t last_calc = 0;
    uint32_t last_send = 0;
    char msg[96];

    while (true) {
        uint32_t now = millis();

        // 每8秒计算一次呼吸速率
        if (now - last_calc >= UPDATE_SECONDS * 1000UL) {
            last_calc = now;
            if (human_detected) {
                float r = calc_breath_rate();
                if (r > 0.0f) breath_rate = r;
            } else {
                // 无人时清空缓冲
                filter_initialized = false;
                xSemaphoreTake(phase_mutex, portMAX_DELAY);
                phase_cnt = 0;
                phase_wr  = 0;
                xSemaphoreGive(phase_mutex);
                breath_rate = 0.0f;
            }

            // 呼吸速率更新消息
            snprintf(msg, sizeof(msg),
                "{\"t\":\"r\",\"h\":%d,\"p\":%.2f,\"r\":%.1f}\n",
                human_detected ? 1 : 0,
                latest_filtered,
                breath_rate);
            tcp_send(msg);
            Serial.print(msg);
        }

        // 按TCP_SEND_HZ发送相位数据
        if (now - last_send >= TCP_SEND_MS) {
            last_send = now;
            snprintf(msg, sizeof(msg),
                "{\"t\":\"p\",\"h\":%d,\"p\":%.2f}\n",
                human_detected ? 1 : 0,
                latest_filtered);
            tcp_send(msg);
        }

        // 处理TCP客户端连接/断开
        if (!tcp_client || !tcp_client.connected()) {
            tcp_client = tcp_server.available();
            if (tcp_client) {
                Serial.println("TCP client connected");
                tcp_client.println("{\"t\":\"info\",\"msg\":\"ESP32-C3 Radar Ready\"}");
            }
        }

        vTaskDelay(pdMS_TO_TICKS(10));
    }
}

// ==================== WiFi连接 ====================
static void wifi_connect() {
    Serial.printf("Connecting to WiFi: %s\n", WIFI_SSID);
    WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
    uint32_t t = millis();
    while (WiFi.status() != WL_CONNECTED && millis() - t < 15000) {
        delay(500);
        Serial.print(".");
    }
    Serial.println();
    if (WiFi.status() == WL_CONNECTED) {
        Serial.printf("WiFi OK. IP: %s  TCP Port: %d\n",
            WiFi.localIP().toString().c_str(), TCP_PORT);
    } else {
        Serial.println("WiFi FAILED. Rebooting...");
        delay(2000);
        ESP.restart();
    }
}

// ==================== setup / loop ====================
void setup() {
    Serial.begin(115200);
    Serial.println("\n=== ESP32-C3 Radar Parser (WiFi TCP) ===");

    // 雷达串口（高波特率，增大缓冲区防溢出）
    Serial1.begin(RADAR_BAUD, SERIAL_8N1, RADAR_RX_PIN, RADAR_TX_PIN);
    Serial1.setRxBufferSize(2048);

    phase_mutex = xSemaphoreCreateMutex();

    wifi_connect();
    tcp_server.begin();

    // UART接收任务：高优先级，绑定Core 0
    xTaskCreatePinnedToCore(uart_task,    "UART",    4096, NULL, 2, NULL, 0);
    // 数据处理+TCP发送：较低优先级，同样Core 0
    xTaskCreatePinnedToCore(process_task, "Process", 4096, NULL, 1, NULL, 0);

    Serial.println("System Ready!");
}

void loop() {
    // WiFi断线重连
    if (WiFi.status() != WL_CONNECTED) {
        Serial.println("WiFi lost, reconnecting...");
        wifi_connect();
    }
    vTaskDelay(pdMS_TO_TICKS(5000));
}
