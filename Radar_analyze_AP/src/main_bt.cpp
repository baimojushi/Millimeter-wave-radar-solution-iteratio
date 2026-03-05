cat > /home/claude/radar_project/src/main_bt.cpp << 'CPPEOF'
/**
 * ESP32 毫米波雷达（经典蓝牙 SPP）
 * 输出：原始总相位 + IIR滤波相位，10Hz
 * 格式：{"raw":1.23,"p":0.45}\n
 */

#include <Arduino.h>
#include <BluetoothSerial.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

#if !defined(CONFIG_BT_ENABLED) || !defined(CONFIG_BLUEDROID_ENABLED)
#error "Bluetooth is not enabled!"
#endif

// ==================== 配置 ====================
#define BT_DEVICE_NAME  "ESP32-Radar-BT"
#define RADAR_RX_PIN    16
#define RADAR_TX_PIN    4
#define RADAR_BAUD      1382400
#define SEND_INTERVAL_MS  100   // 10Hz

#define FILTER_ALPHA    0.15f   // IIR截止约0.5Hz，可调

// ==================== TinyFrame ====================
#define TF_SOF         0x01
#define TF_TYPE_HUMAN  0x0F09
#define TF_TYPE_PHASE  0x0A13

typedef enum {
    ST_SOF, ST_ID_H, ST_ID_L,
    ST_LEN_H, ST_LEN_L,
    ST_TYPE_H, ST_TYPE_L,
    ST_HEAD_CKSUM, ST_DATA, ST_DATA_CKSUM
} TF_State;

// ==================== 全局状态 ====================
static BluetoothSerial BT;
static volatile bool   spp_ready = false;

// TF解析状态
static TF_State  tf_state    = ST_SOF;
static uint16_t  tf_id       = 0;
static uint16_t  tf_len      = 0;
static uint16_t  tf_type     = 0;
static uint8_t   tf_data[64] = {};
static uint16_t  tf_data_idx = 0;
static uint8_t   xor_acc     = 0;

// 数据输出（uart_task写，process_task读，float对齐赋值在Xtensa上是原子的）
static volatile float raw_phase      = 0.0f;
static volatile float filtered_phase = 0.0f;
static volatile bool  human_detected = false;
static volatile bool  new_data       = false;  // 有新相位帧标志

// IIR滤波器状态
static float   filt_state = 0.0f;
static bool    filt_init  = false;

// ==================== SPP回调（只写标志位）====================
static void spp_callback(esp_spp_cb_event_t event, esp_spp_cb_param_t*) {
    if      (event == ESP_SPP_SRV_OPEN_EVT) spp_ready = true;
    else if (event == ESP_SPP_CLOSE_EVT)    spp_ready = false;
}

// ==================== 工具 ====================
static float bytes_to_float(const uint8_t* b) {
    uint32_t v = (uint32_t)b[0] | ((uint32_t)b[1]<<8)
               | ((uint32_t)b[2]<<16) | ((uint32_t)b[3]<<24);
    float f; memcpy(&f, &v, 4); return f;
}

static void tf_reset() { tf_state = ST_SOF; tf_data_idx = 0; xor_acc = 0; }

// ==================== TF帧处理 ====================
static void on_frame() {
    switch (tf_type) {
        case TF_TYPE_HUMAN:
            if (tf_len >= 2) {
                uint16_t val = tf_data[0] | ((uint16_t)tf_data[1] << 8);
                human_detected = (val == 0x0001);
            }
            break;

        case TF_TYPE_PHASE:
            if (tf_len >= 4) {
                float raw = bytes_to_float(&tf_data[0]);
                raw_phase = raw;

                // IIR低通滤波
                if (!filt_init) { filt_state = raw; filt_init = true; }
                else filt_state = FILTER_ALPHA * raw + (1.0f - FILTER_ALPHA) * filt_state;
                filtered_phase = filt_state;

                new_data = true;
            }
            break;

        default: break;
    }
}

// ==================== TF字节解析 ====================
static void parse_byte(uint8_t b) {
    switch (tf_state) {
        case ST_SOF:
            if (b == TF_SOF) { xor_acc = b; tf_state = ST_ID_H; } break;
        case ST_ID_H:
            xor_acc ^= b; tf_id  = (uint16_t)b<<8; tf_state = ST_ID_L; break;
        case ST_ID_L:
            xor_acc ^= b; tf_id  |= b; tf_state = ST_LEN_H; break;
        case ST_LEN_H:
            xor_acc ^= b; tf_len  = (uint16_t)b<<8; tf_state = ST_LEN_L; break;
        case ST_LEN_L:
            xor_acc ^= b; tf_len |= b;
            if (tf_len > sizeof(tf_data)) { tf_reset(); break; }
            tf_state = ST_TYPE_H; break;
        case ST_TYPE_H:
            xor_acc ^= b; tf_type  = (uint16_t)b<<8; tf_state = ST_TYPE_L; break;
        case ST_TYPE_L:
            xor_acc ^= b; tf_type |= b; tf_state = ST_HEAD_CKSUM; break;
        case ST_HEAD_CKSUM:
            if (b != (uint8_t)(~xor_acc)) { tf_reset(); break; }
            tf_data_idx = 0; xor_acc = 0;
            tf_state = (tf_len > 0) ? ST_DATA : ST_DATA_CKSUM; break;
        case ST_DATA:
            tf_data[tf_data_idx++] = b; xor_acc ^= b;
            if (tf_data_idx >= tf_len) tf_state = ST_DATA_CKSUM; break;
        case ST_DATA_CKSUM:
            if (tf_len == 0 || b == (uint8_t)(~xor_acc)) on_frame();
            tf_reset(); break;
    }
}

// ==================== UART任务（Core 0，高优先级）====================
static void uart_task(void*) {
    uint8_t buf[256];
    while (true) {
        int n = Serial2.available();
        if (n > 0) {
            n = Serial2.readBytes(buf, min(n, (int)sizeof(buf)));
            for (int i = 0; i < n; i++) parse_byte(buf[i]);
        } else {
            vTaskDelay(1);
        }
    }
}

// ==================== BT发送任务（Core 1，低优先级）====================
static void send_task(void*) {
    char    msg[64];
    bool    was_ready    = false;
    uint32_t last_send   = 0;

    while (true) {
        bool ready = spp_ready;

        // SPP连接变化 —— 不在此处降频，避免BT栈TX速率跌落
        if (ready && !was_ready) {
            // 握手包：单次发送，不影响后续节奏
            BT.println("{\"info\":\"ESP32 Radar Ready\"}");
            Serial.println("[BT] SPP open");
        } else if (!ready && was_ready) {
            Serial.println("[BT] SPP closed");
            // 滤波器重置，下次连接时从新数据起步
            filt_init = false;
        }
        was_ready = ready;

        uint32_t now = millis();
        if (now - last_send >= SEND_INTERVAL_MS) {
            last_send = now;

            float raw  = raw_phase;
            float filt = filtered_phase;
            new_data   = false;

            // 拼JSON
            snprintf(msg, sizeof(msg),
                "{\"raw\":%.3f,\"p\":%.3f}\n", raw, filt);

            if (ready) {
                BT.print(msg);
            }

            // USB串口始终打印（调试用）
            Serial.print(msg);
        }

        vTaskDelay(5);  // 5ms轮询，不阻塞
    }
}

// ==================== setup / loop ====================
void setup() {
    Serial.begin(115200);
    delay(200);
    Serial.println("=== ESP32 Radar BT SPP ===");

    Serial2.begin(RADAR_BAUD, SERIAL_8N1, RADAR_RX_PIN, RADAR_TX_PIN);
    Serial2.setRxBufferSize(4096);

    BT.register_callback(spp_callback);
    if (!BT.begin(BT_DEVICE_NAME)) {
        Serial.println("BT FAILED, reboot");
        delay(2000); ESP.restart();
    }
    Serial.printf("BT: %s\nRX=GPIO%d  TX=GPIO%d\n",
                  BT_DEVICE_NAME, RADAR_RX_PIN, RADAR_TX_PIN);

    xTaskCreatePinnedToCore(uart_task, "UART", 4096, NULL, 2, NULL, 0);
    xTaskCreatePinnedToCore(send_task, "Send", 4096, NULL, 1, NULL, 1);

    Serial.println("Ready.");
}

void loop() { vTaskDelay(5000); }
CPPEOF
