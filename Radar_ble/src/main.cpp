/**
 * ESP32-C3 毫米波雷达数据解析程序 (BLE版本)
 * 
 * 功能：
 * 1. 接收HLK-LD6002雷达TinyFrame协议数据
 * 2. 解析有无人检测(0x0F09)和相位数据(0x0A13)
 * 3. 低通滤波处理总相位数据
 * 4. 基于20秒窗口计算呼吸速率，每8秒更新
 * 5. 通过BLE发送滤波后相位数据和呼吸速率
 * 
 * BLE服务：Nordic UART Service (NUS)
 * 数据格式：JSON，约5Hz发送频率（低功耗优化）
 * 
 * Windows连接方式：
 * 1. 打开Windows蓝牙设置，搜索并配对"ESP32-Radar"
 * 2. 使用Microsoft Store应用"Bluetooth LE Lab"查看数据
 *    或使用任何支持NUS的BLE串口工具
 */

#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/semphr.h>

// ==================== BLE配置 ====================
#define DEVICE_NAME "ESP32-Radar"

// Nordic UART Service UUID
#define SERVICE_UUID           "6E400001-B5A3-F393-E0A9-E50E24DCCA9E"
#define CHARACTERISTIC_UUID_RX "6E400002-B5A3-F393-E0A9-E50E24DCCA9E"
#define CHARACTERISTIC_UUID_TX "6E400003-B5A3-F393-E0A9-E50E24DCCA9E"

// ==================== 串口配置 ====================
#define RADAR_SERIAL Serial1
#define RADAR_BAUD 1382400
#define RADAR_RX_PIN 4
#define RADAR_TX_PIN 5

// ==================== 数据处理参数 ====================
#define PHASE_SAMPLE_RATE 20
#define WINDOW_SECONDS 20
#define UPDATE_SECONDS 8
#define MAX_PHASE_SAMPLES (PHASE_SAMPLE_RATE * WINDOW_SECONDS)

// BLE发送频率：5Hz（每200ms发送一次，低功耗友好）
#define BLE_SEND_INTERVAL_MS 200

// 滤波器参数
#define FILTER_ALPHA 0.15f

// ==================== TinyFrame协议 ====================
#define TF_SOF 0x01
#define TF_TYPE_HUMAN_DETECT 0x0F09
#define TF_TYPE_PHASE_RESULT 0x0A13

typedef enum {
    TF_STATE_SOF, TF_STATE_ID_H, TF_STATE_ID_L,
    TF_STATE_LEN_H, TF_STATE_LEN_L,
    TF_STATE_TYPE_H, TF_STATE_TYPE_L,
    TF_STATE_HEAD_CKSUM, TF_STATE_DATA, TF_STATE_DATA_CKSUM
} TF_State;

// ==================== 全局变量 ====================
// BLE
BLEServer* pServer = NULL;
BLECharacteristic* pTxCharacteristic = NULL;
volatile bool deviceConnected = false;
volatile bool oldDeviceConnected = false;

// TF解析状态
TF_State tf_state = TF_STATE_SOF;
uint16_t tf_id = 0, tf_len = 0, tf_type = 0;
uint8_t tf_data[32];
uint16_t tf_data_idx = 0;
uint8_t xor_acc = 0;

// 相位数据缓冲区
int16_t phase_buffer[MAX_PHASE_SAMPLES];
uint16_t phase_write_idx = 0;
uint16_t phase_count = 0;
SemaphoreHandle_t phase_mutex;

// 滤波状态
float filtered_phase = 0.0f;
float latest_filtered_phase = 0.0f;
bool filter_initialized = false;

// 当前状态
volatile bool human_detected = false;
volatile float current_breath_rate = 0.0f;
volatile uint32_t last_breath_update = 0;

// ==================== BLE回调 ====================
class ServerCallbacks : public BLEServerCallbacks {
    void onConnect(BLEServer* pServer) {
        deviceConnected = true;
        Serial.println("BLE Client Connected");
    }

    void onDisconnect(BLEServer* pServer) {
        deviceConnected = false;
        Serial.println("BLE Client Disconnected");
    }
};

class RxCallbacks : public BLECharacteristicCallbacks {
    void onWrite(BLECharacteristic* pCharacteristic) {
        std::string rxValue = pCharacteristic->getValue();
        if (rxValue.length() > 0) {
            Serial.print("BLE RX: ");
            Serial.println(rxValue.c_str());
            // 可以在这里处理来自客户端的命令
        }
    }
};

// ==================== 工具函数 ====================
float bytes_to_float(uint8_t* bytes) {
    uint32_t val = bytes[0] | (bytes[1] << 8) | (bytes[2] << 16) | (bytes[3] << 24);
    return *((float*)&val);
}

float low_pass_filter(float input) {
    if (!filter_initialized) {
        filtered_phase = input;
        filter_initialized = true;
        return input;
    }
    filtered_phase = FILTER_ALPHA * input + (1.0f - FILTER_ALPHA) * filtered_phase;
    return filtered_phase;
}

void tf_reset() {
    tf_state = TF_STATE_SOF;
    tf_data_idx = 0;
    xor_acc = 0;
}

// ==================== 存储相位数据 ====================
void store_phase_data(float phase) {
    float filtered = low_pass_filter(phase);
    latest_filtered_phase = filtered;
    
    int16_t fixed_point = (int16_t)(filtered * 100.0f);
    
    xSemaphoreTake(phase_mutex, portMAX_DELAY);
    phase_buffer[phase_write_idx] = fixed_point;
    phase_write_idx = (phase_write_idx + 1) % MAX_PHASE_SAMPLES;
    if (phase_count < MAX_PHASE_SAMPLES) phase_count++;
    xSemaphoreGive(phase_mutex);
}

// ==================== 处理TF帧 ====================
void process_frame() {
    if (tf_type == TF_TYPE_HUMAN_DETECT && tf_len >= 2) {
        uint16_t is_human = tf_data[0] | (tf_data[1] << 8);
        human_detected = (is_human == 0x0001);
    }
    else if (tf_type == TF_TYPE_PHASE_RESULT && tf_len >= 12) {
        float total_phase = bytes_to_float(&tf_data[0]);
        if (human_detected) {
            store_phase_data(total_phase);
        }
    }
}

// ==================== TF解析 ====================
void parse_byte(uint8_t byte) {
    switch (tf_state) {
        case TF_STATE_SOF:
            if (byte == TF_SOF) { xor_acc = byte; tf_state = TF_STATE_ID_H; }
            break;
        case TF_STATE_ID_H:
            xor_acc ^= byte; tf_id = byte << 8; tf_state = TF_STATE_ID_L;
            break;
        case TF_STATE_ID_L:
            xor_acc ^= byte; tf_id |= byte; tf_state = TF_STATE_LEN_H;
            break;
        case TF_STATE_LEN_H:
            xor_acc ^= byte; tf_len = byte << 8; tf_state = TF_STATE_LEN_L;
            break;
        case TF_STATE_LEN_L:
            xor_acc ^= byte; tf_len |= byte;
            if (tf_len > sizeof(tf_data)) { tf_reset(); break; }
            tf_state = TF_STATE_TYPE_H;
            break;
        case TF_STATE_TYPE_H:
            xor_acc ^= byte; tf_type = byte << 8; tf_state = TF_STATE_TYPE_L;
            break;
        case TF_STATE_TYPE_L:
            xor_acc ^= byte; tf_type |= byte; tf_state = TF_STATE_HEAD_CKSUM;
            break;
        case TF_STATE_HEAD_CKSUM:
            if (byte != (uint8_t)(~xor_acc)) { tf_reset(); break; }
            tf_data_idx = 0; xor_acc = 0;
            tf_state = (tf_len > 0) ? TF_STATE_DATA : TF_STATE_DATA_CKSUM;
            break;
        case TF_STATE_DATA:
            tf_data[tf_data_idx++] = byte; xor_acc ^= byte;
            if (tf_data_idx >= tf_len) tf_state = TF_STATE_DATA_CKSUM;
            break;
        case TF_STATE_DATA_CKSUM:
            if (tf_len == 0 || byte == (uint8_t)(~xor_acc)) process_frame();
            tf_reset();
            break;
    }
}

// ==================== 呼吸速率计算 ====================
float calculate_breath_rate() {
    if (phase_count < PHASE_SAMPLE_RATE * 5) return 0.0f;
    
    xSemaphoreTake(phase_mutex, portMAX_DELAY);
    
    uint16_t samples = min(phase_count, (uint16_t)MAX_PHASE_SAMPLES);
    uint16_t read_idx = (phase_write_idx + MAX_PHASE_SAMPLES - samples) % MAX_PHASE_SAMPLES;
    
    // 计算均值
    int32_t sum = 0;
    for (uint16_t i = 0; i < samples; i++) {
        sum += phase_buffer[(read_idx + i) % MAX_PHASE_SAMPLES];
    }
    int16_t mean = sum / samples;
    
    // 计算标准差
    int32_t var_sum = 0;
    for (uint16_t i = 0; i < samples; i++) {
        int16_t diff = phase_buffer[(read_idx + i) % MAX_PHASE_SAMPLES] - mean;
        var_sum += diff * diff;
    }
    float std_dev = sqrtf((float)var_sum / samples);
    
    int16_t threshold = (int16_t)(3.0f * std_dev);
    if (threshold < 50) threshold = 50;
    
    // 过零检测
    uint16_t zero_crossings = 0;
    bool was_above = (phase_buffer[read_idx] > mean);
    uint16_t valid_samples = 0;
    uint16_t consecutive_invalid = 0;
    
    for (uint16_t i = 1; i < samples; i++) {
        uint16_t idx = (read_idx + i) % MAX_PHASE_SAMPLES;
        int16_t val = phase_buffer[idx];
        int16_t diff = val - mean;
        
        if (abs(diff) > threshold) {
            consecutive_invalid++;
            if (consecutive_invalid > PHASE_SAMPLE_RATE / 2) {
                xSemaphoreGive(phase_mutex);
                return 0.0f;
            }
            continue;
        }
        consecutive_invalid = 0;
        valid_samples++;
        
        bool is_above = (val > mean);
        if (is_above != was_above) {
            zero_crossings++;
            was_above = is_above;
        }
    }
    
    xSemaphoreGive(phase_mutex);
    
    if (valid_samples < samples * 0.7f) return 0.0f;
    
    float cycles = zero_crossings / 2.0f;
    float duration = (float)valid_samples / PHASE_SAMPLE_RATE;
    float rate = (cycles / duration) * 60.0f;
    
    if (rate < 6.0f || rate > 30.0f) return 0.0f;
    
    return rate;
}

// ==================== BLE发送任务 ====================
void ble_send_task(void* param) {
    char msg[80];
    uint32_t last_send = 0;
    uint32_t last_calc = 0;
    
    while (true) {
        uint32_t now = millis();
        
        // 每8秒计算呼吸速率
        if (now - last_calc >= UPDATE_SECONDS * 1000) {
            last_calc = now;
            if (human_detected) {
                float rate = calculate_breath_rate();
                if (rate > 0) {
                    current_breath_rate = rate;
                    last_breath_update = now;
                }
            } else {
                // 无人时重置
                filter_initialized = false;
                xSemaphoreTake(phase_mutex, portMAX_DELAY);
                phase_count = 0;
                phase_write_idx = 0;
                xSemaphoreGive(phase_mutex);
                current_breath_rate = 0.0f;
            }
        }
        
        // 每200ms发送BLE数据（5Hz）
        if (now - last_send >= BLE_SEND_INTERVAL_MS) {
            last_send = now;
            
            // 构建JSON消息
            // h=有无人, p=滤波后相位, r=呼吸速率
            snprintf(msg, sizeof(msg), 
                "{\"h\":%d,\"p\":%.2f,\"r\":%.1f}\n",
                human_detected ? 1 : 0,
                latest_filtered_phase,
                current_breath_rate);
            
            // 发送BLE
            if (deviceConnected) {
                pTxCharacteristic->setValue((uint8_t*)msg, strlen(msg));
                pTxCharacteristic->notify();
            }
            
            // USB串口同时输出（调试用）
            Serial.print(msg);
        }
        
        // 处理BLE重连
        if (!deviceConnected && oldDeviceConnected) {
            delay(500);
            pServer->startAdvertising();
            Serial.println("BLE Advertising...");
            oldDeviceConnected = deviceConnected;
        }
        if (deviceConnected && !oldDeviceConnected) {
            oldDeviceConnected = deviceConnected;
        }
        
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}

// ==================== UART接收任务 ====================
void uart_task(void* param) {
    uint8_t buf[128];
    
    while (true) {
        int avail = RADAR_SERIAL.available();
        if (avail > 0) {
            int len = min(avail, (int)sizeof(buf));
            RADAR_SERIAL.readBytes(buf, len);
            for (int i = 0; i < len; i++) {
                parse_byte(buf[i]);
            }
        } else {
            vTaskDelay(pdMS_TO_TICKS(1));
        }
    }
}

// ==================== BLE初始化 ====================
void init_ble() {
    BLEDevice::init(DEVICE_NAME);
    
    // 设置BLE功率（低功耗）
    esp_ble_tx_power_set(ESP_BLE_PWR_TYPE_DEFAULT, ESP_PWR_LVL_P3);
    
    pServer = BLEDevice::createServer();
    pServer->setCallbacks(new ServerCallbacks());
    
    BLEService* pService = pServer->createService(SERVICE_UUID);
    
    // TX特征（ESP32发送数据给客户端）
    pTxCharacteristic = pService->createCharacteristic(
        CHARACTERISTIC_UUID_TX,
        BLECharacteristic::PROPERTY_NOTIFY
    );
    pTxCharacteristic->addDescriptor(new BLE2902());
    
    // RX特征（接收客户端命令）
    BLECharacteristic* pRxCharacteristic = pService->createCharacteristic(
        CHARACTERISTIC_UUID_RX,
        BLECharacteristic::PROPERTY_WRITE
    );
    pRxCharacteristic->setCallbacks(new RxCallbacks());
    
    pService->start();
    
    BLEAdvertising* pAdvertising = BLEDevice::getAdvertising();
    pAdvertising->addServiceUUID(SERVICE_UUID);
    pAdvertising->setScanResponse(true);
    pAdvertising->setMinPreferred(0x06);  // 连接间隔
    pAdvertising->setMinPreferred(0x12);
    BLEDevice::startAdvertising();
    
    Serial.println("BLE Ready, advertising as: " DEVICE_NAME);
}

// ==================== 主程序 ====================
void setup() {
    Serial.begin(115200);
    Serial.println("\n=== ESP32-C3 Radar Parser (BLE) ===");
    
    // 雷达串口
    RADAR_SERIAL.begin(RADAR_BAUD, SERIAL_8N1, RADAR_RX_PIN, RADAR_TX_PIN);
    RADAR_SERIAL.setRxBufferSize(1024);
    
    // 互斥锁
    phase_mutex = xSemaphoreCreateMutex();
    
    // BLE初始化
    init_ble();
    
    // 创建任务
    xTaskCreatePinnedToCore(uart_task, "UART", 4096, NULL, 2, NULL, 0);
    xTaskCreatePinnedToCore(ble_send_task, "BLE_Send", 4096, NULL, 1, NULL, 0);
    
    Serial.println("System Ready!");
    Serial.println("Open 'Bluetooth LE Lab' on Windows to view data");
}

void loop() {
    vTaskDelay(pdMS_TO_TICKS(1000));
}
