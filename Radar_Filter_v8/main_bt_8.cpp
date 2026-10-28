/**
 * ESP32 millimeter-wave radar respiratory-rate firmware v8.
 *
 * Target: original ESP32 with Classic Bluetooth SPP (NOT ESP32-S3/C3 BLE-only).
 * Wiring/protocol inherited from upstream Radar_analyze_test/main_bt_7.cpp.
 * UART2 RX GPIO16 / TX GPIO4 / 1382400 baud, annotation button GPIO17.
 *
 * Changes from v7: shared portable radar_resp_filter_v8.h filter, human/no-data
 * gating, phase wrap, finite checks, normalized ACF + quadrature periodicity
 * cross-check, safe frame parsing, button debouncing, coherent cross-core
 * snapshots. "hil_ph" retained but now denotes fitted respiratory phase.
 *
 * IMPORTANT: This is a research prototype, not a medical respiratory monitor.
 */
#include <Arduino.h>
#include <BluetoothSerial.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/queue.h>
#include <string.h>
#include "radar_resp_filter_v8.h"

#if !defined(CONFIG_BT_ENABLED) || !defined(CONFIG_BLUEDROID_ENABLED)
#error "This build requires ESP32 Classic Bluetooth / Bluedroid enabled."
#endif

static constexpr char BT_NAME[] = "ESP32-Radar-v8";
static constexpr int RADAR_RX_PIN = 16;
static constexpr int RADAR_TX_PIN = 4;
static constexpr uint32_t RADAR_BAUD = 1382400UL;
static constexpr int BUTTON_PIN = 17;
static constexpr uint32_t BUTTON_DEBOUNCE_MS = 80;
static constexpr uint32_t SEND_INTERVAL_MS = 100;

static constexpr uint8_t TF_START = 0x01;
static constexpr uint16_t TF_HUMAN = 0x0F09;
static constexpr uint16_t TF_PHASE = 0x0A13;
static constexpr size_t MAX_PAYLOAD = 64;

static BluetoothSerial BT;
static volatile bool spp_ready = false;
static radar_v8::Filter filter;
static portMUX_TYPE snap_mux = portMUX_INITIALIZER_UNLOCKED;

struct Published {
    radar_v8::Result result;
    uint32_t phase_frames;
    uint32_t crc_errors;
    uint32_t queue_drops;
    uint32_t max_filter_us;
};
static Published published = {};
static uint32_t phase_frame_count = 0;
static uint32_t crc_error_count = 0;
static uint32_t queue_drop_count = 0;
static uint32_t max_filter_us = 0;

struct KeyEvent {
    uint32_t timestamp;
    float bpm;
    float lag_error;
    uint8_t n_keys;
    bool lag_valid;
    bool gap;
};
static QueueHandle_t key_queue = nullptr;

// UART task owns the filter and all protocol/parser state.
// send_task only reads Published copies under snap_mux.
static void publish_snapshot() {
    Published local = {};
    local.result = filter.result();
    local.phase_frames = phase_frame_count;
    local.crc_errors = crc_error_count;
    local.queue_drops = queue_drop_count;
    local.max_filter_us = max_filter_us;
    portENTER_CRITICAL(&snap_mux);
    published = local;
    portEXIT_CRITICAL(&snap_mux);
}

static float little_endian_f32(const uint8_t* p) {
    uint32_t u = (uint32_t)p[0] |
        ((uint32_t)p[1] << 8) |
        ((uint32_t)p[2] << 16) |
        ((uint32_t)p[3] << 24);
    float v;
    memcpy(&v, &u, sizeof(v));
    return v;
}

static uint32_t last_accepted_key = 0;
static bool have_accepted_key = false;
static uint32_t key_stamps[8] = {};
static uint8_t key_count = 0, key_head = 0;
static constexpr uint32_t KEY_INTERVAL_MIN = 1800; // ~33 bpm, tolerance above 30
static constexpr uint32_t KEY_INTERVAL_MAX = 7500; //  8 bpm, tolerance below 9

static void key_pressed(uint32_t now) {
    bool gap = false;
    if (have_accepted_key) {
        uint32_t interval = (uint32_t)(now - last_accepted_key);
        if (interval < KEY_INTERVAL_MIN) return;
        if (interval > KEY_INTERVAL_MAX) {
            // Do not mix old and new annotations across a missed press.
            key_count = key_head = 0;
            gap = true;
        }
    }
    last_accepted_key = now;
    have_accepted_key = true;
    key_stamps[key_head] = now;
    key_head = (key_head + 1) % 8;
    if (key_count < 8) ++key_count;

    float intervals[7] = {};
    uint8_t n = 0;
    const uint8_t first = (key_head + 8 - key_count) % 8;
    for (int i = 1; i < key_count; ++i) {
        uint8_t a = (first + i - 1) % 8;
        uint8_t b = (first + i) % 8;
        uint32_t diff = (uint32_t)(key_stamps[b] - key_stamps[a]);
        if (diff >= KEY_INTERVAL_MIN && diff <= KEY_INTERVAL_MAX)
            intervals[n++] = (float)diff;
    }
    for (uint8_t i = 1; i < n; ++i) {
        float x = intervals[i];
        int j = i - 1;
        while (j >= 0 && intervals[j] > x) {
            intervals[j+1] = intervals[j];
            --j;
        }
        intervals[j+1] = x;
    }
    float gap_ms = 0.0f;
    if (n) gap_ms = (n % 2)
        ? intervals[n/2]
        : 0.5f * (intervals[n/2-1] + intervals[n/2]);

    KeyEvent e = {};
    e.timestamp = now;
    e.bpm = gap_ms > 0.0f ? 60000.0f / gap_ms : 0.0f;
    e.n_keys = key_count;
    e.gap = gap;
    const radar_v8::Result& r = filter.result();
    e.lag_valid = (r.status == radar_v8::OK && r.lag > 0.0f && gap_ms > 0.0f);
    if (e.lag_valid) {
        float reference_lag = gap_ms * radar_v8::FS / 1000.0f;
        e.lag_error = 100.0f * (r.lag - reference_lag) / reference_lag;
    }
    if (!key_queue || xQueueSend(key_queue, &e, 0) != pdTRUE)
        ++queue_drop_count;
}

static void poll_button() {
    static bool raw_previous = HIGH;
    static bool stable_level = HIGH;
    static uint32_t last_change = 0;
    const uint32_t now = millis();
    const bool raw = (digitalRead(BUTTON_PIN) != LOW);
    if (raw != raw_previous) {
        raw_previous = raw;
        last_change = now;
    }
    // Require the new level to remain stable for 80 ms.
    if ((uint32_t)(now - last_change) >= BUTTON_DEBOUNCE_MS &&
        stable_level != raw) {
        stable_level = raw;
        if (stable_level == LOW)
            key_pressed(now);
    }
}

enum ParserState : uint8_t {
    FIND_SOF, ID_HIGH, ID_LOW, LEN_HIGH, LEN_LOW,
    TYPE_HIGH, TYPE_LOW, HEADER_XOR, PAYLOAD, DATA_XOR
};
static ParserState state = FIND_SOF;
static uint8_t header_xor = 0, data_xor = 0;
static uint16_t frame_id = 0, payload_len = 0, packet_type = 0;
static uint8_t payload[MAX_PAYLOAD] = {};
static uint16_t payload_pos = 0;
static uint32_t last_parser_byte_ms = 0;

static void parser_reset() {
    state = FIND_SOF;
    header_xor = data_xor = 0;
    payload_pos = 0;
}

static void complete_frame() {
    if (packet_type == TF_HUMAN && payload_len >= 2) {
        const uint16_t flag =
            (uint16_t)payload[0] | ((uint16_t)payload[1] << 8);
        filter.on_human(flag == 0x0001);
    } else if (packet_type == TF_PHASE && payload_len >= 4) {
        float raw = little_endian_f32(payload);
        // No trust in unvalidated floats; the Filter rejects NaN/Inf.
        uint32_t start_us = micros();
        filter.add(raw, millis());
        uint32_t used_us = (uint32_t)(micros() - start_us);
        if (used_us > max_filter_us) max_filter_us = used_us;
        ++phase_frame_count;
    }
}

static void parse_byte(uint8_t b) {
    uint32_t now = millis();
    // Reset incomplete/corrupt frames after a real 200ms inter-byte gap.
    if (state != FIND_SOF &&
        (uint32_t)(now - last_parser_byte_ms) > 200U) parser_reset();
    last_parser_byte_ms = now;
    switch (state) {
      case FIND_SOF:
        if (b == TF_START) { header_xor = b; state = ID_HIGH; }
        break;
      case ID_HIGH: header_xor ^= b; frame_id = (uint16_t)b << 8;
                    state = ID_LOW; break;
      case ID_LOW:  header_xor ^= b; frame_id |= b;
                    state = LEN_HIGH; break;
      case LEN_HIGH: header_xor ^= b; payload_len = (uint16_t)b << 8;
                     state = LEN_LOW; break;
      case LEN_LOW:
        header_xor ^= b;
        payload_len |= b;
        if (payload_len > MAX_PAYLOAD) {
            ++crc_error_count; parser_reset();
        } else state = TYPE_HIGH;
        break;
      case TYPE_HIGH: header_xor ^= b; packet_type = (uint16_t)b << 8;
                      state = TYPE_LOW; break;
      case TYPE_LOW:  header_xor ^= b; packet_type |= b;
                      state = HEADER_XOR; break;
      case HEADER_XOR:
        if (b != (uint8_t)(~header_xor)) {
            ++crc_error_count; parser_reset();
        } else {
            payload_pos = 0; data_xor = 0;
            state = payload_len ? PAYLOAD : DATA_XOR;
        }
        break;
      case PAYLOAD:
        // Length was checked before entering this state.
        payload[payload_pos++] = b;
        data_xor ^= b;
        if (payload_pos == payload_len) state = DATA_XOR;
        break;
      case DATA_XOR:
        if (b == (uint8_t)(~data_xor)) {
            complete_frame();
        } else {
            ++crc_error_count;
        }
        parser_reset();
        break;
    }
}

// Owns Serial2 + core filter. Heavy spectral processing never holds the mux.
static void uart_task(void*) {
    for (;;) {
        poll_button();
        int consumed = 0;
        while (Serial2.available() > 0 && consumed < 4096) {
            int x = Serial2.read();
            if (x < 0) break;
            parse_byte((uint8_t)x);
            ++consumed;
        }
        filter.tick(millis());
        publish_snapshot();
        // Allow scheduler/BT tasks even under a continuous inbound byte stream.
        vTaskDelay(pdMS_TO_TICKS(2));
    }
}

static void send_task(void*) {
    char msg[384];
    bool last_connected = false;
    uint32_t last_send = millis();

    for (;;) {
        bool connected = spp_ready;
        if (connected && !last_connected) {
            BT.println("{\"info\":\"ESP32 Radar v8\",\"protocol\":8}");
        }
        last_connected = connected;

        KeyEvent e = {};
        while (key_queue && xQueueReceive(key_queue, &e, 0) == pdTRUE) {
            snprintf(msg, sizeof(msg),
                "{\"evt\":\"key\",\"t\":%lu,\"br_key\":%.2f,"
                "\"n_keys\":%u,\"lag_err\":%.2f,\"lag_valid\":%s,\"gap\":%s}\n",
                (unsigned long)e.timestamp, e.bpm, (unsigned)e.n_keys,
                e.lag_error, e.lag_valid ? "true" : "false",
                e.gap ? "true" : "false");
            if (connected) BT.print(msg);
            Serial.print(msg);
        }

        const uint32_t now = millis();
        if ((uint32_t)(now - last_send) >= SEND_INTERVAL_MS) {
            last_send = now;
            Published snap = {};
            portENTER_CRITICAL(&snap_mux);
            snap = published;
            portEXIT_CRITICAL(&snap_mux);
            // Additional sender-side stale protection if parser task is blocked.
            radar_v8::Result r = snap.result;
            if (r.last_sample_ms &&
                (uint32_t)(now - r.last_sample_ms) > radar_v8::STALE_MS &&
                r.status != radar_v8::NO_HUMAN) {
                r.br = 0.0f;
                r.status = radar_v8::STALE;
            }
            snprintf(msg, sizeof(msg),
                "{\"t\":%lu,\"raw\":%.4f,\"lp\":%.4f,\"hil_ph\":%.3f,"
                "\"br\":%.2f,\"conf\":%.3f,\"amp\":%.3f,"
                "\"cv\":%.3f,\"score\":%.3f,\"fs\":%.2f,\"lag\":%.2f,"
                "\"q\":\"%s\",\"human\":%d,\"n\":%u,"
                "\"frames\":%lu,\"crc_err\":%lu,\"key_drops\":%lu,\"calc_us_max\":%lu}\n",
                (unsigned long)now,
                r.raw, r.lp, r.hil_ph, r.br, r.conf, r.amp, r.cv,
                r.score, r.fs, r.lag, radar_v8::status_name(r.status),
                (int)r.human, (unsigned)r.count,
                (unsigned long)snap.phase_frames,
                (unsigned long)snap.crc_errors,
                (unsigned long)snap.queue_drops,
                (unsigned long)snap.max_filter_us);
            if (connected) BT.print(msg);
            Serial.print(msg);
        }
        vTaskDelay(pdMS_TO_TICKS(5));
    }
}
static void bluetooth_event(esp_spp_cb_event_t event, esp_spp_cb_param_t*) {
    if (event == ESP_SPP_SRV_OPEN_EVT) spp_ready = true;
    if (event == ESP_SPP_CLOSE_EVT) spp_ready = false;
}

void setup() {
    Serial.begin(115200);
    delay(200);
    Serial.println("=== ESP32 mmWave radar v8, 50 Hz phase, 9-30 bpm ===");
    pinMode(BUTTON_PIN, INPUT_PULLUP);

    key_queue = xQueueCreate(16, sizeof(KeyEvent));
    if (!key_queue) {
        Serial.println("FAIL: key queue allocation");
        for (;;) delay(1000);
    }
    Serial2.setRxBufferSize(8192); // must be configured before begin()
    Serial2.begin(RADAR_BAUD, SERIAL_8N1, RADAR_RX_PIN, RADAR_TX_PIN);

    BT.register_callback(bluetooth_event);
    if (!BT.begin(BT_NAME)) {
        Serial.println("FAIL: Classic Bluetooth initialization");
        for (;;) delay(1000);
    }
    publish_snapshot();

    BaseType_t a = xTaskCreatePinnedToCore(
        uart_task, "RadarUART", 8192, nullptr, 2, nullptr, 0);
    BaseType_t b = xTaskCreatePinnedToCore(
        send_task, "RadarSend", 4096, nullptr, 1, nullptr, 1);
    if (a != pdPASS || b != pdPASS) {
        Serial.println("FAIL: task creation");
        for (;;) delay(1000);
    }
}

void loop() {
    vTaskDelay(pdMS_TO_TICKS(1000));
}
