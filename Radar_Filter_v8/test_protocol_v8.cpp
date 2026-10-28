// Host-only parser test: includes the ESP32 wrapper against minimal API stubs.
// This confirms parser behavior and C++ syntax, NOT actual hardware compatibility.
#include <vector>
#include <cstdio>
#include <cmath>
#include <cstring>
#include <limits>
#include "main_bt_8.cpp"

HardwareSerial Serial, Serial2;
static uint32_t fake_ms = 0;
static int fake_button = HIGH;
uint32_t millis() { return fake_ms; }
uint32_t micros() { return fake_ms * 1000u; }
void delay(unsigned long) {}
int digitalRead(int) { return fake_button; }
void pinMode(int, int) {}

static std::vector<uint8_t> frame(uint16_t type,
                                  const std::vector<uint8_t>& data) {
    std::vector<uint8_t> p = {1, 0, 1, (uint8_t)(data.size() >> 8),
       (uint8_t)(data.size() & 255), (uint8_t)(type >> 8),
       (uint8_t)type};
    uint8_t h = 0, d = 0;
    for (uint8_t b : p) h ^= b;
    p.push_back((uint8_t)(~h));
    for (uint8_t b : data) { p.push_back(b); d ^= b; }
    p.push_back((uint8_t)(~d));
    return p;
}
static void feed(const std::vector<uint8_t>& bytes) {
    for (uint8_t b : bytes) parse_byte(b);
}
static void phase_packet(float f) {
    uint8_t bytes[4]; std::memcpy(bytes, &f, 4);
    feed(frame(TF_PHASE, {bytes[0], bytes[1], bytes[2], bytes[3]}));
}
static void human_packet(bool present) {
    feed(frame(TF_HUMAN, {(uint8_t)(present ? 1 : 0), 0}));
}
static int failures = 0;
static void verify(bool ok, const char* msg) {
    std::printf("[%s] %s\n", ok ? "PASS" : "FAIL", msg);
    if (!ok) ++failures;
}
int main() {
    phase_packet(1.1f);
    verify(phase_frame_count == 1, "accept valid phase frame");
    auto invalid_header = frame(TF_PHASE, {0,0,0,0});
    invalid_header[7] ^= 0x20;
    feed(invalid_header);
    verify(phase_frame_count == 1 && crc_error_count >= 1,
           "reject corrupt header checksum");
    // Regression: an invalid checksum equal to SOF (0x01) must not
    // consume the NEXT valid frame's SOF as an ID byte.
    const uint32_t before_recovery = phase_frame_count;
    auto bad_tail_sof = frame(TF_PHASE, {0,0,0,0});
    bad_tail_sof.back() = 0x01;
    feed(bad_tail_sof);
    phase_packet(0.8f);
    verify(phase_frame_count == before_recovery + 1,
           "parser recovers after malformed CRC byte 0x01");
    auto invalid_data = frame(TF_PHASE, {0,0,0,0});
    invalid_data.back() ^= 0x20;
    feed(invalid_data);
    verify(phase_frame_count == 2 && crc_error_count >= 3,
           "reject corrupt payload checksum");

    // Oversized 65-byte payload announced in header; parser must not overflow.
    uint32_t before = crc_error_count;
    feed({1, 0, 1, 0, 65});
    verify(crc_error_count == before + 1 && state == FIND_SOF,
           "reject oversized payload before storage");

    // Recovery from a truncated frame, using 200ms inter-byte timeout.
    const auto truncated = frame(TF_PHASE, {0,0,0,0});
    feed({truncated[0], truncated[1], truncated[2], truncated[3],
          truncated[4], truncated[5], truncated[6], truncated[7],
          truncated[8]});
    fake_ms += 300;
    const uint32_t before_timeout = phase_frame_count;
    phase_packet(0.9f);
    verify(phase_frame_count == before_timeout + 1,
           "parser recovers from truncated frame after 200ms idle");
    human_packet(true);
    for (int i = 0; i < 1300; ++i) {
        fake_ms = i * 20u;
        phase_packet(0.8f * sinf(radar_v8::K_TAU * 16.0f/60.0f * (i/50.0f)));
    }
    verify(filter.result().status == radar_v8::OK &&
           fabsf(filter.result().br-16.0f) < 0.8f,
           "live protocol->filter path estimates 16 bpm");
    human_packet(false);
    verify(filter.result().status == radar_v8::NO_HUMAN &&
           filter.result().br == 0.0f,
           "no-human TF flag immediately clears estimate");
    phase_packet(100.0f);
    verify(filter.result().status == radar_v8::NO_HUMAN,
           "phase packets ignored with no human");
    human_packet(true);
    fake_ms += 20;
    phase_packet(std::numeric_limits<float>::infinity());
    verify(filter.result().status == radar_v8::BAD_INPUT,
           "valid TF packet with Inf float rejected");

    key_queue = xQueueCreate(16, sizeof(KeyEvent));
    fake_ms = 100;
    fake_button = HIGH;
    poll_button();
    fake_button = LOW;
    poll_button();
    fake_ms = 150;
    poll_button();
    verify(key_count == 0, "button bounce (<80ms) is ignored");
    fake_ms = 190;
    poll_button();
    verify(key_count == 1, "stable press accepted after debounce");
    fake_button = HIGH;
    fake_ms = 300; poll_button();
    fake_ms = 400; poll_button();
    fake_button = LOW;
    fake_ms = 3200; poll_button();
    fake_ms = 3300; poll_button();
    verify(key_count == 2, "second stable respiratory annotation accepted");

    std::printf("PROTOCOL RESULT: %d failures\n", failures);
    return failures ? 1 : 0;
}
