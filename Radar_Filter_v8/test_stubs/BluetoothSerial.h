#pragma once
enum esp_spp_cb_event_t { ESP_SPP_SRV_OPEN_EVT, ESP_SPP_CLOSE_EVT };
struct esp_spp_cb_param_t {};
class BluetoothSerial {
public:
    bool begin(const char*) { return true; }
    void register_callback(void (*)(esp_spp_cb_event_t, esp_spp_cb_param_t*)) {}
    template<class T> void print(const T&) {}
    template<class T> void println(const T&) {}
};
