#pragma once
#include "FreeRTOS.h"
inline void vTaskDelay(unsigned long) {}
inline BaseType_t xTaskCreatePinnedToCore(
    void (*)(void*), const char*, unsigned, void*, int, void*, int) { return pdPASS; }
