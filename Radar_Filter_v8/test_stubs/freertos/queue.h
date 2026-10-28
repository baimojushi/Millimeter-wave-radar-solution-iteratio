#pragma once
#include "FreeRTOS.h"
inline QueueHandle_t xQueueCreate(unsigned, unsigned) { return (void*)1; }
inline BaseType_t xQueueSend(QueueHandle_t, const void*, unsigned) { return pdTRUE; }
inline BaseType_t xQueueReceive(QueueHandle_t, void*, unsigned) { return 0; }
