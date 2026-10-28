#pragma once
typedef int BaseType_t;
typedef void* QueueHandle_t;
typedef int portMUX_TYPE;
#define portMUX_INITIALIZER_UNLOCKED 0
#define portENTER_CRITICAL(x) (void)(x)
#define portEXIT_CRITICAL(x) (void)(x)
#define pdTRUE 1
#define pdPASS 1
#define pdMS_TO_TICKS(x) (x)
