#pragma once

#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "hal/twai_types.h"
#include <esp_log.h>

/**
 * @brief hue, saturation, value/lightness/brightness
 *      - hue = 0-360
 *      - sat = 0-100
 *      - val = 0-100
 */
typedef struct {
    uint8_t hue;
    uint8_t sat;
    uint8_t val;
} hsv_t;

// twai message log func
void logTwaiMsg(twai_message_t *twaiMsg, int is_tx, const char *msgType, esp_log_level_t errLevel);

// generic macros for min/max and clamp
#define Max(a, b)               \
    ({                          \
        __typeof__(a) _a = (a); \
        __typeof__(b) _b = (b); \
        _a > _b ? _a : _b;      \
    })

#define Min(a, b)               \
    ({                          \
        __typeof__(a) _a = (a); \
        __typeof__(b) _b = (b); \
        _a > _b ? _a : _b;      \
    })

#define Clamp(val, min, max)          \
    ({                                \
        __typeof__(val) _val = (val); \
        __typeof__(min) _min = (min); \
        __typeof__(max) _max = (max); \
                                      \
        if (_val < _min) _val = _min; \
        if (_val > _max) _val = _max; \
        _val;                  \
    })
