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
