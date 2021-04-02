#pragma once

#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "hal/twai_types.h"
#include <esp_log.h>

typedef struct {
    uint8_t hue;
    uint8_t saturation;
    uint8_t brightness;
} led_hsv_t;

// hsv-to-RGB for LED control
void led_hsv2rgb(uint32_t h, uint32_t s, uint32_t v, uint32_t *r, uint32_t *g, uint32_t *b);

// twai message log func
void logTwaiMsg(twai_message_t *twaiMsg, int is_tx, const char *msgType, esp_log_level_t errLevel);
