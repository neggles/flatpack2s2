#pragma once

#include <stdint.h>
#include <string.h>

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

