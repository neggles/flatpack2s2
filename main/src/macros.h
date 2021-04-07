#pragma once

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
        _val;                         \
    })

#define LowByte(val)  ({ (uint8_t)(val & 0xff); })
#define HighByte(val) ({ (uint8_t)((val >> 8) & 0xff); })

// TWAI general config for ISR mode
#define TWAI_GENERAL_CONFIG_IRAM(tx_io_num, rx_io_num, op_mode)                                                \
    {                                                                                                          \
        .mode = op_mode, .tx_io = tx_io_num, .rx_io = rx_io_num, .clkout_io = TWAI_IO_UNUSED,                  \
        .bus_off_io = TWAI_IO_UNUSED, .tx_queue_len = 5, .rx_queue_len = 5, .alerts_enabled = TWAI_ALERT_NONE, \
        .clkout_divider = 0, .intr_flags = ESP_INTR_FLAG_IRAM                                                  \
    }
