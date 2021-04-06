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