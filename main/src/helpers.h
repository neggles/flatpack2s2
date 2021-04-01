#ifndef PROJ_HELPERS_H_
#define PROJ_HELPERS_H_

typedef struct {
    uint8_t hue;
    uint8_t saturation;
    uint8_t brightness;
} led_hsv_t;

// hsv-to-RGB for LED control
void led_hsv2rgb(uint32_t h, uint32_t s, uint32_t v, uint32_t *r, uint32_t *g, uint32_t *b);

#endif /* !PROJ_HELPERS_H_ */