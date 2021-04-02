#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "driver/gpio.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/task.h"

#include "sdkconfig.h"
#include "lvgl_gpiodev.h"

// lvgl graphics library
#ifdef LV_LVGL_H_INCLUDE_SIMPLE
#include "src/lv_core/lv_indev.h"
#include "src/lv_hal/lv_hal_indev.h"
#else
#include "lvgl/src/lv_core/lv_indev.h"
#include "lvgl/src/lv_hal/lv_hal_indev.h"
#endif

// LVGL indev pin definitions
#define BTN_SELECT CONFIG_FP2S2_SW_S_GPIO
#define BTN_LEFT   CONFIG_FP2S2_SW_L_GPIO
#define BTN_RIGHT  CONFIG_FP2S2_SW_R_GPIO
#define BTN_BACK   CONFIG_FP2S2_SW_B_GPIO

#define GPIO_BTN_PIN_SEL      ((1ULL << BTN_SELECT) | (1ULL << BTN_LEFT) | (1ULL << BTN_RIGHT) | (1ULL << BTN_BACK))
#define ESP_INTR_FLAG_DEFAULT 0

// gpio state var
static uint32_t last_gpio;

// function prototypes

static void IRAM_ATTR gpio_isr_handler(void *arg) {
    last_gpio = (uint32_t)arg;
}

void lvgl_gpiodev_init(void) {
    // configure GPIOs for input device
    gpio_config_t io_conf;
    // we want to be told about both rising and falling edges
    io_conf.intr_type = GPIO_INTR_NEGEDGE;
    // input mode
    io_conf.mode = GPIO_MODE_INPUT;
    // bitmask for the pins we're configuring
    io_conf.pin_bit_mask = GPIO_BTN_PIN_SEL;
    // pull-up, no pull-down
    io_conf.pull_up_en   = 1;
    io_conf.pull_down_en = 0;
    // submit config
    gpio_config(&io_conf);

    // install ISR service
    gpio_install_isr_service(ESP_INTR_FLAG_DEFAULT);

    // hook ISR handler to the pins
    gpio_isr_handler_add(BTN_SELECT, gpio_isr_handler, (void *)BTN_SELECT);
    gpio_isr_handler_add(BTN_LEFT, gpio_isr_handler, (void *)BTN_LEFT);
    gpio_isr_handler_add(BTN_RIGHT, gpio_isr_handler, (void *)BTN_RIGHT);
    gpio_isr_handler_add(BTN_BACK, gpio_isr_handler, (void *)BTN_BACK);
}

// map GPIO number to LVGL keycode
static uint32_t gpio_to_keycode(uint32_t io_num) {
    switch (io_num) {
        case BTN_SELECT: return LV_KEY_ENTER;
        case BTN_LEFT: return LV_KEY_LEFT;
        case BTN_RIGHT: return LV_KEY_RIGHT;
        case BTN_BACK: return LV_KEY_ESC;
        default: return 0;
    }
}

bool lvgl_gpiodev_read(lv_indev_drv_t *drv, lv_indev_data_t *data) {
    (void)drv;
    int level = gpio_get_level(last_gpio);
    if (level == 0) {
        data->state = LV_INDEV_STATE_PR;
    } else {
        data->state = LV_INDEV_STATE_REL;
    }
    data->key = gpio_to_keycode(last_gpio);
    return false; // no more data
}
