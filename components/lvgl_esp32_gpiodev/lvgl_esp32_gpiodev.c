#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "driver/gpio.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/task.h"

#include "lvgl_esp32_gpiodev.h"
#include "sdkconfig.h"

// lvgl graphics library
#include "lvgl.h"
#include "lvgl_helpers.h"

// LVGL indev pin definitions
#define BTN_SELECT = CONFIG_FP2S2_SW_S_GPIO;
#define BTN_LEFT   = CONFIG_FP2S2_SW_L_GPIO;
#define BTN_RIGHT  = CONFIG_FP2S2_SW_R_GPIO;
#define BTN_BACK   = CONFIG_FP2S2_SW_B_GPIO;

#define GPIO_BTN_PIN_SEL      ((1ULL << BTN_SELECT) | (1ULL << BTN_LEFT) | (1ULL << BTN_RIGHT) | (1ULL << BTN_BACK))
#define ESP_INTR_FLAG_DEFAULT 0

// gpio event queue
static xQueueHandle gpio_evt_queue = NULL;

// gpio states
static bool btn_sel_state;
static bool btn_left_state;
static bool btn_right_state;
static bool btn_back_state;

static void IRAM_ATTR gpio_isr_handler(void *arg) {
    uint32_t gpio_num = (uint32_t)arg;
    xQueueSendFromISR(gpio_evt_queue, &gpio_num, NULL);
}

void lvgl_indev_init(void) {
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

    // create event queue
    gpio_evt_queue = xQueueCreate(10, sizeof(uint32_t));
    // start gpio update task
    xTaskCreate(gpio_state_update, "lvgl_gpiodev_update", 2048, NULL, 10, NULL);

    // install ISR service
    gpio_install_isr_service(ESP_INTR_FLAG_DEFAULT);

    // hook ISR handler to the pins
    gpio_isr_handler_add(BTN_SELECT, gpio_isr_handler, (void *)BTN_SELECT);
    gpio_isr_handler_add(BTN_LEFT, gpio_isr_handler, (void *)BTN_LEFT);
    gpio_isr_handler_add(BTN_RIGHT, gpio_isr_handler, (void *)BTN_RIGHT);
    gpio_isr_handler_add(BTN_BACK, gpio_isr_handler, (void *)BTN_BACK);

    // configure GPIO keypad driver
    lv_indev_drv_t lv_keypad;
    lv_indev_drv_init(&lv_keypad);
    lv_keypad.type = LV_INDEV_TYPE_ENCODER;
}

bool lvgl_gpiodev_read(lv_indev_drv_t *drv, lv_indev_data_t *data) {
    (void)drv;
    uint32_t io_num;
    if (xQueueReceive(gpio_evt_queue, &io_num, 0)) {
        if(gpio_get_level(io_num) == 0) {
            data->state = LV_INDEV_STATE_PRESSED;
        } else {
            data->state = LV_INDEV_STATE_RELEASED;
        }
        data->key = gpio_to_keycode(io_num);
    }
    if(uxQueueMessagesWaiting(gpio_evt_queue) > 0) {
        return true; // we have more events to process
    } else {
        return false; // no more data
    }
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