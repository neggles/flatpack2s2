#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include <esp_netif.h>
#include <esp_spi_flash.h>
#include <esp_wifi.h>

#include <soc/rtc.h>

#include <driver/gpio.h>
#include <driver/i2c.h>
#include <driver/spi_master.h>

#include "sdkconfig.h"

#include "esp_log.h"
#include "esp_system.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "u8g2.h"
#include "u8g2_esp32_hal.h"
#include "wifi_manager.h"

#define PIN_SDA GPIO_NUM_4
#define PIN_SCL GPIO_NUM_5
#define PIN_RST GPIO_NUM_16

static const char *TAG = "flatpack2s2";

void task_init_oled(void *ignore) {

    u8g2_esp32_hal_t u8g2_esp32_hal = U8G2_ESP32_HAL_DEFAULT;
    u8g2_esp32_hal.sda              = PIN_SDA;
    u8g2_esp32_hal.scl              = PIN_SCL;
    u8g2_esp32_hal.reset            = PIN_RST;
    u8g2_esp32_hal_init(u8g2_esp32_hal);

    u8g2_t u8g2; // a structure which will contain all the data for one display
    u8g2_Setup_ssd1306_i2c_128x64_noname_f(
        &u8g2,
        U8G2_R0,
        u8g2_esp32_i2c_byte_cb,
        u8g2_esp32_gpio_and_delay_cb); // init u8g2 structure
    u8x8_SetI2CAddress(&u8g2.u8x8, 0x78);

    ESP_LOGI(TAG, "u8g2_InitDisplay");
    u8g2_InitDisplay(&u8g2); // send init sequence to the display, display is in sleep mode after this,
    ESP_LOGI(TAG, "u8g2_SetPowerSave");
    u8g2_SetPowerSave(&u8g2, 0); // wake up display
    ESP_LOGI(TAG, "u8g2_ClearBuffer");
    u8g2_ClearBuffer(&u8g2);

    ESP_LOGI(TAG, "u8g2_DrawBox");
    u8g2_DrawBox(&u8g2, 0, 26, 80, 6);
    u8g2_DrawFrame(&u8g2, 0, 26, 128, 6);

    ESP_LOGI(TAG, "u8g2_SetFont");
    u8g2_SetFont(&u8g2, u8g2_font_profont12_tf);
    ESP_LOGI(TAG, "u8g2_DrawStr");
    u8g2_DrawStr(&u8g2, 2, 17, "hi i am esp32s2");
    ESP_LOGI(TAG, "u8g2_SendBuffer");
    u8g2_SendBuffer(&u8g2);

    ESP_LOGI(TAG, "All done!");
    vTaskDelete(NULL);
}

/**
 * @brief wifi connection manager callback
 */
void cb_connection_ok(void *pvParameter) {
    ip_event_got_ip_t *param = (ip_event_got_ip_t *)pvParameter;

    /* transform IP to human readable string */
    char str_ip[16];
    esp_ip4addr_ntoa(&param->ip_info.ip, str_ip, IP4ADDR_STRLEN_MAX);

    ESP_LOGI(TAG, "I have a connection and my IP is %s!", str_ip);
}

void app_main(void) {
    printf("flatpack2s2 startup!\n");

    /* start the wifi manager */
    wifi_manager_start();
    /* register wifi connection manager callback */
    wifi_manager_set_callback(WM_EVENT_STA_GOT_IP, &cb_connection_ok);

    //* Print chip information
    esp_chip_info_t chip_info;
    esp_chip_info(&chip_info);
    rtc_cpu_freq_config_t clk_config;
    rtc_clk_cpu_freq_get_config(&clk_config);
    printf("ESP32-S2 at %dMHz, WiFi%s%s, ",
           clk_config.freq_mhz,
           (chip_info.features & CHIP_FEATURE_BT) ? "/BT" : "",
           (chip_info.features & CHIP_FEATURE_BLE) ? "/BLE" : "");
    printf("silicon revision %d, ", chip_info.revision);
    printf("%dMB %s flash\n", spi_flash_get_chip_size() / (1024 * 1024),
           (chip_info.features & CHIP_FEATURE_EMB_FLASH) ? "embedded" : "external");

    //* fire off the i2c oled task
    xTaskCreate(task_init_oled, "i2c_oled_init", 1024 * 2, (void *)0, 10, NULL);
}
