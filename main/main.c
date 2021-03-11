#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>
#include <sys/time.h>

#include "freertos/FreeRTOS.h"
#include "freertos/event_groups.h"
#include "freertos/task.h"


#include "esp_system.h"
#include "esp_event.h"
#include "esp_log.h"

#include "esp_wifi.h"
#include "esp_netif.h"
#include "esp_spi_flash.h"
#include "esp_sntp.h"

#include "soc/rtc.h"

#include "driver/gpio.h"
#include "driver/i2c.h"
#include "driver/twai.h"
#include "driver/temp_sensor.h"
#include "driver/rmt.h"

#include "sdkconfig.h"

#include "u8g2.h"
#include "u8g2_esp32_hal.h"
#include "wifi_manager.h"
#include "led_strip.h"

#define OLED_SDA CONFIG_I2C_OLED_SDA
#define OLED_SCL CONFIG_I2C_OLED_SCL
#define OLED_RST CONFIG_I2C_OLED_RST

#define RMT_TX_CHANNEL RMT_CHANNEL_0
#define EXAMPLE_CHASE_SPEED_MS (10)

static const char *TAG = "flatpack2s2";

// Event group for inter-task communication
static EventGroupHandle_t wifi_event_group;
const int WIFI_CONNECTED_BIT = BIT0;

/**
 * @brief Simple helper function, converting HSV color space to RGB color space
 */
void led_hsv2rgb(uint32_t h, uint32_t s, uint32_t v, uint32_t *r, uint32_t *g, uint32_t *b)
{
    h %= 360; // h -> [0,360]
    uint32_t rgb_max = v * 2.55f;
    uint32_t rgb_min = rgb_max * (100 - s) / 100.0f;

    uint32_t i = h / 60;
    uint32_t diff = h % 60;

    // RGB adjustment amount by hue
    uint32_t rgb_adj = (rgb_max - rgb_min) * diff / 60;

    switch (i) {
    case 0:
        *r = rgb_max;
        *g = rgb_min + rgb_adj;
        *b = rgb_min;
        break;
    case 1:
        *r = rgb_max - rgb_adj;
        *g = rgb_max;
        *b = rgb_min;
        break;
    case 2:
        *r = rgb_min;
        *g = rgb_max;
        *b = rgb_min + rgb_adj;
        break;
    case 3:
        *r = rgb_min;
        *g = rgb_max - rgb_adj;
        *b = rgb_max;
        break;
    case 4:
        *r = rgb_min + rgb_adj;
        *g = rgb_min;
        *b = rgb_max;
        break;
    default:
        *r = rgb_max;
        *g = rgb_min;
        *b = rgb_max - rgb_adj;
        break;
    }
}

void task_display(void *ignore) {

    u8g2_esp32_hal_t u8g2_esp32_hal = U8G2_ESP32_HAL_DEFAULT;
    u8g2_esp32_hal.sda              = OLED_SDA;
    u8g2_esp32_hal.scl              = OLED_SCL;
    u8g2_esp32_hal.reset            = OLED_RST;
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
    u8g2_SetPowerSave(&u8g2, 0); // wake up display
    u8g2_ClearBuffer(&u8g2);
    ESP_LOGI(TAG, "u8g2 init complete, doing a thing!");

    u8g2_DrawBox(&u8g2, 0, 26, 80, 6);
    u8g2_DrawFrame(&u8g2, 0, 26, 128, 6);

    u8g2_SetFont(&u8g2, u8g2_font_profont12_tf);
    u8g2_DrawStr(&u8g2, 2, 17, "hi i am esp32s2");
    u8g2_SendBuffer(&u8g2);

    ESP_LOGI(TAG, "u8g2 sleeping for 5S then turning off the oled so it doesn't died");
    vTaskDelay(5000 / portTICK_RATE_MS);
    ESP_LOGI(TAG, "sleep done, turning off");
    u8g2_ClearBuffer(&u8g2);
    u8g2_SendBuffer(&u8g2);
    u8g2_SetPowerSave(&u8g2, 1);

    ESP_LOGI(TAG, "ending task");

    vTaskDelete(NULL);
}


/**
 * @brief wifi connection manager callbacks
 */
void cb_connection_ok(void *pvParameter) {
    ip_event_got_ip_t *param = (ip_event_got_ip_t *)pvParameter;
    xEventGroupSetBits(wifi_event_group, WIFI_CONNECTED_BIT);

    /* transform IP to human readable string */
    char str_ip[16];
    esp_ip4addr_ntoa(&param->ip_info.ip, str_ip, IP4ADDR_STRLEN_MAX);

    ESP_LOGI(TAG, "I have a connection and my IP is %s!", str_ip);
}

void cb_connection_lost(void *pvParameter) {
    wifi_event_sta_disconnected_t *param = (wifi_event_sta_disconnected_t *)pvParameter;
    xEventGroupClearBits(wifi_event_group, WIFI_CONNECTED_BIT);

    ESP_LOGI(TAG, "wifi disconnected! reason:%d", param->reason);
}

void task_temp_sensor(void *arg)
{
    // Initialize touch pad peripheral, it will start a timer to run a filter
    ESP_LOGI(TAG, "Initializing Temperature sensor");
    float tsens_out;
    temp_sensor_config_t temp_sensor = TSENS_CONFIG_DEFAULT();
    temp_sensor_get_config(&temp_sensor);
    ESP_LOGI(TAG, "default dac %d, clk_div %d", temp_sensor.dac_offset, temp_sensor.clk_div);
    temp_sensor.dac_offset = TSENS_DAC_DEFAULT; // DEFAULT: range:-10℃ ~  80℃, error < 1℃.
    temp_sensor_set_config(temp_sensor);
    temp_sensor_start();
    ESP_LOGI(TAG, "Temperature sensor started");
    while (1) {
        vTaskDelay(10000 / portTICK_RATE_MS);
        temp_sensor_read_celsius(&tsens_out);
        ESP_LOGI(TAG, "chip internal temperature: %f°C", tsens_out);
    }
    vTaskDelete(NULL);
}

void task_rgb_led(void *arg) {
  uint32_t red = 0;
    uint32_t green = 0;
    uint32_t blue = 0;
    uint16_t hue = 0;
    uint16_t start_rgb = 0;

    rmt_config_t config = RMT_DEFAULT_CONFIG_TX(CONFIG_FP2S2_RGB_GPIO, RMT_TX_CHANNEL);
    // set counter clock to 40MHz
    config.clk_div = 2;

    ESP_ERROR_CHECK(rmt_config(&config));
    ESP_ERROR_CHECK(rmt_driver_install(config.channel, 0, 0));

    // install ws2812 driver
    led_strip_config_t strip_config = LED_STRIP_DEFAULT_CONFIG(1, (led_strip_dev_t)config.channel);
    led_strip_t *strip = led_strip_new_rmt_ws2812(&strip_config);
    if (!strip) {
        ESP_LOGE(TAG, "install WS2812 driver failed");
    }
    // Clear LED strip (turn off all LEDs)
    ESP_ERROR_CHECK(strip->clear(strip, 100));
    // Show simple rainbow chasing pattern
    ESP_LOGI(TAG, "LED Rainbow Chase Start");
    while (true) {
        for (int i = 0; i < 3; i++) {
            for (int j = i; j < 1; j += 3) {
                // Build RGB values
                hue = j * 360 / 1 + start_rgb;
                led_hsv2rgb(hue, 100, 100, &red, &green, &blue);
                // Write RGB values to strip driver
                ESP_ERROR_CHECK(strip->set_pixel(strip, j, red, green, blue));
            }
            // Flush RGB values to LEDs
            ESP_ERROR_CHECK(strip->refresh(strip, 100));
            vTaskDelay(pdMS_TO_TICKS(EXAMPLE_CHASE_SPEED_MS));
            strip->clear(strip, 50);
            vTaskDelay(pdMS_TO_TICKS(EXAMPLE_CHASE_SPEED_MS));
        }
        start_rgb += 60;
    }
}



/**
 * @brief app main func (does not autoloop)
 */
void app_main(void) {
    printf("flatpack2s2 startup!\n");

    //* initialize the wifi event group
    wifi_event_group = xEventGroupCreate();

    /* start the wifi manager */
    wifi_manager_start();
    /* register wifi connection manager callbacks */
    wifi_manager_set_callback(WM_EVENT_STA_GOT_IP, &cb_connection_ok);
    wifi_manager_set_callback(WM_EVENT_STA_DISCONNECTED, &cb_connection_lost);

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
    xTaskCreate(&task_display, "oled_display", 1024 * 2, NULL, 10, NULL);
    //* and the temp task
    xTaskCreate(&task_temp_sensor, "temp_sensor", 1024 * 2, NULL, 10, NULL);
    //* and the rgb led task
    xTaskCreate(&task_rgb_led, "rgb_led", 1024 * 2, NULL, 10, NULL);
}
