#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/time.h>

#include "freertos/FreeRTOS.h"
#include "freertos/event_groups.h"
#include "freertos/semphr.h"
#include "freertos/task.h"

#include "esp_event.h"
#include "esp_freertos_hooks.h"
#include "esp_log.h"
#include "esp_system.h"

#include "esp_netif.h"
#include "esp_sntp.h"
#include "esp_spi_flash.h"
#include "esp_wifi.h"

#include "soc/rtc.h"

#include "driver/gpio.h"
#include "driver/i2c.h"

#include "driver/rmt.h"
#include "led_strip.h"

#include "driver/temp_sensor.h"

#include "driver/twai.h"

#include "sdkconfig.h"

/* Littlevgl specific */
#ifdef LV_LVGL_H_INCLUDE_SIMPLE
#    include "lvgl.h"
#else
#    include "lvgl/lvgl.h"
#endif
#include "lvgl_helpers.h"
#define LV_TICK_PERIOD_MS 1

#include "wifi_manager.h"

//* shoving fiddly little helper functions in here
#include "helper_funcs.h"

// TZ string for sntp
#define POSIX_TZ CONFIG_POSIX_TZ

// WS2812 RGB LED RMT channel
#define RMT_LED_CHANNEL RMT_CHANNEL_0

// project log tag
static const char *TAG = "flatpack2s2";

//* Config vars for pin assignments
//static const int btn_left  = CONFIG_FP2S2_SW_L_GPIO;
//static const int btn_sel   = CONFIG_FP2S2_SW_L_GPIO;
//static const int btn_right = CONFIG_FP2S2_SW_L_GPIO;
//static const int btn_back  = GPIO_NUM_0;

//* WiFi status event group
static EventGroupHandle_t appEventGroup;
//* event group bit assignments, 24 bits total
static const int WIFI_CONNECTED_BIT = BIT0;
//static const int TIME_SYNCED_BIT    = BIT1;
//static const int CAN_READY_BIT      = BIT10;
//static const int DISP_READY_BIT     = BIT11;
static const int TEMP_READY_BIT = BIT12;

//* GUI task semaphore
SemaphoreHandle_t xDisplaySemaphore;

//* Global variables - only 32-bit values, so reads and writes are atomic
static float cur_internal_temp;


/**
 * @brief static function prototypes
 */
// tasks
static void displayTask(void *pvParameter);
static void lv_tick_task(void *arg);
static void rgbTask(void *ignore);
static void networkTask(void *ignore);
static void sntpTask(void *ignore);
static void tempSensorTask(void *ignore);
// funcs
static void lvAppCreate(void);
// callbacks
static void cb_connection_ok(void *pvParameter);
static void cb_connection_lost(void *pvParameter);
static void cb_time_sync_event(struct timeval *tv);


/**
 * @brief app main func (does not autoloop)
 */
void app_main(void) {
    printf("flatpack2s2 startup!\n");

    //* initialize the wifi event group
    appEventGroup = xEventGroupCreate();

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

    //* Create OLED display task
    xTaskCreate(&displayTask, "oled_display", 4096 * 2, NULL, 10, NULL);

    //* Create RGB LED update task
    xTaskCreate(&rgbTask, "rgb_led", 1024 * 2, NULL, 5, NULL);

    //* create wifi initialization task
    xTaskCreate(&networkTask, "network_init", 1024 * 2, NULL, 5, NULL);

    //* Create SNTP management task
    xTaskCreate(&sntpTask, "sntp", 1024 * 2, NULL, 2, NULL);

    //* Create temp sensor polling task
    xTaskCreate(&tempSensorTask, "temp_sensor", 1024 * 2, NULL, 2, NULL);
}


//* Task definitions

void sntpTask(void *ignore) {
    static const char *TASKTAG = "sntpTask";
    ESP_LOGI(TASKTAG, "sntp task starting");

    //* Set time zone and SNTP operating parameters
    setenv("TZ", POSIX_TZ, 1);
    tzset();
    sntp_setoperatingmode(SNTP_OPMODE_POLL);
    sntp_setservername(0, CONFIG_SNTP_SERVER);
    sntp_set_time_sync_notification_cb(cb_time_sync_event);
    ESP_LOGI(TASKTAG, "sntp config set, waiting for wifi");

    // wait for wifi connected bit in event group
    xEventGroupWaitBits(appEventGroup, WIFI_CONNECTED_BIT, pdFALSE, pdTRUE, portMAX_DELAY);

    // start SNTP
    ESP_LOGI(TASKTAG, "starting sntp service");
    sntp_init();

    char strftime_buf[64];

    // wait for the service to set the time
    time_t    now         = 0;
    struct tm timeinfo    = {0};
    int       retry       = 0;
    const int retry_count = 10;
    while (sntp_get_sync_status() == SNTP_SYNC_STATUS_RESET && ++retry < retry_count) {
        ESP_LOGI(TASKTAG, "Waiting for system time to be set... (%d/%d)", retry, retry_count);
        vTaskDelay(pdMS_TO_TICKS(3000));
    }
    time(&now);
    localtime_r(&now, &timeinfo);

    strftime(strftime_buf, sizeof(strftime_buf), "%c", &timeinfo);

    ESP_LOGI(TASKTAG, "current date/time: %s", strftime_buf);
    vTaskDelete(NULL);
}

void displayTask(void *pvParameter) {
    static const char *TASKTAG = "displayTask";
    ESP_LOGI(TASKTAG, "display task begin");

    (void)pvParameter;
    xDisplaySemaphore = xSemaphoreCreateMutex();

    lv_init();

    /* Initialize SPI or I2C bus used by the drivers */
    lvgl_driver_init();

    lv_color_t *buf1 = heap_caps_malloc(DISP_BUF_SIZE * sizeof(lv_color_t), MALLOC_CAP_DMA);
    assert(buf1 != NULL);
    static lv_color_t *  buf2 = NULL;
    static lv_disp_buf_t disp_buf;
    /* Actual size in pixels, not bytes. */
    uint32_t             size_in_px = DISP_BUF_SIZE * 8;

    /* Initialize the working buffer depending on the selected display.
     * NOTE: buf2 == NULL when using monochrome displays. */
    lv_disp_buf_init(&disp_buf, buf1, buf2, size_in_px);

    lv_disp_drv_t disp_drv;
    lv_disp_drv_init(&disp_drv);
    disp_drv.flush_cb = disp_driver_flush;

    /* When using a monochrome display we need to register the callbacks:
     * - rounder_cb
     * - set_px_cb */
    disp_drv.rounder_cb = disp_driver_rounder;
    disp_drv.set_px_cb  = disp_driver_set_px;
    disp_drv.buffer     = &disp_buf;
    lv_disp_drv_register(&disp_drv);

    /* Create and start a periodic timer interrupt to call lv_tick_inc */
    const esp_timer_create_args_t periodic_timer_args = {
        .callback = &lv_tick_task,
        .name     = "periodic_gui"};
    esp_timer_handle_t periodic_timer;
    ESP_ERROR_CHECK(esp_timer_create(&periodic_timer_args, &periodic_timer));
    ESP_ERROR_CHECK(esp_timer_start_periodic(periodic_timer, LV_TICK_PERIOD_MS * 1000));

    // start the app
    lvAppCreate();

    while (1) {
        /* Delay 1 tick (assumes FreeRTOS tick is 10ms */
        vTaskDelay(pdMS_TO_TICKS(10));

        /* Try to take the semaphore, call lvgl related function on success */
        if (pdTRUE == xSemaphoreTake(xDisplaySemaphore, portMAX_DELAY)) {
            lv_task_handler();
            xSemaphoreGive(xDisplaySemaphore);
        }
    }

    /* A task should NEVER return */
    free(buf1);
    vTaskDelete(NULL);
}

static void lv_tick_task(void *arg) {
    (void)arg;

    lv_tick_inc(LV_TICK_PERIOD_MS);
}

static void lvAppCreate(void) {
    static const char *TASKTAG = "lvAppCreate";
    /* use a pretty small demo for monochrome displays */
    /* Get the current screen  */
    lv_obj_t *scr = lv_disp_get_scr_act(NULL);

    /*Create a Label on the currently active screen*/
    lv_obj_t *label1 = lv_label_create(scr, NULL);

    /*Modify the Label's text*/
    lv_label_set_text(label1, "Hello\nworld");

    /* Align the Label to the center
     * NULL means align on parent (which is the screen now)
     * 0, 0 at the end means an x, y offset after alignment*/
    lv_obj_align(label1, NULL, LV_ALIGN_CENTER, 0, 0);

    ESP_LOGI(TASKTAG, "completed lvAppCreate");
}

void tempSensorTask(void *arg) {
    static const char *TASKTAG = "rgbTask";
    ESP_LOGI(TASKTAG, "Initializing temp sensor");

    temp_sensor_config_t temp_sensor = TSENS_CONFIG_DEFAULT();
    temp_sensor_get_config(&temp_sensor);
    ESP_LOGI(TASKTAG, "default dac %d, clk_div %d", temp_sensor.dac_offset, temp_sensor.clk_div);

    temp_sensor.dac_offset = TSENS_DAC_DEFAULT; // DEFAULT: range:-10℃ ~  80℃, error < 1℃.
    temp_sensor_set_config(temp_sensor);
    temp_sensor_start();
    ESP_LOGI(TAG, "Initialization complete");

    temp_sensor_read_celsius(&cur_internal_temp);
    xEventGroupSetBits(appEventGroup, TEMP_READY_BIT);
    while (true) {
        temp_sensor_read_celsius(&cur_internal_temp);
        ESP_LOGI(TASKTAG, "current chip temp %.3f°C", cur_internal_temp);
        vTaskDelay(pdMS_TO_TICKS(10000));
    }

    ESP_LOGI(TASKTAG, "ending task");
    vTaskDelete(NULL);
}

void rgbTask(void *arg) {
    static const char *TASKTAG = "rgbTask";

    uint32_t red   = 0;
    uint32_t green = 0;
    uint32_t blue  = 0;

    rmt_config_t config = RMT_DEFAULT_CONFIG_TX(CONFIG_FP2S2_RGB_GPIO, RMT_LED_CHANNEL);
    // set counter clock to 40MHz
    config.clk_div = 2;
    // install RMT driver
    ESP_ERROR_CHECK(rmt_config(&config));
    ESP_ERROR_CHECK(rmt_driver_install(config.channel, 0, 0));

    // install ws2812 driver
    led_strip_config_t led_config = LED_STRIP_DEFAULT_CONFIG(1, (led_strip_dev_t)config.channel);
    led_strip_t *      rgb_led    = led_strip_new_rmt_ws2812(&led_config);
    if (!rgb_led) {
        ESP_LOGE(TASKTAG, "initialization failed! giving up...");
        vTaskDelete(NULL);
    }

    // turn off LED
    ESP_ERROR_CHECK(rgb_led->clear(rgb_led, 100));
    ESP_LOGI(TASKTAG, "initialization complete");

    // this is a placeholder that just does a rainbow cycle until I have this actually set up
    while (true) {
        for (int i = 0; i < 360; i++) {
            // build RGB value
            led_hsv2rgb(i, 100, 30, &red, &green, &blue);
            ESP_ERROR_CHECK(rgb_led->set_pixel(rgb_led, 0, red, green, blue));
            // Flush RGB value to LED
            ESP_ERROR_CHECK(rgb_led->refresh(rgb_led, 100));
            vTaskDelay(pdMS_TO_TICKS(10));
            //rgb_led->clear(rgb_led, 50);
        }
    }
    vTaskDelete(NULL);
}

/**
 * @brief initializes wifi manager. doesn't really need to be its own task, but lets me set this as low-prio for background
 */
void networkTask(void *arg) {
    // start the wifi manager
    wifi_manager_start();
    // register wifi callbacks
    wifi_manager_set_callback(WM_EVENT_STA_GOT_IP, &cb_connection_ok);
    wifi_manager_set_callback(WM_EVENT_STA_DISCONNECTED, &cb_connection_lost);
    // done
    vTaskDelete(NULL);
}


//* Callbacks
/**
 * @brief wifi connection manager callbacks
 */
void cb_connection_ok(void *pvParameter) {
    ip_event_got_ip_t *param = (ip_event_got_ip_t *)pvParameter;
    xEventGroupSetBits(appEventGroup, WIFI_CONNECTED_BIT);

    /* transform IP to human readable string */
    char str_ip[16];
    esp_ip4addr_ntoa(&param->ip_info.ip, str_ip, IP4ADDR_STRLEN_MAX);

    ESP_LOGI(TAG, "wifi connected, yay! acquired IP address: %s", str_ip);
}

void cb_connection_lost(void *pvParameter) {
    wifi_event_sta_disconnected_t *param = (wifi_event_sta_disconnected_t *)pvParameter;
    xEventGroupClearBits(appEventGroup, WIFI_CONNECTED_BIT);

    ESP_LOGW(TAG, "wifi disconnected, boo! reason:%d", param->reason);
}

void cb_time_sync_event(struct timeval *tv) {
    ESP_LOGI(TAG, "Notification of a time synchronization event");
}
