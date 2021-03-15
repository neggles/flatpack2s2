/**
 * * IDF Includes
 */
// cstd libraries
#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/time.h>

// FreeRTOS Components
#include "freertos/FreeRTOS.h"
#include "freertos/event_groups.h"
#include "freertos/semphr.h"
#include "freertos/task.h"

// SDK config
#include "sdkconfig.h"

// IDF system components
#include "esp_event.h"
#include "esp_freertos_hooks.h"
#include "esp_log.h"
#include "esp_netif.h"
#include "esp_spi_flash.h"
#include "esp_system.h"
#include "esp_wifi.h"

// IDF applications
#include "esp_sntp.h"

// IDF drivers
#include "driver/gpio.h"
#include "driver/i2c.h"
#include "driver/rmt.h"
#include "driver/temp_sensor.h"
#include "driver/twai.h"
#include "soc/rtc.h"

/**
 * * Project Components
 */
// RGB LED strip driver (using RMT peripheral)
#include "led_strip.h"

// lvgl graphics library
#ifdef LV_LVGL_H_INCLUDE_SIMPLE
#    include "lvgl.h"
#else
#    include "lvgl/lvgl.h"
#endif
#include "lvgl_helpers.h"
#define LV_TICK_PERIOD_MS 1

// ESP Wifi Manager
#include "wifi_manager.h"

// Extra project source files (helper functions etc.)
#include "helper_funcs.h"

/**
 * * Log tag strings
 */
static const char *TAG               = "flatpack2s2";
static const char *APP_CREATE_TAG    = "lvAppCreate";
static const char *DISP_TASK_TAG     = "displayTask";
static const char *TWAI_TASK_TAG     = "twaiTask";
static const char *LED_TASK_TAG      = "ledTask";
static const char *TIMESYNC_TASK_TAG = "timeSyncTask";
static const char *FP2_ALERT_TAG     = "fp2AlertMessage";

/**
 * * App Configuration
 */

// TZ string for sntp
#define POSIX_TZ CONFIG_POSIX_TZ

// WS2812 RGB LED RMT channel
#define RMT_LED_CHANNEL RMT_CHANNEL_0

// TWAI transcever (MAX3051) enable pin, active-low
#define GPIO_TWAI_EN     CONFIG_TWAI_EN_GPIO
#define GPIO_TWAI_EN_SEL (1ULL << GPIO_TWAI_EN)

// interval for internal temp sensor measurements
#define TEMP_POLL_PERIOD 10

// * Pin definitions
static const int btn_left  = CONFIG_FP2S2_SW_L_GPIO;
static const int btn_enter = CONFIG_FP2S2_SW_S_GPIO;
static const int btn_right = CONFIG_FP2S2_SW_R_GPIO;
static const int btn_back  = GPIO_NUM_0;


// * Event Group for general status flags
static EventGroupHandle_t appEventGroup;
// bit assignments
static const int WIFI_CONNECTED_BIT = BIT0;
static const int TWAI_RUN_BIT       = BIT2;
static const int FP2_FOUND_BIT      = BIT3;
static const int FP2_LOGGED_IN_BIT  = BIT4;
static const int TIMESYNC_RUN_BIT   = BIT10;
static const int DISP_RUN_BIT       = BIT11;
static const int TEMP_RUN_BIT       = BIT12;
static const int LED_RUN_BIT        = BIT13;

// * Semaphores
SemaphoreHandle_t xDisplaySemaphore; // lvgl2
SemaphoreHandle_t xTwaiSemaphore;


/**
 * * Flatpack2 related stuff
 */

// bitmasks to extract command / PSU address from msg ID field
#define FP2_MSG_MASK    0xff00ffff
#define FP2_ADDR_MASK   0x00ff0000
#define FP2_STATUS_MASK 0xff00ff00

// commands - TX to power supply
#define FP2_CMD_LOGIN       0x05004800
#define FP2_CMD_SET_VOLTAGE 0x05009c00
#define FP2_CMD_GET_ALARM   0x0500b00c

// responses - RX from power supply
#define FP2_MSG_STATUS    0x05004000
#define FP2_MSG_LOGIN_REQ 0x05004400
#define FP2_MSG_ALARMS    0x0500BFFC

// status codes (last byte of ID in MSG_STATUS)
#define FP2_STATUS_NORMAL  0x04
#define FP2_STATUS_WARNING 0x08
#define FP2_STATUS_ALARM   0x0C
#define FP2_STATUS_WALKIN  0x10

const char *fp2_alarm0_strings[] = {"OVS Lockout", "Primary Module Failure", "Secondary Module Failure", "Mains Voltage High", "Mains Voltage Low", "Temperature High", "Temperature Low", "Current Over Limit"};
const char *fp2_alarm1_strings[] = {"Internal Voltage Fault", "Module Failure", "Secondary Module Failure", "Fan 1 Speed Low", "Fan 2 Speed Low", "Sub-module 1 Failure", "Fan 3 Speed Low", "Internal Voltage Fault"};


/**
 * * Global variables
 * ! Reads and writes to globals are only atomic for 32-bit values !
 */
// esp internal
static float esp_internal_temp;

// fp2 power supply status
static uint8_t  fp2_id;
static uint8_t  fp2_serial[6];
static uint32_t fp2_input_volts;
static float    fp2_output_volts;
static float    fp2_output_amps;
static float    fp2_output_watts;
static uint32_t fp2_inlet_temp;
static uint32_t fp2_outlet_temp;


/**
 * * Static prototypes
 */
// Tasks
static void displayTask(void *pvParameter);
static void lvTickTimer(void *ignore);
static void ledTask(void *ignore);
static void networkTask(void *ignore);
static void timeSyncTask(void *ignore);
static void tempSensorTask(void *ignore);
static void twaiTask(void *ignore);

// Functions
static void lvAppCreate(void);

// Callbacks
static void cb_netConnected(void *pvParameter);
static void cb_netDisconnected(void *pvParameter);
static void cb_timeSyncEvent(struct timeval *tv);


/****************************************************************
 * * app_main function - run on startup
 * TODO: implement button handling and screen switching here maybe?
 * TODO: Set GPIO pull-ups for buttons etc.
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
    xTaskCreate(&displayTask, "display", 1024 * 8, NULL, 10, NULL);

    //* Create TWAI communication task
    xTaskCreate(&twaiTask, "twai", 1024 * 4, NULL, 8, NULL);

    //* Create RGB LED update task
    xTaskCreate(&ledTask, "rgb", 1024 * 2, NULL, 5, NULL);

    //* create wifi initialization task
    xTaskCreate(&networkTask, "network", 1024 * 2, NULL, 5, NULL);

    //* Create SNTP management task
    xTaskCreate(&timeSyncTask, "timeSync", 1024 * 2, NULL, 2, NULL);

    //* Create temp sensor polling task
    xTaskCreate(&tempSensorTask, "tempSensor", 1024 * 2, NULL, 2, NULL);
}


/****************************************************************
 * * lvgl display management
 */
void displayTask(void *pvParameter) {
    ESP_LOGI(DISP_TASK_TAG, "display task begin");

    xDisplaySemaphore = xSemaphoreCreateMutex();

    lv_init();

    /* Initialize SPI or I2C bus used by the drivers */
    lvgl_driver_init();

    lv_color_t *buf1 = heap_caps_malloc(DISP_BUF_SIZE * sizeof(lv_color_t), MALLOC_CAP_DMA);
    assert(buf1 != NULL);
    static lv_color_t *  buf2 = NULL;
    static lv_disp_buf_t disp_buf;
    /* Actual size in pixels, not bytes. */
    uint32_t size_in_px = DISP_BUF_SIZE * 8;

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
        .callback = &lvTickTimer,
        .name     = "periodic_gui"};
    esp_timer_handle_t periodic_timer;
    ESP_ERROR_CHECK(esp_timer_create(&periodic_timer_args, &periodic_timer));
    ESP_ERROR_CHECK(esp_timer_start_periodic(periodic_timer, LV_TICK_PERIOD_MS * 1000));

    // set up initial screen state
    lvAppCreate();

    // set display ready bit
    xEventGroupSetBits(appEventGroup, DISP_RUN_BIT);

    /* Delay 1 tick (assumes FreeRTOS tick is 10ms */
    vTaskDelay(pdMS_TO_TICKS(10));
    static uint32_t lv_task_delay = 0;
    while (1) {
        /* Try to take the semaphore, call lvgl task handler on success */
        if (pdTRUE == xSemaphoreTake(xDisplaySemaphore, portMAX_DELAY)) {
            lv_task_delay = lv_task_handler();
            xSemaphoreGive(xDisplaySemaphore);
        }
        if (lv_task_delay < 10) lv_task_delay = 10;
        vTaskDelay(pdMS_TO_TICKS(lv_task_delay));
    }

    /* A task should NEVER return */
    free(buf1);
    vTaskDelete(NULL);
}

// lvgl tick timer
static void lvTickTimer(void *ignore) {
    lv_tick_inc(LV_TICK_PERIOD_MS);
}


// setup initial display state
static void lvAppCreate(void) {
    ESP_LOGI(APP_CREATE_TAG, "setting up initial display state");

    /* Get the current screen */
    lv_obj_t *scr = lv_disp_get_scr_act(NULL);

    /*Create a Label on the currently active screen*/
    lv_obj_t *label1 = lv_label_create(scr, NULL);

    /*Modify the Label's text*/
    lv_label_set_text(label1, "flatpack2s2\n");

    /* Align the Label to the center
     * NULL means align on parent (which is the screen now)
     * 0, 0 at the end means an x, y offset after alignment*/
    lv_obj_align(label1, NULL, LV_ALIGN_CENTER, 0, 0);

    ESP_LOGI(APP_CREATE_TAG, "complete");
}


/****************************************************************
 * * TWAI init and run task
 * TODO: Literally all of this
 */
void twaiTask(void *ignore) {
    ESP_LOGI(TWAI_TASK_TAG, "initializing TWAI");

    // Initialize config structures and install driver
    static const twai_general_config_t fp2_twai_g_config = TWAI_GENERAL_CONFIG_DEFAULT(CONFIG_TWAI_TX_GPIO, CONFIG_TWAI_RX_GPIO, TWAI_MODE_NORMAL);
    static const twai_timing_config_t  fp2_twai_t_config = TWAI_TIMING_CONFIG_125KBITS();
    static const twai_filter_config_t  fp2_twai_f_config = TWAI_FILTER_CONFIG_ACCEPT_ALL();
    ESP_ERROR_CHECK(twai_driver_install(&fp2_twai_g_config, &fp2_twai_t_config, &fp2_twai_f_config));
    ESP_LOGI(TWAI_TASK_TAG, "TWAI driver configured");

    // Configure TWAI_EN GPIO
    static const gpio_config_t twai_en_conf = {
        .pin_bit_mask = GPIO_TWAI_EN_SEL,
        .mode         = GPIO_MODE_OUTPUT_OD,
        .intr_type    = GPIO_INTR_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .pull_up_en   = GPIO_PULLUP_DISABLE,
    };
    ESP_ERROR_CHECK(gpio_config(&twai_en_conf));

    // Enable transceiver
    gpio_set_level(GPIO_TWAI_EN, 0);
    ESP_LOGI("TWAI bus transceiver enabled, starting driver");
    // give it 10ms to wake up; it only wants 20us, but eh.
    vTaskDelay(pdMS_TO_TICKS(10));

    ESP_ERROR_CHECK(twai_start());
    ESP_LOGI(TWAI_TASK_TAG, "TWAI driver started");
    xEventGroupSetBits(appEventGroup, TWAI_RUN_BIT);

    // wait for a login request and save the PSU ID
    ESP_LOGI(TWAI_TASK_TAG, "Waiting for PSU login request.");
    while (true) {
        twai_message_t rxBuf;
        ESP_ERROR_CHECK(twai_receive(&rxBuf, portMAX_DELAY));
        // check if login message, ignore if not
        if ((rxBuf.identifier & FP2_MSG_MASK) == FP2_MSG_LOGIN_REQ) {
            // extract current PSU ID and serial number
            fp2_id = (rxBuf.identifier & 0x00ff0000);
            for (int i = 0; i < 6; i++) {
                fp2_serial[i] = rxBuf.data[i];
            }

            char snbuf[24];
            snprintf(snbuf, sizeof(snbuf), "%d%d%d%d%d%d", fp2_serial[0], fp2_serial[1], fp2_serial[2], fp2_serial[3], fp2_serial[4], fp2_serial[5]);
            ESP_LOGI(TWAI_TASK_TAG, "Received login request from SN %s with ID %2x", snbuf, (fp2_id >> 16));
            xEventGroupSetBits(appEventGroup, FP2_FOUND_BIT);
            break;
        }
    }

    // send first login
    fp2Login();

    // main task loop, brother
    while (true) {
        twai_message_t rxBuf;
        if (twai_receive(&rxBuf, portMAX_DELAY) == ESP_OK) {
            uint32_t msg_id = (rxBuf.identifier & FP2_MSG_MASK);
            switch (msg_id) {
                case 1:
                    /* code */
                    break;

                default:
                    break;
            }
        }
    }

    vTaskDelete(NULL);
}

void fp2Login() {
    uint8_t txBuf[8] = {0};
    for (int i = 0; i < 6; ++i) {
        txBuf[i] = fp2_serial[i];
    }
    twai_message_t login_msg = {
        .identifier       = (fp2_id | FP2_CMD_LOGIN),
        .extd             = 1,
        .data_length_code = 8,
        .data             = txBuf,
    };
}

void fp2AlertMsgProcess(uint8_t alertBuf) {
    uint8_t alarm_byte_0 = alertBuf[3];
    uint8_t alarm_byte_1 = alertBuf[4];
    switch (alertBuf[1]) {
        case FP2_STATUS_WARNING:
            ESP_LOGW(FP2_ALERT_TAG, "PSU has warnings:");
            for (int i = 0; i < 8; i++) {
                if (alarm_byte_0 & (1 << i)) {
                    ESP_LOGW(FP2_ALERT_TAG, "%s", fp2_alarm0_strings[i]);
                }
                if (alarm_byte_1 & (1 << i)) {
                    ESP_LOGW(FP2_ALERT_TAG, "%s", fp2_alarm1_strings[i]);
                }
            }
            break;
        case FP2_STATUS_ALERT:
            ESP_LOGE(FP2_ALERT_TAG, "PSU has errors:");
            for (int i = 0; i < 8; i++) {
                if (alarm_byte_0 & (1 << i)) {
                    ESP_LOGE(FP2_ALERT_TAG, "%s", fp2_alarm0_strings[i]);
                }
                if (alarm_byte_1 & (1 << i)) {
                    ESP_LOGE(FP2_ALERT_TAG, "%s", fp2_alarm1_strings[i]);
                }
            }
            break;
    }
}


/**
 * * RGB LED control task
 * TODO: Make this receive color set requests instead of just doing a rainbow fade
 */
void ledTask(void *ignore) {
    ESP_LOGI(LED_TASK_TAG, "initializing RGB LED");

    uint32_t red   = 0;
    uint32_t green = 0;
    uint32_t blue  = 0;

    rmt_config_t config = RMT_DEFAULT_CONFIG_TX(CONFIG_FP2S2_RGB_GPIO, RMT_LED_CHANNEL);
    // set counter clock to 40MHz
    config.clk_div = 2;
    // install RMT driver
    ESP_ERROR_CHECK(rmt_config(&config));
    ESP_LOGI(LED_TASK_TAG, "RMT driver configured");
    ESP_ERROR_CHECK(rmt_driver_install(config.channel, 0, 0));
    ESP_LOGI(LED_TASK_TAG, "RMT driver installed");

    // install ws2812 driver
    led_strip_config_t led_config = LED_STRIP_DEFAULT_CONFIG(1, (led_strip_dev_t)config.channel);
    led_strip_t *      rgb_led    = led_strip_new_rmt_ws2812(&led_config);
    if (!rgb_led) {
        ESP_LOGE(LED_TASK_TAG, "initialization failed! giving up...");
        vTaskDelete(NULL);
    }

    // turn off LED
    ESP_ERROR_CHECK(rgb_led->clear(rgb_led, 100));
    ESP_LOGI(LED_TASK_TAG, "initialization complete");

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


/****************************************************************
 * * SNTP time sync setup task
 * TODO: Maybe pick up SNTP from DHCP?
 */
void timeSyncTask(void *ignore) {
    ESP_LOGI(TIMESYNC_TASK_TAG, "sntp task starting");

    //* Set time zone and SNTP operating parameters
    setenv("TZ", POSIX_TZ, 1);
    tzset();

    // wait for wifi connected bit in event group
    ESP_LOGI(TIMESYNC_TASK_TAG, "TZ set, waiting for wifi");
    xEventGroupWaitBits(appEventGroup, WIFI_CONNECTED_BIT, pdFALSE, pdTRUE, portMAX_DELAY);

    // configure ant start SNTP
    ESP_LOGI(TIMESYNC_TASK_TAG, "Starting sntp service");
    sntp_setoperatingmode(SNTP_OPMODE_POLL);
    sntp_setservername(0, CONFIG_SNTP_SERVER);
    sntp_set_time_sync_notification_cb(cb_timeSyncEvent);
    sntp_init();

    ESP_LOGI(TIMESYNC_TASK_TAG, "Waiting for system time to be set...");
    xEventGroupWaitBits(appEventGroup, TIMESYNC_RUN_BIT, pdFALSE, pdTRUE, portMAX_DELAY);

    ESP_LOGI(TIMESYNC_TASK_TAG, "System time set, exiting");
    vTaskDelete(NULL);
}

// sntp event callback
void cb_timeSyncEvent(struct timeval *tv) {
    ESP_LOGI(TIMESYNC_TASK_TAG, "SNTP sync event notification");

    if ((xEventGroupGetBits(appEventGroup) & BIT1) == 0) xEventGroupSetBits(appEventGroup, TIMESYNC_RUN_BIT);

    char      strftime_sntp[64];
    time_t    sntp     = (time_t)tv->tv_sec;
    struct tm sntpinfo = {0};
    localtime_r(&sntp, &sntpinfo);
    strftime(strftime_sntp, sizeof(strftime_sntp), "%c", &sntpinfo);
    ESP_LOGI(TIMESYNC_TASK_TAG, "received date/time: %s", strftime_sntp);
}


/****************************************************************
 * * WiFi Manager Init task; doesn't need to be separate, really, but means it doesn't block main
 * TODO: Maybe pick up SNTP from DHCP?
 */
void networkTask(void *ignore) {
    // start the wifi manager
    wifi_manager_start();
    // register wifi callbacks
    wifi_manager_set_callback(WM_EVENT_STA_GOT_IP, &cb_netConnected);
    wifi_manager_set_callback(WM_EVENT_STA_DISCONNECTED, &cb_netDisconnected);
    // done
    vTaskDelete(NULL);
}

void cb_netConnected(void *pvParameter) {
    ip_event_got_ip_t *param = (ip_event_got_ip_t *)pvParameter;
    xEventGroupSetBits(appEventGroup, WIFI_CONNECTED_BIT);

    /* transform IP to human readable string */
    char str_ip[16];
    esp_ip4addr_ntoa(&param->ip_info.ip, str_ip, IP4ADDR_STRLEN_MAX);

    ESP_LOGI(TAG, "wifi connected, yay! acquired IP address: %s", str_ip);
}

void cb_netDisconnected(void *pvParameter) {
    wifi_event_sta_disconnected_t *param = (wifi_event_sta_disconnected_t *)pvParameter;
    xEventGroupClearBits(appEventGroup, WIFI_CONNECTED_BIT);

    ESP_LOGW(TAG, "wifi disconnected, boo! reason:%d", param->reason);
}


/****************************************************************
 * * ESP Internal Temp Sensor task
 */
void tempSensorTask(void *ignore) {
    static const char *TEMP_TASK_TAG = "tempSensorTask";
    ESP_LOGI(TEMP_TASK_TAG, "temp sensor starting...");

    // configure sensor
    temp_sensor_config_t temp_sensor = TSENS_CONFIG_DEFAULT();
    temp_sensor_get_config(&temp_sensor);
    temp_sensor.dac_offset = TSENS_DAC_DEFAULT; // DEFAULT: range:-10℃ ~  80℃, error < 1℃.
    temp_sensor_set_config(temp_sensor);

    // start sensor
    ESP_ERROR_CHECK(temp_sensor_start());
    ESP_LOGI(TEMP_TASK_TAG, "temp sensor started!")

    // get current temp and store in global
    temp_sensor_read_celsius(&esp_internal_temp);
    xEventGroupSetBits(appEventGroup, TEMP_RUN_BIT);
    ESP_LOGI(TEMP_TASK_TAG, "current chip temp %.3f°C", esp_internal_temp);

    // configure a low-priority timer to update the temp sensor reading
    const int                     temp_poll_period_sec   = TEMP_POLL_PERIOD;
    const esp_timer_create_args_t temp_sensor_timer_args = {
        .callback = cb_tempSensorPoll,
        .name     = "tempSensorPoll"};
    esp_timer_handle_t temp_sensor_poll;
    ESP_ERROR_CHECK(esp_timer_create(&temp_sensor_timer_args, &temp_sensor_poll));

    // wait for one poll period before starting the timer
    vTaskDelay(pdMS_TO_TICKS(temp_poll_period_sec * 1000));
    ESP_ERROR_CHECK(esp_timer_start_periodic(temp_sensor_poll, temp_poll_period_sec * 1000));

    ESP_LOGI(TEMP_TASK_TAG, "timer started, polling temp sensor every %d seconds", temp_poll_period_sec)

    vTaskDelete(NULL);
}

void cb_tempSensorPoll(void) {
    temp_sensor_read_celsius(&esp_internal_temp);
}