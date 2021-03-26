/*
 * Flatpack2s2 ESP32-S2 application
   TODO: This documentation comment block, heh
 */

// cstd11
#include <fcntl.h>
#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/time.h>

// sdkconfig
#include "sdkconfig.h"

// FreeRTOS
#include "freertos/FreeRTOS.h"
#include "freertos/event_groups.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"
#include "freertos/task.h"

// ESP-IDF components
#include <esp_err.h>
#include <esp_event.h>
#include <esp_freertos_hooks.h>
#include <esp_log.h>
#include <esp_netif.h>
#include <esp_ota_ops.h>
#include <esp_sntp.h>
#include <esp_system.h>
#include <esp_wifi.h>

// ESP-IDF drivers
#include "driver/gpio.h"
#include "driver/i2c.h"
#include "driver/rmt.h"
#include "driver/temp_sensor.h"
#include "driver/twai.h"
#include "nvs.h"
#include "nvs_flash.h"
#include "soc/rtc.h"

// * --------------------------- Project components ------------------------- */

// Serial command console component
#include "argtable3/argtable3.h"
#include "cmd_nvs.h"
#include "cmd_system.h"
#include "esp_console.h"
#include "esp_vfs_cdcacm.h"
#include "linenoise/linenoise.h"

// RGB LED strip driver (using RMT peripheral)
#include "led_strip.h"

// lvgl graphics library
#include "lvgl.h"
#include "lvgl_helpers.h"

// Extra project source files (helper functions etc.)
#include "helpers.h"
// fp2 #defines go in here
#include "flatpack2s2.h"


// * -------------------- Definitions and static variables ------------------ */

// TZ string for sntp
#define POSIX_TZ CONFIG_POSIX_TZ

// RGB LED driver config
#define RMT_LED_CHANNEL    RMT_CHANNEL_0
#define LED_UPDATE_RATE_HZ 50

// TWAI transcever (MAX3051ESA+ in my case) configuration
#define TWAI_EN_GPIO        CONFIG_TWAI_EN_GPIO
#define TWAI_EN_GPIO_SEL    (1ULL << TWAI_EN_GPIO)
#define TWAI_EN_ACTIVE      CONFIG_TWAI_EN_ACTIVE_STATE
#define TWAI_RX_TIMEOUT_SEC 30
#define TWAI_RX_LOG_ALL     1

// interval for internal temp sensor measurements
#define ESP_TEMP_POLL_SEC 10

// lvgl task handler interval and tick update interval
#define LV_TICK_PERIOD_MS 5
#define LV_TASK_PERIOD_MS 20

// LVGL indev pin definitions
static const int btn_left  = CONFIG_FP2S2_SW_L_GPIO;
static const int btn_enter = CONFIG_FP2S2_SW_S_GPIO;
static const int btn_right = CONFIG_FP2S2_SW_R_GPIO;
static const int btn_back  = GPIO_NUM_0;

// Log tag strings
static const char *TAG                = "flatpack2s2";
static const char *LVGL_TAG           = "lvgl";
static const char *DISP_TASK_TAG      = "display";
static const char *LED_TASK_TAG       = "led";
static const char *CONSOLE_TASK_TAG   = "usbConsole";
static const char *TWAI_CTRL_TASK_TAG = "twaiCtrl";
static const char *TWAI_TX_TASK_TAG   = "twaiTx";
static const char *TWAI_RX_TASK_TAG   = "twaiRx";
static const char *TWAI_RX_LOG_TAG    = "twaiMsgLog";
static const char *FP2_ALERT_TAG      = "fp2AlertMessage";
static const char *FP2_LOGIN_TASK_TAG = "fp2Login";

// * Flatpack2-related Constants
// ! see also: src/flatpack2.h
static const uint32_t fp2_vout_min = CONFIG_FP2_VOUT_MIN;
static const uint32_t fp2_vout_max = CONFIG_FP2_VOUT_MAX;
static const uint32_t fp2_iout_max = CONFIG_FP2_IOUT_MAX;

const char *fp2_alerts0_str[] = {"OVS Lockout",       "Primary Module Failure", "Secondary Module Failure", "Mains Voltage High",
                                 "Mains Voltage Low", "Temperature High",       "Temperature Low",          "Current Over Limit"};
const char *fp2_alerts1_str[] = {"Internal Voltage Fault", "Module Failure",        "Secondary Module Failure",
                                 "Fan 1 Speed Low",        "Fan 2 Speed Low",       "Sub-module 1 Failure",
                                 "Fan 3 Speed Low",        "Internal Voltage Fault"};


// * --------------- Eventgroup, queue, and semaphore handles --------------- */

// General shared event group
static EventGroupHandle_t appEventGroup;
// bit assignments
static const int DISP_RUN_BIT       = BIT0;
static const int TWAI_RUN_BIT       = BIT1;
static const int ESP_TEMP_RUN_BIT   = BIT2;
static const int LED_RUN_BIT        = BIT3;
static const int CONSOLE_RUN_BIT    = BIT4;
static const int WIFI_CONNECTED_BIT = BIT20;
static const int TIMESYNC_RUN_BIT   = BIT21;
static const int TIME_VALID_BIT     = BIT22;

// flatpack2 event group
static EventGroupHandle_t fp2EventGroup;
// bit assignments
static const int FP2_FOUND_BIT         = BIT0;
static const int FP2_LOGIN_BIT         = BIT1;
static const int FP2_STATUS_CV_BIT     = BIT2;
static const int FP2_STATUS_CC_BIT     = BIT3;
static const int FP2_STATUS_WALKIN_BIT = BIT4;
static const int FP2_LOGIN_REQUEST_BIT = BIT5;
//
SemaphoreHandle_t fp2LoginReqSem;
SemaphoreHandle_t xTwaiMutex;

// lvgl driver mutex
SemaphoreHandle_t xLvglMutex; // lvgl2 mutex


// * --------------------------- Global Variables --------------------------- */
// ! Reads and writes to globals are only atomic for 32-bit values !

// esp internal temperature
static float esp_internal_temp;

// fp2 power supply status
static flatpack2_t fp2;
static float       fp2_in_volts;
static float       fp2_out_volts;
static float       fp2_out_amps;
static float       fp2_out_watts;
static uint32_t    fp2_temp_intake;
static uint32_t    fp2_temp_exhaust;
static uint8_t     fp2_status;


// * ---------------------- Static function prototypes ---------------------- */

//  Persistent tasks
static void displayTask(void *pvParameter);
static void lvTickTimer(void *ignore);
static void ledTask(void *ignore);
static void tempSensorTask(void *ignore);
static void twaiCtrlTask(void *ignore);
static void consoleTask(void *ignore);

// Non-persistent tasks
static void fp2LoginTask(void *arg);
static void twaiRxTask(void *arg);
static void twaiTxTask(void *arg);

// Functions
static void lvAppCreate(void);
static void initialize_nvs(void);
static void initialize_console(void);
static void logTwaiRxMsg(twai_message_t *rxMsg);
static void saveFp2Details(twai_message_t *rxMsg);

// Callbacks
static void cb_netConnected(void *ignore);
static void cb_netDisconnected(void *ignore);
static void cb_timeSyncEvent(struct timeval *tv);
static void cb_lvglLog(lv_log_level_t level, const char *file, uint32_t line, const char *fn_name, const char *dsc);


// * -------------------------- Tasks & Functions --------------------------- */

/****************************************************************
 * * app_main function - run on startup
 * TODO: implement button handling and screen switching here maybe?
 * TODO: Set GPIO pull-ups for buttons etc.
 */
void app_main(void) {
    ESP_LOGW(TAG, "flatpack2s2 startup!");

    //* initialize the main app event group
    appEventGroup = xEventGroupCreate();


    //* start the command-line console task and wait for init
    xTaskCreate(&consoleTask, "console", 1024 * 4, NULL, 5, NULL);
    xEventGroupWaitBits(appEventGroup, CONSOLE_RUN_BIT, pdFALSE, pdTRUE, portMAX_DELAY);

    //* Create OLED display task
    xLvglMutex = xSemaphoreCreateMutex();
    assert(xLvglMutex != NULL);
    // xTaskCreate(&displayTask, "display", 1024 * 16, NULL, 10, NULL);

    //* Create TWAI setup task
    fp2EventGroup = xEventGroupCreate();
    xTwaiMutex    = xSemaphoreCreateMutex();
    assert(xTwaiMutex != NULL);
    xTaskCreate(&twaiCtrlTask, "twai", 1024 * 4, NULL, 6, NULL);

    //* Create RGB LED update task
    xTaskCreate(&ledTask, "rgb", 1024 * 2, NULL, 3, NULL);

    //* Create temp sensor polling task
    xTaskCreate(&tempSensorTask, "tempSensor", 1024 * 2, NULL, 2, NULL);
}


/****************************************************************
 * * lvgl display management
 */
static void displayTask(void *pvParameter) {
    ESP_LOGI(DISP_TASK_TAG, "display task begin");

    lv_init();
    lvgl_driver_init();

    // configure for B&W OLED
    lv_color_t *buf1 = heap_caps_malloc(DISP_BUF_SIZE * sizeof(lv_color_t), MALLOC_CAP_DMA);
    assert(buf1 != NULL);
    static lv_color_t *  buf2 = NULL;
    static lv_disp_buf_t disp_buf;
    /* Actual size in pixels, not bytes. */
    uint32_t size_in_px = DISP_BUF_SIZE * 8;

    /**
     * Initialize the working buffer depending on the selected display.
     * NOTE: buf2 == NULL when using monochrome displays.
     * NOTE: When using a monochrome display we need to register two extra callbacks;
     *        - rounder_cb
     *        - set_px_cb
     */
    lv_disp_buf_init(&disp_buf, buf1, buf2, size_in_px);
    lv_disp_drv_t disp_drv;
    lv_disp_drv_init(&disp_drv);
    disp_drv.flush_cb   = disp_driver_flush;
    disp_drv.rounder_cb = disp_driver_rounder;
    disp_drv.set_px_cb  = disp_driver_set_px;
    disp_drv.buffer     = &disp_buf;
    lv_disp_drv_register(&disp_drv);
    lv_log_register_print_cb(&cb_lvglLog);

    /* Create and start a periodic timer interrupt to call lv_tick_inc from lvTickTimer() */
    const esp_timer_create_args_t lvgl_timer_args = {
        .callback = &lvTickTimer,
        .name     = "lvglTick",
        //         .dispatch_method       = ESP_TIMER_ISR, // doesn't work on IDF 4.3 :(
        .skip_unhandled_events = 1,
    };
    esp_timer_handle_t lvgl_timer;
    ESP_ERROR_CHECK(esp_timer_create(&lvgl_timer_args, &lvgl_timer));
    ESP_ERROR_CHECK(esp_timer_start_periodic(lvgl_timer, LV_TICK_PERIOD_MS * 1000));

    // set up initial screen state
    lvAppCreate();

    // set display ready bit
    xEventGroupSetBits(appEventGroup, DISP_RUN_BIT);

    // initialize task handler delay loop
    TickType_t       xLvglWakeTime;
    const TickType_t xLvglTaskInterval = pdMS_TO_TICKS(LV_TASK_PERIOD_MS);

    // run task handler delay loop forever and ever
    xLvglWakeTime = xTaskGetTickCount();
    while (true) {
        // Try to take the semaphore, call lvgl task handler on success
        if (pdTRUE == xSemaphoreTake(xLvglMutex, portMAX_DELAY)) {
            lv_task_handler();
            xSemaphoreGive(xLvglMutex);
        }
        // wait for next interval
        vTaskDelayUntil(&xLvglWakeTime, xLvglTaskInterval);
    }

    /* A task should NEVER return */
    free(buf1);
    vTaskDelete(NULL);
}

// lvgl tick timer callback
static void lvTickTimer(void *ignore) {
    lv_tick_inc(LV_TICK_PERIOD_MS);
}

// lvgl log callback
void cb_lvglLog(lv_log_level_t level, const char *file, uint32_t line, const char *fn_name, const char *dsc) {
    // use ESP_LOGx macros
    switch (level) {
        case LV_LOG_LEVEL_INFO: ESP_LOGI(LVGL_TAG, "%s: %s at %s:%d", fn_name, dsc, file, line); break;
        case LV_LOG_LEVEL_WARN: ESP_LOGW(LVGL_TAG, "%s: %s at %s:%d", fn_name, dsc, file, line); break;
        case LV_LOG_LEVEL_ERROR: ESP_LOGE(LVGL_TAG, "%s: %s at %s:%d", fn_name, dsc, file, line); break;
        case LV_LOG_LEVEL_TRACE: ESP_LOGV(LVGL_TAG, "%s: %s at %s:%d", fn_name, dsc, file, line); break;
        default: ESP_LOGV(LVGL_TAG, "%s: %s at %s:%d", fn_name, dsc, file, line); break;
    }
}

// setup initial display state
static void lvAppCreate(void) {
    ESP_LOGI(DISP_TASK_TAG, "setting up initial display state");

    // get app data
    const esp_app_desc_t *app_info = esp_ota_get_app_description();

    /* Get the current screen */
    lv_obj_t *scr_def        = lv_disp_get_scr_act(NULL);
    lv_obj_t *scr_fp2_status = lv_obj_create(NULL, NULL);

    /*Create a Label on the currently active screen*/
    lv_obj_t *app_name = lv_label_create(scr_def, NULL);
    lv_label_set_text(app_name, app_info->project_name);
    lv_obj_align(app_name, NULL, LV_ALIGN_IN_BOTTOM_MID, 0, 0);

    /* create spinny loading icon thing */
    lv_obj_t *load_spinner = lv_spinner_create(scr_def, NULL);
    lv_obj_set_size(load_spinner, 90, 90);
    lv_obj_align(load_spinner, NULL, LV_ALIGN_CENTER, 0, -5);
    lv_spinner_set_type(load_spinner, LV_SPINNER_TYPE_CONSTANT_ARC);

    // lv_obj_align(load_spinner, NULL, LV_ALIGN_CENTER, 0, -8);
    lv_obj_set_style_local_line_opa(load_spinner, LV_SPINNER_PART_BG, LV_STATE_DEFAULT, 0);
    lv_obj_set_style_local_bg_opa(load_spinner, LV_SPINNER_PART_BG, LV_STATE_DEFAULT, 0);
    lv_obj_set_style_local_border_opa(load_spinner, LV_SPINNER_PART_BG, LV_STATE_DEFAULT, 0);
}


/****************************************************************
 * * TWAI control task
 * TODO: most of this if we are honest
 */
void twaiCtrlTask(void *ignore) {
    ESP_LOGI(TWAI_CTRL_TASK_TAG, "TWAI initialization");

    fp2LoginReqSem = xSemaphoreCreateBinary();

    // TWAI_EN transceiver enable GPIO config
    static const gpio_config_t twai_en_conf = {
        .pin_bit_mask = TWAI_EN_GPIO_SEL,
        .mode         = GPIO_MODE_OUTPUT,
        .intr_type    = GPIO_INTR_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .pull_up_en   = GPIO_PULLUP_ENABLE,
    };

    // TWAI controller configuration; 125kbps, all msgIDs are 0x05xxxxxx
    static const twai_general_config_t fp2_twai_g_config =
        TWAI_GENERAL_CONFIG_DEFAULT(CONFIG_TWAI_TX_GPIO, CONFIG_TWAI_RX_GPIO, TWAI_MODE_NORMAL);
    static const twai_timing_config_t fp2_twai_t_config = TWAI_TIMING_CONFIG_125KBITS();
    static const twai_filter_config_t fp2_twai_f_config = {
        .acceptance_code = 0x05000000,
        .acceptance_mask = 0x00FFFFFF,
        .single_filter   = true,
    };

    // Configure transceiver enable pin and disable transceiver
    ESP_ERROR_CHECK(gpio_config(&twai_en_conf));
    gpio_set_level(TWAI_EN_GPIO, !TWAI_EN_ACTIVE);
    vTaskDelay(pdMS_TO_TICKS(2));

    // Install TWAI driver
    ESP_ERROR_CHECK_WITHOUT_ABORT(twai_driver_install(&fp2_twai_g_config, &fp2_twai_t_config, &fp2_twai_f_config));
    ESP_LOGI(TWAI_CTRL_TASK_TAG, "TWAI driver and enable pin configured");

    // Enable transceiver and wait for wakeup; 2ms should be more than enough for any transceiver
    gpio_set_level(TWAI_EN_GPIO, TWAI_EN_ACTIVE);
    vTaskDelay(pdMS_TO_TICKS(2));
    ESP_LOGI(TWAI_CTRL_TASK_TAG, "Transceiver enabled");

    // Start driver
    esp_err_t err = twai_start();
    if (err != ESP_OK) {
        ESP_LOGE(TWAI_CTRL_TASK_TAG, "Driver failed to start! Terminating...");
        vTaskDelete(NULL);
    };
    ESP_LOGI(TWAI_CTRL_TASK_TAG, "Driver started");

    // reconfigure TWAI alerts
    twai_reconfigure_alerts(TWAI_ALERT_ALL | TWAI_ALERT_AND_LOG, NULL);
    ESP_LOGI(TWAI_CTRL_TASK_TAG, "Alert configuration set");

    xEventGroupSetBits(appEventGroup, TWAI_RUN_BIT);
    ESP_LOGI(TWAI_CTRL_TASK_TAG, "Initialization complete");

    // create RX task
    TaskHandle_t rxTaskHandle;
    xTaskCreate(&twaiRxTask, "twaiRxTask", 1024 * 4, NULL, 4, &rxTaskHandle);

    // create TX task
    // TaskHandle_t txTaskHandle;
    // txTaskHandle = xTaskCreate(&twaiTxTask, "twaiTxTask", 1024 * 4, &fp2, 4, NULL);

    // handle alerts
    // TODO: handle alerts
    while (true) {
        vTaskDelay(portMAX_DELAY);
    }

    // should never execute
    xEventGroupClearBits(appEventGroup, TWAI_RUN_BIT);
    vTaskDelete(NULL);
}

void twaiTxTask(void *arg) {
    // get passed parameter
    flatpack2_t *fp2 = (flatpack2_t *)arg;

    // should never execute
    vTaskDelete(NULL);
}

void twaiRxTask(void *arg) {
    // get messages and process the heck out of them
    while (true) {
        twai_message_t rxMsg;
        esp_err_t      rxErr = twai_receive(&rxMsg, pdMS_TO_TICKS(TWAI_RX_TIMEOUT_SEC * 1000));
        if (rxErr == ESP_OK) {
            bool     msgProcessed         = false;
            uint32_t rx_identifier_masked = rxMsg.identifier & FP2_MSG_MASK;

            // is this an FP2_MSG_LOGIN_REQ ?
            if (rxMsg.identifier == (FP2_MSG_LOGIN_REQ | (rxMsg.data[5] << 8) | rxMsg.data[6])) {
                // it is! update saved PSU details
                saveFp2Details(&rxMsg);
                // give login task permission to #sendit
                xSemaphoreGive(fp2LoginReqSem);
                // restart loop
                continue;
            }

            // is this an FP2_MSG_HELLO?
            else if ((rxMsg.identifier & FP2_MSG_MASK) == FP2_MSG_HELLO) {
                // it is! save the updated details
                saveFp2Details(&rxMsg);
                // give login task permission to #sendit
                xSemaphoreGive(fp2LoginReqSem);
            }

            // log unknown message types if enabled
            if (TWAI_RX_LOG_ALL || !msgProcessed) {
                logTwaiRxMsg(&rxMsg);
            }
        } else if (rxErr != ESP_ERR_TIMEOUT) {
            const char *rx_err_string = esp_err_to_name(rxErr);
            ESP_LOGW(TWAI_RX_TASK_TAG, "rxMsg error! %s", rx_err_string);
        }
    }

    // tasks should never return or exit; we should never get here, but if we do, ask FreeRTOS to terminate this task
    vTaskDelete(NULL);
}

// function to dump received TWAI messages to console
static void logTwaiRxMsg(twai_message_t *rxMsg) {
    char logBuf[80];
    snprintf(logBuf, sizeof(logBuf), "rxMsg ID 0x%.8x: 0x%.2x 0x%.2x 0x%.2x 0x%.2x 0x%.2x 0x%.2x 0x%.2x 0x%.2x", rxMsg->identifier,
             rxMsg->data[0], rxMsg->data[1], rxMsg->data[2], rxMsg->data[3], rxMsg->data[4], rxMsg->data[5], rxMsg->data[6],
             rxMsg->data[7]);
    ESP_LOGI(TWAI_RX_LOG_TAG, "%s", logBuf);
}

// fill out the login information in the flatpack2_t structure
static void saveFp2Details(twai_message_t *rxMsg) {
    // only update if we don't know this PSU SN already
    if (!memcmp(&fp2.serial, &rxMsg->data, 6)) {
        // extract current PSU ID and serial number
        fp2.id = (uint8_t)(rxMsg->identifier >> 16 & 0xff);
        memcpy(&fp2.serial, &rxMsg->data, sizeof(fp2.serial));

        // assemble login message ID; 0x050048xx, where xx = (desired ID) * 4
        // shifting received ID 18 bits left puts it in the right spot and multiplies it by 4
        fp2.login_id = (uint32_t)((fp2.id << 2) | FP2_CMD_LOGIN);

        // print device found message
        char logBuf[80];
        snprintf(logBuf, sizeof(logBuf), "found flatpack2! S/N %.2x%.2x%.2x%.2x%.2x%.2x, ID 0x%02x, login_id 0x%08x", fp2.serial[0],
                 fp2.serial[1], fp2.serial[2], fp2.serial[3], fp2.serial[4], fp2.serial[5], fp2.id, fp2.login_id);
        ESP_LOGW(TWAI_CTRL_TASK_TAG, "%s", logBuf); // warning log level so it stands out
        xEventGroupSetBits(appEventGroup, FP2_FOUND_BIT);
    }
}

// convert FP2 alert bits to strings
static void processFp2Alerts(uint8_t alertBuf[]) {
    uint8_t alert_byte_low  = alertBuf[3];
    uint8_t alert_byte_high = alertBuf[4];
    switch (alertBuf[1]) {
        case FP2_STATUS_WARN:
            ESP_LOGW(FP2_ALERT_TAG, "PSU has warnings:");
            for (int i = 0; i < 8; i++) {
                if (alert_byte_low & (0x1 << i)) {
                    ESP_LOGW(FP2_ALERT_TAG, "%s", fp2_alerts0_str[i]);
                }
                if (alert_byte_high & (0x1 << i)) {
                    ESP_LOGW(FP2_ALERT_TAG, "%s", fp2_alerts1_str[i]);
                }
            }
            break;
        case FP2_STATUS_ALARM:
            ESP_LOGE(FP2_ALERT_TAG, "PSU has errors:");
            for (int i = 0; i < 8; i++) {
                if (alert_byte_low & (1 << i)) {
                    ESP_LOGE(FP2_ALERT_TAG, "%s", fp2_alerts0_str[i]);
                }
                if (alert_byte_high & (1 << i)) {
                    ESP_LOGE(FP2_ALERT_TAG, "%s", fp2_alerts1_str[i]);
                }
            }
            break;
    }
}


/**
 * * flatpack2 login loop task
 * TODO: make sure this like, works
 */
void fp2LoginTask(void *arg) {
    // get passed parameter
    flatpack2_t *fp2 = (flatpack2_t *)arg;

    // initialize login msg
    twai_message_t login_msg = {
        .extd             = 1,
        .rtr              = 0,
        .self             = 0,
        .dlc_non_comp     = 0,
        .data_length_code = 8,
        .data             = {0},
    };

    // wait for login request semaphore and send message
    while (true) {
        if (pdTRUE == xSemaphoreTake(fp2LoginReqSem, portMAX_DELAY)) {
            // check if the PSU has changed and update login message as required
            if (!memcmp(&login_msg.data, &fp2->serial, 6)) {
                memcpy(&login_msg.data, &fp2->serial, sizeof(fp2->serial));
                login_msg.identifier = fp2->login_id;
            }

            // send the message and go back to waiting
            esp_err_t err;
            err = twai_transmit(&login_msg, pdMS_TO_TICKS(TWAI_RX_TIMEOUT_SEC * 1000));
            if (err == ESP_OK) {
                ESP_LOGI(FP2_LOGIN_TASK_TAG, "[TX][0x%.8x] sent login to PSU 0x%.2x", login_msg.identifier, fp2->id);
            } else {
                const char *tx_err_str = esp_err_to_name(err);
                ESP_LOGE(FP2_LOGIN_TASK_TAG, "failed to send login! %s", tx_err_str);
            }
        }
    }

    // should never execute
    vTaskDelete(NULL);
}

/****************************************************************
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

    // initialize task handler delay loop
    TickType_t       xLastWakeTime = xTaskGetTickCount();
    const TickType_t xTaskInterval = pdMS_TO_TICKS(1000 / LED_UPDATE_RATE_HZ);

    // this is a placeholder that just does a rainbow cycle until I have this actually set up
    while (true) {
        for (int i = 0; i < 360; i++) {
            // build RGB value
            led_hsv2rgb(i, 100, 30, &red, &green, &blue);
            rgb_led->set_pixel(rgb_led, 0, red, green, blue);
            // Flush RGB value to LED
            rgb_led->refresh(rgb_led, 100);
            // wait for next interval
            vTaskDelayUntil(&xLastWakeTime, xTaskInterval);
        }
    }
    vTaskDelete(NULL);
}


/****************************************************************
 * * ESP Internal Temp Sensor task
 */
void tempSensorTask(void *ignore) {
    static const char *TEMP_TASK_TAG = "tempSensorTask";
    ESP_LOGI(TEMP_TASK_TAG, "temp sensor starting...");

    // configure sensor
    temp_sensor_config_t temp_sensor = TSENS_CONFIG_DEFAULT();
    temp_sensor_set_config(temp_sensor);

    // start sensor
    ESP_ERROR_CHECK(temp_sensor_start());
    ESP_LOGI(TEMP_TASK_TAG, "temp sensor started!");

    // get current temp and store in global
    temp_sensor_read_celsius(&esp_internal_temp);
    xEventGroupSetBits(appEventGroup, ESP_TEMP_RUN_BIT);
    ESP_LOGI(TEMP_TASK_TAG, "current chip temp %.3f°C", esp_internal_temp);

    // initialize task handler delay loop
    TickType_t       xLastWakeTime;
    const TickType_t xTaskInterval = pdMS_TO_TICKS(ESP_TEMP_POLL_SEC * 1000);

    // update the temp sensor reading roughly every ESP_TEMP_POLL_SEC seconds
    xLastWakeTime = xTaskGetTickCount();
    while (true) {
        vTaskDelayUntil(&xLastWakeTime, xTaskInterval);
        temp_sensor_read_celsius(&esp_internal_temp);
        // ESP_LOGI(TEMP_TASK_TAG, "current chip temp %.3f°C", esp_internal_temp);
    }

    xEventGroupClearBits(appEventGroup, ESP_TEMP_RUN_BIT);
    vTaskDelete(NULL);
}


/****************************************************************
 * * Command line console task
 */
void consoleTask(void *ignore) {
    //* Initialize console
    initialize_console();
    /* Register commands */
    esp_console_register_help_command();
    register_system_common();
    register_system_sleep();
    register_nvs();

    /**
     * Prompt to be printed before each line.
     * This can be customized, made dynamic, etc.
     */
    const char *prompt = LOG_COLOR(LOG_COLOR_PURPLE) "flatpack2s2> " LOG_RESET_COLOR;

    printf("\n"
           "Welcome to flatpack2s2.\n"
           "Type 'help' to get the list of commands.\n"
           "Use UP/DOWN arrows to navigate through command history.\n"
           "Press TAB when typing command name to auto-complete.\n");

    // Figure out if the terminal supports escape sequences
    int probe_status = linenoiseProbe();
    if (probe_status) { // zero indicates success
        printf("\n"
               "linenoise probe failed, enabling dumb mode...\n");
        linenoiseSetDumbMode(1);
    }

    // tell everybody we're on our way
    xEventGroupSetBits(appEventGroup, CONSOLE_RUN_BIT);

    /* Main loop */
    while (true) {
        /* Get a line using linenoise. Line is returned when ENTER is pressed. */
        char *line = linenoise(prompt);
        if (line == NULL) { /* Ignore empty lines */
            continue;
        }
        /* Add the command to the history */
        linenoiseHistoryAdd(line);

        /* Try to run the command */
        int       ret;
        esp_err_t err = esp_console_run(line, &ret);
        if (err == ESP_ERR_NOT_FOUND) {
            printf("Unrecognized command\n");
        } else if (err == ESP_ERR_INVALID_ARG) {
            // command was empty
        } else if (err == ESP_OK && ret != ESP_OK) {
            printf("Command returned non-zero error code: 0x%x (%s)\n", ret, esp_err_to_name(ret));
        } else if (err != ESP_OK) {
            printf("Internal error: %s\n", esp_err_to_name(err));
        }
        /* linenoise allocates line buffer on the heap, so need to free it */
        linenoiseFree(line);
    }
}

static void initialize_nvs(void) {
    esp_err_t err = nvs_flash_init();
    if (err == ESP_ERR_NVS_NO_FREE_PAGES || err == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        err = nvs_flash_init();
    }
    ESP_ERROR_CHECK(err);
}

static void initialize_console(void) {
    /* Disable buffering on stdin and stdout */
    setvbuf(stdin, NULL, _IONBF, 0);

    /* Minicom, screen, idf_monitor send CR when ENTER key is pressed */
    esp_vfs_dev_cdcacm_set_rx_line_endings(ESP_LINE_ENDINGS_CR);
    /* Move the caret to the beginning of the next line on '\n' */
    esp_vfs_dev_cdcacm_set_tx_line_endings(ESP_LINE_ENDINGS_CRLF);

    /* Enable non-blocking mode on stdin and stdout */
    fcntl(fileno(stdout), F_SETFL, 0);
    fcntl(fileno(stdin), F_SETFL, 0);

    /* Initialize the console */
    esp_console_config_t console_config = {.max_cmdline_args = 8, .max_cmdline_length = 256, .hint_color = atoi(LOG_COLOR_CYAN)};
    ESP_ERROR_CHECK(esp_console_init(&console_config));

    /* Configure linenoise line completion library. */
    /* Enable multiline editing. If not set, long commands will scroll within single line. */
    linenoiseSetMultiLine(1);

    /* Tell linenoise where to get command completions and hints */
    linenoiseSetCompletionCallback(&esp_console_get_completion);
    linenoiseSetHintsCallback((linenoiseHintsCallback *)&esp_console_get_hint);

    /* Set command history size */
    linenoiseHistorySetMaxLen(10);
}