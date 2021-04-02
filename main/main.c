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
// gpiodev and theme
#include "lvgl_gpiodev.h"
#include "lv_theme_fp2.h"

// Extra project source files (helper functions etc.)
#include "helpers.h"
// flatpack2 helper functions and definitions
#include "flatpack2.h"


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
#define TWAI_TX_TIMEOUT_SEC 1
#define TWAI_RX_TIMEOUT_SEC 10
#define TWAI_TX_RETRIES     3
#define TWAI_MSG_LOG_ALL    0

// interval for internal temp sensor measurements
#define ESP_TEMP_POLL_SEC 10

// lvgl task handler interval and tick update interval
#define LV_TICK_PERIOD_MS 5
#define LV_TASK_PERIOD_MS 20

// LVGL indev pin definitions
static const int btn_left  = CONFIG_FP2S2_SW_L_GPIO;
static const int btn_enter = CONFIG_FP2S2_SW_S_GPIO;
static const int btn_right = CONFIG_FP2S2_SW_R_GPIO;
static const int btn_back  = CONFIG_FP2S2_SW_B_GPIO;

// Log tag strings
static const char *TAG                = "flatpack2s2";
static const char *LVGL_TAG           = "lvgl";
static const char *DISP_TASK_TAG      = "display";
static const char *LED_TASK_TAG       = "led";
static const char *CONSOLE_TASK_TAG   = "console";
static const char *TEMP_TASK_TAG      = "espTemp";
static const char *TWAI_CTRL_TASK_TAG = "twai.Ctrl";
static const char *TWAI_TX_TASK_TAG   = "twai.Tx";
static const char *TWAI_RX_TASK_TAG   = "twai.Rx";
static const char *TWAI_MSG_LOG_TAG   = "twai.Msg";


// * Flatpack2-related Constants
// ! see also: src/flatpack2.h
static const uint32_t fp2_vout_min = CONFIG_FP2_VOUT_MIN;
static const uint32_t fp2_vout_max = CONFIG_FP2_VOUT_MAX;
static const uint32_t fp2_iout_max = CONFIG_FP2_IOUT_MAX;


// * --------------- Eventgroup, queue, and semaphore handles --------------- */

// General shared event group
static EventGroupHandle_t appEventGroup;
// bit assignments
static const int DISP_RUN_BIT       = BIT0;
static const int TWAI_CTRL_RUN_BIT  = BIT1;
static const int TWAI_RX_RUN_BIT    = BIT2;
static const int TWAI_TX_RUN_BIT    = BIT3;
static const int WIFI_CONNECTED_BIT = BIT10;
static const int TIMESYNC_RUN_BIT   = BIT11;
static const int TIME_VALID_BIT     = BIT12;
static const int ESP_TEMP_RUN_BIT   = BIT13;
static const int LED_RUN_BIT        = BIT14;
static const int CONSOLE_RUN_BIT    = BIT15;


// TWAI transmit task queue
static QueueHandle_t xTwaiTxQueue;
static QueueHandle_t xLedQueue;

// lvgl driver mutex
static SemaphoreHandle_t xLvglMutex; // lvgl2 mutex


// * --------------------------- Global Variables --------------------------- */
// ! Reads and writes to globals are only atomic for 32-bit values !

// esp internal temperature
static float esp_internal_temp;

// fp2 object
static flatpack2_t fp2;

// lvgl objects
static lv_indev_t *gpio_indev;
static lv_obj_t *  scr_def;
static lv_obj_t *  scr_status;
static lv_obj_t *  vout_gauge;


// setpoints
static uint32_t set_voltage; // desired setpoint voltage
static uint32_t max_voltage; // Over-Voltage Protection voltage
static uint32_t max_current; // Maximum current limit


// * ---------------------- Static function prototypes ---------------------- */

//  Persistent tasks
static void displayTask(void *ignore);
static void lvTickTimer(void *ignore);
static void twaiCtrlTask(void *ignore);
static void consoleTask(void *ignore);
static void ledTask(void *ignore);
static void tempSensorTask(void *ignore);

// Non-persistent tasks
static void twaiRxTask(void *ignore);
static void twaiTxTask(void *ignore);

// Functions
static void lvAppCreate(void);
static void initialiseConsole(void);
static void lv_fp2_val_update(lv_task_t *task);

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

    //* logging config
    esp_log_level_set("*", ESP_LOG_INFO);
    if (TWAI_MSG_LOG_ALL) esp_log_level_set(TWAI_MSG_LOG_TAG, ESP_LOG_DEBUG);

    //* initialise the main app event group
    appEventGroup = xEventGroupCreate();

    //* start the command-line console task and wait for init
    xTaskCreate(&consoleTask, "consoleTask", 1024 * 8, NULL, 6, NULL);
    xEventGroupWaitBits(appEventGroup, CONSOLE_RUN_BIT, pdFALSE, pdTRUE, portMAX_DELAY);

    //* Create OLED display task
    xLvglMutex = xSemaphoreCreateMutex();
    assert(xLvglMutex != NULL);
    xTaskCreate(&displayTask, "display", 1024 * 16, NULL, 10, NULL);

    //* Create TWAI setup task
    xTaskCreate(&twaiCtrlTask, "twaiCtrlTask", 1024 * 8, NULL, 6, NULL);

    //* Create RGB LED update task
    xLedQueue = xQueueCreate(10, sizeof(led_hsv_t));
    assert(xLedQueue != NULL);
    xTaskCreate(&ledTask, "ledTask", 1024 * 4, NULL, 3, NULL);

    //* Create temp sensor polling task
    xTaskCreate(&tempSensorTask, "tempSensor", 1024 * 4, NULL, tskIDLE_PRIORITY, NULL);
}


/****************************************************************
 * * lvgl display management
 */
static void displayTask(void *ignore) {
    ESP_LOGI(DISP_TASK_TAG, "display task begin");

    lv_init();
    lvgl_driver_init();
    lvgl_gpiodev_init();

    /**
     * configure for B&W OLED
     * NOTE: buf2 == NULL when using monochrome displays.
     * NOTE: When using a monochrome display we need to register two extra callbacks;
     *        - rounder_cb
     *        - set_px_cb
     */
    // create display buffer
    lv_color_t *buf1 = heap_caps_malloc(DISP_BUF_SIZE * sizeof(lv_color_t), MALLOC_CAP_DMA);
    assert(buf1 != NULL);
    static lv_color_t *  buf2 = NULL;
    static lv_disp_buf_t disp_buf;
    /* Actual size in pixels, not bytes. */
    uint32_t size_in_px = DISP_BUF_SIZE * 8;

    // initialize display driver
    lv_disp_buf_init(&disp_buf, buf1, buf2, size_in_px);
    lv_disp_drv_t disp_drv;
    lv_disp_drv_init(&disp_drv);
    disp_drv.flush_cb   = disp_driver_flush;
    disp_drv.rounder_cb = disp_driver_rounder;
    disp_drv.set_px_cb  = disp_driver_set_px;
    disp_drv.buffer     = &disp_buf;
    lv_disp_drv_register(&disp_drv);
    lv_log_register_print_cb(&cb_lvglLog);

    // initialize GPIO indev_drv
    lv_indev_drv_t lv_gpiodev;
    lv_indev_drv_init(&lv_gpiodev);
    lv_gpiodev.type    = LV_INDEV_TYPE_ENCODER;
    lv_gpiodev.read_cb = lvgl_gpiodev_read;
    gpio_indev         = lv_indev_drv_register(&lv_gpiodev);

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

    // set up LVGL app
    lvAppCreate();

    // set display ready bit
    xEventGroupSetBits(appEventGroup, DISP_RUN_BIT);

    // run lv_task_handler loop
    uint32_t lv_delay;
    while (true) {
        lv_delay = 10;
        // Try to take the semaphore, call lvgl task handler on success
        if (pdTRUE == xSemaphoreTake(xLvglMutex, pdMS_TO_TICKS(lv_delay) / 2)) {
            lv_delay = lv_task_handler();
            xSemaphoreGive(xLvglMutex);
        }
        // wait for next interval
        vTaskDelay(pdMS_TO_TICKS(lv_delay));
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

// setup lvgl app screens and transitions
static void lvAppCreate(void) {
    ESP_LOGI(DISP_TASK_TAG, "setting up initial display state");

    //lv_theme_t * fp2_theme = lv_theme_fp2_init();

    // get app data
    const esp_app_desc_t *app_info = esp_ota_get_app_description();

    // Get current/default screen
    scr_def = lv_disp_get_scr_act(NULL);

    // Create project name label
    lv_obj_t *app_name = lv_label_create(scr_def, NULL);
    lv_label_set_text(app_name, app_info->project_name);
    lv_obj_align(app_name, NULL, LV_ALIGN_IN_BOTTOM_MID, 0, 0);

    // create spinny loading icon
    lv_obj_t *load_spinner = lv_spinner_create(scr_def, NULL);
    lv_obj_set_size(load_spinner, 90, 90);
    lv_obj_align(load_spinner, NULL, LV_ALIGN_CENTER, 0, -5);
    lv_spinner_set_type(load_spinner, LV_SPINNER_TYPE_CONSTANT_ARC);

    // Get rid of annoying border around spinny loading icon
    lv_obj_set_style_local_line_opa(load_spinner, LV_SPINNER_PART_BG, LV_STATE_DEFAULT, LV_OPA_TRANSP);
    lv_obj_set_style_local_bg_opa(load_spinner, LV_SPINNER_PART_BG, LV_STATE_DEFAULT, LV_OPA_TRANSP);
    lv_obj_set_style_local_border_opa(load_spinner, LV_SPINNER_PART_BG, LV_STATE_DEFAULT, LV_OPA_TRANSP);

    // create status screen
    scr_status = lv_obj_create(NULL, NULL);

    // put volt gauge on it
    static lv_color_t needle_colors[1];
    needle_colors[0] = lv_color_hex(0xffffff);

    vout_gauge = lv_gauge_create(scr_status, NULL);
    lv_gauge_set_needle_count(vout_gauge, 1, needle_colors);
    lv_gauge_set_range(vout_gauge, 234, 13);

    lv_obj_set_style_local_line_opa(vout_gauge, LV_GAUGE_PART_MAIN, LV_STATE_DEFAULT, LV_OPA_TRANSP);
    //lv_obj_set_style_local_bg_opa(vout_gauge, LV_GAUGE_PART_MAIN, LV_STATE_DEFAULT, LV_OPA_TRANSP);
    lv_obj_set_style_local_border_opa(vout_gauge, LV_GAUGE_PART_MAIN, LV_STATE_DEFAULT, LV_OPA_TRANSP);

    lv_obj_set_size(vout_gauge, 64, 64);
    lv_obj_align(vout_gauge, NULL, LV_ALIGN_CENTER, 0, 0);

    // set up fp2 value update lvgl task
    lv_task_t * lv_fp2_task = lv_task_create(lv_fp2_val_update, 500, LV_TASK_PRIO_MID, NULL);

    // switch to the new screen, for testing, because EFFORT
    lv_scr_load_anim(scr_status, LV_SCR_LOAD_ANIM_MOVE_BOTTOM, 350, 0, false);
}

void lv_fp2_val_update(lv_task_t *task) {
    lv_gauge_set_value(vout_gauge, 1, (uint16_t)(fp2.out_volts * 100));
}


/****************************************************************
 * * TWAI control task
 * TODO: most of this if we are honest
 */
void twaiCtrlTask(void *ignore) {
    ESP_LOGI(TWAI_CTRL_TASK_TAG, "TWAI initialization");

    // TWAI_EN transceiver enable GPIO config
    static const gpio_config_t twai_en_conf = {
        .pin_bit_mask = TWAI_EN_GPIO_SEL,
        .mode         = GPIO_MODE_OUTPUT,
        .intr_type    = GPIO_INTR_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .pull_up_en   = GPIO_PULLUP_DISABLE,
    };

    // TWAI controller configuration; 125kbps, all msgIDs are 0x05xxxxxx
    static const twai_general_config_t fp2_twai_g_config =
        TWAI_GENERAL_CONFIG_DEFAULT(CONFIG_TWAI_TX_GPIO, CONFIG_TWAI_RX_GPIO, TWAI_MODE_NORMAL);
    static const twai_timing_config_t fp2_twai_t_config = TWAI_TIMING_CONFIG_125KBITS();
    static const twai_filter_config_t fp2_twai_f_config = TWAI_FILTER_CONFIG_ACCEPT_ALL();

    // Configure transceiver enable pin and disable transceiver
    ESP_ERROR_CHECK(gpio_config(&twai_en_conf));
    gpio_set_level(TWAI_EN_GPIO, 1);
    vTaskDelay(pdMS_TO_TICKS(2));

    // Install TWAI driver
    ESP_ERROR_CHECK_WITHOUT_ABORT(twai_driver_install(&fp2_twai_g_config, &fp2_twai_t_config, &fp2_twai_f_config));
    ESP_LOGD(TWAI_CTRL_TASK_TAG, "TWAI driver and enable pin configured");

    // Enable transceiver and wait for wakeup; 2ms should be more than enough for any transceiver
    gpio_set_level(TWAI_EN_GPIO, 0);
    vTaskDelay(pdMS_TO_TICKS(2));
    ESP_LOGD(TWAI_CTRL_TASK_TAG, "Transceiver enabled");

    // Start driver
    esp_err_t err = twai_start();
    if (err != ESP_OK) {
        ESP_LOGE(TWAI_CTRL_TASK_TAG, "Driver failed to start! Terminating...");
        vTaskDelete(NULL);
    };
    ESP_LOGD(TWAI_CTRL_TASK_TAG, "Driver started");

    // reconfigure TWAI alerts
    uint32_t fp2_twai_a_config = (TWAI_ALERT_AND_LOG | TWAI_ALERT_ERR_ACTIVE | TWAI_ALERT_ARB_LOST | TWAI_ALERT_RX_QUEUE_FULL |
                                  TWAI_ALERT_ABOVE_ERR_WARN | TWAI_ALERT_ERR_PASS | TWAI_ALERT_BUS_OFF);
    twai_reconfigure_alerts(fp2_twai_a_config, NULL);
    ESP_LOGD(TWAI_CTRL_TASK_TAG, "Alert configuration set");

    xEventGroupSetBits(appEventGroup, TWAI_CTRL_RUN_BIT);
    ESP_LOGI(TWAI_CTRL_TASK_TAG, "Initialization complete");

    // set basics of msg_login
    fp2.msg_login.extd             = 1;
    fp2.msg_login.data_length_code = 8;

    // create RX task
    TaskHandle_t rxTaskHandle;
    xTaskCreate(&twaiRxTask, "twaiRxTask", 1024 * 8, &fp2, 4, &rxTaskHandle);

    // create TX task
    TaskHandle_t txTaskHandle;
    xTwaiTxQueue = xQueueCreate(10, sizeof(twai_message_t));
    xTaskCreate(&twaiTxTask, "twaiTxTask", 1024 * 8, NULL, 5, &txTaskHandle);

    // handle alerts and print status
    while (true) {
        esp_err_t err;

        uint32_t alerts;
        err = twai_read_alerts(&alerts, pdMS_TO_TICKS(TWAI_RX_TIMEOUT_SEC * 1000));
        if (err == ESP_OK) {
            // we have some alerts to read
            if (alerts & TWAI_ALERT_RX_QUEUE_FULL) {
                ESP_LOGW(TWAI_CTRL_TASK_TAG, "[ALERT] RX queue is full!");
            }
            if (alerts & TWAI_ALERT_ABOVE_ERR_WARN) {
                ESP_LOGW(TWAI_CTRL_TASK_TAG, "[ALERT] Error Warning Limit Exceeded!");
            }
            if (alerts & TWAI_ALERT_ERR_PASS) {
                ESP_LOGW(TWAI_CTRL_TASK_TAG, "[ALERT] Error Passive State!");
            }
            if (alerts & TWAI_ALERT_ERR_ACTIVE) {
                ESP_LOGW(TWAI_CTRL_TASK_TAG, "[ALERT] Error Active State!");
            }
            if (alerts & TWAI_ALERT_BUS_OFF) {
                ESP_LOGE(TWAI_CTRL_TASK_TAG, "[CRITICAL] Bus-off! Preparing for recovery");
                vTaskSuspend(txTaskHandle);
                vTaskSuspend(rxTaskHandle);
                ESP_LOGE(TWAI_CTRL_TASK_TAG, "[RECOVERY] Suspended tx and rx tasks, initiating recovery");
                twai_reconfigure_alerts(TWAI_ALERT_BUS_RECOVERED, NULL);
                twai_initiate_recovery();
                continue;
            }
            if (alerts & TWAI_ALERT_BUS_RECOVERED) {
                ESP_LOGE(TWAI_CTRL_TASK_TAG, "[RECOVERY] Bus recovery complete! Restarting driver");
                for (int i = 0; i < 3; i++) {
                    esp_err_t err = twai_start();
                    if (err == ESP_OK) {
                        ESP_LOGE(TWAI_CTRL_TASK_TAG, "[RECOVERY] TWAI recovery complete! Resuming tx/rx tasks");
                        vTaskResume(txTaskHandle);
                        vTaskResume(rxTaskHandle);
                        break;
                    } else {
                        ESP_LOGE(TWAI_CTRL_TASK_TAG, "[RECOVERY] TWAI restart attempt %d failed, retrying in 3s...", i + 1);
                        vTaskDelay(pdMS_TO_TICKS(3000));
                    }
                }
            }
        } else if (err != ESP_ERR_TIMEOUT) {
            const char *alert_err_string = esp_err_to_name(err);
            ESP_LOGE(TWAI_CTRL_TASK_TAG, "Retrieving TWAI alerts failed! %s", alert_err_string);
        }
        // get and log status, regardless of whether we succeeded or failed
        twai_status_info_t status;
        twai_get_status_info(&status);
        ESP_LOGD(TWAI_CTRL_TASK_TAG, "[TWAI] state=%d arb=%d err=%d [TX] q=%d err=%d fail=%d [RX] q=%d err=%d miss=%d",
                 (int)status.state, status.arb_lost_count, status.bus_error_count, status.msgs_to_tx, status.tx_error_counter,
                 status.tx_failed_count, status.msgs_to_rx, status.rx_error_counter, status.rx_missed_count);
    }

    // tasks should never return or exit, only ask to be killed
    xEventGroupClearBits(appEventGroup, TWAI_CTRL_RUN_BIT);
    vTaskDelete(NULL);
}

void twaiTxTask(void *ignore) {
    ESP_LOGI(TWAI_TX_TASK_TAG, "TWAI TX task start");
    xEventGroupSetBits(appEventGroup, TWAI_TX_RUN_BIT);

    while (true) {
        twai_message_t txMsg;
        xQueueReceive(xTwaiTxQueue, &txMsg, portMAX_DELAY);
        for (int i = 0; i < TWAI_TX_RETRIES; i++) {
            esp_err_t txErr;
            txErr = twai_transmit(&txMsg, pdMS_TO_TICKS(TWAI_TX_TIMEOUT_SEC * 1000));
            if (txErr == ESP_OK) {
                if (TWAI_MSG_LOG_ALL) logTwaiMsg(&txMsg, 1, "CMD", ESP_LOG_DEBUG);
                break;
            } else {
                const char *tx_err_string = esp_err_to_name(txErr);
                ESP_LOGE(TWAI_TX_TASK_TAG, "[TX][RETRY %d] twai_transmit failed! %s", i + 1, tx_err_string);
            }
        }
    }

    // should never execute
    xEventGroupClearBits(appEventGroup, TWAI_TX_RUN_BIT);
    vTaskDelete(NULL);
}

void twaiRxTask(void *ignore) {
    ESP_LOGI(TWAI_RX_TASK_TAG, "TWAI RX task start");
    xEventGroupSetBits(appEventGroup, TWAI_RX_RUN_BIT);

    // get messages and process the heck out of them
    while (true) {
        twai_message_t rxMsg;
        esp_err_t      rxErr = twai_receive(&rxMsg, portMAX_DELAY);
        if (rxErr == ESP_OK) {
            // will log message at ESP_LOG_WARN level if it doesn't match known IDs
            bool msgProcessed = false;

            // MSG_LOGIN_REQ has a non-fixed ID of 0x0500XXXX, where XXXX = last 4 digits of PSU SN
            // PSU SN is also found in payload bytes 6-7 of this message, so we match on that.
            if (rxMsg.identifier == (FP2_MSG_LOGIN_REQ | (rxMsg.data[5] << 8) | rxMsg.data[6])) {
                if (TWAI_MSG_LOG_ALL) logTwaiMsg(&rxMsg, 0, "LOGIN_REQ", ESP_LOG_DEBUG);
                // updated saved PSU details
                updateFp2Details(&rxMsg, &fp2, 1);
                // send login request, then restart loop
                xQueueSendToFront(xTwaiTxQueue, &fp2.msg_login, portMAX_DELAY);
                continue;
            }

            // now for every other message we care about
            switch (rxMsg.identifier & FP2_MSG_MASK) {
                case FP2_MSG_HELLO:
                    if (TWAI_MSG_LOG_ALL) logTwaiMsg(&rxMsg, 0, "HELLO", ESP_LOG_DEBUG);
                    // updated saved PSU details
                    updateFp2Details(&rxMsg, &fp2, 0);
                    // send a login request
                    xQueueSendToFront(xTwaiTxQueue, &fp2.msg_login, portMAX_DELAY);
                    msgProcessed = true;
                    break;
                case (FP2_MSG_STATUS | 0x04): // 0x04 = CV / normal
                case (FP2_MSG_STATUS | 0x08): // 0x08 = CC / warning
                case (FP2_MSG_STATUS | 0x0C): // 0x0C = alert
                case (FP2_MSG_STATUS | 0x10): // 0x10 = walk-in
                    if (TWAI_MSG_LOG_ALL) logTwaiMsg(&rxMsg, 0, "STATUS", ESP_LOG_DEBUG);
                    // process status message payload
                    processFp2Status(&rxMsg, &fp2);
                    //  queue an alarm request message
                    twai_message_t txMsg;
                    txMsg.extd             = 1;
                    txMsg.identifier       = FP2_CMD_GET_ALARM | fp2.cmd_id;
                    txMsg.data_length_code = 3;
                    txMsg.data[0]          = 0x08;
                    txMsg.data[1]          = rxMsg.identifier & 0xff;
                    txMsg.data[2]          = 0x00;
                    xQueueSend(xTwaiTxQueue, &txMsg, portMAX_DELAY);
                    msgProcessed = true;
                    break;
                case FP2_MSG_ALERTS:
                    if (TWAI_MSG_LOG_ALL) logTwaiMsg(&rxMsg, 0, "ALERTS", ESP_LOG_DEBUG);
                    processFp2Alert(&rxMsg);
                    msgProcessed = true;
                    break;
                default: break;
            }
            // log unknown message types
            if (msgProcessed != true) logTwaiMsg(&rxMsg, 0, "UNKNOWN", ESP_LOG_WARN);
        } else if (rxErr != ESP_ERR_TIMEOUT) {
            const char *rx_err_string = esp_err_to_name(rxErr);
            ESP_LOGE(TWAI_RX_TASK_TAG, "rxMsg error! %s", rx_err_string);
        }
    }

    // tasks should never return or exit, only ask to be killed
    xEventGroupClearBits(appEventGroup, TWAI_RX_RUN_BIT);
    vTaskDelete(NULL);
}


/****************************************************************
 * * RGB LED control task
 * TODO: Make this receive color set requests instead of just doing a rainbow fade
 */
void ledTask(void *ignore) {
    ESP_LOGI(LED_TASK_TAG, "initializing RGB LED");

    // local RGB value variables
    uint32_t red   = 0;
    uint32_t green = 0;
    uint32_t blue  = 0;

    // configure RMT peripheral
    rmt_config_t config = RMT_DEFAULT_CONFIG_TX(CONFIG_FP2S2_RGB_GPIO, RMT_LED_CHANNEL);
    config.clk_div      = 2;
    ESP_ERROR_CHECK(rmt_config(&config));
    ESP_LOGV(LED_TASK_TAG, "RMT driver configured");
    ESP_ERROR_CHECK(rmt_driver_install(config.channel, 0, 0));
    ESP_LOGV(LED_TASK_TAG, "RMT driver installed");

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

    // initialise task handler delay loop
    TickType_t       xLastWakeTime = xTaskGetTickCount();
    const TickType_t xTaskInterval = pdMS_TO_TICKS(1000 / LED_UPDATE_RATE_HZ);

    // this is a placeholder that just does a rainbow cycle until I have this actually set up
    led_hsv_t hsv_val;
    while (true) {
        xQueueReceive(xLedQueue, &hsv_val, 0);
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

    // should never execute
    vQueueDelete(xLedQueue);
    vTaskDelete(NULL);
}


/****************************************************************
 * * ESP Internal Temp Sensor task
 */
void tempSensorTask(void *ignore) {
    ESP_LOGI(TEMP_TASK_TAG, "initialising temp sensor");

    // configure sensor
    temp_sensor_config_t temp_sensor = TSENS_CONFIG_DEFAULT();
    temp_sensor_set_config(temp_sensor);

    // start sensor
    ESP_ERROR_CHECK(temp_sensor_start());
    ESP_LOGD(TEMP_TASK_TAG, "temp sensor initialised!");

    // get current temp and store in global
    temp_sensor_read_celsius(&esp_internal_temp);
    xEventGroupSetBits(appEventGroup, ESP_TEMP_RUN_BIT);
    ESP_LOGI(TEMP_TASK_TAG, "current chip temp %.3f°C", esp_internal_temp);

    // initialise task handler delay loop
    TickType_t       xLastWakeTime;
    const TickType_t xTaskInterval = pdMS_TO_TICKS(ESP_TEMP_POLL_SEC * 1000);

    // update the temp sensor reading roughly every ESP_TEMP_POLL_SEC seconds
    xLastWakeTime = xTaskGetTickCount();
    while (true) {
        vTaskDelayUntil(&xLastWakeTime, xTaskInterval);
        temp_sensor_read_celsius(&esp_internal_temp);
        ESP_LOGD(TEMP_TASK_TAG, "current chip temp %.3f°C", esp_internal_temp);
    }

    xEventGroupClearBits(appEventGroup, ESP_TEMP_RUN_BIT);
    vTaskDelete(NULL);
}


/****************************************************************
 * * Command line console task
 */
void consoleTask(void *ignore) {
    //* initialise console
    initialiseConsole();
    /* Register commands */
    esp_console_register_help_command();
    register_system_common();
    // register_system_sleep();
    // register_nvs();

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

static void initialiseConsole(void) {
    /* Disable buffering on stdin and stdout */
    setvbuf(stdin, NULL, _IONBF, 0);

    /* Minicom, screen, idf_monitor send CR when ENTER key is pressed */
    esp_vfs_dev_cdcacm_set_rx_line_endings(ESP_LINE_ENDINGS_CR);
    /* Move the caret to the beginning of the next line on '\n' */
    esp_vfs_dev_cdcacm_set_tx_line_endings(ESP_LINE_ENDINGS_CRLF);

    /* Enable non-blocking mode on stdin and stdout */
    fcntl(fileno(stdout), F_SETFL, 0);
    fcntl(fileno(stdin), F_SETFL, 0);

    /* initialise the console */
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