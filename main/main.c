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
#include "esp_err.h"
#include "esp_event.h"
#include "esp_freertos_hooks.h"
#include "esp_log.h"
#include "esp_netif.h"
#include "esp_ota_ops.h"
#include "esp_smartconfig.h"
#include "esp_sntp.h"
#include "esp_system.h"
#include "esp_wifi.h"
#include "esp_wpa2.h"
#include "nvs_flash.h"

// ESP-IDF drivers
#include "driver/gpio.h"
#include "driver/i2c.h"
#include "driver/rmt.h"
#include "driver/temp_sensor.h"
#include "driver/twai.h"
#include "soc/rtc.h"

// * --------------------------- Project components ------------------------- */

// Serial command console component
#include "argtable3/argtable3.h"
#include "cmd_nvs.h"
#include "cmd_system.h"
#include "esp_console.h"
#include "esp_vfs_cdcacm.h"
#include "linenoise/linenoise.h"

// RGB LED driver (using RMT peripheral)
#include "ws2812_led.h"

// lvgl graphics library
#include "lvgl.h"
#include "lvgl_helpers.h"
// gpiodev and theme
#include "lv_theme_fp2.h"
#include "lvgl_gpiodev.h"

// ESP Wifi Manager
#include "wifi_manager.h"

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
#define LV_TASK_PERIOD_MS 15

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
static const char *TIMESYNC_TASK_TAG  = "sntp";


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
static const int CONSOLE_RUN_BIT    = BIT4;
static const int LED_RUN_BIT        = BIT5;
static const int TEMP_RUN_BIT       = BIT6;
static const int WIFI_CONNECTED_BIT = BIT10;
static const int ESPTOUCH_DONE_BIT  = BIT11;
static const int TIMESYNC_RUN_BIT   = BIT12;
static const int TIME_VALID_BIT     = BIT13;


// Task Queues
static QueueHandle_t xTwaiTxQueue;
static QueueHandle_t xLedQueue;

// LVGL-related
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
static lv_obj_t *  lv_tile_output;
static lv_obj_t *  lv_tile_status;
static lv_obj_t *  lv_tile_vars;
static lv_obj_t *  lv_vin;
static lv_obj_t *  lv_vout;
static lv_obj_t *  lv_iout;
static lv_obj_t *  lv_wout;
static lv_obj_t *  lv_temp;
static lv_obj_t *  app_name;
static lv_obj_t *  load_spinner;

static lv_obj_t *lv_tileview;

// setpoints
static uint32_t set_voltage; // desired setpoint voltage
static uint32_t max_voltage; // Over-Voltage Protection voltage
static uint32_t max_current; // Maximum current limit


// * ---------------------- Static function prototypes ---------------------- */

//  Persistent tasks
static void twaiCtrlTask(void *ignore);
static void consoleTask(void *ignore);
static void displayTask(void *ignore);
static void lvTaskHandler(void *ignore);
static void ledTask(void *ignore);
static void tempSensorTask(void *ignore);
static void timeSyncTask(void *ignore);

// Non-persistent tasks
static void twaiRxTask(void *ignore);
static void twaiTxTask(void *ignore);
static void smartConfigTask(void *ignore);

// Functions
static void wifi_init(void);
static void initialiseConsole(void);
static void lvAppCreate(void);
static void lv_val_update(lv_task_t *task);

// Callbacks
static void scEventHandler(void *arg, esp_event_base_t event_base, int32_t event_id, void *event_data);
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

    //* initialize NVS and start the wifi setup process
    ESP_ERROR_CHECK(nvs_flash_init());
    // wifi_init();

    //* start the command-line console task and wait for init
    initialiseConsole();
    esp_console_register_help_command();
    register_system_common();
    xTaskCreate(&consoleTask, "consoleTask", 1024 * 6, NULL, 6, NULL);
    xEventGroupWaitBits(appEventGroup, CONSOLE_RUN_BIT, pdFALSE, pdTRUE, portMAX_DELAY);

    //* Create OLED display task
    xLvglMutex = xSemaphoreCreateMutex();
    assert(xLvglMutex != NULL);

    // spawn task
    xTaskCreate(&displayTask, "display", 1024 * 8, NULL, 15, NULL);
    xEventGroupWaitBits(appEventGroup, DISP_RUN_BIT, pdFALSE, pdTRUE, portMAX_DELAY);

    //* Create TWAI setup task
    xTaskCreate(&twaiCtrlTask, "twaiCtrlTask", 1024 * 6, NULL, 5, NULL);
    xEventGroupWaitBits(appEventGroup, (TWAI_CTRL_RUN_BIT | TWAI_TX_RUN_BIT | TWAI_RX_RUN_BIT), pdFALSE, pdTRUE, portMAX_DELAY);

    //* Create RGB LED update task
    xLedQueue = xQueueCreate(3, sizeof(hsv_t));
    assert(xLedQueue != NULL);
    hsv_t led_initial = {
        .hue = 0,
        .sat = 100,
        .val = 50,
    };
    xQueueSend(xLedQueue, &led_initial, 0);
    xTaskCreate(&ledTask, "ledTask", 1024 * 2, NULL, 3, NULL);
    xEventGroupWaitBits(appEventGroup, LED_RUN_BIT, pdFALSE, pdTRUE, portMAX_DELAY);

    //* Create temp sensor polling task
    xTaskCreate(&tempSensorTask, "tempSensor", 1024 * 2, NULL, tskIDLE_PRIORITY, NULL);
    xEventGroupWaitBits(appEventGroup, TEMP_RUN_BIT, pdFALSE, pdTRUE, portMAX_DELAY);

    //* init wifi manager, register callbacks
    // wrapping the stupid thing in delays seems to make it happier? idfk, there's something fucky going on with lvgl here
    // wifi_manager_start();

    //* Create SNTP management task
    xTaskCreate(&timeSyncTask, "timeSync", 1024 * 2, NULL, 2, NULL);
}

/****************************************************************
 * * lvgl display management
 */
static void displayTask(void *ignore) {
    ESP_LOGI(DISP_TASK_TAG, "display task begin");

    // initialize LVGL itself + the ESP32 driver components
    lv_init();
    lvgl_driver_init();
    lvgl_gpiodev_init();

    /**
     * Configure LVGL display and input devices
     *
     * NOTE: When using a monochrome display:
     *      - No double-buffering, so buf2 == NULL
     *      - We need two extra callbacks:
     *          - rounder_cb
     *          - set_px_cb
     *      - Normally size_in_px = DISP_BUF_SIZE, but this display is 1BPP,
     *        so size_in_px = DISP_BUF_SIZE * 8
     */
    // create display buffer
    lv_color_t *buf1 = heap_caps_malloc(DISP_BUF_SIZE * sizeof(lv_color_t), MALLOC_CAP_DMA);
    assert(buf1 != NULL);
    static lv_color_t *  buf2 = NULL;
    static lv_disp_buf_t disp_buf;
    uint32_t             size_in_px = DISP_BUF_SIZE * 8;

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

    // set up LVGL app
    lvAppCreate();

    // start handler task
    xTaskCreate(&lvTaskHandler, "lvgl.handler", 1024 * 2, NULL, 15, NULL);

    // set display ready bit
    xEventGroupSetBits(appEventGroup, DISP_RUN_BIT);

    while (true) {
        vTaskDelay(portMAX_DELAY);
    }

    /* A task should NEVER return */
    free(buf1);
    vTaskDelete(NULL);
}

// putting this in its own stupid thing
static void lvTaskHandler(void *ignore) {
    uint32_t lv_delay;
    while (true) {
        lv_delay = LV_TASK_PERIOD_MS;
        // try to take the semaphore
        if (pdTRUE == xSemaphoreTake(xLvglMutex, pdMS_TO_TICKS(lv_delay / 2))) {
            // run LVGL task handler - returns required delay before running again
            lv_delay = lv_task_handler();
            xSemaphoreGive(xLvglMutex);
        }
        // wait for next interval
        vTaskDelay(pdMS_TO_TICKS(lv_delay));
    }
}

// setup lvgl app screens and transitions
static void lvAppCreate(void) {
    ESP_LOGI(DISP_TASK_TAG, "setting up initial display state");

    // lv_theme_t * fp2_theme = lv_theme_fp2_init();

    // get app data
    const esp_app_desc_t *app_info = esp_ota_get_app_description();

    // Get current/default screen
    scr_def = lv_disp_get_scr_act(NULL);
    // Create project name label
    app_name = lv_label_create(scr_def, NULL);
    lv_label_set_text(app_name, app_info->project_name);
    lv_obj_align(app_name, NULL, LV_ALIGN_IN_BOTTOM_MID, 0, 0);
    // create spinny loading icon
    load_spinner = lv_spinner_create(scr_def, NULL);
    lv_obj_set_size(load_spinner, 90, 90);
    lv_obj_align(load_spinner, NULL, LV_ALIGN_CENTER, 0, -5);
    lv_spinner_set_type(load_spinner, LV_SPINNER_TYPE_CONSTANT_ARC);
    // Get rid of annoying border around spinny loading icon
    lv_obj_set_style_local_line_opa(load_spinner, LV_SPINNER_PART_BG, LV_STATE_DEFAULT, LV_OPA_TRANSP);
    lv_obj_set_style_local_bg_opa(load_spinner, LV_SPINNER_PART_BG, LV_STATE_DEFAULT, LV_OPA_TRANSP);
    lv_obj_set_style_local_border_opa(load_spinner, LV_SPINNER_PART_BG, LV_STATE_DEFAULT, LV_OPA_TRANSP);


    // create status screen
    scr_status  = lv_obj_create(NULL, NULL);
    lv_tileview = lv_tileview_create(scr_status, NULL);
    // valid position grid
    static lv_point_t lv_tiles[] = {{0, 0}, {1, 0}, {2, 0}};

    lv_tileview_set_valid_positions(lv_tileview, lv_tiles, 3);
    lv_tileview_set_edge_flash(lv_tileview, true);
    lv_page_set_scrollbar_mode(lv_tileview, LV_SCRLBAR_MODE_OFF);

    // tile 1: output voltage, current, watts
    lv_tile_output = lv_obj_create(lv_tileview, NULL);
    lv_obj_set_size(lv_tile_output, LV_HOR_RES, LV_VER_RES);
    lv_tileview_add_element(lv_tileview, lv_tile_output);

    // tile_status: input voltage, temps, status code
    lv_tile_status = lv_obj_create(lv_tileview, NULL);
    lv_obj_set_size(lv_tile_status, LV_HOR_RES, LV_VER_RES);
    lv_tileview_add_element(lv_tileview, lv_tile_status);

    // tile_vars: all available status values, in a list
    lv_tile_vars = lv_list_create(lv_tileview, NULL);
    lv_obj_set_size(lv_tile_vars, LV_HOR_RES, LV_VER_RES);
    lv_obj_set_pos(lv_tile_vars, 0, LV_VER_RES);
    lv_list_set_scroll_propagation(lv_tile_vars, true);
    lv_list_set_scrollbar_mode(lv_tile_vars, LV_SCROLLBAR_MODE_OFF);


    // set up fp2 value update lvgl task
    lv_task_t *lv_val_update_task = lv_task_create(lv_val_update, 500, LV_TASK_PRIO_MID, NULL);

    // switch to the new screen, for testing, because EFFORT
    lv_scr_load_anim(scr_status, LV_SCR_LOAD_ANIM_MOVE_BOTTOM, 350, 0, false);
}


// lvgl event callbacks
void lv_val_update(lv_task_t *task) {
    lv_label_set_text_fmt(lv_vin, " Vin: %4dV", fp2.in_volts);
    lv_label_set_text_fmt(lv_vout, "Vout: %4.1fV", fp2.out_volts);
    lv_label_set_text_fmt(lv_iout, "Iout: %4.1fA", fp2.out_amps);
    lv_label_set_text_fmt(lv_temp, "Temp: I %2d°C O %2d°C", fp2.temp_intake, fp2.temp_exhaust);
    lv_label_set_text_fmt(lv_wout, "Wout: %4.1fW", fp2.out_watts);
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
    // enable logging, unless TWAI ISR is in IRAM in which case it doesn't work anyway
#ifdef CONFIG_TWAI_ISR_IN_IRAM
    uint32_t fp2_twai_a_config = (TWAI_ALERT_ERR_ACTIVE | TWAI_ALERT_ARB_LOST | TWAI_ALERT_RX_QUEUE_FULL | TWAI_ALERT_ABOVE_ERR_WARN |
                                  TWAI_ALERT_ERR_PASS | TWAI_ALERT_BUS_OFF);
#else
    uint32_t fp2_twai_a_config = (TWAI_ALERT_AND_LOG | TWAI_ALERT_ERR_ACTIVE | TWAI_ALERT_ARB_LOST | TWAI_ALERT_RX_QUEUE_FULL |
                                  TWAI_ALERT_ABOVE_ERR_WARN | TWAI_ALERT_ERR_PASS | TWAI_ALERT_BUS_OFF);
#endif // CONFIG_TWAI_ISR_IN_IRAM

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
                    twai_reconfigure_alerts(fp2_twai_a_config, NULL);
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

    hsv_t hsv;

    // do init
    led_init();

    // turn off LED
    led_clear();

    // initialise task handler delay loop
    TickType_t       xLastWakeTime = xTaskGetTickCount();
    const TickType_t xTaskInterval = pdMS_TO_TICKS(1000 / LED_UPDATE_RATE_HZ);

    ESP_LOGI(LED_TASK_TAG, "initialization complete");
    xEventGroupSetBits(appEventGroup, LED_RUN_BIT);

    // get me requested LED state and display it
    // TODO: replace this with something using an effects library probably
    while (true) {
        xQueueReceive(xLedQueue, &hsv, portMAX_DELAY);
        led_set_hsv(hsv.hue, hsv.sat, hsv.val);
        vTaskDelayUntil(&xLastWakeTime, xTaskInterval);
    }

    // should never execute
    vQueueDelete(xLedQueue);
    vTaskDelete(NULL);
}


/****************************************************************
 * * Wifi Provisioning Task
 */
static void smartConfigTask(void *ignore) {
    EventBits_t uxBits;
    ESP_ERROR_CHECK(esp_smartconfig_set_type(SC_TYPE_ESPTOUCH));
    smartconfig_start_config_t cfg = SMARTCONFIG_START_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_smartconfig_start(&cfg));
    while (1) {
        uxBits = xEventGroupWaitBits(appEventGroup, WIFI_CONNECTED_BIT | ESPTOUCH_DONE_BIT, true, false, portMAX_DELAY);
        if (uxBits & WIFI_CONNECTED_BIT) {
            ESP_LOGI(TAG, "WiFi Connected");
        }
        if (uxBits & ESPTOUCH_DONE_BIT) {
            ESP_LOGI(TAG, "SmartConfig Done");
            esp_smartconfig_stop();
            vTaskDelete(NULL);
        }
    }
}

static void wifi_init(void) {
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    esp_netif_t *sta_netif = esp_netif_create_default_wifi_sta();
    assert(sta_netif);

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));

    ESP_ERROR_CHECK(esp_event_handler_register(WIFI_EVENT, ESP_EVENT_ANY_ID, &scEventHandler, NULL));
    ESP_ERROR_CHECK(esp_event_handler_register(IP_EVENT, IP_EVENT_STA_GOT_IP, &scEventHandler, NULL));
    ESP_ERROR_CHECK(esp_event_handler_register(SC_EVENT, ESP_EVENT_ANY_ID, &scEventHandler, NULL));

    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_start());
}

static void scEventHandler(void *arg, esp_event_base_t event_base, int32_t event_id, void *event_data) {
    if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_START) {
        xTaskCreate(smartConfigTask, "smartconfig", 1024 * 4, NULL, 3, NULL);
    } else if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_DISCONNECTED) {
        esp_wifi_connect();
        xEventGroupClearBits(appEventGroup, WIFI_CONNECTED_BIT);
    } else if (event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP) {
        xEventGroupSetBits(appEventGroup, WIFI_CONNECTED_BIT);
    } else if (event_base == SC_EVENT && event_id == SC_EVENT_SCAN_DONE) {
        ESP_LOGI(TAG, "Scan done");
    } else if (event_base == SC_EVENT && event_id == SC_EVENT_FOUND_CHANNEL) {
        ESP_LOGI(TAG, "Found channel");
    } else if (event_base == SC_EVENT && event_id == SC_EVENT_GOT_SSID_PSWD) {
        ESP_LOGI(TAG, "Got SSID and password");

        smartconfig_event_got_ssid_pswd_t *evt = (smartconfig_event_got_ssid_pswd_t *)event_data;
        wifi_config_t                      wifi_config;
        uint8_t                            ssid[33]     = {0};
        uint8_t                            password[65] = {0};
        uint8_t                            rvd_data[33] = {0};

        bzero(&wifi_config, sizeof(wifi_config_t));
        memcpy(wifi_config.sta.ssid, evt->ssid, sizeof(wifi_config.sta.ssid));
        memcpy(wifi_config.sta.password, evt->password, sizeof(wifi_config.sta.password));
        wifi_config.sta.bssid_set = evt->bssid_set;
        if (wifi_config.sta.bssid_set == true) {
            memcpy(wifi_config.sta.bssid, evt->bssid, sizeof(wifi_config.sta.bssid));
        }

        memcpy(ssid, evt->ssid, sizeof(evt->ssid));
        memcpy(password, evt->password, sizeof(evt->password));
        ESP_LOGI(TAG, "SSID:%s", ssid);
        ESP_LOGI(TAG, "PASSWORD:%s", password);
        if (evt->type == SC_TYPE_ESPTOUCH_V2) {
            ESP_ERROR_CHECK(esp_smartconfig_get_rvd_data(rvd_data, sizeof(rvd_data)));
            ESP_LOGI(TAG, "RVD_DATA:");
            for (int i = 0; i < 33; i++) {
                printf("%02x ", rvd_data[i]);
            }
            printf("\n");
        }

        ESP_ERROR_CHECK(esp_wifi_disconnect());
        ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &wifi_config));
        esp_wifi_connect();
    } else if (event_base == SC_EVENT && event_id == SC_EVENT_SEND_ACK_DONE) {
        xEventGroupSetBits(appEventGroup, ESPTOUCH_DONE_BIT);
    }
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

    // configure and start SNTP
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

    if ((xEventGroupGetBits(appEventGroup) & TIMESYNC_RUN_BIT) == 0) xEventGroupSetBits(appEventGroup, TIMESYNC_RUN_BIT);

    char      strftime_sntp[64];
    time_t    sntp     = (time_t)tv->tv_sec;
    struct tm sntpinfo = {0};
    localtime_r(&sntp, &sntpinfo);
    strftime(strftime_sntp, sizeof(strftime_sntp), "%c", &sntpinfo);
    ESP_LOGI(TIMESYNC_TASK_TAG, "received date/time: %s", strftime_sntp);
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
    xEventGroupSetBits(appEventGroup, TEMP_RUN_BIT);
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

    xEventGroupClearBits(appEventGroup, TEMP_RUN_BIT);
    vTaskDelete(NULL);
}


/****************************************************************
 * * Command line console task
 */
void consoleTask(void *ignore) {

    /**
     * Prompt to be printed before each line.
     * This can be customized, made dynamic, etc.
     */
    const char *prompt = LOG_COLOR(LOG_COLOR_PURPLE) "flatpack2s2> " LOG_RESET_COLOR;

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