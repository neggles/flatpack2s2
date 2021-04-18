/*
 * Flatpack2s2 ESP32-S2 application
   TODO: This documentation comment block, heh
 */
// * ----------------------------- SDK components --------------------------- */
#pragma region includes

// cstd11
#include <fcntl.h>
#include <stdalign.h>
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
#include "esp_ota_ops.h"
#include "esp_sntp.h"
#include "esp_system.h"
#include "nvs_flash.h"

// ESP-IDF drivers
#include "driver/gpio.h"
#include "driver/i2c.h"
#include "driver/rmt.h"
#include "driver/temp_sensor.h"
#include "driver/twai.h"
#include "soc/rtc.h"
#include "soc/rtc_wdt.h"

// wifi prov mgmt
#include "esp_netif.h"
#include "esp_wifi_types.h"
#include "wifi_manager.h"

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
// #include "lv_theme_fp2.h"
#include "lvgl_gpiodev.h"

// Extra project source files (helper functions etc.)
#include "macros.h"
#include "types.h"
// flatpack2 functions, types, etc - should probably make this a component
#include "flatpack2.h"
#pragma endregion includes

// * -------------------- Definitions and static variables ------------------ */
#pragma region constants

// TZ string for sntp
#define POSIX_TZ CONFIG_POSIX_TZ

// RGB LED driver config
#define RMT_LED_CHANNEL    RMT_CHANNEL_0
#define LED_UPDATE_RATE_HZ 40

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
#define LV_TASK_PERIOD_MS CONFIG_LV_DISP_DEF_REFR_PERIOD


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
static const char *TIMESYNC_TASK_TAG  = "timeSync";


// * Flatpack2-related Constants
// ! see also: src/flatpack2.h
static const uint32_t fp2_abs_vmin = CONFIG_FP2_VOUT_MIN;
static const uint32_t fp2_abs_vmax = CONFIG_FP2_VOUT_MAX;
static const uint32_t fp2_abs_vovp = CONFIG_FP2_VOUT_OVP; // 2 volts over max vdc seems to work
static const uint32_t fp2_abs_imax = CONFIG_FP2_IOUT_MAX;
#pragma endregion constants

// * --------------- Eventgroup, queue, and semaphore handles --------------- */
#pragma region handles

// General shared event group
static EventGroupHandle_t appEventGroup;
// bit assignments
static const int DISP_RUN_BIT       = BIT0;
static const int TWAI_CTRL_RUN_BIT  = BIT1;
static const int TWAI_RX_RUN_BIT    = BIT2;
static const int TWAI_TX_RUN_BIT    = BIT3;
static const int TWAI_ALL_RUN_BITS  = (TWAI_CTRL_RUN_BIT | TWAI_TX_RUN_BIT | TWAI_RX_RUN_BIT);
static const int CONSOLE_RUN_BIT    = BIT4;
static const int LED_RUN_BIT        = BIT5;
static const int TEMP_RUN_BIT       = BIT6;
static const int WIFI_CONNECTED_BIT = BIT10;
static const int TIMESYNC_RUN_BIT   = BIT11;
static const int TIME_VALID_BIT     = BIT12;
static const int FP2_FOUND_BIT      = BIT13;
static const int FP2_SET_REQ_BIT    = BIT14;

// lvgl checks these before moving off the loading screen
static const int ALL_RUN_BITS = (DISP_RUN_BIT | TWAI_ALL_RUN_BITS | CONSOLE_RUN_BIT | LED_RUN_BIT | TEMP_RUN_BIT);

// Task Queues
QueueHandle_t xTwaiTxQueue;
QueueHandle_t xLedQueue;

// LVGL-related
SemaphoreHandle_t xLvglMutex;
#pragma endregion handles


// * --------------------------- Global Variables --------------------------- */
// ! Reads and writes to globals are only atomic for 32-bit values !
#pragma region globals

// esp internal temperature
static float esp_internal_temp;

// fp2 object
static flatpack2_t fp2            = {0};
static fp2_setting_t fp2_settings = {0};

// lvgl objects
static lv_indev_t *gpio_indev;
static lv_group_t *lv_group;
static lv_theme_t *fp2_theme;

static lv_obj_t *scr_def;
static lv_obj_t *scr_status;

static lv_obj_t *lv_tileview;
static lv_obj_t *lv_tile_output;
static lv_obj_t *lv_tile_status;
static lv_obj_t *lv_tile_vars;

static union {
    struct {
        lv_obj_t *vdc;
        lv_obj_t *amps;
        lv_obj_t *watts;
    };
    lv_obj_t *all[3];
} lv_t1_labels;

static union {
    struct {
        lv_obj_t *vac;
        lv_obj_t *intake;
        lv_obj_t *exhaust;
        lv_obj_t *status;
    };
    lv_obj_t *all[4];
} lv_t2_labels;

static union {
    struct {
        lv_obj_t *vac;
        lv_obj_t *vdc;
        lv_obj_t *amps;
        lv_obj_t *watts;
        lv_obj_t *intake;
        lv_obj_t *exhaust;
        lv_obj_t *status;
        lv_obj_t *serial;
    };
    lv_obj_t *all[8];
} lv_t3_labels;

static union {
    struct {
        lv_obj_t *vac;
        lv_obj_t *vdc;
        lv_obj_t *amps;
        lv_obj_t *watts;
        lv_obj_t *intake;
        lv_obj_t *exhaust;
        lv_obj_t *status;
        lv_obj_t *serial;
    };
    lv_obj_t *all[8];
} lv_t3_btns;

static lv_obj_t *app_name;
static lv_obj_t *load_spinner;
#pragma endregion globals

// * ---------------------- Static function prototypes ---------------------- */
#pragma region prototypes

//  Persistent tasks
static void twaiCtrlTask(void *ignore);
static void consoleTask(void *ignore);
static void displayTask(void *ignore);
static void ledTask(void *ignore);
static void tempSensorTask(void *ignore);

// Non-persistent tasks
static void twaiRxTask(void *ignore);
static void twaiTxTask(void *ignore);
static void timeSyncTask(void *ignore);

// Functions
static void initialiseConsole(void);
static void set_fp2_output(void);
static void lvAppCreate(void);
static void lv_val_update(lv_task_t *task);
static void lv_check_startup(lv_task_t *task);

// Callbacks
static void lv_group_focus_cb(lv_group_t *group);
static void lv_log_cb(lv_log_level_t level, const char *file, uint32_t line, const char *fn_name, const char *dsc);
// esp32-wifi-manager
static void cb_netConnected(void *arg);
static void cb_netDisconnected(void *arg);
static void cb_timeSyncEvent(struct timeval *tv);
#pragma endregion prototypes

// * -------------------------- Tasks & Functions --------------------------- */

void set_fp2_output(void) {
    fp2_settings.vset      = 4800;              // 48 volts DC
    fp2_settings.vmeas     = fp2_settings.vset; // no feedback
    fp2_settings.vovp      = fp2_abs_vovp;      // OVP to CONFIG_FP2_VOUT_MAX + 1.5V
    fp2_settings.iout      = 200;               // 20 amps max because of reasons
    fp2_settings.walkin    = FP2_WALKIN_5S;
    fp2_settings.broadcast = 0;
}

/****************************************************************
 * * app_main function - run on startup
 * TODO: implement button handling and screen switching here maybe?
 * TODO: Set GPIO pull-ups for buttons etc.
 */
void app_main(void) {
    ESP_LOGW(TAG, "flatpack2s2 startup!");

    //* logging config
    //esp_log_level_set("*", ESP_LOG_INFO);
    if (TWAI_MSG_LOG_ALL) esp_log_level_set(TWAI_MSG_LOG_TAG, ESP_LOG_DEBUG);
    // esp_log_level_set("wifi:*", ESP_LOG_INFO);
    // esp_log_level_set("httpd_parse", ESP_LOG_INFO);
    // esp_log_level_set("httpd_txrx", ESP_LOG_INFO);
    // esp_log_level_set("httpd_sess", ESP_LOG_INFO);

    //* initialise the main app event group and default event loop
    appEventGroup = xEventGroupCreate();
    assert(appEventGroup != NULL);

    //* wait 2 seconds to allow esptool to flash
    vTaskDelay(pdMS_TO_TICKS(2000));

    //* Initialize NVS partition
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        /* NVS partition was truncated and needs to be erased */
        ESP_ERROR_CHECK(nvs_flash_erase());

        /* Retry nvs_flash_init */
        ESP_ERROR_CHECK(nvs_flash_init());
    }

    //!  temporarary fixed setpoints for PSU
    set_fp2_output();

    //* Create RGB LED update task
    xLedQueue = xQueueCreate(3, sizeof(hsv_t));
    assert(xLedQueue != NULL);
    const hsv_t led_initial = {
        .hue = 260,
        .sat = 100,
        .val = 50,
    };
    xQueueSend(xLedQueue, &led_initial, 0);
    xTaskCreate(&ledTask, "ledTask", 1024 * 2, NULL, 3, NULL);
    xEventGroupWaitBits(appEventGroup, LED_RUN_BIT, pdFALSE, pdTRUE, portMAX_DELAY);

    //* Create TWAI setup task
    xTaskCreate(&twaiCtrlTask, "twaiCtrlTask", 1024 * 6, NULL, 5, NULL);
    xEventGroupWaitBits(appEventGroup, TWAI_ALL_RUN_BITS, pdFALSE, pdTRUE, portMAX_DELAY);

    //* Create temp sensor polling task
    xTaskCreate(&tempSensorTask, "tempSensor", 1024 * 2, NULL, tskIDLE_PRIORITY, NULL);
    xEventGroupWaitBits(appEventGroup, TEMP_RUN_BIT, pdFALSE, pdTRUE, portMAX_DELAY);

    //* start the command-line console task and wait for init
    xTaskCreate(&consoleTask, "consoleTask", 1024 * 6, NULL, 8, NULL);
    xEventGroupWaitBits(appEventGroup, CONSOLE_RUN_BIT, pdFALSE, pdTRUE, portMAX_DELAY);

    //* init wifi manager, register callbacks
    wifi_manager_start();
    // register wifi callbacks
    wifi_manager_set_callback(WM_EVENT_STA_GOT_IP, &cb_netConnected);
    wifi_manager_set_callback(WM_EVENT_STA_DISCONNECTED, &cb_netDisconnected);

    //* wait a second or two for that to do its thing
    vTaskDelay(pdMS_TO_TICKS(1000));

    //* Create OLED display task
    xLvglMutex = xSemaphoreCreateMutex();
    assert(xLvglMutex != NULL);
    // spawn task
    xTaskCreate(&displayTask, "display", 1024 * 12, NULL, 6, NULL);
    // display task may fail, so only waiting 2 seconds here
    xEventGroupWaitBits(appEventGroup, DISP_RUN_BIT, pdFALSE, pdTRUE, pdMS_TO_TICKS(2000));

    //* Create SNTP management task
    xTaskCreate(&timeSyncTask, "timeSync", 1024 * 2, NULL, 2, NULL);

    //* kill the startup watchdog
    rtc_wdt_disable();

    ESP_LOGW(TAG, "flatpack2s2 startup complete!");
}

/****************************************************************
 * * lvgl display management
 */
static void displayTask(void *ignore) {
    ESP_LOGI(DISP_TASK_TAG, "task start");

    // initialize LVGL itself + the ESP32 driver components
    lv_init();
    esp_err_t drv_err = lvgl_driver_init();
    if (drv_err != ESP_OK) {
        ESP_LOGE(DISP_TASK_TAG, "Display driver initialization failed! Terminating display task...");
        vTaskDelete(NULL);
    }
    lv_log_register_print_cb(&lv_log_cb);
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
    lv_color_t *bufmem = heap_caps_malloc(DISP_BUF_SIZE * sizeof(lv_color_t), MALLOC_CAP_DMA);
    assert(bufmem != NULL);
    static lv_disp_buf_t disp_buf;
    uint32_t size_in_px = DISP_BUF_SIZE * 8;

    // initialize display driver
    lv_disp_buf_init(&disp_buf, bufmem, NULL, size_in_px);
    lv_disp_drv_t disp_drv;
    lv_disp_drv_init(&disp_drv);
    disp_drv.flush_cb   = disp_driver_flush;
    disp_drv.rounder_cb = disp_driver_rounder;
    disp_drv.set_px_cb  = disp_driver_set_px;
    disp_drv.buffer     = &disp_buf;
    lv_disp_drv_register(&disp_drv);

    // initialize GPIO indev_drv
    lv_indev_drv_t lv_gpiodev;
    lv_indev_drv_init(&lv_gpiodev);
    lv_gpiodev.type    = LV_INDEV_TYPE_ENCODER;
    lv_gpiodev.read_cb = lvgl_gpiodev_read;
    gpio_indev         = lv_indev_drv_register(&lv_gpiodev);
    // assign it to the group
    lv_group = lv_group_create();
    lv_indev_set_group(gpio_indev, lv_group);

    // set up LVGL app
    lvAppCreate();

    // set display ready bit
    xEventGroupSetBits(appEventGroup, DISP_RUN_BIT);

    lv_scr_load_anim(scr_status, LV_SCR_LOAD_ANIM_MOVE_BOTTOM, 500, 2000, true);

    uint32_t lv_delay;
    while (true) {
        lv_delay = LV_TASK_PERIOD_MS;
        // try to take the semaphore
        if (pdTRUE == xSemaphoreTake(xLvglMutex, pdMS_TO_TICKS(lv_delay))) {
            // run LVGL task handler - returns required delay before running again
            lv_delay = lv_task_handler();
            xSemaphoreGive(xLvglMutex);
        }
        // wait for next interval
        vTaskDelay(pdMS_TO_TICKS(lv_delay));
    }

    /* A task should NEVER return */
    free(bufmem);
    vTaskDelete(NULL);
}

// setup lvgl app screens and transitions
static void lvAppCreate(void) {
    ESP_LOGI(DISP_TASK_TAG, "setting up initial display state");

/*
    lv_theme_t *base_theme = lv_theme_get_act();
    fp2_theme = lv_theme_fp2_init(LV_THEME_DEFAULT_COLOR_PRIMARY, LV_THEME_DEFAULT_COLOR_SECONDARY,
                                  LV_THEME_DEFAULT_FLAG, LV_THEME_DEFAULT_FONT_SMALL, LV_THEME_DEFAULT_FONT_NORMAL,
                                  LV_THEME_DEFAULT_FONT_SUBTITLE, LV_THEME_DEFAULT_FONT_TITLE);
    lv_theme_set_base(fp2_theme, base_theme);
    lv_theme_set_act(fp2_theme);
*/

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
    static lv_point_t lv_tile_valid[] = {{0, 0}, {1, 0}, {2, 0}};

    lv_tileview_set_valid_positions(lv_tileview, lv_tile_valid, 3);
    lv_tileview_set_edge_flash(lv_tileview, true);
    lv_page_set_scrollbar_mode(lv_tileview, LV_SCROLLBAR_MODE_OFF);

    // tile_output: output voltage, current, watts
    lv_tile_output = lv_obj_create(lv_tileview, NULL);
    lv_obj_set_size(lv_tile_output, LV_HOR_RES, LV_VER_RES);
    lv_obj_set_pos(lv_tile_status, 0, 0);
    lv_tileview_add_element(lv_tileview, lv_tile_output);

    lv_t1_labels.vdc = lv_label_create(lv_tile_output, NULL);
    lv_obj_align(lv_t1_labels.vdc, lv_tile_output, LV_ALIGN_IN_TOP_LEFT, 0, 0);
    lv_t1_labels.amps = lv_label_create(lv_tile_output, NULL);
    lv_obj_align(lv_t1_labels.vdc, lv_tile_output, LV_ALIGN_IN_LEFT_MID, 0, 0);
    lv_t1_labels.watts = lv_label_create(lv_tile_output, NULL);
    lv_obj_align(lv_t1_labels.vdc, lv_tile_output, LV_ALIGN_IN_BOTTOM_LEFT, 0, 0);


    // tile_status: input voltage, temps, status code
    lv_tile_status = lv_obj_create(lv_tileview, NULL);
    lv_obj_set_size(lv_tile_status, LV_HOR_RES, LV_VER_RES);
    lv_obj_set_pos(lv_tile_status, LV_HOR_RES, 0);
    lv_tileview_add_element(lv_tileview, lv_tile_status);

    lv_t2_labels.vac = lv_label_create(lv_tile_status, NULL);
    lv_obj_align(lv_t2_labels.vac, lv_tile_status, LV_ALIGN_IN_TOP_LEFT, 0, 0);
    lv_t2_labels.intake = lv_label_create(lv_tile_status, NULL);
    lv_obj_align(lv_t2_labels.intake, lv_tile_status, LV_ALIGN_IN_TOP_RIGHT, 0, 0);
    lv_t2_labels.exhaust = lv_label_create(lv_tile_status, NULL);
    lv_obj_align(lv_t2_labels.exhaust, lv_tile_status, LV_ALIGN_IN_RIGHT_MID, 0, 0);
    lv_t2_labels.status = lv_label_create(lv_tile_status, NULL);
    lv_obj_align(lv_t2_labels.status, lv_tile_status, LV_ALIGN_IN_BOTTOM_LEFT, 0, 0);


    // tile_vars: all available status values, in a list
    lv_tile_vars = lv_list_create(lv_tileview, NULL);
    lv_obj_set_size(lv_tile_vars, LV_HOR_RES, LV_VER_RES);
    lv_obj_set_pos(lv_tile_vars, LV_HOR_RES * 2, 0);
    lv_list_set_scroll_propagation(lv_tile_vars, true);
    lv_list_set_scrollbar_mode(lv_tile_vars, LV_SCROLLBAR_MODE_OFF);

    // add buttons to the list
    lv_t3_btns.vac     = lv_list_add_btn(lv_tile_vars, NULL, "Vin: ?");
    lv_t3_btns.vdc     = lv_list_add_btn(lv_tile_vars, NULL, "Vout: ?");
    lv_t3_btns.amps    = lv_list_add_btn(lv_tile_vars, NULL, "Iout: ?");
    lv_t3_btns.watts   = lv_list_add_btn(lv_tile_vars, NULL, "Power: ?");
    lv_t3_btns.intake  = lv_list_add_btn(lv_tile_vars, NULL, "Intake: ?");
    lv_t3_btns.exhaust = lv_list_add_btn(lv_tile_vars, NULL, "Exhaust: ?");
    lv_t3_btns.status  = lv_list_add_btn(lv_tile_vars, NULL, "Status: ?");
    lv_t3_btns.serial  = lv_list_add_btn(lv_tile_vars, NULL, "SN: ?");

    // get label objects for the buttons so we can update their values later
    lv_t3_labels.vac     = lv_list_get_btn_label(lv_t3_btns.vac);
    lv_t3_labels.vdc     = lv_list_get_btn_label(lv_t3_btns.vdc);
    lv_t3_labels.amps    = lv_list_get_btn_label(lv_t3_btns.amps);
    lv_t3_labels.watts   = lv_list_get_btn_label(lv_t3_btns.watts);
    lv_t3_labels.intake  = lv_list_get_btn_label(lv_t3_btns.intake);
    lv_t3_labels.exhaust = lv_list_get_btn_label(lv_t3_btns.exhaust);
    lv_t3_labels.status  = lv_list_get_btn_label(lv_t3_btns.status);
    lv_t3_labels.serial  = lv_list_get_btn_label(lv_t3_btns.serial);

    // set the fit on all the buttons
    uint32_t num = sizeof(lv_t3_btns.all) / sizeof(lv_t3_btns.all[0]);
    for (uint32_t i = 0; i < num; i++) {
        lv_btn_set_fit2(lv_t3_btns.all[i], LV_FIT_PARENT, LV_FIT_TIGHT);
    }

    // add tiles to lv group
    lv_group_add_obj(lv_group, lv_tile_output);
    lv_group_add_obj(lv_group, lv_tile_status);
    lv_group_add_obj(lv_group, lv_tile_vars);
    lv_group_set_focus_cb(lv_group, &lv_group_focus_cb);
    // lv_group_add_obj(lv_group, lv_tileview);

    // set up fp2 value update lvgl task
    lv_task_t *lv_val_update_task = lv_task_create(lv_val_update, 500, LV_TASK_PRIO_MID, NULL);

    // temporarily set active tile to pg 3
    // lv_tileview_set_tile_act(lv_tileview, 2, 0, LV_ANIM_ON);
}

// lvgl task callbacks
static void lv_val_update(lv_task_t *task) {
    lv_coord_t tile_x;
    lv_coord_t tile_y;
    lv_tileview_get_tile_act(lv_tileview, &tile_x, &tile_y);
    const char *cur_fp2_status = fp2_status_to_str(fp2.status);

    switch (tile_x) {
        case 0: // lv_tile_output
            lv_label_set_text_fmt(lv_t1_labels.vdc, "%4.1fVDC", fp2.sensors.vdc);
            lv_label_set_text_fmt(lv_t1_labels.amps, "%4.1fA", fp2.sensors.amps);
            lv_label_set_text_fmt(lv_t1_labels.watts, "%4.1fW", fp2.sensors.watts);
            break;
        case 1: // lv_tile_status
            lv_label_set_text_fmt(lv_t2_labels.vac, "%4.1fVAC", fp2.sensors.vac);
            lv_label_set_text_fmt(lv_t2_labels.intake, "%2d°C", fp2.sensors.intake);
            lv_label_set_text_fmt(lv_t2_labels.exhaust, "%2d°C", fp2.sensors.exhaust);
            lv_label_set_text_fmt(lv_t2_labels.status, "%s", cur_fp2_status);
            break;
        case 2: // lv_tile_vars
            lv_label_set_text_fmt(lv_t3_labels.vac, "Vin: %3dV AC", fp2.sensors.vac);
            lv_label_set_text_fmt(lv_t3_labels.vdc, "Vout: %4.1fV DC", fp2.sensors.vdc);
            lv_label_set_text_fmt(lv_t3_labels.amps, "Iout: %4.1f A", fp2.sensors.amps);
            lv_label_set_text_fmt(lv_t3_labels.watts, "Power: %4.1f W", fp2.sensors.watts);
            lv_label_set_text_fmt(lv_t3_labels.intake, "Intake: %2d°C", fp2.sensors.intake);
            lv_label_set_text_fmt(lv_t3_labels.exhaust, "Exhaust: %2d°C", fp2.sensors.exhaust);
            lv_label_set_text_fmt(lv_t3_labels.status, "Status: %s", cur_fp2_status);
            lv_label_set_text_fmt(lv_t3_labels.serial, "SN: %02x%02x%02x%02x%02x%02x", fp2.serial[0], fp2.serial[1],
                                  fp2.serial[2], fp2.serial[3], fp2.serial[4], fp2.serial[5]);
            break;
        default: break;
    }
}

// lvgl event Callbacks
static void lv_group_focus_cb(lv_group_t *group) {
    static const char *LOCAL_TAG = "lv_group_focus";
    lv_obj_t *obj                = lv_group_get_focused(group);
    lv_coord_t tile_x;
    lv_coord_t tile_y = 0;
    if (obj == lv_tile_output) {
        tile_x = 0;
        lv_tileview_set_tile_act(lv_tileview, tile_x, tile_y, LV_ANIM_ON);
    } else if (obj == lv_tile_status) {
        tile_x = 1;
        lv_tileview_set_tile_act(lv_tileview, tile_x, tile_y, LV_ANIM_ON);
    } else if (obj == lv_tile_vars) {
        tile_x = 2;
        lv_tileview_set_tile_act(lv_tileview, tile_x, tile_y, LV_ANIM_ON);
    } else {
        ESP_LOGI(LOCAL_TAG, "no tile found to focus");
    }
}

/* static void lv_tv_event_cb(lv_obj_t *tv, lv_event_t e) {
    if (e == LV_EVENT_VALUE_CHANGED || e == LV_EVENT_REFRESH) {
        lv_group_remove_all_objs(lv_group);

        uint16_t tab    = lv_tabview_get_tab_act(tv);
        size_t size     = 0;
        lv_obj_t **objs = NULL;
        if (tab == 0) {
            size = sizeof(selector_objs);
            objs = (lv_obj_t **)&selector_objs;
        } else if (tab == 1) {
            size = sizeof(textinput_objs);
            objs = (lv_obj_t **)&textinput_objs;
        }

        lv_group_add_obj(lv_group, tv);

        uint32_t i;
        for (i = 0; i < size / sizeof(lv_obj_t *); i++) {
            if (objs[i] == NULL) continue;
            lv_group_add_obj(lv_group, objs[i]);
        }
    }
}
 */

// lvgl log callback
static void lv_log_cb(lv_log_level_t level, const char *file, uint32_t line, const char *fn_name, const char *dsc) {
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
 * TODO: actual dynamic adjustment of setpoint
 */
void twaiCtrlTask(void *ignore) {
    ESP_LOGI(TWAI_CTRL_TASK_TAG, "ctrl task start");

    // Configure transceiver enable pin and disable transceiver
    static const gpio_config_t twai_en_conf = {
        .pin_bit_mask = TWAI_EN_GPIO_SEL,
        .mode         = GPIO_MODE_OUTPUT,
        .intr_type    = GPIO_INTR_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .pull_up_en   = GPIO_PULLUP_DISABLE,
    };
    ESP_ERROR_CHECK(gpio_config(&twai_en_conf));
    gpio_set_level(TWAI_EN_GPIO, 1);
    ESP_LOGD(TWAI_CTRL_TASK_TAG, "Transceiver disabled");

    // TWAI controller configuration
#ifdef CONFIG_TWAI_ISR_IN_IRAM
    static const twai_general_config_t fp2_twai_g_config =
        TWAI_GENERAL_CONFIG_IRAM(CONFIG_TWAI_TX_GPIO, CONFIG_TWAI_RX_GPIO, TWAI_MODE_NORMAL);
#else
    static const twai_general_config_t fp2_twai_g_config =
        TWAI_GENERAL_CONFIG_DEFAULT(CONFIG_TWAI_TX_GPIO, CONFIG_TWAI_RX_GPIO, TWAI_MODE_NORMAL);
#endif // CONFIG_TWAI_ISR_IN_IRAM

    // 125kbps
    static const twai_timing_config_t fp2_twai_t_config = TWAI_TIMING_CONFIG_125KBITS();

    // TWAI message filter
#ifdef CONFIG_TWAI_USE_FP2_FILTER
    static const twai_filter_config_t fp2_twai_f_config = TWAI_FILTER_CONFIG_FP2();
#else
    static const twai_filter_config_t fp2_twai_f_config = TWAI_FILTER_CONFIG_ACCEPT_ALL();
#endif

    // enabled TWAI alerts
    uint32_t fp2_twai_a_config = (TWAI_ALERT_ERR_ACTIVE | TWAI_ALERT_ARB_LOST | TWAI_ALERT_RX_QUEUE_FULL |
                                  TWAI_ALERT_ABOVE_ERR_WARN | TWAI_ALERT_ERR_PASS | TWAI_ALERT_BUS_OFF);

    // Install TWAI driver
    ESP_ERROR_CHECK_WITHOUT_ABORT(twai_driver_install(&fp2_twai_g_config, &fp2_twai_t_config, &fp2_twai_f_config));
    ESP_LOGD(TWAI_CTRL_TASK_TAG, "Driver configured");

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

    twai_reconfigure_alerts(fp2_twai_a_config, NULL);
    ESP_LOGD(TWAI_CTRL_TASK_TAG, "Alerts configured");

    xEventGroupSetBits(appEventGroup, TWAI_CTRL_RUN_BIT);
    ESP_LOGI(TWAI_CTRL_TASK_TAG, "Initialization complete, starting tasks");

    // create RX task
    TaskHandle_t rxTaskHandle;
    xTaskCreate(&twaiRxTask, "twaiRxTask", 1024 * 8, NULL, 4, &rxTaskHandle);

    // create TX task
    TaskHandle_t txTaskHandle;
    xTwaiTxQueue = xQueueCreate(10, sizeof(twai_message_t));
    xTaskCreate(&twaiTxTask, "twaiTxTask", 1024 * 8, NULL, 5, &txTaskHandle);

    ESP_LOGD(TWAI_CTRL_TASK_TAG, "Tasks created, entering monitoring loop");

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
                ESP_LOGE(TWAI_CTRL_TASK_TAG, "[RECOVERY] Bus recovered, restarting driver");
                for (int i = 0; i < 3; i++) {
                    twai_reconfigure_alerts(fp2_twai_a_config, NULL);
                    esp_err_t err = twai_start();
                    if (err == ESP_OK) {
                        ESP_LOGE(TWAI_CTRL_TASK_TAG, "[RECOVERY] TWAI recovery complete! Resuming tx/rx tasks");
                        vTaskResume(txTaskHandle);
                        vTaskResume(rxTaskHandle);
                        break;
                    } else {
                        ESP_LOGE(TWAI_CTRL_TASK_TAG, "[RECOVERY] restart attempt %d failed, retrying in 3s...", i + 1);
                        vTaskDelay(pdMS_TO_TICKS(3000));
                    }
                }
            }
        } else if (err != ESP_ERR_TIMEOUT) {
            // something else has gone wrong here
            const char *alert_err_string = esp_err_to_name(err);
            ESP_LOGE(TWAI_CTRL_TASK_TAG, "[ERROR] Retrieving TWAI alerts failed! %s", alert_err_string);
        }
        // get and log status, regardless of whether we succeeded or failed
        twai_status_info_t status;
        twai_get_status_info(&status);
        ESP_LOGD(TWAI_CTRL_TASK_TAG,
                 "[STATUS] state=%d arb=%d err=%d [TX] q=%d err=%d fail=%d [RX] q=%d err=%d miss=%d", (int)status.state,
                 status.arb_lost_count, status.bus_error_count, status.msgs_to_tx, status.tx_error_counter,
                 status.tx_failed_count, status.msgs_to_rx, status.rx_error_counter, status.rx_missed_count);
    }

    // tasks should never return or exit, only ask to be killed
    xEventGroupClearBits(appEventGroup, TWAI_CTRL_RUN_BIT);
    vTaskDelete(NULL);
}

void twaiTxTask(void *ignore) {
    ESP_LOGI(TWAI_TX_TASK_TAG, "tx task start");
    xEventGroupSetBits(appEventGroup, TWAI_TX_RUN_BIT);

    while (true) {
        twai_message_t txMsg = {0};
        xQueueReceive(xTwaiTxQueue, &txMsg, portMAX_DELAY);
        for (int i = 0; i < TWAI_TX_RETRIES; i++) {
            esp_err_t txErr;
            txErr = twai_transmit(&txMsg, pdMS_TO_TICKS(TWAI_TX_TIMEOUT_SEC * 1000));
            if (txErr == ESP_OK) {
                if (TWAI_MSG_LOG_ALL) log_twai_msg(&txMsg, 1, "CMD", ESP_LOG_DEBUG);
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
    ESP_LOGI(TWAI_RX_TASK_TAG, "rx task start");
    xEventGroupSetBits(appEventGroup, TWAI_RX_RUN_BIT);

    // get messages and process the heck out of them
    while (true) {
        // some buffers
        twai_message_t rxMsg = {0};
        twai_message_t txMsg = {0};
        esp_err_t rxErr      = twai_receive(&rxMsg, portMAX_DELAY);
        if (rxErr == ESP_OK) {
            // will log message at ESP_LOG_WARN level if it doesn't match known IDs
            bool msgProcessed = false;

            // MSG_LOGIN_REQ has a non-fixed ID of 0x0500XXXX, where XXXX = last 4 digits of PSU SN
            // PSU SN is also found in payload bytes 6-7 of this message, so we match on that.
            if (rxMsg.identifier == (FP2_MSG_LOGIN_REQ | (rxMsg.data[5] << 8) | rxMsg.data[6])) {
                if (TWAI_MSG_LOG_ALL) log_twai_msg(&rxMsg, 0, "LOGIN_REQ", ESP_LOG_DEBUG);
                // updated saved PSU details
                fp2_save_details(&rxMsg, &fp2, 1);
                // send login request
                xQueueSendToFront(xTwaiTxQueue, &fp2.msg_login, portMAX_DELAY);
                continue;
            }

            // now for every other message we care about
            switch (rxMsg.identifier & FP2_MSG_MASK) {
                case FP2_MSG_HELLO:
                    if (TWAI_MSG_LOG_ALL) log_twai_msg(&rxMsg, 0, "HELLO", ESP_LOG_DEBUG);
                    // updated saved PSU details
                    fp2_save_details(&rxMsg, &fp2, 0);
                    // send a login request
                    xQueueSendToFront(xTwaiTxQueue, &fp2.msg_login, portMAX_DELAY);
                    txMsg = fp2_gen_cmd_set(&fp2, &fp2_settings, fp2_settings.broadcast);
                    // send set request, then restart loop
                    xQueueSend(xTwaiTxQueue, &txMsg, portMAX_DELAY);
                    msgProcessed = true;
                    break;
                case (FP2_MSG_STATUS | FP2_STATUS_WARN):  // 0x08 = CC / warning
                case (FP2_MSG_STATUS | FP2_STATUS_ALERT): // 0x0C = alert
                    // queue alert request only for these two status values
                    txMsg = fp2_gen_cmd_alerts(&fp2, rxMsg.identifier, fp2_settings.broadcast);
                    xQueueSend(xTwaiTxQueue, &txMsg, portMAX_DELAY);
                    // fall through
                case (FP2_MSG_STATUS | FP2_STATUS_OK):     // 0x04 = CV / normal
                case (FP2_MSG_STATUS | FP2_STATUS_WALKIN): // 0x10 = walk-in
                    if (TWAI_MSG_LOG_ALL) log_twai_msg(&rxMsg, 0, "STATUS", ESP_LOG_DEBUG);
                    // process status message payload
                    fp2_update_status(&rxMsg, &fp2);
                    // send a set command
                    txMsg = fp2_gen_cmd_set(&fp2, &fp2_settings, fp2_settings.broadcast);
                    xQueueSend(xTwaiTxQueue, &txMsg, portMAX_DELAY);
                    msgProcessed = true;
                    break;
                case FP2_MSG_ALERTS:
                    if (TWAI_MSG_LOG_ALL) log_twai_msg(&rxMsg, 0, "ALERTS", ESP_LOG_DEBUG);
                    fp2_update_alert(&rxMsg, &fp2);
                    msgProcessed = true;
                    break;
                default: break;
            }
            // log unknown message types
            if (msgProcessed != true) log_twai_msg(&rxMsg, 0, "UNKNOWN", ESP_LOG_WARN);
        } else if (rxErr != ESP_ERR_TIMEOUT) {
            const char *rx_err_string = esp_err_to_name(rxErr);
            ESP_LOGE(TWAI_RX_TASK_TAG, "rxMsg error! %s", rx_err_string);
            continue;
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
    ESP_LOGI(LED_TASK_TAG, "led task start");

    hsv_t hsv;

    // initialise RMT driver, clear LED, set to dim red
    led_init();
    led_clear();
    led_set_hsv(360, 100, 20);

    // set up delay loop
    TickType_t xLastWakeTime       = xTaskGetTickCount();
    const TickType_t xTaskInterval = pdMS_TO_TICKS(1000 / LED_UPDATE_RATE_HZ);

    // signal app_main to continue
    ESP_LOGI(LED_TASK_TAG, "initialization complete");
    xEventGroupSetBits(appEventGroup, LED_RUN_BIT);

    // get me requested LED state and display it
    // TODO: replace this with something using an effects library probably
    while (true) {
        xQueueReceive(xLedQueue, &hsv, pdMS_TO_TICKS(1000 / LED_UPDATE_RATE_HZ));
        led_set_hsv(hsv.hue, hsv.sat, hsv.val);
        vTaskDelayUntil(&xLastWakeTime, xTaskInterval);
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
    xEventGroupSetBits(appEventGroup, TEMP_RUN_BIT);
    ESP_LOGI(TEMP_TASK_TAG, "current chip temp %.3f°C", esp_internal_temp);

    // initialise task handler delay loop
    TickType_t xLastWakeTime;
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

    initialiseConsole();
    esp_console_register_help_command();
    register_system_common();
    register_nvs();

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
        int ret;
        esp_err_t err = esp_console_run(line, &ret);
        if (err == ESP_ERR_NOT_FOUND) {
            printf("Unrecognized command\n");
        } else if (err == ESP_ERR_INVALID_ARG) {
            // command was empty
        } else if (err == ESP_OK && ret != ESP_OK) {
            printf("Command returned non-zero error code: %#x (%s)\n", ret, esp_err_to_name(ret));
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
    esp_console_config_t console_config = {
        .max_cmdline_args = 8, .max_cmdline_length = 256, .hint_color = atoi(LOG_COLOR_CYAN)};
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


/****************************************************************
 * * WiFi/SNTP related tasks and callbacks
 * TODO: Maybe pick up SNTP from DHCP?
 */
void timeSyncTask(void *ignore) {
    ESP_LOGI(TIMESYNC_TASK_TAG, "task start");

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

    if ((xEventGroupGetBits(appEventGroup) & TIMESYNC_RUN_BIT) == 0)
        xEventGroupSetBits(appEventGroup, TIMESYNC_RUN_BIT);

    char strftime_sntp[64];
    time_t sntp        = (time_t)tv->tv_sec;
    struct tm sntpinfo = {0};
    localtime_r(&sntp, &sntpinfo);
    strftime(strftime_sntp, sizeof(strftime_sntp), "%c", &sntpinfo);
    ESP_LOGI(TIMESYNC_TASK_TAG, "received date/time: %s", strftime_sntp);
}

// wifi connected callback
void cb_netConnected(void *arg) {
    ip_event_got_ip_t *net_event = (ip_event_got_ip_t *)arg;
    xEventGroupSetBits(appEventGroup, WIFI_CONNECTED_BIT);

    /* transform IP to human readable string */
    char str_ip[16];
    esp_ip4addr_ntoa(&net_event->ip_info.ip, str_ip, IP4ADDR_STRLEN_MAX);

    ESP_LOGI(TAG, "WiFi connected, IP %s", str_ip);
}

// wifi disconnected callback
void cb_netDisconnected(void *arg) {
    wifi_event_sta_disconnected_t *net_event = (wifi_event_sta_disconnected_t *)arg;
    xEventGroupClearBits(appEventGroup, WIFI_CONNECTED_BIT);

    ESP_LOGW(TAG, "wifi disconnected, boo! reason:%d", net_event->reason);
}