#include "helpers.h"

#include "argtable3/argtable3.h"
#include "esp_console.h"
#include "esp_log.h"
#include "sdkconfig.h"
#include "wifi_provisioning/manager.h"
#include <ctype.h>
#include <stdio.h>
#include <string.h>
#include <unistd.h>

/**
 * Log tags
 */
static const char *TAG              = "fp2s2_helpers";
static const char *TWAI_MSG_LOG_TAG = "twai.Msg";

static void register_wifi_prov(void);

void register_flatpack2(void) {
    register_wifi_prov();
}

// dump received TWAI message to console
void logTwaiMsg(twai_message_t *twaiMsg, int is_tx, const char *msgType, esp_log_level_t errLevel) {
    // print message ID and type at desired error level
    const char *msgDir = NULL;
    if (is_tx) {
        msgDir = "TX";
    } else {
        msgDir = "RX";
    }

    // print message into buffer and log at appropriate level
    char buf[100];
    snprintf(buf, sizeof(buf), "[%s][%s] ID 0x%08x, len=%.2d, extd=%.1d, rtr=%.1d, dlc_non_comp=%.1d", msgDir, msgType,
             twaiMsg->identifier, twaiMsg->data_length_code, twaiMsg->extd, twaiMsg->rtr, twaiMsg->dlc_non_comp);
    switch (errLevel) {
        case ESP_LOG_VERBOSE: ESP_LOGV(TWAI_MSG_LOG_TAG, "%s", buf); break;
        case ESP_LOG_DEBUG: ESP_LOGD(TWAI_MSG_LOG_TAG, "%s", buf); break;
        case ESP_LOG_INFO: ESP_LOGI(TWAI_MSG_LOG_TAG, "%s", buf); break;
        case ESP_LOG_WARN: ESP_LOGW(TWAI_MSG_LOG_TAG, "%s", buf); break;
        case ESP_LOG_ERROR: ESP_LOGE(TWAI_MSG_LOG_TAG, "%s", buf); break;
        default: ESP_LOGV(TWAI_MSG_LOG_TAG, "%s", buf); break;
    }

    // dump message payload bytes
    ESP_LOG_BUFFER_HEXDUMP(TWAI_MSG_LOG_TAG, &twaiMsg->data[0], twaiMsg->data_length_code, errLevel);
}

static struct {
    struct arg_str *arg;
    struct arg_end *end;
} wifi_prov_args;

static int wifi_prov(int argc, char **argv) {
    int nerrors = arg_parse(argc, argv, (void **)&wifi_prov_args);
    if (nerrors != 0) {
        arg_print_errors(stderr, wifi_prov_args.end, argv[0]);
        return 1;
    }

    const char *action = wifi_prov_args.arg->sval[0];
    if (strcmp(action, "reset") == 0) {
        wifi_prov_mgr_reset_provisioning();
        ESP_LOGW(TAG, "WiFi provisioning reset! Restarting in 3s...");
        vTaskDelay(pdMS_TO_TICKS(3000));
        esp_restart();
    }

    ESP_LOGE(TAG, "cmd wifi_prov action %s not recognized", action);
    return 1;
}

static void register_wifi_prov(void) {
    const esp_console_cmd_t cmd = {
        .command = "wifi",
        .help    = "WiFi Provisioning Manager control",
        .hint    = NULL,
        .func    = &wifi_prov,
    };
    ESP_ERROR_CHECK(esp_console_cmd_register(&cmd));
}