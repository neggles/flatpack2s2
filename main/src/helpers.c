#include "helpers.h"

/**
 * Log tags
 */
static const char *TWAI_MSG_LOG_TAG = "twai.Msg";

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