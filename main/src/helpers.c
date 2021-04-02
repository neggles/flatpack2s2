#include "helpers.h"

/**
 * Log tags
 */
static const char *TWAI_MSG_LOG_TAG = "twai.Msg";

/**
 * @brief Simple helper function, converting HSV color space to RGB color space
 */
void led_hsv2rgb(uint32_t h, uint32_t s, uint32_t v, uint32_t *r, uint32_t *g, uint32_t *b) {
    h %= 360; // h -> [0,360]
    uint32_t rgb_max = v * 2.55f;
    uint32_t rgb_min = rgb_max * (100 - s) / 100.0f;

    uint32_t i    = h / 60;
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