// freeRTOS
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"

// project includes
#include "flatpack2.h"
#include "macros.h"
#include "types.h"


// * --------------------------- Global Variables --------------------------- */
// Log tags
const char *TAG           = "fp2";
const char *FP2_ALERT_TAG = "fp2.Alert";
const char *FP2_LOGIN_TAG = "fp2.Login";

// Alert strings
const char *fp2_alerts0_str[] = {"OVS Lockout",        "Primary Module Failure", "Secondary Module Failure",
                                 "Mains Voltage High", "Mains Voltage Low",      "Temperature High",
                                 "Temperature Low",    "Current Over Limit"};
const char *fp2_alerts1_str[] = {"Internal Voltage Fault", "Module Failure",        "Secondary Module Failure",
                                 "Fan 1 Speed Low",        "Fan 2 Speed Low",       "Sub-module 1 Failure",
                                 "Fan 3 Speed Low",        "Internal Voltage Fault"};


const fp2_status_code_t fp2_status_table[] = {
    {0x0, "NOT PRESENT"},
    {FP2_STATUS_OK, "OK (CV)"},
    {FP2_STATUS_WARN, "WARN (CC)"},
    {FP2_STATUS_ALERT, "ALERT"},
    {FP2_STATUS_WALKIN, "WALK-IN"}
};

static const hsv_t led_fp2_found = {
    .hue = 120,
    .sat = 100,
    .val = 50
};

extern QueueHandle_t xLedQueue;

// * ------------------------------ Functions ------------------------------- */

/**
 * @brief log TWAI RX message
 *
 * @param twaiMsg   pointer to twai_message_t to log
 * @param is_tx     0 = RX message, 1 = TX message
 * @param msgType   contents of second [] box in log entry
 * @param errLevel  esp_log_level_t to log this at
 */
void log_twai_msg(twai_message_t *twaiMsg, int is_tx, const char *msgType, esp_log_level_t errLevel) {
    static const char *TAG = "twai.Msg";
    // print message ID and type at desired error level
    const char *msgDir = NULL;
    if (is_tx) {
        msgDir = "TX";
    } else {
        msgDir = "RX";
    }

    // print message into buffer and log at appropriate level
    char buf[100];
    snprintf(buf, sizeof(buf), "[%s][%s] ID %#010x\tlen:%.2d\textd:%.1d\trtr:%.1d\tdlc_nc=%.1d", msgDir, msgType,
             twaiMsg->identifier, twaiMsg->data_length_code, twaiMsg->extd, twaiMsg->rtr, twaiMsg->dlc_non_comp);
    switch (errLevel) {
        case ESP_LOG_VERBOSE: ESP_LOGV(TAG, "%s", buf); break;
        case ESP_LOG_DEBUG: ESP_LOGD(TAG, "%s", buf); break;
        case ESP_LOG_INFO: ESP_LOGI(TAG, "%s", buf); break;
        case ESP_LOG_WARN: ESP_LOGW(TAG, "%s", buf); break;
        case ESP_LOG_ERROR: ESP_LOGE(TAG, "%s", buf); break;
        default: ESP_LOGI(TAG, "%s", buf); break;
    }

    // dump message payload bytes
    ESP_LOG_BUFFER_HEXDUMP(TAG, &twaiMsg->data[0], twaiMsg->data_length_code, errLevel);
}


// * --------------------- Flatpack2-specific functions --------------------- */

/**
 * @brief Process a MSG_LOGIN_REQ or MSG_HELLO and update the flatpack2 object to match
 *
 * @param rxMsg         Pointer to twai_message_t
 * @param psu           Pointer to flatpack2_t
 * @param isLoginReq    true if MSG_LOGIN_REQ, false if MSG_HELLO
 */
void fp2_save_details(twai_message_t *rxMsg, flatpack2_t *psu, int isLoginReq) {
    const char *msgType = NULL;
    msgType             = isLoginReq ? "LOGIN_REQ" : "HELLO";

    // set basics of msg_login
    psu->msg_login.extd             = 1;
    psu->msg_login.data_length_code = 8;

    if (memcmp(&psu->serial[0], &rxMsg->data[isLoginReq], sizeof(psu->serial)) != 0) {
        ESP_LOGI(FP2_LOGIN_TAG, "[RX][%s] Saved PSU SN mismatch, updating", msgType);

        // update PSU ID
        if (isLoginReq == 1) {
            // MSG_LOGIN_REQ doesn't have a PSU ID for us to extract, default to ID FP2_ID_DEFAULT
            psu->id     = FP2_ID_DEFAULT;
            psu->cmd_id = (FP2_ID_DEFAULT << 18);
        } else {
            // Extract existing PSU ID from MSG_HELLO
            psu->id     = (rxMsg->identifier & 0x00ff0000) >> 18;
            psu->cmd_id = (rxMsg->identifier & 0x00ff0000);
        }

        // update serial number
        memcpy(&psu->serial[0], &rxMsg->data[isLoginReq], sizeof(psu->serial));

        // update msg_login identifier and payload
        psu->msg_login.identifier = (psu->id << 2) | FP2_CMD_LOGIN;
        memcpy(&psu->msg_login.data[0], &rxMsg->data[isLoginReq], sizeof(psu->serial));

        // send a color change to the led task
        xQueueSend(xLedQueue, &led_fp2_found, pdMS_TO_TICKS(100));

        // print device found message, warning log level so it stands out
        ESP_LOGW(FP2_LOGIN_TAG, "[RX][%s] PSU ID %#04x found: S/N %02x%02x%02x%02x%02x%02x, msg_login id %#010x",
                 msgType, psu->id, psu->serial[0], psu->serial[1], psu->serial[2], psu->serial[3], psu->serial[4],
                 psu->serial[5], psu->msg_login.identifier);

    } else {
        ESP_LOGD(FP2_LOGIN_TAG, "[RX][%s] Saved PSU SN matches, no action", msgType);
    }
}


/**
 * @brief process flatpack2 status message and update the data struct
 *
 * @param rxMsg pointer to twai_message_t received status message
 * @param psu   pointer to flatpack2_t this message is for
 */
void fp2_update_status(twai_message_t *rxMsg, flatpack2_t *psu) {
    psu->sensors.intake  = rxMsg->data[INTAKE_TEMP_BYTE];
    psu->sensors.exhaust = rxMsg->data[EXHAUST_TEMP_BYTE];
    psu->sensors.amps    = ((rxMsg->data[IOUT_HIGH_BYTE] << 8) + rxMsg->data[IOUT_LOW_BYTE]) * 0.1;
    psu->sensors.vdc     = ((rxMsg->data[VOUT_HIGH_BYTE] << 8) + rxMsg->data[VOUT_LOW_BYTE]) * 0.01;
    psu->sensors.watts   = psu->sensors.amps * psu->sensors.vdc;
    psu->sensors.vac     = ((rxMsg->data[VIN_HIGH_BYTE] << 8) + rxMsg->data[VIN_LOW_BYTE]);
    psu->status          = rxMsg->identifier & 0xff;
    ESP_LOGI(TAG, "[RX][MSG_STATUS] ID %#04x: VIn %03dV, VOut %2.2fV, IOut %2.1fA, POut %4.2fW", psu->id,
             psu->sensors.vac, psu->sensors.vdc, psu->sensors.amps, psu->sensors.watts);
    ESP_LOGI(TAG, "[RX][MSG_STATUS]          Intake %d°C, Exhaust %d°C, Status %#04x", psu->sensors.intake,
             psu->sensors.exhaust, psu->status);
}

/**
 * @brief process flatpack2 alert messages
 *
 * @param rxMsg pointer to twai_message_t received alert message
 * @param psu   pointer to flatpack2_t this message is for
 */
void fp2_update_alert(twai_message_t *rxMsg, flatpack2_t *psu) {
    psu->alert.byte1 = rxMsg->data[3];
    psu->alert.byte2 = rxMsg->data[4];

    switch (rxMsg->data[1]) {
        case FP2_STATUS_OK:
            ESP_LOGW(FP2_ALERT_TAG, "Warning message from PSU %#04x:", psu->id);
            for (int i = 0; i < 8; i++) {
                if (psu->alert.byte1 & (0x1 << i)) {
                    ESP_LOGW(FP2_ALERT_TAG, "  W: %s", fp2_alerts0_str[i]);
                }
                if (psu->alert.byte2 & (0x1 << i)) {
                    ESP_LOGW(FP2_ALERT_TAG, "  W: %s", fp2_alerts1_str[i]);
                }
            }
            break;
        case FP2_STATUS_WARN:
            ESP_LOGE(FP2_ALERT_TAG, "Alert message from PSU %#04x:", psu->id);
            for (int i = 0; i < 8; i++) {
                if (psu->alert.byte1 & (1 << i)) {
                    ESP_LOGE(FP2_ALERT_TAG, "  E: %s", fp2_alerts0_str[i]);
                }
                if (psu->alert.byte2 & (1 << i)) {
                    ESP_LOGE(FP2_ALERT_TAG, "  E: %s", fp2_alerts1_str[i]);
                }
            }
            break;
        default:
            ESP_LOGE(FP2_ALERT_TAG, "Unknown alert message from PSU %#04x:", psu->id);
            ESP_LOG_BUFFER_HEXDUMP(FP2_ALERT_TAG, &rxMsg->data[0], rxMsg->data_length_code, ESP_LOG_ERROR);
            break;
    }
}


/**
 * @brief generate a set command
 *
 * @param psu       pointer to flatpack2_t
 * @param set       pointer to fp2_settings_t
 * @param broadcast true if we're addressing this to ID 0xFF, false if PSU-specific
 * @return          twai_message_t message to be passed to the tx queue
 */
twai_message_t fp2_gen_cmd_set(flatpack2_t *psu, fp2_setting_t *set, uint32_t broadcast) {
    twai_message_t txMsg = {0};
    // feed data to message
    if (broadcast == 1) {
        txMsg.identifier = (FP2_CMD_ADDR_ALL | FP2_CMD_SET_OUT | set->walkin);
    } else {
        txMsg.identifier = (psu->cmd_id | FP2_CMD_SET_OUT | set->walkin);
    }

    txMsg.extd             = 1;
    txMsg.self             = 0;
    txMsg.data_length_code = 8;
    memcpy(&txMsg.data[0], &set->data[0], txMsg.data_length_code);

    ESP_LOGI(TAG, "[TX][CMD_SET]    ID %#04x: Vset %04d Vmeas %04d Vmax %04d Iout %04d msgId %#010x", psu->id,
             set->vset, set->vmeas, set->vovp, set->iout, txMsg.identifier);
    return txMsg;
}

/**
 * @brief generate a get_alert command
 *
 * @param psu       pointer to flatpack2_t
 * @param msgId     message id of the received message
 * @return          twai_message_t message to be passed to the tx queue
 */
twai_message_t fp2_gen_cmd_alerts(flatpack2_t *psu, uint32_t msgId, uint32_t broadcast) {
    twai_message_t txMsg = {0};

    if (broadcast == 1) {
        txMsg.identifier = (FP2_CMD_ADDR_ALL | FP2_CMD_ALERTS);
    } else {
        txMsg.identifier = (psu->cmd_id | FP2_CMD_ALERTS);
    }

    txMsg.extd             = 1;
    txMsg.self             = 0;
    txMsg.data_length_code = 3;
    txMsg.data[0]          = 0x08;
    txMsg.data[1]          = LowByte(msgId);
    txMsg.data[2]          = 0x00;
    ESP_LOGI(TAG, "[TX][CMD_ALERTS]    ID %#04x: status %#04x msgId %#010x", psu->id, txMsg.data[1], txMsg.identifier);
    return txMsg;
}

/**
 * @brief generate a set defaults command
 *
 * @param psu       pointer to flatpack2_t
 * @param set       pointer to fp2_settings_t
 * @param broadcast true if we're addressing this to ID 0xFF, false if PSU-specific
 * @return          twai_message_t message to be passed to the tx queue
 */
twai_message_t fp2_gen_cmd_defaults(flatpack2_t *psu, fp2_setting_t *set, uint32_t broadcast) {
    twai_message_t txMsg = {0};
    // feed data to message
    if (broadcast == 1) {
        txMsg.identifier = (FP2_CMD_ADDR_ALL | FP2_CMD_SET_DEF);
    } else {
        txMsg.identifier = (psu->cmd_id | FP2_CMD_SET_DEF);
    }

    txMsg.extd             = 1;
    txMsg.self             = 0;
    txMsg.data_length_code = 8;
    memcpy(&txMsg.data[0], &set->data[0], txMsg.data_length_code);

    ESP_LOGI(TAG, "[TX][CMD_DEF]    ID %#04x: Vset %04d Vmeas %04d Vmax %04d Iout %04d msgId %#010x", psu->id,
             set->vset, set->vmeas, set->vovp, set->iout, txMsg.identifier);
    return txMsg;
}

/**
 * @brief converts a flatpack2 status code 0xYY to a string
 *
 * @param code          the code to convert
 * @return const char*  pointer to the string representation
 */
const char *fp2_status_to_str(fp2_status_t code) {
    size_t i;
    for (i = 0; i < sizeof(fp2_status_table) / sizeof(fp2_status_table[0]); ++i) {
        if (fp2_status_table[i].code == code) {
            return fp2_status_table[i].msg;
        }
    }
    return "UNKNOWN";
}