#include "flatpack2.h"
// project macros
#include "macros.h"

// * --------------------------- Global Variables --------------------------- */
/**
 * Log tags
 */
const char *TAG           = "fp2";
const char *FP2_ALERT_TAG = "fp2.Alert";
const char *FP2_LOGIN_TAG = "fp2.Login";

/**
 * Alert strings
 */
const char *fp2_alerts0_str[] = {"OVS Lockout",        "Primary Module Failure", "Secondary Module Failure",
                                 "Mains Voltage High", "Mains Voltage Low",      "Temperature High",
                                 "Temperature Low",    "Current Over Limit"};
const char *fp2_alerts1_str[] = {"Internal Voltage Fault", "Module Failure",        "Secondary Module Failure",
                                 "Fan 1 Speed Low",        "Fan 2 Speed Low",       "Sub-module 1 Failure",
                                 "Fan 3 Speed Low",        "Internal Voltage Fault"};


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
    snprintf(buf, sizeof(buf), "[%s][%s] ID %#010x, len=%.2d, extd=%.1d, rtr=%.1d, dlc_non_comp=%.1d", msgDir, msgType,
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
 * @param rxMsg Pointer to twai_message_t
 * @param psu Pointer to flatpack2_t
 * @param isLoginReq true if MSG_LOGIN_REQ, false if MSG_HELLO
 */
void fp2_save_details(twai_message_t *rxMsg, flatpack2_t *psu, int isLoginReq) {
    const char *msgType = NULL;
    if (isLoginReq) {
        msgType = "LOGIN_REQ";
    } else {
        msgType = "HELLO";
    }

    // set basics of msg_login
    psu->msg_login.extd             = 1;
    psu->msg_login.data_length_code = 8;

    if (memcmp(&psu->serial[0], &rxMsg->data[isLoginReq], sizeof(psu->serial)) != 0) {
        ESP_LOGI(FP2_LOGIN_TAG, "[RX][%s] Saved PSU SN mismatch, updating", msgType);

        // update PSU ID
        if (isLoginReq) {
            // MSG_LOGIN_REQ doesn't have a PSU ID for us to extract, default to ID FP2_ID_DEFAULT
            psu->id     = FP2_ID_DEFAULT;
            psu->cmd_id = (FP2_ID_DEFAULT << 16);
        } else {
            // Extract existing PSU ID from MSG_HELLO
            psu->id     = (rxMsg->identifier >> 18) & 0xff;
            psu->cmd_id = ((psu->id << 16) & 0x00ff0000);
        }

        // update serial number
        memcpy(&psu->serial[0], &rxMsg->data[isLoginReq], sizeof(psu->serial));

        // update msg_login identifier and payload
        psu->msg_login.identifier = FP2_CMD_LOGIN | (psu->id << 2);
        memcpy(&psu->msg_login.data[0], &rxMsg->data[isLoginReq], sizeof(psu->serial));

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
 * @param psu pointer to flatpack2_t this message is for
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
 * @param psu pointer to flatpack2_t this message is for
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
 * @param settings  pointer to fp2_settings_t
 * @param txMsg     pointer to twai_message_t to prepare
 */
twai_message_t fp2_gen_cmd_set(flatpack2_t *psu, fp2_setting_t *set) {
    twai_message_t txMsg;
    // feed data to message
    txMsg.extd             = 1;
    txMsg.identifier       = (psu->cmd_id | FP2_CMD_SET_OUT);
    txMsg.data_length_code = 8;
    txMsg.self             = 0;
    memcpy(&txMsg.data[0], &set->data[0], txMsg.data_length_code);

    ESP_LOGI(TAG, "[TX][CMD_SET][%#010x] PSU %02d: Vset %04d Vmeas %04d Vmax %04d Iout %04d",
             txMsg.identifier, psu->id, set->vset, set->vmeas, set->vovp, set->iout);
    return txMsg;
}

/**
 * @brief generate a get_alert command
 *
 * @param psu       pointer to flatpack2_t
 * @param msgId     message id of the received message
 * @return twai_message_t message to be passed to the tx queue
 */
twai_message_t fp2_gen_cmd_alerts(flatpack2_t *psu, uint32_t msgId) {
    twai_message_t txMsg;
    txMsg.extd             = 1;
    txMsg.identifier       = (psu->cmd_id | FP2_CMD_GET_ALARM);
    txMsg.data_length_code = 3;
    txMsg.self             = 0;
    txMsg.data[0]          = 0x08;
    txMsg.data[1]          = LowByte(msgId);
    txMsg.data[2]          = 0x00;
    return txMsg;
}