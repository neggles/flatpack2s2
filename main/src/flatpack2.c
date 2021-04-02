#include "flatpack2.h"

// * --------------------------- Global Variables --------------------------- */

/**
 * Alert strings
 */
const char *fp2_alerts0_str[] = {"OVS Lockout",       "Primary Module Failure", "Secondary Module Failure", "Mains Voltage High",
                                 "Mains Voltage Low", "Temperature High",       "Temperature Low",          "Current Over Limit"};
const char *fp2_alerts1_str[] = {"Internal Voltage Fault", "Module Failure",        "Secondary Module Failure",
                                 "Fan 1 Speed Low",        "Fan 2 Speed Low",       "Sub-module 1 Failure",
                                 "Fan 3 Speed Low",        "Internal Voltage Fault"};
/**
 * Log tags
 */
const char *FP2_STATUS_TAG = "fp2.Status";
const char *FP2_ALERT_TAG  = "fp2.Alert";
const char *FP2_LOGIN_TAG  = "fp2.Login";


// * ---------------------- Flatpack2 related functions --------------------- */

/**
 * @brief Process a MSG_LOGIN_REQ or MSG_HELLO and update the flatpack2 object to match
 *
 * @param rxMsg Pointer to twai_message_t
 * @param psu Pointer to flatpack2_t
 * @param isLoginReq true if MSG_LOGIN_REQ, false if MSG_HELLO
 */
void updateFp2Details(twai_message_t *rxMsg, flatpack2_t *psu, int isLoginReq) {
    const char *msgType = NULL;
    if (isLoginReq) {
        msgType = "LOGIN_REQ";
    } else {
        msgType = "HELLO";
    }

    if (memcmp(&psu->serial[0], &rxMsg->data[isLoginReq], sizeof(psu->serial)) != 0) {
        ESP_LOGI(FP2_LOGIN_TAG, "[RX][%s] Saved PSU SN mismatch, updating", msgType);

        // update PSU ID
        if (isLoginReq) {
            // MSG_LOGIN_REQ doesn't have a PSU ID for us to extract, default to ID FP2_ID_DEFAULT
            psu->id     = FP2_ID_DEFAULT;
            psu->cmd_id = (FP2_ID_DEFAULT << 18);
        } else {
            // Extract existing PSU ID from MSG_HELLO
            psu->id     = (rxMsg->identifier >> 16) & 0xff;
            psu->cmd_id = (rxMsg->identifier & 0x00ff0000) << 2;
        }

        // update serial number
        memcpy(&psu->serial[0], &rxMsg->data[isLoginReq], sizeof(psu->serial));

        // update msg_login identifier and payload
        psu->msg_login.identifier = FP2_CMD_LOGIN | (psu->id << 2);
        memcpy(&psu->msg_login.data[0], &rxMsg->data[isLoginReq], sizeof(psu->serial));

        // print device found message, warning log level so it stands out
        ESP_LOGW(FP2_LOGIN_TAG, "[RX][%s] PSU ID 0x%02x found: S/N %02x%02x%02x%02x%02x%02x, msg_login id 0x%08x", msgType, psu->id,
                 psu->serial[0], psu->serial[1], psu->serial[2], psu->serial[3], psu->serial[4], psu->serial[5],
                 psu->msg_login.identifier);

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
void processFp2Status(twai_message_t *rxMsg, flatpack2_t *psu) {
    psu->temp_intake  = rxMsg->data[FP2_BYTE_INTAKE_TEMP];
    psu->temp_exhaust = rxMsg->data[FP2_BYTE_EXHAUST_TEMP];
    psu->out_amps     = ((rxMsg->data[FP2_BYTE_IOUT_H] << 8) + rxMsg->data[FP2_BYTE_IOUT_L]) * 0.1;
    psu->out_volts    = ((rxMsg->data[FP2_BYTE_VOUT_H] << 8) + rxMsg->data[FP2_BYTE_VOUT_L]) * 0.01;
    psu->out_watts    = psu->out_amps * psu->out_volts;
    psu->in_volts     = ((rxMsg->data[FP2_BYTE_VIN_H] << 8) + rxMsg->data[FP2_BYTE_VIN_L]);
    psu->status       = rxMsg->identifier & 0xff;
    ESP_LOGI(FP2_STATUS_TAG, "[RX][MSG_STATUS] ID 0x%02x: VIn %03dV, VOut %2.2fV, IOut %2.2fA, POut %4.2fW", psu->id, psu->in_volts,
             psu->out_volts, psu->out_amps, psu->out_watts);
    ESP_LOGI(FP2_STATUS_TAG, "[RX][MSG_STATUS]          Intake %d°C, Exhaust %d°C, Status 0x%02x", psu->temp_intake, psu->temp_exhaust,
             psu->status);
}

/**
 * @brief process flatpack2 alert messages
 *
 * @param rxMsg pointer to twai_message_t received alert message
 */
void processFp2Alert(twai_message_t *rxMsg) {
    uint8_t alert_byte_low  = rxMsg->data[3];
    uint8_t alert_byte_high = rxMsg->data[4];
    uint8_t alert_psu_id    = (rxMsg->identifier >> 16) & 0xff;
    switch (rxMsg->data[1]) {
        case FP2_STATUS_WARN:
            ESP_LOGW(FP2_ALERT_TAG, "Warning message from PSU %02x:", alert_psu_id);
            for (int i = 0; i < 8; i++) {
                if (alert_byte_low & (0x1 << i)) {
                    ESP_LOGW(FP2_ALERT_TAG, "  W: %s", fp2_alerts0_str[i]);
                }
                if (alert_byte_high & (0x1 << i)) {
                    ESP_LOGW(FP2_ALERT_TAG, "  W: %s", fp2_alerts1_str[i]);
                }
            }
            break;
        case FP2_STATUS_ALARM:
            ESP_LOGE(FP2_ALERT_TAG, "Alert message from PSU %02x:", alert_psu_id);
            for (int i = 0; i < 8; i++) {
                if (alert_byte_low & (1 << i)) {
                    ESP_LOGE(FP2_ALERT_TAG, "  E: %s", fp2_alerts0_str[i]);
                }
                if (alert_byte_high & (1 << i)) {
                    ESP_LOGE(FP2_ALERT_TAG, "  E: %s", fp2_alerts1_str[i]);
                }
            }
            break;
        default: break;
    }
}