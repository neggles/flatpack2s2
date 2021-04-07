#pragma once
/* flatpack2.h  */

#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

// need the twai_message_t and esp_log funcs
#include "hal/twai_types.h"
#include <esp_log.h>

// default PSU ID to assign a PSU
#define FP2_ID_DEFAULT 0x01

// bitmasks to extract command / PSU address from msg ID field
#define FP2_MSG_MASK    0xff00ffff
#define FP2_ADDR_MASK   0x00ff0000
#define FP2_STATUS_MASK 0xff00ff00

// TX message IDs for PSU commands
#define FP2_CMD_LOGIN    0x05004800
#define FP2_CMD_SET_DEF  0x05009c00
#define FP2_CMD_SET_OUT  0x05004000 //< must be ORed with fp2_walkin_t
#define FP2_CMD_ALERTS   0x0500bffc
#define FP2_CMD_ADDR_ALL 0x00ff0000 //< command address for "all logged-in PSUs"

// RX message IDs from power supply
#define FP2_MSG_STATUS    0x05004000
#define FP2_MSG_LOGIN_REQ 0x05000000
#define FP2_MSG_HELLO     0x05004400
#define FP2_MSG_ALERTS    0x0500BFFC

// status codes (last byte of ID in MSG_STATUS)
#define FP2_STATUS_OK     0x04 /* PSU status normal, CV mode */
#define FP2_STATUS_WARN   0x08 /* PSU status normal, CC mode, triggers warning */
#define FP2_STATUS_ALERT  0x0C /* PSU has a fault, query for fault codes */
#define FP2_STATUS_WALKIN 0x10 /* PSU is in walk-in / warm-up state (lasts 5s or 60s depending on config) */

// TWAI filter initializer macro
// TODO: make this work... it worked before...
#define TWAI_FILTER_CONFIG_FP2() \
    { .acceptance_code = 0x05000000, .acceptance_mask = 0x00FFFFFF, .single_filter = true }


/**
 * Status message payload bytes.
 * Expected RX ID is 0x05XX40YY, XX = PSU ID, YY = one of FP2_STATUS_* (0x04/08/0C/10)
 *
 * All values are little-endian, split into low and high bytes
 * Intake and exhaust temperatures are 8-bit uints in degrees Celsius
 * Vin, Vout, Iout are all 16-bit uints split into low and high bytes
 * Vin and Vout are in centiVolts (1/100ths of a volt or 10mV/bit)
 * Iout is in deciAmps (1/10ths of an amp or 100mA/bit)
 * All values are little-endian
 *
 */
typedef enum {
    INTAKE_TEMP_BYTE, // Intake temperature
    IOUT_LOW_BYTE,    // Iout low byte
    IOUT_HIGH_BYTE,   // Iout high byte
    VOUT_LOW_BYTE,    // Vout low byte
    VOUT_HIGH_BYTE,   // Vout high byte
    VIN_LOW_BYTE,     // Vin low byte
    VIN_HIGH_BYTE,    // Vin high byte
    EXHAUST_TEMP_BYTE // Exhaust temperature
} msg_status_byte_t;

/**
 * Set command payload bytes.
 * Send to 0x05XX400Y, XX = PSU ID, Y = 4 for 5s walk-in, 5 for 6s walk-in
 *
 * Same formats as the status messages above, all 16-bit in this case.
 * Measured voltage is for point-of-load voltage feedback, Desired voltage is voltage setpoint
 * Set them to the same value to use factory calibration, or set meas to actual measured PoL voltage if available
 * OVP voltage is the threshold for PSU emergency shutdown, on a FP2 HE 48V/2000W this should be 59.5V
 *
 */
typedef enum {
    IMAX_LOW_BYTE,   // Iout low byte
    IMAX_HIGH_BYTE,  // Iout high byte
    VMEAS_LOW_BYTE,  // Vmeas low byte
    VMEAS_HIGH_BYTE, // Vmeas high byte
    VSET_LOW_BYTE,   // Vdesired low byte
    VSET_HIGH_BYTE,  // Vdesired high byte
    VOVP_LOW_BYTE,   // Vovp low byte
    VOVP_HIGH_BYTE,  // Vovp high byte
} cmd_set_byte_t;

/**
 * Defaults set command payload bytes.
 * Send to 0x05XX9C00, XX = PSU ID
 *
 * Does not take effect until the supply logs out.
 * First 3 bytes are fixed 0x29 0x15 0x00
 * Next two bytes are default voltage in the same format as the status messages.
 *
 * No definition here, this comment is just for note/reference purposes.
 */

/**
 * Shared power supply settings structures
 */

// walk-in rate - set via last octet of CMD_SET message ID
typedef enum {
    FP2_WALKIN_5S  = 0x04, //< 5-second ramp-up
    FP2_WALKIN_60S = 0x05  //< 60-second ramp-up
} fp2_walkin_t;

// used to store shared PSU settings; probably doesn't need to be a typedef
typedef struct {
    union {
        uint8_t data[8];
        struct {
            uint32_t iout : 16;  //< current limit, deciAmps
            uint32_t vmeas : 16; //< measured PSU output, centiVolts
            uint32_t vset : 16;  //< setpoint voltage
            uint32_t vovp : 16;  //< overvoltage protection point
        };
    };
    uint32_t     broadcast;
    fp2_walkin_t walkin;
} fp2_setting_t;


/**
 * Power supply unit data structures
 */

// convenience structure for PSU's current alert flag bytes
typedef struct {
    union {
        struct {
            uint32_t byte1 : 8;
            uint32_t byte2 : 8;
            uint32_t reserved : 16;
        };
        uint32_t flags;
    };
} fp2_alert_t;

// PSU readings
typedef struct {
    uint32_t vac;
    float    vdc;
    float    amps;
    float    watts;
    uint32_t intake;
    uint32_t exhaust;
} fp2_sensor_t;

// actual PSU object, combining the above
typedef struct {
    uint8_t        serial[6]; //< Serial number as hex digits, e.g. 0x120271100871 = SN 120271100871
    uint32_t       id;        // PSU ID, 0x01-0x3F
    uint32_t       cmd_id;    // PSU ID left-shifted 16 bits for convenience
    uint32_t       offset;    //< PSU-specific value for feedback voltage sensing; not implemented
    uint32_t       status;
    fp2_sensor_t   sensors;
    fp2_alert_t    alert;
    twai_message_t msg_login; //< message to send this PSU to log into it
} flatpack2_t;

extern const char *fp2_alerts0_str[];
extern const char *fp2_alerts1_str[];
extern const char *FP2_ALERT_TAG;
extern const char *FP2_LOGIN_TAG;

/**
 * Function declarations
 */
// log a twai message (fairly generic function)
void log_twai_msg(twai_message_t *twaiMsg, int is_tx, const char *msgType, esp_log_level_t errLevel);

// update saved details
void fp2_save_details(twai_message_t *rxMsg, flatpack2_t *psu, int isLoginReq);

// process status message
void fp2_update_status(twai_message_t *rxMsg, flatpack2_t *psu);

// process alert message
void fp2_update_alert(twai_message_t *rxMsg, flatpack2_t *psu);

// generate a set command
twai_message_t fp2_gen_cmd_set(flatpack2_t *psu, fp2_setting_t *set, uint32_t broadcast);

// generate an alarm get command
twai_message_t fp2_gen_cmd_alerts(flatpack2_t *psu, uint32_t msgId, uint32_t broadcast);