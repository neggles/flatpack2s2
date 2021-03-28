/* flatpack2.h  */
#ifndef FLATPACK2_H_
#define FLATPACK2_H_

// need the twai_message_t
#include "driver/twai.h"

// default PSU ID to assign a PSU
#define FP2_ID_DEFAULT 0x01

// bitmasks to extract command / PSU address from msg ID field
#define FP2_MSG_MASK    0xff00ffff
#define FP2_ADDR_MASK   0x00ff0000
#define FP2_STATUS_MASK 0xff00ff00

// TX message IDs for PSU commands
#define FP2_CMD_LOGIN     0x05004800
#define FP2_CMD_SET_DEF   0x05009c00
#define FP2_CMD_SET_OUT   0x05004004
#define FP2_CMD_GET_ALARM 0x0500b00c
#define FP2_WALKIN_5S     0x04
#define FP2_WALKIN_60S    0x05

// RX message IDs from power supply
#define FP2_MSG_STATUS    0x05004000
#define FP2_MSG_LOGIN_REQ 0x05000000
#define FP2_MSG_HELLO     0x05004400
#define FP2_MSG_ALERTS    0x0500BFFC

// status codes (last byte of ID in MSG_STATUS)
#define FP2_STATUS_OK     0x04 /* PSU status normal, CV mode */
#define FP2_STATUS_WARN   0x08 /* PSU status normal, CC mode, triggers warning */
#define FP2_STATUS_ALARM  0x0C /* PSU has a fault, query for fault codes */
#define FP2_STATUS_WALKIN 0x10 /* PSU is in walk-in / warm-up state (lasts 5s or 60s depending on config) */

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
#define FP2_BYTE_INTAKE_TEMP  0 // Intake temperature
#define FP2_BYTE_IOUT_L       1 // Iout low byte
#define FP2_BYTE_IOUT_H       2 // Iout high byte
#define FP2_BYTE_VOUT_L       3 // Vout low byte
#define FP2_BYTE_VOUT_H       4 // Vout high byte
#define FP2_BYTE_VIN_L        5 // Vin low byte
#define FP2_BYTE_VIN_H        6 // Vin high byte
#define FP2_BYTE_EXHAUST_TEMP 7 // Exhaust temperature

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
#define FP2_BYTE_IMAX_L  0 // Iout low byte
#define FP2_BYTE_IMAX_H  1 // Iout high byte
#define FP2_BYTE_VMEAS_L 2 // Vmeas low byte
#define FP2_BYTE_VMEAS_H 4 // Vmeas high byte
#define FP2_BYTE_VSET_L  5 // Vdesired low byte
#define FP2_BYTE_VSET_H  6 // Vdesired high byte
#define FP2_BYTE_VOVP_L  7 // Vovp low byte
#define FP2_BYTE_VOVP_H  8 // Vovp high byte

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
 * Power supply unit data structure
 */

typedef struct {
    uint8_t        serial[6]; //< Serial number as hex digits, e.g. 0x120271100871 = SN 120271100871
    uint8_t        id;        //< PSU's chosen/assigned ID number, 0x04-0x3F
    uint32_t       cmd_id;    //< PSU's command ID number, ID left-shifted by 18 bits
    uint32_t          in_volts; //< current input voltage
    float          out_volts;
    float          out_amps;
    float          out_watts;
    uint32_t       temp_intake;
    uint32_t       temp_exhaust;
    uint8_t        status;
    twai_message_t msg_login; //< message to send this PSU to log into it
} flatpack2_t;

typedef enum {
    send_login,
    set_defaults,
    set_output,
    request_alerts,
} twai_tx_action_t;

#endif /* !FLATPACK2_H_ */