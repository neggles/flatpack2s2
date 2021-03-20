/* flatpack2.h  */
#ifndef FLATPACK2_H_
#define FLATPACK2_H_

// bitmasks to extract command / PSU address from msg ID field
#define FP2_MSG_MASK    0xff00ffff
#define FP2_ADDR_MASK   0x00ff0000
#define FP2_LOGIN_MASK  0xffff0000
#define FP2_STATUS_MASK 0xffffff00

// commands - TX to power supply
#define FP2_CMD_LOGIN     0x05004800
#define FP2_CMD_SET_DEF   0x05009c00
#define FP2_CMD_SET_OUT   0x05004004
#define FP2_CMD_GET_ALARM 0x0500b00c
#define FP2_WALKIN_5S     0x04
#define FP2_WALKIN_60S    0x05

// responses - RX from power supply
#define FP2_MSG_STATUS       0x05004000
#define FP2_MSG_LOGIN_REQ    0x05000000
#define FP2_MSG_LOGIN_WALKIN 0x05004400
#define FP2_MSG_ALARMS       0x0500BFFC

// status codes (last byte of ID in MSG_STATUS)
#define FP2_STATUS_OK     0x04
#define FP2_STATUS_WARN   0x08
#define FP2_STATUS_ALARM  0x0C
#define FP2_STATUS_WALKIN 0x10

// login repeat interval - 10 sec should be fine, but 5 sec is nicer
#define FP2_LOGIN_INTERVAL 5

// byte defs in status
#define FP2_BYTE_INTAKE_TEMP  0
#define FP2_BYTE_IOUT_L       1
#define FP2_BYTE_IOUT_H       2
#define FP2_BYTE_VOUT_L       3
#define FP2_BYTE_VOUT_H       4
#define FP2_BYTE_VIN_L        5
#define FP2_BYTE_VIN_H        6
#define FP2_BYTE_EXHAUST_TEMP 7

typedef struct {
    uint8_t  serial[6];
    uint8_t  id;
    uint32_t login_id;
} flatpack2_t;

typedef enum {
    TX_FP2_LOGIN,
    TX_FP2_SET_DEF,
    TX_FP2_SET_OUT,
    TX_FP2_GET_ALARM,
    TX_TASK_EXIT,
} twai_tx_action_t;

typedef enum {
    RX_FP2_ID,
    RX_FP2_STATUS,
    RX_FP2_ALARMS,
    RX_FP2_LOGIN_REQ,
} twai_rx_action_t;

#endif /* !FLATPACK2_H_ */