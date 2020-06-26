/*
 / _____)             _              | |
( (____  _____ ____ _| |_ _____  ____| |__
 \____ \| ___ |    (_   _) ___ |/ ___)  _ \
 _____) ) ____| | | || |_| ____( (___| | | |
(______/|_____)_|_|_| \__)_____)\____)_| |_|
  (C)2019 Semtech

Description:
    Functions used to handle LoRa concentrator SX1261 radio.

License: Revised BSD License, see LICENSE.TXT file include in the project
*/


#ifndef _SX1261_DEFS_H
#define _SX1261_DEFS_H

/* -------------------------------------------------------------------------- */
/* --- DEPENDANCIES --------------------------------------------------------- */

#include <stdint.h>     /* C99 types*/

#include "config.h"     /* library configuration options (dynamically generated) */

/* -------------------------------------------------------------------------- */
/* --- PUBLIC MACROS -------------------------------------------------------- */

#define SX1261_FREQ_TO_REG(f) (uint32_t)((uint64_t)f * (1 << 25) / 32000000U)

/* -------------------------------------------------------------------------- */
/* --- PUBLIC CONSTANTS ----------------------------------------------------- */

/* -------------------------------------------------------------------------- */
/* --- PUBLIC TYPES --------------------------------------------------------- */

typedef enum {
    SX1261_CALIBRATE_IMAGE          = 0x98,
    SX1261_CLR_IRQ_STATUS           = 0x02,
    SX1261_STOP_TIMER_ON_PREAMBLE   = 0x9F,
    SX1261_SET_RFSWITCHMODE         = 0x9D,
    SX1261_GET_IRQ_STATUS           = 0x12,
    SX1261_GET_RX_BUFFER_STATUS     = 0x13,
    SX1261_GET_PACKET_STATUS        = 0x14,
    SX1261_GET_RSSI_INST            = 0x15,
    SX1261_READ_BUFFER              = 0x1E,
    SX1261_READ_REGISTER            = 0x1D,
    SX1261_SET_DIO_IRQ_PARAMS       = 0x08,
    SX1261_SET_MODULATION_PARAMS    = 0x8B,
    SX1261_SET_PA_CONFIG            = 0x95,
    SX1261_SET_PACKET_PARAMS        = 0x8C,
    SX1261_SET_PACKET_TYPE          = 0x8A,
    SX1261_SET_RF_FREQUENCY         = 0x86,
    SX1261_SET_BUFFER_BASE_ADDRESS  = 0x8F,
    SX1261_SET_SLEEP                = 0x84,
    SX1261_SET_STANDBY              = 0x80,
    SX1261_SET_RX                   = 0x82,
    SX1261_SET_TX                   = 0x83,
    SX1261_SET_TX_PARAMS            = 0x8E,
    SX1261_WRITE_BUFFER             = 0x0E,
    SX1261_WRITE_REGISTER           = 0x0D,
    SX1261_SET_TXCONTINUOUSWAVE     = 0xD1,
    SX1261_SET_TXCONTINUOUSPREAMBLE = 0xD2,
    SX1261_GET_STATUS               = 0xC0,
    SX1261_SET_REGULATORMODE        = 0x96,
    SX1261_SET_FS                   = 0xC1,
    SX1261_GET_DEVICE_ERRORS        = 0x17
} sx1261_op_code_t;

typedef enum {
    SX1261_STDBY_RC                 = 0x00,
    SX1261_STDBY_XOSC               = 0x01
} sx1261_standby_modes_t;

typedef enum {
    SX1261_PACKET_TYPE_GFSK         = 0x00,
    SX1261_PACKET_TYPE_LORA         = 0x01
} sx1261_packet_type_t;

typedef enum {
    SX1261_SET_RAMP_10U             = 0x00,
    SX1261_SET_RAMP_20U             = 0x01,
    SX1261_SET_RAMP_40U             = 0x02,
    SX1261_SET_RAMP_80U             = 0x03,
    SX1261_SET_RAMP_200U            = 0x04,
    SX1261_SET_RAMP_800U            = 0x05,
    SX1261_SET_RAMP_1700U           = 0x06,
    SX1261_SET_RAMP_3400U           = 0x07
} sx1261_ramp_time_t;

typedef enum {
    SX1261_STATUS_MODE_STBY_RC      = 0x20, /* 0x02 - bits 6:4 */
    SX1261_STATUS_MODE_STBY_XOSC    = 0x30, /* 0x03 - bits 6:4 */
    SX1261_STATUS_MODE_FS           = 0x40, /* 0x04 - bits 6:4 */
    SX1261_STATUS_MODE_RX           = 0x50, /* 0x05 - bits 6:4 */
    SX1261_STATUS_MODE_TX           = 0x60  /* 0x06 - bits 6:4 */
} sx1261_status_mode_t;

typedef enum {
    SX1261_STATUS_READY             = 0x02, /* 0x02 - bits 3:1 */
    SX1261_STATUS_TIMEOUT           = 0x03, /* 0x03 - bits 3:1 */
    SX1261_STATUS_PROCESSING_ERROR  = 0x04, /* 0x04 - bits 3:1 */
    SX1261_STATUS_EXECUTION_FAILED  = 0x05, /* 0x05 - bits 3:1 */
    SX1261_STATUS_TX_DONE           = 0x06  /* 0x06 - bits 3:1 */
} sx1261_status_command_status_t;

#endif

/* --- EOF ------------------------------------------------------------------ */
