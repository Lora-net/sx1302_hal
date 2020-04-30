/*
 / _____)             _              | |
( (____  _____ ____ _| |_ _____  ____| |__
 \____ \| ___ |    (_   _) ___ |/ ___)  _ \
 _____) ) ____| | | || |_| ____( (___| | | |
(______/|_____)_|_|_| \__)_____)\____)_| |_|
  (C)2019 Semtech

Description:
    Functions used to handle LoRa concentrator SX1250 radios.

License: Revised BSD License, see LICENSE.TXT file include in the project
*/


#ifndef _SX1250_DEFS_H
#define _SX1250_DEFS_H

/* -------------------------------------------------------------------------- */
/* --- DEPENDANCIES --------------------------------------------------------- */

#include <stdint.h>     /* C99 types*/

#include "config.h"     /* library configuration options (dynamically generated) */

/* -------------------------------------------------------------------------- */
/* --- PUBLIC MACROS -------------------------------------------------------- */

#define SX1250_FREQ_TO_REG(f) (uint32_t)((uint64_t)f * (1 << 25) / 32000000U)

/* -------------------------------------------------------------------------- */
/* --- PUBLIC CONSTANTS ----------------------------------------------------- */

/* -------------------------------------------------------------------------- */
/* --- PUBLIC TYPES --------------------------------------------------------- */

typedef enum {
    CALIBRATE               = 0x89,
    CALIBRATE_IMAGE         = 0x98,
    CLR_IRQ_STATUS          = 0x02,
    STOP_TIMER_ON_PREAMBLE  = 0x9F,
    SET_RFSWITCHMODE        = 0x9D,
    GET_IRQ_STATUS          = 0x12,
    GET_RX_BUFFER_STATUS    = 0x13,
    GET_PACKET_STATUS       = 0x14,
    READ_BUFFER             = 0x1E,
    READ_REGISTER           = 0x1D,
    SET_DIO_IRQ_PARAMS      = 0x08,
    SET_MODULATION_PARAMS   = 0x8B,
    SET_PA_CONFIG           = 0x95,
    SET_PACKET_PARAMS       = 0x8C,
    SET_PACKET_TYPE         = 0x8A,
    SET_RF_FREQUENCY        = 0x86,
    SET_BUFFER_BASE_ADDRESS = 0x8F,
    SET_SLEEP               = 0x84,
    SET_STANDBY             = 0x80,
    SET_RX                  = 0x82,
    SET_TX                  = 0x83,
    SET_TX_PARAMS           = 0x8E,
    WRITE_BUFFER            = 0x0E,
    WRITE_REGISTER          = 0x0D,
    SET_TXCONTINUOUSWAVE    = 0xD1,
    SET_TXCONTINUOUSPREAMBLE= 0xD2,
    GET_STATUS              = 0xC0,
    SET_REGULATORMODE       = 0x96,
    SET_FS                  = 0xC1,
    GET_DEVICE_ERRORS       = 0x17
} sx1250_op_code_t;

typedef enum {
    STDBY_RC                = 0x00,
    STDBY_XOSC              = 0x01
} sx1250_standby_modes_t;

typedef enum {
    PACKET_TYPE_GFSK        = 0x00,
    PACKET_TYPE_LORA        = 0x01
} sx1250_packet_type_t;

typedef enum {
    SET_RAMP_10U            = 0x00,
    SET_RAMP_20U            = 0x01,
    SET_RAMP_40U            = 0x02,
    SET_RAMP_80U            = 0x03,
    SET_RAMP_200U           = 0x04,
    SET_RAMP_800U           = 0x05,
    SET_RAMP_1700U          = 0x06,
    SET_RAMP_3400U          = 0x07
} sx1250_ramp_time_t;

#endif

/* --- EOF ------------------------------------------------------------------ */
