/*
 / _____)             _              | |
( (____  _____ ____ _| |_ _____  ____| |__
 \____ \| ___ |    (_   _) ___ |/ ___)  _ \
 _____) ) ____| | | || |_| ____( (___| | | |
(______/|_____)_|_|_| \__)_____)\____)_| |_|
  (C)2019 Semtech

Description:
    LoRa concentrator radio calibration functions

License: Revised BSD License, see LICENSE.TXT file include in the project
*/


#ifndef _LORAGW_CAL_H
#define _LORAGW_CAL_H

/* -------------------------------------------------------------------------- */
/* --- DEPENDANCIES --------------------------------------------------------- */

#include <stdint.h>     /* C99 types*/

#include "config.h"     /* library configuration options (dynamically generated) */

/* -------------------------------------------------------------------------- */
/* --- PUBLIC CONSTANTS ----------------------------------------------------- */

/* -------------------------------------------------------------------------- */
/* --- PUBLIC MACROS -------------------------------------------------------- */

/* -------------------------------------------------------------------------- */
/* --- PUBLIC TYPES --------------------------------------------------------- */

struct lgw_sx125x_cal_rx_result_s {
    int8_t amp;
    int8_t phi;
    uint16_t rej;
    uint16_t rej_init;
    uint16_t snr;
};

struct lgw_sx125x_cal_tx_result_s {
    uint8_t dac_gain;
    uint8_t mix_gain;
    int8_t offset_i;
    int8_t offset_q;
    uint16_t rej;
    uint16_t sig;
};

/* -------------------------------------------------------------------------- */
/* --- PUBLIC FUNCTIONS PROTOTYPES ------------------------------------------ */

int sx1302_cal_start(uint8_t version, struct lgw_conf_rxrf_s * rf_chain_cfg, struct lgw_tx_gain_lut_s * txgain_lut);

#endif

/* --- EOF ------------------------------------------------------------------ */
