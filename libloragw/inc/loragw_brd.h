/*
 / _____)             _              | |
( (____  _____ ____ _| |_ _____  ____| |__
 \____ \| ___ |    (_   _) ___ |/ ___)  _ \
 _____) ) ____| | | || |_| ____( (___| | | |
(______/|_____)_|_|_| \__)_____)\____)_| |_|
  (C)2018 Semtech

Description:
    TODO

License: Revised BSD License, see LICENSE.TXT file include in the project
*/


#ifndef _LORAGW_BRD_H
#define _LORAGW_BRD_H

/* -------------------------------------------------------------------------- */
/* --- DEPENDANCIES --------------------------------------------------------- */

#include <stdint.h>     /* C99 types */
#include <stdbool.h>    /* bool type */

#include "config.h"     /* library configuration options (dynamically generated) */

/* -------------------------------------------------------------------------- */
/* --- PUBLIC TYPES --------------------------------------------------------- */

typedef struct {
    uint8_t rf_clkout;
    bool    full_duplex;
} lgw_rf_board_cfg_t;

typedef struct {
    bool                rf_enable;
    uint32_t            rf_rx_freq; /* absolute, in Hz */
    float               rf_rssi_offset;
    bool                rf_tx_enable;
    lgw_radio_type_t    rf_radio_type;
} lgw_rf_chain_cfg_t;

#endif

/* --- EOF ------------------------------------------------------------------ */
