/*
 / _____)             _              | |
( (____  _____ ____ _| |_ _____  ____| |__
 \____ \| ___ |    (_   _) ___ |/ ___)  _ \
 _____) ) ____| | | || |_| ____( (___| | | |
(______/|_____)_|_|_| \__)_____)\____)_| |_|
  (C)2020 Semtech

Description:
    LoRa concentrator Listen-Before-Talk functions

License: Revised BSD License, see LICENSE.TXT file include in the project
*/


#ifndef _LORAGW_LBT_H
#define _LORAGW_LBT_H

/* -------------------------------------------------------------------------- */
/* --- DEPENDANCIES --------------------------------------------------------- */

#include <stdint.h>     /* C99 types */
#include <stdbool.h>    /* bool type */

#include "loragw_hal.h"

#include "config.h"     /* library configuration options (dynamically generated) */

/* -------------------------------------------------------------------------- */
/* --- PUBLIC MACROS -------------------------------------------------------- */

/* -------------------------------------------------------------------------- */
/* --- PUBLIC CONSTANTS ----------------------------------------------------- */

/* -------------------------------------------------------------------------- */
/* --- PUBLIC TYPES --------------------------------------------------------- */

/* -------------------------------------------------------------------------- */
/* --- PUBLIC FUNCTIONS PROTOTYPES ------------------------------------------ */

/**
@brief Configure the SX1261 and start LBT channel scanning
@param sx1261_context the sx1261 radio parameters to take into account for scanning
@param pkt description of the packet to be transmitted
@return 0 for success, -1 for failure
*/
int lgw_lbt_start(const struct lgw_conf_sx1261_s * sx1261_context, const struct lgw_pkt_tx_s * pkt);

/**
@brief Stop LBT scanning
@return 0 for success, -1 for failure
*/
int lgw_lbt_stop(void);

/**
@brief Check if packet was allowed to be transmitted or not
@param rf_chain the TX path on which TX was requested
@param tx_ok pointer to return if the packet was allowed to be transmitted or not.
@return 0 for success, -1 for failure
*/
int lgw_lbt_tx_status(uint8_t rf_chain, bool * tx_ok);

#endif

/* --- EOF ------------------------------------------------------------------ */
