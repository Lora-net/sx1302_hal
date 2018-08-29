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


#ifndef _LORAGW_SX1302_H
#define _LORAGW_SX1302_H

/* -------------------------------------------------------------------------- */
/* --- DEPENDANCIES --------------------------------------------------------- */

#include <stdint.h>     /* C99 types*/

#include "config.h"     /* library configuration options (dynamically generated) */

/* -------------------------------------------------------------------------- */
/* --- PUBLIC CONSTANTS ----------------------------------------------------- */

/* -------------------------------------------------------------------------- */
/* --- PUBLIC MACROS -------------------------------------------------------- */

#define REG_SELECT(rf_chain, a, b) ((rf_chain == 0) ? a : b)

/* -------------------------------------------------------------------------- */
/* --- PUBLIC TYPES --------------------------------------------------------- */

typedef enum {
    SX1302_RADIO_TYPE_SX1250,   /* sx1250 */
    SX1302_RADIO_TYPE_SX125X    /* sx1255/1257 */
} sx1302_radio_type_t;

/* -------------------------------------------------------------------------- */
/* --- PUBLIC FUNCTIONS PROTOTYPES ------------------------------------------ */

int sx1302_radio_clock_select(uint8_t rf_chain);
int sx1302_radio_reset(uint8_t rf_chain, sx1302_radio_type_t type);
int sx1302_radio_set_mode(uint8_t rf_chain, sx1302_radio_type_t type);

int sx1302_clock_enable(void);

int sx1302_channelizer_configure();
int sx1302_correlator_configure();
int sx1302_modem_configure();
int sx1302_modem_enable();
int sx1302_agc_configure();

int sx1302_lora_syncword();

#endif

/* --- EOF ------------------------------------------------------------------ */
