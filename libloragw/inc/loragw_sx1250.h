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


#ifndef _LORAGW_SX1250_H
#define _LORAGW_SX1250_H

/* -------------------------------------------------------------------------- */
/* --- DEPENDANCIES --------------------------------------------------------- */

#include <stdint.h>     /* C99 types*/
#include <stdbool.h>    /* bool type */

#include "sx1250_defs.h"

#include "config.h"     /* library configuration options (dynamically generated) */

/* -------------------------------------------------------------------------- */
/* --- PUBLIC MACROS -------------------------------------------------------- */

/* -------------------------------------------------------------------------- */
/* --- PUBLIC CONSTANTS ----------------------------------------------------- */

/* -------------------------------------------------------------------------- */
/* --- PUBLIC TYPES --------------------------------------------------------- */

/* -------------------------------------------------------------------------- */
/* --- PUBLIC FUNCTIONS PROTOTYPES ------------------------------------------ */

int sx1250_calibrate(uint8_t rf_chain, uint32_t freq_hz);
int sx1250_setup(uint8_t rf_chain, uint32_t freq_hz, bool single_input_mode);

int sx1250_reg_w(sx1250_op_code_t op_code, uint8_t *data, uint16_t size, uint8_t rf_chain);
int sx1250_reg_r(sx1250_op_code_t op_code, uint8_t *data, uint16_t size, uint8_t rf_chain);

#endif

/* --- EOF ------------------------------------------------------------------ */
