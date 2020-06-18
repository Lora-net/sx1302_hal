/*
 / _____)             _              | |
( (____  _____ ____ _| |_ _____  ____| |__
 \____ \| ___ |    (_   _) ___ |/ ___)  _ \
 _____) ) ____| | | || |_| ____( (___| | | |
(______/|_____)_|_|_| \__)_____)\____)_| |_|
  (C)2019 Semtech

Description:
    Functions used to handle LoRa concentrator SX1261 radio used to handle LBT
    and Spectral Scan.

License: Revised BSD License, see LICENSE.TXT file include in the project
*/


#ifndef _LORAGW_SX1261_H
#define _LORAGW_SX1261_H

/* -------------------------------------------------------------------------- */
/* --- DEPENDANCIES --------------------------------------------------------- */

#include <stdint.h>     /* C99 types*/
#include <stdbool.h>    /* bool type */

#include "sx1261_defs.h"

#include "config.h"     /* library configuration options (dynamically generated) */

/* -------------------------------------------------------------------------- */
/* --- PUBLIC MACROS -------------------------------------------------------- */

/* -------------------------------------------------------------------------- */
/* --- PUBLIC CONSTANTS ----------------------------------------------------- */

/* -------------------------------------------------------------------------- */
/* --- PUBLIC TYPES --------------------------------------------------------- */

/* -------------------------------------------------------------------------- */
/* --- PUBLIC FUNCTIONS PROTOTYPES ------------------------------------------ */

int sx1261_connect(const char * spi_path);
int sx1261_disconnect(void);

int sx1261_reg_w(sx1261_op_code_t op_code, uint8_t *data, uint16_t size);
int sx1261_reg_r(sx1261_op_code_t op_code, uint8_t *data, uint16_t size);

int sx1261_load_pram(void);
int sx1261_setup(uint32_t freq_hz);

#endif

/* --- EOF ------------------------------------------------------------------ */
