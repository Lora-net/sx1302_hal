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


#ifndef _SX1250_COM_H
#define _SX1250_COM_H

/* -------------------------------------------------------------------------- */
/* --- DEPENDANCIES --------------------------------------------------------- */

#include <stdint.h>     /* C99 types*/

#include "loragw_com.h"
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

int sx1250_com_w(lgw_com_type_t com_type, void *com_target, uint8_t spi_mux_target, sx1250_op_code_t op_code, uint8_t *data, uint16_t size);
int sx1250_com_r(lgw_com_type_t com_type, void *com_target, uint8_t spi_mux_target, sx1250_op_code_t op_code, uint8_t *data, uint16_t size);

#endif

/* --- EOF ------------------------------------------------------------------ */
