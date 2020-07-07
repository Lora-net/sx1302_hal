/*
 / _____)             _              | |
( (____  _____ ____ _| |_ _____  ____| |__
 \____ \| ___ |    (_   _) ___ |/ ___)  _ \
 _____) ) ____| | | || |_| ____( (___| | | |
(______/|_____)_|_|_| \__)_____)\____)_| |_|
  (C)2019 Semtech

Description:
    Functions used to handle the sx1261 radio used for LBT/Spectral Scan.

License: Revised BSD License, see LICENSE.TXT file include in the project
*/


#ifndef _SX1261_COM_H
#define _SX1261_COM_H

/* -------------------------------------------------------------------------- */
/* --- DEPENDANCIES --------------------------------------------------------- */

#include <stdint.h>     /* C99 types*/

#include "loragw_com.h"
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

/**
 *
*/
int sx1261_com_open(lgw_com_type_t com_type, const char *com_path);

/**
 *
*/
int sx1261_com_close(void);

/**
 *
*/
int sx1261_com_w(sx1261_op_code_t op_code, uint8_t *data, uint16_t size);

/**
 *
*/
int sx1261_com_r(sx1261_op_code_t op_code, uint8_t *data, uint16_t size);

/**
 *
*/
int sx1261_com_set_write_mode(lgw_com_write_mode_t write_mode);

/**
 *
*/
int sx1261_com_flush(void);

#endif

/* --- EOF ------------------------------------------------------------------ */
