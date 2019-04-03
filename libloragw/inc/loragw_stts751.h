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


#ifndef _LORAGW_STTS751_H
#define _LORAGW_STTS751_H

/* -------------------------------------------------------------------------- */
/* --- DEPENDANCIES --------------------------------------------------------- */

#include <stdint.h>     /* C99 types */
#include <stdbool.h>    /* bool type */

#include "config.h"     /* library configuration options (dynamically generated) */

/* -------------------------------------------------------------------------- */
/* --- INTERNAL SHARED TYPES ------------------------------------------------ */

/* -------------------------------------------------------------------------- */
/* --- INTERNAL SHARED FUNCTIONS -------------------------------------------- */

/* -------------------------------------------------------------------------- */
/* --- PUBLIC CONSTANTS ----------------------------------------------------- */

#define I2C_PORT_TEMP_SENSOR    0x39

/* -------------------------------------------------------------------------- */
/* --- PUBLIC FUNCTIONS ----------------------------------------------------- */

/**
@brief TODO
@param TODO
@return TODO
*/
int lgw_stts751_configure(void);

/**
@brief TODO
@param TODO
@return TODO
*/
int lgw_stts751_get_temperature(float * temperature);

#endif

/* --- EOF ------------------------------------------------------------------ */
