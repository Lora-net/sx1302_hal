/*
 / _____)             _              | |
( (____  _____ ____ _| |_ _____  ____| |__
 \____ \| ___ |    (_   _) ___ |/ ___)  _ \
 _____) ) ____| | | || |_| ____( (___| | | |
(______/|_____)_|_|_| \__)_____)\____)_| |_|
  (C)2020 Semtech

Description:
    Functions to abstract the communication interface used to communicate with
    the concentrator.
    Single-byte read/write and burst read/write.

License: Revised BSD License, see LICENSE.TXT file include in the project
*/


#ifndef _LORAGW_COM_H
#define _LORAGW_COM_H

/* -------------------------------------------------------------------------- */
/* --- DEPENDANCIES --------------------------------------------------------- */

#include <stdint.h>   /* C99 types*/

#include "config.h"   /* library configuration options (dynamically generated) */

/* -------------------------------------------------------------------------- */
/* --- PUBLIC CONSTANTS ----------------------------------------------------- */

#define LGW_COM_SUCCESS     0
#define LGW_COM_ERROR       -1

#define LGW_SPI_MUX_TARGET_SX1302   0x00
#define LGW_SPI_MUX_TARGET_RADIOA   0x01
#define LGW_SPI_MUX_TARGET_RADIOB   0x02

/* -------------------------------------------------------------------------- */
/* --- PUBLIC FUNCTIONS PROTOTYPES ------------------------------------------ */

/**
 * 
*/

int lgw_com_open(const char *com_path, void **com_target_ptr);

/**
 * 
*/

int lgw_com_close(void *com_target);

/**
 *
*/
int lgw_com_w(void *com_target, uint8_t spi_mux_target, uint16_t address, uint8_t data);

/**
 *
*/
int lgw_com_r(void *com_target, uint8_t spi_mux_target, uint16_t address, uint8_t *data);

/**
 *
*/
int lgw_com_wb(void *com_target, uint8_t spi_mux_target, uint16_t address, const uint8_t *data, uint16_t size);

/**
 *
*/
int lgw_com_rb(void *com_target, uint8_t spi_mux_target, uint16_t address, uint8_t *data, uint16_t size);

#endif

/* --- EOF ------------------------------------------------------------------ */
