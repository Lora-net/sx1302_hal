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
/* --- PUBLIC TYPES --------------------------------------------------------- */

typedef enum com_type_e {
    LGW_COM_SPI,
    LGW_COM_USB,
    LGW_COM_UNKNOWN
} lgw_com_type_t;

typedef enum com_write_mode_e {
    LGW_COM_WRITE_MODE_SINGLE,
    LGW_COM_WRITE_MODE_BULK,
    LGW_COM_WRITE_MODE_UNKNOWN
} lgw_com_write_mode_t;

/* -------------------------------------------------------------------------- */
/* --- PUBLIC FUNCTIONS PROTOTYPES ------------------------------------------ */

/**
 *
*/
int lgw_com_open(lgw_com_type_t com_type, const char *com_path);

/**
 *
*/
int lgw_com_close(void);

/**
 *
*/
int lgw_com_w(uint8_t spi_mux_target, uint16_t address, uint8_t data);

/**
 *
*/
int lgw_com_r(uint8_t spi_mux_target, uint16_t address, uint8_t *data);

/**
 *
*/
int lgw_com_rmw(uint8_t spi_mux_target, uint16_t address, uint8_t offs, uint8_t leng, uint8_t data);

/**
 *
*/
int lgw_com_wb(uint8_t spi_mux_target, uint16_t address, const uint8_t *data, uint16_t size);

/**
 *
*/
int lgw_com_rb(uint8_t spi_mux_target, uint16_t address, uint8_t *data, uint16_t size);

/**
 *
*/
int lgw_com_set_write_mode(lgw_com_write_mode_t write_mode);

/**
 *
*/
int lgw_com_flush(void);

/**
 *
*/
uint16_t lgw_com_chunk_size(void);

/**
 *
 **/
int lgw_com_get_temperature(float * temperature);

/**
 *
 **/
void* lgw_com_target(void);

/**
 *
 **/
lgw_com_type_t lgw_com_type(void);

#endif

/* --- EOF ------------------------------------------------------------------ */
