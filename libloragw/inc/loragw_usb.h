/*
 / _____)             _              | |
( (____  _____ ____ _| |_ _____  ____| |__
 \____ \| ___ |    (_   _) ___ |/ ___)  _ \
 _____) ) ____| | | || |_| ____( (___| | | |
(______/|_____)_|_|_| \__)_____)\____)_| |_|
  (C)2020 Semtech

Description:
    Host specific functions to address the LoRa concentrator registers through
    a USB interface.
    Single-byte read/write and burst read/write.

License: Revised BSD License, see LICENSE.TXT file include in the project
*/


#ifndef _LORAGW_USB_H
#define _LORAGW_USB_H

/* -------------------------------------------------------------------------- */
/* --- DEPENDANCIES --------------------------------------------------------- */

#include <stdint.h>   /* C99 types*/

#include "loragw_com.h"

#include "config.h"   /* library configuration options (dynamically generated) */

/* -------------------------------------------------------------------------- */
/* --- PUBLIC CONSTANTS ----------------------------------------------------- */

#define LGW_USB_SUCCESS     0
#define LGW_USB_ERROR       -1

/* -------------------------------------------------------------------------- */
/* --- PUBLIC FUNCTIONS PROTOTYPES ------------------------------------------ */

/**
 *
*/

int lgw_usb_open(const char * com_path, void **com_target_ptr);

/**
 *
*/

int lgw_usb_close(void *com_target);

/**
 *
*/
int lgw_usb_w(void *com_target, uint8_t spi_mux_target, uint16_t address, uint8_t data);

/**
 *
*/
int lgw_usb_r(void *com_target, uint8_t spi_mux_target, uint16_t address, uint8_t *data);

/**
 *
*/
int lgw_usb_wb(void *com_target, uint8_t spi_mux_target, uint16_t address, const uint8_t *data, uint16_t size);

/**
 *
*/
int lgw_usb_rb(void *com_target, uint8_t spi_mux_target, uint16_t address, uint8_t *data, uint16_t size);

/**
 *
*/
int lgw_usb_rmw(void *com_target, uint16_t address, uint8_t offs, uint8_t leng, uint8_t data);

/**
 *
 **/
int lgw_usb_set_write_mode(lgw_com_write_mode_t write_mode);

/**
 *
 **/
int lgw_usb_flush(void *com_target);

/**
 *
 **/
uint16_t lgw_usb_chunk_size(void);

/**
 *
 **/
int lgw_usb_get_temperature(void *com_target, float * temperature);

#endif

/* --- EOF ------------------------------------------------------------------ */
