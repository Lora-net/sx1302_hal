/*
 / _____)             _              | |
( (____  _____ ____ _| |_ _____  ____| |__
 \____ \| ___ |    (_   _) ___ |/ ___)  _ \
 _____) ) ____| | | || |_| ____( (___| | | |
(______/|_____)_|_|_| \__)_____)\____)_| |_|
  (C)2019 Semtech

Description:
    Host specific functions to address the LoRa concentrator registers through
    a SPI interface.
    Single-byte read/write and burst read/write.
    Could be used with multiple SPI ports in parallel (explicit file descriptor)

License: Revised BSD License, see LICENSE.TXT file include in the project
*/


#ifndef _LORAGW_SPI_H
#define _LORAGW_SPI_H

/* -------------------------------------------------------------------------- */
/* --- DEPENDANCIES --------------------------------------------------------- */

#include <stdint.h>        /* C99 types*/

#include "config.h"    /* library configuration options (dynamically generated) */

/* -------------------------------------------------------------------------- */
/* --- PUBLIC CONSTANTS ----------------------------------------------------- */

#define LGW_SPI_SUCCESS     0
#define LGW_SPI_ERROR       -1

#define SPI_SPEED       2000000

/* -------------------------------------------------------------------------- */
/* --- PUBLIC FUNCTIONS PROTOTYPES ------------------------------------------ */

/**
@brief LoRa concentrator SPI setup (configure I/O and peripherals)
@param com_path path to the SPI device to be used to connect to the SX1302
@param spi_target_ptr pointer on a generic pointer to SPI target (implementation dependant)
@return status of register operation (LGW_SPI_SUCCESS/LGW_SPI_ERROR)
*/

int lgw_spi_open(const char * com_path, void **com_target_ptr);

/**
@brief LoRa concentrator SPI close
@param spi_target generic pointer to SPI target (implementation dependant)
@return status of register operation (LGW_SPI_SUCCESS/LGW_SPI_ERROR)
*/

int lgw_spi_close(void *com_target);

/**
@brief LoRa concentrator SPI single-byte write
@param spi_target generic pointer to SPI target (implementation dependant)
@param address 7-bit register address
@param data data byte to write
@return status of register operation (LGW_SPI_SUCCESS/LGW_SPI_ERROR)
*/
int lgw_spi_w(void *com_target, uint8_t spi_mux_target, uint16_t address, uint8_t data);

/**
@brief LoRa concentrator SPI single-byte read
@param spi_target generic pointer to SPI target (implementation dependant)
@param address 7-bit register address
@param data data byte to write
@return status of register operation (LGW_SPI_SUCCESS/LGW_SPI_ERROR)
*/
int lgw_spi_r(void *com_target, uint8_t spi_mux_target, uint16_t address, uint8_t *data);

/**
@brief LoRa concentrator SPI single-byte read-modify-write
@param spi_target generic pointer to SPI target (implementation dependant)
@param address 7-bit register address
@param offs start offset of the bits to be modified
@param leng number of bits to be modified
@param data value to be written in the selected bits
@return status of register operation (LGW_SPI_SUCCESS/LGW_SPI_ERROR)
*/
int lgw_spi_rmw(void *com_target, uint8_t spi_mux_target, uint16_t address, uint8_t offs, uint8_t leng, uint8_t data);

/**
@brief LoRa concentrator SPI burst (multiple-byte) write
@param spi_target generic pointer to SPI target (implementation dependant)
@param address 7-bit register address
@param data pointer to byte array that will be sent to the LoRa concentrator
@param size size of the transfer, in byte(s)
@return status of register operation (LGW_SPI_SUCCESS/LGW_SPI_ERROR)
*/
int lgw_spi_wb(void *com_target, uint8_t spi_mux_target, uint16_t address, const uint8_t *data, uint16_t size);

/**
@brief LoRa concentrator SPI burst (multiple-byte) read
@param spi_target generic pointer to SPI target (implementation dependant)
@param address 7-bit register address
@param data pointer to byte array that will be written from the LoRa concentrator
@param size size of the transfer, in byte(s)
@return status of register operation (LGW_SPI_SUCCESS/LGW_SPI_ERROR)
*/
int lgw_spi_rb(void *com_target, uint8_t spi_mux_target, uint16_t address, uint8_t *data, uint16_t size);

/**
 *
 **/
uint16_t lgw_spi_chunk_size(void);

#endif

/* --- EOF ------------------------------------------------------------------ */
