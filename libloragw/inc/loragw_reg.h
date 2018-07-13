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


#ifndef _LORAGW_REG_H
#define _LORAGW_REG_H

/* -------------------------------------------------------------------------- */
/* --- DEPENDANCIES --------------------------------------------------------- */

#include <stdint.h>     /* C99 types */
#include <stdbool.h>    /* bool type */

#include "config.h"     /* library configuration options (dynamically generated) */

/* -------------------------------------------------------------------------- */
/* --- INTERNAL SHARED TYPES ------------------------------------------------ */

struct lgw_reg_s {
    int8_t   page;        /*!< page containing the register (-1 for all pages) */
    uint16_t addr;        /*!< base address of the register (15 bit) */
    uint8_t  offs;        /*!< position of the register LSB (between 0 to 7) */
    bool     sign;        /*!< 1 indicates the register is signed (2 complem.) */
    uint8_t  leng;        /*!< number of bits in the register */
    bool     rdon;        /*!< 1 indicates a read-only register */
    int32_t  dflt;        /*!< register default value */
};

/* -------------------------------------------------------------------------- */
/* --- INTERNAL SHARED FUNCTIONS -------------------------------------------- */

int reg_w_align32(void *spi_target, uint8_t spi_mux_target, struct lgw_reg_s r, int32_t reg_value);
int reg_r_align32(void *spi_target, uint8_t spi_mux_target, struct lgw_reg_s r, int32_t *reg_value);

/* -------------------------------------------------------------------------- */
/* --- PUBLIC CONSTANTS ----------------------------------------------------- */

#define LGW_REG_SUCCESS  0
#define LGW_REG_ERROR    -1

#define LGW_VERSION 0
#define LGW_AGCMCU__RF_EN_A__RADIO_EN 1
#define LGW_AGCMCU__RF_EN_A__RADIO_RST 2

#define LGW_TOTALREGS 3

/* -------------------------------------------------------------------------- */
/* --- PUBLIC FUNCTIONS PROTOTYPES ------------------------------------------ */

/**
@brief Connect LoRa concentrator by opening SPI link
@return status of register operation (LGW_REG_SUCCESS/LGW_REG_ERROR)
*/
int lgw_connect(void);

/**
@brief Disconnect LoRa concentrator by closing SPI link
@return status of register operation (LGW_REG_SUCCESS/LGW_REG_ERROR)
*/
int lgw_disconnect(void);

/**
@brief LoRa concentrator register write
@param register_id register number in the data structure describing registers
@param reg_value signed value to write to the register (for u32, use cast)
@return status of register operation (LGW_REG_SUCCESS/LGW_REG_ERROR)
*/
int lgw_reg_w(uint16_t register_id, int32_t reg_value);

/**
@brief LoRa concentrator register read
@param register_id register number in the data structure describing registers
@param reg_value pointer to a variable where to write register read value
@return status of register operation (LGW_REG_SUCCESS/LGW_REG_ERROR)
*/
int lgw_reg_r(uint16_t register_id, int32_t *reg_value);

/**
@brief LoRa concentrator register burst write
@param register_id register number in the data structure describing registers
@param data pointer to byte array that will be sent to the LoRa concentrator
@param size size of the transfer, in byte(s)
@return status of register operation (LGW_REG_SUCCESS/LGW_REG_ERROR)
*/
int lgw_reg_wb(uint16_t register_id, uint8_t *data, uint16_t size);

/**
@brief LoRa concentrator register burst read
@param register_id register number in the data structure describing registers
@param data pointer to byte array that will be written from the LoRa concentrator
@param size size of the transfer, in byte(s)
@return status of register operation (LGW_REG_SUCCESS/LGW_REG_ERROR)
*/
int lgw_reg_rb(uint16_t register_id, uint8_t *data, uint16_t size);


#endif

/* --- EOF ------------------------------------------------------------------ */
