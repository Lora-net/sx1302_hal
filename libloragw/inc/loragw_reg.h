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

#define SX1302_REG_COMMON_PAGE_PAGE 0
#define SX1302_REG_COMMON_CTRL0_CLK32_RIF_CTRL 1
#define SX1302_REG_COMMON_CTRL0_HOST_RADIO_CTRL 2
#define SX1302_REG_COMMON_CTRL0_RADIO_MISC_EN 3
#define SX1302_REG_COMMON_CTRL0_SX1261_MODE_RADIO_B 4
#define SX1302_REG_COMMON_CTRL0_SX1261_MODE_RADIO_A 5
#define SX1302_REG_COMMON_SPI_DIV_RATIO_SPI_HALF_PERIOD 6
#define SX1302_REG_COMMON_RADIO_SELECT_RADIO_SELECT 7
#define SX1302_REG_COMMON_GEN_GLOBAL_EN 8
#define SX1302_REG_COMMON_GEN_FSK_MODEM_ENABLE 9
#define SX1302_REG_COMMON_GEN_CONCENTRATOR_MODEM_ENABLE 10
#define SX1302_REG_COMMON_GEN_MBWSSF_MODEM_ENABLE 11
#define SX1302_REG_COMMON_VERSION_VERSION 12
#define SX1302_REG_AGC_MCU_CTRL_FORCE_HOST_FE_CTRL 13
#define SX1302_REG_AGC_MCU_CTRL_MCU_CLEAR 14
#define SX1302_REG_AGC_MCU_CTRL_HOST_PROG 15
#define SX1302_REG_AGC_MCU_MCU_AGC_STATUS_PARITY_ERROR 16
#define SX1302_REG_AGC_MCU_MCU_AGC_STATUS_MCU_AGC_STATUS 17
#define SX1302_REG_AGC_MCU_PA_GAIN_PA_B_GAIN 18
#define SX1302_REG_AGC_MCU_PA_GAIN_PA_A_GAIN 19
#define SX1302_REG_AGC_MCU_RF_EN_A_RADIO_RST 20
#define SX1302_REG_AGC_MCU_RF_EN_A_RADIO_EN 21
#define SX1302_REG_AGC_MCU_RF_EN_A_PA_EN 22
#define SX1302_REG_AGC_MCU_RF_EN_A_LNA_EN 23
#define SX1302_REG_AGC_MCU_RF_EN_B_RADIO_RST 24
#define SX1302_REG_AGC_MCU_RF_EN_B_RADIO_EN 25
#define SX1302_REG_AGC_MCU_RF_EN_B_PA_EN 26
#define SX1302_REG_AGC_MCU_RF_EN_B_LNA_EN 27
#define SX1302_REG_AGC_MCU_LUT_TABLE_A_PA_LUT 28
#define SX1302_REG_AGC_MCU_LUT_TABLE_A_LNA_LUT 29
#define SX1302_REG_AGC_MCU_LUT_TABLE_B_PA_LUT 30
#define SX1302_REG_AGC_MCU_LUT_TABLE_B_LNA_LUT 31
#define SX1302_REG_AGC_MCU_UART_CFG_MSBF 32
#define SX1302_REG_AGC_MCU_UART_CFG_PAR_EN 33
#define SX1302_REG_AGC_MCU_UART_CFG_PAR_MODE 34
#define SX1302_REG_AGC_MCU_UART_CFG_START_LEN 35
#define SX1302_REG_AGC_MCU_UART_CFG_STOP_LEN 36
#define SX1302_REG_AGC_MCU_UART_CFG_WORD_LEN 37
#define SX1302_REG_AGC_MCU_MCU_MAIL_BOX_WR_DATA_BYTE3_MCU_MAIL_BOX_WR_DATA 38
#define SX1302_REG_AGC_MCU_MCU_MAIL_BOX_WR_DATA_BYTE2_MCU_MAIL_BOX_WR_DATA 39
#define SX1302_REG_AGC_MCU_MCU_MAIL_BOX_WR_DATA_BYTE1_MCU_MAIL_BOX_WR_DATA 40
#define SX1302_REG_AGC_MCU_MCU_MAIL_BOX_WR_DATA_BYTE0_MCU_MAIL_BOX_WR_DATA 41
#define SX1302_REG_AGC_MCU_MCU_MAIL_BOX_RD_DATA_BYTE3_MCU_MAIL_BOX_RD_DATA 42
#define SX1302_REG_AGC_MCU_MCU_MAIL_BOX_RD_DATA_BYTE2_MCU_MAIL_BOX_RD_DATA 43
#define SX1302_REG_AGC_MCU_MCU_MAIL_BOX_RD_DATA_BYTE1_MCU_MAIL_BOX_RD_DATA 44
#define SX1302_REG_AGC_MCU_MCU_MAIL_BOX_RD_DATA_BYTE0_MCU_MAIL_BOX_RD_DATA 45

#define LGW_TOTALREGS 46

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
