/*
 / _____)             _              | |
( (____  _____ ____ _| |_ _____  ____| |__
 \____ \| ___ |    (_   _) ___ |/ ___)  _ \
 _____) ) ____| | | || |_| ____( (___| | | |
(______/|_____)_|_|_| \__)_____)\____)_| |_|
  (C)2019 Semtech

Description:
    Basic driver for ST ts751 temperature sensor

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

/*
  0x39: STTS751-0DP3F
  0x3B: STTS751-1DP3F
  0x38: STTS751-0DP3F on full duplex CN490 ref design
  */
static const uint8_t I2C_PORT_TEMP_SENSOR[] = {0x39, 0x3B, 0x38};

/* -------------------------------------------------------------------------- */
/* --- PUBLIC FUNCTIONS ----------------------------------------------------- */

/**
@brief Configure the temperature sensor (ST TS751)
@param i2c_fd file descriptor to access the sensor through I2C
@param i2c_addr the I2C device address of the sensor
@return LGW_I2C_ERROR if fails, LGW_I2C_SUCCESS otherwise
*/
int stts751_configure(int i2c_fd, uint8_t i2c_addr);

/**
@brief Get the temperature from the sensor
@param i2c_fd file descriptor to access the sensor through I2C
@param i2c_addr the I2C device address of the sensor
@param temperature pointer to store the temerature read from sensor
@return LGW_I2C_ERROR if fails, LGW_I2C_SUCCESS otherwise
*/
int stts751_get_temperature(int i2c_fd, uint8_t i2c_addr, float * temperature);

#endif

/* --- EOF ------------------------------------------------------------------ */
