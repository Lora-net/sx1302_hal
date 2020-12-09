/*
 / _____)             _              | |
( (____  _____ ____ _| |_ _____  ____| |__
 \____ \| ___ |    (_   _) ___ |/ ___)  _ \
 _____) ) ____| | | || |_| ____( (___| | | |
(______/|_____)_|_|_| \__)_____)\____)_| |_|
  (C)2020 Semtech

Description:
    Basic driver for Analog AD5338R DAC.

License: Revised BSD License, see LICENSE.TXT file include in the project
*/


#ifndef _LORAGW_AD5338R_H
#define _LORAGW_AD5338R_H

#include <stdint.h>     /* C99 types */
#include <stdbool.h>    /* bool type */

#include "config.h"     /* library configuration options (dynamically generated) */

/* -------------------------------------------------------------------------- */
/* --- PRIVATE MACROS ------------------------------------------------------- */

#define VOLTAGE2HEX_H(a) ( (a)*1024/5/4 )
#define VOLTAGE2HEX_L(a) ( (((int)((a)*1024/5)) & 0x3) * 64 )

/* -------------------------------------------------------------------------- */
/* --- PRIVATE CONSTANTS ---------------------------------------------------- */

#define I2C_PORT_DAC_AD5338R 0x0C

#define AD5338R_CMD_SIZE 3

/* -------------------------------------------------------------------------- */
/* --- PUBLIC FUNCTIONS ----------------------------------------------------- */

int ad5338r_configure(int i2c_fd, uint8_t i2c_addr);
int ad5338r_write(int i2c_fd, uint8_t i2c_addr, uint8_t buf[static AD5338R_CMD_SIZE]);

#endif
