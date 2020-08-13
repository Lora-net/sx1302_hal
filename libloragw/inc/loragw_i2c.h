/*
 / _____)             _              | |
( (____  _____ ____ _| |_ _____  ____| |__
 \____ \| ___ |    (_   _) ___ |/ ___)  _ \
 _____) ) ____| | | || |_| ____( (___| | | |
(______/|_____)_|_|_| \__)_____)\____)_| |_|
  (C)2019 Semtech

Description:
    Host specific functions to address the LoRa concentrator I2C peripherals.

License: Revised BSD License, see LICENSE.TXT file include in the project
*/


#ifndef _LORAGW_I2C_H
#define _LORAGW_I2C_H

/* -------------------------------------------------------------------------- */
/* --- DEPENDANCIES --------------------------------------------------------- */

#include <stdint.h>        /* C99 types*/

#include "config.h"    /* library configuration options (dynamically generated) */

/* -------------------------------------------------------------------------- */
/* --- PUBLIC CONSTANTS ----------------------------------------------------- */

#define LGW_I2C_SUCCESS     0
#define LGW_I2C_ERROR       -1

#define I2C_DEVICE          "/dev/i2c-1"

/* -------------------------------------------------------------------------- */
/* --- PUBLIC FUNCTIONS PROTOTYPES ------------------------------------------ */

/**
@brief Open I2C port
@param path         Path to the I2C device driver (absolute or relative)
@param device_addr  Address of the device
@param i2c_fd      Pointer to receive I2C port file descriptor index
@return 0 if I2C port was open successfully, -1 else
*/
int i2c_linuxdev_open(const char *path, uint8_t device_addr, int *i2c_fd);

/**
@brief Close I2C port
@param i2c_fd      I2C port file descriptor index
@return 0 if I2C port was closed successfully, -1 else
*/
int i2c_linuxdev_close(int i2c_fd);

/**
@brief Read data from an I2C port
@param i2c_fd      I2C port file descriptor index
@param device_addr  I2C device address
@param reg_addr     Address of the register to be read
@param data         Pointer to a buffer to store read data
@return 0 if I2C data read is successful, -1 else
*/
int i2c_linuxdev_read(int i2c_fd, uint8_t device_addr, uint8_t reg_addr, uint8_t *data);

/**
@brief Write data to an I2C port
@param i2c_fd      I2C port file descriptor index
@param device_addr  I2C device address
@param reg_addr     Address of the register to write to
@param data         byte to write in the register
@return 0 if I2C data write is successful, -1 else
*/
int i2c_linuxdev_write(int i2c_fd, uint8_t device_addr, uint8_t reg_addr, uint8_t data);

/**
@brief Write a raw buffer to an I2C port
@param i2c_fd       I2C port file descriptor index
@param device_addr  I2C device address
@param buffer       Buffer to be written to the device
@param size         Size of the buffer to be written
@return 0 if I2C data write is successful, -1 else
*/
int i2c_linuxdev_write_buffer(int i2c_fd, uint8_t device_addr, uint8_t *buffer, uint8_t size);

#endif

/* --- EOF ------------------------------------------------------------------ */
