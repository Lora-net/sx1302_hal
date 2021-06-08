/*
 / _____)             _              | |
( (____  _____ ____ _| |_ _____  ____| |__
 \____ \| ___ |    (_   _) ___ |/ ___)  _ \
 _____) ) ____| | | || |_| ____( (___| | | |
(______/|_____)_|_|_| \__)_____)\____)_| |_|
  (C)2019 Semtech

Description:
    Basic driver for SHT3x temperature sensor

License: Revised BSD License, see LICENSE.TXT file include in the project
*/


/* -------------------------------------------------------------------------- */
/* --- DEPENDANCIES --------------------------------------------------------- */

#include <stdint.h>     /* C99 types */
#include <stdbool.h>    /* bool type */
#include <stdio.h>      /* printf fprintf */

#include "loragw_i2c.h"
#include "loragw_sht.h"

/* -------------------------------------------------------------------------- */
/* --- PRIVATE MACROS ------------------------------------------------------- */

#define ARRAY_SIZE(a) (sizeof(a) / sizeof((a)[0]))
#if DEBUG_I2C == 1
    #define DEBUG_MSG(str)              fprintf(stdout, str)
    #define DEBUG_PRINTF(fmt, args...)  fprintf(stdout,"%s:%d: "fmt, __FUNCTION__, __LINE__, args)
    #define CHECK_NULL(a)               if(a==NULL){fprintf(stderr,"%s:%d: ERROR: NULL POINTER AS ARGUMENT\n", __FUNCTION__, __LINE__);return LGW_REG_ERROR;}
#else
    #define DEBUG_MSG(str)
    #define DEBUG_PRINTF(fmt, args...)
    #define CHECK_NULL(a)               if(a==NULL){return LGW_REG_ERROR;}
#endif

/* -------------------------------------------------------------------------- */
/* --- PRIVATE CONSTANTS ---------------------------------------------------- */

/* all measurement commands return T (CRC) RH (CRC) */
#define SHTC1_CMD_MEASURE_HPM 0x7866
#define SHTC1_CMD_MEASURE_LPM 0x609C

//static const uint16_t SHTC3_CMD_SLEEP = 0xB098;
//static const uint16_t SHTC3_CMD_WAKEUP = 0x3517;

#define CRC8_LEN 1
#define CRC8_POLYNOMIAL 0x31
#define CRC8_INIT 0xFF

#define SENSIRION_COMMAND_SIZE 2

#define SENSIRION_WORD_SIZE 2
#define SENSIRION_NUM_WORDS(x) (sizeof(x) / SENSIRION_WORD_SIZE)
#define SENSIRION_MAX_BUFFER_WORDS 32

#define NO_ERROR 0
/* deprecated defines, use NO_ERROR or custom error codes instead */
#define STATUS_OK 0
#define STATUS_FAIL (-1)

/* -------------------------------------------------------------------------- */
/* --- PRIVATE VARIABLES ---------------------------------------------------- */

/* -------------------------------------------------------------------------- */
/* --- PRIVATE FUNCTIONS ---------------------------------------------------- */

/* -------------------------------------------------------------------------- */
/* --- PUBLIC FUNCTIONS DEFINITION ------------------------------------------ */

int sht3c_wakeup(int i2c_fd, uint8_t i2c_addr) {
	int err;
    err = i2c_linuxdev_write(i2c_fd, i2c_addr, 0x35, 0x17); /* ..., ..., Address, Command */
    if (err != 0) {
        DEBUG_PRINTF("ERROR: failed to wake up I2C device 0x%02X (err=%i)\n", i2c_addr, err);
        return LGW_I2C_ERROR;
    }
    return LGW_I2C_SUCCESS;
}

int sht3c_sleep(int i2c_fd, uint8_t i2c_addr) {
	int err;
	err = i2c_linuxdev_write(i2c_fd, i2c_addr, 0xB0, 0x98); /* ..., ..., Address, Command */
    if (err != 0) {
        DEBUG_PRINTF("ERROR: failed to sleep I2C device 0x%02X (err=%i)\n", i2c_addr, err);
        return LGW_I2C_ERROR;
    }
    return LGW_I2C_SUCCESS;
}

/* ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ */

int sht_configure(int i2c_fd, uint8_t i2c_addr) {
    int err;
    //uint8_t val;

    /* Check Input Params */
    if (i2c_fd <= 0) {
        printf("ERROR: invalid I2C file descriptor\n");
        return LGW_I2C_ERROR;
    }

    err = sht3c_wakeup(i2c_fd, i2c_addr);
    //err = sht3c_sleep(i2c_fd, i2c_addr);

    return err;
}

/* ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ */

int sht_get_temperature(int i2c_fd, uint8_t i2c_addr, float * temperature) {
    int err;
    uint8_t high_byte, low_byte;
    int8_t h;
    uint16_t words[2];

    /* Check Input Params */
    if (i2c_fd <= 0) {
        printf("ERROR: invalid I2C file descriptor\n");
        return LGW_I2C_ERROR;
    }

    //(void)sht3c_wakeup(i2c_fd, i2c_addr);

    /* Send measure command */
    err = i2c_linuxdev_write(i2c_fd, i2c_addr, 0x78, 0x66); /* ..., ..., Address, Command */
    if (err != 0) {
        printf("ERROR: failed to request temperature from I2C device 0x%02X (err=%i)\n", i2c_addr, err);
        return LGW_I2C_ERROR;
    }

    /* Read Temperature LSB */
    while ( (err != 0) || (low_byte == 0x00) ) {
    	err = i2c_linuxdev_read(i2c_fd, i2c_addr, 0x78, &low_byte);
    }
	if (err != 0) {
		printf("ERROR: failed to read low byte of I2C device 0x%02X (err=%i) (byte=0x%02X)\n", i2c_addr, err, low_byte);
	    return LGW_I2C_ERROR;
	} else {
		printf("DEBUG: SUCCESSS read low byte of I2C device 0x%02X (err=%i) (byte=0x%02X)\n", i2c_addr, err, low_byte);
	}

    /* Read Temperature MSB */
    err = i2c_linuxdev_read(i2c_fd, i2c_addr, 0x78, &high_byte);
    if (err != 0) {
        printf("ERROR: failed to read high byte of I2C device 0x%02X (err=%i)\n", i2c_addr, err);
        return LGW_I2C_ERROR;
    } else {
    	printf("DEBUG: SUCCESSS read high byte of I2C device 0x%02X (err=%i)\n", i2c_addr, err);
    }

    h = (int8_t)high_byte;
    words[0] = ((h << 8) | low_byte);

    *temperature = ((21875 * (int32_t)words[0]) >> 13) - 45000;

    //*humidity = ((12500 * (int32_t)words[1]) >> 13); //Not used for loragw

    DEBUG_PRINTF("Temperature: %f C (h:0x%02X l:0x%02X)\n", *temperature, high_byte, low_byte);

    //(void)sht3c_sleep(i2c_fd, i2c_addr);

    return LGW_I2C_SUCCESS;
}

/* --- EOF ------------------------------------------------------------------ */
