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

#include <sys/ioctl.h>
#include <linux/i2c.h>
#include <linux/i2c-dev.h>
#include <unistd.h> /* usleep */

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

    /* Check Input Params */
    if (i2c_fd <= 0) {
        printf("ERROR: invalid I2C file descriptor\n");
        return LGW_I2C_ERROR;
    }

    err = sht3c_wakeup(i2c_fd, i2c_addr);
    err = sht3c_sleep(i2c_fd, i2c_addr);

    return err;
}

/* ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ */
int sht_get_temperature(int i2c_fd, uint8_t i2c_addr, float * temperature, float * humidity) {
	int err;
	#define BUFFER_SIZE 6
    uint8_t buf[BUFFER_SIZE] = {0};

    static uint8_t READTEMP_REG = 0x78; /* Reg Addr of Temp Data */

    /* Check Input Params */
    if (i2c_fd <= 0) {
        printf("ERROR: invalid I2C file descriptor\n");
        return LGW_I2C_ERROR;
    }

    (void)sht3c_wakeup(i2c_fd, i2c_addr);

    usleep(250000); /* Just wait instead of using clock stretching */

    /* Send measure command */
    err = i2c_linuxdev_write(i2c_fd, i2c_addr, READTEMP_REG, 0x66); /* ..., ..., Address, Command */
    if (err < 0) {
        printf("ERROR: failed to request temperature from I2C device 0x%02X (err=%i) \n", i2c_addr, err);
        return LGW_I2C_ERROR;
    }

    usleep(250000); /* Just wait instead of using clock stretching */

    /* Setup request structure */
    struct i2c_rdwr_ioctl_data packets;
    struct i2c_msg messages[2];

    messages[0].addr = i2c_addr;
    messages[0].flags= 0;
    messages[0].len = 1;
    messages[0].buf = &READTEMP_REG;

    messages[1].addr = i2c_addr;
    messages[1].flags = I2C_M_RD;
    messages[1].len = BUFFER_SIZE; /* Number of bytes to receive from I2C device. */
    messages[1].buf = buf;

    packets.msgs = messages;
    packets.nmsgs = 2;

    /* Send request packet */
    err = ioctl(i2c_fd, I2C_RDWR, &packets);
    if (err < 0) {
        DEBUG_PRINTF("ERROR: Read from I2C Device failed (%d, 0x%02x, 0x%02x) - %s\n", i2c_fd, device_addr, reg_addr, strerror(err));
        return LGW_I2C_ERROR;
    }

    if ( (buf[0] == 0x00) && (buf[1] == 0x00) && (buf[3] == 0x00) && (buf[4] == 0x00) ) return LGW_I2C_ERROR;

    /* Read Temperature */
    {
    	/* measurement commands return T (CRC) RH (CRC) */
		uint8_t high_byte, low_byte;
		uint16_t words[2];

		high_byte = (uint8_t)buf[0];
		low_byte = buf[1];
		words[0] = ((uint16_t)high_byte << 8) | low_byte;
		*temperature = ( ((21875 * (int32_t)words[0]) >> 13) - 45000 );
		*temperature = *temperature / 1000; //Keep on own line to decimal
		DEBUG_PRINTF("Temperature: %f C (h:0x%02X l:0x%02X)\n", *temperature, high_byte, low_byte);

		high_byte = (uint8_t)buf[3];
		low_byte = buf[4];
		words[1] = ((uint16_t)high_byte << 8) | low_byte;
		*humidity = (12500 * (int32_t)words[1]) >> 13; //Not used for loragw
		*humidity = *humidity / 1000; //Keep on own line to decimal
		DEBUG_PRINTF("Humidity: %f %% (h:0x%02X l:0x%02X)\n", humidity, high_byte, low_byte);

    }

    (void)sht3c_sleep(i2c_fd, i2c_addr);

    return LGW_I2C_SUCCESS;
}

/* --- EOF ------------------------------------------------------------------ */
