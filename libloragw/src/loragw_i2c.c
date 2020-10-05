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


/* -------------------------------------------------------------------------- */
/* --- DEPENDANCIES --------------------------------------------------------- */

#include <stdint.h>     /* C99 types */
#include <stdio.h>      /* printf fprintf */
#include <unistd.h>     /* lseek, close */
#include <fcntl.h>      /* open */
#include <errno.h>      /* errno */

#include <sys/ioctl.h>
#include <linux/i2c.h>
#include <linux/i2c-dev.h>

#include "loragw_i2c.h"
#include "loragw_aux.h"

/* -------------------------------------------------------------------------- */
/* --- PRIVATE MACROS ------------------------------------------------------- */

#define ARRAY_SIZE(a) (sizeof(a) / sizeof((a)[0]))
#if DEBUG_I2C == 1
    #define DEBUG_MSG(str)                fprintf(stdout, str)
    #define DEBUG_PRINTF(fmt, args...)    fprintf(stdout,"%s:%d: "fmt, __FUNCTION__, __LINE__, args)
    #define CHECK_NULL(a)                if(a==NULL){fprintf(stderr,"%s:%d: ERROR: NULL POINTER AS ARGUMENT\n", __FUNCTION__, __LINE__);return LGW_I2C_ERROR;}
#else
    #define DEBUG_MSG(str)
    #define DEBUG_PRINTF(fmt, args...)
    #define CHECK_NULL(a)                if(a==NULL){return LGW_I2C_ERROR;}
#endif

/* -------------------------------------------------------------------------- */
/* --- PRIVATE CONSTANTS ---------------------------------------------------- */

/* -------------------------------------------------------------------------- */
/* --- PUBLIC FUNCTIONS DEFINITION ------------------------------------------ */

int i2c_linuxdev_open(const char *path, uint8_t device_addr, int *i2c_fd) {
    int dev;

    /* Check input variables */
    if (path == NULL) {
        DEBUG_MSG("ERROR: null pointer path\n");
        return LGW_I2C_ERROR;
    }
    if (i2c_fd == NULL) {
        DEBUG_MSG("ERROR: null pointer i2c_fd\n");
        return LGW_I2C_ERROR;
    }

    /* Open I2C device */
    dev = open(path, O_RDWR);
    if (dev < 0) {
        DEBUG_PRINTF("ERROR: Failed to open I2C %s - %s\n", path, strerror(errno));
        return LGW_I2C_ERROR;
    }

    /* Setting I2C device mode to slave */
    if (ioctl(dev, I2C_SLAVE, device_addr) < 0) {
        DEBUG_PRINTF("ERROR: Failed to acquire bus access and/or talk to slave - %s\n", strerror(errno));
        return LGW_I2C_ERROR;
    }

    DEBUG_PRINTF("INFO: I2C port opened successfully (%s, 0x%02X)\n", path, device_addr);
    *i2c_fd = dev; /* return file descriptor index */

    return LGW_I2C_SUCCESS;
}

/* ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ */

int i2c_linuxdev_read(int i2c_fd, uint8_t device_addr, uint8_t reg_addr, uint8_t *data) {
    uint8_t *inbuff, outbuff;
    struct i2c_rdwr_ioctl_data packets;
    struct i2c_msg messages[2];

    outbuff = reg_addr;
    messages[0].addr = device_addr;
    messages[0].flags= 0;
    messages[0].len = sizeof(outbuff);
    messages[0].buf = &outbuff;

    inbuff = data;
    messages[1].addr = device_addr;
    messages[1].flags = I2C_M_RD;
    messages[1].len = sizeof(*inbuff);
    messages[1].buf = inbuff;

    packets.msgs = messages;
    packets.nmsgs = 2;

    if (ioctl(i2c_fd, I2C_RDWR, &packets) < 0) {
        DEBUG_PRINTF("ERROR: Read from I2C Device failed (%d, 0x%02x, 0x%02x) - %s\n", i2c_fd, device_addr, reg_addr, strerror(errno));
        return LGW_I2C_ERROR;
    }

    return LGW_I2C_SUCCESS;
}

/* ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ */

int i2c_linuxdev_write(int i2c_fd, uint8_t device_addr, uint8_t reg_addr, uint8_t data) {
    unsigned char buff[2];
    struct i2c_rdwr_ioctl_data packets;
    struct i2c_msg messages[1];

    buff[0] = reg_addr;
    buff[1] = data;

    messages[0].addr = device_addr;
    messages[0].flags = 0;
    messages[0].len = sizeof(buff);
    messages[0].buf = buff;

    packets.msgs = messages;
    packets.nmsgs = 1;

    if (ioctl(i2c_fd, I2C_RDWR, &packets) < 0) {
        DEBUG_PRINTF("ERROR: Write to I2C Device failed (%d, 0x%02x, 0x%02x) - %s\n", i2c_fd, device_addr, reg_addr, strerror(errno));
        return LGW_I2C_ERROR;
    }

    return LGW_I2C_SUCCESS;
}

/* ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ */

int i2c_linuxdev_write_buffer(int i2c_fd, uint8_t device_addr, uint8_t *buffer, uint8_t size) {
    struct i2c_rdwr_ioctl_data packets;
    struct i2c_msg messages[1];

    /* Check input parameters */
    CHECK_NULL(buffer);

    messages[0].addr = device_addr;
    messages[0].flags = 0;
    messages[0].len = size;
    messages[0].buf = buffer;

    packets.msgs = messages;
    packets.nmsgs = 1;

    if (ioctl(i2c_fd, I2C_RDWR, &packets) < 0) {
        DEBUG_PRINTF("ERROR: Write buffer to I2C Device failed (%d, 0x%02x) - %s\n", i2c_fd, device_addr, strerror(errno));
        return LGW_I2C_ERROR;
    }

    return LGW_I2C_SUCCESS;
}

/* ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ */

int i2c_linuxdev_close(int i2c_fd) {
    int i;

    i = close(i2c_fd);
    if (i == 0) {
        DEBUG_MSG("INFO: I2C port closed successfully\n");
        return LGW_I2C_SUCCESS;
    } else {
        DEBUG_PRINTF("ERROR: Failed to close I2C - %s\n", strerror(errno));
        return LGW_I2C_ERROR;
    }
}

/* --- EOF ------------------------------------------------------------------ */
