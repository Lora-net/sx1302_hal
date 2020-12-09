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


/* -------------------------------------------------------------------------- */
/* --- DEPENDANCIES --------------------------------------------------------- */

#include <stdint.h>     /* C99 types */
#include <stdio.h>      /* printf fprintf */
#include <stdlib.h>     /* malloc free */
#include <unistd.h>     /* lseek, close */
#include <fcntl.h>      /* open */
#include <string.h>     /* memset */

#include <sys/ioctl.h>
#include <linux/spi/spidev.h>

#include "loragw_spi.h"
#include "loragw_aux.h"

/* -------------------------------------------------------------------------- */
/* --- PRIVATE MACROS ------------------------------------------------------- */

#define ARRAY_SIZE(a) (sizeof(a) / sizeof((a)[0]))
#if DEBUG_COM == 1
    #define DEBUG_MSG(str)                fprintf(stdout, str)
    #define DEBUG_PRINTF(fmt, args...)    fprintf(stdout,"%s:%d: "fmt, __FUNCTION__, __LINE__, args)
    #define CHECK_NULL(a)                if(a==NULL){fprintf(stderr,"%s:%d: ERROR: NULL POINTER AS ARGUMENT\n", __FUNCTION__, __LINE__);return LGW_SPI_ERROR;}
#else
    #define DEBUG_MSG(str)
    #define DEBUG_PRINTF(fmt, args...)
    #define CHECK_NULL(a)                if(a==NULL){return LGW_SPI_ERROR;}
#endif

/* -------------------------------------------------------------------------- */
/* --- PRIVATE CONSTANTS ---------------------------------------------------- */

#define READ_ACCESS     0x00
#define WRITE_ACCESS    0x80

#define LGW_BURST_CHUNK     1024

/* -------------------------------------------------------------------------- */
/* --- PUBLIC FUNCTIONS DEFINITION ------------------------------------------ */

/* SPI initialization and configuration */
int lgw_spi_open(const char * com_path, void **com_target_ptr) {
    int *spi_device = NULL;
    int dev;
    int a=0, b=0;
    int i;

    /* check input variables */
    CHECK_NULL(com_path);
    CHECK_NULL(com_target_ptr);

    /* allocate memory for the device descriptor */
    spi_device = malloc(sizeof(int));
    if (spi_device == NULL) {
        DEBUG_MSG("ERROR: MALLOC FAIL\n");
        return LGW_SPI_ERROR;
    }

    /* open SPI device */
    dev = open(com_path, O_RDWR);
    if (dev < 0) {
        DEBUG_PRINTF("ERROR: failed to open SPI device %s\n", com_path);
        return LGW_SPI_ERROR;
    }

    /* setting SPI mode to 'mode 0' */
    i = SPI_MODE_0;
    a = ioctl(dev, SPI_IOC_WR_MODE, &i);
    b = ioctl(dev, SPI_IOC_RD_MODE, &i);
    if ((a < 0) || (b < 0)) {
        DEBUG_MSG("ERROR: SPI PORT FAIL TO SET IN MODE 0\n");
        close(dev);
        free(spi_device);
        return LGW_SPI_ERROR;
    }

    /* setting SPI max clk (in Hz) */
    i = SPI_SPEED;
    a = ioctl(dev, SPI_IOC_WR_MAX_SPEED_HZ, &i);
    b = ioctl(dev, SPI_IOC_RD_MAX_SPEED_HZ, &i);
    if ((a < 0) || (b < 0)) {
        DEBUG_MSG("ERROR: SPI PORT FAIL TO SET MAX SPEED\n");
        close(dev);
        free(spi_device);
        return LGW_SPI_ERROR;
    }

    /* setting SPI to MSB first */
    i = 0;
    a = ioctl(dev, SPI_IOC_WR_LSB_FIRST, &i);
    b = ioctl(dev, SPI_IOC_RD_LSB_FIRST, &i);
    if ((a < 0) || (b < 0)) {
        DEBUG_MSG("ERROR: SPI PORT FAIL TO SET MSB FIRST\n");
        close(dev);
        free(spi_device);
        return LGW_SPI_ERROR;
    }

    /* setting SPI to 8 bits per word */
    i = 0;
    a = ioctl(dev, SPI_IOC_WR_BITS_PER_WORD, &i);
    b = ioctl(dev, SPI_IOC_RD_BITS_PER_WORD, &i);
    if ((a < 0) || (b < 0)) {
        DEBUG_MSG("ERROR: SPI PORT FAIL TO SET 8 BITS-PER-WORD\n");
        close(dev);
        return LGW_SPI_ERROR;
    }

    *spi_device = dev;
    *com_target_ptr = (void *)spi_device;
    DEBUG_MSG("Note: SPI port opened and configured ok\n");
    return LGW_SPI_SUCCESS;
}

/* ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ */

/* SPI release */
int lgw_spi_close(void *com_target) {
    int spi_device;
    int a;

    /* check input variables */
    CHECK_NULL(com_target);

    /* close file & deallocate file descriptor */
    spi_device = *(int *)com_target; /* must check that spi_target is not null beforehand */
    a = close(spi_device);
    free(com_target);

    /* determine return code */
    if (a < 0) {
        DEBUG_MSG("ERROR: SPI PORT FAILED TO CLOSE\n");
        return LGW_SPI_ERROR;
    } else {
        DEBUG_MSG("Note: SPI port closed\n");
        return LGW_SPI_SUCCESS;
    }
}

/* ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ */

/* Simple write */
int lgw_spi_w(void *com_target, uint8_t spi_mux_target, uint16_t address, uint8_t data) {
    int spi_device;
    uint8_t out_buf[4];
    uint8_t command_size;
    struct spi_ioc_transfer k;
    int a;

    /* check input variables */
    CHECK_NULL(com_target);

    spi_device = *(int *)com_target; /* must check that spi_target is not null beforehand */

    /* prepare frame to be sent */
    out_buf[0] = spi_mux_target;
    out_buf[1] = WRITE_ACCESS | ((address >> 8) & 0x7F);
    out_buf[2] =                ((address >> 0) & 0xFF);
    out_buf[3] = data;
    command_size = 4;

    /* I/O transaction */
    memset(&k, 0, sizeof(k)); /* clear k */
    k.tx_buf = (unsigned long) out_buf;
    k.len = command_size;
    k.speed_hz = SPI_SPEED;
    k.cs_change = 0;
    k.bits_per_word = 8;
    a = ioctl(spi_device, SPI_IOC_MESSAGE(1), &k);

    /* determine return code */
    if (a != (int)k.len) {
        DEBUG_MSG("ERROR: SPI WRITE FAILURE\n");
        return LGW_SPI_ERROR;
    } else {
        DEBUG_MSG("Note: SPI write success\n");
        return LGW_SPI_SUCCESS;
    }
}

/* ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ */

/* Simple read */
int lgw_spi_r(void *com_target, uint8_t spi_mux_target, uint16_t address, uint8_t *data) {
    int spi_device;
    uint8_t out_buf[5];
    uint8_t command_size;
    uint8_t in_buf[ARRAY_SIZE(out_buf)];
    struct spi_ioc_transfer k;
    int a;

    /* check input variables */
    CHECK_NULL(com_target);
    CHECK_NULL(data);

    spi_device = *(int *)com_target; /* must check that com_target is not null beforehand */

    /* prepare frame to be sent */
    out_buf[0] = spi_mux_target;
    out_buf[1] = READ_ACCESS | ((address >> 8) & 0x7F);
    out_buf[2] =               ((address >> 0) & 0xFF);
    out_buf[3] = 0x00;
    out_buf[4] = 0x00;
    command_size = 5;

    /* I/O transaction */
    memset(&k, 0, sizeof(k)); /* clear k */
    k.tx_buf = (unsigned long) out_buf;
    k.rx_buf = (unsigned long) in_buf;
    k.len = command_size;
    k.cs_change = 0;
    a = ioctl(spi_device, SPI_IOC_MESSAGE(1), &k);

    /* determine return code */
    if (a != (int)k.len) {
        DEBUG_MSG("ERROR: SPI READ FAILURE\n");
        return LGW_SPI_ERROR;
    } else {
        DEBUG_MSG("Note: SPI read success\n");
        *data = in_buf[command_size - 1];
        return LGW_SPI_SUCCESS;
    }
}

/* ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ */

/* Single Byte Read-Modify-Write */
int lgw_spi_rmw(void *com_target, uint8_t spi_mux_target, uint16_t address, uint8_t offs, uint8_t leng, uint8_t data) {
    int spi_stat = LGW_SPI_SUCCESS;
    uint8_t buf[4] = "\x00\x00\x00\x00";

    /* Read */
    spi_stat += lgw_spi_r(com_target, spi_mux_target, address, &buf[0]);

    /* Modify */
    buf[1] = ((1 << leng) - 1) << offs; /* bit mask */
    buf[2] = ((uint8_t)data) << offs; /* new data offsetted */
    buf[3] = (~buf[1] & buf[0]) | (buf[1] & buf[2]); /* mixing old & new data */

    /* Write */
    spi_stat += lgw_spi_w(com_target, spi_mux_target, address, buf[3]);

    return spi_stat;
}

/* ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ */

/* Burst (multiple-byte) write */
int lgw_spi_wb(void *com_target, uint8_t spi_mux_target, uint16_t address, const uint8_t *data, uint16_t size) {
    int spi_device;
    uint8_t command[3];
    uint8_t command_size;
    struct spi_ioc_transfer k[2];
    int size_to_do, chunk_size, offset;
    int byte_transfered = 0;
    int i;

    /* check input parameters */
    CHECK_NULL(com_target);
    CHECK_NULL(data);
    if (size == 0) {
        DEBUG_MSG("ERROR: BURST OF NULL LENGTH\n");
        return LGW_SPI_ERROR;
    }

    spi_device = *(int *)com_target; /* must check that com_target is not null beforehand */

    /* prepare command byte */
    command[0] = spi_mux_target;
    command[1] = WRITE_ACCESS | ((address >> 8) & 0x7F);
    command[2] =                ((address >> 0) & 0xFF);
    command_size = 3;
    size_to_do = size;

    /* I/O transaction */
    memset(&k, 0, sizeof(k)); /* clear k */
    k[0].tx_buf = (unsigned long) &command[0];
    k[0].len = command_size;
    k[0].cs_change = 0;
    k[1].cs_change = 0;
    for (i=0; size_to_do > 0; ++i) {
        chunk_size = (size_to_do < LGW_BURST_CHUNK) ? size_to_do : LGW_BURST_CHUNK;
        offset = i * LGW_BURST_CHUNK;
        k[1].tx_buf = (unsigned long)(data + offset);
        k[1].len = chunk_size;
        byte_transfered += (ioctl(spi_device, SPI_IOC_MESSAGE(2), &k) - k[0].len );
        DEBUG_PRINTF("BURST WRITE: to trans %d # chunk %d # transferred %d \n", size_to_do, chunk_size, byte_transfered);
        size_to_do -= chunk_size; /* subtract the quantity of data already transferred */
    }

    /* determine return code */
    if (byte_transfered != size) {
        DEBUG_MSG("ERROR: SPI BURST WRITE FAILURE\n");
        return LGW_SPI_ERROR;
    } else {
        DEBUG_MSG("Note: SPI burst write success\n");
        return LGW_SPI_SUCCESS;
    }
}

/* ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ */

/* Burst (multiple-byte) read */
int lgw_spi_rb(void *com_target, uint8_t spi_mux_target, uint16_t address, uint8_t *data, uint16_t size) {
    int spi_device;
    uint8_t command[4];
    uint8_t command_size;
    struct spi_ioc_transfer k[2];
    int size_to_do, chunk_size, offset;
    int byte_transfered = 0;
    int i;

    /* check input parameters */
    CHECK_NULL(com_target);
    CHECK_NULL(data);
    if (size == 0) {
        DEBUG_MSG("ERROR: BURST OF NULL LENGTH\n");
        return LGW_SPI_ERROR;
    }

    spi_device = *(int *)com_target; /* must check that com_target is not null beforehand */

    /* prepare command byte */
    command[0] = spi_mux_target;
    command[1] = READ_ACCESS | ((address >> 8) & 0x7F);
    command[2] =               ((address >> 0) & 0xFF);
    command[3] = 0x00;
    command_size = 4;
    size_to_do = size;

    /* I/O transaction */
    memset(&k, 0, sizeof(k)); /* clear k */
    k[0].tx_buf = (unsigned long) &command[0];
    k[0].len = command_size;
    k[0].cs_change = 0;
    k[1].cs_change = 0;
    for (i=0; size_to_do > 0; ++i) {
        chunk_size = (size_to_do < LGW_BURST_CHUNK) ? size_to_do : LGW_BURST_CHUNK;
        offset = i * LGW_BURST_CHUNK;
        k[1].rx_buf = (unsigned long)(data + offset);
        k[1].len = chunk_size;
        byte_transfered += (ioctl(spi_device, SPI_IOC_MESSAGE(2), &k) - k[0].len );
        DEBUG_PRINTF("BURST READ: to trans %d # chunk %d # transferred %d \n", size_to_do, chunk_size, byte_transfered);
        size_to_do -= chunk_size;  /* subtract the quantity of data already transferred */
    }

    /* determine return code */
    if (byte_transfered != size) {
        DEBUG_MSG("ERROR: SPI BURST READ FAILURE\n");
        return LGW_SPI_ERROR;
    } else {
        DEBUG_MSG("Note: SPI burst read success\n");
        return LGW_SPI_SUCCESS;
    }
}

/* ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ */

uint16_t lgw_spi_chunk_size(void) {
    return (uint16_t)LGW_BURST_CHUNK;
}

/* --- EOF ------------------------------------------------------------------ */
