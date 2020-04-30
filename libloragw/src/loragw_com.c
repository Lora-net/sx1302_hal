/*
 / _____)             _              | |
( (____  _____ ____ _| |_ _____  ____| |__
 \____ \| ___ |    (_   _) ___ |/ ___)  _ \
 _____) ) ____| | | || |_| ____( (___| | | |
(______/|_____)_|_|_| \__)_____)\____)_| |_|
  (C)2020 Semtech

Description:
    Functions to abstract the communication interface used to communicate with
    the concentrator.
    Single-byte read/write and burst read/write.

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

#include "loragw_com.h"
#include "loragw_usb.h"
#include "loragw_spi.h"
#include "loragw_aux.h"

/* -------------------------------------------------------------------------- */
/* --- PRIVATE MACROS ------------------------------------------------------- */

#define ARRAY_SIZE(a) (sizeof(a) / sizeof((a)[0]))
#if DEBUG_COM == 1
    #define DEBUG_MSG(str)                fprintf(stderr, str)
    #define DEBUG_PRINTF(fmt, args...)    fprintf(stderr,"%s:%d: "fmt, __FUNCTION__, __LINE__, args)
    #define CHECK_NULL(a)                if(a==NULL){fprintf(stderr,"%s:%d: ERROR: NULL POINTER AS ARGUMENT\n", __FUNCTION__, __LINE__);return LGW_COM_ERROR;}
#else
    #define DEBUG_MSG(str)
    #define DEBUG_PRINTF(fmt, args...)
    #define CHECK_NULL(a)                if(a==NULL){return LGW_COM_ERROR;}
#endif

/* -------------------------------------------------------------------------- */
/* --- PRIVATE CONSTANTS ---------------------------------------------------- */

/* -------------------------------------------------------------------------- */
/* --- PRIVATE VARIABLES ---------------------------------------------------- */

lgw_com_type_t lgw_com_type = LGW_COM_UNKNOWN;

/* -------------------------------------------------------------------------- */
/* --- PUBLIC FUNCTIONS DEFINITION ------------------------------------------ */

int lgw_com_open(lgw_com_type_t com_type, const char * com_path, void **com_target_ptr) {
    int com_stat;

    /* Check input parameters */
    CHECK_NULL(com_path);
    CHECK_NULL(com_target_ptr)
    if ((com_type != LGW_COM_SPI) && (com_type != LGW_COM_USB)) {
        DEBUG_MSG("ERROR: COMMUNICATION INTERFACE TYPE IS NOT SUPPORTED\n");
        return LGW_COM_ERROR;
    }

    /* set current com type */
    lgw_com_type = com_type;

    switch (com_type) {
        case LGW_COM_SPI:
            printf("Opening SPI communication interface\n");
            com_stat = lgw_spi_open(com_path, com_target_ptr);
            break;
        case LGW_COM_USB:
            printf("Opening USB communication interface\n");
            com_stat = lgw_usb_open(com_path, com_target_ptr);
            break;
        default:
            com_stat = LGW_COM_ERROR;
            break;
    }

    return com_stat;
}

/* ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ */

/* SPI release */
int lgw_com_close(void *com_target) {
    int com_stat;

    /* Check input parameters */
    CHECK_NULL(com_target);

    switch (lgw_com_type) {
        case LGW_COM_SPI:
            printf("Closing SPI communication interface\n");
            com_stat = lgw_spi_close(com_target);
            break;
        case LGW_COM_USB:
            printf("Closing USB communication interface\n");
            com_stat = lgw_usb_close(com_target);
            break;
        default:
            printf("ERROR: wrong communication type (SHOULD NOT HAPPEN)\n");
            com_stat = LGW_COM_ERROR;
            break;
    }

    return com_stat;
}

/* ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ */

/* Simple write */
int lgw_com_w(void *com_target, uint8_t spi_mux_target, uint16_t address, uint8_t data) {
    int com_stat;

    /* Check input parameters */
    CHECK_NULL(com_target);

    switch (lgw_com_type) {
        case LGW_COM_SPI:
            com_stat = lgw_spi_w(com_target, spi_mux_target, address, data);
            break;
        case LGW_COM_USB:
            com_stat = lgw_usb_w(com_target, spi_mux_target, address, data);
            break;
        default:
            printf("ERROR: wrong communication type (SHOULD NOT HAPPEN)\n");
            com_stat = LGW_COM_ERROR;
            break;
    }

    return com_stat;
}

/* ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ */

/* Simple read */
int lgw_com_r(void *com_target, uint8_t spi_mux_target, uint16_t address, uint8_t *data) {
    int com_stat;

    /* Check input parameters */
    CHECK_NULL(com_target);
    CHECK_NULL(data);

    switch (lgw_com_type) {
        case LGW_COM_SPI:
            com_stat = lgw_spi_r(com_target, spi_mux_target, address, data);
            break;
        case LGW_COM_USB:
            com_stat = lgw_usb_r(com_target, spi_mux_target, address, data);
            break;
        default:
            printf("ERROR: wrong communication type (SHOULD NOT HAPPEN)\n");
            com_stat = LGW_COM_ERROR;
            break;
    }

    return com_stat;
}

/* ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ */

/* Burst (multiple-byte) write */
int lgw_com_wb(void *com_target, uint8_t spi_mux_target, uint16_t address, const uint8_t *data, uint16_t size) {
    int com_stat;

    /* Check input parameters */
    CHECK_NULL(com_target);
    CHECK_NULL(data);

    switch (lgw_com_type) {
        case LGW_COM_SPI:
            com_stat = lgw_spi_wb(com_target, spi_mux_target, address, data, size);
            break;
        case LGW_COM_USB:
            com_stat = lgw_usb_wb(com_target, spi_mux_target, address, data, size);
            break;
        default:
            printf("ERROR: wrong communication type (SHOULD NOT HAPPEN)\n");
            com_stat = LGW_COM_ERROR;
            break;
    }

    return com_stat;
}

/* ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ */

/* Burst (multiple-byte) read */
int lgw_com_rb(void *com_target, uint8_t spi_mux_target, uint16_t address, uint8_t *data, uint16_t size) {
    int com_stat;

    /* Check input parameters */
    CHECK_NULL(com_target);
    CHECK_NULL(data);

    switch (lgw_com_type) {
        case LGW_COM_SPI:
            com_stat = lgw_spi_rb(com_target, spi_mux_target, address, data, size);
            break;
        case LGW_COM_USB:
            com_stat = lgw_usb_rb(com_target, spi_mux_target, address, data, size);
            break;
        default:
            printf("ERROR: wrong communication type (SHOULD NOT HAPPEN)\n");
            com_stat = LGW_COM_ERROR;
            break;
    }

    return com_stat;
}

/* --- EOF ------------------------------------------------------------------ */
