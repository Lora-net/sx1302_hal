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

#include "loragw_com.h"
#include "loragw_usb.h"
#include "loragw_spi.h"
#include "loragw_aux.h"

/* -------------------------------------------------------------------------- */
/* --- PRIVATE MACROS ------------------------------------------------------- */

#define ARRAY_SIZE(a) (sizeof(a) / sizeof((a)[0]))
#if DEBUG_COM == 1
    #define DEBUG_MSG(str)                fprintf(stdout, str)
    #define DEBUG_PRINTF(fmt, args...)    fprintf(stdout,"%s:%d: "fmt, __FUNCTION__, __LINE__, args)
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

/**
@brief The current communication type in use (SPI, USB)
*/
static lgw_com_type_t _lgw_com_type = LGW_COM_UNKNOWN;

/**
@brief A generic pointer to the COM device (file descriptor)
*/
static void* _lgw_com_target = NULL;

/* -------------------------------------------------------------------------- */
/* --- PUBLIC FUNCTIONS DEFINITION ------------------------------------------ */

int lgw_com_open(lgw_com_type_t com_type, const char * com_path) {
    int com_stat;

    /* Check input parameters */
    CHECK_NULL(com_path);
    if ((com_type != LGW_COM_SPI) && (com_type != LGW_COM_USB)) {
        DEBUG_MSG("ERROR: COMMUNICATION INTERFACE TYPE IS NOT SUPPORTED\n");
        return LGW_COM_ERROR;
    }

    if (_lgw_com_target != NULL) {
        DEBUG_MSG("WARNING: CONCENTRATOR WAS ALREADY CONNECTED\n");
        lgw_com_close();
    }

    /* set current com type */
    _lgw_com_type = com_type;

    switch (com_type) {
        case LGW_COM_SPI:
            printf("Opening SPI communication interface\n");
            com_stat = lgw_spi_open(com_path, &_lgw_com_target);
            break;
        case LGW_COM_USB:
            printf("Opening USB communication interface\n");
            com_stat = lgw_usb_open(com_path, &_lgw_com_target);
            break;
        default:
            com_stat = LGW_COM_ERROR;
            break;
    }

    return com_stat;
}

/* ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ */

/* SPI release */
int lgw_com_close(void) {
    int com_stat;

    if (_lgw_com_target == NULL) {
        printf("ERROR: concentrator is not connected\n");
        return -1;
    }

    switch (_lgw_com_type) {
        case LGW_COM_SPI:
            printf("Closing SPI communication interface\n");
            com_stat = lgw_spi_close(_lgw_com_target);
            break;
        case LGW_COM_USB:
            printf("Closing USB communication interface\n");
            com_stat = lgw_usb_close(_lgw_com_target);
            break;
        default:
            printf("ERROR(%s:%d): wrong communication type (SHOULD NOT HAPPEN)\n", __FUNCTION__, __LINE__);
            com_stat = LGW_COM_ERROR;
            break;
    }

    _lgw_com_target = NULL;

    return com_stat;
}

/* ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ */

/* Simple write */
int lgw_com_w(uint8_t spi_mux_target, uint16_t address, uint8_t data) {
    int com_stat;
    /* performances variables */
    struct timeval tm;

    /* Record function start time */
    _meas_time_start(&tm);

    /* Check input parameters */
    CHECK_NULL(_lgw_com_target);

    switch (_lgw_com_type) {
        case LGW_COM_SPI:
            com_stat = lgw_spi_w(_lgw_com_target, spi_mux_target, address, data);
            break;
        case LGW_COM_USB:
            com_stat = lgw_usb_w(_lgw_com_target, spi_mux_target, address, data);
            break;
        default:
            printf("ERROR(%s:%d): wrong communication type (SHOULD NOT HAPPEN)\n", __FUNCTION__, __LINE__);
            com_stat = LGW_COM_ERROR;
            break;
    }

    /* Compute time spent in this function */
    _meas_time_stop(5, tm, __FUNCTION__);

    return com_stat;
}

/* ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ */

/* Simple read */
int lgw_com_r(uint8_t spi_mux_target, uint16_t address, uint8_t *data) {
    int com_stat;
    /* performances variables */
    struct timeval tm;

    /* Record function start time */
    _meas_time_start(&tm);

    /* Check input parameters */
    CHECK_NULL(_lgw_com_target);
    CHECK_NULL(data);

    switch (_lgw_com_type) {
        case LGW_COM_SPI:
            com_stat = lgw_spi_r(_lgw_com_target, spi_mux_target, address, data);
            break;
        case LGW_COM_USB:
            com_stat = lgw_usb_r(_lgw_com_target, spi_mux_target, address, data);
            break;
        default:
            printf("ERROR(%s:%d): wrong communication type (SHOULD NOT HAPPEN)\n", __FUNCTION__, __LINE__);
            com_stat = LGW_COM_ERROR;
            break;
    }

    /* Compute time spent in this function */
    _meas_time_stop(5, tm, __FUNCTION__);

    return com_stat;
}

/* ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ */

int lgw_com_rmw(uint8_t spi_mux_target, uint16_t address, uint8_t offs, uint8_t leng, uint8_t data) {
    int com_stat;
    /* performances variables */
    struct timeval tm;

    /* Record function start time */
    _meas_time_start(&tm);

    /* Check input parameters */
    CHECK_NULL(_lgw_com_target);

    switch (_lgw_com_type) {
        case LGW_COM_SPI:
            com_stat = lgw_spi_rmw(_lgw_com_target, spi_mux_target, address, offs, leng, data);
            break;
        case LGW_COM_USB:
            com_stat = lgw_usb_rmw(_lgw_com_target, address, offs, leng, data);
            break;
        default:
            printf("ERROR(%s:%d): wrong communication type (SHOULD NOT HAPPEN)\n", __FUNCTION__, __LINE__);
            com_stat = LGW_COM_ERROR;
            break;
    }

    /* Compute time spent in this function */
    _meas_time_stop(5, tm, __FUNCTION__);

    return com_stat;
}

/* ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ */

/* Burst (multiple-byte) write */
int lgw_com_wb(uint8_t spi_mux_target, uint16_t address, const uint8_t *data, uint16_t size) {
    int com_stat;
    /* performances variables */
    struct timeval tm;

    /* Record function start time */
    _meas_time_start(&tm);

    /* Check input parameters */
    CHECK_NULL(_lgw_com_target);
    CHECK_NULL(data);

    switch (_lgw_com_type) {
        case LGW_COM_SPI:
            com_stat = lgw_spi_wb(_lgw_com_target, spi_mux_target, address, data, size);
            break;
        case LGW_COM_USB:
            com_stat = lgw_usb_wb(_lgw_com_target, spi_mux_target, address, data, size);
            break;
        default:
            printf("ERROR(%s:%d): wrong communication type (SHOULD NOT HAPPEN)\n", __FUNCTION__, __LINE__);
            com_stat = LGW_COM_ERROR;
            break;
    }

    /* Compute time spent in this function */
    _meas_time_stop(5, tm, __FUNCTION__);

    return com_stat;
}

/* ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ */

/* Burst (multiple-byte) read */
int lgw_com_rb(uint8_t spi_mux_target, uint16_t address, uint8_t *data, uint16_t size) {
    int com_stat;
    /* performances variables */
    struct timeval tm;

    /* Record function start time */
    _meas_time_start(&tm);

    /* Check input parameters */
    CHECK_NULL(_lgw_com_target);
    CHECK_NULL(data);

    switch (_lgw_com_type) {
        case LGW_COM_SPI:
            com_stat = lgw_spi_rb(_lgw_com_target, spi_mux_target, address, data, size);
            break;
        case LGW_COM_USB:
            com_stat = lgw_usb_rb(_lgw_com_target, spi_mux_target, address, data, size);
            break;
        default:
            printf("ERROR(%s:%d): wrong communication type (SHOULD NOT HAPPEN)\n", __FUNCTION__, __LINE__);
            com_stat = LGW_COM_ERROR;
            break;
    }

    /* Compute time spent in this function */
    _meas_time_stop(5, tm, __FUNCTION__);

    return com_stat;
}

/* ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ */

int lgw_com_set_write_mode(lgw_com_write_mode_t write_mode) {
    int com_stat = LGW_COM_SUCCESS;

    switch (_lgw_com_type) {
        case LGW_COM_SPI:
            /* Do nothing: only single mode is supported on SPI */
            break;
        case LGW_COM_USB:
            com_stat = lgw_usb_set_write_mode(write_mode);
            break;
        default:
            printf("ERROR(%s:%d): wrong communication type (SHOULD NOT HAPPEN)\n", __FUNCTION__, __LINE__);
            com_stat = LGW_COM_ERROR;
            break;
    }

    return com_stat;
}

/* ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ */

int lgw_com_flush(void) {
    int com_stat = LGW_COM_SUCCESS;

    switch (_lgw_com_type) {
        case LGW_COM_SPI:
            /* Do nothing: only single mode is supported on SPI */
            break;
        case LGW_COM_USB:
            com_stat = lgw_usb_flush(_lgw_com_target);
            break;
        default:
            printf("ERROR(%s:%d): wrong communication type (SHOULD NOT HAPPEN)\n", __FUNCTION__, __LINE__);
            com_stat = LGW_COM_ERROR;
            break;
    }

    return com_stat;
}

/* ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ */

uint16_t lgw_com_chunk_size(void) {
    switch (_lgw_com_type) {
        case LGW_COM_SPI:
            return lgw_spi_chunk_size();
        case LGW_COM_USB:
            return lgw_usb_chunk_size();
            break;
        default:
            printf("ERROR(%s:%d): wrong communication type (SHOULD NOT HAPPEN)\n", __FUNCTION__, __LINE__);
            return 0;
    }
}

/* ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ */

int lgw_com_get_temperature(float * temperature) {
    /* Check input parameters */
    CHECK_NULL(_lgw_com_target);
    CHECK_NULL(temperature);

    switch (_lgw_com_type) {
        case LGW_COM_SPI:
            printf("ERROR(%s:%d): not supported for SPI com\n", __FUNCTION__, __LINE__);
            return -1;
        case LGW_COM_USB:
            return lgw_usb_get_temperature(_lgw_com_target, temperature);
        default:
            printf("ERROR(%s:%d): wrong communication type (SHOULD NOT HAPPEN)\n", __FUNCTION__, __LINE__);
            return LGW_COM_ERROR;
    }
}

/* ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ */

void* lgw_com_target(void) {
    return _lgw_com_target;
}

/* ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ */

lgw_com_type_t lgw_com_type(void) {
    return _lgw_com_type;
}

/* --- EOF ------------------------------------------------------------------ */
