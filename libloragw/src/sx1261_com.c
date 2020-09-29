/*
 / _____)             _              | |
( (____  _____ ____ _| |_ _____  ____| |__
 \____ \| ___ |    (_   _) ___ |/ ___)  _ \
 _____) ) ____| | | || |_| ____( (___| | | |
(______/|_____)_|_|_| \__)_____)\____)_| |_|
  (C)2019 Semtech

Description:
    Functions used to handle the sx1261 radio used for LBT/Spectral Scan.

License: Revised BSD License, see LICENSE.TXT file include in the project
*/


/* -------------------------------------------------------------------------- */
/* --- DEPENDANCIES --------------------------------------------------------- */

#include <stdint.h>     /* C99 types */
#include <stdio.h>      /* printf fprintf */

#include "loragw_com.h"
#include "loragw_spi.h"
#include "sx1261_com.h"
#include "sx1261_spi.h"
#include "sx1261_usb.h"

/* -------------------------------------------------------------------------- */
/* --- PRIVATE MACROS ------------------------------------------------------- */

#define ARRAY_SIZE(a) (sizeof(a) / sizeof((a)[0]))
#if DEBUG_LBT == 1
    #define DEBUG_MSG(str)                fprintf(stdout, str)
    #define DEBUG_PRINTF(fmt, args...)    fprintf(stdout,"%s:%d: "fmt, __FUNCTION__, __LINE__, args)
    #define CHECK_NULL(a)                if(a==NULL){fprintf(stderr,"%s:%d: ERROR: NULL POINTER AS ARGUMENT\n", __FUNCTION__, __LINE__);return -1;}
#else
    #define DEBUG_MSG(str)
    #define DEBUG_PRINTF(fmt, args...)
    #define CHECK_NULL(a)                if(a==NULL){return -1;}
#endif

/* -------------------------------------------------------------------------- */
/* --- PRIVATE CONSTANTS ---------------------------------------------------- */

/* -------------------------------------------------------------------------- */
/* --- PRIVATE VARIABLES ---------------------------------------------------- */

/**
@brief The current communication type in use (SPI, USB)
*/
static lgw_com_type_t _sx1261_com_type = LGW_COM_UNKNOWN;

/**
@brief A generic pointer to the COM device (file descriptor)
*/
static void* _sx1261_com_target = NULL;

/* -------------------------------------------------------------------------- */
/* --- PUBLIC FUNCTIONS DEFINITION ------------------------------------------ */

/**
 *
*/
int sx1261_com_open(lgw_com_type_t com_type, const char *com_path) {
    int spi_stat = LGW_COM_SUCCESS;

    _sx1261_com_type = com_type;

    switch(com_type) {
        case LGW_COM_SPI:
            /* open the SPI link */
            spi_stat = lgw_spi_open(com_path, &_sx1261_com_target);
            if (spi_stat != LGW_SPI_SUCCESS) {
                printf("ERROR: %s: Failed to connect to sx1261 radio on %s\n", __FUNCTION__, com_path);
                return LGW_COM_ERROR;
            }
            DEBUG_PRINTF("SX1261: connected with SPI %s\n", com_path);
            break;
        case LGW_COM_USB:
            /* the USB link has already been opened (lgw_connect) */
            _sx1261_com_target = lgw_com_target();
            DEBUG_MSG("SX1261: connected with USB\n");
            break;
        default:
            printf("ERROR: %s: wrong COM type\n", __FUNCTION__);
            return LGW_COM_ERROR;
    }

    return LGW_COM_SUCCESS;
}

/* ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ */

int sx1261_com_close(void) {
    int spi_stat = LGW_COM_SUCCESS;

    switch(_sx1261_com_type) {
        case LGW_COM_SPI:
            /* Close the SPI link */
            spi_stat = lgw_spi_close(_sx1261_com_target);
            if (spi_stat != LGW_SPI_SUCCESS) {
                printf("ERROR: %s: Failed to disconnect SX1261 radio\n", __FUNCTION__);
                return LGW_COM_ERROR;
            }
            break;
        case LGW_COM_USB:
            break;
        default:
            printf("ERROR: %s: sx1261 not connected\n", __FUNCTION__);
            return LGW_COM_ERROR;
    }

    _sx1261_com_type = LGW_COM_UNKNOWN;
    _sx1261_com_target = NULL;

    return LGW_COM_SUCCESS;
}

/* ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ */

int sx1261_com_w(sx1261_op_code_t op_code, uint8_t *data, uint16_t size) {
    int com_stat;

    /* Check input parameters */
    CHECK_NULL(_sx1261_com_target);
    CHECK_NULL(data);

    switch (_sx1261_com_type) {
        case LGW_COM_SPI:
            com_stat = sx1261_spi_w(_sx1261_com_target, op_code, data, size);
            break;
        case LGW_COM_USB:
            com_stat = sx1261_usb_w(_sx1261_com_target, op_code, data, size);
            break;
        default:
            printf("ERROR: wrong communication type (SHOULD NOT HAPPEN)\n");
            com_stat = LGW_COM_ERROR;
            break;
    }

    return com_stat;
}

/* ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ */

int sx1261_com_r(sx1261_op_code_t op_code, uint8_t *data, uint16_t size) {
    int com_stat;

    /* Check input parameters */
    CHECK_NULL(_sx1261_com_target);
    CHECK_NULL(data);

    switch (_sx1261_com_type) {
        case LGW_COM_SPI:
            com_stat = sx1261_spi_r(_sx1261_com_target, op_code, data, size);
            break;
        case LGW_COM_USB:
            com_stat = sx1261_usb_r(_sx1261_com_target, op_code, data, size);
            break;
        default:
            printf("ERROR: wrong communication type (SHOULD NOT HAPPEN)\n");
            com_stat = LGW_COM_ERROR;
            break;
    }

    return com_stat;
}

/* ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ */

int sx1261_com_set_write_mode(lgw_com_write_mode_t write_mode) {
    int com_stat = LGW_COM_SUCCESS;

    switch (_sx1261_com_type) {
        case LGW_COM_SPI:
            /* Do nothing: only single mode is supported on SPI */
            break;
        case LGW_COM_USB:
            com_stat = sx1261_usb_set_write_mode(write_mode);
            break;
        default:
            printf("ERROR(%s:%d): wrong communication type (SHOULD NOT HAPPEN)\n", __FUNCTION__, __LINE__);
            com_stat = LGW_COM_ERROR;
            break;
    }

    return com_stat;
}

/* ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ */

int sx1261_com_flush(void) {
    int com_stat = LGW_COM_SUCCESS;

    switch (_sx1261_com_type) {
        case LGW_COM_SPI:
            /* Do nothing: only single mode is supported on SPI */
            break;
        case LGW_COM_USB:
            com_stat = sx1261_usb_flush(_sx1261_com_target);
            break;
        default:
            printf("ERROR(%s:%d): wrong communication type (SHOULD NOT HAPPEN)\n", __FUNCTION__, __LINE__);
            com_stat = LGW_COM_ERROR;
            break;
    }

    return com_stat;
}

/* --- EOF ------------------------------------------------------------------ */
