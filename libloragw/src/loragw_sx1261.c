/*
 / _____)             _              | |
( (____  _____ ____ _| |_ _____  ____| |__
 \____ \| ___ |    (_   _) ___ |/ ___)  _ \
 _____) ) ____| | | || |_| ____( (___| | | |
(______/|_____)_|_|_| \__)_____)\____)_| |_|
  (C)2019 Semtech

Description:
    Functions used to handle LoRa concentrator SX1261 radio used to handle LBT
    and Spectral Scan.

License: Revised BSD License, see LICENSE.TXT file include in the project
*/


/* -------------------------------------------------------------------------- */
/* --- DEPENDANCIES --------------------------------------------------------- */

#include <stdint.h>     /* C99 types */
#include <stdio.h>      /* printf fprintf */
#include <stdlib.h>     /* malloc free */
#include <unistd.h>     /* lseek, close */

#include "loragw_sx1261.h"
#include "loragw_spi.h"
#include "loragw_com.h"
#include "loragw_aux.h"
#include "loragw_reg.h"

#include "sx1261_com.h"

/* -------------------------------------------------------------------------- */
/* --- PRIVATE MACROS ------------------------------------------------------- */

#define ARRAY_SIZE(a) (sizeof(a) / sizeof((a)[0]))
#if DEBUG_RAD == 1
    #define DEBUG_MSG(str)                fprintf(stderr, str)
    #define DEBUG_PRINTF(fmt, args...)    fprintf(stderr,"%s:%d: "fmt, __FUNCTION__, __LINE__, args)
    #define CHECK_NULL(a)                if(a==NULL){fprintf(stderr,"%s:%d: ERROR: NULL POINTER AS ARGUMENT\n", __FUNCTION__, __LINE__);return LGW_REG_ERROR;}
#else
    #define DEBUG_MSG(str)
    #define DEBUG_PRINTF(fmt, args...)
    #define CHECK_NULL(a)                if(a==NULL){return LGW_REG_ERROR;}
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

int sx1261_connect(const char * spi_path) {
    int spi_stat = LGW_SPI_SUCCESS;

    if (spi_path != NULL) {
        _sx1261_com_type = LGW_COM_SPI;

        /* open the SPI link */
        spi_stat = lgw_spi_open(spi_path, &_sx1261_com_target);
        if (spi_stat != LGW_SPI_SUCCESS) {
            printf("ERROR CONNECTING SX1261 RADIO\n");
            return LGW_REG_ERROR;
        }

        printf("SX1261: connected with SPI %s\n", spi_path);
    } else {
        _sx1261_com_type = LGW_COM_USB;

        /* the USB link has already been opened (lgw_connect) */
        printf("SX1261: connected with USB\n");
    }

    return LGW_REG_SUCCESS;
}

/* ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ */

int sx1261_disconnect(void) {
    int spi_stat = LGW_SPI_SUCCESS;

    if (_sx1261_com_type == LGW_COM_SPI) {
        /* Close the SPI link */
        spi_stat = lgw_spi_close(_sx1261_com_target);
        if (spi_stat != LGW_SPI_SUCCESS) {
            printf("ERROR DISCONNECTING SX1261 RADIO\n");
            return LGW_REG_ERROR;
        }
    }

    /* Reset context */
    _sx1261_com_target = NULL;
    _sx1261_com_type = LGW_COM_UNKNOWN;

    return LGW_REG_SUCCESS;
}

/* ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ */

int sx1261_reg_w(sx1261_op_code_t op_code, uint8_t *data, uint16_t size) {
    int com_stat;

    /* checking input parameters */
    CHECK_NULL(data);

    com_stat = sx1261_com_w(_sx1261_com_type, _sx1261_com_target, op_code, data, size);

    if (com_stat != LGW_COM_SUCCESS) {
        printf("ERROR: COM ERROR DURING SX1261 RADIO REGISTER WRITE\n");
        return LGW_REG_ERROR;
    } else {
        return LGW_REG_SUCCESS;
    }
}

/* ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ */

int sx1261_reg_r(sx1261_op_code_t op_code, uint8_t *data, uint16_t size) {
    int com_stat;

    /* checking input parameters */
    CHECK_NULL(data);

    com_stat = sx1261_com_r(_sx1261_com_type, _sx1261_com_target, op_code, data, size);

    if (com_stat != LGW_COM_SUCCESS) {
        printf("ERROR: COM ERROR DURING SX1261 RADIO REGISTER READ\n");
        return LGW_REG_ERROR;
    } else {
        return LGW_REG_SUCCESS;
    }
}

/* --- EOF ------------------------------------------------------------------ */
