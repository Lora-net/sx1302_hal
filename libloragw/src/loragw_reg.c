/*
 / _____)             _              | |
( (____  _____ ____ _| |_ _____  ____| |__
 \____ \| ___ |    (_   _) ___ |/ ___)  _ \
 _____) ) ____| | | || |_| ____( (___| | | |
(______/|_____)_|_|_| \__)_____)\____)_| |_|
  (C)2018 Semtech

Description:
    TODO

License: Revised BSD License, see LICENSE.TXT file include in the project
*/


/* -------------------------------------------------------------------------- */
/* --- DEPENDANCIES --------------------------------------------------------- */

#include <stdint.h>     /* C99 types */
#include <stdbool.h>    /* bool type */
#include <stdio.h>      /* printf fprintf */

#include "loragw_spi.h"
#include "loragw_reg.h"

/* -------------------------------------------------------------------------- */
/* --- PRIVATE MACROS ------------------------------------------------------- */

#define ARRAY_SIZE(a) (sizeof(a) / sizeof((a)[0]))
#if DEBUG_REG == 1
    #define DEBUG_MSG(str)              fprintf(stderr, str)
    #define DEBUG_PRINTF(fmt, args...)  fprintf(stderr,"%s:%d: "fmt, __FUNCTION__, __LINE__, args)
    #define CHECK_NULL(a)               if(a==NULL){fprintf(stderr,"%s:%d: ERROR: NULL POINTER AS ARGUMENT\n", __FUNCTION__, __LINE__);return LGW_REG_ERROR;}
#else
    #define DEBUG_MSG(str)
    #define DEBUG_PRINTF(fmt, args...)
    #define CHECK_NULL(a)               if(a==NULL){return LGW_REG_ERROR;}
#endif

/* -------------------------------------------------------------------------- */
/* --- PRIVATE CONSTANTS ---------------------------------------------------- */

#define SX1302_REG_COMMON_BASE_ADDRESS      0x5600
#define SX1302_REG_AGCMCU_BASE_ADDRESS      0x5780

#define SX1302_REG_COMMON_VERSION_OFFSET    5
#define SX1302_REG_AGCMCU_RF_EN_A_OFFSET    3

const struct lgw_reg_s loregs[LGW_TOTALREGS] = {
    {0,SX1302_REG_COMMON_BASE_ADDRESS+SX1302_REG_COMMON_VERSION_OFFSET,0,0,8,1,1},
    {0,SX1302_REG_AGCMCU_BASE_ADDRESS+SX1302_REG_AGCMCU_RF_EN_A_OFFSET,2,0,1,0,0},  /* RADIO_EN */
    {0,SX1302_REG_AGCMCU_BASE_ADDRESS+SX1302_REG_AGCMCU_RF_EN_A_OFFSET,3,0,1,0,0}   /* RADIO_RST */
};

/* -------------------------------------------------------------------------- */
/* --- PRIVATE VARIABLES ---------------------------------------------------- */

/* -------------------------------------------------------------------------- */
/* --- INTERNAL SHARED VARIABLES -------------------------------------------- */

void *lgw_spi_target = NULL; /*! generic pointer to the SPI device */

/* -------------------------------------------------------------------------- */
/* --- PRIVATE FUNCTIONS ---------------------------------------------------- */

int reg_w_align32(void *spi_target, uint8_t spi_mux_target, struct lgw_reg_s r, int32_t reg_value) {
    int spi_stat = LGW_REG_SUCCESS;
    int i, size_byte;
    uint8_t buf[4] = "\x00\x00\x00\x00";

    if ((r.leng == 8) && (r.offs == 0)) {
        /* direct write */
        spi_stat += lgw_spi_w(spi_target, spi_mux_target, r.addr, (uint8_t)reg_value);
    } else if ((r.offs + r.leng) <= 8) {
        /* single-byte read-modify-write, offs:[0-7], leng:[1-7] */
        spi_stat += lgw_spi_r(spi_target, spi_mux_target, r.addr, &buf[0]);
        buf[1] = ((1 << r.leng) - 1) << r.offs; /* bit mask */
        buf[2] = ((uint8_t)reg_value) << r.offs; /* new data offsetted */
        buf[3] = (~buf[1] & buf[0]) | (buf[1] & buf[2]); /* mixing old & new data */
        spi_stat += lgw_spi_w(spi_target, spi_mux_target, r.addr, buf[3]);
    } else if ((r.offs == 0) && (r.leng > 0) && (r.leng <= 32)) {
        /* multi-byte direct write routine */
        size_byte = (r.leng + 7) / 8; /* add a byte if it's not an exact multiple of 8 */
        for (i=0; i<size_byte; ++i) {
            /* big endian register file for a file on N bytes
            Least significant byte is stored in buf[0], most one in buf[N-1] */
            buf[i] = (uint8_t)(0x000000FF & reg_value);
            reg_value = (reg_value >> 8);
        }
        spi_stat += lgw_spi_wb(spi_target, spi_mux_target, r.addr, buf, size_byte); /* write the register in one burst */
    } else {
        /* register spanning multiple memory bytes but with an offset */
        DEBUG_MSG("ERROR: REGISTER SIZE AND OFFSET ARE NOT SUPPORTED\n");
        return LGW_REG_ERROR;
    }

    return spi_stat;
}

/* ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ */

int reg_r_align32(void *spi_target, uint8_t spi_mux_target, struct lgw_reg_s r, int32_t *reg_value) {
    int spi_stat = LGW_SPI_SUCCESS;
    uint8_t bufu[4] = "\x00\x00\x00\x00";
    int8_t *bufs = (int8_t *)bufu;
    int i, size_byte;
    uint32_t u = 0;

    if ((r.offs + r.leng) <= 8) {
        /* read one byte, then shift and mask bits to get reg value with sign extension if needed */
        spi_stat += lgw_spi_r(spi_target, spi_mux_target, r.addr, &bufu[0]);
        bufu[1] = bufu[0] << (8 - r.leng - r.offs); /* left-align the data */
        if (r.sign == true) {
            bufs[2] = bufs[1] >> (8 - r.leng); /* right align the data with sign extension (ARITHMETIC right shift) */
            *reg_value = (int32_t)bufs[2]; /* signed pointer -> 32b sign extension */
        } else {
            bufu[2] = bufu[1] >> (8 - r.leng); /* right align the data, no sign extension */
            *reg_value = (int32_t)bufu[2]; /* unsigned pointer -> no sign extension */
        }
    } else if ((r.offs == 0) && (r.leng > 0) && (r.leng <= 32)) {
        size_byte = (r.leng + 7) / 8; /* add a byte if it's not an exact multiple of 8 */
        spi_stat += lgw_spi_rb(spi_target, spi_mux_target, r.addr, bufu, size_byte);
        u = 0;
        for (i=(size_byte-1); i>=0; --i) {
            u = (uint32_t)bufu[i] + (u << 8); /* transform a 4-byte array into a 32 bit word */
        }
        if (r.sign == true) {
            u = u << (32 - r.leng); /* left-align the data */
            *reg_value = (int32_t)u >> (32 - r.leng); /* right-align the data with sign extension (ARITHMETIC right shift) */
        } else {
            *reg_value = (int32_t)u; /* unsigned value -> return 'as is' */
        }
    } else {
        /* register spanning multiple memory bytes but with an offset */
        DEBUG_MSG("ERROR: REGISTER SIZE AND OFFSET ARE NOT SUPPORTED\n");
        return LGW_REG_ERROR;
    }

    return spi_stat;
}

/* -------------------------------------------------------------------------- */
/* --- PUBLIC FUNCTIONS DEFINITION ------------------------------------------ */

/* Concentrator connect */
int lgw_connect(void) {
    int spi_stat = LGW_SPI_SUCCESS;
    uint8_t u = 0;

    /* check SPI link status */
    if (lgw_spi_target != NULL) {
        DEBUG_MSG("WARNING: concentrator was already connected\n");
        lgw_spi_close(lgw_spi_target);
    }

    /* open the SPI link */
    spi_stat = lgw_spi_open(&lgw_spi_target);
    if (spi_stat != LGW_SPI_SUCCESS) {
        DEBUG_MSG("ERROR CONNECTING CONCENTRATOR\n");
        return LGW_REG_ERROR;
    }

    /* check SX1301 version */
    spi_stat = lgw_spi_r(lgw_spi_target, LGW_SPI_MUX_TARGET_SX1302, loregs[LGW_VERSION].addr, &u);
    if (spi_stat != LGW_SPI_SUCCESS) {
        DEBUG_MSG("ERROR READING CHIP VERSION REGISTER\n");
        return LGW_REG_ERROR;
    }
    if (u != loregs[LGW_VERSION].dflt) {
        DEBUG_PRINTF("ERROR: NOT EXPECTED CHIP VERSION (v%u)\n", u);
        return LGW_REG_ERROR;
    }
    DEBUG_PRINTF("Note: chip version is 0x%02X\n", u);

    DEBUG_MSG("Note: success connecting the concentrator\n");
    return LGW_REG_SUCCESS;
}

/* ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ */

/* Concentrator disconnect */
int lgw_disconnect(void) {
    if (lgw_spi_target != NULL) {
        lgw_spi_close(lgw_spi_target);
        lgw_spi_target = NULL;
        DEBUG_MSG("Note: success disconnecting the concentrator\n");
        return LGW_REG_SUCCESS;
    } else {
        DEBUG_MSG("WARNING: concentrator was already disconnected\n");
        return LGW_REG_ERROR;
    }
}

/* ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ */

/* Write to a register addressed by name */
int lgw_reg_w(uint16_t register_id, int32_t reg_value) {
    int spi_stat = LGW_SPI_SUCCESS;
    struct lgw_reg_s r;

    /* check input parameters */
    if (register_id >= LGW_TOTALREGS) {
        DEBUG_MSG("ERROR: REGISTER NUMBER OUT OF DEFINED RANGE\n");
        return LGW_REG_ERROR;
    }

    /* check if SPI is initialised */
    if (lgw_spi_target == NULL) {
        DEBUG_MSG("ERROR: CONCENTRATOR UNCONNECTED\n");
        return LGW_REG_ERROR;
    }

    /* get register struct from the struct array */
    r = loregs[register_id];

    /* reject write to read-only registers */
    if (r.rdon == 1){
        DEBUG_MSG("ERROR: TRYING TO WRITE A READ-ONLY REGISTER\n");
        return LGW_REG_ERROR;
    }

    spi_stat += reg_w_align32(lgw_spi_target, LGW_SPI_MUX_TARGET_SX1302, r, reg_value);

    if (spi_stat != LGW_SPI_SUCCESS) {
        DEBUG_MSG("ERROR: SPI ERROR DURING REGISTER WRITE\n");
        return LGW_REG_ERROR;
    } else {
        return LGW_REG_SUCCESS;
    }
}

/* ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ */

/* Read to a register addressed by name */
int lgw_reg_r(uint16_t register_id, int32_t *reg_value) {
    int spi_stat = LGW_SPI_SUCCESS;
    struct lgw_reg_s r;

    /* check input parameters */
    CHECK_NULL(reg_value);
    if (register_id >= LGW_TOTALREGS) {
        DEBUG_MSG("ERROR: REGISTER NUMBER OUT OF DEFINED RANGE\n");
        return LGW_REG_ERROR;
    }

    /* check if SPI is initialised */
    if (lgw_spi_target == NULL) {
        DEBUG_MSG("ERROR: CONCENTRATOR UNCONNECTED\n");
        return LGW_REG_ERROR;
    }

    /* get register struct from the struct array */
    r = loregs[register_id];

    spi_stat += reg_r_align32(lgw_spi_target, LGW_SPI_MUX_TARGET_SX1302, r, reg_value);

    if (spi_stat != LGW_SPI_SUCCESS) {
        DEBUG_MSG("ERROR: SPI ERROR DURING REGISTER WRITE\n");
        return LGW_REG_ERROR;
    } else {
        return LGW_REG_SUCCESS;
    }
}

/* ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ */

/* Point to a register by name and do a burst write */
int lgw_reg_wb(uint16_t register_id, uint8_t *data, uint16_t size) {
    int spi_stat = LGW_SPI_SUCCESS;
    struct lgw_reg_s r;

    /* check input parameters */
    CHECK_NULL(data);
    if (size == 0) {
        DEBUG_MSG("ERROR: BURST OF NULL LENGTH\n");
        return LGW_REG_ERROR;
    }
    if (register_id >= LGW_TOTALREGS) {
        DEBUG_MSG("ERROR: REGISTER NUMBER OUT OF DEFINED RANGE\n");
        return LGW_REG_ERROR;
    }

    /* check if SPI is initialised */
    if (lgw_spi_target == NULL) {
        DEBUG_MSG("ERROR: CONCENTRATOR UNCONNECTED\n");
        return LGW_REG_ERROR;
    }

    /* get register struct from the struct array */
    r = loregs[register_id];

    /* reject write to read-only registers */
    if (r.rdon == 1){
        DEBUG_MSG("ERROR: TRYING TO BURST WRITE A READ-ONLY REGISTER\n");
        return LGW_REG_ERROR;
    }

    /* do the burst write */
    spi_stat += lgw_spi_wb(lgw_spi_target, LGW_SPI_MUX_TARGET_SX1302, r.addr, data, size);

    if (spi_stat != LGW_SPI_SUCCESS) {
        DEBUG_MSG("ERROR: SPI ERROR DURING REGISTER BURST WRITE\n");
        return LGW_REG_ERROR;
    } else {
        return LGW_REG_SUCCESS;
    }
}

/* ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ */

/* Point to a register by name and do a burst read */
int lgw_reg_rb(uint16_t register_id, uint8_t *data, uint16_t size) {
    int spi_stat = LGW_SPI_SUCCESS;
    struct lgw_reg_s r;

    /* check input parameters */
    CHECK_NULL(data);
    if (size == 0) {
        DEBUG_MSG("ERROR: BURST OF NULL LENGTH\n");
        return LGW_REG_ERROR;
    }
    if (register_id >= LGW_TOTALREGS) {
        DEBUG_MSG("ERROR: REGISTER NUMBER OUT OF DEFINED RANGE\n");
        return LGW_REG_ERROR;
    }

    /* check if SPI is initialised */
    if (lgw_spi_target == NULL) {
        DEBUG_MSG("ERROR: CONCENTRATOR UNCONNECTED\n");
        return LGW_REG_ERROR;
    }

    /* get register struct from the struct array */
    r = loregs[register_id];

    /* do the burst read */
    spi_stat += lgw_spi_rb(lgw_spi_target, LGW_SPI_MUX_TARGET_SX1302, r.addr, data, size);

    if (spi_stat != LGW_SPI_SUCCESS) {
        DEBUG_MSG("ERROR: SPI ERROR DURING REGISTER BURST READ\n");
        return LGW_REG_ERROR;
    } else {
        return LGW_REG_SUCCESS;
    }
}

/* --- EOF ------------------------------------------------------------------ */
