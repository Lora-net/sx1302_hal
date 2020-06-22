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
#include "loragw_hal.h"

#include "sx1261_com.h"

#include "sx1261_pram.var"

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
/* --- PRIVATE FUNCTIONS ---------------------------------------------------- */

int sx1261_pram_get_version(void) {
    uint8_t buff[32];
    int x, i;

    buff[0] = 0x03;
    buff[1] = 0x20;
    buff[2] = 0x00;
    buff[3] = 0x00;
    buff[4] = 0x00;
    buff[5] = 0x00;
    buff[6] = 0x00;
    buff[7] = 0x00;
    buff[8] = 0x00;
    buff[9] = 0x00;
    buff[10] = 0x00;
    buff[11] = 0x00;
    buff[12] = 0x00;
    buff[13] = 0x00;
    buff[14] = 0x00;
    buff[15] = 0x00;
    buff[16] = 0x00;
    buff[17] = 0x00;
    x = sx1261_reg_r(SX1261_READ_REGISTER, buff, 18);
    if (x != LGW_REG_SUCCESS) {
        printf("ERROR: failed to read SX1261 PRAM version\n");
        return x;
    }

    /* TODO: return the version string */

    printf("SX1261: PRAM: ");
    for(i = 0; i < 18; i++) {
        printf("%c", (char)buff[i]);
    }
    printf("\n");

    return LGW_REG_SUCCESS;
}

/* -------------------------------------------------------------------------- */
/* --- PUBLIC FUNCTIONS DEFINITION ------------------------------------------ */

int sx1261_connect(const char * spi_path) {
    int spi_stat = LGW_REG_SUCCESS;

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

/* ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ */

int sx1261_load_pram(void) {
    int i;
    uint8_t buff[32];
    uint32_t val, addr;

    /* Set Radio in Standby mode */
    buff[0] = (uint8_t)SX1261_STDBY_RC;
    sx1261_reg_w(SX1261_SET_STANDBY, buff, 1);
    wait_ms(10);

    /* Get status */
    buff[0] = 0x00;
    sx1261_reg_r( SX1261_GET_STATUS, buff, 1);
    DEBUG_PRINTF("SX1261: %s: get_status: 0x%02X\n", __FUNCTION__, buff[0]);
    /* TODO: check if status is as expected */

    sx1261_pram_get_version();

    /* Enable patch update */
    buff[0] = 0x06;
    buff[1] = 0x10;
    buff[2] = 0x10;
    sx1261_reg_w( SX1261_WRITE_REGISTER, buff, 3);

    /* Load patch */
    for (i = 0; i < (int)PRAM_COUNT; i++) {
        val = pram[i];
        addr = 0x8000 + 4*i;

        buff[0] = (addr >> 8) & 0xFF;
        buff[1] = (addr >> 0) & 0xFF;
        buff[2] = (val >> 24) & 0xFF;
        buff[3] = (val >> 16) & 0xFF;
        buff[4] = (val >> 8)  & 0xFF;
        buff[5] = (val >> 0)  & 0xFF;
        sx1261_reg_w(SX1261_WRITE_REGISTER, buff, 6);
    }

    /* Disable patch update */
    buff[0] = 0x06;
    buff[1] = 0x10;
    buff[2] = 0x00;
    sx1261_reg_w( SX1261_WRITE_REGISTER, buff, 3);

    /* Update pram */
    buff[0] = 0;
    sx1261_reg_w(0xd9, buff, 0);

    sx1261_pram_get_version();
    /* TODO: check PRAM version is correct */

    return 0;
}

/* ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ */

int sx1261_setup(uint32_t freq_hz) {
    int32_t freq_reg;
    uint8_t buff[32];

    /* Set Radio in Standby mode */
    buff[0] = (uint8_t)SX1261_STDBY_RC;
    sx1261_reg_w(SX1261_SET_STANDBY, buff, 1);
    wait_ms(10);
    sx1261_reg_w(SX1261_SET_FS, buff, 0);
    wait_us(150);

    /* Check radio status */
    buff[0] = 0x00;
    sx1261_reg_r(SX1261_GET_STATUS, buff, 1);
    if (buff[0] != 0x42) { // TODO: check possible values (busy ?)
        printf("ERROR: %s: unexpected status (0x%02X), should be 0x42\n", __FUNCTION__, buff[0]);
        return LGW_REG_ERROR;
    }

    /* Set PacketType */
    buff[0] = 0x00; /* FSK */
    sx1261_reg_w(SX1261_SET_PACKET_TYPE, buff, 1);

    /* Set frequency */
    freq_reg = SX1261_FREQ_TO_REG(freq_hz);
    buff[0] = (uint8_t)(freq_reg >> 24);
    buff[1] = (uint8_t)(freq_reg >> 16);
    buff[2] = (uint8_t)(freq_reg >> 8);
    buff[3] = (uint8_t)(freq_reg >> 0);
    sx1261_reg_w(SX1261_SET_RF_FREQUENCY, buff, 4);

    /* Set modulation params for FSK */
    buff[0] = 0;    /* bitrate */
    buff[1] = 0x14; /* bitrate */
    buff[2] = 0x00; /* bitrate */
    buff[3] = 0x00; /* Gaussian BT disabled */
    buff[4] = 0x0A; /* BW_234300 (232.3 kHz DSB)  0x0A fo 232.3 kHz 0x09 for 467 kHz 0x18 for 624 kHz */
    buff[5] = 0x02; /* FDEV */
    buff[6] = 0xE9; /* FDEV */
    buff[7] = 0x0F; /* FDEV */
    sx1261_reg_w(SX1261_SET_MODULATION_PARAMS, buff, 8);

    /* Set packet params for FSK */
    buff[0] = 0x00; /* Preamble length MSB */
    buff[1] = 0x20; /* Preamble length LSB 32 bits*/
    buff[2] = 0x05; /* Preamble detector lenght 16 bits */
    buff[3] = 0x20; /* SyncWordLength 32 bits*/
    buff[4] = 0x00; /* AddrComp disabled */
    buff[5] = 0x01; /* PacketType variable size */
    buff[6] = 0xff; /* PayloadLength 255 bytes */
    buff[7] = 0x00; /* CRCType 1 Byte */
    buff[8] = 0x00; /* Whitening disabled*/
    sx1261_reg_w(SX1261_SET_PACKET_PARAMS, buff, 9);

    /* Set Buffer Base address */
    buff[0] = 0x80;
    buff[1] = 0x80;
    sx1261_reg_w(SX1261_SET_BUFFER_BASE_ADDRESS, buff, 2);

    /* Configure RSSI averaging window */
    buff[0] = 0x08;
    buff[1] = 0x9B;
    buff[2] = 0x05 << 2;
    sx1261_reg_w(SX1261_WRITE_REGISTER, buff, 3);

    /* sensi adjust */
    buff[0] = 0x08;
    buff[1] = 0xAC;
    buff[2] = 0xCB;
    sx1261_reg_w(SX1261_WRITE_REGISTER, buff, 3);

    /* Set Radio in Rx continuous mode */
    buff[0] = 0xFF;
    buff[1] = 0xFF;
    buff[2] = 0xFF;
    sx1261_reg_w(SX1261_SET_RX, buff, 3);
    wait_us(150);

    /* Check radio status */
    buff[0] = 0;
    sx1261_reg_r(SX1261_GET_STATUS, buff, 1);
    if (buff[0] != 0x52) { // TODO: check possible values (busy ?)
        printf("ERROR: %s : unexpected status (0x%02X), should be 0x52\n", __FUNCTION__, buff[0]);
        return LGW_REG_ERROR;
    }

    printf("SX1261: setup for LBT / Spectral Scan done\n");

    return LGW_REG_SUCCESS;
}

/* ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ */

int sx1261_set_rf_frequency(uint32_t freq_hz, uint8_t bandwidth) {
    uint8_t buff[16];
    int32_t freq_reg;
    uint8_t fsk_bw_reg;

    /* Set FS */
    sx1261_reg_w(SX1261_SET_FS, buff, 0);
    wait_us(100);

    /* Check radio status */
    buff[0] = 0x00;
    sx1261_reg_r(SX1261_GET_STATUS, buff, 1);
    if (buff[0] != 0x42) {
        printf("ERROR: %s : unexpected status (0x%02X), should be 0x42\n", __FUNCTION__, buff[0]);
        return LGW_REG_ERROR;
    }

    /* Set frequency */
    freq_reg = SX1261_FREQ_TO_REG(freq_hz);
    buff[0] = (uint8_t)(freq_reg >> 24);
    buff[1] = (uint8_t)(freq_reg >> 16);
    buff[2] = (uint8_t)(freq_reg >> 8);
    buff[3] = (uint8_t)(freq_reg >> 0);
    sx1261_reg_w(SX1261_SET_RF_FREQUENCY, buff, 4);

    /* Configure RSSI averaging window */
    buff[0] = 0x08;
    buff[1] = 0x9B;
    buff[2] = 0x05 << 2;
    sx1261_reg_w(SX1261_WRITE_REGISTER, buff, 3);

    /* Set PacketType */
    buff[0] = 0x00; /* FSK */
    sx1261_reg_w(SX1261_SET_PACKET_TYPE, buff, 1);

    /* Set GFSK bandwidth */
    switch (bandwidth) {
        case BW_125KHZ:
            fsk_bw_reg = 0x0A; /* RX_BW_234300 Hz */
            break;
        case BW_250KHZ:
            fsk_bw_reg = 0x09; /* RX_BW_467000 Hz */
            break;
        default:
            printf("ERROR: %s: Cannot configure sx1261 for bandwidth %u\n", __FUNCTION__, bandwidth);
            return LGW_REG_ERROR;
    }

    /* Set modulation params for FSK */
    buff[0] = 0;    // BR
    buff[1] = 0x14; // BR
    buff[2] = 0x00; // BR
    buff[3] = 0x00; // Gaussian BT disabled
    buff[4] = fsk_bw_reg;
    buff[5] = 0x02; // FDEV
    buff[6] = 0xE9; // FDEV
    buff[7] = 0x0F; // FDEV
    sx1261_reg_w(SX1261_SET_MODULATION_PARAMS, buff, 8);

    /* Set packet params for FSK */
    buff[0] = 0x00; /* Preamble length MSB */
    buff[1] = 0x20; /* Preamble length LSB 32 bits*/
    buff[2] = 0x05; /* Preamble detector lenght 16 bits */
    buff[3] = 0x20; /* SyncWordLength 32 bits*/
    buff[4] = 0x00; /* AddrComp disabled */
    buff[5] = 0x01; /* PacketType variable size */
    buff[6] = 0xff; /* PayloadLength 255 bytes */
    buff[7] = 0x00; /* CRCType 1 Byte */
    buff[8] = 0x00; /* Whitening disabled*/
    sx1261_reg_w(SX1261_SET_PACKET_PARAMS, buff, 9);

    /* Set Radio in Rx continuous mode */
    buff[0] = 0xFF;
    buff[1] = 0xFF;
    buff[2] = 0xFF;
    sx1261_reg_w(SX1261_SET_RX, buff, 3);
    wait_us(150);

    /* Check radio status */
    buff[0] = 0;
    sx1261_reg_r(SX1261_GET_STATUS, buff, 1);
    if (buff[0] != 0x52) { // TODO: check possible values (busy ?)
        printf("ERROR: %s : unexpected status (0x%02X), should be 0x52\n", __FUNCTION__, buff[0]);
        return LGW_REG_ERROR;
    }

    printf("SX1261: RF frequency set to %u Hz (bw:0x%02X)\n", freq_hz, bandwidth);

    return LGW_REG_SUCCESS;
}

/* ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ */

int sx1261_lbt_start(uint16_t scan_time_us, int8_t threshold_dbm) {
    uint8_t buff[16];
    uint16_t nb_scan;
    uint8_t threshold_reg = -2 * threshold_dbm;

    nb_scan = (uint16_t)((float)scan_time_us / 8.2 + 0.5);
    printf("===> nb_scan %u\n", nb_scan);

    /* Check radio status */
    buff[0] = 0x00;
    sx1261_reg_r(SX1261_GET_STATUS, buff, 1);
    if (buff[0] != 0x52) {
        printf("ERROR: %s : unexpected status (0x%02X), should be 0x52\n", __FUNCTION__, buff[0]);
        return LGW_REG_ERROR;
    }

    /* Configure LBT scan */
    buff[0] = 11; // intervall_rssi_read (10 => 7.68 usec,11 => 8.2 usec, 12 => 8.68 usec)
    buff[1] = (nb_scan >> 8) & 0xFF;
    buff[2] = (nb_scan >> 0) & 0xFF;
    buff[3] = threshold_reg;
    buff[4] = 1; // gpioId
    sx1261_reg_w(0x9a, buff, 5);

    /* Check radio status */
    buff[0] = 0x00;
    sx1261_reg_r( SX1261_GET_STATUS, buff, 1);
    if (buff[0] != 0xD2) {
        printf("ERROR: %s : unexpected status (0x%02X), should be 0xD2\n", __FUNCTION__, buff[0]);
        return LGW_REG_ERROR;
    }

    /* Wait for Scan Time before TX trigger request */
    wait_us(scan_time_us);

    printf("SX1261: LBT started: scan time = %uus, threshold = %ddBm\n", scan_time_us, threshold_dbm);

    return LGW_REG_SUCCESS;

}

/* --- EOF ------------------------------------------------------------------ */
