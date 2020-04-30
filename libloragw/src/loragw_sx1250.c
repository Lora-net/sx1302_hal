/*
 / _____)             _              | |
( (____  _____ ____ _| |_ _____  ____| |__
 \____ \| ___ |    (_   _) ___ |/ ___)  _ \
 _____) ) ____| | | || |_| ____( (___| | | |
(______/|_____)_|_|_| \__)_____)\____)_| |_|
  (C)2019 Semtech

Description:
    Functions used to handle LoRa concentrator SX1250 radios.

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

#include "sx1250_com.h"
#include "loragw_reg.h"
#include "loragw_aux.h"
#include "loragw_sx1250.h"

/* -------------------------------------------------------------------------- */
/* --- PRIVATE MACROS ------------------------------------------------------- */

#define ARRAY_SIZE(a) (sizeof(a) / sizeof((a)[0]))
#if DEBUG_RAD == 1
    #define DEBUG_MSG(str)                fprintf(stderr, str)
    #define DEBUG_PRINTF(fmt, args...)    fprintf(stderr,"%s:%d: "fmt, __FUNCTION__, __LINE__, args)
    #define CHECK_NULL(a)                if(a==NULL){fprintf(stderr,"%s:%d: ERROR: NULL POINTER AS ARGUMENT\n", __FUNCTION__, __LINE__);return LGW_SPI_ERROR;}
#else
    #define DEBUG_MSG(str)
    #define DEBUG_PRINTF(fmt, args...)
    #define CHECK_NULL(a)                if(a==NULL){return LGW_SPI_ERROR;}
#endif

/* -------------------------------------------------------------------------- */
/* --- PRIVATE CONSTANTS ---------------------------------------------------- */

/* -------------------------------------------------------------------------- */
/* --- INTERNAL SHARED VARIABLES -------------------------------------------- */

/* -------------------------------------------------------------------------- */
/* --- PUBLIC FUNCTIONS DEFINITION ------------------------------------------ */

int sx1250_calibrate(uint8_t rf_chain, uint32_t freq_hz) {
    uint8_t buff[16];

    buff[0] = 0x00;
    sx1250_com_r(rf_chain, GET_STATUS, buff, 1);

    /* Run calibration */
    if ((freq_hz > 430E6) && (freq_hz < 440E6)) {
        buff[0] = 0x6B;
        buff[1] = 0x6F;
    } else if ((freq_hz > 470E6) && (freq_hz < 510E6)) {
        buff[0] = 0x75;
        buff[1] = 0x81;
    } else if ((freq_hz > 779E6) && (freq_hz < 787E6)) {
        buff[0] = 0xC1;
        buff[1] = 0xC5;
    } else if ((freq_hz > 863E6) && (freq_hz < 870E6)) {
        buff[0] = 0xD7;
        buff[1] = 0xDB;
    } else if ((freq_hz > 902E6) && (freq_hz < 928E6)) {
        buff[0] = 0xE1;
        buff[1] = 0xE9;
    } else {
        printf("ERROR: failed to calibrate sx1250 radio, frequency range not supported (%u)\n", freq_hz);
        return -1;
    }
    sx1250_com_w(rf_chain, CALIBRATE_IMAGE, buff, 2);

    /* Wait for calibration to complete */
    wait_ms(10);

    buff[0] = 0x00;
    buff[1] = 0x00;
    buff[2] = 0x00;
    sx1250_com_r(rf_chain, GET_DEVICE_ERRORS, buff, 3);
    if (TAKE_N_BITS_FROM(buff[2], 4, 1) != 0) {
        printf("ERROR: sx1250 Image Calibration Error\n");
        return -1;
    }

    return 0;
}

/* ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ */

int sx1250_setup(uint8_t rf_chain, uint32_t freq_hz, bool single_input_mode) {
    int32_t freq_reg;
    uint8_t buff[16];

    /* Set Radio in Standby for calibrations */
    buff[0] = (uint8_t)STDBY_RC;
    sx1250_com_w(rf_chain, SET_STANDBY, buff, 1);
    wait_ms(10);

    /* Get status to check Standby mode has been properly set */
    buff[0] = 0x00;
    sx1250_com_r(rf_chain, GET_STATUS, buff, 1);
    if ((uint8_t)(TAKE_N_BITS_FROM(buff[0], 4, 3)) != 0x02) {
        printf("ERROR: Failed to set SX1250_%u in STANDBY_RC mode\n", rf_chain);
        return -1;
    }

    /* Run all calibrations (TCXO) */
    buff[0] = 0x7F;
    sx1250_com_w(rf_chain, CALIBRATE, buff, 1);
    wait_ms(10);

    /* Set Radio in Standby with XOSC ON */
    buff[0] = (uint8_t)STDBY_XOSC;
    sx1250_com_w(rf_chain, SET_STANDBY, buff, 1);
    wait_ms(10);

    /* Get status to check Standby mode has been properly set */
    buff[0] = 0x00;
    sx1250_com_r(rf_chain, GET_STATUS, buff, 1);
    if ((uint8_t)(TAKE_N_BITS_FROM(buff[0], 4, 3)) != 0x03) {
        printf("ERROR: Failed to set SX1250_%u in STANDBY_XOSC mode\n", rf_chain);
        return -1;
    }

    /* Set Bitrate to maximum (to lower TX to FS switch time) */
    buff[0] = 0x06;
    buff[1] = 0xA1;
    buff[2] = 0x01;
    sx1250_com_w(rf_chain, WRITE_REGISTER, buff, 3);
    buff[0] = 0x06;
    buff[1] = 0xA2;
    buff[2] = 0x00;
    sx1250_com_w(rf_chain, WRITE_REGISTER, buff, 3);
    buff[0] = 0x06;
    buff[1] = 0xA3;
    buff[2] = 0x00;
    sx1250_com_w(rf_chain, WRITE_REGISTER, buff, 3);

    /* Configure DIO for Rx */
    buff[0] = 0x05;
    buff[1] = 0x82;
    buff[2] = 0x00;
    sx1250_com_w(rf_chain, WRITE_REGISTER, buff, 3); /* Drive strength to min */
    buff[0] = 0x05;
    buff[1] = 0x83;
    buff[2] = 0x00;
    sx1250_com_w(rf_chain, WRITE_REGISTER, buff, 3); /* Input enable, all disabled */
    buff[0] = 0x05;
    buff[1] = 0x84;
    buff[2] = 0x00;
    sx1250_com_w(rf_chain, WRITE_REGISTER, buff, 3); /* No pull up */
    buff[0] = 0x05;
    buff[1] = 0x85;
    buff[2] = 0x00;
    sx1250_com_w(rf_chain, WRITE_REGISTER, buff, 3); /* No pull down */
    buff[0] = 0x05;
    buff[1] = 0x80;
    buff[2] = 0x00;
    sx1250_com_w(rf_chain, WRITE_REGISTER, buff, 3); /* Output enable, all enabled */

    /* Set fix gain (??) */
    buff[0] = 0x08;
    buff[1] = 0xB6;
    buff[2] = 0x2A;
    sx1250_com_w(rf_chain, WRITE_REGISTER, buff, 3);

    /* Set frequency */
    freq_reg = SX1250_FREQ_TO_REG(freq_hz);
    buff[0] = (uint8_t)(freq_reg >> 24);
    buff[1] = (uint8_t)(freq_reg >> 16);
    buff[2] = (uint8_t)(freq_reg >> 8);
    buff[3] = (uint8_t)(freq_reg >> 0);
    sx1250_com_w(rf_chain, SET_RF_FREQUENCY, buff, 4);

    /* Set frequency offset to 0 */
    buff[0] = 0x08;
    buff[1] = 0x8F;
    buff[2] = 0x00;
    buff[3] = 0x00;
    buff[4] = 0x00;
    sx1250_com_w(rf_chain, WRITE_REGISTER, buff, 5);

    /* Set Radio in Rx mode, necessary to give a clock to SX1302 */
    buff[0] = 0xFF;
    buff[1] = 0xFF;
    buff[2] = 0xFF;
    sx1250_com_w(rf_chain, SET_RX, buff, 3); /* Rx Continuous */

    /* Select single input or differential input mode */
    if (single_input_mode == true) {
        printf("INFO: Configuring SX1250_%u in single input mode\n", rf_chain);
        buff[0] = 0x08;
        buff[1] = 0xE2;
        buff[2] = 0x0D;
        sx1250_com_w(rf_chain, WRITE_REGISTER, buff, 3);
    }

    buff[0] = 0x05;
    buff[1] = 0x87;
    buff[2] = 0x0B;
    sx1250_com_w(rf_chain, WRITE_REGISTER, buff, 3); /* FPGA_MODE_RX */

    return 0;
}

/* --- EOF ------------------------------------------------------------------ */
