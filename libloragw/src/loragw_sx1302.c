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
#include <stdio.h>      /* printf fprintf */
#include <stdlib.h>     /* malloc free */
#include <unistd.h>     /* lseek, close */
#include <fcntl.h>      /* open */
#include <string.h>     /* memset */

#include <sys/ioctl.h>
#include <linux/spi/spidev.h>

#include "loragw_reg.h"
#include "loragw_aux.h"
#include "loragw_hal.h"
#include "loragw_sx1302.h"

/* -------------------------------------------------------------------------- */
/* --- PRIVATE MACROS ------------------------------------------------------- */

#define ARRAY_SIZE(a) (sizeof(a) / sizeof((a)[0]))
#if DEBUG_REG == 1
    #define DEBUG_MSG(str)                fprintf(stderr, str)
    #define DEBUG_PRINTF(fmt, args...)    fprintf(stderr,"%s:%d: "fmt, __FUNCTION__, __LINE__, args)
    #define CHECK_NULL(a)                if(a==NULL){fprintf(stderr,"%s:%d: ERROR: NULL POINTER AS ARGUMENT\n", __FUNCTION__, __LINE__);return LGW_SPI_ERROR;}
#else
    #define DEBUG_MSG(str)
    #define DEBUG_PRINTF(fmt, args...)
    #define CHECK_NULL(a)                if(a==NULL){return LGW_SPI_ERROR;}
#endif

#define IF_HZ_TO_REG(f)             ((f << 5) / 15625)

/* -------------------------------------------------------------------------- */
/* --- PRIVATE CONSTANTS ---------------------------------------------------- */

/* -------------------------------------------------------------------------- */
/* --- INTERNAL SHARED VARIABLES -------------------------------------------- */

/* -------------------------------------------------------------------------- */
/* --- PUBLIC FUNCTIONS DEFINITION ------------------------------------------ */

int sx1302_radio_clock_select(uint8_t rf_chain) {
    /* Check input parameters */
    if (rf_chain >= LGW_RF_CHAIN_NB)
    {
        DEBUG_MSG("ERROR: invalid RF chain\n");
        return LGW_REG_ERROR;
    }

    /* Switch SX1302 clock from SPI clock to radio clock of the selected RF chain */
    switch (rf_chain) {
        case 0:
            printf("Select Radio A clock\n");
            lgw_reg_w(SX1302_REG_CLK_CTRL_CLK_SEL_CLK_RADIO_A_SEL, 0x01);
            lgw_reg_w(SX1302_REG_CLK_CTRL_CLK_SEL_CLK_RADIO_B_SEL, 0x00);
            break;
        case 1:
            printf("Select Radio B clock\n");
            lgw_reg_w(SX1302_REG_CLK_CTRL_CLK_SEL_CLK_RADIO_A_SEL, 0x00);
            lgw_reg_w(SX1302_REG_CLK_CTRL_CLK_SEL_CLK_RADIO_B_SEL, 0x01);
            break;
        default:
            return LGW_REG_ERROR;
    }

    /* Enable clock dividers */
    lgw_reg_w(SX1302_REG_CLK_CTRL_CLK_SEL_CLKDIV_EN, 0x01);

    /* Set the RIF clock to the 32MHz clock of the radio */
    lgw_reg_w(SX1302_REG_COMMON_CTRL0_CLK32_RIF_CTRL, 0x01);

    return LGW_REG_SUCCESS;
}

/* ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ */

int sx1302_radio_reset(uint8_t rf_chain, sx1302_radio_type_t type) {
    uint16_t reg_radio_en;
    uint16_t reg_radio_rst;

    /* Check input parameters */
    if (rf_chain >= LGW_RF_CHAIN_NB)
    {
        DEBUG_MSG("ERROR: invalid RF chain\n");
        return LGW_REG_ERROR;
    }
    if ((type != SX1302_RADIO_TYPE_SX1250) && (type != SX1302_RADIO_TYPE_SX125X))
    {
        DEBUG_MSG("ERROR: invalid radio mode\n");
        return LGW_REG_ERROR;
    }

    /* Enable the radio */
    reg_radio_en = REG_SELECT(rf_chain, SX1302_REG_AGC_MCU_RF_EN_A_RADIO_EN, SX1302_REG_AGC_MCU_RF_EN_B_RADIO_EN);
    lgw_reg_w(reg_radio_en, 0x01);

    /* Select the proper reset sequence depending on the radio type */
    reg_radio_rst = REG_SELECT(rf_chain, SX1302_REG_AGC_MCU_RF_EN_A_RADIO_RST, SX1302_REG_AGC_MCU_RF_EN_B_RADIO_RST);
    lgw_reg_w(reg_radio_rst, 0x01);
    wait_ms(500);
    lgw_reg_w(reg_radio_rst, 0x00);
    wait_ms(10);
    switch (type) {
        case SX1302_RADIO_TYPE_SX125X:
            /* Do nothing */
            DEBUG_PRINTF("INFO: reset sx125x (RADIO_%s) done\n", REG_SELECT(rf_chain, "A", "B"));
            break;
        case SX1302_RADIO_TYPE_SX1250:
            lgw_reg_w(reg_radio_rst, 0x01);
            DEBUG_PRINTF("INFO: reset sx1250 (RADIO_%s) done\n", REG_SELECT(rf_chain, "A", "B"));
            break;
        default:
            return LGW_REG_ERROR;
    }

    return LGW_REG_SUCCESS;
}

/* ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ */

int sx1302_radio_set_mode(uint8_t rf_chain, sx1302_radio_type_t type) {
    uint16_t reg;

    /* Check input parameters */
    if (rf_chain >= LGW_RF_CHAIN_NB)
    {
        DEBUG_MSG("ERROR: invalid RF chain\n");
        return LGW_REG_ERROR;
    }
    if ((type != SX1302_RADIO_TYPE_SX1250) && (type != SX1302_RADIO_TYPE_SX125X))
    {
        DEBUG_MSG("ERROR: invalid radio mode\n");
        return LGW_REG_ERROR;
    }

    /* Set the radio mode */
    reg = REG_SELECT(rf_chain,  SX1302_REG_COMMON_CTRL0_SX1261_MODE_RADIO_A,
                                SX1302_REG_COMMON_CTRL0_SX1261_MODE_RADIO_B);
    switch (type) {
        case SX1302_RADIO_TYPE_SX1250:
            printf("Setting rf_chain_%u in sx1250 mode\n", rf_chain);
            lgw_reg_w(reg, 0x01);
            break;
        default:
            printf("Setting rf_chain_%u in sx125x mode\n", rf_chain);
            lgw_reg_w(reg, 0x00);
            break;
    }

    return LGW_REG_SUCCESS;
}

/* ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ */

int sx1302_clock_enable(void) {
    return LGW_REG_SUCCESS;
}

/* ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ */

int sx1302_channelizer_configure(bool * if_rf_chain, int32_t * channel_if) {
    int32_t if_freq;
    uint8_t channels_mask = 0x00;
    int i;

    for (i = 0; i < 8; i++) {
        channels_mask |= (if_rf_chain[i] << i);
    }
    printf("RX radio select: 0x%02X\n", channels_mask);
    lgw_reg_w(SX1302_REG_RX_TOP_RADIO_SELECT_RADIO_SELECT, channels_mask);

    if_freq = IF_HZ_TO_REG(channel_if[0]);
    lgw_reg_w(SX1302_REG_RX_TOP_FREQ_0_MSB_IF_FREQ_0, (if_freq >> 8) & 0x0000001F);
    lgw_reg_w(SX1302_REG_RX_TOP_FREQ_0_LSB_IF_FREQ_0, (if_freq >> 0) & 0x000000FF);

    if_freq = IF_HZ_TO_REG(channel_if[1]);
    lgw_reg_w(SX1302_REG_RX_TOP_FREQ_1_MSB_IF_FREQ_1, (if_freq >> 8) & 0x0000001F);
    lgw_reg_w(SX1302_REG_RX_TOP_FREQ_1_LSB_IF_FREQ_1, (if_freq >> 0) & 0x000000FF);

    if_freq = IF_HZ_TO_REG(channel_if[2]);
    lgw_reg_w(SX1302_REG_RX_TOP_FREQ_2_MSB_IF_FREQ_2, (if_freq >> 8) & 0x0000001F);
    lgw_reg_w(SX1302_REG_RX_TOP_FREQ_2_LSB_IF_FREQ_2, (if_freq >> 0) & 0x000000FF);

    if_freq = IF_HZ_TO_REG(channel_if[3]);
    lgw_reg_w(SX1302_REG_RX_TOP_FREQ_3_MSB_IF_FREQ_3, (if_freq >> 8) & 0x0000001F);
    lgw_reg_w(SX1302_REG_RX_TOP_FREQ_3_LSB_IF_FREQ_3, (if_freq >> 0) & 0x000000FF);

    if_freq = IF_HZ_TO_REG(channel_if[4]);
    lgw_reg_w(SX1302_REG_RX_TOP_FREQ_4_MSB_IF_FREQ_4, (if_freq >> 8) & 0x0000001F);
    lgw_reg_w(SX1302_REG_RX_TOP_FREQ_4_LSB_IF_FREQ_4, (if_freq >> 0) & 0x000000FF);

    if_freq = IF_HZ_TO_REG(channel_if[5]);
    lgw_reg_w(SX1302_REG_RX_TOP_FREQ_5_MSB_IF_FREQ_5, (if_freq >> 8) & 0x0000001F);
    lgw_reg_w(SX1302_REG_RX_TOP_FREQ_5_LSB_IF_FREQ_5, (if_freq >> 0) & 0x000000FF);

    if_freq = IF_HZ_TO_REG(channel_if[6]);
    lgw_reg_w(SX1302_REG_RX_TOP_FREQ_6_MSB_IF_FREQ_6, (if_freq >> 8) & 0x0000001F);
    lgw_reg_w(SX1302_REG_RX_TOP_FREQ_6_LSB_IF_FREQ_6, (if_freq >> 0) & 0x000000FF);

    if_freq = IF_HZ_TO_REG(channel_if[7]);
    lgw_reg_w(SX1302_REG_RX_TOP_FREQ_7_MSB_IF_FREQ_7, (if_freq >> 8) & 0x0000001F);
    lgw_reg_w(SX1302_REG_RX_TOP_FREQ_7_LSB_IF_FREQ_7, (if_freq >> 0) & 0x000000FF);

    return LGW_REG_SUCCESS;
}

/* ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ */

int sx1302_correlator_configure() {
    lgw_reg_w(SX1302_REG_RX_TOP_SF5_CFG1_ACC_2_SAME_PEAKS, 0);
    lgw_reg_w(SX1302_REG_RX_TOP_SF5_CFG1_ACC_AUTO_RESCALE, 1);
    lgw_reg_w(SX1302_REG_RX_TOP_SF5_CFG1_ACC_COEFF, 2);
    lgw_reg_w(SX1302_REG_RX_TOP_SF5_CFG1_ACC_PEAK_POS_SEL, 1);
    lgw_reg_w(SX1302_REG_RX_TOP_SF5_CFG1_ACC_PEAK_SUM_EN, 1);
    lgw_reg_w(SX1302_REG_RX_TOP_SF5_CFG3_MIN_SINGLE_PEAK, 11);
    lgw_reg_w(SX1302_REG_RX_TOP_SF5_CFG6_MSP_CNT_MODE, 0);
    lgw_reg_w(SX1302_REG_RX_TOP_SF5_CFG6_MSP_POS_SEL, 1);
    lgw_reg_w(SX1302_REG_RX_TOP_SF5_CFG7_NOISE_COEFF, 2);
    lgw_reg_w(SX1302_REG_RX_TOP_SF5_CFG2_ACC_PNR, 55);
    lgw_reg_w(SX1302_REG_RX_TOP_SF5_CFG4_MSP_PNR, 55);
    lgw_reg_w(SX1302_REG_RX_TOP_SF5_CFG5_MSP2_PNR, 55);
    lgw_reg_w(SX1302_REG_RX_TOP_SF5_CFG6_MSP_PEAK_NB, 5);
    lgw_reg_w(SX1302_REG_RX_TOP_SF5_CFG7_MSP2_PEAK_NB, 5);

    lgw_reg_w(SX1302_REG_RX_TOP_SF6_CFG1_ACC_2_SAME_PEAKS, 0);
    lgw_reg_w(SX1302_REG_RX_TOP_SF6_CFG1_ACC_AUTO_RESCALE, 1);
    lgw_reg_w(SX1302_REG_RX_TOP_SF6_CFG1_ACC_COEFF, 2);
    lgw_reg_w(SX1302_REG_RX_TOP_SF6_CFG1_ACC_PEAK_POS_SEL, 1);
    lgw_reg_w(SX1302_REG_RX_TOP_SF6_CFG1_ACC_PEAK_SUM_EN, 1);
    lgw_reg_w(SX1302_REG_RX_TOP_SF6_CFG3_MIN_SINGLE_PEAK, 11);
    lgw_reg_w(SX1302_REG_RX_TOP_SF6_CFG6_MSP_CNT_MODE, 0);
    lgw_reg_w(SX1302_REG_RX_TOP_SF6_CFG6_MSP_POS_SEL, 1);
    lgw_reg_w(SX1302_REG_RX_TOP_SF6_CFG7_NOISE_COEFF, 2);
    lgw_reg_w(SX1302_REG_RX_TOP_SF6_CFG2_ACC_PNR, 55);
    lgw_reg_w(SX1302_REG_RX_TOP_SF6_CFG4_MSP_PNR, 55);
    lgw_reg_w(SX1302_REG_RX_TOP_SF6_CFG5_MSP2_PNR, 55);
    lgw_reg_w(SX1302_REG_RX_TOP_SF6_CFG6_MSP_PEAK_NB, 4);
    lgw_reg_w(SX1302_REG_RX_TOP_SF6_CFG7_MSP2_PEAK_NB, 4);

    lgw_reg_w(SX1302_REG_RX_TOP_SF7_CFG1_ACC_2_SAME_PEAKS, 1);
    lgw_reg_w(SX1302_REG_RX_TOP_SF7_CFG1_ACC_AUTO_RESCALE, 1);
    lgw_reg_w(SX1302_REG_RX_TOP_SF7_CFG1_ACC_COEFF, 2);
    lgw_reg_w(SX1302_REG_RX_TOP_SF7_CFG1_ACC_PEAK_POS_SEL, 1);
    lgw_reg_w(SX1302_REG_RX_TOP_SF7_CFG1_ACC_PEAK_SUM_EN, 1);
    lgw_reg_w(SX1302_REG_RX_TOP_SF7_CFG3_MIN_SINGLE_PEAK, 11);
    lgw_reg_w(SX1302_REG_RX_TOP_SF7_CFG6_MSP_CNT_MODE, 0);
    lgw_reg_w(SX1302_REG_RX_TOP_SF7_CFG6_MSP_POS_SEL, 1);
    lgw_reg_w(SX1302_REG_RX_TOP_SF7_CFG7_NOISE_COEFF, 2);
    lgw_reg_w(SX1302_REG_RX_TOP_SF7_CFG2_ACC_PNR, 55);
    lgw_reg_w(SX1302_REG_RX_TOP_SF7_CFG4_MSP_PNR, 55);
    lgw_reg_w(SX1302_REG_RX_TOP_SF7_CFG5_MSP2_PNR, 55);
    lgw_reg_w(SX1302_REG_RX_TOP_SF7_CFG6_MSP_PEAK_NB, 3);
    lgw_reg_w(SX1302_REG_RX_TOP_SF7_CFG7_MSP2_PEAK_NB, 3);

    lgw_reg_w(SX1302_REG_RX_TOP_SF8_CFG1_ACC_2_SAME_PEAKS, 0);
    lgw_reg_w(SX1302_REG_RX_TOP_SF8_CFG1_ACC_AUTO_RESCALE, 1);
    lgw_reg_w(SX1302_REG_RX_TOP_SF8_CFG1_ACC_COEFF, 2);
    lgw_reg_w(SX1302_REG_RX_TOP_SF8_CFG1_ACC_PEAK_POS_SEL, 1);
    lgw_reg_w(SX1302_REG_RX_TOP_SF8_CFG1_ACC_PEAK_SUM_EN, 1);
    lgw_reg_w(SX1302_REG_RX_TOP_SF8_CFG3_MIN_SINGLE_PEAK, 11);
    lgw_reg_w(SX1302_REG_RX_TOP_SF8_CFG6_MSP_CNT_MODE, 0);
    lgw_reg_w(SX1302_REG_RX_TOP_SF8_CFG6_MSP_POS_SEL, 1);
    lgw_reg_w(SX1302_REG_RX_TOP_SF8_CFG7_NOISE_COEFF, 2);
    lgw_reg_w(SX1302_REG_RX_TOP_SF8_CFG2_ACC_PNR, 56);
    lgw_reg_w(SX1302_REG_RX_TOP_SF8_CFG4_MSP_PNR, 56);
    lgw_reg_w(SX1302_REG_RX_TOP_SF8_CFG5_MSP2_PNR, 56);
    lgw_reg_w(SX1302_REG_RX_TOP_SF8_CFG6_MSP_PEAK_NB, 3);
    lgw_reg_w(SX1302_REG_RX_TOP_SF8_CFG7_MSP2_PEAK_NB, 3);

    lgw_reg_w(SX1302_REG_RX_TOP_SF9_CFG1_ACC_2_SAME_PEAKS, 0);
    lgw_reg_w(SX1302_REG_RX_TOP_SF9_CFG1_ACC_AUTO_RESCALE, 1);
    lgw_reg_w(SX1302_REG_RX_TOP_SF9_CFG1_ACC_COEFF, 2);
    lgw_reg_w(SX1302_REG_RX_TOP_SF9_CFG1_ACC_PEAK_POS_SEL, 1);
    lgw_reg_w(SX1302_REG_RX_TOP_SF9_CFG1_ACC_PEAK_SUM_EN, 1);
    lgw_reg_w(SX1302_REG_RX_TOP_SF9_CFG3_MIN_SINGLE_PEAK, 11);
    lgw_reg_w(SX1302_REG_RX_TOP_SF9_CFG6_MSP_CNT_MODE, 0);
    lgw_reg_w(SX1302_REG_RX_TOP_SF9_CFG6_MSP_POS_SEL, 1);
    lgw_reg_w(SX1302_REG_RX_TOP_SF9_CFG7_NOISE_COEFF, 2);
    lgw_reg_w(SX1302_REG_RX_TOP_SF9_CFG2_ACC_PNR, 58);
    lgw_reg_w(SX1302_REG_RX_TOP_SF9_CFG4_MSP_PNR, 58);
    lgw_reg_w(SX1302_REG_RX_TOP_SF9_CFG5_MSP2_PNR, 58);
    lgw_reg_w(SX1302_REG_RX_TOP_SF9_CFG6_MSP_PEAK_NB, 3);
    lgw_reg_w(SX1302_REG_RX_TOP_SF9_CFG7_MSP2_PEAK_NB, 3);

    lgw_reg_w(SX1302_REG_RX_TOP_SF10_CFG1_ACC_2_SAME_PEAKS, 0);
    lgw_reg_w(SX1302_REG_RX_TOP_SF10_CFG1_ACC_AUTO_RESCALE, 1);
    lgw_reg_w(SX1302_REG_RX_TOP_SF10_CFG1_ACC_COEFF, 2);
    lgw_reg_w(SX1302_REG_RX_TOP_SF10_CFG1_ACC_PEAK_POS_SEL, 1);
    lgw_reg_w(SX1302_REG_RX_TOP_SF10_CFG1_ACC_PEAK_SUM_EN, 1);
    lgw_reg_w(SX1302_REG_RX_TOP_SF10_CFG3_MIN_SINGLE_PEAK, 11);
    lgw_reg_w(SX1302_REG_RX_TOP_SF10_CFG6_MSP_CNT_MODE, 0);
    lgw_reg_w(SX1302_REG_RX_TOP_SF10_CFG6_MSP_POS_SEL, 1);
    lgw_reg_w(SX1302_REG_RX_TOP_SF10_CFG7_NOISE_COEFF, 2);
    lgw_reg_w(SX1302_REG_RX_TOP_SF10_CFG2_ACC_PNR, 60);
    lgw_reg_w(SX1302_REG_RX_TOP_SF10_CFG4_MSP_PNR, 60);
    lgw_reg_w(SX1302_REG_RX_TOP_SF10_CFG5_MSP2_PNR, 60);
    lgw_reg_w(SX1302_REG_RX_TOP_SF10_CFG6_MSP_PEAK_NB, 3);
    lgw_reg_w(SX1302_REG_RX_TOP_SF10_CFG7_MSP2_PEAK_NB, 3);

    lgw_reg_w(SX1302_REG_RX_TOP_SF11_CFG1_ACC_2_SAME_PEAKS, 0);
    lgw_reg_w(SX1302_REG_RX_TOP_SF11_CFG1_ACC_AUTO_RESCALE, 1);
    lgw_reg_w(SX1302_REG_RX_TOP_SF11_CFG1_ACC_COEFF, 2);
    lgw_reg_w(SX1302_REG_RX_TOP_SF11_CFG1_ACC_PEAK_POS_SEL, 1);
    lgw_reg_w(SX1302_REG_RX_TOP_SF11_CFG1_ACC_PEAK_SUM_EN, 1);
    lgw_reg_w(SX1302_REG_RX_TOP_SF11_CFG3_MIN_SINGLE_PEAK, 11);
    lgw_reg_w(SX1302_REG_RX_TOP_SF11_CFG6_MSP_CNT_MODE, 0);
    lgw_reg_w(SX1302_REG_RX_TOP_SF11_CFG6_MSP_POS_SEL, 1);
    lgw_reg_w(SX1302_REG_RX_TOP_SF11_CFG7_NOISE_COEFF, 2);
    lgw_reg_w(SX1302_REG_RX_TOP_SF11_CFG2_ACC_PNR, 60);
    lgw_reg_w(SX1302_REG_RX_TOP_SF11_CFG4_MSP_PNR, 60);
    lgw_reg_w(SX1302_REG_RX_TOP_SF11_CFG5_MSP2_PNR, 60);
    lgw_reg_w(SX1302_REG_RX_TOP_SF11_CFG6_MSP_PEAK_NB, 3);
    lgw_reg_w(SX1302_REG_RX_TOP_SF11_CFG7_MSP2_PEAK_NB, 3);

    lgw_reg_w(SX1302_REG_RX_TOP_SF12_CFG1_ACC_2_SAME_PEAKS, 0);
    lgw_reg_w(SX1302_REG_RX_TOP_SF12_CFG1_ACC_AUTO_RESCALE, 1);
    lgw_reg_w(SX1302_REG_RX_TOP_SF12_CFG1_ACC_COEFF, 2);
    lgw_reg_w(SX1302_REG_RX_TOP_SF12_CFG1_ACC_PEAK_POS_SEL, 1);
    lgw_reg_w(SX1302_REG_RX_TOP_SF12_CFG1_ACC_PEAK_SUM_EN, 1);
    lgw_reg_w(SX1302_REG_RX_TOP_SF12_CFG3_MIN_SINGLE_PEAK, 11);
    lgw_reg_w(SX1302_REG_RX_TOP_SF12_CFG6_MSP_CNT_MODE, 0);
    lgw_reg_w(SX1302_REG_RX_TOP_SF12_CFG6_MSP_POS_SEL, 1);
    lgw_reg_w(SX1302_REG_RX_TOP_SF12_CFG7_NOISE_COEFF, 2);
    lgw_reg_w(SX1302_REG_RX_TOP_SF12_CFG2_ACC_PNR, 60);
    lgw_reg_w(SX1302_REG_RX_TOP_SF12_CFG4_MSP_PNR, 60);
    lgw_reg_w(SX1302_REG_RX_TOP_SF12_CFG5_MSP2_PNR, 60);
    lgw_reg_w(SX1302_REG_RX_TOP_SF12_CFG6_MSP_PEAK_NB, 3);
    lgw_reg_w(SX1302_REG_RX_TOP_SF12_CFG7_MSP2_PEAK_NB, 3);

    lgw_reg_w(SX1302_REG_RX_TOP_CORRELATOR_SF_EN_CORR_SF_EN, 0xFC); /* 12 11 10 9 8 7 6 5 */
    lgw_reg_w(SX1302_REG_RX_TOP_CORRELATOR_EN_CORR_EN, 0xFF); /* 1 correlator per channel */

    return LGW_REG_SUCCESS;
}

/* ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ */

int sx1302_modem_configure() {
    lgw_reg_w(SX1302_REG_RX_TOP_DC_NOTCH_CFG1_ENABLE, 0x00);
    lgw_reg_w(SX1302_REG_RX_TOP_RX_DFE_AGC1_FREEZE_ON_SYNC, 0x00);
    lgw_reg_w(SX1302_REG_RX_TOP_RX_DFE_AGC1_FORCE_DEFAULT_FIR, 0x01);
    lgw_reg_w(SX1302_REG_RX_TOP_RX_CFG0_SWAP_IQ, 0x00);
    lgw_reg_w(SX1302_REG_RX_TOP_RX_CFG0_CHIRP_INVERT, 0x01);
    lgw_reg_w(SX1302_REG_RX_TOP_MODEM_SYNC_DELTA_LSB_MODEM_SYNC_DELTA, 7);
    lgw_reg_w(SX1302_REG_RX_TOP_MODEM_SYNC_DELTA_MSB_MODEM_SYNC_DELTA, 0);
    lgw_reg_w(SX1302_REG_OTP_MODEM_EN_0_MODEM_EN, 0xFF); /* Only 2 full modems available on FPGA */
    lgw_reg_w(SX1302_REG_OTP_MODEM_EN_1_MODEM_EN, 0x0F); /* No limited modem available */

    lgw_reg_w(SX1302_REG_RX_TOP_MODEM_PPM_OFFSET1_PPM_OFFSET_SF5, 0x00);
    lgw_reg_w(SX1302_REG_RX_TOP_MODEM_PPM_OFFSET1_PPM_OFFSET_SF6, 0x00);
    lgw_reg_w(SX1302_REG_RX_TOP_MODEM_PPM_OFFSET1_PPM_OFFSET_SF7, 0x00);
    lgw_reg_w(SX1302_REG_RX_TOP_MODEM_PPM_OFFSET1_PPM_OFFSET_SF8, 0x00);
    lgw_reg_w(SX1302_REG_RX_TOP_MODEM_PPM_OFFSET2_PPM_OFFSET_SF9, 0x00);
    lgw_reg_w(SX1302_REG_RX_TOP_MODEM_PPM_OFFSET2_PPM_OFFSET_SF10, 0x00);
    lgw_reg_w(SX1302_REG_RX_TOP_MODEM_PPM_OFFSET2_PPM_OFFSET_SF11, 0x01);
    lgw_reg_w(SX1302_REG_RX_TOP_MODEM_PPM_OFFSET2_PPM_OFFSET_SF12, 0x01);

    return LGW_REG_SUCCESS;
}

/* ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ */

int sx1302_modem_enable() {
    lgw_reg_w(SX1302_REG_COMMON_GEN_CONCENTRATOR_MODEM_ENABLE, 0x01);
    lgw_reg_w(SX1302_REG_COMMON_GEN_FSK_MODEM_ENABLE, 0x01);
    lgw_reg_w(SX1302_REG_COMMON_GEN_GLOBAL_EN, 0x01);

    return LGW_REG_SUCCESS;
}

/* ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ */

int sx1302_lora_syncword(bool public) {
    if (public) {
        lgw_reg_w(SX1302_REG_RX_TOP_FRAME_SYNCH0_PEAK1_POS, 6);
        lgw_reg_w(SX1302_REG_RX_TOP_FRAME_SYNCH1_PEAK2_POS, 8);
    } else {
        lgw_reg_w(SX1302_REG_RX_TOP_FRAME_SYNCH0_PEAK1_POS, 2);
        lgw_reg_w(SX1302_REG_RX_TOP_FRAME_SYNCH1_PEAK2_POS, 4);
    }

    return LGW_REG_SUCCESS;
}

/* ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ */

int sx1302_get_cnt(bool pps, uint32_t* cnt_us) {
    int x;
    uint8_t buff[4];

    /* Get the 32MHz timestamp counter - 4 bytes */
    /* step of 31.25 ns */
    x = lgw_reg_rb((pps == true) ? SX1302_REG_TIMESTAMP_TIMESTAMP_PPS_MSB2_TIMESTAMP_PPS : SX1302_REG_TIMESTAMP_TIMESTAMP_MSB2_TIMESTAMP, &buff[0], 4);
    if (x != LGW_REG_SUCCESS) {
        printf("ERROR: Failed to get timestamp counter value\n");
        *cnt_us = 0;
        return LGW_HAL_ERROR;
    }

    *cnt_us  = (uint32_t)((buff[0] << 24) & 0xFF000000);
    *cnt_us |= (uint32_t)((buff[1] << 16) & 0x00FF0000);
    *cnt_us |= (uint32_t)((buff[2] << 8)  & 0x0000FF00);
    *cnt_us |= (uint32_t)((buff[3] << 0)  & 0x000000FF);

    *cnt_us /= 32; /* scale to 1MHz */

    return LGW_HAL_SUCCESS;
}

/* --- EOF ------------------------------------------------------------------ */
