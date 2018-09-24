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

#define IF_HZ_TO_REG(f)     ((f << 5) / 15625)
#define SET_PPM_ON(bw,dr)   (((bw == BW_125KHZ) && ((dr == DR_LORA_SF11) || (dr == DR_LORA_SF12))) || ((bw == BW_250KHZ) && (dr == DR_LORA_SF12)))

/* -------------------------------------------------------------------------- */
/* --- PRIVATE CONSTANTS ---------------------------------------------------- */

#define AGC_RADIO_A_INIT_DONE   0x80
#define AGC_RADIO_B_INIT_DONE   0x20

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

int sx1302_lora_channelizer_configure(bool * if_rf_chain, int32_t * channel_if) {
    int32_t if_freq;
    uint8_t channels_mask = 0x00;
    int i;

    /* Check input parameters */
    if ((if_rf_chain == NULL) || (channel_if == NULL)) {
        printf("ERROR: Failed to configure LoRa channelizer\n");
        return LGW_REG_ERROR;
    }

    /* Configure multi-SF channels */
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

int sx1302_lora_service_channelizer_configure(bool * if_rf_chain, int32_t * channel_if) {
    int32_t if_freq;

    /* Check input parameters */
    if ((if_rf_chain == NULL) || (channel_if == NULL)) {
        printf("ERROR: Failed to configure LoRa service channelizer\n");
        return LGW_REG_ERROR;
    }

    /* Configure LoRa service channel */
    lgw_reg_w(SX1302_REG_RX_TOP_LORA_SERVICE_FSK_LORA_SERVICE_RADIO_SEL_RADIO_SELECT, if_rf_chain[9]);

    if_freq = IF_HZ_TO_REG(channel_if[8]);
    lgw_reg_w(SX1302_REG_RX_TOP_LORA_SERVICE_FSK_LORA_SERVICE_FREQ_MSB_IF_FREQ_0, (if_freq >> 8) & 0x0000001F);
    lgw_reg_w(SX1302_REG_RX_TOP_LORA_SERVICE_FSK_LORA_SERVICE_FREQ_LSB_IF_FREQ_0, (if_freq >> 0) & 0x000000FF);

    return LGW_REG_SUCCESS;
}

/* ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ */

int sx1302_lora_correlator_configure() {
    lgw_reg_w(SX1302_REG_RX_TOP_SF5_CFG1_ACC_2_SAME_PEAKS, 0);
    lgw_reg_w(SX1302_REG_RX_TOP_SF5_CFG1_ACC_AUTO_RESCALE, 1);
    lgw_reg_w(SX1302_REG_RX_TOP_SF5_CFG1_ACC_COEFF, 2);
    lgw_reg_w(SX1302_REG_RX_TOP_SF5_CFG1_ACC_PEAK_POS_SEL, 1);
    lgw_reg_w(SX1302_REG_RX_TOP_SF5_CFG1_ACC_PEAK_SUM_EN, 1);
    lgw_reg_w(SX1302_REG_RX_TOP_SF5_CFG3_MIN_SINGLE_PEAK, 11);
    lgw_reg_w(SX1302_REG_RX_TOP_SF5_CFG6_MSP_CNT_MODE, 0);
    lgw_reg_w(SX1302_REG_RX_TOP_SF5_CFG6_MSP_POS_SEL, 1);
    lgw_reg_w(SX1302_REG_RX_TOP_SF5_CFG7_NOISE_COEFF, 2);
    lgw_reg_w(SX1302_REG_RX_TOP_SF5_CFG2_ACC_PNR, 64);
    lgw_reg_w(SX1302_REG_RX_TOP_SF5_CFG4_MSP_PNR, 32);
    lgw_reg_w(SX1302_REG_RX_TOP_SF5_CFG5_MSP2_PNR, 48);
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
    lgw_reg_w(SX1302_REG_RX_TOP_SF6_CFG2_ACC_PNR, 64);
    lgw_reg_w(SX1302_REG_RX_TOP_SF6_CFG4_MSP_PNR, 32);
    lgw_reg_w(SX1302_REG_RX_TOP_SF6_CFG5_MSP2_PNR, 48);
    lgw_reg_w(SX1302_REG_RX_TOP_SF6_CFG6_MSP_PEAK_NB, 4);
    lgw_reg_w(SX1302_REG_RX_TOP_SF6_CFG7_MSP2_PEAK_NB, 5);

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
    lgw_reg_w(SX1302_REG_RX_TOP_SF7_CFG4_MSP_PNR, 32);
    lgw_reg_w(SX1302_REG_RX_TOP_SF7_CFG5_MSP2_PNR, 48);
    lgw_reg_w(SX1302_REG_RX_TOP_SF7_CFG6_MSP_PEAK_NB, 3);
    lgw_reg_w(SX1302_REG_RX_TOP_SF7_CFG7_MSP2_PEAK_NB, 4);

    lgw_reg_w(SX1302_REG_RX_TOP_SF8_CFG1_ACC_2_SAME_PEAKS, 1);
    lgw_reg_w(SX1302_REG_RX_TOP_SF8_CFG1_ACC_AUTO_RESCALE, 1);
    lgw_reg_w(SX1302_REG_RX_TOP_SF8_CFG1_ACC_COEFF, 2);
    lgw_reg_w(SX1302_REG_RX_TOP_SF8_CFG1_ACC_PEAK_POS_SEL, 1);
    lgw_reg_w(SX1302_REG_RX_TOP_SF8_CFG1_ACC_PEAK_SUM_EN, 1);
    lgw_reg_w(SX1302_REG_RX_TOP_SF8_CFG3_MIN_SINGLE_PEAK, 11);
    lgw_reg_w(SX1302_REG_RX_TOP_SF8_CFG6_MSP_CNT_MODE, 0);
    lgw_reg_w(SX1302_REG_RX_TOP_SF8_CFG6_MSP_POS_SEL, 1);
    lgw_reg_w(SX1302_REG_RX_TOP_SF8_CFG7_NOISE_COEFF, 2);
    lgw_reg_w(SX1302_REG_RX_TOP_SF8_CFG2_ACC_PNR, 64);
    lgw_reg_w(SX1302_REG_RX_TOP_SF8_CFG4_MSP_PNR, 32);
    lgw_reg_w(SX1302_REG_RX_TOP_SF8_CFG5_MSP2_PNR, 48);
    lgw_reg_w(SX1302_REG_RX_TOP_SF8_CFG6_MSP_PEAK_NB, 3);
    lgw_reg_w(SX1302_REG_RX_TOP_SF8_CFG7_MSP2_PEAK_NB, 5);

    lgw_reg_w(SX1302_REG_RX_TOP_SF9_CFG1_ACC_2_SAME_PEAKS, 1);
    lgw_reg_w(SX1302_REG_RX_TOP_SF9_CFG1_ACC_AUTO_RESCALE, 1);
    lgw_reg_w(SX1302_REG_RX_TOP_SF9_CFG1_ACC_COEFF, 2);
    lgw_reg_w(SX1302_REG_RX_TOP_SF9_CFG1_ACC_PEAK_POS_SEL, 1);
    lgw_reg_w(SX1302_REG_RX_TOP_SF9_CFG1_ACC_PEAK_SUM_EN, 1);
    lgw_reg_w(SX1302_REG_RX_TOP_SF9_CFG3_MIN_SINGLE_PEAK, 11);
    lgw_reg_w(SX1302_REG_RX_TOP_SF9_CFG6_MSP_CNT_MODE, 0);
    lgw_reg_w(SX1302_REG_RX_TOP_SF9_CFG6_MSP_POS_SEL, 1);
    lgw_reg_w(SX1302_REG_RX_TOP_SF9_CFG7_NOISE_COEFF, 2);
    lgw_reg_w(SX1302_REG_RX_TOP_SF9_CFG2_ACC_PNR, 64);
    lgw_reg_w(SX1302_REG_RX_TOP_SF9_CFG4_MSP_PNR, 32);
    lgw_reg_w(SX1302_REG_RX_TOP_SF9_CFG5_MSP2_PNR, 48);
    lgw_reg_w(SX1302_REG_RX_TOP_SF9_CFG6_MSP_PEAK_NB, 3);
    lgw_reg_w(SX1302_REG_RX_TOP_SF9_CFG7_MSP2_PEAK_NB, 5);

    lgw_reg_w(SX1302_REG_RX_TOP_SF10_CFG1_ACC_2_SAME_PEAKS, 1);
    lgw_reg_w(SX1302_REG_RX_TOP_SF10_CFG1_ACC_AUTO_RESCALE, 1);
    lgw_reg_w(SX1302_REG_RX_TOP_SF10_CFG1_ACC_COEFF, 2);
    lgw_reg_w(SX1302_REG_RX_TOP_SF10_CFG1_ACC_PEAK_POS_SEL, 1);
    lgw_reg_w(SX1302_REG_RX_TOP_SF10_CFG1_ACC_PEAK_SUM_EN, 1);
    lgw_reg_w(SX1302_REG_RX_TOP_SF10_CFG3_MIN_SINGLE_PEAK, 11);
    lgw_reg_w(SX1302_REG_RX_TOP_SF10_CFG6_MSP_CNT_MODE, 0);
    lgw_reg_w(SX1302_REG_RX_TOP_SF10_CFG6_MSP_POS_SEL, 1);
    lgw_reg_w(SX1302_REG_RX_TOP_SF10_CFG7_NOISE_COEFF, 2);
    lgw_reg_w(SX1302_REG_RX_TOP_SF10_CFG2_ACC_PNR, 64);
    lgw_reg_w(SX1302_REG_RX_TOP_SF10_CFG4_MSP_PNR, 32);
    lgw_reg_w(SX1302_REG_RX_TOP_SF10_CFG5_MSP2_PNR, 48);
    lgw_reg_w(SX1302_REG_RX_TOP_SF10_CFG6_MSP_PEAK_NB, 3);
    lgw_reg_w(SX1302_REG_RX_TOP_SF10_CFG7_MSP2_PEAK_NB, 5);

    lgw_reg_w(SX1302_REG_RX_TOP_SF11_CFG1_ACC_2_SAME_PEAKS, 1);
    lgw_reg_w(SX1302_REG_RX_TOP_SF11_CFG1_ACC_AUTO_RESCALE, 1);
    lgw_reg_w(SX1302_REG_RX_TOP_SF11_CFG1_ACC_COEFF, 2);
    lgw_reg_w(SX1302_REG_RX_TOP_SF11_CFG1_ACC_PEAK_POS_SEL, 1);
    lgw_reg_w(SX1302_REG_RX_TOP_SF11_CFG1_ACC_PEAK_SUM_EN, 1);
    lgw_reg_w(SX1302_REG_RX_TOP_SF11_CFG3_MIN_SINGLE_PEAK, 11);
    lgw_reg_w(SX1302_REG_RX_TOP_SF11_CFG6_MSP_CNT_MODE, 0);
    lgw_reg_w(SX1302_REG_RX_TOP_SF11_CFG6_MSP_POS_SEL, 1);
    lgw_reg_w(SX1302_REG_RX_TOP_SF11_CFG7_NOISE_COEFF, 2);
    lgw_reg_w(SX1302_REG_RX_TOP_SF11_CFG2_ACC_PNR, 64);
    lgw_reg_w(SX1302_REG_RX_TOP_SF11_CFG4_MSP_PNR, 32);
    lgw_reg_w(SX1302_REG_RX_TOP_SF11_CFG5_MSP2_PNR, 48);
    lgw_reg_w(SX1302_REG_RX_TOP_SF11_CFG6_MSP_PEAK_NB, 3);
    lgw_reg_w(SX1302_REG_RX_TOP_SF11_CFG7_MSP2_PEAK_NB, 5);

    lgw_reg_w(SX1302_REG_RX_TOP_SF12_CFG1_ACC_2_SAME_PEAKS, 1);
    lgw_reg_w(SX1302_REG_RX_TOP_SF12_CFG1_ACC_AUTO_RESCALE, 1);
    lgw_reg_w(SX1302_REG_RX_TOP_SF12_CFG1_ACC_COEFF, 2);
    lgw_reg_w(SX1302_REG_RX_TOP_SF12_CFG1_ACC_PEAK_POS_SEL, 1);
    lgw_reg_w(SX1302_REG_RX_TOP_SF12_CFG1_ACC_PEAK_SUM_EN, 1);
    lgw_reg_w(SX1302_REG_RX_TOP_SF12_CFG3_MIN_SINGLE_PEAK, 11);
    lgw_reg_w(SX1302_REG_RX_TOP_SF12_CFG6_MSP_CNT_MODE, 0);
    lgw_reg_w(SX1302_REG_RX_TOP_SF12_CFG6_MSP_POS_SEL, 1);
    lgw_reg_w(SX1302_REG_RX_TOP_SF12_CFG7_NOISE_COEFF, 2);
    lgw_reg_w(SX1302_REG_RX_TOP_SF12_CFG2_ACC_PNR, 64);
    lgw_reg_w(SX1302_REG_RX_TOP_SF12_CFG4_MSP_PNR, 32);
    lgw_reg_w(SX1302_REG_RX_TOP_SF12_CFG5_MSP2_PNR, 48);
    lgw_reg_w(SX1302_REG_RX_TOP_SF12_CFG6_MSP_PEAK_NB, 3);
    lgw_reg_w(SX1302_REG_RX_TOP_SF12_CFG7_MSP2_PEAK_NB, 5);

    lgw_reg_w(SX1302_REG_RX_TOP_CORRELATOR_ENABLE_ONLY_FIRST_DET_EDGE_ENABLE_ONLY_FIRST_DET_EDGE, 0xFF);
    lgw_reg_w(SX1302_REG_RX_TOP_CORRELATOR_ENABLE_ACC_CLEAR_ENABLE_CORR_ACC_CLEAR, 0x00);
    lgw_reg_w(SX1302_REG_RX_TOP_CORRELATOR_SF_EN_CORR_SF_EN, 0xFC); /* 12 11 10 9 8 7 6 5 */
    lgw_reg_w(SX1302_REG_RX_TOP_CORRELATOR_EN_CORR_EN, 0xFF); /* 1 correlator per channel */

    lgw_reg_w(SX1302_REG_RX_TOP_CHANN_DAGC_CFG5_CHAN_DAGC_MODE, 0x01); /* Enable software AGC mode */

    return LGW_REG_SUCCESS;
}

/* ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ */

int sx1302_lora_service_correlator_configure(uint8_t sf) {
    switch (sf) {
        case 5:
            lgw_reg_w(SX1302_REG_RX_TOP_LORA_SERVICE_FSK_TXRX_CFG2_FINE_SYNCH_EN, 1);
            lgw_reg_w(SX1302_REG_RX_TOP_LORA_SERVICE_FSK_DETECT_ACC1_ACC_PNR, 55);
            lgw_reg_w(SX1302_REG_RX_TOP_LORA_SERVICE_FSK_DETECT_MSP0_MSP_PNR, 55);
            lgw_reg_w(SX1302_REG_RX_TOP_LORA_SERVICE_FSK_DETECT_MSP1_MSP2_PNR, 55);
            lgw_reg_w(SX1302_REG_RX_TOP_LORA_SERVICE_FSK_DETECT_MSP2_MSP_PEAK_NB, 5);
            lgw_reg_w(SX1302_REG_RX_TOP_LORA_SERVICE_FSK_DETECT_MSP2_MSP2_PEAK_NB, 5);
            break;
        case 6:
            lgw_reg_w(SX1302_REG_RX_TOP_LORA_SERVICE_FSK_TXRX_CFG2_FINE_SYNCH_EN, 1);
            lgw_reg_w(SX1302_REG_RX_TOP_LORA_SERVICE_FSK_DETECT_ACC1_ACC_PNR, 55);
            lgw_reg_w(SX1302_REG_RX_TOP_LORA_SERVICE_FSK_DETECT_MSP0_MSP_PNR, 55);
            lgw_reg_w(SX1302_REG_RX_TOP_LORA_SERVICE_FSK_DETECT_MSP1_MSP2_PNR, 55);
            lgw_reg_w(SX1302_REG_RX_TOP_LORA_SERVICE_FSK_DETECT_MSP2_MSP_PEAK_NB, 4);
            lgw_reg_w(SX1302_REG_RX_TOP_LORA_SERVICE_FSK_DETECT_MSP2_MSP2_PEAK_NB, 4);
            break;
        case 7:
            lgw_reg_w(SX1302_REG_RX_TOP_LORA_SERVICE_FSK_TXRX_CFG2_FINE_SYNCH_EN, 0);
            lgw_reg_w(SX1302_REG_RX_TOP_LORA_SERVICE_FSK_DETECT_ACC1_ACC_PNR, 55);
            lgw_reg_w(SX1302_REG_RX_TOP_LORA_SERVICE_FSK_DETECT_MSP0_MSP_PNR, 55);
            lgw_reg_w(SX1302_REG_RX_TOP_LORA_SERVICE_FSK_DETECT_MSP1_MSP2_PNR, 55);
            lgw_reg_w(SX1302_REG_RX_TOP_LORA_SERVICE_FSK_DETECT_MSP2_MSP_PEAK_NB, 3);
            lgw_reg_w(SX1302_REG_RX_TOP_LORA_SERVICE_FSK_DETECT_MSP2_MSP2_PEAK_NB, 3);
            break;
        case 8:
            lgw_reg_w(SX1302_REG_RX_TOP_LORA_SERVICE_FSK_TXRX_CFG2_FINE_SYNCH_EN, 0);
            lgw_reg_w(SX1302_REG_RX_TOP_LORA_SERVICE_FSK_DETECT_ACC1_ACC_PNR, 56);
            lgw_reg_w(SX1302_REG_RX_TOP_LORA_SERVICE_FSK_DETECT_MSP0_MSP_PNR, 56);
            lgw_reg_w(SX1302_REG_RX_TOP_LORA_SERVICE_FSK_DETECT_MSP1_MSP2_PNR, 56);
            lgw_reg_w(SX1302_REG_RX_TOP_LORA_SERVICE_FSK_DETECT_MSP2_MSP_PEAK_NB, 3);
            lgw_reg_w(SX1302_REG_RX_TOP_LORA_SERVICE_FSK_DETECT_MSP2_MSP2_PEAK_NB, 3);
            break;
        case 9:
            lgw_reg_w(SX1302_REG_RX_TOP_LORA_SERVICE_FSK_TXRX_CFG2_FINE_SYNCH_EN, 0);
            lgw_reg_w(SX1302_REG_RX_TOP_LORA_SERVICE_FSK_DETECT_ACC1_ACC_PNR, 58);
            lgw_reg_w(SX1302_REG_RX_TOP_LORA_SERVICE_FSK_DETECT_MSP0_MSP_PNR, 58);
            lgw_reg_w(SX1302_REG_RX_TOP_LORA_SERVICE_FSK_DETECT_MSP1_MSP2_PNR, 58);
            lgw_reg_w(SX1302_REG_RX_TOP_LORA_SERVICE_FSK_DETECT_MSP2_MSP_PEAK_NB, 3);
            lgw_reg_w(SX1302_REG_RX_TOP_LORA_SERVICE_FSK_DETECT_MSP2_MSP2_PEAK_NB, 3);
            break;
        case 10:
            lgw_reg_w(SX1302_REG_RX_TOP_LORA_SERVICE_FSK_TXRX_CFG2_FINE_SYNCH_EN, 0);
            lgw_reg_w(SX1302_REG_RX_TOP_LORA_SERVICE_FSK_DETECT_ACC1_ACC_PNR, 60);
            lgw_reg_w(SX1302_REG_RX_TOP_LORA_SERVICE_FSK_DETECT_MSP0_MSP_PNR, 60);
            lgw_reg_w(SX1302_REG_RX_TOP_LORA_SERVICE_FSK_DETECT_MSP1_MSP2_PNR, 60);
            lgw_reg_w(SX1302_REG_RX_TOP_LORA_SERVICE_FSK_DETECT_MSP2_MSP_PEAK_NB, 3);
            lgw_reg_w(SX1302_REG_RX_TOP_LORA_SERVICE_FSK_DETECT_MSP2_MSP2_PEAK_NB, 3);
            break;
        case 11:
            lgw_reg_w(SX1302_REG_RX_TOP_LORA_SERVICE_FSK_TXRX_CFG2_FINE_SYNCH_EN, 0);
            lgw_reg_w(SX1302_REG_RX_TOP_LORA_SERVICE_FSK_DETECT_ACC1_ACC_PNR, 60);
            lgw_reg_w(SX1302_REG_RX_TOP_LORA_SERVICE_FSK_DETECT_MSP0_MSP_PNR, 60);
            lgw_reg_w(SX1302_REG_RX_TOP_LORA_SERVICE_FSK_DETECT_MSP1_MSP2_PNR, 60);
            lgw_reg_w(SX1302_REG_RX_TOP_LORA_SERVICE_FSK_DETECT_MSP2_MSP_PEAK_NB, 3);
            lgw_reg_w(SX1302_REG_RX_TOP_LORA_SERVICE_FSK_DETECT_MSP2_MSP2_PEAK_NB, 3);
            break;
        case 12:
            lgw_reg_w(SX1302_REG_RX_TOP_LORA_SERVICE_FSK_TXRX_CFG2_FINE_SYNCH_EN, 0);
            lgw_reg_w(SX1302_REG_RX_TOP_LORA_SERVICE_FSK_DETECT_ACC1_ACC_PNR, 60);
            lgw_reg_w(SX1302_REG_RX_TOP_LORA_SERVICE_FSK_DETECT_MSP0_MSP_PNR, 60);
            lgw_reg_w(SX1302_REG_RX_TOP_LORA_SERVICE_FSK_DETECT_MSP1_MSP2_PNR, 60);
            lgw_reg_w(SX1302_REG_RX_TOP_LORA_SERVICE_FSK_DETECT_MSP2_MSP_PEAK_NB, 3);
            lgw_reg_w(SX1302_REG_RX_TOP_LORA_SERVICE_FSK_DETECT_MSP2_MSP2_PEAK_NB, 3);
            break;
        default:
            printf("ERROR: Failed to configure LoRa service modem correlators\n");
            return LGW_REG_ERROR;
    }

    return LGW_REG_SUCCESS;
}

/* ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ */

int sx1302_lora_modem_configure() {
    lgw_reg_w(SX1302_REG_RX_TOP_DC_NOTCH_CFG1_ENABLE, 0x00);
    lgw_reg_w(SX1302_REG_RX_TOP_RX_DFE_AGC1_FREEZE_ON_SYNC, 0x00);
    lgw_reg_w(SX1302_REG_RX_TOP_RX_DFE_AGC1_FORCE_DEFAULT_FIR, 0x01);
    lgw_reg_w(SX1302_REG_RX_TOP_DAGC_CFG_GAIN_DROP_COMP, 0x01);
    lgw_reg_w(SX1302_REG_RX_TOP_DAGC_CFG_TARGET_LVL, 0x01);
    lgw_reg_w(SX1302_REG_RX_TOP_FINE_TIMING3_GAIN_I_EN, 0x00);

    lgw_reg_w(SX1302_REG_RX_TOP_MODEM_SYNC_DELTA_LSB_MODEM_SYNC_DELTA, 65);
    lgw_reg_w(SX1302_REG_RX_TOP_MODEM_SYNC_DELTA_MSB_MODEM_SYNC_DELTA, 0);

    /* Enable full modems */
    lgw_reg_w(SX1302_REG_OTP_MODEM_EN_0_MODEM_EN, 0xFF);

    /* Enable limited modems */
    lgw_reg_w(SX1302_REG_OTP_MODEM_EN_1_MODEM_EN, 0x0F);

    /* Configure PPM offset */
    lgw_reg_w(SX1302_REG_RX_TOP_MODEM_PPM_OFFSET1_PPM_OFFSET_SF5, 0x00);
    lgw_reg_w(SX1302_REG_RX_TOP_MODEM_PPM_OFFSET1_PPM_OFFSET_SF6, 0x00);
    lgw_reg_w(SX1302_REG_RX_TOP_MODEM_PPM_OFFSET1_PPM_OFFSET_SF7, 0x00);
    lgw_reg_w(SX1302_REG_RX_TOP_MODEM_PPM_OFFSET1_PPM_OFFSET_SF8, 0x00);
    lgw_reg_w(SX1302_REG_RX_TOP_MODEM_PPM_OFFSET2_PPM_OFFSET_SF9, 0x00);
    lgw_reg_w(SX1302_REG_RX_TOP_MODEM_PPM_OFFSET2_PPM_OFFSET_SF10, 0x00);
    lgw_reg_w(SX1302_REG_RX_TOP_MODEM_PPM_OFFSET2_PPM_OFFSET_SF11, 0x01);
    lgw_reg_w(SX1302_REG_RX_TOP_MODEM_PPM_OFFSET2_PPM_OFFSET_SF12, 0x01);

    /* Latch end-of-packet timestamp (sx1301 compatibility) */
    lgw_reg_w(SX1302_REG_RX_TOP_RX_BUFFER_LEGACY_TIMESTAMP, 0x01);

    return LGW_REG_SUCCESS;
}

/* ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ */

int sx1302_lora_service_modem_configure(uint8_t sf, uint8_t bw) {
    lgw_reg_w(SX1302_REG_RX_TOP_LORA_SERVICE_FSK_DC_NOTCH_CFG1_ENABLE, 0x00);
    lgw_reg_w(SX1302_REG_RX_TOP_LORA_SERVICE_FSK_RX_DFE_AGC1_FREEZE_ON_SYNC, 0x00);
    lgw_reg_w(SX1302_REG_RX_TOP_LORA_SERVICE_FSK_RX_DFE_AGC1_FORCE_DEFAULT_FIR, 0x01);
    lgw_reg_w(SX1302_REG_RX_TOP_LORA_SERVICE_FSK_DAGC_CFG_GAIN_DROP_COMP, 0x01);
    lgw_reg_w(SX1302_REG_RX_TOP_LORA_SERVICE_FSK_DAGC_CFG_TARGET_LVL, 0x01);
    lgw_reg_w(SX1302_REG_RX_TOP_LORA_SERVICE_FSK_FINE_TIMING3_GAIN_I_EN, 0x0);

    lgw_reg_w(SX1302_REG_RX_TOP_LORA_SERVICE_FSK_TXRX_CFG0_MODEM_SF, sf);
    lgw_reg_w(SX1302_REG_RX_TOP_LORA_SERVICE_FSK_TXRX_CFG0_MODEM_BW, bw);

    //SX1302_REG_RX_TOP_LORA_SERVICE_FSK_TXRX_CFG6_PREAMBLE_SYMB_NB
    //SX1302_REG_RX_TOP_LORA_SERVICE_FSK_TXRX_CFG7_PREAMBLE_SYMB_NB
    //SX1302_REG_RX_TOP_LORA_SERVICE_FSK_TXRX_CFG1_PPM_OFFSET
    //SX1302_REG_RX_TOP_LORA_SERVICE_FSK_TXRX_CFG1_CODING_RATE

    lgw_reg_w(SX1302_REG_RX_TOP_LORA_SERVICE_FSK_TXRX_CFG1_PPM_OFFSET, SET_PPM_ON(bw, sf));

    lgw_reg_w(SX1302_REG_RX_TOP_LORA_SERVICE_FSK_TXRX_CFG1_MODEM_EN, 1);
    lgw_reg_w(SX1302_REG_RX_TOP_LORA_SERVICE_FSK_TXRX_CFG2_CADRXTX, 1);

    lgw_reg_w(SX1302_REG_RX_TOP_LORA_SERVICE_FSK_TXRX_CFG2_MODEM_START, 1);

    return LGW_REG_SUCCESS;
}

/* ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ */

int sx1302_modem_enable() {
    lgw_reg_w(SX1302_REG_COMMON_GEN_CONCENTRATOR_MODEM_ENABLE, 0x01);
    lgw_reg_w(SX1302_REG_COMMON_GEN_MBWSSF_MODEM_ENABLE, 0x01);
    lgw_reg_w(SX1302_REG_COMMON_GEN_FSK_MODEM_ENABLE, 0x01);
    lgw_reg_w(SX1302_REG_COMMON_GEN_GLOBAL_EN, 0x01);

    return LGW_REG_SUCCESS;
}

/* ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ */

int sx1302_lora_syncword(bool public) {
    if (public) {
        /* multi-sf modems */
        lgw_reg_w(SX1302_REG_RX_TOP_FRAME_SYNCH0_PEAK1_POS, 6);
        lgw_reg_w(SX1302_REG_RX_TOP_FRAME_SYNCH1_PEAK2_POS, 8);

        /* LoRa service modem */
        lgw_reg_w(SX1302_REG_RX_TOP_LORA_SERVICE_FSK_FRAME_SYNCH0_PEAK1_POS, 6);
        lgw_reg_w(SX1302_REG_RX_TOP_LORA_SERVICE_FSK_FRAME_SYNCH1_PEAK2_POS, 8);
    } else {
        /* multi-sf modems */
        lgw_reg_w(SX1302_REG_RX_TOP_FRAME_SYNCH0_PEAK1_POS, 2);
        lgw_reg_w(SX1302_REG_RX_TOP_FRAME_SYNCH1_PEAK2_POS, 4);

        /* LoRa service modem */
        lgw_reg_w(SX1302_REG_RX_TOP_LORA_SERVICE_FSK_FRAME_SYNCH0_PEAK1_POS, 2);
        lgw_reg_w(SX1302_REG_RX_TOP_LORA_SERVICE_FSK_FRAME_SYNCH1_PEAK2_POS, 4);
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

/* ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ */

int sx1302_agc_status(uint8_t* status) {
    int32_t val;

#if 0
    if (lgw_reg_r(SX1302_REG_AGC_MCU_MCU_AGC_STATUS_MCU_AGC_STATUS, &val) != LGW_REG_SUCCESS) {
#else
    if (lgw_reg_r(SX1302_REG_AGC_MCU_MCU_MAIL_BOX_RD_DATA_BYTE3_MCU_MAIL_BOX_RD_DATA, &val) != LGW_REG_SUCCESS) {
#endif
        printf("ERROR: Failed to get AGC status\n");
        return LGW_HAL_ERROR;
    }

    *status = (uint8_t)val;

    return LGW_HAL_SUCCESS;
}

/* ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ */

int sx1302_agc_wait_status(uint8_t status) {
    uint8_t val;

    do {
        if (sx1302_agc_status(&val) != LGW_HAL_SUCCESS) {
            return LGW_HAL_ERROR;
        }
    } while (val != status);

    return LGW_HAL_SUCCESS;
}

/* ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ */

int sx1302_agc_mailbox_read(uint8_t mailbox, uint8_t* value) {
    uint16_t reg;
    int32_t val;

    /* Check parameters */
    if (mailbox > 3) {
        printf("ERROR: invalid AGC mailbox ID\n");
        return LGW_HAL_ERROR;
    }

    reg = SX1302_REG_AGC_MCU_MCU_MAIL_BOX_RD_DATA_BYTE0_MCU_MAIL_BOX_RD_DATA - mailbox;
    if (lgw_reg_r(reg, &val) != LGW_REG_SUCCESS) {
        printf("ERROR: failed to read AGC mailbox\n");
        return LGW_HAL_ERROR;
    }

    *value = (uint8_t)val;

    return LGW_HAL_SUCCESS;
}

/* ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ */

int sx1302_agc_mailbox_write(uint8_t mailbox, uint8_t value) {
    uint16_t reg;

    /* Check parameters */
    if (mailbox > 3) {
        printf("ERROR: invalid AGC mailbox ID\n");
        return LGW_HAL_ERROR;
    }

    reg = SX1302_REG_AGC_MCU_MCU_MAIL_BOX_WR_DATA_BYTE0_MCU_MAIL_BOX_WR_DATA - mailbox;
    if (lgw_reg_w(reg, (int32_t)value) != LGW_REG_SUCCESS) {
        printf("ERROR: failed to write AGC mailbox\n");
        return LGW_HAL_ERROR;
    }

    return LGW_HAL_SUCCESS;
}

/* ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ */

int sx1302_agc_start(uint8_t version, sx1302_radio_type_t radio_type, uint8_t ana_gain, uint8_t dec_gain, uint8_t fdd_mode) {
    uint8_t val;

    /* Wait for AGC fw to be started, and VERSION available in mailbox */
    sx1302_agc_wait_status(0x01); /* fw has started, VERSION is ready in mailbox */

    sx1302_agc_mailbox_read(0, &val);
    if (val != version) {
        printf("ERROR: wrong AGC fw version (%d)\n", val);
        return LGW_HAL_ERROR;
    }
    printf("AGC FW VERSION: %d\n", val);

    /* Configure Radio A gains */
    sx1302_agc_mailbox_write(0, ana_gain); /* 0:auto agc*/
    sx1302_agc_mailbox_write(1, dec_gain);
    if (radio_type == SX1302_RADIO_TYPE_SX125X) {
        sx1302_agc_mailbox_write(2, fdd_mode);
    }

    /* notify AGC that gains has been set to mailbox for Radio A */
    sx1302_agc_mailbox_write(3, AGC_RADIO_A_INIT_DONE);

    /* Wait for AGC to acknoledge it has received gain settings for Radio A */
    sx1302_agc_wait_status(0x02);

    /* Check ana_gain setting */
    sx1302_agc_mailbox_read(0, &val);
    if (val != ana_gain) {
        printf("ERROR: Analog gain of Radio A has not been set properly\n");
        return LGW_HAL_ERROR;
    }

    /* Check dec_gain setting */
    sx1302_agc_mailbox_read(1, &val);
    if (val != dec_gain) {
        printf("ERROR: Decimator gain of Radio A has not been set properly\n");
        return LGW_HAL_ERROR;
    }

    /* Check FDD mode setting */
    sx1302_agc_mailbox_read(2, &val);
    if (val != fdd_mode) {
        printf("ERROR: FDD mode of Radio A has not been set properly\n");
        return LGW_HAL_ERROR;
    }

    printf("AGC: Radio A config done\n");

    /* Configure Radio B gains */
    sx1302_agc_mailbox_write(0, ana_gain); /* 0:auto agc*/
    sx1302_agc_mailbox_write(1, dec_gain);
    if (radio_type == SX1302_RADIO_TYPE_SX125X) {
        sx1302_agc_mailbox_write(2, fdd_mode);
    }

    /* notify AGC that gains has been set to mailbox for Radio B */
    sx1302_agc_mailbox_write(3, AGC_RADIO_B_INIT_DONE);

    /* Wait for AGC to acknoledge it has received gain settings for Radio B */
    sx1302_agc_wait_status(0x00);

    /* Check ana_gain setting */
    sx1302_agc_mailbox_read(0, &val);
    if (val != ana_gain) {
        printf("ERROR: Analog gain of Radio B has not been set properly\n");
        return LGW_HAL_ERROR;
    }

    /* Check dec_gain setting */
    sx1302_agc_mailbox_read(1, &val);
    if (val != dec_gain) {
        printf("ERROR: Decimator gain of Radio B has not been set properly\n");
        return LGW_HAL_ERROR;
    }

    /* Check FDD mode setting */
    sx1302_agc_mailbox_read(2, &val);
    if (val != fdd_mode) {
        printf("ERROR: FDD mode of Radio B has not been set properly\n");
        return LGW_HAL_ERROR;
    }

    printf("AGC: Radio B config done\n");

    return LGW_HAL_SUCCESS;
}

/* ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ */

int sx1302_arb_status(uint8_t* status) {
    int32_t val;

#if 0
    if (lgw_reg_r(SX1302_REG_ARB_MCU_MCU_ARB_STATUS_MCU_ARB_STATUS, &val) != LGW_REG_SUCCESS) {
#else
    if (lgw_reg_r(SX1302_REG_ARB_MCU_ARB_DEBUG_STS_1_ARB_DEBUG_STS_1, &val) != LGW_REG_SUCCESS) {
#endif
        printf("ERROR: Failed to get AGC status\n");
        return LGW_HAL_ERROR;
    }

    *status = (uint8_t)val;

    return LGW_HAL_SUCCESS;
}

/* ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ */

int sx1302_arb_wait_status(uint8_t status) {
    uint8_t val;

    do {
        if (sx1302_arb_status(&val) != LGW_HAL_SUCCESS) {
            return LGW_HAL_ERROR;
        }
    } while (val != status);

    return LGW_HAL_SUCCESS;
}

/* ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ */

int sx1302_agc_debug_read(uint8_t reg_id, uint8_t* value) {
    uint16_t reg;
    int32_t val;

    /* Check parameters */
    if (reg_id > 15) {
        printf("ERROR: invalid ARB debug register ID\n");
        return LGW_HAL_ERROR;
    }

    reg = SX1302_REG_ARB_MCU_ARB_DEBUG_STS_0_ARB_DEBUG_STS_0 + reg_id;
    if (lgw_reg_r(reg, &val) != LGW_REG_SUCCESS) {
        printf("ERROR: failed to read ARB debug register\n");
        return LGW_HAL_ERROR;
    }

    *value = (uint8_t)val;

    return LGW_HAL_SUCCESS;
}

/* ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ */

int sx1302_agc_debug_write(uint8_t reg_id, uint8_t value) {
    uint16_t reg;

    /* Check parameters */
    if (reg_id > 3) {
        printf("ERROR: invalid ARB debug register ID\n");
        return LGW_HAL_ERROR;
    }

    reg = SX1302_REG_ARB_MCU_ARB_DEBUG_CFG_0_ARB_DEBUG_CFG_0 + reg_id;
    if (lgw_reg_w(reg, (int32_t)value) != LGW_REG_SUCCESS) {
        printf("ERROR: failed to write ARB debug register ID\n");
        return LGW_HAL_ERROR;
    }

    return LGW_HAL_SUCCESS;
}

/* ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ */

int sx1302_arb_start(uint8_t version) {
    uint8_t val;

    /* Wait for ARB fw to be started, and VERSION available in debug registers */
    sx1302_arb_wait_status(0x01);

    /* Get firmware VERSION */
    sx1302_agc_debug_read(0, &val);
    if (val != version) {
        printf("ERROR: wrong ARB fw version (%d)\n", val);
        return LGW_HAL_ERROR;
    }
    printf("ARB FW VERSION: %d\n", val);

    /* Notify ARB that it can resume */
    sx1302_agc_debug_write(1, 1);

    /* Wait for ARB to acknoledge */
    sx1302_arb_wait_status(0x00);

    return LGW_HAL_SUCCESS;
}

/* --- EOF ------------------------------------------------------------------ */
