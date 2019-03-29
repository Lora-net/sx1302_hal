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
#include <inttypes.h>
#include <time.h>
#include <assert.h>

#include <sys/ioctl.h>
#include <linux/spi/spidev.h>

#include "loragw_reg.h"
#include "loragw_aux.h"
#include "loragw_hal.h"
#include "loragw_sx1302.h"
#include "loragw_agc_params.h"

/* -------------------------------------------------------------------------- */
/* --- PRIVATE MACROS ------------------------------------------------------- */

#define ARRAY_SIZE(a) (sizeof(a) / sizeof((a)[0]))
#if DEBUG_REG == 1
    #define DEBUG_MSG(str)                fprintf(stderr, str)
    #define DEBUG_PRINTF(fmt, args...)    fprintf(stderr,"%s:%d: "fmt, __FUNCTION__, __LINE__, args)
    #define CHECK_NULL(a)                if(a==NULL){fprintf(stderr,"%s:%d: ERROR: NULL POINTER AS ARGUMENT\n", __FUNCTION__, __LINE__);return LGW_REG_ERROR;}
#else
    #define DEBUG_MSG(str)
    #define DEBUG_PRINTF(fmt, args...)
    #define CHECK_NULL(a)                if(a==NULL){return LGW_REG_ERROR;}
#endif

#define IF_HZ_TO_REG(f)     ((f << 5) / 15625)
#define SET_PPM_ON(bw,dr)   (((bw == BW_125KHZ) && ((dr == DR_LORA_SF11) || (dr == DR_LORA_SF12))) || ((bw == BW_250KHZ) && (dr == DR_LORA_SF12)))

/* -------------------------------------------------------------------------- */
/* --- PRIVATE CONSTANTS ---------------------------------------------------- */

#define AGC_RADIO_A_INIT_DONE   0x80
#define AGC_RADIO_B_INIT_DONE   0x20

#define MCU_AGC                 0x01
#define MCU_ARB                 0x02

#define AGC_MEM_ADDR            0x0000
#define ARB_MEM_ADDR            0x2000

#define MCU_FW_SIZE             8192 /* size of the firmware IN BYTES (= twice the number of 14b words) */

/* -------------------------------------------------------------------------- */
/* --- INTERNAL SHARED VARIABLES -------------------------------------------- */

/* -------------------------------------------------------------------------- */
/* --- PRIVATE FUNCTIONS DEFINITION ----------------------------------------- */

extern int32_t lgw_sf_getval(int x);
extern int32_t lgw_bw_getval(int x);

/* ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ */

int calculate_freq_to_time_drift(uint32_t freq_hz, uint8_t bw, uint16_t * mant, uint8_t * exp) {
    uint64_t mantissa_u64;
    uint8_t exponent = 0;
    int32_t bw_hz;

    /* check input variables */
    CHECK_NULL(mant);
    CHECK_NULL(exp);

    bw_hz = lgw_bw_getval(bw);
    if (bw_hz < 0) {
        printf("ERROR: Unsupported bandwidth for frequency to time drift calculation\n");
        return LGW_REG_ERROR;
    }

    mantissa_u64 = (uint64_t)bw_hz * (2 << (20-1)) / freq_hz;
    while (mantissa_u64 < 2048) {
        exponent += 1;
        mantissa_u64 <<= 1;
    }

    *mant = (uint16_t)mantissa_u64;
    *exp = exponent;

    return LGW_REG_SUCCESS;
}

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

int sx1302_radio_reset(uint8_t rf_chain, lgw_radio_type_t type) {
    uint16_t reg_radio_en;
    uint16_t reg_radio_rst;

    /* Check input parameters */
    if (rf_chain >= LGW_RF_CHAIN_NB)
    {
        DEBUG_MSG("ERROR: invalid RF chain\n");
        return LGW_REG_ERROR;
    }
    if ((type != LGW_RADIO_TYPE_SX1255) && (type != LGW_RADIO_TYPE_SX1257) && (type != LGW_RADIO_TYPE_SX1250)) {
        DEBUG_MSG("ERROR: invalid radio type\n");
        return LGW_REG_ERROR;
    }

    /* Switch to SPI clock before reseting the radio */
    lgw_reg_w(SX1302_REG_COMMON_CTRL0_CLK32_RIF_CTRL, 0x00);

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
        case LGW_RADIO_TYPE_SX1255:
        case LGW_RADIO_TYPE_SX1257:
            /* Do nothing */
            DEBUG_PRINTF("INFO: reset sx125x (RADIO_%s) done\n", REG_SELECT(rf_chain, "A", "B"));
            break;
        case LGW_RADIO_TYPE_SX1250:
            lgw_reg_w(reg_radio_rst, 0x01);
            wait_ms(10); /* wait for auto calibration to complete */
            DEBUG_PRINTF("INFO: reset sx1250 (RADIO_%s) done\n", REG_SELECT(rf_chain, "A", "B"));
            break;
        default:
            return LGW_REG_ERROR;
    }

    return LGW_REG_SUCCESS;
}

/* ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ */

int sx1302_radio_set_mode(uint8_t rf_chain, lgw_radio_type_t type) {
    uint16_t reg;

    /* Check input parameters */
    if (rf_chain >= LGW_RF_CHAIN_NB) {
        DEBUG_MSG("ERROR: invalid RF chain\n");
        return LGW_REG_ERROR;
    }
    if ((type != LGW_RADIO_TYPE_SX1255) && (type != LGW_RADIO_TYPE_SX1257) && (type != LGW_RADIO_TYPE_SX1250)) {
        DEBUG_MSG("ERROR: invalid radio type\n");
        return LGW_REG_ERROR;
    }

    /* Set the radio mode */
    reg = REG_SELECT(rf_chain,  SX1302_REG_COMMON_CTRL0_SX1261_MODE_RADIO_A,
                                SX1302_REG_COMMON_CTRL0_SX1261_MODE_RADIO_B);
    switch (type) {
        case LGW_RADIO_TYPE_SX1250:
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

int sx1302_radio_host_ctrl(bool host_ctrl) {
    lgw_reg_w(SX1302_REG_COMMON_CTRL0_HOST_RADIO_CTRL, (host_ctrl == false) ? 0x00 : 0x01);

    return LGW_REG_SUCCESS;
}

/* ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ */

int sx1302_radio_fe_configure() {
    lgw_reg_w(SX1302_REG_RADIO_FE_RSSI_BB_FILTER_ALPHA_RADIO_A_RSSI_BB_FILTER_ALPHA, 0x03);
    lgw_reg_w(SX1302_REG_RADIO_FE_RSSI_DEC_FILTER_ALPHA_RADIO_A_RSSI_DEC_FILTER_ALPHA, 0x07);
    lgw_reg_w(SX1302_REG_RADIO_FE_RSSI_BB_FILTER_ALPHA_RADIO_B_RSSI_BB_FILTER_ALPHA, 0x03);
    lgw_reg_w(SX1302_REG_RADIO_FE_RSSI_DEC_FILTER_ALPHA_RADIO_B_RSSI_DEC_FILTER_ALPHA, 0x07);

    lgw_reg_w(SX1302_REG_RADIO_FE_RSSI_DB_DEF_RADIO_A_RSSI_DB_DEFAULT_VALUE, 23);
    lgw_reg_w(SX1302_REG_RADIO_FE_RSSI_DEC_DEF_RADIO_A_RSSI_DEC_DEFAULT_VALUE, 66);
    lgw_reg_w(SX1302_REG_RADIO_FE_RSSI_DB_DEF_RADIO_B_RSSI_DB_DEFAULT_VALUE, 23);
    lgw_reg_w(SX1302_REG_RADIO_FE_RSSI_DEC_DEF_RADIO_B_RSSI_DEC_DEFAULT_VALUE, 66);

    lgw_reg_w(SX1302_REG_RADIO_FE_CTRL0_RADIO_A_DC_NOTCH_EN, 1);
    lgw_reg_w(SX1302_REG_RADIO_FE_CTRL0_RADIO_A_HOST_FILTER_GAIN, 0x0b);
    lgw_reg_w(SX1302_REG_RADIO_FE_CTRL0_RADIO_B_DC_NOTCH_EN, 1);
    lgw_reg_w(SX1302_REG_RADIO_FE_CTRL0_RADIO_B_HOST_FILTER_GAIN, 0x0b);

    return LGW_REG_SUCCESS;
}

/* ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ */

int sx1302_channelizer_configure(struct lgw_conf_rxif_s * if_cfg, bool fix_gain) {
    int32_t if_freq;
    uint8_t channels_mask = 0x00;
    int i;

    /* Check input parameters */
    if (if_cfg == NULL) {
        printf("ERROR: Failed to configure LoRa channelizer\n");
        return LGW_REG_ERROR;
    }

    /* Select which radio is connected to each multi-SF channel */
    for (i = 0; i < LGW_MULTI_NB; i++) {
        channels_mask |= (if_cfg[i].rf_chain << i);
    }
    printf("LoRa multi-SF radio select: 0x%02X\n", channels_mask);
    lgw_reg_w(SX1302_REG_RX_TOP_RADIO_SELECT_RADIO_SELECT, channels_mask);

    /* Select which radio is connected to the LoRa service channel */
    printf("LoRa service radio select: 0x%02X\n", if_cfg[8].rf_chain);
    lgw_reg_w(SX1302_REG_RX_TOP_LORA_SERVICE_FSK_LORA_SERVICE_RADIO_SEL_RADIO_SELECT, if_cfg[8].rf_chain);

    /* Select which radio is connected to the FSK channel */
    printf("FSK radio select %u\n", if_cfg[9].rf_chain);
    lgw_reg_w(SX1302_REG_RX_TOP_LORA_SERVICE_FSK_FSK_CFG_3_RADIO_SELECT, if_cfg[9].rf_chain);

    /* Configure multi-SF channels IF frequencies */
    if_freq = IF_HZ_TO_REG(if_cfg[0].freq_hz);
    lgw_reg_w(SX1302_REG_RX_TOP_FREQ_0_MSB_IF_FREQ_0, (if_freq >> 8) & 0x0000001F);
    lgw_reg_w(SX1302_REG_RX_TOP_FREQ_0_LSB_IF_FREQ_0, (if_freq >> 0) & 0x000000FF);

    if_freq = IF_HZ_TO_REG(if_cfg[1].freq_hz);
    lgw_reg_w(SX1302_REG_RX_TOP_FREQ_1_MSB_IF_FREQ_1, (if_freq >> 8) & 0x0000001F);
    lgw_reg_w(SX1302_REG_RX_TOP_FREQ_1_LSB_IF_FREQ_1, (if_freq >> 0) & 0x000000FF);

    if_freq = IF_HZ_TO_REG(if_cfg[2].freq_hz);
    lgw_reg_w(SX1302_REG_RX_TOP_FREQ_2_MSB_IF_FREQ_2, (if_freq >> 8) & 0x0000001F);
    lgw_reg_w(SX1302_REG_RX_TOP_FREQ_2_LSB_IF_FREQ_2, (if_freq >> 0) & 0x000000FF);

    if_freq = IF_HZ_TO_REG(if_cfg[3].freq_hz);
    lgw_reg_w(SX1302_REG_RX_TOP_FREQ_3_MSB_IF_FREQ_3, (if_freq >> 8) & 0x0000001F);
    lgw_reg_w(SX1302_REG_RX_TOP_FREQ_3_LSB_IF_FREQ_3, (if_freq >> 0) & 0x000000FF);

    if_freq = IF_HZ_TO_REG(if_cfg[4].freq_hz);
    lgw_reg_w(SX1302_REG_RX_TOP_FREQ_4_MSB_IF_FREQ_4, (if_freq >> 8) & 0x0000001F);
    lgw_reg_w(SX1302_REG_RX_TOP_FREQ_4_LSB_IF_FREQ_4, (if_freq >> 0) & 0x000000FF);

    if_freq = IF_HZ_TO_REG(if_cfg[5].freq_hz);
    lgw_reg_w(SX1302_REG_RX_TOP_FREQ_5_MSB_IF_FREQ_5, (if_freq >> 8) & 0x0000001F);
    lgw_reg_w(SX1302_REG_RX_TOP_FREQ_5_LSB_IF_FREQ_5, (if_freq >> 0) & 0x000000FF);

    if_freq = IF_HZ_TO_REG(if_cfg[6].freq_hz);
    lgw_reg_w(SX1302_REG_RX_TOP_FREQ_6_MSB_IF_FREQ_6, (if_freq >> 8) & 0x0000001F);
    lgw_reg_w(SX1302_REG_RX_TOP_FREQ_6_LSB_IF_FREQ_6, (if_freq >> 0) & 0x000000FF);

    if_freq = IF_HZ_TO_REG(if_cfg[7].freq_hz);
    lgw_reg_w(SX1302_REG_RX_TOP_FREQ_7_MSB_IF_FREQ_7, (if_freq >> 8) & 0x0000001F);
    lgw_reg_w(SX1302_REG_RX_TOP_FREQ_7_LSB_IF_FREQ_7, (if_freq >> 0) & 0x000000FF);

    /* Configure LoRa service channel IF frequency */
    if_freq = IF_HZ_TO_REG(if_cfg[8].freq_hz);
    lgw_reg_w(SX1302_REG_RX_TOP_LORA_SERVICE_FSK_LORA_SERVICE_FREQ_MSB_IF_FREQ_0, (if_freq >> 8) & 0x0000001F);
    lgw_reg_w(SX1302_REG_RX_TOP_LORA_SERVICE_FSK_LORA_SERVICE_FREQ_LSB_IF_FREQ_0, (if_freq >> 0) & 0x000000FF);

    /* Configure FSK channel IF frequency */
    if_freq = IF_HZ_TO_REG(if_cfg[9].freq_hz);
    lgw_reg_w(SX1302_REG_RX_TOP_LORA_SERVICE_FSK_FSK_FREQ_MSB_IF_FREQ_0, (if_freq >> 8) & 0x0000001F);
    lgw_reg_w(SX1302_REG_RX_TOP_LORA_SERVICE_FSK_FSK_FREQ_LSB_IF_FREQ_0, (if_freq >> 0) & 0x000000FF);

    /* Set the low pass filtering corner frequency for RSSI indicator */
    lgw_reg_w(SX1302_REG_RX_TOP_RSSI_CONTROL_RSSI_FILTER_ALPHA, 0x05);

    /* Set the channelizer RSSI reset value */
    lgw_reg_w(SX1302_REG_RX_TOP_RSSI_DEF_VALUE_CHAN_RSSI_DEF_VALUE, 85);

    /* Force channelizer in fix gain, or let it be controlled by AGC */
    if (fix_gain == true) {
        lgw_reg_w(SX1302_REG_RX_TOP_CHANN_DAGC_CFG5_CHAN_DAGC_MODE, 0x00);
        lgw_reg_w(SX1302_REG_RX_TOP_GAIN_CONTROL_CHAN_GAIN, 5);
    } else {
        /* Allow the AGC to control gains */
        lgw_reg_w(SX1302_REG_RX_TOP_CHANN_DAGC_CFG5_CHAN_DAGC_MODE, 0x01);
    }

    return LGW_REG_SUCCESS;
}

/* ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ */

int sx1302_fsk_configure(struct lgw_conf_rxif_s * cfg) {
    uint64_t fsk_sync_word_reg;
    uint32_t fsk_br_reg;

    printf("FSK: syncword:0x%" PRIx64 ", syncword_size:%u\n", cfg->sync_word, cfg->sync_word_size);

    lgw_reg_w(SX1302_REG_RX_TOP_LORA_SERVICE_FSK_FSK_CFG_1_PSIZE, cfg->sync_word_size - 1);
    fsk_sync_word_reg = cfg->sync_word << (8 * (8 - cfg->sync_word_size));
    lgw_reg_w(SX1302_REG_RX_TOP_LORA_SERVICE_FSK_FSK_REF_PATTERN_BYTE0_FSK_REF_PATTERN, (uint8_t)(fsk_sync_word_reg >> 0));
    lgw_reg_w(SX1302_REG_RX_TOP_LORA_SERVICE_FSK_FSK_REF_PATTERN_BYTE1_FSK_REF_PATTERN, (uint8_t)(fsk_sync_word_reg >> 8));
    lgw_reg_w(SX1302_REG_RX_TOP_LORA_SERVICE_FSK_FSK_REF_PATTERN_BYTE2_FSK_REF_PATTERN, (uint8_t)(fsk_sync_word_reg >> 16));
    lgw_reg_w(SX1302_REG_RX_TOP_LORA_SERVICE_FSK_FSK_REF_PATTERN_BYTE3_FSK_REF_PATTERN, (uint8_t)(fsk_sync_word_reg >> 24));
    lgw_reg_w(SX1302_REG_RX_TOP_LORA_SERVICE_FSK_FSK_REF_PATTERN_BYTE4_FSK_REF_PATTERN, (uint8_t)(fsk_sync_word_reg >> 32));
    lgw_reg_w(SX1302_REG_RX_TOP_LORA_SERVICE_FSK_FSK_REF_PATTERN_BYTE5_FSK_REF_PATTERN, (uint8_t)(fsk_sync_word_reg >> 40));
    lgw_reg_w(SX1302_REG_RX_TOP_LORA_SERVICE_FSK_FSK_REF_PATTERN_BYTE6_FSK_REF_PATTERN, (uint8_t)(fsk_sync_word_reg >> 48));
    lgw_reg_w(SX1302_REG_RX_TOP_LORA_SERVICE_FSK_FSK_REF_PATTERN_BYTE7_FSK_REF_PATTERN, (uint8_t)(fsk_sync_word_reg >> 56));

    fsk_br_reg = 32000000 / cfg->datarate;
    lgw_reg_w(SX1302_REG_RX_TOP_LORA_SERVICE_FSK_BIT_RATE_MSB_BIT_RATE, (uint8_t)(fsk_br_reg >> 8));
    lgw_reg_w(SX1302_REG_RX_TOP_LORA_SERVICE_FSK_BIT_RATE_LSB_BIT_RATE, (uint8_t)(fsk_br_reg >> 0));
    lgw_reg_w(SX1302_REG_RX_TOP_LORA_SERVICE_FSK_FSK_CFG_1_CH_BW_EXPO, 0x03); /* 125KHz */

    lgw_reg_w(SX1302_REG_RX_TOP_LORA_SERVICE_FSK_FSK_CFG_3_RX_INVERT, 0);
    lgw_reg_w(SX1302_REG_RX_TOP_LORA_SERVICE_FSK_FSK_CFG_3_MODEM_INVERT_IQ, 0);

    lgw_reg_w(SX1302_REG_RX_TOP_LORA_SERVICE_FSK_FSK_CFG_4_RSSI_LENGTH, 4);
    lgw_reg_w(SX1302_REG_RX_TOP_LORA_SERVICE_FSK_FSK_CFG_0_PKT_MODE, 1); /* variable length */
    lgw_reg_w(SX1302_REG_RX_TOP_LORA_SERVICE_FSK_FSK_CFG_0_CRC_EN, 1);
    lgw_reg_w(SX1302_REG_RX_TOP_LORA_SERVICE_FSK_FSK_CFG_0_DCFREE_ENC, 2);
    lgw_reg_w(SX1302_REG_RX_TOP_LORA_SERVICE_FSK_FSK_CFG_0_CRC_IBM, 0);
    lgw_reg_w(SX1302_REG_RX_TOP_LORA_SERVICE_FSK_FSK_CFG_4_ERROR_OSR_TOL, 10);
    lgw_reg_w(SX1302_REG_RX_TOP_LORA_SERVICE_FSK_FSK_PKT_LENGTH_PKT_LENGTH, 255);
    lgw_reg_w(SX1302_REG_RX_TOP_LORA_SERVICE_FSK_FSK_NODE_ADRS_NODE_ADRS, 0);
    lgw_reg_w(SX1302_REG_RX_TOP_LORA_SERVICE_FSK_FSK_BROADCAST_BROADCAST, 0);
    lgw_reg_w(SX1302_REG_RX_TOP_LORA_SERVICE_FSK_FSK_CFG_3_AUTO_AFC, 1); /* ?? */
    lgw_reg_w(SX1302_REG_RX_TOP_LORA_SERVICE_FSK_FSK_TIMEOUT_MSB_TIMEOUT, 0);
    lgw_reg_w(SX1302_REG_RX_TOP_LORA_SERVICE_FSK_FSK_TIMEOUT_LSB_TIMEOUT, 128);

    return LGW_REG_SUCCESS;
}

/* ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ */

int sx1302_lora_correlator_configure() {
    lgw_reg_w(SX1302_REG_RX_TOP_SF5_CFG6_MSP_CNT_MODE, 0);
    lgw_reg_w(SX1302_REG_RX_TOP_SF5_CFG2_ACC_PNR, 55);
    lgw_reg_w(SX1302_REG_RX_TOP_SF5_CFG4_MSP_PNR, 24);
    lgw_reg_w(SX1302_REG_RX_TOP_SF5_CFG5_MSP2_PNR, 48);
    lgw_reg_w(SX1302_REG_RX_TOP_SF5_CFG6_MSP_PEAK_NB, 7);
    lgw_reg_w(SX1302_REG_RX_TOP_SF5_CFG7_MSP2_PEAK_NB, 5);

    lgw_reg_w(SX1302_REG_RX_TOP_SF6_CFG6_MSP_CNT_MODE, 0);
    lgw_reg_w(SX1302_REG_RX_TOP_SF6_CFG2_ACC_PNR, 55);
    lgw_reg_w(SX1302_REG_RX_TOP_SF6_CFG4_MSP_PNR, 24);
    lgw_reg_w(SX1302_REG_RX_TOP_SF6_CFG5_MSP2_PNR, 48);
    lgw_reg_w(SX1302_REG_RX_TOP_SF6_CFG6_MSP_PEAK_NB, 7);
    lgw_reg_w(SX1302_REG_RX_TOP_SF6_CFG7_MSP2_PEAK_NB, 5);

    lgw_reg_w(SX1302_REG_RX_TOP_SF7_CFG6_MSP_CNT_MODE, 0);
    lgw_reg_w(SX1302_REG_RX_TOP_SF7_CFG2_ACC_PNR, 52);
    lgw_reg_w(SX1302_REG_RX_TOP_SF7_CFG4_MSP_PNR, 24);
    lgw_reg_w(SX1302_REG_RX_TOP_SF7_CFG5_MSP2_PNR, 48);
    lgw_reg_w(SX1302_REG_RX_TOP_SF7_CFG6_MSP_PEAK_NB, 7);
    lgw_reg_w(SX1302_REG_RX_TOP_SF7_CFG7_MSP2_PEAK_NB, 4);

    lgw_reg_w(SX1302_REG_RX_TOP_SF8_CFG6_MSP_CNT_MODE, 0);
    lgw_reg_w(SX1302_REG_RX_TOP_SF8_CFG2_ACC_PNR, 52);
    lgw_reg_w(SX1302_REG_RX_TOP_SF8_CFG4_MSP_PNR, 24);
    lgw_reg_w(SX1302_REG_RX_TOP_SF8_CFG5_MSP2_PNR, 48);
    lgw_reg_w(SX1302_REG_RX_TOP_SF8_CFG6_MSP_PEAK_NB, 7);
    lgw_reg_w(SX1302_REG_RX_TOP_SF8_CFG7_MSP2_PEAK_NB, 5);

    lgw_reg_w(SX1302_REG_RX_TOP_SF9_CFG6_MSP_CNT_MODE, 0);
    lgw_reg_w(SX1302_REG_RX_TOP_SF9_CFG2_ACC_PNR, 52);
    lgw_reg_w(SX1302_REG_RX_TOP_SF9_CFG4_MSP_PNR, 24);
    lgw_reg_w(SX1302_REG_RX_TOP_SF9_CFG5_MSP2_PNR, 48);
    lgw_reg_w(SX1302_REG_RX_TOP_SF9_CFG6_MSP_PEAK_NB, 7);
    lgw_reg_w(SX1302_REG_RX_TOP_SF9_CFG7_MSP2_PEAK_NB, 5);

    lgw_reg_w(SX1302_REG_RX_TOP_SF10_CFG6_MSP_CNT_MODE, 0);
    lgw_reg_w(SX1302_REG_RX_TOP_SF10_CFG2_ACC_PNR, 52);
    lgw_reg_w(SX1302_REG_RX_TOP_SF10_CFG4_MSP_PNR, 24);
    lgw_reg_w(SX1302_REG_RX_TOP_SF10_CFG5_MSP2_PNR, 48);
    lgw_reg_w(SX1302_REG_RX_TOP_SF10_CFG6_MSP_PEAK_NB, 7);
    lgw_reg_w(SX1302_REG_RX_TOP_SF10_CFG7_MSP2_PEAK_NB, 5);

    lgw_reg_w(SX1302_REG_RX_TOP_SF11_CFG6_MSP_CNT_MODE, 0);
    lgw_reg_w(SX1302_REG_RX_TOP_SF11_CFG2_ACC_PNR, 52);
    lgw_reg_w(SX1302_REG_RX_TOP_SF11_CFG4_MSP_PNR, 24);
    lgw_reg_w(SX1302_REG_RX_TOP_SF11_CFG5_MSP2_PNR, 48);
    lgw_reg_w(SX1302_REG_RX_TOP_SF11_CFG6_MSP_PEAK_NB, 7);
    lgw_reg_w(SX1302_REG_RX_TOP_SF11_CFG7_MSP2_PEAK_NB, 5);

    lgw_reg_w(SX1302_REG_RX_TOP_SF12_CFG6_MSP_CNT_MODE, 0);
    lgw_reg_w(SX1302_REG_RX_TOP_SF12_CFG2_ACC_PNR, 52);
    lgw_reg_w(SX1302_REG_RX_TOP_SF12_CFG4_MSP_PNR, 24);
    lgw_reg_w(SX1302_REG_RX_TOP_SF12_CFG5_MSP2_PNR, 48);
    lgw_reg_w(SX1302_REG_RX_TOP_SF12_CFG6_MSP_PEAK_NB, 7);
    lgw_reg_w(SX1302_REG_RX_TOP_SF12_CFG7_MSP2_PEAK_NB, 5);

    lgw_reg_w(SX1302_REG_RX_TOP_CORR_CLOCK_ENABLE_CLK_EN, 0xFF);

    lgw_reg_w(SX1302_REG_RX_TOP_CORRELATOR_ENABLE_ONLY_FIRST_DET_EDGE_ENABLE_ONLY_FIRST_DET_EDGE, 0xFF);
    lgw_reg_w(SX1302_REG_RX_TOP_CORRELATOR_ENABLE_ACC_CLEAR_ENABLE_CORR_ACC_CLEAR, 0xFF);
    lgw_reg_w(SX1302_REG_RX_TOP_CORRELATOR_SF_EN_CORR_SF_EN, 0xFF); /* 12 11 10 9 8 7 6 5 */
    lgw_reg_w(SX1302_REG_RX_TOP_CORRELATOR_EN_CORR_EN, 0xFF); /* 1 correlator per channel */

    /* For debug: get packets with sync_error and header_error in FIFO */
#if 0
    lgw_reg_w(SX1302_REG_RX_TOP_RX_BUFFER_STORE_SYNC_FAIL_META, 0x01);
    lgw_reg_w(SX1302_REG_RX_TOP_RX_BUFFER_STORE_HEADER_ERR_META, 0x01);
#endif

    return LGW_REG_SUCCESS;
}

/* ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ */

int sx1302_lora_service_correlator_configure(struct lgw_conf_rxif_s * cfg) {
    switch (cfg->datarate) {
        case 5:
            lgw_reg_w(SX1302_REG_RX_TOP_LORA_SERVICE_FSK_TXRX_CFG2_FINE_SYNCH_EN, 1);
            lgw_reg_w(SX1302_REG_RX_TOP_LORA_SERVICE_FSK_DETECT_MSP3_MSP_CNT_MODE, 0);
            lgw_reg_w(SX1302_REG_RX_TOP_LORA_SERVICE_FSK_DETECT_ACC1_ACC_PNR, 55);
            lgw_reg_w(SX1302_REG_RX_TOP_LORA_SERVICE_FSK_DETECT_MSP0_MSP_PNR, 24);
            lgw_reg_w(SX1302_REG_RX_TOP_LORA_SERVICE_FSK_DETECT_MSP1_MSP2_PNR, 48);
            lgw_reg_w(SX1302_REG_RX_TOP_LORA_SERVICE_FSK_DETECT_MSP2_MSP_PEAK_NB, 7);
            lgw_reg_w(SX1302_REG_RX_TOP_LORA_SERVICE_FSK_DETECT_MSP2_MSP2_PEAK_NB, 5);
            break;
        case 6:
            lgw_reg_w(SX1302_REG_RX_TOP_LORA_SERVICE_FSK_TXRX_CFG2_FINE_SYNCH_EN, 1);
            lgw_reg_w(SX1302_REG_RX_TOP_LORA_SERVICE_FSK_DETECT_MSP3_MSP_CNT_MODE, 0);
            lgw_reg_w(SX1302_REG_RX_TOP_LORA_SERVICE_FSK_DETECT_ACC1_ACC_PNR, 55);
            lgw_reg_w(SX1302_REG_RX_TOP_LORA_SERVICE_FSK_DETECT_MSP0_MSP_PNR, 24);
            lgw_reg_w(SX1302_REG_RX_TOP_LORA_SERVICE_FSK_DETECT_MSP1_MSP2_PNR, 48);
            lgw_reg_w(SX1302_REG_RX_TOP_LORA_SERVICE_FSK_DETECT_MSP2_MSP_PEAK_NB, 7);
            lgw_reg_w(SX1302_REG_RX_TOP_LORA_SERVICE_FSK_DETECT_MSP2_MSP2_PEAK_NB, 5);
            break;
        case 7:
            lgw_reg_w(SX1302_REG_RX_TOP_LORA_SERVICE_FSK_TXRX_CFG2_FINE_SYNCH_EN, 0);
            lgw_reg_w(SX1302_REG_RX_TOP_LORA_SERVICE_FSK_DETECT_MSP3_MSP_CNT_MODE, 0);
            lgw_reg_w(SX1302_REG_RX_TOP_LORA_SERVICE_FSK_DETECT_ACC1_ACC_PNR, 52);
            lgw_reg_w(SX1302_REG_RX_TOP_LORA_SERVICE_FSK_DETECT_MSP0_MSP_PNR, 24);
            lgw_reg_w(SX1302_REG_RX_TOP_LORA_SERVICE_FSK_DETECT_MSP1_MSP2_PNR, 48);
            lgw_reg_w(SX1302_REG_RX_TOP_LORA_SERVICE_FSK_DETECT_MSP2_MSP_PEAK_NB, 7);
            lgw_reg_w(SX1302_REG_RX_TOP_LORA_SERVICE_FSK_DETECT_MSP2_MSP2_PEAK_NB, 4);
            break;
        case 8:
            lgw_reg_w(SX1302_REG_RX_TOP_LORA_SERVICE_FSK_TXRX_CFG2_FINE_SYNCH_EN, 0);
            lgw_reg_w(SX1302_REG_RX_TOP_LORA_SERVICE_FSK_DETECT_MSP3_MSP_CNT_MODE, 0);
            lgw_reg_w(SX1302_REG_RX_TOP_LORA_SERVICE_FSK_DETECT_ACC1_ACC_PNR, 52);
            lgw_reg_w(SX1302_REG_RX_TOP_LORA_SERVICE_FSK_DETECT_MSP0_MSP_PNR, 24);
            lgw_reg_w(SX1302_REG_RX_TOP_LORA_SERVICE_FSK_DETECT_MSP1_MSP2_PNR, 48);
            lgw_reg_w(SX1302_REG_RX_TOP_LORA_SERVICE_FSK_DETECT_MSP2_MSP_PEAK_NB, 7);
            lgw_reg_w(SX1302_REG_RX_TOP_LORA_SERVICE_FSK_DETECT_MSP2_MSP2_PEAK_NB, 5);
            break;
        case 9:
            lgw_reg_w(SX1302_REG_RX_TOP_LORA_SERVICE_FSK_TXRX_CFG2_FINE_SYNCH_EN, 0);
            lgw_reg_w(SX1302_REG_RX_TOP_LORA_SERVICE_FSK_DETECT_MSP3_MSP_CNT_MODE, 0);
            lgw_reg_w(SX1302_REG_RX_TOP_LORA_SERVICE_FSK_DETECT_ACC1_ACC_PNR, 52);
            lgw_reg_w(SX1302_REG_RX_TOP_LORA_SERVICE_FSK_DETECT_MSP0_MSP_PNR, 24);
            lgw_reg_w(SX1302_REG_RX_TOP_LORA_SERVICE_FSK_DETECT_MSP1_MSP2_PNR, 48);
            lgw_reg_w(SX1302_REG_RX_TOP_LORA_SERVICE_FSK_DETECT_MSP2_MSP_PEAK_NB, 7);
            lgw_reg_w(SX1302_REG_RX_TOP_LORA_SERVICE_FSK_DETECT_MSP2_MSP2_PEAK_NB, 5);
            break;
        case 10:
            lgw_reg_w(SX1302_REG_RX_TOP_LORA_SERVICE_FSK_TXRX_CFG2_FINE_SYNCH_EN, 0);
            lgw_reg_w(SX1302_REG_RX_TOP_LORA_SERVICE_FSK_DETECT_MSP3_MSP_CNT_MODE, 0);
            lgw_reg_w(SX1302_REG_RX_TOP_LORA_SERVICE_FSK_DETECT_ACC1_ACC_PNR, 52);
            lgw_reg_w(SX1302_REG_RX_TOP_LORA_SERVICE_FSK_DETECT_MSP0_MSP_PNR, 24);
            lgw_reg_w(SX1302_REG_RX_TOP_LORA_SERVICE_FSK_DETECT_MSP1_MSP2_PNR, 48);
            lgw_reg_w(SX1302_REG_RX_TOP_LORA_SERVICE_FSK_DETECT_MSP2_MSP_PEAK_NB, 7);
            lgw_reg_w(SX1302_REG_RX_TOP_LORA_SERVICE_FSK_DETECT_MSP2_MSP2_PEAK_NB, 5);
            break;
        case 11:
            lgw_reg_w(SX1302_REG_RX_TOP_LORA_SERVICE_FSK_TXRX_CFG2_FINE_SYNCH_EN, 0);
            lgw_reg_w(SX1302_REG_RX_TOP_LORA_SERVICE_FSK_DETECT_MSP3_MSP_CNT_MODE, 0);
            lgw_reg_w(SX1302_REG_RX_TOP_LORA_SERVICE_FSK_DETECT_ACC1_ACC_PNR, 52);
            lgw_reg_w(SX1302_REG_RX_TOP_LORA_SERVICE_FSK_DETECT_MSP0_MSP_PNR, 24);
            lgw_reg_w(SX1302_REG_RX_TOP_LORA_SERVICE_FSK_DETECT_MSP1_MSP2_PNR, 48);
            lgw_reg_w(SX1302_REG_RX_TOP_LORA_SERVICE_FSK_DETECT_MSP2_MSP_PEAK_NB, 7);
            lgw_reg_w(SX1302_REG_RX_TOP_LORA_SERVICE_FSK_DETECT_MSP2_MSP2_PEAK_NB, 5);
            break;
        case 12:
            lgw_reg_w(SX1302_REG_RX_TOP_LORA_SERVICE_FSK_TXRX_CFG2_FINE_SYNCH_EN, 0);
            lgw_reg_w(SX1302_REG_RX_TOP_LORA_SERVICE_FSK_DETECT_MSP3_MSP_CNT_MODE, 0);
            lgw_reg_w(SX1302_REG_RX_TOP_LORA_SERVICE_FSK_DETECT_ACC1_ACC_PNR, 52);
            lgw_reg_w(SX1302_REG_RX_TOP_LORA_SERVICE_FSK_DETECT_MSP0_MSP_PNR, 24);
            lgw_reg_w(SX1302_REG_RX_TOP_LORA_SERVICE_FSK_DETECT_MSP1_MSP2_PNR, 48);
            lgw_reg_w(SX1302_REG_RX_TOP_LORA_SERVICE_FSK_DETECT_MSP2_MSP_PEAK_NB, 7);
            lgw_reg_w(SX1302_REG_RX_TOP_LORA_SERVICE_FSK_DETECT_MSP2_MSP2_PEAK_NB, 5);
            break;
        default:
            printf("ERROR: Failed to configure LoRa service modem correlators\n");
            return LGW_REG_ERROR;
    }

    return LGW_REG_SUCCESS;
}

/* ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ */

int sx1302_lora_modem_configure(uint32_t radio_freq_hz) {
    uint16_t mantissa = 0;
    uint8_t exponent = 0;

    /* TODO: test if channel is enabled */

    lgw_reg_w(SX1302_REG_RX_TOP_DC_NOTCH_CFG1_ENABLE, 0x00);
    lgw_reg_w(SX1302_REG_RX_TOP_RX_DFE_AGC1_FORCE_DEFAULT_FIR, 0x01);
    lgw_reg_w(SX1302_REG_RX_TOP_DAGC_CFG_GAIN_DROP_COMP, 0x01);
    lgw_reg_w(SX1302_REG_RX_TOP_DAGC_CFG_TARGET_LVL, 0x01);

    /* Enable full modems */
    printf("Configuring 8 full-SF modems\n");
    lgw_reg_w(SX1302_REG_OTP_MODEM_EN_0_MODEM_EN, 0xFF);

    /* Enable limited modems */
#if FPGA_BOARD_16_CH
    printf("Configuring 8 limited-SF modems\n");
    lgw_reg_w(SX1302_REG_OTP_MODEM_EN_1_MODEM_EN, 0xFF);
#else
    printf("Configuring 4 limited-SF modems\n");
    lgw_reg_w(SX1302_REG_OTP_MODEM_EN_1_MODEM_EN, 0x0F);
#endif

    /* Configure Channel sync offset */
    lgw_reg_w(SX1302_REG_ARB_MCU_CHANNEL_SYNC_OFFSET_01_CHANNEL_0_OFFSET, 5);
    lgw_reg_w(SX1302_REG_ARB_MCU_CHANNEL_SYNC_OFFSET_01_CHANNEL_1_OFFSET, 7);
    lgw_reg_w(SX1302_REG_ARB_MCU_CHANNEL_SYNC_OFFSET_23_CHANNEL_2_OFFSET, 9);
    lgw_reg_w(SX1302_REG_ARB_MCU_CHANNEL_SYNC_OFFSET_23_CHANNEL_3_OFFSET, 11);
    lgw_reg_w(SX1302_REG_ARB_MCU_CHANNEL_SYNC_OFFSET_45_CHANNEL_4_OFFSET, 5);
    lgw_reg_w(SX1302_REG_ARB_MCU_CHANNEL_SYNC_OFFSET_45_CHANNEL_5_OFFSET, 7);
    lgw_reg_w(SX1302_REG_ARB_MCU_CHANNEL_SYNC_OFFSET_67_CHANNEL_6_OFFSET, 9);
    lgw_reg_w(SX1302_REG_ARB_MCU_CHANNEL_SYNC_OFFSET_67_CHANNEL_7_OFFSET, 11);

    /* Configure PPM offset */
    lgw_reg_w(SX1302_REG_RX_TOP_MODEM_PPM_OFFSET1_PPM_OFFSET_SF5, 0x00);
    lgw_reg_w(SX1302_REG_RX_TOP_MODEM_PPM_OFFSET1_PPM_OFFSET_SF6, 0x00);
    lgw_reg_w(SX1302_REG_RX_TOP_MODEM_PPM_OFFSET1_PPM_OFFSET_SF7, 0x00);
    lgw_reg_w(SX1302_REG_RX_TOP_MODEM_PPM_OFFSET1_PPM_OFFSET_SF8, 0x00);
    lgw_reg_w(SX1302_REG_RX_TOP_MODEM_PPM_OFFSET2_PPM_OFFSET_SF9, 0x00);
    lgw_reg_w(SX1302_REG_RX_TOP_MODEM_PPM_OFFSET2_PPM_OFFSET_SF10, 0x00);
    lgw_reg_w(SX1302_REG_RX_TOP_MODEM_PPM_OFFSET2_PPM_OFFSET_SF11, 0x01);
    lgw_reg_w(SX1302_REG_RX_TOP_MODEM_PPM_OFFSET2_PPM_OFFSET_SF12, 0x01);

    /* Improve SF11/SF12 performances */
    lgw_reg_w(SX1302_REG_RX_TOP_FINE_TIMING_A_5_GAIN_I_EN_SF11, 1);
    lgw_reg_w(SX1302_REG_RX_TOP_FINE_TIMING_A_5_GAIN_I_EN_SF12, 1);
    lgw_reg_w(SX1302_REG_RX_TOP_FINE_TIMING_B_5_GAIN_I_EN_SF11, 1);
    lgw_reg_w(SX1302_REG_RX_TOP_FINE_TIMING_B_5_GAIN_I_EN_SF12, 1);

    /* Freq2TimeDrift computation */
    if (calculate_freq_to_time_drift(radio_freq_hz, BW_125KHZ, &mantissa, &exponent) != 0) {
        printf("ERROR: failed to calculate frequency to time drift for LoRa modem\n");
        return LGW_REG_ERROR;
    }
    printf("Freq2TimeDrift: Mantissa = %u (0x%02X, 0x%02X), Exponent = %d (0x%02X)\n", mantissa, (mantissa >> 8) & 0x00FF, (mantissa) & 0x00FF, exponent, exponent);
    lgw_reg_w(SX1302_REG_RX_TOP_FREQ_TO_TIME0_FREQ_TO_TIME_DRIFT_MANT, (mantissa >> 8) & 0x00FF);
    lgw_reg_w(SX1302_REG_RX_TOP_FREQ_TO_TIME1_FREQ_TO_TIME_DRIFT_MANT, (mantissa) & 0x00FF);
    lgw_reg_w(SX1302_REG_RX_TOP_FREQ_TO_TIME2_FREQ_TO_TIME_DRIFT_EXP, exponent);

    /* Time drift compensation */
    lgw_reg_w(SX1302_REG_RX_TOP_FREQ_TO_TIME3_FREQ_TO_TIME_INVERT_TIME_SYMB, 1);


    return LGW_REG_SUCCESS;
}

/* ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ */

int sx1302_lora_service_modem_configure(struct lgw_conf_rxif_s * cfg, uint32_t radio_freq_hz) {
    uint16_t mantissa = 0;
    uint8_t exponent = 0;

    lgw_reg_w(SX1302_REG_RX_TOP_LORA_SERVICE_FSK_DC_NOTCH_CFG1_ENABLE, 0x00);
    lgw_reg_w(SX1302_REG_RX_TOP_LORA_SERVICE_FSK_RX_DFE_AGC1_FORCE_DEFAULT_FIR, 0x01);
    lgw_reg_w(SX1302_REG_RX_TOP_LORA_SERVICE_FSK_DAGC_CFG_GAIN_DROP_COMP, 0x01);
    lgw_reg_w(SX1302_REG_RX_TOP_LORA_SERVICE_FSK_DAGC_CFG_TARGET_LVL, 0x01);

    lgw_reg_w(SX1302_REG_RX_TOP_LORA_SERVICE_FSK_FINE_TIMING1_GAIN_P_AUTO, 0x01);
    lgw_reg_w(SX1302_REG_RX_TOP_LORA_SERVICE_FSK_FINE_TIMING2_GAIN_I_PAYLOAD, 0x01);

    switch (cfg->datarate) {
        case DR_LORA_SF5:
        case DR_LORA_SF6:
            lgw_reg_w(SX1302_REG_RX_TOP_LORA_SERVICE_FSK_FINE_TIMING1_GAIN_P_PREAMB, 0x04);
            lgw_reg_w(SX1302_REG_RX_TOP_LORA_SERVICE_FSK_FINE_TIMING2_GAIN_I_EN, 0x00);
            break;
        case DR_LORA_SF7:
        case DR_LORA_SF8:
        case DR_LORA_SF9:
        case DR_LORA_SF10:
            lgw_reg_w(SX1302_REG_RX_TOP_LORA_SERVICE_FSK_FINE_TIMING1_GAIN_P_PREAMB, 0x06);
            lgw_reg_w(SX1302_REG_RX_TOP_LORA_SERVICE_FSK_FINE_TIMING2_GAIN_I_EN, 0x00);
            break;
        case DR_LORA_SF11:
        case DR_LORA_SF12:
            lgw_reg_w(SX1302_REG_RX_TOP_LORA_SERVICE_FSK_FINE_TIMING1_GAIN_P_PREAMB, 0x07);
            switch (cfg->bandwidth) {
                case BW_125KHZ:
                    lgw_reg_w(SX1302_REG_RX_TOP_LORA_SERVICE_FSK_FINE_TIMING2_GAIN_I_EN, 0x01);
                    break;
                case BW_250KHZ:
                    lgw_reg_w(SX1302_REG_RX_TOP_LORA_SERVICE_FSK_FINE_TIMING2_GAIN_I_EN, 0x02);
                    break;
                case BW_500KHZ:
                    lgw_reg_w(SX1302_REG_RX_TOP_LORA_SERVICE_FSK_FINE_TIMING2_GAIN_I_EN, 0x03);
                    break;
                default:
                    printf("ERROR: unsupported bandwidth %u for LoRa Service modem\n", cfg->bandwidth);
                    break;
            }
            break;
        default:
            printf("ERROR: unsupported datarate %u for LoRa Service modem\n", cfg->datarate);
            break;
    }

    lgw_reg_w(SX1302_REG_RX_TOP_LORA_SERVICE_FSK_TXRX_CFG2_IMPLICIT_HEADER, (cfg->implicit_hdr == true) ? 1 : 0);
    lgw_reg_w(SX1302_REG_RX_TOP_LORA_SERVICE_FSK_TXRX_CFG2_CRC_EN, (cfg->implicit_crc_en == true) ? 1 : 0);
    lgw_reg_w(SX1302_REG_RX_TOP_LORA_SERVICE_FSK_TXRX_CFG1_CODING_RATE, cfg->implicit_coderate);
    lgw_reg_w(SX1302_REG_RX_TOP_LORA_SERVICE_FSK_TXRX_CFG3_PAYLOAD_LENGTH, cfg->implicit_payload_length);

    lgw_reg_w(SX1302_REG_RX_TOP_LORA_SERVICE_FSK_TXRX_CFG0_MODEM_SF, cfg->datarate);
    lgw_reg_w(SX1302_REG_RX_TOP_LORA_SERVICE_FSK_TXRX_CFG0_MODEM_BW, cfg->bandwidth);
    lgw_reg_w(SX1302_REG_RX_TOP_LORA_SERVICE_FSK_TXRX_CFG1_PPM_OFFSET, SET_PPM_ON(cfg->bandwidth, cfg->datarate));

    //SX1302_REG_RX_TOP_LORA_SERVICE_FSK_TXRX_CFG6_PREAMBLE_SYMB_NB
    //SX1302_REG_RX_TOP_LORA_SERVICE_FSK_TXRX_CFG7_PREAMBLE_SYMB_NB

    /* Freq2TimeDrift computation */
    if (calculate_freq_to_time_drift(radio_freq_hz, cfg->bandwidth, &mantissa, &exponent) != 0) {
        printf("ERROR: failed to calculate frequency to time drift for LoRa service modem\n");
        return LGW_REG_ERROR;
    }
    lgw_reg_w(SX1302_REG_RX_TOP_LORA_SERVICE_FSK_FREQ_TO_TIME0_FREQ_TO_TIME_DRIFT_MANT, (mantissa >> 8) & 0x00FF);
    lgw_reg_w(SX1302_REG_RX_TOP_LORA_SERVICE_FSK_FREQ_TO_TIME1_FREQ_TO_TIME_DRIFT_MANT, (mantissa) & 0x00FF);
    lgw_reg_w(SX1302_REG_RX_TOP_LORA_SERVICE_FSK_FREQ_TO_TIME2_FREQ_TO_TIME_DRIFT_EXP, exponent);
    printf("Freq2TimeDrift: Mantissa = %d (0x%02X, 0x%02X), Exponent = %d (0x%02X)\n", mantissa, (mantissa >> 8) & 0x00FF, (mantissa) & 0x00FF, exponent, exponent);

    /* Time drift compensation */
    lgw_reg_w(SX1302_REG_RX_TOP_LORA_SERVICE_FSK_FREQ_TO_TIME3_FREQ_TO_TIME_INVERT_TIME_SYMB, 1);

    lgw_reg_w(SX1302_REG_RX_TOP_LORA_SERVICE_FSK_RX_DFE_AGC2_DAGC_IN_COMP, 1);

    lgw_reg_w(SX1302_REG_RX_TOP_LORA_SERVICE_FSK_TXRX_CFG1_MODEM_EN, 1);
    lgw_reg_w(SX1302_REG_RX_TOP_LORA_SERVICE_FSK_TXRX_CFG2_CADRXTX, 1);

    lgw_reg_w(SX1302_REG_RX_TOP_LORA_SERVICE_FSK_TXRX_CFG2_MODEM_START, 1);

    return LGW_REG_SUCCESS;
}

/* ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ */

int sx1302_modem_enable() {
    /* Enable LoRa multi-SF modems */
    lgw_reg_w(SX1302_REG_COMMON_GEN_CONCENTRATOR_MODEM_ENABLE, 0x01);

    /* Enable LoRa service modem */
    lgw_reg_w(SX1302_REG_COMMON_GEN_MBWSSF_MODEM_ENABLE, 0x01);

    /* Enable FSK modem */
    lgw_reg_w(SX1302_REG_COMMON_GEN_FSK_MODEM_ENABLE, 0x01);

    /* Enable RX */
    lgw_reg_w(SX1302_REG_COMMON_GEN_GLOBAL_EN, 0x01);

    return LGW_REG_SUCCESS;
}

/* ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ */

int sx1302_lora_syncword(bool public, uint8_t lora_service_sf) {
    /* Multi-SF modem configuration */
    printf("INFO: configuring LoRa (Multi-SF) SF5->SF6 with syncword PRIVATE (0x12)\n");
    lgw_reg_w(SX1302_REG_RX_TOP_FRAME_SYNCH0_SF5_PEAK1_POS_SF5, 2);
    lgw_reg_w(SX1302_REG_RX_TOP_FRAME_SYNCH1_SF5_PEAK2_POS_SF5, 4);
    lgw_reg_w(SX1302_REG_RX_TOP_FRAME_SYNCH0_SF6_PEAK1_POS_SF6, 2);
    lgw_reg_w(SX1302_REG_RX_TOP_FRAME_SYNCH1_SF6_PEAK2_POS_SF6, 4);
    if (public == true) {
        printf("INFO: configuring LoRa (Multi-SF) SF7->SF12 with syncword PUBLIC (0x34)\n");
        lgw_reg_w(SX1302_REG_RX_TOP_FRAME_SYNCH0_SF7TO12_PEAK1_POS_SF7TO12, 6);
        lgw_reg_w(SX1302_REG_RX_TOP_FRAME_SYNCH1_SF7TO12_PEAK2_POS_SF7TO12, 8);
    } else {
        printf("INFO: configuring LoRa (Multi-SF) SF7->SF12 with syncword PRIVATE (0x12)\n");
        lgw_reg_w(SX1302_REG_RX_TOP_FRAME_SYNCH0_SF7TO12_PEAK1_POS_SF7TO12, 2);
        lgw_reg_w(SX1302_REG_RX_TOP_FRAME_SYNCH1_SF7TO12_PEAK2_POS_SF7TO12, 4);
    }

    /* LoRa Service modem configuration */
    if ((public == false) || (lora_service_sf == DR_LORA_SF5) || (lora_service_sf == DR_LORA_SF6)) {
        printf("INFO: configuring LoRa (Service) SF%u with syncword PRIVATE (0x12)\n", lora_service_sf);
        lgw_reg_w(SX1302_REG_RX_TOP_LORA_SERVICE_FSK_FRAME_SYNCH0_PEAK1_POS, 2);
        lgw_reg_w(SX1302_REG_RX_TOP_LORA_SERVICE_FSK_FRAME_SYNCH1_PEAK2_POS, 4);
    } else {
        printf("INFO: configuring LoRa (Service) SF%u with syncword PUBLIC (0x34)\n", lora_service_sf);
        lgw_reg_w(SX1302_REG_RX_TOP_LORA_SERVICE_FSK_FRAME_SYNCH0_PEAK1_POS, 6);
        lgw_reg_w(SX1302_REG_RX_TOP_LORA_SERVICE_FSK_FRAME_SYNCH1_PEAK2_POS, 8);
    }

    return LGW_REG_SUCCESS;
}

/* ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ */

int sx1302_timestamp_mode(struct lgw_conf_timestamp_s * conf) {
    if (conf->enable_precision_ts == false) {
        printf("INFO: using legacy timestamp\n");
        /* Latch end-of-packet timestamp (sx1301 compatibility) */
        lgw_reg_w(SX1302_REG_RX_TOP_RX_BUFFER_LEGACY_TIMESTAMP, 0x01);
    } else {
        printf("INFO: using precision timestamp (max_ts_metrics:%u nb_symbols:%u)\n", conf->max_ts_metrics, conf->nb_symbols);
        /* Latch end-of-preamble timestamp */
        lgw_reg_w(SX1302_REG_RX_TOP_RX_BUFFER_LEGACY_TIMESTAMP, 0x00);
        lgw_reg_w(SX1302_REG_RX_TOP_RX_BUFFER_TIMESTAMP_CFG_MAX_TS_METRICS, conf->max_ts_metrics);

        /* LoRa multi-SF modems */
        lgw_reg_w(SX1302_REG_RX_TOP_TIMESTAMP_ENABLE, 0x01);
        lgw_reg_w(SX1302_REG_RX_TOP_TIMESTAMP_NB_SYMB, conf->nb_symbols);

        /* LoRa service modem */
        lgw_reg_w(SX1302_REG_RX_TOP_LORA_SERVICE_FSK_TIMESTAMP_ENABLE, 0x01);
        lgw_reg_w(SX1302_REG_RX_TOP_LORA_SERVICE_FSK_TIMESTAMP_NB_SYMB, conf->nb_symbols);
    }

    return LGW_REG_SUCCESS;
}

/* ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ */

/* memory variables */
static uint32_t counter_us_raw_27bits_inst_prev = 0;
static uint32_t counter_us_raw_27bits_pps_prev = 0;
static uint8_t  counter_us_raw_27bits_inst_wrap = 0;
static uint8_t  counter_us_raw_27bits_pps_wrap = 0;

/* ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ */

int sx1302_timestamp_expand(bool pps, uint32_t * cnt_us) {
    if (pps == true) {
        *cnt_us = (counter_us_raw_27bits_pps_wrap << 27) | *cnt_us;
    } else {
        *cnt_us = (counter_us_raw_27bits_inst_wrap << 27) | *cnt_us;
    }
    return LGW_REG_SUCCESS;
}

/* ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ */

int sx1302_timestamp_counter(bool pps, uint32_t * cnt_us) {
    int x;
    uint8_t buff[4];
    uint32_t counter_us_32bits;
    uint32_t counter_us_raw_27bits_now;
    uint32_t counter_us_raw_27bits_prev;
    uint8_t  counter_us_raw_27bits_wrap;

    /* Get the 32MHz timestamp counter - 4 bytes */
    /* step of 31.25 ns */
    x = lgw_reg_rb((pps == true) ? SX1302_REG_TIMESTAMP_TIMESTAMP_PPS_MSB2_TIMESTAMP_PPS : SX1302_REG_TIMESTAMP_TIMESTAMP_MSB2_TIMESTAMP, &buff[0], 4);
    if (x != LGW_REG_SUCCESS) {
        printf("ERROR: Failed to get timestamp counter value\n");
        *cnt_us = 0;
        return LGW_HAL_ERROR;
    }

    counter_us_raw_27bits_now  = (uint32_t)((buff[0] << 24) & 0xFF000000);
    counter_us_raw_27bits_now |= (uint32_t)((buff[1] << 16) & 0x00FF0000);
    counter_us_raw_27bits_now |= (uint32_t)((buff[2] << 8)  & 0x0000FF00);
    counter_us_raw_27bits_now |= (uint32_t)((buff[3] << 0)  & 0x000000FF);
    counter_us_raw_27bits_now /= 32; /* scale to 1MHz */

    /* Get the previous value of the counter we want to get */
    counter_us_raw_27bits_prev = ((pps == true) ? counter_us_raw_27bits_pps_prev : counter_us_raw_27bits_inst_prev);

    /* Get the current wrap status */
    counter_us_raw_27bits_wrap = ((pps == true) ? counter_us_raw_27bits_pps_wrap : counter_us_raw_27bits_inst_wrap);

    /* Check if counter has wrapped */
    if (counter_us_raw_27bits_now < counter_us_raw_27bits_prev) {
        counter_us_raw_27bits_wrap += 1;
        counter_us_raw_27bits_wrap = counter_us_raw_27bits_wrap % 32;
    }

    /* Store counter value and wrap status for next time */
    if (pps == true) {
        counter_us_raw_27bits_pps_prev = counter_us_raw_27bits_now;
        counter_us_raw_27bits_pps_wrap = counter_us_raw_27bits_wrap;
    } else {
        counter_us_raw_27bits_inst_prev = counter_us_raw_27bits_now;
        counter_us_raw_27bits_inst_wrap = counter_us_raw_27bits_wrap;
    }

    /* Convert 27-bits counter to 32-bits counter */
    counter_us_32bits = (counter_us_raw_27bits_wrap << 27) | counter_us_raw_27bits_now;

    //printf("%u,%u,%u\n", counter_us_raw_27bits_now, counter_us_32bits, counter_us_raw_27bits_wrap);

    *cnt_us = counter_us_32bits;

    return LGW_HAL_SUCCESS;
}

/* ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ */

int sx1302_gps_enable(bool enable) {
    if (enable == true) {
        lgw_reg_w(SX1302_REG_TIMESTAMP_GPS_CTRL_GPS_EN, 1);
        lgw_reg_w(SX1302_REG_TIMESTAMP_GPS_CTRL_GPS_POL, 1); /* invert polarity for PPS */
    } else {
        lgw_reg_w(SX1302_REG_TIMESTAMP_GPS_CTRL_GPS_EN, 0);
    }

    return LGW_HAL_SUCCESS;
}

/* ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ */

int sx1302_agc_load_firmware(const uint8_t *firmware) {
    int32_t val;
    uint8_t fw_check[MCU_FW_SIZE];
    int32_t gpio_sel = MCU_AGC;

    /* Configure GPIO to let AGC MCU access board LEDs */
    lgw_reg_w(SX1302_REG_GPIO_GPIO_SEL_0_SELECTION, gpio_sel);
    lgw_reg_w(SX1302_REG_GPIO_GPIO_SEL_1_SELECTION, gpio_sel);
    lgw_reg_w(SX1302_REG_GPIO_GPIO_SEL_2_SELECTION, gpio_sel);
    lgw_reg_w(SX1302_REG_GPIO_GPIO_SEL_3_SELECTION, gpio_sel);
    lgw_reg_w(SX1302_REG_GPIO_GPIO_SEL_4_SELECTION, gpio_sel);
    lgw_reg_w(SX1302_REG_GPIO_GPIO_SEL_5_SELECTION, gpio_sel);
    lgw_reg_w(SX1302_REG_GPIO_GPIO_SEL_6_SELECTION, gpio_sel);
    lgw_reg_w(SX1302_REG_GPIO_GPIO_SEL_7_SELECTION, gpio_sel);
    lgw_reg_w(SX1302_REG_GPIO_GPIO_DIR_L_DIRECTION, 0xFF); /* GPIO output direction */

    /* Take control over AGC MCU */
    lgw_reg_w(SX1302_REG_AGC_MCU_CTRL_MCU_CLEAR, 0x01);
    lgw_reg_w(SX1302_REG_AGC_MCU_CTRL_HOST_PROG, 0x01);
    lgw_reg_w(SX1302_REG_COMMON_PAGE_PAGE, 0x00);

    /* Write AGC fw in AGC MEM */
    lgw_mem_wb(AGC_MEM_ADDR, firmware, MCU_FW_SIZE);

    /* Read back and check */
    lgw_mem_rb(AGC_MEM_ADDR, fw_check, MCU_FW_SIZE, false);
    if (memcmp(firmware, fw_check, sizeof fw_check) != 0) {
        printf ("ERROR: Failed to load fw\n");
        return -1;
    }

#if BYPASS_FW_INIT
    printf("Disable AGC init protocol\n");
    sx1302_agc_mailbox_write(2, 0xF7);  /* To be done before fw starts */
#endif

    /* Release control over AGC MCU */
    lgw_reg_w(SX1302_REG_AGC_MCU_CTRL_HOST_PROG, 0x00);
    lgw_reg_w(SX1302_REG_AGC_MCU_CTRL_MCU_CLEAR, 0x00);

    lgw_reg_r(SX1302_REG_AGC_MCU_CTRL_PARITY_ERROR, &val);
    printf("AGC fw loaded (parity error:0x%02X)\n", val);

    printf("Waiting for AGC fw to start...\n");

    return LGW_HAL_SUCCESS;
}

/* ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ */

int sx1302_agc_status(uint8_t* status) {
    int32_t val;

    if (lgw_reg_r(SX1302_REG_AGC_MCU_MCU_AGC_STATUS_MCU_AGC_STATUS, &val) != LGW_REG_SUCCESS) {
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

int sx1302_agc_start(uint8_t version, lgw_radio_type_t radio_type, uint8_t ana_gain, uint8_t dec_gain, uint8_t fdd_mode) {
    uint8_t val;
    struct agc_gain_params_s agc_params;

    /* Check parameters */
    if ((radio_type != LGW_RADIO_TYPE_SX1255) && (radio_type != LGW_RADIO_TYPE_SX1257) && (radio_type != LGW_RADIO_TYPE_SX1250)) {
        DEBUG_MSG("ERROR: invalid radio type\n");
        return LGW_REG_ERROR;
    }

    /* Wait for AGC fw to be started, and VERSION available in mailbox */
    sx1302_agc_wait_status(0x01); /* fw has started, VERSION is ready in mailbox */

    sx1302_agc_mailbox_read(0, &val);
    if (val != version) {
        printf("ERROR: wrong AGC fw version (%d)\n", val);
        return LGW_HAL_ERROR;
    }
    printf("AGC FW VERSION: %d\n", val);

#if BYPASS_FW_INIT
    printf("Bypass AGC init protocol\n");
    return 0;
#endif

    /* Configure Radio A gains */
    sx1302_agc_mailbox_write(0, ana_gain); /* 0:auto agc*/
    sx1302_agc_mailbox_write(1, dec_gain);
    if (radio_type != LGW_RADIO_TYPE_SX1250) {
        printf("AGC: setting fdd_mode to %u\n", fdd_mode);
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
    if (radio_type != LGW_RADIO_TYPE_SX1250) {
        sx1302_agc_mailbox_write(2, fdd_mode);
    }

    /* notify AGC that gains has been set to mailbox for Radio B */
    sx1302_agc_mailbox_write(3, AGC_RADIO_B_INIT_DONE);

    /* Wait for AGC to acknoledge it has received gain settings for Radio B */
    sx1302_agc_wait_status(0x03);

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

    /* Configure AGC gains */
    agc_params = (radio_type == LGW_RADIO_TYPE_SX1250) ? agc_params_sx1250 : agc_params_sx125x;

    /* Configure analog gain min/max */
    sx1302_agc_mailbox_write(0, agc_params.ana_min);
    sx1302_agc_mailbox_write(1, agc_params.ana_max);

    /* notify AGC that params have been set to mailbox */
    sx1302_agc_mailbox_write(3, 0x03);

    /* Wait for AGC to acknoledge it has received params */
    sx1302_agc_wait_status(0x04);

    /* Check params */
    sx1302_agc_mailbox_read(0, &val);
    if (val != agc_params.ana_min) {
        printf("ERROR: wrong ana_min (w:%u r:%u)\n", agc_params.ana_min, val);
        return LGW_HAL_ERROR;
    }
    sx1302_agc_mailbox_read(1, &val);
    if (val != agc_params.ana_max) {
        printf("ERROR: ana_max (w:%u r:%u)\n", agc_params.ana_max, val);
        return LGW_HAL_ERROR;
    }

    printf("AGC: config of analog gain min/max done\n");

    /* Configure analog thresholds */
    sx1302_agc_mailbox_write(0, agc_params.ana_thresh_l);
    sx1302_agc_mailbox_write(1, agc_params.ana_thresh_h);

    /* notify AGC that params have been set to mailbox */
    sx1302_agc_mailbox_write(3, 0x04);

    /* Wait for AGC to acknoledge it has received params */
    sx1302_agc_wait_status(0x05);

    /* Check params */
    sx1302_agc_mailbox_read(0, &val);
    if (val != agc_params.ana_thresh_l) {
        printf("ERROR: wrong ana_thresh_l (w:%u r:%u)\n", agc_params.ana_thresh_l, val);
        return LGW_HAL_ERROR;
    }
    sx1302_agc_mailbox_read(1, &val);
    if (val != agc_params.ana_thresh_h) {
        printf("ERROR: wrong ana_thresh_h (w:%u r:%u)\n", agc_params.ana_thresh_h, val);
        return LGW_HAL_ERROR;
    }

    printf("AGC: config of analog threshold done\n");

    /* Configure decimator attenuation min/max */
    sx1302_agc_mailbox_write(0, agc_params.dec_attn_min);
    sx1302_agc_mailbox_write(1, agc_params.dec_attn_max);

    /* notify AGC that params have been set to mailbox */
    sx1302_agc_mailbox_write(3, 0x05);

    /* Wait for AGC to acknoledge it has received params */
    sx1302_agc_wait_status(0x06);

    /* Check params */
    sx1302_agc_mailbox_read(0, &val);
    if (val != agc_params.dec_attn_min) {
        printf("ERROR: wrong dec_attn_min (w:%u r:%u)\n", agc_params.dec_attn_min, val);
        return LGW_HAL_ERROR;
    }
    sx1302_agc_mailbox_read(1, &val);
    if (val != agc_params.dec_attn_max) {
        printf("ERROR: wrong dec_attn_max (w:%u r:%u)\n", agc_params.dec_attn_max, val);
        return LGW_HAL_ERROR;
    }

    printf("AGC: config of decimator atten min/max done\n");

    /* Configure decimator attenuation thresholds */
    sx1302_agc_mailbox_write(0, agc_params.dec_thresh_l);
    sx1302_agc_mailbox_write(1, agc_params.dec_thresh_h1);
    sx1302_agc_mailbox_write(2, agc_params.dec_thresh_h2);

    /* notify AGC that params have been set to mailbox */
    sx1302_agc_mailbox_write(3, 0x06);

    /* Wait for AGC to acknoledge it has received params */
    sx1302_agc_wait_status(0x07);

        /* Check params */
    sx1302_agc_mailbox_read(0, &val);
    if (val != agc_params.dec_thresh_l) {
        printf("ERROR: wrong dec_thresh_l (w:%u r:%u)\n", agc_params.dec_thresh_l, val);
        return LGW_HAL_ERROR;
    }
    sx1302_agc_mailbox_read(1, &val);
    if (val != agc_params.dec_thresh_h1) {
        printf("ERROR: wrong dec_thresh_h1 (w:%u r:%u)\n", agc_params.dec_thresh_h1, val);
        return LGW_HAL_ERROR;
    }
    sx1302_agc_mailbox_read(2, &val);
    if (val != agc_params.dec_thresh_h2) {
        printf("ERROR: wrong dec_thresh_h2 (w:%u r:%u)\n", agc_params.dec_thresh_h2, val);
        return LGW_HAL_ERROR;
    }

    printf("AGC: config of decimator threshold done\n");

    /* Configure channel attenuation min/max */
    sx1302_agc_mailbox_write(0, agc_params.chan_attn_min);
    sx1302_agc_mailbox_write(1, agc_params.chan_attn_max);

    /* notify AGC that params have been set to mailbox */
    sx1302_agc_mailbox_write(3, 0x07);

    /* Wait for AGC to acknoledge it has received params */
    sx1302_agc_wait_status(0x08);

    /* Check params */
    sx1302_agc_mailbox_read(0, &val);
    if (val != agc_params.chan_attn_min) {
        printf("ERROR: wrong chan_attn_min (w:%u r:%u)\n", agc_params.chan_attn_min, val);
        return LGW_HAL_ERROR;
    }
    sx1302_agc_mailbox_read(1, &val);
    if (val != agc_params.chan_attn_max) {
        printf("ERROR: wrong chan_attn_max (w:%u r:%u)\n", agc_params.chan_attn_max, val);
        return LGW_HAL_ERROR;
    }

    printf("AGC: config of channel atten min/max done\n");

    /* Configure channel attenuation threshold */
    sx1302_agc_mailbox_write(0, agc_params.chan_thresh_l);
    sx1302_agc_mailbox_write(1, agc_params.chan_thresh_h);

    /* notify AGC that params have been set to mailbox */
    sx1302_agc_mailbox_write(3, 0x08);

    /* Wait for AGC to acknoledge it has received params */
    sx1302_agc_wait_status(0x09);

    /* Check params */
    sx1302_agc_mailbox_read(0, &val);
    if (val != agc_params.chan_thresh_l) {
        printf("ERROR: wrong chan_thresh_l (w:%u r:%u)\n", agc_params.chan_thresh_l, val);
        return LGW_HAL_ERROR;
    }
    sx1302_agc_mailbox_read(1, &val);
    if (val != agc_params.chan_thresh_h) {
        printf("ERROR: wrong chan_thresh_h (w:%u r:%u)\n", agc_params.chan_thresh_h, val);
        return LGW_HAL_ERROR;
    }

    printf("AGC: config of channel atten threshold done\n");

    if (radio_type == LGW_RADIO_TYPE_SX1250) {
        /* Configure sx1250 SetPAConfig */
        sx1302_agc_mailbox_write(0, agc_params.deviceSel);
        sx1302_agc_mailbox_write(1, agc_params.hpMax);
        sx1302_agc_mailbox_write(2, agc_params.paDutyCycle);

        /* notify AGC that params have been set to mailbox */
        sx1302_agc_mailbox_write(3, 0x09);

        /* Wait for AGC to acknoledge it has received params */
        sx1302_agc_wait_status(0x0A);

        /* Check params */
        sx1302_agc_mailbox_read(0, &val);
        if (val != agc_params.deviceSel) {
            printf("ERROR: wrong deviceSel (w:%u r:%u)\n", agc_params.deviceSel, val);
            return LGW_HAL_ERROR;
        }
        sx1302_agc_mailbox_read(1, &val);
        if (val != agc_params.hpMax) {
            printf("ERROR: wrong hpMax (w:%u r:%u)\n", agc_params.hpMax, val);
            return LGW_HAL_ERROR;
        }
        sx1302_agc_mailbox_read(2, &val);
        if (val != agc_params.paDutyCycle) {
            printf("ERROR: wrong paDutyCycle (w:%u r:%u)\n", agc_params.paDutyCycle, val);
            return LGW_HAL_ERROR;
        }

        printf("AGC: config of sx1250 PA optimal settings done\n");

        /* notify AGC that it can resume */
        sx1302_agc_mailbox_write(3, 0x0A);
    } else {
        /* notify AGC that it can resume */
        sx1302_agc_mailbox_write(3, 0x09);
    }

    printf("AGC: started\n");

    return LGW_HAL_SUCCESS;
}

/* ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ */

int sx1302_arb_load_firmware(const uint8_t *firmware) {
    uint8_t fw_check[MCU_FW_SIZE];
    int32_t gpio_sel = MCU_ARB;
    int32_t val;

    lgw_reg_w(SX1302_REG_GPIO_GPIO_SEL_0_SELECTION, gpio_sel);
    lgw_reg_w(SX1302_REG_GPIO_GPIO_SEL_1_SELECTION, gpio_sel);
    lgw_reg_w(SX1302_REG_GPIO_GPIO_SEL_2_SELECTION, gpio_sel);
    lgw_reg_w(SX1302_REG_GPIO_GPIO_SEL_3_SELECTION, gpio_sel);
    lgw_reg_w(SX1302_REG_GPIO_GPIO_SEL_4_SELECTION, gpio_sel);
    lgw_reg_w(SX1302_REG_GPIO_GPIO_SEL_5_SELECTION, gpio_sel);
    lgw_reg_w(SX1302_REG_GPIO_GPIO_SEL_6_SELECTION, gpio_sel);
    lgw_reg_w(SX1302_REG_GPIO_GPIO_SEL_7_SELECTION, gpio_sel);
    lgw_reg_w(SX1302_REG_GPIO_GPIO_DIR_L_DIRECTION, 0xFF); /* GPIO output direction */

    /* Take control over ARB MCU */
    lgw_reg_w(SX1302_REG_ARB_MCU_CTRL_MCU_CLEAR, 0x01);
    lgw_reg_w(SX1302_REG_ARB_MCU_CTRL_HOST_PROG, 0x01);
    lgw_reg_w(SX1302_REG_COMMON_PAGE_PAGE, 0x00);

    /* Write ARB fw in ARB MEM */
    lgw_mem_wb(ARB_MEM_ADDR, &firmware[0], MCU_FW_SIZE);

    /* Read back and check */
    lgw_mem_rb(ARB_MEM_ADDR, fw_check, MCU_FW_SIZE, false);
    if (memcmp(firmware, fw_check, sizeof fw_check) != 0) {
        printf ("ERROR: Failed to load fw\n");
        return -1;
    }

#if BYPASS_FW_INIT
    printf("Disable ARB init protocol\n");
    sx1302_arb_debug_write(2, 0xF7); /* To be done before fw starts */
#endif

    /* Release control over ARB MCU */
    lgw_reg_w(SX1302_REG_ARB_MCU_CTRL_HOST_PROG, 0x00);
    lgw_reg_w(SX1302_REG_ARB_MCU_CTRL_MCU_CLEAR, 0x00);

    lgw_reg_r(SX1302_REG_ARB_MCU_CTRL_PARITY_ERROR, &val);
    printf("ARB fw loaded (parity error:0x%02X)\n", val);

    printf("Waiting for ARB fw to start...\n");

    return 0;
}

/* ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ */

int sx1302_arb_status(uint8_t* status) {
    int32_t val;

    if (lgw_reg_r(SX1302_REG_ARB_MCU_MCU_ARB_STATUS_MCU_ARB_STATUS, &val) != LGW_REG_SUCCESS) {
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

int sx1302_arb_debug_read(uint8_t reg_id, uint8_t* value) {
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

int sx1302_arb_debug_write(uint8_t reg_id, uint8_t value) {
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

void sx1302_arb_set_debug_stats(bool enable, uint8_t sf) {
    if (enable == true) {
        printf("ARB: Debug stats enabled for SF%u\n", sf);
        lgw_reg_w(SX1302_REG_ARB_MCU_ARB_DEBUG_CFG_0_ARB_DEBUG_CFG_0, sf);
    } else {
        printf("ARB: Debug stats disabled\n");
    }
}

/* ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ */

uint8_t sx1302_arb_get_debug_stats_detect(uint8_t channel) {
    int32_t dbg_val;

    if (channel >= 8) {
        printf("ERROR: wrong configuration, channel num must be < 8");
        return 0;
    }
    lgw_reg_r(SX1302_REG_ARB_MCU_ARB_DEBUG_STS_0_ARB_DEBUG_STS_0 + channel, &dbg_val);

    return (uint8_t)dbg_val;
}

/* ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ */

uint8_t sx1302_arb_get_debug_stats_alloc(uint8_t channel) {
    int32_t dbg_val;

    if (channel >= 8) {
        printf("ERROR: wrong configuration, channel num must be < 8");
        return 0;
    }
    lgw_reg_r(SX1302_REG_ARB_MCU_ARB_DEBUG_STS_8_ARB_DEBUG_STS_8 + channel, &dbg_val);

    return (uint8_t)dbg_val;
}

/* ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ */

void sx1302_arb_print_debug_stats(bool full) {
    int i;
    uint8_t nb_detect;
    uint8_t nb_alloc;
    int nb_detect_total = 0;
    int nb_alloc_total = 0;

    /* Get number of detects for all channels */
    nb_detect_total = 0;
    for (i = 0; i < 8; i++) {
        nb_detect = sx1302_arb_get_debug_stats_detect(i);
        if (full == true) {
            printf("ARB: CH%d: nb detect %u\n", i, nb_detect);
        }
        nb_detect_total += nb_detect;
    }

    /* Get number of modem allocation for all channels */
    nb_alloc_total = 0;
    for (i = 0; i < 8; i++) {
        nb_alloc = sx1302_arb_get_debug_stats_alloc(i);
        if (full == true) {
            printf("ARB: CH%d: nb alloc %u\n", i, nb_alloc);
        }
        nb_alloc_total += nb_alloc;
    }

    printf("ARB: DEBUG STATS: nb detect:%d, nb_alloc:%d\n", nb_detect_total, nb_alloc_total);
}

/* ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ */

int sx1302_arb_start(uint8_t version) {
    uint8_t val;

    /* Wait for ARB fw to be started, and VERSION available in debug registers */
    sx1302_arb_wait_status(0x01);

    /* Get firmware VERSION */
    sx1302_arb_debug_read(0, &val);
    if (val != version) {
        printf("ERROR: wrong ARB fw version (%d)\n", val);
        return LGW_HAL_ERROR;
    }
    printf("ARB FW VERSION: %d\n", val);

#if BYPASS_FW_INIT
    printf("Bypass ARB init protocol\n");
    return 0;
#endif

    /* Enable/disable ARB detect/modem alloc stats for the specified SF */
    sx1302_arb_set_debug_stats(true, DR_LORA_SF5);

    /* 0:Disable 1:Enable double demod for different timing set (best_timestamp / best_demodulation) - Only available for SF9 -> SF12 */
    sx1302_arb_debug_write(3, 0);

    /* Set double detect packet filtering threshold [0..3] */
    sx1302_arb_debug_write(2, 3);

    /* Notify ARB that it can resume */
    sx1302_arb_debug_write(1, 1);

    /* Wait for ARB to acknoledge */
    sx1302_arb_wait_status(0x00);

    printf("ARB: started\n");

    return LGW_HAL_SUCCESS;
}

/* ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ */

void sx1302_rx_buffer_dump(FILE * file, uint16_t start_addr, uint16_t end_addr) {
    int i;
    uint8_t rx_buffer[4096];

    printf("Dumping %u bytes, from 0x%X to 0x%X\n", end_addr - start_addr + 1, start_addr, end_addr);

    memset(rx_buffer, 0, sizeof rx_buffer);

    lgw_reg_w(SX1302_REG_RX_TOP_RX_BUFFER_DIRECT_RAM_IF, 1);
    lgw_mem_rb(0x4000 + start_addr, rx_buffer, end_addr - start_addr + 1, false);
    lgw_reg_w(SX1302_REG_RX_TOP_RX_BUFFER_DIRECT_RAM_IF, 0);

    for (i = 0; i < (end_addr - start_addr + 1); i++) {
        if (file == NULL) {
            printf("%02X ", rx_buffer[i]);
        } else {
            fprintf(file, "%02X ", rx_buffer[i]);
        }
    }
    if (file == NULL) {
        printf("\n");
    } else {
        fprintf(file, "\n");
    }

    /* Switching to direct-access memory could lead to corruption, so to be done only for debugging */
    assert(0);
}

/* ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ */

uint16_t sx1302_rx_buffer_read_ptr_addr(void) {
    int32_t val;
    uint16_t addr;

    lgw_reg_r(SX1302_REG_RX_TOP_RX_BUFFER_LAST_ADDR_READ_MSB_LAST_ADDR_READ, &val); /* mandatory to read MSB first */
    addr  = (uint16_t)(val << 8);
    lgw_reg_r(SX1302_REG_RX_TOP_RX_BUFFER_LAST_ADDR_READ_LSB_LAST_ADDR_READ, &val);
    addr |= (uint16_t)val;

    return addr;
}

/* ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ */

uint16_t sx1302_rx_buffer_write_ptr_addr(void) {
    int32_t val;
    uint16_t addr;

    lgw_reg_r(SX1302_REG_RX_TOP_RX_BUFFER_LAST_ADDR_WRITE_MSB_LAST_ADDR_WRITE, &val);  /* mandatory to read MSB first */
    addr  = (uint16_t)(val << 8);
    lgw_reg_r(SX1302_REG_RX_TOP_RX_BUFFER_LAST_ADDR_WRITE_LSB_LAST_ADDR_WRITE, &val);
    addr |= (uint16_t)val;

    return addr;
}

/* ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ */

void lora_crc16(const char data, int *crc) {
    int next = 0;
    next  =  (((data>>0)&1) ^ ((*crc>>12)&1) ^ ((*crc>> 8)&1)                 )      ;
    next += ((((data>>1)&1) ^ ((*crc>>13)&1) ^ ((*crc>> 9)&1)                 )<<1 ) ;
    next += ((((data>>2)&1) ^ ((*crc>>14)&1) ^ ((*crc>>10)&1)                 )<<2 ) ;
    next += ((((data>>3)&1) ^ ((*crc>>15)&1) ^ ((*crc>>11)&1)                 )<<3 ) ;
    next += ((((data>>4)&1) ^ ((*crc>>12)&1)                                  )<<4 ) ;
    next += ((((data>>5)&1) ^ ((*crc>>13)&1) ^ ((*crc>>12)&1) ^ ((*crc>> 8)&1))<<5 ) ;
    next += ((((data>>6)&1) ^ ((*crc>>14)&1) ^ ((*crc>>13)&1) ^ ((*crc>> 9)&1))<<6 ) ;
    next += ((((data>>7)&1) ^ ((*crc>>15)&1) ^ ((*crc>>14)&1) ^ ((*crc>>10)&1))<<7 ) ;
    next += ((((*crc>>0)&1) ^ ((*crc>>15)&1) ^ ((*crc>>11)&1)                 )<<8 ) ;
    next += ((((*crc>>1)&1) ^ ((*crc>>12)&1)                                  )<<9 ) ;
    next += ((((*crc>>2)&1) ^ ((*crc>>13)&1)                                  )<<10) ;
    next += ((((*crc>>3)&1) ^ ((*crc>>14)&1)                                  )<<11) ;
    next += ((((*crc>>4)&1) ^ ((*crc>>15)&1) ^ ((*crc>>12)&1) ^ ((*crc>> 8)&1))<<12) ;
    next += ((((*crc>>5)&1) ^ ((*crc>>13)&1) ^ ((*crc>> 9)&1)                 )<<13) ;
    next += ((((*crc>>6)&1) ^ ((*crc>>14)&1) ^ ((*crc>>10)&1)                 )<<14) ;
    next += ((((*crc>>7)&1) ^ ((*crc>>15)&1) ^ ((*crc>>11)&1)                 )<<15) ;
    (*crc) = next;
}

/* ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ */

uint16_t sx1302_lora_payload_crc(const uint8_t * data, uint8_t size) {
    int i;
    int crc = 0;

    for (i = 0; i < size; i++) {
        lora_crc16(data[i], &crc);
    }

    //printf("CRC16: 0x%02X 0x%02X (%X)\n", (uint8_t)(crc >> 8), (uint8_t)crc, crc);
    return (uint16_t)crc;
}

/* --- EOF ------------------------------------------------------------------ */
