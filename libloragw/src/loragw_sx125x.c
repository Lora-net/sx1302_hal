/*
 / _____)             _              | |
( (____  _____ ____ _| |_ _____  ____| |__
 \____ \| ___ |    (_   _) ___ |/ ___)  _ \
 _____) ) ____| | | || |_| ____( (___| | | |
(______/|_____)_|_|_| \__)_____)\____)_| |_|
  (C)2019 Semtech

Description:
    Functions used to handle LoRa concentrator SX1255/SX1257 radios.

License: Revised BSD License, see LICENSE.TXT file include in the project
*/

/* -------------------------------------------------------------------------- */
/* --- DEPENDANCIES --------------------------------------------------------- */

#include <stdint.h>     /* C99 types */
#include <stdbool.h>    /* bool type */
#include <stdio.h>      /* printf fprintf */

#include "sx125x_com.h"
#include "loragw_sx125x.h"
#include "loragw_com.h"
#include "loragw_aux.h"
#include "loragw_reg.h"
#include "loragw_hal.h"

/* -------------------------------------------------------------------------- */
/* --- PRIVATE MACROS ------------------------------------------------------- */

#define ARRAY_SIZE(a) (sizeof(a) / sizeof((a)[0]))
#if DEBUG_RAD == 1
    #define DEBUG_MSG(str)              fprintf(stdout, str)
    #define DEBUG_PRINTF(fmt, args...)  fprintf(stdout,"%s:%d: "fmt, __FUNCTION__, __LINE__, args)
    #define CHECK_NULL(a)               if(a==NULL){fprintf(stderr,"%s:%d: ERROR: NULL POINTER AS ARGUMENT\n", __FUNCTION__, __LINE__);return LGW_REG_ERROR;}
#else
    #define DEBUG_MSG(str)
    #define DEBUG_PRINTF(fmt, args...)
    #define CHECK_NULL(a)               if(a==NULL){return LGW_REG_ERROR;}
#endif

/* -------------------------------------------------------------------------- */
/* --- PRIVATE TYPES -------------------------------------------------------- */

/* -------------------------------------------------------------------------- */
/* --- PRIVATE CONSTANTS ---------------------------------------------------- */

#define PLL_LOCK_MAX_ATTEMPTS 5

static const struct radio_reg_s sx125x_regs[RADIO_TOTALREGS] = {
    {0,0,8}, /* MODE */
    {0,3,1}, /* MODE__PA_DRIVER_EN */
    {0,2,1}, /* MODE__TX_EN */
    {0,1,1}, /* MODE__RX_EN */
    {0,0,1}, /* MODE__STANDBY_EN */
    {1,0,8}, /* FRF_RX_MSB */
    {2,0,8}, /* FRF_RX_MID */
    {3,0,8}, /* FRF_RX_LSB */
    {4,0,8}, /* FRF_TX_MSB */
    {5,0,8}, /* FRF_TX_MID */
    {6,0,8}, /* FRF_TX_LSB */
    {7,0,8}, /* VERSION */
    {8,0,8}, /* TX_GAIN */
    {8,4,3}, /* TX_GAIN__DAC_GAIN */
    {8,0,4}, /* TX_GAIN__MIX_GAIN */
    {10,0,8}, /* TX_BW */
    {10,5,2}, /* TX_BW__PLL_BW */
    {10,0,5}, /* TX_BW__ANA_BW */
    {11,0,8}, /* TX_DAC_BW */
    {12,0,8}, /* RX_ANA_GAIN */
    {12,5,3}, /* RX_ANA_GAIN__LNA_GAIN */
    {12,1,4}, /* RX_ANA_GAIN__BB_GAIN */
    {12,0,1}, /* RX_ANA_GAIN__LNA_ZIN */
    {13,0,8}, /* RX_BW */
    {13,5,3}, /* RX_BW__ADC_BW */
    {13,2,3}, /* RX_BW__ADC_TRIM */
    {13,0,2}, /* RX_BW__BB_BW */
    {14,0,8}, /* RX_PLL_BW */
    {14,1,2}, /* RX_PLL_BW__PLL_BW */
    {14,0,1}, /* RX_PLL_BW__ADC_TEMP_EN */
    {15,0,8}, /* DIO_MAPPING */
    {15,6,2}, /* DIO_MAPPING__DIO_0_MAPPING */
    {15,4,2}, /* DIO_MAPPING__DIO_1_MAPPING */
    {15,2,2}, /* DIO_MAPPING__DIO_2_MAPPING */
    {15,0,2}, /* DIO_MAPPING__DIO_3_MAPPING */
    {16,0,8}, /* CLK_SELECT */
    {16,3,1}, /* CLK_SELECT__DIG_LOOPBACK_EN */
    {16,2,1}, /* CLK_SELECT__RF_LOOPBACK_EN */
    {16,1,1}, /* CLK_SELECT__CLK_OUT */
    {16,0,1}, /* CLK_SELECT__DAC_CLK_SELECT */
    {17,0,8}, /* MODE_STATUS */
    {17,2,1}, /* MODE_STATUS__LOW_BAT_EN */
    {17,1,1}, /* MODE_STATUS__RX_PLL_LOCKED */
    {17,0,1}, /* MODE_STATUS__TX_PLL_LOCKED */
    {26,0,8}, /* LOW_BAT_THRESH */
    {38,0,8}, /* SX1257_XOSC_TEST */
    {38,4,3}, /* SX1257_XOSC_TEST__DISABLE */
    {38,0,4}, /* SX1257_XOSC_TEST__GM_STARTUP */
    {40,0,8}, /* SX1255_XOSC_TEST */
    {40,4,3}, /* SX1255_XOSC_TEST__DISABLE */
    {40,0,4} /* SX1255_XOSC_TEST__GM_STARTUP */
};

/* -------------------------------------------------------------------------- */
/* --- PRIVATE VARIABLES ---------------------------------------------------- */

/* -------------------------------------------------------------------------- */
/* --- PRIVATE FUNCTIONS ---------------------------------------------------- */

/* -------------------------------------------------------------------------- */
/* --- PUBLIC FUNCTIONS DEFINITION ------------------------------------------ */

int sx125x_reg_w(radio_reg_t idx, uint8_t data, uint8_t rf_chain) {

    int com_stat;
    struct radio_reg_s reg;
    uint8_t mask;
    uint8_t r;
    uint8_t w;
    uint8_t val_check;

    /* checking input parameters */
    if (rf_chain >= LGW_RF_CHAIN_NB) {
        DEBUG_MSG("ERROR: INVALID RF_CHAIN\n");
        return LGW_REG_ERROR;
    }
    if (idx >= RADIO_TOTALREGS) {
        DEBUG_MSG("ERROR: REGISTER NUMBER OUT OF DEFINED RANGE\n");
        return LGW_REG_ERROR;
    }

    reg = sx125x_regs[idx];

    if ((reg.leng == 8) && (reg.offs == 0)){
        /* direct write */
        com_stat = sx125x_com_w(lgw_com_type(), lgw_com_target(), ((rf_chain == 0) ? LGW_SPI_MUX_TARGET_RADIOA : LGW_SPI_MUX_TARGET_RADIOB), reg.addr, data);
    } else {
        /* read-modify-write */
        com_stat = sx125x_com_r(lgw_com_type(), lgw_com_target(), ((rf_chain == 0) ? LGW_SPI_MUX_TARGET_RADIOA : LGW_SPI_MUX_TARGET_RADIOB), reg.addr, &r);
        mask = ((1 << reg.leng) - 1) << reg.offs;
        w = (r & ~mask) | ((data << reg.offs) & mask);
        com_stat |= sx125x_com_w(lgw_com_type(), lgw_com_target(), ((rf_chain == 0) ? LGW_SPI_MUX_TARGET_RADIOA : LGW_SPI_MUX_TARGET_RADIOB), reg.addr, w);
    }

    /* Check that we can read what we have written */
    sx125x_reg_r(idx, &val_check, rf_chain);
    if (val_check != data) {
        printf("ERROR: sx125x register %d write failed (w:%u r:%u)!!\n", idx, data, val_check);
        com_stat = LGW_COM_ERROR;
    }

    if (com_stat != LGW_COM_SUCCESS) {
        DEBUG_MSG("ERROR: COM ERROR DURING RADIO REGISTER WRITE\n");
        return LGW_REG_ERROR;
    } else {
        return LGW_REG_SUCCESS;
    }
}

/* ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ */

int sx125x_reg_r(radio_reg_t idx, uint8_t *data, uint8_t rf_chain) {

    int com_stat;
    struct radio_reg_s reg;
    uint8_t mask;
    uint8_t r;

    /* checking input parameters */
    if (rf_chain >= LGW_RF_CHAIN_NB) {
        DEBUG_MSG("ERROR: INVALID RF_CHAIN\n");
        return LGW_REG_ERROR;
    }
    if (idx >= RADIO_TOTALREGS) {
        DEBUG_MSG("ERROR: REGISTER NUMBER OUT OF DEFINED RANGE\n");
        return LGW_REG_ERROR;
    }

    reg = sx125x_regs[idx];

    com_stat = sx125x_com_r(lgw_com_type(), lgw_com_target(), ((rf_chain == 0) ? LGW_SPI_MUX_TARGET_RADIOA : LGW_SPI_MUX_TARGET_RADIOB), reg.addr, &r);
    mask = ((1 << reg.leng) - 1) << reg.offs;
    *data = (r & mask) >> reg.offs;

    if (com_stat != LGW_COM_SUCCESS) {
        DEBUG_MSG("ERROR: COM ERROR DURING RADIO REGISTER READ\n");
        return LGW_REG_ERROR;
    } else {
        return LGW_REG_SUCCESS;
    }
}

/* ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ */

int sx125x_setup(uint8_t rf_chain, uint8_t rf_clkout, bool rf_enable, uint8_t rf_radio_type, uint32_t freq_hz) {
    uint32_t part_int = 0;
    uint32_t part_frac = 0;
    int cpt_attempts = 0;
    uint8_t val;

    if (rf_chain >= LGW_RF_CHAIN_NB) {
        DEBUG_MSG("ERROR: INVALID RF_CHAIN\n");
        return -1;
    }

    /* Get version to identify SX1255/57 silicon revision */
    sx125x_reg_r(SX125x_REG_VERSION, &val, rf_chain);
    DEBUG_PRINTF("Note: SX125x #%d version register returned 0x%02x\n", rf_chain, val);

    /* General radio setup */
    if (rf_clkout == rf_chain) {
        sx125x_reg_w(SX125x_REG_CLK_SELECT, SX125x_TX_DAC_CLK_SEL + 2, rf_chain);
        DEBUG_PRINTF("Note: SX125x #%d clock output enabled\n", rf_chain);
    } else {
        sx125x_reg_w(SX125x_REG_CLK_SELECT, SX125x_TX_DAC_CLK_SEL, rf_chain);
        DEBUG_PRINTF("Note: SX125x #%d clock output disabled\n", rf_chain);
    }

    switch (rf_radio_type) {
        case LGW_RADIO_TYPE_SX1255:
            sx125x_reg_w(SX125x_REG_SX1255_XOSC_TEST__GM_STARTUP, SX125x_XOSC_GM_STARTUP, rf_chain);
            sx125x_reg_w(SX125x_REG_SX1255_XOSC_TEST__DISABLE, SX125x_XOSC_DISABLE, rf_chain);
            break;
        case LGW_RADIO_TYPE_SX1257:
            sx125x_reg_w(SX125x_REG_SX1257_XOSC_TEST__GM_STARTUP, SX125x_XOSC_GM_STARTUP, rf_chain);
            sx125x_reg_w(SX125x_REG_SX1257_XOSC_TEST__DISABLE, SX125x_XOSC_DISABLE, rf_chain);
            break;
        default:
            DEBUG_PRINTF("ERROR: UNEXPECTED VALUE %d FOR RADIO TYPE\n", rf_radio_type);
            break;
    }

    if (rf_enable == true) {
        /* Tx gain and trim */
        sx125x_reg_w(SX125x_REG_TX_GAIN__MIX_GAIN, SX125x_TX_MIX_GAIN, rf_chain);
        sx125x_reg_w(SX125x_REG_TX_GAIN__DAC_GAIN, SX125x_TX_DAC_GAIN, rf_chain);

        sx125x_reg_w(SX125x_REG_TX_BW__ANA_BW, SX125x_TX_ANA_BW, rf_chain);
        sx125x_reg_w(SX125x_REG_TX_BW__PLL_BW, SX125x_TX_PLL_BW, rf_chain);

        sx125x_reg_w(SX125x_REG_TX_DAC_BW, SX125x_TX_DAC_BW, rf_chain);

        /* Rx gain and trim */
        sx125x_reg_w(SX125x_REG_RX_ANA_GAIN__LNA_ZIN, SX125x_LNA_ZIN, rf_chain);
        sx125x_reg_w(SX125x_REG_RX_ANA_GAIN__BB_GAIN, SX125x_RX_BB_GAIN, rf_chain);
        sx125x_reg_w(SX125x_REG_RX_ANA_GAIN__LNA_GAIN, SX125x_RX_LNA_GAIN, rf_chain);

        sx125x_reg_w(SX125x_REG_RX_BW__BB_BW, SX125x_RX_BB_BW, rf_chain);
        sx125x_reg_w(SX125x_REG_RX_BW__ADC_TRIM, SX125x_RX_ADC_TRIM, rf_chain);
        sx125x_reg_w(SX125x_REG_RX_BW__ADC_BW, SX125x_RX_ADC_BW, rf_chain);

        sx125x_reg_w(SX125x_REG_RX_PLL_BW__ADC_TEMP_EN, SX125x_ADC_TEMP, rf_chain);
        sx125x_reg_w(SX125x_REG_RX_PLL_BW__PLL_BW, SX125x_RX_PLL_BW, rf_chain);

        /* set RX PLL frequency */
        switch (rf_radio_type) {
            case LGW_RADIO_TYPE_SX1255:
                part_int = freq_hz / (SX125x_32MHz_FRAC << 7); /* integer part, gives the MSB */
                part_frac = ((freq_hz % (SX125x_32MHz_FRAC << 7)) << 9) / SX125x_32MHz_FRAC; /* fractional part, gives middle part and LSB */
                break;
            case LGW_RADIO_TYPE_SX1257:
                part_int = freq_hz / (SX125x_32MHz_FRAC << 8); /* integer part, gives the MSB */
                part_frac = ((freq_hz % (SX125x_32MHz_FRAC << 8)) << 8) / SX125x_32MHz_FRAC; /* fractional part, gives middle part and LSB */
                break;
            default:
                DEBUG_PRINTF("ERROR: UNEXPECTED VALUE %d FOR RADIO TYPE\n", rf_radio_type);
                break;
        }

        sx125x_reg_w(SX125x_REG_FRF_RX_MSB, 0xFF & part_int, rf_chain);
        sx125x_reg_w(SX125x_REG_FRF_RX_MID, 0xFF & (part_frac >> 8), rf_chain);
        sx125x_reg_w(SX125x_REG_FRF_RX_LSB, 0xFF & part_frac, rf_chain);

        /* start and PLL lock */
        do {
            if (cpt_attempts >= PLL_LOCK_MAX_ATTEMPTS) {
                DEBUG_MSG("ERROR: FAIL TO LOCK PLL\n");
                return -1;
            }
            sx125x_reg_w(SX125x_REG_MODE, 1, rf_chain);
            sx125x_reg_w(SX125x_REG_MODE, 3, rf_chain);
            ++cpt_attempts;
            DEBUG_PRINTF("Note: SX125x #%d PLL start (attempt %d)\n", rf_chain, cpt_attempts);
            wait_ms(1);
            sx125x_reg_r(SX125x_REG_MODE_STATUS, &val, rf_chain);
        } while ((val & 0x02) == 0);
    } else {
        DEBUG_PRINTF("Note: SX125x #%d kept in standby mode\n", rf_chain);
    }

    return 0;
}

/* --- EOF ------------------------------------------------------------------ */
