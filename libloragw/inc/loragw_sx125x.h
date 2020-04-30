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

#ifndef _LORAGW_SX125X_H
#define _LORAGW_SX125X_H

/* -------------------------------------------------------------------------- */
/* --- DEPENDANCIES --------------------------------------------------------- */

#include <stdint.h>     /* C99 types */
#include <stdbool.h>    /* bool type */

/* -------------------------------------------------------------------------- */
/* --- INTERNAL SHARED TYPES ------------------------------------------------ */

struct radio_reg_s
{
	uint8_t addr; /* base address of the register */
	uint8_t offs; /* position of the register LSB (between 0 to 7) */
	uint8_t leng; /* number of bits in the register */
};

/* -------------------------------------------------------------------------- */
/* --- PUBLIC MACROS -------------------------------------------------------- */

#define SX1257_FREQ_TO_REG(f)       (uint32_t)((uint64_t)f * (1 << 19) / 32000000U)
#define SX1255_FREQ_TO_REG(f)       (uint32_t)((uint64_t)f * (1 << 20) / 32000000U)

/* -------------------------------------------------------------------------- */
/* --- PUBLIC CONSTANTS ----------------------------------------------------- */

#define LGW_REG_SUCCESS 0
#define LGW_REG_ERROR -1

#define SX125x_32MHz_FRAC 15625 /* irreductible fraction for PLL register caculation */

#define SX125x_TX_DAC_CLK_SEL   0   /* 0:int, 1:ext */
#define SX125x_TX_DAC_GAIN      2   /* 3:0, 2:-3, 1:-6, 0:-9 dBFS (default 2) */
#define SX125x_TX_MIX_GAIN      14  /* -38 + 2*TxMixGain dB (default 14) */
#define SX125x_TX_PLL_BW        1   /* 0:75, 1:150, 2:225, 3:300 kHz (default 3) */
#define SX125x_TX_ANA_BW        0   /* 17.5 / 2*(41-TxAnaBw) MHz (default 0) */
#define SX125x_TX_DAC_BW        5   /* 24 + 8*TxDacBw Nb FIR taps (default 2) */
#define SX125x_RX_LNA_GAIN      1   /* 1 to 6, 1 highest gain */
#define SX125x_RX_BB_GAIN       15  /* 0 to 15 , 15 highest gain */
#define SX125x_LNA_ZIN          0   /* 0:50, 1:200 Ohms (default 1) */
#define SX125x_RX_ADC_BW        7   /* 0 to 7, 2:100<BW<200, 5:200<BW<400,7:400<BW kHz SSB (default 7) */
#define SX125x_RX_ADC_TRIM      6   /* 0 to 7, 6 for 32MHz ref, 5 for 36MHz ref */
#define SX125x_RX_BB_BW         0   /* 0:750, 1:500, 2:375; 3:250 kHz SSB (default 1, max 3) */
#define SX125x_RX_PLL_BW        0   /* 0:75, 1:150, 2:225, 3:300 kHz (default 3, max 3) */
#define SX125x_ADC_TEMP         0   /* ADC temperature measurement mode (default 0) */
#define SX125x_XOSC_GM_STARTUP  13  /* (default 13) */
#define SX125x_XOSC_DISABLE     2   /* Disable of Xtal Oscillator blocks bit0:regulator, bit1:core(gm), bit2:amplifier */

typedef enum {
    SX125x_REG_MODE = 0,
    SX125x_REG_MODE__PA_DRIVER_EN = 1,
    SX125x_REG_MODE__TX_EN = 2,
    SX125x_REG_MODE__RX_EN = 3,
    SX125x_REG_MODE__STANDBY_EN = 4,
    SX125x_REG_FRF_RX_MSB = 5,
    SX125x_REG_FRF_RX_MID = 6,
    SX125x_REG_FRF_RX_LSB = 7,
    SX125x_REG_FRF_TX_MSB = 8,
    SX125x_REG_FRF_TX_MID = 9,
    SX125x_REG_FRF_TX_LSB = 10,
    SX125x_REG_VERSION = 11,
    SX125x_REG_TX_GAIN = 12,
    SX125x_REG_TX_GAIN__DAC_GAIN = 13,
    SX125x_REG_TX_GAIN__MIX_GAIN = 14,
    SX125x_REG_TX_BW = 15,
    SX125x_REG_TX_BW__PLL_BW = 16,
    SX125x_REG_TX_BW__ANA_BW = 17,
    SX125x_REG_TX_DAC_BW = 18,
    SX125x_REG_RX_ANA_GAIN = 19,
    SX125x_REG_RX_ANA_GAIN__LNA_GAIN = 20,
    SX125x_REG_RX_ANA_GAIN__BB_GAIN = 21,
    SX125x_REG_RX_ANA_GAIN__LNA_ZIN = 22,
    SX125x_REG_RX_BW = 23,
    SX125x_REG_RX_BW__ADC_BW = 24,
    SX125x_REG_RX_BW__ADC_TRIM = 25,
    SX125x_REG_RX_BW__BB_BW = 26,
    SX125x_REG_RX_PLL_BW = 27,
    SX125x_REG_RX_PLL_BW__PLL_BW = 28,
    SX125x_REG_RX_PLL_BW__ADC_TEMP_EN = 29,
    SX125x_REG_DIO_MAPPING = 30,
    SX125x_REG_DIO_MAPPING__DIO_0_MAPPING = 31,
    SX125x_REG_DIO_MAPPING__DIO_1_MAPPING = 32,
    SX125x_REG_DIO_MAPPING__DIO_2_MAPPING = 33,
    SX125x_REG_DIO_MAPPING__DIO_3_MAPPING = 34,
    SX125x_REG_CLK_SELECT = 35,
    SX125x_REG_CLK_SELECT__DIG_LOOPBACK_EN = 36,
    SX125x_REG_CLK_SELECT__RF_LOOPBACK_EN = 37,
    SX125x_REG_CLK_SELECT__CLK_OUT = 38,
    SX125x_REG_CLK_SELECT__DAC_CLK_SELECT = 39,
    SX125x_REG_MODE_STATUS = 40,
    SX125x_REG_MODE_STATUS__LOW_BAT_EN = 41,
    SX125x_REG_MODE_STATUS__RX_PLL_LOCKED = 42,
    SX125x_REG_MODE_STATUS__TX_PLL_LOCKED = 43,
    SX125x_REG_LOW_BAT_THRESH = 44,
    SX125x_REG_SX1257_XOSC_TEST = 45,
    SX125x_REG_SX1257_XOSC_TEST__DISABLE = 46,
    SX125x_REG_SX1257_XOSC_TEST__GM_STARTUP = 47,
    SX125x_REG_SX1255_XOSC_TEST = 48,
    SX125x_REG_SX1255_XOSC_TEST__DISABLE = 49,
    SX125x_REG_SX1255_XOSC_TEST__GM_STARTUP = 50
}
radio_reg_t;

#define RADIO_TOTALREGS 51

/* -------------------------------------------------------------------------- */
/* --- PUBLIC CONSTANTS ----------------------------------------------------- */
/*

SX1257 frequency setting :
F_register(24bit) = F_rf (Hz) / F_step(Hz)
                  = F_rf (Hz) * 2^19 / F_xtal(Hz)
                  = F_rf (Hz) * 2^19 / 32e6
                  = F_rf (Hz) * 256/15625

SX1255 frequency setting :
F_register(24bit) = F_rf (Hz) / F_step(Hz)
                  = F_rf (Hz) * 2^20 / F_xtal(Hz)
                  = F_rf (Hz) * 2^20 / 32e6
                  = F_rf (Hz) * 512/15625
*/

/* -------------------------------------------------------------------------- */
/* --- PUBLIC FUNCTIONS PROTOTYPES ------------------------------------------ */

int sx125x_setup(uint8_t rf_chain, uint8_t rf_clkout, bool rf_enable, uint8_t rf_radio_type, uint32_t freq_hz);

int sx125x_reg_w(radio_reg_t idx, uint8_t data, uint8_t rf_chain);
int sx125x_reg_r(radio_reg_t idx, uint8_t *data, uint8_t rf_chain);

#endif
/* --- EOF ------------------------------------------------------------------ */
