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

#include <sys/ioctl.h>
#include <linux/spi/spidev.h>

#include "loragw_reg.h"
#include "loragw_aux.h"
#include "loragw_hal.h"
#include "loragw_sx1302.h"
#include "loragw_sx125x.h"
#include "loragw_cal.h"

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

/* -------------------------------------------------------------------------- */
/* --- PRIVATE CONSTANTS ---------------------------------------------------- */

#define CAL_TX_TONE_FREQ_HZ     250000
#define CAL_ITER                3 /* Number of calibration iterations */

/* -------------------------------------------------------------------------- */
/* --- INTERNAL SHARED VARIABLES -------------------------------------------- */

/* Record Rx IQ mismatch corrections from calibration */
static int8_t rf_rx_image_amp[LGW_RF_CHAIN_NB];
static int8_t rf_rx_image_phi[LGW_RF_CHAIN_NB];

/* -------------------------------------------------------------------------- */
/* --- PRIVATE FUNCTIONS DEFINITION ----------------------------------------- */

void cal_rx_result_init(struct lgw_sx125x_cal_rx_result_s *res_rx_min, struct lgw_sx125x_cal_rx_result_s *res_rx_max);
void cal_rx_result_sort(struct lgw_sx125x_cal_rx_result_s *res_rx, struct lgw_sx125x_cal_rx_result_s *res_rx_min, struct lgw_sx125x_cal_rx_result_s *res_rx_max);
bool cal_rx_result_assert(struct lgw_sx125x_cal_rx_result_s *res_rx_min, struct lgw_sx125x_cal_rx_result_s *res_rx_max);
int sx125x_cal_rx_image(uint8_t rf_chain, uint32_t freq_hz, bool use_loopback, uint8_t radio_type, struct lgw_sx125x_cal_rx_result_s * res);

void cal_tx_result_init(struct lgw_sx125x_cal_tx_result_s *res_tx_min, struct lgw_sx125x_cal_tx_result_s *res_tx_max);
void cal_tx_result_sort(struct lgw_sx125x_cal_tx_result_s *res_tx, struct lgw_sx125x_cal_tx_result_s *res_tx_min, struct lgw_sx125x_cal_tx_result_s *res_tx_max);
bool cal_tx_result_assert(struct lgw_sx125x_cal_tx_result_s *res_tx_min, struct lgw_sx125x_cal_tx_result_s *res_tx_max);
int sx125x_cal_tx_dc_offset(uint8_t rf_chain, uint32_t freq_hz, uint8_t dac_gain, uint8_t mix_gain, uint8_t radio_type, struct lgw_sx125x_cal_tx_result_s * res);

/* -------------------------------------------------------------------------- */
/* --- PUBLIC FUNCTIONS DEFINITION ------------------------------------------ */

int sx1302_cal_start(uint8_t version, bool * rf_enable, uint32_t * rf_rx_freq, enum lgw_radio_type_e * rf_radio_type, struct lgw_tx_gain_lut_s * txgain_lut, bool * rf_tx_enable) {
    int i, j, k;
    uint8_t val;
    bool cal_status = false;
    uint8_t x_max;
    int x_max_idx;
    uint8_t dac_gain[TX_GAIN_LUT_SIZE_MAX];
    uint8_t mix_gain[TX_GAIN_LUT_SIZE_MAX];
    int8_t offset_i[TX_GAIN_LUT_SIZE_MAX];
    int8_t offset_q[TX_GAIN_LUT_SIZE_MAX];
    uint8_t nb_gains;
    bool unique_gains;
    struct lgw_sx125x_cal_rx_result_s cal_rx[CAL_ITER], cal_rx_min, cal_rx_max;
    struct lgw_sx125x_cal_tx_result_s cal_tx[CAL_ITER], cal_tx_min, cal_tx_max;

    /* Wait for AGC fw to be started, and VERSION available in mailbox */
    sx1302_agc_wait_status(0x01); /* fw has started, VERSION is ready in mailbox */

    sx1302_agc_mailbox_read(0, &val);
    if (val != version) {
        printf("ERROR: wrong CAL fw version (%d)\n", val);
        return LGW_HAL_ERROR;
    }
    printf("CAL FW VERSION: %d\n", val);

    /* notify CAL that it can resume */
    sx1302_agc_mailbox_write(3, 0x01);

    /* Wait for ARB to acknoledge */
    sx1302_agc_wait_status(0x00);

    printf("CAL: started\n");

    /* Run Rx image calibration */
    for (i = 0; i < LGW_RF_CHAIN_NB; i++) { /* TODO: loop on rf_chain */
        if (rf_enable[i]) {
            /* Calibration using the other radio for Tx */
            if (rf_radio_type[0] == rf_radio_type[1]) {
                cal_rx_result_init(&cal_rx_min, &cal_rx_max);
                for (j = 0; j < CAL_ITER; j++) {
                    sx125x_cal_rx_image(i, rf_rx_freq[i], false, rf_radio_type[i], &cal_rx[j]);
                    cal_rx_result_sort(&cal_rx[j], &cal_rx_min, &cal_rx_max);
                }
                cal_status = cal_rx_result_assert(&cal_rx_min, &cal_rx_max);
            }

            /* If failed or different radios, run calibration using RF loopback (assuming that it is better than no calibration) */
            if ((cal_status == false) || (rf_radio_type[0] != rf_radio_type[1])) {
                cal_rx_result_init(&cal_rx_min, &cal_rx_max);
                for (j = 0; j < CAL_ITER; j++) {
                    sx125x_cal_rx_image(i, rf_rx_freq[i], true, rf_radio_type[i], &cal_rx[j]);
                    cal_rx_result_sort(&cal_rx[j], &cal_rx_min, &cal_rx_max);
                }
                cal_status = cal_rx_result_assert(&cal_rx_min, &cal_rx_max);
            }

            if (cal_status == false) {
                DEBUG_MSG("*********************************************\n");
                DEBUG_PRINTF("ERROR: Rx image calibration of radio %d failed\n",i);
                DEBUG_MSG("*********************************************\n");
#if 0
                return LGW_HAL_ERROR;
#endif
            }

            /* Use the results of the best iteration */
            x_max = 0;
            x_max_idx = 0;
            for (j=0; j<CAL_ITER; j++) {
                if (cal_rx[j].rej > x_max) {
                    x_max = cal_rx[j].rej;
                    x_max_idx = j;
                }
            }
            rf_rx_image_amp[i] = cal_rx[x_max_idx].amp;
            rf_rx_image_phi[i] = cal_rx[x_max_idx].phi;

            DEBUG_PRINTF("INFO: Rx image calibration of radio %d succeeded. Improved image rejection from %2d to %2d dB (Amp:%3d Phi:%3d)\n", i, cal_rx[x_max_idx].rej_init, cal_rx[x_max_idx].rej, cal_rx[x_max_idx].amp, cal_rx[x_max_idx].phi);
        } else {
            rf_rx_image_amp[i] = 0;
            rf_rx_image_phi[i] = 0;
        }
    }

    /* Apply calibrated IQ mismatch compensation */
    lgw_reg_w(SX1302_REG_RADIO_FE_IQ_COMP_AMP_COEFF_RADIO_A_AMP_COEFF, (int32_t)rf_rx_image_amp[0]);
    lgw_reg_w(SX1302_REG_RADIO_FE_IQ_COMP_PHI_COEFF_RADIO_A_PHI_COEFF, (int32_t)rf_rx_image_phi[0]);
    lgw_reg_w(SX1302_REG_RADIO_FE_IQ_COMP_AMP_COEFF_RADIO_B_AMP_COEFF, (int32_t)rf_rx_image_amp[1]);
    lgw_reg_w(SX1302_REG_RADIO_FE_IQ_COMP_PHI_COEFF_RADIO_B_PHI_COEFF, (int32_t)rf_rx_image_phi[1]);


    /* Get List of unique combinations of DAC and mixer gains */
    nb_gains = 0;
    for (i = 0; i < txgain_lut->size; i++) {
        unique_gains = true;
        for (j = 0; j < nb_gains; j++) {
            if ((txgain_lut->lut[i].dac_gain == dac_gain[j]) && (txgain_lut->lut[i].mix_gain == mix_gain[j])) {
                unique_gains = false;
            }
        }
        if (unique_gains) {
            dac_gain[nb_gains] = txgain_lut->lut[i].dac_gain;
            mix_gain[nb_gains] = txgain_lut->lut[i].mix_gain;
            nb_gains++;
        }
    }

    /* Run Tx image calibration */
    for (i = 0; i < LGW_RF_CHAIN_NB; i++) {
        if (rf_tx_enable[i]) {
            for (j = 0; j < nb_gains; j++) {
                cal_tx_result_init(&cal_tx_min, &cal_tx_max);
                for (k = 0; k < CAL_ITER; k++){
                    sx125x_cal_tx_dc_offset(i, rf_rx_freq[i], dac_gain[j], mix_gain[j], rf_radio_type[i], &cal_tx[k]);
                    cal_tx_result_sort(&cal_tx[k], &cal_tx_min, &cal_tx_max);
                }
                cal_status = cal_tx_result_assert(&cal_tx_min, &cal_tx_max);

                if (cal_status == false) {
                    DEBUG_MSG("*********************************************\n");
                    DEBUG_PRINTF("ERROR: Tx DC offset calibration of radio %d for DAC gain %d and mixer gain %2d failed\n", i, dac_gain[j], mix_gain[j]);
                    DEBUG_MSG("*********************************************\n");
                    return LGW_HAL_ERROR;
                }

                /* Use the results of the best iteration */
                x_max = 0;
                x_max_idx = 0;
                for (k = 0; k < CAL_ITER; k++) {
                    if (cal_tx[k].rej > x_max) {
                        x_max = cal_tx[k].rej;
                        x_max_idx = k;
                    }
                }
                offset_i[j] = cal_tx[x_max_idx].offset_i;
                offset_q[j] = cal_tx[x_max_idx].offset_q;

                DEBUG_PRINTF("INFO: Tx DC offset calibration of radio %d for DAC gain %d and mixer gain %2d succeeded. Improved DC rejection by %2d dB (I:%4d Q:%4d)\n", i, dac_gain[j], mix_gain[j], cal_tx[x_max_idx].rej, cal_tx[x_max_idx].offset_i, cal_tx[x_max_idx].offset_q);
            }
        }
    }

    /* Fill DC offsets in Tx LUT */
    for (i = 0; i < txgain_lut->size; i++) {
        for (j = 0; j < nb_gains; j++) {
            if ((txgain_lut->lut[i].dac_gain == dac_gain[j]) && (txgain_lut->lut[i].mix_gain == mix_gain[j])) {
                break;
            }
        }
        txgain_lut->lut[i].offset_i = offset_i[j];
        txgain_lut->lut[i].offset_q = offset_q[j];
    }

    return LGW_HAL_SUCCESS;
}

/* ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ */

int sx125x_cal_rx_image(uint8_t rf_chain, uint32_t freq_hz, bool use_loopback, uint8_t radio_type, struct lgw_sx125x_cal_rx_result_s * res) {
    uint8_t rx, tx;
    uint32_t rx_freq_hz, tx_freq_hz;
    uint32_t rx_freq_int, rx_freq_frac;
    uint32_t tx_freq_int, tx_freq_frac;
    uint8_t rx_pll_locked, tx_pll_locked;
    int32_t r;
    uint8_t val;

    DEBUG_MSG("\n");
    DEBUG_PRINTF("rf_chain:%u, freq_hz:%u, loopback:%d, radio_type:%d\n", rf_chain, freq_hz, use_loopback, radio_type);

    /* Indentify which radio is transmitting the test tone */
    rx = rf_chain;
    if (use_loopback == true) {
        tx = rf_chain;
    } else {
        tx = 1-rf_chain;
    }

    lgw_sx125x_reg_r(SX125x_REG_VERSION, &val, rx);
    printf("CAL: radio version 0x%02X\n", val);

    /* Set PLL frequencies */
    rx_freq_hz = freq_hz;
    tx_freq_hz = freq_hz + CAL_TX_TONE_FREQ_HZ;
    switch (radio_type) {
        case LGW_RADIO_TYPE_SX1255:
            rx_freq_int = rx_freq_hz / (SX125x_32MHz_FRAC << 7); /* integer part, gives the MSB */
            rx_freq_frac = ((rx_freq_hz % (SX125x_32MHz_FRAC << 7)) << 9) / SX125x_32MHz_FRAC; /* fractional part, gives middle part and LSB */
            tx_freq_int = tx_freq_hz / (SX125x_32MHz_FRAC << 7); /* integer part, gives the MSB */
            tx_freq_frac = ((tx_freq_hz % (SX125x_32MHz_FRAC << 7)) << 9) / SX125x_32MHz_FRAC; /* fractional part, gives middle part and LSB */
            break;
        case LGW_RADIO_TYPE_SX1257:
            rx_freq_int = rx_freq_hz / (SX125x_32MHz_FRAC << 8); /* integer part, gives the MSB */
            rx_freq_frac = ((rx_freq_hz % (SX125x_32MHz_FRAC << 8)) << 8) / SX125x_32MHz_FRAC; /* fractional part, gives middle part and LSB */
            tx_freq_int = tx_freq_hz / (SX125x_32MHz_FRAC << 8); /* integer part, gives the MSB */
            tx_freq_frac = ((tx_freq_hz % (SX125x_32MHz_FRAC << 8)) << 8) / SX125x_32MHz_FRAC; /* fractional part, gives middle part and LSB */
            break;
        default:
            DEBUG_PRINTF("ERROR: UNEXPECTED VALUE %d FOR RADIO TYPE\n", radio_type);
            return LGW_HAL_ERROR;
    }
    lgw_sx125x_reg_w(SX125x_REG_FRF_RX_MSB, 0xFF & rx_freq_int, rx);
    lgw_sx125x_reg_w(SX125x_REG_FRF_RX_MID, 0xFF & (rx_freq_frac >> 8), rx);
    lgw_sx125x_reg_w(SX125x_REG_FRF_RX_LSB, 0xFF & rx_freq_frac, rx);
    lgw_sx125x_reg_w(SX125x_REG_FRF_TX_MSB, 0xFF & tx_freq_int, tx);
    lgw_sx125x_reg_w(SX125x_REG_FRF_TX_MID, 0xFF & (tx_freq_frac >> 8), tx);
    lgw_sx125x_reg_w(SX125x_REG_FRF_TX_LSB, 0xFF & tx_freq_frac, tx);

#if 0
    printf("w: 0x%02x\n", 0xFF & rx_freq_int);
    printf("w: 0x%02x\n", 0xFF & (rx_freq_frac >> 8));
    printf("w: 0x%02x\n", 0xFF & rx_freq_frac);
    lgw_sx125x_reg_r(SX125x_REG_FRF_RX_MSB, &val, rx);
    printf("r: 0x%02x\n", val);
    lgw_sx125x_reg_r(SX125x_REG_FRF_RX_MID, &val, rx);
    printf("r: 0x%02x\n", val);
    lgw_sx125x_reg_r(SX125x_REG_FRF_RX_LSB, &val, rx);
    printf("r: 0x%02x\n", val);
    printf("--\n");
#endif

    /* Radio settings for calibration */
    //sx125x_reg_w(SX125x_RX_ANA_GAIN__LNA_ZIN, 1, rx); /* Default: 1 */
    //sx125x_reg_w(SX125x_RX_ANA_GAIN__BB_GAIN, 15, rx); /* Default: 15 */
    //sx125x_reg_w(SX125x_RX_ANA_GAIN__LNA_GAIN, 1, rx); /* Default: 1 */
    lgw_sx125x_reg_w(SX125x_REG_RX_BW__BB_BW, 0, rx);
    lgw_sx125x_reg_w(SX125x_REG_RX_BW__ADC_TRIM, 6, rx);
    //sx125x_reg_w(SX125x_RX_BW__ADC_BW, 7, rx);  /* Default: 7 */
    lgw_sx125x_reg_w(SX125x_REG_RX_PLL_BW__PLL_BW, 0, rx);
    lgw_sx125x_reg_w(SX125x_REG_TX_BW__PLL_BW, 0, tx);
    //sx125x_reg_w(SX125x_TX_BW__ANA_BW, 0, tx); /* Default: 0 */
    lgw_sx125x_reg_w(SX125x_REG_TX_DAC_BW, 5, tx);
    //sx125x_reg_w(SX125x_CLK_SELECT__DAC_CLK_SELECT, 0, tx); /* Use internal clock, in case no Tx connection from SX1301, Default: 0  */
    if (use_loopback == true) {
        lgw_sx125x_reg_w(SX125x_REG_TX_GAIN__DAC_GAIN, 3, tx);
        lgw_sx125x_reg_w(SX125x_REG_TX_GAIN__MIX_GAIN, 10, tx); //8
        lgw_sx125x_reg_w(SX125x_REG_CLK_SELECT__RF_LOOPBACK_EN, 1, tx);
        lgw_sx125x_reg_w(SX125x_REG_MODE, 15, tx);
    } else {
        lgw_sx125x_reg_w(SX125x_REG_TX_GAIN__DAC_GAIN, 3, tx);
        lgw_sx125x_reg_w(SX125x_REG_TX_GAIN__MIX_GAIN, 15, tx);
        lgw_sx125x_reg_w(SX125x_REG_MODE, 3, rx);
        lgw_sx125x_reg_w(SX125x_REG_MODE, 13, tx);
    }
    wait_ms(10);
    lgw_sx125x_reg_r(SX125x_REG_MODE_STATUS__RX_PLL_LOCKED, &rx_pll_locked, rx);
    lgw_sx125x_reg_r(SX125x_REG_MODE_STATUS__TX_PLL_LOCKED, &tx_pll_locked, tx);
    if ((rx_pll_locked == 0) || (tx_pll_locked == 0)) {
        DEBUG_MSG("ERROR: PLL failed to lock\n");
        return LGW_HAL_ERROR;
    }

    /* Trig calibration */
#if 0
    lgw_reg_w(LGW_TX_DATA_BUF_ADDR, 0);
    lgw_reg_w(LGW_TX_DATA_BUF_DATA, rf_chain);
    lgw_reg_w(LGW_TX_DATA_BUF_DATA, (int32_t)(CAL_TX_TONE_FREQ_HZ*64e-6));
    lgw_reg_w(LGW_PAGE_REG, 3);
    lgw_reg_w(LGW_EMERGENCY_FORCE_HOST_CTRL, 0);

    wait_ms(125); // Calibration duration

    lgw_reg_w(LGW_EMERGENCY_FORCE_HOST_CTRL, 1);
#else
    DEBUG_PRINTF("CAL: start RX calibration of rf_chain %u\n", rf_chain);

    /* Set calibration parameters */
    sx1302_agc_mailbox_write(2, rf_chain); /* Set RX test config: radioA:0 radioB:1 */
    sx1302_agc_mailbox_write(1, CAL_TX_TONE_FREQ_HZ * 64e-6); /* Set frequency */

    /* send calibration request (with above parameters) */
    sx1302_agc_mailbox_write(3, 0x55);

    /* sync */
    sx1302_agc_wait_status(0x00);

    /* reset calibration request */
    sx1302_agc_mailbox_write(3, 0x00);

    /* Wait for calibration to be completed */
    DEBUG_MSG("  CAL: waiting for RX calibration to complete...\n");
    sx1302_agc_wait_status((rf_chain == 0) ? 0x11 : 0x22);
    DEBUG_MSG("CAL: RX Calibration Done\n");
#endif

    /* Get calibration results */
    if (rf_chain) {
        lgw_reg_r(SX1302_REG_RADIO_FE_IQ_COMP_AMP_COEFF_RADIO_B_AMP_COEFF, &r); /* TODO: REG_SELECT */
        res->amp = (r > 31) ? (int8_t)(r - 64) : (int8_t)r;
        lgw_reg_r(SX1302_REG_RADIO_FE_IQ_COMP_PHI_COEFF_RADIO_B_PHI_COEFF, &r); /* TODO: REG_SELECT */
        res->phi = (r > 31) ? (int8_t)(r - 64) : (int8_t)r;
    }
    else {
        lgw_reg_r(SX1302_REG_RADIO_FE_IQ_COMP_AMP_COEFF_RADIO_A_AMP_COEFF, &r);
        res->amp = (r > 31) ? (int8_t)(r - 64) : (int8_t)r;
        lgw_reg_r(SX1302_REG_RADIO_FE_IQ_COMP_PHI_COEFF_RADIO_A_PHI_COEFF, &r);
        res->phi = (r > 31) ? (int8_t)(r - 64) : (int8_t)r;
    }

    printf("  CAL: rf_chain:%u amp:%d phi:%d\n", rf_chain, res->amp, res->phi);

#if 0
    lgw_reg_w(LGW_DBG_AGC_MCU_RAM_ADDR, 0x24);
    lgw_reg_r(LGW_DBG_AGC_MCU_RAM_DATA, &r);
    res->snr = (uint8_t)r;
    lgw_reg_w(LGW_DBG_AGC_MCU_RAM_ADDR, 0x27);
    lgw_reg_r(LGW_DBG_AGC_MCU_RAM_DATA, &r);
    res->rej_init = (uint8_t)r;
    lgw_reg_w(LGW_DBG_AGC_MCU_RAM_ADDR, 0x2A);
    lgw_reg_r(LGW_DBG_AGC_MCU_RAM_DATA, &r);
    res->rej = (uint8_t)r;
#else
    sx1302_agc_mailbox_read(3, &val);
    printf("  CAL: RSIG:0x%02X\n", val);
    sx1302_agc_mailbox_read(2, &val);
    printf("  CAL: SNR:0x%02X\n", val);
    res->snr = val;
    sx1302_agc_mailbox_read(1, &val);
    printf("  CAL: REJ_INIT:0x%02X\n", val);
    res->rej_init = val;
    sx1302_agc_mailbox_read(0, &val);
    printf("  CAL: REJ:0x%02X\n", val);
    res->rej = val;
#endif

    return LGW_HAL_SUCCESS;
}

/* ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ */

int sx125x_cal_tx_dc_offset(uint8_t rf_chain, uint32_t freq_hz, uint8_t dac_gain, uint8_t mix_gain, uint8_t radio_type, struct lgw_sx125x_cal_tx_result_s * res) {

    uint32_t rx_freq_hz, tx_freq_hz;
    uint32_t rx_freq_int, rx_freq_frac;
    uint32_t tx_freq_int, tx_freq_frac;
    uint8_t rx_pll_locked, tx_pll_locked;
    int32_t r;
    uint16_t reg;
    uint8_t val;

    DEBUG_MSG("\n");
    DEBUG_PRINTF("rf_chain:%u, freq_hz:%u, dac_gain:%u, mix_gain:%u, radio_type:%d\n", rf_chain, freq_hz, dac_gain, mix_gain, radio_type);

    /* Set PLL frequencies */
    rx_freq_hz = freq_hz - CAL_TX_TONE_FREQ_HZ;
    tx_freq_hz = freq_hz;
    switch (radio_type) {
        case LGW_RADIO_TYPE_SX1255:
            rx_freq_int = rx_freq_hz / (SX125x_32MHz_FRAC << 7); /* integer part, gives the MSB */
            rx_freq_frac = ((rx_freq_hz % (SX125x_32MHz_FRAC << 7)) << 9) / SX125x_32MHz_FRAC; /* fractional part, gives middle part and LSB */
            tx_freq_int = tx_freq_hz / (SX125x_32MHz_FRAC << 7); /* integer part, gives the MSB */
            tx_freq_frac = ((tx_freq_hz % (SX125x_32MHz_FRAC << 7)) << 9) / SX125x_32MHz_FRAC; /* fractional part, gives middle part and LSB */
            break;
        case LGW_RADIO_TYPE_SX1257:
            rx_freq_int = rx_freq_hz / (SX125x_32MHz_FRAC << 8); /* integer part, gives the MSB */
            rx_freq_frac = ((rx_freq_hz % (SX125x_32MHz_FRAC << 8)) << 8) / SX125x_32MHz_FRAC; /* fractional part, gives middle part and LSB */
            tx_freq_int = tx_freq_hz / (SX125x_32MHz_FRAC << 8); /* integer part, gives the MSB */
            tx_freq_frac = ((tx_freq_hz % (SX125x_32MHz_FRAC << 8)) << 8) / SX125x_32MHz_FRAC; /* fractional part, gives middle part and LSB */
            break;
        default:
            DEBUG_PRINTF("ERROR: UNEXPECTED VALUE %d FOR RADIO TYPE\n", radio_type);
            return LGW_HAL_ERROR;
    }
    lgw_sx125x_reg_w(SX125x_REG_FRF_RX_MSB, 0xFF & rx_freq_int, rf_chain);
    lgw_sx125x_reg_w(SX125x_REG_FRF_RX_MID, 0xFF & (rx_freq_frac >> 8), rf_chain);
    lgw_sx125x_reg_w(SX125x_REG_FRF_RX_LSB, 0xFF & rx_freq_frac, rf_chain);
    lgw_sx125x_reg_w(SX125x_REG_FRF_TX_MSB, 0xFF & tx_freq_int, rf_chain);
    lgw_sx125x_reg_w(SX125x_REG_FRF_TX_MID, 0xFF & (tx_freq_frac >> 8), rf_chain);
    lgw_sx125x_reg_w(SX125x_REG_FRF_TX_LSB, 0xFF & tx_freq_frac, rf_chain);

    /* Radio settings for calibration */
    //lgw_sx125x_reg_w(SX125x_RX_ANA_GAIN__LNA_ZIN, 1, rf_chain); /* Default: 1 */
    //lgw_sx125x_reg_w(SX125x_RX_ANA_GAIN__BB_GAIN, 15, rf_chain); /* Default: 15 */
    //lgw_sx125x_reg_w(SX125x_RX_ANA_GAIN__LNA_GAIN, 1, rf_chain); /* Default: 1 */
    lgw_sx125x_reg_w(SX125x_REG_RX_BW__BB_BW, 0, rf_chain);
    lgw_sx125x_reg_w(SX125x_REG_RX_BW__ADC_TRIM, 6, rf_chain);
    //lgw_sx125x_reg_w(SX125x_RX_BW__ADC_BW, 7, rf_chain);  /* Default: 7 */
    lgw_sx125x_reg_w(SX125x_REG_RX_PLL_BW__PLL_BW, 0, rf_chain);
    lgw_sx125x_reg_w(SX125x_REG_TX_BW__PLL_BW, 0, rf_chain);
    //lgw_sx125x_reg_w(SX125x_TX_BW__ANA_BW, 0, rf_chain); /* Default: 0 */
    lgw_sx125x_reg_w(SX125x_REG_TX_DAC_BW, 5, rf_chain);
    lgw_sx125x_reg_w(SX125x_REG_CLK_SELECT__DAC_CLK_SELECT, 1, rf_chain); /* Use external clock from SX1301 */
    lgw_sx125x_reg_w(SX125x_REG_TX_GAIN__DAC_GAIN, dac_gain, rf_chain);
    lgw_sx125x_reg_w(SX125x_REG_TX_GAIN__MIX_GAIN, mix_gain, rf_chain);
    lgw_sx125x_reg_w(SX125x_REG_CLK_SELECT__RF_LOOPBACK_EN, 1, rf_chain);
    lgw_sx125x_reg_w(SX125x_REG_MODE, 15, rf_chain);
    wait_ms(1);
    lgw_sx125x_reg_r(SX125x_REG_MODE_STATUS__RX_PLL_LOCKED, &rx_pll_locked, rf_chain);
    lgw_sx125x_reg_r(SX125x_REG_MODE_STATUS__TX_PLL_LOCKED, &tx_pll_locked, rf_chain);
    if ((rx_pll_locked == 0) || (tx_pll_locked == 0)) {
        DEBUG_MSG("ERROR: PLL failed to lock\n");
        return LGW_HAL_ERROR;
    }

    /* Trig calibration */
#if 0
    lgw_reg_w(LGW_TX_DATA_BUF_ADDR, 0);
    lgw_reg_w(LGW_TX_DATA_BUF_DATA, 0x02 | rf_chain);
    lgw_reg_w(LGW_TX_DATA_BUF_DATA, (int32_t)(CAL_TX_TONE_FREQ_HZ*64e-6));

    lgw_reg_w(LGW_PAGE_REG, 3);
    lgw_reg_w(LGW_EMERGENCY_FORCE_HOST_CTRL, 0);

    wait_ms(150); // Calibration duration

    lgw_reg_w(LGW_EMERGENCY_FORCE_HOST_CTRL, 1);
#else
    DEBUG_PRINTF("CAL: start TX calibration of rf_chain %u\n", rf_chain);

    /* Set calibration parameters */
    sx1302_agc_mailbox_write(2, rf_chain + 2); /* Set TX test config: radioA:2 radioB:3 */
    sx1302_agc_mailbox_write(1, CAL_TX_TONE_FREQ_HZ * 64e-6); /* Set frequency */

    /* send calibration request (with above parameters) */
    sx1302_agc_mailbox_write(3, 0x55);

    /* sync */
    sx1302_agc_wait_status(0x00);

    /* reset calibration request */
    sx1302_agc_mailbox_write(3, 0x00);

    /* Wait for calibration to be completed */
    DEBUG_MSG("  CAL: waiting for TX calibration to complete...\n");
    sx1302_agc_wait_status((rf_chain == 0) ? 0x33 : 0x44);
    DEBUG_MSG("CAL: TX Calibration Done\n");
#endif

    /* Get calibration results */
#if 0
    lgw_reg_r(LGW_TX_OFFSET_I, &r);
    res->offset_i = (int8_t)r;
    lgw_reg_r(LGW_TX_OFFSET_Q, &r);
    res->offset_q = (int8_t)r;
    lgw_reg_w(LGW_DBG_AGC_MCU_RAM_ADDR, 0x32);
    lgw_reg_r(LGW_DBG_AGC_MCU_RAM_DATA, &r);
    res->sig = (uint8_t)r;
    lgw_reg_w(LGW_DBG_AGC_MCU_RAM_ADDR, 0x37);
    lgw_reg_r(LGW_DBG_AGC_MCU_RAM_DATA, &r);
    res->rej = (uint8_t)r;
#else
    reg = REG_SELECT(rf_chain,  SX1302_REG_TX_TOP_A_TX_RFFE_IF_I_OFFSET_I_OFFSET,
                                SX1302_REG_TX_TOP_B_TX_RFFE_IF_I_OFFSET_I_OFFSET);
    lgw_reg_r(reg, &r);
    res->offset_i = (int8_t)r;

    reg = REG_SELECT(rf_chain,  SX1302_REG_TX_TOP_A_TX_RFFE_IF_Q_OFFSET_Q_OFFSET,
                                SX1302_REG_TX_TOP_B_TX_RFFE_IF_Q_OFFSET_Q_OFFSET);
    lgw_reg_r(reg, &r);
    res->offset_q = (int8_t)r;

    printf("  CAL: offset_i:%d offset_q:%d\n", res->offset_i, res->offset_q);

    sx1302_agc_mailbox_read(3, &val);
    printf("  CAL: TX_SIG:0x%02X\n", val);
    res->sig = (uint8_t)val;
    sx1302_agc_mailbox_read(2, &val);
    printf("  CAL: dec_gain:0x%02X\n", val);
    sx1302_agc_mailbox_read(1, &val);
    printf("  CAL: TX_DC_INIT:0x%02X\n", val);
    sx1302_agc_mailbox_read(0, &val);
    printf("  CAL: TX_DC_REJ:0x%02X\n", val);
    res->rej = (uint8_t)val;
#endif

    return LGW_HAL_SUCCESS;
}

/* ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ */

void cal_rx_result_init(struct lgw_sx125x_cal_rx_result_s *res_rx_min, struct lgw_sx125x_cal_rx_result_s *res_rx_max) {
    res_rx_min->amp = 31;
    res_rx_min->phi = 31;
    res_rx_min->rej = 255;
    res_rx_min->rej_init = 255;
    res_rx_min->snr = 255;

    res_rx_max->amp = -32;
    res_rx_max->phi = -32;
    res_rx_max->rej = 0;
    res_rx_max->rej_init = 0;
    res_rx_max->snr = 0;
}

/* ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ */

void cal_rx_result_sort(struct lgw_sx125x_cal_rx_result_s *res_rx, struct lgw_sx125x_cal_rx_result_s *res_rx_min, struct lgw_sx125x_cal_rx_result_s *res_rx_max) {
    if (res_rx->amp < res_rx_min->amp)
        res_rx_min->amp = res_rx->amp;
    if (res_rx->phi < res_rx_min->phi)
        res_rx_min->phi = res_rx->phi;
    if (res_rx->rej < res_rx_min->rej)
        res_rx_min->rej = res_rx->rej;
    if (res_rx->rej_init < res_rx_min->rej_init)
        res_rx_min->rej_init = res_rx->rej_init;
    if (res_rx->snr < res_rx_min->snr)
        res_rx_min->snr = res_rx->snr;

    if (res_rx->amp > res_rx_max->amp)
        res_rx_max->amp = res_rx->amp;
    if (res_rx->phi > res_rx_max->phi)
        res_rx_max->phi = res_rx->phi;
    if (res_rx->rej > res_rx_max->rej)
        res_rx_max->rej = res_rx->rej;
    if (res_rx->rej_init > res_rx_max->rej_init)
        res_rx_max->rej_init = res_rx->rej_init;
    if (res_rx->snr > res_rx_max->snr)
        res_rx_max->snr = res_rx->snr;
}

/* ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ */

bool cal_rx_result_assert(struct lgw_sx125x_cal_rx_result_s *res_rx_min, struct lgw_sx125x_cal_rx_result_s *res_rx_max) {
    if (    ((res_rx_max->amp - res_rx_min->amp) > 4)
         || ((res_rx_max->phi - res_rx_min->phi) > 4)
         || (res_rx_min->rej < 50)
         || (res_rx_min->snr < 50) )
        return false;
    else
        return true;
}

/* ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ */

void cal_tx_result_init(struct lgw_sx125x_cal_tx_result_s *res_tx_min, struct lgw_sx125x_cal_tx_result_s *res_tx_max) {
    res_tx_min->offset_i = 127;
    res_tx_min->offset_q = 127;
    res_tx_min->rej = 255;
    res_tx_min->sig = 255;

    res_tx_max->offset_i = -128;
    res_tx_max->offset_q = -128;
    res_tx_max->rej = 0;
    res_tx_max->sig = 0;
}

/* ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ */

void cal_tx_result_sort(struct lgw_sx125x_cal_tx_result_s *res_tx, struct lgw_sx125x_cal_tx_result_s *res_tx_min, struct lgw_sx125x_cal_tx_result_s *res_tx_max) {
    if (res_tx->offset_i < res_tx_min->offset_i)
        res_tx_min->offset_i = res_tx->offset_i;
    if (res_tx->offset_q < res_tx_min->offset_q)
        res_tx_min->offset_q = res_tx->offset_q;
    if (res_tx->rej < res_tx_min->rej)
        res_tx_min->rej = res_tx->rej;
    if (res_tx->sig < res_tx_min->sig)
        res_tx_min->sig = res_tx->sig;

    if (res_tx->offset_i > res_tx_max->offset_i)
        res_tx_max->offset_i = res_tx->offset_i;
    if (res_tx->offset_q > res_tx_max->offset_q)
        res_tx_max->offset_q = res_tx->offset_q;
    if (res_tx->rej > res_tx_max->rej)
        res_tx_max->rej = res_tx->rej;
    if (res_tx->sig > res_tx_max->sig)
        res_tx_max->sig = res_tx->sig;
}

bool cal_tx_result_assert(struct lgw_sx125x_cal_tx_result_s *res_tx_min, struct lgw_sx125x_cal_tx_result_s *res_tx_max) {
    if (    ((res_tx_max->offset_i - res_tx_min->offset_i) > 4)
         || ((res_tx_max->offset_q - res_tx_min->offset_q) > 4)
         || (res_tx_min->rej < 10) )
        return false;
    else
        return true;
}

/* --- EOF ------------------------------------------------------------------ */
