/*
 / _____)             _              | |
( (____  _____ ____ _| |_ _____  ____| |__
 \____ \| ___ |    (_   _) ___ |/ ___)  _ \
 _____) ) ____| | | || |_| ____( (___| | | |
(______/|_____)_|_|_| \__)_____)\____)_| |_|
  (C)2019 Semtech

Description:
    LoRa concentrator radio calibration functions

License: Revised BSD License, see LICENSE.TXT file include in the project
*/


/* -------------------------------------------------------------------------- */
/* --- DEPENDANCIES --------------------------------------------------------- */

#include <stdint.h>     /* C99 types */
#include <stdio.h>      /* printf fprintf */
#include <math.h>       /* log10 */

#include "loragw_reg.h"
#include "loragw_aux.h"
#include "loragw_hal.h"
#include "loragw_sx1302.h"
#include "loragw_sx125x.h"
#include "loragw_cal.h"

/* -------------------------------------------------------------------------- */
/* --- DEBUG FLAGS ---------------------------------------------------------- */

#define TX_CALIB_DONE_BY_HAL    0

/* -------------------------------------------------------------------------- */
/* --- PRIVATE MACROS ------------------------------------------------------- */

#define ARRAY_SIZE(a) (sizeof(a) / sizeof((a)[0]))
#if DEBUG_CAL == 1
    #define DEBUG_MSG(str)                fprintf(stdout, str)
    #define DEBUG_PRINTF(fmt, args...)    fprintf(stdout,"%s:%d: "fmt, __FUNCTION__, __LINE__, args)
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
#define CAL_TX_CORR_DURATION    0 /* 0:1ms, 1:2ms, 2:4ms, 3:8ms */

/* -------------------------------------------------------------------------- */
/* --- PRIVATE VARIABLES -------------------------------------------- */

/* Record Rx IQ mismatch corrections from calibration */
static int8_t rf_rx_image_amp[LGW_RF_CHAIN_NB] = {0, 0};
static int8_t rf_rx_image_phi[LGW_RF_CHAIN_NB] = {0, 0};

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

int sx1302_cal_start(uint8_t version, struct lgw_conf_rxrf_s * rf_chain_cfg, struct lgw_tx_gain_lut_s * txgain_lut) {
    int i, j, k;
    uint8_t val;
    bool cal_status = false;
    uint8_t x_max;
    int x_max_idx;
    uint8_t dac_gain[LGW_RF_CHAIN_NB][TX_GAIN_LUT_SIZE_MAX];
    uint8_t mix_gain[LGW_RF_CHAIN_NB][TX_GAIN_LUT_SIZE_MAX];
    int8_t offset_i[LGW_RF_CHAIN_NB][TX_GAIN_LUT_SIZE_MAX];
    int8_t offset_q[LGW_RF_CHAIN_NB][TX_GAIN_LUT_SIZE_MAX];
    uint8_t nb_gains[LGW_RF_CHAIN_NB];
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
    sx1302_agc_mailbox_write(3, 0xFF);

    /* Wait for AGC to acknoledge */
    sx1302_agc_wait_status(0x00);

    printf("CAL: started\n");

    /* Run Rx image calibration */
    for (i = 0; i < LGW_RF_CHAIN_NB; i++) {
        if (rf_chain_cfg[i].enable) {
            /* Calibration using the other radio for Tx */
            if (rf_chain_cfg[0].type == rf_chain_cfg[1].type) {
                cal_rx_result_init(&cal_rx_min, &cal_rx_max);
                for (j = 0; j < CAL_ITER; j++) {
                    sx125x_cal_rx_image(i, rf_chain_cfg[i].freq_hz, false, rf_chain_cfg[i].type, &cal_rx[j]);
                    cal_rx_result_sort(&cal_rx[j], &cal_rx_min, &cal_rx_max);
                }
                cal_status = cal_rx_result_assert(&cal_rx_min, &cal_rx_max);
            }

            /* If failed or different radios, run calibration using RF loopback (assuming that it is better than no calibration) */
            if ((cal_status == false) || (rf_chain_cfg[0].type != rf_chain_cfg[1].type)) {
                cal_rx_result_init(&cal_rx_min, &cal_rx_max);
                for (j = 0; j < CAL_ITER; j++) {
                    sx125x_cal_rx_image(i, rf_chain_cfg[i].freq_hz, true, rf_chain_cfg[i].type, &cal_rx[j]);
                    cal_rx_result_sort(&cal_rx[j], &cal_rx_min, &cal_rx_max);
                }
                cal_status = cal_rx_result_assert(&cal_rx_min, &cal_rx_max);
            }

            if (cal_status == false) {
                DEBUG_MSG("*********************************************\n");
                DEBUG_PRINTF("ERROR: Rx image calibration of radio %d failed\n",i);
                DEBUG_MSG("*********************************************\n");
                return LGW_HAL_ERROR;
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
    for (k = 0; k < LGW_RF_CHAIN_NB; k++) {
        nb_gains[k] = 0;
        for (i = 0; i < txgain_lut[k].size; i++) {
            unique_gains = true;
            for (j = 0; j < nb_gains[k]; j++) {
                if ((txgain_lut[k].lut[i].dac_gain == dac_gain[k][j]) && (txgain_lut[k].lut[i].mix_gain == mix_gain[k][j])) {
                    unique_gains = false;
                }
            }
            if (unique_gains) {
                dac_gain[k][nb_gains[k]] = txgain_lut[k].lut[i].dac_gain;
                mix_gain[k][nb_gains[k]] = txgain_lut[k].lut[i].mix_gain;
                nb_gains[k] += 1;
            }
        }
    }

    /* Run Tx image calibration */
    for (i = 0; i < LGW_RF_CHAIN_NB; i++) {
        if (rf_chain_cfg[i].tx_enable) {
            for (j = 0; j < nb_gains[i]; j++) {
                cal_tx_result_init(&cal_tx_min, &cal_tx_max);
                for (k = 0; k < CAL_ITER; k++){
                    sx125x_cal_tx_dc_offset(i, rf_chain_cfg[i].freq_hz, dac_gain[i][j], mix_gain[i][j], rf_chain_cfg[i].type, &cal_tx[k]);
                    cal_tx_result_sort(&cal_tx[k], &cal_tx_min, &cal_tx_max);
                }
                cal_status = cal_tx_result_assert(&cal_tx_min, &cal_tx_max);

                if (cal_status == false) {
                    DEBUG_MSG("*********************************************\n");
                    DEBUG_PRINTF("ERROR: Tx DC offset calibration of radio %d for DAC gain %d and mixer gain %2d failed\n", i, dac_gain[i][j], mix_gain[i][j]);
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
                offset_i[i][j] = cal_tx[x_max_idx].offset_i;
                offset_q[i][j] = cal_tx[x_max_idx].offset_q;

                DEBUG_PRINTF("INFO: Tx DC offset calibration of radio %d for DAC gain %d and mixer gain %2d succeeded. Improved DC rejection by %2d dB (I:%4d Q:%4d)\n", i, dac_gain[i][j], mix_gain[i][j], cal_tx[x_max_idx].rej, cal_tx[x_max_idx].offset_i, cal_tx[x_max_idx].offset_q);
            }
        }
    }

    /* Fill DC offsets in Tx LUT */
    for (k = 0; k < LGW_RF_CHAIN_NB; k++) {
        for (i = 0; i < txgain_lut[k].size; i++) {
            for (j = 0; j < nb_gains[k]; j++) {
                if ((txgain_lut[k].lut[i].dac_gain == dac_gain[k][j]) && (txgain_lut[k].lut[i].mix_gain == mix_gain[k][j])) {
                    break;
                }
            }
            txgain_lut[k].lut[i].offset_i = offset_i[k][j];
            txgain_lut[k].lut[i].offset_q = offset_q[k][j];
        }
    }

    printf("-------------------------------------------------------------------\n");
    printf("Radio calibration completed:\n");
    printf("  RadioA: amp:%d phi:%d\n", rf_rx_image_amp[0], rf_rx_image_phi[0]);
    printf("  RadioB: amp:%d phi:%d\n", rf_rx_image_amp[1], rf_rx_image_phi[1]);
    for (k = 0; k < LGW_RF_CHAIN_NB; k++) {
        printf("  TX calibration params for rf_chain %d:\n", k);
        for (i = 0; i < txgain_lut[k].size; i++) {
            printf("  -- power:%d\tdac:%u\tmix:%u\toffset_i:%d\toffset_q:%d\n", txgain_lut[k].lut[i].rf_power, txgain_lut[k].lut[i].dac_gain, txgain_lut[k].lut[i].mix_gain, txgain_lut[k].lut[i].offset_i, txgain_lut[k].lut[i].offset_q);
        }
    }
    printf("-------------------------------------------------------------------\n");

    return LGW_HAL_SUCCESS;
}

/* ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ */

int sx125x_cal_rx_image(uint8_t rf_chain, uint32_t freq_hz, bool use_loopback, uint8_t radio_type, struct lgw_sx125x_cal_rx_result_s * res) {
    uint8_t rx, tx;
    uint32_t rx_freq_hz, tx_freq_hz;
    uint32_t rx_freq_int, rx_freq_frac;
    uint32_t tx_freq_int, tx_freq_frac;
    uint8_t rx_pll_locked, tx_pll_locked;
    uint8_t rx_threshold = 8; /* Used by AGC to set decimation gain to increase signal and its image: value is MSB => x * 256 */

    printf("\n%s: rf_chain:%u, freq_hz:%u, loopback:%d, radio_type:%d\n", __FUNCTION__, rf_chain, freq_hz, use_loopback, radio_type);

    /* Indentify which radio is transmitting the test tone */
    rx = rf_chain;
    if (use_loopback == true) {
        tx = rf_chain;
    } else {
        tx = 1-rf_chain;
    }

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
    sx125x_reg_w(SX125x_REG_FRF_RX_MSB, 0xFF & rx_freq_int, rx);
    sx125x_reg_w(SX125x_REG_FRF_RX_MID, 0xFF & (rx_freq_frac >> 8), rx);
    sx125x_reg_w(SX125x_REG_FRF_RX_LSB, 0xFF & rx_freq_frac, rx);
    sx125x_reg_w(SX125x_REG_FRF_TX_MSB, 0xFF & tx_freq_int, tx);
    sx125x_reg_w(SX125x_REG_FRF_TX_MID, 0xFF & (tx_freq_frac >> 8), tx);
    sx125x_reg_w(SX125x_REG_FRF_TX_LSB, 0xFF & tx_freq_frac, tx);

    /* Radio settings for calibration */
    //sx125x_reg_w(SX125x_RX_ANA_GAIN__LNA_ZIN, 1, rx); /* Default: 1 */
    //sx125x_reg_w(SX125x_RX_ANA_GAIN__BB_GAIN, 15, rx); /* Default: 15 */
    //sx125x_reg_w(SX125x_RX_ANA_GAIN__LNA_GAIN, 1, rx); /* Default: 1 */
    sx125x_reg_w(SX125x_REG_RX_BW__BB_BW, 0, rx);
    sx125x_reg_w(SX125x_REG_RX_BW__ADC_TRIM, 6, rx);
    //sx125x_reg_w(SX125x_RX_BW__ADC_BW, 7, rx);  /* Default: 7 */
    sx125x_reg_w(SX125x_REG_RX_PLL_BW__PLL_BW, 0, rx);
    sx125x_reg_w(SX125x_REG_TX_BW__PLL_BW, 0, tx);
    //sx125x_reg_w(SX125x_TX_BW__ANA_BW, 0, tx); /* Default: 0 */
    sx125x_reg_w(SX125x_REG_TX_DAC_BW, 5, tx);
    //sx125x_reg_w(SX125x_CLK_SELECT__DAC_CLK_SELECT, 0, tx); /* Use internal clock, in case no Tx connection from SX1302, Default: 0  */
    if (use_loopback == true) {
        sx125x_reg_w(SX125x_REG_TX_GAIN__DAC_GAIN, 3, tx);
        sx125x_reg_w(SX125x_REG_TX_GAIN__MIX_GAIN, 10, tx); //8
        sx125x_reg_w(SX125x_REG_CLK_SELECT__RF_LOOPBACK_EN, 1, tx);
        sx125x_reg_w(SX125x_REG_MODE, 15, tx);
    } else {
        sx125x_reg_w(SX125x_REG_TX_GAIN__DAC_GAIN, 3, tx);
        sx125x_reg_w(SX125x_REG_TX_GAIN__MIX_GAIN, 15, tx);
        sx125x_reg_w(SX125x_REG_MODE, 3, rx);
        sx125x_reg_w(SX125x_REG_MODE, 13, tx);
    }
    wait_ms(10);
    sx125x_reg_r(SX125x_REG_MODE_STATUS__RX_PLL_LOCKED, &rx_pll_locked, rx);
    sx125x_reg_r(SX125x_REG_MODE_STATUS__TX_PLL_LOCKED, &tx_pll_locked, tx);
    if ((rx_pll_locked == 0) || (tx_pll_locked == 0)) {
        DEBUG_MSG("ERROR: PLL failed to lock\n");
        return LGW_HAL_ERROR;
    }

    /* Trig calibration */

    /* Select radio to be connected to the Signal Analyzer (warning: RadioA:1, RadioB:0) */
    lgw_reg_w(SX1302_REG_RADIO_FE_SIG_ANA_CFG_RADIO_SEL, (rf_chain == 0) ? 1 : 0);

    /* Set calibration parameters */
    sx1302_agc_mailbox_write(2, rf_chain); /* Set RX test config: radioA:0 radioB:1 */
    sx1302_agc_mailbox_write(1, CAL_TX_TONE_FREQ_HZ * 64e-6); /* Set frequency */
    sx1302_agc_mailbox_write(0, CAL_TX_CORR_DURATION);

    sx1302_agc_mailbox_write(3, 0x00);
    sx1302_agc_mailbox_write(3, 0x01);
    sx1302_agc_wait_status(0x01);

    sx1302_agc_mailbox_write(3, 0x02);
    sx1302_agc_wait_status(0x02);

    sx1302_agc_mailbox_write(3, 0x03);
    sx1302_agc_wait_status(0x03);

    sx1302_agc_mailbox_write(2, 0); /* dec_gain (not used) */
    sx1302_agc_mailbox_write(1, rx_threshold);

    sx1302_agc_mailbox_write(3, 0x04);

    /* Get calibration results */
    sx1302_agc_wait_status(0x06);
    uint8_t threshold, cal_dec_gain, rx_sig_1, rx_sig_0;
    sx1302_agc_mailbox_read(3, &threshold);
    sx1302_agc_mailbox_read(2, &cal_dec_gain);
    sx1302_agc_mailbox_read(1, &rx_sig_1);
    sx1302_agc_mailbox_read(0, &rx_sig_0);
    DEBUG_PRINTF("threshold:%u, cal_dec_gain:%u, rx_sig:%u\n", threshold * 256, cal_dec_gain, rx_sig_1 * 256 + rx_sig_0);
    sx1302_agc_mailbox_write(3, 0x06);

    sx1302_agc_wait_status(0x07);
    uint8_t rx_img_init_0, rx_img_init_1, amp, phi;
    sx1302_agc_mailbox_read(3, &rx_img_init_1);
    sx1302_agc_mailbox_read(2, &rx_img_init_0);
    sx1302_agc_mailbox_read(1, &amp);
    sx1302_agc_mailbox_read(0, &phi);
    DEBUG_PRINTF("rx_img_init_0:%u, rx_img_init_1:%u, amp:%d, phi:%d\n", rx_img_init_0, rx_img_init_1, (int8_t)amp, (int8_t)phi);
    sx1302_agc_mailbox_write(3, 0x07);

    sx1302_agc_wait_status(0x08);
    uint8_t rx_img_0, rx_img_1, rx_noise_raw_0, rx_noise_raw_1;
    float rx_img, rx_noise_raw, rx_img_init, rx_sig;
    sx1302_agc_mailbox_read(3, &rx_img_1);
    sx1302_agc_mailbox_read(2, &rx_img_0);
    sx1302_agc_mailbox_read(1, &rx_noise_raw_1);
    sx1302_agc_mailbox_read(0, &rx_noise_raw_0);
    DEBUG_PRINTF("rx_img_1:%u, rx_img_0:%u, rx_noise_raw_1:%u, rx_noise_raw_0:%u\n", rx_img_1, rx_img_0, rx_noise_raw_1, rx_noise_raw_0);
    rx_sig = (float)rx_sig_1 * 256 + (float)rx_sig_0;
    rx_noise_raw = (float)rx_noise_raw_1 * 256 + (float)rx_noise_raw_0;
    rx_img_init = (float)rx_img_init_1 * 256 + (float)rx_img_init_0;
    rx_img = (float)rx_img_1 * 256 + (float)rx_img_0;
    DEBUG_PRINTF("rx_img:%u, rx_noise_raw:%u\n", (uint16_t)rx_img, (uint16_t)rx_noise_raw);
    sx1302_agc_mailbox_write(3, 0x08);

    res->amp = (int8_t)amp;
    res->phi = (int8_t)phi;
    res->snr = (uint16_t)(20 * log10(rx_sig/rx_noise_raw));
    res->rej_init = (uint16_t)(20 * log10(rx_sig/rx_img_init));
    res->rej = (uint16_t)(20 * log10(rx_sig/rx_img));
    DEBUG_PRINTF("snr:%u, rej:%u, rej_init:%u\n", res->snr, res->rej, res->rej_init);

    /* Wait for calibration to be completed */
    DEBUG_MSG("  CAL: waiting for RX calibration to complete...\n");
    sx1302_agc_wait_status((rf_chain == 0) ? 0x11 : 0x22);
    DEBUG_MSG("CAL: RX Calibration Done\n");

    printf("%s, RESULT: rf_chain:%u amp:%d phi:%d\n", __FUNCTION__, rf_chain, res->amp, res->phi);

    return LGW_HAL_SUCCESS;
}

/* ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ */

int sx125x_cal_tx_dc_offset(uint8_t rf_chain, uint32_t freq_hz, uint8_t dac_gain, uint8_t mix_gain, uint8_t radio_type, struct lgw_sx125x_cal_tx_result_s * res) {

    uint32_t rx_freq_hz, tx_freq_hz;
    uint32_t rx_freq_int, rx_freq_frac;
    uint32_t tx_freq_int, tx_freq_frac;
    uint8_t rx_pll_locked, tx_pll_locked;
    uint16_t reg;
    uint8_t tx_threshold = 64;
    int i;

    printf("\n%s: rf_chain:%u, freq_hz:%u, dac_gain:%u, mix_gain:%u, radio_type:%d\n", __FUNCTION__, rf_chain, freq_hz, dac_gain, mix_gain, radio_type);

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
    sx125x_reg_w(SX125x_REG_FRF_RX_MSB, 0xFF & rx_freq_int, rf_chain);
    sx125x_reg_w(SX125x_REG_FRF_RX_MID, 0xFF & (rx_freq_frac >> 8), rf_chain);
    sx125x_reg_w(SX125x_REG_FRF_RX_LSB, 0xFF & rx_freq_frac, rf_chain);
    sx125x_reg_w(SX125x_REG_FRF_TX_MSB, 0xFF & tx_freq_int, rf_chain);
    sx125x_reg_w(SX125x_REG_FRF_TX_MID, 0xFF & (tx_freq_frac >> 8), rf_chain);
    sx125x_reg_w(SX125x_REG_FRF_TX_LSB, 0xFF & tx_freq_frac, rf_chain);

    /* Radio settings for calibration */
    //sx125x_reg_w(SX125x_RX_ANA_GAIN__LNA_ZIN, 1, rf_chain); /* Default: 1 */
    //sx125x_reg_w(SX125x_RX_ANA_GAIN__BB_GAIN, 15, rf_chain); /* Default: 15 */
    //sx125x_reg_w(SX125x_RX_ANA_GAIN__LNA_GAIN, 1, rf_chain); /* Default: 1 */
    sx125x_reg_w(SX125x_REG_RX_BW__BB_BW, 0, rf_chain);
    sx125x_reg_w(SX125x_REG_RX_BW__ADC_TRIM, 6, rf_chain);
    //sx125x_reg_w(SX125x_RX_BW__ADC_BW, 7, rf_chain);  /* Default: 7 */
    sx125x_reg_w(SX125x_REG_RX_PLL_BW__PLL_BW, 0, rf_chain);
    sx125x_reg_w(SX125x_REG_TX_BW__PLL_BW, 0, rf_chain);
    //sx125x_reg_w(SX125x_TX_BW__ANA_BW, 0, rf_chain); /* Default: 0 */
    sx125x_reg_w(SX125x_REG_TX_DAC_BW, 5, rf_chain);
    sx125x_reg_w(SX125x_REG_CLK_SELECT__DAC_CLK_SELECT, 1, rf_chain); /* Use external clock from SX1302 */
    sx125x_reg_w(SX125x_REG_TX_GAIN__DAC_GAIN, dac_gain, rf_chain);
    sx125x_reg_w(SX125x_REG_TX_GAIN__MIX_GAIN, mix_gain, rf_chain);
    sx125x_reg_w(SX125x_REG_CLK_SELECT__RF_LOOPBACK_EN, 1, rf_chain);
    sx125x_reg_w(SX125x_REG_MODE, 15, rf_chain);
    wait_ms(1);
    sx125x_reg_r(SX125x_REG_MODE_STATUS__RX_PLL_LOCKED, &rx_pll_locked, rf_chain);
    sx125x_reg_r(SX125x_REG_MODE_STATUS__TX_PLL_LOCKED, &tx_pll_locked, rf_chain);
    if ((rx_pll_locked == 0) || (tx_pll_locked == 0)) {
        DEBUG_MSG("ERROR: PLL failed to lock\n");
        return LGW_HAL_ERROR;
    }

    /* Trig calibration */

    /* Select radio to be connected to the Signal Analyzer (warning: RadioA:1, RadioB:0) */
    lgw_reg_w(SX1302_REG_RADIO_FE_SIG_ANA_CFG_RADIO_SEL, (rf_chain == 0) ? 1 : 0);

    reg = REG_SELECT(rf_chain,  SX1302_REG_TX_TOP_A_TX_RFFE_IF_CTRL_TX_MODE,
                                SX1302_REG_TX_TOP_B_TX_RFFE_IF_CTRL_TX_MODE);
    lgw_reg_w(reg, 0);

    reg = REG_SELECT(rf_chain,  SX1302_REG_TX_TOP_A_TX_TRIG_TX_TRIG_IMMEDIATE,
                                SX1302_REG_TX_TOP_B_TX_TRIG_TX_TRIG_IMMEDIATE);
    lgw_reg_w(reg, 1);
    lgw_reg_w(reg, 0);

    reg = REG_SELECT(rf_chain,  SX1302_REG_RADIO_FE_CTRL0_RADIO_A_DC_NOTCH_EN,
                                SX1302_REG_RADIO_FE_CTRL0_RADIO_B_DC_NOTCH_EN);
    lgw_reg_w(reg, 1);

#if TX_CALIB_DONE_BY_HAL /* For debug */

    lgw_reg_w(SX1302_REG_RADIO_FE_SIG_ANA_CFG_FORCE_HAL_CTRL, 1);
    agc_cal_tx_dc_offset(rf_chain, CAL_TX_TONE_FREQ_HZ * 64e-6, rf_rx_image_amp[rf_chain], rf_rx_image_phi[rf_chain], tx_threshold, 0, &(res->offset_i), &(res->offset_q), &(res->rej));
    lgw_reg_w(SX1302_REG_RADIO_FE_SIG_ANA_CFG_FORCE_HAL_CTRL, 0);

#else

    /* Set calibration parameters */
    sx1302_agc_mailbox_write(2, rf_chain + 2); /* Set TX test config: radioA:2 radioB:3 */
    sx1302_agc_mailbox_write(1, CAL_TX_TONE_FREQ_HZ * 64e-6); /* Set frequency */
    sx1302_agc_mailbox_write(0, 0); /* correlation duration: 0:1ms, 1:2ms, 2:4ms, 3:8ms) */

    sx1302_agc_mailbox_write(3, 0x00); /* sync */
    sx1302_agc_mailbox_write(3, 0x01); /* sync */
    sx1302_agc_wait_status(0x01);

    sx1302_agc_mailbox_write(2, rf_rx_image_amp[rf_chain]); /* amp */
    sx1302_agc_mailbox_write(1, rf_rx_image_phi[rf_chain]); /* phi */

    sx1302_agc_mailbox_write(3, 0x02); /* sync */
    sx1302_agc_wait_status(0x02);

    sx1302_agc_mailbox_write(2, 0); /* i offset init */
    sx1302_agc_mailbox_write(1, 0); /* q offset init */

    sx1302_agc_mailbox_write(3, 0x03); /* sync */
    sx1302_agc_wait_status(0x03);

    sx1302_agc_mailbox_write(2, 0);
    sx1302_agc_mailbox_write(1, tx_threshold);

    sx1302_agc_mailbox_write(3, 0x04); /* sync */

    /* Get calibration results */
    sx1302_agc_wait_status(0x06);
    uint8_t threshold, cal_dec_gain, tx_sig_0, tx_sig_1;
    sx1302_agc_mailbox_read(3, &threshold);
    sx1302_agc_mailbox_read(2, &cal_dec_gain);
    sx1302_agc_mailbox_read(1, &tx_sig_1);
    sx1302_agc_mailbox_read(0, &tx_sig_0);
    DEBUG_PRINTF("threshold:%u, cal_dec_gain:%u, tx_sig:%u\n", threshold * 256, cal_dec_gain, tx_sig_0 * 256 + tx_sig_1);
    sx1302_agc_mailbox_write(3, 0x06); /* sync */

    sx1302_agc_wait_status(0x07);
    uint8_t tx_dc_0, tx_dc_1, offset_i, offset_q;
    float tx_sig, tx_dc;
    sx1302_agc_mailbox_read(3, &tx_dc_1);
    sx1302_agc_mailbox_read(2, &tx_dc_0);
    sx1302_agc_mailbox_read(1, &offset_i);
    sx1302_agc_mailbox_read(0, &offset_q);
    tx_sig = (float)tx_sig_1 * 256 + (float)tx_sig_0;
    tx_dc = (float)tx_dc_1 * 256 + (float)tx_dc_0;
    res->rej = (uint16_t)(20 * log10(tx_sig/tx_dc));
    res->offset_i = (int8_t)offset_i;
    res->offset_q = (int8_t)offset_q;
    DEBUG_PRINTF("tx_dc:%u, offset_i:%d, offset_q:%d\n", tx_dc_0 * 256 + tx_dc_1, (int8_t)offset_i, (int8_t)offset_q);
    sx1302_agc_mailbox_write(3, 0x07); /* sync */

    /* -----------------------------------------------*/
    /* DEBUG: Get IQ offsets selected for iterations */
    uint8_t index[12];

    sx1302_agc_wait_status(0x08);
    sx1302_agc_mailbox_read(3, &index[0]);
    sx1302_agc_mailbox_read(2, &index[1]);
    sx1302_agc_mailbox_read(1, &index[2]);
    sx1302_agc_mailbox_read(0, &index[3]);
    sx1302_agc_mailbox_write(3, 0x08); /* sync */

    sx1302_agc_wait_status(0x09);
    sx1302_agc_mailbox_read(3, &index[4]);
    sx1302_agc_mailbox_read(2, &index[5]);
    sx1302_agc_mailbox_read(1, &index[6]);
    sx1302_agc_mailbox_read(0, &index[7]);
    sx1302_agc_mailbox_write(3, 0x09); /* sync */

    sx1302_agc_wait_status(0x0a);
    sx1302_agc_mailbox_read(3, &index[8]);
    sx1302_agc_mailbox_read(2, &index[9]);
    sx1302_agc_mailbox_read(1, &index[10]);
    sx1302_agc_mailbox_read(0, &index[11]);
    sx1302_agc_mailbox_write(3, 0x0a); /* sync */

#if DEBUG_CAL == 1
    int16_t lut_calib[9] = {64, 43, 28, 19, 13, 8, 6, 4, 2};
    int16_t offset_i_tmp = 0;
    int16_t offset_q_tmp = 0;

    printf("IQ sequence:\n");
    for (i = 0; i < 9; i++) {
        if (index[i] == 0) {
            offset_i_tmp = offset_i_tmp + 0;
            offset_q_tmp = offset_q_tmp + 0;

        }else if(index[i] == 1) {
            offset_i_tmp = offset_i_tmp + lut_calib[i];
            offset_q_tmp = offset_q_tmp + lut_calib[i];
        }else if(index[i] == 2) {
            offset_i_tmp = offset_i_tmp + lut_calib[i];
            offset_q_tmp = offset_q_tmp - lut_calib[i];
        }else if(index[i] == 3) {
            offset_i_tmp = offset_i_tmp - lut_calib[i];
            offset_q_tmp = offset_q_tmp + lut_calib[i];
        }else if(index[i] == 4) {
            offset_i_tmp = offset_i_tmp - lut_calib[i];
            offset_q_tmp = offset_q_tmp - lut_calib[i];
        }
        printf("i:%d q:%d\n", offset_i_tmp, offset_q_tmp);
    }
    printf("\n");

#endif /* DEBUG_CAL */
    /* -----------------------------------------------*/

    /* -----------------------------------------------*/
    /* DEBUG: Get TX_SIG returned by siognal analyzer */
    uint8_t msb[40];
    uint8_t lsb[40];

    for (i = 0; i < 20; i++) {
        sx1302_agc_wait_status(0x0c + i);
        sx1302_agc_mailbox_read(3, &msb[2*i]);
        sx1302_agc_mailbox_read(2, &lsb[2*i]);
        sx1302_agc_mailbox_read(1, &msb[2*i+1]);
        sx1302_agc_mailbox_read(0, &lsb[2*i+1]);
        sx1302_agc_mailbox_write(3, 0x0c + i); /* sync */
    }
    sx1302_agc_wait_status(0x0c + 20);

#if DEBUG_CAL == 1
    printf("TX_SIG values returned by signal analyzer:");
    for (i = 0; i < 40; i++) {
        if (i%5 == 0) {
            printf("\n");
        }
        printf("%u ", msb[i] * 256 + lsb[i]);
    }
    printf("\n");
#endif /* DEBUG_CAL */

    sx1302_agc_mailbox_write(3, 0x0c + 20); /* sync */
    /* -----------------------------------------------*/

    printf("%s: RESULT: offset_i:%d offset_q:%d rej:%u\n", __FUNCTION__, res->offset_i, res->offset_q, res->rej);

    /* Wait for calibration to be completed */
    DEBUG_MSG("waiting for TX calibration to complete...\n");
    sx1302_agc_wait_status((rf_chain == 0) ? 0x33 : 0x44);

#endif /* TX_CALIB_DONE_BY_HAL */

    DEBUG_MSG("TX Calibration Done\n");

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
    if (   ((res_tx_max->offset_i - res_tx_min->offset_i) > 4)
        || ((res_tx_max->offset_q - res_tx_min->offset_q) > 4)
        || (res_tx_min->rej < 10) )
        return false;
    else
        return true;
}

/* ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ */

#if TX_CALIB_DONE_BY_HAL

int8_t clip_8b(int8_t val1, int8_t val2) {
    int16_t a, b;

    a = (int16_t)val1;
    b = (int16_t)val2;

    if ( (a + b) > 127 ) {
        return 127;
    } else if ( (a+b) < -128 ) {
        return -128;
    } else {
        return (int8_t)(a+b);
    }
}

/* This functions implements what is being done by CAL fw for TX calibration */
void agc_cal_tx_dc_offset(uint8_t rf_chain, signed char freq, char amp_hal, char phi_hal, char level_reqired, char precision, int8_t * offset_i_res, int8_t * offset_q_res, uint16_t * rej) {
    signed char offset_i_set[9];
    signed char offset_q_set[9];
    signed char offset_i;
    signed char offset_q;
    const signed char span[] = {64, 43, 28, 19, 13, 8, 6, 4, 2};
    char dec_gain;
    char i, j; // loop variables
    char idx; // max/min variables
    uint16_t reg;
    int32_t abs_corr_max_i16;
    int32_t abs_corr_min_i16;
    int32_t abs_corr_i16;
    int32_t tx_sig_i16;
    int32_t tx_dc_i16;
    int DEC_GAIN_MAX = 11;
    int DEC_GAIN_MIN = 7;
    int32_t val;
    int32_t abs_lsb, abs_msb;

    reg = REG_SELECT(rf_chain, SX1302_REG_RADIO_FE_IQ_COMP_AMP_COEFF_RADIO_A_AMP_COEFF,
                                SX1302_REG_RADIO_FE_IQ_COMP_AMP_COEFF_RADIO_B_AMP_COEFF);
    lgw_reg_w(reg, amp_hal);

    reg = REG_SELECT(rf_chain, SX1302_REG_RADIO_FE_IQ_COMP_PHI_COEFF_RADIO_A_PHI_COEFF,
                                SX1302_REG_RADIO_FE_IQ_COMP_PHI_COEFF_RADIO_B_PHI_COEFF);
    lgw_reg_w(reg, phi_hal);

    lgw_reg_w(SX1302_REG_RADIO_FE_SIG_ANA_FREQ_FREQ, freq);

    lgw_reg_w(SX1302_REG_RADIO_FE_SIG_ANA_CFG_DURATION, precision);
    lgw_reg_w(SX1302_REG_RADIO_FE_SIG_ANA_CFG_EN, 1);

    // Set dec gain and signal analyser according to potential maximum DC level
    offset_i_set[0] = 0;
    offset_q_set[0] = 0;
    offset_i_set[1] = -span[0];
    offset_q_set[1] = -span[0];
    offset_i_set[2] = -span[0];
    offset_q_set[2] = span[0];
    offset_i_set[3] = span[0];
    offset_q_set[3] = -span[0];
    offset_i_set[4] = span[0];
    offset_q_set[4] = span[0];
    for (i = DEC_GAIN_MAX; i >= DEC_GAIN_MIN; i--) {
        dec_gain = i;
        /* ------------ */
        reg = REG_SELECT(rf_chain,  SX1302_REG_RADIO_FE_CTRL0_RADIO_A_HOST_FILTER_GAIN,
                                    SX1302_REG_RADIO_FE_CTRL0_RADIO_B_HOST_FILTER_GAIN);
        lgw_reg_w(reg, dec_gain);

        /* ------------ */
        reg = REG_SELECT(rf_chain,  SX1302_REG_TX_TOP_A_TX_RFFE_IF_I_OFFSET_I_OFFSET,
                                    SX1302_REG_TX_TOP_B_TX_RFFE_IF_I_OFFSET_I_OFFSET);
        lgw_reg_w(reg, (int8_t)offset_i_set[0]);

        reg = REG_SELECT(rf_chain,  SX1302_REG_TX_TOP_A_TX_RFFE_IF_Q_OFFSET_Q_OFFSET,
                                    SX1302_REG_TX_TOP_B_TX_RFFE_IF_Q_OFFSET_Q_OFFSET);
        lgw_reg_w(reg, (int8_t)offset_q_set[0]);

        /* ------------ */
        lgw_reg_w(SX1302_REG_RADIO_FE_SIG_ANA_CFG_START, 0);
        lgw_reg_w(SX1302_REG_RADIO_FE_SIG_ANA_CFG_START, 1);

        do {
            lgw_reg_r(SX1302_REG_RADIO_FE_SIG_ANA_CFG_VALID, &val);
            wait_ms(1);
            /* TODO: addtimeout */
        } while (val == 0);

        lgw_reg_r(SX1302_REG_RADIO_FE_SIG_ANA_ABS_LSB_CORR_ABS_OUT, &abs_lsb);
        lgw_reg_r(SX1302_REG_RADIO_FE_SIG_ANA_ABS_MSB_CORR_ABS_OUT, &abs_msb);
        /* ------------ */

        abs_corr_max_i16 = abs_msb * 256 + abs_lsb;

        idx = 0;
        for (j = 1; j < 5; j++) {
            reg = REG_SELECT(rf_chain,  SX1302_REG_TX_TOP_A_TX_RFFE_IF_I_OFFSET_I_OFFSET,
                                        SX1302_REG_TX_TOP_B_TX_RFFE_IF_I_OFFSET_I_OFFSET);
            lgw_reg_w(reg, (int8_t)offset_i_set[j]);

            reg = REG_SELECT(rf_chain,  SX1302_REG_TX_TOP_A_TX_RFFE_IF_Q_OFFSET_Q_OFFSET,
                                        SX1302_REG_TX_TOP_B_TX_RFFE_IF_Q_OFFSET_Q_OFFSET);
            lgw_reg_w(reg, (int8_t)offset_q_set[j]);

            lgw_reg_w(SX1302_REG_RADIO_FE_SIG_ANA_CFG_START, 0);
            lgw_reg_w(SX1302_REG_RADIO_FE_SIG_ANA_CFG_START, 1);

            do {
                lgw_reg_r(SX1302_REG_RADIO_FE_SIG_ANA_CFG_VALID, &val);
                wait_ms(1);
            } while (val == 0);

            lgw_reg_r(SX1302_REG_RADIO_FE_SIG_ANA_ABS_LSB_CORR_ABS_OUT, &abs_lsb);
            lgw_reg_r(SX1302_REG_RADIO_FE_SIG_ANA_ABS_MSB_CORR_ABS_OUT, &abs_msb);

            abs_corr_i16 = abs_msb * 256 + abs_lsb;

            if (abs_corr_i16 > abs_corr_max_i16) {
                abs_corr_max_i16 = abs_corr_i16;
                idx = j;
            }
        }

        if (abs_corr_max_i16 > (level_reqired * 256)) {
            break;
        }
    }

    printf("dec_gain:%d\n", dec_gain);

    // store the max results
    tx_sig_i16 = abs_corr_max_i16;
    printf("tx_sig:%d\n", tx_sig_i16);

    // Calbration algorithm
    offset_i = 0;
    offset_q = 0;
    for (i = 0; i<sizeof (span); i++) {

        offset_i_set[0] = offset_i;
        offset_q_set[0] = offset_q;
        offset_i_set[1] = clip_8b(offset_i, span[i]);
        offset_q_set[1] = clip_8b(offset_q, span[i]);
        offset_i_set[2] = clip_8b(offset_i, span[i]);
        offset_q_set[2] = clip_8b(offset_q, -span[i]);
        offset_i_set[3] = clip_8b(offset_i, -span[i]);
        offset_q_set[3] = clip_8b(offset_q, span[i]);
        offset_i_set[4] = clip_8b(offset_i, -span[i]);
        offset_q_set[4] = clip_8b(offset_q, -span[i]);

        //center point
        reg = REG_SELECT(rf_chain,  SX1302_REG_TX_TOP_A_TX_RFFE_IF_I_OFFSET_I_OFFSET,
                                    SX1302_REG_TX_TOP_B_TX_RFFE_IF_I_OFFSET_I_OFFSET);
        lgw_reg_w(reg, (int8_t)offset_i_set[0]);

        reg = REG_SELECT(rf_chain,  SX1302_REG_TX_TOP_A_TX_RFFE_IF_Q_OFFSET_Q_OFFSET,
                                    SX1302_REG_TX_TOP_B_TX_RFFE_IF_Q_OFFSET_Q_OFFSET);
        lgw_reg_w(reg, (int8_t)offset_q_set[0]);

        lgw_reg_w(SX1302_REG_RADIO_FE_SIG_ANA_CFG_START, 0);
        lgw_reg_w(SX1302_REG_RADIO_FE_SIG_ANA_CFG_START, 1);

        do {
            lgw_reg_r(SX1302_REG_RADIO_FE_SIG_ANA_CFG_VALID, &val);
            wait_ms(1);
        } while (val == 0);

        lgw_reg_r(SX1302_REG_RADIO_FE_SIG_ANA_ABS_LSB_CORR_ABS_OUT, &abs_lsb);
        lgw_reg_r(SX1302_REG_RADIO_FE_SIG_ANA_ABS_MSB_CORR_ABS_OUT, &abs_msb);

        abs_corr_min_i16 = abs_msb * 256 + abs_lsb;
        printf("abs_corr_min_i16:%d ", abs_corr_min_i16);

        idx = 0;

        //four points around
        for (j = 1; j < 5; j++) {
            reg = REG_SELECT(rf_chain,  SX1302_REG_TX_TOP_A_TX_RFFE_IF_I_OFFSET_I_OFFSET,
                                    SX1302_REG_TX_TOP_B_TX_RFFE_IF_I_OFFSET_I_OFFSET);
            lgw_reg_w(reg, (int8_t)offset_i_set[j]);

            reg = REG_SELECT(rf_chain,  SX1302_REG_TX_TOP_A_TX_RFFE_IF_Q_OFFSET_Q_OFFSET,
                                    SX1302_REG_TX_TOP_B_TX_RFFE_IF_Q_OFFSET_Q_OFFSET);
            lgw_reg_w(reg, (int8_t)offset_q_set[j]);

            lgw_reg_w(SX1302_REG_RADIO_FE_SIG_ANA_CFG_START, 0);
            lgw_reg_w(SX1302_REG_RADIO_FE_SIG_ANA_CFG_START, 1);

            do {
                lgw_reg_r(SX1302_REG_RADIO_FE_SIG_ANA_CFG_VALID, &val);
                wait_ms(1);
            } while (val == 0);

            lgw_reg_r(SX1302_REG_RADIO_FE_SIG_ANA_ABS_LSB_CORR_ABS_OUT, &abs_lsb);
            lgw_reg_r(SX1302_REG_RADIO_FE_SIG_ANA_ABS_MSB_CORR_ABS_OUT, &abs_msb);

            abs_corr_i16 = abs_msb * 256 + abs_lsb;
            printf("abs_corr_i16:%d ", abs_corr_i16);

            if (abs_corr_i16 < abs_corr_min_i16) {
                abs_corr_min_i16 = abs_corr_i16;
                idx = j;
            }
        }

        printf("\n");
        offset_i = offset_i_set[idx];
        offset_q = offset_q_set[idx];
    }

    offset_i_set[0] = clip_8b(offset_i, -1);
    offset_q_set[0] = clip_8b(offset_q, -1);
    offset_i_set[1] = clip_8b(offset_i, -1);
    offset_q_set[1] = offset_q;
    offset_i_set[2] = clip_8b(offset_i, -1);
    offset_q_set[2] = clip_8b(offset_q, 1);
    offset_i_set[3] = offset_i;
    offset_q_set[3] = clip_8b(offset_q, -1);
    offset_i_set[4] = offset_i;
    offset_q_set[4] = offset_q;
    offset_i_set[5] = offset_i;
    offset_q_set[5] = clip_8b(offset_q, 1);
    offset_i_set[6] = clip_8b(offset_i, 1);
    offset_q_set[6] = clip_8b(offset_q, -1);
    offset_i_set[7] = clip_8b(offset_i, 1);
    offset_q_set[7] = offset_q;
    offset_i_set[8] = clip_8b(offset_i, 1);
    offset_q_set[8] = clip_8b(offset_q, 1);

    //center point
    idx = 0;
    reg = REG_SELECT(rf_chain,  SX1302_REG_TX_TOP_A_TX_RFFE_IF_I_OFFSET_I_OFFSET,
                                SX1302_REG_TX_TOP_B_TX_RFFE_IF_I_OFFSET_I_OFFSET);
    lgw_reg_w(reg, (int8_t)offset_i_set[0]);

    reg = REG_SELECT(rf_chain,  SX1302_REG_TX_TOP_A_TX_RFFE_IF_Q_OFFSET_Q_OFFSET,
                                SX1302_REG_TX_TOP_B_TX_RFFE_IF_Q_OFFSET_Q_OFFSET);
    lgw_reg_w(reg, (int8_t)offset_q_set[0]);

    lgw_reg_w(SX1302_REG_RADIO_FE_SIG_ANA_CFG_START, 0);
    lgw_reg_w(SX1302_REG_RADIO_FE_SIG_ANA_CFG_START, 1);

    do {
        lgw_reg_r(SX1302_REG_RADIO_FE_SIG_ANA_CFG_VALID, &val);
        wait_ms(1);
    } while (val == 0);

    lgw_reg_r(SX1302_REG_RADIO_FE_SIG_ANA_ABS_LSB_CORR_ABS_OUT, &abs_lsb);
    lgw_reg_r(SX1302_REG_RADIO_FE_SIG_ANA_ABS_MSB_CORR_ABS_OUT, &abs_msb);

    abs_corr_min_i16 = abs_msb * 256 + abs_lsb;

    //8 points around
    for (j = 1; j < 9; j++) {
        reg = REG_SELECT(rf_chain,  SX1302_REG_TX_TOP_A_TX_RFFE_IF_I_OFFSET_I_OFFSET,
                                    SX1302_REG_TX_TOP_B_TX_RFFE_IF_I_OFFSET_I_OFFSET);
        lgw_reg_w(reg, (int8_t)offset_i_set[j]);

        reg = REG_SELECT(rf_chain,  SX1302_REG_TX_TOP_A_TX_RFFE_IF_Q_OFFSET_Q_OFFSET,
                                    SX1302_REG_TX_TOP_B_TX_RFFE_IF_Q_OFFSET_Q_OFFSET);
        lgw_reg_w(reg, (int8_t)offset_q_set[j]);

        lgw_reg_w(SX1302_REG_RADIO_FE_SIG_ANA_CFG_START, 0);
        lgw_reg_w(SX1302_REG_RADIO_FE_SIG_ANA_CFG_START, 1);

        do {
            lgw_reg_r(SX1302_REG_RADIO_FE_SIG_ANA_CFG_VALID, &val);
            wait_ms(1);
        } while (val == 0);

        lgw_reg_r(SX1302_REG_RADIO_FE_SIG_ANA_ABS_LSB_CORR_ABS_OUT, &abs_lsb);
        lgw_reg_r(SX1302_REG_RADIO_FE_SIG_ANA_ABS_MSB_CORR_ABS_OUT, &abs_msb);

        abs_corr_i16 = abs_msb * 256 + abs_lsb;
        if (abs_corr_i16 < abs_corr_min_i16) {
            abs_corr_min_i16 = abs_corr_i16;
            idx = j;
        }
    }
    offset_i = offset_i_set[idx];
    offset_q = offset_q_set[idx];

    tx_dc_i16 = abs_corr_min_i16;
    printf("tx_dc:%d\n", tx_dc_i16);

    // Return results of calibration
    *rej = 20 * log10(tx_sig_i16/(tx_dc_i16 + 1));
    *offset_i_res = (int8_t)offset_i;
    *offset_q_res = (int8_t)offset_q;
    printf("offset_i:%d offset_q:%d rej:%u\n", offset_i, offset_q, *rej);
}

#endif /* TX_CALIB_DONE_BY_HAL */

/* --- EOF ------------------------------------------------------------------ */
