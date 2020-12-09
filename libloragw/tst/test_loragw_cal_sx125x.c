/*
 / _____)             _              | |
( (____  _____ ____ _| |_ _____  ____| |__
 \____ \| ___ |    (_   _) ___ |/ ___)  _ \
 _____) ) ____| | | || |_| ____( (___| | | |
(______/|_____)_|_|_| \__)_____)\____)_| |_|
  (C)2019 Semtech

Description:
    Minimum test program for HAL calibration for sx1255/sx1257 radios

License: Revised BSD License, see LICENSE.TXT file include in the project
*/


/* -------------------------------------------------------------------------- */
/* --- DEPENDANCIES --------------------------------------------------------- */

/* fix an issue between POSIX and C99 */
#if __STDC_VERSION__ >= 199901L
    #define _XOPEN_SOURCE 600
#else
    #define _XOPEN_SOURCE 500
#endif

#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <math.h>
#include <signal.h>     /* sigaction */
#include <getopt.h>     /* getopt_long */
#include <sys/time.h>

#include "loragw_hal.h"
#include "loragw_reg.h"
#include "loragw_com.h"
#include "loragw_sx1302.h"
#include "loragw_sx125x.h"
#include "loragw_aux.h"
#include "loragw_cal.h"

/* -------------------------------------------------------------------------- */
/* --- PRIVATE MACROS ------------------------------------------------------- */

#define RAND_RANGE(min, max)        (rand() % (max + 1 - min) + min)

#define DEBUG_MSG(str)                fprintf(stdout, str)
#define DEBUG_PRINTF(fmt, args...)    fprintf(stdout,"%s:%d: "fmt, __FUNCTION__, __LINE__, args)

/* -------------------------------------------------------------------------- */
/* --- PRIVATE CONSTANTS ---------------------------------------------------- */

#define COM_TYPE_DEFAULT LGW_COM_SPI
#define COM_PATH_DEFAULT "/dev/spidev0.0"

#define DEFAULT_CLK_SRC     0
#define DEFAULT_FREQ_HZ     868500000U

#define DEFAULT_DAC_GAIN    3
#define DEFAULT_MIX_GAIN    15

#define CAL_TX_TONE_FREQ_HZ     250000
#define CAL_DEC_GAIN            8
#define CAL_SIG_ANA_DURATION    0 /* correlation duration: 0:1, 1:2, 2:4, 3:8 ms) */

#define TEST_FREQ_SCAN          0
#define TEST_OFFSET_IQ          1
#define TEST_AMP_PHI            2

/* -------------------------------------------------------------------------- */
/* --- PRIVATE TYPES -------------------------------------------------------- */
struct cal_tx_log {
    int32_t mean;
    int32_t i_offset;
    int32_t q_offset;
};

/* -------------------------------------------------------------------------- */
/* --- PRIVATE VARIABLES ---------------------------------------------------- */

FILE * fp;

static uint32_t rf_rx_freq[LGW_RF_CHAIN_NB] = {865500000, 865500000};
static lgw_radio_type_t rf_radio_type[LGW_RF_CHAIN_NB] = {LGW_RADIO_TYPE_SX1257, LGW_RADIO_TYPE_SX1257};
static struct lgw_tx_gain_lut_s txlut; /* TX gain table */

/* Signal handling variables */
static int exit_sig = 0; /* 1 -> application terminates cleanly (shut down hardware, close open files, etc) */
static int quit_sig = 0; /* 1 -> application terminates without shutting down the hardware */

#include "../src/cal_fw.var" /* text_cal_sx1257_16_Nov_1 */

/* -------------------------------------------------------------------------- */
/* --- PRIVATE FUNCTIONS ---------------------------------------------------- */

/* describe command line options */
void usage(void) {
    //printf("Library version information: %s\n", lgw_version_info());
    printf("Available options:\n");
    printf(" -h print this help\n");
    printf(" -u        Set COM type as USB (default is SPI)\n");
    printf(" -d [path] Path to the COM interface\n");
    printf("            => default path: " COM_PATH_DEFAULT "\n");
    printf(" -k <uint> Concentrator clock source (Radio A or Radio B) [0..1]\n");
    printf(" -c <uint> RF chain to be used for TX (Radio A or Radio B) [0..1]\n");
    printf(" -r <uint> Radio type (1255, 1257)\n");
    printf( "~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~\n" );
    printf(" --pa   <uint> PA gain [0..3]\n");
    printf(" --dig  <uint> sx1302 digital gain [0..3]\n");
    printf(" --dac  <uint> sx1257 DAC gain [0..3]\n");
    printf(" --mix  <uint> sx1257 MIX gain [0..15]\n");
}

/* handle signals */
static void sig_handler(int sigio)
{
    if (sigio == SIGQUIT) {
        quit_sig = 1;
    }
    else if((sigio == SIGINT) || (sigio == SIGTERM)) {
        exit_sig = 1;
    }
}

int setup_tx_dc_offset(uint8_t rf_chain, uint32_t freq_hz, uint8_t dac_gain, uint8_t mix_gain, uint8_t radio_type) {
    uint32_t rx_freq_hz, tx_freq_hz;
    uint32_t rx_freq_int, rx_freq_frac;
    uint32_t tx_freq_int, tx_freq_frac;
    uint8_t rx_pll_locked, tx_pll_locked;

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

    return 0;
}

int cal_tx_dc_offset(uint8_t test_id, uint8_t rf_chain, uint32_t freq_hz, uint8_t dac_gain, uint8_t mix_gain, uint8_t radio_type, int32_t f_offset, int32_t i_offset, int32_t q_offset, bool full_log, bool use_agc, uint8_t amp, uint8_t phi) {
    int i;
    uint16_t reg;
    int32_t val_min, val_max;
    int32_t acc;
    int32_t val_mean;
    float val_std;
    float acc2 = 0 ;
    int loop_len = 3;
    float res_sig[loop_len];
    struct timeval start, stop;

    //DEBUG_MSG("\n");
    //DEBUG_PRINTF("rf_chain:%u, freq_hz:%u, dac_gain:%u, mix_gain:%u, radio_type:%d\n", rf_chain, freq_hz, dac_gain, mix_gain, radio_type);

    if (setup_tx_dc_offset(rf_chain, freq_hz, dac_gain, mix_gain, radio_type) != LGW_HAL_SUCCESS) {
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

    /* Measuring */
    if (use_agc == true) {
        uint8_t val_sig, val_sig2;

        /* Set calibration parameters */
        sx1302_agc_mailbox_write(2, rf_chain + 4); /* Sig ana test radio A/B */
        sx1302_agc_mailbox_write(1, f_offset/*(CAL_TX_TONE_FREQ_HZ + f_offset) * 64e-6*/); /* Set frequency */
        sx1302_agc_mailbox_write(0, CAL_SIG_ANA_DURATION);

        /*  */
        sx1302_agc_mailbox_write(3, 0x00);
        sx1302_agc_mailbox_write(3, 0x01);
        sx1302_agc_wait_status(0x01);

        sx1302_agc_mailbox_write(2, amp); /* amp */
        sx1302_agc_mailbox_write(1, phi); /* phi */

        sx1302_agc_mailbox_write(3, 0x02);
        sx1302_agc_wait_status(0x02);

        sx1302_agc_mailbox_write(2, i_offset); /* i offset init */
        sx1302_agc_mailbox_write(1, q_offset); /* q offset init */

        sx1302_agc_mailbox_write(3, 0x03);
        sx1302_agc_wait_status(0x03);

        sx1302_agc_mailbox_write(2, CAL_DEC_GAIN); /* dec_gain */
        sx1302_agc_mailbox_write(2, 0); /* threshold (not used) */

        sx1302_agc_mailbox_write(3, 0x04);

        reg = REG_SELECT(rf_chain,  SX1302_REG_TX_TOP_A_TX_TRIG_TX_TRIG_IMMEDIATE,
                                    SX1302_REG_TX_TOP_B_TX_TRIG_TX_TRIG_IMMEDIATE);
        lgw_reg_w(reg, 0);

        gettimeofday (&start, NULL);
        for (i = 0; i < loop_len; i++) {
            sx1302_agc_wait_status(0x06);
            sx1302_agc_mailbox_write(3, 0x06);

            sx1302_agc_wait_status(0x07);
            sx1302_agc_mailbox_read(0, &val_sig);
            sx1302_agc_mailbox_read(1, &val_sig2);
            res_sig[i] = val_sig2 * 256 + val_sig;

            if (i == (loop_len - 1)) {
                sx1302_agc_mailbox_write(3, 0x07); /* unlock */
            } else {
                sx1302_agc_mailbox_write(3, 0x00); /* unlock */
            }
        }
        gettimeofday (&stop, NULL);
        //printf("processing time: %ld us\n", ((stop.tv_sec - start.tv_sec) * 1000000 + stop.tv_usec) - start.tv_usec);
    } else {
        int32_t val;
        int32_t abs_lsb, abs_msb;
        float abs_iq;

        reg = REG_SELECT(rf_chain,  SX1302_REG_TX_TOP_A_TX_RFFE_IF_Q_OFFSET_Q_OFFSET,
                                    SX1302_REG_TX_TOP_B_TX_RFFE_IF_Q_OFFSET_Q_OFFSET);
        lgw_reg_w(reg, (int8_t)q_offset);

        reg = REG_SELECT(rf_chain,  SX1302_REG_TX_TOP_A_TX_RFFE_IF_I_OFFSET_I_OFFSET,
                                    SX1302_REG_TX_TOP_B_TX_RFFE_IF_I_OFFSET_I_OFFSET);
        lgw_reg_w(reg, (int8_t)i_offset);

        reg = REG_SELECT(rf_chain,  SX1302_REG_RADIO_FE_CTRL0_RADIO_A_DC_NOTCH_EN,
                                    SX1302_REG_RADIO_FE_CTRL0_RADIO_B_DC_NOTCH_EN);
        lgw_reg_w(reg, 1);

        reg = REG_SELECT(rf_chain,  SX1302_REG_RADIO_FE_CTRL0_RADIO_A_FORCE_HOST_FILTER_GAIN,
                                    SX1302_REG_RADIO_FE_CTRL0_RADIO_B_FORCE_HOST_FILTER_GAIN);
        lgw_reg_w(reg, 0x01);

        reg = REG_SELECT(rf_chain,  SX1302_REG_RADIO_FE_CTRL0_RADIO_A_HOST_FILTER_GAIN,
                                    SX1302_REG_RADIO_FE_CTRL0_RADIO_B_HOST_FILTER_GAIN);
        lgw_reg_w(reg, CAL_DEC_GAIN);

        lgw_reg_w(SX1302_REG_RADIO_FE_SIG_ANA_CFG_FORCE_HAL_CTRL, 1);

        lgw_reg_w(SX1302_REG_RADIO_FE_SIG_ANA_FREQ_FREQ, f_offset);

        lgw_reg_w(SX1302_REG_RADIO_FE_SIG_ANA_CFG_DURATION, CAL_SIG_ANA_DURATION);
        lgw_reg_w(SX1302_REG_RADIO_FE_SIG_ANA_CFG_EN, 1);

        gettimeofday (&start, NULL);
        for (i = 0; i < loop_len; i++) {
            lgw_reg_w(SX1302_REG_RADIO_FE_SIG_ANA_CFG_START, 0);
            lgw_reg_w(SX1302_REG_RADIO_FE_SIG_ANA_CFG_START, 1);

            do {
                lgw_reg_r(SX1302_REG_RADIO_FE_SIG_ANA_CFG_VALID, &val);
                wait_ms(1);
            } while (val == 0);

            lgw_reg_r(SX1302_REG_RADIO_FE_SIG_ANA_ABS_LSB_CORR_ABS_OUT, &abs_lsb);
            lgw_reg_r(SX1302_REG_RADIO_FE_SIG_ANA_ABS_MSB_CORR_ABS_OUT, &abs_msb);
            abs_iq = (abs_msb << 8) | abs_lsb;

            res_sig[i] = abs_iq;
        }
        gettimeofday (&stop, NULL);
        //printf("processing time: %ld us\n", ((stop.tv_sec - start.tv_sec) * 1000000 + stop.tv_usec) - start.tv_usec);

        lgw_reg_w(SX1302_REG_RADIO_FE_SIG_ANA_CFG_FORCE_HAL_CTRL, 0);
    }

    if (full_log == true) {
        printf("i_offset:%d q_offset:%d f_offset:%d dac_gain:%d mix_gain:%d dec_gain:%d amp:%u phi:%u => ", i_offset, q_offset, f_offset, dac_gain, mix_gain, CAL_DEC_GAIN, amp, phi);
    } else {
        switch (test_id) {
            case TEST_FREQ_SCAN:
                fprintf(fp, "%u ", f_offset);
                break;
            case TEST_OFFSET_IQ:
                fprintf(fp, "%d %d ", i_offset, q_offset);
                break;
            case TEST_AMP_PHI:
                fprintf(fp, "%d %d ", amp, phi);
                break;
            default:
                printf("ERROR: wrong test ID (%u)\n", test_id);
                break;
        }
    }

    /* Analyze result */
    val_min = res_sig[0];
    val_max = res_sig[0];
    acc = 0;
    for (i = 0; i < loop_len; i++) {
        if (res_sig[i] > val_max) {
            val_max = res_sig[i];
        }
        if (res_sig[i] < val_min) {
            val_min = res_sig[i];
        }
        acc += res_sig[i];
    }
    val_mean = acc / loop_len;

    for (i = 0; i < loop_len; i++) {
        acc2 += pow((res_sig[i]-val_mean),2);
    }
    val_std = sqrt(acc2/loop_len);

    if (full_log == true) {
        printf(" min:%u max:%u mean:%u std:%f\n", val_min, val_max, val_mean, val_std);
    } else {
        switch (test_id) {
            case TEST_OFFSET_IQ:
            case TEST_AMP_PHI:
                fprintf(fp, "%u %u %u %f\n", val_min, val_max, val_mean, val_std);
                break;
            case TEST_FREQ_SCAN:
                fprintf(fp, "%u\n", val_mean);
                break;
            default:
                break;
        }
    }

    return LGW_HAL_SUCCESS;
}

/* -------------------------------------------------------------------------- */
/* --- MAIN FUNCTION -------------------------------------------------------- */

int test_freq_scan(uint8_t rf_chain, bool full_log, bool use_agc) {
    int f;

    printf("-------------------------------------\n");
    for (f = 0; f < 256; f++)
    {
        cal_tx_dc_offset(TEST_FREQ_SCAN, rf_chain, rf_rx_freq[rf_chain], txlut.lut[0].dac_gain, txlut.lut[0].mix_gain, rf_radio_type[rf_chain], f, 0, 0, full_log, use_agc, 0, 0);

        if ((quit_sig == 1) || (exit_sig == 1)) {
            break;
        }
    }

    return 0;
}

int test_iq_offset(uint8_t rf_chain, uint8_t f_offset, bool full_log, bool use_agc) {
    int i, q;

    printf("-------------------------------------\n");
    for (i = -128; i < 127; i+=8)
    {
        for (q = -128; q < 127; q+=8)
        {
            cal_tx_dc_offset(TEST_OFFSET_IQ, rf_chain, rf_rx_freq[rf_chain], txlut.lut[0].dac_gain, txlut.lut[0].mix_gain, rf_radio_type[rf_chain], f_offset, i, q, full_log, use_agc, 0, 0);
            if ((quit_sig == 1) || (exit_sig == 1)) {
                return 0;
            }
        }
    }

    return 0;
}

int test_amp_phi(uint8_t rf_chain, uint8_t f_offset, bool full_log, bool use_agc) {
    int amp, phi;

    printf("-------------------------------------\n");
    for (amp = 0; amp < 64; amp++)
    {
        for (phi = 0; phi < 64; phi++)
        {
            cal_tx_dc_offset(TEST_AMP_PHI, rf_chain, rf_rx_freq[rf_chain], txlut.lut[0].dac_gain, txlut.lut[0].mix_gain, rf_radio_type[rf_chain], f_offset, 0, 0, full_log, use_agc, amp, phi);
            if ((quit_sig == 1) || (exit_sig == 1)) {
                return 0;
            }
        }
    }

    return 0;
}

int test_capture_ram(uint8_t rf_chain) {
    uint16_t reg;

    setup_tx_dc_offset(rf_chain, rf_rx_freq[rf_chain], txlut.lut[0].dac_gain, txlut.lut[0].mix_gain, rf_radio_type[rf_chain]);

    reg = REG_SELECT(rf_chain,  SX1302_REG_RADIO_FE_CTRL0_RADIO_A_DC_NOTCH_EN,
                                SX1302_REG_RADIO_FE_CTRL0_RADIO_B_DC_NOTCH_EN);
    lgw_reg_w(reg, 1);

    printf("Waiting...\n");
    while ((quit_sig != 1) && (exit_sig != 1)) {
        wait_ms(1000);
    }

    return 0;
}

int main(int argc, char **argv)
{
    int i, x;
    unsigned int arg_u;
    uint8_t clocksource = 0;
    uint8_t rf_chain = 0;
    lgw_radio_type_t radio_type = LGW_RADIO_TYPE_SX1257;

    static struct sigaction sigact; /* SIGQUIT&SIGINT&SIGTERM signal handling */

    /* SPI interfaces */
    const char com_path_default[] = COM_PATH_DEFAULT;
    const char * com_path = com_path_default;
    lgw_com_type_t com_type = COM_TYPE_DEFAULT;

    /* Initialize TX gain LUT */
    txlut.size = 1;
    memset(txlut.lut, 0, sizeof txlut.lut);
    txlut.lut[0].dac_gain = DEFAULT_DAC_GAIN;
    txlut.lut[0].mix_gain = DEFAULT_MIX_GAIN;

    /* Parameter parsing */
    int option_index = 0;
    static struct option long_options[] = {
        {"dac", 1, 0, 0},
        {"mix", 1, 0, 0},
        {0, 0, 0, 0}
    };

    /* parse command line options */
    while ((i = getopt_long (argc, argv, "hk:r:c:d:u", long_options, &option_index)) != -1) {
        switch (i) {
            case 'h':
                usage();
                return -1;
                break;

            case 'u':
                com_type = LGW_COM_USB;
                break;

            case 'd':
                if (optarg != NULL) {
                    com_path = optarg;
                }
                break;

            case 'r': /* <uint> Radio type */
                i = sscanf(optarg, "%u", &arg_u);
                if ((i != 1) || ((arg_u != 1255) && (arg_u != 1257))) {
                    printf("ERROR: argument parsing of -r argument. Use -h to print help\n");
                    return EXIT_FAILURE;
                } else {
                    switch (arg_u) {
                        case 1255:
                            radio_type = LGW_RADIO_TYPE_SX1255;
                            break;
                        case 1257:
                            radio_type = LGW_RADIO_TYPE_SX1257;
                            break;
                        default:
                            /* should not happen */
                            break;
                    }
                }
                break;

            case 'k': /* <uint> Clock Source */
                i = sscanf(optarg, "%u", &arg_u);
                if ((i != 1) || (arg_u > 1)) {
                    printf("ERROR: argument parsing of -k argument. Use -h to print help\n");
                    return EXIT_FAILURE;
                } else {
                    clocksource = (uint8_t)arg_u;
                }
                break;

            case 'c': /* <uint> RF chain */
                i = sscanf(optarg, "%u", &arg_u);
                if ((i != 1) || (arg_u > 1)) {
                    printf("ERROR: argument parsing of -c argument. Use -h to print help\n");
                    return EXIT_FAILURE;
                } else {
                    rf_chain = (uint8_t)arg_u;
                }
                break;

            case 0:
                if (strcmp(long_options[option_index].name, "dac") == 0) {
                    i = sscanf(optarg, "%u", &arg_u);
                    if ((i != 1) || (arg_u > 3)) {
                        printf("ERROR: argument parsing of --dac argument. Use -h to print help\n");
                        return EXIT_FAILURE;
                    } else {
                        txlut.size = 1;
                        txlut.lut[0].dac_gain = (uint8_t)arg_u;
                    }
                } else if (strcmp(long_options[option_index].name, "mix") == 0) {
                    i = sscanf(optarg, "%u", &arg_u);
                    if ((i != 1) || (arg_u > 15)) {
                        printf("ERROR: argument parsing of --mix argument. Use -h to print help\n");
                        return EXIT_FAILURE;
                    } else {
                        txlut.size = 1;
                        txlut.lut[0].mix_gain = (uint8_t)arg_u;
                    }
                } else {
                    printf("ERROR: argument parsing options. Use -h to print help\n");
                    return EXIT_FAILURE;
                }
                break;

            default:
                printf("ERROR: argument parsing\n");
                usage();
                return -1;
        }
    }

    /* Configure signal handling */
    sigemptyset( &sigact.sa_mask );
    sigact.sa_flags = 0;
    sigact.sa_handler = sig_handler;
    sigaction( SIGQUIT, &sigact, NULL );
    sigaction( SIGINT, &sigact, NULL );
    sigaction( SIGTERM, &sigact, NULL );

    /* USB is currently not supported for sx1255/sx1257 radios */
    if (com_type == LGW_COM_USB) {
        printf("ERROR: USB interface is currently not supported for sx1255/sx1257 radios\n");
        exit(EXIT_FAILURE);
    }

    if (com_type == LGW_COM_SPI) {
        /* Board reset */
        if (system("./reset_lgw.sh start") != 0) {
            printf("ERROR: failed to reset SX1302, check your reset_lgw.sh script\n");
            exit(EXIT_FAILURE);
        }
    }

    /* open log file for writing */
    fp = fopen("log.txt", "w+");

    /* connect the gateway */
    x = lgw_connect(com_type, com_path);
    if (x != 0) {
        printf("ERROR: failed to connect the gateway\n");
        return EXIT_FAILURE;
    }

    sx1302_radio_reset(rf_chain, radio_type);
    sx1302_radio_clock_select(clocksource);
    sx1302_radio_set_mode(rf_chain, radio_type);

    printf("Loading CAL fw for sx125x\n");
    if (sx1302_agc_load_firmware(cal_firmware_sx125x) != LGW_HAL_SUCCESS) {
        return LGW_HAL_ERROR;
    }

    printf("waiting for capture ram\n");
    wait_ms(1000);

    /* testing */
    printf("testing: rf_chain:%u, dac_gain: %u, mix_gain:%u, dec_gain:%u, sig_ana_duration:%u\n", rf_chain, txlut.lut[0].dac_gain, txlut.lut[0].mix_gain, CAL_DEC_GAIN, CAL_SIG_ANA_DURATION);

    test_freq_scan(rf_chain, false, false); /* rf_chain, full_log, use_agc */
    /* gnuplot> plot 'log.txt' with lines */

    //test_iq_offset(rf_chain, 16, false, false); /* rf_chain, f_offset, full_log, use_agc */

    //test_amp_phi(rf_chain, 240, true, true); /* rf_chain, f_offset, full_log, use_agc */

    //test_capture_ram(rf_chain);

    sx1302_radio_reset(0, radio_type);
    sx1302_radio_reset(1, radio_type);

    /* disconnect the gateway */
    x = lgw_disconnect();
    if (x != 0) {
        printf("ERROR: failed to disconnect the gateway\n");
        return EXIT_FAILURE;
    }

    /* Close log file */
    fclose(fp);

    if (com_type == LGW_COM_SPI) {
        /* Board reset */
        if (system("./reset_lgw.sh stop") != 0) {
            printf("ERROR: failed to reset SX1302, check your reset_lgw.sh script\n");
            exit(EXIT_FAILURE);
        }
    }

    printf("=========== Test End ===========\n");

    return 0;
}

/* --- EOF ------------------------------------------------------------------ */
