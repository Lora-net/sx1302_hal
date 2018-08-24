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

#include "loragw_reg.h"
#include "loragw_spi.h"
#include "loragw_sx125x.h"
#include "loragw_sx1302.h"
#include "loragw_aux.h"

/* -------------------------------------------------------------------------- */
/* --- PRIVATE MACROS ------------------------------------------------------- */

#define SX1302_FREQ_TO_REG(f)       (uint32_t)((uint64_t)f * (1 << 18) / 32000000U)
#define SX125X_FREQ_TO_REG(f)       (uint32_t)((uint64_t)f * (1 << 19) / 32000000U)

/* -------------------------------------------------------------------------- */
/* --- PRIVATE CONSTANTS ---------------------------------------------------- */

#define BUFF_SIZE           1024
#define DEFAULT_FREQ_HZ     868500000U
#define DEFAULT_SF          7U
#define DEFAULT_BW_HZ       125000U
#define DEFAULT_TX_DURATION 60U

#define MOD_CW              0
#define MOD_LORA            1
#define MOD_GFSK            2

/* -------------------------------------------------------------------------- */
/* --- PRIVATE VARIABLES ---------------------------------------------------- */

/* -------------------------------------------------------------------------- */
/* --- PRIVATE FUNCTIONS ---------------------------------------------------- */

int sx125x_init(void) {
    uint8_t version;

    /* Enable and reset the radio */
    sx1302_radio_reset(1, SX1302_RADIO_TYPE_SX125X);

    /* Check radio version */
    version = sx125x_read(LGW_SPI_MUX_TARGET_RADIOB, 0x07);
    switch (version) {
        case 0x11:
            printf("sx1255 detected\n");
            break;
        case 0x21:
            printf("sx1257 detected\n");
            break;
        default:
            printf("ERROR: failed to detect radio version (0x%02X)\n", version);
            return -1;
    }

    /* Set radio mode */
    sx1302_radio_set_mode(1, SX1302_RADIO_TYPE_SX125X);

    /* Enable 32 MHz oscillator */
    sx125x_write(LGW_SPI_MUX_TARGET_RADIOB, 0x00, 0x01); /* MODE_REG_ADDR, REF_ENABLE */
    sx125x_write(LGW_SPI_MUX_TARGET_RADIOB, 0x10, 0x02); /* CK_SEL_REG_ADDR, CKOUT_ENABLE */

    lgw_reg_w(SX1302_REG_CLK_CTRL_CLK_SEL_CLKDIV_EN, 0x01); /* Mandatory */

    return 0;
}

int sx125x_set_idle(void) {
    sx125x_write(LGW_SPI_MUX_TARGET_RADIOB, 0x00, 0x01); /* MODE_REG_ADDR, STDBY_MODE */
    lgw_reg_w(SX1302_REG_TX_TOP_B_TX_TRIG_TX_TRIG_IMMEDIATE, 0x00);

    return 0;
}

int sx125x_set_tx_continuous(uint32_t freq_hz, uint8_t modulation, uint8_t sf, uint32_t bw, uint32_t tx_duration) {
    uint8_t status;
    uint32_t freq_reg;
    uint32_t preamble_symb_nb;

    uint32_t freq_dev = bw/2;

    /* Switch SX1302 clock from SPI clock to SX125x clock */
    lgw_reg_w(SX1302_REG_CLK_CTRL_CLK_SEL_CLK_RADIO_A_SEL, 0x00);
    lgw_reg_w(SX1302_REG_CLK_CTRL_CLK_SEL_CLK_RADIO_B_SEL, 0x01);

    /* Set frequency */
    freq_reg = SX125X_FREQ_TO_REG(freq_hz);
    sx125x_write(LGW_SPI_MUX_TARGET_RADIOB, 0x04, (freq_reg >> 16) & 0xFF); /* FRFH_TX_REG_ADDR */
    sx125x_write(LGW_SPI_MUX_TARGET_RADIOB, 0x05, (freq_reg >>  8) & 0xFF); /* FRFM_TX_REG_ADDR */
    sx125x_write(LGW_SPI_MUX_TARGET_RADIOB, 0x06, (freq_reg >>  0) & 0xFF); /* FRFL_TX_REG_ADDR */

    /* Configure Tx output */
    sx125x_write(LGW_SPI_MUX_TARGET_RADIOB, 0x08, (2 << 4) | 10);       /* TXFE1_REG_ADDR */
    sx125x_write(LGW_SPI_MUX_TARGET_RADIOB, 0x09, (3 << 5) | (0 << 0)); /* TXFE2_REG_ADDR */
    sx125x_write(LGW_SPI_MUX_TARGET_RADIOB, 0x0A, 5);                   /* TXFE3_REG_ADDR */

    /* Tx RFFE interface control (TxRffeIfCtrl) */
    lgw_reg_w(SX1302_REG_TX_TOP_B_TX_RFFE_IF_CTRL_PLL_DIV_CTRL, 0x00); /* VCO divider by 2 */
    lgw_reg_w(SX1302_REG_TX_TOP_B_TX_RFFE_IF_CTRL_TX_CLK_EDGE, 0x00); /* 1:Data on rising edge */
    lgw_reg_w(SX1302_REG_TX_TOP_B_TX_RFFE_IF_CTRL_TX_MODE, 0x01); /* 0: Freq. synthesis, 1: Modulation */
    lgw_reg_w(SX1302_REG_TX_TOP_B_TX_RFFE_IF_CTRL_TX_IF_DST, 0x00); /* 0: SX1255/57 Tx RFFE */

    /* Configure modulation */
    switch(modulation) {
        case MOD_CW:
            printf("Modulation CW\n");
            lgw_reg_w(SX1302_REG_TX_TOP_B_TX_RFFE_IF_CTRL_TX_IF_SRC, 0x00); /* 0: Signal, 1: LoRa, 2: GFSK */
            lgw_reg_w(SX1302_REG_TX_TOP_B_TX_RFFE_IF_TEST_MOD_FREQ, 0x00);
            break;
        case MOD_LORA:
            printf("Modulation LoRa\n");
            lgw_reg_w(SX1302_REG_TX_TOP_B_TX_RFFE_IF_CTRL_TX_IF_SRC, 0x01); /* 0: Signal, 1: LoRa, 2: GFSK */
            lgw_reg_w(SX1302_REG_TX_TOP_B_GEN_CFG_0_MODULATION_TYPE, 0x00); /* 0: LoRa, 1: GFSK */
            /* Preamble length */
            preamble_symb_nb = tx_duration / ((pow(2, sf) / bw));
            lgw_reg_w(SX1302_REG_TX_TOP_B_TXRX_CFG1_3_PREAMBLE_SYMB_NB, (preamble_symb_nb >> 8) & 0xFF); /* MSB */
            lgw_reg_w(SX1302_REG_TX_TOP_B_TXRX_CFG1_2_PREAMBLE_SYMB_NB, (preamble_symb_nb >> 0) & 0xFF); /* MSB */
            /* LoRa modem parameters */
            switch(bw) {
                case 125000:
                    printf("bandwidth 125khz\n");
                    lgw_reg_w(SX1302_REG_TX_TOP_B_TXRX_CFG0_0_MODEM_BW, 0x04); /* 4: 125khz, 5: 250khz, 6: 500khz */
                    break;
                case 250000:
                    printf("bandwidth 250khz\n");
                    lgw_reg_w(SX1302_REG_TX_TOP_B_TXRX_CFG0_0_MODEM_BW, 0x05); /* 4: 125khz, 5: 250khz, 6: 500khz */
                    break;
                case 500000:
                    printf("bandwidth 500khz\n");
                    lgw_reg_w(SX1302_REG_TX_TOP_B_TXRX_CFG0_0_MODEM_BW, 0x06); /* 4: 125khz, 5: 250khz, 6: 500khz */
                    break;
                default:
                    printf("ERROR: bandwidth %d not supported\n", bw);
                    break;
            }
            printf("Datarate SF%u\n", sf);
            lgw_reg_w(SX1302_REG_TX_TOP_B_TXRX_CFG0_0_MODEM_SF, sf);
            lgw_reg_w(SX1302_REG_TX_TOP_B_TX_CFG0_0_CHIRP_LOWPASS, 7);

            lgw_reg_w(SX1302_REG_TX_TOP_B_TXRX_CFG0_2_MODEM_EN, 1);
            lgw_reg_w(SX1302_REG_TX_TOP_B_TXRX_CFG0_2_CADRXTX, 2);

            lgw_reg_w(SX1302_REG_TX_TOP_B_TXRX_CFG1_1_MODEM_START, 1);
            break;
        case MOD_GFSK:
            printf("Modulation GFSK\n");
            /* TODO */
            break;
        default:
            printf("ERROR: modulation %d not supported\n", modulation);
            break;
    }

    /* Set frequency deviation: Tx RFFE interface frequency deviation (TxRffeIfFreqDevH/L)*/
    freq_reg = SX1302_FREQ_TO_REG(freq_dev);
    printf("freq_reg(freq_dev)=%u (0x%02X)\n", freq_reg, freq_reg);
    lgw_reg_w(SX1302_REG_TX_TOP_B_TX_RFFE_IF_FREQ_DEV_H_FREQ_DEV, (freq_reg >>  8) & 0xFF);
    lgw_reg_w(SX1302_REG_TX_TOP_B_TX_RFFE_IF_FREQ_DEV_L_FREQ_DEV, (freq_reg >>  0) & 0xFF);

    lgw_reg_w(SX1302_REG_TX_TOP_B_TX_START_DELAY_MSB_TX_START_DELAY, (uint8_t)((1500 * 32) >> 8));
    lgw_reg_w(SX1302_REG_TX_TOP_B_TX_START_DELAY_LSB_TX_START_DELAY, (uint8_t)((1500 * 32) >> 0));

    printf("Start Tx\n");
    lgw_reg_w(SX1302_REG_TX_TOP_B_TX_TRIG_TX_TRIG_IMMEDIATE, 0x01);

    /* Enable PLL and PA output */
    sx125x_write(LGW_SPI_MUX_TARGET_RADIOB, 0x00, 0x05); /* MODE_REG_ADDR, TX_MODE */
    wait_ms(10);
    sx125x_write(LGW_SPI_MUX_TARGET_RADIOB, 0x00, 0x0D); /* MODE_REG_ADDR, TXPA_MODE */

    wait_ms(100);
    /* Read radio status */
    status = sx125x_read(LGW_SPI_MUX_TARGET_RADIOB, 0x11);
    printf("sx125x status: 0x%02X\n", status);

    return 0;
}

/* -------------------------------------------------------------------------- */
/* --- MAIN FUNCTION -------------------------------------------------------- */

/* describe command line options */
void usage(void) {
    //printf("Library version information: %s\n", lgw_version_info());
    printf( "Available options:\n");
    printf( " -h print this help\n");
    printf( " -f <float> Radio TX frequency in MHz\n");
    printf( " -s <uint> LoRa datarate [7..12]\n");
    printf( " -b <uint> LoRa bandwidth in khz [125, 250, 500]\n");
    printf( " -t <uint> TX duration in seconds\n");
}

int main(int argc, char **argv)
{
    int i, x;
    uint32_t ft = DEFAULT_FREQ_HZ;
    uint8_t sf = DEFAULT_SF;
    uint32_t bw = DEFAULT_BW_HZ;
    uint32_t tx_duration = DEFAULT_TX_DURATION;
    double arg_d = 0.0;
    unsigned int arg_u;

    /* parse command line options */
    while ((i = getopt (argc, argv, "hf:s:b:t:")) != -1) {
        switch (i) {
            case 'h':
                usage();
                return -1;
                break;
            case 'f': /* <float> Radio TX frequency in MHz */
                i = sscanf(optarg, "%lf", &arg_d);
                if (i != 1) {
                    printf("ERROR: argument parsing of -f argument. Use -h to print help\n");
                    return EXIT_FAILURE;
                } else {
                    ft = (uint32_t)((arg_d*1e6) + 0.5); /* .5 Hz offset to get rounding instead of truncating */
                }
                break;
            case 's': /* <uint> LoRa datarate */
                i = sscanf(optarg, "%u", &arg_u);
                if ((i != 1) || (arg_u < 7) || (arg_u > 12)) {
                    printf("ERROR: argument parsing of -s argument. Use -h to print help\n");
                    return EXIT_FAILURE;
                } else {
                    sf = (uint8_t)arg_u;
                }
                break;
            case 'b': /* <uint> LoRa bandwidth in khz */
                i = sscanf(optarg, "%u", &arg_u);
                if ((i != 1) || ((arg_u != 125) && (arg_u != 250) && (arg_u != 500))) {
                    printf("ERROR: argument parsing of -b argument. Use -h to print help\n");
                    return EXIT_FAILURE;
                } else {
                    bw = (uint32_t)(arg_u * 1E3);
                }
                break;
            case 't': /* <uint> TX duration in seconds */
                i = sscanf(optarg, "%u", &arg_u);
                if (i != 1) {
                    printf("ERROR: argument parsing of -t argument. Use -h to print help\n");
                    return EXIT_FAILURE;
                } else {
                    tx_duration = (uint32_t)arg_u;
                }
                break;
            default:
                printf("ERROR: argument parsing\n");
                usage();
                return -1;
        }
    }

    /* Board reset */
    system("./reset_lgw.sh start");

    printf("===== sx1302 sx125x TX test =====\n");
    lgw_connect();

    x = sx125x_init();
    if (x != 0) {
        return EXIT_FAILURE;;
    }

    sx125x_set_tx_continuous(ft, MOD_LORA, sf, bw, tx_duration);

    printf("Press enter to stop Tx\n");
    getchar();

    printf("Stop Tx\n");
    sx125x_set_idle();

    lgw_disconnect();
    printf("=========== Test End ===========\n");

    return 0;
}

/* --- EOF ------------------------------------------------------------------ */
