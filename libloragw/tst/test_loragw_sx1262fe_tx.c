/*
 / _____)             _              | |
( (____  _____ ____ _| |_ _____  ____| |__
 \____ \| ___ |    (_   _) ___ |/ ___)  _ \
 _____) ) ____| | | || |_| ____( (___| | | |
(______/|_____)_|_|_| \__)_____)\____)_| |_|
  (C)2018 Semtech

Description:
    Minimum test program for the loragw_spi 'library'

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
#include "loragw_sx1262fe.h"
#include "loragw_aux.h"

/* -------------------------------------------------------------------------- */
/* --- PRIVATE MACROS ------------------------------------------------------- */

#define SX1262FE_FREQ_TO_REG(f)     (uint32_t)((uint64_t)f * (1 << 25) / 32000000U)
#define SX1302_FREQ_TO_REG(f)       (uint32_t)((uint64_t)f * (1 << 18) / 32000000U)

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

int sx1262fe_init(void) {
    uint8_t buff[16];

    lgw_reg_w(SX1302_REG_COMMON_CTRL0_SX1261_MODE_RADIO_A, 0x01);

    /* Enable Sx1262 and perform chip reset */
    lgw_reg_w(SX1302_REG_AGC_MCU_RF_EN_A_RADIO_EN, 0x01);
    lgw_reg_w(SX1302_REG_AGC_MCU_RF_EN_A_RADIO_RST, 0x01);
    wait_ms(500);
    lgw_reg_w(SX1302_REG_AGC_MCU_RF_EN_A_RADIO_RST, 0x00);
    wait_ms(10);
    lgw_reg_w(SX1302_REG_AGC_MCU_RF_EN_A_RADIO_RST, 0x01);

    /* Enable 32 MHz oscillator */
    buff[0] = (uint8_t)STDBY_XOSC;
    sx1262fe_write_command(SET_STANDBY, buff, 1);
    buff[0] = 0x00;
    sx1262fe_read_command(GET_STATUS, buff, 1);
    printf("%s: get_status: 0x%02X\n", __FUNCTION__, buff[0]);

    /* Configure DIO for Rx (not necessary here, just for reference) */
    buff[0] = 0x05;
    buff[1] = 0x82;
    buff[2] = 0x00;
    sx1262fe_write_command(WRITE_REGISTER, buff, 3); /* Drive strength to min */
    buff[0] = 0x05;
    buff[1] = 0x83;
    buff[2] = 0x00;
    sx1262fe_write_command(WRITE_REGISTER, buff, 3); /* Input enable, all disabled */
    buff[0] = 0x05;
    buff[1] = 0x84;
    buff[2] = 0x00;
    sx1262fe_write_command(WRITE_REGISTER, buff, 3); /* No pull up */
    buff[0] = 0x05;
    buff[1] = 0x85;
    buff[2] = 0x00;
    sx1262fe_write_command(WRITE_REGISTER, buff, 3); /* No pull down */
    buff[0] = 0x05;
    buff[1] = 0x80;
    buff[2] = 0x00;
    sx1262fe_write_command(WRITE_REGISTER, buff, 3); /* Output enable, all enabled */
    buff[0] = 0x05;
    buff[1] = 0x87;
    buff[2] = 0x08;
    sx1262fe_write_command(WRITE_REGISTER, buff, 3); /* FPGA_MODE_RX */

    lgw_reg_w(SX1302_REG_CLK_CTRL_CLK_SEL_CLKDIV_EN, 0x01); /* Mandatory */

    return 0;
}

int sx1262fe_set_idle(void) {
    uint8_t buff[16];
    int32_t val;

    buff[0] = (uint8_t)STDBY_XOSC;
    sx1262fe_write_command(SET_STANDBY, buff, 1);

    lgw_reg_w(SX1302_REG_TX_TOP_A_TX_TRIG_TX_TRIG_IMMEDIATE, 0x00);

    lgw_reg_r(SX1302_REG_TX_TOP_A_TX_STATUS_TX_STATUS, &val);
    printf("Tx status: 0x%02X\n", val);

    buff[0] = 0x05;
    buff[1] = 0x87;
    buff[2] = 0x0E;
    sx1262fe_write_command(WRITE_REGISTER, buff, 3); /* Default value */

    return 0;
}

int sx1262fe_set_tx_continuous(uint32_t freq_hz, uint8_t modulation, uint8_t sf, uint32_t bw, uint32_t tx_duration) {
    uint32_t freq_reg;
    uint8_t buff[16];
    uint32_t preamble_symb_nb;

    uint32_t freq_dev = bw/2;

    /* Switch SX1302 clock from SPI clock to SX1262 clock */
    lgw_reg_w(SX1302_REG_CLK_CTRL_CLK_SEL_CLK_RADIO_A_SEL, 0x01);
    lgw_reg_w(SX1302_REG_CLK_CTRL_CLK_SEL_CLK_RADIO_B_SEL, 0x00);

    /* Set frequency */
    freq_reg = SX1262FE_FREQ_TO_REG(freq_hz);
    buff[0] = (uint8_t)(freq_reg >> 24);
    buff[1] = (uint8_t)(freq_reg >> 16);
    buff[2] = (uint8_t)(freq_reg >> 8);
    buff[3] = (uint8_t)(freq_reg >> 0);
    sx1262fe_write_command(SET_RF_FREQUENCY, buff, 4);

    /* RF frequency, only for Sx126x: Tx RFFE interface frequency (TxRffeIfFreqRfH/M/L) */
    freq_reg = SX1302_FREQ_TO_REG(freq_hz);
    lgw_reg_w(SX1302_REG_TX_TOP_A_TX_RFFE_IF_FREQ_RF_H_FREQ_RF, (freq_reg >> 16) & 0xFF);
    lgw_reg_w(SX1302_REG_TX_TOP_A_TX_RFFE_IF_FREQ_RF_M_FREQ_RF, (freq_reg >>  8) & 0xFF);
    lgw_reg_w(SX1302_REG_TX_TOP_A_TX_RFFE_IF_FREQ_RF_L_FREQ_RF, (freq_reg >>  0) & 0xFF);

    /* SX126x in FS mode from Rx mode */
    sx1262fe_write_command(SET_FS, buff, 0);
    buff[0] = 0x00;
    sx1262fe_read_command(GET_STATUS, buff, 1);
    printf("%s: get_status: 0x%02X\n", __FUNCTION__, buff[0]);

    /* Configure DIO for Rx */
    buff[0] = 0x05;
    buff[1] = 0x82;
    buff[2] = 0x00;
    sx1262fe_write_command(WRITE_REGISTER, buff, 3); /* Drive strength to min */
    buff[0] = 0x05;
    buff[1] = 0x83;
    buff[2] = 0x3E;
    sx1262fe_write_command(WRITE_REGISTER, buff, 3); /* Input enable, all enabled except clk_32m */
    buff[0] = 0x05;
    buff[1] = 0x84;
    buff[2] = 0x00;
    sx1262fe_write_command(WRITE_REGISTER, buff, 3); /* No pull up */
    buff[0] = 0x05;
    buff[1] = 0x85;
    buff[2] = 0x00;
    sx1262fe_write_command(WRITE_REGISTER, buff, 3); /* No pull down */
    buff[0] = 0x05;
    buff[1] = 0x80;
    buff[2] = 0x3E;
    sx1262fe_write_command(WRITE_REGISTER, buff, 3); /* Output enable, all disabled except clk_32m */

    /* Configure SX1262FE for Tx */
    buff[0] = 0x0;
    buff[1] = (uint8_t)SET_RAMP_40U;
    sx1262fe_write_command(SET_TX_PARAMS, buff, 2); /* SetTxParams (power, RAMP_40U) */
    buff[0] = 0x04;
    buff[1] = 0x07;
    buff[2] = 0x00;
    buff[3] = 0x01;
    sx1262fe_write_command(SET_PA_CONFIG, buff, 4); /* SetPaConfig - high power PA - +22dBm */

    /* Tx RFFE interface control (TxRffeIfCtrl) */
    lgw_reg_w(SX1302_REG_TX_TOP_A_TX_RFFE_IF_CTRL_PLL_DIV_CTRL, 0x00); /* VCO divider by 2 */
    lgw_reg_w(SX1302_REG_TX_TOP_A_TX_RFFE_IF_CTRL_TX_CLK_EDGE, 0x01); /* 1:Data on falling edge */
    lgw_reg_w(SX1302_REG_TX_TOP_A_TX_RFFE_IF_CTRL_TX_MODE, 0x01); /* 0: Freq. synthesis, 1: Modulation */
    lgw_reg_w(SX1302_REG_TX_TOP_A_TX_RFFE_IF_CTRL_TX_IF_DST, 0x01); /* 1: SX126x Tx RFFE */

    /* Configure modulation */
    switch(modulation) {
        case MOD_CW:
            printf("Modulation CW\n");
            lgw_reg_w(SX1302_REG_TX_TOP_A_TX_RFFE_IF_CTRL_TX_IF_SRC, 0x00); /* 0: Signal, 1: LoRa, 2: GFSK */
            lgw_reg_w(SX1302_REG_TX_TOP_A_TX_RFFE_IF_TEST_MOD_FREQ, 0x00);
            break;
        case MOD_LORA:
            printf("Modulation LoRa\n");
            lgw_reg_w(SX1302_REG_TX_TOP_A_TX_RFFE_IF_CTRL_TX_IF_SRC, 0x01); /* 0: Signal, 1: LoRa, 2: GFSK */
            lgw_reg_w(SX1302_REG_TX_TOP_A_GEN_CFG_0_MODULATION_TYPE, 0x00); /* 0: LoRa, 1: GFSK */
            /* Preamble length */
            preamble_symb_nb = tx_duration / ((pow(2, sf) / bw));
            lgw_reg_w(SX1302_REG_TX_TOP_A_TXRX_CFG1_3_PREAMBLE_SYMB_NB, (preamble_symb_nb >> 8) & 0xFF); /* MSB */
            lgw_reg_w(SX1302_REG_TX_TOP_A_TXRX_CFG1_2_PREAMBLE_SYMB_NB, (preamble_symb_nb >> 0) & 0xFF); /* MSB */
            /* LoRa modem parameters */
            switch(bw) {
                case 125000:
                    printf("bandwidth 125khz\n");
                    lgw_reg_w(SX1302_REG_TX_TOP_A_TXRX_CFG0_0_MODEM_BW, 0x04); /* 4: 125khz, 5: 250khz, 6: 500khz */
                    break;
                case 250000:
                    printf("bandwidth 250khz\n");
                    lgw_reg_w(SX1302_REG_TX_TOP_A_TXRX_CFG0_0_MODEM_BW, 0x05); /* 4: 125khz, 5: 250khz, 6: 500khz */
                    break;
                case 500000:
                    printf("bandwidth 500khz\n");
                    lgw_reg_w(SX1302_REG_TX_TOP_A_TXRX_CFG0_0_MODEM_BW, 0x06); /* 4: 125khz, 5: 250khz, 6: 500khz */
                    break;
                default:
                    printf("ERROR: bandwidth %d not supported\n", bw);
                    break;
            }
            printf("Datarate SF%u\n", sf);
            lgw_reg_w(SX1302_REG_TX_TOP_A_TXRX_CFG0_0_MODEM_SF, sf);
            lgw_reg_w(SX1302_REG_TX_TOP_A_TX_CFG0_0_CHIRP_LOWPASS, 7);

            lgw_reg_w(SX1302_REG_TX_TOP_A_TXRX_CFG0_2_MODEM_EN, 1);
            lgw_reg_w(SX1302_REG_TX_TOP_A_TXRX_CFG0_2_CADRXTX, 2);

            lgw_reg_w(SX1302_REG_TX_TOP_A_TXRX_CFG1_1_MODEM_START, 1);
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
    lgw_reg_w(SX1302_REG_TX_TOP_A_TX_RFFE_IF_FREQ_DEV_H_FREQ_DEV, (freq_reg >>  8) & 0xFF);
    lgw_reg_w(SX1302_REG_TX_TOP_A_TX_RFFE_IF_FREQ_DEV_L_FREQ_DEV, (freq_reg >>  0) & 0xFF);

    buff[0] = 0x05;
    buff[1] = 0x87;
    buff[2] = 0x09;
    sx1262fe_write_command(WRITE_REGISTER, buff, 3); /* FPGA_MODE_TX */

    printf("Start Tx\n");
    buff[0] = 0x00;
    sx1262fe_write_command(SET_TXCONTINUOUSWAVE, buff, 0); /* SetTxContinuousWave */
    lgw_reg_w(SX1302_REG_TX_TOP_A_TX_TRIG_TX_TRIG_IMMEDIATE, 0x01);

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
    int i;
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

    printf("===== sx1302 sx1262fe TX test =====\n");
    lgw_connect();

    sx1262fe_init();
    sx1262fe_set_tx_continuous(ft, MOD_LORA, sf, bw, tx_duration);

    printf("Press enter to stop Tx\n");
    getchar();

    printf("Stop Tx\n");
    sx1262fe_set_idle();

    lgw_disconnect();
    printf("=========== Test End ===========\n");

    return 0;
}

/* --- EOF ------------------------------------------------------------------ */
