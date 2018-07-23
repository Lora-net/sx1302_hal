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

#include "loragw_hal.h"
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
#define DEFAULT_NB_PKT      1U

/* -------------------------------------------------------------------------- */
/* --- PRIVATE VARIABLES ---------------------------------------------------- */

/* -------------------------------------------------------------------------- */
/* --- PRIVATE FUNCTIONS ---------------------------------------------------- */

int sx1262fe_send_pkt(uint32_t freq_hz, uint8_t sf, uint32_t bw, uint16_t fcnt) {
    uint8_t buff[16];
    uint32_t freq_reg;
    int32_t val;
    int i;
    uint32_t preamble_symb_nb;
    uint32_t freq_dev = bw/2;
    uint16_t tx_start_delay;
    uint8_t payload[255];
    uint8_t payload_len;

    /* give radio control to HOST */
    lgw_reg_w(SX1302_REG_COMMON_CTRL0_HOST_RADIO_CTRL, 0x01);

    /* Configure radio */
    buff[0] = 0x08;
    buff[1] = 0xE6;
    buff[2] = 0x1C;
    sx1262fe_write_command(WRITE_REGISTER, buff, 3); /* ?? */

    buff[0] = 0x01; /* LoRa */
    sx1262fe_write_command(SET_PACKET_TYPE, buff, 1);

    freq_reg = SX1262FE_FREQ_TO_REG(freq_hz);
    buff[0] = (uint8_t)(freq_reg >> 24);
    buff[1] = (uint8_t)(freq_reg >> 16);
    buff[2] = (uint8_t)(freq_reg >> 8);
    buff[3] = (uint8_t)(freq_reg >> 0);
    sx1262fe_write_command(SET_RF_FREQUENCY, buff, 4);

    buff[0] = 0x0E; /* power */
    buff[1] = 0x02; /* RAMP_40U */
    sx1262fe_write_command(SET_TX_PARAMS, buff, 2);

    buff[0] = 0x04; /* paDutyCycle */
    buff[1] = 0x07; /* hpMax */
    buff[2] = 0x00; /* deviceSel */
    buff[3] = 0x01; /* paLut */
    sx1262fe_write_command(SET_PA_CONFIG, buff, 4); /* SX1262 Output Power +22dBm */

    /* give radio control to AGC MCU */
    lgw_reg_w(SX1302_REG_COMMON_CTRL0_HOST_RADIO_CTRL, 0x00);

    lgw_reg_w(SX1302_REG_TX_TOP_A_TX_RFFE_IF_CTRL_PLL_DIV_CTRL, 0x00); /* VCO divider by 2 */
    lgw_reg_w(SX1302_REG_TX_TOP_A_TX_RFFE_IF_CTRL_TX_IF_SRC, 0x01); /* LoRa */
    lgw_reg_w(SX1302_REG_TX_TOP_A_TX_RFFE_IF_CTRL_TX_IF_DST, 0x01); /* SX126x Tx RFFE */
    lgw_reg_w(SX1302_REG_TX_TOP_A_TX_RFFE_IF_CTRL_TX_MODE, 0x01); /* Modulation */
    lgw_reg_w(SX1302_REG_TX_TOP_A_TX_RFFE_IF_CTRL_TX_CLK_EDGE, 0x01); /* Data on falling edge */
    lgw_reg_w(SX1302_REG_TX_TOP_A_GEN_CFG_0_MODULATION_TYPE, 0x00); /* LoRa */

    /* Set Tx frequency */
    freq_reg = SX1302_FREQ_TO_REG(freq_hz);
    lgw_reg_w(SX1302_REG_TX_TOP_A_TX_RFFE_IF_FREQ_RF_H_FREQ_RF, (freq_reg >> 16) & 0xFF);
    lgw_reg_w(SX1302_REG_TX_TOP_A_TX_RFFE_IF_FREQ_RF_M_FREQ_RF, (freq_reg >> 8) & 0xFF);
    lgw_reg_w(SX1302_REG_TX_TOP_A_TX_RFFE_IF_FREQ_RF_L_FREQ_RF, (freq_reg >> 0) & 0xFF);

    /* Set bandwidth */
    freq_reg = SX1302_FREQ_TO_REG(freq_dev);
    lgw_reg_w(SX1302_REG_TX_TOP_A_TX_RFFE_IF_FREQ_DEV_H_FREQ_DEV, (freq_reg >>  8) & 0xFF);
    lgw_reg_w(SX1302_REG_TX_TOP_A_TX_RFFE_IF_FREQ_DEV_L_FREQ_DEV, (freq_reg >>  0) & 0xFF);

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
    /* Preamble length */
    preamble_symb_nb = 8;
    lgw_reg_w(SX1302_REG_TX_TOP_A_TXRX_CFG1_3_PREAMBLE_SYMB_NB, (preamble_symb_nb >> 8) & 0xFF); /* MSB */
    lgw_reg_w(SX1302_REG_TX_TOP_A_TXRX_CFG1_2_PREAMBLE_SYMB_NB, (preamble_symb_nb >> 0) & 0xFF); /* LSB */

    /* LoRa datarate */
    lgw_reg_w(SX1302_REG_TX_TOP_A_TXRX_CFG0_0_MODEM_SF, sf);
    lgw_reg_w(SX1302_REG_TX_TOP_A_TX_CFG0_0_CHIRP_LOWPASS, 7);

    /* Start LoRa modem */
    lgw_reg_w(SX1302_REG_TX_TOP_A_TXRX_CFG0_2_MODEM_EN, 1);
    lgw_reg_w(SX1302_REG_TX_TOP_A_TXRX_CFG0_2_CADRXTX, 2);
    lgw_reg_w(SX1302_REG_TX_TOP_A_TXRX_CFG1_1_MODEM_START, 1);
    lgw_reg_w(SX1302_REG_TX_TOP_A_TX_CFG0_0_CONTINUOUS, 0);

#if 0 /* TODO: how to do continuous lora modulation ?*/
    lgw_reg_w(SX1302_REG_TX_TOP_A_TX_CFG0_0_CONTINUOUS, 1);
    lgw_reg_w(SX1302_REG_TX_TOP_B_TX_CFG1_0_FRAME_NB, 2);
#endif

    /* Uplink configuration */
    lgw_reg_w(SX1302_REG_TX_TOP_A_TX_CFG0_0_CHIRP_INVERT, 0); /* non-inverted */
    lgw_reg_w(SX1302_REG_TX_TOP_A_TXRX_CFG0_2_IMPLICIT_HEADER, 0); /*  */
    lgw_reg_w(SX1302_REG_TX_TOP_A_TXRX_CFG0_2_CRC_EN, 1); /*  */
    lgw_reg_w(SX1302_REG_TX_TOP_A_FRAME_SYNCH_0_PEAK1_POS, 3); /*  */
    lgw_reg_w(SX1302_REG_TX_TOP_A_FRAME_SYNCH_1_PEAK2_POS, 4); /*  */
    
    /* Set TX start delay */
    tx_start_delay = 1000 * 32; /* us */ /* TODO: which value should we put?? */
    lgw_reg_w(SX1302_REG_TX_TOP_A_TX_START_DELAY_MSB_TX_START_DELAY, (uint8_t)(tx_start_delay >> 8));
    lgw_reg_w(SX1302_REG_TX_TOP_A_TX_START_DELAY_LSB_TX_START_DELAY, (uint8_t)(tx_start_delay >> 0));

    /* Set Payload length */
    payload_len = (uint8_t)(rand() % (255 + 1 - 12) + 12);
    lgw_reg_w(SX1302_REG_TX_TOP_A_TXRX_CFG0_3_PAYLOAD_LENGTH, payload_len);

    /* Write payload in transmit buffer */
    payload[0] = 0x40; /* Confirmed Data Up */
    payload[1] = 0xAB;
    payload[2] = 0xAB;
    payload[3] = 0xAB;
    payload[4] = 0xAB;
    payload[5] = 0x00; /* FCTrl */
    payload[6] = (uint8_t)(fcnt >> 0); /* FCnt */
    payload[7] = (uint8_t)(fcnt >> 8); /* FCnt */
    payload[8] = 0x02; /* FPort */
    for (i = 9; i < 255; i++) {
        payload[i] = i;
    }
    lgw_reg_w(SX1302_REG_TX_TOP_A_TX_CTRL_WRITE_BUFFER, 0x01);
    lgw_mem_wb(0x5300, &payload[0], payload_len);
    lgw_reg_w(SX1302_REG_TX_TOP_A_TX_CTRL_WRITE_BUFFER, 0x00);

    printf("Start Tx: Freq:%u SF%u size:%u\n", freq_hz, sf, payload_len);
    lgw_reg_w(SX1302_REG_TX_TOP_A_TX_TRIG_TX_TRIG_IMMEDIATE, 0x01);

    do {
        //lgw_reg_r(SX1302_REG_TX_TOP_A_LORA_TX_STATE_STATUS, &val);
        //lgw_reg_r(SX1302_REG_TX_TOP_A_LORA_TX_FLAG_FRAME_DONE, &val);
        //lgw_reg_r(SX1302_REG_TX_TOP_B_LORA_TX_FLAG_CONT_DONE, &val);
        //printf("cont done 0x%02X\n", val);
        lgw_reg_r(SX1302_REG_TX_TOP_A_TX_STATUS_TX_STATUS, &val);
        wait_ms(10);
    } while (val != 0x80);

#if 1
    /* TODO: should be done by AGC fw? */
    printf("Stop Tx\n");
    lgw_reg_w(SX1302_REG_TX_TOP_A_TX_TRIG_TX_TRIG_IMMEDIATE, 0x00);
#endif

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
    printf( " -x <uint> Number of packets to be sent\n");
}

int main(int argc, char **argv)
{
    int i, x;
    uint32_t ft = DEFAULT_FREQ_HZ;
    uint8_t sf = DEFAULT_SF;
    uint32_t bw = DEFAULT_BW_HZ;
    uint32_t nb_pkt = DEFAULT_NB_PKT;
    double arg_d = 0.0;
    unsigned int arg_u;

    struct lgw_conf_rxrf_s rfconf;

    /* parse command line options */
    while ((i = getopt (argc, argv, "hf:s:b:n:")) != -1) {
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
            case 'n': /* <uint> Number of packets to be sent */
                i = sscanf(optarg, "%u", &arg_u);
                if (i != 1) {
                    printf("ERROR: argument parsing of -n argument. Use -h to print help\n");
                    return EXIT_FAILURE;
                } else {
                    nb_pkt = (uint32_t)arg_u;
                }
                break;
            default:
                printf("ERROR: argument parsing\n");
                usage();
                return -1;
        }
    }

    printf("===== sx1302 sx1262fe TX test =====\n");

    /* Board reset */
    system("./reset_lgw.sh start");

    /* Configure the gateway */
    memset( &rfconf, 0, sizeof rfconf);
    rfconf.enable = true;
    rfconf.freq_hz = 868500000;
    rfconf.type = LGW_RADIO_TYPE_SX1262FE;
    lgw_rxrf_setconf(0, rfconf);

    x = lgw_start();
    if (x != 0) {
        printf("ERROR: failed to start the gateway\n");
        return EXIT_FAILURE;
    }

    for (i = 0; i < (int)nb_pkt; i++) {
        sx1262fe_send_pkt(ft, sf, bw, (uint16_t)i);
    }

    x = lgw_stop();
    if (x != 0) {
        printf("ERROR: failed to stop the gateway\n");
        return EXIT_FAILURE;
    }
    printf("=========== Test End ===========\n");

    return 0;
}

/* --- EOF ------------------------------------------------------------------ */
