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
#include "loragw_aux.h"

/* -------------------------------------------------------------------------- */
/* --- PRIVATE MACROS ------------------------------------------------------- */

#define RAND_RANGE(min, max) (rand() % (max + 1 - min) + min)

/* -------------------------------------------------------------------------- */
/* --- PRIVATE CONSTANTS ---------------------------------------------------- */

#define DEFAULT_FREQ_HZ     868500000U
#define DEFAULT_NB_PKT      1U

/* -------------------------------------------------------------------------- */
/* --- PRIVATE VARIABLES ---------------------------------------------------- */

/* -------------------------------------------------------------------------- */
/* --- PRIVATE FUNCTIONS ---------------------------------------------------- */

/* -------------------------------------------------------------------------- */
/* --- MAIN FUNCTION -------------------------------------------------------- */

/* describe command line options */
void usage(void) {
    //printf("Library version information: %s\n", lgw_version_info());
    printf( "Available options:\n");
    printf( " -h print this help\n");
    printf( " -f <float> Radio TX frequency in MHz\n");
    printf( " -s <uint> LoRa datarate 0:random, [7..12]\n");
    printf( " -b <uint> LoRa bandwidth in khz 0:random, [125, 250, 500]\n");
    printf( " -n <uint> Number of packets to be sent\n");
    printf( " -z <uint> size of packets to be sent 0:random, [9..255]\n");
}

int main(int argc, char **argv)
{
    int i, x;
    uint32_t ft = DEFAULT_FREQ_HZ;
    uint8_t sf = 0;
    uint32_t bw = 0;
    uint32_t nb_pkt = DEFAULT_NB_PKT;
    uint8_t size = 0;
    double arg_d = 0.0;
    unsigned int arg_u;

    struct lgw_conf_board_s boardconf;
    struct lgw_conf_rxrf_s rfconf;
    struct lgw_pkt_tx_s pkt;
    uint8_t tx_status;

    /* parse command line options */
    while ((i = getopt (argc, argv, "hf:s:b:n:z:")) != -1) {
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
                    switch (arg_u) {
                        case 125:
                            bw = BW_125KHZ;
                            break;
                        case 250:
                            bw = BW_250KHZ;
                            break;
                        case 500:
                            bw = BW_500KHZ;
                            break;
                        default:
                            break;
                    }
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
            case 'z': /* <uint> packet size */
                i = sscanf(optarg, "%u", &arg_u);
                if ((i != 1) || (arg_u < 9) || (arg_u > 255)) {
                    printf("ERROR: argument parsing of -z argument. Use -h to print help\n");
                    return EXIT_FAILURE;
                } else {
                    size = (uint8_t)arg_u;
                }
                break;
            default:
                printf("ERROR: argument parsing\n");
                usage();
                return -1;
        }
    }

    printf("===== sx1302 HAL TX test =====\n");

    /* Board reset */
    system("./reset_lgw.sh start");

    /* Configure the gateway */
    memset( &boardconf, 0, sizeof boardconf);
    boardconf.lorawan_public = true;
    boardconf.clksrc = 0;

    memset( &rfconf, 0, sizeof rfconf);
    rfconf.enable = true;
    rfconf.freq_hz = 868500000;
    rfconf.type = LGW_RADIO_TYPE_SX1250;
    lgw_rxrf_setconf(0, rfconf);

    x = lgw_start();
    if (x != 0) {
        printf("ERROR: failed to start the gateway\n");
        return EXIT_FAILURE;
    }

    /* Send packets */
    memset(&pkt, 0, sizeof pkt);
    pkt.freq_hz = ft;
    pkt.tx_mode = IMMEDIATE;
    pkt.modulation = MOD_LORA;
    pkt.invert_pol = false;
    pkt.preamble = 8;
    pkt.no_crc = false;
    pkt.no_header = false;
    pkt.payload[0] = 0x40; /* Confirmed Data Up */
    pkt.payload[1] = 0xAB;
    pkt.payload[2] = 0xAB;
    pkt.payload[3] = 0xAB;
    pkt.payload[4] = 0xAB;
    pkt.payload[5] = 0x00; /* FCTrl */
    pkt.payload[6] = 0; /* FCnt */
    pkt.payload[7] = 0; /* FCnt */
    pkt.payload[8] = 0x02; /* FPort */
    for (i = 9; i < 255; i++) {
        pkt.payload[i] = i;
    }

    for (i = 0; i < (int)nb_pkt; i++) {
        pkt.datarate = (sf == 0) ? (uint8_t)RAND_RANGE(7, 12) : sf;
        pkt.size = (size == 0) ? (uint8_t)RAND_RANGE(9, 255) : size;
        pkt.bandwidth = (bw == 0) ? (uint8_t)RAND_RANGE(4, 6) : bw;
        pkt.payload[6] = (uint8_t)(i >> 0); /* FCnt */
        pkt.payload[7] = (uint8_t)(i >> 8); /* FCnt */
        x = lgw_send(pkt);
        if (x != 0) {
            printf("ERROR: failed to send packet\n");
            return EXIT_FAILURE;
        }
        /* wait for packet to finish sending */
        do {
            wait_ms(5);
            lgw_status(TX_STATUS, &tx_status); /* get TX status */
        } while (tx_status != TX_FREE);
        printf("TX done\n");
    }

    /* Stop the gateway */
    x = lgw_stop();
    if (x != 0) {
        printf("ERROR: failed to stop the gateway\n");
        return EXIT_FAILURE;
    }
    printf("=========== Test End ===========\n");

    return 0;
}

/* --- EOF ------------------------------------------------------------------ */
