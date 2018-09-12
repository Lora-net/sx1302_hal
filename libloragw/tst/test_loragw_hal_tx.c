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
#include <signal.h>     /* sigaction */

#include "loragw_hal.h"
#include "loragw_reg.h"
#include "loragw_aux.h"

/* -------------------------------------------------------------------------- */
/* --- PRIVATE MACROS ------------------------------------------------------- */

#define RAND_RANGE(min, max) (rand() % (max + 1 - min) + min)

/* -------------------------------------------------------------------------- */
/* --- PRIVATE CONSTANTS ---------------------------------------------------- */

#define DEFAULT_CLK_SRC     0
#define DEFAULT_FREQ_HZ     868500000U

/* -------------------------------------------------------------------------- */
/* --- PRIVATE VARIABLES ---------------------------------------------------- */

/* Signal handling variables */
static int exit_sig = 0; /* 1 -> application terminates cleanly (shut down hardware, close open files, etc) */
static int quit_sig = 0; /* 1 -> application terminates without shutting down the hardware */

/* -------------------------------------------------------------------------- */
/* --- PRIVATE FUNCTIONS ---------------------------------------------------- */

/* describe command line options */
void usage(void) {
    //printf("Library version information: %s\n", lgw_version_info());
    printf( "Available options:\n");
    printf( " -h print this help\n");
    printf( " -k <uint> Concentrator clock source (Radio A or Radio B) [0..1]\n");
    printf( " -c <uint> RF chain to be used for TX (Radio A or Radio B) [0..1]\n");
    printf( " -r <uint> Radio type (1255, 1257, 1250)\n");
    printf( " -f <float> Radio TX frequency in MHz\n");
    printf( " -s <uint> LoRa datarate 0:random, [7..12]\n");
    printf( " -b <uint> LoRa bandwidth in khz 0:random, [125, 250, 500]\n");
    printf( " -n <uint> Number of packets to be sent\n");
    printf( " -z <uint> size of packets to be sent 0:random, [9..255]\n");
    printf( " -p <int>  RF power [0..15] -- TBD sx1250 --\n");
    printf( " -t <uint> TX mode timestamped with delay in ms\n");
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

/* -------------------------------------------------------------------------- */
/* --- MAIN FUNCTION -------------------------------------------------------- */

int main(int argc, char **argv)
{
    int i, x;
    uint32_t ft = DEFAULT_FREQ_HZ;
    int8_t rf_power = 0;
    uint8_t sf = 0;
    uint32_t bw = 0;
    uint32_t nb_pkt = 1;
    uint8_t size = 0;
    double arg_d = 0.0;
    unsigned int arg_u;
    int arg_i;
    uint8_t clocksource = 0;
    uint8_t rf_chain = 0;
    enum lgw_radio_type_e radio_type = LGW_RADIO_TYPE_NONE;
    uint16_t preamble = 8;

    struct lgw_conf_board_s boardconf;
    struct lgw_conf_rxrf_s rfconf;
    struct lgw_pkt_tx_s pkt;
    uint8_t tx_status;
    uint32_t count_us;
    uint32_t trig_delay_us = 1000000;
    bool trig_delay = false;

    static struct sigaction sigact; /* SIGQUIT&SIGINT&SIGTERM signal handling */

    /* parse command line options */
    while ((i = getopt (argc, argv, "hf:s:b:n:z:p:k:r:c:l:t:")) != -1) {
        switch (i) {
            case 'h':
                usage();
                return -1;
                break;
            case 'r': /* <uint> Radio type */
                i = sscanf(optarg, "%u", &arg_u);
                if ((i != 1) || ((arg_u != 1255) && (arg_u != 1257) && (arg_u != 1250))) {
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
                        default: /* 1250 */
                            radio_type = LGW_RADIO_TYPE_SX1250;
                            break;
                    }
                }
                break;
            case 'l': /* <uint> LoRa preamble length */
                i = sscanf(optarg, "%u", &arg_u);
                if ((i != 1) || (arg_u > 65535)) {
                    printf("ERROR: argument parsing of -l argument. Use -h to print help\n");
                    return EXIT_FAILURE;
                } else {
                    preamble = (uint16_t)arg_u;
                }
                break;
            case 't': /* <uint> Trigger delay in ms */
                i = sscanf(optarg, "%u", &arg_u);
                if (i != 1) {
                    printf("ERROR: argument parsing of -t argument. Use -h to print help\n");
                    return EXIT_FAILURE;
                } else {
                    trig_delay = true;
                    trig_delay_us = (uint32_t)(arg_u * 1E3);
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
            case 'p': /* <int> RF power */
                i = sscanf(optarg, "%d", &arg_i);
                if (i != 1) {
                    printf("ERROR: argument parsing of -p argument. Use -h to print help\n");
                    return EXIT_FAILURE;
                } else {
                    rf_power = (int8_t)arg_i;
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

    /* Configure signal handling */
    sigemptyset( &sigact.sa_mask );
    sigact.sa_flags = 0;
    sigact.sa_handler = sig_handler;
    sigaction( SIGQUIT, &sigact, NULL );
    sigaction( SIGINT, &sigact, NULL );
    sigaction( SIGTERM, &sigact, NULL );

    /* Board reset */
    system("./reset_lgw.sh start");

    /* Configure the gateway */
    memset( &boardconf, 0, sizeof boardconf);
    boardconf.lorawan_public = true;
    boardconf.clksrc = clocksource;
    lgw_board_setconf(boardconf);

    memset( &rfconf, 0, sizeof rfconf);
    rfconf.enable = ((rf_chain == 0) ? true : false);
    rfconf.freq_hz = 868500000; /* dummy */
    rfconf.type = radio_type;
    rfconf.tx_enable = true;
    lgw_rxrf_setconf(0, rfconf);

    memset( &rfconf, 0, sizeof rfconf);
    rfconf.enable = ((rf_chain == 1) ? true : false);
    rfconf.freq_hz = 868500000; /* dummy */
    rfconf.type = radio_type;
    rfconf.tx_enable = true;
    lgw_rxrf_setconf(1, rfconf);

    x = lgw_start();
    if (x != 0) {
        printf("ERROR: failed to start the gateway\n");
        return EXIT_FAILURE;
    }

    /* Send packets */
    memset(&pkt, 0, sizeof pkt);
    pkt.rf_chain = rf_chain;
    pkt.freq_hz = ft;
    pkt.rf_power = rf_power;
    if (trig_delay == false) {
        pkt.tx_mode = IMMEDIATE;
    } else {
        pkt.tx_mode = TIMESTAMPED;
    }
    pkt.modulation = MOD_LORA;
    pkt.invert_pol = false;
    pkt.preamble = preamble;
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
        if (trig_delay == true) {
            lgw_get_instcnt(&count_us);
            printf("count_us:%u\n", count_us);
            pkt.count_us = count_us + trig_delay_us;
            printf("programming TX for %u\n", pkt.count_us);
        }

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
            lgw_status(pkt.rf_chain, TX_STATUS, &tx_status); /* get TX status */
        } while ((tx_status != TX_FREE) && (quit_sig != 1) && (exit_sig != 1));

        if ((quit_sig == 1) || (exit_sig == 1)) {
            break;
        }
        printf("TX done\n");
    }

    /* Stop the gateway */
    x = lgw_stop();
    if (x != 0) {
        printf("ERROR: failed to stop the gateway\n");
        return EXIT_FAILURE;
    }

    /* Board reset */
    system("./reset_lgw.sh start");

    printf("=========== Test End ===========\n");

    return 0;
}

/* --- EOF ------------------------------------------------------------------ */
