/*
 / _____)             _              | |
( (____  _____ ____ _| |_ _____  ____| |__
 \____ \| ___ |    (_   _) ___ |/ ___)  _ \
 _____) ) ____| | | || |_| ____( (___| | | |
(______/|_____)_|_|_| \__)_____)\____)_| |_|
  (C)2019 Semtech

Description:
    Minimum test program for HAL RX capability

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
#include <signal.h>
#include <math.h>

#include "loragw_hal.h"
#include "loragw_reg.h"
#include "loragw_aux.h"

/* -------------------------------------------------------------------------- */
/* --- PRIVATE MACROS ------------------------------------------------------- */

#define LINUXDEV_PATH_DEFAULT "/dev/spidev0.0"

#define ARRAY_SIZE(a) (sizeof(a) / sizeof((a)[0]))
#define RAND_RANGE(min, max) (rand() % (max + 1 - min) + min)

/* -------------------------------------------------------------------------- */
/* --- PRIVATE CONSTANTS ---------------------------------------------------- */

#define DEFAULT_FREQ_HZ     868500000U

/* -------------------------------------------------------------------------- */
/* --- PRIVATE VARIABLES ---------------------------------------------------- */

static int exit_sig = 0; /* 1 -> application terminates cleanly (shut down hardware, close open files, etc) */
static int quit_sig = 0; /* 1 -> application terminates without shutting down the hardware */

/* -------------------------------------------------------------------------- */
/* --- PRIVATE FUNCTIONS ---------------------------------------------------- */

static void sig_handler(int sigio) {
    if (sigio == SIGQUIT) {
        quit_sig = 1;
    } else if ((sigio == SIGINT) || (sigio == SIGTERM)) {
        exit_sig = 1;
    }
}

void usage(void) {
    //printf("Library version information: %s\n", lgw_version_info());
    printf( "Available options:\n");
    printf( " -h print this help\n");
    printf( " -k <uint> Concentrator clock source (Radio A or Radio B) [0..1]\n");
    printf( " -r <uint> Radio type (1255, 1257, 1250)\n");
    printf( " -a <float> Radio A RX frequency in MHz\n");
    printf( " -b <float> Radio B RX frequency in MHz\n");
    printf( " -n <uint> number of packet received with CRC OK for each HAL start/stop loop\n");
}

/* -------------------------------------------------------------------------- */
/* --- MAIN FUNCTION -------------------------------------------------------- */

int main(int argc, char **argv)
{
    /* SPI interfaces */
    const char spidev_path_default[] = LINUXDEV_PATH_DEFAULT;
    const char * spidev_path = spidev_path_default;

    struct sigaction sigact; /* SIGQUIT&SIGINT&SIGTERM signal handling */

    int i, j, x;
    uint32_t fa = DEFAULT_FREQ_HZ;
    uint32_t fb = DEFAULT_FREQ_HZ;
    double arg_d = 0.0;
    unsigned int arg_u;
    uint8_t clocksource = 0;
    lgw_radio_type_t radio_type = LGW_RADIO_TYPE_NONE;

    struct lgw_conf_board_s boardconf;
    struct lgw_conf_rxrf_s rfconf;
    struct lgw_conf_rxif_s ifconf;
    struct lgw_pkt_rx_s rxpkt[16];

    unsigned long nb_pkt_crc_ok = 0, nb_loop = 1, cnt_loop;
    int nb_pkt;

    const int32_t channel_if[9] = {
        -400000,
        -200000,
        0,
        -400000,
        -200000,
        0,
        200000,
        400000,
        -200000 /* lora service */
    };

    const uint8_t channel_rfchain[9] = { 1, 1, 1, 0, 0, 0, 0, 0, 1 };

    /* parse command line options */
    while ((i = getopt (argc, argv, "ha:b:k:r:n:")) != -1) {
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
            case 'k': /* <uint> Clock Source */
                i = sscanf(optarg, "%u", &arg_u);
                if ((i != 1) || (arg_u > 1)) {
                    printf("ERROR: argument parsing of -k argument. Use -h to print help\n");
                    return EXIT_FAILURE;
                } else {
                    clocksource = (uint8_t)arg_u;
                }
                break;
            case 'a': /* <float> Radio A RX frequency in MHz */
                i = sscanf(optarg, "%lf", &arg_d);
                if (i != 1) {
                    printf("ERROR: argument parsing of -f argument. Use -h to print help\n");
                    return EXIT_FAILURE;
                } else {
                    fa = (uint32_t)((arg_d*1e6) + 0.5); /* .5 Hz offset to get rounding instead of truncating */
                }
                break;
            case 'b': /* <float> Radio B RX frequency in MHz */
                i = sscanf(optarg, "%lf", &arg_d);
                if (i != 1) {
                    printf("ERROR: argument parsing of -f argument. Use -h to print help\n");
                    return EXIT_FAILURE;
                } else {
                    fb = (uint32_t)((arg_d*1e6) + 0.5); /* .5 Hz offset to get rounding instead of truncating */
                }
                break;
            case 'n': /* <uint> NUmber of packets to be received before exiting */
                i = sscanf(optarg, "%u", &arg_u);
                if (i != 1) {
                    printf("ERROR: argument parsing of -n argument. Use -h to print help\n");
                    return EXIT_FAILURE;
                } else {
                    nb_loop = arg_u;
                }
                break;
            default:
                printf("ERROR: argument parsing\n");
                usage();
                return -1;
        }
    }

    /* configure signal handling */
    sigemptyset(&sigact.sa_mask);
    sigact.sa_flags = 0;
    sigact.sa_handler = sig_handler;
    sigaction(SIGQUIT, &sigact, NULL);
    sigaction(SIGINT, &sigact, NULL);
    sigaction(SIGTERM, &sigact, NULL);

    printf("===== sx1302 HAL RX test =====\n");

    /* Board reset */
    if (system("./reset_lgw.sh start") != 0) {
        printf("ERROR: failed to reset SX1302, check your reset_lgw.sh script\n");
        exit(EXIT_FAILURE);
    }

    /* Configure the gateway */
    memset( &boardconf, 0, sizeof boardconf);
    boardconf.lorawan_public = true;
    boardconf.clksrc = clocksource;
    boardconf.full_duplex = false;
    strncpy(boardconf.spidev_path, spidev_path, sizeof boardconf.spidev_path);
    if (lgw_board_setconf(&boardconf) != LGW_HAL_SUCCESS) {
        printf("ERROR: failed to configure board\n");
        return EXIT_FAILURE;
    }

    /* set configuration for RF chains */
    memset( &rfconf, 0, sizeof rfconf);
    rfconf.enable = true;
    rfconf.freq_hz = fa;
    rfconf.type = radio_type;
    rfconf.tx_enable = false;
    if (lgw_rxrf_setconf(0, &rfconf) != LGW_HAL_SUCCESS) {
        printf("ERROR: failed to configure rxrf 0\n");
        return EXIT_FAILURE;
    }

    memset( &rfconf, 0, sizeof rfconf);
    rfconf.enable = true;
    rfconf.freq_hz = fb;
    rfconf.type = radio_type;
    rfconf.tx_enable = false;
    if (lgw_rxrf_setconf(1, &rfconf) != LGW_HAL_SUCCESS) {
        printf("ERROR: failed to configure rxrf 1\n");
        return EXIT_FAILURE;
    }

    /* set configuration for LoRa multi-SF channels (bandwidth cannot be set) */
    memset(&ifconf, 0, sizeof(ifconf));
    for (i = 0; i < 9; i++) {
        ifconf.enable = true;
        ifconf.rf_chain = channel_rfchain[i];
        ifconf.freq_hz = channel_if[i];
        ifconf.datarate = DR_LORA_SF7;
        if (lgw_rxif_setconf(i, &ifconf) != LGW_HAL_SUCCESS) {
            printf("ERROR: failed to configure rxif %d\n", i);
            return EXIT_FAILURE;
        }
    }

    /* Loop until user quits */
    cnt_loop = 0;
    while( (quit_sig != 1) && (exit_sig != 1) )
    {
        cnt_loop += 1;

        /* connect, configure and start the LoRa concentrator */
        x = lgw_start();
        if (x != 0) {
            printf("ERROR: failed to start the gateway\n");
            return EXIT_FAILURE;
        }

        /* Loop until we have enough packets with CRC OK */
        printf("Waiting for packets...\n");
        nb_pkt_crc_ok = 0;
        while ((nb_pkt_crc_ok < nb_loop) && (quit_sig != 1) && (exit_sig != 1)) {
            /* fetch N packets */
            nb_pkt = lgw_receive(ARRAY_SIZE(rxpkt), rxpkt);

            if (nb_pkt == 0) {
                wait_ms(10);
            } else {
                printf("Received %d packets\n", nb_pkt);
                for (i = 0; i < nb_pkt; i++) {
                    if (rxpkt[i].status == STAT_CRC_OK) {
                        nb_pkt_crc_ok += 1;
                    }
                    printf("\n----- %s packet -----\n", (rxpkt[i].modulation == MOD_LORA) ? "LoRa" : "FSK");
                    printf("  count_us: %u\n", rxpkt[i].count_us);
                    printf("  size:     %u\n", rxpkt[i].size);
                    printf("  chan:     %u\n", rxpkt[i].if_chain);
                    printf("  status:   0x%02X\n", rxpkt[i].status);
                    printf("  datr:     %u\n", rxpkt[i].datarate);
                    printf("  codr:     %u\n", rxpkt[i].coderate);
                    printf("  rf_chain  %u\n", rxpkt[i].rf_chain);
                    printf("  freq_hz   %u\n", rxpkt[i].freq_hz);
                    printf("  snr_avg:  %.1f\n", rxpkt[i].snr);
                    printf("  rssi_chan:%.1f\n", rxpkt[i].rssic);
                    printf("  rssi_sig :%.1f\n", rxpkt[i].rssis);
                    printf("  crc:      0x%04X\n", rxpkt[i].crc);
                    for (j = 0; j < rxpkt[i].size; j++) {
                        printf("%02X ", rxpkt[i].payload[j]);
                    }
                    printf("\n");
                }
            }
        }

        printf( "\nNb valid packets received: %lu CRC OK (%lu)\n", nb_pkt_crc_ok, cnt_loop );

        /* Stop the gateway */
        x = lgw_stop();
        if (x != 0) {
            printf("ERROR: failed to stop the gateway\n");
            return EXIT_FAILURE;
        }
    }

    /* Board reset */
    if (system("./reset_lgw.sh stop") != 0) {
        printf("ERROR: failed to reset SX1302, check your reset_lgw.sh script\n");
        exit(EXIT_FAILURE);
    }

    printf("=========== Test End ===========\n");

    return 0;
}

/* --- EOF ------------------------------------------------------------------ */
