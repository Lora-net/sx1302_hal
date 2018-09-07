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
#include <signal.h>
#include <math.h>

#include "loragw_hal.h"
#include "loragw_reg.h"
#include "loragw_aux.h"

/* -------------------------------------------------------------------------- */
/* --- PRIVATE MACROS ------------------------------------------------------- */

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
        quit_sig = 1;;
    } else if ((sigio == SIGINT) || (sigio == SIGTERM)) {
        exit_sig = 1;
    }
}

void usage(void) {
    //printf("Library version information: %s\n", lgw_version_info());
    printf( "Available options:\n");
    printf( " -h print this help\n");
    printf( " -a <float> Radio A RX frequency in MHz\n");
    printf( " -b <float> Radio B RX frequency in MHz\n");
}

/* -------------------------------------------------------------------------- */
/* --- MAIN FUNCTION -------------------------------------------------------- */

int main(int argc, char **argv)
{
    struct sigaction sigact; /* SIGQUIT&SIGINT&SIGTERM signal handling */

    int i, j, x;
    uint32_t fa = DEFAULT_FREQ_HZ;
    uint32_t fb = DEFAULT_FREQ_HZ;
    double arg_d = 0.0;

    struct lgw_conf_board_s boardconf;
    struct lgw_conf_rxrf_s rfconf;
    struct lgw_conf_rxif_s ifconf;
    struct lgw_pkt_rx_s rxpkt[8];

    int nb_pkt;

    const int32_t channel_if[8] = {
        700000,
        500000,
        300000,
        100000,
        -100000,
        -300000,
        -500000,
        -700000
    };

    /* parse command line options */
    while ((i = getopt (argc, argv, "ha:b:")) != -1) {
        switch (i) {
            case 'h':
                usage();
                return -1;
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
    system("./reset_lgw.sh start");

    /* Configure the gateway */
    memset( &boardconf, 0, sizeof boardconf);
    boardconf.lorawan_public = true;
    boardconf.clksrc = 0;
    lgw_board_setconf(boardconf);

    /* set configuration for RF chains */
    memset( &rfconf, 0, sizeof rfconf);
    rfconf.enable = true;
    rfconf.freq_hz = fa;
    rfconf.type = LGW_RADIO_TYPE_SX1250;
    rfconf.tx_enable = false;
    lgw_rxrf_setconf(0, rfconf);

    memset( &rfconf, 0, sizeof rfconf);
    rfconf.enable = false;
    rfconf.freq_hz = fb;
    rfconf.type = LGW_RADIO_TYPE_SX1250;
    rfconf.tx_enable = false;
    lgw_rxrf_setconf(1, rfconf);

    /* set configuration for LoRa multi-SF channels (bandwidth cannot be set) */
    memset(&ifconf, 0, sizeof(ifconf));

    for (i = 0; i < 8; i++) {
        ifconf.enable = true;
        ifconf.rf_chain = 0;
        ifconf.freq_hz = channel_if[i];
        ifconf.datarate = DR_LORA_SF7;
        lgw_rxif_setconf(i, ifconf);
    }

    /* connect, configure and start the LoRa concentrator */
    x = lgw_start();
    if (x != 0) {
        printf("ERROR: failed to start the gateway\n");
        return EXIT_FAILURE;
    }

    /* Receive packets */
    printf("Waiting for packets...\n");
    while ((quit_sig != 1) && (exit_sig != 1)) {
        /* fetch N packets */
        nb_pkt = lgw_receive(ARRAY_SIZE(rxpkt), rxpkt);

        if (nb_pkt == 0) {
            wait_ms(10);
        } else {
            //wait_ms(2000);
            printf("Received %d packets\n", nb_pkt);
            for (i = 0; i < nb_pkt; i++) {
                printf("\n----- %s packet -----\n", (rxpkt[i].modulation == MOD_LORA) ? "LoRa" : "FSK");
                printf("  size:     %u\n", rxpkt[i].size);
                printf("  chan:     %u\n", rxpkt[i].if_chain);
                printf("  status:   0x%02X\n", rxpkt[i].status);
                printf("  datr:     %u\n", rxpkt[i].datarate);
                printf("  codr:     %u\n", rxpkt[i].coderate);
                printf("  rf_chain  %u\n", rxpkt[i].rf_chain);
                printf("  freq_hz   %u\n", rxpkt[i].freq_hz);
                printf("  snr_avg:  %.1f\n", rxpkt[i].snr);
                printf("  rssi_chan:%.1f\n", rxpkt[i].rssi);
                for (j = 0; j < rxpkt[i].size; j++) {
                    printf("%02X ", rxpkt[i].payload[j]);
                }
                printf("\n");
            }
        }
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
