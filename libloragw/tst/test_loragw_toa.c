/*
 / _____)             _              | |
( (____  _____ ____ _| |_ _____  ____| |__
 \____ \| ___ |    (_   _) ___ |/ ___)  _ \
 _____) ) ____| | | || |_| ____( (___| | | |
(______/|_____)_|_|_| \__)_____)\____)_| |_|
  (C)2020 Semtech

Description:
    Utility to compute Time on Air of a LoRa packet

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

#include <stdint.h>     /* C99 types */
#include <stdbool.h>    /* bool type */
#include <stdio.h>      /* printf fprintf */
#include <stdlib.h>     /* EXIT_FAILURE */
#include <getopt.h>     /* getopt_long */
#include <string.h>     /* strcmp */

#include "loragw_hal.h"

/* -------------------------------------------------------------------------- */
/* --- PRIVATE MACROS ------------------------------------------------------- */

/* -------------------------------------------------------------------------- */
/* --- PRIVATE CONSTANTS ---------------------------------------------------- */

/* -------------------------------------------------------------------------- */
/* --- PRIVATE VARIABLES ---------------------------------------------------- */

/* -------------------------------------------------------------------------- */
/* --- PRIVATE FUNCTIONS DECLARATION ---------------------------------------- */

/* -------------------------------------------------------------------------- */
/* --- PRIVATE FUNCTIONS DEFINITION ----------------------------------------- */

/* describe command line options */
void usage(void) {
    printf("Library version information: %s\n", lgw_version_info());
    printf("Available options:\n");
    printf(" -h print this help\n");
    printf(" -s <uint>  LoRa datarate [5..12]\n");
    printf(" -b <uint>  LoRa bandwidth in khz [125, 250, 500]\n");
    printf(" -l <uint>  LoRa preamble length, [6..65535]\n");
    printf(" -c <uint>  LoRa coding rate [1=4/5 2=4/6 3=4/7 4=4/8]\n");
    printf(" -z <uint>  Payload length [0..255]\n");
    printf(" -i         Implicit header (no header)\n");
    printf(" -r         CRC enabled\n");
}

/* -------------------------------------------------------------------------- */
/* --- MAIN FUNCTION -------------------------------------------------------- */

int main(int argc, char **argv) {
    int i;
    unsigned int arg_u;

    struct lgw_pkt_tx_s pkt;
    uint32_t toa_u;

    /* mandatory params to be set by user */
    bool sf = false;
    bool bw = false;
    bool preamb = false;
    bool cr = false;
    bool sz = false;

    /* Parameter parsing */
    int option_index = 0;
    static struct option long_options[] = {
        {0, 0, 0, 0}
    };

    memset(&pkt, 0, sizeof pkt);
    pkt.no_crc = true;
    pkt.modulation = MOD_LORA;

    /* parse command line options */
    while ((i = getopt_long (argc, argv, "hirs:b:z:l:c:", long_options, &option_index)) != -1) {
        switch (i) {
            case 'h':
                usage();
                return -1;
                break;
            case 'i':
                pkt.no_header = true;
                break;
            case 'r':
                pkt.no_crc = false;
                break;
            case 'l':
                preamb = true; /* param set */
                i = sscanf(optarg, "%u", &arg_u);
                if ((i != 1) || (arg_u > 65535)) {
                    printf("ERROR: argument parsing of -l argument. Use -h to print help\n");
                    return EXIT_FAILURE;
                } else {
                    pkt.preamble = (uint16_t)arg_u;
                }
                break;
            case 's':
                sf = true; /* param set */
                i = sscanf(optarg, "%u", &arg_u);
                if ((i != 1) || (arg_u < 5) || (arg_u > 12)) {
                    printf("ERROR: argument parsing of -s argument. Use -h to print help\n");
                    return EXIT_FAILURE;
                } else {
                    pkt.datarate = arg_u;
                }
                break;
            case 'b':
                bw = true; /* param set */
                i = sscanf(optarg, "%u", &arg_u);
                if (i != 1) {
                    printf("ERROR: argument parsing of -b argument. Use -h to print help\n");
                    return EXIT_FAILURE;
                } else {
                    switch (arg_u) {
                        case 125:
                            pkt.bandwidth = BW_125KHZ;
                            break;
                        case 250:
                            pkt.bandwidth = BW_250KHZ;
                            break;
                        case 500:
                            pkt.bandwidth = BW_500KHZ;
                            break;
                        default:
                            printf("ERROR: argument parsing of -b argument. Use -h to print help\n");
                            return EXIT_FAILURE;
                    }
                }
                break;
            case 'c':
                cr = true; /* param set */
                i = sscanf(optarg, "%u", &arg_u);
                if (i != 1) {
                    printf("ERROR: argument parsing of -b argument. Use -h to print help\n");
                    return EXIT_FAILURE;
                } else {
                    switch (arg_u) {
                        case 1:
                            pkt.coderate = CR_LORA_4_5;
                            break;
                        case 2:
                            pkt.coderate = CR_LORA_4_6;
                            break;
                        case 3:
                            pkt.coderate = CR_LORA_4_7;
                            break;
                        case 4:
                            pkt.coderate = CR_LORA_4_8;
                            break;
                        default:
                            printf("ERROR: argument parsing of -b argument. Use -h to print help\n");
                            return EXIT_FAILURE;
                    }
                }
                break;
            case 'z':
                sz = true; /* param set */
                i = sscanf(optarg, "%u", &arg_u);
                if ((i != 1) || (arg_u > 255)) {
                    printf("ERROR: argument parsing of -z argument. Use -h to print help\n");
                    return EXIT_FAILURE;
                } else {
                    pkt.size = (uint8_t)arg_u;
                }
                break;
            default:
                printf("ERROR: argument parsing\n");
                usage();
                return EXIT_FAILURE;
        }
    }

    printf("### LoRa - Time On Air Calculator ###\n");

    if (sf == false ||
        bw == false ||
        preamb == false ||
        cr == false ||
        sz == false) {
            printf("ERROR: missing mandatory packet description parameter\n");
            usage();
            return EXIT_FAILURE;
        }

    toa_u = lgw_time_on_air(&pkt);
    printf("=> %u ms\n", toa_u);

    return 0;
}
