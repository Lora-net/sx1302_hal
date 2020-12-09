/*
 / _____)             _              | |
( (____  _____ ____ _| |_ _____  ____| |__
 \____ \| ___ |    (_   _) ___ |/ ___)  _ \
 _____) ) ____| | | || |_| ____( (___| | | |
(______/|_____)_|_|_| \__)_____)\____)_| |_|
  (C)2019 Semtech

Description:
    Minimum test program for HAL TX capability

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

#include "loragw_hal.h"
#include "loragw_reg.h"
#include "loragw_aux.h"

/* -------------------------------------------------------------------------- */
/* --- PRIVATE MACROS ------------------------------------------------------- */

#define RAND_RANGE(min, max) (rand() % (max + 1 - min) + min)

/* -------------------------------------------------------------------------- */
/* --- PRIVATE CONSTANTS ---------------------------------------------------- */

#define COM_TYPE_DEFAULT LGW_COM_SPI
#define COM_PATH_DEFAULT "/dev/spidev0.0"

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
    printf("Available options:\n");
    printf(" -h print this help\n");
    printf(" -u         Set COM type as USB (default is SPI)\n");
    printf(" -d <path>  COM path to be used to connect the concentrator\n");
    printf("            => default path: " COM_PATH_DEFAULT "\n");
    printf(" -k <uint>  Concentrator clock source (Radio A or Radio B) [0..1]\n");
    printf(" -c <uint>  RF chain to be used for TX (Radio A or Radio B) [0..1]\n");
    printf(" -r <uint>  Radio type (1255, 1257, 1250)\n");
    printf(" -f <float> Radio TX frequency in MHz\n");
    printf(" -m <str>   modulation type ['CW', 'LORA', 'FSK']\n");
    printf(" -o <int>   CW frequency offset from Radio TX frequency in kHz [-65..65]\n");
    printf(" -s <uint>  LoRa datarate 0:random, [5..12]\n");
    printf(" -b <uint>  LoRa bandwidth in khz 0:random, [125, 250, 500]\n");
    printf(" -l <uint>  FSK/LoRa preamble length, [6..65535]\n");
    printf(" -n <uint>  Number of packets to be sent\n");
    printf(" -z <uint>  size of packets to be sent 0:random, [9..255]\n");
    printf(" -t <uint>  TX mode timestamped with delay in ms. If delay is 0, TX mode GPS trigger\n");
    printf(" -p <int>   RF power in dBm\n");
    printf(" -i         Send LoRa packet using inverted modulation polarity\n");
    printf(" -j         Set radio in single input mode (SX1250 only)\n");
    printf( "~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~\n" );
    printf(" --fdev <uint>  FSK frequency deviation in kHz [1:250]\n");
    printf(" --br   <float> FSK bitrate in kbps [0.5:250]\n");
    printf( "~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~\n" );
    printf(" --pa   <uint> PA gain SX125x:[0..3], SX1250:[0,1]\n");
    printf(" --dig  <uint> sx1302 digital gain for sx125x [0..3]\n");
    printf(" --dac  <uint> sx125x DAC gain [0..3]\n");
    printf(" --mix  <uint> sx125x MIX gain [5..15]\n");
    printf(" --pwid <uint> sx1250 power index [0..22]\n");
    printf( "~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~\n" );
    printf(" --nhdr        Send LoRa packet with implicit header\n");
    printf( "~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~\n" );
    printf(" --loop        Number of loops for HAL start/stop (HAL unitary test)\n");
    printf( "~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~\n" );
    printf(" --fdd         Enable Full-Duplex mode (CN490 reference design)\n");
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
    uint16_t bw_khz = 0;
    uint32_t nb_pkt = 1;
    unsigned int nb_loop = 1, cnt_loop;
    uint8_t size = 0;
    char mod[64] = "LORA";
    float br_kbps = 50;
    uint8_t fdev_khz = 25;
    int8_t freq_offset = 0;
    double arg_d = 0.0;
    unsigned int arg_u;
    int arg_i;
    char arg_s[64];
    float xf = 0.0;
    uint8_t clocksource = 0;
    uint8_t rf_chain = 0;
    lgw_radio_type_t radio_type = LGW_RADIO_TYPE_NONE;
    uint16_t preamble = 8;
    bool invert_pol = false;
    bool no_header = false;
    bool single_input_mode = false;
    bool full_duplex = false;

    struct lgw_conf_board_s boardconf;
    struct lgw_conf_rxrf_s rfconf;
    struct lgw_pkt_tx_s pkt;
    struct lgw_tx_gain_lut_s txlut; /* TX gain table */
    uint8_t tx_status;
    uint32_t count_us;
    uint32_t trig_delay_us = 1000000;
    bool trig_delay = false;

    /* SPI interfaces */
    const char com_path_default[] = COM_PATH_DEFAULT;
    const char * com_path = com_path_default;
    lgw_com_type_t com_type = COM_TYPE_DEFAULT;

    static struct sigaction sigact; /* SIGQUIT&SIGINT&SIGTERM signal handling */

    /* Initialize TX gain LUT */
    txlut.size = 0;
    memset(txlut.lut, 0, sizeof txlut.lut);

    /* Parameter parsing */
    int option_index = 0;
    static struct option long_options[] = {
        {"fdev", required_argument, 0, 0},
        {"br",   required_argument, 0, 0},
        {"pa",   required_argument, 0, 0},
        {"dac",  required_argument, 0, 0},
        {"dig",  required_argument, 0, 0},
        {"mix",  required_argument, 0, 0},
        {"pwid", required_argument, 0, 0},
        {"loop", required_argument, 0, 0},
        {"nhdr", no_argument, 0, 0},
        {"fdd",  no_argument, 0, 0},
        {0, 0, 0, 0}
    };

    /* parse command line options */
    while ((i = getopt_long (argc, argv, "hjif:s:b:n:z:p:k:r:c:l:t:m:o:ud:", long_options, &option_index)) != -1) {
        switch (i) {
            case 'h':
                usage();
                return -1;
                break;
            case 'u':
                com_type = LGW_COM_USB;
                break;
            case 'd': /* <char> COM path */
                if (optarg != NULL) {
                    com_path = optarg;
                }
                break;
            case 'i': /* Send packet using inverted modulation polarity */
                invert_pol = true;
                break;
            case 'j': /* Set radio in single input mode */
                single_input_mode = true;
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
            case 'l': /* <uint> LoRa/FSK preamble length */
                i = sscanf(optarg, "%u", &arg_u);
                if ((i != 1) || (arg_u > 65535)) {
                    printf("ERROR: argument parsing of -l argument. Use -h to print help\n");
                    return EXIT_FAILURE;
                } else {
                    preamble = (uint16_t)arg_u;
                }
                break;
            case 'm': /* <str> Modulation type */
                i = sscanf(optarg, "%s", arg_s);
                if ((i != 1) || ((strcmp(arg_s, "CW") != 0) && (strcmp(arg_s, "LORA") != 0) && (strcmp(arg_s, "FSK")))) {
                    printf("ERROR: invalid modulation type\n");
                    return EXIT_FAILURE;
                } else {
                    sprintf(mod, "%s", arg_s);
                }
                break;
            case 'o': /* <int> CW frequency offset from Radio TX frequency */
                i = sscanf(optarg, "%d", &arg_i);
                if ((arg_i < -65) || (arg_i > 65)) {
                    printf("ERROR: invalid frequency offset\n");
                    return EXIT_FAILURE;
                } else {
                    freq_offset = (int32_t)arg_i;
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
                if ((i != 1) || (arg_u < 5) || (arg_u > 12)) {
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
                    bw_khz = (uint16_t)arg_u;
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
                    txlut.size = 1;
                    txlut.lut[0].rf_power = rf_power;
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
            case 0:
                if (strcmp(long_options[option_index].name, "fdev") == 0) {
                    i = sscanf(optarg, "%u", &arg_u);
                    if ((i != 1) || (arg_u < 1) || (arg_u > 250)) {
                        printf("ERROR: invalid FSK frequency deviation\n");
                        return EXIT_FAILURE;
                    } else {
                        fdev_khz = (uint8_t)arg_u;
                    }
                } else if (strcmp(long_options[option_index].name, "br") == 0) {
                    i = sscanf(optarg, "%f", &xf);
                    if ((i != 1) || (xf < 0.5) || (xf > 250)) {
                        printf("ERROR: invalid FSK bitrate\n");
                        return EXIT_FAILURE;
                    } else {
                        br_kbps = xf;
                    }
                } else if (strcmp(long_options[option_index].name, "pa") == 0) {
                    i = sscanf(optarg, "%u", &arg_u);
                    if ((i != 1) || (arg_u > 3)) {
                        printf("ERROR: argument parsing of --pa argument. Use -h to print help\n");
                        return EXIT_FAILURE;
                    } else {
                        txlut.size = 1;
                        txlut.lut[0].pa_gain = (uint8_t)arg_u;
                    }
                } else if (strcmp(long_options[option_index].name, "dac") == 0) {
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
                } else if (strcmp(long_options[option_index].name, "dig") == 0) {
                    i = sscanf(optarg, "%u", &arg_u);
                    if ((i != 1) || (arg_u > 3)) {
                        printf("ERROR: argument parsing of --dig argument. Use -h to print help\n");
                        return EXIT_FAILURE;
                    } else {
                        txlut.size = 1;
                        txlut.lut[0].dig_gain = (uint8_t)arg_u;
                    }
                } else if (strcmp(long_options[option_index].name, "pwid") == 0) {
                    i = sscanf(optarg, "%u", &arg_u);
                    if ((i != 1) || (arg_u > 22)) {
                        printf("ERROR: argument parsing of --pwid argument. Use -h to print help\n");
                        return EXIT_FAILURE;
                    } else {
                        txlut.size = 1;
                        txlut.lut[0].mix_gain = 5; /* TODO: rework this, should not be needed for sx1250 */
                        txlut.lut[0].pwr_idx = (uint8_t)arg_u;
                    }
                } else if (strcmp(long_options[option_index].name, "loop") == 0) {
                    printf("%p\n", optarg);
                    i = sscanf(optarg, "%u", &arg_u);
                    if (i != 1) {
                        printf("ERROR: argument parsing of --loop argument. Use -h to print help\n");
                        return EXIT_FAILURE;
                    } else {
                        nb_loop = arg_u;
                    }
                } else if (strcmp(long_options[option_index].name, "nhdr") == 0) {
                    no_header = true;
                } else if (strcmp(long_options[option_index].name, "fdd") == 0) {
                    full_duplex = true;
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

    /* Summary of packet parameters */
    if (strcmp(mod, "CW") == 0) {
        printf("Sending %i CW on %u Hz (Freq. offset %d kHz) at %i dBm\n", nb_pkt, ft, freq_offset, rf_power);
    }
    else if (strcmp(mod, "FSK") == 0) {
        printf("Sending %i FSK packets on %u Hz (FDev %u kHz, Bitrate %.2f, %i bytes payload, %i symbols preamble) at %i dBm\n", nb_pkt, ft, fdev_khz, br_kbps, size, preamble, rf_power);
    } else {
        printf("Sending %i LoRa packets on %u Hz (BW %i kHz, SF %i, CR %i, %i bytes payload, %i symbols preamble, %s header, %s polarity) at %i dBm\n", nb_pkt, ft, bw_khz, sf, 1, size, preamble, (no_header == false) ? "explicit" : "implicit", (invert_pol == false) ? "non-inverted" : "inverted", rf_power);
    }

    /* Configure signal handling */
    sigemptyset( &sigact.sa_mask );
    sigact.sa_flags = 0;
    sigact.sa_handler = sig_handler;
    sigaction( SIGQUIT, &sigact, NULL );
    sigaction( SIGINT, &sigact, NULL );
    sigaction( SIGTERM, &sigact, NULL );

    /* Configure the gateway */
    memset( &boardconf, 0, sizeof boardconf);
    boardconf.lorawan_public = true;
    boardconf.clksrc = clocksource;
    boardconf.full_duplex = full_duplex;
    boardconf.com_type = com_type;
    strncpy(boardconf.com_path, com_path, sizeof boardconf.com_path);
    boardconf.com_path[sizeof boardconf.com_path - 1] = '\0'; /* ensure string termination */
    if (lgw_board_setconf(&boardconf) != LGW_HAL_SUCCESS) {
        printf("ERROR: failed to configure board\n");
        return EXIT_FAILURE;
    }

    memset( &rfconf, 0, sizeof rfconf);
    rfconf.enable = true; /* rf chain 0 needs to be enabled for calibration to work on sx1257 */
    rfconf.freq_hz = ft;
    rfconf.type = radio_type;
    rfconf.tx_enable = true;
    rfconf.single_input_mode = single_input_mode;
    if (lgw_rxrf_setconf(0, &rfconf) != LGW_HAL_SUCCESS) {
        printf("ERROR: failed to configure rxrf 0\n");
        return EXIT_FAILURE;
    }

    memset( &rfconf, 0, sizeof rfconf);
    rfconf.enable = (((rf_chain == 1) || (clocksource == 1)) ? true : false);
    rfconf.freq_hz = ft;
    rfconf.type = radio_type;
    rfconf.tx_enable = false;
    rfconf.single_input_mode = single_input_mode;
    if (lgw_rxrf_setconf(1, &rfconf) != LGW_HAL_SUCCESS) {
        printf("ERROR: failed to configure rxrf 1\n");
        return EXIT_FAILURE;
    }

    if (txlut.size > 0) {
        if (lgw_txgain_setconf(rf_chain, &txlut) != LGW_HAL_SUCCESS) {
            printf("ERROR: failed to configure txgain lut\n");
            return EXIT_FAILURE;
        }
    }

    for (cnt_loop = 0; cnt_loop < nb_loop; cnt_loop++) {
        if (com_type == LGW_COM_SPI) {
        /* Board reset */
            if (system("./reset_lgw.sh start") != 0) {
                printf("ERROR: failed to reset SX1302, check your reset_lgw.sh script\n");
                exit(EXIT_FAILURE);
            }
        }

        /* connect, configure and start the LoRa concentrator */
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
            if (trig_delay_us == 0) {
                pkt.tx_mode = ON_GPS;
            } else {
                pkt.tx_mode = TIMESTAMPED;
            }
        }
        if ( strcmp( mod, "CW" ) == 0 ) {
            pkt.modulation = MOD_CW;
            pkt.freq_offset = freq_offset;
            pkt.f_dev = fdev_khz;
        }
        else if( strcmp( mod, "FSK" ) == 0 ) {
            pkt.modulation = MOD_FSK;
            pkt.no_crc = false;
            pkt.datarate = br_kbps * 1e3;
            pkt.f_dev = fdev_khz;
        } else {
            pkt.modulation = MOD_LORA;
            pkt.coderate = CR_LORA_4_5;
            pkt.no_crc = true;
        }
        pkt.invert_pol = invert_pol;
        pkt.preamble = preamble;
        pkt.no_header = no_header;
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
                if (trig_delay_us > 0) {
                    lgw_get_instcnt(&count_us);
                    printf("count_us:%u\n", count_us);
                    pkt.count_us = count_us + trig_delay_us;
                    printf("programming TX for %u\n", pkt.count_us);
                } else {
                    printf("programming TX for next PPS (GPS)\n");
                }
            }

            if( strcmp( mod, "LORA" ) == 0 ) {
                pkt.datarate = (sf == 0) ? (uint8_t)RAND_RANGE(5, 12) : sf;
            }

            switch (bw_khz) {
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
                    pkt.bandwidth = (uint8_t)RAND_RANGE(BW_125KHZ, BW_500KHZ);
                    break;
            }

            pkt.size = (size == 0) ? (uint8_t)RAND_RANGE(9, 255) : size;

            pkt.payload[6] = (uint8_t)(i >> 0); /* FCnt */
            pkt.payload[7] = (uint8_t)(i >> 8); /* FCnt */
            x = lgw_send(&pkt);
            if (x != 0) {
                printf("ERROR: failed to send packet\n");
                break;
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

        printf( "\nNb packets sent: %u (%u)\n", i, cnt_loop + 1 );

        /* Stop the gateway */
        x = lgw_stop();
        if (x != 0) {
            printf("ERROR: failed to stop the gateway\n");
        }

        if (com_type == LGW_COM_SPI) {
            /* Board reset */
            if (system("./reset_lgw.sh stop") != 0) {
                printf("ERROR: failed to reset SX1302, check your reset_lgw.sh script\n");
                exit(EXIT_FAILURE);
            }
        }
    }

    printf("=========== Test End ===========\n");

    return 0;
}

/* --- EOF ------------------------------------------------------------------ */
