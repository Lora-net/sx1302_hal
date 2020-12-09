/*
 / _____)             _              | |
( (____  _____ ____ _| |_ _____  ____| |__
 \____ \| ___ |    (_   _) ___ |/ ___)  _ \
 _____) ) ____| | | || |_| ____( (___| | | |
(______/|_____)_|_|_| \__)_____)\____)_| |_|
  (C)2020 Semtech

Description:
    Set the sx1261 radio of the Corecell in RX continuous mode, and measure RSSI

License: Revised BSD License, see LICENSE.TXT file include in the project
*/


/* -------------------------------------------------------------------------- */
/* --- DEPENDANCIES --------------------------------------------------------- */

/* Fix an issue between POSIX and C99 */
#if __STDC_VERSION__ >= 199901L
    #define _XOPEN_SOURCE 600
#else
    #define _XOPEN_SOURCE 500
#endif

#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <signal.h>     /* sigaction */
#include <unistd.h>     /* getopt, access */

#include "loragw_aux.h"
#include "loragw_hal.h"
#include "loragw_reg.h"
#include "loragw_com.h"
#include "loragw_sx1261.h"

/* -------------------------------------------------------------------------- */
/* --- PRIVATE MACROS ------------------------------------------------------- */

/* -------------------------------------------------------------------------- */
/* --- PRIVATE CONSTANTS ---------------------------------------------------- */

#define BUFF_SIZE           16

#define COM_TYPE_DEFAULT    LGW_COM_SPI
#define COM_PATH_DEFAULT    "/dev/spidev0.0"
#define SX1261_PATH_DEFAULT "/dev/spidev0.1"

#define DEFAULT_FREQ_HZ     868500000U

/* -------------------------------------------------------------------------- */
/* --- GLOBAL VARIABLES ----------------------------------------------------- */

/* Signal handling variables */
static int exit_sig = 0; /* 1 -> application terminates cleanly (shut down hardware, close open files, etc) */
static int quit_sig = 0; /* 1 -> application terminates without shutting down the hardware */

/* -------------------------------------------------------------------------- */
/* --- SUBFUNCTIONS DECLARATION --------------------------------------------- */

static void sig_handler(int sigio);
static void usage(void);
static void exit_failure();

/* -------------------------------------------------------------------------- */
/* --- MAIN FUNCTION -------------------------------------------------------- */

int main(int argc, char ** argv)
{
    static struct sigaction sigact; /* SIGQUIT&SIGINT&SIGTERM signal handling */

    int i, x;
    double arg_d = 0.0;
    unsigned int arg_u;

    uint8_t buff[BUFF_SIZE];
    uint32_t freq_hz = 0;
    float rssi_inst;
    uint32_t fa = DEFAULT_FREQ_HZ;
    uint32_t fb = DEFAULT_FREQ_HZ;
    uint8_t clocksource = 0;
    lgw_radio_type_t radio_type = LGW_RADIO_TYPE_SX1250;

    /* COM interfaces */
    const char com_path_default[] = COM_PATH_DEFAULT;
    const char * com_path = com_path_default;
    lgw_com_type_t com_type = COM_TYPE_DEFAULT;
    const char sx1261_path_default[] = SX1261_PATH_DEFAULT;
    const char * sx1261_path = sx1261_path_default;

    struct lgw_conf_board_s boardconf;
    struct lgw_conf_rxrf_s rfconf;

    /* Parse command line options */
    while ((i = getopt(argc, argv, "hd:uf:D:k:r:a:b:")) != -1) {
        switch (i) {
            case 'h':
                usage();
                return EXIT_SUCCESS;
                break;

            case 'd':
                if (optarg != NULL) {
                    com_path = optarg;
                }
                break;

            case 'D':
                if (optarg != NULL) {
                    sx1261_path = optarg;
                }
                break;

            case 'u':
                com_type = LGW_COM_USB;
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

            case 'f': /* <float> SX1261 Radio RX frequency in MHz */
                i = sscanf(optarg, "%lf", &arg_d);
                if (i != 1) {
                    printf("ERROR: argument parsing of -f argument. Use -h to print help\n");
                    return EXIT_FAILURE;
                } else {
                    freq_hz = (uint32_t)((arg_d*1e6) + 0.5); /* .5 Hz offset to get rounding instead of truncating */
                }
                break;

            default:
                printf("ERROR: argument parsing options, use -h option for help\n");
                usage();
                return EXIT_FAILURE;
            }
    }

    /* Check mandatory params */
    if (freq_hz == 0) {
        printf("ERROR: frequency must me set\n");
        usage();
        return EXIT_FAILURE;
    }

    /* Configure signal handling */
    sigemptyset( &sigact.sa_mask );
    sigact.sa_flags = 0;
    sigact.sa_handler = sig_handler;
    sigaction( SIGQUIT, &sigact, NULL );
    sigaction( SIGINT, &sigact, NULL );
    sigaction( SIGTERM, &sigact, NULL );

    /* Board reset */
    if (com_type == LGW_COM_SPI) {
        if (system("./reset_lgw.sh start") != 0) {
            printf("ERROR: failed to reset SX1302, check your reset_lgw.sh script\n");
            exit(EXIT_FAILURE);
        }
    }

    /* Configure the gateway */
    memset( &boardconf, 0, sizeof boardconf);
    boardconf.lorawan_public = true;
    boardconf.clksrc = clocksource;
    boardconf.full_duplex = false;
    boardconf.com_type = com_type;
    strncpy(boardconf.com_path, com_path, sizeof boardconf.com_path);
    boardconf.com_path[sizeof boardconf.com_path - 1] = '\0'; /* ensure string termination */
    if (lgw_board_setconf(&boardconf) != LGW_HAL_SUCCESS) {
        printf("ERROR: failed to configure board\n");
        return EXIT_FAILURE;
    }

    /* set configuration for RF chains */
    memset( &rfconf, 0, sizeof rfconf);
    rfconf.enable = true; /* must be enabled to proper RF matching */
    rfconf.freq_hz = fa;
    rfconf.type = radio_type;
    rfconf.rssi_offset = 0.0;
    rfconf.tx_enable = false;
    rfconf.single_input_mode = false;
    if (lgw_rxrf_setconf(0, &rfconf) != LGW_HAL_SUCCESS) {
        printf("ERROR: failed to configure rxrf 0\n");
        return EXIT_FAILURE;
    }

    memset( &rfconf, 0, sizeof rfconf);
    rfconf.enable = true; /* must be enabled to proper RF matching */
    rfconf.freq_hz = fb;
    rfconf.type = radio_type;
    rfconf.rssi_offset = 0.0;
    rfconf.tx_enable = false;
    rfconf.single_input_mode = false;
    if (lgw_rxrf_setconf(1, &rfconf) != LGW_HAL_SUCCESS) {
        printf("ERROR: failed to configure rxrf 1\n");
        return EXIT_FAILURE;
    }

    /* Connect to the concentrator board */
    x = lgw_start();
    if (x != LGW_REG_SUCCESS) {
        printf("ERROR: Failed to connect to the concentrator using COM %s\n", com_path);
        return EXIT_FAILURE;
    }

    /* Connect to the sx1261 radio */
    x = sx1261_connect(com_type, sx1261_path);
    if (x != LGW_REG_SUCCESS) {
        printf("ERROR: Failed to connect to the sx1261 using COM %s\n", com_path);
        return EXIT_FAILURE;
    }

    x = sx1261_calibrate(freq_hz);
    if (x != LGW_REG_SUCCESS) {
        printf("ERROR: Failed to calibrate the sx1261\n");
        exit_failure();
    }

    x = sx1261_setup();
    if (x != LGW_REG_SUCCESS) {
        printf("ERROR: Failed to setup the sx1261\n");
        exit_failure();
    }

    x = sx1261_set_rx_params(freq_hz, BW_125KHZ);
    if (x != LGW_REG_SUCCESS) {
        printf("ERROR: Failed to set RX params\n");
        exit_failure();
    }

    /* databuffer R/W stress test */
    while ((quit_sig != 1) && (exit_sig != 1)) {
        buff[0] = 0x00;
        buff[1] = 0x00;
        sx1261_reg_r(SX1261_GET_RSSI_INST, buff, 2);

        rssi_inst = -((float)buff[1] / 2);

        printf("\rSX1261 RSSI at %uHz: %f dBm", freq_hz, rssi_inst);
        fflush(stdout);

        wait_ms(100);
    }
    printf("\n");

    /* Disconnect from the sx1261 radio */
    x = sx1261_disconnect();
    if (x != LGW_REG_SUCCESS) {
        printf("ERROR: Failed to disconnect from the SX1261 radio\n");
    }

    /* Disconnect from the concentrator board */
    x = lgw_stop();
    if (x != LGW_REG_SUCCESS) {
        printf("ERROR: Failed to disconnect from the concentrator\n");
    }

    printf("Disconnected\n");

    if (com_type == LGW_COM_SPI) {
        /* Board reset */
        if (system("./reset_lgw.sh stop") != 0) {
            printf("ERROR: failed to reset SX1302, check your reset_lgw.sh script\n");
            exit(EXIT_FAILURE);
        }
    }

    return 0;
}

/* -------------------------------------------------------------------------- */
/* --- SUBFUNCTIONS DEFINITION ---------------------------------------------- */

static void sig_handler(int sigio) {
    if (sigio == SIGQUIT) {
        quit_sig = 1;
    } else if((sigio == SIGINT) || (sigio == SIGTERM)) {
        exit_sig = 1;
    }
}

/* ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ */

static void exit_failure() {
    sx1261_disconnect();
    lgw_disconnect();

    printf("End of test for loragw_spi_sx1261.c\n");

    exit(EXIT_FAILURE);
}

/* ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ */

static void usage(void) {
    printf("~~~ Library version string~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~\n");
    printf(" %s\n", lgw_version_info());
    printf("~~~ Available options ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~\n");
    printf(" -h            print this help\n");
    printf(" -u            set COM type as USB (default is SPI)\n");
    printf(" -d <path>     path to access the main COM device\n");
    printf("               => default path: " COM_PATH_DEFAULT "\n");
    printf(" -D [path]     Path to the SX1261 SPI interface (not used for USB)\n");
    printf("               => default path: " SX1261_PATH_DEFAULT "\n");
    printf(" -k <uint>     Concentrator clock source (Radio A or Radio B) [0..1]\n");
    printf(" -r <uint>     Radio type (1255, 1257, 1250)\n");
    printf(" -a <float>    Radio A RX frequency in MHz\n");
    printf(" -b <float>    Radio B RX frequency in MHz\n");
    printf(" -f <float>    SX1261 frequency for RSSI scanning, in MHz\n");
}

/* --- EOF ------------------------------------------------------------------ */
