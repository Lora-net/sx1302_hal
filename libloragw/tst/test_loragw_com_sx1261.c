/*
 / _____)             _              | |
( (____  _____ ____ _| |_ _____  ____| |__
 \____ \| ___ |    (_   _) ___ |/ ___)  _ \
 _____) ) ____| | | || |_| ____( (___| | | |
(______/|_____)_|_|_| \__)_____)\____)_| |_|
  (C)2019 Semtech

Description:
    Minimum test program for the sx1261_com module

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

    uint8_t test_buff[BUFF_SIZE];
    uint8_t read_buff[BUFF_SIZE];
    uint32_t test_val, read_val;
    int cycle_number = 0;
    int i, x;

    /* COM interfaces */
    const char com_path_default[] = COM_PATH_DEFAULT;
    const char * com_path = com_path_default;
    lgw_com_type_t com_type = COM_TYPE_DEFAULT;

    /* Parse command line options */
    while ((i = getopt(argc, argv, "hd:u")) != -1) {
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

            case 'u':
                com_type = LGW_COM_USB;
                break;

            default:
                printf("ERROR: argument parsing options, use -h option for help\n");
                usage();
                return EXIT_FAILURE;
            }
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

    /* Connect to the concentrator board */
    x = lgw_connect(com_type, com_path);
    if (x != LGW_REG_SUCCESS) {
        printf("ERROR: Failed to connect to the concentrator using COM %s\n", com_path);
        return EXIT_FAILURE;
    }

    /* Connect to the sx1261 radio */
    x = sx1261_connect(com_type, SX1261_PATH_DEFAULT);
    if (x != LGW_REG_SUCCESS) {
        printf("ERROR: Failed to connect to the sx1261 using COM %s\n", com_path);
        return EXIT_FAILURE;
    }

    /* Set Radio in Standby mode */
    test_buff[0] = (uint8_t)SX1261_STDBY_RC;
    x = sx1261_reg_w(SX1261_SET_STANDBY, test_buff, 1);
    if (x != LGW_REG_SUCCESS) {
        printf("ERROR(%d): Failed to configure sx1261\n", __LINE__);
        exit_failure();
    }
    wait_ms(10);

    test_buff[0] = 0x00;
    x = sx1261_reg_r(SX1261_GET_STATUS, test_buff, 1);
    if (x != LGW_REG_SUCCESS) {
        printf("ERROR(%d): Failed to get sx1261 status\n", __LINE__);
        exit_failure();
    }
    printf("SX1261: get_status: 0x%02X\n", test_buff[0]);

    /* databuffer R/W stress test */
    while ((quit_sig != 1) && (exit_sig != 1)) {
        test_buff[0] = rand() & 0x7F;
        test_buff[1] = rand() & 0xFF;
        test_buff[2] = rand() & 0xFF;
        test_buff[3] = rand() & 0xFF;
        test_val = (test_buff[0] << 24) | (test_buff[1] << 16) | (test_buff[2] << 8) | (test_buff[3] << 0);
        sx1261_reg_w(SX1261_SET_RF_FREQUENCY, test_buff, 4);

        read_buff[0] = 0x08;
        read_buff[1] = 0x8B;
        read_buff[2] = 0x00;
        read_buff[3] = 0x00;
        read_buff[4] = 0x00;
        read_buff[5] = 0x00;
        read_buff[6] = 0x00;
        sx1261_reg_r(SX1261_READ_REGISTER, read_buff, 7);
        read_val = (read_buff[3] << 24) | (read_buff[4] << 16) | (read_buff[5] << 8) | (read_buff[6] << 0);

        printf("Cycle %i > ", cycle_number);
        if (read_val != test_val) {
            printf("error during the buffer comparison\n");
            printf("Written value: %08X\n", test_val);
            printf("Read value:    %08X\n", read_val);
            return EXIT_FAILURE;
        } else {
            printf("did a %i-byte R/W on a register with no error\n", 4);
            ++cycle_number;
        }
    }

    /* Disconnect from the sx1261 radio */
    sx1261_disconnect();

    /* Disconnect from the concentrator board */
    lgw_disconnect();

    printf("End of test for loragw_spi_sx1261.c\n");

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
    printf(" -d <path>     path to access the COM device\n");
    printf("               => default path: " COM_PATH_DEFAULT "\n");
    printf(" -u            set COM type as USB (default is SPI)\n");
}

/* --- EOF ------------------------------------------------------------------ */
