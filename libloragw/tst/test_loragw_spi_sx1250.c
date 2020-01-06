/*
 / _____)             _              | |
( (____  _____ ____ _| |_ _____  ____| |__
 \____ \| ___ |    (_   _) ___ |/ ___)  _ \
 _____) ) ____| | | || |_| ____( (___| | | |
(______/|_____)_|_|_| \__)_____)\____)_| |_|
  (C)2019 Semtech

Description:
    Minimum test program for the sx1250 module

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

#include "loragw_spi.h"
#include "loragw_aux.h"
#include "loragw_reg.h"
#include "loragw_hal.h"
#include "loragw_sx1250.h"
#include "loragw_sx1302.h"

/* -------------------------------------------------------------------------- */
/* --- PRIVATE MACROS ------------------------------------------------------- */

/* -------------------------------------------------------------------------- */
/* --- PRIVATE CONSTANTS ---------------------------------------------------- */

#define BUFF_SIZE           16

#define LINUXDEV_PATH_DEFAULT "/dev/spidev0.0"

/* -------------------------------------------------------------------------- */
/* --- GLOBAL VARIABLES ----------------------------------------------------- */

/* Signal handling variables */
static int exit_sig = 0; /* 1 -> application terminates cleanly (shut down hardware, close open files, etc) */
static int quit_sig = 0; /* 1 -> application terminates without shutting down the hardware */

/* -------------------------------------------------------------------------- */
/* --- SUBFUNCTIONS DECLARATION --------------------------------------------- */

static void sig_handler(int sigio);
static void usage(void);

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

    /* SPI interfaces */
    const char spidev_path_default[] = LINUXDEV_PATH_DEFAULT;
    const char * spidev_path = spidev_path_default;

    /* Parse command line options */
    while ((i = getopt(argc, argv, "hd:")) != -1) {
        switch (i) {
            case 'h':
                usage();
                return EXIT_SUCCESS;
                break;

            case 'd':
                if (optarg != NULL) {
                    spidev_path = optarg;
                }
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
    if (system("./reset_lgw.sh start") != 0) {
        printf("ERROR: failed to reset SX1302, check your reset_lgw.sh script\n");
        exit(EXIT_FAILURE);
    }

    x = lgw_connect(spidev_path);
    if (x != LGW_REG_SUCCESS) {
        printf("ERROR: Failed to connect to the concentrator using SPI %s\n", spidev_path);
        return EXIT_FAILURE;
    }

    /* Reset radios */
    for (i = 0; i < LGW_RF_CHAIN_NB; i++) {
        sx1302_radio_reset(i, LGW_RADIO_TYPE_SX1250);
        sx1302_radio_set_mode(i, LGW_RADIO_TYPE_SX1250);
    }

    /* Select the radio which provides the clock to the sx1302 */
    sx1302_radio_clock_select(0);

    /* Ensure we can control the radio */
    lgw_reg_w(SX1302_REG_COMMON_CTRL0_HOST_RADIO_CTRL, 0x01);

    /* Ensure PA/LNA are disabled */
    lgw_reg_w(SX1302_REG_AGC_MCU_CTRL_FORCE_HOST_FE_CTRL, 1);
    lgw_reg_w(SX1302_REG_AGC_MCU_RF_EN_A_PA_EN, 0);
    lgw_reg_w(SX1302_REG_AGC_MCU_RF_EN_A_LNA_EN, 0);

    /* Set Radio in Standby mode */
    test_buff[0] = (uint8_t)STDBY_XOSC;
    sx1250_write_command(0, SET_STANDBY, test_buff, 1);
    sx1250_write_command(1, SET_STANDBY, test_buff, 1);
    wait_ms(10);

    test_buff[0] = 0x00;
    sx1250_read_command(0, GET_STATUS, test_buff, 1);
    printf("Radio0: get_status: 0x%02X\n", test_buff[0]);
    sx1250_read_command(1, GET_STATUS, test_buff, 1);
    printf("Radio1: get_status: 0x%02X\n", test_buff[0]);

    /* databuffer R/W stress test */
    while ((quit_sig != 1) && (exit_sig != 1)) {
        test_buff[0] = rand() & 0x7F;
        test_buff[1] = rand() & 0xFF;
        test_buff[2] = rand() & 0xFF;
        test_buff[3] = rand() & 0xFF;
        test_val = (test_buff[0] << 24) | (test_buff[1] << 16) | (test_buff[2] << 8) | (test_buff[3] << 0);
        sx1250_write_command(0, SET_RF_FREQUENCY, test_buff, 4);

        read_buff[0] = 0x08;
        read_buff[1] = 0x8B;
        read_buff[2] = 0x00;
        read_buff[3] = 0x00;
        read_buff[4] = 0x00;
        read_buff[5] = 0x00;
        read_buff[6] = 0x00;
        sx1250_read_command(0, READ_REGISTER, read_buff, 7);
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

    lgw_disconnect();
    printf("End of test for loragw_spi_sx1250.c\n");

    /* Board reset */
    if (system("./reset_lgw.sh stop") != 0) {
        printf("ERROR: failed to reset SX1302, check your reset_lgw.sh script\n");
        exit(EXIT_FAILURE);
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

static void usage(void) {
    printf("~~~ Library version string~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~\n");
    printf(" %s\n", lgw_version_info());
    printf("~~~ Available options ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~\n");
    printf(" -h            print this help\n");
    printf(" -d <path>     use Linux SPI device driver\n");
    printf("               => default path: " LINUXDEV_PATH_DEFAULT "\n");
}

/* --- EOF ------------------------------------------------------------------ */
