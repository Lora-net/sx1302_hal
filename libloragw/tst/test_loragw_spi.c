/*
 / _____)             _              | |
( (____  _____ ____ _| |_ _____  ____| |__
 \____ \| ___ |    (_   _) ___ |/ ___)  _ \
 _____) ) ____| | | || |_| ____( (___| | | |
(______/|_____)_|_|_| \__)_____)\____)_| |_|
  (C)2019 Semtech

Description:
    Minimum test program for the loragw_spi module

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
#include <time.h>

#include "loragw_spi.h"
#include "loragw_aux.h"
#include "loragw_hal.h"

/* -------------------------------------------------------------------------- */
/* --- PRIVATE MACROS ------------------------------------------------------- */

/* -------------------------------------------------------------------------- */
/* --- PRIVATE CONSTANTS ---------------------------------------------------- */

#define BUFF_SIZE           1024

#define SX1302_AGC_MCU_MEM  0x0000
#define SX1302_REG_COMMON   0x5600
#define SX1302_REG_AGC_MCU  0x5780

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

    uint8_t data = 0;
    uint8_t test_buff[BUFF_SIZE];
    uint8_t read_buff[BUFF_SIZE];
    int cycle_number = 0;
    int i;
    uint16_t size;

    /* SPI interfaces */
    const char spidev_path_default[] = LINUXDEV_PATH_DEFAULT;
    const char * spidev_path = spidev_path_default;
    void *spi_target = NULL;

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

    printf("Beginning of test for loragw_spi.c\n");
    i = lgw_spi_open(spidev_path, &spi_target);
    if (i != 0) {
        printf("ERROR: failed to open SPI device %s\n", spidev_path);
        return -1;
    }

    /* normal R/W test */
    /* TODO */

    /* burst R/W test, small bursts << LGW_BURST_CHUNK */
    /* TODO */

    /* burst R/W test, large bursts >> LGW_BURST_CHUNK */
    /* TODO */

    lgw_spi_r(spi_target, LGW_SPI_MUX_TARGET_SX1302, SX1302_REG_COMMON + 6, &data);
    printf("SX1302 version: 0x%02X\n", data);

    lgw_spi_r(spi_target, LGW_SPI_MUX_TARGET_SX1302, SX1302_REG_AGC_MCU + 0, &data);
    lgw_spi_w(spi_target, LGW_SPI_MUX_TARGET_SX1302, SX1302_REG_AGC_MCU + 0, 0x06); /* mcu_clear, host_prog */

    srand(time(NULL));

    /* databuffer R/W stress test */
    while ((quit_sig != 1) && (exit_sig != 1)) {
        size = rand() % BUFF_SIZE;
        for (i = 0; i < size; ++i) {
            test_buff[i] = rand() & 0xFF;
        }
        printf("Cycle %i > ", cycle_number);
        lgw_spi_wb(spi_target, LGW_SPI_MUX_TARGET_SX1302, SX1302_AGC_MCU_MEM, test_buff, size);
        lgw_spi_rb(spi_target, LGW_SPI_MUX_TARGET_SX1302, SX1302_AGC_MCU_MEM, read_buff, size);
        for (i=0; ((i<size) && (test_buff[i] == read_buff[i])); ++i);
        if (i != size) {
            printf("error during the buffer comparison\n");
            printf("Written values:\n");
            for (i=0; i<size; ++i) {
                printf(" %02X ", test_buff[i]);
                if (i%16 == 15) printf("\n");
            }
            printf("\n");
            printf("Read values:\n");
            for (i=0; i<size; ++i) {
                printf(" %02X ", read_buff[i]);
                if (i%16 == 15) printf("\n");
            }
            printf("\n");
            return EXIT_FAILURE;
        } else {
            printf("did a %i-byte R/W on a data buffer with no error\n", size);
            ++cycle_number;
        }
    }

    lgw_spi_close(spi_target);
    printf("End of test for loragw_spi.c\n");

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
