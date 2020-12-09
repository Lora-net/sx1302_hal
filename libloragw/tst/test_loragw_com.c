/*
 / _____)             _              | |
( (____  _____ ____ _| |_ _____  ____| |__
 \____ \| ___ |    (_   _) ___ |/ ___)  _ \
 _____) ) ____| | | || |_| ____( (___| | | |
(______/|_____)_|_|_| \__)_____)\____)_| |_|
  (C)2019 Semtech

Description:
    Minimum test program for the loragw_com module

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
#include <errno.h>

#include "loragw_com.h"
#include "loragw_aux.h"
#include "loragw_hal.h"

/* -------------------------------------------------------------------------- */
/* --- PRIVATE MACROS ------------------------------------------------------- */

#define RAND_RANGE(min, max) (rand() % (max + 1 - min) + min)

/* -------------------------------------------------------------------------- */
/* --- PRIVATE CONSTANTS ---------------------------------------------------- */

#define BUFF_SIZE_SPI       1024
#define BUFF_SIZE_USB       4096

#define SX1302_AGC_MCU_MEM  0x0000
#define SX1302_REG_COMMON   0x5600
#define SX1302_REG_AGC_MCU  0x5780

#define COM_TYPE_DEFAULT LGW_COM_SPI
#define COM_PATH_DEFAULT "/dev/spidev0.0"

/* -------------------------------------------------------------------------- */
/* --- GLOBAL VARIABLES ----------------------------------------------------- */

/* Signal handling variables */
static int exit_sig = 0; /* 1 -> application terminates cleanly (shut down hardware, close open files, etc) */
static int quit_sig = 0; /* 1 -> application terminates without shutting down the hardware */

/* Buffers */
static uint8_t * test_buff = NULL;
static uint8_t * read_buff = NULL;

/* -------------------------------------------------------------------------- */
/* --- SUBFUNCTIONS DECLARATION --------------------------------------------- */

static void sig_handler(int sigio);
static void usage(void);
static void exit_failure(void);

/* -------------------------------------------------------------------------- */
/* --- MAIN FUNCTION -------------------------------------------------------- */

int main(int argc, char ** argv)
{
    static struct sigaction sigact; /* SIGQUIT&SIGINT&SIGTERM signal handling */

    uint16_t max_buff_size;
    uint8_t data = 0;
    int cycle_number = 0;
    int i, x;
    uint16_t size;

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

    printf("Beginning of test for loragw_com.c\n");
    x = lgw_com_open(com_type, com_path);
    if (x != 0) {
        printf("ERROR: failed to open COM device %s\n", com_path);
        exit(EXIT_FAILURE);
    }

    /* normal R/W test */
    /* TODO */

    /* burst R/W test, small bursts << LGW_BURST_CHUNK */
    /* TODO */

    /* burst R/W test, large bursts >> LGW_BURST_CHUNK */
    /* TODO */

    x = lgw_com_r(LGW_SPI_MUX_TARGET_SX1302, SX1302_REG_COMMON + 6, &data);
    if (x != 0) {
        printf("ERROR (%d): failed to read register\n", __LINE__);
        exit_failure();
    }
    printf("SX1302 version: 0x%02X\n", data);

    x = lgw_com_r(LGW_SPI_MUX_TARGET_SX1302, SX1302_REG_AGC_MCU + 0, &data);
    if (x != 0) {
        printf("ERROR (%d): failed to read register\n", __LINE__);
        exit_failure();
    }
    x = lgw_com_w(LGW_SPI_MUX_TARGET_SX1302, SX1302_REG_AGC_MCU + 0, 0x06); /* mcu_clear, host_prog */
    if (x != 0) {
        printf("ERROR (%d): failed to write register\n", __LINE__);
        exit_failure();
    }

    srand(time(NULL));

    /* Allocate buffers according to com type capabilities */
    max_buff_size = (com_type == LGW_COM_SPI) ? BUFF_SIZE_SPI : BUFF_SIZE_USB;
    test_buff = (uint8_t*)malloc(max_buff_size * sizeof(uint8_t));
    if (test_buff == NULL) {
        printf("ERROR: failed to allocate memory for test_buff - %s\n", strerror(errno));
        exit_failure();
    }
    read_buff = (uint8_t*)malloc(max_buff_size * sizeof(uint8_t));
    if (read_buff == NULL) {
        printf("ERROR: failed to allocate memory for read_buff - %s\n", strerror(errno));
        exit_failure();
    }

    /* databuffer R/W stress test */
    while ((quit_sig != 1) && (exit_sig != 1)) {
        /*************************************************
         *
         *      WRITE BURST TEST
         *
         * ***********************************************/

        size = rand() % max_buff_size;
        for (i = 0; i < size; ++i) {
            test_buff[i] = rand() & 0xFF;
        }
        printf("Cycle %i> ", cycle_number);

        /* Write burst with random data */
        x = lgw_com_wb(LGW_SPI_MUX_TARGET_SX1302, SX1302_AGC_MCU_MEM, test_buff, size);
        if (x != 0) {
            printf("ERROR (%d): failed to write burst\n", __LINE__);
            exit_failure();
        }

        /* Read back */
        x = lgw_com_rb(LGW_SPI_MUX_TARGET_SX1302, SX1302_AGC_MCU_MEM, read_buff, size);
        if (x != 0) {
            printf("ERROR (%d): failed to read burst\n", __LINE__);
            exit_failure();
        }

        /* Compare read / write buffers */
        for (i=0; ((i<size) && (test_buff[i] == read_buff[i])); ++i);
        if (i != size) {
            printf("error during the buffer comparison\n");

            /* Print what has been written */
            printf("Written values:\n");
            for (i=0; i<size; ++i) {
                printf(" %02X ", test_buff[i]);
                if (i%16 == 15) printf("\n");
            }
            printf("\n");

            /* Print what has been read back */
            printf("Read values:\n");
            for (i=0; i<size; ++i) {
                printf(" %02X ", read_buff[i]);
                if (i%16 == 15) printf("\n");
            }
            printf("\n");

            /* exit */
            exit_failure();
        } else {
            printf("did a %i-byte R/W on a data buffer with no error\n", size);
            ++cycle_number;
        }

        /*************************************************
         *
         *      WRITE SINGLE BYTE TEST
         *
         * ***********************************************/

        /* Single byte r/w test */
        printf("Cycle %i> ", cycle_number);

        test_buff[0] = rand() & 0xFF;

        /* Write single byte */
        x = lgw_com_w(LGW_SPI_MUX_TARGET_SX1302, SX1302_AGC_MCU_MEM, test_buff[0]);
        if (x != 0) {
            printf("ERROR (%d): failed to write burst\n", __LINE__);
            exit_failure();
        }

        /* Read back */
        x = lgw_com_r(LGW_SPI_MUX_TARGET_SX1302, SX1302_AGC_MCU_MEM, &read_buff[0]);
        if (x != 0) {
            printf("ERROR (%d): failed to read burst\n", __LINE__);
            exit_failure();
        }

        /* Compare read / write bytes */
        if (test_buff[0] != read_buff[0]) {
            printf("error during the byte comparison\n");

            /* Print what has been written */
            printf("Written value: %02X\n", test_buff[0]);

            /* Print what has been read back */
            printf("Read values: %02X\n", read_buff[0]);

            /* exit */
            exit_failure();
        } else {
            printf("did a 1-byte R/W on a data buffer with no error\n");
            ++cycle_number;
        }

        /*************************************************
         *
         *      WRITE WITH BULK (USB only mode)
         *
         * ***********************************************/
        x = lgw_com_set_write_mode(LGW_COM_WRITE_MODE_BULK);
        if (x != 0) {
            printf("ERROR (%d): failed to set bulk write mode\n", __LINE__);
            exit_failure();
        }

        uint16_t num_req = RAND_RANGE(1, 254); /* keep one req for remaining bytes */
        size = RAND_RANGE(num_req, max_buff_size / 2); /* TODO: test proper limit */
        for (i = 0; i < size; i++) {
            test_buff[i] = rand() & 0xFF;
        }
        uint16_t size_per_req = size / num_req;
        uint16_t size_remaining = size - (num_req * size_per_req);
        printf("Cycle %i> ", cycle_number);

        uint16_t size_written = 0;
        for (i = 0; i < num_req; i++) {
            x = lgw_com_wb(LGW_SPI_MUX_TARGET_SX1302, SX1302_AGC_MCU_MEM + size_written, test_buff + size_written, size_per_req);
            if (x != 0) {
                printf("ERROR (%d): failed to write burst\n", __LINE__);
                exit_failure();
            }
            size_written += (size_per_req);
        }
        if (size_remaining > 0) {
            x = lgw_com_wb(LGW_SPI_MUX_TARGET_SX1302, SX1302_AGC_MCU_MEM + size_written, test_buff + size_written, size_remaining);
            if (x != 0) {
                printf("ERROR (%d): failed to write burst\n", __LINE__);
                exit_failure();
            }
        }

        /* Send data to MCU (UBS mode only) */
        x = lgw_com_flush();
        if (x != 0) {
            printf("ERROR (%d): failed to flush write\n", __LINE__);
            exit_failure();
        }

        /* Read back */
        x = lgw_com_rb(LGW_SPI_MUX_TARGET_SX1302, SX1302_AGC_MCU_MEM, read_buff, size);
        if (x != 0) {
            printf("ERROR (%d): failed to read burst\n", __LINE__);
            exit_failure();
        }

        /* Compare read / write buffers */
        for (i=0; ((i<size) && (test_buff[i] == read_buff[i])); ++i);
        if (i != size) {
            printf("error during the buffer comparison\n");

            /* Print what has been written */
            printf("Written values:\n");
            for (i=0; i<size; ++i) {
                printf(" %02X ", test_buff[i]);
                if (i%16 == 15) printf("\n");
            }
            printf("\n");

            /* Print what has been read back */
            printf("Read values:\n");
            for (i=0; i<size; ++i) {
                printf(" %02X ", read_buff[i]);
                if (i%16 == 15) printf("\n");
            }
            printf("\n");

            /* exit */
            exit_failure();
        } else {
            printf("did a %i-byte bulk R/W on a data buffer with no error\n", size);
            ++cycle_number;
        }
    }

    lgw_com_close();
    printf("End of test for loragw_com.c\n");

    /* deallocate buffers */
    if (test_buff != NULL) {
        free(test_buff);
        test_buff = NULL;
    }
    if (read_buff != NULL) {
        free(read_buff);
        read_buff = NULL;
    }

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

static void exit_failure(void) {
    lgw_com_close();
    printf("End of test for loragw_com.c\n");

    /* deallocate buffers */
    if (test_buff != NULL) {
        free(test_buff);
        test_buff = NULL;
    }
    if (read_buff != NULL) {
        free(read_buff);
        read_buff = NULL;
    }

    exit(EXIT_FAILURE);
}

/* ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ */

static void usage(void) {
    printf("~~~ Library version string~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~\n");
    printf(" %s\n", lgw_version_info());
    printf("~~~ Available options ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~\n");
    printf(" -h            print this help\n");
    printf(" -u            set COM type as USB (default is SPI)\n");
    printf(" -d <path>     COM path to be used to connect the concentrator\n");
    printf("               => default path (SPI): " COM_PATH_DEFAULT "\n");
}

/* --- EOF ------------------------------------------------------------------ */
