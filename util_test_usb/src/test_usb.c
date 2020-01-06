/*
  ______                              _
 / _____)             _              | |
( (____  _____ ____ _| |_ _____  ____| |__
 \____ \| ___ |    (_   _) ___ |/ ___)  _ \
 _____) ) ____| | | || |_| ____( (___| | | |
(______/|_____)_|_|_| \__)_____)\____)_| |_|
  (C)2019 Semtech

Description:
    Utility to perform basic USB communication with SX1302 CoreCell

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
#include <inttypes.h>   /* PRIx64, PRIu64... */
#include <string.h>
#include <unistd.h>
#include <math.h>
#include <signal.h>     /* sigaction */
#include <getopt.h>     /* getopt_long */

#include "loragw_hal.h"
#include "loragw_reg.h"
#include "loragw_aux.h"
#include "loragw_mcu.h"

/* -------------------------------------------------------------------------- */
/* --- PRIVATE MACROS ------------------------------------------------------- */

#define RAND_RANGE(min, max) (rand() % (max + 1 - min) + min)

/* -------------------------------------------------------------------------- */
/* --- PRIVATE CONSTANTS ---------------------------------------------------- */

#define LINUXDEV_PATH_DEFAULT "/dev/spidev0.0"

#define DEFAULT_CLK_SRC     0
#define DEFAULT_FREQ_HZ     868500000U

#define HEADER_CMD_SIZE  4
#define WRITE_SIZE_MAX 280
#define READ_SIZE_MAX 500
/* -------------------------------------------------------------------------- */
/* --- PRIVATE VARIABLES ---------------------------------------------------- */


/* -------------------------------------------------------------------------- */
/* --- PRIVATE FUNCTIONS ---------------------------------------------------- */

/* describe command line options */
void usage(void) {
    printf("Library version information: %s\n", lgw_version_info());
    printf("Available options:\n");
    printf(" -h print this help\n");
    printf(" -d [path] Path the spidev file (ex: /dev/spidev0.0)\n");
}

static int  mcu_fd;
/* -------------------------------------------------------------------------- */
/* --- MAIN FUNCTION -------------------------------------------------------- */

int main(int argc, char **argv)
{
    int i;


    /* SPI interfaces */
    const char com_path_default[] = LINUXDEV_PATH_DEFAULT;
    const char * com_path = com_path_default;

    /* Parameter parsing */
    int option_index = 0;
    static struct option long_options[] = {
        {0, 0, 0, 0}
    };

    /* parse command line options */
    while ((i = getopt_long (argc, argv, "hd:k:r:", long_options, &option_index)) != -1) {
        switch (i) {
            case 'h':
                usage();
                return -1;
                break;

            case 'd':
                com_path = optarg;
                break;

            default:
                printf("ERROR: argument parsing\n");
                usage();
                return -1;
        }
    }

    s_ping_info gw_info;
    mcu_fd = mcu_open(com_path);
    if (mcu_fd == -1) {
        return -1;
    }

    if (mcu_ping(mcu_fd, &gw_info) != 0) {
        return -1;
    }
    uint8_t tx_buffer [256];
    uint8_t rx_buffer [256];
    int k;
    tx_buffer[0] = 0x00; //Target SPI SX1302-1250
    tx_buffer[1] = 0x00; // Target SX1302
    tx_buffer[2] = 0x56;
    tx_buffer[3] = 0x06;
    tx_buffer[4] = 0x00;
    tx_buffer[5] = 0x00;


    mcu_spi_access(mcu_fd,tx_buffer,6,rx_buffer);

    for (k = 0 ; k < 10 ; k++) {
        printf("0x%x ", rx_buffer[k]);
    }
    printf("\n");



    tx_buffer[0] = 0x00; //Target SPI SX1302-1250
    tx_buffer[1] = 0x00; // Target SX1302
    tx_buffer[2] = 0x57 ;
    tx_buffer[3] = 0x89;
    tx_buffer[4] = 0x00;
    tx_buffer[5] = 0x00;


    mcu_spi_access(mcu_fd,tx_buffer,6,rx_buffer);

    for (k = 0 ; k < 10 ; k++) {
        printf("0x%x ", rx_buffer[k]);
    }
    printf("\n");


    tx_buffer[0] = 0x00; //Target SPI SX1302-1250
    tx_buffer[1] = 0x00; // Target SX1302
    tx_buffer[2] = 0x80+0x57 ;
    tx_buffer[3] = 0x89;
    tx_buffer[4] = 0x55;


    mcu_spi_access(mcu_fd,tx_buffer,5,rx_buffer);

    for (k = 0 ; k < 10 ; k++) {
        printf("0x%x ", rx_buffer[k]);
    }
    printf("\n");
    
    tx_buffer[0] = 0x00; //Target SPI SX1302-1250
    tx_buffer[1] = 0x00; // Target SX1302
    tx_buffer[2] = 0x57 ;
    tx_buffer[3] = 0x89;
    tx_buffer[4] = 0x00;
    tx_buffer[5] = 0x00;


    mcu_spi_access(mcu_fd,tx_buffer,6,rx_buffer);

    for (k = 0 ; k < 10 ; k++) {
        printf("0x%x ", rx_buffer[k]);
    }
    printf("\n");
    return 0;
}




/* --- EOF ------------------------------------------------------------------ */
