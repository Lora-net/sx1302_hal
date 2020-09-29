/*
 / _____)             _              | |
( (____  _____ ____ _| |_ _____  ____| |__
 \____ \| ___ |    (_   _) ___ |/ ___)  _ \
 _____) ) ____| | | || |_| ____( (___| | | |
(______/|_____)_|_|_| \__)_____)\____)_| |_|
  (C)2019 Semtech

Description:
    Minimum test program to test the capture RAM block

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

#include <stdio.h>      /* printf */
#include <stdlib.h>
#include <signal.h>     /* sigaction */
#include <getopt.h>     /* getopt_long */

#include "loragw_hal.h"
#include "loragw_com.h"
#include "loragw_reg.h"
#include "loragw_aux.h"
#include "loragw_sx1250.h"
#include "loragw_sx125x.h"
#include "loragw_sx1302.h"

/* -------------------------------------------------------------------------- */
/* --- PRIVATE MACROS ------------------------------------------------------- */

#define DEBUG_MSG(str) fprintf(stdout, str)

#define COM_TYPE_DEFAULT LGW_COM_SPI
#define COM_PATH_DEFAULT "/dev/spidev0.0"

#define CAPTURE_RAM_SIZE 0x4000

/* -------------------------------------------------------------------------- */
/* --- PRIVATE VARIABLES ---------------------------------------------------- */

/* Signal handling variables */
static int exit_sig = 0; /* 1 -> application terminates cleanly (shut down hardware, close open files, etc) */
static int quit_sig = 0; /* 1 -> application terminates without shutting down the hardware */

uint32_t sampling_frequency[] = {4e6, 4e6, 4e6, 4e6, 4e6, 4e6, 4e6, 0, 0, 1e6, 125e3, 125e3, 125e3, 125e3, 125e3, 125e3, 125e3, 125e3, 8e6, 125e3, 125e3, 125e3, 0, 32e6, 32e6, 0, 32e6, 32e6, 0, 32e6, 32e6, 32e6};

/* -------------------------------------------------------------------------- */
/* --- PRIVATE FUNCTIONS ---------------------------------------------------- */

/* describe command line options */
void usage(void)
{
    printf("Available options:\n");
    printf(" -h print this help\n");
    printf(" -d [path] Path to the SPI interface (USB is not supported)\n");
    printf("            => default path: " COM_PATH_DEFAULT "\n");
    printf(" -s <uint> Capture source [0..31]\n");
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

/* Main program */
int main(int argc, char **argv)
{
    int i;
    int32_t val = 0;
    int reg_stat;
    unsigned int arg_u;
    uint8_t capture_source = 0;
    uint16_t period_value = 0;
    int16_t real = 0, imag = 0;
    uint8_t capture_ram_buffer[CAPTURE_RAM_SIZE];

    static struct sigaction sigact; /* SIGQUIT&SIGINT&SIGTERM signal handling */

    /* SPI interfaces */
    const char com_path_default[] = COM_PATH_DEFAULT;
    const char * com_path = com_path_default;
    lgw_com_type_t com_type = COM_TYPE_DEFAULT;

    /* Parameter parsing */
    int option_index = 0;
    static struct option long_options[] = {
        {0, 0, 0, 0}
    };

    /* parse command line options */
    while ((i = getopt_long (argc, argv, "h:s:d:", long_options, &option_index)) != -1) {
        switch (i) {
            case 'h':
                usage();
                return -1;
                break;

            case 'd':
                if (optarg != NULL) {
                    com_path = optarg;
                }
                break;

            case 's': /* <uint> Capture Source */
                i = sscanf(optarg, "%u", &arg_u);
                if ((i != 1) || (arg_u > 31)) {
                    printf("ERROR: argument parsing of -s argument. Use -h to print help\n");
                    return EXIT_FAILURE;
                } else {
                    capture_source = arg_u;
                }
                break;

            default:
                printf("ERROR: argument parsing\n");
                usage();
                return -1;
        }
    }

    /* Configure signal handling */
    sigemptyset( &sigact.sa_mask );
    sigact.sa_flags = 0;
    sigact.sa_handler = sig_handler;
    sigaction( SIGQUIT, &sigact, NULL );
    sigaction( SIGINT, &sigact, NULL );
    sigaction( SIGTERM, &sigact, NULL );

    /* Initialize memory for capture */
    for (i = 0; i < CAPTURE_RAM_SIZE; i++) {
        capture_ram_buffer[i] = i%256;
    }

    reg_stat = lgw_connect(com_type, com_path);
    if (reg_stat == LGW_REG_ERROR) {
        DEBUG_MSG("ERROR: FAIL TO CONNECT BOARD\n");
        return LGW_HAL_ERROR;
    }

    // lgw_reg_w(SX1302_REG_CAPTURE_RAM_CLOCK_GATE_OVERRIDE_CLK_OVERRIDE, 3);

    /* Configure the Capture Ram block */
    lgw_reg_w(SX1302_REG_CAPTURE_RAM_CAPTURE_CFG_ENABLE, 1);    /* Enable Capture RAM */
    lgw_reg_w(SX1302_REG_CAPTURE_RAM_CAPTURE_CFG_CAPTUREWRAP, 0);   /* Capture once, and stop when memory is full */
    lgw_reg_w(SX1302_REG_CAPTURE_RAM_CAPTURE_CFG_RAMCONFIG, 0);   /* RAM configuration, 0: 4kx32, 1: 2kx64 */
    fprintf(stdout, "Capture source: %d\n", capture_source);
    lgw_reg_w(SX1302_REG_CAPTURE_RAM_CAPTURE_SOURCE_A_SOURCEMUX, capture_source);

    printf("Sampling frequency: %d\n", sampling_frequency[capture_source]);
    if (sampling_frequency[capture_source] != 0) {
        period_value = (32e6/sampling_frequency[capture_source]) - 1;
    } else {
        fprintf(stderr ,"ERROR: Sampling frequency is null\n");
        return -1;
    }

    // fprintf(stdout, "period_value=%04X\n", period_value);
    lgw_reg_w(SX1302_REG_CAPTURE_RAM_CAPTURE_PERIOD_0_CAPTUREPERIOD, period_value & 0xFF);  // LSB
    lgw_reg_w(SX1302_REG_CAPTURE_RAM_CAPTURE_PERIOD_1_CAPTUREPERIOD, (period_value>>8) & 0xFF); // MSB

    /* Read back registers */
    // lgw_reg_r(SX1302_REG_CAPTURE_RAM_CAPTURE_PERIOD_0_CAPTUREPERIOD, &val);
    // fprintf(stdout, "SX1302_REG_CAPTURE_RAM_CAPTURE_PERIOD_0_CAPTUREPERIOD value: %d\n", val);
    // lgw_reg_r(SX1302_REG_CAPTURE_RAM_CAPTURE_PERIOD_1_CAPTUREPERIOD, &val);
    // fprintf(stdout, "SX1302_REG_CAPTURE_RAM_CAPTURE_PERIOD_1_CAPTUREPERIOD value: %d\n", val);

    /* Launch capture */
    lgw_reg_w(SX1302_REG_CAPTURE_RAM_CAPTURE_CFG_CAPTURESTART, 1);
    // lgw_reg_w(SX1302_REG_CAPTURE_RAM_CAPTURE_CFG_CAPTUREFORCETRIGGER, 1);

    /* Poll Status.CapComplete */
    do {
        lgw_reg_r(SX1302_REG_CAPTURE_RAM_STATUS_CAPCOMPLETE, &val);

        wait_ms(10);
        if ((quit_sig == 1) || (exit_sig == 1)) {
            break;
        }
    } while (val != 1);
    lgw_reg_w(SX1302_REG_CAPTURE_RAM_CAPTURE_CFG_CAPTURESTART, 0);

    // lgw_reg_r(SX1302_REG_CAPTURE_RAM_LAST_RAM_ADDR_0_LASTRAMADDR, &val);
    // fprintf(stdout, "SX1302_REG_CAPTURE_RAM_LAST_RAM_ADDR_0_LASTRAMADDR value: %02x\n", val);
    // lgw_reg_r(SX1302_REG_CAPTURE_RAM_LAST_RAM_ADDR_1_LASTRAMADDR, &val);
    // fprintf(stdout, "SX1302_REG_CAPTURE_RAM_LAST_RAM_ADDR_1_LASTRAMADDR value: %02x\n", val);

    lgw_reg_w(SX1302_REG_COMMON_PAGE_PAGE, 1);
    lgw_mem_rb(0, capture_ram_buffer, CAPTURE_RAM_SIZE, false);
    lgw_reg_w(SX1302_REG_COMMON_PAGE_PAGE, 0);

    printf("Data:\n");
    for (i = 0; i < CAPTURE_RAM_SIZE; i += 4) {
        if (((capture_source >= 2) && (capture_source <= 3)) || (capture_source == 9)) {
            real = (int16_t)((((uint16_t)(capture_ram_buffer[i+3]) << 8) & 0xFF00) + ((uint16_t)capture_ram_buffer[i+2] & 0x00FF));
            imag = (int16_t)((((uint16_t)(capture_ram_buffer[i+1]) << 8) & 0xFF00) + ((uint16_t)capture_ram_buffer[i+0] & 0x00FF));
            real >>= 4; // 12 bits I
            imag >>= 4; // 12 bits Q
        } else if ((capture_source >= 4) && (capture_source <= 6)) {
            real = (int16_t)((((uint16_t)(capture_ram_buffer[i+3]) << 8) & 0xFF00) + ((uint16_t)capture_ram_buffer[i+2] & 0x00FF)); // 16 bits I
            imag = (int16_t)((((uint16_t)(capture_ram_buffer[i+1]) << 8) & 0xFF00) + ((uint16_t)capture_ram_buffer[i+0] & 0x00FF)); // 16 bits Q
        } else if ((capture_source >= 10) && (capture_source <= 17)) {
            real = (int8_t)(capture_ram_buffer[i+3]); // 8 bits I
            imag = (int8_t)(capture_ram_buffer[i+1]); // 8 bits Q
        } else {
            real = 0;
            imag = 0;
        }

        if (((capture_source >= 2) && (capture_source <= 6)) || ((capture_source >= 9) && (capture_source <= 17))) {
            fprintf(stdout, "%d", real);
            if (imag >= 0) {
                fprintf(stdout, "+");
            }
            fprintf(stdout, "%di\n", imag);
        } else {
            printf("%02X ", capture_ram_buffer[i]);
        }
    }
    printf("End of Data\n");

    return 0;
}
