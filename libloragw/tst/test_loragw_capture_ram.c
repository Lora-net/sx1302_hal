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


// #include <stdint.h>
#include <stdio.h>      /* printf */
#include <stdlib.h>
#include <signal.h>     /* sigaction */
#include <getopt.h>     /* getopt_long */

#include "loragw_hal.h"
#include "loragw_reg.h"
#include "loragw_aux.h"
#include "loragw_sx1250.h"
#include "loragw_sx125x.h"
#include "loragw_sx1302.h"

/* -------------------------------------------------------------------------- */
/* --- PRIVATE MACROS ------------------------------------------------------- */

#define DEBUG_MSG(str)                fprintf(stderr, str)

#define LINUXDEV_PATH_DEFAULT "/dev/spidev0.0"

#define FULL_INIT 0
#define CAPTURE_RAM_SIZE 0x4000

/* -------------------------------------------------------------------------- */
/* --- PRIVATE VARIABLES ---------------------------------------------------- */

/* Signal handling variables */
static int exit_sig = 0; /* 1 -> application terminates cleanly (shut down hardware, close open files, etc) */
static int quit_sig = 0; /* 1 -> application terminates without shutting down the hardware */

uint32_t sampling_frequency[] = {4e6, 4e6, 4e6, 4e6, 4e6, 4e6, 4e6, 0, 0, 1e6, 125e3, 125e3, 125e3, 125e3, 125e3, 125e3, 125e3, 125e3, 8e6, 125e3, 125e3, 125e3, 0, 32e6, 32e6, 0, 32e6, 32e6, 0, 32e6, 32e6, 32e6};

#if FULL_INIT
#include "src/text_agc_sx1250_27_Nov_1.var"
#include "src/text_agc_sx1257_19_Nov_1.var"
#include "src/text_arb_sx1302_13_Nov_3.var"

#define FW_VERSION_CAL      0 /* Expected version of calibration firmware */ /* TODO */
#define FW_VERSION_AGC      1 /* Expected version of AGC firmware */
#define FW_VERSION_ARB      1 /* Expected version of arbiter firmware */

static bool rf_enable[LGW_RF_CHAIN_NB];
static uint32_t rf_rx_freq[LGW_RF_CHAIN_NB]; /* absolute, in Hz */
static lgw_radio_type_t rf_radio_type[LGW_RF_CHAIN_NB];
static uint8_t rf_clkout = 0;
#endif

/* -------------------------------------------------------------------------- */
/* --- PRIVATE FUNCTIONS ---------------------------------------------------- */

/* describe command line options */
void usage(void)
{
    printf("Available options:\n");
    printf(" -h print this help\n");
    printf(" -d <path> use Linux SPI device driver\n");
    printf("           => default path: " LINUXDEV_PATH_DEFAULT "\n");
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
#if FULL_INIT
    uint32_t val1, val2;
#endif
    uint8_t capture_ram_buffer[CAPTURE_RAM_SIZE];

    static struct sigaction sigact; /* SIGQUIT&SIGINT&SIGTERM signal handling */

    /* SPI interfaces */
    const char spidev_path_default[] = LINUXDEV_PATH_DEFAULT;
    const char * spidev_path = spidev_path_default;

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
                    spidev_path = optarg;
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

#if FULL_INIT
    /* Board reset */
    if (system("./reset_lgw.sh start") != 0) {
        printf("ERROR: failed to reset SX1302, check your reset_lgw.sh script\n");
        exit(EXIT_FAILURE);
    }
#endif

    /* Initialize memory for capture */
    for (i = 0; i < CAPTURE_RAM_SIZE; i++) {
        capture_ram_buffer[i] = i%256;
    }

    reg_stat = lgw_connect(spidev_path);
    if (reg_stat == LGW_REG_ERROR) {
        DEBUG_MSG("ERROR: FAIL TO CONNECT BOARD\n");
        return LGW_HAL_ERROR;
    }

    /* Manual init */
#if FULL_INIT
    rf_radio_type[0] = LGW_RADIO_TYPE_SX1250;
    rf_radio_type[1] = LGW_RADIO_TYPE_SX1257;
    rf_enable[0] = false;
    rf_enable[1] = true;
    rf_clkout = 1;
    rf_rx_freq[1] = 863700000;

    /* setup radios */
    for (i=0; i < 2; i++)
    {
        if (rf_enable[i] == true) {
            sx1302_radio_reset(i, rf_radio_type[i]);
            switch (radio_type) {
                case LGW_RADIO_TYPE_SX1250:
                    sx1250_setup(i, rf_rx_freq[i], false);
                    break;
                case LGW_RADIO_TYPE_SX1255:
                case LGW_RADIO_TYPE_SX1257:
                    sx125x_setup(i, rf_clkout, true, rf_radio_type[i], rf_rx_freq[i]);
                    break;
                default:
                    DEBUG_MSG("ERROR: RADIO TYPE NOT SUPPORTED\n");
                    return LGW_HAL_ERROR;
            }
            sx1302_radio_set_mode(i, radio_type);
        }
    }

    /* Select the radio which provides the clock to the sx1302 */
    sx1302_radio_clock_select(rf_clkout);

    /* Check that the SX1302 timestamp counter is running */
    lgw_get_instcnt(&val1);
    lgw_get_instcnt(&val2);
    if (val1 == val2) {
        printf("ERROR: SX1302 timestamp counter is not running (val:%u)\n", (uint32_t)val1);
        return -1;
    }

    /* Configure Radio FE */
    sx1302_radio_fe_configure();

    /* give radio control to AGC MCU */
    lgw_reg_w(SX1302_REG_COMMON_CTRL0_HOST_RADIO_CTRL, 0x00);

    /* Load firmware */
    switch (rf_radio_type[rf_clkout]) {
        case LGW_RADIO_TYPE_SX1250:
            printf("Loading AGC fw for sx1250\n");
            if (sx1302_agc_load_firmware(agc_firmware_sx1250) != LGW_HAL_SUCCESS) {
                return LGW_HAL_ERROR;
            }
            if (sx1302_agc_start(FW_VERSION_AGC, SX1302_RADIO_TYPE_SX1250, SX1302_AGC_RADIO_GAIN_AUTO, SX1302_AGC_RADIO_GAIN_AUTO, 0) != LGW_HAL_SUCCESS) {
                return LGW_HAL_ERROR;
            }
            break;
        case LGW_RADIO_TYPE_SX1257:
            printf("Loading AGC fw for sx125x\n");
            if (sx1302_agc_load_firmware(agc_firmware_sx125x) != LGW_HAL_SUCCESS) {
                return LGW_HAL_ERROR;
            }
            if (sx1302_agc_start(FW_VERSION_AGC, SX1302_RADIO_TYPE_SX125X, SX1302_AGC_RADIO_GAIN_AUTO, SX1302_AGC_RADIO_GAIN_AUTO, 0) != LGW_HAL_SUCCESS) {
            // if (sx1302_agc_start(FW_VERSION_AGC, SX1302_RADIO_TYPE_SX125X, 1, 7, 0) != LGW_HAL_SUCCESS) {
                return LGW_HAL_ERROR;
            }
            break;
        default:
            break;
    }
    printf("Loading ARB fw\n");
    if (sx1302_arb_load_firmware(arb_firmware) != LGW_HAL_SUCCESS) {
        return LGW_HAL_ERROR;
    }
    if (sx1302_arb_start(FW_VERSION_ARB) != LGW_HAL_SUCCESS) {
        return LGW_HAL_ERROR;
    }
#endif

    // lgw_reg_w(SX1302_REG_CAPTURE_RAM_CLOCK_GATE_OVERRIDE_CLK_OVERRIDE, 3);

    /* Configure the Capture Ram block */
    lgw_reg_w(SX1302_REG_CAPTURE_RAM_CAPTURE_CFG_ENABLE, 1);    /* Enable Capture RAM */
    lgw_reg_w(SX1302_REG_CAPTURE_RAM_CAPTURE_CFG_CAPTUREWRAP, 0);   /* Capture once, and stop when memory is full */
    lgw_reg_w(SX1302_REG_CAPTURE_RAM_CAPTURE_CFG_RAMCONFIG, 0);   /* RAM configuration, 0: 4kx32, 1: 2kx64 */
    fprintf(stdout, "Capture source: %d\n", capture_source);
    lgw_reg_w(SX1302_REG_CAPTURE_RAM_CAPTURE_SOURCE_A_SOURCEMUX, capture_source);

    printf("Sampling frequency: %d\n", sampling_frequency[capture_source]);
    if (sampling_frequency[capture_source] != 0)
    {
        period_value = (32e6/sampling_frequency[capture_source]) - 1;
    }
    else
    {
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
    do{
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
    for (i = 0; i < CAPTURE_RAM_SIZE; i += 4)
    {
        if (((capture_source >= 2) && (capture_source <= 3)) || (capture_source == 9))
        {
            real = (int16_t)((((uint16_t)(capture_ram_buffer[i+3]) << 8) & 0xFF00) + ((uint16_t)capture_ram_buffer[i+2] & 0x00FF));
            imag = (int16_t)((((uint16_t)(capture_ram_buffer[i+1]) << 8) & 0xFF00) + ((uint16_t)capture_ram_buffer[i+0] & 0x00FF));
            real >>= 4;   // 12 bits I
            imag >>= 4;   // 12 bits Q
        }
        else if ((capture_source >= 4) && (capture_source <= 6))
        {
            real = (int16_t)((((uint16_t)(capture_ram_buffer[i+3]) << 8) & 0xFF00) + ((uint16_t)capture_ram_buffer[i+2] & 0x00FF));    // 16 bits I
            imag = (int16_t)((((uint16_t)(capture_ram_buffer[i+1]) << 8) & 0xFF00) + ((uint16_t)capture_ram_buffer[i+0] & 0x00FF));    // 16 bits Q
        }
        else if ((capture_source >= 10) && (capture_source <= 17))
        {
            real = (int8_t)(capture_ram_buffer[i+3]);    // 8 bits I
            imag = (int8_t)(capture_ram_buffer[i+1]);    // 8 bits Q
        }
        else
        {
            real = 0;
            imag = 0;
        }

        if (((capture_source >= 2) && (capture_source <= 6)) || ((capture_source >= 9) && (capture_source <= 17)))
        {
            fprintf(stdout, "%d", real);
            if (imag >= 0)
            {
                fprintf(stdout, "+");
            }
            fprintf(stdout, "%di\n", imag);
        }
        else
        {
            printf("%02X ", capture_ram_buffer[i]);
        }
    }
    printf("End of Data\n");

#if FULL_INIT
    /* Board reset */
    if (system("./reset_lgw.sh stop") != 0) {
        printf("ERROR: failed to reset SX1302, check your reset_lgw.sh script\n");
        exit(EXIT_FAILURE);
    }
#endif

    return 0;
}
