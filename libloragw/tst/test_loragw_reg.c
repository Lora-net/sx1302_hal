/*
 / _____)             _              | |
( (____  _____ ____ _| |_ _____  ____| |__
 \____ \| ___ |    (_   _) ___ |/ ___)  _ \
 _____) ) ____| | | || |_| ____( (___| | | |
(______/|_____)_|_|_| \__)_____)\____)_| |_|
  (C)2019 Semtech

Description:
    Minimum test program for the loragw_reg module

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
#include <unistd.h>     /* getopt, access */
#include <math.h>

#include "loragw_com.h"
#include "loragw_reg.h"
#include "loragw_aux.h"
#include "loragw_hal.h"

/* -------------------------------------------------------------------------- */
/* --- PRIVATE MACROS ------------------------------------------------------- */

#define COM_TYPE_DEFAULT LGW_COM_SPI
#define COM_PATH_DEFAULT "/dev/spidev0.0"

/* -------------------------------------------------------------------------- */
/* --- PRIVATE CONSTANTS ---------------------------------------------------- */

extern const struct lgw_reg_s loregs[LGW_TOTALREGS+1];

/* -------------------------------------------------------------------------- */
/* --- SUBFUNCTIONS DECLARATION --------------------------------------------- */

static void usage(void);

/* -------------------------------------------------------------------------- */
/* --- MAIN FUNCTION -------------------------------------------------------- */

int main(int argc, char ** argv)
{
    int x, i;
    int32_t val;
    bool error_found = false;
    uint8_t rand_values[LGW_TOTALREGS];
    bool reg_ignored[LGW_TOTALREGS]; /* store register to be ignored */
    uint8_t reg_val;
    uint8_t reg_max;

    /* SPI interfaces */
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

            case 'u': /* Configure USB connection type */
                com_type = LGW_COM_USB;
                break;

            case 'd':
                if (optarg != NULL) {
                    com_path = optarg;
                }
                break;

            default:
                printf("ERROR: argument parsing options, use -h option for help\n");
                usage();
                return EXIT_FAILURE;
            }
    }

    if (com_type == LGW_COM_SPI) {
        /* Board reset */
        if (system("./reset_lgw.sh start") != 0) {
            printf("ERROR: failed to reset SX1302, check your reset_lgw.sh script\n");
            exit(EXIT_FAILURE);
        }
    }

    x = lgw_connect(com_type, com_path);
    if (x != LGW_REG_SUCCESS) {
        printf("ERROR: failed to connect\n");
        return -1;
    }

    /* The following registers cannot be tested this way */
    memset(reg_ignored, 0, sizeof reg_ignored);
    reg_ignored[SX1302_REG_COMMON_CTRL0_CLK32_RIF_CTRL] = true; /* all test fails if we set this one to 1 */

    /* Test 1: read all registers and check default value for non-read-only registers */
    printf("## TEST#1: read all registers and check default value for non-read-only registers\n");
    error_found = false;
    for (i = 0; i < LGW_TOTALREGS; i++) {
        if (loregs[i].rdon == 0) {
            x = lgw_reg_r(i, &val);
            if (x != LGW_REG_SUCCESS) {
                printf("ERROR: failed to read register at index %d\n", i);
                return -1;
            }
            if (val != loregs[i].dflt) {
                printf("ERROR: default value for register at index %d is %d, should be %d\n", i, val, loregs[i].dflt);
                error_found = true;
            }
        }
    }
    printf("------------------\n");
    printf(" TEST#1 %s\n", (error_found == false) ? "PASSED" : "FAILED");
    printf("------------------\n\n");

    /* Test 2: read/write test on all non-read-only, non-pulse, non-w0clr, non-w1clr registers */
    printf("## TEST#2: read/write test on all non-read-only, non-pulse, non-w0clr, non-w1clr registers\n");
    /* Write all registers with a random value */
    error_found = false;
    for (i = 0; i < LGW_TOTALREGS; i++) {
        if ((loregs[i].rdon == 0) && (reg_ignored[i] == false)) {
            /* Peek a random value different form the default reg value */
            reg_max = pow(2, loregs[i].leng) - 1;
            if (loregs[i].leng == 1) {
                reg_val = !loregs[i].dflt;
            } else {
                /* ensure random value is not the default one */
                do {
                    if (loregs[i].sign == 1) {
                        reg_val = rand() % (reg_max / 2);
                    } else {
                        reg_val = rand() % reg_max;
                    }
                } while (reg_val == loregs[i].dflt);
            }
            /* Write selected value */
            x = lgw_reg_w(i, reg_val);
            if (x != LGW_REG_SUCCESS) {
                printf("ERROR: failed to read register at index %d\n", i);
                return -1;
            }
            /* store value for later check */
            rand_values[i] = reg_val;
        }
    }
    /* Read all registers and check if we got proper random value back */
    for (i = 0; i < LGW_TOTALREGS; i++) {
        if ((loregs[i].rdon == 0) && (loregs[i].chck == 1) && (reg_ignored[i] == false)) {
            x = lgw_reg_r(i, &val);
            if (x != LGW_REG_SUCCESS) {
                printf("ERROR: failed to read register at index %d\n", i);
                return -1;
            }
            /* check value */
            if (val != rand_values[i]) {
                printf("ERROR: value read from register at index %d differs from the written value (w:%u r:%d)\n", i, rand_values[i], val);
                error_found = true;
            } else {
                //printf("INFO: MATCH reg %d (%u, %u)\n", i, rand_values[i], (uint8_t)val);
            }
        }
    }
    printf("------------------\n");
    printf(" TEST#2 %s\n", (error_found == false) ? "PASSED" : "FAILED");
    printf("------------------\n\n");

    x = lgw_disconnect();
    if (x != LGW_REG_SUCCESS) {
        printf("ERROR: failed to disconnect\n");
        return -1;
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

static void usage(void) {
    printf("~~~ Library version string~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~\n");
    printf(" %s\n", lgw_version_info());
    printf("~~~ Available options ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~\n");
    printf(" -h         print this help\n");
    printf(" -u         set COM type as USB (default is SPI)\n");
    printf(" -d <path>  COM path to be used to connect the concentrator\n");
    printf("            => default path: " COM_PATH_DEFAULT "\n");
}


/* --- EOF ------------------------------------------------------------------ */
