/*
 / _____)             _              | |
( (____  _____ ____ _| |_ _____  ____| |__
 \____ \| ___ |    (_   _) ___ |/ ___)  _ \
 _____) ) ____| | | || |_| ____( (___| | | |
(______/|_____)_|_|_| \__)_____)\____)_| |_|
  (C)2018 Semtech

Description:
    Minimum test program for the loragw_reg 'library'

License: Revised BSD License, see LICENSE.TXT file include in the project
*/


/* -------------------------------------------------------------------------- */
/* --- DEPENDANCIES --------------------------------------------------------- */

#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "loragw_reg.h"
#include "loragw_aux.h"

/* -------------------------------------------------------------------------- */
/* --- PRIVATE MACROS ------------------------------------------------------- */

/* -------------------------------------------------------------------------- */
/* --- PRIVATE CONSTANTS ---------------------------------------------------- */

/* -------------------------------------------------------------------------- */
/* --- MAIN FUNCTION -------------------------------------------------------- */

int main()
{
    int x;
    int32_t val;

    x = lgw_connect();
    if (x != LGW_REG_SUCCESS) {
        printf("ERROR: failed to connect\n");
        return -1;
    }

    x = lgw_reg_r(LGW_AGCMCU__RF_EN_A__RADIO_EN, &val);
    if (x != LGW_REG_SUCCESS) {
        printf("ERROR: failed to read register\n");
        return -1;
    }
    printf("LGW_AGCMCU__RF_EN_A__RADIO_EN: 0x%02X\n", val);

    x = lgw_reg_r(LGW_AGCMCU__RF_EN_A__RADIO_RST, &val);
    if (x != LGW_REG_SUCCESS) {
        printf("ERROR: failed to read register\n");
        return -1;
    }
    printf("LGW_AGCMCU__RF_EN_A__RADIO_EN: 0x%02X\n", val);

    x  = lgw_reg_w(LGW_AGCMCU__RF_EN_A__RADIO_EN, 0x01);
    x |= lgw_reg_w(LGW_AGCMCU__RF_EN_A__RADIO_RST, 0x01);
    if (x != LGW_REG_SUCCESS) {
        printf("ERROR: failed to write registers\n");
        return -1;
    }

    x = lgw_reg_r(LGW_AGCMCU__RF_EN_A__RADIO_EN, &val);
    if (x != LGW_REG_SUCCESS) {
        printf("ERROR: failed to read register\n");
        return -1;
    }
    printf("LGW_AGCMCU__RF_EN_A__RADIO_EN: 0x%02X\n", val);

    x = lgw_reg_r(LGW_AGCMCU__RF_EN_A__RADIO_RST, &val);
    if (x != LGW_REG_SUCCESS) {
        printf("ERROR: failed to read register\n");
        return -1;
    }
    printf("LGW_AGCMCU__RF_EN_A__RADIO_EN: 0x%02X\n", val);

    x = lgw_disconnect();
    if (x != LGW_REG_SUCCESS) {
        printf("ERROR: failed to disconnect\n");
        return -1;
    }

    return 0;
}

/* --- EOF ------------------------------------------------------------------ */
