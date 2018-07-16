/*
 / _____)             _              | |
( (____  _____ ____ _| |_ _____  ____| |__
 \____ \| ___ |    (_   _) ___ |/ ___)  _ \
 _____) ) ____| | | || |_| ____( (___| | | |
(______/|_____)_|_|_| \__)_____)\____)_| |_|
  (C)2018 Semtech

Description:
    Minimum test program for the loragw_spi 'library'

License: Revised BSD License, see LICENSE.TXT file include in the project
*/


/* -------------------------------------------------------------------------- */
/* --- DEPENDANCIES --------------------------------------------------------- */

#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "loragw_reg.h"
#include "loragw_sx1262fe.h"
#include "loragw_aux.h"

/* -------------------------------------------------------------------------- */
/* --- PRIVATE MACROS ------------------------------------------------------- */

/* -------------------------------------------------------------------------- */
/* --- PRIVATE CONSTANTS ---------------------------------------------------- */

#define BUFF_SIZE           1024

/* -------------------------------------------------------------------------- */
/* --- MAIN FUNCTION -------------------------------------------------------- */

int main()
{
    uint8_t buff[16];

    printf("===== sx1302 sx1262fe TX test =====\n");
    lgw_connect();

    printf("Setup SX126x in Tx mode\n");

    lgw_reg_w(SX1302_REG_AGC_MCU_RF_EN_A_RADIO_EN, 0x01);
    lgw_reg_w(SX1302_REG_AGC_MCU_RF_EN_A_RADIO_RST, 0x01);

    buff[0] = 0x00;
    sx1262fe_write_command(0x80, buff, 1); /* SetStandby STDBY_RC */

    buff[0] = 0x00;
    sx1262fe_read_command(0xC0, buff, 1); /* GetStatus */
    printf("SX1262FE status: 0x%02X\n", buff[0]);

    buff[0] = 0x00;
    sx1262fe_write_command(0x8A, buff, 1); /* SetPacketType FSK */

    buff[0] = 0x36;
    buff[1] = 0x48;
    buff[2] = 0x00;
    buff[3] = 0x00;
    sx1262fe_write_command(0x86, buff, 4); /* SetRfFrequency */

    buff[0] = 0x0E;
    buff[1] = 0x02;
    sx1262fe_write_command(0x8E, buff, 2); /* SetTxParams (power, RAMP_40U) */

    buff[0] = 0x04;
    buff[1] = 0x07;
    buff[2] = 0x00;
    buff[3] = 0x01;
    sx1262fe_write_command(0x95, buff, 4); /* SetPaOpt */

    buff[0] = 0x00;
    sx1262fe_write_command(0xD1, buff, 1); /* SetTxContinuousWave */

    buff[0] = 0x00;
    sx1262fe_read_command(0xC0, buff, 1); /* GetStatus */
    printf("SX1262FE status: 0x%02X\n", buff[0]);

    lgw_disconnect();
    printf("=========== Test End ===========\n");

    return 0;
}

/* --- EOF ------------------------------------------------------------------ */
