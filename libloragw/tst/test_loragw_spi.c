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

#include "loragw_spi.h"
#include "loragw_aux.h"

/* -------------------------------------------------------------------------- */
/* --- PRIVATE MACROS ------------------------------------------------------- */

/* -------------------------------------------------------------------------- */
/* --- PRIVATE CONSTANTS ---------------------------------------------------- */

#define BUFF_SIZE           1024

#define SX1302_AGC_MCU_MEM  0x0000
#define SX1302_REG_COMMON   0x5600
#define SX1302_REG_AGC_MCU  0x5780

/* -------------------------------------------------------------------------- */
/* --- MAIN FUNCTION -------------------------------------------------------- */

int main()
{
    void *spi_target = NULL;
    uint8_t data = 0;
    uint8_t buff[16];
    uint8_t test_buff[BUFF_SIZE];
    uint8_t read_buff[BUFF_SIZE];
    int cycle_number = 0;
    int i;

    printf("Beginning of test for loragw_spi.c\n");
    lgw_spi_open(&spi_target);

    /* normal R/W test */
    /* TODO */

    /* burst R/W test, small bursts << LGW_BURST_CHUNK */
    /* TODO */

    /* burst R/W test, large bursts >> LGW_BURST_CHUNK */
    /* TODO */

    lgw_spi_r(spi_target, LGW_SPI_MUX_TARGET_SX1302, SX1302_REG_COMMON + 5, &data);
    printf("SX1302 version: 0x%02X\n", data);

#if 1
    lgw_spi_r(spi_target, LGW_SPI_MUX_TARGET_SX1302, SX1302_REG_AGC_MCU + 0, &data);
    printf("agc_mcu.ctrl=0x%02X\n", data);
    lgw_spi_w(spi_target, LGW_SPI_MUX_TARGET_SX1302, SX1302_REG_AGC_MCU + 0, 0x06); /* mcu_clear, host_prog */

    /* databuffer R/W stress test */
    while (1) { //(quit_sig != 1) && (exit_sig != 1)) {
        for (i=0; i<BUFF_SIZE; ++i) {
            test_buff[i] = rand() & 0xFF;
        }
        printf("Cycle %i > ", cycle_number);
        lgw_spi_wb(spi_target, LGW_SPI_MUX_TARGET_SX1302, SX1302_AGC_MCU_MEM, test_buff, BUFF_SIZE);
        lgw_spi_rb(spi_target, LGW_SPI_MUX_TARGET_SX1302, SX1302_AGC_MCU_MEM, read_buff, BUFF_SIZE);
        for (i=0; ((i<BUFF_SIZE) && (test_buff[i] == read_buff[i])); ++i);
        if (i != BUFF_SIZE) {
            printf("error during the buffer comparison\n");
            printf("Written values:\n");
            for (i=0; i<BUFF_SIZE; ++i) {
                printf(" %02X ", test_buff[i]);
                if (i%16 == 15) printf("\n");
            }
            printf("\n");
            printf("Read values:\n");
            for (i=0; i<BUFF_SIZE; ++i) {
                printf(" %02X ", read_buff[i]);
                if (i%16 == 15) printf("\n");
            }
            printf("\n");
            return EXIT_FAILURE;
        } else {
            printf("did a %i-byte R/W on a data buffer with no error\n", BUFF_SIZE);
            ++cycle_number;
        }
    }

#else
    lgw_spi_w(spi_target, LGW_SPI_MUX_TARGET_SX1302, 0x5783, 0x0C); /* Set radio_en & radio_rst to 1 */
    lgw_spi_r(spi_target, LGW_SPI_MUX_TARGET_SX1302, 0x5783, &data);
    printf("SX1302 agc_mcu.rf_en_a: 0x%02X\n", data);

    buff[0] = 0x00;
    sx1262fe_write_command(spi_target, 0x80, buff, 1); /* SetStandby STDBY_RC */

    buff[0] = 0x00;
    sx1262fe_read_command(spi_target, 0xC0, buff, 1); /* GetStatus */
    printf("SX1262FE status: 0x%02X\n", buff[0]);

    buff[0] = 0x00;
    sx1262fe_write_command(spi_target, 0x8A, buff, 1); /* SetPacketType FSK */

    buff[0] = 0x36;
    buff[1] = 0x48;
    buff[2] = 0x00;
    buff[3] = 0x00;
    sx1262fe_write_command(spi_target, 0x86, buff, 4); /* SetRfFrequency */

    buff[0] = 0x0E;
    buff[1] = 0x02;
    sx1262fe_write_command(spi_target, 0x8E, buff, 2); /* SetTxParams (power, RAMP_40U) */

    buff[0] = 0x04;
    buff[1] = 0x07;
    buff[2] = 0x00;
    buff[3] = 0x01;
    sx1262fe_write_command(spi_target, 0x95, buff, 4); /* SetPaOpt */

    buff[0] = 0x00;
    sx1262fe_write_command(spi_target, 0xD1, buff, 1); /* SetTxContinuousWave */

    buff[0] = 0x00;
    sx1262fe_read_command(spi_target, 0xC0, buff, 1); /* GetStatus */
    printf("SX1262FE status: 0x%02X\n", buff[0]);
#endif

    lgw_spi_close(spi_target);
    printf("End of test for loragw_spi.c\n");

    return 0;
}

/* --- EOF ------------------------------------------------------------------ */
