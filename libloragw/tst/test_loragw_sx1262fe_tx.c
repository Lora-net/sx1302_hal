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

    printf("================================\n");
    printf("Press enter to continue\n");
    getchar();
    printf("================================\n");
    printf("Switch SX126x in FPGA Tx mode\n");

    buff[0] = 0x05;
    buff[1] = 0x84;
    buff[2] = 0x00;
    sx1262fe_write_command(0x0D, buff, 3); /* WriteRegister: mod="DioCtrl", reg="puReg", data=0x00, targetFe=True */

    buff[0] = 0x05;
    buff[1] = 0x85;
    buff[2] = 0x00;
    sx1262fe_write_command(0x0D, buff, 3); /* WriteRegister: mod="DioCtrl", reg="pdReg", data=0x00, targetFe=True */

    buff[0] = 0x05;
    buff[1] = 0x82;
    buff[2] = 0x0F;
    sx1262fe_write_command(0x0D, buff, 3); /* WriteRegister: mod="DioCtrl", reg="dsValReg", data=0xf, targetFe=True */

    buff[0] = 0x05;
    buff[1] = 0x80;
    buff[2] = 0x3E;
    sx1262fe_write_command(0x0D, buff, 3); /* WriteRegister: mod="DioCtrl", reg="outEnReg", data=self.ioTxMode, targetFe=True */

    buff[0] = 0x05;
    buff[1] = 0x83;
    buff[2] = 0x3E;
    sx1262fe_write_command(0x0D, buff, 3); /* WriteRegister: mod="DioCtrl", reg="ieValReg", data=self.ioTxMode, targetFe=True */

    buff[0] = 0x05;
    buff[1] = 0x87;
    buff[2] = 0x09;
    sx1262fe_write_command(0x0D, buff, 3); /* WriteRegister: mod="DioCtrl", reg="alfCfgReg", data=self.stdbyTestMode, targetFe=True */

    printf("Switch SX1302 clock from SPI clock to SX126x clock\n");

    lgw_reg_w(SX1302_REG_CLK_CTRL_CLK_SEL_CLK_RADIO_A_SEL, 0x01);

    printf("Setup radio A FE I/F in Tx mode(SX126x)\n");

    lgw_reg_w(SX1302_REG_COMMON_CTRL0_SX1261_MODE_RADIO_A, 0x01);

    lgw_reg_w(SX1302_REG_TX_TOP_A_TX_RFFE_IF_CTRL_TX_IF_SRC, 0x00);
    lgw_reg_w(SX1302_REG_TX_TOP_A_TX_RFFE_IF_CTRL_TX_IF_DST, 0x01);
    lgw_reg_w(SX1302_REG_TX_TOP_A_TX_RFFE_IF_CTRL_TX_MODE, 0x01);
    
    lgw_reg_w(SX1302_REG_TX_TOP_A_TX_RFFE_IF_FREQ_RF_H_FREQ_RF, 0x6C);
    lgw_reg_w(SX1302_REG_TX_TOP_A_TX_RFFE_IF_FREQ_RF_M_FREQ_RF, 0x90);
    lgw_reg_w(SX1302_REG_TX_TOP_A_TX_RFFE_IF_FREQ_RF_L_FREQ_RF, 0x00);

    lgw_reg_w(SX1302_REG_TX_TOP_A_TX_RFFE_IF_FREQ_DEV_H_FREQ_DEV, 0x08);
    lgw_reg_w(SX1302_REG_TX_TOP_A_TX_RFFE_IF_FREQ_DEV_L_FREQ_DEV, 0x00);

    printf("================================\n");
    printf("Start Tx\n");

    lgw_reg_w(SX1302_REG_TX_TOP_A_TX_TRIG_TX_TRIG_IMMEDIATE, 0x01);

    printf("================================\n");
    printf("Press enter to stop Tx\n");
    getchar();
    printf("================================\n");
    printf("Stop Tx\n");

    lgw_reg_w(SX1302_REG_TX_TOP_A_TX_TRIG_TX_TRIG_IMMEDIATE, 0x00);

    lgw_disconnect();
    printf("=========== Test End ===========\n");

    return 0;
}

/* --- EOF ------------------------------------------------------------------ */
