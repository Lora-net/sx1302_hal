/*
 / _____)             _              | |
( (____  _____ ____ _| |_ _____  ____| |__
 \____ \| ___ |    (_   _) ___ |/ ___)  _ \
 _____) ) ____| | | || |_| ____( (___| | | |
(______/|_____)_|_|_| \__)_____)\____)_| |_|
  (C)2018 Semtech

Description:
    TODO

License: Revised BSD License, see LICENSE.TXT file include in the project
*/


/* -------------------------------------------------------------------------- */
/* --- DEPENDANCIES --------------------------------------------------------- */

#include <stdint.h>     /* C99 types */
#include <stdbool.h>    /* bool type */
#include <stdio.h>      /* printf fprintf */

#include "loragw_spi.h"
#include "loragw_reg.h"

/* -------------------------------------------------------------------------- */
/* --- PRIVATE MACROS ------------------------------------------------------- */

#define ARRAY_SIZE(a) (sizeof(a) / sizeof((a)[0]))
#if DEBUG_REG == 1
    #define DEBUG_MSG(str)              fprintf(stderr, str)
    #define DEBUG_PRINTF(fmt, args...)  fprintf(stderr,"%s:%d: "fmt, __FUNCTION__, __LINE__, args)
    #define CHECK_NULL(a)               if(a==NULL){fprintf(stderr,"%s:%d: ERROR: NULL POINTER AS ARGUMENT\n", __FUNCTION__, __LINE__);return LGW_REG_ERROR;}
#else
    #define DEBUG_MSG(str)
    #define DEBUG_PRINTF(fmt, args...)
    #define CHECK_NULL(a)               if(a==NULL){return LGW_REG_ERROR;}
#endif

/* -------------------------------------------------------------------------- */
/* --- PRIVATE CONSTANTS ---------------------------------------------------- */

#define SX1302_REG_EXT_MEM_PAGED_BASE_ADDR 0x0
#define SX1302_REG_RX_BUFFER_BASE_ADDR 0x4000
#define SX1302_REG_TX_TOP_A_BASE_ADDR 0x5200
#define SX1302_REG_TX_TOP_B_BASE_ADDR 0x5400
#define SX1302_REG_COMMON_BASE_ADDR 0x5600
#define SX1302_REG_GPIO_BASE_ADDR 0x5640
#define SX1302_REG_MBIST_BASE_ADDR 0x56c0
#define SX1302_REG_RADIO_FE_BASE_ADDR 0x5700
#define SX1302_REG_AGC_MCU_BASE_ADDR 0x5780
#define SX1302_REG_CLK_CTRL_BASE_ADDR 0x57c0
#define SX1302_REG_RX_TOP_BASE_ADDR 0x5800
#define SX1302_REG_RX_TOP_LORA_SERVICE_FSK_BASE_ADDR 0x5b00
#define SX1302_REG_CAPTURE_RAM_BASE_ADDR 0x6000
#define SX1302_REG_ARB_MCU_BASE_ADDR 0x6080
#define SX1302_REG_TIMESTAMP_BASE_ADDR 0x6100
#define SX1302_REG_OTP_BASE_ADDR 0x6180

const struct lgw_reg_s loregs[LGW_TOTALREGS+1] = {
    {0,SX1302_REG_COMMON_BASE_ADDR+0,0,0,2,0,0}, // COMMON_PAGE_PAGE
    {0,SX1302_REG_COMMON_BASE_ADDR+1,4,0,1,0,0}, // COMMON_CTRL0_CLK32_RIF_CTRL
    {0,SX1302_REG_COMMON_BASE_ADDR+1,3,0,1,0,1}, // COMMON_CTRL0_HOST_RADIO_CTRL
    {0,SX1302_REG_COMMON_BASE_ADDR+1,2,0,1,0,0}, // COMMON_CTRL0_RADIO_MISC_EN
    {0,SX1302_REG_COMMON_BASE_ADDR+1,1,0,1,0,0}, // COMMON_CTRL0_SX1261_MODE_RADIO_B
    {0,SX1302_REG_COMMON_BASE_ADDR+1,0,0,1,0,0}, // COMMON_CTRL0_SX1261_MODE_RADIO_A
    {0,SX1302_REG_COMMON_BASE_ADDR+2,0,0,8,0,2}, // COMMON_SPI_DIV_RATIO_SPI_HALF_PERIOD
    {0,SX1302_REG_COMMON_BASE_ADDR+3,0,0,8,0,128}, // COMMON_RADIO_SELECT_RADIO_SELECT
    {0,SX1302_REG_COMMON_BASE_ADDR+4,3,0,1,0,0}, // COMMON_GEN_GLOBAL_EN
    {0,SX1302_REG_COMMON_BASE_ADDR+4,2,0,1,0,0}, // COMMON_GEN_FSK_MODEM_ENABLE
    {0,SX1302_REG_COMMON_BASE_ADDR+4,1,0,1,0,0}, // COMMON_GEN_CONCENTRATOR_MODEM_ENABLE
    {0,SX1302_REG_COMMON_BASE_ADDR+4,0,0,1,0,0}, // COMMON_GEN_MBWSSF_MODEM_ENABLE
    {0,SX1302_REG_COMMON_BASE_ADDR+5,0,0,8,1,3}, // COMMON_VERSION_VERSION
    {0,SX1302_REG_COMMON_BASE_ADDR+6,0,0,1,0,0}, // COMMON_DUMMY_DUMMY
    {0,SX1302_REG_AGC_MCU_BASE_ADDR+0,3,0,1,0,0}, // AGC_MCU_CTRL_FORCE_HOST_FE_CTRL
    {0,SX1302_REG_AGC_MCU_BASE_ADDR+0,2,0,1,0,0}, // AGC_MCU_CTRL_MCU_CLEAR
    {0,SX1302_REG_AGC_MCU_BASE_ADDR+0,1,0,1,0,0}, // AGC_MCU_CTRL_HOST_PROG
    {0,SX1302_REG_AGC_MCU_BASE_ADDR+1,1,0,1,1,0}, // AGC_MCU_MCU_AGC_STATUS_PARITY_ERROR
    {0,SX1302_REG_AGC_MCU_BASE_ADDR+1,0,0,1,1,0}, // AGC_MCU_MCU_AGC_STATUS_MCU_AGC_STATUS
    {0,SX1302_REG_AGC_MCU_BASE_ADDR+2,2,0,2,0,0}, // AGC_MCU_PA_GAIN_PA_B_GAIN
    {0,SX1302_REG_AGC_MCU_BASE_ADDR+2,0,0,2,0,0}, // AGC_MCU_PA_GAIN_PA_A_GAIN
    {0,SX1302_REG_AGC_MCU_BASE_ADDR+3,3,0,1,0,0}, // AGC_MCU_RF_EN_A_RADIO_RST
    {0,SX1302_REG_AGC_MCU_BASE_ADDR+3,2,0,1,0,0}, // AGC_MCU_RF_EN_A_RADIO_EN
    {0,SX1302_REG_AGC_MCU_BASE_ADDR+3,1,0,1,0,0}, // AGC_MCU_RF_EN_A_PA_EN
    {0,SX1302_REG_AGC_MCU_BASE_ADDR+3,0,0,1,0,0}, // AGC_MCU_RF_EN_A_LNA_EN
    {0,SX1302_REG_AGC_MCU_BASE_ADDR+4,3,0,1,0,0}, // AGC_MCU_RF_EN_B_RADIO_RST
    {0,SX1302_REG_AGC_MCU_BASE_ADDR+4,2,0,1,0,0}, // AGC_MCU_RF_EN_B_RADIO_EN
    {0,SX1302_REG_AGC_MCU_BASE_ADDR+4,1,0,1,0,0}, // AGC_MCU_RF_EN_B_PA_EN
    {0,SX1302_REG_AGC_MCU_BASE_ADDR+4,0,0,1,0,0}, // AGC_MCU_RF_EN_B_LNA_EN
    {0,SX1302_REG_AGC_MCU_BASE_ADDR+5,4,0,4,0,0}, // AGC_MCU_LUT_TABLE_A_PA_LUT
    {0,SX1302_REG_AGC_MCU_BASE_ADDR+5,0,0,4,0,0}, // AGC_MCU_LUT_TABLE_A_LNA_LUT
    {0,SX1302_REG_AGC_MCU_BASE_ADDR+6,4,0,4,0,0}, // AGC_MCU_LUT_TABLE_B_PA_LUT
    {0,SX1302_REG_AGC_MCU_BASE_ADDR+6,0,0,4,0,0}, // AGC_MCU_LUT_TABLE_B_LNA_LUT
    {0,SX1302_REG_AGC_MCU_BASE_ADDR+7,5,0,1,0,0}, // AGC_MCU_UART_CFG_MSBF
    {0,SX1302_REG_AGC_MCU_BASE_ADDR+7,4,0,1,0,0}, // AGC_MCU_UART_CFG_PAR_EN
    {0,SX1302_REG_AGC_MCU_BASE_ADDR+7,3,0,1,0,0}, // AGC_MCU_UART_CFG_PAR_MODE
    {0,SX1302_REG_AGC_MCU_BASE_ADDR+7,2,0,1,0,0}, // AGC_MCU_UART_CFG_START_LEN
    {0,SX1302_REG_AGC_MCU_BASE_ADDR+7,1,0,1,0,0}, // AGC_MCU_UART_CFG_STOP_LEN
    {0,SX1302_REG_AGC_MCU_BASE_ADDR+7,0,0,1,0,1}, // AGC_MCU_UART_CFG_WORD_LEN
    {0,SX1302_REG_AGC_MCU_BASE_ADDR+8,0,0,8,0,0}, // AGC_MCU_MCU_MAIL_BOX_WR_DATA_BYTE3_MCU_MAIL_BOX_WR_DATA
    {0,SX1302_REG_AGC_MCU_BASE_ADDR+9,0,0,8,0,0}, // AGC_MCU_MCU_MAIL_BOX_WR_DATA_BYTE2_MCU_MAIL_BOX_WR_DATA
    {0,SX1302_REG_AGC_MCU_BASE_ADDR+10,0,0,8,0,0}, // AGC_MCU_MCU_MAIL_BOX_WR_DATA_BYTE1_MCU_MAIL_BOX_WR_DATA
    {0,SX1302_REG_AGC_MCU_BASE_ADDR+11,0,0,8,0,0}, // AGC_MCU_MCU_MAIL_BOX_WR_DATA_BYTE0_MCU_MAIL_BOX_WR_DATA
    {0,SX1302_REG_AGC_MCU_BASE_ADDR+12,0,0,8,1,0}, // AGC_MCU_MCU_MAIL_BOX_RD_DATA_BYTE3_MCU_MAIL_BOX_RD_DATA
    {0,SX1302_REG_AGC_MCU_BASE_ADDR+13,0,0,8,1,0}, // AGC_MCU_MCU_MAIL_BOX_RD_DATA_BYTE2_MCU_MAIL_BOX_RD_DATA
    {0,SX1302_REG_AGC_MCU_BASE_ADDR+14,0,0,8,1,0}, // AGC_MCU_MCU_MAIL_BOX_RD_DATA_BYTE1_MCU_MAIL_BOX_RD_DATA
    {0,SX1302_REG_AGC_MCU_BASE_ADDR+15,0,0,8,1,0}, // AGC_MCU_MCU_MAIL_BOX_RD_DATA_BYTE0_MCU_MAIL_BOX_RD_DATA
    {0,SX1302_REG_CLK_CTRL_BASE_ADDR+0,2,0,1,0,0}, // CLK_CTRL_CLK_SEL_CLKDIV_EN
    {0,SX1302_REG_CLK_CTRL_BASE_ADDR+0,1,0,1,0,0}, // CLK_CTRL_CLK_SEL_CLK_RADIO_B_SEL
    {0,SX1302_REG_CLK_CTRL_BASE_ADDR+0,0,0,1,0,0}, // CLK_CTRL_CLK_SEL_CLK_RADIO_A_SEL
    {0,SX1302_REG_CLK_CTRL_BASE_ADDR+1,3,0,1,0,0}, // CLK_CTRL_DUMMY_DUMMY
    {0,SX1302_REG_TX_TOP_A_BASE_ADDR+0,2,0,1,0,0}, // TX_TOP_A_TX_TRIG_TX_TRIG_GPS
    {0,SX1302_REG_TX_TOP_A_BASE_ADDR+0,1,0,1,0,0}, // TX_TOP_A_TX_TRIG_TX_TRIG_DELAYED
    {0,SX1302_REG_TX_TOP_A_BASE_ADDR+0,0,0,1,0,0}, // TX_TOP_A_TX_TRIG_TX_TRIG_IMMEDIATE
    {0,SX1302_REG_TX_TOP_A_BASE_ADDR+1,0,0,8,0,0}, // TX_TOP_A_TIMER_TRIG_BYTE3_TIMER_DELAYED_TRIG
    {0,SX1302_REG_TX_TOP_A_BASE_ADDR+2,0,0,8,0,0}, // TX_TOP_A_TIMER_TRIG_BYTE2_TIMER_DELAYED_TRIG
    {0,SX1302_REG_TX_TOP_A_BASE_ADDR+3,0,0,8,0,0}, // TX_TOP_A_TIMER_TRIG_BYTE1_TIMER_DELAYED_TRIG
    {0,SX1302_REG_TX_TOP_A_BASE_ADDR+4,0,0,8,0,0}, // TX_TOP_A_TIMER_TRIG_BYTE0_TIMER_DELAYED_TRIG
    {0,SX1302_REG_TX_TOP_A_BASE_ADDR+5,0,0,8,0,0}, // TX_TOP_A_TX_START_DELAY_MSB_TX_START_DELAY
    {0,SX1302_REG_TX_TOP_A_BASE_ADDR+6,0,0,8,0,0}, // TX_TOP_A_TX_START_DELAY_LSB_TX_START_DELAY
    {0,SX1302_REG_TX_TOP_A_BASE_ADDR+7,0,0,1,0,1}, // TX_TOP_A_TX_CTRL_WRITE_BUFFER
    {0,SX1302_REG_TX_TOP_A_BASE_ADDR+8,0,0,3,0,1}, // TX_TOP_A_TX_RAMP_DURATION_TX_RAMP_DURATION
    {0,SX1302_REG_TX_TOP_A_BASE_ADDR+9,5,0,1,0,0}, // TX_TOP_A_GEN_CFG_0_TX_RADIO_SEL
    {0,SX1302_REG_TX_TOP_A_BASE_ADDR+9,4,0,1,0,0}, // TX_TOP_A_GEN_CFG_0_MODULATION_TYPE
    {0,SX1302_REG_TX_TOP_A_BASE_ADDR+9,0,0,4,0,0}, // TX_TOP_A_GEN_CFG_0_TX_POWER
    {0,SX1302_REG_TX_TOP_A_BASE_ADDR+10,5,0,3,0,0}, // TX_TOP_A_TX_RFFE_IF_CTRL_PLL_DIV_CTRL
    {0,SX1302_REG_TX_TOP_A_BASE_ADDR+10,4,0,1,0,0}, // TX_TOP_A_TX_RFFE_IF_CTRL_TX_CLK_EDGE
    {0,SX1302_REG_TX_TOP_A_BASE_ADDR+10,3,0,1,0,0}, // TX_TOP_A_TX_RFFE_IF_CTRL_TX_MODE
    {0,SX1302_REG_TX_TOP_A_BASE_ADDR+10,2,0,1,0,0}, // TX_TOP_A_TX_RFFE_IF_CTRL_TX_IF_DST
    {0,SX1302_REG_TX_TOP_A_BASE_ADDR+10,0,0,2,0,0}, // TX_TOP_A_TX_RFFE_IF_CTRL_TX_IF_SRC
    {0,SX1302_REG_TX_TOP_A_BASE_ADDR+11,0,0,2,0,0}, // TX_TOP_A_TX_RFFE_IF_IQ_GAIN_IQ_GAIN
    {0,SX1302_REG_TX_TOP_A_BASE_ADDR+12,0,0,8,0,0}, // TX_TOP_A_TX_RFFE_IF_IQ_OFFSET_IQ_OFFSET
    {0,SX1302_REG_TX_TOP_A_BASE_ADDR+13,0,0,8,0,0}, // TX_TOP_A_TX_RFFE_IF_FREQ_RF_H_FREQ_RF
    {0,SX1302_REG_TX_TOP_A_BASE_ADDR+14,0,0,8,0,0}, // TX_TOP_A_TX_RFFE_IF_FREQ_RF_M_FREQ_RF
    {0,SX1302_REG_TX_TOP_A_BASE_ADDR+15,0,0,8,0,0}, // TX_TOP_A_TX_RFFE_IF_FREQ_RF_L_FREQ_RF
    {0,SX1302_REG_TX_TOP_A_BASE_ADDR+16,0,0,4,0,0}, // TX_TOP_A_TX_RFFE_IF_FREQ_DEV_H_FREQ_DEV
    {0,SX1302_REG_TX_TOP_A_BASE_ADDR+17,0,0,8,0,0}, // TX_TOP_A_TX_RFFE_IF_FREQ_DEV_L_FREQ_DEV
    {0,SX1302_REG_TX_TOP_A_BASE_ADDR+18,0,0,8,0,64}, // TX_TOP_A_TX_RFFE_IF_TEST_MOD_FREQ
    {0,SX1302_REG_TX_TOP_A_BASE_ADDR+19,0,0,8,0,0}, // TX_TOP_A_PKT_LEN_PKT_LENGTH
    {0,SX1302_REG_TX_TOP_A_BASE_ADDR+20,5,0,1,0,0}, // TX_TOP_A_FSK_CFG_0_TX_MODE
    {0,SX1302_REG_TX_TOP_A_BASE_ADDR+20,4,0,1,0,0}, // TX_TOP_A_FSK_CFG_0_CRC_IBM
    {0,SX1302_REG_TX_TOP_A_BASE_ADDR+20,2,0,2,0,0}, // TX_TOP_A_FSK_CFG_0_DCFREE_ENC
    {0,SX1302_REG_TX_TOP_A_BASE_ADDR+20,1,0,1,0,0}, // TX_TOP_A_FSK_CFG_0_CRC_EN
    {0,SX1302_REG_TX_TOP_A_BASE_ADDR+20,0,0,1,0,0}, // TX_TOP_A_FSK_CFG_0_PKT_MODE
    {0,SX1302_REG_TX_TOP_A_BASE_ADDR+21,0,0,8,0,0}, // TX_TOP_A_PREAMBLE_SIZE_MSB_PREAMBLE_SIZE
    {0,SX1302_REG_TX_TOP_A_BASE_ADDR+22,0,0,8,0,0}, // TX_TOP_A_PREAMBLE_SIZE_LSB_PREAMBLE_SIZE
    {0,SX1302_REG_TX_TOP_A_BASE_ADDR+23,0,0,8,0,0}, // TX_TOP_A_BIT_RATE_MSB_BIT_RATE
    {0,SX1302_REG_TX_TOP_A_BASE_ADDR+24,0,0,8,0,0}, // TX_TOP_A_BIT_RATE_LSB_BIT_RATE
    {0,SX1302_REG_TX_TOP_A_BASE_ADDR+25,5,0,3,0,0}, // TX_TOP_A_FSK_BT_FSK_REF_PATTERN_SIZE
    {0,SX1302_REG_TX_TOP_A_BASE_ADDR+25,4,0,1,0,0}, // TX_TOP_A_FSK_BT_FSK_PREAMBLE_SEQ
    {0,SX1302_REG_TX_TOP_A_BASE_ADDR+25,3,0,1,0,1}, // TX_TOP_A_FSK_BT_FSK_REF_PATTERN_EN
    {0,SX1302_REG_TX_TOP_A_BASE_ADDR+25,1,0,2,0,0}, // TX_TOP_A_FSK_BT_FSK_GAUSSIAN_SELECT_BT
    {0,SX1302_REG_TX_TOP_A_BASE_ADDR+25,0,0,1,0,0}, // TX_TOP_A_FSK_BT_FSK_GAUSSIAN_EN
    {0,SX1302_REG_TX_TOP_A_BASE_ADDR+26,0,0,8,0,0}, // TX_TOP_A_FSK_REF_PATTERN_BYTE7_FSK_REF_PATTERN
    {0,SX1302_REG_TX_TOP_A_BASE_ADDR+27,0,0,8,0,0}, // TX_TOP_A_FSK_REF_PATTERN_BYTE6_FSK_REF_PATTERN
    {0,SX1302_REG_TX_TOP_A_BASE_ADDR+28,0,0,8,0,0}, // TX_TOP_A_FSK_REF_PATTERN_BYTE5_FSK_REF_PATTERN
    {0,SX1302_REG_TX_TOP_A_BASE_ADDR+29,0,0,8,0,0}, // TX_TOP_A_FSK_REF_PATTERN_BYTE4_FSK_REF_PATTERN
    {0,SX1302_REG_TX_TOP_A_BASE_ADDR+30,0,0,8,0,0}, // TX_TOP_A_FSK_REF_PATTERN_BYTE3_FSK_REF_PATTERN
    {0,SX1302_REG_TX_TOP_A_BASE_ADDR+31,0,0,8,0,0}, // TX_TOP_A_FSK_REF_PATTERN_BYTE2_FSK_REF_PATTERN
    {0,SX1302_REG_TX_TOP_A_BASE_ADDR+32,0,0,8,0,0}, // TX_TOP_A_FSK_REF_PATTERN_BYTE1_FSK_REF_PATTERN
    {0,SX1302_REG_TX_TOP_A_BASE_ADDR+33,0,0,8,0,0}, // TX_TOP_A_FSK_REF_PATTERN_BYTE0_FSK_REF_PATTERN
    {0,SX1302_REG_TX_TOP_A_BASE_ADDR+34,0,0,8,1,0}, // TX_TOP_A_TX_STATUS_TX_STATUS
    {0,SX1302_REG_TX_TOP_A_BASE_ADDR+35,4,0,4,0,5}, // TX_TOP_A_TXRX_CFG0_0_MODEM_BW
    {0,SX1302_REG_TX_TOP_A_BASE_ADDR+35,0,0,4,0,7}, // TX_TOP_A_TXRX_CFG0_0_MODEM_SF
    {0,SX1302_REG_TX_TOP_A_BASE_ADDR+36,6,0,2,0,2}, // TX_TOP_A_TXRX_CFG0_1_PPM_OFFSET_HDR_CTRL
    {0,SX1302_REG_TX_TOP_A_BASE_ADDR+36,4,0,2,0,0}, // TX_TOP_A_TXRX_CFG0_1_PPM_OFFSET
    {0,SX1302_REG_TX_TOP_A_BASE_ADDR+36,3,0,1,0,0}, // TX_TOP_A_TXRX_CFG0_1_POST_PREAMBLE_GAP_LONG
    {0,SX1302_REG_TX_TOP_A_BASE_ADDR+36,0,0,3,0,2}, // TX_TOP_A_TXRX_CFG0_1_CODING_RATE
    {0,SX1302_REG_TX_TOP_A_BASE_ADDR+37,7,0,1,0,0}, // TX_TOP_A_TXRX_CFG0_2_FINE_SYNCH_EN
    {0,SX1302_REG_TX_TOP_A_BASE_ADDR+37,6,0,1,0,0}, // TX_TOP_A_TXRX_CFG0_2_MODEM_EN
    {0,SX1302_REG_TX_TOP_A_BASE_ADDR+37,4,0,2,0,2}, // TX_TOP_A_TXRX_CFG0_2_CADRXTX
    {0,SX1302_REG_TX_TOP_A_BASE_ADDR+37,1,0,1,0,0}, // TX_TOP_A_TXRX_CFG0_2_IMPLICIT_HEADER
    {0,SX1302_REG_TX_TOP_A_BASE_ADDR+37,0,0,1,0,1}, // TX_TOP_A_TXRX_CFG0_2_CRC_EN
    {0,SX1302_REG_TX_TOP_A_BASE_ADDR+38,0,0,8,0,12}, // TX_TOP_A_TXRX_CFG0_3_PAYLOAD_LENGTH
    {0,SX1302_REG_TX_TOP_A_BASE_ADDR+39,7,0,1,0,0}, // TX_TOP_A_TXRX_CFG1_0_INT_STEP_ORIDE_EN
    {0,SX1302_REG_TX_TOP_A_BASE_ADDR+39,0,0,6,0,0}, // TX_TOP_A_TXRX_CFG1_0_INT_STEP_ORIDE
    {0,SX1302_REG_TX_TOP_A_BASE_ADDR+40,7,0,1,0,0}, // TX_TOP_A_TXRX_CFG1_1_MODEM_START
    {0,SX1302_REG_TX_TOP_A_BASE_ADDR+40,6,0,1,0,0}, // TX_TOP_A_TXRX_CFG1_1_HEADER_DIFF_MODE
    {0,SX1302_REG_TX_TOP_A_BASE_ADDR+40,0,0,6,0,0}, // TX_TOP_A_TXRX_CFG1_1_ZERO_PAD
    {0,SX1302_REG_TX_TOP_A_BASE_ADDR+41,0,0,8,0,8}, // TX_TOP_A_TXRX_CFG1_2_PREAMBLE_SYMB_NB
    {0,SX1302_REG_TX_TOP_A_BASE_ADDR+42,0,0,8,0,0}, // TX_TOP_A_TXRX_CFG1_3_PREAMBLE_SYMB_NB
    {0,SX1302_REG_TX_TOP_A_BASE_ADDR+43,6,0,1,0,1}, // TX_TOP_A_TXRX_CFG1_4_AUTO_ACK_INT_DELAY
    {0,SX1302_REG_TX_TOP_A_BASE_ADDR+43,5,0,1,0,0}, // TX_TOP_A_TXRX_CFG1_4_AUTO_ACK_RX
    {0,SX1302_REG_TX_TOP_A_BASE_ADDR+43,4,0,1,0,0}, // TX_TOP_A_TXRX_CFG1_4_AUTO_ACK_TX
    {0,SX1302_REG_TX_TOP_A_BASE_ADDR+44,4,0,3,0,0}, // TX_TOP_A_TX_CFG0_0_CHIRP_LOWPASS
    {0,SX1302_REG_TX_TOP_A_BASE_ADDR+44,3,0,1,0,0}, // TX_TOP_A_TX_CFG0_0_PPM_OFFSET_SIG
    {0,SX1302_REG_TX_TOP_A_BASE_ADDR+44,2,0,1,0,1}, // TX_TOP_A_TX_CFG0_0_CONTCHIRP
    {0,SX1302_REG_TX_TOP_A_BASE_ADDR+44,1,0,1,0,1}, // TX_TOP_A_TX_CFG0_0_CHIRP_INVERT
    {0,SX1302_REG_TX_TOP_A_BASE_ADDR+44,0,0,1,0,1}, // TX_TOP_A_TX_CFG0_0_CONTINUOUS
    {0,SX1302_REG_TX_TOP_A_BASE_ADDR+45,0,0,6,0,20}, // TX_TOP_A_TX_CFG0_1_POWER_RANGING
    {0,SX1302_REG_TX_TOP_A_BASE_ADDR+46,0,0,8,0,0}, // TX_TOP_A_TX_CFG1_0_FRAME_NB
    {0,SX1302_REG_TX_TOP_A_BASE_ADDR+47,5,0,2,0,0}, // TX_TOP_A_TX_CFG1_1_HOP_CTRL
    {0,SX1302_REG_TX_TOP_A_BASE_ADDR+47,0,0,5,0,10}, // TX_TOP_A_TX_CFG1_1_IFS
    {0,SX1302_REG_TX_TOP_A_BASE_ADDR+48,7,0,1,0,1}, // TX_TOP_A_FRAME_SYNCH_0_AUTO_SCALE
    {0,SX1302_REG_TX_TOP_A_BASE_ADDR+48,6,0,1,0,0}, // TX_TOP_A_FRAME_SYNCH_0_DROP_ON_SYNCH
    {0,SX1302_REG_TX_TOP_A_BASE_ADDR+48,5,0,1,0,1}, // TX_TOP_A_FRAME_SYNCH_0_GAIN
    {0,SX1302_REG_TX_TOP_A_BASE_ADDR+48,0,1,5,0,2}, // TX_TOP_A_FRAME_SYNCH_0_PEAK1_POS
    {0,SX1302_REG_TX_TOP_A_BASE_ADDR+49,7,0,1,0,0}, // TX_TOP_A_FRAME_SYNCH_1_FINETIME_ON_LAST
    {0,SX1302_REG_TX_TOP_A_BASE_ADDR+49,5,0,2,0,3}, // TX_TOP_A_FRAME_SYNCH_1_TIMEOUT_OPT
    {0,SX1302_REG_TX_TOP_A_BASE_ADDR+49,0,1,5,0,4}, // TX_TOP_A_FRAME_SYNCH_1_PEAK2_POS
    {0,SX1302_REG_TX_TOP_A_BASE_ADDR+50,0,0,4,1,0}, // TX_TOP_A_LORA_TX_STATE_STATUS
    {0,SX1302_REG_TX_TOP_A_BASE_ADDR+51,2,0,1,0,0}, // TX_TOP_A_LORA_TX_FLAG_FRAME_DONE
    {0,SX1302_REG_TX_TOP_A_BASE_ADDR+51,1,0,1,0,0}, // TX_TOP_A_LORA_TX_FLAG_CONT_DONE
    {0,SX1302_REG_TX_TOP_A_BASE_ADDR+51,0,0,1,0,0}, // TX_TOP_A_LORA_TX_FLAG_PLD_DONE
    {0,SX1302_REG_TX_TOP_A_BASE_ADDR+52,3,0,1,0,0}, // TX_TOP_A_DUMMY_DUMMY
    {0,SX1302_REG_TX_TOP_B_BASE_ADDR+0,2,0,1,0,0}, // TX_TOP_B_TX_TRIG_TX_TRIG_GPS
    {0,SX1302_REG_TX_TOP_B_BASE_ADDR+0,1,0,1,0,0}, // TX_TOP_B_TX_TRIG_TX_TRIG_DELAYED
    {0,SX1302_REG_TX_TOP_B_BASE_ADDR+0,0,0,1,0,0}, // TX_TOP_B_TX_TRIG_TX_TRIG_IMMEDIATE
    {0,SX1302_REG_TX_TOP_B_BASE_ADDR+1,0,0,8,0,0}, // TX_TOP_B_TIMER_TRIG_BYTE3_TIMER_DELAYED_TRIG
    {0,SX1302_REG_TX_TOP_B_BASE_ADDR+2,0,0,8,0,0}, // TX_TOP_B_TIMER_TRIG_BYTE2_TIMER_DELAYED_TRIG
    {0,SX1302_REG_TX_TOP_B_BASE_ADDR+3,0,0,8,0,0}, // TX_TOP_B_TIMER_TRIG_BYTE1_TIMER_DELAYED_TRIG
    {0,SX1302_REG_TX_TOP_B_BASE_ADDR+4,0,0,8,0,0}, // TX_TOP_B_TIMER_TRIG_BYTE0_TIMER_DELAYED_TRIG
    {0,SX1302_REG_TX_TOP_B_BASE_ADDR+5,0,0,8,0,0}, // TX_TOP_B_TX_START_DELAY_MSB_TX_START_DELAY
    {0,SX1302_REG_TX_TOP_B_BASE_ADDR+6,0,0,8,0,0}, // TX_TOP_B_TX_START_DELAY_LSB_TX_START_DELAY
    {0,SX1302_REG_TX_TOP_B_BASE_ADDR+7,0,0,1,0,1}, // TX_TOP_B_TX_CTRL_WRITE_BUFFER
    {0,SX1302_REG_TX_TOP_B_BASE_ADDR+8,0,0,3,0,1}, // TX_TOP_B_TX_RAMP_DURATION_TX_RAMP_DURATION
    {0,SX1302_REG_TX_TOP_B_BASE_ADDR+9,5,0,1,0,0}, // TX_TOP_B_GEN_CFG_0_TX_RADIO_SEL
    {0,SX1302_REG_TX_TOP_B_BASE_ADDR+9,4,0,1,0,0}, // TX_TOP_B_GEN_CFG_0_MODULATION_TYPE
    {0,SX1302_REG_TX_TOP_B_BASE_ADDR+9,0,0,4,0,0}, // TX_TOP_B_GEN_CFG_0_TX_POWER
    {0,SX1302_REG_TX_TOP_B_BASE_ADDR+10,5,0,3,0,0}, // TX_TOP_B_TX_RFFE_IF_CTRL_PLL_DIV_CTRL
    {0,SX1302_REG_TX_TOP_B_BASE_ADDR+10,4,0,1,0,0}, // TX_TOP_B_TX_RFFE_IF_CTRL_TX_CLK_EDGE
    {0,SX1302_REG_TX_TOP_B_BASE_ADDR+10,3,0,1,0,0}, // TX_TOP_B_TX_RFFE_IF_CTRL_TX_MODE
    {0,SX1302_REG_TX_TOP_B_BASE_ADDR+10,2,0,1,0,0}, // TX_TOP_B_TX_RFFE_IF_CTRL_TX_IF_DST
    {0,SX1302_REG_TX_TOP_B_BASE_ADDR+10,0,0,2,0,0}, // TX_TOP_B_TX_RFFE_IF_CTRL_TX_IF_SRC
    {0,SX1302_REG_TX_TOP_B_BASE_ADDR+11,0,0,2,0,0}, // TX_TOP_B_TX_RFFE_IF_IQ_GAIN_IQ_GAIN
    {0,SX1302_REG_TX_TOP_B_BASE_ADDR+12,0,0,8,0,0}, // TX_TOP_B_TX_RFFE_IF_IQ_OFFSET_IQ_OFFSET
    {0,SX1302_REG_TX_TOP_B_BASE_ADDR+13,0,0,8,0,0}, // TX_TOP_B_TX_RFFE_IF_FREQ_RF_H_FREQ_RF
    {0,SX1302_REG_TX_TOP_B_BASE_ADDR+14,0,0,8,0,0}, // TX_TOP_B_TX_RFFE_IF_FREQ_RF_M_FREQ_RF
    {0,SX1302_REG_TX_TOP_B_BASE_ADDR+15,0,0,8,0,0}, // TX_TOP_B_TX_RFFE_IF_FREQ_RF_L_FREQ_RF
    {0,SX1302_REG_TX_TOP_B_BASE_ADDR+16,0,0,4,0,0}, // TX_TOP_B_TX_RFFE_IF_FREQ_DEV_H_FREQ_DEV
    {0,SX1302_REG_TX_TOP_B_BASE_ADDR+17,0,0,8,0,0}, // TX_TOP_B_TX_RFFE_IF_FREQ_DEV_L_FREQ_DEV
    {0,SX1302_REG_TX_TOP_B_BASE_ADDR+18,0,0,8,0,64}, // TX_TOP_B_TX_RFFE_IF_TEST_MOD_FREQ
    {0,SX1302_REG_TX_TOP_B_BASE_ADDR+19,0,0,8,0,0}, // TX_TOP_B_PKT_LEN_PKT_LENGTH
    {0,SX1302_REG_TX_TOP_B_BASE_ADDR+20,5,0,1,0,0}, // TX_TOP_B_FSK_CFG_0_TX_MODE
    {0,SX1302_REG_TX_TOP_B_BASE_ADDR+20,4,0,1,0,0}, // TX_TOP_B_FSK_CFG_0_CRC_IBM
    {0,SX1302_REG_TX_TOP_B_BASE_ADDR+20,2,0,2,0,0}, // TX_TOP_B_FSK_CFG_0_DCFREE_ENC
    {0,SX1302_REG_TX_TOP_B_BASE_ADDR+20,1,0,1,0,0}, // TX_TOP_B_FSK_CFG_0_CRC_EN
    {0,SX1302_REG_TX_TOP_B_BASE_ADDR+20,0,0,1,0,0}, // TX_TOP_B_FSK_CFG_0_PKT_MODE
    {0,SX1302_REG_TX_TOP_B_BASE_ADDR+21,0,0,8,0,0}, // TX_TOP_B_PREAMBLE_SIZE_MSB_PREAMBLE_SIZE
    {0,SX1302_REG_TX_TOP_B_BASE_ADDR+22,0,0,8,0,0}, // TX_TOP_B_PREAMBLE_SIZE_LSB_PREAMBLE_SIZE
    {0,SX1302_REG_TX_TOP_B_BASE_ADDR+23,0,0,8,0,0}, // TX_TOP_B_BIT_RATE_MSB_BIT_RATE
    {0,SX1302_REG_TX_TOP_B_BASE_ADDR+24,0,0,8,0,0}, // TX_TOP_B_BIT_RATE_LSB_BIT_RATE
    {0,SX1302_REG_TX_TOP_B_BASE_ADDR+25,5,0,3,0,0}, // TX_TOP_B_FSK_BT_FSK_REF_PATTERN_SIZE
    {0,SX1302_REG_TX_TOP_B_BASE_ADDR+25,4,0,1,0,0}, // TX_TOP_B_FSK_BT_FSK_PREAMBLE_SEQ
    {0,SX1302_REG_TX_TOP_B_BASE_ADDR+25,3,0,1,0,1}, // TX_TOP_B_FSK_BT_FSK_REF_PATTERN_EN
    {0,SX1302_REG_TX_TOP_B_BASE_ADDR+25,1,0,2,0,0}, // TX_TOP_B_FSK_BT_FSK_GAUSSIAN_SELECT_BT
    {0,SX1302_REG_TX_TOP_B_BASE_ADDR+25,0,0,1,0,0}, // TX_TOP_B_FSK_BT_FSK_GAUSSIAN_EN
    {0,SX1302_REG_TX_TOP_B_BASE_ADDR+26,0,0,8,0,0}, // TX_TOP_B_FSK_REF_PATTERN_BYTE7_FSK_REF_PATTERN
    {0,SX1302_REG_TX_TOP_B_BASE_ADDR+27,0,0,8,0,0}, // TX_TOP_B_FSK_REF_PATTERN_BYTE6_FSK_REF_PATTERN
    {0,SX1302_REG_TX_TOP_B_BASE_ADDR+28,0,0,8,0,0}, // TX_TOP_B_FSK_REF_PATTERN_BYTE5_FSK_REF_PATTERN
    {0,SX1302_REG_TX_TOP_B_BASE_ADDR+29,0,0,8,0,0}, // TX_TOP_B_FSK_REF_PATTERN_BYTE4_FSK_REF_PATTERN
    {0,SX1302_REG_TX_TOP_B_BASE_ADDR+30,0,0,8,0,0}, // TX_TOP_B_FSK_REF_PATTERN_BYTE3_FSK_REF_PATTERN
    {0,SX1302_REG_TX_TOP_B_BASE_ADDR+31,0,0,8,0,0}, // TX_TOP_B_FSK_REF_PATTERN_BYTE2_FSK_REF_PATTERN
    {0,SX1302_REG_TX_TOP_B_BASE_ADDR+32,0,0,8,0,0}, // TX_TOP_B_FSK_REF_PATTERN_BYTE1_FSK_REF_PATTERN
    {0,SX1302_REG_TX_TOP_B_BASE_ADDR+33,0,0,8,0,0}, // TX_TOP_B_FSK_REF_PATTERN_BYTE0_FSK_REF_PATTERN
    {0,SX1302_REG_TX_TOP_B_BASE_ADDR+34,0,0,8,1,0}, // TX_TOP_B_TX_STATUS_TX_STATUS
    {0,SX1302_REG_TX_TOP_B_BASE_ADDR+35,4,0,4,0,5}, // TX_TOP_B_TXRX_CFG0_0_MODEM_BW
    {0,SX1302_REG_TX_TOP_B_BASE_ADDR+35,0,0,4,0,7}, // TX_TOP_B_TXRX_CFG0_0_MODEM_SF
    {0,SX1302_REG_TX_TOP_B_BASE_ADDR+36,6,0,2,0,2}, // TX_TOP_B_TXRX_CFG0_1_PPM_OFFSET_HDR_CTRL
    {0,SX1302_REG_TX_TOP_B_BASE_ADDR+36,4,0,2,0,0}, // TX_TOP_B_TXRX_CFG0_1_PPM_OFFSET
    {0,SX1302_REG_TX_TOP_B_BASE_ADDR+36,3,0,1,0,0}, // TX_TOP_B_TXRX_CFG0_1_POST_PREAMBLE_GAP_LONG
    {0,SX1302_REG_TX_TOP_B_BASE_ADDR+36,0,0,3,0,2}, // TX_TOP_B_TXRX_CFG0_1_CODING_RATE
    {0,SX1302_REG_TX_TOP_B_BASE_ADDR+37,7,0,1,0,0}, // TX_TOP_B_TXRX_CFG0_2_FINE_SYNCH_EN
    {0,SX1302_REG_TX_TOP_B_BASE_ADDR+37,6,0,1,0,0}, // TX_TOP_B_TXRX_CFG0_2_MODEM_EN
    {0,SX1302_REG_TX_TOP_B_BASE_ADDR+37,4,0,2,0,2}, // TX_TOP_B_TXRX_CFG0_2_CADRXTX
    {0,SX1302_REG_TX_TOP_B_BASE_ADDR+37,1,0,1,0,0}, // TX_TOP_B_TXRX_CFG0_2_IMPLICIT_HEADER
    {0,SX1302_REG_TX_TOP_B_BASE_ADDR+37,0,0,1,0,1}, // TX_TOP_B_TXRX_CFG0_2_CRC_EN
    {0,SX1302_REG_TX_TOP_B_BASE_ADDR+38,0,0,8,0,12}, // TX_TOP_B_TXRX_CFG0_3_PAYLOAD_LENGTH
    {0,SX1302_REG_TX_TOP_B_BASE_ADDR+39,7,0,1,0,0}, // TX_TOP_B_TXRX_CFG1_0_INT_STEP_ORIDE_EN
    {0,SX1302_REG_TX_TOP_B_BASE_ADDR+39,0,0,6,0,0}, // TX_TOP_B_TXRX_CFG1_0_INT_STEP_ORIDE
    {0,SX1302_REG_TX_TOP_B_BASE_ADDR+40,7,0,1,0,0}, // TX_TOP_B_TXRX_CFG1_1_MODEM_START
    {0,SX1302_REG_TX_TOP_B_BASE_ADDR+40,6,0,1,0,0}, // TX_TOP_B_TXRX_CFG1_1_HEADER_DIFF_MODE
    {0,SX1302_REG_TX_TOP_B_BASE_ADDR+40,0,0,6,0,0}, // TX_TOP_B_TXRX_CFG1_1_ZERO_PAD
    {0,SX1302_REG_TX_TOP_B_BASE_ADDR+41,0,0,8,0,8}, // TX_TOP_B_TXRX_CFG1_2_PREAMBLE_SYMB_NB
    {0,SX1302_REG_TX_TOP_B_BASE_ADDR+42,0,0,8,0,0}, // TX_TOP_B_TXRX_CFG1_3_PREAMBLE_SYMB_NB
    {0,SX1302_REG_TX_TOP_B_BASE_ADDR+43,6,0,1,0,1}, // TX_TOP_B_TXRX_CFG1_4_AUTO_ACK_INT_DELAY
    {0,SX1302_REG_TX_TOP_B_BASE_ADDR+43,5,0,1,0,0}, // TX_TOP_B_TXRX_CFG1_4_AUTO_ACK_RX
    {0,SX1302_REG_TX_TOP_B_BASE_ADDR+43,4,0,1,0,0}, // TX_TOP_B_TXRX_CFG1_4_AUTO_ACK_TX
    {0,SX1302_REG_TX_TOP_B_BASE_ADDR+44,4,0,3,0,0}, // TX_TOP_B_TX_CFG0_0_CHIRP_LOWPASS
    {0,SX1302_REG_TX_TOP_B_BASE_ADDR+44,3,0,1,0,0}, // TX_TOP_B_TX_CFG0_0_PPM_OFFSET_SIG
    {0,SX1302_REG_TX_TOP_B_BASE_ADDR+44,2,0,1,0,1}, // TX_TOP_B_TX_CFG0_0_CONTCHIRP
    {0,SX1302_REG_TX_TOP_B_BASE_ADDR+44,1,0,1,0,1}, // TX_TOP_B_TX_CFG0_0_CHIRP_INVERT
    {0,SX1302_REG_TX_TOP_B_BASE_ADDR+44,0,0,1,0,1}, // TX_TOP_B_TX_CFG0_0_CONTINUOUS
    {0,SX1302_REG_TX_TOP_B_BASE_ADDR+45,0,0,6,0,20}, // TX_TOP_B_TX_CFG0_1_POWER_RANGING
    {0,SX1302_REG_TX_TOP_B_BASE_ADDR+46,0,0,8,0,0}, // TX_TOP_B_TX_CFG1_0_FRAME_NB
    {0,SX1302_REG_TX_TOP_B_BASE_ADDR+47,5,0,2,0,0}, // TX_TOP_B_TX_CFG1_1_HOP_CTRL
    {0,SX1302_REG_TX_TOP_B_BASE_ADDR+47,0,0,5,0,10}, // TX_TOP_B_TX_CFG1_1_IFS
    {0,SX1302_REG_TX_TOP_B_BASE_ADDR+48,7,0,1,0,1}, // TX_TOP_B_FRAME_SYNCH_0_AUTO_SCALE
    {0,SX1302_REG_TX_TOP_B_BASE_ADDR+48,6,0,1,0,0}, // TX_TOP_B_FRAME_SYNCH_0_DROP_ON_SYNCH
    {0,SX1302_REG_TX_TOP_B_BASE_ADDR+48,5,0,1,0,1}, // TX_TOP_B_FRAME_SYNCH_0_GAIN
    {0,SX1302_REG_TX_TOP_B_BASE_ADDR+48,0,1,5,0,2}, // TX_TOP_B_FRAME_SYNCH_0_PEAK1_POS
    {0,SX1302_REG_TX_TOP_B_BASE_ADDR+49,7,0,1,0,0}, // TX_TOP_B_FRAME_SYNCH_1_FINETIME_ON_LAST
    {0,SX1302_REG_TX_TOP_B_BASE_ADDR+49,5,0,2,0,3}, // TX_TOP_B_FRAME_SYNCH_1_TIMEOUT_OPT
    {0,SX1302_REG_TX_TOP_B_BASE_ADDR+49,0,1,5,0,4}, // TX_TOP_B_FRAME_SYNCH_1_PEAK2_POS
    {0,SX1302_REG_TX_TOP_B_BASE_ADDR+50,0,0,4,1,0}, // TX_TOP_B_LORA_TX_STATE_STATUS
    {0,SX1302_REG_TX_TOP_B_BASE_ADDR+51,2,0,1,0,0}, // TX_TOP_B_LORA_TX_FLAG_FRAME_DONE
    {0,SX1302_REG_TX_TOP_B_BASE_ADDR+51,1,0,1,0,0}, // TX_TOP_B_LORA_TX_FLAG_CONT_DONE
    {0,SX1302_REG_TX_TOP_B_BASE_ADDR+51,0,0,1,0,0}, // TX_TOP_B_LORA_TX_FLAG_PLD_DONE
    {0,SX1302_REG_TX_TOP_B_BASE_ADDR+52,3,0,1,0,0}, // TX_TOP_B_DUMMY_DUMMY
    {0,0,0,0,0,0,0}
};

/* -------------------------------------------------------------------------- */
/* --- PRIVATE VARIABLES ---------------------------------------------------- */

/* -------------------------------------------------------------------------- */
/* --- INTERNAL SHARED VARIABLES -------------------------------------------- */

void *lgw_spi_target = NULL; /*! generic pointer to the SPI device */

/* -------------------------------------------------------------------------- */
/* --- PRIVATE FUNCTIONS ---------------------------------------------------- */

int reg_w_align32(void *spi_target, uint8_t spi_mux_target, struct lgw_reg_s r, int32_t reg_value) {
    int spi_stat = LGW_REG_SUCCESS;
    int i, size_byte;
    uint8_t buf[4] = "\x00\x00\x00\x00";

    if ((r.leng == 8) && (r.offs == 0)) {
        /* direct write */
        spi_stat += lgw_spi_w(spi_target, spi_mux_target, r.addr, (uint8_t)reg_value);
    } else if ((r.offs + r.leng) <= 8) {
        /* single-byte read-modify-write, offs:[0-7], leng:[1-7] */
        spi_stat += lgw_spi_r(spi_target, spi_mux_target, r.addr, &buf[0]);
        buf[1] = ((1 << r.leng) - 1) << r.offs; /* bit mask */
        buf[2] = ((uint8_t)reg_value) << r.offs; /* new data offsetted */
        buf[3] = (~buf[1] & buf[0]) | (buf[1] & buf[2]); /* mixing old & new data */
        spi_stat += lgw_spi_w(spi_target, spi_mux_target, r.addr, buf[3]);
    } else if ((r.offs == 0) && (r.leng > 0) && (r.leng <= 32)) {
        /* multi-byte direct write routine */
        size_byte = (r.leng + 7) / 8; /* add a byte if it's not an exact multiple of 8 */
        for (i=0; i<size_byte; ++i) {
            /* big endian register file for a file on N bytes
            Least significant byte is stored in buf[0], most one in buf[N-1] */
            buf[i] = (uint8_t)(0x000000FF & reg_value);
            reg_value = (reg_value >> 8);
        }
        spi_stat += lgw_spi_wb(spi_target, spi_mux_target, r.addr, buf, size_byte); /* write the register in one burst */
    } else {
        /* register spanning multiple memory bytes but with an offset */
        DEBUG_MSG("ERROR: REGISTER SIZE AND OFFSET ARE NOT SUPPORTED\n");
        return LGW_REG_ERROR;
    }

    return spi_stat;
}

/* ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ */

int reg_r_align32(void *spi_target, uint8_t spi_mux_target, struct lgw_reg_s r, int32_t *reg_value) {
    int spi_stat = LGW_SPI_SUCCESS;
    uint8_t bufu[4] = "\x00\x00\x00\x00";
    int8_t *bufs = (int8_t *)bufu;
    int i, size_byte;
    uint32_t u = 0;

    if ((r.offs + r.leng) <= 8) {
        /* read one byte, then shift and mask bits to get reg value with sign extension if needed */
        spi_stat += lgw_spi_r(spi_target, spi_mux_target, r.addr, &bufu[0]);
        bufu[1] = bufu[0] << (8 - r.leng - r.offs); /* left-align the data */
        if (r.sign == true) {
            bufs[2] = bufs[1] >> (8 - r.leng); /* right align the data with sign extension (ARITHMETIC right shift) */
            *reg_value = (int32_t)bufs[2]; /* signed pointer -> 32b sign extension */
        } else {
            bufu[2] = bufu[1] >> (8 - r.leng); /* right align the data, no sign extension */
            *reg_value = (int32_t)bufu[2]; /* unsigned pointer -> no sign extension */
        }
    } else if ((r.offs == 0) && (r.leng > 0) && (r.leng <= 32)) {
        size_byte = (r.leng + 7) / 8; /* add a byte if it's not an exact multiple of 8 */
        spi_stat += lgw_spi_rb(spi_target, spi_mux_target, r.addr, bufu, size_byte);
        u = 0;
        for (i=(size_byte-1); i>=0; --i) {
            u = (uint32_t)bufu[i] + (u << 8); /* transform a 4-byte array into a 32 bit word */
        }
        if (r.sign == true) {
            u = u << (32 - r.leng); /* left-align the data */
            *reg_value = (int32_t)u >> (32 - r.leng); /* right-align the data with sign extension (ARITHMETIC right shift) */
        } else {
            *reg_value = (int32_t)u; /* unsigned value -> return 'as is' */
        }
    } else {
        /* register spanning multiple memory bytes but with an offset */
        DEBUG_MSG("ERROR: REGISTER SIZE AND OFFSET ARE NOT SUPPORTED\n");
        return LGW_REG_ERROR;
    }

    return spi_stat;
}

/* -------------------------------------------------------------------------- */
/* --- PUBLIC FUNCTIONS DEFINITION ------------------------------------------ */

/* Concentrator connect */
int lgw_connect(void) {
    int spi_stat = LGW_SPI_SUCCESS;
    uint8_t u = 0;

    /* check SPI link status */
    if (lgw_spi_target != NULL) {
        DEBUG_MSG("WARNING: concentrator was already connected\n");
        lgw_spi_close(lgw_spi_target);
    }

    /* open the SPI link */
    spi_stat = lgw_spi_open(&lgw_spi_target);
    if (spi_stat != LGW_SPI_SUCCESS) {
        DEBUG_MSG("ERROR CONNECTING CONCENTRATOR\n");
        return LGW_REG_ERROR;
    }

    /* check SX1301 version */
    spi_stat = lgw_spi_r(lgw_spi_target, LGW_SPI_MUX_TARGET_SX1302, loregs[SX1302_REG_COMMON_VERSION_VERSION].addr, &u);
    if (spi_stat != LGW_SPI_SUCCESS) {
        DEBUG_MSG("ERROR READING CHIP VERSION REGISTER\n");
        return LGW_REG_ERROR;
    }
    if (u != loregs[SX1302_REG_COMMON_VERSION_VERSION].dflt) {
        DEBUG_PRINTF("ERROR: NOT EXPECTED CHIP VERSION (v%u)\n", u);
        return LGW_REG_ERROR;
    }
    DEBUG_PRINTF("Note: chip version is 0x%02X\n", u);

    DEBUG_MSG("Note: success connecting the concentrator\n");
    return LGW_REG_SUCCESS;
}

/* ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ */

/* Concentrator disconnect */
int lgw_disconnect(void) {
    if (lgw_spi_target != NULL) {
        lgw_spi_close(lgw_spi_target);
        lgw_spi_target = NULL;
        DEBUG_MSG("Note: success disconnecting the concentrator\n");
        return LGW_REG_SUCCESS;
    } else {
        DEBUG_MSG("WARNING: concentrator was already disconnected\n");
        return LGW_REG_ERROR;
    }
}

/* ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ */

/* Write to a register addressed by name */
int lgw_reg_w(uint16_t register_id, int32_t reg_value) {
    int spi_stat = LGW_SPI_SUCCESS;
    struct lgw_reg_s r;

    /* check input parameters */
    if (register_id >= LGW_TOTALREGS) {
        DEBUG_MSG("ERROR: REGISTER NUMBER OUT OF DEFINED RANGE\n");
        return LGW_REG_ERROR;
    }

    /* check if SPI is initialised */
    if (lgw_spi_target == NULL) {
        DEBUG_MSG("ERROR: CONCENTRATOR UNCONNECTED\n");
        return LGW_REG_ERROR;
    }

    /* get register struct from the struct array */
    r = loregs[register_id];

    /* reject write to read-only registers */
    if (r.rdon == 1){
        DEBUG_MSG("ERROR: TRYING TO WRITE A READ-ONLY REGISTER\n");
        return LGW_REG_ERROR;
    }

    spi_stat += reg_w_align32(lgw_spi_target, LGW_SPI_MUX_TARGET_SX1302, r, reg_value);

    if (spi_stat != LGW_SPI_SUCCESS) {
        DEBUG_MSG("ERROR: SPI ERROR DURING REGISTER WRITE\n");
        return LGW_REG_ERROR;
    } else {
        return LGW_REG_SUCCESS;
    }
}

/* ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ */

/* Read to a register addressed by name */
int lgw_reg_r(uint16_t register_id, int32_t *reg_value) {
    int spi_stat = LGW_SPI_SUCCESS;
    struct lgw_reg_s r;

    /* check input parameters */
    CHECK_NULL(reg_value);
    if (register_id >= LGW_TOTALREGS) {
        DEBUG_MSG("ERROR: REGISTER NUMBER OUT OF DEFINED RANGE\n");
        return LGW_REG_ERROR;
    }

    /* check if SPI is initialised */
    if (lgw_spi_target == NULL) {
        DEBUG_MSG("ERROR: CONCENTRATOR UNCONNECTED\n");
        return LGW_REG_ERROR;
    }

    /* get register struct from the struct array */
    r = loregs[register_id];

    spi_stat += reg_r_align32(lgw_spi_target, LGW_SPI_MUX_TARGET_SX1302, r, reg_value);

    if (spi_stat != LGW_SPI_SUCCESS) {
        DEBUG_MSG("ERROR: SPI ERROR DURING REGISTER WRITE\n");
        return LGW_REG_ERROR;
    } else {
        return LGW_REG_SUCCESS;
    }
}

/* ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ */

/* Point to a register by name and do a burst write */
int lgw_reg_wb(uint16_t register_id, uint8_t *data, uint16_t size) {
    int spi_stat = LGW_SPI_SUCCESS;
    struct lgw_reg_s r;

    /* check input parameters */
    CHECK_NULL(data);
    if (size == 0) {
        DEBUG_MSG("ERROR: BURST OF NULL LENGTH\n");
        return LGW_REG_ERROR;
    }
    if (register_id >= LGW_TOTALREGS) {
        DEBUG_MSG("ERROR: REGISTER NUMBER OUT OF DEFINED RANGE\n");
        return LGW_REG_ERROR;
    }

    /* check if SPI is initialised */
    if (lgw_spi_target == NULL) {
        DEBUG_MSG("ERROR: CONCENTRATOR UNCONNECTED\n");
        return LGW_REG_ERROR;
    }

    /* get register struct from the struct array */
    r = loregs[register_id];

    /* reject write to read-only registers */
    if (r.rdon == 1){
        DEBUG_MSG("ERROR: TRYING TO BURST WRITE A READ-ONLY REGISTER\n");
        return LGW_REG_ERROR;
    }

    /* do the burst write */
    spi_stat += lgw_spi_wb(lgw_spi_target, LGW_SPI_MUX_TARGET_SX1302, r.addr, data, size);

    if (spi_stat != LGW_SPI_SUCCESS) {
        DEBUG_MSG("ERROR: SPI ERROR DURING REGISTER BURST WRITE\n");
        return LGW_REG_ERROR;
    } else {
        return LGW_REG_SUCCESS;
    }
}

/* ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ */

/* Point to a register by name and do a burst read */
int lgw_reg_rb(uint16_t register_id, uint8_t *data, uint16_t size) {
    int spi_stat = LGW_SPI_SUCCESS;
    struct lgw_reg_s r;

    /* check input parameters */
    CHECK_NULL(data);
    if (size == 0) {
        DEBUG_MSG("ERROR: BURST OF NULL LENGTH\n");
        return LGW_REG_ERROR;
    }
    if (register_id >= LGW_TOTALREGS) {
        DEBUG_MSG("ERROR: REGISTER NUMBER OUT OF DEFINED RANGE\n");
        return LGW_REG_ERROR;
    }

    /* check if SPI is initialised */
    if (lgw_spi_target == NULL) {
        DEBUG_MSG("ERROR: CONCENTRATOR UNCONNECTED\n");
        return LGW_REG_ERROR;
    }

    /* get register struct from the struct array */
    r = loregs[register_id];

    /* do the burst read */
    spi_stat += lgw_spi_rb(lgw_spi_target, LGW_SPI_MUX_TARGET_SX1302, r.addr, data, size);

    if (spi_stat != LGW_SPI_SUCCESS) {
        DEBUG_MSG("ERROR: SPI ERROR DURING REGISTER BURST READ\n");
        return LGW_REG_ERROR;
    } else {
        return LGW_REG_SUCCESS;
    }
}

/* --- EOF ------------------------------------------------------------------ */
