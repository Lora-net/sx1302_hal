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

/* fix an issue between POSIX and C99 */
#if __STDC_VERSION__ >= 199901L
    #define _XOPEN_SOURCE 600
#else
    #define _XOPEN_SOURCE 500
#endif

#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <math.h>
#include <assert.h>

#include "loragw_reg.h"
#include "loragw_sx1250.h"
#include "loragw_sx1302.h"
#include "loragw_aux.h"

/* -------------------------------------------------------------------------- */
/* --- PRIVATE MACROS ------------------------------------------------------- */

#define SX1250_FREQ_TO_REG(f)       (uint32_t)((uint64_t)f * (1 << 25) / 32000000U)
#define SX1302_FREQ_TO_REG(f)       (uint32_t)((uint64_t)f * (1 << 18) / 32000000U)
#define IF_HZ_TO_REG(f)             ((f << 5) / 15625)

/* -------------------------------------------------------------------------- */
/* --- PRIVATE CONSTANTS ---------------------------------------------------- */

#define RADIO_A             0

#define BUFF_SIZE           1024
#define DEFAULT_FREQ_HZ     868500000U

#define MOD_CW              0
#define MOD_LORA            1
#define MOD_GFSK            2

const int32_t channel_if[8] = {
    700000,
    500000,
    300000,
    100000,
    -100000,
    -300000,
    -500000,
    -700000
};

/* -------------------------------------------------------------------------- */
/* --- PRIVATE VARIABLES ---------------------------------------------------- */

#include "src/text_arb_sx1302_18_sep_3.var"

static uint32_t nb_pkt_received = 0;
static uint16_t fcnt_prev = 0xFFFF;

/* -------------------------------------------------------------------------- */
/* --- PRIVATE FUNCTIONS ---------------------------------------------------- */

int sx1250_init(uint32_t freq_hz) {
    int32_t freq_reg;
    uint8_t buff[16];

    /* Enable and reset the radio */
    sx1302_radio_reset(RADIO_A, SX1302_RADIO_TYPE_SX1250);

    /* Set radio mode */
    sx1302_radio_set_mode(RADIO_A, SX1302_RADIO_TYPE_SX1250);

    /* Enable 32 MHz oscillator */
    buff[0] = (uint8_t)STDBY_XOSC;
    sx1250_write_command(RADIO_A, SET_STANDBY, buff, 1);
    buff[0] = 0x00;
    sx1250_read_command(RADIO_A, GET_STATUS, buff, 1);
    printf("%s: get_status: 0x%02X\n", __FUNCTION__, buff[0]);

    /* Configure DIO for Rx (not necessary here, just for reference) */
    buff[0] = 0x05;
    buff[1] = 0x82;
    buff[2] = 0x00;
    sx1250_write_command(RADIO_A, WRITE_REGISTER, buff, 3); /* Drive strength to min */
    buff[0] = 0x05;
    buff[1] = 0x83;
    buff[2] = 0x00;
    sx1250_write_command(RADIO_A, WRITE_REGISTER, buff, 3); /* Input enable, all disabled */
    buff[0] = 0x05;
    buff[1] = 0x84;
    buff[2] = 0x00;
    sx1250_write_command(RADIO_A, WRITE_REGISTER, buff, 3); /* No pull up */
    buff[0] = 0x05;
    buff[1] = 0x85;
    buff[2] = 0x00;
    sx1250_write_command(RADIO_A, WRITE_REGISTER, buff, 3); /* No pull down */
    buff[0] = 0x05;
    buff[1] = 0x80;
    buff[2] = 0x00;
    sx1250_write_command(RADIO_A, WRITE_REGISTER, buff, 3); /* Output enable, all enabled */

    /* TODO ?? */
    buff[0] = 0x08;
    buff[1] = 0xB6;
    buff[2] = 0x2A;
    sx1250_write_command(RADIO_A, WRITE_REGISTER, buff, 3); /* Fix gain 10 */

    /* Set frequency */
    freq_reg = SX1250_FREQ_TO_REG(freq_hz);
    buff[0] = (uint8_t)(freq_reg >> 24);
    buff[1] = (uint8_t)(freq_reg >> 16);
    buff[2] = (uint8_t)(freq_reg >> 8);
    buff[3] = (uint8_t)(freq_reg >> 0);
    sx1250_write_command(RADIO_A, SET_RF_FREQUENCY, buff, 4);

    /* Set frequency offset to 0 */
    buff[0] = 0x08;
    buff[1] = 0x8F;
    buff[2] = 0x00;
    buff[3] = 0x00;
    buff[4] = 0x00;
    sx1250_write_command(RADIO_A, WRITE_REGISTER, buff, 5);

    /* Set Rx */
    buff[0] = 0xFF;
    buff[1] = 0xFF;
    buff[2] = 0xFF;
    sx1250_write_command(RADIO_A, SET_RX, buff, 3);

    buff[0] = 0x05;
    buff[1] = 0x87;
    buff[2] = 0x08;
    sx1250_write_command(RADIO_A, WRITE_REGISTER, buff, 3); /* FPGA_MODE_RX */

    /* Select the radio which provides the clock to the sx1302 */
    sx1302_radio_clock_select(RADIO_A);

    return 0;
}

int sx1250_set_idle(void) {
    uint8_t buff[16];

    buff[0] = (uint8_t)STDBY_XOSC;
    sx1250_write_command(RADIO_A, SET_STANDBY, buff, 1);

    buff[0] = 0x05;
    buff[1] = 0x87;
    buff[2] = 0x0E;
    sx1250_write_command(RADIO_A, WRITE_REGISTER, buff, 3); /* Default value */

    return 0;
}

int load_firmware_arb(const uint8_t *firmware) {
    int i;
    uint8_t fw_check[8192];
    int32_t gpio_sel = 0x02; /* ARB MCU */
    int32_t val;

    /* Configure GPIO to let AGC MCU access board LEDs */
    lgw_reg_w(SX1302_REG_GPIO_GPIO_SEL_0_SELECTION, gpio_sel);
    lgw_reg_w(SX1302_REG_GPIO_GPIO_SEL_1_SELECTION, gpio_sel);
    lgw_reg_w(SX1302_REG_GPIO_GPIO_SEL_2_SELECTION, gpio_sel);
    lgw_reg_w(SX1302_REG_GPIO_GPIO_SEL_3_SELECTION, gpio_sel);
    lgw_reg_w(SX1302_REG_GPIO_GPIO_SEL_4_SELECTION, gpio_sel);
    lgw_reg_w(SX1302_REG_GPIO_GPIO_SEL_5_SELECTION, gpio_sel);
    lgw_reg_w(SX1302_REG_GPIO_GPIO_SEL_6_SELECTION, gpio_sel);
    lgw_reg_w(SX1302_REG_GPIO_GPIO_SEL_7_SELECTION, gpio_sel);
    lgw_reg_w(SX1302_REG_GPIO_GPIO_DIR_L_DIRECTION, 0xFF); /* GPIO output direction */

    /* Take control over ARB MCU */
    lgw_reg_w(SX1302_REG_ARB_MCU_CTRL_MCU_CLEAR, 0x01);
    lgw_reg_w(SX1302_REG_ARB_MCU_CTRL_HOST_PROG, 0x01);
    lgw_reg_w(SX1302_REG_COMMON_PAGE_PAGE, 0x00);

    /* Write AGC fw in AGC MEM */
    for (i = 0; i < 8; i++) {
        lgw_mem_wb(0x2000+(i*1024), &firmware[i*1024], 1024);
    }

    /* Read back and check */
    for (i = 0; i < 8; i++) {
        lgw_mem_rb(0x2000+(i*1024), &fw_check[i*1024], 1024);
    }
    if (memcmp(firmware, fw_check, 8192) != 0) {
        printf ("ERROR: Failed to load fw\n");
        return -1;
    }

    /* Release control over AGC MCU */
    lgw_reg_w(SX1302_REG_ARB_MCU_CTRL_HOST_PROG, 0x00);
    lgw_reg_w(SX1302_REG_ARB_MCU_CTRL_MCU_CLEAR, 0x00);

    lgw_reg_r(SX1302_REG_ARB_MCU_CTRL_PARITY_ERROR, &val);
    printf("ARB fw loaded (parity error:0x%02X)\n", val);

    return 0;
}

int sx1250_configure_channels(void) {
    int32_t cnt, cnt2;
    int32_t if_freq;

    /* Configure channelizer */
    lgw_reg_w(SX1302_REG_RX_TOP_RADIO_SELECT_RADIO_SELECT, 0x00); /* all on RadioA */

    if_freq = IF_HZ_TO_REG(channel_if[0]);
    lgw_reg_w(SX1302_REG_RX_TOP_FREQ_0_MSB_IF_FREQ_0, (if_freq >> 8) & 0x0000001F);
    lgw_reg_w(SX1302_REG_RX_TOP_FREQ_0_LSB_IF_FREQ_0, (if_freq >> 0) & 0x000000FF);

    if_freq = IF_HZ_TO_REG(channel_if[1]);
    lgw_reg_w(SX1302_REG_RX_TOP_FREQ_1_MSB_IF_FREQ_1, (if_freq >> 8) & 0x0000001F);
    lgw_reg_w(SX1302_REG_RX_TOP_FREQ_1_LSB_IF_FREQ_1, (if_freq >> 0) & 0x000000FF);

    if_freq = IF_HZ_TO_REG(channel_if[2]);
    lgw_reg_w(SX1302_REG_RX_TOP_FREQ_2_MSB_IF_FREQ_2, (if_freq >> 8) & 0x0000001F);
    lgw_reg_w(SX1302_REG_RX_TOP_FREQ_2_LSB_IF_FREQ_2, (if_freq >> 0) & 0x000000FF);

    if_freq = IF_HZ_TO_REG(channel_if[3]);
    lgw_reg_w(SX1302_REG_RX_TOP_FREQ_3_MSB_IF_FREQ_3, (if_freq >> 8) & 0x0000001F);
    lgw_reg_w(SX1302_REG_RX_TOP_FREQ_3_LSB_IF_FREQ_3, (if_freq >> 0) & 0x000000FF);

    if_freq = IF_HZ_TO_REG(channel_if[4]);
    lgw_reg_w(SX1302_REG_RX_TOP_FREQ_4_MSB_IF_FREQ_4, (if_freq >> 8) & 0x0000001F);
    lgw_reg_w(SX1302_REG_RX_TOP_FREQ_4_LSB_IF_FREQ_4, (if_freq >> 0) & 0x000000FF);

    if_freq = IF_HZ_TO_REG(channel_if[5]);
    lgw_reg_w(SX1302_REG_RX_TOP_FREQ_5_MSB_IF_FREQ_5, (if_freq >> 8) & 0x0000001F);
    lgw_reg_w(SX1302_REG_RX_TOP_FREQ_5_LSB_IF_FREQ_5, (if_freq >> 0) & 0x000000FF);

    if_freq = IF_HZ_TO_REG(channel_if[6]);
    lgw_reg_w(SX1302_REG_RX_TOP_FREQ_6_MSB_IF_FREQ_6, (if_freq >> 8) & 0x0000001F);
    lgw_reg_w(SX1302_REG_RX_TOP_FREQ_6_LSB_IF_FREQ_6, (if_freq >> 0) & 0x000000FF);

    if_freq = IF_HZ_TO_REG(channel_if[7]);
    lgw_reg_w(SX1302_REG_RX_TOP_FREQ_7_MSB_IF_FREQ_7, (if_freq >> 8) & 0x0000001F);
    lgw_reg_w(SX1302_REG_RX_TOP_FREQ_7_LSB_IF_FREQ_7, (if_freq >> 0) & 0x000000FF);

    /* Configure correlators */
    lgw_reg_w(SX1302_REG_RX_TOP_SF7_CFG1_ACC_2_SAME_PEAKS, 1);
    lgw_reg_w(SX1302_REG_RX_TOP_SF7_CFG1_ACC_AUTO_RESCALE, 1);
    lgw_reg_w(SX1302_REG_RX_TOP_SF7_CFG1_ACC_COEFF, 2);
    lgw_reg_w(SX1302_REG_RX_TOP_SF7_CFG1_ACC_PEAK_POS_SEL, 1);
    lgw_reg_w(SX1302_REG_RX_TOP_SF7_CFG1_ACC_PEAK_SUM_EN, 1);
    lgw_reg_w(SX1302_REG_RX_TOP_SF7_CFG3_MIN_SINGLE_PEAK, 11);
    lgw_reg_w(SX1302_REG_RX_TOP_SF7_CFG6_MSP_CNT_MODE, 0);
    lgw_reg_w(SX1302_REG_RX_TOP_SF7_CFG6_MSP_POS_SEL, 1);
    lgw_reg_w(SX1302_REG_RX_TOP_SF7_CFG7_NOISE_COEFF, 2);
    lgw_reg_w(SX1302_REG_RX_TOP_SF7_CFG2_ACC_PNR, 55);
    lgw_reg_w(SX1302_REG_RX_TOP_SF7_CFG4_MSP_PNR, 55);
    lgw_reg_w(SX1302_REG_RX_TOP_SF7_CFG5_MSP2_PNR, 55);
    lgw_reg_w(SX1302_REG_RX_TOP_SF7_CFG6_MSP_PEAK_NB, 3);
    lgw_reg_w(SX1302_REG_RX_TOP_SF7_CFG7_MSP2_PEAK_NB, 3);

    lgw_reg_w(SX1302_REG_RX_TOP_SF8_CFG1_ACC_2_SAME_PEAKS, 0);
    lgw_reg_w(SX1302_REG_RX_TOP_SF8_CFG1_ACC_AUTO_RESCALE, 1);
    lgw_reg_w(SX1302_REG_RX_TOP_SF8_CFG1_ACC_COEFF, 2);
    lgw_reg_w(SX1302_REG_RX_TOP_SF8_CFG1_ACC_PEAK_POS_SEL, 1);
    lgw_reg_w(SX1302_REG_RX_TOP_SF8_CFG1_ACC_PEAK_SUM_EN, 1);
    lgw_reg_w(SX1302_REG_RX_TOP_SF8_CFG3_MIN_SINGLE_PEAK, 11);
    lgw_reg_w(SX1302_REG_RX_TOP_SF8_CFG6_MSP_CNT_MODE, 0);
    lgw_reg_w(SX1302_REG_RX_TOP_SF8_CFG6_MSP_POS_SEL, 1);
    lgw_reg_w(SX1302_REG_RX_TOP_SF8_CFG7_NOISE_COEFF, 2);
    lgw_reg_w(SX1302_REG_RX_TOP_SF8_CFG2_ACC_PNR, 56);
    lgw_reg_w(SX1302_REG_RX_TOP_SF8_CFG4_MSP_PNR, 56);
    lgw_reg_w(SX1302_REG_RX_TOP_SF8_CFG5_MSP2_PNR, 56);
    lgw_reg_w(SX1302_REG_RX_TOP_SF8_CFG6_MSP_PEAK_NB, 3);
    lgw_reg_w(SX1302_REG_RX_TOP_SF8_CFG7_MSP2_PEAK_NB, 3);

    lgw_reg_w(SX1302_REG_RX_TOP_SF9_CFG1_ACC_2_SAME_PEAKS, 0);
    lgw_reg_w(SX1302_REG_RX_TOP_SF9_CFG1_ACC_AUTO_RESCALE, 1);
    lgw_reg_w(SX1302_REG_RX_TOP_SF9_CFG1_ACC_COEFF, 2);
    lgw_reg_w(SX1302_REG_RX_TOP_SF9_CFG1_ACC_PEAK_POS_SEL, 1);
    lgw_reg_w(SX1302_REG_RX_TOP_SF9_CFG1_ACC_PEAK_SUM_EN, 1);
    lgw_reg_w(SX1302_REG_RX_TOP_SF9_CFG3_MIN_SINGLE_PEAK, 11);
    lgw_reg_w(SX1302_REG_RX_TOP_SF9_CFG6_MSP_CNT_MODE, 0);
    lgw_reg_w(SX1302_REG_RX_TOP_SF9_CFG6_MSP_POS_SEL, 1);
    lgw_reg_w(SX1302_REG_RX_TOP_SF9_CFG7_NOISE_COEFF, 2);
    lgw_reg_w(SX1302_REG_RX_TOP_SF9_CFG2_ACC_PNR, 58);
    lgw_reg_w(SX1302_REG_RX_TOP_SF9_CFG4_MSP_PNR, 58);
    lgw_reg_w(SX1302_REG_RX_TOP_SF9_CFG5_MSP2_PNR, 58);
    lgw_reg_w(SX1302_REG_RX_TOP_SF9_CFG6_MSP_PEAK_NB, 3);
    lgw_reg_w(SX1302_REG_RX_TOP_SF9_CFG7_MSP2_PEAK_NB, 3);

    lgw_reg_w(SX1302_REG_RX_TOP_SF10_CFG1_ACC_2_SAME_PEAKS, 0);
    lgw_reg_w(SX1302_REG_RX_TOP_SF10_CFG1_ACC_AUTO_RESCALE, 1);
    lgw_reg_w(SX1302_REG_RX_TOP_SF10_CFG1_ACC_COEFF, 2);
    lgw_reg_w(SX1302_REG_RX_TOP_SF10_CFG1_ACC_PEAK_POS_SEL, 1);
    lgw_reg_w(SX1302_REG_RX_TOP_SF10_CFG1_ACC_PEAK_SUM_EN, 1);
    lgw_reg_w(SX1302_REG_RX_TOP_SF10_CFG3_MIN_SINGLE_PEAK, 11);
    lgw_reg_w(SX1302_REG_RX_TOP_SF10_CFG6_MSP_CNT_MODE, 0);
    lgw_reg_w(SX1302_REG_RX_TOP_SF10_CFG6_MSP_POS_SEL, 1);
    lgw_reg_w(SX1302_REG_RX_TOP_SF10_CFG7_NOISE_COEFF, 2);
    lgw_reg_w(SX1302_REG_RX_TOP_SF10_CFG2_ACC_PNR, 60);
    lgw_reg_w(SX1302_REG_RX_TOP_SF10_CFG4_MSP_PNR, 60);
    lgw_reg_w(SX1302_REG_RX_TOP_SF10_CFG5_MSP2_PNR, 60);
    lgw_reg_w(SX1302_REG_RX_TOP_SF10_CFG6_MSP_PEAK_NB, 3);
    lgw_reg_w(SX1302_REG_RX_TOP_SF10_CFG7_MSP2_PEAK_NB, 3);

    lgw_reg_w(SX1302_REG_RX_TOP_SF11_CFG1_ACC_2_SAME_PEAKS, 0);
    lgw_reg_w(SX1302_REG_RX_TOP_SF11_CFG1_ACC_AUTO_RESCALE, 1);
    lgw_reg_w(SX1302_REG_RX_TOP_SF11_CFG1_ACC_COEFF, 2);
    lgw_reg_w(SX1302_REG_RX_TOP_SF11_CFG1_ACC_PEAK_POS_SEL, 1);
    lgw_reg_w(SX1302_REG_RX_TOP_SF11_CFG1_ACC_PEAK_SUM_EN, 1);
    lgw_reg_w(SX1302_REG_RX_TOP_SF11_CFG3_MIN_SINGLE_PEAK, 11);
    lgw_reg_w(SX1302_REG_RX_TOP_SF11_CFG6_MSP_CNT_MODE, 0);
    lgw_reg_w(SX1302_REG_RX_TOP_SF11_CFG6_MSP_POS_SEL, 1);
    lgw_reg_w(SX1302_REG_RX_TOP_SF11_CFG7_NOISE_COEFF, 2);
    lgw_reg_w(SX1302_REG_RX_TOP_SF11_CFG2_ACC_PNR, 60);
    lgw_reg_w(SX1302_REG_RX_TOP_SF11_CFG4_MSP_PNR, 60);
    lgw_reg_w(SX1302_REG_RX_TOP_SF11_CFG5_MSP2_PNR, 60);
    lgw_reg_w(SX1302_REG_RX_TOP_SF11_CFG6_MSP_PEAK_NB, 3);
    lgw_reg_w(SX1302_REG_RX_TOP_SF11_CFG7_MSP2_PEAK_NB, 3);

    lgw_reg_w(SX1302_REG_RX_TOP_SF12_CFG1_ACC_2_SAME_PEAKS, 0);
    lgw_reg_w(SX1302_REG_RX_TOP_SF12_CFG1_ACC_AUTO_RESCALE, 1);
    lgw_reg_w(SX1302_REG_RX_TOP_SF12_CFG1_ACC_COEFF, 2);
    lgw_reg_w(SX1302_REG_RX_TOP_SF12_CFG1_ACC_PEAK_POS_SEL, 1);
    lgw_reg_w(SX1302_REG_RX_TOP_SF12_CFG1_ACC_PEAK_SUM_EN, 1);
    lgw_reg_w(SX1302_REG_RX_TOP_SF12_CFG3_MIN_SINGLE_PEAK, 11);
    lgw_reg_w(SX1302_REG_RX_TOP_SF12_CFG6_MSP_CNT_MODE, 0);
    lgw_reg_w(SX1302_REG_RX_TOP_SF12_CFG6_MSP_POS_SEL, 1);
    lgw_reg_w(SX1302_REG_RX_TOP_SF12_CFG7_NOISE_COEFF, 2);
    lgw_reg_w(SX1302_REG_RX_TOP_SF12_CFG2_ACC_PNR, 60);
    lgw_reg_w(SX1302_REG_RX_TOP_SF12_CFG4_MSP_PNR, 60);
    lgw_reg_w(SX1302_REG_RX_TOP_SF12_CFG5_MSP2_PNR, 60);
    lgw_reg_w(SX1302_REG_RX_TOP_SF12_CFG6_MSP_PEAK_NB, 3);
    lgw_reg_w(SX1302_REG_RX_TOP_SF12_CFG7_MSP2_PEAK_NB, 3);

    lgw_reg_w(SX1302_REG_RX_TOP_CORRELATOR_SF_EN_CORR_SF_EN, 0xFC); /* 12 11 10 9 8 7 6 5 */
    lgw_reg_w(SX1302_REG_RX_TOP_CORRELATOR_EN_CORR_EN, 0xFF);

    /* Configure multi-sf */
    lgw_reg_w(SX1302_REG_RX_TOP_DC_NOTCH_CFG1_ENABLE, 0x00);
    lgw_reg_w(SX1302_REG_RX_TOP_RX_DFE_AGC1_FREEZE_ON_SYNC, 0x00);
    lgw_reg_w(SX1302_REG_RX_TOP_RX_DFE_AGC1_FORCE_DEFAULT_FIR, 0x01);
    lgw_reg_w(SX1302_REG_RX_TOP_RX_CFG0_SWAP_IQ, 0x00);
    lgw_reg_w(SX1302_REG_RX_TOP_RX_CFG0_CHIRP_INVERT, 0x01);
    lgw_reg_w(SX1302_REG_RX_TOP_MODEM_SYNC_DELTA_LSB_MODEM_SYNC_DELTA, 7);
    lgw_reg_w(SX1302_REG_RX_TOP_MODEM_SYNC_DELTA_MSB_MODEM_SYNC_DELTA, 0);
    lgw_reg_w(SX1302_REG_OTP_MODEM_EN_0_MODEM_EN, 0x03); /* Only 2 full modems available on FPGA */
    lgw_reg_w(SX1302_REG_OTP_MODEM_EN_1_MODEM_EN, 0x00); /* No limited modem available */
    lgw_reg_w(SX1302_REG_RX_TOP_MODEM_PPM_OFFSET1_PPM_OFFSET_SF5, 0x00);
    lgw_reg_w(SX1302_REG_RX_TOP_MODEM_PPM_OFFSET1_PPM_OFFSET_SF6, 0x00);
    lgw_reg_w(SX1302_REG_RX_TOP_MODEM_PPM_OFFSET1_PPM_OFFSET_SF7, 0x00);
    lgw_reg_w(SX1302_REG_RX_TOP_MODEM_PPM_OFFSET1_PPM_OFFSET_SF8, 0x00);
    lgw_reg_w(SX1302_REG_RX_TOP_MODEM_PPM_OFFSET2_PPM_OFFSET_SF9, 0x00);
    lgw_reg_w(SX1302_REG_RX_TOP_MODEM_PPM_OFFSET2_PPM_OFFSET_SF10, 0x00);
    lgw_reg_w(SX1302_REG_RX_TOP_MODEM_PPM_OFFSET2_PPM_OFFSET_SF11, 0x01);
    lgw_reg_w(SX1302_REG_RX_TOP_MODEM_PPM_OFFSET2_PPM_OFFSET_SF12, 0x01);

    /* Configure Syncwork Public/Private */
    lgw_reg_w(SX1302_REG_RX_TOP_FRAME_SYNCH0_PEAK1_POS, 6);
    lgw_reg_w(SX1302_REG_RX_TOP_FRAME_SYNCH1_PEAK2_POS, 8);

    /* Configure agc */
    lgw_reg_w(SX1302_REG_RADIO_FE_RSSI_BB_FILTER_ALPHA_RADIO_A_RSSI_BB_FILTER_ALPHA, 0x06);
    lgw_reg_w(SX1302_REG_RADIO_FE_RSSI_DEC_FILTER_ALPHA_RADIO_A_RSSI_DEC_FILTER_ALPHA, 0x07);
    lgw_reg_w(SX1302_REG_RADIO_FE_RSSI_DB_DEF_RADIO_A_RSSI_DB_DEFAULT_VALUE, 23);
    lgw_reg_w(SX1302_REG_RADIO_FE_RSSI_DEC_DEF_RADIO_A_RSSI_DEC_DEFAULT_VALUE, 66);
    lgw_reg_w(SX1302_REG_RX_TOP_RSSI_CONTROL_RSSI_FILTER_ALPHA, 0x00);
    lgw_reg_w(SX1302_REG_RX_TOP_RSSI_DEF_VALUE_CHAN_RSSI_DEF_VALUE, 85);
    lgw_reg_w(SX1302_REG_RADIO_FE_CTRL0_RADIO_A_DC_NOTCH_EN, 0x00);
    lgw_reg_w(SX1302_REG_RADIO_FE_CTRL0_RADIO_A_HOST_FILTER_GAIN, 0x08);
    lgw_reg_w(SX1302_REG_COMMON_CTRL0_HOST_RADIO_CTRL, 0x01);
    lgw_reg_w(SX1302_REG_RADIO_FE_CTRL0_RADIO_A_FORCE_HOST_FILTER_GAIN, 0x01);
    lgw_reg_w(SX1302_REG_RX_TOP_GAIN_CONTROL_CHAN_GAIN_VALID, 0x01);
    lgw_reg_w(SX1302_REG_RX_TOP_GAIN_CONTROL_CHAN_GAIN, 0x0F);

    /* Check that the SX1302 timestamp counter is running */
    lgw_reg_r(SX1302_REG_TIMESTAMP_TIMESTAMP_LSB1_TIMESTAMP, &cnt);
    lgw_reg_r(SX1302_REG_TIMESTAMP_TIMESTAMP_LSB1_TIMESTAMP, &cnt2);
    if (cnt == cnt2) {
        printf("ERROR: SX1302 timestamp counter is not running (val:%u)\n", cnt);
        return -1;
    }

    /* load ARB fw */
    load_firmware_arb(arb_firmware);

    /* Enable modem */
    lgw_reg_w(SX1302_REG_COMMON_GEN_CONCENTRATOR_MODEM_ENABLE, 0x01);
    lgw_reg_w(SX1302_REG_COMMON_GEN_FSK_MODEM_ENABLE, 0x01);
    lgw_reg_w(SX1302_REG_COMMON_GEN_GLOBAL_EN, 0x01);

    return 0;
}

int sx1250_receive(void) {
    uint16_t nb_bytes;
    uint8_t buff[2];
    uint8_t fifo[1024];
    int i;
    int idx;
    uint8_t payload_size;
    uint32_t count_us;
    uint16_t fcnt;

    lgw_reg_rb(SX1302_REG_RX_TOP_RX_BUFFER_NB_BYTES_MSB_RX_BUFFER_NB_BYTES, buff, sizeof buff);
    nb_bytes  = (uint16_t)((buff[0] << 8) & 0xFF00);
    nb_bytes |= (uint16_t)((buff[1] << 0) & 0x00FF);

    if (nb_bytes > 1024) {
        printf("ERROR: more than 1024 bytes in the FIFO, to be reworked\n");
        assert(0);
    }

    if (nb_bytes > 0) {
        printf("nb_bytes received: %u (%u %u)\n", nb_bytes, buff[1], buff[0]);

        /* read bytes from fifo */
        memset(fifo, 0, sizeof fifo);
        lgw_mem_rb(0x4000, fifo, nb_bytes);
        for (i = 0; i < nb_bytes; i++) {
            printf("%02X ", fifo[i]);
        }
        printf("\n");

        /* parse packet */
        idx = 0;
        while (idx < nb_bytes) {
            if ((fifo[idx] == 0xA5) && (fifo[idx+1] == 0xC0)) {
                nb_pkt_received += 1;
                /* we found the start of a packet, parse it */
                fcnt = (fifo[idx+6+7] << 8) | fifo[idx+6+6];
                printf("\n----- new packet (%u) (fcnt:%u, missed:%u)-----\n", nb_pkt_received, fcnt, (fcnt - fcnt_prev - 1));
                fcnt_prev = fcnt;
                payload_size = fifo[idx+2];
                printf("  size:     %u\n", payload_size);
                printf("  chan:     %u\n", fifo[idx+3]);
                printf("  crc_en:   %u\n", TAKE_N_BITS_FROM(fifo[idx+4], 0, 1));
                printf("  codr:     %u\n", TAKE_N_BITS_FROM(fifo[idx+4], 1, 3));
                printf("  datr:     %u\n", TAKE_N_BITS_FROM(fifo[idx+4], 4, 4));
                printf("  modem:    %u\n", fifo[idx+5]);
                printf("  payload: ");
                for (i = 0; i < payload_size; i++) {
                    printf("%02X ", fifo[idx+6+i]);
                }
                printf("\n");
                printf("  status:   %u\n", TAKE_N_BITS_FROM(fifo[idx+6+payload_size], 0, 1));
                printf("  snr_avg:  %d\n", fifo[idx+7+payload_size]);
                printf("  rssi_chan:%d\n", fifo[idx+8+payload_size]);
                printf("  rssi_sig: %d\n", fifo[idx+9+payload_size]);
                count_us  = (uint32_t)((fifo[idx+12+payload_size] <<  0) & 0x000000FF);
                count_us |= (uint32_t)((fifo[idx+13+payload_size] <<  8) & 0x0000FF00);
                count_us |= (uint32_t)((fifo[idx+14+payload_size] << 16) & 0x00FF0000);
                count_us |= (uint32_t)((fifo[idx+15+payload_size] << 24) & 0xFF000000);
                printf("  timestamp:%u (count_us:%u)\n", count_us, count_us/32);
            }
            idx += 1;
        }

    }

    return 0;
}

/* -------------------------------------------------------------------------- */
/* --- MAIN FUNCTION -------------------------------------------------------- */

/* describe command line options */
void usage(void) {
    //printf("Library version information: %s\n", lgw_version_info());
    printf( "Available options:\n");
    printf( " -h print this help\n");
    printf( " -f <float> Radio RX frequency in MHz\n");
}

int main(int argc, char **argv)
{
    int i, x;
    uint32_t ft = DEFAULT_FREQ_HZ;
    double arg_d = 0.0;

    /* parse command line options */
    while ((i = getopt (argc, argv, "hf:")) != -1) {
        switch (i) {
            case 'h':
                usage();
                return -1;
                break;
            case 'f': /* <float> Radio TX frequency in MHz */
                i = sscanf(optarg, "%lf", &arg_d);
                if (i != 1) {
                    printf("ERROR: argument parsing of -f argument. Use -h to print help\n");
                    return EXIT_FAILURE;
                } else {
                    ft = (uint32_t)((arg_d*1e6) + 0.5); /* .5 Hz offset to get rounding instead of truncating */
                }
                break;
            default:
                printf("ERROR: argument parsing\n");
                usage();
                return -1;
        }
    }

    printf("===== sx1302 sx1250 RX test =====\n");

    /* Board reset */
    system("./reset_lgw.sh start");

    lgw_connect();

    sx1250_init(ft);

    printf("Channel configuration:\n");
    for (i = 0; i < 8; i++) {
        printf(" %d: %u Hz, if:%d Hz, reg:%d\n", i, (int32_t)ft + channel_if[i], channel_if[i], IF_HZ_TO_REG(channel_if[i]));
    }

    x = sx1250_configure_channels();
    if (x != 0) {
        printf("ERROR: failed to configure channels\n");
        return -1;
    }

    printf("Waiting for packets...\n");
    while (1) {
        sx1250_receive();
        wait_ms(10);
    }

    sx1250_set_idle();

    lgw_disconnect();
    printf("=========== Test End ===========\n");

    return 0;
}

/* --- EOF ------------------------------------------------------------------ */
