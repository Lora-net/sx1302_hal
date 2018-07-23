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

#include "loragw_hal.h"
#include "loragw_reg.h"
#include "loragw_sx1262fe.h"
#include "loragw_aux.h"

/* -------------------------------------------------------------------------- */
/* --- PRIVATE MACROS ------------------------------------------------------- */

#define SX1262FE_FREQ_TO_REG(f)     (uint32_t)((uint64_t)f * (1 << 25) / 32000000U)
#define SX1302_FREQ_TO_REG(f)       (uint32_t)((uint64_t)f * (1 << 18) / 32000000U)

/* -------------------------------------------------------------------------- */
/* --- PRIVATE CONSTANTS ---------------------------------------------------- */

#define BUFF_SIZE           1024
#define DEFAULT_FREQ_HZ     868500000U
#define DEFAULT_SF          7U
#define DEFAULT_BW_HZ       125000U
#define DEFAULT_NB_PKT      1U

/* -------------------------------------------------------------------------- */
/* --- PRIVATE VARIABLES ---------------------------------------------------- */

#include "src/test_bao_sx1262.var"

/* -------------------------------------------------------------------------- */
/* --- PRIVATE FUNCTIONS ---------------------------------------------------- */

int sx1262fe_init(void) {
    uint8_t buff[16];
    uint32_t val, val2;

    lgw_reg_w(SX1302_REG_COMMON_CTRL0_SX1261_MODE_RADIO_A, 0x01);

    /* Enable Sx1262 and perform chip reset */
    lgw_reg_w(SX1302_REG_AGC_MCU_RF_EN_A_RADIO_EN, 0x01);
    lgw_reg_w(SX1302_REG_AGC_MCU_RF_EN_A_RADIO_RST, 0x01);
    wait_ms(500);
    lgw_reg_w(SX1302_REG_AGC_MCU_RF_EN_A_RADIO_RST, 0x00);
    wait_ms(10);
    lgw_reg_w(SX1302_REG_AGC_MCU_RF_EN_A_RADIO_RST, 0x01);

    /* Set Radio in Standby mode */
    buff[0] = (uint8_t)STDBY_XOSC;
    sx1262fe_write_command(SET_STANDBY, buff, 1);
    buff[0] = 0x00;
    sx1262fe_read_command(GET_STATUS, buff, 1);
    printf("%s: get_status: 0x%02X\n", __FUNCTION__, buff[0]);

    /* Configure DIO for Rx */
    buff[0] = 0x05;
    buff[1] = 0x82;
    buff[2] = 0x00;
    sx1262fe_write_command(WRITE_REGISTER, buff, 3); /* Drive strength to min */
    buff[0] = 0x05;
    buff[1] = 0x83;
    buff[2] = 0x00;
    sx1262fe_write_command(WRITE_REGISTER, buff, 3); /* Input enable, all disabled */
    buff[0] = 0x05;
    buff[1] = 0x84;
    buff[2] = 0x00;
    sx1262fe_write_command(WRITE_REGISTER, buff, 3); /* No pull up */
    buff[0] = 0x05;
    buff[1] = 0x85;
    buff[2] = 0x00;
    sx1262fe_write_command(WRITE_REGISTER, buff, 3); /* No pull down */
    buff[0] = 0x05;
    buff[1] = 0x80;
    buff[2] = 0x00;
    sx1262fe_write_command(WRITE_REGISTER, buff, 3); /* Output enable, all enabled */
    buff[0] = 0x05;
    buff[1] = 0x87;
    buff[2] = 0x08;
    sx1262fe_write_command(WRITE_REGISTER, buff, 3); /* FPGA_MODE_RX */

    /* Set Radio in Rx mode, necessary to give a clock to SX1302 */
    buff[0] = 0xFF;
    buff[1] = 0xFF;
    buff[2] = 0xFF;
    sx1262fe_write_command(SET_RX, buff, 3); /* Rx Continuous */

    /* Enable clock divider (Mandatory) */
    lgw_reg_w(SX1302_REG_CLK_CTRL_CLK_SEL_CLKDIV_EN, 0x01);
    
    /* Switch SX1302 clock from SPI clock to SX1262 clock */
    lgw_reg_w(SX1302_REG_CLK_CTRL_CLK_SEL_CLK_RADIO_A_SEL, 0x01);
    lgw_reg_w(SX1302_REG_CLK_CTRL_CLK_SEL_CLK_RADIO_B_SEL, 0x00);


    /* Check that the SX1302 timestamp counter is running */
    lgw_get_instcnt(&val);
    lgw_get_instcnt(&val2);
    if (val == val2) {
        printf("ERROR: SX1302 timestamp counter is not running (val:%u)\n", (uint32_t)val);
        return -1;
    }

    /* Set RADIO_A to SX1262FE_MODE */
    lgw_reg_w(SX1302_REG_COMMON_CTRL0_SX1261_MODE_RADIO_A, 0x01);

    return 0;
}

int sx1262fe_set_idle(void) {
    uint8_t buff[16];

    buff[0] = (uint8_t)STDBY_XOSC;
    sx1262fe_write_command(SET_STANDBY, buff, 1);

    buff[0] = 0x05;
    buff[1] = 0x87;
    buff[2] = 0x0E;
    sx1262fe_write_command(WRITE_REGISTER, buff, 3); /* Default value */

    return 0;
}

int load_agc_fw(void) {
    int i;
    uint8_t fw_check[8192];
    int32_t gpio_sel = 0x01; /* AGC MCU */

    /* Configure GPIO to let AGC MCU access board LEDs */
    lgw_reg_w(SX1302_REG_GPIO_GPIO_SEL_0_SELECTION, gpio_sel);
    lgw_reg_w(SX1302_REG_GPIO_GPIO_SEL_1_SELECTION, gpio_sel);
    lgw_reg_w(SX1302_REG_GPIO_GPIO_SEL_2_SELECTION, gpio_sel);
    lgw_reg_w(SX1302_REG_GPIO_GPIO_SEL_3_SELECTION, gpio_sel);
    lgw_reg_w(SX1302_REG_GPIO_GPIO_SEL_4_SELECTION, gpio_sel);
    lgw_reg_w(SX1302_REG_GPIO_GPIO_SEL_5_SELECTION, gpio_sel);
    lgw_reg_w(SX1302_REG_GPIO_GPIO_SEL_6_SELECTION, gpio_sel);
    lgw_reg_w(SX1302_REG_GPIO_GPIO_SEL_7_SELECTION, gpio_sel);
    lgw_reg_w(SX1302_REG_GPIO_GPIO_DIR_DIRECTION, 0xFF); /* GPIO output direction */

    /* Take control over AGC MCU */
    lgw_reg_w(SX1302_REG_AGC_MCU_CTRL_MCU_CLEAR, 0x01);
    lgw_reg_w(SX1302_REG_AGC_MCU_CTRL_HOST_PROG, 0x01);
    lgw_reg_w(SX1302_REG_COMMON_PAGE_PAGE, 0x00);

    /* Write AGC fw in AGC MEM */
    for(i=0; i<8; i++) {
        lgw_mem_wb(0x0000+(i*1024), &agc_firmware[i*1024], 1024);
    }

    /* Read back and check */
    for(i=0; i<8; i++) {
        lgw_mem_rb(0x0000+(i*1024), &fw_check[i*1024], 1024);
    }
    if (memcmp(agc_firmware, fw_check, 8192) != 0) {
        printf ("ERROR: Failed to load fw\n");
        return -1;
    }

    printf("AGC fw loaded\n");

    /* Release control over AGC MCU */
    lgw_reg_w(SX1302_REG_AGC_MCU_CTRL_HOST_PROG, 0x00);
    lgw_reg_w(SX1302_REG_AGC_MCU_CTRL_MCU_CLEAR, 0x00);

    printf("Waiting for AGC fw to start...\n");
    wait_ms(3000);

    return 0;
}

int sx1262fe_send_pkt(uint32_t freq_hz, uint8_t sf, uint32_t bw, uint16_t fcnt) {
    uint8_t buff[16];
    uint32_t freq_reg;
    int32_t val;
    int i;
    uint32_t preamble_symb_nb;
    uint32_t freq_dev = bw/2;
    uint16_t tx_start_delay;
    uint8_t payload[255];
    uint8_t payload_len;

    /* give radio control to HOST */
    lgw_reg_w(SX1302_REG_COMMON_CTRL0_HOST_RADIO_CTRL, 0x01);

    /* Configure radio */
    buff[0] = 0x08;
    buff[1] = 0xE6;
    buff[2] = 0x1C;
    sx1262fe_write_command(WRITE_REGISTER, buff, 3); /* ?? */

    buff[0] = 0x01; /* LoRa */
    sx1262fe_write_command(SET_PACKET_TYPE, buff, 1);

    freq_reg = SX1262FE_FREQ_TO_REG(freq_hz);
    buff[0] = (uint8_t)(freq_reg >> 24);
    buff[1] = (uint8_t)(freq_reg >> 16);
    buff[2] = (uint8_t)(freq_reg >> 8);
    buff[3] = (uint8_t)(freq_reg >> 0);
    sx1262fe_write_command(SET_RF_FREQUENCY, buff, 4);

    buff[0] = 0x0E; /* power */
    buff[1] = 0x02; /* RAMP_40U */
    sx1262fe_write_command(SET_TX_PARAMS, buff, 2);

    buff[0] = 0x04; /* paDutyCycle */
    buff[1] = 0x07; /* hpMax */
    buff[2] = 0x00; /* deviceSel */
    buff[3] = 0x01; /* paLut */
    sx1262fe_write_command(SET_PA_CONFIG, buff, 4); /* SX1262 Output Power +22dBm */

    /* give radio control to AGC MCU */
    lgw_reg_w(SX1302_REG_COMMON_CTRL0_HOST_RADIO_CTRL, 0x00);

    lgw_reg_w(SX1302_REG_TX_TOP_A_TX_RFFE_IF_CTRL_PLL_DIV_CTRL, 0x00); /* VCO divider by 2 */
    lgw_reg_w(SX1302_REG_TX_TOP_A_TX_RFFE_IF_CTRL_TX_IF_SRC, 0x01); /* LoRa */
    lgw_reg_w(SX1302_REG_TX_TOP_A_TX_RFFE_IF_CTRL_TX_IF_DST, 0x01); /* SX126x Tx RFFE */
    lgw_reg_w(SX1302_REG_TX_TOP_A_TX_RFFE_IF_CTRL_TX_MODE, 0x01); /* Modulation */
    lgw_reg_w(SX1302_REG_TX_TOP_A_TX_RFFE_IF_CTRL_TX_CLK_EDGE, 0x01); /* Data on falling edge */
    lgw_reg_w(SX1302_REG_TX_TOP_A_GEN_CFG_0_MODULATION_TYPE, 0x00); /* LoRa */

    /* Set Tx frequency */
    freq_reg = SX1302_FREQ_TO_REG(freq_hz);
    lgw_reg_w(SX1302_REG_TX_TOP_A_TX_RFFE_IF_FREQ_RF_H_FREQ_RF, (freq_reg >> 16) & 0xFF);
    lgw_reg_w(SX1302_REG_TX_TOP_A_TX_RFFE_IF_FREQ_RF_M_FREQ_RF, (freq_reg >> 8) & 0xFF);
    lgw_reg_w(SX1302_REG_TX_TOP_A_TX_RFFE_IF_FREQ_RF_L_FREQ_RF, (freq_reg >> 0) & 0xFF);

    /* Set bandwidth */
    freq_reg = SX1302_FREQ_TO_REG(freq_dev);
    lgw_reg_w(SX1302_REG_TX_TOP_A_TX_RFFE_IF_FREQ_DEV_H_FREQ_DEV, (freq_reg >>  8) & 0xFF);
    lgw_reg_w(SX1302_REG_TX_TOP_A_TX_RFFE_IF_FREQ_DEV_L_FREQ_DEV, (freq_reg >>  0) & 0xFF);

    /* LoRa modem parameters */
    switch(bw) {
        case 125000:
            printf("bandwidth 125khz\n");
            lgw_reg_w(SX1302_REG_TX_TOP_A_TXRX_CFG0_0_MODEM_BW, 0x04); /* 4: 125khz, 5: 250khz, 6: 500khz */
            break;
        case 250000:
            printf("bandwidth 250khz\n");
            lgw_reg_w(SX1302_REG_TX_TOP_A_TXRX_CFG0_0_MODEM_BW, 0x05); /* 4: 125khz, 5: 250khz, 6: 500khz */
            break;
        case 500000:
            printf("bandwidth 500khz\n");
            lgw_reg_w(SX1302_REG_TX_TOP_A_TXRX_CFG0_0_MODEM_BW, 0x06); /* 4: 125khz, 5: 250khz, 6: 500khz */
            break;
        default:
            printf("ERROR: bandwidth %d not supported\n", bw);
            break;
    }
    /* Preamble length */
    preamble_symb_nb = 8;
    lgw_reg_w(SX1302_REG_TX_TOP_A_TXRX_CFG1_3_PREAMBLE_SYMB_NB, (preamble_symb_nb >> 8) & 0xFF); /* MSB */
    lgw_reg_w(SX1302_REG_TX_TOP_A_TXRX_CFG1_2_PREAMBLE_SYMB_NB, (preamble_symb_nb >> 0) & 0xFF); /* LSB */

    /* LoRa datarate */
    lgw_reg_w(SX1302_REG_TX_TOP_A_TXRX_CFG0_0_MODEM_SF, sf);
    lgw_reg_w(SX1302_REG_TX_TOP_A_TX_CFG0_0_CHIRP_LOWPASS, 7);

    /* Start LoRa modem */
    lgw_reg_w(SX1302_REG_TX_TOP_A_TXRX_CFG0_2_MODEM_EN, 1);
    lgw_reg_w(SX1302_REG_TX_TOP_A_TXRX_CFG0_2_CADRXTX, 2);
    lgw_reg_w(SX1302_REG_TX_TOP_A_TXRX_CFG1_1_MODEM_START, 1);
    lgw_reg_w(SX1302_REG_TX_TOP_A_TX_CFG0_0_CONTINUOUS, 0);

#if 0 /* TODO: how to do continuous lora modulation ?*/
    lgw_reg_w(SX1302_REG_TX_TOP_A_TX_CFG0_0_CONTINUOUS, 1);
    lgw_reg_w(SX1302_REG_TX_TOP_B_TX_CFG1_0_FRAME_NB, 2);
#endif

    /* Uplink configuration */
    lgw_reg_w(SX1302_REG_TX_TOP_A_TX_CFG0_0_CHIRP_INVERT, 0); /* non-inverted */
    lgw_reg_w(SX1302_REG_TX_TOP_A_TXRX_CFG0_2_IMPLICIT_HEADER, 0); /*  */
    lgw_reg_w(SX1302_REG_TX_TOP_A_TXRX_CFG0_2_CRC_EN, 1); /*  */
    lgw_reg_w(SX1302_REG_TX_TOP_A_FRAME_SYNCH_0_PEAK1_POS, 3); /*  */
    lgw_reg_w(SX1302_REG_TX_TOP_A_FRAME_SYNCH_1_PEAK2_POS, 4); /*  */
    
    /* Set TX start delay */
    tx_start_delay = 1000 * 32; /* us */ /* TODO: which value should we put?? */
    lgw_reg_w(SX1302_REG_TX_TOP_A_TX_START_DELAY_MSB_TX_START_DELAY, (uint8_t)(tx_start_delay >> 8));
    lgw_reg_w(SX1302_REG_TX_TOP_A_TX_START_DELAY_LSB_TX_START_DELAY, (uint8_t)(tx_start_delay >> 0));

    /* Set Payload length */
    payload_len = (uint8_t)(rand() % (255 + 1 - 12) + 12);
    lgw_reg_w(SX1302_REG_TX_TOP_A_TXRX_CFG0_3_PAYLOAD_LENGTH, payload_len);

    /* Write payload in transmit buffer */
    payload[0] = 0x40; /* Confirmed Data Up */
    payload[1] = 0xAB;
    payload[2] = 0xAB;
    payload[3] = 0xAB;
    payload[4] = 0xAB;
    payload[5] = 0x00; /* FCTrl */
    payload[6] = (uint8_t)(fcnt >> 0); /* FCnt */
    payload[7] = (uint8_t)(fcnt >> 8); /* FCnt */
    payload[8] = 0x02; /* FPort */
    for (i = 9; i < 255; i++) {
        payload[i] = i;
    }
    lgw_reg_w(SX1302_REG_TX_TOP_A_TX_CTRL_WRITE_BUFFER, 0x01);
    lgw_mem_wb(0x5300, &payload[0], payload_len);
    lgw_reg_w(SX1302_REG_TX_TOP_A_TX_CTRL_WRITE_BUFFER, 0x00);

    printf("Start Tx: Freq:%u SF%u size:%u\n", freq_hz, sf, payload_len);
    lgw_reg_w(SX1302_REG_TX_TOP_A_TX_TRIG_TX_TRIG_IMMEDIATE, 0x01);

    do {
        //lgw_reg_r(SX1302_REG_TX_TOP_A_LORA_TX_STATE_STATUS, &val);
        //lgw_reg_r(SX1302_REG_TX_TOP_A_LORA_TX_FLAG_FRAME_DONE, &val);
        //lgw_reg_r(SX1302_REG_TX_TOP_B_LORA_TX_FLAG_CONT_DONE, &val);
        //printf("cont done 0x%02X\n", val);
        lgw_reg_r(SX1302_REG_TX_TOP_A_TX_STATUS_TX_STATUS, &val);
        wait_ms(10);
    } while (val != 0x80);

#if 1
    /* TODO: should be done by AGC fw? */
    printf("Stop Tx\n");
    lgw_reg_w(SX1302_REG_TX_TOP_A_TX_TRIG_TX_TRIG_IMMEDIATE, 0x00);
#endif

    return 0;
}

/* -------------------------------------------------------------------------- */
/* --- MAIN FUNCTION -------------------------------------------------------- */

/* describe command line options */
void usage(void) {
    //printf("Library version information: %s\n", lgw_version_info());
    printf( "Available options:\n");
    printf( " -h print this help\n");
    printf( " -f <float> Radio TX frequency in MHz\n");
    printf( " -s <uint> LoRa datarate [7..12]\n");
    printf( " -b <uint> LoRa bandwidth in khz [125, 250, 500]\n");
    printf( " -x <uint> Number of packets to be sent\n");
}

int main(int argc, char **argv)
{
    int i, x;
    uint32_t ft = DEFAULT_FREQ_HZ;
    uint8_t sf = DEFAULT_SF;
    uint32_t bw = DEFAULT_BW_HZ;
    uint32_t nb_pkt = DEFAULT_NB_PKT;
    double arg_d = 0.0;
    unsigned int arg_u;

    /* parse command line options */
    while ((i = getopt (argc, argv, "hf:s:b:n:")) != -1) {
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
            case 's': /* <uint> LoRa datarate */
                i = sscanf(optarg, "%u", &arg_u);
                if ((i != 1) || (arg_u < 7) || (arg_u > 12)) {
                    printf("ERROR: argument parsing of -s argument. Use -h to print help\n");
                    return EXIT_FAILURE;
                } else {
                    sf = (uint8_t)arg_u;
                }
                break;
            case 'b': /* <uint> LoRa bandwidth in khz */
                i = sscanf(optarg, "%u", &arg_u);
                if ((i != 1) || ((arg_u != 125) && (arg_u != 250) && (arg_u != 500))) {
                    printf("ERROR: argument parsing of -b argument. Use -h to print help\n");
                    return EXIT_FAILURE;
                } else {
                    bw = (uint32_t)(arg_u * 1E3);
                }
                break;
            case 'n': /* <uint> Number of packets to be sent */
                i = sscanf(optarg, "%u", &arg_u);
                if (i != 1) {
                    printf("ERROR: argument parsing of -n argument. Use -h to print help\n");
                    return EXIT_FAILURE;
                } else {
                    nb_pkt = (uint32_t)arg_u;
                }
                break;
            default:
                printf("ERROR: argument parsing\n");
                usage();
                return -1;
        }
    }

    printf("===== sx1302 sx1262fe TX test =====\n");

    /* Board reset */
    system("./reset_lgw.sh start");

    lgw_connect();

    x = sx1262fe_init();
    if (x != 0) {
        printf("ERROR: failed to initialize radio\n");
        return EXIT_FAILURE;
    }

    x = load_agc_fw();
    if (x != 0) {
        printf("ERROR: failed to load AGC firmware\n");
        return EXIT_FAILURE;
    }

    for (i = 0; i < (int)nb_pkt; i++) {
        sx1262fe_send_pkt(ft, sf, bw, (uint16_t)i);
    }

    sx1262fe_set_idle();

    lgw_disconnect();
    printf("=========== Test End ===========\n");

    return 0;
}

/* --- EOF ------------------------------------------------------------------ */
