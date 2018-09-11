/*
 / _____)             _              | |
( (____  _____ ____ _| |_ _____  ____| |__
 \____ \| ___ |    (_   _) ___ |/ ___)  _ \
 _____) ) ____| | | || |_| ____( (___| | | |
(______/|_____)_|_|_| \__)_____)\____)_| |_|
  (C)2013 Semtech-Cycleo

Description:
    LoRa concentrator Hardware Abstraction Layer

License: Revised BSD License, see LICENSE.TXT file include in the project
Maintainer: Sylvain Miermont
*/


/* -------------------------------------------------------------------------- */
/* --- DEPENDANCIES --------------------------------------------------------- */

#include <stdint.h>     /* C99 types */
#include <stdbool.h>    /* bool type */
#include <stdio.h>      /* printf fprintf */
#include <string.h>     /* memcpy */
#include <math.h>       /* pow, cell */
#include <assert.h>

#include "loragw_reg.h"
#include "loragw_hal.h"
#include "loragw_aux.h"
#include "loragw_spi.h"
#include "loragw_sx1250.h"
#include "loragw_sx125x.h"
#include "loragw_sx1302.h"

/* -------------------------------------------------------------------------- */
/* --- PRIVATE MACROS ------------------------------------------------------- */

#define ARRAY_SIZE(a) (sizeof(a) / sizeof((a)[0]))
#if DEBUG_HAL == 1
    #define DEBUG_MSG(str)                fprintf(stderr, str)
    #define DEBUG_PRINTF(fmt, args...)    fprintf(stderr,"%s:%d: "fmt, __FUNCTION__, __LINE__, args)
    #define DEBUG_ARRAY(a,b,c)            for(a=0;a<b;++a) fprintf(stderr,"%x.",c[a]);fprintf(stderr,"end\n")
    #define CHECK_NULL(a)                 if(a==NULL){fprintf(stderr,"%s:%d: ERROR: NULL POINTER AS ARGUMENT\n", __FUNCTION__, __LINE__);return LGW_HAL_ERROR;}
#else
    #define DEBUG_MSG(str)
    #define DEBUG_PRINTF(fmt, args...)
    #define DEBUG_ARRAY(a,b,c)            for(a=0;a!=0;){}
    #define CHECK_NULL(a)                 if(a==NULL){return LGW_HAL_ERROR;}
#endif

#define SET_PPM_ON(bw,dr)   (((bw == BW_125KHZ) && ((dr == DR_LORA_SF11) || (dr == DR_LORA_SF12))) || ((bw == BW_250KHZ) && (dr == DR_LORA_SF12)))
#define TRACE()             fprintf(stderr, "@ %s %d\n", __FUNCTION__, __LINE__);

#define SX1257_FREQ_TO_REG(f)       (uint32_t)((uint64_t)f * (1 << 19) / 32000000U)
#define SX1255_FREQ_TO_REG(f)       (uint32_t)((uint64_t)f * (1 << 20) / 32000000U)
#define SX1250_FREQ_TO_REG(f)       (uint32_t)((uint64_t)f * (1 << 25) / 32000000U)
#define SX1302_FREQ_TO_REG(f)       (uint32_t)((uint64_t)f * (1 << 18) / 32000000U)

/* -------------------------------------------------------------------------- */
/* --- PRIVATE CONSTANTS & TYPES -------------------------------------------- */

/* all the code that should be enabled on final board */
#define __SX1302_TODO__     0

#define MCU_ARB             0
#define MCU_AGC             1
#define MCU_ARB_FW_BYTE     8192 /* size of the firmware IN BYTES (= twice the number of 14b words) */
#define MCU_AGC_FW_BYTE     8192 /* size of the firmware IN BYTES (= twice the number of 14b words) */
#define FW_VERSION_ADDR     0x0 /* Address of firmware version in data memory */ /* TODO */
#define FW_VERSION_CAL      0 /* Expected version of calibration firmware */ /* TODO */
#define FW_VERSION_AGC      0 /* Expected version of AGC firmware */ /* TODO */
#define FW_VERSION_ARB      0 /* Expected version of arbiter firmware */ /* TODO */

#define TX_METADATA_NB      16
#define RX_METADATA_NB      16

#define AGC_CMD_WAIT        16
#define AGC_CMD_ABORT       17

#define MIN_LORA_PREAMBLE   6
#define STD_LORA_PREAMBLE   8
#define MIN_FSK_PREAMBLE    3
#define STD_FSK_PREAMBLE    5

#define RSSI_MULTI_BIAS     -35 /* difference between "multi" modem RSSI offset and "stand-alone" modem RSSI offset */
#define RSSI_FSK_POLY_0     60 /* polynomiam coefficients to linearize FSK RSSI */
#define RSSI_FSK_POLY_1     1.5351
#define RSSI_FSK_POLY_2     0.003

/* Useful bandwidth of SX125x radios to consider depending on channel bandwidth */
/* Note: the below values come from lab measurements. For any question, please contact Semtech support */
#define LGW_RF_RX_BANDWIDTH_125KHZ  1600000     /* for 125KHz channels */
#define LGW_RF_RX_BANDWIDTH_250KHZ  1600000     /* for 250KHz channels */
#define LGW_RF_RX_BANDWIDTH_500KHZ  1600000     /* for 500KHz channels */

#define TX_START_DELAY_DEFAULT  1497 /* Calibrated value for 500KHz BW */ /* TODO */

/* constant arrays defining hardware capability */
const uint8_t ifmod_config[LGW_IF_CHAIN_NB] = LGW_IFMODEM_CONFIG;

/* Version string, used to identify the library version/options once compiled */
const char lgw_version_string[] = "Version: " LIBLORAGW_VERSION ";";

/* -------------------------------------------------------------------------- */
/* --- PRIVATE VARIABLES ---------------------------------------------------- */

//#include "arb_fw.var" /* external definition of the variable */
//#include "agc_fw.var" /* external definition of the variable */
//#include "cal_fw.var" /* external definition of the variable */
#include "src/text_agc_sx1250_07_sep_1.var"
#include "src/text_agc_sx1257_10_sep_1.var"
#include "src/text_arb_sx1302_10_sep_1.var"

/*
The following static variables are the configuration set that the user can
modify using rxrf_setconf, rxif_setconf and txgain_setconf functions.
The functions _start and _send then use that set to configure the hardware.

Parameters validity and coherency is verified by the _setconf functions and
the _start and _send functions assume they are valid.
*/

static bool lgw_is_started;

static bool rf_enable[LGW_RF_CHAIN_NB];
static uint32_t rf_rx_freq[LGW_RF_CHAIN_NB]; /* absolute, in Hz */
static float rf_rssi_offset[LGW_RF_CHAIN_NB];
static bool rf_tx_enable[LGW_RF_CHAIN_NB];
static enum lgw_radio_type_e rf_radio_type[LGW_RF_CHAIN_NB];

static bool if_enable[LGW_IF_CHAIN_NB];
static bool if_rf_chain[LGW_IF_CHAIN_NB]; /* for each IF, 0 -> radio A, 1 -> radio B */
static int32_t if_freq[LGW_IF_CHAIN_NB]; /* relative to radio frequency, +/- in Hz */

static uint8_t lora_multi_sfmask[LGW_MULTI_NB]; /* enables SF for LoRa 'multi' modems */

static uint8_t lora_rx_bw; /* bandwidth setting for LoRa standalone modem */
static uint8_t lora_rx_sf; /* spreading factor setting for LoRa standalone modem */
static bool lora_rx_ppm_offset;

static uint8_t fsk_rx_bw; /* bandwidth setting of FSK modem */
static uint32_t fsk_rx_dr; /* FSK modem datarate in bauds */
static uint8_t fsk_sync_word_size = 3; /* default number of bytes for FSK sync word */
static uint64_t fsk_sync_word= 0xC194C1; /* default FSK sync word (ALIGNED RIGHT, MSbit first) */

static bool lorawan_public = false;
static uint8_t rf_clkout = 0;

static struct lgw_tx_gain_lut_s txgain_lut = {
    .size = 2,
    .lut[0] = {
        .dig_gain = 0,
        .pa_gain = 2,
        .dac_gain = 3,
        .mix_gain = 10,
        .rf_power = 14
    },
    .lut[1] = {
        .dig_gain = 0,
        .pa_gain = 3,
        .dac_gain = 3,
        .mix_gain = 14,
        .rf_power = 27
    }};

static uint8_t rx_fifo[4096];

/* -------------------------------------------------------------------------- */
/* --- PRIVATE FUNCTIONS DECLARATION ---------------------------------------- */

int load_firmware_agc(const uint8_t * firmware);
int load_firmware_arb(const uint8_t * firmware);

void lgw_constant_adjust(void);

int32_t lgw_sf_getval(int x);
int32_t lgw_bw_getval(int x);

/* -------------------------------------------------------------------------- */
/* --- PRIVATE FUNCTIONS DEFINITION ----------------------------------------- */

void led(uint8_t val) {
    int32_t gpio_sel = 0x00; /* SPI */

    lgw_reg_w(SX1302_REG_GPIO_GPIO_SEL_0_SELECTION, gpio_sel);
    lgw_reg_w(SX1302_REG_GPIO_GPIO_SEL_1_SELECTION, gpio_sel);
    lgw_reg_w(SX1302_REG_GPIO_GPIO_SEL_2_SELECTION, gpio_sel);
    lgw_reg_w(SX1302_REG_GPIO_GPIO_SEL_3_SELECTION, gpio_sel);
    lgw_reg_w(SX1302_REG_GPIO_GPIO_SEL_4_SELECTION, gpio_sel);
    lgw_reg_w(SX1302_REG_GPIO_GPIO_SEL_5_SELECTION, gpio_sel);
    lgw_reg_w(SX1302_REG_GPIO_GPIO_SEL_6_SELECTION, gpio_sel);
    lgw_reg_w(SX1302_REG_GPIO_GPIO_SEL_7_SELECTION, gpio_sel);
    lgw_reg_w(SX1302_REG_GPIO_GPIO_DIR_L_DIRECTION, 0xFF); /* GPIO output direction */

    lgw_reg_w(SX1302_REG_GPIO_GPIO_OUT_L_OUT_VALUE, val);
}

/* ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ */

int load_firmware_agc(const uint8_t *firmware) {
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
    lgw_reg_w(SX1302_REG_GPIO_GPIO_DIR_L_DIRECTION, 0xFF); /* GPIO output direction */

    /* Take control over AGC MCU */
    lgw_reg_w(SX1302_REG_AGC_MCU_CTRL_MCU_CLEAR, 0x01);
    lgw_reg_w(SX1302_REG_AGC_MCU_CTRL_HOST_PROG, 0x01);
    lgw_reg_w(SX1302_REG_COMMON_PAGE_PAGE, 0x00);

    /* Write AGC fw in AGC MEM */
    for (i = 0; i < 8; i++) {
        lgw_mem_wb(0x0000+(i*1024), &firmware[i*1024], 1024);
    }

    /* Read back and check */
    for (i = 0; i < 8; i++) {
        lgw_mem_rb(0x0000+(i*1024), &fw_check[i*1024], 1024);
    }
    if (memcmp(firmware, fw_check, 8192) != 0) {
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

/* ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ */

int load_firmware_arb(const uint8_t *firmware) {
    int i;
    uint8_t fw_check[8192];
    int32_t gpio_sel = 0x02; /* ARB MCU */
    int32_t val;

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

    /* Write ARB fw in ARB MEM */
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

    /* Release control over ARB MCU */
    lgw_reg_w(SX1302_REG_ARB_MCU_CTRL_HOST_PROG, 0x00);
    lgw_reg_w(SX1302_REG_ARB_MCU_CTRL_MCU_CLEAR, 0x00);

    lgw_reg_r(SX1302_REG_ARB_MCU_CTRL_PARITY_ERROR, &val);
    printf("ARB fw loaded (parity error:0x%02X)\n", val);

    return 0;
}

/* ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ */

void lgw_constant_adjust(void) {
    /* TODO */

    return;
}

/* ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ */

int32_t lgw_bw_getval(int x) {
    switch (x) {
        case BW_500KHZ: return 500000;
        case BW_250KHZ: return 250000;
        case BW_125KHZ: return 125000;
        default: return -1;
    }
}

/* ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ */

int32_t lgw_sf_getval(int x) {
    switch (x) {
        case DR_LORA_SF5: return 5;
        case DR_LORA_SF6: return 6;
        case DR_LORA_SF7: return 7;
        case DR_LORA_SF8: return 8;
        case DR_LORA_SF9: return 9;
        case DR_LORA_SF10: return 10;
        case DR_LORA_SF11: return 11;
        case DR_LORA_SF12: return 12;
        default: return -1;
    }
}

/* ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ */

uint16_t lgw_get_tx_start_delay(uint8_t bw) {
    /* TODO */
    if (bw == 0) {} /* dummy */

    return (uint16_t)TX_START_DELAY_DEFAULT; /* keep truncating instead of rounding: better behaviour measured */
}

/* -------------------------------------------------------------------------- */
/* --- PUBLIC FUNCTIONS DEFINITION ------------------------------------------ */

int lgw_board_setconf(struct lgw_conf_board_s conf) {

    /* check if the concentrator is running */
    if (lgw_is_started == true) {
        DEBUG_MSG("ERROR: CONCENTRATOR IS RUNNING, STOP IT BEFORE TOUCHING CONFIGURATION\n");
        return LGW_HAL_ERROR;
    }

    /* set internal config according to parameters */
    lorawan_public = conf.lorawan_public;
    rf_clkout = conf.clksrc;

    DEBUG_PRINTF("Note: board configuration; lorawan_public:%d, clksrc:%d\n", lorawan_public, rf_clkout);

    return LGW_HAL_SUCCESS;
}

/* ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ */

int lgw_rxrf_setconf(uint8_t rf_chain, struct lgw_conf_rxrf_s conf) {

    /* check if the concentrator is running */
    if (lgw_is_started == true) {
        DEBUG_MSG("ERROR: CONCENTRATOR IS RUNNING, STOP IT BEFORE TOUCHING CONFIGURATION\n");
        return LGW_HAL_ERROR;
    }

    if (conf.enable == false) {
        /* nothing to do */
        DEBUG_PRINTF("Note: rf_chain %d disabled\n", rf_chain);
        return LGW_HAL_SUCCESS;
    }

    /* check input range (segfault prevention) */
    if (rf_chain >= LGW_RF_CHAIN_NB) {
        DEBUG_MSG("ERROR: NOT A VALID RF_CHAIN NUMBER\n");
        return LGW_HAL_ERROR;
    }

    /* check if radio type is supported */
    if ((conf.type != LGW_RADIO_TYPE_SX1255) && (conf.type != LGW_RADIO_TYPE_SX1257) && (conf.type != LGW_RADIO_TYPE_SX1250)) {
        DEBUG_PRINTF("ERROR: NOT A VALID RADIO TYPE (%d)\n", conf.type);
        return LGW_HAL_ERROR;
    }

    /* set internal config according to parameters */
    rf_enable[rf_chain] = conf.enable;
    rf_rx_freq[rf_chain] = conf.freq_hz;
    rf_rssi_offset[rf_chain] = conf.rssi_offset;
    rf_radio_type[rf_chain] = conf.type;
    rf_tx_enable[rf_chain] = conf.tx_enable;

    DEBUG_PRINTF("Note: rf_chain %d configuration; en:%d freq:%d rssi_offset:%f radio_type:%d tx_enable:%d\n", rf_chain, rf_enable[rf_chain], rf_rx_freq[rf_chain], rf_rssi_offset[rf_chain], rf_radio_type[rf_chain], rf_tx_enable[rf_chain]);

    return LGW_HAL_SUCCESS;
}

/* ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ */

int lgw_rxif_setconf(uint8_t if_chain, struct lgw_conf_rxif_s conf) {
    int32_t bw_hz;
    uint32_t rf_rx_bandwidth;

    /* check if the concentrator is running */
    if (lgw_is_started == true) {
        DEBUG_MSG("ERROR: CONCENTRATOR IS RUNNING, STOP IT BEFORE TOUCHING CONFIGURATION\n");
        return LGW_HAL_ERROR;
    }

    /* check input range (segfault prevention) */
    if (if_chain >= LGW_IF_CHAIN_NB) {
        DEBUG_PRINTF("ERROR: %d NOT A VALID IF_CHAIN NUMBER\n", if_chain);
        return LGW_HAL_ERROR;
    }

    /* if chain is disabled, don't care about most parameters */
    if (conf.enable == false) {
        if_enable[if_chain] = false;
        if_freq[if_chain] = 0;
        DEBUG_PRINTF("Note: if_chain %d disabled\n", if_chain);
        return LGW_HAL_SUCCESS;
    }

    /* check 'general' parameters */
    if (ifmod_config[if_chain] == IF_UNDEFINED) {
        DEBUG_PRINTF("ERROR: IF CHAIN %d NOT CONFIGURABLE\n", if_chain);
    }
    if (conf.rf_chain >= LGW_RF_CHAIN_NB) {
        DEBUG_MSG("ERROR: INVALID RF_CHAIN TO ASSOCIATE WITH A LORA_STD IF CHAIN\n");
        return LGW_HAL_ERROR;
    }
    /* check if IF frequency is optimal based on channel and radio bandwidths */
    switch (conf.bandwidth) {
        case BW_250KHZ:
            rf_rx_bandwidth = LGW_RF_RX_BANDWIDTH_250KHZ; /* radio bandwidth */
            break;
        case BW_500KHZ:
            rf_rx_bandwidth = LGW_RF_RX_BANDWIDTH_500KHZ; /* radio bandwidth */
            break;
        default:
            /* For 125KHz and below */
            rf_rx_bandwidth = LGW_RF_RX_BANDWIDTH_125KHZ; /* radio bandwidth */
            break;
    }
    bw_hz = lgw_bw_getval(conf.bandwidth); /* channel bandwidth */
    if ((conf.freq_hz + ((bw_hz==-1)?LGW_REF_BW:bw_hz)/2) > ((int32_t)rf_rx_bandwidth/2)) {
        DEBUG_PRINTF("ERROR: IF FREQUENCY %d TOO HIGH\n", conf.freq_hz);
        return LGW_HAL_ERROR;
    } else if ((conf.freq_hz - ((bw_hz==-1)?LGW_REF_BW:bw_hz)/2) < -((int32_t)rf_rx_bandwidth/2)) {
        DEBUG_PRINTF("ERROR: IF FREQUENCY %d TOO LOW\n", conf.freq_hz);
        return LGW_HAL_ERROR;
    }

    /* check parameters according to the type of IF chain + modem,
    fill default if necessary, and commit configuration if everything is OK */
    switch (ifmod_config[if_chain]) {
        case IF_LORA_STD:
            /* fill default parameters if needed */
            if (conf.bandwidth == BW_UNDEFINED) {
                conf.bandwidth = BW_250KHZ;
            }
            if (conf.datarate == DR_UNDEFINED) {
                conf.datarate = DR_LORA_SF9;
            }
            /* check BW & DR */
            if (!IS_LORA_BW(conf.bandwidth)) {
                DEBUG_MSG("ERROR: BANDWIDTH NOT SUPPORTED BY LORA_STD IF CHAIN\n");
                return LGW_HAL_ERROR;
            }
            if (!IS_LORA_DR(conf.datarate)) {
                DEBUG_MSG("ERROR: DATARATE NOT SUPPORTED BY LORA_STD IF CHAIN\n");
                return LGW_HAL_ERROR;
            }
            /* set internal configuration  */
            if_enable[if_chain] = conf.enable;
            if_rf_chain[if_chain] = conf.rf_chain;
            if_freq[if_chain] = conf.freq_hz;
            lora_rx_bw = conf.bandwidth;
            lora_rx_sf = conf.datarate;
            if (SET_PPM_ON(conf.bandwidth, conf.datarate)) {
                lora_rx_ppm_offset = true;
            } else {
                lora_rx_ppm_offset = false;
            }

            DEBUG_PRINTF("Note: LoRa 'std' if_chain %d configuration; en:%d freq:%d bw:%d dr:%d\n", if_chain, if_enable[if_chain], if_freq[if_chain], lora_rx_bw, lora_rx_sf);
            break;

        case IF_LORA_MULTI:
            /* fill default parameters if needed */
            if (conf.bandwidth == BW_UNDEFINED) {
                conf.bandwidth = BW_125KHZ;
            }
            if (conf.datarate == DR_UNDEFINED) {
                conf.datarate = DR_LORA_SF7;
            }
            /* check BW & DR */
            if (conf.bandwidth != BW_125KHZ) {
                DEBUG_MSG("ERROR: BANDWIDTH NOT SUPPORTED BY LORA_MULTI IF CHAIN\n");
                return LGW_HAL_ERROR;
            }
            if (!IS_LORA_DR(conf.datarate)) {
                DEBUG_MSG("ERROR: DATARATE(S) NOT SUPPORTED BY LORA_MULTI IF CHAIN\n");
                return LGW_HAL_ERROR;
            }
            /* set internal configuration  */
            if_enable[if_chain] = conf.enable;
            if_rf_chain[if_chain] = conf.rf_chain;
            if_freq[if_chain] = conf.freq_hz;
            lora_multi_sfmask[if_chain] = conf.datarate;

            DEBUG_PRINTF("Note: LoRa 'multi' if_chain %d configuration; en:%d freq:%d SF_mask:0x%02x\n", if_chain, if_enable[if_chain], if_freq[if_chain], lora_multi_sfmask[if_chain]);
            break;

        case IF_FSK_STD:
            /* fill default parameters if needed */
            if (conf.bandwidth == BW_UNDEFINED) {
                conf.bandwidth = BW_250KHZ;
            }
            if (conf.datarate == DR_UNDEFINED) {
                conf.datarate = 64000; /* default datarate */
            }
            /* check BW & DR */
            if(!IS_FSK_BW(conf.bandwidth)) {
                DEBUG_MSG("ERROR: BANDWIDTH NOT SUPPORTED BY FSK IF CHAIN\n");
                return LGW_HAL_ERROR;
            }
            if(!IS_FSK_DR(conf.datarate)) {
                DEBUG_MSG("ERROR: DATARATE NOT SUPPORTED BY FSK IF CHAIN\n");
                return LGW_HAL_ERROR;
            }
            /* set internal configuration  */
            if_enable[if_chain] = conf.enable;
            if_rf_chain[if_chain] = conf.rf_chain;
            if_freq[if_chain] = conf.freq_hz;
            fsk_rx_bw = conf.bandwidth;
            fsk_rx_dr = conf.datarate;
            if (conf.sync_word > 0) {
                fsk_sync_word_size = conf.sync_word_size;
                fsk_sync_word = conf.sync_word;
            }
            DEBUG_PRINTF("Note: FSK if_chain %d configuration; en:%d freq:%d bw:%d dr:%d (%d real dr) sync:0x%0*llX\n", if_chain, if_enable[if_chain], if_freq[if_chain], fsk_rx_bw, fsk_rx_dr, LGW_XTAL_FREQU/(LGW_XTAL_FREQU/fsk_rx_dr), 2*fsk_sync_word_size, fsk_sync_word);
            break;

        default:
            DEBUG_PRINTF("ERROR: IF CHAIN %d TYPE NOT SUPPORTED\n", if_chain);
            return LGW_HAL_ERROR;
    }

    return LGW_HAL_SUCCESS;
}

/* ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ */

int lgw_txgain_setconf(struct lgw_tx_gain_lut_s *conf) {
    int i;

    /* Check LUT size */
    if ((conf->size < 1) || (conf->size > TX_GAIN_LUT_SIZE_MAX)) {
        DEBUG_PRINTF("ERROR: TX gain LUT must have at least one entry and  maximum %d entries\n", TX_GAIN_LUT_SIZE_MAX);
        return LGW_HAL_ERROR;
    }

    txgain_lut.size = conf->size;

    for (i = 0; i < txgain_lut.size; i++) {
        /* Check gain range */
        if (conf->lut[i].dig_gain > 3) {
            DEBUG_MSG("ERROR: TX gain LUT: SX1301 digital gain must be between 0 and 3\n");
            return LGW_HAL_ERROR;
        }
        if (conf->lut[i].dac_gain != 3) {
            DEBUG_MSG("ERROR: TX gain LUT: SX1257 DAC gains != 3 are not supported\n");
            return LGW_HAL_ERROR;
        }
        if (conf->lut[i].mix_gain > 15) {
            DEBUG_MSG("ERROR: TX gain LUT: SX1257 mixer gain must not exceed 15\n");
            return LGW_HAL_ERROR;
        } else if (conf->lut[i].mix_gain < 8) {
            DEBUG_MSG("ERROR: TX gain LUT: SX1257 mixer gains < 8 are not supported\n");
            return LGW_HAL_ERROR;
        }
        if (conf->lut[i].pa_gain > 3) {
            DEBUG_MSG("ERROR: TX gain LUT: External PA gain must not exceed 3\n");
            return LGW_HAL_ERROR;
        }

        /* Set internal LUT */
        txgain_lut.lut[i].dig_gain = conf->lut[i].dig_gain;
        txgain_lut.lut[i].dac_gain = conf->lut[i].dac_gain;
        txgain_lut.lut[i].mix_gain = conf->lut[i].mix_gain;
        txgain_lut.lut[i].pa_gain  = conf->lut[i].pa_gain;
        txgain_lut.lut[i].rf_power = conf->lut[i].rf_power;
    }

    return LGW_HAL_SUCCESS;
}

/* ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ */

int lgw_start(void) {
    uint32_t val, val2;
    int reg_stat;
    sx1302_radio_type_t radio_type;

    if (lgw_is_started == true) {
        DEBUG_MSG("Note: LoRa concentrator already started, restarting it now\n");
    }

    reg_stat = lgw_connect();
    if (reg_stat == LGW_REG_ERROR) {
        DEBUG_MSG("ERROR: FAIL TO CONNECT BOARD\n");
        return LGW_HAL_ERROR;
    }

    /* reset the registers (also shuts the radios down) */

    /* gate clocks */

    /* setup radio A */
    radio_type = ((rf_radio_type[0] == LGW_RADIO_TYPE_SX1250) ? SX1302_RADIO_TYPE_SX1250 : SX1302_RADIO_TYPE_SX125X);
    if (rf_enable[0] == true) {
        sx1302_radio_reset(0, radio_type);
        switch (radio_type) {
            case SX1302_RADIO_TYPE_SX1250:
                sx1250_setup(0, rf_rx_freq[0]);
                break;
            case SX1302_RADIO_TYPE_SX125X:
                sx125x_setup(0, rf_clkout, true, rf_radio_type[0], rf_rx_freq[0]);
                break;
            default:
                DEBUG_MSG("ERROR: RADIO TYPE NOT SUPPORTED\n");
                return LGW_HAL_ERROR;
        }
        sx1302_radio_set_mode(0, radio_type);
    }

    /* setup radio B */
    radio_type = ((rf_radio_type[1] == LGW_RADIO_TYPE_SX1250) ? SX1302_RADIO_TYPE_SX1250 : SX1302_RADIO_TYPE_SX125X);
    if (rf_enable[1] == true) {
        sx1302_radio_reset(1, radio_type);
        switch (radio_type) {
            case SX1302_RADIO_TYPE_SX1250:
                sx1250_setup(1, rf_rx_freq[1]);
                break;
            case SX1302_RADIO_TYPE_SX125X:
                sx125x_setup(1, rf_clkout, true, rf_radio_type[1], rf_rx_freq[1]);
                break;
            default:
                DEBUG_MSG("ERROR: RADIO TYPE NOT SUPPORTED\n");
                return LGW_HAL_ERROR;
        }
        sx1302_radio_set_mode(1, radio_type);
    }

    /* Select the radio which provides the clock to the sx1302 */
    sx1302_radio_clock_select(rf_clkout);

#if !__SX1302_TODO__ /* Sanity check */ /* TODO: to be removed */
    /* Check that the SX1302 timestamp counter is running */
    lgw_get_instcnt(&val);
    lgw_get_instcnt(&val2);
    if (val == val2) {
        printf("ERROR: SX1302 timestamp counter is not running (val:%u)\n", (uint32_t)val);
        return -1;
    }
#endif

    /* GPIOs table :
    DGPIO0 -> N/A
    DGPIO1 -> N/A
    DGPIO2 -> N/A
    DGPIO3 -> TX digital filter ON
    DGPIO4 -> TX ON
    */

    /* select calibration command */

    /* Load the calibration firmware  */

    /* Check firmware version */

    /* Wait for calibration to end */

    /* Get calibration status */

    /* load adjusted parameters */
    lgw_constant_adjust();

    /* Sanity check for RX frequency */
    if (rf_rx_freq[0] == 0) {
        DEBUG_MSG("ERROR: wrong configuration, rf_rx_freq[0] is not set\n");
        return LGW_HAL_ERROR;
    }

    /* configure LoRa 'multi' demodulators */
    sx1302_channelizer_configure(if_rf_chain, if_freq);
    sx1302_correlator_configure();
    sx1302_modem_configure();
    sx1302_lora_syncword(lorawan_public);

    /* configure LoRa 'stand-alone' modem */
    /* TODO */

    /* configure FSK modem */
    /* TODO */

    /* Load firmware */
    switch (rf_radio_type[rf_clkout]) {
        case LGW_RADIO_TYPE_SX1250:
            printf("Loading AGC fw for sx1250\n");
            load_firmware_agc(agc_firmware_sx1250); /* TODO: check version */
            break;
        case LGW_RADIO_TYPE_SX1257:
            printf("Loading AGC fw for sx125x\n");
            load_firmware_agc(agc_firmware_sx125x); /* TODO: check version */
            break;
        default:
            break;
    }
    load_firmware_arb(arb_firmware); /* TODO: check version */

    /* give radio control to AGC MCU */
    lgw_reg_w(SX1302_REG_COMMON_CTRL0_HOST_RADIO_CTRL, 0x00);

    /* enable demodulators */
    sx1302_modem_enable();

    lgw_is_started = true;
    return LGW_HAL_SUCCESS;
}

/* ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ */

int lgw_stop(void) {
    //lgw_soft_reset();
    lgw_disconnect();

    lgw_is_started = false;
    return LGW_HAL_SUCCESS;
}

/* ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ */

int lgw_receive(uint8_t max_pkt, struct lgw_pkt_rx_s *pkt_data) {
    uint16_t nb_bytes;
    uint8_t buff[2];
    int i;
    uint16_t nb_pkt_fetch = 0;
    uint16_t nb_pkt_dropped = 0;
    struct lgw_pkt_rx_s *p;
    int ifmod; /* type of if_chain/modem a packet was received by */
    uint32_t sf, cr = 0;

    /* check if the concentrator is running */
    if (lgw_is_started == false) {
        DEBUG_MSG("ERROR: CONCENTRATOR IS NOT RUNNING, START IT BEFORE RECEIVING\n");
        return LGW_HAL_ERROR;
    }

    /* check input variables */
    if ((max_pkt <= 0) || (max_pkt > LGW_PKT_FIFO_SIZE)) {
        DEBUG_PRINTF("ERROR: %d = INVALID MAX NUMBER OF PACKETS TO FETCH\n", max_pkt);
        return LGW_HAL_ERROR;
    }
    CHECK_NULL(pkt_data);

    lgw_reg_rb(SX1302_REG_RX_TOP_RX_BUFFER_NB_BYTES_MSB_RX_BUFFER_NB_BYTES, buff, sizeof buff);
    nb_bytes  = (uint16_t)((buff[0] << 8) & 0xFF00);
    nb_bytes |= (uint16_t)((buff[1] << 0) & 0x00FF);

    /* TODO */
    if (nb_bytes > 1024) {
        printf("ERROR: received %u bytes (more than 1024 bytes in the FIFO, to be reworked)\n", nb_bytes);
        assert(0);
    }

    if (nb_bytes > 0) {
        printf("nb_bytes received: %u (%u %u)\n", nb_bytes, buff[1], buff[0]);
        /* read bytes from fifo */
        memset(rx_fifo, 0, sizeof rx_fifo);
        lgw_mem_rb(0x4000, rx_fifo, nb_bytes);
        for (i = 0; i < nb_bytes; i++) {
            printf("%02X ", rx_fifo[i]);
        }
        printf("\n");

        /* parse packets */
        for (i = 0; i < nb_bytes; i++) {
            if ((rx_fifo[i] == 0xA5) && (rx_fifo[i+1] == 0xC0)) {
                /* point to the proper struct in the struct array */
                p = &pkt_data[nb_pkt_fetch];

                /* we found the start of a packet, parse it */
                if ((nb_pkt_fetch + 1) > max_pkt) {
                    printf("WARNING: no space left, dropping packet\n");
                    nb_pkt_dropped += 1;
                    continue;
                }
                nb_pkt_fetch += 1;

                p->size = rx_fifo[i+2];

                 /* copy payload to result struct */
                memcpy((void *)p->payload, (void *)(&rx_fifo[i+6]), p->size);

                /* process metadata */
                p->if_chain = rx_fifo[i+3];
                if (p->if_chain >= LGW_IF_CHAIN_NB) {
                    DEBUG_PRINTF("WARNING: %u NOT A VALID IF_CHAIN NUMBER, ABORTING\n", p->if_chain);
                    break;
                }
                ifmod = ifmod_config[p->if_chain];
                DEBUG_PRINTF("[%d %d]\n", p->if_chain, ifmod);

                p->rf_chain = (uint8_t)if_rf_chain[p->if_chain];
                p->freq_hz = (uint32_t)((int32_t)rf_rx_freq[p->rf_chain] + if_freq[p->if_chain]);
                p->rssi = (float)rx_fifo[i+8+p->size] + rf_rssi_offset[p->rf_chain];

                if ((ifmod == IF_LORA_MULTI) || (ifmod == IF_LORA_STD)) {
                    DEBUG_PRINTF("Note: LoRa packet (modem %u chan %u)\n", rx_fifo[i+5], p->if_chain);
                    if (rx_fifo[i+4] & 0x01) {
                        /* CRC enabled */
                        if (rx_fifo[i+6+p->size] & 0x01) {
                            p->status = STAT_CRC_BAD;
                        } else {
                            p->status = STAT_CRC_OK;
                        }
                    } else {
                        /* CRC disabled */
                        p->status = STAT_NO_CRC;
                    }
                    p->modulation = MOD_LORA;
                    p->snr = rx_fifo[i+7+p->size];
                    if (ifmod == IF_LORA_MULTI) {
                        p->bandwidth = BW_125KHZ; /* fixed in hardware */
                    } else {
                        p->bandwidth = lora_rx_bw; /* get the parameter from the config variable */
                    }
                    sf = TAKE_N_BITS_FROM(rx_fifo[i+4], 4, 4);
                    switch (sf) {
                        case 5: p->datarate = DR_LORA_SF5; break;
                        case 6: p->datarate = DR_LORA_SF6; break;
                        case 7: p->datarate = DR_LORA_SF7; break;
                        case 8: p->datarate = DR_LORA_SF8; break;
                        case 9: p->datarate = DR_LORA_SF9; break;
                        case 10: p->datarate = DR_LORA_SF10; break;
                        case 11: p->datarate = DR_LORA_SF11; break;
                        case 12: p->datarate = DR_LORA_SF12; break;
                        default: p->datarate = DR_UNDEFINED;
                    }
                    cr = TAKE_N_BITS_FROM(rx_fifo[i+4], 1, 3);
                    switch (cr) {
                        case 1: p->coderate = CR_LORA_4_5; break;
                        case 2: p->coderate = CR_LORA_4_6; break;
                        case 3: p->coderate = CR_LORA_4_7; break;
                        case 4: p->coderate = CR_LORA_4_8; break;
                        default: p->coderate = CR_UNDEFINED;
                    }
                } else if (ifmod == IF_FSK_STD) {
                    DEBUG_MSG("Note: FSK packet\n");
                } else {
                    DEBUG_MSG("ERROR: UNEXPECTED PACKET ORIGIN\n");
                    p->status = STAT_UNDEFINED;
                    p->modulation = MOD_UNDEFINED;
                    p->rssi = -128.0;
                    p->snr = -128.0;
                    p->snr_min = -128.0;
                    p->snr_max = -128.0;
                    p->bandwidth = BW_UNDEFINED;
                    p->datarate = DR_UNDEFINED;
                    p->coderate = CR_UNDEFINED;
                }

                p->count_us  = (uint32_t)((rx_fifo[i+12+p->size] <<  0) & 0x000000FF);
                p->count_us |= (uint32_t)((rx_fifo[i+13+p->size] <<  8) & 0x0000FF00);
                p->count_us |= (uint32_t)((rx_fifo[i+14+p->size] << 16) & 0x00FF0000);
                p->count_us |= (uint32_t)((rx_fifo[i+15+p->size] << 24) & 0xFF000000);
                p->count_us /= 32;

                p->crc = (uint16_t)rx_fifo[i+16+p->size] + ((uint16_t)rx_fifo[i+17+p->size] << 8);
            }
        }
        printf("INFO: nb pkt fetched:%u dropped:%u\n", nb_pkt_fetch, nb_pkt_dropped);
    }

    return nb_pkt_fetch;
}

/* ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ */

int lgw_send(struct lgw_pkt_tx_s pkt_data) {
    uint32_t freq_reg, fdev_reg;
    uint32_t freq_dev = lgw_bw_getval(pkt_data.bandwidth) / 2;;
    uint16_t tx_start_delay;
    uint16_t reg;
    uint16_t mem_addr;

    /* Check if there is a TX on-going */
    /* TODO */

    /* check input variables */
    if ((pkt_data.rf_power < 0) || (pkt_data.rf_power > 15)) { /* TODO: if sx1250 */
        DEBUG_MSG("ERROR: RF power not supported\n");
        return LGW_HAL_ERROR;
    }

    reg = REG_SELECT(pkt_data.rf_chain, SX1302_REG_TX_TOP_A_TX_RFFE_IF_CTRL_PLL_DIV_CTRL,
                                        SX1302_REG_TX_TOP_B_TX_RFFE_IF_CTRL_PLL_DIV_CTRL);
    lgw_reg_w(reg, 0x00); /* VCO divider by 2 */

    reg = REG_SELECT(pkt_data.rf_chain, SX1302_REG_TX_TOP_A_TX_RFFE_IF_CTRL_TX_IF_SRC,
                                        SX1302_REG_TX_TOP_B_TX_RFFE_IF_CTRL_TX_IF_SRC);
    lgw_reg_w(reg, 0x01); /* LoRa */

    reg = REG_SELECT(pkt_data.rf_chain, SX1302_REG_TX_TOP_A_TX_RFFE_IF_CTRL_TX_IF_DST,
                                        SX1302_REG_TX_TOP_B_TX_RFFE_IF_CTRL_TX_IF_DST);
    switch (rf_radio_type[pkt_data.rf_chain]) {
        case LGW_RADIO_TYPE_SX1250:
            lgw_reg_w(reg, 0x01); /* SX126x Tx RFFE */
            break;
        case LGW_RADIO_TYPE_SX1257:
            lgw_reg_w(reg, 0x00); /* SX1255/57 Tx RFFE */
            break;
        default:
            DEBUG_MSG("ERROR: radio type not supported\n");
            return LGW_HAL_ERROR;
    }

    reg = REG_SELECT(pkt_data.rf_chain, SX1302_REG_TX_TOP_A_TX_RFFE_IF_CTRL_TX_MODE,
                                        SX1302_REG_TX_TOP_B_TX_RFFE_IF_CTRL_TX_MODE);
    lgw_reg_w(reg, 0x01); /* Modulation */

    reg = REG_SELECT(pkt_data.rf_chain, SX1302_REG_TX_TOP_A_TX_RFFE_IF_CTRL_TX_CLK_EDGE,
                                        SX1302_REG_TX_TOP_B_TX_RFFE_IF_CTRL_TX_CLK_EDGE);
    lgw_reg_w(reg, 0x00); /* Data on rising edge */

    reg = REG_SELECT(pkt_data.rf_chain, SX1302_REG_TX_TOP_A_GEN_CFG_0_TX_RADIO_SEL,
                                        SX1302_REG_TX_TOP_B_GEN_CFG_0_TX_RADIO_SEL);
    lgw_reg_w(reg, pkt_data.rf_chain);

    switch (pkt_data.modulation) {
        case MOD_LORA:
            reg = REG_SELECT(pkt_data.rf_chain, SX1302_REG_TX_TOP_A_GEN_CFG_0_MODULATION_TYPE,
                                                SX1302_REG_TX_TOP_B_GEN_CFG_0_MODULATION_TYPE);
            lgw_reg_w(reg, 0x00);
            break;
        case MOD_FSK:
            reg = REG_SELECT(pkt_data.rf_chain, SX1302_REG_TX_TOP_A_GEN_CFG_0_MODULATION_TYPE,
                                                SX1302_REG_TX_TOP_B_GEN_CFG_0_MODULATION_TYPE);
            lgw_reg_w(reg, 0x01);
            break;
        default:
            DEBUG_MSG("ERROR: modulation type not supported\n");
            return LGW_HAL_ERROR;
    }

    reg = REG_SELECT(pkt_data.rf_chain, SX1302_REG_TX_TOP_A_GEN_CFG_0_TX_POWER,
                                        SX1302_REG_TX_TOP_B_GEN_CFG_0_TX_POWER);
    lgw_reg_w(reg, pkt_data.rf_power);

    /* Get TX frequency and bandwidth (fdev) */
    freq_reg = SX1302_FREQ_TO_REG(pkt_data.freq_hz); /* TODO: AGC fw to be updated for sx1255 */
    fdev_reg = SX1302_FREQ_TO_REG(freq_dev);

    /* Set Tx frequency */
    reg = REG_SELECT(pkt_data.rf_chain, SX1302_REG_TX_TOP_A_TX_RFFE_IF_FREQ_RF_H_FREQ_RF,
                                        SX1302_REG_TX_TOP_B_TX_RFFE_IF_FREQ_RF_H_FREQ_RF);
    lgw_reg_w(reg, (freq_reg >> 16) & 0xFF);

    reg = REG_SELECT(pkt_data.rf_chain, SX1302_REG_TX_TOP_A_TX_RFFE_IF_FREQ_RF_M_FREQ_RF,
                                        SX1302_REG_TX_TOP_B_TX_RFFE_IF_FREQ_RF_M_FREQ_RF);
    lgw_reg_w(reg, (freq_reg >> 8) & 0xFF);

    reg = REG_SELECT(pkt_data.rf_chain, SX1302_REG_TX_TOP_A_TX_RFFE_IF_FREQ_RF_L_FREQ_RF,
                                        SX1302_REG_TX_TOP_B_TX_RFFE_IF_FREQ_RF_L_FREQ_RF);
    lgw_reg_w(reg, (freq_reg >> 0) & 0xFF);

    /* Set bandwidth */
    printf("Bandwidth %dkHz\n", (int)(lgw_bw_getval(pkt_data.bandwidth) / 1E3));
    reg = REG_SELECT(pkt_data.rf_chain, SX1302_REG_TX_TOP_A_TX_RFFE_IF_FREQ_DEV_H_FREQ_DEV,
                                        SX1302_REG_TX_TOP_B_TX_RFFE_IF_FREQ_DEV_H_FREQ_DEV);
    lgw_reg_w(reg, (fdev_reg >>  8) & 0xFF);

    reg = REG_SELECT(pkt_data.rf_chain, SX1302_REG_TX_TOP_A_TX_RFFE_IF_FREQ_DEV_L_FREQ_DEV,
                                        SX1302_REG_TX_TOP_B_TX_RFFE_IF_FREQ_DEV_L_FREQ_DEV);
    lgw_reg_w(reg, (fdev_reg >>  0) & 0xFF);

    /* Condifure modem */
    switch (pkt_data.modulation) {
        case MOD_LORA:
            reg = REG_SELECT(pkt_data.rf_chain, SX1302_REG_TX_TOP_A_TXRX_CFG0_0_MODEM_BW,
                                                SX1302_REG_TX_TOP_B_TXRX_CFG0_0_MODEM_BW);
            lgw_reg_w(reg, pkt_data.bandwidth);

            /* Preamble length */
            reg = REG_SELECT(pkt_data.rf_chain, SX1302_REG_TX_TOP_A_TXRX_CFG1_3_PREAMBLE_SYMB_NB,
                                                SX1302_REG_TX_TOP_B_TXRX_CFG1_3_PREAMBLE_SYMB_NB);
            lgw_reg_w(reg, (pkt_data.preamble >> 8) & 0xFF); /* MSB */

            reg = REG_SELECT(pkt_data.rf_chain, SX1302_REG_TX_TOP_A_TXRX_CFG1_2_PREAMBLE_SYMB_NB,
                                                SX1302_REG_TX_TOP_B_TXRX_CFG1_2_PREAMBLE_SYMB_NB);
            lgw_reg_w(reg, (pkt_data.preamble >> 0) & 0xFF); /* LSB */

            /* LoRa datarate */
            reg = REG_SELECT(pkt_data.rf_chain, SX1302_REG_TX_TOP_A_TXRX_CFG0_0_MODEM_SF,
                                                SX1302_REG_TX_TOP_B_TXRX_CFG0_0_MODEM_SF);
            lgw_reg_w(reg, pkt_data.datarate);

            reg = REG_SELECT(pkt_data.rf_chain, SX1302_REG_TX_TOP_A_TX_CFG0_0_CHIRP_LOWPASS,
                                                SX1302_REG_TX_TOP_B_TX_CFG0_0_CHIRP_LOWPASS);
            lgw_reg_w(reg, 7);

            /* Start LoRa modem */
            reg = REG_SELECT(pkt_data.rf_chain, SX1302_REG_TX_TOP_A_TXRX_CFG0_2_MODEM_EN,
                                                SX1302_REG_TX_TOP_B_TXRX_CFG0_2_MODEM_EN);
            lgw_reg_w(reg, 1);

            reg = REG_SELECT(pkt_data.rf_chain, SX1302_REG_TX_TOP_A_TXRX_CFG0_2_CADRXTX,
                                                SX1302_REG_TX_TOP_B_TXRX_CFG0_2_CADRXTX);
            lgw_reg_w(reg, 2);

            reg = REG_SELECT(pkt_data.rf_chain, SX1302_REG_TX_TOP_A_TXRX_CFG1_1_MODEM_START,
                                                SX1302_REG_TX_TOP_B_TXRX_CFG1_1_MODEM_START);
            lgw_reg_w(reg, 1);

            reg = REG_SELECT(pkt_data.rf_chain, SX1302_REG_TX_TOP_A_TX_CFG0_0_CONTINUOUS,
                                                SX1302_REG_TX_TOP_B_TX_CFG0_0_CONTINUOUS);
            lgw_reg_w(reg, 0);

            /*  */
            reg = REG_SELECT(pkt_data.rf_chain, SX1302_REG_TX_TOP_A_TX_CFG0_0_CHIRP_INVERT,
                                                SX1302_REG_TX_TOP_B_TX_CFG0_0_CHIRP_INVERT);
            lgw_reg_w(reg, (pkt_data.invert_pol) ? 1 : 0);

            reg = REG_SELECT(pkt_data.rf_chain, SX1302_REG_TX_TOP_A_TXRX_CFG0_2_IMPLICIT_HEADER,
                                                SX1302_REG_TX_TOP_B_TXRX_CFG0_2_IMPLICIT_HEADER);
            lgw_reg_w(reg, (pkt_data.no_header) ? 1 : 0);

            reg = REG_SELECT(pkt_data.rf_chain, SX1302_REG_TX_TOP_A_TXRX_CFG0_2_CRC_EN,
                                                SX1302_REG_TX_TOP_B_TXRX_CFG0_2_CRC_EN);
            lgw_reg_w(reg, (pkt_data.no_crc) ? 0 : 1);

            /* Syncword */
            reg = REG_SELECT(pkt_data.rf_chain, SX1302_REG_TX_TOP_A_FRAME_SYNCH_0_PEAK1_POS,
                                                SX1302_REG_TX_TOP_B_FRAME_SYNCH_0_PEAK1_POS);
            lgw_reg_w(reg, 6); /* public */

            reg = REG_SELECT(pkt_data.rf_chain, SX1302_REG_TX_TOP_A_FRAME_SYNCH_1_PEAK2_POS,
                                                SX1302_REG_TX_TOP_B_FRAME_SYNCH_1_PEAK2_POS);
            lgw_reg_w(reg, 8); /* public */

            /* Set Payload length */
            reg = REG_SELECT(pkt_data.rf_chain, SX1302_REG_TX_TOP_A_TXRX_CFG0_3_PAYLOAD_LENGTH,
                                                SX1302_REG_TX_TOP_B_TXRX_CFG0_3_PAYLOAD_LENGTH);
            lgw_reg_w(reg, pkt_data.size);
            break;
        default:
            printf("ERROR: Modulation not supported\n");
            return LGW_HAL_ERROR;
    }

    /* Set TX start delay */
    tx_start_delay = 1500 * 32; /* us */ /* TODO: which value should we put?? */
    reg = REG_SELECT(pkt_data.rf_chain, SX1302_REG_TX_TOP_A_TX_START_DELAY_MSB_TX_START_DELAY,
                                        SX1302_REG_TX_TOP_B_TX_START_DELAY_MSB_TX_START_DELAY);
    lgw_reg_w(reg, (uint8_t)(tx_start_delay >> 8));

    reg = REG_SELECT(pkt_data.rf_chain, SX1302_REG_TX_TOP_A_TX_START_DELAY_LSB_TX_START_DELAY,
                                        SX1302_REG_TX_TOP_B_TX_START_DELAY_LSB_TX_START_DELAY);
    lgw_reg_w(reg, (uint8_t)(tx_start_delay >> 0));

    /* Write payload in transmit buffer */
    reg = REG_SELECT(pkt_data.rf_chain, SX1302_REG_TX_TOP_A_TX_CTRL_WRITE_BUFFER,
                                        SX1302_REG_TX_TOP_B_TX_CTRL_WRITE_BUFFER);
    lgw_reg_w(reg, 0x01);
    mem_addr = REG_SELECT(pkt_data.rf_chain, 0x5300, 0x5500);
    lgw_mem_wb(mem_addr, &(pkt_data.payload[0]), pkt_data.size);
    lgw_reg_w(reg, 0x00);

    /* Trigger transmit */
    printf("Start Tx: Freq:%u SF%u size:%u\n", pkt_data.freq_hz, pkt_data.datarate, pkt_data.size);
    switch (pkt_data.tx_mode) {
        case IMMEDIATE:
            reg = REG_SELECT(pkt_data.rf_chain, SX1302_REG_TX_TOP_A_TX_TRIG_TX_TRIG_IMMEDIATE,
                                                SX1302_REG_TX_TOP_B_TX_TRIG_TX_TRIG_IMMEDIATE);
            lgw_reg_w(reg, 0x00); /* reset state machine */
            lgw_reg_w(reg, 0x01);
            break;
        case TIMESTAMPED:
            reg = REG_SELECT(pkt_data.rf_chain, SX1302_REG_TX_TOP_A_TX_TRIG_TX_TRIG_DELAYED,
                                                SX1302_REG_TX_TOP_B_TX_TRIG_TX_TRIG_DELAYED);
            lgw_reg_w(reg, 0x00); /* reset state machine */
            lgw_reg_w(reg, 0x01);
            break;
        case ON_GPS:
            reg = REG_SELECT(pkt_data.rf_chain, SX1302_REG_TX_TOP_A_TX_TRIG_TX_TRIG_GPS,
                                                SX1302_REG_TX_TOP_B_TX_TRIG_TX_TRIG_GPS);
            lgw_reg_w(reg, 0x00); /* reset state machine */
            lgw_reg_w(reg, 0x01);
            break;
        default:
            printf("ERROR: TX mode not supported\n");
            return LGW_HAL_ERROR;
    }

    return 0;
}

/* ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ */

int lgw_status(uint8_t rf_chain, uint8_t select, uint8_t *code) {
    int32_t read_value;
    uint16_t reg;

    /* check input variables */
    CHECK_NULL(code);
    if (rf_chain >= LGW_RF_CHAIN_NB) {
        DEBUG_MSG("ERROR: NOT A VALID RF_CHAIN NUMBER\n");
        return LGW_HAL_ERROR;
    }

    if (select == TX_STATUS) {
        //lgw_reg_r(SX1302_REG_TX_TOP_A_LORA_TX_STATE_STATUS, &val);
        //lgw_reg_r(SX1302_REG_TX_TOP_A_LORA_TX_FLAG_FRAME_DONE, &val);
        //lgw_reg_r(SX1302_REG_TX_TOP_B_LORA_TX_FLAG_CONT_DONE, &val);
        reg = REG_SELECT(rf_chain,  SX1302_REG_TX_TOP_A_TX_FSM_STATUS_TX_STATUS,
                                    SX1302_REG_TX_TOP_B_TX_FSM_STATUS_TX_STATUS);
        lgw_reg_r(reg, &read_value);
        // TODO: select porper TX rf chain
        if (lgw_is_started == false) {
            *code = TX_OFF;
        } else if (read_value == 0x80) {
            *code = TX_FREE;
        } else if ((read_value == 0x30) || (read_value == 0x50) || (read_value == 0x70)) {
            *code = TX_EMITTING;
        } else if ((read_value == 0x91) || (read_value == 0x92)) {
            *code = TX_SCHEDULED;
        } else {
            *code = TX_STATUS_UNKNOWN;
            DEBUG_PRINTF("ERROR: UNKNOWN TX STATUS 0x%02X\n", read_value);
            return LGW_HAL_ERROR;
        }
    } else if (select == RX_STATUS) {
        *code = RX_STATUS_UNKNOWN; /* todo */
    } else {
        DEBUG_MSG("ERROR: SELECTION INVALID, NO STATUS TO RETURN\n");
        return LGW_HAL_ERROR;
    }

    return LGW_HAL_SUCCESS;
}

/* ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ */

int lgw_abort_tx(void) {
    /* TODO */

    return LGW_HAL_SUCCESS;
}

/* ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ */

int sx1302_get_cnt(bool pps, uint32_t* cnt_us) {
    int x;
    uint8_t buff[4];

    /* Get the 32MHz timestamp counter - 4 bytes */
    /* step of 31.25 ns */
    x = lgw_reg_rb((pps == true) ? SX1302_REG_TIMESTAMP_TIMESTAMP_PPS_MSB2_TIMESTAMP_PPS : SX1302_REG_TIMESTAMP_TIMESTAMP_MSB2_TIMESTAMP, &buff[0], 4);
    if (x != LGW_REG_SUCCESS) {
        printf("ERROR: Failed to get timestamp counter value\n");
        *cnt_us = 0;
        return LGW_HAL_ERROR;
    }

    *cnt_us  = (uint32_t)((buff[0] << 24) & 0xFF000000);
    *cnt_us |= (uint32_t)((buff[1] << 16) & 0x00FF0000);
    *cnt_us |= (uint32_t)((buff[2] << 8)  & 0x0000FF00);
    *cnt_us |= (uint32_t)((buff[3] << 0)  & 0x000000FF);

    *cnt_us /= 32; /* scale to 1MHz */

    return LGW_HAL_SUCCESS;
}

/* ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ */

int lgw_get_trigcnt(uint32_t* trig_cnt_us) {
    return (sx1302_get_cnt(true, trig_cnt_us));
}

/* ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ */

int lgw_get_instcnt(uint32_t* inst_cnt_us) {
    return (sx1302_get_cnt(false, inst_cnt_us));
}

/* ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ */
const char* lgw_version_info() {
    return lgw_version_string;
}

/* ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ */

uint32_t lgw_time_on_air(struct lgw_pkt_tx_s *packet) {
    int32_t val;
    uint8_t SF, H, DE;
    uint16_t BW;
    uint32_t payloadSymbNb, Tpacket;
    double Tsym, Tpreamble, Tpayload, Tfsk;

    if (packet == NULL) {
        DEBUG_MSG("ERROR: Failed to compute time on air, wrong parameter\n");
        return 0;
    }

    if (packet->modulation == MOD_LORA) {
        /* Get bandwidth */
        val = lgw_bw_getval(packet->bandwidth);
        if (val != -1) {
            BW = (uint16_t)(val / 1E3);
        } else {
            DEBUG_PRINTF("ERROR: Cannot compute time on air for this packet, unsupported bandwidth (0x%02X)\n", packet->bandwidth);
            return 0;
        }

        /* Get datarate */
        val = lgw_sf_getval(packet->datarate);
        if (val != -1) {
            SF = (uint8_t)val;
        } else {
            DEBUG_PRINTF("ERROR: Cannot compute time on air for this packet, unsupported datarate (0x%02X)\n", packet->datarate);
            return 0;
        }

        /* Duration of 1 symbol */
        Tsym = pow(2, SF) / BW;

        /* Duration of preamble */
        Tpreamble = ((double)(packet->preamble) + 4.25) * Tsym;

        /* Duration of payload */
        H = (packet->no_header==false) ? 0 : 1; /* header is always enabled, except for beacons */
        DE = (SF >= 11) ? 1 : 0; /* Low datarate optimization enabled for SF11 and SF12 */

        payloadSymbNb = 8 + (ceil((double)(8*packet->size - 4*SF + 28 + 16 - 20*H) / (double)(4*(SF - 2*DE))) * (packet->coderate + 4)); /* Explicitely cast to double to keep precision of the division */

        Tpayload = payloadSymbNb * Tsym;

        /* Duration of packet */
        Tpacket = Tpreamble + Tpayload;
    } else if (packet->modulation == MOD_FSK) {
        /* PREAMBLE + SYNC_WORD + PKT_LEN + PKT_PAYLOAD + CRC
                PREAMBLE: default 5 bytes
                SYNC_WORD: default 3 bytes
                PKT_LEN: 1 byte (variable length mode)
                PKT_PAYLOAD: x bytes
                CRC: 0 or 2 bytes
        */
        Tfsk = (8 * (double)(packet->preamble + fsk_sync_word_size + 1 + packet->size + ((packet->no_crc == true) ? 0 : 2)) / (double)packet->datarate) * 1E3;

        /* Duration of packet */
        Tpacket = (uint32_t)Tfsk + 1; /* add margin for rounding */
    } else {
        Tpacket = 0;
        DEBUG_PRINTF("ERROR: Cannot compute time on air for this packet, unsupported modulation (0x%02X)\n", packet->modulation);
    }

    return Tpacket;
}

/* --- EOF ------------------------------------------------------------------ */
