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

#include "loragw_reg.h"
#include "loragw_hal.h"
#include "loragw_aux.h"
#include "loragw_spi.h"
#include "loragw_sx1250.h"
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
#define LGW_RF_RX_BANDWIDTH_125KHZ  925000      /* for 125KHz channels */
#define LGW_RF_RX_BANDWIDTH_250KHZ  1000000     /* for 250KHz channels */
#define LGW_RF_RX_BANDWIDTH_500KHZ  1100000     /* for 500KHz channels */

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
#include "src/test_bao_sx1262.var"

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

/* -------------------------------------------------------------------------- */
/* --- PRIVATE FUNCTIONS DECLARATION ---------------------------------------- */

int load_firmware_agc(const uint8_t * firmware);

void lgw_constant_adjust(void);

int32_t lgw_sf_getval(int x);
int32_t lgw_bw_getval(int x);

/* -------------------------------------------------------------------------- */
/* --- PRIVATE FUNCTIONS DEFINITION ----------------------------------------- */

/* size is the firmware size in bytes (not 14b words) */
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
    lgw_reg_w(SX1302_REG_GPIO_GPIO_DIR_DIRECTION, 0xFF); /* GPIO output direction */

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

    /* check input range (segfault prevention) */
    if (rf_chain >= LGW_RF_CHAIN_NB) {
        DEBUG_MSG("ERROR: NOT A VALID RF_CHAIN NUMBER\n");
        return LGW_HAL_ERROR;
    }

    /* check if radio type is supported */
    if ((conf.type != LGW_RADIO_TYPE_SX1255) && (conf.type != LGW_RADIO_TYPE_SX1257) && (conf.type != LGW_RADIO_TYPE_SX1250)) {
        DEBUG_MSG("ERROR: NOT A VALID RADIO TYPE\n");
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
        sx1250_setup(0);
    }
    sx1302_radio_set_mode(0, radio_type);

    /* setup radio B */
    radio_type = ((rf_radio_type[1] == LGW_RADIO_TYPE_SX1250) ? SX1302_RADIO_TYPE_SX1250 : SX1302_RADIO_TYPE_SX125X);
    if (rf_enable[1] == true) {
        sx1302_radio_reset(1, radio_type);
        sx1250_setup(1);
    }
    sx1302_radio_set_mode(1, radio_type);

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

    /* configure LoRa 'stand-alone' modem */
    /* TODO */

    /* configure FSK modem */
    /* TODO */

    /* Load firmware */
    switch (rf_radio_type[0]) {
        case LGW_RADIO_TYPE_SX1250:
            load_firmware_agc(agc_firmware); /* TODO: check version */
            break;
        case LGW_RADIO_TYPE_SX1257:
            break;
        default:
            break;
    }

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
    /* TODO */
    if (max_pkt > 0) pkt_data->freq_hz = 0;

    return 0;
}

/* ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ */

int lgw_send(struct lgw_pkt_tx_s pkt_data) {
    uint8_t buff[16];
    uint32_t freq_reg;
    uint32_t freq_dev;
    int32_t val;
    uint16_t tx_start_delay;

    /* Check if there is a TX on-going */
    /* TODO */

#if !__SX1302_TODO__ /* TODO: should be done by AGC fw */
    /* give radio control to HOST */
    lgw_reg_w(SX1302_REG_COMMON_CTRL0_HOST_RADIO_CTRL, 0x01);

    buff[0] = 0x08;
    buff[1] = 0xE6;
    buff[2] = 0x1C;
    sx1250_write_command(0, WRITE_REGISTER, buff, 3); /* ?? */

    buff[0] = 0x01; /* LoRa */
    sx1250_write_command(0, SET_PACKET_TYPE, buff, 1);

    freq_reg = SX1250_FREQ_TO_REG(pkt_data.freq_hz);
    buff[0] = (uint8_t)(freq_reg >> 24);
    buff[1] = (uint8_t)(freq_reg >> 16);
    buff[2] = (uint8_t)(freq_reg >> 8);
    buff[3] = (uint8_t)(freq_reg >> 0);
    sx1250_write_command(0, SET_RF_FREQUENCY, buff, 4);

    buff[0] = 0x0E; /* power */
    buff[1] = 0x02; /* RAMP_40U */
    sx1250_write_command(0, SET_TX_PARAMS, buff, 2);

    buff[0] = 0x04; /* paDutyCycle */
    buff[1] = 0x07; /* hpMax */
    buff[2] = 0x00; /* deviceSel */
    buff[3] = 0x01; /* paLut */
    sx1250_write_command(0, SET_PA_CONFIG, buff, 4); /* SX1250 Output Power +22dBm */

    /* give radio control to AGC MCU */
    lgw_reg_w(SX1302_REG_COMMON_CTRL0_HOST_RADIO_CTRL, 0x00);
#endif

    lgw_reg_w(SX1302_REG_TX_TOP_A_TX_RFFE_IF_CTRL_PLL_DIV_CTRL, 0x00); /* VCO divider by 2 */
    lgw_reg_w(SX1302_REG_TX_TOP_A_TX_RFFE_IF_CTRL_TX_IF_SRC, 0x01); /* LoRa */
    lgw_reg_w(SX1302_REG_TX_TOP_A_TX_RFFE_IF_CTRL_TX_IF_DST, 0x01); /* SX126x Tx RFFE */
    lgw_reg_w(SX1302_REG_TX_TOP_A_TX_RFFE_IF_CTRL_TX_MODE, 0x01); /* Modulation */
    lgw_reg_w(SX1302_REG_TX_TOP_A_TX_RFFE_IF_CTRL_TX_CLK_EDGE, 0x00); /* Data on rising edge */
    lgw_reg_w(SX1302_REG_TX_TOP_A_GEN_CFG_0_MODULATION_TYPE, 0x00); /* LoRa */

    /* Set Tx frequency */
    freq_reg = SX1302_FREQ_TO_REG(pkt_data.freq_hz);
    lgw_reg_w(SX1302_REG_TX_TOP_A_TX_RFFE_IF_FREQ_RF_H_FREQ_RF, (freq_reg >> 16) & 0xFF);
    lgw_reg_w(SX1302_REG_TX_TOP_A_TX_RFFE_IF_FREQ_RF_M_FREQ_RF, (freq_reg >> 8) & 0xFF);
    lgw_reg_w(SX1302_REG_TX_TOP_A_TX_RFFE_IF_FREQ_RF_L_FREQ_RF, (freq_reg >> 0) & 0xFF);

    /* Set bandwidth */
    printf("Bandwidth %dkHz\n", (int)(lgw_bw_getval(pkt_data.bandwidth) / 1E3));
    freq_dev = lgw_bw_getval(pkt_data.bandwidth) / 2;
    freq_reg = SX1302_FREQ_TO_REG(freq_dev);
    lgw_reg_w(SX1302_REG_TX_TOP_A_TX_RFFE_IF_FREQ_DEV_H_FREQ_DEV, (freq_reg >>  8) & 0xFF);
    lgw_reg_w(SX1302_REG_TX_TOP_A_TX_RFFE_IF_FREQ_DEV_L_FREQ_DEV, (freq_reg >>  0) & 0xFF);

    /* Condifure modem */
    switch (pkt_data.modulation) {
        case MOD_LORA:
            lgw_reg_w(SX1302_REG_TX_TOP_A_TXRX_CFG0_0_MODEM_BW, pkt_data.bandwidth);

            /* Preamble length */
            lgw_reg_w(SX1302_REG_TX_TOP_A_TXRX_CFG1_3_PREAMBLE_SYMB_NB, (pkt_data.preamble >> 8) & 0xFF); /* MSB */
            lgw_reg_w(SX1302_REG_TX_TOP_A_TXRX_CFG1_2_PREAMBLE_SYMB_NB, (pkt_data.preamble >> 0) & 0xFF); /* LSB */

            /* LoRa datarate */
            lgw_reg_w(SX1302_REG_TX_TOP_A_TXRX_CFG0_0_MODEM_SF, pkt_data.datarate);
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

            /*  */
            lgw_reg_w(SX1302_REG_TX_TOP_A_TX_CFG0_0_CHIRP_INVERT, (pkt_data.invert_pol) ? 1 : 0);
            lgw_reg_w(SX1302_REG_TX_TOP_A_TXRX_CFG0_2_IMPLICIT_HEADER, (pkt_data.no_header) ? 1 : 0); /*  */
            lgw_reg_w(SX1302_REG_TX_TOP_A_TXRX_CFG0_2_CRC_EN, (pkt_data.no_crc) ? 0 : 1); /*  */

            lgw_reg_w(SX1302_REG_TX_TOP_A_FRAME_SYNCH_0_PEAK1_POS, 6); /* public */
            lgw_reg_w(SX1302_REG_TX_TOP_A_FRAME_SYNCH_1_PEAK2_POS, 8); /* public */

            /* Set Payload length */
            lgw_reg_w(SX1302_REG_TX_TOP_A_TXRX_CFG0_3_PAYLOAD_LENGTH, pkt_data.size);
            break;
        default:
            printf("ERROR: Modulation not supported\n");
            return LGW_HAL_ERROR;
    }

    /* Set TX start delay */
    tx_start_delay = 1500 * 32; /* us */ /* TODO: which value should we put?? */
    lgw_reg_w(SX1302_REG_TX_TOP_A_TX_START_DELAY_MSB_TX_START_DELAY, (uint8_t)(tx_start_delay >> 8));
    lgw_reg_w(SX1302_REG_TX_TOP_A_TX_START_DELAY_LSB_TX_START_DELAY, (uint8_t)(tx_start_delay >> 0));

    /* Write payload in transmit buffer */
    lgw_reg_w(SX1302_REG_TX_TOP_A_TX_CTRL_WRITE_BUFFER, 0x01);
    lgw_mem_wb(0x5300, &(pkt_data.payload[0]), pkt_data.size);
    lgw_reg_w(SX1302_REG_TX_TOP_A_TX_CTRL_WRITE_BUFFER, 0x00);

    /* Trigger transmit */
    printf("Start Tx: Freq:%u SF%u size:%u\n", pkt_data.freq_hz, pkt_data.datarate, pkt_data.size);
    switch (pkt_data.tx_mode) {
        case IMMEDIATE:
            lgw_reg_w(SX1302_REG_TX_TOP_A_TX_TRIG_TX_TRIG_IMMEDIATE, 0x01);
            break;
        case TIMESTAMPED:
            lgw_reg_w(SX1302_REG_TX_TOP_B_TX_TRIG_TX_TRIG_DELAYED, 0x01);
            break;
        case ON_GPS:
            lgw_reg_w(SX1302_REG_TX_TOP_B_TX_TRIG_TX_TRIG_GPS, 0x01);
            break;
        default:
            printf("ERROR: TX mode not supported\n");
            return LGW_HAL_ERROR;
    }

#if 1
    /* TODO: should be done by AGC fw? */
    do {
        //lgw_reg_r(SX1302_REG_TX_TOP_A_LORA_TX_STATE_STATUS, &val);
        //lgw_reg_r(SX1302_REG_TX_TOP_A_LORA_TX_FLAG_FRAME_DONE, &val);
        //lgw_reg_r(SX1302_REG_TX_TOP_B_LORA_TX_FLAG_CONT_DONE, &val);
        //printf("cont done 0x%02X\n", val);
        lgw_reg_r(SX1302_REG_TX_TOP_A_TX_STATUS_TX_STATUS, &val);
        wait_ms(10);
    } while (val != 0x80);

    printf("Stop Tx\n");
    lgw_reg_w(SX1302_REG_TX_TOP_A_TX_TRIG_TX_TRIG_IMMEDIATE, 0x00);
#endif

    return 0;
}

/* ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ */

int lgw_status(uint8_t select, uint8_t *code) {
    /* TODO */
    if (select == 0) *code = 0; /* dummy */

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
