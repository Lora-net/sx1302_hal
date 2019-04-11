/*
 / _____)             _              | |
( (____  _____ ____ _| |_ _____  ____| |__
 \____ \| ___ |    (_   _) ___ |/ ___)  _ \
 _____) ) ____| | | || |_| ____( (___| | | |
(______/|_____)_|_|_| \__)_____)\____)_| |_|
  (C)2018 Semtech-Cycleo

Description:
    LoRa concentrator Hardware Abstraction Layer

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

#include <stdint.h>     /* C99 types */
#include <stdbool.h>    /* bool type */
#include <stdio.h>      /* printf fprintf */
#include <string.h>     /* memcpy */
#include <math.h>       /* pow, cell */
#include <time.h>
#include <unistd.h>     /* symlink, unlink */
#include <fcntl.h>

#include "loragw_reg.h"
#include "loragw_hal.h"
#include "loragw_aux.h"
#include "loragw_spi.h"
#include "loragw_i2c.h"
#include "loragw_sx1250.h"
#include "loragw_sx125x.h"
#include "loragw_sx1302.h"
#include "loragw_stts751.h"
#include "loragw_cal.h"
#include "loragw_debug.h"

/* -------------------------------------------------------------------------- */
/* --- DEBUG CONSTANTS ------------------------------------------------------ */

#define HAL_DEBUG_FILE_LOG  0

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

#define FW_VERSION_CAL      1 /* Expected version of calibration firmware */
#define FW_VERSION_AGC      1 /* Expected version of AGC firmware */
#define FW_VERSION_ARB      1 /* Expected version of arbiter firmware */

#define MIN_LORA_PREAMBLE   6
#define STD_LORA_PREAMBLE   8
#define MIN_FSK_PREAMBLE    3
#define STD_FSK_PREAMBLE    5

#define RSSI_MULTI_BIAS     -35 /* difference between "multi" modem RSSI offset and "stand-alone" modem RSSI offset */
#define RSSI_FSK_POLY_0     86 /* polynomiam coefficients to linearize FSK RSSI */
#define RSSI_FSK_POLY_1     1
#define RSSI_FSK_POLY_2     0

/* Useful bandwidth of SX125x radios to consider depending on channel bandwidth */
/* Note: the below values come from lab measurements. For any question, please contact Semtech support */
#define LGW_RF_RX_BANDWIDTH_125KHZ  1600000     /* for 125KHz channels */
#define LGW_RF_RX_BANDWIDTH_250KHZ  1600000     /* for 250KHz channels */
#define LGW_RF_RX_BANDWIDTH_500KHZ  1600000     /* for 500KHz channels */

#define LGW_RF_RX_FREQ_MIN          100E6
#define LGW_RF_RX_FREQ_MAX          1E9

#define FREQ_OFFSET_LSB_125KHZ      0.11920929f     /* 125000 * 32 / 2^6 / 2^19 */
#define FREQ_OFFSET_LSB_250KHZ      0.238418579f    /* 250000 * 32 / 2^6 / 2^19 */
#define FREQ_OFFSET_LSB_500KHZ      0.476837158f    /* 500000 * 32 / 2^6 / 2^19 */

/* constant arrays defining hardware capability */
const uint8_t ifmod_config[LGW_IF_CHAIN_NB] = LGW_IFMODEM_CONFIG;

/* Version string, used to identify the library version/options once compiled */
const char lgw_version_string[] = "Version: " LIBLORAGW_VERSION ";";

/* -------------------------------------------------------------------------- */
/* --- PRIVATE VARIABLES ---------------------------------------------------- */

//#include "arb_fw.var" /* external definition of the variable */
//#include "agc_fw.var" /* external definition of the variable */
//#include "cal_fw.var" /* external definition of the variable */
#include "src/text_agc_sx1250_07_Fev_2019.var"
#include "src/text_agc_sx1257_19_Nov_1.var"
#include "src/text_cal_sx1257_16_Nov_1.var"
#include "src/text_arb_sx1302_13_Nov_3.var"

typedef struct lgw_context_s {
    /* Global context */
    bool                        is_started;
    struct lgw_conf_board_s     board_cfg;
    /* RX context */
    struct lgw_conf_rxrf_s      rf_chain_cfg[LGW_RF_CHAIN_NB];
    struct lgw_conf_rxif_s      if_chain_cfg[LGW_IF_CHAIN_NB];
    struct lgw_conf_rxif_s      lora_service_cfg;                       /* LoRa service channel config parameters */
    struct lgw_conf_rxif_s      fsk_cfg;                                /* FSK channel config parameters */
    /* TX context */
    struct lgw_tx_gain_lut_s    tx_gain_lut[LGW_RF_CHAIN_NB];
    /* Misc */
    struct lgw_conf_timestamp_s timestamp_cfg;
    /* Debug */
    struct lgw_conf_debug_s     debug_cfg;
} lgw_context_t;

#define CONTEXT_STARTED         lgw_context.is_started
#define CONTEXT_SPI             lgw_context.board_cfg.spidev_path
#define CONTEXT_LWAN_PUBLIC     lgw_context.board_cfg.lorawan_public
#define CONTEXT_BOARD           lgw_context.board_cfg
#define CONTEXT_RF_CHAIN        lgw_context.rf_chain_cfg
#define CONTEXT_IF_CHAIN        lgw_context.if_chain_cfg
#define CONTEXT_LORA_SERVICE    lgw_context.lora_service_cfg
#define CONTEXT_FSK             lgw_context.fsk_cfg
#define CONTEXT_TX_GAIN_LUT     lgw_context.tx_gain_lut
#define CONTEXT_TIMESTAMP       lgw_context.timestamp_cfg
#define CONTEXT_DEBUG           lgw_context.debug_cfg

/*
The following static variable holds the gateway configuration provided by the
user that need to be propagated in the drivers.

Parameters validity and coherency is verified by the _setconf functions and
the _start and _send functions assume they are valid.
*/
static lgw_context_t lgw_context = {
    .is_started = false,
    .board_cfg.spidev_path = "/dev/spidev0.0",
    .board_cfg.lorawan_public = true,
    .board_cfg.clksrc = 0,
    .board_cfg.full_duplex = false,
    .rf_chain_cfg = {{0}},
    .if_chain_cfg = {{0}},
    .lora_service_cfg = {
        .enable = 0,    /* not used, handled by if_chain_cfg */
        .rf_chain = 0,  /* not used, handled by if_chain_cfg */
        .freq_hz = 0,   /* not used, handled by if_chain_cfg */
        .bandwidth = BW_250KHZ,
        .datarate = DR_LORA_SF7,
        .implicit_hdr = false,
        .implicit_payload_length = 0,
        .implicit_crc_en = 0,
        .implicit_coderate = 0
    },
    .fsk_cfg = {
        .enable = 0,    /* not used, handled by if_chain_cfg */
        .rf_chain = 0,  /* not used, handled by if_chain_cfg */
        .freq_hz = 0,   /* not used, handled by if_chain_cfg */
        .bandwidth = BW_125KHZ,
        .datarate = 50000,
        .sync_word_size = 3,
        .sync_word = 0xC194C1
    },
    .tx_gain_lut = {
        {
            .size = 1,
            .lut[0] = {
                .rf_power = 14,
                .dig_gain = 0,
                .pa_gain = 2,
                .dac_gain = 3,
                .mix_gain = 10,
                .offset_i = 0,
                .offset_q = 0,
                .pwr_idx = 0
            }
        },{
            .size = 1,
            .lut[0] = {
                .rf_power = 14,
                .dig_gain = 0,
                .pa_gain = 2,
                .dac_gain = 3,
                .mix_gain = 10,
                .offset_i = 0,
                .offset_q = 0,
                .pwr_idx = 0
            }
        }
    },
    .timestamp_cfg = {
        .enable_precision_ts = false,
        .max_ts_metrics = 0xFF,
        .nb_symbols = 1
    },
    .debug_cfg = {
        .nb_ref_payload = 0,
        .log_file_name = "loragw_hal.log"
    }
};

/* Buffer to fetch received packets from sx1302 */
static uint8_t rx_fifo[4096];

/* File handle to write debug logs */
static FILE * log_file = NULL;

/* File descriptor to I2C linux device */
int lgw_i2c_target = -1;

/* -------------------------------------------------------------------------- */
/* --- PRIVATE FUNCTIONS DECLARATION ---------------------------------------- */

int32_t lgw_sf_getval(int x);
int32_t lgw_bw_getval(int x);

/* -------------------------------------------------------------------------- */
/* --- PRIVATE FUNCTIONS DEFINITION ----------------------------------------- */

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

uint16_t lgw_get_tx_start_delay(lgw_radio_type_t radio_type, uint8_t modulation, uint8_t bw) {
    uint16_t tx_start_delay = TX_START_DELAY_DEFAULT * 32;
    uint16_t radio_bw_delay = 0;
    uint16_t filter_delay = 0;
    uint16_t modem_delay = 0;
    int32_t bw_hz = lgw_bw_getval(bw);
    int32_t val;
    uint8_t chirp_low_pass = 0;

    /* Adjust with radio type and bandwidth */
    switch (radio_type) {
        case LGW_RADIO_TYPE_SX1250:
            if (bw == BW_125KHZ) {
                radio_bw_delay = 19;
            } else if (bw == BW_250KHZ) {
                radio_bw_delay = 24;
            } else if (bw == BW_500KHZ) {
                radio_bw_delay = 21;
            } else {
                DEBUG_MSG("ERROR: bandwidth not supported\n");
                return LGW_HAL_ERROR;
            }
            break;
        case LGW_RADIO_TYPE_SX1255:
        case LGW_RADIO_TYPE_SX1257:
            radio_bw_delay = 3*32 + 4;
            if (bw == BW_125KHZ) {
                radio_bw_delay += 0;
            } else if (bw == BW_250KHZ) {
                radio_bw_delay += 6;
            } else if (bw == BW_500KHZ) {
                radio_bw_delay += 0;
            } else {
                DEBUG_MSG("ERROR: bandwidth not supported\n");
                return LGW_HAL_ERROR;
            }
            break;
        default:
            DEBUG_MSG("ERROR: radio type not supported\n");
            return LGW_HAL_ERROR;
    }

    /* Adjust with modulation */
    if (modulation == MOD_LORA) {
        lgw_reg_r(SX1302_REG_TX_TOP_TX_CFG0_0_CHIRP_LOWPASS(0), &val);
        chirp_low_pass = (uint8_t)val;
        filter_delay = ((1 << chirp_low_pass) - 1) * 1e6 / bw_hz;
        modem_delay = 8 * (32e6 / (32 * bw_hz)); /* if bw=125k then modem freq=4MHz */
    } else {
        /* TODO */
        filter_delay = 0;
        modem_delay = 0;
    }

    /* Compute total delay */
    tx_start_delay -= (radio_bw_delay + filter_delay + modem_delay);

    printf("INFO: tx_start_delay=%u (%u, radio_bw_delay=%u, filter_delay=%u, modem_delay=%u)\n", (uint16_t)tx_start_delay, TX_START_DELAY_DEFAULT*32, radio_bw_delay, filter_delay, modem_delay);

    return tx_start_delay;
}

/* -------------------------------------------------------------------------- */
/* --- PUBLIC FUNCTIONS DEFINITION ------------------------------------------ */

int lgw_board_setconf(struct lgw_conf_board_s conf) {

    /* check if the concentrator is running */
    if (CONTEXT_STARTED == true) {
        DEBUG_MSG("ERROR: CONCENTRATOR IS RUNNING, STOP IT BEFORE TOUCHING CONFIGURATION\n");
        return LGW_HAL_ERROR;
    }

    /* set internal config according to parameters */
    CONTEXT_LWAN_PUBLIC = conf.lorawan_public;
    CONTEXT_BOARD.clksrc = conf.clksrc;
    CONTEXT_BOARD.full_duplex = conf.full_duplex;
    strncpy(CONTEXT_SPI, conf.spidev_path, sizeof CONTEXT_SPI);

    DEBUG_PRINTF("Note: board configuration: spidev_path: %s, lorawan_public:%d, clksrc:%d, full_duplex:%d\n",  CONTEXT_SPI,
                                                                                                                CONTEXT_LWAN_PUBLIC,
                                                                                                                CONTEXT_BOARD.clksrc,
                                                                                                                CONTEXT_BOARD.full_duplex);

    return LGW_HAL_SUCCESS;
}

/* ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ */

int lgw_rxrf_setconf(uint8_t rf_chain, struct lgw_conf_rxrf_s conf) {

    /* check if the concentrator is running */
    if (CONTEXT_STARTED == true) {
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

    /* check if the radio central frequency is valid */
    if ((conf.freq_hz < LGW_RF_RX_FREQ_MIN) || (conf.freq_hz > LGW_RF_RX_FREQ_MAX)) {
        DEBUG_PRINTF("ERROR: NOT A VALID RADIO CENTER FREQUENCY, PLEASE CHECK IF IT HAS BEEN GIVEN IN HZ (%u)\n", conf.freq_hz);
        return LGW_HAL_ERROR;
    }

    /* set internal config according to parameters */
    CONTEXT_RF_CHAIN[rf_chain].enable = conf.enable;
    CONTEXT_RF_CHAIN[rf_chain].freq_hz = conf.freq_hz;
    CONTEXT_RF_CHAIN[rf_chain].rssi_offset = conf.rssi_offset;
    CONTEXT_RF_CHAIN[rf_chain].type = conf.type;
    CONTEXT_RF_CHAIN[rf_chain].tx_enable = conf.tx_enable;

    DEBUG_PRINTF("Note: rf_chain %d configuration; en:%d freq:%d rssi_offset:%f radio_type:%d tx_enable:%d\n",  rf_chain,
                                                                                                                CONTEXT_RF_CHAIN[rf_chain].enable,
                                                                                                                CONTEXT_RF_CHAIN[rf_chain].freq_hz,
                                                                                                                CONTEXT_RF_CHAIN[rf_chain].rssi_offset,
                                                                                                                CONTEXT_RF_CHAIN[rf_chain].type,
                                                                                                                CONTEXT_RF_CHAIN[rf_chain].tx_enable);

    return LGW_HAL_SUCCESS;
}

/* ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ */

int lgw_rxif_setconf(uint8_t if_chain, struct lgw_conf_rxif_s conf) {
    int32_t bw_hz;
    uint32_t rf_rx_bandwidth;

    /* check if the concentrator is running */
    if (CONTEXT_STARTED == true) {
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
        CONTEXT_IF_CHAIN[if_chain].enable = false;
        CONTEXT_IF_CHAIN[if_chain].freq_hz = 0;
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
                conf.datarate = DR_LORA_SF7;
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
            CONTEXT_IF_CHAIN[if_chain].enable = conf.enable;
            CONTEXT_IF_CHAIN[if_chain].rf_chain = conf.rf_chain;
            CONTEXT_IF_CHAIN[if_chain].freq_hz = conf.freq_hz;
            CONTEXT_LORA_SERVICE.bandwidth = conf.bandwidth;
            CONTEXT_LORA_SERVICE.datarate = conf.datarate;
            CONTEXT_LORA_SERVICE.implicit_hdr = conf.implicit_hdr;
            CONTEXT_LORA_SERVICE.implicit_payload_length = conf.implicit_payload_length;
            CONTEXT_LORA_SERVICE.implicit_crc_en   = conf.implicit_crc_en;
            CONTEXT_LORA_SERVICE.implicit_coderate = conf.implicit_coderate;

            DEBUG_PRINTF("Note: LoRa 'std' if_chain %d configuration; en:%d freq:%d bw:%d dr:%d\n", if_chain,
                                                                                                    CONTEXT_IF_CHAIN[if_chain].enable,
                                                                                                    CONTEXT_IF_CHAIN[if_chain].freq_hz,
                                                                                                    CONTEXT_LORA_SERVICE.bandwidth,
                                                                                                    CONTEXT_LORA_SERVICE.datarate);
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
            CONTEXT_IF_CHAIN[if_chain].enable = conf.enable;
            CONTEXT_IF_CHAIN[if_chain].rf_chain = conf.rf_chain;
            CONTEXT_IF_CHAIN[if_chain].freq_hz = conf.freq_hz;

            DEBUG_PRINTF("Note: LoRa 'multi' if_chain %d configuration; en:%d freq:%d\n",   if_chain,
                                                                                            CONTEXT_IF_CHAIN[if_chain].enable,
                                                                                            CONTEXT_IF_CHAIN[if_chain].freq_hz);
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
            CONTEXT_IF_CHAIN[if_chain].enable = conf.enable;
            CONTEXT_IF_CHAIN[if_chain].rf_chain = conf.rf_chain;
            CONTEXT_IF_CHAIN[if_chain].freq_hz = conf.freq_hz;
            CONTEXT_FSK.bandwidth = conf.bandwidth;
            CONTEXT_FSK.datarate = conf.datarate;
            if (conf.sync_word > 0) {
                CONTEXT_FSK.sync_word_size = conf.sync_word_size;
                CONTEXT_FSK.sync_word = conf.sync_word;
            }
            DEBUG_PRINTF("Note: FSK if_chain %d configuration; en:%d freq:%d bw:%d dr:%d (%d real dr) sync:0x%0*llX\n", if_chain,
                                                                                                                        CONTEXT_IF_CHAIN[if_chain].enable,
                                                                                                                        CONTEXT_IF_CHAIN[if_chain].freq_hz,
                                                                                                                        CONTEXT_FSK.bandwidth,
                                                                                                                        CONTEXT_FSK.datarate,
                                                                                                                        LGW_XTAL_FREQU/(LGW_XTAL_FREQU/CONTEXT_FSK.datarate),
                                                                                                                        2*CONTEXT_FSK.sync_word_size,
                                                                                                                        CONTEXT_FSK.sync_word);
            break;

        default:
            DEBUG_PRINTF("ERROR: IF CHAIN %d TYPE NOT SUPPORTED\n", if_chain);
            return LGW_HAL_ERROR;
    }

    return LGW_HAL_SUCCESS;
}

/* ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ */

int lgw_txgain_setconf(uint8_t rf_chain, struct lgw_tx_gain_lut_s *conf) {
    int i;

    /* Check LUT size */
    if ((conf->size < 1) || (conf->size > TX_GAIN_LUT_SIZE_MAX)) {
        DEBUG_PRINTF("ERROR: TX gain LUT must have at least one entry and  maximum %d entries\n", TX_GAIN_LUT_SIZE_MAX);
        return LGW_HAL_ERROR;
    }

    CONTEXT_TX_GAIN_LUT[rf_chain].size = conf->size;

    for (i = 0; i < CONTEXT_TX_GAIN_LUT[rf_chain].size; i++) {
        /* Check gain range */
        if (conf->lut[i].dig_gain > 3) {
            DEBUG_MSG("ERROR: TX gain LUT: SX1302 digital gain must be between 0 and 3\n");
            return LGW_HAL_ERROR;
        }
        if (conf->lut[i].dac_gain > 3) {
            DEBUG_MSG("ERROR: TX gain LUT: SX1257 DAC gains must not exceed 3\n");
            return LGW_HAL_ERROR;
        }
        if ((conf->lut[i].mix_gain < 5) || (conf->lut[i].mix_gain > 15)) {
            DEBUG_MSG("ERROR: TX gain LUT: SX1257 mixer gain must be betwen [5..15]\n");
            return LGW_HAL_ERROR;
        }
        if (conf->lut[i].pa_gain > 3) {
            DEBUG_MSG("ERROR: TX gain LUT: External PA gain must not exceed 3\n");
            return LGW_HAL_ERROR;
        }
        if (conf->lut[i].pwr_idx > 31) {
            DEBUG_MSG("ERROR: TX gain LUT: SX1250 power iundex must not exceed 31\n");
            return LGW_HAL_ERROR;
        }

        /* Set internal LUT */
        CONTEXT_TX_GAIN_LUT[rf_chain].lut[i].rf_power = conf->lut[i].rf_power;
        CONTEXT_TX_GAIN_LUT[rf_chain].lut[i].dig_gain = conf->lut[i].dig_gain;
        CONTEXT_TX_GAIN_LUT[rf_chain].lut[i].pa_gain  = conf->lut[i].pa_gain;
        /* sx125x */
        CONTEXT_TX_GAIN_LUT[rf_chain].lut[i].dac_gain = conf->lut[i].dac_gain;
        CONTEXT_TX_GAIN_LUT[rf_chain].lut[i].mix_gain = conf->lut[i].mix_gain;
        CONTEXT_TX_GAIN_LUT[rf_chain].lut[i].offset_i = 0; /* To be calibrated */
        CONTEXT_TX_GAIN_LUT[rf_chain].lut[i].offset_q = 0; /* To be calibrated */

        /* sx1250 */
        CONTEXT_TX_GAIN_LUT[rf_chain].lut[i].pwr_idx = conf->lut[i].pwr_idx;
    }

    return LGW_HAL_SUCCESS;
}

/* ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ */

int lgw_timestamp_setconf(struct lgw_conf_timestamp_s *conf) {
    CONTEXT_TIMESTAMP.enable_precision_ts = conf->enable_precision_ts;
    CONTEXT_TIMESTAMP.max_ts_metrics = conf->max_ts_metrics;
    CONTEXT_TIMESTAMP.nb_symbols = conf->nb_symbols;

    return LGW_HAL_SUCCESS;
}

/* ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ */

int lgw_debug_setconf(struct lgw_conf_debug_s *conf) {
    int i;

    CONTEXT_DEBUG.nb_ref_payload = conf->nb_ref_payload;
    for (i = 0; i < CONTEXT_DEBUG.nb_ref_payload; i++) {
        /* Get user configuration */
        CONTEXT_DEBUG.ref_payload[i].id = conf->ref_payload[i].id;

        /* Initialize global context */
        CONTEXT_DEBUG.ref_payload[i].prev_cnt = 0;
        CONTEXT_DEBUG.ref_payload[i].payload[0] = (uint8_t)(CONTEXT_DEBUG.ref_payload[i].id >> 24);
        CONTEXT_DEBUG.ref_payload[i].payload[1] = (uint8_t)(CONTEXT_DEBUG.ref_payload[i].id >> 16);
        CONTEXT_DEBUG.ref_payload[i].payload[2] = (uint8_t)(CONTEXT_DEBUG.ref_payload[i].id >> 8);
        CONTEXT_DEBUG.ref_payload[i].payload[3] = (uint8_t)(CONTEXT_DEBUG.ref_payload[i].id >> 0);
    }

    if (conf->log_file_name != NULL) {
        strncpy(CONTEXT_DEBUG.log_file_name, conf->log_file_name, strlen(conf->log_file_name));
    }

    return LGW_HAL_SUCCESS;
}

/* ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ */

int lgw_start(void) {
    int i, err;
    uint32_t val, val2;
    int reg_stat;

    if (CONTEXT_STARTED == true) {
        DEBUG_MSG("Note: LoRa concentrator already started, restarting it now\n");
    }

    reg_stat = lgw_connect(CONTEXT_SPI);
    if (reg_stat == LGW_REG_ERROR) {
        DEBUG_MSG("ERROR: FAIL TO CONNECT BOARD\n");
        return LGW_HAL_ERROR;
    }

    /* Radio calibration - START */
    /* -- Reset radios */
    for (i = 0; i < LGW_RF_CHAIN_NB; i++) {
        if (CONTEXT_RF_CHAIN[i].enable == true) {
            sx1302_radio_reset(i, CONTEXT_RF_CHAIN[i].type);
            sx1302_radio_set_mode(i, CONTEXT_RF_CHAIN[i].type);
        }
    }
    /* -- Select the radio which provides the clock to the sx1302 */
    sx1302_radio_clock_select(CONTEXT_BOARD.clksrc);

    /* -- Ensure PA/LNA are disabled */
    lgw_reg_w(SX1302_REG_AGC_MCU_CTRL_FORCE_HOST_FE_CTRL, 1);
    lgw_reg_w(SX1302_REG_AGC_MCU_RF_EN_A_PA_EN, 0);
    lgw_reg_w(SX1302_REG_AGC_MCU_RF_EN_A_LNA_EN, 0);
    /* -- Start calibration */
    if ((CONTEXT_RF_CHAIN[CONTEXT_BOARD.clksrc].type == LGW_RADIO_TYPE_SX1257) ||
        (CONTEXT_RF_CHAIN[CONTEXT_BOARD.clksrc].type == LGW_RADIO_TYPE_SX1255)) {
        printf("Loading CAL fw for sx125x\n");
        if (sx1302_agc_load_firmware(cal_firmware_sx125x) != LGW_HAL_SUCCESS) {
            printf("ERROR: Failed to load calibration fw\n");
            return LGW_HAL_ERROR;
        }
        if (sx1302_cal_start(FW_VERSION_CAL, CONTEXT_RF_CHAIN, &CONTEXT_TX_GAIN_LUT[0]) != LGW_HAL_SUCCESS) {
            printf("ERROR: radio calibration failed\n");
            sx1302_radio_reset(0, CONTEXT_RF_CHAIN[0].type);
            sx1302_radio_reset(1, CONTEXT_RF_CHAIN[1].type);
            return LGW_HAL_ERROR;
        }
    } else {
        printf("Calibrating sx1250 radios\n");
        for (i = 0; i < LGW_RF_CHAIN_NB; i++) {
            if (CONTEXT_RF_CHAIN[i].enable == true) {
                if (sx1250_calibrate(i, CONTEXT_RF_CHAIN[i].freq_hz)) {
                    printf("ERROR: radio calibration failed\n");
                    return LGW_HAL_ERROR;
                }
            }
        }
    }
    /* -- Release control over FE */
    lgw_reg_w(SX1302_REG_AGC_MCU_CTRL_FORCE_HOST_FE_CTRL, 0);
    /* Radio calibration - END */

    /* Setup radios for RX */
    for (i = 0; i < LGW_RF_CHAIN_NB; i++) {
        if (CONTEXT_RF_CHAIN[i].enable == true) {
            sx1302_radio_reset(i, CONTEXT_RF_CHAIN[i].type);
            switch (CONTEXT_RF_CHAIN[i].type) {
                case LGW_RADIO_TYPE_SX1250:
                    sx1250_setup(i, CONTEXT_RF_CHAIN[i].freq_hz);
                    break;
                case LGW_RADIO_TYPE_SX1255:
                case LGW_RADIO_TYPE_SX1257:
                    sx125x_setup(i, CONTEXT_BOARD.clksrc, true, CONTEXT_RF_CHAIN[i].type, CONTEXT_RF_CHAIN[i].freq_hz);
                    break;
                default:
                    DEBUG_PRINTF("ERROR: RADIO TYPE NOT SUPPORTED (RF_CHAIN %d)\n", i);
                    return LGW_HAL_ERROR;
            }
            sx1302_radio_set_mode(i, CONTEXT_RF_CHAIN[i].type);
        }
    }

    /* Select the radio which provides the clock to the sx1302 */
    sx1302_radio_clock_select(CONTEXT_BOARD.clksrc);

    /* Release host control on radio (will be controlled by AGC) */
    sx1302_radio_host_ctrl(false);

    /* Check that the SX1302 timestamp counter is running */
    lgw_get_instcnt(&val);
    lgw_get_instcnt(&val2);
    if (val == val2) {
        printf("ERROR: SX1302 timestamp counter is not running (val:%u)\n", (uint32_t)val);
        return -1;
    }

    /* Configure PA/LNA LUTs */
    lgw_reg_w(SX1302_REG_AGC_MCU_LUT_TABLE_A_PA_LUT, 0x04);     /* Enable PA: RADIO_CTRL[2] is high when PA_EN=1 & LNA_EN=0 */
    lgw_reg_w(SX1302_REG_AGC_MCU_LUT_TABLE_B_PA_LUT, 0x04);     /* Enable PA: RADIO_CTRL[8] is high when PA_EN=1 & LNA_EN=0 */
    lgw_reg_w(SX1302_REG_AGC_MCU_LUT_TABLE_A_LNA_LUT, 0x02);    /* Enable LNA: RADIO_CTRL[1] is high when PA_EN=0 & LNA_EN=1 */
    lgw_reg_w(SX1302_REG_AGC_MCU_LUT_TABLE_B_LNA_LUT, 0x02);    /* Enable LNA: RADIO_CTRL[7] is high when PA_EN=0 & LNA_EN=1 */

    /* Configure Radio FE */
    sx1302_radio_fe_configure();

    /* Configure the Channelizer */
    sx1302_channelizer_configure(CONTEXT_IF_CHAIN, false);

    /* configure LoRa 'multi' demodulators */
    sx1302_lora_correlator_configure();
    sx1302_lora_modem_configure(CONTEXT_RF_CHAIN[0].freq_hz); /* TODO: freq_hz used to confiogure freq to time drift, based on RF0 center freq only */

    /* configure LoRa 'stand-alone' modem */
    if (CONTEXT_IF_CHAIN[8].enable == true) {
        sx1302_lora_service_correlator_configure(&(CONTEXT_LORA_SERVICE));
        sx1302_lora_service_modem_configure(&(CONTEXT_LORA_SERVICE), CONTEXT_RF_CHAIN[0].freq_hz);  /* TODO: freq_hz used to confiogure freq to time drift, based on RF0 center freq only */
    }

    /* configure FSK modem */
    if (CONTEXT_IF_CHAIN[9].enable == true) {
        sx1302_fsk_configure(&(CONTEXT_FSK));
    }

    /* configure syncword */
    sx1302_lora_syncword(CONTEXT_LWAN_PUBLIC, CONTEXT_LORA_SERVICE.datarate);

    /* configure timestamp */
    sx1302_timestamp_mode(&CONTEXT_TIMESTAMP);

    /* Load firmware */
    switch (CONTEXT_RF_CHAIN[CONTEXT_BOARD.clksrc].type) {
        case LGW_RADIO_TYPE_SX1250:
            printf("Loading AGC fw for sx1250\n");
            if (sx1302_agc_load_firmware(agc_firmware_sx1250) != LGW_HAL_SUCCESS) {
                return LGW_HAL_ERROR;
            }
            break;
        case LGW_RADIO_TYPE_SX1257:
            printf("Loading AGC fw for sx125x\n");
            if (sx1302_agc_load_firmware(agc_firmware_sx125x) != LGW_HAL_SUCCESS) {
                return LGW_HAL_ERROR;
            }
            break;
        default:
            break;
    }
    if (sx1302_agc_start(FW_VERSION_AGC, CONTEXT_RF_CHAIN[CONTEXT_BOARD.clksrc].type, SX1302_AGC_RADIO_GAIN_AUTO, SX1302_AGC_RADIO_GAIN_AUTO, (CONTEXT_BOARD.full_duplex == true) ? 1 : 0) != LGW_HAL_SUCCESS) {
        return LGW_HAL_ERROR;
    }
    printf("Loading ARB fw\n");
    if (sx1302_arb_load_firmware(arb_firmware) != LGW_HAL_SUCCESS) {
        return LGW_HAL_ERROR;
    }
    if (sx1302_arb_start(FW_VERSION_ARB) != LGW_HAL_SUCCESS) {
        return LGW_HAL_ERROR;
    }

    /* enable demodulators */
    sx1302_modem_enable();

    /* enable GPS */
    sx1302_gps_enable(true);

    /* For debug logging */
#if HAL_DEBUG_FILE_LOG
    char timestamp_str[40];
    struct tm *timenow;

    /* Append current time to log file name */
    time_t now = time(NULL);
    timenow = gmtime(&now);
    strftime(timestamp_str, sizeof(timestamp_str), ".%Y-%m-%d_%H%M%S", timenow);
    strncat(CONTEXT_DEBUG.log_file_name, timestamp_str, sizeof CONTEXT_DEBUG.log_file_name);

    /* Open the file for writting */
    log_file = fopen(CONTEXT_DEBUG.log_file_name, "w+"); /* create log file, overwrite if file already exist */
    if (log_file == NULL) {
        printf("ERROR: impossible to create log file %s\n", CONTEXT_DEBUG.log_file_name);
        return LGW_HAL_ERROR;
    } else {
        printf("INFO: %s file opened for debug log\n", CONTEXT_DEBUG.log_file_name);

        /* Create "pktlog.csv" symlink to simplify user life */
        unlink("loragw_hal.log");
        i = symlink(CONTEXT_DEBUG.log_file_name, "loragw_hal.log");
        if (i < 0) {
            printf("ERROR: impossible to create symlink to log file %s\n", CONTEXT_DEBUG.log_file_name);
        }
    }
#endif

    /* Configure the pseudo-random generator (For Debug) */
    dbg_init_random();

#if 0
    /* Configure a GPIO to be toggled for debug purpose */
    dbg_init_gpio();
#endif

    /* Open I2C */
    err = i2c_linuxdev_open(I2C_DEVICE, I2C_PORT_TEMP_SENSOR, &lgw_i2c_target);
    if ((err != 0) || (lgw_i2c_target <= 0)) {
        printf("ERROR: failed to open I2C device %s (err=%i)\n", I2C_DEVICE, err);
        return LGW_HAL_ERROR;
    }

    /* Configure the corecell temperature sensor */
    if (lgw_stts751_configure() != LGW_I2C_SUCCESS) {
        printf("ERROR: failed to configure temperature sensor\n");
        return LGW_HAL_ERROR;
    }

    /* set hal state */
    CONTEXT_STARTED = true;

    return LGW_HAL_SUCCESS;
}

/* ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ */

int lgw_stop(void) {
    int i, err;

    DEBUG_MSG("INFO: aborting TX\n");
    for (i = 0; i < LGW_RF_CHAIN_NB; i++) {
        lgw_abort_tx(i);
    }

    /* Close log file */
    if (log_file != NULL) {
        fclose(log_file);
        log_file = NULL;
    }

    DEBUG_MSG("INFO: Disconnecting\n");
    lgw_disconnect();

    DEBUG_MSG("INFO: Closing I2C\n");
    err = i2c_linuxdev_close(lgw_i2c_target);
    if (err != 0) {
        printf("ERROR: failed to close I2C device (err=%i)\n", err);
        /* TODO: return error or not ? */
    }

    CONTEXT_STARTED = false;
    return LGW_HAL_SUCCESS;
}

/* ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ */

int lgw_receive(uint8_t max_pkt, struct lgw_pkt_rx_s *pkt_data) {
    int i, j, res;
    uint8_t buff[2];
    uint16_t sz = 0;
    uint16_t buffer_index;
    uint16_t nb_pkt_found = 0;
    uint16_t nb_pkt_dropped = 0;
    uint16_t payload_length;
    uint16_t payload_crc16_calc;
    uint16_t payload_crc16_read;
    int32_t freq_offset_raw, freq_offset;
    uint8_t num_ts_metrics = 0;
    uint8_t sanity_check;
    bool rx_buffer_error = false;
    int32_t val;
    uint16_t last_addr_read;
    uint16_t last_addr_write;
    uint32_t dummy;
    float current_temperature;

    struct lgw_pkt_rx_s *p;
    int ifmod; /* type of if_chain/modem a packet was received by */
    uint32_t sf, cr, bw_pow, crc_en, ppm; /* used to calculate timestamp correction */
    uint32_t delay_x, delay_y, delay_z; /* temporary variable for timestamp offset calculation */
    uint32_t timestamp_correction; /* correction to account for processing delay */

    /* Check that AGC/ARB firmwares are not corrupted */
    lgw_reg_r(SX1302_REG_AGC_MCU_CTRL_PARITY_ERROR, &val);
    if (val != 0) {
        printf("ERROR: Parity error check failed on AGC firmware\n");
        return LGW_HAL_ERROR;
    }
    lgw_reg_r(SX1302_REG_ARB_MCU_CTRL_PARITY_ERROR, &val);
    if (val != 0) {
        printf("ERROR: Parity error check failed on ARB firmware\n");
        return LGW_HAL_ERROR;
    }

    /* Check if there is data in the FIFO */
    lgw_reg_rb(SX1302_REG_RX_TOP_RX_BUFFER_NB_BYTES_MSB_RX_BUFFER_NB_BYTES, buff, sizeof buff);
    sz  = (uint16_t)((buff[0] << 8) & 0xFF00);
    sz |= (uint16_t)((buff[1] << 0) & 0x00FF);
    if( sz == 0 )
    {
        //printf( "No message in DSP%d FIFO\n", dsp);
        lgw_get_instcnt(&dummy); /* Maintain 27bits to 32bits internal counter conversion */
        return 0;
    }

#if 0
    /* Print statistics of number of detects and modem allocations from ARB for configured SF (see sx1302_arb_start()) */
    sx1302_arb_print_debug_stats(true);
#endif

    printf("-----------------\n");
    printf("lgw_receive()\n");
    printf("nb_bytes received: %u (%u %u)\n", sz, buff[1], buff[0]);

#if 0 /* FOR TESTING: Wait for FIFO to be full: 91 packets of 22 bytes => 4095 bytes */
      /* Need to have a device sending packets with 22-bytes payload */
    if (sz < (4095 - (SX1302_PKT_HEAD_METADATA + 22 + SX1302_PKT_TAIL_METADATA - 1))) {
        return 0;
    }
#endif

    /* read bytes from fifo */
    memset(rx_fifo, 0, sizeof rx_fifo);
    res = lgw_mem_rb(0x4000, rx_fifo, sz, true);
    if (res != LGW_REG_SUCCESS) {
        printf("ERROR: Failed to read RX buffer, SPI error\n");
        return 0;
    }

    /* print debug info : TODO to be removed */
    printf("RX_BUFFER: ");
    for (i = 0; i < sz; i++) {
        printf("%02X ", rx_fifo[i]);
    }
    printf("\n");
    last_addr_write = sx1302_rx_buffer_write_ptr_addr();
    last_addr_read = sx1302_rx_buffer_read_ptr_addr();
    printf("RX_BUFFER: write_ptr 0x%04X, read_ptr 0x%04X\n", last_addr_write, last_addr_read);

    /* Update counter wrap status for packet timestamp conversion (27bits -> 32bits) */
    lgw_get_instcnt(&dummy);

    /* Get the current temperature for further RSSI compensation : TODO */
    if (lgw_stts751_get_temperature(&current_temperature) != LGW_I2C_SUCCESS) {
        printf("ERROR: failed to get current temperature\n");
        return LGW_HAL_ERROR;
    }
    printf("INFO: current temperature is %f C\n", current_temperature);

    /* Parse raw data and fill messages array */
    buffer_index = 0;
    while ((nb_pkt_found <= max_pkt) && (buffer_index < sz)) {
        /* Get pkt sync words */
        if ((rx_fifo[buffer_index] != SX1302_PKT_SYNCWORD_BYTE_0) || (rx_fifo[buffer_index + 1] != SX1302_PKT_SYNCWORD_BYTE_1) ) {
            printf("INFO: searching syncword...\n");
            buffer_index++;
            continue;
        }
        printf("INFO: pkt syncword found at index %u\n", buffer_index);

        /* Get payload length */
        payload_length = SX1302_PKT_PAYLOAD_LENGTH(rx_fifo, buffer_index);

        /* Get fine timestamp metrics */
        num_ts_metrics = SX1302_PKT_NUM_TS_METRICS(rx_fifo, buffer_index + payload_length);
        if((buffer_index + SX1302_PKT_HEAD_METADATA + payload_length + SX1302_PKT_TAIL_METADATA + (2 * num_ts_metrics)) > sz) {
            printf("WARNING: aborting truncated message (size=%u), got %u messages\n", sz, nb_pkt_found);
            break;
        }

        /* checksum */
        uint8_t checksum_calc = 0;
        uint16_t checksum_pos = SX1302_PKT_HEAD_METADATA + payload_length + SX1302_PKT_TAIL_METADATA + (2 * num_ts_metrics) - 1;
        uint8_t checksum = rx_fifo[buffer_index + checksum_pos];
        for (i = 0; i < (int)checksum_pos; i++) {
            checksum_calc += rx_fifo[buffer_index + i];
        }
        if (checksum != checksum_calc) {
            printf("WARNING: checksum failed (got:0x%02X calc:0x%02X)\n", checksum, checksum_calc);
            if (log_file != NULL) {
                fprintf(log_file, "\nWARNING: checksum failed (got:0x%02X calc:0x%02X)\n", checksum, checksum_calc);
                dbg_log_buffer_to_file(log_file, rx_fifo, sz);
            }
#if 0
            /* toggle GPIO for logic analyzer capture */
            dbg_toggle_gpio();
#endif
#if 1
            /* direct-memory access of the whole rx_buffer (assert) */
            sx1302_rx_buffer_dump(log_file, 0, 4095);
#endif
            return 0; /* drop all packets fetched in case of checksum error */
        } else {
            printf("Packet checksum OK (0x%02X)\n", checksum);
        }

        printf("-----------------\n");
        printf("  modem:      %u\n", SX1302_PKT_MODEM_ID(rx_fifo, buffer_index));
        printf("  chan:       %u\n", SX1302_PKT_CHANNEL(rx_fifo, buffer_index));
        printf("  size:       %u\n", SX1302_PKT_PAYLOAD_LENGTH(rx_fifo, buffer_index));
        printf("  crc_en:     %u\n", SX1302_PKT_CRC_EN(rx_fifo, buffer_index));
        printf("  crc_err:    %u\n", SX1302_PKT_CRC_ERROR(rx_fifo, buffer_index + payload_length));
        printf("  sync_err:   %u\n", SX1302_PKT_SYNC_ERROR(rx_fifo, buffer_index + payload_length));
        printf("  hdr_err:    %u\n", SX1302_PKT_HEADER_ERROR(rx_fifo, buffer_index + payload_length));
        printf("  timing_set: %u\n", SX1302_PKT_TIMING_SET(rx_fifo, buffer_index + payload_length));
        printf("  codr:       %u\n", SX1302_PKT_CODING_RATE(rx_fifo, buffer_index));
        printf("  datr:       %u\n", SX1302_PKT_DATARATE(rx_fifo, buffer_index));
        printf("  num_ts:     %u\n", SX1302_PKT_NUM_TS_METRICS(rx_fifo, buffer_index + payload_length));
        printf("-----------------\n");

        /* Sanity checks */
        sanity_check = SX1302_PKT_MODEM_ID(rx_fifo, buffer_index);
        if (sanity_check > SX1302_FSK_MODEM_ID) {
            printf("ERROR: modem_id is out of range - %u\n", sanity_check);
            rx_buffer_error = true;
        }
        if (sanity_check < SX1302_FSK_MODEM_ID) {
            sanity_check = SX1302_PKT_CHANNEL(rx_fifo, buffer_index);
            if (sanity_check > 9) {
                printf("ERROR: channel is out of range - %u\n", sanity_check);
                rx_buffer_error = true;
            }

            sanity_check = SX1302_PKT_DATARATE(rx_fifo, buffer_index);
            if ((sanity_check < 5) || (sanity_check > 12)) {
                printf("ERROR: SF is out of range - %u\n", sanity_check);
                rx_buffer_error = true;
            }
        }
        if (rx_buffer_error == true) {
            if (log_file != NULL) {
                fprintf(log_file, "ERROR: METADATA ERROR (%u)\n", nb_pkt_found);
                dbg_log_buffer_to_file(log_file, rx_fifo, sz);

            }
#if 1 /* TODO: TO BE REMOVED */
            sx1302_rx_buffer_dump(log_file, 0, 4095);
#endif
            return 0;
        }

        /* point to the proper struct in the struct array */
        p = &pkt_data[nb_pkt_found];

        /* we found the start of a packet, parse it */
        if ((nb_pkt_found + 1) > max_pkt) {
            printf("WARNING: no space left, dropping packet\n");
            nb_pkt_dropped += 1;
            continue;
        }
        nb_pkt_found += 1;

        /* copy payload to result struct */
        memcpy((void *)p->payload, (void *)(&rx_fifo[buffer_index + SX1302_PKT_HEAD_METADATA]), payload_length);

        /* process metadata */
        p->if_chain = SX1302_PKT_CHANNEL(rx_fifo, buffer_index);
        if (p->if_chain >= LGW_IF_CHAIN_NB) {
            DEBUG_PRINTF("WARNING: %u NOT A VALID IF_CHAIN NUMBER, ABORTING\n", p->if_chain);
            break;
        }
        ifmod = ifmod_config[p->if_chain];
        DEBUG_PRINTF("[%d %d]\n", p->if_chain, ifmod);

        p->size = payload_length;
        p->rf_chain = (uint8_t)CONTEXT_IF_CHAIN[p->if_chain].rf_chain;
        p->modem_id = SX1302_PKT_MODEM_ID(rx_fifo, buffer_index);
        p->freq_hz = (uint32_t)((int32_t)CONTEXT_RF_CHAIN[p->rf_chain].freq_hz + CONTEXT_IF_CHAIN[p->if_chain].freq_hz);
        p->rssic = (float)SX1302_PKT_RSSI_CHAN(rx_fifo, buffer_index + p->size) + CONTEXT_RF_CHAIN[p->rf_chain].rssi_offset;
        p->rssis = (float)SX1302_PKT_RSSI_SIG(rx_fifo, buffer_index + p->size) + CONTEXT_RF_CHAIN[p->rf_chain].rssi_offset;
        /* TODO: RSSI correction */

        /* Get CRC status */
        if ((ifmod == IF_LORA_MULTI) || (ifmod == IF_LORA_STD)) {
            DEBUG_PRINTF("Note: LoRa packet (modem %u chan %u)\n", SX1302_PKT_MODEM_ID(rx_fifo, buffer_index), p->if_chain);
            /* TODO: handle sync_err and hdr_err, to be reported when enabled (RX_BUFFER_STORE_SYNC_FAIL_META, RX_BUFFER_STORE_HEADER_ERR_META) */
            if (SX1302_PKT_CRC_EN(rx_fifo, buffer_index) || (CONTEXT_LORA_SERVICE.implicit_crc_en == true)) {
                /* CRC enabled */
                if (SX1302_PKT_CRC_ERROR(rx_fifo, buffer_index + p->size)) {
                    p->status = STAT_CRC_BAD;
                    crc_en = 1;
                } else {
                    p->status = STAT_CRC_OK;
                    crc_en = 1;

#if 1
                    /* FOR DEBUG:
                        We compare the received payload with predefined ones to ensure that the payload content is what we expect.
                        4 bytes: ID to identify the payload
                        4 bytes: packet counter used to initialize the seed for pseudo-random generation
                        x bytes: pseudo-random payload
                    */
                    for (j = 0; j < CONTEXT_DEBUG.nb_ref_payload; j++) {
                        res = dbg_check_payload(&CONTEXT_DEBUG, log_file, p->payload, p->size, j, SX1302_PKT_DATARATE(rx_fifo, buffer_index));
                        if (res == -1) {
                            printf("ERROR: 0x%08X payload error\n", CONTEXT_DEBUG.ref_payload[j].id);
                            if (log_file != NULL) {
                                fprintf(log_file, "ERROR: 0x%08X payload error (pkt:%u)\n", CONTEXT_DEBUG.ref_payload[j].id, nb_pkt_found - 1);
                                dbg_log_buffer_to_file(log_file, rx_fifo, sz);
                                dbg_log_payload_diff_to_file(log_file, p->payload, CONTEXT_DEBUG.ref_payload[j].payload, p->size);
                            }
                        } else if (res == 1) {
                            printf("0x%08X payload matches\n", CONTEXT_DEBUG.ref_payload[j].id);
                        } else {
                            /* Do nothing */
                        }
                    }
#endif

                    /* check payload CRC */
                    if (p->size > 0) {
                        payload_crc16_calc = sx1302_lora_payload_crc(p->payload, p->size);
                        payload_crc16_read  = (uint16_t)((SX1302_PKT_CRC_PAYLOAD_7_0(rx_fifo, buffer_index + p->size) <<  0) & 0x00FF);
                        payload_crc16_read |= (uint16_t)((SX1302_PKT_CRC_PAYLOAD_15_8(rx_fifo, buffer_index + p->size) <<  8) & 0xFF00);
                        if (payload_crc16_calc != payload_crc16_read) {
                            printf("ERROR: Payload CRC16 check failed (got:0x%04X calc:0x%04X)\n", payload_crc16_read, payload_crc16_calc);
                            if (log_file != NULL) {
                                fprintf(log_file, "ERROR: Payload CRC16 check failed (got:0x%04X calc:0x%04X) (pkt:%u)\n", payload_crc16_read, payload_crc16_calc, nb_pkt_found-1);
                                dbg_log_buffer_to_file(log_file, rx_fifo, sz);
                            }
                        } else {
                            printf("Payload CRC check OK (0x%04X)\n", payload_crc16_read);
                        }
                    }
                }
            } else {
                /* CRC disabled */
                p->status = STAT_NO_CRC;
                crc_en = 0;
            }

            p->modulation = MOD_LORA;
            p->snr = (float)((int8_t)SX1302_PKT_SNR_AVG(rx_fifo, buffer_index + p->size)) / 4;

            /* Get bandwidth */
            if (ifmod == IF_LORA_MULTI) {
                p->bandwidth = BW_125KHZ; /* fixed in hardware */
            } else {
                p->bandwidth = CONTEXT_LORA_SERVICE.bandwidth; /* get the parameter from the config variable */
            }

            /* Get datarate */
            sf = SX1302_PKT_DATARATE(rx_fifo, buffer_index);
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

            /* Get coding rate */
            if ((ifmod == IF_LORA_MULTI) || (CONTEXT_LORA_SERVICE.implicit_hdr == false)) {
                cr = SX1302_PKT_CODING_RATE(rx_fifo, buffer_index);
            } else {
                cr = CONTEXT_LORA_SERVICE.implicit_coderate;
            }
            switch (cr) {
                case 1: p->coderate = CR_LORA_4_5; break;
                case 2: p->coderate = CR_LORA_4_6; break;
                case 3: p->coderate = CR_LORA_4_7; break;
                case 4: p->coderate = CR_LORA_4_8; break;
                default: p->coderate = CR_UNDEFINED;
            }

            /* Get raw frequency offset as reported */
            freq_offset_raw = (int32_t)((SX1302_PKT_FREQ_OFFSET_19_16(rx_fifo, buffer_index) << 16) | (SX1302_PKT_FREQ_OFFSET_15_8(rx_fifo, buffer_index) << 8) | (SX1302_PKT_FREQ_OFFSET_7_0(rx_fifo, buffer_index) << 0));
            /* Handle signed value on 20bits */
            if (freq_offset_raw >= (1<<19)) {
                freq_offset_raw = (freq_offset_raw - (1<<20));
            }
            /* Get frequency offset in Hz depending on bandwidth */
            switch (p->bandwidth) {
                case BW_125KHZ:
                    freq_offset = (int32_t)((float)freq_offset_raw * FREQ_OFFSET_LSB_125KHZ );
                    break;
                case BW_250KHZ:
                    freq_offset = (int32_t)((float)freq_offset_raw * FREQ_OFFSET_LSB_250KHZ );
                    break;
                case BW_500KHZ:
                    freq_offset = (int32_t)((float)freq_offset_raw * FREQ_OFFSET_LSB_500KHZ );
                    break;
                default:
                    freq_offset = 0;
                    printf("Invalid frequency offset\n");
                    break;
            }
            printf("  f_offset: %d Hz\n", freq_offset);
            p->freq_offset = freq_offset;

            /* determine if 'PPM mode' is on, needed for timestamp correction */
            if (SET_PPM_ON(p->bandwidth, p->datarate)) {
                ppm = 1;
            } else {
                ppm = 0;
            }

            /* timestamp correction code, base delay */
            if (ifmod == IF_LORA_STD) { /* if packet was received on the stand-alone LoRa modem */
                switch (CONTEXT_LORA_SERVICE.bandwidth) {
                    case BW_125KHZ:
                        delay_x = 64;
                        bw_pow = 1;
                        break;
                    case BW_250KHZ:
                        delay_x = 32;
                        bw_pow = 2;
                        break;
                    case BW_500KHZ:
                        delay_x = 16;
                        bw_pow = 4;
                        break;
                    default:
                        DEBUG_PRINTF("ERROR: UNEXPECTED VALUE %d IN SWITCH STATEMENT\n", p->bandwidth);
                        delay_x = 0;
                        bw_pow = 0;
                }
            } else { /* packet was received on one of the sensor channels = 125kHz */
                delay_x = 64;
                bw_pow = 1;
            }

            /* timestamp correction code, variable delay */
            if ((sf >= 6) && (sf <= 12) && (bw_pow > 0)) {
                if ((2*(sz + 2*crc_en) - (sf-7)) <= 0) { /* payload fits entirely in first 8 symbols */
                    delay_y = ( ((1<<(sf-1)) * (sf+1)) + (3 * (1<<(sf-4))) ) / bw_pow;
                    delay_z = 32 * (2*(sz+2*crc_en) + 5) / bw_pow;
                } else {
                    delay_y = ( ((1<<(sf-1)) * (sf+1)) + ((4 - ppm) * (1<<(sf-4))) ) / bw_pow;
                    delay_z = (16 + 4*cr) * (((2*(sz+2*crc_en)-sf+6) % (sf - 2*ppm)) + 1) / bw_pow;
                }
                timestamp_correction = delay_x + delay_y + delay_z;
            } else if (sf == 5) {
                timestamp_correction = 0;
                DEBUG_MSG("WARNING: TODO: timestamp correction for SF5\n");
            } else {
                timestamp_correction = 0;
                DEBUG_MSG("WARNING: invalid packet, no timestamp correction\n");
            }

            /* RSSI correction */
            /* TODO ? */
        } else if (ifmod == IF_FSK_STD) {
            DEBUG_PRINTF("Note: FSK packet (modem %u chan %u)\n", SX1302_PKT_MODEM_ID(rx_fifo, buffer_index), p->if_chain);
            if (SX1302_PKT_CRC_EN(rx_fifo, buffer_index)) {
                /* CRC enabled */
                if (SX1302_PKT_CRC_ERROR(rx_fifo, buffer_index + p->size)) {
                    printf("FSK: CRC ERR\n");
                    p->status = STAT_CRC_BAD;
                } else {
                    printf("FSK: CRC OK\n");
                    p->status = STAT_CRC_OK;
                }
            } else {
                /* CRC disabled */
                p->status = STAT_NO_CRC;
            }
            p->modulation = MOD_FSK;
            p->snr = -128.0;
            p->bandwidth = CONTEXT_FSK.bandwidth;
            p->datarate = CONTEXT_FSK.datarate;
            p->coderate = CR_UNDEFINED;
            timestamp_correction = ((uint32_t)680000 / CONTEXT_FSK.datarate) - 20;

            /* RSSI correction */
            p->rssic = RSSI_FSK_POLY_0 + RSSI_FSK_POLY_1 * p->rssic + RSSI_FSK_POLY_2 * pow(p->rssic, 2);
            p->rssis = -128.0;
        } else {
            DEBUG_MSG("ERROR: UNEXPECTED PACKET ORIGIN\n");
            p->status = STAT_UNDEFINED;
            p->modulation = MOD_UNDEFINED;
            p->rssic = -128.0;
            p->rssis = -128.0;
            p->snr = -128.0;
            p->snr_min = -128.0;
            p->snr_max = -128.0;
            p->bandwidth = BW_UNDEFINED;
            p->datarate = DR_UNDEFINED;
            p->coderate = CR_UNDEFINED;
            timestamp_correction = 0;
        }

        /* Packet timestamp (32MHz ) */
        p->count_us  = (uint32_t)((SX1302_PKT_TIMESTAMP_7_0(rx_fifo, buffer_index + p->size) <<  0) & 0x000000FF);
        p->count_us |= (uint32_t)((SX1302_PKT_TIMESTAMP_15_8(rx_fifo, buffer_index + p->size) <<  8) & 0x0000FF00);
        p->count_us |= (uint32_t)((SX1302_PKT_TIMESTAMP_23_16(rx_fifo, buffer_index + p->size) << 16) & 0x00FF0000);
        p->count_us |= (uint32_t)((SX1302_PKT_TIMESTAMP_31_24(rx_fifo, buffer_index + p->size) << 24) & 0xFF000000);
        /* Scale packet timestamp to 1 MHz (microseconds) */
        p->count_us /= 32;
        /* Expand 27-bits counter to 32-bits counter, based on current wrapping status */
        sx1302_timestamp_expand(false, &(p->count_us));
        /* Apply timestamp correction */
        p->count_us -= timestamp_correction;

        /* Packet CRC status */
        p->crc = (uint16_t)(SX1302_PKT_CRC_PAYLOAD_7_0(rx_fifo, buffer_index + p->size)) + ((uint16_t)(SX1302_PKT_CRC_PAYLOAD_15_8(rx_fifo, buffer_index + p->size)) << 8);

        /* move buffer index toward next message */
        buffer_index += (SX1302_PKT_HEAD_METADATA + payload_length + SX1302_PKT_TAIL_METADATA + (2 * num_ts_metrics));
    }

    printf("INFO: nb pkt found:%u dropped:%u\n", nb_pkt_found, nb_pkt_dropped);

    return nb_pkt_found;
}

/* ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ */

int lgw_send(struct lgw_pkt_tx_s pkt_data) {
    uint32_t freq_reg, fdev_reg;
    uint32_t freq_dev;
    uint32_t fsk_br_reg;
    uint64_t fsk_sync_word_reg;
    uint16_t tx_start_delay;
    uint16_t reg;
    uint16_t mem_addr;
    uint32_t count_us;
    uint8_t power;
    uint8_t pow_index;
    uint8_t mod_bw;

    /* check if the concentrator is running */
    if (CONTEXT_STARTED == false) {
        DEBUG_MSG("ERROR: CONCENTRATOR IS NOT RUNNING, START IT BEFORE SENDING\n");
        return LGW_HAL_ERROR;
    }

    /* check input range (segfault prevention) */
    if (pkt_data.rf_chain >= LGW_RF_CHAIN_NB) {
        DEBUG_MSG("ERROR: INVALID RF_CHAIN TO SEND PACKETS\n");
        return LGW_HAL_ERROR;
    }

    /* check input variables */
    if (CONTEXT_RF_CHAIN[pkt_data.rf_chain].tx_enable == false) {
        DEBUG_MSG("ERROR: SELECTED RF_CHAIN IS DISABLED FOR TX ON SELECTED BOARD\n");
        return LGW_HAL_ERROR;
    }
    if (CONTEXT_RF_CHAIN[pkt_data.rf_chain].enable == false) {
        DEBUG_MSG("ERROR: SELECTED RF_CHAIN IS DISABLED\n");
        return LGW_HAL_ERROR;
    }
    if (!IS_TX_MODE(pkt_data.tx_mode)) {
        DEBUG_MSG("ERROR: TX_MODE NOT SUPPORTED\n");
        return LGW_HAL_ERROR;
    }
    if (pkt_data.modulation == MOD_LORA) {
        if (!IS_LORA_BW(pkt_data.bandwidth)) {
            DEBUG_MSG("ERROR: BANDWIDTH NOT SUPPORTED BY LORA TX\n");
            return LGW_HAL_ERROR;
        }
        if (!IS_LORA_DR(pkt_data.datarate)) {
            DEBUG_MSG("ERROR: DATARATE NOT SUPPORTED BY LORA TX\n");
            return LGW_HAL_ERROR;
        }
        if (!IS_LORA_CR(pkt_data.coderate)) {
            DEBUG_MSG("ERROR: CODERATE NOT SUPPORTED BY LORA TX\n");
            return LGW_HAL_ERROR;
        }
        if (pkt_data.size > 255) {
            DEBUG_MSG("ERROR: PAYLOAD LENGTH TOO BIG FOR LORA TX\n");
            return LGW_HAL_ERROR;
        }
    } else if (pkt_data.modulation == MOD_FSK) {
        if((pkt_data.f_dev < 1) || (pkt_data.f_dev > 200)) {
            DEBUG_MSG("ERROR: TX FREQUENCY DEVIATION OUT OF ACCEPTABLE RANGE\n");
            return LGW_HAL_ERROR;
        }
        if(!IS_FSK_DR(pkt_data.datarate)) {
            DEBUG_MSG("ERROR: DATARATE NOT SUPPORTED BY FSK IF CHAIN\n");
            return LGW_HAL_ERROR;
        }
        if (pkt_data.size > 255) {
            DEBUG_MSG("ERROR: PAYLOAD LENGTH TOO BIG FOR FSK TX\n");
            return LGW_HAL_ERROR;
        }
    } else {
        DEBUG_MSG("ERROR: INVALID TX MODULATION\n");
        return LGW_HAL_ERROR;
    }

    /* Let AGC control PLL DIV (sx1250 only) */
    lgw_reg_w(SX1302_REG_TX_TOP_TX_RFFE_IF_CTRL2_PLL_DIV_CTRL_AGC(pkt_data.rf_chain), 1);

    /* Set radio type */
    reg = SX1302_REG_TX_TOP_TX_RFFE_IF_CTRL_TX_IF_DST(pkt_data.rf_chain);
    switch (CONTEXT_RF_CHAIN[pkt_data.rf_chain].type) {
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

    lgw_reg_w(SX1302_REG_TX_TOP_TX_RFFE_IF_CTRL_TX_MODE(pkt_data.rf_chain), 0x01); /* Modulation */
    lgw_reg_w(SX1302_REG_TX_TOP_TX_RFFE_IF_CTRL_TX_CLK_EDGE(pkt_data.rf_chain), 0x00); /* Data on rising edge */

    switch (pkt_data.modulation) {
        case MOD_LORA:
            lgw_reg_w(SX1302_REG_TX_TOP_GEN_CFG_0_MODULATION_TYPE(pkt_data.rf_chain), 0x00);
            lgw_reg_w(SX1302_REG_TX_TOP_TX_RFFE_IF_CTRL_TX_IF_SRC(pkt_data.rf_chain), 0x01);
            break;
        case MOD_FSK:
            lgw_reg_w(SX1302_REG_TX_TOP_GEN_CFG_0_MODULATION_TYPE(pkt_data.rf_chain), 0x01);
            lgw_reg_w(SX1302_REG_TX_TOP_TX_RFFE_IF_CTRL_TX_IF_SRC(pkt_data.rf_chain), 0x02);
            break;
        default:
            DEBUG_MSG("ERROR: modulation type not supported\n");
            return LGW_HAL_ERROR;
    }

    /* Find the proper index in the TX gain LUT according to requested rf_power */
    for (pow_index = CONTEXT_TX_GAIN_LUT[pkt_data.rf_chain].size-1; pow_index > 0; pow_index--) {
        if (CONTEXT_TX_GAIN_LUT[pkt_data.rf_chain].lut[pow_index].rf_power <= pkt_data.rf_power) {
            break;
        }
    }
    printf("INFO: selecting TX Gain LUT index %u\n", pow_index);

    /* loading calibrated Tx DC offsets */
    lgw_reg_w(SX1302_REG_TX_TOP_TX_RFFE_IF_I_OFFSET_I_OFFSET(pkt_data.rf_chain), CONTEXT_TX_GAIN_LUT[pkt_data.rf_chain].lut[pow_index].offset_i);
    lgw_reg_w(SX1302_REG_TX_TOP_TX_RFFE_IF_Q_OFFSET_Q_OFFSET(pkt_data.rf_chain), CONTEXT_TX_GAIN_LUT[pkt_data.rf_chain].lut[pow_index].offset_q);

    printf("INFO: Applying IQ offset (i:%d, q:%d)\n", CONTEXT_TX_GAIN_LUT[pkt_data.rf_chain].lut[pow_index].offset_i, CONTEXT_TX_GAIN_LUT[pkt_data.rf_chain].lut[pow_index].offset_q);

    /* Set the power parameters to be used for TX */
    switch (CONTEXT_RF_CHAIN[pkt_data.rf_chain].type) {
        case LGW_RADIO_TYPE_SX1250:
            power = (CONTEXT_TX_GAIN_LUT[pkt_data.rf_chain].lut[pow_index].pa_gain << 6) | CONTEXT_TX_GAIN_LUT[pkt_data.rf_chain].lut[pow_index].pwr_idx;
            break;
        case LGW_RADIO_TYPE_SX1257:
            power = (CONTEXT_TX_GAIN_LUT[pkt_data.rf_chain].lut[pow_index].pa_gain << 6) | (CONTEXT_TX_GAIN_LUT[pkt_data.rf_chain].lut[pow_index].dac_gain << 4) | CONTEXT_TX_GAIN_LUT[pkt_data.rf_chain].lut[pow_index].mix_gain;
            break;
        default:
            DEBUG_MSG("ERROR: radio type not supported\n");
            return LGW_HAL_ERROR;
    }
    lgw_reg_w(SX1302_REG_TX_TOP_AGC_TX_PWR_AGC_TX_PWR(pkt_data.rf_chain), power);

    /* Set digital gain */
    lgw_reg_w(SX1302_REG_TX_TOP_TX_RFFE_IF_IQ_GAIN_IQ_GAIN(pkt_data.rf_chain), CONTEXT_TX_GAIN_LUT[pkt_data.rf_chain].lut[pow_index].dig_gain);

    /* Set Tx frequency */
    freq_reg = SX1302_FREQ_TO_REG(pkt_data.freq_hz); /* TODO: AGC fw to be updated for sx1255 */
    lgw_reg_w(SX1302_REG_TX_TOP_TX_RFFE_IF_FREQ_RF_H_FREQ_RF(pkt_data.rf_chain), (freq_reg >> 16) & 0xFF);
    lgw_reg_w(SX1302_REG_TX_TOP_TX_RFFE_IF_FREQ_RF_M_FREQ_RF(pkt_data.rf_chain), (freq_reg >> 8) & 0xFF);
    lgw_reg_w(SX1302_REG_TX_TOP_TX_RFFE_IF_FREQ_RF_L_FREQ_RF(pkt_data.rf_chain), (freq_reg >> 0) & 0xFF);

    /* Set AGC bandwidth and modulation type*/
    switch (pkt_data.modulation) {
        case MOD_LORA:
            mod_bw = pkt_data.bandwidth;
            break;
        case MOD_FSK:
            mod_bw = (0x01 << 7) | pkt_data.bandwidth;
            break;
        default:
            printf("ERROR: Modulation not supported\n");
            return LGW_HAL_ERROR;
    }
    lgw_reg_w(SX1302_REG_TX_TOP_AGC_TX_BW_AGC_TX_BW(pkt_data.rf_chain), mod_bw);

    /* Configure modem */
    switch (pkt_data.modulation) {
        case MOD_LORA:
            /* Set bandwidth */
            printf("Bandwidth %dkHz\n", (int)(lgw_bw_getval(pkt_data.bandwidth) / 1E3));
            freq_dev = lgw_bw_getval(pkt_data.bandwidth) / 2;
            fdev_reg = SX1302_FREQ_TO_REG(freq_dev);
            lgw_reg_w(SX1302_REG_TX_TOP_TX_RFFE_IF_FREQ_DEV_H_FREQ_DEV(pkt_data.rf_chain), (fdev_reg >>  8) & 0xFF);
            lgw_reg_w(SX1302_REG_TX_TOP_TX_RFFE_IF_FREQ_DEV_L_FREQ_DEV(pkt_data.rf_chain), (fdev_reg >>  0) & 0xFF);
            lgw_reg_w(SX1302_REG_TX_TOP_TXRX_CFG0_0_MODEM_BW(pkt_data.rf_chain), pkt_data.bandwidth);

            /* Preamble length */
            if (pkt_data.preamble == 0) { /* if not explicit, use recommended LoRa preamble size */
                pkt_data.preamble = STD_LORA_PREAMBLE;
            } else if (pkt_data.preamble < MIN_LORA_PREAMBLE) { /* enforce minimum preamble size */
                pkt_data.preamble = MIN_LORA_PREAMBLE;
                DEBUG_MSG("Note: preamble length adjusted to respect minimum LoRa preamble size\n");
            }
            lgw_reg_w(SX1302_REG_TX_TOP_TXRX_CFG1_3_PREAMBLE_SYMB_NB(pkt_data.rf_chain), (pkt_data.preamble >> 8) & 0xFF); /* MSB */
            lgw_reg_w(SX1302_REG_TX_TOP_TXRX_CFG1_2_PREAMBLE_SYMB_NB(pkt_data.rf_chain), (pkt_data.preamble >> 0) & 0xFF); /* LSB */

            /* LoRa datarate */
            lgw_reg_w(SX1302_REG_TX_TOP_TXRX_CFG0_0_MODEM_SF(pkt_data.rf_chain), pkt_data.datarate);
            if (pkt_data.datarate < 10) {
                lgw_reg_w(SX1302_REG_TX_TOP_TX_CFG0_0_CHIRP_LOWPASS(pkt_data.rf_chain), 6); /* less filtering for low SF : TBC */
            } else {
                lgw_reg_w(SX1302_REG_TX_TOP_TX_CFG0_0_CHIRP_LOWPASS(pkt_data.rf_chain), 7);
            }

            /* Coding Rate */
            lgw_reg_w(SX1302_REG_TX_TOP_TXRX_CFG0_1_CODING_RATE(pkt_data.rf_chain), pkt_data.coderate);

            /* Start LoRa modem */
            lgw_reg_w(SX1302_REG_TX_TOP_TXRX_CFG0_2_MODEM_EN(pkt_data.rf_chain), 1);
            lgw_reg_w(SX1302_REG_TX_TOP_TXRX_CFG0_2_CADRXTX(pkt_data.rf_chain), 2);
            lgw_reg_w(SX1302_REG_TX_TOP_TXRX_CFG1_1_MODEM_START(pkt_data.rf_chain), 1);
            lgw_reg_w(SX1302_REG_TX_TOP_TX_CFG0_0_CONTINUOUS(pkt_data.rf_chain), 0);

            /* Modulation options */
            lgw_reg_w(SX1302_REG_TX_TOP_TX_CFG0_0_CHIRP_INVERT(pkt_data.rf_chain), (pkt_data.invert_pol) ? 1 : 0);
            lgw_reg_w(SX1302_REG_TX_TOP_TXRX_CFG0_2_IMPLICIT_HEADER(pkt_data.rf_chain), (pkt_data.no_header) ? 1 : 0);
            lgw_reg_w(SX1302_REG_TX_TOP_TXRX_CFG0_2_CRC_EN(pkt_data.rf_chain), (pkt_data.no_crc) ? 0 : 1);

            /* Syncword */
            if ((CONTEXT_LWAN_PUBLIC == false) || (pkt_data.datarate == DR_LORA_SF5) || (pkt_data.datarate == DR_LORA_SF6)) {
                printf("Setting LoRa syncword 0x12\n");
                lgw_reg_w(SX1302_REG_TX_TOP_FRAME_SYNCH_0_PEAK1_POS(pkt_data.rf_chain), 2);
                lgw_reg_w(SX1302_REG_TX_TOP_FRAME_SYNCH_1_PEAK2_POS(pkt_data.rf_chain), 4);
            } else {
                printf("Setting LoRa syncword 0x34\n");
                lgw_reg_w(SX1302_REG_TX_TOP_FRAME_SYNCH_0_PEAK1_POS(pkt_data.rf_chain), 6);
                lgw_reg_w(SX1302_REG_TX_TOP_FRAME_SYNCH_1_PEAK2_POS(pkt_data.rf_chain), 8);
            }

            /* Set Fine Sync for SF5/SF6 */
            if ((pkt_data.datarate == DR_LORA_SF5) || (pkt_data.datarate == DR_LORA_SF6)) {
                printf("Enable Fine Sync\n");
                lgw_reg_w(SX1302_REG_TX_TOP_TXRX_CFG0_2_FINE_SYNCH_EN(pkt_data.rf_chain), 1);
            } else {
                printf("Disable Fine Sync\n");
                lgw_reg_w(SX1302_REG_TX_TOP_TXRX_CFG0_2_FINE_SYNCH_EN(pkt_data.rf_chain), 0);
            }

            /* Set Payload length */
            lgw_reg_w(SX1302_REG_TX_TOP_TXRX_CFG0_3_PAYLOAD_LENGTH(pkt_data.rf_chain), pkt_data.size);

            /* Set PPM offset (low datarate optimization) */
            lgw_reg_w(SX1302_REG_TX_TOP_TXRX_CFG0_1_PPM_OFFSET_HDR_CTRL(pkt_data.rf_chain), 0);
            if (SET_PPM_ON(pkt_data.bandwidth, pkt_data.datarate)) {
                printf("Low datarate optimization ENABLED\n");
                lgw_reg_w(SX1302_REG_TX_TOP_TXRX_CFG0_1_PPM_OFFSET(pkt_data.rf_chain), 1);
            } else {
                printf("Low datarate optimization DISABLED\n");
                lgw_reg_w(SX1302_REG_TX_TOP_TXRX_CFG0_1_PPM_OFFSET(pkt_data.rf_chain), 0);
            }
            break;
        case MOD_FSK:
            /* Set frequency deviation */
            printf("f_dev %dkHz\n", (int)(pkt_data.f_dev));
            freq_dev = pkt_data.f_dev * 1e3;
            fdev_reg = SX1302_FREQ_TO_REG(freq_dev);
            lgw_reg_w(SX1302_REG_TX_TOP_TX_RFFE_IF_FREQ_DEV_H_FREQ_DEV(pkt_data.rf_chain), (fdev_reg >>  8) & 0xFF);
            lgw_reg_w(SX1302_REG_TX_TOP_TX_RFFE_IF_FREQ_DEV_L_FREQ_DEV(pkt_data.rf_chain), (fdev_reg >>  0) & 0xFF);

            /* Send frequency deviation to AGC fw for radio config */
            fdev_reg = SX1250_FREQ_TO_REG(freq_dev);
            lgw_reg_w(SX1302_REG_AGC_MCU_MCU_MAIL_BOX_WR_DATA_BYTE2_MCU_MAIL_BOX_WR_DATA, (fdev_reg >> 16) & 0xFF); /* Needed by AGC to configure the sx1250 */
            lgw_reg_w(SX1302_REG_AGC_MCU_MCU_MAIL_BOX_WR_DATA_BYTE1_MCU_MAIL_BOX_WR_DATA, (fdev_reg >>  8) & 0xFF); /* Needed by AGC to configure the sx1250 */
            lgw_reg_w(SX1302_REG_AGC_MCU_MCU_MAIL_BOX_WR_DATA_BYTE0_MCU_MAIL_BOX_WR_DATA, (fdev_reg >>  0) & 0xFF); /* Needed by AGC to configure the sx1250 */

            /* Modulation parameters */
            lgw_reg_w(SX1302_REG_TX_TOP_FSK_CFG_0_PKT_MODE(pkt_data.rf_chain), 1); /* Variable length */
            lgw_reg_w(SX1302_REG_TX_TOP_FSK_CFG_0_CRC_EN(pkt_data.rf_chain), (pkt_data.no_crc) ? 0 : 1);
            lgw_reg_w(SX1302_REG_TX_TOP_FSK_CFG_0_CRC_IBM(pkt_data.rf_chain), 0); /* CCITT CRC */
            lgw_reg_w(SX1302_REG_TX_TOP_FSK_CFG_0_DCFREE_ENC(pkt_data.rf_chain), 2); /* Whitening Encoding */
            lgw_reg_w(SX1302_REG_TX_TOP_FSK_MOD_FSK_GAUSSIAN_EN(pkt_data.rf_chain), 1);
            lgw_reg_w(SX1302_REG_TX_TOP_FSK_MOD_FSK_GAUSSIAN_SELECT_BT(pkt_data.rf_chain), 2);
            lgw_reg_w(SX1302_REG_TX_TOP_FSK_MOD_FSK_REF_PATTERN_EN(pkt_data.rf_chain), 1);
            lgw_reg_w(SX1302_REG_TX_TOP_FSK_MOD_FSK_REF_PATTERN_SIZE(pkt_data.rf_chain), CONTEXT_FSK.sync_word_size - 1);

            /* Syncword */
            fsk_sync_word_reg = CONTEXT_FSK.sync_word << (8 * (8 - CONTEXT_FSK.sync_word_size));
            lgw_reg_w(SX1302_REG_TX_TOP_FSK_REF_PATTERN_BYTE0_FSK_REF_PATTERN(pkt_data.rf_chain), (uint8_t)(fsk_sync_word_reg >> 0));
            lgw_reg_w(SX1302_REG_TX_TOP_FSK_REF_PATTERN_BYTE1_FSK_REF_PATTERN(pkt_data.rf_chain), (uint8_t)(fsk_sync_word_reg >> 8));
            lgw_reg_w(SX1302_REG_TX_TOP_FSK_REF_PATTERN_BYTE2_FSK_REF_PATTERN(pkt_data.rf_chain), (uint8_t)(fsk_sync_word_reg >> 16));
            lgw_reg_w(SX1302_REG_TX_TOP_FSK_REF_PATTERN_BYTE3_FSK_REF_PATTERN(pkt_data.rf_chain), (uint8_t)(fsk_sync_word_reg >> 24));
            lgw_reg_w(SX1302_REG_TX_TOP_FSK_REF_PATTERN_BYTE4_FSK_REF_PATTERN(pkt_data.rf_chain), (uint8_t)(fsk_sync_word_reg >> 32));
            lgw_reg_w(SX1302_REG_TX_TOP_FSK_REF_PATTERN_BYTE5_FSK_REF_PATTERN(pkt_data.rf_chain), (uint8_t)(fsk_sync_word_reg >> 40));
            lgw_reg_w(SX1302_REG_TX_TOP_FSK_REF_PATTERN_BYTE6_FSK_REF_PATTERN(pkt_data.rf_chain), (uint8_t)(fsk_sync_word_reg >> 48));
            lgw_reg_w(SX1302_REG_TX_TOP_FSK_REF_PATTERN_BYTE7_FSK_REF_PATTERN(pkt_data.rf_chain), (uint8_t)(fsk_sync_word_reg >> 56));
            lgw_reg_w(SX1302_REG_TX_TOP_FSK_MOD_FSK_PREAMBLE_SEQ(pkt_data.rf_chain), 0);

            /* Set datarate */
            fsk_br_reg = 32000000 / pkt_data.datarate;
            lgw_reg_w(SX1302_REG_TX_TOP_FSK_BIT_RATE_MSB_BIT_RATE(pkt_data.rf_chain), fsk_br_reg >> 8);
            lgw_reg_w(SX1302_REG_TX_TOP_FSK_BIT_RATE_LSB_BIT_RATE(pkt_data.rf_chain), fsk_br_reg >> 0);

            /* Preamble length */
            if (pkt_data.preamble == 0) { /* if not explicit, use LoRa MAC preamble size */
                pkt_data.preamble = STD_FSK_PREAMBLE;
            } else if (pkt_data.preamble < MIN_FSK_PREAMBLE) { /* enforce minimum preamble size */
                pkt_data.preamble = MIN_FSK_PREAMBLE;
                DEBUG_MSG("Note: preamble length adjusted to respect minimum FSK preamble size\n");
            }
            lgw_reg_w(SX1302_REG_TX_TOP_FSK_PREAMBLE_SIZE_MSB_PREAMBLE_SIZE(pkt_data.rf_chain), (pkt_data.preamble >> 8) & 0xFF); /* MSB */
            lgw_reg_w(SX1302_REG_TX_TOP_FSK_PREAMBLE_SIZE_LSB_PREAMBLE_SIZE(pkt_data.rf_chain), (pkt_data.preamble >> 0) & 0xFF); /* LSB */

            /* Set Payload length */
            lgw_reg_w(SX1302_REG_TX_TOP_FSK_PKT_LEN_PKT_LENGTH(pkt_data.rf_chain), pkt_data.size);
            break;
        default:
            printf("ERROR: Modulation not supported\n");
            return LGW_HAL_ERROR;
    }

    /* Set TX start delay */
    tx_start_delay = lgw_get_tx_start_delay(CONTEXT_RF_CHAIN[pkt_data.rf_chain].type, pkt_data.modulation, pkt_data.bandwidth);
    lgw_reg_w(SX1302_REG_TX_TOP_TX_START_DELAY_MSB_TX_START_DELAY(pkt_data.rf_chain), (uint8_t)(tx_start_delay >> 8));
    lgw_reg_w(SX1302_REG_TX_TOP_TX_START_DELAY_LSB_TX_START_DELAY(pkt_data.rf_chain), (uint8_t)(tx_start_delay >> 0));

    /* Write payload in transmit buffer */
    lgw_reg_w(SX1302_REG_TX_TOP_TX_CTRL_WRITE_BUFFER(pkt_data.rf_chain), 0x01);
    mem_addr = REG_SELECT(pkt_data.rf_chain, 0x5300, 0x5500);
    if (pkt_data.modulation == MOD_FSK) {
        lgw_mem_wb(mem_addr, (uint8_t *)(&(pkt_data.size)), 1); /* insert payload size in the packet for FSK variable mode (1 byte) */
        lgw_mem_wb(mem_addr+1, &(pkt_data.payload[0]), pkt_data.size);
    } else {
        lgw_mem_wb(mem_addr, &(pkt_data.payload[0]), pkt_data.size);
    }
    lgw_reg_w(SX1302_REG_TX_TOP_TX_CTRL_WRITE_BUFFER(pkt_data.rf_chain), 0x00);

    /* Trigger transmit */
    printf("Start Tx: Freq:%u %s%u size:%u preamb:%u\n", pkt_data.freq_hz, (pkt_data.modulation == MOD_LORA) ? "SF" : "DR:", pkt_data.datarate, pkt_data.size, pkt_data.preamble);
    switch (pkt_data.tx_mode) {
        case IMMEDIATE:
            lgw_reg_w(SX1302_REG_TX_TOP_TX_TRIG_TX_TRIG_IMMEDIATE(pkt_data.rf_chain), 0x00); /* reset state machine */
            lgw_reg_w(SX1302_REG_TX_TOP_TX_TRIG_TX_TRIG_IMMEDIATE(pkt_data.rf_chain), 0x01);
            break;
        case TIMESTAMPED:
            count_us = pkt_data.count_us * 32;
            printf("--> programming trig delay at %u (%u)\n", pkt_data.count_us, count_us);

            lgw_reg_w(SX1302_REG_TX_TOP_TIMER_TRIG_BYTE0_TIMER_DELAYED_TRIG(pkt_data.rf_chain), (uint8_t)((count_us >>  0) & 0x000000FF));
            lgw_reg_w(SX1302_REG_TX_TOP_TIMER_TRIG_BYTE1_TIMER_DELAYED_TRIG(pkt_data.rf_chain), (uint8_t)((count_us >>  8) & 0x000000FF));
            lgw_reg_w(SX1302_REG_TX_TOP_TIMER_TRIG_BYTE2_TIMER_DELAYED_TRIG(pkt_data.rf_chain), (uint8_t)((count_us >> 16) & 0x000000FF));
            lgw_reg_w(SX1302_REG_TX_TOP_TIMER_TRIG_BYTE3_TIMER_DELAYED_TRIG(pkt_data.rf_chain), (uint8_t)((count_us >> 24) & 0x000000FF));

            lgw_reg_w(SX1302_REG_TX_TOP_TX_TRIG_TX_TRIG_DELAYED(pkt_data.rf_chain), 0x00); /* reset state machine */
            lgw_reg_w(SX1302_REG_TX_TOP_TX_TRIG_TX_TRIG_DELAYED(pkt_data.rf_chain), 0x01);

#if 0
            wait_ms(1000);

            lgw_reg_w(SX1302_REG_COMMON_CTRL0_HOST_RADIO_CTRL, 0x01);

            //sx1250_setup(0, 865800000);

            buff2[0] = 0x08;
            buff2[1] = 0x8B;
            buff2[2] = 0x00;
            buff2[3] = 0x00;
            buff2[4] = 0x00;
            buff2[5] = 0x00;
            buff2[6] = 0x00;
            sx1250_read_command(0, READ_REGISTER, buff2, 7);
            printf("reading %u\n", buff2[3]);
            printf("reading %u\n", buff2[4]);
            printf("reading %u\n", buff2[5]);
            printf("reading %u\n", buff2[6]);

            lgw_reg_w(SX1302_REG_COMMON_CTRL0_HOST_RADIO_CTRL, 0x00);
#endif

            break;
        case ON_GPS:
            lgw_reg_w(SX1302_REG_TX_TOP_TX_TRIG_TX_TRIG_GPS(pkt_data.rf_chain), 0x00); /* reset state machine */
            lgw_reg_w(SX1302_REG_TX_TOP_TX_TRIG_TX_TRIG_GPS(pkt_data.rf_chain), 0x01);
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

    /* check input variables */
    CHECK_NULL(code);
    if (rf_chain >= LGW_RF_CHAIN_NB) {
        DEBUG_MSG("ERROR: NOT A VALID RF_CHAIN NUMBER\n");
        return LGW_HAL_ERROR;
    }

    if (select == TX_STATUS) {
        lgw_reg_r(SX1302_REG_TX_TOP_TX_FSM_STATUS_TX_STATUS(rf_chain), &read_value);
        // TODO: select porper TX rf chain
        if (CONTEXT_STARTED == false) {
            *code = TX_OFF;
        } else if (read_value == 0x80) {
            *code = TX_FREE;
        } else if ((read_value == 0x30) || (read_value == 0x50) || (read_value == 0x60) || (read_value == 0x70)) {
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

    //DEBUG_PRINTF("INFO: STATUS %u\n", *code);
    return LGW_HAL_SUCCESS;
}

/* ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ */

int lgw_abort_tx(uint8_t rf_chain) {
    uint8_t tx_status;

    lgw_reg_w(SX1302_REG_TX_TOP_TX_TRIG_TX_TRIG_IMMEDIATE(rf_chain), 0x00);
    lgw_reg_w(SX1302_REG_TX_TOP_TX_TRIG_TX_TRIG_DELAYED(rf_chain), 0x00);
    lgw_reg_w(SX1302_REG_TX_TOP_TX_TRIG_TX_TRIG_GPS(rf_chain), 0x00);

    do {
        wait_ms(1);
        lgw_status(rf_chain, TX_STATUS, &tx_status);
    } while (tx_status != TX_FREE);

    return LGW_HAL_SUCCESS;
}

/* ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ */

int lgw_get_trigcnt(uint32_t* trig_cnt_us) {
    return (sx1302_timestamp_counter(true, trig_cnt_us));
}

/* ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ */

int lgw_get_instcnt(uint32_t* inst_cnt_us) {
    return (sx1302_timestamp_counter(false, inst_cnt_us));
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
            /* TODO: update formula for SF5/SF6 */
            if (SF < 7) {
                DEBUG_MSG("WARNING: clipping time on air computing to SF7 for SF5/SF6\n");
                SF = 7;
            }
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
        Tfsk = (8 * (double)(packet->preamble + CONTEXT_FSK.sync_word_size + 1 + packet->size + ((packet->no_crc == true) ? 0 : 2)) / (double)packet->datarate) * 1E3;

        /* Duration of packet */
        Tpacket = (uint32_t)Tfsk + 1; /* add margin for rounding */
    } else {
        Tpacket = 0;
        DEBUG_PRINTF("ERROR: Cannot compute time on air for this packet, unsupported modulation (0x%02X)\n", packet->modulation);
    }

    return Tpacket;
}

/* --- EOF ------------------------------------------------------------------ */
