/*
 / _____)             _              | |
( (____  _____ ____ _| |_ _____  ____| |__
 \____ \| ___ |    (_   _) ___ |/ ___)  _ \
 _____) ) ____| | | || |_| ____( (___| | | |
(______/|_____)_|_|_| \__)_____)\____)_| |_|
  (C)2019 Semtech

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
#include <inttypes.h>

#include "loragw_reg.h"
#include "loragw_hal.h"
#include "loragw_aux.h"
#include "loragw_spi.h"
#include "loragw_i2c.h"
#include "loragw_sx1250.h"
#include "loragw_sx125x.h"
#include "loragw_sx1302.h"

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

#define TRACE()             fprintf(stderr, "@ %s %d\n", __FUNCTION__, __LINE__);

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

/* -------------------------------------------------------------------------- */
/* --- PRIVATE CONSTANTS & TYPES -------------------------------------------- */

#define FW_VERSION_AGC      1 /* Expected version of AGC firmware */
#define FW_VERSION_ARB      1 /* Expected version of arbiter firmware */

/* Useful bandwidth of SX125x radios to consider depending on channel bandwidth */
/* Note: the below values come from lab measurements. For any question, please contact Semtech support */
#define LGW_RF_RX_BANDWIDTH_125KHZ  1600000     /* for 125KHz channels */
#define LGW_RF_RX_BANDWIDTH_250KHZ  1600000     /* for 250KHz channels */
#define LGW_RF_RX_BANDWIDTH_500KHZ  1600000     /* for 500KHz channels */

#define LGW_RF_RX_FREQ_MIN          100E6
#define LGW_RF_RX_FREQ_MAX          1E9

/* Version string, used to identify the library version/options once compiled */
const char lgw_version_string[] = "Version: " LIBLORAGW_VERSION ";";

/* -------------------------------------------------------------------------- */
/* --- PRIVATE VARIABLES ---------------------------------------------------- */

#include "arb_fw.var"           /* text_arb_sx1302_13_Nov_3 */
#include "agc_fw_sx1250.var"    /* text_agc_sx1250_05_Juillet_2019_3 */
#include "agc_fw_sx1257.var"    /* text_agc_sx1257_19_Nov_1 */

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

/* File handle to write debug logs */
FILE * log_file = NULL;

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

/* -------------------------------------------------------------------------- */
/* --- PUBLIC FUNCTIONS DEFINITION ------------------------------------------ */

int lgw_board_setconf(struct lgw_conf_board_s * conf) {
    CHECK_NULL(conf);

    /* check if the concentrator is running */
    if (CONTEXT_STARTED == true) {
        DEBUG_MSG("ERROR: CONCENTRATOR IS RUNNING, STOP IT BEFORE TOUCHING CONFIGURATION\n");
        return LGW_HAL_ERROR;
    }

    /* set internal config according to parameters */
    CONTEXT_LWAN_PUBLIC = conf->lorawan_public;
    CONTEXT_BOARD.clksrc = conf->clksrc;
    CONTEXT_BOARD.full_duplex = conf->full_duplex;
    strncpy(CONTEXT_SPI, conf->spidev_path, sizeof CONTEXT_SPI);
    CONTEXT_SPI[sizeof CONTEXT_SPI - 1] = '\0'; /* ensure string termination */

    DEBUG_PRINTF("Note: board configuration: spidev_path: %s, lorawan_public:%d, clksrc:%d, full_duplex:%d\n",  CONTEXT_SPI,
                                                                                                                CONTEXT_LWAN_PUBLIC,
                                                                                                                CONTEXT_BOARD.clksrc,
                                                                                                                CONTEXT_BOARD.full_duplex);

    return LGW_HAL_SUCCESS;
}

/* ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ */

int lgw_rxrf_setconf(uint8_t rf_chain, struct lgw_conf_rxrf_s * conf) {
    CHECK_NULL(conf);

    /* check if the concentrator is running */
    if (CONTEXT_STARTED == true) {
        DEBUG_MSG("ERROR: CONCENTRATOR IS RUNNING, STOP IT BEFORE TOUCHING CONFIGURATION\n");
        return LGW_HAL_ERROR;
    }

    if (conf->enable == false) {
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
    if ((conf->type != LGW_RADIO_TYPE_SX1255) && (conf->type != LGW_RADIO_TYPE_SX1257) && (conf->type != LGW_RADIO_TYPE_SX1250)) {
        DEBUG_PRINTF("ERROR: NOT A VALID RADIO TYPE (%d)\n", conf->type);
        return LGW_HAL_ERROR;
    }

    /* check if the radio central frequency is valid */
    if ((conf->freq_hz < LGW_RF_RX_FREQ_MIN) || (conf->freq_hz > LGW_RF_RX_FREQ_MAX)) {
        DEBUG_PRINTF("ERROR: NOT A VALID RADIO CENTER FREQUENCY, PLEASE CHECK IF IT HAS BEEN GIVEN IN HZ (%u)\n", conf->freq_hz);
        return LGW_HAL_ERROR;
    }

    /* set internal config according to parameters */
    CONTEXT_RF_CHAIN[rf_chain].enable = conf->enable;
    CONTEXT_RF_CHAIN[rf_chain].freq_hz = conf->freq_hz;
    CONTEXT_RF_CHAIN[rf_chain].rssi_offset = conf->rssi_offset;
    CONTEXT_RF_CHAIN[rf_chain].rssi_tcomp.coeff_a = conf->rssi_tcomp.coeff_a;
    CONTEXT_RF_CHAIN[rf_chain].rssi_tcomp.coeff_b = conf->rssi_tcomp.coeff_b;
    CONTEXT_RF_CHAIN[rf_chain].rssi_tcomp.coeff_c = conf->rssi_tcomp.coeff_c;
    CONTEXT_RF_CHAIN[rf_chain].rssi_tcomp.coeff_d = conf->rssi_tcomp.coeff_d;
    CONTEXT_RF_CHAIN[rf_chain].rssi_tcomp.coeff_e = conf->rssi_tcomp.coeff_e;
    CONTEXT_RF_CHAIN[rf_chain].type = conf->type;
    CONTEXT_RF_CHAIN[rf_chain].tx_enable = conf->tx_enable;
    CONTEXT_RF_CHAIN[rf_chain].single_input_mode = conf->single_input_mode;

    DEBUG_PRINTF("Note: rf_chain %d configuration; en:%d freq:%d rssi_offset:%f radio_type:%d tx_enable:%d single_input_mode:%d\n",  rf_chain,
                                                                                                                CONTEXT_RF_CHAIN[rf_chain].enable,
                                                                                                                CONTEXT_RF_CHAIN[rf_chain].freq_hz,
                                                                                                                CONTEXT_RF_CHAIN[rf_chain].rssi_offset,
                                                                                                                CONTEXT_RF_CHAIN[rf_chain].type,
                                                                                                                CONTEXT_RF_CHAIN[rf_chain].tx_enable,
                                                                                                                CONTEXT_RF_CHAIN[rf_chain].single_input_mode);

    return LGW_HAL_SUCCESS;
}

/* ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ */

int lgw_rxif_setconf(uint8_t if_chain, struct lgw_conf_rxif_s * conf) {
    int32_t bw_hz;
    uint32_t rf_rx_bandwidth;

    CHECK_NULL(conf);

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
    if (conf->enable == false) {
        CONTEXT_IF_CHAIN[if_chain].enable = false;
        CONTEXT_IF_CHAIN[if_chain].freq_hz = 0;
        DEBUG_PRINTF("Note: if_chain %d disabled\n", if_chain);
        return LGW_HAL_SUCCESS;
    }

    /* check 'general' parameters */
    if (sx1302_get_ifmod_config(if_chain) == IF_UNDEFINED) {
        DEBUG_PRINTF("ERROR: IF CHAIN %d NOT CONFIGURABLE\n", if_chain);
    }
    if (conf->rf_chain >= LGW_RF_CHAIN_NB) {
        DEBUG_MSG("ERROR: INVALID RF_CHAIN TO ASSOCIATE WITH A LORA_STD IF CHAIN\n");
        return LGW_HAL_ERROR;
    }
    /* check if IF frequency is optimal based on channel and radio bandwidths */
    switch (conf->bandwidth) {
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
    bw_hz = lgw_bw_getval(conf->bandwidth); /* channel bandwidth */
    if ((conf->freq_hz + ((bw_hz==-1)?LGW_REF_BW:bw_hz)/2) > ((int32_t)rf_rx_bandwidth/2)) {
        DEBUG_PRINTF("ERROR: IF FREQUENCY %d TOO HIGH\n", conf->freq_hz);
        return LGW_HAL_ERROR;
    } else if ((conf->freq_hz - ((bw_hz==-1)?LGW_REF_BW:bw_hz)/2) < -((int32_t)rf_rx_bandwidth/2)) {
        DEBUG_PRINTF("ERROR: IF FREQUENCY %d TOO LOW\n", conf->freq_hz);
        return LGW_HAL_ERROR;
    }

    /* check parameters according to the type of IF chain + modem,
    fill default if necessary, and commit configuration if everything is OK */
    switch (sx1302_get_ifmod_config(if_chain)) {
        case IF_LORA_STD:
            /* fill default parameters if needed */
            if (conf->bandwidth == BW_UNDEFINED) {
                conf->bandwidth = BW_250KHZ;
            }
            if (conf->datarate == DR_UNDEFINED) {
                conf->datarate = DR_LORA_SF7;
            }
            /* check BW & DR */
            if (!IS_LORA_BW(conf->bandwidth)) {
                DEBUG_MSG("ERROR: BANDWIDTH NOT SUPPORTED BY LORA_STD IF CHAIN\n");
                return LGW_HAL_ERROR;
            }
            if (!IS_LORA_DR(conf->datarate)) {
                DEBUG_MSG("ERROR: DATARATE NOT SUPPORTED BY LORA_STD IF CHAIN\n");
                return LGW_HAL_ERROR;
            }
            /* set internal configuration  */
            CONTEXT_IF_CHAIN[if_chain].enable = conf->enable;
            CONTEXT_IF_CHAIN[if_chain].rf_chain = conf->rf_chain;
            CONTEXT_IF_CHAIN[if_chain].freq_hz = conf->freq_hz;
            CONTEXT_LORA_SERVICE.bandwidth = conf->bandwidth;
            CONTEXT_LORA_SERVICE.datarate = conf->datarate;
            CONTEXT_LORA_SERVICE.implicit_hdr = conf->implicit_hdr;
            CONTEXT_LORA_SERVICE.implicit_payload_length = conf->implicit_payload_length;
            CONTEXT_LORA_SERVICE.implicit_crc_en   = conf->implicit_crc_en;
            CONTEXT_LORA_SERVICE.implicit_coderate = conf->implicit_coderate;

            DEBUG_PRINTF("Note: LoRa 'std' if_chain %d configuration; en:%d freq:%d bw:%d dr:%d\n", if_chain,
                                                                                                    CONTEXT_IF_CHAIN[if_chain].enable,
                                                                                                    CONTEXT_IF_CHAIN[if_chain].freq_hz,
                                                                                                    CONTEXT_LORA_SERVICE.bandwidth,
                                                                                                    CONTEXT_LORA_SERVICE.datarate);
            break;

        case IF_LORA_MULTI:
            /* fill default parameters if needed */
            if (conf->bandwidth == BW_UNDEFINED) {
                conf->bandwidth = BW_125KHZ;
            }
            if (conf->datarate == DR_UNDEFINED) {
                conf->datarate = DR_LORA_SF7;
            }
            /* check BW & DR */
            if (conf->bandwidth != BW_125KHZ) {
                DEBUG_MSG("ERROR: BANDWIDTH NOT SUPPORTED BY LORA_MULTI IF CHAIN\n");
                return LGW_HAL_ERROR;
            }
            if (!IS_LORA_DR(conf->datarate)) {
                DEBUG_MSG("ERROR: DATARATE(S) NOT SUPPORTED BY LORA_MULTI IF CHAIN\n");
                return LGW_HAL_ERROR;
            }
            /* set internal configuration  */
            CONTEXT_IF_CHAIN[if_chain].enable = conf->enable;
            CONTEXT_IF_CHAIN[if_chain].rf_chain = conf->rf_chain;
            CONTEXT_IF_CHAIN[if_chain].freq_hz = conf->freq_hz;

            DEBUG_PRINTF("Note: LoRa 'multi' if_chain %d configuration; en:%d freq:%d\n",   if_chain,
                                                                                            CONTEXT_IF_CHAIN[if_chain].enable,
                                                                                            CONTEXT_IF_CHAIN[if_chain].freq_hz);
            break;

        case IF_FSK_STD:
            /* fill default parameters if needed */
            if (conf->bandwidth == BW_UNDEFINED) {
                conf->bandwidth = BW_250KHZ;
            }
            if (conf->datarate == DR_UNDEFINED) {
                conf->datarate = 64000; /* default datarate */
            }
            /* check BW & DR */
            if(!IS_FSK_BW(conf->bandwidth)) {
                DEBUG_MSG("ERROR: BANDWIDTH NOT SUPPORTED BY FSK IF CHAIN\n");
                return LGW_HAL_ERROR;
            }
            if(!IS_FSK_DR(conf->datarate)) {
                DEBUG_MSG("ERROR: DATARATE NOT SUPPORTED BY FSK IF CHAIN\n");
                return LGW_HAL_ERROR;
            }
            /* set internal configuration  */
            CONTEXT_IF_CHAIN[if_chain].enable = conf->enable;
            CONTEXT_IF_CHAIN[if_chain].rf_chain = conf->rf_chain;
            CONTEXT_IF_CHAIN[if_chain].freq_hz = conf->freq_hz;
            CONTEXT_FSK.bandwidth = conf->bandwidth;
            CONTEXT_FSK.datarate = conf->datarate;
            if (conf->sync_word > 0) {
                CONTEXT_FSK.sync_word_size = conf->sync_word_size;
                CONTEXT_FSK.sync_word = conf->sync_word;
            }
            DEBUG_PRINTF("Note: FSK if_chain %d configuration; en:%d freq:%d bw:%d dr:%d (%d real dr) sync:0x%0*" PRIu64 "\n", if_chain,
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

int lgw_txgain_setconf(uint8_t rf_chain, struct lgw_tx_gain_lut_s * conf) {
    int i;

    CHECK_NULL(conf);

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
        if (conf->lut[i].pwr_idx > 22) {
            DEBUG_MSG("ERROR: TX gain LUT: SX1250 power iundex must not exceed 22\n");
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

int lgw_timestamp_setconf(struct lgw_conf_timestamp_s * conf) {
    CHECK_NULL(conf);

    CONTEXT_TIMESTAMP.enable_precision_ts = conf->enable_precision_ts;
    CONTEXT_TIMESTAMP.max_ts_metrics = conf->max_ts_metrics;
    CONTEXT_TIMESTAMP.nb_symbols = conf->nb_symbols;

    return LGW_HAL_SUCCESS;
}

/* ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ */

int lgw_debug_setconf(struct lgw_conf_debug_s * conf) {
    int i;

    CHECK_NULL(conf);

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
        strncpy(CONTEXT_DEBUG.log_file_name, conf->log_file_name, sizeof CONTEXT_DEBUG.log_file_name);
        CONTEXT_DEBUG.log_file_name[sizeof CONTEXT_DEBUG.log_file_name - 1] = '\0'; /* ensure string termination */
    }

    return LGW_HAL_SUCCESS;
}

/* ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ */

int lgw_start(void) {
    int i, err;
    int reg_stat;

    if (CONTEXT_STARTED == true) {
        DEBUG_MSG("Note: LoRa concentrator already started, restarting it now\n");
    }

    reg_stat = lgw_connect(CONTEXT_SPI);
    if (reg_stat == LGW_REG_ERROR) {
        DEBUG_MSG("ERROR: FAIL TO CONNECT BOARD\n");
        return LGW_HAL_ERROR;
    }

    /* Calibrate radios */
    err = sx1302_radio_calibrate(&CONTEXT_RF_CHAIN[0], CONTEXT_BOARD.clksrc, &CONTEXT_TX_GAIN_LUT[0]);
    if (err != LGW_REG_SUCCESS) {
        printf("ERROR: radio calibration failed\n");
        return LGW_HAL_ERROR;
    }

    /* Setup radios for RX */
    for (i = 0; i < LGW_RF_CHAIN_NB; i++) {
        if (CONTEXT_RF_CHAIN[i].enable == true) {
            sx1302_radio_reset(i, CONTEXT_RF_CHAIN[i].type);
            switch (CONTEXT_RF_CHAIN[i].type) {
                case LGW_RADIO_TYPE_SX1250:
                    sx1250_setup(i, CONTEXT_RF_CHAIN[i].freq_hz, CONTEXT_RF_CHAIN[i].single_input_mode);
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

    /* Basic initialization of the sx1302 */
    sx1302_init(&CONTEXT_TIMESTAMP);

    /* Configure PA/LNA LUTs */
    sx1302_pa_lna_lut_configure();

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

    /* enable demodulators - to be done before starting AGC/ARB */
    sx1302_modem_enable();

    /* Load firmware */
    switch (CONTEXT_RF_CHAIN[CONTEXT_BOARD.clksrc].type) {
        case LGW_RADIO_TYPE_SX1250:
            DEBUG_MSG("Loading AGC fw for sx1250\n");
            if (sx1302_agc_load_firmware(agc_firmware_sx1250) != LGW_HAL_SUCCESS) {
                return LGW_HAL_ERROR;
            }
            break;
        case LGW_RADIO_TYPE_SX1257:
            DEBUG_MSG("Loading AGC fw for sx125x\n");
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
    DEBUG_MSG("Loading ARB fw\n");
    if (sx1302_arb_load_firmware(arb_firmware) != LGW_HAL_SUCCESS) {
        return LGW_HAL_ERROR;
    }
    if (sx1302_arb_start(FW_VERSION_ARB) != LGW_HAL_SUCCESS) {
        return LGW_HAL_ERROR;
    }

    /* static TX configuration */
    sx1302_tx_configure(CONTEXT_RF_CHAIN[CONTEXT_BOARD.clksrc].type);

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

    /* set hal state */
    CONTEXT_STARTED = true;

    return LGW_HAL_SUCCESS;
}

/* ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ */

int lgw_stop(void) {
    int i;

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

    CONTEXT_STARTED = false;
    return LGW_HAL_SUCCESS;
}

/* ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ */

int lgw_receive(uint8_t max_pkt, struct lgw_pkt_rx_s *pkt_data) {
    int res;
    uint8_t  nb_pkt_fetched = 0;
    uint16_t nb_pkt_found = 0;
    uint16_t nb_pkt_left = 0;
    float current_temperature = 30.0;
    float rssi_temperature_offset;

    /* Check that AGC/ARB firmwares are not corrupted, and update internal counter */
    /* WARNING: this needs to be called regularly by the upper layer */
    res = sx1302_update();
    if (res != LGW_REG_SUCCESS) {
        return LGW_HAL_ERROR;
    }

    /* Get packets from SX1302, if any */
    res = sx1302_fetch(&nb_pkt_fetched);
    if (res != LGW_REG_SUCCESS) {
        printf("ERROR: failed to fetch packets from SX1302\n");
        return LGW_HAL_ERROR;
    }
    if (nb_pkt_fetched == 0) {
        return 0;
    }
    if (nb_pkt_fetched > max_pkt) {
        nb_pkt_left = nb_pkt_fetched - max_pkt;
        printf("WARNING: not enough space allocated, fetched %d packet(s), %d will be left in RX buffer\n", nb_pkt_fetched, nb_pkt_left);
    }

    /* Iterate on the RX buffer to get parsed packets */
    for (nb_pkt_found = 0; nb_pkt_found < ((nb_pkt_fetched <= max_pkt) ? nb_pkt_fetched : max_pkt); nb_pkt_found++) {
        /* Get packet and move to next one */
        res = sx1302_parse(&lgw_context, &pkt_data[nb_pkt_found]);
        if (res != LGW_REG_SUCCESS) {
            printf("ERROR: failed to parse fetched packet %d, aborting...\n", nb_pkt_found);
            return LGW_HAL_ERROR;
        }

        /* Appli RSSI offset calibrated for the board */
        pkt_data[nb_pkt_found].rssic += CONTEXT_RF_CHAIN[pkt_data[nb_pkt_found].rf_chain].rssi_offset;
        pkt_data[nb_pkt_found].rssis += CONTEXT_RF_CHAIN[pkt_data[nb_pkt_found].rf_chain].rssi_offset;

        rssi_temperature_offset = sx1302_rssi_get_temperature_offset(&CONTEXT_RF_CHAIN[pkt_data[nb_pkt_found].rf_chain].rssi_tcomp, current_temperature);
        pkt_data[nb_pkt_found].rssic += rssi_temperature_offset;
        pkt_data[nb_pkt_found].rssis += rssi_temperature_offset;
        DEBUG_PRINTF("INFO: RSSI temperature offset applied: %.3f dB (current temperature %.1f C)\n", rssi_temperature_offset, current_temperature);
    }

    DEBUG_PRINTF("INFO: nb pkt found:%u left:%u\n", nb_pkt_found, nb_pkt_left);

    return nb_pkt_found;
}

/* ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ */

int lgw_send(struct lgw_pkt_tx_s * pkt_data) {
    /* check if the concentrator is running */
    if (CONTEXT_STARTED == false) {
        DEBUG_MSG("ERROR: CONCENTRATOR IS NOT RUNNING, START IT BEFORE SENDING\n");
        return LGW_HAL_ERROR;
    }

    CHECK_NULL(pkt_data);

    /* check input range (segfault prevention) */
    if (pkt_data->rf_chain >= LGW_RF_CHAIN_NB) {
        DEBUG_MSG("ERROR: INVALID RF_CHAIN TO SEND PACKETS\n");
        return LGW_HAL_ERROR;
    }

    /* check input variables */
    if (CONTEXT_RF_CHAIN[pkt_data->rf_chain].tx_enable == false) {
        DEBUG_MSG("ERROR: SELECTED RF_CHAIN IS DISABLED FOR TX ON SELECTED BOARD\n");
        return LGW_HAL_ERROR;
    }
    if (CONTEXT_RF_CHAIN[pkt_data->rf_chain].enable == false) {
        DEBUG_MSG("ERROR: SELECTED RF_CHAIN IS DISABLED\n");
        return LGW_HAL_ERROR;
    }
    if (!IS_TX_MODE(pkt_data->tx_mode)) {
        DEBUG_MSG("ERROR: TX_MODE NOT SUPPORTED\n");
        return LGW_HAL_ERROR;
    }
    if (pkt_data->modulation == MOD_LORA) {
        if (!IS_LORA_BW(pkt_data->bandwidth)) {
            DEBUG_MSG("ERROR: BANDWIDTH NOT SUPPORTED BY LORA TX\n");
            return LGW_HAL_ERROR;
        }
        if (!IS_LORA_DR(pkt_data->datarate)) {
            DEBUG_MSG("ERROR: DATARATE NOT SUPPORTED BY LORA TX\n");
            return LGW_HAL_ERROR;
        }
        if (!IS_LORA_CR(pkt_data->coderate)) {
            DEBUG_MSG("ERROR: CODERATE NOT SUPPORTED BY LORA TX\n");
            return LGW_HAL_ERROR;
        }
        if (pkt_data->size > 255) {
            DEBUG_MSG("ERROR: PAYLOAD LENGTH TOO BIG FOR LORA TX\n");
            return LGW_HAL_ERROR;
        }
    } else if (pkt_data->modulation == MOD_FSK) {
        if((pkt_data->f_dev < 1) || (pkt_data->f_dev > 200)) {
            DEBUG_MSG("ERROR: TX FREQUENCY DEVIATION OUT OF ACCEPTABLE RANGE\n");
            return LGW_HAL_ERROR;
        }
        if(!IS_FSK_DR(pkt_data->datarate)) {
            DEBUG_MSG("ERROR: DATARATE NOT SUPPORTED BY FSK IF CHAIN\n");
            return LGW_HAL_ERROR;
        }
        if (pkt_data->size > 255) {
            DEBUG_MSG("ERROR: PAYLOAD LENGTH TOO BIG FOR FSK TX\n");
            return LGW_HAL_ERROR;
        }
    } else if (pkt_data->modulation == MOD_CW) {
        /* do nothing */
    } else {
        DEBUG_MSG("ERROR: INVALID TX MODULATION\n");
        return LGW_HAL_ERROR;
    }

    return sx1302_send(CONTEXT_RF_CHAIN[pkt_data->rf_chain].type, &CONTEXT_TX_GAIN_LUT[pkt_data->rf_chain], CONTEXT_LWAN_PUBLIC, &CONTEXT_FSK, pkt_data);
}

/* ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ */

int lgw_status(uint8_t rf_chain, uint8_t select, uint8_t *code) {
    /* check input variables */
    CHECK_NULL(code);
    if (rf_chain >= LGW_RF_CHAIN_NB) {
        DEBUG_MSG("ERROR: NOT A VALID RF_CHAIN NUMBER\n");
        return LGW_HAL_ERROR;
    }

    /* Get status */
    if (select == TX_STATUS) {
        if (CONTEXT_STARTED == false) {
            *code = TX_OFF;
        } else {
            *code = sx1302_tx_status(rf_chain);
        }
    } else if (select == RX_STATUS) {
        if (CONTEXT_STARTED == false) {
            *code = RX_OFF;
        } else {
            *code = sx1302_rx_status(rf_chain);
        }
    } else {
        DEBUG_MSG("ERROR: SELECTION INVALID, NO STATUS TO RETURN\n");
        return LGW_HAL_ERROR;
    }

    //DEBUG_PRINTF("INFO: STATUS %u\n", *code);
    return LGW_HAL_SUCCESS;
}

/* ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ */

int lgw_abort_tx(uint8_t rf_chain) {
    /* check input variables */
    if (rf_chain >= LGW_RF_CHAIN_NB) {
        DEBUG_MSG("ERROR: NOT A VALID RF_CHAIN NUMBER\n");
        return LGW_HAL_ERROR;
    }

    /* Abort current TX */
    return sx1302_tx_abort(rf_chain);
}

/* ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ */

int lgw_get_trigcnt(uint32_t* trig_cnt_us) {
    CHECK_NULL(trig_cnt_us);

    *trig_cnt_us = sx1302_timestamp_counter(true);

    return LGW_HAL_SUCCESS;
}

/* ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ */

int lgw_get_instcnt(uint32_t* inst_cnt_us) {
    CHECK_NULL(inst_cnt_us);

    *inst_cnt_us = sx1302_timestamp_counter(false);

    return LGW_HAL_SUCCESS;
}

/* ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ */

int lgw_get_eui(uint64_t* eui) {
    CHECK_NULL(eui);

    if (sx1302_get_eui(eui) != LGW_REG_SUCCESS) {
        return LGW_HAL_ERROR;
    }
    return LGW_HAL_SUCCESS;
}

/* ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ */

int lgw_get_temperature(float* temperature) {
    CHECK_NULL(temperature);
    *temperature = 30.0;

    return LGW_HAL_SUCCESS;
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
