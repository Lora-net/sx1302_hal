/*
 / _____)             _              | |
( (____  _____ ____ _| |_ _____  ____| |__
 \____ \| ___ |    (_   _) ___ |/ ___)  _ \
 _____) ) ____| | | || |_| ____( (___| | | |
(______/|_____)_|_|_| \__)_____)\____)_| |_|
  (C)2019 Semtech

Description:
    SX1302 Hardware Abstraction Layer entry functions.

License: Revised BSD License, see LICENSE.TXT file include in the project
*/


/* -------------------------------------------------------------------------- */
/* --- DEPENDANCIES --------------------------------------------------------- */

#include <stdint.h>     /* C99 types */
#include <stdio.h>      /* printf fprintf */
#include <string.h>     /* memcmp */
#include <math.h>       /* pow, cell */
#include <inttypes.h>
#include <time.h>

#include "loragw_reg.h"
#include "loragw_aux.h"
#include "loragw_hal.h"
#include "loragw_sx1302.h"
#include "loragw_sx1302_timestamp.h"
#include "loragw_sx1302_rx.h"
#include "loragw_sx1250.h"
#include "loragw_agc_params.h"
#include "loragw_cal.h"
#include "loragw_debug.h"

/* -------------------------------------------------------------------------- */
/* --- PRIVATE MACROS ------------------------------------------------------- */

#define ARRAY_SIZE(a) (sizeof(a) / sizeof((a)[0]))
#if DEBUG_SX1302 == 1
    #define DEBUG_MSG(str)              fprintf(stdout, str)
    #define DEBUG_PRINTF(fmt, args...)  fprintf(stdout, fmt, args)
    #define CHECK_NULL(a)               if(a==NULL){fprintf(stderr,"%s:%d: ERROR: NULL POINTER AS ARGUMENT\n", __FUNCTION__, __LINE__);return LGW_REG_ERROR;}
#else
    #define DEBUG_MSG(str)
    #define DEBUG_PRINTF(fmt, args...)
    #define CHECK_NULL(a)               if(a==NULL){return LGW_REG_ERROR;}
#endif
#define CHECK_ERR(a)                    if(a==-1){return LGW_REG_ERROR;}

#define IF_HZ_TO_REG(f)     ((f * 32) / 15625)

#define SX1302_FREQ_TO_REG(f)   (uint32_t)((uint64_t)f * (1 << 18) / 32000000U)

/* -------------------------------------------------------------------------- */
/* --- PRIVATE TYPES -------------------------------------------------------- */

/* -------------------------------------------------------------------------- */
/* --- PRIVATE CONSTANTS ---------------------------------------------------- */

#define AGC_RADIO_A_INIT_DONE   0x80
#define AGC_RADIO_B_INIT_DONE   0x20

#define MCU_AGC                 0x01
#define MCU_ARB                 0x02

#define AGC_MEM_ADDR            0x0000
#define ARB_MEM_ADDR            0x2000

#define MCU_FW_SIZE             8192 /* size of the firmware IN BYTES (= twice the number of 14b words) */

#define FW_VERSION_CAL          1 /* Expected version of calibration firmware */

#define RSSI_FSK_POLY_0         90.636423 /* polynomiam coefficients to linearize FSK RSSI */
#define RSSI_FSK_POLY_1         0.420835
#define RSSI_FSK_POLY_2         0.007129
#define RSSI_FSK_POLY_3         -0.000026

#define FREQ_OFFSET_LSB_125KHZ  0.11920929f     /* 125000 * 32 / 2^6 / 2^19 */
#define FREQ_OFFSET_LSB_250KHZ  0.238418579f    /* 250000 * 32 / 2^6 / 2^19 */
#define FREQ_OFFSET_LSB_500KHZ  0.476837158f    /* 500000 * 32 / 2^6 / 2^19 */

/* sx1302 hardware modem capabilities */
#define LGW_IFMODEM_CONFIG {\
        IF_LORA_MULTI, \
        IF_LORA_MULTI, \
        IF_LORA_MULTI, \
        IF_LORA_MULTI, \
        IF_LORA_MULTI, \
        IF_LORA_MULTI, \
        IF_LORA_MULTI, \
        IF_LORA_MULTI, \
        IF_LORA_STD, \
        IF_FSK_STD } /* configuration of available IF chains and modems on the hardware */

/* constant arrays defining hardware capability */
const uint8_t ifmod_config[LGW_IF_CHAIN_NB] = LGW_IFMODEM_CONFIG;

#define MIN_LORA_PREAMBLE   6
#define STD_LORA_PREAMBLE   8
#define MIN_FSK_PREAMBLE    3
#define STD_FSK_PREAMBLE    5

#define GPIO_CFG_REGISTER               0x00
#define GPIO_CFG_AGC                    0x01
#define GPIO_CFG_ARB                    0x02
#define GPIO_CFG_SPI_EXP_1              0x03
#define GPIO_CFG_CSN_SPI_EXP            0x04
#define GPIO_CFG_SPI_EXP_2              0x05
#define GPIO_CFG_UART                   0x06
#define GPIO_CFG_SX1255_IQ              0x07
#define GPIO_CFG_SX1261_IQ              0x08
#define GPIO_CFG_STATUS                 0x09
#define GPIO_CFG_MBIST                  0x0A
#define GPIO_CFG_OTP                    0x0B

/* -------------------------------------------------------------------------- */
/* --- PRIVATE VARIABLES ---------------------------------------------------- */

/* Radio calibration firmware */
#include "cal_fw.var" /* text_cal_sx1257_16_Nov_1 */

/* Buffer to hold RX data */
rx_buffer_t rx_buffer;

/* Internal timestamp counter */
timestamp_counter_t counter_us;

/* -------------------------------------------------------------------------- */
/* --- PRIVATE FUNCTIONS DECLARATION ---------------------------------------- */

/**
@brief TODO
@param TODO
@return TODO
*/
extern int32_t lgw_sf_getval(int x);

/**
@brief TODO
@param TODO
@return TODO
*/
extern int32_t lgw_bw_getval(int x);

/**
@brief TODO
@param TODO
@return TODO
*/
void lora_crc16(const char data, int *crc);

/* -------------------------------------------------------------------------- */
/* --- INTERNAL SHARED VARIABLES -------------------------------------------- */

/* Log file */
extern FILE * log_file;

/* -------------------------------------------------------------------------- */
/* --- PRIVATE FUNCTIONS DEFINITION ----------------------------------------- */

int calculate_freq_to_time_drift(uint32_t freq_hz, uint8_t bw, uint16_t * mant, uint8_t * exp) {
    uint64_t mantissa_u64;
    uint8_t exponent = 0;
    int32_t bw_hz;

    /* check input variables */
    CHECK_NULL(mant);
    CHECK_NULL(exp);

    bw_hz = lgw_bw_getval(bw);
    if (bw_hz < 0) {
        printf("ERROR: Unsupported bandwidth for frequency to time drift calculation\n");
        return LGW_REG_ERROR;
    }

    mantissa_u64 = (uint64_t)bw_hz * (2 << (20-1)) / freq_hz;
    while (mantissa_u64 < 2048) {
        exponent += 1;
        mantissa_u64 <<= 1;
    }

    *mant = (uint16_t)mantissa_u64;
    *exp = exponent;

    return LGW_REG_SUCCESS;
}

/* ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ */

void lora_crc16(const char data, int *crc) {
    int next = 0;
    next  =  (((data>>0)&1) ^ ((*crc>>12)&1) ^ ((*crc>> 8)&1)                 )      ;
    next += ((((data>>1)&1) ^ ((*crc>>13)&1) ^ ((*crc>> 9)&1)                 )<<1 ) ;
    next += ((((data>>2)&1) ^ ((*crc>>14)&1) ^ ((*crc>>10)&1)                 )<<2 ) ;
    next += ((((data>>3)&1) ^ ((*crc>>15)&1) ^ ((*crc>>11)&1)                 )<<3 ) ;
    next += ((((data>>4)&1) ^ ((*crc>>12)&1)                                  )<<4 ) ;
    next += ((((data>>5)&1) ^ ((*crc>>13)&1) ^ ((*crc>>12)&1) ^ ((*crc>> 8)&1))<<5 ) ;
    next += ((((data>>6)&1) ^ ((*crc>>14)&1) ^ ((*crc>>13)&1) ^ ((*crc>> 9)&1))<<6 ) ;
    next += ((((data>>7)&1) ^ ((*crc>>15)&1) ^ ((*crc>>14)&1) ^ ((*crc>>10)&1))<<7 ) ;
    next += ((((*crc>>0)&1) ^ ((*crc>>15)&1) ^ ((*crc>>11)&1)                 )<<8 ) ;
    next += ((((*crc>>1)&1) ^ ((*crc>>12)&1)                                  )<<9 ) ;
    next += ((((*crc>>2)&1) ^ ((*crc>>13)&1)                                  )<<10) ;
    next += ((((*crc>>3)&1) ^ ((*crc>>14)&1)                                  )<<11) ;
    next += ((((*crc>>4)&1) ^ ((*crc>>15)&1) ^ ((*crc>>12)&1) ^ ((*crc>> 8)&1))<<12) ;
    next += ((((*crc>>5)&1) ^ ((*crc>>13)&1) ^ ((*crc>> 9)&1)                 )<<13) ;
    next += ((((*crc>>6)&1) ^ ((*crc>>14)&1) ^ ((*crc>>10)&1)                 )<<14) ;
    next += ((((*crc>>7)&1) ^ ((*crc>>15)&1) ^ ((*crc>>11)&1)                 )<<15) ;
    (*crc) = next;
}

/* ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ */

int sx1302_config_gpio(void) {
    int err;

    err = lgw_reg_w(SX1302_REG_GPIO_GPIO_SEL_0_SELECTION, GPIO_CFG_REGISTER); /* GPIO_0 => CONFIG_DONE */
    CHECK_ERR(err);
    err = lgw_reg_w(SX1302_REG_GPIO_GPIO_SEL_1_SELECTION, GPIO_CFG_REGISTER); /* GPIO_1 => UNUSED */
    CHECK_ERR(err);
    err = lgw_reg_w(SX1302_REG_GPIO_GPIO_SEL_2_SELECTION, GPIO_CFG_STATUS);   /* GPIO_2 => Tx ON */
    CHECK_ERR(err);
    err = lgw_reg_w(SX1302_REG_GPIO_GPIO_SEL_3_SELECTION, GPIO_CFG_REGISTER); /* GPIO_3 => UNUSED */
    CHECK_ERR(err);
    err = lgw_reg_w(SX1302_REG_GPIO_GPIO_SEL_4_SELECTION, GPIO_CFG_STATUS);   /* GPIO_4 => RX ON (PKT_RECEIVE_TOGGLE_OUT) */
    CHECK_ERR(err);
    err = lgw_reg_w(SX1302_REG_GPIO_GPIO_SEL_5_SELECTION, GPIO_CFG_REGISTER); /* GPIO_5 => UNUSED */
    CHECK_ERR(err);
    err = lgw_reg_w(SX1302_REG_GPIO_GPIO_SEL_6_SELECTION, GPIO_CFG_REGISTER); /* GPIO_6 => UNUSED */
    CHECK_ERR(err);
    err = lgw_reg_w(SX1302_REG_GPIO_GPIO_SEL_7_SELECTION, GPIO_CFG_AGC);      /* GPIO_7 => USED FOR LBT (MUST BE INPUT) */
    CHECK_ERR(err);
    err = lgw_reg_w(SX1302_REG_GPIO_GPIO_DIR_L_DIRECTION, 0x7F);              /* GPIO output direction (0 for input and 1 for output) */
    CHECK_ERR(err);

    return LGW_REG_SUCCESS;
}

/* -------------------------------------------------------------------------- */
/* --- PUBLIC FUNCTIONS DEFINITION ------------------------------------------ */

int sx1302_init(const struct lgw_conf_ftime_s * ftime_context) {
    sx1302_model_id_t model_id;
    int x;

    /* Check input parameters */
    CHECK_NULL(ftime_context);

    /* Initialize internal counter */
    timestamp_counter_new(&counter_us);

    /* Initialize RX buffer */
    rx_buffer_new(&rx_buffer);

    /* Configure timestamping mode */
    if (ftime_context->enable == true) {
        x = sx1302_get_model_id(&model_id);
        if (x != LGW_REG_SUCCESS) {
            printf("ERROR: failed to get Chip Model ID\n");
            return LGW_REG_ERROR;
        }

        if (model_id != CHIP_MODEL_ID_SX1303) {
            printf("ERROR: Fine Timestamping is not supported on this Chip Model ID 0x%02X\n", model_id);
            return LGW_REG_ERROR;
        }
    }
    x = timestamp_counter_mode(ftime_context->enable);
    if (x != LGW_REG_SUCCESS) {
        printf("ERROR: failed to configure timestamp counter mode\n");
        return LGW_REG_ERROR;
    }

    x = sx1302_config_gpio();
    if (x != LGW_REG_SUCCESS) {
        printf("ERROR: failed to configure sx1302 GPIOs\n");
        return LGW_REG_ERROR;
    }

    return LGW_REG_SUCCESS;
}

/* ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ */

int sx1302_get_eui(uint64_t * eui) {
    int i, err;
    int32_t val;

    *eui = 0;
    for (i = 0; i < 8; i++) {
        err = lgw_reg_w(SX1302_REG_OTP_BYTE_ADDR_ADDR, i);
        if (err != LGW_REG_SUCCESS) {
            return LGW_REG_ERROR;
        }
        err = lgw_reg_r(SX1302_REG_OTP_RD_DATA_RD_DATA, &val);
        if (err != LGW_REG_SUCCESS) {
            return LGW_REG_ERROR;
        }

        *eui |= (uint64_t)((uint8_t)val) << (56 - (i * 8));
    }

    return LGW_REG_SUCCESS;
}

/* ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ */

int sx1302_get_model_id(sx1302_model_id_t * model_id) {
    int  err;
    int32_t val;

    /* Select ChipModelID */
    err = lgw_reg_w(SX1302_REG_OTP_BYTE_ADDR_ADDR, 0xD0);
    if (err != LGW_REG_SUCCESS) {
        return LGW_REG_ERROR;
    }

    /* Read Modem ID */
    err = lgw_reg_r(SX1302_REG_OTP_RD_DATA_RD_DATA, &val);
    if (err != LGW_REG_SUCCESS) {
        return LGW_REG_ERROR;
    }
    *model_id = (sx1302_model_id_t)val;

    return LGW_REG_SUCCESS;
}

/* ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ */

int sx1302_update(void) {
    uint32_t inst, pps;
    /* performances variables */
    struct timeval tm;

    /* Record function start time */
    _meas_time_start(&tm);

#if 0 /* Disabled because it brings latency on USB, for low value. TODO: do this less frequently ? */
    int32_t val;

    /* Check MCUs parity errors */
    lgw_reg_r(SX1302_REG_AGC_MCU_CTRL_PARITY_ERROR, &val);
    if (val != 0) {
        printf("ERROR: Parity error check failed on AGC firmware\n");
        return LGW_REG_ERROR;
    }
    lgw_reg_r(SX1302_REG_ARB_MCU_CTRL_PARITY_ERROR, &val);
    if (val != 0) {
        printf("ERROR: Parity error check failed on ARB firmware\n");
        return LGW_REG_ERROR;
    }
#endif

    /* Update internal timestamp counter wrapping status */
    timestamp_counter_get(&counter_us, &inst, &pps);

    _meas_time_stop(2, tm, __FUNCTION__);

    return LGW_REG_SUCCESS;
}

/* ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ */

int sx1302_radio_clock_select(uint8_t rf_chain) {
    int err = LGW_REG_SUCCESS;

    /* Check input parameters */
    if (rf_chain >= LGW_RF_CHAIN_NB)
    {
        DEBUG_MSG("ERROR: invalid RF chain\n");
        return LGW_REG_ERROR;
    }

    /* Switch SX1302 clock from SPI clock to radio clock of the selected RF chain */
    switch (rf_chain) {
        case 0:
            DEBUG_MSG("Select Radio A clock\n");
            err |= lgw_reg_w(SX1302_REG_CLK_CTRL_CLK_SEL_CLK_RADIO_A_SEL, 0x01);
            err |= lgw_reg_w(SX1302_REG_CLK_CTRL_CLK_SEL_CLK_RADIO_B_SEL, 0x00);
            break;
        case 1:
            DEBUG_MSG("Select Radio B clock\n");
            err |= lgw_reg_w(SX1302_REG_CLK_CTRL_CLK_SEL_CLK_RADIO_A_SEL, 0x00);
            err |= lgw_reg_w(SX1302_REG_CLK_CTRL_CLK_SEL_CLK_RADIO_B_SEL, 0x01);
            break;
        default:
            return LGW_REG_ERROR;
    }

    /* Enable clock dividers */
    err |= lgw_reg_w(SX1302_REG_CLK_CTRL_CLK_SEL_CLKDIV_EN, 0x01);

    /* Set the RIF clock to the 32MHz clock of the radio */
    err |= lgw_reg_w(SX1302_REG_COMMON_CTRL0_CLK32_RIF_CTRL, 0x01);

    /* Check if something went wrong */
    if (err != LGW_REG_SUCCESS) {
        printf("ERROR: failed to select radio clock for radio_%u\n", rf_chain);
        return LGW_REG_ERROR;
    }

    return LGW_REG_SUCCESS;
}

/* ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ */

int sx1302_radio_reset(uint8_t rf_chain, lgw_radio_type_t type) {
    uint16_t reg_radio_en;
    uint16_t reg_radio_rst;
    int err = LGW_REG_SUCCESS;

    /* Check input parameters */
    if (rf_chain >= LGW_RF_CHAIN_NB)
    {
        DEBUG_MSG("ERROR: invalid RF chain\n");
        return LGW_REG_ERROR;
    }
    if ((type != LGW_RADIO_TYPE_SX1255) && (type != LGW_RADIO_TYPE_SX1257) && (type != LGW_RADIO_TYPE_SX1250)) {
        DEBUG_MSG("ERROR: invalid radio type\n");
        return LGW_REG_ERROR;
    }

    /* Switch to SPI clock before reseting the radio */
    err |= lgw_reg_w(SX1302_REG_COMMON_CTRL0_CLK32_RIF_CTRL, 0x00);

    /* Enable the radio */
    reg_radio_en = REG_SELECT(rf_chain, SX1302_REG_AGC_MCU_RF_EN_A_RADIO_EN, SX1302_REG_AGC_MCU_RF_EN_B_RADIO_EN);
    err |= lgw_reg_w(reg_radio_en, 0x01);

    /* Select the proper reset sequence depending on the radio type */
    reg_radio_rst = REG_SELECT(rf_chain, SX1302_REG_AGC_MCU_RF_EN_A_RADIO_RST, SX1302_REG_AGC_MCU_RF_EN_B_RADIO_RST);
    err |= lgw_reg_w(reg_radio_rst, 0x01);
    wait_ms(500);
    err |= lgw_reg_w(reg_radio_rst, 0x00);
    wait_ms(10);
    switch (type) {
        case LGW_RADIO_TYPE_SX1255:
        case LGW_RADIO_TYPE_SX1257:
            /* Do nothing */
            DEBUG_PRINTF("INFO: reset sx125x (RADIO_%s) done\n", REG_SELECT(rf_chain, "A", "B"));
            break;
        case LGW_RADIO_TYPE_SX1250:
            err |= lgw_reg_w(reg_radio_rst, 0x01);
            wait_ms(10); /* wait for auto calibration to complete */
            DEBUG_PRINTF("INFO: reset sx1250 (RADIO_%s) done\n", REG_SELECT(rf_chain, "A", "B"));
            break;
        default:
            return LGW_REG_ERROR;
    }

    /* Check if something went wrong */
    if (err != LGW_REG_SUCCESS) {
        printf("ERROR: failed to reset the radios\n");
        return LGW_REG_ERROR;
    }

    return LGW_REG_SUCCESS;
}

/* ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ */

int sx1302_radio_set_mode(uint8_t rf_chain, lgw_radio_type_t type) {
    uint16_t reg;
    int err;

    /* Check input parameters */
    if (rf_chain >= LGW_RF_CHAIN_NB) {
        DEBUG_MSG("ERROR: invalid RF chain\n");
        return LGW_REG_ERROR;
    }
    if ((type != LGW_RADIO_TYPE_SX1255) && (type != LGW_RADIO_TYPE_SX1257) && (type != LGW_RADIO_TYPE_SX1250)) {
        DEBUG_MSG("ERROR: invalid radio type\n");
        return LGW_REG_ERROR;
    }

    /* Set the radio mode */
    reg = REG_SELECT(rf_chain,  SX1302_REG_COMMON_CTRL0_SX1261_MODE_RADIO_A,
                                SX1302_REG_COMMON_CTRL0_SX1261_MODE_RADIO_B);
    switch (type) {
        case LGW_RADIO_TYPE_SX1250:
            DEBUG_PRINTF("Setting rf_chain_%u in sx1250 mode\n", rf_chain);
            err = lgw_reg_w(reg, 0x01);
            break;
        default:
            DEBUG_PRINTF("Setting rf_chain_%u in sx125x mode\n", rf_chain);
            err = lgw_reg_w(reg, 0x00);
            break;
    }
    if (err != LGW_REG_SUCCESS) {
        printf("ERROR: failed to set mode for radio %u\n", rf_chain);
        return LGW_REG_ERROR;
    }

    return LGW_REG_SUCCESS;
}

/* ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ */

int sx1302_radio_host_ctrl(bool host_ctrl) {
    return lgw_reg_w(SX1302_REG_COMMON_CTRL0_HOST_RADIO_CTRL, (host_ctrl == false) ? 0x00 : 0x01);
}

/* ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ */

int sx1302_radio_calibrate(struct lgw_conf_rxrf_s * context_rf_chain, uint8_t clksrc, struct lgw_tx_gain_lut_s * txgain_lut) {
    int i;
    int err = LGW_REG_SUCCESS;

    /* -- Reset radios */
    for (i = 0; i < LGW_RF_CHAIN_NB; i++) {
        if (context_rf_chain[i].enable == true) {
            err = sx1302_radio_reset(i, context_rf_chain[i].type);
            if (err != LGW_REG_SUCCESS) {
                printf("ERROR: failed to reset radio %d\n", i);
                return LGW_REG_ERROR;
            }

            err = sx1302_radio_set_mode(i, context_rf_chain[i].type);
            if (err != LGW_REG_SUCCESS) {
                printf("ERROR: failed to set radio %d mode\n", i);
                return LGW_REG_ERROR;
            }
        }
    }
    /* -- Select the radio which provides the clock to the sx1302 */
    err = sx1302_radio_clock_select(clksrc);
    if (err != LGW_REG_SUCCESS) {
        printf("ERROR: failed to get select clock from radio %u\n", clksrc);
        return LGW_REG_ERROR;
    }

    /* -- Ensure PA/LNA are disabled */
    err |= lgw_reg_w(SX1302_REG_AGC_MCU_CTRL_FORCE_HOST_FE_CTRL, 1);
    err |= lgw_reg_w(SX1302_REG_AGC_MCU_RF_EN_A_PA_EN, 0);
    err |= lgw_reg_w(SX1302_REG_AGC_MCU_RF_EN_A_LNA_EN, 0);
    /* -- Start calibration */
    if ((context_rf_chain[clksrc].type == LGW_RADIO_TYPE_SX1257) ||
        (context_rf_chain[clksrc].type == LGW_RADIO_TYPE_SX1255)) {
        DEBUG_MSG("Loading CAL fw for sx125x\n");
        err = sx1302_agc_load_firmware(cal_firmware_sx125x);
        if (err != LGW_REG_SUCCESS) {
            printf("ERROR: Failed to load calibration fw\n");
            return LGW_REG_ERROR;
        }
        err = sx1302_cal_start(FW_VERSION_CAL, context_rf_chain, txgain_lut);
        if (err != LGW_REG_SUCCESS) {
            printf("ERROR: radio calibration failed\n");
            sx1302_radio_reset(0, context_rf_chain[0].type);
            sx1302_radio_reset(1, context_rf_chain[1].type);
            return LGW_REG_ERROR;
        }
    } else {
        DEBUG_MSG("Calibrating sx1250 radios\n");
        for (i = 0; i < LGW_RF_CHAIN_NB; i++) {
            if (context_rf_chain[i].enable == true) {
                err = sx1250_calibrate(i, context_rf_chain[i].freq_hz);
                if (err != LGW_REG_SUCCESS) {
                    printf("ERROR: radio calibration failed\n");
                    return LGW_REG_ERROR;
                }
            }
        }
    }
    /* -- Release control over FE */
    err |= lgw_reg_w(SX1302_REG_AGC_MCU_CTRL_FORCE_HOST_FE_CTRL, 0);

    return err;
}

/* ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ */

int sx1302_pa_lna_lut_configure(struct lgw_conf_board_s * context_board) {
    int err = LGW_REG_SUCCESS;

    /* Configure LUT Table A */
    if (context_board->full_duplex == true) {
        err |= lgw_reg_w(SX1302_REG_AGC_MCU_LUT_TABLE_A_PA_LUT, 0x0C);     /* Enable PA: RADIO_CTRL[2] is high when PA_EN=1 */
        err |= lgw_reg_w(SX1302_REG_AGC_MCU_LUT_TABLE_A_LNA_LUT, 0x0F);    /* Enable LNA: RADIO_CTRL[1] is always high */
    } else {
        err |= lgw_reg_w(SX1302_REG_AGC_MCU_LUT_TABLE_A_PA_LUT, 0x04);     /* Enable PA: RADIO_CTRL[2] is high when PA_EN=1 */
        err |= lgw_reg_w(SX1302_REG_AGC_MCU_LUT_TABLE_A_LNA_LUT, 0x02);    /* Enable LNA: RADIO_CTRL[1] is high when PA_EN=0 & LNA_EN=1 */
    }

    /* Configure LUT Table B */
    err |= lgw_reg_w(SX1302_REG_AGC_MCU_LUT_TABLE_B_PA_LUT, 0x04);         /* Enable PA: RADIO_CTRL[8] is high when PA_EN=1 & LNA_EN=0 */
    err |= lgw_reg_w(SX1302_REG_AGC_MCU_LUT_TABLE_B_LNA_LUT, 0x02);        /* Enable LNA: RADIO_CTRL[7] is high when PA_EN=0 & LNA_EN=1 */

    return err;
}

/* ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ */

int sx1302_radio_fe_configure(void) {
    int err = LGW_REG_SUCCESS;

    err |= lgw_reg_w(SX1302_REG_RADIO_FE_RSSI_BB_FILTER_ALPHA_RADIO_A_RSSI_BB_FILTER_ALPHA, 0x03);
    err |= lgw_reg_w(SX1302_REG_RADIO_FE_RSSI_DEC_FILTER_ALPHA_RADIO_A_RSSI_DEC_FILTER_ALPHA, 0x07);
    err |= lgw_reg_w(SX1302_REG_RADIO_FE_RSSI_BB_FILTER_ALPHA_RADIO_B_RSSI_BB_FILTER_ALPHA, 0x03);
    err |= lgw_reg_w(SX1302_REG_RADIO_FE_RSSI_DEC_FILTER_ALPHA_RADIO_B_RSSI_DEC_FILTER_ALPHA, 0x07);

    err |= lgw_reg_w(SX1302_REG_RADIO_FE_RSSI_DB_DEF_RADIO_A_RSSI_DB_DEFAULT_VALUE, 23);
    err |= lgw_reg_w(SX1302_REG_RADIO_FE_RSSI_DEC_DEF_RADIO_A_RSSI_DEC_DEFAULT_VALUE, 66);
    err |= lgw_reg_w(SX1302_REG_RADIO_FE_RSSI_DB_DEF_RADIO_B_RSSI_DB_DEFAULT_VALUE, 23);
    err |= lgw_reg_w(SX1302_REG_RADIO_FE_RSSI_DEC_DEF_RADIO_B_RSSI_DEC_DEFAULT_VALUE, 66);

    err |= lgw_reg_w(SX1302_REG_RADIO_FE_CTRL0_RADIO_A_DC_NOTCH_EN, 1);
    err |= lgw_reg_w(SX1302_REG_RADIO_FE_CTRL0_RADIO_A_HOST_FILTER_GAIN, 0x0b);
    err |= lgw_reg_w(SX1302_REG_RADIO_FE_CTRL0_RADIO_B_DC_NOTCH_EN, 1);
    err |= lgw_reg_w(SX1302_REG_RADIO_FE_CTRL0_RADIO_B_HOST_FILTER_GAIN, 0x0b);

    return err;
}

/* ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ */

uint8_t sx1302_get_ifmod_config(uint8_t if_chain) {
    return ifmod_config[if_chain];
}

/* ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ */

int sx1302_channelizer_configure(struct lgw_conf_rxif_s * if_cfg, bool fix_gain) {
    int32_t if_freq;
    uint8_t channels_mask = 0x00;
    int i;
    int err = LGW_REG_SUCCESS;

    /* Check input parameters */
    CHECK_NULL(if_cfg);

    /* Select which radio is connected to each multi-SF channel */
    for (i = 0; i < LGW_MULTI_NB; i++) {
        channels_mask |= (if_cfg[i].rf_chain << i);
    }
    DEBUG_PRINTF("LoRa multi-SF radio select: 0x%02X\n", channels_mask);
    err |= lgw_reg_w(SX1302_REG_RX_TOP_RADIO_SELECT_RADIO_SELECT, channels_mask);

    /* Select which radio is connected to the LoRa service channel */
    DEBUG_PRINTF("LoRa service radio select: 0x%02X\n", if_cfg[8].rf_chain);
    err |= lgw_reg_w(SX1302_REG_RX_TOP_LORA_SERVICE_FSK_LORA_SERVICE_RADIO_SEL_RADIO_SELECT, if_cfg[8].rf_chain);

    /* Select which radio is connected to the FSK channel */
    DEBUG_PRINTF("FSK radio select %u\n", if_cfg[9].rf_chain);
    err |= lgw_reg_w(SX1302_REG_RX_TOP_LORA_SERVICE_FSK_FSK_CFG_3_RADIO_SELECT, if_cfg[9].rf_chain);

    /* Configure multi-SF channels IF frequencies */
    if_freq = IF_HZ_TO_REG(if_cfg[0].freq_hz);
    err |= lgw_reg_w(SX1302_REG_RX_TOP_FREQ_0_MSB_IF_FREQ_0, (if_freq >> 8) & 0x0000001F);
    err |= lgw_reg_w(SX1302_REG_RX_TOP_FREQ_0_LSB_IF_FREQ_0, (if_freq >> 0) & 0x000000FF);

    if_freq = IF_HZ_TO_REG(if_cfg[1].freq_hz);
    err |= lgw_reg_w(SX1302_REG_RX_TOP_FREQ_1_MSB_IF_FREQ_1, (if_freq >> 8) & 0x0000001F);
    err |= lgw_reg_w(SX1302_REG_RX_TOP_FREQ_1_LSB_IF_FREQ_1, (if_freq >> 0) & 0x000000FF);

    if_freq = IF_HZ_TO_REG(if_cfg[2].freq_hz);
    err |= lgw_reg_w(SX1302_REG_RX_TOP_FREQ_2_MSB_IF_FREQ_2, (if_freq >> 8) & 0x0000001F);
    err |= lgw_reg_w(SX1302_REG_RX_TOP_FREQ_2_LSB_IF_FREQ_2, (if_freq >> 0) & 0x000000FF);

    if_freq = IF_HZ_TO_REG(if_cfg[3].freq_hz);
    err |= lgw_reg_w(SX1302_REG_RX_TOP_FREQ_3_MSB_IF_FREQ_3, (if_freq >> 8) & 0x0000001F);
    err |= lgw_reg_w(SX1302_REG_RX_TOP_FREQ_3_LSB_IF_FREQ_3, (if_freq >> 0) & 0x000000FF);

    if_freq = IF_HZ_TO_REG(if_cfg[4].freq_hz);
    err |= lgw_reg_w(SX1302_REG_RX_TOP_FREQ_4_MSB_IF_FREQ_4, (if_freq >> 8) & 0x0000001F);
    err |= lgw_reg_w(SX1302_REG_RX_TOP_FREQ_4_LSB_IF_FREQ_4, (if_freq >> 0) & 0x000000FF);

    if_freq = IF_HZ_TO_REG(if_cfg[5].freq_hz);
    err |= lgw_reg_w(SX1302_REG_RX_TOP_FREQ_5_MSB_IF_FREQ_5, (if_freq >> 8) & 0x0000001F);
    err |= lgw_reg_w(SX1302_REG_RX_TOP_FREQ_5_LSB_IF_FREQ_5, (if_freq >> 0) & 0x000000FF);

    if_freq = IF_HZ_TO_REG(if_cfg[6].freq_hz);
    err |= lgw_reg_w(SX1302_REG_RX_TOP_FREQ_6_MSB_IF_FREQ_6, (if_freq >> 8) & 0x0000001F);
    err |= lgw_reg_w(SX1302_REG_RX_TOP_FREQ_6_LSB_IF_FREQ_6, (if_freq >> 0) & 0x000000FF);

    if_freq = IF_HZ_TO_REG(if_cfg[7].freq_hz);
    err |= lgw_reg_w(SX1302_REG_RX_TOP_FREQ_7_MSB_IF_FREQ_7, (if_freq >> 8) & 0x0000001F);
    err |= lgw_reg_w(SX1302_REG_RX_TOP_FREQ_7_LSB_IF_FREQ_7, (if_freq >> 0) & 0x000000FF);

    /* Configure LoRa service channel IF frequency */
    if_freq = IF_HZ_TO_REG(if_cfg[8].freq_hz);
    err |= lgw_reg_w(SX1302_REG_RX_TOP_LORA_SERVICE_FSK_LORA_SERVICE_FREQ_MSB_IF_FREQ_0, (if_freq >> 8) & 0x0000001F);
    err |= lgw_reg_w(SX1302_REG_RX_TOP_LORA_SERVICE_FSK_LORA_SERVICE_FREQ_LSB_IF_FREQ_0, (if_freq >> 0) & 0x000000FF);

    /* Configure FSK channel IF frequency */
    if_freq = IF_HZ_TO_REG(if_cfg[9].freq_hz);
    err |= lgw_reg_w(SX1302_REG_RX_TOP_LORA_SERVICE_FSK_FSK_FREQ_MSB_IF_FREQ_0, (if_freq >> 8) & 0x0000001F);
    err |= lgw_reg_w(SX1302_REG_RX_TOP_LORA_SERVICE_FSK_FSK_FREQ_LSB_IF_FREQ_0, (if_freq >> 0) & 0x000000FF);

    /* Set the low pass filtering corner frequency for RSSI indicator */
    err |= lgw_reg_w(SX1302_REG_RX_TOP_RSSI_CONTROL_RSSI_FILTER_ALPHA, 0x05);

    /* Set the channelizer RSSI reset value */
    err |= lgw_reg_w(SX1302_REG_RX_TOP_RSSI_DEF_VALUE_CHAN_RSSI_DEF_VALUE, 85);

    /* Force channelizer in fix gain, or let it be controlled by AGC */
    if (fix_gain == true) {
        err |= lgw_reg_w(SX1302_REG_RX_TOP_CHANN_DAGC_CFG5_CHAN_DAGC_MODE, 0x00);
        err |= lgw_reg_w(SX1302_REG_RX_TOP_GAIN_CONTROL_CHAN_GAIN, 5);
    } else {
        /* Allow the AGC to control gains */
        err |= lgw_reg_w(SX1302_REG_RX_TOP_CHANN_DAGC_CFG5_CHAN_DAGC_MODE, 0x01);
        /* Disable the internal DAGC */
        err |= lgw_reg_w(SX1302_REG_RX_TOP_CHANN_DAGC_CFG1_CHAN_DAGC_THRESHOLD_HIGH, 255 );
        err |= lgw_reg_w(SX1302_REG_RX_TOP_CHANN_DAGC_CFG2_CHAN_DAGC_THRESHOLD_LOW, 0 );
        err |= lgw_reg_w(SX1302_REG_RX_TOP_CHANN_DAGC_CFG3_CHAN_DAGC_MAX_ATTEN, 15 );
        err |= lgw_reg_w(SX1302_REG_RX_TOP_CHANN_DAGC_CFG3_CHAN_DAGC_MIN_ATTEN, 0 );
    }

    return err;
}

/* ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ */

int sx1302_fsk_configure(struct lgw_conf_rxif_s * cfg) {
    uint64_t fsk_sync_word_reg;
    uint32_t fsk_br_reg;
    int err = LGW_REG_SUCCESS;

    DEBUG_PRINTF("FSK: syncword:0x%" PRIx64 ", syncword_size:%u\n", cfg->sync_word, cfg->sync_word_size);

    err |= lgw_reg_w(SX1302_REG_RX_TOP_LORA_SERVICE_FSK_FSK_CFG_1_PSIZE, cfg->sync_word_size - 1);
    fsk_sync_word_reg = cfg->sync_word << (8 * (8 - cfg->sync_word_size));
    err |= lgw_reg_w(SX1302_REG_RX_TOP_LORA_SERVICE_FSK_FSK_REF_PATTERN_BYTE0_FSK_REF_PATTERN, (uint8_t)(fsk_sync_word_reg >> 0));
    err |= lgw_reg_w(SX1302_REG_RX_TOP_LORA_SERVICE_FSK_FSK_REF_PATTERN_BYTE1_FSK_REF_PATTERN, (uint8_t)(fsk_sync_word_reg >> 8));
    err |= lgw_reg_w(SX1302_REG_RX_TOP_LORA_SERVICE_FSK_FSK_REF_PATTERN_BYTE2_FSK_REF_PATTERN, (uint8_t)(fsk_sync_word_reg >> 16));
    err |= lgw_reg_w(SX1302_REG_RX_TOP_LORA_SERVICE_FSK_FSK_REF_PATTERN_BYTE3_FSK_REF_PATTERN, (uint8_t)(fsk_sync_word_reg >> 24));
    err |= lgw_reg_w(SX1302_REG_RX_TOP_LORA_SERVICE_FSK_FSK_REF_PATTERN_BYTE4_FSK_REF_PATTERN, (uint8_t)(fsk_sync_word_reg >> 32));
    err |= lgw_reg_w(SX1302_REG_RX_TOP_LORA_SERVICE_FSK_FSK_REF_PATTERN_BYTE5_FSK_REF_PATTERN, (uint8_t)(fsk_sync_word_reg >> 40));
    err |= lgw_reg_w(SX1302_REG_RX_TOP_LORA_SERVICE_FSK_FSK_REF_PATTERN_BYTE6_FSK_REF_PATTERN, (uint8_t)(fsk_sync_word_reg >> 48));
    err |= lgw_reg_w(SX1302_REG_RX_TOP_LORA_SERVICE_FSK_FSK_REF_PATTERN_BYTE7_FSK_REF_PATTERN, (uint8_t)(fsk_sync_word_reg >> 56));

    fsk_br_reg = 32000000 / cfg->datarate;
    err |= lgw_reg_w(SX1302_REG_RX_TOP_LORA_SERVICE_FSK_BIT_RATE_MSB_BIT_RATE, (uint8_t)(fsk_br_reg >> 8));
    err |= lgw_reg_w(SX1302_REG_RX_TOP_LORA_SERVICE_FSK_BIT_RATE_LSB_BIT_RATE, (uint8_t)(fsk_br_reg >> 0));
    err |= lgw_reg_w(SX1302_REG_RX_TOP_LORA_SERVICE_FSK_FSK_CFG_1_CH_BW_EXPO, 0x03); /* 125KHz */

    err |= lgw_reg_w(SX1302_REG_RX_TOP_LORA_SERVICE_FSK_FSK_CFG_3_RX_INVERT, 0);
    err |= lgw_reg_w(SX1302_REG_RX_TOP_LORA_SERVICE_FSK_FSK_CFG_3_MODEM_INVERT_IQ, 0);

    err |= lgw_reg_w(SX1302_REG_RX_TOP_LORA_SERVICE_FSK_FSK_CFG_4_RSSI_LENGTH, 4);
    err |= lgw_reg_w(SX1302_REG_RX_TOP_LORA_SERVICE_FSK_FSK_CFG_0_PKT_MODE, 1); /* variable length */
    err |= lgw_reg_w(SX1302_REG_RX_TOP_LORA_SERVICE_FSK_FSK_CFG_0_CRC_EN, 1);
    err |= lgw_reg_w(SX1302_REG_RX_TOP_LORA_SERVICE_FSK_FSK_CFG_0_DCFREE_ENC, 2);
    err |= lgw_reg_w(SX1302_REG_RX_TOP_LORA_SERVICE_FSK_FSK_CFG_0_CRC_IBM, 0);
    err |= lgw_reg_w(SX1302_REG_RX_TOP_LORA_SERVICE_FSK_FSK_CFG_4_ERROR_OSR_TOL, 10);
    err |= lgw_reg_w(SX1302_REG_RX_TOP_LORA_SERVICE_FSK_FSK_PKT_LENGTH_PKT_LENGTH, 255);
    err |= lgw_reg_w(SX1302_REG_RX_TOP_LORA_SERVICE_FSK_FSK_NODE_ADRS_NODE_ADRS, 0);
    err |= lgw_reg_w(SX1302_REG_RX_TOP_LORA_SERVICE_FSK_FSK_BROADCAST_BROADCAST, 0);
    err |= lgw_reg_w(SX1302_REG_RX_TOP_LORA_SERVICE_FSK_FSK_CFG_3_AUTO_AFC, 1); /* ?? */
    err |= lgw_reg_w(SX1302_REG_RX_TOP_LORA_SERVICE_FSK_FSK_TIMEOUT_MSB_TIMEOUT, 0);
    err |= lgw_reg_w(SX1302_REG_RX_TOP_LORA_SERVICE_FSK_FSK_TIMEOUT_LSB_TIMEOUT, 128);

    return err;
}

/* ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ */

int sx1302_lora_correlator_configure(struct lgw_conf_rxif_s * if_cfg, struct lgw_conf_demod_s * demod_cfg) {
    int i, err = LGW_REG_SUCCESS;
    uint8_t channels_mask = 0x00;

    /* Check input parameters */
    CHECK_NULL(if_cfg);

    err |= lgw_reg_w(SX1302_REG_RX_TOP_SF5_CFG2_ACC_PNR, 52);
    err |= lgw_reg_w(SX1302_REG_RX_TOP_SF5_CFG4_MSP_PNR, 24);
    err |= lgw_reg_w(SX1302_REG_RX_TOP_SF5_CFG6_MSP_PEAK_NB, 7);
    err |= lgw_reg_w(SX1302_REG_RX_TOP_SF5_CFG7_MSP2_PEAK_NB, 5);

    err |= lgw_reg_w(SX1302_REG_RX_TOP_SF6_CFG2_ACC_PNR, 52);
    err |= lgw_reg_w(SX1302_REG_RX_TOP_SF6_CFG4_MSP_PNR, 24);
    err |= lgw_reg_w(SX1302_REG_RX_TOP_SF6_CFG6_MSP_PEAK_NB, 7);
    err |= lgw_reg_w(SX1302_REG_RX_TOP_SF6_CFG7_MSP2_PEAK_NB, 5);

    err |= lgw_reg_w(SX1302_REG_RX_TOP_SF7_CFG2_ACC_PNR, 52);
    err |= lgw_reg_w(SX1302_REG_RX_TOP_SF7_CFG4_MSP_PNR, 24);
    err |= lgw_reg_w(SX1302_REG_RX_TOP_SF7_CFG6_MSP_PEAK_NB, 7);
    err |= lgw_reg_w(SX1302_REG_RX_TOP_SF7_CFG7_MSP2_PEAK_NB, 5);

    err |= lgw_reg_w(SX1302_REG_RX_TOP_SF8_CFG2_ACC_PNR, 52);
    err |= lgw_reg_w(SX1302_REG_RX_TOP_SF8_CFG4_MSP_PNR, 24);
    err |= lgw_reg_w(SX1302_REG_RX_TOP_SF8_CFG6_MSP_PEAK_NB, 7);
    err |= lgw_reg_w(SX1302_REG_RX_TOP_SF8_CFG7_MSP2_PEAK_NB, 5);

    err |= lgw_reg_w(SX1302_REG_RX_TOP_SF9_CFG2_ACC_PNR, 52);
    err |= lgw_reg_w(SX1302_REG_RX_TOP_SF9_CFG4_MSP_PNR, 24);
    err |= lgw_reg_w(SX1302_REG_RX_TOP_SF9_CFG6_MSP_PEAK_NB, 7);
    err |= lgw_reg_w(SX1302_REG_RX_TOP_SF9_CFG7_MSP2_PEAK_NB, 5);

    err |= lgw_reg_w(SX1302_REG_RX_TOP_SF10_CFG2_ACC_PNR, 52);
    err |= lgw_reg_w(SX1302_REG_RX_TOP_SF10_CFG4_MSP_PNR, 24);
    err |= lgw_reg_w(SX1302_REG_RX_TOP_SF10_CFG6_MSP_PEAK_NB, 7);
    err |= lgw_reg_w(SX1302_REG_RX_TOP_SF10_CFG7_MSP2_PEAK_NB, 5);

    err |= lgw_reg_w(SX1302_REG_RX_TOP_SF11_CFG2_ACC_PNR, 52);
    err |= lgw_reg_w(SX1302_REG_RX_TOP_SF11_CFG4_MSP_PNR, 24);
    err |= lgw_reg_w(SX1302_REG_RX_TOP_SF11_CFG6_MSP_PEAK_NB, 7);
    err |= lgw_reg_w(SX1302_REG_RX_TOP_SF11_CFG7_MSP2_PEAK_NB, 5);

    err |= lgw_reg_w(SX1302_REG_RX_TOP_SF12_CFG2_ACC_PNR, 52);
    err |= lgw_reg_w(SX1302_REG_RX_TOP_SF12_CFG4_MSP_PNR, 24);
    err |= lgw_reg_w(SX1302_REG_RX_TOP_SF12_CFG6_MSP_PEAK_NB, 7);
    err |= lgw_reg_w(SX1302_REG_RX_TOP_SF12_CFG7_MSP2_PEAK_NB, 5);

    err |= lgw_reg_w(SX1302_REG_RX_TOP_CORRELATOR_ENABLE_ONLY_FIRST_DET_EDGE_ENABLE_ONLY_FIRST_DET_EDGE, 0xFF);
    err |= lgw_reg_w(SX1302_REG_RX_TOP_CORRELATOR_ENABLE_ACC_CLEAR_ENABLE_CORR_ACC_CLEAR, 0xFF);

    /* Enabled selected spreading factors */
    err |= lgw_reg_w(SX1302_REG_RX_TOP_CORRELATOR_SF_EN_CORR_SF_EN, demod_cfg->multisf_datarate);
    DEBUG_PRINTF("INFO: LoRa multi-SF correlator SF enable mask: 0x%02X\n", demod_cfg->multisf_datarate);

    /* Enable correlator if channel is enabled (1 correlator per channel) */
    for (i = 0; i < LGW_MULTI_NB; i++) {
        channels_mask |= (if_cfg[i].enable << i);
    }
    DEBUG_PRINTF("INFO: LoRa multi-SF channel enable mask: 0x%02X\n", channels_mask);
    err |= lgw_reg_w(SX1302_REG_RX_TOP_CORR_CLOCK_ENABLE_CLK_EN, channels_mask);
    err |= lgw_reg_w(SX1302_REG_RX_TOP_CORRELATOR_EN_CORR_EN, channels_mask);

    /* For debug: get packets with sync_error and header_error in FIFO */
#if 0
    err |= lgw_reg_w(SX1302_REG_RX_TOP_RX_BUFFER_STORE_SYNC_FAIL_META, 0x01);
    err |= lgw_reg_w(SX1302_REG_RX_TOP_RX_BUFFER_STORE_HEADER_ERR_META, 0x01);
#endif

    return err;
}

/* ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ */

int sx1302_lora_service_correlator_configure(struct lgw_conf_rxif_s * cfg) {
    int err = LGW_REG_SUCCESS;

    /* Common config for all SF */
    err |= lgw_reg_w(SX1302_REG_RX_TOP_LORA_SERVICE_FSK_DETECT_MSP2_MSP_PEAK_NB, 7);
    err |= lgw_reg_w(SX1302_REG_RX_TOP_LORA_SERVICE_FSK_DETECT_MSP2_MSP2_PEAK_NB, 5);
    err |= lgw_reg_w(SX1302_REG_RX_TOP_LORA_SERVICE_FSK_DETECT_ACC1_USE_GAIN_SYMB, 1);

    switch (cfg->datarate) {
        case 5:
            err |= lgw_reg_w(SX1302_REG_RX_TOP_LORA_SERVICE_FSK_TXRX_CFG2_FINE_SYNCH_EN, 1);
            err |= lgw_reg_w(SX1302_REG_RX_TOP_LORA_SERVICE_FSK_DETECT_ACC1_ACC_PNR, 52);
            break;
        case 6:
            err |= lgw_reg_w(SX1302_REG_RX_TOP_LORA_SERVICE_FSK_TXRX_CFG2_FINE_SYNCH_EN, 1);
            err |= lgw_reg_w(SX1302_REG_RX_TOP_LORA_SERVICE_FSK_DETECT_ACC1_ACC_PNR, 52);
            break;
        case 7:
            err |= lgw_reg_w(SX1302_REG_RX_TOP_LORA_SERVICE_FSK_TXRX_CFG2_FINE_SYNCH_EN, 0);
            err |= lgw_reg_w(SX1302_REG_RX_TOP_LORA_SERVICE_FSK_DETECT_ACC1_ACC_PNR, 52);
            break;
        case 8:
            err |= lgw_reg_w(SX1302_REG_RX_TOP_LORA_SERVICE_FSK_TXRX_CFG2_FINE_SYNCH_EN, 0);
            err |= lgw_reg_w(SX1302_REG_RX_TOP_LORA_SERVICE_FSK_DETECT_ACC1_ACC_PNR, 52);
            break;
        case 9:
            err |= lgw_reg_w(SX1302_REG_RX_TOP_LORA_SERVICE_FSK_TXRX_CFG2_FINE_SYNCH_EN, 0);
            err |= lgw_reg_w(SX1302_REG_RX_TOP_LORA_SERVICE_FSK_DETECT_ACC1_ACC_PNR, 52);
            break;
        case 10:
            err |= lgw_reg_w(SX1302_REG_RX_TOP_LORA_SERVICE_FSK_TXRX_CFG2_FINE_SYNCH_EN, 0);
            err |= lgw_reg_w(SX1302_REG_RX_TOP_LORA_SERVICE_FSK_DETECT_ACC1_ACC_PNR, 52);
            break;
        case 11:
            err |= lgw_reg_w(SX1302_REG_RX_TOP_LORA_SERVICE_FSK_TXRX_CFG2_FINE_SYNCH_EN, 0);
            err |= lgw_reg_w(SX1302_REG_RX_TOP_LORA_SERVICE_FSK_DETECT_ACC1_ACC_PNR, 52);
            break;
        case 12:
            err |= lgw_reg_w(SX1302_REG_RX_TOP_LORA_SERVICE_FSK_TXRX_CFG2_FINE_SYNCH_EN, 0);
            err |= lgw_reg_w(SX1302_REG_RX_TOP_LORA_SERVICE_FSK_DETECT_ACC1_ACC_PNR, 52);
            break;
        default:
            printf("ERROR: Failed to configure LoRa service modem correlators\n");
            return LGW_REG_ERROR;
    }

    return err;
}

/* ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ */

int sx1302_lora_modem_configure(uint32_t radio_freq_hz) {
    uint16_t mantissa = 0;
    uint8_t exponent = 0;
    int err = LGW_REG_SUCCESS;

    err |= lgw_reg_w(SX1302_REG_RX_TOP_DC_NOTCH_CFG1_ENABLE, 0x00);
    err |= lgw_reg_w(SX1302_REG_RX_TOP_RX_DFE_AGC1_FORCE_DEFAULT_FIR, 0x01);
    err |= lgw_reg_w(SX1302_REG_RX_TOP_DAGC_CFG_GAIN_DROP_COMP, 0x01);
    err |= lgw_reg_w(SX1302_REG_RX_TOP_DAGC_CFG_TARGET_LVL, 0x01);

    /* Enable full modems */
    DEBUG_MSG("Configuring 8 full-SF modems\n");
    err |= lgw_reg_w(SX1302_REG_OTP_MODEM_EN_0_MODEM_EN, 0xFF);

    /* Enable limited modems */
    DEBUG_MSG("Configuring 8 limited-SF modems\n");
    err |= lgw_reg_w(SX1302_REG_OTP_MODEM_EN_1_MODEM_EN, 0xFF);

    /* Configure coarse sync between correlators and modems */
    err |= lgw_reg_w(SX1302_REG_RX_TOP_MODEM_SYNC_DELTA_MSB_MODEM_SYNC_DELTA, 0);
    err |= lgw_reg_w(SX1302_REG_RX_TOP_MODEM_SYNC_DELTA_LSB_MODEM_SYNC_DELTA, 126);

    /* Configure fine sync offset for each channel */
    err |= lgw_reg_w(SX1302_REG_ARB_MCU_CHANNEL_SYNC_OFFSET_01_CHANNEL_0_OFFSET, 1);
    err |= lgw_reg_w(SX1302_REG_ARB_MCU_CHANNEL_SYNC_OFFSET_01_CHANNEL_1_OFFSET, 5);
    err |= lgw_reg_w(SX1302_REG_ARB_MCU_CHANNEL_SYNC_OFFSET_23_CHANNEL_2_OFFSET, 9);
    err |= lgw_reg_w(SX1302_REG_ARB_MCU_CHANNEL_SYNC_OFFSET_23_CHANNEL_3_OFFSET, 13);
    err |= lgw_reg_w(SX1302_REG_ARB_MCU_CHANNEL_SYNC_OFFSET_45_CHANNEL_4_OFFSET, 1);
    err |= lgw_reg_w(SX1302_REG_ARB_MCU_CHANNEL_SYNC_OFFSET_45_CHANNEL_5_OFFSET, 5);
    err |= lgw_reg_w(SX1302_REG_ARB_MCU_CHANNEL_SYNC_OFFSET_67_CHANNEL_6_OFFSET, 9);
    err |= lgw_reg_w(SX1302_REG_ARB_MCU_CHANNEL_SYNC_OFFSET_67_CHANNEL_7_OFFSET, 13);

    /* Configure PPM offset */
    err |= lgw_reg_w(SX1302_REG_RX_TOP_MODEM_PPM_OFFSET1_PPM_OFFSET_SF5, 0x00);
    err |= lgw_reg_w(SX1302_REG_RX_TOP_MODEM_PPM_OFFSET1_PPM_OFFSET_SF6, 0x00);
    err |= lgw_reg_w(SX1302_REG_RX_TOP_MODEM_PPM_OFFSET1_PPM_OFFSET_SF7, 0x00);
    err |= lgw_reg_w(SX1302_REG_RX_TOP_MODEM_PPM_OFFSET1_PPM_OFFSET_SF8, 0x00);
    err |= lgw_reg_w(SX1302_REG_RX_TOP_MODEM_PPM_OFFSET2_PPM_OFFSET_SF9, 0x00);
    err |= lgw_reg_w(SX1302_REG_RX_TOP_MODEM_PPM_OFFSET2_PPM_OFFSET_SF10, 0x00);
    err |= lgw_reg_w(SX1302_REG_RX_TOP_MODEM_PPM_OFFSET2_PPM_OFFSET_SF11, 0x01);
    err |= lgw_reg_w(SX1302_REG_RX_TOP_MODEM_PPM_OFFSET2_PPM_OFFSET_SF12, 0x01);

    /* Improve SF5 and SF6 performances */
    err |= lgw_reg_w(SX1302_REG_RX_TOP_FINE_TIMING_A_1_GAIN_P_AUTO, 3); // Default is 1
    err |= lgw_reg_w(SX1302_REG_RX_TOP_FINE_TIMING_A_1_GAIN_P_PAYLOAD, 3); // Default is 2

    /* Improve SF11/SF12 performances */
    err |= lgw_reg_w(SX1302_REG_RX_TOP_FINE_TIMING_A_5_GAIN_I_EN_SF11, 1);
    err |= lgw_reg_w(SX1302_REG_RX_TOP_FINE_TIMING_A_5_GAIN_I_EN_SF12, 1);

    /* Set threshold for 1bin correction (CAN-314) */
    err |= lgw_reg_w(SX1302_REG_RX_TOP_FREQ_TRACK4_FREQ_SYNCH_THR, 15);

    /* Configure modems for best tracking (best demodulation) */
    err |= lgw_reg_w(SX1302_REG_RX_TOP_FREQ_TRACK_A_0_FREQ_TRACK_EN_SF5, RX_FREQ_TRACK_AUTO);
    err |= lgw_reg_w(SX1302_REG_RX_TOP_FREQ_TRACK_A_0_FREQ_TRACK_EN_SF6, RX_FREQ_TRACK_AUTO);
    err |= lgw_reg_w(SX1302_REG_RX_TOP_FREQ_TRACK_A_0_FREQ_TRACK_EN_SF7, RX_FREQ_TRACK_AUTO);
    err |= lgw_reg_w(SX1302_REG_RX_TOP_FREQ_TRACK_A_0_FREQ_TRACK_EN_SF8, RX_FREQ_TRACK_AUTO);
    err |= lgw_reg_w(SX1302_REG_RX_TOP_FREQ_TRACK_A_1_FREQ_TRACK_EN_SF9, RX_FREQ_TRACK_AUTO);
    err |= lgw_reg_w(SX1302_REG_RX_TOP_FREQ_TRACK_A_1_FREQ_TRACK_EN_SF10, RX_FREQ_TRACK_AUTO);
    err |= lgw_reg_w(SX1302_REG_RX_TOP_FREQ_TRACK_A_1_FREQ_TRACK_EN_SF11, RX_FREQ_TRACK_AUTO);
    err |= lgw_reg_w(SX1302_REG_RX_TOP_FREQ_TRACK_A_1_FREQ_TRACK_EN_SF12, RX_FREQ_TRACK_AUTO);

    /* Configure modems for best timestamping (only valid when double demodulation is enabled) */
    err |= lgw_reg_w(SX1302_REG_RX_TOP_FREQ_TRACK_B_0_FREQ_TRACK_EN_SF5, RX_FREQ_TRACK_OFF);
    err |= lgw_reg_w(SX1302_REG_RX_TOP_FREQ_TRACK_B_0_FREQ_TRACK_EN_SF6, RX_FREQ_TRACK_OFF);
    err |= lgw_reg_w(SX1302_REG_RX_TOP_FREQ_TRACK_B_0_FREQ_TRACK_EN_SF7, RX_FREQ_TRACK_OFF);
    err |= lgw_reg_w(SX1302_REG_RX_TOP_FREQ_TRACK_B_0_FREQ_TRACK_EN_SF8, RX_FREQ_TRACK_OFF);
    err |= lgw_reg_w(SX1302_REG_RX_TOP_FREQ_TRACK_B_1_FREQ_TRACK_EN_SF9, RX_FREQ_TRACK_OFF);
    err |= lgw_reg_w(SX1302_REG_RX_TOP_FREQ_TRACK_B_1_FREQ_TRACK_EN_SF10, RX_FREQ_TRACK_OFF);
    err |= lgw_reg_w(SX1302_REG_RX_TOP_FREQ_TRACK_B_1_FREQ_TRACK_EN_SF11, RX_FREQ_TRACK_OFF);
    err |= lgw_reg_w(SX1302_REG_RX_TOP_FREQ_TRACK_B_1_FREQ_TRACK_EN_SF12, RX_FREQ_TRACK_OFF);
    /* -- */
    err |= lgw_reg_w(SX1302_REG_RX_TOP_FINE_TIMING_B_5_GAIN_I_EN_SF11, 0);
    err |= lgw_reg_w(SX1302_REG_RX_TOP_FINE_TIMING_B_5_GAIN_I_EN_SF12, 0);
    err |= lgw_reg_w(SX1302_REG_RX_TOP_FINE_TIMING_B_0_ROUNDING, 1);
    err |= lgw_reg_w(SX1302_REG_RX_TOP_FINE_TIMING_B_0_MODE, RX_FINE_TIMING_MODE_LINEAR);
    /* -- */
    err |= lgw_reg_w(SX1302_REG_RX_TOP_FINE_TIMING_B_1_GAIN_P_AUTO, 0);
    err |= lgw_reg_w(SX1302_REG_RX_TOP_FINE_TIMING_B_1_GAIN_P_PREAMB, 6);
    err |= lgw_reg_w(SX1302_REG_RX_TOP_FINE_TIMING_B_1_GAIN_P_PAYLOAD, 2);
    /* -- */
    err |= lgw_reg_w(SX1302_REG_RX_TOP_FINE_TIMING_B_2_GAIN_I_AUTO, 0);
    err |= lgw_reg_w(SX1302_REG_RX_TOP_FINE_TIMING_B_2_GAIN_I_PREAMB, 1);
    err |= lgw_reg_w(SX1302_REG_RX_TOP_FINE_TIMING_B_2_GAIN_I_PAYLOAD, 0);

    /* Set preamble size to 10 (to handle 12 for SF5/SF6 and 8 for SF7->SF12) */
    err |= lgw_reg_w(SX1302_REG_RX_TOP_TXRX_CFG7_PREAMBLE_SYMB_NB,  0); /* MSB */
    err |= lgw_reg_w(SX1302_REG_RX_TOP_TXRX_CFG6_PREAMBLE_SYMB_NB, 10); /* LSB */

    /* Freq2TimeDrift computation */
    if (calculate_freq_to_time_drift(radio_freq_hz, BW_125KHZ, &mantissa, &exponent) != 0) {
        printf("ERROR: failed to calculate frequency to time drift for LoRa modem\n");
        return LGW_REG_ERROR;
    }
    DEBUG_PRINTF("Freq2TimeDrift MultiSF: Mantissa = %d (0x%02X, 0x%02X), Exponent = %d (0x%02X)\n", mantissa, (mantissa >> 8) & 0x00FF, (mantissa) & 0x00FF, exponent, exponent);
    err |= lgw_reg_w(SX1302_REG_RX_TOP_FREQ_TO_TIME0_FREQ_TO_TIME_DRIFT_MANT, (mantissa >> 8) & 0x00FF);
    err |= lgw_reg_w(SX1302_REG_RX_TOP_FREQ_TO_TIME1_FREQ_TO_TIME_DRIFT_MANT, (mantissa) & 0x00FF);
    err |= lgw_reg_w(SX1302_REG_RX_TOP_FREQ_TO_TIME2_FREQ_TO_TIME_DRIFT_EXP, exponent);

    /* Time drift compensation */
    err |= lgw_reg_w(SX1302_REG_RX_TOP_FREQ_TO_TIME3_FREQ_TO_TIME_INVERT_TIME_SYMB, 1);

    /* DFT peak mode : set to AUTO, check timestamp_counter_correction() if changed */
    err |= lgw_reg_w(SX1302_REG_RX_TOP_RX_CFG0_DFT_PEAK_EN, RX_DFT_PEAK_MODE_AUTO);

    return err;
}

/* ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ */

int sx1302_lora_service_modem_configure(struct lgw_conf_rxif_s * cfg, uint32_t radio_freq_hz) {
    uint16_t mantissa = 0;
    uint8_t exponent = 0;
    uint8_t preamble_nb_symb;
    int err = LGW_REG_SUCCESS;

    err |= lgw_reg_w(SX1302_REG_RX_TOP_LORA_SERVICE_FSK_DC_NOTCH_CFG1_ENABLE, 0x00);
    err |= lgw_reg_w(SX1302_REG_RX_TOP_LORA_SERVICE_FSK_RX_DFE_AGC1_FORCE_DEFAULT_FIR, 0x01);
    err |= lgw_reg_w(SX1302_REG_RX_TOP_LORA_SERVICE_FSK_DAGC_CFG_GAIN_DROP_COMP, 0x01);
    err |= lgw_reg_w(SX1302_REG_RX_TOP_LORA_SERVICE_FSK_DAGC_CFG_TARGET_LVL, 0x01);

    err |= lgw_reg_w(SX1302_REG_RX_TOP_LORA_SERVICE_FSK_FINE_TIMING1_GAIN_P_AUTO, 0x03);
    err |= lgw_reg_w(SX1302_REG_RX_TOP_LORA_SERVICE_FSK_FINE_TIMING2_GAIN_I_PAYLOAD, 0x03);

    switch (cfg->datarate) {
        case DR_LORA_SF5:
        case DR_LORA_SF6:
            err |= lgw_reg_w(SX1302_REG_RX_TOP_LORA_SERVICE_FSK_FINE_TIMING1_GAIN_P_PREAMB, 0x04); // Default value
            err |= lgw_reg_w(SX1302_REG_RX_TOP_LORA_SERVICE_FSK_FINE_TIMING2_GAIN_I_EN, 0x00); // Default value
            break;
        case DR_LORA_SF7:
        case DR_LORA_SF8:
        case DR_LORA_SF9:
        case DR_LORA_SF10:
            err |= lgw_reg_w(SX1302_REG_RX_TOP_LORA_SERVICE_FSK_FINE_TIMING1_GAIN_P_PREAMB, 0x06);
            err |= lgw_reg_w(SX1302_REG_RX_TOP_LORA_SERVICE_FSK_FINE_TIMING2_GAIN_I_EN, 0x00);
            break;
        case DR_LORA_SF11:
        case DR_LORA_SF12:
            err |= lgw_reg_w(SX1302_REG_RX_TOP_LORA_SERVICE_FSK_FINE_TIMING1_GAIN_P_PREAMB, 0x07);
            switch (cfg->bandwidth) {
                case BW_125KHZ:
                    err |= lgw_reg_w(SX1302_REG_RX_TOP_LORA_SERVICE_FSK_FINE_TIMING2_GAIN_I_EN, 0x01);
                    break;
                case BW_250KHZ:
                    err |= lgw_reg_w(SX1302_REG_RX_TOP_LORA_SERVICE_FSK_FINE_TIMING2_GAIN_I_EN, 0x02);
                    break;
                case BW_500KHZ:
                    err |= lgw_reg_w(SX1302_REG_RX_TOP_LORA_SERVICE_FSK_FINE_TIMING2_GAIN_I_EN, 0x03);
                    break;
                default:
                    printf("ERROR: unsupported bandwidth %u for LoRa Service modem\n", cfg->bandwidth);
                    break;
            }
            break;
        default:
            printf("ERROR: unsupported datarate %u for LoRa Service modem\n", cfg->datarate);
            break;
    }

    err |= lgw_reg_w(SX1302_REG_RX_TOP_LORA_SERVICE_FSK_TXRX_CFG2_IMPLICIT_HEADER, (cfg->implicit_hdr == true) ? 1 : 0);
    err |= lgw_reg_w(SX1302_REG_RX_TOP_LORA_SERVICE_FSK_TXRX_CFG2_CRC_EN, (cfg->implicit_crc_en == true) ? 1 : 0);
    err |= lgw_reg_w(SX1302_REG_RX_TOP_LORA_SERVICE_FSK_TXRX_CFG1_CODING_RATE, cfg->implicit_coderate);
    err |= lgw_reg_w(SX1302_REG_RX_TOP_LORA_SERVICE_FSK_TXRX_CFG3_PAYLOAD_LENGTH, cfg->implicit_payload_length);

    err |= lgw_reg_w(SX1302_REG_RX_TOP_LORA_SERVICE_FSK_TXRX_CFG0_MODEM_SF, cfg->datarate);
    err |= lgw_reg_w(SX1302_REG_RX_TOP_LORA_SERVICE_FSK_TXRX_CFG0_MODEM_BW, cfg->bandwidth);
    err |= lgw_reg_w(SX1302_REG_RX_TOP_LORA_SERVICE_FSK_TXRX_CFG1_PPM_OFFSET, SET_PPM_ON(cfg->bandwidth, cfg->datarate));

    /* Set preamble size to 8 for SF7->SF12 and to 12 for SF5->SF6 (aligned with end-device drivers) */
    if ((cfg->datarate == DR_LORA_SF5) || (cfg->datarate == DR_LORA_SF6)) {
        preamble_nb_symb = 12;
    } else {
        preamble_nb_symb = 8;
    }
    printf("INFO: LoRa Service modem: configuring preamble size to %u symbols\n", preamble_nb_symb);
    err |= lgw_reg_w(SX1302_REG_RX_TOP_LORA_SERVICE_FSK_TXRX_CFG7_PREAMBLE_SYMB_NB, (preamble_nb_symb >> 8) & 0xFF); /* MSB */
    err |= lgw_reg_w(SX1302_REG_RX_TOP_LORA_SERVICE_FSK_TXRX_CFG6_PREAMBLE_SYMB_NB, (preamble_nb_symb >> 0) & 0xFF); /* LSB */

    /* Freq2TimeDrift computation */
    if (calculate_freq_to_time_drift(radio_freq_hz, cfg->bandwidth, &mantissa, &exponent) != 0) {
        printf("ERROR: failed to calculate frequency to time drift for LoRa service modem\n");
        return LGW_REG_ERROR;
    }
    err |= lgw_reg_w(SX1302_REG_RX_TOP_LORA_SERVICE_FSK_FREQ_TO_TIME0_FREQ_TO_TIME_DRIFT_MANT, (mantissa >> 8) & 0x00FF);
    err |= lgw_reg_w(SX1302_REG_RX_TOP_LORA_SERVICE_FSK_FREQ_TO_TIME1_FREQ_TO_TIME_DRIFT_MANT, (mantissa) & 0x00FF);
    err |= lgw_reg_w(SX1302_REG_RX_TOP_LORA_SERVICE_FSK_FREQ_TO_TIME2_FREQ_TO_TIME_DRIFT_EXP, exponent);
    DEBUG_PRINTF("Freq2TimeDrift SingleSF: Mantissa = %d (0x%02X, 0x%02X), Exponent = %d (0x%02X)\n", mantissa, (mantissa >> 8) & 0x00FF, (mantissa) & 0x00FF, exponent, exponent);

    /* Time drift compensation */
    err |= lgw_reg_w(SX1302_REG_RX_TOP_LORA_SERVICE_FSK_FREQ_TO_TIME3_FREQ_TO_TIME_INVERT_TIME_SYMB, 1);

    err |= lgw_reg_w(SX1302_REG_RX_TOP_LORA_SERVICE_FSK_RX_DFE_AGC2_DAGC_IN_COMP, 1);

    err |= lgw_reg_w(SX1302_REG_RX_TOP_LORA_SERVICE_FSK_TXRX_CFG1_MODEM_EN, 1);
    err |= lgw_reg_w(SX1302_REG_RX_TOP_LORA_SERVICE_FSK_TXRX_CFG2_CADRXTX, 1);

    err |= lgw_reg_w(SX1302_REG_RX_TOP_LORA_SERVICE_FSK_TXRX_CFG2_MODEM_START, 1);

    /* DFT peak mode : set to AUTO, check timestamp_counter_correction() if changed */
    err |= lgw_reg_w(SX1302_REG_RX_TOP_LORA_SERVICE_FSK_RX_CFG0_DFT_PEAK_EN, RX_DFT_PEAK_MODE_AUTO);

    return err;
}

/* ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ */

int sx1302_modem_enable(void) {
    int err = LGW_REG_SUCCESS;

    /* Enable LoRa multi-SF modems */
    err |= lgw_reg_w(SX1302_REG_COMMON_GEN_CONCENTRATOR_MODEM_ENABLE, 0x01);

    /* Enable LoRa service modem */
    err |= lgw_reg_w(SX1302_REG_COMMON_GEN_MBWSSF_MODEM_ENABLE, 0x01);

    /* Enable FSK modem */
    err |= lgw_reg_w(SX1302_REG_COMMON_GEN_FSK_MODEM_ENABLE, 0x01);

    /* Enable RX */
    err |= lgw_reg_w(SX1302_REG_COMMON_GEN_GLOBAL_EN, 0x01);

    return err;
}

/* ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ */

int sx1302_lora_syncword(bool public, uint8_t lora_service_sf) {
    int err = LGW_REG_SUCCESS;

    /* Multi-SF modem configuration */
    DEBUG_MSG("INFO: configuring LoRa (Multi-SF) SF5->SF6 with syncword PRIVATE (0x12)\n");
    err |= lgw_reg_w(SX1302_REG_RX_TOP_FRAME_SYNCH0_SF5_PEAK1_POS_SF5, 2);
    err |= lgw_reg_w(SX1302_REG_RX_TOP_FRAME_SYNCH1_SF5_PEAK2_POS_SF5, 4);
    err |= lgw_reg_w(SX1302_REG_RX_TOP_FRAME_SYNCH0_SF6_PEAK1_POS_SF6, 2);
    err |= lgw_reg_w(SX1302_REG_RX_TOP_FRAME_SYNCH1_SF6_PEAK2_POS_SF6, 4);
    if (public == true) {
        DEBUG_MSG("INFO: configuring LoRa (Multi-SF) SF7->SF12 with syncword PUBLIC (0x34)\n");
        err |= lgw_reg_w(SX1302_REG_RX_TOP_FRAME_SYNCH0_SF7TO12_PEAK1_POS_SF7TO12, 6);
        err |= lgw_reg_w(SX1302_REG_RX_TOP_FRAME_SYNCH1_SF7TO12_PEAK2_POS_SF7TO12, 8);
    } else {
        DEBUG_MSG("INFO: configuring LoRa (Multi-SF) SF7->SF12 with syncword PRIVATE (0x12)\n");
        err |= lgw_reg_w(SX1302_REG_RX_TOP_FRAME_SYNCH0_SF7TO12_PEAK1_POS_SF7TO12, 2);
        err |= lgw_reg_w(SX1302_REG_RX_TOP_FRAME_SYNCH1_SF7TO12_PEAK2_POS_SF7TO12, 4);
    }

    /* LoRa Service modem configuration */
    if ((public == false) || (lora_service_sf == DR_LORA_SF5) || (lora_service_sf == DR_LORA_SF6)) {
        DEBUG_PRINTF("INFO: configuring LoRa (Service) SF%u with syncword PRIVATE (0x12)\n", lora_service_sf);
        err |= lgw_reg_w(SX1302_REG_RX_TOP_LORA_SERVICE_FSK_FRAME_SYNCH0_PEAK1_POS, 2);
        err |= lgw_reg_w(SX1302_REG_RX_TOP_LORA_SERVICE_FSK_FRAME_SYNCH1_PEAK2_POS, 4);
    } else {
        DEBUG_PRINTF("INFO: configuring LoRa (Service) SF%u with syncword PUBLIC (0x34)\n", lora_service_sf);
        err |= lgw_reg_w(SX1302_REG_RX_TOP_LORA_SERVICE_FSK_FRAME_SYNCH0_PEAK1_POS, 6);
        err |= lgw_reg_w(SX1302_REG_RX_TOP_LORA_SERVICE_FSK_FRAME_SYNCH1_PEAK2_POS, 8);
    }

    return err;
}

/* ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ */

uint32_t sx1302_timestamp_counter(bool pps) {
    uint32_t inst_cnt, pps_cnt;
    timestamp_counter_get(&counter_us, &inst_cnt, &pps_cnt);
    return ((pps == true) ? pps_cnt : inst_cnt);
}

/* ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ */

int sx1302_gps_enable(bool enable) {
    int err = LGW_REG_SUCCESS;

    if (enable == true) {
        err |= lgw_reg_w(SX1302_REG_TIMESTAMP_GPS_CTRL_GPS_EN, 1);
        err |= lgw_reg_w(SX1302_REG_TIMESTAMP_GPS_CTRL_GPS_POL, 1); /* invert polarity for PPS */
    } else {
        err |= lgw_reg_w(SX1302_REG_TIMESTAMP_GPS_CTRL_GPS_EN, 0);
    }

    return err;
}

/* ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ */

int sx1302_agc_load_firmware(const uint8_t *firmware) {
    int32_t val;
    uint8_t fw_check[MCU_FW_SIZE];
    int err = LGW_REG_SUCCESS;

    /* Take control over AGC MCU */
    err |= lgw_reg_w(SX1302_REG_AGC_MCU_CTRL_MCU_CLEAR, 0x01);
    err |= lgw_reg_w(SX1302_REG_AGC_MCU_CTRL_HOST_PROG, 0x01);
    err |= lgw_reg_w(SX1302_REG_COMMON_PAGE_PAGE, 0x00);

    /* Write AGC fw in AGC MEM */
    err |= lgw_mem_wb(AGC_MEM_ADDR, firmware, MCU_FW_SIZE);

    /* Read back and check */
    err |= lgw_mem_rb(AGC_MEM_ADDR, fw_check, MCU_FW_SIZE, false);
    if (memcmp(firmware, fw_check, sizeof fw_check) != 0) {
        printf("ERROR: AGC fw read/write check failed\n");
        return LGW_REG_ERROR;
    }

    /* Release control over AGC MCU */
    err |= lgw_reg_w(SX1302_REG_AGC_MCU_CTRL_HOST_PROG, 0x00);
    err |= lgw_reg_w(SX1302_REG_AGC_MCU_CTRL_MCU_CLEAR, 0x00);

    err |= lgw_reg_r(SX1302_REG_AGC_MCU_CTRL_PARITY_ERROR, &val);
    if (val != 0) {
        printf("ERROR: Failed to load AGC fw: parity error check failed\n");
        return LGW_REG_ERROR;
    }
    DEBUG_MSG("AGC fw loaded\n");

    return err;
}

/* ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ */

int sx1302_agc_status(uint8_t* status) {
    int32_t val;
    int err = LGW_REG_SUCCESS;

    err = lgw_reg_r(SX1302_REG_AGC_MCU_MCU_AGC_STATUS_MCU_AGC_STATUS, &val);
    if (err != LGW_REG_SUCCESS) {
        printf("ERROR: Failed to get AGC status\n");
        return LGW_REG_ERROR;
    }

    *status = (uint8_t)val;

    return err;
}

/* ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ */

int sx1302_agc_wait_status(uint8_t status) {
    uint8_t val;

    do {
        if (sx1302_agc_status(&val) != LGW_REG_SUCCESS) {
            return LGW_REG_ERROR;
        }
        /* TODO: add timeout */
    } while (val != status);

    return LGW_REG_SUCCESS;
}

/* ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ */

int sx1302_agc_mailbox_read(uint8_t mailbox, uint8_t* value) {
    uint16_t reg;
    int32_t val;

    /* Check parameters */
    if (mailbox > 3) {
        printf("ERROR: invalid AGC mailbox ID\n");
        return LGW_REG_ERROR;
    }

    reg = SX1302_REG_AGC_MCU_MCU_MAIL_BOX_RD_DATA_BYTE0_MCU_MAIL_BOX_RD_DATA - mailbox;
    if (lgw_reg_r(reg, &val) != LGW_REG_SUCCESS) {
        printf("ERROR: failed to read AGC mailbox\n");
        return LGW_REG_ERROR;
    }

    *value = (uint8_t)val;

    return LGW_REG_SUCCESS;
}

/* ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ */

int sx1302_agc_mailbox_write(uint8_t mailbox, uint8_t value) {
    uint16_t reg;

    /* Check parameters */
    if (mailbox > 3) {
        printf("ERROR: invalid AGC mailbox ID\n");
        return LGW_REG_ERROR;
    }

    reg = SX1302_REG_AGC_MCU_MCU_MAIL_BOX_WR_DATA_BYTE0_MCU_MAIL_BOX_WR_DATA - mailbox;
    if (lgw_reg_w(reg, (int32_t)value) != LGW_REG_SUCCESS) {
        printf("ERROR: failed to write AGC mailbox\n");
        return LGW_REG_ERROR;
    }

    return LGW_REG_SUCCESS;
}

/* ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ */

int sx1302_agc_start(uint8_t version, lgw_radio_type_t radio_type, uint8_t ana_gain, uint8_t dec_gain, bool full_duplex, bool lbt_enable) {
    uint8_t val;
    struct agc_gain_params_s agc_params;
    uint8_t pa_start_delay;
    uint8_t fdd_mode = ((full_duplex == true) ? 1 : 0);

    /* Check parameters */
    if ((radio_type != LGW_RADIO_TYPE_SX1255) && (radio_type != LGW_RADIO_TYPE_SX1257) && (radio_type != LGW_RADIO_TYPE_SX1250)) {
        DEBUG_MSG("ERROR: invalid radio type\n");
        return LGW_REG_ERROR;
    }

    /* Wait for AGC fw to be started, and VERSION available in mailbox */
    sx1302_agc_wait_status(0x01); /* fw has started, VERSION is ready in mailbox */

    sx1302_agc_mailbox_read(0, &val);
    if (val != version) {
        printf("ERROR: wrong AGC fw version (%d)\n", val);
        return LGW_REG_ERROR;
    }
    DEBUG_PRINTF("AGC FW VERSION: %d\n", val);

    /* -----------------------------------------------------------------------*/

    /* Configure Radio A gains */
    sx1302_agc_mailbox_write(0, ana_gain); /* 0:auto agc*/
    sx1302_agc_mailbox_write(1, dec_gain);
    if (radio_type != LGW_RADIO_TYPE_SX1250) {
        printf("AGC: setting fdd_mode to %u\n", fdd_mode);
        sx1302_agc_mailbox_write(2, fdd_mode);
    }

    /* notify AGC that gains has been set to mailbox for Radio A */
    sx1302_agc_mailbox_write(3, AGC_RADIO_A_INIT_DONE);

    /* Wait for AGC to acknoledge it has received gain settings for Radio A */
    sx1302_agc_wait_status(0x02);

    /* Check ana_gain setting */
    sx1302_agc_mailbox_read(0, &val);
    if (val != ana_gain) {
        printf("ERROR: Analog gain of Radio A has not been set properly\n");
        return LGW_REG_ERROR;
    }

    /* Check dec_gain setting */
    sx1302_agc_mailbox_read(1, &val);
    if (val != dec_gain) {
        printf("ERROR: Decimator gain of Radio A has not been set properly\n");
        return LGW_REG_ERROR;
    }

    /* Check FDD mode setting */
    sx1302_agc_mailbox_read(2, &val);
    if (val != fdd_mode) {
        printf("ERROR: FDD mode of Radio A has not been set properly\n");
        return LGW_REG_ERROR;
    }

    DEBUG_MSG("AGC: Radio A config done\n");

    /* -----------------------------------------------------------------------*/

    /* Configure Radio B gains */
    sx1302_agc_mailbox_write(0, ana_gain); /* 0:auto agc*/
    sx1302_agc_mailbox_write(1, dec_gain);
    if (radio_type != LGW_RADIO_TYPE_SX1250) {
        sx1302_agc_mailbox_write(2, fdd_mode);
    }

    /* notify AGC that gains has been set to mailbox for Radio B */
    sx1302_agc_mailbox_write(3, AGC_RADIO_B_INIT_DONE);

    /* Wait for AGC to acknoledge it has received gain settings for Radio B */
    sx1302_agc_wait_status(0x03);

    /* Check ana_gain setting */
    sx1302_agc_mailbox_read(0, &val);
    if (val != ana_gain) {
        printf("ERROR: Analog gain of Radio B has not been set properly\n");
        return LGW_REG_ERROR;
    }

    /* Check dec_gain setting */
    sx1302_agc_mailbox_read(1, &val);
    if (val != dec_gain) {
        printf("ERROR: Decimator gain of Radio B has not been set properly\n");
        return LGW_REG_ERROR;
    }

    /* Check FDD mode setting */
    sx1302_agc_mailbox_read(2, &val);
    if (val != fdd_mode) {
        printf("ERROR: FDD mode of Radio B has not been set properly\n");
        return LGW_REG_ERROR;
    }

    DEBUG_MSG("AGC: Radio B config done\n");

    /* -----------------------------------------------------------------------*/

    /* Configure AGC gains */
    agc_params = (radio_type == LGW_RADIO_TYPE_SX1250) ? agc_params_sx1250 : agc_params_sx125x;

    /* Configure analog gain min/max */
    sx1302_agc_mailbox_write(0, agc_params.ana_min);
    sx1302_agc_mailbox_write(1, agc_params.ana_max);

    /* notify AGC that params have been set to mailbox */
    sx1302_agc_mailbox_write(3, 0x03);

    /* Wait for AGC to acknoledge it has received params */
    sx1302_agc_wait_status(0x04);

    /* Check params */
    sx1302_agc_mailbox_read(0, &val);
    if (val != agc_params.ana_min) {
        printf("ERROR: wrong ana_min (w:%u r:%u)\n", agc_params.ana_min, val);
        return LGW_REG_ERROR;
    }
    sx1302_agc_mailbox_read(1, &val);
    if (val != agc_params.ana_max) {
        printf("ERROR: ana_max (w:%u r:%u)\n", agc_params.ana_max, val);
        return LGW_REG_ERROR;
    }

    DEBUG_MSG("AGC: config of analog gain min/max done\n");

    /* -----------------------------------------------------------------------*/

    /* Configure analog thresholds */
    sx1302_agc_mailbox_write(0, agc_params.ana_thresh_l);
    sx1302_agc_mailbox_write(1, agc_params.ana_thresh_h);

    /* notify AGC that params have been set to mailbox */
    sx1302_agc_mailbox_write(3, 0x04);

    /* Wait for AGC to acknoledge it has received params */
    sx1302_agc_wait_status(0x05);

    /* Check params */
    sx1302_agc_mailbox_read(0, &val);
    if (val != agc_params.ana_thresh_l) {
        printf("ERROR: wrong ana_thresh_l (w:%u r:%u)\n", agc_params.ana_thresh_l, val);
        return LGW_REG_ERROR;
    }
    sx1302_agc_mailbox_read(1, &val);
    if (val != agc_params.ana_thresh_h) {
        printf("ERROR: wrong ana_thresh_h (w:%u r:%u)\n", agc_params.ana_thresh_h, val);
        return LGW_REG_ERROR;
    }

    DEBUG_MSG("AGC: config of analog threshold done\n");

    /* -----------------------------------------------------------------------*/

    /* Configure decimator attenuation min/max */
    sx1302_agc_mailbox_write(0, agc_params.dec_attn_min);
    sx1302_agc_mailbox_write(1, agc_params.dec_attn_max);

    /* notify AGC that params have been set to mailbox */
    sx1302_agc_mailbox_write(3, 0x05);

    /* Wait for AGC to acknoledge it has received params */
    sx1302_agc_wait_status(0x06);

    /* Check params */
    sx1302_agc_mailbox_read(0, &val);
    if (val != agc_params.dec_attn_min) {
        printf("ERROR: wrong dec_attn_min (w:%u r:%u)\n", agc_params.dec_attn_min, val);
        return LGW_REG_ERROR;
    }
    sx1302_agc_mailbox_read(1, &val);
    if (val != agc_params.dec_attn_max) {
        printf("ERROR: wrong dec_attn_max (w:%u r:%u)\n", agc_params.dec_attn_max, val);
        return LGW_REG_ERROR;
    }

    DEBUG_MSG("AGC: config of decimator atten min/max done\n");

    /* -----------------------------------------------------------------------*/

    /* Configure decimator attenuation thresholds */
    sx1302_agc_mailbox_write(0, agc_params.dec_thresh_l);
    sx1302_agc_mailbox_write(1, agc_params.dec_thresh_h1);
    sx1302_agc_mailbox_write(2, agc_params.dec_thresh_h2);

    /* notify AGC that params have been set to mailbox */
    sx1302_agc_mailbox_write(3, 0x06);

    /* Wait for AGC to acknoledge it has received params */
    sx1302_agc_wait_status(0x07);

        /* Check params */
    sx1302_agc_mailbox_read(0, &val);
    if (val != agc_params.dec_thresh_l) {
        printf("ERROR: wrong dec_thresh_l (w:%u r:%u)\n", agc_params.dec_thresh_l, val);
        return LGW_REG_ERROR;
    }
    sx1302_agc_mailbox_read(1, &val);
    if (val != agc_params.dec_thresh_h1) {
        printf("ERROR: wrong dec_thresh_h1 (w:%u r:%u)\n", agc_params.dec_thresh_h1, val);
        return LGW_REG_ERROR;
    }
    sx1302_agc_mailbox_read(2, &val);
    if (val != agc_params.dec_thresh_h2) {
        printf("ERROR: wrong dec_thresh_h2 (w:%u r:%u)\n", agc_params.dec_thresh_h2, val);
        return LGW_REG_ERROR;
    }

    DEBUG_MSG("AGC: config of decimator threshold done\n");

    /* -----------------------------------------------------------------------*/

    /* Configure channel attenuation min/max */
    sx1302_agc_mailbox_write(0, agc_params.chan_attn_min);
    sx1302_agc_mailbox_write(1, agc_params.chan_attn_max);

    /* notify AGC that params have been set to mailbox */
    sx1302_agc_mailbox_write(3, 0x07);

    /* Wait for AGC to acknoledge it has received params */
    sx1302_agc_wait_status(0x08);

    /* Check params */
    sx1302_agc_mailbox_read(0, &val);
    if (val != agc_params.chan_attn_min) {
        printf("ERROR: wrong chan_attn_min (w:%u r:%u)\n", agc_params.chan_attn_min, val);
        return LGW_REG_ERROR;
    }
    sx1302_agc_mailbox_read(1, &val);
    if (val != agc_params.chan_attn_max) {
        printf("ERROR: wrong chan_attn_max (w:%u r:%u)\n", agc_params.chan_attn_max, val);
        return LGW_REG_ERROR;
    }

    DEBUG_MSG("AGC: config of channel atten min/max done\n");

    /* -----------------------------------------------------------------------*/

    /* Configure channel attenuation threshold */
    sx1302_agc_mailbox_write(0, agc_params.chan_thresh_l);
    sx1302_agc_mailbox_write(1, agc_params.chan_thresh_h);

    /* notify AGC that params have been set to mailbox */
    sx1302_agc_mailbox_write(3, 0x08);

    /* Wait for AGC to acknoledge it has received params */
    sx1302_agc_wait_status(0x09);

    /* Check params */
    sx1302_agc_mailbox_read(0, &val);
    if (val != agc_params.chan_thresh_l) {
        printf("ERROR: wrong chan_thresh_l (w:%u r:%u)\n", agc_params.chan_thresh_l, val);
        return LGW_REG_ERROR;
    }
    sx1302_agc_mailbox_read(1, &val);
    if (val != agc_params.chan_thresh_h) {
        printf("ERROR: wrong chan_thresh_h (w:%u r:%u)\n", agc_params.chan_thresh_h, val);
        return LGW_REG_ERROR;
    }

    DEBUG_MSG("AGC: config of channel atten threshold done\n");

    /* -----------------------------------------------------------------------*/

    /* Configure sx1250 SetPAConfig */
    if (radio_type == LGW_RADIO_TYPE_SX1250) {
        sx1302_agc_mailbox_write(0, agc_params.deviceSel);
        sx1302_agc_mailbox_write(1, agc_params.hpMax);
        sx1302_agc_mailbox_write(2, agc_params.paDutyCycle);

        /* notify AGC that params have been set to mailbox */
        sx1302_agc_mailbox_write(3, 0x09);

        /* Wait for AGC to acknoledge it has received params */
        sx1302_agc_wait_status(0x0A);

        /* Check params */
        sx1302_agc_mailbox_read(0, &val);
        if (val != agc_params.deviceSel) {
            printf("ERROR: wrong deviceSel (w:%u r:%u)\n", agc_params.deviceSel, val);
            return LGW_REG_ERROR;
        }
        sx1302_agc_mailbox_read(1, &val);
        if (val != agc_params.hpMax) {
            printf("ERROR: wrong hpMax (w:%u r:%u)\n", agc_params.hpMax, val);
            return LGW_REG_ERROR;
        }
        sx1302_agc_mailbox_read(2, &val);
        if (val != agc_params.paDutyCycle) {
            printf("ERROR: wrong paDutyCycle (w:%u r:%u)\n", agc_params.paDutyCycle, val);
            return LGW_REG_ERROR;
        }

        DEBUG_MSG("AGC: config of sx1250 PA optimal settings done\n");
    }

    /* -----------------------------------------------------------------------*/

    /* Set PA start delay */
    pa_start_delay = 8;
    sx1302_agc_mailbox_write(0, pa_start_delay); /* 1 LSB = 100 µs*/

    /* notify AGC that params have been set to mailbox */
    sx1302_agc_mailbox_write(3, 0x0A);

    /* Wait for AGC to acknoledge it has received params */
    sx1302_agc_wait_status(0x0B);

    /* Check params */
    sx1302_agc_mailbox_read(0, &val);
    if (val != pa_start_delay) {
        printf("ERROR: wrong PA start delay (w:%u r:%u)\n", pa_start_delay, val);
        return LGW_REG_ERROR;
    }

    DEBUG_MSG("AGC: config of PA start delay done\n");

    /* -----------------------------------------------------------------------*/

    /* Enable LBT if required */
    sx1302_agc_mailbox_write(0, (lbt_enable == true) ? 1 : 0);

    /* notify AGC that params have been set to mailbox */
    sx1302_agc_mailbox_write(3, 0x0B);

    /* Wait for AGC to acknoledge it has received params */
    sx1302_agc_wait_status(0x0F);

     /* Check params */
    sx1302_agc_mailbox_read(0, &val);
    if ((bool)val != lbt_enable) {
        printf("ERROR: wrong LBT configuration (w:%u r:%u)\n", lbt_enable, val);
        return LGW_REG_ERROR;
    }

    DEBUG_PRINTF("AGC: LBT is %s\n", (lbt_enable == true) ? "enabled" : "disabled");

    /* -----------------------------------------------------------------------*/

    /* notify AGC that configuration is finished */
    sx1302_agc_mailbox_write(3, 0x0F);

    DEBUG_MSG("AGC: started\n");

    return LGW_REG_SUCCESS;
}

/* ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ */

int sx1302_arb_load_firmware(const uint8_t *firmware) {
    uint8_t fw_check[MCU_FW_SIZE];
    int32_t val;
    int err = LGW_REG_SUCCESS;

    /* Take control over ARB MCU */
    err |= lgw_reg_w(SX1302_REG_ARB_MCU_CTRL_MCU_CLEAR, 0x01);
    err |= lgw_reg_w(SX1302_REG_ARB_MCU_CTRL_HOST_PROG, 0x01);
    err |= lgw_reg_w(SX1302_REG_COMMON_PAGE_PAGE, 0x00);

    /* Write ARB fw in ARB MEM */
    err |= lgw_mem_wb(ARB_MEM_ADDR, &firmware[0], MCU_FW_SIZE);

    /* Read back and check */
    err |= lgw_mem_rb(ARB_MEM_ADDR, fw_check, MCU_FW_SIZE, false);
    if (memcmp(firmware, fw_check, sizeof fw_check) != 0) {
        printf("ERROR: ARB fw read/write check failed\n");
        return LGW_REG_ERROR;
    }

    /* Release control over ARB MCU */
    err |= lgw_reg_w(SX1302_REG_ARB_MCU_CTRL_HOST_PROG, 0x00);
    err |= lgw_reg_w(SX1302_REG_ARB_MCU_CTRL_MCU_CLEAR, 0x00);

    err |= lgw_reg_r(SX1302_REG_ARB_MCU_CTRL_PARITY_ERROR, &val);
    if (val != 0) {
        printf("ERROR: Failed to load ARB fw: parity error check failed\n");
        return LGW_REG_ERROR;
    }
    DEBUG_MSG("ARB fw loaded\n");

    return err;
}

/* ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ */

int sx1302_arb_status(uint8_t* status) {
    int32_t val;
    int err = LGW_REG_SUCCESS;

    err = lgw_reg_r(SX1302_REG_ARB_MCU_MCU_ARB_STATUS_MCU_ARB_STATUS, &val);
    if (err != LGW_REG_SUCCESS) {
        printf("ERROR: Failed to get ARB status\n");
        return LGW_REG_ERROR;
    }

    *status = (uint8_t)val;

    return err;
}

/* ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ */

int sx1302_arb_wait_status(uint8_t status) {
    uint8_t val;

    do {
        if (sx1302_arb_status(&val) != LGW_REG_SUCCESS) {
            return LGW_REG_ERROR;
        }
        /* TODO: add timeout */
    } while (val != status);

    return LGW_REG_SUCCESS;
}

/* ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ */

int sx1302_arb_debug_read(uint8_t reg_id, uint8_t* value) {
    uint16_t reg;
    int32_t val;

    /* Check parameters */
    if (reg_id > 15) {
        printf("ERROR: invalid ARB debug register ID\n");
        return LGW_REG_ERROR;
    }

    reg = SX1302_REG_ARB_MCU_ARB_DEBUG_STS_0_ARB_DEBUG_STS_0 + reg_id;
    if (lgw_reg_r(reg, &val) != LGW_REG_SUCCESS) {
        printf("ERROR: failed to read ARB debug register\n");
        return LGW_REG_ERROR;
    }

    *value = (uint8_t)val;

    return LGW_REG_SUCCESS;
}

/* ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ */

int sx1302_arb_debug_write(uint8_t reg_id, uint8_t value) {
    uint16_t reg;

    /* Check parameters */
    if (reg_id > 3) {
        printf("ERROR: invalid ARB debug register ID\n");
        return LGW_REG_ERROR;
    }

    reg = SX1302_REG_ARB_MCU_ARB_DEBUG_CFG_0_ARB_DEBUG_CFG_0 + reg_id;
    if (lgw_reg_w(reg, (int32_t)value) != LGW_REG_SUCCESS) {
        printf("ERROR: failed to write ARB debug register ID\n");
        return LGW_REG_ERROR;
    }

    return LGW_REG_SUCCESS;
}

/* ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ */

void sx1302_arb_set_debug_stats(bool enable, uint8_t sf) {
    if (enable == true) {
        DEBUG_PRINTF("ARB: Debug stats enabled for SF%u\n", sf);
        lgw_reg_w(SX1302_REG_ARB_MCU_ARB_DEBUG_CFG_0_ARB_DEBUG_CFG_0, sf);
    } else {
        DEBUG_MSG("ARB: Debug stats disabled\n");
    }
}

/* ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ */

uint8_t sx1302_arb_get_debug_stats_detect(uint8_t channel) {
    int32_t dbg_val;

    if (channel >= 8) {
        printf("ERROR: wrong configuration, channel num must be < 8");
        return 0;
    }
    lgw_reg_r(SX1302_REG_ARB_MCU_ARB_DEBUG_STS_0_ARB_DEBUG_STS_0 + channel, &dbg_val);

    return (uint8_t)dbg_val;
}

/* ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ */

uint8_t sx1302_arb_get_debug_stats_alloc(uint8_t channel) {
    int32_t dbg_val;

    if (channel >= 8) {
        printf("ERROR: wrong configuration, channel num must be < 8");
        return 0;
    }
    lgw_reg_r(SX1302_REG_ARB_MCU_ARB_DEBUG_STS_8_ARB_DEBUG_STS_8 + channel, &dbg_val);

    return (uint8_t)dbg_val;
}

/* ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ */

void sx1302_arb_print_debug_stats(void) {
    int i;
    uint8_t nb_detect;
    uint8_t nb_alloc;
    int nb_detect_total = 0;
    int nb_alloc_total = 0;

    /* Get number of detects for all channels */
    nb_detect_total = 0;
    DEBUG_MSG("ARB: nb_detect: [");
    for (i = 0; i < 8; i++) {
        nb_detect = sx1302_arb_get_debug_stats_detect(i);
        DEBUG_PRINTF("%u ", nb_detect);
        nb_detect_total += nb_detect;
    }
    DEBUG_MSG("]\n");

    /* Get number of modem allocation for all channels */
    nb_alloc_total = 0;
    DEBUG_MSG("ARB: nb_alloc:  [");
    for (i = 0; i < 8; i++) {
        nb_alloc = sx1302_arb_get_debug_stats_alloc(i);
        DEBUG_PRINTF("%u ", nb_alloc);
        nb_alloc_total += nb_alloc;
    }
    DEBUG_MSG("]\n");
}

/* ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ */

int sx1302_arb_start(uint8_t version, const struct lgw_conf_ftime_s * ftime_context) {
    uint8_t val;

    /* Wait for ARB fw to be started, and VERSION available in debug registers */
    sx1302_arb_wait_status(0x01);

    /* Get firmware VERSION */
    sx1302_arb_debug_read(0, &val);
    if (val != version) {
        printf("ERROR: wrong ARB fw version (%d)\n", val);
        return LGW_REG_ERROR;
    }
    DEBUG_PRINTF("ARB FW VERSION: %d\n", val);

    /* Enable/disable ARB detect/modem alloc stats for the specified SF */
    sx1302_arb_set_debug_stats(true, DR_LORA_SF7);

    /* Enable/Disable double demod for different timing set (best timestamp / best demodulation) - 1 bit per SF (LSB=SF5, MSB=SF12) => 0:Disable 1:Enable */
    if (ftime_context->enable == false) {
        printf("ARB: dual demodulation disabled for all SF\n");
        sx1302_arb_debug_write(3, 0x00); /* double demod disabled for all SF */
    } else {
        if (ftime_context->mode == LGW_FTIME_MODE_ALL_SF) {
            printf("ARB: dual demodulation enabled for all SF\n");
            sx1302_arb_debug_write(3, 0xFF); /* double demod enabled for all SF */
        } else if (ftime_context->mode == LGW_FTIME_MODE_HIGH_CAPACITY) {
            printf("ARB: dual demodulation enabled for SF5 -> SF10\n");
            sx1302_arb_debug_write(3, 0x3F); /* double demod enabled for SF10 <- SF5 */
        } else {
            printf("ERROR: fine timestamp mode is not supported (%d)\n", ftime_context->mode);
            return LGW_REG_ERROR;
        }
    }

    /* Set double detect packet filtering threshold [0..3] */
    sx1302_arb_debug_write(2, 3);

    /* Notify ARB that it can resume */
    sx1302_arb_debug_write(1, 1);

    /* Wait for ARB to acknoledge */
    sx1302_arb_wait_status(0x00);

    DEBUG_MSG("ARB: started\n");

    return LGW_REG_SUCCESS;
}

/* ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ */

int sx1302_fetch(uint8_t * nb_pkt) {
    int err;
    struct timeval tm;

    /* Record function start time */
    _meas_time_start(&tm);

    /* Fetch packets from sx1302 if no more left in RX buffer */
    if (rx_buffer.buffer_pkt_nb == 0) {
        /* Initialize RX buffer */
        err = rx_buffer_new(&rx_buffer);
        if (err != LGW_REG_SUCCESS) {
            printf("ERROR: Failed to initialize RX buffer\n");
            return LGW_REG_ERROR;
        }

        /* Fetch RX buffer if any data available */
        err = rx_buffer_fetch(&rx_buffer);
        if (err != LGW_REG_SUCCESS) {
            printf("ERROR: Failed to fetch RX buffer\n");
            return LGW_REG_ERROR;
        }
    } else {
        printf("Note: remaining %u packets in RX buffer, do not fetch sx1302 yet...\n", rx_buffer.buffer_pkt_nb);
    }

    /* Return the number of packet fetched */
    *nb_pkt = rx_buffer.buffer_pkt_nb;

    _meas_time_stop(2, tm, __FUNCTION__);

    return LGW_REG_SUCCESS;
}

/* ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ */

int sx1302_parse(lgw_context_t * context, struct lgw_pkt_rx_s * p) {
    int err;
    int ifmod; /* type of if_chain/modem a packet was received by */
    int32_t if_freq_hz;
    int32_t if_freq_error;
    double pkt_freq_error;
    uint16_t payload_crc16_calc;
    uint8_t cr;
    int32_t timestamp_correction;
    rx_packet_t pkt;
    struct timeval tm;

    /* Record function start time */
    _meas_time_start(&tm);

    /* Check input params */
    CHECK_NULL(context);
    CHECK_NULL(p);

#if 0
    /* For DEBUG: WARNING: it is quite time consuming in USB mode, due to SPI over USB latency
        Print statistics of number of detects and modem allocations from ARB for configured SF (see sx1302_arb_start())
    */
    sx1302_arb_print_debug_stats();
#endif

    /* get packet from RX buffer */
    err = rx_buffer_pop(&rx_buffer, &pkt);
    if (err == LGW_REG_WARNING) {
        rx_buffer_del(&rx_buffer); /* clear the buffer */
        return err;
    } else if (err == LGW_REG_ERROR) {
        return err;
    }

    /* copy payload to result struct */
    memcpy((void *)p->payload, (void *)(&(pkt.payload)), pkt.rxbytenb_modem);
    p->size = pkt.rxbytenb_modem;

    /* process metadata */
    p->modem_id = pkt.modem_id;
    p->if_chain = pkt.rx_channel_in;
    if (p->if_chain >= LGW_IF_CHAIN_NB) {
        DEBUG_PRINTF("WARNING: %u NOT A VALID IF_CHAIN NUMBER, ABORTING\n", p->if_chain);
        return LGW_REG_ERROR;
    }
    ifmod = ifmod_config[p->if_chain];
    DEBUG_PRINTF("[%d 0x%02X]\n", p->if_chain, ifmod);

    p->rf_chain = (uint8_t)context->if_chain_cfg[p->if_chain].rf_chain;

    /* Get the frequency for the channel configuration */
    p->freq_hz = (uint32_t)((int32_t)context->rf_chain_cfg[p->rf_chain].freq_hz + context->if_chain_cfg[p->if_chain].freq_hz);

    /* Get signal strength : offset and temperature compensation will be applied later */
    p->rssic = (float)(pkt.rssi_chan_avg);
    p->rssis = (float)(pkt.rssi_signal_avg);

    /* Get modulation metadata */
    if ((ifmod == IF_LORA_MULTI) || (ifmod == IF_LORA_STD)) {
        DEBUG_PRINTF("Note: LoRa packet (modem %u chan %u)\n", p->modem_id, p->if_chain);
        p->modulation = MOD_LORA;

        /* Get CRC status */
        if (pkt.crc_en || ((ifmod == IF_LORA_STD) && (context->lora_service_cfg.implicit_crc_en == true))) {
            /* CRC enabled */
            if (pkt.payload_crc_error) {
                p->status = STAT_CRC_BAD;
            } else {
                p->status = STAT_CRC_OK;

                /* Sanity check of the payload CRC */
                if (p->size > 0) {
                    payload_crc16_calc = sx1302_lora_payload_crc(p->payload, p->size);
                    if (payload_crc16_calc != pkt.rx_crc16_value) {
                        printf("ERROR: Payload CRC16 check failed (got:0x%04X calc:0x%04X)\n", pkt.rx_crc16_value, payload_crc16_calc);
                        if (log_file != NULL) {
                            fprintf(log_file, "ERROR: Payload CRC16 check failed (got:0x%04X calc:0x%04X)\n", pkt.rx_crc16_value, payload_crc16_calc);
                            dbg_log_buffer_to_file(log_file, rx_buffer.buffer, rx_buffer.buffer_size);
                        }
                        return LGW_REG_ERROR;
                    } else {
                        DEBUG_PRINTF("Payload CRC check OK (0x%04X)\n", pkt.rx_crc16_value);
                    }
                }
            }
        } else {
            /* CRC disabled */
            p->status = STAT_NO_CRC;
        }

#if 0
        int i;
        /* FOR DEBUG: Check data integrity for known devices (debug context) */
        if (p->status == STAT_CRC_OK || p->status == STAT_NO_CRC) {
            /*  We compare the received payload with predefined ones to ensure that the payload content is what we expect.
                4 bytes: ID to identify the payload
                4 bytes: packet counter used to initialize the seed for pseudo-random generation
                x bytes: pseudo-random payload
            */
            int res;
            for (i = 0; i < context->debug_cfg.nb_ref_payload; i++) {
                res = dbg_check_payload(&(context->debug_cfg), log_file, p->payload, p->size, i, pkt.rx_rate_sf);
                if (res == -1) {
                    printf("ERROR: 0x%08X payload error\n", context->debug_cfg.ref_payload[i].id);
                    if (log_file != NULL) {
                        fprintf(log_file, "ERROR: 0x%08X payload error\n", context->debug_cfg.ref_payload[i].id);
                        dbg_log_buffer_to_file(log_file, rx_buffer.buffer, rx_buffer.buffer_size);
                        dbg_log_payload_diff_to_file(log_file, p->payload, context->debug_cfg.ref_payload[i].payload, p->size);
                    }
                    return LGW_REG_ERROR;
                } else if (res == 1) {
                    DEBUG_PRINTF("0x%08X payload matches\n", context->debug_cfg.ref_payload[i].id);
                } else {
                    /* Do nothing */
                }
            }
        }
#endif

        /* Get SNR - converted from 0.25dB step to dB */
        p->snr = (float)(pkt.snr_average) / 4;

        /* Get bandwidth */
        if (ifmod == IF_LORA_MULTI) {
            p->bandwidth = BW_125KHZ; /* fixed in hardware */
        } else {
            p->bandwidth = context->lora_service_cfg.bandwidth; /* get the parameter from the config variable */
        }

        /* Get datarate */
        switch (pkt.rx_rate_sf) {
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
        if ((ifmod == IF_LORA_MULTI) || (context->lora_service_cfg.implicit_hdr == false)) {
            cr = pkt.coding_rate;
        } else {
            cr = context->lora_service_cfg.implicit_coderate;
        }
        switch (cr) {
            case 1: p->coderate = CR_LORA_4_5; break;
            case 2: p->coderate = CR_LORA_4_6; break;
            case 3: p->coderate = CR_LORA_4_7; break;
            case 4: p->coderate = CR_LORA_4_8; break;
            default: p->coderate = CR_UNDEFINED;
        }

        /* Get frequency offset in Hz depending on bandwidth */
        switch (p->bandwidth) {
            case BW_125KHZ:
                p->freq_offset = (int32_t)((float)(pkt.frequency_offset_error) * FREQ_OFFSET_LSB_125KHZ);
                break;
            case BW_250KHZ:
                p->freq_offset = (int32_t)((float)(pkt.frequency_offset_error) * FREQ_OFFSET_LSB_250KHZ);
                break;
            case BW_500KHZ:
                p->freq_offset = (int32_t)((float)(pkt.frequency_offset_error) * FREQ_OFFSET_LSB_500KHZ);
                break;
            default:
                p->freq_offset = 0;
                printf("Invalid frequency offset\n");
                break;
        }

        /* Adjust the frequency offset with channel IF frequency error:
        When the channel IF frequency has been configured, a precision error may have been introduced
        due to the register precision. We calculate this error here, and adjust the returned frequency error
        accordingly. */
        if_freq_hz = context->if_chain_cfg[p->if_chain].freq_hz; /* The IF frequency set in the registers, is the offset from the zero IF. */
        if_freq_error = if_freq_hz - (IF_HZ_TO_REG(if_freq_hz) * 15625 / 32); /* The error corresponds to how many Hz are missing to get to actual 0 IF. */
        /* Example to better understand what we get here:
            - For a channel set to IF 400000Hz
            - The IF frequency register will actually be set to 399902Hz due to its resolution
            - This means that the modem, to shift to 0 IF, will apply -399902, instead of -400000.
            - This means that the modem will be centered +98hz above the real 0 IF
            - As the freq_offset given is supposed to be relative to the 0 IF, we add this resolution error to it */
        p->freq_offset += if_freq_error;

        /* Get timestamp correction to be applied to count_us */
        timestamp_correction = timestamp_counter_correction(context, p->bandwidth, p->datarate, p->coderate, pkt.crc_en, pkt.rxbytenb_modem, RX_DFT_PEAK_MODE_AUTO);

        /* Compute fine timestamp for packets coming from the modem optimized for fine timestamping, if CRC is OK */
        p->ftime_received = false;
        p->ftime = 0;
        if ((pkt.num_ts_metrics_stored > 0) && (pkt.timing_set == true) && (p->status == STAT_CRC_OK)) {
            /* The actual packet frequency error compared to the channel frequency, need to compute the ftime */
            pkt_freq_error = ((double)(p->freq_hz + p->freq_offset) / (double)(p->freq_hz)) - 1.0;

            /* Compute the fine timestamp */
            err = precise_timestamp_calculate(pkt.num_ts_metrics_stored, &pkt.timestamp_avg[0], pkt.timestamp_cnt, pkt.rx_rate_sf, context->if_chain_cfg[p->if_chain].freq_hz, pkt_freq_error, &(p->ftime));
            if (err == 0) {
                p->ftime_received = true;
            }
        }
    } else if (ifmod == IF_FSK_STD) {
        DEBUG_PRINTF("Note: FSK packet (modem %u chan %u)\n", pkt.modem_id, p->if_chain);
        p->modulation = MOD_FSK;

        /* Get CRC status */
        if (pkt.crc_en) {
            /* CRC enabled */
            if (pkt.payload_crc_error) {
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

        /* Get modulation params */
        p->bandwidth = context->fsk_cfg.bandwidth;
        p->datarate = context->fsk_cfg.datarate;

        /* Compute timestamp correction to be applied */
        timestamp_correction = ((uint32_t)680000 / context->fsk_cfg.datarate) - 20;

        /* RSSI correction */
        p->rssic = RSSI_FSK_POLY_0 + RSSI_FSK_POLY_1 * p->rssic + RSSI_FSK_POLY_2 * pow(p->rssic, 2) + RSSI_FSK_POLY_3 * pow(p->rssic, 3);

        /* Undefined for FSK */
        p->coderate = CR_UNDEFINED;
        p->snr = -128.0;
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

    /* Scale 32 MHz packet timestamp to 1 MHz (microseconds) */
    p->count_us = pkt.timestamp_cnt / 32;

    /* Expand 27-bits counter to 32-bits counter, based on current wrapping status (updated after fetch) */
    p->count_us = timestamp_pkt_expand(&counter_us, p->count_us);


#if 0 // debug code to check for failed submicros/micros handling
    {
        static uint32_t last_valid = 0;
        static uint32_t last_us32 = 0;
        static uint32_t last_pkt_num;
        int32_t diff = p->count_us - last_us32;
        uint32_t pkt_num = (p->payload[4] << 24) | (p->payload[5] << 16) | (p->payload[6] << 8) | (p->payload[7] << 0);

        printf("XXXXXXXXXXXXXXXX inst - ref=%u wrap=%u\n", counter_us.inst.counter_us_27bits_ref, counter_us.inst.counter_us_27bits_wrap);
        printf("XXXXXXXXXXXXXXXX pps  - ref=%u wrap=%u\n", counter_us.pps.counter_us_27bits_ref, counter_us.pps.counter_us_27bits_wrap);
        printf("XXXXXXXXXXXXXXXX pkt=%u (%u) last=%u diff=%d\n", p->count_us, pkt.timestamp_cnt / 32, last_us32, diff);
        printf("XXXXXXXXXXXXXXXX pkt num=%u\n", pkt_num);
        if (last_valid && (diff > 30000000) && (pkt_num == (last_pkt_num + 1))) {
            printf("XXXXXXXXXXXXXXXX ERROR jump ahead count_us\n");
            exit(1);
        }
        last_us32 = p->count_us;
        last_pkt_num = pkt_num;
        last_valid = 1;
    }
#endif

    /* Packet timestamp corrected */
    p->count_us = p->count_us + timestamp_correction;

    /* Packet CRC status */
    p->crc = pkt.rx_crc16_value;

    _meas_time_stop(2, tm, __FUNCTION__);

    return LGW_REG_SUCCESS;
}

/* ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ */

uint16_t sx1302_lora_payload_crc(const uint8_t * data, uint8_t size) {
    int i;
    int crc = 0;

    for (i = 0; i < size; i++) {
        lora_crc16(data[i], &crc);
    }

    //printf("CRC16: 0x%02X 0x%02X (%X)\n", (uint8_t)(crc >> 8), (uint8_t)crc, crc);
    return (uint16_t)crc;
}

/* ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ */

int sx1302_tx_set_start_delay(uint8_t rf_chain, lgw_radio_type_t radio_type, uint8_t modulation, uint8_t bandwidth, uint8_t chirp_lowpass, uint16_t * delay) {
    int err;
    uint16_t tx_start_delay = TX_START_DELAY_DEFAULT * 32;
    uint16_t radio_bw_delay = 0;
    uint16_t filter_delay = 0;
    uint16_t modem_delay = 0;
    int32_t bw_hz = lgw_bw_getval(bandwidth);
    uint8_t buff[2]; /* for 16 bits register write operation */

    CHECK_NULL(delay);

    /* tx start delay only necessary for beaconing (LoRa) */
    if (modulation != MOD_LORA) {
        *delay = 0;
        return LGW_REG_SUCCESS;
    }

    /* Adjust with radio type and bandwidth */
    switch (radio_type) {
        case LGW_RADIO_TYPE_SX1250:
            if (bandwidth == BW_125KHZ) {
                radio_bw_delay = 19;
            } else if (bandwidth == BW_250KHZ) {
                radio_bw_delay = 24;
            } else if (bandwidth == BW_500KHZ) {
                radio_bw_delay = 21;
            } else {
                DEBUG_MSG("ERROR: bandwidth not supported\n");
                return LGW_REG_ERROR;
            }
            break;
        case LGW_RADIO_TYPE_SX1255:
        case LGW_RADIO_TYPE_SX1257:
            radio_bw_delay = 3*32 + 4;
            if (bandwidth == BW_125KHZ) {
                radio_bw_delay += 0;
            } else if (bandwidth == BW_250KHZ) {
                radio_bw_delay += 6;
            } else if (bandwidth == BW_500KHZ) {
                radio_bw_delay += 0;
            } else {
                DEBUG_MSG("ERROR: bandwidth not supported\n");
                return LGW_REG_ERROR;
            }
            break;
        default:
            DEBUG_MSG("ERROR: radio type not supported\n");
            return LGW_REG_ERROR;
    }

    /* Adjust with modulation */
    filter_delay = ((1 << chirp_lowpass) - 1) * 1e6 / bw_hz;
    modem_delay = 8 * (32e6 / (32 * bw_hz)); /* if bw=125k then modem freq=4MHz */

    /* Compute total delay */
    tx_start_delay -= (radio_bw_delay + filter_delay + modem_delay);

    DEBUG_PRINTF("INFO: tx_start_delay=%u (%u, radio_bw_delay=%u, filter_delay=%u, modem_delay=%u)\n", (uint16_t)tx_start_delay, TX_START_DELAY_DEFAULT*32, radio_bw_delay, filter_delay, modem_delay);

    buff[0] = (uint8_t)(tx_start_delay >> 8);
    buff[1] = (uint8_t)(tx_start_delay >> 0);
    err = lgw_reg_wb(SX1302_REG_TX_TOP_TX_START_DELAY_MSB_TX_START_DELAY(rf_chain), buff, 2);
    CHECK_ERR(err);

    /* return tx_start_delay */
    *delay = tx_start_delay;

    return LGW_REG_SUCCESS;
}

/* ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ */

float sx1302_rssi_get_temperature_offset(struct lgw_rssi_tcomp_s * context, float temperature) {
    /* Chekc params */
    CHECK_NULL(context);

    DEBUG_MSG   ("INFO: RSSI temperature compensation:\n");
    DEBUG_PRINTF("       coeff_a: %.3f\n", context->coeff_a);
    DEBUG_PRINTF("       coeff_b: %.3f\n", context->coeff_b);
    DEBUG_PRINTF("       coeff_c: %.3f\n", context->coeff_c);
    DEBUG_PRINTF("       coeff_d: %.3f\n", context->coeff_d);
    DEBUG_PRINTF("       coeff_e: %.3f\n", context->coeff_e);

    /* Compute the offset to be applied to RSSI for given temperature */
    return ((context->coeff_a * pow(temperature, 4)) +
            (context->coeff_b * pow(temperature, 3)) +
            (context->coeff_c * pow(temperature, 2)) +
            (context->coeff_d * temperature) + context->coeff_e) / pow(2, 16);
}

/* ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ */

uint8_t sx1302_tx_status(uint8_t rf_chain) {
    int err;
    int32_t read_value;

    err = lgw_reg_r(SX1302_REG_TX_TOP_TX_FSM_STATUS_TX_STATUS(rf_chain), &read_value);
    if (err != LGW_REG_SUCCESS) {
        printf("ERROR: Failed to read TX STATUS\n");
        return TX_STATUS_UNKNOWN;
    }

    if (read_value == 0x80) {
        return TX_FREE;
    } else if ((read_value == 0x30) || (read_value == 0x50) || (read_value == 0x60) || (read_value == 0x70)) {
        return TX_EMITTING;
    } else if ((read_value == 0x91) || (read_value == 0x92)) {
        return TX_SCHEDULED;
    } else {
        printf("ERROR: UNKNOWN TX STATUS 0x%02X\n", read_value);
        return TX_STATUS_UNKNOWN;
    }
}

/* ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ */

uint8_t sx1302_rx_status(uint8_t rf_chain) {
    if (rf_chain) {}; /* dummy for compilation */
    /* Not implemented */
    return RX_STATUS_UNKNOWN;
}

/* ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ */

int sx1302_tx_abort(uint8_t rf_chain) {
    int err;
    uint8_t tx_status = TX_STATUS_UNKNOWN;
    struct timeval tm_start;

    err  = lgw_reg_w(SX1302_REG_TX_TOP_TX_TRIG_TX_TRIG_IMMEDIATE(rf_chain), 0x00);
    err |= lgw_reg_w(SX1302_REG_TX_TOP_TX_TRIG_TX_TRIG_DELAYED(rf_chain), 0x00);
    err |= lgw_reg_w(SX1302_REG_TX_TOP_TX_TRIG_TX_TRIG_GPS(rf_chain), 0x00);
    if (err != LGW_REG_SUCCESS) {
        printf("ERROR: Failed to stop TX trigger\n");
        return err;
    }

    timeout_start(&tm_start);
    do {
        /* handle timeout */
        if (timeout_check(tm_start, 1000) != 0) {
            printf("ERROR: %s: TIMEOUT on TX abort\n", __FUNCTION__);
            return LGW_REG_ERROR;
        }

        /* get tx status */
        tx_status = sx1302_tx_status(rf_chain);
        wait_ms(1);
    } while (tx_status != TX_FREE);

    return LGW_REG_SUCCESS;
}

/* ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ */

int sx1302_tx_configure(lgw_radio_type_t radio_type) {
    int err = LGW_REG_SUCCESS;

    /* Select the TX destination interface */
    switch (radio_type) {
        case LGW_RADIO_TYPE_SX1250:
            /* Let AGC control PLL DIV (sx1250 only) */
            err |= lgw_reg_w(SX1302_REG_TX_TOP_A_TX_RFFE_IF_CTRL2_PLL_DIV_CTRL_AGC, 1);
            err |= lgw_reg_w(SX1302_REG_TX_TOP_B_TX_RFFE_IF_CTRL2_PLL_DIV_CTRL_AGC, 1);

            /* SX126x Tx RFFE */
            err |= lgw_reg_w(SX1302_REG_TX_TOP_A_TX_RFFE_IF_CTRL_TX_IF_DST, 0x01);
            err |= lgw_reg_w(SX1302_REG_TX_TOP_B_TX_RFFE_IF_CTRL_TX_IF_DST, 0x01);
            break;
        case LGW_RADIO_TYPE_SX1255:
        case LGW_RADIO_TYPE_SX1257:
            /* SX1255/57 Tx RFFE */
            err |= lgw_reg_w(SX1302_REG_TX_TOP_A_TX_RFFE_IF_CTRL_TX_IF_DST, 0x00);
            err |= lgw_reg_w(SX1302_REG_TX_TOP_B_TX_RFFE_IF_CTRL_TX_IF_DST, 0x00);
            break;
        default:
            DEBUG_MSG("ERROR: radio type not supported\n");
            return LGW_REG_ERROR;
    }

    /* Configure the TX mode of operation */
    err |= lgw_reg_w(SX1302_REG_TX_TOP_A_TX_RFFE_IF_CTRL_TX_MODE, 0x01); /* Modulation */
    err |= lgw_reg_w(SX1302_REG_TX_TOP_B_TX_RFFE_IF_CTRL_TX_MODE, 0x01); /* Modulation */

    /* Configure the output data clock edge */
    err |= lgw_reg_w(SX1302_REG_TX_TOP_A_TX_RFFE_IF_CTRL_TX_CLK_EDGE, 0x00); /* Data on rising edge */
    err |= lgw_reg_w(SX1302_REG_TX_TOP_B_TX_RFFE_IF_CTRL_TX_CLK_EDGE, 0x00); /* Data on rising edge */

    return err;
}

/* ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ */

int sx1302_send(lgw_radio_type_t radio_type, struct lgw_tx_gain_lut_s * tx_lut, bool lwan_public, struct lgw_conf_rxif_s * context_fsk, struct lgw_pkt_tx_s * pkt_data) {
    int err;
    uint32_t freq_reg, fdev_reg;
    uint32_t freq_dev;
    uint32_t fsk_br_reg;
    uint64_t fsk_sync_word_reg;
    uint16_t mem_addr;
    uint32_t count_us;
    uint8_t power;
    uint8_t pow_index;
    uint8_t mod_bw;
    uint8_t pa_en;
    uint16_t tx_start_delay;
    uint8_t chirp_lowpass = 0;
    uint8_t buff[2]; /* for 16-bits register write operation */
    /* performances variables */
    struct timeval tm;

    /* Record function start time */
    _meas_time_start(&tm);

    /* Check input parameters */
    CHECK_NULL(tx_lut);
    CHECK_NULL(pkt_data);

    /* Setting BULK write mode (to speed up configuration on USB) */
    err = lgw_com_set_write_mode(LGW_COM_WRITE_MODE_BULK);
    CHECK_ERR(err);

    /* Select the proper modem */
    switch (pkt_data->modulation) {
        case MOD_CW:
            err = lgw_reg_w(SX1302_REG_TX_TOP_GEN_CFG_0_MODULATION_TYPE(pkt_data->rf_chain), 0x00);
            CHECK_ERR(err);
            err = lgw_reg_w(SX1302_REG_TX_TOP_TX_RFFE_IF_CTRL_TX_IF_SRC(pkt_data->rf_chain), 0x00);
            CHECK_ERR(err);
            break;
        case MOD_LORA:
            err = lgw_reg_w(SX1302_REG_TX_TOP_GEN_CFG_0_MODULATION_TYPE(pkt_data->rf_chain), 0x00);
            CHECK_ERR(err);
            err = lgw_reg_w(SX1302_REG_TX_TOP_TX_RFFE_IF_CTRL_TX_IF_SRC(pkt_data->rf_chain), 0x01);
            CHECK_ERR(err);
            break;
        case MOD_FSK:
            err = lgw_reg_w(SX1302_REG_TX_TOP_GEN_CFG_0_MODULATION_TYPE(pkt_data->rf_chain), 0x01);
            CHECK_ERR(err);
            err = lgw_reg_w(SX1302_REG_TX_TOP_TX_RFFE_IF_CTRL_TX_IF_SRC(pkt_data->rf_chain), 0x02);
            CHECK_ERR(err);
            break;
        default:
            DEBUG_MSG("ERROR: modulation type not supported\n");
            return LGW_REG_ERROR;
    }

    /* Find the proper index in the TX gain LUT according to requested rf_power */
    for (pow_index = tx_lut->size-1; pow_index > 0; pow_index--) {
        if (tx_lut->lut[pow_index].rf_power <= pkt_data->rf_power) {
            break;
        }
    }
    DEBUG_PRINTF("INFO: selecting TX Gain LUT index %u\n", pow_index);

    /* loading calibrated Tx DC offsets */
    err = lgw_reg_w(SX1302_REG_TX_TOP_TX_RFFE_IF_I_OFFSET_I_OFFSET(pkt_data->rf_chain), tx_lut->lut[pow_index].offset_i);
    CHECK_ERR(err);
    err = lgw_reg_w(SX1302_REG_TX_TOP_TX_RFFE_IF_Q_OFFSET_Q_OFFSET(pkt_data->rf_chain), tx_lut->lut[pow_index].offset_q);
    CHECK_ERR(err);

    DEBUG_PRINTF("INFO: Applying IQ offset (i:%d, q:%d)\n", tx_lut->lut[pow_index].offset_i, tx_lut->lut[pow_index].offset_q);

    /* Set the power parameters to be used for TX */
    switch (radio_type) {
        case LGW_RADIO_TYPE_SX1250:
            pa_en = (tx_lut->lut[pow_index].pa_gain > 0) ? 1 : 0; /* only 1 bit used to control the external PA */
            power = (pa_en << 6) | tx_lut->lut[pow_index].pwr_idx;
            break;
        case LGW_RADIO_TYPE_SX1255:
        case LGW_RADIO_TYPE_SX1257:
            power = (tx_lut->lut[pow_index].pa_gain << 6) | (tx_lut->lut[pow_index].dac_gain << 4) | tx_lut->lut[pow_index].mix_gain;
            break;
        default:
            DEBUG_MSG("ERROR: radio type not supported\n");
            return LGW_REG_ERROR;
    }
    err = lgw_reg_w(SX1302_REG_TX_TOP_AGC_TX_PWR_AGC_TX_PWR(pkt_data->rf_chain), power);
    CHECK_ERR(err);

    /* Set digital gain */
    err = lgw_reg_w(SX1302_REG_TX_TOP_TX_RFFE_IF_IQ_GAIN_IQ_GAIN(pkt_data->rf_chain), tx_lut->lut[pow_index].dig_gain);
    CHECK_ERR(err);

    /* Set Tx frequency */
    if (radio_type == LGW_RADIO_TYPE_SX1255) {
        freq_reg = SX1302_FREQ_TO_REG(pkt_data->freq_hz * 2);
    } else {
        freq_reg = SX1302_FREQ_TO_REG(pkt_data->freq_hz);
    }
    err = lgw_reg_w(SX1302_REG_TX_TOP_TX_RFFE_IF_FREQ_RF_H_FREQ_RF(pkt_data->rf_chain), (freq_reg >> 16) & 0xFF);
    CHECK_ERR(err);
    err = lgw_reg_w(SX1302_REG_TX_TOP_TX_RFFE_IF_FREQ_RF_M_FREQ_RF(pkt_data->rf_chain), (freq_reg >> 8) & 0xFF);
    CHECK_ERR(err);
    err = lgw_reg_w(SX1302_REG_TX_TOP_TX_RFFE_IF_FREQ_RF_L_FREQ_RF(pkt_data->rf_chain), (freq_reg >> 0) & 0xFF);
    CHECK_ERR(err);

    /* Set AGC bandwidth and modulation type*/
    switch (pkt_data->modulation) {
        case MOD_LORA:
            mod_bw = pkt_data->bandwidth;
            break;
        case MOD_CW:
        case MOD_FSK:
            mod_bw = (0x01 << 7) | pkt_data->bandwidth;
            break;
        default:
            printf("ERROR: Modulation not supported\n");
            return LGW_REG_ERROR;
    }
    err = lgw_reg_w(SX1302_REG_TX_TOP_AGC_TX_BW_AGC_TX_BW(pkt_data->rf_chain), mod_bw);
    CHECK_ERR(err);

    /* Configure modem */
    switch (pkt_data->modulation) {
        case MOD_CW:
            /* Set frequency deviation */
            freq_dev = ceil(fabs( (float)pkt_data->freq_offset / 10) ) * 10e3;
            printf("CW: f_dev %d Hz\n", (int)(freq_dev));
            fdev_reg = SX1302_FREQ_TO_REG(freq_dev);
            err = lgw_reg_w(SX1302_REG_TX_TOP_TX_RFFE_IF_FREQ_DEV_H_FREQ_DEV(pkt_data->rf_chain), (fdev_reg >>  8) & 0xFF);
            CHECK_ERR(err);
            err = lgw_reg_w(SX1302_REG_TX_TOP_TX_RFFE_IF_FREQ_DEV_L_FREQ_DEV(pkt_data->rf_chain), (fdev_reg >>  0) & 0xFF);
            CHECK_ERR(err);

            /* Send frequency deviation to AGC fw for radio config */
            fdev_reg = SX1250_FREQ_TO_REG(freq_dev);
            err = lgw_reg_w(SX1302_REG_AGC_MCU_MCU_MAIL_BOX_WR_DATA_BYTE2_MCU_MAIL_BOX_WR_DATA, (fdev_reg >> 16) & 0xFF); /* Needed by AGC to configure the sx1250 */
            CHECK_ERR(err);
            err = lgw_reg_w(SX1302_REG_AGC_MCU_MCU_MAIL_BOX_WR_DATA_BYTE1_MCU_MAIL_BOX_WR_DATA, (fdev_reg >>  8) & 0xFF); /* Needed by AGC to configure the sx1250 */
            CHECK_ERR(err);
            err = lgw_reg_w(SX1302_REG_AGC_MCU_MCU_MAIL_BOX_WR_DATA_BYTE0_MCU_MAIL_BOX_WR_DATA, (fdev_reg >>  0) & 0xFF); /* Needed by AGC to configure the sx1250 */
            CHECK_ERR(err);

            /* Set the frequency offset (ratio of the frequency deviation)*/
            printf("CW: IF test mod freq %d\n", (int)(((float)pkt_data->freq_offset*1e3*64/(float)freq_dev)));
            err = lgw_reg_w(SX1302_REG_TX_TOP_TX_RFFE_IF_TEST_MOD_FREQ(pkt_data->rf_chain), (int)(((float)pkt_data->freq_offset*1e3*64/(float)freq_dev)));
            CHECK_ERR(err);
            break;
        case MOD_LORA:
            /* Set bandwidth */
            freq_dev = lgw_bw_getval(pkt_data->bandwidth) / 2;
            fdev_reg = SX1302_FREQ_TO_REG(freq_dev);
            err = lgw_reg_w(SX1302_REG_TX_TOP_TX_RFFE_IF_FREQ_DEV_H_FREQ_DEV(pkt_data->rf_chain), (fdev_reg >>  8) & 0xFF);
            CHECK_ERR(err);
            err = lgw_reg_w(SX1302_REG_TX_TOP_TX_RFFE_IF_FREQ_DEV_L_FREQ_DEV(pkt_data->rf_chain), (fdev_reg >>  0) & 0xFF);
            CHECK_ERR(err);
            err = lgw_reg_w(SX1302_REG_TX_TOP_TXRX_CFG0_0_MODEM_BW(pkt_data->rf_chain), pkt_data->bandwidth);
            CHECK_ERR(err);

            /* Preamble length */
            if (pkt_data->preamble == 0) { /* if not explicit, use recommended LoRa preamble size */
                pkt_data->preamble = STD_LORA_PREAMBLE;
            } else if (pkt_data->preamble < MIN_LORA_PREAMBLE) { /* enforce minimum preamble size */
                pkt_data->preamble = MIN_LORA_PREAMBLE;
                DEBUG_MSG("Note: preamble length adjusted to respect minimum LoRa preamble size\n");
            }
            err = lgw_reg_w(SX1302_REG_TX_TOP_TXRX_CFG1_3_PREAMBLE_SYMB_NB(pkt_data->rf_chain), (pkt_data->preamble >> 8) & 0xFF); /* MSB */
            CHECK_ERR(err);
            err = lgw_reg_w(SX1302_REG_TX_TOP_TXRX_CFG1_2_PREAMBLE_SYMB_NB(pkt_data->rf_chain), (pkt_data->preamble >> 0) & 0xFF); /* LSB */
            CHECK_ERR(err);

            /* LoRa datarate */
            err = lgw_reg_w(SX1302_REG_TX_TOP_TXRX_CFG0_0_MODEM_SF(pkt_data->rf_chain), pkt_data->datarate);
            CHECK_ERR(err);

            /* Chirp filtering */
            chirp_lowpass = (pkt_data->datarate < 10) ? 6 : 7;
            err = lgw_reg_w(SX1302_REG_TX_TOP_TX_CFG0_0_CHIRP_LOWPASS(pkt_data->rf_chain), (int32_t)chirp_lowpass);
            CHECK_ERR(err);

            /* Coding Rate */
            err = lgw_reg_w(SX1302_REG_TX_TOP_TXRX_CFG0_1_CODING_RATE(pkt_data->rf_chain), pkt_data->coderate);
            CHECK_ERR(err);

            /* Start LoRa modem */
            err = lgw_reg_w(SX1302_REG_TX_TOP_TXRX_CFG0_2_MODEM_EN(pkt_data->rf_chain), 1);
            CHECK_ERR(err);
            err = lgw_reg_w(SX1302_REG_TX_TOP_TXRX_CFG0_2_CADRXTX(pkt_data->rf_chain), 2);
            CHECK_ERR(err);
            err = lgw_reg_w(SX1302_REG_TX_TOP_TXRX_CFG1_1_MODEM_START(pkt_data->rf_chain), 1);
            CHECK_ERR(err);
            err = lgw_reg_w(SX1302_REG_TX_TOP_TX_CFG0_0_CONTINUOUS(pkt_data->rf_chain), 0);
            CHECK_ERR(err);

            /* Modulation options */
            err = lgw_reg_w(SX1302_REG_TX_TOP_TX_CFG0_0_CHIRP_INVERT(pkt_data->rf_chain), (pkt_data->invert_pol) ? 1 : 0);
            CHECK_ERR(err);
            err = lgw_reg_w(SX1302_REG_TX_TOP_TXRX_CFG0_2_IMPLICIT_HEADER(pkt_data->rf_chain), (pkt_data->no_header) ? 1 : 0);
            CHECK_ERR(err);
            err = lgw_reg_w(SX1302_REG_TX_TOP_TXRX_CFG0_2_CRC_EN(pkt_data->rf_chain), (pkt_data->no_crc) ? 0 : 1);
            CHECK_ERR(err);

            /* Syncword */
            if ((lwan_public == false) || (pkt_data->datarate == DR_LORA_SF5) || (pkt_data->datarate == DR_LORA_SF6)) {
                DEBUG_MSG("Setting LoRa syncword 0x12\n");
                err = lgw_reg_w(SX1302_REG_TX_TOP_FRAME_SYNCH_0_PEAK1_POS(pkt_data->rf_chain), 2);
                CHECK_ERR(err);
                err = lgw_reg_w(SX1302_REG_TX_TOP_FRAME_SYNCH_1_PEAK2_POS(pkt_data->rf_chain), 4);
                CHECK_ERR(err);
            } else {
                DEBUG_MSG("Setting LoRa syncword 0x34\n");
                err = lgw_reg_w(SX1302_REG_TX_TOP_FRAME_SYNCH_0_PEAK1_POS(pkt_data->rf_chain), 6);
                CHECK_ERR(err);
                err = lgw_reg_w(SX1302_REG_TX_TOP_FRAME_SYNCH_1_PEAK2_POS(pkt_data->rf_chain), 8);
                CHECK_ERR(err);
            }

            /* Set Fine Sync for SF5/SF6 */
            if ((pkt_data->datarate == DR_LORA_SF5) || (pkt_data->datarate == DR_LORA_SF6)) {
                DEBUG_MSG("Enable Fine Sync\n");
                err = lgw_reg_w(SX1302_REG_TX_TOP_TXRX_CFG0_2_FINE_SYNCH_EN(pkt_data->rf_chain), 1);
                CHECK_ERR(err);
            } else {
                DEBUG_MSG("Disable Fine Sync\n");
                err = lgw_reg_w(SX1302_REG_TX_TOP_TXRX_CFG0_2_FINE_SYNCH_EN(pkt_data->rf_chain), 0);
                CHECK_ERR(err);
            }

            /* Set Payload length */
            err = lgw_reg_w(SX1302_REG_TX_TOP_TXRX_CFG0_3_PAYLOAD_LENGTH(pkt_data->rf_chain), pkt_data->size);
            CHECK_ERR(err);

            /* Set PPM offset (low datarate optimization) */
            err = lgw_reg_w(SX1302_REG_TX_TOP_TXRX_CFG0_1_PPM_OFFSET_HDR_CTRL(pkt_data->rf_chain), 0);
            CHECK_ERR(err);
            if (SET_PPM_ON(pkt_data->bandwidth, pkt_data->datarate)) {
                DEBUG_MSG("Low datarate optimization ENABLED\n");
                err = lgw_reg_w(SX1302_REG_TX_TOP_TXRX_CFG0_1_PPM_OFFSET(pkt_data->rf_chain), 1);
                CHECK_ERR(err);
            } else {
                DEBUG_MSG("Low datarate optimization DISABLED\n");
                err = lgw_reg_w(SX1302_REG_TX_TOP_TXRX_CFG0_1_PPM_OFFSET(pkt_data->rf_chain), 0);
                CHECK_ERR(err);
            }
            break;
        case MOD_FSK:
            CHECK_NULL(context_fsk);

            /* Set frequency deviation */
            freq_dev = pkt_data->f_dev * 1e3;
            fdev_reg = SX1302_FREQ_TO_REG(freq_dev);
            err = lgw_reg_w(SX1302_REG_TX_TOP_TX_RFFE_IF_FREQ_DEV_H_FREQ_DEV(pkt_data->rf_chain), (fdev_reg >>  8) & 0xFF);
            CHECK_ERR(err);
            err = lgw_reg_w(SX1302_REG_TX_TOP_TX_RFFE_IF_FREQ_DEV_L_FREQ_DEV(pkt_data->rf_chain), (fdev_reg >>  0) & 0xFF);
            CHECK_ERR(err);

            /* Send frequency deviation to AGC fw for radio config */
            fdev_reg = SX1250_FREQ_TO_REG(freq_dev);
            err = lgw_reg_w(SX1302_REG_AGC_MCU_MCU_MAIL_BOX_WR_DATA_BYTE2_MCU_MAIL_BOX_WR_DATA, (fdev_reg >> 16) & 0xFF); /* Needed by AGC to configure the sx1250 */
            CHECK_ERR(err);
            err = lgw_reg_w(SX1302_REG_AGC_MCU_MCU_MAIL_BOX_WR_DATA_BYTE1_MCU_MAIL_BOX_WR_DATA, (fdev_reg >>  8) & 0xFF); /* Needed by AGC to configure the sx1250 */
            CHECK_ERR(err);
            err = lgw_reg_w(SX1302_REG_AGC_MCU_MCU_MAIL_BOX_WR_DATA_BYTE0_MCU_MAIL_BOX_WR_DATA, (fdev_reg >>  0) & 0xFF); /* Needed by AGC to configure the sx1250 */
            CHECK_ERR(err);

            /* Modulation parameters */
            err = lgw_reg_w(SX1302_REG_TX_TOP_FSK_CFG_0_PKT_MODE(pkt_data->rf_chain), 1); /* Variable length */
            CHECK_ERR(err);
            err = lgw_reg_w(SX1302_REG_TX_TOP_FSK_CFG_0_CRC_EN(pkt_data->rf_chain), (pkt_data->no_crc) ? 0 : 1);
            CHECK_ERR(err);
            err = lgw_reg_w(SX1302_REG_TX_TOP_FSK_CFG_0_CRC_IBM(pkt_data->rf_chain), 0); /* CCITT CRC */
            CHECK_ERR(err);
            err = lgw_reg_w(SX1302_REG_TX_TOP_FSK_CFG_0_DCFREE_ENC(pkt_data->rf_chain), 2); /* Whitening Encoding */
            CHECK_ERR(err);
            err = lgw_reg_w(SX1302_REG_TX_TOP_FSK_MOD_FSK_GAUSSIAN_EN(pkt_data->rf_chain), 1);
            CHECK_ERR(err);
            err = lgw_reg_w(SX1302_REG_TX_TOP_FSK_MOD_FSK_GAUSSIAN_SELECT_BT(pkt_data->rf_chain), 2);
            CHECK_ERR(err);
            err = lgw_reg_w(SX1302_REG_TX_TOP_FSK_MOD_FSK_REF_PATTERN_EN(pkt_data->rf_chain), 1);
            CHECK_ERR(err);
            err = lgw_reg_w(SX1302_REG_TX_TOP_FSK_MOD_FSK_REF_PATTERN_SIZE(pkt_data->rf_chain), context_fsk->sync_word_size - 1);
            CHECK_ERR(err);

            /* Syncword */
            fsk_sync_word_reg = context_fsk->sync_word << (8 * (8 - context_fsk->sync_word_size));
            err = lgw_reg_w(SX1302_REG_TX_TOP_FSK_REF_PATTERN_BYTE0_FSK_REF_PATTERN(pkt_data->rf_chain), (uint8_t)(fsk_sync_word_reg >> 0));
            CHECK_ERR(err);
            err = lgw_reg_w(SX1302_REG_TX_TOP_FSK_REF_PATTERN_BYTE1_FSK_REF_PATTERN(pkt_data->rf_chain), (uint8_t)(fsk_sync_word_reg >> 8));
            CHECK_ERR(err);
            err = lgw_reg_w(SX1302_REG_TX_TOP_FSK_REF_PATTERN_BYTE2_FSK_REF_PATTERN(pkt_data->rf_chain), (uint8_t)(fsk_sync_word_reg >> 16));
            CHECK_ERR(err);
            err = lgw_reg_w(SX1302_REG_TX_TOP_FSK_REF_PATTERN_BYTE3_FSK_REF_PATTERN(pkt_data->rf_chain), (uint8_t)(fsk_sync_word_reg >> 24));
            CHECK_ERR(err);
            err = lgw_reg_w(SX1302_REG_TX_TOP_FSK_REF_PATTERN_BYTE4_FSK_REF_PATTERN(pkt_data->rf_chain), (uint8_t)(fsk_sync_word_reg >> 32));
            CHECK_ERR(err);
            err = lgw_reg_w(SX1302_REG_TX_TOP_FSK_REF_PATTERN_BYTE5_FSK_REF_PATTERN(pkt_data->rf_chain), (uint8_t)(fsk_sync_word_reg >> 40));
            CHECK_ERR(err);
            err = lgw_reg_w(SX1302_REG_TX_TOP_FSK_REF_PATTERN_BYTE6_FSK_REF_PATTERN(pkt_data->rf_chain), (uint8_t)(fsk_sync_word_reg >> 48));
            CHECK_ERR(err);
            err = lgw_reg_w(SX1302_REG_TX_TOP_FSK_REF_PATTERN_BYTE7_FSK_REF_PATTERN(pkt_data->rf_chain), (uint8_t)(fsk_sync_word_reg >> 56));
            CHECK_ERR(err);
            err = lgw_reg_w(SX1302_REG_TX_TOP_FSK_MOD_FSK_PREAMBLE_SEQ(pkt_data->rf_chain), 0);
            CHECK_ERR(err);

            /* Set datarate */
            fsk_br_reg = 32000000 / pkt_data->datarate;
            buff[0] = (uint8_t)(fsk_br_reg >> 8);
            buff[1] = (uint8_t)(fsk_br_reg >> 0);
            err = lgw_reg_wb(SX1302_REG_TX_TOP_FSK_BIT_RATE_MSB_BIT_RATE(pkt_data->rf_chain), buff, 2);
            CHECK_ERR(err);

            /* Preamble length */
            if (pkt_data->preamble == 0) { /* if not explicit, use LoRaWAN preamble size */
                pkt_data->preamble = STD_FSK_PREAMBLE;
            } else if (pkt_data->preamble < MIN_FSK_PREAMBLE) { /* enforce minimum preamble size */
                pkt_data->preamble = MIN_FSK_PREAMBLE;
                DEBUG_MSG("Note: preamble length adjusted to respect minimum FSK preamble size\n");
            }
            buff[0] = (uint8_t)(pkt_data->preamble >> 8);
            buff[1] = (uint8_t)(pkt_data->preamble >> 0);
            err = lgw_reg_wb(SX1302_REG_TX_TOP_FSK_PREAMBLE_SIZE_MSB_PREAMBLE_SIZE(pkt_data->rf_chain), buff, 2);
            CHECK_ERR(err);

            /* Set Payload length */
            err = lgw_reg_w(SX1302_REG_TX_TOP_FSK_PKT_LEN_PKT_LENGTH(pkt_data->rf_chain), pkt_data->size);
            CHECK_ERR(err);
            break;
        default:
            printf("ERROR: Modulation not supported\n");
            return LGW_REG_ERROR;
    }

    /* Set TX start delay */
    err = sx1302_tx_set_start_delay(pkt_data->rf_chain, radio_type, pkt_data->modulation, pkt_data->bandwidth, chirp_lowpass, &tx_start_delay);
    CHECK_ERR(err);

    /* Write payload in transmit buffer */
    err = lgw_reg_w(SX1302_REG_TX_TOP_TX_CTRL_WRITE_BUFFER(pkt_data->rf_chain), 0x01);
    CHECK_ERR(err);
    mem_addr = REG_SELECT(pkt_data->rf_chain, 0x5300, 0x5500);
    if (pkt_data->modulation == MOD_FSK) {
        err = lgw_mem_wb(mem_addr, (uint8_t *)(&(pkt_data->size)), 1); /* insert payload size in the packet for FSK variable mode (1 byte) */
        CHECK_ERR(err);
        err = lgw_mem_wb(mem_addr+1, &(pkt_data->payload[0]), pkt_data->size);
        CHECK_ERR(err);
    } else {
        err = lgw_mem_wb(mem_addr, &(pkt_data->payload[0]), pkt_data->size);
        CHECK_ERR(err);
    }
    err = lgw_reg_w(SX1302_REG_TX_TOP_TX_CTRL_WRITE_BUFFER(pkt_data->rf_chain), 0x00);
    CHECK_ERR(err);

    /* Trigger transmit */
    DEBUG_PRINTF("Start Tx: Freq:%u %s%u size:%u preamb:%u\n", pkt_data->freq_hz, (pkt_data->modulation == MOD_LORA) ? "SF" : "DR:", pkt_data->datarate, pkt_data->size, pkt_data->preamble);
    switch (pkt_data->tx_mode) {
        case IMMEDIATE:
            err = lgw_reg_w(SX1302_REG_TX_TOP_TX_TRIG_TX_TRIG_IMMEDIATE(pkt_data->rf_chain), 0x00); /* reset state machine */
            CHECK_ERR(err);
            err = lgw_reg_w(SX1302_REG_TX_TOP_TX_TRIG_TX_TRIG_IMMEDIATE(pkt_data->rf_chain), 0x01);
            CHECK_ERR(err);
            break;
        case TIMESTAMPED:
            count_us = pkt_data->count_us * 32 - tx_start_delay;
            DEBUG_PRINTF("--> programming trig delay at %u (%u)\n", pkt_data->count_us - (tx_start_delay / 32), count_us);

            err = lgw_reg_w(SX1302_REG_TX_TOP_TIMER_TRIG_BYTE0_TIMER_DELAYED_TRIG(pkt_data->rf_chain), (uint8_t)((count_us >>  0) & 0x000000FF));
            CHECK_ERR(err);
            err = lgw_reg_w(SX1302_REG_TX_TOP_TIMER_TRIG_BYTE1_TIMER_DELAYED_TRIG(pkt_data->rf_chain), (uint8_t)((count_us >>  8) & 0x000000FF));
            CHECK_ERR(err);
            err = lgw_reg_w(SX1302_REG_TX_TOP_TIMER_TRIG_BYTE2_TIMER_DELAYED_TRIG(pkt_data->rf_chain), (uint8_t)((count_us >> 16) & 0x000000FF));
            CHECK_ERR(err);
            err = lgw_reg_w(SX1302_REG_TX_TOP_TIMER_TRIG_BYTE3_TIMER_DELAYED_TRIG(pkt_data->rf_chain), (uint8_t)((count_us >> 24) & 0x000000FF));
            CHECK_ERR(err);

            err = lgw_reg_w(SX1302_REG_TX_TOP_TX_TRIG_TX_TRIG_DELAYED(pkt_data->rf_chain), 0x00); /* reset state machine */
            CHECK_ERR(err);
            err = lgw_reg_w(SX1302_REG_TX_TOP_TX_TRIG_TX_TRIG_DELAYED(pkt_data->rf_chain), 0x01);
            CHECK_ERR(err);
            break;
        case ON_GPS:
            err = lgw_reg_w(SX1302_REG_TX_TOP_TX_TRIG_TX_TRIG_GPS(pkt_data->rf_chain), 0x00); /* reset state machine */
            CHECK_ERR(err);
            err = lgw_reg_w(SX1302_REG_TX_TOP_TX_TRIG_TX_TRIG_GPS(pkt_data->rf_chain), 0x01);
            CHECK_ERR(err);
            break;
        default:
            printf("ERROR: TX mode not supported\n");
            return LGW_REG_ERROR;
    }

    /* Flush write (USB BULK mode) */
    err = lgw_com_flush();
    CHECK_ERR(err);

    /* Setting back to SINGLE BULK write mode */
    err = lgw_com_set_write_mode(LGW_COM_WRITE_MODE_SINGLE);
    CHECK_ERR(err);

    /* Compute time spent in this function */
    _meas_time_stop(2, tm, __FUNCTION__);

    return LGW_REG_SUCCESS;
}

/* ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ */

int sx1302_set_gpio(uint8_t gpio_reg_val) {
    int err;

    err = lgw_reg_w(SX1302_REG_GPIO_GPIO_OUT_L_OUT_VALUE, gpio_reg_val); /* set all GPIOs at once, 1 bit per GPIO */
    CHECK_ERR(err);

    return LGW_REG_SUCCESS;
}

/* ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ */

double sx1302_dc_notch_delay(double if_freq_khz) {
    double delay;

    if ((if_freq_khz < -75.0) || (if_freq_khz > 75.0)) {
        delay = 0.0;
    } else {
        delay = 1.7e-6 * pow(if_freq_khz, 4) + 2.4e-6 * pow(if_freq_khz, 3) - 0.0101 * pow(if_freq_khz, 2) - 0.01275 * if_freq_khz + 10.2922;
    }

    /* Number of 32MHz clock cycles */
    return delay;
}

/* --- EOF ------------------------------------------------------------------ */
