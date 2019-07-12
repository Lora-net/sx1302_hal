/*
 / _____)             _              | |
( (____  _____ ____ _| |_ _____  ____| |__
 \____ \| ___ |    (_   _) ___ |/ ___)  _ \
 _____) ) ____| | | || |_| ____( (___| | | |
(______/|_____)_|_|_| \__)_____)\____)_| |_|
  (C)2019 Semtech

Description:
    SX1302 timestamp counter Hardware Abstraction Layer
    Handles the conversion of a 32-bits 32MHz counter into a 32-bits 1 MHz counter.
    This modules MUST be called regularly by the application to maintain counter
    wrapping handling for conversion in 1MHz counter.
    Provides function to compute the correction to be applied to the received
    timestamp for demodulation processing time.

License: Revised BSD License, see LICENSE.TXT file include in the project
*/


/* -------------------------------------------------------------------------- */
/* --- DEPENDANCIES --------------------------------------------------------- */

#include <stdint.h>     /* C99 types */
#include <stdio.h>      /* printf fprintf */

#include "loragw_sx1302_timestamp.h"
#include "loragw_reg.h"
#include "loragw_hal.h"
#include "loragw_sx1302.h"

/* -------------------------------------------------------------------------- */
/* --- PRIVATE MACROS ------------------------------------------------------- */

#define ARRAY_SIZE(a) (sizeof(a) / sizeof((a)[0]))
#if DEBUG_SX1302 == 1
    #define DEBUG_MSG(str)                fprintf(stderr, str)
    #define DEBUG_PRINTF(fmt, args...)    fprintf(stderr, fmt, args)
    #define CHECK_NULL(a)                if(a==NULL){fprintf(stderr,"%s:%d: ERROR: NULL POINTER AS ARGUMENT\n", __FUNCTION__, __LINE__);return LGW_REG_ERROR;}
#else
    #define DEBUG_MSG(str)
    #define DEBUG_PRINTF(fmt, args...)
    #define CHECK_NULL(a)                if(a==NULL){return LGW_REG_ERROR;}
#endif

/* -------------------------------------------------------------------------- */
/* --- PRIVATE TYPES -------------------------------------------------------- */

/* -------------------------------------------------------------------------- */
/* --- PRIVATE CONSTANTS ---------------------------------------------------- */

/* -------------------------------------------------------------------------- */
/* --- PRIVATE VARIABLES ---------------------------------------------------- */

/* -------------------------------------------------------------------------- */
/* --- PRIVATE FUNCTIONS DECLARATION ---------------------------------------- */

/* -------------------------------------------------------------------------- */
/* --- INTERNAL SHARED VARIABLES -------------------------------------------- */

/* -------------------------------------------------------------------------- */
/* --- PRIVATE FUNCTIONS DEFINITION ----------------------------------------- */

/* -------------------------------------------------------------------------- */
/* --- PUBLIC FUNCTIONS DEFINITION ------------------------------------------ */

void timestamp_counter_new(timestamp_counter_t * self) {
    self->counter_us_raw_27bits_inst_prev = 0;
    self->counter_us_raw_27bits_pps_prev = 0;
    self->counter_us_raw_27bits_inst_wrap = 0;
    self->counter_us_raw_27bits_pps_wrap = 0;
}

/* ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ */

void timestamp_counter_delete(timestamp_counter_t * self) {
    self->counter_us_raw_27bits_inst_prev = 0;
    self->counter_us_raw_27bits_pps_prev = 0;
    self->counter_us_raw_27bits_inst_wrap = 0;
    self->counter_us_raw_27bits_pps_wrap = 0;
}

/* ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ */

void timestamp_counter_update(timestamp_counter_t * self, bool pps, uint32_t cnt) {
    uint32_t counter_us_raw_27bits_prev;
    uint8_t  counter_us_raw_27bits_wrap;

    /* Get the previous counter value and wrap status */
    if (pps == true) {
        counter_us_raw_27bits_prev = self->counter_us_raw_27bits_pps_prev;
        counter_us_raw_27bits_wrap = self->counter_us_raw_27bits_pps_wrap;
    } else {
        counter_us_raw_27bits_prev = self->counter_us_raw_27bits_inst_prev;
        counter_us_raw_27bits_wrap = self->counter_us_raw_27bits_inst_wrap;
    }

    /* Check if counter has wrapped, and update wrap status if necessary */
    if (cnt < counter_us_raw_27bits_prev) {
        counter_us_raw_27bits_wrap += 1;
        counter_us_raw_27bits_wrap = counter_us_raw_27bits_wrap % 32;
    }

    /* Store counter value and wrap status for next time */
    if (pps == true) {
        self->counter_us_raw_27bits_pps_prev = cnt;
        self->counter_us_raw_27bits_pps_wrap = counter_us_raw_27bits_wrap;
    } else {
        self->counter_us_raw_27bits_inst_prev = cnt;
        self->counter_us_raw_27bits_inst_wrap = counter_us_raw_27bits_wrap;
    }
}

/* ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ */

uint32_t timestamp_counter_get(timestamp_counter_t * self, bool pps) {
    int x;
    uint8_t buff[4];
    uint32_t counter_us_raw_27bits_now;

    /* Get the 32MHz timestamp counter - 4 bytes */
    /* step of 31.25 ns */
    x = lgw_reg_rb((pps == true) ? SX1302_REG_TIMESTAMP_TIMESTAMP_PPS_MSB2_TIMESTAMP_PPS : SX1302_REG_TIMESTAMP_TIMESTAMP_MSB2_TIMESTAMP, &buff[0], 4);
    if (x != LGW_REG_SUCCESS) {
        printf("ERROR: Failed to get timestamp counter value\n");
        return 0;
    }

    counter_us_raw_27bits_now  = (uint32_t)((buff[0] << 24) & 0xFF000000);
    counter_us_raw_27bits_now |= (uint32_t)((buff[1] << 16) & 0x00FF0000);
    counter_us_raw_27bits_now |= (uint32_t)((buff[2] << 8)  & 0x0000FF00);
    counter_us_raw_27bits_now |= (uint32_t)((buff[3] << 0)  & 0x000000FF);
    counter_us_raw_27bits_now /= 32; /* scale to 1MHz */

    /* Update counter wrapping status */
    timestamp_counter_update(self, pps, counter_us_raw_27bits_now);

    /* Convert 27-bits counter to 32-bits counter */
    return timestamp_counter_expand(self, pps, counter_us_raw_27bits_now);
}

/* ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ */

uint32_t timestamp_counter_expand(timestamp_counter_t * self, bool pps, uint32_t cnt_us) {
    uint32_t counter_us_32bits;

    if (pps == true) {
        counter_us_32bits = (self->counter_us_raw_27bits_pps_wrap << 27) | cnt_us;
    } else {
        counter_us_32bits = (self->counter_us_raw_27bits_inst_wrap << 27) | cnt_us;
    }

#if 0
    /* DEBUG: to be enabled when running test_loragw_counter test application
       This generates a CSV log, and can be plotted with gnuplot:
        > set datafile separator comma
        > plot for [col=1:2:1] 'log_count.txt' using col with lines
    */
    printf("%u,%u,%u\n", cnt_us, counter_us_32bits, (pps == true) ? self->counter_us_raw_27bits_pps_wrap : self->counter_us_raw_27bits_inst_wrap);
#endif

    return counter_us_32bits;
}

/* ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ */

int timestamp_counter_mode(bool enable_precision_ts, uint8_t max_ts_metrics, uint8_t nb_symbols) {
    if (enable_precision_ts == false) {
        DEBUG_MSG("INFO: using legacy timestamp\n");
        /* Latch end-of-packet timestamp (sx1301 compatibility) */
        lgw_reg_w(SX1302_REG_RX_TOP_RX_BUFFER_LEGACY_TIMESTAMP, 0x01);
    } else {
        DEBUG_PRINTF("INFO: using precision timestamp (max_ts_metrics:%u nb_symbols:%u)\n", max_ts_metrics, nb_symbols);
        /* Latch end-of-preamble timestamp */
        lgw_reg_w(SX1302_REG_RX_TOP_RX_BUFFER_LEGACY_TIMESTAMP, 0x00);
        lgw_reg_w(SX1302_REG_RX_TOP_RX_BUFFER_TIMESTAMP_CFG_MAX_TS_METRICS, max_ts_metrics);

        /* LoRa multi-SF modems */
        lgw_reg_w(SX1302_REG_RX_TOP_TIMESTAMP_ENABLE, 0x01);
        lgw_reg_w(SX1302_REG_RX_TOP_TIMESTAMP_NB_SYMB, nb_symbols);

        /* LoRa service modem */
        lgw_reg_w(SX1302_REG_RX_TOP_LORA_SERVICE_FSK_TIMESTAMP_ENABLE, 0x01);
        lgw_reg_w(SX1302_REG_RX_TOP_LORA_SERVICE_FSK_TIMESTAMP_NB_SYMB, nb_symbols);
    }

    return LGW_REG_SUCCESS;
}

/* ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ */

uint32_t timestamp_counter_correction(int ifmod, uint8_t bandwidth, uint8_t datarate, uint8_t coderate, uint32_t crc_en, uint16_t payload_length) {
    int32_t val;
    uint32_t sf = (uint32_t)datarate, cr = (uint32_t)coderate, bw_pow, ppm;
    uint32_t clk_period;
    uint32_t nb_nibble, nb_nibble_in_hdr, nb_nibble_in_last_block;
    uint32_t dft_peak_en, nb_iter;
    uint32_t demap_delay, decode_delay, fft_delay_state3, fft_delay, delay_x;
    uint32_t timestamp_correction;

    /* determine if 'PPM mode' is on */
    if (SET_PPM_ON(bandwidth, datarate)) {
        ppm = 1;
    } else {
        ppm = 0;
    }

    /* timestamp correction code, base delay */
    switch (bandwidth)
    {
        case BW_125KHZ:
            bw_pow = 1;
            delay_x = 16000000 / bw_pow + 2031250;
            break;
        case BW_250KHZ:
            bw_pow = 2;
            delay_x = 16000000 / bw_pow + 2031250;
            break;
        case BW_500KHZ:
            bw_pow = 4;
            delay_x = 16000000 / bw_pow + 2031250;
            break;
        default:
            DEBUG_PRINTF("ERROR: UNEXPECTED VALUE %d IN SWITCH STATEMENT\n", bandwidth);
            delay_x = 0;
            bw_pow = 0;
            break;
    }
    clk_period = 250000;

    nb_nibble = (payload_length + 2 * crc_en) * 2 + 5;

    if ((sf == 5) || (sf == 6)) {
        nb_nibble_in_hdr = sf;
    } else {
        nb_nibble_in_hdr = sf - 2;
    }

    nb_nibble_in_last_block = nb_nibble - nb_nibble_in_hdr - (sf - 2 * ppm) * ((nb_nibble - nb_nibble_in_hdr) / (sf - 2 * ppm));
    if (nb_nibble_in_last_block == 0) {
        nb_nibble_in_last_block = sf - 2 * ppm;
    }

    nb_iter = ((sf + 1) >> 1);

    /* timestamp correction code, variable delay */
    if (ifmod == IF_LORA_STD) {
        lgw_reg_r(SX1302_REG_RX_TOP_LORA_SERVICE_FSK_RX_CFG0_DFT_PEAK_EN, &val);
    } else {
        lgw_reg_r(SX1302_REG_RX_TOP_RX_CFG0_DFT_PEAK_EN, &val);
    }
    if (val != 0) {
        /* TODO: should we differentiate the mode (FULL/TRACK) ? */
        dft_peak_en = 1;
    } else {
        dft_peak_en = 0;
    }


    if ((sf >= 5) && (sf <= 12) && (bw_pow > 0)) {
        if ((2 * (payload_length + 2 * crc_en) - (sf - 7)) <= 0) { /* payload fits entirely in first 8 symbols (header) */
            if (sf > 6) {
                nb_nibble_in_last_block = sf - 2;
            } else {
                nb_nibble_in_last_block = sf; // can't be acheived
            }
            dft_peak_en = 0;
            cr = 4; /* header coding rate is 4 */
            demap_delay = clk_period + (1 << sf) * clk_period * 3 / 4 + 3 * clk_period + (sf - 2) * clk_period;
        } else {
            demap_delay = clk_period + (1 << sf) * clk_period * (1 - ppm / 4) + 3 * clk_period + (sf - 2 * ppm) * clk_period;
        }

        fft_delay_state3 = clk_period * (((1 << sf) - 6) + 2 * ((1 << sf) * (nb_iter - 1) + 6)) + 4 * clk_period;

        if (dft_peak_en) {
            fft_delay = (5 - 2 * ppm) * ((1 << sf) * clk_period + 7 * clk_period) + 2 * clk_period;
        } else {
            fft_delay = (1 << sf) * 2 * clk_period + 3 * clk_period;
        }

        decode_delay = 5 * clk_period + (9 * clk_period + clk_period * cr) * nb_nibble_in_last_block + 3 * clk_period;
        timestamp_correction = (uint32_t)(delay_x + fft_delay_state3 + fft_delay + demap_delay + decode_delay + 0.5e6) / 1e6;
        //printf("INFO: timestamp_correction = %u us (delay_x %u, fft_delay_state3=%u, fft_delay=%u, demap_delay=%u, decode_delay = %u)\n", timestamp_correction, delay_x, fft_delay_state3, fft_delay, demap_delay, decode_delay);
    }
    else
    {
        timestamp_correction = 0;
        DEBUG_MSG("WARNING: invalid packet, no timestamp correction\n");
    }

    return timestamp_correction;
}

/* --- EOF ------------------------------------------------------------------ */
