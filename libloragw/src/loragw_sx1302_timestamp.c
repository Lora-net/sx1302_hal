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
#include <stdbool.h>    /* boolean type */
#include <stdio.h>      /* printf fprintf */
#include <memory.h>     /* memset */
#include <inttypes.h>   /* PRIx64, PRIu64... */
#include <assert.h>

#include "loragw_sx1302_timestamp.h"
#include "loragw_reg.h"
#include "loragw_aux.h"

/* -------------------------------------------------------------------------- */
/* --- PRIVATE MACROS ------------------------------------------------------- */

#define ARRAY_SIZE(a) (sizeof(a) / sizeof((a)[0]))
#if DEBUG_FTIME == 1
    #define DEBUG_MSG(str)                fprintf(stdout, str)
    #define DEBUG_PRINTF(fmt, args...)    fprintf(stdout, fmt, args)
    #define CHECK_NULL(a)                if(a==NULL){fprintf(stderr,"%s:%d: ERROR: NULL POINTER AS ARGUMENT\n", __FUNCTION__, __LINE__);return LGW_REG_ERROR;}
#else
    #define DEBUG_MSG(str)
    #define DEBUG_PRINTF(fmt, args...)
    #define CHECK_NULL(a)                if(a==NULL){return LGW_REG_ERROR;}
#endif

/* -------------------------------------------------------------------------- */
/* --- PRIVATE TYPES -------------------------------------------------------- */

#define MAX_TIMESTAMP_PPS_HISTORY 16
struct timestamp_pps_history_s {
    uint32_t history[MAX_TIMESTAMP_PPS_HISTORY];
    uint8_t idx; /* next slot to be written */
    uint8_t size; /* current size */
};

/* -------------------------------------------------------------------------- */
/* --- PRIVATE CONSTANTS ---------------------------------------------------- */

#define PRECISION_TIMESTAMP_TS_METRICS_MAX  32 /* reduce number of metrics to better match GW v2 fine timestamp (max is 255) */
#define PRECISION_TIMESTAMP_NB_SYMBOLS      0

/* -------------------------------------------------------------------------- */
/* --- PRIVATE VARIABLES ---------------------------------------------------- */

/* history of the last PPS timestamps */
static struct timestamp_pps_history_s timestamp_pps_history = {
    .history = { 0 },
    .idx = 0,
    .size = 0
};

/* -------------------------------------------------------------------------- */
/* --- PRIVATE FUNCTIONS DECLARATION ---------------------------------------- */

/**
@brief TODO
@param TODO
@return The correction to be applied to the packet timestamp, in microseconds
*/
int32_t legacy_timestamp_correction(uint8_t bandwidth, uint8_t datarate, uint8_t coderate, bool no_crc, uint8_t payload_length, sx1302_rx_dft_peak_mode_t dft_peak_mode);

/**
@brief TODO
@param TODO
@return The correction to be applied to the packet timestamp, in microseconds
*/
int32_t precision_timestamp_correction(uint8_t bandwidth, uint8_t datarate, uint8_t coderate, bool crc_en, uint8_t payload_length);


/**
@brief TODO
@param TODO
@return The correction to be applied to the packet timestamp, in microseconds
*/
void timestamp_pps_history_save(uint32_t timestamp_pps_reg);

/* -------------------------------------------------------------------------- */
/* --- PRIVATE FUNCTIONS DEFINITION ----------------------------------------- */

int32_t legacy_timestamp_correction(uint8_t bandwidth, uint8_t sf, uint8_t cr, bool crc_en, uint8_t payload_length, sx1302_rx_dft_peak_mode_t dft_peak_mode) {
    uint64_t clk_period, filtering_delay, demap_delay, fft_delay_state3, fft_delay, decode_delay, total_delay;
    uint32_t nb_nibble, nb_nibble_in_hdr, nb_nibble_in_last_block;
    uint8_t nb_iter, bw_pow, dft_peak_en = (dft_peak_mode == RX_DFT_PEAK_MODE_DISABLED) ? 0 : 1;
    uint8_t ppm = SET_PPM_ON(bandwidth, sf) ? 1 : 0;
    int32_t timestamp_correction;
    bool payload_fits_in_header = false;
    uint8_t cr_local = cr;

    switch (bandwidth)
    {
        case BW_125KHZ:
            bw_pow = 1;
            break;
        case BW_250KHZ:
            bw_pow = 2;
            break;
        case BW_500KHZ:
            bw_pow = 4;
            break;
        default:
            printf("ERROR: UNEXPECTED VALUE %d IN SWITCH STATEMENT - %s\n", bandwidth, __FUNCTION__);
            return 0;
    }

    /* Prepare variables for delay computing */
    clk_period = 250E3 / bw_pow;

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

    nb_iter = (sf + 1) / 2; /* intended to be truncated */

    /* Update some variables if payload fits entirely in the header */
    if (((int)(2 * (payload_length + 2 * crc_en) - (sf - 7)) <= 0) || ((payload_length == 0) && (crc_en == false))) {
        /* Payload fits entirely in first 8 symbols (header):
            - not possible for SF5/SF6, unless payload length is 0 and no CRC
        */
        payload_fits_in_header = true;

        /* overwrite some variables accordingly */
        dft_peak_en = 0;

        cr_local = 4; /* header coding rate is 4 */

        if (sf > 6) {
            nb_nibble_in_last_block = sf - 2;
        } else {
            nb_nibble_in_last_block = sf;
        }
    }

    /* Filtering delay : I/Q 32Mhz -> 4Mhz */
    filtering_delay = 16000E3 / bw_pow + 2031250;

    /* demap delay */
    if (payload_fits_in_header == true) {
        demap_delay = clk_period + (1 << sf) * clk_period * 3 / 4 + 3 * clk_period + (sf - 2) * clk_period;
    } else {
        demap_delay = clk_period + (1 << sf) * clk_period * (1 - ppm / 4) + 3 * clk_period + (sf - 2 * ppm) * clk_period;
    }

    /* FFT delays */
    fft_delay_state3 = clk_period * (((1 << sf) - 6) + 2 * ((1 << sf) * (nb_iter - 1) + 6)) + 4 * clk_period;

    if (dft_peak_en) {
        fft_delay = (5 - 2 * ppm) * ((1 << sf) * clk_period + 7 * clk_period) + 2 * clk_period;
    } else {
        fft_delay = (1 << sf) * 2 * clk_period + 3 * clk_period;
    }

    /* Decode delay */
    decode_delay = 5 * clk_period + (9 * clk_period + clk_period * cr_local) * nb_nibble_in_last_block + 3 * clk_period;

    /* Cumulated delays */
    total_delay = (filtering_delay + fft_delay_state3 + fft_delay + demap_delay + decode_delay + 500E3) / 1E6;

    if (total_delay > INT32_MAX) {
        printf("ERROR: overflow error for timestamp correction (SHOULD NOT HAPPEN)\n");
        printf("=> filtering_delay %" PRIu64 "\n", filtering_delay);
        printf("=> fft_delay_state3 %" PRIu64 "\n", fft_delay_state3);
        printf("=> fft_delay %" PRIu64 "\n", fft_delay);
        printf("=> demap_delay %" PRIu64 "\n", demap_delay);
        printf("=> decode_delay %" PRIu64 "\n", decode_delay);
        printf("=> total_delay %" PRIu64 "\n", total_delay);
        assert(0);
    }

    timestamp_correction = -((int32_t)total_delay); /* compensate all decoding processing delays */

    DEBUG_PRINTF("FTIME OFF : filtering_delay %llu \n", filtering_delay);
    DEBUG_PRINTF("FTIME OFF : fft_delay_state3 %llu \n", fft_delay_state3);
    DEBUG_PRINTF("FTIME OFF : fft_delay %llu \n", fft_delay);
    DEBUG_PRINTF("FTIME OFF : demap_delay %llu \n", demap_delay);
    DEBUG_PRINTF("FTIME OFF : decode_delay %llu \n", decode_delay);
    DEBUG_PRINTF("FTIME OFF : timestamp correction %d \n", timestamp_correction);

    return timestamp_correction;
}

/* ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ */

int32_t precision_timestamp_correction(uint8_t bandwidth, uint8_t datarate, uint8_t coderate, bool crc_en, uint8_t payload_length) {
    uint32_t nb_symbols_payload;
    uint16_t t_symbol_us;
    int32_t timestamp_correction;
    uint8_t bw_pow;
    uint32_t filtering_delay;

    switch (bandwidth)
    {
        case BW_125KHZ:
            bw_pow = 1;
            break;
        case BW_250KHZ:
            bw_pow = 2;
            break;
        case BW_500KHZ:
            bw_pow = 4;
            break;
        default:
            printf("ERROR: UNEXPECTED VALUE %d IN SWITCH STATEMENT - %s\n", bandwidth, __FUNCTION__);
            return 0;
    }

    filtering_delay = 16000000 / bw_pow + 2031250;

    /* NOTE: no need of the preamble size, only the payload duration is needed */
    /* WARNING: implicit header not supported */
    if (lora_packet_time_on_air(bandwidth, datarate, coderate, 0, false, !crc_en, payload_length, NULL, &nb_symbols_payload, &t_symbol_us) == 0) {
        printf("ERROR: failed to compute packet time on air - %s\n", __FUNCTION__);
        return 0;
    }

    timestamp_correction = 0;
    timestamp_correction += (nb_symbols_payload * t_symbol_us); /* shift from end of header to end of packet */
    timestamp_correction -= (filtering_delay + 500E3) / 1E6; /* compensate the filtering delay */

    DEBUG_PRINTF("FTIME ON : timestamp correction %d \n", timestamp_correction);

    return timestamp_correction;
}

/* ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ */

void timestamp_pps_history_save(uint32_t timestamp_pps_reg) {
    /* Store it only if different from the previous one */
    if ((timestamp_pps_reg != timestamp_pps_history.history[timestamp_pps_history.idx] || (timestamp_pps_history.size == 0))) {
        /* Select next index */
        if (timestamp_pps_history.size > 0) {
            timestamp_pps_history.idx += 1;
        }
        if (timestamp_pps_history.idx == MAX_TIMESTAMP_PPS_HISTORY) {
            timestamp_pps_history.idx = 0;
        }

        /* Set PPS counter value */
        timestamp_pps_history.history[timestamp_pps_history.idx] = timestamp_pps_reg;

        /* Add one entry to the history */
        if (timestamp_pps_history.size < MAX_TIMESTAMP_PPS_HISTORY) {
            timestamp_pps_history.size += 1;
        }

#if 0
        printf("---- timestamp PPS history (idx:%u size:%u) ----\n",  timestamp_pps_history.idx,  timestamp_pps_history.size);
        for (int i = 0; i < timestamp_pps_history.size; i++) {
            printf("  %u\n", timestamp_pps_history.history[i]);
        }
        printf("--------------------------------\n");
#endif
    }
}

/* -------------------------------------------------------------------------- */
/* --- PUBLIC FUNCTIONS DEFINITION ------------------------------------------ */

void timestamp_counter_new(timestamp_counter_t * self) {
    memset(self, 0, sizeof(*self));
}

/* ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ */

void timestamp_counter_delete(timestamp_counter_t * self) {
    memset(self, 0, sizeof(*self));
}

/* ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ */

void timestamp_counter_update(timestamp_counter_t * self, uint32_t pps, uint32_t inst) {
    //struct timestamp_info_s* tinfo = (pps == true) ? &self->pps : &self->inst;

    /* Check if counter has wrapped, and update wrap status if necessary */
    if (pps < self->pps.counter_us_27bits_ref) {
        self->pps.counter_us_27bits_wrap += 1;
        self->pps.counter_us_27bits_wrap %= 32;
    }
    if (inst < self->inst.counter_us_27bits_ref) {
        self->inst.counter_us_27bits_wrap += 1;
        self->inst.counter_us_27bits_wrap %= 32;
    }

    /* Update counter reference */
    self->pps.counter_us_27bits_ref = pps;
    self->inst.counter_us_27bits_ref = inst;
}

/* ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ */

int timestamp_counter_get(timestamp_counter_t * self, uint32_t * inst, uint32_t * pps) {
    int x;
    uint8_t buff[8];
    uint8_t buff_wa[8];
    uint32_t counter_inst_us_raw_27bits_now;
    uint32_t counter_pps_us_raw_27bits_now;

    /* Get the freerun and pps 32MHz timestamp counters - 8 bytes
            0 -> 3 : PPS counter
            4 -> 7 : Freerun counter (inst)
    */
    x = lgw_reg_rb(SX1302_REG_TIMESTAMP_TIMESTAMP_PPS_MSB2_TIMESTAMP_PPS, &buff[0], 8);
    if (x != LGW_REG_SUCCESS) {
        printf("ERROR: Failed to get timestamp counter value\n");
        return -1;
    }

    /* Workaround concentrator chip issue:
        - read MSB again
        - if MSB changed, read the full counter again
     */
    x = lgw_reg_rb(SX1302_REG_TIMESTAMP_TIMESTAMP_PPS_MSB2_TIMESTAMP_PPS, &buff_wa[0], 8);
    if (x != LGW_REG_SUCCESS) {
        printf("ERROR: Failed to get timestamp counter MSB value\n");
        return -1;
    }
    if ((buff[0] != buff_wa[0]) || (buff[4] != buff_wa[4])) {
        x = lgw_reg_rb(SX1302_REG_TIMESTAMP_TIMESTAMP_PPS_MSB2_TIMESTAMP_PPS, &buff_wa[0], 8);
        if (x != LGW_REG_SUCCESS) {
            printf("ERROR: Failed to get timestamp counter MSB value\n");
            return -1;
        }
        memcpy(buff, buff_wa, 8); /* use the new read value */
    }

    counter_pps_us_raw_27bits_now  = (buff[0]<<24) | (buff[1]<<16) | (buff[2]<<8) | buff[3];
    counter_inst_us_raw_27bits_now = (buff[4]<<24) | (buff[5]<<16) | (buff[6]<<8) | buff[7];

    /* Store PPS counter to history, for fine timestamp calculation */
    timestamp_pps_history_save(counter_pps_us_raw_27bits_now);

    /* Scale to 1MHz */
    counter_pps_us_raw_27bits_now /= 32;
    counter_inst_us_raw_27bits_now /= 32;

    /* Update counter wrapping status */
    timestamp_counter_update(self, counter_pps_us_raw_27bits_now, counter_inst_us_raw_27bits_now);

    /* Convert 27-bits counter to 32-bits counter */
    *inst = timestamp_counter_expand(self, false, counter_inst_us_raw_27bits_now);
    *pps  = timestamp_counter_expand(self, true, counter_pps_us_raw_27bits_now);

    return 0;
}

/* ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ */

uint32_t timestamp_counter_expand(timestamp_counter_t * self, bool pps, uint32_t cnt_us) {
    struct timestamp_info_s* tinfo = (pps == true) ? &self->pps : &self->inst;
    uint32_t counter_us_32bits;

    counter_us_32bits = (tinfo->counter_us_27bits_wrap << 27) | cnt_us;

#if 0
    /* DEBUG: to be enabled when running test_loragw_counter test application
       This generates a CSV log, and can be plotted with gnuplot:
        > set datafile separator comma
        > plot for [col=1:2:1] 'log_count.txt' using col with lines
    */
    printf("%u,%u,%u\n", cnt_us, counter_us_32bits, tinfo->counter_us_27bits_wrap);
#endif

    return counter_us_32bits;
}


/* ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ */

uint32_t timestamp_pkt_expand(timestamp_counter_t * self, uint32_t pkt_cnt_us) {
    struct timestamp_info_s* tinfo = &self->inst;
    uint32_t counter_us_32bits;
    uint8_t wrap_status;

    /* Check if counter has wrapped since the packet has been received in the sx1302 internal FIFO */
    /* If the sx1302 counter was greater than the pkt timestamp, it means that the internal counter
        hasn't rolled over since the packet has been received by the sx1302
        case 1: --|-P--|----|--R-|----|--||-|----|-- : use current wrap status counter
        case 2: --|-P-||-|-R--|-- : use previous wrap status counter
        P : packet received in sx1302 internal FIFO
        R : read packet from sx1302 internal FIFO
        | : last update internal counter ref value.
        ||: sx1302 internal counter rollover (wrap)
    */

    /* Use current wrap counter or previous ? */
    wrap_status = tinfo->counter_us_27bits_wrap - ((tinfo->counter_us_27bits_ref >= pkt_cnt_us) ? 0 : 1);
    wrap_status &= 0x1F; /* [0..31] */

    /* Expand packet counter */
    counter_us_32bits = (wrap_status << 27) | pkt_cnt_us;

    return counter_us_32bits;
}

/* ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ */

int timestamp_counter_mode(bool ftime_enable) {
    int x = LGW_REG_SUCCESS;

    if (ftime_enable == false) {
        printf("INFO: using legacy timestamp\n");
        /* Latch end-of-packet timestamp (sx1301 compatibility) */
        x |= lgw_reg_w(SX1302_REG_RX_TOP_RX_BUFFER_LEGACY_TIMESTAMP, 0x01);
    } else {
        printf("INFO: using precision timestamp (max_ts_metrics:%u nb_symbols:%u)\n", PRECISION_TIMESTAMP_TS_METRICS_MAX, PRECISION_TIMESTAMP_NB_SYMBOLS);

        /* Latch end-of-preamble timestamp */
        x |= lgw_reg_w(SX1302_REG_RX_TOP_RX_BUFFER_LEGACY_TIMESTAMP, 0x00);
        x |= lgw_reg_w(SX1302_REG_RX_TOP_RX_BUFFER_TIMESTAMP_CFG_MAX_TS_METRICS, (int32_t)PRECISION_TIMESTAMP_TS_METRICS_MAX);

        /* LoRa multi-SF modems */
        x |= lgw_reg_w(SX1302_REG_RX_TOP_TIMESTAMP_ENABLE, 0x01);
        x |= lgw_reg_w(SX1302_REG_RX_TOP_TIMESTAMP_NB_SYMB, (int32_t)PRECISION_TIMESTAMP_NB_SYMBOLS);
    }

    return x;
}

/* ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ */

int32_t timestamp_counter_correction(lgw_context_t * context, uint8_t bandwidth, uint8_t datarate, uint8_t coderate, bool crc_en, uint8_t payload_length, sx1302_rx_dft_peak_mode_t dft_peak_mode) {
    /* Check input parameters */
    CHECK_NULL(context);
    if (IS_LORA_DR(datarate) == false) {
        printf("ERROR: wrong datarate (%u) - %s\n", datarate, __FUNCTION__);
        return 0;
    }
    if (IS_LORA_BW(bandwidth) == false) {
        printf("ERROR: wrong bandwidth (%u) - %s\n", bandwidth, __FUNCTION__);
        return 0;
    }
    if (IS_LORA_CR(coderate) == false) {
        printf("ERROR: wrong coding rate (%u) - %s\n", coderate, __FUNCTION__);
        return 0;
    }

    /* Calculate the correction to be applied */
    if (context->ftime_cfg.enable == false) {
        return legacy_timestamp_correction(bandwidth, datarate, coderate, crc_en, payload_length, dft_peak_mode);
    } else {
        return precision_timestamp_correction(bandwidth, datarate, coderate, crc_en, payload_length);
    }
}

/* ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ */

int precise_timestamp_calculate(uint8_t ts_metrics_nb, const int8_t * ts_metrics, uint32_t timestamp_cnt, uint8_t sf, int32_t if_freq_hz, double pkt_freq_error, uint32_t * result_ftime) {
    int i, x, timestamp_pps_idx, timestamp_pps_idx_next, timestamp_pps_idx_prev;
    int32_t ftime_sum;
    int32_t ftime[256];
    float ftime_mean;
    uint32_t timestamp_cnt_end_of_preamble;
    uint32_t timestamp_pps = 0;
    uint32_t timestamp_pps_reg = 0;
    uint32_t offset_preamble_hdr;
    uint8_t buff[4];
    uint32_t diff_pps;
    double pkt_ftime;
    uint8_t ts_metrics_nb_clipped;
    double xtal_correct;

    /* Check input parameters */
    CHECK_NULL(ts_metrics);
    CHECK_NULL(result_ftime);

    /* Check if we can calculate a ftime */
    if (timestamp_pps_history.size < MAX_TIMESTAMP_PPS_HISTORY) {
        printf("INFO: Cannot compute ftime yet, PPS history is too short\n");
        return -1;
    }

    /* Coarse timestamp correction to match with GW v2 (end of header -> end of preamble) */
    offset_preamble_hdr =   256 * (1 << sf) * (8 + 4 + (((sf == 5) || (sf == 6)) ? 2 : 0)) +
                            256 * ((1 << sf) / 4 - 1); /* 32e6 / 125e3 = 256 */

    /* Take the packet frequency error in account in the offset */
    offset_preamble_hdr += ((double)offset_preamble_hdr * pkt_freq_error + 0.5);

    timestamp_cnt_end_of_preamble = timestamp_cnt - offset_preamble_hdr + 2138; /* 2138 is the number of 32MHz clock cycle offset b/w GW_V2 and SX1303 decimation/filtering group delay */

    /* Shift the packet coarse timestamp which is used to get ref PPS counter */
    timestamp_cnt = timestamp_cnt_end_of_preamble;

    /* Clip the number of metrics depending on Spreading Factor, reduce fine timestamp variation versus packet duration */
    switch (sf) {
        case 12:
            ts_metrics_nb_clipped = MIN(4, ts_metrics_nb);
            break;
        case 11:
            ts_metrics_nb_clipped = MIN(8, ts_metrics_nb);
            break;
        case 10:
            ts_metrics_nb_clipped = MIN(16, ts_metrics_nb);
            break;
        default:
            ts_metrics_nb_clipped = MIN(32, ts_metrics_nb);
            break;
    }

#if 0
    printf("%s\n", __FUNCTION__);
    printf("ts_metrics_nb_clipped*2: %u\n", ts_metrics_nb_clipped * 2);
    for (i = 0; i < (2 * ts_metrics_nb_clipped); i++) {
        printf("%d ", ts_metrics[i]);
    }
    printf("\n");
#endif

    /* Compute the ftime cumulative sum */
    ftime[0] = (int32_t)ts_metrics[0];
    ftime_sum = ftime[0];
    for (i = 1; i < (2 * ts_metrics_nb_clipped); i++) {
        ftime[i] = ftime[i-1] + ts_metrics[i];
        ftime_sum += ftime[i];
    }

    /* Compute the mean of the cumulative sum */
    ftime_mean = (float)ftime_sum / (float)(2 * ts_metrics_nb_clipped);

    /* Find the last timestamp_pps before packet to use as reference for ftime */
    x = lgw_reg_rb(SX1302_REG_TIMESTAMP_TIMESTAMP_PPS_MSB2_TIMESTAMP_PPS , &buff[0], 4);
    if (x != LGW_REG_SUCCESS) {
        printf("ERROR: Failed to get timestamp counter value\n");
        return 0;
    }
    timestamp_pps_reg  = (uint32_t)((buff[0] << 24) & 0xFF000000);
    timestamp_pps_reg |= (uint32_t)((buff[1] << 16) & 0x00FF0000);
    timestamp_pps_reg |= (uint32_t)((buff[2] << 8)  & 0x0000FF00);
    timestamp_pps_reg |= (uint32_t)((buff[3] << 0)  & 0x000000FF);

    /* Ensure that the timestamp PPS history is up-to-date */
    timestamp_pps_history_save(timestamp_pps_reg);

    /* Check if timestamp_pps_reg we just read is the reference to be used to compute ftime or not */
    if ((timestamp_cnt - timestamp_pps_reg) > 32e6) {
        /* The timestamp_pps_reg we just read is after the packet timestamp, we need to rewind */
        for (timestamp_pps_idx = 0; timestamp_pps_idx < timestamp_pps_history.size; timestamp_pps_idx++) {
            /* search the pps counter in history */
            if ((timestamp_cnt - timestamp_pps_history.history[timestamp_pps_idx]) < 32e6) {
                timestamp_pps = timestamp_pps_history.history[timestamp_pps_idx];
                DEBUG_PRINTF("==> timestamp_pps found at history[%d] => %u\n", timestamp_pps_idx, timestamp_pps);
                break;
            }
        }
        if (timestamp_pps_idx == timestamp_pps_history.size) {
            printf("ERROR: failed to find the reference timestamp_pps, cannot compute ftime\n");
            return -1;
        }

        /* Calculate the Xtal error between the reference PPS we just found and the next one */
        timestamp_pps_idx_next = (timestamp_pps_idx == (MAX_TIMESTAMP_PPS_HISTORY - 1)) ? 0 : timestamp_pps_idx + 1;
        diff_pps = timestamp_pps_history.history[timestamp_pps_idx_next] - timestamp_pps_history.history[timestamp_pps_idx];
        xtal_correct = (double)32e6 / (double)(diff_pps);
    } else {
        /* The timestamp_pps_reg we just read is the reference we use to calculate the fine timestamp */
        timestamp_pps = timestamp_pps_reg;
        DEBUG_PRINTF("==> timestamp_pps => %u\n", timestamp_pps);

        /* Calculate the Xtal error between the reference PPS we just found and the previous one */
        timestamp_pps_idx = timestamp_pps_history.idx;
        timestamp_pps_idx_prev = (timestamp_pps_idx == 0) ? (MAX_TIMESTAMP_PPS_HISTORY - 1) : (timestamp_pps_idx - 1);
        diff_pps = timestamp_pps_history.history[timestamp_pps_idx] - timestamp_pps_history.history[timestamp_pps_idx_prev];
        xtal_correct = (double)32e6 / (double)(diff_pps);
    }

    /* Sanity Check on xtal_correct */
    if ((xtal_correct > 1.2) || (xtal_correct < 0.8)) {
        printf("ERROR: xtal_error is invalid (%.15lf)\n", xtal_correct);
        return -1;
    }

    /* Coarse timestamp based on PPS reference */
    diff_pps = timestamp_cnt - timestamp_pps;

    DEBUG_PRINTF("timestamp_cnt : %u\n", timestamp_cnt);
    DEBUG_PRINTF("timestamp_pps : %u\n", timestamp_pps);
    DEBUG_PRINTF("diff_pps : %d\n", diff_pps);

    /* Compute the fine timestamp */
    pkt_ftime = (double)diff_pps + (double)ftime_mean;
    DEBUG_PRINTF("pkt_ftime = %f\n", pkt_ftime);

    /* Add the DC notch filtering delay if necessary */
    pkt_ftime += sx1302_dc_notch_delay((double)if_freq_hz / 1E3);

    /* Convert fine timestamp from 32 Mhz clock to nanoseconds */
    pkt_ftime *= 31.25;

    /* Apply current XTAL error correction */
    pkt_ftime *= xtal_correct;

    *result_ftime = (uint32_t)pkt_ftime;
    if (*result_ftime > 1E9) {
        printf("ERROR: fine timestamp is out of range (%u)\n", *result_ftime);
        return -1;
    }

    DEBUG_PRINTF("==> ftime = %u ns since last PPS (%.15lf)\n", *result_ftime, pkt_ftime);

    return 0;
}

/* --- EOF ------------------------------------------------------------------ */
