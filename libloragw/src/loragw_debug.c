/*
 / _____)             _              | |
( (____  _____ ____ _| |_ _____  ____| |__
 \____ \| ___ |    (_   _) ___ |/ ___)  _ \
 _____) ) ____| | | || |_| ____( (___| | | |
(______/|_____)_|_|_| \__)_____)\____)_| |_|
  (C)2019 Semtech

Description:
    LoRa concentrator debug functions

License: Revised BSD License, see LICENSE.TXT file include in the project
*/


/* -------------------------------------------------------------------------- */
/* --- DEPENDANCIES --------------------------------------------------------- */

#include <stdint.h>     /* C99 types */
#include <stdbool.h>    /* bool type */
#include <stdio.h>      /* printf fprintf */
#include <string.h>     /* memcmp */
#include <time.h>

#include "loragw_aux.h"
#include "loragw_reg.h"
#include "loragw_hal.h"
#include "loragw_debug.h"

#include "tinymt32.h"

/* -------------------------------------------------------------------------- */
/* --- DEBUG CONSTANTS ------------------------------------------------------ */

/* -------------------------------------------------------------------------- */
/* --- PRIVATE MACROS ------------------------------------------------------- */

/* -------------------------------------------------------------------------- */
/* --- PRIVATE CONSTANTS & TYPES -------------------------------------------- */

/* -------------------------------------------------------------------------- */
/* --- PRIVATE VARIABLES ---------------------------------------------------- */

static tinymt32_t tinymt;

/* -------------------------------------------------------------------------- */
/* --- PRIVATE FUNCTIONS DECLARATION ---------------------------------------- */

/* -------------------------------------------------------------------------- */
/* --- PRIVATE FUNCTIONS DEFINITION ----------------------------------------- */

/* -------------------------------------------------------------------------- */
/* --- PUBLIC FUNCTIONS DEFINITION ------------------------------------------ */

void dbg_init_random(void) {
    tinymt.mat1 = 0x8f7011ee;
    tinymt.mat2 = 0xfc78ff1f;
    tinymt.tmat = 0x3793fdff;
}

/* ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ */

void dbg_log_buffer_to_file(FILE * file, uint8_t * buffer, uint16_t size) {
    int i;
    char stat_timestamp[24];
    time_t t;

    t = time(NULL);
    strftime(stat_timestamp, sizeof stat_timestamp, "%F %T %Z", gmtime(&t));
    fprintf(file, "---------(%s)------------\n", stat_timestamp);
    for (i = 0; i < size; i++) {
        fprintf(file, "%02X ", buffer[i]);
    }
    fprintf(file, "\n");

    fflush(file);
}

/* ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ */

void dbg_log_payload_diff_to_file(FILE * file, uint8_t * buffer1, uint8_t * buffer2, uint16_t size) {
    int i, j;
    uint16_t nb_bits_diff = 0;
    uint8_t debug_payload_diff[255];

    fprintf(file, "Diff: ");
    /* bit comparison of payloads */
    for (j = 0; j < size; j++) {
        debug_payload_diff[j] = buffer1[j] ^ buffer2[j];
        fprintf(file, "%02X ", debug_payload_diff[j]);
    }
    fprintf(file, "\n");

    /* count number of bits flipped, and display bit by bit */
    for (j = 0; j < size; j++) {
        for (i = 7; i >= 0; i--) {
            fprintf(file, "%u", TAKE_N_BITS_FROM(debug_payload_diff[j], i, 1));
            if (TAKE_N_BITS_FROM(debug_payload_diff[j], i, 1) == 1) {
                nb_bits_diff += 1;
            }
        }
        fprintf(file, " ");
    }
    fprintf(file, "\n");
    fprintf(file, "%u bits flipped\n", nb_bits_diff);

    fflush(file);
}

/* ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ */

void dbg_generate_random_payload(uint32_t pkt_cnt, uint8_t * buffer_expected, uint8_t size) {
    int k;

    /* construct payload we should get for this packet counter */
    tinymt32_init(&tinymt, (int)pkt_cnt);
    buffer_expected[4] = (uint8_t)(pkt_cnt >> 24);
    buffer_expected[5] = (uint8_t)(pkt_cnt >> 16);
    buffer_expected[6] = (uint8_t)(pkt_cnt >> 8);
    buffer_expected[7] = (uint8_t)(pkt_cnt >> 0);
    tinymt32_generate_uint32(&tinymt); /* dummy: for sync with random size generation */
    for (k = 8; k < (int)size; k++) {
        buffer_expected[k] = (uint8_t)tinymt32_generate_uint32(&tinymt);
    }
}

/* ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ */

int dbg_check_payload(struct lgw_conf_debug_s * context, FILE * file, uint8_t * payload_received, uint8_t size, uint8_t ref_payload_idx, uint8_t sf) {
    int k;
    uint32_t debug_payload_cnt;

    /* If the 4 first bytes of received payload match with the expected ones, go on with comparison */
    if (memcmp((void*)payload_received, (void*)(context->ref_payload[ref_payload_idx].payload), 4) == 0) {
        /* get counter to initialize random seed */
        debug_payload_cnt = (unsigned int)(payload_received[4] << 24) | (unsigned int)(payload_received[5] << 16) | (unsigned int)(payload_received[6] << 8) | (unsigned int)(payload_received[7] << 0);

        /* check if we missed some packets */
        if (debug_payload_cnt > (context->ref_payload[ref_payload_idx].prev_cnt + 1)) {
            printf("ERROR: 0x%08X missed %u pkt before %u (SF%u, size:%u)\n", context->ref_payload[ref_payload_idx].id, debug_payload_cnt - context->ref_payload[ref_payload_idx].prev_cnt - 1, debug_payload_cnt, sf, size);
            if (file != NULL) {
                fprintf(file, "ERROR: 0x%08X missed %u pkt before %u (SF%u, size:%u)\n", context->ref_payload[ref_payload_idx].id, debug_payload_cnt - context->ref_payload[ref_payload_idx].prev_cnt - 1, debug_payload_cnt, sf, size);
                fflush(file);
            }
        } else if (debug_payload_cnt < context->ref_payload[ref_payload_idx].prev_cnt) {
            if (file != NULL) {
                fprintf(file, "INFO:  0x%08X got missing pkt %u (SF%u, size:%u) ?\n", context->ref_payload[ref_payload_idx].id, debug_payload_cnt, sf, size);
                fflush(file);
            }
        } else {
#if 0
            if (file != NULL) {
                fprintf(file, "0x%08X %u (SF%u, size:%u)\n", context.ref_payload[ref_payload_idx].id, debug_payload_cnt, sf, size);
            }
#endif
        }
        context->ref_payload[ref_payload_idx].prev_cnt = debug_payload_cnt;

        /* generate the random payload which is expected for this packet count */
        dbg_generate_random_payload(debug_payload_cnt, context->ref_payload[ref_payload_idx].payload, size);

        /* compare expected with received */
        if (memcmp((void *)payload_received, (void *)(context->ref_payload[ref_payload_idx].payload), size) != 0) {
            if (file != NULL) {
                fprintf(file, "RECEIVED:");
                for (k = 0; k < (int)size; k++) {
                    fprintf(file, "%02X ", payload_received[k]);
                }
                fprintf(file, "\n");
                fprintf(file, "EXPECTED:");
                for (k = 0; k < (int)size; k++) {
                    fprintf(file, "%02X ", context->ref_payload[ref_payload_idx].payload[k]);
                }
                fprintf(file, "\n");
            }
            return -1;
        } else {
            return 1; /* matches */
        }
    }

    return 0; /* ignored */
}
