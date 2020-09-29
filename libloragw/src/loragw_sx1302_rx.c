/*
 / _____)             _              | |
( (____  _____ ____ _| |_ _____  ____| |__
 \____ \| ___ |    (_   _) ___ |/ ___)  _ \
 _____) ) ____| | | || |_| ____( (___| | | |
(______/|_____)_|_|_| \__)_____)\____)_| |_|
  (C)2019 Semtech

Description:
    SX1302 RX buffer Hardware Abstraction Layer

License: Revised BSD License, see LICENSE.TXT file include in the project
*/


/* -------------------------------------------------------------------------- */
/* --- DEPENDANCIES --------------------------------------------------------- */

#include <stdint.h>     /* C99 types */
#include <stdio.h>      /* printf fprintf */
#include <string.h>     /* memset */
#include <assert.h>     /* assert */

#include "loragw_aux.h"
#include "loragw_reg.h"
#include "loragw_sx1302_rx.h"
#include "loragw_sx1302_timestamp.h"

/* -------------------------------------------------------------------------- */
/* --- PRIVATE MACROS ------------------------------------------------------- */

#define ARRAY_SIZE(a) (sizeof(a) / sizeof((a)[0]))
#if DEBUG_SX1302 == 1
    #define DEBUG_MSG(str)                fprintf(stdout, str)
    #define DEBUG_PRINTF(fmt, args...)    fprintf(stdout, fmt, args)
    #define CHECK_NULL(a)                if(a==NULL){fprintf(stderr,"%s:%d: ERROR: NULL POINTER AS ARGUMENT\n", __FUNCTION__, __LINE__);return LGW_REG_ERROR;}
#else
    #define DEBUG_MSG(str)
    #define DEBUG_PRINTF(fmt, args...)
    #define CHECK_NULL(a)                if(a==NULL){return LGW_REG_ERROR;}
#endif

#define SX1302_PKT_PAYLOAD_LENGTH(buffer, start_index)          TAKE_N_BITS_FROM(buffer[start_index +  2], 0, 8)
#define SX1302_PKT_CHANNEL(buffer, start_index)                 TAKE_N_BITS_FROM(buffer[start_index +  3], 0, 8)
#define SX1302_PKT_CRC_EN(buffer, start_index)                  TAKE_N_BITS_FROM(buffer[start_index +  4], 0, 1)
#define SX1302_PKT_CODING_RATE(buffer, start_index)             TAKE_N_BITS_FROM(buffer[start_index +  4], 1, 3)
#define SX1302_PKT_DATARATE(buffer, start_index)                TAKE_N_BITS_FROM(buffer[start_index +  4], 4, 4)
#define SX1302_PKT_MODEM_ID(buffer, start_index)                TAKE_N_BITS_FROM(buffer[start_index +  5], 0, 8)
#define SX1302_PKT_FREQ_OFFSET_7_0(buffer, start_index)         TAKE_N_BITS_FROM(buffer[start_index +  6], 0, 8)
#define SX1302_PKT_FREQ_OFFSET_15_8(buffer, start_index)        TAKE_N_BITS_FROM(buffer[start_index +  7], 0, 8)
#define SX1302_PKT_FREQ_OFFSET_19_16(buffer, start_index)       TAKE_N_BITS_FROM(buffer[start_index +  8], 0, 4)
#define SX1302_PKT_CRC_ERROR(buffer, start_index)               TAKE_N_BITS_FROM(buffer[start_index +  9], 0, 1)
#define SX1302_PKT_SYNC_ERROR(buffer, start_index)              TAKE_N_BITS_FROM(buffer[start_index +  9], 2, 1)
#define SX1302_PKT_HEADER_ERROR(buffer, start_index)            TAKE_N_BITS_FROM(buffer[start_index +  9], 3, 1)
#define SX1302_PKT_TIMING_SET(buffer, start_index)              TAKE_N_BITS_FROM(buffer[start_index +  9], 4, 1)
#define SX1302_PKT_SNR_AVG(buffer, start_index)                 TAKE_N_BITS_FROM(buffer[start_index + 10], 0, 8)
#define SX1302_PKT_RSSI_CHAN(buffer, start_index)               TAKE_N_BITS_FROM(buffer[start_index + 11], 0, 8)
#define SX1302_PKT_RSSI_SIG(buffer, start_index)                TAKE_N_BITS_FROM(buffer[start_index + 12], 0, 8)
#define SX1302_PKT_RSSI_CHAN_MAX_NEG_DELTA(buffer, start_index) TAKE_N_BITS_FROM(buffer[start_index + 13], 0, 4)
#define SX1302_PKT_RSSI_CHAN_MAX_POS_DELTA(buffer, start_index) TAKE_N_BITS_FROM(buffer[start_index + 13], 4, 4)
#define SX1302_PKT_RSSI_SIG_MAX_NEG_DELTA(buffer, start_index)  TAKE_N_BITS_FROM(buffer[start_index + 14], 0, 4)
#define SX1302_PKT_RSSI_SIG_MAX_POS_DELTA(buffer, start_index)  TAKE_N_BITS_FROM(buffer[start_index + 14], 4, 4)
#define SX1302_PKT_TIMESTAMP_7_0(buffer, start_index)           TAKE_N_BITS_FROM(buffer[start_index + 15], 0, 8)
#define SX1302_PKT_TIMESTAMP_15_8(buffer, start_index)          TAKE_N_BITS_FROM(buffer[start_index + 16], 0, 8)
#define SX1302_PKT_TIMESTAMP_23_16(buffer, start_index)         TAKE_N_BITS_FROM(buffer[start_index + 17], 0, 8)
#define SX1302_PKT_TIMESTAMP_31_24(buffer, start_index)         TAKE_N_BITS_FROM(buffer[start_index + 18], 0, 8)
#define SX1302_PKT_CRC_PAYLOAD_7_0(buffer, start_index)         TAKE_N_BITS_FROM(buffer[start_index + 19], 0, 8)
#define SX1302_PKT_CRC_PAYLOAD_15_8(buffer, start_index)        TAKE_N_BITS_FROM(buffer[start_index + 20], 0, 8)
#define SX1302_PKT_NUM_TS_METRICS(buffer, start_index)          TAKE_N_BITS_FROM(buffer[start_index + 21], 0, 8)

/* -------------------------------------------------------------------------- */
/* --- PRIVATE TYPES -------------------------------------------------------- */

/* -------------------------------------------------------------------------- */
/* --- PRIVATE CONSTANTS ---------------------------------------------------- */

/* RX buffer packet structure */
#define SX1302_PKT_SYNCWORD_BYTE_0  0xA5
#define SX1302_PKT_SYNCWORD_BYTE_1  0xC0
#define SX1302_PKT_HEAD_METADATA    9
#define SX1302_PKT_TAIL_METADATA    14

/* modem IDs */
#define SX1302_LORA_MODEM_ID_MAX    15
#define SX1302_LORA_STD_MODEM_ID    16
#define SX1302_FSK_MODEM_ID         17

/* -------------------------------------------------------------------------- */
/* --- PRIVATE VARIABLES ---------------------------------------------------- */

/* -------------------------------------------------------------------------- */
/* --- PRIVATE FUNCTIONS DECLARATION ---------------------------------------- */

/* -------------------------------------------------------------------------- */
/* --- PRIVATE FUNCTIONS DEFINITION ----------------------------------------- */

/* -------------------------------------------------------------------------- */
/* --- PUBLIC FUNCTIONS DEFINITION ------------------------------------------ */

int rx_buffer_new(rx_buffer_t * self) {
    /* Check input params */
    CHECK_NULL(self);

    /* Initialize members */
    memset(self->buffer, 0, sizeof self->buffer);
    self->buffer_size = 0;
    self->buffer_index = 0;
    self->buffer_pkt_nb = 0;

    return LGW_REG_SUCCESS;
}

/* ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ */

int rx_buffer_del(rx_buffer_t * self) {
    /* Check input params */
    CHECK_NULL(self);

    /* Reset index & size */
    self->buffer_size = 0;
    self->buffer_index = 0;
    self->buffer_pkt_nb = 0;

    return LGW_REG_SUCCESS;
}

/* ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ */

int rx_buffer_fetch(rx_buffer_t * self) {
    int i, res;
    uint8_t buff[2];
    uint8_t payload_len;
    uint16_t next_pkt_idx;
    int idx;
    uint16_t nb_bytes_1, nb_bytes_2;

    /* Check input params */
    CHECK_NULL(self);

    /* Check if there is data in the FIFO */
    lgw_reg_rb(SX1302_REG_RX_TOP_RX_BUFFER_NB_BYTES_MSB_RX_BUFFER_NB_BYTES, buff, sizeof buff);
    nb_bytes_1 = (buff[0] << 8) | (buff[1] << 0);

    /* Workaround for multi-byte read issue: read again and ensure new read is not lower than the previous one */
    lgw_reg_rb(SX1302_REG_RX_TOP_RX_BUFFER_NB_BYTES_MSB_RX_BUFFER_NB_BYTES, buff, sizeof buff);
    nb_bytes_2 = (buff[0] << 8) | (buff[1] << 0);

    self->buffer_size = (nb_bytes_2 > nb_bytes_1) ? nb_bytes_2 : nb_bytes_1;

    /* Fetch bytes from fifo if any */
    if (self->buffer_size > 0) {
        DEBUG_MSG   ("-----------------\n");
        DEBUG_PRINTF("%s: nb_bytes to be fetched: %u (%u %u)\n", __FUNCTION__, self->buffer_size, buff[1], buff[0]);

        memset(self->buffer, 0, sizeof self->buffer);
        res = lgw_mem_rb(0x4000, self->buffer, self->buffer_size, true);
        if (res != LGW_REG_SUCCESS) {
            printf("ERROR: Failed to read RX buffer, SPI error\n");
            return LGW_REG_ERROR;
        }

        /* print debug info */
        DEBUG_MSG("RX_BUFFER: ");
        for (i = 0; i < self->buffer_size; i++) {
            DEBUG_PRINTF("%02X ", self->buffer[i]);
        }
        DEBUG_MSG("\n");

        /* Sanity check: is there at least 1 complete packet in the buffer */
        if (self->buffer_size < (SX1302_PKT_HEAD_METADATA + SX1302_PKT_TAIL_METADATA)) {
            printf("WARNING: not enough data to have a complete packet, discard rx_buffer\n");
            return rx_buffer_del(self);
        }

        /* Sanity check: is there a syncword at 0 ? If not, move to the first syncword found */
        idx = 0;
        while (idx <= (self->buffer_size - 2)) {
            if ((self->buffer[idx] == SX1302_PKT_SYNCWORD_BYTE_0) && (self->buffer[idx + 1] == SX1302_PKT_SYNCWORD_BYTE_1)) {
                DEBUG_PRINTF("INFO: syncword found at idx %d\n", idx);
                break;
            } else {
                printf("INFO: syncword not found at idx %d\n", idx);
                idx += 1;
            }
        }
        if (idx > self->buffer_size - 2) {
            printf("WARNING: no syncword found, discard rx_buffer\n");
            return rx_buffer_del(self);
        }
        if (idx != 0) {
            printf("INFO: re-sync rx_buffer at idx %d\n", idx);
            memmove((void *)(self->buffer), (void *)(self->buffer + idx), self->buffer_size - idx);
            self->buffer_size -= idx;
        }

        /* Rewind and parse buffer to get the number of packet fetched */
        idx = 0;
        while (idx < self->buffer_size) {
            if ((self->buffer[idx] != SX1302_PKT_SYNCWORD_BYTE_0) || (self->buffer[idx + 1] != SX1302_PKT_SYNCWORD_BYTE_1)) {
                printf("WARNING: syncword not found at idx %d, discard the rx_buffer\n", idx);
                return rx_buffer_del(self);
            }
            /* One packet found in the buffer */
            self->buffer_pkt_nb += 1;

            /* Compute the number of bytes for this packet */
            payload_len = SX1302_PKT_PAYLOAD_LENGTH(self->buffer, idx);
            next_pkt_idx =  SX1302_PKT_HEAD_METADATA +
                            payload_len +
                            SX1302_PKT_TAIL_METADATA +
                            (2 * SX1302_PKT_NUM_TS_METRICS(self->buffer, idx + payload_len));

            /* Move to next packet */
            idx += (int)next_pkt_idx;
        }
    }

    /* Initialize the current buffer index to iterate on */
    self->buffer_index = 0;

    return LGW_REG_SUCCESS;
}

/* ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ */

int rx_buffer_pop(rx_buffer_t * self, rx_packet_t * pkt) {
    int i;
    uint8_t checksum_rcv, checksum_calc = 0;
    uint16_t checksum_idx;
    uint16_t pkt_num_bytes;

    /* Check input params */
    CHECK_NULL(self);
    CHECK_NULL(pkt);

    /* Is there any data to be parsed ? */
    if (self->buffer_index >= self->buffer_size) {
        DEBUG_MSG("INFO: No more data to be parsed\n");
        return LGW_REG_ERROR;
    }

    /* Get pkt sync words */
    if ((self->buffer[self->buffer_index] != SX1302_PKT_SYNCWORD_BYTE_0) || (self->buffer[self->buffer_index + 1] != SX1302_PKT_SYNCWORD_BYTE_1)) {
        return LGW_REG_ERROR;
    }
    DEBUG_PRINTF("INFO: pkt syncword found at index %u\n", self->buffer_index);

    /* Get payload length */
    pkt->rxbytenb_modem = SX1302_PKT_PAYLOAD_LENGTH(self->buffer, self->buffer_index);

    /* Get fine timestamp metrics */
    pkt->num_ts_metrics_stored = SX1302_PKT_NUM_TS_METRICS(self->buffer, self->buffer_index + pkt->rxbytenb_modem);

    /* Calculate the total number of bytes in the packet */
    pkt_num_bytes = SX1302_PKT_HEAD_METADATA + pkt->rxbytenb_modem + SX1302_PKT_TAIL_METADATA + (2 * pkt->num_ts_metrics_stored);

    /* Check if we have a complete packet in the rx buffer fetched */
    if((self->buffer_index + pkt_num_bytes) > self->buffer_size) {
        printf("WARNING: aborting truncated message (size=%u)\n", self->buffer_size);
        return LGW_REG_WARNING;
    }

    /* Get the checksum as received in the RX buffer */
    checksum_idx = pkt_num_bytes - 1;
    checksum_rcv = self->buffer[self->buffer_index + pkt_num_bytes - 1];

    /* Calculate the checksum from the actual payload bytes received */
    for (i = 0; i < (int)checksum_idx; i++) {
        checksum_calc += self->buffer[self->buffer_index + i];
    }

    /* Check if the checksum is correct */
    if (checksum_rcv != checksum_calc) {
        printf("WARNING: checksum failed (got:0x%02X calc:0x%02X)\n", checksum_rcv, checksum_calc);
        return LGW_REG_WARNING;
    } else {
        DEBUG_PRINTF("Packet checksum OK (0x%02X)\n", checksum_rcv);
    }

    /* Parse packet metadata */
    pkt->modem_id = SX1302_PKT_MODEM_ID(self->buffer, self->buffer_index);
    pkt->rx_channel_in = SX1302_PKT_CHANNEL(self->buffer, self->buffer_index);
    pkt->crc_en = SX1302_PKT_CRC_EN(self->buffer, self->buffer_index);
    pkt->payload_crc_error = SX1302_PKT_CRC_ERROR(self->buffer, self->buffer_index + pkt->rxbytenb_modem);
    pkt->sync_error = SX1302_PKT_SYNC_ERROR(self->buffer, self->buffer_index + pkt->rxbytenb_modem);
    pkt->header_error = SX1302_PKT_HEADER_ERROR(self->buffer, self->buffer_index + pkt->rxbytenb_modem);
    pkt->timing_set = SX1302_PKT_TIMING_SET(self->buffer, self->buffer_index + pkt->rxbytenb_modem);
    pkt->coding_rate = SX1302_PKT_CODING_RATE(self->buffer, self->buffer_index);
    pkt->rx_rate_sf = SX1302_PKT_DATARATE(self->buffer, self->buffer_index);
    pkt->rssi_chan_avg = SX1302_PKT_RSSI_CHAN(self->buffer, self->buffer_index + pkt->rxbytenb_modem);
    pkt->rssi_signal_avg = SX1302_PKT_RSSI_SIG(self->buffer, self->buffer_index + pkt->rxbytenb_modem);
    pkt->rx_crc16_value  = (uint16_t)((SX1302_PKT_CRC_PAYLOAD_7_0(self->buffer, self->buffer_index + pkt->rxbytenb_modem) <<  0) & 0x00FF);
    pkt->rx_crc16_value |= (uint16_t)((SX1302_PKT_CRC_PAYLOAD_15_8(self->buffer, self->buffer_index + pkt->rxbytenb_modem) <<  8) & 0xFF00);
    pkt->snr_average = (int8_t)SX1302_PKT_SNR_AVG(self->buffer, self->buffer_index + pkt->rxbytenb_modem);

    pkt->frequency_offset_error = (int32_t)((SX1302_PKT_FREQ_OFFSET_19_16(self->buffer, self->buffer_index) << 16) | (SX1302_PKT_FREQ_OFFSET_15_8(self->buffer, self->buffer_index) << 8) | (SX1302_PKT_FREQ_OFFSET_7_0(self->buffer, self->buffer_index) << 0));
    if (pkt->frequency_offset_error >= (1<<19)) { /* Handle signed value on 20bits */
        pkt->frequency_offset_error = (pkt->frequency_offset_error - (1<<20));
    }

    /* Packet timestamp (32MHz ) */
    pkt->timestamp_cnt  = (uint32_t)((SX1302_PKT_TIMESTAMP_7_0(self->buffer, self->buffer_index + pkt->rxbytenb_modem) <<  0) & 0x000000FF);
    pkt->timestamp_cnt |= (uint32_t)((SX1302_PKT_TIMESTAMP_15_8(self->buffer, self->buffer_index + pkt->rxbytenb_modem) <<  8) & 0x0000FF00);
    pkt->timestamp_cnt |= (uint32_t)((SX1302_PKT_TIMESTAMP_23_16(self->buffer, self->buffer_index + pkt->rxbytenb_modem) << 16) & 0x00FF0000);
    pkt->timestamp_cnt |= (uint32_t)((SX1302_PKT_TIMESTAMP_31_24(self->buffer, self->buffer_index + pkt->rxbytenb_modem) << 24) & 0xFF000000);

    /* TS metrics: it is expected the nb_symbols parameter is set to 0 here */
    for (i = 0; i < (pkt->num_ts_metrics_stored * 2); i++) {
        pkt->timestamp_avg[i] = (int8_t)SX1302_PKT_NUM_TS_METRICS(self->buffer, self->buffer_index + pkt->rxbytenb_modem + 1 + i);
        pkt->timestamp_stddev[i] = 0; /* no stddev when nb_symbols == 0 */
    }

    DEBUG_MSG   ("-----------------\n");
    DEBUG_PRINTF("  modem:      %u\n", pkt->modem_id);
    DEBUG_PRINTF("  chan:       %u\n", pkt->rx_channel_in);
    DEBUG_PRINTF("  size:       %u\n", pkt->rxbytenb_modem);
    DEBUG_PRINTF("  crc_en:     %u\n", pkt->crc_en);
    DEBUG_PRINTF("  crc_err:    %u\n", pkt->payload_crc_error);
    DEBUG_PRINTF("  sync_err:   %u\n", pkt->sync_error);
    DEBUG_PRINTF("  hdr_err:    %u\n", pkt->header_error);
    DEBUG_PRINTF("  timing_set: %u\n", pkt->timing_set);
    DEBUG_PRINTF("  codr:       %u\n", pkt->coding_rate);
    DEBUG_PRINTF("  datr:       %u\n", pkt->rx_rate_sf);
    DEBUG_PRINTF("  num_ts:     %u\n", pkt->num_ts_metrics_stored);
    if (pkt->num_ts_metrics_stored > 0) {
        DEBUG_MSG("  ts_avg:     ");
        for (i = 0; i < (pkt->num_ts_metrics_stored * 2); i++) {
            DEBUG_PRINTF("%d ", pkt->timestamp_avg[i]);
        }
        DEBUG_MSG("\n");
        DEBUG_MSG("  ts_stdev:   NONE (nb_symbols=0)\n");
    }
    DEBUG_MSG   ("-----------------\n");

    /* Sanity checks: check the range of few metadata */
    if (pkt->modem_id > SX1302_FSK_MODEM_ID) {
        printf("ERROR: modem_id is out of range - %u\n", pkt->modem_id);
        return LGW_REG_ERROR;
    } else {
        if (pkt->modem_id <= SX1302_LORA_STD_MODEM_ID) { /* LoRa modems */
            if (pkt->rx_channel_in > 9) {
                printf("ERROR: channel is out of range - %u\n", pkt->rx_channel_in);
                return LGW_REG_ERROR;
            }
            if ((pkt->rx_rate_sf < 5) || (pkt->rx_rate_sf > 12)) {
                printf("ERROR: SF is out of range - %u\n", pkt->rx_rate_sf);
                return LGW_REG_ERROR;
            }
        } else { /* FSK modem */
            /* TODO: not checked */
        }
    }

    /* Parse & copy payload in packet struct */
    memcpy((void *)pkt->payload, (void *)(&(self->buffer[self->buffer_index + SX1302_PKT_HEAD_METADATA])), pkt->rxbytenb_modem);

    /* Move buffer index toward next message */
    self->buffer_index += (SX1302_PKT_HEAD_METADATA + pkt->rxbytenb_modem + SX1302_PKT_TAIL_METADATA + (2 * pkt->num_ts_metrics_stored));

    /* Update the number of packets currently stored in the rx_buffer */
    self->buffer_pkt_nb -= 1;

    return LGW_REG_SUCCESS;
}

/* -------------------------------------------------------------------------- */
/* --- DEBUG FUNCTIONS DEFINITION ------------------------------------------- */

uint16_t rx_buffer_read_ptr_addr(void) {
    int32_t val;
    uint16_t addr;

    lgw_reg_r(SX1302_REG_RX_TOP_RX_BUFFER_LAST_ADDR_READ_MSB_LAST_ADDR_READ, &val); /* mandatory to read MSB first */
    addr  = (uint16_t)(val << 8);
    lgw_reg_r(SX1302_REG_RX_TOP_RX_BUFFER_LAST_ADDR_READ_LSB_LAST_ADDR_READ, &val);
    addr |= (uint16_t)val;

    return addr;
}

/* ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ */

uint16_t rx_buffer_write_ptr_addr(void) {
    int32_t val;
    uint16_t addr;

    lgw_reg_r(SX1302_REG_RX_TOP_RX_BUFFER_LAST_ADDR_WRITE_MSB_LAST_ADDR_WRITE, &val);  /* mandatory to read MSB first */
    addr  = (uint16_t)(val << 8);
    lgw_reg_r(SX1302_REG_RX_TOP_RX_BUFFER_LAST_ADDR_WRITE_LSB_LAST_ADDR_WRITE, &val);
    addr |= (uint16_t)val;

    return addr;
}

/* ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ */

void rx_buffer_dump(FILE * file, uint16_t start_addr, uint16_t end_addr) {
    int i;
    uint8_t rx_buffer_debug[4096];

    printf("Dumping %u bytes, from 0x%X to 0x%X\n", end_addr - start_addr + 1, start_addr, end_addr);

    memset(rx_buffer_debug, 0, sizeof rx_buffer_debug);

    lgw_reg_w(SX1302_REG_RX_TOP_RX_BUFFER_DIRECT_RAM_IF, 1);
    lgw_mem_rb(0x4000 + start_addr, rx_buffer_debug, end_addr - start_addr + 1, false);
    lgw_reg_w(SX1302_REG_RX_TOP_RX_BUFFER_DIRECT_RAM_IF, 0);

    for (i = 0; i < (end_addr - start_addr + 1); i++) {
        if (file == NULL) {
            printf("%02X ", rx_buffer_debug[i]);
        } else {
            fprintf(file, "%02X ", rx_buffer_debug[i]);
        }
    }
    if (file == NULL) {
        printf("\n");
    } else {
        fprintf(file, "\n");
    }

    /* Switching to direct-access memory could lead to corruption, so to be done only for debugging */
    assert(0);
}

/* --- EOF ------------------------------------------------------------------ */
