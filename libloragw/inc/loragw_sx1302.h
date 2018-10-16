/*
 / _____)             _              | |
( (____  _____ ____ _| |_ _____  ____| |__
 \____ \| ___ |    (_   _) ___ |/ ___)  _ \
 _____) ) ____| | | || |_| ____( (___| | | |
(______/|_____)_|_|_| \__)_____)\____)_| |_|
  (C)2018 Semtech

Description:
    TODO

License: Revised BSD License, see LICENSE.TXT file include in the project
*/


#ifndef _LORAGW_SX1302_H
#define _LORAGW_SX1302_H

/* -------------------------------------------------------------------------- */
/* --- DEPENDANCIES --------------------------------------------------------- */

#include <stdint.h>     /* C99 types*/

#include "config.h"     /* library configuration options (dynamically generated) */

/* -------------------------------------------------------------------------- */
/* --- PUBLIC CONSTANTS ----------------------------------------------------- */

#define SX1302_AGC_RADIO_GAIN_AUTO  0xFF

#define SX1302_PKT_SYNCWORD_BYTE_0  0xA5
#define SX1302_PKT_SYNCWORD_BYTE_1  0xC0
#define SX1302_PKT_HEAD_METADATA    9
#define SX1302_PKT_TAIL_METADATA    14

/* -------------------------------------------------------------------------- */
/* --- PUBLIC MACROS -------------------------------------------------------- */

#define REG_SELECT(rf_chain, a, b) ((rf_chain == 0) ? a : b)

#define SX1302_PKT_PAYLOAD_LENGTH(buffer, start_index)      TAKE_N_BITS_FROM(buffer[start_index + 2], 0, 8)
#define SX1302_PKT_CHANNEL(buffer, start_index)             TAKE_N_BITS_FROM(buffer[start_index + 3], 0, 8)
#define SX1302_PKT_CRC_EN(buffer, start_index)              TAKE_N_BITS_FROM(buffer[start_index + 4], 0, 1)
#define SX1302_PKT_CODING_RATE(buffer, start_index)         TAKE_N_BITS_FROM(buffer[start_index + 4], 1, 3)
#define SX1302_PKT_DATARATE(buffer, start_index)            TAKE_N_BITS_FROM(buffer[start_index + 4], 4, 4)
#define SX1302_PKT_MODEM_ID(buffer, start_index)            TAKE_N_BITS_FROM(buffer[start_index + 5], 0, 8)
/* TODO: freq offset */
#define SX1302_PKT_CRC_ERROR(buffer, start_index)           TAKE_N_BITS_FROM(buffer[start_index + 9], 0, 1)
#define SX1302_PKT_SYNC_ERROR(buffer, start_index)          TAKE_N_BITS_FROM(buffer[start_index + 9], 2, 1)
#define SX1302_PKT_HEADER_ERROR(buffer, start_index)        TAKE_N_BITS_FROM(buffer[start_index + 9], 3, 1)
#define SX1302_PKT_SNR_AVG(buffer, start_index)             TAKE_N_BITS_FROM(buffer[start_index + 10], 0, 8)
#define SX1302_PKT_RSSI_CHAN(buffer, start_index)           TAKE_N_BITS_FROM(buffer[start_index + 11], 0, 8)
#define SX1302_PKT_RSSI_SIG(buffer, start_index)            TAKE_N_BITS_FROM(buffer[start_index + 12], 0, 8)
/* TODO: RSSI chan max delta */
/* TODO: RSSI sig max delta */
#define SX1302_PKT_TIMESTAMP_7_0(buffer, start_index)       TAKE_N_BITS_FROM(buffer[start_index + 15], 0, 8)
#define SX1302_PKT_TIMESTAMP_15_8(buffer, start_index)      TAKE_N_BITS_FROM(buffer[start_index + 16], 0, 8)
#define SX1302_PKT_TIMESTAMP_23_16(buffer, start_index)     TAKE_N_BITS_FROM(buffer[start_index + 17], 0, 8)
#define SX1302_PKT_TIMESTAMP_31_24(buffer, start_index)     TAKE_N_BITS_FROM(buffer[start_index + 18], 0, 8)
#define SX1302_PKT_CRC_PAYLOAD_7_0(buffer, start_index)     TAKE_N_BITS_FROM(buffer[start_index + 19], 0, 8)
#define SX1302_PKT_CRC_PAYLOAD_15_8(buffer, start_index)    TAKE_N_BITS_FROM(buffer[start_index + 20], 0, 8)
#define SX1302_PKT_NUM_TS_METRICS(buffer, start_index)      TAKE_N_BITS_FROM(buffer[start_index + 21], 0, 8)

/* -------------------------------------------------------------------------- */
/* --- PUBLIC TYPES --------------------------------------------------------- */

typedef enum {
    SX1302_RADIO_TYPE_SX1250,   /* sx1250 */
    SX1302_RADIO_TYPE_SX125X    /* sx1255/1257 */
} sx1302_radio_type_t;

/* -------------------------------------------------------------------------- */
/* --- PUBLIC FUNCTIONS PROTOTYPES ------------------------------------------ */

int sx1302_radio_clock_select(uint8_t rf_chain, bool switch_clock);
int sx1302_radio_reset(uint8_t rf_chain, sx1302_radio_type_t type);
int sx1302_radio_set_mode(uint8_t rf_chain, sx1302_radio_type_t type);

int sx1302_clock_enable(void);

int sx1302_radio_fe_configure();

int sx1302_lora_channelizer_configure(bool * if_rf_chain, int32_t * channel_if, bool fix_gain);
int sx1302_lora_correlator_configure();
int sx1302_lora_modem_configure();

int sx1302_lora_service_channelizer_configure(bool * if_rf_chain, int32_t * channel_if);
int sx1302_lora_service_correlator_configure(uint8_t sf);
int sx1302_lora_service_modem_configure(uint8_t sf, uint8_t bw);

int sx1302_fsk_configure(bool * if_rf_chain, int32_t * channel_if, uint64_t sync_word, uint8_t sync_word_size);

int sx1302_modem_enable();

int sx1302_lora_syncword(bool public);

int sx1302_get_cnt(bool pps, uint32_t* cnt_us);

int sx1302_agc_load_firmware(const uint8_t *firmware);
int sx1302_agc_status(uint8_t* status);
int sx1302_agc_wait_status(uint8_t status);
int sx1302_agc_mailbox_read(uint8_t mailbox, uint8_t* value);
int sx1302_agc_mailbox_write(uint8_t mailbox, uint8_t value);
int sx1302_agc_start(uint8_t version, sx1302_radio_type_t radio_type, uint8_t ana_gain, uint8_t dec_gain, uint8_t fdd_mode);

int sx1302_arb_load_firmware(const uint8_t *firmware);
int sx1302_arb_status(uint8_t* status);
int sx1302_arb_wait_status(uint8_t status);
int sx1302_arb_debug_read(uint8_t reg_id, uint8_t* value);
int sx1302_arb_debug_write(uint8_t reg_id, uint8_t value);
int sx1302_arb_start(uint8_t version);

#endif

/* --- EOF ------------------------------------------------------------------ */
