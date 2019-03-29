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

/* Default values */
#define SX1302_AGC_RADIO_GAIN_AUTO  0xFF
#define TX_START_DELAY_DEFAULT      1500 /* Calibrated value for 500KHz BW */ /* TODO */

/* RX buffer packet structure */
#define SX1302_PKT_SYNCWORD_BYTE_0  0xA5
#define SX1302_PKT_SYNCWORD_BYTE_1  0xC0
#define SX1302_PKT_HEAD_METADATA    9
#define SX1302_PKT_TAIL_METADATA    14

/* modem IDs */
#if FPGA_BOARD_16_CH
#define SX1302_LORA_MODEM_ID_MAX    15
#define SX1302_LORA_STD_MODEM_ID    16
#define SX1302_FSK_MODEM_ID         17
#else
#define SX1302_LORA_MODEM_ID_MAX    11
#define SX1302_LORA_STD_MODEM_ID    12
#define SX1302_FSK_MODEM_ID         13
#endif

/* -------------------------------------------------------------------------- */
/* --- PUBLIC MACROS -------------------------------------------------------- */

#define REG_SELECT(rf_chain, a, b) ((rf_chain == 0) ? a : b)

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
/* --- PUBLIC TYPES --------------------------------------------------------- */

/**
@struct sx1302_if_cfg_t
@brief TODO
*/
typedef struct {
    bool        if_enable;
    bool        if_rf_chain; /* for each IF, 0 -> radio A, 1 -> radio B */
    int32_t     if_freq; /* relative to radio frequency, +/- in Hz */
} sx1302_if_cfg_t;

/**
@struct sx1302_lora_service_cfg_t
@brief TODO
*/
typedef struct {
    uint8_t     lora_rx_bw; /* bandwidth setting for LoRa standalone modem */
    uint8_t     lora_rx_sf; /* spreading factor setting for LoRa standalone modem */
    bool        lora_rx_implicit_hdr; /* implicit header setting for LoRa standalone modem */
    uint8_t     lora_rx_implicit_length; /* implicit header payload length setting for LoRa standalone modem */
    bool        lora_rx_implicit_crc_en; /* implicit header payload crc enable setting for LoRa standalone modem */
    uint8_t     lora_rx_implicit_coderate; /* implicit header payload coderate setting for LoRa standalone modem */
} sx1302_lora_service_cfg_t;

/**
@struct sx1302_fsk_cfg_t
@brief TODO
*/
typedef struct {
    uint8_t     fsk_rx_bw; /* bandwidth setting of FSK modem */
    uint32_t    fsk_rx_dr; /* FSK modem datarate in bauds */
    uint8_t     fsk_sync_word_size; /* default number of bytes for FSK sync word */
    uint64_t    fsk_sync_word; /* default FSK sync word (ALIGNED RIGHT, MSbit first) */
} sx1302_fsk_cfg_t;

/* -------------------------------------------------------------------------- */
/* --- PUBLIC FUNCTIONS PROTOTYPES ------------------------------------------ */

int sx1302_radio_clock_select(uint8_t rf_chain);
int sx1302_radio_reset(uint8_t rf_chain, lgw_radio_type_t type);
int sx1302_radio_set_mode(uint8_t rf_chain, lgw_radio_type_t type);
int sx1302_radio_host_ctrl(bool host_ctrl);

int sx1302_radio_fe_configure();

int sx1302_channelizer_configure(struct lgw_conf_rxif_s * if_cfg, bool fix_gain);

int sx1302_lora_correlator_configure();
int sx1302_lora_service_correlator_configure(struct lgw_conf_rxif_s * cfg);
int sx1302_lora_syncword(bool public, uint8_t lora_service_sf);

int sx1302_lora_modem_configure(uint32_t radio_freq_hz);
int sx1302_lora_service_modem_configure(struct lgw_conf_rxif_s * cfg, uint32_t radio_freq_hz);
int sx1302_fsk_configure(struct lgw_conf_rxif_s * cfg);

int sx1302_modem_enable();

int sx1302_gps_enable(bool enable);

int sx1302_timestamp_mode(struct lgw_conf_timestamp_s *conf);
int sx1302_timestamp_counter(bool pps, uint32_t* cnt_us);
int sx1302_timestamp_expand(bool pps, uint32_t * cnt_us);

int sx1302_agc_load_firmware(const uint8_t *firmware);
int sx1302_agc_status(uint8_t* status);
int sx1302_agc_wait_status(uint8_t status);
int sx1302_agc_mailbox_read(uint8_t mailbox, uint8_t* value);
int sx1302_agc_mailbox_write(uint8_t mailbox, uint8_t value);
int sx1302_agc_start(uint8_t version, lgw_radio_type_t radio_type, uint8_t ana_gain, uint8_t dec_gain, uint8_t fdd_mode);

int sx1302_arb_load_firmware(const uint8_t *firmware);
int sx1302_arb_status(uint8_t* status);
int sx1302_arb_wait_status(uint8_t status);
int sx1302_arb_debug_read(uint8_t reg_id, uint8_t* value);
int sx1302_arb_debug_write(uint8_t reg_id, uint8_t value);
int sx1302_arb_start(uint8_t version);

uint8_t sx1302_arb_get_debug_stats_detect(uint8_t channel);
uint8_t sx1302_arb_get_debug_stats_alloc(uint8_t channel);
void sx1302_arb_print_debug_stats(bool full);

uint16_t sx1302_lora_payload_crc(const uint8_t * data, uint8_t size);

void sx1302_rx_buffer_dump(FILE * file, uint16_t start_addr, uint16_t end_addr);
uint16_t sx1302_rx_buffer_read_ptr_addr(void);
uint16_t sx1302_rx_buffer_write_ptr_addr(void);

#endif

/* --- EOF ------------------------------------------------------------------ */
