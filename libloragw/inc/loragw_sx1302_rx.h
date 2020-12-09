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


#ifndef _LORAGW_SX1302_RX_H
#define _LORAGW_SX1302_RX_H

/* -------------------------------------------------------------------------- */
/* --- DEPENDANCIES --------------------------------------------------------- */

#include <stdint.h>     /* C99 types*/

#include "config.h"     /* library configuration options (dynamically generated) */

/* -------------------------------------------------------------------------- */
/* --- PUBLIC CONSTANTS ----------------------------------------------------- */

/* -------------------------------------------------------------------------- */
/* --- PUBLIC MACROS -------------------------------------------------------- */

/* -------------------------------------------------------------------------- */
/* --- PUBLIC TYPES --------------------------------------------------------- */

/**
@struct rx_packet_s
@brief packet structure as contained in the sx1302 RX packet engine
*/
typedef struct rx_packet_s {
    uint8_t     rxbytenb_modem;
    uint8_t     rx_channel_in;
    bool        crc_en;
    uint8_t     coding_rate;                /* LoRa only */
    uint8_t     rx_rate_sf;                 /* LoRa only */
    uint8_t     modem_id;
    int32_t     frequency_offset_error;     /* LoRa only */
    uint8_t     payload[255];
    bool        payload_crc_error;
    bool        sync_error;                 /* LoRa only */
    bool        header_error;               /* LoRa only */
    bool        timing_set;                 /* LoRa only */
    int8_t      snr_average;                /* LoRa only */
    uint8_t     rssi_chan_avg;
    uint8_t     rssi_signal_avg;            /* LoRa only */
    uint8_t     rssi_chan_max_neg_delta;
    uint8_t     rssi_chan_max_pos_delta;
    uint8_t     rssi_sig_max_neg_delta;     /* LoRa only */
    uint8_t     rssi_sig_max_pos_delta;     /* LoRa only */
    uint32_t    timestamp_cnt;
    uint16_t    rx_crc16_value;             /* LoRa only */
    uint8_t     num_ts_metrics_stored;      /* LoRa only */
    int8_t      timestamp_avg[255];         /* LoRa only */
    int8_t      timestamp_stddev[255];      /* LoRa only */
    uint8_t     packet_checksum;
} rx_packet_t;

/**
@struct rx_buffer_s
@brief buffer to hold the data fetched from the sx1302 RX buffer
*/
typedef struct rx_buffer_s {
    uint8_t buffer[4096];   /*!> byte array to hald the data fetched from the RX buffer */
    uint16_t buffer_size;   /*!> The number of bytes currently stored in the buffer */
    int buffer_index;       /*!> Current parsing index in the buffer */
    uint8_t buffer_pkt_nb;
} rx_buffer_t;

/* -------------------------------------------------------------------------- */
/* --- PUBLIC FUNCTIONS PROTOTYPES ------------------------------------------ */

/**
@brief Initialize the rx_buffer instance
@param self     A pointer to a rx_buffer handler
@return LGW_REG_SUCCESS if success, LGW_REG_ERROR otherwise
*/
int rx_buffer_new(rx_buffer_t * self);

/**
@brief Reset the rx_buffer instance
@param self     A pointer to a rx_buffer handler
@return LGW_REG_SUCCESS if success, LGW_REG_ERROR otherwise
*/
int rx_buffer_del(rx_buffer_t * self);

/**
@brief Fetch packets from the SX1302 internal RX buffer, and count packets available.
@param self     A pointer to a rx_buffer handler
@return LGW_REG_SUCCESS if success, LGW_REG_ERROR otherwise
*/
int rx_buffer_fetch(rx_buffer_t * self);

/**
@brief Parse the rx_buffer and return the first packet available in the given structure.
@param self     A pointer to a rx_buffer handler
@param pkt      A pointer to the structure to receive the packet parsed
@return LGW_REG_SUCCESS if success, LGW_REG_ERROR otherwise
*/
int rx_buffer_pop(rx_buffer_t * self, rx_packet_t * pkt);

/* -------------------------------------------------------------------------- */
/* --- DEBUG FUNCTIONS PROTOTYPES ------------------------------------------- */

uint16_t rx_buffer_read_ptr_addr(void);

uint16_t rx_buffer_write_ptr_addr(void);

void rx_buffer_dump(FILE * file, uint16_t start_addr, uint16_t end_addr);

#endif

/* --- EOF ------------------------------------------------------------------ */
