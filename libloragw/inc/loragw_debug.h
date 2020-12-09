/*
 / _____)             _              | |
( (____  _____ ____ _| |_ _____  ____| |__
 \____ \| ___ |    (_   _) ___ |/ ___)  _ \
 _____) ) ____| | | || |_| ____( (___| | | |
(______/|_____)_|_|_| \__)_____)\____)_| |_|
  (C)2019 Semtech

Description:
    LoRa concentrator HAL debug functions

License: Revised BSD License, see LICENSE.TXT file include in the project
*/


#ifndef _LORAGW_DBG_H
#define _LORAGW_DBG_H

/* -------------------------------------------------------------------------- */
/* --- DEPENDANCIES --------------------------------------------------------- */

#include "config.h"    /* library configuration options (dynamically generated) */

/* -------------------------------------------------------------------------- */
/* --- PUBLIC MACROS -------------------------------------------------------- */

/* -------------------------------------------------------------------------- */
/* --- PUBLIC FUNCTIONS PROTOTYPES ------------------------------------------ */

/**
@brief
@param
*/
void dbg_log_buffer_to_file(FILE * file, uint8_t * buffer, uint16_t size);

/**
@brief
@param
*/
void dbg_log_payload_diff_to_file(FILE * file, uint8_t * buffer1, uint8_t * buffer2, uint16_t size);

/**
@brief
@param
*/
void dbg_init_random(void);

/**
@brief
@param
*/
void dbg_generate_random_payload(uint32_t pkt_cnt, uint8_t * buffer_expected, uint8_t size);

/**
@brief
@param
*/
int dbg_check_payload(struct lgw_conf_debug_s * context, FILE * file, uint8_t * payload_received, uint8_t size, uint8_t ref_payload_idx, uint8_t sf);

#endif

/* --- EOF ------------------------------------------------------------------ */
