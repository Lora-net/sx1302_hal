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

#define SX1302_AGC_RADIO_GAIN_AUTO 0

/* -------------------------------------------------------------------------- */
/* --- PUBLIC MACROS -------------------------------------------------------- */

#define REG_SELECT(rf_chain, a, b) ((rf_chain == 0) ? a : b)

/* -------------------------------------------------------------------------- */
/* --- PUBLIC TYPES --------------------------------------------------------- */

typedef enum {
    SX1302_RADIO_TYPE_SX1250,   /* sx1250 */
    SX1302_RADIO_TYPE_SX125X    /* sx1255/1257 */
} sx1302_radio_type_t;

/* -------------------------------------------------------------------------- */
/* --- PUBLIC FUNCTIONS PROTOTYPES ------------------------------------------ */

int sx1302_radio_clock_select(uint8_t rf_chain);
int sx1302_radio_reset(uint8_t rf_chain, sx1302_radio_type_t type);
int sx1302_radio_set_mode(uint8_t rf_chain, sx1302_radio_type_t type);

int sx1302_clock_enable(void);

int sx1302_lora_channelizer_configure(bool * if_rf_chain, int32_t * channel_if);
int sx1302_lora_correlator_configure();
int sx1302_lora_modem_configure();

int sx1302_lora_service_channelizer_configure(bool * if_rf_chain, int32_t * channel_if);
int sx1302_lora_service_correlator_configure(uint8_t sf);
int sx1302_lora_service_modem_configure(uint8_t sf, uint8_t bw);

int sx1302_modem_enable();

int sx1302_lora_syncword(bool public);

int sx1302_get_cnt(bool pps, uint32_t* cnt_us);

int sx1302_agc_status(uint8_t* status);
int sx1302_agc_wait_status(uint8_t status);
int sx1302_agc_mailbox_read(uint8_t mailbox, uint8_t* value);
int sx1302_agc_mailbox_write(uint8_t mailbox, uint8_t value);
int sx1302_agc_start(uint8_t version, sx1302_radio_type_t radio_type, uint8_t ana_gain, uint8_t dec_gain, uint8_t fdd_mode);

int sx1302_arb_status(uint8_t* status);
int sx1302_arb_wait_status(uint8_t status);
int sx1302_agc_debug_read(uint8_t reg_id, uint8_t* value);
int sx1302_agc_debug_write(uint8_t reg_id, uint8_t value);
int sx1302_arb_start(uint8_t version);

#endif

/* --- EOF ------------------------------------------------------------------ */
