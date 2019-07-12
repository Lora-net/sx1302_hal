/*
 / _____)             _              | |
( (____  _____ ____ _| |_ _____  ____| |__
 \____ \| ___ |    (_   _) ___ |/ ___)  _ \
 _____) ) ____| | | || |_| ____( (___| | | |
(______/|_____)_|_|_| \__)_____)\____)_| |_|
  (C)2019 Semtech

Description:
    SX1302 AGC parameters definition.

License: Revised BSD License, see LICENSE.TXT file include in the project
*/


#ifndef _LORAGW_AGC_PARAMS_H
#define _LORAGW_AGC_PARAMS_H

/* -------------------------------------------------------------------------- */
/* --- PRIVATE TYPES -------------------------------------------------------- */

struct agc_gain_params_s {
    uint8_t ana_min;
    uint8_t ana_max;
    uint8_t ana_thresh_l;
    uint8_t ana_thresh_h;
    uint8_t dec_attn_min;
    uint8_t dec_attn_max;
    uint8_t dec_thresh_l;
    uint8_t dec_thresh_h1;
    uint8_t dec_thresh_h2;
    uint8_t chan_attn_min;
    uint8_t chan_attn_max;
    uint8_t chan_thresh_l;
    uint8_t chan_thresh_h;
    uint8_t deviceSel;      /* sx1250 only */
    uint8_t hpMax;          /* sx1250 only */
    uint8_t paDutyCycle;    /* sx1250 only */
};

/* -------------------------------------------------------------------------- */
/* --- PRIVATE CONSTANTS ---------------------------------------------------- */

const struct agc_gain_params_s agc_params_sx1250 = {
    .ana_min = 1,
    .ana_max = 13,
    .ana_thresh_l = 3,
    .ana_thresh_h = 12,
    .dec_attn_min = 4,
    .dec_attn_max = 15,
    .dec_thresh_l = 40,
    .dec_thresh_h1 = 80,
    .dec_thresh_h2 = 90,
    .chan_attn_min = 4,
    .chan_attn_max = 14,
    .chan_thresh_l = 52,
    .chan_thresh_h = 132,
    .deviceSel = 0,
    .hpMax = 7,
    .paDutyCycle = 4
};

const struct agc_gain_params_s agc_params_sx125x = {
    .ana_min = 0,
    .ana_max = 9,
    .ana_thresh_l = 16,
    .ana_thresh_h = 35,
    .dec_attn_min = 7,
    .dec_attn_max = 11,
    .dec_thresh_l = 45,
    .dec_thresh_h1 = 100,
    .dec_thresh_h2 = 115,
    .chan_attn_min = 4,
    .chan_attn_max = 14,
    .chan_thresh_l = 52,
    .chan_thresh_h = 132,
    .deviceSel = 0,
    .hpMax = 0,
    .paDutyCycle = 0
};

#endif

/* --- EOF ------------------------------------------------------------------ */
