/*
 / _____)             _              | |
( (____  _____ ____ _| |_ _____  ____| |__
 \____ \| ___ |    (_   _) ___ |/ ___)  _ \
 _____) ) ____| | | || |_| ____( (___| | | |
(______/|_____)_|_|_| \__)_____)\____)_| |_|
  (C)2019 Semtech

Description:
    SX1302 Hardware Abstraction Layer entry functions.

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
#define TX_START_DELAY_DEFAULT      1500    /* Calibrated value for 500KHz BW */

/* type of if_chain + modem */
#define IF_UNDEFINED                0
#define IF_LORA_STD                 0x10    /* if + standard single-SF LoRa modem */
#define IF_LORA_MULTI               0x11    /* if + LoRa receiver with multi-SF capability */
#define IF_FSK_STD                  0x20    /* if + standard FSK modem */

/* -------------------------------------------------------------------------- */
/* --- PUBLIC MACROS -------------------------------------------------------- */

#define REG_SELECT(rf_chain, a, b) ((rf_chain == 0) ? a : b)

#define SET_PPM_ON(bw,dr)   (((bw == BW_125KHZ) && ((dr == DR_LORA_SF11) || (dr == DR_LORA_SF12))) || ((bw == BW_250KHZ) && (dr == DR_LORA_SF12)))

/* -------------------------------------------------------------------------- */
/* --- PUBLIC TYPES --------------------------------------------------------- */

/**
@enum sx1302_model_id_t
@brief
*/
typedef enum {
    CHIP_MODEL_ID_SX1302 = 0x02, /* SX1302 can be 0x00 or 0x02 */
    CHIP_MODEL_ID_SX1303 = 0x03,
    CHIP_MODEL_ID_UNKNOWN
} sx1302_model_id_t;

/**
@enum sx1302_rx_frequency_tracking_t
@brief Frequency Tracking mode
*/
typedef enum {
    RX_FREQ_TRACK_OFF  = 0x00,
    RX_FREQ_TRACK_ON   = 0x01,
    RX_FREQ_TRACK_AUTO = 0x03
} sx1302_rx_frequency_tracking_t;

/**
@enum sx1302_rx_fine_timing_mode_t
@brief Fine Timing mode
*/
typedef enum {
    RX_FINE_TIMING_MODE_ABS     = 0x01,
    RX_FINE_TIMING_MODE_LINEAR  = 0x02,
    RX_FINE_TIMING_MODE_AUTO    = 0x03
} sx1302_rx_fine_timing_mode_t;

/**
@enum sx1302_rx_dft_peak_mode_t
@brief DFT peak mode
*/
typedef enum {
    RX_DFT_PEAK_MODE_DISABLED    = 0x00,
    RX_DFT_PEAK_MODE_FULL        = 0x01,
    RX_DFT_PEAK_MODE_TRACK       = 0x02,
    RX_DFT_PEAK_MODE_AUTO        = 0x03
} sx1302_rx_dft_peak_mode_t;


/* -------------------------------------------------------------------------- */
/* --- PUBLIC FUNCTIONS PROTOTYPES ------------------------------------------ */

/**
@brief Initialize sx1302 for operating, and needed internal structures (rx_buffer,....)
@param conf a pointer to the fine timestamp configuration context
@return LGW_REG_SUCCESS if no error, LGW_REG_ERROR otherwise
*/
int sx1302_init(const struct lgw_conf_ftime_s *conf);

/**
@brief Get the SX1302 unique identifier
@param eui  pointerto the memory holding the concentrator EUI
@return LGW_REG_SUCCESS if no error, LGW_REG_ERROR otherwise
*/
int sx1302_get_eui(uint64_t * eui);

/**
@brief Get the SX1302/SX1303 Chip Model ID
@param model_id pointer to the memory holding the Chip Model ID
@return LGW_REG_SUCCESS if no error, LGW_REG_ERROR otherwise
*/
int sx1302_get_model_id(sx1302_model_id_t * model_id);

/**
@brief Check AGC & ARB MCUs parity error, and update timestamp counter wraping status
@brief This function needs to be called regularly (every few seconds) by the upper layer
@param N/A
@return LGW_REG_SUCCESS if no error, LGW_REG_ERROR otherwise
*/
int sx1302_update(void);

/**
@brief Select the clock source radio
@param rf_chain The RF chain index from which to get the clock source
@return LGW_REG_SUCCESS if success, LGW_REG_ERROR otherwise
*/
int sx1302_radio_clock_select(uint8_t rf_chain);

/**
@brief Apply the radio reset sequence to the required RF chain index
@param rf_chain The RF chain index of the radio to be reset
@param type     The type of radio to be reset
@return LGW_REG_SUCCESS if success, LGW_REG_ERROR otherwise
*/
int sx1302_radio_reset(uint8_t rf_chain, lgw_radio_type_t type);

/**
@brief Configure the radio type for the given RF chain
@param rf_chain The RF chain index to be configured
@param type     The type of radio to be set for the given RF chain
@return LGW_REG_SUCCESS if success, LGW_REG_ERROR otherwise
*/
int sx1302_radio_set_mode(uint8_t rf_chain, lgw_radio_type_t type);

/**
@brief Give/Release control over the radios to/from the Host
@param host_ctrl    Set to true to give control to the host, false otherwise
@return LGW_REG_SUCCESS if success, LGW_REG_ERROR otherwise
*/
int sx1302_radio_host_ctrl(bool host_ctrl);

/**
@brief Perform the radio calibration sequence and fill the TX gain LUT with calibration offsets
@param context_rf_chain The RF chains array from which to get RF chains current configuration
@param clksrc           The RF chain index which provides the clock source
@param txgain_lut       A pointer to the TX gain LUT to be filled
@return LGW_REG_SUCCESS if success, LGW_REG_ERROR otherwise
*/
int sx1302_radio_calibrate(struct lgw_conf_rxrf_s * context_rf_chain, uint8_t clksrc, struct lgw_tx_gain_lut_s * txgain_lut);

/**
@brief Configure the PA and LNA LUTs
@param context_board A pointer to the current board configuration context
@return LGW_REG_SUCCESS if success, LGW_REG_ERROR otherwise
*/
int sx1302_pa_lna_lut_configure(struct lgw_conf_board_s * context_board);

/**
@brief Configure the Radio Front-End stage of the SX1302
@param N/A
@return LGW_REG_SUCCESS if success, LGW_REG_ERROR otherwise
*/
int sx1302_radio_fe_configure(void);

/**
@brief Returns the type of the given modem index (LoRa MultiSF, LoRa SingleSF, FSK)
@param if_chain the index if the IF chain
@return The IF chain type
*/
uint8_t sx1302_get_ifmod_config(uint8_t if_chain);

/**
@brief Configure the channelizer stage of the SX1302
@param if_cfg   A pointer to the channels configuration
@param fix_gain Set to true to force the channelizer to a fixed gain, false to let the AGC controlling it
@return LGW_REG_SUCCESS if success, LGW_REG_ERROR otherwise
*/
int sx1302_channelizer_configure(struct lgw_conf_rxif_s * if_cfg, bool fix_gain);

/**
@brief Configure the correlator stage of the SX1302 LoRa multi-SF modems
@param if_cfg       A pointer to the channels configuration
@param demod_cfg    A pointer to the demodulators configuration
@return LGW_REG_SUCCESS if success, LGW_REG_ERROR otherwise
*/
int sx1302_lora_correlator_configure(struct lgw_conf_rxif_s * if_cfg, struct lgw_conf_demod_s * demod_cfg);

/**
@brief Configure the correlator stage of the SX1302 LoRa single-SF modem
@param cfg  A pointer to the channel configuration
@return LGW_REG_SUCCESS if success, LGW_REG_ERROR otherwise
*/
int sx1302_lora_service_correlator_configure(struct lgw_conf_rxif_s * cfg);

/**
@brief Configure the syncword to be used by LoRa modems (public:0x34, private:0x12)
@param public           Set to true to use the "public" syncword, false to use the private one
@param lora_service_sf  The spreading factor configured for the single-SF LoRa modem
@return LGW_REG_SUCCESS if success, LGW_REG_ERROR otherwise
*/
int sx1302_lora_syncword(bool public, uint8_t lora_service_sf);

/**
@brief Configure the LoRa multi-SF modems
@param radio_freq_hz    The center frequency of the RF chain (0 or 1)
@return LGW_REG_SUCCESS if success, LGW_REG_ERROR otherwise
*/
int sx1302_lora_modem_configure(uint32_t radio_freq_hz);

/**
@brief Configure the LoRa single-SF modem
@param cfg              A pointer to the channel configuration
@param radio_freq_hz    The center frequency of the RF chain (0 or 1)
@return LGW_REG_SUCCESS if success, LGW_REG_ERROR otherwise
*/
int sx1302_lora_service_modem_configure(struct lgw_conf_rxif_s * cfg, uint32_t radio_freq_hz);

/**
@brief Configure the FSK modem
@param cfg  A pointer to the channel configuration
@return LGW_REG_SUCCESS if success, LGW_REG_ERROR otherwise
*/
int sx1302_fsk_configure(struct lgw_conf_rxif_s * cfg);

/**
@brief Enable the modems
@param N/A
@return LGW_REG_SUCCESS if success, LGW_REG_ERROR otherwise
*/
int sx1302_modem_enable(void);

/**
@brief Enable/Disable the GPS to allow PPS trigger and counter sampling
@param enbale   Set to true to enable, false otherwise
@return LGW_REG_SUCCESS if success, LGW_REG_ERROR otherwise
*/
int sx1302_gps_enable(bool enable);

/**
@brief Get the current SX1302 internal counter value
@param pps      True for getting the counter value at last PPS
@return the counter value in mciroseconds (32-bits)
*/
uint32_t sx1302_timestamp_counter(bool pps);

/**
@brief Load firmware to AGC MCU memory
@param firmware A pointer to the fw binary to be loaded
@return LGW_REG_SUCCESS if success, LGW_REG_ERROR otherwise
*/
int sx1302_agc_load_firmware(const uint8_t *firmware);

/**
@brief Read the AGC status register for current status
@param status A pointer to store the current status returned
@return LGW_REG_SUCCESS if success, LGW_REG_ERROR otherwise
*/
int sx1302_agc_status(uint8_t* status);

/**
@brief TODO
@param TODO
@return TODO
*/
int sx1302_agc_wait_status(uint8_t status);

/**
@brief TODO
@param TODO
@return TODO
*/
int sx1302_agc_mailbox_read(uint8_t mailbox, uint8_t* value);

/**
@brief TODO
@param TODO
@return TODO
*/
int sx1302_agc_mailbox_write(uint8_t mailbox, uint8_t value);

/**
@brief TODO
@param TODO
@return TODO
*/
int sx1302_agc_start(uint8_t version, lgw_radio_type_t radio_type, uint8_t ana_gain, uint8_t dec_gain, bool full_duplex, bool lbt_enable);

/**
@brief TODO
@param TODO
@return TODO
*/
int sx1302_arb_load_firmware(const uint8_t *firmware);

/**
@brief TODO
@param TODO
@return TODO
*/
int sx1302_arb_status(uint8_t* status);

/**
@brief TODO
@param TODO
@return TODO
*/
int sx1302_arb_wait_status(uint8_t status);

/**
@brief TODO
@param TODO
@return TODO
*/
int sx1302_arb_debug_read(uint8_t reg_id, uint8_t* value);

/**
@brief TODO
@param TODO
@return TODO
*/
int sx1302_arb_debug_write(uint8_t reg_id, uint8_t value);

/**
@brief TODO
@param TODO
@return TODO
*/
int sx1302_arb_start(uint8_t version, const struct lgw_conf_ftime_s * ftime_context);

/**
@brief TODO
@param TODO
@return TODO
*/
uint8_t sx1302_arb_get_debug_stats_detect(uint8_t channel);

/**
@brief TODO
@param TODO
@return TODO
*/
uint8_t sx1302_arb_get_debug_stats_alloc(uint8_t channel);

/**
@brief TODO
@param TODO
@return TODO
*/
void sx1302_arb_print_debug_stats(void);

/**
@brief TODO
@param TODO
@return TODO
*/
uint16_t sx1302_lora_payload_crc(const uint8_t * data, uint8_t size);

/**
@brief Get the number of packets available in rx_buffer and fetch data from ...
@brief ... the SX1302 if rx_buffer is empty.
@param  nb_pkt A pointer to allocated memory to hold the number of packet fetched
@return LGW_REG_SUCCESS if success, LGW_REG_ERROR otherwise
*/
int sx1302_fetch(uint8_t * nb_pkt);

/**
@brief Parse and return the next packet available in rx_buffer.
@param context      Gateway configuration context
@param p            The structure to get the packet parsed
@return LGW_REG_SUCCESS if a packet could be parsed, LGW_REG_ERROR otherwise
*/
int sx1302_parse(lgw_context_t * context, struct lgw_pkt_rx_s * p);

/**
@brief Configure the delay to be applied by the SX1302 for TX to start
@param rf_chain      RF chain index to be configured
@param radio_type    Type of radio for this RF chain
@param modulation    Modulation used for the TX
@param bandwidth     Bandwidth used for the TX
@param chirp_lowpass Chirp Low Pass filtering configuration
@param delay         TX start delay calculated and applied
@return LGW_REG_SUCCESS if success, LGW_REG_ERROR otherwise
*/
int sx1302_tx_set_start_delay(uint8_t rf_chain, lgw_radio_type_t radio_type, uint8_t modulation, uint8_t bandwidth, uint8_t chirp_lowpass, uint16_t * delay);

/**
@brief Compute the offset to be applied on RSSI for temperature compensation
@param context  a pointer to the memory that holds the current temp comp context
@param temperature  the temperature for which to compute the offset to be applied
@return the offset to be applied to RSSI
*/
float sx1302_rssi_get_temperature_offset(struct lgw_rssi_tcomp_s * context, float temperature);

/**
@brief Get current TX status of the SX1302
@param rf_chain the TX chain we want to get the status from
@return current status
*/
uint8_t sx1302_tx_status(uint8_t rf_chain);

/**
@brief Get current RX status of the SX1302
@param rf_chain the RX chain we want to get the status from
@return current status
@note NOT IMPLEMENTED
*/
uint8_t sx1302_rx_status(uint8_t rf_chain);

/**
@brief Abort current transmit
@param rf_chain the TX chain on which we want to abort transmit
@return LGW_REG_SUCCESS if success, LGW_REG_ERROR otherwise
*/
int sx1302_tx_abort(uint8_t rf_chain);

/**
@brief TODO
@param TODO
@return TODO
*/
int sx1302_tx_configure(lgw_radio_type_t radio_type);

/**
@brief TODO
@param TODO
@return TODO
*/
int sx1302_send(lgw_radio_type_t radio_type, struct lgw_tx_gain_lut_s * tx_lut, bool lwan_public, struct lgw_conf_rxif_s * context_fsk, struct lgw_pkt_tx_s * pkt_data);

/**
@brief TODO
@param TODO
@return TODO
*/
int sx1302_set_gpio(uint8_t gpio_reg_val);

/**
@brief TODO
@param TODO
@return TODO
*/
double sx1302_dc_notch_delay(double if_freq_hz);

#endif

/* --- EOF ------------------------------------------------------------------ */
