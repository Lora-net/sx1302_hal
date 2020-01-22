/*
 / _____)             _              | |
( (____  _____ ____ _| |_ _____  ____| |__
 \____ \| ___ |    (_   _) ___ |/ ___)  _ \
 _____) ) ____| | | || |_| ____( (___| | | |
(______/|_____)_|_|_| \__)_____)\____)_| |_|
  (C)2019 Semtech

Description:
    LoRa 2.4GHz concentrator MCU interface functions

License: Revised BSD License, see LICENSE.TXT file include in the project
*/

#ifndef _LORAGW_MCU_H
#define _LORAGW_MCU_H

/* -------------------------------------------------------------------------- */
/* --- DEPENDANCIES --------------------------------------------------------- */

#include "loragw_hal.h"

#include "config.h"    /* library configuration options (dynamically generated) */

/* -------------------------------------------------------------------------- */
/* --- PUBLIC MACROS -------------------------------------------------------- */

/* -------------------------------------------------------------------------- */
/* --- PUBLIC CONSTANTS ----------------------------------------------------- */

#define NB_RADIO_RX_MAX                 (3)

// Command without payload
#define ORDER_REQ_PING_SIZE             (0)
#define ORDER_REQ_GET_STATUS_SIZE       (0)
#define ORDER_UNKNOW_CMD_SIZE           (0)
#define ORDER_REQ_GET_RX_MSG_SIZE       (0)
#define ORDER_REQ_GET_TX_STATUS         (0)
#define ORDER_REQ_BOOTLOADER_MODE_SIZE  (0)
#define ACK_BOOTLOADER_MODE_SIZE        (0)
#define ACK_WRITE_REG_SIZE              (0)


/* -------------------------------------------------------------------------- */
/* --- PUBLIC TYPES --------------------------------------------------------- */

typedef enum {
    ORDER_ID__REQ_PING               = 0x00,
    ORDER_ID__REQ_GET_STATUS         = 0x01,
    ORDER_ID__REQ_BOOTLOADER_MODE    = 0x02,
    ORDER_ID__REQ_RESET              = 0x03,
    ORDER_ID__REQ_WRITE_GPIO         = 0x04,
    ORDER_ID__REQ_SPI                = 0x05,

    ORDER_ID__ACK_PING               = 0x40,
    ORDER_ID__ACK_GET_STATUS         = 0x41,
    ORDER_ID__ACK_BOOTLOADER_MODE    = 0x42,
    ORDER_ID__ACK_RESET              = 0x43,
    ORDER_ID__ACK_WRITE_GPIO         = 0x44,
    ORDER_ID__ACK_SPI                = 0x45,

    ORDER_ID__UNKNOW_CMD = 0xFF

} e_order_cmd;

typedef enum {
    CMD_OFFSET__ID,
    CMD_OFFSET__SIZE_MSB,
    CMD_OFFSET__SIZE_LSB,
    CMD_OFFSET__CMD,
    CMD_OFFSET__DATA
} e_cmd_order_offset;

typedef enum
{
    REQ_PREPARE_TX__MSG_IS_TIMESTAMP,
    REQ_PREPARE_TX__TIMESTAMP_31_24,    REQ_PREPARE_TX__TIMESTAMP_23_16,    REQ_PREPARE_TX__TIMESTAMP_15_8,     REQ_PREPARE_TX__TIMESTAMP_7_0,
    REQ_PREPARE_TX__POWER,
    REQ_PREPARE_TX__FREQ_31_24,         REQ_PREPARE_TX__FREQ_23_16,         REQ_PREPARE_TX__FREQ_15_8,          REQ_PREPARE_TX__FREQ_7_0,
    REQ_PREPARE_TX__BW,
    REQ_PREPARE_TX__SF,
    REQ_PREPARE_TX__USE_INVERSE_IQ,
    REQ_PREPARE_TX__CR,
    REQ_PREPARE_TX__USE_IMPLICIT_HEADER,
    REQ_PREPARE_TX__USE_CRC,
    REQ_PREPARE_TX__RAMP_UP,
    REQ_PREPARE_TX__PREAMBLE_15_8,      REQ_PREPARE_TX__PREAMBLE_7_0,
    REQ_PREPARE_TX__PAYLOAD_LEN,
    REQ_PREPARE_TX__PAYLOAD /* keep this as the last enum */
    // size isn't fix
} e_cmd_offset_req_prepare_tx;

typedef enum {
    ACK_GET_STATUS__SYSTEM_TIME_31_24,      ACK_GET_STATUS__SYSTEM_TIME_23_16,      ACK_GET_STATUS__SYSTEM_TIME_15_8,   ACK_GET_STATUS__SYSTEM_TIME_7_0,
    ACK_GET_STATUS__PRECISE_TIMER_31_24,    ACK_GET_STATUS__PRECISE_TIMER_23_16,    ACK_GET_STATUS__PRECISE_TIMER_15_8, ACK_GET_STATUS__PRECISE_TIMER_7_0,
    ACK_GET_STATUS__PPS_STATUS,
    ACK_GET_STATUS__PPS_TIME_31_24,         ACK_GET_STATUS__PPS_TIME_23_16,         ACK_GET_STATUS__PPS_TIME_15_8,      ACK_GET_STATUS__PPS_TIME_7_0,
    ACK_GET_STATUS__TEMPERATURE_15_8,       ACK_GET_STATUS__TEMPERATURE_7_0,
    ACK_GET_STATUS__RX_STATUS /* 4 bytes per radio : NB_PKT_CRC_OK | NB_PKT_CRC_ERR */
    //Size is fix but determine by NB_RADIO_RX
} e_cmd_offset_ack_get_status;

typedef enum
{
    ACK_PING__UNIQUE_ID_0,  ACK_PING__UNIQUE_ID_1,  ACK_PING__UNIQUE_ID_2,  ACK_PING__UNIQUE_ID_3,
    ACK_PING__UNIQUE_ID_4,  ACK_PING__UNIQUE_ID_5,  ACK_PING__UNIQUE_ID_6,  ACK_PING__UNIQUE_ID_7,
    ACK_PING__UNIQUE_ID_8,  ACK_PING__UNIQUE_ID_9,  ACK_PING__UNIQUE_ID_10, ACK_PING__UNIQUE_ID_11,
    ACK_PING__VERSION_0,    ACK_PING__VERSION_1,    ACK_PING__VERSION_2,    ACK_PING__VERSION_3,    ACK_PING__VERSION_4,
    ACK_PING__VERSION_5,    ACK_PING__VERSION_6,    ACK_PING__VERSION_7,    ACK_PING__VERSION_8,
    ACK_PING__NB_RADIO_TX,  ACK_PING__NB_RADIO_RX,
    ACK_PING_SIZE,
} e_cmd_offset_ack_ping;


typedef enum {
    ACK_PREPARE_TX__STATUS,
    ACK_PREPARE_TX_SIZE
} e_cmd_offset_ack_prepare_tx;

typedef enum
{
    ACK_GET_TX_STATUS__STATUS,
    ACK_GET_TX_STATUS_SIZE
} e_cmd_offset_ack_get_tx_status;

typedef enum {
    TX_STATUS__IDLE,
    TX_STATUS__LOADED,
    TX_STATUS__ON_AIR,
    TX_STATUS__DONE,
    TX_STATUS__ERROR_PARAM,
    TX_STATUS__ERROR_FAIL_TO_SEND,
    TX_STATUS__ERROR_TX_TIMEOUT
} e_tx_msg_status;

typedef enum
{
    PPS_STATUS__NEVER_LOCK,
    PPS_STATUS__LOCK,
    PPS_STATUS__LOCK_LOST
} e_pps_status;

typedef enum {
    PREPARE_TX_STATUS__OK,
    PREPARE_TX_STATUS__INVALID,
    PREPARE_TX_STATUS__ERROR_FIFO_FULL
} e_prepare_tx_status;

typedef enum {
    REQ_CONF_RX__RADIO_IDX,
    REQ_CONF_RX__FREQ_31_24,        REQ_CONF_RX__FREQ_23_16,        REQ_CONF_RX__FREQ_15_8,     REQ_CONF_RX__FREQ_7_0,
    REQ_CONF_RX__PREAMBLE_LEN_15_8, REQ_CONF_RX__PREAMBLE_LEN_7_0,
    REQ_CONF_RX__SF,
    REQ_CONF_RX__BW,
    REQ_CONF_RX__USE_IQ_INVERTED,
    REQ_CONF_RX_SIZE
} e_cmd_offset_req_config_rx;

typedef enum {
    ACK_CONFIG_RX__STATUS,
    ACK_CONFIG_RX_SIZE
} e_cmd_offset_ack_config_rx;

typedef enum {
    CONFIG_RX_SATUS__DONE,
    CONFIG_RX_SATUS__ERROR_PARAM,
    CONFIG_RX_SATUS__ERROR_FAILED
} e_config_rx_status;

typedef enum
{
    ACK_GET_RX_MSG__NB_MSG,
    ACK_GET_RX_MSG__NB_BYTES_15_8,    ACK_GET_RX_MSG__NB_BYTES_7_0,
    ACK_GET_RX_MSG__MSG_PENDING,
    ACK_GET_RX_MSG__LOST_MESSAGE,
    ACK_GET_RX_MSG_SIZE
} e_cmd_offset_ack_get_rx_msg;


typedef enum {
    EVT_MSG_RECEIVE__RADIO_IDX,
    EVT_MSG_RECEIVE__TIMESTAMP_31_24, EVT_MSG_RECEIVE__TIMESTAMP_23_16, EVT_MSG_RECEIVE__TIMESTAMP_15_8, EVT_MSG_RECEIVE__TIMESTAMP_7_0,
    EVT_MSG_RECEIVE__ERROR_FREQ_31_24, EVT_MSG_RECEIVE__ERROR_FREQ_23_16, EVT_MSG_RECEIVE__ERROR_FREQ_15_8, EVT_MSG_RECEIVE__ERROR_FREQ_7_0,
    EVT_MSG_RECEIVE__SNR,
    EVT_MSG_RECEIVE__RSSI,
    EVT_MSG_RECEIVE__PAYLOAD_LEN,
    EVT_MSG_RECEIVE__PAYLOAD
    // size isn't fix
} e_cmd_offset_evt_msg_receive;

typedef enum
{
    REQ_RESET__TYPE,
    REQ_RESET_SIZE
} e_cmd_offset_req_reset;

typedef enum
{
    ACK_RESET__STATUS,
    ACK_RESET_SIZE
} e_cmd_offset_ack_reset;

typedef enum
{
    REQ_READ_REGS__RADIO_IDX,
    REQ_READ_REGS__ADDR_15_8,   REQ_READ_REGS__ADDR_7_0,
    REQ_READ_REGS_SIZE
} e_cmd_offset_req_read_regs;

typedef enum
{
    ACK_READ_REG__VALUE,
    ACK_READ_REG_SIZE,
} e_cmd_offset_ack_read_reg;


typedef enum
{
    ACK_SPI__VALUE,
    ACK_SPI_SIZE,
} e_cmd_offset_ack_spi_access;

typedef enum
{
    ACK_GPIO_WRITE__STATUS,
    ACK_GPIO_WRITE_SIZE
}e_cmd_offset_ack_gpio_write;

typedef enum
{
    REQ_WRITE_REGS__RADIO_IDX,
    REQ_WRITE_REGS__ADDR_15_8,   REQ_WRITE_REGS__ADDR_7_0,
    REQ_WRITE_REGS__DATA,
    REQ_WRITE_REGS_SIZE,
} e_cmd_offset_req_write_regs;

typedef enum
{
    RESET_TYPE__GTW,
    RESET_TYPE__TX,
    RESET_TYPE__RX1,
    RESET_TYPE__RX2,
    RESET_TYPE__RX3,
    RESET_TYPE__RX4,
    RESET_TYPE__RX5,
    RESET_TYPE__RX6,
    RESET_TYPE__RX_ALL,
} e_reset_type;

typedef struct {
    uint8_t nb_radio_tx;
    uint8_t nb_radio_rx;
    uint32_t unique_id_high;
    uint32_t unique_id_mid;
    uint32_t unique_id_low;
    char version[10]; /* format is V00.00.00\0 */
} s_ping_info;

typedef struct {
    uint32_t system_time_ms;
    uint32_t precise_time_us;
    e_pps_status pps_status;
    uint32_t pps_time_us;
    float temperature;
    uint16_t rx_crc_ok[NB_RADIO_RX_MAX];
    uint16_t rx_crc_err[NB_RADIO_RX_MAX];
} s_status;

typedef struct {
    uint8_t nb_msg;
    uint16_t nb_bytes;
    uint8_t pending;
    uint8_t lost_message;
} s_rx_msg;

/* -------------------------------------------------------------------------- */
/* --- PUBLIC FUNCTIONS PROTOTYPES ------------------------------------------ */

int mcu_open(const char * tty_path);

int mcu_close(int fd);

int mcu_get_status(int fd , float * temperature);

int mcu_ping(int fd, s_ping_info * info);
int mcu_reset(int fd, bool reset_mcu);

int mcu_boot(int fd);


uint8_t mcu_get_nb_rx_radio(void);

uint8_t mcu_get_nb_tx_radio(void);
int mcu_spi_access(int fd, uint8_t * buf, size_t buf_size,uint8_t * buf_ack) ;
int mcu_gpio_write(int fd,  uint8_t gpio_port ,uint8_t gpio_id, uint8_t gpio_value);




int lgw_usb_w(int usb_target, uint8_t spi_mux_target, uint16_t address, uint8_t data) ;

int lgw_usb_r(int usb_target, uint8_t spi_mux_target, uint16_t address, uint8_t *data);


/* Burst (multiple-byte) write */
int lgw_usb_wb(int usb_target, uint8_t spi_mux_target, uint16_t address, const uint8_t *data, uint16_t size);
/* ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ */

/* Burst (multiple-byte) read */
int lgw_usb_rb(int usb_target, uint8_t spi_mux_target, uint16_t address, uint8_t *data, uint16_t size);



/* --- EOF ------------------------------------------------------------------ */
#endif