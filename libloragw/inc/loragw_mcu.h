/*
 / _____)             _              | |
( (____  _____ ____ _| |_ _____  ____| |__
 \____ \| ___ |    (_   _) ___ |/ ___)  _ \
 _____) ) ____| | | || |_| ____( (___| | | |
(______/|_____)_|_|_| \__)_____)\____)_| |_|
  (C)2020 Semtech

Description:
    Host specific functions to address the LoRa concentrator MCU for USB
    interface.
    Single-byte read/write and burst read/write.

License: Revised BSD License, see LICENSE.TXT file include in the project
*/


#ifndef _LORAGW_MCU_H
#define _LORAGW_MCU_H

/* -------------------------------------------------------------------------- */
/* --- DEPENDANCIES --------------------------------------------------------- */

#include <stdint.h>   /* C99 types*/

#include "config.h"   /* library configuration options (dynamically generated) */

/* -------------------------------------------------------------------------- */
/* --- PUBLIC CONSTANTS ----------------------------------------------------- */

static const char mcu_version_string[] = "01.00.00";

#define MAX_SIZE_COMMAND ( 4200 )
#define MAX_SPI_COMMAND ( MAX_SIZE_COMMAND - CMD_OFFSET__DATA - 1 )

#define LGW_USB_BURST_CHUNK ( 4096 )

/* -------------------------------------------------------------------------- */
/* --- PUBLIC TYPES --------------------------------------------------------- */

typedef enum order_id_e
{
    ORDER_ID__REQ_PING            = 0x00,
    ORDER_ID__REQ_GET_STATUS      = 0x01,
    ORDER_ID__REQ_BOOTLOADER_MODE = 0x02,
    ORDER_ID__REQ_RESET           = 0x03,
    ORDER_ID__REQ_WRITE_GPIO      = 0x04,
    ORDER_ID__REQ_MULTIPLE_SPI    = 0x05,

    ORDER_ID__ACK_PING            = 0x40,
    ORDER_ID__ACK_GET_STATUS      = 0x41,
    ORDER_ID__ACK_BOOTLOADER_MODE = 0x42,
    ORDER_ID__ACK_RESET           = 0x43,
    ORDER_ID__ACK_WRITE_GPIO      = 0x44,
    ORDER_ID__ACK_MULTIPLE_SPI    = 0x45,

    ORDER_ID__CMD_ERROR = 0xFF
} order_id_t;

typedef enum
{
    CMD_OFFSET__ID,
    CMD_OFFSET__SIZE_MSB,
    CMD_OFFSET__SIZE_LSB,
    CMD_OFFSET__CMD,
    CMD_OFFSET__DATA
} e_cmd_order_offset;

typedef enum
{
    REQ_RESET__TYPE,
    REQ_RESET_SIZE
} e_cmd_offset_req_reset;

typedef enum
{
    REQ_WRITE_GPIO__PORT,
    REQ_WRITE_GPIO__PIN,
    REQ_WRITE_GPIO__STATE,
    REQ_WRITE_GPIO_SIZE
} e_cmd_offset_req_write_gpio;

typedef enum
{
    ACK_PING__UNIQUE_ID_0,  ACK_PING__UNIQUE_ID_1,  ACK_PING__UNIQUE_ID_2,  ACK_PING__UNIQUE_ID_3,
    ACK_PING__UNIQUE_ID_4,  ACK_PING__UNIQUE_ID_5,  ACK_PING__UNIQUE_ID_6,  ACK_PING__UNIQUE_ID_7,
    ACK_PING__UNIQUE_ID_8,  ACK_PING__UNIQUE_ID_9,  ACK_PING__UNIQUE_ID_10, ACK_PING__UNIQUE_ID_11,
    ACK_PING__VERSION_0,    ACK_PING__VERSION_1,    ACK_PING__VERSION_2,    ACK_PING__VERSION_3,    ACK_PING__VERSION_4,
    ACK_PING__VERSION_5,    ACK_PING__VERSION_6,    ACK_PING__VERSION_7,    ACK_PING__VERSION_8,
    ACK_PING_SIZE,
} e_cmd_offset_ack_ping;

typedef enum
{
    ACK_GET_STATUS__SYSTEM_TIME_31_24,      ACK_GET_STATUS__SYSTEM_TIME_23_16,      ACK_GET_STATUS__SYSTEM_TIME_15_8,   ACK_GET_STATUS__SYSTEM_TIME_7_0,
    ACK_GET_STATUS__TEMPERATURE_15_8,       ACK_GET_STATUS__TEMPERATURE_7_0,
    ACK_GET_STATUS_SIZE
} e_cmd_offset_ack_get_status;

typedef enum
{
    ACK_GPIO_WRITE__STATUS,
    ACK_GPIO_WRITE_SIZE
} e_cmd_offset_ack_gpio_write;

typedef enum
{
    ACK_RESET__STATUS,
    ACK_RESET_SIZE
} e_cmd_offset_ack_reset;

typedef enum
{
    MCU_SPI_TARGET_SX1302,  /* SX1302 + SX1250 */
    MCU_SPI_TARGET_SX1261   /* LBT/Spectral Scan additional radio */
} e_cmd_spi_target;

typedef enum
{
    MCU_SPI_REQ_TYPE_READ_WRITE         = 0x01, /* Read/Write SPI request */
    MCU_SPI_REQ_TYPE_READ_MODIFY_WRITE  = 0x02  /* Read-Modify-Write SPI request */
} e_cmd_spi_req_type;

typedef enum
{
    RESET_TYPE__GTW
} e_reset_type;

typedef enum
{
    SPI_STATUS_OK,
    SPI_STATUS_FAIL,
    SPI_STATUS_WRONG_PARAM,
    SPI_STATUS_TIMEOUT
} e_spi_status;

typedef struct {
    uint32_t unique_id_high;
    uint32_t unique_id_mid;
    uint32_t unique_id_low;
    char version[10]; /* format is V00.00.00\0 */
} s_ping_info;

typedef struct {
    uint32_t system_time_ms;
    float temperature;
} s_status;

/* -------------------------------------------------------------------------- */
/* --- PUBLIC FUNCTIONS PROTOTYPES ------------------------------------------ */

/**
 *
*/
int mcu_ping(int fd, s_ping_info * info);

/**
 *
*/
int mcu_boot(int fd);

/**
 *
*/
int mcu_get_status(int fd, s_status * status);

/**
 *
*/
int mcu_gpio_write(int fd, uint8_t gpio_port, uint8_t gpio_id, uint8_t gpio_value);

/**
@brief Send a SX1302 read/write SPI request to the MCU
@param fd File descriptor of the device used to access the MCU
@param in_out_buf The buffer containing the multiple requests to be sent to the
SX1302 with the SPI header (r/w, target mux). This buffer will also contain the
SPI answer when the function exits.
@param buf_size The size of the given input/output buffer
@return 0 for SUCCESS, -1 for failure
*/
int mcu_spi_write(int fd, uint8_t * in_out_buf, size_t buf_size);

/**
 *
*/
int mcu_spi_store(uint8_t * in_out_buf, size_t buf_size);

/**
 *
*/
int mcu_spi_flush(int fd);

#endif

/* --- EOF ------------------------------------------------------------------ */
