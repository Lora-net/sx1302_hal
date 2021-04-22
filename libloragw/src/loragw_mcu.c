/*
 / _____)             _              | |
( (____  _____ ____ _| |_ _____  ____| |__
 \____ \| ___ |    (_   _) ___ |/ ___)  _ \
 _____) ) ____| | | || |_| ____( (___| | | |
(______/|_____)_|_|_| \__)_____)\____)_| |_|
  (C)2020 Semtech

Description:
    Host specific functions to address the LoRa concentrator registers through
    a USB interface.
    Single-byte read/write and burst read/write.

License: Revised BSD License, see LICENSE.TXT file include in the project
*/


/* -------------------------------------------------------------------------- */
/* --- DEPENDANCIES --------------------------------------------------------- */

#include <stdint.h>     /* C99 types */
#include <stdbool.h>    /* bool type */
#include <stdio.h>      /* printf fprintf */
#include <stdlib.h>     /* rand */
#include <unistd.h>     /* lseek, close */
#include <string.h>     /* memset */
#include <errno.h>      /* Error number definitions */
#include <termios.h>    /* POSIX terminal control definitions */

#include "loragw_mcu.h"
#include "loragw_aux.h"

/* -------------------------------------------------------------------------- */
/* --- PRIVATE MACROS ------------------------------------------------------- */

#define ARRAY_SIZE(a) (sizeof(a) / sizeof((a)[0]))
#if DEBUG_MCU == 1
    #define DEBUG_MSG(str)                fprintf(stdout, str)
    #define DEBUG_PRINTF(fmt, args...)    fprintf(stdout, fmt, args)
    #define CHECK_NULL(a)                if(a==NULL){fprintf(stderr,"%s:%d: ERROR: NULL POINTER AS ARGUMENT\n", __FUNCTION__, __LINE__);return -1;}
#else
    #define DEBUG_MSG(str)
    #define DEBUG_PRINTF(fmt, args...)
    #define CHECK_NULL(a)                if(a==NULL){return -1;}
#endif

/* -------------------------------------------------------------------------- */
/* --- PRIVATE CONSTANTS ---------------------------------------------------- */

#if DEBUG_MCU == 1
#define DEBUG_VERBOSE 0
#endif

#define HEADER_CMD_SIZE 4

/* -------------------------------------------------------------------------- */
/* --- PRIVATE TYPES -------------------------------------------------------- */

typedef struct spi_req_bulk_s {
    uint16_t size;
    uint8_t nb_req;
    uint8_t buffer[LGW_USB_BURST_CHUNK];
} spi_req_bulk_t;

/* -------------------------------------------------------------------------- */
/* --- PRIVATE VARIABLES  --------------------------------------------------- */

static uint8_t buf_hdr[HEADER_CMD_SIZE];

static spi_req_bulk_t spi_bulk_buffer = {
    .size = 0,
    .nb_req = 0,
    .buffer = { 0 }
};

/* -------------------------------------------------------------------------- */
/* --- PRIVATE FUNCTIONS DEFINITION ----------------------------------------- */

int spi_req_bulk_insert(spi_req_bulk_t * bulk_buffer, uint8_t * req, uint16_t req_size) {
    /* Check input parameters */
    CHECK_NULL(bulk_buffer);
    CHECK_NULL(req);

    if (bulk_buffer->nb_req == 255) {
        printf("ERROR: cannot insert a new SPI request in bulk buffer - too many requests\n");
        return -1;
    }

    if ((bulk_buffer->size + req_size) > LGW_USB_BURST_CHUNK) {
        printf("ERROR: cannot insert a new SPI request in bulk buffer - buffer full\n");
        return -1;
    }

    /* Add a new request entry in storage buffer */
    memcpy(bulk_buffer->buffer + bulk_buffer->size, req, req_size);

    bulk_buffer->nb_req += 1;
    bulk_buffer->size += req_size;

    return 0;
}

/* ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ */

uint32_t bytes_be_to_uint32_le(const uint8_t * bytes) {
    uint32_t val = 0;

    if (bytes != NULL) {
        /* Big endian to Little Endian */
        val  = (uint32_t)(bytes[0] << 24);
        val |= (uint32_t)(bytes[1] << 16);
        val |= (uint32_t)(bytes[2] << 8);
        val |= (uint32_t)(bytes[3] << 0);
    }

    return val;
}

/* ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ */

int32_t bytes_be_to_int32_le(const uint8_t * bytes) {
    int32_t val = 0;

    if (bytes != NULL) {
        /* Big endian to Little Endian */
        val  = (int32_t)(bytes[0] << 24);
        val |= (int32_t)(bytes[1] << 16);
        val |= (int32_t)(bytes[2] << 8);
        val |= (int32_t)(bytes[3] << 0);
    }

    return val;
}

/* ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ */

const char * cmd_get_str(const uint8_t cmd) {
    switch (cmd) {
        case ORDER_ID__REQ_PING:
            return "REQ_PING";
        case ORDER_ID__REQ_GET_STATUS:
            return "REQ_GET_STATUS";
        case ORDER_ID__REQ_BOOTLOADER_MODE:
            return "REQ_BOOTLOADER_MODE";
        case ORDER_ID__REQ_RESET:
            return "REQ_RESET";
        case ORDER_ID__REQ_WRITE_GPIO:
            return "REQ_WRITE_GPIO";
        case ORDER_ID__REQ_MULTIPLE_SPI:
            return "REQ_MULTIPLE_SPI";
        default:
            return "UNKNOWN";
    }
}

/* ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ */

uint8_t cmd_get_id(const uint8_t * bytes) {
    return bytes[0];
}

/* ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ */

uint16_t cmd_get_size(const uint8_t * bytes) {
    return (uint16_t)(bytes[1] << 8) | bytes[2];
}

/* ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ */

uint8_t cmd_get_type(const uint8_t * bytes) {
    return bytes[3];
}

/* ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ */

const char * spi_status_get_str(const uint8_t status) {
    switch (status) {
        case SPI_STATUS_OK:
            return "SPI_STATUS_OK";
        case SPI_STATUS_FAIL:
            return "SPI_STATUS_FAIL";
        case SPI_STATUS_WRONG_PARAM:
            return "SPI_STATUS_WRONG_PARAM";
        case SPI_STATUS_TIMEOUT:
            return "SPI_STATUS_TIMEOUT";
        default:
            return "SPI_STATUS_UNKNOWN";
    }
}

/* ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ */

int write_req(int fd, order_id_t cmd, const uint8_t * payload, uint16_t payload_size ) {
    uint8_t buf_w[HEADER_CMD_SIZE];
    int n;
    /* performances variables */
    struct timeval tm;
    /* debug variables */
#if DEBUG_MCU == 1
    struct timeval write_tv;
#endif

    /* Record function start time */
    _meas_time_start(&tm);

    /* Check input params */
    if (payload_size > MAX_SIZE_COMMAND) {
        printf("ERROR: payload size exceeds maximum transfer size (req:%u, max:%d)\n", payload_size, MAX_SIZE_COMMAND);
        return -1;
    }

    /* Write command header */
    buf_w[0] = rand() % 255;
    buf_w[1] = (uint8_t)(payload_size >> 8); /* MSB */
    buf_w[2] = (uint8_t)(payload_size >> 0); /* LSB */
    buf_w[3] = cmd;
    n = write(fd, buf_w, HEADER_CMD_SIZE);
    if (n < 0) {
        printf("ERROR: failed to write command header to com port\n");
        return -1;
    }

    /* Write command payload */
    if (payload_size > 0) {
        if (payload == NULL) {
            printf("ERROR: invalid payload\n");
            return -1;
        }
        n = write(fd, payload, payload_size);
        if (n < 0) {
            printf("ERROR: failed to write command payload to com port\n");
            return -1;
        }
    }

#if DEBUG_MCU == 1
    gettimeofday(&write_tv, NULL);
#endif
    DEBUG_PRINTF("\nINFO: %ld.%ld: write_req 0x%02X (%s) done, id:0x%02X, size:%u\n", write_tv.tv_sec, write_tv.tv_usec, cmd, cmd_get_str(cmd), buf_w[0], payload_size);

#if DEBUG_VERBOSE
    int i;
    for (i = 0; i < 4; i++) {
        printf("%02X ", buf_w[i]);
    }
    for (i = 0; i < payload_size; i++) {
        printf("%02X ", payload[i]);
    }
    printf("\n");
#endif

    /* Compute time spent in this function */
    _meas_time_stop(5, tm, __FUNCTION__);

    return 0;
}

/* ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ */

int read_ack(int fd, uint8_t * hdr, uint8_t * buf, size_t buf_size) {
#if DEBUG_VERBOSE
    int i;
#endif
    int n;
    size_t size;
    int nb_read = 0;
    /* performances variables */
    struct timeval tm;
    /* debug variables */
#if DEBUG_MCU == 1
    struct timeval read_tv;
#endif

    /* Record function start time */
    _meas_time_start(&tm);

    /* Read message header first, handle EINTR as it is a blocking call */
    do {
        n = read(fd, &hdr[0], (size_t)HEADER_CMD_SIZE);
    } while (n == -1 && errno == EINTR);

    if (n == -1) {
        perror("ERROR: Unable to read /dev/ttyACMx - ");
        return -1;
    } else {
#if DEBUG_MCU == 1
        gettimeofday(&read_tv, NULL);
#endif
        DEBUG_PRINTF("INFO: %ld.%ld: read %d bytes for header from gateway\n", read_tv.tv_sec, read_tv.tv_usec, n);
    }

    /* Compute time spent in this function */
    _meas_time_stop(5, tm, "read_ack(hdr)");

#if DEBUG_VERBOSE
    printf("read_ack(hdr):");
    /* debug print */
    for (i = 0; i < (int)(HEADER_CMD_SIZE); i++) {
        printf("%02X ", hdr[i]);
    }
    printf("\n");
#endif

    /* Record function start time */
    _meas_time_start(&tm);

    /* Check if the command id is valid */
    if ((cmd_get_type(hdr) < 0x40) || (cmd_get_type(hdr) > 0x46)) {
        printf("ERROR: received wrong ACK type (0x%02X)\n", cmd_get_type(hdr));
        return -1;
    }

    /* Get remaining payload size (metadata + pkt payload) */
    size = (size_t)cmd_get_size(hdr);
    if (size > buf_size) {
        printf("ERROR: not enough memory to store all data (%zd)\n", size);
        return -1;
    }

    /* Read payload if any */
    if (size > 0) {
        do {
            /* handle EINTR as it is a blocking call */
            do {
                n = read(fd, &buf[nb_read], size - nb_read);
            } while (n == -1 && errno == EINTR);

            if (n == -1) {
                perror("ERROR: Unable to read /dev/ttyACMx - ");
                return -1;
            } else {
#if DEBUG_MCU == 1
                gettimeofday(&read_tv, NULL);
#endif
                DEBUG_PRINTF("INFO: %ld.%ld: read %d bytes from gateway\n", read_tv.tv_sec, read_tv.tv_usec, n);
                nb_read += n;
            }
        } while (nb_read < (int)size); /* we want to read only the expected payload, not more */

#if DEBUG_VERBOSE
        /* debug print */
        printf("read_ack(pld):");
        for (i = 0; i < (int)size; i++) {
            printf("%02X ", buf[i]);
        }
        printf("\n");
#endif
    }

    /* Compute time spent in this function */
    _meas_time_stop(5, tm, "read_ack(payload)");

    return nb_read;
}

/* ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ */

int decode_ack_ping(const uint8_t * hdr, const uint8_t * payload, s_ping_info * info) {
    /* sanity checks */
    if ((hdr == NULL) || (payload == NULL) || (info == NULL)) {
        printf("ERROR: invalid parameter\n");
        return -1;
    }

    if (cmd_get_type(hdr) != ORDER_ID__ACK_PING) {
        printf("ERROR: wrong ACK type for PING (expected:0x%02X, got 0x%02X)\n", ORDER_ID__ACK_PING, cmd_get_type(hdr));
        return -1;
    }

    /* payload info */
    info->unique_id_high = bytes_be_to_uint32_le(&payload[ACK_PING__UNIQUE_ID_0]);
    info->unique_id_mid  = bytes_be_to_uint32_le(&payload[ACK_PING__UNIQUE_ID_4]);
    info->unique_id_low  = bytes_be_to_uint32_le(&payload[ACK_PING__UNIQUE_ID_8]);

    memcpy(info->version, &payload[ACK_PING__VERSION_0], (sizeof info->version) - 1);
    info->version[(sizeof info->version) - 1] = '\0'; /* terminate string */

#if DEBUG_VERBOSE
    DEBUG_MSG   ("## ACK_PING\n");
    DEBUG_PRINTF("   id:           0x%02X\n", cmd_get_id(hdr));
    DEBUG_PRINTF("   size:         %u\n", cmd_get_size(hdr));
    DEBUG_PRINTF("   unique_id:    0x%08X%08X%08X\n", info->unique_id_high, info->unique_id_mid, info->unique_id_low);
    DEBUG_PRINTF("   FW version:   %s\n", info->version);
#endif

    return 0;
}

/* ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ */

int decode_ack_bootloader_mode(const uint8_t * hdr) {
     /* sanity checks */
    if (hdr == NULL) {
        printf("ERROR: invalid parameter\n");
        return -1;
    }

    if (cmd_get_type(hdr) != ORDER_ID__ACK_BOOTLOADER_MODE) {
        printf("ERROR: wrong ACK type for ACK_BOOTLOADER_MODE (expected:0x%02X, got 0x%02X)\n", ORDER_ID__ACK_BOOTLOADER_MODE, cmd_get_type(hdr));
        return -1;
    }

#if DEBUG_VERBOSE
    DEBUG_MSG   ("## ACK_BOOTLOADER_MODE\n");
    DEBUG_PRINTF("   id:           0x%02X\n", cmd_get_id(hdr));
    DEBUG_PRINTF("   size:         %u\n", cmd_get_size(hdr));
#endif

    return 0;
}

/* ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ */

int decode_ack_get_status(const uint8_t * hdr, const uint8_t * payload, s_status * status) {
    int16_t temperature_sensor;

    /* sanity checks */
    if ((payload == NULL) || (status == NULL)) {
        printf("ERROR: invalid parameter\n");
        return -1;
    }

    if (cmd_get_type(hdr) != ORDER_ID__ACK_GET_STATUS) {
        printf("ERROR: wrong ACK type for GET_STATUS (expected:0x%02X, got 0x%02X)\n", ORDER_ID__ACK_GET_STATUS, cmd_get_type(hdr));
        return -1;
    }

    /* payload info */
    status->system_time_ms = bytes_be_to_uint32_le(&payload[ACK_GET_STATUS__SYSTEM_TIME_31_24]);

    temperature_sensor = (int16_t)(payload[ACK_GET_STATUS__TEMPERATURE_15_8] << 8) |
                         (int16_t)(payload[ACK_GET_STATUS__TEMPERATURE_7_0]  << 0);
    status->temperature = (float)temperature_sensor / 100.0;


#if DEBUG_VERBOSE
    DEBUG_MSG   ("## ACK_GET_STATUS\n");
    DEBUG_PRINTF("   id:            0x%02X\n", cmd_get_id(hdr));
    DEBUG_PRINTF("   size:          %u\n", cmd_get_size(hdr));
    DEBUG_PRINTF("   sys_time:      %u\n", status->system_time_ms);
    DEBUG_PRINTF("   temperature:   %.1f\n", status->temperature);
#endif

    return 0;
}

/* ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ */

int decode_ack_gpio_access(const uint8_t * hdr, const uint8_t * payload, uint8_t * write_status) {
    if ((hdr == NULL) || (payload == NULL) || (write_status == NULL)) {
        printf("ERROR: invalid parameter\n");
        return -1;
    }

    if (cmd_get_type(hdr) != ORDER_ID__ACK_WRITE_GPIO) {
        printf("ERROR: wrong ACK type for WRITE_GPIO (expected:0x%02X, got 0x%02X)\n", ORDER_ID__ACK_WRITE_GPIO, cmd_get_type(hdr));
        return -1;
    }

    /* payload info */
    *write_status = payload[ACK_GPIO_WRITE__STATUS];

#if DEBUG_VERBOSE
    DEBUG_MSG   ("## ACK_WRITE_GPIO\n");
    DEBUG_PRINTF("   id:           0x%02X\n", cmd_get_id(hdr));
    DEBUG_PRINTF("   size:         %u\n", cmd_get_size(hdr));
    DEBUG_PRINTF("   status:       %u\n", *write_status);
#endif

    return 0;
}

/* ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ */

int decode_ack_spi_bulk(const uint8_t * hdr, const uint8_t * payload) {
    uint8_t req_id, req_type, req_status;
    uint16_t frame_size;
    int i;

    /* sanity checks */
    if ((hdr == NULL) || (payload == NULL)) {
        printf("ERROR: invalid parameter\n");
        return -1;
    }

    if (cmd_get_type(hdr) != ORDER_ID__ACK_MULTIPLE_SPI) {
        printf("ERROR: wrong ACK type for ACK_MULTIPLE_SPI (expected:0x%02X, got 0x%02X)\n", ORDER_ID__ACK_MULTIPLE_SPI, cmd_get_type(hdr));
        return -1;
    }

#if DEBUG_VERBOSE
    DEBUG_MSG   ("## ACK_SPI_BULK\n");
    DEBUG_PRINTF("   id:           0x%02X\n", cmd_get_id(hdr));
    DEBUG_PRINTF("   size:         %u\n", cmd_get_size(hdr));
#endif

    i = 0;
    while (i < cmd_get_size(hdr)) {
        /* parse the request */
        req_id      = payload[i + 0];
        req_type    = payload[i + 1];
        if (req_type != MCU_SPI_REQ_TYPE_READ_WRITE && req_type != MCU_SPI_REQ_TYPE_READ_MODIFY_WRITE) {
            printf("ERROR: %s: wrong type for SPI request %u (0x%02X)\n", __FUNCTION__, req_id, req_type);
            return -1;
        }
        req_status  = payload[i + 2];
        if (req_status != 0) {
            /* Exit if any of the requests failed */
            printf("ERROR: %s: SPI request %u failed with %u - %s\n", __FUNCTION__, req_id, req_status, spi_status_get_str(req_status));
            return -1;
        }
#if DEBUG_VERBOSE
        DEBUG_PRINTF("   ----- REQ_SPI %u -----\n", req_id);
        DEBUG_PRINTF("   type %s\n", (req_type == MCU_SPI_REQ_TYPE_READ_WRITE) ? "read/write" : "read-modify-write");
        DEBUG_PRINTF("   status %u\n", req_status);
#endif
        /* Move to the next REQ */
        if (req_type == MCU_SPI_REQ_TYPE_READ_WRITE) {
            frame_size = (uint16_t)(payload[i + 3] << 8) | (uint16_t)(payload[i + 4]);
#if DEBUG_VERBOSE
            int j;
            DEBUG_PRINTF("   RAW SPI frame (sz:%u): ", frame_size);
            for (j = 0; j < frame_size; j++) {
                DEBUG_PRINTF(" %02X", payload[i + 5 + j]);
            }
            DEBUG_MSG("\n");
#endif
            i += (5 + frame_size); /* REQ ACK metadata + SPI raw frame */
        } else {
#if DEBUG_VERBOSE
            DEBUG_PRINTF("   read value     0x%02X\n", payload[i + 3]);
            DEBUG_PRINTF("   modified value 0x%02X\n", payload[i + 4]);
#endif
            i += 5;
        }
    }

    return 0;
}

/* -------------------------------------------------------------------------- */
/* --- PUBLIC FUNCTIONS DEFINITION ------------------------------------------ */

int mcu_ping(int fd, s_ping_info * info) {
    uint8_t buf_ack[ACK_PING_SIZE];

    CHECK_NULL(info);

    if (write_req(fd, ORDER_ID__REQ_PING, NULL, 0) != 0) {
        printf("ERROR: failed to write PING request\n");
        return -1;
    }

    if (read_ack(fd, buf_hdr, buf_ack, sizeof buf_ack) < 0) {
        printf("ERROR: failed to read PING ack\n");
        return -1;
    }

    if (decode_ack_ping(buf_hdr, buf_ack, info) != 0) {
        printf("ERROR: invalid PING ack\n");
        return -1;
    }

    return 0;
}

/* ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ */

int mcu_boot(int fd) {
    if (write_req(fd, ORDER_ID__REQ_BOOTLOADER_MODE, NULL, 0) != 0) {
        printf("ERROR: failed to write BOOTLOADER_MODE request\n");
        return -1;
    }

    if (read_ack(fd, buf_hdr, NULL, 0) < 0) {
        printf("ERROR: failed to read BOOTLOADER_MODE ack\n");
        return -1;
    }

    if (decode_ack_bootloader_mode(buf_hdr) != 0) {
        printf("ERROR: invalid BOOTLOADER_MODE ack\n");
        return -1;
    }

    return 0;
}

/* ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ */

int mcu_get_status(int fd, s_status * status) {
    uint8_t buf_ack[ACK_GET_STATUS_SIZE];

    CHECK_NULL(status);

    if (write_req(fd, ORDER_ID__REQ_GET_STATUS, NULL, 0) != 0) {
        printf("ERROR: failed to write GET_STATUS request\n");
        return -1;
    }

    if (read_ack(fd, buf_hdr, buf_ack, sizeof buf_ack) < 0) {
        printf("ERROR: failed to read GET_STATUS ack\n");
        return -1;
    }

    if (decode_ack_get_status(buf_hdr, buf_ack, status) != 0) {
        printf("ERROR: invalid GET_STATUS ack\n");
        return -1;
    }

    return 0;
}

/* ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ */

int mcu_gpio_write(int fd, uint8_t gpio_port, uint8_t gpio_id, uint8_t gpio_value) {
    uint8_t status;
    uint8_t buf_req[REQ_WRITE_GPIO_SIZE];
    uint8_t buf_ack[ACK_GPIO_WRITE_SIZE];

    buf_req[REQ_WRITE_GPIO__PORT]   = gpio_port;
    buf_req[REQ_WRITE_GPIO__PIN]    = gpio_id;
    buf_req[REQ_WRITE_GPIO__STATE]  = gpio_value;
    if (write_req(fd, ORDER_ID__REQ_WRITE_GPIO, buf_req, REQ_WRITE_GPIO_SIZE) != 0) {
        printf("ERROR: failed to write REQ_WRITE_GPIO request\n");
        return -1;
    }

    if (read_ack(fd, buf_hdr, buf_ack, sizeof buf_ack) < 0) {
        printf("ERROR: failed to read PING ack\n");
        return -1;
    }

    if (decode_ack_gpio_access(buf_hdr, buf_ack, &status) != 0) {
        printf("ERROR: invalid REQ_WRITE_GPIO ack\n");
        return -1;
    }

    if (status != 0) {
        printf("ERROR: Failed to write GPIO (port:%u id:%u value:%u)\n", gpio_port, gpio_id, gpio_value);
        return -1;
    }

    return 0;
}

/* ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ */

int mcu_spi_write(int fd, uint8_t * in_out_buf, size_t buf_size) {
    /* Check input parameters */
    CHECK_NULL(in_out_buf);

    if (write_req(fd, ORDER_ID__REQ_MULTIPLE_SPI, in_out_buf, buf_size) != 0) {
        printf("ERROR: failed to write REQ_MULTIPLE_SPI request\n");
        return -1;
    }

    if (read_ack(fd, buf_hdr, in_out_buf, buf_size) < 0) {
        printf("ERROR: failed to read REQ_MULTIPLE_SPI ack\n");
        return -1;
    }

    if (decode_ack_spi_bulk(buf_hdr, in_out_buf) != 0) {
        printf("ERROR: invalid REQ_MULTIPLE_SPI ack\n");
        return -1;
    }

    return 0;
}

/* ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ */

int mcu_spi_store(uint8_t * in_out_buf, size_t buf_size) {
    CHECK_NULL(in_out_buf);

    return spi_req_bulk_insert(&spi_bulk_buffer, in_out_buf, buf_size);
}

/* ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ */

int mcu_spi_flush(int fd) {
    /* Write pending SPI requests to MCU */
    if (mcu_spi_write(fd, spi_bulk_buffer.buffer, spi_bulk_buffer.size) != 0) {
        printf("ERROR: %s: failed to write SPI requests to MCU\n", __FUNCTION__);
        return -1;
    }

    /* Reset bulk storage buffer */
    spi_bulk_buffer.nb_req = 0;
    spi_bulk_buffer.size = 0;

    return 0;
}

/* --- EOF ------------------------------------------------------------------ */
