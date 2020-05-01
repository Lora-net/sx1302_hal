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
#include <stdlib.h>     /* malloc free */
#include <unistd.h>     /* lseek, close */
#include <fcntl.h>      /* open */
#include <string.h>     /* memset */
#include <errno.h>      /* Error number definitions */
#include <termios.h>    /* POSIX terminal control definitions */

#include "loragw_usb.h"
#include "loragw_aux.h"

/* -------------------------------------------------------------------------- */
/* --- PRIVATE MACROS ------------------------------------------------------- */

#define ARRAY_SIZE(a) (sizeof(a) / sizeof((a)[0]))
#if DEBUG_COM == 1
    #define DEBUG_MSG(str)                fprintf(stderr, str)
    //#define DEBUG_PRINTF(fmt, args...)    fprintf(stderr,"%s:%d: "fmt, __FUNCTION__, __LINE__, args)
    #define DEBUG_PRINTF(fmt, args...)    fprintf(stderr, fmt, args)
    #define CHECK_NULL(a)                if(a==NULL){fprintf(stderr,"%s:%d: ERROR: NULL POINTER AS ARGUMENT\n", __FUNCTION__, __LINE__);return LGW_USB_ERROR;}
#else
    #define DEBUG_MSG(str)
    #define DEBUG_PRINTF(fmt, args...)
    #define CHECK_NULL(a)                if(a==NULL){return LGW_USB_ERROR;}
#endif

/* -------------------------------------------------------------------------- */
/* --- PRIVATE CONSTANTS ---------------------------------------------------- */

const char mcu_version_string[] = "00.01.00";

#define DEBUG_VERBOSE 1

#define HEADER_CMD_SIZE 4
#define WRITE_SIZE_MAX  (1024 + HEADER_CMD_SIZE + 5)
#define READ_SIZE_MAX   (1024 + HEADER_CMD_SIZE + 5)

/* -------------------------------------------------------------------------- */
/* --- PRIVATE TYPES -------------------------------------------------------- */

typedef enum
{
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
    ACK_PING__UNIQUE_ID_0,  ACK_PING__UNIQUE_ID_1,  ACK_PING__UNIQUE_ID_2,  ACK_PING__UNIQUE_ID_3,
    ACK_PING__UNIQUE_ID_4,  ACK_PING__UNIQUE_ID_5,  ACK_PING__UNIQUE_ID_6,  ACK_PING__UNIQUE_ID_7,
    ACK_PING__UNIQUE_ID_8,  ACK_PING__UNIQUE_ID_9,  ACK_PING__UNIQUE_ID_10, ACK_PING__UNIQUE_ID_11,
    ACK_PING__VERSION_0,    ACK_PING__VERSION_1,    ACK_PING__VERSION_2,    ACK_PING__VERSION_3,    ACK_PING__VERSION_4,
    ACK_PING__VERSION_5,    ACK_PING__VERSION_6,    ACK_PING__VERSION_7,    ACK_PING__VERSION_8,
    ACK_PING_SIZE,
} e_cmd_offset_ack_ping;

typedef enum
{
    REQ_WRITE_GPIO__PORT,
    REQ_WRITE_GPIO__PIN,
    REQ_WRITE_GPIO__STATE,
    REQ_WRITE_GPIO_SIZE
} e_cmd_offset_req_write_gpio;

typedef enum
{
    ACK_GPIO_WRITE__STATUS,
    ACK_GPIO_WRITE_SIZE
} e_cmd_offset_ack_gpio_write;

typedef struct {
    uint32_t unique_id_high;
    uint32_t unique_id_mid;
    uint32_t unique_id_low;
    char version[10]; /* format is V00.00.00\0 */
} s_ping_info;

/* -------------------------------------------------------------------------- */
/* --- PRIVATE VARIABLES  --------------------------------------------------- */

static uint8_t buf_req[WRITE_SIZE_MAX];
static uint8_t buf_ack[READ_SIZE_MAX];

/* -------------------------------------------------------------------------- */
/* --- PRIVATE FUNCTIONS DEFINITION ----------------------------------------- */

int set_interface_attribs_linux(int fd, int speed) {
    struct termios tty;

    memset(&tty, 0, sizeof tty);

    /* Get current attributes */
    if (tcgetattr(fd, &tty) != 0) {
        DEBUG_PRINTF("ERROR: tcgetattr failed with %d - %s", errno, strerror(errno));
        return LGW_USB_ERROR;
    }

    cfsetospeed(&tty, speed);
    cfsetispeed(&tty, speed);

    /* Control Modes */
    tty.c_cflag = (tty.c_cflag & ~CSIZE) | CS8; /* set 8-bit characters */
    tty.c_cflag |= CLOCAL;                      /* local connection, no modem control */
    tty.c_cflag |= CREAD;                       /* enable receiving characters */
    tty.c_cflag &= ~PARENB;                     /* no parity */
    tty.c_cflag &= ~CSTOPB;                     /* one stop bit */
    /* Input Modes */
    tty.c_iflag &= ~IGNBRK;
    tty.c_iflag &= ~(IXON | IXOFF | IXANY | ICRNL);
    /* Output Modes */
    tty.c_oflag &= ~IGNBRK;
    tty.c_oflag &= ~(IXON | IXOFF | IXANY | ICRNL);
    /* Local Modes */
    tty.c_lflag = 0;
    /* Settings for non-canonical mode */
    tty.c_cc[VMIN] = 0;                         /* non-blocking mode */
    tty.c_cc[VTIME] = 50;                       /* wait for (n * 0.1) seconds before returning */

    /* Set attributes */
    if (tcsetattr(fd, TCSANOW, &tty) != 0) {
        DEBUG_PRINTF("ERROR: tcsetattr failed with %d - %s", errno, strerror(errno));
        return LGW_USB_ERROR;
    }

    return LGW_USB_SUCCESS;
}

/* ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ */

/* configure serial interface to be read blocking or not*/
int set_blocking_linux(int fd, bool blocking) {
    struct termios tty;

    memset(&tty, 0, sizeof tty);

    /* Get current attributes */
    if (tcgetattr(fd, &tty) != 0) {
        DEBUG_PRINTF("ERROR: tcgetattr failed with %d - %s", errno, strerror(errno));
        return LGW_USB_ERROR;
    }

    tty.c_cc[VMIN] = (blocking == true) ? 1 : 0;    /* set blocking or non-blocking mode */
    tty.c_cc[VTIME] = 1;                            /* wait for (n * 0.1) seconds before returning */

    /* Set attributes */
    if (tcsetattr(fd, TCSANOW, &tty) != 0) {
        DEBUG_PRINTF("ERROR: tcsetattr failed with %d - %s", errno, strerror(errno));
        return LGW_USB_ERROR;
    }

    return LGW_USB_SUCCESS;
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
        case ORDER_ID__REQ_SPI:
            return "REQ_SPI";
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

int write_req(int fd, e_order_cmd cmd, uint16_t size, const uint8_t * payload) {
    uint8_t buf_w[HEADER_CMD_SIZE];
    int n;

    /* Write command header */
    buf_w[0] = rand() % 255;
    buf_w[1] = (uint8_t)(size >> 8); /* MSB */
    buf_w[2] = (uint8_t)(size >> 0); /* LSB */
    buf_w[3] = cmd;
    n = write(fd, buf_w, HEADER_CMD_SIZE);
    if (n < 0) {
        printf("ERROR: failed to write command header to com port\n");
        return -1;
    }

    /* Write command payload */
    if (size > 0) {
        if (payload == NULL) {
            printf("ERROR: invalid payload\n");
            return -1;
        }
        n = write(fd, payload, size);
        if (n < 0) {
            printf("ERROR: failed to write command payload to com port\n");
            return -1;
        }
    }

    DEBUG_PRINTF("\nINFO: write_req 0x%02X (%s) done, id:0x%02X\n", cmd, cmd_get_str(cmd), buf_w[0]);

#if DEBUG_VERBOSE
    int i;
    for (i = 0; i < 4; i++) {
        printf("%02X ", buf_w[i]);
    }
    for (i = 0; i < size; i++) {
        printf("%02X ", payload[i]);
    }
    printf("\n");
#endif

    return 0;
}

/* ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ */

int read_ack(int fd, uint8_t * buf, size_t buf_size) {
    int i, n;
    size_t size;
    int nb_read = 0;

    /* Read message header first */
    n = read(fd, &buf[0], (size_t)HEADER_CMD_SIZE);
    if (errno == EINTR) {
        printf("INFO: syscall was interrupted, continue...\n");
        return -1;
    } else if (n == -1) {
        perror("ERROR: Unable to read /dev/ttyACMx - ");
        return -1;
    } else {
        DEBUG_PRINTF("INFO: read %d bytes for header from gateway\n", n);
        nb_read += n;
    }

#if DEBUG_VERBOSE
    printf("read_ack(hdr):");
    /* debug print */
    for (i = 0; i < (int)(HEADER_CMD_SIZE); i++) {
        printf("%02X ", buf[i]);
    }
    printf("\n");
#endif

    /* Get remaining payload size (metadata + pkt payload) */
    size  = (size_t)buf[CMD_OFFSET__SIZE_MSB] << 8;
    size |= (size_t)buf[CMD_OFFSET__SIZE_LSB] << 0;
    if (((size_t)HEADER_CMD_SIZE + size) > buf_size) {
        printf("ERROR: not enough memory to store all data (%zd)\n", (size_t)HEADER_CMD_SIZE + size);
        return -1;
    }

    /* Read payload if any */
    if (size > 0) {
        do {
            n = read(fd, &buf[nb_read], size - (nb_read - HEADER_CMD_SIZE));
            if (errno == EINTR) {
                printf("INFO: syscall was interrupted, continue...\n");
                return -1;
            } else if (n == -1) {
                perror("ERROR: Unable to read /dev/ttyACMx - ");
                return -1;
            } else {
                DEBUG_PRINTF("INFO: read %d bytes from gateway\n", n);
                nb_read += n;
            }
        } while ((nb_read - HEADER_CMD_SIZE) < (int)size); /* we want to read only the expected payload, not more */

#if DEBUG_VERBOSE
        /* debug print */
        printf("read_ack(pld):");
        for (i = HEADER_CMD_SIZE; i < (int)(HEADER_CMD_SIZE + size); i++) {
            printf("%02X ", buf[i]);
        }
        printf("\n");
#endif
    }

    return nb_read;
}

/* ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ */

int decode_ack_ping(const uint8_t * payload, s_ping_info * info) {
    /* sanity checks */
    if ((payload == NULL) || (info == NULL)) {
        printf("ERROR: invalid parameter\n");
        return -1;
    }

    if (cmd_get_type(payload) != ORDER_ID__ACK_PING) {
        printf("ERROR: wrong ACK type for PING (expected:0x%02X, got 0x%02X)\n", ORDER_ID__ACK_PING, cmd_get_type(payload));
        return -1;
    }

    /* payload info */
    info->unique_id_high = bytes_be_to_uint32_le(&payload[HEADER_CMD_SIZE + ACK_PING__UNIQUE_ID_0]);
    info->unique_id_mid  = bytes_be_to_uint32_le(&payload[HEADER_CMD_SIZE + ACK_PING__UNIQUE_ID_4]);
    info->unique_id_low  = bytes_be_to_uint32_le(&payload[HEADER_CMD_SIZE + ACK_PING__UNIQUE_ID_8]);

    memcpy(info->version, &payload[HEADER_CMD_SIZE + ACK_PING__VERSION_0], (sizeof info->version) - 1);
    info->version[(sizeof info->version) - 1] = '\0'; /* terminate string */

#if DEBUG_VERBOSE
    DEBUG_MSG   ("## ACK_PING\n");
    DEBUG_PRINTF("   id:           0x%02X\n", cmd_get_id(payload));
    DEBUG_PRINTF("   size:         %u\n", cmd_get_size(payload));
    DEBUG_PRINTF("   unique_id:    0x%08X%08X%08X\n", info->unique_id_high, info->unique_id_mid, info->unique_id_low);
    DEBUG_PRINTF("   FW version:   %s\n", info->version);
#endif

    return 0;
}

/* ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ */

int decode_ack_gpio_access(const uint8_t * payload, uint8_t * write_status) {
    if (payload == NULL) {
        printf("ERROR: invalid parameter\n");
        return -1;
    }

    if (cmd_get_type(payload) != ORDER_ID__ACK_WRITE_GPIO) {
        printf("ERROR: wrong ACK type for WRITE_GPIO (expected:0x%02X, got 0x%02X)\n", ORDER_ID__ACK_WRITE_GPIO, cmd_get_type(payload));
        return -1;
    }

    /* payload info */
    *write_status = payload[HEADER_CMD_SIZE + ACK_GPIO_WRITE__STATUS];

#if DEBUG_VERBOSE
    DEBUG_MSG   ("## ACK_WRITE_GPIO\n");
    DEBUG_PRINTF("   id:           0x%02X\n", cmd_get_id(payload));
    DEBUG_PRINTF("   size:         %u\n", cmd_get_size(payload));
    DEBUG_PRINTF("   status:       %u\n", *write_status);
#endif

    return 0;
}

/* ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ */

int decode_ack_spi_access(const uint8_t * payload) {
    /* sanity checks */
    if (payload == NULL) {
        printf("ERROR: invalid parameter\n");
        return -1;
    }

    if (cmd_get_type(payload) != ORDER_ID__ACK_SPI) {
        printf("ERROR: wrong ACK type for ACK_SPI (expected:0x%02X, got 0x%02X)\n", ORDER_ID__ACK_SPI, cmd_get_type(payload));
        return -1;
    }
    
#if DEBUG_VERBOSE
    DEBUG_MSG   ("## ACK_SPI_ACCESS\n");
    DEBUG_PRINTF("   id:           0x%02X\n", cmd_get_id(payload));
    DEBUG_PRINTF("   size:         %u\n", cmd_get_size(payload));
#endif

    return 0;
}

/* ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ */

int mcu_ping(int fd, s_ping_info * info) {
    CHECK_NULL(info);

    if (write_req(fd, ORDER_ID__REQ_PING, 0, NULL) != 0) {
        printf("ERROR: failed to write PING request\n");
        return -1;
    }

    if (read_ack(fd, buf_ack, sizeof buf_ack) < 0) {
        printf("ERROR: failed to read PING ack\n");
        return -1;
    }

    if (decode_ack_ping(buf_ack, info) != 0) {
        printf("ERROR: invalid PING ack\n");
        return -1;
    }

    return 0;
}

/* ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ */

int mcu_gpio_write(int fd, uint8_t gpio_port, uint8_t gpio_id, uint8_t gpio_value) {
    uint8_t status;

    buf_req[REQ_WRITE_GPIO__PORT]   = gpio_port;
    buf_req[REQ_WRITE_GPIO__PIN]    = gpio_id;
    buf_req[REQ_WRITE_GPIO__STATE]  = gpio_value;
    if (write_req(fd, ORDER_ID__REQ_WRITE_GPIO, REQ_WRITE_GPIO_SIZE, buf_req) != 0) {
        printf("ERROR: failed to write REQ_WRITE_GPIO request\n");
        return -1;
    }

    if (read_ack(fd, buf_ack, sizeof buf_ack) < 0) {
        printf("ERROR: failed to read PING ack\n");
        return -1;
    }

    if (decode_ack_gpio_access(buf_ack, &status) != 0) {
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

int mcu_spi_access(int fd, uint8_t * out_buf, size_t buf_size, uint8_t * in_buf) {
    /* Check input parameters */
    CHECK_NULL(out_buf);
    CHECK_NULL(in_buf);

    if (write_req(fd, ORDER_ID__REQ_SPI, buf_size, out_buf) != 0) {
        printf("ERROR: failed to write REQ_SPI request\n");
        return -1;
    }

    if (read_ack(fd, buf_ack, sizeof buf_ack) < 0) {
        printf("ERROR: failed to read REQ_SPI ack\n");
        return -1;
    }

    if (decode_ack_spi_access(buf_ack) != 0) {
        printf("ERROR: invalid REQ_SPI ack\n");
        return -1;
    }

    /* ACK is correct, just return the actual payload */
    memcpy(in_buf, buf_ack + HEADER_CMD_SIZE, buf_size); /* in_buf must be allocated with same size as out_buf */

    return 0;
}

/* -------------------------------------------------------------------------- */
/* --- PUBLIC FUNCTIONS DEFINITION ------------------------------------------ */

int lgw_usb_open(const char * com_path, void **com_target_ptr) {
    int *usb_device = NULL;
    char portname[50];
    int x;
    int fd;
    s_ping_info gw_info;

    /*check input variables*/
    CHECK_NULL(com_target_ptr);

    usb_device = malloc(sizeof(int));
    if (usb_device == NULL) {
        DEBUG_MSG("ERROR : MALLOC FAIL\n");
        return LGW_USB_ERROR;
    }

    /* open tty port */
    sprintf(portname, "%s", com_path);
    fd = open(portname, O_RDWR | O_NOCTTY | O_SYNC);
    if (fd < 0) {
        printf("ERROR: failed to open COM port %s - %s\n", portname, strerror(errno));
    } else {
        x = set_interface_attribs_linux(fd, B115200);
        x |= set_blocking_linux(fd, true);
        if (x != 0) {
            printf("ERROR: failed to configure COM port %s\n", portname);
            free(usb_device);
            return LGW_USB_ERROR;
        }

        *usb_device = fd;
        *com_target_ptr = (void*)usb_device;

        /* Initialize pseudo-random generator for MCU request ID */
        srand(0);

        /* Check MCU version (ignore first char of the received version (release/debug) */
        if (mcu_ping(fd, &gw_info) != 0) {
            printf("ERROR: failed to ping the concentrator MCU\n");
            return LGW_USB_ERROR;
        }
        if (strncmp(gw_info.version + 1, mcu_version_string, sizeof mcu_version_string) != 0) {
            printf("ERROR: MCU version mismatch (expected:%s, got:%s)\n", mcu_version_string, gw_info.version);
            return -1;
        }
        printf("INFO: Concentrator MCU version is %s\n", gw_info.version);

        /* Reset SX1302 */
        x  = mcu_gpio_write(fd, 0, 1, 1); /*   set PA1 : POWER_EN*/
        x |= mcu_gpio_write(fd, 0, 2, 1); /*   set PA2 : SX1302_RESET active*/
        x |= mcu_gpio_write(fd, 0, 2, 0); /* unset PA2 : SX1302_RESET inactive*/
        if (x != 0) {
            printf("ERROR: failed to reset SX1302\n");
            free(usb_device);
            return LGW_USB_ERROR;
        }

        return LGW_USB_SUCCESS;
    }

    free(usb_device);
    return LGW_USB_ERROR;
}

/* ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ */

/* SPI release */
int lgw_usb_close(void *com_target) {
    int usb_device;
    int a;

    /* check input variables */
    CHECK_NULL(com_target);

    /* close file & deallocate file descriptor */
    usb_device = *(int *)com_target;
    a = close(usb_device);
    free(com_target);

    /* determine return code */
    if (a < 0) {
        DEBUG_MSG("ERROR: USB PORT FAILED TO CLOSE\n");
        return LGW_USB_ERROR;
    } else {
        DEBUG_MSG("Note: USB port closed\n");
        return LGW_USB_SUCCESS;
    }
}

/* ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ */

/* Simple write */
int lgw_usb_w(void *com_target, uint8_t spi_mux_target, uint16_t address, uint8_t data) {
    int usb_device;
    uint8_t command_size = 5;
    uint8_t out_buf[command_size];
    uint8_t in_buf[command_size];
    int a;

    /* check input variables */
    CHECK_NULL(com_target);

    usb_device = *(int *)com_target;

    /* prepare frame to be sent */
    out_buf[0] = 0;
    out_buf[1] = spi_mux_target;
    out_buf[2] = 0x80 | ((address >> 8) & 0x7F);
    out_buf[3] =        ((address >> 0) & 0xFF);
    out_buf[4] = data;
    a = mcu_spi_access(usb_device, out_buf, command_size, in_buf);

    /* determine return code */
    if (a != 0) {
        DEBUG_MSG("ERROR: USB WRITE FAILURE\n");
        return -1;
    } else {
        DEBUG_MSG("Note: USB write success\n");
        return 0;
    }
}

/* ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ */

/* Simple read */
int lgw_usb_r(void *com_target, uint8_t spi_mux_target, uint16_t address, uint8_t *data) {
    int usb_device;
    uint8_t command_size = 6;
    uint8_t out_buf[command_size];
    uint8_t in_buf[command_size];
    int a;

    /* check input variables */
    CHECK_NULL(com_target);
    CHECK_NULL(data);

    usb_device = *(int *)com_target;

    /* prepare frame to be sent */
    out_buf[0] = 0;
    out_buf[1] = spi_mux_target;
    out_buf[2] = 0x00 | ((address >> 8) & 0x7F);
    out_buf[3] =        ((address >> 0) & 0xFF);
    out_buf[4] = 0x00;
    out_buf[5] = 0x00;
    a = mcu_spi_access(usb_device, out_buf, command_size, in_buf);

    /* determine return code */
    if (a != 0) {
        DEBUG_MSG("ERROR: USB READ FAILURE\n");
        return -1;
    } else {
        DEBUG_MSG("Note: USB read success\n");
        *data = in_buf[command_size - 1]; /* the last byte contains the register value */
        return 0;
    }
}

/* ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ */

/* Burst (multiple-byte) write */
int lgw_usb_wb(void *com_target, uint8_t spi_mux_target, uint16_t address, const uint8_t *data, uint16_t size) {
    int usb_device;
    uint16_t command_size = size + 4;
    uint8_t in_buf[command_size];
    int i;
    int a;

    printf("%s\n", __FUNCTION__);

    /* check input parameters */
    CHECK_NULL(com_target);
    CHECK_NULL(data);

    usb_device = *(int *)com_target;

    /* prepare command byte */
    buf_req[0] = 0;
    buf_req[1] = spi_mux_target;
    buf_req[2] = 0x80 | ((address >> 8) & 0x7F);
    buf_req[3] =        ((address >> 0) & 0xFF);
    for (i = 0; i < size; i++) {
        buf_req[i + 4] = data[i];
    }
    a = mcu_spi_access(usb_device, buf_req, command_size, in_buf);

    /* determine return code */
    if (a != 0) {
        DEBUG_MSG("ERROR: USB WRITE BURST FAILURE\n");
        return -1;
    } else {
        DEBUG_MSG("Note: USB write burst success\n");
        return 0;
    }
}

/* ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ */

/* Burst (multiple-byte) read */
int lgw_usb_rb(void *com_target, uint8_t spi_mux_target, uint16_t address, uint8_t *data, uint16_t size) {
    int usb_device;
    uint16_t command_size = size + 5;
    uint8_t in_buf[command_size];
    int i;
    int a;

    printf("%s\n", __FUNCTION__);

    /* check input parameters */
    CHECK_NULL(com_target);
    CHECK_NULL(data);

    usb_device = *(int *)com_target;

    /* prepare command byte */
    buf_req[0] = 0;
    buf_req[1] = spi_mux_target;
    buf_req[2] = 0x00 | ((address >> 8) & 0x7F);
    buf_req[3] =        ((address >> 0) & 0xFF);
    buf_req[4] = 0x00;
    for (i = 0; i < size; i++) {
        buf_req[i + 5] = 0;
    }

    a = mcu_spi_access(usb_device, buf_req, command_size, in_buf);

    /* determine return code */
    if (a != 0) {
        DEBUG_MSG("ERROR: USB READ BURST FAILURE\n");
        return -1;
    } else {
        DEBUG_MSG("Note: USB read burst success\n");
        memcpy(data, in_buf + 5, size);
        return 0;
    }
}

/* --- EOF ------------------------------------------------------------------ */
